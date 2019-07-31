/*
 * CONFIDENTIAL
 *
 * Copyright 2018 Saso Kiselkov. All rights reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property
 * of Saso Kiselkov. The intellectual and technical concepts contained
 * herein are proprietary to Saso Kiselkov and may be covered by U.S. and
 * Foreign Patents, patents in process, and are protected by trade secret
 * or copyright law. Dissemination of this information or reproduction of
 * this material is strictly forbidden unless prior written permission is
 * obtained from Saso Kiselkov.
 */

#include <stddef.h>

#include <acfutils/avl.h>
#include <acfutils/math.h>
#include <acfutils/list.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/worker.h>

#include "libelec.h"

#define	EXEC_INTVAL		50000	/* us */
#define	MAX_NETWORK_DEPTH	100	/* dimensionless */

struct elec_sys_s {
	worker_t	worker;
	mutex_t		lock;

	avl_tree_t	info2comp;

	list_t		comps;
	list_t		gens_batts;
};

typedef struct {
	elec_comp_t	*bus;
	double		prev_amps;
	double		chg_rel;
} elec_batt_t;

typedef struct {
	elec_comp_t	*bus;
	double		ctr_rpm;
	double		min_stab;
	double		max_stab;
	double		stab_factor;
} elec_gen_t;

typedef struct {
	elec_comp_t	*ac;
	elec_comp_t	*dc;
} elec_tru_t;

typedef struct {
	elec_comp_t	*bus;
} elec_load_t;

typedef struct {
	elec_comp_t	**comps;
	size_t		n_comps;
} elec_bus_t;

typedef struct {
	elec_comp_t	*sides[2];
	bool_t		set;
	double		temp;		/* relative 0.0 - 1.0 */
} elec_cb_t;

typedef struct {
	size_t		n_buses;
	elec_comp_t	**buses;
	bool_t		*state;
} elec_tie_t;

typedef struct {
	elec_comp_t	*sides[2];	/* bias: sides[0] -> sides[1] */
} elec_diode_t;

struct elec_comp_s {
	elec_sys_t		*sys;
	const elec_comp_info_t	*info;

	double			in_volts;
	double			out_volts;
	double			in_amps;
	double			out_amps;
	double			in_pwr;		/* Watts */
	double			out_pwr;	/* Watts */

	elec_comp_t		*src;
	elec_comp_t		*upstream;

	bool_t			seen;

	union {
		elec_batt_t	batt;
		elec_gen_t	gen;
		elec_tru_t	tru;
		elec_load_t	load;
		elec_bus_t	bus;
		elec_cb_t	cb;
		elec_tie_t	tie;
		elec_diode_t	diode;
	};

	list_node_t		comps_node;
	avl_node_t		info2comp_node;
};

static bool_t elec_sys_worker(void *userinfo);
static void comp_free(elec_comp_t *comp);
static void network_paint_src_comp(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth);
static void network_load_integrate_comp(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *comp, unsigned depth);

static int
info2comp_compar(const void *a, const void *b)
{
	const elec_comp_t *ca = a, *cb = b;
	if (ca->info < cb->info)
		return (-1);
	if (ca->info > cb->info)
		return (1);
	return (0);
}

static void
resolve_bus_links(elec_sys_t *sys, elec_comp_t *bus)
{
	ASSERT(sys != NULL);
	ASSERT(bus != NULL);
	ASSERT(bus->info != NULL);
	ASSERT3U(bus->info->type, ==, ELEC_BUS);

	for (int i = 0; bus->info->bus.comps[i] != NULL; i++)
		bus->bus.n_comps++;
	bus->bus.comps = safe_calloc(bus->bus.n_comps,
	    sizeof (*bus->bus.comps));

	for (int i = 0; bus->info->bus.comps[i] != NULL; i++) {
		elec_comp_t *comp;
		elec_comp_t srch = { .info = bus->info->bus.comps[i] };

		comp = avl_find(&sys->info2comp, &srch, NULL);
		ASSERT_MSG(comp != NULL, "Component for info %s not found "
		    "(referenced from %s)", srch.info->name, bus->info->name);
		bus->bus.comps[i] = comp;
		ASSERT(comp->info != NULL);
		switch (comp->info->type) {
		case ELEC_BATT:
			/* Batteries are DC-only devices! */
			ASSERT(!bus->info->bus.ac);
			comp->batt.bus = bus;
			break;
		case ELEC_GEN:
			/* Generators can be DC or AC */
			ASSERT3U(bus->info->bus.ac, ==, comp->info->gen.ac);
			comp->gen.bus = bus;
			break;
		case ELEC_TRU:
			if (comp->info->tru.ac == bus->info) {
				ASSERT(bus->info->bus.ac);
				comp->tru.ac = bus;
			} else {
				ASSERT3P(comp->info->tru.dc, ==, bus->info);
				ASSERT(!bus->info->bus.ac);
				comp->tru.dc = bus;
			}
			break;
		case ELEC_LOAD:
			ASSERT3U(bus->info->bus.ac, ==, comp->info->load.ac);
			comp->load.bus = bus;
			break;
		case ELEC_BUS:
			VERIFY_MSG(0, "Invalid link: cannot connect bus %s "
			    "directly to bus %s", bus->info->name,
			    comp->info->name);
			break;
		case ELEC_CB:
			if (comp->cb.sides[0] == NULL) {
				comp->cb.sides[0] = bus;
			} else {
				ASSERT_MSG(comp->cb.sides[1] == NULL,
				    "Too many connections to %s",
				    comp->info->name);
				comp->cb.sides[1] = bus;
			}
			break;
		case ELEC_TIE:
			comp->tie.n_buses++;
			comp->tie.buses = safe_realloc(comp->tie.buses,
			    comp->tie.n_buses * sizeof (*comp->tie.buses));
			comp->tie.buses[comp->tie.n_buses - 1] = bus;
			free(comp->tie.state);
			comp->tie.state = safe_calloc(comp->tie.n_buses,
			    sizeof (*comp->tie.state));
			break;
		case ELEC_DIODE:
			/*
			 * Diodes are DC-only devices. Don't attempt to build
			 * rectifiers, use a TRU for that!
			 */
			ASSERT(!bus->info->bus.ac);
			if (comp->info->diode.sides[0] == bus->info) {
				comp->diode.sides[0] = bus;
			} else {
				ASSERT3P(comp->info->diode.sides[1], ==,
				    bus->info);
				comp->diode.sides[1] = bus;
			}
			break;
		default:
			VERIFY(0);
		}
	}
}

static void
resolve_comp_links(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		if (comp->info->type == ELEC_BUS)
			resolve_bus_links(sys, comp);
	}
}

static void
check_comp_links(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

#define	CHECK_LINK(field) \
	VERIFY_MSG((field) != NULL, "Component %s is missing a network link", \
	    comp->info->name)
	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		switch (comp->info->type) {
		case ELEC_BATT:
			CHECK_LINK(comp->batt.bus);
			break;
		case ELEC_GEN:
			CHECK_LINK(comp->gen.bus);
			break;
		case ELEC_TRU:
			CHECK_LINK(comp->tru.ac);
			CHECK_LINK(comp->tru.dc);
			break;
		case ELEC_LOAD:
			CHECK_LINK(comp->load.bus);
			break;
		case ELEC_BUS:
			CHECK_LINK(comp->bus.comps);
			break;
		case ELEC_CB:
			CHECK_LINK(comp->cb.sides[0]);
			CHECK_LINK(comp->cb.sides[1]);
			break;
		case ELEC_TIE:
			CHECK_LINK(comp->tie.buses);
			break;
		case ELEC_DIODE:
			CHECK_LINK(comp->diode.sides[0]);
			CHECK_LINK(comp->diode.sides[1]);
			break;
		}
	}
#undef	CHECK_LINK
}

elec_sys_t *
libelec_new(const elec_comp_info_t *comp_infos, size_t num_infos)
{
	elec_sys_t *sys = safe_calloc(1, sizeof (*sys));

	ASSERT(comp_infos != NULL);
	ASSERT3U(num_infos, >, 0);

	mutex_init(&sys->lock);
	list_create(&sys->comps, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, comps_node));
	avl_create(&sys->info2comp, info2comp_compar, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, info2comp_node));

	for (size_t i = 0; i < num_infos; i++) {
		elec_comp_t *comp = safe_calloc(1, sizeof (*comp));
		avl_index_t where;

		comp->sys = sys;
		comp->info = &comp_infos[i];
		VERIFY_MSG(avl_find(&sys->info2comp, comp, &where) == NULL,
		    "Duplicate elec info usage %s", comp_infos[i].name);
		avl_insert(&sys->info2comp, comp, where);
		list_insert_tail(&sys->comps, comp);

		/* Validate info structure */
		switch (comp->info->type) {
		case ELEC_BATT:
			ASSERT3F(comp->info->batt.volts, >, 0);
			ASSERT3F(comp->info->batt.max_pwr, >, 0);
			ASSERT3F(comp->info->batt.capacity, >=, 0);
			comp->batt.chg_rel = 1.0;
			break;
		case ELEC_GEN:
			ASSERT3F(comp->info->gen.volts, >, 0);
			ASSERT3F(comp->info->gen.min_rpm, >, 0);
			ASSERT3F(comp->info->gen.max_rpm, >, 0);
			ASSERT3F(comp->info->gen.min_rpm, <,
			    comp->info->gen.max_rpm);
			ASSERT3F(comp->info->gen.max_pwr, >, 0);
			ASSERT(comp->info->gen.eff_curve != NULL);
			ASSERT(comp->info->gen.get_rpm != NULL);
			comp->gen.ctr_rpm = AVG(comp->info->gen.min_rpm,
			    comp->info->gen.max_rpm);
			comp->gen.max_stab =
			    comp->gen.ctr_rpm / comp->info->gen.min_rpm;
			comp->gen.min_stab =
			    comp->gen.ctr_rpm / comp->info->gen.max_rpm;
			break;
		case ELEC_TRU:
			ASSERT3F(comp->info->tru.in_volts, >, 0);
			ASSERT3F(comp->info->tru.out_volts, >, 0);
			ASSERT3F(comp->info->tru.max_pwr, >, 0);
			ASSERT(comp->info->tru.eff_curve != NULL);
			ASSERT(comp->info->tru.ac != NULL);
			ASSERT(comp->info->tru.dc != NULL);
			break;
		case ELEC_LOAD:
			/*
			 * Stabilized loads MUST declare a minimum voltage.
			 * Non-stabilized loads don't need to.
			 */
			ASSERT(comp->info->load.min_volts > 0 ||
			    !comp->info->load.stab);
			ASSERT(comp->info->load.get_load != NULL);
			break;
		case ELEC_BUS:
			ASSERT(comp->info->bus.comps != NULL);
			break;
		case ELEC_CB:
			comp->cb.set = B_TRUE;
			break;
		case ELEC_TIE:
			break;
		case ELEC_DIODE:
			ASSERT(comp->info->diode.sides[0] != NULL);
			ASSERT(comp->info->diode.sides[1] != NULL);
			break;
		}
	}

	/* Resolve component links */
	resolve_comp_links(sys);
	check_comp_links(sys);

#ifndef	LIBELEC_SLOW_DEBUG
	worker_init(&sys->worker, elec_sys_worker, EXEC_INTVAL, sys,
	    "elec_sys");
#else	/* !LIBELEC_SLOW_DEBUG */
	worker_init(&sys->worker, elec_sys_worker, 20 * EXEC_INTVAL, sys,
	    "elec_sys");
#endif	/* !LIBELEC_SLOW_DEBUG */

	return (sys);
}

void
libelec_destroy(elec_sys_t *sys)
{
	elec_comp_t *comp;
	void *cookie;

	ASSERT(sys != NULL);

	worker_fini(&sys->worker);

	cookie = NULL;
	while (avl_destroy_nodes(&sys->info2comp, &cookie) != NULL)
		;
	while ((comp = list_remove_head(&sys->comps)) != NULL)
		comp_free(comp);
	list_destroy(&sys->comps);
	mutex_destroy(&sys->lock);

	free(sys);
}

static const char*
comp_type2str(elec_comp_type_t type)
{
	switch (type) {
	case ELEC_BATT:
		return ("BATT");
	case ELEC_GEN:
		return ("GEN");
	case ELEC_TRU:
		return ("TRU");
	case ELEC_LOAD:
		return ("LOAD");
	case ELEC_BUS:
		return ("BUS");
	case ELEC_CB:
		return ("CB");
	case ELEC_TIE:
		return ("TIE");
	case ELEC_DIODE:
		return ("DIODE");
	default:
		VERIFY(0);
	}
}

static bool_t
bind_list_find(const elec_func_bind_t *binds, const char *name, void *ptr)
{
	void **pp = (void **)ptr;

	for (size_t i = 0; binds != NULL && binds[i].name != NULL; i++) {
		if (strcmp(binds[i].name, name) == 0) {
			*pp = binds[i].value;
			return (B_TRUE);
		}
	}
	return (B_FALSE);
}

static bool_t
add_info_link(elec_comp_info_t *info, elec_comp_info_t *info2,
    const char *slot_qual)
{
	switch (info->type) {
	case ELEC_TRU:
		if (slot_qual == NULL)
			return (B_FALSE);
		if (strcmp(slot_qual, "AC") == 0) {
			if (info->tru.ac != NULL)
				return (B_FALSE);
			info->tru.ac = info2;
		} else {
			if (info->tru.dc != NULL)
				return (B_FALSE);
			info->tru.dc = info2;
		}
		break;
	case ELEC_BUS: {
		size_t num;

		for (num = 0; info->bus.comps != NULL &&
		    info->bus.comps[num] != NULL; num++)
			;
		info->bus.comps = safe_realloc(info->bus.comps,
		    (num + 2) * sizeof (*info->bus.comps));
		info->bus.comps[num] = info2;
		info->bus.comps[num + 1] = NULL;
		break;
	}
	case ELEC_DIODE:
		if (slot_qual == NULL)
			return (B_FALSE);
		if (strcmp(slot_qual, "IN") == 0) {
			if (info->diode.sides[0] != NULL)
				return (B_FALSE);
			info->diode.sides[0] = info2;
		} else {
			if (info->diode.sides[1] != NULL)
				return (B_FALSE);
			info->diode.sides[1] = info2;
		}
		break;
	default:
		return(B_TRUE);
	}

	return (B_TRUE);
}

static elec_comp_info_t *
find_comp_info(elec_comp_info_t *infos, size_t num_comps, const char *name)
{
	for (unsigned i = 0; i < num_comps; i++) {
		if (infos[i].name != NULL && strcmp(infos[i].name, name) == 0)
			return (&infos[i]);
	}
	return (NULL);
}

elec_comp_info_t *
libelec_infos_parse(const char *filename, const elec_func_bind_t *binds,
    size_t *num_infos)
{
	FILE *fp;
	size_t comp_i = 0, num_comps = 0;
	elec_comp_info_t *infos;
	elec_comp_info_t *info = NULL;
	char *line = NULL;
	size_t linecap = 0;
	unsigned linenum = 0;

	ASSERT(filename != NULL);
	/* binds can be NULL */
	ASSERT(num_infos != NULL);

	fp = fopen(filename, "r");
	if (fp == NULL) {
		logMsg("Can't open electrical network file %s: %s",
		    filename, strerror(errno));
		return (NULL);
	}

	/* First count all components to know how many to allocate */
	while (parser_get_next_line(fp, &line, &linecap, &linenum) > 0) {
		if (strncmp(line, "BATT ", 5) == 0 ||
		    strncmp(line, "GEN ", 4) == 0 ||
		    strncmp(line, "TRU ", 4) == 0 ||
		    strncmp(line, "LOAD ", 5) == 0 ||
		    strncmp(line, "BUS ", 4) == 0 ||
		    strncmp(line, "CB ", 3) == 0 ||
		    strncmp(line, "TIE ", 4) == 0 ||
		    strncmp(line, "DIODE ", 6) == 0) {
			num_comps++;
		} else if (strncmp(line, "LOADCB ", 7) == 0) {
			num_comps += 2;
		}
	}
	rewind(fp);

	infos = safe_calloc(num_comps, sizeof (*infos));

	linenum = 0;
	while (parser_get_next_line(fp, &line, &linecap, &linenum) > 0) {
		char **comps;
		const char *cmd;
		size_t n_comps;

#define	INVALID_LINE_FOR_COMP_TYPE \
	do { \
		logMsg("%s:%d: invalid %s line for component of type %s", \
		    filename, linenum, cmd, comp_type2str(info->type)); \
		free_strlist(comps, n_comps); \
		goto errout; \
	} while (0)
#define	GET_USERPTR(field) \
	do { \
		if (!bind_list_find(binds, comps[2], &(field))) { \
			logMsg("%s:%d: USERPTR %s not found in bind list", \
			    filename, linenum, comps[2]); \
			free_strlist(comps, n_comps); \
			goto errout; \
		} \
	} while (0)
#define	CHECK_DUP_NAME(__name__) \
	do { \
		if (find_comp_info(infos, comp_i, (__name__)) != NULL) { \
			logMsg("%s:%d: duplicate component name %s", \
			    filename, linenum, (__name__)); \
			free_strlist(comps, n_comps); \
			goto errout; \
		} \
	} while (0)

		comps = strsplit(line, " ", B_TRUE, &n_comps);
		cmd = comps[0];
		if (strcmp(cmd, "BATT") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_BATT;
			info->name = strdup(comps[1]);
		} else if (strcmp(cmd, "GEN") == 0 && n_comps == 3) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_GEN;
			info->name = strdup(comps[1]);
			info->gen.ac = (strcmp(comps[2], "AC") == 0);
		} else if (strcmp(cmd, "TRU") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_TRU;
			info->name = strdup(comps[1]);
		} else if (strcmp(cmd, "LOAD") == 0 &&
		    (n_comps == 2 || n_comps == 3)) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_LOAD;
			info->name = strdup(comps[1]);
			if (n_comps == 3)
				info->load.ac = (strcmp(comps[2], "AC") == 0);
			bind_list_find(binds, "get_load", &info->load.get_load);
		} else if (strcmp(cmd, "BUS") == 0 && n_comps == 3) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_BUS;
			info->name = strdup(comps[1]);
			info->bus.ac = (strcmp(comps[2], "AC") == 0);
		} else if (strcmp(cmd, "CB") == 0 && n_comps == 3) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_CB;
			info->name = strdup(comps[1]);
			info->cb.rate = 1;
			info->cb.max_amps = atof(comps[2]);
		} else if (strcmp(cmd, "TIE") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_TIE;
			info->name = strdup(comps[1]);
		} else if (strcmp(cmd, "DIODE") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_DIODE;
			info->name = strdup(comps[1]);
		} else if (strcmp(cmd, "VOLTS") == 0 && n_comps == 2 &&
		    info != NULL) {
			if (info->type == ELEC_BATT)
				info->batt.volts = atof(comps[1]);
			else if (info->type == ELEC_GEN)
				info->gen.volts = atof(comps[1]);
			else
				INVALID_LINE_FOR_COMP_TYPE;
		} else if (strcmp(cmd, "IN_VOLTS") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_TRU) {
			info->tru.in_volts = atof(comps[1]);
		} else if (strcmp(cmd, "OUT_VOLTS") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_TRU) {
			info->tru.out_volts = atof(comps[1]);
		} else if (strcmp(cmd, "MIN_VOLTS") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_LOAD) {
			info->load.min_volts = atof(comps[1]);
		} else if (strcmp(cmd, "INPUT_CAP") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_LOAD) {
			info->load.input_cap = atof(comps[1]);
		} else if (strcmp(cmd, "CAPACITY") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_BATT) {
			info->batt.capacity = atof(comps[1]);
		} else if (strcmp(cmd, "STAB") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_LOAD) {
			info->load.stab = (strcmp(comps[1], "TRUE") == 0);
		} else if (strcmp(cmd, "STAB_RATE") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			info->gen.stab_rate = atof(comps[1]);
		} else if (strcmp(cmd, "MIN_RPM") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			info->gen.min_rpm = atof(comps[1]);
		} else if (strcmp(cmd, "MAX_RPM") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			info->gen.max_rpm = atof(comps[1]);
		} else if (strcmp(cmd, "RATE") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_CB) {
			info->cb.rate = clamp(atof(comps[1]), 0.001, 1000);
		} else if (strcmp(cmd, "MAX_PWR") == 0 && n_comps == 2 &&
		    info != NULL) {
			if (info->type == ELEC_BATT)
				info->batt.max_pwr = atof(comps[1]);
			else if (info->type == ELEC_GEN)
				info->gen.max_pwr = atof(comps[1]);
			else if (info->type == ELEC_TRU)
				info->tru.max_pwr = atof(comps[1]);
			else
				INVALID_LINE_FOR_COMP_TYPE;
		} else if (strcmp(cmd, "CURVEPT") == 0 && n_comps == 4 &&
		    info != NULL) {
			vect2_t **curve_pp;
			size_t num = 0;

			if (info->type == ELEC_GEN)
				curve_pp = &info->gen.eff_curve;
			else if (info->type == ELEC_TRU)
				curve_pp = &info->tru.eff_curve;
			else
				INVALID_LINE_FOR_COMP_TYPE;
			if (*curve_pp != NULL) {
				for (vect2_t *curve = *curve_pp;
				    !IS_NULL_VECT(*curve); curve++)
					num++;
			} else {
				num = 0;
			}
			*curve_pp = safe_realloc(*curve_pp,
			    (num + 2) * sizeof (vect2_t));
			(*curve_pp)[num] =
			    VECT2(atof(comps[2]), atof(comps[3]));
			(*curve_pp)[num + 1] = NULL_VECT2;
		} else if (strcmp(cmd, "USERPTR") == 0 && n_comps == 3 &&
		    info != NULL) {
			if (info->type == ELEC_GEN) {
				if (strcmp(comps[1], "get_rpm") == 0) {
					GET_USERPTR(info->gen.get_rpm);
				} else if (strcmp(comps[1], "userinfo") == 0) {
					GET_USERPTR(info->gen.userinfo);
				} else {
					logMsg("%s:%d: invalid USERPTR type %s",
					    filename, linenum, comps[1]);
					free_strlist(comps, n_comps);
					goto errout;
				}
			} else if (info->type == ELEC_BATT) {
				if (strcmp(comps[1], "get_temp") == 0) {
					GET_USERPTR(info->batt.get_temp);
				} else if (strcmp(comps[1], "userinfo") == 0) {
					GET_USERPTR(info->batt.userinfo);
				} else {
					logMsg("%s:%d: invalid USERPTR type %s",
					    filename, linenum, comps[1]);
					free_strlist(comps, n_comps);
					goto errout;
				}
			} else if (info->type == ELEC_LOAD) {
				if (strcmp(comps[1], "get_load") == 0) {
					GET_USERPTR(info->load.get_load);
				} else if (strcmp(comps[1], "userinfo") == 0) {
					GET_USERPTR(info->load.userinfo);
				} else {
					logMsg("%s:%d: invalid USERPTR type %s",
					    filename, linenum, comps[1]);
					free_strlist(comps, n_comps);
					goto errout;
				}
			} else {
				INVALID_LINE_FOR_COMP_TYPE;
			}
		} else if (strcmp(cmd, "ENDPT") == 0 && info != NULL &&
		    info->type == ELEC_BUS && (n_comps == 2 || n_comps == 3)) {
			elec_comp_info_t *info2 =
			    find_comp_info(infos, num_comps, comps[1]);

			if (info2 == NULL) {
				logMsg("%s:%d: unknown component %s",
				    filename, linenum, comps[1]);
				free_strlist(comps, n_comps);
				goto errout;
			}
			if (!add_info_link(info, info2,
			    (n_comps == 3 ? comps[2] : NULL)) ||
			    !add_info_link(info2, info,
			    (n_comps == 3 ? comps[2] : NULL))) {
				logMsg("%s:%d: bad component link line",
				    filename, linenum);
				free_strlist(comps, n_comps);
				goto errout;
			}
		} else if (strcmp(cmd, "LOADCB") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_LOAD) {
			elec_comp_info_t *cb, *bus;

			ASSERT3U(comp_i + 1, <, num_comps);
			cb = &infos[comp_i++];
			cb->type = ELEC_CB;
			cb->name = sprintf_alloc("CB_%s", info->name);
			cb->cb.rate = 1;
			cb->cb.max_amps = atof(comps[1]);
			cb->autogen = B_TRUE;

			bus = &infos[comp_i++];
			bus->type = ELEC_BUS;
			bus->name = sprintf_alloc("CB_BUS_%s", info->name);
			bus->bus.ac = info->load.ac;
			bus->autogen = B_TRUE;

			VERIFY(add_info_link(bus, info, NULL));
			VERIFY(add_info_link(bus, cb, NULL));
		} else {
			logMsg("%s:%d: unknown or malformed line",
			    filename, linenum);
			free_strlist(comps, n_comps);
			goto errout;
		}
		free_strlist(comps, n_comps);
	}

#undef	INVALID_LINE_FOR_COMP_TYPE
#undef	GET_USERPTR
#undef	CHECK_DUP_NAME

	fclose(fp);
	free(line);
	*num_infos = num_comps;

	return (infos);
errout:
	fclose(fp);
	free(line);
	*num_infos = 0;

	return (NULL);
}

void
libelec_parsed_info_free(elec_comp_info_t *infos, size_t num_infos)
{
	ASSERT(infos != NULL || num_infos == 0);
	for (size_t i = 0; i < num_infos; i++) {
		elec_comp_info_t *info = &infos[i];

		free(info->name);
		if (info->type == ELEC_GEN)
			free(info->gen.eff_curve);
		else if (info->type == ELEC_TRU)
			free(info->tru.eff_curve);
		else if (info->type == ELEC_BUS)
			free(info->bus.comps);
	}
	free(infos);
}

void
libelec_walk_comps(elec_sys_t *sys, void (*cb)(elec_comp_t *, void*),
    void *userinfo)
{
	ASSERT(sys != NULL);
	ASSERT(cb != NULL);
	/* userinfo can be NULL */

	/*
	 * The component list is immutable, so we can walk it without
	 * holding the lock. But the callback might be changing system
	 * state, so we'll grab the lock before any manipulation can occur.
	 */
	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		mutex_enter(&sys->lock);
		cb(comp, userinfo);
		mutex_exit(&sys->lock);
	}
}

elec_comp_t *
libelec_info2comp(const elec_sys_t *sys, const elec_comp_info_t *info)
{
	ASSERT(info != NULL);

	/* The component list is immutable, so no locking is required */
	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		if (comp->info == info)
			return (comp);
	}

	return (NULL);
}

const elec_comp_info_t *
libelec_comp2info(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	/* immutable binding */
	return (comp->info);
}

elec_comp_t *
libelec_comp_get_src(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->src);
}

elec_comp_t *
libelec_comp_get_upstream(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->upstream);
}

size_t
libelec_comp_get_num_conns(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);

	switch (comp->info->type) {
	case ELEC_BUS:
		return (comp->bus.n_comps);
	case ELEC_TIE:
		return (comp->tie.n_buses);
	case ELEC_TRU:
	case ELEC_CB:
	case ELEC_DIODE:
		return (2);
	default:
		return (1);
	}
}

elec_comp_t *
libelec_comp_get_conn(const elec_comp_t *comp, size_t i)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);

	switch (comp->info->type) {
	case ELEC_BATT:
		ASSERT0(i);
		return (comp->batt.bus);
	case ELEC_GEN:
		ASSERT0(i);
		return (comp->gen.bus);
	case ELEC_TRU:
		ASSERT3U(i, <, 2);
		if (i == 0)
			return (comp->tru.ac);
		else
			return (comp->tru.dc);
	case ELEC_LOAD:
		ASSERT0(i);
		return (comp->load.bus);
	case ELEC_BUS:
		ASSERT3U(i, <, comp->bus.n_comps);
		return (comp->bus.comps[i]);
	case ELEC_CB:
		ASSERT3U(i, <, 2);
		return (comp->cb.sides[i]);
	case ELEC_TIE:
		ASSERT3U(i, <, comp->tie.n_buses);
		return (comp->tie.buses[i]);
	case ELEC_DIODE:
		ASSERT3U(i, <, 2);
		return (comp->cb.sides[i]);
	default:
		VERIFY(0);
	}
}

double
libelec_comp_get_in_volts(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->in_volts);
}

double
libelec_comp_get_out_volts(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->out_volts);
}

double
libelec_comp_get_in_amps(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->in_amps);
}

double
libelec_comp_get_out_amps(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->out_amps);
}

double
libelec_comp_get_in_pwr(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->in_pwr);
}

double
libelec_comp_get_out_pwr(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->out_pwr);
}

static void
network_reset(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		comp->upstream = NULL;
		comp->src = NULL;
		comp->in_volts = 0;
		comp->in_amps = 0;
		comp->out_volts = 0;
		comp->out_amps = 0;
	}
}

static void
network_update_gen(elec_comp_t *gen, double d_t)
{
	double rpm, stab_factor;

	ASSERT(gen != NULL);
	ASSERT(gen->info != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);
	ASSERT(gen->info->gen.get_rpm != NULL);

	rpm = gen->info->gen.get_rpm(gen);
	ASSERT(!isnan(rpm));
	rpm = MAX(rpm, 1);
	/*
	 * Gradual voltage stabilization adjustment, to simulate that
	 * the CSD takes a little time to adjust to rpm changes.
	 */
	stab_factor = clamp(gen->gen.ctr_rpm / rpm,
	    gen->gen.min_stab, gen->gen.max_stab);
	FILTER_IN(gen->gen.stab_factor, stab_factor, d_t,
	    gen->info->gen.stab_rate);

	gen->in_volts = (rpm / gen->gen.ctr_rpm) * gen->gen.stab_factor *
	    gen->info->gen.volts;
	gen->out_volts = gen->in_volts;
}

static void
network_update_batt(elec_comp_t *batt, double d_t)
{
	const vect2_t chg_volt_curve[] = {
	    VECT2(0.00, 0.00),
	    VECT2(0.04, 0.70),
	    VECT2(0.10, 0.80),
	    VECT2(0.20, 0.87),
	    VECT2(0.30, 0.91),
	    VECT2(0.45, 0.94),
	    VECT2(0.60, 0.95),
	    VECT2(0.80, 0.96),
	    VECT2(0.90, 0.97),
	    VECT2(1.00, 1.00),
	    NULL_VECT2
	};
	const vect2_t temp_energy_curve[] = {
	    VECT2(0, 0),
	    VECT2(C2KELVIN(-75), 0.0),
	    VECT2(C2KELVIN(-50), 0.25),
	    VECT2(C2KELVIN(15), 0.95),
	    VECT2(C2KELVIN(40), 1.0),
	    VECT2(C2KELVIN(50), 1.0),
	    NULL_VECT2
	};
	double U, J, T, temp_coeff, J_max, I_max, I_rel;

	ASSERT(batt != NULL);
	ASSERT(batt->info != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);
	ASSERT(batt->info->batt.get_temp != NULL);

	T = batt->info->batt.get_temp(batt);
	ASSERT3F(T, >, 0);
	temp_coeff = fx_lin_multi(T, temp_energy_curve, B_TRUE);

	I_max = batt->info->batt.max_pwr / batt->info->batt.volts;
	I_rel = batt->batt.prev_amps / I_max;
	U = batt->info->batt.volts * (1 - pow(I_rel, 1.45)) *
	    fx_lin_multi(batt->batt.chg_rel, chg_volt_curve, B_TRUE);

	J_max = batt->info->batt.capacity * temp_coeff;
	J = batt->batt.chg_rel * J_max;
	J -= U * batt->batt.prev_amps * d_t;

	/* Recalculate the new voltage and relative charge state */
	batt->in_volts = U;
	batt->out_volts = U;
	batt->batt.chg_rel = J / J_max;
}

static void
network_update_cb(elec_comp_t *cb, double d_t)
{
	double amps_rat;

	ASSERT(cb != NULL);
	ASSERT(cb->info != NULL);
	ASSERT3U(cb->info->type, ==, ELEC_CB);
	ASSERT3F(cb->info->cb.max_amps, >, 0);

	amps_rat = cb->out_amps / cb->info->cb.max_amps;
	FILTER_IN(cb->cb.temp, amps_rat, d_t, cb->info->cb.rate);

	if (cb->cb.temp >= 1.0)
		cb->cb.set = B_FALSE;
}

static void
network_srcs_update(elec_sys_t *sys, double d_t)
{
	ASSERT(sys != NULL);
	ASSERT3F(d_t, >, 0);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		if (comp->info->type == ELEC_GEN)
			network_update_gen(comp, d_t);
		else if (comp->info->type == ELEC_BATT)
			network_update_batt(comp, d_t);
	}
}

static void
network_loads_update(elec_sys_t *sys, double d_t)
{
	ASSERT(sys != NULL);
	ASSERT3F(d_t, >, 0);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		comp->in_pwr = comp->in_volts * comp->in_amps;
		comp->out_pwr = comp->out_volts * comp->out_amps;

		if (comp->info->type == ELEC_CB)
			network_update_cb(comp, d_t);
	}
}

static void
network_paint_src_bus(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_BUS);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (comp->in_volts < src->out_volts) {
		comp->src = src;
		comp->upstream = upstream;
		comp->in_volts = src->out_volts;

		for (unsigned i = 0; i < comp->bus.n_comps; i++) {
			if (upstream != comp->bus.comps[i]) {
				network_paint_src_comp(src, comp,
				    comp->bus.comps[i], depth + 1);
			}
		}
	}
}

static void
network_paint_src_tie(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	bool_t found = B_FALSE, tied = B_FALSE;

	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	/* Check if the upstream bus is currently tied */
	for (unsigned i = 0; i < comp->tie.n_buses; i++) {
		if (upstream == comp->tie.buses[i]) {
			if (comp->tie.state[i]) {
				if (comp->in_volts < src->out_volts) {
					comp->src = src;
					comp->upstream = upstream;
					comp->in_volts = src->out_volts;
				}
				tied = B_TRUE;
			}
			found = B_TRUE;
			break;
		}
	}
	ASSERT(found);
	if (tied) {
		for (unsigned i = 0; i < comp->tie.n_buses; i++) {
			if (upstream != comp->tie.buses[i] &&
			    comp->tie.state[i]) {
				network_paint_src_comp(src, comp,
				    comp->tie.buses[i], depth + 1);
			}
		}
	}
}

static void
network_paint_src_tru(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TRU);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	/* Output rectification prevents back-flow of power from DC to AC */
	if (upstream == comp->tru.ac && comp->in_volts < src->out_volts) {
		comp->src = src;
		comp->upstream = upstream;
		comp->in_volts = src->out_volts;
		comp->out_volts = comp->info->tru.out_volts *
		    (comp->in_volts / comp->info->tru.in_volts);
		ASSERT(comp->tru.dc != NULL);
		network_paint_src_comp(comp, comp, comp->tru.dc, depth + 1);
	}
}

static void
network_paint_src_cb(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (comp->cb.set && comp->in_volts < src->out_volts) {
		comp->src = src;
		comp->upstream = upstream;
		comp->in_volts = src->out_volts;
		if (upstream == comp->cb.sides[0]) {
			ASSERT(comp->cb.sides[1] != NULL);
			network_paint_src_comp(src, comp, comp->cb.sides[1],
			    depth + 1);
		} else {
			ASSERT3P(upstream, ==, comp->cb.sides[1]);
			ASSERT(comp->cb.sides[0] != NULL);
			network_paint_src_comp(src, comp, comp->cb.sides[0],
			    depth + 1);
		}
	}
}

static void
network_paint_src_diode(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_DIODE);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (upstream == comp->diode.sides[0] &&
	    comp->in_volts < src->out_volts) {
		comp->src = src;
		comp->upstream = upstream;
		comp->in_volts = src->out_volts;
		network_paint_src_comp(src, comp, comp->diode.sides[1],
		    depth + 1);
	}
}

static void
network_paint_src_comp(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(src->info != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	switch (comp->info->type) {
	case ELEC_BATT:
	case ELEC_GEN:
		break;
	case ELEC_BUS:
		if (src->info->type == ELEC_BATT || src->info->type == ELEC_TRU)
			ASSERT(!comp->info->bus.ac);
		else
			ASSERT3U(comp->info->gen.ac, ==, comp->info->bus.ac);
		network_paint_src_bus(src, upstream, comp, depth);
		break;
	case ELEC_TRU:
		network_paint_src_tru(src, upstream, comp, depth);
		break;
	case ELEC_LOAD:
		if (comp->in_volts < src->out_volts) {
			comp->src = src;
			comp->upstream = upstream;
			comp->in_volts = src->out_volts;
		}
		break;
	case ELEC_CB:
		network_paint_src_cb(src, upstream, comp, depth);
		break;
	case ELEC_TIE:
		network_paint_src_tie(src, upstream, comp, depth);
		break;
	case ELEC_DIODE:
		network_paint_src_diode(src, upstream, comp, depth);
		break;
	}
}

static void
network_paint_src(elec_sys_t *sys, elec_comp_t *comp)
{
	elec_comp_t *bus;

	ASSERT(sys != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT(comp->info->type == ELEC_BATT || comp->info->type == ELEC_GEN);

	if (comp->info->type == ELEC_BATT)
		bus = comp->batt.bus;
	else
		bus = comp->gen.bus;
	ASSERT(bus != NULL);
	network_paint_src_comp(comp, comp, bus, 0);
}

static void
network_paint(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		if (comp->info->type == ELEC_BATT ||
		    comp->info->type == ELEC_GEN)
			network_paint_src(sys, comp);
	}
}

static void
network_load_integrate_tru(elec_comp_t *comp, unsigned depth)
{
	double eff;

	ASSERT(comp != NULL);
	ASSERT(comp->tru.ac != NULL);
	ASSERT(comp->tru.dc != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TRU);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	/* When hopping over to the DC network, the TRU becomes the src */
	network_load_integrate_comp(comp, comp, comp->tru.dc, depth + 1);

	comp->out_amps = comp->tru.dc->in_amps;
	eff = fx_lin_multi(comp->out_amps, comp->info->tru.eff_curve, B_TRUE);
	ASSERT3F(eff, >, 0);
	comp->in_amps = ((comp->out_volts / comp->in_volts) * comp->out_amps) /
	    eff;
	ASSERT(comp->tru.ac != NULL);
}

static void
network_load_integrate_load(elec_comp_t *comp, unsigned depth)
{
	double load, volts;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_LOAD);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (comp->in_volts > comp->info->load.min_volts) {
		ASSERT(comp->info->load.get_load != NULL);
		load = comp->info->load.get_load(comp);
	} else {
		load = 0;
	}
	ASSERT3F(load, >=, 0);
	volts = MAX(comp->in_volts, comp->info->load.min_volts);
	/* load is in watts */
	if (comp->info->load.stab) {
		ASSERT3F(volts, >, 0);
		comp->in_amps = load / volts;
	} else {
		comp->in_amps = load;
	}
}

static void
network_load_integrate_bus(const elec_comp_t *src, elec_comp_t *comp,
    unsigned depth)
{
	double amps = 0;

	ASSERT(src != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_BUS);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	for (unsigned i = 0; i < comp->bus.n_comps; i++) {
		ASSERT(comp->bus.comps[i] != NULL);
		if (comp->bus.comps[i] != comp->upstream &&
		    comp->bus.comps[i]->src == src) {
			network_load_integrate_comp(src, comp,
			    comp->bus.comps[i], depth + 1);
			ASSERT(!isnan(comp->bus.comps[i]->in_amps));
			amps += comp->bus.comps[i]->in_amps;
		}
	}
	comp->out_amps = amps;
	comp->in_amps = amps;
}

static void
network_load_integrate_tie(const elec_comp_t *src, elec_comp_t *comp,
    unsigned depth)
{
	double amps = 0;

	ASSERT(src != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	for (unsigned i = 0; i < comp->tie.n_buses; i++) {
		ASSERT(comp->tie.buses[i] != NULL);
		if (comp->tie.state[i] &&
		    comp->tie.buses[i] != comp->upstream) {
			network_load_integrate_comp(src, comp,
			    comp->tie.buses[i], depth + 1);
			ASSERT(!isnan(comp->tie.buses[i]->in_amps));
			amps += comp->tie.buses[i]->in_amps;
		}
	}

	comp->out_amps = amps;
	comp->in_amps = amps;
}

static void
network_load_integrate_cb(const elec_comp_t *src, elec_comp_t *comp,
    unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (!comp->cb.set)
		return;
	if (comp->upstream == comp->cb.sides[0]) {
		network_load_integrate_comp(src, comp,
		    comp->cb.sides[1], depth + 1);
		comp->out_amps = comp->cb.sides[1]->in_amps;
	} else {
		ASSERT3P(comp->upstream, ==, comp->cb.sides[1]);
		network_load_integrate_comp(src, comp,
		    comp->cb.sides[0], depth + 1);
		comp->out_amps = comp->cb.sides[0]->in_amps;
	}
	ASSERT(!isnan(comp->in_amps));
	comp->in_amps = comp->out_amps;
}

static void
network_load_integrate_comp(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(src->info != NULL);
	ASSERT(src->info->type == ELEC_BATT || src->info->type == ELEC_GEN ||
	    src->info->type == ELEC_TRU);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (src == comp)
		ASSERT0(depth);
	if (comp != src && (comp->src != src || upstream != comp->upstream))
		return;

	switch (comp->info->type) {
	case ELEC_BATT:
		network_load_integrate_comp(src, comp, comp->batt.bus,
		    depth + 1);
		comp->out_amps = comp->batt.bus->in_amps;
		comp->batt.prev_amps = comp->out_amps;
		break;
	case ELEC_GEN:
		network_load_integrate_comp(src, comp, comp->gen.bus,
		    depth + 1);
		comp->out_amps = comp->gen.bus->in_amps;
		comp->in_volts = comp->out_volts;
		comp->in_amps = comp->out_amps /
		    fx_lin_multi(comp->in_volts * comp->out_amps,
		    comp->info->gen.eff_curve, B_TRUE);
		break;
	case ELEC_TRU:
		network_load_integrate_tru(comp, depth);
		break;
	case ELEC_LOAD:
		network_load_integrate_load(comp, depth);
		break;
	case ELEC_BUS:
		network_load_integrate_bus(src, comp, depth);
		break;
	case ELEC_CB:
		network_load_integrate_cb(src, comp, depth);
		break;
	case ELEC_TIE:
		network_load_integrate_tie(src, comp, depth);
		break;
	case ELEC_DIODE:
		network_load_integrate_comp(src, comp,
		    comp->diode.sides[1], depth + 1);
		comp->out_amps = comp->diode.sides[1]->in_amps;
		comp->in_amps = comp->out_amps;
		ASSERT(!isnan(comp->in_amps));
		break;
	}
}

static void
network_load_integrate(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		if (comp->info->type == ELEC_BATT ||
		    comp->info->type == ELEC_GEN) {
			network_load_integrate_comp(comp, comp, comp, 0);
		}
	}
}

static bool_t
elec_sys_worker(void *userinfo)
{
	elec_sys_t *sys;
	const double d_t = USEC2SEC(EXEC_INTVAL);

	ASSERT(userinfo != NULL);
	sys = userinfo;

	mutex_enter(&sys->lock);

	network_reset(sys);
	network_srcs_update(sys, d_t);
	network_paint(sys);
	network_load_integrate(sys);
	network_loads_update(sys, d_t);

	mutex_exit(&sys->lock);

	return (B_TRUE);
}

static void
comp_free(elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info);

	if (comp->info->type == ELEC_BUS) {
		free(comp->bus.comps);
	} else if (comp->info->type == ELEC_TIE) {
		free(comp->tie.buses);
		free(comp->tie.state);
	}

	free(comp);
}

void
libelec_cb_set(elec_comp_t *comp, bool_t set)
{
	elec_sys_t *sys;

	ASSERT(comp != NULL);
	ASSERT(comp->sys != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);

	sys = comp->sys;
	mutex_enter(&sys->lock);
	comp->cb.set = set;
	mutex_exit(&sys->lock);
}

bool_t
libelec_cb_get(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	/* atomic read, no need to lock */
	return (comp->cb.set);
}

double
libelec_cb_get_temp(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	/* atomic read, no need to lock */
	return (comp->cb.temp);
}

void
libelec_tie_set(elec_comp_t *comp, elec_comp_info_t *const* bus_list,
    size_t list_len)
{
	elec_sys_t *sys;
	bool_t *new_state;

	ASSERT(comp != NULL);
	ASSERT(comp->sys != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT(bus_list != NULL || list_len == 0);

	sys = comp->sys;
	if (list_len == 0) {
		mutex_enter(&sys->lock);
		memset(comp->tie.state, 0,
		    comp->tie.n_buses * sizeof (*comp->tie.state));
		mutex_exit(&sys->lock);
		return;
	}

	new_state = safe_calloc(comp->tie.n_buses, sizeof (*new_state));
	for (size_t i = 0; i < comp->tie.n_buses; i++) {
		for (size_t j = 0; j < list_len; j++) {
			if (comp->tie.buses[i]->info == bus_list[j]) {
				new_state[i] = B_TRUE;
				break;
			}
		}
	}
	mutex_enter(&sys->lock);
	memcpy(comp->tie.state, new_state,
	    comp->tie.n_buses * sizeof (*comp->tie.state));
	mutex_exit(&sys->lock);
	free(new_state);
}

void
libelec_tie_set_all(elec_comp_t *comp, bool_t tied)
{
	elec_sys_t *sys;

	ASSERT(comp != NULL);
	ASSERT(comp->sys != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	sys = comp->sys;
	mutex_enter(&sys->lock);
	for (unsigned i = 0; i < comp->tie.n_buses; i++)
		comp->tie.state[i] = tied;
	mutex_exit(&sys->lock);
}

bool_t
libelec_tie_get_all(const elec_comp_t *comp)
{
	elec_sys_t *sys;
	bool_t tied = B_TRUE;

	ASSERT(comp != NULL);
	ASSERT(comp->sys != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	sys = comp->sys;
	mutex_enter(&sys->lock);
	for (unsigned i = 0; tied && i < comp->tie.n_buses; i++)
		tied &= comp->tie.state[i];
	mutex_exit(&sys->lock);

	return (tied);
}

size_t
libelec_tie_get(const elec_comp_t *comp, elec_comp_t **bus_list)
{
	elec_sys_t *sys;
	size_t n_buses = 0;

	ASSERT(comp != NULL);
	ASSERT(comp->sys != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	sys = comp->sys;
	mutex_enter(&sys->lock);
	for (unsigned i = 0; i < comp->tie.n_buses; i++) {
		if (comp->tie.state[i])
			bus_list[n_buses++] = comp->tie.buses[i];
	}
	mutex_exit(&sys->lock);

	return (n_buses);
}
