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

#include <acfutils/list.h>
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
} elec_batt_t;

typedef struct {
	elec_comp_t	*bus;
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
} elec_cb_t;

typedef struct {
	size_t		n_buses;
	elec_comp_t	*buses;
	bool_t		*status;
} elec_tie_t;

typedef struct {
	elec_comp_t	sides[2];	/* bias: sides[0] -> sides[1] */
} elec_diode_t;

struct elec_comp_s {
	elec_sys_t		*sys;
	const elec_comp_info_t	*info;

	double			in_volts;
	double			out_volts;
	double			in_amps;
	double			out_amps;
	elec_comp_t		*src;
	elec_comp_t		*upstream;

	bool_t			seen;

	union {
		elec_batt_t	batt;
		zelec_gen_t	gen;
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
static void network_load_integrate_comp(elec_comp_t *comp, unsigned depth);

static int
info2comp_compar(const void *a, const void *b)
{
	const elec_comp_t *ca, = a, cb = b;
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
			comp->batt.bus = bus;
			break;
		case ELEC_GEN:
			comp->gen.bus = bus;
			break;
		case ELEC_TRU:
			if (comp->info->tru.ac == bus->info) {
				comp->tru.ac = bus;
			} else {
				ASSERT3P(comp->info->tru.dc, ==, bus->info);
				comp->tru.dc = bus;
			}
			break;
		case ELEC_LOAD:
			comp->load.bus = bus;
			break;
		case ELEC_BUS:
			VERIFY_MSG(0, "Invalid link: cannot connect bus %s "
			    "directly to bus %s", bus->info->name,
			    comp->info->name);
			break;
		case ELEC_CB:
			if (comp->info->cb.sides[0] == bus->info) {
				comp->cb.sides[0] = bus;
			} else {
				ASSERt3P(comp->info->cb.sides[1], ==,
				    bus->info);
				comp->cb.sides[1] = bus;
			}
			break;
		case ELEC_TIE:
			comp->tie.n_buses++;
			comp->tie.buses = realloc(comp->tie.n_buses *
			    sizeof (*comp->tie.buses));
			comp->tie.buses[comp->tie.n_buses - 1] = bus;
			free(comp->tie.status);
			comp->tie.status = safe_calloc(comp->tie.n_buses,
			    sizeof (*comp->tie.status));
			break;
		case ELEC_DIODE:
			if (comp->info->diode.sides[0] == bus->info) {
				comp->diode.sides[0] = bus;
			} else {
				ASSERT3P(comp->info->diode.sides[1], ==,
				    bus->info) {
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

elec_sys_t *
libelec_new(elec_comp_info_t **comp_infos)
{
	elec_sys_t *sys = safe_alloc(1, sizeof (*sys));

	ASSERT(comp_infos != NULL);

	mutex_init(&sys->lock);
	list_create(&sys->comps, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, comps_node));
	avl_create(&sys->info2comp, info2comp_compar, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, info2comp_node);

	for (int i = 0; comp_infos[i] != NULL; i++) {
		elec_comp_t *comp = safe_calloc(1, sizeof (*comp));
		avl_index_t where;

		comp->sys = sys;
		comp->info = comp_infos[i];
		VERIFY(avl_find(&sys->info2comp, comp, &where) == NULL,
		    "Duplicate elec info usage %s", comp_infos[i]->name);
		avl_insert(&sys->info2comp, comp, where);
		list_insert_tail(&sys->comps, comp);

		/* Validate info structure */
		switch (comp->info->type) {
		case ELEC_BATT:
			ASSERT3F(comp->info->batt.volts, >, 0);
			ASSERT3F(comp->info->batt.max_amps, >, 0);
			ASSERT3F(comp->info->batt.capacity, >=, 0);
			break;
		case ELEC_GEN:
			ASSERT3F(comp->info->gen.volts, >, 0);
			ASSERT3F(comp->info->gen.min_rpm, >, 0);
			ASSERT3F(comp->info->gen.max_rpm, >, 0);
			ASSERT3F(comp->info->gen.min_rpm, <,
			    comp->info->gen.max_rpm);
			ASSERT3F(comp->info->gen.max_amps, >, 0);
			ASSERT(comp->info->gen.eff_curve != NULL);
			ASSERT(comp->info->gen.get_rpm != NULL);
			comp->gen.ctr_rpm = AVG(comp->info->gen.min_rpm,
			    comp->info->gen.max_rpm);
			comp->gen.max_comp =
			    comp->gen.ctr_rpm / comp->gen.min_rpm;
			comp->gen.min_comp =
			    comp->gen.ctr_rpm / comp->gen.max_rpm;
			break;
		case ELEC_TRU:
			ASSERT3F(comp->info->tru.in_volts_min, >, 0);
			ASSERT3F(comp->info->tru.in_volts_max, >, 0);
			ASSERT3F(comp->info->tru.in_volts_min, >=,
			    comp->info->tru.in_volts_max);
			ASSERT3F(comp->info->tru.out_volts, >, 0);
			ASSERT3F(comp->info->tru.max_amps, >, 0);
			ASSERT(comp->info->tru.eff_curve != NULL);
			ASSERT(comp->info->tru.ac != NULL);
			ASSERT(comp->info->tru.dc != NULL);
			break;
		case ELEC_LOAD:
			ASSERT3F(comp->info->load.min_volts, >, 0);
			ASSERT(comp->info->load.get_load != NULL);
			break;
		case ELEC_BUS:
			ASSERT(comp->info->bus.comps != NULL);
			break;
		case ELEC_CB:
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

	worker_init(&sys->worker, elec_sys_worker, EXEC_INTVAL, sys,
	    "elec_sys");

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

const elec_comp_info_t *
libelec_comp_get_info(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->info);
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

static double
comp_update_gen(elec_comp_t *gen, double d_t)
{
	double rpm, stab_factor;

	ASSERT(gen != NULL);
	ASSERT(gen->info != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);
	ASSERT(gen->info->gen.get_rpm != NULL);

	rpm = gen->info->gen.get_rpm(comp);
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

	gen->out_volts = (rpm / gen->gen.ctr_rpm) * gen->gen.stab_factor *
	    gen->info->gen.volts;
}

static void
comps_update(elec_sys_t *sys, double d_t)
{
	ASSERT(sys != NULL);
	ASSERT3F(d_t, >, 0);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		if (comp->info->type == ELEC_GEN)
			comp_update_gen(comp, d_t);
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
			if (comp->tie.status[i]) {
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
			    comp->tie.status[i]) {
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

	if (comp->in_volts < src->out_volts) {
		comp->src = src;
		comp->upstream = upstream;
		comp->in_volts = src->out_volts;
		if (upstream == elec->cb.sides[0]) {
			network_paint_src_comp(src, comp, elec->cb.sides[1],
			    depth + 1);
		} else {
			ASSERT3P(upstream, ==, elec->cb.sides[1]);
			network_paint_src_comp(src, comp, elec->cb.sides[0],
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
		network_paint_src_comp(src, comp, elec->cb.sides[1], depth + 1);
	}
}

static void
network_paint_src_comp(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	switch (comp->info->type) {
	case ELEC_BATT:
	case ELEC_GEN:
		/* Electrical sources - don't paint these */
		break;
	case ELEC_BUS:
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
	network_paint_src_comp(sys, comp, comp, bus, 0);
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

	comp->out_amps = comp->tru.dc->in_amps;
	eff = fx_lin_multi(comp->out_amps, comp->info->tru.eff_curve, B_TRUE);
	ASSERT3F(eff, >, 0);
	comp->in_amps = ((comp->out_volts / comp->in_volts) * comp->out_amps) /
	    eff;
	ASSERT(comp->tru.ac != NULL);
	network_load_integrate_comp(comp, comp->tru.ac, depth + 1);
}

static void
network_load_integrate_load(elec_comp_t *comp, unsigned depth)
{
	double load, amps, volts;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_LOAD);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (comp->in_volts >= comp->info->load.min_volts) {
		ASSERT(comp->info->load.get_load != NULL);
		load = comp->info->load.get_load(comp);
	} else {
		load = 0;
	}
	ASSERT3F(load, >=, 0);
	volts = MAX(comp->in_volts, comp->info->load.min_volts);
	/* load is in watts */
	if (comp->info->load.stab)
		amps = load / volts;
	else
		amps = load;
	comp->in_amps = load;
	ASSERT(comp->bus != NULL);
	network_load_integrate_comp(comp, comp->bus, depth + 1);
}

static void
network_load_integrate_bus(elec_comp_t *comp, unsigned depth)
{
	double amps;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_BUS);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	for (unsigned i = 0; i < comp->bus.n_comps; i++) {
		ASSERT(comp->bus.comps[i] != NULL);
		if (comp->bus.comps[i] != comp->upstream)
			amps += comp->bus.comps[i]->in_amps;
	}

	comp->out_amps = amps;
	comp->in_amps = amps;
	network_load_integrate_comp(comp, comp->upstream, depth + 1);
}

static void
network_load_integrate_tie(elec_comp_t *comp, unsigned depth)
{
	double amps;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	for (unsigned i = 0; i < comp->tie.n_buses; i++) {
		ASSERT(comp->tie.buses[i] != NULL);
		if (comp->tie.status[i] && comp->tie.buses[i] != comp->upstream)
			amps += comp->tie.buses[i]->in_amps;
	}

	comp->out_amps = amps;
	comp->in_amps = amps;

	network_load_integrate_comp(comp, comp->upstream, depth + 1);
}

static void
network_load_integrate_comp(elec_comp_t *comp, unsigned depth)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (comp->upstream == NULL) {
		/* No power supplied to this component */
		ASSERT0(comp->in_volts);
		ASSERT0(comp->out_volts);
		ASSERT3P(comp->src, ==, NULL);
		return;
	}

	switch (comp->info->type) {
	case ELEC_BATT:
		comp->out_amps = comp->batt.bus->in_amps;
		break;
	case ELEC_GEN:
		comp->out_amps = comp->gen.bus->in_amps;
		break;
	case ELEC_TRU:
		network_load_integrate_tru(comp, depth);
		break;
	case ELEC_LOAD:
		network_load_integrate_load(comp, depth);
		break;
	case ELEC_BUS:
		network_load_integrate_bus(comp, depth);
		break;
	case ELEC_CB:
		if (comp->upstream == comp->cb.sides[0]) {
			comp->out_amps = comp->cb.sides[1]->in_amps;
		} else {
			ASSERT3P(comp->upstream, ==, comp->cb.sides[1]);
			comp->out_amps = comp->cb.sides[0]->in_amps;
		}
		comp->in_amps = comp->out_amps;
		network_load_integrate_comp(comp, comp->upstream, depth + 1);
		break;
	case ELEC_TIE:
		network_load_integrate_tie(comp, depth);
		break;
	case ELEC_DIODE:
		ASSERT3P(upstream, ==, comp->diode.sides[0]);
		ASSERT3P(downstream, ==, comp->diode.sides[1]);
		comp->out_amps = downstream->in_amps;
		comp->in_amps = comp->out_amps;
		network_load_integrate_comp(comp, comp->upstream, depth + 1);
		break;
	}

static void
network_load_integrate(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		network_load_integrate_comp(comp, comp, 0);
	}
}

static bool_t
elec_sys_worker(void *userinfo)
{
	elec_sys_t *sys;

	ASSERT(userinfo != NULL);
	sys = userinfo;

	mutex_enter(&sys->lock);

	network_reset(sys);
	network_update(sys, USEC2SEC(EXEC_INTVAL));
	network_paint(sys);
	network_load_integrate(sys);

	mutex_exit(&sys->lock);

	return (B_TRUE);
}

static void
comp_free(elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info);

	switch (comp->info->type) {
	case ELEC_BATT:
		break;
	case ELEC_BUS:
		free(comp->bus.comps);
		break;
	case ELEC_TIE:
		free(comp->tie.buses);
		break;
	}

	free(comp);
}
