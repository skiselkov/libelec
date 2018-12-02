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

#define	EXEC_INTVAL	50000	/* us */

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

	double			voltage;
	double			current;

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
			ASSERT3F(comp->info->gen.max_amps, >, 0);
			ASSERT(comp->info->gen.eff_curve != NULL);
			ASSERT(comp->info->gen.get_rpm != NULL);
			break;
		case ELEC_TRU:
			ASSERT3F(comp->info->tru.in_volts_min, >, 0);
			ASSERT3F(comp->info->tru.out_volts, >, 0);
			ASSERT3F(comp->info->tru.max_amps, >, 0);
			ASSERT(comp->info->tru.eff_curve != NULL);
			ASSERT(comp->info->tru.ac != NULL);
			ASSERT(comp->info->tru.dc != NULL);
			break;
		case ELEC_LOAD:
			ASSERT3F(comp->info->load.min_volt, >, 0);
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

static bool_t
elec_sys_worker(void *userinfo)
{
	elec_sys_t *sys;

	ASSERT(userinfo != NULL);
	sys = userinfo;

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
