/*
 * CONFIDENTIAL
 *
 * Copyright 2020 Saso Kiselkov. All rights reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property
 * of Saso Kiselkov. The intellectual and technical concepts contained
 * herein are proprietary to Saso Kiselkov and may be covered by U.S. and
 * Foreign Patents, patents in process, and are protected by trade secret
 * or copyright law. Dissemination of this information or reproduction of
 * this material is strictly forbidden unless prior written permission is
 * obtained from Saso Kiselkov.
 */

#include <stdarg.h>
#include <stddef.h>

#ifdef	XPLANE
#include <XPLMDisplay.h>
#endif

#include <acfutils/avl.h>
#include <acfutils/crc64.h>
#include <acfutils/math.h>
#include <acfutils/list.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/worker.h>

#ifdef	LIBELEC_WITH_DRS
#include <acfutils/dr.h>
#endif

#ifndef	LIBELEC_NO_LIBSWITCH
#include <libswitch.h>
#endif

#include <abus_ser.h>

#include "libelec.h"

#define	EXEC_INTVAL		50000	/* us */
#define	MAX_NETWORK_DEPTH	100	/* dimensionless */

struct elec_sys_s {
	bool		started;
	worker_t	worker;
	mutex_t		worker_interlock;

	mutex_t		paused_lock;
	bool		paused;		/* protected by paused_lock */
	double		time_factor;	/* only accessed from main thread */
#ifdef	XPLANE
	double		prev_sim_time;
struct {
		dr_t	paused;
		dr_t	replay;
		dr_t	sim_speed_act;
		dr_t	sim_time;
	} drs;
#endif	/* defined(XPLANE) */

	avl_tree_t	info2comp;
	avl_tree_t	name2comp;

	mutex_t		user_cbs_lock;
	avl_tree_t	user_cbs;

	list_t		comps;
	list_t		gens_batts;
};

typedef struct {
	elec_comp_t	*bus;
	SERIALIZE_START_MARKER;
	double		prev_amps;
	double		chg_rel;
	SERIALIZE_END_MARKER;
} elec_batt_t;

typedef struct {
	elec_comp_t	*bus;
	double		ctr_rpm;
	double		min_stab;
	double		max_stab;
	SERIALIZE_START_MARKER;
	double		stab_factor;
	SERIALIZE_END_MARKER;
} elec_gen_t;

typedef struct {
	elec_comp_t	*ac;
	elec_comp_t	*dc;
} elec_tru_t;

typedef struct {
	elec_comp_t	*bus;
	SERIALIZE_START_MARKER;
	/* Virtual input capacitor voltage */
	double		incap_U;
	SERIALIZE_END_MARKER;
	/* Change of input capacitor charge */
	double		incap_d_Q;

	bool		seen;
} elec_load_t;

typedef struct {
	elec_comp_t	**comps;
	size_t		n_comps;
} elec_bus_t;

typedef struct {
	elec_comp_t	*sides[2];
#ifndef	LIBELEC_NO_LIBSWITCH
	switch_t	*sw;		/* optional libswitch link */
#endif
	SERIALIZE_START_MARKER;
	bool_t		cur_set;
	bool_t		wk_set;
	double		temp;		/* relative 0.0 - 1.0 */
	SERIALIZE_END_MARKER;
} elec_cb_t;

typedef struct {
	size_t		n_buses;
	elec_comp_t	**buses;
	/*
	 * To prevent global locking from front-end functions, the tie
	 * state is kept split between `cur_state' and `wk_state':
	 *	cur_state - is read and adjusted from external functions
	 *	wk_state - is read-only from the worker thread
	 * At the start of the network worker loop, `cur_state' is
	 * transferred into `wk_state'. This guarantees a consistent tie
	 * state during a single worker invocation without having to hold
	 * the tie state lock throughout the worker loop.
	 */
	mutex_t		lock;
	bool		*cur_state;
	bool		*wk_state;
} elec_tie_t;

typedef struct {
	elec_comp_t	*sides[2];	/* bias: sides[0] -> sides[1] */
} elec_diode_t;

struct elec_comp_s {
	elec_sys_t		*sys;
	elec_comp_info_t	*info;

	/*
	 * Bitmask that gets checked by the network state integrator
	 * and reset in network_reset. This is used to prevent double-
	 * accounting of buses.
	 */
	uint64_t		integ_mask;

	SERIALIZE_START_MARKER;
	mutex_t			rw_ro_lock;
	struct {
		double		in_volts;
		double		out_volts;
		double		in_amps;
		double		out_amps;
		double		in_pwr;		/* Watts */
		double		out_pwr;	/* Watts */
		bool		failed;
		/*
		 * Shorted components leak a lot of their energy out
		 * to the environment and so we must avoid returning
		 * the leakage to in libelec_comp_get_xxx. Other
		 * parts of the system depend on those being the
		 * actual useful work being done by the component.
		 */
		bool		shorted;
		double		leak_factor;
	} rw, ro;
	SERIALIZE_END_MARKER;

	elec_comp_t		*src;
	elec_comp_t		*upstream;

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

#ifdef	LIBELEC_WITH_DRS
	struct {
		dr_t	in_volts;
		dr_t	out_volts;
		dr_t	in_amps;
		dr_t	out_amps;
		dr_t	in_pwr;
		dr_t	out_pwr;
	} drs;
#endif	/* defined(LIBELEC_WITH_DRS) */

	list_node_t		comps_node;
	avl_node_t		info2comp_node;
	avl_node_t		name2comp_node;
};

typedef struct {
	bool		pre;
	elec_user_cb_t	cb;
	void		*userinfo;
	avl_node_t	node;
} user_cb_info_t;

static bool_t elec_sys_worker(void *userinfo);
static void comp_free(elec_comp_t *comp);
static void network_paint_src_comp(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth);
static void network_load_integrate_comp(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *comp, unsigned depth,
    uint64_t src_mask, double d_t);
static void network_load_integrate_load(elec_comp_t *comp, unsigned depth,
    double d_t);

#ifdef	XPLANE
/*
 * When X-Plane is in a modal UI window (such as the weather setup
 * screen, or user settings) flight loop callbacks are never called.
 * But window callbacks are, so we piggyback onto the Window draw
 * callback to figure out if the sim is paused or not.
 */
static int
elec_draw_cb(XPLMDrawingPhase phase, int before, void *refcon)
{
	elec_sys_t *sys;
	double sim_time, time_factor;

	UNUSED(phase);
	UNUSED(before);
	ASSERT(refcon != NULL);
	sys = refcon;

	sim_time = dr_getf_prot(&sys->drs.sim_time);
	/* Caution: sim_speed_act likes to contain a NAN while paused */
	time_factor = round(dr_getf(&sys->drs.sim_speed_act) * 10) / 10;
	if (sys->prev_sim_time >= sim_time || dr_geti(&sys->drs.replay) != 0 ||
	    dr_geti(&sys->drs.paused) != 0 || time_factor == 0) {
		mutex_enter(&sys->paused_lock);
		sys->paused = true;
		mutex_exit(&sys->paused_lock);
		return (1);
	}
	sys->paused = false;
	/*
	 * If the time factor returns to 1.0x, we want to make sure we
	 * go back to 1.0x exactly, instead of some fractional 1.04x thing
	 * in case the sim was only ever so slightly accelerated/decelerated.
	 */
	if (time_factor != sys->time_factor ||
	    (time_factor == 1 && sys->time_factor != 1)) {
		sys->time_factor = time_factor;
		if (sys->started) {
			worker_set_interval_nowake(&sys->worker,
			    EXEC_INTVAL / time_factor);
		}
	}

	return (1);
}
#endif	/* defined(XPLANE) */

static int
user_cb_info_compar(const void *a, const void *b)
{
	const user_cb_info_t *ucbi_a = a, *ucbi_b = b;

	if (!ucbi_a->pre && ucbi_b->pre)
		return (-1);
	if (ucbi_a->pre && !ucbi_b->pre)
		return (1);
	if ((void *)ucbi_a->cb < (void *)ucbi_b->cb)
		return (-1);
	if ((void *)ucbi_a->cb > (void *)ucbi_b->cb)
		return (1);
	if (ucbi_a->userinfo < ucbi_b->userinfo)
		return (-1);
	if (ucbi_a->userinfo > ucbi_b->userinfo)
		return (1);
	return (0);
}

static int
name2comp_compar(const void *a, const void *b)
{
	const elec_comp_t *ca = a, *cb = b;
	int res = strcmp(ca->info->name, cb->info->name);
	if (res < 0)
		return (-1);
	if (res > 0)
		return (1);
	return (0);
}

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
			free(comp->tie.cur_state);
			comp->tie.cur_state = safe_calloc(comp->tie.n_buses,
			    sizeof (*comp->tie.cur_state));
			free(comp->tie.wk_state);
			comp->tie.wk_state = safe_calloc(comp->tie.n_buses,
			    sizeof (*comp->tie.wk_state));
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
libelec_new(elec_comp_info_t *comp_infos, size_t num_infos)
{
	elec_sys_t *sys = safe_calloc(1, sizeof (*sys));

	ASSERT(comp_infos != NULL);
	ASSERT3U(num_infos, >, 0);

	list_create(&sys->comps, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, comps_node));
	avl_create(&sys->info2comp, info2comp_compar, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, info2comp_node));
	avl_create(&sys->name2comp, name2comp_compar, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, name2comp_node));

	mutex_init(&sys->user_cbs_lock);
	avl_create(&sys->user_cbs, user_cb_info_compar,
	    sizeof (user_cb_info_t), offsetof(user_cb_info_t, node));
	mutex_init(&sys->worker_interlock);
	mutex_init(&sys->paused_lock);
	sys->time_factor = 1;

	for (size_t i = 0; i < num_infos; i++) {
		elec_comp_t *comp = safe_calloc(1, sizeof (*comp));
		avl_index_t where;
		elec_comp_info_t *info = &comp_infos[i];

		comp->sys = sys;
		comp->info = info;
		mutex_init(&comp->rw_ro_lock);
		VERIFY_MSG(avl_find(&sys->info2comp, comp, &where) == NULL,
		    "Duplicate elec info usage %s", info->name);
		avl_insert(&sys->info2comp, comp, where);
		list_insert_tail(&sys->comps, comp);

		VERIFY_MSG(avl_find(&sys->name2comp, comp, &where) ==
		    NULL, "Duplicate info name %s", info->name);
		avl_insert(&sys->name2comp, comp, where);

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
			ASSERT_MSG(comp->info->load.min_volts > 0 ||
			    !comp->info->load.stab, "Load %s must declare "
			    "a minimum voltage for a stabilized PSU",
			    comp->info->name);
			ASSERT3F(comp->info->load.incap_C, >=, 0);
			if (comp->info->load.incap_C > 0)
				ASSERT3F(comp->info->load.incap_R, >, 0);
			break;
		case ELEC_BUS:
			ASSERT(comp->info->bus.comps != NULL);
			break;
		case ELEC_CB:
			comp->cb.cur_set = comp->cb.wk_set = true;
			break;
		case ELEC_TIE:
			mutex_init(&comp->tie.lock);
			break;
		case ELEC_DIODE:
			ASSERT(comp->info->diode.sides[0] != NULL);
			ASSERT(comp->info->diode.sides[1] != NULL);
			break;
		}

#ifdef	LIBELEC_WITH_DRS
		dr_create_f64(&comp->drs.in_volts, &comp->ro.in_volts,
		    false, "libelec/comp/%s/in_volts", comp->info->name);
		dr_create_f64(&comp->drs.out_volts, &comp->ro.out_volts,
		    false, "libelec/comp/%s/out_volts", comp->info->name);
		dr_create_f64(&comp->drs.in_amps, &comp->ro.in_amps,
		    false, "libelec/comp/%s/in_amps", comp->info->name);
		dr_create_f64(&comp->drs.out_amps, &comp->ro.out_amps,
		    false, "libelec/comp/%s/out_amps", comp->info->name);
		dr_create_f64(&comp->drs.in_pwr, &comp->ro.in_pwr,
		    false, "libelec/comp/%s/in_pwr", comp->info->name);
		dr_create_f64(&comp->drs.out_pwr, &comp->ro.out_pwr,
		    false, "libelec/comp/%s/out_pwr", comp->info->name);
#endif	/* defined(LIBELEC_WITH_DRS) */
	}

	/* Resolve component links */
	resolve_comp_links(sys);
	check_comp_links(sys);

#ifdef	XPLANE
	fdr_find(&sys->drs.sim_speed_act, "sim/time/sim_speed_actual");
	fdr_find(&sys->drs.sim_time, "sim/time/total_running_time_sec");
	fdr_find(&sys->drs.paused, "sim/time/paused");
	fdr_find(&sys->drs.replay, "sim/time/is_in_replay");
	VERIFY(XPLMRegisterDrawCallback(elec_draw_cb, xplm_Phase_Window,
	    0, sys));
#endif	/* defined(XPLANE) */

	return (sys);
}

void
libelec_sys_start(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	if (!sys->started) {
#ifndef	LIBELEC_SLOW_DEBUG
		worker_init(&sys->worker, elec_sys_worker, EXEC_INTVAL, sys,
		    "elec_sys");
#else	/* !LIBELEC_SLOW_DEBUG */
		worker_init(&sys->worker, elec_sys_worker, 0, sys, "elec_sys");
#endif	/* !LIBELEC_SLOW_DEBUG */
		sys->started = true;
	}
}

void
libelec_sys_stop(elec_sys_t *sys)
{
	if (!sys->started)
		return;

	worker_fini(&sys->worker);
	sys->started = false;
}

static void
elec_comp_serialize(elec_comp_t *comp, conf_t *ser, const char *prefix)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT(comp->info->name != NULL);
	ASSERT(ser != NULL);
	ASSERT(prefix != NULL);

	SERIALIZE_DATA_V(comp, ser, "%s/%s/data",
	    prefix, comp->info->name);

	switch (comp->info->type) {
	case ELEC_BATT:
		SERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/batt",
		    prefix, comp->info->name);
		break;
	case ELEC_GEN:
		SERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/gen",
		    prefix, comp->info->name);
		break;
	case ELEC_LOAD:
		SERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/load",
		    prefix, comp->info->name);
		break;
	case ELEC_CB:
		SERIALIZE_DATA_V(&comp->cb, ser, "%s/%s/cb",
		    prefix, comp->info->name);
		break;
	case ELEC_TIE:
		mutex_enter(&comp->tie.lock);
		conf_set_data_v(ser, "%s/%s/cur_state", comp->tie.cur_state,
		    comp->tie.n_buses * sizeof (*comp->tie.cur_state),
		    prefix, comp->info->name);
		mutex_exit(&comp->tie.lock);
		break;
	default:
		break;
	}
}

static bool
elec_comp_deserialize(elec_comp_t *comp, const conf_t *ser, const char *prefix)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT(comp->info->name != NULL);
	ASSERT(ser != NULL);
	ASSERT(prefix != NULL);

	DESERIALIZE_DATA_V(comp, ser, "%s/%s/data",
	    prefix, comp->info->name);

	switch (comp->info->type) {
	case ELEC_BATT:
		DESERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/batt",
		    prefix, comp->info->name);
		break;
	case ELEC_GEN:
		DESERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/gen",
		    prefix, comp->info->name);
		break;
	case ELEC_LOAD:
		DESERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/load",
		    prefix, comp->info->name);
		break;
	case ELEC_CB:
		DESERIALIZE_DATA_V(&comp->cb, ser, "%s/%s/cb",
		    prefix, comp->info->name);
		break;
	case ELEC_TIE:
		mutex_enter(&comp->tie.lock);
		if (conf_get_data_v(ser, "%s/%s/cur_state", comp->tie.cur_state,
		    comp->tie.n_buses * sizeof (*comp->tie.cur_state),
		    prefix, comp->info->name) !=
		    comp->tie.n_buses * sizeof (*comp->tie.cur_state)) {
			mutex_exit(&comp->tie.lock);
			return (false);
		}
		mutex_exit(&comp->tie.lock);
		break;
	default:
		break;
	}

	return (true);
}

void
libelec_serialize(elec_sys_t *sys, conf_t *ser, const char *prefix)
{
	ASSERT(sys != NULL);
	ASSERT(ser != NULL);
	ASSERT(prefix != NULL);

	mutex_enter(&sys->worker_interlock);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		elec_comp_serialize(comp, ser, prefix);
	}

	mutex_exit(&sys->worker_interlock);
}

bool
libelec_deserialize(elec_sys_t *sys, const conf_t *ser, const char *prefix)
{
	ASSERT(sys != NULL);
	ASSERT(ser != NULL);
	ASSERT(prefix != NULL);

	mutex_enter(&sys->worker_interlock);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		if (!elec_comp_deserialize(comp, ser, prefix)) {
			logMsg("Failed to deserialize %s: malformed state",
			    comp->info->name);
			mutex_exit(&sys->worker_interlock);
			return (false);
		}
	}

	mutex_exit(&sys->worker_interlock);

	return (true);
}

void
libelec_destroy(elec_sys_t *sys)
{
	elec_comp_t *comp;
	user_cb_info_t *ucbi;
	void *cookie;

	ASSERT(sys != NULL);

	/* libelec_sys_stop MUST be called first! */
	ASSERT(!sys->started);

	cookie = NULL;
	while ((ucbi = avl_destroy_nodes(&sys->user_cbs, &cookie)) != NULL)
		free(ucbi);
	avl_destroy(&sys->user_cbs);
	mutex_destroy(&sys->user_cbs_lock);

	cookie = NULL;
	while (avl_destroy_nodes(&sys->info2comp, &cookie) != NULL)
		;
	avl_destroy(&sys->info2comp);

	cookie = NULL;
	while (avl_destroy_nodes(&sys->name2comp, &cookie) != NULL)
		;
	avl_destroy(&sys->info2comp);

	while ((comp = list_remove_head(&sys->comps)) != NULL)
		comp_free(comp);
	list_destroy(&sys->comps);

	mutex_destroy(&sys->worker_interlock);
	mutex_destroy(&sys->paused_lock);

#ifdef	XPLANE
	VERIFY(XPLMUnregisterDrawCallback(elec_draw_cb, xplm_Phase_Window,
	    0, sys));
#endif

	memset(sys, 0, sizeof (*sys));
	free(sys);
}

#ifndef	LIBELEC_NO_LIBSWITCH

void
libelec_create_cb_switches(const elec_sys_t *sys, const char *prefix,
    float anim_rate)
{
	ASSERT(sys != NULL);
	ASSERT(prefix != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		if (comp->info->type == ELEC_CB) {
			char name[128], desc[128];

			VERIFY3S(snprintf(name, sizeof (name), "%s%s",
			    prefix, comp->info->name), <, sizeof (name));
			VERIFY3S(snprintf(desc, sizeof (desc),
			    "Circuit breaker %s", comp->info->name), <,
			    sizeof (desc));
			comp->cb.sw = libswitch_add_toggle(name, desc,
			    anim_rate);
			/* Invert the CB so '0' is popped and '1' is pushed */
			libswitch_set_anim_offset(comp->cb.sw, -1, 1);
			libswitch_set(comp->cb.sw, 0);
		}
	}
}

#endif	/* LIBELEC_NO_LIBSWITCH */

#ifdef	LIBELEC_SLOW_DEBUG

void
libelec_step(elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	worker_wake_up(&sys->worker);
}

#endif

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

static bool
bind_list_find(const elec_func_bind_t *binds, const char *name, void *ptr)
{
	void **pp = (void **)ptr;

	for (size_t i = 0; binds != NULL && binds[i].name != NULL; i++) {
		if (strcmp(binds[i].name, name) == 0) {
			*pp = binds[i].value;
			return (true);
		}
	}
	return (false);
}

static bool
add_info_link(elec_comp_info_t *info, elec_comp_info_t *info2,
    const char *slot_qual)
{
	switch (info->type) {
	case ELEC_TRU:
		if (slot_qual == NULL)
			return (false);
		if (strcmp(slot_qual, "AC") == 0) {
			if (info->tru.ac != NULL)
				return (false);
			info->tru.ac = info2;
		} else {
			if (info->tru.dc != NULL)
				return (false);
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
			return (false);
		if (strcmp(slot_qual, "IN") == 0) {
			if (info->diode.sides[0] != NULL)
				return (false);
			info->diode.sides[0] = info2;
		} else {
			if (info->diode.sides[1] != NULL)
				return (false);
			info->diode.sides[1] = info2;
		}
		break;
	default:
		return(true);
	}

	return (true);
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
#define	MAX_BUS_UNIQ	256
	uint64_t bus_IDs_seen[256] = { 0 };
	unsigned bus_ID_cur = 0;
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

		comps = strsplit(line, " ", true, &n_comps);
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
			memset(bus_IDs_seen, 0, sizeof (bus_IDs_seen));
			bus_ID_cur = 0;
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
		} else if (strcmp(cmd, "INCAP") == 0 &&
		    (n_comps == 3 || n_comps == 4) &&
		    info != NULL && info->type == ELEC_LOAD) {
			info->load.incap_C = atof(comps[1]);
			info->load.incap_R = atof(comps[2]);
			if (n_comps == 4) {
				info->load.incap_leak_Qps = atof(comps[3]);
			} else {
				info->load.incap_leak_Qps =
				    info->load.incap_C / 200;
			}
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
			uint64_t cur_ID = crc64(comps[1], strlen(comps[1]));
			elec_comp_info_t *info2 =
			    find_comp_info(infos, num_comps, comps[1]);

			if (info2 == NULL) {
				logMsg("%s:%d: unknown component %s",
				    filename, linenum, comps[1]);
				free_strlist(comps, n_comps);
				goto errout;
			}
			for (unsigned i = 0; i < bus_ID_cur; i++) {
				if (bus_IDs_seen[i] == cur_ID) {
					logMsg("%s:%d: duplicate endpoint %s",
					    filename, linenum, comps[1]);
					free_strlist(comps, n_comps);
					goto errout;
				}
			}
			if (bus_ID_cur < MAX_BUS_UNIQ)
				bus_IDs_seen[bus_ID_cur++] = cur_ID;
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
			cb->autogen = true;

			bus = &infos[comp_i++];
			bus->type = ELEC_BUS;
			bus->name = sprintf_alloc("CB_BUS_%s", info->name);
			bus->bus.ac = info->load.ac;
			bus->autogen = true;

			VERIFY(add_info_link(bus, info, NULL));
			VERIFY(add_info_link(bus, cb, NULL));
		} else if (strcmp(cmd, "STD_LOAD") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_LOAD) {
			info->load.std_load = atof(comps[1]);
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
libelec_add_user_cb(elec_sys_t *sys, bool pre, elec_user_cb_t cb,
    void *userinfo)
{
	user_cb_info_t *info = safe_calloc(1, sizeof (*info));
	avl_index_t where;

	ASSERT(sys != NULL);
	ASSERT(cb != NULL);

	info->pre = !!pre;
	info->cb = cb;
	info->userinfo = userinfo;

	mutex_enter(&sys->user_cbs_lock);
	VERIFY3P(avl_find(&sys->user_cbs, info, &where), ==, NULL);
	avl_insert(&sys->user_cbs, info, where);
	mutex_exit(&sys->user_cbs_lock);
}

void
libelec_remove_user_cb(elec_sys_t *sys, bool pre, elec_user_cb_t cb,
    void *userinfo)
{
	user_cb_info_t srch, *info;

	ASSERT(sys != NULL);
	ASSERT(cb != NULL);

	srch.pre = !!pre;
	srch.cb = cb;
	srch.userinfo = userinfo;

	mutex_enter(&sys->user_cbs_lock);
	info = avl_find(&sys->user_cbs, &srch, NULL);
	VERIFY(info != NULL);
	avl_remove(&sys->user_cbs, info);
	mutex_exit(&sys->user_cbs_lock);
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
	 * holding the lock.
	 */
	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		cb(comp, userinfo);
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

elec_comp_info_t *
libelec_comp2info(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	/* immutable binding */
	return (comp->info);
}

elec_comp_t *
libelec_comp_find(elec_sys_t *sys, const char *name)
{
	elec_comp_info_t srch_info = { .name = (char *)name };
	const elec_comp_t srch_comp = { .info = &srch_info };

	ASSERT(sys != NULL);
	/* Component list is immutable, no need to lock */
	return (avl_find(&sys->name2comp, &srch_comp, NULL));
}

size_t
libelec_comp_get_num_conns(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);

	/* Electrical configuration is immutable, no need to lock */
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

	/* Electrical configuration is immutable, no need to lock */
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
	double volts;

	ASSERT(comp != NULL);

	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	volts = comp->ro.in_volts;
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (volts);
}

double
libelec_comp_get_out_volts(const elec_comp_t *comp)
{
	double volts;

	ASSERT(comp != NULL);

	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	volts = comp->ro.out_volts;
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (volts);
}

double
libelec_comp_get_in_amps(const elec_comp_t *comp)
{
	double amps;

	ASSERT(comp != NULL);

	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	amps = comp->ro.in_amps * (1 - comp->ro.leak_factor);
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (amps);
}

double
libelec_comp_get_out_amps(const elec_comp_t *comp)
{
	double amps;

	ASSERT(comp != NULL);

	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	amps = comp->ro.out_amps * (1 - comp->ro.leak_factor);
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (amps);
}

double
libelec_comp_get_in_pwr(const elec_comp_t *comp)
{
	double watts;

	ASSERT(comp != NULL);

	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	watts = comp->ro.in_pwr * (1 - comp->ro.leak_factor);
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (watts);
}

double
libelec_comp_get_out_pwr(const elec_comp_t *comp)
{
	double watts;

	ASSERT(comp != NULL);

	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	watts = comp->ro.out_pwr * (1 - comp->ro.leak_factor);
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (watts);
}

double
libelec_comp_get_incap_volts(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_LOAD);
	return (comp->load.incap_U);
}

void
libelec_comp_set_failed(elec_comp_t *comp, bool failed)
{
	ASSERT(comp != NULL);
	comp->ro.failed = failed;
}

bool
libelec_comp_get_failed(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->ro.failed);
}

void
libelec_comp_set_shorted(elec_comp_t *comp, bool shorted)
{
	ASSERT(comp != NULL);
	comp->ro.shorted = shorted;
}

bool
libelec_comp_get_shorted(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->ro.shorted);
}

static void
network_reset(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		comp->upstream = NULL;
		comp->src = NULL;

		comp->rw.in_volts = 0;
		comp->rw.in_amps = 0;
		comp->rw.out_volts = 0;
		comp->rw.out_amps = 0;

		mutex_enter(&comp->rw_ro_lock);
		comp->rw.failed = comp->ro.failed;
		comp->rw.shorted = comp->ro.shorted;
		if (comp->rw.shorted) {
			comp->rw.leak_factor = wavg(0.9, 0.999,
			    crc64_rand_fract());
		} else {
			comp->rw.leak_factor = 0;
		}
		mutex_exit(&comp->rw_ro_lock);

		comp->integ_mask = 0;
		switch (comp->info->type) {
		case ELEC_LOAD:
			comp->load.seen = false;
			break;
		case ELEC_TIE:
			/* Transfer the latest tie state to the worker set */
			mutex_enter(&comp->tie.lock);
			memcpy(comp->tie.wk_state, comp->tie.cur_state,
			    comp->tie.n_buses * sizeof (*comp->tie.wk_state));
			mutex_exit(&comp->tie.lock);
			break;
		case ELEC_CB:
#ifndef	LIBELEC_NO_LIBSWITCH
			if (comp->cb.sw != NULL) {
				comp->cb.cur_set =
				    !libswitch_read(comp->cb.sw, NULL);
			}
#endif	/* LIBELEC_NO_LIBSWITCH */
			comp->cb.wk_set = comp->cb.cur_set;
			break;
		default:
			break;
		}
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

	if (!gen->rw.failed) {
		gen->rw.in_volts = (rpm / gen->gen.ctr_rpm) *
		    gen->gen.stab_factor * gen->info->gen.volts;
		gen->rw.out_volts = gen->rw.in_volts;
	} else {
		gen->rw.in_volts = 0;
		gen->rw.out_volts = 0;
	}
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
	temp_coeff = fx_lin_multi(T, temp_energy_curve, true);

	I_max = batt->info->batt.max_pwr / batt->info->batt.volts;
	I_rel = batt->batt.prev_amps / I_max;
	U = batt->info->batt.volts * (1 - pow(I_rel, 1.45)) *
	    fx_lin_multi(batt->batt.chg_rel, chg_volt_curve, true);

	J_max = batt->info->batt.capacity * temp_coeff;
	J = batt->batt.chg_rel * J_max;
	J -= U * batt->batt.prev_amps * d_t;

	/* Recalculate the new voltage and relative charge state */
	if (!batt->rw.failed) {
		batt->rw.in_volts = U;
		batt->rw.out_volts = U;
	} else {
		batt->rw.in_volts = 0;
		batt->rw.out_volts = 0;
	}
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

	amps_rat = cb->rw.out_amps / cb->info->cb.max_amps;
	FILTER_IN(cb->cb.temp, amps_rat, d_t, cb->info->cb.rate);

	if (cb->cb.temp >= 1.0)
		cb->cb.cur_set = cb->cb.wk_set = false;
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
load_incap_update(elec_comp_t *comp, double d_t)
{
	const elec_comp_info_t *info;
	double d_Q;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_LOAD);
	ASSERT3F(d_t, >, 0);

	info = comp->info;
	if (info->load.incap_C == 0)
		return;

	d_Q = comp->load.incap_d_Q - info->load.incap_leak_Qps * d_t;
	comp->load.incap_U += d_Q / info->load.incap_C;
	comp->load.incap_U = MAX(comp->load.incap_U, 0);
	if (comp->rw.failed)
		comp->load.incap_U = 0;
}

static void
network_loads_update(elec_sys_t *sys, double d_t)
{
	ASSERT(sys != NULL);
	ASSERT3F(d_t, >, 0);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);

		if (comp->info->type == ELEC_CB) {
			network_update_cb(comp, d_t);
		} else if (comp->info->type == ELEC_LOAD) {
			/*
			 * If we haven't seen this component, that means we
			 * need to run the load integration manually to take
			 * care of input capacitance.
			 */
			if (!comp->load.seen)
				network_load_integrate_load(comp, 0, d_t);
			load_incap_update(comp, d_t);
		}

		comp->rw.in_pwr = comp->rw.in_volts * comp->rw.in_amps;
		comp->rw.out_pwr = comp->rw.out_volts * comp->rw.out_amps;
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

	if (comp->rw.in_volts < src->rw.out_volts) {
		comp->src = src;
		comp->upstream = upstream;

		if (!comp->rw.failed)
			comp->rw.in_volts = src->rw.out_volts;
		else
			comp->rw.in_volts = 0;

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
	bool found = false, tied = false;

	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	/* Check if the upstream bus is currently tied */
	for (unsigned i = 0; i < comp->tie.n_buses; i++) {
		if (upstream == comp->tie.buses[i]) {
			if (comp->tie.wk_state[i]) {
				if (comp->rw.in_volts < src->rw.out_volts) {
					comp->src = src;
					comp->upstream = upstream;
					comp->rw.in_volts = src->rw.out_volts;
				}
				if (comp->rw.failed)
					comp->rw.in_volts = 0;
				tied = true;
			}
			found = true;
			break;
		}
	}
	ASSERT(found);
	if (tied) {
		for (unsigned i = 0; i < comp->tie.n_buses; i++) {
			if (upstream != comp->tie.buses[i] &&
			    comp->tie.wk_state[i]) {
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
	if (upstream == comp->tru.ac && comp->rw.in_volts < src->rw.out_volts) {
		comp->src = src;
		comp->upstream = upstream;
		if (!comp->rw.failed) {
			comp->rw.in_volts = src->rw.out_volts;
			comp->rw.out_volts = comp->info->tru.out_volts *
			    (comp->rw.in_volts / comp->info->tru.in_volts);
		} else {
			comp->rw.in_volts = 0;
			comp->rw.out_volts = 0;
		}
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

	if (comp->cb.wk_set && comp->rw.in_volts < src->rw.out_volts) {
		comp->src = src;
		comp->upstream = upstream;
		if (!comp->rw.failed)
			comp->rw.in_volts = src->rw.out_volts;
		else
			comp->rw.in_volts = 0;
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
	    comp->rw.in_volts < src->rw.out_volts) {
		comp->src = src;
		comp->upstream = upstream;
		if (!comp->rw.failed)
			comp->rw.in_volts = src->rw.out_volts;
		else
			comp->rw.in_volts = 0;
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
		if (comp->rw.in_volts < src->rw.out_volts) {
			comp->src = src;
			comp->upstream = upstream;
			if (!comp->rw.failed)
				comp->rw.in_volts = src->rw.out_volts;
			else
				comp->rw.in_volts = 0;
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
		if ((comp->info->type == ELEC_BATT ||
		    comp->info->type == ELEC_GEN) && comp->rw.out_volts != 0) {
			network_paint_src(sys, comp);
		}
	}
}

static void
network_load_integrate_tru(elec_comp_t *comp, unsigned depth,
    uint64_t src_mask, double d_t)
{
	double eff;

	ASSERT(comp != NULL);
	ASSERT(comp->tru.ac != NULL);
	ASSERT(comp->tru.dc != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TRU);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	/* When hopping over to the DC network, the TRU becomes the src */
	network_load_integrate_comp(comp, comp, comp->tru.dc, depth + 1,
	    src_mask, d_t);

	if (comp->rw.failed || comp->rw.in_volts == 0) {
		comp->rw.in_amps = 0;
		comp->rw.out_amps = 0;
		return;
	}
	comp->rw.out_amps = comp->tru.dc->rw.in_amps;
	eff = fx_lin_multi(comp->rw.out_amps, comp->info->tru.eff_curve,
	    true);
	ASSERT3F(eff, >, 0);
	comp->rw.in_amps = ((comp->rw.out_volts / comp->rw.in_volts) *
	    comp->rw.out_amps) / eff;
	ASSERT(comp->tru.ac != NULL);
}

static void
network_load_integrate_load(elec_comp_t *comp, unsigned depth, double d_t)
{
	double load_WorI, load_I, in_volts_net, incap_I;
	const elec_comp_info_t *info;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_LOAD);
	UNUSED(depth);

	info = comp->info;

	/*
	 * If the input voltage is lower than our input capacitance voltage,
	 * it will be our input capacitance powering the load, not the input.
	 */
	in_volts_net = MAX(comp->rw.in_volts, comp->load.incap_U);

	/*
	 * Only ask the load if we are receiving sufficient volts.
	 */
	if (in_volts_net >= comp->info->load.min_volts) {
		load_WorI = info->load.std_load;
		if (info->load.get_load != NULL)
			load_WorI += info->load.get_load(comp);
	} else {
		load_WorI = 0;
	}
	ASSERT3F(load_WorI, >=, 0);

	/*
	 * If the load use a stabilized power supply, the load value is
	 * in Watts. Calculate the effective current.
	 */
	if (info->load.stab) {
		double volts = MAX(in_volts_net, info->load.min_volts);
		ASSERT3F(volts, >, 0);
		load_I = load_WorI / volts;
	} else {
		load_I = load_WorI;
	}
	if (comp->rw.shorted) {
		/*
		 * Shorted components randomly consume 10-100x their
		 * nominal power.
		 */
		load_I *= wavg(10, 100, crc64_rand_fract());
	} else if (comp->rw.failed) {
		/*
		 * Failed components just drop their power consumption to zero
		 */
		load_I = 0;
	}
	/*
	 * When the input voltage is greater than the input capacitance
	 * voltage, we will be charging up the input capacitance.
	 */
	if (info->load.incap_C > 0 &&
	    comp->rw.in_volts > comp->load.incap_U + 0.01) {
		/*
		 * Capacitor voltage U_c is:
		 *
		 * U_c = U_in * (1 - e^(-t / RC))
		 *
		 * Where:
		 *	U_in - the input charging voltage
		 *	t - amount of time the cap has been charging in seconds
		 *	R - a series resistance to limit charge current in Ohms
		 *	C - the capacitance in Farad
		 *
		 * Since we operate in a fixed time simulation steps, we
		 * want to instead compute the amount of change in charge
		 * in one simulation step. So we start with the capacitor
		 * voltage from the previous simulation loop, U_c_old, and
		 * subtract it from the input voltage. We can then compute
		 * the new capacitor voltage using a stepped algorithm:
		 *
		 * U_c_new = U_c_old + ((U_in - U_c_old) * (1 - e^(-t / RC)))
		 */
		double U_in = comp->rw.in_volts;
		double U_c_old = comp->load.incap_U;
		double R = info->load.incap_R;
		double C = info->load.incap_C;
		double incap_U_new = U_c_old +
		    ((U_in - U_c_old) * (1 - exp(-d_t / (R * C))));
		/*
		 * Next we convert the previous and new capacitor voltages
		 * to a total charge value and figure out the net change
		 * in charge.
		 *
		 * Q_old = U_c_old * C
		 * Q_new = U_c_new * C
		 * Q_delta = Q_new - Q_old
		 *
		 * Where Q_old and Q_new are the old and new capacitor
		 * charge states (in Coulomb). To calculate the charge
		 * current, we then simply divide the change in charge
		 * Q_delta by the time quantum:
		 *
		 * I = Q_delta / t
		 */
		double Q_old = comp->load.incap_U * info->load.incap_C;
		double Q_new = incap_U_new * info->load.incap_C;
		double Q_delta = Q_new - Q_old;

		incap_I = Q_delta / d_t;
	} else {
		incap_I = 0;
	}

	/*
	 * If the input capacitance has a greater voltage than the network
	 * input voltage, then we need to calculate how much of the charge
	 * will be provided by the input capacitance. The amount of charge
	 * Q_load that the load will draw in one time step is:
	 *
	 * Q_load = I_load * t
	 *
	 * The amount of charge that the capacitor Q_c can provide is
	 * dependent upon the delta between capacitor charge and network
	 * input voltage:
	 *
	 * Q_c = (U_c - U_in) * C
	 *
	 * After draining this amount of charge, the capacitor's voltage
	 * will be lower than the input voltage and so no more charge can
	 * be drawn from it.
	 */
	if (comp->load.incap_U > comp->rw.in_volts && load_I > 0) {
		/* Amount of charge requested by the load in this time step */
		double load_Q = load_I * d_t;
		/* Amount of charge that can be drawn from the incap */
		double avail_Q = (comp->load.incap_U - comp->rw.in_volts) *
		    info->load.incap_C;
		/* Amount of charge actually drawn from the incap */
		double used_Q = MIN(load_Q, avail_Q);

		/*
		 * Subtract the charge provided by the incap from the
		 * network-demanded charge.
		 */
		load_Q -= used_Q;

		/*
		 * Actual network current is the delta vs what the incap
		 * can provide.
		 */
		comp->rw.in_amps = load_Q / d_t;
		comp->rw.out_amps = load_I;
		/*
		 * The final voltage is a mixture of the network voltage
		 * and incap voltage, in a ratio determined by how many
		 * units of charge were provided from the incap (at the
		 * the incap's voltage) vs how many came in from the
		 * network (at the network's input voltage).
		 */
		ASSERT(used_Q + load_Q != 0);
		comp->rw.out_volts = (comp->load.incap_U * used_Q +
		    comp->rw.in_volts * load_Q) / (used_Q + load_Q);
		comp->load.incap_d_Q = -used_Q;
	} else {
		/*
		 * Don't forget to add the input capacitance charging
		 * current to the network current draw.
		 */
		comp->rw.in_amps = load_I + incap_I;
		comp->rw.out_amps = load_I;
		comp->rw.out_volts = comp->rw.in_volts;
		comp->load.incap_d_Q = incap_I * d_t;
	}
	ASSERT(!isnan(comp->rw.out_amps));
	ASSERT(!isnan(comp->rw.out_volts));
	comp->load.seen = true;
}

static void
network_load_integrate_bus(const elec_comp_t *src, elec_comp_t *comp,
    unsigned depth, uint64_t src_mask, double d_t)
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
			    comp->bus.comps[i], depth + 1, src_mask, d_t);
			ASSERT(!isnan(comp->bus.comps[i]->rw.in_amps));
			amps += comp->bus.comps[i]->rw.in_amps;
		}
	}
	comp->rw.out_amps = amps;
	comp->rw.in_amps = amps;
}

static void
network_load_integrate_tie(const elec_comp_t *src, elec_comp_t *comp,
    unsigned depth, uint64_t src_mask, double d_t)
{
	double amps = 0;

	ASSERT(src != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	for (unsigned i = 0; i < comp->tie.n_buses; i++) {
		ASSERT(comp->tie.buses[i] != NULL);
		if (comp->tie.wk_state[i] &&
		    comp->tie.buses[i] != comp->upstream) {
			network_load_integrate_comp(src, comp,
			    comp->tie.buses[i], depth + 1, src_mask, d_t);
			ASSERT(!isnan(comp->tie.buses[i]->rw.in_amps));
			amps += comp->tie.buses[i]->rw.in_amps;
		}
	}

	comp->rw.out_amps = amps;
	comp->rw.in_amps = amps;
}

static void
network_load_integrate_cb(const elec_comp_t *src, elec_comp_t *comp,
    unsigned depth, uint64_t src_mask, double d_t)
{
	ASSERT(src != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (!comp->cb.wk_set)
		return;
	if (comp->upstream == comp->cb.sides[0]) {
		network_load_integrate_comp(src, comp,
		    comp->cb.sides[1], depth + 1, src_mask, d_t);
		comp->rw.out_amps = comp->cb.sides[1]->rw.in_amps;
	} else {
		ASSERT3P(comp->upstream, ==, comp->cb.sides[1]);
		network_load_integrate_comp(src, comp,
		    comp->cb.sides[0], depth + 1, src_mask, d_t);
		comp->rw.out_amps = comp->cb.sides[0]->rw.in_amps;
	}
	ASSERT(!isnan(comp->rw.in_amps));
	comp->rw.in_amps = comp->rw.out_amps;
}

static void
network_load_integrate_comp(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *comp, unsigned depth,
    uint64_t src_mask, double d_t)
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

	ASSERT0(comp->integ_mask & src_mask);
	comp->integ_mask |= src_mask;

	switch (comp->info->type) {
	case ELEC_BATT:
		network_load_integrate_comp(src, comp, comp->batt.bus,
		    depth + 1, src_mask, d_t);
		comp->rw.out_amps = comp->batt.bus->rw.in_amps;
		comp->batt.prev_amps = comp->rw.out_amps;
		break;
	case ELEC_GEN:
		network_load_integrate_comp(src, comp, comp->gen.bus,
		    depth + 1, src_mask, d_t);
		comp->rw.out_amps = comp->gen.bus->rw.in_amps;
		comp->rw.in_volts = comp->rw.out_volts;
		comp->rw.in_amps = comp->rw.out_amps /
		    fx_lin_multi(comp->rw.in_volts * comp->rw.out_amps,
		    comp->info->gen.eff_curve, true);
		break;
	case ELEC_TRU:
		network_load_integrate_tru(comp, depth, src_mask, d_t);
		break;
	case ELEC_LOAD:
		network_load_integrate_load(comp, depth, d_t);
		break;
	case ELEC_BUS:
		network_load_integrate_bus(src, comp, depth, src_mask, d_t);
		break;
	case ELEC_CB:
		network_load_integrate_cb(src, comp, depth, src_mask, d_t);
		break;
	case ELEC_TIE:
		network_load_integrate_tie(src, comp, depth, src_mask, d_t);
		break;
	case ELEC_DIODE:
		network_load_integrate_comp(src, comp,
		    comp->diode.sides[1], depth + 1, src_mask, d_t);
		comp->rw.out_amps = comp->diode.sides[1]->rw.in_amps;
		comp->rw.in_amps = comp->rw.out_amps;
		ASSERT(!isnan(comp->rw.in_amps));
		break;
	}
}

static void
network_load_integrate(elec_sys_t *sys, double d_t)
{
	int src_i = 0;

	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		if (comp->info->type == ELEC_BATT ||
		    comp->info->type == ELEC_GEN) {
			ASSERT3U(src_i, <, 64);
			network_load_integrate_comp(comp, comp, comp, 0,
			    (uint64_t)1 << src_i, d_t);
			src_i++;
		}
	}
}

static void
network_state_xfer(elec_sys_t *sys)
{
	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		mutex_enter(&comp->rw_ro_lock);
		comp->ro = comp->rw;
		mutex_exit(&comp->rw_ro_lock);
	}
}

static bool_t
elec_sys_worker(void *userinfo)
{
	elec_sys_t *sys;
	const double d_t = USEC2SEC(EXEC_INTVAL);

	ASSERT(userinfo != NULL);
	sys = userinfo;

	mutex_enter(&sys->paused_lock);
	if (sys->paused) {
		mutex_exit(&sys->paused_lock);
		return (B_TRUE);
	}
	mutex_exit(&sys->paused_lock);

	mutex_enter(&sys->worker_interlock);

	mutex_enter(&sys->user_cbs_lock);
	for (user_cb_info_t *ucbi = avl_first(&sys->user_cbs); ucbi != NULL;
	    ucbi = AVL_NEXT(&sys->user_cbs, ucbi)) {
		if (ucbi->pre) {
			ASSERT(ucbi->cb != NULL);
			ucbi->cb(sys, true, ucbi->userinfo);
		}
	}
	mutex_exit(&sys->user_cbs_lock);

	network_reset(sys);
	network_srcs_update(sys, d_t);
	network_paint(sys);
	network_load_integrate(sys, d_t);
	network_loads_update(sys, d_t);
	network_state_xfer(sys);

	mutex_enter(&sys->user_cbs_lock);
	for (user_cb_info_t *ucbi = avl_first(&sys->user_cbs); ucbi != NULL;
	    ucbi = AVL_NEXT(&sys->user_cbs, ucbi)) {
		if (!ucbi->pre) {
			ASSERT(ucbi->cb != NULL);
			ucbi->cb(sys, false, ucbi->userinfo);
		}
	}
	mutex_exit(&sys->user_cbs_lock);

	mutex_exit(&sys->worker_interlock);

	return (true);
}

static void
comp_free(elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info);

#ifdef	LIBELEC_WITH_DRS
	dr_delete(&comp->drs.in_volts);
	dr_delete(&comp->drs.out_volts);
	dr_delete(&comp->drs.in_amps);
	dr_delete(&comp->drs.out_amps);
	dr_delete(&comp->drs.in_pwr);
	dr_delete(&comp->drs.out_pwr);
#endif	/* defined(LIBELEC_WITH_DRS) */

	if (comp->info->type == ELEC_BUS) {
		free(comp->bus.comps);
	} else if (comp->info->type == ELEC_TIE) {
		free(comp->tie.buses);
		free(comp->tie.cur_state);
		free(comp->tie.wk_state);
		mutex_destroy(&comp->tie.lock);
	}
	mutex_destroy(&comp->rw_ro_lock);

	free(comp);
}

void
libelec_cb_set(elec_comp_t *comp, bool set)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	/*
	 * This is atomic, no locking required. Also, the worker
	 * copies its the breaker state to wk_set at the start.
	 */
	comp->cb.cur_set = set;
}

bool
libelec_cb_get(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	/* atomic read, no need to lock */
	return (comp->cb.cur_set);
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
libelec_tie_set_info_list(elec_comp_t *comp,
    elec_comp_info_t **bus_list, size_t list_len)
{
	bool *new_state;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT(bus_list != NULL || list_len == 0);

	if (list_len == 0) {
		mutex_enter(&comp->tie.lock);
		memset(comp->tie.cur_state, 0,
		    comp->tie.n_buses * sizeof (*comp->tie.cur_state));
		mutex_exit(&comp->tie.lock);
		return;
	}

	/* The buslist is immutable, so no need to lock */
	new_state = safe_calloc(comp->tie.n_buses, sizeof (*new_state));
	for (size_t i = 0; i < list_len; i++) {
		size_t j;
		for (j = 0; j < comp->tie.n_buses; j++) {
			if (comp->tie.buses[j]->info == bus_list[i]) {
				new_state[j] = true;
				break;
			}
		}
		ASSERT_MSG(j != comp->tie.n_buses,
		    "Tie %s is not connected to bus %s", comp->info->name,
		    bus_list[i]->name);
	}

	mutex_enter(&comp->tie.lock);
	memcpy(comp->tie.cur_state, new_state,
	    comp->tie.n_buses * sizeof (*comp->tie.cur_state));
	mutex_exit(&comp->tie.lock);

	free(new_state);
}

void
libelec_tie_set(elec_comp_t *comp, ...)
{
	va_list ap;

	ASSERT(comp != NULL);

	va_start(ap, comp);
	libelec_tie_set_v(comp, ap);
	va_end(ap);
}

void
libelec_tie_set_v(elec_comp_t *comp, va_list ap)
{
	va_list ap2;
	elec_comp_info_t **bus_list;
	size_t len;

	ASSERT(comp != NULL);
	ASSERT(comp->sys != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	va_copy(ap2, ap);
	for (len = 0; va_arg(ap2, elec_comp_t *) != NULL; len++)
		;
	va_end(ap2);

	bus_list = safe_malloc(sizeof (*bus_list) * len);
	for (size_t i = 0; i < len; i++) {
		/* Fuck C and its const fuckness */
		bus_list[i] =
		    (elec_comp_info_t *)(va_arg(ap, elec_comp_t *))->info;
	}
	libelec_tie_set_info_list(comp, bus_list, len);
	free(bus_list);
}

void
libelec_tie_set_all(elec_comp_t *comp, bool tied)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	mutex_enter(&comp->tie.lock);
	for (unsigned i = 0; i < comp->tie.n_buses; i++)
		comp->tie.cur_state[i] = tied;
	mutex_exit(&comp->tie.lock);
}

bool
libelec_tie_get_all(elec_comp_t *comp)
{
	bool tied = true;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	mutex_enter(&comp->tie.lock);
	for (unsigned i = 0; tied && i < comp->tie.n_buses; i++)
		tied &= comp->tie.cur_state[i];
	mutex_exit(&comp->tie.lock);

	return (tied);
}

size_t
libelec_tie_get(elec_comp_t *comp, elec_comp_t **bus_list)
{
	size_t n_buses = 0;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	mutex_enter(&comp->tie.lock);
	for (unsigned i = 0; i < comp->tie.n_buses; i++) {
		if (comp->tie.cur_state[i])
			bus_list[n_buses++] = comp->tie.buses[i];
	}
	mutex_exit(&comp->tie.lock);

	return (n_buses);
}

size_t
libelec_tie_get_num_buses(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	/* This is immutable, so no need to lock */
	return (comp->tie.n_buses);
}

/*
 * This function returns internal mutable state, so it MUSTN'T be used
 * for production code!
 */
elec_comp_t *
libelec_comp_get_src(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->src);
}

/*
 * This function returns internal mutable state, so it MUSTN'T be used
 * for production code!
 */
elec_comp_t *
libelec_comp_get_upstream(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->upstream);
}

double
libelec_batt_get_chg_rel(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_BATT);
	return (comp->batt.chg_rel);
}
