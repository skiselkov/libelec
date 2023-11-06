/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
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

#ifdef	LIBELEC_WITH_LIBSWITCH
#include <libswitch.h>
#endif

#ifdef	LIBELEC_WITH_NETLINK
#include <netlink.h>
#endif

#include "libelec.h"
#include "libelec_types_impl.h"

#define	EXEC_INTVAL		40000	/* us */
#define	MAX_NETWORK_DEPTH	100	/* dimensionless */
#define	NO_NEG_ZERO(x)		((x) == 0.0 ? 0.0 : (x))
#define	CB_SW_ON_DELAY		0.33	/* sec */
#define	MAX_COMPS		(UINT16_MAX + 1)
#define	GEN_MIN_RPM		1e-3

#ifdef	LIBELEC_WITH_NETLINK

#define	LIBELEC_NET_VERSION	1
#define	NETMAPGET(map, idx)	\
	((((map)[(idx) >> 3]) & (1 << ((idx) & 7))) != 0)
#define	NETMAPSET(map, idx) \
	do { \
		(map)[(idx) >> 3] |= (1 << ((idx) & 7)); \
	} while (0)
#define	NETMAPSZ(sys)		(list_count(&sys->comps) % 8 == 0 ? \
    (list_count(&sys->comps) / 8) : (list_count(&sys->comps) / 8 + 1))
#define	NETMAPSZ_REQ(sys)	(sizeof (net_req_map_t) + NETMAPSZ(sys))
static bool send_net_recv_map(elec_sys_t *sys);
static void net_add_recv_comp(elec_comp_t *comp);

#define	NET_XMIT_INTVAL		5	/* divisor for 1 / EXEC_INTVAL */
#define	NET_VOLTS_FACTOR	20.0	/* 0.05 V */
#define	NET_AMPS_FACTOR		40.0	/* 0.025 A */
#define	NET_FREQ_FACTOR		20.0	/* 0.05 Hz */

#define	NET_ADD_RECV_COMP(comp)	net_add_recv_comp((elec_comp_t *)comp)

/* #define	LIBELEC_NET_DBG */

#ifdef	LIBELEC_NET_DBG
#define	NET_DBG_LOG(...)	logMsg(__VA_ARGS__)
#else
#define	NET_DBG_LOG(...)
#endif

#else	/* !defined(LIBELEC_WITH_NETLINK) */

#define	NET_ADD_RECV_COMP(comp)

#endif	/* !defined(LIBELEC_WITH_NETLINK) */

typedef struct {
	bool		pre;
	elec_user_cb_t	cb;
	void		*userinfo;
	avl_node_t	node;
} user_cb_info_t;

/*
 * Can't use VECT2() and NULL_VECT2 macros here, MSVC doesn't have proper
 * support for compound literals.
 */
static const vect2_t batt_temp_energy_curve[] = {
    {C2KELVIN(-90), 0.01},
    {C2KELVIN(-75), 0.01},
    {C2KELVIN(-50), 0.125},
    {C2KELVIN(-20), 0.45},
    {C2KELVIN(-5), 0.7},
    {C2KELVIN(15), 0.925},
    {C2KELVIN(40), 1.0},
    {C2KELVIN(50), 1.0}
};

static elec_comp_info_t *infos_parse(const char *filename, size_t *num_infos);
static void infos_free(elec_comp_info_t *infos, size_t num_infos);

static bool_t elec_sys_worker(void *userinfo);
static void comp_free(elec_comp_t *comp);
static void network_paint_src_comp(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth);
static double network_load_integrate_comp(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *comp, unsigned depth, double d_t);
static double network_load_integrate_load(const elec_comp_t *src,
    elec_comp_t *comp, unsigned depth, double d_t);

static double network_trace(const elec_comp_t *upstream,
    const elec_comp_t *comp, unsigned depth, bool do_print);

#ifdef	LIBELEC_WITH_NETLINK

static void netlink_send_msg_notif(netlink_conn_id_t conn_id, const void *buf,
    size_t bufsz, void *userinfo);
static void netlink_recv_msg_notif(netlink_conn_id_t conn_id, const void *buf,
    size_t bufsz, void *userinfo);

static void elec_net_send_update(elec_sys_t *sys);
static void elec_net_recv_update(elec_sys_t *sys);

#endif	/* defined(LIBELEC_WITH_NETLINK) */

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
		/* Reset the worker interval to default */
		if (sys->started)
			worker_set_interval_nowake(&sys->worker, EXEC_INTVAL);
		mutex_enter(&sys->paused_lock);
		sys->paused = true;
		sys->time_factor = 0;
		mutex_exit(&sys->paused_lock);
		return (1);
	}
	sys->prev_sim_time = sim_time;
	/*
	 * If the time factor returns to 1.0x, we want to make sure we
	 * go back to 1.0x exactly, instead of some fractional 1.04x thing
	 * in case the sim was only ever so slightly accelerated/decelerated.
	 */
	if ((time_factor != sys->time_factor ||
	    (time_factor == 1 && sys->time_factor != 1)) && sys->started) {
		worker_set_interval_nowake(&sys->worker,
		    EXEC_INTVAL / time_factor);
	}
	mutex_enter(&sys->paused_lock);
	sys->paused = false;
	sys->time_factor = time_factor;
	mutex_exit(&sys->paused_lock);

	return (1);
}
#endif	/* defined(XPLANE) */

static inline bool
src_is_AC(const elec_comp_info_t *info)
{
	ASSERT(info != NULL);
	switch (info->type) {
	case ELEC_GEN:
		return (info->gen.freq != 0);
	case ELEC_INV:
		return (true);
	case ELEC_XFRMR:
		return (true);
	default:
		VERIFY_FAIL();
	}
}

static bool
check_upstream(const elec_comp_t *comp, const elec_comp_t *src,
    const elec_comp_t *upstream)
{
	ASSERT(comp != NULL);
	ASSERT(src != NULL);
	ASSERT3U(src->src_idx, <, ELEC_MAX_SRCS);
	ASSERT(upstream != NULL);

	for (unsigned i = 0; i < comp->n_links; i++) {
		if (comp->links[i].comp == upstream)
			return (comp->links[i].srcs[src->src_idx] == src);
	}
	return (false);
}

static double
get_src_fract(const elec_comp_t *comp, const elec_comp_t *src)
{
	ASSERT(comp != NULL);
	ASSERT(src != NULL);

	if (comp->src_int_cond_total > 1e-12) {
		double src_cond = (1.0 / src->info->int_R) * src->rw.out_volts;
		return (MIN(src_cond / comp->src_int_cond_total, 1));
	} else {
		return (1);
	}
}

static double
sum_link_amps(const elec_link_t *link)
{
	double amps = 0;

	ASSERT(link != NULL);
	for (unsigned i = 0; i < ELEC_MAX_SRCS; i++)
		amps += link->out_amps[i];

	return (amps);
}

static int
user_cb_info_compar(const void *a, const void *b)
{
	const user_cb_info_t *ucbi_a = a, *ucbi_b = b;
	const uintptr_t cb_a = (uintptr_t)ucbi_a->cb;
	const uintptr_t cb_b = (uintptr_t)ucbi_a->cb;

	if (!ucbi_a->pre && ucbi_b->pre)
		return (-1);
	if (ucbi_a->pre && !ucbi_b->pre)
		return (1);
	if (cb_a < cb_b)
		return (-1);
	if (cb_a > cb_b)
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

static elec_comp_t *
find_comp(elec_sys_t *sys, const elec_comp_info_t *info, const elec_comp_t *src)
{
	elec_comp_t *comp;
	const elec_comp_t srch = { .info = (elec_comp_info_t *)info };

	ASSERT(sys != NULL);
	ASSERT(info != NULL);
	ASSERT(src != NULL);

	comp = avl_find(&sys->info2comp, &srch, NULL);
	ASSERT_MSG(comp != NULL, "Component for info %s not found "
	    "(referenced from %s)", srch.info->name, src->info->name);

	return (comp);
}

static bool
resolve_bus_links(elec_sys_t *sys, elec_comp_t *bus)
{
	ASSERT(sys != NULL);
	ASSERT(bus != NULL);
	ASSERT(bus->info != NULL);
	ASSERT3U(bus->info->type, ==, ELEC_BUS);

	bus->n_links = bus->info->bus.n_comps;
	bus->links = safe_calloc(bus->n_links, sizeof (*bus->links));

#define	CHECK_COMP(cond, reason) \
	do { \
		if (!(cond)) { \
			logMsg("%s (%s:%d): %s", comp->info->name, \
			    sys->conf_filename, comp->info->parse_linenum, \
			    (reason)); \
			return (false); \
		} \
	} while (0)
#define	CHECK_COMP_V(cond, reason, ...) \
	do { \
		if (!(cond)) { \
			logMsg("%s (%s:%d): " reason, comp->info->name, \
			    sys->conf_filename, comp->info->parse_linenum, \
			    __VA_ARGS__); \
			return (false); \
		} \
	} while (0)

	for (size_t i = 0; i < bus->info->bus.n_comps; i++) {
		elec_comp_t *comp = find_comp(sys, bus->info->bus.comps[i],
		    bus);

		bus->links[i].comp = comp;
		ASSERT(comp->info != NULL);
		switch (comp->info->type) {
		case ELEC_BATT:
			/* Batteries are DC-only devices! */
			CHECK_COMP(!bus->info->bus.ac, "batteries cannot "
			    "connect to AC buses (batteries are inherently "
			    "DC-only devices)");
			ASSERT3U(comp->n_links, ==, 1);
			ASSERT(comp->links != NULL);
			comp->links[0].comp = bus;
			break;
		case ELEC_GEN:
			/* Generators can be DC or AC */
			CHECK_COMP_V(bus->info->bus.ac == src_is_AC(comp->info),
			    "AC/DC status is mismatched between the generator "
			    "and its output bus %s", bus->info->name);
			ASSERT3U(comp->n_links, ==, 1);
			ASSERT(comp->links != NULL);
			comp->links[0].comp = bus;
			break;
		case ELEC_TRU:
			ASSERT3U(comp->n_links, ==, 2);
			ASSERT(comp->links != NULL);
			if (comp->info->tru.ac == bus->info) {
				CHECK_COMP_V(bus->info->bus.ac, "input to the "
				    "TRU must connect to an AC bus, but "
				    "%s is DC", bus->info->name);
				comp->links[0].comp = bus;
			} else {
				ASSERT3P(comp->info->tru.dc, ==, bus->info);
				CHECK_COMP_V(!bus->info->bus.ac, "output of "
				    "the TRU must connect to a DC bus, "
				    "but %s is AC", bus->info->name);
				comp->links[1].comp = bus;
			}
			break;
		case ELEC_INV:
			ASSERT3U(comp->n_links, ==, 2);
			ASSERT(comp->links != NULL);
			if (comp->info->tru.dc == bus->info) {
				CHECK_COMP_V(!bus->info->bus.ac, "input to "
				    "the inverter must connect to a DC bus, "
				    "but %s is AC", bus->info->name);
				comp->links[0].comp = bus;
			} else {
				ASSERT3P(comp->info->tru.ac, ==, bus->info);
				CHECK_COMP_V(bus->info->bus.ac, "output of "
				    "the inverter must connect to an AC bus, "
				    "but %s is DC", bus->info->name);
				comp->links[1].comp = bus;
			}
			break;
		case ELEC_XFRMR:
			ASSERT3U(comp->n_links, ==, 2);
			ASSERT(comp->links != NULL);
			if (comp->info->xfrmr.input == bus->info) {
				CHECK_COMP_V(bus->info->bus.ac, "input to "
				    "the transformer must connect to an AC "
				    "bus, but %s is DC", bus->info->name);
				comp->links[0].comp = bus;
			} else {
				ASSERT3P(comp->info->xfrmr.output, ==,
				    bus->info);
				CHECK_COMP_V(bus->info->bus.ac, "output of "
				    "the transformer must connect to an "
				    "AC bus, but %s is DC", bus->info->name);
				comp->links[1].comp = bus;
			}
			break;
		case ELEC_LOAD:
			CHECK_COMP_V(bus->info->bus.ac == comp->info->load.ac,
			    "cannot connect %s load to %s bus",
			    comp->info->load.ac ? "AC" : "DC",
			    bus->info->bus.ac ? "AC" : "DC");
			ASSERT3U(comp->n_links, ==, 1);
			ASSERT(comp->links != NULL);
			comp->links[0].comp = bus;
			break;
		case ELEC_BUS:
			CHECK_COMP_V(false, "Invalid link: cannot connect "
			    "bus %s directly to bus %s", bus->info->name,
			    comp->info->name);
			break;
		case ELEC_CB:
		case ELEC_SHUNT:
			/* 3-phase breakers are only allowed on AC buses */
			if (comp->info->type == ELEC_CB) {
				CHECK_COMP(!comp->info->cb.triphase ||
				    bus->info->bus.ac, "3-phase breakers "
				    "cannot be connected to DC buses");
			}
			ASSERT3U(comp->n_links, ==, 2);
			ASSERT(comp->links != NULL);
			if (comp->links[0].comp == NULL) {
				comp->links[0].comp = bus;
			} else {
				elec_comp_t *other_bus = comp->links[0].comp;

				CHECK_COMP(comp->links[1].comp == NULL,
				    "too many connections");
				comp->links[1].comp = bus;
				CHECK_COMP_V(bus->info->bus.ac ==
				    other_bus->info->bus.ac, "cannot link "
				    "two buses of incompatible type (%s is "
				    "%s and %s is %s)",
				    bus->info->name,
				    bus->info->bus.ac ? "AC" : "DC",
				    other_bus->info->name,
				    other_bus->info->bus.ac ? "AC" : "DC");
			}
			break;
		case ELEC_TIE:
			comp->n_links++;
			comp->links = safe_realloc(comp->links,
			    comp->n_links * sizeof (*comp->links));
			comp->links[comp->n_links - 1].comp = bus;
			free(comp->tie.cur_state);
			comp->tie.cur_state = safe_calloc(comp->n_links,
			    sizeof (*comp->tie.cur_state));
			free(comp->tie.wk_state);
			comp->tie.wk_state = safe_calloc(comp->n_links,
			    sizeof (*comp->tie.wk_state));
			break;
		case ELEC_DIODE:
			/*
			 * Diodes are DC-only devices. Don't attempt to build
			 * rectifiers, use a TRU for that!
			 */
			CHECK_COMP_V(!bus->info->bus.ac, "cannot connect "
			    "diode %s to an AC bus (libelec cannot be used "
			    "to build a bridge rectifier, use a \"TRU\" "
			    "component for that)", comp->info->name);
			ASSERT3U(comp->n_links, ==, 2);
			ASSERT(comp->links != NULL);
			if (comp->info->diode.sides[0] == bus->info) {
				comp->links[0].comp = bus;
			} else {
				ASSERT3P(comp->info->diode.sides[1], ==,
				    bus->info);
				comp->links[1].comp = bus;
			}
			break;
		default:
			VERIFY_FAIL();
		}
	}
#undef	CHECK_COMP
#undef	CHECK_COMP_V
	return (true);
}

static bool
resolve_comp_links(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		if (comp->info->type == ELEC_BUS) {
			if (!resolve_bus_links(sys, comp))
				return (false);
		} else if (comp->info->type == ELEC_TRU &&
		    comp->info->tru.charger) {
			comp->tru.batt = find_comp(sys,
			    comp->info->tru.batt, comp);
			comp->tru.batt_conn = find_comp(sys,
			    comp->info->tru.batt_conn, comp);
		}
	}
	return (true);
}

WARN_UNUSED_RES_ATTR static bool
check_comp_links(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		for (unsigned i = 0; i < comp->n_links; i++) {
			if (comp->links[i].comp == NULL) {
				logMsg("Component %s is missing a network link",
				    comp->info->name);
				return (false);
			}
		}
	}
	return (true);
}

/**
 * @return All component info structures in the network as a flat array.
 *	This can be useful for enumerating all infos during debugging.
 *	You must NOT free the returned pointer, since the library retains
 *	ownership over it.
 * @param num_infos Mandatory return parameter, which will be filled
 *	with the number of elements in the returned array.
 */
const elec_comp_info_t *
libelec_get_comp_infos(const elec_sys_t *sys, size_t *num_infos)
{
	ASSERT(sys != NULL);
	ASSERT(num_infos != NULL);
	*num_infos = sys->num_infos;
	return (sys->comp_infos);
}

static bool
comp_alloc(elec_sys_t *sys, elec_comp_info_t *info, unsigned *src_i)
{
	elec_comp_t *comp = safe_calloc(1, sizeof (*comp));
	avl_index_t where;
	ASSERT(info != NULL);

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
	/*
	 * Initialize component fields and default values
	 */
	switch (comp->info->type) {
	case ELEC_LOAD:
		comp->load.random_load_factor = 1;
		break;
	case ELEC_BATT:
		comp->src_idx = *src_i;
		(*src_i)++;
		mutex_init(&comp->batt.lock);
		comp->batt.chg_rel = 1.0;
		comp->batt.T = C2KELVIN(15);
		break;
	case ELEC_GEN:
		comp->src_idx = *src_i;
		(*src_i)++;
		mutex_init(&comp->gen.lock);
		comp->gen.tgt_volts = comp->info->gen.volts;
		comp->gen.tgt_freq = comp->info->gen.freq;
		comp->gen.ctr_rpm = AVG(comp->info->gen.min_rpm,
		    comp->info->gen.max_rpm);
		comp->gen.rpm = GEN_MIN_RPM;
		comp->gen.max_stab_U =
		    comp->gen.ctr_rpm / comp->info->gen.min_rpm;
		comp->gen.min_stab_U =
		    comp->gen.ctr_rpm / comp->info->gen.max_rpm;
		if (comp->info->gen.stab_rate_f > 0) {
			comp->gen.max_stab_f = comp->gen.ctr_rpm /
			    comp->info->gen.min_rpm;
			comp->gen.min_stab_f = comp->gen.ctr_rpm /
			    comp->info->gen.max_rpm;
		}
		break;
	case ELEC_TRU:
	case ELEC_INV:
		/* TRUs and inverters are basically the same thing in libelec */
		comp->src_idx = *src_i;
		(*src_i)++;
		break;
	case ELEC_XFRMR:
		comp->src_idx = *src_i;
		(*src_i)++;
		break;
	case ELEC_BUS:
		break;
	case ELEC_CB:
	case ELEC_SHUNT:
		/* CBs and shunts are basically the same thing in libelec */
		comp->scb.cur_set = comp->scb.wk_set = true;
		break;
	case ELEC_TIE:
		mutex_init(&comp->tie.lock);
		break;
	case ELEC_DIODE:
		break;
	case ELEC_LABEL_BOX:
		break;
	}
	/*
	 * Initialize links
	 */
	switch (comp->info->type) {
	case ELEC_BATT:
	case ELEC_GEN:
	case ELEC_LOAD:
		comp->n_links = 1;
		break;
	case ELEC_TRU:
	case ELEC_INV:
	case ELEC_XFRMR:
	case ELEC_CB:
	case ELEC_SHUNT:
	case ELEC_DIODE:
		comp->n_links = 2;
		break;
	case ELEC_BUS:
	case ELEC_LABEL_BOX:
	case ELEC_TIE:
		break;
	}
	if (comp->n_links != 0)
		comp->links = safe_calloc(comp->n_links, sizeof (*comp->links));
	/*
	 * Check if we're trying to create too many sources
	 */
	if (comp->src_idx > ELEC_MAX_SRCS) {
		logMsg("%s:%d: too many electrical sources (max: 64).",
		    sys->conf_filename, info->parse_linenum);
		comp_free(comp);
		return (false);
	}
	/*
	 * Insert the component into the relevant type-specific lists
	 */
	if (comp->info->type == ELEC_BATT || comp->info->type == ELEC_GEN)
		list_insert_tail(&sys->gens_batts, comp);
	else if (comp->info->type == ELEC_TIE)
		list_insert_tail(&sys->ties, comp);
	/*
	 * If dataref exposing is enabled, create those now.
	 */
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

	return (true);
}

/**
 * Allocates and initializes a new electrical system. You must supply
 * a file, which holds the definition of the electrical network layout.
 * See [Electrical Network File Format](../ConfFileFormat.md) for more
 * information.
 *
 * After initialization, the electrical network is in a stopped state.
 * This means, the simulation isn't yet running, allowing you to
 * configure the network by setting up custom callbacks, or manipulating
 * the parsed info structures in the components. Use libelec_comp_find()
 * to locate components by name and libelec_comp2info() to extract the
 * parsed info structures for the respective component and modify the
 * info structure as necessary.
 *
 * Once you are satisfied with the configuration of the network, you can
 * start the simulation up by running libelec_sys_start(). This spawns a
 * background thread, running the physics simulation of the network.
 * You should only call this when the host simulator has finished loading
 * your airplane (i.e. on the first frame of the simulator actually
 * running).
 *
 * On unload, first stop the network using libelec_sys_stop(). After
 * stopping the network, you could potentially modify the info structs
 * and restart the network again. A stopped network can be freed using
 * libelec_destroy().
 *
 * @param filename Full file path and name to the electrical network
 *	definition file. This must conform to the syntax described in
 *	[Configuration File Format](../ConfFileFormat.md).
 *
 * @return The parsed and initialized electrical network, if the network
 *	could be successfully constructuted. The networked is a stopped
 *	state, meaning, the physics worker thread isn't operating yet
 *	and no simulation is taking place. After you have performed any
 *	runtime configuration of the network, it must be started using
 *	libelec_sys_start() on the first simulator frame.
 *
 * @return If an error occurred, this function returns NULL instead. You
 *	MUST be prepared to handle a NULL return out of this function,
 *	since it's easy to make a mistake in the
 *	definition file and this is your primary method of detecting
 *	such a failure. The failure reason is printed to the system log
 *	using the logMsg() function of libacfutils (you must initialize
 *	libacfutils' logging subsystem beforehand).
 *
 * @see libelec_sys_start()
 * @see libelec_sys_stop()
 * @see libelec_destroy()
 * @see libelec_comp_find()
 * @see libelec_comp2info()
 */
elec_sys_t *
libelec_new(const char *filename)
{
	elec_sys_t *sys = safe_calloc(1, sizeof (*sys));
	unsigned src_i = 0, comp_i = 0;
	void *buf;
	size_t bufsz;

	ASSERT(filename != NULL);

	buf = file2buf(filename, &bufsz);
	if (buf == NULL) {
		logMsg("Can't open %s: %s", filename, strerror(errno));
		ZERO_FREE(sys);
		return (NULL);
	}
	sys->conf_filename = safe_strdup(filename);
	sys->conf_crc = crc64(buf, bufsz);
	free(buf);

	sys->comp_infos = infos_parse(filename, &sys->num_infos);
	if (sys->comp_infos == NULL) {
		ZERO_FREE(sys);
		return (NULL);
	}
	list_create(&sys->comps, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, comps_node));
	list_create(&sys->gens_batts, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, gens_batts_node));
	list_create(&sys->ties, sizeof (elec_comp_t),
	    offsetof(elec_comp_t, ties_node));
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

	for (size_t i = 0; i < sys->num_infos; i++) {
		if (!comp_alloc(sys, &sys->comp_infos[i], &src_i))
			goto errout;
	}
	/* Resolve component links */
	if (!resolve_comp_links(sys) || !check_comp_links(sys))
		goto errout;
	/*
	 * Network sending is using 16-bit indices
	 */
	ASSERT3U(list_count(&sys->comps), <=, MAX_COMPS);
	sys->comps_array = safe_calloc(list_count(&sys->comps),
	    sizeof (*sys->comps_array));
	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		sys->comps_array[comp_i] = comp;
		comp->comp_idx = comp_i;
		comp_i++;
	}
#ifdef	XPLANE
	fdr_find(&sys->drs.sim_speed_act, "sim/time/sim_speed_actual");
	fdr_find(&sys->drs.sim_time, "sim/time/total_running_time_sec");
	fdr_find(&sys->drs.paused, "sim/time/paused");
	fdr_find(&sys->drs.replay, "sim/time/is_in_replay");
	VERIFY(XPLMRegisterDrawCallback(elec_draw_cb, xplm_Phase_Window,
	    0, sys));
#endif	/* defined(XPLANE) */

	return (sys);
errout:
	libelec_destroy(sys);
	return (NULL);
}

/*
 * Validates the elec_comp_info_t's we've parsed out of the file.
 * This needs to run after the parsing phase, but before the network
 * is fully ready to operate. The user might want to set up info
 * struct callbacks before this is done, so we shouldn't error out
 * on missing mandatory callbacks. That check is done in
 * validate_elec_comp_infos_start() just before the network is started.
 */
static bool
validate_elec_comp_infos_parse(const elec_comp_info_t *infos, size_t n_infos,
    const char *filename)
{
	ASSERT(infos != NULL || n_infos == 0);
	ASSERT(filename != NULL);

#define	CHECK_COMP(cond, reason) \
	do { \
		if (!(cond)) { \
			logMsg("%s (%s:%d): %s", info->name, filename, \
			    info->parse_linenum, (reason)); \
			return (false); \
		} \
	} while (0)

	for (size_t i = 0; i < n_infos; i++) {
		const elec_comp_info_t *info = &infos[i];

		switch (info->type) {
		case ELEC_BATT:
			CHECK_COMP(info->batt.volts > 0,
			    "missing required \"VOLTS\" parameter");
			CHECK_COMP(info->batt.max_pwr > 0,
			    "missing required \"MAX_PWR\" parameter");
			CHECK_COMP(info->batt.capacity >= 0,
			    "missing required \"CAPACITY\" parameter");
			CHECK_COMP(info->batt.chg_R > 0,
			    "missing required \"CHG_R\" parameter");
			break;
		case ELEC_GEN:
			CHECK_COMP(info->gen.volts > 0,
			    "missing required \"VOLTS\" parameter");
			CHECK_COMP(info->gen.exc_rpm <= info->gen.min_rpm,
			    "\"EXC_RPM\" parameter must be less than or "
			    "equal to \"MIN_RPM\"");
			CHECK_COMP(info->gen.min_rpm > 0,
			    "missing required \"MIN_RPM\" parameter");
			CHECK_COMP(info->gen.max_rpm > 0,
			    "missing required \"MAX_RPM\" parameter");
			CHECK_COMP(info->gen.min_rpm < info->gen.max_rpm,
			    "\"MIN_RPM\" must be lower than \"MAX_RPM\"");
			CHECK_COMP(info->gen.eff_curve != NULL, "generators "
			    "require at least two \"CURVEPT EFF\" parameters");
			break;
		case ELEC_TRU:
		case ELEC_INV:
			CHECK_COMP(info->tru.in_volts > 0,
			    "missing required \"IN_VOLTS\" parameter");
			CHECK_COMP(info->tru.out_volts > 0,
			    "missing required \"OUT_VOLTS\" parameter");
			if (info->type == ELEC_INV) {
				CHECK_COMP(info->tru.out_freq > 0,
				    "missing required \"OUT_FREQ\" parameter");
			}
			CHECK_COMP(info->tru.eff_curve != NULL,
			    "at least two \"CURVEPT EFF\" parameters required");
			CHECK_COMP(info->tru.ac != NULL,
			    "AC side not connected");
			CHECK_COMP(info->tru.dc != NULL,
			    "DC side not connected");
			break;
		case ELEC_XFRMR:
			CHECK_COMP(info->xfrmr.in_volts > 0,
			    "missing required \"IN_VOLTS\" parameter");
			CHECK_COMP(info->xfrmr.out_volts > 0,
			    "missing required \"OUT_VOLTS\" parameter");
			CHECK_COMP(info->xfrmr.input != NULL,
			    "input side not connected");
			CHECK_COMP(info->xfrmr.output != NULL,
			    "output side not connected");
			break;
		case ELEC_LOAD:
			/*
			 * Stabilized loads MUST declare a minimum voltage.
			 * Non-stabilized loads don't need to.
			 */
			CHECK_COMP(info->load.min_volts > 0 ||
			    !info->load.stab, "loads must specify "
			    "a \"MIN_VOLTS\" when \"STAB\" is set to TRUE");
			ASSERT3F(info->load.incap_C, >=, 0);
			if (info->load.incap_C > 0)
				ASSERT3F(info->load.incap_R, >, 0);
			break;
		case ELEC_BUS:
			CHECK_COMP(info->bus.comps != NULL,
			    "buses must connect to at least 1 component");
			break;
		case ELEC_CB:
		case ELEC_SHUNT:
		case ELEC_TIE:
			break;
		case ELEC_DIODE:
			CHECK_COMP(info->diode.sides[0] != NULL &&
			    info->diode.sides[1] != NULL,
			    "diodes need to have both end points connected");
			break;
		case ELEC_LABEL_BOX:
			break;
		}
	}

#undef	CHECK_COMP

	return (true);
}

/**
 * @return True if the network has been started (libelec_sys_start() has
 *	been called successfully), false if the network is stopped.
 *	Please note that this doesn't detect a paused state of the
 *	simulator. When the host simulator is paused, the network
 *	simulation doesn't "stop", it simply pauses.
 */
bool
libelec_sys_is_started(const elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	return (sys->started);
}

/**
 * Performs a validation check on the electrical network to see if it is
 * ready for start.
 * @return True if the network has passed validation checks and is ready
 *	to be started. If the validation checks failed, returns false
 *	instead and logs the error reason to the logging subsystem. If
 *	the network is already started, always returns false.
 */
bool
libelec_sys_can_start(const elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	return (!sys->started);
}

/**
 * Starts a stopped libelec network. If the network was already started,
 * this function does nothing.
 *
 * Initially, after loading with libelec_new(), a libelec instance is
 * stopped and not performing any physics simulation. You should configure
 * the returned network as you see fit prior to starting - any
 * modifications to the component info structure are not allowed after the
 * network has been started, as that can lead to race conditions and
 * unpredictable behavior.
 *
 * Since there is usually a significant delay between your aircraft loading
 * into the simulator and the simulation actually starting up, you should
 * delay starting the electrical network until the first simulator frame.
 *
 * @return True if starting the network succeeded, or false if there was
 *	a network configuration error. To allow for more dynamic
 *	reconfiguration of the network 
 */
bool
libelec_sys_start(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	if (!sys->started) {
		if (!libelec_sys_can_start(sys))
			return (false);
#ifndef	LIBELEC_SLOW_DEBUG
		worker_init(&sys->worker, elec_sys_worker, EXEC_INTVAL, sys,
		    "elec_sys");
#else	/* !LIBELEC_SLOW_DEBUG */
		worker_init(&sys->worker, elec_sys_worker, 0, sys, "elec_sys");
#endif	/* !LIBELEC_SLOW_DEBUG */
		sys->started = true;
	}
	return (true);
}

/**
 * Stops the electrical network after it has been started using
 * libelec_sys_start(). If the network was already stopped, this function
 * does nothing.
 * @note You MUST stop the network manually using libelec_sys_stop()
 *	prior to calling libelec_destroy() to free the network.
 */
void
libelec_sys_stop(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	if (!sys->started)
		return;

	worker_fini(&sys->worker);
#ifdef	LIBELEC_WITH_NETLINK
	if (sys->net_recv.active) {
		memset(sys->net_recv.map, 0, NETMAPSZ(sys));
		send_net_recv_map(sys);
	}
#endif	/* defined(LIBELEC_WITH_NETLINK) */
	sys->started = false;
}

/**
 * Sets the simulation rate of libelec. This can be used to adapt libelec's
 * physics to the host simulator's simulation rate, in case the simulator is
 * running faster or slower than real time, or is paused.
 *
 * @note If libelec was built with the `XPLANE` macro ddefined, this
 *	function does nothing, as libelec automatically adapts to X-Plane's
 *	simulation rate.
 *
 * @param time_factor The non-negative factor by which time is accelerated.
 *	Normal real-time operation is 1.0. If the simulator is running twice
 *	as fast as real time, `time_factor` is 2.0. If the simulator is
 *	running half as fast as real time, `time_factor` is 0.5. To tell
 *	libelec that the simulator is paused, set `time_factor` to 0.
 * @note Faster simulation rates simply speed up the rate at which libelec
 *	performs its physics calculations. This comes with an inherent CPU
 *	cost. If running real-time libelec's worker thread consumes 5% of
 *	one CPU core, then running it with `time_factor=10` will consume 50%
 *	of one CPU core. libelec cannot run faster than a single CPU core
 *	on the machine, imposing a hard cap on the maximum `time_factor`
 *	that can be effectively followed. If the CPU cannot keep up, libelec
 *	will appear to be lagging behind the desired simulation rate.
 */
void
libelec_sys_set_time_factor(elec_sys_t *sys, double time_factor)
{
	ASSERT(sys != NULL);
	ASSERT3F(time_factor, >=, 0);
#ifndef	XPLANE
	if (time_factor == 0) {
		/* Reset the worker interval to default */
		if (sys->started)
			worker_set_interval_nowake(&sys->worker, EXEC_INTVAL);
		mutex_enter(&sys->paused_lock);
		sys->paused = true;
		sys->time_factor = 0;
		mutex_exit(&sys->paused_lock);
		return;
	}
	/*
	 * If the time factor returns to 1.0x, we want to make sure we
	 * go back to 1.0x exactly, instead of some fractional 1.04x thing
	 * in case the sim was only ever so slightly accelerated/decelerated.
	 */
	if ((fabs(time_factor - sys->time_factor) > 0.1 ||
	    (time_factor == 1 && sys->time_factor != 1)) && sys->started) {
		worker_set_interval_nowake(&sys->worker,
		    EXEC_INTVAL / time_factor);
	}
	mutex_enter(&sys->paused_lock);
	sys->paused = false;
	sys->time_factor = time_factor;
	mutex_exit(&sys->paused_lock);
#endif	/* !defined(XPLANE) */
}

/**
 * @return The current simulation time factor.
 * @see libelec_sys_set_time_factor()
 */
double
libelec_sys_get_time_factor(const elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	return (sys->time_factor);
}

static void
elec_comp_serialize(elec_comp_t *comp, conf_t *ser, const char *prefix)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT(comp->info->name != NULL);
	ASSERT(ser != NULL);
	ASSERT(prefix != NULL);

	LIBELEC_SERIALIZE_DATA_V(comp, ser, "%s/%s/data",
	    prefix, comp->info->name);

	switch (comp->info->type) {
	case ELEC_BATT:
		LIBELEC_SERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/batt",
		    prefix, comp->info->name);
		break;
	case ELEC_GEN:
		LIBELEC_SERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/gen",
		    prefix, comp->info->name);
		break;
	case ELEC_LOAD:
		LIBELEC_SERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/load",
		    prefix, comp->info->name);
		break;
	case ELEC_CB:
		LIBELEC_SERIALIZE_DATA_V(&comp->scb, ser, "%s/%s/cb",
		    prefix, comp->info->name);
		break;
	case ELEC_SHUNT:
		/* Nothing to serialize for a shunt */
		break;
	case ELEC_TIE:
		mutex_enter(&comp->tie.lock);
		conf_set_data_v(ser, "%s/%s/cur_state", comp->tie.cur_state,
		    comp->n_links * sizeof (*comp->tie.cur_state),
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

	LIBELEC_DESERIALIZE_DATA_V(comp, ser, "%s/%s/data",
	    prefix, comp->info->name);

	switch (comp->info->type) {
	case ELEC_BATT:
		LIBELEC_DESERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/batt",
		    prefix, comp->info->name);
		break;
	case ELEC_GEN:
		LIBELEC_DESERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/gen",
		    prefix, comp->info->name);
		break;
	case ELEC_LOAD:
		LIBELEC_DESERIALIZE_DATA_V(&comp->batt, ser, "%s/%s/load",
		    prefix, comp->info->name);
		break;
	case ELEC_CB:
		LIBELEC_DESERIALIZE_DATA_V(&comp->scb, ser, "%s/%s/cb",
		    prefix, comp->info->name);
		break;
	case ELEC_SHUNT:
		/* Nothing to deserialize for a shunt */
		break;
	case ELEC_TIE:
		mutex_enter(&comp->tie.lock);
		if (conf_get_data_v(ser, "%s/%s/cur_state", comp->tie.cur_state,
		    comp->n_links * sizeof (*comp->tie.cur_state),
		    prefix, comp->info->name) !=
		    comp->n_links * sizeof (*comp->tie.cur_state)) {
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

/**
 * Serializes the run-time state of the network. You can use this to
 * implement aircraft state persistence. When saving your aircraft
 * state, call this function to save all electrical network state.
 * Use libelec_deserialize() to restore a previously serialized state.
 *
 * This function is safe to call after the network has been started
 * (in fact, that is its intended purpose - you don't have to stop
 * the network to perform state serializations). Since this function
 * can block while trying to interlock with the network's physics
 * worker thread, it is recommended not to call this from the simulator's
 * main thread (to prevent micro-stutter), but instead from a dedicated
 * state saving background thread.
 *
 * @param ser A libacfutils `conf_t` object, which will be filled
 *	with the serialized network state. This is the object which
 *	you should write to persistence storage to later reload.
 * @param prefix A name prefix which will be prepended to the
 *	configuration keys that will be placed into the `ser` `conf_t`
 *	object. This is to prevent namespace clashing with any other
 *	data you might be using that `conf_t` object for. A common
 *	name prefix would be something like `"libelec"`.
 */
void
libelec_serialize(elec_sys_t *sys, conf_t *ser, const char *prefix)
{
	ASSERT(sys != NULL);
	ASSERT(ser != NULL);
	ASSERT(prefix != NULL);
#ifdef	LIBELEC_WITH_NETLINK
	ASSERT(!sys->net_recv.active);
#endif
	conf_set_data_v(ser, "%s/conf_crc64", &sys->conf_crc,
	    sizeof (sys->conf_crc), prefix);

	mutex_enter(&sys->worker_interlock);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		elec_comp_serialize(comp, ser, prefix);
	}

	mutex_exit(&sys->worker_interlock);
}

/**
 * Deserializes a serialized network state previously saved using
 * libelec_serialize(). Before attempting deserialization, the library
 * compares the saved state to the currently running one. If there were
 * any alterations done to the electrical definition file, no attempt
 * is made to try to reload the serialized state and this function
 * aborts with an error.
 *
 * @param ser A libacfutils `conf_t` object, which was reloaded from
 *	persistent storage after having been previously populated with
 *	libelec_serialize().
 * @param prefix The same conf key prefix as was passed to
 *	libelec_serialize().
 *
 * @return True if the network state was reloaded successfully. If an
 *	error occurred, false is returned instead. The error reason is
 *	logged using libacfutils' logging facility.
 */
bool
libelec_deserialize(elec_sys_t *sys, const conf_t *ser, const char *prefix)
{
	uint64_t crc;

	ASSERT(sys != NULL);
	ASSERT(ser != NULL);
	ASSERT(prefix != NULL);
#ifdef	LIBELEC_WITH_NETLINK
	ASSERT(!sys->net_recv.active);
#endif
	if (!conf_get_data_v(ser, "%s/conf_crc64", &crc, sizeof (crc),
	    prefix)) {
		logMsg("Cannot deserialize libelec state: missing required "
		    "state key %s/conf_crc64", prefix);
		return (false);
	}
	if (crc != sys->conf_crc) {
		logMsg("Cannot deserialize libelec state: configuration "
		    "file CRC mismatch");
		return (false);
	}
	mutex_enter(&sys->worker_interlock);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		if (!elec_comp_deserialize(comp, ser, prefix)) {
			logMsg("Failed to deserialize %s: malformed state",
			    comp->info->name);
		}
	}

	mutex_exit(&sys->worker_interlock);

	return (true);
}

/**
 * Destroys a libelec network, which was previously loaded using
 * libelec_new(). You MUST stop the network using libelec_sys_stop()
 * before attempting to destroy it.
 */
void
libelec_destroy(elec_sys_t *sys)
{
	elec_comp_t *comp;
	user_cb_info_t *ucbi;
	void *cookie;

	ASSERT(sys != NULL);

	/* libelec_sys_stop MUST be called first! */
	ASSERT(!sys->started);

#ifdef	LIBELEC_WITH_NETLINK
	libelec_disable_net_send(sys);
	libelec_disable_net_recv(sys);
#endif	/* defined(LIBELEC_WITH_NETLINK) */

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

	while (list_remove_head(&sys->gens_batts) != NULL)
		;
	list_destroy(&sys->gens_batts);

	while (list_remove_head(&sys->ties) != NULL)
		;
	list_destroy(&sys->ties);

	while ((comp = list_remove_head(&sys->comps)) != NULL)
		comp_free(comp);
	list_destroy(&sys->comps);
	free(sys->comps_array);

	mutex_destroy(&sys->worker_interlock);
	mutex_destroy(&sys->paused_lock);

	infos_free(sys->comp_infos, sys->num_infos);

	free(sys->conf_filename);
#ifdef	XPLANE
	(void)XPLMUnregisterDrawCallback(elec_draw_cb,
	    xplm_Phase_Window, 0, sys);
#endif

	memset(sys, 0, sizeof (*sys));
	free(sys);
}

#ifdef	LIBELEC_WITH_LIBSWITCH

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
			comp->scb.sw = libswitch_add_toggle(name, desc,
			    anim_rate);
			/* Invert the CB so '0' is popped and '1' is pushed */
			libswitch_set_anim_offset(comp->scb.sw, -1, 1);
			libswitch_set(comp->scb.sw, 0);
			libswitch_button_set_turn_on_delay(comp->scb.sw,
			    CB_SW_ON_DELAY);
		}
	}
}

#endif	/* defined(LIBELEC_WITH_LIBSWITCH) */

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
	case ELEC_INV:
		return ("INV");
	case ELEC_XFRMR:
		return ("XFRMR");
	case ELEC_LOAD:
		return ("LOAD");
	case ELEC_BUS:
		return ("BUS");
	case ELEC_CB:
		return ("CB");
	case ELEC_SHUNT:
		return ("SHUNT");
	case ELEC_TIE:
		return ("TIE");
	case ELEC_DIODE:
		return ("DIODE");
	default:
		VERIFY(0);
	}
}

static bool
add_info_link(elec_comp_info_t *info, elec_comp_info_t *info2,
    const char *slot_qual)
{
	switch (info->type) {
	case ELEC_TRU:
	case ELEC_INV:
		if (slot_qual == NULL)
			return (false);
		if (strcmp(slot_qual, "AC") == 0) {
			if (info->tru.ac != NULL)
				return (false);
			info->tru.ac = info2;
		} else if (strcmp(slot_qual, "DC") == 0) {
			if (info->tru.dc != NULL)
				return (false);
			info->tru.dc = info2;
		} else {
			return (false);
		}
		break;
	case ELEC_XFRMR:
		if (slot_qual == NULL)
			return (false);
		if (strcmp(slot_qual, "IN") == 0) {
			if (info->xfrmr.input != NULL) {
				return (false);
			}
			info->xfrmr.input = info2;
		} else if (strcmp(slot_qual, "OUT") == 0) {
			if (info->xfrmr.output != NULL) {
				return (false);
			}
			info->xfrmr.output = info2;
		} else {
			return (false);
		}
		break;
	case ELEC_BUS:
		info->bus.comps = safe_realloc(info->bus.comps,
		    (info->bus.n_comps + 1) * sizeof (*info->bus.comps));
		info->bus.comps[info->bus.n_comps] = info2;
		info->bus.n_comps++;
		break;
	case ELEC_DIODE:
		if (slot_qual == NULL)
			return (false);
		if (strcmp(slot_qual, "IN") == 0) {
			if (info->diode.sides[0] != NULL)
				return (false);
			info->diode.sides[0] = info2;
		} else {
			if (strcmp(slot_qual, "OUT") != 0)
				return (false);
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

static gui_load_type_t
str2load_type(const char *str)
{
	ASSERT(str != NULL);
	if (strcmp(str, "MOTOR") == 0)
		return (GUI_LOAD_MOTOR);
	return (GUI_LOAD_GENERIC);
}

static elec_comp_info_t *
infos_parse(const char *filename, size_t *num_infos)
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
		    strncmp(line, "INV ", 4) == 0 ||
		    strncmp(line, "XFRMR ", 4) == 0 ||
		    strncmp(line, "LOAD ", 5) == 0 ||
		    strncmp(line, "BUS ", 4) == 0 ||
		    strncmp(line, "CB ", 3) == 0 ||
		    strncmp(line, "CB3 ", 4) == 0 ||
		    strncmp(line, "SHUNT ", 6) == 0 ||
		    strncmp(line, "TIE ", 4) == 0 ||
		    strncmp(line, "DIODE ", 6) == 0 ||
		    strncmp(line, "LABEL_BOX ", 10) == 0) {
			num_comps++;
		} else if (strncmp(line, "LOADCB ", 7) == 0 ||
		    strncmp(line, "LOADCB3 ", 8) == 0) {
			num_comps += 2;
		}
	}
	rewind(fp);

	infos = safe_calloc(num_comps, sizeof (*infos));
	for (size_t i = 0; i < num_comps; i++) {
		elec_comp_info_t *info = &infos[i];
		info->gui.pos = NULL_VECT2;
	}

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
#define	CHECK_DUP_NAME(__name__) \
	do { \
		elec_comp_info_t *info2 = find_comp_info(infos, comp_i, \
		    (__name__)); \
		if (info2 != NULL) { \
			logMsg("%s:%d: duplicate component name %s " \
			    "(previously found on line %d)", \
			    filename, linenum, (__name__), \
			    info2->parse_linenum); \
			free_strlist(comps, n_comps); \
			goto errout; \
		} \
	} while (0)
#define	CHECK_COMP(cond, reason) \
	do { \
		if (!(cond)) { \
			logMsg("%s:%d: %s", filename, linenum, (reason)); \
			free_strlist(comps, n_comps); \
			goto errout; \
		} \
	} while (0)
#define	CHECK_COMP_V(cond, reason, ...) \
	do { \
		if (!(cond)) { \
			logMsg("%s:%d: " reason, filename, linenum, \
			    __VA_ARGS__); \
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
			info->parse_linenum = linenum;
			info->type = ELEC_BATT;
			info->name = safe_strdup(comps[1]);
			info->int_R = 1;
		} else if (strcmp(cmd, "GEN") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_GEN;
			info->name = safe_strdup(comps[1]);
			info->int_R = 1;
		} else if (strcmp(cmd, "TRU") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_TRU;
			info->name = safe_strdup(comps[1]);
			info->int_R = 1;
		} else if (strcmp(cmd, "INV") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_INV;
			info->name = safe_strdup(comps[1]);
			info->int_R = 1;
		} else if (strcmp(cmd, "XFRMR") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_XFRMR;
			info->name = safe_strdup(comps[1]);
			info->int_R = 1;
		} else if (strcmp(cmd, "LOAD") == 0 &&
		    (n_comps == 2 || n_comps == 3)) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_LOAD;
			info->name = safe_strdup(comps[1]);
			if (n_comps == 3)
				info->load.ac = (strcmp(comps[2], "AC") == 0);
		} else if (strcmp(cmd, "BUS") == 0 && n_comps == 3) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_BUS;
			info->name = safe_strdup(comps[1]);
			info->bus.ac = (strcmp(comps[2], "AC") == 0);
			memset(bus_IDs_seen, 0, sizeof (bus_IDs_seen));
			bus_ID_cur = 0;
		} else if ((strcmp(cmd, "CB") == 0 ||
		    strcmp(cmd, "CB3") == 0) && n_comps == 3) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_CB;
			info->name = safe_strdup(comps[1]);
			info->cb.rate = 4;
			info->cb.max_amps = atof(comps[2]);
			info->cb.triphase = (strcmp(cmd, "CB3") == 0);
		} else if (strcmp(cmd, "SHUNT") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_SHUNT;
			info->name = safe_strdup(comps[1]);
		} else if (strcmp(cmd, "TIE") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_TIE;
			info->name = safe_strdup(comps[1]);
		} else if (strcmp(cmd, "DIODE") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_DIODE;
			info->name = safe_strdup(comps[1]);
		} else if (strcmp(cmd, "LABEL_BOX") == 0 && n_comps >= 7) {
			size_t sz = 0;
			ASSERT3U(comp_i, <, num_comps);
			info = &infos[comp_i++];
			info->parse_linenum = linenum;
			info->type = ELEC_LABEL_BOX;
			info->label_box.pos =
			    VECT2(atof(comps[1]), atof(comps[2]));
			info->label_box.sz =
			    VECT2(atof(comps[3]), atof(comps[4]));
			info->label_box.font_scale = atof(comps[5]);
			for (size_t i = 6; i < n_comps; i++) {
				append_format(&info->name, &sz, "%s%s",
				    comps[i], i + 1 < n_comps ? " " : "");
			}
		} else if (strcmp(cmd, "VOLTS") == 0 && n_comps == 2 &&
		    info != NULL) {
			if (info->type == ELEC_BATT) {
				info->batt.volts = atof(comps[1]);
				CHECK_COMP(info->batt.volts > 0,
				    "battery voltage must be positive");
			} else if (info->type == ELEC_GEN) {
				info->gen.volts = atof(comps[1]);
				CHECK_COMP(info->gen.volts > 0,
				    "generator voltage must be positive");
			} else {
				INVALID_LINE_FOR_COMP_TYPE;
			}
		} else if (strcmp(cmd, "FREQ") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			info->gen.freq = atof(comps[1]);
			CHECK_COMP(info->gen.freq >= 0,
			    "frequency must be a non-negative number");
		} else if (strcmp(cmd, "OUT_FREQ") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_INV) {
			info->tru.out_freq = atof(comps[1]);
			CHECK_COMP(info->tru.out_freq > 0,
			    "frequency must be a positive number");
		} else if (strcmp(cmd, "IN_VOLTS") == 0 && n_comps == 2 &&
		    info != NULL && (info->type == ELEC_TRU ||
		    info->type == ELEC_INV)) {
			info->tru.in_volts = atof(comps[1]);
		} else if (strcmp(cmd, "IN_VOLTS") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_XFRMR) {
			info->xfrmr.in_volts = atof(comps[1]);
		} else if (strcmp(cmd, "OUT_VOLTS") == 0 && n_comps == 2 &&
		    info != NULL && (info->type == ELEC_TRU ||
		    info->type == ELEC_INV)) {
			info->tru.out_volts = atof(comps[1]);
		} else if (strcmp(cmd, "OUT_VOLTS") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_XFRMR) {
			info->xfrmr.out_volts = atof(comps[1]);
		} else if (strcmp(cmd, "MIN_VOLTS") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_LOAD) {
			info->load.min_volts = atof(comps[1]);
		} else if (strcmp(cmd, "MIN_VOLTS") == 0 && n_comps == 2 &&
		    info != NULL && (info->type == ELEC_TRU ||
		    info->type == ELEC_INV)) {
			info->tru.min_volts = atof(comps[1]);
			CHECK_COMP(info->tru.min_volts < info->tru.out_volts,
			    "minimum voltage must be lower than nominal "
			    "output voltage");
		} else if (strcmp(cmd, "INCAP") == 0 &&
		    (n_comps == 3 || n_comps == 4) &&
		    info != NULL && info->type == ELEC_LOAD) {
			info->load.incap_C = atof(comps[1]);
			CHECK_COMP_V(info->load.incap_C > 0, "invalid input "
			    "capacitance %s: must be positive (in Farads)",
			    comps[1]);
			info->load.incap_R = atof(comps[2]);
			CHECK_COMP_V(info->load.incap_R > 0, "invalid input "
			    "capacitance internal resistance %s: must be "
			    "positive (in Ohms)", comps[2]);
			if (n_comps == 4) {
				info->load.incap_leak_Qps = atof(comps[3]);
			} else {
				info->load.incap_leak_Qps =
				    info->load.incap_C / 200;
			}
		} else if (strcmp(cmd, "CAPACITY") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_BATT) {
			info->batt.capacity = atof(comps[1]);
			CHECK_COMP(info->batt.capacity > 0, "battery CAPACITY "
			    "must be positive (in Watt-Hours)");
		} else if (strcmp(cmd, "STAB") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_LOAD) {
			info->load.stab = (strcmp(comps[1], "TRUE") == 0);
		} else if (strcmp(cmd, "STAB_RATE") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			info->gen.stab_rate_U = atof(comps[1]);
		} else if (strcmp(cmd, "STAB_RATE_F") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			CHECK_COMP(info->gen.freq != 0, "cannot define "
			    "frequency stabilization rate for DC generators, "
			    "or you must place the FREQ line before the "
			    "STAB_RATE_F line");
			info->gen.stab_rate_f = atof(comps[1]);
		} else if (strcmp(cmd, "EXC_RPM") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			info->gen.exc_rpm = atof(comps[1]);
			CHECK_COMP(info->gen.exc_rpm >= 0,
			    "excitation rpm must be non-negative");
		} else if (strcmp(cmd, "MIN_RPM") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			info->gen.min_rpm = atof(comps[1]);
			CHECK_COMP(info->gen.min_rpm > 0,
			    "generator MIN_RPM must be positive");
		} else if (strcmp(cmd, "MAX_RPM") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			info->gen.max_rpm = atof(comps[1]);
			CHECK_COMP(info->gen.max_rpm > 0,
			    "generator MAX_RPM must be positive");
		} else if (strcmp(cmd, "RATE") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_CB) {
			info->cb.rate = clamp(atof(comps[1]), 0.001, 1000);
		} else if (strcmp(cmd, "MAX_PWR") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_BATT) {
			info->batt.max_pwr = atof(comps[1]);
			CHECK_COMP(info->batt.max_pwr > 0,
			    "MAX_PWR must be positive (in Watts)");
		} else if (strcmp(cmd, "CURVEPT") == 0 && n_comps == 4 &&
		    info != NULL) {
			vect2_t **curve_pp;
			size_t num = 0;

			switch (info->type) {
			case ELEC_GEN:
				curve_pp = &info->gen.eff_curve;
				break;
			case ELEC_TRU:
			case ELEC_INV:
				curve_pp = &info->tru.eff_curve;
				break;
			case ELEC_XFRMR:
				curve_pp = &info->xfrmr.eff_curve;
				break;
			default:
				INVALID_LINE_FOR_COMP_TYPE;
				break;
			}
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
		} else if ((strcmp(cmd, "LOADCB") == 0 ||
		    strcmp(cmd, "LOADCB3") == 0) && (n_comps == 2 ||
		    n_comps == 3) && info != NULL && info->type == ELEC_LOAD) {
			elec_comp_info_t *cb, *bus;

			ASSERT3U(comp_i + 1, <, num_comps);
			cb = &infos[comp_i++];
			cb->parse_linenum = linenum;
			cb->type = ELEC_CB;
			cb->name = sprintf_alloc("CB_%s", info->name);
			cb->cb.rate = 1;
			cb->cb.max_amps = atof(comps[1]);
			cb->cb.triphase = (strcmp(cmd, "LOADCB3") == 0);
			cb->autogen = true;
			if (n_comps == 3) {
				strlcpy(cb->location, comps[2],
				    sizeof (cb->location));
			}

			bus = &infos[comp_i++];
			bus->parse_linenum = linenum;
			bus->type = ELEC_BUS;
			bus->name = sprintf_alloc("CB_BUS_%s", info->name);
			bus->bus.ac = info->load.ac;
			bus->autogen = true;

			VERIFY(add_info_link(bus, info, NULL));
			VERIFY(add_info_link(bus, cb, NULL));
			ASSERT_MSG(!cb->cb.triphase || info->load.ac,
			    "Can't connect 3-phase CB %s to a DC bus",
			    info->name);
		} else if (strcmp(cmd, "STD_LOAD") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_LOAD) {
			info->load.std_load = atof(comps[1]);
		} else if (strcmp(cmd, "FUSE") == 0 && n_comps == 1 &&
		    info != NULL && info->type == ELEC_CB) {
			info->cb.fuse = true;
		} else if (strcmp(cmd, "GUI_POS") == 0 && (n_comps == 3 ||
		    n_comps == 4) && info != NULL) {
			info->gui.pos = VECT2(atof(comps[1]), atof(comps[2]));
			if (n_comps == 4)
				info->gui.sz = atof(comps[3]);
			else
				info->gui.sz = 1;
		} else if (strcmp(cmd, "GUI_ROT") == 0 && n_comps == 2 &&
		    info != NULL) {
			info->gui.rot = atof(comps[1]);
		} else if (strcmp(cmd, "GUI_LOAD") == 0 && n_comps == 2 &&
		    info != NULL) {
			info->gui.load_type = str2load_type(comps[1]);
		} else if (strcmp(cmd, "GUI_VIRT") == 0 && n_comps == 1 &&
		    info != NULL) {
			info->gui.virt = true;
		} else if (strcmp(cmd, "GUI_INVIS") == 0 && n_comps == 1 &&
		    info != NULL) {
			info->gui.invis = true;
		} else if (strcmp(cmd, "GUI_COLOR") == 0 && n_comps == 4 &&
		    info != NULL) {
			info->gui.color = VECT3(atof(comps[1]),
			    atof(comps[2]), atof(comps[3]));
		} else if (strcmp(cmd, "CHGR_BATT") == 0 && n_comps == 4 &&
		    info != NULL && info->type == ELEC_TRU) {
			info->tru.charger = true;
			info->tru.batt =
			    find_comp_info(infos, num_comps, comps[1]);
			if (info->tru.batt == NULL) {
				logMsg("%s:%d: unknown component %s",
				    filename, linenum, comps[1]);
				free_strlist(comps, n_comps);
				goto errout;
			}
			info->tru.batt_conn =
			    find_comp_info(infos, num_comps, comps[2]);
			if (info->tru.batt_conn == NULL) {
				logMsg("%s:%d: unknown component %s",
				    filename, linenum, comps[1]);
				free_strlist(comps, n_comps);
				goto errout;
			}
			info->tru.curr_lim = atof(comps[3]);
			if (info->tru.curr_lim <= 0) {
				logMsg("%s:%d: current limit must be positive",
				    filename, linenum);
				free_strlist(comps, n_comps);
				goto errout;
			}
		} else if (strcmp(cmd, "CHG_R") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_BATT) {
			info->batt.chg_R = atof(comps[1]);
			CHECK_COMP(info->batt.chg_R > 0,
			    "charge resistance must be positive");
		} else if (strcmp(cmd, "INT_R") == 0 && n_comps == 2 &&
		    info != NULL && (info->type == ELEC_BATT ||
		    info->type == ELEC_GEN || info->type == ELEC_TRU ||
		    info->type == ELEC_INV || info->type == ELEC_XFRMR)) {
			info->int_R = atof(comps[1]);
			CHECK_COMP(info->int_R > 0,
			    "internal resistance must be positive");
		} else if (strcmp(cmd, "LOCATION") == 0 && n_comps == 2 &&
		    info != NULL) {
			strlcpy(info->location, comps[1],
			    sizeof (info->location));
		} else {
			logMsg("%s:%d: unknown or malformed line",
			    filename, linenum);
			free_strlist(comps, n_comps);
			goto errout;
		}
		free_strlist(comps, n_comps);
	}

	if (!validate_elec_comp_infos_parse(infos, num_comps, filename))
		goto errout;

#undef	INVALID_LINE_FOR_COMP_TYPE
#undef	CHECK_DUP_NAME
#undef	CHECK_COMP
#undef	CHECK_COMP_V

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

static void
infos_free(elec_comp_info_t *infos, size_t num_infos)
{
	ASSERT(infos != NULL || num_infos == 0);
	for (size_t i = 0; i < num_infos; i++) {
		elec_comp_info_t *info = &infos[i];

		free(info->name);
		if (info->type == ELEC_GEN)
			free(info->gen.eff_curve);
		else if (info->type == ELEC_TRU || info->type == ELEC_INV)
			free(info->tru.eff_curve);
		else if (info->type == ELEC_XFRMR)
			free(info->xfrmr.eff_curve);
		else if (info->type == ELEC_BUS)
			ZERO_FREE_N(info->bus.comps, info->bus.n_comps);
	}
	ZERO_FREE_N(infos, num_infos);
}

/**
 * Adds a custom user callback to the library. This will be called from
 * the physics calculation thread, allowing you perform precise accounting
 * of all physics state as it's being calculated.
 * @param pre If set to true, the callback will be called just before
 *	the physics integration is run. If set to false, will be called
 *	immediately after the physics integration.
 * @param cb The callback to be called.
 * @param userinfo A custom userinfo pointer parameter, which will be
 *	passed to the callback function every time it is called.
 * @note You must NOT register the exact same pre + callback + userinfo
 *	pointer combo more than once.
 * @see libelec_remove_user_cb()
 */
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

/**
 * Removes a custom user callback, which was previously registered
 * using libelec_add_user_cb(). You must pass the exact same combination
 * of `pre`, `cb` and `userinfo` as was previously used during the
 * registration. The callback MUST exist, otherwise an assertion failure
 * is triggered.
 * @see libelec_add_user_cb()
 */
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

	ZERO_FREE(info);
}

/**
 * Walker function, which will go through all components on the network.
 * This is mostly used for debugging.
 * @param cb A callback, which you must supply and will be called in
 *	turn with every electrical component on the network in its
 *	first argument.
 * @param userinfo An optional custom pointer parameter. This will be
 *	passed in the second argument to the `cb` callback you supply
 *	above.
 */
void
libelec_walk_comps(const elec_sys_t *sys, void (*cb)(elec_comp_t *, void*),
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

/**
 * @return The \ref elec_comp_info_t in use by a particular electrical
 *	component in a network. This function never fails, since every
 *	electrical component MUST have an associated info structure.
 */
const elec_comp_info_t *
libelec_comp2info(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	/* immutable binding */
	return (comp->info);
}

/**
 * Locates an electrical component on the network by name.
 * @param name The name of the component to find.
 * @return If found on the network, a pointer to the electrical
 *	component. You can then interrogate this component using
 *	other libelec functions. If no component of the specified
 *	name exists, returns NULL instead.
 */
elec_comp_t *
libelec_comp_find(elec_sys_t *sys, const char *name)
{
	elec_comp_info_t srch_info = { .name = (char *)name };
	const elec_comp_t srch_comp = { .info = &srch_info };

	ASSERT(sys != NULL);
	/* Component list is immutable, no need to lock */
	return (avl_find(&sys->name2comp, &srch_comp, NULL));
}

/**
 * @return The number of connections of a particular component. This
 *	can be used to dynamically figure out the network structure.
 *	You can then retrieve individual connected components using
 *	libelec_comp_get_conn().
 */
size_t
libelec_comp_get_num_conns(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);

	/* Electrical configuration is immutable, no need to lock */
	switch (comp->info->type) {
	case ELEC_BUS:
		return (comp->n_links);
	case ELEC_TIE:
		return (comp->n_links);
	case ELEC_TRU:
	case ELEC_INV:
	case ELEC_XFRMR:
	case ELEC_CB:
	case ELEC_SHUNT:
	case ELEC_DIODE:
		return (2);
	default:
		return (1);
	}
}

/**
 * @return The electrical component which is connected to this component
 *	using connection `i`.
 * @param i The index of the connection. This MUST be less than the
 *	return value of libelec_comp_get_num_conns().
 */
elec_comp_t *
libelec_comp_get_conn(const elec_comp_t *comp, size_t i)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(i, <, comp->n_links);
	/* Electrical configuration is immutable, no need to lock */
	return (comp->links[i].comp);
}

/**
 * @return The input voltage of the component. While every component
 *	has a notion of an input and output voltage, their physical
 *	meaning changes depending on component type:
 *	- \ref ELEC_BATT : the charging voltage (if any)
 *	- \ref ELEC_TRU : the input voltage on the AC side of the TRU
 *	- \ref ELEC_INV : the input voltage on the DC side of the inverter
 *	- \ref ELEC_XFRMR : the input voltage to the transformer
 *	- \ref ELEC_LOAD : the input voltage into the power supply of the
 *		load. This will immediately follow the bus voltage
 *		to which the load is attached. Please note that simply
 *		seeing the voltage drop to zero doesn't mean the load
 *		immediately becomes unpowered. If the load has a
 *		power supply with non-zero input capacitance, its
 *		output voltage out of this power supply will decay
 *		more slowly, allowing the load to "bridge" short
 *		periods of loss of input power (e.g. during bus
 *		contactor switching).
 *	- \ref ELEC_GEN, \ref ELEC_BUS, \ref ELEC_CB, \ref ELEC_SHUNT and
 *		\ref ELEC_TIE :
 *		always equal to the output voltage
 *	- \ref ELEC_DIODE : voltage at the input side of the diode. If the
 *		input voltage of any source on the input side of the
 *		diode is higher than that of any source on the output
 *		side, the diode allows current flow from input to output.
 *		Otherwise, the diode blocks current flow.
 * @see libelec_comp_get_out_volts()
 */
double
libelec_comp_get_in_volts(const elec_comp_t *comp)
{
	double volts;

	ASSERT(comp != NULL);

	NET_ADD_RECV_COMP(comp);
	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	volts = comp->ro.in_volts;
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (volts);
}

/**
 * @return The Output voltage of the component. While every component
 *	has a notion of an input and output voltage, their physical
 *	meaning changes depending on component type:
 *	- \ref ELEC_BATT : the voltage the battery puts on the connected
 *		bus and which can be used by downstream components to
 *		extract energy out of the battery.
 *	- \ref ELEC_TRU : the output voltage on the DC side of the TRU
 *	- \ref ELEC_INV : the output voltage on the AC side of the inverter
 *	- \ref ELEC_XFRMR : the output voltage of the transformer
 *	- \ref ELEC_LOAD : the output voltage out of the power supply of
 *		the load. If the load's power supply has no input
 *		capacitance, this will immediately follow the input
 *		voltage from the bus the load is attached to. However,
 *		if the load's power supply has non-zero input
 *		capacitance, the output voltage out of this power
 *		supply will decay more slowly, allowing the load to
 *		"bridge" short periods of loss of input power (e.g.
 *		during bus contactor switching).
 *	- \ref ELEC_GEN, \ref ELEC_BUS, \ref ELEC_CB, \ref ELEC_SHUNT and
 *		\ref ELEC_TIE :
 *		always equal to the input voltage
 *	- \ref ELEC_DIODE : voltage at the output side of the diode. If the
 *		input voltage of any source on the input side of the
 *		diode is higher than that of any source on the output
 *		side, the diode allows current flow from input to output.
 *		Otherwise, the diode blocks current flow.
 * @see libelec_comp_get_in_volts()
 */
double
libelec_comp_get_out_volts(const elec_comp_t *comp)
{
	double volts;

	ASSERT(comp != NULL);

	NET_ADD_RECV_COMP(comp);
	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	volts = comp->ro.out_volts;
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (volts);
}

/**
 * @return The input current of the component. While every component
 *	has a notion of an input and output current, their physical
 *	meaning changes depending on component type:
 *	- \ref ELEC_BATT : the charging current (if any)
 *	- \ref ELEC_TRU : the input current on the AC side of the TRU
 *	- \ref ELEC_INV : the input current on the DC side of the inverter
 *	- \ref ELEC_XFRMR : the input current to the transformer
 *	- \ref ELEC_LOAD : the input current into the power supply of the
 *		load. If the load has no input capacitance in its power
 *		supply, this will always be equal to the actual current
 *		consumption of the load. However, with input capacitance,
 *		some input current is also generated by the charging of
 *		this input capacitance. The inrush current can actually
 *		be quite significant, so you should balance the charging
 *		resistance of the power supply carefully against the
 *		breaker ratings on the bus.
 *	- \ref ELEC_GEN : input current is an approximation of the mechanical
 *		load (but expressed in electrical terms as a fake current),
 *		which the generator puts on its input drive, accounting
 *		for generator efficiency losses. Simply put, the input
 *		current is equal to the output current divided by the
 *		generator's efficiency.
 *	- \ref ELEC_BUS, \ref ELEC_CB, \ref ELEC_SHUNT, \ref ELEC_TIE and
 *		\ref ELEC_DIODE :
 *		always equal to the output current. These components are
 *		assumed to have zero resistance, thus resulting in no
 *		energy loss for current passing through them.
 * @note Input current demand also accounts for leakage current due to
 *	a simulated short-circuit in the component.
 * @see libelec_comp_get_out_amps()
 */
double
libelec_comp_get_in_amps(const elec_comp_t *comp)
{
	double amps;

	ASSERT(comp != NULL);

	NET_ADD_RECV_COMP(comp);
	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	amps = comp->ro.in_amps * (1 - comp->ro.leak_factor);
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (amps);
}

/**
 * @return The output current of the component. While every component
 *	has a notion of an input and output current, their physical
 *	meaning changes depending on component type:
 *	- \ref ELEC_BATT : the discharging current (if any)
 *	- \ref ELEC_TRU : the output current on the DC side of the TRU
 *	- \ref ELEC_INV : the output current on the AC side of the inverter
 *	- \ref ELEC_XFRMR : the output current out of the transformer
 *	- \ref ELEC_LOAD : the output current out of the power supply of the
 *		load into its internal working components.
 *	- \ref ELEC_GEN : output current demand on the generator. This is
 *		dictated by the current demands of all downstream
 *		components powered by the generator.
 *	- \ref ELEC_BUS, \ref ELEC_CB, \ref ELEC_SHUNT , \ref ELEC_TIE and
 *		\ref ELEC_DIODE :
 *		always equal to the input current. These components are
 *		assumed to have zero resistance, thus resulting in no
 *		energy loss for current passing through them.
 * @note Output current demand also accounts for leakage current due to
 *	a simulated short-circuit in the component.
 * @see libelec_comp_get_in_amps()
 */
double
libelec_comp_get_out_amps(const elec_comp_t *comp)
{
	double amps;

	ASSERT(comp != NULL);

	NET_ADD_RECV_COMP(comp);
	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	amps = comp->ro.out_amps * (1 - comp->ro.leak_factor);
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (amps);
}

/**
 * @return The input power demand of the component.
 * @note libelec has no notion of apparent power, power factor or
 *	cos-Phi. All power flows are assumed to be completely resistive.
 *	As such, input power is simply the product of input voltage
 *	and input current.
 * @return While every component has a notion of an input and output
 *	power, their physical meaning changes depending on component
 *	type:
 *	- \ref ELEC_BATT : the charging power flow (if any)
 *	- \ref ELEC_TRU : the input power on the AC side of the TRU. This
 *		will be higher than the output power on the DC side
 *		because of the efficiency losses during rectification
 *		and voltage transformation inside of the TRU.
 *	- \ref ELEC_INV : the input power on the DC side of the inverter.
 *		This will be higher than the output power on the AC side
 *		because of the efficiency losses during inversion and
 *		voltage transformation inside of the inverter.
 *	- \ref ELEC_XFRMR : the input power into the transformer. This
 *		will be higher than the output power because of the
 *		efficiency losses during voltage transformation.
 *	- \ref ELEC_LOAD : the input power into the power supply of the load.
 *	- \ref ELEC_GEN : the mechanical load which the generator causes on
 *		its input shaft. You can use this to account for
 *		generator-induced engine drag. The input power will always
 *		be greater than the generator's electrical output power,
 *		because the generator has a sub-100% efficiency during
 *		electrical power generation.
 *	- \ref ELEC_BUS, \ref ELEC_CB , \ref ELEC_SHUNT, \ref ELEC_TIE and
 *		\ref ELEC_DIODE :
 *		always equal to the output power. These components are
 *		assumed to have zero resistance, thus resulting in no
 *		energy loss for power passing through them.
 * @see libelec_comp_get_out_pwr()
 */
double
libelec_comp_get_in_pwr(const elec_comp_t *comp)
{
	double watts;

	ASSERT(comp != NULL);

	NET_ADD_RECV_COMP(comp);
	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	watts = comp->ro.in_pwr * (1 - comp->ro.leak_factor);
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (watts);
}

/**
 * @return The output power demand flow out of the component.
 * @note libelec has no notion of apparent power, power factor or
 *	cos-Phi. All power flows are assumed to be completely resistive.
 *	As such, output power is simply the product of output voltage
 *	and output current.
 * @return While every component has a notion of an input and output
 *	power, their physical meaning changes depending on component
 *	type:
 *	- \ref ELEC_BATT : the discharging power flow out of the battery.
 *		Note that libelec doesn't simulate battery self-discharge.
 *		You will want to implement that yourself, by slowly
 *		reducing the battery's state of charge (using
 *		libelec_batt_set_chg_rel()) over time, according to your
 *		battery's behavioral characteristics.
 *	- \ref ELEC_TRU : the output power on the DC side of the TRU.
 *		This will be lower than the input power on the AC side
 *		because of the efficiency losses during rectification
 *		and voltage transformation inside of the TRU.
 *	- \ref ELEC_INV : the output power on the AC side of the inverter.
 *		This will be lower than the input power on the DC side
 *		because of the efficiency losses during inversion and
 *		voltage transformation inside of the inverter.
 *	- \ref ELEC_XFRMR : the output power out of the transformer. This
 *		will be lower than the input power because of efficiency
 *		losses during voltage transformation.
 *	- \ref ELEC_LOAD : the output power out of the load's power supply
 *		and into the load's internal electronics. If you see this
 *		drop to zero, you can be sure that the load is no longer
 *		powered (you should use libelec_comp_is_powered() for
 *		components which have a `MIN_VOLTS` value defined, as that
 *		communicates intent better than simply checking the output
 *		power).
 *	- \ref ELEC_GEN : the electrical power load on the generator, caused
 *		by all of its downstream loads. Note that this wiill always
 *		be lower than the input power load on the mechanical input
 *		of the generator due to efficiency losses.
 *	- \ref ELEC_BUS, \ref ELEC_CB, \ref ELEC_SHUNT, \ref ELEC_TIE and
 *		\ref ELEC_DIODE :
 *		always equal to the input power. These components are
 *		assumed to have zero resistance, thus resulting in no
 *		energy loss for power passing through them.
 * @see libelec_comp_get_in_pwr()
 */
double
libelec_comp_get_out_pwr(const elec_comp_t *comp)
{
	double watts;

	ASSERT(comp != NULL);

	NET_ADD_RECV_COMP(comp);
	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	watts = comp->ro.out_pwr * (1 - comp->ro.leak_factor);
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (watts);
}

/**
 * @return The input frequency of the component. While every component has
 * a notion of an input and output frequency, their physical meaning changes
 * depending on component type:
 *	- \ref ELEC_BATT and \ref ELEC_DIODE : these are DC-only devices,
 *		so this will always be zero.
 *	- \ref ELEC_TRU : the AC frequency on the AC side input of the TRU.
 *	- \ref ELEC_INV : always zero, since the inverter input is DC.
 *	- \ref ELEC_XFRMR : the frequency of the AC current flowing through
 *		the transformer. Both input and output frequency will always
 *		be equal to each other.
 *	- \ref ELEC_LOAD : the AC frequency of the input bus into the load's
 *		power supply. If the load is DC, this will always be zero.
 *	- \ref ELEC_GEN : always equal to the output frequency (see
 *		libelec_comp_get_out_freq()).
 *	- \ref ELEC_BUS, \ref ELEC_CB, \ref ELEC_SHUNT and \ref ELEC_TIE :
 *		always equal to the output frequency. These components
 *		do not do anything with AC frequency, and so they simply
 *		pass the parameter through.
 */
double
libelec_comp_get_in_freq(const elec_comp_t *comp)
{
	double freq;

	ASSERT(comp != NULL);

	NET_ADD_RECV_COMP(comp);
	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	freq = comp->ro.in_freq;
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (freq);
}

/**
 * @return The output frequency of the component. While every component has
 * a notion of an input and output frequency, their physical meaning changes
 * depending on component type:
 *	- \ref ELEC_BATT and \ref ELEC_DIODE : these are DC-only devices,
 *		so this will always be zero.
 *	- \ref ELEC_TRU : always zero, since the TRU performs AC-to-DC
 *		rectification.
 *	- \ref ELEC_INV : the AC frequency generated by the inverter.
 *	- \ref ELEC_XFRMR : the frequency of the AC current flowing through
 *		the transformer. Both input and output frequency will always
 *		be equal to each other.
 *	- \ref ELEC_LOAD : the AC frequency of the input bus into the load's
 *		power supply. If the load is DC, this will always be zero.
 *		If the load is unpowered, this remains at the last value
 *		to which it was set. You should probably not really rely
 *		on this value to do anything sensible.
 *	- \ref ELEC_GEN : for AC generators, this is the output frequency of
 *		the generator. For DC generators, this is always zero. AC
 *		generator frequency behavior depends on the input rpm and
 *		stabilization limits defined for the generator in the
 *		network definition file.
 *	- \ref ELEC_BUS, \ref ELEC_CB, \ref ELEC_SHUNT and \ref ELEC_TIE :
 *		always equal to the input frequency. These components do
 *		not do anything with AC frequency, and so they simply
 *		pass the parameter through.
 */
double
libelec_comp_get_out_freq(const elec_comp_t *comp)
{
	double freq;

	ASSERT(comp != NULL);

	NET_ADD_RECV_COMP(comp);
	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	freq = comp->ro.out_freq;
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	return (freq);
}

/**
 * @param comp The component for which to return the input capacitance
 *	voltage. This MUST be a component of type \ref ELEC_LOAD.
 * @return The voltage of the internal power supply imaginary "capacitor"
 *	used to model power supply input capacitance. This can be used to
 *	estimate the "state of charge" of the power supply's capacitance.
 *	If the input voltage into the load is higher than this capacitor
 *	voltage, the imaginary capacitor will begin to charge up. If the
 *	input voltage is lower than the power supply capacitance voltage,
 *	the capacitance will begin to "make up the difference" to the
 *	desired energy flow to the load's electrical demand. Loads with
 *	no power supply capacitance always return 0.
 */
double
libelec_comp_get_incap_volts(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_LOAD);
	return (comp->load.incap_U);
}

/**
 * @return Depending on component type:
 *	- \ref ELEC_BATT and \ref ELEC_DIODE : always return false
 *	- \ref ELEC_TRU, \ref ELEC_INV and \ref ELEC_XFRMR : always returns true
 *	- Any other component type will return true if is an AC component,
 *		and false if it is a DC component.
 */
bool
libelec_comp_is_AC(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	switch (comp->info->type) {
	case ELEC_BATT:
	case ELEC_DIODE:
		return (false);
	case ELEC_TRU:
	case ELEC_INV:
	case ELEC_XFRMR:
		return (true);
	case ELEC_GEN:
		return (comp->info->gen.freq != 0);
	case ELEC_LOAD:
		ASSERT(comp->links[0].comp != NULL);
		return (comp->links[0].comp->info->bus.ac);
	case ELEC_BUS:
		return (comp->info->bus.ac);
	case ELEC_CB:
	case ELEC_SHUNT:
		ASSERT(comp->links[0].comp != 0);
		return (comp->links[0].comp->info->bus.ac);
		break;
	case ELEC_TIE:
		ASSERT(comp->n_links != 0);
		ASSERT(comp->links[0].comp != NULL);
		return (comp->links[0].comp->info->bus.ac);
	case ELEC_LABEL_BOX:
		VERIFY_FAIL();
	}
	VERIFY_FAIL();
}

elec_comp_type_t
libelec_comp_get_type(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->info->type);
}

const char *
libelec_comp_get_name(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->info->name);
}

const char *
libelec_comp_get_location(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->info->location);
}

bool
libelec_comp_get_autogen(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->info->autogen);
}

/**
 * @return True if the component is powered, false if it isn't. A component
 * is considered powered if its output voltage is greater than zero. For
 * most component this requires that the component be connected to a
 * suitable energy source.
 *
 * @return Components of type \ref ELEC_LOAD get special treatment.
 *	- If the input voltage into the load is insufficient, but the
 *	  component's power supply has some input capacitance, the residual
 *	  capacitance of the power supply will serve to briefly power the
 *	  load's power demand. As such, libelec_comp_is_powered() might
 *	  actually return true briefly, even if there is no power generator
 *	  or battery connected to the load.
 *	- Loads which declare `MIN_VOLTS` stanza will be considered
 *	  unpowered if their power supply output voltage drops below
 *	  the `MIN_VOLTS` requirement.
 */
bool
libelec_comp_is_powered(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (libelec_comp_get_out_volts(comp) != 0);
}

/**
 * @return The current efficiency of the component `comp`. This MUST be a
 *	component of type \ref ELEC_GEN, \ref ELEC_TRU, \ref ELEC_INV or
 *	\ref ELEC_XFRMR.
 */
double
libelec_comp_get_eff(const elec_comp_t *comp)
{
	double eff;
	ASSERT(comp != NULL);
	switch (comp->info->type) {
	case ELEC_GEN:
		mutex_enter(&((elec_comp_t *)comp)->gen.lock);
		eff = comp->gen.eff;
		mutex_exit(&((elec_comp_t *)comp)->gen.lock);
		break;
	case ELEC_TRU:
	case ELEC_INV:
		eff = comp->tru.eff;
		break;
	case ELEC_XFRMR:
		eff = comp->xfrmr.eff;
		break;
	default:
		VERIFY_FAIL();
	}
	return (eff);
}

unsigned
libelec_comp_get_srcs(const elec_comp_t *comp,
    elec_comp_t *srcs[CONST_ARRAY_LEN_ARG(ELEC_MAX_SRCS)])
{
	ASSERT(comp != NULL);
	ASSERT(srcs != NULL);

	mutex_enter((mutex_t *)&comp->rw_ro_lock);
	memcpy(srcs, comp->srcs_ext, sizeof (elec_comp_t *) * ELEC_MAX_SRCS);
	mutex_exit((mutex_t *)&comp->rw_ro_lock);

	for (unsigned i = 0; i < ELEC_MAX_SRCS; i++) {
		if (srcs[i] == NULL)
			return (i);
	}
	return (ELEC_MAX_SRCS);
}

/**
 * Sets a component's failed status. The behavior of a failed component
 * depends on its type:
 *	- `ELEC_BATT`: provides no output voltage and cannot be charged.
 *	- `ELEC_DIODE`, `ELEC_BUS`, `ELEC_CB`, `ELEC_SHUNT` and `ELEC_TIE`:
 *		doesn't allow any current to pass through the component.
 *	- `ELEC_TRU`: generates no electrical load on its AC input and
 *		provides no voltage on its DC output.
 *	- `ELEC_INV`: generates no electrical load on its DC input and
 *		provides no voltage on its AC output.
 *	- `ELEC_XFRMR`: generates no electrical load on its input and
 *		provides no voltage on its output.
 *	- `ELEC_GEN`: generates no voltage on its output and requires no
 *		mechanical input power.
 *	- `ELEC_LOAD`: draws no electrical current and is permanently
 *		unpowered (libelec_comp_is_powered() always returns false).
 * @see libelec_comp_get_failed()
 */
void
libelec_comp_set_failed(elec_comp_t *comp, bool failed)
{
	ASSERT(comp != NULL);
	mutex_enter(&comp->rw_ro_lock);
	comp->ro.failed = failed;
	mutex_exit(&comp->rw_ro_lock);
}

/**
 * @return True if the component is set to failed, false if is not.
 * @see libelec_comp_set_failed()
 */
bool
libelec_comp_get_failed(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	NET_ADD_RECV_COMP(comp);
	return (comp->ro.failed);
}

/**
 * Sets a component's short-circuit failure status. Short circuits are
 * simulated by creating a random massive increase in a leakage current
 * entering the component. This energy is not directed to any other
 * connected components, or accounted for in a load's output power, but
 * is rather "lost" to ground. The amount of leakage current is
 * randomized and depends on the component's normal input current flow.
 * It will typically oscillate somewhere between 30-40x the normal
 * current. This should guarantee that any breaker protecting this
 * device will trip in fairly short order.
 * @see libelec_comp_get_shorted()
 */
void
libelec_comp_set_shorted(elec_comp_t *comp, bool shorted)
{
	ASSERT(comp != NULL);
	mutex_enter(&comp->rw_ro_lock);
	comp->ro.shorted = shorted;
	mutex_exit(&comp->rw_ro_lock);
}

/**
 * @return True if the component's short-circuit failure is set,
 *	or false if it is not.
 * @see libelec_comp_set_shorted()
 */
bool
libelec_comp_get_shorted(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	NET_ADD_RECV_COMP(comp);
	return (comp->ro.shorted);
}

static double
gen_set_random_param(elec_comp_t *comp, double *param, double norm_value,
    double stddev)
{
	double new_param;

	ASSERT(comp != NULL);
	ASSERT(param != NULL);
	ASSERT3F(stddev, >=, 0);

	if (stddev != 0) {
		new_param = norm_value + crc64_rand_normal(stddev);
		/*
		 * Make sure the error is at least 0.5 standard deviations
		 * and at most 1.5 standard deviations. This is to guarantee
		 * that at least something is observed by the user.
		 */
		if (new_param > norm_value) {
			new_param = clamp(new_param, norm_value + 0.5 * stddev,
			    norm_value + 1.5 * stddev);
		} else {
			new_param = clamp(new_param, norm_value - 1.5 * stddev,
			    norm_value - 0.5 * stddev);
		}
	} else {
		new_param = norm_value;
	}
	mutex_enter(&comp->sys->worker_interlock);
	*param = norm_value;
	mutex_exit(&comp->sys->worker_interlock);

	return (new_param);
}

/**
 * Sets up a random under- or over-voltage failure condition on a generator.
 * @param comp The generator for which to set the failure up. This MUST
 *	be a component of type \ref ELEC_GEN.
 * @param stddev Standard deviation of the error. The library will use a
 *	Gaussian random number generator to pick a value which is at least
 *	0.5x stddev and at most 1.5x stddev away from the nominal voltage
 *	of the generator. The standard deviation MUST be less than 66% of
 *	the nominal voltage (to guarantee that the random voltage value
 *	doesn't go to zero or negative). For example, a good value for a
 *	115V generator would be something like 30V. This would generate
 *	a random voltage deviation of 15-45V from the nominal (randomly
 *	selected to either be an over-voltage or under-voltage condition).
 *	To disable the failure, simply set `stddev` to zero.
 * @return The new random voltage target of the generator. If `stddev`
 *	was zero, returns the nominal voltage of the generator.
 * @see libelec_gen_set_random_freq()
 */
double
libelec_gen_set_random_volts(elec_comp_t *comp, double stddev)
{
	ASSERT(comp != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_GEN);
	ASSERT3F(1.5 * stddev, <, comp->info->gen.volts);
	return (gen_set_random_param(comp, &comp->gen.tgt_volts,
	    comp->info->gen.volts, stddev));
}

/**
 * Same as libelec_gen_set_random_volts(), but operates on an AC
 * generator's frequency stability.
 * @see libelec_gen_set_random_volts()
 */
double
libelec_gen_set_random_freq(elec_comp_t *comp, double stddev)
{
	ASSERT(comp != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_GEN);
	ASSERT(libelec_comp_is_AC(comp));
	ASSERT3F(comp->info->gen.freq - 1.5 * stddev, >, 0);
	return (gen_set_random_param(comp, &comp->gen.tgt_freq,
	    comp->info->gen.freq, stddev));
}

/**
 * Sets the userinfo pointer of a component. This userinfo pointer is
 * passed to all callbacks (such as those set up using
 * libelec_batt_set_temp_cb() or ibelec_gen_set_rpm_cb()) in the second
 * argument of the callback. You can use this information to pass along
 * any extra information to the callbacks that you may need.
 */
void
libelec_comp_set_userinfo(elec_comp_t *comp, void *userinfo)
{
	ASSERT(comp != NULL);
	ASSERT(!comp->sys->started);
	comp->info->userinfo = userinfo;
}

/**
 * @return The userinfo pointer previously passed to
 *	libelec_comp_set_userinfo(). If no userinfo pointer was ever
 *	set, returns `NULL`.
 */
void *
libelec_comp_get_userinfo(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (comp->info->userinfo);
}

/**
 * Sets the battery temperature callback. libelec doesn't itself
 * implement battery heat flow. Rather, it relies on external code
 * to tell it what temperature the battery is. libelec then employs
 * that value to determine the battery's electrical properties
 * (such as voltage, max current, capacity, etc.)
 * @note You must NOT call this function after the network has been
 *	started by a call to libelec_sys_start(). The network MUST
 *	be stopped before attempting to reconfigure it.
 * @param batt The battery for which to configure the callback.
 *	Must be a component of type \ref ELEC_BATT.
 * @param cb The callback to set. You may pass `NULL` here to remove
 *	the callback. You may also use libelec_batt_set_temp() to set
 *	the battery's temperature directly.
 * @see elec_get_temp_cb_t
 */
void
libelec_batt_set_temp_cb(elec_comp_t *batt, elec_get_temp_cb_t cb)
{
	ASSERT(batt != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);
	ASSERT(!batt->sys->started);
	batt->info->batt.get_temp = cb;
}

/**
 * @return The battery temperature callback, which was previously set
 *	using libelec_batt_set_temp_cb(), or `NULL` if no callback
 *	was set.
 * @see libelec_batt_set_temp_cb()
 */
elec_get_temp_cb_t
libelec_batt_get_temp_cb(const elec_comp_t *batt)
{
	ASSERT(batt != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);
	return (batt->info->batt.get_temp);
}

/**
 * Sets the generator rpm callback. libelec uses this callback to
 * determine the rotational speed of the generator's input shaft,
 * which it then uses to calculate the output voltage and frequency.
 * @note You must NOT call this function after the network has been
 *	started by a call to libelec_sys_start(). The network MUST
 *	be stopped before attempting to reconfigure it.
 * @param batt The generator for which to configure the callback.
 *	Must be a component of type \ref ELEC_GEN.
 * @param cb The callback to set. You may pass `NULL` here to remove
 *	the calllback. You can use libelec_gen_set_rpm() to set the
 *	rpm directly.
 * @see elec_get_rpm_cb_t
 * @see libelec_gen_set_rpm()
 */
void
libelec_gen_set_rpm_cb(elec_comp_t *gen, elec_get_rpm_cb_t cb)
{
	ASSERT(gen != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);
	ASSERT(!gen->sys->started);
	gen->info->gen.get_rpm = cb;
}

/**
 * @return The generator rpm callback, which was previously set using
 *	libelec_gen_set_rpm_cb(), or `NULL` if no callback was set.
 * @see libelec_gen_set_rpm_cb()
 */
elec_get_rpm_cb_t
libelec_gen_get_rpm_cb(const elec_comp_t *gen)
{
	ASSERT(gen != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);
	return (gen->info->gen.get_rpm);
}

/**
 * Sets the load demand callback for an electrical load. libelec can
 * use this callback to determine the electrical load demand of the
 * component.
 * @note You must NOT call this function after the network has been
 *	started by a call to libelec_sys_start(). The network MUST
 *	be stopped before attempting to reconfigure it.
 * @param batt The load for which to configure the callback. Must be
 *	a component of type \ref ELEC_LOAD.
 * @param cb The callback to set. You may pass `NULL` here to remove
 *	the callback. A load which does not have a custom load
 *	callback configured can still fall back to its standard load
 *	demand, as set using the `STD_LOAD` configuration stanza.
 * @see elec_get_load_cb_t
 */
void
libelec_load_set_load_cb(elec_comp_t *load, elec_get_load_cb_t cb)
{
	ASSERT(load != NULL);
	ASSERT3U(load->info->type, ==, ELEC_LOAD);
	ASSERT(!load->sys->started);
	load->info->load.get_load = cb;
}

/**
 * @return The load demand callback, which was previously set using
 *	libelec_load_set_load_cb(), or `NULL` if no callback was set.
 * @see elec_get_load_cb_t
 */
elec_get_load_cb_t
libelec_load_get_load_cb(elec_comp_t *load)
{
	ASSERT(load != NULL);
	ASSERT3U(load->info->type, ==, ELEC_LOAD);
	return (load->info->load.get_load);
}

static void
update_short_leak_factor(elec_comp_t *comp, double d_t)
{
	ASSERT(comp != NULL);
	ASSERT3F(d_t, >, 0);

	if (comp->rw.shorted) {
		/*
		 * Gradually ramp up the leak to give the breaker a bit of
		 * time to stay pushed in.
		 */
		if (comp->info->type == ELEC_LOAD) {
			if (comp->ro.in_pwr != 0)
				FILTER_IN(comp->rw.leak_factor, 0.99, d_t, 1);
			else
				comp->rw.leak_factor = 0;
		} else {
			comp->rw.leak_factor =
			    wavg(0.97, 0.975, crc64_rand_fract());
		}
	} else {
		comp->rw.leak_factor = 0;
	}
}

static void
network_reset(elec_sys_t *sys, double d_t)
{
	ASSERT(sys != NULL);
	ASSERT_MUTEX_HELD(&sys->worker_interlock);
	ASSERT3F(d_t, >, 0);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		elec_comp_t *srcs_ext[ELEC_MAX_SRCS];

		memcpy(srcs_ext, comp->srcs, sizeof (srcs_ext));

		comp->rw.in_volts = 0;
		comp->rw.in_pwr = 0;
		comp->rw.in_amps = 0;
		comp->rw.in_freq = 0;
		comp->rw.out_volts = 0;
		comp->rw.out_pwr = 0;
		comp->rw.out_amps = 0;
		comp->rw.out_freq = 0;
		comp->rw.short_amps = 0;
		comp->src_int_cond_total = 0;
		memset(comp->srcs, 0, sizeof (comp->srcs));
		comp->n_srcs = 0;
		for (unsigned i = 0; i < comp->n_links; i++) {
			memset(comp->links[i].out_amps, 0,
			    sizeof (comp->links[i].out_amps));
		}

		mutex_enter(&comp->rw_ro_lock);
		comp->rw.failed = comp->ro.failed;
		comp->rw.shorted = comp->ro.shorted;
		update_short_leak_factor(comp, d_t);
		memcpy(comp->srcs_ext, srcs_ext, sizeof (comp->srcs_ext));
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
			    comp->n_links * sizeof (*comp->tie.wk_state));
			mutex_exit(&comp->tie.lock);
			break;
		case ELEC_CB:
#ifdef	LIBELEC_WITH_LIBSWITCH
			if (comp->scb.sw != NULL) {
				comp->scb.cur_set =
				    !libswitch_read(comp->scb.sw, NULL);
			}
#endif	/* defined(LIBELEC_WITH_LIBSWITCH) */
			comp->scb.wk_set = comp->scb.cur_set;
			break;
		default:
			break;
		}
	}
}

static void
network_update_gen(elec_comp_t *gen, double d_t)
{
	ASSERT(gen != NULL);
	ASSERT(gen->info != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);

	if (gen->info->gen.get_rpm != NULL) {
		double rpm = gen->info->gen.get_rpm(gen, gen->info->userinfo);
		ASSERT(!isnan(rpm));
		mutex_enter(&gen->gen.lock);
		gen->gen.rpm = MAX(rpm, GEN_MIN_RPM);
		mutex_exit(&gen->gen.lock);
	}
	if (gen->gen.rpm <= GEN_MIN_RPM) {
		gen->gen.stab_factor_U = 1;
		gen->gen.stab_factor_f = 1;
		gen->rw.in_volts = 0;
		gen->rw.in_freq = 0;
		gen->rw.out_volts = 0;
		gen->rw.out_freq = 0;
		return;
	}
	/*
	 * Gradual voltage & frequency stabilization adjustment, to simulate
	 * that the CSD takes a little time to adjust to rpm changes.
	 */
	if (gen->info->gen.stab_rate_U > 0) {
		double stab_factor_U = clamp(gen->gen.ctr_rpm / gen->gen.rpm,
		    gen->gen.min_stab_U, gen->gen.max_stab_U);
		double stab_rate_mod =
		    clamp(1 + crc64_rand_normal(0.1), 0.1, 10);
		FILTER_IN(gen->gen.stab_factor_U, stab_factor_U, d_t,
		    gen->info->gen.stab_rate_U * stab_rate_mod);
	} else {
		gen->gen.stab_factor_U = 1;
	}
	if (gen->info->gen.stab_rate_f > 0) {
		double stab_factor_f = clamp(gen->gen.ctr_rpm / gen->gen.rpm,
		    gen->gen.min_stab_f, gen->gen.max_stab_f);
		double stab_rate_mod =
		    clamp(1 + crc64_rand_normal(0.1), 0.1, 10);
		FILTER_IN(gen->gen.stab_factor_f, stab_factor_f, d_t,
		    gen->info->gen.stab_rate_f * stab_rate_mod);
	} else {
		gen->gen.stab_factor_f = 1;
	}
	if (!gen->rw.failed) {
		if (gen->gen.rpm < gen->info->gen.exc_rpm) {
			gen->rw.in_volts = 0;
			gen->rw.in_freq = 0;
		} else {
			ASSERT(gen->gen.tgt_volts != 0);
			gen->rw.in_volts = (gen->gen.rpm / gen->gen.ctr_rpm) *
			    gen->gen.stab_factor_U * gen->gen.tgt_volts;
			if (gen->gen.tgt_freq != 0) {
				gen->rw.in_freq = (gen->gen.rpm /
				    gen->gen.ctr_rpm) *
				    gen->gen.stab_factor_f * gen->gen.tgt_freq;
			}
		}
		gen->rw.out_volts = gen->rw.in_volts;
		gen->rw.out_freq = gen->rw.in_freq;
	} else {
		gen->rw.in_volts = 0;
		gen->rw.in_freq = 0;
		gen->rw.out_volts = 0;
		gen->rw.out_freq = 0;
	}
}

static void
network_update_batt(elec_comp_t *batt, double d_t)
{
	double U, J, temp_coeff, J_max, I_max, I_rel;

	ASSERT(batt != NULL);
	ASSERT(batt->info != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);

	if (batt->info->batt.get_temp != NULL) {
		double T = batt->info->batt.get_temp(batt,
		    batt->info->userinfo);
		ASSERT3F(T, >, 0);
		mutex_enter(&batt->batt.lock);
		batt->batt.T = T;
		mutex_exit(&batt->batt.lock);
	}
	temp_coeff = fx_lin_multi2(batt->batt.T, batt_temp_energy_curve,
	    ARRAY_NUM_ELEM(batt_temp_energy_curve), true);

	I_max = batt->info->batt.max_pwr / batt->info->batt.volts;
	I_rel = batt->batt.prev_amps / I_max;
	U = libelec_phys_get_batt_voltage(batt->info->batt.volts,
	    batt->batt.chg_rel, I_rel);

	J_max = batt->info->batt.capacity * temp_coeff;
	J = batt->batt.chg_rel * J_max;
	J -= U * batt->batt.prev_amps * d_t;
	/* Incorporate charging change */
	J += batt->batt.rechg_W * d_t;
	batt->batt.rechg_W = 0;

	/* Recalculate the new voltage and relative charge state */
	if (!batt->rw.failed) {
		batt->rw.in_volts = U;
		batt->rw.out_volts = U;
	} else {
		batt->rw.in_volts = 0;
		batt->rw.out_volts = 0;
	}
	/*
	 * If the temperature is very cold, we might slightly overshoot
	 * capacity here, so clamp to 0-1.
	 */
	batt->batt.chg_rel = clamp(J / J_max, 0, 1);
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
	/* 3-phase CBs evenly split the power between themselves */
	if (cb->info->cb.triphase)
		amps_rat /= 3;
	amps_rat = MIN(amps_rat, 5 * cb->info->cb.rate);
	FILTER_IN(cb->scb.temp, amps_rat, d_t, cb->info->cb.rate);

	if (cb->scb.temp >= 1.0) {
		cb->scb.wk_set = false;
		cb->scb.cur_set = false;
#ifdef	LIBELEC_WITH_LIBSWITCH
		/* Also pull the switch if one is present */
		if (cb->scb.sw != NULL)
			libswitch_set(cb->scb.sw, true);
#endif	/* defined(LIBELEC_WITH_LIBSWITCH) */
		if (cb->info->cb.fuse)
			cb->rw.failed = true;
	}
}

static void
network_update_tru(elec_comp_t *tru, double d_t)
{
	ASSERT(tru != NULL);
	ASSERT(tru->info != NULL);
	ASSERT3U(tru->info->type, ==, ELEC_TRU);
	ASSERT3F(d_t, >, 0);

	if (tru->ro.in_volts < tru->info->tru.min_volts) {
		tru->tru.regul = 0;
		return;
	}
	if (!tru->info->tru.charger) {
		/*
		 * In simple TRU operating mode, we operate as a fixed-rate
		 * transformer. Our input voltage is directly proportional
		 * to our output and we provide no output voltage regulation.
		 */
		tru->tru.regul = 1;
	} else {
		/*
		 * In charger mode, we control an additional voltage
		 * regulator parameter that lets us adjust our output
		 * voltage as necessary to stabilize the charging current.
		 */
		ASSERT3F(tru->info->tru.curr_lim, >, 0);
		double oc_ratio = tru->tru.prev_amps / tru->info->tru.curr_lim;
		double regul_tgt = clamp(oc_ratio > 0 ?
		    tru->tru.regul / oc_ratio : 1, 0, 1);
		/*
		 * Don't enable the output if the battery isn't being sensed.
		 */
		if (!libelec_tie_get_all(tru->tru.batt_conn) || oc_ratio > 4) {
			/*
			 * When a super-high current consumer is persistently
			 * on, we might blow our output breaker by trying to
			 * keep on charging. So just back off completely and
			 * slowly come up later to retry.
			 */
			tru->tru.regul = 0;
		} else if (regul_tgt > tru->tru.regul) {
			FILTER_IN(tru->tru.regul, regul_tgt, d_t, 1);
		} else {
			FILTER_IN(tru->tru.regul, regul_tgt, d_t, 2 * d_t);
		}
	}
}

static void
network_srcs_update(elec_sys_t *sys, double d_t)
{
	ASSERT(sys != NULL);
	ASSERT_MUTEX_HELD(&sys->worker_interlock);
	ASSERT3F(d_t, >, 0);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		ASSERT(comp->info != NULL);
		switch (comp->info->type) {
		case ELEC_BATT:
			network_update_batt(comp, d_t);
			break;
		case ELEC_GEN:
			network_update_gen(comp, d_t);
			break;
		case ELEC_TRU:
			network_update_tru(comp, d_t);
			break;
		default:
			break;
		}
	}
}

static void
network_loads_randomize(elec_sys_t *sys, double d_t)
{
	ASSERT(sys != NULL);
	ASSERT3F(d_t, >, 0);

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		if (comp->info->type == ELEC_LOAD) {
			FILTER_IN(comp->load.random_load_factor,
			    clamp(1.0 + crc64_rand_normal(0.1), 0.8, 1.2),
			    d_t, 0.25);
		}
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
				network_load_integrate_load(NULL, comp, 0, d_t);
			load_incap_update(comp, d_t);
		}

		comp->rw.in_pwr = comp->rw.in_volts * comp->rw.in_amps;
		comp->rw.out_pwr = comp->rw.out_volts * comp->rw.out_amps;
	}
}

static void
network_ties_update(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	for (elec_comp_t *comp = list_head(&sys->ties); comp != NULL;
	    comp = list_next(&sys->ties, comp)) {
		unsigned n_tied = 0;
		int tied[2] = { -1, -1 };
		for (unsigned i = 0; i < comp->n_links; i++) {
			if (comp->tie.wk_state[i]) {
				tied[n_tied] = i;
				n_tied++;
				if (n_tied == ARRAY_NUM_ELEM(tied))
					break;
			}
		}
		if (n_tied == 2) {
			double amps[2] = {
			    sum_link_amps(&comp->links[tied[0]]),
			    sum_link_amps(&comp->links[tied[1]])
			};
			comp->rw.out_amps = amps[0] - amps[1];
			comp->rw.out_amps = NO_NEG_ZERO(ABS(comp->rw.out_amps));
			comp->rw.in_amps = comp->rw.out_amps;
		}
	}
}

static inline void
add_src_up(elec_comp_t *comp, elec_comp_t *src, const elec_comp_t *upstream)
{
	ASSERT(comp != NULL);
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);

	ASSERT0(src->src_mask & (1 << src->src_idx));
	ASSERT3U(comp->n_srcs, <, ELEC_MAX_SRCS);
	comp->srcs[comp->n_srcs] = src;
	comp->n_srcs++;
	ASSERT3F(src->info->int_R, >, 0);
	comp->src_int_cond_total += (1.0 / src->info->int_R) *
	    src->rw.out_volts;
	for (unsigned i = 0; i < comp->n_links; i++) {
		if (comp->links[i].comp == upstream) {
			comp->links[i].srcs[src->src_idx] = src;
			return;
		}
	}
	VERIFY_FAIL();
}

static void
network_paint_src_bus(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (comp->rw.failed)
		return;

	add_src_up(comp, src, upstream);
	if (comp->rw.in_volts < src->rw.out_volts) {
		comp->rw.in_volts = src->rw.out_volts;
		comp->rw.in_freq = src->rw.out_freq;
		comp->rw.out_volts = comp->rw.in_volts;
		comp->rw.out_freq = comp->rw.in_freq;
	}
	for (unsigned i = 0; i < comp->n_links; i++) {
		if (comp->links[i].comp != upstream) {
			network_paint_src_comp(src, comp,
			    comp->links[i].comp, depth + 1);
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
	for (unsigned i = 0; i < comp->n_links; i++) {
		if (upstream == comp->links[i].comp) {
			if (comp->tie.wk_state[i]) {
				add_src_up(comp, src, upstream);
				if (comp->rw.in_volts < src->rw.out_volts) {
					comp->rw.in_volts = src->rw.out_volts;
					comp->rw.in_freq = src->rw.out_freq;
				}
				tied = true;
			}
			found = true;
			break;
		}
	}
	ASSERT(found);
	if (tied) {
		for (unsigned i = 0; i < comp->n_links; i++) {
			if (upstream != comp->links[i].comp &&
			    comp->tie.wk_state[i]) {
				network_paint_src_comp(src, comp,
				    comp->links[i].comp, depth + 1);
			}
		}
	}
}

static void
recalc_out_volts_tru(elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TRU);
	comp->rw.out_volts = comp->tru.regul * comp->info->tru.out_volts *
	    (comp->rw.in_volts / comp->info->tru.in_volts);
}

static void
recalc_out_volts_freq_inv(elec_comp_t *comp)
{
	double mult_U, mult_f;

	ASSERT(comp != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_INV);

	mult_U = fx_lin(comp->rw.in_volts, comp->info->tru.min_volts,
	    0.95, comp->info->tru.in_volts, 1);
	mult_f = fx_lin(comp->rw.in_volts, comp->info->tru.min_volts,
	    0.97, comp->info->tru.in_volts, 1);
	comp->rw.out_volts = mult_U * comp->info->tru.out_volts;
	comp->rw.out_freq = mult_f * comp->info->tru.out_freq;
}

static void
network_paint_src_tru_inv(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT(comp->info->type == ELEC_TRU ||
	    comp->info->type == ELEC_INV);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	/* Conversion prevents back-flow of power from output to input */
	if (upstream != comp->links[0].comp)
		return;

	add_src_up(comp, src, upstream);
	if (comp->info->type == ELEC_TRU) {
		ASSERT_MSG(comp->n_srcs == 1, "%s attempted to add a second "
		    "AC power source ([0]=%s, [1]=%s). Multi-source feeding "
		    "is NOT supported in AC networks.", comp->info->name,
		    comp->srcs[0]->info->name, comp->srcs[1]->info->name);
	}
	if (!comp->rw.failed) {
		if (comp->rw.in_volts < src->rw.out_volts &&
		    src->rw.out_volts > comp->info->tru.min_volts) {
			comp->rw.in_volts = src->rw.out_volts;
			comp->rw.in_freq = src->rw.out_freq;
			if (comp->info->type == ELEC_TRU)
				recalc_out_volts_tru(comp);
			else
				recalc_out_volts_freq_inv(comp);
		}
	} else {
		comp->rw.in_volts = 0;
		comp->rw.in_freq = 0;
		comp->rw.out_volts = 0;
		comp->rw.out_freq = 0;
	}
	ASSERT(comp->links[1].comp != NULL);
	/*
	 * The TRU/inverter becomes the source for downstream buses.
	 */
	if (comp->rw.out_volts != 0) {
		network_paint_src_comp(comp, comp,
		    comp->links[1].comp, depth + 1);
	}
}

static void
network_paint_src_xfrmr(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT(comp->info->type == ELEC_XFRMR);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	/* Transformers prevents back-flow of power from output to input */
	if (upstream != comp->links[0].comp)
		return;

	add_src_up(comp, src, upstream);
	ASSERT_MSG(comp->n_srcs == 1, "%s attempted to add a second "
	    "AC power source ([0]=%s, [1]=%s). Multi-source feeding "
	    "is NOT supported in AC networks.", comp->info->name,
	    comp->srcs[0]->info->name, comp->srcs[1]->info->name);

	if (!comp->rw.failed) {
		if (comp->rw.in_volts < src->rw.out_volts) {
			comp->rw.in_volts = src->rw.out_volts;
			comp->rw.out_volts = comp->rw.in_volts *
			    (comp->info->xfrmr.out_volts /
			    comp->info->xfrmr.in_volts);
			comp->rw.in_freq = src->rw.out_freq;
			comp->rw.out_freq = comp->rw.in_freq;
		}
	} else {
		comp->rw.in_volts = 0;
		comp->rw.out_volts = 0;
		comp->rw.in_freq = 0;
		comp->rw.out_freq = 0;
	}
	ASSERT(comp->links[1].comp != NULL);
	/*
	 * The transformer becomes the source for downstream buses.
	 */
	if (comp->rw.out_volts != 0) {
		network_paint_src_comp(comp, comp,
		    comp->links[1].comp, depth + 1);
	}
}

static void
network_paint_src_scb(elec_comp_t *src, elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (!comp->rw.failed && comp->scb.wk_set) {
		add_src_up(comp, src, upstream);
		if (comp->rw.in_volts < src->rw.out_volts) {
			comp->rw.in_volts = src->rw.out_volts;
			comp->rw.in_freq = src->rw.out_freq;
			comp->rw.out_volts = src->rw.out_volts;
			comp->rw.out_freq = src->rw.out_freq;
		}
		if (upstream == comp->links[0].comp) {
			ASSERT(comp->links[1].comp != NULL);
			network_paint_src_comp(src, comp,
			    comp->links[1].comp, depth + 1);
		} else {
			ASSERT3P(upstream, ==, comp->links[1].comp);
			ASSERT(comp->links[0].comp != NULL);
			network_paint_src_comp(src, comp,
			    comp->links[0].comp, depth + 1);
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
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (upstream == comp->links[0].comp) {
		add_src_up(comp, src, upstream);
		ASSERT0(src->rw.out_freq);
		if (!comp->rw.failed) {
			if (comp->rw.in_volts < src->rw.out_volts)
				comp->rw.in_volts = src->rw.out_volts;
		} else {
			comp->rw.in_volts = 0;
		}
		network_paint_src_comp(src, comp, comp->links[1].comp,
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
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	switch (comp->info->type) {
	case ELEC_BATT:
		if (src != comp && comp->rw.out_volts < src->rw.out_volts)
			add_src_up(comp, src, upstream);
		break;
	case ELEC_GEN:
		break;
	case ELEC_BUS:
		if (src->info->type == ELEC_BATT ||
		    src->info->type == ELEC_TRU) {
			ASSERT(!comp->info->bus.ac);
		} else {
			ASSERT3U(src_is_AC(src->info), ==, comp->info->bus.ac);
		}
		network_paint_src_bus(src, upstream, comp, depth);
		break;
	case ELEC_TRU:
	case ELEC_INV:
		network_paint_src_tru_inv(src, upstream, comp, depth);
		break;
	case ELEC_XFRMR:
		network_paint_src_xfrmr(src, upstream, comp, depth);
		break;
	case ELEC_LOAD:
		add_src_up(comp, src, upstream);
		if (!comp->rw.failed) {
			if (comp->rw.in_volts < src->rw.out_volts) {
				comp->rw.in_volts = src->rw.out_volts;
				comp->rw.in_freq = src->rw.out_freq;
			}
		} else {
			comp->rw.in_volts = 0;
			comp->rw.in_freq = 0;
		}
		break;
	case ELEC_CB:
	case ELEC_SHUNT:
		network_paint_src_scb(src, upstream, comp, depth);
		break;
	case ELEC_TIE:
		network_paint_src_tie(src, upstream, comp, depth);
		break;
	case ELEC_DIODE:
		network_paint_src_diode(src, upstream, comp, depth);
		break;
	case ELEC_LABEL_BOX:
		VERIFY_FAIL();
	}
}

static void
network_paint(elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	ASSERT_MUTEX_HELD(&sys->worker_interlock);

	for (elec_comp_t *comp = list_head(&sys->gens_batts); comp != NULL;
	    comp = list_next(&sys->gens_batts, comp)) {
		ASSERT(comp->info != NULL);
		if ((comp->info->type == ELEC_BATT ||
		    comp->info->type == ELEC_GEN) && comp->rw.out_volts != 0) {
			network_paint_src_comp(comp, comp,
			    comp->links[0].comp, 0);
		}
	}
}

static double
network_load_integrate_tru_inv(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *comp, unsigned depth, double d_t)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->links[0].comp != NULL);
	ASSERT(comp->links[1].comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT(comp->info->type == ELEC_TRU || comp->info->type == ELEC_INV);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (upstream != comp->links[0].comp)
		return (0);

	/* When hopping over to the output network, we become the src */
	comp->rw.out_amps = network_load_integrate_comp(
	    comp, comp, comp->links[1].comp, depth + 1, d_t);
	if (comp->rw.failed || comp->rw.in_volts == 0) {
		comp->tru.prev_amps = 0;
		comp->rw.in_amps = 0;
		comp->rw.out_amps = 0;
		return (0);
	}
	/*
	 * Stash the amps value so we can use it to update voltage regulation
	 * on battery chargers.
	 */
	comp->tru.prev_amps = comp->rw.out_amps;
	comp->tru.eff = fx_lin_multi(comp->rw.out_volts * comp->rw.out_amps,
	    comp->info->tru.eff_curve, true);
	ASSERT3F(comp->tru.eff, >, 0);
	ASSERT3F(comp->tru.eff, <, 1);
	comp->rw.in_amps = ((comp->rw.out_volts / comp->rw.in_volts) *
	    comp->rw.out_amps) / comp->tru.eff;

	return (comp->rw.in_amps);
}

static double
network_load_integrate_xfrmr(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *comp, unsigned depth, double d_t)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->links[0].comp != NULL);
	ASSERT(comp->links[1].comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT(comp->info->type == ELEC_XFRMR);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (upstream != comp->links[0].comp)
		return (0);

	/* When hopping over to the output network, we become the src */
	comp->rw.out_amps = network_load_integrate_comp(
	    comp, comp, comp->links[1].comp, depth + 1, d_t);
	if (comp->rw.failed || comp->rw.in_volts == 0) {
		comp->rw.in_amps = 0;
		comp->rw.out_amps = 0;
		return (0);
	}
	comp->xfrmr.eff = fx_lin_multi(comp->rw.out_volts * comp->rw.out_amps,
	    comp->info->xfrmr.eff_curve, true);
	ASSERT3F(comp->xfrmr.eff, >, 0);
	ASSERT3F(comp->xfrmr.eff, <, 1);
	comp->rw.in_amps = ((comp->rw.out_volts / comp->rw.in_volts) *
	    comp->rw.out_amps) / comp->xfrmr.eff;

	return (comp->rw.in_amps);
}

static double
network_load_integrate_load(const elec_comp_t *src, elec_comp_t *comp,
    unsigned depth, double d_t)
{
	double load_WorI, load_I, in_volts_net, incap_I, src_fract;
	const elec_comp_info_t *info;

	/* src can be NULL */
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
		if (info->load.get_load != NULL) {
			load_WorI += info->load.get_load(comp,
			    comp->info->userinfo);
		}
	} else {
		load_WorI = 0;
	}
	ASSERT3F(load_WorI, >=, 0);

	load_WorI *= comp->load.random_load_factor;
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
		 * Shorted components boost their current draw.
		 */
		ASSERT3F(comp->rw.leak_factor, <, 1);
		load_I /= (1 - comp->rw.leak_factor);
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
	if (comp->load.incap_U > comp->rw.in_volts) {
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
		if (comp->load.incap_U >= comp->info->load.min_volts)
			comp->rw.out_volts = comp->load.incap_U;
		else
			comp->rw.out_volts = 0;
		comp->load.incap_d_Q = -used_Q;
	} else {
		/*
		 * Don't forget to add the input capacitance charging
		 * current to the network current draw.
		 */
		comp->rw.in_amps = load_I + incap_I;
		comp->rw.out_amps = load_I;
		comp->rw.out_volts = comp->rw.in_volts;
		comp->rw.out_freq = comp->rw.in_freq;
		comp->load.incap_d_Q = incap_I * d_t;
	}
	ASSERT(!isnan(comp->rw.out_amps));
	ASSERT(!isnan(comp->rw.out_volts));
	comp->load.seen = true;
	if (src != NULL) {
		src_fract = get_src_fract(comp, src);
		comp->links[0].out_amps[src->src_idx] =
		    NO_NEG_ZERO(-comp->rw.in_amps * src_fract);
	} else {
		src_fract = 1;
	}

	return (comp->rw.in_amps * src_fract);
}

static double
network_load_integrate_bus(const elec_comp_t *src, const elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth, double d_t)
{
	double out_amps_total = 0;

	ASSERT(src != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_BUS);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	for (unsigned i = 0; i < comp->n_links; i++) {
		ASSERT(comp->links[i].comp != NULL);
		if (comp->links[i].comp != upstream) {
			comp->links[i].out_amps[src->src_idx] =
			    network_load_integrate_comp(src, comp,
			    comp->links[i].comp, depth + 1, d_t);
			ASSERT3F(comp->links[i].out_amps[src->src_idx], >=, 0);
			out_amps_total += comp->links[i].out_amps[src->src_idx];
		}
	}
	out_amps_total /= (1 - comp->rw.leak_factor);
	return (out_amps_total);
}

static double
network_load_integrate_tie(const elec_comp_t *src, const elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth, double d_t)
{
	double out_amps_total = 0;

	ASSERT(src != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	for (unsigned i = 0; i < comp->n_links; i++) {
		ASSERT(comp->links[i].comp != NULL);
		if (comp->links[i].comp == upstream && !comp->tie.wk_state[i])
			return (0);
	}
	for (unsigned i = 0; i < comp->n_links; i++) {
		ASSERT(comp->links[i].comp != NULL);
		if (comp->tie.wk_state[i] && comp->links[i].comp != upstream) {
			comp->links[i].out_amps[src->src_idx] =
			    network_load_integrate_comp(src, comp,
			    comp->links[i].comp, depth + 1, d_t);
			ASSERT3F(comp->links[i].out_amps[src->src_idx], >=, 0);
			out_amps_total += comp->links[i].out_amps[src->src_idx];
		}
	}
	return (out_amps_total);
}

static double
network_load_integrate_scb(const elec_comp_t *src, const elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth, double d_t)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT(comp->info->type == ELEC_CB || comp->info->type == ELEC_SHUNT);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (!comp->scb.wk_set)
		return (0);
	for (unsigned i = 0; i < 2; i++) {
		if (comp->links[i].comp == upstream) {
			comp->links[!i].out_amps[src->src_idx] =
			    network_load_integrate_comp(src, comp,
			    comp->links[!i].comp, depth + 1, d_t);
			comp->rw.out_amps = sum_link_amps(&comp->links[0]) -
			    sum_link_amps(&comp->links[1]);
			comp->rw.out_amps = NO_NEG_ZERO(ABS(comp->rw.out_amps));
			comp->rw.in_amps = comp->rw.out_amps;

			return (comp->links[!i].out_amps[src->src_idx]);
		}
	}
	VERIFY_FAIL();
}

static double
network_load_integrate_batt(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *batt, unsigned depth, double d_t)
{
	ASSERT(src != NULL);
	ASSERT(batt != NULL);
	ASSERT(batt->info != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);

	if (depth != 0 && check_upstream(batt, src, upstream)) {
		double U_delta = MAX(src->rw.out_volts - batt->rw.out_volts, 0);

		ASSERT0(src->rw.out_freq);
		if (batt->batt.chg_rel < 1) {
			double R = batt->info->batt.chg_R /
			    (1 - batt->batt.chg_rel);
			batt->rw.in_volts = src->rw.out_volts;
			batt->rw.in_amps = U_delta / R;
			/*
			 * Store the charging rate so network_update_batt can
			 * incorporate it into its battery energy state
			 * calculation.
			 */
			batt->batt.rechg_W = batt->rw.in_volts *
			    batt->rw.in_amps;
		}
		batt->rw.out_amps = 0;

		return (batt->rw.in_amps);
	} else if (depth == 0) {
		batt->rw.out_amps = network_load_integrate_comp(batt, batt,
		    batt->links[0].comp, depth + 1, d_t);
		batt->batt.prev_amps = batt->rw.out_amps;

		return (batt->rw.out_amps);
	} else {
		return (0);
	}
}

static double
network_load_integrate_gen(elec_comp_t *gen, unsigned depth, double d_t)
{
	double out_pwr;

	ASSERT(gen != NULL);
	ASSERT(gen->info != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);

	if (depth != 0)
		return (0);

	gen->rw.out_amps = network_load_integrate_comp(gen, gen,
	    gen->links[0].comp, depth + 1, d_t);
	gen->rw.in_volts = gen->rw.out_volts;
	gen->rw.in_freq = gen->rw.out_freq;
	out_pwr = gen->rw.in_volts * gen->rw.out_amps;
	gen->gen.eff = fx_lin_multi(out_pwr, gen->info->gen.eff_curve, true);
	gen->rw.in_amps = gen->rw.out_amps / gen->gen.eff;

	return (gen->rw.out_amps);
}

static double
network_load_integrate_diode(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *comp, unsigned depth, double d_t)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT3P(upstream, ==, comp->links[0].comp);
	ASSERT3F(d_t, >, 0);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	comp->links[1].out_amps[src->src_idx] = network_load_integrate_comp(
	    src, comp, comp->links[1].comp, depth + 1, d_t);
	comp->rw.out_amps = sum_link_amps(&comp->links[1]);
	comp->rw.in_amps = comp->rw.out_amps;
	ASSERT(!isnan(comp->rw.in_amps));

	return (comp->links[1].out_amps[src->src_idx]);
}

static double
network_load_integrate_comp(const elec_comp_t *src,
    const elec_comp_t *upstream, elec_comp_t *comp, unsigned depth, double d_t)
{
	ASSERT(src != NULL);
	ASSERT(src->info != NULL);
	ASSERT(src->info->type == ELEC_BATT || src->info->type == ELEC_GEN ||
	    src->info->type == ELEC_TRU || src->info->type == ELEC_INV ||
	    src->info->type == ELEC_XFRMR);
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);
	ASSERT3F(d_t, >, 0);

	if (comp != src && !check_upstream(comp, src, upstream))
		return (0);

	switch (comp->info->type) {
	case ELEC_BATT:
		return (network_load_integrate_batt(src, upstream, comp,
		    depth, d_t));
	case ELEC_GEN:
		return (network_load_integrate_gen(comp, depth, d_t));
	case ELEC_TRU:
	case ELEC_INV:
		ASSERT3P(upstream, ==, comp->links[0].comp);
		return (network_load_integrate_tru_inv(src, upstream, comp,
		    depth, d_t));
	case ELEC_XFRMR:
		ASSERT3P(upstream, ==, comp->links[0].comp);
		return (network_load_integrate_xfrmr(src, upstream, comp,
		    depth, d_t));
	case ELEC_LOAD:
		return (network_load_integrate_load(src, comp, depth, d_t));
	case ELEC_BUS:
		return (network_load_integrate_bus(src, upstream, comp,
		    depth, d_t));
	case ELEC_CB:
	case ELEC_SHUNT:
		return (network_load_integrate_scb(src, upstream, comp,
		    depth, d_t));
	case ELEC_TIE:
		return (network_load_integrate_tie(src, upstream, comp,
		    depth, d_t));
	case ELEC_DIODE:
		return (network_load_integrate_diode(src, upstream, comp,
		    depth, d_t));
	case ELEC_LABEL_BOX:
		VERIFY_FAIL();
	}
	VERIFY_FAIL();
}

static void
network_load_integrate(elec_sys_t *sys, double d_t)
{
	ASSERT(sys != NULL);
	ASSERT3F(d_t, >, 0);

	for (elec_comp_t *comp = list_head(&sys->gens_batts); comp != NULL;
	    comp = list_next(&sys->gens_batts, comp)) {
		comp->rw.out_amps = network_load_integrate_comp(comp, comp,
		    comp, 0, d_t);
	}
}

static void
network_state_xfer(elec_sys_t *sys)
{
	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		mutex_enter(&comp->rw_ro_lock);
		/* Copy in caller-side settings that might have been changed */
		if (comp->ro.failed != comp->rw.failed)
			comp->rw.failed = comp->ro.failed;
		if (comp->ro.shorted != comp->rw.shorted)
			comp->rw.shorted = comp->ro.shorted;
		comp->ro = comp->rw;
		mutex_exit(&comp->rw_ro_lock);
	}
}

static void
mk_spaces(char *spaces, unsigned len)
{
	memset(spaces, 0, len);
	for (unsigned i = 0; i + 1 < len; i += 2) {
		spaces[i] = '|';
		if (i + 3 < len)
			spaces[i + 1] = ' ';
		else
			spaces[i + 1] = '-';
	}
}

static void
print_trace_data(const elec_comp_t *comp, unsigned depth, bool out_data,
    double load)
{
	char *spaces;
	double W;

	ASSERT(comp != NULL);

	spaces = safe_malloc(2 * depth + 1);
	mk_spaces(spaces, 2 * depth + 1);
	W = out_data ? (comp->rw.out_volts * comp->rw.out_amps) :
	    (comp->rw.in_volts * comp->rw.in_amps);
	logMsg("%s%-5s  %s  %3s: %.2fW  LOADS: %.2fW",
	    spaces, comp_type2str(comp->info->type), comp->info->name,
	    out_data ? "OUT" : "IN", W, load);
	free(spaces);
}

static double
network_trace_scb(const elec_comp_t *upstream, const elec_comp_t *comp,
    unsigned depth, bool do_print)
{
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->info->type == ELEC_CB || comp->info->type == ELEC_SHUNT);

	if (!comp->scb.wk_set)
		return (0);
	if (upstream == comp->links[0].comp) {
		return (network_trace(comp, comp->links[1].comp,
		    depth + 1, do_print));
	} else {
		return (network_trace(comp, comp->links[0].comp,
		    depth + 1, do_print));
	}
}

static double
network_trace(const elec_comp_t *upstream, const elec_comp_t *comp,
    unsigned depth, bool do_print)
{
	double load_trace = 0;

	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);

	switch (comp->info->type) {
	case ELEC_BATT:
		load_trace = network_trace(comp, comp->links[0].comp, depth + 1,
		    false);
		load_trace += comp->rw.out_volts * comp->rw.in_amps;
		if (do_print) {
			if (upstream == comp) {
				print_trace_data(comp, depth, true, load_trace);
			} else {
				print_trace_data(comp, depth, false,
				    load_trace);
			}
			network_trace(comp, comp->links[0].comp, depth + 1,
			    true);
		}
		break;
	case ELEC_GEN:
		load_trace = network_trace(comp, comp->links[0].comp,
		    depth + 1, false);
		if (do_print) {
			print_trace_data(comp, depth, true, load_trace);
			network_trace(comp, comp->links[0].comp, depth + 1,
			    true);
		}
		break;
	case ELEC_TRU:
	case ELEC_INV:
	case ELEC_XFRMR:
		if (upstream != comp) {
			if (do_print)
				print_trace_data(comp, depth, false, 0);
			return (comp->rw.in_volts * comp->rw.in_amps);
		} else {
			load_trace = network_trace(comp,
			    comp->links[0].comp, depth + 1, false);
			if (do_print) {
				print_trace_data(comp, depth, true, load_trace);
				network_trace(comp, comp->links[0].comp,
				    depth + 1, true);
			}
		}
		break;
	case ELEC_LOAD:
		if (do_print)
			print_trace_data(comp, depth, false, 0);
		return (comp->rw.in_volts * comp->rw.in_amps);
	case ELEC_BUS:
		for (unsigned i = 0; i < comp->n_links; i++) {
			load_trace += network_trace(comp, comp->links[i].comp,
			    depth + 1, false);
		}
		if (do_print) {
			print_trace_data(comp, depth, false, load_trace);
			for (unsigned i = 0; i < comp->n_links; i++) {
				network_trace(comp, comp->links[i].comp,
				    depth + 1, true);
			}
		}
		break;
	case ELEC_CB:
	case ELEC_SHUNT:
		load_trace = network_trace_scb(upstream, comp, depth, false);
		if (do_print) {
			print_trace_data(comp, depth, false, load_trace);
			network_trace_scb(upstream, comp, depth, true);
		}
		break;
	case ELEC_TIE:
		for (unsigned i = 0; i < comp->n_links; i++) {
			if (comp->tie.wk_state[i]) {
				load_trace += network_trace(comp,
				    comp->links[i].comp, depth + 1, false);
			}
		}
		if (do_print) {
			print_trace_data(comp, depth, false, load_trace);
			for (unsigned i = 0; i < comp->n_links; i++) {
				if (comp->tie.wk_state[i]) {
					load_trace += network_trace(comp,
					    comp->links[i].comp, depth + 1,
					    true);
				}
			}
		}
		break;
	case ELEC_DIODE:
		load_trace = network_trace(comp, comp->links[1].comp,
		    depth + 1, false);
		if (do_print) {
			print_trace_data(comp, depth, false, load_trace);
			network_trace(comp, comp->links[1].comp,
			    depth + 1, true);
		}
		break;
	default:
		VERIFY_FAIL();
	}

	return (load_trace);
}

static bool_t
elec_sys_worker(void *userinfo)
{
	elec_sys_t *sys;
	uint64_t now = microclock();
	double d_t;
#ifdef	LIBELEC_TIMING_DEBUG
	static int steps = 0;
	static double d_t_total = 0;
	static uint64_t last_report = 0;
#endif	/* defined(LIBELEC_TIMING_DEBUG) */

	ASSERT(userinfo != NULL);
	sys = userinfo;

	mutex_enter(&sys->paused_lock);
	if (sys->paused || sys->prev_clock == 0) {
		mutex_exit(&sys->paused_lock);
		sys->prev_clock = now;
		return (B_TRUE);
	}
	d_t = USEC2SEC(now - sys->prev_clock) * sys->time_factor;
	sys->prev_clock = now;
	mutex_exit(&sys->paused_lock);

#ifdef	LIBELEC_TIMING_DEBUG
	steps++;
	d_t_total += d_t;
	if (now - last_report > SEC2USEC(1)) {
		logMsg("steps: %4d  d_t_avg: %f", steps, d_t_total / steps);
		steps = 0;
		d_t_total = 0;
		last_report = now;
	}
#endif	/* defined(LIBELEC_TIMING_DEBUG) */

	/*
	 * In net-recv mode, we only listen in for updates to our requested
	 * endpoints and nothing else.
	 */
#ifdef	LIBELEC_WITH_NETLINK
	if (sys->net_recv.active) {
		elec_net_recv_update(sys);
		return (true);
	}
#endif	/* defined(LIBELEC_WITH_NETLINK) */
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

	network_reset(sys, d_t);
	network_srcs_update(sys, d_t);
	network_loads_randomize(sys, d_t);
	network_paint(sys);
	network_load_integrate(sys, d_t);
	network_loads_update(sys, d_t);
	network_ties_update(sys);
	/*
	 * Must occur AFTER the integrity check! network_state_xfer touches
	 * the rw state and syncs it to the ro state.
	 */
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

#ifdef	LIBELEC_WITH_NETLINK
	elec_net_send_update(sys);
#endif
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

	if (comp->info->type == ELEC_BATT)
		mutex_destroy(&comp->batt.lock);
	else if (comp->info->type == ELEC_GEN)
		mutex_destroy(&comp->gen.lock);

	ZERO_FREE_N(comp->links, comp->n_links);
	if (comp->info->type == ELEC_TIE) {
		free(comp->tie.cur_state);
		free(comp->tie.wk_state);
		mutex_destroy(&comp->tie.lock);
	}
	mutex_destroy(&comp->rw_ro_lock);

	memset(comp, 0, sizeof (*comp));
	free(comp);
}

/**
 * Sets whether a circuit breaker is current set (closed) or reset (open).
 * @note The passed `comp` MUST be of type \ref ELEC_CB.
 */
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
	comp->scb.cur_set = set;
}

/**
 * @return True if the circuit breaker is currently closed,
 *	false if it is open.
 * @note The passed `comp` MUST be of type \ref ELEC_CB.
 */
bool
libelec_cb_get(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	/* atomic read, no need to lock */
	return (comp->scb.cur_set);
}

/**
 * @return The non-dimensional temperature of the circuit breaker's
 *	control filament.
 * @note The passed `comp` MUST be of type \ref ELEC_CB.
 */
double
libelec_cb_get_temp(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	/* atomic read, no need to lock */
	return (comp->scb.temp);
}

/**
 * Reconfigures a tie's current state to tie together the bus connections
 * matching a list of \ref elec_comp_t pointers.
 * @param comp The tie to reconfigure. This must be of type \ref ELEC_TIE.
 * @param list_len Number of elements in `bus_list`.
 * @param bus_list List of \ref elec_comp_t pointers, which should be
 *	tied together. The tie MUST be connected to these buses. Any
 *	buses which the tie is connected to but are not mentioned on
 *	this list become untied.
 * @see \ref ELEC_TIE
 */
void
libelec_tie_set_list(elec_comp_t *comp, size_t list_len,
    elec_comp_t *const bus_list[STATIC_ARRAY_LEN_ARG(list_len)])
{
	bool *new_state;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT(bus_list != NULL || list_len == 0);

	/* A failure of a tie means it gets stuck in its current position */
	if (comp->ro.failed)
		return;

	if (list_len == 0) {
		mutex_enter(&comp->tie.lock);
		memset(comp->tie.cur_state, 0,
		    comp->n_links * sizeof (*comp->tie.cur_state));
		mutex_exit(&comp->tie.lock);
		return;
	}

	/* The buslist is immutable, so no need to lock */
	new_state = safe_calloc(comp->n_links, sizeof (*new_state));
	for (size_t i = 0; i < list_len; i++) {
		size_t j;
		for (j = 0; j < comp->n_links; j++) {
			if (comp->links[j].comp == bus_list[i]) {
				new_state[j] = true;
				break;
			}
		}
		ASSERT_MSG(j != comp->n_links,
		    "Tie %s is not connected to bus %s", comp->info->name,
		    bus_list[i]->info->name);
	}

	mutex_enter(&comp->tie.lock);
	memcpy(comp->tie.cur_state, new_state,
	    comp->n_links * sizeof (*comp->tie.cur_state));
	mutex_exit(&comp->tie.lock);

	free(new_state);
}

/**
 * A convenience variadic version of libelec_tie_set_list().
 * @param comp The tie to reconfigure. This must be of type \ref ELEC_TIE.
 * @param ... A variadic list of \ref elec_comp_t pointers to buses, which
 *	should be tied together. This list MUST be terminated by a single
 *	`NULL` pointer (sentinel). Any buses which the tie is connected to
 *	but are not mentioned on this list become untied.
 */
void
libelec_tie_set(elec_comp_t *comp, ...)
{
	va_list ap;

	ASSERT(comp != NULL);

	va_start(ap, comp);
	libelec_tie_set_v(comp, ap);
	va_end(ap);
}

/**
 * Same as libelec_tie_set(), but takes a `va_list` argument, allowing
 * for nesting within other variadic functions.
 * @param comp The tie to reconfigure. This must be of type \ref ELEC_TIE.
 */
void
libelec_tie_set_v(elec_comp_t *comp, va_list ap)
{
	va_list ap2;
	elec_comp_t **bus_list;
	size_t len;

	ASSERT(comp != NULL);
	ASSERT(comp->sys != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	if (comp->ro.failed)
		return;

	va_copy(ap2, ap);
	for (len = 0; va_arg(ap2, elec_comp_t *) != NULL; len++)
		;
	va_end(ap2);

	bus_list = safe_malloc(sizeof (*bus_list) * len);
	for (size_t i = 0; i < len; i++)
		bus_list[i] = va_arg(ap, elec_comp_t *);
	libelec_tie_set_list(comp, len, bus_list);
	free(bus_list);
}

/**
 * Convenience function, which allows either tying all connections of
 * an \ref ELEC_TIE component, or untying all of them. This is
 * equivalent to calling libelec_tie_set_list() with a list of all
 * buses to which the tie is connected.
 * @param comp The tie to reconfigure. This must be of type \ref ELEC_TIE.
 * @param tied Flag indicating whether all endpoints of the tie should
 *	be tied, or untied.
 * @see libelec_tie_get_all()
 */
void
libelec_tie_set_all(elec_comp_t *comp, bool tied)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	if (comp->ro.failed)
		return;

	mutex_enter(&comp->tie.lock);
	for (unsigned i = 0; i < comp->n_links; i++)
		comp->tie.cur_state[i] = tied;
	mutex_exit(&comp->tie.lock);
}

/**
 * @param comp The tie to examine. This must be of type \ref ELEC_TIE.
 * @return True if all endpoints of the tie are currently tied together,
 *	or false otherwise. This is useful for ties which you are
 *	utilizing as a simple switch. Use libelec_tie_set_all() to
 *	either open and close the switch, and libelec_tie_get_all() to
 *	examine if the switch is open or closed.
 * @see libelec_tie_set_all()
 */
bool
libelec_tie_get_all(elec_comp_t *comp)
{
	bool tied = true;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);

	mutex_enter(&comp->tie.lock);
	for (unsigned i = 0; tied && i < comp->n_links; i++)
		tied &= comp->tie.cur_state[i];
	mutex_exit(&comp->tie.lock);

	return (tied);
}

/**
 * Retrieves the exact list of tied buses for a \ref ELEC_TIE component.
 * @param comp The tie for which to retrieve the list. This MUST be a
 *	component of \ref ELEC_TIE.
 * @param cap Capacity of `bus_list`. This MUST be greater than or equal
 *	to the return value of libelec_tie_get_num_buses().
 * @param bus_list A return argument, which will be filled with the list
 *	of buses currently tied. Any buses NOT mentioned in this list
 *	are currently NOT tied. This array MUST be able to hold at least
 *	libelec_tie_get_num_buses() elements.
 * @return The actual number of buses filled into `bus_list`.
 * @see libelec_tie_get_num_buses()
 */
size_t
libelec_tie_get_list(elec_comp_t *comp, size_t cap,
    elec_comp_t *bus_list[STATIC_ARRAY_LEN_ARG(cap)])
{
	size_t n_tied = 0;

	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	ASSERT3U(cap, >=, comp->n_links);
	ASSERT(bus_list != NULL);

	mutex_enter(&comp->tie.lock);
	for (unsigned i = 0; i < comp->n_links; i++) {
		if (comp->tie.cur_state[i])
			bus_list[n_tied++] = comp->links[i].comp;
	}
	mutex_exit(&comp->tie.lock);

	return (n_tied);
}

/**
 * @return The total number of bus connections in tie `comp`. This MUST
 *	be a component of \ref ELEC_TIE. The returned number DOESN'T
 *	signify that that number of buses are currently tied together,
 *	only that the tie is connected to this number of buses and CAN
 *	be tied together. Use libelec_tie_get_list() to determine the
 *	exact buses which are currently tied together.
 * @see libelec_tie_get_list()
 */
size_t
libelec_tie_get_num_buses(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TIE);
	/* This is immutable, so no need to lock */
	return (comp->n_links);
}

/**
 * Given a variadic argument list of buses, determines if the buses are
 * currently tied.
 * @param tie The tie to examine. This must be a component of type
 *	\ref ELEC_TIE.
 * @param exhaustive See description of return value below.
 * @param ... List elec_comp_t pointers to the buses which are to be
 *	examined. The argument list MUST be terminated by a NULL pointer
 *	(sentinel).
 * @return True if all of the buses specified in the argument list are
 *	tied. If even a single one of the buses is untied, this function
 *	returns false. In addition, if `exhaustive` is set to true, this
 *	function returns `false` if the argument list doesn't mention
 *	all of the buses currently tied (meaning, the argument list must
 *	be an exhaustive list of buses which are currently tied).
 */
bool
libelec_tie_get(elec_comp_t *tie, bool_t exhaustive, ...)
{
	va_list ap;
	bool res;

	ASSERT(tie != NULL);
	ASSERT3U(tie->info->type, ==, ELEC_TIE);
	va_start(ap, exhaustive);
	res = libelec_tie_get_v(tie, exhaustive, ap);
	va_end(ap);

	return (res);
}

/**
 * Same as libelec_tie_get(), but takes a `va_list` argument for the
 * bus list, to allow for nesting inside of other variadic functions.
 */
bool
libelec_tie_get_v(elec_comp_t *tie, bool exclusive, va_list ap)
{
	bool res = true;
	size_t list_len = 0, n_tied = 0;
	const elec_comp_t *bus;

	ASSERT(tie != NULL);
	ASSERT3U(tie->info->type, ==, ELEC_TIE);

	mutex_enter(&tie->tie.lock);
	for (unsigned i = 0; i < tie->n_links; i++) {
		if (tie->tie.cur_state[i])
			n_tied++;
	}
	for (list_len = 0; (bus = va_arg(ap, const elec_comp_t *)) != NULL;
	    list_len++) {
		bool found = false;
		for (unsigned i = 0; i < tie->n_links; i++) {
			if (tie->links[i].comp == bus) {
				found = true;
				if (!tie->tie.cur_state[i]) {
					res = false;
					goto out;
				}
				break;
			}
		}
		ASSERT(found);
	}
	if (list_len != n_tied && exclusive)
		res = false;
out:
	mutex_exit(&tie->tie.lock);

	return (res);
}

/**
 * Sets the generator's RPM value.
 * @param gen The generator for which to set the rpm. This MUST be a
 *	component of type \ref ELEC_GEN.
 * @param rpm The RPM to set on the generator.
 * @note Despite its name, the units of `rpm` are arbitrary. You can use
 *	whatever units you prefer. You simply need to be consistent with
 *	the `EXC_RPM`, `MIN_RPM` and `MAX_RPM` stanzas when defining
 *	the generator in the config file.
 * @see libelec_gen_get_rpm()
 */
void
libelec_gen_set_rpm(elec_comp_t *gen, double rpm)
{
	ASSERT(gen != NULL);
	ASSERT(gen->info != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);
	ASSERT_MSG(gen->info->gen.get_rpm == NULL,
	    "Attempted to call libelec_gen_set_rpm() for generator %s, "
	    "however this generator already had an rpm callback set using "
	    "libelec_gen_set_rpm_cb(). You may NOT mix both mechanisms for "
	    "setting a generator's speed. Either set the speed directly "
	    "using libelec_gen_set_rpm() -OR- use the callback method "
	    "using libelec_gen_set_rpm_cb(), but not both.", gen->info->name);
	mutex_enter(&gen->gen.lock);
	gen->gen.rpm = rpm;
	mutex_exit(&gen->gen.lock);
}

/**
 * @return The current generator RPM of `gen`. This MUST be a component of
 *	type \ref ELEC_GEN.
 * @note Despite its name, the units of speed are arbitrary and are
 *	dependent on what you set with either libelec_gen_set_rpm(), or
 *	what the elec_gen_rpm_cb_t callback returned for the generator.
 * @see libelec_gen_set_rpm()
 * @see elec_gen_rpm_cb_t
 */
double
libelec_gen_get_rpm(const elec_comp_t *gen)
{
	double rpm;
	ASSERT(gen != NULL);
	ASSERT(gen->info != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);
	mutex_enter(&((elec_comp_t *)gen)->gen.lock);
	rpm = gen->gen.rpm;
	mutex_exit(&((elec_comp_t *)gen)->gen.lock);
	return (rpm);
}

/**
 * @return The relative state-of-charge of the battery `batt`. This
 *	MUST be a component of type \ref ELEC_BATT.
 * @see libelec_batt_set_chg_rel()
 */
double
libelec_batt_get_chg_rel(const elec_comp_t *batt)
{
	ASSERT(batt != NULL);
	ASSERT(batt->info != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);
	return (batt->batt.chg_rel);
}

/**
 * Sets the relative state-of-charge of a battery. Due to the
 * temperature-dependent behavior of batteries, libelec uses relative
 * state of charge, rather than absolute energy content.
 * @see libelec_batt_get_chg_rel()
 */
void
libelec_batt_set_chg_rel(elec_comp_t *batt, double chg_rel)
{
	ASSERT(batt != NULL);
	ASSERT(batt->info != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);
	ASSERT3F(chg_rel, >=, 0);
	ASSERT3F(chg_rel, <=, 1);

	mutex_enter(&batt->sys->worker_interlock);
	batt->batt.chg_rel = chg_rel;
	/* To prevent over-charging if the previous cycle was charging */
	batt->batt.rechg_W = 0;
	mutex_exit(&batt->sys->worker_interlock);
}

/**
 * @return The temperature of the battery `batt` in Kelvin. This
 *	MUST be a component of type \ref ELEC_BATT.
 * @see libelec_batt_set_temp()
 */
double
libelec_batt_get_temp(const elec_comp_t *batt)
{
	double T;
	ASSERT(batt != NULL);
	ASSERT(batt->info != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);
	mutex_enter(&((elec_comp_t *)batt)->batt.lock);
	T = batt->batt.T;
	mutex_exit(&((elec_comp_t *)batt)->batt.lock);
	return (T);
}

/**
 * Sets the temperature of a battery. This affects the battery's capability
 * to output current and total energy content, due to temperature
 * significantly affecting how the chemistry inside of the battery behaves.
 * @param T The new temperature of the battery in Kelvin. Must NOT be negative.
 * @see libelec_batt_get_temp()
 */
void
libelec_batt_set_temp(elec_comp_t *batt, double T)
{
	ASSERT(batt != NULL);
	ASSERT(batt->info != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);
	ASSERT3F(T, >, 0);
	mutex_enter(&batt->batt.lock);
	batt->batt.T = T;
	mutex_exit(&batt->batt.lock);
}

/**
 * @return Bool indicating if the battery charger is operating normally.
 */
bool
libelec_chgr_get_working(const elec_comp_t *chgr)
{
	ASSERT(chgr != NULL);
	ASSERT(chgr->info != NULL);
	ASSERT3U(chgr->info->type, ==, ELEC_TRU);
	ASSERT(chgr->info->tru.charger);
	ASSERT(chgr->tru.batt_conn != NULL);
	return (chgr->ro.in_volts > 90 &&
	    libelec_tie_get_all(chgr->tru.batt_conn));
}

/**
 * Utility function which allows you to determine the battery voltage of
 * a custom battery, mirroring the same algorithm which libelec uses to
 * calculate battery voltage.
 * @param U_nominal Nominal battery voltage when at full state of charge
 *	and standard temperature (typically around 20°C).
 * @param chg_rel Relative state of charge from 0.0 to 1.0.
 * @param I_rel Relative current draw out of the battery, compared to
 *	its nominal design current draw rating. Higher current draws
 *	cause the battery voltage to depress more, due to the effects
 *	of internal resistance.
 */
double
libelec_phys_get_batt_voltage(double U_nominal, double chg_rel, double I_rel)
{
	static const vect2_t chg_volt_curve[] = {
	    {0.00, 0.00},
	    {0.04, 0.70},
	    {0.10, 0.80},
	    {0.20, 0.87},
	    {0.30, 0.91},
	    {0.45, 0.94},
	    {0.60, 0.95},
	    {0.80, 0.96},
	    {0.90, 0.97},
	    {1.00, 1.00}
	};
	ASSERT3F(U_nominal, >, 0);
	ASSERT3F(chg_rel, >=, 0);
	/*
	 * Small numerical precision errors during a state restore can cause
	 * this value to go slightly over '1'. Ignore those cases.
	 */
	ASSERT3F(chg_rel, <=, 1.0001);
	I_rel = clamp(I_rel, 0, 1);
	return (U_nominal * (1 - clamp(pow(I_rel, 1.45), 0, 1)) *
	    fx_lin_multi2(chg_rel, chg_volt_curve,
	    ARRAY_NUM_ELEM(chg_volt_curve), true));
}

#ifdef	LIBELEC_WITH_NETLINK

static void
kill_conn(elec_sys_t *sys, net_conn_t *conn)
{
	ASSERT(sys != NULL);
	ASSERT_MUTEX_HELD(&sys->worker_interlock);
	ASSERT(conn != NULL);

	list_remove(&sys->net_send.conns_list, conn);
	htbl_remove(&sys->net_send.conns, &conn->conn_id, false);

	free(conn->map);
	ZERO_FREE(conn);
}

static void
conn_add_notif(netlink_conn_id_t conn_id, netlink_conn_ev_t ev, void *userinfo)
{
	elec_sys_t *sys;

	UNUSED(conn_id);
	UNUSED(ev);
	ASSERT(userinfo != NULL);
	sys = userinfo;

	mutex_enter(&sys->worker_interlock);
	if (sys->net_recv.active && sys->started)
		send_net_recv_map(sys);
	mutex_exit(&sys->worker_interlock);
}

static void
conn_rem_notif(netlink_conn_id_t conn_id, netlink_conn_ev_t ev,
    void *userinfo)
{
	elec_sys_t *sys;
	net_conn_t *conn;

	UNUSED(ev);
	ASSERT(userinfo != NULL);
	sys = userinfo;

	mutex_enter(&sys->worker_interlock);
	conn = htbl_lookup(&sys->net_send.conns, &conn_id);
	if (conn != NULL)
		kill_conn(sys, conn);
	mutex_exit(&sys->worker_interlock);
}

void
libelec_enable_net_send(elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	ASSERT(!sys->started);
	ASSERT(!sys->net_send.active);
	ASSERT(!sys->net_recv.active);

	sys->net_send.active = true;
	sys->net_send.xmit_ctr = 0;
	htbl_create(&sys->net_send.conns, 128, sizeof (netlink_conn_id_t),
	    false);
	list_create(&sys->net_send.conns_list, sizeof (net_conn_t),
	    offsetof(net_conn_t, node));

	sys->net_send.proto.proto_id = NETLINK_PROTO_LIBELEC;
	sys->net_send.proto.name = "libelec";
	sys->net_send.proto.msg_rcvd_notif = netlink_send_msg_notif;
	sys->net_send.proto.conn_rem_notif = conn_rem_notif;
	sys->net_send.proto.userinfo = sys;
	netlink_add_proto(&sys->net_send.proto);
}

static void
net_conn_free(void *net_conn, void *unused)
{
	net_conn_t *nc;

	ASSERT(net_conn != NULL);
	nc = net_conn;
	UNUSED(unused);
	free(nc->map);
	ZERO_FREE(nc);
}

void
libelec_disable_net_send(elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	ASSERT(!sys->started);

	if (sys->net_send.active) {
		netlink_remove_proto(&sys->net_send.proto);
		while (list_remove_head(&sys->net_send.conns_list) != NULL)
			;
		list_destroy(&sys->net_send.conns_list);
		htbl_empty(&sys->net_send.conns, net_conn_free, NULL);
		htbl_destroy(&sys->net_send.conns);
		sys->net_send.active = false;
	}
}

void
libelec_enable_net_recv(elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	ASSERT(!sys->started);
	ASSERT(!sys->net_send.active);
	ASSERT(!sys->net_recv.active);

	sys->net_recv.map = safe_calloc(NETMAPSZ(sys),
	    sizeof (*sys->net_recv.map));
	sys->net_recv.map_dirty = false;
	sys->net_recv.active = true;

	sys->net_recv.proto.proto_id = NETLINK_PROTO_LIBELEC;
	sys->net_recv.proto.name = "libelec";
	sys->net_recv.proto.msg_rcvd_notif = netlink_recv_msg_notif;
	sys->net_recv.proto.conn_add_notif = conn_add_notif;
	sys->net_recv.proto.userinfo = sys;

	netlink_add_proto(&sys->net_recv.proto);
}

void
libelec_disable_net_recv(elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	ASSERT(!sys->started);
	if (sys->net_recv.active) {
		netlink_remove_proto(&sys->net_recv.proto);
		free(sys->net_recv.map);
		sys->net_recv.map = NULL;
		sys->net_recv.active = false;
	}
}

static net_conn_t *
get_net_conn(elec_sys_t *sys, netlink_conn_id_t conn_id)
{
	net_conn_t *conn;

	ASSERT(sys != NULL);
	ASSERT_MUTEX_HELD(&sys->worker_interlock);

	conn = htbl_lookup(&sys->net_send.conns, &conn_id);
	if (conn == NULL) {
		conn = safe_calloc(1, sizeof (*conn));
		conn->conn_id = conn_id;
		conn->map = safe_calloc(NETMAPSZ(sys), sizeof (*conn->map));
		delay_line_init(&conn->kill_delay, SEC2USEC(20));
		htbl_set(&sys->net_send.conns, &conn_id, conn);
		list_insert_tail(&sys->net_send.conns_list, conn);
	}

	return (conn);
}

static void
handle_net_req_map(elec_sys_t *sys, net_conn_t *conn, const net_req_map_t *req)
{
	ASSERT(sys != NULL);
	ASSERT(conn != NULL);
	ASSERT(req != NULL);

	memcpy(conn->map, req->map, NETMAPSZ(sys));
	conn->num_active = 0;
	for (unsigned i = 0, n = list_count(&sys->comps); i < n; i++) {
		if (NETMAPGET(conn->map, i))
			conn->num_active++;
	}
	DELAY_LINE_PUSH_IMM(&conn->kill_delay, false);
}

static void
netlink_send_msg_notif(netlink_conn_id_t conn_id, const void *buf, size_t sz,
    void *userinfo)
{
	elec_sys_t *sys;
	const net_req_t *req;
	net_conn_t *conn;

	ASSERT(buf != NULL);
	ASSERT(userinfo != NULL);
	sys = userinfo;

	if (sz < sizeof (net_req_t)) {
		logMsg("Received bad packet length %d", (int)sz);
		return;
	}
	req = buf;
	if (req->version != LIBELEC_NET_VERSION) {
		logMsg("Received bad version %d which doesn't match ours (%d)",
		    req->version, LIBELEC_NET_VERSION);
		return;
	}
	mutex_enter(&sys->worker_interlock);
	conn = get_net_conn(sys, conn_id);
	if (req->req == NET_REQ_MAP) {
		const net_req_map_t *map = buf;

		if (map->conf_crc == sys->conf_crc &&
		    sz == sizeof (net_req_map_t) + NETMAPSZ(sys)) {
			handle_net_req_map(sys, conn, buf);
		} else {
#if	!IBM
			logMsg("Cannot handle net map req, elec file "
			    "CRC mismatch (ours: %llx theirs: %llx)",
			    (unsigned long long)sys->conf_crc,
			    (unsigned long long)map->conf_crc);
#else	/* IBM */
			logMsg("Cannot handle net map req, elec file "
			    "CRC mismatch");
#endif	/* IBM */
		}
	} else {
		logMsg("Unknown or malformed req %x of length %d",
		    req->req, (int)sz);
	}
	mutex_exit(&sys->worker_interlock);
}

static void
send_xmit_data_conn(elec_sys_t *sys, net_conn_t *conn)
{
	size_t repsz;
	net_rep_comps_t *rep;

	ASSERT(sys != NULL);
	ASSERT_MUTEX_HELD(&sys->worker_interlock);
	ASSERT(conn != NULL);

	repsz = sizeof (net_rep_comps_t) +
	    conn->num_active * sizeof (net_comp_data_t);
	rep = safe_calloc(1, repsz);
	rep->version = LIBELEC_NET_VERSION;
	rep->rep = NET_REP_COMPS;
	rep->conf_crc = sys->conf_crc;
	for (unsigned i = 0, n = list_count(&sys->comps); i < n; i++) {
		const elec_comp_t *comp = sys->comps_array[i];

		if (NETMAPGET(conn->map, i)) {
			net_comp_data_t *data = &rep->comps[rep->n_comps++];
			ASSERT3U(rep->n_comps, <=, conn->num_active);

			data->idx = i;

			if (comp->ro.failed)
				data->flags |= LIBELEC_NET_FLAG_FAILED;
			if (comp->ro.shorted)
				data->flags |= LIBELEC_NET_FLAG_SHORTED;
			data->in_volts = clampi(round(comp->ro.in_volts *
			    NET_VOLTS_FACTOR), 0, UINT16_MAX);
			data->out_volts = clampi(round(comp->ro.out_volts *
			    NET_VOLTS_FACTOR), 0, UINT16_MAX);
			data->in_amps = clampi(round(comp->ro.in_amps *
			    NET_AMPS_FACTOR), 0, UINT16_MAX);
			data->out_amps = clampi(round(comp->ro.out_amps *
			    NET_AMPS_FACTOR), 0, UINT16_MAX);
			data->in_freq = clampi(round(comp->ro.in_freq *
			    NET_FREQ_FACTOR), 0, UINT16_MAX);
			data->out_freq = clampi(round(comp->ro.out_freq *
			    NET_FREQ_FACTOR), 0, UINT16_MAX);
			data->leak_factor = round(comp->ro.leak_factor * 10000);
		}
	}
	ASSERT3U(rep->n_comps, ==, conn->num_active);
	(void)netlink_sendto(NETLINK_PROTO_LIBELEC, rep, repsz,
	    conn->conn_id, 0);
}

static void
elec_net_send_update(elec_sys_t *sys)
{
	ASSERT(sys != NULL);

	sys->net_send.xmit_ctr = (sys->net_send.xmit_ctr + 1) % NET_XMIT_INTVAL;
	if (sys->net_send.xmit_ctr != 0)
		return;
	/*
	 * Sending data can kill the conn and remove it from the list,
	 * so be sure to grab the next conn pointer ahead of time.
	 */
	mutex_enter(&sys->worker_interlock);
	for (net_conn_t *conn = list_head(&sys->net_send.conns_list),
	    *next_conn = NULL; conn != NULL; conn = next_conn) {
		next_conn = list_next(&sys->net_send.conns_list, conn);
		send_xmit_data_conn(sys, conn);
	}
	mutex_exit(&sys->worker_interlock);
}

static void
handle_net_rep_comps(elec_sys_t *sys, const net_rep_comps_t *comps)
{
	ASSERT(sys != NULL);
	ASSERT(comps != NULL);

	NET_DBG_LOG("New dev data with %d comps", (int)comps->n_comps);

	for (unsigned i = 0; i < comps->n_comps; i++) {
		const net_comp_data_t *data = &comps->comps[i];
		elec_comp_t *comp;

		if (data->idx >= list_count(&sys->comps))
			continue;
		comp = sys->comps_array[data->idx];

		mutex_enter(&comp->rw_ro_lock);

		comp->rw.in_volts = (data->in_volts / NET_VOLTS_FACTOR);
		comp->rw.out_volts = (data->out_volts / NET_VOLTS_FACTOR);
		comp->rw.in_amps = (data->in_amps / NET_AMPS_FACTOR);
		comp->rw.out_amps = (data->out_amps / NET_AMPS_FACTOR);
		comp->rw.in_pwr = comp->rw.in_volts * comp->rw.in_amps;
		comp->rw.out_pwr = comp->rw.out_volts * comp->rw.out_amps;
		comp->rw.in_freq = (data->in_freq / NET_FREQ_FACTOR);
		comp->rw.out_freq = (data->out_freq / NET_FREQ_FACTOR);
		comp->rw.leak_factor = (data->leak_factor / 10000.0);
		comp->rw.failed = !!(data->flags & LIBELEC_NET_FLAG_FAILED);
		comp->rw.shorted = !!(data->flags & LIBELEC_NET_FLAG_SHORTED);

		comp->ro = comp->rw;

		mutex_exit(&comp->rw_ro_lock);
	}
}

static void
netlink_recv_msg_notif(netlink_conn_id_t conn_id, const void *buf, size_t sz,
    void *userinfo)
{
	elec_sys_t *sys;
	const net_rep_t *rep;
	const net_rep_comps_t *rep_comps;

	UNUSED(conn_id);
	ASSERT(buf != NULL);
	rep = buf;
	rep_comps = buf;
	ASSERT(userinfo != NULL);
	sys = userinfo;

	if (sz < sizeof (net_rep_t)) {
		logMsg("Received bad packet length %d", (int)sz);
		return;
	}
	if (rep->version != LIBELEC_NET_VERSION) {
		logMsg("Received bad version %d which doesn't "
		    "match ours (%d)", rep->version, LIBELEC_NET_VERSION);
		return;
	}
	if (rep->rep == NET_REP_COMPS &&
	    sz >= sizeof (net_rep_comps_t) &&
	    sz == sizeof (net_rep_comps_t) + rep_comps->n_comps *
	    sizeof (net_comp_data_t)) {
		if (rep_comps->conf_crc == sys->conf_crc) {
			handle_net_rep_comps(sys, rep_comps);
		} else {
#if	!IBM
			logMsg("Cannot handle rep COMPS, elec file "
			    "CRC mismatch (ours: %llx theirs: %llx)",
			    (unsigned long long)sys->conf_crc,
			    (unsigned long long)rep_comps->conf_crc);
#else	/* IBM */
			logMsg("Cannot handle rep COMPS, elec file "
			    "CRC mismatch");
#endif	/* IBM */
		}
	} else {
		logMsg("Unknown or malformed rep %x of length %d",
		    rep->rep, (int)sz);
	}
}

static void
elec_net_recv_update(elec_sys_t *sys)
{
	ASSERT(sys != NULL);
	if (netlink_started() && sys->net_recv.map_dirty) {
		mutex_enter(&sys->worker_interlock);
		if (send_net_recv_map(sys))
			sys->net_recv.map_dirty = false;
		mutex_exit(&sys->worker_interlock);
	}
}

static bool
send_net_recv_map(elec_sys_t *sys)
{
	net_req_map_t *req;
	bool res;

	ASSERT(sys != NULL);
	ASSERT(sys->started);
	ASSERT(sys->net_recv.active);
	req = safe_calloc(1, NETMAPSZ_REQ(sys));
	req->version = LIBELEC_NET_VERSION;
	req->req = NET_REQ_MAP;
	req->conf_crc = sys->conf_crc;
	memcpy(req->map, sys->net_recv.map, NETMAPSZ(sys));
	res = netlink_send(NETLINK_PROTO_LIBELEC, req, NETMAPSZ_REQ(sys), 0);
	ZERO_FREE(req);

	return (res);
}

static void
net_add_recv_comp(elec_comp_t *comp)
{
	elec_sys_t *sys;

	ASSERT(comp != NULL);
	sys = comp->sys;
	if (!sys->net_recv.active)
		return;
	ASSERT3U(comp->comp_idx, <, list_count(&sys->comps));
	if (!NETMAPGET(sys->net_recv.map, comp->comp_idx)) {
		mutex_enter(&sys->worker_interlock);
		if (!NETMAPGET(sys->net_recv.map, comp->comp_idx)) {
			NETMAPSET(sys->net_recv.map, comp->comp_idx);
			sys->net_recv.map_dirty = true;
		}
		mutex_exit(&sys->worker_interlock);
	}
}

#endif	/* defined(LIBELEC_WITH_NETLINK) */
