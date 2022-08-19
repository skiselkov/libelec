/*
 * CONFIDENTIAL
 *
 * Copyright 2022 Saso Kiselkov. All rights reserved.
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
#include <netlink.h>

#include "libelec.h"
#include "libelec_types_impl.h"

#define	EXEC_INTVAL		40000	/* us */
#define	MAX_NETWORK_DEPTH	100	/* dimensionless */
#define	NO_NEG_ZERO(x)		((x) == 0.0 ? 0.0 : (x))
#define	CB_SW_ON_DELAY		0.33	/* sec */
#define	MAX_COMPS		(UINT16_MAX + 1)

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

typedef struct {
	bool		pre;
	elec_user_cb_t	cb;
	void		*userinfo;
	avl_node_t	node;
} user_cb_info_t;

static const vect2_t batt_temp_energy_curve[] = {
    VECT2(C2KELVIN(-90), 0.01),
    VECT2(C2KELVIN(-75), 0.01),
    VECT2(C2KELVIN(-50), 0.125),
    VECT2(C2KELVIN(-20), 0.45),
    VECT2(C2KELVIN(-5), 0.7),
    VECT2(C2KELVIN(15), 0.925),
    VECT2(C2KELVIN(40), 1.0),
    VECT2(C2KELVIN(50), 1.0),
    NULL_VECT2
};

static elec_comp_info_t *infos_parse(const char *filename,
    const elec_func_bind_t *binds, size_t *num_infos);
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

static void netlink_send_msg_notif(nng_pipe pipe, const void *buf,
    size_t bufsz, void *userinfo);
static void netlink_recv_msg_notif(nng_pipe pipe, const void *buf,
    size_t bufsz, void *userinfo);

static void elec_net_send_update(elec_sys_t *sys);
static void elec_net_recv_update(elec_sys_t *sys);

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
gen_is_AC(const elec_comp_info_t *info)
{
	ASSERT(info != NULL);
	ASSERT3U(info->type, ==, ELEC_GEN);
	return (info->gen.freq != 0);
}

static bool
check_upstream(const elec_comp_t *comp, const elec_comp_t *src,
    const elec_comp_t *upstream)
{
	ASSERT(comp != NULL);
	ASSERT(src != NULL);
	ASSERT3U(src->src_idx, <, MAX_SRCS);
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
	for (unsigned i = 0; i < MAX_SRCS; i++)
		amps += link->out_amps[i];

	return (amps);
}

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

static elec_comp_t *
find_comp(elec_sys_t *sys, elec_comp_info_t *info, const elec_comp_t *src)
{
	elec_comp_t *comp;
	elec_comp_t srch = { .info = info };

	ASSERT(sys != NULL);
	ASSERT(info != NULL);
	ASSERT(src != NULL);

	comp = avl_find(&sys->info2comp, &srch, NULL);
	ASSERT_MSG(comp != NULL, "Component for info %s not found "
	    "(referenced from %s)", srch.info->name, src->info->name);

	return (comp);
}

static void
resolve_bus_links(elec_sys_t *sys, elec_comp_t *bus)
{
	ASSERT(sys != NULL);
	ASSERT(bus != NULL);
	ASSERT(bus->info != NULL);
	ASSERT3U(bus->info->type, ==, ELEC_BUS);

	for (int i = 0; bus->info->bus.comps[i] != NULL; i++)
		bus->n_links++;
	bus->links = safe_calloc(bus->n_links, sizeof (*bus->links));

	for (int i = 0; bus->info->bus.comps[i] != NULL; i++) {
		elec_comp_t *comp = find_comp(sys, bus->info->bus.comps[i],
		    bus);

		bus->links[i].comp = comp;
		ASSERT(comp->info != NULL);
		switch (comp->info->type) {
		case ELEC_BATT:
			/* Batteries are DC-only devices! */
			ASSERT(!bus->info->bus.ac);
			comp->links[0].comp = bus;
			break;
		case ELEC_GEN:
			/* Generators can be DC or AC */
			ASSERT3U(bus->info->bus.ac, ==, gen_is_AC(comp->info));
			comp->links[0].comp = bus;
			break;
		case ELEC_TRU:
			if (comp->info->tru.ac == bus->info) {
				ASSERT(bus->info->bus.ac);
				comp->links[0].comp = bus;
			} else {
				ASSERT3P(comp->info->tru.dc, ==, bus->info);
				ASSERT(!bus->info->bus.ac);
				comp->links[1].comp = bus;
			}
			break;
		case ELEC_LOAD:
			ASSERT3U(bus->info->bus.ac, ==, comp->info->load.ac);
			comp->links[0].comp = bus;
			break;
		case ELEC_BUS:
			VERIFY_MSG(0, "Invalid link: cannot connect bus %s "
			    "directly to bus %s", bus->info->name,
			    comp->info->name);
			break;
		case ELEC_CB:
		case ELEC_SHUNT:
			/* 3-phase breakers are only allowed on AC buses */
			ASSERT(!comp->info->cb.triphase || bus->info->bus.ac);
			if (comp->links[0].comp == NULL) {
				comp->links[0].comp = bus;
			} else {
				elec_comp_t *other_bus = comp->links[0].comp;

				ASSERT_MSG(comp->links[1].comp == NULL,
				    "Too many connections to %s",
				    comp->info->name);
				comp->links[1].comp = bus;
				ASSERT_MSG(bus->info->bus.ac ==
				    other_bus->info->bus.ac, "%s is linking "
				    "two buses of incompatible type (%s is "
				    "%s and %s is %s)",
				    comp->info->name, bus->info->name,
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
			ASSERT(!bus->info->bus.ac);
			if (comp->info->diode.sides[0] == bus->info) {
				comp->links[0].comp = bus;
			} else {
				ASSERT3P(comp->info->diode.sides[1], ==,
				    bus->info);
				comp->links[1].comp = bus;
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
		if (comp->info->type == ELEC_BUS) {
			resolve_bus_links(sys, comp);
		} else if (comp->info->type == ELEC_TRU &&
		    comp->info->tru.charger) {
			comp->tru.batt = find_comp(sys,
			    comp->info->tru.batt, comp);
			comp->tru.batt_conn = find_comp(sys,
			    comp->info->tru.batt_conn, comp);
		}
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
		for (unsigned i = 0; i < comp->n_links; i++)
			CHECK_LINK(comp->links[i].comp);
	}
#undef	CHECK_LINK
}

elec_comp_info_t *
libelec_get_comp_infos(const elec_sys_t *sys, size_t *num_infos)
{
	ASSERT(sys != NULL);
	ASSERT(num_infos != NULL);
	*num_infos = sys->num_infos;
	return (sys->comp_infos);
}

elec_sys_t *
libelec_new(const char *filename, const elec_func_bind_t *binds)
{
	elec_sys_t *sys = safe_calloc(1, sizeof (*sys));
	unsigned src_i = 0, comp_i = 0;
	void *buf;
	size_t bufsz;

	ASSERT(filename != NULL);
	/* binds can be NULL */

	buf = file2buf(filename, &bufsz);
	if (buf == NULL) {
		logMsg("Can't open %s: %s", filename, strerror(errno));
		ZERO_FREE(sys);
		return (NULL);
	}
	sys->conf_crc = crc64(buf, bufsz);
	free(buf);

	sys->comp_infos = infos_parse(filename, binds, &sys->num_infos);
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
		elec_comp_t *comp = safe_calloc(1, sizeof (*comp));
		avl_index_t where;
		elec_comp_info_t *info = &sys->comp_infos[i];

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

		/* Initialize component */
		if (comp->info->type == ELEC_LOAD) {
			comp->load.random_load_factor = 1;
		} else if (comp->info->type == ELEC_BATT ||
		    comp->info->type == ELEC_GEN ||
		    comp->info->type == ELEC_TRU) {
			comp->src_idx = src_i;
			src_i++;
		}
		if (comp->info->type == ELEC_BATT ||
		    comp->info->type == ELEC_GEN) {
			list_insert_tail(&sys->gens_batts, comp);
		} else if (comp->info->type == ELEC_TIE) {
			list_insert_tail(&sys->ties, comp);
		}
		/* Initialize links */
		switch (comp->info->type) {
		case ELEC_BATT:
		case ELEC_GEN:
		case ELEC_LOAD:
			comp->n_links = 1;
			comp->links = safe_calloc(1, sizeof (*comp->links));
			break;
		case ELEC_TRU:
		case ELEC_CB:
		case ELEC_SHUNT:
		case ELEC_DIODE:
			comp->n_links = 2;
			comp->links = safe_calloc(2, sizeof (*comp->links));
			break;
		default:
			break;
		}
		/* Validate info structure */
		switch (comp->info->type) {
		case ELEC_BATT:
			ASSERT3F(comp->info->batt.volts, >, 0);
			ASSERT3F(comp->info->batt.max_pwr, >, 0);
			ASSERT3F(comp->info->batt.capacity, >=, 0);
			ASSERT3F(comp->info->batt.chg_R, >, 0);
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
			ASSERT(comp->info->gen.get_rpm != NULL ||
			    binds == NULL);

			comp->gen.ctr_rpm = AVG(comp->info->gen.min_rpm,
			    comp->info->gen.max_rpm);
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
		case ELEC_SHUNT:
			comp->scb.cur_set = comp->scb.wk_set = true;
			break;
		case ELEC_TIE:
			mutex_init(&comp->tie.lock);
			break;
		case ELEC_DIODE:
			ASSERT(comp->info->diode.sides[0] != NULL);
			ASSERT(comp->info->diode.sides[1] != NULL);
			break;
		case ELEC_LABEL_BOX:
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
	if (sys->net_recv.active) {
		memset(sys->net_recv.map, 0, NETMAPSZ(sys));
		send_net_recv_map(sys);
	}
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
		SERIALIZE_DATA_V(&comp->scb, ser, "%s/%s/cb",
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
		DESERIALIZE_DATA_V(&comp->scb, ser, "%s/%s/cb",
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

void
libelec_serialize(elec_sys_t *sys, conf_t *ser, const char *prefix)
{
	ASSERT(sys != NULL);
	ASSERT(ser != NULL);
	ASSERT(prefix != NULL);
	ASSERT(!sys->net_recv.active);

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
	ASSERT(!sys->net_recv.active);

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

void
libelec_destroy(elec_sys_t *sys)
{
	elec_comp_t *comp;
	user_cb_info_t *ucbi;
	void *cookie;

	ASSERT(sys != NULL);

	/* libelec_sys_stop MUST be called first! */
	ASSERT(!sys->started);

	libelec_disable_net_send(sys);
	libelec_disable_net_recv(sys);

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
bind_list_find(const elec_func_bind_t *binds, const char *name, void *ptr)
{
	void **pp = (void **)ptr;

	if (binds == NULL) {
		*pp = NULL;
		return (true);
	}
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

static gui_load_type_t
str2load_type(const char *str)
{
	ASSERT(str != NULL);
	if (strcmp(str, "MOTOR") == 0)
		return (GUI_LOAD_MOTOR);
	return (GUI_LOAD_GENERIC);
}

static elec_comp_info_t *
infos_parse(const char *filename, const elec_func_bind_t *binds,
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
			info->int_R = 1;
		} else if (strcmp(cmd, "GEN") == 0 && n_comps == 3) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_GEN;
			info->name = strdup(comps[1]);
			info->gen.freq = atof(comps[2]);
			info->int_R = 1;
		} else if (strcmp(cmd, "TRU") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_TRU;
			info->name = strdup(comps[1]);
			info->int_R = 1;
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
		} else if ((strcmp(cmd, "CB") == 0 ||
		    strcmp(cmd, "CB3") == 0) && n_comps == 3) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_CB;
			info->name = strdup(comps[1]);
			info->cb.rate = 4;
			info->cb.max_amps = atof(comps[2]);
			info->cb.triphase = (strcmp(cmd, "CB3") == 0);
		} else if (strcmp(cmd, "SHUNT") == 0 && n_comps == 2) {
			ASSERT3U(comp_i, <, num_comps);
			CHECK_DUP_NAME(comps[1]);
			info = &infos[comp_i++];
			info->type = ELEC_SHUNT;
			info->name = strdup(comps[1]);
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
		} else if (strcmp(cmd, "LABEL_BOX") == 0 && n_comps >= 7) {
			size_t sz = 0;
			ASSERT3U(comp_i, <, num_comps);
			info = &infos[comp_i++];
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
			info->gen.stab_rate_U = atof(comps[1]);
		} else if (strcmp(cmd, "STAB_RATE_F") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			ASSERT(info->gen.freq != 0);
			info->gen.stab_rate_f = atof(comps[1]);
		} else if (strcmp(cmd, "EXC_RPM") == 0 && n_comps == 2 &&
		    info != NULL && info->type == ELEC_GEN) {
			info->gen.exc_rpm = atof(comps[1]);
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
		} else if ((strcmp(cmd, "LOADCB") == 0 ||
		    strcmp(cmd, "LOADCB3") == 0) && (n_comps == 2 ||
		    n_comps == 3) && info != NULL && info->type == ELEC_LOAD) {
			elec_comp_info_t *cb, *bus;

			ASSERT3U(comp_i + 1, <, num_comps);
			cb = &infos[comp_i++];
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
			if (info->batt.chg_R <= 0) {
				logMsg("%s:%d: charge resistance must be "
				    "positive", filename, linenum);
				free_strlist(comps, n_comps);
				goto errout;
			}
		} else if (strcmp(cmd, "INT_R") == 0 && n_comps == 2 &&
		    info != NULL && (info->type == ELEC_BATT ||
		    info->type == ELEC_GEN || info->type == ELEC_TRU)) {
			info->int_R = atof(comps[1]);
			if (info->int_R <= 0) {
				logMsg("%s:%d: internal resistance must be "
				    "positive", filename, linenum);
				free_strlist(comps, n_comps);
				goto errout;
			}
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

static void
infos_free(elec_comp_info_t *infos, size_t num_infos)
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
		return (comp->n_links);
	case ELEC_TIE:
		return (comp->n_links);
	case ELEC_TRU:
	case ELEC_CB:
	case ELEC_SHUNT:
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
	ASSERT3U(i, <, comp->n_links);
	/* Electrical configuration is immutable, no need to lock */
	return (comp->links[i].comp);
}

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

double
libelec_comp_get_incap_volts(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_LOAD);
	return (comp->load.incap_U);
}

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

bool
libelec_comp_is_powered(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	return (libelec_comp_get_out_volts(comp) != 0);
}

void
libelec_comp_set_failed(elec_comp_t *comp, bool failed)
{
	ASSERT(comp != NULL);
	mutex_enter(&comp->rw_ro_lock);
	comp->ro.failed = failed;
	mutex_exit(&comp->rw_ro_lock);
}

bool
libelec_comp_get_failed(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	NET_ADD_RECV_COMP(comp);
	return (comp->ro.failed);
}

void
libelec_comp_set_shorted(elec_comp_t *comp, bool shorted)
{
	ASSERT(comp != NULL);
	mutex_enter(&comp->rw_ro_lock);
	comp->ro.shorted = shorted;
	mutex_exit(&comp->rw_ro_lock);
}

bool
libelec_comp_get_shorted(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	NET_ADD_RECV_COMP(comp);
	return (comp->ro.shorted);
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
			    wavg(0.9, 0.99, crc64_rand_fract());
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
		elec_comp_t *srcs_vis[MAX_SRCS];

		memcpy(srcs_vis, comp->srcs, sizeof (srcs_vis));

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
		memcpy(comp->srcs_vis, srcs_vis, sizeof (comp->srcs_vis));
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
#ifndef	LIBELEC_NO_LIBSWITCH
			if (comp->scb.sw != NULL) {
				comp->scb.cur_set =
				    !libswitch_read(comp->scb.sw, NULL);
			}
#endif	/* LIBELEC_NO_LIBSWITCH */
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
	double rpm;

	ASSERT(gen != NULL);
	ASSERT(gen->info != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);
	ASSERT(gen->info->gen.get_rpm != NULL);

	rpm = gen->info->gen.get_rpm(gen);
	ASSERT(!isnan(rpm));
	rpm = MAX(rpm, 1e-3);
	gen->gen.rpm = rpm;
	if (rpm <= 1e-3) {
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
		double stab_factor_U = clamp(gen->gen.ctr_rpm / rpm,
		    gen->gen.min_stab_U, gen->gen.max_stab_U);
		double stab_rate_mod =
		    clamp(1 + crc64_rand_normal(0.1), 0.1, 10);
		FILTER_IN(gen->gen.stab_factor_U, stab_factor_U, d_t,
		    gen->info->gen.stab_rate_U * stab_rate_mod);
	} else {
		gen->gen.stab_factor_U = 1;
	}
	if (gen->info->gen.stab_rate_f > 0) {
		double stab_factor_f = clamp(gen->gen.ctr_rpm / rpm,
		    gen->gen.min_stab_f, gen->gen.max_stab_f);
		double stab_rate_mod =
		    clamp(1 + crc64_rand_normal(0.1), 0.1, 10);
		FILTER_IN(gen->gen.stab_factor_f, stab_factor_f, d_t,
		    gen->info->gen.stab_rate_f * stab_rate_mod);
	} else {
		gen->gen.stab_factor_f = 1;
	}
	if (!gen->rw.failed) {
		if (rpm < gen->info->gen.exc_rpm) {
			gen->rw.in_volts = 0;
			gen->rw.in_freq = 0;
		} else {
			gen->rw.in_volts = (rpm / gen->gen.ctr_rpm) *
			    gen->gen.stab_factor_U * gen->info->gen.volts;
			gen->rw.in_freq = (rpm / gen->gen.ctr_rpm) *
			    gen->gen.stab_factor_f * gen->info->gen.freq;
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
	ASSERT(batt->info->batt.get_temp != NULL);

	batt->batt.T = batt->info->batt.get_temp(batt);
	ASSERT3F(batt->batt.T, >, 0);
	temp_coeff = fx_lin_multi(batt->batt.T, batt_temp_energy_curve, true);

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
#ifndef	LIBELEC_NO_LIBSWITCH
		/* Also pull the switch if one is present */
		if (cb->scb.sw != NULL)
			libswitch_set(cb->scb.sw, true);
#endif	/* LIBELEC_NO_LIBSWITCH */
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
	if (!tru->info->tru.charger) {
		/*
		 * In simple TRU operating mode, we operate as a fixed-rate
		 * transformer. Our input voltage is directly proportional
		 * to our output and we provide no output voltage regulation.
		 */
		tru->tru.chgr_regul = 1;
	} else {
		/*
		 * In charger mode, we control an additional voltage
		 * regulator parameter that lets us adjust our output
		 * voltage as necessary to stabilize the charging current.
		 */
		ASSERT3F(tru->info->tru.curr_lim, >, 0);
		double oc_ratio = tru->tru.prev_amps / tru->info->tru.curr_lim;
		double regul_tgt = clamp(oc_ratio > 0 ?
		    tru->tru.chgr_regul / oc_ratio : 1, 0, 1);
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
			tru->tru.chgr_regul = 0;
		} else if (regul_tgt > tru->tru.chgr_regul) {
			FILTER_IN(tru->tru.chgr_regul, regul_tgt, d_t, 1);
		} else {
			FILTER_IN(tru->tru.chgr_regul, regul_tgt, d_t, 2 * d_t);
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
		if (comp->info->type == ELEC_GEN)
			network_update_gen(comp, d_t);
		else if (comp->info->type == ELEC_BATT)
			network_update_batt(comp, d_t);
		else if (comp->info->type == ELEC_TRU)
			network_update_tru(comp, d_t);
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
	ASSERT3U(comp->n_srcs, <, MAX_SRCS);
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
	if (upstream == comp->links[0].comp) {
		add_src_up(comp, src, upstream);
		ASSERT_MSG(comp->n_srcs == 1, "%s attempted to add a second "
		    "AC power source ([0]=%s, [1]=%s). Multi-source feeding "
		    "is NOT supported in AC networks.", comp->info->name,
		    comp->srcs[0]->info->name, comp->srcs[1]->info->name);
		if (!comp->rw.failed) {
			if (comp->rw.in_volts < src->rw.out_volts) {
				comp->rw.in_volts = src->rw.out_volts;
				comp->rw.in_freq = src->rw.out_freq;
				comp->rw.out_volts = comp->tru.chgr_regul *
				    comp->info->tru.out_volts *
				    (comp->rw.in_volts /
				    comp->info->tru.in_volts);
			}
		} else {
			comp->rw.in_volts = 0;
			comp->rw.in_freq = 0;
			comp->rw.out_volts = 0;
		}
		ASSERT(comp->links[1].comp != NULL);
		/*
		 * The TRU becomes the source for downstream buses.
		 */
		network_paint_src_comp(comp, comp, comp->links[1].comp,
		    depth + 1);
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
			ASSERT3U(gen_is_AC(src->info), ==, comp->info->bus.ac);
		}
		network_paint_src_bus(src, upstream, comp, depth);
		break;
	case ELEC_TRU:
		network_paint_src_tru(src, upstream, comp, depth);
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
network_load_integrate_tru(const elec_comp_t *src, const elec_comp_t *upstream,
    elec_comp_t *comp, unsigned depth, double d_t)
{
	ASSERT(src != NULL);
	ASSERT(upstream != NULL);
	ASSERT(comp != NULL);
	ASSERT(comp->links[0].comp != NULL);
	ASSERT(comp->links[1].comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_TRU);
	ASSERT3U(depth, <, MAX_NETWORK_DEPTH);

	if (upstream != comp->links[0].comp)
		return (0);

	/* When hopping over to the DC network, the TRU becomes the src */
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
		if (info->load.get_load != NULL)
			load_WorI += info->load.get_load(comp);
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
	    src->info->type == ELEC_TRU);
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
		ASSERT3P(upstream, ==, comp->links[0].comp);
		return (network_load_integrate_tru(src, upstream, comp,
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
	char spaces[2 * depth + 1];
	double W;

	ASSERT(comp != NULL);

	mk_spaces(spaces, 2 * depth + 1);
	W = out_data ? (comp->rw.out_volts * comp->rw.out_amps) :
	    (comp->rw.in_volts * comp->rw.in_amps);
	logMsg("%s%-5s  %s  %3s: %.2fW  LOADS: %.2fW",
	    spaces, comp_type2str(comp->info->type), comp->info->name,
	    out_data ? "OUT" : "IN", W, load);
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
	if (sys->net_recv.active) {
		elec_net_recv_update(sys);
		return (true);
	}
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

	elec_net_send_update(sys);

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

bool
libelec_cb_get(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	/* atomic read, no need to lock */
	return (comp->scb.cur_set);
}

double
libelec_cb_get_temp(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	ASSERT(comp->info != NULL);
	ASSERT3U(comp->info->type, ==, ELEC_CB);
	/* atomic read, no need to lock */
	return (comp->scb.temp);
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
			if (comp->links[j].comp->info == bus_list[i]) {
				new_state[j] = true;
				break;
			}
		}
		ASSERT_MSG(j != comp->n_links,
		    "Tie %s is not connected to bus %s", comp->info->name,
		    bus_list[i]->name);
	}

	mutex_enter(&comp->tie.lock);
	memcpy(comp->tie.cur_state, new_state,
	    comp->n_links * sizeof (*comp->tie.cur_state));
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

	if (comp->ro.failed)
		return;

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

	if (comp->ro.failed)
		return;

	mutex_enter(&comp->tie.lock);
	for (unsigned i = 0; i < comp->n_links; i++)
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
	for (unsigned i = 0; tied && i < comp->n_links; i++)
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
	for (unsigned i = 0; i < comp->n_links; i++) {
		if (comp->tie.cur_state[i])
			bus_list[n_buses++] = comp->links[i].comp;
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
	return (comp->n_links);
}

bool
libelec_tie_get_list(elec_comp_t *tie, bool_t exclusive, ...)
{
	va_list ap;
	bool res;

	ASSERT(tie != NULL);
	ASSERT3U(tie->info->type, ==, ELEC_TIE);
	va_start(ap, exclusive);
	res = libelec_tie_get_list_v(tie, exclusive, ap);
	va_end(ap);

	return (res);
}

bool
libelec_tie_get_list_v(elec_comp_t *tie, bool exclusive, va_list ap)
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

double
libelec_batt_get_chg_rel(const elec_comp_t *batt)
{
	ASSERT(batt != NULL);
	ASSERT(batt->info != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);
	return (batt->batt.chg_rel);
}

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

double
libelec_phys_get_batt_voltage(double U_nominal, double chg_rel, double I_rel)
{
	static const vect2_t chg_volt_curve[] = {
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
	ASSERT3F(U_nominal, >, 0);
	ASSERT3F(chg_rel, >=, 0);
	/*
	 * Small numerical precision errors during a state restore can cause
	 * this value to go slightly over '1'. Ignore those cases.
	 */
	ASSERT3F(chg_rel, <=, 1.0001);
	I_rel = clamp(I_rel, 0, 1);
	return (U_nominal * (1 - clamp(pow(I_rel, 1.45), 0, 1)) *
	    fx_lin_multi(chg_rel, chg_volt_curve, true));
}

static void
kill_conn(elec_sys_t *sys, net_conn_t *conn)
{
	ASSERT(sys != NULL);
	ASSERT_MUTEX_HELD(&sys->worker_interlock);
	ASSERT(conn != NULL);

	list_remove(&sys->net_send.conns_list, conn);
	htbl_remove(&sys->net_send.conns, &conn->pipe, false);

	free(conn->map);
	ZERO_FREE(conn);
}

static void
pipe_conn_notif(nng_pipe pipe, nng_pipe_ev ev, void *userinfo)
{
	elec_sys_t *sys;

	UNUSED(pipe);
	UNUSED(ev);
	ASSERT(userinfo != NULL);
	sys = userinfo;

	mutex_enter(&sys->worker_interlock);
	if (sys->net_recv.active && sys->started)
		send_net_recv_map(sys);
	mutex_exit(&sys->worker_interlock);
}

static void
pipe_discon_notif(nng_pipe pipe, nng_pipe_ev ev, void *userinfo)
{
	elec_sys_t *sys;
	net_conn_t *conn;

	UNUSED(ev);
	ASSERT(userinfo != NULL);
	sys = userinfo;

	mutex_enter(&sys->worker_interlock);
	conn = htbl_lookup(&sys->net_send.conns, &pipe);
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
	htbl_create(&sys->net_send.conns, 128, sizeof (nng_pipe), false);
	list_create(&sys->net_send.conns_list, sizeof (net_conn_t),
	    offsetof(net_conn_t, node));

	sys->net_send.proto.proto_id = NETLINK_PROTO_LIBELEC;
	sys->net_send.proto.name = "libelec";
	sys->net_send.proto.msg_rcvd_notif = netlink_send_msg_notif;
	sys->net_send.proto.pipe_rem_post_notif = pipe_discon_notif;
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
	sys->net_recv.proto.pipe_add_post_notif = pipe_conn_notif;
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
get_net_conn(elec_sys_t *sys, nng_pipe pipe)
{
	net_conn_t *conn;

	ASSERT(sys != NULL);
	ASSERT_MUTEX_HELD(&sys->worker_interlock);

	conn = htbl_lookup(&sys->net_send.conns, &pipe);
	if (conn == NULL) {
		conn = safe_calloc(1, sizeof (*conn));
		conn->pipe = pipe;
		conn->map = safe_calloc(NETMAPSZ(sys), sizeof (*conn->map));
		delay_line_init(&conn->kill_delay, SEC2USEC(20));
		htbl_set(&sys->net_send.conns, &pipe, conn);
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
netlink_send_msg_notif(nng_pipe pipe, const void *buf, size_t sz,
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
	conn = get_net_conn(sys, pipe);
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
	(void)netlink_sendto(NETLINK_PROTO_LIBELEC, rep, repsz, conn->pipe,
	    NETLINK_FLAG_NONBLOCK);
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
netlink_recv_msg_notif(nng_pipe pipe, const void *buf, size_t sz,
    void *userinfo)
{
	elec_sys_t *sys;
	const net_rep_t *rep;
	const net_rep_comps_t *rep_comps;

	UNUSED(pipe);
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
	res = netlink_send(NETLINK_PROTO_LIBELEC, req, NETMAPSZ_REQ(sys),
	    NETLINK_FLAG_NONBLOCK);
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
