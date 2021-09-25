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

#ifndef	__LIBELEC_TYPES_IMPL_H__
#define	__LIBELEC_TYPES_IMPL_H__

#ifdef	XPLANE
#include <acfutils/dr.h>
#endif
#include <acfutils/list.h>
#include <acfutils/worker.h>
#include <acfutils/thread.h>

#include <abus_ser.h>
#ifndef	LIBELEC_NO_LIBSWITCH
#include <libswitch.h>
#endif

#include "libelec.h"

#ifdef	__cplusplus
extern "C" {
#endif

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

	const elec_comp_info_t	*comp_infos;	/* immutable */
	size_t			num_infos;
};

typedef struct {
	elec_comp_t	*bus;
	SERIALIZE_START_MARKER;
	double		prev_amps;
	double		chg_rel;
	double		rechg_W;
	SERIALIZE_END_MARKER;
	double		T;		/* Kelvin */
} elec_batt_t;

typedef struct {
	elec_comp_t	*bus;
	double		ctr_rpm;
	double		min_stab_U;
	double		max_stab_U;
	double		min_stab_f;
	double		max_stab_f;
	double		eff;
	double		rpm;
	SERIALIZE_START_MARKER;
	double		stab_factor_U;
	double		stab_factor_f;
	SERIALIZE_END_MARKER;
} elec_gen_t;

typedef struct {
	elec_comp_t	*ac;
	elec_comp_t	*dc;
	elec_comp_t	*batt;
	elec_comp_t	*batt_conn;
	double		prev_amps;
	double		chgr_regul;
	double		eff;
} elec_tru_t;

typedef struct {
	elec_comp_t	*bus;
	SERIALIZE_START_MARKER;
	/* Virtual input capacitor voltage */
	double		incap_U;
	double		random_load_factor;
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
} elec_scb_t;

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

	mutex_t			rw_ro_lock;

	SERIALIZE_START_MARKER;
	struct {
		double		in_volts;
		double		out_volts;
		double		in_amps;
		double		out_amps;
		double		short_amps;
		double		in_pwr;		/* Watts */
		double		out_pwr;	/* Watts */
		double		in_freq;	/* Hz */
		double		out_freq;	/* Hz */
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
	/*
	 * Version for visualizer to avoid blinking when `src' gets reset.
	 */
	elec_comp_t		*src_vis;

	union {
		elec_batt_t	batt;
		elec_gen_t	gen;
		elec_tru_t	tru;
		elec_load_t	load;
		elec_bus_t	bus;
		elec_scb_t	scb;
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

#ifdef	__cplusplus
}
#endif

#endif	/* __LIBELEC_TYPES_IMPL_H__ */
