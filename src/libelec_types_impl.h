/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */

#ifndef	__LIBELEC_TYPES_IMPL_H__
#define	__LIBELEC_TYPES_IMPL_H__

#ifdef	XPLANE
#include <acfutils/dr.h>
#endif
#include <acfutils/htbl.h>
#include <acfutils/list.h>
#include <acfutils/worker.h>
#include <acfutils/thread.h>

#ifdef	LIBELEC_WITH_LIBSWITCH
#include <libswitch.h>
#endif

#ifdef	LIBELEC_WITH_NETLINK
#include "libelec_types_net.h"
#endif

#include "libelec.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	LIBELEC_SER_START_MARKER \
	ALIGN_ATTR(16) int	__serialize_start_marker[1]
#define	LIBELEC_SER_END_MARKER	\
	int	__serialize_end_marker[1]
#define	LIBELEC_SERIALIZE_DATA_V(data, ser, key, ...) \
	conf_set_data_v((ser), (key), &(data)->__serialize_start_marker, \
	    ((uintptr_t)&(data)->__serialize_end_marker) - \
	    ((uintptr_t)&(data)->__serialize_start_marker), __VA_ARGS__)
#define	LIBELEC_DESERIALIZE_DATA_V(data, ser, key, ...) \
	do { \
		size_t __len = ((uintptr_t)&(data)->__serialize_end_marker) - \
		    ((uintptr_t)&(data)->__serialize_start_marker); \
		void *tmpbuf = safe_malloc(__len); \
		size_t __act_len = conf_get_data_v((ser), (key), tmpbuf, \
		    __len, __VA_ARGS__); \
		if (__act_len != __len) { \
			logMsg("Error deserializing key " key ": " \
			    "data length mismatch, wanted %d bytes, got %d", \
			    __VA_ARGS__, (int)__len, (int)__act_len); \
			free(tmpbuf); \
			return (false); \
		} \
		memcpy(&(data)->__serialize_start_marker, tmpbuf, __len); \
		free(tmpbuf); \
	} while (0)


struct elec_sys_s {
	bool		started;
	worker_t	worker;
	mutex_t		worker_interlock;

	mutex_t		paused_lock;
	bool		paused;		/* protected by paused_lock */
	double		time_factor;	/* only accessed from main thread */
	uint64_t	prev_clock;
#ifdef	XPLANE
	double		prev_sim_time;
	struct {
		dr_t	paused;
		dr_t	replay;
		dr_t	sim_speed_act;
		dr_t	sim_time;
	} drs;
#endif	/* defined(XPLANE) */

	char		*conf_filename;
	uint64_t	conf_crc;

	avl_tree_t	info2comp;
	avl_tree_t	name2comp;

	mutex_t		user_cbs_lock;
	avl_tree_t	user_cbs;

	list_t		comps;
	elec_comp_t	**comps_array;		/* length list_count(&comps) */
	list_t		gens_batts;
	list_t		ties;

	elec_comp_info_t	*comp_infos;	/* immutable after parse */
	size_t			num_infos;
#ifdef	LIBELEC_WITH_NETLINK
	struct {
		bool		active;
		/* protected by worker_interlock */
		htbl_t		conns;		/* list of net_conn_t's */
		list_t		conns_list;
		/* only accessed from worker thread */
		unsigned	xmit_ctr;
		netlink_proto_t	proto;
	} net_send;
	struct {
		bool		active;
		uint8_t		*map;
		bool		map_dirty;
		netlink_proto_t	proto;
	} net_recv;
#endif	/* defined(LIBELEC_WITH_NETLINK) */
};

typedef struct {
	LIBELEC_SER_START_MARKER;
	double		prev_amps;
	double		chg_rel;
	double		rechg_W;
	LIBELEC_SER_END_MARKER;
	mutex_t		lock;
	double		T;		/* Kelvin, protected by `lock` above */
} elec_batt_t;

typedef struct {
	double		ctr_rpm;
	double		min_stab_U;
	double		max_stab_U;
	double		min_stab_f;
	double		max_stab_f;
	double		eff;
	mutex_t		lock;
	double		rpm;		/* protected by `lock` above */
	LIBELEC_SER_START_MARKER;
	double		tgt_volts;
	double		tgt_freq;
	double		stab_factor_U;
	double		stab_factor_f;
	LIBELEC_SER_END_MARKER;
} elec_gen_t;

typedef struct {
	elec_comp_t	*batt;
	elec_comp_t	*batt_conn;
	double		prev_amps;
	double		regul;
	double		eff;
} elec_tru_t;

typedef struct {
	elec_comp_t	*bus;
	LIBELEC_SER_START_MARKER;
	/* Virtual input capacitor voltage */
	double		incap_U;
	double		random_load_factor;
	LIBELEC_SER_END_MARKER;
	/* Change of input capacitor charge */
	double		incap_d_Q;

	bool		seen;
} elec_load_t;

typedef struct {
#ifdef	LIBELEC_WITH_LIBSWITCH
	switch_t	*sw;		/* optional libswitch link */
#endif
	LIBELEC_SER_START_MARKER;
	bool_t		cur_set;
	bool_t		wk_set;
	double		temp;		/* relative 0.0 - 1.0 */
	LIBELEC_SER_END_MARKER;
} elec_scb_t;

typedef struct {
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
	elec_comp_t		*comp;
	double			out_amps[ELEC_MAX_SRCS];
	elec_comp_t		*srcs[ELEC_MAX_SRCS];
} elec_link_t;

struct elec_comp_s {
	elec_sys_t		*sys;
	elec_comp_info_t	*info;

	/*
	 * Bitmask that gets checked by the network state integrator
	 * and reset in network_reset. This is used to prevent double-
	 * accounting of buses.
	 */
	uint64_t		integ_mask;
	elec_link_t		*links;
	unsigned		n_links;
	unsigned		src_idx;
	unsigned		comp_idx;

	mutex_t			rw_ro_lock;

	LIBELEC_SER_START_MARKER;
	struct {
		double		in_volts;
		double		out_volts;
		double		in_amps;
		double		out_amps;
		double		short_amps;
		double		in_pwr;			/* Watts */
		double		out_pwr;		/* Watts */
		double		in_freq;		/* Hz */
		double		out_freq;		/* Hz */
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
	LIBELEC_SER_END_MARKER;

	double			src_int_cond_total; /* Conductance, abstract */
	uint64_t		src_mask;
	elec_comp_t		*srcs[ELEC_MAX_SRCS];
	unsigned		n_srcs;
	/*
	 * Version for external consumers, which is only updated after a
	 * network integration pass. This avoids e.g. blinking when the
	 * when `srcs' array gets reset during the integration pass.
	 * Protected by rw_ro_lock.
	 */
	elec_comp_t		*srcs_ext[ELEC_MAX_SRCS];

	union {
		elec_batt_t	batt;
		elec_gen_t	gen;
		elec_tru_t	tru;
		elec_load_t	load;
		elec_scb_t	scb;
		elec_tie_t	tie;
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
	list_node_t		gens_batts_node;
	list_node_t		ties_node;
	avl_node_t		info2comp_node;
	avl_node_t		name2comp_node;
};

#ifdef	__cplusplus
}
#endif

#endif	/* __LIBELEC_TYPES_IMPL_H__ */
