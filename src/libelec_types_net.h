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

#ifndef	__LIBELEC_TYPES_NET_H__
#define	__LIBELEC_TYPES_NET_H__

#include <stdint.h>

#include <acfutils/delay_line.h>
#include <acfutils/list.h>

#include <netlink.h>

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	nng_pipe		pipe;
	uint8_t			*map;	/* NETMAPSZ bytes */
	unsigned		num_active;
	delay_line_t		kill_delay;
	list_node_t		node;	/* net_send.conns_list node */
} net_conn_t;

#define	NET_REQ_MAP		0x0001	/* net_req_map_t */

typedef struct {
	uint16_t		version;
	uint16_t		req;
} net_req_t;

typedef struct {
	uint16_t		version;
	uint16_t		req;
	uint64_t		conf_crc;
	uint8_t			map[0];	/* variable length */
} net_req_map_t;

enum {
    LIBELEC_NET_FLAG_FAILED =	1 << 0,
    LIBELEC_NET_FLAG_SHORTED =	1 << 1
};

#define	NET_REP_COMPS		0x0001		/* net_rep_comps_t */

typedef struct {
	uint16_t		version;
	uint16_t		rep;
} net_rep_t;

typedef struct {
	uint16_t		idx;		/* component index */
	uint16_t		flags;		/* LIBELEC_NET_FLAG_* mask */
	uint16_t		in_volts;	/* 0.05 V */
	uint16_t		out_volts;	/* 0.05 V */
	uint16_t		in_amps;	/* 0.025 Amps */
	uint16_t		out_amps;	/* 0.025 Amps */
	uint16_t		in_freq;	/* 0.05 Hz */
	uint16_t		out_freq;	/* 0.05 Hz */
	uint16_t		leak_factor;	/* 1/10000th */
} net_comp_data_t;

typedef struct {
	uint16_t		version;
	uint16_t		rep;
	uint64_t		conf_crc;
	uint16_t		n_comps;
	net_comp_data_t		comps[0];	/* variable length */
} net_rep_comps_t;

#ifdef	__cplusplus
}
#endif

#endif	/* __LIBELEC_TYPES_NET_H__ */
