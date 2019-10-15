/*
 * CONFIDENTIAL
 *
 * Copyright 2019 Saso Kiselkov. All rights reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property
 * of Saso Kiselkov. The intellectual and technical concepts contained
 * herein are proprietary to Saso Kiselkov and may be covered by U.S. and
 * Foreign Patents, patents in process, and are protected by trade secret
 * or copyright law. Dissemination of this information or reproduction of
 * this material is strictly forbidden unless prior written permission is
 * obtained from Saso Kiselkov.
 */

#ifndef	_LIBELEC_H_
#define	_LIBELEC_H_

#include <stdarg.h>

#include <acfutils/geom.h>
#include <acfutils/sysmacros.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct elec_sys_s elec_sys_t;
typedef struct elec_comp_s elec_comp_t;
typedef struct elec_comp_info_s elec_comp_info_t;

typedef enum {
	ELEC_BATT,
	ELEC_GEN,
	ELEC_TRU,
	ELEC_LOAD,
	ELEC_BUS,
	ELEC_CB,
	ELEC_TIE,
	ELEC_DIODE
} elec_comp_type_t;

typedef struct {
	double	volts;		/* nominal voltage */
	double	capacity;	/* capacity in Joules at 15C */
	double	max_pwr;	/* max power draw in Watts */
	/* returns the battery temperature in kelvin */
	double	(*get_temp)(elec_comp_t *comp);
	void	*userinfo;
} elec_batt_info_t;

typedef struct elec_gen_info_s {
	bool_t	ac;		/* B_TRUE = AC, B_FALSE = DC */
	double	volts;		/* nominal voltage at operating rpm */
	double	stab_rate;	/* stabilization adaptation rate (FILTER_IN) */
	double	min_rpm;	/* min rpm at which volts can be achieved */
	double	max_rpm;	/* max rpm above which regulation fails */
	double	max_pwr;	/* max power draw in Watts */
	vect2_t	*eff_curve;	/* Watts -> efficiency curve for fx_lin_multi */
	double	(*get_rpm)(elec_comp_t *comp);
	void	*userinfo;
} elec_gen_info_t;

typedef struct {
	double		in_volts;	/* nominal input voltage */
	double		out_volts;	/* nominal output voltage */
	double		max_pwr;	/* max output power in Watts */
	vect2_t		*eff_curve;	/* output Watts -> efficiency curve */
	elec_comp_info_t *ac;		/* AC bus side */
	elec_comp_info_t *dc;		/* DC bus side */
} elec_tru_info_t;

typedef struct {
	bool_t		stab;		/* Stabilized PSU, or const-current? */
	double		incap_C;	/* Input capacitance in Farad */
	double		incap_R;	/* Resitance (Ohm) for incap charging */
	double		incap_leak_Qps;	/* Incap leakage rate in Coulomb/s */
	bool_t		ac;		/* Needs AC or DC input? */
	double		min_volts;	/* minimum voltage to operate */
	/* Unstabilized loads return Amps, stabilized loads return Watts. */
	double		(*get_load)(elec_comp_t *comp);
	double		std_load;	/* shorthand for fixed loads */
	void		*userinfo;
} elec_load_info_t;

typedef struct {
	bool_t			ac;
	double			impedance;	/* Ohms */
	elec_comp_info_t	**comps;	/* NULL-terminated */
} elec_bus_info_t;

typedef struct {
	double			max_amps;
	double			rate;		/* heating rate */
} elec_cb_info_t;

typedef struct {
	elec_comp_info_t	*sides[2];
} elec_diode_info_t;

struct elec_comp_info_s {
	elec_comp_type_t		type;
	char				*name;
	void				*userinfo;
	bool_t				autogen;
	union {
		elec_batt_info_t	batt;
		elec_gen_info_t		gen;
		elec_tru_info_t		tru;
		elec_load_info_t	load;
		elec_bus_info_t		bus;
		elec_cb_info_t		cb;
		elec_diode_info_t	diode;
	};
};

typedef struct {
	const char	*name;
	void		*value;
} elec_func_bind_t;

typedef void (*elec_user_cb_t)(elec_sys_t *sys, bool_t pre, void *userinfo);

elec_sys_t *libelec_new(elec_comp_info_t *comp_infos, size_t num_infos);
void libelec_destroy(elec_sys_t *sys);

void libelec_sys_start(elec_sys_t *sys);
void libelec_sys_stop(elec_sys_t *sys);

#ifndef	LIBELEC_NO_LIBSWITCH
void libelec_create_cb_switches(const elec_sys_t *sys, const char *prefix,
    float anim_rate);
#endif	/* LIBELEC_NO_LIBSWITCH */

#ifdef	LIBELEC_SLOW_DEBUG
void libelec_step(elec_sys_t *sys);
#endif

elec_comp_info_t *libelec_infos_parse(const char *filename,
    const elec_func_bind_t *binds, size_t *num_infos);
void libelec_parsed_info_free(elec_comp_info_t *infos, size_t num_infos);

void libelec_add_user_cb(elec_sys_t *sys, bool_t pre, elec_user_cb_t cb,
    void *userinfo);
void libelec_remove_user_cb(elec_sys_t *sys, bool_t pre, elec_user_cb_t cb,
    void *userinfo);

void libelec_walk_comps(elec_sys_t *sys, void (*cb)(elec_comp_t *, void *),
    void *userinfo);
elec_comp_t *libelec_info2comp(const elec_sys_t *sys,
    const elec_comp_info_t *info);
elec_comp_info_t *libelec_comp2info(const elec_comp_t *comp);

elec_comp_t *libelec_comp_find(elec_sys_t *sys, const char *name);
size_t libelec_comp_get_num_conns(const elec_comp_t *comp);
elec_comp_t *libelec_comp_get_conn(const elec_comp_t *comp, size_t i);

double libelec_comp_get_in_volts(const elec_comp_t *comp);
double libelec_comp_get_out_volts(const elec_comp_t *comp);
double libelec_comp_get_in_amps(const elec_comp_t *comp);
double libelec_comp_get_out_amps(const elec_comp_t *comp);
double libelec_comp_get_in_pwr(const elec_comp_t *comp);
double libelec_comp_get_out_pwr(const elec_comp_t *comp);
double libelec_comp_get_incap_volts(const elec_comp_t *comp);

void libelec_cb_set(elec_comp_t *comp, bool_t set);
bool_t libelec_cb_get(const elec_comp_t *comp);
double libelec_cb_get_temp(const elec_comp_t *comp);

void libelec_tie_set_info_list(elec_comp_t *comp,
    elec_comp_info_t **bus_list, size_t list_len);
void libelec_tie_set(elec_comp_t *comp, ...) SENTINEL_ATTR;
void libelec_tie_set_v(elec_comp_t *comp, va_list ap);
void libelec_tie_set_all(elec_comp_t *comp, bool_t tied);
bool_t libelec_tie_get_all(elec_comp_t *comp);
size_t libelec_tie_get(elec_comp_t *comp, elec_comp_t **bus_list);
size_t libelec_tie_get_num_buses(const elec_comp_t *comp);

#ifdef __cplusplus
}
#endif

#endif	/* _LIBELEC_H_ */
