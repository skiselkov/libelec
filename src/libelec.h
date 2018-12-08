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

#ifndef	_LIBELEC_H_
#define	_LIBELEC_H_

#include <acfutils/geom.h>

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
	double		input_cap;	/* Input capacitance in Coulomb */
	double		min_volts;	/* minimum voltage to operate */
	/* Unstabilized loads return Amps, stabilized loads return Watts. */
	double		(*get_load)(elec_comp_t *comp);
	void		*userinfo;
} elec_load_info_t;

typedef struct {
	bool_t			ac;
	double			impedance;	/* Ohms */
	elec_comp_info_t	**comps;	/* NULL-terminated */
} elec_bus_info_t;

typedef struct {
	double			max_amps;
} elec_cb_info_t;

typedef struct {
	elec_comp_info_t	*sides[2];
} elec_diode_info_t;

struct elec_comp_info_s {
	elec_comp_type_t		type;
	char				*name;
	void				*userinfo;
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

elec_sys_t *libelec_new(const elec_comp_info_t *comp_infos, size_t num_infos);
void libelec_destroy(elec_sys_t *sys);

elec_comp_info_t *libelec_infos_parse(const char *filename,
    const elec_func_bind_t *binds, size_t *num_infos);
void libelec_parsed_info_free(elec_comp_info_t *infos, size_t num_infos);

void libelec_walk_comps(elec_sys_t *sys, void (*cb)(elec_comp_t *, void*),
    void *userinfo);
elec_comp_t *libelec_info2comp(const elec_sys_t *sys,
    const elec_comp_info_t *info);
const elec_comp_info_t *libelec_comp2info(const elec_comp_t *comp);

elec_comp_t *libelec_comp_get_src(const elec_comp_t *comp);
elec_comp_t *libelec_comp_get_upstream(const elec_comp_t *comp);

double libelec_comp_get_in_volts(const elec_comp_t *comp);
double libelec_comp_get_out_volts(const elec_comp_t *comp);
double libelec_comp_get_in_amps(const elec_comp_t *comp);
double libelec_comp_get_out_amps(const elec_comp_t *comp);
double libelec_comp_get_in_pwr(const elec_comp_t *comp);
double libelec_comp_get_out_pwr(const elec_comp_t *comp);

void libelec_cb_set(elec_comp_t *comp, bool_t set);
bool_t libelec_cb_get(const elec_comp_t *comp);
double libelec_cb_get_temp(const elec_comp_t *comp);

void libelec_tie_set(elec_comp_t *comp, elec_comp_info_t *const*bus_list,
    size_t list_len);
void libelec_tie_set_all(elec_comp_t *comp, bool_t tied);
bool_t libelec_tie_get_all(const elec_comp_t *comp);

#ifdef __cplusplus
}
#endif

#endif	/* _LIBELEC_H_ */
