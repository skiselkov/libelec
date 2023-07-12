/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */
/**
 * \file
 * This is the master libelec include file. You should include this file
 * to interface with libelec.
 */

#ifndef	_LIBELEC_H_
#define	_LIBELEC_H_

#include <stdarg.h>
#include <stdbool.h>

#include <acfutils/conf.h>
#include <acfutils/geom.h>
#include <acfutils/sysmacros.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct elec_sys_s elec_sys_t;
typedef struct elec_comp_s elec_comp_t;
typedef struct elec_comp_info_s elec_comp_info_t;

/**
 * Identifies the type of electrical component. Every component in a libelec
 * network must be one of these enumerations.
 */
typedef enum {
	/**
	 * A battery is a component which can hold a certain amount of
	 * energy. This energy can then either be discharged (when a load
	 * is connected to the battery), or recharged (when a generator
	 * or energy source of a higher voltage than the battery's output
	 * voltage is attached). The rate of recharge is controlled by
	 * the battery's internal resistance, while the rate of discharge
	 * is largely driven by the demands of the downstream loads. To
	 * simulate the effects of internal battery resistance to
	 * discharge, a battery will depress its output voltage as it
	 * approaches its power flow limits.
	 */
	ELEC_BATT,
	/**
	 * A generator is a component which converts mechanical input
	 * energy into electrical output energy. You MUST supply a
	 * callback using libelec_gen_set_rpm_cb(), which informs
	 * libelec how fast the input shaft of the generator is turning.
	 * @see libelec_gen_set_rpm_cb()
	 * @see elec_get_rpm_cb_t
	 */
	ELEC_GEN,
	/**
	 * A Transformer-Rectifier Unit is a component which performs two
	 * tasks:
	 *
	 * 1. It transforms the voltage of the input to a different voltage
	 *	on the output.
	 * 2. It rectifies AC to DC
	 *
	 * libelec's TRUs are simple devices, whose output voltage always
	 * scales in proportion to their input voltage. Thus, they do not
	 * provide a stabilized output voltage, but instead depend on the
	 * ability of the AC-side generator to supply a stable voltage. A
	 * TRU doesn't permit reverse current flow from its output side to
	 * the input.
	 */
	ELEC_TRU,
	/**
	 * An inverter is conceptually the opposite of a TRU. It changes
	 * DC power into AC power, and optionally also changes the input
	 * voltage to a different output voltage.
	 * @see \ref ELEC_TRU
	 */
	ELEC_INV,
	/**
	 * A load is a consumer of electrical energy. This can be anything
	 * you need it to be. libelec tells you if the load is powered
	 * (and how much energy it is consuming), and you then implement
	 * your custom systems behaviors based on that energy usage. Loads
	 * can either have internal power supplies, or be simple
	 * unregulated energy consumers. You can customize all of this
	 * behavior to suit your needs.
	 */
	ELEC_LOAD,
	/**
	 * Buses are the interconnecting "tissue" of the libelec network.
	 * Every component on the network must connect to one or more
	 * buses, and the buses themselves serve to distribute energy
	 * flows throughout the network. libelec buses are idealized
	 * power distribution mechanisms with no internal resistance.
	 */
	ELEC_BUS,
	/**
	 * Circuit breakers are devices which allow the passage of
	 * electrical current when closed. If open, current is not
	 * allowed to flow. A breaker can become open by either by
	 * manual pilot action, or automatically if the current flowing
	 * through the breaker exceeds the breaker's maximum rating. As
	 * such, circuit breakers are safety devices which prevent
	 * excessive current draw to protect the electrical network from
	 * damage due to faults or short-circuits.
	 *
	 * libelec assumes CBs as being specifically thermal circuit
	 * breakers, which are breakers where a small amount of energy
	 * is used to heat up an internal filament. This amount of
	 * energy loss is carefully callibrated such that when current
	 * flow through the breaker exceeds its rating, the filament
	 * deforms sufficiently to trigger an internal control circuit,
	 * which then opens the breaker. As such, thermal breakers have
	 * a certain amount of "inertia" before responding. If the
	 * current exceedance is very brief and/or not very high, the
	 * filament's thermal inertia might cause the breaker not to
	 * "trip." Conversely, when the breaker trips, the filament
	 * might remain very hot and must cool off - the operator must
	 * wait a few seconds before attempting to close the breaker
	 * again, to account for this cooling period.
	 */
	ELEC_CB,
	/**
	 * Shunts are current measuring devices inserted in-line with
	 * a circuit. In reality, shunts are very small, carefully
	 * callibrated resistors, which cause a small amount of voltage
	 * drop across their leads. This amount of voltage drop is then
	 * measured using a voltmeter and converted into a current
	 * measurement mathematically.
	 *
	 * Since a properly sized shunt is meant to only cause a very
	 * small voltage drop in the circuit, libelec shunts are
	 * idealized devices which cause no voltage drop at all. You
	 * can also use these idealized shunts to represent other
	 * current measuring devices, such as AC current clamps (which
	 * cause no voltage drop in the circuit).
	 */
	ELEC_SHUNT,
	/**
	 * Ties are devices which connect two or more buses together.
	 * You can think of ties as a generalization of contactors and
	 * relays. Each of the endpoints of a tie has two possible
	 * states: tied or not tied. When tied, the endpoint allows
	 * current to pass (in any direction) to all other endpoints
	 * which are currently tied. Conceptually, the tie behaves as
	 * a single electrical node, with links to the outside world
	 * which can be dynamically connected and disconnected:
	 *```
	 *    4-pole tie fully open                B & C tied, A & D open
	 *    (no current can flow)           (current can flow between B & C)
	 *
	 *          A o   o C                           A o   o B
	 *                                                   /
	 *              +                                   +
	 *                                                 /
	 *          B o   o D                           C o   o D
	 *
	 *
	 *      A & B tied, C & D open              All endpoints tied
	 * (current can flow between A & B)   (current can flow between all)
	 *
	 *          A o   o B                           A o   o B
	 *             \ /                                 \ /
	 *              +                                   +
	 *                                                 / \
	 *          C o   o D                           C o   o D
	 *```
	 */
	ELEC_TIE,
	/**
	 * A diode is a component which prevents current flow opposite to
	 * the diode's forward sense. libelec's diode are DC-only devices
	 * and cannot be used on AC circuits. Use a component of type
	 * ELEC_TRU to implement an AC-to-DC rectifier.
	 */
	ELEC_DIODE,
	/**
	 * This is a pseudo component, only used placing text labels on
	 * the graphical network drawing. It plays no role in the actual
	 * electrical simulation.
	 */
	ELEC_LABEL_BOX
} elec_comp_type_t;

/**
 * This is the callback type used by batteries to determine the temperature
 * of the battery. Temperature has a significant effect on a battery's
 * ability to provide current, as well as its absolute energy capacity.
 * @return The temperature the battery in **Kelvin**.
 */
typedef double (*elec_get_temp_cb_t)(elec_comp_t *comp, void *userinfo);
/**
 * Info structure describing a battery.
 */
typedef struct {
	double	volts;		/**< Nominal voltage. */
	double	capacity;	/**< Capacity in Joules at 15°C. */
	double	max_pwr;	/**< Max power draw in Watts. */
	double	chg_R;		/**< Charging resistance, Ohms. */
	/**
	 * Battery temperature callback as set using libelec_batt_set_temp_cb().
	 * @see elec_get_temp_cb_t
	 */
	elec_get_temp_cb_t get_temp;
} elec_batt_info_t;

/**
 * This is the callback type used by generators to determine the speed
 * of the generator's input shaft, which is then used for determining
 * the generator's output voltage and frequency behavior. This callback
 * is installed for generators using libelec_gen_set_rpm_cb().
 * @note A generator MUST have an rpm callback configured before the
 *	network can be started using libelec_sys_start().
 * @param comp The component for which the generator rpm is being queried.
 * @param userinfo Custom userinfo pointer, which was previously set up
 *	on the component using libelec_comp_set_userinfo().
 * @return The generator rpm, using the same units as what you have used
 *	for the `EXC_RPM`, `MIN_RPM` and `MAX_RPM` stanzas when defining
 *	the generator in the config file. libelec doesn't enforce any
 *	specific type of units for generator, so you can use whatever
 *	units are most convenient.
 */
typedef double (*elec_get_rpm_cb_t)(elec_comp_t *comp, void *userinfo);
/**
 * Info structure describing a generator.
 */
typedef struct elec_gen_info_s {
	double	volts;		/**< Nominal voltage at operating rpm. */
	double	freq;		/**< Nominal frequency at operating rpm. */
	double	stab_rate_U;	/**< Voltage stabilization adaptation rate. */
	double	stab_rate_f;	/**< Frequency stabilization adaptation rate. */
	double	exc_rpm;	/**< Min rpm for generator field excitation. */
	double	min_rpm;	/**< Min rpm at which stabilization works. */
	double	max_rpm;	/**< Max rpm at which stabilization works. */
	vect2_t	*eff_curve;	/**< Watts -> efficiency curve. */
	/**
	 * Generator speed callback as set using libelec_gen_set_rpm_cb().
	 * @see elec_get_rpm_cb_t
	 */
	elec_get_rpm_cb_t get_rpm;
} elec_gen_info_t;

/**
 * Info structure describing a TRU, inverter or battery charger.
 */
typedef struct {
	double		in_volts;	/**< Nominal input voltage. */
	double		min_volts;	/**< Minimum input voltage. */
	double		out_volts;	/**< Nominal output voltage. */
	double		out_freq;	/**< Output frequency (for inverters) */
	vect2_t		*eff_curve;	/**< Output Watts -> efficiency curve */
	const elec_comp_info_t *ac;	/**< AC bus side */
	const elec_comp_info_t *dc;	/**< DC bus side */
	/**
	 * When operating in battery-charger mode, `charger' must be true
	 * and the battery link non-NULL.
	 */
	bool		charger;
	const elec_comp_info_t *batt;
	const elec_comp_info_t *batt_conn;
	double		curr_lim;	/**< Charger current limit (in Amps). */
} elec_tru_info_t;

/**
 * This is the callback type used by electrical loads for cases where
 * the caller has installed a custom load demand callback using
 * libelec_load_set_load_cb().
 * @param comp The component for which the load demand is being queried.
 * @param userinfo Custom userinfo pointer, which was previously set up
 *	on the component using libelec_comp_set_userinfo().
 * @return
 *	1. For components with stabilized power supplies (`STAB TRUE` in
 *	   the definition file), this callback must return the **power**
 *	   demand of the load in **Watts**.
 *	2. For components with unstabilized power supplies (`STAB FALSE`
 *	   in the definition file, the default), this callback must
 *	   return the **current** demand of the load in **Amps**.
 */
typedef double (*elec_get_load_cb_t)(elec_comp_t *comp, void *userinfo);
/**
 * Info structure describing a load.
 */
typedef struct {
	/** Needs AC or DC input? */
	bool		ac;
	/** Does the load have a stabilized PSU, or is constant-current? */
	bool		stab;
	/** Input capacitance in Farad. */
	double		incap_C;
	/** Resitance (in Ohms) for incap charging. */
	double		incap_R;
	/** Incap leakage rate in Coulomb/s. */
	double		incap_leak_Qps;
	/** Minimum voltage to be powered up (for libelec_comp_is_powered()) */
	double		min_volts;
	/** @see elec_get_load_cb_t */
	elec_get_load_cb_t get_load;
	/**
	 * Fixed load (in Watts for stabilized and Amps for unstabilized)
	 * for constant-demand loads (specified using the `STD_LOAD` stanza).
	 */
	double		std_load;
} elec_load_info_t;

/**
 * Info structure describing a bus.
 */
typedef struct {
	bool			ac;		/**< Is the bus AC or DC? */
	const elec_comp_info_t	**comps;	/**< Connected components */
	size_t			n_comps;	/**< Number of components */
} elec_bus_info_t;

/**
 * Info structure describing a circuit breaker.
 */
typedef struct {
	double			max_amps;	/**< Max current rating */
	double			rate;		/**< Heating rate */
	bool			fuse;		/**< Is this breaker a fuse? */
	bool			triphase;	/**< Is a 3-phase breaker? */
} elec_cb_info_t;

/**
 * Info structure describing a diode.
 */
typedef struct {
	/**
	 * The two sides of a diode. Slot [0] is the input, and slot [1]
	 * the output, so current flow can only occur [0]->[1].
	 */
	const elec_comp_info_t	*sides[2];
} elec_diode_info_t;

typedef enum {
    GUI_LOAD_GENERIC,
    GUI_LOAD_MOTOR
} gui_load_type_t;

typedef struct {
	vect2_t			pos;
	vect2_t			sz;
	double			font_scale;
} elec_label_box_info_t;

/**
 * \struct elec_comp_info_t
 * After parsing the electrical definition, each component gets an info
 * structure generated, containing the parsed information. You can
 * request this structure from the component using libelec_comp2info().
 * This is mostly useful for interrogating various information about the
 * component, such as its type, name and various configuration details
 * pertaining to its configuration.
 *
 * Each elec_comp_info_t structure contains a union of various
 * component-type-specific information. You should only access the
 * type-specific structures matched to the component type.
 */
struct elec_comp_info_s {
	elec_comp_type_t		type;	/**< Component type. */
	char				*name;	/**< Component name. */
	/** Userinfo pointer set using libelec_comp_set_userinfo(). */
	void				*userinfo;
	/**
	 * Was this component auto-generated? CBs are auto-generated
	 * when specified using the LOADCB stanza.
	 */
	bool				autogen;
	/** Descriptive location name (e.g. breaker panel row & column). */
	char				location[32];
	/** Internal resistance in Ohms. */
	double				int_R;
	/** Line number in input file on which the component was found. */
	unsigned			parse_linenum;
	union {
		elec_batt_info_t	batt;	/**< Valid for an ELEC_BATT. */
		elec_gen_info_t		gen;	/**< Valid for an ELEC_GEN. */
		/** Valid for an ELEC_TRU and ELEC_INV. */
		elec_tru_info_t		tru;
		elec_load_info_t	load;	/**< Valid for an ELEC_LOAD. */
		elec_bus_info_t		bus;	/**< Valid for an ELEC_BUS. */
		elec_cb_info_t		cb;	/**< Valid for an ELEC_CB. */
		elec_diode_info_t	diode;	/**< Valid for an ELEC_DIODE. */
		/** Valid for an ELEC_LABEL_BOX */
		elec_label_box_info_t	label_box;
	};
	/** Visual information, for drawing on the network diagram plot */
	struct {
		vect2_t			pos;
		double			sz;
		int			rot;
		gui_load_type_t		load_type;
		bool			virt;
		bool			invis;
		vect3_t			color;
	} gui;
};

/**
 * Custom physics callback, which you can install using libelec_add_user_cb(),
 * or remove using libelec_remove_user_cb(). This will be called from the
 * physics calculation thread, allowing you perform precise accounting of
 * all physics state as it's being calculated.
 * @see libelec_add_user_cb()
 * @see libelec_remove_user_cb()
 */
typedef void (*elec_user_cb_t)(elec_sys_t *sys, bool pre, void *userinfo);

elec_sys_t *libelec_new(const char *filename);
void libelec_destroy(elec_sys_t *sys);

elec_comp_info_t *libelec_get_comp_infos(const elec_sys_t *sys,
    size_t *num_infos);

bool libelec_sys_start(elec_sys_t *sys);
void libelec_sys_stop(elec_sys_t *sys);
bool libelec_sys_is_started(const elec_sys_t *sys);
bool libelec_sys_can_start(const elec_sys_t *sys);

void libelec_sys_set_time_factor(elec_sys_t *sys, double time_factor);
double libelec_sys_get_time_factor(const elec_sys_t *sys);

void libelec_serialize(elec_sys_t *sys, conf_t *ser, const char *prefix);
bool libelec_deserialize(elec_sys_t *sys, const conf_t *ser,
    const char *prefix);

#ifdef	LIBELEC_WITH_NETLINK
void libelec_enable_net_send(elec_sys_t *sys);
void libelec_disable_net_send(elec_sys_t *sys);
void libelec_enable_net_recv(elec_sys_t *sys);
void libelec_disable_net_recv(elec_sys_t *sys);
#endif	/* defined(LIBELEC_WITH_NETLINK) */

#ifdef	LIBELEC_WITH_LIBSWITCH
void libelec_create_cb_switches(const elec_sys_t *sys, const char *prefix,
    float anim_rate);
#endif	/* defined(LIBELEC_WITH_LIBSWITCH) */

#ifdef	LIBELEC_SLOW_DEBUG
void libelec_step(elec_sys_t *sys);
#endif

/* User callbacks */
void libelec_add_user_cb(elec_sys_t *sys, bool pre, elec_user_cb_t cb,
    void *userinfo);
void libelec_remove_user_cb(elec_sys_t *sys, bool pre, elec_user_cb_t cb,
    void *userinfo);

/* Finding devices and interrogating their configuration */
elec_comp_t *libelec_comp_find(elec_sys_t *sys, const char *name);
void libelec_walk_comps(elec_sys_t *sys, void (*cb)(elec_comp_t *, void *),
    void *userinfo);
const elec_comp_info_t *libelec_comp2info(const elec_comp_t *comp);
bool libelec_comp_is_AC(const elec_comp_t *comp);

size_t libelec_comp_get_num_conns(const elec_comp_t *comp);
elec_comp_t *libelec_comp_get_conn(const elec_comp_t *comp, size_t i);

/* Electrical state querying */
double libelec_comp_get_in_volts(const elec_comp_t *comp);
double libelec_comp_get_out_volts(const elec_comp_t *comp);
double libelec_comp_get_in_amps(const elec_comp_t *comp);
double libelec_comp_get_out_amps(const elec_comp_t *comp);
double libelec_comp_get_in_pwr(const elec_comp_t *comp);
double libelec_comp_get_out_pwr(const elec_comp_t *comp);
double libelec_comp_get_in_freq(const elec_comp_t *comp);
double libelec_comp_get_out_freq(const elec_comp_t *comp);
double libelec_comp_get_incap_volts(const elec_comp_t *comp);
bool libelec_comp_is_powered(const elec_comp_t *comp);

/* Failures */
void libelec_comp_set_failed(elec_comp_t *comp, bool failed);
bool libelec_comp_get_failed(const elec_comp_t *comp);
void libelec_comp_set_shorted(elec_comp_t *comp, bool shorted);
bool libelec_comp_get_shorted(const elec_comp_t *comp);
double libelec_gen_set_random_volts(elec_comp_t *comp, double stddev);
double libelec_gen_set_random_freq(elec_comp_t *comp, double stddev);

/* Callback setup */
void libelec_comp_set_userinfo(elec_comp_t *comp, void *userinfo);
void *libelec_comp_get_userinfo(const elec_comp_t *comp);

void libelec_batt_set_temp_cb(elec_comp_t *batt, elec_get_temp_cb_t cb);
elec_get_temp_cb_t libelec_batt_get_temp_cb(const elec_comp_t *batt);

void libelec_gen_set_rpm_cb(elec_comp_t *gen, elec_get_rpm_cb_t cb);
elec_get_rpm_cb_t libelec_gen_get_rpm_cb(const elec_comp_t *gen);

void libelec_load_set_load_cb(elec_comp_t *load, elec_get_load_cb_t cb);
elec_get_load_cb_t libelec_load_get_load_cb(elec_comp_t *load);

/* Circuit breakers */
void libelec_cb_set(elec_comp_t *comp, bool set);
bool libelec_cb_get(const elec_comp_t *comp);
double libelec_cb_get_temp(const elec_comp_t *comp);

/* Ties */
#ifdef	__cplusplus
void libelec_tie_set_list(elec_comp_t *comp, size_t list_len,
    elec_comp_t *const*bus_list);
#else	/* !defined(__cplusplus) */
void libelec_tie_set_list(elec_comp_t *comp, size_t list_len,
    elec_comp_t *const bus_list[list_len]);
#endif	/* !defined(__cplusplus) */
void libelec_tie_set(elec_comp_t *comp, ...) SENTINEL_ATTR;
void libelec_tie_set_v(elec_comp_t *comp, va_list ap);
void libelec_tie_set_all(elec_comp_t *comp, bool tied);
bool libelec_tie_get_all(elec_comp_t *comp);
#ifdef	__cplusplus
size_t libelec_tie_get_list(elec_comp_t *comp, size_t cap,
    elec_comp_t **bus_list);
#else	/* !defined(__cplusplus) */
size_t libelec_tie_get_list(elec_comp_t *comp, size_t cap,
    elec_comp_t *bus_list[static cap]);
#endif	/* !defined(__cplusplus) */
size_t libelec_tie_get_num_buses(const elec_comp_t *comp);
bool libelec_tie_get(elec_comp_t *tie, bool exhaustive, ...)
    SENTINEL_ATTR;
bool libelec_tie_get_v(elec_comp_t *tie, bool exhaustive, va_list ap);

/* Batteries */
double libelec_batt_get_chg_rel(const elec_comp_t *batt);
void libelec_batt_set_chg_rel(elec_comp_t *batt, double chg_rel);

/* Miscellaneous */
bool libelec_chgr_get_working(const elec_comp_t *chgr);

double libelec_phys_get_batt_voltage(double U_nominal, double chg_rel,
    double I_rel);

#ifdef __cplusplus
}
#endif

#endif	/* _LIBELEC_H_ */
