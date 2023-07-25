/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stddef.h>

#ifdef	WITH_READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif	/* defined(WITH_READLINE) */

#include <acfutils/avl.h>
#include <acfutils/assert.h>
#include <acfutils/crc64.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>
#include <acfutils/mt_cairo_render.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>

#include "libelec.h"
#include "libelec_drawing.h"

#define	WHITE_RGB	1, 1, 1
#define	BLACK_RGB	0, 0, 0
#define	LIGHT_GRAY_RGB	0.67, 0.67, 0.67
#define	DARK_GRAY_RGB	0.33, 0.33, 0.33

#define	AUTOFMT_VALUE_5(val)	(val) >= 999.95 ? 0 : 1, (val)
#define	AUTOFMT_VOLTS_5(val)	fixed_decimals(val, 4), (val)
#define	AUTOFMT_AMPS_5(val)	fixed_decimals(val, 4), (val)
#define	AUTOFMT_PWR_6(val)	\
	pwr_length(val), pwr_decimals(val), pwr_conv(val), pwr_units(val)

static char **cmd_comps = NULL;
static size_t n_cmd_comps = 0;
static size_t cmd_comp_i = 0;

typedef struct {
	const elec_comp_info_t	*info;
	float			load;
	avl_node_t		node;
} load_info_t;

static elec_sys_t *sys = NULL;
static avl_tree_t load_infos;

static double get_load(elec_comp_t *comp, void *userinfo);

static enum {
    FORMAT_HUMAN_READABLE,
    FORMAT_CSV,
    FORMAT_JSON
} output_format = FORMAT_HUMAN_READABLE;

static int
pwr_length(double val)
{
	return (val < 10000 ? 5 : 4);
}

static int
pwr_decimals(double val)
{
	if (val < 10000)
		return (0);
	if (val < 99950)
		return (1);
	return (0);
}

static double
pwr_conv(double val)
{
	return (val < 10000 ? val : val / 1000.0);
}

static const char *
pwr_units(double val)
{
	return (val < 10000 ? "W" : "kW");
}

static bool
get_next_word(char *buf, size_t cap)
{
	ASSERT(buf != NULL);
	ASSERT(cap != 0);
	buf[0] = '\0';
	if (cmd_comp_i >= n_cmd_comps)
		return (false);
	strlcpy(buf, cmd_comps[cmd_comp_i], cap);
	cmd_comp_i++;
	return (true);
}

static bool
check_comp_attachment(const elec_comp_t *comp, const char *tgt_name)
{
	ASSERT(comp != NULL);
	ASSERT(tgt_name != NULL);

	for (unsigned i = 0, n = libelec_comp_get_num_conns(comp); i < n; i++) {
		const elec_comp_t *comp2 = libelec_comp_get_conn(comp, i);
		if (strcmp(libelec_comp2info(comp2)->name, tgt_name) == 0)
			return (true);
	}
	return (false);
}

static int
load_info_compar(const void *a, const void *b)
{
	const load_info_t *la = a, *lb = b;

	if (la->info < lb->info)
		return (-1);
	if (la->info > lb->info)
		return (1);
	return (0);
}

static void
print_usage(FILE *fp, const char *progname)
{
	fprintf(fp, "Usage: %s [-hvJC] [-i <init_cmds_file>] <elec_file>"
#ifdef	LIBELEC_WITH_NETLINK
	    " [-s <url>|-r <url>]\n"
#else	/* !defined(LIBELEC_WITH_NETLINK) */
	    "\n"
#endif	/* !defined(LIBELEC_WITH_NETLINK) */
	    "  -h : Show this help screen.\n"
	    "  -v : Show version number and copyright screen, then exit.\n"
	    "  -J : Enable JSON output formatting instead of human-readable "
	    "formatting.\n"
	    "       This also disables interactive editing features, prompt "
	    "generation\n"
	    "       and configures nettest to operate as a scriptable "
	    "backend.\n"
	    "  -C : Enable CSV output formatting instead of human-readable "
	    "formatting.\n"
	    "       nettest won't emit table headers and each field is "
	    "separated by a\n"
	    "       single comma with no whitespace. Strings are quoted "
	    "using \" characters.\n"
	    "       This also disables interactive editing features, prompt "
	    "generation\n"
	    "       and configures nettest to operate as a scriptable "
	    "backend.\n"
	    "  -i <init_cmds_file> : File containing list of commands to "
	    "run at startup.\n"
	    "       Use this to configure the network to an initial state. "
	    "After running\n"
	    "       these commands, nettest will switch to interactive mode.\n",
	    progname);
}

static void
print_version(void)
{
	printf("nettest utility libelec commit %s (built %s)\n"
	    "\n"
	    "Copyright 2023 Saso Kiselkov. All rights reserved.\n"
	    "\n"
	    "Use of this program is subject to the terms of the Mozilla "
	    "Public License v2.0.\n"
	    "You can obtain a copy of the license at "
	    "https://mozilla.org/MPL/2.0/.\n",
	    LIBELEC_VERSION, BUILD_TIMESTAMP);
}

static void
debug_print(const char *str)
{
	fprintf(stderr, "%s",str);
}

static bool print_first_data_row = true;

SENTINEL_ATTR static void
print_table_header(const char *first, ...)
{
	va_list ap;
	bool is_first = true;

	switch (output_format) {
	case FORMAT_HUMAN_READABLE:
		va_start(ap, first);
		for (const char *name = first; name != NULL;
		    name = va_arg(ap, const char *)) {
			int width = va_arg(ap, int);
			printf("%s%*s", !is_first ? "  " : "", width, name);
			is_first = false;
		}
		va_end(ap);
		printf("\n");

		is_first = true;
		va_start(ap, first);
		for (const char *name = first; name != NULL;
		    name = va_arg(ap, const char *)) {
			int width = va_arg(ap, int);
			if (!is_first)
				printf("  ");
			for (int i = 0; i < ABS(width); i++)
				printf("-");
			is_first = false;
		}
		va_end(ap);
		printf("\n");
		break;
	case FORMAT_CSV:
		break;
	case FORMAT_JSON:
		printf("[");
		break;
	}
	print_first_data_row = true;
}

static void
print_table_footer(void)
{
	switch (output_format) {
	case FORMAT_HUMAN_READABLE:
	case FORMAT_CSV:
		break;
	case FORMAT_JSON:
		printf("\n]\n");
		break;
	}
}

typedef enum {
    PRINT_DATA_STR,
    PRINT_DATA_I32,
    PRINT_DATA_BOOL,
    PRINT_DATA_F64,
    PRINT_DATA_STR_ARRAY
} print_data_type_t;

typedef struct {
	const char		*name;
	print_data_type_t	type;
	int			width;
	int			decimals;
	const char		*units;
	union {
		const char	*str;
		int		i32;
		bool		b;
		double		f64;
		struct {
			const char	**str_array;
			unsigned	str_array_len;
		};
	};
} print_data_t;

#define	PRINT_STR(data_name, data_width, data_str) \
	(&(print_data_t){ \
	    .type = PRINT_DATA_STR, \
	    .name = (data_name), \
	    .str = (data_str), \
	    .width = (data_width) \
	})

#define	PRINT_I32(data_name, data_width, data_i32, data_units) \
	(&(print_data_t){ \
	    .type = PRINT_DATA_I32, \
	    .name = (data_name), \
	    .i32 = (data_i32), \
	    .width = (data_width), \
	    .units = (data_units) \
	})

#define	PRINT_F64(data_name, data_width, data_decimals, data_f64, data_units) \
	(&(print_data_t){ \
	    .type = PRINT_DATA_F64, \
	    .name = (data_name), \
	    .f64 = (data_f64), \
	    .width = (data_width), \
	    .decimals = (data_decimals), \
	    .units = (data_units) \
	})

#define	PRINT_BOOL(data_name, data_width, data_bool) \
	(&(print_data_t){ \
	    .type = PRINT_DATA_BOOL, \
	    .name = (data_name), \
	    .b = (data_bool), \
	    .width = (data_width) \
	})

#define	PRINT_STR_ARRAY(data_name, str_array_data, str_array_data_len) \
	(&(print_data_t){ \
	    .type = PRINT_DATA_STR_ARRAY, \
	    .name = (data_name), \
	    .str_array = str_array_data, \
	    .str_array_len = str_array_data_len \
	})

#define	PRINT_VOLTS(data_name, volts) \
	PRINT_F64((data_name), 5, fixed_decimals((volts), 4), (volts), "V")
#define	PRINT_AMPS(data_name, amps) \
	PRINT_F64((data_name), 5, fixed_decimals((amps), 4), (amps), "A")
#define	PRINT_PWR(data_name, pwr) \
	PRINT_F64((data_name), pwr_length(pwr), pwr_decimals(pwr), \
	    pwr_conv(pwr), pwr_units(pwr))

static void
print_table_data_str(FILE *fp, const print_data_t *data)
{
	char format[32];

	ASSERT(fp != NULL);
	ASSERT(data != NULL);
	ASSERT3U(data->type, ==, PRINT_DATA_STR);
	ASSERT(data->name != NULL);

	switch (output_format) {
	case FORMAT_HUMAN_READABLE:
		if (data->width != 0)
			snprintf(format, sizeof (format), "%%%ds", data->width);
		else
			lacf_strlcpy(format, "%s", sizeof (format));
		fprintf(fp, format, data->str);
		if (data->units != NULL)
			fprintf(fp, "%s", data->units);
		break;
	case FORMAT_CSV:
		fprintf(fp, "\"%s\"", data->str);
		break;
	case FORMAT_JSON:
		fprintf(fp, "    \"%s\": \"%s\"", data->name, data->str);
		break;
	}
}

static void
print_table_data_i32(FILE *fp, const print_data_t *data)
{
	char format[32];

	ASSERT(fp != NULL);
	ASSERT(data != NULL);
	ASSERT3U(data->type, ==, PRINT_DATA_I32);
	ASSERT(data->name != NULL);

	switch (output_format) {
	case FORMAT_HUMAN_READABLE:
		if (data->width != 0)
			snprintf(format, sizeof (format), "%%%dd", data->width);
		else
			lacf_strlcpy(format, "%d", sizeof (format));
		fprintf(fp, format, data->i32);
		if (data->units != NULL)
			fprintf(fp, "%s", data->units);
		break;
	case FORMAT_CSV:
		fprintf(fp, "%d", data->i32);
		break;
	case FORMAT_JSON:
		fprintf(fp, "    \"%s\": %d", data->name, data->i32);
		break;
	}
}

static void
print_table_data_bool(FILE *fp, const print_data_t *data)
{
	char format[32];

	ASSERT(fp != NULL);
	ASSERT(data != NULL);
	ASSERT3U(data->type, ==, PRINT_DATA_BOOL);
	ASSERT(data->name != NULL);

	switch (output_format) {
	case FORMAT_HUMAN_READABLE:
		if (data->width != 0)
			snprintf(format, sizeof (format), "%%%ds", data->width);
		else
			lacf_strlcpy(format, "%s", sizeof (format));
		fprintf(fp, format, data->b != false ? "YES" : "NO");
		break;
	case FORMAT_CSV:
		fprintf(fp, "%d", data->b != false);
		break;
	case FORMAT_JSON:
		fprintf(fp, "    \"%s\": %s", data->name, data->b != false ?
		    "true" : "false");
		break;
	}
}

static void
print_table_data_f64(FILE *fp, const print_data_t *data)
{
	char format[32];

	ASSERT(fp != NULL);
	ASSERT(data != NULL);
	ASSERT3U(data->type, ==, PRINT_DATA_F64);
	ASSERT(data->name != NULL);

	switch (output_format) {
	case FORMAT_HUMAN_READABLE:
		if (data->width != 0) {
			snprintf(format, sizeof (format), "%%%d.%df",
			    data->width, data->decimals);
		} else {
			lacf_strlcpy(format, "%f", sizeof (format));
		}
		fprintf(fp, format, data->f64);
		if (data->units != NULL)
			fprintf(fp, "%s", data->units);
		break;
	case FORMAT_CSV:
		fprintf(fp, "%f", data->f64);
		break;
	case FORMAT_JSON:
		fprintf(fp, "    \"%s\": %f", data->name, data->f64);
		break;
	}
}

static void
print_table_data_str_array(FILE *fp, const print_data_t *data)
{
	ASSERT(fp != NULL);
	ASSERT(data != NULL);
	ASSERT3U(data->type, ==, PRINT_DATA_STR_ARRAY);
	ASSERT(data->name != NULL);

	switch (output_format) {
	case FORMAT_HUMAN_READABLE:
		for (unsigned i = 0; i < data->str_array_len; i++) {
			fprintf(fp, "%s%s", data->str_array[i],
			    i + 1 < data->str_array_len ? ", " : "");
		}
		break;
	case FORMAT_CSV:
		fprintf(fp, "\"");
		for (unsigned i = 0; i < data->str_array_len; i++) {
			fprintf(fp, "%s%s", data->str_array[i],
			    i + 1 < data->str_array_len ? "," : "");
		}
		fprintf(fp, "\"");
		break;
	case FORMAT_JSON:
		fprintf(fp, "    \"%s\": [", data->name);
		for (unsigned i = 0; i < data->str_array_len; i++) {
			fprintf(fp, "\"%s\"%s", data->str_array[i],
			    i + 1 < data->str_array_len ? ", " : "");
		}
		fprintf(fp, "]");
		break;
	}
}

SENTINEL_ATTR static void
print_table_row(FILE *fp, ...)
{
	va_list ap;
	bool first = true;

	ASSERT(fp != NULL);

	switch (output_format) {
	case FORMAT_HUMAN_READABLE:
	case FORMAT_CSV:
		break;
	case FORMAT_JSON:
		fprintf(fp, "%s\n  {", print_first_data_row ? "" : ",");
		break;
	}
	print_first_data_row = false;

	va_start(ap, fp);
	for (const print_data_t *data = va_arg(ap, const print_data_t *);
	    data != NULL; data = va_arg(ap, const print_data_t *)) {
		if (!first) {
			switch (output_format) {
			case FORMAT_HUMAN_READABLE:
				fprintf(fp, "  ");
				break;
			case FORMAT_CSV:
				fprintf(fp, ",");
				break;
			case FORMAT_JSON:
				fprintf(fp, ",\n");
				break;
			}
		} else {
			switch (output_format) {
			case FORMAT_HUMAN_READABLE:
			case FORMAT_CSV:
				break;
			case FORMAT_JSON:
				fprintf(fp, "\n");
				break;
			}
		}
		switch (data->type) {
		case PRINT_DATA_STR:
			print_table_data_str(fp, data);
			break;
		case PRINT_DATA_I32:
			print_table_data_i32(fp, data);
			break;
		case PRINT_DATA_BOOL:
			print_table_data_bool(fp, data);
			break;
		case PRINT_DATA_F64:
			print_table_data_f64(fp, data);
			break;
		case PRINT_DATA_STR_ARRAY:
			print_table_data_str_array(fp, data);
			break;
		}
		first = false;
	}
	va_end(ap);

	switch (output_format) {
	case FORMAT_HUMAN_READABLE:
	case FORMAT_CSV:
		fprintf(fp, "\n");
		break;
	case FORMAT_JSON:
		fprintf(fp, "\n  }");
		break;
	}
}

static void
print_buses_i(elec_comp_t *comp, void *userinfo)
{
	ASSERT(comp != NULL);
	const elec_comp_info_t *info = libelec_comp2info(comp);
	const char *bus_name = (userinfo != NULL ? userinfo : NULL);

	if (info->type != ELEC_BUS || info->autogen || (bus_name != NULL &&
	    lacf_strcasecmp(info->name, bus_name) != 0)) {
		return;
	}
	print_table_row(stdout,
	    PRINT_STR("NAME", -30, info->name),
	    PRINT_VOLTS("U", libelec_comp_get_in_volts(comp)),
	    NULL);
}

static const char *
elec_comp_type2str(elec_comp_type_t type)
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
	case ELEC_LABEL_BOX:
		return ("LABEL");
	}
	VERIFY_FAIL();
}

static void
print_bus_dev(const elec_comp_t *comp)
{
	ASSERT(comp != NULL);
	const elec_comp_info_t *info = libelec_comp2info(comp);

	print_table_row(stdout,
	    PRINT_STR("TYPE", -5, elec_comp_type2str(info->type)),
	    PRINT_STR("NAME", -30, info->name),
	    PRINT_VOLTS("U_in", libelec_comp_get_in_volts(comp)),
	    PRINT_AMPS("I_in", libelec_comp_get_in_amps(comp)),
	    PRINT_PWR("W_in", libelec_comp_get_in_pwr(comp)),
	    info->type == ELEC_CB ?
	    PRINT_BOOL("SET", 3, libelec_cb_get(comp)) : NULL,
	    NULL);
}

static void
bus_cmd(void)
{
	char bus_name[128], subcmd[32];
	elec_comp_t *comp;

	if (!get_next_word(bus_name, sizeof (bus_name))) {
		print_table_header("NAME", -30, "U", 6, NULL);
		libelec_walk_comps(sys, print_buses_i, NULL);
		print_table_footer();
		return;
	}
	comp = libelec_comp_find(sys, bus_name);
	if (comp == NULL || libelec_comp2info(comp)->type != ELEC_BUS) {
		fprintf(stderr, "Error: unknown component %s, or %s is not "
		    "a bus\n", bus_name, bus_name);
		return;
	}
	if (!get_next_word(subcmd, sizeof (subcmd))) {
		print_table_header("NAME", -30, "U", 6, NULL);
		libelec_walk_comps(sys, print_buses_i, bus_name);
		print_table_footer();
		return;
	}
	if (lacf_strcasecmp(subcmd, "list") == 0) {
		char comp_name[128];
		size_t n = libelec_comp_get_num_conns(comp);

		print_table_header("TYPE", -5, "NAME", -30,
		    "U_in", 6, "I_in", 6, "W_in", 6, "SET", 3, NULL);
		if (!get_next_word(comp_name, sizeof (comp_name))) {
			for (size_t i = 0; i < n; i++)
				print_bus_dev(libelec_comp_get_conn(comp, i));
		} else {
			do {
				for (size_t i = 0; i < n; i++) {
					elec_comp_t *conn_comp =
					    libelec_comp_get_conn(comp, i);
					if (lacf_strcasecmp(libelec_comp2info(
					    conn_comp)->name, comp_name) == 0) {
						print_bus_dev(conn_comp);
					}
				}
			} while (get_next_word(comp_name, sizeof (comp_name)));
		}
		print_table_footer();
	} else {
		fprintf(stderr, "Error: unknown bus subcommand \"%s\"\n",
		    subcmd);
	}
}

static void
print_trus_i(elec_comp_t *comp, void *userinfo)
{
	const char *name = (userinfo != NULL ? userinfo : NULL);
	const elec_comp_info_t *info;

	ASSERT(comp != NULL);
	info = libelec_comp2info(comp);

	if ((info->type != ELEC_TRU && info->type != ELEC_INV) ||
	    (name != NULL && strcmp(info->name, name) != 0)) {
		return;
	}
	print_table_row(stdout,
	    PRINT_STR("NAME", -30, info->name),
	    PRINT_VOLTS("U_in", libelec_comp_get_in_volts(comp)),
	    PRINT_PWR("W_in", libelec_comp_get_in_pwr(comp)),
	    PRINT_F64("Eff", 5, 1, libelec_comp_get_eff(comp) * 100, "%"),
	    PRINT_VOLTS("U_out", libelec_comp_get_out_volts(comp)),
	    PRINT_AMPS("I_out", libelec_comp_get_out_amps(comp)),
	    PRINT_PWR("W_out", libelec_comp_get_out_pwr(comp)),
	    NULL);
}

static void
tru_cmd(void)
{
	char tru_name[128];
	elec_comp_t *comp;

	if (!get_next_word(tru_name, sizeof (tru_name))) {
		print_table_header("NAME", -30, "U_in", 6, "W_in", 6,
		    "Eff", 6, "U_out", 6, "I_out", 6, "W_out", 6, NULL);
		libelec_walk_comps(sys, print_trus_i, NULL);
		print_table_footer();
		return;
	}
	comp = libelec_comp_find(sys, tru_name);
	if (comp == NULL ||
	    (libelec_comp2info(comp)->type != ELEC_TRU &&
	    libelec_comp2info(comp)->type != ELEC_INV)) {
		fprintf(stderr, "Error: unknown component %s, or %s is not a "
		    "TRU/inverter\n", tru_name, tru_name);
		return;
	}
	print_table_header("NAME", -30, "U_in", 6, "W_in", 6,
	    "Eff", 6, "U_out", 6, "I_out", 6, "W_out", 6, NULL);
	libelec_walk_comps(sys, print_trus_i, tru_name);
	print_table_footer();
}

static void
print_gens_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);
	const char *gen_name = (userinfo != NULL ? userinfo : NULL);
	double rpm;

	UNUSED(userinfo);

	if (info->type != ELEC_GEN || (gen_name != NULL &&
	    lacf_strcasecmp(gen_name, info->name) != 0)) {
		return;
	}
	rpm = libelec_gen_get_rpm(comp);
	print_table_row(stdout,
	    PRINT_STR("NAME", -30, info->name),
	    PRINT_F64("RPM", 6, fixed_decimals(rpm, 4), rpm, NULL),
	    PRINT_PWR("W_in", libelec_comp_get_in_pwr(comp)),
	    PRINT_F64("Eff", 5, 1, libelec_comp_get_eff(comp) * 100, "%"),
	    PRINT_VOLTS("U_out", libelec_comp_get_out_volts(comp)),
	    PRINT_AMPS("I_out", libelec_comp_get_out_amps(comp)),
	    PRINT_PWR("W_out", libelec_comp_get_out_pwr(comp)),
	    NULL);
}

static void
gen_cmd(void)
{
	char gen_name[128], subcmd[128];
	elec_comp_t *comp;

	if (!get_next_word(gen_name, sizeof (gen_name))) {
		print_table_header("NAME", -30, "RPM", 6, "W_in", 6,
		    "Eff", 6, "U_out", 6, "I_out", 6, "W_out", 6, NULL);
		libelec_walk_comps(sys, print_gens_i, NULL);
		print_table_footer();
		return;
	}
	comp = libelec_comp_find(sys, gen_name);
	if (comp == NULL || libelec_comp2info(comp)->type != ELEC_GEN) {
		fprintf(stderr, "Error: unknown component %s, or %s is not a "
		    "generator\n", gen_name, gen_name);
		return;
	}
	if (!get_next_word(subcmd, sizeof (subcmd))) {
		print_table_header("NAME", -30, "RPM", 6, "W_in", 6,
		    "Eff", 6, "U_out", 6, "I_out", 6, "W_out", 6, NULL);
		libelec_walk_comps(sys, print_gens_i, gen_name);
		print_table_footer();
		return;
	}
	if (lacf_strcasecmp(subcmd, "rpm") == 0) {
		char rpm_str[32];
		double rpm;

		if (!get_next_word(rpm_str, sizeof (rpm_str)) ||
		    sscanf(rpm_str, "%lf", &rpm) != 1) {
			fprintf(stderr, "Error: missing or malformed RPM "
			    "argument to \"rpm\" subcommand\n");
			return;
		}
		libelec_gen_set_rpm(comp, rpm);
	} else {
		fprintf(stderr, "Error: unknown gen subcommand \"%s\"\n",
		    subcmd);
	}
}

static void
print_ties_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);
	size_t n_conns = libelec_comp_get_num_conns(comp);
	elec_comp_t *bus_list[n_conns];
	size_t n_tied;
	const char *tie_name = (userinfo != NULL ? userinfo : NULL);
	const char **bus_names = NULL;

	if (info->type != ELEC_TIE || (tie_name != NULL &&
	    lacf_strcasecmp(tie_name, info->name) != 0)) {
		return;
	}
	n_tied = libelec_tie_get_list(comp, n_conns, bus_list);
	bus_names = safe_calloc(n_tied, sizeof (*bus_names));
	for (size_t i = 0; i < n_tied; i++) {
		const elec_comp_info_t *bus_info =
		    libelec_comp2info(bus_list[i]);
		bus_names[i] = bus_info->name;
	}
	print_table_row(stdout,
	    PRINT_STR("NAME", -30, info->name),
	    PRINT_STR_ARRAY("BUSES", bus_names, n_tied),
	    NULL);
	free(bus_names);
}

static void
tie_cmd(void)
{
	char tie_name[128], bus[128];
	elec_comp_t *comp;
	elec_comp_t **bus_list = NULL;
	size_t list_len = 0;
	/*
	 * If no tie name was provided on the command line, print the state
	 * of all ties in the network.
	 */
	if (!get_next_word(tie_name, sizeof (tie_name))) {
		print_table_header("NAME", -30, "BUSES", -30, NULL);
		libelec_walk_comps(sys, print_ties_i, NULL);
		print_table_footer();
		return;
	}
	comp = libelec_comp_find(sys, tie_name);
	if (comp == NULL) {
		fprintf(stderr, "Error: unknown tie name %s\n", tie_name);
		return;
	}
	/*
	 * Consume bus names on the command line to build a list of
	 * buses to be tied.
	 */
	while (get_next_word(bus, sizeof (bus))) {
		if (lacf_strcasecmp(bus, "all") == 0) {
			libelec_tie_set_all(comp, true);
			goto out;
		}
		if (lacf_strcasecmp(bus, "none") == 0) {
			libelec_tie_set_all(comp, false);
			goto out;
		}
		if (!check_comp_attachment(comp, bus)) {
			fprintf(stderr, "Error: %s is not connected to %s\n",
			    tie_name, bus);
			goto out;
		}
		bus_list = safe_realloc(bus_list,
		    (list_len + 1) * sizeof (*bus_list));
		bus_list[list_len] = libelec_comp_find(sys, bus);
		ASSERT(bus_list[list_len] != NULL);
		list_len++;
	}
	if (list_len != 0) {
		libelec_tie_set_list(comp, list_len, bus_list);
	} else {
		/*
		 * No buses provided, so just print the state of the
		 * one tie specified by the user.
		 */
		print_table_header("NAME", -30, "BUSES", -30, NULL);
		libelec_walk_comps(sys, print_ties_i, tie_name);
		print_table_footer();
	}
out:
	free(bus_list);
}

static void
print_cbs_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);
	const char *cb_name = (userinfo != NULL ? userinfo : NULL);

	UNUSED(userinfo);

	if (info->type != ELEC_CB || (cb_name != NULL &&
	    lacf_strcasecmp(cb_name, info->name) != 0)) {
		return;
	}
	print_table_row(stdout,
	    PRINT_STR("NAME", -30, info->name),
	    PRINT_VOLTS("U", libelec_comp_get_in_volts(comp)),
	    PRINT_AMPS("I", libelec_comp_get_in_amps(comp)),
	    PRINT_F64("TEMP", 4, 2, libelec_cb_get_temp(comp), NULL),
	    PRINT_BOOL("SET", 3, libelec_cb_get(comp)),
	    NULL);
}

static void
cb_cmd(void)
{
	char cb_name[64], subcmd[32];
	elec_comp_t *comp;

	if (!get_next_word(cb_name, sizeof (cb_name))) {
		print_table_header("NAME", -30, "U", 6, "I", 6, "TEMP", 4,
		    "SET", 3, NULL);
		libelec_walk_comps(sys, print_cbs_i, NULL);
		print_table_footer();
		return;
	}
	comp = libelec_comp_find(sys, cb_name);
	if (comp == NULL) {
		fprintf(stderr, "Error: unknown CB %s\n", cb_name);
		return;
	}
	if (!get_next_word(subcmd, sizeof (subcmd))) {
		print_table_header("NAME", -30, "U", 6, "I", 6, "TEMP", 4,
		    "SET", 3, NULL);
		libelec_walk_comps(sys, print_cbs_i, cb_name);
		print_table_footer();
		return;
	}
	if (lacf_strcasecmp(subcmd, "set") == 0) {
		char set_str[8];
		bool set;

		if (!get_next_word(set_str, sizeof (set_str))) {
			fprintf(stderr, "Error: missing argument to \"set\" "
			    "subcommand. Try typing \"help\".\n");
			return;
		}
		if (tolower(set_str[0]) == 'y' || set_str[0] == '1') {
			set = true;
		} else if (tolower(set_str[0]) == 'n' ||
		    set_str[0] == '0') {
			set = false;
		} else {
			fprintf(stderr, "Error: \"set\" subcommand argument "
			    "must be one of '0', 'N', '1' or 'Y'. "
			    "Try typing \"help\".\n");
			return;
		}
		libelec_cb_set(comp, set);
	} else {
		fprintf(stderr, "Error: unknown cb subcommand \"%s\". "
		    "Try typing \"help\".\n", subcmd);
	}
}

static void
print_batts_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);
	const char *batt_name = (userinfo != NULL ? userinfo : NULL);

	if (info->type != ELEC_BATT || (batt_name != NULL &&
	    lacf_strcasecmp(batt_name, info->name)) != 0) {
		return;
	}
	print_table_row(stdout,
	    PRINT_STR("NAME", -30, info->name),
	    PRINT_VOLTS("U_out", libelec_comp_get_out_volts(comp)),
	    PRINT_AMPS("I_out", libelec_comp_get_out_amps(comp)),
	    PRINT_AMPS("I_in", libelec_comp_get_in_amps(comp)),
	    PRINT_F64("CHG", 5,
	    libelec_batt_get_chg_rel(comp) * 100 > 99.95 ? 1 : 2,
	    libelec_batt_get_chg_rel(comp) * 100, "%"),
	    PRINT_F64("TEMP", 3, 0, KELVIN2C(libelec_batt_get_temp(comp)),
	    "\u00B0C"),
	    NULL);
}

static void
batt_cmd(void)
{
	char batt_name[64], subcmd[32];
	elec_comp_t *comp;

	if (!get_next_word(batt_name, sizeof (batt_name))) {
		print_table_header("NAME", -30, "U_out", 6, "I_out", 6,
		    "I_in", 6, "CHG", 6, "TEMP", 5, NULL);
		libelec_walk_comps(sys, print_batts_i, NULL);
		print_table_footer();
		return;
	}
	comp = libelec_comp_find(sys, batt_name);
	if (comp == NULL || libelec_comp2info(comp)->type != ELEC_BATT) {
		fprintf(stderr, "Error: unknown component %s, or %s is not a "
		    "battery\n", batt_name, batt_name);
		return;
	}
	if (!get_next_word(subcmd, sizeof (subcmd))) {
		print_table_header("NAME", -30, "U_out", 6, "I_out", 6,
		    "I_in", 6, "CHG%", 6, "TEMP", 5, NULL);
		libelec_walk_comps(sys, print_batts_i, batt_name);
		print_table_footer();
		return;
	}
	if (lacf_strcasecmp(subcmd, "chg") == 0) {
		char chg_str[32];
		float chg;
		if (!get_next_word(chg_str, sizeof (chg_str))) {
			fprintf(stderr, "Error: missing argument to \"chg\" "
			    "subcommand. Try typing \"help\".\n");
			return;
		}
		if (sscanf(chg_str, "%f", &chg) != 1 || chg < 0 || chg > 100) {
			fprintf(stderr, "Error: state of charge argument to "
			    "\"chg\" subcommand must be a number 0-100, "
			    "inclusive. Try typing \"help\".\n");
			return;
		}
		libelec_batt_set_chg_rel(comp, clamp(chg / 100, 0, 1));
	} else if (lacf_strcasecmp(subcmd, "temp") == 0) {
		char temp_str[32];
		float temp;
		if (!get_next_word(temp_str, sizeof (temp_str))) {
			fprintf(stderr, "Error: missing argument to \"temp\" "
			    "subcommand. Try typing \"help\".\n");
			return;
		}
		if (sscanf(temp_str, "%f", &temp) != 1 ||
		    temp < -90 || temp > 90) {
			fprintf(stderr, "Error: temperature argument to "
			    "\"temp\" subcommand must be a number "
			    "-90..+90, inclusive. Try typing \"help\".\n");
			return;
		}
		libelec_batt_set_temp(comp, C2KELVIN(temp));
	} else {
		fprintf(stderr, "Error: unknown batt subcommand \"%s\". "
		    "Try typing \"help\".\n", subcmd);
	}
}

static void
print_loads_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);
	const char *name = (userinfo != NULL ? userinfo : NULL);

	UNUSED(userinfo);

	if (info->type != ELEC_LOAD || (name != NULL &&
	    lacf_strcasecmp(name, info->name) != 0)) {
		return;
	}
	print_table_row(stdout,
	    PRINT_STR("NAME", -30, info->name),
	    PRINT_VOLTS("U_out", libelec_comp_get_out_volts(comp)),
	    PRINT_AMPS("I_out", libelec_comp_get_out_amps(comp)),
	    PRINT_PWR("W_out", libelec_comp_get_out_pwr(comp)),
	    PRINT_VOLTS("U_c_in", libelec_comp_get_incap_volts(comp)),
	    PRINT_AMPS("I_in", libelec_comp_get_in_amps(comp)),
	    NULL);
}

static void
load_cmd(void)
{
	char name[128], subcmd[32];
	elec_comp_t *comp;

	if (!get_next_word(name, sizeof (name))) {
		print_table_header("NAME", -30, "U_out", 6, "I_out", 6,
		    "W_out", 6, "U_c_in", 6, "I_in", 6, NULL);
		libelec_walk_comps(sys, print_loads_i, NULL);
		print_table_footer();
		return;
	}
	comp = libelec_comp_find(sys, name);
	if (comp == NULL || libelec_comp2info(comp)->type != ELEC_LOAD) {
		fprintf(stderr, "Error: unknown component %s, or %s is not a "
		    "load\n", name, name);
		return;
	}
	if (!get_next_word(subcmd, sizeof (subcmd))) {
		print_table_header("NAME", -30, "U_out", 6, "I_out", 6,
		    "W_out", 6, "U_c_in", 6, "I_in", 6, NULL);
		libelec_walk_comps(sys, print_loads_i, name);
		print_table_footer();
		return;
	}
	if (lacf_strcasecmp(subcmd, "set") == 0) {
		char load_str[32];
		load_info_t srch, *load_info;
		float load;
		avl_index_t where;

		if (!get_next_word(load_str, sizeof (load_str)) ||
		    sscanf(load_str, "%f", &load) != 1 || load < 0) {
			fprintf(stderr, "Error: missing or malformed "
			    "argument. Try typing \"help\".\n");
			return;
		}
		srch.info = libelec_comp2info(comp);
		load_info = avl_find(&load_infos, &srch, &where);
		if (load_info == NULL) {
			load_info = safe_calloc(1, sizeof (*load_info));
			load_info->info = libelec_comp2info(comp);
			avl_insert(&load_infos, load_info, where);
		}
		load_info->load = load;
	} else {
		fprintf(stderr, "Error: unknown load subcommand \"%s\". "
		    "Try typing \"help\".\n", subcmd);
	}
}

static void
draw_cmd(void)
{
	static char filename[256] = {0};
	static double offset[2] = {0};
	static double pos_scale = 16;
	static double fontsz = 14;
	static unsigned imgsz[2] = { 2048, 2048 };

	char subcmd[256], comp_name[128];
	cairo_surface_t *surf;
	cairo_t *cr;

	if (!get_next_word(subcmd, sizeof (subcmd))) {
		if (filename[0] == '\0') {
			fprintf(stderr, "Error: missing filename argument. "
			    "You must pass a filename at least once, before\n"
			    "invoking the \"draw\" command without arguments "
			    "to redraw the same image.\n"
			    "Try typing \"help\".\n");
			return;
		}
		lacf_strlcpy(subcmd, filename, sizeof (subcmd));
	}
	if (lacf_strcasecmp(subcmd, "offset") == 0) {
		char offset_str[2][16];
		double new_offset[2];
		if (!get_next_word(offset_str[0], sizeof (offset_str[0])) ||
		    !get_next_word(offset_str[1], sizeof (offset_str[1])) ||
		    sscanf(offset_str[0], "%lf", &new_offset[0]) != 1 ||
		    sscanf(offset_str[1], "%lf", &new_offset[1]) != 1) {
			fprintf(stderr, "Error: missing offset arguments, "
			    "or one of the offsets is invalid. "
			    "Try typing \"help\".\n");
			return;
		}
		offset[0] = new_offset[0];
		offset[1] = new_offset[1];
		return;
	}
	if (lacf_strcasecmp(subcmd, "scale") == 0) {
		char scale_str[16];
		double new_scale;
		if (!get_next_word(scale_str, sizeof (scale_str)) ||
		    sscanf(scale_str, "%lf", &new_scale) != 1 ||
		    new_scale <= 0) {
			fprintf(stderr, "Error: missing scale argument, or "
			    "scale is invalid. Try typing \"help\".\n");
			return;
		}
		pos_scale = new_scale;
		return;
	}
	if (lacf_strcasecmp(subcmd, "fontsz") == 0) {
		char fontsz_str[16];
		double new_fontsz;
		if (!get_next_word(fontsz_str, sizeof (fontsz_str)) ||
		    sscanf(fontsz_str, "%lf", &new_fontsz) != 1 ||
		    new_fontsz <= 0) {
			fprintf(stderr, "Error: missing fontsz argument, or "
			    "fontsz is invalid. Try typing \"help\".\n");
			return;
		}
		fontsz = new_fontsz;
		return;
	}
	if (lacf_strcasecmp(subcmd, "imgsz") == 0) {
		char imgsz_str[2][16];
		int new_imgsz[2];
		if (!get_next_word(imgsz_str[0], sizeof (imgsz_str[0])) ||
		    !get_next_word(imgsz_str[1], sizeof (imgsz_str[1])) ||
		    sscanf(imgsz_str[0], "%d", &new_imgsz[0]) != 1 ||
		    sscanf(imgsz_str[1], "%d", &new_imgsz[1]) != 1 ||
		    new_imgsz[0] <= 256 || new_imgsz[1] <= 256) {
			fprintf(stderr, "Error: missing imgsz argument, or "
			    "imgsz is invalid. Try typing \"help\".\n");
			return;
		}
		imgsz[0] = new_imgsz[0];
		imgsz[1] = new_imgsz[1];
		return;
	}

	surf = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
	    imgsz[0], imgsz[1]);
	cr = cairo_create(surf);

	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_paint(cr);
	cairo_translate(cr, offset[0], offset[1]);

	cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL,
	    CAIRO_FONT_WEIGHT_NORMAL);

	libelec_draw_layout(sys, cr, pos_scale, fontsz);

	if (get_next_word(comp_name, sizeof (comp_name))) {
		const elec_comp_t *comp = libelec_comp_find(sys, comp_name);
		const elec_comp_info_t *info;

		if (comp == NULL) {
			fprintf(stderr, "Error: component %s not found\n",
			    comp_name);
			goto errout;
		}
		info = libelec_comp2info(comp);
		if (IS_NULL_VECT(info->gui.pos)) {
			fprintf(stderr, "Error: component %s has no defined "
			    "graphical position\n", comp_name);
			goto errout;
		}
		libelec_draw_comp_info(comp, cr, pos_scale, fontsz,
		    info->gui.pos);
	}
	cairo_surface_write_to_png(surf, subcmd);
	lacf_strlcpy(filename, subcmd, sizeof (filename));
errout:
	cairo_destroy(cr);
	cairo_surface_destroy(surf);
}

static void
print_help(const char *cmd)
{
	bool cmd_found = false;

	/* cmd can be NULL */
	if (cmd == NULL) {
		printf(
		    "nettest expects that you type interactive commands after "
		    "the '>' prompt.\n"
		    "Commands use the following general syntax:\n"
		    "\n"
		    "	COMMAND [optional_argument(s)...]\n"
		    "	COMMAND <mandatory_argument> SUBCOMMAND "
		    "[optional_argument(s)...]\n"
		    "\n"
		    "If an argument uses angle brackets (\"<something>\"), you "
		    "MUST provide a value\n"
		    "for the argument. If the argument uses square brackets "
		    "instead (\"[something]\"),\n"
		    "the argument is optional and may be omitted.\n"
		    "Most commands have a plain version without any arguments. "
		    "Those commands will\n"
		    "print out the state of all the instances of a given "
		    "component type as a table.\n");
#ifdef	WITH_READLINE
		printf(
		    "\n"
		    "nettest has been compiled with GNU readline support. "
		    "That means you can use\n"
		    "readline-style editing features:\n"
		    "	- use the <TAB> key for command and component name "
		    "auto-completion\n"
		    "	- use the up- and down-arrow keys to navigate command "
		    "history\n"
		    "	- use Ctrl+R for reverse-i-search in the command "
		    "history\n"
		    "	- use meta-B and meta-F for backward and forward "
		    "jumping "
		    "by word\n"
		    "	- use Ctrl+A and Ctrl+E to the start or end of the "
		    "line\n"
		    "	  (etc.)\n"
		    "\n");
#endif	/* defined(WITH_READLINE) */
	}
	if (cmd == NULL) {
		printf(
		    "===============\n"
		    "==== BUSES ====\n"
		    "===============\n");
	}
	if (cmd == NULL || lacf_strcasecmp(cmd, "bus") == 0) {
		cmd_found = true;
		printf(
		    "bus [BUS_NAME]\n"
		    "    Print all buses with voltages, currents and power "
		    "flows. Use this to get a\n"
		    "    quick overview of the network state. If you provide "
		    "an optional bus name,\n"
		    "    only the data for the listed bus will be printed. "
		    "Table columns:\n"
		    "	U - voltage on the bus\n"
		    "bus <BUS_NAME> list [DEVICE ...]\n"
		    "    Lists the state of all devices attached to the "
		    "specified bus. You may\n"
		    "    provide an optional list of devices to narrow the "
		    "printout to only those\n"
		    "    devices listed.\n");
	}
	if (cmd == NULL) {
		printf("\n"
		    "====================\n"
		    "==== GENERATORS ====\n"
		    "====================\n");
	}
	if (cmd == NULL || lacf_strcasecmp(cmd, "gen") == 0) {
		cmd_found = true;
		printf(
		    "gen [GEN_NAME]\n"
		    "    Prints all generators on the network. If you provide "
		    "an optional generator\n"
		    "    name, only the data for the listed generator will be "
		    "printed. Table columns:\n"
		    "	RPM - the RPM value at which the generator is "
		    "currently operating\n"
		    "	W_in - input power demand from the generator on its "
		    "mechanical input.\n"
		    "	Eff - current generator efficiency in percent.\n"
		    "	U_out - current generator output voltage.\n"
		    "	I_out - current generator output current.\n"
		    "	W_out - current generator output power load.\n"
		    "gen <GEN_NAME> rpm <RPM>\n"
		    "    Sets a new generator rpm value in the same units as "
		    "what was used in the\n"
		    "    electrical network definition.\n");
	}
	if (cmd == NULL) {
		printf("\n"
		    "==========================\n"
		    "==== TRUS & INVERTERS ====\n"
		    "==========================\n");
	}
	if (cmd == NULL || lacf_strcasecmp(cmd, "tru") == 0) {
		cmd_found = true;
		printf(
		    "tru [TRU_NAME|INV_NAME]\n"
		    "    Prints all TRUs and inverters on the network. If "
		    "you provide an optional\n"
		    "    TRU/inverter name, only the data for the listed "
		    "TRU/inverter will be\n"
		    "    printed. Table columns:\n"
		    "	U_in - input voltage into the TRU/inverter in Volts\n"
		    "	W_in - input power draw into the TRU/inverter in "
		    "Watts\n"
		    "	Eff - TRU/inverter power conversion efficiency in "
		    "percent\n"
		    "	U_out - output voltage out of the TRU/inverter in "
		    "Volts\n"
		    "	I_out - output current out of the TRU/inverter in "
		    "Amps\n"
		    "	W_out - output power out of the TRU/inverter in "
		    "Watts\n");
	}
	if (cmd == NULL) {
		printf("\n"
		    "===============\n"
		    "==== LOADS ====\n"
		    "===============\n");
	}
	if (cmd == NULL || lacf_strcasecmp(cmd, "load") == 0) {
		cmd_found = true;
		printf(
		    "load [LOAD_NAME]\n"
		    "    Print all loads. If you provide an optional load "
		    "name, only the data for\n"
		    "    the listed load will be printed. "
		    "Table columns:\n"
		    "	U_out - output voltage out of the load's power supply\n"
		    "	I_out - output current out of the load's power supply\n"
		    "	W_out - output power out of the load's power supply\n"
		    "	U_c_in - voltage of the power supply's virtual input "
		    "capacitance\n"
		    "	I_in - input current into the load's power supply\n"
		    "load <LOAD_NAME> set <AMPS|WATTS>\n"
		    "    Configures a constant load for an ELEC_LOAD device. "
		    "Whether the load expects\n"
		    "    a specification in Amps or Watts depends on whether "
		    "the device is declared\n"
		    "    as having a stabilized input power supply or not in "
		    "the network definition\n"
		    "    (STAB line).\n"
		    "    N.B. the load argument must NOT be negative.\n");
	}
	if (cmd == NULL) {
		printf("\n"
		    "==========================\n"
		    "==== CIRCUIT BREAKERS ====\n"
		    "==========================\n");
	}
	if (cmd == NULL || lacf_strcasecmp(cmd, "cb") == 0) {
		cmd_found = true;
		printf(
		    "cb [CB_NAME]\n"
		    "    Print the state of all circuit breakers. If you "
		    "provide an optional\n"
		    "    breaker name, only the data for the listed breaker "
		    "will be printed.\n"
		    "cb <CB_NAME> set <Y|N|1|0>\n"
		    "    Sets/resets a circuit breaker. A breaker that's set "
		    "('1' or 'Y')\n"
		    "    allows current flow, while a reset breaker "
		    "('0' or 'N') does not.\n");
	}
	if (cmd == NULL) {
		printf("\n"
		    "==============\n"
		    "==== TIES ====\n"
		    "==============\n");
	}
	if (cmd == NULL || lacf_strcasecmp(cmd, "tie") == 0) {
		cmd_found = true;
		printf(
		    "tie [TIE_NAME]\n"
		    "    Prints a list of all ties and their state. The "
		    "table lists each tie,\n"
		    "    and a list of buses currently tied into it. If you "
		    "provide an optional\n"
		    "    tie name, only the data for the listed tie will be "
		    "printed.\n"
		    "tie <TIE_NAME> <all|none|BUS1 BUS2 ...>\n"
		    "    Configures a tie. The remaining arguments must "
		    "be a list of buses to\n"
		    "    which the tie connects. Any buses not mentioned "
		    "will become untied.\n"
		    "    You can also use the symbolic keywords \"none\" and "
		    "\"all\" to untie and\n"
		    "    tie all buses connected to the tie, "
		    "respectively.\n");
	}
	if (cmd == NULL) {
		printf("\n"
		    "===================\n"
		    "==== BATTERIES ====\n"
		    "===================\n");
	}
	if (cmd == NULL || lacf_strcasecmp(cmd, "batt") == 0) {
		cmd_found = true;
		printf(
		    "batt [BATT_NAME]\n"
		    "    Print the state of all batteries. If you provide "
		    "an optional battery name,\n"
		    "    only the data for the listed battery will be printed. "
		    "Table columns:\n"
		    "	U_out - output voltage in Volts\n"
		    "	I_out - discharge current in Amps\n"
		    "	I_in - recharge current in Amps\n"
		    "	CHG - relative state of charge in percent\n"
		    "	TEMP - temperature in degrees Celsius\n"
		    "batt <BATT_NAME> chg <0..100>\n"
		    "    Sets a new relative charge state of the battery "
		    "in percent.\n"
		    "batt <BATT_NAME> temp <TEMP\u00B0C>\n"
		    "    Sets a new battery temperature in degrees Celsius.\n");
	}
	if (cmd == NULL) {
		printf("\n"
		    "=======================\n"
		    "==== IMAGE DRAWING ====\n"
		    "=======================\n");
	}
	if (cmd == NULL || lacf_strcasecmp(cmd, "draw") == 0) {
		cmd_found = true;
		printf(
		    "draw [filename.png] [COMP_NAME]\n"
		    "    Draw a rendered image of the network. Your network "
		    "must use \"GUI_*\"\n"
		    "    stanzas in its network definition to control how the "
		    "render is to be done.\n"
		    "    This is useful for quickly iterating on the network "
		    "render, instead of\n"
		    "    having to wait for an aircraft reload in the "
		    "simulator. The rendering\n"
		    "    offset, scale factor, font size and image size can "
		    "be changed using the\n"
		    "    subcommands below.\n"
		    "    N.B. the filename argument is optional only after "
		    "the first successful\n"
		    "    invocation of this command. This defines which file "
		    "nettest is supposed\n"
		    "    to write into. Subsequent invocations of the \"draw\" "
		    "command without\n"
		    "    arguments simply overwrite this file with a new "
		    "image when it becomes\n"
		    "    available. If you specify a component following "
		    "the filename, that\n"
		    "    component is drawn with its details box overlaid on "
		    "top of it, as if\n"
		    "    the user had clicked on it in the interactive network "
		    "visualizer.\n"
		    "draw offset <pixels_x> <pixels_y>\n"
		    "    Sets the network drawing offset in pixels. The "
		    "default offset is zero\n"
		    "    for both X and Y.\n"
		    "draw scale <scale>\n"
		    "    Sets the rendering scale for network drawing. The "
		    "default rendering scale\n"
		    "    is 16.\n"
		    "draw fontsz <size>\n"
		    "    Sets the font size for network drawing. The default "
		    "font size is 14 points.\n"
		    "draw imgsz <pixels_x> <pixels_y>\n"
		    "    Sets the image size for network drawing. The default "
		    "image size is\n"
		    "    2048x2048 pixels.\n");
	}
	if (cmd != NULL && !cmd_found) {
		fprintf(stderr, "Error: unknown command \"%s\". "
		    "Try typing \"help\".\n", cmd);
	}
}

#ifdef	WITH_READLINE

enum { MAX_SUBCMD_PARTS = 8 };

#define	COMP_TYPE_ANY_MASK \
	((1 << ELEC_BATT) | (1 << ELEC_GEN) | (1 << ELEC_TRU) | \
	(1 << ELEC_INV) | (1 << ELEC_LOAD) | (1 << ELEC_BUS) | \
	(1 << ELEC_CB) | (1 << ELEC_SHUNT) | (1 << ELEC_TIE) | \
	(1 << ELEC_DIODE))

typedef enum {
    CMD_PART_NONE = 0,
    CMD_PART_KEYWORD,
    CMD_PART_COMP_NAME,
    CMD_PART_FILE_NAME
} cmd_part_type_t;

typedef struct cmd_part_s {
	cmd_part_type_t		type;
	const char		*keyword;
	uint32_t		comp_type_mask;
	bool			variadic;
	bool			attached;
	bool			autogen;
	unsigned		comp_name_word_idx;
	const struct cmd_part_s	*subparts[MAX_SUBCMD_PARTS];
} cmd_part_t;

static const cmd_part_t *cmd_parts_top[] = {
    &(cmd_part_t){
	.type = CMD_PART_KEYWORD,
	.keyword = "bus",
	.subparts = {
	    &(cmd_part_t){
		.type = CMD_PART_COMP_NAME,
		.comp_type_mask = 1 << ELEC_BUS,
		.subparts = {
		    &(cmd_part_t){
			.type = CMD_PART_KEYWORD,
			.keyword = "list",
			.subparts = {
			    &(cmd_part_t){
				.type = CMD_PART_COMP_NAME,
				.comp_type_mask = COMP_TYPE_ANY_MASK,
				.attached = true,
				.comp_name_word_idx = 1,
				.variadic = true
			    }
			}
		    }
		}
	    }
	}
    },
    &(cmd_part_t){
	.type = CMD_PART_KEYWORD,
	.keyword = "gen",
	.subparts = {
	    &(cmd_part_t){
		.type = CMD_PART_COMP_NAME,
		.comp_type_mask = 1 << ELEC_GEN,
		.subparts = {
		    &(cmd_part_t){
			.type = CMD_PART_KEYWORD,
			.keyword = "rpm"
		    }
		}
	    }
	}
    },
    &(cmd_part_t){
	.type = CMD_PART_KEYWORD,
	.keyword = "tru",
	.subparts = {
	    &(cmd_part_t){
		.type = CMD_PART_COMP_NAME,
		.comp_type_mask = (1 << ELEC_TRU) | (1 << ELEC_INV)
	    }
	}
    },
    &(cmd_part_t){
	.type = CMD_PART_KEYWORD,
	.keyword = "load",
	.subparts = {
	    &(cmd_part_t){
		.type = CMD_PART_COMP_NAME,
		.comp_type_mask = 1 << ELEC_LOAD,
		.subparts = {
		    &(cmd_part_t){
			.type = CMD_PART_KEYWORD,
			.keyword = "set"
		    }
		}
	    }
	}
    },
    &(cmd_part_t){
	.type = CMD_PART_KEYWORD,
	.keyword = "cb",
	.subparts = {
	    &(cmd_part_t){
		.type = CMD_PART_COMP_NAME,
		.comp_type_mask = 1 << ELEC_CB,
		.autogen = true,
		.subparts = {
		    &(cmd_part_t){
			.type = CMD_PART_KEYWORD,
			.keyword = "set",
			.subparts = {
			    &(cmd_part_t){
				.type = CMD_PART_KEYWORD,
				.keyword = "0"
			    },
			    &(cmd_part_t){
				.type = CMD_PART_KEYWORD,
				.keyword = "1"
			    },
			    &(cmd_part_t){
				.type = CMD_PART_KEYWORD,
				.keyword = "N"
			    },
			    &(cmd_part_t){
				.type = CMD_PART_KEYWORD,
				.keyword = "Y"
			    }
			}
		    }
		}
	    }
	}
    },
    &(cmd_part_t){
	.type = CMD_PART_KEYWORD,
	.keyword = "tie",
	.subparts = {
	    &(cmd_part_t){
		.type = CMD_PART_COMP_NAME,
		.comp_type_mask = 1 << ELEC_TIE,
		.subparts = {
		    /*
		     * "all" and "none" must come first, because the
		     * component name is variadic, so it will stop
		     * any depth-search afterwards.
		     */
		    &(cmd_part_t){
			.type = CMD_PART_KEYWORD,
			.keyword = "all"
		    },
		    &(cmd_part_t){
			.type = CMD_PART_KEYWORD,
			.keyword = "none"
		    },
		    &(cmd_part_t){
			.type = CMD_PART_COMP_NAME,
			.comp_type_mask = 1 << ELEC_BUS,
			.attached = true,
			.comp_name_word_idx = 1,
			.subparts = {
			    &(cmd_part_t){
				.type = CMD_PART_COMP_NAME,
				.comp_type_mask = 1 << ELEC_BUS,
				.variadic = true,
				.attached = true,
				.comp_name_word_idx = 1
			    }
			}
		    }
		}
	    }
	}
    },
    &(cmd_part_t){
	.type = CMD_PART_KEYWORD,
	.keyword = "batt",
	.subparts = {
	    &(cmd_part_t){
		.type = CMD_PART_COMP_NAME,
		.comp_type_mask = 1 << ELEC_BATT,
		.subparts = {
		    &(cmd_part_t){
			.type = CMD_PART_KEYWORD,
			.keyword = "chg"
		    },
		    &(cmd_part_t){
			.type = CMD_PART_KEYWORD,
			.keyword = "temp"
		    }
		}
	    }
	}
    },
    &(cmd_part_t){
	.type = CMD_PART_KEYWORD,
	.keyword = "draw",
	.subparts = {
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "offset"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "scale"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "fontsz"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "imgsz"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_FILE_NAME,
		.subparts = {
		    &(cmd_part_t){
			.type = CMD_PART_COMP_NAME,
			.comp_type_mask = COMP_TYPE_ANY_MASK
		    }
		}
	    }
	}
    },
    &(cmd_part_t){
	.type = CMD_PART_KEYWORD,
	.keyword = "help",
	.subparts = {
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "bus"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "gen"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "tru"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "load"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "cb"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "tie"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "batt"
	    },
	    &(cmd_part_t){
		.type = CMD_PART_KEYWORD,
		.keyword = "draw"
	    }
	}
    }
};

static unsigned
find_completion_word_idx(int start, int end)
{
	size_t word_idx = 0;

	ASSERT(rl_line_buffer != NULL);

	for (int i = 0; rl_line_buffer[i] != '\0'; word_idx++) {
		/* Skip any leading whitespace */
		while (isspace(rl_line_buffer[i]) && rl_line_buffer[i] != '\0')
			i++;
		/* Reached end of line, stop */
		if (rl_line_buffer[i] == '\0')
			break;
		/* Search to the end of the word */
		while (!isspace(rl_line_buffer[i]) &&
		    rl_line_buffer[i] != '\0') {
			if (i >= start && i <= end)
				return (word_idx);
			i++;
		}
		/* Reached end of word, start next word */
	}
	return (word_idx);
}

static const cmd_part_t **
find_cur_cmd_part(const cmd_part_t **parts, unsigned num_parts,
    unsigned depth, unsigned word_idx,
    const char **comps, size_t n_comps, unsigned *num_parts_out)
{
	ASSERT(comps != NULL);
	ASSERT(num_parts_out != NULL);

	*num_parts_out = 0;
	if (depth >= word_idx) {
		*num_parts_out = num_parts;
		return (parts);
	}
	if (depth >= n_comps)
		return (NULL);
	for (unsigned part_i = 0;
	    part_i < num_parts && parts[part_i] != NULL &&
	    parts[part_i]->type != CMD_PART_NONE;
	    part_i++) {
		const cmd_part_t *part = parts[part_i];
		if (part->type == CMD_PART_KEYWORD &&
		    lacf_strcasecmp(comps[depth], part->keyword) == 0) {
			return (find_cur_cmd_part(
			    (const cmd_part_t **)part->subparts,
			    MAX_SUBCMD_PARTS, depth + 1, word_idx,
			    comps, n_comps, num_parts_out));
		} else if (part->type == CMD_PART_COMP_NAME ||
		    part->type == CMD_PART_FILE_NAME) {
			if (part->variadic) {
				return (find_cur_cmd_part(parts, num_parts,
				    depth + 1, word_idx, comps, n_comps,
				    num_parts_out));
			} else {
				return (find_cur_cmd_part(
				    (const cmd_part_t **)part->subparts,
				    MAX_SUBCMD_PARTS, depth + 1, word_idx,
				    comps, n_comps, num_parts_out));
			}
		}
	}
	return (NULL);
}

static const cmd_part_t **completion_parts = NULL;
static unsigned completion_num_parts = 0;
static unsigned completion_part_idx = 0;
static unsigned completion_info_idx = 0;

static bool
completion_check_comp_attachment(const char *name, const cmd_part_t *part)
{
	size_t n_comps;
	char **comps;
	const elec_comp_t *comp;
	bool result = false;

	ASSERT(name != NULL);
	ASSERT(part != NULL);
	ASSERT3U(part->type, ==, CMD_PART_COMP_NAME);

	if (!part->attached)
		return (true);
	comps = strsplit(rl_line_buffer, " ", true, &n_comps);
	if (part->comp_name_word_idx >= n_comps)
		goto errout;
	comp = libelec_comp_find(sys, comps[part->comp_name_word_idx]);
	if (comp == NULL)
		goto errout;
	result = check_comp_attachment(comp, name);
errout:
	free_strlist(comps, n_comps);
	return (result);
}

static char *
tab_completion_generator(const char *text, int state)
{
	const cmd_part_t *part;
	const elec_comp_info_t *infos;
	size_t num_infos;

	ASSERT(text != NULL);
	/*
	 * Disable readline's automatic filename completion unless
	 * explicitly enabled by a CMD_PART_FILE_NAME handler.
	 */
	rl_attempted_completion_over = 1;
	if (completion_parts == NULL ||
	    completion_part_idx >= completion_num_parts) {
		return (NULL);
	}
	part = completion_parts[completion_part_idx];
	if (part == NULL) {
		return (NULL);
	}
	switch (part->type) {
	case CMD_PART_NONE:
		return (NULL);
	case CMD_PART_KEYWORD:
		ASSERT(part->keyword != NULL);
		completion_info_idx = 0;
		if (lacf_strncasecmp(text, part->keyword, strlen(text)) == 0) {
			completion_part_idx++;
			return (safe_strdup(part->keyword));
		}
		completion_part_idx++;
		return (tab_completion_generator(text, state));
	case CMD_PART_COMP_NAME:
		infos = libelec_get_comp_infos(sys, &num_infos);
		for (; completion_info_idx < num_infos;
		    completion_info_idx++) {
			const elec_comp_info_t *info =
			    &infos[completion_info_idx];
			if (lacf_strncasecmp(text, info->name, strlen(text)) ==
			    0 && (!info->autogen || part->autogen) &&
			    (part->comp_type_mask & (1 << info->type)) &&
			    completion_check_comp_attachment(info->name,
			    part)) {
				completion_info_idx++;
				return (safe_strdup(info->name));
			}
		}
		completion_part_idx++;
		completion_info_idx = 0;
		return (tab_completion_generator(text, state));
	case CMD_PART_FILE_NAME:
		/* Re-enable  */
		rl_attempted_completion_over = 0;
		return (NULL);
	}
	VERIFY_FAIL();
}

static char **
tab_completion(const char *text, int start, int end)
{
	unsigned word_idx = find_completion_word_idx(start, end);
	size_t n_comps;
	char **comps = strsplit(rl_line_buffer, " ", true, &n_comps);
	unsigned num_parts;
	const cmd_part_t **parts = find_cur_cmd_part(cmd_parts_top,
	    ARRAY_NUM_ELEM(cmd_parts_top), 0, word_idx,
	    (const char **)comps, n_comps, &num_parts);

	ASSERT(text != NULL);

	free_strlist(comps, n_comps);

	completion_parts = parts;
	completion_num_parts = num_parts;
	completion_part_idx = 0;
	completion_info_idx = 0;
	return (rl_completion_matches(text, tab_completion_generator));
}

#endif	/* defined(WITH_READLINE) */

/*
 * Read user commands from the provided file pointer line-by-line and
 * perform them. See print_help() for a list of commands we support.
 */
static bool
read_commands(FILE *fp, const char *filename, bool interactive)
{
	char *cmdline = NULL;
	size_t cmdline_cap = 0;
	char cmd[32];

	ASSERT(fp != NULL);
	ASSERT(filename != NULL);

	for (int linenum = 0;; linenum++) {
		ssize_t n_chars;
#ifdef	WITH_READLINE
		if (interactive) {
			cmdline = readline("> ");
			if (cmdline == NULL)
				break;
		} else {
			n_chars = lacf_getline(&cmdline, &cmdline_cap, fp);
			if (n_chars <= 0)
				break;
		}
#else	/* !defined(WITH_READLINE) */
		if (interactive) {
			printf("> ");
			fflush(stdout);
		}
		n_chars = lacf_getline(&cmdline, &cmdline_cap, fp);
		if (n_chars <= 0)
			break;
#endif	/* !defined(WITH_READLINE) */
		ASSERT(cmdline != NULL);
		for (size_t i = 0, n = strlen(cmdline); i < n; i++) {
			if (isspace(cmdline[i])) {
				cmdline[i] = ' ';
			} else if (cmdline[i] == '#') {
				/* Strip away comments */
				cmdline[i] = '\0';
				break;
			}
		}
		strip_space(cmdline);
		if (cmdline[0] == '\0') {
			free(cmdline);
			cmdline = NULL;
			cmdline_cap = 0;
			continue;
		}
#ifdef	WITH_READLINE
		if (interactive)
			add_history(cmdline);
#endif	/* defined(WITH_READLINE) */
		DESTROY_STRLIST(cmd_comps, n_cmd_comps);
		cmd_comps = strsplit(cmdline, " ", true, &n_cmd_comps);
		cmd_comp_i = 0;

		if (!get_next_word(cmd, sizeof (cmd)))
			break;
		if (lacf_strcasecmp(cmd, "quit") == 0) {
			break;
		} else if (lacf_strcasecmp(cmd, "bus") == 0) {
			bus_cmd();
		} else if (lacf_strcasecmp(cmd, "gen") == 0) {
			gen_cmd();
		} else if (lacf_strcasecmp(cmd, "tru") == 0) {
			tru_cmd();
		} else if (lacf_strcasecmp(cmd, "load") == 0) {
			load_cmd();
		} else if (lacf_strcasecmp(cmd, "tie") == 0) {
			tie_cmd();
		} else if (lacf_strcasecmp(cmd, "cb") == 0) {
			cb_cmd();
		} else if (lacf_strcasecmp(cmd, "batt") == 0) {
			batt_cmd();
		} else if (lacf_strcasecmp(cmd, "draw") == 0) {
			draw_cmd();
		} else if (lacf_strcasecmp(cmd, "help") == 0) {
			char subcmd[32];
			if (get_next_word(subcmd, sizeof (subcmd)))
				print_help(subcmd);
			else
				print_help(NULL);
#ifdef	LIBELEC_SLOW_DEBUG
		} else if (lacf_strcasecmp(cmd, "s") == 0) {
			libelec_step(sys);
#endif	/* LIBELEC_SLOW_DEBUG */
		} else {
			if (interactive) {
				fprintf(stderr, "Error: unknown command: "
				    "\"%s\". Try typing \"help\".\n", cmd);
			} else {
				fprintf(stderr, "Error: %s:%d: Unknown "
				    "command: \"%s\"\n", filename, linenum,
				    cmd);
			}
		}
		free(cmdline);
		cmdline = NULL;
		cmdline_cap = 0;
	}
	DESTROY_STRLIST(cmd_comps, n_cmd_comps);
#ifndef	WITH_READLINE
	free(cmdline);
#endif
	return (true);
}

static void
setup_comp_binds(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info;

	ASSERT(comp != NULL);
	info = libelec_comp2info(comp);
	UNUSED(userinfo);

	if (info->type != ELEC_LOAD || info->load.std_load != 0)
		return;
	libelec_load_set_load_cb(comp, get_load);
}

int
main(int argc, char **argv)
{
	const char *filename = NULL, *init_filename = NULL;
	int opt;
	void *cookie;
#ifdef	LIBELEC_WITH_NETLINK
	const char *send_url = NULL, *recv_url = NULL;
#endif
	load_info_t *load_info;
	/*
	 * Initialize libacfutils' logging facility.
	 */
	log_init(debug_print, "test");
	/*
	 * We need to initliaze libacfutils' CRC64 machinery and fast PRNG.
	 * For debugging purposes, we initialize the PRNG to the same zero
	 * seed every time, to make testing reproducible.
	 */
	crc64_init();
	crc64_srand(0);

#ifdef	WITH_READLINE
	rl_attempted_completion_function = tab_completion;
#endif
	avl_create(&load_infos, load_info_compar, sizeof (load_info_t),
	    offsetof(load_info_t, node));
	/*
	 * Command line argument parsing.
	 */
	while ((opt = getopt(argc, argv, "hvi:s:r:JC")) != -1) {
		switch (opt) {
		case 'h':
			print_usage(stdout, argv[0]);
			exit(EXIT_SUCCESS);
		case 'v':
			print_version();
			exit(EXIT_SUCCESS);
		case 'i':
			init_filename = optarg;
			break;
		case 'J':
			output_format = FORMAT_JSON;
			break;
		case 'C':
			output_format = FORMAT_CSV;
			break;
#ifdef	LIBELEC_WITH_NETLINK
		case 's':
			if (recv_url != NULL) {
				logMsg("-s and -r are mutually exclusive");
				exit(EXIT_FAILURE);
			}
			send_url = optarg;
			break;
		case 'r':
			if (send_url != NULL) {
				logMsg("-s and -r are mutually exclusive");
				exit(EXIT_FAILURE);
			}
			recv_url = optarg;
			break;
#endif	/* defined(LIBELEC_WITH_NETLINK) */
		default: /* '?' */
			print_usage(stderr, argv[0]);
			exit(EXIT_FAILURE);
		}
	}
	/*
	 * Check for the last non-option argument. That's our network filename.
	 */
	if (optind + 1 > argc) {
		print_usage(stderr, argv[0]);
		exit(EXIT_FAILURE);
	}
	filename = argv[optind++];
	/*
	 * libelec initialization. We now have the network definition file
	 * in `filename`, so just pass that to libelec_new() to load.
	 */
	sys = libelec_new(filename);
	if (sys == NULL)
		exit(EXIT_FAILURE);
	/*
	 * We want to configure all ELEC_LOAD devices to use our get_load()
	 * callback, so we can dynamically modify their electrical load.
	 */
	libelec_walk_comps(sys, setup_comp_binds, NULL);
	/*
	 * We allow the user to pass an "initial commands" file, to set up
	 * this test utility for easier testing. So this feature simply
	 * reads the file in `init_filename` and consumes user commands
	 * from that file, as-if the user had typed them on the interactive
	 * prompt right at startup.
	 */
	if (init_filename != NULL) {
		FILE *fp = fopen(init_filename, "r");
		if (fp == NULL) {
			fprintf(stderr, "Error: can't open %s: %s\n",
			    init_filename, strerror(errno));
			exit(EXIT_FAILURE);
		}
		if (!read_commands(fp, init_filename, false))
			exit(EXIT_FAILURE);
		fclose(fp);
	}
	/*
	 * Debugging of network synchronization.
	 */
#ifdef	LIBELEC_WITH_NETLINK
	if (send_url != NULL)
		libelec_enable_net_send(sys, send_url);
	if (recv_url != NULL)
		libelec_enable_net_recv(sys, recv_url);
#endif	/* defined(LIBELEC_WITH_NETLINK) */
	/*
	 * Having loaded and configured the network to our needs, we can
	 * start the libelec() physics thread.
	 */
	VERIFY(libelec_sys_start(sys));
	/*
	 * Read and implement the user's interactive commands from STDIN.
	 */
	read_commands(stdin, "<stdin>", output_format == FORMAT_HUMAN_READABLE);
	/*
	 * Shut down and free the network.
	 */
	libelec_sys_stop(sys);
	libelec_destroy(sys);
	/*
	 * Clean out any load_infos the user might have set up using the
	 * "load" command, then destroy the tree.
	 */
	cookie = NULL;
	while ((load_info = avl_destroy_nodes(&load_infos, &cookie)) != NULL)
		free(load_info);
	avl_destroy(&load_infos);

	return (0);
}

static double
get_load(elec_comp_t *comp, void *userinfo)
{
	load_info_t srch = { .info = libelec_comp2info(comp) };
	load_info_t *load_info;

	UNUSED(userinfo);

	load_info = avl_find(&load_infos, &srch, NULL);
	if (load_info != NULL)
		return (load_info->load);

	return (0);
}
