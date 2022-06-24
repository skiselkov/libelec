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

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stddef.h>

#include <acfutils/avl.h>
#include <acfutils/assert.h>
#include <acfutils/crc64.h>
#include <acfutils/helpers.h>
#include <acfutils/mt_cairo_render.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>

#include "libelec.h"
#include "libelec_drawing.h"

#define	WHITE_RGB	1, 1, 1
#define	BLACK_RGB	0, 0, 0
#define	LIGHT_GRAY_RGB	0.67, 0.67, 0.67
#define	DARK_GRAY_RGB	0.33, 0.33, 0.33

typedef struct {
	const elec_comp_info_t	*info;
	float			load;
	avl_node_t		node;
} load_info_t;

static elec_sys_t *sys = NULL;
static avl_tree_t load_infos;

static double get_gen_rpm(elec_comp_t *comp);
static double get_batt_temp(elec_comp_t *comp);
static double get_load(elec_comp_t *comp);

static elec_func_bind_t binds[] = {
    { "get_gen_rpm", get_gen_rpm },
    { "get_batt_temp", get_batt_temp },
    { "get_load", get_load },
    { "gen_1", NULL },
    { "gen_2", NULL },
    { "apu_gen", NULL },
    { "adg_gen", NULL },
    { "ac_gen", NULL },
    { "dc_gen", NULL },
    { "main_batt", NULL },
    { "apu_batt", NULL },
    { "ac_bus_load_1", NULL },
    { "ac_bus_load_2", NULL },
    { "get_load_hyd", get_load },
    { "get_load_adshc", get_load },
    { "apu_get_fuel_pmp_load", get_load },
    { "apu_get_ecu_load", get_load },
    { "apu_get_starter_load", get_load },
    { NULL, NULL }	/* list terminator */
};

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
	fprintf(fp, "Usage: %s [-h] <elec_file> [-s <url>|-r <url>]\n",
	    progname);
}

static void
debug_print(const char *str)
{
	fprintf(stderr, "%s",str);
}

static elec_comp_info_t *
name2info(const char *name)
{
	size_t num_infos;
	elec_comp_info_t *infos = libelec_get_comp_infos(sys, &num_infos);
	for (size_t i = 0; i < num_infos; i++) {
		if (strcmp(name, infos[i].name) == 0)
			return (&infos[i]);
	}
	return (NULL);
}

static elec_comp_t *
name2comp(const char *name)
{
	elec_comp_info_t *info = name2info(name);

	if (info != NULL)
		return (libelec_info2comp(sys, info));
	return (NULL);
}

static void
paint_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);

	UNUSED(userinfo);

	if ((info->type != ELEC_BUS && info->type != ELEC_TRU) ||
	    info->autogen) {
		return;
	}

	printf("%-24s %5.1fV %5.1fA %4.0fW %5.1fV %5.1fA %4.0fW\n",
	    info->name,
	    libelec_comp_get_in_volts(comp),
	    libelec_comp_get_in_amps(comp),
	    libelec_comp_get_in_pwr(comp),
	    libelec_comp_get_out_volts(comp),
	    libelec_comp_get_out_amps(comp),
	    libelec_comp_get_out_pwr(comp));
}

static void
paint(void)
{
	printf(
	    "BUS NAME                  U_in   I_in  W_in  "
	    "U_out  I_out W_out\n"
	    "-----------------        -----  ----- -----  "
	    "-----  ----- -----\n");
       libelec_walk_comps(sys, paint_i, NULL);
}

static void
print_cbs_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);

	UNUSED(userinfo);

	if (info->type != ELEC_CB)
		return;

	printf("%-24s %5.1fV %5.1fA %5.2f  %3s\n",
	    info->name,
	    libelec_comp_get_in_volts(comp),
	    libelec_comp_get_in_amps(comp),
	    libelec_cb_get_temp(comp),
	    libelec_cb_get(comp) ? "Y" : "");
}

static void
print_cbs(void)
{
	printf(
	    "CB NAME                        U      I  TEMP  "
	    "SET\n"
	    "-------                    -----  -----  ----  "
	    "---\n");
	libelec_walk_comps(sys, print_cbs_i, NULL);
}

static void
print_loads_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);

	UNUSED(userinfo);

	if (info->type != ELEC_LOAD)
		return;

	printf("%-30s %6.1fV %5.1fA %6.1fW %6.1fV %5.1fA\n",
	    info->name,
	    libelec_comp_get_out_volts(comp),
	    libelec_comp_get_out_amps(comp),
	    libelec_comp_get_out_pwr(comp),
	    libelec_comp_get_incap_volts(comp),
	    libelec_comp_get_in_amps(comp));
}

static void
print_loads(void)
{
	printf(
	    "NAME                                         "
	    " U_out  I_out   W_out  U_c_in   I_in\n"
	    "------------------                           "
	    "------  -----  ------  ------  -----\n");
	libelec_walk_comps(sys, print_loads_i, NULL);
}

static void
print_ties_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);
	elec_comp_t *bus_list[libelec_comp_get_num_conns(comp)];
	size_t n_tied;

	UNUSED(userinfo);

	if (info->type != ELEC_TIE)
		return;

	printf("%-25s ", info->name);

	n_tied = libelec_tie_get(comp, bus_list);
	for (size_t i = 0; i < n_tied; i++) {
		const elec_comp_info_t *bus_info =
		    libelec_comp2info(bus_list[i]);
		printf("%s%s", bus_info->name, (i + 1 < n_tied) ? "," : "");
	}

	printf("\n");
}

static void
print_ties(void)
{
	printf("NAME                      BUSES\n"
	    "------------------------  ------------------\n");
	libelec_walk_comps(sys, print_ties_i, NULL);
}

static void
tie(void)
{
	char tie[64] = { 0 }, buses[1024] = { 0 };
	elec_comp_t *comp;

	if (scanf("%63s %1023s", tie, buses) != 2)
		return;

	comp = name2comp(tie);
	if (comp == NULL) {
		fprintf(stderr, "Unknown tie name %s\n", tie);
		return;
	}
	if (strcmp(buses, "none") == 0) {
		libelec_tie_set_all(comp, B_FALSE);
	} else if (strcmp(buses, "all") == 0) {
		libelec_tie_set_all(comp, B_TRUE);
	} else {
		elec_comp_info_t **bus_list;
		size_t list_len;
		char **comps = strsplit(buses, ",", B_FALSE, &list_len);

		bus_list = safe_calloc(list_len, sizeof (*bus_list));
		for (size_t i = 0; i < list_len; i++) {
			bus_list[i] = name2info(comps[i]);
			if (bus_list[i] == NULL) {
				fprintf(stderr, "Unknown bus name %s\n",
				    comps[i]);
				free_strlist(comps, list_len);
				free(bus_list);
				return;
			}
		}

		libelec_tie_set_info_list(comp, bus_list, list_len);

		free_strlist(comps, list_len);
		free(bus_list);
	}
}

static void
print_batts_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);

	UNUSED(userinfo);

	if (info->type != ELEC_BATT)
		return;

	printf("%-18s  %5.1fV  %6.1fA  %6.1fA  %.2f%%\n",
	    info->name, libelec_comp_get_out_volts(comp),
	    libelec_comp_get_in_amps(comp),
	    libelec_comp_get_out_amps(comp),
	    libelec_batt_get_chg_rel(comp) * 100);
}

static void
print_batts(void)
{
	printf("NAME                 U_out     I_in    I_out     CHG%% \n"
	    "------------------  ------  -------  -------  -------\n");
	libelec_walk_comps(sys, print_batts_i, NULL);
}

static void
cb(void)
{
	char cb_name[64];
	elec_comp_t *comp;
	bool_t set;

	if (scanf("%63s %d", cb_name, &set) != 2)
		return;
	comp = name2comp(cb_name);
	if (comp == NULL) {
		fprintf(stderr, "Unknown CB %s\n", cb_name);
		return;
	}
	libelec_cb_set(comp, set);
}

static void
batt(void)
{
	char batt_name[64];
	elec_comp_t *comp;
	float chg_rel;

	if (scanf("%63s %f", batt_name, &chg_rel) != 2)
		return;
	comp = name2comp(batt_name);
	if (comp == NULL) {
		fprintf(stderr, "Unknown batt %s\n", batt_name);
		return;
	}
	libelec_batt_set_chg_rel(comp, clamp(chg_rel, 0, 1));
}

static void
load_cmd(void)
{
	char name[64];
	elec_comp_info_t *info;
	load_info_t srch, *load_info;
	float load;
	avl_index_t where;

	if (scanf("%63s %f", name, &load) != 2)
		return;
	info = name2info(name);
	if (info == NULL) {
		fprintf(stderr, "Unknown load %s\n", name);
		return;
	}
	srch.info = info;
	load_info = avl_find(&load_infos, &srch, &where);
	if (load_info == NULL) {
		load_info = safe_calloc(1, sizeof (*load_info));
		load_info->info = info;
		avl_insert(&load_infos, load_info, where);
	}
	load_info->load = load;
}

static void
dump_img(void)
{
	char filename[64];
	cairo_surface_t *surf;
	cairo_t *cr;

	if (scanf("%63s", filename) != 1)
		return;

	surf = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 2048, 2048);
	cr = cairo_create(surf);

	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_paint(cr);
	cairo_translate(cr, 600, 0);

	cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL,
	    CAIRO_FONT_WEIGHT_NORMAL);

	libelec_draw_layout(sys, cr, 16, 14);

	cairo_surface_write_to_png(surf, filename);
	cairo_destroy(cr);
	cairo_surface_destroy(surf);
}

static void
print_help(void)
{
	printf("p - print buses\n"
	    "l - print loads\n"
	    "cbs - print circuit breakers\n"
	    "cb <CB_NAME> <0|1> - sets/resets a circuit breaker\n"
	    "ties - print ties\n"
	    "tie <TIE_NAME> <BUS1,BUS2,...> - tie buses together\n"
	    "load <LOAD_NAME> <WATTS> - configures a constant load\n"
	    "dump <filename.png> <BUS_NAME> - dump image of network at bus\n");
}

int
main(int argc, char **argv)
{
	const char *filename;
	int opt;
	char cmd[32];
	void *cookie;
	load_info_t *load_info;
	const char *send_url = NULL, *recv_url = NULL;

	crc64_init();
	crc64_srand(0);
	log_init(debug_print, "test");

	avl_create(&load_infos, load_info_compar, sizeof (load_info_t),
	    offsetof(load_info_t, node));

	while ((opt = getopt(argc, argv, "hs:r:")) != -1) {
		switch (opt) {
		case 'h':
			print_usage(stdout, argv[0]);
			exit(EXIT_SUCCESS);
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
		default: /* '?' */
			print_usage(stderr, argv[0]);
			exit(EXIT_FAILURE);
		}
	}
	if (optind + 1 > argc) {
		print_usage(stderr, argv[0]);
		exit(EXIT_FAILURE);
	}
	filename = argv[optind++];

	sys = libelec_new(filename, binds);
	if (sys == NULL)
		exit(EXIT_FAILURE);
	if (send_url != NULL)
		libelec_enable_net_send(sys, send_url);
	if (recv_url != NULL)
		libelec_enable_net_recv(sys, recv_url);
	libelec_sys_start(sys);

	for (;;) {
		printf("> ");
		fflush(stdout);
		if (scanf("%31s", cmd) != 1)
			break;
		if (strcmp(cmd, "p") == 0) {
			paint();
		} else if (strcmp(cmd, "l") == 0) {
			print_loads();
		} else if (strcmp(cmd, "cbs") == 0) {
			print_cbs();
		} else if (strcmp(cmd, "ties") == 0) {
			print_ties();
		} else if (strcmp(cmd, "tie") == 0) {
			tie();
		} else if (strcmp(cmd, "cb") == 0) {
			cb();
		} else if (strcmp(cmd, "load") == 0) {
			load_cmd();
		} else if (strcmp(cmd, "dump") == 0) {
			dump_img();
		} else if (strcmp(cmd, "b") == 0) {
			print_batts();
		} else if (strcmp(cmd, "batt") == 0) {
			batt();
		} else if (strcmp(cmd, "help") == 0) {
			print_help();
#ifdef	LIBELEC_SLOW_DEBUG
		} else if (strcmp(cmd, "s") == 0) {
			libelec_step(sys);
#endif	/* LIBELEC_SLOW_DEBUG */
		} else {
			fprintf(stderr, "Unknown command: \"%s\". "
			    "Try \"help\"\n", cmd);
		}
	}

	libelec_sys_stop(sys);
	libelec_destroy(sys);

	cookie = NULL;
	while ((load_info = avl_destroy_nodes(&load_infos, &cookie)) != NULL)
		free(load_info);
	avl_destroy(&load_infos);

	return (0);
}


static double
get_gen_rpm(elec_comp_t *comp)
{
	UNUSED(comp);
	return (100);
}

static double
get_batt_temp(elec_comp_t *comp)
{
	UNUSED(comp);
	return (C2KELVIN(15));
}

static double
get_load(elec_comp_t *comp)
{
	load_info_t srch = { .info = libelec_comp2info(comp) };
	load_info_t *load_info;

	load_info = avl_find(&load_infos, &srch, NULL);
	if (load_info != NULL)
		return (load_info->load);

	return (0);
}
