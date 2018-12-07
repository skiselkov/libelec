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

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <acfutils/assert.h>
#include <acfutils/crc64.h>
#include <acfutils/helpers.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>

#include "libelec.h"

static elec_sys_t *sys = NULL;
static elec_comp_info_t *infos = NULL;
static size_t num_infos = 0;

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
    { NULL, NULL }	/* list terminator */
};

static void
print_usage(FILE *fp, const char *progname)
{
	fprintf(fp, "Usage: %s [-h] <elec_file>\n", progname);
}

static void
debug_print(const char *str)
{
	fprintf(stderr, "%s",str);
}

static elec_comp_info_t *
name2info(const char *name)
{
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
	elec_comp_t *src = libelec_comp_get_src(comp);
	const elec_comp_info_t *src_info =
	    (src != NULL ? libelec_comp2info(src) : NULL);
	const char *src_name;

	UNUSED(userinfo);

	if (info->type != ELEC_BUS)
		return;

	src_name = (src_info != NULL ? src_info->name : "");
	printf("%-20s %-12s %5.1fV %5.1fA %5.1fV %5.1fA\n",
	    info->name, src_name,
	    libelec_comp_get_in_volts(comp),
	    libelec_comp_get_in_amps(comp),
	    libelec_comp_get_out_volts(comp),
	    libelec_comp_get_out_amps(comp));
}

static void
paint(void)
{
	libelec_walk_comps(sys, paint_i, NULL);
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

		libelec_tie_set(comp, bus_list, list_len);

		free_strlist(comps, list_len);
		free(bus_list);
	}
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

int
main(int argc, char **argv)
{
	const char *filename;
	int opt;
	char cmd[32];

	crc64_init();
	log_init(debug_print, "test");

	while ((opt = getopt(argc, argv, "h")) != -1) {
		switch (opt) {
		case 'h':
			print_usage(stdout, argv[0]);
			exit(EXIT_SUCCESS);
		default: /* '?' */
			print_usage(stderr, argv[0]);
			exit(EXIT_FAILURE);
		}
	}
	if (optind + 1 > argc) {
		print_usage(stderr, argv[0]);
		return (1);
	}
	filename = argv[optind++];

	infos = libelec_infos_parse(filename, binds, &num_infos);
	if (num_infos == 0)
		return (1);
	sys = libelec_new(infos, num_infos);
	ASSERT(sys != NULL);

	for (;;) {
		printf("> ");
		fflush(stdout);
		if (scanf("%31s", cmd) != 1)
			break;
		if (strcmp(cmd, "p") == 0) {
			paint();
		} else if (strcmp(cmd, "tie") == 0) {
			tie();
		} else if (strcmp(cmd, "cb") == 0) {
			cb();
		}
	}

	libelec_destroy(sys);
	libelec_parsed_info_free(infos, num_infos);

	return (0);
}


static double
get_gen_rpm(elec_comp_t *comp)
{
	UNUSED(comp);
	return (2000);
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
	UNUSED(comp);
	return (1);
}
