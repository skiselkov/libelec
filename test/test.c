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
#include "libelec_test.h"

#define	WHITE_RGB	1, 1, 1
#define	BLACK_RGB	0, 0, 0
#define	LIGHT_GRAY_RGB	0.67, 0.67, 0.67
#define	DARK_GRAY_RGB	0.33, 0.33, 0.33

#define	COLOR_TABLE_SZ	16
static const double color_table[COLOR_TABLE_SZ][3] = {
     { 0.00, 0.00, 0.00 },
     { 0.00, 0.00, 0.67 },
     { 0.00, 0.67, 0.00 },
     { 0.00, 0.67, 0.67 },
     { 0.67, 0.00, 0.00 },
     { 0.67, 0.00, 0.67 },
     { 0.67, 0.33, 0.00 },
     { 0.67, 0.67, 0.67 },
     { 0.33, 0.33, 0.33 },
     { 0.33, 0.33, 1.00 },
     { 0.33, 1.00, 0.33 },
     { 0.33, 1.00, 1.00 },
     { 1.00, 0.33, 0.33 },
     { 1.00, 0.33, 1.00 },
     { 1.00, 1.00, 0.33 },
     { 0.00, 0.33, 0.33 }
};

typedef struct {
	const elec_comp_info_t	*info;
	float			load;
	avl_node_t		node;
} load_info_t;

static elec_sys_t *sys = NULL;
static elec_comp_info_t *infos = NULL;
static size_t num_infos = 0;
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
	elec_comp_t *upstream = libelec_comp_get_upstream(comp);
	const elec_comp_info_t *upstream_info =
	    (upstream != NULL ? libelec_comp2info(upstream) : NULL);
	const char *src_name, *upstream_name;

	UNUSED(userinfo);

	if ((info->type != ELEC_BUS && info->type != ELEC_TRU) ||
	    info->autogen)
		return;

	src_name = (src_info != NULL ? src_info->name : "");
	upstream_name = (upstream_info != NULL ? upstream_info->name : "");
	UNUSED(upstream_name);
	printf("%-24s %-12s %5.1fV %5.1fA %4.0fW %5.1fV %5.1fA %4.0fW\n",
	    info->name, src_name,
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
	    "BUS NAME                 SRC            U_in   I_in  W_in  "
	    "U_out  I_out W_out\n"
	    "-----------------        -----------   -----  ----- -----  "
	    "-----  ----- -----\n");
	libelec_walk_comps(sys, paint_i, NULL);
}

static void
print_cbs_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);
	elec_comp_t *src = libelec_comp_get_src(comp);
	const elec_comp_info_t *src_info =
	    (src != NULL ? libelec_comp2info(src) : NULL);
	const char *src_name;

	UNUSED(userinfo);

	if (info->type != ELEC_CB)
		return;

	src_name = (src_info != NULL ? src_info->name : "");
	printf("%-24s %-14s %5.1fV %5.1fA %5.2f  %3s\n",
	    info->name, src_name,
	    libelec_comp_get_in_volts(comp),
	    libelec_comp_get_in_amps(comp),
	    libelec_cb_get_temp(comp),
	    libelec_cb_get(comp) ? "Y" : "");
}

static void
print_cbs(void)
{
	printf(
	    "CB NAME                    SRC               U      I  TEMP  "
	    "SET\n"
	    "-------                    ----------    -----  -----  ----  "
	    "---\n");
	libelec_walk_comps(sys, print_cbs_i, NULL);
}

static void
print_loads_i(elec_comp_t *comp, void *userinfo)
{
	const elec_comp_info_t *info = libelec_comp2info(comp);
	elec_comp_t *src = libelec_comp_get_src(comp);
	const elec_comp_info_t *src_info =
	    (src != NULL ? libelec_comp2info(src) : NULL);
	const char *src_name;

	UNUSED(userinfo);

	if (info->type != ELEC_LOAD)
		return;

	src_name = (src_info != NULL ? src_info->name : "");
	printf("%-30s %-12s %6.1fV %5.1fA %6.1fW %6.1fV %5.1fA\n",
	    info->name, src_name,
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

typedef struct {
	const char	*name;
	elec_comp_t	*comp;
} srch_info_t;

typedef struct {
	cairo_t		*cr;
	elec_comp_t	*upstream;
	double		rot;	/* radians */
} draw_info_t;

static void
find_comp(elec_comp_t *comp, void *userinfo)
{
	srch_info_t *info = userinfo;

	if (info->comp != NULL)
		return;
	if (strcmp(libelec_comp2info(comp)->name, info->name) == 0)
		info->comp = comp;
}

static void
draw_comp_name(const elec_comp_t *comp, const draw_info_t *di)
{
	const char *name = libelec_comp2info(comp)->name;
	cairo_text_extents_t te;
	double font_sizes[] = {
	    /* This matches the types in elec_comp_type_t */
	    14, 14, 16, 12, 17, 12, 14, 12
	};
	enum { BORDER = 2 };

	cairo_rotate(di->cr, -di->rot);

	cairo_set_font_size(di->cr, font_sizes[libelec_comp2info(comp)->type]);

	cairo_text_extents(di->cr, name, &te);

	cairo_set_source_rgba(di->cr, BLACK_RGB, 0.7);
	cairo_rectangle(di->cr, -te.width / 2 - BORDER,
	    -te.height / 2 - BORDER, te.width + 2 * BORDER,
	    te.height + 2 * BORDER);
	cairo_fill(di->cr);

	cairo_set_source_rgb(di->cr, WHITE_RGB);
	cairo_move_to(di->cr, -te.width / 2 - te.x_bearing,
	    -te.height / 2 - te.y_bearing);
	cairo_show_text(di->cr, name);

	cairo_rotate(di->cr, di->rot);
}

enum { SYMBOL_SZ = 35 };

static void
draw_bus(cairo_t *cr, double r, double g, double b)
{
	cairo_set_source_rgb(cr, BLACK_RGB);
	cairo_arc(cr, 0, 0, SYMBOL_SZ, 0, DEG2RAD(360));
	cairo_fill(cr);

	cairo_set_source_rgb(cr, r, g, b);
	cairo_arc(cr, 0, 0, SYMBOL_SZ * 0.8, 0, DEG2RAD(360));
	cairo_fill(cr);
}

static void
draw_cb(cairo_t *cr, double r, double g, double b)
{
	cairo_set_source_rgb(cr, WHITE_RGB);
	cairo_arc(cr, 0, 0, SYMBOL_SZ / 2, 0, DEG2RAD(360));
	cairo_fill(cr);

	cairo_set_source_rgb(cr, r, g, b);
	cairo_arc(cr, 0, 0, SYMBOL_SZ / 2, 0, DEG2RAD(360));
	cairo_stroke(cr);
}

static void
draw_tie(cairo_t *cr, double r, double g, double b)
{
	cairo_set_source_rgb(cr, r, g, b);
	cairo_rectangle(cr, -SYMBOL_SZ / 2, -SYMBOL_SZ / 2,
	    SYMBOL_SZ, SYMBOL_SZ);
	cairo_fill(cr);

	cairo_set_source_rgb(cr, BLACK_RGB);
	cairo_rectangle(cr, -SYMBOL_SZ / 2, -SYMBOL_SZ / 2,
	    SYMBOL_SZ, SYMBOL_SZ);
	cairo_stroke(cr);
}

static void
draw_diode(cairo_t *cr, double r, double g, double b)
{
	cairo_set_source_rgb(cr, r, g, b);
	cairo_move_to(cr, -SYMBOL_SZ / 2, SYMBOL_SZ / 2);
	cairo_rel_line_to(cr, SYMBOL_SZ, 0);
	cairo_rel_line_to(cr, -SYMBOL_SZ / 2, -SYMBOL_SZ);
	cairo_close_path(cr);
	cairo_fill(cr);

	cairo_set_source_rgb(cr, BLACK_RGB);
	cairo_move_to(cr, -SYMBOL_SZ / 2, -SYMBOL_SZ / 2);
	cairo_rel_line_to(cr, SYMBOL_SZ, 0);
	cairo_move_to(cr, -SYMBOL_SZ / 2, SYMBOL_SZ / 2);
	cairo_rel_line_to(cr, SYMBOL_SZ, 0);
	cairo_rel_line_to(cr, -SYMBOL_SZ / 2, -SYMBOL_SZ);
	cairo_close_path(cr);
	cairo_stroke(cr);
}

static void
draw_comp(elec_comp_t *comp, const draw_info_t *in_di, int depth)
{
	size_t n_conns;
	draw_info_t di = *in_di;
	double rot_step;
	int idx = crc64_rand() & (COLOR_TABLE_SZ - 1);
	const double *color = color_table[idx];
	const elec_comp_info_t *info = libelec_comp2info(comp);

	cairo_save(di.cr);

	/* rotate the render so our upstream connection is on the bottom */
	n_conns = libelec_comp_get_num_conns(comp);
	rot_step = DEG2RAD(360) / n_conns;

	for (size_t i = 0; i < n_conns; i++) {
		if (libelec_comp_get_conn(comp, i) == di.upstream) {
			double angle = i * rot_step;
			double rot_incr = DEG2RAD(180) - angle +
			    (crc64_rand_fract() - 0.5) *
			    MIN(rot_step, DEG2RAD(20));

			di.rot += rot_incr;
			cairo_rotate(di.cr, rot_incr);
			break;
		}
	}

	for (size_t i = 0; i < n_conns && !info->autogen && depth < 5; i++) {
		enum { LINE_LEN = 400 };
		elec_comp_t *subcomp = libelec_comp_get_conn(comp, i);
		draw_info_t ndi = di;

		if (subcomp == ndi.upstream)
			continue;

		cairo_save(di.cr);

		cairo_rotate(di.cr, rot_step * i);

		cairo_set_source_rgb(di.cr, color[0], color[1], color[2]);
		cairo_move_to(di.cr, 0, 0);
		cairo_line_to(di.cr, 0, LINE_LEN);
		cairo_stroke(di.cr);

		cairo_translate(di.cr, 0, LINE_LEN);

		ndi.rot += rot_step * i;
		ndi.upstream = comp;
		draw_comp(subcomp, &ndi, depth + 1);

		cairo_restore(di.cr);
	}

	switch (info->type) {
	case ELEC_BUS:
		draw_bus(di.cr, color[0], color[1], color[2]);
		break;
	case ELEC_CB:
		draw_cb(di.cr, color[0], color[1], color[2]);
		break;
	case ELEC_TIE:
		draw_tie(di.cr, color[0], color[1], color[2]);
		break;
	case ELEC_DIODE:
		draw_diode(di.cr, color[0], color[1], color[2]);
		break;
	default:
		break;
	}
	draw_comp_name(comp, &di);

	cairo_restore(di.cr);
}

static void
dump_img(void)
{
	char filename[64], busname[64];
	srch_info_t srch = { .name = busname, .comp = NULL };
	draw_info_t di = { .cr = NULL };
	elec_comp_t *bus;
	cairo_surface_t *surf;

	if (scanf("%63s %63s", filename, busname) != 2)
		return;

	libelec_walk_comps(sys, find_comp, &srch);
	bus = srch.comp;
	if (bus == NULL) {
		logMsg("Component %s not found", busname);
		return;
	}
	if (libelec_comp2info(bus)->type != ELEC_BUS) {
		logMsg("Component %s isn't a bus", busname);
		return;
	}

	surf = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 4096, 4096);
	di.cr = cairo_create(surf);

	cairo_set_source_rgb(di.cr, 1, 1, 1);
	cairo_paint(di.cr);
	cairo_translate(di.cr, 2048, 2048);
	cairo_select_font_face(di.cr, "monospace", CAIRO_FONT_SLANT_NORMAL,
	    CAIRO_FONT_WEIGHT_NORMAL);

	draw_comp(bus, &di, 0);

	cairo_surface_write_to_png(surf, filename);
	cairo_destroy(di.cr);
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

	crc64_init();
	crc64_srand(0);
	log_init(debug_print, "test");

	avl_create(&load_infos, load_info_compar, sizeof (load_info_t),
	    offsetof(load_info_t, node));

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

	libelec_destroy(sys);
	libelec_parsed_info_free(infos, num_infos);

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
