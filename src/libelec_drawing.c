/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */

#include <acfutils/cairo_utils.h>
#include <acfutils/perf.h>

#include "libelec_drawing.h"
#include "libelec_types_impl.h"

#define	PX(pos)			(pos_scale * (pos))
#define	LINE_HEIGHT		1
#define	COMP_INFO_BG_RGB	0.8, 0.8, 0.8
#define	MAX_NAME_LEN		128

enum {
    TEXT_ALIGN_LEFT,
    TEXT_ALIGN_CENTER,
    TEXT_ALIGN_RIGHT
};

static void show_text_aligned(cairo_t *cr, double x, double y, unsigned align,
    const char *format, ...) PRINTF_ATTR(5);

static void
make_comp_name(const char *in_name, char out_name[MAX_NAME_LEN])
{
	unsigned n;

	ASSERT(in_name);
	ASSERT(out_name);

	strlcpy(out_name, in_name, MAX_NAME_LEN);
	for (unsigned i = 0, n = strlen(out_name); i < n; i++) {
		if (out_name[i] == '_')
			out_name[i] = ' ';
	}
	if (strncmp(out_name, "CB ", 3) == 0)
		memmove(out_name, &out_name[3], strlen(out_name) - 3 + 1);
	n = strlen(out_name);
	if (n > 4 && strcmp(&out_name[n - 4], " O P") == 0)
		out_name[n - 2] = '/';
}

static void
get_srcs(const elec_comp_t *comp, elec_comp_t *srcs[ELEC_MAX_SRCS])
{
	ASSERT(comp != NULL);
	ASSERT(srcs != NULL);

	mutex_enter(&((elec_comp_t *)comp)->rw_ro_lock);
	memcpy(srcs, comp->srcs_ext, sizeof (*srcs) * ELEC_MAX_SRCS);
	mutex_exit(&((elec_comp_t *)comp)->rw_ro_lock);
}

static unsigned
count_srcs(elec_comp_t *srcs[ELEC_MAX_SRCS])
{
	ASSERT(srcs != NULL);

	for (unsigned n_srcs = 0; n_srcs < ELEC_MAX_SRCS; n_srcs++) {
		if (srcs[n_srcs] == NULL)
			return (n_srcs);
	}
	return (ELEC_MAX_SRCS);
}

static void
show_text_aligned(cairo_t *cr, double x, double y, unsigned align,
    const char *format, ...)
{
	cairo_text_extents_t te;
	char *str;
	va_list ap;

	ASSERT(cr != NULL);
	ASSERT(format != NULL);
	va_start(ap, format);
	str = vsprintf_alloc(format, ap);
	va_end(ap);

	cairo_text_extents(cr, str, &te);
	switch (align) {
	case TEXT_ALIGN_LEFT:
		cairo_move_to(cr, x - te.x_bearing,
		    y - te.height / 2 - te.y_bearing);
		break;
	case TEXT_ALIGN_CENTER:
		cairo_move_to(cr, x - te.width / 2 - te.x_bearing,
		    y - te.height / 2 - te.y_bearing);
		break;
	case TEXT_ALIGN_RIGHT:
		cairo_move_to(cr, x - te.width - te.x_bearing,
		    y - te.height / 2 - te.y_bearing);
		break;
	}
	cairo_show_text(cr, str);

	free(str);
}

static vect2_t
tie_node_pos(const elec_comp_t *tie, unsigned i)
{
	vect2_t pos;

	ASSERT(tie != NULL);
	ASSERT3U(tie->info->type, ==, ELEC_TIE);
	ASSERT3U(i, <, tie->n_links);
	ASSERT(tie->n_links != 0);

	pos = tie->info->gui.pos;
	if (tie->n_links == 2) {
		if (i == 0)
			return (VECT2(pos.x - 1, pos.y));
		else
			return (VECT2(pos.x + 1, pos.y));
	} else if (tie->n_links == 3) {
		vect2_t off;
		switch (i) {
		case 0:
			off = VECT2(0, 1);
			break;
		case 1:
			off = VECT2(-1, -1);
			break;
		case 2:
			off = VECT2(1, -1);
			break;
		}
		off = vect2_rot(off, tie->info->gui.rot);
		return (vect2_add(pos, VECT2(off.x, -off.y)));
	} else {
		vect2_t off = vect2_rot(VECT2(0, 1), tie->info->gui.rot);

		off = vect2_rot(off, i * (360.0 / tie->n_links));
		return (vect2_add(pos, VECT2(off.x, -off.y)));
	}
}

static vect2_t
pick_closer(vect2_t ref, vect2_t a, vect2_t b)
{
	double la = vect2_abs(vect2_sub(a, ref));
	double lb = vect2_abs(vect2_sub(b, ref));
	return (la <= lb ? a : b);
}

static bool
elec_comp_get_nearest_pos(const elec_comp_t *comp, vect2_t *comp_pos,
    vect2_t *bus_pos, double bus_sz, bool *align_vert)
{
	vect2_t pos;

	ASSERT(comp != NULL);
	ASSERT(comp_pos != NULL);
	ASSERT(bus_pos != NULL);
	ASSERT(align_vert != NULL);

	pos = comp->info->gui.pos;
	if (IS_NULL_VECT(comp->info->gui.pos))
		return (false);

	*align_vert = (comp->info->type == ELEC_TRU ||
	    comp->info->type == ELEC_INV || comp->info->type == ELEC_BATT);

	switch (comp->info->type) {
	case ELEC_BATT:
		*comp_pos = VECT2(pos.x, pos.y - 0.2);
		break;
	case ELEC_CB:
		*comp_pos = pick_closer(*bus_pos, vect2_add(pos, VECT2(-1, 0)),
		    vect2_add(pos, VECT2(1, 0)));
		break;
	case ELEC_SHUNT:
		*comp_pos = pick_closer(*bus_pos,
		    vect2_add(pos, VECT2(-2.5, 0)),
		    vect2_add(pos, VECT2(2.5, 0)));
		break;
	case ELEC_TIE: {
		vect2_t p = VECT2(1e9, 1e9);

		for (unsigned i = 0; i < comp->n_links; i++)
			p = pick_closer(*bus_pos, p, tie_node_pos(comp, i));
		*comp_pos = p;
		break;
	}
	default:
		*comp_pos = comp->info->gui.pos;
		break;
	}
	*bus_pos = VECT2(bus_pos->x, clamp(comp_pos->y,
	    bus_pos->y - bus_sz, bus_pos->y + bus_sz));

	return (true);
}

static void
draw_src_path(cairo_t *cr, cairo_path_t *path, const elec_comp_t *comp)
{
	cairo_pattern_t *pat;
	vect3_t color;
	elec_comp_t *srcs[ELEC_MAX_SRCS];
	unsigned n_srcs;

	ASSERT(cr != NULL);
	ASSERT(path != NULL);
	ASSERT(comp != NULL);

	get_srcs(comp, srcs);
	n_srcs = count_srcs(srcs);

	switch (n_srcs) {
	case 0:
		break;
	case 1:
		color = srcs[0]->info->gui.color;
		cairo_append_path(cr, path);
		cairo_set_source_rgb(cr, color.x, color.y, color.z);
		cairo_stroke(cr);
		cairo_set_source_rgb(cr, 0, 0, 0);
		break;
	default:
		pat = cairo_pattern_create_linear(0, 0, n_srcs * 8, n_srcs * 8);
		cairo_pattern_set_extend(pat, CAIRO_EXTEND_REPEAT);
		for (unsigned i = 0; i < n_srcs; i++) {
			vect3_t color = srcs[i]->info->gui.color;
			double off1 = i / (double)n_srcs;
			double off2 = (i + 1) / (double)n_srcs;
			cairo_pattern_add_color_stop_rgb(pat, off1,
			    color.x, color.y, color.z);
			cairo_pattern_add_color_stop_rgb(pat, off2,
			    color.x, color.y, color.z);
		}
		cairo_append_path(cr, path);
		cairo_set_source(cr, pat);
		cairo_stroke(cr);
		cairo_set_source_rgb(cr, 0, 0, 0);
		cairo_pattern_destroy(pat);
		break;
	}
	cairo_path_destroy(path);
}

static void
draw_bus_conns(cairo_t *cr, double pos_scale, const elec_comp_t *bus)
{
	ASSERT(cr != NULL);
	ASSERT(bus != NULL);
	ASSERT3U(bus->info->type, ==, ELEC_BUS);

	if (IS_NULL_VECT(bus->info->gui.pos))
		return;

	cairo_new_path(cr);

	for (unsigned i = 0; i < bus->n_links; i++) {
		vect2_t bus_pos = bus->info->gui.pos;
		vect2_t comp_pos;
		const elec_comp_t *comp = bus->links[i].comp;
		bool align_vert;
		cairo_path_t *path;

		if (!elec_comp_get_nearest_pos(comp, &comp_pos, &bus_pos,
		    bus->info->gui.sz, &align_vert)) {
			continue;
		}
		/*
		 * The connection line itself.
		 */
		if (align_vert) {
			cairo_move_to(cr, PX(bus_pos.x), PX(bus_pos.y));
			cairo_line_to(cr, PX(comp_pos.x), PX(bus_pos.y));
			cairo_line_to(cr, PX(comp_pos.x), PX(comp_pos.y));
		} else {
			cairo_move_to(cr, PX(bus_pos.x), PX(bus_pos.y));
			cairo_line_to(cr, PX(AVG(bus_pos.x, comp_pos.x)),
			    PX(bus_pos.y));
			cairo_line_to(cr, PX(AVG(bus_pos.x, comp_pos.x)),
			    PX(comp_pos.y));
			cairo_line_to(cr, PX(comp_pos.x), PX(comp_pos.y));
		}
		path = cairo_copy_path(cr);
		cairo_set_line_width(cr, 3);
		cairo_stroke(cr);

		cairo_set_line_width(cr, 2);
		draw_src_path(cr, path, bus);
		/*
		 * The black dimple on the bus showing the connection
		 */
		if (!bus->info->gui.invis) {
			if (bus->info->gui.sz != 0 && !bus->info->gui.virt) {
				cairo_arc(cr, PX(bus_pos.x), PX(bus_pos.y),
				    PX(0.4), 0, DEG2RAD(360));
			} else if (bus->n_links > 2) {
				cairo_arc(cr, PX(bus_pos.x), PX(bus_pos.y),
				    PX(0.25), 0, DEG2RAD(360));
			}
			cairo_fill(cr);
		}
	}
}

static void
draw_gen(cairo_t *cr, double pos_scale, const elec_comp_info_t *info)
{
	vect2_t pos;
	vect3_t color;
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;
	color = info->gui.color;

	cairo_new_path(cr);

	cairo_set_source_rgb(cr, color.x, color.y, color.z);
	cairo_arc(cr, PX(pos.x), PX(pos.y), PX(1.2), 0, DEG2RAD(360));
	cairo_fill(cr);

	cairo_set_line_width(cr, 2);
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_arc(cr, PX(pos.x), PX(pos.y), PX(1.2), 0, DEG2RAD(360));
	if (info->gen.freq != 0) {
		cairo_move_to(cr, PX(pos.x - 0.9), PX(pos.y));
		cairo_rel_curve_to(cr, PX(0.2), PX(-0.7),
		    PX(0.7), PX(-0.7), PX(0.9), 0);
		cairo_rel_curve_to(cr, PX(0.2), PX(0.7),
		    PX(0.7), PX(0.7), PX(0.9), 0);
	} else {
		cairo_move_to(cr, PX(pos.x - 0.8), PX(pos.y - 0.2));
		cairo_rel_line_to(cr, PX(1.6), 0);
		cairo_move_to(cr, PX(pos.x - 0.8), PX(pos.y + 0.2));
		cairo_rel_line_to(cr, PX(1.6), 0);
	}
	cairo_stroke(cr);

	make_comp_name(info->name, name);
	show_text_aligned(cr, PX(pos.x), PX(pos.y + 2), TEXT_ALIGN_CENTER,
	    "%s", name);
}

static void
draw_bus(cairo_t *cr, double pos_scale, const elec_comp_t *bus)
{
	const elec_comp_info_t *info;
	vect2_t pos;

	ASSERT(cr != NULL);
	ASSERT(bus != NULL);
	info = bus->info;
	pos = info->gui.pos;

	if (info->gui.invis)
		return;

	cairo_new_path(cr);

	if (info->gui.sz != 0) {
		cairo_path_t *path;

		cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
		if (!info->gui.virt)
			cairo_set_line_width(cr, 10);
		else
			cairo_set_line_width(cr, 3);
		cairo_move_to(cr, PX(pos.x), PX(pos.y - info->gui.sz));
		cairo_rel_line_to(cr, 0, PX(2 * info->gui.sz));
		path = cairo_copy_path(cr);
		cairo_stroke(cr);
		cairo_set_line_width(cr, !info->gui.virt ? 4 : 2);
		draw_src_path(cr, path, bus);
		cairo_set_line_width(cr, 2);
		cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT);
	}

	if (info->gui.sz != 0 && !info->gui.virt) {
		char name[MAX_NAME_LEN];
		make_comp_name(info->name, name);
		show_text_aligned(cr, PX(pos.x), PX(pos.y - info->gui.sz - 1),
		    TEXT_ALIGN_CENTER, "%s", name);
	}
}

static void
draw_cb_icon(cairo_t *cr, double pos_scale, double font_sz, vect2_t pos,
    bool fuse, bool set, bool triphase, const char *comp_name,
    vect3_t bg_color, const elec_comp_t *comp)
{
	double text_y_off;
	cairo_path_t *path;
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(comp_name != NULL);
	ASSERT(comp != NULL);

	make_comp_name(comp_name, name);
	cairo_new_path(cr);

	if (!fuse) {
		/* Yoke on the top */
		cairo_move_to(cr, PX(pos.x), PX(pos.y - (set ? 0.5 : 1)));
		cairo_set_line_width(cr, 2);
		cairo_rel_line_to(cr, 0, PX(-0.5));
		cairo_rel_move_to(cr, PX(-0.4), 0);
		cairo_rel_line_to(cr, PX(0.8), 0);
		cairo_stroke(cr);
		/* The arch of the breaker */
		cairo_set_line_width(cr, 3);
		cairo_arc(cr, PX(pos.x), PX(pos.y + (set ? 0.5 : 0)), PX(1.1),
		    DEG2RAD(215), DEG2RAD(-35));
	} else {
		cairo_set_line_width(cr, 3);
		cairo_move_to(cr, PX(pos.x - 1), PX(pos.y));
		cairo_rel_curve_to(cr, PX(0.2), PX(0.8), PX(0.8), PX(0.8),
		    PX(1), 0);
		cairo_rel_curve_to(cr, PX(0.2), PX(-0.8), PX(0.8), PX(-0.8),
		    PX(1), 0);
	}
	path = cairo_copy_path(cr);
	cairo_stroke(cr);

	cairo_set_line_width(cr, 2);
	draw_src_path(cr, path, comp);

	if (fuse && !set) {
		/* If the fuse is broken, remove the middle bit */
		cairo_set_source_rgb(cr, bg_color.x, bg_color.y, bg_color.z);
		cairo_arc(cr, PX(pos.x), PX(pos.y), PX(0.3), 0, DEG2RAD(360));
		cairo_fill(cr);
	}
	/* The two dimples on either side */
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_arc(cr, PX(pos.x - 1), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_new_sub_path(cr);
	cairo_arc(cr, PX(pos.x + 1), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_fill(cr);
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_arc(cr, PX(pos.x - 1), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_new_sub_path(cr);
	cairo_arc(cr, PX(pos.x + 1), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_stroke(cr);

	if (triphase) {
		cairo_set_font_size(cr, round(0.75 * font_sz));
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_CENTER,
		    "3P");
		cairo_set_font_size(cr, font_sz);
	}

	text_y_off = (fuse ? 1.5 : 0.8);
	show_text_aligned(cr, PX(pos.x), PX(pos.y + text_y_off),
	    TEXT_ALIGN_CENTER, "%s", name);
}

static void
draw_cb(cairo_t *cr, double pos_scale, const elec_comp_t *cb, double font_sz,
    vect3_t bg_color)
{
	ASSERT(cr != NULL);
	ASSERT(cb != NULL);
	draw_cb_icon(cr, pos_scale, font_sz, cb->info->gui.pos,
	    cb->info->cb.fuse, !cb->ro.failed && cb->scb.cur_set,
	    cb->info->cb.triphase, cb->info->name, bg_color, cb);
}

static void
draw_shunt(cairo_t *cr, double pos_scale, const elec_comp_t *shunt)
{
	vect2_t pos;
	cairo_path_t *path;
	const elec_comp_info_t *info;
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(shunt != NULL);
	info = shunt->info;
	pos = info->gui.pos;

	cairo_new_path(cr);
	cairo_set_line_width(cr, 3);
	cairo_move_to(cr, PX(pos.x - 2.5), PX(pos.y));
	cairo_rel_line_to(cr, PX(1), PX(0));
	for (int i = 0; i < 3; i++) {
		cairo_rel_line_to(cr, PX(0.25), PX(-0.7));
		cairo_rel_line_to(cr, PX(0.5), PX(1.4));
		cairo_rel_line_to(cr, PX(0.25), PX(-0.7));
	}
	cairo_rel_line_to(cr, PX(1), PX(0));
	path = cairo_copy_path(cr);
	cairo_stroke(cr);

	cairo_set_line_width(cr, 2);
	draw_src_path(cr, path, shunt);

	make_comp_name(info->name, name);
	show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.7),
	    TEXT_ALIGN_CENTER, "%s", name);
}

static void
draw_tru_inv(cairo_t *cr, double pos_scale, const elec_comp_info_t *info)
{
	vect2_t pos;
	vect3_t color;
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;
	color = info->gui.color;

	cairo_new_path(cr);

	/* Background fill */
	cairo_set_source_rgb(cr, color.x, color.y, color.z);
	cairo_rectangle(cr, PX(pos.x - 1.5), PX(pos.y - 1.5), PX(3), PX(3));
	cairo_fill(cr);

	/* Line art */
	cairo_set_source_rgb(cr, 0, 0, 0);
	/* Surrounding box */
	cairo_rectangle(cr, PX(pos.x - 1.5), PX(pos.y - 1.5), PX(3), PX(3));

	/* Cross-line through the box */
	cairo_move_to(cr, PX(pos.x - 1.5), PX(pos.y + 1.5));
	cairo_rel_line_to(cr, PX(3), PX(-3));

	/* Wavy line showing AC side */
	if (info->type == ELEC_TRU)
		cairo_move_to(cr, PX(pos.x - 0.8), PX(pos.y - 1));
	else
		cairo_move_to(cr, PX(pos.x + 0.8), PX(pos.y + 1));
	cairo_rel_move_to(cr, PX(-0.5), 0);
	cairo_rel_curve_to(cr, PX(0.1), PX(-0.4),
	    PX(0.4), PX(-0.4), PX(0.5), 0);
	cairo_rel_curve_to(cr, PX(0.1), PX(0.4),
	    PX(0.4), PX(0.4), PX(0.5), 0);

	/* Flat line showing DC side */
	if (info->type == ELEC_TRU)
		cairo_move_to(cr, PX(pos.x + 0.8), PX(pos.y + 1));
	else
		cairo_move_to(cr, PX(pos.x - 0.8), PX(pos.y - 1));
	cairo_rel_move_to(cr, PX(-0.5), 0);
	cairo_rel_line_to(cr, PX(1), 0);

	cairo_stroke(cr);

	make_comp_name(info->name, name);
	show_text_aligned(cr, PX(pos.x - 2), PX(pos.y),
	    TEXT_ALIGN_RIGHT, "%s", name);
}

static void
draw_xfrmr(cairo_t *cr, double pos_scale, const elec_comp_info_t *info)
{
	vect2_t pos;
	vect3_t color;
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;
	color = info->gui.color;

	cairo_new_path(cr);

	/* Background fill */
	cairo_set_source_rgb(cr, color.x, color.y, color.z);
	cairo_rectangle(cr, PX(pos.x - 1.5), PX(pos.y - 1.5), PX(3), PX(3));
	cairo_fill(cr);

	/* Line art */
	cairo_set_source_rgb(cr, 0, 0, 0);
	/* Surrounding box */
	cairo_rectangle(cr, PX(pos.x - 1.5), PX(pos.y - 1.5), PX(3), PX(3));

	/* Cross-line through the box */
	cairo_move_to(cr, PX(pos.x - 1.5), PX(pos.y + 1.5));
	cairo_rel_line_to(cr, PX(3), PX(-3));

	/* Wavy line showing input & output sidesside */
	cairo_move_to(cr, PX(pos.x - 0.8), PX(pos.y - 1));
	cairo_rel_move_to(cr, PX(-0.5), 0);
	cairo_rel_curve_to(cr, PX(0.1), PX(-0.4),
	    PX(0.4), PX(-0.4), PX(0.5), 0);
	cairo_rel_curve_to(cr, PX(0.1), PX(0.4),
	    PX(0.4), PX(0.4), PX(0.5), 0);

	cairo_move_to(cr, PX(pos.x + 0.8), PX(pos.y + 1));
	cairo_rel_move_to(cr, PX(-0.5), 0);
	cairo_rel_curve_to(cr, PX(0.1), PX(-0.4),
	    PX(0.4), PX(-0.4), PX(0.5), 0);
	cairo_rel_curve_to(cr, PX(0.1), PX(0.4),
	    PX(0.4), PX(0.4), PX(0.5), 0);

	cairo_stroke(cr);

	make_comp_name(info->name, name);
	show_text_aligned(cr, PX(pos.x - 2), PX(pos.y),
	    TEXT_ALIGN_RIGHT, "%s", name);
}

static void
draw_node(cairo_t *cr, double pos_scale, vect2_t pos)
{
	ASSERT(cr != NULL);

	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_arc(cr, PX(pos.x), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_fill(cr);

	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_arc(cr, PX(pos.x), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_stroke(cr);
}

static void
draw_tie(cairo_t *cr, double pos_scale, const elec_comp_t *tie)
{
	vect2_t endpt[2] = { NULL_VECT2, NULL_VECT2 };
	vect2_t pos;
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(tie != NULL);
	pos = tie->info->gui.pos;

	cairo_new_path(cr);

	cairo_set_line_width(cr, 4);
	for (unsigned i = 0; i < tie->n_links; i++) {
		elec_comp_t *remote_bus = tie->links[i].comp;
		vect2_t conn = VECT2(1e9, 1e9);

		if (!tie->tie.cur_state[i])
			continue;

		for (unsigned j = 0; j < tie->n_links; j++) {
			conn = pick_closer(remote_bus->info->gui.pos, conn,
			    tie_node_pos(tie, j));
		}
		if (IS_NULL_VECT(endpt[0])) {
			endpt[0] = conn;
		} else {
			endpt[1] = conn;
			break;
		}
	}
	if (!IS_NULL_VECT(endpt[0]) && !IS_NULL_VECT(endpt[1])) {
		cairo_path_t *path;

		cairo_move_to(cr, PX(endpt[0].x), PX(endpt[0].y));
		cairo_line_to(cr, PX(endpt[1].x), PX(endpt[1].y));
		path = cairo_copy_path(cr);
		cairo_stroke(cr);
		cairo_set_line_width(cr, 2);
		draw_src_path(cr, path, tie);
	} else {
		/* Nothing tied, show the unconnected state */
		if (tie->n_links == 2) {
			cairo_move_to(cr, PX(pos.x - 1), PX(pos.y - 1));
			cairo_rel_line_to(cr, PX(2), 0);
		} else {
			vect2_t node_pos = tie_node_pos(tie, 0);
			vect2_t line = vect2_rot(VECT2(0, -2),
			    tie->info->gui.rot);
			cairo_move_to(cr, PX(node_pos.x), PX(node_pos.y));
			cairo_rel_line_to(cr, PX(line.x), PX(-line.y));
		}
		cairo_stroke(cr);
	}
	cairo_set_line_width(cr, 2);
	for (unsigned i = 0; i < tie->n_links; i++)
		draw_node(cr, pos_scale, tie_node_pos(tie, i));

	make_comp_name(tie->info->name, name);
	if (tie->n_links == 3) {
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.8),
		    TEXT_ALIGN_CENTER, "%s", name);
	} else {
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.5),
		    TEXT_ALIGN_CENTER, "%s", name);
	}
}

static void
draw_diode(cairo_t *cr, double pos_scale, const elec_comp_t *diode,
    bool draw_line)
{
	vect2_t pos;
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(diode != NULL);
	pos = diode->info->gui.pos;

	cairo_save(cr);
	cairo_translate(cr, PX(pos.x), PX(pos.y));
	cairo_rotate(cr, DEG2RAD(diode->info->gui.rot));
	cairo_move_to(cr, PX(0.4), 0);
	cairo_rel_line_to(cr, PX(-1.3), PX(-0.8));
	cairo_rel_line_to(cr, 0, PX(1.6));
	cairo_fill(cr);
	cairo_set_line_width(cr, 4);
	cairo_move_to(cr, PX(0.5), PX(-0.8));
	cairo_rel_line_to(cr, 0, PX(1.6));
	if (draw_line) {
		cairo_move_to(cr, PX(-2), 0);
		cairo_rel_line_to(cr, PX(4), 0);
	}
	cairo_stroke(cr);
	cairo_restore(cr);

	make_comp_name(diode->info->name, name);
	show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.5),
	    TEXT_ALIGN_CENTER, "%s", name);
}

static void
draw_load(cairo_t *cr, double pos_scale, double font_sz,
    const elec_comp_info_t *info)
{
	vect2_t pos;
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;

	cairo_new_path(cr);
	switch (info->gui.load_type) {
	case GUI_LOAD_GENERIC:
		cairo_set_source_rgb(cr, 1, 1, 1);
		cairo_rectangle(cr, PX(pos.x - 1), PX(pos.y - 1), PX(2), PX(2));
		cairo_fill(cr);
		cairo_set_source_rgb(cr, 0, 0, 0);
		cairo_rectangle(cr, PX(pos.x - 1), PX(pos.y - 1), PX(2), PX(2));
		cairo_stroke(cr);
		break;
	case GUI_LOAD_MOTOR:
		cairo_set_source_rgb(cr, 1, 1, 1);
		cairo_arc(cr, PX(pos.x), PX(pos.y), PX(1), 0, DEG2RAD(360));
		cairo_fill(cr);
		cairo_set_source_rgb(cr, 0, 0, 0);
		cairo_arc(cr, PX(pos.x), PX(pos.y), PX(1), 0, DEG2RAD(360));
		cairo_stroke(cr);
		cairo_set_font_size(cr, 2 * font_sz);
		show_text_aligned(cr, PX(pos.x), PX(pos.y),
		    TEXT_ALIGN_CENTER, "M");
		cairo_set_font_size(cr, font_sz);
		break;
	}

	make_comp_name(info->name, name);
	show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.7),
	    TEXT_ALIGN_CENTER, "%s", name);
}

static void
draw_batt(cairo_t *cr, double pos_scale, const elec_comp_info_t *info,
    bool draw_ground)
{
	vect2_t pos;
	vect3_t color;
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;
	color = info->gui.color;

	cairo_new_path(cr);

	cairo_set_source_rgb(cr, color.x, color.y, color.z);
	cairo_arc(cr, PX(pos.x), PX(pos.y), PX(1.2), 0, DEG2RAD(360));
	cairo_fill(cr);

	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_arc(cr, PX(pos.x), PX(pos.y), PX(1.2), 0, DEG2RAD(360));
	cairo_move_to(cr, PX(pos.x), PX(pos.y - 0.2));
	cairo_rel_line_to(cr, 0, PX(-1.0));
	cairo_stroke(cr);

	cairo_move_to(cr, PX(pos.x + 0.4), PX(pos.y - 0.6));
	cairo_rel_line_to(cr, PX(0.4), 0);
	cairo_move_to(cr, PX(pos.x + 0.6), PX(pos.y - 0.8));
	cairo_rel_line_to(cr, 0, PX(0.4));

	cairo_move_to(cr, PX(pos.x - 1), PX(pos.y - 0.2));
	cairo_rel_line_to(cr, PX(2), PX(0));
	cairo_move_to(cr, PX(pos.x - 0.6), PX(pos.y + 0.2));
	cairo_rel_line_to(cr, PX(1.2), PX(0));

	if (draw_ground) {
		cairo_move_to(cr, PX(pos.x), PX(pos.y + 0.2));
		cairo_rel_line_to(cr, 0, PX(2.3));
		cairo_move_to(cr, PX(pos.x - 1), PX(pos.y + 2.5));
		cairo_rel_line_to(cr, PX(2), PX(0));
		cairo_move_to(cr, PX(pos.x - 0.7), PX(pos.y + 2.9));
		cairo_rel_line_to(cr, PX(1.4), PX(0));
		cairo_move_to(cr, PX(pos.x - 0.4), PX(pos.y + 3.3));
		cairo_rel_line_to(cr, PX(0.8), PX(0));
	} else {
		cairo_move_to(cr, PX(pos.x), PX(pos.y + 0.2));
		cairo_rel_line_to(cr, 0, PX(1));
	}

	cairo_stroke(cr);

	make_comp_name(info->name, name);
	show_text_aligned(cr, PX(pos.x - 1.4), PX(pos.y),
	    TEXT_ALIGN_RIGHT, "%s", name);
}

static void
draw_label_box(cairo_t *cr, double pos_scale, double font_sz,
    const elec_comp_info_t *info)
{
	cairo_text_extents_t te;
	vect3_t color;
	vect2_t pos, sz;
	double dash[2] = { PX(1), PX(0.5) };
	char name[MAX_NAME_LEN];

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	ASSERT3U(info->type, ==, ELEC_LABEL_BOX);

	cairo_save(cr);

	cairo_set_font_size(cr, font_sz * info->label_box.font_scale);

	make_comp_name(info->name, name);
	cairo_text_extents(cr, name, &te);
	pos = info->label_box.pos;
	sz = info->label_box.sz;
	color = info->gui.color;

	cairo_set_dash(cr, dash, 2, 0);

	cairo_set_source_rgb(cr, color.x, color.y, color.z);
	cairo_rectangle(cr, PX(pos.x), PX(pos.y), PX(sz.x), PX(sz.y));
	cairo_stroke(cr);

	/*
	 * We add half te.height to the width of the surrounding box to
	 * add a bit of a border around the text.
	 */
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_set_dash(cr, NULL, 0, 0);
	cairo_rectangle(cr, PX(pos.x + sz.x / 2) - te.width / 2 - te.height / 2,
	    PX(pos.y) - te.height * 0.75, te.width + te.height,
	    te.height * 1.5);
	cairo_stroke(cr);
	/*
	 * White background
	 */
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_rectangle(cr, PX(pos.x + sz.x / 2) - te.width / 2 - te.height / 2,
	    PX(pos.y) - te.height * 0.75, te.width + te.height,
	    te.height * 1.5);
	cairo_fill(cr);

	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_move_to(cr, PX(pos.x + sz.x / 2) - te.width / 2,
	    PX(pos.y) - te.height / 2 - te.y_bearing);
	cairo_show_text(cr, name);

	cairo_restore(cr);
}

/**
 * Draws the network base image into a `cairo_t` instance. You should
 * use this before drawing any overlays (such as an open component info
 * screen drawn using libelec_draw_comp_info()).
 * @param sys The network to be drawn.
 * @param cr The `cairo_t` instance into which the drawing will be performed.
 * @param pos_scale A scaling multiplier applied to all `GUI_POS` stanzas
 *	in the network layout configuration file. This lets you use abstract
 *	size units in the config file and then scale them up as you see fit.
 * @param font_sz Default font size to be used for all drawing. These
 *	functions never reset the font face, so whatever you set using
 *	cairo_set_font_face() will be used. The font size is only needed
 *	in order to draw suffixes and smaller font information correctly
 *	scaled to the default text size (which is used for headers, etc.)
 */
void
libelec_draw_layout(const elec_sys_t *sys, cairo_t *cr, double pos_scale,
    double font_sz)
{
	ASSERT(sys != NULL);
	ASSERT(cr != NULL);

	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_set_font_size(cr, font_sz);
	cairo_set_line_width(cr, 2);

	/* First draw bus connections, so all components sit on top of them */
	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		const elec_comp_info_t *info = comp->info;

		ASSERT(info != NULL);
		if (info->type == ELEC_BUS)
			draw_bus_conns(cr, pos_scale, comp);
	}

	for (elec_comp_t *comp = list_head(&sys->comps); comp != NULL;
	    comp = list_next(&sys->comps, comp)) {
		const elec_comp_info_t *info = comp->info;

		ASSERT(info != NULL);
		if (IS_NULL_VECT(info->gui.pos) || info->gui.invis)
			continue;

		switch (info->type) {
		case ELEC_BUS:
			draw_bus(cr, pos_scale, comp);
			break;
		case ELEC_GEN:
			draw_gen(cr, pos_scale, info);
			break;
		case ELEC_CB:
			draw_cb(cr, pos_scale, comp, font_sz, VECT3(1, 1, 1));
			break;
		case ELEC_SHUNT:
			draw_shunt(cr, pos_scale, comp);
			break;
		case ELEC_TRU:
		case ELEC_INV:
			draw_tru_inv(cr, pos_scale, info);
			break;
		case ELEC_XFRMR:
			draw_xfrmr(cr, pos_scale, info);
			break;
		case ELEC_TIE:
			draw_tie(cr, pos_scale, comp);
			break;
		case ELEC_DIODE:
			draw_diode(cr, pos_scale, comp, false);
			break;
		case ELEC_LOAD:
			draw_load(cr, pos_scale, font_sz, info);
			break;
		case ELEC_BATT:
			draw_batt(cr, pos_scale, info, true);
			break;
		case ELEC_LABEL_BOX:
			VERIFY_FAIL();
			break;
		}
	}
	for (size_t i = 0; i < sys->num_infos; i++) {
		const elec_comp_info_t *info = &sys->comp_infos[i];

		if (info->type == ELEC_LABEL_BOX)
			draw_label_box(cr, pos_scale, font_sz, info);
	}
}

static void
draw_comp_bg(cairo_t *cr, double pos_scale, vect2_t pos, vect2_t sz)
{
	cairo_path_t *path;

	ASSERT(cr != NULL);

	cairo_utils_rounded_rect(cr, PX(pos.x - sz.x / 2),
	    PX(pos.y - sz.y / 2), PX(sz.x), PX(sz.y), PX(0.5));
	path = cairo_copy_path(cr);
	cairo_set_source_rgb(cr, COMP_INFO_BG_RGB);
	cairo_fill(cr);

	cairo_append_path(cr, path);
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_stroke(cr);

	cairo_path_destroy(path);
}

static void
draw_in_out_suffixes(cairo_t *cr, double pos_scale, vect2_t pos,
    unsigned num_in, unsigned num_out)
{
	ASSERT(cr != NULL);

	for (unsigned i = 0; i < num_in + num_out; i++) {
		show_text_aligned(cr, PX(pos.x + 0.5),
		    PX(pos.y + (i + 0.25) * LINE_HEIGHT),
		    TEXT_ALIGN_LEFT, i < num_in ? "IN" : "OUT");
	}
}

static void
draw_comp_info(const elec_comp_t *comp, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	bool ac;
	double U_in, I_in, W_in, U_out, I_out, W_out, f;
	elec_comp_t *srcs[ELEC_MAX_SRCS];
	unsigned n_srcs;

	ASSERT(comp != NULL);
	ASSERT(cr != NULL);

	U_in = libelec_comp_get_in_volts(comp);
	U_out = libelec_comp_get_out_volts(comp);
	ac = libelec_comp_is_AC(comp);
	if (libelec_comp2info(comp)->type == ELEC_INV)
		f = libelec_comp_get_out_freq(comp);
	else
		f = (ac ? libelec_comp_get_in_freq(comp) : 0);
	I_in = libelec_comp_get_in_amps(comp);
	I_out = libelec_comp_get_out_amps(comp);
	W_in = libelec_comp_get_in_pwr(comp);
	W_out = libelec_comp_get_out_pwr(comp);
	get_srcs(comp, srcs);
	n_srcs = count_srcs(srcs);

	if (comp->info->type != ELEC_GEN) {
		char name[MAX_NAME_LEN];
		const char *powered_by;

		switch (n_srcs) {
		case 0:
			powered_by = "nothing";
			break;
		case 1:
			make_comp_name(srcs[0]->info->name, name);
			powered_by = name;
			break;
		default:
			powered_by = "(multiple)";
			break;
		}
		show_text_aligned(cr, PX(pos.x), PX(pos.y),
		    TEXT_ALIGN_LEFT, "Powered by: %s", powered_by);
		pos.y += LINE_HEIGHT;
	}

	switch (comp->info->type) {
	case ELEC_BATT:
	case ELEC_TRU:
	case ELEC_INV:
	case ELEC_XFRMR:
		cairo_set_font_size(cr, 0.75 * font_sz);
		if (comp->info->type == ELEC_INV)
			draw_in_out_suffixes(cr, pos_scale, pos, 3, 4);
		else if (comp->info->type == ELEC_TRU)
			draw_in_out_suffixes(cr, pos_scale, pos, 4, 3);
		else
			draw_in_out_suffixes(cr, pos_scale, pos, 3, 3);

		cairo_set_font_size(cr, font_sz);
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "U   : %.*fV", fixed_decimals(U_in, 4), U_in);
		pos.y += LINE_HEIGHT;
		if (comp->info->type != ELEC_INV && ac) {
			show_text_aligned(cr, PX(pos.x), PX(pos.y),
			    TEXT_ALIGN_LEFT, "f   : %.*fHz",
			    fixed_decimals(f, 4), f);
			pos.y += LINE_HEIGHT;
		}
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "I   : %.*fA", fixed_decimals(I_in, 4), I_in);
		pos.y += LINE_HEIGHT;
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "W   : %.*fW", fixed_decimals(W_in, 4), W_in);
		pos.y += LINE_HEIGHT;
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "U   : %.*fV", fixed_decimals(U_out, 4), U_out);
		pos.y += LINE_HEIGHT;
		if (comp->info->type == ELEC_INV) {
			show_text_aligned(cr, PX(pos.x), PX(pos.y),
			    TEXT_ALIGN_LEFT, "f   : %.*fHz",
			    fixed_decimals(f, 4), f);
			pos.y += LINE_HEIGHT;
		}
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "I   : %.*fA", fixed_decimals(I_out, 4), I_out);
		pos.y += LINE_HEIGHT;
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "W   : %.*fW", fixed_decimals(W_out, 4), W_out);
		pos.y += LINE_HEIGHT;
		break;
	case ELEC_GEN:
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "U: %.*fV", fixed_decimals(U_out, 4), U_out);
		pos.y += LINE_HEIGHT;
		if (ac) {
			show_text_aligned(cr, PX(pos.x), PX(pos.y),
			    TEXT_ALIGN_LEFT, "f: %.*fHz",
			    fixed_decimals(f, 4), f);
			pos.y += LINE_HEIGHT;
		}
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "I: %.*fA", fixed_decimals(I_out, 4), I_out);
		pos.y += LINE_HEIGHT;
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "W: %.*fW", fixed_decimals(W_out, 4), W_out);
		break;
	case ELEC_BUS:
	case ELEC_LOAD:
	case ELEC_CB:
	case ELEC_SHUNT:
	case ELEC_TIE:
	case ELEC_DIODE:
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "U: %.*fV", fixed_decimals(U_in, 4), U_in);
		pos.y += LINE_HEIGHT;
		if (ac) {
			show_text_aligned(cr, PX(pos.x), PX(pos.y),
			    TEXT_ALIGN_LEFT, "f: %.*fHz",
			    fixed_decimals(f, 4), f);
			pos.y += LINE_HEIGHT;
		}
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "I: %.*fA", fixed_decimals(I_in, 4), I_in);
		pos.y += LINE_HEIGHT;
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_LEFT,
		    "W: %.*fW", fixed_decimals(W_in, 4), W_in);
		break;
	case ELEC_LABEL_BOX:
		break;
	}
}

static void
draw_diode_info(const elec_comp_t *diode, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	const double TEXT_OFF_X = -6.5, TEXT_OFF_Y = 2.5;

	ASSERT(diode != NULL);
	ASSERT(diode->info != NULL);
	ASSERT3U(diode->info->type, ==, ELEC_DIODE);

	draw_comp_bg(cr, pos_scale, VECT2(pos.x, pos.y + 3), VECT2(14, 10));

	draw_diode(cr, pos_scale, diode, true);
	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(pos.y + TEXT_OFF_Y),
	    TEXT_ALIGN_LEFT, "Type: Diode");
	draw_comp_info(diode, cr, pos_scale, font_sz,
	    VECT2(pos.x + TEXT_OFF_X, pos.y + TEXT_OFF_Y + LINE_HEIGHT));
}

static void
draw_scb_info(const elec_comp_t *cb, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	const double TEXT_OFF_X = -6.5, TEXT_OFF_Y = 2.5;
	const char *type;
	double y, height, box_y_off;

	ASSERT(cb != NULL);
	ASSERT(cb->info != NULL);
	ASSERT(cb->info->type == ELEC_CB || cb->info->type == ELEC_SHUNT);

	height = (cb->info->type == ELEC_CB ? 14 : 12);
	box_y_off = (cb->info->type == ELEC_CB ? 5 : 4);
	draw_comp_bg(cr, pos_scale, VECT2(pos.x, pos.y + box_y_off),
	    VECT2(14, height));

	if (cb->info->type == ELEC_CB) {
		draw_cb(cr, pos_scale, cb, font_sz,
		    (vect3_t){COMP_INFO_BG_RGB});
	} else {
		draw_shunt(cr, pos_scale, cb);
	}

	y = pos.y + TEXT_OFF_Y;
	if (cb->info->type == ELEC_SHUNT)
		y += LINE_HEIGHT;
	if (cb->info->type == ELEC_CB) {
		type = (cb->info->cb.fuse ? "Fuse" : "Circuit Breaker");
	} else {
		type = "Shunt Resistor";
	}
	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y),
	    TEXT_ALIGN_LEFT, "Type: %s%s",
	    cb->info->cb.triphase ? "3-Phase " : "", type);
	y += LINE_HEIGHT;
	if (cb->info->type == ELEC_CB) {
		show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y),
		    TEXT_ALIGN_LEFT, "State: %s",
		    libelec_cb_get(cb) ? "Closed" : "Open");
		y += LINE_HEIGHT;
		show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y),
		    TEXT_ALIGN_LEFT, "Limit: %s%.*fA",
		    cb->info->cb.triphase ? "3 x " : "",
		    fixed_decimals(cb->info->cb.max_amps, 2),
		    cb->info->cb.max_amps);
		y += LINE_HEIGHT;
		show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y),
		    TEXT_ALIGN_LEFT, "Location: %s", cb->info->location);
		y += LINE_HEIGHT;
	}
	draw_comp_info(cb, cr, pos_scale, font_sz,
	    VECT2(pos.x + TEXT_OFF_X, y));
}

static void
draw_gen_info(const elec_comp_t *gen, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	const double TEXT_OFF_X = -6.5, TEXT_OFF_Y = 3;
	double y;

	ASSERT(gen != NULL);
	ASSERT(gen->info != NULL);
	ASSERT3U(gen->info->type, ==, ELEC_GEN);

	draw_comp_bg(cr, pos_scale, VECT2(pos.x, pos.y + 4), VECT2(14, 12));
	draw_gen(cr, pos_scale, gen->info);

	y = pos.y + TEXT_OFF_Y;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Type: %s Generator", gen->info->gen.freq != 0 ? "AC" : "DC");
	y += LINE_HEIGHT;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "RPM: %.0f", gen->gen.rpm);
	y += LINE_HEIGHT;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Efficiency: %.1f%%", gen->gen.eff * 100);
	y += LINE_HEIGHT;

	draw_comp_info(gen, cr, pos_scale, font_sz,
	    VECT2(pos.x + TEXT_OFF_X, y));
}

static const char *
tru_inv2str(const elec_comp_info_t *info)
{
	ASSERT(info != NULL);
	if (info->type == ELEC_INV)
		return ("Inverter");
	if (info->tru.charger)
		return ("Battery Charger");
	return ("Transformer-Rectifier");
}

static void
draw_tru_inv_info(const elec_comp_t *tru, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	const double TEXT_OFF_X = -8.5, TEXT_OFF_Y = 3;
	double y;

	ASSERT(tru != NULL);
	ASSERT(tru->info != NULL);
	ASSERT(tru->info->type == ELEC_TRU || tru->info->type == ELEC_INV);

	draw_comp_bg(cr, pos_scale, VECT2(pos.x - 2, pos.y + 5.5),
	    VECT2(14, 15.5));
	draw_tru_inv(cr, pos_scale, tru->info);

	y = pos.y + TEXT_OFF_Y;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Type: %s", tru_inv2str(tru->info));
	y += LINE_HEIGHT;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Efficiency: %.1f%%", tru->tru.eff * 100);
	y += LINE_HEIGHT;

	draw_comp_info(tru, cr, pos_scale, font_sz,
	    VECT2(pos.x + TEXT_OFF_X, y));
}

static void
draw_xfrmr_info(const elec_comp_t *xfrmr, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	const double TEXT_OFF_X = -8.5, TEXT_OFF_Y = 3;
	double y;

	ASSERT(xfrmr != NULL);
	ASSERT(xfrmr->info != NULL);
	ASSERT(xfrmr->info->type == ELEC_XFRMR);

	draw_comp_bg(cr, pos_scale, VECT2(pos.x - 2, pos.y + 5.5),
	    VECT2(14, 15.5));
	draw_xfrmr(cr, pos_scale, xfrmr->info);

	y = pos.y + TEXT_OFF_Y;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Type: Transformer");
	y += LINE_HEIGHT;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Efficiency: %.1f%%", xfrmr->xfrmr.eff * 100);
	y += LINE_HEIGHT;

	draw_comp_info(xfrmr, cr, pos_scale, font_sz,
	    VECT2(pos.x + TEXT_OFF_X, y));
}

static void
draw_tie_info(const elec_comp_t *tie, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	const double TEXT_OFF_X = -6.5, TEXT_OFF_Y = 3;
	double y;

	ASSERT(tie != NULL);
	ASSERT(tie->info != NULL);
	ASSERT3U(tie->info->type, ==, ELEC_TIE);

	draw_comp_bg(cr, pos_scale, VECT2(pos.x, pos.y + 3.5), VECT2(14, 11));
	draw_tie(cr, pos_scale, tie);

	y = pos.y + TEXT_OFF_Y;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Type: Tie/Contactor");
	y += LINE_HEIGHT;

	draw_comp_info(tie, cr, pos_scale, font_sz,
	    VECT2(pos.x + TEXT_OFF_X, y));
}

static void
draw_batt_info(const elec_comp_t *batt, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	const double TEXT_OFF_X = -7.5, TEXT_OFF_Y = 3;
	double y;

	ASSERT(batt != NULL);
	ASSERT(batt->info != NULL);
	ASSERT3U(batt->info->type, ==, ELEC_BATT);

	draw_comp_bg(cr, pos_scale, VECT2(pos.x - 1.5, pos.y + 5.5),
	    VECT2(13, 16));
	draw_batt(cr, pos_scale, batt->info, false);

	y = pos.y + TEXT_OFF_Y;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Type: Battery");
	y += LINE_HEIGHT;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Charge: %.1f%%", batt->batt.chg_rel * 100);
	y += LINE_HEIGHT;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Temp: %.1f C", KELVIN2C(batt->batt.T));
	y += LINE_HEIGHT;

	draw_comp_info(batt, cr, pos_scale, font_sz,
	    VECT2(pos.x + TEXT_OFF_X, y));
}

static void
draw_load_info(const elec_comp_t *load, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	const double TEXT_OFF_X = -6.5, TEXT_OFF_Y = 3;
	const char *type;
	double y;

	ASSERT(load != NULL);
	ASSERT(load->info != NULL);
	ASSERT3U(load->info->type, ==, ELEC_LOAD);

	draw_comp_bg(cr, pos_scale, VECT2(pos.x, pos.y + 3), VECT2(14, 10));
	draw_load(cr, pos_scale, font_sz, load->info);

	y = pos.y + TEXT_OFF_Y;

	switch (load->info->gui.load_type) {
	case GUI_LOAD_MOTOR:
		type = "Motor";
		break;
	default:
		type = "Load";
		break;
	}

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Type: %s", type);
	y += LINE_HEIGHT;

	draw_comp_info(load, cr, pos_scale, font_sz,
	    VECT2(pos.x + TEXT_OFF_X, y));
}

static void
draw_bus_info(const elec_comp_t *bus, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	enum { LINE_H = 3 };
	unsigned comp_i = 0, num_loads = 0;
	double y, height, U;
	cairo_path_t *path;
	char name[MAX_NAME_LEN];

	ASSERT(bus != NULL);
	ASSERT(bus->info != NULL);
	ASSERT3U(bus->info->type, ==, ELEC_BUS);

	for (unsigned i = 0; i < bus->n_links; i++) {
		ASSERT(bus->links[i].comp != NULL);
		ASSERT(bus->links[i].comp->info != NULL);
		if (bus->links[i].comp->info->type == ELEC_CB)
			num_loads++;
	}
	height = LINE_H * (1 + ceil(num_loads / 2.0));
	draw_comp_bg(cr, pos_scale, pos, VECT2(30, height));

	make_comp_name(bus->info->name, name);
	show_text_aligned(cr, PX(pos.x), PX(pos.y - height / 2 + 0.3 * LINE_H),
	    TEXT_ALIGN_CENTER, "%s", name);
	U = libelec_comp_get_in_volts(bus);
	show_text_aligned(cr, PX(pos.x), PX(pos.y - height / 2 + 0.7 * LINE_H),
	    TEXT_ALIGN_CENTER, "U: %.*fV", fixed_decimals(U, 4), U);
	y = pos.y - height / 2 + LINE_H * 1.5;

	for (unsigned i = 0; i < bus->n_links; i++) {
		const elec_comp_t *comp = bus->links[i].comp;
		const elec_comp_info_t *info = comp->info;
		vect2_t comp_pos;
		double I, W;

		if (info->type != ELEC_CB)
			continue;

		cairo_set_line_width(cr, 3);
		if (comp_i % 2 == 0) {
			comp_pos = VECT2(pos.x - 7.5, y);
			cairo_move_to(cr, PX(comp_pos.x + 1), PX(y));
		} else {
			comp_pos = VECT2(pos.x + 7.5, y);
			cairo_move_to(cr, PX(comp_pos.x - 1), PX(y));
		}
		cairo_line_to(cr, PX(pos.x), PX(y));
		path = cairo_copy_path(cr);
		cairo_stroke(cr);
		cairo_set_line_width(cr, 2);
		draw_src_path(cr, path, comp);

		if (comp_i + 2 < num_loads) {
			cairo_set_line_width(cr, 1);
			cairo_set_source_rgb(cr, 0.6, 0.6, 0.6);
			if (comp_i % 2 == 0) {
				cairo_move_to(cr, PX(pos.x - 14.5),
				    PX(y + LINE_H / 2.0));
				cairo_rel_line_to(cr, PX(13.5), 0);
			} else {
				cairo_move_to(cr, PX(pos.x + 14.5),
				    PX(y + LINE_H / 2.0));
				cairo_rel_line_to(cr, PX(-13.5), 0);
			}
			cairo_stroke(cr);
			cairo_set_line_width(cr, 2);
			cairo_set_source_rgb(cr, 0, 0, 0);
		}
		draw_cb_icon(cr, pos_scale, font_sz, comp_pos,
		    info->cb.fuse, comp->scb.cur_set,
		    info->cb.triphase, info->name,
		    (vect3_t){COMP_INFO_BG_RGB}, comp);
		I = libelec_comp_get_in_amps(comp);
		W = libelec_comp_get_in_pwr(comp);
		if (comp_i % 2 == 0) {
			show_text_aligned(cr, PX(pos.x - 14.5),
			    PX(y - 0.33 * LINE_H), TEXT_ALIGN_LEFT,
			    "I: %.*fA", fixed_decimals(I, 3), I);
			show_text_aligned(cr, PX(pos.x - 14.5), PX(y),
			    TEXT_ALIGN_LEFT,
			    "W: %.*fW", fixed_decimals(W, 3), W);
			show_text_aligned(cr, PX(pos.x - 1),
			    PX(y - 0.33 * LINE_H), TEXT_ALIGN_RIGHT,
			    "%s", comp->info->location);
		} else {
			show_text_aligned(cr, PX(pos.x + 10.5),
			    PX(y - 0.33 * LINE_H), TEXT_ALIGN_LEFT,
			    "I: %.*fA", fixed_decimals(I, 3), I);
			show_text_aligned(cr, PX(pos.x + 10.5), PX(y),
			    TEXT_ALIGN_LEFT,
			    "W: %.*fW", fixed_decimals(W, 3), W);
			show_text_aligned(cr, PX(pos.x + 1),
			    PX(y - 0.33 * LINE_H), TEXT_ALIGN_LEFT, "%s", comp->info->location);
		}
		comp_i++;
		if (comp_i % 2 == 0)
			y += LINE_H;
	}

	cairo_set_line_width(cr, 10);
	cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
	cairo_move_to(cr, PX(pos.x), PX(pos.y - height / 2 + LINE_H + 0.5));
	cairo_line_to(cr, PX(pos.x), PX(pos.y + height / 2 - LINE_H / 2));
	path = cairo_copy_path(cr);
	cairo_stroke(cr);
	cairo_set_line_width(cr, 4);
	draw_src_path(cr, path, bus);
	cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT);
	cairo_set_line_width(cr, 2);
}

/**
 * Draws a "component info" overlay window for a particular component.
 * This should be called if you want to draw a popup showing additional
 * information about a particular component.
 */
void
libelec_draw_comp_info(const elec_comp_t *comp, cairo_t *cr,
    double pos_scale, double font_sz, vect2_t pos)
{
	ASSERT(comp != NULL);
	ASSERT(cr != NULL);

	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_set_font_size(cr, font_sz);
	cairo_set_line_width(cr, 2);

	ASSERT(comp->info != NULL);
	switch (comp->info->type) {
	case ELEC_BATT:
		draw_batt_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_GEN:
		draw_gen_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_TRU:
	case ELEC_INV:
		draw_tru_inv_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_XFRMR:
		draw_xfrmr_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_LOAD:
		draw_load_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_BUS:
		draw_bus_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_CB:
	case ELEC_SHUNT:
		draw_scb_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_TIE:
		draw_tie_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_DIODE:
		draw_diode_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_LABEL_BOX:
		break;
	}
}
