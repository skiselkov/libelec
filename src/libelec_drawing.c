/*
 * CONFIDENTIAL
 *
 * Copyright 2020 Saso Kiselkov. All rights reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property
 * of Saso Kiselkov. The intellectual and technical concepts contained
 * herein are proprietary to Saso Kiselkov and may be covered by U.S. and
 * Foreign Patents, patents in process, and are protected by trade secret
 * or copyright law. Dissemination of this information or reproduction of
 * this material is strictly forbidden unless prior written permission is
 * obtained from Saso Kiselkov.
 */

#include <acfutils/perf.h>

#include "libelec_drawing.h"
#include "libelec_types_impl.h"

#define	PX(pos)			(pos_scale * (pos))
#define	LINE_HEIGHT		1
#define	COMP_INFO_BG_RGB	0.8, 0.8, 0.8

enum {
    TEXT_ALIGN_LEFT,
    TEXT_ALIGN_CENTER,
    TEXT_ALIGN_RIGHT
};

static void show_text_aligned(cairo_t *cr, double x, double y, unsigned align,
    const char *format, ...) PRINTF_ATTR(5);

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
	ASSERT3U(i, <, tie->tie.n_buses);
	ASSERT(tie->tie.n_buses != 0);

	pos = tie->info->gui.pos;
	if (tie->tie.n_buses == 2) {
		if (i == 0)
			return (VECT2(pos.x - 1, pos.y));
		else
			return (VECT2(pos.x + 1, pos.y));
	} else if (tie->tie.n_buses == 3) {
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

		off = vect2_rot(off, i * (360.0 / tie->tie.n_buses));
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
	    comp->info->type == ELEC_BATT);

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

		for (unsigned i = 0; i < comp->tie.n_buses; i++)
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
	const volatile elec_comp_t *src;

	ASSERT(cr != NULL);
	ASSERT(path != NULL);
	ASSERT(comp != NULL);

	src = comp->src_vis;
	if (src != NULL) {
		vect3_t color = src->info->gui.color;
		cairo_append_path(cr, path);
		cairo_set_source_rgb(cr, color.x, color.y, color.z);
		cairo_stroke(cr);
		cairo_set_source_rgb(cr, 0, 0, 0);
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

	for (unsigned i = 0; i < bus->bus.n_comps; i++) {
		vect2_t bus_pos = bus->info->gui.pos;
		vect2_t comp_pos;
		const elec_comp_t *comp = bus->bus.comps[i];
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
			} else if (bus->bus.n_comps > 2) {
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
	if (info->gen.ac) {
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

	show_text_aligned(cr, PX(pos.x), PX(pos.y + 2), TEXT_ALIGN_CENTER,
	    "%s", info->name);
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
		show_text_aligned(cr, PX(pos.x), PX(pos.y - info->gui.sz - 1),
		    TEXT_ALIGN_CENTER, "%s", info->name);
	}
}

static void
draw_cb(cairo_t *cr, double pos_scale, const elec_comp_t *cb, double font_sz,
    vect3_t bg_color)
{
	double text_y_off;
	const elec_comp_info_t *info;
	vect2_t pos;
	cairo_path_t *path;
	const char *name;

	ASSERT(cr != NULL);
	ASSERT(cb != NULL);
	info = cb->info;
	pos = info->gui.pos;

	cairo_new_path(cr);

	if (!cb->info->cb.fuse) {
		/* Yoke on the top */
		if (cb->scb.cur_set)
			cairo_move_to(cr, PX(pos.x), PX(pos.y - 0.5));
		else
			cairo_move_to(cr, PX(pos.x), PX(pos.y - 1));
		cairo_set_line_width(cr, 2);
		cairo_rel_line_to(cr, 0, PX(-0.5));
		cairo_rel_move_to(cr, PX(-0.4), 0);
		cairo_rel_line_to(cr, PX(0.8), 0);
		cairo_stroke(cr);
		/* The arch of the breaker */
		cairo_set_line_width(cr, 3);
		if (cb->scb.cur_set) {
			cairo_arc(cr, PX(pos.x), PX(pos.y + 0.5), PX(1.1),
			    DEG2RAD(215), DEG2RAD(-35));
		} else {
			cairo_arc(cr, PX(pos.x), PX(pos.y), PX(1.1),
			    DEG2RAD(215), DEG2RAD(-35));
		}
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
	draw_src_path(cr, path, cb);

	if (cb->info->cb.fuse && (cb->ro.failed || !cb->scb.cur_set)) {
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

	if (info->cb.triphase) {
		cairo_set_font_size(cr, round(0.75 * font_sz));
		show_text_aligned(cr, PX(pos.x), PX(pos.y), TEXT_ALIGN_CENTER,
		    "3P");
		cairo_set_font_size(cr, font_sz);
	}

	text_y_off = (cb->info->cb.fuse ? 1.5 : 0.8);
	name = (strncmp(info->name, "CB_", 3) == 0 ? &info->name[3] :
	    info->name);
	show_text_aligned(cr, PX(pos.x), PX(pos.y + text_y_off),
	    TEXT_ALIGN_CENTER, "%s", name);
}

static void
draw_shunt(cairo_t *cr, double pos_scale, const elec_comp_t *shunt)
{
	vect2_t pos;
	cairo_path_t *path;
	const elec_comp_info_t *info;

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

	show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.7),
	    TEXT_ALIGN_CENTER, "%s", info->name);
}

static void
draw_tru(cairo_t *cr, double pos_scale, const elec_comp_info_t *info)
{
	vect2_t pos;
	vect3_t color;

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;
	color = info->gui.color;

	cairo_new_path(cr);

	cairo_set_source_rgb(cr, color.x, color.y, color.z);
	cairo_rectangle(cr, PX(pos.x - 1.5), PX(pos.y - 1.5), PX(3), PX(3));
	cairo_fill(cr);

	cairo_set_source_rgb(cr, 0, 0, 0);

	cairo_rectangle(cr, PX(pos.x - 1.5), PX(pos.y - 1.5), PX(3), PX(3));

	cairo_move_to(cr, PX(pos.x - 1.5), PX(pos.y + 1.5));
	cairo_rel_line_to(cr, PX(3), PX(-3));

	cairo_move_to(cr, PX(pos.x - 1.3), PX(pos.y - 1));
	cairo_rel_curve_to(cr, PX(0.1), PX(-0.4),
	    PX(0.4), PX(-0.4), PX(0.5), 0);
	cairo_rel_curve_to(cr, PX(0.1), PX(0.4),
	    PX(0.4), PX(0.4), PX(0.5), 0);

	cairo_move_to(cr, PX(pos.x + 1.3), PX(pos.y + 1));
	cairo_rel_line_to(cr, PX(-1), 0);

	cairo_stroke(cr);

	show_text_aligned(cr, PX(pos.x - 2), PX(pos.y),
	    TEXT_ALIGN_RIGHT, "%s", info->name);
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

	ASSERT(cr != NULL);
	ASSERT(tie != NULL);
	pos = tie->info->gui.pos;

	cairo_new_path(cr);

	cairo_set_line_width(cr, 4);
	for (unsigned i = 0; i < tie->tie.n_buses; i++) {
		elec_comp_t *remote_bus = tie->tie.buses[i];
		vect2_t conn = VECT2(1e9, 1e9);

		if (!tie->tie.cur_state[i])
			continue;

		for (unsigned j = 0; j < tie->tie.n_buses; j++) {
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
		if (tie->tie.n_buses == 2) {
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
	for (unsigned i = 0; i < tie->tie.n_buses; i++)
		draw_node(cr, pos_scale, tie_node_pos(tie, i));

	if (tie->tie.n_buses == 3) {
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.8),
		    TEXT_ALIGN_CENTER, "%s", tie->info->name);
	} else {
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.5),
		    TEXT_ALIGN_CENTER, "%s", tie->info->name);
	}
}

static void
draw_diode(cairo_t *cr, double pos_scale, const elec_comp_t *diode,
    bool draw_line)
{
	vect2_t pos;

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

	show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.5),
	    TEXT_ALIGN_CENTER, "%s", diode->info->name);
}

static void
draw_load(cairo_t *cr, double pos_scale, double font_sz,
    const elec_comp_info_t *info)
{
	vect2_t pos;

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

	show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.7),
	    TEXT_ALIGN_CENTER, "%s", info->name);
}

static void
draw_batt(cairo_t *cr, double pos_scale, const elec_comp_info_t *info,
    bool draw_ground)
{
	vect2_t pos;
	vect3_t color;

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

	show_text_aligned(cr, PX(pos.x - 1.4), PX(pos.y),
	    TEXT_ALIGN_RIGHT, "%s", info->name);
}

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
		if (IS_NULL_VECT(info->gui.pos))
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
			draw_tru(cr, pos_scale, info);
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
		default:
			break;
		}
	}
}

static void
draw_comp_bg(cairo_t *cr, double pos_scale, vect2_t pos, vect2_t sz)
{
	cairo_path_t *path;

	ASSERT(cr != NULL);

#ifndef	LIBELEC_NO_GL
	mt_cairo_render_rounded_rectangle(cr, PX(pos.x - sz.x / 2),
	    PX(pos.y - sz.y / 2), PX(sz.x), PX(sz.y), PX(0.5));
#else	/* defined(LIBELEC_NO_GL) */
	UNUSED(pos_scale);
	UNUSED(pos);
	UNUSED(sz);
#endif	/* defined(LIBELEC_NO_GL) */
	path = cairo_copy_path(cr);
	cairo_set_source_rgb(cr, COMP_INFO_BG_RGB);
	cairo_fill(cr);

	cairo_append_path(cr, path);
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_stroke(cr);

	cairo_path_destroy(path);
}

static void
draw_comp_info(const elec_comp_t *comp, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	double U_in, I_in, W_in, U_out, I_out, W_out;
	const elec_comp_t *src;

	ASSERT(comp != NULL);
	ASSERT(cr != NULL);
	src = comp->src_vis;

	U_in = libelec_comp_get_in_volts(comp);
	U_out = libelec_comp_get_out_volts(comp);
	I_in = libelec_comp_get_in_amps(comp);
	I_out = libelec_comp_get_out_amps(comp);
	W_in = libelec_comp_get_in_pwr(comp);
	W_out = libelec_comp_get_out_pwr(comp);

	if (comp->info->type != ELEC_GEN) {
		show_text_aligned(cr, PX(pos.x), PX(pos.y),
		    TEXT_ALIGN_LEFT, "Powered by: %s",
		    src != NULL ? src->info->name : "nothing");
		pos.y += LINE_HEIGHT;
	}

	switch (comp->info->type) {
	case ELEC_BATT:
	case ELEC_TRU:
		cairo_set_font_size(cr, 0.75 * font_sz);
		for (int i = 0; i < 6; i++) {
			show_text_aligned(cr, PX(pos.x + 0.5),
			    PX(pos.y + (i + 0.25) * LINE_HEIGHT),
			    TEXT_ALIGN_LEFT, i < 3 ? "IN" : "OUT");
		}

		cairo_set_font_size(cr, font_sz);
		show_text_aligned(cr, PX(pos.x), PX(pos.y),
		    TEXT_ALIGN_LEFT, "U   : %.*fV", fixed_decimals(U_in, 3),
		    U_in);
		show_text_aligned(cr, PX(pos.x), PX(pos.y + LINE_HEIGHT),
		    TEXT_ALIGN_LEFT, "I   : %.*fA", fixed_decimals(I_in, 3),
		    I_in);
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 2 * LINE_HEIGHT),
		    TEXT_ALIGN_LEFT, "W   : %.*fW", fixed_decimals(W_in, 3),
		    W_in);
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 3 * LINE_HEIGHT),
		    TEXT_ALIGN_LEFT, "U   : %.*fV", fixed_decimals(U_out, 3),
		    U_out);
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 4 * LINE_HEIGHT),
		    TEXT_ALIGN_LEFT, "I   : %.*fA", fixed_decimals(I_out, 3),
		    I_out);
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 5 * LINE_HEIGHT),
		    TEXT_ALIGN_LEFT, "W   : %.*fW", fixed_decimals(W_out, 3),
		    W_out);
		break;
	case ELEC_BUS:
	case ELEC_GEN:
	case ELEC_LOAD:
	case ELEC_CB:
	case ELEC_SHUNT:
	case ELEC_TIE:
	case ELEC_DIODE:
		show_text_aligned(cr, PX(pos.x), PX(pos.y),
		    TEXT_ALIGN_LEFT, "U: %.*fV", fixed_decimals(U_in, 3), U_in);
		show_text_aligned(cr, PX(pos.x), PX(pos.y + LINE_HEIGHT),
		    TEXT_ALIGN_LEFT, "I: %.*fA", fixed_decimals(I_in, 3), I_in);
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 2 * LINE_HEIGHT),
		    TEXT_ALIGN_LEFT, "W: %.*fW", fixed_decimals(W_in, 3), W_in);
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
	double y, height;

	ASSERT(cb != NULL);
	ASSERT(cb->info != NULL);
	ASSERT(cb->info->type == ELEC_CB || cb->info->type == ELEC_SHUNT);

	height = (cb->info->type == ELEC_CB ? 13 : 12);
	draw_comp_bg(cr, pos_scale, VECT2(pos.x, pos.y + 4), VECT2(14, height));

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
	    "Type: %s Generator", gen->info->gen.ac ? "AC" : "DC");
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

static void
draw_tru_info(const elec_comp_t *tru, cairo_t *cr, double pos_scale,
    double font_sz, vect2_t pos)
{
	const double TEXT_OFF_X = -8.5, TEXT_OFF_Y = 3;
	double y;

	ASSERT(tru != NULL);
	ASSERT(tru->info != NULL);
	ASSERT3U(tru->info->type, ==, ELEC_TRU);

	draw_comp_bg(cr, pos_scale, VECT2(pos.x - 2, pos.y + 5), VECT2(14, 15));
	draw_tru(cr, pos_scale, tru->info);

	y = pos.y + TEXT_OFF_Y;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Type: %s", !tru->info->tru.charger ? "Transformer-Rectifier" :
	    "Battery Charger");
	y += LINE_HEIGHT;

	show_text_aligned(cr, PX(pos.x + TEXT_OFF_X), PX(y), TEXT_ALIGN_LEFT,
	    "Efficiency: %.1f%%", tru->tru.eff * 100);
	y += LINE_HEIGHT;

	draw_comp_info(tru, cr, pos_scale, font_sz,
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

	draw_comp_bg(cr, pos_scale, VECT2(pos.x, pos.y + 3), VECT2(14, 10));
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
		draw_tru_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_LOAD:
		draw_load_info(comp, cr, pos_scale, font_sz, pos);
		break;
	case ELEC_BUS:
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
	}
}
