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

#include "libelec_drawing.h"
#include "libelec_types_impl.h"

#define	PX(pos)	(pos_scale * (pos))

enum {
    TEXT_ALIGN_LEFT,
    TEXT_ALIGN_CENTER,
    TEXT_ALIGN_RIGHT
};

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
		*comp_pos = pick_closer(*bus_pos, vect2_add(pos, VECT2(-3, 0)),
		    vect2_add(pos, VECT2(2, 0)));
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

		if (!elec_comp_get_nearest_pos(comp, &comp_pos, &bus_pos,
		    bus->info->gui.sz, &align_vert)) {
			continue;
		}
		if (bus->info->gui.sz != 0 && !bus->info->gui.virt) {
			cairo_arc(cr, PX(bus_pos.x), PX(bus_pos.y), PX(0.5),
			    0, DEG2RAD(360));
		} else {
			cairo_arc(cr, PX(bus_pos.x), PX(bus_pos.y), PX(0.2),
			    0, DEG2RAD(360));
		}
		cairo_fill(cr);

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
		cairo_stroke(cr);
	}
}

static void
draw_gen(cairo_t *cr, double pos_scale, const elec_comp_info_t *info)
{
	vect2_t pos;

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;

	cairo_new_path(cr);
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_arc(cr, PX(pos.x), PX(pos.y), PX(1.2), 0, DEG2RAD(360));
	cairo_fill(cr);
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
draw_bus(cairo_t *cr, double pos_scale, const elec_comp_info_t *info)
{
	vect2_t pos;

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;

	cairo_new_path(cr);

	if (!info->gui.virt)
		cairo_set_line_width(cr, 10);
	cairo_move_to(cr, PX(pos.x), PX(pos.y - info->gui.sz));
	cairo_rel_line_to(cr, 0, PX(2 * info->gui.sz));
	cairo_stroke(cr);
	cairo_set_line_width(cr, 2);

	if (info->gui.sz != 0 && !info->gui.virt) {
		show_text_aligned(cr, PX(pos.x), PX(pos.y - info->gui.sz - 1),
		    TEXT_ALIGN_CENTER, "%s", info->name);
	}
}

static void
draw_cb(cairo_t *cr, double pos_scale, const elec_comp_t *cb)
{
	const elec_comp_info_t *info;
	vect2_t pos;

	ASSERT(cr != NULL);
	ASSERT(cb != NULL);
	info = cb->info;
	pos = info->gui.pos;

	cairo_new_path(cr);

	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_arc(cr, PX(pos.x - 1), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_new_sub_path(cr);
	cairo_arc(cr, PX(pos.x + 1), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_fill(cr);

	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_arc(cr, PX(pos.x - 1), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_new_sub_path(cr);
	cairo_arc(cr, PX(pos.x + 1), PX(pos.y), PX(0.2), 0, DEG2RAD(360));
	cairo_new_sub_path(cr);
	if (cb->scb.cur_set) {
		cairo_arc(cr, PX(pos.x), PX(pos.y + 0.5), PX(1.1), DEG2RAD(215),
		    DEG2RAD(-35));
		cairo_move_to(cr, PX(pos.x), PX(pos.y - 0.5));
	} else {
		cairo_arc(cr, PX(pos.x), PX(pos.y), PX(1.1), DEG2RAD(215),
		    DEG2RAD(-35));
		cairo_move_to(cr, PX(pos.x), PX(pos.y - 1));
	}
	cairo_rel_line_to(cr, 0, PX(-0.5));
	cairo_rel_move_to(cr, PX(-0.4), 0);
	cairo_rel_line_to(cr, PX(0.8), 0);
	cairo_stroke(cr);

	if (strncmp(info->name, "CB_", 3) == 0) {
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 0.8),
		    TEXT_ALIGN_CENTER, "%s", &info->name[3]);
	} else {
		show_text_aligned(cr, PX(pos.x), PX(pos.y + 0.8),
		    TEXT_ALIGN_CENTER, "%s", info->name);
	}
}

static void
draw_shunt(cairo_t *cr, double pos_scale, const elec_comp_info_t *info)
{
	vect2_t pos;

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;

	cairo_new_path(cr);
	cairo_move_to(cr, PX(pos.x - 3), PX(pos.y));
	cairo_rel_line_to(cr, PX(1), PX(0));
	for (int i = 0; i < 3; i++) {
		cairo_rel_line_to(cr, PX(0.25), PX(-0.7));
		cairo_rel_line_to(cr, PX(0.5), PX(1.4));
		cairo_rel_line_to(cr, PX(0.25), PX(-0.7));
	}
	cairo_rel_line_to(cr, PX(1), PX(0));
	cairo_stroke(cr);

	show_text_aligned(cr, PX(pos.x), PX(pos.y + 1.7),
	    TEXT_ALIGN_CENTER, "%s", info->name);
}

static void
draw_tru(cairo_t *cr, double pos_scale, const elec_comp_info_t *info)
{
	vect2_t pos;

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;

	cairo_new_path(cr);

	cairo_set_source_rgb(cr, 1, 1, 1);
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
		cairo_move_to(cr, PX(endpt[0].x), PX(endpt[0].y));
		cairo_line_to(cr, PX(endpt[1].x), PX(endpt[1].y));
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
	}
	cairo_stroke(cr);
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
draw_diode(cairo_t *cr, double pos_scale, const elec_comp_t *diode)
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
draw_batt(cairo_t *cr, double pos_scale, const elec_comp_info_t *info)
{
	vect2_t pos;

	ASSERT(cr != NULL);
	ASSERT(info != NULL);
	pos = info->gui.pos;

	cairo_new_path(cr);

	cairo_move_to(cr, PX(pos.x + 0.4), PX(pos.y - 0.6));
	cairo_rel_line_to(cr, PX(0.4), 0);
	cairo_move_to(cr, PX(pos.x + 0.6), PX(pos.y - 0.8));
	cairo_rel_line_to(cr, 0, PX(0.4));

	cairo_move_to(cr, PX(pos.x - 1), PX(pos.y - 0.2));
	cairo_rel_line_to(cr, PX(2), PX(0));
	cairo_move_to(cr, PX(pos.x - 0.6), PX(pos.y + 0.2));
	cairo_rel_line_to(cr, PX(1.2), PX(0));

	cairo_move_to(cr, PX(pos.x), PX(pos.y + 0.2));
	cairo_rel_line_to(cr, 0, PX(2.3));
	cairo_move_to(cr, PX(pos.x - 1), PX(pos.y + 2.5));
	cairo_rel_line_to(cr, PX(2), PX(0));
	cairo_move_to(cr, PX(pos.x - 0.7), PX(pos.y + 2.9));
	cairo_rel_line_to(cr, PX(1.4), PX(0));
	cairo_move_to(cr, PX(pos.x - 0.4), PX(pos.y + 3.3));
	cairo_rel_line_to(cr, PX(0.8), PX(0));
	cairo_stroke(cr);

	show_text_aligned(cr, PX(pos.x - 1.2), PX(pos.y),
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
			draw_bus(cr, pos_scale, info);
			break;
		case ELEC_GEN:
			draw_gen(cr, pos_scale, info);
			break;
		case ELEC_CB:
			draw_cb(cr, pos_scale, comp);
			break;
		case ELEC_SHUNT:
			draw_shunt(cr, pos_scale, info);
			break;
		case ELEC_TRU:
			draw_tru(cr, pos_scale, info);
			break;
		case ELEC_TIE:
			draw_tie(cr, pos_scale, comp);
			break;
		case ELEC_DIODE:
			draw_diode(cr, pos_scale, comp);
			break;
		case ELEC_LOAD:
			draw_load(cr, pos_scale, font_sz, info);
			break;
		case ELEC_BATT:
			draw_batt(cr, pos_scale, info);
			break;
		default:
			break;
		}
	}
}
