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

#include <XPLMDisplay.h>

#include <acfutils/helpers.h>

#include "libelec_types_impl.h"
#include "libelec_drawing.h"
#include "libelec_vis.h"

#define	WIN_SZ		600
#define	WIN_FPS		20
#define	WIN_FPS_FAST	30

struct libelec_vis_s {
	elec_sys_t		*sys;
	mt_cairo_render_t	*mtcr;
	mt_cairo_uploader_t	*mtul;
	XPLMWindowID		win;
	double			pos_scale;
	double			font_sz;
	double			zoom;
	vect2_t			offset;
	vect2_t			mouse_down;
	vect2_t			mouse_prev;

	mutex_t			lock;
	const elec_comp_t	*highlight;
	const elec_comp_t	*selected;
};

static vect2_t
comp_info2sz(const elec_comp_info_t *info)
{
	ASSERT(info != NULL);

	switch (info->type) {
	case ELEC_BATT:
	case ELEC_GEN:
	case ELEC_CB:
	case ELEC_TIE:
	case ELEC_DIODE:
		return (VECT2(3, 3));
	case ELEC_TRU:
	case ELEC_LOAD:
		return (VECT2(3.5, 3.5));
	case ELEC_BUS:
		if (info->gui.sz == 0)
			return (NULL_VECT2);
		return (VECT2(2, 1 + 2 * info->gui.sz));
	case ELEC_SHUNT:
		return (VECT2(6, 2));
	default:
		VERIFY_FAIL();
	}
}

static vect2_t
cursor_coords_xlate(libelec_vis_t *vis, int x, int y)
{
	int left, top, right, bottom, w, h;
	vect2_t mouse;

	ASSERT(vis != NULL);

	XPLMGetWindowGeometry(vis->win, &left, &top, &right, &bottom);
	w = right - left;
	h = top - bottom;
	x = x - left - w / 2;
	y = top - y - h / 2;

	mouse = vect2_scmul(VECT2(x, y), 1 / vis->zoom);
	mouse = vect2_sub(mouse, vect2_scmul(vis->offset, 1 / vis->zoom));
	mouse = vect2_scmul(mouse, 1 / vis->pos_scale);

	return (mouse);
}

static const elec_comp_t *
hit_test(libelec_vis_t *vis, int x, int y)
{
	vect2_t mouse;
	ASSERT(vis != NULL);

	mouse = cursor_coords_xlate(vis, x, y);

	for (const elec_comp_t *comp = list_head(&vis->sys->comps);
	    comp != NULL; comp = list_next(&vis->sys->comps, comp)) {
		vect2_t pos, sz;

		pos = comp->info->gui.pos;
		if (IS_NULL_VECT(pos) || comp->info->gui.virt ||
		    comp->info->gui.invis) {
			continue;
		}
		sz = comp_info2sz(comp->info);
		if (mouse.x >= pos.x - sz.x / 2 &&
		    mouse.x < pos.x + sz.x / 2 &&
		    mouse.y >= pos.y - sz.y / 2 &&
		    mouse.y < pos.y + sz.y / 2) {
			return (comp);
		}
	}

	return (NULL);
}

static void
draw_highlight(cairo_t *cr, libelec_vis_t *vis)
{
	ASSERT(cr != NULL);
	ASSERT(vis != NULL);

	mutex_enter(&vis->lock);
	if (vis->highlight != NULL) {
		vect2_t pos = vis->highlight->info->gui.pos;
		vect2_t sz = comp_info2sz(vis->highlight->info);

		cairo_set_line_width(cr, 3);
		cairo_set_source_rgb(cr, 0, 0, 0);
		cairo_rectangle(cr, vis->pos_scale * (pos.x - sz.x / 2),
		    vis->pos_scale * (pos.y - sz.y / 2),
		    vis->pos_scale * sz.x, vis->pos_scale * sz.y);
		cairo_stroke(cr);

		cairo_set_line_width(cr, 2);
		cairo_set_source_rgb(cr, 0, 1, 1);
		cairo_rectangle(cr, vis->pos_scale * (pos.x - sz.x / 2),
		    vis->pos_scale * (pos.y - sz.y / 2),
		    vis->pos_scale * sz.x, vis->pos_scale * sz.y);
		cairo_stroke(cr);
	}
	mutex_exit(&vis->lock);
}

static void
draw_selected(cairo_t *cr, libelec_vis_t *vis)
{
	ASSERT(cr != NULL);
	ASSERT(vis != NULL);

	mutex_enter(&vis->lock);
	if (vis->selected) {
		libelec_draw_comp_info(vis->selected, cr, vis->pos_scale,
		    vis->font_sz, vis->selected->info->gui.pos);
	}
	mutex_exit(&vis->lock);
}

static void
render_cb(cairo_t *cr, unsigned w, unsigned h, void *userinfo)
{
	libelec_vis_t *vis;

	ASSERT(cr != NULL);
	UNUSED(w);
	UNUSED(h);
	ASSERT(userinfo != NULL);
	vis = userinfo;

	cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL,
	    CAIRO_FONT_WEIGHT_NORMAL);

	cairo_identity_matrix(cr);

	cairo_translate(cr, w / 2, h / 2);
	cairo_translate(cr, vis->offset.x, vis->offset.y);
	cairo_scale(cr, vis->zoom, vis->zoom);
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_paint(cr);
	libelec_draw_layout(vis->sys, cr, vis->pos_scale, vis->font_sz);

	draw_highlight(cr, vis);
	draw_selected(cr, vis);
}

static void
recreate_mtcr(libelec_vis_t *vis)
{
	int left, top, right, bottom;

	ASSERT(vis != NULL);

	if (vis->mtcr != NULL)
		mt_cairo_render_fini(vis->mtcr);
	ASSERT(vis->win != NULL);
	XPLMGetWindowGeometry(vis->win, &left, &top, &right, &bottom);
	vis->mtcr = mt_cairo_render_init(right - left, top - bottom, WIN_FPS,
	    NULL, render_cb, NULL, vis);
	if (vis->mtul != NULL)
		mt_cairo_render_set_uploader(vis->mtcr, vis->mtul);
}

static int
win_click(XPLMWindowID win, int x, int y, XPLMMouseStatus mouse, void *refcon)
{
	int left, top, x_rel, y_rel;

	libelec_vis_t* vis;
	vect2_t off;

	ASSERT(win != NULL);
	ASSERT(refcon != NULL);
	vis = refcon;

	XPLMGetWindowGeometry(vis->win, &left, &top, NULL, NULL);
	x_rel = x - left;
	y_rel = top - y;

	if (mouse == xplm_MouseDown)
		vis->mouse_prev = vis->mouse_down = VECT2(x_rel, y_rel);
	off = vect2_sub(VECT2(x_rel, y_rel), vis->mouse_prev);
	vis->offset = vect2_add(vis->offset, off);
	vis->mouse_prev = VECT2(x_rel, y_rel);

	/* Increase rendering rate while dragging */
	if (mouse == xplm_MouseDown || mouse == xplm_MouseDrag) {
		if (mt_cairo_render_get_fps(vis->mtcr) != WIN_FPS_FAST)
			mt_cairo_render_set_fps(vis->mtcr, WIN_FPS_FAST);
	} else {
		mt_cairo_render_set_fps(vis->mtcr, WIN_FPS);
	}
	if (mouse == xplm_MouseUp &&
	    vect2_abs(vect2_sub(vis->mouse_down, vis->mouse_prev)) < 4) {
		mutex_enter(&vis->lock);
		vis->selected = hit_test(vis, x, y);
		mutex_exit(&vis->lock);
	}

	return (1);
}

static int
win_wheel(XPLMWindowID win, int x, int y, int wheel, int clicks, void *refcon)
{
	int left, top, right, bottom, w, h;
	libelec_vis_t *vis;

	ASSERT(win != NULL);
	UNUSED(x);
	UNUSED(y);
	ASSERT(refcon != NULL);
	vis = refcon;

	if (wheel != 0)
		return (1);


	XPLMGetWindowGeometry(vis->win, &left, &top, &right, &bottom);
	w = right - left;
	h = top - bottom;
	UNUSED(w);
	UNUSED(h);

	/*
	 * Limit the zoom range
	 */
	if (vis->zoom > 10)
		clicks = MIN(clicks, 0);
	if (vis->zoom < 0.1)
		clicks = MAX(clicks, 0);
	for (; clicks > 0; clicks--) {
		vis->zoom *= 1.25;
		vis->offset = vect2_scmul(vis->offset, 1.25);
	}
	for (; clicks < 0; clicks++) {
		vis->zoom /= 1.25;
		vis->offset = vect2_scmul(vis->offset, 1.0 / 1.25);
	}

	return (1);
}

static void
win_draw(XPLMWindowID win, void *refcon)
{
	int left, top, right, bottom, w, h;
	libelec_vis_t *vis;

	ASSERT(win != NULL);
	ASSERT(refcon != NULL);
	vis = refcon;

	XPLMGetWindowGeometry(vis->win, &left, &top, &right, &bottom);
	w = right - left;
	h = top - bottom;
	ASSERT(vis->mtcr != NULL);
	if (w != (int)mt_cairo_render_get_width(vis->mtcr) ||
	    h != (int)mt_cairo_render_get_height(vis->mtcr)) {
		recreate_mtcr(vis);
	}
	mt_cairo_render_draw(vis->mtcr, VECT2(left, bottom), VECT2(w, h));
}

static XPLMCursorStatus
win_cursor(XPLMWindowID win, int x, int y, void *refcon)
{
	libelec_vis_t *vis;

	ASSERT(win != NULL);
	ASSERT(refcon != NULL);
	vis = refcon;

	mutex_enter(&vis->lock);
	vis->highlight = hit_test(vis, x, y);
	mutex_exit(&vis->lock);

	return (xplm_CursorArrow);
}

libelec_vis_t *
libelec_vis_new(elec_sys_t *sys, double pos_scale, double font_sz,
    mt_cairo_uploader_t *mtul)
{
	libelec_vis_t *vis = safe_calloc(1, sizeof (*vis));
	XPLMCreateWindow_t cr = {
	    .structSize = sizeof (cr),
	    .left = 100,
	    .top = 100 + WIN_SZ,
	    .right = 100 + WIN_SZ,
	    .bottom = 100,
	    .handleMouseClickFunc = win_click,
	    .handleMouseWheelFunc = win_wheel,
	    .handleCursorFunc = win_cursor,
	    .drawWindowFunc = win_draw,
	    .decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle,
	    .layer = xplm_WindowLayerFloatingWindows,
	    .refcon = vis
	};

	ASSERT(sys != NULL);

	vis->sys = sys;
	vis->win = XPLMCreateWindowEx(&cr);
	ASSERT(vis->win != NULL);
	vis->pos_scale = pos_scale;
	vis->font_sz = font_sz;
	vis->mtul = mtul;
	vis->zoom = 1;
	mutex_init(&vis->lock);

	XPLMSetWindowTitle(vis->win, "Electrical Network");
	XPLMSetWindowResizingLimits(vis->win, 200, 200, 100000, 100000);

	recreate_mtcr(vis);

	return (vis);
}

void
libelec_vis_destroy(libelec_vis_t *vis)
{
	if (vis == NULL)
		return;

	mt_cairo_render_fini(vis->mtcr);
	mutex_destroy(&vis->lock);
	XPLMDestroyWindow(vis->win);

	free(vis);
}

void
libelec_vis_set_offset(libelec_vis_t *vis, vect2_t offset)
{
	ASSERT(vis != NULL);
	vis->offset = offset;
}

bool
libelec_vis_is_open(libelec_vis_t *vis)
{
	ASSERT(vis != NULL);
	ASSERT(vis->win != NULL);
	return (XPLMGetWindowIsVisible(vis->win));
}

void
libelec_vis_open(libelec_vis_t *vis)
{
	ASSERT(vis != NULL);
	ASSERT(vis->win != NULL);
	XPLMSetWindowIsVisible(vis->win, true);
}
