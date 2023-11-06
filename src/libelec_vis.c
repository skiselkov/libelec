/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */

#include <XPLMDisplay.h>
#include <XPLMProcessing.h>

#include <acfutils/helpers.h>
#include <acfutils/widget.h>

#include "libelec_types_impl.h"
#include "libelec_drawing.h"
#include "libelec_vis.h"

#define	WIN_WIDTH	900	/* px */
#define	WIN_HEIGHT	600	/* px */
#define	WIN_FPS		20
#define	WIN_FPS_FAST	30

struct libelec_vis_s {
	const elec_sys_t	*sys;
	mt_cairo_render_t	*mtcr;
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

	XPLMFlightLoopID	floop;
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
	case ELEC_INV:
	case ELEC_XFRMR:
	case ELEC_LOAD:
		return (VECT2(3.5, 3.5));
	case ELEC_BUS:
		if (info->gui.sz == 0)
			return (NULL_VECT2);
		return (VECT2(2, 1 + 2 * info->gui.sz));
	case ELEC_SHUNT:
		return (VECT2(6, 2));
	case ELEC_LABEL_BOX:
		VERIFY_FAIL();
	}
	VERIFY_FAIL();
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

static float
vis_floop_cb(UNUSED_ATTR float unused1, UNUSED_ATTR float unused2,
    UNUSED_ATTR int unused3, void *refcon)
{
	libelec_vis_t *vis;

	ASSERT(refcon != NULL);
	vis = refcon;

	if (!libelec_vis_is_open(vis) && vis->mtcr != NULL) {
		mt_cairo_render_fini(vis->mtcr);
		vis->mtcr = NULL;
		/*
		 * Stop the flight loop callback, we will reschedule it
		 * again when the window is re-opened.
		 */
		return (0);
	}
	return (-1);
}

/**
 * Creates a window showing a visualization of an electrical network.
 * This is using the drawing routines within `libelec_drawing.h` to
 * draw an image of the network, as well as implement mouse interaction
 * for panning & zooming around in the network.
 *
 * @param sys The electrical system to display. This must have been
 *	previously initialized using libelec_new(). The visualizer keeps
 *	a reference to the system, so you must NOT destroy the
 *	elec_sys_t before the visualizer.
 * @param pos_scale A scaling multiplier applied to all `GUI_POS` stanzas
 *	in the network layout configuration file. This lets you use abstract
 *	size units in the config file and then scale them up as you see fit.
 * @param font_sz Default font size to be used for all drawing. This sets
 *	the default header and title sizes, and is also used as a baseline
 *	for smaller font scales for things such as subscripts.
 * @return An initialized visualizer, ready for display. The window of the
 *	visualizer is not yet visible. You should call libelec_vis_open()
 *	to make it visible.
 */
libelec_vis_t *
libelec_vis_new(const elec_sys_t *sys, double pos_scale, double font_sz)
{
	libelec_vis_t *vis = safe_calloc(1, sizeof (*vis));
	XPLMCreateWindow_t cr = {
	    .structSize = sizeof (cr),
	    .left = 100,
	    .top = 100 + WIN_HEIGHT,
	    .right = 100 + WIN_WIDTH,
	    .bottom = 100,
	    .handleMouseClickFunc = win_click,
	    .handleMouseWheelFunc = win_wheel,
	    .handleCursorFunc = win_cursor,
	    .drawWindowFunc = win_draw,
	    .decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle,
	    .layer = xplm_WindowLayerFloatingWindows,
	    .refcon = vis
	};
	XPLMCreateFlightLoop_t floop = {
	    .structSize = sizeof (floop),
	    .phase = xplm_FlightLoop_Phase_BeforeFlightModel,
	    .callbackFunc = vis_floop_cb,
	    .refcon = vis
	};

	ASSERT(sys != NULL);

	vis->sys = sys;
	vis->win = XPLMCreateWindowEx(&cr);
	ASSERT(vis->win != NULL);
	vis->pos_scale = pos_scale;
	vis->font_sz = font_sz;
	vis->zoom = 1;
	mutex_init(&vis->lock);
	vis->floop = XPLMCreateFlightLoop(&floop);

	XPLMSetWindowTitle(vis->win, "Electrical Network");
	XPLMSetWindowResizingLimits(vis->win, 200, 200, 100000, 100000);
	classic_win_center(vis->win);

	return (vis);
}

/**
 * Destroys a previously created libelec_vis_t. You must call this before
 * destroying the underlying elec_sys_t handle.
 */
void
libelec_vis_destroy(libelec_vis_t *vis)
{
	if (vis == NULL)
		return;

	if (vis->mtcr != NULL)
		mt_cairo_render_fini(vis->mtcr);
	mutex_destroy(&vis->lock);
	XPLMDestroyWindow(vis->win);
	XPLMDestroyFlightLoop(vis->floop);

	ZERO_FREE(vis);
}

/**
 * Sets the panning offset of the contents of the rendering. Initially
 * this is (0,0), but if your network layout is better centered on
 * another position, you can use this function to pre-pan the view to
 * that offset after creating the visualizer.
 * @param offset The visual offset to set in pixels.
 */
void
libelec_vis_set_offset(libelec_vis_t *vis, vect2_t offset)
{
	ASSERT(vis != NULL);
	vis->offset = offset;
}

/**
 * @return The current panning offset of the view.
 */
vect2_t
libelec_vis_get_offset(const libelec_vis_t *vis)
{
	ASSERT(vis != NULL);
	return (vis->offset);
}

/**
 * @return True if the visualizer window is open, false if it isn't.
 */
bool
libelec_vis_is_open(libelec_vis_t *vis)
{
	ASSERT(vis != NULL);
	ASSERT(vis->win != NULL);
	return (XPLMGetWindowIsVisible(vis->win));
}

/**
 * Opens the libelec_vis_t window. The window will automatically switch
 * to VR positioning mode, if the user is in VR (or return back to 2D
 * mode, if they have exited VR).
 */
void
libelec_vis_open(libelec_vis_t *vis)
{
	ASSERT(vis != NULL);
	ASSERT(vis->win != NULL);

	if (!XPLMGetWindowIsVisible(vis->win)) {
		recreate_mtcr(vis);
		mt_cairo_render_once_wait(vis->mtcr);
		XPLMSetWindowIsVisible(vis->win, true);
		XPLMScheduleFlightLoop(vis->floop, -1, true);
	}
	window_follow_VR(vis->win);
}

/**
 * Closes the libelec_vis_t window, if open, otherwise does nothing.
 * Closing the window stops the renderer thread and deinitializes the
 * renderer's framebuffer, so removes essentially all CPU, RAM and VRAM
 * footprint that the visualizer had while open. As such, you don't need
 * to immediately perform a libelec_vis_destroy() as soon as the window
 * is closed. You can delay that until simulator shutdown. This will
 * help preserve the window's position and size, if the user moved
 * and/or resized it.
 */
void
libelec_vis_close(libelec_vis_t *vis)
{
	ASSERT(vis != NULL);
	ASSERT(vis->win != NULL);
	XPLMSetWindowIsVisible(vis->win, false);
}
