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
};

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

	cairo_translate(cr, vis->offset.x, vis->offset.y);
	cairo_scale(cr, vis->zoom, vis->zoom);
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_paint(cr);
	libelec_draw_layout(vis->sys, cr, vis->pos_scale, vis->font_sz);
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
	int left, top;

	libelec_vis_t* vis;
	vect2_t off;

	ASSERT(win != NULL);
	ASSERT(refcon != NULL);
	vis = refcon;

	XPLMGetWindowGeometry(vis->win, &left, &top, NULL, NULL);
	x -= left;
	y = top - y;

	if (mouse == xplm_MouseDown)
		vis->mouse_down = VECT2(x, y);
	off = vect2_sub(VECT2(x, y), vis->mouse_down);
	vis->offset = vect2_add(vis->offset, off);
	vis->mouse_down = VECT2(x, y);

	/* Increase rendering rate while dragging */
	if (mouse == xplm_MouseDown || mouse == xplm_MouseDrag) {
		if (mt_cairo_render_get_fps(vis->mtcr) != WIN_FPS_FAST)
			mt_cairo_render_set_fps(vis->mtcr, WIN_FPS_FAST);
	} else {
		mt_cairo_render_set_fps(vis->mtcr, WIN_FPS);
	}

	return (1);
}

static int
win_wheel(XPLMWindowID win, int x, int y, int wheel, int clicks, void *refcon)
{
	libelec_vis_t *vis;

	ASSERT(win != NULL);
	UNUSED(x);
	UNUSED(y);
	ASSERT(refcon != NULL);
	vis = refcon;

	if (wheel != 0)
		return (1);

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

	recreate_mtcr(vis);

	return (vis);
}

void
libelec_vis_destroy(libelec_vis_t *vis)
{
	if (vis == NULL)
		return;

	mt_cairo_render_fini(vis->mtcr);
	XPLMDestroyWindow(vis->win);

	free(vis);
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
