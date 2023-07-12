/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */
/**
 * \file
 * This file contains drawing routines utilizing the cairo graphics API,
 * which let you draw a picture of the network layout into a
 * `cairo_surface_t`. If you are using X-Plane, see `libelec_vis.h` for
 * a fully functional visualizer, with mouse dragging & zoom support, as
 * well as X-Plane window system integration. If you want to use the
 * library outside of X-Plane, the functions contained in this file will
 * give you the same visual style as `libelec_vis.h`, but you will need
 * to supply your own windowing, compositing and mouse interaction
 * implementations.
 *
 * @see libelec_draw_layout
 * @see libelec_draw_comp_info
 */

#ifndef	__LIBELEC_DRAWING_H__
#define	__LIBELEC_DRAWING_H__

#include <cairo.h>

#include "libelec.h"

#ifdef __cplusplus
extern "C" {
#endif

void libelec_draw_layout(const elec_sys_t *sys, cairo_t *cr, double pos_scale,
    double font_sz);
void libelec_draw_comp_info(const elec_comp_t *comp, cairo_t *cr,
    double pos_scale, double font_sz, vect2_t pos);

#ifdef __cplusplus
}
#endif

#endif	/* __LIBELEC_DRAWING_H__ */
