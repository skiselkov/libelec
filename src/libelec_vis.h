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
 * This file contains a simple visualizer for libelec networks. This is
 * using X-Plane's XPLM SDK for window handling, compositing and mouse
 * interaction.
 *
 * To create a new visualizer, use libelec_vis_new(), passing it a
 * handle to an initialized and started elec_sys_t. On shutdown,
 * you must call libelec_vis_destroy() before stopping and destroying
 * the referenced electrical system. You can open the window using
 * libelec_vis_open() if it was closed by the user (use
 * libelec_vis_is_open() to check).
 *
 * @note If you plan on running libelec in a simulator other than X-Plane,
 * or outside of a simulator entirely, you can still use the same
 * visualizations by using the functions from `libelec_drawing.h` and
 * implementing your own window handling and interaction layers.
 *
 * @see libelec_vis_new()
 * @see libelec_vis_destroy()
 * @see libelec_vis_open()
 * @see libelec_vis_is_open()
 */

#ifndef	__LIBELEC_VIS_H__
#define	__LIBELEC_VIS_H__

#include <stdbool.h>

#include <acfutils/mt_cairo_render.h>

#include "libelec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct libelec_vis_s libelec_vis_t;

libelec_vis_t *libelec_vis_new(const elec_sys_t *sys, double pos_scale,
    double font_sz);
void libelec_vis_destroy(libelec_vis_t *vis);

bool libelec_vis_is_open(libelec_vis_t *vis);
void libelec_vis_open(libelec_vis_t *vis);
void libelec_vis_close(libelec_vis_t *vis);

void libelec_vis_set_offset(libelec_vis_t *vis, vect2_t offset);
vect2_t libelec_vis_get_offset(const libelec_vis_t *vis);

#ifdef __cplusplus
}
#endif

#endif	/* __LIBELEC_VIS_H__ */
