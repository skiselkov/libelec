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

#ifndef	__LIBELEC_DRAWING_H__
#define	__LIBELEC_DRAWING_H__

#include <acfutils/mt_cairo_render.h>

#include "libelec.h"

#ifdef __cplusplus
extern "C" {
#endif

void libelec_draw_layout(const elec_sys_t *sys, cairo_t *cr, double pos_scale,
    double font_sz);

#ifdef __cplusplus
}
#endif

#endif	/* __LIBELEC_DRAWING_H__ */
