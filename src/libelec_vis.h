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

#ifndef	__LIBELEC_VIS_H__
#define	__LIBELEC_VIS_H__

#include <stdbool.h>

#include <acfutils/mt_cairo_render.h>

#include "libelec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct libelec_vis_s libelec_vis_t;

libelec_vis_t *libelec_vis_new(elec_sys_t *sys, double pos_scale,
    double font_sz, mt_cairo_uploader_t *mtul);
void libelec_vis_destroy(libelec_vis_t *vis);

bool libelec_vis_is_open(libelec_vis_t *vis);
void libelec_vis_open(libelec_vis_t *vis);

#ifdef __cplusplus
}
#endif

#endif	/* __LIBELEC_VIS_H__ */
