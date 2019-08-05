/*
 * CONFIDENTIAL
 *
 * Copyright 2019 Saso Kiselkov. All rights reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property
 * of Saso Kiselkov. The intellectual and technical concepts contained
 * herein are proprietary to Saso Kiselkov and may be covered by U.S. and
 * Foreign Patents, patents in process, and are protected by trade secret
 * or copyright law. Dissemination of this information or reproduction of
 * this material is strictly forbidden unless prior written permission is
 * obtained from Saso Kiselkov.
 */

#ifndef	_LIBELEC_TEST_H_
#define	_LIBELEC_TEST_H_

#include "libelec.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CAUTION: the actual values of the source or upstream component can
 * change at any time after the call, or the functions can even return
 * NULL. DO NOT use these for anything other than debug reporting!
 */
elec_comp_t *libelec_comp_get_src(const elec_comp_t *comp);
elec_comp_t *libelec_comp_get_upstream(const elec_comp_t *comp);

#ifdef __cplusplus
}
#endif

#endif	/* _LIBELEC_TEST_H_ */
