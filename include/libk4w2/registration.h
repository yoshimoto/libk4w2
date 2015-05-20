/**
 * @file   registration.h
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 22:16:20 2015
 * 
 * @brief  Image registration
 * 
 * 
 */

#ifndef __LIBK4W2_REGISTRATION_H_INCLUDED__
#define __LIBK4W2_REGISTRATION_H_INCLUDED__

#include "libk4w2/libk4w2.h"

#ifdef __cplusplus
#  define EXTERN_C_BEGIN extern "C" {
#  define EXTERN_C_END   }
#else
#  define EXTERN_C_BEGIN
#  define EXTERN_C_END
#endif

EXTERN_C_BEGIN

typedef struct k4w2_registration * k4w2_registration_t;

k4w2_registration_t k4w2_registration_create_from_dir(const char *dirname);
k4w2_registration_t k4w2_registration_create_from_ctx(k4w2_t ctx);

k4w2_registration_t k4w2_registration_create(struct kinect2_color_camera_param *color,
					     struct kinect2_depth_camera_param *depth);
void k4w2_registration_release(k4w2_registration_t *registration);
void k4w2_registration_depth_to_color(k4w2_registration_t registration,
				      int dx, int dy, float dz,
				      float *cx, float *cy);

EXTERN_C_END

#undef EXTERN_C_BEGIN
#undef EXTERN_C_END

#endif /* #ifndef __LIBK4W2_REGISTRATION_H_INCLUDED__ */


/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
