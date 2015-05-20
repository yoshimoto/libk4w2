/**
 * @file   libk4w2.h
 * @author Hiromasa YOSHIMOTO
 * @date   Thu Apr 23 23:33:03 2015
 * 
 * @brief  library for kinect for windows 2
 * 
 * 
 */

#ifndef __LIBK4W2_LIBK4W2_H_INCLUDED__
#define __LIBK4W2_LIBK4W2_H_INCLUDED__

#define LIBK4W2_API_VERSION 20150423

#include "libk4w2/kinect2.h"

#ifdef __cplusplus
#  define EXTERN_C_BEGIN extern "C" {
#  define EXTERN_C_END   }
#else
#  define EXTERN_C_BEGIN
#  define EXTERN_C_END
#endif

EXTERN_C_BEGIN

#define K4W2_SUCCESS 0		/**< success */
#define K4W2_ERROR   1		/**< error */

/** handle for a k4w2 device */
typedef struct k4w2_driver_ctx * k4w2_t;

/* flags for k4w2_open() */
#define K4W2_DEFAULT        0       /**< use default configration */

#define K4W2_DISABLE_COLOR (1<<1)   /**< disable color stream */
#define K4W2_DISABLE_DEPTH (1<<2)   /**< disable depth stream */
#define K4W2_DISABLE_V4L2   (1<<17) /**< disable v4l2 driver   */
#define K4W2_DISABLE_LIBUSB (1<<16) /**< disable libusb driver */

k4w2_t k4w2_open(unsigned int deviceid, unsigned int flags);

typedef void (*k4w2_callback_t)(const void *buffer, int length, void *userdata);
int k4w2_set_color_callback(k4w2_t ctx,
			    k4w2_callback_t callback,
			    void *userdata);
int k4w2_set_depth_callback(k4w2_t ctx,
			    k4w2_callback_t callback,
			    void *userdata);

int k4w2_start(k4w2_t ctx);
int k4w2_stop(k4w2_t ctx);
void k4w2_close(k4w2_t *ctx);

int k4w2_read_color_camera_param(k4w2_t ctx,
				 struct kinect2_color_camera_param *param);
int k4w2_read_depth_camera_param(k4w2_t ctx,
				 struct kinect2_depth_camera_param *param);
int k4w2_read_p0table(k4w2_t ctx,
		      struct kinect2_p0table *p0table);
int k4w2_read_version_string(k4w2_t ctx,
			     char *buf, int length);

int k4w2_set_debug_level(int newlevel);


int k4w2_camera_params_load(const char *dirname,
			    struct kinect2_color_camera_param *color,
			    struct kinect2_depth_camera_param *depth,
			    struct kinect2_p0table *p0table);
int k4w2_camera_params_save(struct kinect2_color_camera_param *color,
			    struct kinect2_depth_camera_param *depth,
			    struct kinect2_p0table *p0table,
			    const char *dirname);

EXTERN_C_END

#undef EXTERN_C_BEGIN
#undef EXTERN_C_END

#endif /* #ifndef __LIBK4W2_LIBK4W2_H_INCLUDED__ */

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
