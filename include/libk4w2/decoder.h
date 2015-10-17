/**
 * @file   decoder.h
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 16:18:04 2015
 * 
 * @brief  
 * 
 * 
 */

#ifndef __LIBK42W_DECODER_H_INCLUDED__
#define __LIBK42W_DECODER_H_INCLUDED__

#include "libk4w2/libk4w2.h"

#ifdef __cplusplus
#  define EXTERN_C_BEGIN extern "C" {
#  define EXTERN_C_END   }
#else
#  define EXTERN_C_BEGIN
#  define EXTERN_C_END
#endif

EXTERN_C_BEGIN

typedef struct k4w2_decoder_ctx * k4w2_decoder_t;

#define K4W2_DECODER_COLOR     0
#define K4W2_DECODER_DEPTH     1
#define K4W2_DECODER_TYPE_MASK 0x0f
#define K4W2_DECODER_DISABLE_OPENCL (1<<5)
#define K4W2_DECODER_DISABLE_CUDA   (1<<6)
/* Enables OpenGL Interoperability */
#define K4W2_DECODER_ENABLE_OPENGL  (1<<7)

k4w2_decoder_t k4w2_decoder_open(unsigned int type, int num_slot);
int k4w2_decoder_set_params(k4w2_decoder_t ctx,
			    struct kinect2_color_camera_param *,
			    struct kinect2_depth_camera_param *,
			    struct kinect2_p0table *);
int k4w2_decoder_request(k4w2_decoder_t ctx, int slot, const void *src, int src_length);
int k4w2_decoder_wait(k4w2_decoder_t ctx, int slot);
int k4w2_decoder_fetch(k4w2_decoder_t ctx, int slot, void *dst, int dst_length);
void k4w2_decoder_close(k4w2_decoder_t *ctx);

int k4w2_decoder_get_gl_texture(k4w2_decoder_t ctx, int slot, unsigned int option,
				unsigned int *texturename);

EXTERN_C_END

#undef EXTERN_C_BEGIN
#undef EXTERN_C_END

#endif /* #ifndef __LIBK42W_DECODER_H_INCLUDED__ */
/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
