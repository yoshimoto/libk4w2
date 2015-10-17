/**
 * @file   color_cuda.c
 * @author Hiromasa YOSHIMOTO
 * @date   Tue Jun  2 18:05:39 2015
 * 
 * @brief  
 * 
 * 
 */
#if defined(HAVE_GLEW)
#include <GL/glew.h>
#define CHECK_GL() do {							\
	GLenum e;							\
	while ( (e = glGetError()) != GL_NO_ERROR ) {			\
	    VERBOSE("glGetError() returns '%s (0x%X)'",			\
		    glewGetErrorString(e), e );				\
	}								\
    } while(0)
#else
#define CHECK_GL() (void)0
#endif

#include "module.h"

#if ! defined HAVE_LIBGPUJPEG
#  error "gpujpeg is not installed"
#else

#include <assert.h>
#include <libgpujpeg/gpujpeg.h>

typedef struct {
    unsigned int texture_id;
    struct gpujpeg_decoder_output output;
    struct gpujpeg_opengl_texture *texture;
} decoder_slot;

typedef struct {
    struct k4w2_decoder_ctx decoder; 

    struct gpujpeg_decoder* cuda;

    decoder_slot *slot;

} decoder_cuda;

static int
color_cuda_open(k4w2_decoder_t ctx, unsigned int type)
{
    decoder_cuda * d = (decoder_cuda *)ctx;

    if ( (type & K4W2_DECODER_TYPE_MASK) != K4W2_DECODER_COLOR)
	goto err;
    if ( type & K4W2_DECODER_DISABLE_CUDA )
	goto err;

    int flags = 0;
    if (k4w2_debug_level > 1)
	flags |= GPUJPEG_VERBOSE;
    if ( type & K4W2_DECODER_ENABLE_OPENGL )
	flags |= GPUJPEG_OPENGL_INTEROPERABILITY;

    /* 
     * Note; gpujpeg_init_device() will call cudaGLSetGLDevice() when
     * GPUJPEG_OPENGL_INTEROPERABILITY is set.
     * If you want to use gpujpeg with OpenCL/OpenGL,
     * OpenCL/OpenGL context must be initialized and attacted before.
     */
    gpujpeg_init_device(0, flags);

    d->cuda = gpujpeg_decoder_create();
    if (!d->cuda) {
	VERBOSE("gpujpeg_decoder_create() failed.");
	goto err;
    }

    assert(1 <= ctx->num_slot);
    d->slot = (decoder_slot*) malloc (sizeof(decoder_slot) * ctx->num_slot);
    if (type & K4W2_DECODER_ENABLE_OPENGL) {
	for (size_t s = 0 ; s < ctx->num_slot; ++s) {
	    d->slot[s].texture_id = gpujpeg_opengl_texture_create(1920, 1080, NULL);
	    d->slot[s].texture    =
		gpujpeg_opengl_texture_register(d->slot[s].texture_id,
						GPUJPEG_OPENGL_TEXTURE_WRITE);
	}
	CHECK_GL();
    }

    if (type & K4W2_DECODER_ENABLE_OPENGL) {
	for (size_t s = 0 ; s < ctx->num_slot; ++s) {
	    gpujpeg_decoder_output_set_texture(&d->slot[s].output,
					       d->slot[s].texture);
	}
	CHECK_GL();
    } else {
	for (size_t s = 0 ; s < ctx->num_slot; ++s) {
	    gpujpeg_decoder_output_set_default(&d->slot[s].output);
	}
    }

    return K4W2_SUCCESS;
err:
    return K4W2_ERROR;
}


static int
color_cuda_request(k4w2_decoder_t ctx, int slot, const void *src, int src_length)
{
    decoder_cuda * d = (decoder_cuda *)ctx;
    struct kinect2_color_header* h = (struct kinect2_color_header*)src;
    const size_t s = slot % ctx->num_slot;

    if (d->slot[s].texture_id) {
	gpujpeg_decoder_decode(d->cuda, h->image, src_length, &d->slot[s].output);
    } else {
	gpujpeg_decoder_request(d->cuda,
				h->image,
				src_length);
    }
    return K4W2_SUCCESS;
}

static int
color_cuda_fetch(k4w2_decoder_t ctx, int slot, void *dst, int dst_length)
{
    decoder_cuda * d = (decoder_cuda *)ctx;
    const size_t s = slot % ctx->num_slot;
    if (d->slot[s].texture_id) {
    } else {
	gpujpeg_decoder_fetch(d->cuda, &d->slot[s].output);
    }
    memcpy(dst, d->slot[s].output.data, dst_length);

    return K4W2_SUCCESS;
}

static int
color_cuda_get_gl_texture(k4w2_decoder_t ctx, int slot, unsigned int options, unsigned int *texturename)
{
    decoder_cuda * d = (decoder_cuda *)ctx;
    const size_t s = slot % ctx->num_slot;
    *texturename = d->slot[s].texture_id;
    return K4W2_SUCCESS;
}

static int
color_cuda_close(k4w2_decoder_t ctx)
{
    decoder_cuda * d = (decoder_cuda *)ctx;
    if (d && d->cuda) {
	gpujpeg_decoder_destroy(d->cuda);
	d->cuda = NULL;
    }
    return K4W2_SUCCESS;
}

static const k4w2_decoder_ops ops = {
    .open	= color_cuda_open,
    .set_params = NULL,
    .request	= color_cuda_request,
    .get_gl_texture = color_cuda_get_gl_texture,
    .fetch	= color_cuda_fetch,
    .close	= color_cuda_close,
};

REGISTER_MODULE(k4w2_decoder_color_cuda_init)
{
    k4w2_register_decoder("color cuda", &ops, sizeof(decoder_cuda));
}


#endif /* #if ! defined HAVE_GPUJPEG */


/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */

