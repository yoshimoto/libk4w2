/**
 * @file   color_cuda.c
 * @author Hiromasa YOSHIMOTO
 * @date   Tue Jun  2 18:05:39 2015
 * 
 * @brief  
 * 
 * 
 */

#include "module.h"

#if ! defined HAVE_LIBGPUJPEG
#  error "gpujpeg is not installed"
#else

#include <libgpujpeg/gpujpeg.h>

typedef struct {
    struct k4w2_decoder_ctx decoder; 

    struct gpujpeg_decoder* cuda;
    struct gpujpeg_decoder_output output;
} decoder_cuda;

static int
color_cuda_open(k4w2_decoder_t ctx, unsigned int type)
{
    decoder_cuda * d = (decoder_cuda *)ctx;

    if ( (type & K4W2_DECODER_TYPE_MASK) != K4W2_DECODER_COLOR)
	goto err;
    if ( type & K4W2_DECODER_DISABLE_CUDA )
	goto err;

    int flags = GPUJPEG_VERBOSE;

    if ( type & K4W2_DECODER_USE_OPENGL )
	flags |= GPUJPEG_OPENGL_INTEROPERABILITY;

    gpujpeg_init_device(0, flags);

    d->cuda = gpujpeg_decoder_create();
    if (!d->cuda) {
	VERBOSE("gpujpeg_decoder_create() failed.");
	goto err;
    }

/*    if (0) {
	struct gpujpeg_parameters param_coder;
	struct gpujpeg_image_parameters param_image;
	gpujpeg_set_default_parameters(&param_coder);
	gpujpeg_image_set_default_parameters(&param_image);

	//param_image.width  = 1920;
	//param_image.height = 1080;
	//param_image.comp_count = 3;

	if (gpujpeg_decoder_init(d->cuda, &param_coder, &param_image)) {
	    VERBOSE("gpujpeg_decoder_init() failed.");
	    goto err;
	}
	}*/
    gpujpeg_decoder_output_set_default(&d->output);

    return K4W2_SUCCESS;
err:
    return K4W2_ERROR;
}


static int
color_cuda_request(k4w2_decoder_t ctx, int slot, const void *src, int src_length)
{
    decoder_cuda * d = (decoder_cuda *)ctx;
    struct kinect2_color_header* h = (struct kinect2_color_header*)src;

    gpujpeg_decoder_request(d->cuda,
			    h->image,
			    src_length);
    return K4W2_SUCCESS;
}

/*
static int
color_cuda_wait(k4w2_decoder_t ctx, int slot)
{
    decoder_cuda * d = (decoder_cuda *)ctx;
    return K4W2_SUCCESS;
}
*/

static int
color_cuda_fetch(k4w2_decoder_t ctx, int slot, void *dst, int dst_length)
{
    decoder_cuda * d = (decoder_cuda *)ctx;
    gpujpeg_decoder_fetch(d->cuda, &d->output);
    memcpy(dst, d->output.data, dst_length);

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
/*    .wait	= color_cuda_wait,*/
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

