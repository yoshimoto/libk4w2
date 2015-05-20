/**
 * @file   decoder_color_cpu.c
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 16:36:27 2015
 * 
 * @brief  
 * 
 * 
 */

#include "module.h"

#if !defined HAVE_TURBOJPEG
#   error "turbojpeg is not installed"
#else

#include <turbojpeg.h>

typedef struct {
    struct k4w2_decoder_ctx decoder; 
    tjhandle tj;
    unsigned char **buf;
} decoder_tj;

static int
color_tj_open(k4w2_decoder_t ctx, unsigned int type)
{
    decoder_tj * d = (decoder_tj *)ctx;

    if (type != K4W2_DECODER_COLOR)
	goto err;

    d->buf = allocate_bufs(ctx->num_slot, 1920 * 1080 * 3);
    if (!d->buf)
	goto err;
    d->tj = tjInitDecompress();

    return K4W2_SUCCESS;
err:
    free_bufs(d->buf);
    d->buf = 0;
    return K4W2_ERROR;
}


static int
color_tj_request(k4w2_decoder_t ctx, int slot, const void *src, int src_length)
{
    decoder_tj * d = (decoder_tj *)ctx;
    struct kinect2_color_header* h = (struct kinect2_color_header*)src;
    int res;
    res = tjDecompress2(d->tj,
			h->image,
			src_length,
			d->buf[slot],
			1920, 1920 *3, 1080,
			TJPF_BGR, TJFLAG_FASTDCT);
    return (0==res)?K4W2_SUCCESS:K4W2_ERROR;
}

/*
static int
color_tj_wait(k4w2_decoder_t ctx, int slot)
{
    decoder_tj * d = (decoder_tj *)ctx;
    return K4W2_SUCCESS;
}
*/

static int
color_tj_fetch(k4w2_decoder_t ctx, int slot, void *dst, int dst_length)
{
    decoder_tj * d = (decoder_tj *)ctx;
    memcpy(dst, d->buf[slot], dst_length);
    return K4W2_SUCCESS;
}

static int
color_tj_close(k4w2_decoder_t ctx)
{
    decoder_tj * d = (decoder_tj *)ctx;
    free_bufs(d->buf);
    d->buf = 0;

    return K4W2_SUCCESS;
}

static const k4w2_decoder_ops ops = {
    .open	= color_tj_open,
    .set_params = NULL,
    .request	= color_tj_request,
/*    .wait	= color_tj_wait,*/
    .fetch	= color_tj_fetch,
    .close	= color_tj_close,
};

REGISTER_MODULE(k4w2_decoder_color_cpu_init)
{
    k4w2_register_decoder("color cpu", &ops, sizeof(decoder_tj));
}
#endif /* #if defined HAVE_TURBOJPEG */

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
