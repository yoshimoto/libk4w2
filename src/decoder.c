/**
 * @file   decoder.c
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 16:33:50 2015
 * 
 * @brief  
 * 
 * 
 */

#include "module.h"

#include <assert.h>
#include <stdlib.h> /* malloc() */
#include <string.h> /* memset() */


#define CHECK(ctx) \
    do { if (!(ctx)) { VERBOSE("wrong decoder"); return K4W2_ERROR; } } while (0)

typedef struct {
    const char *name;
    const k4w2_decoder_ops *ops;
    int ctx_size;
} decoder_entry;
static decoder_entry decoder[16] = {{NULL,NULL,-1}};
static int num_decoder = 0;
static MUTEX_T decoder_mutex = MUTEX_INITIALIZER;

void
k4w2_register_decoder(const char *name,
		      const k4w2_decoder_ops *ops,
		      int ctx_size)
{
    int i;
    i = num_decoder++;
    decoder[i].name = name;
    decoder[i].ops  = ops;
    decoder[i].ctx_size = ctx_size;
}

k4w2_decoder_t
allocate_decoder(const k4w2_decoder_ops *ops, int ctx_size)
{
    k4w2_decoder_t ctx = (k4w2_decoder_t)malloc(ctx_size);

    assert((size_t)ctx_size >= sizeof(k4w2_decoder_t));

    memset(ctx, 0, sizeof(ctx_size));
    ctx->ops = ops;

    return ctx;
}

k4w2_decoder_t
k4w2_decoder_open(unsigned int type, int num_slot)
{
    int i = 0;
    static int firsttime = 1;
    k4w2_decoder_t ctx = NULL;
    MUTEX_LOCK(&decoder_mutex);

    if (firsttime) {
	INITIALIZE_MODULE(k4w2_decoder_depth_cl_init);
	INITIALIZE_MODULE(k4w2_decoder_depth_cpu_init);
	INITIALIZE_MODULE(k4w2_decoder_color_cuda_init);
	INITIALIZE_MODULE(k4w2_decoder_color_cpu_init);

	if (getenv("LIBK4W2_VERBOSE")) {
	    k4w2_debug_level = atoi(getenv("LIBK4W2_VERBOSE"));
	}

	firsttime = 0;
    }

    for (i = 0; i<num_decoder; ++i) {
	assert(decoder[i].ops);
	assert(decoder[i].ctx_size >= 0);
	ctx = allocate_decoder(decoder[i].ops, decoder[i].ctx_size);
	if (!ctx)
	    continue;

	ctx->num_slot =  num_slot;

	if (!ctx->ops->open) {
	    VERBOSE("internal error; open() is not implemented.");
	} else if (K4W2_SUCCESS == ctx->ops->open(ctx, type)) {
	    VERBOSE("%s decoder is selected.", decoder[i].name);
	    goto exit;
	} else {
	    VERBOSE("%s decoder is skipped.", decoder[i].name);
	}

	free(ctx);
	ctx = NULL;
    }

exit:
    MUTEX_UNLOCK(&decoder_mutex);
    return ctx;
}


int
k4w2_decoder_set_params(k4w2_decoder_t ctx,
			struct kinect2_color_camera_param * color,
			struct kinect2_depth_camera_param * depth,
			struct kinect2_p0table * p0table)
{
    CHECK(ctx);
    if (!ctx->ops->set_params) {
	VERBOSE("not implemented");
	return K4W2_DECODER_DEPTH;
    }

    return ctx->ops->set_params(ctx, color, depth, p0table);
}


int
k4w2_decoder_request(k4w2_decoder_t ctx, int slot, const void *src, int src_length)
{
    CHECK(ctx);
    return ctx->ops->request(ctx, slot, src, src_length);
}

int
k4w2_decoder_wait(k4w2_decoder_t ctx, int slot)
{
    CHECK(ctx);
    return ctx->ops->wait(ctx, slot);
}

int
k4w2_decoder_fetch(k4w2_decoder_t ctx, int slot, void *dst, int dst_length)
{
    CHECK(ctx);
    return ctx->ops->fetch(ctx, slot, dst, dst_length);
}

void
k4w2_decoder_close(k4w2_decoder_t *ctx)
{
    if(!ctx)
	return;
    if (*ctx) {
	(*ctx)->ops->close(*ctx);
	*ctx = 0;
    }
}

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
