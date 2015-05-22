/**
 * @file   driver.c
 * @author Hiromasa YOSHIMOTO
 * @date   Thu Apr 23 23:32:31 2015
 * 
 * @brief  
 * 
 * 
 */

#include "module.h"

#include <assert.h>
#include <stdlib.h> /* malloc(), getenv() */
#include <string.h> /* memset() */

#define CHECK(ctx) \
    do { if (!(ctx)) { VERBOSE("wrong ctx"); return K4W2_ERROR; } } while (0)

int k4w2_debug_level = 0;

int
k4w2_set_debug_level(int newlevel)
{
    int oldlevel = k4w2_debug_level;
    k4w2_debug_level = newlevel;
    return oldlevel;
}

typedef struct {
    const char *name;
    const k4w2_driver_ops *ops;
    int ctx_size;
} driver_entry;
static driver_entry drivers[16] = {{NULL,NULL,-1}};
static int num_drivers = 0;
static MUTEX_T driver_mutex = MUTEX_INITIALIZER;

void
k4w2_register_driver(const char *name,
		     const k4w2_driver_ops *ops,
		     int ctx_size)
{
    int i;
    i = num_drivers++;
    drivers[i].name = name;
    drivers[i].ops  = ops;
    drivers[i].ctx_size = ctx_size;
}

k4w2_t
allocate_driver(const k4w2_driver_ops *ops, int ctx_size)
{
    k4w2_t ctx = (k4w2_t)malloc(ctx_size);

    assert((size_t)ctx_size >= sizeof(k4w2_t));

    memset(ctx, 0, sizeof(ctx_size));
    ctx->ops = ops;

    return ctx;
}

/** 
 * 
 * 
 * @param deviceid 
 * @param flags 
 * 
 * @return 
 *
 * @note this function is threaded-safe.
 */
k4w2_t
k4w2_open(unsigned int deviceid, unsigned int flags)
{
    int i = 0;
    static int firsttime = 1;
    k4w2_t ctx = NULL;
    MUTEX_LOCK(&driver_mutex);

    if (firsttime) {
#if defined WITH_V4L2
	INITIALIZE_MODULE(k4w2_driver_v4l2_init);
#endif
#if defined WITH_LIBUSB
	INITIALIZE_MODULE(k4w2_driver_libusb_init);
#endif
	if (getenv("LIBK4W2_VERBOSE")) {
	    k4w2_debug_level = atoi(getenv("LIBK4W2_VERBOSE"));
	}

	firsttime = 0;
    }

    for (i = 0; i<num_drivers; ++i) {
	assert(drivers[i].ops);
	assert(drivers[i].ctx_size >= 0);
	ctx = allocate_driver(drivers[i].ops, drivers[i].ctx_size);
	if (!ctx)
	    continue;

	ctx->begin = (flags & K4W2_DISABLE_COLOR)?DEPTH_CH:COLOR_CH;
	ctx->end   = (flags & K4W2_DISABLE_DEPTH)?COLOR_CH:DEPTH_CH;
	if (!ctx->ops->open) {
	    VERBOSE("internal error; open() is not implemented.");
	} else if (K4W2_SUCCESS == ctx->ops->open(ctx, deviceid, flags)) {
	    VERBOSE("%s driver is selected.", drivers[i].name);
	    goto exit;
	}

	free(ctx);
	ctx = NULL;
    }

exit:
    MUTEX_UNLOCK(&driver_mutex);
    return ctx;
}

int
k4w2_set_color_callback(k4w2_t ctx,
			k4w2_callback_t callback,
			void *userdata)
{
    CHECK(ctx);
    ctx->callback[COLOR_CH] = callback;
    ctx->userdata[COLOR_CH] = userdata;
    return K4W2_SUCCESS;
}

int
k4w2_set_depth_callback(k4w2_t ctx,
			k4w2_callback_t callback,
			void *userdata)
{
    CHECK(ctx);
    ctx->callback[DEPTH_CH] = callback;
    ctx->userdata[DEPTH_CH] = userdata;
    return K4W2_SUCCESS;
}

int
k4w2_start(k4w2_t ctx)
{
    CHECK(ctx);
    return ctx->ops->start(ctx);
}

int
k4w2_stop(k4w2_t ctx)
{
    CHECK(ctx);
    return ctx->ops->stop(ctx);
}

void
k4w2_close(k4w2_t *ctx)
{
    if (!ctx)
	return;
    if (*ctx) {
	(*ctx)->ops->close(*ctx);
	*ctx = 0;
    }
}

int
k4w2_read_color_camera_param(k4w2_t ctx,
			     struct kinect2_color_camera_param *param)
{
    CHECK(ctx);
    return ctx->ops->read_param(ctx,COLOR_PARAM, param,sizeof(*param));
}

int
k4w2_read_depth_camera_param(k4w2_t ctx,
			     struct kinect2_depth_camera_param *param)
{
    CHECK(ctx);
    return ctx->ops->read_param(ctx,DEPTH_PARAM, param,sizeof(*param));
}

int
k4w2_read_p0table(k4w2_t ctx,
		  struct kinect2_p0table *p0table)
{
    CHECK(ctx);
    return ctx->ops->read_param(ctx, P0TABLE, p0table, sizeof(*p0table));
}

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
