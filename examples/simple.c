/**
 * @file   simple.c
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 01:18:41 2015
 * 
 * @brief  
 * 
 * 
 */

#include "libk4w2/libk4w2.h"
#include <stdio.h>
#include <unistd.h> /* sleep(3) */
#include <stdlib.h> /* exit() */

#define ABORT(fmt, ...) do { fprintf(stderr, fmt "\n", ## __VA_ARGS__); exit(EXIT_FAILURE); } while(0)


void color_cb(const void *buffer, int length, void *userdata)
{
    /* const struct kinect2_color_header *h = (const struct kinect2_color_header*)buffer; */
    const struct kinect2_color_footer *f = KINECT2_GET_COLOR_FOOTER(buffer, length);

    fprintf(stderr, "color: sequence:%10d timestamp:%10d\n", f->sequence, f->timestamp);
}

void depth_cb(const void *buffer, int length, void *userdata)
{
    const struct kinect2_depth_footer *f = KINECT2_GET_DEPTH_FOOTER(buffer);

    fprintf(stderr, "depth: sequence:%10d timestamp:%10d\n", f->sequence, f->timestamp);
}

int
main()
{
    /* Opens kinect device by using default driver with default setting */
    k4w2_t ctx = k4w2_open(0, 0);
    if (!ctx) {
	ABORT("failed to open kinect device.");
    }

    /* Sets callback functions that will be called when new image is available */
    k4w2_set_color_callback(ctx, color_cb, NULL);
    k4w2_set_depth_callback(ctx, depth_cb, NULL);

    /* Starts kinect device */
    k4w2_start(ctx);

    /* Waits for 10 seconds */
    sleep( 10 );

    
    k4w2_stop(ctx);
    k4w2_close(&ctx);

    return 0;
}
