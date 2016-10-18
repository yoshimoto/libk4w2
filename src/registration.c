/**
 * @file   registration.c
 * @author Hiromasa YOSHIMOTO
 * @date   Thu Apr 30 11:21:43 2015
 * 
 * @brief  
 * 
 * 
 */

#include <math.h>
#include <assert.h>
#include <stdlib.h> // malloc
#include <string.h>

#include "libk4w2/registration.h"
#include "module.h"
/*
 * See https://github.com/OpenKinect/libfreenect2/issues/41
 */

/* these seem to be hardcoded in the original SDK */
static const float depth_q = 0.01;
static const float color_q = 0.002199;

struct k4w2_registration {
    struct kinect2_depth_camera_param depth;
    struct kinect2_color_camera_param color;

    float undistort_map[512][424][2];
    float depth_to_color_map[512][424][2];
};


static inline void
distort_depth(k4w2_registration_t reg, int mx, int my, float* x, float* y)
{
    // see http://en.wikipedia.org/wiki/Distortion_(optics) for description
    double dx = ((double)mx - reg->depth.cx) / reg->depth.fx;
    double dy = ((double)my - reg->depth.cy) / reg->depth.fy;
    double dx2 = dx * dx;
    double dy2 = dy * dy;
    double r2 = dx2 + dy2;
    double dxdy2 = 2 * dx * dy;
    double kr = 1 + ((reg->depth.k3 * r2 + reg->depth.k2) * r2 + reg->depth.k1) * r2;
    *x = reg->depth.fx * (dx * kr + reg->depth.p2 * (r2 + 2 * dx2) + reg->depth.p1 * dxdy2) + reg->depth.cx;
    *y = reg->depth.fy * (dy * kr + reg->depth.p1 * (r2 + 2 * dy2) + reg->depth.p2 * dxdy2) + reg->depth.cy;
}

static inline void
depth_to_color(k4w2_registration_t reg, float mx, float my, float* rx, float* ry)
{
    mx = (mx - reg->depth.cx) * depth_q;
    my = (my - reg->depth.cy) * depth_q;

    float wx =
	(mx * mx * mx * reg->color.mx_x3y0) + (my * my * my * reg->color.mx_x0y3) +
	(mx * mx * my * reg->color.mx_x2y1) + (my * my * mx * reg->color.mx_x1y2) +
	(mx * mx * reg->color.mx_x2y0) + (my * my * reg->color.mx_x0y2) + (mx * my * reg->color.mx_x1y1) +
	(mx * reg->color.mx_x1y0) + (my * reg->color.mx_x0y1) + (reg->color.mx_x0y0);

    float wy =
	(mx * mx * mx * reg->color.my_x3y0) + (my * my * my * reg->color.my_x0y3) +
	(mx * mx * my * reg->color.my_x2y1) + (my * my * mx * reg->color.my_x1y2) +
	(mx * mx * reg->color.my_x2y0) + (my * my * reg->color.my_x0y2) + (mx * my * reg->color.my_x1y1) +
	(mx * reg->color.my_x1y0) + (my * reg->color.my_x0y1) + (reg->color.my_x0y0);

    *rx = (wx / (reg->color.f * color_q)) - (reg->color.shift_m / reg->color.shift_d);
    *ry = (wy / color_q) + reg->color.cy;
}

void
k4w2_registration_depth_to_color(k4w2_registration_t reg,
				 int dx, int dy, float dz,
				 float *cx, float *cy)
{
    float rx = reg->depth_to_color_map[dx][dy][0];
    *cy = reg->depth_to_color_map[dx][dy][1];

    rx += reg->color.shift_m / dz;
    *cx = rx * reg->color.f + reg->color.cx;
}


k4w2_registration_t
k4w2_registration_create_from_dir(const char *dirname)
{
    struct kinect2_color_camera_param color_param;
    struct kinect2_depth_camera_param depth_param;
    int r;
    r = k4w2_camera_params_load(dirname,
				&color_param,
				&depth_param,
				NULL);
    if (K4W2_SUCCESS != r) {
	VERBOSE("camera_param_load(%s)", dirname);
	return NULL;
    }
    return k4w2_registration_create(&color_param, &depth_param);
}

k4w2_registration_t
k4w2_registration_create_from_ctx(k4w2_t ctx)
{
    struct kinect2_color_camera_param color_param;
    struct kinect2_depth_camera_param depth_param;
    int r;
    r = k4w2_read_color_camera_param(ctx, &color_param);
    if (K4W2_SUCCESS != r) {
	VERBOSE("read_color_camera_param() failed");
	return NULL;
    }
    r = k4w2_read_depth_camera_param(ctx, &depth_param);
    if (K4W2_SUCCESS != r) {
	VERBOSE("read_depth_camera_param() failed");
	return NULL;
    }

    return k4w2_registration_create(&color_param,
				    &depth_param);
}

k4w2_registration_t
k4w2_registration_create(struct kinect2_color_camera_param *color,
			 struct kinect2_depth_camera_param *depth)
{
    int mx, my;
    
    k4w2_registration_t reg = (k4w2_registration_t)malloc(sizeof(*reg));
    memcpy(&reg->depth, depth, sizeof(reg->depth));
    memcpy(&reg->color, color, sizeof(reg->color));


    for (mx = 0; mx < 512; mx++)
	for (my = 0; my < 424; my++) {
	    float x, y;
	    distort_depth(reg, mx,my, &x, &y);
	    reg->undistort_map[mx][my][0] = x;
	    reg->undistort_map[mx][my][1] = y;
	}

    for (mx = 0; mx < 512; mx++)
	for (my = 0; my < 424; my++) {
	    float rx, ry;
	    depth_to_color(reg,
			   reg->undistort_map[mx][my][0],
			   reg->undistort_map[mx][my][1], &rx, &ry);
	    reg->depth_to_color_map[mx][my][0] = rx;
	    reg->depth_to_color_map[mx][my][1] = ry;
	}

    return reg;
}

void
k4w2_registration_release(k4w2_registration_t *registration)
{
    if (registration) {
	free(*registration);
	*registration = 0;
    }
}

/**
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
