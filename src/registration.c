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
undistort_depth(k4w2_registration_t reg, int x, int y, float* mx, float* my)
{
    float dx = ((float)x - reg->depth.cx) / reg->depth.fx;
    float dy = ((float)y - reg->depth.cy) / reg->depth.fy;

    float ps = (dx * dx) + (dy * dy);
    float qs = ((ps * reg->depth.k3 + reg->depth.k2) * ps + reg->depth.k1) * ps + 1.0;
    int i;
    for (i = 0; i < 9; i++) {
	float qd = ps / (qs * qs);
	qs = ((qd * reg->depth.k3 + reg->depth.k2) * qd + reg->depth.k1) * qd + 1.0;
    }

    *mx = dx / qs;
    *my = dy / qs;
}

static inline void
depth_to_color(k4w2_registration_t reg, float mx, float my, float* rx, float* ry)
{
    mx *= reg->depth.fx * depth_q;
    my *= reg->depth.fy * depth_q;

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

    *rx = wx / (reg->color.f * color_q);
    *ry = wy / (reg->color.f * color_q);
}

void
k4w2_registration_depth_to_color(k4w2_registration_t reg,
				    int dx, int dy, float dz,
				    float *cx, float *cy)
{
    float rx = reg->depth_to_color_map[dx][dy][0];
    float ry = reg->depth_to_color_map[dx][dy][1];

    rx += (reg->color.shift_m / dz) - (reg->color.shift_m / reg->color.shift_d);

    // !!FIXME!!
    *cx = rx * reg->color.f + reg->color.cx;
    *cy = ry * reg->color.f + reg->color.cy;
}


k4w2_registration_t
k4w2_registration_create_from_file(const char *dirname)
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
    int x, y;
    
    k4w2_registration_t reg = (k4w2_registration_t)malloc(sizeof(*reg));
    memcpy(&reg->depth, depth, sizeof(reg->depth));
    memcpy(&reg->color, color, sizeof(reg->color));


    for (x = 0; x < 512; x++)
	for (y = 0; y < 424; y++) {
	    float mx, my;
	    undistort_depth(reg, x,y, &mx, &my);
	    reg->undistort_map[x][y][0] = mx;
	    reg->undistort_map[x][y][1] = my;
	}

    for (x = 0; x < 512; x++)
	for (y = 0; y < 424; y++) {
	    float rx, ry;
	    depth_to_color(reg,
			   reg->undistort_map[x][y][0],
			   reg->undistort_map[x][y][1], &rx, &ry);
	    reg->depth_to_color_map[x][y][0] = rx;
	    reg->depth_to_color_map[x][y][1] = ry;
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
