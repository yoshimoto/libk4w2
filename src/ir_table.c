/*
 * This file is a part of the OpenKinect project
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/**
 *
 * https://github.com/OpenKinect/libfreenect2/issues/144
 *
 */

#include <float.h>  /* for DBL_EPSILON */
#include <math.h>   /* for fabs() */
#include "libk4w2/libk4w2.h"
#include "module.h"

int
k4w2_create_lut_table(short lut[], const size_t lut_size)
{
    if ( sizeof(lut[0])*2048 != lut_size )
	return K4W2_ERROR;

    short y = 0;
    int x;
    for (x = 0; x < 1024; x++)
    {
	unsigned inc = 1 << (x/128 - (x>=128));
	lut[x] = y;
	lut[1024 + x] = -y;
	y += inc;
    }
    lut[1024] = 32767;

    return K4W2_SUCCESS;
}

/**
 * x,y: undistorted, normalized coordinates
 * xd,yd: distorted, normalized coordinates
 */
static void
distort(const struct kinect2_depth_camera_param *p,
	double x, double y, double *xd, double *yd) 
{
    double x2 = x * x;
    double y2 = y * y;
    double r2 = x2 + y2;
    double xy = x * y;
    double kr = ((p->k3 * r2 + p->k2) * r2 + p->k1) * r2 + 1.0;
    *xd = x*kr + p->p2*(r2 + 2*x2) + 2*p->p1*xy;
    *yd = y*kr + p->p1*(r2 + 2*y2) + 2*p->p2*xy;
}

/**
 * The inverse of distort() using Newton's method
 * Return true if converged correctly
 * This function considers tangential distortion with double precision.
 */
static int
undistort(const struct kinect2_depth_camera_param *p,
	  double x, double y, double *xu, double *yu) 
{
    double x0 = x;
    double y0 = y;

    double last_x = x;
    double last_y = y;
    const int max_iterations = 100;
    int iter;
    for (iter = 0; iter < max_iterations; iter++) {
	double x2 = x*x;
	double y2 = y*y;
	double x2y2 = x2 + y2;
	double x2y22 = x2y2*x2y2;
	double x2y23 = x2y2*x2y22;

	//Jacobian matrix
	double Ja = p->k3*x2y23 + (p->k2+6*p->k3*x2)*x2y22 + (p->k1+4*p->k2*x2)*x2y2 + 2*p->k1*x2 + 6*p->p2*x + 2*p->p1*y + 1;
	double Jb = 6*p->k3*x*y*x2y22 + 4*p->k2*x*y*x2y2 + 2*p->k1*x*y + 2*p->p1*x + 2*p->p2*y;
	double Jc = Jb;
	double Jd = p->k3*x2y23 + (p->k2+6*p->k3*y2)*x2y22 + (p->k1+4*p->k2*y2)*x2y2 + 2*p->k1*y2 + 2*p->p2*x + 6*p->p1*y + 1;

	//Inverse Jacobian
	double Jdet = 1/(Ja*Jd - Jb*Jc);
	double a = Jd*Jdet;
	double b = -Jb*Jdet;
	double c = -Jc*Jdet;
	double d = Ja*Jdet;

	double f, g;
	distort(p, x, y, &f, &g);
	f -= x0;
	g -= y0;

	x -= a*f + b*g;
	y -= c*f + d*g;
	const double eps = DBL_EPSILON * 16;
	if (fabs(x - last_x) <= eps && fabs(y - last_y) <= eps)
	    break;
	last_x = x;
	last_y = y;
    }
    *xu = x;
    *yu = y;
    return iter < max_iterations;
}

int
k4w2_create_xz_table(const struct kinect2_depth_camera_param *p,
		     float xtable[], size_t xtable_size,
		     float ztable[], size_t ztable_size)
{
    if ( 512*424*sizeof(float) != xtable_size )
	return K4W2_ERROR;
    if ( 512*424*sizeof(float) != ztable_size )
	return K4W2_ERROR;

    const double scaling_factor = 8192;
    const double unambigious_dist = 6250.0/3;
    size_t divergence = 0;
    size_t i;
    for (i = 0; i < 512*424; i++)
    {
	size_t xi = i % 512;
	size_t yi = i / 512;
	double xd = (xi + 0.5 - p->cx)/p->fx;
	double yd = (yi + 0.5 - p->cy)/p->fy;
	double xu, yu;
	divergence += !undistort(p, xd, yd, &xu, &yu);
	xtable[i] = scaling_factor*xu;
	ztable[i] = unambigious_dist/sqrt(xu*xu + yu*yu + 1);
    }
    if (divergence > 0) {
	VERBOSE("%zd pixels in x/ztable have incorrect undistortion.", divergence);
    }

    return K4W2_SUCCESS;
}

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
