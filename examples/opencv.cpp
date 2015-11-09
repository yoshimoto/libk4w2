/**
 * @file   opencv.cpp
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 01:24:41 2015
 * 
 * @brief  
 * 
 * 
 */

#include "libk4w2/libk4w2.h"
#include "libk4w2/decoder.h"
#include "libk4w2/registration.h"

#include <stdio.h>
#include <time.h> /* nanosleep() */
#include <stdlib.h> /* exit() */

#include <opencv2/opencv.hpp>

#define OUTPUT(fmt, ...) do { fprintf(stderr, fmt "\n", ## __VA_ARGS__); } while(0)
#define ABORT(fmt, ...) do { OUTPUT(fmt, ## __VA_ARGS__ ); exit(EXIT_FAILURE); } while(0)
#define CHK_K4W2( exp ) do { int res = exp; if (K4W2_SUCCESS != res) { OUTPUT(#exp " failed (error code is %d)", res); } } while(0)

enum {
    COLOR=0,
    DEPTH=1,
};
const void *last_ptr[2] = {0};
int last_len[2] = {0};

static void color_cb(const void *buffer, int length, void *userdata)
{
    const struct kinect2_color_footer *f = KINECT2_GET_COLOR_FOOTER(buffer, length);

    if (length < 10000) {
	OUTPUT("bad color frame?");
	return;
    }

    fprintf(stderr, "color: sequence:%10d timestamp:%10d\n", f->sequence, f->timestamp);
    last_ptr[COLOR] = buffer;
    last_len[COLOR] = length;
}

static void depth_cb(const void *buffer, int length, void *userdata)
{
    const struct kinect2_depth_footer *f = KINECT2_GET_DEPTH_FOOTER(buffer);

    if (length != KINECT2_DEPTH_FRAME_SIZE*10) {
	OUTPUT("bad depth frame?");
	return;
    } 

    fprintf(stderr, "depth: sequence:%10d timestamp:%10d\n", f->sequence, f->timestamp);
    last_ptr[DEPTH] = buffer;
    last_len[DEPTH] = length;
}


int
main(int argc, const char *argv[])
{
    int deviceid = 0;
    if (argc >= 2) {
	deviceid  = atoi( argv[1] );
    }
    k4w2_t ctx = k4w2_open(deviceid, 0);
    if (!ctx) {
	ABORT("failed to open kinect device #%d", deviceid);
    }

    k4w2_decoder_t decoder[2] = {0};

    unsigned int options = 0;
    // options |= K4W2_DECODER_DISABLE_CUDA;
    // options |= K4W2_DECODER_DISABLE_OPENCL;
    decoder[1] = k4w2_decoder_open(K4W2_DECODER_DEPTH | options, 1);
    decoder[0] = k4w2_decoder_open(K4W2_DECODER_COLOR | options, 1);

    if (decoder[1]) {
	struct kinect2_color_camera_param colorparam;
	struct kinect2_depth_camera_param depthparam;
	struct kinect2_p0table p0table;
	CHK_K4W2( k4w2_read_color_camera_param(ctx, &colorparam) );
	CHK_K4W2( k4w2_read_depth_camera_param(ctx, &depthparam) );
	CHK_K4W2( k4w2_read_p0table(ctx, &p0table) );
	CHK_K4W2( k4w2_decoder_set_params(decoder[1],
					  &colorparam,
					  &depthparam,
					  &p0table) );
    }

    k4w2_set_color_callback(ctx, color_cb, decoder[0]);
    k4w2_set_depth_callback(ctx, depth_cb, decoder[1]);

    CHK_K4W2( k4w2_start(ctx) );

    cv::Mat rgb8U3(1080, 1920, CV_8UC3);
    cv::Mat resized8U3(1080/4, 1920/4, CV_8UC3);

    float *tmpbuf = new float[512*424*2];
    int sizes[2]={424, 512};
    cv::Mat depth32F1(2, sizes, CV_32FC1, tmpbuf);
    cv::Mat ir32F1(2, sizes, CV_32FC1, tmpbuf + 512*424);

    const bool is_rgb_colorspace = (K4W2_COLORSPACE_RGB == k4w2_decoder_get_colorspace(decoder[COLOR]));

    int shutdown = 0;
    while (!shutdown) {
	const int slot = 0;

	if (last_ptr[COLOR]) {
	    k4w2_decoder_request(decoder[COLOR], slot, last_ptr[COLOR], last_len[COLOR]);
	    k4w2_decoder_fetch(decoder[COLOR], slot, rgb8U3.data, 1920*1080*3);

	    cv::resize(rgb8U3, resized8U3, cv::Size(), 0.5, 0.5);

	    if (is_rgb_colorspace)
		cv::cvtColor(resized8U3, resized8U3, CV_RGB2BGR);
	    cv::imshow("rgb", resized8U3);

	    last_ptr[COLOR] = NULL;
	}

	if (last_ptr[DEPTH]) {
	    k4w2_decoder_request(decoder[DEPTH], slot, last_ptr[DEPTH], last_len[DEPTH]);
	    k4w2_decoder_fetch(decoder[DEPTH], slot, tmpbuf, 512*424*2*sizeof(float));

	    cv::imshow("depth", depth32F1/ 4500.f);

	    cv::imshow("ir", ir32F1/50000.f);

	    last_ptr[DEPTH] = NULL;
	}

	const int rawkey = cv::waitKey(1);
	switch (rawkey & 0x0ffff) {
	case 'Q':
	case 'q':
	    shutdown = true;
	    break;
	}

    }

    CHK_K4W2( k4w2_stop(ctx) );

    k4w2_close(&ctx);

    k4w2_decoder_close(&decoder[0]);
    k4w2_decoder_close(&decoder[1]);

    return 0;
}

/*
 * Local Variables:
 * mode: c++
 * c-basic-offset:  4
 * End:
 */
