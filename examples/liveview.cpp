/**
 * @file   live.cpp
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
#define CHK( exp ) do { int res = exp; if (!res) { OUTPUT(#exp " failed."); } } while(0)

enum {
    COLOR=0,
    DEPTH=1,
};
const void *last_ptr[2] = {0};
int last_len[2] = {0};

static void color_cb(const void *buffer, int length, void *userdata)
{
    //const struct kinect2_color_header *h = (const struct kinect2_color_header*)buffer;
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


    k4w2_registration_t registration = 0;

    k4w2_decoder_t decoder[2] = {0};

    /* 
     * Note; If you want to use CUDA and OpenCL at the same time,
     * OpenCL context must be initialized first.  To guarantee this
     * initialization order, the code below initializes the depth
     * decoder first.
     */
    unsigned int type = 0;
    //type |= K4W2_DECODER_DISABLE_CUDA;
    //type |= K4W2_DECODER_DISABLE_OPENCL;
    //type |= K4W2_DECODER_ENABLE_OPENGL;
    decoder[1] = k4w2_decoder_open(K4W2_DECODER_DEPTH | type, 1);
    decoder[0] = k4w2_decoder_open(K4W2_DECODER_COLOR | type , 1);

    {
	struct kinect2_color_camera_param colorparam;
	struct kinect2_depth_camera_param depthparam;
	struct kinect2_p0table p0table;
	CHK( K4W2_SUCCESS == k4w2_read_color_camera_param(ctx, &colorparam) );
	CHK( K4W2_SUCCESS == k4w2_read_depth_camera_param(ctx, &depthparam) );
	CHK( K4W2_SUCCESS == k4w2_read_p0table(ctx, &p0table) );

	CHK( K4W2_SUCCESS == k4w2_decoder_set_params(decoder[1],
						     &colorparam,
						     &depthparam,
						     &p0table) );

	registration = k4w2_registration_create(&colorparam,
						&depthparam);
    }

    k4w2_set_color_callback(ctx, color_cb, decoder[0]);
    k4w2_set_depth_callback(ctx, depth_cb, decoder[1]);

    CHK( K4W2_SUCCESS == k4w2_start(ctx) );

    cv::Mat rgb8U3(1080, 1920, CV_8UC3);
    cv::Mat resized8U3(1080/4, 1920/4, CV_8UC3);


    float *tmpbuf = new float[512*424*2];
    int sizes[2]={424, 512};
    cv::Mat depth32F1(2, sizes, CV_32FC1, tmpbuf);
    cv::Mat ir32F1(2, sizes, CV_32FC1, tmpbuf + 512*424);

    cv::Mat mapped8U3(424, 512, CV_8UC3);

    const bool is_rgb_colorspace = (K4W2_COLORSPACE_RGB == k4w2_decoder_get_colorspace(decoder[COLOR]));

    int shutdown = 0;
    int slot = 0;
    while (!shutdown) {
	if (last_ptr[COLOR]) {
	    k4w2_decoder_request(decoder[COLOR], slot, last_ptr[COLOR], last_len[COLOR]);
	    k4w2_decoder_fetch(decoder[COLOR], slot, rgb8U3.data, 1920*1080*3);

	    if (is_rgb_colorspace)
		cv::cvtColor(rgb8U3, rgb8U3, cv::COLOR_RGB2BGR);

	    cv::resize(rgb8U3, resized8U3, cv::Size(), 1, 1);

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

	if (1) {
	    int dy;
#ifdef _OPENMP
#pragma omp parallel for
#endif
	    for(dy = 0; dy < 424; ++dy){
		int dx;
		for(dx = 0; dx < 512; ++dx) {
		    float z = depth32F1.at<float>(dy, dx);

		    if (z < 500 || 5000 < z) {
			mapped8U3.at<cv::Vec3b>(dy, dx)[0]=255;
			mapped8U3.at<cv::Vec3b>(dy, dx)[1]=0;
			mapped8U3.at<cv::Vec3b>(dy, dx)[2]=0;
		    } else {
			float cx, cy;
			k4w2_registration_depth_to_color(registration,
							 dx, dy, z,
							 &cx, &cy);
			if (0<=cx && cx<=1920 && 0<cy && cy<1080) {
			    mapped8U3.at<cv::Vec3b>(dy,dx) = rgb8U3.at<cv::Vec3b>( int(cy), int(cx));
			}
		    }
		}
	    }
	    cv::imshow("mapped", mapped8U3);
	}


	const int rawkey = cv::waitKey(1);
	switch (rawkey & 0x0ffff) {
	case 'Q':
	case 'q':
	    shutdown = true;
	    break;
	}

    }

    

     
    CHK( K4W2_SUCCESS == k4w2_stop(ctx) );

    k4w2_close(&ctx);

    k4w2_decoder_close(&decoder[0]);
    k4w2_decoder_close(&decoder[1]);

    k4w2_registration_release(&registration);

    return 0;
}

/*
 * Local Variables:
 * mode: c++
 * c-basic-offset:  4
 * End:
 */
