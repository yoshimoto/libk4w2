/**
 * @file   driver_libusb.c
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 02:35:02 2015
 * 
 * @brief  Implementation of libusb-based driver
 *
 * This driver shall work on linux, mac os, and microsoft windows.
 * 
 * 
 */

#if ! defined WITH_V4L2
#  error "WITH_LIBUSB is not defined, while driver_libusb.c is compiled"
#endif

#include <libusb-1.0/libusb.h>
#include "module.h"

typedef struct {
     struct k4w2_driver_ctx k4w2; /* !! must be the first item */
     int ch0;
     int ch1;
     THREAD_T thread;
     volatile unsigned shutdown:1;
} k4w2_libusb;

static int
k4w2_libusb_open(k4w2_t ctx, unsigned int deviceid, unsigned int flags)
{
    k4w2_libusb * libusb = (k4w2_libusb *)ctx;
    int res = K4W2_ERROR;
    return res;	
}

static int
k4w2_libusb_start(k4w2_t ctx)
{
    k4w2_libusb * libusb = (k4w2_libusb *)ctx;
    return K4W2_ERROR;
}

static int
k4w2_libusb_stop(k4w2_t ctx)
{
    k4w2_libusb * libusb = (k4w2_libusb *)ctx;
    return K4W2_ERROR;
}

static int
k4w2_libusb_close(k4w2_t ctx)
{
    k4w2_libusb * libusb = (k4w2_libusb *)ctx;
    return K4W2_ERROR;
}

static int
k4w2_libusb_read_param(k4w2_t ctx, PARAM_ID id, void *param, int length)
{
    k4w2_libusb * libusb = (k4w2_libusb *)ctx;
    int res = K4W2_ERROR;
    return res;
}


static const k4w2_driver_ops ops =
{
    .open	= k4w2_libusb_open,
    .start	= k4w2_libusb_start,
    .stop	= k4w2_libusb_stop,
    .close	= k4w2_libusb_close,
    .read_param = k4w2_libusb_read_param,
};

REGISTER_MODULE(k4w2_driver_libusb_init)
{
    k4w2_register_driver("libusb", &ops, sizeof(k4w2_libusb));
}

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
