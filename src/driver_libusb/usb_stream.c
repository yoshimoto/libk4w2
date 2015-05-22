/**
 * @file   usb_stream.c
 * @author Hiromasa YOSHIMOTO
 * @date   Sat Apr 11 15:37:19 2015
 * 
 * @brief  
 * 
 * 
 */

#include "usb_dev.h"
#include "../module.h"

#include <stdarg.h>
#include <assert.h>
#include <string.h> /* memset() */


static const char *
get_stream_type_str(unsigned char type)
{
    const char *r;
    switch (type) {
    case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS: r = "iso"; break;
    case LIBUSB_TRANSFER_TYPE_BULK: r = "bulk"; break;
    default: r = "unknown"; break;
    }
    return r;
}

struct usb_stream_ctx {
    volatile int shutdown; /* set 1 to shutdown */

    unsigned char type;
    libusb_device_handle * handle;
    int num_xfers;
    int num_pkts;
    int pkt_len;
    usb_stream_callback callback;
    void *callback_arg;
    unsigned char *buffers;
    struct libusb_transfer **xfers;
    int num_inactive_xfers;

    MUTEX_T lock;
    /* cond_inactive will be signaled when num_inactive_xfers == num_xfers */
    COND_T  cond_inactive;
};

#define CHECK_STREAM_CTX(ctx) do { if (0==(ctx)) return -1; } while(0)

static void
usbmisc_stream_callback(struct libusb_transfer *xfer)
{
    usb_stream_t strm = (usb_stream_t)xfer->user_data;
    //VERBOSE("cb; %d", strm->shutdown);
    assert( !strm->shutdown );

    int r;
    switch(xfer->status) {
    case LIBUSB_TRANSFER_COMPLETED: /* Normal operation. */
	if (strm->callback)
	    strm->callback(xfer, strm->callback_arg);
	if (!strm->shutdown) {
	    r = libusb_submit_transfer(xfer);
	    if (LIBUSB_SUCCESS != r) {
		VERBOSE("type:%s libusb_submit_transfer() returns %s",
			get_stream_type_str(strm->type),
			libusb_error_name(r));
		++strm->num_inactive_xfers;
	    }
	} else{
	    ++strm->num_inactive_xfers;
	}
	break;
    case LIBUSB_TRANSFER_NO_DEVICE:
	VERBOSE("no device");
	strm->shutdown = 1;
	++strm->num_inactive_xfers;
	break;
    case LIBUSB_TRANSFER_CANCELLED:
	VERBOSE("cancelled");
	++strm->num_inactive_xfers;
	break;
    default:
	/* On other errors, resubmit the transfer - in particular, libusb
	   on OSX tends to hit random errors a lot.  If we don't resubmit
	   the transfers, eventually all of them die and then we don't get
	   any more data from the Kinect. */
	VERBOSE("%s transfer error: %d",
		get_stream_type_str(strm->type),
		xfer->status);
	if (!strm->shutdown) {
	    r = libusb_submit_transfer(xfer);
	    if (r != 0) {
		VERBOSE("%s transfer resubmission failed after unknown error: %d",
			get_stream_type_str(strm->type),
			r);
		++strm->num_inactive_xfers;
		if (r == LIBUSB_ERROR_NO_DEVICE) {
		    strm->shutdown = 1;
		}
	    }
	}
	break;
    }

    if (strm->num_inactive_xfers >= strm->num_xfers) {
	COND_BROADCAST(&strm->cond_inactive);
    }
}


static int
wait_for_stopped(usb_stream_t strm)
{
#if ! defined __APPLE__
    struct timespec deadline;
    int r;
    MUTEX_LOCK(&strm->lock);
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += 3;
    r = 0;
    while (strm->num_inactive_xfers < strm->num_xfers && !r) {
	r = COND_TIMEDWAIT(&strm->cond_inactive, &strm->lock, &deadline);
    }
    MUTEX_UNLOCK(&strm->lock);
    if (r) {
	VERBOSE("timeout ?");
    }
#else
/*    while (strm->num_inactive_xfers < strm->num_xfers) {
      }*/
#endif
    return 0;
}

usb_stream_t 
usb_stream_open(libusb_device_handle *handle,
		unsigned char type,
		unsigned char endpoint,
		int num_xfers,
		int num_pkts,
		int pkt_length,
		usb_stream_callback callback,
		void *callback_arg)
{
    usb_stream_t strm = (usb_stream_t)malloc(sizeof(struct usb_stream_ctx));
    memset(strm, 0, sizeof(struct usb_stream_ctx));

    switch (type) {
    case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
	break;
    case LIBUSB_TRANSFER_TYPE_BULK:
	assert(1==num_pkts);
	break;
    default:
	VERBOSE("not implemented yet");
	assert(0);
    }

    strm->type      = type;
    strm->handle    = handle;
    strm->callback  = callback;
    strm->callback_arg = callback_arg;
    strm->num_xfers = num_xfers;
    strm->num_pkts  = num_pkts;
    strm->pkt_len   = pkt_length;
    strm->buffers   = (unsigned char*)malloc(num_xfers * num_pkts * pkt_length);
    strm->xfers     = (struct libusb_transfer**)malloc(sizeof(struct libusb_transfer*) * num_xfers);
    strm->num_inactive_xfers = 0;

    MUTEX_INIT(&strm->lock);
    COND_INIT(&strm->cond_inactive);

    unsigned char *pointer = strm->buffers;
    int i;
    for (i = 0; i < num_xfers; ++i) {
	switch (type) {
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
	    strm->xfers[i] = libusb_alloc_transfer(num_pkts);
	    assert(strm->xfers[i]);

	    libusb_fill_iso_transfer(strm->xfers[i],
				     handle,
				     endpoint,
				     pointer,
				     num_pkts * pkt_length,
				     num_pkts,
				     usbmisc_stream_callback,
				     strm,
				     0 /* no timeout */);
	    libusb_set_iso_packet_lengths(strm->xfers[i], pkt_length);
	    break;
	case LIBUSB_TRANSFER_TYPE_BULK:
	    strm->xfers[i] = libusb_alloc_transfer(0);
	    assert(strm->xfers[i]);

	    libusb_fill_bulk_transfer(strm->xfers[i],
				      handle,
				      endpoint,
				      pointer,
				      pkt_length,
				      usbmisc_stream_callback,
				      strm,
				      0 /* no timeout */);
	    break;
	}

	pointer += num_pkts * pkt_length;
    }
    return strm;
}

int
usb_stream_set_callback(usb_stream_t strm,
			usb_stream_callback callback,
			void *callback_arg)
{
    CHECK_STREAM_CTX(strm);

    strm->callback = callback;
    strm->callback_arg = callback_arg;
    return 0;
}

int
usb_stream_start(usb_stream_t strm)
{
    int i;
    CHECK_STREAM_CTX(strm);

    for (i = 0; i < strm->num_xfers; ++i) {
	int r = libusb_submit_transfer(strm->xfers[i]);
	if (LIBUSB_SUCCESS != r) {
	    VERBOSE("submit(%d) returns %s",i, libusb_error_name(r));
	    ++strm->num_inactive_xfers;
	}
    }
    return 0;
}


int
usb_stream_stop(usb_stream_t strm)
{
    CHECK_STREAM_CTX(strm);

    strm->shutdown = 1;
    return 0;
}

int
usb_stream_close(usb_stream_t *pointer)
{
    if (!pointer)
	return -1;
    CHECK_STREAM_CTX(*pointer);

    usb_stream_t strm = *pointer;
    usb_stream_stop(strm);

    wait_for_stopped(strm);
    
    if (strm->xfers) {
	int i;
	for (i = 0; i < strm->num_xfers; i++)
	    libusb_free_transfer(strm->xfers[i]);

	free(strm->xfers);
	strm->xfers = 0;
    }
    if (strm->buffers) {
	free(strm->buffers);
	strm->buffers = 0;
    }

    COND_DESTROY(&strm->cond_inactive);
    MUTEX_DESTROY(&strm->lock);

    free(*pointer);
    *pointer = 0;

    return 0;
}


/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
