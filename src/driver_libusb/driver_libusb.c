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

#if ! defined WITH_LIBUSB
#  error "WITH_LIBUSB is not defined, while driver_libusb.c is compiled"
#endif

#include <assert.h>
#include <stdarg.h>
#include <time.h> /* nanosleep() */
#include <stddef.h> /* offsetof() */

#include "usb_dev.h"
#include "module.h"

typedef struct {
    struct k4w2_driver_ctx k4w2; /* !! must be the first item */

    libusb_context *context;
    libusb_device *dev;
    libusb_device_handle *handle;
    usb_stream_t stream[2]; /* 0:color, 1:depth */

    uint32_t request_sequence;

    THREAD_T thread;
    volatile unsigned shutdown:1;
} k4w2_libusb;

static const struct DeviceID {
    uint16_t vendor_id;
    uint16_t product_id;
} device_id[] = {
    {0x045e, 0x02d8}, /* kinect for windows 2 */
    {0x045e, 0x02c4}, /* kinect for windows 2 preview? */
};

#define ControlAndRgbInterfaceId 0
#define IrInterfaceId            1

#define OUTBOUND_ENDPOINT 0x002
#define INBOUND_ENDPOINT  0x081

#define REQUEST_MAGIC   0x06022009
#define RESPONSE_MAGIC  0x0A6FE000

#define KCMD_READ_DATA_PAGE		0x22 /* read paramters */
#define KCMD_SET_STREAMING		0x2B /* start/stop color stream */
#define STREAMING_ON  1
#define STREAMING_OFF 0
#define KCMD_START_DEPTH		0x09 /* start depth stream */
#define KCMD_STOP_DEPTH			0x0A /* stop depth stream */

static const unsigned char inbound_endpoint  = 0x081;
static const unsigned char outbound_endpoint = 0x002;

#define TIMEOUT 1000

#define cpu_to_le32(x) (x)

#define STRICT( exp ) do {						\
	int r = (exp); if (LIBUSB_SUCCESS != r) {			\
	    VERBOSE(#exp " returns %s", libusb_error_name(r));		\
	    goto exit;							\
	}								\
    } while (0)
#define PERMISSIVE( exp ) do {				       \
	int r = (exp);					       \
	if (LIBUSB_SUCCESS != r) {			       \
	    VERBOSE(#exp " returns %s", libusb_error_name(r)); \
	}						       \
    } while (0)


static int
get_max_iso_packet_size(libusb_device *device,
			int configuration, int alternate_setting, int endpoint,
			int *size)
{
    struct libusb_config_descriptor *config_desc;
    int r = LIBUSB_ERROR_NOT_FOUND;

    r = libusb_get_config_descriptor_by_value(device, configuration, &config_desc);

    if(r == LIBUSB_SUCCESS) {
	int interface_idx;
	for(interface_idx = 0; interface_idx < config_desc->bNumInterfaces; ++interface_idx) {
	    const struct libusb_interface* iface = &config_desc->interface[interface_idx];

	    if(iface->num_altsetting > alternate_setting) {
		const struct libusb_interface_descriptor *i_desc = &iface->altsetting[alternate_setting];
		const struct libusb_endpoint_descriptor *endpoint_desc = 0;
		int endpoint_idx;
		for(endpoint_idx = 0; endpoint_idx < i_desc->bNumEndpoints; ++endpoint_idx) {
		    if(i_desc->endpoint[endpoint_idx].bEndpointAddress == endpoint &&
		       (i_desc->endpoint[endpoint_idx].bmAttributes & 0x3) == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
			endpoint_desc = i_desc->endpoint + endpoint_idx;
			break;
		    }
		}

		if(endpoint_desc != 0) {
		    struct libusb_ss_endpoint_companion_descriptor *companion_desc;
		    /* ctx is only used for error reporting, libusb should better
		     * ask for a libusb_device anyway...*/
		    r = libusb_get_ss_endpoint_companion_descriptor(NULL /* ctx */,
								    endpoint_desc,
								    &companion_desc);
		    if(r != LIBUSB_SUCCESS)
			continue;
		    *size = companion_desc->wBytesPerInterval;
		    libusb_free_ss_endpoint_companion_descriptor(companion_desc);
		    break;
		}
	    }
	}
    }
    libusb_free_config_descriptor(config_desc);

    return r;
}

static void
set_isochronous_delay(libusb_device_handle *handle)
{
    // for details see USB 3.1 r1 spec section 9.4.11

    uint8_t bmRequestType = LIBUSB_RECIPIENT_DEVICE;
    uint8_t bRequest = LIBUSB_SET_ISOCH_DELAY;
    // if no super speed hubs in between,
    // then it is equal to tTPTransmissionDelay(=40ns)
    uint16_t wValue  = 40; // 40 nanoseconds
    uint16_t wIndex  = 0;
    uint16_t wLength = 0;
    uint8_t *data    = 0;

    int r = libusb_control_transfer(handle, bmRequestType, bRequest,
				    wValue, wIndex, data, wLength, TIMEOUT);
}

static libusb_device_handle *
open_handle(libusb_device *device)
{
    int res;
    libusb_device_handle *handle;
    static struct timespec wait_for_initialize = {1,0};
    STRICT( libusb_open(device, &handle) );

    if (NULL==handle)
	goto exit;

    /* reset the device; this is a workaround for initializing
     * kinect2's configration */
    res = libusb_reset_device(handle);
    switch (res) {
    case LIBUSB_SUCCESS:
	break;
    case LIBUSB_ERROR_NOT_FOUND:
	/* reopen is required */
	libusb_close(handle);
	nanosleep(&wait_for_initialize, 0);
	STRICT( libusb_open(device, &handle) );
	break;
    default:
	VERBOSE("libusb_reset_device() returns %s", libusb_error_name(res));
	break;
    }
exit:
    return handle;
}


static int
send_cmd(k4w2_libusb *usb, uint32_t command,
	 void *response, size_t max_response_size,
	 int num_params,
	 ...)
{
    int i;
    int done;
    int response_size = 0;
    struct Request {
	uint32_t magic;
	uint32_t sequence;
	uint32_t max_response;
	uint32_t command;
	uint32_t reserved0;
	uint32_t param[1];
    } req;
    va_list ap;
    int reqsize = offsetof(struct Request,param)
	+ sizeof(req.param[0])*num_params;
    int sequence = ++usb->request_sequence;
    
    assert(num_params <= ARRAY_SIZE(req.param));
    req.magic = cpu_to_le32(REQUEST_MAGIC);
    req.sequence = cpu_to_le32(sequence);
    req.max_response = cpu_to_le32(max_response_size);
    req.command = cpu_to_le32(command);
    req.reserved0 = 0;

    va_start(ap, num_params);
    for (i = 0; i < num_params; ++i) {
	req.param[i] = cpu_to_le32( va_arg(ap, uint32_t) );
    }
    va_end(ap);


    /* Send request */
    STRICT( libusb_bulk_transfer(usb->handle, OUTBOUND_ENDPOINT,
				 (unsigned char *)&req, reqsize,
				 &done, TIMEOUT));
    if (done != reqsize){
	VERBOSE("%d != %d", done, reqsize);
	return -1;
    }

    /* Receive response if needed */
    if (max_response_size > 0) {
	STRICT( libusb_bulk_transfer(usb->handle, INBOUND_ENDPOINT,
				     response,
				     max_response_size,
				     &response_size, TIMEOUT) );
    }

    /* Receive completion */
    uint32_t completion[32] = {0};
    STRICT( libusb_bulk_transfer(usb->handle, INBOUND_ENDPOINT,
				 (unsigned char*)completion,
				 sizeof(completion),
				 &done, TIMEOUT));

    if (cpu_to_le32(RESPONSE_MAGIC) != completion[0]) {
	VERBOSE("unexpected magic in response; %08x != %08x",
		0x0A6FE000, completion[0]);
    } else if (req.sequence != completion[1]) {
	VERBOSE("unexpected sequence in response ; %08x != %08x",
		req.sequence, completion[1]);
    }

    return response_size;
exit:
    return -1;
}

static int k4w2_libusb_close(k4w2_t ctx);

static void
kinect2_found_callback(libusb_device *device,
		       const struct libusb_device_descriptor *desc,
		       void *userdata)
{
    k4w2_libusb * usb = (k4w2_libusb*)userdata;
    int res;

    if (usb->dev) {
	VERBOSE("already assigned");
	return ;
    }
    usb->dev = device;
}

static void *
k4w2_libusb_thread(void *userarg)
{
    k4w2_t *ctx = (k4w2_t *)ctx;
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
    struct timeval t = {0, 100};

    while (usb->shutdown) {
	libusb_handle_events_timeout_completed(usb->context, &t, 0);
    }
}

static void
depth_cb(struct libusb_transfer *xfer, void *userarg)
{
    k4w2_t *ctx = (k4w2_t *)userarg;
    k4w2_libusb * usb = (k4w2_libusb *)userarg;
}

static void
color_cb(struct libusb_transfer *xfer, void *userarg)
{
    k4w2_t *ctx = (k4w2_t *)userarg;
    k4w2_libusb * usb = (k4w2_libusb *)userarg;
}

static int
k4w2_libusb_open(k4w2_t ctx, unsigned int deviceid, unsigned int flags)
{
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
    int i;
    CHANNEL ch;

    STRICT( libusb_init(&usb->context) );

    for (i = 0; i < ARRAY_SIZE(device_id); ++i) {
	usb_foreach_device(usb->context,
			   device_id[i].vendor_id, device_id[i].product_id,
			   kinect2_found_callback,
			   usb);
    }
    if (!usb->dev) {
	VERBOSE("no kinect2 found.");
	goto exit;
    }

    usb->handle = open_handle(usb->dev);
    if (!usb->handle) {
	VERBOSE("failed to open device");
	goto exit;
    }

    STRICT( libusb_set_configuration(usb->handle, 1) );
    STRICT( libusb_claim_interface(usb->handle,
				   ControlAndRgbInterfaceId) );

    if (DEPTH_ENABLED(ctx)) {
	libusb_claim_interface(usb->handle, IrInterfaceId) ;
	set_isochronous_delay(usb->handle);
	STRICT(libusb_set_interface_alt_setting(usb->handle,
						IrInterfaceId, 1));
    }

    if (COLOR_ENABLED(ctx)) {
	const int num_xfers = 64;
	usb->stream[0] = usb_stream_open(usb->handle,
					 LIBUSB_TRANSFER_TYPE_BULK,
					 0x83,
					 num_xfers,
					 1,
					 0x4000,
					 color_cb,
					 ctx);
	if (!usb->stream[0]) {
	    VERBOSE("failed to create bulk stream for color data");
	    goto exit;
	}
    }

    if (DEPTH_ENABLED(ctx)) {
	int max_iso_packet_size = 0;
	get_max_iso_packet_size(usb->dev, 1, 1, 0x084,
				&max_iso_packet_size);
	if (0 == max_iso_packet_size) {
	    VERBOSE("max_iso_packet_size is zero");
	    goto exit;
	}
	VERBOSE("iso packet size is %d bytes", max_iso_packet_size);
	const int num_pkts  = 16;
	const int num_xfers = 4;

	usb->stream[1] = usb_stream_open(usb->handle,
					 LIBUSB_TRANSFER_TYPE_ISOCHRONOUS,
					 0x84,
					 num_xfers,
					 num_pkts, 
					 max_iso_packet_size,
					 depth_cb,
					 ctx);
	if (!usb->stream[1]) {
	    VERBOSE("failed to create isoc stream for depth data");
	    goto exit;
	}
    }

    for (ch = ctx->begin; ch <= ctx->end; ++ch) {
	usb_stream_start(usb->stream[ch]);
    }

    usb->shutdown = 0;
    if (THREAD_CREATE(&usb->thread, k4w2_libusb_thread, ctx)) {
	VERBOSE("THREAD_CREATE() failed.");
	return K4W2_ERROR;
    }

    return K4W2_SUCCESS;

exit:
    k4w2_libusb_close(ctx);
    return K4W2_ERROR;
}

static int
k4w2_libusb_start(k4w2_t ctx)
{
    k4w2_libusb * usb = (k4w2_libusb *)ctx;


    if (COLOR_ENABLED(ctx)) {
	send_cmd(usb, KCMD_SET_STREAMING, NULL, 0, 1, STREAMING_ON);
    }
    if (DEPTH_ENABLED(ctx)) {
	send_cmd(usb, KCMD_START_DEPTH, NULL, 0, 0);
    }

    return K4W2_ERROR;
}

static int
k4w2_libusb_stop(k4w2_t ctx)
{
    k4w2_libusb * usb = (k4w2_libusb *)ctx;

    if (COLOR_ENABLED(ctx)) {
	send_cmd(usb, KCMD_SET_STREAMING, NULL, 0, 1, STREAMING_OFF);
    }
    if (DEPTH_ENABLED(ctx)) {
	send_cmd(usb, KCMD_STOP_DEPTH, NULL, 0, 0);
    }

    return K4W2_ERROR;
}

static int
k4w2_libusb_close(k4w2_t ctx)
{
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
    CHANNEL ch;

    VERBOSE("stop stream(s)");
    for (ch = ctx->begin; ch <= ctx->end; ++ch) {
	VERBOSE("stop stream(%d)", ch);
	usb_stream_close(&usb->stream[ch]);
    }

    
    VERBOSE("release interface(s)");

    PERMISSIVE( libusb_release_interface(usb->handle,
					 ControlAndRgbInterfaceId) );
    if (DEPTH_ENABLED(ctx)) {
	PERMISSIVE( libusb_release_interface(usb->handle, IrInterfaceId) );
    }

    VERBOSE("stop thread()");
    if (usb->thread) {
	usb->shutdown = 1;
	THREAD_JOIN(usb->thread);
	usb->thread = 0;
    }

    if (usb->handle) {
	libusb_close(usb->handle);
    }

    if (usb->context) {
	libusb_exit(usb->context);
    }
    return K4W2_SUCCESS;
}

static int
k4w2_libusb_read_param(k4w2_t ctx, PARAM_ID id, void *param, int length)
{
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
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
