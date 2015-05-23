/**
 * @file   driver_libusb.c
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 02:35:02 2015
 * 
 * @brief  libusb-1.0 backend for libk4w2
 *
 * This code is based on the OpenKinect project and libfreenect2
 * See the following URLs for details;
 * http://openkinect.org/wiki/
 *
 * This code has been tested on Linux and Mac OS X only.
 * I wish it worked on windows :-p
 * 
 */

#if ! defined WITH_LIBUSB
#  error "WITH_LIBUSB is not defined, while driver_libusb.c is compiled"
#endif

#include <assert.h>
#include <stdarg.h>
#include <time.h>   /* for nanosleep() */
#include <stddef.h> /* for offsetof() */

#include "usb_dev.h"
#include "module.h"

/* ========= framebuffer =========== */

typedef struct {
    unsigned char *pointer; /* length bytes buffer */
    int length;
} buffer_t;

typedef struct {
    buffer_t *slot; /* slot[num_slot] */
    int num_slot;
    int buf_size;
    buffer_t *next; /* a pointer to the being updated element in slot[] */
    buffer_t *last; /* a pointer to the latest element in slot[] */
} ringbuffer_t;


static void
release_ringbuf(ringbuffer_t *rg)
{
    if (rg->slot) {
	int i;
	for (i = 0; i < rg->num_slot; ++i) {
	    free(rg->slot[i].pointer);
	    rg->slot[i].pointer = NULL;
	}
	free(rg->slot);
	rg->slot = NULL;
    }
}

static int
allocate_ringbuf(ringbuffer_t *rg, int num_slot, int buf_size){
    int i;
    memset(rg, 0, sizeof(*rg));
    rg->num_slot = num_slot;
    rg->slot = (buffer_t*)malloc(num_slot * sizeof(buffer_t));
    rg->buf_size = buf_size;
    for (i = 0; i < num_slot; ++i) {
	rg->slot[i].pointer = (unsigned char*)malloc(buf_size);
	if (!rg->slot[i].pointer)
	    goto exit;
    }
    rg->next = &rg->slot[0];
    rg->next->length = 0;
    rg->last = NULL;

    return K4W2_SUCCESS;
exit:
    release_ringbuf(rg);
    return K4W2_ERROR;
}

static int
append_data(ringbuffer_t *rg, const void *pointer, int length)
{
    if (rg->buf_size < rg->next->length + length) {
	return K4W2_ERROR;
    }

    memcpy(rg->next->pointer + rg->next->length, pointer, length);
    rg->next->length += length;
    return K4W2_SUCCESS;
}

static void
commit_frame(ringbuffer_t *rg)
{
    rg->last = rg->next;
    rg->next = rg->slot + (rg->next - rg->slot + 1)%rg->num_slot;
    rg->next->length = 0;
}
static void
rollback_frame(ringbuffer_t *rg)
{
    rg->next->length = 0;
}

/* ========= driver =========== */

typedef struct {
    struct k4w2_driver_ctx k4w2;  /* !! must be the first item */

    libusb_context *context;
    libusb_device *dev;
    libusb_device_handle *handle;

    usb_stream_t stream[2];       /* 0:color/bulk stream, 1:depth/isoc stream */
    ringbuffer_t ring[2];
    unsigned depth_synced:1;

    uint32_t request_sequence;

    THREAD_T thread;              /* libusb's event loop */
    volatile unsigned shutdown:1; /* set 1 will terminate event loop */
} k4w2_libusb;

#define ControlAndRgbInterfaceId 0
#define IrInterfaceId            1

#define OUTBOUND_ENDPOINT	0x002
#define INBOUND_ENDPOINT	0x081

#define REQUEST_MAGIC		0x06022009
#define RESPONSE_MAGIC		0x0A6FE000

#define KCMD_READ_DATA_PAGE	0x022 /* read paramters */
#define KCMD_CTRL_COLOR		0x02B /* start/stop color stream */
#define CTRL_COLOR_START	0x01  /* argument for KCMD_CTRL_COLOR */
#define CTRL_COLOR_STOP		0x00  /* argument for KCMD_CTRL_COLOR */
#define KCMD_START_DEPTH	0x009 /* start depth stream */
#define KCMD_STOP_DEPTH		0x00A /* stop depth stream */

static const unsigned char inbound_endpoint  = 0x081;
static const unsigned char outbound_endpoint = 0x002;

#define CTRL_TIMEOUT 1000
#define BULK_SIZE    0x4000

#define NUM_FRAMEBUFFERS 30

#define cpu_to_le32(x) (x)

#define STRICT( exp ) do {						\
	TRACE(#exp);							\
	int r = (exp); if (LIBUSB_SUCCESS != r) {			\
	    VERBOSE(#exp " returns %s", libusb_error_name(r));		\
	    goto exit;							\
	}								\
    } while (0)
#define PERMISSIVE( exp ) do {				       \
	TRACE(#exp);					       \
	int r = (exp);					       \
	if (LIBUSB_SUCCESS != r) {			       \
	    VERBOSE(#exp " returns %s", libusb_error_name(r)); \
	}						       \
    } while (0)


static void
kinect2_found_callback(libusb_device *device,
		       const struct libusb_device_descriptor *desc,
		       void *userdata)
{
    k4w2_libusb *usb = (k4w2_libusb*)userdata;

    usb->dev = device;
}

static int
open_device(k4w2_libusb *usb, unsigned int device_id, int attempt_reset)
{
    int res;

    if (!usb->dev) {
	static const struct DeviceTable {
	    uint16_t vendor_id;
	    uint16_t product_id;
	} tbl[] = {
	    {0x045e, 0x02d8}, /* kinect for windows 2 */
	    {0x045e, 0x02c4}, /* kinect for windows 2 preview? */
	};
	int i;
	for (i = 0; i < ARRAY_SIZE(tbl); ++i) {
	    usb_foreach_device(usb->context,
			       tbl[i].vendor_id, tbl[i].product_id,
			       kinect2_found_callback,
			       usb);
	}
	if (!usb->dev) {
	    VERBOSE("no kinect2 found.");
	    goto exit;
	}
    }

    assert(NULL == usb->handle);
    STRICT( libusb_open(usb->dev, &usb->handle) );
    if (!usb->handle) {
	VERBOSE("libusb_open() failed.");
	goto exit;
    }

    /* The following lines for "attempt_reset" was taken from
     * libfreenect2/example/protonect/src/libfreenect2.cpp.
     * Thank you!!
     */
    if (attempt_reset) {
	static struct timespec wait_for_initialize = {1,0};

	res = libusb_reset_device(usb->handle);
	switch (res) {
	case LIBUSB_SUCCESS:
	    break;
	case LIBUSB_ERROR_NOT_FOUND:
	    /* From libusb documentation:
	     * "If the reset fails, the descriptors change, or the previous state
	     * cannot be restored, the device will appear to be disconnected and
	     * reconnected. This means that the device handle is no longer valid (you
	     * should close it) and rediscover the device. A return code of
	     * LIBUSB_ERROR_NOT_FOUND indicates when this is the case." 
	     */

	    /* be a good citizen */
	    libusb_close(usb->handle);
	    usb->handle = NULL;

	    /* HACK: wait for the planets to align... (When the reset fails it may
	     * take a short while for the device to show up on the bus again. In the
	     * absence of hotplug support, we just wait a little. If this code path
	     * is followed there will already be a delay opening the device fully so
	     * adding a little more is tolerable.) 
	     */
	    nanosleep(&wait_for_initialize, 0);

	    return open_device(usb, device_id, 0);
	    /* break; */
	}
    }
    return LIBUSB_SUCCESS;
exit:
    return LIBUSB_ERROR_NOT_FOUND;
}

/* The function was taken from
 * libfreenect2/example/protonect/src/libfreenect2.cpp.
 */
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
    /* for details see USB 3.1 r1 spec section 9.4.11 */

    /* !!FIXME!! delay should be calculated from USB's bus topology.
     * if no super speed hubs in between,
     * then it is equal to tTPTransmissionDelay(= 40ns)
     */
    uint16_t delay  = 40;
    PERMISSIVE( libusb_control_transfer(handle,
					LIBUSB_RECIPIENT_DEVICE,
					LIBUSB_SET_ISOCH_DELAY,
					delay, 0,
					NULL, 0,
					CTRL_TIMEOUT) );
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
	req.param[i] = cpu_to_le32( va_arg(ap, int) );
    }
    va_end(ap);

    /* Send request */
    STRICT( libusb_bulk_transfer(usb->handle, OUTBOUND_ENDPOINT,
				 (unsigned char *)&req, reqsize,
				 &done, CTRL_TIMEOUT) );
    if (done != reqsize){
	VERBOSE("%d != %d", done, reqsize);
	return -1;
    }

    /* Receive response if needed */
    if (max_response_size > 0) {
	STRICT( libusb_bulk_transfer(usb->handle, INBOUND_ENDPOINT,
				     response,
				     max_response_size,
				     &response_size, CTRL_TIMEOUT) );
    }

    /* Receive completion */
    uint32_t completion[32] = {0};
    STRICT( libusb_bulk_transfer(usb->handle, INBOUND_ENDPOINT,
				 (unsigned char*)completion,
				 sizeof(completion),
				 &done, CTRL_TIMEOUT) );

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


static void *
k4w2_libusb_thread(void *userarg)
{
    k4w2_t ctx = (k4w2_t)userarg;
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
    struct timeval t = {1, 0};

    TRACE("libusb_thread begin");

    while (!usb->shutdown) {
	libusb_handle_events_timeout_completed(usb->context, &t, 0);
    }

    TRACE("libusb_thread end");
    return NULL;
}

static void
depth_cb(struct libusb_transfer *xfer, void *userarg)
{
    k4w2_t ctx = (k4w2_t)userarg;
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
    ringbuffer_t *rg = &usb->ring[DEPTH_CH];
    const unsigned char *ptr = xfer->buffer;
    int i;
    for (i = 0; i < xfer->num_iso_packets; ++i) {
	const struct libusb_iso_packet_descriptor* d = &xfer->iso_packet_desc[i];
	if (0 < d->actual_length) {
	    if (usb->depth_synced) {
		if (K4W2_SUCCESS != append_data(rg, ptr, d->actual_length)) {
		    VERBOSE("buffer overrun!!");
		    usb->depth_synced = 0;
		    rollback_frame(rg);
		}
	    }
	    if (d->actual_length != d->length) {
		/* this packet is last one */
		const struct kinect2_depth_footer *f;
		f = (struct kinect2_depth_footer*)(ptr
						   + d->actual_length
						   - sizeof(*f));
		if (0x00 != f->magic0 ||
		    KINECT2_DEPTH_IMAGE_SIZE != f->length) {
		    VERBOSE("wrong pkt; i:%d, synced:%d, len: %d",
			    i,
			    usb->depth_synced,
			    f->length);
		    usb->depth_synced = 0;
		    rollback_frame(rg);
		} else {
		    if (9==f->subsequence) {
			usb->depth_synced = 1;
			commit_frame(rg);
			if (ctx->callback[DEPTH_CH]) {
			    ctx->callback[DEPTH_CH](rg->last->pointer, rg->last->length,
						    ctx->userdata[DEPTH_CH]);
			}
		    }
		}
	    }
	}
	ptr += d->length;
    }
}

static void
color_cb(struct libusb_transfer *xfer, void *userarg)
{
    k4w2_t ctx = (k4w2_t)userarg;
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
    ringbuffer_t *rg = &usb->ring[COLOR_CH];

    assert (0 != xfer->actual_length);

    if (BULK_SIZE != xfer->actual_length) {
	/* last packet */
	append_data(rg, xfer->buffer, xfer->actual_length);
	commit_frame(rg);
	if (ctx->callback[COLOR_CH]) {
	    ctx->callback[COLOR_CH](rg->last->pointer, rg->last->length,
				    ctx->userdata[COLOR_CH]);
	}
    } else {
	if (0 == usb->ring[COLOR_CH].next->length) {
	    /* first packet */
	    const struct kinect2_color_header *frm =
		(struct kinect2_color_header*)xfer->buffer;
	    if (0x42424242 != frm->magic) {
		VERBOSE("skip broken color packet.");
		rollback_frame(rg);
		return;
	    }
	} 
	/* first or inter packet */
	append_data(rg, xfer->buffer, xfer->actual_length);
    }
}

static int
k4w2_libusb_open(k4w2_t ctx, unsigned int device_id, unsigned int flags)
{
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
    int i;
    CHANNEL ch;
    int attempt_reset = 1; /* !0 enables attempt_reset workaround */
    int current;

    STRICT( libusb_init(&usb->context) );
    STRICT( open_device(usb, device_id, attempt_reset) );
    if (!usb->dev || !usb->handle)
	goto exit;

    STRICT( libusb_get_configuration(usb->handle, &current) );
    if (1 != current) {
	STRICT( libusb_set_configuration(usb->handle, 1) );
    }
    STRICT( libusb_claim_interface(usb->handle,
				   ControlAndRgbInterfaceId) );

    if (DEPTH_ENABLED(ctx)) {
	libusb_claim_interface(usb->handle, IrInterfaceId) ;
	set_isochronous_delay(usb->handle);
	STRICT(libusb_set_interface_alt_setting(usb->handle,
						IrInterfaceId, 1));
    }

    if (COLOR_ENABLED(ctx)) {
	const int num_xfers = 16;
	usb->stream[0] = usb_stream_open(usb->handle,
					 LIBUSB_TRANSFER_TYPE_BULK,
					 0x83,
					 num_xfers,
					 1,
					 BULK_SIZE,
					 color_cb,
					 ctx);
	if (!usb->stream[0]) {
	    VERBOSE("failed to create bulk stream for color data");
	    goto exit;
	}

	if (K4W2_SUCCESS != allocate_ringbuf(&usb->ring[0], NUM_FRAMEBUFFERS,
					     64*0x4000) ) {
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
	const int num_pkts  = 10;
	const int num_xfers = 32;

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

    	if (K4W2_SUCCESS != allocate_ringbuf(&usb->ring[1], NUM_FRAMEBUFFERS,
					     KINECT2_DEPTH_FRAME_SIZE*10)) {
	    goto exit;
	}
    }


    usb->shutdown = 0;
    if (THREAD_CREATE(&usb->thread, k4w2_libusb_thread, ctx)) {
	VERBOSE("THREAD_CREATE() failed.");
	goto exit;
    }

    for (ch = ctx->begin; ch <= ctx->end; ++ch)
	usb_stream_start(usb->stream[ch]);

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
	VERBOSE("start color");
	send_cmd(usb, KCMD_CTRL_COLOR, NULL, 0, 1, CTRL_COLOR_START);
    }
    if (DEPTH_ENABLED(ctx)) {
	VERBOSE("start depth");
	send_cmd(usb, KCMD_START_DEPTH, NULL, 0, 0);
    }

    return K4W2_SUCCESS;
}

static int
k4w2_libusb_stop(k4w2_t ctx)
{
    k4w2_libusb * usb = (k4w2_libusb *)ctx;

    if (COLOR_ENABLED(ctx)) {
	usb_stream_stop(usb->stream[COLOR_CH]);
	send_cmd(usb, KCMD_CTRL_COLOR, NULL, 0, 1, CTRL_COLOR_STOP);
    }
    if (DEPTH_ENABLED(ctx)) {
	usb_stream_stop(usb->stream[DEPTH_CH]);
	send_cmd(usb, KCMD_STOP_DEPTH, NULL, 0, 0);
    }

    return K4W2_SUCCESS;
}

static int
k4w2_libusb_close(k4w2_t ctx)
{
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
    CHANNEL ch;
    VERBOSE("k4w2_libusb_close");

    for (ch = ctx->begin; ch <= ctx->end; ++ch)
	usb_stream_close(&usb->stream[ch]);

    PERMISSIVE( libusb_release_interface(usb->handle,
					 ControlAndRgbInterfaceId) );
    if (DEPTH_ENABLED(ctx)) {
	PERMISSIVE( libusb_set_interface_alt_setting(usb->handle,
						     IrInterfaceId, 0) );
	PERMISSIVE( libusb_release_interface(usb->handle, IrInterfaceId) );
    }

    if (usb->thread) {
	usb->shutdown = 1;
	THREAD_JOIN(usb->thread);
	usb->thread = 0;
    }

    if (usb->handle)
	libusb_close(usb->handle);

    if (usb->context)
	libusb_exit(usb->context);
    return K4W2_SUCCESS;
}

static int
k4w2_libusb_read_param(k4w2_t ctx, PARAM_ID id, void *param, int length)
{
    k4w2_libusb * usb = (k4w2_libusb *)ctx;
    int res = K4W2_ERROR;

    static const struct cmd_tbl {
	unsigned long cmd;
	int len;
    } tbl[NUM_PARAMS] = {
	{0x04, sizeof(struct kinect2_color_camera_param)},
	{0x03, sizeof(struct kinect2_depth_camera_param)},
	{0x02, sizeof(struct kinect2_p0table)},
    };
    if (0 <= id && id < NUM_PARAMS) {
	if (length != tbl[id].len) {
	    goto exit;
	}
	if (length == send_cmd(usb, KCMD_READ_DATA_PAGE,
			       param, length,
			       1, tbl[id].cmd)) {
	    /* !!FIXME!! response data should be verified here */
	    res = K4W2_SUCCESS;
	}
    }
exit:
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
