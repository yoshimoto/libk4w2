/**
 * @file   usb_dev.h
 * @author Hiromasa YOSHIMOTO
 * @date   Thu Apr  9 14:32:31 2015
 * 
 * @brief  
 * 
 * 
 */
#ifndef __USB_DEV_H_INCLUDED__
#define __USB_DEV_H_INCLUDED__

#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <stdlib.h> /* for size_t */
#include <stdarg.h>

#ifdef __cplusplus
#  define EXTERN_C_BEGIN extern "C" {
#  define EXTERN_C_END   }
#else
#  define EXTERN_C_BEGIN
#  define EXTERN_C_END
#endif

EXTERN_C_BEGIN

typedef void (*usb_found_callback_t)(libusb_device *device,
				     const struct libusb_device_descriptor *desc,
				     void *userdata);
void usb_foreach_device(libusb_context *usb_context,
			uint16_t vendor_id, uint16_t product_id,
			usb_found_callback_t callback,
			void *userdata);


/*
 * usb_stream
 *
 */
typedef struct usb_stream_ctx *usb_stream_t;
typedef void (*usb_stream_callback)(struct libusb_transfer *xfer,
				    void *callback_arg);
usb_stream_t usb_stream_open(libusb_device_handle * handle,
			     unsigned char type,
			     unsigned char endpoint,
			     int num_xfers,
			     int num_pkts,
			     int pkt_length,
			     usb_stream_callback callback,
			     void *callback_arg);
int usb_stream_set_callback(usb_stream_t strm,
			    usb_stream_callback callback,
			    void *callback_arg);
int usb_stream_start(usb_stream_t strm);
int usb_stream_stop(usb_stream_t strm);
int usb_stream_close(usb_stream_t *strm);

EXTERN_C_END

#undef EXTERN_C_BEGIN
#undef EXTERN_C_END

#endif /* #ifdef __USB_DEV_H_INCLUDED__ */

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
