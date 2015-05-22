/**
 * @file   usb_dev.c
 * @author Hiromasa YOSHIMOTO
 * @date   Thu Apr  9 14:06:25 2015
 * 
 * @brief  
 * 
 * 
 */


#include "usb_dev.h"
#include "../module.h"

#include <pthread.h>
#include <assert.h>

void
usb_foreach_device(libusb_context *usb_context,
		   uint16_t vendor_id, uint16_t product_id,
		   usb_found_callback_t callback,
		   void *userdata)
{
    libusb_device **list;
    int num_devices;
    int idx;
    int res;

    if (!callback)
	return;

    num_devices = libusb_get_device_list(usb_context, &list);

    for(idx = 0; idx < num_devices; ++idx) {
	struct libusb_device_descriptor desc;

	res = libusb_get_device_descriptor(list[idx], &desc);
	if (LIBUSB_SUCCESS != res) {
	    VERBOSE("libusb_get_device_descriptor() returns %s",
		    libusb_error_name(res));
	    continue;
	}

	if (desc.idVendor  != vendor_id || desc.idProduct != product_id)
	    continue;

	callback(list[idx], &desc, userdata);
    }

    /* release all devices in list[] */
    libusb_free_device_list(list, 0);
}

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
