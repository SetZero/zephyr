/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB host HID class driver, boot-protocol mouse.
 *
 * Matches HID interfaces declaring the boot-mouse code triple
 * (class 3 / subclass 1 / protocol 2), switches the device to the
 * deterministic boot protocol and polls its interrupt IN endpoint.
 * Reports are delivered through the input subsystem as relative
 * pointer events (INPUT_REL_X/Y/WHEEL, INPUT_BTN_LEFT/RIGHT/MIDDLE),
 * so consumers treat the mouse exactly like any other input device.
 *
 * Boot protocol defines only the first three report bytes
 * (buttons, dX, dY). Nearly all mice append the wheel as a fourth
 * byte even in boot protocol; if a device does not, wheel events are
 * simply absent (proper support would need a HID report-descriptor
 * parser and the report protocol).
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/usbh.h>

#include "../usbh_ch9.h"
#include "../usbh_class.h"
#include "../usbh_desc.h"
#include "../usbh_device.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbh_hid, CONFIG_USBH_HID_LOG_LEVEL);

#define HID_SUBCLASS_BOOT	1U
#define HID_PROTOCOL_MOUSE	2U

/* HID class requests */
#define HID_REQ_SET_IDLE	0x0AU
#define HID_REQ_SET_PROTOCOL	0x0BU
#define HID_PROTO_BOOT		0U

struct usbh_hid_data {
	struct usb_device *udev;
	const struct usb_ep_descriptor *int_in;
	uint8_t iface;
	uint8_t prev_buttons;
	uint8_t err_streak;
	bool running;
};

static struct usbh_hid_data hid_data;

static int usbh_hid_submit_in(struct usbh_hid_data *const data);

static int usbh_hid_report_cb(struct usb_device *const udev,
			      struct uhc_transfer *const xfer)
{
	struct usbh_hid_data *const data = &hid_data;
	struct net_buf *buf = xfer->buf;

	/* Device gone: stop polling and release. */
	if (!data->running || udev != data->udev) {
		if (buf != NULL) {
			usbh_xfer_buf_free(udev, buf);
		}
		return usbh_xfer_free(udev, xfer);
	}

	/* Transient transfer error (e.g. a STALL or timeout on the
	 * interrupt IN): keep polling rather than giving up for good, but
	 * back off after repeated failures so a persistently halted
	 * endpoint cannot busy-loop. */
	if (xfer->err != 0) {
		const int xfer_err = xfer->err;

		if (buf != NULL) {
			usbh_xfer_buf_free(udev, buf);
		}
		(void)usbh_xfer_free(udev, xfer);

		if (xfer_err == -ECONNRESET) {
			/* Disconnect: stop polling now instead of retrying
			 * against a dead bus; usbh_hid_removed() finishes
			 * the cleanup and re-arms the class for a replug.
			 */
			data->running = false;
			return 0;
		}

		if (++data->err_streak > 32U) {
			LOG_WRN("Mouse poll giving up after repeated err");
			data->running = false;
			return 0;
		}
		k_msleep(4);
		return usbh_hid_submit_in(data);
	}
	data->err_streak = 0U;

	if (buf != NULL && buf->len >= 3U) {
		const uint8_t buttons = buf->data[0];
		const int8_t dx = (int8_t)buf->data[1];
		const int8_t dy = (int8_t)buf->data[2];
		const int8_t wheel = (buf->len >= 4U) ? (int8_t)buf->data[3] : 0;
		const uint8_t changed = buttons ^ data->prev_buttons;
		/* The sync flag goes on the LAST report of the batch. */
		unsigned int left = (changed & BIT(0) ? 1U : 0U) +
				    (changed & BIT(1) ? 1U : 0U) +
				    (changed & BIT(2) ? 1U : 0U) +
				    (dx != 0 ? 1U : 0U) + (dy != 0 ? 1U : 0U) +
				    (wheel != 0 ? 1U : 0U);

		data->prev_buttons = buttons;

		if (changed & BIT(0)) {
			input_report_key(NULL, INPUT_BTN_LEFT,
					 buttons & BIT(0), --left == 0U,
					 K_NO_WAIT);
		}
		if (changed & BIT(1)) {
			input_report_key(NULL, INPUT_BTN_RIGHT,
					 buttons & BIT(1), --left == 0U,
					 K_NO_WAIT);
		}
		if (changed & BIT(2)) {
			input_report_key(NULL, INPUT_BTN_MIDDLE,
					 buttons & BIT(2), --left == 0U,
					 K_NO_WAIT);
		}
		if (dx != 0) {
			input_report_rel(NULL, INPUT_REL_X, dx, --left == 0U,
					 K_NO_WAIT);
		}
		if (dy != 0) {
			input_report_rel(NULL, INPUT_REL_Y, dy, --left == 0U,
					 K_NO_WAIT);
		}
		if (wheel != 0) {
			input_report_rel(NULL, INPUT_REL_WHEEL, wheel,
					 --left == 0U, K_NO_WAIT);
		}
	}

	if (buf != NULL) {
		usbh_xfer_buf_free(udev, buf);
	}
	(void)usbh_xfer_free(udev, xfer);

	return usbh_hid_submit_in(data);
}

static int usbh_hid_submit_in(struct usbh_hid_data *const data)
{
	struct uhc_transfer *xfer;
	struct net_buf *buf;
	int err;

	xfer = usbh_xfer_alloc(data->udev, data->int_in->bEndpointAddress,
			       usbh_hid_report_cb, NULL);
	if (xfer == NULL) {
		LOG_ERR("Failed to allocate interrupt IN transfer");
		return -ENOMEM;
	}

	buf = usbh_xfer_buf_alloc(data->udev,
				  sys_le16_to_cpu(data->int_in->wMaxPacketSize));
	if (buf == NULL) {
		(void)usbh_xfer_free(data->udev, xfer);
		LOG_ERR("Failed to allocate interrupt IN buffer");
		return -ENOMEM;
	}

	err = usbh_xfer_buf_add(data->udev, xfer, buf);
	if (err != 0) {
		usbh_xfer_buf_free(data->udev, buf);
		(void)usbh_xfer_free(data->udev, xfer);
		return err;
	}

	err = usbh_xfer_enqueue(data->udev, xfer);
	if (err != 0) {
		usbh_xfer_buf_free(data->udev, buf);
		(void)usbh_xfer_free(data->udev, xfer);
	}

	return err;
}

/* Locate the interrupt IN endpoint descriptor of the matched interface */
static const struct usb_ep_descriptor *
usbh_hid_get_int_in(struct usb_device *const udev, const uint8_t iface)
{
	const struct usb_desc_header *dhp = usbh_desc_get_iface(udev, iface);

	if (dhp == NULL) {
		return NULL;
	}

	for (dhp = usbh_desc_get_next(dhp); dhp != NULL && dhp->bLength != 0U;
	     dhp = usbh_desc_get_next(dhp)) {
		if (dhp->bDescriptorType == USB_DESC_INTERFACE) {
			break;
		}

		if (dhp->bDescriptorType == USB_DESC_ENDPOINT) {
			const struct usb_ep_descriptor *ed = (const void *)dhp;

			if (USB_EP_DIR_IS_IN(ed->bEndpointAddress) &&
			    (ed->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
			    USB_EP_TYPE_INTERRUPT) {
				return ed;
			}
		}
	}

	return NULL;
}

static int usbh_hid_probe(struct usbh_class_data *const c_data,
			  struct usb_device *const udev, const uint8_t iface)
{
	struct usbh_hid_data *const data = &hid_data;
	int err;

	LOG_DBG("HID probe: interface %u", iface);

	if (data->running) {
		LOG_WRN("Boot mouse already bound, ignoring interface %u",
			iface);
		return -EBUSY;
	}

	data->udev = udev;
	data->iface = iface;
	data->prev_buttons = 0U;
	data->err_streak = 0U;

	data->int_in = usbh_hid_get_int_in(udev, iface);
	if (data->int_in == NULL) {
		LOG_ERR("No interrupt IN endpoint on interface %u", iface);
		return -ENOTSUP;
	}

	/* Deterministic report layout */
	err = usbh_req_setup(udev,
			     USB_REQTYPE_DIR_TO_DEVICE << 7 |
			     USB_REQTYPE_TYPE_CLASS << 5 |
			     USB_REQTYPE_RECIPIENT_INTERFACE,
			     HID_REQ_SET_PROTOCOL, HID_PROTO_BOOT, iface, 0,
			     NULL);
	if (err != 0) {
		/* Boot-only devices may STALL this; carry on. */
		LOG_WRN("SET_PROTOCOL(boot) failed (%d), continuing", err);
	}

	/* Report only on change (idle rate 0) */
	err = usbh_req_setup(udev,
			     USB_REQTYPE_DIR_TO_DEVICE << 7 |
			     USB_REQTYPE_TYPE_CLASS << 5 |
			     USB_REQTYPE_RECIPIENT_INTERFACE,
			     HID_REQ_SET_IDLE, 0, iface, 0, NULL);
	if (err != 0) {
		LOG_DBG("SET_IDLE failed (%d), continuing", err);
	}

	data->running = true;

	err = usbh_hid_submit_in(data);
	if (err != 0) {
		data->running = false;
		return err;
	}

	LOG_INF("Boot mouse bound: interface %u, EP 0x%02x, interval %u",
		iface, data->int_in->bEndpointAddress,
		data->int_in->bInterval);

	return 0;
}

static int usbh_hid_removed(struct usbh_class_data *const c_data)
{
	struct usbh_hid_data *const data = &hid_data;

	/* The in-flight transfer is failed by the controller driver on
	 * disconnect and freed in the completion callback.
	 */
	data->running = false;
	data->udev = NULL;
	data->int_in = NULL;

	if (data->prev_buttons != 0U) {
		/* Do not leave phantom pressed buttons behind. */
		unsigned int left = (data->prev_buttons & BIT(0) ? 1U : 0U) +
				    (data->prev_buttons & BIT(1) ? 1U : 0U) +
				    (data->prev_buttons & BIT(2) ? 1U : 0U);

		if (data->prev_buttons & BIT(0)) {
			input_report_key(NULL, INPUT_BTN_LEFT, 0,
					 --left == 0U, K_NO_WAIT);
		}
		if (data->prev_buttons & BIT(1)) {
			input_report_key(NULL, INPUT_BTN_RIGHT, 0,
					 --left == 0U, K_NO_WAIT);
		}
		if (data->prev_buttons & BIT(2)) {
			input_report_key(NULL, INPUT_BTN_MIDDLE, 0,
					 --left == 0U, K_NO_WAIT);
		}
		data->prev_buttons = 0U;
	}

	LOG_INF("Boot mouse removed");

	return 0;
}

static int usbh_hid_init(struct usbh_class_data *const c_data)
{
	ARG_UNUSED(c_data);

	return 0;
}

static struct usbh_class_api usbh_hid_api = {
	.init = usbh_hid_init,
	.probe = usbh_hid_probe,
	.removed = usbh_hid_removed,
};

static const struct usbh_class_filter usbh_hid_filters[] = {
	{
		.class = USB_BCC_HID,
		.sub = HID_SUBCLASS_BOOT,
		.proto = HID_PROTOCOL_MOUSE,
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
	},
	{
		.flags = 0,
	},
};

USBH_DEFINE_CLASS(usbh_hid, &usbh_hid_api, &hid_data, usbh_hid_filters);
