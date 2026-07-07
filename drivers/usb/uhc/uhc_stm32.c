/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * UHC (USB host controller) driver for STM32 OTG_HS instances,
 * layered on the STM32Cube HAL HCD driver — the host-mode mirror of
 * udc_stm32.c. Written for (and so far only exercised on) the
 * STM32N6's second OTG instance with the embedded USBPHYC HS PHY.
 *
 * Model: the host stack enqueues uhc_transfer items; a per-instance
 * driver thread owns all port state handling (attach debounce, reset,
 * speed detection) and transfer scheduling. Control transfers run
 * their SETUP/DATA/STATUS stages on a fixed pair of HAL host channels
 * (0 = OUT, 1 = IN); interrupt/bulk pipes get a dedicated channel
 * each, so a NAK-polling interrupt endpoint (HID) does not block
 * control traffic. The HAL tracks per-channel data toggles as long as
 * the channel is not re-initialized; control PIDs are managed inside
 * HAL_HCD_HC_SubmitRequest.
 */

#define DT_DRV_COMPAT st_stm32_otghs_uhc

#include <soc.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <stm32_ll_pwr.h>

#include "uhc_common.h"
#include <stm32_usb_common.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc_stm32, CONFIG_UHC_DRIVER_LOG_LEVEL);

/* HAL host channel allocation */
#define CH_CTRL_OUT	0U
#define CH_CTRL_IN	1U
#define CH_PIPE_FIRST	2U
#define CH_COUNT	16U

/* Driver thread event bits */
#define EV_CONNECT	BIT(0)
#define EV_DISCONNECT	BIT(1)
#define EV_KICK		BIT(2)	/* new transfer enqueued */
#define EV_URB(ch)	BIT(8 + (ch))
#define EV_URB_MASK	GENMASK(8 + CH_COUNT - 1, 8)
#define EV_ANY		(EV_CONNECT | EV_DISCONNECT | EV_KICK | EV_URB_MASK)

/* How long a NAKed interrupt IN pipe waits before the next attempt
 * when the endpoint's bInterval is zero/invalid.
 */
#define INTR_INTERVAL_FALLBACK_MS 8U

struct uhc_stm32_pipe {
	/* Transfer currently owning this channel, NULL = channel free */
	struct uhc_transfer *xfer;
	/* Endpoint address (with direction bit) the channel is set up for */
	uint8_t ep;
	/* Device address the channel is set up for */
	uint8_t addr;
	/* Earliest uptime [ms] for the next (re)submission (interrupt EPs) */
	int64_t next_submit;
	/* Waiting for a URB completion on this channel */
	bool busy;
};

struct uhc_stm32_data {
	const struct device *dev;
	HCD_HandleTypeDef hcd;

	struct k_thread thread;
	struct k_event event;

	/* Port/device state, driver thread context only */
	bool connected;
	enum usb_device_speed speed;

	/* Active control transfer and its channel-pair bookkeeping */
	struct uhc_transfer *ctrl_xfer;
	bool ctrl_busy;

	/* Dedicated pipes (interrupt/bulk), index = HAL channel number */
	struct uhc_stm32_pipe pipe[CH_COUNT];
};

struct uhc_stm32_config {
	USB_OTG_GlobalTypeDef *base;
	const struct stm32_pclken *pclken;
	size_t num_clocks;
	const struct stm32_usb_phy *phy;
	void (*irq_connect_func)(void);
	uint32_t irqn;
	k_thread_stack_t *thread_stack;
	size_t thread_stack_size;
	uint8_t num_host_channels;
	bool hs_capable;
};

/*
 * ISR-side HAL callbacks. The HCD handle's pData carries the device
 * pointer. These only translate HAL events into driver thread events.
 */

static inline struct uhc_stm32_data *hhcd2priv(HCD_HandleTypeDef *hhcd)
{
	return (struct uhc_stm32_data *)uhc_get_private((const struct device *)hhcd->pData);
}

void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
	struct uhc_stm32_data *priv = hhcd2priv(hhcd);

	k_event_post(&priv->event, EV_CONNECT);
}

void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{
	struct uhc_stm32_data *priv = hhcd2priv(hhcd);

	k_event_post(&priv->event, EV_DISCONNECT);
}

void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
					 HCD_URBStateTypeDef urb_state)
{
	struct uhc_stm32_data *priv = hhcd2priv(hhcd);

	if (chnum >= CH_COUNT || urb_state == URB_IDLE) {
		return;
	}

	/* Wake the driver thread to poll HAL_HCD_HC_GetURBState for this
	 * channel; the event is only a low-latency hint (the poll is the
	 * source of truth). */
	k_event_post(&priv->event, EV_URB(chnum));
}

static void uhc_stm32_isr(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	HAL_HCD_IRQHandler(&priv->hcd);
}

/*
 * Helpers, driver thread context.
 */

static uint8_t udev_hal_speed(const struct uhc_stm32_data *priv,
			      const struct usb_device *udev)
{
	/* The stack only talks to the root device (no hub support), so
	 * the port speed applies. Kept as a helper for future splits.
	 */
	ARG_UNUSED(udev);

	switch (priv->speed) {
	case USB_SPEED_SPEED_HS:
		return HCD_DEVICE_SPEED_HIGH;
	case USB_SPEED_SPEED_LS:
		return HCD_DEVICE_SPEED_LOW;
	default:
		return HCD_DEVICE_SPEED_FULL;
	}
}

static int urb_to_errno(HCD_URBStateTypeDef state)
{
	switch (state) {
	case URB_DONE:
		return 0;
	case URB_STALL:
		return -EPIPE;
	case URB_NOTREADY:
	case URB_NYET:
		return -EAGAIN;
	case URB_ERROR:
	default:
		return -EIO;
	}
}

/*
 * Submit one chunk on a channel and poll HAL for its terminal URB
 * state. Polling HAL_HCD_HC_GetURBState (updated by the HAL channel ISR)
 * is the source of truth — the ST examples do the same. An earlier
 * event-based scheme raced: HAL fires NotifyURBChange several times per
 * transaction (NAK, then done), and a stale wakeup event let the next
 * attempt observe the freshly-reset IDLE state as a bogus error.
 * Retries NAKed stages (URB_NOTREADY) with a small backoff, since HAL
 * halts the channel on each NAK.
 */
static int ctrl_stage_xact(struct uhc_stm32_data *priv, uint8_t ch,
			   uint8_t direction, uint8_t token,
			   uint8_t *buf, uint16_t len)
{
	for (unsigned int attempt = 0U; attempt < 200U; attempt++) {
		HCD_URBStateTypeDef st = URB_IDLE;
		int err;

		if (HAL_HCD_HC_SubmitRequest(&priv->hcd, ch, direction,
					     EP_TYPE_CTRL, token, buf, len,
					     0U) != HAL_OK) {
			return -EIO;
		}

		/* HAL_HCD_HC_SubmitRequest resets urb_state to IDLE; wait
		 * for its channel ISR to advance it (~ms for a FS EP0). */
		for (unsigned int i = 0U; i < 500U; i++) {
			st = HAL_HCD_HC_GetURBState(&priv->hcd, ch);
			if (st != URB_IDLE) {
				break;
			}
			if (!priv->connected) {
				return -ECONNRESET;
			}
			k_msleep(1);
		}

		if (st == URB_IDLE) {
			LOG_ERR("ctrl xact ch%u tok%u dir%u len%u: timeout",
				ch, token, direction, len);
			return -ETIMEDOUT;
		}

		err = urb_to_errno(st);
		if (err != -EAGAIN) {
			return err;
		}

		if (!priv->connected) {
			return -ECONNRESET;
		}
		k_msleep(1);
	}

	return -ETIMEDOUT;
}

static int ctrl_open_channels(struct uhc_stm32_data *priv,
			      struct uhc_transfer *const xfer)
{
	const uint8_t speed = udev_hal_speed(priv, xfer->udev);
	const uint8_t addr = xfer->udev != NULL ? xfer->udev->addr : 0U;

	if (HAL_HCD_HC_Init(&priv->hcd, CH_CTRL_OUT, 0x00U, addr, speed,
			    EP_TYPE_CTRL, xfer->mps) != HAL_OK) {
		return -EIO;
	}
	if (HAL_HCD_HC_Init(&priv->hcd, CH_CTRL_IN, 0x80U, addr, speed,
			    EP_TYPE_CTRL, xfer->mps) != HAL_OK) {
		return -EIO;
	}

	return 0;
}

/*
 * Run a complete control transfer (all three stages, blocking within
 * the driver thread). SET_ADDRESS is special-cased: the channels are
 * (re)opened with the device's address as tracked by the host stack,
 * which updates udev->addr only after this transfer succeeds.
 */
static int process_control_xfer(struct uhc_stm32_data *priv,
				struct uhc_transfer *const xfer)
{
	struct usb_setup_packet *setup =
		(struct usb_setup_packet *)xfer->setup_pkt;
	const bool data_in = (setup->bmRequestType & 0x80U) != 0U;
	struct net_buf *buf = xfer->buf;
	int err;

	err = ctrl_open_channels(priv, xfer);
	if (err != 0) {
		return err;
	}

	/* SETUP stage */
	err = ctrl_stage_xact(priv, CH_CTRL_OUT, 0U, 0U, xfer->setup_pkt, 8U);
	if (err != 0) {
		LOG_DBG("SETUP stage failed (%d)", err);
		return err;
	}

	/* DATA stage */
	if (buf != NULL && setup->wLength > 0U) {
		if (data_in) {
			uint16_t want = MIN(setup->wLength,
					    net_buf_tailroom(buf));

			err = ctrl_stage_xact(priv, CH_CTRL_IN, 1U, 1U,
					      net_buf_tail(buf), want);
			if (err == 0) {
				uint32_t got = HAL_HCD_HC_GetXferCount(&priv->hcd,
								       CH_CTRL_IN);

				net_buf_add(buf, MIN(got, want));
			}
		} else {
			err = ctrl_stage_xact(priv, CH_CTRL_OUT, 0U, 1U,
					      buf->data, buf->len);
		}

		if (err != 0) {
			LOG_DBG("DATA stage failed (%d)", err);
			return err;
		}
	}

	/* STATUS stage (opposite direction; IN status after OUT/no-data) */
	if (xfer->no_status) {
		return 0;
	}

	if (buf != NULL && setup->wLength > 0U && data_in) {
		err = ctrl_stage_xact(priv, CH_CTRL_OUT, 0U, 1U, NULL, 0U);
	} else {
		err = ctrl_stage_xact(priv, CH_CTRL_IN, 1U, 1U, NULL, 0U);
	}
	if (err != 0) {
		LOG_DBG("STATUS stage failed (%d)", err);
	}

	return err;
}

/*
 * Dedicated pipes (interrupt, bulk).
 */

static struct uhc_stm32_pipe *pipe_find(struct uhc_stm32_data *priv,
					uint8_t addr, uint8_t ep)
{
	for (uint8_t ch = CH_PIPE_FIRST; ch < CH_COUNT; ch++) {
		if (priv->pipe[ch].xfer != NULL &&
		    priv->pipe[ch].ep == ep && priv->pipe[ch].addr == addr) {
			return &priv->pipe[ch];
		}
	}

	return NULL;
}

static struct uhc_stm32_pipe *pipe_claim(struct uhc_stm32_data *priv,
					 struct uhc_transfer *const xfer)
{
	const uint8_t addr = xfer->udev != NULL ? xfer->udev->addr : 0U;

	for (uint8_t ch = CH_PIPE_FIRST; ch < CH_COUNT; ch++) {
		struct uhc_stm32_pipe *pipe = &priv->pipe[ch];

		if (pipe->xfer != NULL) {
			continue;
		}

		/* (Re)initialize the channel only if it was last used
		 * for a different endpoint/address: HC_Init resets the
		 * HAL's data toggle tracking, which must survive
		 * between interrupt IN URBs.
		 */
		if (pipe->ep != xfer->ep || pipe->addr != addr) {
			if (HAL_HCD_HC_Init(&priv->hcd, ch, xfer->ep, addr,
					    udev_hal_speed(priv, xfer->udev),
					    xfer->type, xfer->mps) != HAL_OK) {
				return NULL;
			}
			pipe->ep = xfer->ep;
			pipe->addr = addr;
			pipe->next_submit = 0;
		}

		pipe->xfer = xfer;
		pipe->busy = false;

		return pipe;
	}

	return NULL;
}

static uint8_t pipe_channel(struct uhc_stm32_data *priv,
			    const struct uhc_stm32_pipe *pipe)
{
	return (uint8_t)ARRAY_INDEX(priv->pipe, pipe);
}

static void pipe_submit(struct uhc_stm32_data *priv, struct uhc_stm32_pipe *pipe)
{
	struct uhc_transfer *xfer = pipe->xfer;
	const uint8_t ch = pipe_channel(priv, pipe);
	const bool is_in = USB_EP_DIR_IS_IN(xfer->ep);
	uint8_t *data;
	uint16_t len;

	if (is_in) {
		data = net_buf_tail(xfer->buf);
		len = MIN(xfer->buf->size - xfer->buf->len, xfer->mps);
	} else {
		data = xfer->buf->data;
		len = xfer->buf->len;
	}

	pipe->busy = true;

	if (HAL_HCD_HC_SubmitRequest(&priv->hcd, ch, is_in ? 1U : 0U,
				     xfer->type, 1U, data, len, 0U) != HAL_OK) {
		pipe->busy = false;
		pipe->xfer = NULL;
		uhc_xfer_return(priv->dev, xfer, -EIO);
	}
}

static void pipe_urb_done(struct uhc_stm32_data *priv, uint8_t ch)
{
	struct uhc_stm32_pipe *pipe = &priv->pipe[ch];
	struct uhc_transfer *xfer = pipe->xfer;
	HCD_URBStateTypeDef state;
	int err;

	if (xfer == NULL || !pipe->busy) {
		return;
	}

	/* Poll HAL for the truth; a wakeup can arrive before the URB
	 * actually reaches a terminal state. */
	state = HAL_HCD_HC_GetURBState(&priv->hcd, ch);
	if (state == URB_IDLE) {
		return;
	}
	pipe->busy = false;

	err = urb_to_errno(state);
	if (err == -EAGAIN) {
		/* NAK: retry after the endpoint's interval. */
		uint16_t interval = xfer->interval;

		if (interval == 0U) {
			interval = INTR_INTERVAL_FALLBACK_MS;
		}
		pipe->next_submit = k_uptime_get() + interval;
		return;
	}

	if (err == 0 && USB_EP_DIR_IS_IN(xfer->ep)) {
		uint32_t got = HAL_HCD_HC_GetXferCount(&priv->hcd, ch);

		net_buf_add(xfer->buf,
			    MIN(got, xfer->buf->size - xfer->buf->len));
	}

	pipe->xfer = NULL;
	uhc_xfer_return(priv->dev, xfer, err);
}

/*
 * Port state handling, driver thread context.
 */

static void handle_connect(struct uhc_stm32_data *priv)
{
	uint32_t hal_speed;

	if (priv->connected) {
		return;
	}

	/* Attach debounce (USB 2.0: TATTDB 100 ms) */
	k_msleep(100);

	/* The HAL reset drives the port reset signaling and enables the
	 * port; only afterwards is the negotiated speed known.
	 */
	if (HAL_HCD_ResetPort(&priv->hcd) != HAL_OK) {
		LOG_ERR("Port reset failed");
		uhc_submit_event(priv->dev, UHC_EVT_ERROR, -EIO);
		return;
	}
	k_msleep(30);

	hal_speed = HAL_HCD_GetCurrentSpeed(&priv->hcd);
	priv->connected = true;

	switch (hal_speed) {
	case HCD_DEVICE_SPEED_HIGH:
		priv->speed = USB_SPEED_SPEED_HS;
		uhc_submit_event(priv->dev, UHC_EVT_DEV_CONNECTED_HS, 0);
		break;
	case HCD_DEVICE_SPEED_LOW:
		priv->speed = USB_SPEED_SPEED_LS;
		uhc_submit_event(priv->dev, UHC_EVT_DEV_CONNECTED_LS, 0);
		break;
	default:
		priv->speed = USB_SPEED_SPEED_FS;
		uhc_submit_event(priv->dev, UHC_EVT_DEV_CONNECTED_FS, 0);
		break;
	}

	LOG_INF("Device connected, speed %u", (unsigned int)hal_speed);
}

static void handle_disconnect(struct uhc_stm32_data *priv)
{
	struct uhc_data *data = priv->dev->data;
	struct uhc_transfer *xfer;
	sys_dnode_t *node;

	if (!priv->connected) {
		return;
	}
	priv->connected = false;

	/* Fail the active and all still-queued transfers. */
	if (priv->ctrl_xfer != NULL) {
		/* The control machine notices !connected and returns the
		 * transfer itself; nothing to do here.
		 */
	}

	for (uint8_t ch = CH_PIPE_FIRST; ch < CH_COUNT; ch++) {
		struct uhc_stm32_pipe *pipe = &priv->pipe[ch];

		if (pipe->xfer != NULL) {
			(void)HAL_HCD_HC_Halt(&priv->hcd, ch);
			xfer = pipe->xfer;
			pipe->xfer = NULL;
			uhc_xfer_return(priv->dev, xfer, -ECONNRESET);
		}

		/* Force re-init on next use, idle channels included: a
		 * re-attached device typically gets the same address and
		 * endpoints again and must not inherit this channel's
		 * data toggle from the previous device.
		 */
		pipe->busy = false;
		pipe->ep = 0U;
		pipe->addr = 0U;
		pipe->next_submit = 0;
	}

	while ((node = sys_dlist_peek_head(&data->ctrl_xfers)) != NULL) {
		xfer = SYS_DLIST_CONTAINER(node, xfer, node);
		uhc_xfer_return(priv->dev, xfer, -ECONNRESET);
	}

	LOG_INF("Device removed");
	uhc_submit_event(priv->dev, UHC_EVT_DEV_REMOVED, 0);
}

/*
 * Transfer scheduling, driver thread context.
 *
 * All enqueued transfers sit in uhc_data.ctrl_xfers (uhc_xfer_append
 * puts everything there) until uhc_xfer_return() removes them. Claimed
 * transfers are tracked in priv (ctrl_xfer / pipe[].xfer); this walk
 * picks up the not-yet-claimed remainder.
 */

static bool xfer_is_claimed(struct uhc_stm32_data *priv,
			    struct uhc_transfer *const xfer)
{
	if (priv->ctrl_xfer == xfer) {
		return true;
	}

	for (uint8_t ch = CH_PIPE_FIRST; ch < CH_COUNT; ch++) {
		if (priv->pipe[ch].xfer == xfer) {
			return true;
		}
	}

	return false;
}

static void schedule_xfers(struct uhc_stm32_data *priv)
{
	struct uhc_data *data = priv->dev->data;
	struct uhc_transfer *xfer, *tmp;

	SYS_DLIST_FOR_EACH_CONTAINER_SAFE(&data->ctrl_xfers, xfer, tmp, node) {
		if (xfer_is_claimed(priv, xfer)) {
			continue;
		}

		if (!priv->connected) {
			uhc_xfer_return(priv->dev, xfer, -ECONNRESET);
			continue;
		}

		if (xfer->ep == 0x00U || xfer->ep == 0x80U) {
			int err;

			/* Control transfers run to completion here, one
			 * at a time; interrupt pipes stay serviced via
			 * their own channels/events meanwhile.
			 */
			priv->ctrl_xfer = xfer;
			err = process_control_xfer(priv, xfer);
			priv->ctrl_xfer = NULL;
			uhc_xfer_return(priv->dev, xfer, err);
		} else if (xfer->type == EP_TYPE_INTR ||
			   xfer->type == EP_TYPE_BULK) {
			struct uhc_stm32_pipe *pipe;

			if (pipe_find(priv, xfer->udev != NULL ?
				      xfer->udev->addr : 0U, xfer->ep) != NULL) {
				/* Pipe still busy with a previous URB for
				 * this endpoint; leave queued.
				 */
				continue;
			}

			pipe = pipe_claim(priv, xfer);
			if (pipe == NULL) {
				uhc_xfer_return(priv->dev, xfer, -ENODEV);
				continue;
			}
			pipe_submit(priv, pipe);
		} else {
			LOG_ERR("Unsupported EP type %u", xfer->type);
			uhc_xfer_return(priv->dev, xfer, -ENOTSUP);
		}
	}
}

static k_timeout_t next_deadline(struct uhc_stm32_data *priv)
{
	int64_t now = k_uptime_get();
	int64_t nearest = INT64_MAX;

	for (uint8_t ch = CH_PIPE_FIRST; ch < CH_COUNT; ch++) {
		struct uhc_stm32_pipe *pipe = &priv->pipe[ch];

		if (pipe->xfer == NULL) {
			continue;
		}
		if (pipe->busy) {
			/* Poll an in-flight URB promptly for completion. */
			return K_MSEC(1);
		}
		nearest = MIN(nearest, pipe->next_submit);
	}

	if (nearest == INT64_MAX) {
		/* Cap idle wait so the HPRT-based connect poll keeps running. */
		return K_MSEC(250);
	}

	return (nearest <= now) ? K_NO_WAIT : K_MSEC(MIN(nearest - now, 250));
}

static void service_pipes(struct uhc_stm32_data *priv)
{
	int64_t now = k_uptime_get();

	for (uint8_t ch = CH_PIPE_FIRST; ch < CH_COUNT; ch++) {
		struct uhc_stm32_pipe *pipe = &priv->pipe[ch];

		if (pipe->xfer != NULL && !pipe->busy &&
		    pipe->next_submit <= now && priv->connected) {
			pipe_submit(priv, pipe);
		}
	}
}

static void uhc_stm32_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	uint32_t ev;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		ev = k_event_wait(&priv->event, EV_ANY, false,
				  next_deadline(priv));
		k_event_clear(&priv->event,
			      ev & (EV_CONNECT | EV_DISCONNECT | EV_KICK));

		/* Poll-based connect detection (robust for a device that is
		 * already attached when the host starts: there is no connect
		 * edge, so HAL_HCD_Connect_Callback never fires). HPRT.PCSTS
		 * (bit 0) = device present. handle_connect/disconnect are
		 * idempotent (guarded by priv->connected). */
		{
			USB_OTG_GlobalTypeDef *b = priv->hcd.Instance;
			uint32_t hprt = *(volatile uint32_t *)((uint32_t)b +
					USB_OTG_HOST_PORT_BASE);
			bool present = (hprt & 1U) != 0U;

			if (present && !priv->connected) {
				handle_connect(priv);
			} else if (!present && priv->connected) {
				handle_disconnect(priv);
			}
		}

		if (ev & EV_DISCONNECT) {
			handle_disconnect(priv);
		}

		if (ev & EV_CONNECT) {
			handle_connect(priv);
		}

		/* Poll every busy pipe for completion (pipe_urb_done reads
		 * HAL and is a no-op while the URB is still in flight). The
		 * EV_URB events are only wakeup hints now. */
		k_event_clear(&priv->event, EV_URB_MASK);
		for (uint8_t ch = CH_PIPE_FIRST; ch < CH_COUNT; ch++) {
			pipe_urb_done(priv, ch);
		}

		schedule_xfers(priv);
		service_pipes(priv);
	}
}

/*
 * UHC API implementation.
 */

static int uhc_stm32_lock(const struct device *dev)
{
	return uhc_lock_internal(dev, K_FOREVER);
}

static int uhc_stm32_unlock(const struct device *dev)
{
	return uhc_unlock_internal(dev);
}

static int uhc_stm32_init(const struct device *dev)
{
	const struct uhc_stm32_config *cfg = dev->config;
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	int err;

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	/* VddUSB supply + monitor. Shared with any device-mode
	 * controller (e.g. the CDC console on the other OTG instance);
	 * enable is refcounted/idempotent and this driver never
	 * disables it.
	 */
	err = stm32_usb_pwr_enable();
	if (err != 0) {
		LOG_ERR("Failed to enable USB PWR (%d)", err);
		return err;
	}

	/* PHY reference clock source selection (second clocks element) */
	if (cfg->num_clocks > 1U) {
		err = clock_control_configure(clk,
					      (clock_control_subsys_t)&cfg->pclken[1],
					      NULL);
		if (err != 0) {
			LOG_ERR("Failed to configure PHY ref clock (%d)", err);
			return err;
		}
	}

	/* OTG core clock gate — must be on before any PHY register
	 * access (the PHY controller MMIO is gated by the same bit).
	 */
	err = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (err != 0) {
		LOG_ERR("Failed to enable OTG clock (%d)", err);
		return err;
	}

	/* Embedded HS PHY (USBPHYC) configuration + clock */
	if (cfg->phy != NULL) {
		err = cfg->phy->enable(cfg->phy);
		if (err != 0) {
			LOG_ERR("Failed to enable USB PHY (%d)", err);
			return err;
		}
	}

	priv->hcd.Instance = cfg->base;
	priv->hcd.Init.Host_channels = cfg->num_host_channels;
	priv->hcd.Init.speed = cfg->hs_capable ? HCD_SPEED_HIGH : HCD_SPEED_FULL;
	priv->hcd.Init.dma_enable = DISABLE;
	/* Must be USB_OTG_HS_EMBEDDED_PHY (3): the N6 HAL's USB_CoreInit
	 * only accepts that value for the on-chip USBPHYC. The legacy
	 * HCD_PHY_EMBEDDED (2) alias is for the old FS PHY on other
	 * families and makes USB_CoreInit fail with HAL_ERROR.
	 */
	priv->hcd.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
	priv->hcd.Init.Sof_enable = DISABLE;
	priv->hcd.Init.low_power_enable = DISABLE;
	priv->hcd.Init.vbus_sensing_enable = DISABLE;
	priv->hcd.Init.use_external_vbus = ENABLE;
	priv->hcd.pData = (void *)dev;

	if (HAL_HCD_Init(&priv->hcd) != HAL_OK) {
		LOG_ERR("HAL_HCD_Init failed");
		return -EIO;
	}

	LOG_DBG("HCD initialized (%s-speed capable)",
		cfg->hs_capable ? "high" : "full");

	return 0;
}

static int uhc_stm32_enable(const struct device *dev)
{
	const struct uhc_stm32_config *cfg = dev->config;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	if (HAL_HCD_Start(&priv->hcd) != HAL_OK) {
		return -EIO;
	}

	irq_enable(cfg->irqn);

	return 0;
}

static int uhc_stm32_disable(const struct device *dev)
{
	const struct uhc_stm32_config *cfg = dev->config;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	irq_disable(cfg->irqn);

	if (HAL_HCD_Stop(&priv->hcd) != HAL_OK) {
		return -EIO;
	}

	return 0;
}

static int uhc_stm32_shutdown(const struct device *dev)
{
	const struct uhc_stm32_config *cfg = dev->config;
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (HAL_HCD_DeInit(&priv->hcd) != HAL_OK) {
		return -EIO;
	}

	if (cfg->phy != NULL) {
		(void)cfg->phy->disable(cfg->phy);
	}

	/* Note: VddUSB stays enabled on purpose (shared supply, see init). */

	return clock_control_off(clk, (clock_control_subsys_t)&cfg->pclken[0]);
}

static int uhc_stm32_bus_reset(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	/* Synchronous port reset; the host stack calls this once before
	 * enumerating the root device. HAL_HCD_ResetPort blocks for the
	 * reset signaling duration.
	 */
	if (HAL_HCD_ResetPort(&priv->hcd) != HAL_OK) {
		return -EIO;
	}
	k_msleep(30);

	return uhc_submit_event(dev, UHC_EVT_RESETED, 0);
}

static int uhc_stm32_sof_enable(const struct device *dev)
{
	/* SOF generation runs whenever the port is enabled. */
	ARG_UNUSED(dev);

	return 0;
}

static int uhc_stm32_bus_suspend(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static int uhc_stm32_bus_resume(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static int uhc_stm32_ep_enqueue(const struct device *dev,
				struct uhc_transfer *const xfer)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	int err;

	err = uhc_xfer_append(dev, xfer);
	if (err != 0) {
		return err;
	}

	k_event_post(&priv->event, EV_KICK);

	return 0;
}

static int uhc_stm32_ep_dequeue(const struct device *dev,
				struct uhc_transfer *const xfer)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(xfer);

	return -ENOTSUP;
}

static const struct uhc_api uhc_stm32_api = {
	.lock = uhc_stm32_lock,
	.unlock = uhc_stm32_unlock,
	.init = uhc_stm32_init,
	.enable = uhc_stm32_enable,
	.disable = uhc_stm32_disable,
	.shutdown = uhc_stm32_shutdown,
	.bus_reset = uhc_stm32_bus_reset,
	.sof_enable = uhc_stm32_sof_enable,
	.bus_suspend = uhc_stm32_bus_suspend,
	.bus_resume = uhc_stm32_bus_resume,
	.ep_enqueue = uhc_stm32_ep_enqueue,
	.ep_dequeue = uhc_stm32_ep_dequeue,
};

static int uhc_stm32_driver_preinit(const struct device *dev)
{
	const struct uhc_stm32_config *cfg = dev->config;
	struct uhc_data *data = dev->data;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	priv->dev = dev;
	data->caps.hs = cfg->hs_capable ? 1 : 0;
	k_mutex_init(&data->mutex);
	k_event_init(&priv->event);

	k_thread_create(&priv->thread, cfg->thread_stack,
			cfg->thread_stack_size, uhc_stm32_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_PREEMPT(CONFIG_UHC_STM32_THREAD_PRIORITY),
			0, K_NO_WAIT);
	k_thread_name_set(&priv->thread, dev->name);

	cfg->irq_connect_func();

	return 0;
}

#define UHC_STM32_INIT(n)							\
	static void uhc_stm32_irq_connect_##n(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    uhc_stm32_isr, DEVICE_DT_INST_GET(n), 0);		\
	}									\
										\
	static K_KERNEL_STACK_DEFINE(uhc_stm32_stack_##n,			\
				     CONFIG_UHC_STM32_THREAD_STACK_SIZE);	\
										\
	static const struct stm32_pclken uhc_stm32_pclken_##n[] =		\
		STM32_DT_INST_CLOCKS(n);					\
										\
	static const struct uhc_stm32_config uhc_stm32_cfg_##n = {		\
		.base = (USB_OTG_GlobalTypeDef *)DT_INST_REG_ADDR(n),		\
		.pclken = uhc_stm32_pclken_##n,					\
		.num_clocks = DT_INST_NUM_CLOCKS(n),				\
		.phy = USB_STM32_PHY_PSEUDODEV_GET_OR_NULL(DT_DRV_INST(n)),	\
		.irq_connect_func = uhc_stm32_irq_connect_##n,			\
		.irqn = DT_INST_IRQN(n),					\
		.thread_stack = uhc_stm32_stack_##n,				\
		.thread_stack_size = K_KERNEL_STACK_SIZEOF(uhc_stm32_stack_##n), \
		.num_host_channels = CH_COUNT,					\
		.hs_capable = DT_INST_ENUM_HAS_VALUE(n, maximum_speed,		\
						     high_speed),		\
	};									\
										\
	static struct uhc_stm32_data uhc_stm32_priv_##n;			\
										\
	static struct uhc_data uhc_stm32_data_##n = {				\
		.priv = &uhc_stm32_priv_##n,					\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, uhc_stm32_driver_preinit, NULL,		\
			      &uhc_stm32_data_##n, &uhc_stm32_cfg_##n,		\
			      POST_KERNEL,					\
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,		\
			      &uhc_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(UHC_STM32_INIT)
