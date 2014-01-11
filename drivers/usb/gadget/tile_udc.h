/*
 * Copyright 2013 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 *
 * Tile-Gx on-chip USB device controller.
 */

#ifndef _TILE_UDC_H
#define _TILE_UDC_H

#include <linux/usb/tilegx.h>

struct tile_ep {
	spinlock_t				lock;
	struct list_head		queue;
	struct usb_gadget		*gadget;
	struct tile_udc			*dev;
	const struct usb_endpoint_descriptor *desc;
	struct usb_ep			ep;
	u8				num;

	unsigned short			fifo_size;
	u8				bEndpointAddress;
	u8				bmAttributes;

	unsigned			halted;
};


/* Note : ep0 has a fifo of 1024 bytes */
#define EP0_FIFO_SIZE		64
#define EP_FIFO_SIZE		512
#define DEFAULT_POWER_STATE	0x00

#define TILE_EP_FIFO_SIZE	1024

static const char ep0name[] = "ep0";

static const char *const ep_name[] = {
	ep0name,                                /* everyone has ep0 */
	/* tile four bidirectional bulk endpoints */
	"ep1-bulk", "ep2-bulk", "ep3-bulk", "ep4-bulk",
	"ep5-bulk"
};

#define TILE_ENDPOINTS       ARRAY_SIZE(ep_name)

struct tile_request {
	struct list_head		queue;		/* ep's requests */
	struct usb_request		req;
};

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_END_XFER,
	EP0_STALL,
};

static const char * const ep0states[] = {
	"EP0_IDLE",
	"EP0_IN_DATA_PHASE",
	"EP0_OUT_DATA_PHASE",
	"EP0_END_XFER",
	"EP0_STALL",
};

struct tile_udc {
	spinlock_t			lock;

	struct tile_ep			ep[TILE_ENDPOINTS];
	int				address;
	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;
	struct tile_request		fifo_req;
	u8				fifo_buf[EP_FIFO_SIZE];
	u16				devstatus;

	u32				port_status;
	int				ep0state;

	unsigned			got_irq:1;

	unsigned			req_std:1;
	unsigned			req_config:1;
	unsigned			req_pending:1;
	struct dentry			*regs_info;
	gxio_usb_dev_context_t		*usb_ctx;
};

#endif
