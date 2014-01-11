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
 *
 * Some portions copied from the S3C24xx version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/prefetch.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/usb.h>
#include <linux/usb/gadget.h>

#include <asm/byteorder.h>
#include <asm/irq.h>

#include <arch/usb_device.h>
#include <arch/usb_device_def.h>

#include <gxio/iorpc_usb_dev.h>
#include <gxio/usb_dev.h>

#include "tile_udc.h"

#define DRIVER_DESC	"TILE-Gx USB Device Controller Gadget"
#define DRIVER_VERSION	"16 May 2013"

static const char		gadget_name[] = "tile_udc";
static const char		driver_desc[] = DRIVER_DESC;

static struct tile_udc		*the_controller;
static struct dentry		*tile_udc_debugfs_root;
static struct platform_device	*tile_udc_platform_device;
static struct tile_udc		memory;

static char dummy_buf[512];

/*************************** DEBUG FUNCTION ***************************/
#define DEBUG_NORMAL	1
#define DEBUG_VERBOSE	2

#ifdef CONFIG_USB_TILEGX_DEBUG
#define USB_TILEGX_DEBUG_LEVEL 0

static int dprintk(int level, const char *fmt, ...)
{
	static char printk_buf[1024];
	va_list args;
	int len;

	if (level > USB_TILEGX_DEBUG_LEVEL)
		return 0;

	len = scnprintf(printk_buf, sizeof(printk_buf), "USB: ");

	va_start(args, fmt);
	len = vscnprintf(printk_buf+len,
			 sizeof(printk_buf)-len, fmt, args);
	va_end(args);

	return printk(KERN_ERR "%s", printk_buf);
}
#else
static int dprintk(int level, const char *fmt, ...)
{
	return 0;
}
#endif

static int tile_udc_debugfs_seq_show(struct seq_file *m, void *p)
{
	struct tile_udc *udc = the_controller;
	u32 d_cfg, d_ctrl, d_sts, d_intr, d_intr_msk;
	u32 ep_intr, ep_intr_msk;
	u32 ep0_ne, ep1_ne, ep2_ne, ep3_ne, ep4_ne, ep5_ne;
	u32 ep0_in_ctrl, ep0_in_sts, ep0_in_bufsize, ep0_in_mpkt;
	u32 ep0_out_ctrl, ep0_out_sts, ep0_out_bufsize;
	u32 ep1_in_ctrl, ep1_in_sts, ep1_in_bufsize, ep1_in_mpkt;
	u32 ep2_out_ctrl, ep2_out_sts, ep2_out_bufsize;
	u32 ep3_in_ctrl, ep3_in_sts, ep3_in_bufsize, ep3_in_mpkt;
	u32 ep4_out_ctrl, ep4_out_sts, ep4_out_bufsize;
	u32 ep5_in_ctrl, ep5_in_sts, ep5_in_bufsize, ep5_in_mpkt;

	d_cfg	= gxio_usb_dev_get_d_cfg(udc->usb_ctx);
	d_ctrl	= gxio_usb_dev_get_d_ctrl(udc->usb_ctx);
	d_sts	= gxio_usb_dev_get_d_sts(udc->usb_ctx);
	d_intr	= gxio_usb_dev_get_d_intr(udc->usb_ctx);
	d_intr_msk	= gxio_usb_dev_get_d_intr_msk(udc->usb_ctx);

	ep_intr  = gxio_usb_dev_get_ep_intr(udc->usb_ctx);
	ep_intr_msk = gxio_usb_dev_get_ep_intr_msk(udc->usb_ctx);

	ep0_ne	= gxio_usb_dev_get_udc_ep(udc->usb_ctx, 0);
	ep1_ne	= gxio_usb_dev_get_udc_ep(udc->usb_ctx, 1);
	ep2_ne	= gxio_usb_dev_get_udc_ep(udc->usb_ctx, 2);
	ep3_ne	= gxio_usb_dev_get_udc_ep(udc->usb_ctx, 3);
	ep4_ne	= gxio_usb_dev_get_udc_ep(udc->usb_ctx, 4);
	ep5_ne	= gxio_usb_dev_get_udc_ep(udc->usb_ctx, 5);

	ep0_in_ctrl	= gxio_usb_dev_get_in_ctrl(udc->usb_ctx, 0);
	ep0_in_sts	= gxio_usb_dev_get_in_sts(udc->usb_ctx, 0);
	ep0_in_bufsize	= gxio_usb_dev_get_in_bufsize(udc->usb_ctx, 0);
	ep0_in_mpkt	= gxio_usb_dev_get_in_mpkt_sz(udc->usb_ctx, 0);

	ep0_out_ctrl	= gxio_usb_dev_get_out_ctrl(udc->usb_ctx, 0);
	ep0_out_sts	= gxio_usb_dev_get_out_sts(udc->usb_ctx, 0);
	ep0_out_bufsize	= gxio_usb_dev_get_out_bufsize(udc->usb_ctx, 0);

	ep1_in_ctrl	= gxio_usb_dev_get_in_ctrl(udc->usb_ctx, 1);
	ep1_in_sts	= gxio_usb_dev_get_in_sts(udc->usb_ctx, 1);
	ep1_in_bufsize	= gxio_usb_dev_get_in_bufsize(udc->usb_ctx, 1);
	ep1_in_mpkt	= gxio_usb_dev_get_in_mpkt_sz(udc->usb_ctx, 1);

	ep2_out_ctrl	= gxio_usb_dev_get_out_ctrl(udc->usb_ctx, 2);
	ep2_out_sts	= gxio_usb_dev_get_out_sts(udc->usb_ctx, 2);
	ep2_out_bufsize	= gxio_usb_dev_get_out_bufsize(udc->usb_ctx, 2);

	ep3_in_ctrl	= gxio_usb_dev_get_in_ctrl(udc->usb_ctx, 3);
	ep3_in_sts	= gxio_usb_dev_get_in_sts(udc->usb_ctx, 3);
	ep3_in_bufsize	= gxio_usb_dev_get_in_bufsize(udc->usb_ctx, 3);
	ep3_in_mpkt	= gxio_usb_dev_get_in_mpkt_sz(udc->usb_ctx, 3);

	ep4_out_ctrl	= gxio_usb_dev_get_out_ctrl(udc->usb_ctx, 4);
	ep4_out_sts	= gxio_usb_dev_get_out_sts(udc->usb_ctx, 4);
	ep4_out_bufsize	= gxio_usb_dev_get_out_bufsize(udc->usb_ctx, 4);

	ep5_in_ctrl	= gxio_usb_dev_get_in_ctrl(udc->usb_ctx, 5);
	ep5_in_sts	= gxio_usb_dev_get_in_sts(udc->usb_ctx, 5);
	ep5_in_bufsize	= gxio_usb_dev_get_in_bufsize(udc->usb_ctx, 5);
	ep5_in_mpkt	= gxio_usb_dev_get_in_mpkt_sz(udc->usb_ctx, 5);

	seq_printf(m, "D_CFG  : 0x%04X\n"
		 "D_CTRL        : 0x%04X\n"
		 "D_STS     : 0x%04X\n"
		 "D_INTR    : 0x%04X\n"
		 "D_INTR_MSK  : 0x%04X\n"
		 "EP_INTR : 0x%04X\n"
		 "EP_INTR_MSK        : 0x%04X\n"
		 "EP0_NE     : 0x%04X\n"
		 "EP1_NE     : 0x%04X\n"
		 "EP2_NE     : 0x%04X\n"
		 "EP3_NE     : 0x%04X\n"
		 "EP4_NE     : 0x%04X\n"
		 "EP5_NE     : 0x%04X\n"
		 "EP0_IN_CTRL     : 0x%04X\n"
		 "EP0_IN_STS    : 0x%04X\n"
		 "EP0_IN_BUFSIZE    : 0x%04X\n"
		 "EP0_IN_MPKT	: 0x%04X\n"
		 "EP0_OUT_CTRL	 : 0x%04X\n"
		 "EP0_OUT_STS    : 0x%04X\n"
		 "EP0_OUT_BUFSIZE    : 0x%04X\n"
		 "EP1_IN_CTRL	 : 0x%04X\n"
		 "EP1_IN_STS    : 0x%04X\n"
		 "EP1_IN_BUFSIZE    : 0x%04X\n"
		 "EP1_IN_MPKT   : 0x%04X\n"
		 "EP2_OUT_CTRL	: 0x%04X\n"
		 "EP2_OUT_STS	: 0x%04X\n"
		 "EP2_OUT_BUFSIZE	: 0x%04X\n"
		 "EP3_IN_CTRL	: 0x%04X\n"
		 "EP3_IN_STS    : 0x%04X\n"
		 "EP3_IN_BUFSIZE    : 0x%04X\n"
		 "EP3_IN_MPKT   : 0x%04X\n"
		 "EP4_OUT_CTRL  : 0x%04X\n"
		 "EP4_OUT_STS   : 0x%04X\n"
		 "EP4_OUT_BUFSIZE   : 0x%04X\n"
		 "EP5_IN_CTRL	: 0x%04X\n"
		 "EP5_IN_STS    : 0x%04X\n"
		 "EP5_IN_BUFSIZE    : 0x%04X\n"
		 "EP5_IN_MPKT   : 0x%04X\n",
			d_cfg, d_ctrl, d_sts, d_intr, d_intr_msk,
			ep_intr, ep_intr_msk,
			ep0_ne, ep1_ne, ep2_ne, ep3_ne, ep4_ne, ep5_ne,
			ep0_in_ctrl, ep0_in_sts, ep0_in_bufsize, ep0_in_mpkt,
			ep0_out_ctrl, ep0_out_sts, ep0_out_bufsize,
			ep1_in_ctrl, ep1_in_sts, ep1_in_bufsize, ep1_in_mpkt,
			ep2_out_ctrl, ep2_out_sts, ep2_out_bufsize,
			ep3_in_ctrl, ep3_in_sts, ep3_in_bufsize, ep3_in_mpkt,
			ep4_out_ctrl, ep4_out_sts, ep4_out_bufsize,
			ep5_in_ctrl, ep5_in_sts, ep5_in_bufsize, ep5_in_mpkt
		);

	return 0;
}

static int tile_udc_debugfs_fops_open(struct inode *inode,
					 struct file *file)
{
	return single_open(file, tile_udc_debugfs_seq_show, NULL);
}

static const struct file_operations tile_udc_debugfs_fops = {
	.open		= tile_udc_debugfs_fops_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static void tile_udc_done(struct tile_ep *ep,
		struct tile_request *req, int status)
{
	unsigned halted = ep->halted;

	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	ep->halted = 1;

	/*
	 * Some complete hook function will call tile_udc_queue()
	 * to send more data, so we release the lock first.
	 */
	spin_unlock(&ep->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->lock);

	ep->halted = halted;
}

static void tile_udc_nuke(struct tile_udc *udc,
			  struct tile_ep *ep, int status)
{
	/* Sanity check */
	if (&ep->queue == NULL)
		return;

	while (!list_empty(&ep->queue)) {
		struct tile_request *req;
		req = list_entry(ep->queue.next, struct tile_request,
				queue);
		tile_udc_done(ep, req, status);
	}
}

static inline int tile_udc_write_packet(gxio_usb_dev_context_t *usb_ctx,
					struct tile_request *req,
					u32 ep,
					unsigned max)
{
	unsigned len = min(req->req.length - req->req.actual, max);
	u8 *buf = req->req.buf + req->req.actual;

	prefetch(buf);

	dprintk(DEBUG_NORMAL, "%s %d %d %d %d\n", __func__,
		req->req.actual, req->req.length, len, req->req.actual + len);

	req->req.actual += len;

	/* Do not send zero-length packet to ep0. */
	if (ep == 0 && len == 0)
		return 0;

	gxio_usb_dev_write_tx_fifo(usb_ctx, ep, buf, len);
	return len;
}

/*
 *	tile_udc_write_fifo
 *
 * return:  0 = still running, 1 = completed, negative = errno
 */
static int tile_udc_write_fifo(gxio_usb_dev_context_t *usb_ctx,
			       struct tile_ep *ep,
			       struct tile_request *req)
{
	unsigned	count;
	int		is_last;
	u32		idx;

	idx = ep->bEndpointAddress & 0x7F;
	count = tile_udc_write_packet(usb_ctx, req, idx, ep->ep.maxpacket);

	/* last packet is often short (sometimes a zlp) */
	if (count != ep->ep.maxpacket)
		is_last = 1;
	else if (req->req.length != req->req.actual || req->req.zero)
		is_last = 0;
	else
		is_last = 2;

	/* Only ep0 debug messages are interesting */
	if (idx == 0)
		dprintk(DEBUG_NORMAL,
			"Written ep%d %d.%d of %d b [last %d,z %d]\n",
			idx, count, req->req.actual, req->req.length,
			is_last, req->req.zero);

	if (is_last) {
		tile_udc_done(ep, req, 0);
		is_last = 1;
	}

	return is_last;
}

static inline int tile_udc_read_packet(gxio_usb_dev_context_t *usb_ctx,
				       u32 ep, u8 *buf,
				       struct tile_request *req,
				       unsigned avail)
{
	unsigned len;

	len = min(req->req.length - req->req.actual, avail);
	req->req.actual += len;

	gxio_usb_dev_read_rx_fifo(usb_ctx, ep, buf, len);
	return len;
}

/*
 * return:  0 = still running, 1 = queue empty, negative = errno
 */
static int tile_udc_read_fifo(gxio_usb_dev_context_t *usb_ctx,
			      struct tile_ep *ep,
			      struct tile_request *req)
{
	u8		*buf;
	u32		ep_sts;
	unsigned	bufferspace;
	int		is_last = 1;
	unsigned	avail;
	int		fifo_count = 0;
	u32		idx;

	idx = ep->bEndpointAddress & 0x7F;

	if (!req->req.length)
		return 1;

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;
	if (!bufferspace) {
		dprintk(DEBUG_NORMAL, "%s: buffer full!\n", __func__);
		return -1;
	}

	ep_sts = gxio_usb_dev_get_out_sts(usb_ctx, idx);
	fifo_count =
		(ep_sts >> USB_DEVICE_EP0_IN_STS_UDC_REG__RX_PKT_SIZE_SHIFT) &
		USB_DEVICE_EP0_IN_STS_UDC_REG__RX_PKT_SIZE_RMASK;
	dprintk(DEBUG_NORMAL, "%s: ep_sts=%x, fifo count : %d\n",
		__func__, ep_sts, fifo_count);

	avail = min(fifo_count, (int)ep->ep.maxpacket);
	fifo_count = tile_udc_read_packet(usb_ctx, idx, buf, req, avail);

	/* checking this with ep0 is not accurate as we already
	 * read a control request
	 **/
	if (idx != 0 && fifo_count < ep->ep.maxpacket) {
		is_last = 1;
		/* overflowed this request?  flush extra data */
		if (fifo_count != avail)
			req->req.status = -EOVERFLOW;
	} else {
		is_last = (req->req.length <= req->req.actual) ? 1 : 0;
	}

	/* Only ep0 debug messages are interesting */
	if (idx == 0)
		dprintk(DEBUG_VERBOSE, "%s fifo count : %d [last %d]\n",
			__func__, fifo_count, is_last);

	if (is_last)
		tile_udc_done(ep, req, 0);

	return is_last;
}

static int tile_udc_set_halt(struct usb_ep *_ep, int value);

static void tile_udc_handle_ep0_idle(struct tile_udc *dev,
					struct tile_ep *ep,
					struct usb_ctrlrequest *crq,
					u32 ep0csr)
{
	int ret;
	int out_rcv, rx_pkt_size;

	out_rcv = (ep0csr >> USB_DEVICE_EP0_IN_STS_UDC_REG__OUT_RCV_SHIFT) &
		USB_DEVICE_EP0_IN_STS_UDC_REG__OUT_RCV_RMASK;

	/*
	 * Discard received data packets if any, since we only care about
	 * SETUP packets in IDLE state.
	 */
	if (out_rcv == 1) {
		rx_pkt_size = (ep0csr >>
			USB_DEVICE_EP0_IN_STS_UDC_REG__RX_PKT_SIZE_SHIFT) &
			USB_DEVICE_EP0_IN_STS_UDC_REG__RX_PKT_SIZE_RMASK;
		dprintk(DEBUG_NORMAL, "read %d bytes\n", rx_pkt_size);
		gxio_usb_dev_read_rx_fifo(dev->usb_ctx, 0,
					  dummy_buf, rx_pkt_size);
		return;
	}

	/* Start control request? */
	gxio_usb_dev_read_rx_fifo(dev->usb_ctx, 0, crq, sizeof(*crq));
	dprintk(DEBUG_NORMAL, "bRequest = %d bRequestType %d wLength = %d\n",
		crq->bRequest, crq->bRequestType, crq->wLength);
	dprintk(DEBUG_NORMAL, "wValue = %x wIndex =%x \n",
		crq->wValue, crq->wIndex);

	/* Cope with automagic for some standard requests. */
	dev->req_std = (crq->bRequestType & USB_TYPE_MASK)
			== USB_TYPE_STANDARD;

	if (crq->bRequestType & USB_DIR_IN)
		dev->ep0state = EP0_IN_DATA_PHASE;
	else
		dev->ep0state = EP0_OUT_DATA_PHASE;

	if (!dev->driver)
		return;

	/* Deliver the request to the gadget driver. */
	ret = dev->driver->setup(&dev->gadget, crq);
	if (ret < 0) {
		if (dev->req_config) {
			dprintk(DEBUG_NORMAL, "config change %02x fail %d?\n",
				crq->bRequest, ret);
			return;
		}

		if (ret == -EOPNOTSUPP)
			dprintk(DEBUG_NORMAL, "Operation not supported\n");
		else
			dprintk(DEBUG_NORMAL,
				"dev->driver->setup failed. (%d)\n", ret);

		dev->ep0state = EP0_IDLE;
	}

	dprintk(DEBUG_VERBOSE, "ep0state %s\n", ep0states[dev->ep0state]);
}

static void tile_udc_handle_in_ep0(struct tile_udc *dev)
{
	u32			ep0_in_sts;
	struct tile_ep	*ep = &dev->ep[0];
	struct tile_request	*req;
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct tile_request, queue);

	ep0_in_sts = gxio_usb_dev_get_in_sts(dev->usb_ctx, 0);

	dprintk(DEBUG_NORMAL, "ep0 in sts 0x%x ep0state %s\n",
		ep0_in_sts, ep0states[dev->ep0state]);

	if (ep0_in_sts & USB_DEVICE_EP0_IN_STS_UDC_REG__TXEMPTY_MASK) {
		dprintk(DEBUG_NORMAL, "TX completed\n");
		gxio_usb_dev_set_in_sts(dev->usb_ctx, 0,
			USB_DEVICE_EP0_IN_STS_UDC_REG__TXEMPTY_MASK);
	}

	if (ep0_in_sts & USB_DEVICE_EP0_IN_STS_UDC_REG__IN_RCV_MASK) {
		dprintk(DEBUG_NORMAL, "IN token received\n");
		gxio_usb_dev_set_in_sts(dev->usb_ctx, 0,
			USB_DEVICE_EP0_IN_STS_UDC_REG__IN_RCV_MASK);
	}

	switch (dev->ep0state) {
	case EP0_IDLE:
		dprintk(DEBUG_NORMAL, "EP0_IDLE ... what now?\n");
		break;

	case EP0_IN_DATA_PHASE:			/* GET_DESCRIPTOR etc */
		dprintk(DEBUG_NORMAL, "EP0_IN_DATA_PHASE ...\n");
		if ((ep0_in_sts &
		      (USB_DEVICE_EP0_IN_STS_UDC_REG__TXEMPTY_MASK |
		       USB_DEVICE_EP0_IN_STS_UDC_REG__IN_RCV_MASK))
		      && req) {
			if (tile_udc_write_fifo(dev->usb_ctx, ep, req))
				dev->ep0state = EP0_IDLE;
		}
		break;

	case EP0_OUT_DATA_PHASE:		/* SET_DESCRIPTOR etc */
		dprintk(DEBUG_NORMAL, "EP0_OUT_DATA_PHASE ... what now?\n");
		break;
	}

	spin_unlock_irqrestore(&ep->lock, flags);
}

static void tile_udc_handle_out_ep0(struct tile_udc *dev)
{
	u32			ep0_out_sts;
	struct tile_ep	*ep = &dev->ep[0];
	struct tile_request	*req;
	struct usb_ctrlrequest	crq;
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct tile_request, queue);

	ep0_out_sts = gxio_usb_dev_get_out_sts(dev->usb_ctx, 0);

	dprintk(DEBUG_NORMAL, "ep0 out sts %x ep0state %s\n",
		ep0_out_sts, ep0states[dev->ep0state]);

	switch (dev->ep0state) {
	case EP0_IDLE:
		dprintk(DEBUG_NORMAL, "EP0_IDLE ...\n");
		spin_unlock_irqrestore(&ep->lock, flags);
		tile_udc_handle_ep0_idle(dev, ep, &crq, ep0_out_sts);
		spin_lock_irqsave(&ep->lock, flags);
		break;

	case EP0_IN_DATA_PHASE:			/* GET_DESCRIPTOR etc */
		dprintk(DEBUG_NORMAL, "EP0_IN_DATA_PHASE ... what now?\n");
		break;

	case EP0_OUT_DATA_PHASE:		/* SET_DESCRIPTOR etc */
		dprintk(DEBUG_NORMAL, "EP0_OUT_DATA_PHASE ...\n");
		if ((ep0_out_sts &
		    USB_DEVICE_EP0_IN_STS_UDC_REG__OUT_RCV_MASK) && req) {
			if (tile_udc_read_fifo(dev->usb_ctx, ep, req))
				dev->ep0state = EP0_IDLE;
		}
		break;
	}
	spin_unlock_irqrestore(&ep->lock, flags);
}

/*
 *	handle_in_ep - Manage IN endpoints
 */
static void tile_udc_handle_in_ep(struct tile_udc *dev,
				  int ep_num)
{
	struct tile_request	*req;
	struct tile_ep		*ep = &dev->ep[ep_num];
	u32			ep_sts;
	u32			int_en_reg;
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);

	if (likely(!list_empty(&ep->queue)))
		req = list_entry(ep->queue.next,
				struct tile_request, queue);
	else
		req = NULL;

	if (req == NULL) {
		/* Disable irqs. */
		int_en_reg = gxio_usb_dev_get_ep_intr_msk(dev->usb_ctx);
		gxio_usb_dev_set_ep_intr_msk(dev->usb_ctx,
				int_en_reg | (1 << ep_num));
		dprintk(DEBUG_VERBOSE, "ep%d disable irq\n", ep_num);
		spin_unlock_irqrestore(&ep->lock, flags);
		return;
	}

	ep_sts = gxio_usb_dev_get_in_sts(dev->usb_ctx, ep_num);

	dprintk(DEBUG_NORMAL, "ep%d in sts %x, req=%p\n",
		ep_num, ep_sts, req);

	/*
	 * FIXME: For massive data transfer, we need to wait at least 200
	 * us for the USB device controller processing the data, otherwise
	 * the USB device controller stops generating TXEMPTY interrupt.
	 */
	udelay(200);

	if (ep_sts & USB_DEVICE_EP0_IN_STS_UDC_REG__IN_RCV_MASK) {
		dprintk(DEBUG_VERBOSE, "IN Token\n");
		if (req) {
			tile_udc_write_fifo(dev->usb_ctx, ep, req);
			req = NULL;
		}

		gxio_usb_dev_set_in_sts(dev->usb_ctx, ep_num,
			USB_DEVICE_EP0_IN_STS_UDC_REG__IN_RCV_MASK);
	}

	if (ep_sts & USB_DEVICE_EP0_IN_STS_UDC_REG__TXEMPTY_MASK) {
		dprintk(DEBUG_VERBOSE, "IN TXEMPTY\n");
		if (req)
			tile_udc_write_fifo(dev->usb_ctx, ep, req);

		gxio_usb_dev_set_in_sts(dev->usb_ctx, ep_num,
			USB_DEVICE_EP0_IN_STS_UDC_REG__TXEMPTY_MASK);
	}

	spin_unlock_irqrestore(&ep->lock, flags);
}

/*
 *	handle_out_ep - Manage OUT endpoints
 */
static void tile_udc_handle_out_ep(struct tile_udc *dev,
				   int ep_num)
{
	struct tile_request	*req;
	struct tile_ep		*ep = &dev->ep[ep_num];
	u32			ep_sts;
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);

	if (likely(!list_empty(&ep->queue)))
		req = list_entry(ep->queue.next,
				struct tile_request, queue);
	else
		req = NULL;

	ep_sts = gxio_usb_dev_get_out_sts(dev->usb_ctx, ep_num);

	dprintk(DEBUG_VERBOSE, "ep%d OUT sts %x, req=%p\n",
		ep_num, ep_sts, req);

	if ((ep_sts &
	    USB_DEVICE_EP0_IN_STS_UDC_REG__OUT_RCV_MASK) && req) {
		tile_udc_read_fifo(dev->usb_ctx, ep, req);

		gxio_usb_dev_set_out_sts(dev->usb_ctx, ep_num, 0x10);
	}

	spin_unlock_irqrestore(&ep->lock, flags);
}

/*
 *	tile_udc_irq - interrupt handler
 */
static irqreturn_t tile_udc_irq(int dummy, void *dev)
{
	struct tile_udc *udc = (struct tile_udc *)dev;
	unsigned int dev_intr, dev_stat, dev_ctrl;
	unsigned int ep_intr, ep_bit;
	unsigned long flags;
	struct usb_ctrlrequest crq;
	int ret;

	spin_lock_irqsave(&udc->lock, flags);

	/* Read interrupt status registers. */
	dev_intr = gxio_usb_dev_get_d_intr(udc->usb_ctx);
	ep_intr = gxio_usb_dev_get_ep_intr(udc->usb_ctx);

	dprintk(DEBUG_NORMAL, "dev_intr=0x%x, ep_intr=0x%x\n",
		dev_intr, ep_intr);

	/*
	 * Now, handle interrupts. There's two types :
	 * - Reset, Enumeration complete, IDLE ... -> dev_intr
	 * - EP -> ep_intr
	 */

	/* RESET */
	if (dev_intr & USB_DEVICE_D_INTR_UDC_REG__UR_MASK) {
		dprintk(DEBUG_NORMAL, "USB reset\n");
		udc->gadget.speed = USB_SPEED_UNKNOWN;
		udc->ep0state = EP0_IDLE;
	}

	/* Enumeration Complete */
	if (dev_intr & USB_DEVICE_D_INTR_UDC_REG__ENUM_COMP_MASK) {
		dprintk(DEBUG_NORMAL, "Speed enumeration is complete.\n");
		dev_stat = gxio_usb_dev_get_d_sts(udc->usb_ctx);
		udc->gadget.speed =
			(dev_stat & USB_DEVICE_D_STS_UDC_REG__ENUM_SPD_MASK)
			? USB_SPEED_FULL : USB_SPEED_HIGH;
		udc->ep0state = EP0_IDLE;
	}

	/* IDLE */
	if (dev_intr & USB_DEVICE_D_INTR_UDC_REG__ES_MASK) {
		dprintk(DEBUG_NORMAL, "Idle state is detected.\n");
		udc->ep0state = EP0_IDLE;
	}

	/* Set Configuration */
	if (dev_intr & USB_DEVICE_D_INTR_UDC_REG__SC_MASK) {
		dev_stat = gxio_usb_dev_get_d_sts(udc->usb_ctx);
		dprintk(DEBUG_NORMAL,
			"USB Set Configuration dev_sts 0x%8x\n", dev_stat);

		crq.bRequest = USB_REQ_SET_CONFIGURATION;
		crq.bRequestType = USB_RECIP_DEVICE;
		crq.wValue = dev_stat & USB_DEVICE_D_STS_UDC_REG__CFG_MASK;
		crq.wIndex = 0;
		crq.wLength = 0;

		if (udc->driver) {
			/* Deliver the request to the gadget driver */
			ret = udc->driver->setup(&udc->gadget, &crq);
			if (ret < 0) {
				if (udc->req_config) {
					dprintk(DEBUG_NORMAL,
						"config change %02x fail %d?\n",
						crq.bRequest, ret);
				}

				if (ret == -EOPNOTSUPP)
					dprintk(DEBUG_NORMAL,
						"Operation not supported\n");
				else
					dprintk(DEBUG_NORMAL,
						"setup failed. (%d)\n", ret);
			}
		}

		/*
		 * Acknowledge the Set Configuration command by setting
		 * CSR_DONE in the device control regisger.
		 */
		dev_ctrl = gxio_usb_dev_get_d_ctrl(udc->usb_ctx);
		gxio_usb_dev_set_d_ctrl(udc->usb_ctx,
			dev_ctrl | USB_DEVICE_D_CTRL_UDC_REG__CSR_DONE_MASK);
		udc->ep0state = EP0_IDLE;
	}



	/* Clear the device interrupt bit by setting it to 1. */
	gxio_usb_dev_set_d_intr(udc->usb_ctx, dev_intr);

	/* Clear the endpoint interrupt bit by setting it to 1. */
	gxio_usb_dev_set_ep_intr(udc->usb_ctx, ep_intr);

	/* Endpoint data transfers. */
	while (ep_intr) {
		ep_bit = (8 * sizeof(int)) - 1 - __builtin_clz(ep_intr);
		ep_intr -= 1 << ep_bit;

		if (ep_bit == 0) {
			dprintk(DEBUG_VERBOSE, "USB ep0 in\n");
			tile_udc_handle_in_ep0(udc);
		} else if (ep_bit == 16) {
			dprintk(DEBUG_VERBOSE, "USB ep0 out\n");
			tile_udc_handle_out_ep0(udc);
		} else if (ep_bit < 16) {
			dprintk(DEBUG_VERBOSE, "USB ep%d in\n", ep_bit);
			tile_udc_handle_in_ep(udc, ep_bit);
		} else {
			dprintk(DEBUG_VERBOSE, "USB ep%d out\n", ep_bit - 16);
			tile_udc_handle_out_ep(udc, ep_bit - 16);
		}
	}

	/* Set Interface */
	if (dev_intr & USB_DEVICE_D_INTR_UDC_REG__SI_MASK) {
		dprintk(DEBUG_NORMAL, "USB Set Interface\n");
		dev_stat = gxio_usb_dev_get_d_sts(udc->usb_ctx);
		dprintk(DEBUG_NORMAL,
				"USB Set Interface dev_sts 0x%8x\n", dev_stat);

		crq.bRequest = USB_REQ_SET_INTERFACE;
		crq.bRequestType = USB_RECIP_INTERFACE;
		crq.wValue = (dev_stat & USB_DEVICE_D_STS_UDC_REG__ALT_MASK)
			>> USB_DEVICE_D_STS_UDC_REG__ALT_SHIFT;
		crq.wIndex = (dev_stat & USB_DEVICE_D_STS_UDC_REG__INTF_MASK)
			>> USB_DEVICE_D_STS_UDC_REG__INTF_SHIFT;
		crq.wLength = 0;

		if (udc->driver) {
			/* Deliver the request to the gadget driver */
			ret = udc->driver->setup(&udc->gadget, &crq);
			if (ret < 0) {
				if (udc->req_config) {
					dprintk(DEBUG_NORMAL,
						"config change %02x fail %d?\n",
						crq.bRequest, ret);
				}

				if (ret == -EOPNOTSUPP)
					dprintk(DEBUG_NORMAL,
						"Operation not supported\n");
				else
					dprintk(DEBUG_NORMAL,
						"setup failed. (%d)\n", ret);
			}
		}


		/*
		 * Acknowledge the Set Interface command by setting
		 * CSR_DONE in the device control regisger.
		 */
		dev_ctrl = gxio_usb_dev_get_d_ctrl(udc->usb_ctx);
		gxio_usb_dev_set_d_ctrl(udc->usb_ctx,
			dev_ctrl | USB_DEVICE_D_CTRL_UDC_REG__CSR_DONE_MASK);
		udc->ep0state = EP0_IDLE;
	}

	gxio_usb_dev_reset_interrupt(udc->usb_ctx);

	dprintk(DEBUG_VERBOSE, "irq: tile_udc_irq done\n\n");
	spin_unlock_irqrestore(&udc->lock, flags);

	return IRQ_HANDLED;
}

/*------------------------- tile_ep_ops ----------------------------------*/
static inline struct tile_ep *to_tile_ep(struct usb_ep *ep)
{
	return container_of(ep, struct tile_ep, ep);
}

static inline struct tile_udc *to_tile_udc(struct usb_gadget *gadget)
{
	return container_of(gadget, struct tile_udc, gadget);
}

static inline struct tile_request *to_tile_req(struct usb_request *req)
{
	return container_of(req, struct tile_request, req);
}

static int tile_udc_ep_enable(struct usb_ep *_ep,
			      const struct usb_endpoint_descriptor *desc)
{
	struct tile_udc	*dev;
	struct tile_ep	*ep;
	u32			max, tmp;
	unsigned long		flags;
	u32			int_en_reg;

	ep = to_tile_ep(_ep);

	dprintk(DEBUG_NORMAL, "%s(): ep_enable %s address 0x%x\n",
		__func__, _ep->name, desc->bEndpointAddress);

	if (!_ep || !desc || ep->desc
			|| _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	max = le16_to_cpu(desc->wMaxPacketSize) & 0x1FFF;

	local_irq_save(flags);
	spin_lock_init(&ep->lock);
	_ep->maxpacket = max & 0x7FF;
	ep->desc = desc;
	ep->halted = 0;
	ep->bEndpointAddress = desc->bEndpointAddress;

	/* Enable irqs. */
	int_en_reg = gxio_usb_dev_get_ep_intr_msk(dev->usb_ctx);

	if (desc->bEndpointAddress & USB_DIR_IN)
		gxio_usb_dev_set_ep_intr_msk(dev->usb_ctx,
			int_en_reg & (~(1 << ep->num)));
	else
		gxio_usb_dev_set_ep_intr_msk(dev->usb_ctx,
			int_en_reg & (~(1 << (ep->num + 16))));

	/* Print some debug message. */
	tmp = desc->bEndpointAddress;
	dprintk(DEBUG_NORMAL, "enable %s(%d) ep%x%s-blk max %02x\n",
		 _ep->name, ep->num, tmp,
		 desc->bEndpointAddress & USB_DIR_IN ? "in" : "out", max);

	local_irq_restore(flags);
	tile_udc_set_halt(_ep, 0);

	return 0;
}

static int tile_udc_ep_disable(struct usb_ep *_ep)
{
	struct tile_ep *ep = to_tile_ep(_ep);
	struct tile_udc	*dev = ep->dev;
	unsigned long flags;
	u32 int_en_reg;

	if (!_ep || !ep->desc) {
		dprintk(DEBUG_NORMAL, "%s not enabled\n",
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->lock, flags);

	dprintk(DEBUG_NORMAL, "ep_disable: %s\n", _ep->name);

	ep->desc = NULL;
	ep->halted = 1;

	tile_udc_nuke(dev, ep, -ESHUTDOWN);

	/* Disable irqs. */
	int_en_reg = gxio_usb_dev_get_ep_intr_msk(dev->usb_ctx);

	if (ep->bEndpointAddress & USB_DIR_IN)
		gxio_usb_dev_set_ep_intr_msk(dev->usb_ctx,
			int_en_reg | (1 << ep->num));
	else
		gxio_usb_dev_set_ep_intr_msk(dev->usb_ctx,
			int_en_reg | (1 << (ep->num + 16)));

	spin_unlock_irqrestore(&ep->lock, flags);
	dprintk(DEBUG_NORMAL, "%s disabled\n", _ep->name);

	return 0;
}

static struct usb_request *
tile_udc_alloc_request(struct usb_ep *_ep, gfp_t mem_flags)
{
	struct tile_request *req;

	dprintk(DEBUG_VERBOSE, "%s(%p,%d)\n", __func__, _ep, mem_flags);

	if (!_ep)
		return NULL;

	req = kzalloc(sizeof(struct tile_request), mem_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void
tile_udc_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct tile_ep	*ep = to_tile_ep(_ep);
	struct tile_request	*req = to_tile_req(_req);

	dprintk(DEBUG_VERBOSE, "%s(%p,%p)\n", __func__, _ep, _req);

	if (!ep || !_req || (!ep->desc && ep->num != 0))
		return;

	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/*
 * SETUP requests set a NAK bit on the endpoint, which we need to
 * clear.
 */
static void tile_udc_clear_nak(gxio_usb_dev_context_t *usb_ctx,
				int ep_num)
{
	int out_ctrl, in_ctrl;

	out_ctrl = gxio_usb_dev_get_out_ctrl(usb_ctx, ep_num);
	if (out_ctrl & USB_DEVICE_EP0_IN_CTRL_UDC_REG__NAK_MASK) {
		gxio_usb_dev_set_out_ctrl(usb_ctx, ep_num,
		out_ctrl | USB_DEVICE_EP0_IN_CTRL_UDC_REG__CNAK_MASK);
	}

	in_ctrl = gxio_usb_dev_get_in_ctrl(usb_ctx, ep_num);
	if (in_ctrl & USB_DEVICE_EP0_IN_CTRL_UDC_REG__NAK_MASK) {
		gxio_usb_dev_set_in_ctrl(usb_ctx, ep_num,
		in_ctrl | USB_DEVICE_EP0_IN_CTRL_UDC_REG__CNAK_MASK);
	}
}

static int tile_udc_queue(struct usb_ep *_ep, struct usb_request *_req,
			  gfp_t gfp_flags)
{
	struct tile_request	*req = to_tile_req(_req);
	struct tile_ep		*ep = to_tile_ep(_ep);
	struct tile_udc		*dev = ep->dev;
	u32			ep_sts = 0;
	u32			int_en_reg;
	int			fifo_count = 0;
	unsigned long		flags;
	int			ep_num = ep->bEndpointAddress & 0x7F;

	if (unlikely(!_ep || (!ep->desc && ep->num != 0))) {
		dprintk(DEBUG_NORMAL, "%s: invalid args\n", __func__);
		return -EINVAL;
	}

	if (unlikely(!dev->driver)) {
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&ep->lock, flags);

	if (unlikely(!_req || !_req->complete
			|| !_req->buf || !list_empty(&req->queue))) {
		if (!_req)
			dprintk(DEBUG_NORMAL, "%s: 1 X X X\n", __func__);
		else {
			dprintk(DEBUG_NORMAL, "%s: 0 %01d %01d %01d\n",
				__func__, !_req->complete, !_req->buf,
				!list_empty(&req->queue));
		}

		spin_unlock_irqrestore(&ep->lock, flags);
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	dprintk(DEBUG_VERBOSE, "%s: ep%x len %d\n",
		 __func__, ep->bEndpointAddress, _req->length);

	if (ep->bEndpointAddress & USB_DIR_IN) {
		ep_sts = gxio_usb_dev_get_in_sts(dev->usb_ctx, ep_num);
		dprintk(DEBUG_VERBOSE, "%s: ep%d IN ep_sts 0x%x\n",
			__func__, ep_num, ep_sts);
	} else {
		ep_sts = gxio_usb_dev_get_out_sts(dev->usb_ctx, ep_num);
		fifo_count = (ep_sts >>
			USB_DEVICE_EP0_IN_STS_UDC_REG__RX_PKT_SIZE_SHIFT) &
			USB_DEVICE_EP0_IN_STS_UDC_REG__RX_PKT_SIZE_RMASK;
		dprintk(DEBUG_VERBOSE, "%s: OUT ep_sts 0x%x, fifo_count %d\n",
		 __func__, ep_sts, fifo_count);
	}

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->halted) {
		dprintk(DEBUG_VERBOSE, "kickstart this io queue\n");
		if (ep->bEndpointAddress == 0) {
			switch (dev->ep0state) {
			case EP0_IDLE:
				dprintk(DEBUG_VERBOSE,
					"ep0, IDLE state, what now?\n");
			case EP0_IN_DATA_PHASE:
					if (tile_udc_write_fifo(dev->usb_ctx,
							ep, req)) {
						dev->ep0state = EP0_IDLE;
						req = NULL;
					}
				break;

			case EP0_OUT_DATA_PHASE:
				if ((!_req->length)
					|| ((((ep_sts >> 4) & 3) == 2)
					&& tile_udc_read_fifo(dev->usb_ctx,
							ep, req))) {
					dev->ep0state = EP0_IDLE;
					req = NULL;
				}
				break;

			default:
				tile_udc_clear_nak(dev->usb_ctx, ep_num);
				spin_unlock_irqrestore(&ep->lock, flags);
				return -EL2HLT;
			}

			/*  SETUP requests set a NAK bit. */
			tile_udc_clear_nak(dev->usb_ctx, ep_num);

		} else if ((ep->bEndpointAddress & USB_DIR_IN) != 0) {
			/* Enable irqs. */
			int_en_reg = gxio_usb_dev_get_ep_intr_msk(dev->usb_ctx);
			gxio_usb_dev_set_ep_intr_msk(dev->usb_ctx,
					int_en_reg & (~(1 << ep_num)));
			dprintk(DEBUG_VERBOSE, "ep%d enable irq\n", ep_num);
			/*
			 * FIXME: For massive data transfer, we need to
			 * wait at least 200 us for the USB device
			 * controller processing the data, otherwise
			 * the USB device controller stops generating
			 * TXEMPTY interrupt.
			 */
			udelay(200);

			if ((ep_sts &
			    (USB_DEVICE_EP0_IN_STS_UDC_REG__TXEMPTY_MASK |
			    USB_DEVICE_EP0_IN_STS_UDC_REG__IN_RCV_MASK)) &&
			    tile_udc_write_fifo(dev->usb_ctx, ep, req))
				req = NULL;

		} else {
			if ((ep_sts &
			    USB_DEVICE_EP0_IN_STS_UDC_REG__OUT_RCV_MASK) &&
			    req) {
				if (tile_udc_read_fifo(dev->usb_ctx, ep, req))
					req = NULL;

				gxio_usb_dev_set_out_sts(dev->usb_ctx,
							 ep_num, 0x10);
			}
		}
	}

	/* irq handler advances the queue. */
	if (likely(req != NULL)) {
		dprintk(DEBUG_NORMAL, "%s list_add_tail req\n", __func__);
		list_add_tail(&req->queue, &ep->queue);
	}

	spin_unlock_irqrestore(&ep->lock, flags);

	dprintk(DEBUG_NORMAL, "%s ok\n", __func__);
	return 0;
}

static int tile_udc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct tile_ep	*ep = to_tile_ep(_ep);
	struct tile_udc	*udc;
	int			retval = -EINVAL;
	unsigned long		flags;
	struct tile_request	*req = NULL;

	dprintk(DEBUG_VERBOSE, "%s(%p,%p)\n", __func__, _ep, _req);

	if (!the_controller->driver)
		return -ESHUTDOWN;

	if (!_ep || !_req)
		return retval;

	udc = to_tile_udc(ep->gadget);

	spin_lock_irqsave(&ep->lock, flags);

	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req) {
			list_del_init(&req->queue);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}

	if (retval == 0) {
		dprintk(DEBUG_VERBOSE,
			"dequeued req %p from %s, len %d buf %p\n",
			req, _ep->name, _req->length, _req->buf);

		tile_udc_done(ep, req, -ECONNRESET);
	}

	spin_unlock_irqrestore(&ep->lock, flags);
	return retval;
}

static int tile_udc_set_halt(struct usb_ep *_ep, int value)
{
	struct tile_ep	*ep = to_tile_ep(_ep);
	unsigned long		flags;

	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		dprintk(DEBUG_NORMAL, "%s: inval 2\n", __func__);
		return -EINVAL;
	}

	local_irq_save(flags);
	ep->halted = value ? 1 : 0;
	local_irq_restore(flags);

	return 0;
}

static const struct usb_ep_ops tile_ep_ops = {
	.enable		= tile_udc_ep_enable,
	.disable	= tile_udc_ep_disable,

	.alloc_request	= tile_udc_alloc_request,
	.free_request	= tile_udc_free_request,

	.queue		= tile_udc_queue,
	.dequeue	= tile_udc_dequeue,

	.set_halt	= tile_udc_set_halt,
};

/*------------------------- usb_gadget_ops ----------------------------------*/

static int tile_udc_get_frame(struct usb_gadget *gadget)
{
	int dev_sts, frame;
	struct tile_udc *udc = to_tile_udc(gadget);

	dev_sts = gxio_usb_dev_get_d_sts(udc->usb_ctx);
	frame = (dev_sts >> USB_DEVICE_D_STS_UDC_REG__TS_SHIFT) &
		USB_DEVICE_D_STS_UDC_REG__TS_RMASK;

	return frame;
}

static int tile_udc_set_selfpowered(struct usb_gadget *gadget, int value)
{
	struct tile_udc *udc = to_tile_udc(gadget);

	if (value)
		udc->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
	else
		udc->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);

	return 0;
}

static int tile_udc_start(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *));
static int tile_udc_stop(struct usb_gadget_driver *driver);

static int tile_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	dprintk(DEBUG_NORMAL, "%s()\n", __func__);
	return 0;
}

static const struct usb_gadget_ops tile_ops = {
	.get_frame		= tile_udc_get_frame,
	.set_selfpowered	= tile_udc_set_selfpowered,
	.start			= tile_udc_start,
	.stop			= tile_udc_stop,
	.pullup			= tile_udc_pullup,
};

/*------------------------- gadget driver handling---------------------------*/
static void tile_udc_disable(struct tile_udc *dev)
{
	dprintk(DEBUG_NORMAL, "%s()\n", __func__);

	/* Disable all interrupts */
	gxio_usb_dev_set_d_intr_msk(dev->usb_ctx, ~0);
	gxio_usb_dev_set_ep_intr_msk(dev->usb_ctx, ~0);

	/* Clear the interrupt registers */
	gxio_usb_dev_set_d_intr(dev->usb_ctx, ~0);
	gxio_usb_dev_set_ep_intr(dev->usb_ctx, ~0);

	/* Set speed to unknown */
	dev->gadget.speed = USB_SPEED_UNKNOWN;
}

static void tile_udc_reinit(struct tile_udc *dev)
{
	u32 i;

	/* device/ep0 records init. */
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
	dev->ep0state = EP0_IDLE;

	for (i = 0; i < HV_USB_DEV_NUM_EP; i++) {
		struct tile_ep *ep = &dev->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->dev = dev;
		ep->desc = NULL;
		ep->halted = 0;
		INIT_LIST_HEAD(&ep->queue);
	}
}

static void tile_udc_enable(struct tile_udc *dev)
{
	/* Enable all of the interrupts, except for start-of-frame. */
	gxio_usb_dev_set_d_intr_msk(dev->usb_ctx, 0x20);

	/* Enable ep0 interrupt. */
	gxio_usb_dev_set_ep_intr_msk(dev->usb_ctx, 0xfffefffe);
	gxio_usb_dev_reset_interrupt(dev->usb_ctx);
}

 /*
  * Initialize GXIO hardware and irq resources for USB device.
  */
static int tile_udc_init(struct platform_device *pdev)
{
	struct tile_udc *udc = &memory;
	struct device *dev = &pdev->dev;
	struct tilegx_udc_platform_data *pdata = pdev->dev.platform_data;
	int my_cpu, ret;

	spin_lock_init(&udc->lock);

	/*
	 * Try to initialize our GXIO context; if we can't, the device
	 * doesn't exist.
	 */
	if (gxio_usb_dev_init(&pdata->usb_ctx, pdata->dev_index) != 0)
		return -ENXIO;

	udc->usb_ctx = &pdata->usb_ctx;

	/*
	 * Do register setup.  We have to do this as soon as possible
	 * after gxio_usb_dev_init() returns; if we take more than 6 ms,
	 * it's possible that the PHY will have gone into suspend mode
	 * and the accesses to the registers which are in the PHY clock
	 * domain will hang.
	 */

	/* Logical endpoint 0 (control, so in/out) */
	/* Control endpoint */
	gxio_usb_dev_set_in_ctrl(udc->usb_ctx, 0, 0);
	/* 1024 bytes in TX FIFO */
	gxio_usb_dev_set_in_bufsize(udc->usb_ctx, 0, 1024 / 4);
	/* 64 byte packet size */
	gxio_usb_dev_set_in_mpkt_sz(udc->usb_ctx, 0, 64);
	gxio_usb_dev_set_out_ctrl(udc->usb_ctx, 0, 0);
	/* 1024 bytes in RX FIFO, 64-byte packets */
	gxio_usb_dev_set_out_bufsize(udc->usb_ctx, 0, 0x1000040);

	/* Logical endpoint 1 (in) */
	/* Bulk endpoint */
	gxio_usb_dev_set_in_ctrl(udc->usb_ctx, 1, 0x20);
	/* 1024 bytes in TX FIFO */
	gxio_usb_dev_set_in_bufsize(udc->usb_ctx, 1, 1024 / 4);
	/* 512 byte packet size */
	gxio_usb_dev_set_in_mpkt_sz(udc->usb_ctx, 1, 512);

	/* Logical endpoint 2 (out) */
	/* Bulk endpoint */
	gxio_usb_dev_set_out_ctrl(udc->usb_ctx, 2, 0x20);
	/* 1024 bytes in RX FIFO, 512-byte packets */
	gxio_usb_dev_set_out_bufsize(udc->usb_ctx, 2, 0x1000200);

	/* Logical endpoint 3 (in) */
	/* Bulk endpoint */
	gxio_usb_dev_set_in_ctrl(udc->usb_ctx, 3, 0x20);
	/* 1024 bytes in TX FIFO */
	gxio_usb_dev_set_in_bufsize(udc->usb_ctx, 3, 1024 / 4);
	/* 512 byte packet size */
	gxio_usb_dev_set_in_mpkt_sz(udc->usb_ctx, 3, 512);

	/* Logical endpoint 4 (out) */
	/* Bulk endpoint */
	gxio_usb_dev_set_out_ctrl(udc->usb_ctx, 4, 0x20);
	/* 1024 bytes in RX FIFO, 512-byte packets */
	gxio_usb_dev_set_out_bufsize(udc->usb_ctx, 4, 0x1000200);

	/* Logical endpoint 5 (in) */
	/* Bulk endpoint */
	gxio_usb_dev_set_in_ctrl(udc->usb_ctx, 5, 0x20);
	/* 1024 bytes in TX FIFO */
	gxio_usb_dev_set_in_bufsize(udc->usb_ctx, 5, 1024 / 4);
	/* 512 byte packet size */
	gxio_usb_dev_set_in_mpkt_sz(udc->usb_ctx, 5, 512);


	/* Dev config: self-powered, 8-bit UTMI, dynamic CSR programming */
	gxio_usb_dev_set_d_cfg(udc->usb_ctx, 0x20028);
	gxio_usb_dev_set_d_ctrl(udc->usb_ctx, 0);

	/* ep 0, out, control, config 0; 64-byte packets */
	gxio_usb_dev_set_udc_ep(udc->usb_ctx, 0, 0x02000000);
	/* ep 1, in, bulk, config 1; 512-byte packets */
	gxio_usb_dev_set_udc_ep(udc->usb_ctx, 1, 0x100000d1);
	/* ep 2, out, bulk, config 1; 512-byte packets */
	gxio_usb_dev_set_udc_ep(udc->usb_ctx, 2, 0x100000c2);

	/* ep 3, in, bulk, config 1; 512-byte packets */
	gxio_usb_dev_set_udc_ep(udc->usb_ctx, 3, 0x100000d3);
	/* ep 4, out, bulk, config 1; 512-byte packets */
	gxio_usb_dev_set_udc_ep(udc->usb_ctx, 4, 0x100000c4);
	/* ep 5, in, bulk, config 1; 512-byte packets */
	gxio_usb_dev_set_udc_ep(udc->usb_ctx, 5, 0x100000d3);

	the_controller = udc;
	platform_set_drvdata(pdev, udc);

	tile_udc_disable(udc);
	tile_udc_reinit(udc);

	/* Create our IRQs and register them. */
	pdata->irq = create_irq();
	if (pdata->irq < 0) {
		ret = -ENXIO;
		goto err_no_irq;
	}

	tile_irq_activate(pdata->irq, TILE_IRQ_PERCPU);

	/* irq setup after old hardware state is cleaned up. */
	ret = request_irq(pdata->irq, tile_udc_irq,
			  0, gadget_name, (void *)udc);

	if (ret != 0) {
		dev_err(dev, "cannot get irq %i, err %d\n", pdata->irq, ret);
		ret = -EBUSY;
		goto err_destroy_irq;
	}

	dev_dbg(dev, "got irq %i\n", pdata->irq);

	my_cpu = (raw_smp_processor_id() + 1) % nr_cpu_ids;
	/* Configure interrupts. */
	ret = gxio_usb_dev_cfg_interrupt(udc->usb_ctx,
					  cpu_x(my_cpu), cpu_y(my_cpu),
					  KERNEL_PL, pdata->irq);
	if (ret) {
		ret = -ENXIO;
		goto err_free_irq;
	}

	if (tile_udc_debugfs_root) {
		udc->regs_info = debugfs_create_file("registers", S_IRUGO,
				tile_udc_debugfs_root,
				udc, &tile_udc_debugfs_fops);
		if (!udc->regs_info)
			dev_warn(dev, "debugfs file creation failed\n");
	}

	dev_dbg(dev, "probe ok\n");

	return 0;

err_free_irq:
	free_irq(pdata->irq, udc);
err_destroy_irq:
	destroy_irq(pdata->irq);
err_no_irq:
	gxio_usb_dev_destroy(&pdata->usb_ctx);
	return ret;
}

static int tile_udc_start(struct usb_gadget_driver *driver,
			    int (*bind)(struct usb_gadget *))
{
	struct tile_udc *udc;
	int		retval;

	retval = tile_udc_init(tile_udc_platform_device);
	if (retval != 0)
		return retval;

	udc = the_controller;

	dprintk(DEBUG_NORMAL, "%s() '%s'\n", __func__, driver->driver.name);

	/* Sanity checks. */
	if (!udc)
		return -ENODEV;

	if (udc->driver)
		return -EBUSY;

	if (!bind || !driver->setup || driver->max_speed < USB_SPEED_FULL) {
		printk(KERN_ERR "Invalid driver: bind %p setup %p speed %d\n",
			bind, driver->setup, driver->max_speed);
		return -EINVAL;
	}
#if defined(MODULE)
	if (!driver->unbind) {
		printk(KERN_ERR "Invalid driver: no unbind method\n");
		return -EINVAL;
	}
#endif

	/* Hook the driver. */
	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;

	/* Bind the driver. */
	retval = device_add(&udc->gadget.dev);
	if (retval != 0) {
		printk(KERN_ERR "Error in device_add() : %d\n", retval);
		goto register_error;
	}

	dprintk(DEBUG_NORMAL, "binding gadget driver '%s'\n",
		driver->driver.name);

	retval = bind(&udc->gadget);
	if (retval != 0) {
		device_del(&udc->gadget.dev);
		goto register_error;
	}

	/* Enable udc. */
	tile_udc_enable(udc);

	return 0;

register_error:
	udc->driver = NULL;
	udc->gadget.dev.driver = NULL;
	return retval;
}

/*
 *	Un-initialize GXIO hardware resources for USB device.
 */
static int tile_udc_fini(struct platform_device *pdev)
{

	struct tile_udc *udc = platform_get_drvdata(pdev);
	struct tilegx_udc_platform_data *pdata = pdev->dev.platform_data;

	if (udc->driver)
		return -EBUSY;

	debugfs_remove(udc->regs_info);
	free_irq(pdata->irq, udc);
	gxio_usb_dev_destroy(&pdata->usb_ctx);

	platform_set_drvdata(pdev, NULL);
	the_controller = NULL;

	dev_dbg(&pdev->dev, "remove ok\n");
	return 0;
}

static int tile_udc_stop(struct usb_gadget_driver *driver)
{
	struct tile_udc *udc = the_controller;

	if (!udc)
		return -ENODEV;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	dprintk(DEBUG_NORMAL, "usb_gadget_unregister_driver() '%s'\n",
		driver->driver.name);

	/* Report disconnect. */
	if (driver->disconnect)
		driver->disconnect(&udc->gadget);

	driver->unbind(&udc->gadget);

	device_del(&udc->gadget.dev);
	udc->driver = NULL;

	/* Disable udc. */
	tile_udc_disable(udc);

	/* Release GXIO resources. */
	tile_udc_fini(tile_udc_platform_device);

	return 0;
}

/*---------------------------------------------------------------------------*/
static struct tile_udc memory = {
	.gadget = {
		.ops		= &tile_ops,
		.ep0		= &memory.ep[0].ep,
		.name		= gadget_name,
		.dev = {
			.init_name	= "gadget",
		},
	},

	/* control endpoint */
	.ep[0] = {
		.num		= 0,
		.ep = {
			.name		= ep0name,
			.ops		= &tile_ep_ops,
			.maxpacket	= EP0_FIFO_SIZE,
		},
		.dev		= &memory,
	},

	/* first group of endpoints */
	.ep[1] = {
		.num		= 1,
		.ep = {
			.name		= "ep1in-bulk",
			.ops		= &tile_ep_ops,
			.maxpacket	= EP_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= EP_FIFO_SIZE,
		.bEndpointAddress = 1,
		.bmAttributes	= USB_ENDPOINT_XFER_BULK,
	},
	.ep[2] = {
		.num		= 2,
		.ep = {
			.name		= "ep2out-bulk",
			.ops		= &tile_ep_ops,
			.maxpacket	= EP_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= EP_FIFO_SIZE,
		.bEndpointAddress = 2,
		.bmAttributes	= USB_ENDPOINT_XFER_BULK,
	},
	.ep[3] = {
		.num		= 3,
		.ep = {
			.name		= "ep3in-bulk",
			.ops		= &tile_ep_ops,
			.maxpacket	= EP_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= EP_FIFO_SIZE,
		.bEndpointAddress = 3,
		.bmAttributes	= USB_ENDPOINT_XFER_BULK,
	},
	.ep[4] = {
		.num		= 4,
		.ep = {
			.name		= "ep4out-bulk",
			.ops		= &tile_ep_ops,
			.maxpacket	= EP_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= EP_FIFO_SIZE,
		.bEndpointAddress = 4,
		.bmAttributes	= USB_ENDPOINT_XFER_BULK,
	},
	.ep[5] = {
		.num		= 5,
		.ep = {
			.name		= "ep5in-bulk",
			.ops		= &tile_ep_ops,
			.maxpacket	= EP_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= EP_FIFO_SIZE,
		.bEndpointAddress = 5,
		.bmAttributes	= USB_ENDPOINT_XFER_BULK,
	}
};

/*
 *	probe - binds to the platform device
 */
static int tile_udc_probe(struct platform_device *pdev)
{
	struct tile_udc *udc = &memory;

	tile_udc_platform_device = pdev;

	device_initialize(&udc->gadget.dev);
	usb_add_gadget_udc(&pdev->dev, &udc->gadget);
	udc->gadget.dev.parent = &pdev->dev;
	udc->gadget.dev.dma_mask = pdev->dev.dma_mask;

	return 0;
}

static struct platform_driver udc_driver_tile = {
	.driver		= {
		.name	= "tile-usbgadget",
		.owner	= THIS_MODULE,
	},
	.probe		= tile_udc_probe,
};

static int __init udc_init(void)
{
	int retval;

	dprintk(DEBUG_NORMAL, "%s: version %s\n", gadget_name, DRIVER_VERSION);

	tile_udc_debugfs_root = debugfs_create_dir(gadget_name, NULL);
	if (IS_ERR(tile_udc_debugfs_root)) {
		printk(KERN_ERR "%s: debugfs dir creation failed %ld\n",
			gadget_name, PTR_ERR(tile_udc_debugfs_root));
		tile_udc_debugfs_root = NULL;
	}

	retval = platform_driver_register(&udc_driver_tile);
	if (retval)
		goto err;

	return 0;

err:
	debugfs_remove(tile_udc_debugfs_root);
	return retval;
}

static void __exit udc_exit(void)
{
	platform_driver_unregister(&udc_driver_tile);
	debugfs_remove(tile_udc_debugfs_root);
}

module_init(udc_init);
module_exit(udc_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tile-usbgadget");
