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
 */

/*
 *
 * Implementation of USB device gxio calls.
 */

#include <linux/io.h>
#include <linux/errno.h>
#include <linux/module.h>

#include <gxio/usb_dev.h>
#include <gxio/iorpc_globals.h>
#include <gxio/iorpc_usb_dev.h>
#include <gxio/kiorpc.h>

int gxio_usb_dev_init(gxio_usb_dev_context_t * ctx, int usb_dev_index)
{
	char file[32];
	int fd, i;

	memset(ctx, 0, sizeof(*ctx));
	for (i = 0; i < 2 * HV_USB_DEV_NUM_EP; i++)
		ctx->fifo_offset[i] = 0x800;

	snprintf(file, sizeof(file), "usb_dev/%d/iorpc", usb_dev_index);
	fd = hv_dev_open((HV_VirtAddr) file, 0);
	if (fd < 0) {
		if (fd >= GXIO_ERR_MIN && fd <= GXIO_ERR_MAX)
			return fd;
		else
			return -ENODEV;
	}

	ctx->fd = fd;

	// Map in the MMIO space.
	ctx->mmio_base = (void __force *)
		iorpc_ioremap(fd, 0, HV_USB_DEV_MMIO_SIZE);

	if (ctx->mmio_base == NULL) {
		hv_dev_close(ctx->fd);
		return -ENODEV;
	}

	return 0;
}

EXPORT_SYMBOL_GPL(gxio_usb_dev_init);

int gxio_usb_dev_destroy(gxio_usb_dev_context_t * ctx)
{

	iounmap((void __force __iomem *)(ctx->mmio_base));
	hv_dev_close(ctx->fd);

	ctx->mmio_base = NULL;
	ctx->fd = -1;

	return 0;
}

EXPORT_SYMBOL_GPL(gxio_usb_dev_destroy);

void *gxio_usb_dev_get_reg_start(gxio_usb_dev_context_t * ctx)
{
	return ctx->mmio_base;
}

EXPORT_SYMBOL_GPL(gxio_usb_dev_get_reg_start);

size_t gxio_usb_dev_get_reg_len(gxio_usb_dev_context_t * ctx)
{
	return HV_USB_DEV_MMIO_SIZE;
}

EXPORT_SYMBOL_GPL(gxio_usb_dev_get_reg_len);

//
// delta is new fifo size in words minus old fifo size in words.
//
void __gxio_usb_dev_adjust_fifo_ptrs(gxio_usb_dev_context_t * ctx, int ep,
				     int is_tx, int delta)
{
	int i;

	for (i = 1 + ep + (is_tx ? HV_USB_DEV_NUM_EP : 0);
	     i < 2 * HV_USB_DEV_NUM_EP; i++)
		ctx->fifo_offset[i] += 4 * delta;
}

EXPORT_SYMBOL_GPL(__gxio_usb_dev_adjust_fifo_ptrs);

void gxio_usb_dev_read_rx_fifo(gxio_usb_dev_context_t * ctx, int ep,
			       void *dest, int len)
{
	int inptr;

	if (len <= 0) {
		gxio_usb_dev_rd_cfrm(ctx, ep);
		return;
	}

	if (((uintptr_t) dest & 3) == 0) {
		for (inptr = 0; len >= 4; len -= 4, inptr += 4) {
			uint32_t inword = __gxio_mmio_read32(ctx->mmio_base +
							     ctx->
							     fifo_offset[ep] +
							     inptr);
			*((uint32_t *)dest) = inword;
			dest += 4;
		}
	} else if (((uintptr_t) dest & 1) == 0) {
		for (inptr = 0; len >= 4; len -= 4, inptr += 4) {
			uint32_t inword = __gxio_mmio_read32(ctx->mmio_base +
							     ctx->
							     fifo_offset[ep] +
							     inptr);
			*((uint16_t *)dest) = inword;
			dest += 2;
			*((uint16_t *)dest) = inword >> 16;
			dest += 2;
		}
	} else {
		for (inptr = 0; len >= 4; len -= 4, inptr += 4) {
			uint32_t inword = __gxio_mmio_read32(ctx->mmio_base +
							     ctx->
							     fifo_offset[ep] +
							     inptr);
			*((uint8_t *)dest++) = inword;
			*((uint8_t *)dest++) = inword >> 8;
			*((uint8_t *)dest++) = inword >> 16;
			*((uint8_t *)dest++) = inword >> 24;
		}
	}

	//
	// This handles any non-full-word residue.
	//
	for (; len; len--)
		*(uint8_t *)dest++ =
			__gxio_mmio_read8(ctx->mmio_base +
					  ctx->fifo_offset[ep] + inptr++);
}

EXPORT_SYMBOL_GPL(gxio_usb_dev_read_rx_fifo);

void gxio_usb_dev_write_tx_fifo(gxio_usb_dev_context_t * ctx, int ep,
				void *src, int len)
{
	int outptr;

	if (((uintptr_t) src & 3) == 0) {
		for (outptr = 0; len >= 4; len -= 4, outptr += 4) {
			uint32_t outword = *((uint32_t *)src);
			src += 4;
			__gxio_mmio_write32(ctx->mmio_base +
					    ctx->
					    fifo_offset[HV_USB_DEV_NUM_EP +
							ep] + outptr, outword);
		}
	} else if (((uintptr_t) src & 1) == 0) {
		for (outptr = 0; len >= 4; len -= 4, outptr += 4) {
			uint32_t outword = *((uint16_t *)src);
			src += 2;
			outword |= *((uint16_t *)src) << 16;
			src += 2;
			__gxio_mmio_write32(ctx->mmio_base +
					    ctx->
					    fifo_offset[HV_USB_DEV_NUM_EP +
							ep] + outptr, outword);
		}
	} else {
		for (outptr = 0; len >= 4; len -= 4, outptr += 4) {
			uint32_t outword = *(uint8_t *)src++;
			outword |= *(uint8_t *)src++ << 8;
			outword |= *(uint8_t *)src++ << 16;
			outword |= *(uint8_t *)src++ << 24;
			__gxio_mmio_write32(ctx->mmio_base +
					    ctx->
					    fifo_offset[HV_USB_DEV_NUM_EP +
							ep] + outptr, outword);
		}
	}

	//
	// This handles any non-full-word residue.
	//
	for (; len; len--)
		__gxio_mmio_write8(ctx->mmio_base +
				   ctx->fifo_offset[HV_USB_DEV_NUM_EP + ep] +
				   outptr++, *(uint8_t *)src++);

	gxio_usb_dev_wr_cfrm(ctx, ep);
}

EXPORT_SYMBOL_GPL(gxio_usb_dev_write_tx_fifo);
