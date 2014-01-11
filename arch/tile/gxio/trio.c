/*
 * Copyright 2012 Tilera Corporation. All Rights Reserved.
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
 * Implementation of trio gxio calls.
 */

#include <linux/errno.h>
#include <linux/io.h>
#include <linux/module.h>

#include <gxio/trio.h>
#include <gxio/iorpc_globals.h>
#include <gxio/iorpc_trio.h>
#include <gxio/kiorpc.h>

int gxio_trio_destroy(gxio_trio_context_t *context)
{

	if (context->fd >= 0)
		hv_dev_close(context->fd);

	return 0;
}

EXPORT_SYMBOL_GPL(gxio_trio_destroy);

int gxio_trio_init(gxio_trio_context_t *context, unsigned int trio_index)
{
	char file[32];
	int fd;

	snprintf(file, sizeof(file), "trio/%d/iorpc", trio_index);
	fd = hv_dev_open((HV_VirtAddr) file, 0);
	if (fd < 0) {
		context->fd = -1;

		if (fd >= GXIO_ERR_MIN && fd <= GXIO_ERR_MAX)
			return fd;
		else
			return -ENODEV;
	}

	context->fd = fd;

	return 0;
}

EXPORT_SYMBOL_GPL(gxio_trio_init);

int gxio_trio_free_memory_map(gxio_trio_context_t *context, unsigned int map)
{
	return gxio_trio_free_memory_map_aux(context, map);
}

int gxio_trio_init_memory_map(gxio_trio_context_t *context, unsigned int map,
			      void *target_mem, size_t target_size,
			      unsigned int asid, unsigned int mac,
			      uint64_t bus_address,
			      gxio_trio_order_mode_t order_mode)
{
	unsigned long vpn;

	/* Memory maps must be 4kB aligned. */
	if ((unsigned long)target_mem & ((1 << HV_TRIO_PAGE_SHIFT) - 1))
		return GXIO_ERR_ALIGNMENT;

	vpn = (unsigned long)target_mem >> HV_TRIO_PAGE_SHIFT;
	return gxio_trio_init_memory_map_aux(context, map, vpn, target_size,
					     asid, mac, bus_address,
					     order_mode);
}

EXPORT_SYMBOL_GPL(gxio_trio_init_memory_map);

int gxio_trio_unregister_page(gxio_trio_context_t *context, unsigned int asid,
			      void *page)
{
	/*
	 * Initialize a dummy but legal page_size for gxio_trio_unregister_page_aux()
	 * below, though it is not used to unregister a page at all.
	 */

	size_t page_size = HV_TRIO_PAGE_SIZE;

	return gxio_trio_unregister_page_aux(context, page, page_size, 0,
					     asid,
					     (unsigned long)page >>
					     HV_TRIO_PAGE_SHIFT);
}

int gxio_trio_register_page(gxio_trio_context_t *context, unsigned int asid,
			    void *page, size_t page_size,
			    unsigned int page_flags)
{
	return gxio_trio_register_page_aux(context, page, page_size,
					   page_flags, asid,
					   (unsigned long)page >>
					   HV_TRIO_PAGE_SHIFT);
}

EXPORT_SYMBOL_GPL(gxio_trio_register_page);

int gxio_trio_read_isr_status(gxio_trio_context_t *context,
			      unsigned int vec_num)
{
	if (vec_num >= 5)
		return GXIO_ERR_INVAL;

	return gxio_trio_read_isr_status_aux(context, vec_num);
}

EXPORT_SYMBOL_GPL(gxio_trio_read_isr_status);

int gxio_trio_write_isr_status(gxio_trio_context_t *context,
			       unsigned int vec_num, uint32_t bits_to_clear)
{
	if (vec_num >= 5)
		return GXIO_ERR_INVAL;

	return gxio_trio_write_isr_status_aux(context, vec_num, bits_to_clear);
}

EXPORT_SYMBOL_GPL(gxio_trio_write_isr_status);

int gxio_trio_write_mmi_bits(gxio_trio_context_t *context, unsigned int map,
			     unsigned int bits, unsigned int mode)
{
	return gxio_trio_write_mmi_bits_aux(context, map, bits, mode);
}

EXPORT_SYMBOL_GPL(gxio_trio_write_mmi_bits);

int gxio_trio_free_push_dma_ring(gxio_trio_context_t *context,
				 unsigned int ring)
{
	int result = gxio_trio_free_push_dma_ring_aux(context, ring);
	if (result < 0)
		return result;

	return 0;
}

int gxio_trio_init_push_dma_ring(gxio_trio_context_t *context,
				 unsigned int ring, unsigned int mac,
				 unsigned int asid, unsigned int req_flags,
				 void *mem, size_t mem_size,
				 unsigned int mem_flags)
{
	/* Configure the hardware registers. */
	int result = gxio_trio_init_push_dma_ring_aux(context, mem, mem_size,
						      mem_flags, ring, mac,
						      asid,
						      req_flags);
	if (result < 0)
		return result;

	/* Map the push DMA register region into VA space. */
	if (context->mmio_push_dma[ring] == NULL) {
		context->mmio_push_dma[ring] = iorpc_ioremap(context->fd,
							     HV_TRIO_PUSH_DMA_OFFSET
							     (ring),
							     HV_TRIO_DMA_REGION_SIZE);
		if (context->mmio_push_dma[ring] == NULL)
			return -ENODEV;
	}

	return 0;
}

EXPORT_SYMBOL_GPL(gxio_trio_init_push_dma_ring);

int gxio_trio_free_pull_dma_ring(gxio_trio_context_t *context,
				 unsigned int ring)
{
	int result = gxio_trio_free_pull_dma_ring_aux(context, ring);
	if (result < 0)
		return result;

	return 0;
}

int gxio_trio_init_pull_dma_ring(gxio_trio_context_t *context,
				 unsigned int ring, unsigned int mac,
				 unsigned int asid, unsigned int req_flags,
				 void *mem, size_t mem_size,
				 unsigned int mem_flags)
{
	/* Configure the hardware registers. */
	int result = gxio_trio_init_pull_dma_ring_aux(context, mem, mem_size,
						      mem_flags, ring, mac,
						      asid,
						      req_flags);
	if (result < 0)
		return result;

	/* Map the push DMA register region into VA space. */
	if (context->mmio_pull_dma[ring] == NULL) {
		context->mmio_pull_dma[ring] = iorpc_ioremap(context->fd,
							     HV_TRIO_PULL_DMA_OFFSET
							     (ring),
							     HV_TRIO_DMA_REGION_SIZE);
		if (context->mmio_pull_dma[ring] == NULL)
			return -ENODEV;
	}

	return 0;
}

EXPORT_SYMBOL_GPL(gxio_trio_init_pull_dma_ring);

int gxio_trio_init_push_dma_queue(gxio_trio_dma_queue_t *queue,
				  gxio_trio_context_t *context,
				  unsigned int ring, unsigned int mac,
				  unsigned int asid, unsigned int req_flags,
				  void *mem, unsigned int mem_size,
				  unsigned int mem_flags)
{
	/* The init call below will verify that "mem_size" is legal. */
	unsigned int num_entries = mem_size / sizeof(gxio_trio_dma_desc_t);

	int result =
		gxio_trio_init_push_dma_ring(context, ring, mac, asid,
					     req_flags,
					     mem, mem_size, mem_flags);
	if (result < 0)
		return result;

	memset(queue, 0, sizeof(*queue));

	__gxio_dma_queue_init(&queue->dma_queue, context->mmio_push_dma[ring],
			      num_entries, 0);

	queue->dma_descs = mem;
	queue->mask_num_entries = num_entries - 1;
	queue->log2_num_entries = __builtin_ctz(num_entries);

	return 0;
}

EXPORT_SYMBOL_GPL(gxio_trio_init_push_dma_queue);

int gxio_trio_init_pull_dma_queue(gxio_trio_dma_queue_t *queue,
				  gxio_trio_context_t *context,
				  unsigned int ring, unsigned int mac,
				  unsigned int asid, unsigned int req_flags,
				  void *mem, unsigned int mem_size,
				  unsigned int mem_flags)
{
	/* The init call below will verify that "mem_size" is legal. */
	unsigned int num_entries = mem_size / sizeof(gxio_trio_dma_desc_t);

	int result =
		gxio_trio_init_pull_dma_ring(context, ring, mac, asid,
					     req_flags,
					     mem, mem_size, mem_flags);
	if (result < 0)
		return result;

	memset(queue, 0, sizeof(*queue));

	__gxio_dma_queue_init(&queue->dma_queue, context->mmio_pull_dma[ring],
			      num_entries, 0);

	queue->dma_descs = mem;
	queue->mask_num_entries = num_entries - 1;
	queue->log2_num_entries = __builtin_ctz(num_entries);

	return 0;
}

EXPORT_SYMBOL_GPL(gxio_trio_init_pull_dma_queue);
