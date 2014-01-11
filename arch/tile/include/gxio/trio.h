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
 * An API for allocating, configuring, and manipulating TRIO hardware
 * resources
 */

/*
 *
 * The TILE-Gx TRIO shim provides connections to external devices via
 * PCIe or other transaction IO standards.  The gxio_trio_ API,
 * declared in <gxio/trio.h>, allows applications to allocate and
 * configure TRIO IO resources like DMA command rings, memory map
 * windows, and device interrupts.  The following sections introduce
 * the various components of the API.  We strongly recommend reading
 * the TRIO section of the IO Device Guide (UG404) before working with
 * this API.
 *
 * @section trio__ingress TRIO Ingress Hardware Resources
 *
 * The TRIO ingress hardware is responsible for examining incoming
 * PCIe or StreamIO packets and choosing a processing mechanism based
 * on the packets' bus address.  The gxio_trio_ API can be used to
 * configure different handlers for different ranges of bus address
 * space.  The user can configure "mapped memory" and "scatter queue"
 * regions to match incoming packets within 4kB-aligned ranges of bus
 * addresses.  Each range specifies a different set of mapping
 * parameters to be applied when handling the ingress packet.  The
 * following sections describe how to work with MapMem and scatter
 * queue regions.
 *
 * @subsection trio__mapmem TRIO MapMem Regions
 *
 * TRIO mapped memory (or MapMem) regions allow the user to map
 * incoming read and write requests directly to the application's
 * memory space.  MapMem regions are allocated via
 * gxio_trio_alloc_memory_maps().  Given an integer MapMem number,
 * applications can use gxio_trio_init_memory_map() to specify the
 * range of bus addresses that will match the region and the range of
 * virtual addresses to which those packets will be applied.
 *
 * As with many other gxio APIs, the programmer must be sure to
 * register memory pages that will be used with MapMem regions.  Pages
 * can be registered with TRIO by allocating an ASID (address space
 * identifier) and then using gxio_trio_register_page() to register up to
 * 16 pages with the hardware.  The initialization functions for
 * resources that require registered memory (MapMem, scatter queues,
 * push DMA, and pull DMA) then take an 'asid' parameter in order to
 * configure which set of registered pages is used by each resource.
 *
 * @subsection trio__scatter_queue TRIO Scatter Queues
 *
 * The TRIO shim's scatter queue regions allow users to dynamically
 * map buffers from a large address space into a small range of bus
 * addresses.  This is particularly helpful for PCIe endpoint devices,
 * where the host generally limits the size of BARs to tens of
 * megabytes.
 *
 * Each scatter queue consists of a memory map region, a queue of
 * tile-side buffer VAs to be mapped to that region, and a bus-mapped
 * "doorbell" register that the remote endpoint can write to trigger a
 * dequeue of the current buffer VA, thus swapping in a new buffer.
 * The VAs pushed onto a scatter queue must be 4kB aligned, so
 * applications may need to use higher-level protocols to inform
 * remote entities that they should apply some additional, sub-4kB
 * offset when reading or writing the scatter queue region.  For more
 * information, see the IO Device Guide (UG404).
 *
 * @section trio__egress TRIO Egress Hardware Resources
 *
 * The TRIO shim supports two mechanisms for egress packet generation:
 * programmed IO (PIO) and push/pull DMA.  PIO allows applications to
 * create MMIO mappings for PCIe or StreamIO address space, such that
 * the application can generate word-sized read or write transactions
 * by issuing load or store instructions.  Push and pull DMA are tuned
 * for larger transactions; they use specialized hardware engines to
 * transfer large blocks of data at line rate.
 *
 * @subsection trio__pio TRIO Programmed IO
 *
 * Programmed IO allows applications to create MMIO mappings for PCIe
 * or StreamIO address space.  The hardware PIO regions support access
 * to PCIe configuration, IO, and memory space, but the gxio_trio API
 * only supports memory space accesses.  PIO regions are allocated
 * with gxio_trio_alloc_pio_regions() and initialized via
 * gxio_trio_init_pio_region().  Once a region is bound to a range of
 * bus address via the initialization function, the application can
 * use gxio_trio_map_pio_region() to create MMIO mappings from its VA
 * space onto the range of bus addresses supported by the PIO region.
 *
 * @subsection trio_dma TRIO Push and Pull DMA
 *
 * The TRIO push and pull DMA engines allow users to copy blocks of
 * data between application memory and the bus.  Push DMA generates
 * write packets that copy from application memory to the bus and pull
 * DMA generates read packets that copy from the bus into application
 * memory.  The DMA engines are managed via an API that is very
 * similar to the mPIPE eDMA interface.  For a detailed explanation of
 * the eDMA queue API, see @ref gxio_mpipe_wrappers.
 *
 * Push and pull DMA queues are allocated via
 * gxio_trio_alloc_push_dma_ring() / gxio_trio_alloc_pull_dma_ring().
 * Once allocated, users generally use a ::gxio_trio_dma_queue_t
 * object to manage the queue, providing easy wrappers for reserving
 * command slots in the DMA command ring, filling those slots, and
 * waiting for commands to complete.  DMA queues can be initialized
 * via gxio_trio_init_push_dma_queue() or
 * gxio_trio_init_pull_dma_queue().
 *
 * See @ref trio/push_dma/app.c for an example of how to use push DMA.
 *
 * @section trio_shortcomings Plans for Future API Revisions
 *
 * The simulation framework is incomplete.  Future features include:
 *
 * - Support for reset and deallocation of resources.
 *
 * - Support for pull DMA.
 *
 * - Support for interrupt regions and user-space interrupt delivery.
 *
 * - Support for getting BAR mappings and reserving regions of BAR
 *   address space.
 */
#ifndef _GXIO_TRIO_H_
#define _GXIO_TRIO_H_

#include <linux/types.h>

#include <gxio/common.h>
#include <gxio/dma_queue.h>

#include <arch/trio_constants.h>
#include <arch/trio.h>
#include <arch/trio_pcie_intfc.h>
#include <arch/trio_pcie_rc.h>

#include <arch/trio_pcie_ep.h>

#include <arch/trio_shm.h>
#include <hv/drv_trio_intf.h>
#include <hv/iorpc.h>

/* A context object used to manage TRIO hardware resources. */
typedef struct {

	/* File descriptor for calling up to Linux (and thus the HV). */
	int fd;

	/* The VA at which the MAC MMIO registers are mapped. */
	char *mmio_base_mac;

	/* The VA at which the PIO config space are mapped for each PCIe MAC.
	   Gx36 has max 3 PCIe MACs per TRIO shim. */
	char *mmio_base_pio_cfg[TILEGX_TRIO_PCIES];

#ifdef USE_SHARED_PCIE_CONFIG_REGION
	/* Index of the shared PIO region for PCI config access. */
	int pio_cfg_index;
#else
	/* Index of the PIO region for PCI config access per MAC. */
	int pio_cfg_index[TILEGX_TRIO_PCIES];
#endif

	/*  The VA at which the push DMA MMIO registers are mapped. */
	char *mmio_push_dma[TRIO_NUM_PUSH_DMA_RINGS];

	/*  The VA at which the pull DMA MMIO registers are mapped. */
	char *mmio_pull_dma[TRIO_NUM_PUSH_DMA_RINGS];

	/* Application space ID. */
	unsigned int asid;

} gxio_trio_context_t;

/* Command descriptor for push or pull DMA. */
typedef TRIO_DMA_DESC_t gxio_trio_dma_desc_t;

/* A convenient, thread-safe interface to an eDMA ring. */
typedef struct {

	/* State object for tracking head and tail pointers. */
	__gxio_dma_queue_t dma_queue;

	/* The ring entries. */
	gxio_trio_dma_desc_t *dma_descs;

	/* The number of entries minus one. */
	unsigned long mask_num_entries;

	/* The log2() of the number of entries. */
	unsigned int log2_num_entries;

} gxio_trio_dma_queue_t;

/* Destroy a TRIO context.
 *
 * This function de-allocates a TRIO "service domain" and unmaps the MMIO
 * registers from the the caller's VA space.
 *
 * @param context An initialized context object to be destroyed, returned
 * by gxio_trio_init().
 */
extern int gxio_trio_destroy(gxio_trio_context_t *context);

/* Initialize a TRIO context.
 *
 * This function allocates a TRIO "service domain" and maps the MMIO
 * registers into the the caller's VA space.
 *
 * @param context Context object to be initialized.
 * @param trio_index Which TRIO shim; Gx36 must pass 0.
 */
extern int gxio_trio_init(gxio_trio_context_t *context,
			  unsigned int trio_index);

/* This indicates that an ASID hasn't been allocated. */
#define GXIO_ASID_NULL -1

/* Ordering modes for map memory regions and scatter queue regions. */
typedef enum gxio_trio_order_mode_e {
	/* Writes are not ordered.  Reads always wait for previous writes. */
	GXIO_TRIO_ORDER_MODE_UNORDERED =
		TRIO_MAP_MEM_SETUP__ORDER_MODE_VAL_UNORDERED,
	/* Both writes and reads wait for previous transactions to complete. */
	GXIO_TRIO_ORDER_MODE_STRICT =
		TRIO_MAP_MEM_SETUP__ORDER_MODE_VAL_STRICT,
	/* Writes are ordered unless the incoming packet has the
	   relaxed-ordering attributes set. */
	GXIO_TRIO_ORDER_MODE_OBEY_PACKET =
		TRIO_MAP_MEM_SETUP__ORDER_MODE_VAL_REL_ORD
} gxio_trio_order_mode_t;

/* Free a memory mapping region.
 *
 * @param context An initialized TRIO context.
 * @param map An initialized Memory map region.
 * @return Zero on success, else ::GXIO_TRIO_ERR_BAD_MEMORY_MAP if illegal
 *   memory map region.
 */
extern int gxio_trio_free_memory_map(gxio_trio_context_t *context,
				     unsigned int map);

/* Initialize a memory mapping region.
 *
 * @param context An initialized TRIO context.
 * @param map A Memory map region allocated by gxio_trio_alloc_memory_maps().
 * @param target_mem VA of backing memory, should be registered via
 *   gxio_trio_register_page() and aligned to 4kB.
 * @param target_size Length of the memory mapping, must be a multiple
 * of 4kB.
 * @param asid ASID to be used for Tile-side address translation.
 * @param mac MAC number.
 * @param bus_address Bus address at which the mapping starts.
 * @param order_mode Memory ordering mode for this mapping.
 * @return Zero on success, else ::GXIO_TRIO_ERR_BAD_MEMORY_MAP,
 *   GXIO_TRIO_ERR_BAD_ASID, or ::GXIO_TRIO_ERR_BAD_BUS_RANGE.
 */
extern int gxio_trio_init_memory_map(gxio_trio_context_t *context,
				     unsigned int map, void *target_mem,
				     size_t target_size, unsigned int asid,
				     unsigned int mac, uint64_t bus_address,
				     gxio_trio_order_mode_t order_mode);

/* Flags that can be passed to resource allocation functions. */
enum gxio_trio_alloc_flags_e {
	GXIO_TRIO_ALLOC_FIXED = HV_TRIO_ALLOC_FIXED,
};

/* Flags that can be passed to memory registration functions. */
enum gxio_trio_mem_flags_e {
	/* Do not fill L3 when writing, and invalidate lines upon egress. */
	GXIO_TRIO_MEM_FLAG_NT_HINT = IORPC_MEM_BUFFER_FLAG_NT_HINT,

	/* L3 cache fills should only populate IO cache ways. */
	GXIO_TRIO_MEM_FLAG_IO_PIN = IORPC_MEM_BUFFER_FLAG_IO_PIN,
};

/* Flag indicating a request generator uses a special traffic
    class. */
#define GXIO_TRIO_FLAG_TRAFFIC_CLASS(N) HV_TRIO_FLAG_TC(N)

/* Flag indicating a request generator uses a virtual function
    number. */
#define GXIO_TRIO_FLAG_VFUNC(N) HV_TRIO_FLAG_VFUNC(N)

/*****************************************************************
 *                       Memory Registration                      *
 ******************************************************************/

/* De-allocate an allocated Application Space Identifiers (ASIDs).
 *
 * @param context An initialized TRIO context.
 * @param asid Index of ASID to be de-allocated.
 * @return 0 on success, or ::GXIO_TRIO_ERR_BAD_ASID if de-allocation
 *   failed.
 */
extern int gxio_trio_dealloc_asid(gxio_trio_context_t *context,
				  unsigned int asid);

/* Allocate Application Space Identifiers (ASIDs).  Each ASID can
 * register up to 16 page translations.  ASIDs are used by memory map
 * regions, scatter queues, and DMA queues to translate application
 * VAs into memory system PAs.
 *
 * @param context An initialized TRIO context.
 * @param count Number of ASIDs required.
 * @param first Index of first ASID if ::GXIO_TRIO_ALLOC_FIXED flag
 *   is set, otherwise ignored.
 * @param flags Flag bits, including bits from ::gxio_trio_alloc_flags_e.
 * @return Index of first ASID, or ::GXIO_TRIO_ERR_NO_ASID if allocation
 *   failed.
 */
extern int gxio_trio_alloc_asids(gxio_trio_context_t *context,
				 unsigned int count, unsigned int first,
				 unsigned int flags);

/* Unregister a page with an ASID.
 *
 * @param context An initialized TRIO context.
 * @param asid An ASID allocated via gxio_trio_alloc_asid().
 * @param page Starting VA of a contiguous memory page.
 * @return Zero on success, EINVAL if page does not map a contiguous
 *   page, ::GXIO_TRIO_ERR_BAD_ASID if illegal ASID or ::GXIO_ERR_IOTLB_ENTRY
 *   if no registered IOTLB entry is found.
 */
extern int gxio_trio_unregister_page(gxio_trio_context_t *context,
				     unsigned int asid, void *page);

/* Register a page with an ASID.  Each ASID can map up to 16 pages.
 * Tile-side memory addresses generated by memory map regions, scatter
 * queues, push DMA rings, and pull DMA rings must reference a VA that
 * has been registered with this function.
 *
 * @param context An initialized TRIO context.
 * @param asid An ASID allocated via gxio_trio_alloc_asid().
 * @param page Starting VA of a contiguous memory page.
 * @param page_size Size of the page.
 * @param page_flags ::gxio_trio_mem_flags_e memory flags.
 * @return Zero on success, EINVAL if page does not map a contiguous
 *   page, ::GXIO_ERR_IOTLB_ENTRY if no more IOTLB entries are
 *   available.
 */
extern int gxio_trio_register_page(gxio_trio_context_t *context,
				   unsigned int asid, void *page,
				   size_t page_size, unsigned int page_flags);

/* Free a push DMA ring.
 *
 * @param context An initialized TRIO context.
 * @param ring An initialized push DMA ring.
 * @return Zero on success, else ::GXIO_TRIO_ERR_BAD_PUSH_DMA_RING if illegal
 *   push DMA ring.
 */
extern int gxio_trio_free_push_dma_ring(gxio_trio_context_t *context,
					unsigned int ring);

/* Initialize a gxio_trio_dma_queue_t for use with push DMA.
 *
 * Takes the queue plus the same args as gxio_trio_init_push_dma_ring().
 */
extern int gxio_trio_init_push_dma_queue(gxio_trio_dma_queue_t *queue,
					 gxio_trio_context_t *context,
					 unsigned int ring, unsigned int mac,
					 unsigned int asid,
					 unsigned int req_flags, void *mem,
					 unsigned int mem_size,
					 unsigned int mem_flags);

/* Free a pull DMA ring.
 *
 * @param context An initialized TRIO context.
 * @param ring An initialized pull DMA ring.
 * @return Zero on success, else ::GXIO_TRIO_ERR_BAD_PULL_DMA_RING if illegal
 *   pull DMA ring.
 */
extern int gxio_trio_free_pull_dma_ring(gxio_trio_context_t *context,
					unsigned int ring);

/* Initialize a gxio_trio_dma_queue_t for use with pull DMA.
 *
 * Takes the queue plus the same args as gxio_trio_init_pull_dma_ring().
 */
extern int gxio_trio_init_pull_dma_queue(gxio_trio_dma_queue_t *queue,
					 gxio_trio_context_t *context,
					 unsigned int ring, unsigned int mac,
					 unsigned int asid,
					 unsigned int req_flags, void *mem,
					 unsigned int mem_size,
					 unsigned int mem_flags);

/* Reserve slots for DMA commands.
 *
 * Use gxio_trio_dma_queue_put_at() to actually populate the slots.
 *
 * @param queue An dma queue initialized via
 * gxio_trio_push_dma_queue_init() or gxio_trio_pull_dma_queue_init().
 * @param num Number of slots to reserve.
 * @return The first reserved slot, or a negative error code.
 */
static int64_t inline gxio_trio_dma_queue_reserve(gxio_trio_dma_queue_t *queue,
						  unsigned int num)
{
	return __gxio_dma_queue_reserve_aux(&queue->dma_queue, num, 1);
}

/* Post a DMA command to a DMA queue at a given slot.
 *
 * This routine is often used to insert DMA commands into the queue in
 * a pre-determined order.  The implementation does not check to make
 * sure DMA command credits are available.  It is assumed that the
 * user knows the ring can never overflow, generally because the
 * application uses fewer buffers than there are slots in the DMA
 * queue.
 *
 * @param queue An dma queue initialized via
 * gxio_trio_push_dma_queue_init() or gxio_trio_pull_dma_queue_init().
 * @param dma_desc DMA command to be posted.
 * @param slot An egress slot (only the low bits are actually used).
 */
static void inline gxio_trio_dma_queue_put_at(gxio_trio_dma_queue_t *queue,
					      gxio_trio_dma_desc_t dma_desc,
					      uint64_t slot)
{
	uint64_t dma_slot = slot & queue->mask_num_entries;
	gxio_trio_dma_desc_t *desc_p = &queue->dma_descs[dma_slot];

	/*
	 * ISSUE: Could set DMA ring to be on generation 1 at start, which
	 * would avoid the negation here, perhaps allowing __insn_bfins().
	 */
	dma_desc.gen = !((slot >> queue->log2_num_entries) & 1);

	/*
	 * We use ACCESS_ONCE(), plus the fact that the DMA
	 * queue alignment restrictions ensure that these two words are on
	 * the same cacheline, to force proper ordering between the stores.
	 */
	ACCESS_ONCE(desc_p->words[1]) = dma_desc.words[1];
	ACCESS_ONCE(desc_p->words[0]) = dma_desc.words[0];
}

/* Ask the TRIO hardware to start DMA immediately.
 *
 * This call is not necessary, but may slightly reduce overall latency.
 *
 * @param queue A DMA queue initialized via
 * gxio_trio_push_dma_queue_init() or gxio_trio_pull_dma_queue_init().
 */
static void inline gxio_trio_dma_queue_flush(gxio_trio_dma_queue_t *queue)
{
	/* Use "ring_idx = 0" and "count = 0" to "wake up" the DMA ring. */
	TRIO_PULL_DMA_REGION_VAL_t val = { {0} };
	__insn_flushwb();	/* Flush the write buffers. */
	__gxio_mmio_write(queue->dma_queue.post_region_addr, val.word);
}

/* Find out how many DMA queue descriptors have been completely processed.
 *
 * @param queue A DMA queue initialized via
 * gxio_trio_push_dma_queue_init() or gxio_trio_pull_dma_queue_init().
 * @result A 16-bit running count of the number of descriptors that
 * have been completely processed.
 */
static uint16_t inline
gxio_trio_read_dma_queue_complete_count(gxio_trio_dma_queue_t *queue)
{
	TRIO_PUSH_DMA_REGION_VAL_t read_val;
	read_val.word = __gxio_mmio_read(queue->dma_queue.post_region_addr);

	return read_val.count;
}

/* Read out the interrupt status from TRIO_INT_VEC0 to 4.
 *
 * @param context An initialized TRIO context.
 * @param vec_num Interrupt vector number.
 * @return Interrupt status on success, else ::GXIO_ERR_INVAL.
 */
extern int gxio_trio_read_isr_status(gxio_trio_context_t *context,
				     unsigned int vec_num);

/* Write the interrupt status to TRIO_INT_VEC0 to 4 to clear particular
 *  interrupts.
 *
 * @param context An initialized TRIO context.
 * @param vec_num Interrupt vector number.
 * @param bits_to_clear Interrupt bits to be cleared within the paticular
 * interrupt vector.
 * @return Zero on success, else ::GXIO_ERR_INVAL.
 */
extern int gxio_trio_write_isr_status(gxio_trio_context_t *context,
				      unsigned int vec_num,
				      uint32_t bits_to_clear);

/*****************************************************************
 *                      Host Interrupt management                 *
 ******************************************************************/

/* Trigger the PCIe RC host MSI/MSI-X interrupt.
 *
 * @param context An initialized TRIO context.
 * @param mac MAC number.
 * @param can_spin Flag indicating if this can spin.
 * @param msix_addr MSI-X message address.
 * @param msix_data MSI-X message data.
 * @return Zero on success, else ::GXIO_ERR_BUSY.
 */
static int inline gxio_trio_trigger_host_interrupt(gxio_trio_context_t
						   *context, unsigned int mac,
						   unsigned int can_spin,
						   unsigned long msix_addr,
						   unsigned int msix_data)
{
	TRIO_PCIE_INTFC_EP_INT_GEN_t intr_setup;
	TRIO_SEMAPHORE0_t sem;

	/* Use the Semaphore register for synchronization purpose. */
	sem.word = __gxio_mmio_read(context->mmio_base_mac +
				    ((TRIO_SEMAPHORE0 <<
				      TRIO_CFG_REGION_ADDR__REG_SHIFT) |
				     (TRIO_CFG_REGION_ADDR__INTFC_VAL_TRIO <<
				      TRIO_CFG_REGION_ADDR__INTFC_SHIFT)));
	if (sem.val && !can_spin)
		return GXIO_ERR_BUSY;

	while (sem.val)
		sem.word = __gxio_mmio_read(context->mmio_base_mac +
					    ((TRIO_SEMAPHORE0 <<
					      TRIO_CFG_REGION_ADDR__REG_SHIFT)
					     |
					     (TRIO_CFG_REGION_ADDR__INTFC_VAL_TRIO
					      <<
					      TRIO_CFG_REGION_ADDR__INTFC_SHIFT)));

	__insn_mf();

	/*
	 * Generate either MSI or MSI-X, based on the msi_ena field
	 * in register TRIO_PCIE_INTFC_EP_INT_GEN.
	 */
	intr_setup.word = __gxio_mmio_read(context->mmio_base_mac +
					   ((TRIO_PCIE_INTFC_EP_INT_GEN <<
					     TRIO_CFG_REGION_ADDR__REG_SHIFT) |
					    (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE
					     <<
					     TRIO_CFG_REGION_ADDR__INTFC_SHIFT)
					    | (mac <<
					       TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)));

	if (intr_setup.msi_ena == 0) {
		/* Dispatch MSI-X. Set the MSIX_DATA and MSIX_ADDR registers. */
		TRIO_PCIE_INTFC_EP_MSIX_DATA_t data;
		TRIO_PCIE_INTFC_EP_MSIX_ADDR_t addr;

		data.word = 0;
		data.val = msix_data;
		addr.word = 0;
		addr.val = msix_addr;
		__gxio_mmio_write(context->mmio_base_mac +
				  ((TRIO_PCIE_INTFC_EP_MSIX_DATA <<
				    TRIO_CFG_REGION_ADDR__REG_SHIFT) |
				   (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE
				    << TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
				   (mac <<
				    TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)),
				  data.word);

		__gxio_mmio_write(context->mmio_base_mac +
				  ((TRIO_PCIE_INTFC_EP_MSIX_ADDR <<
				    TRIO_CFG_REGION_ADDR__REG_SHIFT) |
				   (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE
				    << TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
				   (mac <<
				    TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)),
				  addr.word);
	}

	while (intr_setup.send_msi)
		intr_setup.word = __gxio_mmio_read(context->mmio_base_mac +
						   ((TRIO_PCIE_INTFC_EP_INT_GEN
						     <<
						     TRIO_CFG_REGION_ADDR__REG_SHIFT)
						    |
						    (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE
						     <<
						     TRIO_CFG_REGION_ADDR__INTFC_SHIFT)
						    | (mac <<
						       TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)));

	intr_setup.msi_vf_ena = 0;
	intr_setup.send_msi = 1;
	__gxio_mmio_write(context->mmio_base_mac +
			  ((TRIO_PCIE_INTFC_EP_INT_GEN <<
			    TRIO_CFG_REGION_ADDR__REG_SHIFT) |
			   (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			    TRIO_CFG_REGION_ADDR__INTFC_SHIFT) | (mac <<
								  TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)),
			  intr_setup.word);

	__insn_mf();

	sem.val = 0;
	__gxio_mmio_write(context->mmio_base_mac +
			  ((TRIO_SEMAPHORE0 << TRIO_CFG_REGION_ADDR__REG_SHIFT)
			   | (TRIO_CFG_REGION_ADDR__INTFC_VAL_TRIO <<
			      TRIO_CFG_REGION_ADDR__INTFC_SHIFT)), sem.word);

	return 0;
}

/* Trigger the PCIe RC host MSI/MSI-X interrupt for a virtual function.
 *
 * @param context An initialized TRIO context.
 * @param mac MAC number.
 * @param can_spin Flag indicating if this can spin.
 * @param msix_addr MSI-X message address.
 * @param msix_data MSI-X message data.
 * @param vf Virtual function instance number.
 * @return Zero on success, else ::GXIO_ERR_BUSY.
 */
static int inline gxio_trio_trigger_host_interrupt_vf(gxio_trio_context_t
						      *context,
						      unsigned int mac,
						      unsigned int can_spin,
						      unsigned long msix_addr,
						      unsigned int msix_data,
						      unsigned int vf)
{
	TRIO_PCIE_INTFC_EP_INT_GEN_t intr_setup;
	TRIO_SEMAPHORE0_t sem;

	/* Use the Semaphore register for synchronization purpose. */
	sem.word = __gxio_mmio_read(context->mmio_base_mac +
				    ((TRIO_SEMAPHORE0 <<
				      TRIO_CFG_REGION_ADDR__REG_SHIFT) |
				     (TRIO_CFG_REGION_ADDR__INTFC_VAL_TRIO <<
				      TRIO_CFG_REGION_ADDR__INTFC_SHIFT)));
	if (sem.val && !can_spin)
		return GXIO_ERR_BUSY;

	while (sem.val)
		sem.word = __gxio_mmio_read(context->mmio_base_mac +
					    ((TRIO_SEMAPHORE0 <<
					      TRIO_CFG_REGION_ADDR__REG_SHIFT)
					     |
					     (TRIO_CFG_REGION_ADDR__INTFC_VAL_TRIO
					      <<
					      TRIO_CFG_REGION_ADDR__INTFC_SHIFT)));

	__insn_mf();

	/*
	 * Generate either MSI or MSI-X, based on the msi_ena field
	 * in register TRIO_PCIE_INTFC_EP_INT_GEN.
	 */
	intr_setup.word = __gxio_mmio_read(context->mmio_base_mac +
					   ((TRIO_PCIE_INTFC_EP_INT_GEN <<
					     TRIO_CFG_REGION_ADDR__REG_SHIFT) |
					    (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE
					     <<
					     TRIO_CFG_REGION_ADDR__INTFC_SHIFT)
					    | (mac <<
					       TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)));

	/*
	 * Note that the msi_ena field reflects the state of the MSI enable
	 * in the PF. This implies that, in order to use the MSI for the VF,
	 * the PF must be configured for MSI instead of MSI-X.
	 */
	if (intr_setup.msi_ena == 0) {
		/* Dispatch MSI-X. Set the MSIX_DATA and MSIX_ADDR registers. */
		TRIO_PCIE_INTFC_EP_MSIX_DATA_t data;
		TRIO_PCIE_INTFC_EP_MSIX_ADDR_t addr;

		data.word = 0;
		data.val = msix_data;
		addr.word = 0;
		addr.val = msix_addr;
		__gxio_mmio_write(context->mmio_base_mac +
				  ((TRIO_PCIE_INTFC_EP_MSIX_DATA <<
				    TRIO_CFG_REGION_ADDR__REG_SHIFT) |
				   (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE
				    << TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
				   (mac <<
				    TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)),
				  data.word);

		__gxio_mmio_write(context->mmio_base_mac +
				  ((TRIO_PCIE_INTFC_EP_MSIX_ADDR <<
				    TRIO_CFG_REGION_ADDR__REG_SHIFT) |
				   (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE
				    << TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
				   (mac <<
				    TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)),
				  addr.word);
	}

	while (intr_setup.send_msi)
		intr_setup.word = __gxio_mmio_read(context->mmio_base_mac +
						   ((TRIO_PCIE_INTFC_EP_INT_GEN
						     <<
						     TRIO_CFG_REGION_ADDR__REG_SHIFT)
						    |
						    (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE
						     <<
						     TRIO_CFG_REGION_ADDR__INTFC_SHIFT)
						    | (mac <<
						       TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)));

	intr_setup.msi_vf_ena = 1;
	intr_setup.msi_vf_sel = vf;
	intr_setup.send_msi = 1;
	__gxio_mmio_write(context->mmio_base_mac +
			  ((TRIO_PCIE_INTFC_EP_INT_GEN <<
			    TRIO_CFG_REGION_ADDR__REG_SHIFT) |
			   (TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			    TRIO_CFG_REGION_ADDR__INTFC_SHIFT) | (mac <<
								  TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT)),
			  intr_setup.word);

	__insn_mf();

	sem.val = 0;
	__gxio_mmio_write(context->mmio_base_mac +
			  ((TRIO_SEMAPHORE0 << TRIO_CFG_REGION_ADDR__REG_SHIFT)
			   | (TRIO_CFG_REGION_ADDR__INTFC_VAL_TRIO <<
			      TRIO_CFG_REGION_ADDR__INTFC_SHIFT)), sem.word);

	return 0;
}

/* Actions that can occur when memory map interrupt bits are
 * written via Tile-side MMIO.
 */
typedef enum {
	GXIO_TRIO_MMI_BITS_WRITE = 0,
	GXIO_TRIO_MMI_BITS_W1TC = 1,
	GXIO_TRIO_MMI_BITS_W1TS = 2
} gxio_trio_mmi_write_effect_t;

/* Write the bus-to-tile interrupt state bits associated with a
 * memory map region.
 *
 * @param context An initialized TRIO context.
 * @param map A Memory map region allocated by gxio_trio_alloc_memory_map().
 * @param bits Bits to be written.
 * @param mode Select different memory map interrupt registers as the write
 * target, which can generate different behavious.
 * @return Zero on success, else ::GXIO_TRIO_ERR_BAD_MEMORY_MAP or
 * ::GXIO_ERR_INVAL.
 */
extern int gxio_trio_write_mmi_bits(gxio_trio_context_t *context,
				    unsigned int map, unsigned int bits,
				    gxio_trio_mmi_write_effect_t mode);

#endif /* ! _GXIO_TRIO_H_ */
