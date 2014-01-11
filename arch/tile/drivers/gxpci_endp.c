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
 * Tilera Gx endpoint-side driver
 *
 * This source code is derived from code provided in "Linux Device
 * Drivers, Third Edition", by Jonathan Corbet, Alessandro Rubini, and
 * Greg Kroah-Hartman, published by O'Reilly Media, Inc.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/pci.h>

#include <asm/compat.h>
#include <asm/homecache.h>
#include <asm/tilegxpci.h>

#include <gxio/trio.h>
#include <gxio/iorpc_trio.h>

#include "gxpci_chip_to_chip.h"
#include "gxpci_endp.h"

#include <arch/sim.h>


static const char driver_name[] = DRIVER_NAME_STRING;

/* Information on the PCIe EP ports configuration. */
static int __initdata pcie_ep[TILEGX_NUM_TRIO][TILEGX_TRIO_PCIES];

/* Mask of CPUs that should receive PCIe interrupts. */
struct cpumask intr_cpus_map;

/*
 * The HV doesn't allow the same MMIO ranges to be mapped multiple
 * times. Therefore, all drivers should share a single set of TRIO
 * contexts.
 *
 * This has dependency on the PCIe RC driver which defines and
 * initializes the TRIO contexts, which are declared in <linux/pci.h>.
 */

static struct tlr_pcie_dev *pcie_ep_devs;
static int num_ep_devs;

/*
 * To support different host virtual NIC port configurations for a single
 * TILE PCIe interface, we need to configure the number of host VNIC port and
 * the number of per-port TX/RX queue.
 * The exact number of host VNIC port is specified with a kernel boot argument
 * in the form of "pcie_host_vnic_ports=T,P,N", where T is the TRIO instance
 * number, P is the port number and N is the number of host VNIC port which has
 * a default (maximum) value defined from GXPCI_HOST_NIC_COUNT.
 * The exact number of per-port TX queue is specified with a kernel boot
 * argument in the form of "pcie_host_vnic_tx_queues=T,P,N", where T is the TRIO
 * instance number, P is the port number and N is the number of per-port TX
 * queue which has a default value defined from GXPCI_HOST_NIC_TX_QUEUES.
 * The exact number of per-port RX queue is specified with a kernel boot
 * argument in the form of "pcie_host_vnic_rx_queues=T,P,N", where T is the TRIO
 * instance number, P is the port number and N is the number of per-port RX
 * queues which has a default value defined from GXPCI_HOST_NIC_RX_QUEUES.
 */
int host_vnic_ports[TILEGX_NUM_TRIO][TILEGX_TRIO_PCIES];
static int host_vnic_tx_queues[TILEGX_NUM_TRIO][TILEGX_TRIO_PCIES];
static int host_vnic_rx_queues[TILEGX_NUM_TRIO][TILEGX_TRIO_PCIES];

/*
 * To support different packet queue interface configurations for a single
 * TILE PCIe interface, we need to configure the number of packet queue
 * interfaces in both H2T and T2H directions.
 * The exact number of H2T packet queue is specified with a kernel boot argument
 * in the form of "pcie_host_pq_h2t_queues=T,P,N", where T is the TRIO instance
 * number, P is the port number and N is the number of H2T packet queue
 * interfaces which has a default value defined from GXPCI_HOST_PQ_H2T_COUNT.
 * The exact number of T2H packet queue is specified with a kernel boot argument
 * in the form of "pcie_host_pq_t2h_queues=T,P,N", where T is the TRIO instance
 * number, P is the port number and N is the number of T2H packet queue
 * interfaces which has a default value defined from GXPCI_HOST_PQ_T2H_COUNT.
 */
static int host_pq_h2t_queues[TILEGX_NUM_TRIO][TILEGX_TRIO_PCIES];
static int host_pq_t2h_queues[TILEGX_NUM_TRIO][TILEGX_TRIO_PCIES];

/* Link wait timeout, in seconds. */
static int ep_link_wait = 60;
module_param(ep_link_wait, int, S_IRUGO);
MODULE_PARM_DESC(ep_link_wait, "Time in seconds to wait for the PCIe "
		 "endpoint link to come up.");

/*
 * Bitmap of the PCIe MACs which should have networks associated with them.
 * The bitmap index is (mac_number + 4 * trio_number). For a standard
 * TILExtreme-Gx Duo config, this would be set to 4 on the B nodes only.
 */
static int net_macs;
module_param(net_macs, int, S_IRUGO);
MODULE_PARM_DESC(net_macs, "Mask of MACs to export as network drivers; each "
		 "nybble corresponds to a TRIO shim.");

/* VF host NIC support flag, mutually exclusive with enable_vf_pq. */
static int enable_vf_nic = 1;
module_param(enable_vf_nic, int, S_IRUGO);
MODULE_PARM_DESC(enable_vf_nic, "Enable virtual function's support for "
		 "the host NIC interfaces.");

/* Total waiting time for the reset acknowledgement from the other end. */
#define GXPCI_RELEASE_WAIT_MAX_MSECS		1000

/* VF host Packet Queue support flag, mutually exclusive with enable_vf_nic. */
static int enable_vf_pq = 0;
module_param(enable_vf_pq, int, S_IRUGO);
MODULE_PARM_DESC(enable_vf_pq, "Enable virtual function's support for "
		 "the host Packet Queue interfaces, disabled by default.");

/* Number of host Packet Queue T2H queues in each SR-IOV virtual function. */
static int pq_t2h_queues_in_vf = GXPCI_HOST_PQ_VF_T2H_COUNT;
module_param(pq_t2h_queues_in_vf, int, S_IRUGO);
MODULE_PARM_DESC(pq_t2h_queues_in_vf, "Number of PQ T2H queues per VF.");

/* Number of host Packet Queue H2T queues in each SR-IOV virtual function. */
static int pq_h2t_queues_in_vf = GXPCI_HOST_PQ_VF_H2T_COUNT;
module_param(pq_h2t_queues_in_vf, int, S_IRUGO);
MODULE_PARM_DESC(pq_h2t_queues_in_vf, "Number of PQ H2T queues per VF.");

/*
 * This function creates a PIO mapping to the PCI space.
 * If parameter "vf" is 0, this mapping is for a physical function;
 * otherwise "vf" is the virtual function number plus 1.
 */
static int create_pio_mapping(struct tlr_pcie_dev *ep_dev, void **va,
			      uint64_t bus_addr, uint32_t size, uint32_t vf)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	void *mapped_addr;
	int ret;
	int pio;

	ret = gxio_trio_alloc_pio_regions(trio_context, 1, 0, 0);
	if (ret < 0) {
		pr_err("%s: TRIO %d MAC %d PIO alloc failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, ret);
		return -1;
	}

	pio = ret;

	ret = gxio_trio_init_pio_region_aux(trio_context, pio, ep_dev->mac,
		bus_addr >> 32, vf ? HV_TRIO_FLAG_VFUNC(vf - 1) : 0);
	if (ret < 0) {
		pr_err("%s: TRIO %d MAC %d PIO init failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, ret);
		return -1;
	}

	mapped_addr = iorpc_ioremap(trio_context->fd,
		HV_TRIO_PIO_OFFSET(pio) + (bus_addr & 0xffffffff), size) +
		(bus_addr & (PAGE_SIZE - 1));
	if (mapped_addr == NULL) {
		pr_err("%s: TRIO %d MAC %d PIO mapping failure.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac);
		return -1;
	}

	*va = mapped_addr;

	return pio;
}

/*
 * This function frees a PIO mapping to the PCI space.
 */
static int free_pio_mapping(struct tlr_pcie_dev *ep_dev, void *va,
			    uint32_t pio)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	int ret;

	iounmap(va);

	ret = gxio_trio_free_pio_region_aux(trio_context, pio);
	if (ret < 0) {
		pr_err("%s: TRIO %d MAC %d PIO free failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, ret);
		return -1;
	}

	return 0;
}

/**********************************************************************/
/*                   Interrupt Handler and Worker Thread              */
/**********************************************************************/

static inline void pcie_dma_from_bus(gxio_trio_dma_queue_t *dma_queue,
				     uint64_t bus_addr, void *va, size_t size,
				     int enable_intr)
{
	int64_t slot;

	/* Setup the NTF bit to request DMA completion interrupts. */
	gxio_trio_dma_desc_t desc = {
		{
			.va = (long) va,
			.bsz = 0,
			.c = 0,
			.notif = enable_intr,
			.smod = 0,
			.xsize = size,
			.io_address = bus_addr,
		}
	};

	/* FIXME: Limit to GXPCI_MAX_CMD_SIZE bytes for a DMA transfer. */
	if (unlikely(size >= GXPCI_MAX_CMD_SIZE)) {
		desc.smod = 1;
		desc.xsize = 0;
	}

	slot = gxio_trio_dma_queue_reserve(dma_queue, 1);
	if (likely(slot >= 0)) {
		__insn_mf();
		gxio_trio_dma_queue_put_at(dma_queue, desc, slot);
		gxio_trio_dma_queue_flush(dma_queue);
	} else
		ERR("Empty DMA command slot\n");
}


static inline void pcie_dma_to_bus(gxio_trio_dma_queue_t *dma_queue,
				   uint64_t bus_addr, void *va, size_t size,
				   int enable_intr)
{
	int64_t slot;

	/* Setup the NTF bit to request DMA completion interrupts. */
	gxio_trio_dma_desc_t desc = {
		{
			.va = (long) va,
			.bsz = 0,
			.c = 0,
			.notif = enable_intr,
			.smod = 0,
			.xsize = size,
			.io_address = bus_addr,
		}
	};

	/* FIXME: Limit to GXPCI_MAX_CMD_SIZE bytes for a DMA transfer. */
	if (unlikely(size >= GXPCI_MAX_CMD_SIZE)) {
		desc.smod = 1;
		desc.xsize = 0;
	}

	slot = gxio_trio_dma_queue_reserve(dma_queue, 1);
	if (likely(slot >= 0)) {
		__insn_mf();
		gxio_trio_dma_queue_put_at(dma_queue, desc, slot);
		gxio_trio_dma_queue_flush(dma_queue);
	} else
		ERR("Empty DMA command slot\n");
}


/*
 * Call this function to allocate a free dma_cmd structure, initialize
 * a few fields, and queue it to the pending DMA command list.
 */
static inline dma_cmd_t *gxpci_alloc_dma(struct gxpci_queue_state *queue_state,
					 host_cmd_t *host_cmd,
					 tile_cmd_t *tile_cmd)
{
	dma_cmd_list_t *list = &queue_state->dma_list;

	/* Allocate a new dma_cmd structure. */
	dma_cmd_t *dma_cmd = queue_state->free_dma_cmds;
	if (dma_cmd == NULL)
		ERR("gxpci_alloc_dma() failed\n");

	if (dma_cmd->next_index == DMA_CMD_NULL) {
		queue_state->free_dma_cmds = NULL;
	} else {
		queue_state->free_dma_cmds =
			I2P(queue_state->dma_cmd_array, dma_cmd_t,
			    dma_cmd->next_index);
	}

	/* Zero it out. */
	memset(dma_cmd, 0, sizeof(dma_cmd_t));

	/* Queue to the pending DMA command list. */
	if (list->head) {
		list->tail->next_index =
			P2I(queue_state->dma_cmd_array, dma_cmd_t, dma_cmd);
	} else {
		list->head = dma_cmd;
	}

	list->tail = dma_cmd;
	dma_cmd->next_index = DMA_CMD_NULL;

	/*
	 * Note that these indices will be totally bogus if the cmds are
	 * NULL, but the subsequent code relies on bitfields to know whether
	 * the cmd indices are valid.
	 */
	dma_cmd->host_cmd_index =
		P2I(queue_state->host_cmd_array, host_cmd_t, host_cmd);
	dma_cmd->tile_cmd_index =
		P2I(queue_state->tile_cmd_array, tile_cmd_t, tile_cmd);

	return dma_cmd;
}


/* Update completion count for char streams. */
static inline void gxpci_update_host(struct tlr_stream *stream,
				     gxpci_stream_type_t stream_type)
{
	struct gxpci_queue_state *queue_state;
	struct tlr_pcie_dev *ep_dev = stream->dev;
	gxio_trio_context_t *context = ep_dev->trio;
	gxio_trio_dma_queue_t *dma_queue_data;
	gxio_trio_dma_queue_t *dma_queue_comp;
	uint16_t new_completions, batch_completions;
	unsigned int i, dma_cmds_completed;
	unsigned int index_start, index_end;
	volatile uint16_t hw_cnt;
	int result;

	if (stream_type == GXPCI_CHAR_H2T_STREAM) {
		dma_queue_data = &stream->h2t_dma_resource.pull_dma_queue_data;
		dma_queue_comp = &stream->h2t_dma_resource.push_dma_queue_comp;
		queue_state = &stream->h2t_queue_state;
	} else {
		dma_queue_data = &stream->t2h_dma_resource.push_dma_queue_data;
		dma_queue_comp = &stream->t2h_dma_resource.push_dma_queue_comp;
		queue_state = &stream->t2h_queue_state;
	}

	/* Reset new completion counter. */
	new_completions = 0;

	dma_cmds_completed = queue_state->comp_dmas_completed;
	hw_cnt = gxio_trio_read_dma_queue_complete_count(dma_queue_comp);
	batch_completions = (uint16_t) (hw_cnt -
		(dma_cmds_completed & TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));
	if (batch_completions) {
		index_start = queue_state->comp_dmas_completed;
		index_end = index_start + batch_completions;
		for (i = index_start; i != index_end; i++) {
			new_completions +=
				queue_state->host_cpls_cnt_array[i &
					(PCIE_CMD_QUEUE_ENTRIES - 1)];
		}
		queue_state->comp_dmas_completed += batch_completions;
	}

	/* Update the counter and the MMIO copy. */
	queue_state->host_cpl_consumed += new_completions;

	/* Return directly if there is no completion to update. */
	if (new_completions == 0)
		return;

	gxpci_update_msix_vector(&stream->msix_msg);

	/*
	 * Interrupt the host, if it is not already asserted and it is
	 * enabled.
	 */
	if (stream_type == GXPCI_CHAR_H2T_STREAM) {
		stream->h2t_regs->completion_posted_count =
			queue_state->host_cpl_consumed;

		result = gxio_trio_trigger_host_interrupt(context,
			ep_dev->mac, 0,
			stream->msix_msg.msix_addr,
			stream->msix_msg.msix_data);

		/*
		 * Re-arm the timer to make sure that the endpoint
		 * can trigger MSI/MSI-X interrupt successfully.
		 */
		if (result)
			mod_timer(&stream->intr_timer,
				jiffies + EP_INTR_TIMER_INTERVAL);
	} else {
		stream->t2h_regs->completion_posted_count =
			queue_state->host_cpl_consumed;

		result = gxio_trio_trigger_host_interrupt(context,
			ep_dev->mac, 0,
			stream->msix_msg.msix_addr,
			stream->msix_msg.msix_data);

		/*
		 * Re-arm the timer to make sure that the endpoint
		 * can trigger MSI/MSI-X interrupt successfully.
		 */
		if (result)
			mod_timer(&stream->intr_timer,
				  jiffies + EP_INTR_TIMER_INTERVAL);
	}

	return;
}


/* Send completions for char streams. */
static inline void gxpci_process_comps(struct tlr_stream *stream,
				       gxpci_stream_type_t stream_type)
{
	gxio_trio_dma_queue_t *dma_queue_data;
	gxio_trio_dma_queue_t *dma_queue_comp;
	struct gxpci_queue_state *queue_state;
	pcie_host_completion_t *host_entry;
	pcie_tile_completion_t *send_comps;
	dma_cmd_t *cpl_head;
	dma_cmd_t *complete;
	dma_cmd_t *last_complete;
	host_cmd_t *host_cmd;
	tile_cmd_t *tile_cmd;
	void *tile_addr;
	unsigned int dma_started, dma_cmds_completed;
	volatile uint16_t hw_cnt;
	uint16_t batch_completions;
	uint32_t first_dma, unposted_host_completions, empty_host_cpl_entries;
	uint32_t comp_index, cpl_index, rd_index;
	uint64_t host_addr;

	int enable_intr = 0;

	/*
	 * Check if any data DMAs have been completely processed, and if so,
	 * generate completions.
	 */
	if (stream_type == GXPCI_CHAR_H2T_STREAM) {
		dma_queue_data = &stream->h2t_dma_resource.pull_dma_queue_data;
		dma_queue_comp = &stream->h2t_dma_resource.push_dma_queue_comp;
		queue_state = &stream->h2t_queue_state;
	} else {
		dma_queue_data = &stream->t2h_dma_resource.push_dma_queue_data;
		dma_queue_comp = &stream->t2h_dma_resource.push_dma_queue_comp;
		queue_state = &stream->t2h_queue_state;
	}

	dma_cmds_completed = queue_state->dma_list_consumed;
	hw_cnt = gxio_trio_read_dma_queue_complete_count(dma_queue_data);
	batch_completions = (uint16_t) (hw_cnt -
		(dma_cmds_completed & TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));
	if (batch_completions)
		queue_state->dma_list_consumed += batch_completions;

	cpl_head = queue_state->cpl_list.head;
	complete = cpl_head;
	last_complete = complete;

	while (complete) {
		host_cmd = I2P(queue_state->host_cmd_array, host_cmd_t,
			complete->host_cmd_index);
		tile_cmd = I2P(queue_state->tile_cmd_array, tile_cmd_t,
			complete->tile_cmd_index);
		/*
		 * For dma command that led to data transfer, increment the DMA
		 * counter. Stop handling completions if this DMA is still
		 * pending.
		 */
		if (complete->host_cmd_done && complete->tile_cmd_done) {
			if (batch_completions == 0)
				break;
			/* Count it down. */
			batch_completions--;
		}

		/*
		 * Build host completion. If this is a tile cmd with
		 * complet-with-reset, skip this.
		 */
		if (!complete->tile_cmd_reset && complete->host_cmd_done) {
			cpl_index = queue_state->host_cpl_posted &
				(PCIE_CMD_QUEUE_ENTRIES - 1);
			/* Fill in the host completion dma array entry. */
			host_entry =
			  &queue_state->host_cpl_dma_buffer[cpl_index];
			host_entry->buffer_addr = host_cmd->buffer_addr;
			host_entry->size = complete->size;
			host_entry->reset = complete->host_cmd_reset;
			if (host_entry->reset)
				host_entry->size = 0;

			if (stream_type == GXPCI_CHAR_H2T_STREAM) {
				rd_index = stream->reads_completed;
				stream->read_sizes[rd_index &
					(BUFFERS_PER_STREAM - 1)] =
					host_entry->size;
				wmb();
				stream->reads_completed++;
				wake_up_interruptible(&stream->read_queue);
			} else {
				stream->writes_completed++;
				wake_up_interruptible(&stream->write_queue);
			}

			/* Set the receive overflow bit. */
			if (stream_type == GXPCI_CHAR_T2H_STREAM)
				host_entry->overflow = complete->overflow;

			/*
			 * Return the completed host_cmd_t's to the free list.
			 */
			if (queue_state->free_host_cmds) {
				host_cmd->next_index =
					P2I(queue_state->host_cmd_array,
					    host_cmd_t,
					    queue_state->free_host_cmds);
			} else {
				host_cmd->next_index = HOST_CMD_NULL;
			}
			queue_state->free_host_cmds = host_cmd;
			queue_state->host_cpl_posted++;
		}

		/*
		 * Build tile completion. If this is a host cmd with
		 * complet-with-reset, skip this.
		 */
		if (!complete->host_cmd_reset && complete->tile_cmd_done) {
			/*
			 * This is the start of the tile command completion
			 * array.
			 */
			send_comps = (pcie_tile_completion_t *)
				queue_state->tile_cpl_buffer;
			comp_index = queue_state->tile_cpl_posted &
				(PCIE_CMD_QUEUE_ENTRIES - 1);

			send_comps[comp_index].buffer_addr =
				(uint64_t) tile_cmd->buffer_addr;
			send_comps[comp_index].size = complete->size;
			send_comps[comp_index].reset = complete->tile_cmd_reset;
			if (complete->tile_cmd_reset)
				send_comps[comp_index].size = 0;

			send_comps[comp_index].eop = 1;

			/* Set the receive overflow bit. */
			if (stream_type == GXPCI_CHAR_H2T_STREAM)
				send_comps[comp_index].overflow =
					complete->overflow;

			/*
			 * Return the completed tile_cmd_t's to the free list.
			 */
			if (queue_state->free_tile_cmds) {
				tile_cmd->next_index =
					P2I(queue_state->tile_cmd_array,
					    tile_cmd_t,
					    queue_state->free_tile_cmds);
			} else {
				tile_cmd->next_index = TILE_CMD_NULL;
			}
			queue_state->free_tile_cmds = tile_cmd;
			queue_state->tile_cpl_posted++;
		}

		last_complete = complete;

		if (complete->next_index != DMA_CMD_NULL)
			complete = I2P(queue_state->dma_cmd_array, dma_cmd_t,
				       complete->next_index);
		else {
			complete = NULL;
		}

		/* Return the completed dma_cmd_t's to the free list. */
		if (queue_state->free_dma_cmds) {
			last_complete->next_index =
				P2I(queue_state->dma_cmd_array, dma_cmd_t,
				    queue_state->free_dma_cmds);
		} else {
			last_complete->next_index = DMA_CMD_NULL;
		}
		queue_state->free_dma_cmds = last_complete;
	}

	/* Update cpl_list. */
	queue_state->cpl_list.head = complete;

	unposted_host_completions = queue_state->host_cpl_posted -
		queue_state->host_cpl_dma_started;

	if (stream_type == GXPCI_CHAR_H2T_STREAM) {
		empty_host_cpl_entries = PCIE_CMD_QUEUE_ENTRIES -
			(queue_state->host_cpl_dma_started -
			 stream->h2t_regs->completion_consumed_count);
	} else {
		empty_host_cpl_entries = PCIE_CMD_QUEUE_ENTRIES -
			(queue_state->host_cpl_dma_started -
			 stream->t2h_regs->completion_consumed_count);
	}

	/*
	 * Calculate the number of host completions that tile can send and that
	 * the host can accept.
	 */
	unposted_host_completions =
		MIN(unposted_host_completions, empty_host_cpl_entries);

	/*
	 * DMA the completions, if any, to the host; we may need to split the
	 * completions into three DMA commands if they happen to wrap around
	 * the end of the local and/or remote host completion buffers.
	 */
	while (unposted_host_completions) {
		cpl_index = queue_state->host_cpl_dma_started &
			(PCIE_CMD_QUEUE_ENTRIES - 1);
		first_dma = PCIE_CMD_QUEUE_ENTRIES - cpl_index;
		first_dma = MIN(unposted_host_completions, first_dma);
		first_dma = MIN(first_dma,
				GXPCI_MAX_CMD_SIZE /
				sizeof(struct pcie_host_completion));

		if (unposted_host_completions - first_dma == 0)
			enable_intr = 1;

		host_addr = queue_state->host_cpl_bus_addr +
			sizeof(struct pcie_host_completion) * cpl_index;

		tile_addr =
			(void *) &queue_state->host_cpl_dma_buffer[cpl_index];

		dma_started = queue_state->comp_dmas_started++;

		queue_state->host_cpls_cnt_array[dma_started &
			(PCIE_CMD_QUEUE_ENTRIES - 1)] = first_dma;

		pcie_dma_to_bus(dma_queue_comp, host_addr, tile_addr,
				sizeof(struct pcie_host_completion) *
				first_dma, enable_intr);

		queue_state->host_cpl_dma_started += first_dma;
		unposted_host_completions -= first_dma;
	}
}


/* Process and schedule the next batch of data DMAs for char streams. */
static inline void gxpci_process_host_cmds(
	gxio_trio_dma_queue_t *dma_queue_data,
	struct gxpci_queue_state *queue_state,
	gxpci_stream_type_t stream_type)
{
	dma_cmd_t *message = queue_state->dma_list.head;
	dma_cmd_t *last_msg;
	dma_cmd_t *dma_cmd;
	host_cmd_t *host_cmd;
	tile_cmd_t *tile_cmd;
	void *tile_addr;
	uint64_t host_addr;
	int enable_intr = 0;
	int dma_count;

	if (message) {
		dma_count = 0;
		while (message) {
			if (message->size) {
				/*
				 * Interrupt at the last push DMA cmd in the
				 * batch.
				 */
				last_msg = I2P(queue_state->dma_cmd_array,
					       dma_cmd_t,
					       message->next_index);
				if (message->next_index == DMA_CMD_NULL)
					enable_intr = 1;

				host_cmd = I2P(queue_state->host_cmd_array,
					       host_cmd_t,
					       message->host_cmd_index);
				tile_cmd = I2P(queue_state->tile_cmd_array,
					       tile_cmd_t,
					       message->tile_cmd_index);

				host_addr = host_cmd->buffer_addr;
				tile_addr = tile_cmd->buffer_addr;

				if (stream_type == GXPCI_CHAR_H2T_STREAM) {
					pcie_dma_from_bus(dma_queue_data,
							  host_addr,
							  tile_addr,
							  message->size,
							  enable_intr);
				} else {
					pcie_dma_to_bus(dma_queue_data,
							host_addr,
							tile_addr,
							message->size,
							enable_intr);
				}

				dma_count++;
			}

			/*
			 * We've queued up the DMA, move to the next dma_cmd_t.
			 */
			if (message->next_index != DMA_CMD_NULL) {
				message = I2P(queue_state->dma_cmd_array,
					      dma_cmd_t, message->next_index);
			} else {
				message = NULL;
			}
		}

		/*
		 * Now that the dma_list is processed, queue it to the tail
		 * of next-to-complete DMAs.
		 */
		if (queue_state->cpl_list.head) {
			dma_cmd = queue_state->dma_list.head;
			queue_state->cpl_list.tail->next_index =
				P2I(queue_state->dma_cmd_array, dma_cmd_t,
				    dma_cmd);
		} else {
			queue_state->cpl_list.head = queue_state->dma_list.head;
		}
		queue_state->cpl_list.tail = queue_state->dma_list.tail;
		queue_state->dma_list.consumed += dma_count;
		queue_state->dma_list.head = NULL;
	}
}


/*
 * Check for and process any new host-side CS commands that have been DMA'ed
 * from the host. This function is responsible for taking host buffer_cmds and
 * entering them into the host_cmd_t data structures.
 */
static inline void gxpci_parse_host_cmds(struct gxpci_queue_state *queue_state,
					 gxpci_stream_type_t stream_type)
{
	uint32_t posted, consumed, host_index;
	uint32_t send_size, recv_size;
	uint16_t host_cmd_index;
	struct pcie_host_buffer_cmd *cmd;
	host_cmd_t *host_cmd;
	host_cmd_t *tail_cmd;
	tile_cmd_t *tile_cmd;
	dma_cmd_t *dma_cmd;
	pending_cmd_list_t *list;

	posted = queue_state->host_cmds_posted;
	consumed = queue_state->host_cmds_consumed;

	for (; consumed != posted; consumed++) {

		host_index = consumed & (PCIE_CMD_QUEUE_ENTRIES - 1);

		cmd = &queue_state->host_cmd_dma_buffer[host_index];

		/* Get a free host_cmd_t structure from the free list. */
		host_cmd = queue_state->free_host_cmds;
		if (host_cmd == NULL)
			ERR("NULL host_cmd\n");

		if (host_cmd->next_index == HOST_CMD_NULL) {
			queue_state->free_host_cmds = NULL;
		} else {
			queue_state->free_host_cmds =
				I2P(queue_state->host_cmd_array, host_cmd_t,
				    host_cmd->next_index);
		}

		host_cmd->buffer_addr = cmd->buffer_addr;
		host_cmd->size = cmd->size;

		host_cmd_index = P2I(queue_state->host_cmd_array, host_cmd_t,
				     host_cmd) + 1;

		list = &queue_state->unmatched_cmds;

		if (list->head == 0) {
			/*
			 * The list is empty; allocate an item and set the list
			 * flag as containing unmatched host commands.
			 */
			TRACE("Host: new list\n");

			list->head = host_cmd_index;
			list->tail = host_cmd_index;
			list->is_host_list = 1;
			host_cmd->next_index = HOST_CMD_NULL;
		} else if (!list->is_host_list) {
			/*
			 * The list contains unmatched tile commands; we'll
			 * match the first one so pop it off and put it on the
			 * dma_cmds list.
			 */
			TRACE("Host: match\n");

			tile_cmd = I2P(queue_state->tile_cmd_array, tile_cmd_t,
				       (list->head - 1));

			/* Set the transfer size for the new command. */
			if (stream_type == GXPCI_CHAR_H2T_STREAM) {
				send_size = host_cmd->size;
				recv_size = tile_cmd->size;
			} else {
				send_size = tile_cmd->size;
				recv_size = host_cmd->size;
			}

			/* Allocate a new dma_cmd structure and queue it up. */
			dma_cmd = gxpci_alloc_dma(queue_state, host_cmd,
						  tile_cmd);

			dma_cmd->size = MIN(send_size, recv_size);

			if (recv_size < send_size)
				dma_cmd->overflow = 1;

			dma_cmd->host_cmd_done = 1;
			dma_cmd->tile_cmd_done = 1;

			/* Dequeue this tile cmd from the unmatched list. */
			if (tile_cmd->next_index == TILE_CMD_NULL) {
				list->head = 0;
				list->tail = 0;
			} else {
				list->head = tile_cmd->next_index;
			}
		} else {
			/*
			 * The list contains unmatched host commands;
			 * add ourselves to the tail of the list.
			 */
			TRACE("Host: add to tail\n");

			tail_cmd = I2P(queue_state->host_cmd_array, host_cmd_t,
				(list->tail - 1));
			tail_cmd->next_index = host_cmd_index;
			list->tail = host_cmd_index;
			host_cmd->next_index = HOST_CMD_NULL;
		}
	}
	queue_state->host_cmds_consumed = consumed;
}


/* Fetch host-side commands for char streams. */
static inline void gxpci_fetch_host_cmds(struct tlr_stream *stream,
					 gxpci_stream_type_t stream_type,
					 int enable_intr)
{
	struct gxpci_queue_state *queue_state;
	gxio_trio_dma_queue_t *dma_queue_desc;
	uint32_t host_cmds_to_fetch;
	uint32_t empty_host_cmd_buffers;
	uint32_t first_dma;
	uint32_t host_cmd_posted;
	uint32_t cmd_index;
	uint32_t dmas_started;
	uint64_t bus_addr;
	uint64_t offset;
	int last_intr = 0;

	if (stream_type == GXPCI_CHAR_H2T_STREAM) {
		queue_state = &stream->h2t_queue_state;
		dma_queue_desc = &stream->h2t_dma_resource.pull_dma_queue_desc;
		host_cmd_posted = stream->h2t_regs->buffer_cmd_posted_count;
	} else {
		queue_state = &stream->t2h_queue_state;
		dma_queue_desc = &stream->t2h_dma_resource.pull_dma_queue_desc;
		host_cmd_posted = stream->t2h_regs->buffer_cmd_posted_count;
	}

	/* See if we need to DMA in more host commands. */
	host_cmds_to_fetch =
		host_cmd_posted - queue_state->host_cmds_dma_started;

	if (host_cmds_to_fetch) {
		empty_host_cmd_buffers = PCIE_CMD_QUEUE_ENTRIES -
			(queue_state->host_cmds_dma_started -
			 queue_state->host_cmds_consumed);
		host_cmds_to_fetch =
			MIN(host_cmds_to_fetch, empty_host_cmd_buffers);

		while (host_cmds_to_fetch) {
			cmd_index = queue_state->host_cmds_dma_started &
				(PCIE_CMD_QUEUE_ENTRIES - 1);
			first_dma = PCIE_CMD_QUEUE_ENTRIES - cmd_index;
			first_dma = MIN(host_cmds_to_fetch, first_dma);
			first_dma = MIN(first_dma, GXPCI_MAX_CMD_SIZE /
					sizeof(struct pcie_host_buffer_cmd));

			if (host_cmds_to_fetch - first_dma == 0)
				last_intr = 1;

			bus_addr = queue_state->host_cmd_bus_addr;
			offset =
				sizeof(struct pcie_host_buffer_cmd) * cmd_index;
			bus_addr += offset;

			dmas_started = queue_state->fetch_dmas_started &
				(PCIE_CMD_QUEUE_ENTRIES - 1);
			queue_state->host_cmds_cnt_array[dmas_started] =
				first_dma;

			pcie_dma_from_bus(dma_queue_desc, bus_addr,
			  (void *) queue_state->host_cmd_dma_buffer +
			  offset, first_dma *
			  sizeof(struct pcie_host_buffer_cmd),
			  (last_intr & enable_intr));

			queue_state->host_cmds_dma_started += first_dma;
			queue_state->fetch_dmas_started++;
			host_cmds_to_fetch -= first_dma;
		}
	}
}


/*
 * Check for and process any new tile-side CS commands. This function is
 * responsible for taking tile buffer_cmds and entering them into the
 * tile_cmd_t data structures.
 */
static inline void gxpci_check_tile_cmds(struct gxpci_queue_state *queue_state,
					 gxpci_stream_type_t stream_type)
{
	uint32_t posted, consumed, tile_index;
	uint32_t send_size, recv_size;
	uint16_t tile_cmd_index;
	struct pcie_tile_buffer_cmd *cmd;
	tile_cmd_t *tile_cmd;
	tile_cmd_t *tail_cmd;
	host_cmd_t *host_cmd;
	dma_cmd_t *dma_cmd;
	pending_cmd_list_t *list;

	posted = queue_state->tile_cmds_posted;
	consumed = queue_state->tile_cmds_consumed;

	for (; consumed != posted; consumed++) {
		tile_index = consumed & (PCIE_CMD_QUEUE_ENTRIES - 1);
		cmd = &queue_state->tile_cmd_buffer[tile_index];

		/* Get a free tile_cmd_t structure from the free list. */
		tile_cmd = queue_state->free_tile_cmds;
		if (tile_cmd == NULL)
			ERR("NULL tile_cmd\n");

		if (tile_cmd->next_index == TILE_CMD_NULL) {
			queue_state->free_tile_cmds = NULL;
		} else {
			queue_state->free_tile_cmds =
				I2P(queue_state->tile_cmd_array, tile_cmd_t,
				    tile_cmd->next_index);
		}

		tile_cmd_index = P2I(queue_state->tile_cmd_array, tile_cmd_t,
				     tile_cmd) + 1;

		tile_cmd->buffer_addr = (void *) cmd->buffer_addr;
		tile_cmd->size = cmd->size;

		list = &queue_state->unmatched_cmds;

		if (list->head == 0) {
			/*
			 * The list is empty; allocate an item and set the list
			 * flag as containing unmatched tile commands.
			 */
			TRACE("Tile: new list\n");

			list->head = tile_cmd_index;
			list->tail = tile_cmd_index;
			list->is_host_list = 0;
			tile_cmd->next_index = TILE_CMD_NULL;
		} else if (list->is_host_list) {
			/*
			 * The list contains unmatched host commands; we'll
			 * match the first one so pop it off and put it on the
			 * dma_cmds list.
			 */
			TRACE("Tile: match\n");

			host_cmd = I2P(queue_state->host_cmd_array, host_cmd_t,
				       (list->head - 1));

			/* Set the transfer size for the new command. */
			if (stream_type == GXPCI_CHAR_H2T_STREAM) {
				send_size = host_cmd->size;
				recv_size = tile_cmd->size;
			} else {
				send_size = tile_cmd->size;
				recv_size = host_cmd->size;
			}

			/* Allocate a new dma_cmd structure and queue it up. */
			dma_cmd = gxpci_alloc_dma(queue_state, host_cmd,
						  tile_cmd);

			dma_cmd->size = MIN(send_size, recv_size);

			if (recv_size < send_size)
				dma_cmd->overflow = 1;

			dma_cmd->host_cmd_done = 1;
			dma_cmd->tile_cmd_done = 1;

			/* Dequeue this host cmd from the unmatched list. */
			if (host_cmd->next_index == HOST_CMD_NULL) {
				list->head = 0;
				list->tail = 0;
			} else {
				list->head = host_cmd->next_index;
			}
		} else {
			/*
			 * The list contains unmatched tile commands;
			 * add ourselves to the tail of the list.
			 */
			TRACE("Tile: add to tail\n");

			tail_cmd = I2P(queue_state->tile_cmd_array, tile_cmd_t,
				       (list->tail - 1));
			tail_cmd->next_index = tile_cmd_index;
			list->tail = tile_cmd_index;
			tile_cmd->next_index = TILE_CMD_NULL;
		}
	}

	queue_state->tile_cmds_consumed = consumed;
}


#if GXPCI_HOST_ZC_QUEUE_COUNT
/* Update completion count for zero-copy queues. */
void gxpci_zc_update_host(struct tlr_zc_stream *stream)
{
	struct tlr_pcie_dev *ep_dev = stream->tlr;
	struct gxpci_queue_state *queue_state;
	struct gxpci_queue_state *shared_queue_state;
	struct tlr_zc_stream *shared_stream;
	gxio_trio_context_t *context = ep_dev->trio;
	gxio_trio_dma_queue_t *dma_queue_data;
	gxio_trio_dma_queue_t *dma_queue_comp;
	uint16_t batch_completions;
	unsigned int i, dma_cmds_completed;
	unsigned int index, index_start, index_end;
	volatile uint16_t hw_cnt;

	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
		dma_queue_data = &stream->h2t_dma_resource->pull_dma_queue_data;
		dma_queue_comp = &stream->h2t_dma_resource->push_dma_queue_comp;
		queue_state = &stream->h2t_queue_state;
	} else {
		dma_queue_data = &stream->t2h_dma_resource->push_dma_queue_data;
		dma_queue_comp = &stream->t2h_dma_resource->push_dma_queue_comp;
		queue_state = &stream->t2h_queue_state;
	}

	hw_cnt = gxio_trio_read_dma_queue_complete_count(dma_queue_comp);
	batch_completions = (uint16_t) (hw_cnt -
		(stream->trio_state->comp_dma_chan_dequeued &
		 TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));

	/* Dispatch all the push DMA completions. */
	index_start = stream->trio_state->comp_dma_chan_dequeued;
	index_end = index_start + batch_completions;
	for (i = index_start; i != index_end; i++) {
		index = stream->trio_state->comp_dma_chans[i &
			(GXPCI_STREAM_PULL_DMA_RING_LEN - 1)];

		if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
			shared_stream = tlr_get_zc_h2t_stream(ep_dev, index);
			shared_queue_state = &shared_stream->h2t_queue_state;
		} else {
			shared_stream = tlr_get_zc_t2h_stream(ep_dev, index);
			shared_queue_state = &shared_stream->t2h_queue_state;
		}

		/* Update the number of completed data DMA. */
		dma_cmds_completed = shared_queue_state->comp_dmas_completed;
		shared_queue_state->comp_completions +=
			shared_queue_state->host_cpls_cnt_array[
				dma_cmds_completed &
					(PCIE_CMD_QUEUE_ENTRIES - 1)];
		shared_queue_state->host_cpl_consumed +=
			shared_queue_state->host_cpls_cnt_array[
				dma_cmds_completed &
					(PCIE_CMD_QUEUE_ENTRIES - 1)];
		shared_queue_state->comp_dmas_completed++;
	}
	stream->trio_state->comp_dma_chan_dequeued += batch_completions;

	/* Return directly if there is no completion to update. */
	if (queue_state->comp_completions == 0)
		return;
	queue_state->comp_completions = 0;

	/*
	 * Interrupt the host, if it is not already asserted and it is
	 * enabled.
	 */
	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
		stream->h2t_regs->completion_posted_count =
			queue_state->host_cpl_consumed;

		gxpci_update_msix_vector(&stream->h2t_msix_msg);

		/* Send the interrupt if the host can accept it. */
		if (stream->h2t_regs->interrupt_enable)
			gxio_trio_trigger_host_interrupt(context,
				ep_dev->mac, 0,
				stream->h2t_msix_msg.msix_addr,
				stream->h2t_msix_msg.msix_data);

		/*
		 * Re-arm the per queue timer to monitor any pending
		 * interrupts to be sent.
		 */
		mod_timer(&stream->intr_timer,
			jiffies + EP_INTR_TIMER_INTERVAL);
	} else {
		stream->t2h_regs->completion_posted_count =
			queue_state->host_cpl_consumed;

		gxpci_update_msix_vector(&stream->t2h_msix_msg);

		/* Send the interrupt if the host can accept it. */
		if (stream->t2h_regs->interrupt_enable)
			gxio_trio_trigger_host_interrupt(context,
				ep_dev->mac, 0,
				stream->t2h_msix_msg.msix_addr,
				stream->t2h_msix_msg.msix_data);

		/*
		 * Re-arm the per queue timer to monitor any pending
		 * interrupts to be sent.
		 */
		mod_timer(&stream->intr_timer,
			  jiffies + EP_INTR_TIMER_INTERVAL);
	}

	return;
}


/* Send completions for zero-copy queues. */
static void gxpci_zc_process_comps(struct tlr_zc_stream *stream)
{
	gxio_trio_dma_queue_t *dma_queue_data;
	gxio_trio_dma_queue_t *dma_queue_comp;
	struct tlr_pcie_dev *tlr;
	struct tlr_zc_stream *shared_stream;
	struct gxpci_queue_state *queue_state;
	struct gxpci_queue_state *shared_queue_state;
	pcie_host_completion_t *host_entry;
	pcie_tile_completion_t *send_comps;
	dma_cmd_t *cpl_head;
	dma_cmd_t *complete;
	dma_cmd_t *last_complete;
	host_cmd_t *host_cmd;
	tile_cmd_t *tile_cmd;
	void *tile_addr;
	unsigned int i, index, index_start, index_end, dma_started;
	volatile uint16_t hw_cnt;
	uint16_t batch_completions;
	uint32_t first_dma, unposted_host_completions, empty_host_cpl_entries;
	uint32_t comp_index, cpl_index;
	uint64_t host_addr;

	int enable_intr = 0;
	first_dma = 0;

	/*
	 * Check if any data DMAs have been completely processed, and if so,
	 * generate completions.
	 */
	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
		dma_queue_data = &stream->h2t_dma_resource->pull_dma_queue_data;
		dma_queue_comp = &stream->h2t_dma_resource->push_dma_queue_comp;
		queue_state = &stream->h2t_queue_state;
	} else {
		dma_queue_data = &stream->t2h_dma_resource->push_dma_queue_data;
		dma_queue_comp = &stream->t2h_dma_resource->push_dma_queue_comp;
		queue_state = &stream->t2h_queue_state;
	}

	tlr = stream->tlr;
	hw_cnt = gxio_trio_read_dma_queue_complete_count(dma_queue_data);
	batch_completions = (uint16_t) (hw_cnt -
		(stream->trio_state->data_dma_chan_dequeued &
		 TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));

	/*
	 * Dispatch all the DMA completions for TRIO-shared ZC queues.
	 */
	index_start = stream->trio_state->data_dma_chan_dequeued;
	index_end = index_start + batch_completions;
	for (i = index_start; i != index_end; i++) {
		index = stream->trio_state->data_dma_chans[i &
			(GXPCI_STREAM_PULL_DMA_RING_LEN - 1)];

		if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
			shared_stream = tlr_get_zc_h2t_stream(tlr, index);
			shared_queue_state = &shared_stream->h2t_queue_state;
		} else {
			shared_stream = tlr_get_zc_t2h_stream(tlr, index);
			shared_queue_state = &shared_stream->t2h_queue_state;
		}

		/* Update the number of completed pull DMA. */
		shared_queue_state->data_completions++;
		shared_queue_state->dma_list_consumed++;
	}
	stream->trio_state->data_dma_chan_dequeued += batch_completions;

	cpl_head = queue_state->cpl_list.head;
	complete = cpl_head;
	last_complete = complete;

	while (complete) {
		host_cmd = I2P(queue_state->host_cmd_array, host_cmd_t,
			complete->host_cmd_index);
		tile_cmd = I2P(queue_state->tile_cmd_array, tile_cmd_t,
			complete->tile_cmd_index);
		/*
		 * For dma command that led to data transfer, increment the DMA
		 * counter. Stop handling completions if this DMA is still
		 * pending.
		 */
		if (complete->host_cmd_done && complete->tile_cmd_done) {
			if (queue_state->data_completions == 0)
				break;
			/* Count it down. */
			queue_state->data_completions--;
		}

		/*
		 * Build host completion. If this is a tile cmd with
		 * complet-with-reset, skip this.
		 */
		if (!complete->tile_cmd_reset && complete->host_cmd_done) {
			cpl_index = queue_state->host_cpl_posted &
				(PCIE_CMD_QUEUE_ENTRIES - 1);
			/* Fill in the host completion dma array entry. */
			host_entry =
			  &queue_state->host_cpl_dma_buffer[cpl_index];
			host_entry->buffer_addr = host_cmd->buffer_addr;
			host_entry->size = complete->size;
			host_entry->reset = complete->host_cmd_reset;
			if (host_entry->reset)
				host_entry->size = 0;

			/* Set the receive overflow bit. */
			if (stream->cmd_queue->type == TLR_ZC_CMD_T2H)
				host_entry->overflow = complete->overflow;

			/*
			 * Return the completed host_cmd_t's to the free list.
			 */
			if (queue_state->free_host_cmds) {
				host_cmd->next_index =
					P2I(queue_state->host_cmd_array,
					    host_cmd_t,
					    queue_state->free_host_cmds);
			} else {
				host_cmd->next_index = HOST_CMD_NULL;
			}
			queue_state->free_host_cmds = host_cmd;
			queue_state->host_cpl_posted++;

			if (stream->cmd_queue->type == TLR_ZC_CMD_T2H)
				host_entry->tag = tile_cmd->tag;
			else
				host_entry->tag = host_cmd->tag;
		}

		/*
		 * Build tile completion. If this is a host cmd with
		 * complet-with-reset, skip this.
		 */
		if (!complete->host_cmd_reset && complete->tile_cmd_done) {
			/*
			 * This is the start of the tile command completion
			 * array.
			 */
			send_comps = (pcie_tile_completion_t *)
				queue_state->tile_cpl_buffer;
			comp_index = queue_state->tile_cpl_posted &
				(PCIE_CMD_QUEUE_ENTRIES - 1);

			send_comps[comp_index].buffer_addr =
				(uint64_t) tile_cmd->buffer_addr;
			send_comps[comp_index].size = complete->size;
			send_comps[comp_index].reset = complete->tile_cmd_reset;
			if (complete->tile_cmd_reset)
				send_comps[comp_index].size = 0;
			send_comps[comp_index].eop = 1;

			/* Set the receive overflow bit. */
			if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
				send_comps[comp_index].overflow =
					complete->overflow;

			/*
			 * Return the completed tile_cmd_t's to the free list.
			 */
			if (queue_state->free_tile_cmds) {
				tile_cmd->next_index =
					P2I(queue_state->tile_cmd_array,
					    tile_cmd_t,
					    queue_state->free_tile_cmds);
			} else {
				tile_cmd->next_index = TILE_CMD_NULL;
			}
			queue_state->free_tile_cmds = tile_cmd;
			queue_state->tile_cpl_posted++;

			if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
				send_comps[comp_index].tag = host_cmd->tag;
			else
				send_comps[comp_index].tag = tile_cmd->tag;

			tlr_zc_cmd_q_comp(stream->cmd_queue,
					  &send_comps[comp_index],
					  complete->size);
		}

		last_complete = complete;

		if (complete->next_index != DMA_CMD_NULL)
			complete = I2P(queue_state->dma_cmd_array, dma_cmd_t,
				       complete->next_index);
		else {
			complete = NULL;
		}

		/* Return the completed dma_cmd_t's to the free list. */
		if (queue_state->free_dma_cmds) {
			last_complete->next_index =
				P2I(queue_state->dma_cmd_array, dma_cmd_t,
				    queue_state->free_dma_cmds);
		} else {
			last_complete->next_index = DMA_CMD_NULL;
		}
		queue_state->free_dma_cmds = last_complete;
	}

	/* Update cpl_list. */
	queue_state->cpl_list.head = complete;

	unposted_host_completions = queue_state->host_cpl_posted -
		queue_state->host_cpl_dma_started;

	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
		empty_host_cpl_entries = PCIE_CMD_QUEUE_ENTRIES -
			(queue_state->host_cpl_dma_started -
			 stream->h2t_regs->completion_consumed_count);
	} else {
		empty_host_cpl_entries = PCIE_CMD_QUEUE_ENTRIES -
			(queue_state->host_cpl_dma_started -
			 stream->t2h_regs->completion_consumed_count);
	}

	/*
	 * Calculate the number of host completions that tile can send and that
	 * the host can accept.
	 */
	unposted_host_completions =
		MIN(unposted_host_completions, empty_host_cpl_entries);

	/*
	 * DMA the completions, if any, to the host; we may need to split the
	 * completions into three DMA commands if they happen to wrap around
	 * the end of the local and/or remote host completion buffers.
	 */
	while (unposted_host_completions) {
		cpl_index = queue_state->host_cpl_dma_started &
			(PCIE_CMD_QUEUE_ENTRIES - 1);
		first_dma = PCIE_CMD_QUEUE_ENTRIES - cpl_index;
		first_dma = MIN(unposted_host_completions, first_dma);
		first_dma = MIN(first_dma, GXPCI_MAX_CMD_SIZE /
				sizeof(struct pcie_host_completion));

		if (unposted_host_completions - first_dma == 0)
			enable_intr = 1;

		host_addr = queue_state->host_cpl_bus_addr +
			sizeof(struct pcie_host_completion) * cpl_index;

		tile_addr =
			(void *) &queue_state->host_cpl_dma_buffer[cpl_index];

		dma_started = queue_state->comp_dmas_started++;

		queue_state->host_cpls_cnt_array[dma_started &
			(PCIE_CMD_QUEUE_ENTRIES - 1)] = first_dma;

		pcie_dma_to_bus(dma_queue_comp, host_addr, tile_addr,
				sizeof(struct pcie_host_completion) *
				first_dma, enable_intr);

		index = stream->trio_state->comp_dma_chan_enqueued++;
		index &= (GXPCI_STREAM_PULL_DMA_RING_LEN - 1);
		stream->trio_state->comp_dma_chans[index] =
			stream->index;

		queue_state->host_cpl_dma_started += first_dma;
		unposted_host_completions -= first_dma;
	}
}


/* Process and schedule the next batch of data DMAs for zero-copy queues. */
void gxpci_zc_process_host_cmds(struct tlr_zc_stream *stream)
{
	gxio_trio_dma_queue_t *dma_queue_data;
	struct gxpci_queue_state *queue_state;
	dma_cmd_t *last_msg;
	dma_cmd_t *message;
	dma_cmd_t *dma_cmd;
	host_cmd_t *host_cmd;
	tile_cmd_t *tile_cmd;
	void *tile_addr;
	uint64_t host_addr;
	int enable_intr = 0;
	int dma_count;
	int index;

	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
		dma_queue_data = &stream->h2t_dma_resource->pull_dma_queue_data;
		queue_state = &stream->h2t_queue_state;
	} else {
		dma_queue_data = &stream->t2h_dma_resource->push_dma_queue_data;
		queue_state = &stream->t2h_queue_state;
	}

	message = queue_state->dma_list.head;
	if (message) {
		dma_count = 0;
		while (message) {
			if (message->size) {
				/*
				 * Interrupt at the last push DMA cmd in the
				 * batch.
				 */
				last_msg = I2P(queue_state->dma_cmd_array,
					       dma_cmd_t,
					       message->next_index);
				if (message->next_index == DMA_CMD_NULL)
					enable_intr = 1;

				host_cmd = I2P(queue_state->host_cmd_array,
					       host_cmd_t,
					       message->host_cmd_index);
				tile_cmd = I2P(queue_state->tile_cmd_array,
					       tile_cmd_t,
					       message->tile_cmd_index);

				host_addr = host_cmd->buffer_addr;
				tile_addr = tile_cmd->buffer_addr;

				if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
					pcie_dma_from_bus(dma_queue_data,
							  host_addr,
							  tile_addr,
							  message->size,
							  enable_intr);
				else
					pcie_dma_to_bus(dma_queue_data,
							host_addr,
							tile_addr,
							message->size,
							enable_intr);

				index = stream->trio_state->
					data_dma_chan_enqueued++;
				index &= (GXPCI_STREAM_PULL_DMA_RING_LEN - 1);
				stream->trio_state->data_dma_chans[index] =
					stream->index;
				dma_count++;
			}

			/*
			 * We've queued up the DMA, move to the next dma_cmd_t.
			 */
			if (message->next_index != DMA_CMD_NULL) {
				message = I2P(queue_state->dma_cmd_array,
					      dma_cmd_t, message->next_index);
			} else {
				message = NULL;
			}
		}

		/*
		 * Now that the dma_list is processed, queue it to the tail
		 * of next-to-complete DMAs.
		 */
		if (queue_state->cpl_list.head) {
			dma_cmd = queue_state->dma_list.head;
			queue_state->cpl_list.tail->next_index =
				P2I(queue_state->dma_cmd_array, dma_cmd_t,
				    dma_cmd);
		} else {
			queue_state->cpl_list.head = queue_state->dma_list.head;
		}
		queue_state->cpl_list.tail = queue_state->dma_list.tail;
		queue_state->dma_list.consumed += dma_count;
		queue_state->dma_list.head = NULL;
	}
}


/*
 * Check for and process any new host-side ZC commands that have been DMA'ed
 * from the host. This function is responsible for taking host buffer_cmds and
 * entering them into the host_cmd_t data structures.
 */
void gxpci_zc_parse_host_cmds(struct tlr_zc_stream *stream)
{
	uint32_t posted, consumed, host_index;
	uint32_t send_size, recv_size;
	uint16_t host_cmd_index;
	struct gxpci_queue_state *queue_state;
	struct pcie_host_buffer_cmd *cmd;
	pending_cmd_list_t *list;
	host_cmd_t *host_cmd;
	host_cmd_t *tail_cmd;
	tile_cmd_t *tile_cmd;
	dma_cmd_t *dma_cmd;

	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
		queue_state = &stream->h2t_queue_state;
	else
		queue_state = &stream->t2h_queue_state;

	posted = queue_state->host_cmds_posted;
	consumed = queue_state->host_cmds_consumed;

	for (; consumed != posted; consumed++) {

		host_index = consumed & (PCIE_CMD_QUEUE_ENTRIES - 1);

		cmd = &queue_state->host_cmd_dma_buffer[host_index];

		/* Get a free host_cmd_t structure from the free list. */
		host_cmd = queue_state->free_host_cmds;
		if (host_cmd == NULL)
			ERR("NULL host_cmd\n");

		if (host_cmd->next_index == HOST_CMD_NULL) {
			queue_state->free_host_cmds = NULL;
		} else {
			queue_state->free_host_cmds =
				I2P(queue_state->host_cmd_array, host_cmd_t,
				    host_cmd->next_index);
		}

		host_cmd->buffer_addr = cmd->buffer_addr;
		host_cmd->size = cmd->size;

		if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
			host_cmd->tag = cmd->tag;

		host_cmd_index = P2I(queue_state->host_cmd_array, host_cmd_t,
				     host_cmd) + 1;

		list = &queue_state->unmatched_cmds;

		if (list->head == 0) {
			/*
			 * The list is empty; allocate an item and set the list
			 * flag as containing unmatched host commands.
			 */
			TRACE("Host: new list\n");

			list->head = host_cmd_index;
			list->tail = host_cmd_index;
			list->is_host_list = 1;
			host_cmd->next_index = HOST_CMD_NULL;
		} else if (!list->is_host_list) {
			/*
			 * The list contains unmatched tile commands; we'll
			 * match the first one so pop it off and put it on the
			 * dma_cmds list.
			 */
			TRACE("Host: match\n");

			tile_cmd = I2P(queue_state->tile_cmd_array, tile_cmd_t,
				       (list->head - 1));

			/* Set the transfer size for the new command. */
			if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
				send_size = host_cmd->size;
				recv_size = tile_cmd->size;
			} else {
				send_size = tile_cmd->size;
				recv_size = host_cmd->size;
			}

			/* Allocate a new dma_cmd structure and queue it up. */
			dma_cmd = gxpci_alloc_dma(queue_state, host_cmd,
						  tile_cmd);

			dma_cmd->size = MIN(send_size, recv_size);

			if (recv_size < send_size)
				dma_cmd->overflow = 1;

			dma_cmd->host_cmd_done = 1;
			dma_cmd->tile_cmd_done = 1;

			/* Dequeue this tile cmd from the unmatched list. */
			if (tile_cmd->next_index == TILE_CMD_NULL) {
				list->head = 0;
				list->tail = 0;
			} else {
				list->head = tile_cmd->next_index;
			}
		} else {
			/*
			 * The list contains unmatched host commands;
			 * add ourselves to the tail of the list.
			 */
			TRACE("Host: add to tail\n");

			tail_cmd = I2P(queue_state->host_cmd_array, host_cmd_t,
				       (list->tail - 1));
			tail_cmd->next_index = host_cmd_index;
			list->tail = host_cmd_index;
			host_cmd->next_index = HOST_CMD_NULL;
		}
	}
	queue_state->host_cmds_consumed = consumed;
}


/* Fetch host-side commands for zero-copy queues. */
static inline void gxpci_zc_fetch_host_cmds(struct tlr_zc_stream *stream,
					    int enable_intr)
{
	struct gxpci_queue_state *queue_state;
	gxio_trio_dma_queue_t *dma_queue_desc;
	uint32_t host_cmds_to_fetch, empty_host_cmd_buffers;
	uint32_t first_dma, host_cmd_posted;
	uint32_t cmd_index, dmas_started;
	uint64_t bus_addr, offset;
	int index;
	int last_intr = 0;

	first_dma = 0;

	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
		queue_state = &stream->h2t_queue_state;
		dma_queue_desc = &stream->h2t_dma_resource->pull_dma_queue_desc;
		host_cmd_posted = stream->h2t_regs->buffer_cmd_posted_count;
	} else {
		queue_state = &stream->t2h_queue_state;
		dma_queue_desc = &stream->t2h_dma_resource->pull_dma_queue_desc;
		host_cmd_posted = stream->t2h_regs->buffer_cmd_posted_count;
	}

	/* See if we need to DMA in more host commands. */
	host_cmds_to_fetch =
		host_cmd_posted - queue_state->host_cmds_dma_started;

	if (host_cmds_to_fetch) {
		empty_host_cmd_buffers = PCIE_CMD_QUEUE_ENTRIES -
			(queue_state->host_cmds_dma_started -
			 queue_state->host_cmds_consumed);
		host_cmds_to_fetch =
			MIN(host_cmds_to_fetch, empty_host_cmd_buffers);

		while (host_cmds_to_fetch) {
			cmd_index = queue_state->host_cmds_dma_started &
				(PCIE_CMD_QUEUE_ENTRIES - 1);
			first_dma = PCIE_CMD_QUEUE_ENTRIES - cmd_index;
			first_dma = MIN(host_cmds_to_fetch, first_dma);
			first_dma = MIN(first_dma, GXPCI_MAX_CMD_SIZE /
					sizeof(struct pcie_host_buffer_cmd));

			if (host_cmds_to_fetch - first_dma == 0)
				last_intr = 1;

			bus_addr = queue_state->host_cmd_bus_addr;
			offset =
				sizeof(struct pcie_host_buffer_cmd) * cmd_index;
			bus_addr += offset;

			dmas_started = queue_state->fetch_dmas_started &
				(PCIE_CMD_QUEUE_ENTRIES - 1);
			queue_state->host_cmds_cnt_array[dmas_started] =
				first_dma;

			pcie_dma_from_bus(dma_queue_desc, bus_addr,
			  (void *) queue_state->host_cmd_dma_buffer +
			  offset, first_dma *
			  sizeof(struct pcie_host_buffer_cmd),
			  (last_intr & enable_intr));

			/* Record the ZC queue index for the DMA command. */
			index = stream->trio_state->cmd_dma_chan_enqueued++;
			index &= (GXPCI_STREAM_PULL_DMA_RING_LEN - 1);
			stream->trio_state->cmd_dma_chans[index] =
				stream->index;

			queue_state->host_cmds_dma_started += first_dma;
			queue_state->fetch_dmas_started++;
			host_cmds_to_fetch -= first_dma;
		}
	}
}


/*
 * Check for and process any new tile-side ZC commands. This function is
 * responsible for taking tile buffer_cmds and entering them into the
 * tile_cmd_t data structures.
 */
void gxpci_zc_check_tile_cmds(struct tlr_zc_stream *stream)
{
	uint32_t posted, consumed, tile_index;
	uint32_t send_size, recv_size;
	uint16_t tile_cmd_index;
	struct gxpci_queue_state *queue_state;
	struct pcie_tile_buffer_cmd *cmd;
	pending_cmd_list_t *list;
	tile_cmd_t *tile_cmd;
	tile_cmd_t *tail_cmd;
	host_cmd_t *host_cmd;
	dma_cmd_t *dma_cmd;

	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
		queue_state = &stream->h2t_queue_state;
	else
		queue_state = &stream->t2h_queue_state;

	posted = queue_state->tile_cmds_posted;
	consumed = queue_state->tile_cmds_consumed;

	for (; consumed != posted; consumed++) {
		tile_index = consumed & (PCIE_CMD_QUEUE_ENTRIES - 1);
		cmd = &queue_state->tile_cmd_buffer[tile_index];

		/* Get a free tile_cmd_t structure from the free list. */
		tile_cmd = queue_state->free_tile_cmds;
		if (tile_cmd == NULL)
			ERR("NULL tile_cmd\n");

		if (tile_cmd->next_index == TILE_CMD_NULL) {
			queue_state->free_tile_cmds = NULL;
		} else {
			queue_state->free_tile_cmds =
				I2P(queue_state->tile_cmd_array, tile_cmd_t,
				    tile_cmd->next_index);
		}

		tile_cmd_index = P2I(queue_state->tile_cmd_array, tile_cmd_t,
				     tile_cmd) + 1;

		tile_cmd->buffer_addr = (void *) cmd->buffer_addr;
		tile_cmd->size = cmd->size;
		if (stream->cmd_queue->type == TLR_ZC_CMD_T2H)
			tile_cmd->tag = cmd->tag;

		list = &queue_state->unmatched_cmds;

		if (list->head == 0) {
			/*
			 * The list is empty; allocate an item and set the list
			 * flag as containing unmatched tile commands.
			 */
			TRACE("Tile: new list\n");

			list->head = tile_cmd_index;
			list->tail = tile_cmd_index;
			list->is_host_list = 0;
			tile_cmd->next_index = TILE_CMD_NULL;
		} else if (list->is_host_list) {
			/*
			 * The list contains unmatched host commands; we'll
			 * match the first one so pop it off and put it on the
			 * dma_cmds list.
			 */
			TRACE("Tile: match\n");

			host_cmd = I2P(queue_state->host_cmd_array, host_cmd_t,
				       (list->head - 1));

			/* Set the transfer size for the new command. */
			if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
				send_size = host_cmd->size;
				recv_size = tile_cmd->size;
			} else {
				send_size = tile_cmd->size;
				recv_size = host_cmd->size;
			}

			/* Allocate a new dma_cmd structure and queue it up. */
			dma_cmd = gxpci_alloc_dma(queue_state, host_cmd,
						  tile_cmd);

			dma_cmd->size = MIN(send_size, recv_size);
			if (recv_size < send_size)
				dma_cmd->overflow = 1;

			dma_cmd->host_cmd_done = 1;
			dma_cmd->tile_cmd_done = 1;

			/* Dequeue this host cmd from the unmatched list. */
			if (host_cmd->next_index == HOST_CMD_NULL) {
				list->head = 0;
				list->tail = 0;
			} else {
				list->head = host_cmd->next_index;
			}
		} else {
			/*
			 * The list contains unmatched tile commands;
			 * add ourselves to the tail of the list.
			 */
			TRACE("Tile: add to tail\n");

			tail_cmd = I2P(queue_state->tile_cmd_array, tile_cmd_t,
				       (list->tail - 1));
			tail_cmd->next_index = tile_cmd_index;
			list->tail = tile_cmd_index;
			tile_cmd->next_index = TILE_CMD_NULL;
		}
	}

	queue_state->tile_cmds_consumed = consumed;
}


void gxpci_zc_intr_timer_handler(unsigned long arg)
{
	struct tlr_zc_stream *stream = (struct tlr_zc_stream *) arg;

	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {

		/* Check whether there are any pending interrupts to be sent. */
		if (stream->h2t_regs->completion_consumed_count !=
		    stream->h2t_regs->completion_posted_count) {

			gxpci_update_msix_vector(&stream->h2t_msix_msg);

			/* Send the interrupt if the host can accept it. */
			if (stream->h2t_regs->interrupt_enable)
				gxio_trio_trigger_host_interrupt(
					stream->tlr->trio,
					stream->tlr->mac, 0,
					stream->h2t_msix_msg.msix_addr,
					stream->h2t_msix_msg.msix_data);

			/*
			 * Re-arm the timer to make sure that the host can get
			 * all the completions from the endpoint.
			 */
			mod_timer(&stream->intr_timer,
				jiffies + EP_INTR_TIMER_INTERVAL);
		}
	} else {
		/* Check whether there are any pending interrupts to be sent. */
		if (stream->t2h_regs->completion_consumed_count !=
		    stream->t2h_regs->completion_posted_count) {

			gxpci_update_msix_vector(&stream->t2h_msix_msg);

			/* Send the interrupt if the host can accept it. */
			if (stream->t2h_regs->interrupt_enable)
				gxio_trio_trigger_host_interrupt(
					stream->tlr->trio,
					stream->tlr->mac, 0,
					stream->t2h_msix_msg.msix_addr,
					stream->t2h_msix_msg.msix_data);

			/*
			 * Re-arm the timer to make sure that the host can get
			 * all the completions from the endpoint.
			 */
			mod_timer(&stream->intr_timer,
				  jiffies + EP_INTR_TIMER_INTERVAL);
		}
	}
}
#endif


static void gxpci_cs_intr_timer_handler(unsigned long arg)
{
	struct tlr_stream *stream = (struct tlr_stream *) arg;
	struct tlr_pcie_dev *ep_dev = stream->dev;
	gxio_trio_context_t *context = ep_dev->trio;
	int result;

	gxpci_update_msix_vector(&stream->msix_msg);

	result = gxio_trio_trigger_host_interrupt(context,
						  ep_dev->mac, 0,
						  stream->msix_msg.msix_addr,
						  stream->msix_msg.msix_data);
	/*
	 * Re-arm the timer to make sure that the endpoint can trigger MSI/MSI-X
	 * interrupt successfully.
	 */
	if (result)
		mod_timer(&stream->intr_timer,
			  jiffies + EP_INTR_TIMER_INTERVAL);
}


static irqreturn_t stream_intr(int irq, void *ep_dev)
{
	struct tlr_pcie_dev *tlr = (struct tlr_pcie_dev *) ep_dev;
	struct tlr_stream *stream;
	struct gxpci_queue_state *queue_state;
	gxio_trio_dma_queue_t *dma_queue_desc;
	gxio_trio_dma_queue_t *dma_queue_data;
	int i;
	unsigned int dma_cmds_completed, j, index_start, index_end;
	volatile uint16_t hw_cnt;
	uint16_t batch_completions;
	unsigned long flags;

	/*
	 * Check if this is an MMI interrupt, MMI irq number is indentical
	 * to all char streams, so just simply read it from stream #0.
	 */
	stream = tlr_get_stream(tlr, 0);
	if (irq == stream->mmi_irq) {

		for (i = 0; i < GXPCI_HOST_CHAR_STREAMS_COUNT; i++) {
			stream = tlr_get_stream(tlr, i);

			if (stream->is_ready) {

				gxpci_fetch_host_cmds(
					stream, GXPCI_CHAR_T2H_STREAM, 1);

				gxpci_fetch_host_cmds(
					stream, GXPCI_CHAR_H2T_STREAM, 1);
			}
		}

		return IRQ_HANDLED;
	}

	/*
	 * Check if this is a pull DMA desc done interrupt, the irq numbers are
	 * identical to all the char streams.
	 */
	for (i = 0; i < GXPCI_HOST_CHAR_STREAMS_COUNT; i++) {
		stream = tlr_get_stream(tlr, i);

		/* H2T queue. */
		if (stream->is_ready &&
		    irq == stream->h2t_dma_resource.pull_dma_desc_irq) {

			/* Update the number of completed pull DMA. */
			queue_state = &stream->h2t_queue_state;

			dma_queue_desc =
				&stream->h2t_dma_resource.pull_dma_queue_desc;

			/* Lock down. */
			spin_lock_irqsave(&stream->h2t_cmd_queue_lock, flags);

			hw_cnt =
				gxio_trio_read_dma_queue_complete_count(
					dma_queue_desc);

			dma_cmds_completed =
				queue_state->fetch_dmas_completed;
			batch_completions = (uint16_t) (hw_cnt -
				(dma_cmds_completed &
				 TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));

			if (batch_completions) {
				index_start = queue_state->fetch_dmas_completed;
				index_end = index_start + batch_completions;
				for (j = index_start; j != index_end; j++) {
					queue_state->host_cmds_posted +=
					  queue_state->host_cmds_cnt_array[j &
						(PCIE_CMD_QUEUE_ENTRIES - 1)];
				}
				queue_state->fetch_dmas_completed +=
					batch_completions;

				gxpci_parse_host_cmds(queue_state,
						      GXPCI_CHAR_H2T_STREAM);
			}

			/* Unlock. */
			spin_unlock_irqrestore(&stream->h2t_cmd_queue_lock,
					       flags);

			dma_queue_data =
				&stream->h2t_dma_resource.pull_dma_queue_data;
			/* Lock down. */
			spin_lock_irqsave(&stream->h2t_cmd_queue_lock, flags);
			gxpci_process_host_cmds(dma_queue_data,	queue_state,
						GXPCI_CHAR_H2T_STREAM);

			/* Unlock. */
			spin_unlock_irqrestore(&stream->h2t_cmd_queue_lock,
					       flags);
		}

		/* T2H queue. */
		if (stream->is_ready &&
		    irq == stream->t2h_dma_resource.pull_dma_desc_irq) {

			/* Lock down. */
			spin_lock_irqsave(&stream->t2h_cmd_queue_lock, flags);

			/* Update the number of completed pull DMA. */
			queue_state = &stream->t2h_queue_state;

			dma_cmds_completed =
				queue_state->fetch_dmas_completed;
			dma_queue_desc =
				&stream->t2h_dma_resource.pull_dma_queue_desc;
			hw_cnt =
				gxio_trio_read_dma_queue_complete_count(
					dma_queue_desc);
			batch_completions = (uint16_t) (hw_cnt -
				(dma_cmds_completed &
				 TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));

			if (batch_completions) {
				index_start = queue_state->fetch_dmas_completed;
				index_end = index_start + batch_completions;
				for (j = index_start; j != index_end; j++) {
					queue_state->host_cmds_posted +=
					  queue_state->host_cmds_cnt_array[j &
						(PCIE_CMD_QUEUE_ENTRIES - 1)];
				}
				queue_state->fetch_dmas_completed +=
					batch_completions;

				gxpci_parse_host_cmds(queue_state,
						      GXPCI_CHAR_T2H_STREAM);
			}

			/* Unlock. */
			spin_unlock_irqrestore(&stream->t2h_cmd_queue_lock,
					       flags);

			/* Lock down. */
			spin_lock_irqsave(&stream->t2h_cmd_queue_lock,
					  flags);

			dma_queue_data =
				&stream->t2h_dma_resource.push_dma_queue_data;
			gxpci_process_host_cmds(dma_queue_data, queue_state,
						GXPCI_CHAR_T2H_STREAM);

			/* Unlock. */
			spin_unlock_irqrestore(&stream->t2h_cmd_queue_lock,
					       flags);
		}
	}

	/*
	 * Check if this is a pull/push DMA data done interrupt, the irq
	 * numbers are identical to all the char streams.
	 */
	for (i = 0; i < GXPCI_HOST_CHAR_STREAMS_COUNT; i++) {
		stream = tlr_get_stream(tlr, i);

		/* H2T queue. */
		if (stream->is_ready &&
		    irq == stream->h2t_dma_resource.pull_dma_data_irq) {

			/* Lock down. */
			spin_lock_irqsave(&stream->h2t_cmd_queue_lock, flags);

			gxpci_process_comps(stream, GXPCI_CHAR_H2T_STREAM);

			/* Unlock. */
			spin_unlock_irqrestore(&stream->h2t_cmd_queue_lock,
					       flags);
		}

		/* T2H queue. */
		if (stream->is_ready &&
		    irq == stream->t2h_dma_resource.push_dma_data_irq) {

			/* Lock down. */
			spin_lock_irqsave(&stream->t2h_cmd_queue_lock, flags);

			gxpci_process_comps(stream, GXPCI_CHAR_T2H_STREAM);

			/* Unlock. */
			spin_unlock_irqrestore(&stream->t2h_cmd_queue_lock,
					       flags);
		}
	}

	/*
	 * Check if this is a push DMA comp done interrupt, the irq
	 * numbers are identical to all the char streams.
	 */
	for (i = 0; i < GXPCI_HOST_CHAR_STREAMS_COUNT; i++) {
		stream = tlr_get_stream(tlr, i);

		/* H2T queue. */
		if (stream->is_ready &&
		    irq == stream->h2t_dma_resource.push_dma_comp_irq) {

			gxpci_update_host(stream, GXPCI_CHAR_H2T_STREAM);
		}

		/* T2H queue. */
		if (stream->is_ready &&
		    irq == stream->t2h_dma_resource.push_dma_comp_irq) {

			gxpci_update_host(stream, GXPCI_CHAR_T2H_STREAM);

		}
	}

	return IRQ_HANDLED;
}


#if GXPCI_HOST_ZC_QUEUE_COUNT
/* Interrupt handler for zero-copy streams. */
irqreturn_t zc_stream_intr(int irq, void *ep_dev)
{
	struct tlr_pcie_dev *tlr = (struct tlr_pcie_dev *) ep_dev;
	struct tlr_zc_stream *stream;
	unsigned long flags;
	int i;

	/* MMI interrupt irq number is identical to all the ZC queues. */
	for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
		stream = tlr_get_zc_h2t_stream(tlr, i);

		/* Lock down. */
		spin_lock_irqsave(&stream->trio_state->trio_queue_lock, flags);

		if (stream->cmd_queue->is_ready)
			gxpci_zc_fetch_host_cmds(stream, 1);

		/* Unlock. */
		spin_unlock_irqrestore(
			&stream->trio_state->trio_queue_lock, flags);

		stream = tlr_get_zc_t2h_stream(tlr, i);

		/* Lock down. */
		spin_lock_irqsave(&stream->trio_state->trio_queue_lock, flags);

		if (stream->cmd_queue->is_ready)
			gxpci_zc_fetch_host_cmds(stream, 1);

		/* Unlock. */
		spin_unlock_irqrestore(
			&stream->trio_state->trio_queue_lock, flags);
	}

	return IRQ_HANDLED;
}


/* DMA interrupt handler for zero-copy H2T streams. */
irqreturn_t zc_h2t_dma_intr(int irq, void *ep_dev)
{
	struct tlr_zc_stream *stream = (struct tlr_zc_stream *) ep_dev;
	struct tlr_zc_stream *shared_stream;
	struct tlr_pcie_dev *tlr = stream->tlr;
	struct gxpci_queue_state *shared_queue_state;
	gxio_trio_dma_queue_t *dma_queue_desc;
	spinlock_t *lock;
	unsigned int stream_index = stream->index;
	unsigned int dma_cmds_completed, i, j;
	unsigned int index, index_start, index_end;
	volatile uint16_t hw_cnt;
	uint16_t batch_completions;
	unsigned long flags;

	for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {

		/* Handle the TRIO-shared queues only. */
		if ((stream_index & (ZC_QUEUE_TRIO_NUM - 1)) !=
		    (i & (ZC_QUEUE_TRIO_NUM - 1)))
			continue;

		stream = tlr_get_zc_h2t_stream(tlr, i);
		lock = &stream->trio_state->trio_queue_lock;

		/* Lock down. */
		spin_lock_irqsave(lock, flags);

		if (!stream->cmd_queue->is_ready) {
			/* Unlock. */
			spin_unlock_irqrestore(lock, flags);
			continue;
		}

		dma_queue_desc = &stream->h2t_dma_resource->pull_dma_queue_desc;
		hw_cnt =
			gxio_trio_read_dma_queue_complete_count(dma_queue_desc);
		batch_completions = (uint16_t) (hw_cnt -
			(stream->trio_state->cmd_dma_chan_dequeued &
			 TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));

		/* Dispatch all the pull DMA completions. */
		index_start = stream->trio_state->cmd_dma_chan_dequeued;
		index_end = index_start + batch_completions;
		for (j = index_start; j != index_end; j++) {
			index = stream->trio_state->cmd_dma_chans[j &
				(GXPCI_STREAM_PULL_DMA_RING_LEN - 1)];
			shared_stream = tlr_get_zc_h2t_stream(tlr, index);

			/* Update the number of completed pull DMA. */
			shared_queue_state = &shared_stream->h2t_queue_state;
			dma_cmds_completed =
				shared_queue_state->fetch_dmas_completed;
			shared_queue_state->host_cmds_posted +=
				shared_queue_state->host_cmds_cnt_array[
					dma_cmds_completed &
					(PCIE_CMD_QUEUE_ENTRIES - 1)];

			shared_queue_state->fetch_dmas_completed++;
		}
		stream->trio_state->cmd_dma_chan_dequeued +=
			batch_completions;

		gxpci_zc_parse_host_cmds(stream);

		/* Unlock. */
		spin_unlock_irqrestore(lock, flags);

		/* Lock down. */
		spin_lock_irqsave(lock, flags);

		gxpci_zc_process_host_cmds(stream);

		/* Unlock. */
		spin_unlock_irqrestore(lock, flags);

		/* Lock down. */
		spin_lock_irqsave(lock, flags);

		gxpci_zc_process_comps(stream);

		/* Unlock. */
		spin_unlock_irqrestore(lock, flags);

		/* Lock down. */
		spin_lock_irqsave(lock, flags);

		gxpci_zc_update_host(stream);

		/* Unlock. */
		spin_unlock_irqrestore(lock, flags);
	}

	return IRQ_HANDLED;
}


/* DMA interrupt handler for zero-copy T2H streams. */
irqreturn_t zc_t2h_dma_intr(int irq, void *ep_dev)
{
	struct tlr_zc_stream *stream = (struct tlr_zc_stream *) ep_dev;
	struct tlr_zc_stream *shared_stream;
	struct tlr_pcie_dev *tlr = stream->tlr;
	struct gxpci_queue_state *shared_queue_state;
	gxio_trio_dma_queue_t *dma_queue_desc;
	spinlock_t *lock;
	unsigned int stream_index = stream->index;
	unsigned int dma_cmds_completed, i, j;
	unsigned int index, index_start, index_end;
	volatile uint16_t hw_cnt;
	uint16_t batch_completions;
	unsigned long flags;

	for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {

		/* Handle the TRIO-shared queues only. */
		if ((stream_index & (ZC_QUEUE_TRIO_NUM - 1)) !=
		    (i & (ZC_QUEUE_TRIO_NUM - 1)))
			continue;

		stream = tlr_get_zc_t2h_stream(tlr, i);
		lock = &stream->trio_state->trio_queue_lock;

		/* Lock down. */
		spin_lock_irqsave(lock, flags);

		if (!stream->cmd_queue->is_ready)  {
			/* Unlock. */
			spin_unlock_irqrestore(lock, flags);
			continue;
		}

		dma_queue_desc = &stream->t2h_dma_resource->pull_dma_queue_desc;
		hw_cnt =
			gxio_trio_read_dma_queue_complete_count(dma_queue_desc);
		batch_completions = (uint16_t) (hw_cnt -
		       (stream->trio_state->cmd_dma_chan_dequeued &
			TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));

		/* Dispatch all the pull DMA completions. */
		index_start = stream->trio_state->cmd_dma_chan_dequeued;
		index_end = index_start + batch_completions;
		for (j = index_start; j != index_end; j++) {
			index = stream->trio_state->cmd_dma_chans[j &
				(GXPCI_STREAM_PULL_DMA_RING_LEN - 1)];
			shared_stream = tlr_get_zc_t2h_stream(tlr, index);

			/* Update the number of completed pull DMA. */
			shared_queue_state = &shared_stream->t2h_queue_state;
			dma_cmds_completed =
				shared_queue_state->fetch_dmas_completed;
			shared_queue_state->host_cmds_posted +=
				shared_queue_state->host_cmds_cnt_array[
					dma_cmds_completed &
					(PCIE_CMD_QUEUE_ENTRIES - 1)];

			shared_queue_state->fetch_dmas_completed++;
		}
		stream->trio_state->cmd_dma_chan_dequeued +=
			batch_completions;

		gxpci_zc_parse_host_cmds(stream);

		/* Unlock. */
		spin_unlock_irqrestore(lock, flags);

		/* Lock down. */
		spin_lock_irqsave(lock, flags);

		gxpci_zc_process_host_cmds(stream);

		/* Unlock. */
		spin_unlock_irqrestore(lock, flags);

		/* Lock down. */
		spin_lock_irqsave(lock, flags);

		gxpci_zc_process_comps(stream);

		/* Unlock. */
		spin_unlock_irqrestore(lock, flags);

		/* Lock down. */
		spin_lock_irqsave(lock, flags);

		gxpci_zc_update_host(stream);

		/* Unlock. */
		spin_unlock_irqrestore(lock, flags);
	}

	return IRQ_HANDLED;
}
#endif


/*
 * Pick a CPU to receive and handle the PCIe interrupts, based on the IRQ #.
 * For now, we simply send interrupts to non-dataplane CPUs.
 * We may implement methods to allow user to specify the target CPUs,
 * e.g. via boot arguments.
 */
static int tile_irq_cpu(int irq)
{
	unsigned int count;
	int i = 0;
	int cpu;

	count = cpumask_weight(&intr_cpus_map);

	if (unlikely(count == 0)) {
		pr_warning("intr_cpus_map empty, interrupts will be"
			   " delievered to dataplane tiles\n");
		return irq % (smp_height * smp_width);
	}

	count = irq % count;
	for_each_cpu(cpu, &intr_cpus_map) {
		if (i++ == count)
			break;
	}

	return cpu;
}


/* Initialize the queue state. */
static int gxpci_queue_state_init(struct tlr_stream *stream, void *backing_mem)
{
	struct gxpci_queue_state *queue_state;
	size_t line_size;
	size_t dma_buff_size;
	int i;

	/*
	 * Allocate buffer for DMAing commands and completions from the
	 * host; these must be cacheline aligned and padded.
	 */
	line_size = (1 << CHIP_L2_LOG_LINE_SIZE());
	dma_buff_size = sizeof(*queue_state->host_cmd_dma_buffer) *
		PCIE_CMD_QUEUE_ENTRIES;
	dma_buff_size = ROUND_UP(dma_buff_size, line_size);

	if (dma_buff_size > GXPCI_HOST_CMDS_COMP_BUF_SIZE_LIMIT)
		return -ENXIO;

	/* H2T queue state init. */
	queue_state = &stream->h2t_queue_state;
	queue_state->host_cmd_dma_buffer = backing_mem +
		GXPCI_H2T_HOST_CMD_DMA_BUF_OFFSET;

	queue_state->host_cmds_posted = 0;
	queue_state->host_cmds_dma_started = 0;
	queue_state->host_cmds_consumed = 0;
	queue_state->fetch_dmas_started = 0;
	queue_state->fetch_dmas_completed = 0;
	queue_state->host_cmds_cnt_array = backing_mem +
		GXPCI_H2T_CMDS_CNT_ARRAY_OFFSET;
	queue_state->host_cpl_dma_buffer = backing_mem +
		GXPCI_H2T_HOST_COMPS_DMA_BUF_OFFSET;

	queue_state->host_cpl_posted = 0;
	queue_state->host_cpl_dma_started = 0;
	queue_state->host_cpl_consumed = 0;
	queue_state->comp_dmas_started = 0;
	queue_state->comp_dmas_completed = 0;
	queue_state->host_cpls_cnt_array = backing_mem +
		GXPCI_H2T_CPLS_CNT_ARRAY_OFFSET;

	queue_state->host_cmd_bus_addr =
		stream->h2t_buf_cmd_array_addr;
	queue_state->host_cpl_bus_addr =
		stream->h2t_cmpl_array_addr;

	queue_state->tile_cmd_buffer = stream->h2t_buffer_cmd_array;

	queue_state->tile_cmds_consumed = 0;
	queue_state->tile_cmds_posted = 0;

	/* Build a free list of tile_cmd_t structures. */
	queue_state->tile_cmd_array = (tile_cmd_t *) (backing_mem +
		GXPCI_H2T_TILE_CMD_ARRAY_BUF_OFFSET);

	for (i = 0; i < PCIE_CMD_QUEUE_ENTRIES; i++) {
		tile_cmd_t *cmd = queue_state->tile_cmd_array + i;
		cmd->next_index = i + 1;
	}
	queue_state->tile_cmd_array[i - 1].next_index = TILE_CMD_NULL;
	queue_state->free_tile_cmds = queue_state->tile_cmd_array;

	queue_state->tile_cpl_buffer = backing_mem +
		GXPCI_H2T_TILE_COMP_ARRAY_BUF_OFFSET;
	queue_state->tile_cpl_posted = 0;
	queue_state->tile_cpl_consumed = 0;

	/* Build a free list of host_cmd_t structures. */
	queue_state->host_cmd_array = (host_cmd_t *) (backing_mem +
		GXPCI_H2T_HOST_CMD_ARRAY_BUF_OFFSET);
	for (i = 0; i < PCIE_CMD_QUEUE_ENTRIES; i++) {
		host_cmd_t *cmd = queue_state->host_cmd_array + i;
		cmd->next_index = i + 1;
	}
	queue_state->host_cmd_array[i - 1].next_index = HOST_CMD_NULL;
	queue_state->free_host_cmds = queue_state->host_cmd_array;

	/*
	 * Build a free list of dma_cmd_t structures; note that we skip the
	 * first entry in the array so that we can use 0 as an end-of-list
	 * flag.
	 */
	queue_state->dma_cmd_array = (dma_cmd_t *) (backing_mem +
		GXPCI_H2T_DMA_CMD_ARRAY_BUF_OFFSET);
	for (i = 1; i < PCIE_CMD_QUEUE_ENTRIES * 2; i++) {
		dma_cmd_t *cmd = queue_state->dma_cmd_array + i;
		cmd->next_index = i + 1;
	}
	queue_state->dma_cmd_array[i - 1].next_index = DMA_CMD_NULL;
	queue_state->free_dma_cmds = queue_state->dma_cmd_array + 1;

	/* T2H queue state init. */
	queue_state = &stream->t2h_queue_state;
	queue_state->host_cmd_dma_buffer = backing_mem +
		GXPCI_T2H_HOST_CMD_DMA_BUF_OFFSET;

	queue_state->host_cmds_posted = 0;
	queue_state->host_cmds_consumed = 0;
	queue_state->host_cmds_dma_started = 0;
	queue_state->fetch_dmas_started = 0;
	queue_state->fetch_dmas_completed = 0;
	queue_state->host_cmds_cnt_array = backing_mem +
		GXPCI_T2H_CMDS_CNT_ARRAY_OFFSET;
	queue_state->host_cpl_dma_buffer = backing_mem +
		GXPCI_T2H_HOST_COMPS_DMA_BUF_OFFSET;

	queue_state->host_cpl_posted = 0;
	queue_state->host_cpl_consumed = 0;
	queue_state->host_cpl_dma_started = 0;
	queue_state->comp_dmas_started = 0;
	queue_state->comp_dmas_completed = 0;
	queue_state->host_cpls_cnt_array = backing_mem +
		GXPCI_T2H_CPLS_CNT_ARRAY_OFFSET;

	queue_state->host_cmd_bus_addr =
		stream->t2h_buf_cmd_array_addr;
	queue_state->host_cpl_bus_addr =
		stream->t2h_cmpl_array_addr;

	queue_state->tile_cmd_buffer = stream->t2h_buffer_cmd_array;

	queue_state->tile_cmds_consumed = 0;
	queue_state->tile_cmds_posted = 0;

	/* Build a free list of tile_cmd_t structures. */
	queue_state->tile_cmd_array = (tile_cmd_t *) (backing_mem +
		GXPCI_T2H_TILE_CMD_ARRAY_BUF_OFFSET);

	for (i = 0; i < PCIE_CMD_QUEUE_ENTRIES; i++) {
		tile_cmd_t *cmd = queue_state->tile_cmd_array + i;
		cmd->next_index = i + 1;
	}
	queue_state->tile_cmd_array[i - 1].next_index = TILE_CMD_NULL;
	queue_state->free_tile_cmds = queue_state->tile_cmd_array;

	queue_state->tile_cpl_buffer = backing_mem +
		GXPCI_T2H_TILE_COMP_ARRAY_BUF_OFFSET;
	queue_state->tile_cpl_posted = 0;
	queue_state->tile_cpl_consumed = 0;

	/* Build a free list of host_cmd_t structures. */
	queue_state->host_cmd_array = (host_cmd_t *) (backing_mem +
		GXPCI_T2H_HOST_CMD_ARRAY_BUF_OFFSET);
	for (i = 0; i < PCIE_CMD_QUEUE_ENTRIES; i++) {
		host_cmd_t *cmd = queue_state->host_cmd_array + i;
		cmd->next_index = i + 1;
	}
	queue_state->host_cmd_array[i - 1].next_index = HOST_CMD_NULL;
	queue_state->free_host_cmds = queue_state->host_cmd_array;

	/*
	 * Build a free list of dma_cmd_t structures; note that we skip the
	 * first entry in the array so that we can use 0 as an end-of-list
	 * flag.
	 */
	queue_state->dma_cmd_array = (dma_cmd_t *) (backing_mem +
		GXPCI_T2H_DMA_CMD_ARRAY_BUF_OFFSET);
	for (i = 1; i < PCIE_CMD_QUEUE_ENTRIES * 2; i++) {
		dma_cmd_t *cmd = queue_state->dma_cmd_array + i;
		cmd->next_index = i + 1;
	}
	queue_state->dma_cmd_array[i - 1].next_index = DMA_CMD_NULL;
	queue_state->free_dma_cmds = queue_state->dma_cmd_array + 1;

	return 0;
}


/* Post a set of read buffers to the iBound. */
static void post_read_buffers(struct tlr_stream *stream, uint32_t start,
			      uint32_t stop)
{
	uint32_t stream_posted;
	uint32_t cmds_posted;
	uint32_t cmd_index;

	/* Build a template command on the local stack. */
	pcie_tile_buffer_cmd_t cmd = { 0 };
	cmd.size = BUFFER_SIZE;

	cmds_posted = stream->h2t_commands_posted;

	/*
	 * This loop posts buffers from the per-stream ring buffer
	 * into the global command ring buffer.  We keep a separate
	 * 'posted' count for each ring, and calculate the index
	 * within each ring as (posted % ring_entries).
	 */
	for (stream_posted = start; stream_posted != stop; stream_posted++) {
		uint32_t index = stream_posted & (BUFFERS_PER_STREAM - 1);
		uint64_t va = (uint64_t) stream->read_buffers[index];
		cmd.buffer_addr = va;
		TRACE("READ size= %d, VA= %#llx\n", cmd.size, va);

		cmd_index = cmds_posted & (PCIE_CMD_QUEUE_ENTRIES - 1);
		stream->h2t_buffer_cmd_array[cmd_index] = cmd;
		cmds_posted++;
	}

	if (start != stop) {
		/*
		 * Make sure the commands are visible, then update the posted
		 * count.
		 */
		__insn_mf();
		stream->h2t_commands_posted = cmds_posted;
		stream->h2t_queue_state.tile_cmds_posted = cmds_posted;
	}
}


/* Post a set of write buffers to the iBound. */
static void post_write_buffers(struct tlr_stream *stream, uint32_t start,
			       uint32_t stop)
{
	uint32_t stream_posted;
	uint32_t cmds_posted;
	uint32_t cmd_index;
	struct gxpci_queue_state *queue_state = &stream->t2h_queue_state;

	/* Build a template command on the local stack. */
	pcie_tile_buffer_cmd_t cmd = { 0 };

	cmds_posted = stream->t2h_commands_posted;

	/*
	 * This loop posts buffers from the per-stream ring buffer
	 * into the global command ring buffer.  We keep a separate
	 * 'posted' count for each ring, and calculate the index
	 * within each ring as (posted % ring_entries).
	 */
	for (stream_posted = start; stream_posted != stop; stream_posted++) {
		uint32_t index = stream_posted & (BUFFERS_PER_STREAM - 1);
		uint64_t va = (uint64_t) stream->write_buffers[index];
		cmd.buffer_addr = va;
		cmd.size = stream->write_sizes[index];
		TRACE("WRITE size= %d, VA= %#llx\n", cmd.size, va);

		cmd_index = cmds_posted & (PCIE_CMD_QUEUE_ENTRIES - 1);
		stream->t2h_buffer_cmd_array[cmd_index] = cmd;
		cmds_posted++;
	}

	if (start != stop) {
		/*
		 * Make sure the commands are visible, then update the posted
		 * count.
		 */
		__insn_mf();
		stream->t2h_commands_posted = cmds_posted;
		queue_state->tile_cmds_posted = cmds_posted;
	}
}


/* Initialize a newly opened stream. */
static int init_stream(struct tlr_stream *stream)
{
	struct tlr_pcie_dev *ep_dev = stream->dev;
	gxio_trio_context_t *trio_context = ep_dev->trio;
	struct page *page;
	unsigned long size;
	char *buffer;
	uint32_t h2t_status = 0;
	uint32_t t2h_status = 0;
	int err = 0;
	int i;
	int irq;
	int cpu;
	int dma_ring;
	int msix_table_index;
	int sleep_msecs = 0;
	const int POLLING_MSECS = 200;
	const int MAX_MSECS = 90000;
	size_t dma_ring_size;
	void *dma_ring_mem;
	void *backing_mem;
	gxio_trio_dma_queue_t *dma_queue;
	unsigned int pull_dma_rings[3];
	unsigned int push_dma_rings[3];
	int pull_dma_irqs[3];
	int push_dma_irqs[3];
	struct gxpci_queue_state *queue_state;

	/* We expect ~4k pages. */
	if (PAGE_SIZE < BUFFER_SIZE) {
		ERR("Page size must be at least %d\n", BUFFER_SIZE);
		return -ENOMEM;
	}

	/* Fill in variables for the host and set up ready flag. */
	stream->h2t_regs->buffer_cmd_posted_count = 0;
	stream->h2t_regs->completion_posted_count = 0;
	stream->h2t_regs->completion_consumed_count = 0;

	stream->t2h_regs->buffer_cmd_posted_count = 0;
	stream->t2h_regs->completion_posted_count = 0;
	stream->t2h_regs->completion_consumed_count = 0;

	/* Fence here to avoid write re-ordering.*/
	__insn_mf();

	/* Start the handshake process */
	stream->h2t_regs->queue_status = GXPCI_TILE_CHAN_READY;
	stream->t2h_regs->queue_status = GXPCI_TILE_CHAN_READY;

	/* Wait until the host is ready. */
	while (1) {
		h2t_status = stream->h2t_regs->queue_status;
		t2h_status = stream->t2h_regs->queue_status;
		atomic_compiler_barrier();

		if (h2t_status == GXPCI_HOST_CHAN_READY &&
		    t2h_status == GXPCI_HOST_CHAN_READY)
			break;

		if (sleep_msecs >= MAX_MSECS) {
			ERR("Host is not ready\n");
			return -ENXIO;
		}

		msleep(POLLING_MSECS);
		sleep_msecs += POLLING_MSECS;
	}

	TRACE("INFO: Host is ready now!\n");

	/* After the flag, record host array bus addresses. */
	stream->h2t_cmpl_array_addr = stream->h2t_regs->completion_array;
	stream->t2h_cmpl_array_addr = stream->t2h_regs->completion_array;

	stream->h2t_buf_cmd_array_addr = stream->h2t_regs->buffer_cmd_array;
	stream->t2h_buf_cmd_array_addr = stream->t2h_regs->buffer_cmd_array;

	/* Choose an arbitrary processor to home the buffer memory. */
	cpu = raw_smp_processor_id();

	size = BUFFERS_PER_STREAM * BUFFER_SIZE * 2;
	page = homecache_alloc_pages(GFP_KERNEL | __GFP_ZERO,
				     get_order(size), cpu);
	if (!page) {
		err = -ENOMEM;
		goto alloc_buf_failed;
	}
	stream->buffer_page = (unsigned long) page_address(page);

	err = gxio_trio_register_page(trio_context, trio_context->asid,
				      page_address(page), size, 0);
	if (err < 0) {
		err = -ENOMEM;
		goto reg_buf_failed;
	}

	buffer = (char *) stream->buffer_page;
	for (i = 0; i < BUFFERS_PER_STREAM; i++) {
		stream->read_buffers[i] = buffer;
		buffer += BUFFER_SIZE;
	}
	for (i = 0; i < BUFFERS_PER_STREAM; i++) {
		stream->write_buffers[i] = buffer;
		buffer += BUFFER_SIZE;
	}

	/* Allocate tile command and tile completion buffers. */
	size = sizeof(struct pcie_tile_buffer_cmd) * PCIE_CMD_QUEUE_ENTRIES * 2;
	page = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(size));
	if (!page) {
		err = -ENOMEM;
		goto alloc_tile_cmd_buf_failed;
	}

	stream->h2t_buffer_cmd_array =
		(struct pcie_tile_buffer_cmd *) page_address(page);
	stream->t2h_buffer_cmd_array =
		(struct pcie_tile_buffer_cmd *) (page_address(page) + size / 2);

	/* Allocate H2T DMA resources. */
	page = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(HPAGE_SIZE));
	if (!page) {
		err = -ENOMEM;
		goto alloc_dma_buf_failed;
	}

	/* Register this DMA memory. */
	backing_mem = page_address(page);
	err = gxio_trio_register_page(trio_context, trio_context->asid,
				      backing_mem, HPAGE_SIZE, 0);
	if (err < 0) {
		err = -ENOMEM;
		goto reg_dma_buf_failed;
	}

	/*
	 * Allocate and initialize a pull DMA ring for the command descriptors.
	 */
	dma_ring = gxio_trio_alloc_pull_dma_ring(trio_context, 1, 0, 0);
	if (dma_ring < 0) {
		err = -ENXIO;
		goto failed;
	}

	/*
	 * Bind the DMA ring to our MAC, and use registered memory to store
	 * the command ring.
	 */
	dma_ring_size = GXPCI_STREAM_PULL_DMA_RING_LEN *
		sizeof(gxio_trio_dma_desc_t);
	dma_ring_mem = backing_mem + GXPCI_STREAM_H2T_PULL_DMA_DESC_OFFSET;
	dma_queue = &stream->h2t_dma_resource.pull_dma_queue_desc;
	err = gxio_trio_init_pull_dma_queue(dma_queue, trio_context, dma_ring,
					    ep_dev->mac, trio_context->asid, 0,
					    dma_ring_mem, dma_ring_size, 0);
	if (err < 0) {
		err = -ENXIO;
		goto failed;
	}
	stream->h2t_dma_resource.pull_dma_desc = dma_ring;
	pull_dma_rings[0] = dma_ring;

	/*
	 * Allocate and initialize a pull DMA ring for the data.
	 */
	dma_ring = gxio_trio_alloc_pull_dma_ring(trio_context, 1, 0, 0);
	if (dma_ring < 0) {
		err = -ENXIO;
		goto failed;
	}

	/*
	 * Bind the DMA ring to our MAC, and use registered memory to store
	 * the command ring.
	 */
	dma_ring_size = GXPCI_STREAM_PULL_DMA_RING_LEN *
		sizeof(gxio_trio_dma_desc_t);
	dma_ring_mem = backing_mem + GXPCI_STREAM_H2T_PULL_DMA_DATA_OFFSET;
	dma_queue = &stream->h2t_dma_resource.pull_dma_queue_data;
	err = gxio_trio_init_pull_dma_queue(dma_queue, trio_context, dma_ring,
					    ep_dev->mac, trio_context->asid, 0,
					    dma_ring_mem, dma_ring_size, 0);
	if (err < 0) {
		err = -ENXIO;
		goto failed;
	}
	stream->h2t_dma_resource.pull_dma_data = dma_ring;
	pull_dma_rings[1] = dma_ring;

	/*
	 * Allocate and initialize a push DMA ring for writing the command
	 * completions back to the host.
	 */
	dma_ring = gxio_trio_alloc_push_dma_ring(trio_context, 1, 0, 0);
	if (dma_ring < 0) {
		err = -ENXIO;
		goto failed;
	}

	/*
	 * Bind the DMA ring to our MAC, and use registered memory to store
	 * the command ring.
	 */
	dma_ring_size = GXPCI_STREAM_PUSH_DMA_RING_LEN *
		sizeof(gxio_trio_dma_desc_t);
	dma_ring_mem = backing_mem + GXPCI_STREAM_H2T_PUSH_DMA_COMP_OFFSET;
	dma_queue = &stream->h2t_dma_resource.push_dma_queue_comp;
	err = gxio_trio_init_push_dma_queue(dma_queue, trio_context, dma_ring,
					    ep_dev->mac, trio_context->asid, 0,
					    dma_ring_mem, dma_ring_size, 0);
	if (err < 0) {
		err = -ENXIO;
		goto failed;
	}
	stream->h2t_dma_resource.push_dma_comp = dma_ring;
	push_dma_rings[0] = dma_ring;

	/* Allocate T2H DMA resources. */
	/*
	 * Allocate and initialize a pull DMA ring for the command descriptors.
	 */
	dma_ring = gxio_trio_alloc_pull_dma_ring(trio_context, 1, 0, 0);
	if (dma_ring < 0) {
		err = -ENXIO;
		goto failed;
	}

	/*
	 * Bind the DMA ring to our MAC, and use registered memory to store
	 * the command ring.
	 */
	dma_ring_size = GXPCI_STREAM_PULL_DMA_RING_LEN *
		sizeof(gxio_trio_dma_desc_t);
	dma_ring_mem = backing_mem + GXPCI_STREAM_T2H_PULL_DMA_DESC_OFFSET;
	dma_queue = &stream->t2h_dma_resource.pull_dma_queue_desc;
	err = gxio_trio_init_pull_dma_queue(dma_queue, trio_context, dma_ring,
					    ep_dev->mac, trio_context->asid, 0,
					    dma_ring_mem, dma_ring_size, 0);
	if (err < 0) {
		err = -ENXIO;
		goto failed;
	}
	stream->t2h_dma_resource.pull_dma_desc = dma_ring;
	pull_dma_rings[2] = dma_ring;

	/*
	 * Allocate and initialize a push DMA ring for the data.
	 */
	dma_ring = gxio_trio_alloc_push_dma_ring(trio_context, 1, 0, 0);
	if (dma_ring < 0) {
		err = -ENXIO;
		goto failed;
	}

	/*
	 * Bind the DMA ring to our MAC, and use registered memory to store
	 * the command ring.
	 */
	dma_ring_size = GXPCI_STREAM_PUSH_DMA_RING_LEN *
		sizeof(gxio_trio_dma_desc_t);
	dma_ring_mem = backing_mem + GXPCI_STREAM_T2H_PUSH_DMA_DATA_OFFSET;
	dma_queue = &stream->t2h_dma_resource.push_dma_queue_data;
	err = gxio_trio_init_push_dma_queue(dma_queue, trio_context, dma_ring,
					    ep_dev->mac, trio_context->asid, 0,
					    dma_ring_mem, dma_ring_size, 0);
	if (err < 0) {
		err = -ENXIO;
		goto failed;
	}
	stream->t2h_dma_resource.push_dma_data = dma_ring;
	push_dma_rings[1] = dma_ring;

	/*
	 * Allocate and initialize a push DMA ring for writing the command
	 * completions back to the host.
	 */
	dma_ring = gxio_trio_alloc_push_dma_ring(trio_context, 1, 0, 0);
	if (dma_ring < 0) {
		err = -ENXIO;
		goto failed;
	}

	/*
	 * Bind the DMA ring to our MAC, and use registered memory to store
	 * the command ring.
	 */
	dma_ring_size = GXPCI_STREAM_PUSH_DMA_RING_LEN *
		sizeof(gxio_trio_dma_desc_t);
	dma_ring_mem = backing_mem + GXPCI_STREAM_T2H_PUSH_DMA_COMP_OFFSET;
	dma_queue = &stream->t2h_dma_resource.push_dma_queue_comp;
	err = gxio_trio_init_push_dma_queue(dma_queue, trio_context, dma_ring,
					    ep_dev->mac, trio_context->asid, 0,
					    dma_ring_mem, dma_ring_size, 0);
	if (err < 0) {
		err = -ENXIO;
		goto failed;
	}
	stream->t2h_dma_resource.push_dma_comp = dma_ring;
	push_dma_rings[2] = dma_ring;

	/*
	 * Allocate and register a common interrupt for push/pull DMA rings to
	 * kernel with the same command char stream interrupt handle, i.e.
	 * stream_intr.
	 */
	irq = create_irq();
	if (irq < 0)
		goto irq_failed;

	tile_irq_activate(irq, TILE_IRQ_PERCPU);
	if (request_irq(irq, stream_intr, 0, driver_name, (void *) ep_dev))
		goto irq_failed;

	/* Try to choose a non-dataplane processor to receive DMA interrupts. */
	cpu = tile_irq_cpu(irq);

	/* Process three pull DMA rings. */
	for (i = 0; i < 3; i++) {
		err = gxio_trio_config_char_intr(trio_context,
						 cpu_x(cpu), cpu_y(cpu),
						 KERNEL_PL, irq,
						 ep_dev->mac, 0, 0,
						 pull_dma_rings[i],
						 PULL_DMA_SEL);
		if (err < 0)
			goto irq_failed;

		pull_dma_irqs[i] = irq;
	}

	/* Process three push DMA rings. */
	for (i = 0; i < 3; i++) {
		err = gxio_trio_config_char_intr(trio_context,
						 cpu_x(cpu), cpu_y(cpu),
						 KERNEL_PL, irq,
						 ep_dev->mac, 0,
						 push_dma_rings[i],
						 0,
						 PUSH_DMA_SEL);
		if (err < 0)
			goto irq_failed;

		push_dma_irqs[i] = irq;
	}

	/* Record the allocated irq numbers. */
	stream->h2t_dma_resource.pull_dma_desc_irq = pull_dma_irqs[0];
	stream->h2t_dma_resource.pull_dma_data_irq = pull_dma_irqs[1];
	stream->t2h_dma_resource.pull_dma_desc_irq = pull_dma_irqs[2];
	stream->h2t_dma_resource.push_dma_comp_irq = push_dma_irqs[0];
	stream->t2h_dma_resource.push_dma_data_irq = push_dma_irqs[1];
	stream->t2h_dma_resource.push_dma_comp_irq = push_dma_irqs[2];

	msix_table_index = ep_dev->msix_cs_q_intr_vec_base + stream->index;
	stream->msix_msg.msix_table_entry_offset = ep_dev->msix_table_base +
		PCI_MSIX_ENTRY_SIZE * msix_table_index;
	stream->msix_msg.msix_addr = *(unsigned long *)
		(stream->msix_msg.msix_table_entry_offset +
		 PCI_MSIX_ENTRY_LOWER_ADDR);
	stream->msix_msg.msix_data = *(unsigned int *)
		(stream->msix_msg.msix_table_entry_offset +
		 PCI_MSIX_ENTRY_DATA);

	/* Initialize queue_state. */
	err = gxpci_queue_state_init(stream, backing_mem);
	if (err < 0)
		goto queue_state_init_failed;

	/*
	 * Initialize a per queue timer to monitor any failure when
	 * trying to trigger MSI/MSI-X interrupts to the host.
	 */
	init_timer(&stream->intr_timer);
	stream->intr_timer.data = (unsigned long) stream;
	stream->intr_timer.function = gxpci_cs_intr_timer_handler;

	/* Post buffers for incoming read data. */
	post_read_buffers(stream, 0, BUFFERS_PER_STREAM);

	/* Add the buffers into tile_cmd list. */
	queue_state = &stream->h2t_queue_state;
	gxpci_check_tile_cmds(queue_state, GXPCI_CHAR_H2T_STREAM);

	/*
	 * Setup the ready flag to avoid this stream being initialized
	 * more than once.
	 */
	stream->is_ready = 1;

	/* Fence here to avoid write re-ordering.*/
	__insn_mf();

	/* Complete the handshake process. */
	stream->h2t_regs->queue_status = GXPCI_TILE_CHAN_READY_ACK;
	stream->t2h_regs->queue_status = GXPCI_TILE_CHAN_READY_ACK;

	return 0;

queue_state_init_failed:
	free_irq(irq, ep_dev);
irq_failed:
	if (irq >= 0)
		destroy_irq(irq);
failed:
reg_dma_buf_failed:
	homecache_free_pages((unsigned long)backing_mem, get_order(HPAGE_SIZE));
alloc_dma_buf_failed:
	size = BUFFERS_PER_STREAM * BUFFER_SIZE * 2;
	homecache_free_pages(stream->buffer_page, get_order(size));
	stream->buffer_page = 0;
alloc_tile_cmd_buf_failed:
reg_buf_failed:
	size = BUFFERS_PER_STREAM * BUFFER_SIZE * 2;
	homecache_free_pages(stream->buffer_page, get_order(size));
	stream->buffer_page = 0;
alloc_buf_failed:
	return err;
}


/**********************************************************************/
/*                        Character Device Routines                   */
/**********************************************************************/

static ssize_t tlr_cdev_read(struct file *filp, char __user *buf,
			     size_t count, loff_t *f_pos)
{
	struct tlr_stream *stream = filp->private_data;
	struct gxpci_queue_state *queue_state = &stream->h2t_queue_state;
	size_t already_read, bytes_read;
	uint32_t reads_consumed;
	unsigned long flags;
	gxio_trio_dma_queue_t *dma_queue_data =
		&stream->h2t_dma_resource.pull_dma_queue_data;
	gxio_trio_dma_queue_t *dma_queue_desc =
		&stream->h2t_dma_resource.pull_dma_queue_desc;
	unsigned int dma_cmds_completed, j, index_start, index_end;
	volatile uint16_t hw_cnt;
	uint16_t batch_completions;

	FOP_TRACE("Entered tlr_cdev_read\n");

	if (count == 0)
		return 0;

	/* Grab the stream read lock. */
	if (down_interruptible(&stream->read_mutex)) {
		EX_TRACE("Exit tlr_cdev_read -ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	/* Wait for data to appear in the read FIFO. */
	while (stream->reads_completed == stream->reads_consumed) {
		up(&stream->read_mutex);
		if (filp->f_flags & O_NONBLOCK) {
			EX_TRACE("Exit tlr_cdev_read -EAGAIN\n");
			return -EAGAIN;
		}

		/* Wait for the worker loop to put some data into the FIFO. */
		FOP_TRACE("Waiting on read_queue\n");
		if (wait_event_interruptible(stream->read_queue,
					     (stream->reads_completed !=
					      stream->reads_consumed))) {
			EX_TRACE("Exit tlr_cdev_read -ERESTARTSYS\n");
			return -ERESTARTSYS;
		}
		FOP_TRACE("Woke from read_queue\n");

		/* Get the read lock again. */
		if (down_interruptible(&stream->read_mutex)) {
			EX_TRACE("Exit tlr_cdev_read -ERESTARTSYS\n");
			return -ERESTARTSYS;
		}
	}

	/*
	 * At this point we hold the read mutex and we know that there
	 * is at least one buffer of data available.  Copy as many
	 * buffers as possible to userspace.
	 */
	already_read = stream->partial_read_bytes;
	bytes_read = 0;
	for (reads_consumed = stream->reads_consumed;
	     reads_consumed != stream->reads_completed;
	     reads_consumed++) {
		uint32_t index = reads_consumed & (BUFFERS_PER_STREAM - 1);
		size_t buf_remaining =
			stream->read_sizes[index] - already_read;
		size_t to_copy = min(count, buf_remaining);
		int err;

		err = copy_to_user(buf + bytes_read,
				   stream->read_buffers[index] + already_read,
				   to_copy);
		if (err) {
			if (bytes_read > 0)
				break;
			else {
				up(&stream->read_mutex);
				return -EFAULT;
			}
		}
		bytes_read += to_copy;
		count -= to_copy;

		if (to_copy == buf_remaining) {
			/*
			 * We've completely drained that buffer; inval
			 * before reposting.
			 */
			already_read = 0;
		} else {
			/* User only asked for part of the buffer. */
			already_read += to_copy;
			break;
		}
	}
	stream->partial_read_bytes = already_read;

	/* Re-post any buffers that we completely consumed. */
	post_read_buffers(stream, stream->reads_consumed, reads_consumed);
	stream->reads_consumed = reads_consumed;

	spin_lock_irqsave(&stream->h2t_cmd_queue_lock, flags);
	gxpci_check_tile_cmds(queue_state, GXPCI_CHAR_H2T_STREAM);
	spin_unlock_irqrestore(&stream->h2t_cmd_queue_lock, flags);

	spin_lock_irqsave(&stream->h2t_cmd_queue_lock, flags);
	dma_cmds_completed =
		queue_state->fetch_dmas_completed;
	hw_cnt = gxio_trio_read_dma_queue_complete_count(dma_queue_desc);
	batch_completions = (uint16_t) (hw_cnt - (dma_cmds_completed &
		TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));

	if (batch_completions) {
		index_start = queue_state->fetch_dmas_completed;
		index_end = index_start + batch_completions;
		for (j = index_start; j != index_end; j++)
			queue_state->host_cmds_posted +=
				queue_state->host_cmds_cnt_array[j &
					(PCIE_CMD_QUEUE_ENTRIES - 1)];

		queue_state->fetch_dmas_completed += batch_completions;

		gxpci_parse_host_cmds(queue_state, GXPCI_CHAR_H2T_STREAM);
	}
	spin_unlock_irqrestore(&stream->h2t_cmd_queue_lock, flags);

	spin_lock_irqsave(&stream->h2t_cmd_queue_lock, flags);
	gxpci_process_host_cmds(dma_queue_data, queue_state,
				GXPCI_CHAR_H2T_STREAM);
	spin_unlock_irqrestore(&stream->h2t_cmd_queue_lock, flags);

	up(&stream->read_mutex);
	EX_TRACE("Exit tlr_cdev_read %d\n", (int) bytes_read);

	return bytes_read;
}


static ssize_t tlr_cdev_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *f_pos)
{
	struct tlr_stream *stream = filp->private_data;
	struct gxpci_queue_state *queue_state = &stream->t2h_queue_state;
	size_t bytes_written, written;
	uint32_t writes_posted;
	unsigned long flags;
	gxio_trio_dma_queue_t *dma_queue_data =
		&stream->t2h_dma_resource.push_dma_queue_data;
	gxio_trio_dma_queue_t *dma_queue_desc =
		&stream->t2h_dma_resource.pull_dma_queue_desc;
	unsigned int dma_cmds_completed, j, index_start, index_end;
	volatile uint16_t hw_cnt;
	uint16_t batch_completions;

	FOP_TRACE("Entered tlr_cdev_write\n");

	if (count == 0) {
		EX_TRACE("Exit tlr_cdev_write\n");
		return 0;
	}

	/* Grab the stream write lock. */
	if (down_interruptible(&stream->write_mutex)) {
		EX_TRACE("Exit tlr_cdev_write\n");
		return -ERESTARTSYS;
	}

	/*
	 * Wait for a NULL write_buffer, indicating we can allocate and fill
	 * a new one.
	 */
	while ((stream->writes_posted - stream->writes_completed)
	       >= BUFFERS_PER_STREAM) {
		up(&stream->write_mutex);
		if (filp->f_flags & O_NONBLOCK) {
			EX_TRACE("Exit tlr_cdev_write -EAGAIN\n");
			return -EAGAIN;
		}

		/*
		 * Wait for the worker loop to indicate that we're ready
		 * for a new buffer.
		 */
		FOP_TRACE("Waiting on write_queue\n");
		if (wait_event_interruptible(stream->write_queue,
					     ((stream->writes_posted -
					       stream->writes_completed) <
					      BUFFERS_PER_STREAM))) {
			EX_TRACE("Exit tlr_cdev_write -ERESTARTSYS\n");
			return -ERESTARTSYS;
		}
		FOP_TRACE("Woke from write_queue\n");

		/* Get the write lock again. */
		if (down_interruptible(&stream->write_mutex)) {
			EX_TRACE("Exit tlr_cdev_write -ERESTARTSYS\n");
			return -ERESTARTSYS;
		}
	}

	/*
	 * At this point we hold the write mutex and we know that
	 * there is at least one write buffer available.  Copy as much
	 * data as possible into buffers...
	 */
	written = 0;
	bytes_written = 0;
	for (writes_posted = stream->writes_posted;
	     writes_posted - stream->writes_completed < BUFFERS_PER_STREAM;
	     writes_posted++) {
		uint32_t index = writes_posted & (BUFFERS_PER_STREAM - 1);
		size_t size = min(count, (size_t) BUFFER_SIZE);

		if (size == 0)
			break;

		stream->write_sizes[index] = size;
		if (copy_from_user(stream->write_buffers[index],
				   buf + written, size)) {
			if (written > 0)
				break;
			else {
				up(&stream->write_mutex);
				return -EFAULT;
			}
		}
		written += size;
		count -= size;
	}

	/* ...and then post the buffers. */
	post_write_buffers(stream, stream->writes_posted, writes_posted);
	stream->writes_posted = writes_posted;

	spin_lock_irqsave(&stream->t2h_cmd_queue_lock, flags);
	gxpci_check_tile_cmds(queue_state, GXPCI_CHAR_T2H_STREAM);
	spin_unlock_irqrestore(&stream->t2h_cmd_queue_lock, flags);

	spin_lock_irqsave(&stream->t2h_cmd_queue_lock, flags);
	dma_cmds_completed =
		queue_state->fetch_dmas_completed;
	hw_cnt = gxio_trio_read_dma_queue_complete_count(dma_queue_desc);
	batch_completions = (uint16_t) (hw_cnt - (dma_cmds_completed &
		TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));

	if (batch_completions) {
		index_start = queue_state->fetch_dmas_completed;
		index_end = index_start + batch_completions;
		for (j = index_start; j != index_end; j++)
			queue_state->host_cmds_posted +=
				queue_state->host_cmds_cnt_array[j &
				(PCIE_CMD_QUEUE_ENTRIES - 1)];

		queue_state->fetch_dmas_completed += batch_completions;

		gxpci_parse_host_cmds(queue_state, GXPCI_CHAR_T2H_STREAM);
	}
	spin_unlock_irqrestore(&stream->t2h_cmd_queue_lock, flags);

	spin_lock_irqsave(&stream->t2h_cmd_queue_lock, flags);
	gxpci_process_host_cmds(dma_queue_data, queue_state,
				GXPCI_CHAR_T2H_STREAM);
	spin_unlock_irqrestore(&stream->t2h_cmd_queue_lock, flags);

	up(&stream->write_mutex);
	EX_TRACE("Exit tlr_cdev_write %d\n", (int) written);

	return written;
}


static long tlr_cdev_ioctl(struct file *filp,
			   unsigned int cmd, unsigned long arg)
{
	struct tlr_stream *stream = filp->private_data;
#if 0
	struct tlr_pcie_dev *tlr = stream->dev;
	int res;
	u32 channel;
#endif

	switch (cmd) {
	case TILEPCI_IOC_CHANNEL_RESET:
#if 0
		/* Reset the two channels associated with this stream. */

		channel = stream->index;
		res = hv_dev_pwrite(tlr->hv_channel_ctl_fd, 0,
			    (HV_VirtAddr)&channel, sizeof(channel),
			    PCIE_CHANNEL_CTL_CHANNEL_RESET_OFF);
		if (res != sizeof(channel)) {
			ERR("Tile channel reset failed, %d\n", channel);
			return -EIO;
		}

		channel = stream->index + PCIE_HOST_TO_TILE_CHANNELS;
		res = hv_dev_pwrite(tlr->hv_channel_ctl_fd, 0,
			    (HV_VirtAddr)&channel, sizeof(channel),
			    PCIE_CHANNEL_CTL_CHANNEL_RESET_OFF);
		if (res != sizeof(channel)) {
			ERR("Tile channel reset failed, %d\n", channel);
			return -EIO;
		}
#endif
		stream->need_write_soc = 1;
		stream->need_read_soc = 1;

		break;
	default:
		return -EINVAL;
	}
	return 0;
}


#ifdef CONFIG_COMPAT
static long tlr_cdev_compat_ioctl(struct file *filp,
				  unsigned int a, unsigned long b)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return tlr_cdev_ioctl(filp, a, (int)(long)b);
}
#endif


static unsigned int tlr_cdev_poll(struct file *filp, poll_table *table)
{
	struct tlr_stream *stream = filp->private_data;
	unsigned int mask = 0;

	FOP_TRACE("Entered tlr_cdev_poll\n");

	/* Add wait queues to the poll table; we don't actually wait here. */
	poll_wait(filp, &stream->read_queue, table);
	poll_wait(filp, &stream->write_queue, table);

	/*
	 * Grab both the read and write semaphores so that this operation is
	 * ordered with respect to any other processes that may be reading
	 * or writing.  Are we allowed to return -ERESTARTSYS here?  Can't
	 * seem to find the appropriate documentation...
	 */
	if (down_interruptible(&stream->read_mutex)) {
		EX_TRACE("Exit tlr_cdev_poll\n");
		return -ERESTARTSYS;
	}
	if (down_interruptible(&stream->write_mutex)) {
		up(&stream->read_mutex);
		EX_TRACE("Exit tlr_cdev_poll\n");
		return -ERESTARTSYS;
	}

	if (stream->reads_consumed != stream->reads_completed)
		mask |= (POLLIN | POLLRDNORM); /* readable */
	if ((stream->writes_posted - stream->writes_completed) <
	    BUFFERS_PER_STREAM)
		mask |= (POLLOUT | POLLWRNORM); /* writable */

	up(&stream->write_mutex);
	up(&stream->read_mutex);

	EX_TRACE("Exit tlr_cdev_poll\n");
	return mask;
}


static int tlr_cdev_open(struct inode *inode, struct file *filp);


static const struct file_operations tlr_cdev_ops = {
	.owner = THIS_MODULE,
	.open = tlr_cdev_open,
	.read = tlr_cdev_read,
	.write = tlr_cdev_write,
	.unlocked_ioctl = tlr_cdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tlr_cdev_compat_ioctl,
#endif
	.poll = tlr_cdev_poll,
};


static int tlr_cdev_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	struct tlr_stream *stream;
	int stream_index;

	/* Set the private data to point at our stream. */
	stream_index = MINOR(inode->i_rdev) - FIRST_PUBLIC_MINOR;

	stream = tlr_get_stream(tlr, stream_index);
	if (stream == NULL) {
		WARNING("Char stream %d is not initialized\n", stream_index);
		return -ENXIO;
	}

	filp->private_data = stream;

	/* Use the stream read, write, etc. */
	filp->f_op = &tlr_cdev_ops;

	FOP_TRACE("Enter tlr_cdev_open\n");

	/*
	 * Initialize the stream if this is the first time we've
	 * opened it.
	 */
	if (down_interruptible(&stream->read_mutex))
		return -ERESTARTSYS;
	if (down_interruptible(&stream->write_mutex)) {
		up(&stream->read_mutex);
		return -ERESTARTSYS;
	}

	if (stream->is_ready == 0)
		result = init_stream(stream);

	up(&stream->write_mutex);
	up(&stream->read_mutex);

	EX_TRACE("Exit tlr_cdev_open\n");
	return result;
}


static long ep_pq_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	struct gxpci_queue_pair_status *queue_status = filp->private_data;

	switch (cmd) {
	case TILEPCI_IOC_ACTIVATE_PACKET_QUEUE:

		__gxio_mmio_write32(&queue_status->tile_queue_status,
				    GXPCI_CHAN_RUNNING);

		break;
	default:
		return -EINVAL;
	}
	return 0;
}


#ifdef CONFIG_COMPAT
static long ep_pq_compat_ioctl(struct file *filp,
			       unsigned int a, unsigned long b)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return ep_pq_ioctl(filp, a, (int) (long) b);
}
#endif


static int ep_pq_release(struct inode *inode, struct file *filp)
{
	struct gxpci_queue_pair_status *queue_pair_status;
	int sleep_msecs = 0;
	int err = 0;

	queue_pair_status = filp->private_data;

	/*
	 * We simply set the queue status to GXPCI_CHAN_RESET informing
	 * the queue status monitor running on the RC node of our intention
	 * to reset the queue. The RC queue status monitor, upon detecting
	 * our reset, sets the queue peer's user-visiable queue status to
	 * to GXPCI_CHAN_RESET. The host user application, upon detecting
	 * GXPCI_CHAN_RESET, will call close() which sets GXPCI_CHAN_RESET
	 * to its queue status in the queue status array. When the queue
	 * status monitor detects GXPCI_CHAN_RESET for both queues' status,
	 * it sets GXPCI_CHAN_RESET_ACK to both queues' status. We'll be
	 * unblocked then and return. If the peer doesn't call close() in
	 * a timely manner or not at all, we just time out and return.
	 */
	__gxio_mmio_write32(&queue_pair_status->tile_queue_status,
			    GXPCI_CHAN_RESET);

	while (__gxio_mmio_read32(&queue_pair_status->tile_queue_status) !=
	       GXPCI_CHAN_RESET_ACK) {
		if (sleep_msecs >= GXPCI_RELEASE_WAIT_MAX_MSECS) {
			err = -ERESTARTSYS;
			break;
		}

		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL)) {
			/*
			 * This tile application is exiting due to a signal.
			 * We simply send the reset event to host queue
			 * status monitor directly.
			 */
			__gxio_mmio_write32(
				&queue_pair_status->host_queue_status,
				GXPCI_CHAN_RESET);
			err = -EINTR;
			break;
		}
		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}

	/* Set the queue status to uninitialized state. */
	__gxio_mmio_write32(&queue_pair_status->tile_queue_status,
			    GXPCI_CHAN_UNINITIALIZED);
	__gxio_mmio_write32(&queue_pair_status->tile_queue_opened, 0);

	return err;
}


static const struct file_operations ep_pq_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ep_pq_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ep_pq_compat_ioctl,
#endif
	.release = ep_pq_release,
};


static int ep_pq_open(struct inode *inode, struct file *filp)
{
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	struct gxpci_queue_pair_status *queue_status;
	int queue_index;

	/*
	 * Stop if the queue status array is not mapped, indicating
	 * handshake breakdown between the Gx and the host.
	 */
	if (tlr->queue_sts_array == NULL) {
		pr_err("%s: %s handshake failure.\n", driver_name, __func__);
		return -ENODEV;
	}

	/* Locate the queue status struct. */
	if (minor >= TILEPCI_FIRST_PQ_T2H_MINOR) {
		queue_index = minor - TILEPCI_FIRST_PQ_T2H_MINOR;
		queue_status = &tlr->queue_sts_array->pq_sts_t2h[queue_index];
	} else {
		queue_index = minor - TILEPCI_FIRST_PQ_H2T_MINOR;
		queue_status = &tlr->queue_sts_array->pq_sts_h2t[queue_index];
	}
	__gxio_mmio_write32(&queue_status->tile_queue_opened, 1);
	filp->private_data = queue_status;
	filp->f_op = &ep_pq_ops;

	return 0;
}

static long vf_ep_pq_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	struct inode *inode = filp->private_data;
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	struct gxpci_queue_status_array *queue_status_array;
	struct gxpci_queue_pair_status *queue_status;
	struct gxpci_ep_vf_state *vf_state;
	int pio = 0;
	int queue;
	int vf;

	if (minor >= TILEPCI_FIRST_VF_PQ_T2H_MINOR) {
		queue = minor - TILEPCI_FIRST_VF_PQ_T2H_MINOR;
		vf = queue / GXPCI_HOST_PQ_VF_T2H_COUNT;
		queue = queue % GXPCI_HOST_PQ_VF_T2H_COUNT;
	} else {
		queue = minor - TILEPCI_FIRST_VF_PQ_H2T_MINOR;
		vf = queue / GXPCI_HOST_PQ_VF_H2T_COUNT;
		queue = queue % GXPCI_HOST_PQ_VF_H2T_COUNT;
	}
	vf_state = &tlr->vfs[vf];

	switch (cmd) {
	case TILEPCI_IOC_ACTIVATE_PACKET_QUEUE:

		if (!arg)	
			return -EINVAL;

		vf_state->queue_sts_bus_addr = arg;

		/*
		 * Allocate and map a PIO region so that we can access
		 * the queue-pair status array in the RC host memory.
		 * The mutex is used to serialize the PIO allocation and
		 * usage, which is always temporary for VFs.
		 */
		down(&tlr->pio_sem);

		pio = create_pio_mapping(tlr, (void **)&queue_status_array,
			vf_state->queue_sts_bus_addr,	
			sizeof(struct gxpci_queue_status_array),
			vf_state->instance + 1);
		if (pio < 0) {
			up(&tlr->pio_sem);
			pr_err("%s: vf_ep_pq_ioctl create_pio_mapping err.\n",
				driver_name);
			return -EFAULT;
		}

		if (minor >= TILEPCI_FIRST_VF_PQ_T2H_MINOR)
			queue_status = &queue_status_array->pq_sts_t2h[queue];
		else
			queue_status = &queue_status_array->pq_sts_h2t[queue];

		__gxio_mmio_write32(&queue_status->tile_queue_opened, 1);
		__gxio_mmio_write32(&queue_status->tile_queue_status,
				    GXPCI_CHAN_RUNNING);

		/* Release the mapping and the PIO region. */
		free_pio_mapping(tlr, queue_status_array, pio);

		up(&tlr->pio_sem);

		break;
	default:

		return -EINVAL;
	}
	return 0;
}


#ifdef CONFIG_COMPAT
static long vf_ep_pq_compat_ioctl(struct file *filp,
				  unsigned int a, unsigned long b)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return vf_ep_pq_ioctl(filp, a, (int) (long) b);
}
#endif


static int vf_ep_pq_release(struct inode *inode, struct file *filp)
{
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	struct gxpci_queue_status_array *queue_status_array;
	struct gxpci_queue_pair_status *queue_pair_status;
	struct gxpci_ep_vf_state *vf_state;
	int sleep_msecs = 0;
	uint32_t pio;
	int queue;
	int err;
	int vf;

	if (minor >= TILEPCI_FIRST_VF_PQ_T2H_MINOR) {
		queue = minor - TILEPCI_FIRST_VF_PQ_T2H_MINOR;
		vf = queue / GXPCI_HOST_PQ_VF_T2H_COUNT;
		queue = queue % GXPCI_HOST_PQ_VF_T2H_COUNT;
	} else {
		queue = minor - TILEPCI_FIRST_VF_PQ_H2T_MINOR;
		vf = queue / GXPCI_HOST_PQ_VF_H2T_COUNT;
		queue = queue % GXPCI_HOST_PQ_VF_H2T_COUNT;
	}
	vf_state = &tlr->vfs[vf];

	/*
	 * If this is called before the TILEPCI_IOC_ACTIVATE_PACKET_QUEUE
	 * ioctl is, just return.
	 */
	if (!vf_state->queue_sts_bus_addr)	
		return 0;

	/*
	 * Allocate and map a PIO region so that we can access
	 * the queue-pair status array in the RC host memory.
	 * The mutex is used to serialize the PIO allocation and
	 * usage, which is always temporary for VFs.
	 */
	down(&tlr->pio_sem);

	err = create_pio_mapping(tlr, (void **)&queue_status_array,
		vf_state->queue_sts_bus_addr,	
		sizeof(struct gxpci_queue_status_array),
		vf_state->instance + 1);
	if (err < 0) {
		up(&tlr->pio_sem);
		pr_err("%s: vf_ep_pq_release create_pio_mapping failure.\n",
			driver_name);
		return -EFAULT;
	}

	pio = err;

	if (minor >= TILEPCI_FIRST_VF_PQ_T2H_MINOR)
		queue_pair_status = &queue_status_array->pq_sts_t2h[queue];
	else
		queue_pair_status = &queue_status_array->pq_sts_h2t[queue];

	/*
	 * We simply set the queue status to GXPCI_CHAN_RESET informing
	 * the queue status monitor running on the RC node of our intention
	 * to reset the queue. The RC queue status monitor, upon detecting
	 * our reset, sets the queue peer's user-visiable queue status to
	 * to GXPCI_CHAN_RESET. The host user application, upon detecting
	 * GXPCI_CHAN_RESET, will call close() which sets GXPCI_CHAN_RESET
	 * to its queue status in the queue status array. When the queue
	 * status monitor detects GXPCI_CHAN_RESET for both queues' status,
	 * it sets GXPCI_CHAN_RESET_ACK to both queues' status. We'll be
	 * unblocked then and return. If the peer doesn't call close() in
	 * a timely manner or not at all, we just time out and return.
	 */
	__gxio_mmio_write32(&queue_pair_status->tile_queue_status,
			    GXPCI_CHAN_RESET);

	while (__gxio_mmio_read32(&queue_pair_status->tile_queue_status) !=
	       GXPCI_CHAN_RESET_ACK) {
		if (sleep_msecs >= GXPCI_RELEASE_WAIT_MAX_MSECS) {
			err = -ERESTARTSYS;
			break;
		}

		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL)) {
			/*
			 * This tile application is exiting due to a signal.
			 * We simply send the reset event to host queue
			 * status monitor directly.
			 */
			__gxio_mmio_write32(
				&queue_pair_status->host_queue_status,
				GXPCI_CHAN_RESET);
			err = -EINTR;
			break;
		}
		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}

	/* Set the queue status to uninitialized state. */
	__gxio_mmio_write32(&queue_pair_status->tile_queue_status,
			    GXPCI_CHAN_UNINITIALIZED);
	__gxio_mmio_write32(&queue_pair_status->tile_queue_opened, 0);

	/* Release the mapping and the PIO region. */
	free_pio_mapping(tlr, queue_status_array, pio);

	up(&tlr->pio_sem);

	return err;
}


static const struct file_operations vf_ep_pq_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vf_ep_pq_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vf_ep_pq_compat_ioctl,
#endif
	.release = vf_ep_pq_release,
};


static int vf_ep_pq_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode;
	filp->f_op = &vf_ep_pq_ops;

	if (!enable_vf_pq)
		return -ENODEV;

	return 0;
}


static long ep_raw_dma_ioctl(struct file *filp,
			     unsigned int cmd, unsigned long arg)
{
	struct gxpci_queue_pair_status *queue_status = filp->private_data;

	switch (cmd) {
	case TILEPCI_IOC_ACTIVATE_RAW_DMA:

		__gxio_mmio_write32(&queue_status->tile_queue_status,
				    GXPCI_CHAN_RUNNING);

		break;
	default:
		return -EINVAL;
	}
	return 0;
}


#ifdef CONFIG_COMPAT
static long ep_raw_dma_compat_ioctl(struct file *filp,
				    unsigned int a, unsigned long b)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return ep_raw_dma_ioctl(filp, a, (int) (long) b);
}
#endif


static int ep_raw_dma_release(struct inode *inode, struct file *filp)
{
	struct gxpci_queue_pair_status *queue_pair_status;
	int sleep_msecs = 0;
	int err = 0;

	queue_pair_status = filp->private_data;

	/*
	 * We simply set the queue status to GXPCI_CHAN_RESET informing
	 * the queue status monitor running on the RC node of our intention
	 * to reset the queue. The RC queue status monitor, upon detecting
	 * our reset, sets the queue peer's user-visiable queue status to
	 * to GXPCI_CHAN_RESET. The host user application, upon detecting
	 * GXPCI_CHAN_RESET, will call close() which sets GXPCI_CHAN_RESET
	 * to its queue status in the queue status array. When the queue
	 * status monitor detects GXPCI_CHAN_RESET for both queues' status,
	 * it sets GXPCI_CHAN_RESET_ACK to both queues' status. We'll be
	 * unblocked then and return. If the peer doesn't call close() in
	 * a timely manner or not at all, we just time out and return.
	 */
	__gxio_mmio_write32(&queue_pair_status->tile_queue_status,
			    GXPCI_CHAN_RESET);

	while (__gxio_mmio_read32(&queue_pair_status->tile_queue_status) !=
	       GXPCI_CHAN_RESET_ACK) {
		if (sleep_msecs >= GXPCI_RELEASE_WAIT_MAX_MSECS) {
			err = -ERESTARTSYS;
			break;
		}

		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL)) {
			/*
			 * This tile application is exiting due to a signal.
			 * We simply send the reset event to host queue
			 * status monitor directly.
			 */
			__gxio_mmio_write32(
				&queue_pair_status->host_queue_status,
				GXPCI_CHAN_RESET);
			err = -EINTR;
			break;
		}
		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}

	/* Set the queue status to uninitialized state. */
	__gxio_mmio_write32(&queue_pair_status->tile_queue_status,
			    GXPCI_CHAN_UNINITIALIZED);
	__gxio_mmio_write32(&queue_pair_status->tile_queue_opened, 0);

	return err;
}


static const struct file_operations ep_raw_dma_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ep_raw_dma_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ep_raw_dma_compat_ioctl,
#endif
	.release = ep_raw_dma_release,
};


static int ep_raw_dma_open(struct inode *inode, struct file *filp)
{
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	struct gxpci_queue_pair_status *queue_status;
	int queue_index;

	/*
	 * Stop if the queue status array is not mapped, indicating
	 * handshake breakdown between the Gx and the host.
	 */
	if (tlr->queue_sts_array == NULL) {
		pr_err("%s: %s handshake failure.\n", driver_name, __func__);
		return -ENODEV;
	}

	/* Locate the queue status struct. */
	if (minor >= TILEPCI_FIRST_RAW_DMA_TX_MINOR) {
		queue_index = minor - TILEPCI_FIRST_RAW_DMA_TX_MINOR;
		queue_status = &tlr->queue_sts_array->rd_sts_t2h[queue_index];
	} else {
		queue_index = minor - TILEPCI_FIRST_RAW_DMA_RX_MINOR;
		queue_status = &tlr->queue_sts_array->rd_sts_h2t[queue_index];
	}
	__gxio_mmio_write32(&queue_status->tile_queue_opened, 1);
	filp->private_data = queue_status;
	filp->f_op = &ep_raw_dma_ops;

	return 0;
}


static int ep_nic_release(struct inode *inode, struct file *filp)
{
	struct gxpci_queue_pair_status *queue_pair_status;
	int sleep_msecs = 0;

	queue_pair_status = filp->private_data;

	/*
	 * We simply set the queue status to GXPCI_CHAN_RESET informing
	 * the queue status monitor running on the RC node of our intention
	 * to reset the queue. The RC queue status monitor, upon detecting
	 * our reset, sets the queue peer's NIC driver queue status to
	 * to GXPCI_CHAN_RESET. The host VNIC driver, upon detecting
	 * GXPCI_CHAN_RESET, will call dev_close() and set GXPCI_CHAN_RESET
	 * to its queue status in the queue status array. When the queue
	 * status monitor detects GXPCI_CHAN_RESET for both queues' status,
	 * it sets GXPCI_CHAN_RESET_ACK to both queues' status. We'll be
	 * unblocked then and return.
	 */
	__gxio_mmio_write32(&queue_pair_status->tile_queue_status,
			    GXPCI_CHAN_RESET);

	while (__gxio_mmio_read32(&queue_pair_status->tile_queue_status) !=
	       GXPCI_CHAN_RESET_ACK) {
		if (sleep_msecs >= GXPCI_RELEASE_WAIT_MAX_MSECS)
			return -ERESTARTSYS;
		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL))
			return -EINTR;
		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}

	/* Set the queue status to uninitialized state. */
	__gxio_mmio_write32(&queue_pair_status->tile_queue_status,
			    GXPCI_CHAN_UNINITIALIZED);

	return 0;
}


static const struct file_operations ep_nic_ops = {
	.owner = THIS_MODULE,
	.release = ep_nic_release,
};


static int ep_nic_open(struct inode *inode, struct file *filp)
{
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	struct gxpci_queue_pair_status *queue_status;
	int queue_index;

	/*
	 * Stop if the queue status array is not mapped, indicating
	 * handshake breakdown between the Gx and the host.
	 */
	if (tlr->queue_sts_array == NULL) {
		pr_err("%s: %s handshake failure.\n", driver_name, __func__);
		return -ENODEV;
	}

	/* Locate the queue status struct. */
	queue_index = minor - TILEPCI_FIRST_NIC_MINOR;
	queue_status = &tlr->queue_sts_array->nic_sts[queue_index];
	filp->private_data = queue_status;
	filp->f_op = &ep_nic_ops;

	return 0;
}


static long ep_c2c_ioctl(struct file *filp,
			 unsigned int cmd, unsigned long arg)
{
	struct gxpci_ep_c2c_state *queue_state = filp->private_data;
	struct gxpci_c2c_queue_sts *queue_pair_status;
	tilegxpci_c2c_activate_queue_t activate_queue;
	uint32_t receiver_link_index;
	uint32_t sender_link_index;
	uint32_t sender_ready;

	queue_pair_status = queue_state->queue_pair_status;

	switch (cmd) {
	case TILEPCI_IOC_C2C_ACTIVATE_QUEUE:
		if (copy_from_user(&activate_queue, (void __user *) arg,
				   sizeof(tilegxpci_c2c_activate_queue_t)))
			return -EFAULT;

		sender_link_index = activate_queue.sender_link_index;
		receiver_link_index = activate_queue.receiver_link_index;

		if (sender_link_index > MAX_PCIE_PORTS_PER_DOMAIN ||
		    receiver_link_index > MAX_PCIE_PORTS_PER_DOMAIN)
				return -EINVAL;

		__gxio_mmio_write32(&queue_pair_status->sender_link_index,
				    sender_link_index);
		__gxio_mmio_write32(&queue_pair_status->receiver_link_index,
				    receiver_link_index);
		__gxio_mmio_write32(&queue_pair_status->sender_status,
				    GXPCI_CHAN_RUNNING);
		__gxio_mmio_write32(&queue_pair_status->receiver_status,
				    GXPCI_CHAN_RUNNING);

		/*
		 * Activate the queue so that the C2C queue monitor can
		 * start monitoring the queue which is deactivated by
		 * the monitor.
		 */
		__gxio_mmio_write32(&queue_pair_status->active, 1);

		break;
	case TILEPCI_IOC_C2C_SET_SENDER_READY:
		__gxio_mmio_write32(&queue_pair_status->sender_status,
				    GXPCI_TILE_CHAN_READY);

		break;
	case TILEPCI_IOC_C2C_GET_SENDER_READY:
		sender_ready =
			__gxio_mmio_read32(&queue_pair_status->sender_status);
		if (copy_to_user((void __user *) arg, &sender_ready,
			sizeof(uint32_t)))
			return -EFAULT;

		break;
	default:
		return -EINVAL;
	}
	return 0;
}


#ifdef CONFIG_COMPAT
static long ep_c2c_compat_ioctl(struct file *filp,
				unsigned int a, unsigned long b)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return ep_c2c_ioctl(filp, a, (int)(long)b);
}
#endif


static void ep_c2c_vma_open(struct vm_area_struct *vma)
{
	struct gxpci_ep_c2c_state *queue_state = vma->vm_private_data;

	down(&queue_state->mutex);

	queue_state->vmas++;

	up(&queue_state->mutex);
}


static void ep_c2c_vma_close(struct vm_area_struct *vma)
{
	struct gxpci_ep_c2c_state *queue_state = vma->vm_private_data;

	down(&queue_state->mutex);

	queue_state->vmas--;

	up(&queue_state->mutex);
}


static int ep_c2c_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct gxpci_ep_c2c_state *queue_state = vma->vm_private_data;
	struct tlr_pcie_dev *ep_dev = queue_state->ep_dev;
	struct page *page;
	void *page_ptr = NULL;
	unsigned long offset;

	unsigned long vaddr = (unsigned long) vmf->virtual_address;
	int ret = VM_FAULT_SIGBUS;

	down(&queue_state->mutex);

	if (vaddr > vma->vm_end)
		goto exit;

	/* Ignore vma->vm_pgoff, which is unrelated here. */
	offset = vaddr - vma->vm_start;

	page_ptr = ep_dev->bar_mem + offset;

	page = virt_to_page(page_ptr);
	get_page(page);

	ret = 0;
	vmf->page = page;

 exit:
	up(&queue_state->mutex);

	return ret;
}


static struct vm_operations_struct ep_c2c_vm_ops = {
	.open	= ep_c2c_vma_open,
	.close	= ep_c2c_vma_close,
	.fault  = ep_c2c_vma_fault,
};


static int ep_c2c_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct gxpci_ep_c2c_state *queue_state = filp->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset != 0) {
		ERR("Tile PCIe c2c mmap offset invalid\n");
		return -EINVAL;
	}

	if (!(vma->vm_flags & VM_SHARED)) {
		ERR("Tile PCIe c2c mmap flags must include VM_SHARED\n");
		return -EINVAL;
	}

	/* Don't try to swap out physical pages. */
	vma->vm_flags |= VM_RESERVED;

	vma->vm_ops = &ep_c2c_vm_ops;
	vma->vm_private_data = (void *)queue_state;

	ep_c2c_vma_open(vma);

	return 0;
}


static int ep_c2c_release(struct inode *inode, struct file *filp)
{
	struct gxpci_ep_c2c_state *queue_state = filp->private_data;
	struct gxpci_c2c_queue_sts *queue_pair_status;
	unsigned int minor = MINOR(inode->i_rdev);
	volatile uint32_t *queue_status;
	uint32_t *peer_status;
	int sleep_msecs = 0;
	int err = 0;

	queue_pair_status = queue_state->queue_pair_status;

	if (minor >= TILEPCI_FIRST_C2C_TX_MINOR) {
		queue_status = &queue_pair_status->sender_status;
		peer_status = &queue_pair_status->receiver_status;
	} else {
		queue_status = &queue_pair_status->receiver_status;
		peer_status = &queue_pair_status->sender_status;
	}

	/*
	 * We simply set the queue status to GXPCI_CHAN_RESET informing
	 * the queue status monitor running on the RC node of our intention
	 * to reset the queue. The RC queue status monitor, upon detecting
	 * our reset, sets the queue peer's C2C driver queue status to
	 * to GXPCI_CHAN_RESET. The peer application, upon detecting
	 * GXPCI_CHAN_RESET, will call close() which sets GXPCI_CHAN_RESET
	 * to its queue status in the queue status array. When the queue
	 * status monitor detects GXPCI_CHAN_RESET for both queues' status,
	 * it sets GXPCI_CHAN_RESET_ACK to both queues' status. We'll be
	 * unblocked then and return.
	 */
	__gxio_mmio_write32(queue_status, GXPCI_CHAN_RESET);

	while (__gxio_mmio_read32(queue_status) != GXPCI_CHAN_RESET_ACK) {
		if (sleep_msecs >= GXPCI_RELEASE_WAIT_MAX_MSECS) {
			err = -ERESTARTSYS;
			break;
		}

		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL)) {
			/*
			 * This application is exiting due to a signal.
			 * We simply send the reset event to peer queue
			 * status monitor directly.
			 */
			__gxio_mmio_write32(peer_status, GXPCI_CHAN_RESET);
			err = -EINTR;
			break;
		}
		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}

	/* Set the queue status to uninitialized state. */
	__gxio_mmio_write32(queue_status, GXPCI_CHAN_UNINITIALIZED);
	
	return err;
}


static const struct file_operations ep_c2c_ops = {
	.owner = THIS_MODULE,
	.mmap		= ep_c2c_mmap,
	.unlocked_ioctl = ep_c2c_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ep_c2c_compat_ioctl,
#endif
	.release = ep_c2c_release,
};


static int ep_c2c_open(struct inode *inode, struct file *filp)
{
	struct tlr_pcie_dev *ep_dev =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	struct gxpci_c2c_queue_sts *queue_status;
	struct gxpci_ep_c2c_state *queue_state;
	struct gxpci_host_regs *regs;
	int queue_index;

	regs = (struct gxpci_host_regs *)ep_dev->bar_mem;

	/* No C2C queue is allowed if there is no C2C queue status array. */
	if (ep_dev->c2c_queue_sts_array == NULL)
		return -ENODEV;

	/* Locate the queue status struct. */
	if (minor >= TILEPCI_FIRST_C2C_TX_MINOR) {
		queue_index = minor - TILEPCI_FIRST_C2C_TX_MINOR;
		queue_state = &ep_dev->c2c_send[queue_index];

		/* Initialize the local queue status flag. */
		regs->c2c_send_status[queue_index].status =
			GXPCI_CHAN_UNINITIALIZED;
	} else {
		queue_index = minor - TILEPCI_FIRST_C2C_RX_MINOR;
		queue_state = &ep_dev->c2c_recv[queue_index];

		/* Initialize the local queue status flag. */
		regs->c2c_recv_status[queue_index].status =
			GXPCI_CHAN_UNINITIALIZED;
	}
	queue_status = &ep_dev->c2c_queue_sts_array->c2c_queue_sts[queue_index];
	queue_state->queue_pair_status = queue_status;
	queue_state->ep_dev = ep_dev;

	filp->private_data = queue_state;
	filp->f_op = &ep_c2c_ops;

	return 0;
}


static long ep_barmem_ioctl(struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
	struct tlr_pcie_dev *ep_dev = filp->private_data;
	tilegxpci_bar_info_t bar_info;
	tilegxpci_get_nic_queue_cfg_t nic_queue_cfg;
	tilegxpci_get_rd_queue_cfg_t rd_queue_cfg;
	tilegxpci_get_pq_queue_msg_t pq_queue_msg;
	tilegxpci_msix_info_t msix_info;
	struct gxpci_host_regs *regs;
	int vf;

	regs = (struct gxpci_host_regs *) ep_dev->bar_mem;

	/* EP should have a non-zero link index. */
	BUG_ON(regs->link_index == 0);

	switch (cmd) {
	case TILEPCI_IOC_GET_BAR:
		if (copy_from_user(&bar_info, (void __user *) arg,
				   sizeof(tilegxpci_bar_info_t)))
			return -EFAULT;

		if (bar_info.link_index ==
		    (uint16_t) TILEGXPCI_LOCAL_LINK_INDEX) {

			bar_info.link_index = regs->link_index;
			if (bar_info.bar_index == 2) {
				bar_info.bar_addr = regs->bar2_addr;
				bar_info.bar_size = regs->bar2_size;
			} else { /* Return BAR0 info by default. */
				bar_info.bar_addr = regs->bar0_addr;
				bar_info.bar_size = regs->bar0_size;
			}
		} else {
			struct gxpci_c2c_queue_sts_array *c2c_sts_array;
			int rem_link = bar_info.link_index;

			if (rem_link >= MAX_PCIE_PORTS_PER_DOMAIN)
				return -EINVAL;

			/*
			 * We don't need the remote BAR's size, just fill
			 * bar_size field with 0 by default.
			 */
			bar_info.bar_size = 0;
			c2c_sts_array = ep_dev->c2c_queue_sts_array;
			if (bar_info.bar_index == 2)
				bar_info.bar_addr =
					c2c_sts_array->bar2_addr[rem_link];
			else /* Return BAR0 info by default. */
				bar_info.bar_addr =
					c2c_sts_array->bar0_addr[rem_link];
		}
		if (copy_to_user((void __user *) arg, &bar_info,
			     sizeof(tilegxpci_bar_info_t)))
			return -EFAULT;

		break;

	case TILEPCI_IOC_GET_MSIX:
		msix_info.msix_vectors = ep_dev->msix_vectors;
		msix_info.msix_cs_q_intr_vec_base =
			ep_dev->msix_cs_q_intr_vec_base;
		msix_info.msix_zc_q_intr_vec_base =
			ep_dev->msix_zc_q_intr_vec_base;
		msix_info.msix_host_nic_intr_vec_base =
			ep_dev->msix_host_nic_intr_vec_base;
		if (copy_to_user((void __user *) arg, &msix_info,
			     sizeof(tilegxpci_msix_info_t)))
			return -EFAULT;

		break;

	case TILEPCI_IOC_GET_NIC_QUEUE_CFG:
		if (copy_from_user(&nic_queue_cfg, (void __user *) arg,
				   sizeof(tilegxpci_get_nic_queue_cfg_t)))
			return -EFAULT;

		if (nic_queue_cfg.trio_index >= TILEGX_NUM_TRIO ||
		    nic_queue_cfg.mac_index >= TILEGX_TRIO_PCIES)
			return -EINVAL;

		nic_queue_cfg.num_ports =
			host_vnic_ports
			  [nic_queue_cfg.trio_index][nic_queue_cfg.mac_index];
		nic_queue_cfg.num_tx_queues =
			host_vnic_tx_queues
			  [nic_queue_cfg.trio_index][nic_queue_cfg.mac_index];
		nic_queue_cfg.num_rx_queues =
			host_vnic_rx_queues
			  [nic_queue_cfg.trio_index][nic_queue_cfg.mac_index];

		if (copy_to_user((void __user *) arg, &nic_queue_cfg,
				 sizeof(tilegxpci_get_nic_queue_cfg_t)))
			return -EFAULT;

		break;

	case TILEPCI_IOC_GET_PQ_QUEUE_MSG:
		if (copy_from_user(&pq_queue_msg, (void __user *) arg,
				   sizeof(tilegxpci_get_pq_queue_msg_t)))
			return -EFAULT;

		if (pq_queue_msg.trio_index >= TILEGX_NUM_TRIO ||
		    pq_queue_msg.mac_index >= TILEGX_TRIO_PCIES)
			return -EINVAL;

		pq_queue_msg.num_h2t_queues =
			host_pq_h2t_queues
			  [pq_queue_msg.trio_index][pq_queue_msg.mac_index];
		pq_queue_msg.num_t2h_queues =
			host_pq_t2h_queues
			  [pq_queue_msg.trio_index][pq_queue_msg.mac_index];

		if (copy_to_user((void __user *) arg, &pq_queue_msg,
				 sizeof(tilegxpci_get_pq_queue_msg_t)))
			return -EFAULT;

		break;

	case TILEPCI_IOC_GET_RD_QUEUE_CFG:
		if (copy_from_user(&rd_queue_cfg, (void __user *) arg,
				   sizeof(tilegxpci_get_rd_queue_cfg_t)))
			return -EFAULT;

		if (rd_queue_cfg.trio_index >= TILEGX_NUM_TRIO ||
		    rd_queue_cfg.mac_index >= TILEGX_TRIO_PCIES)
			return -EINVAL;

		rd_queue_cfg.num_t2h_queues = regs->raw_dma_t2h_queues;
		rd_queue_cfg.num_h2t_queues = regs->raw_dma_h2t_queues;

		if (copy_to_user((void __user *) arg, &rd_queue_cfg,
				 sizeof(tilegxpci_get_rd_queue_cfg_t)))
			return -EFAULT;
		break;

	case TILEPCI_IOC_GET_VF_BAR:
		if (copy_from_user(&bar_info, (void __user *) arg,
				   sizeof(tilegxpci_bar_info_t)))
			return -EFAULT;

		/* Check for invalid virtual function number. */
		vf = bar_info.link_index;
		if (vf >= ep_dev->num_vfs)
			return -EINVAL;

		if (bar_info.bar_index == 2) {
			bar_info.bar_addr = ep_dev->vfs[vf].vf_bar2_address;
			bar_info.bar_size = ep_dev->vf_bar2_size;
		} else { /* Return BAR0 info by default. */
			bar_info.bar_addr = ep_dev->vfs[vf].vf_bar0_address;
			bar_info.bar_size = ep_dev->vf_bar0_size;
		}

		if (copy_to_user((void __user *) arg, &bar_info,
				 sizeof(tilegxpci_bar_info_t)))
			return -EFAULT;

                break;

	case TILEPCI_IOC_VF_GET_PQ_QUEUE_MSG:
		if (copy_from_user(&pq_queue_msg, (void __user *) arg,
				   sizeof(tilegxpci_get_pq_queue_msg_t)))
			return -EFAULT;

		if (!enable_vf_pq)
			return -ENODEV;

		if (pq_queue_msg.trio_index >= TILEGX_NUM_TRIO ||
		    pq_queue_msg.mac_index >= TILEGX_TRIO_PCIES)
			return -EINVAL;

		pq_queue_msg.num_h2t_queues = pq_h2t_queues_in_vf;
		pq_queue_msg.num_t2h_queues = pq_t2h_queues_in_vf;

		if (copy_to_user((void __user *) arg, &pq_queue_msg,
				 sizeof(tilegxpci_get_pq_queue_msg_t)))
			return -EFAULT;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}


#ifdef CONFIG_COMPAT
static long ep_barmem_compat_ioctl(struct file *filp,
				   unsigned int cmd, unsigned long arg)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return ep_barmem_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif


static void ep_barmem_vma_open(struct vm_area_struct *vma)
{
	struct tlr_pcie_dev *ep = (struct tlr_pcie_dev *) vma->vm_private_data;

	down(&ep->mutex);

	ep->vmas++;

	up(&ep->mutex);
}


static void ep_barmem_vma_close(struct vm_area_struct *vma)
{
	struct tlr_pcie_dev *ep = (struct tlr_pcie_dev *) vma->vm_private_data;

	down(&ep->mutex);

	ep->vmas--;

	up(&ep->mutex);
}


static int ep_barmem_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct tlr_pcie_dev *ep = (struct tlr_pcie_dev *) vma->vm_private_data;
	struct page *page;
	void *page_ptr = NULL;
	unsigned long offset;

	unsigned long vaddr = (unsigned long) vmf->virtual_address;
	int ret = VM_FAULT_SIGBUS;

	down(&ep->mutex);

	if (vaddr > vma->vm_end)
		goto exit;

	/* Ignore vma->vm_pgoff, which is unrelated here. */
	offset = vaddr - vma->vm_start;

	page_ptr = ep->msix_table_base + offset;

	page = virt_to_page(page_ptr);
	get_page(page);

	ret = 0;
	vmf->page = page;

 exit:
	up(&ep->mutex);

	return ret;
}


static struct vm_operations_struct ep_barmem_vm_ops = {
	.open	= ep_barmem_vma_open,
	.close	= ep_barmem_vma_close,
	.fault  = ep_barmem_vma_fault,
};


static int ep_barmem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct tlr_pcie_dev *ep_dev = filp->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (ep_dev->msix_table_base == NULL)
		return -EFAULT;

	if (offset != 0) {
		ERR("Tile PCIe barmem mmap offset invalid\n");
		return -EINVAL;
	}

	if (!(vma->vm_flags & VM_SHARED)) {
		ERR("Tile PCIe barmem mmap flags must include VM_SHARED\n");
		return -EINVAL;
	}

	/* Don't try to swap out physical pages. */
	vma->vm_flags |= VM_RESERVED;

	vma->vm_ops = &ep_barmem_vm_ops;
	vma->vm_private_data = (void *) ep_dev;

	ep_barmem_vma_open(vma);

	return 0;
}


static const struct file_operations ep_barmem_ops = {
	.owner		= THIS_MODULE,
	.mmap		= ep_barmem_mmap,
	.unlocked_ioctl = ep_barmem_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = ep_barmem_compat_ioctl,
#endif
};


static int ep_barmem_open(struct tlr_pcie_dev *ep, struct file *filp)
{
	struct page *page = virt_to_page(ep->msix_table_base);
	SetPageReserved(page);

	/*
	 * Increment page count so that the kernel won't
	 * try releasing this page when the application which
	 * has mmap'ed the page exits.
	 */
	get_page(page);

	filp->private_data = ep;
	filp->f_op = &ep_barmem_ops;

	return 0;
}


static int tlr_generic_open(struct inode *inode, struct file *filp)
{
	unsigned int	 minor = MINOR(inode->i_rdev);
	int		 result;
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);

	if ((minor >= FIRST_PUBLIC_MINOR) &&
	    (minor <= LAST_PUBLIC_MINOR)) {
		result = tlr_cdev_open(inode, filp);
	} else if ((minor >= FIRST_ZC_H2T_MINOR) &&
		 (minor <= LAST_ZC_H2T_MINOR)) {
		result = tlr_zc_open(inode, filp);
	} else if ((minor >= FIRST_ZC_T2H_MINOR) &&
		 (minor <= LAST_ZC_T2H_MINOR)) {
		result = tlr_zc_open(inode, filp);
	} else if ((minor >= TILEPCI_FIRST_PQ_H2T_MINOR) &&
		(minor <= (TILEPCI_FIRST_PQ_H2T_MINOR +
		host_pq_h2t_queues[tlr->trio_index][tlr->mac] - 1))) {
		result = ep_pq_open(inode, filp);
	} else if ((minor >= TILEPCI_FIRST_PQ_T2H_MINOR) &&
		(minor <= (TILEPCI_FIRST_PQ_T2H_MINOR +
		host_pq_t2h_queues[tlr->trio_index][tlr->mac] - 1))) {
		result = ep_pq_open(inode, filp);
	} else if ((minor >= TILEPCI_FIRST_VF_PQ_H2T_MINOR) &&
		(minor < (TILEPCI_FIRST_VF_PQ_H2T_MINOR +
		GXPCI_MAX_NUM_VFS * GXPCI_HOST_PQ_VF_H2T_COUNT))) {
		result = vf_ep_pq_open(inode, filp);
	} else if ((minor >= TILEPCI_FIRST_VF_PQ_T2H_MINOR) &&
		(minor < (TILEPCI_FIRST_VF_PQ_T2H_MINOR +
		GXPCI_MAX_NUM_VFS * GXPCI_HOST_PQ_VF_T2H_COUNT))) {
		result = vf_ep_pq_open(inode, filp);
	} else if ((minor >= TILEPCI_FIRST_NIC_MINOR) &&
		(minor <= (TILEPCI_FIRST_NIC_MINOR +
		GXPCI_HOST_NIC_COUNT - 1))) {
		result = ep_nic_open(inode, filp);
	} else if ((minor >= TILEPCI_FIRST_RAW_DMA_RX_MINOR) &&
		(minor <= (TILEPCI_FIRST_RAW_DMA_RX_MINOR +
		GXPCI_RAW_DMA_QUEUE_COUNT - 1))) {
		result = ep_raw_dma_open(inode, filp);
	} else if ((minor >= TILEPCI_FIRST_RAW_DMA_TX_MINOR) &&
		(minor <= (TILEPCI_FIRST_RAW_DMA_TX_MINOR +
		GXPCI_RAW_DMA_QUEUE_COUNT - 1))) {
		result = ep_raw_dma_open(inode, filp);
	} else if ((minor >= TILEPCI_FIRST_C2C_RX_MINOR) &&
		(minor <= (TILEPCI_FIRST_C2C_RX_MINOR +
		GXPCI_C2C_QUEUE_COUNT - 1))) {
		result = ep_c2c_open(inode, filp);
	} else if ((minor >= TILEPCI_FIRST_C2C_TX_MINOR) &&
		(minor <= (TILEPCI_FIRST_C2C_TX_MINOR +
		GXPCI_C2C_QUEUE_COUNT - 1))) {
		result = ep_c2c_open(inode, filp);
	} else if (minor == GXPCI_C2C_BARMEM_MINOR) {
		result = ep_barmem_open(tlr, filp);
	} else {
		result = -ENODEV;
	}

	return result;
}


static const struct file_operations tlr_generic_ops = {
	.owner = THIS_MODULE,
	.open = tlr_generic_open,
};


static int tlr_cdev_setup(struct tlr_pcie_dev *tlr)
{
	struct cdev *cdev = &tlr->cdev;

	/* Allocate some major/minor numbers. */
	dev_t first;
	int err = alloc_chrdev_region(&first, 0, GXPCI_NUM_MINOR_DEVICES,
				      driver_name);
	if (err != 0)
		return err;

	/* Register the device. */
	cdev_init(cdev, &tlr_generic_ops);
	cdev->owner = THIS_MODULE;
	err = cdev_add(cdev, first, GXPCI_NUM_MINOR_DEVICES);
	if (err != 0) {
		unregister_chrdev_region(first, GXPCI_NUM_MINOR_DEVICES);
		return err;
	}
	tlr->first_dev = first;

	return 0;
}


/* Print the association of dev major number, port ID and link index. */
static int tlr_read_ep_major_mac_link(char *buf, char **start, off_t offset,
				      int count, int *eof, void *data)
{
	/* Each line has format "MMM trioX-macX L\n" */
	char scratch[TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES * 40];
	struct tlr_pcie_dev *ep_dev;
	struct gxpci_host_regs *regs;
	char *next = scratch;
	int result = 0;
	int i;

	for (i = 0; i < num_ep_devs; i++) {
		int bytes;

		ep_dev = &pcie_ep_devs[i];
		regs = (struct gxpci_host_regs *) ep_dev->bar_mem;

		/*
		 * Skip this EP port if the link is down.
		 */
		if (regs == NULL)
			continue;

		if (ep_dev->link_index == 0)
			ep_dev->link_index = regs->link_index;

		if (ep_dev->link_index == 0)
			bytes = sprintf(next, "%d trio%d-mac%d <unknown>\n",
					MAJOR(ep_dev->first_dev),
					ep_dev->trio_index, ep_dev->mac);
		else
			bytes = sprintf(next, "%d trio%d-mac%d %d\n",
					MAJOR(ep_dev->first_dev),
					ep_dev->trio_index,
					ep_dev->mac, ep_dev->link_index);
		next += bytes;
		result += bytes;
	}
	result = min(result, count);
	memcpy(buf, scratch, result);
	*eof = 1;
	return result;
}

/* Print the SR-IOV info: number of VFs. */
static int tlr_read_ep_vfs(char *buf, char **start, off_t offset,
			   int count, int *eof, void *data)
{
	/* Each line has format "MMM trioX-macX L\n" */
	char scratch[TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES * 40];
	struct tlr_pcie_dev *ep_dev;
	struct gxpci_host_regs *regs;
	char *next = scratch;
	int result = 0;
	int i;

	for (i = 0; i < num_ep_devs; i++) {
		int bytes;

		ep_dev = &pcie_ep_devs[i];
		regs = (struct gxpci_host_regs *) ep_dev->bar_mem;

		/*
		 * Skip this EP port if the link is down.
		 */
		if (regs == NULL)
			continue;

		bytes = sprintf(next, "trio%d-mac%d %d\n",
			ep_dev->trio_index, ep_dev->mac, ep_dev->num_vfs);

		next += bytes;
		result += bytes;
	}
	result = min(result, count);
	memcpy(buf, scratch, result);
	*eof = 1;
	return result;
}

/*
 * Check if the BAR0 register has been set by the host. We need valid BAR0
 * address to configure the RX BAR0 Address Mask properly, before the host
 * can issue any MMIOs to us.
 */
static int is_bar0_set(struct tlr_pcie_dev *ep_dev)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	unsigned int reg_offset;

	reg_offset =
		(TRIO_PCIE_EP_BASE_ADDR0 << TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);

	ep_dev->bar0_addr =
		__gxio_mmio_read(trio_context->mmio_base_mac + reg_offset) &
				 PCI_BASE_ADDRESS_MEM_MASK;

	return ep_dev->bar0_addr;
}

static void set_ep_state(struct tlr_pcie_dev *ep_dev, uint16_t state)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	unsigned int reg_offset;

	reg_offset =
		(TRIO_PCIE_EP_SUBSYS_ID_SUBSYS_VEN_ID <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);

	__gxio_mmio_write16(trio_context->mmio_base_mac + reg_offset, state);
}

/*
 * The host driver should not issue PCIe memory accesses to the Gx EP port
 * before the EP boots Linux. We use the Subsystem Vendor ID register in the
 * Gx's PCI configuration header as a place for the Linux boot flag that is set
 * by this function and read by the host driver.
 */
static void mark_ep_ready(struct tlr_pcie_dev *ep_dev)
{
	set_ep_state(ep_dev, PCIE_STATUS_READY);
}

static int set_mps_mrs(struct tlr_pcie_dev *ep_dev)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	TRIO_PCIE_EP_DEVICE_CONTROL_t device_control;
	unsigned int reg_offset;
	int ret;

	/*
	 * Set the mac_config register in trio based on the
	 * MPS/MRS of the link.
	 */
	reg_offset = (TRIO_PCIE_EP_DEVICE_CONTROL <<
		TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
		TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	device_control.word =
		__gxio_mmio_read16(trio_context->mmio_base_mac + reg_offset);

	/* Set our max read request size to 4KB. */
	device_control.max_read_request_size = 5;
	__gxio_mmio_write16(trio_context->mmio_base_mac + reg_offset,
			    device_control.word);
	device_control.word =
		__gxio_mmio_read16(trio_context->mmio_base_mac + reg_offset);

	ep_dev->max_payload_size = 128 << device_control.max_payload_size;
	ep_dev->max_read_size = 128 << device_control.max_read_request_size;

	ret = gxio_trio_set_mps_mrs(trio_context,
		device_control.max_payload_size,
		device_control.max_read_request_size,
		ep_dev->mac);
	if (ret < 0) {
		pr_err("gxpci_endp: PCIE_CONFIGURE_MAC_MPS_MRS "
		       "failure, MAC %d on TRIO %d\n",
		       ep_dev->mac, ep_dev->trio_index);
		return ret;
	}
	return 0;
}

#if 0
static void endp_config_msix_vf(struct tlr_pcie_dev *ep_dev, int vf_instance)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	TRIO_PCIE_EP_MSI_X_TABLE_OFFSET_BIR_t msix_tbl;
	TRIO_PCIE_EP_MSI_X_PBA_OFFSET_BIR_t msix_pba;
	TRIO_PCIE_EP_MSI_X_CONTROL_t msix_control;
	TRIO_PCIE_INTFC_VF_ACCESS_t vf_access;
	struct gxpci_ep_vf_state *vf_state;
	int host_nic_queue_vectors;
	unsigned int reg_offset;
	int msix_vectors;

	vf_state = &ep_dev->vfs[vf_instance];

	/*
	 * Allows access to virtual function register spaces.
	 */
	reg_offset =
		(TRIO_PCIE_INTFC_VF_ACCESS <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	vf_access.word= 0;
	vf_access.vf_ena = 1;
	vf_access.vf_sel = vf_instance;
	__gxio_mmio_write(trio_context->mmio_base_mac + reg_offset,
			  vf_access.word);

	/*
 	 * Configure the MSI-X capability structure.
 	 */
	reg_offset =
		(TRIO_PCIE_EP_MSI_X_CONTROL <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);

	/* Initialize MSI-X info. */
	host_nic_queue_vectors =
		MAX(GXPCI_HOST_NIC_TX_QUEUES_VF, GXPCI_HOST_NIC_RX_QUEUES_VF);

	/* Each VF supports one NIC interface only. */
	msix_vectors = host_nic_queue_vectors;

	vf_state->msix_host_nic_intr_vec_base = 0;
	vf_state->msix_vectors = msix_vectors;

	msix_control.msi_x_table_size = msix_vectors - 1;
	msix_control.function_mask = 0;
	msix_control.msi_x_enable = 0;
	__gxio_mmio_write16(trio_context->mmio_base_mac + reg_offset,
			    msix_control.word);

	reg_offset =
		(TRIO_PCIE_EP_MSI_X_TABLE_OFFSET_BIR <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);

	msix_tbl.bir = GXPCI_MSIX_TABLE_BIR_VF;
	msix_tbl.table_offset = GXPCI_MSIX_TABLE_OFFSET_VF >>
		TRIO_PCIE_EP_MSI_X_TABLE_OFFSET_BIR__TABLE_OFFSET_SHIFT;
	__gxio_mmio_write32(trio_context->mmio_base_mac + reg_offset,
			    msix_tbl.word);

	reg_offset =
		(TRIO_PCIE_EP_MSI_X_PBA_OFFSET_BIR <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);

	msix_pba.pending_bit_array_pba_bir = GXPCI_MSIX_PBA_BIR_VF;
	msix_pba.pba_offset = GXPCI_MSIX_PBA_OFFSET_VF >>
		TRIO_PCIE_EP_MSI_X_PBA_OFFSET_BIR__PBA_OFFSET_SHIFT;
	__gxio_mmio_write32(trio_context->mmio_base_mac + reg_offset,
			    msix_pba.word);

	/* Disables access to virtual function register spaces. */
	reg_offset =
		(TRIO_PCIE_INTFC_VF_ACCESS <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	vf_access.word= 0;
	__gxio_mmio_write(trio_context->mmio_base_mac + reg_offset,
			  vf_access.word);

	return;
}
#endif

static void retrieve_vf_info(struct tlr_pcie_dev *ep_dev)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	struct gxpci_host_regs *regs;
	unsigned int reg_offset;
	int total_vfs;
	int i;

	regs = (struct gxpci_host_regs *) ep_dev->bar_mem;

	/*
	 * Retrieve the VF BAR addresses. Addresses for each VF
	 * will be calculated later when it is needed.
	 */
	reg_offset =
		(TRIO_PCIE_EP_VF_BAR1 <<
		TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
		TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	ep_dev->vfs[0].vf_bar0_address =
		__gxio_mmio_read32(trio_context->mmio_base_mac + reg_offset);
	reg_offset =
		(TRIO_PCIE_EP_VF_BAR0 <<
		TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
		TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	ep_dev->vfs[0].vf_bar0_address =
		(ep_dev->vfs[0].vf_bar0_address << 32) +
		(__gxio_mmio_read32(trio_context->mmio_base_mac + reg_offset) &
		PCI_BASE_ADDRESS_MEM_MASK);

	reg_offset =
		(TRIO_PCIE_EP_VF_BAR3 <<
		TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
		TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	ep_dev->vfs[0].vf_bar2_address =
		__gxio_mmio_read32(trio_context->mmio_base_mac + reg_offset);
	reg_offset =
		(TRIO_PCIE_EP_VF_BAR2 <<
		TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
		TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	ep_dev->vfs[0].vf_bar2_address =
		(ep_dev->vfs[0].vf_bar2_address << 32) +
		(__gxio_mmio_read32(trio_context->mmio_base_mac + reg_offset) &
		PCI_BASE_ADDRESS_MEM_MASK);

	reg_offset =
		(TRIO_PCIE_EP_NUMVFS <<
		TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
		TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	ep_dev->num_vfs=
		__gxio_mmio_read16(trio_context->mmio_base_mac + reg_offset);

	reg_offset =
		(TRIO_PCIE_EP_TOTALVFS <<
		TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
		TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	total_vfs =
		__gxio_mmio_read16(trio_context->mmio_base_mac + reg_offset);

	if (!total_vfs)
		return;

	ep_dev->vf_bar0_size = regs->vf_bar0_size / total_vfs;
	ep_dev->vf_bar2_size = regs->vf_bar2_size / total_vfs;

	ep_dev->vfs[0].instance = 0;

	for (i = 1; i < ep_dev->num_vfs; i++) {
		ep_dev->vfs[i].instance = i;
		ep_dev->vfs[i].vf_bar0_address =
			ep_dev->vfs[0].vf_bar0_address +
			i * ep_dev->vf_bar0_size;
		ep_dev->vfs[i].vf_bar2_address =
			ep_dev->vfs[0].vf_bar2_address +
			i * ep_dev ->vf_bar2_size;
	}

#if 0
	/* Configure the MSI-X structures per VF */
	for (i = 0; i < ep_dev->num_vfs; i++)
		endp_config_msix_vf(ep_dev, i);
#endif
}

/*
 * This function creates a mapping between the memory space and the
 * PCI space, by using a regular page to back up part of the BAR0 space.
 */
static int create_bar0_mapping(struct tlr_pcie_dev *ep_dev, void **va,
			       uint64_t bus_addr)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	unsigned int mem_map;
	struct page *page;
	void *bar_mem;
	int err;

	/* Allocate a page to back up the BAR space. */
	page = alloc_page(GFP_ATOMIC | __GFP_ZERO);
	if (!page) {
		pr_err("%s: TRIO %d MAC %d alloc_page failure.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac);
		return -1;
	}
	bar_mem = page_address(page);

	/* Register the page. */
	err = gxio_trio_register_page(trio_context,
				      trio_context->asid,
				      bar_mem,
				      PAGE_SIZE,
				      0);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d register_page failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		return -1;
	}

	/* Allocate and initialize the memory map region. */
	err = gxio_trio_alloc_memory_maps(trio_context, 1, 0, 0);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map alloc failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		return -1;
	}

	mem_map = err;

	err = gxio_trio_init_memory_map(trio_context, mem_map, bar_mem,
					PAGE_SIZE, trio_context->asid,
					ep_dev->mac, bus_addr,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map init failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		return -1;
	}

	*va = bar_mem;

	return mem_map;
}

/*
 * After the BAR0 register is set by the host, this function is called
 * once to set up the BAR0 mapping and the backing memory. In addition,
 * it also allocates the resources for the legacy API drivers.
 */
static int endp_config_bar0(struct tlr_pcie_dev *ep_dev)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	struct tlr_stream *stream;
#if GXPCI_HOST_ZC_QUEUE_COUNT
	struct tlr_zc_stream *zc_stream;
#endif
	unsigned int reg_offset;
	unsigned int mem_map_index;
	struct page *hpage;
#if GXPCI_HOST_ZC_QUEUE_COUNT
	struct page *zc_hpage;
#endif
	int i;
	int irq;
#if GXPCI_HOST_ZC_QUEUE_COUNT
	int zc_irq;
#endif
	int cpu;
	int err = 0;

	/*
	 * Disable the RX BAR0 Address Mask so that the TRIO sees
	 * the full PCI address (not just the BAR offset). Note that
	 * this applies to both the PF and the VFs. The TRIO_MAP_RSH_BASE
	 * must also be written with the full BAR0 address and this is
	 * done in the HV TRIO driver, via gxio_trio_set_mps_mrs().
	 */
	reg_offset =
		(TRIO_PCIE_INTFC_RX_BAR0_ADDR_MASK <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	__gxio_mmio_write(trio_context->mmio_base_mac + reg_offset,
			  TRIO_PCIE_INTFC_RX_BAR0_ADDR_MASK__MASK_MASK);

	/*
	 * Now that the BAR0 address is set by the RC host, we need to modify
	 * the Map Memory region windows to work with full PCI addresses.
	 */
	err = gxio_trio_init_memory_map(trio_context, ep_dev->host_reg_mem_map,
					ep_dev->bar_mem,
					PAGE_SIZE, trio_context->asid,
					ep_dev->mac, ep_dev->bar0_addr +
					GXPCI_HOST_REGS_OFFSET,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map re-config failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto bar_map_init_failed;
	}

	err = gxio_trio_init_memory_map(trio_context, ep_dev->host_msix_mem_map,
					ep_dev->msix_table_base,
					PAGE_SIZE, trio_context->asid,
					ep_dev->mac, ep_dev->bar0_addr +
					GXPCI_MSIX_TABLE_OFFSET,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d msix map re-config failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto msix_map_init_failed;
	}

	/* Initialize semaphore for barmem file. */
	sema_init(&ep_dev->mutex, 1);

	/*
	 * Initialize semaphore for PIO region protection.
	 * It is safe to assume that there are 2 PIO regions free.
	 */
	sema_init(&ep_dev->pio_sem, 2);

	/*
 	 * The single page size should be at least 8KB to hold the MSI-X Table
 	 * entries and MSI-X PBA entries.
 	 */
	BUG_ON(PAGE_SIZE <= 4096);

	for (i = 0; i < GXPCI_C2C_QUEUE_COUNT; i++) {
		sema_init(&ep_dev->c2c_send[i].mutex, 1);
		sema_init(&ep_dev->c2c_recv[i].mutex, 1);
	}

	/* Allocate and initialize the char streams. */
	for (i = 0; i < GXPCI_HOST_CHAR_STREAMS_COUNT; i++) {
		stream = tlr_get_stream(ep_dev, i);
		stream = kmalloc(sizeof(*stream), GFP_KERNEL);

		if (stream == NULL)
			goto stream_alloc_failed;

		tlr_set_stream(ep_dev, i, stream);
		memset(stream, 0, sizeof(*stream));

		/* Char stream init. */
		stream->dev = ep_dev;
		stream->index = i;

		sema_init(&stream->write_mutex, 1);
		init_waitqueue_head(&stream->write_queue);

		sema_init(&stream->read_mutex, 1);
		init_waitqueue_head(&stream->read_queue);

		spin_lock_init(&stream->h2t_cmd_queue_lock);
		spin_lock_init(&stream->t2h_cmd_queue_lock);
	}

	/*
 	 * Allocate a huge page to back up host interface part of the BAR, i.e.
	 * one 64-bit mem-map interrupt register at the top of this page, and
 	 * struct gxpci_host_queue_regs, for all the char streams.
 	 */
	hpage = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(HPAGE_SIZE));
	if (!hpage) {
		err = -ENOMEM;
		goto stream_bar_mem_failed;
	}

	/* Register the huge page. */
	err = gxio_trio_register_page(trio_context,
				      trio_context->asid,
				      page_address(hpage),
				      HPAGE_SIZE,
				      0);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d register_page failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto stream_mem_reg_failed;
	}

	/* Allocate and initialize one memory map region. */
	err = gxio_trio_alloc_memory_maps(trio_context, 1, 0, 0);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map alloc failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto stream_mem_map_alloc_failed;
	}

	mem_map_index = err;

	err = gxio_trio_init_memory_map(trio_context, mem_map_index,
					page_address(hpage) +
					GXPCI_HOST_CHAR_REGS_MAP_SIZE,
					(GXPCI_HOST_CHAR_STREAMS_MAP_SIZE * 2),
					trio_context->asid,
					ep_dev->mac,
					ep_dev->bar0_addr +
					GXPCI_HOST_CHAR_STREAMS_OFFSET,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map init failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto stream_mem_map_init_failed;
	}

	/* Init the mem-map region for each valid char stream device. */
	for (i = 0; i < GXPCI_HOST_CHAR_STREAMS_COUNT; i++) {
		stream = tlr_get_stream(ep_dev, i);
		stream->h2t_regs =
			(struct gxpci_host_queue_regs *) (
			 page_address(hpage) + GXPCI_HOST_CHAR_REGS_MAP_SIZE +
			 i * GXPCI_HOST_CHAR_REGS_MAP_SIZE);
		memset((void *) stream->h2t_regs, 0,
		       sizeof(struct gxpci_host_queue_regs));

		stream->t2h_regs =
			(struct gxpci_host_queue_regs *) (
			 page_address(hpage) + GXPCI_HOST_CHAR_REGS_MAP_SIZE +
			 GXPCI_HOST_CHAR_STREAMS_MAP_SIZE +
			 i * GXPCI_HOST_CHAR_REGS_MAP_SIZE);
		memset((void *) stream->t2h_regs, 0,
		       sizeof(struct gxpci_host_queue_regs));
	}

	/* Allocate and initialize one memory map region for MMI interrupts. */
	err = gxio_trio_alloc_memory_maps(trio_context, 1, 0, 0);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map alloc failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto stream_mmi_mem_map_alloc_failed;
	}
	mem_map_index = err;

	for (i = 0; i < GXPCI_HOST_CHAR_STREAMS_COUNT; i++) {
		stream = tlr_get_stream(ep_dev, i);
		/* Record the mem-map region index. */
		stream->mem_map = mem_map_index;

		TRACE("Char stream %d: mem_map #%d\n", i, mem_map_index);
	}

	/*
	 * Initialize the MMI interrupt bus address in BAR0, i.e.
	 * GXPCI_HOST_CHAR_STREAMS_MMI_REGS_OFFSET.
 	 */
	err = gxio_trio_init_memory_map(trio_context, mem_map_index,
					page_address(hpage),
					GXPCI_HOST_CHAR_REGS_MAP_SIZE,
					trio_context->asid,
					ep_dev->mac,
					ep_dev->bar0_addr +
					GXPCI_HOST_CHAR_STREAMS_MMI_REGS_OFFSET,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map init failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto stream_mmi_mem_map_init_failed;
	}

	/* Allocate and register a common char stream interrupt. */
	irq = create_irq();
	if (irq < 0) {
		pr_err("%s: no free irq vectors.\n", driver_name);
		goto create_irq_failed;
	}

	/* Enable the MMI interrupt in kernel. */
	tile_irq_activate(irq, TILE_IRQ_PERCPU);
	if (request_irq(irq, stream_intr, 0, driver_name, (void *) ep_dev)) {
		pr_err("%s: Failed to register handler for IRQ %d\n",
		       driver_name, irq);
		goto req_irq_kernel_failed;
	}

	cpumask_copy(&intr_cpus_map, cpu_online_mask);

#ifdef CONFIG_DATAPLANE
	/* Remove dataplane cpus. */
	cpumask_andnot(&intr_cpus_map, &intr_cpus_map, &dataplane_map);
#endif
	/* Try to choose a non-dataplane processor to receive MMI interrupts. */
	cpu = tile_irq_cpu(irq);

	/* Enable the MMI interrupt in TRIO. */
	err = gxio_trio_config_char_intr(trio_context,
					 cpu_x(cpu), cpu_y(cpu),
					 KERNEL_PL, irq,
					 ep_dev->mac,
					 mem_map_index, 0, 0,
					 MEM_MAP_SEL);

	if (err < 0) {
		pr_err("%s: Memory map region %d interrupt config failed\n",
		       driver_name, mem_map_index);

		goto cfg_irq_hv_failed;
	}

	/* Record the MMI interrupt irq number. */
	for (i = 0; i < GXPCI_HOST_CHAR_STREAMS_COUNT; i++) {
		stream = tlr_get_stream(ep_dev, i);
		stream->mmi_irq = irq;
	}

#if GXPCI_HOST_ZC_QUEUE_COUNT
	/* Allocate and initialize the zero-copy command queues. */
	err = tlr_zc_init(ep_dev);
	if (err != 0)
		goto zc_init_failed;

	/*
	 * Allocate a huge page to back up host interface part of the BAR, i.e.
	 * a 64-bit mem-map interrupt register at the top of this page, and
 	 * struct gxpci_host_queue_regs, for all the zero-copy streams.
 	 */
	zc_hpage = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(HPAGE_SIZE));
	if (!zc_hpage) {
		err = -ENOMEM;
		goto zc_stream_bar_mem_failed;
	}

	/* Register the page. */
	err = gxio_trio_register_page(trio_context,
				      trio_context->asid,
				      page_address(zc_hpage),
				      HPAGE_SIZE,
				      0);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d register_page failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto zc_stream_mem_reg_failed;
	}

	/* Allocate and initialize one memory map region. */
	err = gxio_trio_alloc_memory_maps(trio_context, 1, 0, 0);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map alloc failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto zc_stream_mem_map_alloc_failed;
	}

	mem_map_index = err;

	err = gxio_trio_init_memory_map(trio_context, mem_map_index,
					page_address(zc_hpage) +
					GXPCI_HOST_CHAR_REGS_MAP_SIZE,
					(GXPCI_HOST_ZC_QUEUE_MAP_SIZE * 2),
					trio_context->asid,
					ep_dev->mac,
					ep_dev->bar0_addr +
					GXPCI_HOST_ZC_QUEUE_REGS_OFFSET,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map init failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto zc_stream_mem_map_init_failed;
	}

	/* Init the mem-map region for each valid zero-copy stream. */
	for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
		zc_stream = tlr_get_zc_h2t_stream(ep_dev, i);
		if (zc_stream) {
			zc_stream->h2t_regs =
				(struct gxpci_host_queue_regs *) (
				page_address(zc_hpage) +
				GXPCI_HOST_CHAR_REGS_MAP_SIZE +
				i * GXPCI_HOST_CHAR_REGS_MAP_SIZE);
			memset((void *) zc_stream->h2t_regs, 0,
			       sizeof(struct gxpci_host_queue_regs));
		}

		zc_stream = tlr_get_zc_t2h_stream(ep_dev, i);
		if (zc_stream) {
			zc_stream->t2h_regs =
				(struct gxpci_host_queue_regs *) (
				page_address(zc_hpage) +
				GXPCI_HOST_CHAR_REGS_MAP_SIZE +
				GXPCI_HOST_ZC_QUEUE_MAP_SIZE +
				i * GXPCI_HOST_CHAR_REGS_MAP_SIZE);
			memset((void *) zc_stream->t2h_regs, 0,
			       sizeof(struct gxpci_host_queue_regs));
		}
	}

	/* Allocate and initialize one memory map region. */
	err = gxio_trio_alloc_memory_maps(trio_context, 1, 0, 0);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map alloc failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto zc_stream_mmi_mem_map_alloc_failed;
	}
	mem_map_index = err;

	for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
		zc_stream = tlr_get_zc_h2t_stream(ep_dev, i);
		if (zc_stream)
			/* Record the mem-map region index. */
			zc_stream->mem_map = mem_map_index;
		TRACE("Zero-copy H2T stream %d: mem_map #%d\n",
		      i, mem_map_index);

		zc_stream = tlr_get_zc_t2h_stream(ep_dev, i);
		if (zc_stream)
			/* Record the mem-map region index. */
			zc_stream->mem_map = mem_map_index;
		TRACE("Zero-copy T2H stream %d: mem_map #%d\n",
		      i, mem_map_index);
	}

	/*
	 * Initialize the MMI interrupt bus address in BAR0, i.e.
 	 * GXPCI_HOST_ZC_QUEUE_MMI_REGS_OFFSET.
 	 */
	err = gxio_trio_init_memory_map(trio_context, mem_map_index,
					page_address(zc_hpage),
					GXPCI_HOST_CHAR_REGS_MAP_SIZE,
					trio_context->asid,
					ep_dev->mac,
					ep_dev->bar0_addr +
					GXPCI_HOST_ZC_QUEUE_MMI_REGS_OFFSET,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d mem_map init failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto zc_stream_mmi_mem_map_init_failed;
	}

	/* Allocate and register a common zero-copy stream interrupt. */
	zc_irq = create_irq();
	if (zc_irq < 0) {
		pr_err("%s: no free irq vectors.\n", driver_name);
		goto zc_create_irq_failed;
	}

	/* Enable the MMI interrupt in kernel. */
	tile_irq_activate(zc_irq, TILE_IRQ_PERCPU);
	if (request_irq(zc_irq,
			zc_stream_intr,
			0,
			driver_name,
			(void *) ep_dev)) {
		pr_err("%s: Failed to register handler for IRQ %d\n",
		       driver_name, zc_irq);
		goto zc_req_irq_kernel_failed;
	}

	/* Choose an arbitrary processor to receive MMI interrupts. */
	cpu = raw_smp_processor_id();

	/* Enable the MMI interrupt in TRIO. */
	err = gxio_trio_config_char_intr(trio_context,
					 cpu_x(cpu), cpu_y(cpu),
					 KERNEL_PL, zc_irq,
					 ep_dev->mac,
					 mem_map_index, 0, 0,
					 MEM_MAP_SEL);
	if (err < 0) {
		pr_err("%s: Memory map region %d interrupt config failed\n",
		       driver_name, mem_map_index);

		goto zc_cfg_irq_hv_failed;
	}

	/* Record the MMI interrupt irq number. */
	for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
		zc_stream = tlr_get_zc_h2t_stream(ep_dev, i);
		if (zc_stream)
			zc_stream->mmi_irq = zc_irq;

		zc_stream = tlr_get_zc_t2h_stream(ep_dev, i);
		if (zc_stream)
			zc_stream->mmi_irq = zc_irq;
	}
#endif

	return err;

#if GXPCI_HOST_ZC_QUEUE_COUNT
zc_cfg_irq_hv_failed:
zc_req_irq_kernel_failed:
	destroy_irq(zc_irq);
zc_create_irq_failed:
zc_stream_mmi_mem_map_init_failed:
zc_stream_mmi_mem_map_alloc_failed:
zc_stream_mem_map_init_failed:
zc_stream_mem_map_alloc_failed:
zc_stream_mem_reg_failed:
	homecache_free_pages((unsigned long) page_address(zc_hpage),
			     get_order(HPAGE_SIZE));
zc_stream_bar_mem_failed:
zc_init_failed:
#endif
cfg_irq_hv_failed:
req_irq_kernel_failed:
	destroy_irq(irq);
create_irq_failed:
stream_mmi_mem_map_init_failed:
stream_mmi_mem_map_alloc_failed:
stream_mem_map_init_failed:
stream_mem_map_alloc_failed:
stream_mem_reg_failed:
	homecache_free_pages((unsigned long) page_address(hpage),
			     get_order(HPAGE_SIZE));
stream_bar_mem_failed:
stream_alloc_failed:
	for (i = 0; i < GXPCI_HOST_CHAR_STREAMS_COUNT; i++) {
		stream = tlr_get_stream(ep_dev, i);
		if (stream != NULL) {
			kfree(stream);
			tlr_set_stream(ep_dev, i, NULL);
		}
	}
msix_map_init_failed:
	homecache_free_page((unsigned long) ep_dev->msix_table_base);
bar_map_init_failed:
	homecache_free_page((unsigned long) ep_dev->bar_mem);
	return err;
}

static void ep_link_manager(struct work_struct *work)
{
	struct tlr_pcie_dev *ep_dev =
		container_of(work, struct tlr_pcie_dev, ep_link_work.work);
	struct gxpci_host_regs *regs;

        /*
	 * In case we lose the PCIE_STATUS_READY flag in the
	 * Subsys Vendor ID Register due to MAC reset, mark this
	 * device ready again here.
	 */
	mark_ep_ready(ep_dev);

	/*
	 * If the host hasn't set the BAR0 address yet, which is more common
	 * in the inter-Gx case, wait until it is done before proceeding with
	 * the hand-shake.
	 *
	 * Once the RC node sees us, it assigns a non-zero link_index to us.
	 */
	regs = (struct gxpci_host_regs *) ep_dev->bar_mem;
	if (is_bar0_set(ep_dev) && regs->link_index) {
		void *mapped_addr;
		int ret;

		if (endp_config_bar0(ep_dev) < 0) {
			pr_err("%s: BAR0 config failure.\n", driver_name);
			return;
		}

		/*
		 * The RC/BIOS should have set the max_payload_size in our
		 * Device Control register. We need to copy it to register
		 * TRIO_MAC_CONFIG, along with the max read req size.
		 */
		ret = set_mps_mrs(ep_dev);
		if (ret < 0)
			return;

		retrieve_vf_info(ep_dev);

		/*
		 * Initialize and map the PIO region so that we can access
		 * the queue-pair status array resident in the RC host memory.
		 */
		ret = create_pio_mapping(ep_dev, &mapped_addr,
			regs->queue_sts_array_bus_addr,
			sizeof(struct gxpci_queue_status_array), 0);
		if (ret < 0) {
			pr_err("%s: queue status array mapping failure.\n",
				driver_name);
			return;
		}

		ep_dev->queue_sts_array = mapped_addr;

		/*
		 * If the C2C queue status array address is set, we need to
		 * map it with a PIO region.
		 */
		if (regs->c2c_queue_sts_array_bus_addr) {
			ret = create_pio_mapping(ep_dev, &mapped_addr,
				regs->c2c_queue_sts_array_bus_addr,
				sizeof(struct gxpci_c2c_queue_sts_array), 0);
			if (ret < 0) {
				pr_err("%s: C2C queue status array mapping "
					"failure %d.\n", driver_name, ret);
				return;
			}

			ep_dev->c2c_queue_sts_array = mapped_addr;
		}

		regs->port_status = PCIE_STATUS_RUNNING;

		if (net_macs & (1 << (ep_dev->mac + 4 * ep_dev->trio_index))) {
			if (ep_dev->num_vfs && enable_vf_nic)
				gxpci_endp_net_devs_init_vf(ep_dev);

			gxpci_endp_net_devs_init(ep_dev);
		}

		return;
	}

	schedule_delayed_work(&ep_dev->ep_link_work, EP_DRV_READY_TIMEOUT);
}

static int endp_config_msix(struct tlr_pcie_dev *ep_dev)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	TRIO_PCIE_EP_MSI_X_TABLE_OFFSET_BIR_t msix_tbl;
	TRIO_PCIE_EP_MSI_X_PBA_OFFSET_BIR_t msix_pba;
	TRIO_PCIE_EP_MSI_X_CONTROL_t msix_control;
	int host_nic_queue_vectors;
	unsigned int reg_offset;
	int msix_vectors;
	int err;

	/*
 	 * Allocate a page to back up host interface part of the BAR.
 	 * This contains MSI-X Table entries and MSI-X PBA entries.
 	 */
	err = create_bar0_mapping(ep_dev, &ep_dev->msix_table_base,
				  GXPCI_MSIX_TABLE_OFFSET);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d MSI-X mapping failure.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac);
		return err;
	}
	ep_dev->host_msix_mem_map = err;

	/*
 	 * Configure the MSI-X capability structure.
 	 */
	reg_offset =
		(TRIO_PCIE_EP_MSI_X_CONTROL <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);

	/* Initialize MSI-X info. */
	host_nic_queue_vectors =
		MAX(host_vnic_tx_queues[ep_dev->trio_index][ep_dev->mac],
		    host_vnic_rx_queues[ep_dev->trio_index][ep_dev->mac]);

	msix_vectors = GXPCI_HOST_CHAR_STREAMS_COUNT +
		GXPCI_HOST_ZC_QUEUE_COUNT + host_nic_queue_vectors *
		host_vnic_ports[ep_dev->trio_index][ep_dev->mac];
	ep_dev->msix_cs_q_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE;
#ifdef GXPCI_INTR_VECTOR_PER_QUEUE
	msix_vectors *= 2;
	ep_dev->msix_zc_q_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		GXPCI_HOST_CHAR_STREAMS_COUNT * 2;
	ep_dev->msix_host_nic_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		(GXPCI_HOST_CHAR_STREAMS_COUNT + GXPCI_HOST_ZC_QUEUE_COUNT) * 2;
#else
	ep_dev->msix_zc_q_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		GXPCI_HOST_CHAR_STREAMS_COUNT;
	ep_dev->msix_host_nic_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		GXPCI_HOST_CHAR_STREAMS_COUNT + GXPCI_HOST_ZC_QUEUE_COUNT;
#endif
	ep_dev->msix_vectors = msix_vectors;
	msix_control.msi_x_table_size = msix_vectors - 1;
	msix_control.function_mask = 0;
	msix_control.msi_x_enable = 0;
	__gxio_mmio_write16(trio_context->mmio_base_mac + reg_offset,
			    msix_control.word);

	reg_offset =
		(TRIO_PCIE_EP_MSI_X_TABLE_OFFSET_BIR <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);

	msix_tbl.bir = GXPCI_MSIX_TABLE_BIR;
	msix_tbl.table_offset = GXPCI_MSIX_TABLE_OFFSET >>
		TRIO_PCIE_EP_MSI_X_TABLE_OFFSET_BIR__TABLE_OFFSET_SHIFT;
	__gxio_mmio_write32(trio_context->mmio_base_mac + reg_offset,
			    msix_tbl.word);

	reg_offset =
		(TRIO_PCIE_EP_MSI_X_PBA_OFFSET_BIR <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);

	msix_pba.pending_bit_array_pba_bir = GXPCI_MSIX_PBA_BIR;
	msix_pba.pba_offset = GXPCI_MSIX_PBA_OFFSET >>
		TRIO_PCIE_EP_MSI_X_PBA_OFFSET_BIR__PBA_OFFSET_SHIFT;
	__gxio_mmio_write32(trio_context->mmio_base_mac + reg_offset,
			    msix_pba.word);

	return 0;
}

/*
 * The host driver and this endpoint port driver participate in the following
 * hand-shake process in order to establish the logical connection:
 *   1. The host driver polls for PCIE_STATUS_READY flag in the port's Subsys
 *	Vendor ID Register before issuing any MMIO access to this device. 
 *   2. In function ep_trio_init(), the EP driver allocates the memory and
 *	sets up the inbound mapping window for the gxpci_host_regs struct
 *	and the MSI-X table in BAR0, based on the default RX_BAR0_ADDR_MASK
 *	setting, i.e. masking off the high bits of the PCI address. Then,
 *	the EP driver sets the port_status register in gxpci_host_regs struct
 *	to PCIE_STATUS_READY and sets the PCIE_STATUS_READY flag in the
 *	Subsys Vendor ID Register. Before returning from this function,
 *	it starts a schedule work to wait for the host driver to assign the
 *	link index to this EP port.
 *   3. Once the host driver sees the PCIE_STATUS_READY flag in the Subsys
 *	Vendor ID Register, it issues MMIO access to the gxpci_host_regs struct
 *	and the MSI-X table in BAR0. As a marker, the last MMIO is used to
 *	write the link index to the link_index register. This concludes the
 *	the host driver's hand-shake process.
 *   4. In function ep_link_manager(), the EP driver sees the link_index
 *	register in the gxpci_host_regs struct having been set. Knowing
 *	that the host driver has set all the important parameters, the EP
 *	driver sets the RX_BAR0_ADDR_MASK register so that BAR0 address
 *	matching will be based on the full 64-bit address from this point on, 
 *	which is necessary to support both SR-IOV PF and VFs.
 */
static void ep_trio_init(struct work_struct *work)
{
	struct tlr_pcie_dev *ep_dev =
		container_of(work, struct tlr_pcie_dev, ep_trio_init_work.work);
	gxio_trio_context_t *trio_context = ep_dev->trio;
	TRIO_PCIE_INTFC_PORT_STATUS_t port_status;
	TRIO_PCIE_INTFC_TX_FIFO_CTL_t tx_fifo_ctl;
	TRIO_PCIE_EP_LINK_STATUS_t link_status;
	struct gxpci_host_regs *bar_regs;
	unsigned int reg_offset;
	int err;

	/* Check for PCIe link-up status. */
	reg_offset =
		(TRIO_PCIE_INTFC_PORT_STATUS <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	port_status.word =
		__gxio_mmio_read(trio_context->mmio_base_mac + reg_offset);

	/*
 	 * If the port still isn't up, tell the user we're going to wait,
 	 * then spin until it is, or until we've waited longer than the
 	 * timeout.
 	 */
	if (!port_status.dl_up) {
		unsigned long start_time = jiffies;

		pr_err("PCI: end-point link is down, MAC %d on "
			"TRIO %d, trying for %d more seconds...\n",
			ep_dev->mac, ep_dev->trio_index,
			ep_link_wait);

		while (!port_status.dl_up &&
		       (jiffies - start_time) < ep_link_wait * HZ) {
			msleep(100);

			/* Try to bring the link up. */
			gxio_trio_force_ep_link_up(trio_context, ep_dev->mac);
			port_status.word =
				__gxio_mmio_read(trio_context->mmio_base_mac +
						 reg_offset);
		}
	}

	if (!port_status.dl_up) {
		pr_err("PCI: end-point link is down, MAC %d on "
		       "TRIO %d\n", ep_dev->mac, ep_dev->trio_index);
		return;
	}

	/* Ensure that the link can come out of L1 power down state. */
	reg_offset =
		(TRIO_PCIE_INTFC_TX_FIFO_CTL <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	tx_fifo_ctl.word =
		__gxio_mmio_read(trio_context->mmio_base_mac + reg_offset);
	tx_fifo_ctl.min_p_credits = 0;
	__gxio_mmio_write(trio_context->mmio_base_mac + reg_offset,
			  tx_fifo_ctl.word);

	/* Check the link status. */
	reg_offset =
		(TRIO_PCIE_EP_LINK_STATUS <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_STANDARD <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	link_status.word =
		__gxio_mmio_read16(trio_context->mmio_base_mac + reg_offset);

	ep_dev->link_width = link_status.negotiated_link_width;
	ep_dev->link_speed = __ffs(link_status.link_speed) + 1;

	pr_info("%s: TRIO %d MAC %d, lanes = %d, Gen %d\n",
		driver_name, ep_dev->trio_index,
		ep_dev->mac, ep_dev->link_width, ep_dev->link_speed);

	/*
	 * Allocate a page to back up host interface part of the BAR.
	 * This contains the struct gxpci_host_regs.
	 * Note that at this moment we rely on the fact that regiser
	 * RX_BAR0_ADDR_MASK is in reset state, i.e. masking off the
	 * high bits of the PCI address. This register will be set
	 * properly after the BAR0 address is assigned by the RC.
	 */
	err = create_bar0_mapping(ep_dev, &ep_dev->bar_mem,
				  GXPCI_HOST_REGS_OFFSET);
	if (err < 0) {
		pr_err("%s: TRIO %d MAC %d host_regs mapping failure.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac);
		return;
	}
	ep_dev->host_reg_mem_map = err;

	err = endp_config_msix(ep_dev);
	if (err < 0)
		return;

	/*
 	 * Mark this EP ready for host communication. The host waits for
 	 * PCIE_STATUS_READY status before accessing the rest of
 	 * gxpci_host_regs.
 	 */
	bar_regs = (struct gxpci_host_regs *) ep_dev->bar_mem;
	bar_regs->version = PCIE_VERSION_DEF(GXPCI_MAJOR, GXPCI_MINOR);
	bar_regs->port_status = PCIE_STATUS_READY;

	/*
 	 * Now that the memory mapping is set up, we mark the port ready for
 	 * MMIO by setting PCIE_STATUS_READY to the Subsys Vendor ID Register.
 	 */
	mark_ep_ready(ep_dev);

	/*
 	 * Schedule work to wait for the host driver to assign us a link index.
 	 */
	INIT_DELAYED_WORK(&ep_dev->ep_link_work, ep_link_manager);
	schedule_delayed_work(&ep_dev->ep_link_work, EP_DRV_READY_TIMEOUT);

	return;
}

static int __devinit setup_pcie_host_vnic_ports(char *str)
{
	unsigned long ports = GXPCI_HOST_NIC_COUNT;
	unsigned long trio_index;
	unsigned long mac;

	if (str == NULL || !isdigit(*str))
		return -EINVAL;
	trio_index = simple_strtoul(str, (char **) &str, 10);
	if (trio_index >= TILEGX_NUM_TRIO)
		return -EINVAL;

	if (*str != ',')
		return -EINVAL;

	str++;
	if (!isdigit(*str))
		return -EINVAL;
	mac = simple_strtoul(str, (char **) &str, 10);
	if (mac >= TILEGX_TRIO_PCIES)
		return -EINVAL;

	if (*str != '\0') {
		if (*str != ',')
			return -EINVAL;

		str++;
		if (!isdigit(*str))
			return -EINVAL;
		ports = simple_strtoul(str, (char **) &str, 10);
	}

	host_vnic_ports[trio_index][mac] = ports;

	return 0;
}
early_param("pcie_host_vnic_ports", setup_pcie_host_vnic_ports);


static int __devinit setup_pcie_host_vnic_tx_queues(char *str)
{
	unsigned long tx_queues = GXPCI_HOST_NIC_TX_QUEUES;
	unsigned long trio_index;
	unsigned long mac;

	if (str == NULL || !isdigit(*str))
		return -EINVAL;
	trio_index = simple_strtoul(str, (char **) &str, 10);
	if (trio_index >= TILEGX_NUM_TRIO)
		return -EINVAL;

	if (*str != ',')
		return -EINVAL;

	str++;
	if (!isdigit(*str))
		return -EINVAL;
	mac = simple_strtoul(str, (char **) &str, 10);
	if (mac >= TILEGX_TRIO_PCIES)
		return -EINVAL;

	if (*str != '\0') {
		if (*str != ',')
			return -EINVAL;

		str++;
		if (!isdigit(*str))
			return -EINVAL;
		tx_queues = simple_strtoul(str, (char **) &str, 10);
	}

	host_vnic_tx_queues[trio_index][mac] = tx_queues;

	return 0;
}
early_param("pcie_host_vnic_tx_queues", setup_pcie_host_vnic_tx_queues);


static int __devinit setup_pcie_host_vnic_rx_queues(char *str)
{
	unsigned long rx_queues = GXPCI_HOST_NIC_RX_QUEUES;
	unsigned long trio_index;
	unsigned long mac;

	if (str == NULL || !isdigit(*str))
		return -EINVAL;
	trio_index = simple_strtoul(str, (char **) &str, 10);
	if (trio_index >= TILEGX_NUM_TRIO)
		return -EINVAL;

	if (*str != ',')
		return -EINVAL;

	str++;
	if (!isdigit(*str))
		return -EINVAL;
	mac = simple_strtoul(str, (char **) &str, 10);
	if (mac >= TILEGX_TRIO_PCIES)
		return -EINVAL;

	if (*str != '\0') {
		if (*str != ',')
			return -EINVAL;

		str++;
		if (!isdigit(*str))
			return -EINVAL;
		rx_queues = simple_strtoul(str, (char **) &str, 10);
	}

	host_vnic_rx_queues[trio_index][mac] = rx_queues;

	return 0;
}
early_param("pcie_host_vnic_rx_queues", setup_pcie_host_vnic_rx_queues);


static int __devinit setup_pcie_host_pq_h2t_queues(char *str)
{
	unsigned long queues = GXPCI_HOST_PQ_H2T_COUNT;
	unsigned long trio_index;
	unsigned long mac;

	if (str == NULL || !isdigit(*str))
		return -EINVAL;
	trio_index = simple_strtoul(str, (char **) &str, 10);
	if (trio_index >= TILEGX_NUM_TRIO)
		return -EINVAL;

	if (*str != ',')
		return -EINVAL;

	str++;
	if (!isdigit(*str))
		return -EINVAL;
	mac = simple_strtoul(str, (char **) &str, 10);
	if (mac >= TILEGX_TRIO_PCIES)
		return -EINVAL;

	if (*str != '\0') {
		if (*str != ',')
			return -EINVAL;

		str++;
		if (!isdigit(*str))
			return -EINVAL;
		queues = simple_strtoul(str, (char **) &str, 10);
	}

	host_pq_h2t_queues[trio_index][mac] = queues;

	return 0;
}
early_param("pcie_host_pq_h2t_queues", setup_pcie_host_pq_h2t_queues);


static int __devinit setup_pcie_host_pq_t2h_queues(char *str)
{
	unsigned long queues = GXPCI_HOST_PQ_T2H_COUNT;
	unsigned long trio_index;
	unsigned long mac;

	if (str == NULL || !isdigit(*str))
		return -EINVAL;
	trio_index = simple_strtoul(str, (char **) &str, 10);
	if (trio_index >= TILEGX_NUM_TRIO)
		return -EINVAL;

	if (*str != ',')
		return -EINVAL;

	str++;
	if (!isdigit(*str))
		return -EINVAL;
	mac = simple_strtoul(str, (char **) &str, 10);
	if (mac >= TILEGX_TRIO_PCIES)
		return -EINVAL;

	if (*str != '\0') {
		if (*str != ',')
			return -EINVAL;

		str++;
		if (!isdigit(*str))
			return -EINVAL;
		queues = simple_strtoul(str, (char **) &str, 10);
	}

	host_pq_t2h_queues[trio_index][mac] = queues;

	return 0;
}
early_param("pcie_host_pq_t2h_queues", setup_pcie_host_pq_t2h_queues);


/*
 * Create a device for the specified PCIE link if the link is up.
 */
static int __init tlr_pcie_dev_probe(int ep_index)
{
	struct tlr_pcie_dev *ep_dev = &pcie_ep_devs[ep_index];
	gxio_trio_context_t *trio_context = ep_dev->trio;
	TRIO_PCIE_INTFC_PORT_STATUS_t port_status;
	unsigned int reg_offset;
	int total_nic_queues;
	int total_nic_ports;
	int total_pq_queues;
	int err;

	/* Check for host VNIC configurations. */
	total_nic_ports = host_vnic_ports[ep_dev->trio_index][ep_dev->mac];
	if (total_nic_ports == 0 || total_nic_ports > GXPCI_HOST_NIC_COUNT) {
		if (total_nic_ports)
			pr_err("Illegal host NIC port number, "
			       "default configuration will be used instead.\n");

		host_vnic_ports[ep_dev->trio_index][ep_dev->mac] =
			GXPCI_HOST_NIC_COUNT;
	}

	if (host_vnic_tx_queues[ep_dev->trio_index][ep_dev->mac] !=
	    host_vnic_rx_queues[ep_dev->trio_index][ep_dev->mac]) {
		pr_err("NIC queue number not equal, use min value.\n");

		host_vnic_tx_queues[ep_dev->trio_index][ep_dev->mac] =
		  MIN(host_vnic_tx_queues[ep_dev->trio_index][ep_dev->mac],
		      host_vnic_rx_queues[ep_dev->trio_index][ep_dev->mac]);
		host_vnic_tx_queues[ep_dev->trio_index][ep_dev->mac] =
		host_vnic_rx_queues[ep_dev->trio_index][ep_dev->mac];
	}

	total_nic_queues = host_vnic_ports[ep_dev->trio_index][ep_dev->mac] *
		host_vnic_tx_queues[ep_dev->trio_index][ep_dev->mac];
	if (total_nic_queues == 0 ||
	    host_vnic_tx_queues[ep_dev->trio_index][ep_dev->mac] >
	    GXPCI_HOST_NIC_TX_QUEUES ||
	    host_vnic_rx_queues[ep_dev->trio_index][ep_dev->mac] >
	    GXPCI_HOST_NIC_RX_QUEUES) {

		if (total_nic_queues)
			pr_err("Illegal host NIC queue parameters, "
			       "default configuration will be used instead.\n");

		host_vnic_tx_queues[ep_dev->trio_index][ep_dev->mac] =
			GXPCI_HOST_NIC_TX_QUEUES;
		host_vnic_rx_queues[ep_dev->trio_index][ep_dev->mac] =
			GXPCI_HOST_NIC_RX_QUEUES;
	}

	/* Check for host packet queue configurations. */
	total_pq_queues =
		host_pq_h2t_queues[ep_dev->trio_index][ep_dev->mac] +
		host_pq_t2h_queues[ep_dev->trio_index][ep_dev->mac];
	if (total_pq_queues == 0 ||
	    total_pq_queues > GXPCI_HOST_PQ_MAX_QUEUE_NUM) {

		if (total_pq_queues)
			pr_err("Illegal host PQ parameters, "
			       "default configuration will be used instead.\n");

		host_pq_h2t_queues[ep_dev->trio_index][ep_dev->mac] =
			GXPCI_HOST_PQ_H2T_COUNT;
		host_pq_t2h_queues[ep_dev->trio_index][ep_dev->mac] =
			GXPCI_HOST_PQ_T2H_COUNT;
	}

	/* Host NIC and PQ must be mutually exclusive on the VFs. */
	if (enable_vf_nic && enable_vf_pq) {
		pr_err("VF can't enable both host NIC and PQ, "
		       "enable PQ only.\n");

		enable_vf_nic = 0;
	}

	/* Check for VF host packet queue configurations. */
	if (pq_h2t_queues_in_vf > GXPCI_HOST_PQ_VF_H2T_COUNT) {
		pr_err("Illegal VF host PQ H2T parameter, "
		       "default configuration will be used instead.\n");

		pq_h2t_queues_in_vf = GXPCI_HOST_PQ_VF_H2T_COUNT;
	}
	if (pq_t2h_queues_in_vf > GXPCI_HOST_PQ_VF_T2H_COUNT) {
		pr_err("Illegal VF host PQ T2H parameter, "
		       "default configuration will be used instead.\n");

		pq_t2h_queues_in_vf = GXPCI_HOST_PQ_VF_T2H_COUNT;
	}

	/* Check for PCIe link-up status. */
	reg_offset =
		(TRIO_PCIE_INTFC_PORT_STATUS <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(ep_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	port_status.word =
		__gxio_mmio_read(trio_context->mmio_base_mac + reg_offset);

	/* If the port isn't already up, try to bring it up immediately. */
	if (!port_status.dl_up)
		gxio_trio_force_ep_link_up(trio_context, ep_dev->mac);

	/* Create character device, which has GXPCI_NUM_MINOR_DEVICES nodes. */
	err = tlr_cdev_setup(ep_dev);
	if (err != 0) {
		pr_err("%s: TRIO %d MAC %d tlr_cdev_setup failure %d.\n",
			driver_name, ep_dev->trio_index, ep_dev->mac, err);
		return err;
	}

	/*
 	 * Schedule work immediately to initialize TRIO when PCIe link is up.
 	 */
	INIT_DELAYED_WORK(&ep_dev->ep_trio_init_work, ep_trio_init);
	schedule_delayed_work(&ep_dev->ep_trio_init_work, 0);

	return 0;
}

/*
 * Return 1 if the port is strapped to operate in EP mode.
 */
static int
strapped_for_ep(gxio_trio_context_t *trio_context, int mac)
{
	TRIO_PCIE_INTFC_PORT_CONFIG_t port_config;
	unsigned int reg_offset;

	/* Check the port configuration. */
	reg_offset =
		(TRIO_PCIE_INTFC_PORT_CONFIG <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT) |
		(mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	port_config.word =
		__gxio_mmio_read(trio_context->mmio_base_mac + reg_offset);

	if ((port_config.strap_state ==
	TRIO_PCIE_INTFC_PORT_CONFIG__STRAP_STATE_VAL_AUTO_CONFIG_ENDPOINT) ||
	(port_config.strap_state ==
	TRIO_PCIE_INTFC_PORT_CONFIG__STRAP_STATE_VAL_AUTO_CONFIG_ENDPOINT_G1))
		return 1;
	else
		return 0;
}


static int __init gxpci_ep_init(void)
{
	int ep_index = 0;
	int i, j;

	if (num_trio_shims == 0 || sim_is_simulator())
		return 0;

	/*
	 * Now determine which PCIe ports are configured to operate in EP mode.
	 * We look at the Board Information Block first and then see if
	 * the port strap state requires otherwise.
	 */
#ifndef GX_FPGA
	for (i = 0; i < TILEGX_NUM_TRIO; i++) {
		gxio_trio_context_t *context = &trio_contexts[i];

		if (context->fd < 0)
			continue;

		for (j = 0; j < TILEGX_TRIO_PCIES; j++) {
			int is_ep = 0;

			if (pcie_ports[i].is_gx72 &&
			    pcie_ports[i].ports[j].allow_ep) {
				if (!pcie_ports[i].ports[j].allow_rc ||
				    strapped_for_ep(context, j))
					is_ep = 1;
			} else if (pcie_ports[i].ports[j].allow_ep &&
			    strapped_for_ep(context, j)) {
				is_ep = 1;
			}
			if (is_ep) {
				pcie_ep[i][j] = 1;
				num_ep_devs++;
			}
		}
	}
#else
	/*
	 * For now, just assume that there is a single EP port on trio/0.
	 */
	num_ep_devs = 1;
	pcie_ep[0][1] = 1;
#endif

	/*
	 * Return if no PCIe ports are configured to operate in end-point mode.
	 */
	if (num_ep_devs == 0)
		return 0;

	pcie_ep_devs = kmalloc(sizeof(struct tlr_pcie_dev) * num_ep_devs,
			       GFP_KERNEL);
	if (pcie_ep_devs == NULL)
		return -ENOMEM;

	memset(pcie_ep_devs, 0, sizeof(struct tlr_pcie_dev) * num_ep_devs);

	/*
	 * Set the TRIO pointer and MAC index for each PCIe EP port.
	 */
	for (i = 0; i < TILEGX_NUM_TRIO; i++) {
		for (j = 0; j < TILEGX_TRIO_PCIES; j++) {
			if (pcie_ep[i][j]) {
				pcie_ep_devs[ep_index].trio = &trio_contexts[i];
				pcie_ep_devs[ep_index].mac = j;
				pcie_ep_devs[ep_index].trio_index = i;
				ep_index++;
				if (ep_index == num_ep_devs)
					goto out;
			}
		}
	}

out:

	for (i = 0; i < num_ep_devs; i++)
		tlr_pcie_dev_probe(i);

	create_proc_read_entry("driver/gxpci_ep_major_mac_link", 0, NULL,
			       tlr_read_ep_major_mac_link, NULL);

	create_proc_read_entry("driver/gxpci_ep_vfs", 0, NULL,
			       tlr_read_ep_vfs, NULL);

	return 0;
}


static void __exit gxpci_ep_exit(void)
{
	/*
	 * We're statically compiled into the kernel, so this should never
	 * be called.
	 */
	pr_err("Removing %s", driver_name);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tilera Corporation");

subsys_initcall(gxpci_ep_init);
module_exit(gxpci_ep_exit);
