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
 * Tilera Gx PCIe endpoint driver that provides a NIC interface to the kernel.
 *
 */
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/etherdevice.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/pagemap.h>
#include <linux/pci.h>
#include <linux/rtnetlink.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#include <asm/homecache.h>

#include "gxpci_endp_nic.h"


static const char driver_name[] = DRV_NAME_STRING;

/* Mask of CPUs that should receive PCIe interrupts. */
extern struct cpumask intr_cpus_map;

/* Actual VNIC port number. */
extern int host_vnic_ports[TILEGX_NUM_TRIO][TILEGX_TRIO_PCIES];


#if 0
#define GXPCI_DUMP_PACKETS
#endif

#if 0
#define GXPCI_RX_DROP_PACKETS
#endif


static irqreturn_t gxpci_nic_intr(int irq, void *nic_dev);

static void gxpci_alloc_rx_buffers(struct gxpci_nic *nic,
                                   struct gxpci_nic_ring *rx_ring,
                                   int cleaned_count);

static inline int gxpci_desc_unused(struct gxpci_nic_ring *ring);

#if GXPCI_NIC_QUEUE_DEBUG
static struct timer_list dump_timer[GXPCI_ENDP_NIC_COUNT];

static void dump(unsigned long arg)
{
	struct gxpci_nic *nic = (struct gxpci_nic *)arg;
	int i;

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_nic_ring *ring = &nic->tx_ring[i];
		printk("nic%d-tx_ring[%d]: cmds_posted %d, dmas_started %d, "
		       "dmas_completed %d\n", nic->if_num, i,
		       ring->commands_posted, ring->dmas_started,
		       ring->dmas_completed);
	}

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_nic_ring *ring = &nic->rx_ring[i];
		printk("nic%d-rx_ring[%d]: cmds_posted %d, dmas_started %d, "
		       "dmas_completed %d\n", nic->if_num, i,
		       ring->commands_posted, ring->dmas_started,
		       ring->dmas_completed);
	}

	mod_timer(&dump_timer[nic->if_num], jiffies + 10 * HZ);
}
#endif

#ifdef GXPCI_DUMP_PACKETS
/* Dump a packet. */
static void dump_packet(unsigned char *data, unsigned long length, char *s)
{
	unsigned long i;
	static unsigned int count = 0;

	printk("dump_packet(data %p, length %lx %s count %x)\n",
	       data, length, s, count++);

	printk("\n");

	for (i = 0; i < length; i++) {
		if ((i & 0xf) == 0) {
			printk("%8.8lx:", i);
		}
		printk(" %2.2x", data[i]);
		if (0xf == (i & 0xf)) {
			printk("\n");
		}
	}

	printk("\n\n");
}
#endif

/* DMA releated operations. */
static inline void pcie_dma_from_bus(gxio_trio_dma_queue_t *dma_queue,
				     uint64_t bus_addr, uint64_t va,
				     size_t size, uint64_t enable_intr)
{
	gxio_trio_dma_desc_t *desc_p;
	uint64_t gen;
	uint64_t xsize;
	uint32_t slot;

	slot = dma_queue->dma_queue.credits_and_next_index &
	       GXPCI_NIC_PULL_RING_MASK;
	desc_p = &dma_queue->dma_descs[slot];

	desc_p->io_address = bus_addr;
	__insn_flushwb();

	gen = ((dma_queue->dma_queue.credits_and_next_index >>
	       GXPCI_NIC_PULL_RING_GEN_BIT) ^ 1) & 1;

	/* SMOD is one bit below XSIZE.  We set SMOD if size is 16KB. */
	xsize = (size << 1) | (size >> TRIO_DMA_DESC_WORD0__XSIZE_WIDTH);

	*((uint64_t *)desc_p) =
		va | (xsize << TRIO_DMA_DESC_WORD0__SMOD_SHIFT) |
		(enable_intr << TRIO_DMA_DESC_WORD0__NOTIF_SHIFT) |
		(gen << TRIO_DMA_DESC_WORD0__GEN_SHIFT);

	gxio_trio_dma_queue_flush(dma_queue);

	dma_queue->dma_queue.credits_and_next_index++;
}

static inline void pcie_dma_to_bus(gxio_trio_dma_queue_t *dma_queue,
				   uint64_t bus_addr, uint64_t va, size_t size,
				   uint64_t enable_intr)
{
	gxio_trio_dma_desc_t *desc_p;
	uint64_t gen;
	uint64_t xsize;
	uint32_t slot;

	slot = dma_queue->dma_queue.credits_and_next_index &
	       GXPCI_NIC_PUSH_RING_MASK;
	desc_p = &dma_queue->dma_descs[slot];

	desc_p->io_address = bus_addr;
	__insn_flushwb();

	gen = ((dma_queue->dma_queue.credits_and_next_index >>
	       GXPCI_NIC_PUSH_RING_GEN_BIT) ^ 1) & 1;

	/* SMOD is one bit below XSIZE.  We set SMOD if size is 16KB. */
	xsize = (size << 1) | (size >> TRIO_DMA_DESC_WORD0__XSIZE_WIDTH);

	*((uint64_t *)desc_p) =
		va | (xsize << TRIO_DMA_DESC_WORD0__SMOD_SHIFT) |
		(enable_intr << TRIO_DMA_DESC_WORD0__NOTIF_SHIFT) |
		(gen << TRIO_DMA_DESC_WORD0__GEN_SHIFT);

	gxio_trio_dma_queue_flush(dma_queue);

	dma_queue->dma_queue.credits_and_next_index++;
}

static void gxpci_nic_process_h2t_cmds(struct gxpci_nic_ring *ring)
{
	gxio_trio_dma_queue_t *dma_queue_data =
		&ring->dma_resource.pull_dma_queue_data;
	tile_nic_dma_cmd_t *next_dma_cmd;
	tile_nic_dma_cmd_t *dma_cmd;
	uint64_t enable_intr = 0;
	uint32_t dma_cmd_index;

	while (1) {
		dma_cmd_index = ring->dmas_started & PCIE_CMD_QUEUE_ENTRY_MASK;
		dma_cmd = &ring->desc[dma_cmd_index];

		__insn_prefetch((void *)dma_cmd + 64);

		if (dma_cmd->host_desc.filled && dma_cmd->tile_desc.filled) {
			/* Get the next dma cmd. */
			next_dma_cmd = dma_cmd + 1;
			if (unlikely(dma_cmd_index ==
				     PCIE_CMD_QUEUE_ENTRY_MASK))
				next_dma_cmd = &ring->desc[0];

			/*
 			 * If the next dma cmd is not ready to process, enable
 			 * the DMA interrupt for the current dma cmd.
 			 */
			if (!next_dma_cmd->host_desc.filled ||
			    !next_dma_cmd->tile_desc.filled)
				enable_intr = 1;

			/* Both commands are ready and we issue the DMA. */
			dma_cmd->tile_desc.size = MIN(dma_cmd->tile_desc.size,
						      dma_cmd->host_desc.size);

			pcie_dma_from_bus(dma_queue_data, dma_cmd->host_addr,
					  dma_cmd->tile_addr,
					  (size_t)dma_cmd->tile_desc.size,
					  enable_intr);

			/*
 			 * Copy the per-packet VLAN flag and Tag to the tile
 			 * descriptor.
 			 */
			dma_cmd->tile_desc.vlan_packet =
				dma_cmd->host_desc.vlan_packet;
			dma_cmd->tile_desc.vlan_tag =
				dma_cmd->host_desc.vlan_tag;

			dma_cmd->host_desc.filled = 0;
			dma_cmd->tile_desc.filled = 0;
			ring->dmas_started++;
		} else
			break;
	}
}

static void gxpci_nic_process_t2h_cmds(struct gxpci_nic_ring *ring)
{
	struct gxpci_host_nic_queue_regs *host_regs = ring->regs;
	gxio_trio_dma_queue_t *dma_queue_data =
		&ring->dma_resource.push_dma_queue_data;
	tile_nic_dma_cmd_t *dma_cmd;
	uint32_t cmds_to_process;
	uint32_t dma_cmd_index;

	/* Get the number of host commands to process. */
	cmds_to_process = host_regs->cmds_posted_count - ring->dmas_started;
	while (cmds_to_process--) {
		dma_cmd_index = ring->dmas_started & PCIE_CMD_QUEUE_ENTRY_MASK;
		dma_cmd = &ring->desc[dma_cmd_index];

		__insn_prefetch((void *)dma_cmd + 64);

		/* Check valid tile side commands. */
		if (dma_cmd->tile_desc.filled) {
			/* Both commands are ready and we issue the DMA. */
			pcie_dma_to_bus(dma_queue_data, dma_cmd->host_addr,
					dma_cmd->tile_addr,
					(size_t)dma_cmd->tile_desc.size, 1);

			dma_cmd->tile_desc.filled = 0;
			ring->dmas_started++;
		} else
			break;
	}
}

static void gxpci_nic_process_h2t_comps(struct gxpci_nic_ring *ring)
{
	struct gxpci_host_nic_queue_regs *host_regs = ring->regs;
	struct tlr_pcie_dev *ep_dev = ring->dev;
	struct gxpci_nic *nic = ring->nic_dev;
	struct net_device *netdev = nic->netdev;
	struct gxpci_buffer *buffer_info;
	struct sk_buff *skb;
	gxio_trio_dma_queue_t *dma_queue_data =
		&ring->dma_resource.pull_dma_queue_data;
	tile_nic_dma_cmd_t *dma_cmd;
	unsigned int dma_cmds_completed;
	unsigned int ret_completions;
	unsigned int pkt_len;
	unsigned int count;
	uint16_t new_completions;
	volatile uint16_t hw_cnt;
	int cpl_index, result, i;
	int cleaned_count = 0;

	/*
 	 * Check if any data DMAs have been completed processed and if so,
 	 * generate host interrupt if needed.
 	 */
	dma_cmds_completed = ring->dmas_completed;
	hw_cnt = gxio_trio_read_dma_queue_complete_count(dma_queue_data);
	new_completions = (uint16_t)(hw_cnt -
		(dma_cmds_completed & TRIO_PULL_DMA_REGION_VAL__COUNT_RMASK));
	if (new_completions) {
		ring->dmas_completed += new_completions;

		/* Update the host cmds complete counter. */
		host_regs->cmds_consumed_count = ring->dmas_completed;
	}

	/* Hand completed data packets into kernel stack. */
	ret_completions = ring->dmas_completed - ring->commands_consumed;
	for (i = 0; i < ret_completions; i++) {
		cpl_index = ring->commands_consumed++ & ring->count_mask;
		buffer_info = &ring->buffer_infos[cpl_index];
		dma_cmd = &ring->desc[cpl_index];

		skb = buffer_info->skb;
		__insn_prefetch(skb->data - NET_IP_ALIGN);
		__insn_prefetch(buffer_info + 1);

		pkt_len = dma_cmd->tile_desc.size;

		buffer_info->dma = 0;
		buffer_info->skb = NULL;

		skb_put(skb, pkt_len);
		skb->dev = netdev;
		skb_record_rx_queue(skb, ring->index);
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb->protocol = eth_type_trans(skb, netdev);

#ifdef GXPCI_DUMP_PACKETS
		dump_packet(skb->data, skb->len, "rx");
#endif

#ifdef GXPCI_RX_DROP_PACKETS
		dev_kfree_skb_any(skb);
#else
		netif_rx(skb);
#endif
		nic->stats.rx_packets++;
		nic->stats.rx_bytes += skb->len;
		nic->netdev->last_rx = jiffies;

		cleaned_count++;

		/* Replenish some receive buffers, one at a time is too slow. */
		if (unlikely(cleaned_count >= GXPCI_RXD_REFILL_THRESHOLD)) {
			gxpci_alloc_rx_buffers(nic, ring, cleaned_count);
			cleaned_count = 0;
		}
	}

	/* Replenish some receive buffers again. */
	count = gxpci_desc_unused(ring);
	if (count)
		gxpci_alloc_rx_buffers(nic, ring, count);

	/*
 	 * Interrupt the host if it is enabled.
	 */
	if (host_regs->interrupt_enable) {
		if (!nic->vf_state)
			gxpci_update_msix_vector(&ring->msix_msg);
		if (nic->vf_state)
			result = gxio_trio_trigger_host_interrupt_vf(
				ep_dev->trio,
				ep_dev->mac, 0, ring->msix_msg.msix_addr,
				ring->msix_msg.msix_data,
				nic->vf_state->instance);
		else
			result = gxio_trio_trigger_host_interrupt(
				ep_dev->trio,
				ep_dev->mac, 0, ring->msix_msg.msix_addr,
				ring->msix_msg.msix_data);

		/*
 		 * Re-arm the timer to make sure that the endpoint can
 		 * trigger MSI/MSI-X interrupt successfully.
 		 */
		if (result)
			mod_timer(&ring->intr_timer,
				  jiffies + EP_INTR_TIMER_INTERVAL);
	}
}

static void gxpci_nic_process_t2h_comps(struct gxpci_nic_ring *ring)
{
	struct gxpci_host_nic_queue_regs *host_regs = ring->regs;
	struct tlr_pcie_dev *ep_dev = ring->dev;
	struct gxpci_nic *nic = ring->nic_dev;
	struct net_device *netdev = nic->netdev;
	struct gxpci_buffer *buffer_info;
	gxio_trio_dma_queue_t *dma_queue_data =
		&ring->dma_resource.push_dma_queue_data;
	unsigned int dma_cmds_completed;
	unsigned int ret_completions;
	uint16_t new_completions;
	volatile uint16_t hw_cnt;
	int cpl_index, result, i;

	/*
 	 * Check if any data DMAs have been completed processed and if so,
 	 * generate host interrupt if needed.
 	 */
	dma_cmds_completed = ring->dmas_completed;
	hw_cnt = gxio_trio_read_dma_queue_complete_count(dma_queue_data);
	new_completions = (uint16_t)(hw_cnt -
		(dma_cmds_completed & TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));
	if (new_completions) {
		ring->dmas_completed += new_completions;

		/* Update the host cmds complete counter. */
		host_regs->cmds_consumed_count = ring->dmas_completed;
	}

	/* Handle completed data packets. */
	ret_completions = ring->dmas_completed - ring->commands_consumed;
	for (i = 0; i < ret_completions; i++) {
		cpl_index = ring->commands_consumed++ & ring->count_mask;
		buffer_info = &ring->buffer_infos[cpl_index];

		if (buffer_info->dma)
			buffer_info->dma = 0;

		if (buffer_info->skb) {
			dev_kfree_skb_any(buffer_info->skb);
			buffer_info->skb = NULL;
		}

		nic->stats.tx_packets++;
		nic->stats.tx_bytes += buffer_info->length;
	}

	if (netif_carrier_ok(netdev) &&
	    gxpci_desc_unused(ring) >= GXPCI_WAKE_THRESHOLD) {
		smp_mb();

		if (__netif_subqueue_stopped(netdev, ring->index) &&
		    !(test_bit(__GXPCI_DOWN, &nic->state)))
			netif_wake_subqueue(netdev, ring->index);
	}

	/*
 	 * Interrupt the host if it is enabled.
	 */
	if (host_regs->interrupt_enable) {
		if (!nic->vf_state)
			gxpci_update_msix_vector(&ring->msix_msg);
		if (nic->vf_state)
			result = gxio_trio_trigger_host_interrupt_vf(
				ep_dev->trio,
				ep_dev->mac, 0, ring->msix_msg.msix_addr,
				ring->msix_msg.msix_data,
				nic->vf_state->instance);
		else
			result = gxio_trio_trigger_host_interrupt(
				ep_dev->trio,
				ep_dev->mac, 0, ring->msix_msg.msix_addr,
				ring->msix_msg.msix_data);

		/*
 		 * Re-arm the timer to make sure that the endpoint can
 		 * trigger MSI/MSI-X interrupt successfully.
 		 */
		if (result)
			mod_timer(&ring->intr_timer,
				  jiffies + EP_INTR_TIMER_INTERVAL);
	}
}

/*
 * gxpci_desc_unused - Calculate if we have unused descriptors.
 */
static inline int gxpci_desc_unused(struct gxpci_nic_ring *ring)
{
	return ring->count - (ring->commands_posted - ring->commands_consumed);
}

/*
 * gxpci_alloc_rx_buffers - Allocate new or replace used receive buffers.
 */
static void gxpci_alloc_rx_buffers(struct gxpci_nic *nic,
				   struct gxpci_nic_ring *rx_ring,
				   int cleaned_count)
{
	struct net_device *netdev = nic->netdev;
	struct gxpci_buffer *buffer_info;
	struct sk_buff *skb;
	unsigned int i, cmd_cnt;
	tile_nic_dma_cmd_t *dma;

	cmd_cnt = rx_ring->commands_posted;

	while (cleaned_count--) {
		i = cmd_cnt & rx_ring->count_mask;
		buffer_info = &rx_ring->buffer_infos[i];
		dma = &rx_ring->desc[i];

		skb = netdev_alloc_skb_ip_align(netdev, nic->rx_buffer_len);
		if (unlikely(!skb)) {
			ERR("gxpci_alloc_rx_buffers alloc_skb failed\n");
			break;
		}

		skb->dev = netdev;

		buffer_info->skb = skb;
		buffer_info->dma = (dma_addr_t) skb->data;

		/* Post new buffers to the Rx descriptor ring. */
		dma->tile_addr = __pa(buffer_info->dma);
		dma->tile_desc.size = nic->rx_buffer_len;
		dma->tile_desc.filled = 1;

		cmd_cnt++;
	}

	/* Make sure the commands are visible, then update the posted count. */
	wmb();
	rx_ring->commands_posted = cmd_cnt;
}

/*
 * DMA interrupt handler for a Rx ring.
 */
static irqreturn_t gxpci_rx_ring_dma_intr(int irq, void *rx_ring)
{
	struct gxpci_nic_ring *ring = (struct gxpci_nic_ring *)rx_ring;
	struct gxpci_nic *nic = ring->nic_dev;
	unsigned long flags;

	/* Lock down. */
	spin_lock_irqsave(&ring->cmd_queue_lock, flags);

	if (nic->channel_state == __GXPCI_NIC_UP)
		gxpci_nic_process_h2t_comps(ring);

	/* Unlock. */
	spin_unlock_irqrestore(&ring->cmd_queue_lock, flags);

	return IRQ_HANDLED;
}

/*
 * DMA interrupt handler for a Tx ring.
 */
static irqreturn_t gxpci_tx_ring_dma_intr(int irq, void *tx_ring)
{
	struct gxpci_nic_ring *ring = (struct gxpci_nic_ring *)tx_ring;
	struct gxpci_nic *nic = ring->nic_dev;
	unsigned long flags;

	/* Lock down. */
	spin_lock_irqsave(&ring->cmd_queue_lock, flags);

	if (nic->channel_state == __GXPCI_NIC_UP)
		gxpci_nic_process_t2h_comps(ring);

	/* Unlock. */
	spin_unlock_irqrestore(&ring->cmd_queue_lock, flags);

	return IRQ_HANDLED;
}

static void gxpci_nic_intr_timer_handler(unsigned long arg)
{
	struct gxpci_nic_ring *ring = (struct gxpci_nic_ring *)arg;
	struct tlr_pcie_dev *ep_dev = ring->dev;
	gxio_trio_context_t *context = ep_dev->trio;
	int result = 1;

	if (ring->regs->interrupt_enable) {
		struct gxpci_nic *nic = ring->nic_dev;

		if (!nic->vf_state)
			gxpci_update_msix_vector(&ring->msix_msg);
		if (nic->vf_state)
			result = gxio_trio_trigger_host_interrupt_vf(context,
				ep_dev->mac, 0, ring->msix_msg.msix_addr,
				ring->msix_msg.msix_data,
				nic->vf_state->instance);
		else
			result = gxio_trio_trigger_host_interrupt(context,
				ep_dev->mac, 0, ring->msix_msg.msix_addr,
				ring->msix_msg.msix_data);
	}

	/*
	 * Re-arm the timer to make sure that the endpoint can trigger MSI/MSI-X
	 * interrupt successfully.
	 */
	if (result)
		mod_timer(&ring->intr_timer, jiffies + EP_INTR_TIMER_INTERVAL);
}

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

/* Initialize a newly opened stream. */
static int gxpci_nic_stream_open(struct gxpci_nic *nic, int in_reset)
{
	gxio_trio_context_t *trio_context = &nic->trio;
	gxio_trio_dma_queue_t *dma_queue;
	struct net_device *netdev = nic->netdev;
	struct tlr_pcie_dev *ep_dev = nic->tlr;
	struct gxpci_nic_ring *rx_ring;
	struct gxpci_nic_ring *tx_ring;
	struct page *page;
	int pull_dma_ring, push_dma_ring;
	int tx, rx, asid, err, i;
	int msix_table_index;
	int tx_cpu, rx_cpu;
	int tx_irq, rx_irq;
	size_t dma_ring_size;
	void *dma_ring_mem;
	void *backing_mem;
	pte_t pte = { 0 };

	/* Initialize the empty TRIO context per virtual network interface. */
	err = gxio_trio_init(trio_context, ep_dev->trio_index);
	if (err < 0) {
		ERR("gxio_trio_init() failed %d.\n", err);
		return -ENXIO;
	}

	/* Allocate an ASID for DMA data, i.e. kernel skb. */
	err = gxio_trio_alloc_asids(trio_context, 1, 0, 0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_alloc_asids() failed %d.\n",
		    driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto alloc_asid_failed;
	}
	asid = err;

	/* Allocate an ASID for backing_mem. */
	err = gxio_trio_alloc_asids(trio_context, 1, 0, 0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_alloc_asids() failed %d.\n",
		    driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto alloc_asid_failed;
	}
	trio_context->asid = err;

	pte = pte_set_home(pte, PAGE_HOME_HASH);
	err = gxio_trio_register_client_memory(trio_context, asid, pte, 0);
	if (err) {
		ERR("%s: TRIO %d MAC %d gxio_trio_register_client_memory() "
		    "failed %d.\n", driver_name, ep_dev->trio_index,
		    ep_dev->mac, err);
		goto reg_client_mem_failed;
	}	

	if (!in_reset) {
		/* Allocate a huge page for all Tx and Rx rings. */
		page = alloc_pages(GFP_KERNEL | __GFP_ZERO,
				   get_order(HPAGE_SIZE));
		if (!page) {
			err = -ENOMEM;
			ERR("%s: TRIO %d MAC %d alloc_pages() failed %d.\n",
			    driver_name, ep_dev->trio_index, ep_dev->mac, err);
			goto alloc_page_failed;
		}

		backing_mem = page_address(page);
		nic->backing_mem = backing_mem;
	} else {
		/* Reuse backing memory in case of reset. */
		backing_mem = nic->backing_mem;
	}

	/* Reset backing memory, especially for those DMA related buffers. */
	memset(backing_mem, 0, HPAGE_SIZE);

	/* Register this DMA memory. */
	err = gxio_trio_register_page(trio_context, trio_context->asid,
				      nic->backing_mem, HPAGE_SIZE, 0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_register_page() failed %d.\n",
		    driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto reg_mem_failed;
	}
	dma_ring_mem = backing_mem;

	/* Allocate DMA channels. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];

		/* Allocate and initialize a pull DMA ring for the skb data. */
		pull_dma_ring =
			gxio_trio_alloc_pull_dma_ring(trio_context, 1, 0, 0);
		if (pull_dma_ring < 0) {
			err = -ENXIO;
			ERR("%s: TRIO %d MAC %d gxio_trio_alloc_pull_dma_ring()"
			    " failed %d.\n", driver_name, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto dma_failed;
		}

		/*
		 * Bind the DMA ring to our MAC, and use registered memory to
		 * store the command ring.
		 */
		dma_ring_size = GXPCI_HOST_NIC_PULL_DMA_RING_LEN *
			sizeof(gxio_trio_dma_desc_t);
		dma_queue = &rx_ring->dma_resource.pull_dma_queue_data;
		err = gxio_trio_init_pull_dma_queue(dma_queue, trio_context,
						    pull_dma_ring, ep_dev->mac,
						    asid, 0, dma_ring_mem,
						    dma_ring_size, 0);
		if (err < 0) {
			ERR("%s: TRIO %d MAC %d gxio_trio_init_pull_dma_queue()"
			    " failed %d.\n", driver_name, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto dma_failed;
		}
		rx_ring->dma_resource.pull_dma_data = pull_dma_ring;
		dma_ring_mem += dma_ring_size;

		tx_ring = &nic->tx_ring[i];

		/* Allocate and initialize a push DMA ring for the skb data. */
		push_dma_ring =
			gxio_trio_alloc_push_dma_ring(trio_context, 1, 0, 0);
		if (push_dma_ring < 0) {
			err = -ENXIO;
			ERR("%s: TRIO %d MAC %d gxio_trio_alloc_push_dma_ring()"
			    " failed %d.\n", driver_name, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto dma_failed;
		}

		/*
		 * Bind the DMA ring to our MAC, and use registered memory to
		 * store the command ring.
		 */
		dma_ring_size = GXPCI_HOST_NIC_PUSH_DMA_RING_LEN *
			sizeof(gxio_trio_dma_desc_t);
		dma_queue = &tx_ring->dma_resource.push_dma_queue_data;
		err = gxio_trio_init_push_dma_queue(dma_queue, trio_context,
						    push_dma_ring, ep_dev->mac,
						    asid, 0, dma_ring_mem,
						    dma_ring_size, 0);
		if (err < 0) {
			ERR("%s: TRIO %d MAC %d gxio_trio_init_push_dma_queue()"
			    " failed %d.\n", driver_name, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto dma_failed;
		}
		tx_ring->dma_resource.push_dma_data = push_dma_ring;
		dma_ring_mem += dma_ring_size;
	}

	/* Allocate an interrupt for each DMA channel. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];

		rx_irq = create_irq();
		if (rx_irq < 0) {
			err = -ENXIO;
			ERR("%s: TRIO %d MAC %d create_irq() failed %d.\n",
			    driver_name, ep_dev->trio_index, ep_dev->mac, err);
			goto create_irq_failed;
		}
		rx_ring->dma_resource.pull_dma_data_irq = rx_irq;

		tile_irq_activate(rx_irq, TILE_IRQ_PERCPU);
	}

	for (i = 0; i < nic->num_tx_queues; i++) {
		tx_ring = &nic->tx_ring[i];

		tx_irq = create_irq();
		if (tx_irq < 0) {
			err = -ENXIO;
			ERR("%s: TRIO %d MAC %d create_irq() failed %d.\n",
			    driver_name, ep_dev->trio_index, ep_dev->mac, err);
			goto create_irq_failed;
		}
		tx_ring->dma_resource.push_dma_data_irq = tx_irq;

		tile_irq_activate(tx_irq, TILE_IRQ_PERCPU);
	}

	/* Register an interrupt for each pull/push DMA channel. */
	for (rx = 0; rx < nic->num_rx_queues; rx++) {
		rx_ring = &nic->rx_ring[rx];
		sprintf(rx_ring->name, "%s-%s-%d", netdev->name, "Rx", rx);
		if (request_irq(rx_ring->dma_resource.pull_dma_data_irq,
				gxpci_rx_ring_dma_intr, 0, rx_ring->name,
				(void *)rx_ring)) {
			err = -ENXIO;
			ERR("%s: TRIO %d MAC %d request_irq() failed %d.\n",
			    driver_name, ep_dev->trio_index, ep_dev->mac, err);
			goto req_pull_dma_irq_failed;
		}
	}

	for (tx = 0; tx < nic->num_tx_queues; tx++) {
		tx_ring = &nic->tx_ring[tx];
		sprintf(tx_ring->name, "%s-%s-%d", netdev->name, "Tx", tx);
		if (request_irq(tx_ring->dma_resource.push_dma_data_irq,
				gxpci_tx_ring_dma_intr, 0, tx_ring->name,
				(void *)tx_ring)) {
			err = -ENXIO;
			ERR("%s: TRIO %d MAC %d request_irq() failed %d.\n",
			    driver_name, ep_dev->trio_index, ep_dev->mac, err);
			goto req_push_dma_irq_failed;
		}
	}

	/* Configure interrupts. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		/*
 		 * Try to choose non-dataplane processors to receive the DMA
 		 * done interrupts.
	 	 */
		rx_ring = &nic->rx_ring[i];
		rx_irq = rx_ring->dma_resource.pull_dma_data_irq;
		rx_cpu = tile_irq_cpu(rx_irq);

		/* Enable pull DMA ring interrupt. */
		err = gxio_trio_config_char_intr(trio_context,
			cpu_x(rx_cpu), cpu_y(rx_cpu),
			KERNEL_PL, rx_irq,
			ep_dev->mac, 0, 0,
			rx_ring->dma_resource.pull_dma_data,
			PULL_DMA_SEL);
		if (err < 0) {
			ERR("%s: TRIO %d MAC %d gxio_trio_config_char_intr() "
			    "failed %d.\n", driver_name, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto cfg_irq_hv_failed;
		}
	}

	for (i = 0; i < nic->num_tx_queues; i++) {
		/*
 		 * Try to choose non-dataplane processors to receive the DMA
 		 * done interrupts.
	 	 */
		tx_ring = &nic->tx_ring[i];
		tx_irq = tx_ring->dma_resource.push_dma_data_irq;
		tx_cpu = tile_irq_cpu(tx_irq);

		/* Enable push DMA ring interrupt. */
		err = gxio_trio_config_char_intr(trio_context,
			cpu_x(tx_cpu), cpu_y(tx_cpu),
			KERNEL_PL, tx_irq,
			ep_dev->mac, 0,
			tx_ring->dma_resource.push_dma_data,
			0,
			PUSH_DMA_SEL);
		if (err < 0) {
			ERR("%s: TRIO %d MAC %d gxio_trio_config_char_intr() "
			    "failed %d.\n", driver_name, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto cfg_irq_hv_failed;
		}

	}

	/* Reset intialization. */
	for (i = 0; i < nic->num_tx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		tx_ring = &nic->tx_ring[i];

		/* Initialize MMIO registers that are read by the host. */
		rx_ring->regs->cmds_consumed_count = 0;
		tx_ring->regs->cmds_consumed_count = 0;

		/* Initialize MSI-X info. */
		msix_table_index = ep_dev->msix_host_nic_intr_vec_base +
				   nic->if_num * nic->int_vectors + i;

		tx_ring->msix_msg.msix_table_entry_offset = 
			ep_dev->msix_table_base +
			PCI_MSIX_ENTRY_SIZE * msix_table_index;
		rx_ring->msix_msg.msix_table_entry_offset =
			tx_ring->msix_msg.msix_table_entry_offset; 
		tx_ring->msix_msg.msix_addr = *(unsigned long *)
			(tx_ring->msix_msg.msix_table_entry_offset +
			 PCI_MSIX_ENTRY_LOWER_ADDR);
		rx_ring->msix_msg.msix_addr = tx_ring->msix_msg.msix_addr;
		tx_ring->msix_msg.msix_data = *(unsigned int *)
			(tx_ring->msix_msg.msix_table_entry_offset +
			 PCI_MSIX_ENTRY_DATA);
		rx_ring->msix_msg.msix_data = tx_ring->msix_msg.msix_data;

	        /*
        	 * Initialize per ring timers to monitor any failure when
	         * trying to trigger MSI/MSI-X interrupts to the RC.
        	 */
	        init_timer(&rx_ring->intr_timer);
        	init_timer(&tx_ring->intr_timer);

	        rx_ring->intr_timer.data = (unsigned long)rx_ring;
        	rx_ring->intr_timer.function = gxpci_nic_intr_timer_handler;

	        tx_ring->intr_timer.data = (unsigned long)tx_ring;
        	tx_ring->intr_timer.function = gxpci_nic_intr_timer_handler;

		/* Initialize per ring DMA counters. */
		rx_ring->dmas_started = 0;
		rx_ring->dmas_completed = 0;

		tx_ring->dmas_started = 0;
		tx_ring->dmas_completed = 0;
	}

	return 0;

cfg_irq_hv_failed:
req_push_dma_irq_failed:
	while (tx--) {
		tx_ring = &nic->tx_ring[tx];
		free_irq(tx_ring->dma_resource.push_dma_data_irq, tx_ring);
	}
req_pull_dma_irq_failed:
	while (rx--) {
		rx_ring = &nic->rx_ring[rx];
		free_irq(rx_ring->dma_resource.pull_dma_data_irq, rx_ring);
	}
create_irq_failed:
	for (i = 0; i < nic->num_rx_queues; i++) {
		tx_ring = &nic->tx_ring[i];
		if (tx_ring->dma_resource.push_dma_data_irq != -1) {
			destroy_irq(tx_ring->dma_resource.push_dma_data_irq);
			tx_ring->dma_resource.push_dma_data_irq = -1;
		}

		rx_ring = &nic->rx_ring[i];
		if (rx_ring->dma_resource.pull_dma_data_irq != -1) {
			destroy_irq(rx_ring->dma_resource.pull_dma_data_irq);
			rx_ring->dma_resource.pull_dma_data_irq = -1;
		}
	}
dma_failed:
	gxio_trio_unregister_page(trio_context, trio_context->asid,
				  nic->backing_mem);
reg_mem_failed:
	if (!in_reset) {
		homecache_free_pages((unsigned long)nic->backing_mem,
				     get_order(HPAGE_SIZE));
	}
alloc_page_failed:
reg_client_mem_failed:
alloc_asid_failed:
	hv_dev_close(trio_context->fd);

	return err;
}

/* Release resources allocated by gxpci_nic_stream_open. */
static void gxpci_nic_stream_close(struct gxpci_nic *nic, int in_reset)
{
	gxio_trio_context_t *trio_context = &nic->trio;
	struct gxpci_nic_ring *rx_ring;
	struct gxpci_nic_ring *tx_ring;
	unsigned int rx_irq, tx_irq;
	int i;

	/* Remove the per ring timer. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		if (rx_ring->intr_timer.data)
			del_timer(&rx_ring->intr_timer);

		tx_ring = &nic->tx_ring[i];
		if (tx_ring->intr_timer.data)
			del_timer(&tx_ring->intr_timer);
	}

	/* Unregister the backing memory. */
	gxio_trio_unregister_page(trio_context, trio_context->asid,
				  nic->backing_mem);

	/* Free the TRIO context and all allocated resources. */
	if (trio_context->fd >= 0)
		hv_dev_close(trio_context->fd);

	/* Release Rx/Tx irqs that we have requested. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		rx_irq = rx_ring->dma_resource.pull_dma_data_irq;
		if (rx_irq != -1) {
			free_irq(rx_irq, rx_ring);
			destroy_irq(rx_irq);
			rx_ring->dma_resource.pull_dma_data_irq = -1;
		}

		tx_ring = &nic->tx_ring[i];
		tx_irq = tx_ring->dma_resource.push_dma_data_irq;
		if (tx_irq != -1) {
			free_irq(tx_irq, tx_ring);
			destroy_irq(tx_irq);
			tx_ring->dma_resource.push_dma_data_irq = -1;
		}
	}

	/* Free the DMA memory. */
	if (!in_reset && nic->backing_mem) {
		homecache_free_pages((unsigned long)nic->backing_mem,
				     get_order(HPAGE_SIZE));
		nic->backing_mem = NULL;
	}
}

/* Initialize a newly opened stream. */
static int gxpci_nic_stream_open_vf(struct gxpci_nic *nic, int in_reset)
{
	gxio_trio_context_t *trio_context = &nic->trio;
	gxio_trio_dma_queue_t *dma_queue;
	struct net_device *netdev = nic->netdev;
	struct tlr_pcie_dev *ep_dev = nic->tlr;
	struct gxpci_ep_vf_state *vf_state;
	struct gxpci_nic_ring *rx_ring;
	struct gxpci_nic_ring *tx_ring;
	int pull_dma_ring, push_dma_ring;
	int tx, rx, asid, err, i;
	int msix_table_index;
	int tx_irq, rx_irq;
	size_t dma_ring_size;
	void *dma_ring_mem;
	pte_t pte = { 0 };
	int cpu;
	int vf;

	vf_state = nic->vf_state;
	vf = vf_state->instance;

	/* Initialize the empty TRIO context per virtual network interface. */
	err = gxio_trio_init(trio_context, ep_dev->trio_index);
	if (err < 0) {
		ERR("VF %d: gxio_trio_init() failed %d.\n", vf, err);
		return -ENXIO;
	}

	/* Allocate an ASID for DMA data, i.e. kernel skb. */
	asid = gxio_trio_alloc_asids(trio_context, 1, 0, 0);
	if (err < 0) {
		ERR("VF %d: TRIO %d MAC %d gxio_trio_alloc_asids() failed %d.\n",
		    vf, ep_dev->trio_index, ep_dev->mac, asid);
		goto alloc_asid_failed;
	}
	trio_context->asid = asid;

	pte = pte_set_home(pte, PAGE_HOME_HASH);
	err = gxio_trio_register_client_memory(trio_context, asid, pte, 0);
	if (err) {
		ERR("VF %d: TRIO %d MAC %d gxio_trio_register_client_memory() "
		    "failed %d.\n", vf, ep_dev->trio_index, ep_dev->mac, err);
		goto reg_client_mem_failed;
	}

	/*
	 * Clear backing memory for DMA descriptor rings, which are located
	 * in the part of the huge page that is not mapped to the BAR0 space.
	 */
	memset(nic->backing_mem, 0, HPAGE_SIZE - GXPCI_VF_HOST_NIC_MAP_SIZE);

	dma_ring_mem = nic->backing_mem;

	/* Allocate DMA channels. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];

		/* Allocate and initialize a pull DMA ring for the skb data. */
		pull_dma_ring =
			gxio_trio_alloc_pull_dma_ring(trio_context, 1, 0, 0);
		if (pull_dma_ring < 0) {
			err = -ENXIO;
			ERR("VF %d: TRIO %d MAC %d alloc_pull_dma_ring()"
			    " failed %d.\n", vf, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto dma_failed;
		}

		/*
		 * Bind the DMA ring to our MAC, and use registered memory to
		 * store the command ring.
		 */
		dma_ring_size = GXPCI_HOST_NIC_PULL_DMA_RING_LEN *
			sizeof(gxio_trio_dma_desc_t);
		dma_queue = &rx_ring->dma_resource.pull_dma_queue_data;
		err = gxio_trio_init_pull_dma_queue(dma_queue, trio_context,
						    pull_dma_ring, ep_dev->mac,
						    asid, HV_TRIO_FLAG_VFUNC(vf),
						    dma_ring_mem, dma_ring_size, 0);
		if (err < 0) {
			ERR("VF %d: TRIO %d MAC %d init_pull_dma_queue()"
			    " failed %d.\n", vf, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto dma_failed;
		}
		rx_ring->dma_resource.pull_dma_data = pull_dma_ring;
		dma_ring_mem += dma_ring_size;

		tx_ring = &nic->tx_ring[i];

		/* Allocate and initialize a push DMA ring for the skb data. */
		push_dma_ring =
			gxio_trio_alloc_push_dma_ring(trio_context, 1, 0, 0);
		if (push_dma_ring < 0) {
			err = -ENXIO;
			ERR("VF %d: TRIO %d MAC %d alloc_push_dma_ring()"
			    " failed %d.\n", vf, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto dma_failed;
		}

		/*
		 * Bind the DMA ring to our MAC, and use registered memory to
		 * store the command ring.
		 */
		dma_ring_size = GXPCI_HOST_NIC_PUSH_DMA_RING_LEN *
			sizeof(gxio_trio_dma_desc_t);
		dma_queue = &tx_ring->dma_resource.push_dma_queue_data;
		err = gxio_trio_init_push_dma_queue(dma_queue, trio_context,
						    push_dma_ring, ep_dev->mac,
						    asid, HV_TRIO_FLAG_VFUNC(vf),
						    dma_ring_mem, dma_ring_size, 0);
		if (err < 0) {
			ERR("VF %d: TRIO %d MAC %d init_push_dma_queue()"
			    " failed %d.\n", vf, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto dma_failed;
		}
		tx_ring->dma_resource.push_dma_data = push_dma_ring;
		dma_ring_mem += dma_ring_size;
	}

	/* Allocate and configure an interrupt for each DMA channel. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];

		rx_irq = create_irq();
		if (rx_irq < 0) {
			err = -ENXIO;
			ERR("VF %d: TRIO %d MAC %d create_irq() failed %d.\n",
			    vf, ep_dev->trio_index, ep_dev->mac, err);
			goto create_irq_failed;
		}
		rx_ring->dma_resource.pull_dma_data_irq = rx_irq;

		tx_ring = &nic->tx_ring[i];

		tx_irq = create_irq();
		if (tx_irq < 0) {
			err = -ENXIO;
			ERR("VF %d: TRIO %d MAC %d create_irq() failed %d.\n",
			    vf, ep_dev->trio_index, ep_dev->mac, err);
			goto create_irq_failed;
		}
		tx_ring->dma_resource.push_dma_data_irq = tx_irq;

		/*
 		 * Try to choose non-dataplane processors to receive the DMA
 		 * done interrupts.
	 	 */
		cpu = tile_irq_cpu(rx_irq);

		/* Enable pull DMA ring interrupt. */
		err = gxio_trio_config_char_intr(trio_context,
			cpu_x(cpu), cpu_y(cpu),
			KERNEL_PL, rx_irq,
			ep_dev->mac, 0, 0,
			rx_ring->dma_resource.pull_dma_data,
			PULL_DMA_SEL);
		if (err < 0) {
			ERR("VF %d: TRIO %d MAC %d gxio_trio_config_char_intr() "
			    "failed %d.\n", vf, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto create_irq_failed;
		}

		cpu = tile_irq_cpu(tx_irq);

		/* Enable push DMA ring interrupt. */
		err = gxio_trio_config_char_intr(trio_context,
			cpu_x(cpu), cpu_y(cpu),
			KERNEL_PL, tx_irq,
			ep_dev->mac, 0,
			tx_ring->dma_resource.push_dma_data,
			0,
			PUSH_DMA_SEL);
		if (err < 0) {
			ERR("VF %d: TRIO %d MAC %d gxio_trio_config_char_intr() "
			    "failed %d.\n", vf, ep_dev->trio_index,
			    ep_dev->mac, err);
			goto create_irq_failed;
		}

		tile_irq_activate(tx_irq, TILE_IRQ_PERCPU);
		tile_irq_activate(rx_irq, TILE_IRQ_PERCPU);
	}

	/* Register an interrupt for each pull DMA channel. */
	for (rx = 0; rx < nic->num_rx_queues; rx++) {
		rx_ring = &nic->rx_ring[rx];
		sprintf(rx_ring->name, "%s-%s-%d", netdev->name, "Rx", rx);
		if (request_irq(rx_ring->dma_resource.pull_dma_data_irq,
				gxpci_rx_ring_dma_intr, 0, rx_ring->name,
				(void *)rx_ring)) {
			err = -ENXIO;
			ERR("VF %d: TRIO %d MAC %d request_irq() failed %d.\n",
			    vf, ep_dev->trio_index, ep_dev->mac, err);
			goto req_pull_dma_irq_failed;
		}
	}

	/* Register an interrupt for each push DMA channel. */
	for (tx = 0; tx < nic->num_tx_queues; tx++) {
		tx_ring = &nic->tx_ring[tx];
		sprintf(tx_ring->name, "%s-%s-%d", netdev->name, "Tx", tx);
		if (request_irq(tx_ring->dma_resource.push_dma_data_irq,
				gxpci_tx_ring_dma_intr, 0, tx_ring->name,
				(void *)tx_ring)) {
			err = -ENXIO;
			ERR("VF %d: TRIO %d MAC %d request_irq() failed %d.\n",
			    vf, ep_dev->trio_index, ep_dev->mac, err);
			goto req_push_dma_irq_failed;
		}
	}

	for (i = 0; i < nic->num_tx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		tx_ring = &nic->tx_ring[i];

		/* Initialize MMIO registers that are read by the host. */
		rx_ring->regs->cmds_consumed_count = 0;
		tx_ring->regs->cmds_consumed_count = 0;

		/* Initialize MSI-X info. */
		msix_table_index = ep_dev->msix_host_nic_intr_vec_base +
				   nic->if_num * nic->int_vectors + i;

#if 0
		tx_ring->msix_msg.msix_table_entry_offset = 
			nic->vf_state->msix_table_base +
			PCI_MSIX_ENTRY_SIZE * msix_table_index;
		rx_ring->msix_msg.msix_table_entry_offset =
			tx_ring->msix_msg.msix_table_entry_offset; 
		tx_ring->msix_msg.msix_addr = *(unsigned long *)
			(tx_ring->msix_msg.msix_table_entry_offset +
			 PCI_MSIX_ENTRY_LOWER_ADDR);
		rx_ring->msix_msg.msix_addr = tx_ring->msix_msg.msix_addr;
		tx_ring->msix_msg.msix_data = *(unsigned int *)
			(tx_ring->msix_msg.msix_table_entry_offset +
			 PCI_MSIX_ENTRY_DATA);
		rx_ring->msix_msg.msix_data = tx_ring->msix_msg.msix_data;
#endif

	        /*
        	 * Initialize per ring timers to monitor any failure when
	         * trying to trigger MSI/MSI-X interrupts to the RC.
        	 */
	        init_timer(&rx_ring->intr_timer);
        	init_timer(&tx_ring->intr_timer);

	        rx_ring->intr_timer.data = (unsigned long)rx_ring;
        	rx_ring->intr_timer.function = gxpci_nic_intr_timer_handler;

	        tx_ring->intr_timer.data = (unsigned long)tx_ring;
        	tx_ring->intr_timer.function = gxpci_nic_intr_timer_handler;

		/* Initialize per ring DMA counters. */
		rx_ring->dmas_started = 0;
		rx_ring->dmas_completed = 0;

		tx_ring->dmas_started = 0;
		tx_ring->dmas_completed = 0;
	}

	return 0;

req_push_dma_irq_failed:
	while (tx--) {
		tx_ring = &nic->tx_ring[tx];
		free_irq(tx_ring->dma_resource.push_dma_data_irq, tx_ring);
	}
req_pull_dma_irq_failed:
	while (rx--) {
		rx_ring = &nic->rx_ring[rx];
		free_irq(rx_ring->dma_resource.pull_dma_data_irq, rx_ring);
	}
create_irq_failed:
	for (i = 0; i < nic->num_rx_queues; i++) {
		tx_ring = &nic->tx_ring[i];
		if (tx_ring->dma_resource.push_dma_data_irq != -1) {
			destroy_irq(tx_ring->dma_resource.push_dma_data_irq);
			tx_ring->dma_resource.push_dma_data_irq = -1;
		}

		rx_ring = &nic->rx_ring[i];
		if (rx_ring->dma_resource.pull_dma_data_irq != -1) {
			destroy_irq(rx_ring->dma_resource.pull_dma_data_irq);
			rx_ring->dma_resource.pull_dma_data_irq = -1;
		}
	}
dma_failed:
reg_client_mem_failed:
alloc_asid_failed:
	hv_dev_close(trio_context->fd);

	return err;
}

/* Release resources allocated by gxpci_nic_stream_open. */
static void gxpci_nic_stream_close_vf(struct gxpci_nic *nic, int in_reset)
{
	gxio_trio_context_t *nic_trio_context = &nic->trio;
	struct gxpci_nic_ring *rx_ring;
	struct gxpci_nic_ring *tx_ring;
	unsigned int rx_irq, tx_irq;
	int i;

	/* Remove the per ring timer. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		if (rx_ring->intr_timer.data)
			del_timer(&rx_ring->intr_timer);

		tx_ring = &nic->tx_ring[i];
		if (tx_ring->intr_timer.data)
			del_timer(&tx_ring->intr_timer);
	}

	/* Free the TRIO context and all allocated resources. */
	if (nic_trio_context->fd >= 0)
		hv_dev_close(nic_trio_context->fd);

	/* Release Rx/Tx irqs that we have requested. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		rx_irq = rx_ring->dma_resource.pull_dma_data_irq;
		if (rx_irq != -1) {
			free_irq(rx_irq, rx_ring);
			destroy_irq(rx_irq);
			rx_ring->dma_resource.pull_dma_data_irq = -1;
		}

		tx_ring = &nic->tx_ring[i];
		tx_irq = tx_ring->dma_resource.push_dma_data_irq;
		if (tx_irq != -1) {
			free_irq(tx_irq, tx_ring);
			destroy_irq(tx_irq);
			tx_ring->dma_resource.push_dma_data_irq = -1;
		}
	}
}

/*
 * gxpci_setup_tx_resources - Allocate Tx resources (Descriptors).
 * @nic: board private structure.
 *
 * Return 0 on success, negative on failure.
 */
static int gxpci_setup_tx_resources(struct gxpci_nic *nic)
{
	int err = -ENOMEM;
	int tx;

	for (tx = 0; tx < nic->num_tx_queues; tx++) {
		struct gxpci_nic_ring *tx_ring = &nic->tx_ring[tx];
		int size;

		tx_ring->index = tx;

		size = sizeof(struct gxpci_buffer) * tx_ring->count;
		tx_ring->buffer_infos = vzalloc(size);
		if (!tx_ring->buffer_infos)
			goto err_buffer_infos;

		if (nic->vf_state)
			tx_ring->desc = nic->nic_desc_vf->t2h_desc[tx].dma_cmds;
		else
			tx_ring->desc = nic->nic_desc->t2h_desc[tx].dma_cmds;

		tx_ring->commands_posted = 0;
		tx_ring->commands_consumed = 0;
	}

	return 0;

err_buffer_infos:
	while (tx--) {
		struct gxpci_nic_ring *tx_ring = &nic->tx_ring[tx];
		vfree(tx_ring->buffer_infos);
		tx_ring->buffer_infos = NULL;
		tx_ring->desc = NULL;
	}
	ERR("Unable to allocate memory "
	    "for the transmit descriptor ring\n");
	return err;
}

/*
 * gxpci_setup_rx_resources - Allocate Rx resources (Descriptors).
 * @nic: board private structure.
 *
 * Return 0 on success, negative on failure.
 */
static int gxpci_setup_rx_resources(struct gxpci_nic *nic)
{
	int err = -ENOMEM;
	int rx;

	for (rx = 0; rx < nic->num_rx_queues; rx++) {
		struct gxpci_nic_ring *rx_ring = &nic->rx_ring[rx];
		int size;

		rx_ring->index = rx;

		size = sizeof(struct gxpci_buffer) * rx_ring->count;
		rx_ring->buffer_infos = vzalloc(size);
		if (!rx_ring->buffer_infos)
			goto err_buffer_infos;

		if (nic->vf_state)
			rx_ring->desc = nic->nic_desc_vf->h2t_desc[rx].dma_cmds; 
		else
			rx_ring->desc = nic->nic_desc->h2t_desc[rx].dma_cmds; 

		rx_ring->commands_posted = 0;
		rx_ring->commands_consumed = 0;
	}

	return 0;

err_buffer_infos:
	while (rx--) {
		struct gxpci_nic_ring *rx_ring = &nic->rx_ring[rx];
		vfree(rx_ring->buffer_infos);
		rx_ring->buffer_infos = NULL;
		rx_ring->desc = NULL;
	}
	ERR("Unable to allocate memory "
	    "for the receive descriptor ring\n");
	return err;
}

/*
 * gxpci_clean_tx_ring - Free Tx Buffers.
 * @tx_ring: descriptor ring to transact packets on.
 */
static void gxpci_clean_tx_ring(struct gxpci_nic_ring *tx_ring)
{
	struct gxpci_buffer *buffer_info;
	unsigned long size;
	unsigned int i;

	for (i = 0; i < tx_ring->count; i++) {
		buffer_info = &tx_ring->buffer_infos[i];

		if (buffer_info->skb) {
			dev_kfree_skb_any(buffer_info->skb);
			buffer_info->skb = NULL;
		}
	}

	size = sizeof(struct gxpci_buffer) * tx_ring->count;
	memset(tx_ring->buffer_infos, 0, size);

	tx_ring->commands_posted = 0;
	tx_ring->commands_consumed = 0;
}

/*
 * gxpci_clean_rx_ring - Free Rx buffers.
 * @rx_ring: descriptor ring to transact packets on.
 */
static void gxpci_clean_rx_ring(struct gxpci_nic_ring *rx_ring)
{
	struct gxpci_buffer *buffer_info;
	unsigned int i, size;

	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rx_ring->count; i++) {
		buffer_info = &rx_ring->buffer_infos[i];

		if (buffer_info->skb) {
			dev_kfree_skb_any(buffer_info->skb);
			buffer_info->skb = NULL;
		}
	}

	size = sizeof(struct gxpci_buffer) * rx_ring->count;
	memset(rx_ring->buffer_infos, 0, size);

	rx_ring->commands_posted = 0;
	rx_ring->commands_consumed = 0;
}

/*
 * gxpci_free_tx_resources - Free Tx resources per queue.
 * @nic: board private structure.
 *
 * Free all transmit software resources.
 */
static void gxpci_free_tx_resources(struct gxpci_nic *nic)
{
	int tx;

	for (tx = 0; tx < nic->num_tx_queues; tx++) {
		struct gxpci_nic_ring *tx_ring = &nic->tx_ring[tx];

		gxpci_clean_tx_ring(tx_ring);

		vfree(tx_ring->buffer_infos);
		tx_ring->buffer_infos = NULL;
		tx_ring->desc = NULL;
	}
}

/*
 * gxpci_free_rx_resources - Free Rx resources per queue.
 * @nic: board private structure.
 *
 * Free all receive software resources.
 */
static void gxpci_free_rx_resources(struct gxpci_nic *nic)
{
	int rx;

	for (rx = 0; rx < nic->num_rx_queues; rx++) {
		struct gxpci_nic_ring *rx_ring = &nic->rx_ring[rx];

		gxpci_clean_rx_ring(rx_ring);

		vfree(rx_ring->buffer_infos);
		rx_ring->buffer_infos = NULL;
		rx_ring->desc = NULL;
	}
}

static void gxpci_up(struct gxpci_nic *nic, int reset)
{
	struct net_device *netdev = nic->netdev;
	int i;

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_nic_ring *rx_ring = &nic->rx_ring[i];
		gxpci_alloc_rx_buffers(nic, rx_ring,
				       gxpci_desc_unused(rx_ring));
	}

	/* Leave carrier/operstate OFF/DOWN if it is called by channel reset. */
	if (!reset) {
		clear_bit(__GXPCI_DOWN, &nic->state);
		netif_carrier_on(nic->netdev);
	}

	netif_tx_start_all_queues(netdev);
}

static void gxpci_down(struct gxpci_nic *nic)
{
	struct net_device *netdev = nic->netdev;
	int i;

	set_bit(__GXPCI_DOWN, &nic->state);

	netif_tx_stop_all_queues(netdev);

	netif_carrier_off(netdev);

#if GXPCI_NIC_QUEUE_DEBUG
	del_timer(&dump_timer[nic->if_num]);
#endif

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_nic_ring *rx_ring = &nic->rx_ring[i];
		gxpci_clean_rx_ring(rx_ring);
	}

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_nic_ring *tx_ring = &nic->tx_ring[i];
		gxpci_clean_tx_ring(tx_ring);
	}
}

static void gxpci_reset(struct gxpci_nic *nic)
{
	struct net_device *netdev = nic->netdev;
	struct gxpci_nic_ring *rx_ring;
	struct gxpci_nic_ring *tx_ring;
	unsigned long rx_flag[GXPCI_HOST_NIC_SIMPLEX_QUEUES];
	unsigned long tx_flag[GXPCI_HOST_NIC_SIMPLEX_QUEUES];
	int i;

	TRACE("%s: Reset channel\n", nic->netdev->name);

	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		spin_lock_irqsave(&rx_ring->cmd_queue_lock, rx_flag[i]);
	}

	for (i = 0; i < nic->num_tx_queues; i++) {
		tx_ring = &nic->tx_ring[i];
		spin_lock_irqsave(&tx_ring->cmd_queue_lock, tx_flag[i]);
	}

	/* First, do partial close work. */
	set_bit(__GXPCI_DOWN, &nic->state);

	netif_tx_stop_all_queues(netdev);

	netif_carrier_off(netdev);

	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		rx_ring->commands_posted = 0;
		rx_ring->commands_consumed = 0;
	}

	for (i = 0; i < nic->num_tx_queues; i++) {
		tx_ring = &nic->tx_ring[i];
		tx_ring->commands_posted = 0;
		tx_ring->commands_consumed = 0;
	}

	if (nic->vf_state)
		gxpci_nic_stream_close_vf(nic, 1);
	else
		gxpci_nic_stream_close(nic, 1);

	for (i = nic->num_tx_queues - 1; i >= 0; i--) {
		tx_ring = &nic->tx_ring[i];
		spin_unlock_irqrestore(&tx_ring->cmd_queue_lock, tx_flag[i]);
	}

	for (i = nic->num_rx_queues - 1; i >= 0; i--) {
		rx_ring = &nic->rx_ring[i];
		spin_unlock_irqrestore(&rx_ring->cmd_queue_lock, rx_flag[i]);
	}

	/* Second, do partial open work. */
	if (nic->vf_state)
		gxpci_nic_stream_open_vf(nic, 1);
	else
		gxpci_nic_stream_open(nic, 1);

	gxpci_up(nic, 1);
}

static int gxpci_open_vf(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	int err;

	/* Disallow open during test. */
	if (test_bit(__GXPCI_TESTING, &nic->state))
		return -EBUSY;

	netif_carrier_off(netdev);

	/* Allocate transmit descriptors. */
	err = gxpci_setup_tx_resources(nic);
	if (err)
		goto err_setup_tx;

	/* Allocate receive descriptors. */
	err = gxpci_setup_rx_resources(nic);
	if (err)
		goto err_setup_rx;

	err = gxpci_nic_stream_open_vf(nic, 0);
	if (err)
		goto err_nic_stream_open;

	gxpci_up(nic, 0);

	/* Notify RC of Tile readiness, passively. */
	nic->nic_regs_vf->queue_pair_status.tile_queue_status =
		GXPCI_TILE_CHAN_READY;

	return 0;

err_nic_stream_open:
	gxpci_free_rx_resources(nic);
err_setup_rx:
	gxpci_free_tx_resources(nic);
err_setup_tx:
	return err;
}

/*
 * gxpci_open - Called when a network interface is made active.
 * @netdev: network interface device structure.
 *
 * Return 0 on success, negative value on failure.
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog timer is started,
 * and the stack is notified that the interface is ready.
 */
static int gxpci_open(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	struct gxpci_queue_pair_status *queue_pair_status =
		&nic->nic_regs->queue_pair_status;
	int err;

	if (nic->vf_state)
		return gxpci_open_vf(netdev);

	/* Disallow open during test. */
	if (test_bit(__GXPCI_TESTING, &nic->state))
		return -EBUSY;

	netif_carrier_off(netdev);

	/* Allocate transmit descriptors. */
	err = gxpci_setup_tx_resources(nic);
	if (err)
		goto err_setup_tx;

	/* Allocate receive descriptors. */
	err = gxpci_setup_rx_resources(nic);
	if (err)
		goto err_setup_rx;

	err = gxpci_nic_stream_open(nic, 0);
	if (err)
		goto err_nic_stream_open;

	gxpci_up(nic, 0);

	/* Notify RC of Tile readiness. */
	queue_pair_status->tile_queue_status = GXPCI_TILE_CHAN_READY;

#if GXPCI_NIC_QUEUE_DEBUG
	init_timer(&dump_timer[nic->if_num]);
	dump_timer[nic->if_num].data = (unsigned long)nic;
	dump_timer[nic->if_num].function = dump;
	dump_timer[nic->if_num].expires = jiffies + 10 * HZ;
	add_timer(&dump_timer[nic->if_num]);
#endif

	return 0;

err_nic_stream_open:
	gxpci_free_rx_resources(nic);
err_setup_rx:
	gxpci_free_tx_resources(nic);
err_setup_tx:
	return err;
}

static int gxpci_close_vf(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	struct gxpci_queue_pair_status *queue_pair_status;
	struct gxpci_nic_ring *rx_ring;
	struct gxpci_nic_ring *tx_ring;
	unsigned long rx_flag[GXPCI_HOST_NIC_SIMPLEX_QUEUES_VF];
	unsigned long tx_flag[GXPCI_HOST_NIC_SIMPLEX_QUEUES_VF];
	int sleep_msecs = 0, i;

	WARN_ON(test_bit(__GXPCI_RESETTING, &nic->state));

	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		spin_lock_irqsave(&rx_ring->cmd_queue_lock, rx_flag[i]);
	}

	for (i = 0; i < nic->num_tx_queues; i++) {
		tx_ring = &nic->tx_ring[i];
		spin_lock_irqsave(&tx_ring->cmd_queue_lock, tx_flag[i]);
	}

	/* Mark the NIC port as DOWN. */
	nic->channel_state = __GXPCI_NIC_DOWN;

	queue_pair_status = &nic->nic_regs_vf->queue_pair_status;

	/* Notify RC of Tile reset, passively. */
	queue_pair_status->tile_queue_status = GXPCI_CHAN_RESET;
	wmb();

	/* Wait until the host side's queue monitor acks or timeout happens. */
	while (queue_pair_status->tile_queue_status != GXPCI_CHAN_RESET_ACK) {

		/* Exiting due to timeout. */
		if (sleep_msecs >= GXPCI_RELEASE_WAIT_MAX_MSECS)
			break;

		/* Exiting due to a signal. */
		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL))
			break;

		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}

	if (!test_bit(__GXPCI_DOWN, &nic->state)) {
		gxpci_down(nic);
		gxpci_nic_stream_close_vf(nic, 0);
	}

	gxpci_free_tx_resources(nic);
	gxpci_free_rx_resources(nic);

	for (i = nic->num_tx_queues - 1; i >= 0; i--) {
		tx_ring = &nic->tx_ring[i];
		spin_unlock_irqrestore(&tx_ring->cmd_queue_lock, tx_flag[i]);
	}

	for (i = nic->num_rx_queues - 1; i >= 0; i--) {
		rx_ring = &nic->rx_ring[i];
		spin_unlock_irqrestore(&rx_ring->cmd_queue_lock, rx_flag[i]);
	}

	return 0;
}

/*
 * gxpci_close - Disable a network interface.
 * @netdev: network interface device structure.
 *
 * Return 0, this is not allowed to fail.
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 */
static int gxpci_close(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	struct gxpci_queue_pair_status *queue_pair_status =
		&nic->nic_regs->queue_pair_status;
	struct gxpci_nic_ring *rx_ring;
	struct gxpci_nic_ring *tx_ring;
	unsigned long rx_flag[GXPCI_HOST_NIC_SIMPLEX_QUEUES];
	unsigned long tx_flag[GXPCI_HOST_NIC_SIMPLEX_QUEUES];
	int sleep_msecs = 0, i;

	if (nic->vf_state)
		return gxpci_close_vf(netdev);

	WARN_ON(test_bit(__GXPCI_RESETTING, &nic->state));

	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];
		spin_lock_irqsave(&rx_ring->cmd_queue_lock, rx_flag[i]);
	}

	for (i = 0; i < nic->num_tx_queues; i++) {
		tx_ring = &nic->tx_ring[i];
		spin_lock_irqsave(&tx_ring->cmd_queue_lock, tx_flag[i]);
	}

	/* Mark the NIC port as DOWN. */
	nic->channel_state = __GXPCI_NIC_DOWN;

	/* Notify RC of Tile reset. */
	queue_pair_status->tile_queue_status = GXPCI_CHAN_RESET;
	wmb();

	/* Wait until the host side's queue monitor acks or timeout happens. */
	while (queue_pair_status->tile_queue_status != GXPCI_CHAN_RESET_ACK) {

		/* Exiting due to timeout. */
		if (sleep_msecs >= GXPCI_RELEASE_WAIT_MAX_MSECS)
			break;

		/* Exiting due to a signal. */
		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL))
			break;

		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}

	if (!test_bit(__GXPCI_DOWN, &nic->state)) {
		gxpci_down(nic);
		gxpci_nic_stream_close(nic, 0);
	}

	gxpci_free_tx_resources(nic);
	gxpci_free_rx_resources(nic);

	for (i = nic->num_tx_queues - 1; i >= 0; i--) {
		tx_ring = &nic->tx_ring[i];
		spin_unlock_irqrestore(&tx_ring->cmd_queue_lock, tx_flag[i]);
	}

	for (i = nic->num_rx_queues - 1; i >= 0; i--) {
		rx_ring = &nic->rx_ring[i];
		spin_unlock_irqrestore(&rx_ring->cmd_queue_lock, rx_flag[i]);
	}

	return 0;
}

static u16 gxpci_select_queue(struct net_device *netdev, struct sk_buff *skb)
{
#define TX_Q_HASH
#ifndef TX_Q_HASH
	u16 txq = smp_processor_id();
	while (unlikely(txq >= netdev->real_num_tx_queues))
		txq -= netdev->real_num_tx_queues;
	return txq;
#else
	return skb_tx_hash(netdev, skb);
#endif
}

static int gxpci_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	struct gxpci_buffer *buffer_info;
	struct gxpci_nic_ring *tx_ring;
	tile_nic_tile_desc_t *t2h_header;
	tile_nic_dma_cmd_t *dma;
	unsigned int len = skb->len;
	unsigned int index;
	unsigned long flags;

	if (test_bit(__GXPCI_DOWN, &nic->state) ||
	    nic->channel_state == __GXPCI_NIC_DOWN) {
		dev_kfree_skb_any(skb);
		nic->stats.tx_carrier_errors++;
		nic->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	if (unlikely(len == 0)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (unlikely(skb_is_nonlinear(skb))) {
		if (net_ratelimit())
			pr_warning("Can not handle packet with fragments.\n");
		skb_linearize(skb);
	}

	tx_ring = &nic->tx_ring[skb->queue_mapping];

	if (unlikely(gxpci_desc_unused(tx_ring) == 0)) {

		netif_stop_subqueue(netdev, tx_ring->index);

		smp_mb();

		/*
 		 * We need to check again in case another CPU has just
 		 * made room available.
 		 */
		if (likely(gxpci_desc_unused(tx_ring) == 0))
			return NETDEV_TX_BUSY;

		/* A reprieve! */
		netif_start_subqueue(netdev, tx_ring->index);
	}

	index = tx_ring->commands_posted & tx_ring->count_mask;
	buffer_info = &tx_ring->buffer_infos[index];
	buffer_info->length = len;
	buffer_info->skb = skb;
	buffer_info->dma = (dma_addr_t)skb->data;

#ifdef GXPCI_DUMP_PACKETS
	dump_packet(skb->data, len, "tx");
#endif

	dma = &tx_ring->desc[index];

	/* Fill our customized header right before the skb buffer. */
	t2h_header = (tile_nic_tile_desc_t *)
		(buffer_info->dma - sizeof(*t2h_header));
	t2h_header->size = len;

	/* Get CPA of current skb. */
	dma->tile_addr = __pa(t2h_header);
	dma->tile_desc.size = buffer_info->length + sizeof(*t2h_header);
	dma->tile_desc.filled = 1;

	/* Make sure the commands are visible, then update the posted count. */
	wmb();
	tx_ring->commands_posted++;

	/* Lock down. */
	spin_lock_irqsave(&tx_ring->cmd_queue_lock, flags);

	if (nic->channel_state == __GXPCI_NIC_UP)
		gxpci_nic_process_t2h_cmds(tx_ring);

	/* Unlock. */
	spin_unlock_irqrestore(&tx_ring->cmd_queue_lock, flags);

	netdev->trans_start = jiffies;

	return NETDEV_TX_OK;
}

static struct net_device_stats *gxpci_get_stats(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	return (&nic->stats);
}

static int gxpci_set_mac_address(struct net_device *netdev, void *p)
{
	const struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, ETH_ALEN);

	return 0;
}

static const struct net_device_ops gxpci_netdev_ops = {
	.ndo_open		= gxpci_open,
	.ndo_stop		= gxpci_close,
	.ndo_select_queue       = gxpci_select_queue,
	.ndo_start_xmit		= gxpci_xmit_frame,
	.ndo_get_stats		= gxpci_get_stats,
	.ndo_set_mac_address    = gxpci_set_mac_address,
};

static void gxpci_setup(struct net_device *netdev)
{
	ether_setup(netdev);
	netdev->netdev_ops	= &gxpci_netdev_ops;
	netdev->mtu		= GXPCI_NIC_MTU_DEFAULT;
}

/*
 * PCIe NIC stream MMI interrupt handler.
 */
static irqreturn_t gxpci_nic_intr(int irq, void *nic_dev)
{
	struct gxpci_nic *nic = (struct gxpci_nic *) nic_dev;
	struct gxpci_queue_pair_status *queue_pair_status;
	struct gxpci_nic_ring *rx_ring;
	struct gxpci_nic_ring *tx_ring;
	unsigned long rx_flags;
	unsigned long tx_flags;
	int i;

	/* Process host commands of Rx rings. */
	for (i = 0; i < nic->num_rx_queues; i++) {
		rx_ring = &nic->rx_ring[i];

		spin_lock_irqsave(&rx_ring->cmd_queue_lock, rx_flags);
		if (nic->channel_state == __GXPCI_NIC_UP)
			gxpci_nic_process_h2t_cmds(rx_ring);
		spin_unlock_irqrestore(&rx_ring->cmd_queue_lock, rx_flags);
	}

	/* Process host commands of Tx rings. */
	for (i = 0; i < nic->num_tx_queues; i++) {
		tx_ring = &nic->tx_ring[i];

		spin_lock_irqsave(&tx_ring->cmd_queue_lock, tx_flags);
		if (nic->channel_state == __GXPCI_NIC_UP)
			gxpci_nic_process_t2h_cmds(tx_ring);
		spin_unlock_irqrestore(&tx_ring->cmd_queue_lock, tx_flags);
	}

	/*
	 * Handle link up and down, by checking the local copy of
	 * the host queue pair status.
	 */
	if (nic->vf_state)
		queue_pair_status = &nic->nic_regs_vf->queue_pair_status;
	else
		queue_pair_status = &nic->nic_regs->queue_pair_status;

	if (queue_pair_status->host_queue_status == GXPCI_HOST_CHAN_READY &&
	    queue_pair_status->tile_queue_status == GXPCI_TILE_CHAN_READY) {
		if (nic->channel_state == __GXPCI_NIC_DOWN) {

			INFO("%s: Link is up\n", nic->netdev->name);

			/* Mark Tile NIC port as UP. */
			nic->channel_state = __GXPCI_NIC_UP;

			/* Clear carrier/operstate OFF/DOWN states. */
			clear_bit(__GXPCI_DOWN, &nic->state);
			netif_carrier_on(nic->netdev);
		}
	} else if (queue_pair_status->host_queue_status == GXPCI_CHAN_RESET) {
		if (nic->channel_state == __GXPCI_NIC_UP) {

			INFO("%s: Link is down\n", nic->netdev->name);

			/* Mark Tile NIC port as DOWN. */
			nic->channel_state = __GXPCI_NIC_DOWN;

 			/* Handle NIC port reset work. */
			schedule_delayed_work(&nic->ep_queue_monitor_work, 0);
		}
	}

	return IRQ_HANDLED;
}

static void ep_queue_monitor(struct work_struct *work)
{
	struct gxpci_nic *nic = container_of(work, struct gxpci_nic,
					     ep_queue_monitor_work.work);
	struct gxpci_queue_pair_status *queue_pair_status =
		&nic->nic_regs->queue_pair_status;

	/* Reset Tile NIC port. */
	gxpci_reset(nic);

	/* Notify RC of Tile reset ack. */
	queue_pair_status->host_queue_status = GXPCI_CHAN_RESET_ACK;
}

static void ep_queue_monitor_vf(struct work_struct *work)
{
	struct gxpci_nic *nic = container_of(work, struct gxpci_nic,
					     ep_queue_monitor_work.work);
	struct gxpci_queue_pair_status *queue_pair_status =
		&nic->nic_regs_vf->queue_pair_status;

	/* Reset Tile NIC port. */
	gxpci_reset(nic);

	/* Notify RC of Tile reset ack. */
	queue_pair_status->host_queue_status = GXPCI_CHAN_RESET_ACK;
}

/*
 * gxpci_nic_mem_free - Free backing memory for all virtual network
 * 			interfaces.
 */
static void gxpci_nic_mem_free(struct tlr_pcie_dev *ep_dev)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;

	/* Free TRIO resources. */
	gxio_trio_free_memory_map(trio_context, ep_dev->nic_bar2_mem_map);
	gxio_trio_unregister_page(trio_context, trio_context->asid,
				  ep_dev->nic_bar2_mem);
	gxio_trio_free_memory_map(trio_context, ep_dev->nic_bar0_mem_map);
	gxio_trio_unregister_page(trio_context, trio_context->asid,
				  ep_dev->nic_bar0_mem);
	/* Free pages. */
	homecache_free_pages((unsigned long)ep_dev->nic_bar2_mem,
			     get_order(HPAGE_SIZE));
	ep_dev->nic_bar2_mem = NULL;
	homecache_free_pages((unsigned long)ep_dev->nic_bar0_mem,
			     get_order(HPAGE_SIZE));
	ep_dev->nic_bar0_mem = NULL;
}

/*
 * gxpci_nic_mem_init - Initialize backing memory for all virtual network
 * 			interfaces.
 * 
 * Return 0 on success, negative on failure.
 */
static int gxpci_nic_mem_init(struct tlr_pcie_dev *ep_dev)
{
	gxio_trio_context_t *trio_context = ep_dev->trio;
	struct gxpci_host_regs *regs =
		(struct gxpci_host_regs *)ep_dev->bar_mem;
	struct page *hpage;
	int err;

	/*
	 * Allocate a huge page to back up host interface part of the BAR0, i.e.
	 * 64-bit mem-map interrupt registers at the top of this page and
	 * struct gxpci_host_nic_regs, for all virtual network interfaces.
	 */
	hpage = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(HPAGE_SIZE));
	if (!hpage) {
		err = -ENOMEM;
		ERR("%s: TRIO %d MAC %d alloc_pages() failed %d.\n",
		    driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto bar0_mem_alloc_failed;
	}
	ep_dev->nic_bar0_mem = page_address(hpage);

	/* Register the huge page. */
	err = gxio_trio_register_page(trio_context,
				      trio_context->asid,
				      ep_dev->nic_bar0_mem,
				      HPAGE_SIZE,
				      0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_register_page() failed %d.\n",
		    driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto bar0_mem_reg_failed;
	}

	/* Allocate and initialize one memory map region. */
	err = gxio_trio_alloc_memory_maps(trio_context, 1, 0, 0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_alloc_memory_maps() "
		    "failed %d.\n", driver_name, ep_dev->trio_index,
		    ep_dev->mac, err);
		goto bar0_mem_map_alloc_failed;
	}
	ep_dev->nic_bar0_mem_map = err;

	err = gxio_trio_init_memory_map(trio_context, ep_dev->nic_bar0_mem_map,
					ep_dev->nic_bar0_mem +
					GXPCI_HOST_NIC_REGS_MAP_SIZE *
					GXPCI_HOST_NIC_COUNT,
					GXPCI_HOST_NIC_REGS_SIZE *
					GXPCI_HOST_NIC_COUNT,
					trio_context->asid,
					ep_dev->mac,
					ep_dev->bar0_addr +
					GXPCI_HOST_NIC_REGS_BAR_OFFSET,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_init_memory_map() "
		    "failed %d.\n", driver_name, ep_dev->trio_index,
		    ep_dev->mac, err);
		goto bar0_mem_map_init_failed;
	}

	/*
	 * Allocate a huge page to back up host interface part of the BAR2, i.e.
	 * struct gxpci_host_nic_desc, for all virtual network interfaces.
	 */
	hpage = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(HPAGE_SIZE));
	if (!hpage) {
		err = -ENOMEM;
		ERR("%s: TRIO %d MAC %d alloc_pages() failed %d.\n",
		    driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto bar2_mem_alloc_failed;
	}
	ep_dev->nic_bar2_mem = page_address(hpage);

	/* Register the huge page. */
	err = gxio_trio_register_page(trio_context,
				      trio_context->asid,
				      ep_dev->nic_bar2_mem,
				      HPAGE_SIZE,
				      0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_register_page() failed %d.\n",
		    driver_name, ep_dev->trio_index, ep_dev->mac, err);
		goto bar2_mem_reg_failed;
	}

	/* Allocate and initialize one memory map region. */
	err = gxio_trio_alloc_memory_maps(trio_context, 1, 0, 0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_alloc_memory_maps() "
		    "failed %d.\n", driver_name, ep_dev->trio_index,
		    ep_dev->mac, err);
		goto bar2_mem_map_alloc_failed;
	}
	ep_dev->nic_bar2_mem_map = err;

	err = gxio_trio_init_memory_map(trio_context, ep_dev->nic_bar2_mem_map,
					ep_dev->nic_bar2_mem,
					GXPCI_HOST_NIC_DMA_RING_SIZE *
					GXPCI_HOST_NIC_COUNT,
					trio_context->asid,
					ep_dev->mac,
					regs->bar2_addr +
					GXPCI_HOST_NIC_DMA_RING_BAR_OFFSET,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_init_memory_map() "
		    "failed %d.\n", driver_name, ep_dev->trio_index,
		    ep_dev->mac, err);
		goto bar2_mem_map_init_failed;
	}

	return 0;

bar2_mem_map_init_failed:
	gxio_trio_free_memory_map(trio_context, ep_dev->nic_bar2_mem_map);
bar2_mem_map_alloc_failed:
	gxio_trio_unregister_page(trio_context, trio_context->asid,
				  ep_dev->nic_bar2_mem);
bar2_mem_reg_failed:
	homecache_free_pages((unsigned long)ep_dev->nic_bar2_mem,
			     get_order(HPAGE_SIZE));
	ep_dev->nic_bar2_mem = NULL;
bar2_mem_alloc_failed:
bar0_mem_map_init_failed:
	gxio_trio_free_memory_map(trio_context, ep_dev->nic_bar0_mem_map);
bar0_mem_map_alloc_failed:
	gxio_trio_unregister_page(trio_context, trio_context->asid,
				  ep_dev->nic_bar0_mem);
bar0_mem_reg_failed:
	homecache_free_pages((unsigned long)ep_dev->nic_bar0_mem,
			     get_order(HPAGE_SIZE));
	ep_dev->nic_bar0_mem = NULL;
bar0_mem_alloc_failed:
	return err;
}

static int gxpci_alloc_tx_rings(struct gxpci_nic *nic)
{
	int i;

	nic->tx_ring = kcalloc(nic->num_tx_queues,
			       sizeof(struct gxpci_nic_ring),
	 		       GFP_KERNEL);
	if (!nic->tx_ring)
		return -ENOMEM;

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_nic_ring *ring = &nic->tx_ring[i];

		/* Set the default ring size. */
		ring->count = GXPCI_DEFAULT_TXD;
		ring->count_mask = ring->count - 1;

		if (nic->vf_state)
			ring->regs = &nic->nic_regs_vf->t2h_regs[i];
		else
			ring->regs = &nic->nic_regs->t2h_regs[i];
		ring->dev = nic->tlr;
		ring->nic_dev = nic;
		ring->dma_resource.push_dma_data_irq = -1;

		spin_lock_init(&ring->cmd_queue_lock);
	}

	return 0;
}

static void gxpci_free_tx_rings(struct gxpci_nic *nic)
{
	int i;

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_nic_ring *ring = &nic->tx_ring[i];
		ring->nic_dev = NULL;
		ring->regs = NULL;
		ring->dev = NULL;
	}

	kfree(nic->tx_ring);
	nic->tx_ring = NULL;
}

static int gxpci_alloc_rx_rings(struct gxpci_nic *nic)
{
	int i;

	nic->rx_ring = kcalloc(nic->num_rx_queues,
			       sizeof(struct gxpci_nic_ring),
	 		       GFP_KERNEL);
	if (!nic->rx_ring)
		return -ENOMEM;

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_nic_ring *ring = &nic->rx_ring[i];

		/* Set the default ring size. */
		ring->count = GXPCI_DEFAULT_RXD;
		ring->count_mask = ring->count - 1;

		if (nic->vf_state)
			ring->regs = &nic->nic_regs_vf->h2t_regs[i];
		else
			ring->regs = &nic->nic_regs->h2t_regs[i];
		ring->dev = nic->tlr;
		ring->nic_dev = nic;
		ring->dma_resource.pull_dma_data_irq = -1;

		spin_lock_init(&ring->cmd_queue_lock);
	}

	return 0;
}

static void gxpci_free_rx_rings(struct gxpci_nic *nic)
{
	int i;

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_nic_ring *ring = &nic->rx_ring[i];
		ring->nic_dev = NULL;
		ring->regs = NULL;
		ring->dev = NULL;
	}

	kfree(nic->rx_ring);
	nic->rx_ring = NULL;
}

/*
 * gxpci_nic_free - Free a virtual network interface.
 */
static void gxpci_nic_free(struct tlr_pcie_dev *tlr, int if_num)
{
	gxio_trio_context_t *trio_context = tlr->trio;
	struct gxpci_nic *nic = tlr->net_devs[if_num];

	/* Free TRIO resource. */
	gxio_trio_free_memory_map(trio_context, nic->mmi_mem_map);

	/* Free interrupt. */
	free_irq(nic->mmi_irq, nic);
	destroy_irq(nic->mmi_irq);

	/* Free network resource. */
	unregister_netdev(nic->netdev);
	gxpci_free_rx_rings(nic);
	gxpci_free_tx_rings(nic);
	free_netdev(nic->netdev);
}

/*
 * gxpci_nic_init - Register a virtual network interface.
 *
 * Return 0 on success, negative on failure.
 */
static int gxpci_nic_init(struct tlr_pcie_dev *tlr, int if_num)
{
       	gxio_trio_context_t *trio_context = tlr->trio;
	struct gxpci_host_regs *regs = (struct gxpci_host_regs *)tlr->bar_mem;
        struct gxpci_queue_pair_status *queue_pair_status;
	struct net_device *netdev;
	struct gxpci_nic *nic;
	char ifname[IFNAMSIZ];
	char *mac;
	int irq;
	int cpu;	
	int err = -ENOMEM;

	sprintf(ifname, "gxpci%d-%d", regs->link_index, if_num);

	/*
 	 * With parameter checking in gxpci endpoint driver, we will have a same
 	 * number of Tx and Rx queues per virtual network interface.
 	 */
	netdev = alloc_netdev_mq(sizeof(struct gxpci_nic), ifname, gxpci_setup,
				 regs->nic_t2h_queues);
	if (!netdev)
		goto err_alloc_netdev;

	nic = netdev_priv(netdev);

	nic->tlr = tlr;
	nic->netdev = netdev;
	nic->if_num = if_num;
	nic->backing_mem = NULL;
	nic->channel_state = __GXPCI_NIC_DOWN;
	nic->num_tx_queues = regs->nic_t2h_queues; 
	nic->num_rx_queues = regs->nic_h2t_queues;
	nic->int_vectors = MAX(nic->num_tx_queues, nic->num_rx_queues);

	/* Set the default receive buffer size. */
	nic->rx_buffer_len = GXPCI_NIC_RX_BUF_LEN;

	/* Set the mapped NIC registers in BAR0 space. */
	nic->nic_regs = (struct gxpci_host_nic_regs *)(tlr->nic_bar0_mem +
			GXPCI_HOST_NIC_COUNT * GXPCI_HOST_NIC_REGS_MAP_SIZE +
			if_num * GXPCI_HOST_NIC_REGS_SIZE);
	memset(nic->nic_regs, 0, sizeof(struct gxpci_host_nic_regs));

	/* Set the mapped NIC DMA descriptor arrays in BAR2 space. */
	nic->nic_desc = (struct gxpci_host_nic_desc *)(tlr->nic_bar2_mem +
			GXPCI_HOST_NIC_DMA_RING_BAR_OFFSET +
			if_num * GXPCI_HOST_NIC_DMA_RING_SIZE);
	memset(nic->nic_desc, 0, sizeof(struct gxpci_host_nic_desc));

	err = gxpci_alloc_tx_rings(nic);
	if (err)
		goto err_tx_rings;

	err = gxpci_alloc_rx_rings(nic);
	if (err)
		goto err_rx_rings;

	set_bit(__GXPCI_DOWN, &nic->state);

	/* Tell the stack to leave us alone until gxpci_open() is called. */
	netif_carrier_off(netdev);
	netif_tx_stop_all_queues(netdev);

	/* 
 	 * Set a random MAC address with its most significant byte set to the
 	 * link index.
	 */
	random_ether_addr(netdev->dev_addr);
	mac = netdev->dev_addr;
	mac[ETH_ALEN - 1] = (uint8_t)regs->link_index;

	/* Reserve more headroom to hold our own custom header. */
	netdev->needed_headroom = sizeof(tile_nic_tile_desc_t);

	err = register_netdev(netdev);
	if (err)
		goto err_register;

	tlr->net_devs[if_num] = nic;

	/*
 	 * Allocate and initialize one memory map region for the MMI interrupt.
 	 */
	err = gxio_trio_alloc_memory_maps(trio_context, 1, 0, 0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_alloc_memory_maps() "
		    "failed %d.\n", driver_name, tlr->trio_index, tlr->mac,
		    err);
		goto mem_map_alloc_failed;
	}
	nic->mmi_mem_map = err;

	err = gxio_trio_init_memory_map(trio_context, nic->mmi_mem_map,
					tlr->nic_bar0_mem +
					if_num * GXPCI_HOST_NIC_REGS_MAP_SIZE,
					GXPCI_HOST_NIC_REGS_MAP_SIZE,
					trio_context->asid,
					tlr->mac,
					tlr->bar0_addr +
					GXPCI_HOST_NIC_STREAMS_MMI_REGS_OFFSET +
					if_num * GXPCI_HOST_NIC_REGS_MAP_SIZE,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_init_memory_map() "
		    "failed %d.\n", driver_name, tlr->trio_index, tlr->mac,
		    err);
		goto mem_map_init_failed;
	}

	/* Allocate and register a common MMI interrupt. */
	irq = create_irq();
	if (irq < 0) {
		ERR("%s: no free irq vectors.\n", driver_name);
		err = -EINVAL;
		goto create_irq_failed;
	}
	nic->mmi_irq = irq;	

	/* Enable the MMI interrupt in kernel. */
	tile_irq_activate(irq, TILE_IRQ_PERCPU);
	sprintf(nic->mmi_irq_name, "%s-mmi", ifname);
	if (request_irq(irq, gxpci_nic_intr, 0, nic->mmi_irq_name,
			(void *)nic)) {
		ERR("%s: Failed to register handler for IRQ %d.\n",
		    driver_name, irq);
		err = -EINVAL;
		goto req_irq_kernel_failed;
	}

	/*
 	 * Try to choose a non-dataplane processor to receive the MMI interrupt.
 	 */
	cpu = tile_irq_cpu(irq);

	/* Enable the MMI interrupt in TRIO. */
	err = gxio_trio_config_char_intr(trio_context,
					 cpu_x(cpu), cpu_y(cpu),
					 KERNEL_PL, irq,
					 tlr->mac,
					 nic->mmi_mem_map, 0, 0,
					 MEM_MAP_SEL);
	if (err < 0) {
		ERR("%s: Memory map region %d interrupt config failed.\n",
		    driver_name, nic->mmi_mem_map);
		goto req_irq_hv_failed;
	}

	/* Reset queue status. */
	queue_pair_status = &nic->nic_regs->queue_pair_status;
	queue_pair_status->tile_queue_status = GXPCI_CHAN_UNINITIALIZED;
	queue_pair_status->host_queue_status = GXPCI_CHAN_UNINITIALIZED;

	INIT_DELAYED_WORK(&nic->ep_queue_monitor_work, ep_queue_monitor);

	INFO("Registering network device %s\n", ifname);

	return 0;

req_irq_hv_failed:
req_irq_kernel_failed:
	destroy_irq(irq);
create_irq_failed:
mem_map_init_failed:
	gxio_trio_free_memory_map(trio_context, nic->mmi_mem_map);
mem_map_alloc_failed:
	unregister_netdev(netdev);
err_register:
	gxpci_free_rx_rings(nic);
err_rx_rings:
	gxpci_free_tx_rings(nic);
err_tx_rings:
	free_netdev(netdev);
err_alloc_netdev:

	return err;
}

/*
 * gxpci_nic_init_vf - Register a virtual network interface on an VF.
 *
 * Return 0 on success, negative on failure.
 */
static int gxpci_nic_init_vf(struct tlr_pcie_dev *tlr, int vf, int if_num)
{
	gxio_trio_context_t *trio_context = tlr->trio;
	struct gxpci_ep_vf_state *vf_state;
	struct net_device *netdev;
	struct gxpci_nic *nic;
	char ifname[IFNAMSIZ];
	struct page *hpage;
	char *mac;
	int err;
	int cpu;
	int irq;

	vf_state = &tlr->vfs[vf];

	sprintf(ifname, "pci_vf%d-%d", vf, if_num);

	netdev = alloc_netdev_mq(sizeof(struct gxpci_nic), ifname, gxpci_setup,
				 GXPCI_HOST_NIC_SIMPLEX_QUEUES_VF);
	if (!netdev) {
		err = -ENOMEM;
		goto err_alloc_netdev;
	}

	nic = netdev_priv(netdev);

	vf_state->nic[if_num] = nic;

	hpage = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(HPAGE_SIZE));
	if (!hpage) {
		err = -ENOMEM;
		ERR("%s: TRIO %d MAC %d alloc_pages() failed %d.\n",
		    driver_name, tlr->trio_index, tlr->mac, err);
		goto bar0_mem_alloc_failed;
	}
	vf_state->nic_bar0_mem[if_num] = page_address(hpage);

	/* Register the backing memory huge page. */
	err = gxio_trio_register_page(trio_context,
				      trio_context->asid,
				      vf_state->nic_bar0_mem[if_num],
				      HPAGE_SIZE,
				      0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_register_page() failed %d.\n",
		    driver_name, tlr->trio_index, tlr->mac, err);
		goto bar0_mem_reg_failed;
	}

	/* Allocate and initialize one memory map region. */
	err = gxio_trio_alloc_memory_maps(trio_context, 1, 0, 0);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_alloc_memory_maps() "
		    "failed %d.\n", driver_name, tlr->trio_index,
		    tlr->mac, err);
		goto bar0_mem_map_alloc_failed;
	}
	vf_state->nic_bar0_mem_map[if_num] = err;

	err = gxio_trio_init_memory_map(trio_context,
					vf_state->nic_bar0_mem_map[if_num],
					vf_state->nic_bar0_mem[if_num] +
					GXPCI_VF_NIC_MAP_INTR_REG_OFFSET,
					GXPCI_VF_HOST_NIC_MAP_SIZE -
					GXPCI_VF_NIC_MAP_INTR_REG_OFFSET,
					trio_context->asid,
					tlr->mac,
					vf_state->vf_bar0_address +
					GXPCI_VF_NIC_MAP_INTR_REG_OFFSET +
					if_num * GXPCI_VF_HOST_NIC_MAP_SIZE,
					GXIO_TRIO_ORDER_MODE_UNORDERED);
	if (err < 0) {
		ERR("%s: TRIO %d MAC %d gxio_trio_init_memory_map() "
		    "failed %d.\n", driver_name, tlr->trio_index,
		    tlr->mac, err);
		goto bar0_mem_map_init_failed;
	}

	nic->mmi_mem_map = vf_state->nic_bar0_mem_map[if_num];

	/* Allocate and register a common MMI interrupt. */
	irq = create_irq();
	if (irq < 0) {
		ERR("%s: no free irq vectors for VF %d.\n", driver_name, vf);
		err = -EINVAL;
		goto mmi_create_irq_failed;
	}
	nic->mmi_irq = irq;	

	/* Enable the MMI interrupt in kernel. */
	tile_irq_activate(irq, TILE_IRQ_PERCPU);
	sprintf(nic->mmi_irq_name, "%s-mmi", netdev->name);
	if (request_irq(irq, gxpci_nic_intr, 0, nic->mmi_irq_name,
			(void *)nic)) {
		ERR("%s: Failed to register handler for IRQ %d for VF %d.\n",
		    driver_name, irq, vf);
		err = -EINVAL;
		goto mmi_req_irq_kernel_failed;
	}

	/*
 	 * Try to choose a non-dataplane processor to receive the MMI interrupt.
 	 */
	cpu = tile_irq_cpu(irq);

	/* Enable the MMI interrupt in TRIO. */
	err = gxio_trio_config_char_intr(trio_context,
					 cpu_x(cpu), cpu_y(cpu),
					 KERNEL_PL, irq,
					 tlr->mac,
					 nic->mmi_mem_map, 0, 0,
					 MEM_MAP_SEL);
	if (err < 0) {
		ERR("%s: region %d interrupt config failed for VF %d.\n",
		    driver_name, nic->mmi_mem_map, vf);
		goto mmi_req_irq_hv_failed;
	}

	nic->tlr = tlr;
	nic->netdev = netdev;
	nic->if_num = if_num;
	nic->vf_state = vf_state;
	nic->channel_state = __GXPCI_NIC_DOWN;
	nic->num_tx_queues = GXPCI_HOST_NIC_TX_QUEUES_VF; 
	nic->num_rx_queues = GXPCI_HOST_NIC_RX_QUEUES_VF;
	nic->int_vectors = MAX(nic->num_tx_queues, nic->num_rx_queues);

	/* This is the start of the DMA descriptor rings. */
	nic->backing_mem =
		vf_state->nic_bar0_mem[if_num] + GXPCI_VF_HOST_NIC_MAP_SIZE;

	/* Set the default receive buffer size. */
	nic->rx_buffer_len = GXPCI_NIC_RX_BUF_LEN;

	/* Set the mapped NIC registers in BAR0 space. */
	nic->nic_regs_vf = (struct gxpci_host_nic_regs_vf *)
		(vf_state->nic_bar0_mem[if_num] +
		GXPCI_VF_HOST_NIC_REG_OFFSET);
	memset(nic->nic_regs_vf, 0, sizeof(struct gxpci_host_nic_regs_vf));

	/* Set the mapped NIC DMA descriptor arrays in BAR0 space. */
	nic->nic_desc_vf = (struct gxpci_host_nic_desc_vf *)
		(vf_state->nic_bar0_mem[if_num] +
		GXPCI_VF_HOST_NIC_DESC_OFFSET);
	memset(nic->nic_desc_vf, 0, sizeof(struct gxpci_host_nic_desc_vf));

	err = gxpci_alloc_tx_rings(nic);
	if (err)
		goto err_tx_rings;

	err = gxpci_alloc_rx_rings(nic);
	if (err)
		goto err_rx_rings;

	set_bit(__GXPCI_DOWN, &nic->state);

	/* Tell the stack to leave us alone until gxpci_open() is called. */
	netif_carrier_off(netdev);
	netif_tx_stop_all_queues(netdev);

	/* 
 	 * Set a random MAC address with its most significant byte set to the
 	 * link index.
	 */
	random_ether_addr(netdev->dev_addr);
	mac = netdev->dev_addr;
	mac[ETH_ALEN - 1] = (uint8_t)vf;

	/* Reserve more headroom to hold our own custom header. */
	netdev->needed_headroom = sizeof(tile_nic_tile_desc_t);

	err = register_netdev(netdev);
	if (err)
		goto err_register;

	INIT_DELAYED_WORK(&nic->ep_queue_monitor_work, ep_queue_monitor_vf);

	INFO("Registering network device %s\n", ifname);

	return 0;

err_register:
	gxpci_free_rx_rings(nic);
err_rx_rings:
	gxpci_free_tx_rings(nic);
err_tx_rings:
mmi_req_irq_hv_failed:
mmi_req_irq_kernel_failed:
	destroy_irq(nic->mmi_irq);
mmi_create_irq_failed:
bar0_mem_map_init_failed:
	gxio_trio_free_memory_map(trio_context,
				  vf_state->nic_bar0_mem_map[if_num]);
bar0_mem_map_alloc_failed:
	gxio_trio_unregister_page(trio_context, trio_context->asid,
				  vf_state->nic_bar0_mem[if_num]);
bar0_mem_reg_failed:
	homecache_free_pages((unsigned long)vf_state->nic_bar0_mem[if_num],
			     get_order(HPAGE_SIZE));
bar0_mem_alloc_failed:
	free_netdev(netdev);
err_alloc_netdev:

	return err;
}

/*
 * gxpci_endp_net_devs_init_vf - Initialize the virtual network interfaces
 *			         supported in the Virtual Functions.
 */
void gxpci_endp_net_devs_init_vf(struct tlr_pcie_dev *ep_dev)
{
	struct gxpci_ep_vf_state *vf_state;
	int err;
	int nic;
	int vf;

	for (vf = 0; vf < ep_dev->num_vfs; vf++) {
		vf_state = &ep_dev->vfs[vf];

		vf_state->nic_ports = GXPCI_HOST_NIC_COUNT_VF;

		for (nic = 0; nic < vf_state->nic_ports; nic++) {
			err = gxpci_nic_init_vf(ep_dev, vf, nic);
			if (err) {
				ERR("%s: TRIO %d MAC %d gxpci_nic_init_vf() "
				    "failed for VF %d nic %d.\n", driver_name,
				    ep_dev->trio_index, ep_dev->mac, vf, nic);

				return;
			}
		}
#if 0
		/* Use the first huge page to back up the VF's MSI-X tables. */
		vf_state->msix_table_base =
			vf_state->nic_bar0_mem[0] + GXPCI_MSIX_TABLE_OFFSET_VF;
#endif
	}

	return;
}

/*
 * gxpci_endp_net_devs_init - Initialize the virtual network interfaces
 *			      supported on this PCIe link.
 *
 * This is called by the Gx PCIe endpoint driver's probe routine.
 */
void gxpci_endp_net_devs_init(struct tlr_pcie_dev *ep_dev)
{
	int i;

	/*
	 * Initialize BAR0 and BAR2 backing memory for all virtual network
	 * interfaces.
	 */
	int err = gxpci_nic_mem_init(ep_dev);
	if (err) {
		ERR("%s: TRIO %d MAC %d gxpci_nic_mem_init() failed %d.\n",
		    driver_name, ep_dev->trio_index, ep_dev->mac, err);
		return;
	}

	/* Initialize and register all virtual network interfaces. */
	for (i = 0; i < host_vnic_ports[ep_dev->trio_index][ep_dev->mac]; i++) {

		/* Do not beyond the max virtual network interface number. */
		if (i >= GXPCI_ENDP_NIC_COUNT)
			break;

		err = gxpci_nic_init(ep_dev, i);
		if (err) {
			ERR("%s: TRIO %d MAC %d gxpci_nic_init() failed %d.\n",
			    driver_name, ep_dev->trio_index, ep_dev->mac, err);	
			goto nic_init_failed;
		}
	}

	return;

nic_init_failed:
	while (i--)
		gxpci_nic_free(ep_dev, i);

	gxpci_nic_mem_free(ep_dev);
}
