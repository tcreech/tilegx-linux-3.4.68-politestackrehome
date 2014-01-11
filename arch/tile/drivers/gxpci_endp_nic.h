/*
 * gxpci_endp_nic.h
 *
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
#ifndef __GXPCI_ENDP_NIC_H__
#define __GXPCI_ENDP_NIC_H__

#include <asm/tilegxpci.h>

#include <linux/netdevice.h>

#include "gxpci_endp.h"
#include "tilegxpci_shared_code.h"


/* Driver name. */
#define DRV_NAME_STRING			"gxpci_endp_nic"

/* Total waiting time for the reset acknowledgement from the other end. */
#define GXPCI_RELEASE_WAIT_MAX_MSECS	1000

/* MTU size, 4000 bytes by default. */
#define GXPCI_NIC_MTU_DEFAULT		4000

#define GXPCI_NIC_QUEUE_DEBUG		0

/*
 * Receive buffer length.
 * MTU + ethernet header + IP header align offset.
 */
#define GXPCI_NIC_RX_BUF_LEN 		\
	(GXPCI_NIC_MTU_DEFAULT + ETH_HLEN + NET_IP_ALIGN)

/* Default number of transmit ring entries. */
#define GXPCI_DEFAULT_TXD		PCIE_CMD_QUEUE_ENTRIES

/* Transmit queue restart threshold. */
#define GXPCI_WAKE_THRESHOLD		(GXPCI_DEFAULT_TXD >> 2)

/* Default number of receive ring entries. */
#define GXPCI_DEFAULT_RXD		PCIE_CMD_QUEUE_ENTRIES

/* Refill threshold for receive ring entries. */
#define GXPCI_RXD_REFILL_THRESHOLD	(GXPCI_DEFAULT_RXD >> 2)

/**
 * The number of entries in the host NIC queue Push DMA descriptor rings.
 * These must be at least as large as the PCIE_CMD_QUEUE_ENTRIES.
 * The hardware supports RING_ORDs of 9 (512 entries), 11 (2048 entries),
 * 13 (8192 entries), and 16 (65536 entries).
 */
#define GXPCI_HOST_NIC_PUSH_DMA_RING_ORD 11

/** The number of entries in the host NIC queue push DMA descriptor ring. */
#define GXPCI_HOST_NIC_PUSH_DMA_RING_LEN (1 << GXPCI_HOST_NIC_PUSH_DMA_RING_ORD)

/**
 * The number of entries in the host NIC queue Pull DMA descriptor rings.
 * These must be at least as large as the PCIE_CMD_QUEUE_ENTRIES.
 * The hardware supports RING_ORDs of 9 (512 entries), 11 (2048 entries),
 * 13 (8192 entries), and 16 (65536 entries).
 */
#define GXPCI_HOST_NIC_PULL_DMA_RING_ORD 11

/** The number of entries in the host NIC queue pull DMA descriptor ring. */
#define GXPCI_HOST_NIC_PULL_DMA_RING_LEN (1 << GXPCI_HOST_NIC_PULL_DMA_RING_ORD)

/*
 * The number of network interfaces supported between the host and a TILE
 * PCIe port.
 */
#define GXPCI_ENDP_NIC_COUNT		2

#define GXPCI_NIC_PULL_RING_GEN_BIT	GXPCI_HOST_NIC_PULL_DMA_RING_ORD
#define GXPCI_NIC_PULL_RING_MASK	(GXPCI_HOST_NIC_PULL_DMA_RING_LEN - 1)
#define GXPCI_NIC_PUSH_RING_GEN_BIT	GXPCI_HOST_NIC_PUSH_DMA_RING_ORD
#define GXPCI_NIC_PUSH_RING_MASK	(GXPCI_HOST_NIC_PUSH_DMA_RING_LEN - 1)

/* Mask of entry number in the host and Tile buffer queues. */
#define PCIE_CMD_QUEUE_ENTRY_MASK	(PCIE_CMD_QUEUE_ENTRIES - 1)


struct gxpci_buffer {

	dma_addr_t dma;

	struct sk_buff *skb;

	uint32_t length;
};

/*
 * State object for each unidirectional queue of a PCIe network interface.
 */
struct gxpci_nic_ring {

	char name[IFNAMSIZ + 16];

	struct tlr_pcie_dev *dev;

	struct gxpci_nic *nic_dev;

	int index;

	/*
	 * VA of memory that backs up the control registers for a unidirectional
	 * host NIC queue, mapped in BAR0 space.
	 */
	struct gxpci_host_nic_queue_regs *regs;

	/*
	 * VA of memory that backs up the descriptor array for a unidirectional
	 * host NIC queue, mapped in BAR2 space.
	 */
	tile_nic_dma_cmd_t *desc;

	/* MSI-X info. */
	tilegxpci_get_msix_msg_t msix_msg;

	/* Length of ring in bytes. */
	unsigned int size;

	/* Number of descriptors in ring. Must be power of 2. */
	unsigned int count;

	/* Mask of the ring descriptor number. */
	unsigned int count_mask;

	/* Number of commands that are posted by the host. */
	unsigned int commands_posted;

	/* Number of commands that are consumed by the device. */
	unsigned int commands_consumed;

	/* Number of started DMAs. */
	unsigned int dmas_started;

	/* Number of completed DMAs. */
	unsigned int dmas_completed;

	struct gxpci_buffer *buffer_infos;

	/* Command queue state. */
	spinlock_t cmd_queue_lock;

	struct gxpci_dma_resource dma_resource;

	struct timer_list intr_timer;
};

typedef enum gxpci_nic_chan_state_e {

	__GXPCI_NIC_DOWN,

	__GXPCI_NIC_UP,

} gxpci_nic_chan_state_t;

/*
 * State object for each PCIe network interface.
 */
struct gxpci_nic {

	struct net_device *netdev;

	struct tlr_pcie_dev *tlr;

	struct pci_dev *pci_dev;

	struct net_device_stats stats;

	/* TRIO context. */
	gxio_trio_context_t trio;

	/* Hold the address of the DMA memory. */
	void *backing_mem;

	/* Mem-map region index to map the BAR0's MMI interrupt register. */
	unsigned int mmi_mem_map;

	/* IRQ for the MMI interrupt. */
	int mmi_irq;

	/* IRQ name for the MMI interrupt. */
	char mmi_irq_name[IFNAMSIZ + 16];

	/* Channel state. */
	gxpci_nic_chan_state_t channel_state;

	/*
	 * The network interface number, between 0 and
	 * (PCIE_ENDP_NIC_COUNT - 1).
	 */
	int if_num;

	/*
	 * VA of memory that backs up the control registers for a host NIC
	 * interface, mapped in BAR0 space.
	 */
	struct gxpci_host_nic_regs *nic_regs;

	/*
	 * VA of memory that backs up the descriptor array for a host NIC
	 * interface, mapped in BAR2 space.
	 */
	struct gxpci_host_nic_desc *nic_desc;

	/* The SR-IOV VF state pointer. NULL means PF. */
	struct gxpci_ep_vf_state *vf_state;

	/*
	 * VA of memory that backs up the control registers for a host NIC
	 * interface for an VF, mapped in BAR0 space.
	 */
	struct gxpci_host_nic_regs_vf *nic_regs_vf;

	/*
	 * VA of memory that backs up the descriptor array for a host NIC
	 * interface for an VF, mapped in BAR0 space.
	 */
	struct gxpci_host_nic_desc_vf *nic_desc_vf;

	int int_vectors;

	u32 rx_buffer_len;

	/* Track device up/down state. */
	unsigned long state;

	struct gxpci_nic_ring *tx_ring ____cacheline_aligned_in_smp;
	int num_tx_queues;

	struct gxpci_nic_ring *rx_ring ____cacheline_aligned_in_smp;
	int num_rx_queues;

	/* This is used to monitor the queue status. */
	struct delayed_work ep_queue_monitor_work;
};

enum gxpci_nic_state_t {

	__GXPCI_TESTING,

	__GXPCI_RESETTING,

	__GXPCI_DOWN,
};


#endif
