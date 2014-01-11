/*
 * gxpci_host_subset.c
 *
 * A minimal PCIe root complex driver for connecting to Tilera Gx
 * endpoints. Provides support for boot and chip-to-chip peer transfers.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/version.h>

#include <asm/tilegxpci.h>

#include <arch/sim.h>

#include "gxpci_chip_to_chip.h"
#include "gxpci_host_nic.h"
#include "gxpci_host_subset.h"

/* Timeout value of the chip-to-chip link setup timer. */
#define C2C_LINK_SETUP_TIMEOUT	(1 * HZ)

static const char driver_name[] = DRIVER_NAME_STRING;

extern int num_rc_controllers;

/*
 * Array of pointers to all the Gx endpoint devices in the system.
 * The first index represents the PCI domain number.
 */
static struct tlr_pcie_dev *
gx_ep_ports[TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES][MAX_PCIE_PORTS_PER_DOMAIN - 1];

extern
struct tlr_pcie_rc_c2c_dev *gxpci_rc_devs[TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES];

/*
 * This is the Gx endpoint link index in each PCI domain and it is set to start
 * at 1 because index 0 is reserved for the RC port.
 */
static int ep_link_index[TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES] =
	{ [0 ... (TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES)-1] = 1 };


/* Host NIC port number for a single TILE PCIe port. */
static int nic_ports = GXPCI_HOST_NIC_COUNT;
module_param(nic_ports, int, S_IRUGO);
MODULE_PARM_DESC(nic_ports,
	"Number of host virtual NIC interfaces for a single PCIe port.");

/* Per-port host NIC TX queue number. */
static int nic_tx_queues = GXPCI_HOST_SUBSET_NIC_TX_QUEUES;
module_param(nic_tx_queues, int, S_IRUGO);
MODULE_PARM_DESC(nic_tx_queues,
	"Number of TX queues per virtual NIC interface.");

/* Per-port host NIC RX queue number. */
static int nic_rx_queues = GXPCI_HOST_SUBSET_NIC_RX_QUEUES;
module_param(nic_rx_queues, int, S_IRUGO);
MODULE_PARM_DESC(nic_rx_queues,
	"Number of RX queues per virtual NIC interface.");

/*
 * Bitmap of the PCIe MACs which should have networks associated with them.
 * The bitmap index is (mac_number + 4 * trio_number). For a standard
 * TILExtreme-Gx Duo config, this would be set to 4 on the A nodes only.
 */
static int net_macs;
module_param(net_macs, int, S_IRUGO);
MODULE_PARM_DESC(net_macs, "Mask of MACs to export as network drivers; each "
		 "nybble corresponds to a TRIO shim.");

/**********************************************************************/
/*                          Boot Device Routines                      */
/**********************************************************************/
static ssize_t gx_boot_write(struct file *filp, const char __user *buf,
			     size_t count, loff_t *f_pos);

static int gx_boot_release(struct inode *inode, struct file *filp);

static const struct file_operations gx_boot_ops = {
	.owner = THIS_MODULE,
	.write = gx_boot_write,
	.release = gx_boot_release
};

/*
 * We check if the link is up and if the PCI memory space can be accessed
 * by validating the content of the RSHIM Rev ID register.
 */
static int
gx_link_is_down(struct tlr_pcie_dev *tlr)
{
#define RSH_REV_ID 0x0100

#define RSH_REV_ID__TILE_REV_ID_SHIFT 0
#define RSH_REV_ID__TILE_REV_ID_RMASK 0xff
#define RSH_REV_ID__TILE_REV_ID_VAL_TILEGX 0x20

	if (((readq(tlr->rshim_regs + RSH_REV_ID) >>
		RSH_REV_ID__TILE_REV_ID_SHIFT) &
		RSH_REV_ID__TILE_REV_ID_RMASK) ==
		RSH_REV_ID__TILE_REV_ID_VAL_TILEGX)
		return 0;
	return 1;
}

static void
gx_do_reset(struct tlr_pcie_dev *tlr)
{
#define PCIE_PREBOOTER

	/* Reset everyting. */
	writeq(0, tlr->rshim_regs + RSH_RESET_MASK);

#ifdef PCIE_PREBOOTER
	/* Make the chip reboot from SROM which contains the prebooter. */
	writeq(RSH_BOOT_CONTROL__BOOT_MODE_VAL_SPI,
			tlr->rshim_regs + RSH_BOOT_CONTROL); 
#else
	/* Make sure the chip doesn't reboot from SROM or I2C. */
	writeq(0, tlr->rshim_regs + RSH_BOOT_CONTROL); 
#endif

	/* Hit the soft reset "button." */
	writeq(RSH_RESET_CONTROL__RESET_CHIP_VAL_KEY,
			tlr->rshim_regs + RSH_RESET_CONTROL); 

	msleep(500);
}


/*
 * Reset the EP device, in preparation for downloading bootstream.
 * Returns 0 if the reset is completed.
 */
static int gx_boot_reset_device(struct tlr_pcie_dev *tlr)
{

	/*
	 * Make sure the board's link is up before we attempt to do
	 * anything.
	 */
	if (gx_link_is_down(tlr)) {
                ERR("PCIe link down; cannot boot.\n");
                return -ENODEV;
        }

	/*
	 * Save the device PCIe state before resetting it.
	 */
	pci_save_state(tlr->pci_dev);

	gx_do_reset(tlr);

	/*
	 * Restore  the PCIe state.
	 */
	pci_restore_state(tlr->pci_dev);

	return 0;
}


/*
 * Open the chip boot device and get it ready to boot.
 */
static int gx_boot_open(struct inode *inode, struct file *filp)
{
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);

	/* Set the private data to point at our state object. */
	filp->private_data = tlr;

	/* Use the stream read, write, etc. */
	filp->f_op = &gx_boot_ops;

	/* One booter per device. */
	if (mutex_lock_interruptible(&tlr->boot_mutex))
		return -ERESTARTSYS;

	if (tlr->is_booting) {
		mutex_unlock(&tlr->boot_mutex);
		return -EBUSY;
	}

	/* Reset the EP device. */
	if (gx_boot_reset_device(tlr)) {
		mutex_unlock(&tlr->boot_mutex);
		return -ENODEV;
	}

	tlr->is_booting = 1;
	return 0;
}


/*
 * Each word written to the boot device is injected into BAR 0.
 */
#define RSH_BOOT_MAX_LOOPS	200
static ssize_t
gx_boot_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	struct tlr_pcie_dev *tlr = filp->private_data;
	size_t size = count & -8;
	size_t bytes_remain;
	size_t write_bytes;
	size_t written = 0;

	uint64_t words_comp;
	uint64_t words_sent;
	uint64_t credits;
	int loops = RSH_BOOT_MAX_LOOPS;

	if (size == 0)
		return 0;

	if (!access_ok(VERIFY_READ, buf, size))
		return -EFAULT;

	bytes_remain = size;

	/*
	 * Get the initial count.  This assumes previous calls waited
	 * for the boot stream to complete.  Since other agents (BTK) can
	 * change the pg_ctl between writes to the boot file, we need
	 * to re-read PG_CTL here and assume that boot fifo is empty.
	 */
	words_comp = ((readq(tlr->rshim_regs + RSH_PG_CTL) >>
		       RSH_PG_CTL__SENT_COUNT_SHIFT) &
		      RSH_PG_CTL__SENT_COUNT_RMASK);
	words_sent = words_comp;
	credits = PCIE_BOOT_FIFO_WORDS;

	while (bytes_remain) {
		int i;

		write_bytes = min((size_t) (credits << 3), bytes_remain);

		/* Copy the bootrom chunk from userspace. */
		if (copy_from_user
		    ((char *) tlr->boot_buf, (buf + written), write_bytes))
			return -EFAULT;

		for (i = 0; i < (write_bytes >> 3); i++)
			writeq(tlr->boot_buf[i], tlr->rshim_regs + RSH_PG_DATA);

		bytes_remain -= write_bytes;
		written += write_bytes;
		words_sent = ((words_sent + (write_bytes >> 3)) &
			      RSH_PG_CTL__SENT_COUNT_RMASK);

		/* 
		 * We implement flow control to prevent boot transactions
		 * from backing up onto the PCIe link. The FC consists of
		 * polling the RSHIM's Packet Generator data-words-sent
		 * counter to see how much data has been sent.
		 * Rather than waiting for just 1 credit, we send bursts of 
		 * PCIE_BOOT_COPY_CHUNK size to improve bus utilization and
		 * amortize any overhead.
		 */
		do {
			words_comp =
			    ((readq(tlr->rshim_regs + RSH_PG_CTL) >>
				RSH_PG_CTL__SENT_COUNT_SHIFT) &
				RSH_PG_CTL__SENT_COUNT_RMASK);
			credits = (PCIE_BOOT_FIFO_WORDS -
				((words_sent - words_comp) &
				RSH_PG_CTL__SENT_COUNT_RMASK));
			if (loops-- == 0) {
				if (msleep_interruptible(5))
					return -EINTR;

				loops = RSH_BOOT_MAX_LOOPS;
			}
		} while (credits < (PCIE_BOOT_COPY_CHUNK >> 3));
	}

	/* 
	 * Wait for all remaining data to be sent so subsequent call
	 * properly tracks credits. 
	 */
	while (words_comp != words_sent) {
		words_comp = ((readq(tlr->rshim_regs + RSH_PG_CTL) >>
				RSH_PG_CTL__SENT_COUNT_SHIFT) &
				RSH_PG_CTL__SENT_COUNT_RMASK);
		if (loops-- == 0) {
			if (msleep_interruptible(5))
				return -EINTR;

			loops = RSH_BOOT_MAX_LOOPS;
		}
	}

	return written;
}


static int gx_boot_release(struct inode *inode, struct file *filp)
{
	struct tlr_pcie_dev *tlr = filp->private_data;

	tlr->is_booting = 0;

	mutex_unlock(&tlr->boot_mutex);
#if 0
	/* Schedule task to set up the EP link after the EP completes boot. */
	schedule_delayed_work(&tlr->ep_setup_work, C2C_LINK_SETUP_TIMEOUT +
				5 * HZ);
#endif
	return 0;
}


/**********************************************************************/
/*                        Chip Information File                       */
/**********************************************************************/


/* Calculate the link speed and width. */
static void calc_effective_link_width(struct pci_dev *dev)
{
	struct tlr_pcie_dev *tlr; 
	int pcie_caps_offset;
	int link_status_offset;
	u16 link_status;

	/* Find the link status regs. */
	pcie_caps_offset = pci_find_capability(dev, PCI_CAP_ID_EXP);
	if (pcie_caps_offset == 0) {
		ERR("Could not read PCI-Express capability regs.\n");
	}

	tlr = pci_get_drvdata(dev);

	/* Check whether we actually got at least the specified link BW. */
	link_status_offset = pcie_caps_offset + PCI_EXP_LNKSTA;
	pci_read_config_word(dev, link_status_offset, &link_status);
	tlr->link_speed = (link_status & PCI_EXP_LNKSTA_CLS);
	tlr->link_width = (link_status & PCI_EXP_LNKSTA_NLW) >>
		PCI_EXP_LNKSTA_NLW_SHIFT;
}


static int gx_info_seq_show(struct seq_file *s, void *token)
{
	struct tlr_pcie_dev *tlr = s->private;

	calc_effective_link_width(tlr->pci_dev);

	seq_printf(s, "CHIP_VERSION %d\n", tlr->chip_version);
	seq_printf(s, "CHIP_WIDTH %d\n", tlr->chip_width);
	seq_printf(s, "CHIP_HEIGHT %d\n", tlr->chip_height);
	seq_printf(s, "HOST_LINK_INDEX %d\n", tlr->link_index);
	seq_printf(s, "LINK_SPEED_GEN %d\n", tlr->link_speed);
	seq_printf(s, "LINK_WIDTH %d\n", tlr->link_width);

	return 0;
}


static const struct file_operations gx_info_ops = {
	.owner          = THIS_MODULE,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};


static int gx_info_open(struct tlr_pcie_dev *tlr, struct file *filp)
{
	filp->f_op = &gx_info_ops;
	return single_open(filp, gx_info_seq_show, tlr);
}


static void fill_chip_version(struct tlr_pcie_dev *tlr)
{
	switch (tlr->pci_dev->device) {
	case TILERA_GX36_DEV_ID:
		tlr->chip_version = 10;
		break;
	default:
		tlr->chip_version = -1;
	}
}


static void fill_chip_size(struct tlr_pcie_dev *tlr)
{
	u64 fabric_dim;
	int col;

	tlr->chip_width = 0;
	tlr->chip_height = 0;

	if (!tlr->rshim_regs)
		 return;

	//
	// First set the height and width to the fabric height and width.
	//
	fabric_dim = readq(tlr->rshim_regs + RSH_FABRIC_DIM);
	tlr->chip_width = (fabric_dim >> RSH_FABRIC_DIM__DIM_X_SHIFT) &
					RSH_FABRIC_DIM__DIM_X_RMASK;
	tlr->chip_height = (fabric_dim >> RSH_FABRIC_DIM__DIM_Y_SHIFT) &
					RSH_FABRIC_DIM__DIM_Y_RMASK;

	//
	// Now examine the tile disable bits to see if the actual tile grid
	// is smaller than the fabric.  We know that the usable tiles form
	// a rectangle which contains tile (0,0).
	//
	for (col = 0; col < tlr->chip_width; col++) {
		//
		// Walk through the columns.  If we hit a column that's
		// totally disabled, set our width to that column's index,
		// and stop walking, since we won't find any other tiles.
		// Otherwise, clip our height based on the number of tiles
		// in this column, and keep going.
		//
		u64 thiscol_dis;
		int tiles;

		thiscol_dis = readq(tlr->rshim_regs +
					RSH_TILE_COL_DISABLE + 8 * col);

		thiscol_dis |= ~(((u64) 1 << tlr->chip_height) - 1);
		tiles = __builtin_ctz(thiscol_dis);

		if (tiles == 0) {
			tlr->chip_width = col;
			break;
		}
		tlr->chip_height = min(tlr->chip_height, tiles);
	}
}


/**********************************************************************/
/*                   Module Loading and Device Probe                  */
/**********************************************************************/


static struct pci_device_id ids[] = {
	{ PCI_DEVICE(TILERA_VENDOR_ID, TILERA_GX36_DEV_ID), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);


static int gx_generic_open(struct inode *inode, struct file *filp)
{
	unsigned int minor = MINOR(inode->i_rdev);
	int result;
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);

	if (minor == TILEPCI_BOOT_MINOR)
		result = gx_boot_open(inode, filp);
	else if (minor == TILEPCI_INFO_MINOR)
		result = gx_info_open(tlr, filp);
	else
		result = -ENODEV;

	return result;
}


static const struct file_operations gx_generic_ops = {
	.owner = THIS_MODULE,
	.open = gx_generic_open,
};


static int gx_add_device_nodes(struct tlr_pcie_dev *tlr)
{
	struct cdev *cdev = &tlr->cdev;

	/* Allocate some major/minor numbers. */
	dev_t first;
	int num_devs = GXPCI_NUM_MINOR_DEVICES;
	int err = alloc_chrdev_region(&first, 0, num_devs,
				      (char *)driver_name);
	if (err != 0)
		return err;

	/* Register the device. */
	cdev_init(cdev, &gx_generic_ops);
	cdev->owner = THIS_MODULE;
	err = cdev_add(cdev, first, num_devs);
	if (err != 0) {
		unregister_chrdev_region(first, num_devs);
		return err;
	}
	tlr->first_dev = first;
	return 0;
}


static void gx_remove_device_nodes(struct tlr_pcie_dev *tlr)
{
	cdev_del(&tlr->cdev);
	unregister_chrdev_region(tlr->first_dev, GXPCI_NUM_MINOR_DEVICES);
}


/* This is the per PCI domain C2C queue status monitor function. */
static void c2c_queue_monitor(struct work_struct *work)
{
	struct tlr_pcie_rc_c2c_dev *rc_dev = container_of(work,
		struct tlr_pcie_rc_c2c_dev, c2c_queue_monitor_work.work);
	struct gxpci_c2c_queue_sts *queue_sts;
	struct tlr_pcie_dev *send_port = NULL;
	struct tlr_pcie_dev *recv_port = NULL;
	struct tlr_c2c_status *send_port_status = NULL;
	struct tlr_c2c_status *recv_port_status = NULL;
	uint16_t send_link_index;
	uint16_t recv_link_index;
	int domain = rc_dev->domain;
	int queue;

	for (queue = 0; queue < GXPCI_C2C_QUEUE_COUNT; queue++) {

		queue_sts = &rc_dev->c2c_queue_sts_array->c2c_queue_sts[queue];

		if (!queue_sts->active)
			continue;

		send_link_index = queue_sts->sender_link_index;
		recv_link_index = queue_sts->receiver_link_index;

		/*
		 * For the end of the C2C queue that runs on an EP node,
		 * its remote C2C queue status register is in the EP port's
		 * gxpci_host_regs structure.
		 */
		if (send_link_index) {
			send_port = gx_ep_ports[domain][send_link_index - 1];
			send_port_status =
				&send_port->regs->c2c_send_status[queue];
		}
		if (recv_link_index) {
			recv_port = gx_ep_ports[domain][recv_link_index - 1];
			recv_port_status =
				&recv_port->regs->c2c_recv_status[queue];
		}

		/*
		 * If both ends has GXPCI_CHAN_RESET, indicating that
		 * both ends are ready for reset, we set GXPCI_CHAN_RESET_ACK
		 * on both sides and invalidate the queue. Note that
		 * this is where both queues should end up under normal
		 * conditions.
		 */
		if ((queue_sts->sender_status == GXPCI_CHAN_RESET) &&
			(queue_sts->receiver_status == GXPCI_CHAN_RESET)) {

			queue_sts->sender_status = GXPCI_CHAN_RESET_ACK;
			queue_sts->receiver_status = GXPCI_CHAN_RESET_ACK;

			queue_sts->active = 0;

		} else if (queue_sts->sender_status == GXPCI_CHAN_RESET) {
			/*
			 * Check for the special case where the receiver
			 * exits due to a signal. The receiver app, via the EP
			 * driver, sets sender_status to GXPCI_CHAN_RESET
			 * before setting receiver_status to
			 * GXPCI_CHAN_UNINITIALIZED. Here we just notify the
			 * sender app of the reset event.
			 */
			if (queue_sts->receiver_status ==
				GXPCI_CHAN_UNINITIALIZED) {

				/* Use MMIO if the sender runs on EP port. */
				if (send_link_index) {
					writel(GXPCI_CHAN_RESET,
						&send_port_status->status);
				} else {
					struct tlr_c2c_status *rc_queue_sts;

					rc_queue_sts =
					rc_dev->c2c_send[queue].queue_status;
					rc_queue_sts->status = GXPCI_CHAN_RESET;
				}

				/* Deactivate the queue. */
				queue_sts->active = 0;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the sender
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			} else {
				/*
				 * Sender initiated the reset. Alert receiver
				 * by setting GXPCI_CHAN_RESET to the receiver's
				 * user-visible status.
				 */
				if (recv_link_index) {
					writel(GXPCI_CHAN_RESET,
						&recv_port_status->status);
				} else {
					struct tlr_c2c_status *rc_queue_sts;

					rc_queue_sts =
					rc_dev->c2c_recv[queue].queue_status;
					rc_queue_sts->status = GXPCI_CHAN_RESET;
				}

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the receiver
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
			 	 */
			}
		} else if (queue_sts->receiver_status == GXPCI_CHAN_RESET) {
			/*
			 * Check for the special case where the sender
			 * exits due to a signal. The sender app, via the EP
			 * driver, sets receiver_status to GXPCI_CHAN_RESET
			 * before setting sender_status to
			 * GXPCI_CHAN_UNINITIALIZED. Here we just notify the
			 * receiver app of the reset event.
			 */
			if (queue_sts->sender_status ==
				GXPCI_CHAN_UNINITIALIZED) {

				/* Use MMIO if the receiver runs on EP port. */
				if (recv_link_index) {
					writel(GXPCI_CHAN_RESET,
						&recv_port_status->status);
				} else {
					struct tlr_c2c_status *rc_queue_sts;

					rc_queue_sts =
					rc_dev->c2c_recv[queue].queue_status;
					rc_queue_sts->status = GXPCI_CHAN_RESET;
				}

				/* Deactivate the queue. */
				queue_sts->active = 0;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the receiver
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			} else {
				/*
				 * Receiver initiated the reset. Alert sender
				 * by setting GXPCI_CHAN_RESET to the sender's
				 * user-visible status.
				 */
				if (send_link_index) {
					writel(GXPCI_CHAN_RESET,
						&send_port_status->status);
				} else {
					struct tlr_c2c_status *rc_queue_sts;

					rc_queue_sts =
					rc_dev->c2c_send[queue].queue_status;
					rc_queue_sts->status = GXPCI_CHAN_RESET;
				}

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the sender
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
			 	 */
			}
		}
	}

	/* Do it again later. */
	schedule_delayed_work(&rc_dev->c2c_queue_monitor_work,
			      GXPCI_QUEUE_MONITOR_INTERVAL);
}


static void c2c_manager(struct work_struct *work)
{
	struct tlr_pcie_dev *tlr =
		container_of(work, struct tlr_pcie_dev, ep_setup_work.work);
	struct tlr_pcie_rc_c2c_dev *rc_dev;
	unsigned int ep_status;
	int pci_domain_num;

	pci_domain_num = pci_domain_nr(tlr->pci_dev->bus);
	rc_dev = gxpci_rc_devs[pci_domain_num];

	/* If the EP is not booted yet, don't issue any MMIO. */
	if (gx_ep_not_booted(tlr)) {
		schedule_delayed_work(&tlr->ep_setup_work,
				      C2C_LINK_SETUP_TIMEOUT);
		return;
	}

	ep_status = readl(&tlr->regs->port_status);

	/* If this EP is ready, set it up for C2C. */
	if (ep_status == PCIE_STATUS_READY) {
		int link_index = tlr->link_index;
		int i;

		tlr->is_ready = 1;

		/* Set the number of host NIC queues supported. */
		writel(tlr->nic_rx_queues, &tlr->regs->nic_t2h_queues);
		writel(tlr->nic_tx_queues, &tlr->regs->nic_h2t_queues);

		/*
 		 * Inform the EP of the non-C2C queue status array bus address.
 		 */
		writeq(tlr->queue_sts_bus_addr,
		       &tlr->regs->queue_sts_array_bus_addr);

		/* Inform the EP of the C2C queue status array bus address. */
		writeq(rc_dev->c2c_queue_sts_array_bus_addr,
		       &tlr->regs->c2c_queue_sts_array_bus_addr);

		writel(pci_resource_len(tlr->pci_dev, 0),
		       &tlr->regs->bar0_size);
		writel(pci_resource_len(tlr->pci_dev, 2),
		       &tlr->regs->bar2_size);

		/* Save this EP's BAR0 and BAR2 address. */
		rc_dev->c2c_queue_sts_array->bar0_addr[link_index] =
			tlr->bar0_address;
		rc_dev->c2c_queue_sts_array->bar2_addr[link_index] =
			tlr->bar2_address;

		writeq(tlr->bar0_address, &tlr->regs->bar0_addr);
		writeq(tlr->bar2_address, &tlr->regs->bar2_addr);

		/*
		 * If this device doesn't provide IP networking over
		 * the inter-Gx PCIe link, don't enable the interrupts.
		 */
		if (!(net_macs &
		     (1 << (rc_dev->mac + 4 * rc_dev->trio_index)))) {

			/* Write the link index to BAR0. */
			writel(link_index, &tlr->regs->link_index);

			return;
		}

		/*
		 * Enable MSI-X or MSI for a Gx RC device.
		 * Note that this must be done here, instead of in function
		 * gxpci_net_devs_init() in the probe stage, because function
		 * pci_enable_msix() issues MMIO access to the MSI-X table
		 * in the BAR0 space which can be accessed only after the
		 * endpoint driver signals PCIE_STATUS_READY in port_status.
		 *
		 * Ideally we want to have one interrupt vector per queue.
		 * If that is not possible, we fall back to using MSI with
		 * a single vector, instead of mapping the allocated vectors
		 * to the queues.
		 */
		for (i = 0; i < tlr->msix_vectors; i++)
			tlr->msix_entries[i].entry = i;

		i = pci_enable_msix(tlr->pci_dev, tlr->msix_entries,
				    tlr->msix_vectors);
		if (i) {
			/* MSI-X allocation failed. */
			dev_err(&tlr->pci_dev->dev,
				"MSI-X allocation failure: %d, try MSI\n", i);

			tlr->msix_vectors = 0;
			i = pci_enable_msi(tlr->pci_dev);
			if (i) {
				dev_err(&tlr->pci_dev->dev,
					"MSI allocation failure: %d, give up\n",
					i);
				return;
			}
		}

		/* Write the link index to BAR0. */
		writel(link_index, &tlr->regs->link_index);

	} else
		schedule_delayed_work(&tlr->ep_setup_work,
				      C2C_LINK_SETUP_TIMEOUT);
}


static int probe(struct pci_dev *pci_dev, const struct pci_device_id *id)
{
	struct tlr_pcie_rc_c2c_dev *rc_dev;
	struct pci_bus_region bus_addr;
	struct tlr_pcie_dev *tlr;
	int host_nic_queue_vectors;
	int total_nic_queues;
	int pci_domain_num;
	int link_index;
	int err;

	pci_domain_num = pci_domain_nr(pci_dev->bus);
	link_index = ep_link_index[pci_domain_num];

	BUG_ON(link_index >= MAX_PCIE_PORTS_PER_DOMAIN);

	/* Our device can use 64-bit DMA addresses. */
	if (!pci_set_dma_mask(pci_dev, DMA_BIT_MASK(64))) {
		err = pci_set_consistent_dma_mask(pci_dev, DMA_BIT_MASK(64));
		if (!err)
			TRACE("Using 64-bit PCI DMA addressing\n");
	} else {
		if (pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32))) {
			if ((err = pci_set_consistent_dma_mask(pci_dev,
				DMA_BIT_MASK(32)))) {
				ERR("Failed to find usable DMA "
				    "configuration\n");
				return err;
			}
		}
	}

	/* Get some memory for this device's driver state. */
	tlr = kmalloc(sizeof(*tlr), GFP_KERNEL);
	if (tlr == NULL)
		return -ENOMEM;
	memset(tlr, 0, sizeof(*tlr));

	/* Initialize TLR object */
	tlr->pci_dev = pci_dev;
	dev_set_drvdata(&pci_dev->dev, tlr);

	gx_ep_ports[pci_domain_num][link_index - 1] = tlr;

	/* Parse NIC queue configurations. */
	if (nic_ports <= 0 || nic_ports > GXPCI_HOST_NIC_COUNT) {
		ERR("%s: Illegal host NIC port number, default configuration "
		    "will be used instead.\n", driver_name);
		nic_ports = GXPCI_HOST_NIC_COUNT;
	}
	tlr->nic_ports = nic_ports;

	if (nic_tx_queues != nic_rx_queues) {
		ERR("%s: NIC queue number not equal, use min value.\n",
		    driver_name);
		nic_tx_queues = MIN(nic_tx_queues, nic_rx_queues);
		nic_rx_queues = nic_rx_queues;
	}

	total_nic_queues = nic_ports * nic_tx_queues;
	if (total_nic_queues <= 0 ||
	    nic_tx_queues > GXPCI_HOST_NIC_TX_QUEUES ||
	    nic_rx_queues > GXPCI_HOST_NIC_RX_QUEUES) {
		ERR("%s: NIC queue number invalid, use default value.\n",
		    driver_name);
		nic_tx_queues = GXPCI_HOST_NIC_TX_QUEUES;
		nic_rx_queues = GXPCI_HOST_NIC_RX_QUEUES;
	}
	tlr->nic_tx_queues = nic_tx_queues;
	tlr->nic_rx_queues = nic_rx_queues;

	/* Set the number of MSI-X interrupt vectors requested. */
	host_nic_queue_vectors = MAX(tlr->nic_tx_queues, tlr->nic_rx_queues);
	tlr->msix_vectors = GXPCI_HOST_CHAR_STREAMS_COUNT +
		GXPCI_HOST_ZC_QUEUE_COUNT +
		tlr->nic_ports * host_nic_queue_vectors;
#ifdef GXPCI_INTR_VECTOR_PER_QUEUE
	tlr->msix_vectors *= 2;
	tlr->msix_host_nic_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		(GXPCI_HOST_CHAR_STREAMS_COUNT + GXPCI_HOST_ZC_QUEUE_COUNT) * 2;
#else
	tlr->msix_host_nic_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		GXPCI_HOST_CHAR_STREAMS_COUNT + GXPCI_HOST_ZC_QUEUE_COUNT;
#endif
	tlr->msix_entries =kmalloc(sizeof(struct msix_entry) *
				   tlr->msix_vectors, GFP_KERNEL);
	if (tlr->msix_entries == NULL) {
		err = -ENOMEM;
		goto alloc_msix_table_failed;
	}
	memset(tlr->msix_entries, 0,
	       sizeof(struct msix_entry) * tlr->msix_vectors);

	/* Global link index 0 is reserved for the RC port. */
	tlr->link_index = link_index;

	pcibios_resource_to_bus(pci_dev, &bus_addr, &pci_dev->resource[0]);
	tlr->bar0_address = bus_addr.start;
	pcibios_resource_to_bus(pci_dev, &bus_addr, &pci_dev->resource[2]);
	tlr->bar2_address = bus_addr.start;

	spin_lock_init(&tlr->is_ready_lock);
	mutex_init(&tlr->boot_mutex);

	/* Enable the device. */
	err = pci_enable_device_mem(pci_dev);
	if (err != 0)
		goto enable_failed;

	/* Map in the memory-mapped IO registers. */
	tlr->regs = ioremap(pci_resource_start(pci_dev, 0) +
			    GXPCI_HOST_REGS_OFFSET,
			    sizeof(struct gxpci_host_regs));

	if (tlr->regs == NULL) {
		ERR("Failed to map Tilera Gx MMIO regs\n");
		err = -ENOMEM;
		goto map_failed;
	}

	/* Figure out what kind of chip architecture it is. */
	fill_chip_version(tlr);

	/* Map in the rshim registers. */
	if (tlr->chip_version == 10) {
		tlr->rshim_regs = ioremap(pci_resource_start(pci_dev, 0) +
					  GXPCI_RSHIM_WINDOW_OFFSET,
					  GXPCI_RSHIM_WINDOW_SIZE);
		if (tlr->rshim_regs == NULL) {
			ERR("Failed to map rshim registers\n");
			err = -ENOMEM;
			goto rshim_map_failed;
		}
	}

	fill_chip_size(tlr);

	/* Create our character and boot devices. */
	err = gx_add_device_nodes(tlr);
	if (err != 0)
		goto cdev_failed;

	/* Enable PCI bus mastering. */
	pci_set_master(pci_dev);

	/* Allocate and map storage for non-C2C queue status array. */
	tlr->queue_sts_array = pci_alloc_consistent(pci_dev, 
		sizeof(struct gxpci_queue_status_array),
		&tlr->queue_sts_bus_addr);
	if (tlr->queue_sts_array == NULL) {
		err = -ENOMEM;
		goto sts_array_failed;
	}

	/*
	 * For each PCI domain, allocate and map storage for
	 * the C2C queue status array, which are accessed not only
	 * by every Gx EP node but also the RC node when the C2C
	 * queue has one end running on the RC node.
	 */
	rc_dev = gxpci_rc_devs[pci_domain_num];
	if (rc_dev->c2c_queue_sts_array == NULL) {
		rc_dev->c2c_queue_sts_array = pci_alloc_consistent(pci_dev,
			sizeof(struct gxpci_c2c_queue_sts_array),
			&rc_dev->c2c_queue_sts_array_bus_addr);
		if (rc_dev->c2c_queue_sts_array == NULL) {
			err = -ENOMEM;
			goto c2c_sts_array_failed;
		}

		/* Set the addresses of the emulated BARs on the RC port. */
		rc_dev->c2c_queue_sts_array->bar0_addr[0] =
			PCIE_HOST_BAR0_START;
		rc_dev->c2c_queue_sts_array->bar2_addr[0] =
			PCIE_HOST_BAR2_START;

		/*
		 * Schedule work to monitor C2C queues' status.
		 */
		INIT_DELAYED_WORK(&rc_dev->c2c_queue_monitor_work,
				  c2c_queue_monitor);
		schedule_delayed_work(&rc_dev->c2c_queue_monitor_work,
				      GXPCI_QUEUE_MONITOR_INTERVAL);
	}

	/* Create the PCIe network device. */
	if (net_macs & (1 << (rc_dev->mac + 4 * rc_dev->trio_index)))
		gxpci_net_devs_init(tlr);

	/*
	 * Schedule work to manage the chip-to-chip connection setup.
	 */
	INIT_DELAYED_WORK(&tlr->ep_setup_work, c2c_manager);
	schedule_delayed_work(&tlr->ep_setup_work, C2C_LINK_SETUP_TIMEOUT);

	dev_info(&pci_dev->dev,
		 "Tilera Gx PCIe endpoint device, domain %d link index %d\n",
		 pci_domain_num, link_index);

	ep_link_index[pci_domain_num]++;

	return 0;

 c2c_sts_array_failed:
	pci_free_consistent(pci_dev, sizeof(struct gxpci_queue_status_array),
			    tlr->queue_sts_array,
			    tlr->queue_sts_bus_addr);
 sts_array_failed:
	gx_remove_device_nodes(tlr);
 cdev_failed:
	iounmap(tlr->rshim_regs);
 rshim_map_failed:
	iounmap(tlr->regs);
 map_failed:
	pci_disable_device(pci_dev);
 enable_failed:
	kfree(tlr->msix_entries);
 alloc_msix_table_failed:
	kfree(tlr);
	return err;
}


/* Called via pci_unregister_driver() when the module is removed. */
static void remove(struct pci_dev *pci_dev)
{
	struct tlr_pcie_dev *tlr = dev_get_drvdata(&pci_dev->dev);
	struct tlr_pcie_rc_c2c_dev *rc_dev;
	int pci_domain_num;

	pci_domain_num = pci_domain_nr(pci_dev->bus);
	rc_dev = gxpci_rc_devs[pci_domain_num];

	ep_link_index[pci_domain_num]--;

	if (rc_dev->c2c_queue_sts_array) {
		cancel_delayed_work_sync(&rc_dev->c2c_queue_monitor_work);

		pci_free_consistent(pci_dev,
			sizeof(struct gxpci_c2c_queue_sts_array),
			rc_dev->c2c_queue_sts_array,
			rc_dev->c2c_queue_sts_array_bus_addr);
		rc_dev->c2c_queue_sts_array = NULL;
		rc_dev->c2c_queue_sts_array_bus_addr = 0;
	}

	/* Remove the PCIe network device. */
	if (net_macs & (1 << (rc_dev->mac + 4 * rc_dev->trio_index)))
		gxpci_net_devs_remove(tlr);

	pci_free_consistent(pci_dev, sizeof(struct gxpci_queue_status_array),
			    tlr->queue_sts_array,
			    tlr->queue_sts_bus_addr);

	gx_remove_device_nodes(tlr);

	iounmap(tlr->regs);

	cancel_delayed_work_sync(&tlr->ep_setup_work);

	pci_disable_device(pci_dev);

	kfree(tlr->msix_entries);
	kfree(tlr);

	dev_set_drvdata(&pci_dev->dev, NULL);
}


static struct pci_driver pci_driver = {
	.name = "gxpci_host_subset",
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};


/*
 * Print the association of dev major number, port ID, link index, PCI domain
 * and bus number.
 */
static int
tlr_read_gxeps_major_mac_domain_link(char *buf, char **start, off_t offset,
				     int count, int *eof, void *data)
{
	char *scratch;
	int scratch_size;
	struct tlr_pcie_rc_c2c_dev *rc_dev;
	char *next;
	int result = 0;
	int dom;
	int i;

	/* Each line has format "MMM trioX-macX D:B L\n" */
	scratch_size = TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES *
		MAX_PCIE_PORTS_PER_DOMAIN * 30;
	scratch = kmalloc(scratch_size, GFP_KERNEL);
	if (scratch == NULL)
		return 0;
	memset(scratch, 0, scratch_size);
	next = scratch;

	for (dom = 0; dom < TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES; dom++) {

		rc_dev = gxpci_rc_devs[dom];
		if (!rc_dev)
			continue;

		for (i = 0; i < MAX_PCIE_PORTS_PER_DOMAIN - 1; i++) {
			struct tlr_pcie_dev *ep_dev;
			int bytes;

			ep_dev = gx_ep_ports[dom][i];

			/* Stop at the first uninitialized or absent EP node. */
			if (!ep_dev)
				break;

			bytes = sprintf(next, "%d trio%d-mac%d %04x:%02x %d\n",
				MAJOR(ep_dev->first_dev), rc_dev->trio_index,
				rc_dev->mac, rc_dev->domain,
				ep_dev->pci_dev->bus->number,
				ep_dev->link_index);
			next += bytes;
			result += bytes;
		}
	}
	result = min(result, count);
	memcpy(buf, scratch, result);
	*eof = 1;
	kfree(scratch);
	return result;
}


static int __init gxpci_host_subset_init(void)
{
	if (sim_is_simulator() || !num_rc_controllers)
		return 0;

	create_proc_read_entry("driver/gxpci_gxeps_major_mac_domain_link", 0,
				NULL, tlr_read_gxeps_major_mac_domain_link,
				NULL);

	return pci_register_driver(&pci_driver);
}


static void __exit gxpci_host_subset_exit(void)
{
	pci_unregister_driver(&pci_driver);
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tilera Corporation");

module_init(gxpci_host_subset_init);

module_exit(gxpci_host_subset_exit);
