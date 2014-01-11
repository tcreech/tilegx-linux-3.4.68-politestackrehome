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
 * Tilera Gx chip-to-chip transfer manager driver
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
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/io.h>
#include <linux/pci.h>

#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/tilegxpci.h>

#include <gxio/iorpc_globals.h>
#include <gxio/kiorpc.h>
#include <gxio/trio.h>
#include <gxio/iorpc_trio.h>

#include "gxpci_chip_to_chip.h"

#include <arch/sim.h>

/* Total waiting time for the reset acknowledgement from the other end. */
#define GXPCI_RELEASE_WAIT_MAX_MSECS	(1000)

static const char driver_name[] = "gxpci_chip_to_chip";

extern int num_rc_controllers;

struct tlr_pcie_rc_c2c_dev *gxpci_rc_devs[TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES];

static long rc_barmem_ioctl(struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
	struct tlr_pcie_rc_c2c_dev *rc_dev = filp->private_data;
	tilegxpci_bar_info_t bar_info;

	switch (cmd) {
	case TILEPCI_IOC_GET_BAR:
		if (copy_from_user(&bar_info, (void __user *)arg,
				   sizeof(tilegxpci_bar_info_t)))
			return -EFAULT;

		if (bar_info.link_index == 
		    (uint16_t)TILEGXPCI_LOCAL_LINK_INDEX) {

			bar_info.link_index = 0;
			if (bar_info.bar_index == 2)
				bar_info.bar_addr = PCIE_HOST_BAR2_START;
			else
				bar_info.bar_addr = PCIE_HOST_BAR0_START;

			/* We don't set the RC port's BAR size. */
		} else {
			struct gxpci_c2c_queue_sts_array *c2c_sts_array;
			unsigned int rem_link = bar_info.link_index;

			if (rem_link >= MAX_PCIE_PORTS_PER_DOMAIN)
				return -EINVAL;

			/*
 			 * If there are no Gx endpoint devices under this root
 			 * port, return error.
 			 */
			c2c_sts_array = rc_dev->c2c_queue_sts_array;
			if (c2c_sts_array == NULL)
				return -ENODEV;

			/* We don't need the remote BAR's size. */
			if (bar_info.bar_index == 2)
				bar_info.bar_addr =
					c2c_sts_array->bar2_addr[rem_link];
			else
				bar_info.bar_addr =
					c2c_sts_array->bar0_addr[rem_link];
		}
		if (copy_to_user((void __user *)arg, &bar_info,
			     sizeof(tilegxpci_bar_info_t)))
			return -EFAULT;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations rc_barmem_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = rc_barmem_ioctl,
};

static int rc_barmem_open(struct tlr_pcie_rc_c2c_dev *rc, struct file *filp)
{
	filp->private_data = rc;
	filp->f_op = &rc_barmem_ops;

	return 0;
}

/*
 * Allocate the queue status buf for the PCIe chip-to-chip queue.
 * This page is shared between this driver and the C2C application
 * running on the Gx RC node.
 */
int rc_c2c_mem_alloc(struct tlr_c2c_state *queue_state)
{
	struct page *page;

	page = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!page) {
		return -ENOMEM;
	} else {
		queue_state->queue_status = page_address(page);

		SetPageReserved(page);

		/*
		 * Increment page count so that the kernel won't
		 * try releasing this page when the application which
		 * has mmap'ed the page exits.
		 */
		get_page(page);
	}

	return 0;
}

static void rc_c2c_vma_open(struct vm_area_struct *vma)
{
	struct tlr_c2c_state *queue_state =
		(struct tlr_c2c_state*) vma->vm_private_data;

	down(&queue_state->mutex);

	queue_state->vmas++;

	up(&queue_state->mutex);
}

static void rc_c2c_vma_close(struct vm_area_struct *vma)
{
	struct tlr_c2c_state *queue_state =
		(struct tlr_c2c_state*) vma->vm_private_data;

	down(&queue_state->mutex);

	queue_state->vmas--;

	up(&queue_state->mutex);
}

static int rc_c2c_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct tlr_c2c_state *queue_state =
		(struct tlr_c2c_state*) vma->vm_private_data;
	struct page *page;
	void *page_ptr = NULL;
	unsigned long offset;

	unsigned long vaddr = (unsigned long)vmf->virtual_address;
	int ret;

	ret = VM_FAULT_SIGBUS;

	down(&queue_state->mutex);

	if (vaddr > vma->vm_end) {
		goto exit;
	}

	/* Ignore vma->vm_pgoff, which is unrelated here. */
	offset = vaddr - vma->vm_start;

	page_ptr = queue_state->queue_status + offset;

	page = virt_to_page(page_ptr);
	get_page(page);

	ret = 0;
	vmf->page = page;

 exit:
	up(&queue_state->mutex);

	return ret;
}

static struct vm_operations_struct rc_c2c_vm_ops = {
	.open	= rc_c2c_vma_open,
	.close	= rc_c2c_vma_close,
	.fault  = rc_c2c_vma_fault,
};

static int rc_c2c_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct tlr_c2c_state *queue_state = filp->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	/* Map the chip-to-chip queue's shared memory. */
	if (queue_state->queue_status == NULL)
		return -EFAULT;

	if (offset != 0) {
		printk("Tile PCIe chip-to-chip offset invalid\n");
		return -EINVAL;
	}

	if (!(vma->vm_flags & VM_SHARED)) {
		printk("Tile PCIe chip-to-chip status mmap flags"
			" must include VM_SHARED\n");
		return -EINVAL;
	}

	/* Don't try to swap out physical pages. */
	vma->vm_flags |= VM_RESERVED;

	vma->vm_ops = &rc_c2c_vm_ops;
	vma->vm_private_data = (void*) queue_state;

	rc_c2c_vma_open(vma);

	return 0;
}

/* This is called by the C2C sender application running on the Gx RC node. */
static long rc_c2c_ioctl(struct file *filp,
			 unsigned int cmd, unsigned long arg)
{
	struct tlr_c2c_state *queue_state = filp->private_data;
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

		queue_pair_status->sender_link_index = sender_link_index;
		queue_pair_status->receiver_link_index = receiver_link_index;
		queue_pair_status->sender_status = GXPCI_CHAN_RUNNING;
		queue_pair_status->receiver_status = GXPCI_CHAN_RUNNING;

		/*
		 * Activate the queue so that the C2C queue monitor can
		 * start monitoring the queue which is deactivated by
		 * the monitor.
		 */
		queue_pair_status->active = 1;

		break;
	case TILEPCI_IOC_C2C_SET_SENDER_READY:
		queue_pair_status->sender_status = GXPCI_TILE_CHAN_READY;

		break;
	case TILEPCI_IOC_C2C_GET_SENDER_READY:
		sender_ready = queue_pair_status->sender_status;
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
static long rc_c2c_compat_ioctl(struct file *filp,
				unsigned int a, unsigned long b)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return rc_c2c_ioctl(filp, a, (int)(long)b);
}
#endif

static int rc_c2c_release(struct inode *inode, struct file *filp)
{
	struct tlr_c2c_state *queue_state = filp->private_data;
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
	*queue_status = GXPCI_CHAN_RESET;

	while (*queue_status != GXPCI_CHAN_RESET_ACK) {
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
			*peer_status = GXPCI_CHAN_RESET;
			err = -EINTR;
			break;
		}
		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}

	/* Set the queue status to uninitialized state. */
	*queue_status = GXPCI_CHAN_UNINITIALIZED;

	return err;
}

static const struct file_operations rc_c2c_ops = {
	.owner = THIS_MODULE,
	.mmap = rc_c2c_mmap,
	.unlocked_ioctl = rc_c2c_ioctl,
#ifdef CONFIG_COMPAT
        .compat_ioctl = rc_c2c_compat_ioctl,
#endif
	.release = rc_c2c_release,
};

static int rc_c2c_open(struct inode *inode, struct file *filp)
{
	struct tlr_pcie_rc_c2c_dev *rc_dev =
		container_of(inode->i_cdev, struct tlr_pcie_rc_c2c_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	struct gxpci_c2c_queue_sts *queue_status;
	struct tlr_c2c_state *queue_state;
	int queue_index;
	int err;

	/* No C2C queue is allowed if there is no C2C queue status array. */
	if (rc_dev->c2c_queue_sts_array == NULL)
		return -ENODEV;

	/* Locate the queue status struct. */
	if (minor >= TILEPCI_FIRST_C2C_TX_MINOR) {
		queue_index = minor - TILEPCI_FIRST_C2C_TX_MINOR;
		queue_state = &rc_dev->c2c_send[queue_index]; 
	} else {
		queue_index = minor - TILEPCI_FIRST_C2C_RX_MINOR;
		queue_state = &rc_dev->c2c_recv[queue_index]; 
	}
	queue_status = &rc_dev->c2c_queue_sts_array->c2c_queue_sts[queue_index];

	/*
	 * Allocate the tlr_c2c_status struct as shared memory
	 * with the application.
	 */
	if (queue_state->queue_status == NULL) {
		err = rc_c2c_mem_alloc(queue_state);
		if (err != 0)
			return err;
	}

	/* Initialize the local queue status flag. */
	queue_state->queue_status->status = GXPCI_CHAN_UNINITIALIZED;

	queue_state->queue_pair_status = queue_status;
	filp->private_data = queue_state;
	filp->f_op = &rc_c2c_ops;

	return 0;
}

static int tlr_generic_open(struct inode *inode, struct file *filp)
{
	unsigned int	 minor = MINOR(inode->i_rdev);
	int		 result;
	struct tlr_pcie_rc_c2c_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_rc_c2c_dev, cdev);

	if (minor == GXPCI_C2C_BARMEM_MINOR) {
		result = rc_barmem_open(tlr, filp);
	}
	else if ((minor >= TILEPCI_FIRST_C2C_RX_MINOR) &&
		(minor <= (TILEPCI_FIRST_C2C_RX_MINOR +
                GXPCI_C2C_QUEUE_COUNT - 1))) {
		result = rc_c2c_open(inode, filp);
	}
	else if ((minor >= TILEPCI_FIRST_C2C_TX_MINOR) &&
		(minor <= (TILEPCI_FIRST_C2C_TX_MINOR +
		GXPCI_C2C_QUEUE_COUNT - 1))) {
		result = rc_c2c_open(inode, filp);
	} else {
		result = -ENODEV;
	}

	return result;
}

static const struct file_operations tlr_generic_ops = {
	.owner = THIS_MODULE,
	.open = tlr_generic_open,
};

static int tlr_cdev_setup(struct tlr_pcie_rc_c2c_dev *tlr)
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


/*
 * Print the association of dev major number, port ID, link index, and
 * PCI domain number.
 */
static int
tlr_read_rc_major_mac_link_domain(char *buf, char **start, off_t offset,
				  int count, int *eof, void *data)
{
	/* Each line has format "MMM trioX-macX 0 D\n" */
	char scratch[TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES * 30];
	struct tlr_pcie_rc_c2c_dev *rc_dev;
	char *next = scratch;
	int result = 0;
	int i;

	for (i = 0; i < num_rc_controllers; i++) {
		int bytes;

		rc_dev = gxpci_rc_devs[i];
		if (!rc_dev)
			continue;

		bytes = sprintf(next, "%d trio%d-mac%d 0 %d\n",
				MAJOR(rc_dev->first_dev), rc_dev->trio_index,
				rc_dev->mac, rc_dev->domain);
		next += bytes;
		result += bytes;
	}
	result = min(result, count);
	memcpy(buf, scratch, result);
	*eof = 1;
	return result;
}

/*
 * Create a device for the specified PCIE RC port.
 */
static int __init tlr_pcie_rc_dev_probe(int rc_index)
{
	struct pci_controller *rc_controller;
	gxio_trio_context_t *trio_context;
	struct tlr_pcie_rc_c2c_dev *rc_dev;
	int ret;
	int i;

	rc_controller = &pci_controllers[rc_index];
	trio_context = rc_controller->trio;

	/* Skip RC port that isn't initialized properly. */
	if (rc_controller->root_bus == NULL)
		return 0;

	rc_dev = kmalloc(sizeof(*rc_dev), GFP_KERNEL);
	if (rc_dev == NULL)
		return -ENOMEM;
	memset(rc_dev, 0, sizeof(*rc_dev));

	rc_dev->trio_index = rc_controller->trio_index;
	rc_dev->mac = rc_controller->mac;
	rc_dev->domain = rc_index;

	for (i = 0; i < GXPCI_C2C_QUEUE_COUNT; i++) {
		sema_init(&rc_dev->c2c_send[i].mutex, 1);
		sema_init(&rc_dev->c2c_recv[i].mutex, 1);
	}

	/* Create character device (which has GXPCI_NUM_MINOR_DEVICES nodes). */
	ret = tlr_cdev_setup(rc_dev);
	if (ret != 0)
		goto cdev_failed;

	gxpci_rc_devs[rc_index] = rc_dev;

	return 0;

 cdev_failed:
	kfree(rc_dev);
	return ret;
}


static int __init gxpci_c2c_init(void)
{
	int i;

	if (sim_is_simulator() || !num_rc_controllers)
		return 0;

	for (i = 0; i < num_rc_controllers; i++)
		tlr_pcie_rc_dev_probe(i);

	create_proc_read_entry("driver/gxpci_rc_major_mac_link_domain", 0, NULL,
			       tlr_read_rc_major_mac_link_domain, NULL);

	return 0;
}


static void __exit gxpci_c2c_exit(void)
{
	/*
	 * We're statically compiled into the kernel, so this should never
	 * be called.
	 */
	
	pr_err("Removing %s", driver_name);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tilera Corporation");

/*
 * We use fs_initcall here to ensure that this driver is initialized
 * before the Gx endpoint driver gxpci_host_subset is, due to the latter's
 * dependency on gxpci_rc_devs[].
 */
fs_initcall(gxpci_c2c_init);
module_exit(gxpci_c2c_exit);
