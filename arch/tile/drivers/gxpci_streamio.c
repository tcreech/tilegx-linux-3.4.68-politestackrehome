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
 * Tilera Gx StreamIO port driver
 *
 * This source code is derived from code provided in "Linux Device
 * Drivers, Third Edition", by Jonathan Corbet, Alessandro Rubini, and
 * Greg Kroah-Hartman, published by O'Reilly Media, Inc.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/io.h>

#include <asm/compat.h>
#include <asm/tilegxpci.h>

#include <gxio/iorpc_globals.h>
#include <gxio/iorpc_trio.h>
#include <gxio/kiorpc.h>
#include <gxio/trio.h>
#include <hv/drv_trio_intf.h>

#include <arch/sim.h>

extern gxio_trio_context_t trio_contexts[TILEGX_NUM_TRIO];

#define DRIVER_NAME_STRING "tilegx_streamio"

static const char driver_name[] = DRIVER_NAME_STRING;

static const char *streamio_port_rates[] = {"2.5", "3.125", "5", "6.25"};

/* Information on the StreamIO ports configuration. */
static int __initdata pcie_streamio[TILEGX_NUM_TRIO][TILEGX_TRIO_PCIES];

/* Mask of CPUs that should receive PCIe interrupts. */
static struct cpumask intr_cpus_map;

/*
 * The HV doesn't allow the same MMIO ranges to be mapped multiple
 * times. Therefore, all drivers should share a single set of TRIO
 * contexts.
 *
 * This has dependency on the PCIe RC driver which defines and
 * initializes the TRIO contexts, which are declared in <linux/pci.h>.
 */

/* Array of the PCIe ports configuration info obtained from the BIB.
 * This is filled out by the RC driver.
 */
extern struct pcie_trio_ports_property pcie_ports[TILEGX_NUM_TRIO];

/* StreamIO device structure definition. */
struct tlr_sio_dev {

	struct cdev cdev;

	dev_t first_dev;

	/* Pointer back to the TRIO that this port is connected to. */
	gxio_trio_context_t *trio;

	/* Index of TRIO shim that contains the MAC. */
	int trio_index;

	/* MAC index on the TRIO shim. */
	int mac;

	/* Port status, up or down. */
	int status;

	/* Port width, values 1: x1, 2: x2, 4: x4. */
	uint32_t port_width;

	/*
	 * Port rate, values: 0: 2.5Gbps, 1: 3.125Gbps,
	 * 2: 5Gbps, 3: 6.25Gbps.
	 */
	uint32_t port_rate;

	/* Lock for error detect device. */
	spinlock_t lock;

	/* Semaphore for error detect device. */
	struct semaphore mutex;

	/* Open counter for error detect device. */
	int open_count;

	/* Irq number of the StreamIO MAC error interrupt. */
	int irq;

	/* StreamIO MAC error status. */
	uint32_t sio_err;

	/* Queue of processes waiting for error detect interrupt. */
	wait_queue_head_t err_detect_queue;

};

static struct tlr_sio_dev *streamio_devs;
static int num_streamio_devs;

/* Port width, values 1: x1, 2: x2, 4: x4. */
static int streamio_port_width = 4;
module_param(streamio_port_width, int, S_IRUGO);
MODULE_PARM_DESC(streamio_port_width, "Width of the StreamIO port.");

/* Port rate, values: 0: 2.5Gbps, 1: 3.125Gbps, 2: 5Gbps, 3: 6.25Gbps. */
static int streamio_port_rate;
module_param(streamio_port_rate, int, S_IRUGO);
MODULE_PARM_DESC(streamio_port_rate, "Rate of the StreamIO port.");

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


/*
 * Return 1 if the port is strapped to operate in StreamIO x1 mode,
 * 4 for StreamIO x4 mode, or 0 if not strapped in StreamIO mode.
 */
static int
strapped_for_streamio(gxio_trio_context_t *trio_context, int mac)
{
	TRIO_PCIE_INTFC_PORT_CONFIG_t port_config;
	unsigned int reg_offset;

	/* Check the port configuration. */
	reg_offset =
		(TRIO_PCIE_INTFC_PORT_CONFIG <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT ) |
		(mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	port_config.word =
		__gxio_mmio_read(trio_context->mmio_base_mac + reg_offset);

	if (port_config.ovd_dev_type)
		return 0;

	if (port_config.strap_state ==
		TRIO_PCIE_INTFC_PORT_CONFIG__STRAP_STATE_VAL_STREAM_X1)
		return 1;
	else if (port_config.strap_state ==
		TRIO_PCIE_INTFC_PORT_CONFIG__STRAP_STATE_VAL_STREAM_X4)
		return 4;
	else
		return 0;
}


/*
 * Return 1 if the StreamIO link is up, otherwise 0.
 */
static int
is_link_up(gxio_trio_context_t *trio_context, int mac)
{
	TRIO_PCIE_INTFC_PORT_STATUS_t port_status;
	unsigned int reg_offset;

	/* Check for the StreamIO link-up status. */
	reg_offset =
		(TRIO_PCIE_INTFC_PORT_STATUS <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT ) |
		(mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	port_status.word =
		__gxio_mmio_read(trio_context->mmio_base_mac + reg_offset);

	if (port_status.dl_up)
		return 1;
	else
		return 0;
}


/*
 * This function configures the MAC to operate in StreamIO mode, using
 * pre-specified port width and rate.
 */
static void
config_streamio(struct tlr_sio_dev *sio_dev)
{
	gxio_trio_context_t *trio_context = sio_dev->trio;
	TRIO_PCIE_INTFC_PORT_CONFIG_t port_config;
	unsigned int reg_offset;

	/* Check the port configuration. */
	reg_offset =
		(TRIO_PCIE_INTFC_PORT_CONFIG <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT ) |
		(sio_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	port_config.word =
		__gxio_mmio_read(trio_context->mmio_base_mac + reg_offset);

	port_config.train_mode =
		TRIO_PCIE_INTFC_PORT_CONFIG__TRAIN_MODE_VAL_TRAIN_DIS;
	port_config.ovd_dev_type = 1;
	port_config.ovd_dev_type_val =
		TRIO_PCIE_INTFC_PORT_CONFIG__OVD_DEV_TYPE_VAL_VAL_STREAM;
	switch (sio_dev->port_width) {
		case 0:
			port_config.stream_width =
			TRIO_PCIE_INTFC_PORT_CONFIG__STREAM_WIDTH_VAL_X1;

			break;

		case 2:
			port_config.stream_width =
			TRIO_PCIE_INTFC_PORT_CONFIG__STREAM_WIDTH_VAL_X2;

			break;

		case 4:
			port_config.stream_width =
			TRIO_PCIE_INTFC_PORT_CONFIG__STREAM_WIDTH_VAL_X4;

			break;
	}
	port_config.stream_rate = sio_dev->port_rate;

	__gxio_mmio_write(trio_context->mmio_base_mac + reg_offset,
			  port_config.word);

	port_config.word =
		__gxio_mmio_read(trio_context->mmio_base_mac + reg_offset);
	port_config.train_mode =
		TRIO_PCIE_INTFC_PORT_CONFIG__TRAIN_MODE_VAL_TRAIN_ENA;

	__gxio_mmio_write(trio_context->mmio_base_mac + reg_offset,
			  port_config.word);

	mdelay(200);
	return;
}


/**********************************************************************/
/*                        Port Error Detect File                      */
/**********************************************************************/

static irqreturn_t sio_mac_err_intr(int irq, void *streamio_dev)
{
	struct tlr_sio_dev *sio_dev = (struct tlr_sio_dev*) streamio_dev;
	gxio_trio_context_t *trio_context = sio_dev->trio;
	TRIO_PCIE_INTFC_MAC_INT_STS_t int_sts;
	TRIO_PCIE_INTFC_STREAM_STATUS_t sio_sts;
	unsigned int reg_offset;
	unsigned long flags;

	/* Check the MAC interrupt status. */
	reg_offset =
		(TRIO_PCIE_INTFC_MAC_INT_STS <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT ) |
		(sio_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	int_sts.word = __gxio_mmio_read(trio_context->mmio_base_mac +
					reg_offset);

	/* Make sure this interrupt comes from StreamIO MAC error. */
	if ((int_sts.word &
	     1 << TRIO_PCIE_INTFC_MAC_INT_STS__STREAM_IO_ERROR_SHIFT) == 0)
		return IRQ_NONE;

	spin_lock_irqsave(&sio_dev->lock, flags);

	/* Clear interrupt status. */
	__gxio_mmio_write(trio_context->mmio_base_mac + reg_offset,
		(1 << TRIO_PCIE_INTFC_MAC_INT_STS__STREAM_IO_ERROR_SHIFT));

	/* Check the exact StreamIO MAC errors. */
	reg_offset =
		(TRIO_PCIE_INTFC_STREAM_STATUS <<
			TRIO_CFG_REGION_ADDR__REG_SHIFT) |
		(TRIO_CFG_REGION_ADDR__INTFC_VAL_MAC_INTERFACE <<
			TRIO_CFG_REGION_ADDR__INTFC_SHIFT ) |
		(sio_dev->mac << TRIO_CFG_REGION_ADDR__MAC_SEL_SHIFT);
	sio_sts.word = __gxio_mmio_read(trio_context->mmio_base_mac +
					reg_offset);

	/* Record the errors after masking all non-error bits. */
	sio_sts.word &= (TRIO_PCIE_INTFC_STREAM_STATUS__FRAMING_ERR_MASK |
			 TRIO_PCIE_INTFC_STREAM_STATUS__CRC_ERR_MASK |
			 TRIO_PCIE_INTFC_STREAM_STATUS__FIFO_ERR_MASK |
			 TRIO_PCIE_INTFC_STREAM_STATUS__DEC_ERR_MASK);
	sio_dev->sio_err = (uint32_t) sio_sts.word;

	/* Wake up the sleeped ioclt(). */
	wake_up_interruptible(&sio_dev->err_detect_queue);

	spin_unlock_irqrestore(&sio_dev->lock, flags);

	return IRQ_HANDLED;
}


static long sio_err_detect_ioctl(struct file *filp, unsigned int cmd,
				 unsigned long arg)
{
	struct tlr_sio_dev *sio_dev = filp->private_data;
	unsigned long flags;

	switch (cmd) {
	case TILEPCI_IOC_STREAMIO_GET_MAC_ERR:

		/* Sleep if there is no error detected. */
		while (sio_dev->sio_err == 0) {
			if (wait_event_interruptible(sio_dev->err_detect_queue,
						     sio_dev->sio_err != 0))
				return -ERESTARTSYS;
		}

		/* At this point there comes StreamIO MAC errors. */
		if (copy_to_user((void __user *) arg, &sio_dev->sio_err,
				 sizeof(tilegxpci_sio_mac_err_t)))
			return -EFAULT;
		
		/* Clear the local error status. */
		spin_lock_irqsave(&sio_dev->lock, flags);
		sio_dev->sio_err = 0;
		spin_unlock_irqrestore(&sio_dev->lock, flags);

		break;

	default:
		return -EINVAL;
	}

	return 0;
}


#ifdef CONFIG_COMPAT
static long sio_err_detect_compat_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return sio_err_detect_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif


static int sio_err_detect_release(struct inode *inode, struct file *filp)
{
	struct tlr_sio_dev *sio_dev = filp->private_data;
	gxio_trio_context_t *trio_context = sio_dev->trio;

	if (down_interruptible(&sio_dev->mutex))
		return -ERESTARTSYS;

	if (--sio_dev->open_count == 0) {

		/* Disable StreamIO MAC error interrupts. */
		gxio_trio_unconfig_sio_mac_err_intr(trio_context, sio_dev->mac);

		/* Free irq resources. */
		if (sio_dev->irq >= 0) {
			free_irq(sio_dev->irq, (void *) sio_dev);
			destroy_irq(sio_dev->irq);

			sio_dev->irq = -1;
		}
	}

	up(&sio_dev->mutex);

	return 0;
}


static const struct file_operations sio_err_detect_ops = {
	.owner 		= THIS_MODULE,
	.release	= sio_err_detect_release,
	.unlocked_ioctl = sio_err_detect_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = sio_err_detect_compat_ioctl,
#endif
};


static int sio_err_detect_open(struct tlr_sio_dev *sio_dev, struct file *filp)
{
	gxio_trio_context_t *trio_context = sio_dev->trio;
	int err;
	int irq;
	int cpu;

	filp->private_data = sio_dev;
	filp->f_op = &sio_err_detect_ops;

	if (down_interruptible(&sio_dev->mutex))
		return -ERESTARTSYS;

	/* Initialize at the 1st time. */
	if (sio_dev->open_count++ > 0) {
		up(&sio_dev->mutex);
		return 0;
	}

	/* Allocate a common interrupt for StreamIO MAC error to kernel. */
	irq = create_irq();
	if (irq < 0)
		goto irq_failed;
	sio_dev->irq = irq;

	tile_irq_activate(irq, TILE_IRQ_PERCPU);
	if (request_irq(irq, sio_mac_err_intr, 0, driver_name, 
			(void *) sio_dev))
		goto irq_failed;

	/* Try to choose a non-dataplane processor to receive the interrupt. */
	cpu = tile_irq_cpu(irq);

	/* Register and bind the interrupt to TRIO. */
	err = gxio_trio_config_sio_mac_err_intr(trio_context,
						cpu_x(cpu), cpu_y(cpu),
						KERNEL_PL, irq, sio_dev->mac);
	if (err < 0)
		goto irq_failed;

	up(&sio_dev->mutex);

	return 0;

irq_failed:
	if (irq >= 0) {
		destroy_irq(irq);
		sio_dev->irq = -1;
		sio_dev->open_count--;
	}

	up(&sio_dev->mutex);

	return err;
}


/**********************************************************************/
/*                        Port Information File                       */
/**********************************************************************/

static int sio_info_seq_show(struct seq_file *s, void *token)
{
	struct tlr_sio_dev *streamio_dev = s->private;

	seq_printf(s, "PORT_STATUS %s\n",
		   streamio_dev->status ? "up" : "down");
	seq_printf(s, "PORT_WIDTH x%d\n", streamio_dev->port_width);
	seq_printf(s, "PORT_RATE %s Gbps\n",
		   streamio_port_rates[streamio_dev->port_rate]);

	return 0;
}


static struct file_operations sio_info_ops = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int sio_info_open(struct tlr_sio_dev *sio_dev, struct file *filp)
{
        filp->f_op = &sio_info_ops;
        return single_open(filp, sio_info_seq_show, sio_dev);
}


/**********************************************************************/
/*                        Port Configuration File                     */
/**********************************************************************/

static long sio_config_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct tlr_sio_dev *streamio_dev = filp->private_data;
	gxio_trio_context_t *trio_context = streamio_dev->trio;
	tilegxpci_sio_config_t sio_config;

	switch (cmd) {
	case TILEPCI_IOC_STREAMIO_CONFIG:
		if (copy_from_user(&sio_config, (void __user *)arg,
				   sizeof(tilegxpci_sio_config_t)))
			return -EFAULT;

		if (sio_config.port_width != 1 &&
		    sio_config.port_width != 2 &&
		    sio_config.port_width != 4)
			return -EINVAL;
		if (sio_config.port_rate >=
		    sizeof(streamio_port_rates)/sizeof(char *))
			return -EINVAL;

		streamio_dev->port_width = sio_config.port_width;
		streamio_dev->port_rate = sio_config.port_rate;
		break;

	case TILEPCI_IOC_STREAMIO_LINKUP:
		config_streamio(streamio_dev);

		streamio_dev->status =
			is_link_up(trio_context, streamio_dev->mac);
		if (!streamio_dev->status)
			return -ENXIO;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}


#ifdef CONFIG_COMPAT
static long sio_config_compat_ioctl(struct file *filp, unsigned int cmd,
				    unsigned long arg)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return sio_config_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif


static const struct file_operations sio_config_ops = {
	.owner 		= THIS_MODULE,
	.unlocked_ioctl = sio_config_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = sio_config_compat_ioctl,
#endif
};


static int sio_config_open(struct tlr_sio_dev *sio_dev, struct file *filp)
{
	filp->private_data = sio_dev;
	filp->f_op = &sio_config_ops;

	return 0;
}


static int tlr_sio_generic_open(struct inode *inode, struct file *filp)
{
	unsigned int	 minor = MINOR(inode->i_rdev);
	int		 result;
	struct tlr_sio_dev *tlr =
		container_of(inode->i_cdev, struct tlr_sio_dev, cdev);

	if (minor == GX_SIO_CONFIG_MINOR) {
		result = sio_config_open(tlr, filp);
	} 
	else if (minor == GX_SIO_INFO_MINOR) {
		result = sio_info_open(tlr, filp);
	} 
	else if (minor == GX_SIO_ERR_DETECT_MINOR) {
		result = sio_err_detect_open(tlr, filp);
	}
	else {
		result = -ENODEV;
	}

	return result;
}


static const struct file_operations tlr_sio_generic_ops = {
	.owner = THIS_MODULE,
	.open = tlr_sio_generic_open,
};


static int tlr_cdev_setup(struct tlr_sio_dev *tlr)
{
	struct cdev *cdev = &tlr->cdev;

	/* Allocate some major/minor numbers. */
	dev_t first;
	int err = alloc_chrdev_region(&first, 0, GX_SIO_MINOR_DEVICES,
				      driver_name);
	if (err != 0)
		return err;

	/* Register the device. */
	cdev_init(cdev, &tlr_sio_generic_ops);
	cdev->owner = THIS_MODULE;
	err = cdev_add(cdev, first, GX_SIO_MINOR_DEVICES);
	if (err != 0) {
		unregister_chrdev_region(first, GX_SIO_MINOR_DEVICES);
		return err;
	}
	tlr->first_dev = first;

	return 0;
}


/*
 * Print the association of the StreamIO port's major number and the MAC number.
 */
static int tlr_read_sio_major_mac(char *buf, char **start, off_t offset,
				  int count, int *eof, void *data)
{
	/* Each line has format "MMM trioX-macX\n" */
	char scratch[TILEGX_NUM_TRIO * TILEGX_TRIO_PCIES * 40];
	struct tlr_sio_dev *streamio_dev;
	char *next = scratch;
	int result = 0;
	int i;

	for (i = 0; i < num_streamio_devs; i++) {
		int bytes;

		streamio_dev = &streamio_devs[i];

		bytes = sprintf(next, "%d trio%d-mac%d\n",
				MAJOR(streamio_dev->first_dev),
				streamio_dev->trio_index, streamio_dev->mac);
		next += bytes;
		result += bytes;
	}
	result = min(result, count);
	memcpy(buf, scratch, result);
	*eof = 1;
	return result;
}


/*
 * Create a device for the specified StreamIO link if the link is up.
 */
static int __init tlr_sio_dev_probe(int sio_index)
{
	struct tlr_sio_dev *streamio_dev = &streamio_devs[sio_index];
	gxio_trio_context_t *trio_context = streamio_dev->trio;
	int ret;

	ret = strapped_for_streamio(trio_context, streamio_dev->mac);
	if (ret) {
		streamio_dev->port_width = ret;
		streamio_dev->port_rate = 1;	/* 3.125Gbps. */
	} else {
		/*
		 * The port might be connected to a FPGA device that
		 * is dynamically programmed, and the link will be
		 * brought up later on.
		 */
		if (streamio_port_width != 1 &&
		    streamio_port_width != 2 &&
		    streamio_port_width != 4) {
			pr_warning("Invalid StreamIO port width setting, "
				   "force x1\n");
			streamio_port_width = 1;
		}
		if (streamio_port_rate >=
		    sizeof(streamio_port_rates)/sizeof(char *)) {
			pr_warning("Invalid StreamIO port width setting, "
				   "force 3.125 Gbps\n");
			streamio_port_rate = 1;
		}

		streamio_dev->port_width = streamio_port_width;
		streamio_dev->port_rate = streamio_port_rate;
	}

	/* Initialize the StreamIO MAC error interrupt number. */
	streamio_dev->irq = -1;

	/* Initialize the locks for error detect device. */
	spin_lock_init(&streamio_dev->lock);
	sema_init(&streamio_dev->mutex, 1);

	/* Initialize the waiting queue for error detect device. */
	init_waitqueue_head(&streamio_dev->err_detect_queue);

	streamio_dev->status = is_link_up(trio_context, streamio_dev->mac);

	/* Create character device (which has GX_SIO_MINOR_DEVICES nodes). */
	ret = tlr_cdev_setup(streamio_dev);
	if (ret != 0)
		return ret;

	return 0;
}


static int __init gx_streamio_init(void)
{
	int num_trio_shims = 0;
	int sio_index = 0;
	int i, j;

	if (sim_is_simulator())
		return 0;

	for (i = 0; i < TILEGX_NUM_TRIO; i++) {
		gxio_trio_context_t *context = &trio_contexts[i];

		if (context->fd < 0)
			continue;

		/*
		 * Allocate the ASID if it hasn't be allocated.
		 */
		if (context->asid == GXIO_ASID_NULL) {
			int ret;

			ret = gxio_trio_alloc_asids(context, 1, 0, 0);
			if (ret < 0) {
				pr_err("%s: TRIO %d ASID alloc failure %d\n",
					driver_name, i, ret);
				continue;
			}

			context->asid = ret;
		}

		num_trio_shims++;
	}

	if (num_trio_shims == 0)
		return 0;

	/*
	 * Now determine the StreamIO ports.
	 * We look at the Board Information Block first and then see if
	 * the port strap state requires otherwise.
	 */
#if 0
	for (i = 0; i < TILEGX_NUM_TRIO; i++) {
		gxio_trio_context_t *context = &trio_contexts[i];

		if (context->fd < 0)
			continue;

		for (j = 0; j < TILEGX_TRIO_PCIES; j++) {
			if (pcie_ports[i].ports[j].allow_sio) {
				pcie_streamio[i][j] = 1;
				num_streamio_devs++;
			}
		}
	}
#else
	/*
	 * For now, just assume that there is a single StreamIO port on trio/0.
	 */
        num_streamio_devs = 1;
        pcie_streamio[0][0] = 1;
#endif

	/*
	 * Return if no StreamIO ports are configured.
	 */
	if (num_streamio_devs == 0)
		return 0;

	streamio_devs = kmalloc(sizeof(struct tlr_sio_dev) * num_streamio_devs,
				GFP_KERNEL);
	if (streamio_devs == NULL)
		return -ENOMEM;

	memset(streamio_devs, 0,
	       sizeof(struct tlr_sio_dev) * num_streamio_devs);

	/*
	 * Set the TRIO pointer and MAC index for each StreamIO port.
	 */
	for (i = 0; i < TILEGX_NUM_TRIO; i++) {
		for (j = 0; j < TILEGX_TRIO_PCIES; j++) {
			if (pcie_streamio[i][j]) {
				streamio_devs[sio_index].trio = &trio_contexts[i];
				streamio_devs[sio_index].mac = j;
				streamio_devs[sio_index].trio_index = i;
				sio_index++;
				if (sio_index == num_streamio_devs)
					goto out;
			}
		}
	}

out:

	for (i = 0; i < num_streamio_devs; i++)
		tlr_sio_dev_probe(i);

	create_proc_read_entry("driver/gx_streamio_major_mac", 0,
				NULL, tlr_read_sio_major_mac, NULL);

	return 0;
}


static void __exit gx_streamio_exit(void)
{
	/*
	 * We're statically compiled into the kernel, so this should never
	 * be called.
	 */
	pr_err("Removing %s", driver_name);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tilera Corporation");

subsys_initcall(gx_streamio_init);
module_exit(gx_streamio_exit);
