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
 * I2C Slave interface driver
 *
 * This source code is derived from code provided in "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published by
 * O'Reilly & Associates.
 */

#include <linux/cdev.h>
#include <linux/errno.h>	/* error codes */
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/fs.h>		/* everything... */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>		/* iounmap() */
#include <linux/ioctl.h>
#include <linux/irq.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/poll.h>
#include <linux/slab.h>		/* kmalloc() */
#include <linux/types.h>	/* size_t */

#include <asm/i2cs.h>

#include <gxio/common.h>
#include <gxio/iorpc_globals.h>
#include <gxio/iorpc_i2cs.h>
#include <gxio/kiorpc.h>

#include <hv/drv_i2cs_intf.h>


#if 0
#define I2CS_DEBUG
#endif 

#ifdef I2CS_DEBUG
#define I2CS_TRACE(...) printk(KERN_INFO "i2cs:" __VA_ARGS__)
#else
#define I2CS_TRACE(...)
#endif

/* Dynamic major by default */
#define I2CS_MAJOR 0

/* TFIFO data width (i.e. 8 byte). */
#define TFIFO_WIDTH (I2CS_TFIFO_READ_DATA__RDAT_WIDTH / 8)

/* RFIFO data width (i.e. 8 byte). */
#define RFIFO_WIDTH (I2CS_READ_DATA__RDAT_WIDTH / 8)


static char i2cs_dev_name[16] = "i2cs/0";
static char driver_name[]   = "i2cs";

int i2cs_major = I2CS_MAJOR;

struct i2cs_dev *i2cs_device;

struct i2cs_dev {
	/* GXIO device context. */
 	gxio_i2cs_context_t context;

	/* Character device structure */
	struct cdev cdev;		

	/* Open counter. */
	int open_count; 

	/* Mask returned by poll routine */
	unsigned int poll_mask;         

	/* Irq number for the I2C Slave. */
	unsigned int irq_num;	

	/* CPU that interrupt is delivered to. */
	int irq_cpu;

	/* Guarantee one accessor reader at a time */
	struct semaphore read_mutex; 
   
	/* Guarantee one accessor writer at a time */
	struct semaphore write_mutex; 

	/* Queue of processes waiting for read() buffers. */
	wait_queue_head_t read_queue;  

	/* Queue of processes waiting for write() buffers. */
	wait_queue_head_t write_queue;
};


/** i2cs_intr() - The IRQ handler routine. */
static irqreturn_t i2cs_intr(int irq, void* ptr)
{
	struct i2cs_dev *dev = (struct i2cs_dev*) ptr;
	uint64_t status;
	irqreturn_t result = IRQ_NONE;

	/* Check interrupt status. */
        status = __gxio_mmio_read(dev->context.int_vec);

	/* 1st: RFIFO_WRITE: */
	if (status & I2CS_INT_VEC_MASK__RFIFO_WRITE_MASK) {

		I2CS_TRACE("RFIFO_WRITE interrupt\n");

		/* Wake up any pending read. */
		wake_up_interruptible(&dev->read_queue);

		/* Clear the isr pending status. */
		__gxio_mmio_write(dev->context.int_vec, 
			I2CS_INT_VEC_MASK__RFIFO_WRITE_MASK);

		result = IRQ_HANDLED;
	}

        /* 2nd: TFIFO_READ: */
        if (status & I2CS_INT_VEC_MASK__TFIFO_READ_MASK) {

		I2CS_TRACE("TFIFO_READ interrupt\n");

                /* Wake up any pending write. */
                wake_up_interruptible(&dev->write_queue);

                /* Clear the isr pending status. */
                __gxio_mmio_write(dev->context.int_vec, 
			I2CS_INT_VEC_MASK__TFIFO_READ_MASK);

                result = IRQ_HANDLED;
        }

	return result;
}


/**
 * i2cs_open() - Device open routine.
 * @inode: Inode for this device.
 * @filp: File for this specific open of the device.
 * 
 * Returns zero, or an error code.
 */
static int i2cs_open(struct inode *inode, struct file *filp)
{
	struct i2cs_dev *dev = 
		container_of(inode->i_cdev, struct i2cs_dev, cdev);
	resource_size_t offset = HV_I2CS_MMIO_OFFSET;
	int result = -ENODEV;
	int irq;
	int cpu;

	filp->private_data = dev;

	I2CS_TRACE("Enter i2cs_open\n");

	/* Grab the lock. */
	if (down_interruptible(&dev->read_mutex)) {
		I2CS_TRACE("Exit i2cs_open -ERESTARTSYS\n");
		return -ERESTARTSYS;
	}
        if (down_interruptible(&dev->write_mutex)) {
		up(&dev->read_mutex);
                I2CS_TRACE("Exit i2cs_open -ERESTARTSYS\n");
                return -ERESTARTSYS;
        }

	/* If somebody has tried to open the hypervisor file but failed. */
	if (dev->context.fd < 0) 
		goto end;

	/* Now open the hypervisor device if we haven't already. */
	if (dev->context.fd == 0) { 
		dev->context.fd = hv_dev_open((HV_VirtAddr)i2cs_dev_name, 0);
	        /* If we tried and failed to open it, fail. */
        	if (dev->context.fd < 0) {
                	I2CS_TRACE("Failed to open HV I2CS file\n");
                	result = dev->context.fd;
			goto end;
		}

		/* 
		 * Allocate and register a common interrupt for RFIFO_WRITE
		 * and TFIFO_READ.
		 */
	        irq = create_irq();
	        if (irq < 0) 
        	        goto end;

	        tile_irq_activate(irq, TILE_IRQ_SW_CLEAR);
        	if (request_irq(irq, i2cs_intr, 0, driver_name, (void*)dev)) 
               		goto irq_failed;

		/* Choose an arbitrary cpu to handle interrupts. */
		cpu = raw_smp_processor_id();
		dev->irq_cpu = cpu;

		result = gxio_i2cs_cfg_interrupt(&dev->context,
						 cpu_x(cpu), cpu_y(cpu),
						 KERNEL_PL, irq);
		if (result)
			goto irq_failed;

		dev->irq_num = irq;

		/* Remap the I2CS mmio register space into kernel VA. */
		dev->context.mmio_base = (void __force *) 
			iorpc_ioremap(dev->context.fd, offset,
				      HV_I2CS_IOREMAP_SIZE);
		if (dev->context.mmio_base == NULL) {
			I2CS_TRACE("iorpc_ioremap() failed\n");
			hv_dev_close(dev->context.fd);
			goto irq_failed;
		}

		dev->context.rfifo_base = (void __force *) 
			(dev->context.mmio_base + I2CS_READ_DATA);
		dev->context.tfifo_base = (void __force *)
			(dev->context.mmio_base + I2CS_TFIFO_WRITE_DATA);
		dev->context.fifo_flag  = (void __force *)
			(dev->context.mmio_base + I2CS_FLAG);
		dev->context.int_vec    = (void __force *)
			(dev->context.mmio_base + I2CS_INT_VEC_W1TC);
	}	

	++dev->open_count;

	result = 0;
	goto end;

 irq_failed:
	destroy_irq(irq);
 end:
        up(&dev->write_mutex);
        up(&dev->read_mutex);

	return result;
}


/**
 * i2cs_release() - Device release routine.
 * @inode: Inode for this device.
 * @filp: File for this specific open of the device.
 *
 * Returns zero.
 */
static int i2cs_release(struct inode* inode, struct file* filp)
{
        struct i2cs_dev *dev = filp->private_data;
	int cpu = dev->irq_cpu;
        int result = 0;

        I2CS_TRACE("Enter i2cs_release\n");

        /* Grab the lock. */
        if (down_interruptible(&dev->read_mutex)) {
                I2CS_TRACE("Exit i2cs_release -ERESTARTSYS\n");
                return -ERESTARTSYS;
        }
        if (down_interruptible(&dev->write_mutex)) {
                up(&dev->read_mutex);
                I2CS_TRACE("Exit i2cs_release -ERESTARTSYS\n");
                return -ERESTARTSYS;
        }

        if (!dev->open_count) {
                result = -EFAULT;
                goto end;
        }

        /* Release kernel resources if it is the last close. */
        if (--dev->open_count == 0) {

		/* Unmap the I2CS mmio register space. */
                iounmap(dev->context.mmio_base);

		/* Disable I2CS interrupts before freeing the kernel irq. */
                result = gxio_i2cs_cfg_interrupt(&dev->context,
                                                 cpu_x(cpu), cpu_y(cpu),
                                                 KERNEL_PL, -1);
		if (result)
			goto end;
	
		/* Free irq. */
                destroy_irq(dev->irq_num);
                dev->irq_num = -1;

		/* Close the hypervisor device. */
                result = hv_dev_close(dev->context.fd);
                if (result != 0) {
                        I2CS_TRACE("hv_dev_close() failed\n");
                        goto end;
                }

                dev->context.fd = 0;
        }

 end:
        up(&dev->write_mutex);
        up(&dev->read_mutex);

        return result;
}


/**
 * i2cs_read() - Read data from the Receive FIFO which can be filled by 
 * the I2C master.
 * @filp: File for this specific open of the device.
 * @buf: User's data buffer.
 * @count: Number of bytes requested.
 * @f_pos: File position.
 *
 * Returns number of bytes read, or an error code.
 */
static ssize_t i2cs_read(struct file *filp, char __user *buf, size_t count,
		  	 loff_t *f_pos)
{
        int i;
        int slot_num;
	uint64_t data;
	size_t size;
	size_t bytes_read = 0; 
	struct i2cs_dev *dev = filp->private_data;

	I2CS_TRACE("Enter i2cs_read\n");

        if (count == 0) {
                I2CS_TRACE("Exit i2cs_read\n");
                return 0;
        }

        /* The RFIFO slot we actually need. */
        slot_num = ((count - 1) / RFIFO_WIDTH) + 1;
	
	/* Grab the lock. */
	if (down_interruptible(&dev->read_mutex)) {
		I2CS_TRACE("Exit i2cs_read -ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	for (i = 0; i < slot_num; i++) {
		/* RFIFO is empty? */
                while (__gxio_mmio_read(dev->context.fifo_flag) &
                        I2CS_FLAG__RFIFO_EMPTY_MASK) {
                        up(&dev->read_mutex);
                        if (filp->f_flags & O_NONBLOCK) {
                                I2CS_TRACE("Exit i2cs_read -EAGAIN\n");
                                return -EAGAIN;
                        }

                        /* Wait for valid slots in the RFIFO to send. */
                        I2CS_TRACE("Waiting on read_queue\n");

                        if (wait_event_interruptible(dev->read_queue,
                                !(__gxio_mmio_read(dev->context.fifo_flag) &
                                        I2CS_FLAG__RFIFO_EMPTY_MASK))) {
                                I2CS_TRACE("Exit i2cs_read -ERESTARTSYS\n");
                                return -ERESTARTSYS;
                        }
                        I2CS_TRACE("Woke from read_queue\n");

                        /* Get the read lock again. */
                        if (down_interruptible(&dev->read_mutex)) {
                                I2CS_TRACE("Exit i2cs_read -ERESTARTSYS\n");
                                return -ERESTARTSYS;
                        }
                }

		/*
		 * At this point we hold the read mutex and we know that there
		 * is at least one slot of data available in RFIFO.  Copy up to
		 * RFIFO_WIDTH bytes to userspace.
		 */
		size = min((size_t)RFIFO_WIDTH, count);

		/* Pull the data out of RFIFO. */
		data = __gxio_mmio_read(dev->context.rfifo_base);
		if (copy_to_user(buf + bytes_read, &data, size)) {
			up(&dev->read_mutex);
			return -EFAULT;
		}

		bytes_read += size;
		count -= size;
	}

	up(&dev->read_mutex);

	I2CS_TRACE("Exit i2cs_read %d\n", (int)bytes_read);
	return bytes_read;
}


/**
 * i2cs_write() - Write data to Transmit FIFO which can be popped out by 
 * the I2C master.
 * @filp: File for this specific open of the device.
 * @buf: User's data buffer.
 * @count: Number of bytes requested.
 * @f_pos: File position.
 *
 * Returns number of bytes written, or an error code.
 */
static ssize_t i2cs_write(struct file *filp, const char __user *buf, 
			  size_t count, loff_t *f_pos)
{
	int i;
	int slot_num;
	uint64_t data;
	size_t size;
	size_t bytes_written = 0;
	struct i2cs_dev *dev = filp->private_data;

	I2CS_TRACE("Enter i2cs_write\n");

	if (count == 0) {
		I2CS_TRACE("Exit i2cs_write\n");
		return 0;
	}

	/* The TFIFO slot we actually need. */
	slot_num = ((count - 1) / TFIFO_WIDTH) + 1;

	/* Grab the lock. */
	if (down_interruptible(&dev->write_mutex)) {
		I2CS_TRACE("Exit i2cs_write -ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	for (i = 0; i < slot_num; i++) {
		/* TFIFO is full? */
		while (__gxio_mmio_read(dev->context.fifo_flag) & 
			I2CS_FLAG__TFIFO_FULL_MASK) {
			up(&dev->write_mutex);
			if (filp->f_flags & O_NONBLOCK) {
				I2CS_TRACE("Exit i2cs_write -EAGAIN\n");
				return -EAGAIN;
			}

		 	/* Wait for free slots in the TFIFO to fill. */
			I2CS_TRACE("Waiting on write_queue\n");

			if (wait_event_interruptible(dev->write_queue,
				!(__gxio_mmio_read(dev->context.fifo_flag) & 
					I2CS_FLAG__TFIFO_FULL_MASK))) {
				I2CS_TRACE("Exit i2cs_write -ERESTARTSYS\n");
				return -ERESTARTSYS;
			}
			I2CS_TRACE("Woke from write_queue\n");

			/* Get the write lock again. */
			if (down_interruptible(&dev->write_mutex)) {
				I2CS_TRACE("Exit i2cs_write -ERESTARTSYS\n");
				return -ERESTARTSYS;
			}
		}

		/*
	 	 * At this point we hold the write mutex and we know that
	 	 * there is at least one slot to fill. Send up to TFIFO_WIDTH 
	 	 * bytes data into TFIFO.
	 	 */
		size = min((size_t)TFIFO_WIDTH, count);

		if (copy_from_user(&data, buf + bytes_written, size)) {
			up(&dev->write_mutex);
			return -EFAULT;
		}

                /* Push the data out to TFIFO. */
                __gxio_mmio_write(dev->context.tfifo_base, data);
	
		bytes_written += size;
		count -= size;
	}

	up(&dev->write_mutex);

	I2CS_TRACE("Exit i2cs_write %d\n", (int)bytes_written);
	return bytes_written;
}


static long i2cs_ioctl(struct file *filp,
		       unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_addr;
	resource_size_t reg_offset;
	I2CS_SLAVE_ADDRESS_t slave_addr;
	struct i2cs_dev *dev = filp->private_data;

	I2CS_TRACE("Enter i2cs_ioctl\n");

	switch (cmd) {

	case I2CS_IOC_CONFIG_SLAVE_ADDR:
		if (get_user(new_addr, p))
			return -EFAULT;
		
		/* Check the valid 7-bit slave address. */
		if (new_addr >= 0x80)
			return -EINVAL;

		/* Set up new address to bit[7:1] of I2CS_SLAVE_ADDRESS. */
		reg_offset = I2CS_SLAVE_ADDRESS;
		slave_addr.word = 
			__gxio_mmio_read(dev->context.mmio_base + reg_offset) >> 
				I2CS_SLAVE_ADDRESS__ADDR_SHIFT;
		__gxio_mmio_write(dev->context.mmio_base + reg_offset, 
			(new_addr << I2CS_SLAVE_ADDRESS__ADDR_SHIFT));
		I2CS_TRACE("I2CS_IOC_CONFIG_SLAVE_ADDR: from 0x%llx to 0x%x\n",
			   (unsigned long long)slave_addr.word, new_addr);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}


#ifdef CONFIG_COMPAT
static long i2cs_compat_ioctl(struct file *filp,
			      unsigned int a, unsigned long b)
{
	/* Sign-extend the argument so it can be used as a pointer. */
	return i2cs_ioctl(filp, a, (int)(long)b);
}
#endif


static unsigned int i2cs_poll(struct file *filp, poll_table *table)
{
	struct i2cs_dev *dev = filp->private_data;

	I2CS_TRACE("Enter i2cs_poll\n");

	/* Add wait queues to the poll table; we don't actually wait here. */
        poll_wait(filp, &dev->read_queue, table);
        poll_wait(filp, &dev->write_queue, table);

	/*
	 * Grab the semaphore so that this operation is ordered with respect
	 * to any other processes that may be reading or writing.
	 */
	if (down_interruptible(&dev->read_mutex)) {
		I2CS_TRACE("Exit i2cs_poll -ERESTARTSYS\n");
		return -ERESTARTSYS;
	}
        if (down_interruptible(&dev->write_mutex)) {
		up(&dev->read_mutex);
                I2CS_TRACE("Exit i2cs_poll -ERESTARTSYS\n");
                return -ERESTARTSYS;
        }

	if (!(__gxio_mmio_read(dev->context.fifo_flag) & 
		I2CS_FLAG__RFIFO_EMPTY_MASK))
		dev->poll_mask |= (POLLIN | POLLRDNORM); /* readable */
	if (!(__gxio_mmio_read(dev->context.fifo_flag) & 
		I2CS_FLAG__TFIFO_FULL_MASK))
		dev->poll_mask |= (POLLOUT | POLLWRNORM); /* writable */

	up(&dev->write_mutex);
	up(&dev->read_mutex);

	I2CS_TRACE("POLL returning poll_mask = 0x%x\n", dev->poll_mask);
	return dev->poll_mask;
}


/*
 * The fops
 */
struct file_operations i2cs_fops = {
	.owner =     	  THIS_MODULE,
	.open =	     	  i2cs_open,
	.release =        i2cs_release,
	.read =	     	  i2cs_read,
	.write =	  i2cs_write,
	.unlocked_ioctl = i2cs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =   i2cs_compat_ioctl,
#endif
	.poll =      	  i2cs_poll
};


/**
 * i2cs_setup_cdev() - Set up a device instance in the cdev table.
 * @dev: Per-device I2CS state.
 * @index: Device to set up.
 */
static int i2cs_setup_cdev(struct i2cs_dev *dev, int index)
{
	int err = 0;
	int devno = MKDEV(i2cs_major, index);
    
	cdev_init(&dev->cdev, &i2cs_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &i2cs_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		I2CS_TRACE("i2cs_setup_cdev: Error %d adding i2cs%d", 
			   err, index);

	return err;
}


/** i2cs_init() - Initialize the driver's module. */
static int i2cs_init(void)
{
	int result;
	dev_t dev = MKDEV(i2cs_major, 0);
	
	/* Register our major, and accept a dynamic number. */
	if (i2cs_major) { 
		result = register_chrdev_region(dev, 1, driver_name);
		if (result != 0)
			return result;
	}
	else {
		result = alloc_chrdev_region(&dev, 0, 1, driver_name);
		if (result != 0)
			return result;

		i2cs_major = MAJOR(dev);
		I2CS_TRACE("Enter i2cs_init with MAJOR #%d\n", i2cs_major);
	}

	/* Allocate the device. */
	i2cs_device = kzalloc(sizeof(struct i2cs_dev), GFP_KERNEL);
	if (!i2cs_device) {
		unregister_chrdev_region(dev, 1);
		I2CS_TRACE("Exit i2cs_init -ENOMEM\n");
		return -ENOMEM;
	}

	result = i2cs_setup_cdev(i2cs_device, 0);
	if (result != 0)
		return result;

	/*
	 * Initialize mutex and wait queue for finding out
	 * when interrupts arrive.
	 */
	sema_init(&i2cs_device->read_mutex, 1);
	sema_init(&i2cs_device->write_mutex, 1);
	init_waitqueue_head(&i2cs_device->write_queue);
	init_waitqueue_head(&i2cs_device->read_queue);
	
	return 0;
}


/** i2cs_cleanup() - Cleanup function for I2CS driver. */
static void i2cs_cleanup(void)
{
	cdev_del(&i2cs_device->cdev);
	kfree(i2cs_device);
	unregister_chrdev_region(MKDEV(i2cs_major, 0), 1);
}


module_init(i2cs_init);
module_exit(i2cs_cleanup);
