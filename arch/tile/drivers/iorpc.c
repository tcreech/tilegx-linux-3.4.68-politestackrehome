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
 * A device driver used to track resource allocation calls between
 * user space and the hypervisor.  This interface is used by user
 * applications that require direct access to the Tile processor's
 * onboard I/O devices.  The iorpc driver is responsible for
 * tracking memory resource mapping and handling VA/PA translation.
 *
 * NOTE: it may be worth engaging with the community to put this
 * driver in "drivers/uio" when we push it up.	We would need to
 * propose a model for the actual RPC, either some kind of override
 * for the current uio irq-specific read/write methods, or maybe an
 * ioctl based approach for driver implementation details like RPC.
 */

#include <hv/iorpc.h>

#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/hugetlb.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pagemap.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include <asm/homecache.h>
#include <asm/setup.h>

#include <gxio/iorpc_globals.h>

MODULE_AUTHOR("Tilera Corporation");
MODULE_LICENSE("GPL");

#define DRIVER_NAME_STRING "iorpc"

#define STRINGIFY(x) #x
#define TOSTRING(x)	 STRINGIFY(x)
#define SIMPLE_MSG_LINE    DRIVER_NAME_STRING "(" TOSTRING(__LINE__) "): "

#define INFO(FMT, ...) \
	pr_info(SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)
#define WARNING(FMT, ...) \
	pr_warning(SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)

#ifdef DEBUG_IORPC
#define TRACE(FMT, ...) pr_info(SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)
#else
#define TRACE(...)
#endif

/* Per-file data for iorpc file descriptors. */

struct iorpc_instance {
	/* Serialize access to this structure. */
	struct mutex mutex;

	/* The HV file number. */
	int fd;

	/* A list of pages currently registered with this instance. */
	struct list_head page_list;

	/* A list of pollfd's currently associated with this instance. */
	struct list_head pollfd_list;

	/* Base PTE for MMIO mappings. */
	HV_PTE mmio_base;
};

struct iorpc_page_list_entry {
	struct list_head list;
	struct page *page;
};

/* Per-file data for pollable file descriptors. */

struct iorpc_pollfd_instance {
	/* Serialize access to this structure. */
	spinlock_t lock;

	/* The cookie we got back from the configure operation. */
	int cookie;

	/* IRQ for readable interrupts. */
	int irq;

	/* Are we currently readable? */
	u8 is_readable;

	/* Waiters for readability. */
	wait_queue_head_t readable_wq;

	/* Recipients of signals. */
	struct fasync_struct *async_queue;

	/* Links pollfd's on the same base fd together.  Protected by
	 * pollfd_list_mutex. */
	struct list_head list;

	/* Pointer to our base fd. */
	struct iorpc_instance *base_instance;
};

/*
 * This mutex protects the lists of pollfds hung off of each base iorpc fd.
 * Ideally we would protect this with each base iorpc fd's mutex, but we
 * have to handle the case where a pollfd is being released while the base
 * fd is being released.  If we tried to use the base fd's mutex, we could
 * have the following sequence of events:
 *
 * 1. iorpc_pollfd_release reads pollfd_instance->base_instance.
 *
 * 2. iorpc_release runs, walks the list of pollfd's, deconfigures and
 *    unlinks them from the base fd's list, frees base fd iorpc_instance.
 *
 * 3. iorpc_pollfd_release goes to take the base fd mutex, using the pointer
 *    it got in step 1, but the base fd iorpc_instance it points to has
 *    already been freed.
 *
 * You could have the pollfd path take a mutex on the pollfd instance
 * first, and have the base fd path take that same mutex while
 * deconfiguring each individual pollfd during the list walk, but then
 * you'd have different lock acquisition orders and a potential deadlock.
 *
 * We don't think that configuring and closing pollfds happens often enough
 * to make using one static mutex a performance problem.
 *
 * Note that while this mutex prevents the iorpc base fd from vanishing
 * while the pollfd is being released, we do not take it in iorpc_write()
 * where we both read and write the pointer from the pollfd to the base fd.
 * This is OK because in that routine, we have (or our caller has) done
 * fget() on both fd's, so we know that neither release routine can run
 * simultaneously, and we hold the base fd's mutex, so we know another
 * instance of iorpc_write() can't run simultaneously.
 */
static DEFINE_MUTEX(pollfd_list_mutex);

/* Hypervisor device names corresponding to each minor number.
 * Devices not listed here are not accessible from user space.
 */
static const char *hv_dev_names[] = {
	"test/0/iorpc",
	"mpipe/0/iorpc",
	"mpipe/1/iorpc",
	"mpipe/0/iorpc_info",
	"mpipe/1/iorpc_info",
	"crypto/0/iorpc",
	"crypto/1/iorpc",
	"trio/0/iorpc",
	"comp/0/iorpc",
	"comp/1/iorpc",
	"gpio/0/iorpc",
	NULL,	 /* NULL means this is the minor for /dev/iorpc/pollfd */
	"uart/0/iorpc",
	"uart/1/iorpc",
	"usb_dev/0/iorpc",
	"trio/1/iorpc",
	"crypto/0/pka/iorpc",
	"crypto/1/pka/iorpc",
};

#define NUM_IORPC_DEVS ARRAY_SIZE(hv_dev_names)


static struct cdev iorpc_cdev;
static dev_t iorpc_dev;

static const char driver_name[] = DRIVER_NAME_STRING;

#define MAX_RPC_STACK_BYTES 1024


/* Forward reference so we can use in iorpc_pollfd_open(). */
static const struct file_operations iorpc_pollfd_fops;

/*
 * iorpc_pollfd_open() - Open a new pollable fd.
 */
static int iorpc_pollfd_open(struct inode *inode, struct file *filp)
{
	struct iorpc_pollfd_instance *pollfd_instance;

	TRACE("iorpc_pollfd_open()\n");

	pollfd_instance = kmalloc(sizeof(*pollfd_instance), GFP_KERNEL);
	if (pollfd_instance == NULL) {
		TRACE("kmalloc() failure.\n");
		return -ENOMEM;
	}

	spin_lock_init(&pollfd_instance->lock);
	init_waitqueue_head(&pollfd_instance->readable_wq);
	pollfd_instance->async_queue = NULL;
	pollfd_instance->is_readable = 0;
	pollfd_instance->base_instance = NULL;

	filp->f_op = &iorpc_pollfd_fops;
	filp->private_data = pollfd_instance;

	return 0;
}


/*
 * Clean up a pollable fd; used when it's released, or when the base fd it
 * refers to is released.  The caller must already hold pollfd_list_mutex.
 */
static void reset_pollfd(struct iorpc_pollfd_instance *pollfd_instance)
{
	if (pollfd_instance->base_instance) {
		/*
		 * Cancel any hypervisor or hardware interrupts for this
		 * pollable fd.
		 */
		__iorpc_close_pollfd(pollfd_instance->base_instance->fd,
				     pollfd_instance->cookie);

		/* Delete ourselves from the base fd's list. */
		list_del(&pollfd_instance->list);
		pollfd_instance->base_instance = NULL;

		/* Free our IRQ. */
		free_irq(pollfd_instance->irq, pollfd_instance);
		destroy_irq(pollfd_instance->irq);
	}
}


/*
 * iorpc_pollfd_release() - Release a pollable fd.
 */
static int iorpc_pollfd_release(struct inode *inode, struct file *filp)
{
	struct iorpc_pollfd_instance *pollfd_instance = filp->private_data;

	TRACE("iorpc_pollfd_release()\n");

	mutex_lock(&pollfd_list_mutex);

	reset_pollfd(pollfd_instance);

	mutex_unlock(&pollfd_list_mutex);

	kfree(pollfd_instance);
	filp->private_data = NULL;

	return 0;
}


static unsigned int iorpc_pollfd_poll(struct file *filp, poll_table *wait)
{
	struct iorpc_pollfd_instance *pollfd_instance = filp->private_data;
	unsigned int retval = 0;
	unsigned long flags;

	spin_lock_irqsave(&pollfd_instance->lock, flags);

	poll_wait(filp, &pollfd_instance->readable_wq, wait);

	if (pollfd_instance->is_readable)
		retval |= POLLIN | POLLRDNORM;

	spin_unlock_irqrestore(&pollfd_instance->lock, flags);

	return retval;
}


static int iorpc_pollfd_fasync(int fd, struct file *file, int on)
{
	struct iorpc_pollfd_instance *pollfd_instance = file->private_data;

	return fasync_helper(fd, file, on, &pollfd_instance->async_queue);
}


static const struct file_operations iorpc_pollfd_fops = {
	.owner = THIS_MODULE,
	.open = iorpc_pollfd_open,
	.release = iorpc_pollfd_release,
	.poll = iorpc_pollfd_poll,
	.fasync = iorpc_pollfd_fasync,
};


/** Handle a readable interrupt on the current CPU. */
static irqreturn_t iorpc_pollfd_handle_irq(int irq, void *dev_id)
{
	struct iorpc_pollfd_instance *pollfd_instance = dev_id;
	unsigned long flags;

	TRACE("got readable intr");

	spin_lock_irqsave(&pollfd_instance->lock, flags);
	if (!pollfd_instance->is_readable) {
		pollfd_instance->is_readable = 1;
		wake_up_interruptible(&pollfd_instance->readable_wq);
		kill_fasync(&pollfd_instance->async_queue, SIGIO, POLL_IN);
	}
	spin_unlock_irqrestore(&pollfd_instance->lock, flags);

	return IRQ_HANDLED;
}



/*
 * iorpc_open() - Open a new iorpc context.
 *
 * Each invocation of open() creates a new resource allocation context
 * for the device specified by the cdev minor number.
 */
static int iorpc_open(struct inode *inode, struct file *filp)
{
	unsigned int minor = MINOR(inode->i_rdev);
	int result = 0;
	struct iorpc_instance *instance;
	int fd;

	TRACE("iorpc_open()\n");

	if (minor >= NUM_IORPC_DEVS)
		return -ENODEV;

	if (!hv_dev_names[minor])
		return iorpc_pollfd_open(inode, filp);

	fd = hv_dev_open((HV_VirtAddr)hv_dev_names[minor], 0);
	if (fd < 0) {
		TRACE("hv_dev_open() failed.\n");
		if (fd >= GXIO_ERR_MIN && fd <= GXIO_ERR_MAX)
			return fd;
		else
			return -ENODEV;
	}

	instance = kmalloc(sizeof(*instance), GFP_KERNEL);
	if (instance == NULL) {
		TRACE("kmalloc() failure.\n");
		result = -ENOMEM;
		goto kmalloc_failed;
	}

	if (__iorpc_get_mmio_base(fd, &instance->mmio_base)) {
		TRACE("get_mmio_base() failure.\n");
		result = -EIO;
		goto mmio_failed;
	}

	mutex_init(&instance->mutex);
	instance->fd = fd;
	INIT_LIST_HEAD(&instance->page_list);
	INIT_LIST_HEAD(&instance->pollfd_list);

	filp->private_data = instance;

	return result;

 mmio_failed:
	kfree(instance);

 kmalloc_failed:
	hv_dev_close(fd);
	return result;
}

/*
 * iorpc_release() - Release a iorpc instance.
 *
 * We close the hypervisor file handle, which in turn should trigger
 * any per-device reset and cleanup code that the hypervisor driver
 * requires.
 */
static int iorpc_release(struct inode *inode, struct file *filp)
{
	struct iorpc_instance *instance = filp->private_data;
	struct iorpc_page_list_entry *page_list_ptr;
	struct iorpc_page_list_entry *page_list_next;
	struct iorpc_pollfd_instance *pollfd_ptr;
	struct iorpc_pollfd_instance *pollfd_next;

	TRACE("iorpc_release()\n");

	/* Unconfigure any pollable fds using this as their base. */
	mutex_lock(&pollfd_list_mutex);

	list_for_each_entry_safe(pollfd_ptr, pollfd_next,
				 &instance->pollfd_list, list) {
		reset_pollfd(pollfd_ptr);
	}

	mutex_unlock(&pollfd_list_mutex);

	/* Closing the HV device resets the device. */
	hv_dev_close(instance->fd);

	/* Release all pages that the device used to reference. */
	list_for_each_entry_safe(page_list_ptr, page_list_next,
				 &instance->page_list, list) {
		TRACE("Release pfn %#lx\n", page_to_pfn(page_list_ptr->page));
		page_cache_release(page_list_ptr->page);
		kfree(page_list_ptr);
	}

	kfree(instance);
	filp->private_data = NULL;

	return 0;
}

/* iorpc_read() - Read RPC data from the hypervisor. */
static ssize_t iorpc_read(struct file *filp, char __user *buf,
			     size_t count, loff_t *f_pos)
{
	struct iorpc_instance *instance = filp->private_data;
	union iorpc_offset off = {.offset = *f_pos};
	char stack_tmp[MAX_RPC_STACK_BYTES];
	char *heap_tmp = NULL;
	char *tmp;
	ssize_t result = 0;

	TRACE("iorpc_read()\n");

	if (count > sizeof(stack_tmp)) {
		heap_tmp = kmalloc(count, GFP_KERNEL);
		if (heap_tmp == NULL)
			return -ENOMEM;
		tmp = heap_tmp;
	} else {
		tmp = stack_tmp;
	}

	/*
	 * If this operation's format requires translation, or it's a
	 * kernel-only operation, don't allow it.
	 */
	if (off.format != IORPC_FORMAT_NONE)
		return GXIO_ERR_OPCODE;

	/* Avoid leaks to userspace. */
	memset(tmp, 0, count);

	result = hv_dev_pread(instance->fd, 0,
			      (HV_VirtAddr)tmp, count, off.offset);
	TRACE("hv_dev_pread() returns %zd\n", result);

	/*
	 * hv_dev_pread should never have side effects; if it did we'd
	 * run into trouble because the copy_to_user() might fail
	 * after the HV call succeeded.
	 */
	if (copy_to_user(buf, tmp, count)) {
		TRACE("EFAULT when copying hv_dev_pread() result.\n");
		result = -EFAULT;
	}

	kfree(heap_tmp);

	return result;
}


/*
 * Helper routine for translating and verifying pages that a user
 * application wishes to register with a hypervisor iorpc
 * device.
 */
static int translate_mem_buffer(union iorpc_mem_buffer *params,
				struct iorpc_page_list_entry **entry_out)
{
	union iorpc_mem_buffer translated;
	int count;
	struct vm_area_struct *vma;
	struct iorpc_page_list_entry *entry;
	unsigned long va;
	size_t page_size;
	unsigned long next_page_va;
	pte_t pte = { 0 };
	HV_PhysAddr pfn;
	int result;

	memset(&translated, 0, sizeof(translated));
	*entry_out = NULL;

	entry = kmalloc(sizeof(*entry), GFP_KERNEL);
	if (entry == NULL)
		return -ENOMEM;

	/*
	 * We only allow registration of a single page; getting
	 * multiple pages to be physically contiguous is difficult
	 * anyway.
	 */
	va = (unsigned long)params->user.va;
	down_read(&current->mm->mmap_sem);
	count = get_user_pages(current, current->mm, va, 1 /* num_pages */,
			       1 /* writable */, 0, &entry->page, &vma);
	up_read(&current->mm->mmap_sem);
	if (count != 1) {
		TRACE("get_user_pages() failure.\n");
		result = -EFAULT;
		goto get_user_pages_failed;
	}

	page_size = vma_kernel_pagesize(vma);
	next_page_va = (va & ~(page_size - 1)) + page_size;
	if ((va + params->user.size - 1) < va ||
	    (va + params->user.size - 1) >= next_page_va) {
		TRACE("User buffer crossed page boundary.\n");
		result = -EINVAL; /* special error for 'spanned pages'? */
		goto va_overflow;
	}

	/* Figure out how the page is homed. */
	pte = pte_set_home(pte, page_home(entry->page));

	/* Fill in the translation and replace the old parameters. */
	pfn = page_to_pfn(entry->page);
	translated.kernel.cpa = PFN_PHYS(pfn) | (va & ~PAGE_MASK);
	translated.kernel.size = params->user.size;
	translated.kernel.pte = pte;
	translated.kernel.flags = params->user.flags;
	*params = translated;

	*entry_out = entry;
	return 0;

 va_overflow:
	page_cache_release(entry->page);
 get_user_pages_failed:
	kfree(entry);
	return result;
}


/* iorpc_write() - Write RPC data to the hypervisor. */
static ssize_t iorpc_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *f_pos)
{
	struct iorpc_instance *instance = filp->private_data;
	union iorpc_offset off = {.offset = *f_pos};
	char stack_tmp[MAX_RPC_STACK_BYTES];
	char *heap_tmp = NULL;
	char *tmp;
	ssize_t result = 0;

	TRACE("iorpc_write()\n");

	if (count > sizeof(stack_tmp)) {
		heap_tmp = kmalloc(count, GFP_KERNEL);
		if (heap_tmp == NULL)
			return -ENOMEM;
		tmp = heap_tmp;
	} else {
		tmp = stack_tmp;
	}

	mutex_lock(&instance->mutex);

	if (copy_from_user(tmp, buf, count)) {
		result = -EFAULT;
		goto end;
	}

	/*
	 * Perform any request-specific translation or resource
	 * allocation, as indicated by the opcode's 'format' field.
	 */
	switch (off.format) {
	case IORPC_FORMAT_NONE: {
		/* No special translation required, just do the op. */
		result = hv_dev_pwrite(instance->fd, 0,
				       (HV_VirtAddr)tmp, count, off.offset);
		break;
	}

	case IORPC_FORMAT_USER_INTERRUPT: {
#ifdef CONFIG_HARDWALL
		union iorpc_interrupt translated;
		union iorpc_interrupt *params = (union iorpc_interrupt *)tmp;
		if (!hardwall_ipi_valid(params->user.cpu)) {
			result = -EPERM;
			goto end;
		}
		/* Translate format. */

		translated.kernel.x = cpu_x(params->user.cpu);
		translated.kernel.y = cpu_y(params->user.cpu);
		translated.kernel.ipi = 0;
		translated.kernel.event = params->user.event;
		*params = translated;

		/* Mark format change and pass to the HV. */
		off.format = IORPC_FORMAT_KERNEL_INTERRUPT;
		result = hv_dev_pwrite(instance->fd, 0,
				       (HV_VirtAddr)tmp, count, off.offset);
		break;
#else
		result = -EPERM;
		goto end;
#endif
	}

	case IORPC_FORMAT_USER_MEM: {
		/* Translate VA to PA and get homing information. */
		struct iorpc_page_list_entry *entry;
		union iorpc_mem_buffer *params = (union iorpc_mem_buffer *)tmp;
		if (count < sizeof(*params)) {
			result = -EINVAL;
			break;
		}

		result = translate_mem_buffer(params, &entry);
		if (result != 0) {
			TRACE("translate_mem_buffer() failed, return %zd\n",
			      result);
			break;
		}

		/* Mark format change and pass to the HV. */
		off.format = IORPC_FORMAT_KERNEL_MEM;
		result = hv_dev_pwrite(instance->fd, 0,
				       (HV_VirtAddr)tmp, count, off.offset);
		if (result < 0) {
			TRACE("hv_dev_pwrite() failed, canceling page.\n");
			page_cache_release(entry->page);
			kfree(entry);
		} else {
			/* Keep the page to be released later. */
			TRACE("Added pfn %#lx to page_list.\n",
			      page_to_pfn(entry->page));
			list_add_tail(&entry->list, &instance->page_list);
			if (!PageReserved(entry->page))
				SetPageDirty(entry->page);
		}

		break;
	}

	case IORPC_FORMAT_USER_POLLFD_SETUP: {
		union iorpc_pollfd_setup translated;
		union iorpc_pollfd_setup *params =
			(union iorpc_pollfd_setup *)tmp;
		struct iorpc_pollfd_instance *pollfd_instance;
		int cpu;

		/* Find pollable fd. */

		int fput_needed;
		struct file *file = fget_light(params->user.fd, &fput_needed);

		/* If the fd isn't open, error. */
		if (file == NULL) {
			result = -EBADF;
			goto end;
		}

		/* If the fd isn't a pollfd file, error. */
		if (file->f_op != &iorpc_pollfd_fops) {
			fput_light(file, fput_needed);
			result = -ENOENT;
			goto end;
		}

		pollfd_instance = file->private_data;

		/* If the fd has already been set up, error. */
		if (pollfd_instance->base_instance) {
			fput_light(file, fput_needed);
			result = -EBUSY;
			goto end;
		}

		/* Create our IRQ and register it. */
		pollfd_instance->irq = create_irq();
		if (pollfd_instance->irq < 0) {
			fput_light(file, fput_needed);
			result = -ENXIO;
			goto end;
		}

		tile_irq_activate(pollfd_instance->irq, TILE_IRQ_PERCPU);

		if (request_irq(pollfd_instance->irq, iorpc_pollfd_handle_irq,
				0, "iorpc_pollfd", pollfd_instance)) {
			fput_light(file, fput_needed);
			result = -ENXIO;
			goto end;
		}

		/* Choose an arbitrary cpu to handle device interrupts. */
		cpu = raw_smp_processor_id();

		/* Translate format. */
		translated.kernel.x = cpu_x(cpu);
		translated.kernel.y = cpu_y(cpu);
		translated.kernel.ipi = KERNEL_PL;
		translated.kernel.event = pollfd_instance->irq;
		*params = translated;

		/* Mark format change and pass to the HV. */
		off.format = IORPC_FORMAT_KERNEL_POLLFD_SETUP;
		result = hv_dev_pwrite(instance->fd, 0,
				       (HV_VirtAddr)tmp, count, off.offset);

		/* Save cookie, convert return code. */
		if (result >= 0) {
			pollfd_instance->base_instance = instance;
			list_add_tail(&pollfd_instance->list,
				      &instance->pollfd_list);
			pollfd_instance->cookie = result;
			result = 0;
		} else {
			free_irq(pollfd_instance->irq, pollfd_instance);
			destroy_irq(pollfd_instance->irq);
		}

		fput_light(file, fput_needed);
		break;
	}

	case IORPC_FORMAT_USER_POLLFD: {
		union iorpc_pollfd translated;
		union iorpc_pollfd *params = (union iorpc_pollfd *)tmp;
		struct iorpc_pollfd_instance *pollfd_instance;

		/*
		 * Find pollable fd, make sure it's configured with this
		 * base fd, and get the cookie.
		 */
		int fput_needed;
		struct file *file = fget_light(params->user.fd, &fput_needed);

		/* If the fd isn't open, error. */
		if (file == NULL) {
			result = -EBADF;
			goto end;
		}

		/* If the fd isn't a pollfd file, error. */
		if (file->f_op != &iorpc_pollfd_fops) {
			fput_light(file, fput_needed);
			result = -ENOENT;
			goto end;
		}

		pollfd_instance = file->private_data;

		/* If the pollfd isn't associated with this base fd, error. */
		if (instance != pollfd_instance->base_instance) {
			fput_light(file, fput_needed);
			result = -EBUSY;
			goto end;
		}

		/*
		 * Reset the readable flag, on the theory that this must be
		 * an arm() call.  This is slightly sketchy, but is true
		 * for now; if we ever had other calls which needed to
		 * convert a pollfd to a cookie on the way upstream, we
		 * might want a specialized POLLFD_ARM type.
		 */
		pollfd_instance->is_readable = 0;

		/* Translate format. */
		translated.kernel.cookie = pollfd_instance->cookie;
		*params = translated;

		/* Mark format change and pass to the HV. */
		off.format = IORPC_FORMAT_KERNEL_POLLFD;
		result = hv_dev_pwrite(instance->fd, 0,
				       (HV_VirtAddr)tmp, count, off.offset);

		fput_light(file, fput_needed);
		break;
	}

	case IORPC_FORMAT_NONE_NOUSER:
		/*
		 * User programs are not permitted to issue operations with
		 * this format.  (Yes, we could just lump this in with the
		 * default, but it seems better to make it explicit.)
		 */
		result = GXIO_ERR_OPCODE;
		break;

	default:
		/* We handled all legal formats above, must be bad opcode. */
		result = GXIO_ERR_OPCODE;
		break;
	}

end:
	mutex_unlock(&instance->mutex);
	kfree(heap_tmp);

	TRACE("iorpc_write() returns %zd\n", result);
	return result;
}

static int iorpc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct iorpc_instance *instance = file->private_data;
	size_t size = vma->vm_end - vma->vm_start;
	unsigned long offset = PFN_PHYS(vma->vm_pgoff);
	int err;
	pgprot_t prot = vma->vm_page_prot;
	unsigned long pfn;

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	/* Make sure the requested offset is allowed by the HV driver. */
	err = __iorpc_check_mmio_offset(instance->fd, offset, size);
	if (err)
		return err;

	/* Merge the base HV_PTE with our pgprot and pfn offset. */
	prot = hv_pte_set_mode(prot, HV_PTE_MODE_MMIO);
	prot = hv_pte_set_lotar(prot, hv_pte_get_lotar(instance->mmio_base));
	pfn = pte_pfn(instance->mmio_base) + vma->vm_pgoff;

	vma->vm_flags |= VM_LOCKED | VM_RESERVED;

	err = remap_pfn_range(vma, vma->vm_start, pfn, size, prot);
	if (err) {
		TRACE("remap_pfn_range() failed.\n");
		return err;
	}

	return 0;
}


static const struct file_operations iorpc_fops = {
	.owner = THIS_MODULE,
	.open = iorpc_open,
	.release = iorpc_release,
	.read = iorpc_read,
	.write = iorpc_write,
	.mmap = iorpc_mmap,
};

/*
 * iorpc_init() - Initialize the iorpc driver.
 *
 * Returns zero on success, or a negative error code.
 */
static int iorpc_init(void)
{
	/* Allocate some major/minor numbers. */
	int err = alloc_chrdev_region(&iorpc_dev, 0, NUM_IORPC_DEVS,
				      driver_name);
	if (err != 0) {
		WARNING("Could not allocate iorpc major number.\n");
		return err;
	}

	/* Register the device. */
	cdev_init(&iorpc_cdev, &iorpc_fops);
	iorpc_cdev.owner = THIS_MODULE;
	err = cdev_add(&iorpc_cdev, iorpc_dev, NUM_IORPC_DEVS);
	if (err != 0) {
		WARNING("Failed to add iorpc cdev.\n");
		unregister_chrdev_region(iorpc_dev, NUM_IORPC_DEVS);
		return err;
	}

	return 0;
}


/** iorpc_cleanup() - Clean up the driver's module. */
static void iorpc_cleanup(void)
{
	cdev_del(&iorpc_cdev);
	unregister_chrdev_region(iorpc_dev, NUM_IORPC_DEVS);
}


module_init(iorpc_init);
module_exit(iorpc_cleanup);

#ifdef CONFIG_SYSFS
static ssize_t sysfs_comp_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int fd = hv_dev_open((HV_VirtAddr)"comp/0/reset", 0);
	if (fd < 0)
		TRACE("hv_dev_open() failed.\n");
	else
		hv_dev_close(fd);

	return count;
}

static DEVICE_ATTR(reset, 0200, NULL, sysfs_comp_reset);

static struct bus_type shim_bus = {
	.name = "comp",
	.dev_name = "comp",
};

static struct device device_shim = {
	.id	= 0,
	.bus	= &shim_bus,
};

static int __init init_iorpc_sysfs(void)
{
	int error = subsys_system_register(&shim_bus, NULL);

	if (!error)
		error = device_register(&device_shim);
	if (!error)
		error = device_create_file(
				&device_shim,
				&dev_attr_reset);
	return error;
}

device_initcall(init_iorpc_sysfs);
#endif /* CONFIG_SYSFS */
