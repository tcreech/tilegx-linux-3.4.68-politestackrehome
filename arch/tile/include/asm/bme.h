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
 * Defines the kernel-to-userspace API for access to
 * the Bare Metal Environment.
 */

/**
 *
 */

#ifndef _ASM_TILE_BME_H
#define _ASM_TILE_BME_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* A unique ioctl prefix for the BME memory driver. */

#define BME_IOC_TYPE 0xB6



/* BME User Space API */


/**
 * Descriptor for user memory attributes.
 */
struct bme_user_mem_desc {
	__u64 va;		/**< Address of memory. */
	__u64 len;		/**< Length of memory in bytes. */
};

/**
 * Descriptor for physical memory attributes.
 */
struct bme_phys_mem_desc {
	__u64 pa;		/**< Physical address of memory. */
	__u64 len;		/**< Length of memory in bytes. */
	__u64 pte;		/**< Caching attributes. */
};

/**
 * Describes pages underlying user memory.
 */
struct bme_user_mem_desc_io {
	/** Describes the address range of the user memory. */
	struct bme_user_mem_desc user;
	/** Return value for number of pages underlying the user memory. */
	__u64 num_pages;
};

/**
 * Struct to lock and describe user memory.
 */
struct bme_phys_mem_desc_io {
	/** Describes the address range of the user memory. */
	struct bme_user_mem_desc user;
	/** Specifies number of pages underlying this address range. */
	__u64 num_pages;
	/** Address of buffer for return of physical characteristics of memory.
	 * Must point to num_pages * sizeof(struct bme_phys_mem_desc) bytes. */
	__u64 phys;
	/** 0 if results are not valid, non-zero if all	of the requested
	 * memory was locked. */
	__u8 results_are_valid;
};

/** Get the number of pages this range of memory covers. */
#define BME_IOC_GET_NUM_PAGES _IOWR(BME_IOC_TYPE, 0x0, \
				    struct bme_user_mem_desc_io)

/**
 * Lock the memory so it can be accessed by BME tiles.  User must provide
 * space for the number of pages included in this range.  That number may
 * be obtained by BME_IOC_GET_NUM_PAGES, above.
 */
#define BME_IOC_LOCK_MEMORY _IOWR(BME_IOC_TYPE, 0x1, \
				  struct bme_phys_mem_desc_io)

#endif

