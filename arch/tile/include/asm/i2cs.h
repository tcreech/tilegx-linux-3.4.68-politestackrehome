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

/**
 *
 * Defines the kernel-to-userspace API for I2C Slave.
 */
#ifndef _ASM_TILE_I2CS_H
#define _ASM_TILE_I2CS_H

#ifndef __KERNEL__
#include <stdint.h>
#include <stddef.h>
#else
#include <linux/types.h>
#endif

#include <linux/ioctl.h>

/* A unique ioctl prefix for the I2C Slave subsystem. */


#define I2CS_IOC_TYPE 0xD5



/** Configure the new I2C slave address. */
#define I2CS_IOC_CONFIG_SLAVE_ADDR _IOW(I2CS_IOC_TYPE, 0x0, uint64_t)

#endif /* _ASM_TILE_I2CS_H */
