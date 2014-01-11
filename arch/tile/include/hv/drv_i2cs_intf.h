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
 * Interface definitions for the I2CS driver.
 */

#ifndef _SYS_HV_INCLUDE_DRV_I2CS_INTF_H
#define _SYS_HV_INCLUDE_DRV_I2CS_INTF_H

#include <arch/rsh.h>
#include <arch/i2cs.h>


/** The mmap file offset (PA) of the I2CS MMIO region. */
#define HV_I2CS_MMIO_OFFSET \
  ((unsigned long long)RSH_MMIO_ADDRESS_SPACE__CHANNEL_VAL_I2CS << \
    RSH_MMIO_ADDRESS_SPACE__CHANNEL_SHIFT)

/** The maximum size of the I2CS MMIO region. */
#define HV_I2CS_MMIO_SIZE \
  (1ULL << RSH_MMIO_ADDRESS_SPACE__CHANNEL_SHIFT)

/** The maximum size of the I2CS MMIO region mapped into client. */
#define HV_I2CS_IOREMAP_SIZE \
  (1ULL << I2CS_DEV_INFO__TYPE_WIDTH)

#endif /* _SYS_HV_INCLUDE_DRV_I2CS_INTF_H */
