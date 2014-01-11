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
 * Interface definitions for the USB device driver.
 */

#ifndef _SYS_HV_DRV_USB_DEV_INTF_H
#define _SYS_HV_DRV_USB_DEV_INTF_H

#include <arch/usb_device.h>



/** Offset for the MMIO region; we export the MAC registers only. */
#define HV_USB_DEV_MMIO_OFFSET  0x10000

/** Size of the register MMIO region. */
#define HV_USB_DEV_MMIO_SIZE    ((uint64_t) 0x10000)

/** The number of service domains supported by the USB device shim. */
#define HV_USB_DEV_NUM_SVC_DOM 1

/** The number of logical endpoints supported. */
#define HV_USB_DEV_NUM_EP      6



#endif /* _SYS_HV_DRV_USB_DEV_INTF_H */
