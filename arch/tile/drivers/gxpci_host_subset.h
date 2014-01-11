/*
 * gxpci_host_subset.h
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
#ifndef __GXPCI_HOST_SUBSET_H__
#define __GXPCI_HOST_SUBSET_H__

#if !defined(FALSE)
#define FALSE	(0)
#endif /* !defined(FALSE) */

#if !defined(TRUE)
#define TRUE	(1)
#endif /* !defined(TRUE) */


#define DRIVER_NAME_STRING "gxpci_host_subset"

#define STRINGIFY(x) #x
#define TOSTRING(x)	 STRINGIFY(x)
#define MSG_LINE     "%s(%s:" TOSTRING(__LINE__) "-%d):"
#define SIMPLE_MSG_LINE    DRIVER_NAME_STRING "(" TOSTRING(__LINE__) "): "

#define INFO(FMT, ...)	pr_info(SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)
#define WARNING(FMT, ...) \
	pr_warning(SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)
#define ERR(FMT, ...)	pr_err(SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)

#ifdef TRACE_TILEPCI_HOST_SUBSET
#define TRACE(FMT, ...) pr_info(SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)
#else
#define TRACE(...)
#endif

#include "gxpci_host_shared.h"

/* Compute the minimum of two values. */
#define MIN(A,B) 				(((A) < (B)) ? (A) : (B))

/* Compute the maximum of two values. */
#define MAX(A,B) 				(((A) > (B)) ? (A) : (B))

/*
 * Right now, 2 NICs are supported: one for rootfs and one for the bridge,
 * on the TILExtreme-Gx Duo platform.
 */
#define GXPCI_HOST_SUBSET_NIC_COUNT		2

/* Tx/Rx queues for a single PCIe NIC interface. */
#define GXPCI_HOST_SUBSET_NIC_TX_QUEUES		1
#define GXPCI_HOST_SUBSET_NIC_RX_QUEUES		1

#endif
