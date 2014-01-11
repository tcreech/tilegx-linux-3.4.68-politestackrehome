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
 * Declares state used by the target-side PCIE driver.
 */
#ifndef __GXPCI_ENDP_H__
#define __GXPCI_ENDP_H__

#include <asm/tilegxpci.h>
#include "tilegxpci_shared_code.h"

/** 
 * Definitions of the Tile driver version. 
 */
#define GXPCI_MAJOR 			10
#define GXPCI_MINOR 			0

/**
 * MSI-X support definitions for the SR-IOV Physical Functions.
 *
 * The MSI-X Table structure and the MSI-X Pending Bit Array structure
 * reside in BAR0 space, following struct gxpci_host_regs, for the PF.
 */

/**
 * The MSI-X table offset is based on the offset of struct gxpci_host_regs.
 * The PCI Spec recommends that a PCI function isolate the MSI-X structures
 * from the non-MSI-X structures with aligned 8KB ranges rather than the
 * mandatory aligned 4KB ranges.
 */
#define GXPCI_MSIX_TABLE_OFFSET		\
	(GXPCI_HOST_REGS_OFFSET + (PAGE_SIZE * 2))

/**
 * We use BAR0 to map the MSI-X table.
 */
#define GXPCI_MSIX_TABLE_BIR		0

/**
 * The MSI-X Pending Bit Array is isolated from the MSI-X table with at
 * least 4KB, even though they are permitted to co-reside within a natually
 * aligned 4KB address range by the PCI Spec.
 */
#define GXPCI_MSIX_PBA_OFFSET 		(GXPCI_MSIX_TABLE_OFFSET + 4096)

/**
 * We use BAR0 to map the MSI-X Pending Bit Array.
 */
#define GXPCI_MSIX_PBA_BIR		0

/**
 * MSI-X support definitions for the VFs.
 *
 * The MSI-X Table structure and the MSI-X Pending Bit Array structure
 * reside in BAR0 space, following the MAP-MEM interrupt region.
 */

/**
 * The offset in VF BAR0 of the MSI-X Table.
 */
#define GXPCI_MSIX_TABLE_OFFSET_VF	0x2000

/**
 * We use BAR0 to map the MSI-X table on the VF.
 */
#define GXPCI_MSIX_TABLE_BIR_VF		0

/**
 * The offset in VF BAR0 of the MSI-X PBA.
 */
#define GXPCI_MSIX_PBA_OFFSET_VF	0x3000

/**
 * We use BAR0 to map the MSI-X Pending Bit Array on the VF.
 */
#define GXPCI_MSIX_PBA_BIR_VF		0

/**
 * Array of the PCIe ports configuration info obtained from the BIB.
 * This is filled out by the RC driver.
 */
extern struct pcie_trio_ports_property pcie_ports[TILEGX_NUM_TRIO];

irqreturn_t zc_stream_intr(int irq, void *ep_dev);
irqreturn_t zc_h2t_dma_intr(int irq, void *ep_dev);
irqreturn_t zc_t2h_dma_intr(int irq, void *ep_dev);

/* Update MSI-X vector for interrupt migration. */
static inline void gxpci_update_msix_vector(tilegxpci_get_msix_msg_t *msix_msg)
{
	msix_msg->msix_addr = *(unsigned long *)
		(msix_msg->msix_table_entry_offset + PCI_MSIX_ENTRY_LOWER_ADDR);
	msix_msg->msix_data = *(unsigned int *)
		(msix_msg->msix_table_entry_offset + PCI_MSIX_ENTRY_DATA);
}

extern void gxpci_endp_net_devs_init(struct tlr_pcie_dev *tlr);
extern void gxpci_endp_net_devs_init_vf(struct tlr_pcie_dev *tlr);

#endif	/* !defined(__GXPCI_ENDP_H__) */
