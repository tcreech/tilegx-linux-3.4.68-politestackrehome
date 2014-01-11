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

#ifndef _ASM_TILE_KVM_PARA_H
#define _ASM_TILE_KVM_PARA_H

#ifdef __KERNEL__

static inline unsigned int kvm_arch_para_features(void)
{
	return 0;
}

int hcall_virtio(unsigned long instrument, unsigned long mem);

#endif

#endif /* _ASM_TILE_KVM_PARA_H */
