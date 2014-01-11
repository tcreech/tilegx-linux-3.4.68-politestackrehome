/*
 * Copyright 2010 Tilera Corporation. All Rights Reserved.
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

#ifndef _ASM_TILE_HARDIRQ_H
#define _ASM_TILE_HARDIRQ_H

#include <linux/threads.h>
#include <linux/cache.h>

#include <asm/irq.h>

typedef struct {
	unsigned int __softirq_pending;
	long idle_timestamp;

	/* Hard interrupt statistics. */
	unsigned long irq_timer_count;
	unsigned long irq_syscall_count;
	unsigned long irq_resched_count;
	unsigned long irq_hv_flush_count;
	unsigned long irq_tlb_flush_count;
	unsigned long irq_call_count;
	unsigned long irq_dev_intr_count;

} ____cacheline_aligned irq_cpustat_t;

DECLARE_PER_CPU(irq_cpustat_t, irq_stat);

#define __ARCH_IRQ_STAT
#define __IRQ_STAT(cpu, member) (per_cpu(irq_stat, cpu).member)

#include <linux/irq_cpustat.h>	/* Standard mappings for irq_cpustat_t above */

#define HARDIRQ_BITS	8


extern u64 arch_irq_stat_cpu(unsigned int cpu);
#define arch_irq_stat_cpu	arch_irq_stat_cpu


#endif /* _ASM_TILE_HARDIRQ_H */