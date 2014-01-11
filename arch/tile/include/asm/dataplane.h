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
 * Arguments for the set_dataplane() syscall.
 *
 * A task running alone, pinned to a dataplane cpu, expects not to get
 * any interrupts from the OS while it is running.  The
 * set_dataplane() syscall can modify various aspects of the default
 * behavior.
 *
 * The syscall will return -1 with errno set to EINVAL if the task
 * is not running bound to a single, dataplane cpu.
 */

#ifndef _ASM_TILE_DATAPLANE_H
#define _ASM_TILE_DATAPLANE_H

#ifdef __KERNEL__
#include <linux/percpu.h>
#endif

/*
 * Quiesce the timer interrupt before returning to user space after a
 * system call.  Normally if a task on a dataplane core makes a
 * syscall, the system will run one or more timer ticks after the
 * syscall has completed, causing unexpected interrupts in userspace.
 * Setting DP_QUIESCE avoids that problem by having the kernel "hold"
 * the task in kernel mode until the timer ticks are complete.  This
 * will make syscalls dramatically slower.
 *
 * If multiple dataplane tasks are scheduled on a single core, this
 * in effect silently disables DP_QUIESCE, which allows the tasks to make
 * progress, but without actually disabling the timer tick.
 */
#define DP_QUIESCE	0x1

/*
 * Disallow the application from entering the kernel in any way,
 * unless it calls set_dataplane() again without this bit set.
 * Issuing any other syscall or causing a page fault would generate a
 * kernel message, and "kill -9" the process.
 *
 * Setting this flag automatically sets DP_QUIESCE as well.
 */
#define DP_STRICT	0x2

/*
 * Debug dataplane interrupts, so that if any interrupt source
 * attempts to involve a dataplane cpu, a kernel message and stack
 * backtrace will be generated on the console.  As this warning is a
 * slow event, it may make sense to avoid this mode in production code
 * to avoid making any possible interrupts even more heavyweight.
 *
 * Setting this flag automatically sets DP_QUIESCE as well.
 */
#define DP_DEBUG	0x4

#ifdef __KERNEL__

#ifdef CONFIG_DATAPLANE

/* The flags were just updated, so some interrupts are still expected. */
#define DP_PENDING	0x100

int is_in_nohz(int cpu);
int single_process_wants_tick(void);
struct pt_regs;
int dataplane_return_to_user(struct pt_regs *);
void rcu_user_enter(void);
void rcu_user_exit(void);

/* Dataplane flag state for the current cpu. */
DECLARE_PER_CPU(int, dp_flags);

/* Return whether the specified cpu is in dataplane "debug" mode. */
static inline int dataplane_debug(int cpu)
{
	return (per_cpu(dp_flags, cpu) & (DP_DEBUG|DP_PENDING)) == DP_DEBUG;
}

#else

static inline int dataplane_debug(int cpu) { return 0; }

#endif /* DATAPLANE */

#endif /* KERNEL */

#endif /* _ASM_TILE_DATAPLANE_H */
