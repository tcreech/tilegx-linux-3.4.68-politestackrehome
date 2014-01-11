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
 * This file contains support for dataplane mode.
 */

#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/printk.h>
#include <linux/tick.h>
#include <linux/swap.h>
#include <asm/unistd.h>
#include <asm/dataplane.h>
#include <asm/homecache.h>
#include <asm/pmc.h>
#include <arch/interrupts.h>

/* Copy of dp_flags from currently-executing task. */
DEFINE_PER_CPU(int, dp_flags);

static inline void set_dp_flags(int flags)
{
	__get_cpu_var(dp_flags) = flags;
	current->thread.dp_flags = flags;
}

/* Syscall to modify dp_flags. */
SYSCALL_DEFINE1(set_dataplane, int, flags)
{
	struct task_struct *tsk = current;
	int cpu;

	/* Validate single-core affinity and valid cpu. */
	if (cpumask_weight(&tsk->cpus_allowed) != 1) {
		if (dp_flags & DP_DEBUG)
			pr_warn("%s/%d: affinity not bound to a single cpu\n",
				current->comm, current->pid);
		return -EINVAL;
	}
	cpu = smp_processor_id();   /* Bound to single core by affinity */
	if (!cpumask_test_cpu(cpu, &dataplane_map)) {
		if (dp_flags & DP_DEBUG)
			pr_warn("%s/%d: bound cpu %d not a dataplane cpu\n",
				current->comm, current->pid, cpu);
		return -EINVAL;
	}

	/* "strict" and "debug" imply "quiesce". */
	if (flags & (DP_STRICT|DP_DEBUG))
		flags |= DP_QUIESCE;

	set_dp_flags(flags | DP_PENDING);

	return 0;
}

/* Called (with irqs disabled) on return to userspace on dataplane cores. */
int dataplane_return_to_user(struct pt_regs *regs)
{
	int dp_flags = current->thread.dp_flags;
	bool pmc_in_use = false;
	int quiesce, wants_tick;

	/* Drain the pagevecs to avoid unnecessary IPI flushes later. */
	if (dp_flags & DP_QUIESCE)
		lru_add_drain();

	/*
	 * Unless we just set the flags, or multiple tasks are currently
	 * scheduled on the cpu, check for error cases.
	 */
	wants_tick = single_process_wants_tick();
	if (!(dp_flags & DP_PENDING) && wants_tick >= 0) {
		if (dp_flags & DP_STRICT) {
			pr_warn("Killed dataplane task in kernel.\n");
			dump_stack();
			local_irq_enable();
			do_group_exit(SIGKILL);
		}

		if ((dp_flags & DP_DEBUG) &&
		    !((1ULL << regs->faultnum) & SYNC_INTERRUPTS)) {
			pr_warn("Unexpected dataplane interrupt!\n");
			dump_stack();

			pr_warn("No further warnings will be reported"
				" by this task.\n");
			set_dp_flags(dp_flags & ~(DP_STRICT|DP_DEBUG));
		}
	}

	/*
	 * If the performance counters are in use by the kernel
	 * (for perf or oprofile, for example) we also continue to
	 * run the scheduler tick, since typically that's used to
	 * copy the per-cpu data out to a shared location.  We don't
	 * honor DP_STRICT or DP_DEBUG in this case since we don't
	 * want to have to require the app to be modified to get
	 * performance data, though of course it won't run "dataplane"
	 * once we're doing performance analysis on it.  Similarly
	 * we don't try to quiesce in this case.
	 */
	if (!wants_tick && pmc_used_by_kernel()) {
		pmc_in_use = true;
		wants_tick = true;
	}

	/* Try to stop or start the scheduler tick appropriately. */
	if (!wants_tick)
		tick_nohz_dataplane_enter();
	else if (is_in_nohz(smp_processor_id()))
		tick_nohz_dataplane_exit();

	/*
	 * We normally return immediately to userspace.
	 *
	 * The DP_QUIESCE flag causes us to wait until the tile timer
	 * is masked and can't deliver any more interrupts.  Otherwise
	 * we nap with interrupts enabled and wait for the next
	 * interrupt to fire, then loop back and retry.
	 *
	 * Note that if you schedule two processes that both want to
	 * run with DP_QUIESCE, neither will ever leave the kernel,
	 * and one will have to be killed manually.  Otherwise in
	 * situations where another process is in the runqueue on this
	 * cpu, this task will just wait for that other task to go
	 * idle before it returns from kernel space.
	 */
	quiesce = ((dp_flags & DP_QUIESCE) &&
		   regs->faultnum == INT_SWINT_1 &&
		   !pmc_in_use);
	if (quiesce && !arch_local_irq_is_masked(INT_LINUX_TIMER)) {
		if (current->thread.quiesce_start == 0) {
			current->thread.quiesce_start = jiffies;
			current->thread.quiesce_warned = false;
		} else if (!current->thread.quiesce_warned &&
			   (jiffies - current->thread.quiesce_start) > HZ) {
			pr_warn("%s/%d: dataplane (DP_QUIESCE) task blocked;"
				" check other task affinities.\n",
				current->comm, current->pid);
			current->thread.quiesce_warned = true;
		}
		set_current_state(TASK_INTERRUPTIBLE);
		_cpu_idle();
		set_current_state(TASK_RUNNING);
		return 1;  /* Retry returning to userspace. */
	}
	current->thread.quiesce_start = 0;

	/*
	 * Tell RCU we are going back into dataplane mode, so that it
	 * doesn't require this core for future RCU synchronization.
	 * Note that asynchronous interrupts have an additional set of
	 * rcu_irq_enter() and rcu_irq_exit(), so combined with this
	 * call and the one in intvec_NN.S we end up doubly-nested.
	 */
	rcu_user_enter();

	/* Clear the pending bit so we trigger real warnings going forward. */
	if (dp_flags & DP_PENDING)
		set_dp_flags(dp_flags & ~DP_PENDING);

	/*
	 * Claim (somewhat prematurely) that we are now back in user
	 * space, and barrier so that hv_flush_update() will see it.
	 * This will defer any further TLB flushes to this cpu, and
	 * enables us to spin until any that happened to be ongoing
	 * are finished, and no more will start.  We don't anticipate
	 * that any kernel data that this cpu needs prior to the real
	 * return to user space will be migrated from this point forward.
	 *
	 * See set_remote_flush_count() for more on the flush-waiting
	 * algorithm.
	 */
	if (quiesce) {
		int cpu = smp_processor_id();
		homecache_tlb_defer_exit();
		smp_wmb();
		while (homecache_get_remote_flush_count(cpu) != 0)
			cpu_relax();
	}

	return 0;
}
