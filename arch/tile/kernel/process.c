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

#include <linux/sched.h>
#include <linux/preempt.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kprobes.h>
#include <linux/elfcore.h>
#include <linux/delay.h>
#include <linux/tick.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/compat.h>
#include <linux/hardirq.h>
#include <linux/syscalls.h>
#include <linux/kernel.h>
#include <linux/tracehook.h>
#include <linux/signal.h>
#include <linux/kvm_host.h>
#include <asm/stack.h>
#include <asm/switch_to.h>
#include <asm/homecache.h>
#include <asm/syscalls.h>
#include <asm/traps.h>
#include <asm/setup.h>
#include <asm/uaccess.h>
#include <asm/pmc.h>
#ifdef CONFIG_HARDWALL
#include <asm/hardwall.h>
#endif
#ifdef CONFIG_DATAPLANE
#include <asm/dataplane.h>
#endif
#include <arch/chip.h>
#include <arch/abi.h>
#include <arch/sim_def.h>


/*
 * Use the (x86) "idle=poll" option to prefer low latency when leaving the
 * idle loop over low power while in the idle loop, e.g. if we have
 * one thread per core and we want to get threads out of futex waits fast.
 */
static int no_idle_nap;
static int __init idle_setup(char *str)
{
	if (!str)
		return -EINVAL;

	if (!strcmp(str, "poll")) {
		pr_info("using polling idle threads.\n");
		no_idle_nap = 1;
	} else if (!strcmp(str, "halt"))
		no_idle_nap = 0;
	else
		return -1;

	return 0;
}
early_param("idle", idle_setup);

/*
 * The idle thread. There's no useful work to be
 * done, so just try to conserve power and have a
 * low exit latency (ie sit in a loop waiting for
 * somebody to say that they'd like to reschedule)
 * Called with preemption disabled.
 */
void cpu_idle(void)
{
#ifdef CONFIG_HOMECACHE
	/*
	 * If we enter cpu_idle with our stack still set to be the
	 * initial stack, we switch to a new stack page now (and
	 * free the initial stack back to the heap).
	 * This allows the boot cpu's idle process to run with a
	 * stack that has proper homecaching.
	 */
	if (current_thread_info() == &init_thread_union.thread_info) {
		struct thread_info *ti = alloc_thread_info_node(current, -1);
		struct task_struct *p = current;
		struct page *page = virt_to_page(current_thread_info());
		*ti = *current_thread_info();
		p->stack = ti;
		p->thread.ksp = task_ksp0(p);
		BUILD_BUG_ON(THREAD_SIZE != PAGE_SIZE);
		clear_bit(PG_homecache_nomigrate, &page->flags);
		ClearPageReserved(page);
		cpu_idle_on_new_stack(current_thread_info(), p->thread.ksp,
				      next_current_ksp0(p));
		/*NOTREACHED*/
	}
#endif

	current_thread_info()->status |= TS_POLLING;

	if (no_idle_nap) {
		while (1) {
			while (!need_resched())
				cpu_relax();
			schedule();
		}
	}

	/* endless idle loop with no priority at all */
	while (1) {
		tick_nohz_idle_enter();
		rcu_idle_enter();
		while (!need_resched()) {
			if (cpu_is_offline(smp_processor_id()))
				BUG();  /* no HOTPLUG_CPU */

			local_irq_disable();
			__get_cpu_var(irq_stat).idle_timestamp = jiffies;
			current_thread_info()->status &= ~TS_POLLING;
			/*
			 * TS_POLLING-cleared state must be visible before we
			 * test NEED_RESCHED:
			 */
			smp_mb();

			if (!need_resched())
				_cpu_idle();
			else
				local_irq_enable();
			current_thread_info()->status |= TS_POLLING;
		}
		rcu_idle_exit();
		tick_nohz_idle_exit();
		schedule_preempt_disabled();
	}
}

struct thread_info *alloc_thread_info_node(struct task_struct *task, int node)
{
	struct page *page;
	gfp_t flags = GFP_KERNEL;
#ifdef CONFIG_HOMECACHE
	int home;
#endif

#ifdef CONFIG_DEBUG_STACK_USAGE
	flags |= __GFP_ZERO;
#endif

#ifdef CONFIG_HOMECACHE
	/*
	 * If !kstack_hash, just optimistically guess that the child will
	 * end up on the core we're currently running on.  If not, we'll
	 * just rehome it anyway.
	 */
	if (kstack_hash)
		home = PAGE_HOME_HASH;
	else
		home = raw_smp_processor_id();
	page = homecache_alloc_pages_node(node, flags, THREAD_SIZE_ORDER, home);
#else
	page = alloc_pages_node(node, flags, THREAD_SIZE_ORDER);
#endif
	if (!page)
		return NULL;

	return (struct thread_info *)page_address(page);
}

/*
 * Free a thread_info node, and all of its derivative
 * data structures.
 */
void free_thread_info(struct thread_info *info)
{
	struct single_step_state *step_state = info->step_state;

	if (step_state) {

		/*
		 * FIXME: we don't munmap step_state->buffer
		 * because the mm_struct for this process (info->task->mm)
		 * has already been zeroed in exit_mm().  Keeping a
		 * reference to it here seems like a bad move, so this
		 * means we can't munmap() the buffer, and therefore if we
		 * ptrace multiple threads in a process, we will slowly
		 * leak user memory.  (Note that as soon as the last
		 * thread in a process dies, we will reclaim all user
		 * memory including single-step buffers in the usual way.)
		 * We should either assign a kernel VA to this buffer
		 * somehow, or we should associate the buffer(s) with the
		 * mm itself so we can clean them up that way.
		 */
		kfree(step_state);
	}

	free_pages((unsigned long)info, THREAD_SIZE_ORDER);
}

static void save_arch_state(struct thread_struct *t);

int copy_thread(unsigned long clone_flags, unsigned long sp,
		unsigned long stack_size,
		struct task_struct *p, struct pt_regs *regs)
{
	struct pt_regs *childregs;
	unsigned long ksp;

	/*
	 * When creating a new kernel thread we pass sp as zero.
	 * Assign it to a reasonable value now that we have the stack.
	 */
	if (sp == 0 && regs->ex1 == PL_ICS_EX1(KERNEL_PL, 0))
		sp = task_ksp0(p);

	/*
	 * Do not clone step state from the parent; each thread
	 * must make its own lazily.
	 */
	task_thread_info(p)->step_state = NULL;

#ifdef __tilegx__
	/*
	 * Do not clone unalign jit fixup from the parent; each thread
	 * must allocate its own on demand.
	 */
	task_thread_info(p)->unalign_jit_base = NULL;
#endif

	/*
	 * Start new thread in ret_from_fork so it schedules properly
	 * and then return from interrupt like the parent.
	 */
	p->thread.pc = (unsigned long) ret_from_fork;

	/* Save user stack top pointer so we can ID the stack vm area later. */
	p->thread.usp0 = sp;

	/* Record the pid of the process that created this one. */
	p->thread.creator_pid = current->pid;

	/*
	 * Copy the registers onto the kernel stack so the
	 * return-from-interrupt code will reload it into registers.
	 */
	childregs = task_pt_regs(p);
	*childregs = *regs;
	childregs->regs[0] = 0;         /* return value is zero */
	childregs->sp = sp;  /* override with new user stack pointer */

	/*
	 * If CLONE_SETTLS is set, set "tp" in the new task to "r4",
	 * which is passed in as arg #5 to sys_clone().
	 */
	if (clone_flags & CLONE_SETTLS)
		childregs->tp = regs->regs[4];

	/*
	 * Copy the callee-saved registers from the passed pt_regs struct
	 * into the context-switch callee-saved registers area.
	 * This way when we start the interrupt-return sequence, the
	 * callee-save registers will be correctly in registers, which
	 * is how we assume the compiler leaves them as we start doing
	 * the normal return-from-interrupt path after calling C code.
	 * Zero out the C ABI save area to mark the top of the stack.
	 */
	ksp = (unsigned long) childregs;
	ksp -= C_ABI_SAVE_AREA_SIZE;   /* interrupt-entry save area */
	((long *)ksp)[0] = ((long *)ksp)[1] = 0;
	ksp -= CALLEE_SAVED_REGS_COUNT * sizeof(unsigned long);
	memcpy((void *)ksp, &regs->regs[CALLEE_SAVED_FIRST_REG],
	       CALLEE_SAVED_REGS_COUNT * sizeof(unsigned long));
	ksp -= C_ABI_SAVE_AREA_SIZE;   /* __switch_to() save area */
	((long *)ksp)[0] = ((long *)ksp)[1] = 0;
	p->thread.ksp = ksp;

#if CHIP_HAS_TILE_DMA()
	/*
	 * No DMA in the new thread.  We model this on the fact that
	 * fork() clears the pending signals, alarms, and aio for the child.
	 */
	memset(&p->thread.tile_dma_state, 0, sizeof(struct tile_dma_state));
	memset(&p->thread.dma_async_tlb, 0, sizeof(struct async_tlb));
#endif

	/* New thread has its miscellaneous processor state bits clear. */
	p->thread.proc_status = 0;

#ifdef CONFIG_HARDWALL
	/* New thread does not own any networks. */
	memset(&p->thread.hardwall[0], 0,
	       sizeof(struct hardwall_task) * HARDWALL_TYPES);
#endif

#ifdef CONFIG_HOMECACHE
	/* New thread gets memory without any homecache overrides. */
	p->thread.homecache_desired_home = NULL;
#endif

#ifdef CONFIG_DATAPLANE
	/* If a dataplane thread forks a child, the child is not dataplane. */
	p->thread.dp_flags = 0;
#endif

	/*
	 * Start the new thread with the current architecture state
	 * (user interrupt masks, etc.).
	 */
	save_arch_state(&p->thread);

	return 0;
}

int set_unalign_ctl(struct task_struct *tsk, unsigned int val)
{
	task_thread_info(tsk)->align_ctl = val;
	return 0;
}

int get_unalign_ctl(struct task_struct *tsk, unsigned long adr)
{
	return put_user(task_thread_info(tsk)->align_ctl,
			(unsigned int __user *)adr);
}

/* Given a kernel stack page, see if it has a valid task_struct pointer. */
static struct task_struct *try_current(unsigned long sp)
{
	struct thread_info *ti;
	struct task_struct *tsk;

	/* Fail if stack base (thread_info) is not in kernel memory. */
	ti = (struct thread_info *)(sp & -THREAD_SIZE);
	if ((unsigned long)ti < PAGE_OFFSET ||
	    (unsigned long)ti >= (unsigned long)high_memory)
		return NULL;

	/* Fail if pointed-to struct task_struct doesn't look OK. */
	tsk = ti->task;
	if ((unsigned long)tsk % __alignof__(*tsk) != 0 ||
	    (unsigned long)tsk < PAGE_OFFSET ||
	    (unsigned long)tsk >= (unsigned long)high_memory ||
	    task_thread_info(tsk) != ti)
		return NULL;

	return tsk;
}

static struct task_struct corrupt_current = { .comm = "<corrupt>" };

/*
 * Do our best to find the task_struct, given that one or both of
 * sp and the ksp0 SPR may be corrupted, or else a pointer to a dummy.
 * This can be helpful if we are just trying to emit a clean panic.
 */
struct task_struct *validate_current_from_sp(unsigned long sp)
{
	struct task_struct *tsk;

	tsk = try_current(sp);
	if (tsk == NULL) {
		unsigned long ksp0 = get_current_ksp0();
		tsk = try_current(ksp0);
		if (tsk == NULL) {
			pr_err("Corrupt 'current' (sp %#lx, ksp0 %#lx)\n",
			       sp, ksp0);
			tsk = &corrupt_current;
		} else {
			pr_err("Corrupt sp %#lx for 'current'"
			       " but valid ksp0 %#lx\n", sp, ksp0);
		}
	}
	return tsk;
}

struct task_struct *validate_current(void)
{
	return validate_current_from_sp(stack_pointer);
}

/* Take and return the pointer to the previous task, for schedule_tail(). */
struct task_struct *sim_notify_fork(struct task_struct *prev)
{
#ifndef CONFIG_KVM_GUEST   /* see notify_sim_task_change() */
	struct task_struct *tsk = current;
	__insn_mtspr(SPR_SIM_CONTROL, SIM_CONTROL_OS_FORK_PARENT |
		     (tsk->thread.creator_pid << _SIM_CONTROL_OPERATOR_BITS));
	__insn_mtspr(SPR_SIM_CONTROL, SIM_CONTROL_OS_FORK |
		     (tsk->pid << _SIM_CONTROL_OPERATOR_BITS));
#endif
	return prev;
}

int dump_task_regs(struct task_struct *tsk, elf_gregset_t *regs)
{
	struct pt_regs *ptregs = task_pt_regs(tsk);
	elf_core_copy_regs(regs, ptregs);
	return 1;
}

#if CHIP_HAS_TILE_DMA()

/* Allow user processes to access the DMA SPRs */
void grant_dma_mpls(void)
{
#if CONFIG_KERNEL_PL == 2
	__insn_mtspr(SPR_MPL_DMA_CPL_SET_1, 1);
	__insn_mtspr(SPR_MPL_DMA_NOTIFY_SET_1, 1);
#else
	__insn_mtspr(SPR_MPL_DMA_CPL_SET_0, 1);
	__insn_mtspr(SPR_MPL_DMA_NOTIFY_SET_0, 1);
#endif
}

/* Forbid user processes from accessing the DMA SPRs */
void restrict_dma_mpls(void)
{
#if CONFIG_KERNEL_PL == 2
	__insn_mtspr(SPR_MPL_DMA_CPL_SET_2, 1);
	__insn_mtspr(SPR_MPL_DMA_NOTIFY_SET_2, 1);
#else
	__insn_mtspr(SPR_MPL_DMA_CPL_SET_1, 1);
	__insn_mtspr(SPR_MPL_DMA_NOTIFY_SET_1, 1);
#endif
}

/* Pause the DMA engine, then save off its state registers. */
static void save_tile_dma_state(struct tile_dma_state *dma)
{
	unsigned long state = __insn_mfspr(SPR_DMA_USER_STATUS);
	unsigned long post_suspend_state;

	/* If we're running, suspend the engine. */
	if ((state & DMA_STATUS_MASK) == SPR_DMA_STATUS__RUNNING_MASK)
		__insn_mtspr(SPR_DMA_CTR, SPR_DMA_CTR__SUSPEND_MASK);

	/*
	 * Wait for the engine to idle, then save regs.  Note that we
	 * want to record the "running" bit from before suspension,
	 * and the "done" bit from after, so that we can properly
	 * distinguish a case where the user suspended the engine from
	 * the case where the kernel suspended as part of the context
	 * swap.
	 */
	do {
		post_suspend_state = __insn_mfspr(SPR_DMA_USER_STATUS);
	} while (post_suspend_state & SPR_DMA_STATUS__BUSY_MASK);

	dma->src = __insn_mfspr(SPR_DMA_SRC_ADDR);
	dma->src_chunk = __insn_mfspr(SPR_DMA_SRC_CHUNK_ADDR);
	dma->dest = __insn_mfspr(SPR_DMA_DST_ADDR);
	dma->dest_chunk = __insn_mfspr(SPR_DMA_DST_CHUNK_ADDR);
	dma->strides = __insn_mfspr(SPR_DMA_STRIDE);
	dma->chunk_size = __insn_mfspr(SPR_DMA_CHUNK_SIZE);
	dma->byte = __insn_mfspr(SPR_DMA_BYTE);
	dma->status = (state & SPR_DMA_STATUS__RUNNING_MASK) |
		(post_suspend_state & SPR_DMA_STATUS__DONE_MASK);
}

/* Restart a DMA that was running before we were context-switched out. */
static void restore_tile_dma_state(struct thread_struct *t)
{
	const struct tile_dma_state *dma = &t->tile_dma_state;

	/*
	 * The only way to restore the done bit is to run a zero
	 * length transaction.
	 */
	if ((dma->status & SPR_DMA_STATUS__DONE_MASK) &&
	    !(__insn_mfspr(SPR_DMA_USER_STATUS) & SPR_DMA_STATUS__DONE_MASK)) {
		__insn_mtspr(SPR_DMA_BYTE, 0);
		__insn_mtspr(SPR_DMA_CTR, SPR_DMA_CTR__REQUEST_MASK);
		while (__insn_mfspr(SPR_DMA_USER_STATUS) &
		       SPR_DMA_STATUS__BUSY_MASK)
			;
	}

	__insn_mtspr(SPR_DMA_SRC_ADDR, dma->src);
	__insn_mtspr(SPR_DMA_SRC_CHUNK_ADDR, dma->src_chunk);
	__insn_mtspr(SPR_DMA_DST_ADDR, dma->dest);
	__insn_mtspr(SPR_DMA_DST_CHUNK_ADDR, dma->dest_chunk);
	__insn_mtspr(SPR_DMA_STRIDE, dma->strides);
	__insn_mtspr(SPR_DMA_CHUNK_SIZE, dma->chunk_size);
	__insn_mtspr(SPR_DMA_BYTE, dma->byte);

	/*
	 * Restart the engine if we were running and not done.
	 * Clear a pending async DMA fault that we were waiting on return
	 * to user space to execute, since we expect the DMA engine
	 * to regenerate those faults for us now.  Note that we don't
	 * try to clear the TIF_ASYNC_TLB flag, since it's relatively
	 * harmless if set, and it covers both DMA and the SN processor.
	 */
	if ((dma->status & DMA_STATUS_MASK) == SPR_DMA_STATUS__RUNNING_MASK) {
		t->dma_async_tlb.fault_num = 0;
		__insn_mtspr(SPR_DMA_CTR, SPR_DMA_CTR__REQUEST_MASK);
	}
}

#endif

static void save_arch_state(struct thread_struct *t)
{
#if CHIP_HAS_SPLIT_INTR_MASK()
	t->interrupt_mask = __insn_mfspr(SPR_INTERRUPT_MASK_0_0) |
		((u64)__insn_mfspr(SPR_INTERRUPT_MASK_0_1) << 32);
#else
	t->interrupt_mask = __insn_mfspr(SPR_INTERRUPT_MASK_0);
#endif
	t->ex_context[0] = __insn_mfspr(SPR_EX_CONTEXT_0_0);
	t->ex_context[1] = __insn_mfspr(SPR_EX_CONTEXT_0_1);
	t->system_save[0] = __insn_mfspr(SPR_SYSTEM_SAVE_0_0);
	t->system_save[1] = __insn_mfspr(SPR_SYSTEM_SAVE_0_1);
	t->system_save[2] = __insn_mfspr(SPR_SYSTEM_SAVE_0_2);
	t->system_save[3] = __insn_mfspr(SPR_SYSTEM_SAVE_0_3);
	t->intctrl_0 = __insn_mfspr(SPR_INTCTRL_0_STATUS);
	t->proc_status = __insn_mfspr(SPR_PROC_STATUS);
#if !CHIP_HAS_FIXED_INTVEC_BASE()
	t->interrupt_vector_base = __insn_mfspr(SPR_INTERRUPT_VECTOR_BASE_0);
#endif
	t->tile_rtf_hwm = __insn_mfspr(SPR_TILE_RTF_HWM);
#if CHIP_HAS_DSTREAM_PF()
	t->dstream_pf = __insn_mfspr(SPR_DSTREAM_PF);
#endif
#if CHIP_HAS_IPI() && defined(CONFIG_HARDWALL)
	t->ipi0_event_mask = __insn_mfspr(SPR_IPI_MASK_0);
#endif
#ifdef CONFIG_USE_PMC
	if (userspace_perf_counters) {
		t->perf_count_ctl = __insn_mfspr(SPR_PERF_COUNT_CTL);
		t->aux_perf_count_ctl = __insn_mfspr(SPR_AUX_PERF_COUNT_CTL);
		t->perf_count_0 = __insn_mfspr(SPR_PERF_COUNT_0);
		t->perf_count_1 = __insn_mfspr(SPR_PERF_COUNT_1);
		t->aux_perf_count_0 = __insn_mfspr(SPR_AUX_PERF_COUNT_0);
		t->aux_perf_count_1 = __insn_mfspr(SPR_AUX_PERF_COUNT_1);
	}
#endif
}

static void restore_arch_state(const struct thread_struct *t)
{
#if CHIP_HAS_SPLIT_INTR_MASK()
	__insn_mtspr(SPR_INTERRUPT_MASK_0_0, (u32) t->interrupt_mask);
	__insn_mtspr(SPR_INTERRUPT_MASK_0_1, t->interrupt_mask >> 32);
#else
	__insn_mtspr(SPR_INTERRUPT_MASK_0, t->interrupt_mask);
#endif
	__insn_mtspr(SPR_EX_CONTEXT_0_0, t->ex_context[0]);
	__insn_mtspr(SPR_EX_CONTEXT_0_1, t->ex_context[1]);
	__insn_mtspr(SPR_SYSTEM_SAVE_0_0, t->system_save[0]);
	__insn_mtspr(SPR_SYSTEM_SAVE_0_1, t->system_save[1]);
	__insn_mtspr(SPR_SYSTEM_SAVE_0_2, t->system_save[2]);
	__insn_mtspr(SPR_SYSTEM_SAVE_0_3, t->system_save[3]);
	__insn_mtspr(SPR_INTCTRL_0_STATUS, t->intctrl_0);
	__insn_mtspr(SPR_PROC_STATUS, t->proc_status);
#if !CHIP_HAS_FIXED_INTVEC_BASE()
	__insn_mtspr(SPR_INTERRUPT_VECTOR_BASE_0, t->interrupt_vector_base);
#endif
	__insn_mtspr(SPR_TILE_RTF_HWM, t->tile_rtf_hwm);
#if CHIP_HAS_DSTREAM_PF()
	__insn_mtspr(SPR_DSTREAM_PF, t->dstream_pf);
#endif
#if CHIP_HAS_IPI() && defined(CONFIG_HARDWALL)
	__insn_mtspr(SPR_IPI_MASK_0, t->ipi0_event_mask);
#endif
#ifdef CONFIG_USE_PMC
	if (userspace_perf_counters) {
		__insn_mtspr(SPR_PERF_COUNT_CTL, t->perf_count_ctl);
		__insn_mtspr(SPR_AUX_PERF_COUNT_CTL, t->aux_perf_count_ctl);
		__insn_mtspr(SPR_PERF_COUNT_0, t->perf_count_0);
		__insn_mtspr(SPR_PERF_COUNT_1, t->perf_count_1);
		__insn_mtspr(SPR_AUX_PERF_COUNT_0, t->aux_perf_count_0);
		__insn_mtspr(SPR_AUX_PERF_COUNT_1, t->aux_perf_count_1);
	}
#endif
}


void _prepare_arch_switch(struct task_struct *next)
{
#if CHIP_HAS_TILE_DMA()
	struct tile_dma_state *dma = &current->thread.tile_dma_state;
	if (dma->enabled)
		save_tile_dma_state(dma);
#endif
}


struct task_struct *__sched _switch_to(struct task_struct *prev,
				       struct task_struct *next)
{
#ifdef CONFIG_KVM
	/* vmexit is needed before context switch. */
	BUG_ON(task_thread_info(prev)->vcpu);
#endif

	/* DMA state is already saved; save off other arch state. */
	save_arch_state(&prev->thread);

#if CHIP_HAS_TILE_DMA()
	/*
	 * Restore DMA in new task if desired.
	 * Note that it is only safe to restart here since interrupts
	 * are disabled, so we can't take any DMATLB miss or access
	 * interrupts before we have finished switching stacks.
	 */
	if (next->thread.tile_dma_state.enabled) {
		restore_tile_dma_state(&next->thread);
		grant_dma_mpls();
	} else {
		restrict_dma_mpls();
	}
#endif

	/* Restore other arch state. */
	restore_arch_state(&next->thread);

#ifdef CONFIG_HARDWALL
	/* Enable or disable access to the network registers appropriately. */
	hardwall_switch_tasks(prev, next);
#endif

#ifdef CONFIG_DATAPLANE
	/* Update per-cpu dataplane state to reflect new process. */
	__get_cpu_var(dp_flags) = next->thread.dp_flags;
#endif

	/*
	 * Switch kernel SP, PC, and callee-saved registers.
	 * In the context of the new task, return the old task pointer
	 * (i.e. the task that actually called __switch_to).
	 * Pass the value to use for SYSTEM_SAVE_K_0 when we reset our sp.
	 */
	return __switch_to(prev, next, next_current_ksp0(next));
}

/*
 * This routine is called on return from interrupt if any of the
 * TIF_WORK_MASK flags are set in thread_info->flags.  It is
 * entered with interrupts disabled so we don't miss an event
 * that modified the thread_info flags.  If any flag is set, we
 * handle it and return, and the calling assembly code will
 * re-disable interrupts, reload the thread flags, and call back
 * if more flags need to be handled.
 *
 * We return whether we need to check the thread_info flags again
 * or not.  Note that we don't clear TIF_SINGLESTEP here, so it's
 * important that it be tested last, and then claim that we don't
 * need to recheck the flags.
 */
int do_work_pending(struct pt_regs *regs, u32 thread_info_flags)
{
	/* If we enter in kernel mode, do nothing and exit the caller loop. */
	if (!user_mode(regs))
		return 0;

	/* Enable interrupts; they are disabled again on return to caller. */
	local_irq_enable();

#ifdef CONFIG_KVM
	/*
	 * Some work requires us to exit the VM first.  Typically this
	 * allows the process running the VM to respond to the work
	 * (e.g. a signal), or allows the VM mechanism to latch
	 * modified host state (e.g. a "hypervisor" message sent to a
	 * different vcpu).  It also means that if we are considering
	 * calling schedule(), we exit the VM first, so we never have
	 * to worry about context-switching into a VM.
	 */
	if (current_thread_info()->vcpu) {
		u32 do_exit = thread_info_flags &
			(_TIF_NEED_RESCHED|_TIF_SIGPENDING|_TIF_VIRT_EXIT);

		if (thread_info_flags & _TIF_VIRT_EXIT)
			clear_thread_flag(TIF_VIRT_EXIT);
		if (do_exit) {
			kvm_trigger_vmexit(regs, KVM_EXIT_AGAIN);
			/*NORETURN*/
		}
	}
#endif

	if (thread_info_flags & _TIF_NEED_RESCHED) {
		schedule();
		return 1;
	}
#if CHIP_HAS_TILE_DMA()
	if (thread_info_flags & _TIF_ASYNC_TLB) {
		do_async_page_fault(regs);
		return 1;
	}
#endif
	if (thread_info_flags & _TIF_UPROBE) {
		clear_thread_flag(TIF_UPROBE);
		uprobe_notify_resume(regs);
		return 1;
	}
	if (thread_info_flags & _TIF_SIGPENDING) {
		do_signal(regs);
		return 1;
	}
	if (thread_info_flags & _TIF_NOTIFY_RESUME) {
		clear_thread_flag(TIF_NOTIFY_RESUME);
		tracehook_notify_resume(regs);
		if (current->replacement_session_keyring)
			key_replace_session_keyring();
		return 1;
	}

	/* Handle a few flags here that stay set. */
	if (thread_info_flags & _TIF_SINGLESTEP)
		single_step_once(regs);

	return 0;
}

/* Note there is an implicit fifth argument if (clone_flags & CLONE_SETTLS). */
SYSCALL_DEFINE5(clone, unsigned long, clone_flags, unsigned long, newsp,
		void __user *, parent_tidptr, void __user *, child_tidptr,
		struct pt_regs *, regs)
{
	if (!newsp)
		newsp = regs->sp;
	return do_fork(clone_flags, newsp, regs, 0,
		       parent_tidptr, child_tidptr);
}

/*
 * sys_execve() executes a new program.
 */
SYSCALL_DEFINE4(execve, const char __user *, path,
		const char __user *const __user *, argv,
		const char __user *const __user *, envp,
		struct pt_regs *, regs)
{
	long error;
	char *filename;

	filename = getname(path);
	error = PTR_ERR(filename);
	if (IS_ERR(filename))
		goto out;
	error = do_execve(filename, argv, envp, regs);
	putname(filename);
	if (error == 0)
		single_step_execve();
out:
	return error;
}

#ifdef CONFIG_COMPAT
long compat_sys_execve(const char __user *path,
		       compat_uptr_t __user *argv,
		       compat_uptr_t __user *envp, struct pt_regs *regs)
{
	long error;
	char *filename;

	filename = getname(path);
	error = PTR_ERR(filename);
	if (IS_ERR(filename))
		goto out;
	error = compat_do_execve(filename, argv, envp, regs);
	putname(filename);
	if (error == 0)
		single_step_execve();
out:
	return error;
}
#endif

unsigned long get_wchan(struct task_struct *p)
{
	struct KBacktraceIterator kbt;

	if (!p || p == current || p->state == TASK_RUNNING)
		return 0;

	for (KBacktraceIterator_init(&kbt, p, NULL);
	     !KBacktraceIterator_end(&kbt);
	     KBacktraceIterator_next(&kbt)) {
		if (!in_sched_functions(kbt.it.pc))
			return kbt.it.pc;
	}

	return 0;
}

/*
 * We pass in lr as zero (cleared in kernel_thread) and the caller
 * part of the backtrace ABI on the stack also zeroed (in copy_thread)
 * so that backtraces will stop with this function.
 * Note that we don't use r0, since copy_thread() clears it.
 */
static void start_kernel_thread(int dummy, int (*fn)(int), int arg)
{
	do_exit(fn(arg));
}

/*
 * Create a kernel thread
 */
int kernel_thread(int (*fn)(void *), void * arg, unsigned long flags)
{
	struct pt_regs regs;

	memset(&regs, 0, sizeof(regs));
	regs.ex1 = PL_ICS_EX1(KERNEL_PL, 0);  /* run at kernel PL, no ICS */
	regs.pc = (long) start_kernel_thread;
	regs.flags = PT_FLAGS_CALLER_SAVES;   /* need to restore r1 and r2 */
	regs.regs[1] = (long) fn;             /* function pointer */
	regs.regs[2] = (long) arg;            /* parameter register */

	/* Ok, create the new process.. */
	return do_fork(flags | CLONE_VM | CLONE_UNTRACED, 0, &regs,
		       0, NULL, NULL);
}
EXPORT_SYMBOL(kernel_thread);

/* Flush thread state. */
void flush_thread(void)
{
	/* Nothing */
}

/*
 * Free current thread data structures etc..
 */
void exit_thread(void)
{
#ifdef CONFIG_HARDWALL
	/*
	 * Remove the task from the list of tasks that are associated
	 * with any live hardwalls.  (If the task that is exiting held
	 * the last reference to a hardwall fd, it would already have
	 * been released and deactivated at this point.)
	 */
	hardwall_deactivate_all(current);
#endif
}

void tile_show_regs(struct pt_regs *regs)
{
	int i;
#ifdef __tilegx__
	for (i = 0; i < 17; i++)
		pr_err(" r%-2d: "REGFMT" r%-2d: "REGFMT" r%-2d: "REGFMT"\n",
		       i, regs->regs[i], i+18, regs->regs[i+18],
		       i+36, regs->regs[i+36]);
	pr_err(" r17: "REGFMT" r35: "REGFMT" tp : "REGFMT"\n",
	       regs->regs[17], regs->regs[35], regs->tp);
	pr_err(" sp : "REGFMT" lr : "REGFMT"\n", regs->sp, regs->lr);
#else
	for (i = 0; i < 13; i++)
		pr_err(" r%-2d: "REGFMT" r%-2d: "REGFMT
		       " r%-2d: "REGFMT" r%-2d: "REGFMT"\n",
		       i, regs->regs[i], i+14, regs->regs[i+14],
		       i+27, regs->regs[i+27], i+40, regs->regs[i+40]);
	pr_err(" r13: "REGFMT" tp : "REGFMT" sp : "REGFMT" lr : "REGFMT"\n",
	       regs->regs[13], regs->tp, regs->sp, regs->lr);
#endif
	pr_err(" pc : "REGFMT" ex1: %ld     faultnum: %ld flags:%s%s%s%s\n",
	       regs->pc, regs->ex1, regs->faultnum,
	       is_compat_task() ? " compat" : "",
	       (regs->flags & PT_FLAGS_DISABLE_IRQ) ? " noirq" : "",
	       !(regs->flags & PT_FLAGS_CALLER_SAVES) ? " nocallersave" : "",
	       (regs->flags & PT_FLAGS_RESTORE_REGS) ? " restoreregs" : "");
}

void show_regs(struct pt_regs *regs)
{
	struct task_struct *tsk = validate_current();

	pr_err("\n");
	pr_err(" Pid: %d, comm: %20s, CPU: %d\n",
	       tsk->pid, tsk->comm, raw_smp_processor_id());
	tile_show_regs(regs);
	dump_stack_regs(regs);
}

/* To ensure stack dump on tiles occurs one by one. */
static DEFINE_SPINLOCK(backtrace_lock);
/* To ensure no backtrace occurs before all of the stack dump are done. */
static atomic_t backtrace_cpus;
/* The cpu mask to avoid reentrance. */
static struct cpumask backtrace_mask;

void do_nmi_dump_stack(struct pt_regs *regs)
{
	int is_idle = is_idle_task(current) && !in_interrupt();
	int cpu;

	nmi_enter();
	cpu = smp_processor_id();
	if (WARN_ON_ONCE(!cpumask_test_and_clear_cpu(cpu, &backtrace_mask)))
		goto done;

	spin_lock(&backtrace_lock);
	if (is_idle)
		pr_info("Skipping stack dump of idle task on cpu %d\n", cpu);
	else
		show_regs(regs);
	spin_unlock(&backtrace_lock);
	atomic_dec(&backtrace_cpus);
done:
	nmi_exit();
}

#ifdef __tilegx__
void arch_trigger_all_cpu_backtrace(void)
{
	struct cpumask mask;
	HV_Coord tile;
	unsigned int timeout;
	int cpu;
	int ongoing;
	HV_NMI_Info info[NR_CPUS];

	ongoing = atomic_cmpxchg(&backtrace_cpus, 0, num_online_cpus() - 1);
	if (ongoing != 0) {
		pr_err("All-cpu backtrace ongoing (%d cpus left)\n", ongoing);
		pr_err("Reporting on this cpu only.\n");
		dump_stack();
		return;
	}

	cpumask_copy(&mask, cpu_online_mask);
	cpumask_clear_cpu(smp_processor_id(), &mask);
	cpumask_copy(&backtrace_mask, &mask);

	/* Backtrace for myself first. */
	dump_stack();

	/* Tentatively dump stack on remote tiles via NMI. */
	timeout = 100;
	while (!cpumask_empty(&mask) && timeout) {
		for_each_cpu(cpu, &mask) {
			tile.x = cpu_x(cpu);
			tile.y = cpu_y(cpu);
			info[cpu] = hv_send_nmi(tile, TILE_NMI_DUMP_STACK, 0);
			if (info[cpu].result == HV_NMI_RESULT_OK)
				cpumask_clear_cpu(cpu, &mask);
		}

		mdelay(10);
		timeout--;
	}

	/* Warn about cpus stuck in ICS and decrement their counts here. */
	if (!cpumask_empty(&mask)) {
		for_each_cpu(cpu, &mask) {
			switch (info[cpu].result) {
			case HV_NMI_RESULT_FAIL_ICS:
				pr_warn("Skipping stack dump of cpu %d in ICS"
					" at pc %#llx\n", cpu, info[cpu].pc);
				break;
			case HV_NMI_RESULT_FAIL_HV:
				pr_warn("Skipping stack dump of cpu %d"
					" in hypervisor\n", cpu);
				break;
			default:  /* should not happen */
				pr_warn("Skipping stack dump of cpu %d"
					" [%d,%#llx]\n", cpu,
					info[cpu].result, info[cpu].pc);
				break;
			}
		}
		atomic_sub(cpumask_weight(&mask), &backtrace_cpus);
	} else {
		pr_info("Requested backtrace from all cpus.\n");
	}
}
#endif /* __tilegx_ */
