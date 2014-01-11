/*
 * User-space Probes (UProbes) for tile
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

#include <linux/kdebug.h>
#include <linux/kernel.h>
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/uprobes.h>

#define UPROBE_TRAP_NR		UINT_MAX

/* Post-execution fixups. */

/* Adjust PC back to vicinity of actual insn */
#define UPROBE_FIX_PC		0x1

/* Adjust the return address of a 'jal' insn, etc. */
#define UPROBE_FIX_LR		0x2

/* Rewrite the destination register of a 'lnk' insn */
#define UPROBE_FIX_LNK		0x4


static int set_reg(int regno, void *mem, struct pt_regs *regs)
{
	unsigned long offset = 0;

	if (regno > TREG_LAST_GPR || regno < 0)
		return -EINVAL;

	if (regno < TREG_TP) {
		offset = offsetof(struct pt_regs, regs[regno]);
	} else if (regno ==  TREG_TP) {
		offset = offsetof(struct pt_regs, tp);
	} else if (regno ==  TREG_SP) {
		offset = offsetof(struct pt_regs, sp);
	} else if (regno ==  TREG_LR) {
		offset = offsetof(struct pt_regs, lr);
	}

	memcpy((void *)regs + offset, mem, UPROBE_XOL_SLOT_BYTES);
	return 0;
}

/**
 * Figure out which fixups arch_uprobe_post_xol() will need to perform, and
 * annotate arch_uprobe->fixups accordingly.
 */
static void prepare_fixups(struct arch_uprobe *auprobe)
{
	unsigned int opcode;
	tile_bundle_bits bundle = (tile_bundle_bits)auprobe->ainsn;

	if (get_bundle_mode(bundle) == Y_MODE) {
		if (get_Opcode_Y1(bundle) == RRR_1_OPCODE_Y1 &&
		    get_RRROpcodeExtension_Y1(bundle) ==
		    UNARY_RRR_1_OPCODE_Y1) {
			opcode = get_UnaryOpcodeExtension_Y1(bundle);

			switch (opcode) {
			case LNK_UNARY_OPCODE_Y1:
				auprobe->fixups |= UPROBE_FIX_LNK;
				auprobe->fixups |= UPROBE_FIX_PC;
				return;

			case JALR_UNARY_OPCODE_Y1:
			case JALRP_UNARY_OPCODE_Y1:
				auprobe->fixups |= UPROBE_FIX_LR;
				return;

			case JR_UNARY_OPCODE_Y1:
			case JRP_UNARY_OPCODE_Y1:
				return;
			}
		}
	} else if (get_Opcode_X1(bundle) == RRR_0_OPCODE_X1) {
		if (get_RRROpcodeExtension_X1(bundle) ==
		    UNARY_RRR_0_OPCODE_X1) {
			opcode = get_UnaryOpcodeExtension_X1(bundle);

			switch (opcode) {
			case LNK_UNARY_OPCODE_X1:
				auprobe->fixups |= UPROBE_FIX_LNK;
				auprobe->fixups |= UPROBE_FIX_PC;
				return;

			case JALR_UNARY_OPCODE_X1:
			case JALRP_UNARY_OPCODE_X1:
				auprobe->fixups |= UPROBE_FIX_LR;
				return;

			case JR_UNARY_OPCODE_X1:
			case JRP_UNARY_OPCODE_X1:
				return;
			}
		}
	} else if (get_Opcode_X1(bundle) == JUMP_OPCODE_X1 &&
		   get_JumpOpcodeExtension_X1(bundle) ==
		   JAL_JUMP_OPCODE_X1) {
			auprobe->fixups |= UPROBE_FIX_LR;
			auprobe->fixups |= UPROBE_FIX_PC;
			return;
	}

	// Other insn, need to adjust PC.
	auprobe->fixups |= UPROBE_FIX_PC;
}

/**
 * arch_uprobe_analyze_insn
 * @auprobe: the probepoint information.
 * @mm: the probed address space.
 * @addr: vaddr to probe.
 * Return 0 on success or a -ve number on error.
 */
int arch_uprobe_analyze_insn(struct arch_uprobe *auprobe, struct mm_struct *mm)
{
	prepare_fixups(auprobe);

	return 0;
}

/**
 * arch_uprobe_pre_xol - prepare to execute out of line.
 * @auprobe: the probepoint information.
 * @regs: reflects the saved user state of current task.
 */
int arch_uprobe_pre_xol(struct arch_uprobe *auprobe, struct pt_regs *regs)
{
	struct arch_uprobe_task *autask = &current->utask->autask;

	autask->saved_trap_nr = current->thread.trap_nr;
	current->thread.trap_nr = UPROBE_TRAP_NR;
	instruction_pointer(regs) = current->utask->xol_vaddr;

	user_enable_single_step(current);
	return 0;
}

/**
 * Called after single-stepping. To avoid the SMP problems that can
 * occur when we temporarily put back the original opcode to
 * single-step, we single-stepped a copy of the instruction.
 *
 * This function prepares to resume execution after the single-step.
 */
int arch_uprobe_post_xol(struct arch_uprobe *auprobe, struct pt_regs *regs)
{
	struct uprobe_task *utask = current->utask;
	tile_bundle_bits bundle = (tile_bundle_bits)auprobe->ainsn;
	unsigned long addr = 0;
	long offset = 0;

	WARN_ON_ONCE(current->thread.trap_nr != UPROBE_TRAP_NR);
	current->thread.trap_nr = utask->autask.saved_trap_nr;

	if (auprobe->fixups & UPROBE_FIX_LR) {
		// Address of the subsequent instruction.
		addr = utask->vaddr + UPROBE_XOL_SLOT_BYTES;
		set_reg(TREG_LR, &addr, regs);
	}

	if (auprobe->fixups & UPROBE_FIX_LNK) {
		// Address of the subsequent instruction.
		addr = utask->vaddr + UPROBE_XOL_SLOT_BYTES;
		if (get_bundle_mode(bundle) == Y_MODE)
			set_reg(get_Dest_Y1(bundle), &addr, regs);
		else
			set_reg(get_Dest_X1(bundle), &addr, regs);
	}

	if (auprobe->fixups & UPROBE_FIX_PC) {
		offset = instruction_pointer(regs) - utask->xol_vaddr;
		instruction_pointer(regs) = utask->vaddr + offset;
	}

	user_disable_single_step(current);
	return 0;
}

/**
 * If xol insn itself traps and generates a signal (SIGILL/SIGSEGV/etc),
 * then detect the case where a singlestepped instruction jumps back to its
 * own address. It is assumed that anything like do_page_fault/do_trap/etc
 * sets thread.trap_nr != UINT_MAX.
 *
 * arch_uprobe_pre_xol/arch_uprobe_post_xol save/restore thread.trap_nr,
 * arch_uprobe_xol_was_trapped() simply checks that ->trap_nr is not equal to
 * UPROBE_TRAP_NR == UINT_MAX set by arch_uprobe_pre_xol().
 */
bool arch_uprobe_xol_was_trapped(struct task_struct *tsk)
{
	if (tsk->thread.trap_nr != UPROBE_TRAP_NR)
		return true;

	return false;
}

/**
 * callback routine for handling exceptions.
 */
int arch_uprobe_exception_notify(struct notifier_block *self,
				unsigned long val, void *data)
{
	struct die_args *args = data;
	struct pt_regs *regs = args->regs;

	/* regs == NULL is a kernel bug */
	if (WARN_ON(!regs))
		return NOTIFY_DONE;

        /* We are only interested in userspace traps */
	if (!user_mode(regs))
		return NOTIFY_DONE;

	switch (val) {
	case DIE_BREAK:
		if (uprobe_pre_sstep_notifier(regs))
			return NOTIFY_STOP;
		break;
	case DIE_SSTEPBP:
		if (uprobe_post_sstep_notifier(regs))
			return NOTIFY_STOP;
	default:
		break;
	}
	return NOTIFY_DONE;
}

/**
 * This function gets called when XOL instruction either gets trapped or
 * the thread has a fatal signal, so reset the instruction pointer to its
 * probed address.
 */
void arch_uprobe_abort_xol(struct arch_uprobe *auprobe, struct pt_regs *regs)
{
	struct uprobe_task *utask = current->utask;

	current->thread.trap_nr = utask->autask.saved_trap_nr;
	instruction_pointer_set(regs, utask->vaddr);

	user_disable_single_step(current);
}

/**
 * Avoid singlestepping the original instruction if it can be skipped.
 * Returns true if instruction can be skipped, false otherwise.
 */
bool arch_uprobe_skip_sstep(struct arch_uprobe *auprobe, struct pt_regs *regs)
{
	return false;
}

/**
 * uprobe_get_swbp_addr - compute address of swbp given post-swbp regs
 * @regs: Reflects the saved state of the task after it has hit a breakpoint
 * instruction.
 * Return the address of the breakpoint instruction.
 */
unsigned long uprobe_get_swbp_addr(struct pt_regs *regs)
{
	return instruction_pointer(regs);
}
