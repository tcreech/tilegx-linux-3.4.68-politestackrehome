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
 *
 * Copied from i386: Ross Biro 1/23/92
 */

#include <linux/kernel.h>
#include <linux/ptrace.h>
#include <linux/kprobes.h>
#include <linux/compat.h>
#include <linux/uaccess.h>
#include <linux/regset.h>
#include <linux/elf.h>
#include <linux/tracehook.h>
#include <asm/traps.h>
#include <arch/chip.h>

#define CREATE_TRACE_POINTS
#include <trace/events/syscalls.h>

void user_enable_single_step(struct task_struct *child)
{
	set_tsk_thread_flag(child, TIF_SINGLESTEP);
}

void user_disable_single_step(struct task_struct *child)
{
	clear_tsk_thread_flag(child, TIF_SINGLESTEP);
}

/*
 * Called by kernel/ptrace.c when detaching..
 */
void ptrace_disable(struct task_struct *child)
{
	clear_tsk_thread_flag(child, TIF_SINGLESTEP);

	/*
	 * These two are currently unused, but will be set by arch_ptrace()
	 * and used in the syscall assembly when we do support them.
	 */
	clear_tsk_thread_flag(child, TIF_SYSCALL_TRACE);
}

/*
 * Get registers from task and ready the result for userspace.
 * Note that we localize the API issues to getregs() and putregs() at
 * some cost in performance, e.g. we need a full pt_regs copy for
 * PEEKUSR, and two copies for POKEUSR.  But in general we expect
 * GETREGS/PUTREGS to be the API of choice anyway.
 */
static char *getregs(struct task_struct *child, struct pt_regs *uregs)
{
	*uregs = *task_pt_regs(child);

	/* Set up flags ABI bits. */
	uregs->flags = 0;
#ifdef CONFIG_COMPAT
	if (task_thread_info(child)->status & TS_COMPAT)
		uregs->flags |= PT_FLAGS_COMPAT;
#endif

	return (char *)uregs;
}

/* Put registers back to task. */
static void putregs(struct task_struct *child, struct pt_regs *uregs)
{
	struct pt_regs *regs = task_pt_regs(child);

	/*
	 * Don't allow overwriting the kernel-internal flags word.
	 * But do set PT_FLAGS_RESTORE_REGS so that the kernel will reload
	 * all the callee-saved registers when returning to userspace.
	 */ 
	uregs->flags = regs->flags | PT_FLAGS_RESTORE_REGS;

	/* Only allow setting the ICS bit in the ex1 word. */
	uregs->ex1 = PL_ICS_EX1(USER_PL, EX1_ICS(uregs->ex1));

	*regs = *uregs;
}

enum tile_regset {
	REGSET_GPR,
};

static int tile_gpr_get(struct task_struct *target,
			  const struct user_regset *regset,
			  unsigned int pos, unsigned int count,
			  void *kbuf, void __user *ubuf)
{
	struct pt_regs regs;

	getregs(target, &regs);

	return user_regset_copyout(&pos, &count, &kbuf, &ubuf, &regs, 0,
				   sizeof(regs));
}

static int tile_gpr_set(struct task_struct *target,
			  const struct user_regset *regset,
			  unsigned int pos, unsigned int count,
			  const void *kbuf, const void __user *ubuf)
{
	int ret;
	struct pt_regs regs;

	ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf, &regs, 0,
				 sizeof(regs));
	if (ret)
		return ret;

	putregs(target, &regs);

	return 0;
}

static const struct user_regset tile_user_regset[] = {
	[REGSET_GPR] = {
		.core_note_type = NT_PRSTATUS,
		.n = ELF_NGREG,
		.size = sizeof(elf_greg_t),
		.align = sizeof(elf_greg_t),
		.get = tile_gpr_get,
		.set = tile_gpr_set,
	},
};

static const struct user_regset_view tile_user_regset_view = {
	.name = CHIP_ARCH_NAME,
	.e_machine = ELF_ARCH,
	.ei_osabi = ELF_OSABI,
	.regsets = tile_user_regset,
	.n = ARRAY_SIZE(tile_user_regset),
};

const struct user_regset_view *task_user_regset_view(struct task_struct *task)
{
	return &tile_user_regset_view;
}

long arch_ptrace(struct task_struct *child, long request,
		 unsigned long addr, unsigned long data)
{
	unsigned long __user *datap = (long __user __force *)data;
	unsigned long tmp;
	long ret = -EIO;
	char *childreg;
	struct pt_regs copyregs;

	switch (request) {

	case PTRACE_PEEKUSR:  /* Read register from pt_regs. */
		if (addr >= PTREGS_SIZE)
			break;
		childreg = getregs(child, &copyregs) + addr;
#ifdef CONFIG_COMPAT
		if (is_compat_task()) {
			if (addr & (sizeof(compat_long_t)-1))
				break;
			ret = put_user(*(compat_long_t *)childreg,
				       (compat_long_t __user *)datap);
		} else
#endif
		{
			if (addr & (sizeof(long)-1))
				break;
			ret = put_user(*(long *)childreg, datap);
		}
		break;

	case PTRACE_POKEUSR:  /* Write register in pt_regs. */
		if (addr >= PTREGS_SIZE)
			break;
		childreg = getregs(child, &copyregs) + addr;
#ifdef CONFIG_COMPAT
		if (is_compat_task()) {
			if (addr & (sizeof(compat_long_t)-1))
				break;
			*(compat_long_t *)childreg = data;
		} else
#endif
		{
			if (addr & (sizeof(long)-1))
				break;
			*(long *)childreg = data;
		}
		putregs(child, &copyregs);
		ret = 0;
		break;

	case PTRACE_GETREGS:  /* Get all registers from the child. */
		ret = copy_regset_to_user(child, &tile_user_regset_view,
					  REGSET_GPR, 0,
					  sizeof(struct pt_regs), datap);
		break;

	case PTRACE_SETREGS:  /* Set all registers in the child. */
		ret = copy_regset_from_user(child, &tile_user_regset_view,
					    REGSET_GPR, 0,
					    sizeof(struct pt_regs), datap);
		break;

	case PTRACE_GETFPREGS:  /* Get the child FPU state. */
	case PTRACE_SETFPREGS:  /* Set the child FPU state. */
		break;

	case PTRACE_SETOPTIONS:
		/* Support TILE-specific ptrace options. */
		child->ptrace &= ~PT_TRACE_MASK_TILE;
		tmp = data & PTRACE_O_MASK_TILE;
		data &= ~PTRACE_O_MASK_TILE;
		ret = ptrace_request(child, request, addr, data);
		if (tmp & PTRACE_O_TRACEMIGRATE)
			child->ptrace |= PT_TRACE_MIGRATE;
		break;

	default:
#ifdef CONFIG_COMPAT
		if (task_thread_info(current)->status & TS_COMPAT) {
			ret = compat_ptrace_request(child, request,
						    addr, data);
			break;
		}
#endif
		ret = ptrace_request(child, request, addr, data);
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
/* Not used; we handle compat issues in arch_ptrace() directly. */
long compat_arch_ptrace(struct task_struct *child, compat_long_t request,
			       compat_ulong_t addr, compat_ulong_t data)
{
	BUG();
}
#endif

int do_syscall_trace_enter(struct pt_regs *regs)
{
	if (test_thread_flag(TIF_SYSCALL_TRACE)) {
		if (tracehook_report_syscall_entry(regs))
			regs->regs[TREG_SYSCALL_NR] = -1;
	}

	if (test_thread_flag(TIF_SYSCALL_TRACEPOINT))
		trace_sys_enter(regs, regs->regs[TREG_SYSCALL_NR]);

	return regs->regs[TREG_SYSCALL_NR];
}

void do_syscall_trace_exit(struct pt_regs *regs)
{
	long errno;

	/*
	 * The standard tile calling convention returns the value (or negative
	 * errno) in r0, and zero (or positive errno) in r1.
	 * It saves a couple of cycles on the hot path to do this work in
	 * registers only as we return, rather than updating the in-memory
	 * struct ptregs.
	 */
	errno = (long) regs->regs[0];
	if (errno < 0 && errno > -4096)
		regs->regs[1] = -errno;
	else
		regs->regs[1] = 0;

	if (test_thread_flag(TIF_SYSCALL_TRACE))
		tracehook_report_syscall_exit(regs, 0);

	if (test_thread_flag(TIF_SYSCALL_TRACEPOINT))
		trace_sys_exit(regs, regs->regs[0]);
}

void send_sigtrap(struct task_struct *tsk, struct pt_regs *regs)
{
	struct siginfo info;

	current->thread.trap_nr = TRAP_BRKPT;
	memset(&info, 0, sizeof(info));
	info.si_signo = SIGTRAP;
	info.si_code  = TRAP_BRKPT;
	info.si_addr  = (void __user *) regs->pc;

	/* Send us the fakey SIGTRAP */
	force_sig_info(SIGTRAP, &info, tsk);
}

/* Handle synthetic interrupt delivered only by the simulator. */
void __kprobes do_breakpoint(struct pt_regs* regs, int fault_num)
{
	send_sigtrap(current, regs);
}

struct pt_regs_offset {
	const char *name;
	int offset;
};
#define REG_OFFSET_END {.name = NULL, .offset = 0}

static const struct pt_regs_offset regoffset_table[] = {
	{.name = "r0", .offset = PTREGS_OFFSET_REG(0)},
	{.name = "r1", .offset = PTREGS_OFFSET_REG(1)},
	{.name = "r2", .offset = PTREGS_OFFSET_REG(2)},
	{.name = "r3", .offset = PTREGS_OFFSET_REG(3)},
	{.name = "r4", .offset = PTREGS_OFFSET_REG(4)},
	{.name = "r5", .offset = PTREGS_OFFSET_REG(5)},
	{.name = "r6", .offset = PTREGS_OFFSET_REG(6)},
	{.name = "r7", .offset = PTREGS_OFFSET_REG(7)},
	{.name = "r8", .offset = PTREGS_OFFSET_REG(8)},
	{.name = "r9", .offset = PTREGS_OFFSET_REG(9)},
	{.name = "r10", .offset = PTREGS_OFFSET_REG(10)},
	{.name = "r11", .offset = PTREGS_OFFSET_REG(11)},
	{.name = "r12", .offset = PTREGS_OFFSET_REG(12)},
	{.name = "r13", .offset = PTREGS_OFFSET_REG(13)},
	{.name = "r14", .offset = PTREGS_OFFSET_REG(14)},
	{.name = "r15", .offset = PTREGS_OFFSET_REG(15)},
	{.name = "r16", .offset = PTREGS_OFFSET_REG(16)},
	{.name = "r17", .offset = PTREGS_OFFSET_REG(17)},
	{.name = "r18", .offset = PTREGS_OFFSET_REG(18)},
	{.name = "r19", .offset = PTREGS_OFFSET_REG(19)},
	{.name = "r20", .offset = PTREGS_OFFSET_REG(20)},
	{.name = "r21", .offset = PTREGS_OFFSET_REG(21)},
	{.name = "r22", .offset = PTREGS_OFFSET_REG(22)},
	{.name = "r23", .offset = PTREGS_OFFSET_REG(23)},
	{.name = "r24", .offset = PTREGS_OFFSET_REG(24)},
	{.name = "r25", .offset = PTREGS_OFFSET_REG(25)},
	{.name = "r26", .offset = PTREGS_OFFSET_REG(26)},
	{.name = "r27", .offset = PTREGS_OFFSET_REG(27)},
	{.name = "r28", .offset = PTREGS_OFFSET_REG(28)},
	{.name = "r29", .offset = PTREGS_OFFSET_REG(29)},
	{.name = "r30", .offset = PTREGS_OFFSET_REG(30)},
	{.name = "r31", .offset = PTREGS_OFFSET_REG(31)},
	{.name = "r32", .offset = PTREGS_OFFSET_REG(32)},
	{.name = "r33", .offset = PTREGS_OFFSET_REG(33)},
	{.name = "r34", .offset = PTREGS_OFFSET_REG(34)},
	{.name = "r35", .offset = PTREGS_OFFSET_REG(35)},
	{.name = "r36", .offset = PTREGS_OFFSET_REG(36)},
	{.name = "r37", .offset = PTREGS_OFFSET_REG(37)},
	{.name = "r38", .offset = PTREGS_OFFSET_REG(38)},
	{.name = "r39", .offset = PTREGS_OFFSET_REG(39)},
	{.name = "r40", .offset = PTREGS_OFFSET_REG(40)},
	{.name = "r41", .offset = PTREGS_OFFSET_REG(41)},
	{.name = "r42", .offset = PTREGS_OFFSET_REG(42)},
	{.name = "r43", .offset = PTREGS_OFFSET_REG(43)},
	{.name = "r44", .offset = PTREGS_OFFSET_REG(44)},
	{.name = "r45", .offset = PTREGS_OFFSET_REG(45)},
	{.name = "r46", .offset = PTREGS_OFFSET_REG(46)},
	{.name = "r47", .offset = PTREGS_OFFSET_REG(47)},
	{.name = "r48", .offset = PTREGS_OFFSET_REG(48)},
	{.name = "r49", .offset = PTREGS_OFFSET_REG(49)},
	{.name = "r50", .offset = PTREGS_OFFSET_REG(50)},
	{.name = "r51", .offset = PTREGS_OFFSET_REG(51)},
	{.name = "r52", .offset = PTREGS_OFFSET_REG(52)},
	{.name = "tp", .offset = PTREGS_OFFSET_TP},
	{.name = "sp", .offset = PTREGS_OFFSET_SP},
	{.name = "lr", .offset = PTREGS_OFFSET_LR},
	{.name = "pc", .offset = PTREGS_OFFSET_PC},
	{.name = "ex1", .offset = PTREGS_OFFSET_EX1},
	{.name = "faultnum", .offset = PTREGS_OFFSET_FAULTNUM},
	{.name = "flags", .offset = PTREGS_OFFSET_FLAGS},
	REG_OFFSET_END,
};

/**
 * regs_within_kernel_stack() - check the address in the stack
 * @regs:      pt_regs which contains kernel stack pointer.
 * @addr:      address which is checked.
 *
 * regs_within_kernel_stack() checks @addr is within the kernel stack page(s).
 * If @addr is within the kernel stack, it returns true. If not, returns false.
 */
bool regs_within_kernel_stack(struct pt_regs *regs, unsigned long addr)
{
	return ((addr & ~(THREAD_SIZE - 1))  ==
			(kernel_stack_pointer(regs) & ~(THREAD_SIZE - 1)));
}

/**
 * regs_query_register_offset() - query register offset from its name
 * @name:       the name of a register
 *
 * regs_query_register_offset() returns the offset of a register in struct
 * pt_regs from its name. If the name is invalid, this returns -EINVAL;
 */
int regs_query_register_offset(const char *name)
{
	const struct pt_regs_offset *roff;
	for (roff = regoffset_table; roff->name != NULL; roff++)
		if (!strcmp(roff->name, name))
			return roff->offset;
	return -EINVAL;
}

/**
 * regs_query_register_name() - query register name from its offset
 * @offset:     the offset of a register in struct pt_regs.
 *
 * regs_query_register_name() returns the name of a register from its
 * offset in struct pt_regs. If the @offset is invalid, this returns NULL;
 */
const char *regs_query_register_name(unsigned int offset)
{
	const struct pt_regs_offset *roff;
	for (roff = regoffset_table; roff->name != NULL; roff++)
		if (roff->offset == offset)
			return roff->name;
	return NULL;
}
