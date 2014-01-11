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

#ifndef _ASM_UPROBES_H
#define _ASM_UPROBES_H

#include <linux/notifier.h>
#include <arch/opcode.h>

typedef tile_bundle_bits uprobe_opcode_t;

#define UPROBE_SWBP_INSN	TILE_BPT_BUNDLE
#define UPROBE_SWBP_INSN_SIZE	TILE_BUNDLE_SIZE_IN_BYTES
#define UPROBE_XOL_SLOT_BYTES	TILE_BUNDLE_SIZE_IN_BYTES
#define MAX_UINSN_BYTES		TILE_BUNDLE_SIZE_IN_BYTES

struct arch_uprobe {
	union {
		u8      insn[MAX_UINSN_BYTES];
		u64     ainsn;
	};
	u8	fixups;
};

struct arch_uprobe_task {
	unsigned long   saved_trap_nr;
};

extern int  arch_uprobe_analyze_insn(struct arch_uprobe *aup, struct mm_struct *mm);
extern int  arch_uprobe_pre_xol(struct arch_uprobe *aup, struct pt_regs *regs);
extern int  arch_uprobe_post_xol(struct arch_uprobe *aup, struct pt_regs *regs);
extern bool arch_uprobe_xol_was_trapped(struct task_struct *tsk);
extern int  arch_uprobe_exception_notify(struct notifier_block *self, unsigned long val, void *data);
extern void arch_uprobe_abort_xol(struct arch_uprobe *aup, struct pt_regs *regs);
#endif /* _ASM_UPROBES_H */
