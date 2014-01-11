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
 * Allow using the PMC for data watchpoints within the kernel.
 */

#include <linux/init.h>
#include <asm/pmc.h>

static int string_to_mode(const char *str)
{
#ifdef __tilepro__
	if (strcmp(str, "pc") == 0)
		return 0;
	else if (strcmp(str, "rw") == 0)
		return 1;
	else if (strcmp(str, "r") == 0)
		return 2;
	else if (strcmp(str, "w") == 0 || strcmp(str, "default") == 0)
		return 3;
#else
	/* FIXME: Support the mem_op variants in the high WATCH_VA bits. */
#endif
	return -1;
}

static const char *mode_to_string(int mode)
{
#ifdef __tilepro__
	static const char modes[][4] = { "pc", "rw", "r", "w" };
	return modes[mode];
#else
	/* FIXME: Support the mem_op variants in the high WATCH_VA bits. */
	return "default";
#endif
}

int handle_watch_interrupt(struct pt_regs *regs, int fault)
{
	/* Rewrite status register to clear set bits. */
	unsigned long status = pmc_get_overflow();
	pmc_ack_overflow(status);

	printk("Hit watchpoint somewhat before the following:\n");
	show_regs(regs);

	__insn_mtspr(SPR_PERF_COUNT_0, -1);

	return 0;
}

static int __init setup_watchpoint(char *str)
{
	char *p;
	unsigned long va, width;
	int mode;

	va = simple_strtoul(str, &p, 0);
	if (p == str) {
		pr_err("Unknown 'watch=' numeric value: %s\n", str);
		return -EINVAL;
	}
	str = p;
	if (*str == '/') {
		++str;
		width = simple_strtoul(str, &p, 0);
		if (p == str) {
			pr_err("Unknown 'watch=' width qualifier: %s\n", str);
			return -EINVAL;
		}
		if (__builtin_popcount(width) != 1) {
			pr_err("'watch=' width value not power of two: %ld\n",
			       width);
			return -EINVAL;
		}
		str = p;
	} else {
		width = 1;
	}
	if (*str == ',') {
		mode = string_to_mode(++str);
		if (mode == -1) {
			pr_err("Unknown 'watch=' qualifier '%s'\n", str);
			return -EINVAL;
		}
	} else {
		mode = string_to_mode("default");
	}

	/* FIXME: we only need to reserve PERF_COUNT_0. */
	if (reserve_pmc_hardware(handle_watch_interrupt))
		return -EINVAL;

	pr_notice("Watching VAs %#lx..%#lx,%s\n",
		  va, va + width - 1, mode_to_string(mode));

	__insn_mtspr(SPR_WATCH_VAL, va);

#ifdef __tilegx__
	/* Watch only the VA bits, not the high bits. */
	__insn_mtspr(SPR_WATCH_MASK, (-1ULL << 42) | (width - 1));

	/*
	 * We assume SPR_DIAG_MUX_CTL is set for Mbox output in the hv.
	 * Select "special" for perf counter zero, using "watch SPR" output.
	 */
	__insn_mtspr(SPR_PERF_COUNT_CTL, (7 << 6) | 0);
#else
	/* Watch requested VA bits. */
	__insn_mtspr(SPR_WATCH_MASK, width - 1);
	/* Choose watching the appropriate type of access. */
	__insn_mtspr(SPR_WATCH_CTL, mode);
	/* Select DIAG_SPCL_EVENT_WATCH for performance counter 0. */
	__insn_mtspr(SPR_PERF_COUNT_CTL, 106);
#endif

	__insn_mtspr(SPR_PERF_COUNT_0, -1);
	unmask_pmc_interrupts();

	return 0;
}
early_param("watch", setup_watchpoint);
