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

#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/irqflags.h>
#ifdef CONFIG_KVM_GUEST
#include <linux/virtio_console.h>
#include <linux/kvm_para.h>
#include <asm/kvm_virtio.h>
#endif
#include <asm/setup.h>
#include <hv/hypervisor.h>

static void early_hv_write(struct console *con, const char *s, unsigned n)
{
#ifdef CONFIG_KVM_GUEST
	char buf[512];

	if (n > sizeof(buf) - 1)
		n = sizeof(buf) - 1;
	memcpy(buf, s, n);
	buf[n] = '\0';

	hcall_virtio(KVM_VIRTIO_NOTIFY, __pa(buf));
#else
	tile_console_write(s, n);

	/*
	 * Convert NL to NLCR (close enough to CRNL) during early boot.
	 * We assume newlines are at the ends of strings, which turns out
	 * to be good enough for early boot console output.
	 */
	if (n && s[n-1] == '\n')
		tile_console_write("\r", 1);
#endif
}

static struct console early_hv_console = {
	.name =		"earlyhv",
	.write =	early_hv_write,
	.flags =	CON_PRINTBUFFER | CON_BOOT,
	.index =	-1,
};

/* Direct interface for emergencies */
static struct console *early_console = &early_hv_console;
static int early_console_initialized;

static void early_vprintk(const char *fmt, va_list ap)
{
	char buf[512];
#ifdef CONFIG_KVM_GUEST
	vscnprintf(buf, sizeof(buf), fmt, ap);
	hcall_virtio(KVM_VIRTIO_NOTIFY, __pa(buf));
#else
	int n = vscnprintf(buf, sizeof(buf), fmt, ap);
	early_console->write(early_console, buf, n);
#endif
}

void early_printk(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	early_vprintk(fmt, ap);
	va_end(ap);
}

void early_panic(const char *fmt, ...)
{
	va_list ap;
	arch_local_irq_disable_all();
	va_start(ap, fmt);
	early_printk("Kernel panic - not syncing: ");
	early_vprintk(fmt, ap);
	early_printk("\n");
	va_end(ap);
	dump_stack();
	hv_halt();
}

static int __init setup_early_printk(char *str)
{
	if (early_console_initialized)
		return 1;

	early_console = &early_hv_console;
	early_console_initialized = 1;
	register_console(early_console);

	return 0;
}

early_param("earlyprintk", setup_early_printk);
