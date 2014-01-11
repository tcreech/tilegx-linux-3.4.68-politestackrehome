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

#include <linux/smp.h>
#include <linux/seq_file.h>
#include <linux/threads.h>
#include <linux/cpumask.h>
#include <linux/timex.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#include <linux/pci.h>

#include <linux/sysctl.h>
#include <linux/hardirq.h>
#include <linux/hugetlb.h>
#include <linux/mman.h>
#include <asm/unaligned.h>
#include <asm/pgtable.h>
#include <asm/processor.h>
#include <asm/pmc.h>
#include <asm/sections.h>
#include <asm/homecache.h>
#include <asm/hardwall.h>
#include <arch/chip.h>


/*
 * Support /proc/cpuinfo
 */

#define cpu_to_ptr(n) ((void *)((long)(n)+1))
#define ptr_to_cpu(p) ((long)(p) - 1)

static int show_cpuinfo(struct seq_file *m, void *v)
{
	int n = ptr_to_cpu(v);

	if (n == 0) {
		char buf[NR_CPUS*5];
		cpulist_scnprintf(buf, sizeof(buf), cpu_online_mask);
		seq_printf(m, "cpu count\t: %d\n", num_online_cpus());
		seq_printf(m, "cpu list\t: %s\n", buf);
		seq_printf(m, "model name\t: %s\n", chip_model);
		seq_printf(m, "flags\t\t:\n");  /* nothing for now */
		seq_printf(m, "cpu MHz\t\t: %llu.%06llu\n",
			   get_clock_rate() / 1000000,
			   (get_clock_rate() % 1000000));
		seq_printf(m, "bogomips\t: %lu.%02lu\n\n",
			   loops_per_jiffy/(500000/HZ),
			   (loops_per_jiffy/(5000/HZ)) % 100);
	}

#ifdef CONFIG_SMP
	if (!cpu_online(n))
		return 0;
#endif

	seq_printf(m, "processor\t: %d\n", n);

	/* Print only num_online_cpus() blank lines total. */
	if (cpumask_next(n, cpu_online_mask) < nr_cpu_ids)
		seq_printf(m, "\n");

	return 0;
}

static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < nr_cpu_ids ? cpu_to_ptr(*pos) : NULL;
}
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return c_start(m, pos);
}
static void c_stop(struct seq_file *m, void *v)
{
}
const struct seq_operations cpuinfo_op = {
	.start	= c_start,
	.next	= c_next,
	.stop	= c_stop,
	.show	= show_cpuinfo,
};


/*
 * Support /proc/PID/pgtable
 */

struct addr_marker {
	unsigned long start_address;
	const char *name;
};

/* Address space markers */
static struct addr_marker address_markers[] = {
	{ PAGE_OFFSET, "Low Kernel Mapping" },
#ifdef __tilegx__
	{ _VMALLOC_START, "vmalloc() Area" },
	{ MEM_SV_START, "Static Kernel Code" },
	{ MEM_MODULE_START, "Kernel Modules" },
#else
	{ 0, "vmalloc() Area" },
# ifdef CONFIG_HIGHMEM
	{ 0, "Persistent kmap() Area" },
# endif
	{ 0, "Fixmap Area" },
	{ MEM_SV_START, "Static Kernel Code" },
#endif
	{ -1, NULL }		/* End of list */
};

#ifndef __tilegx__
/* Address markers are not compile-time constants on 32-bit platforms. */
static int __init address_markers_init(void)
{
	struct addr_marker *marker = &address_markers[1];
	(marker++)->start_address = _VMALLOC_START;
#ifdef CONFIG_HIGHMEM
	(marker++)->start_address = PKMAP_BASE;
#endif
	(marker++)->start_address = FIXADDR_START;

	return 0;
}
arch_initcall(address_markers_init);
#endif

int arch_proc_pgtable_show(struct seq_file *m, struct mm_struct *mm,
			   unsigned long vaddr, unsigned long size,
			   pte_t *ptep, void **datap)
{
	pte_t pte = *ptep;
	unsigned long pfn = pte_pfn(pte);
	struct addr_marker *marker;

#ifdef CONFIG_HUGETLB_SUPER_PAGES
	/*
	 * It's awkward to try to express this in the caller, which
	 * is a nominally architecture-independent page table walker,
	 * so we just allow it to look at all the non-present PTEs that
	 * follow a "super" PTE in the page table, and don't worry about it.
	 */
	if (pte_super(pte)) {
		switch (size) {
		case PGDIR_SIZE: size <<= huge_shift[HUGE_SHIFT_PGDIR]; break;
		case PMD_SIZE: size <<= huge_shift[HUGE_SHIFT_PMD]; break;
		case PAGE_SIZE: size <<= huge_shift[HUGE_SHIFT_PAGE]; break;
		default:
			pr_err("Impossible super PTE %#016llx size %#lx\n",
			       pte.val, size);
		}
	}
#endif

	/*
	 * We use %08 as the format here to match /proc/self/maps,
	 * which does this regardless of the underlying size of "long".
	 */
	seq_printf(m, "%08lx %c%c%c", vaddr,
		   hv_pte_get_readable(pte) ?
		   (hv_pte_get_accessed(pte) ? 'R' : 'r') : '-',
		   hv_pte_get_writable(pte) ?
		   (hv_pte_get_dirty(pte) ? 'W' : 'w') : '-',
		   hv_pte_get_executable(pte) ? 'X' : '-');
	seq_printf(m, " PA=%010llx (N%d)", PFN_PHYS(pfn), pfn_to_nid(pfn));
	if (!hv_pte_get_present(pte))
		seq_printf(m, " NotPresent");
	if (size != PAGE_SIZE) {
		if (size >= 1024*1024*1024)
			seq_printf(m, " Huge=%ldG", size >> 30);
		else if (size >= 1024*1024)
			seq_printf(m, " Huge=%ldM", size >> 20);
		else
			seq_printf(m, " Huge=%ldK", size >> 10);
	}
	if (hv_pte_get_migrating(pte))
		seq_printf(m, " Migrating");
	if (hv_pte_get_cached_priority(pte))
		seq_printf(m, " Priority");
	if (hv_pte_get_global(pte))
		seq_printf(m, " Global");
	if (!hv_pte_get_user(pte))
		seq_printf(m, " Kernel");

	/*
	 * If no caching modes are enabled, show "CacheNone",
	 * otherwise show the details of what caching there is.
	 */
	if (hv_pte_get_mode(pte) == HV_PTE_MODE_UNCACHED) {
		seq_printf(m, " CacheNone\n");
		return 0;
	}

	if (hv_pte_get_no_alloc_l1(pte) && hv_pte_get_no_alloc_l2(pte))
		seq_printf(m, " NoLocal");
	else if (hv_pte_get_no_alloc_l1(pte))
		seq_printf(m, " NoL1");
	else if (hv_pte_get_no_alloc_l2(pte))
		seq_printf(m, " NoL2");

	switch (hv_pte_get_mode(pte)) {
	case HV_PTE_MODE_CACHE_NO_L3:
		seq_printf(m, " NoHome");
		break;

	case HV_PTE_MODE_CACHE_TILE_L3:
		seq_printf(m, " Home=%d", get_remote_cache_cpu(pte));
		if (hv_pte_get_nc(pte))
			seq_printf(m, " NC");
		break;

	case HV_PTE_MODE_CACHE_HASH_L3:
		seq_printf(m, " HashHome");
		if (hv_pte_get_nc(pte))
			seq_printf(m, " NC");
		break;

#if CHIP_HAS_MMIO()
	case HV_PTE_MODE_MMIO:
		seq_printf(m, " MMIO");
		break;
#endif

	case 0:
		/* Special case 0, since it often means a cleared PTE. */
		break;

	default:
		seq_printf(m, " UnknownMode_%d", hv_pte_get_mode(pte));
		break;
	}

	if (vaddr >= PAGE_OFFSET) {
		marker = (struct addr_marker *)*datap;
		if (marker == NULL)
			marker = address_markers;
		if (vaddr >= marker->start_address) {
			while (vaddr >= marker[1].start_address)
				++marker;
			seq_printf(m, "  # %s", marker->name);
			++marker;
		}
		*datap = marker;
	}

	seq_printf(m, "\n");
	return 0;
}


/*
 * Support /proc/tile directory
 */


/*
 * Define a /proc/tile file which uses a seq_file to provide a more
 * complex set of data.
 */
#define SEQ_PROC_ENTRY(name)						\
	static int proc_tile_##name##_open(struct inode *inode,		\
					   struct file *file)		\
	{								\
		return single_open(file, proc_tile_##name##_show, NULL); \
	}								\
	static const struct file_operations proc_tile_##name##_fops = { \
		.open		= proc_tile_##name##_open,		\
		.read		= seq_read,				\
		.llseek		= seq_lseek,				\
		.release	= single_release,			\
	};								\
	static void proc_tile_##name##_init(struct proc_dir_entry *root) \
	{								\
		struct proc_dir_entry *entry =				\
			create_proc_entry(#name, 0444, root);		\
		if (entry)						\
			entry->proc_fops = &proc_tile_##name##_fops;	\
	}

static int proc_tile_environment_show(struct seq_file *sf, void *v)
{
	long cpu_temp = hv_sysconf(HV_SYSCONF_CPU_TEMP);
	long board_temp = hv_sysconf(HV_SYSCONF_BOARD_TEMP);

	if (cpu_temp < 0)
		seq_printf(sf, "chip_temp: unknown\n");
	else
		seq_printf(sf, "chip_temp: %ld\n",
			   cpu_temp - HV_SYSCONF_TEMP_KTOC);

	if (board_temp < 0)
		seq_printf(sf, "board_temp: unknown\n");
	else
		seq_printf(sf, "board_temp: %ld\n",
			   board_temp - HV_SYSCONF_TEMP_KTOC);

	return 0;
}
SEQ_PROC_ENTRY(environment)

static unsigned int dimm_size(unsigned char dimm_info)
{
	unsigned char size;

	if (((dimm_info >> HV_DIMM_TYPE_SHIFT ) &
		HV_DIMM_TYPE_MASK) == NO_DIMM)
		return 0;

	size = (dimm_info >> HV_DIMM_SIZE_SHIFT ) & HV_DIMM_SIZE_MASK;
	return (1 << (size + HV_MSH_MIN_DIMM_SIZE_SHIFT - 20));
}

static int proc_tile_memory_show(struct seq_file *sf, void *v)
{
	int node;
	int ctrl;
	int dimms;
	HV_Coord coord = { 0, 0 };
	/*
	 * We make two passes here; one through our memnodes to display
	 * which controllers they correspond with, and one through all
	 * controllers to get their speeds.  We may not actually have
	 * access to all of the controllers whose speeds we retrieve, but
	 * we get them because they're useful for mcstat, which provides
	 * stats for physical controllers whether we're using them or not.
	 */
	for (node = 0; node < MAX_NUMNODES; node++) {
		ctrl = node_controller[node];
		if (ctrl >= 0)
			seq_printf(sf, "controller_%d_node: %d\n", ctrl, node);
	}
	/*
	 * Note that we use MAX_NUMNODES as the limit for the controller
	 * loop because we don't have anything better.
	 */
	for (ctrl = 0; ctrl < MAX_NUMNODES; ctrl++) {
		HV_MemoryControllerInfo info =
			hv_inquire_memory_controller(coord, ctrl);
		if (info.speed) {
			seq_printf(sf, "controller_%d_speed: %llu\n",
				   ctrl, info.speed);
			for (dimms = 0; dimms < HV_MSH_MAX_DIMMS; dimms++) {
				unsigned char dimm_info = info.dimm[dimms];
				if (!dimm_info)
					continue;
				seq_printf(sf,
					   "controller_%d_dimm_%d_mb: %u\n",
					   ctrl, dimms, dimm_size(dimm_info));
			}
		}
	}
	return 0;
}
SEQ_PROC_ENTRY(memory)

static int proc_tile_interrupts_show(struct seq_file *sf, void *v)
{
	int i;

	seq_printf(sf, "%-4s%9s%9s%9s%9s%9s%9s%9s\n", "",
		   "timer", "syscall", "resched", "hvflush", "tlbflush",
		   "SMPcall", "devintr");

	for_each_online_cpu(i) {
		irq_cpustat_t *irq = &per_cpu(irq_stat, i);
		seq_printf(sf, "%-4d %8lu %8lu %8lu %8lu %8lu %8lu %8lu\n", i,
			   irq->irq_timer_count,
			   irq->irq_syscall_count,
			   irq->irq_resched_count,
			   irq->irq_hv_flush_count,
			   irq->irq_tlb_flush_count,
			   irq->irq_call_count,
			   irq->irq_dev_intr_count);
	}
	return 0;
}

/* Reset the interrupt counts on every tile. */
static ssize_t proc_tile_interrupts_write(struct file *file,
					  const char __user *buf,
					  size_t count, loff_t *offs)
{
	int i;
	for_each_online_cpu(i) {
		irq_cpustat_t *irq = &per_cpu(irq_stat, i);
		irq->irq_timer_count = 0;
		irq->irq_syscall_count = 0;
		irq->irq_resched_count = 0;
		irq->irq_hv_flush_count = 0;
		irq->irq_tlb_flush_count = 0;
		irq->irq_call_count = 0;
		irq->irq_dev_intr_count = 0;
	}
	return count;
}

/* Provide a specialized SEQ_PROC_ENTRY() expansion so we can write. */
static int proc_tile_interrupts_open(struct inode *inode,
				     struct file *file)
{
	return single_open(file, proc_tile_interrupts_show, NULL);
}

static const struct file_operations proc_tile_interrupts_fops = {
	.open		= proc_tile_interrupts_open,
	.read		= seq_read,
	.write		= proc_tile_interrupts_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void proc_tile_interrupts_init(struct proc_dir_entry *root)
{
	struct proc_dir_entry *entry =
		create_proc_entry("interrupts", 0644, root);
	if (entry)
		entry->proc_fops = &proc_tile_interrupts_fops;
}

#if defined(CONFIG_PCI) && defined(__tilepro__)
/* Simple /proc/tile/scan_pci file to support PCIe hot-plug. */
static ssize_t scan_pci_read(struct file *filp, char __user *buf, size_t size,
			     loff_t *ppos)
{
	char tmpbuf[8] = { 0 };
	char *format = "%u\n";

	ssize_t length = scnprintf(tmpbuf, sizeof(tmpbuf), format, NULL);
	return simple_read_from_buffer(buf, size, ppos, tmpbuf, length);
}
static ssize_t scan_pci_write(struct file *filp, const char __user *buf, 
			      size_t size, loff_t *ppos)
{
	int num_pcie_controllers = 0;
	
	/* Do PCIe re-init. */
	num_pcie_controllers = tile_pci_init();

	/* Do bus rescan if we have at least one controller. */
	if (num_pcie_controllers > 0)
		pcibios_init();

	return size;
}
static struct file_operations proc_tile_scan_pci_fops = {
	.read		= scan_pci_read,
	.write		= scan_pci_write,
};
static void proc_tile_scan_pci_init(struct proc_dir_entry *root)
{
	struct proc_dir_entry *entry =
		create_proc_entry("scan_pci", 0644, root);
	if (entry)
		entry->proc_fops = &proc_tile_scan_pci_fops;
}
#else
static inline void proc_tile_scan_pci_init(struct proc_dir_entry *root) {}
#endif

u64 arch_irq_stat_cpu(unsigned int cpu)
{
	irq_cpustat_t *irq = &per_cpu(irq_stat, cpu);
	u64 sum = 0;

	/* We do not count irq_syscall_count as an irq for /proc/stat. */

	sum += irq->irq_timer_count;
	sum += irq->irq_resched_count;
	sum += irq->irq_hv_flush_count;
	sum += irq->irq_tlb_flush_count;
	sum += irq->irq_call_count;
	sum += irq->irq_dev_intr_count;

	return sum;
}


static int __init proc_tile_init(void)
{
	struct proc_dir_entry *root = proc_mkdir("tile", NULL);
	if (root == NULL)
		return 0;

	proc_tile_hardwall_init(root);

	proc_tile_environment_init(root);
	proc_tile_memory_init(root);
	proc_tile_interrupts_init(root);
	proc_tile_scan_pci_init(root);


	return 0;
}

arch_initcall(proc_tile_init);

/*
 * Support /proc/sys/tile directory
 */

static ctl_table unaligned_subtable[] = {
	{
		.procname	= "enabled",
		.data		= &unaligned_fixup,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.procname	= "printk",
		.data		= &unaligned_printk,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.procname	= "count",
		.data		= &unaligned_fixup_count,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{}
};

static ctl_table unaligned_table[] = {
	{
		.procname	= "unaligned_fixup",
		.mode		= 0555,
		.child		= unaligned_subtable
	},
	{}
};


static ctl_table hash_default_table[] = {
	{
		.procname	= "hash_default",
		.data		= &uheap_hash,
		.maxlen		= sizeof(int),
		.mode		= 0444,
		.proc_handler	= &proc_dointvec
	},
	{}
};


#ifdef CONFIG_USE_PMC
/*
 * Support for /proc/sys/tile/userspace_perf_counters
 */
static unsigned int zero = 0;
static unsigned int one = 1;

ctl_table userspace_perf_counters_table[] = {
	{
		.procname = "userspace_perf_counters",
		.data = &userspace_perf_counters,
		.maxlen = sizeof(unsigned),
		.mode = 0644,
		.proc_handler = userspace_perf_counters_handler,
		.extra1		= &zero,
		.extra2		= &one,
	},
	{}
};
#endif

static struct ctl_path tile_path[] = {
	{ .procname = "tile" },
	{ }
};

static int __init proc_sys_tile_init(void)
{

	register_sysctl_paths(tile_path, unaligned_table);
#ifdef CONFIG_USE_PMC
	register_sysctl_paths(tile_path, userspace_perf_counters_table);
#endif

	/*
	 * Register this special file (which always has value "1")
	 * only if we are actually in this mode, so we just need
	 * to "stat" the file to perform the check.
	 */
	if (uheap_hash)
		register_sysctl_paths(tile_path, hash_default_table);

	return 0;
}

arch_initcall(proc_sys_tile_init);
