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
 * This code maintains the "home" for each page in the system.
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/bootmem.h>
#include <linux/rmap.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/sysctl.h>
#include <linux/pagevec.h>
#include <linux/ptrace.h>
#include <linux/timex.h>
#include <linux/cache.h>
#include <linux/smp.h>
#include <linux/module.h>
#include <linux/hugetlb.h>
#include <linux/kallsyms.h>
#include <linux/kthread.h>
#include <linux/nmi.h>

#include <asm/page.h>
#include <asm/sections.h>
#include <asm/tlbflush.h>
#include <asm/pgalloc.h>
#include <asm/homecache.h>
#ifdef CONFIG_DATAPLANE
#include <asm/dataplane.h>
#endif

#include <arch/sim.h>

#include "migrate.h"

#ifdef CONFIG_HOMECACHE

#define DEBUG  /* Define to enable debugging. */
#ifdef DEBUG

/* 8-byte structure stored in a circular buffer. */
struct homecache_event {
	u8 gen:1;        /* Generation for buffer-wrapping */
	u8 pad:7;
	u8 type;         /* Event type (see below) */
	u8 cpu;          /* CPU event was generated on */
	u8 home;         /* For migration events, the new home */
	u32 pfn;         /* The page being managed */
};

#define HE_INVALID	0  /* Empty slot */
#define HE_PAGE		1  /* User page migrating */
#define HE_KSTACK	2  /* Kernel stack migrating */
#define HE_KSTACK_DONE	3  /* Kernel stack migration complete */
#define HE_KKSTACK	4  /* Kernel thread's stack migrating */
#define HE_KKSTACK_DONE	5  /* Kernel thread's stack migration complete */
#define HE_UNMAPPED	6  /* Unmapped page home change */
#define HE_TTU_START	7  /* Mapped page try_to_unmap start */
#define HE_MAPPED	8  /* Mapped page fixing kernel PTE */
#define HE_TTU_FINISH	9  /* Mapped page try_to_unmap finish */

#define HE_TYPES	10  /* One more than the largest valid value */

static const char *he_names[] = {
	[HE_INVALID] = "invalid",
	[HE_PAGE] = "process_page",
	[HE_KSTACK] = "kstack",
	[HE_KSTACK_DONE] = "kstack_done",
	[HE_KKSTACK] = "kkstack",
	[HE_KKSTACK_DONE] = "kkstack_done",
	[HE_UNMAPPED] = "unmapped",
	[HE_TTU_START] = "ttu_start",
	[HE_MAPPED] = "mapped",
	[HE_TTU_FINISH] = "ttu_finish",
};

/* Configurable size of history buffer (in units of events). */
#define HE_BUFSHIFT		13
#define HE_BUFSIZE		(1UL << HE_BUFSHIFT)
#define HE_BUFMASK		(HE_BUFSIZE - 1)

/* Circular buffer of homecache events. */
static struct homecache_event he_buf[HE_BUFSIZE];

/*
 * Pointer to next event to fill in, mod HE_BUFSIZE.
 * We use the bit at HE_BUFSIZE itself as a generation count
 * to disambiguate one pass through the buffer from the next.
 */
static atomic_t he_index = ATOMIC_INIT(HE_BUFSIZE);

/*
 * Context information for when a cpu is doing migration.
 * If frames[0] is zero, then the context is invalid.
 */
#define HE_FRAMES 32
struct he_context {
	unsigned long frames[HE_FRAMES];
};

static DEFINE_PER_CPU(struct he_context, he_context);

/*
 * Add a new entry.  We use atomic increment to get this core a slot
 * it can use, and write the generation bit according to how many times
 * we've been around the buffer.  That way we know when to stop when
 * we are unwinding the buffer backwards.
 */
static void he_add(int type, int home, u32 pfn)
{
	unsigned int i = atomic_inc_return(&he_index) - 1;
	unsigned int gen = (i >> HE_BUFSHIFT) & 1;
	int cpu = raw_smp_processor_id();
	struct homecache_event he = { gen, 0, type, cpu, home, pfn };
	he_buf[i & HE_BUFMASK] = he;
}

/*
 * Record our current backtrace information.  Since we don't need to
 * detect leaf function frames or follow userspace stacks, we use a
 * very simple algorithm to capture the stack efficiently and safely.
 *
 * FIXME: This is not preemption-safe, the he_context should be per-
 * thread, so we should put it in the thread info, rather than per-cpu.
 * Given that it is just debugging code, we don't really care much.
 */
static noinline void he_begin(void)
{
	unsigned long *frames = (*__this_cpu_ptr(&he_context)).frames;
	unsigned long top =
		(stack_pointer & -THREAD_SIZE) + THREAD_SIZE - STACK_TOP_DELTA;
	unsigned long fp, lr, sp;
	int i;

	fp = (unsigned long)__builtin_frame_address(0);
	BUG_ON(fp > top || fp < (stack_pointer & -THREAD_SIZE) ||
	       (fp & (sizeof(long) - 1)) != 0);
	for (i = 0; i < HE_FRAMES; ) {
		lr = ((unsigned long *)fp)[0];
		sp = ((unsigned long *)fp)[1];
		frames[i++] = lr;
		if (sp <= fp || sp > top || (sp & (sizeof(long) - 1)) != 0)
			break;
		fp = sp;
	}
	if (i < HE_FRAMES)
		frames[i] = 0;
}

/* Reset the context to empty. */
static void he_end(void)
{
	unsigned long *frames = (*__this_cpu_ptr(&he_context)).frames;
	frames[0] = 0;
}

/* Dump out a symbolic backtrace of the given cpu, if it's in homecache. */
static void he_show_backtrace(int cpu)
{
	char buf[KSYM_NAME_LEN];
	char *modname;
	const char *name;
	unsigned long size, offset, address;
	unsigned long *frames = per_cpu(he_context, cpu).frames;
	int i;

	address = frames[0];
	if (address == 0)
		return;

	pr_err("Backtrace for cpu %d:\n", cpu);
	for (i = 0; address != 0; ) {
		pr_err("%2d: %#lx", i, address);
		name = kallsyms_lookup(address - 1, &size, &offset,
				       &modname, buf);
		if (name != NULL) {
			pr_cont(" %s+%#lx/%#lx", name, offset + 1, size);
			if (modname)
				pr_cont(" [%s]", modname);
		}
		pr_cont("\n");
		if (++i >= HE_FRAMES) {
			pr_err("...\n");
			break;
		}
		address = frames[i];
	}
	pr_err("Done.\n");
}

/* Print an event at a particular (negative) index in the history. */
static void he_print(int index, struct homecache_event *he)
{
	const char *event =
		(he->type < HE_TYPES) ? he_names[he->type] : "illegal";
	pr_err("%d: cpu %d: %s -> %d: pfn %#x\n",
	       index, he->cpu, event, he->home, he->pfn);
}

/*
 * Do we try to continue after encountering a stuck migrating PTE?
 * This is never a good idea and only offered here to support the
 * scenario where we want to have time to cat /proc/kmsg over a
 * network connection before rebooting the machine.
 */
static bool migrating_hang_continue;

static int __init migrating_hang(char *str)
{
	if (strcmp(str, "continue") == 0) {
		pr_notice("Continuing after migrating PTE hang.\n");
		migrating_hang_continue = true;
		return 1;
	}
	pr_err("migrating_hang=%s: invalid option\n", str);
	return 0;
}
__setup("migrating_hang=", migrating_hang);

void homecache_debug_stuck_page(unsigned long address, pte_t *pte)
{
	static DEFINE_SPINLOCK(print_lock);
	unsigned long pfn = pte_pfn(*pte);
	unsigned int i;
	int j, cpu;

	/* Lock to avoid intermixing print outputs. */
	spin_lock_irq(&print_lock);

	pr_err("cpu %d: Hit migrating PTE (%#llx) and page PFN"
	       " %#lx at VA %#lx still migrating",
	       smp_processor_id(), pte->val, pte_pfn(*pte), address);

	pr_err("Nearby PTEs:\n");
	for (j = -8; j <= 8; ++j)
		pr_err("%p: %016llx\n", pte + j, pte[j].val);

	i = atomic_read(&he_index) - HE_BUFSIZE;
	pr_err("Dumping event history buffer (events %d..%d)\n",
	       i, i + (int)HE_BUFSIZE - 1);

	/* Dump out all references to the page in question, oldest first. */
	for (j = 0; j < HE_BUFSIZE; ++j, ++i) {
		struct homecache_event *he = &he_buf[i & HE_BUFMASK];
		if (he->gen != ((i >> HE_BUFSHIFT) & 1))
			continue;
		if (he->pfn == pfn)
			he_print(-HE_BUFSIZE + j, he);
	}

	pr_err("Event history buffer dump complete.\n");

	/* Dump out backtraces for all cpus currently migrating pages. */
	pr_err("Dumping stacktraces of cores in homecache code:\n");
	for_each_cpu(cpu, cpu_online_mask)
		he_show_backtrace(cpu);

	pr_err("Done debug output.\n");

	trigger_all_cpu_backtrace();

	/* Panic at this point if we're not trying to continue. */
	BUG_ON(!migrating_hang_continue);

	/* Dump stack trace. */
	dump_stack();

	/* Bump the ref count on the migrating page so it's never freed. */
	get_page(pfn_to_page(pfn));

	/* Note we enable interrupts here as required for do_group_exit(). */
	spin_unlock_irq(&print_lock);

	/* Try to leave the system up, if possible. */
	do_group_exit(SIGKILL);
}

#else

#define he_begin()			do {} while (0)
#define he_end()			do {} while (0)
#define he_add(type, home, pfn)		do {} while (0)

#endif /* DEBUG */
#endif /* CONFIG_HOMECACHE */
#if !defined(CONFIG_HOMECACHE) || !defined(DEBUG)
void homecache_debug_stuck_page(unsigned long address, pte_t *pte)
{
	panic("Hit migrating PTE (%#llx) and page PFN"
	      " %#lx at VA %#lx still migrating",
	      pte->val, pte_pfn(*pte), address);
}
#endif
#ifdef CONFIG_HOMECACHE

/* Extra hook function in mm/page_alloc.c, just for homecache. */
extern struct page *homecache_rmqueue(struct zone *, unsigned int order,
				      int migratetype);

/* Forward declarations. */
static int homecache_get_desired_home(int home, pgprot_t, int writable);

/* Various statistics exposed through /proc/sys/tile/homecache/. */

/* Pages sequestered at "free" time (e.g. immutable). */
static int homecache_proc_sequestered_free;

/* Times we have dumped sequestered coherent pages back to the allocator. */
static int homecache_proc_coherent_sequestered_purge;

/* Times we have done a chip-wide flush and unsequestered everything. */
static int homecache_proc_sequestered_purge;

/* Number of unmapped pages migrated (i.e. no PTEs yet). */
static int homecache_proc_migrated_unmapped;

/* Number of mapped pages we have migrated. */
static int homecache_proc_migrated_mapped;

/* Number of incoherent pages finv'ed. */
static int homecache_proc_incoherent_finv_pages;

/* Number of tasks that have been migrated. */
static int homecache_proc_migrated_tasks;

/* Number of sequestered pages allowed. */
static unsigned long homecache_proc_max_sequestered_pages = 1024;

#endif /* CONFIG_HOMECACHE */

/*
 * The noallocl2 option suppresses all use of the L2 cache to cache
 * locally from a remote home.
 */
static int __write_once noallocl2;
static int __init set_noallocl2(char *str)
{
	noallocl2 = 1;
	return 0;
}
early_param("noallocl2", set_noallocl2);

#ifdef CONFIG_HOMECACHE

/* Return the cache home of a page. */
int page_home(struct page *page)
{
	return page->home;
}
EXPORT_SYMBOL(page_home);

/*
 * Optimize access to page home in this file.  We use the function
 * definition to make it possible to use modules that reference
 * page_home() regardless of how the kernel is built.
 */
#define page_home(page)			((page)->home)

#endif /* CONFIG_HOMECACHE */
#ifdef CONFIG_DATAPLANE

static DEFINE_PER_CPU(atomic_t, remote_flush_counts);

/*
 * Update the number of remote TLB flushes targeting this cpu.
 *
 * The model is that when a core returns to dataplane userspace, it
 * first arranges for homecache_tlb_can_defer() to return true, and
 * then spins until the remote flush count is zero.  Once
 * homecache_tlb_can_defer() is true, no further increments will
 * be made to the set of cores waiting on a remote flush.
 *
 * Thus, when we are doing a flush, we first conservatively warn
 * all the cpus that we may be flushing them, and then figure out
 * which ones are currently in userspace and shouldn't actually
 * get flushed.  This may result in unnecessary atomic_add() calls
 * here, but that's relatively cheap.
 *
 * We use a barrier here and in dataplane_return_to_user() so we know
 * that the two routines can see each other's values in memory.
 */
static void set_remote_flush_count(const struct cpumask *mask, int delta)
{
	if (mask) {
		int cpu;
		for_each_cpu(cpu, mask)
			atomic_add(delta, &per_cpu(remote_flush_counts, cpu));
		smp_wmb();
	}
}

/* Return the number of remote TLB flushes targeting a cpu. */
int homecache_get_remote_flush_count(int cpu)
{
	return atomic_read(&per_cpu(remote_flush_counts, cpu));
}

/* State of TLB deferral on a dataplane cpu. */
static DEFINE_PER_CPU(atomic_t, dataplane_tlb_state);

/* Provide constants for dataplane TLB deferral. */
#define TLB_DEFER_KERNEL   0    /* in kernel space */
#define TLB_DEFER_USER     1    /* in user space */
#define TLB_DEFER_PENDING  2    /* in user space with a TLB flush pending */

/*
 * This routine is called on kernel entry from userspace for dataplane
 * tiles, so we can properly adjust our state to be TLB_DEFER_KERNEL,
 * and run a TLB flush if necessary.
 *
 * Note that using the atomic routines here implies that the atomic
 * locks are all available to us.  This is true only because if the
 * hypervisor recursively punches through ICS (when an atomic lock
 * might be held, e.g. for a cmpxchg() to vmalloc'ed memory), we will
 * see it very early in the interrupt handler path and call
 * do_page_fault_ics(), which will release the lock before continuing
 * on to the code that will invoke this routine.
 */
void homecache_tlb_defer_enter(void)
{
	atomic_t *state = &__get_cpu_var(dataplane_tlb_state);
	if (atomic_xchg(state, TLB_DEFER_KERNEL) == TLB_DEFER_PENDING) {
		unsigned long size = KERNEL_HIGH_VADDR - PAGE_OFFSET;
		int rc = hv_flush_pages(PAGE_OFFSET, PAGE_SIZE, size);
		rc |= hv_flush_pages(PAGE_OFFSET, HPAGE_SIZE, size);
		BUG_ON(rc != 0);
	}
}

/*
 * This routine is called on kernel exit to userspace for dataplane
 * tiles, so we can properly adjust our state to be TLB_DEFER_USER.
 */
void homecache_tlb_defer_exit(void)
{
	atomic_t *state = &__get_cpu_var(dataplane_tlb_state);

	/*
	 * Note that we could write directly to state->counter here
	 * instead of using the normal serializing atomic_set(),
	 * since we shouldn't be able to race with the other deferral
	 * routines any more than we already do.  But it makes it easier
	 * to reason about this code to use the serializing version,
	 * and I'm not concerned about performance in this context.
	 */
	atomic_set(state, TLB_DEFER_USER);
}

/*
 * This routine determines if we can defer a TLB flush.
 * It must be called with interrupts disabled.
 */
static int homecache_tlb_can_defer(int cpu)
{
	atomic_t *state = &per_cpu(dataplane_tlb_state, cpu);
	int old = atomic_cmpxchg(state, TLB_DEFER_USER, TLB_DEFER_PENDING);
	return (old != TLB_DEFER_KERNEL);
}

/*
 * Return whether the remote cpu is actively running dataplane code
 * and is not in the kernel.
 */
static int dataplane_hvflush_debug(int cpu)
{
	return cpu_online(cpu) && dataplane_debug(cpu) &&
		(atomic_read(&per_cpu(dataplane_tlb_state, cpu)) !=
		 TLB_DEFER_KERNEL);
}
#endif /* CONFIG_DATAPLANE */

/*
 * Update the irq_stat for cpus that we are going to interrupt
 * with TLB or cache flushes.  Also handle removing dataplane cpus
 * from the TLB flush set, and setting dataplane_tlb_state instead.
 */
static void hv_flush_update(unsigned long cache_control,
			    const struct cpumask *cache_cpumask,
			    struct cpumask *tlb_cpumask,
			    unsigned long tlb_va, unsigned long tlb_length,
			    HV_Remote_ASID *asids, int asidcount)
{
	struct cpumask tlb_stat, cache_stat;
	int i, cpu;
#ifdef CONFIG_DATAPLANE
	int dataplane_stack = 0;
#endif

	cpumask_clear(&tlb_stat);
	cpumask_clear(&cache_stat);

	if (tlb_cpumask && tlb_length) {
#ifdef CONFIG_DATAPLANE
		/*
		 * If we are doing TLB flushes to kernel addresses we
		 * check to see if there are any dataplane tiles that
		 * are currently running user code and if so, mark
		 * them as pending a TLB flush, and remove them from
		 * tlb_cpumask.  We only proceed with the flush on
		 * such tiles if they are in the kernel.
		 */
		if (tlb_va >= PAGE_OFFSET &&
		    tlb_va + tlb_length < KERNEL_HIGH_VADDR) {
			for_each_cpu_and(cpu, tlb_cpumask, &dataplane_map) {
				if (homecache_tlb_can_defer(cpu))
					cpumask_clear_cpu(cpu, tlb_cpumask);
			}
		}
#endif
		cpumask_or(&tlb_stat, &tlb_stat, tlb_cpumask);
	}

	for (i = 0; i < asidcount; ++i) {
		int cpu = xy_to_cpu(asids[i].x, asids[i].y);
#ifdef CONFIG_DATAPLANE
		if (dataplane_hvflush_debug(cpu)) {
			dataplane_stack = 1;
			pr_err("cpu %d: ASID flush to dataplane cpu %d\n",
			       raw_smp_processor_id(), cpu);
		}
#endif		
		cpumask_set_cpu(cpu, &tlb_stat);
	}

	if (cache_cpumask) {
		for_each_cpu(cpu, cache_cpumask) {
			unsigned long ctrl = cache_control;
			/*
			 * Don't count L1I flushes that accompany TLB
			 * flushes, since they're just part of our
			 * protocol for managing icache coherency.
			 */
			if ((ctrl & HV_FLUSH_EVICT_L1I) != 0 &&
			    cpumask_test_cpu(cpu, &tlb_stat))
				ctrl &= ~HV_FLUSH_EVICT_L1I;
			if (ctrl)
				cpumask_set_cpu(cpu, &cache_stat);
#ifdef CONFIG_DATAPLANE
			if (dataplane_hvflush_debug(cpu)) {
				dataplane_stack = 1;
				pr_err("cpu %d: cache flush to"
				       " dataplane cpu %d\n",
				       raw_smp_processor_id(), cpu);
			}
#endif
		}
	}

	/*
	 * Don't bother to update atomically; losing a count
	 * here is not that critical.
	 */
	for_each_cpu(cpu, &tlb_stat)
		++per_cpu(irq_stat, cpu).irq_tlb_flush_count;
	for_each_cpu(cpu, &cache_stat)
		++per_cpu(irq_stat, cpu).irq_hv_flush_count;

#ifdef CONFIG_DATAPLANE
	if (dataplane_stack)
		dump_stack();
#endif
}

/*
 * This wrapper function around hv_flush_remote() does several things:
 *
 *  - Provides a return value error-checking panic path, since
 *    there's never any good reason for hv_flush_remote() to fail.
 *  - Accepts a 32-bit PFN rather than a 64-bit PA, which generally
 *    is the type that Linux wants to pass around anyway.
 *  - Canonicalizes that lengths of zero make cpumasks NULL.
 *  - Handles deferring TLB flushes for dataplane tiles.
 *  - Tracks remote interrupts in the per-cpu irq_cpustat_t.
 *
 * Note that we have to wait until the cache flush completes before
 * updating the per-cpu last_cache_flush word, since otherwise another
 * concurrent flush can race, conclude the flush has already
 * completed, and start to use the page while it's still dirty
 * remotely (running concurrently with the actual evict, presumably).
 */
void flush_remote(unsigned long cache_pfn, unsigned long cache_control,
		  const struct cpumask *cache_cpumask_orig,
		  HV_VirtAddr tlb_va, unsigned long tlb_length,
		  unsigned long tlb_pgsize,
		  const struct cpumask *tlb_cpumask_orig,
		  HV_Remote_ASID *asids, int asidcount)
{
	int rc;
	struct cpumask cache_cpumask_copy, tlb_cpumask_copy;
	struct cpumask *cache_cpumask, *tlb_cpumask;
	HV_PhysAddr cache_pa;
	char cache_buf[NR_CPUS*5], tlb_buf[NR_CPUS*5];

	/* Ensure any PTEs set to migrating are visible everywhere. */
	smp_wmb();

	/*
	 * Canonicalize and copy the cpumasks.
	 */
	if (cache_cpumask_orig && cache_control) {
		cpumask_copy(&cache_cpumask_copy, cache_cpumask_orig);
		cache_cpumask = &cache_cpumask_copy;
	} else {
		cpumask_clear(&cache_cpumask_copy);
		cache_cpumask = NULL;
	}
	if (cache_cpumask == NULL)
		cache_control = 0;
	if (tlb_cpumask_orig && tlb_length) {
		cpumask_copy(&tlb_cpumask_copy, tlb_cpumask_orig);
		tlb_cpumask = &tlb_cpumask_copy;
	} else {
		cpumask_clear(&tlb_cpumask_copy);
		tlb_cpumask = NULL;
	}
#ifdef CONFIG_DATAPLANE
	if (tlb_length)
		set_remote_flush_count(tlb_cpumask_orig, +1);
#endif
	hv_flush_update(cache_control, cache_cpumask, tlb_cpumask,
			tlb_va, tlb_length, asids, asidcount);
	cache_pa = (HV_PhysAddr)cache_pfn << PAGE_SHIFT;
	rc = hv_flush_remote(cache_pa, cache_control,
			     cpumask_bits(cache_cpumask),
			     tlb_va, tlb_length, tlb_pgsize,
			     cpumask_bits(tlb_cpumask),
			     asids, asidcount);
#ifdef CONFIG_DATAPLANE
	if (tlb_length)
		set_remote_flush_count(tlb_cpumask_orig, -1);
#endif
	if (rc == 0)
		return;
	cpumask_scnprintf(cache_buf, sizeof(cache_buf), &cache_cpumask_copy);
	cpumask_scnprintf(tlb_buf, sizeof(tlb_buf), &tlb_cpumask_copy);

	pr_err("hv_flush_remote(%#llx, %#lx, %p [%s],"
	       " %#lx, %#lx, %#lx, %p [%s], %p, %d) = %d\n",
	       cache_pa, cache_control, cache_cpumask, cache_buf,
	       (unsigned long)tlb_va, tlb_length, tlb_pgsize,
	       tlb_cpumask, tlb_buf,
	       asids, asidcount, rc);
	panic("Unsafe to continue.");
}

static void homecache_finv_page_va(void* va, int home)
{
	int cpu = get_cpu();
	if (home == cpu) {
		finv_buffer_local(va, PAGE_SIZE);
	} else if (home == PAGE_HOME_HASH) {
		finv_buffer_remote(va, PAGE_SIZE, 1);
	} else {
		BUG_ON(home < 0 || home >= NR_CPUS);
		finv_buffer_remote(va, PAGE_SIZE, 0);
	}
	put_cpu();
}

/*
 * Map a given page with a given home.
 * Return the VA mapped at, and a pointer to the PTE being used to map it.
 */
static unsigned long homecache_map_begin(struct page *page, int home,
					 pte_t **ptepp)
{
	unsigned long va;
	pte_t *ptep;
	pte_t pte;

#ifdef CONFIG_HIGHMEM
	va = __fix_to_virt(FIX_KMAP_BEGIN + kmap_atomic_idx_push() +
			   (KM_TYPE_NR * smp_processor_id()));
#else
	va = __fix_to_virt(FIX_HOMECACHE_BEGIN + smp_processor_id());
#endif
	ptep = virt_to_kpte(va);
	pte = pfn_pte(page_to_pfn(page), PAGE_KERNEL);
	/* Use __set_pte() to allow writing the migrating bit. */
	__set_pte(ptep, pte_set_home(pte, home));
	*ptepp = ptep;
	return va;
}

/* Undo homecache_map_begin(). */
static void homecache_map_end(pte_t *ptep, unsigned long va)
{
	__pte_clear(ptep);
	hv_flush_page(va, PAGE_SIZE);
#ifdef CONFIG_HIGHMEM
	kmap_atomic_idx_pop();
#endif
}

void homecache_finv_map_page(struct page *page, int home)
{
	unsigned long flags;
	unsigned long va;
	pte_t *ptep;

	if (home == PAGE_HOME_UNCACHED)
		return;
	local_irq_save(flags);
	va = homecache_map_begin(page, home, &ptep);
	homecache_finv_page_va((void *)va, home);
	homecache_map_end(ptep, va);
	local_irq_restore(flags);
}

static void homecache_finv_page_home(struct page *page, int home)
{
	if (!PageHighMem(page) && home == page_home(page))
		homecache_finv_page_va(page_address(page), home);
	else
		homecache_finv_map_page(page, home);
}

static inline bool incoherent_home(int home)
{
	return home == PAGE_HOME_IMMUTABLE || home == PAGE_HOME_INCOHERENT;
}

static void homecache_finv_page_internal(struct page *page, int force_map)
{
	int home = page_home(page);
	if (home == PAGE_HOME_UNCACHED)
		return;
	if (incoherent_home(home)) {
		int cpu;
		for_each_cpu(cpu, &cpu_cacheable_map)
			homecache_finv_map_page(page, cpu);
#ifdef CONFIG_HOMECACHE
		++homecache_proc_incoherent_finv_pages;
#endif
	} else if (force_map) {
		/* Force if, e.g., the normal mapping is migrating. */
		homecache_finv_map_page(page, home);
	} else {
		homecache_finv_page_home(page, home);
	}
	sim_validate_lines_evicted(PFN_PHYS(page_to_pfn(page)), PAGE_SIZE);
}

void homecache_finv_page(struct page *page)
{
	homecache_finv_page_internal(page, 0);
}

void homecache_evict(const struct cpumask *mask)
{
	flush_remote(0, HV_FLUSH_EVICT_L2, mask, 0, 0, 0, NULL, NULL, 0);
}

/* Report the home corresponding to a given PTE. */
static int pte_to_home(pte_t pte)
{
	if (hv_pte_get_nc(pte))
		return PAGE_HOME_IMMUTABLE;
	switch (hv_pte_get_mode(pte)) {
	case HV_PTE_MODE_CACHE_TILE_L3:
		return get_remote_cache_cpu(pte);
	case HV_PTE_MODE_CACHE_NO_L3:
		return PAGE_HOME_INCOHERENT;
	case HV_PTE_MODE_UNCACHED:
		return PAGE_HOME_UNCACHED;
	case HV_PTE_MODE_CACHE_HASH_L3:
		return PAGE_HOME_HASH;
	}
	panic("Bad PTE %#llx\n", pte.val);
}

/* Update the home of a PTE if necessary (can also be used for a pgprot_t). */
pte_t pte_set_home(pte_t pte, int home)
{
	/* Check for non-linear file mapping "PTEs" and pass them through. */
	if (pte_file(pte))
		return pte;

#if CHIP_HAS_MMIO()
	/* Check for MMIO mappings and pass them through. */
	if (hv_pte_get_mode(pte) == HV_PTE_MODE_MMIO)
		return pte;
#endif


	/*
	 * Only immutable pages get NC mappings.  If we have a
	 * non-coherent PTE, but the underlying page is not
	 * immutable, it's likely the result of a forced
	 * caching setting running up against ptrace setting
	 * the page to be writable underneath.  In this case,
	 * just keep the PTE coherent.
	 */
	if (hv_pte_get_nc(pte) && home != PAGE_HOME_IMMUTABLE) {
		pte = hv_pte_clear_nc(pte);
		pr_err("non-immutable page incoherently referenced: %#llx\n",
		       pte.val);
	}

	switch (home) {

	case PAGE_HOME_UNCACHED:
		pte = hv_pte_set_mode(pte, HV_PTE_MODE_UNCACHED);
		break;

	case PAGE_HOME_INCOHERENT:
		pte = hv_pte_set_mode(pte, HV_PTE_MODE_CACHE_NO_L3);
		break;

	case PAGE_HOME_IMMUTABLE:
		/*
		 * We could home this page anywhere, since it's immutable,
		 * but by default just home it to follow "hash_default".
		 */
		BUG_ON(hv_pte_get_writable(pte));
		if (pte_get_forcecache(pte)) {
			/* Upgrade "force any cpu" to "No L3" for immutable. */
			if (hv_pte_get_mode(pte) == HV_PTE_MODE_CACHE_TILE_L3
			    && pte_get_anyhome(pte)) {
				pte = hv_pte_set_mode(pte,
						      HV_PTE_MODE_CACHE_NO_L3);
			}
		} else
		if (hash_default)
			pte = hv_pte_set_mode(pte, HV_PTE_MODE_CACHE_HASH_L3);
		else
			pte = hv_pte_set_mode(pte, HV_PTE_MODE_CACHE_NO_L3);
		pte = hv_pte_set_nc(pte);
		break;

	case PAGE_HOME_HASH:
		pte = hv_pte_set_mode(pte, HV_PTE_MODE_CACHE_HASH_L3);
		break;

	default:
		BUG_ON(home < 0 || home >= NR_CPUS ||
		       !cpu_is_valid_lotar(home));
		pte = hv_pte_set_mode(pte, HV_PTE_MODE_CACHE_TILE_L3);
		pte = set_remote_cache_cpu(pte, home);
		break;
	}

	if (noallocl2)
		pte = hv_pte_set_no_alloc_l2(pte);

	/* Simplify "no local and no l3" to "uncached" */
	if (hv_pte_get_no_alloc_l2(pte) && hv_pte_get_no_alloc_l1(pte) &&
	    hv_pte_get_mode(pte) == HV_PTE_MODE_CACHE_NO_L3) {
		pte = hv_pte_set_mode(pte, HV_PTE_MODE_UNCACHED);
	}

	/* Checking this case here gives a better panic than from the hv. */
	BUG_ON(hv_pte_get_mode(pte) == 0);

	return pte;
}
EXPORT_SYMBOL(pte_set_home);
#ifndef CONFIG_HOMECACHE

/*
 * The routines in this section are the "static" versions of the normal
 * dynamic homecaching routines; they just set the home cache
 * of a kernel page once, and require a full-chip cache/TLB flush,
 * so they're not suitable for anything but infrequent use.
 */

int page_home(struct page *page)
{
	if (PageHighMem(page)) {
		return PAGE_HOME_HASH;
	} else {
		unsigned long kva = (unsigned long)page_address(page);
		return pte_to_home(*virt_to_kpte(kva));
	}
}
EXPORT_SYMBOL(page_home);

void homecache_change_page_home(struct page *page, int order, int home)
{
	int i, pages = (1 << order);
	unsigned long kva;

	BUG_ON(PageHighMem(page));
	BUG_ON(page_count(page) > 1);
	BUG_ON(page_mapcount(page) != 0);
	kva = (unsigned long) page_address(page);
	flush_remote(0, HV_FLUSH_EVICT_L2, &cpu_cacheable_map,
		     kva, pages * PAGE_SIZE, PAGE_SIZE, cpu_online_mask,
		     NULL, 0);

	for (i = 0; i < pages; ++i, kva += PAGE_SIZE) {
		pte_t *ptep = virt_to_kpte(kva);
		pte_t pteval = *ptep;
		BUG_ON(!pte_present(pteval) || pte_huge(pteval));
		/* Use __set_pte to bypass home checking. */
		__set_pte(ptep, pte_set_home(pteval, home));
	}
}
EXPORT_SYMBOL(homecache_change_page_home);

struct page *homecache_alloc_pages(gfp_t gfp_mask,
				   unsigned int order, int home)
{
	struct page *page;
	BUG_ON(gfp_mask & __GFP_HIGHMEM);   /* must be lowmem */
	page = alloc_pages(gfp_mask, order);
	if (page)
		homecache_change_page_home(page, order, home);
	return page;
}
EXPORT_SYMBOL(homecache_alloc_pages);

struct page *homecache_alloc_pages_node(int nid, gfp_t gfp_mask,
					unsigned int order, int home)
{
	struct page *page;
	BUG_ON(gfp_mask & __GFP_HIGHMEM);   /* must be lowmem */
	page = alloc_pages_node(nid, gfp_mask, order);
	if (page)
		homecache_change_page_home(page, order, home);
	return page;
}

void __homecache_free_pages(struct page *page, unsigned int order)
{
	if (put_page_testzero(page)) {
		homecache_change_page_home(page, order, PAGE_HOME_HASH);
		if (order == 0) {
			free_hot_cold_page(page, 0);
		} else {
			init_page_count(page);
			__free_pages(page, order);
		}
	}
}
EXPORT_SYMBOL(__homecache_free_pages);

void homecache_free_pages(unsigned long addr, unsigned int order)
{
	if (addr != 0) {
		VM_BUG_ON(!virt_addr_valid((void *)addr));
		__homecache_free_pages(virt_to_page((void *)addr), order);
	}
}
EXPORT_SYMBOL(homecache_free_pages);

#else  /* dynamic homecaching support hooked into the Linux internals */

/*
 * When we free a page, in addition to marking the pages as free,
 * we check the homing to determine what to do with it.
 *
 * Pages that are compatible with the buddy allocator get freed
 * normally.  With hash_default, this means hash-for-home pages only,
 * since we expect most pages to come back that way.  Otherwise, we
 * do all pages that are "cheap" to re-home (i.e. pages cached on
 * a single cpu, plus uncached pages).
 *
 * Pages that are potentially cached on every cpu are put onto
 * a special global "sequestered" list, since we don't anticipate
 * being able to easily reuse them for anything without doing
 * a global round of cache flushing first.
 *
 * Other pages (the kind that didn't go in the buddy allocator)
 * are stored on a special per-zone free list and checked whenever
 * we do an allocation from that zone that requests a home override,
 * so we can re-use them without having to do a remote flush
 * on a page that we pulled from the buddy allocator.
 */

/* Do we want to free this page back to the buddy allocator? */
static int home_is_freeable(int home)
{
	/* For hash-default heap, we only free back hash pages. */
	if (hash_default)
		return home == PAGE_HOME_HASH;

	/* Otherwise, we only free back things that are easy to re-home. */
	return (home == PAGE_HOME_UNCACHED || home >= 0);
}

/*
 * When resetting a page's homecache (e.g. when freshly allocating the
 * page, flushing out the homecache free list, or converting an
 * immutable page) what home should we reset it to?
 */
static int default_page_home(void)
{
	if (hash_default)
		return PAGE_HOME_HASH;
	/* Pick an arbitrary cpu to home the page. */
	return raw_smp_processor_id();
}

/* When true, the homecache checker passes all pages to the buddy allocator. */
static DEFINE_PER_CPU(int, homecache_is_unsequestering);

/* Check if this page should be returned to homecache, not page_alloc. */
int homecache_check_free_page(struct page *page, int order)
{
	int pages, i;
	int home = page_home(page);

	/*
	 * Clear the homecache_nomigrate bit; it would only have been
	 * set if we had vmapped the page and now have unmapped it and
	 * are freeing it.
	 */
	pages = (1 << order);
	for (i = 0; i < pages; ++i)
		__ClearPageHomecacheNomigrate(&page[i]);

	/* Return to buddy allocator if we are unsequestering. */
	if (this_cpu_read(homecache_is_unsequestering))
		return 0;

	/*
	 * Validate that the whole allocation was identically homed.
	 * This should normally be true unless the kernel is playing
	 * games by grouping together contiguous unrelated pages, as
	 * e.g. is done by free_all_bootmem_core().  If we encounter a
	 * mismatch, we just dump all the pages back into the buddy
	 * allocator and count on homecache_new_kernel_page() to fix
	 * it up later when it is allocated.
	 */
	pages = (1 << order);
	for (i = 1; i < pages; ++i)
		if (page_home(&page[i]) != home)
			return 0;

	/*
	 * In kdata=huge mode, any lowmem page mapped by a small
	 * kernel PTE goes onto the per-zone freelists.
	 */
	if ((!kdata_huge ||
	     PageHighMem(page) ||
	     pte_huge(*virt_to_kpte((ulong)page_address(page)))) &&
	    home_is_freeable(home))
		return 0;

	/* Otherwise, the homecache code will keep it. */
	return 1;
}

/*
 * Translate a cache home to the list within the zone where we keep
 * similarly-homed free pages.
 */
static struct list_head *homecache_zone_list(struct zone *zone, int home)
{
	if (home >= 0)
		return &zone->homecache_list[home];
	else if (home == PAGE_HOME_UNCACHED)
		return &zone->homecache_list[ZONE_HOMECACHE_UNCACHED_INDEX];
	else if (home == PAGE_HOME_HASH)
		return &zone->homecache_list[ZONE_HOMECACHE_HFH_INDEX];
	else if (incoherent_home(home))
		return &zone->homecache_list[ZONE_HOMECACHE_INCOHERENT_INDEX];
	else
		return NULL;
}

/*
 * Make sure we're not putting a page on the free list that a later
 * allocation is going to complain about.
 */
static void homecache_add_to_free_list(struct page *page,
				       struct list_head *list)
{
	if (likely((page->flags & PAGE_FLAGS_CHECK_AT_PREP) == 0)) {
		list_add(&page->lru, list);
		return;
	}

	/* Leak the page. */
	printk(KERN_ALERT "BUG: Bad page state in process %s  pfn:%05lx\n",
	       current->comm, page_to_pfn(page));
	dump_page(page);
	dump_stack();
}

/*
 * Actually free a page back to the homecache lists.
 * Should only be called if homecache_check_free_page() is true.
 */
void homecache_keep_free_page(struct page *page, int order)
{
	int home = page_home(page);
	struct zone *zone = page_zone(page);
	struct list_head *list = homecache_zone_list(zone, home);
	unsigned long flags;
	int pages = (1 << order);
	int i;

	BUG_ON(!list);
	spin_lock_irqsave(&zone->lock, flags);
	homecache_proc_sequestered_free += pages;
	for (i = 0; i < pages; ++i)
		homecache_add_to_free_list(&page[i], list);
	if (incoherent_home(home))
		zone->homecache_incoherent_count += pages;
	else
		zone->homecache_coherent_count += pages;
	spin_unlock_irqrestore(&zone->lock, flags);
}

/* Check that the page was allocated properly. */
static void check_page_home(struct page *page, int home)
{
	if (home != page_home(page))
		panic("Allocated page PFN %#lx should have home %d, has %d\n",
		      page_to_pfn(page), home, page_home(page));
	if (!PageHighMem(page)) {
		void *kva = page_address(page);
		pte_t pte = *virt_to_kpte((unsigned long)kva);
		if (pte_to_home(pte) != home)
			panic("Allocated page PFN %#lx should have home %d,"
			      " but has PTE home %d",
			      page_to_pfn(page), home, pte_to_home(pte));
	}
}

struct desired_home {
	int home;
	int depth;
	struct desired_home *next;
};

#define PUSH_DESIRED_HOME(home)					      \
	{							      \
		struct desired_home __dh = {			      \
			home,					      \
			irq_count(),				      \
			current->thread.homecache_desired_home	      \
		};						      \
		current->thread.homecache_desired_home = &__dh
		
#define POP_DESIRED_HOME()					      \
		current->thread.homecache_desired_home = __dh.next;   \
	}

/*
 * Return the desired home set by the current interrupt level.
 * If we were interrupted after setting the home, any allocations
 * in the interrupt context will have a different irq_count()
 * and so will return default_page_home().
 */
static inline int current_desired_home(void)
{
	struct desired_home *dh = current->thread.homecache_desired_home;
	if (dh && dh->depth == irq_count())
		return dh->home;
	else
		return default_page_home();
}

struct page *homecache_alloc_pages(gfp_t gfp_mask,
				   unsigned int order, int home)
{
	struct page *page;
	PUSH_DESIRED_HOME(home);
	page = alloc_pages(gfp_mask, order);
	POP_DESIRED_HOME();
	if (page)
		check_page_home(page, home);
	return page;
}
EXPORT_SYMBOL(homecache_alloc_pages);

struct page *homecache_alloc_pages_node(int nid, gfp_t gfp_mask,
					unsigned int order, int home)
{
	struct page *page;
	PUSH_DESIRED_HOME(home);
	page = alloc_pages_node(nid, gfp_mask, order);
	POP_DESIRED_HOME();
	if (page)
		check_page_home(page, home);
	return page;
}

struct page *homecache_alloc_page_vma(gfp_t gfp_mask,
				      struct vm_area_struct *vma,
				      unsigned long addr)
{
	pgprot_t prot = vma->vm_page_prot;
	if (!pte_get_forcecache(prot)) {
		return alloc_page_vma(gfp_mask, vma, addr);
	} else {
		struct page *page;
		int home = default_page_home();

		home = homecache_get_desired_home(home, prot, 1);
		PUSH_DESIRED_HOME(home);
		page = alloc_page_vma(gfp_mask, vma, addr);
		POP_DESIRED_HOME();
		if (page == NULL)
			return NULL;
		check_page_home(page, home);

		/*
		 * If we are allocating a page with noalloc attributes,
		 * we should ensure it starts with a clean local cache.
		 * Normal coherence won't necessarily have flushed the
		 * local cache.
		 */
		if (hv_pte_get_no_alloc_l2(prot) ||
		    hv_pte_get_no_alloc_l1(prot))
			homecache_finv_page(page);

		return page;
	}
}

void __homecache_free_pages(struct page *page, unsigned int order)
{
	__free_pages(page, order);
}
EXPORT_SYMBOL(__homecache_free_pages);

void homecache_free_pages(unsigned long addr, unsigned int order)
{
	free_pages(addr, order);
}
EXPORT_SYMBOL(homecache_free_pages);

/* Called with interrupts disabled but the zone unlocked. */
struct page *homecache_get_cached_page(struct zone *zone, gfp_t gfp_flags)
{
	struct page *page;
	int home = current_desired_home();
	int require_homecache = kdata_huge && !is_highmem(zone);
	int order;
	struct list_head *list;

	/* Don't bother to look up incoherent pages. */
	if (incoherent_home(home))
		return NULL;

	/*
	 * The __GFP_HIGHMEM flag is a hint to this code that, if it
	 * is in a "require_homecache" zone (lowmem with kdata_huge)
	 * it should go to the homecache free list even if the request
	 * does not appear to merit homecaching.  This is because
	 * such pages are for user space or the page cache, both of
	 * which are prone to homecache adjustment.
	 */
	if (!((gfp_flags & __GFP_HIGHMEM) && require_homecache)) {

		/* Don't bother for types that match the allocator. */
		if (home_is_freeable(home))
			return NULL;
	}

	/*
	 * See if there are any pages on the freelist for pages with the
	 * desired home.
	 */
	list = homecache_zone_list(zone, home);
	spin_lock(&zone->lock);
	if (list && !list_empty(list)) {
		page = list_first_entry(list, struct page, lru);
		check_page_home(page, home);
		list_del(&page->lru);
		--zone->homecache_coherent_count;
		goto unlock;
	}

	/*
	 * If we are using huge pages in the buddy allocator, and this
	 * is a LOWMEM zone, we would rather re-homecache an existing
	 * small page than shatter a new huge page.  So see if we
	 * have anything that is usable if we re-homecache it.
	 * We ignore the "migratetype", which is basically about
	 * defragmentation, and segregating the homecache pages is
	 * too, so it's plausible to ignore migratetype if necessary.
	 */
	if (require_homecache) {
		int i;

		/*
		 * We keep track of the last place we found a page, and
		 * start the search there.
		 */
		i = zone->lru_homecache_list;
		do {
			list = &zone->homecache_list[i];
			if (!list_empty(list)) {
				page = list_first_entry(list, struct page, lru);
				list_del(&page->lru);
				--zone->homecache_coherent_count;
				zone->lru_homecache_list = i;
				goto unlock;
			}
			i = i ? i - 1 : NR_COHERENT_ZONE_HOMECACHE_LISTS - 1;
		} while (i != zone->lru_homecache_list);

		/*
		 * We need to shatter a new huge page.  Ideally we get
		 * an entire huge page and shatter it.  But we work
		 * our way down to order-0 anyway, to be robust.
		 */
		for (order = HUGETLB_PAGE_ORDER; order >= 0; --order) {
			int migratetype = allocflags_to_migratetype(gfp_flags);
			page = homecache_rmqueue(zone, order, migratetype);
			if (page)
				break;
		}
		if (page) {
			int i, pages = 1 << order;
			unsigned long kaddr =
				(unsigned long)page_address(page);
			shatter_huge_page(kaddr);
			for (i = 1; i < pages; ++i) {
				list = homecache_zone_list(zone,
							   page_home(&page[i]));
				BUG_ON(!list);
				homecache_add_to_free_list(&page[i], list);
			}
			zone->homecache_coherent_count += pages - 1;
			goto unlock;
		}
	}

	/* No luck; just get something from the buddy allocator. */
	page = NULL;

 unlock:
	spin_unlock(&zone->lock);
	return page;
}

unsigned long homecache_count_sequestered_pages(bool total_mem,
						bool include_incoherent)
{
	unsigned long npages = 0;
	pg_data_t *pgdat;

	for_each_online_pgdat(pgdat) {
		int i;
		for (i = 0; i < MAX_NR_ZONES; i++) {
			struct zone *zone;
#ifdef CONFIG_HIGHMEM
			if (!total_mem &&
			    (i == ZONE_HIGHMEM ||
			     (zone_movable_is_highmem() && i == ZONE_MOVABLE)))
				continue;
#endif
			zone = &pgdat->node_zones[i];
			npages += zone->homecache_coherent_count;
			if (include_incoherent)
				npages += zone->homecache_incoherent_count;
		}
	}

	return npages;
}

/*
 * Collect all the pages that we mean to return to the allocator.
 * Any pages that were freed but homed on a non-cacheable cpu
 * (e.g. a hypervisor driver cpu) we ignore, but just count up
 * so we can reset the totals appropriately.
 */
static void homecache_collect_free_pages(struct zone *zone,
					 struct list_head *work_list,
					 bool include_incoherent)
{
	int i, npages = 0;
	spin_lock(&zone->lock);
	for (i = 0; i < NR_COHERENT_ZONE_HOMECACHE_LISTS; ++i) {
		if (i < NR_CPUS && !cpu_cacheable(i)) {
			struct list_head *list;
			list_for_each(list, &zone->homecache_list[i])
				++npages;
			continue;
		}
		list_splice_init(&zone->homecache_list[i], work_list);
	}
	zone->homecache_coherent_count = npages;
	if (include_incoherent) {
		struct list_head *list =
			&zone->homecache_list[ZONE_HOMECACHE_INCOHERENT_INDEX];
		list_splice_init(list, work_list);
		zone->homecache_incoherent_count = 0;
	}
	spin_unlock(&zone->lock);
}

/*
 * Walk a list of free pages, flush them from the cache, reset their homes
 * to the default, reset their lowmem PTE (if any) to the new home, and
 * invalidate any lowmem TLB entries.  Return the number of pages on the
 * list.
 */
static int homecache_reset_free_pages(struct list_head *work_list)
{
	unsigned long low_kaddr = ~0UL, high_kaddr = 0;
	struct page *page;
	int npages = 0;

	list_for_each_entry(page, work_list, lru) {
		homecache_finv_page(page);
		set_page_home(page, default_page_home());
		if (!PageHighMem(page)) {
			unsigned long kva = (unsigned long)page_address(page);
			pte_t *pte = virt_to_kpte(kva);
			BUG_ON(pte == NULL || pte_huge(*pte));
			set_pte(pte, mk_pte(page, PAGE_KERNEL));
			if (kva < low_kaddr)
				low_kaddr = kva;
			if (kva + PAGE_SIZE > high_kaddr)
				high_kaddr = kva + PAGE_SIZE;
		}
		++npages;
	}

	if (low_kaddr > high_kaddr) {
		low_kaddr = 0;
		high_kaddr = 0;
	}
	
	/* Flush TLBs everywhere */
	flush_remote(0, 0, NULL, low_kaddr, high_kaddr - low_kaddr,
		PAGE_SIZE, cpu_online_mask, NULL, 0);

	return npages;
}

int homecache_recover_free_pages_zone(struct zone *zone)
{
	unsigned long flags;
	LIST_HEAD(work_list);
	struct page *page, *tmp;

	/* Disable interrupts so we don't uselessly re-enter this routine. */
	local_irq_save(flags);

	/* Collect coherent free pages from this zone. */
	homecache_collect_free_pages(zone, &work_list, 0);

	/* If there are no pages, we're done. */
	if (list_empty(&work_list)) {
		local_irq_restore(flags);
		return 0;
	}

	++homecache_proc_coherent_sequestered_purge;

	/* Free all the pages back to the buddy allocator. */
	__get_cpu_var(homecache_is_unsequestering) = 1;
	list_for_each_entry_safe(page, tmp, &work_list, lru)
		free_hot_cold_page(page, 1);
	__get_cpu_var(homecache_is_unsequestering) = 0;

	/* TODO: this would be an obvious point to unshatter any huge pages. */

	local_irq_restore(flags);
	return 1;
}

int homecache_recover_free_pages(void)
{
	unsigned long flags;
	bool include_incoherent;
	LIST_HEAD(work_list);
	struct page *page, *tmp;
	struct zone *zone;

	/* Disable interrupts so we don't uselessly re-enter this routine. */
	local_irq_save(flags);

	/*
	 * If the coherent freelists have sufficient pages, just free
	 * them back to the allocator without resetting their homes
	 * and doing the extra work required for incoherent pages.
	 * We can safely return pages from the coherent freelists with no
	 * need to do anything else for now, since we can flush those
	 * pages easily as we start re-allocating them.
	 *
	 * FIXME: it doesn't really make sense that our decision to
	 * free incoherent pages is coupled to our decision to reset
	 * the page homes for the coherent pages.  We can make those
	 * decisions separately now and probably should think through
	 * what it is we really want to do here.
	 */
	include_incoherent = (homecache_count_sequestered_pages(1, 0) < 256);

	/* Collect free pages from each zone. */
	for_each_zone(zone)
		homecache_collect_free_pages(zone, &work_list,
					     include_incoherent);

	/* If there are no pages, we're done. */
	if (list_empty(&work_list)) {
		local_irq_restore(flags);
		return 0;
	}

	/* If we're going to flush incoherent pages, update homes and PTEs. */
	if (include_incoherent) {
		homecache_reset_free_pages(&work_list);
		++homecache_proc_sequestered_purge;
	} else {
		++homecache_proc_coherent_sequestered_purge;
	}

	/* Free all the pages back to the buddy allocator. */
	__get_cpu_var(homecache_is_unsequestering) = 1;
	list_for_each_entry_safe(page, tmp, &work_list, lru)
		free_hot_cold_page(page, 1);
	__get_cpu_var(homecache_is_unsequestering) = 0;

	/* TODO: this would be an obvious point to unshatter any huge pages. */

	local_irq_restore(flags);
	return 1;
}


/*
 * Provide a lock and two accessors to lock when we are doing
 * page migration and when we are trying to add new kernel mappings.
 */

#ifdef CONFIG_HIGHMEM
static DEFINE_SPINLOCK(kpte_lock);

unsigned long homecache_kpte_lock_irqsave(void)
	__acquires(kpte_lock)
{
	unsigned long flags;
	spin_lock_irqsave(&kpte_lock, flags);
	return flags;
}

void homecache_kpte_unlock_irqrestore(unsigned long flags)
	__releases(kpte_lock)
{
	spin_unlock_irqrestore(&kpte_lock, flags);
}

void homecache_kpte_lock(void)
	__acquires(kpte_lock)
{
	spin_lock(&kpte_lock);
}

void homecache_kpte_unlock(void)
	__releases(kpte_lock)
{
	spin_unlock(&kpte_lock);
}
#else
/* No need to coordinate with HIGHMEM, so no lock. */
static inline void homecache_kpte_lock(void) {}
static inline void homecache_kpte_unlock(void) {}
#endif


/*
 * Find the kernel PTE mapping this page (either lowmem or kmap) and
 * adjust it as follows.  If "finished" is false, we mark it as
 * migrating; otherwise, we rebuild it "from scratch".  In
 * the normal migration model (mark PTEs migrating; flush TLBs; flush
 * caches; rewrite PTEs) the finished=0 and finished=1 modes
 * correspond to the first and last phases, respectively.
 *
 * FIXME: ptrace writes on huge pages will create temporary mappings
 * of sub-pages within the huge page, and we will not see it since we
 * are only checking for the vaddr of the beginning of the huge page.
 * We could loop calling kmap_fix_kpte() or pass "pages" to kmap_fix_kpte,
 * but either way is still likely pretty inefficient, and we might end
 * up with a set of unrelated kernel VAs that we need to flush.
 */
static unsigned long homecache_fix_kpte(struct page *page, int pages,
					int finished)
{
	unsigned long pfn, vaddr;
	int i;

#ifdef CONFIG_HIGHMEM
	if (PageHighMem(page)) {
		if (pages > 1)
			return 0;
		kmap_atomic_fix_kpte(page, finished);
		return (unsigned long)kmap_fix_kpte(page, finished);
	}
#endif
	pfn = page_to_pfn(page);
	vaddr = (unsigned long)lowmem_page_address(page);
	for (i = 0; i < pages; ++i) {
		int home = page[i].home;
		pgprot_t prot = (home == PAGE_HOME_IMMUTABLE) ?
			PAGE_KERNEL_RO : PAGE_KERNEL;
		pte_t *ptep = virt_to_shattered_kpte(vaddr + (i * PAGE_SIZE));
		pte_t pteval = pfn_pte(pfn + i, pte_set_home(prot, home));
		if (!finished) {
			BUG_ON(!pte_same(*ptep, pteval));
			/* Use __set_pte to allow writing the migrating bit. */
			__set_pte(ptep, pte_mkmigrate(pteval));
		} else {
			set_pte(ptep, pteval);
		}
	}
	return vaddr;
}

/* Evict a contiguous set of pages from cache. */
static void homecache_finv_pages(struct page *page, int pages, int force_map)
{
	int i;
	for (i = 0; i < pages; ++i, ++page)
		homecache_finv_page_internal(page, force_map);
}

/* Mark a group of pages with their new homecache. */
static void set_pages_home(struct page *page, int pages, int home)
{
	int i;
	for (i = 0; i < pages; ++i)
		set_page_home(&page[i], home);
}

/*
 * Remember that we allocated this page on this processor,
 * so that we can set up our PTEs to always reference that home.
 * However, note that we have per-cpu freelists, so that it is
 * at least plausible that we will get mostly same-cpu homed
 * pages once we get into a steady state.
 *
 * Locking requirements are substantially eased in this code
 * because we are guaranteeing that the page(s) are not mapped
 * into user-space anywhere.
 *
 * The "home" argument is the requested new setting for the
 * specified block of pages.
 */
void homecache_change_page_home(struct page *page, int order, int new_home)
{
	struct thread_struct *ts = &current->thread;
	int pages = 1 << order;
	unsigned long vaddr;
	unsigned long flags;
	int i;

	/*
	 * Validate the assumption that the page is unmapped
	 * and is available for migration.
	 */
	for (i = 0; i < pages; ++i) {
		BUG_ON(page_mapcount(&page[i]) != 0);
		BUG_ON(PageHomecacheNomigrate(&page[i]));
	}

	/* Do a quick check if migration is needed at all. */
	for (i = 0; i < pages; ++i) {
		if (page_home(&page[i]) != new_home)
			break;
	}
	if (i == pages)
		return;

	/* Flush page out of cache(s) and reset its TLBs, if any. */
	ts->homecache_is_migrating = true;
	he_begin();
	homecache_proc_migrated_unmapped += pages;
	local_irq_save(flags);
	homecache_kpte_lock();
	he_add(HE_UNMAPPED, new_home, page_to_pfn(page));
	vaddr = homecache_fix_kpte(page, pages, 0);
	if (vaddr)
		flush_remote(0, 0, NULL, vaddr, pages * PAGE_SIZE, PAGE_SIZE,
			     cpu_online_mask, NULL, 0);
	homecache_finv_pages(page, pages, 1);
	set_pages_home(page, pages, new_home);
	homecache_kpte_unlock();
	homecache_fix_kpte(page, pages, 1);
	sim_validate_lines_evicted(PFN_PHYS(page_to_pfn(page)),
				   pages * PAGE_SIZE);
	local_irq_restore(flags);
	he_end();
	ts->homecache_is_migrating = false;
}
EXPORT_SYMBOL(homecache_change_page_home);

void homecache_new_kernel_page(struct page *page, int order)
{
	homecache_change_page_home(page, order, current_desired_home());
}

void homecache_update_migrating_pte(struct page *page, pte_t *ptep,
				    struct vm_area_struct *vma,
				    unsigned long address)
{
	pte_t oldpte = *ptep;
	unsigned long pfn = pte_pfn(oldpte);
	pte_t pte = pfn_pte(pfn, vma->vm_page_prot);
	pte.val = (pte.val & ~_PAGE_ALL) | (oldpte.val & _PAGE_ALL);
	pte = hv_pte_clear_nc(pte_donemigrate(pte));
	pte = pte_set_home(pte, page_home(page));
	set_pte_at(vma->vm_mm, address, ptep, pte);
}

/*
 * Change the homing of a mapped page, flushing any stale PTEs.
 * The page must be locked on entry.
 */
static void homecache_home_mapped_page(struct page *page, int order,
				       int new_home)
{
	struct thread_struct *ts = &current->thread;
	int pages = 1 << order;
	void *vaddr;
	int ttu_flags = TTU_IGNORE_MLOCK | TTU_IGNORE_ACCESS;
	int rc;
	unsigned long flags;
#ifdef DEBUG
	unsigned long pfn = page_to_pfn(page);
#endif

	/* Check some requirements. */
	BUG_ON(!PageLocked(page));

	/* Check if we need to do anything. */
	if (page_home(page) == new_home)
		return;

	/*
	 * If the page is not migratable (e.g. a vmalloc page being
	 * provided by a driver to user space) just ignore this request.
	 * We leave it cached however it was cached before.
	 */
	if (PageHomecacheNomigrate(page))
		return;

	ts->homecache_is_migrating = true;
	he_begin();

	homecache_proc_migrated_mapped += pages;

	/*
	 * Now, find all the places where this PTE used to be set,
	 * mark them all as migrating, and flush the page out of
	 * TLB in all the mm's that are referencing the page,
	 * and out of the kernel lowmem or kmap area (if any).
	 * We flip anonymous PTEs to "migrating" (resetting them below),
	 * but just clear file PTEs outright.
	 */
	if (pages == 1) {
		he_add(HE_TTU_START, new_home, pfn);
		rc = try_to_unmap(page, ttu_flags | TTU_HOMECACHE_START);
		BUG_ON(!PageAnon(page) && rc != SWAP_SUCCESS);
	}

	/*
	 * Lock out any new kmaps to avoid creating an inconsistent mapping.
	 * Also disable interrupts to avoid self-deadlock.
	 */
	local_irq_save(flags);
	homecache_kpte_lock();
	he_add(HE_MAPPED, new_home, pfn);
	vaddr = (void *)homecache_fix_kpte(page, pages, 0);
	if (vaddr) {
		flush_remote(0, 0, NULL,
			     (HV_VirtAddr) vaddr, pages * PAGE_SIZE, PAGE_SIZE,
			     cpu_online_mask, NULL, 0);
	}

	/*
	 * Flush the caches, since no cpu can touch the caches that we
	 * are migrating away from now.
	 */
	homecache_finv_pages(page, pages, 1);

	/* Mark the pages with their new cache info. */
	set_pages_home(page, pages, new_home);

	/* Release the kpte lock since new kmaps can be created now. */
	homecache_kpte_unlock();

	/* Fix the kernel PTE. */
	homecache_fix_kpte(page, pages, 1);

	/*
	 * Restore irqs now that we have fixed up the kernel page mappings.
	 * We need interrupts enabled to run try_to_unmap() safely.
	 */
	local_irq_restore(flags);

	/* Make any anonymous user PTEs assume their correct value. */
	if (PageAnon(page) && pages == 1) {
		he_add(HE_TTU_FINISH, new_home, pfn);
		try_to_unmap(page, ttu_flags | TTU_HOMECACHE_FINISH);
	}

	he_end();
	ts->homecache_is_migrating = false;
}

/*
 * This method checks the given home against the passed pgprot (and
 * whether we intend to write to it) and returns an appropriate new home.
 */
static int homecache_get_desired_home(int home, pgprot_t prot, int writable)
{
	if (home == PAGE_HOME_IMMUTABLE) {
		/*
		 * Immutable pages are treated specially.  If we are
		 * writing to them, we convert them to normal pages
		 * following the pgprot.  Otherwise, we do nothing,
		 * since any pgprot is compatible with an immutable page.
		 */
		if (!writable)
			return home;
		home = default_page_home();
	}

	/* If the pgprot isn't intended to force the mapping, we're done. */
	if (!pte_get_forcecache(prot))
		return home;

	switch (hv_pte_get_mode(prot)) {
	case HV_PTE_MODE_UNCACHED:
		home = PAGE_HOME_UNCACHED;
		break;
	case HV_PTE_MODE_CACHE_NO_L3:
		/*
		 * If we are just caching locally, we must be
		 * either incoherent or immutable.  Tolerate a
		 * read-only mapping of incoherent memory.
		 */
		if (home != PAGE_HOME_INCOHERENT)
			home = writable ? PAGE_HOME_INCOHERENT :
				PAGE_HOME_IMMUTABLE;
		break;
	case HV_PTE_MODE_CACHE_TILE_L3:
		/* Set the page home if requested by the pgprot. */
		if (!pte_get_anyhome(prot)) {
			/*
			 * Get requested CPU.  Note that users can't
			 * mmap() with a hypervisor lotar, so if one were
			 * in the pgprot, we would correctly assert
			 * in get_remote_cache_cpu().
			 */
			home = get_remote_cache_cpu(prot);
		} else {
			/* A lotar with anyhome is confused. */
			BUG_ON(hv_pte_get_lotar(prot));

			/*
			 * Pick an arbitrary single-tile home if the page
			 * didn't already have one.
			 */
			if (home < 0)
				home = raw_smp_processor_id();
		}
		/* Writable pages can't be immutable. */
		if (!writable && hv_pte_get_nc(prot))
			home = PAGE_HOME_IMMUTABLE;
		break;
	case HV_PTE_MODE_CACHE_HASH_L3:
		home = PAGE_HOME_HASH;
		BUG_ON(hv_pte_get_lotar(prot) != 0);
		/* Writable pages can't be immutable. */
		if (!writable && hv_pte_get_nc(prot))
			home = PAGE_HOME_IMMUTABLE;
		break;
	default:
		panic("Invalid mode in pte %#llx", hv_pte_val(prot));
		break;
	}
	return home;
}

void homecache_home_page_here(struct page *page, int order, pgprot_t prot)
{
	int home = page_home(page);

	/*
	 * If this pgprot forces the page to be homed somewhere specific,
	 * just return and don't try to move it around.
	 */
	if (home != PAGE_HOME_IMMUTABLE &&
	    pte_get_forcecache(prot) &&
	    (hv_pte_get_mode(prot) == HV_PTE_MODE_UNCACHED ||
	     hv_pte_get_mode(prot) == HV_PTE_MODE_CACHE_NO_L3 ||
	     hv_pte_get_mode(prot) == HV_PTE_MODE_CACHE_HASH_L3 ||
	     !pte_get_anyhome(prot)))
		return;

	/* Make sure the page is actually homed on a single cpu. */
	if (home < 0 && home != PAGE_HOME_IMMUTABLE)
		return;

	/* Change this page to be coherently cached on this cpu. */
	home = homecache_get_desired_home(default_page_home(), prot, 1);

	/* Re-home the page. */
	homecache_home_mapped_page(page, order, home);
}

void homecache_update_page(struct page *page, int order,
			   struct vm_area_struct *vma, int writable)
{
	int home = page_home(page);
	pgprot_t prot = vma->vm_page_prot;

	/*
	 * If there is already a shared writable mapping for this file, it
	 * will own the caching of its pages, so just return early.
	 *
	 * FIXME: walk through the vmas with vma_prio_tree_foreach()
	 * and if we overlap with a shared one, force its homing here,
	 * and if not, use our requested homing.  This would also give
	 * us better granularity, since there might be a non-overlapping
	 * shared-writable mapping that this mapping could then ignore.
	 */
	if (!(vma->vm_flags & VM_SHARED) &&
	    vma->vm_file && vma->vm_file->f_mapping->i_mmap_writable > 0)
		return;

	/*
	 * If we are setting up a shared writable mapping, we may not
	 * come into this path via an actual write, but we still want
	 * to set up the mapping as writable.
	 */
	if (hv_pte_get_writable(prot))
		writable = 1;

	/*
	 * If the access is for read, and the mapping is private,
	 * and the page is from a file and is not shared writably,
	 * we ignore "prot" and make it immutable instead.
	 *
	 * If noallocl2 is set, we never cache pages locally, so
	 * there's no point in claiming they are immutable.
	 */
	if (!writable && !(vma->vm_flags & VM_SHARED) && !noallocl2 &&
	    vma->vm_file && vma->vm_file->f_mapping->i_mmap_writable == 0) {
		home = PAGE_HOME_IMMUTABLE;
	} else {
		home = homecache_get_desired_home(home, prot, writable);
	}

	homecache_home_mapped_page(page, order, home);
}

void homecache_make_writable(struct page *page, int order)
{
	int home = page_home(page);
	if (home == PAGE_HOME_IMMUTABLE) {
		home = homecache_get_desired_home(home, PAGE_KERNEL, 1);
		homecache_home_mapped_page(page, order, home);
	}
}
EXPORT_SYMBOL(homecache_make_writable);

void homecache_new_user_page(struct page *page, int order,
			     pgprot_t prot, int writable)
{
	int home = page_home(page);

	home = homecache_get_desired_home(home, prot, writable);
	homecache_change_page_home(page, order, home);
}


/* Information needed to migrate user-space PTEs. */
struct migrating_pte {
	pte_t pteval;           /* copy of PTE (with migrating bit set) */
	pte_t *ptep;            /* pointer to PTE */
	spinlock_t *ptl;        /* non-NULL if this entry locked the PTE */
	unsigned long va;       /* user space address for this PTE */
	unsigned long npages;   /* number of small pages here */
	struct page *page_lock; /* non-NULL if this entry locked the page */
};

/*
 * Number of entries to migrate per iteration.
 * This is just a tradeoff of stack memory used vs number of times
 * we need to walk through the outer migration loop.
 */
#define MIGRATING_COUNT 32

/* State for the migration algorithm, kept on-stack. */
struct migrate_state {
	cpumask_t *cache_cpumask;		/* cpus to flush cache on */
	cpumask_t *tlb_cpumask;			/* cpus to flush TLB on */
	int cpu;				/* cpu that owns this struct */
	int num_rem_asids;			/* remote ASID count */
	HV_Remote_ASID *rem_asids;		/* remote ASIDs */
	unsigned long low_kaddr, high_kaddr;	/* bounds of kaddrs to flush */
	int migrating_index;			/* next entry in migrating[] */
	struct migrating_pte migrating[MIGRATING_COUNT];
};

/* Add information for a new migrating_pte to the list. */
static void add_migrating_pte(struct migrate_state *ms,
			      spinlock_t *ptl, pte_t *ptep, unsigned long va,
			      unsigned long npages, struct page *page_lock)
{
	struct migrating_pte *mpte;
	BUG_ON(ms->migrating_index >= MIGRATING_COUNT);
	mpte = &ms->migrating[ms->migrating_index++];
	mpte->ptl = ptl;
	mpte->ptep = ptep;
	mpte->va = va;
	mpte->npages = npages;
	mpte->page_lock = page_lock;
}

/* Check to see if we're already locked a given page. */
static int is_page_locked(struct migrate_state *ms, struct page *page)
{
	int i, count = ms->migrating_index;
	for (i = 0; i < count; ++i)
		if (ms->migrating[i].page_lock == page)
			return 1;
	return 0;
}

/* Check to see if we're already locked a given page table lock. */
static int is_page_table_locked(struct migrate_state *ms, spinlock_t *ptl)
{
	int i, count = ms->migrating_index;
	for (i = 0; i < count; ++i)
		if (ms->migrating[i].ptl == ptl)
			return 1;
	return 0;
}

/*
 * Add information on a region of kernel VAs that we need to flush.
 * Right now we end up just passing a single (start,size) argument to
 * the hypervisor, but we structure it as an API here so that we
 * can use a modified hypervisor API more easily at a later date.
 */
static void add_kaddr_flush(struct migrate_state *ms,
			    unsigned long kaddr, unsigned long size)
{
	unsigned long end = kaddr + size;
	if (kaddr < ms->low_kaddr)
		ms->low_kaddr = kaddr;
	if (end > ms->high_kaddr)
		ms->high_kaddr = end;
}

/*
 * Rewrite the PTE as migrating so any attempt to use it will cause a hang.
 * Save the pteval that we should expect to find when we finish migrating.
 */
static void migrate_start_pte(struct migrate_state *ms,
			      struct migrating_pte *mpte)
{
	pte_t pteval;
	unsigned long pfn;
	struct page *page;
	int npages = mpte->npages;
	unsigned long kva;

	/*
	 * We use ptep_get_and_clear() on tilepro to avoid racing
	 * with the hypervisor's dirty/accessed bit setting.  Note that
	 * there is a brief window of vulnerability where the pte is
	 * zero-valued, but this is true elsewhere too, e.g. mprotect.
	 * On tilegx the hypervisor uses cmpxchg, so we can just write
	 * the new value directly.
	 */
#ifdef __tilegx__
	pteval = *mpte->ptep;
#else
	pteval = ptep_get_and_clear(current->mm, mpte->va, mpte->ptep);
#endif
	pteval = pte_mkmigrate(pteval);
	__set_pte(mpte->ptep, pteval);
	mpte->pteval = pteval;

	pfn = pte_pfn(pteval);
	page = pfn_to_page(pfn);

	kva = homecache_fix_kpte(page, npages, 0);
	if (kva != 0)
		add_kaddr_flush(ms, kva, npages * PAGE_SIZE);

	he_add(HE_PAGE, smp_processor_id(), pfn);
}

/*
 * Number of pages above which we just do a cache evict.  We pick a
 * number based on total cache size (since that determines the cost of
 * an eviction), as well as page size (since that determines the
 * number of finvs, loads, etc, issued by finv_buffer_remote).  The
 * arbitrary constant makes us prefer page-based finv so as to avoid
 * unexpected cache flushes to other cores, rather than to optimize
 * migrating processes that have extra locally-homecached pages.
 */
static unsigned long homecache_proc_migrate_finv_max =
	16 * CHIP_L2_CACHE_SIZE() / PAGE_SIZE;

/* Count the number of times we decided to migrate via eviction. */
static unsigned long homecache_proc_migrate_evict;

/*
 * Scan pages and decide which ones to finv and which ones to displace.
 * Pages homed anywhere other than the core we are migrating from
 * will always get finv'ed to avoid interrupting other cores
 * unnecessarily.  We will count up the number of pages on the core
 * we are migrating from and if we pass a threshold, we will do an
 * eviction flush on that core instead.
 */
static bool homecache_migrate_check_evict(struct migrate_state *ms,
					  bool migrate_stack)
{
	int i;
	int old_cpu = current_thread_info()->homecache_cpu;
	unsigned long count = migrate_stack ? 1 : 0;
	bool do_evict = false;

	for (i = 0; i < ms->migrating_index; ++i) {
		struct migrating_pte *mpte = &ms->migrating[i];
		if (pte_to_home(mpte->pteval) != old_cpu)
			continue;   /* we will finv this page */
		count += mpte->npages;
		if (count > homecache_proc_migrate_finv_max) {
			do_evict = true;
			++homecache_proc_migrate_evict;
			cpumask_set_cpu(old_cpu, ms->cache_cpumask);
			break;
		}
	}

	return do_evict;
}

/* Adjust the page so it is ready to go with its new cpu home. */
static void migrate_finish_page(struct migrating_pte *mpte, int cpu,
				bool do_finv)
{
	pte_t pteval = mpte->pteval;
	struct page *page = pfn_to_page(pte_pfn(pteval));
	int npages = mpte->npages;
	int i;

	/* Finv the page if requested, and fix the struct page home. */
	for (i = 0; i < npages; i++) {
		if (do_finv)
			homecache_finv_page_internal(&page[i], 1);
		set_page_home(&page[i], cpu);
	}
}

/*
 * Adjust the pte(s) so they are ready to go with their new cpu home.
 * On exit, any cpus that were spinning in page fault are now
 * released, get the updated pte and reset their TLBs appropriately.
 */
static void migrate_finish_pte(struct migrating_pte *mpte, int cpu)
{
	pte_t pteval = mpte->pteval;
	struct page *page = pfn_to_page(pte_pfn(pteval));
	pte_t *ptep = mpte->ptep;

	/* Adjust the user PTE. */
	BUG_ON(!pte_migrating(*ptep));
	pteval = pte_donemigrate(set_remote_cache_cpu(pteval, cpu));
	set_pte_at(current->mm, mpte->va, ptep, pteval);

	/* Adjust any kernel PTEs referencing this page. */
	homecache_fix_kpte(page, mpte->npages, 1);
}

/*
 * Given a PTE, inspect it to see if it's one we can migrate; if
 * so, return a pointer to the page so we can try to lock it.
 */
static struct page *pte_to_migratable_page(pte_t pteval, int cpu)
{
	struct page *page;
	int home;

	if (!pte_present(pteval))
		return NULL;

	/* Only migrate pages that are coherently cached on a single cpu. */
	if (hv_pte_get_mode(pteval) != HV_PTE_MODE_CACHE_TILE_L3 ||
	    hv_pte_get_nc(pteval))
		return NULL;

	/* Sanity-check the PTE against the page info. */
	page = pfn_to_page(pte_pfn(pteval));
	home = page_home(page);
	if (home != get_remote_cache_cpu(pteval))
		panic("Candidate PTE %#llx (home %d) has PFN %#lx (home %d)",
		      pteval.val, get_remote_cache_cpu(pteval),
		      pte_pfn(pteval), home);

	/* If we're already homed on this cpu, no need to migrate! */
	if (home == cpu)
		return NULL;

	/* If the cpu is not one the hypervisor can cache-flush, skip it. */
	BUG_ON(home < 0 || home >= NR_CPUS);
	if (!cpu_cacheable(home))
		return NULL;

	/* If no page flags are set, it's probably a device mapping. */
	if ((page->flags & ((1 << __NR_PAGEFLAGS) - 1)) == 0)
		return NULL;

	return page;
}

/* Check that the page is one that we want to migrate. */
static int page_migrates_with_process(pte_t pteval, struct page *page)
{
	/*
	 * If the page is mapped into multiple mm's, we don't migrate
	 * it, since we don't provide try_to_unmap() functionality.
	 * There's also not a clear advantage to migrating it.
	 *
	 * NOTE: This also excludes pages that are mapped twice into
	 * the same mm, but this is a rare case, so we don't worry.
	 * We actually do support migrating a page mapped more than once
	 * (see the is_page_locked() calls in maybe_migrate(), below)
	 * so if we do need to do this later it may not be that hard.
	 *
	 * If the mapcount is zero, it may be a page of memory mapped
	 * by a device driver, in which case we also don't want to try
	 * to migrate it.
	 */
	if (page_mapcount(page) != 1)
		return 0;

	/* Unlikely to map one of these, but might as well check. */
	if (PageHomecacheNomigrate(page))
		return 0;

	return 1;
}

/*
 * We enter with a candidate VA and a flag indicating whether we should
 * use "trylock" instead of lock, and no locks held (other than the
 * mmap_sem held for read).  We return 0 if things went OK, and 1 if
 * we were in "trylock" mode and failed to acquire a lock.
 *
 * First we validate that the PTE is plausible, and return early if not.
 * Then we try to get a lock on the page, and then map and lock the page
 * table.  This is a bit tricky because we have to lock the page before
 * the page table to respect the ordering in mm/rmap.c.  This means we
 * get a tentative page from the pte, then lock it, lock the page table,
 * and validate the PTE.  If the PTE has changed (perhaps because
 * another thread upgraded a zero-page ref to writable while we were
 * working) we try again until the PTE value is stable.  Once we have a
 * stable, migratable PTE, we return.
 *
 * Prior to taking any page or page table locks, we scan the list of
 * locks we are currently holding to avoid double-taking any locks.
 * Note that this means that if we already have a page table lock for
 * some page, we will end up trying to take the page lock after the page
 * table lock, in violation of the rmap.c ordering; but since at that
 * point we must already be in trylock mode, and have already made some
 * progress, it doesn't matter.
 *
 * Note that we must have interrupts enabled during this routine
 * since we are acquiring the page lock and the page table lock.
 */
static int maybe_migrate(struct migrate_state *ms, unsigned long va,
			 unsigned long page_size, int try_lock)
{
	pte_t *ptep;
	pud_t *pud;
	pmd_t *pmd;
	pte_t pteval;
	spinlock_t *ptl;  /* page table lock for "va" */
	struct page *page;
	struct mm_struct *mm = current->mm;
	int took_page_lock, took_page_table_lock;

	/* Find the PTE and the appropriate page table lock. */
	pud = pud_offset(pgd_offset(mm, va), va);
	if (!pud_present(*pud))
		return 0;
	if (pud_huge_page(*pud)) {
		ptep = (pte_t *)pud;
		BUG_ON(page_size < PGDIR_SIZE);
		ptl = &mm->page_table_lock;
	} else {
		pmd = pmd_offset(pud, va);
		if (!pmd_present(*pmd))
			return 0;
		if (pmd_huge_page(*pmd)) {
			ptep = (pte_t *)pmd;
			BUG_ON(page_size < HPAGE_SIZE);
			ptl = &mm->page_table_lock;
		} else {
			ptep = pte_offset_kernel(pmd, va);
			ptl = pte_lockptr(mm, pmd);
		}
	}

	/*
	 * Lock the page table (unless we locked it for a previous page).
	 * We have to do this so it's safe to examine the PTE's page struct.
	 */
	took_page_table_lock = 0;
	if (!is_page_table_locked(ms, ptl)) {
		if (!spin_trylock(ptl)) {
			if (try_lock)
				return 1;
			spin_lock(ptl);
		}
		took_page_table_lock = 1;
	}

 retry:
	/* See if we are interested in this PTE. */
	pteval = *ptep;
	page = pte_to_migratable_page(pteval, ms->cpu);
	if (page == NULL || !page_migrates_with_process(pteval, page)) {
		if (took_page_table_lock)
			spin_unlock(ptl);
		return 0;
	}

	/* Now try to take the page lock. */
	took_page_lock = 0;
	if (!is_page_locked(ms, page)) {
		if (!trylock_page(page)) {
			if (try_lock) {
				if (took_page_table_lock)
					spin_unlock(ptl);
				return 1;
			}

			/*
			 * This is the first page we're trying to acquire,
			 * so we have to take the page lock first to avoid
			 * deadlock with (e.g.) the swapper.  But this
			 * means we have to drop the existing page table
			 * lock, which means we have to bump up the
			 * reference count on the page beforehand, so we
			 * can still validly look at it when we try to lock
			 * it.  Then we have to check the PTE to make sure
			 * it didn't change while we had the PTL dropped.
			 */
			BUG_ON(!took_page_table_lock);
			get_page(page);
			spin_unlock(ptl);
			lock_page(page);
			spin_lock(ptl);
			if (unlikely(!pte_same(*ptep, pteval))) {
				unlock_page(page);
				put_page(page);
				goto retry;
			}

			/*
			 * Drop the extra refcount; we don't need it since
			 * we will leave the PTL locked from now on.
			 */
			put_page(page);
		}

		/* Now that we have the lock, recheck the page. */
		if (!page_migrates_with_process(pteval, page)) {
			unlock_page(page);
			if (took_page_table_lock)
				spin_unlock(ptl);
			return 0;
		}

		took_page_lock = 1;
	}

	/* Record what we migrated and what locks we took out. */
	if (!took_page_lock)
		page = NULL;
	if (!took_page_table_lock)
		ptl = NULL;
	add_migrating_pte(ms, ptl, ptep, va, page_size / PAGE_SIZE, page);
	if (page)
		++homecache_proc_migrated_mapped;

	return 0;
}

/*
 * Walk the user pages and try to start migrating the ones that need
 * it.  We enter holding the mmap_sem for read.  We return 0 if we
 * were able to migrate every page we were interested in, and the VA
 * to restart at if we need to complete this migration pass and then
 * try again.  On exit, the passed migrate_state structure is updated
 * with the list of user PTEs chosen to migrate, and the kernel VA
 * range is updated with any kernel addresses that have to be
 * explicitly flushed.
 *
 * Marking all the pages for migrating is tricky since we have to
 * worry about ABBA deadlock.  If we've already locked some pages and
 * marked them as migrating, then try to lock a new page or a page
 * table, it's possible that some other thread already holds that
 * lock, but is blocked trying to lock, or create a PTE for, a page
 * that we have already started to migrate.  This would be a deadlock,
 * but instead maybe_migrate() bails out (returning a non-zero start
 * va), we short-circuit this routine, complete the whole migration
 * pass for the pages we've already marked for migration, then loop
 * back in homecache_migrate() and retry.  This way we allow the other
 * task to make forward progress, thus allowing us to eventually be
 * able to acquire the lock that we need as well.
 */
static unsigned long migrate_start_user(struct migrate_state *ms,
					unsigned long start_va)
{
	struct task_struct *p = current;
	struct mm_struct *mm = p->mm;
	int is_threaded = (atomic_read(&mm->mm_users) > 1);
	pid_t mypid = current->pid;
	unsigned long usp0 = p->thread.usp0;
	struct vm_area_struct *vm;

	/* Walk user pages and discover which should be migrated. */
	for (vm = mm->mmap; vm != NULL; vm = vm->vm_next) {
		unsigned long va;
		int page_size;

		/* Handle MAP_CACHE_HOME_TASK regions. */
		if (vm->vm_pid != 0) {
			/* Skip regions owned by another task. */
			if (vm->vm_pid != mypid)
				continue;

			/* Update vm_page_prot for subsequent faults. */
			vm->vm_page_prot =
				set_remote_cache_cpu(vm->vm_page_prot,
						     ms->cpu);
		} else {
			/* Don't try to migrate regions with explicit homes */
			if (pte_get_forcecache(vm->vm_page_prot) &&
			    !pte_get_anyhome(vm->vm_page_prot))
				continue;

			/* If threaded, we only migrate the stack. */
			if (is_threaded &&
			    (usp0 < vm->vm_start || usp0 >= vm->vm_end))
				continue;
		}

		/* Walk each page in the region. */
		va = vm->vm_start > start_va ? vm->vm_start : start_va;
		page_size = vma_kernel_pagesize(vm);
		for (; va < vm->vm_end; va += page_size) {
			int try_lock;

			/* If we can't store any more PTE info, retry. */
			if (ms->migrating_index >= MIGRATING_COUNT)
				return va;

			/*
			 * Check this address to see if it needs to
			 * migrate.  If we've already marked page(s) for
			 * migration, use "trylock" to avoid deadlock.
			 * If we get a trylock failure notification,
			 * give up and indicate we should retry.
			 */
			try_lock = (ms->migrating_index != 0);
			if (maybe_migrate(ms, va, page_size, try_lock) != 0)
				return va;
		}
	}

	return 0;
}

static void migrate_stack_and_flush(const struct cpumask *cache_cpumask,
				    struct cpumask *tlb_cpumask,
				    unsigned long va, unsigned long size,
				    HV_Remote_ASID* rem_asids,
				    int num_rem_asids,
				    pte_t stack_pte, pte_t *stack_ptep,
				    unsigned long finv_va)
{
	int rc;

#ifdef CONFIG_DATAPLANE
	struct cpumask tlb_cpumask_copy;
	cpumask_copy(&tlb_cpumask_copy, tlb_cpumask);
	set_remote_flush_count(&tlb_cpumask_copy, +1);
#endif
	BUG_ON(pte_migrating(stack_pte));
	hv_flush_update(HV_FLUSH_EVICT_L2, cache_cpumask,
			tlb_cpumask, va, size, rem_asids, num_rem_asids);
	rc = homecache_migrate_stack_and_flush(stack_pte, va, size,
					       stack_ptep, cache_cpumask,
					       tlb_cpumask, rem_asids,
					       num_rem_asids, finv_va);
	if (rc != 0)
		panic("homecache_migrate_stack_and_flush: %d", rc);
#ifdef CONFIG_DATAPLANE
	set_remote_flush_count(&tlb_cpumask_copy, -1);
#endif
}

/*
 * Kernel tasks only migrate their stack, at most.  So for kernel
 * tasks, we run a minimal version of homecache_trymigrate().
 * Note that we never cache-evict the old stack home, but always
 * use finv_buffer_remote().
 */
void homecache_migrate_kthread(void)
{
	struct thread_info *ti = current_thread_info();
	struct task_struct *p = ti->task;
	struct thread_struct *ts = &p->thread;
	unsigned long stack_va = (unsigned long)ti;
	unsigned long finv_va;
	unsigned long stack_pfn = kaddr_to_pfn((void *)stack_va);
	pte_t *stack_ptep = virt_to_kpte(stack_va);
	pte_t *finv_ptep;
	pte_t stack_pte;
	struct page *stack_page = pfn_to_page(stack_pfn);
	struct cpumask *stack_tlbmask;
	unsigned long flags;
	int stack_home, cpu;

	/* If called from finish_arch_post_lock_switch(), irqs are enabled. */
	local_irq_save(flags);

	/* See if we actually need to do anything. */
	stack_home = page_home(stack_page);
	if (unlikely(stack_home == PAGE_HOME_HASH)) {
		/*
		 * Possible only for the boot idle task during init
		 * before we move it to a properly-homed stack.
		 */
		goto done;
	}
	cpu = smp_processor_id();
	if (unlikely(stack_home == cpu))
		goto done;

	ts->homecache_is_migrating = true;
	he_begin();
	stack_pte = *stack_ptep;
	BUG_ON(pte_huge(stack_pte));
	BUG_ON(stack_home != pte_to_home(stack_pte));
	BUG_ON(stack_home < 0 || stack_home > NR_CPUS);
	stack_pte = set_remote_cache_cpu(stack_pte, cpu);
	stack_tlbmask = (struct cpumask *) p->thread.homecache_tlb_cpumask;
	memcpy(stack_tlbmask, cpu_online_mask->bits, sizeof(*cpu_online_mask));
	finv_va = homecache_map_begin(stack_page, stack_home, &finv_ptep);
	he_add(HE_KKSTACK, cpu, stack_pfn);
	migrate_stack_and_flush(NULL, stack_tlbmask,
				stack_va, THREAD_SIZE, NULL, 0,
				stack_pte, stack_ptep, finv_va);
	homecache_map_end(finv_ptep, finv_va);
	he_add(HE_KKSTACK_DONE, cpu, stack_pfn);
	BUILD_BUG_ON(THREAD_SIZE != PAGE_SIZE);
	set_page_home(stack_page, cpu);
	homecache_proc_migrated_mapped++;
	homecache_proc_migrated_tasks++;
	he_end();
	ts->homecache_is_migrating = false;

	/* Set the homecache_cpu to reflect that we have migrated. */
	ti->homecache_cpu = cpu;

done:
	local_irq_restore(flags);
}

/*
 * Migrate the caching of the current task's pages to its new cpu.
 * Return 0 if we completed successfully, otherwise the VA we should
 * restart at if we faced possible deadlock and gave up part way through.
 * The first invocation must be passed start_va as "0", because this
 * indicates the invocation that will migrate the kernel stack as well.
 */
static unsigned long homecache_trymigrate(struct migrate_state *ms,
					  unsigned long start_va)
{
	struct task_struct *p = current;
	struct page *uninitialized_var(stack_page);
	pte_t *stack_ptep, *finv_ptep;
	pte_t stack_pte;
	int stack_home = 0;
	int old_cpu, cpu = ms->cpu;
	unsigned long end_va;
	unsigned long flags;
	unsigned long stack_pfn = 0, stack_va;
	unsigned long finv_va = 0;
	size_t tlb_flush_len;
	struct cpumask *cache_cpumask;
	int i, other_cpu;
	int migrate_stack;
	bool do_evict;

	/*
	 * For vfork'ed children, just return immediately; the parent
	 * still owns the pages, so we don't want to move any of them.
	 */
	if (p->vfork_done != NULL)
		return 0;

	/* Initialize the migrating_state */
	cpumask_clear(ms->cache_cpumask);
	cpumask_clear(ms->tlb_cpumask);
	ms->num_rem_asids = 0;
	ms->migrating_index = 0;
	ms->high_kaddr = 0;
	ms->low_kaddr = -1UL;

	/*
	 * This should only ever be called just before returning
	 * a task to user-space, but be paranoid and check.
	 */
	BUG_ON(in_interrupt());

	/* Mark user PTEs for migration. */
	down_read(&p->mm->mmap_sem);
	end_va = migrate_start_user(ms, start_va);
	up_read(&p->mm->mmap_sem);

	if (ms->migrating_index == 0) {
		/*
		 * In kstack_hash mode, we won't migrate any
		 * kernel pages, and if we didn't find any
		 * user pages to migrate either, we're done.
		 */
		if (kstack_hash)
			return end_va;
	} else {
		/*
		 * Construct the cpu/ASID vector to flush,
		 * based on what other threads are sharing
		 * this mm.  Once we have real ASID support we
		 * will probably have something like a
		 * cpu/ASID vector in the mm.  For now, we
		 * just construct one manually.
		 */
		for_each_cpu(other_cpu, mm_cpumask(p->mm)) {
			int index = ms->num_rem_asids++;
			HV_Remote_ASID *rem_asid =
				&ms->rem_asids[index];
			rem_asid->x = cpu_x(other_cpu);
			rem_asid->y = cpu_y(other_cpu);
			rem_asid->asid =
				per_cpu(current_asid, other_cpu);
		}
	}

	/*
	 * On our first pass, mark kernel stack for migration.
	 * For kstack_hash, the kernel stack is hash-for-home,
	 * so we never migrate it.
	 */
	migrate_stack = !kstack_hash && (start_va == 0);

	if (migrate_stack) {
		/* See comments above in homecache_migrate_kthread(). */
		stack_va = (unsigned long)(p->stack);
		stack_pfn = kaddr_to_pfn(p->stack);
		stack_ptep = virt_to_kpte(stack_va);
		stack_pte = *stack_ptep;
		stack_page = pfn_to_page(stack_pfn);
		stack_home = page_home(stack_page);
		BUG_ON(pte_huge(stack_pte));
		BUG_ON(stack_home != pte_to_home(stack_pte));
		BUG_ON(stack_home < 0 || stack_home >= NR_CPUS);
		stack_pte = set_remote_cache_cpu(stack_pte, cpu);
		homecache_proc_migrated_mapped++;
		add_kaddr_flush(ms, stack_va, THREAD_SIZE);
	}

	/*
	 * Take out the kpte lock to disable new kernel HIGHMEM mappings.
	 * Disable interrupts (even performance counter interrupts) to
	 * allow this code to run to completion as quickly as possible,
	 * and to avoid the possibility of self-deadlock in an interrupt.
	 */
	local_irq_save(flags);
	interrupt_mask_set_mask(~LINUX_MASKABLE_INTERRUPTS);
	homecache_kpte_lock();
	for (i = 0; i < ms->migrating_index; ++i)
		migrate_start_pte(ms, &ms->migrating[i]);

	/* See if we need to evict the cache on our previous core. */
	do_evict = homecache_migrate_check_evict(ms, migrate_stack);
	if (migrate_stack) {
		if (!do_evict) {
			finv_va = homecache_map_begin(stack_page, stack_home,
						      &finv_ptep);
		} else {
			cpumask_set_cpu(stack_home, ms->cache_cpumask);
		}
	}

	/* Finish figuring out the parameters of the flush call(s). */
	if (ms->low_kaddr > ms->high_kaddr) {
		ms->low_kaddr = 0;
		ms->high_kaddr = 0;
		tlb_flush_len = 0;
	} else {
		cpumask_copy(ms->tlb_cpumask, cpu_online_mask);
		tlb_flush_len = ms->high_kaddr - ms->low_kaddr;
	}
	cache_cpumask =
		cpumask_empty(ms->cache_cpumask) ? NULL : ms->cache_cpumask;

	/*
	 * Issue appropriate TLB and cache flush hypervisor calls.
	 * If we are migrating the stack page, use the assembler helper
	 * to do this, since it migrates the stack PTE and sets
	 * the interrupt critical section mode while it's happening;
	 * otherwise just use regular hypervisor calls from C.
	 */

	if (migrate_stack) {
		he_add(HE_KSTACK, cpu, stack_pfn);
		migrate_stack_and_flush(cache_cpumask, ms->tlb_cpumask,
					ms->low_kaddr, tlb_flush_len,
					ms->rem_asids, ms->num_rem_asids,
					stack_pte, stack_ptep, finv_va);
		he_add(HE_KSTACK_DONE, cpu, stack_pfn);
		BUILD_BUG_ON(THREAD_SIZE != PAGE_SIZE);
		set_page_home(stack_page, cpu);
		if (!do_evict)
			homecache_map_end(finv_ptep, finv_va);
	} else {
		if (tlb_flush_len || ms->num_rem_asids)
			flush_remote(0, 0, NULL, ms->low_kaddr, tlb_flush_len,
				     PAGE_SIZE, ms->tlb_cpumask,
				     ms->rem_asids, ms->num_rem_asids);
		if (cache_cpumask)
			homecache_evict(cache_cpumask);
	}

	/* Do any finvs and reset the page homes. */
	old_cpu = current_thread_info()->homecache_cpu;
	for (i = 0; i < ms->migrating_index; ++i) {
		struct migrating_pte *mpte = &ms->migrating[i];
		bool do_finv =
			!do_evict || pte_to_home(mpte->pteval) != old_cpu;
		migrate_finish_page(&ms->migrating[i], cpu, do_finv);
	}

	/* Release the kpte lock, now that we can safely create new kmaps. */
	homecache_kpte_unlock();

	/*
	 * Finish migrating.  We loop in reverse
	 * order since that way we release any shared locks
	 * after all the PTEs that referenced them.
	 */
	for (i = ms->migrating_index - 1; i >= 0; --i) {
		struct migrating_pte *mpte = &ms->migrating[i];

		/* Validate that we really evicted the page. */
		unsigned long pfn = pte_pfn(mpte->pteval);
		int length = mpte->npages * PAGE_SIZE;
		sim_validate_lines_evicted(PFN_PHYS(pfn), length);

		/* Write the new PTE (and kernel PTE). */
		migrate_finish_pte(mpte, cpu);

		/* Unlock the page and the page table, if necessary. */
		if (mpte->page_lock)
			unlock_page(mpte->page_lock);
		if (mpte->ptl)
			spin_unlock(mpte->ptl);
	}

	interrupt_mask_reset_mask(~LINUX_MASKABLE_INTERRUPTS);
	local_irq_restore(flags);

	return end_va;
}

/*
 * Called to migrate the home cache of any pages associated with the
 * task if the cpu has changed and we are resuming back to userspace.
 */
void homecache_migrate(void)
{
	struct migrate_state ms;
	unsigned long start_va, next_va;
	struct thread_struct *ts = &current->thread;

#ifdef CONFIG_KVM
	if (current_thread_info()->vcpu) {
		/* 
		 * If we are returning to a guest OS, don't try to migrate
		 * here, since we may need to context-switch. Instead do a
		 * vmexit and perform the migration on return to qemu.
		 */
		set_thread_flag(TIF_VIRT_EXIT);
		return;
	}
#endif

	/* kthreadd takes this path, so redirect it to kernel task path. */
	if (current->mm == NULL) {
		homecache_migrate_kthread();
		return;
	}

	/*
	 * Initialize migrate_state to point to some thread_struct state.
	 * This is memory that we need to pass to the hypervisor to
	 * describe the migration, so can't be on the stack itself.
	 */
	BUILD_BUG_ON(sizeof(ts->homecache_cache_cpumask) !=
		     sizeof(struct cpumask));
	ms.cache_cpumask = (struct cpumask *) &ts->homecache_cache_cpumask;
	ms.tlb_cpumask = (struct cpumask *) &ts->homecache_tlb_cpumask;
	ms.rem_asids = ts->homecache_rem_asids;
	ts->homecache_is_migrating = true;
	he_begin();

restart:
	/*
	 * We need interrupts enabled throughout the actual migration
	 * process, in particular so we can handle IPIs to avoid
	 * deadlocks while we are trying to acquire page table locks.
	 */
	local_irq_enable();

	/*
	 * Track the cpu number that we are migrating to.  We don't
	 * use smp_processor_id() during migration, so we perform a
	 * consistent migration, but we recheck at the end in case
	 * we ended up rescheduled to a new cpu before we returned.
	 */
	ms.cpu = raw_smp_processor_id();
	homecache_proc_migrated_tasks++;

	/*
	 * If we hit a potential deadlock (a page or page table
	 * locked while we had other pages marked for migration) we
	 * just complete migrating the pages we were holding, then
	 * go back and rescan and try to pick up some more pages.
	 */
	start_va = 0;
	while ((next_va = homecache_trymigrate(&ms, start_va)) != 0 &&
	       raw_smp_processor_id() == ms.cpu) {
		BUG_ON(next_va <= start_va);
		start_va = next_va;
	}
	current_thread_info()->homecache_cpu = ms.cpu;

	if (unlikely(current->ptrace & PT_TRACE_MIGRATE)) {
		current->ptrace_message = ms.cpu;
		ptrace_notify((PTRACE_EVENT_MIGRATE << 8) | SIGTRAP);
	}

	/* Once we've disabled irqs we won't change cpus. */
	local_irq_disable();

	if (unlikely(ms.cpu != raw_smp_processor_id()))
		goto restart;

	he_end();
	ts->homecache_is_migrating = false;
}

/* Set up homecache-related data in a new zone struct. */
void homecache_init_zone_lists(struct zone *zone)
{
	int i;

	for (i = 0; i < NR_ZONE_HOMECACHE_LISTS; i++)
		INIT_LIST_HEAD(&zone->homecache_list[i]);
	zone->lru_homecache_list = 0;
}

static ctl_table homecache_table[] = {
	{
		.procname	= "migrated_tasks",
		.data		= &homecache_proc_migrated_tasks,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.procname	= "migrated_mapped_pages",
		.data		= &homecache_proc_migrated_mapped,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.procname	= "migrated_unmapped_pages",
		.data		= &homecache_proc_migrated_unmapped,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.procname	= "sequestered_pages_at_free",
		.data		= &homecache_proc_sequestered_free,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.procname	= "coherent_sequestered_purges",
		.data		= &homecache_proc_coherent_sequestered_purge,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.procname	= "sequestered_purges",
		.data		= &homecache_proc_sequestered_purge,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.procname	= "incoherent_finv_pages",
		.data		= &homecache_proc_incoherent_finv_pages,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.procname	= "max_sequestered_pages",
		.data		= &homecache_proc_max_sequestered_pages,
		.maxlen		= sizeof(unsigned long),
		.mode		= 0644,
		.proc_handler	= &proc_doulongvec_minmax
	},
	{
		.procname	= "migrate_finv_max",
		.data		= &homecache_proc_migrate_finv_max,
		.maxlen		= sizeof(unsigned long),
		.mode		= 0644,
		.proc_handler	= &proc_doulongvec_minmax
	},
	{
		.procname	= "migrate_evict",
		.data		= &homecache_proc_migrate_evict,
		.maxlen		= sizeof(unsigned long),
		.mode		= 0644,
		.proc_handler	= &proc_doulongvec_minmax
	},
	{}
};

static ctl_table tile_table[] = {
	{
		.procname	= "homecache",
		.mode		= 0555,
		.child		= homecache_table,
	},
	{},
};

static ctl_table root[] = {
	{
		.procname	= "tile",
		.child		= tile_table,
	},
	{},
};

static int unsequesterd(void *p)
{
	for (;;) {
		if (homecache_count_sequestered_pages(1, 1) >
			homecache_proc_max_sequestered_pages)
			homecache_recover_free_pages();

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ * 5);
	}
	return 0;
}

static int __init homecache_init(void)
{
	struct task_struct *tsk_unsequesterd;

	register_sysctl_table(root);

	tsk_unsequesterd = kthread_run(unsequesterd, NULL, "unsequesterd");
	if (IS_ERR(tsk_unsequesterd)) {
		tsk_unsequesterd = NULL;
		pr_err("Failed to start unsequesterd\n");
	} else {
		tsk_unsequesterd->prio = MAX_PRIO;
		tsk_unsequesterd->static_prio = MAX_PRIO;
		tsk_unsequesterd->normal_prio = MAX_PRIO;
	}
	return 0;
}
subsys_initcall(homecache_init);

#endif /* CONFIG_HOMECACHE */
