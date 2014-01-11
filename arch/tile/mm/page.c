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
 */

#include <linux/export.h>
#include <asm/page.h>

#ifdef __tilegx__
/*
 * Use non-temporal stores to avoid polluting the cache.  The
 * non-temporal stores fill the write-buffer, which is then written to
 * memory rather than the cache on a cache-miss.
 */
void clear_page(void *page)
{
	u64 *v64 = (u64 *)page;
	u64 *end_of_page = (u64 *)(page + PAGE_SIZE);
	do {
		__insn_stnt(v64++, 0);
		__insn_stnt(v64++, 0);
		__insn_stnt(v64++, 0);
		__insn_stnt(v64++, 0);
		__insn_stnt(v64++, 0);
		__insn_stnt(v64++, 0);
		__insn_stnt(v64++, 0);
		__insn_stnt(v64++, 0);
	} while (v64 < end_of_page);
}
EXPORT_SYMBOL(clear_page);
#endif
