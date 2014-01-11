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

#ifndef _ASM_TILE_ASM_H
#define _ASM_TILE_ASM_H

#ifdef __ASSEMBLY__
# define __ASM_FORM(x)	x
# define __ASM_EX_SEC	.section __ex_table, "a"
#else
# define __ASM_FORM(x)	" " #x " "
# define __ASM_EX_SEC	" .section __ex_table,\"a\"\n"
#endif

#ifndef __tilegx__
# define __ASM_SEL(a, b) __ASM_FORM(a)
#else
# define __ASM_SEL(a, b) __ASM_FORM(b)
#endif

#define _ASM_PTR	__ASM_SEL(.long, .quad)
#define _ASM_ALIGN	__ASM_SEL(.balign 4, .balign 8)

/* Exception table entry */
#ifdef __ASSEMBLY__
# define _ASM_EXTABLE(from, to)	    \
	__ASM_EX_SEC ;		    \
	_ASM_ALIGN ;		    \
	_ASM_PTR from , to ;	    \
	.previous
#else
# define _ASM_EXTABLE(from, to) \
	__ASM_EX_SEC	\
	_ASM_ALIGN "\n" \
	_ASM_PTR #from "," #to "\n" \
	" .previous\n"
#endif

#endif /* _ASM_TILE_ASM_H */
