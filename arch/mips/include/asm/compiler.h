/*
 * Copyright (C) 2004, 2007  Maciej W. Rozycki
 * Copyright (C) 2009  Wu Zhangjin, wuzj@lemote.com
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#ifndef _ASM_COMPILER_H
#define _ASM_COMPILER_H

#if __GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4)
#define GCC_IMM_ASM() "n"
#define GCC_REG_ACCUM "$0"
#else
#define GCC_IMM_ASM() "rn"
#define GCC_REG_ACCUM "accum"
#endif

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 4)
#define GCC_NO_H_CONSTRAINT
#ifdef CONFIG_64BIT
typedef unsigned int uintx_t __attribute__((mode(TI)));
#else
typedef u64 uintx_t;
#endif
#endif

#endif /* _ASM_COMPILER_H */
