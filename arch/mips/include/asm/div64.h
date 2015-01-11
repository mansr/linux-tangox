/*
 * Copyright (C) 2000, 2004  Maciej W. Rozycki
 * Copyright (C) 2003, 07 Ralf Baechle (ralf@linux-mips.org)
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#ifndef __ASM_DIV64_H
#define __ASM_DIV64_H

#include <linux/compiler.h>
#include <linux/types.h>

#if BITS_PER_LONG == 64

#include <asm-generic/div64.h>

/*
 * No traps on overflows for any of these...
 */

#define __div64_32(n, base)						\
({									\
	unsigned long __cf, __tmp, __tmp2, __i;				\
	unsigned long __quot32, __mod32;				\
	unsigned long __high, __low;					\
	unsigned long long __n;						\
									\
	__high = *__n >> 32;						\
	__low = __n;							\
	__asm__(							\
	"	.set	push					\n"	\
	"	.set	noat					\n"	\
	"	.set	noreorder				\n"	\
	"	move	%2, $0					\n"	\
	"	move	%3, $0					\n"	\
	"	b	1f					\n"	\
	"	 li	%4, 0x21				\n"	\
	"0:							\n"	\
	"	sll	$1, %0, 0x1				\n"	\
	"	srl	%3, %0, 0x1f				\n"	\
	"	or	%0, $1, %5				\n"	\
	"	sll	%1, %1, 0x1				\n"	\
	"	sll	%2, %2, 0x1				\n"	\
	"1:							\n"	\
	"	bnez	%3, 2f					\n"	\
	"	 sltu	%5, %0, %z6				\n"	\
	"	bnez	%5, 3f					\n"	\
	"2:							\n"	\
	"	 addiu	%4, %4, -1				\n"	\
	"	subu	%0, %0, %z6				\n"	\
	"	addiu	%2, %2, 1				\n"	\
	"3:							\n"	\
	"	bnez	%4, 0b\n\t"					\
	"	 srl	%5, %1, 0x1f\n\t"				\
	"	.set	pop"						\
	: "=&r" (__mod32), "=&r" (__tmp),				\
	  "=&r" (__quot32), "=&r" (__cf),				\
	  "=&r" (__i), "=&r" (__tmp2)					\
	: "Jr" (base), "0" (__high), "1" (__low));			\
									\
	(__n) = __quot32;						\
	__mod32;							\
})

#elif defined(CONFIG_CPU_MIPS32) && GCC_VERSION >= 40300

#include <asm/bug.h>

extern uint32_t __div64_32(uint64_t *dividend, uint32_t divisor);

/*
 * If the divisor happens to be constant, we determine the appropriate
 * inverse at compile time to turn the division into a few inline
 * multiplications instead which is much faster.
 * (It is unfortunate that gcc doesn't perform all this internally.)
 */
#define do_div(n, base)							\
({									\
	unsigned int __r, __b = (base);					\
	if (likely(((n) >> 32) == 0)) {					\
		__r = (u32)(n) % __b;					\
		(n) = (u32)(n) / __b;					\
	} else if (!__builtin_constant_p(__b) || __b == 0) {		\
		/* non-constant divisor (or zero): slow path */		\
		__r = __div64_32(&(n), __b);				\
	} else if ((__b & (__b - 1)) == 0) {				\
		/* Trivial: __b is constant and a power of 2 */		\
		/* gcc does the right thing with this code.  */		\
		__r = n;						\
		__r &= (__b - 1);					\
		n /= __b;						\
	} else {							\
		/* Multiply by inverse of __b: n/b = n*(p/b)/p       */	\
		/* We rely on the fact that most of this code gets   */	\
		/* optimized away at compile time due to constant    */	\
		/* propagation and only a couple inline assembly     */	\
		/* instructions should remain. Better avoid any      */	\
		/* code construct that might prevent that.           */	\
		unsigned long long __res, __x, __t, __m, __n = n;	\
		unsigned int __c, __p, __z = 0;				\
		/* preserve low part of n for reminder computation */	\
		__r = __n;						\
		/* determine number of bits to represent __b */		\
		__p = 1 << __div64_fls(__b);				\
		/* compute __m = ((__p << 64) + __b - 1) / __b */	\
		__m = (~0ULL / __b) * __p;				\
		__m += (((~0ULL % __b + 1) * __p) + __b - 1) / __b;	\
		/* compute __res = __m*(~0ULL/__b*__b-1)/(__p << 64) */	\
		__x = ~0ULL / __b * __b - 1;				\
		__res = (__m & 0xffffffff) * (__x & 0xffffffff);	\
		__res >>= 32;						\
		__res += (__m & 0xffffffff) * (__x >> 32);		\
		__t = __res;						\
		__res += (__x & 0xffffffff) * (__m >> 32);		\
		__t = (__res < __t) ? (1ULL << 32) : 0;			\
		__res = (__res >> 32) + __t;				\
		__res += (__m >> 32) * (__x >> 32);			\
		__res /= __p;						\
		/* Now sanitize and optimize what we've got. */		\
		if (~0ULL % (__b / (__b & -__b)) == 0) {		\
			/* those cases can be simplified with: */	\
			__n /= (__b & -__b);				\
			__m = ~0ULL / (__b / (__b & -__b));		\
			__p = 1;					\
			__c = 1;					\
		} else if (__res != __x / __b) {			\
			/* We can't get away without a correction    */	\
			/* to compensate for bit truncation errors.  */	\
			/* To avoid it we'd need an additional bit   */	\
			/* to represent __m which would overflow it. */	\
			/* Instead we do m=p/b and n/b=(n*m+m)/p.    */	\
			__c = 1;					\
			/* Compute __m = (__p << 64) / __b */		\
			__m = (~0ULL / __b) * __p;			\
			__m += ((~0ULL % __b + 1) * __p) / __b;		\
		} else {						\
			/* Reduce __m/__p, and try to clear bit 31   */	\
			/* of __m when possible otherwise that'll    */	\
			/* need extra overflow handling later.       */	\
			unsigned int __bits = -(__m & -__m);		\
			__bits |= __m >> 32;				\
			__bits = (~__bits) << 1;			\
			/* If __bits == 0 then setting bit 31 is     */	\
			/* unavoidable.  Simply apply the maximum    */	\
			/* possible reduction in that case.          */	\
			/* Otherwise the MSB of __bits indicates the */	\
			/* best reduction we should apply.           */	\
			if (!__bits) {					\
				__p /= (__m & -__m);			\
				__m /= (__m & -__m);			\
			} else {					\
				__p >>= __div64_fls(__bits);		\
				__m >>= __div64_fls(__bits);		\
			}						\
			/* No correction needed. */			\
			__c = 0;					\
		}							\
		/* Now we have a combination of 2 conditions:        */	\
		/* 1) whether or not we need a correction (__c), and */	\
		/* 2) whether or not there might be an overflow in   */	\
		/*    the cross product (__m & ((1<<63) | (1<<31)))  */	\
		/* Select the best insn combination to perform the   */	\
		/* actual __m * __n / (__p << 64) operation.         */	\
		if (!__c) {						\
			__res = (u64)(u32)__m * (u32)__n >> 32;		\
		} else if (!(__m & ((1ULL << 63) | (1ULL << 31)))) {	\
			asm (	"maddu	%q0, %L1, %L2	\n"		\
				: "=ka"(__res)				\
				: "r"(__m), "r"(__n), "0"(__m));	\
			__res >>= 32;					\
		} else {						\
			__t = __m;					\
			asm (	"maddu	%L3, %L4	\n"             \
				"mflo	%L0		\n"             \
				"mfhi	%M0		\n"             \
				"sltu	%1,  %L0, %L3	\n"		\
				"addu	%L0, %M0, %1	\n"		\
				"sltu	%M0, %L0, %M3	\n"		\
				: "=&r"(__res), "=&r"(__c), "+&x"(__t)	\
				: "r"(__m), "r"(__n));			\
		}							\
		if (!(__m & ((1ULL << 63) | (1ULL << 31)))) {		\
			asm (	"maddu	%M2, %L3	\n"		\
				"maddu	%L2, %M3	\n"		\
				"mfhi	%1		\n"		\
				"mthi	$0		\n"		\
				"mtlo	%1		\n"		\
				"maddu	%M2, %M3	\n"		\
				: "+&x"(__res), "=&r"(__c)		\
				: "r"(__m), "r"(__n));			\
		} else {						\
			asm (	"maddu	%M3, %L4	\n"		\
				"mfhi	%2		\n"		\
				"maddu	%L3, %M4	\n"		\
				"mfhi	%1		\n"		\
				"sltu	%2,  %1,  %2	\n"		\
				"mtlo	%1		\n"		\
				"mthi	%2		\n"		\
				"maddu	%M3, %M4	\n"		\
				: "+&x"(__res), "=&r"(__z), "=&r"(__c)	\
				: "r"(__m), "r"(__n));			\
		}							\
		__res /= __p;						\
		/* The reminder can be computed with 32-bit regs     */	\
		/* only, and gcc is good at that.                    */	\
		{							\
			unsigned int __res0 = __res;			\
			unsigned int __b0 = __b;			\
			__r -= __res0 * __b0;				\
		}							\
		/* BUG_ON(__r >= __b || __res * __b + __r != n); */	\
		n = __res;						\
	}								\
	__r;								\
})

/* our own fls implementation to make sure constant propagation is fine */
#define __div64_fls(bits)						\
({									\
	unsigned int __left = (bits), __nr = 0;				\
	if (__left & 0xffff0000) __nr += 16, __left >>= 16;		\
	if (__left & 0x0000ff00) __nr +=  8, __left >>=  8;		\
	if (__left & 0x000000f0) __nr +=  4, __left >>=  4;		\
	if (__left & 0x0000000c) __nr +=  2, __left >>=  2;		\
	if (__left & 0x00000002) __nr +=  1;				\
	__nr;								\
})

#else

#include <asm-generic/div64.h>

#endif /* BITS_PER_LONG == 64 */

#endif /* __ASM_DIV64_H */
