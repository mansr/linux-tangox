
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2011 Sigma Designs, Inc.
 */

#ifndef __ASM_MACH_TANGO4_IRQ_H
#define __ASM_MACH_TANGO4_IRQ_H

#define MIPS_CPU_IRQ_BASE 0
#define NR_IRQS 256
#ifdef GIC_NUM_INTRS
#undef GIC_NUM_INTRS
#define GIC_NUM_INTRS	8
#endif

/* GIC's Nomenclature for Core Interrupt Pins on the Malta */
#define GIC_CPU_INT0		0 /* Core Interrupt 2 	*/
#define GIC_CPU_INT1		1 /* .			*/
#define GIC_CPU_INT2		2 /* .			*/
#define GIC_CPU_INT3		3 /* .			*/
#define GIC_CPU_INT4		4 /* .			*/
#define GIC_CPU_INT5		5 /* Core Interrupt 5   */

#endif
