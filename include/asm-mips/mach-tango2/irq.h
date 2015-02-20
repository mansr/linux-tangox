
#ifndef __ASM_MACH_TANGO2_IRQ_H
#define __ASM_MACH_TANGO2_IRQ_H

#include <linux/config.h>

#ifndef MIPS_CPU_IRQ_BASE
#ifdef CONFIG_I8259
#define MIPS_CPU_IRQ_BASE 16
#else
#define MIPS_CPU_IRQ_BASE 0
#endif /* CONFIG_I8259 */
#endif

#define NR_IRQS 256

#endif
