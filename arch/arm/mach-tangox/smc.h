#ifndef __ARCH_ARM_MACH_TANGOX_SMC_H
#define __ARCH_ARM_MACH_TANGOX_SMC_H

extern void tangox_smc1(u32 fn, u32 arg);
extern u32 tangox_modify_auxcoreboot0(u32 set_mask, u32 clear_mask);
extern void tangox_auxcoreboot_addr(u32 cpu_addr);

#endif
