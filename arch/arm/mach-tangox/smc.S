/*
 * Based on OMAP4 code.
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Written by Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Copyright (C) 2012 Ivaylo Dimitrov <freemangordon@abv.bg>
 * Copyright (C) 2013 Pali Rohár <pali.rohar@gmail.com>
 *
 * This program is free software,you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/linkage.h>

ENTRY(tangox_smc1)
	push	{r2-r12, lr}
	mov	r12, r0
	mov 	r0, r1
	dsb
	smc	#0
	pop	{r2-r12, pc}
ENDPROC(tangox_smc1)

ENTRY(tangox_modify_auxcoreboot0)
	push	{r1-r12, lr}
	ldr	r12, =0x104
	dsb
	smc	#0
	pop	{r1-r12, pc}
ENDPROC(tangox_modify_auxcoreboot0)

ENTRY(tangox_auxcoreboot_addr)
	push	{r2-r12, lr}
	ldr	r12, =0x105
	dsb
	smc	#0
	pop	{r2-r12, pc}
ENDPROC(tangox_auxcoreboot_addr)
