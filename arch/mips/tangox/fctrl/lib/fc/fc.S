/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/* 
 * Assembly routine to implement the change of frequency
 */

#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>

#include <asm/tango2/emhwlib_registers_tango2.h>
#include <asm/tango2/emhwlib_dram_tango2.h>
#include <asm/tango2/emhwlib_lram_others.h>

#define STACK_SIZE		0x400

#ifndef INPUT_TABLE_SIZE
#define INPUT_TABLE_SIZE    32
#endif

#ifdef WITH_IR
#define IR_FLAG  1
#else
#define IR_FLAG  0
#endif

#ifdef WITH_FIP
#define FIP_FLAG  2
#else
#define FIP_FLAG  0
#endif

#ifdef WITH_ETH
#define ETH_FLAG  4
#else
#define ETH_FLAG  0
#endif

#define KSEG1_ADDR 0xa0000000

#define DELAY(reg1,reg2,value) 		\
		li reg1,value;		\
		addiu reg2,zero,0;	\
	100: 	bne reg1,reg2,100b;	\
		addiu reg2,reg2,1;

#if defined(FC_DEBUG) && 0
#define DEBUG(value)			\
		addiu sp,sp,-16; 	\
		sw a0,0(sp);		\
		li a0,value;		\
		jal uart_putc;		\
		nop;			\
		jal uart_putc;		\
		li a0,13;		\
		jal uart_putc;		\
		li a0,10;		\
		lw a0,0(sp);		\
		addiu sp,sp,16;	
#else
#define DEBUG(value)		
#endif

NESTED(__frequency_set, STACK_SIZE, ra)

	.align	5
handler_header:
entry_point:
	.word	_start
params_start:
	.word	param_area
ir_table:
	.word	ir_keys
fip_table:
	.word	fip_keys
return_key:
	.word	0
time_elapse:
	.word	0
input_ctrl:
	.word	(IR_FLAG | FIP_FLAG | ETH_FLAG)
cnt_low:
	.word	0
cnt_high:
	.word	0

	.align	5
param_area:
setting_pll3:
	.word   0	/* pll3 */
setting_pll2:
	.word   0	/* pll2 */
setting_pll1:
	.word   0	/* pll1 */
setting_pll0:
	.word   0	/* pll0 */
setting_mux:
	.word   0	/* mux */
setting_d0cfg:
	.word   0	/* d0cfg */
setting_d0delay:
	.word   0	/* d0delay */
setting_d1cfg:
	.word   0	/* d1cfg */
setting_d1delay:
	.word   0	/* d1delay */
setting_standby:
	.word	0	/* standby mode */
setting_desc:
	.word	0	/* desc */
setting_gpio_dir:
	.word	0	/* gpio dir */
setting_gpio_data:
	.word	0	/* gpio data */

	.align	5
old_pll3:
	.word	0	/* for saving old pll3 */
old_mux:
	.word	0	/* for saving old mux */
old_sp:
	.word	0	/* for sp */

	.align	5
fip_keys:
	.fill	INPUT_TABLE_SIZE
ir_keys:
	.fill	INPUT_TABLE_SIZE

	/* all cache should be flushed (i.e. written back), interrupt should be disabled at this point */

	.set noreorder
	.set mips32r2

	.align	5
_start:
	/* Setup the stack in local RAM */
	addiu	sp,sp,-64
	sw	ra,0(sp)
	sw	sp,4(sp)
	sw	t0,8(sp)
	sw	t1,12(sp)
	sw	t2,16(sp)
	sw	t3,20(sp)
	sw	t4,24(sp)
	sw	t5,28(sp)
	sw	t6,32(sp)
	sw	t7,36(sp)

	la	t0,old_sp
	sw	sp,0(t0)
	li	sp,KSEG1_ADDR+REG_BASE_cpu_block+LR_STACKTOP-16

	DEBUG('0')

	/* Set DRAM0/1 to auto-refresh mode */
	li	t6,KSEG1_ADDR+REG_BASE_dram_controller_0+G2L_RESET_CONTROL
	li	t7,KSEG1_ADDR+REG_BASE_dram_controller_1+G2L_RESET_CONTROL
	addiu	t3,zero,1
	sw	t3,0(t6)
	sw	t3,0(t7)
	sync

	li	t4,KSEG1_ADDR+REG_BASE_dram_controller_0+DRAM_dunit_delay0_ctrl
	li	t5,KSEG1_ADDR+REG_BASE_dram_controller_1+DRAM_dunit_delay0_ctrl
	lui	t3,0x8000
1:
	lw	t0,0(t4)  /* Wait for the MSB of both delay0_ctrl to be set */
	lw	t1,0(t5)
	and	t0,t0,t3
	and	t0,t0,t1
	beq	t0,zero,1b
	nop

	/* Set CPU/SYS/DSP to new divider value, assume PLL is attached here */
	li	t0,KSEG1_ADDR+REG_BASE_system_block+SYS_sysclk_mux
	la	t1,setting_mux
	la	t3,old_mux
	lw	t2,0(t0)
	sw	t2,0(t3)
	lw	t2,0(t1)
	sw	t2,0(t0)
	sync

	DELAY	(t1,t2,255)

	/* Detach PLL from MUX, assumed it's been attached before */
	la	t1,setting_mux
	lw	t2,0(t1)
	addiu	t2,t2,-1
	sw	t2,0(t0)
	sync

	DELAY	(t0,t1,255)
	
	/* Set PLL3 to new frequency */
	li	t0,KSEG1_ADDR+REG_BASE_system_block+SYS_clkgen3_pll
	la	t1,setting_pll3
	la	t3,old_pll3
	lw	t2,0(t0)
	sw	t2,0(t3)
	lw	t2,0(t1)
	sw	t2,0(t0)
	sync

	DELAY	(t0,t1,0x28000)

	/* Re-attach PLL to MUX */
	li	t0,KSEG1_ADDR+REG_BASE_system_block+SYS_sysclk_mux
	la	t1,setting_mux
	lw	t2,0(t1)
	sw	t2,0(t0)
	sync

	DELAY	(t0,t1,1024)

#ifdef WITH_INPUT_POLLING

	/* Check do we need to enter standby mode? */
	la	t0,setting_standby
	lw	t1,0(t0)
	beq	t1,zero,3f
	nop

	DEBUG('1')

2:	/* Yes, we do */
	addiu	v0,zero,0
	la	a0,ir_keys
	la	a1,fip_keys
	la	a2,handler_header
	jal	polling_input
	nop

	la	t0,return_key
	sw	v0,0(t0)

	DEBUG('2')

	DELAY	(t1,t2,255)

	/* Set CPU/SYS/DSP to old divider value, PLL is attached */
	li	t0,KSEG1_ADDR+REG_BASE_system_block+SYS_sysclk_mux
	la	t1,old_mux
	lw	t2,0(t1)
	sw	t2,0(t0)
	sync

	DELAY	(t1,t2,255)

	/* Detach PLL from MUX, assumed it's been attached before */
	la	t1,old_mux
	lw	t2,0(t1)
	addiu	t2,t2,-1
	sw	t2,0(t0)
	sync

	DELAY	(t0,t1,255)

	/* Set PLL3 back to old frequency */
	li	t0,KSEG1_ADDR+REG_BASE_system_block+SYS_clkgen3_pll
	la	t1,old_pll3
	lw	t2,0(t1)
	sw	t2,0(t0)
	sync

	DELAY	(t1,t2,0x28000)

	/* Re-attach PLL from MUX */
	li	t0,KSEG1_ADDR+REG_BASE_system_block+SYS_sysclk_mux
	la	t1,old_mux
	lw	t2,0(t1)
	sw	t2,0(t0)
	sync

	DELAY	(t0,t1,255)

	DEBUG('3')

	b	4f
	nop

3:

#endif
	la	t0,return_key
	la	t1,time_elapse
	sw	zero,0(t0)
	sw	zero,0(t1)

	/* Re-program DRAM0/1 configuration */
	li	t0,KSEG1_ADDR+REG_BASE_dram_controller_0+DRAM_dunit_cfg
	li	t1,KSEG1_ADDR+REG_BASE_dram_controller_1+DRAM_dunit_cfg

	la	t2,setting_d0cfg
	lw	t3,0(t2)
	sw	t3,0(t0)
	la	t2,setting_d1cfg
	lw	t3,0(t2)
	sw	t3,0(t1)

	addiu	t4,t0,(DRAM_dunit_delay0_ctrl-DRAM_dunit_cfg)
	addiu	t5,t1,(DRAM_dunit_delay0_ctrl-DRAM_dunit_cfg)

	la	t2,setting_d0delay
	lw	t3,0(t2)
	sw	t3,0(t4)
	la	t2,setting_d1delay
	lw	t3,0(t2)
	sw	t3,0(t5)
	sync

	DELAY	(t6,t7,255)

4:
	/* bring DRAM0/1 out of auto-refresh mode */
	li	t6,KSEG1_ADDR+REG_BASE_dram_controller_0+G2L_RESET_CONTROL
	li	t7,KSEG1_ADDR+REG_BASE_dram_controller_1+G2L_RESET_CONTROL
	sw	zero,0(t6)
	sw	zero,0(t7)
	sync

	DEBUG('4')

	DELAY	(t6,t7,4096)

	/* Restore stack */
	la	t1,old_sp
	lw	sp,0(t1)

	lw	ra,0(sp)
	lw	t0,8(sp)
	lw	t1,12(sp)
	lw	t2,16(sp)
	lw	t3,20(sp)
	lw	t4,24(sp)
	lw	t5,28(sp)
	lw	t6,32(sp)
	lw	t7,36(sp)

	addiu	sp,sp,64

	jr	ra
	addiu	v0,zero,0

	.set reorder

END(__frequency_set)

LEAF(__sync)
	sync
	jr	ra
	nop
END(__sync)

LEAF(__iob)
	.set noreorder
	sync
	li	t0,KSEG1_ADDR+REG_BASE_cpu_block+LR_STACKTOP
	jr	ra
	lw	zero,0(t0)
END(__iob)

