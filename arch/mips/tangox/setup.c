/*
 * Copyright 2001 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * Copyright (C) 2007 Sigma Designs, Inc.
 * arch/mips/tangox/setup.c
 *     The setup file for tango2/tango3.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <asm/reboot.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/time.h>
#include <asm/serial.h>

#include "setup.h"

#if defined(CONFIG_TANGO2) && defined(CONFIG_TANGOX_USE_TLB_REMAP_DRAM1)
extern unsigned long em86xx_tlb_dram1_map_base;
extern unsigned long em86xx_tlb_dram1_map_size;
#endif

/*
 * helpers to access cpu block registers
 */
#define RD_CPU_REG32(r)	\
		gbus_read_reg32(REG_BASE_cpu_block + (r))

#define WR_CPU_REG32(r, v)	\
		gbus_write_reg32(REG_BASE_cpu_block + (r), (v))

#ifdef CONFIG_PCI
extern void tangox_pci_shutdown(void);
#endif

#ifdef CONFIG_TANGO2
/*
 * we use xrpc to reboot
*/
struct xrpc_block_header {
	u32 callerid;
	u32 xrpcid;

	u32 param0;
	u32 param1;
	u32 param2;
	u32 param3;
	u32 param4;

	u32 headerandblocksize;
};

#define XRPC_ID_REBOOT		19
#define SOFT_IRQ_XRPC		(1 << 4)
#endif

void tangox_machine_restart(char *command)
{
	unsigned long tmp;
	int i;
#ifdef CONFIG_TANGO2
 	struct xrpc_block_header *pB;
	unsigned long base_addr;
	int loop;
#endif

        local_irq_disable();

#ifdef CONFIG_PCI
	tangox_pci_shutdown();
#endif

	/* Resetting TangoX EHCI */
	tmp = gbus_read_reg32(REG_BASE_host_interface + 0x1410);
	tmp &= ~1;
	gbus_write_reg32(REG_BASE_host_interface + 0x1410, tmp);
	mdelay(5);

	/* Resetting TangoX OHCI */
	gbus_write_reg32(REG_BASE_host_interface + 0x1514, 1<<31);
	gbus_write_reg32(REG_BASE_host_interface + 0x1504, 0);
	mdelay(5);

	/* Resetting internal USB PHY in USB Control space */
	tmp = gbus_read_reg32(REG_BASE_host_interface + 0x1700);
	gbus_write_reg32(REG_BASE_host_interface + 0x1700, tmp | 1);
	udelay(30);
	gbus_write_reg32(REG_BASE_host_interface + 0x1700, tmp);
	mdelay(5);

	/* Resetting internal OHCI in USB OHCI space*/
	tmp = gbus_read_reg32(REG_BASE_host_interface + 0x1508);
	gbus_write_reg32(REG_BASE_host_interface + 0x1508, tmp | 0x01);

	/* Reseting OHCI dpll, it says the bit is for simulation */
	tmp = gbus_read_reg32(REG_BASE_host_interface + 0x1700);
	gbus_write_reg32(REG_BASE_host_interface + 0x1700, tmp | (1<<19));
	mdelay(1);

#if defined(CONFIG_TANGO2) 
	/* Resetting ethernet interface */
	gbus_write_reg32(REG_BASE_host_interface + 0x7018, 0);
	gbus_write_reg32(REG_BASE_host_interface + 0x701c, 0);
	gbus_write_reg32(REG_BASE_host_interface + 0x7000, 1);
	for (i = 0; (i < 10) && (gbus_read_reg32(REG_BASE_host_interface + 0x7000) & 1); i++)
		mdelay(1);

	/* Resetting Video, MPEG0/MPEG1 blocks */ 
	gbus_write_reg32(REG_BASE_display_block + G2L_RESET_CONTROL, 3);
	gbus_write_reg32(REG_BASE_mpeg_engine_0 + G2L_RESET_CONTROL, 3);
	gbus_write_reg32(REG_BASE_mpeg_engine_1 + G2L_RESET_CONTROL, 3);
	udelay(1);
	gbus_write_reg32(REG_BASE_display_block + G2L_RESET_CONTROL, 2);
	gbus_write_reg32(REG_BASE_mpeg_engine_0 + G2L_RESET_CONTROL, 2);
	gbus_write_reg32(REG_BASE_mpeg_engine_1 + G2L_RESET_CONTROL, 2);

	/* Resetting Transport demux block */
	gbus_write_reg32(REG_BASE_demux_engine + G2L_RESET_CONTROL, 3);
	udelay(1);
	gbus_write_reg32(REG_BASE_demux_engine + G2L_RESET_CONTROL, 2);

	/* Resetting Audio0/1, host interface blocks */
	gbus_write_reg32(REG_BASE_audio_engine_0 + G2L_RESET_CONTROL, 3);
	gbus_write_reg32(REG_BASE_audio_engine_1 + G2L_RESET_CONTROL, 3);
	gbus_write_reg32(REG_BASE_host_interface + G2L_RESET_CONTROL, 3);
	udelay(1);
	gbus_write_reg32(REG_BASE_audio_engine_0 + G2L_RESET_CONTROL, 2);
	gbus_write_reg32(REG_BASE_audio_engine_1 + G2L_RESET_CONTROL, 2);
	gbus_write_reg32(REG_BASE_host_interface + G2L_RESET_CONTROL, 2);
#endif

	/* restore remap registers to boot state */
	for (i = 0; 
#ifdef CONFIG_TANGO2
		i < 5; 
#elif defined(CONFIG_TANGO3)
		i < 8; 
#endif
		i++) {
		gbus_write_reg32(REG_BASE_cpu_block + CPU_remap + i * 4, em8xxx_remap_registers[i]);
	}
	iob();

	/* Now to handle CPU side */

#ifdef CONFIG_TANGO3
#ifndef CONFIG_TANGOX_FIXED_FREQUENCIES
	/* Using watchdog to trigger reset here */
	gbus_write_reg8(REG_BASE_system_block + SYS_watchdog_configuration + 3, 0x80); 
	gbus_write_reg8(REG_BASE_system_block + SYS_watchdog_configuration, 0x1); /* Use XTAL_IN as source */

	/* For ~100 usec delay */
	gbus_write_reg32(REG_BASE_system_block + SYS_watchdog_counter, TANGOX_BASE_FREQUENCY / 10000);
	gbus_write_reg8(REG_BASE_system_block + SYS_watchdog_configuration + 3, 0); /* Start counting */
#else
	/* Don't use watchdog to reboot */
	printk("System rebooting ...\n");
#endif
#else
	/* nowhere to  jump, everything is  in xload format,  lets ask
	 * xpu to reboot */
	base_addr = DMEM_BASE_audio_engine_0;

	pB = (struct xrpc_block_header *)base_addr;
	gbus_write_reg32((unsigned long)&pB->callerid, 0);
	gbus_write_reg32((unsigned long)&pB->headerandblocksize,
		    (sizeof(struct xrpc_block_header) + 63) & ~63);
	gbus_write_reg32((unsigned long)&pB->xrpcid, XRPC_ID_REBOOT);

	/* try to lock xrpc mutex for at most 1 sec */
	for (loop = 0; loop < 1000; loop++) {
		if (!gbus_read_reg32((RMuint32)XRPC_MUTEX))
			break;
		mdelay(1);
	}
	gbus_write_reg32(REG_BASE_cpu_block + LR_XPU_STAGE, (unsigned long)pB);

	/* cross our fingers now */
	gbus_write_reg32(REG_BASE_irq_handler_block + CPU_irq_softset,
		    SOFT_IRQ_XRPC);
#endif
	while (1); /* wait forever */
}

void tangox_machine_halt(void)
{
	while (1); /* wait forever */
}

void tangox_machine_power_off(void)
{
	while (1);
}

#ifndef CONFIG_TANGOX_USE_CPU_CLOCK
union tangox_cycle_cnt_union {
	u64 cycle64;
	u32 cycle32[2];
};
union tangox_cycle_cnt_union tangox_cycle_cnt;
static u32 old_xtal_cnt = 0;

EXPORT_SYMBOL(tangox_cycle_cnt);

struct clocksource clocksource_tangox = {
	.name		= "TANGOX",
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static cycle_t tangox_get_cycles(void)
{
	tangox_cycle_cnt.cycle32[0] = (u32)gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
	if (old_xtal_cnt > tangox_cycle_cnt.cycle32[0])
		tangox_cycle_cnt.cycle32[1]++;

	old_xtal_cnt = tangox_cycle_cnt.cycle32[0];
	return (cycle_t)tangox_cycle_cnt.cycle64;
}
#endif

static void __init tangox_time_init(void)
{
#ifndef CONFIG_TANGOX_USE_CPU_CLOCK
	u64 temp;
	u32 shift;
#endif

	mips_hpt_frequency = em8xxx_cpu_frequency / 2;

#ifndef CONFIG_TANGOX_USE_CPU_CLOCK
	/* Setup clock source from SYS_xtal_in_cnt */
	clocksource_tangox.rating = em8xxx_cpu_frequency / 1000000;
	tangox_cycle_cnt.cycle32[0] = old_xtal_cnt = (u32)gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
	tangox_cycle_cnt.cycle32[1] = 0;

	/* Find a shift value */
	for (shift = 32; shift > 0; shift--) {
		temp = (u64) NSEC_PER_SEC << shift;
		do_div(temp, TANGOX_BASE_FREQUENCY);
		if ((temp >> 32) == 0)
			break;
	}
	clocksource_tangox.shift = shift;
	clocksource_tangox.mult = (u32)temp;
	clocksource_tangox.read = tangox_get_cycles;

	clocksource_register(&clocksource_tangox);
#endif
}

#ifndef CONFIG_TANGOX_USE_CPU_CLOCK
/*
 * two variants for linux system timer , we can use internal cpu timer
 * or hw timer0
 */
/*
 * CPU_time0_load
 *  clock / HZ / (2 * prescale)
 * CPU_time0_ctrl
 *  PS(D2-3) : prescale. 0x00 = 1, 0x01 = 16, 0x10 = 256
 *    There is a bug, and the actual prescale is 0x01 = 32, 0x10 = 512
 *  M(D6) : periodic mode
 *  E(D7) : enable
 */
#define TICKS_PER_SEC           em8xxx_sys_frequency
//#define TIMER_PRESCALE          2
//#define TIMER_PRESCALEBITS      1
//#define TIMER_PRESCALE          32
//#define TIMER_PRESCALEBITS      5
//#define TIMER_PRESCALE          512
//#define TIMER_PRESCALEBITS      9
//#define TIMER_RELOAD            ((TICKS_PER_SEC / HZ) >> (TIMER_PRESCALEBITS))

#define TIMER_ENABLE            0x80    // D7
#define TIMER_PERIODIC          0x40    // D6
#define TIMER_PRESCALE_1        0x00    // D[2-3] = 00b
#define TIMER_PRESCALE_32       0x04    // D[2-3] = 01b
#define TIMER_PRESCALE_512      0x08    // D[2-3] = 10b

static const unsigned timer_prescale_bit[3] = { 1, 5, 9 };
static const unsigned timer_prescale[3] = { TIMER_PRESCALE_1, TIMER_PRESCALE_32, TIMER_PRESCALE_512 };

static void tangox_timer_ack(void)
{
	WR_CPU_REG32(CPU_time0_clr, 1);
}

void reset_cpu_timer0(void)
{
	int idx;
	unsigned int timer_reload;

	/* CPU_time0_load  register contains  just  16-bits value  So,
	   take care not to let the value to overflow */
	for (idx = 0; idx < 3; idx++) {
		timer_reload = (TICKS_PER_SEC / HZ) >> timer_prescale_bit[idx];
		if ((timer_reload & 0xffff0000) == 0)
			break;
	}
	if (idx >= 3)
		BUG();

	WR_CPU_REG32(CPU_time0_load, timer_reload);
	WR_CPU_REG32(CPU_time0_ctrl, TIMER_ENABLE | TIMER_PERIODIC | timer_prescale[idx]);
	WR_CPU_REG32(CPU_time0_clr, 1);
}

/*
 * Setup Timer0 as the source
 */
void __init plat_timer_setup(struct irqaction *irq)
{
	int idx;
	unsigned int timer_reload;

	/* CPU_time0_load  register contains  just  16-bits value  So,
	   take care not to let the value to overflow */
	for (idx = 0; idx < 3; idx++) {
		timer_reload = (TICKS_PER_SEC / HZ) >> timer_prescale_bit[idx];
		if ((timer_reload & 0xffff0000) == 0)
			break;
	}
	if (idx >= 3)
		BUG();

	WR_CPU_REG32(CPU_time0_load, timer_reload);
	WR_CPU_REG32(CPU_time0_ctrl, TIMER_ENABLE | TIMER_PERIODIC | timer_prescale[idx]);
	WR_CPU_REG32(CPU_time0_clr, 1);

	setup_irq(LOG2_CPU_TIMER0_INT + IRQ_CONTROLLER_IRQ_BASE, irq);

	/* set ack callback */
	mips_timer_ack = tangox_timer_ack;

	/* Clear heart beat counter */
	WR_CPU_REG32(LR_HB_CPU, 0);
}
#else
/*
 * Setup CPU timer (compare/count) as source
 */
void __init plat_timer_setup(struct irqaction *irq)
{
#ifdef CONFIG_TANGO3
	setup_irq(MIPS_CPU_IRQ_BASE + ((read_c0_intctl() >> 29) & 7), irq);
#else
	setup_irq(MIPS_CPU_IRQ_BASE + STATUSB_IP7 - STATUSB_IP0, irq);
#endif

	/* Clear heart beat counter */
	WR_CPU_REG32(LR_HB_CPU, 0);
}
#endif

/*
 * setup remap registers, we may need  to use ioremap() so we can't do
 * this in plat_setup, this function is set as arch_initcall().
 */
static int __init tangox_remap_setup(void)
{
#if defined(CONFIG_TANGO2) && defined(CONFIG_TANGOX_USE_TLB_REMAP_DRAM1)
	memcfg_t *m;
#endif

#if defined(CONFIG_TANGO2) 
	/*
	 * Program CPU_remap so we can see full 256MB space in KSEG0 /
	 * KSEG1
	 */
#ifdef CONFIG_TANGOX_USE_TLB_REMAP_DRAM1
	/*
	 * Use TLB mapping to map the DRAM1 (size specified by memcfg)
	 * into KSEG2
	 */
	m = (memcfg_t *)KSEG1ADDR(MEM_BASE_dram_controller_0_alias + FM_MEMCFG);

	if (m->dram1_size) {
		em86xx_tlb_dram1_map_size = ((m->dram1_size > 0x20000000) ? 
			0x20000000 : m->dram1_size); /* Max. 512MB */
		em86xx_tlb_dram1_map_base =
			(unsigned long)ioremap(MEM_BASE_dram_controller_1_alias,
					       m->dram1_size);
		printk("tangox: creating TLB mapping for 0x%08x at 0x%08lx, "
		       "size 0x%08lx.\n", MEM_BASE_dram_controller_1_alias,
		       em86xx_tlb_dram1_map_base, em86xx_tlb_dram1_map_size);
	} else {
		printk("tangox: dram1 size is 0, _not_ creating mapping\n");
	}
#else
	/*
	 * Use remap strategy (CPU_remap3/4 for 128MB resolution)
	 */
	printk("tangox: creating CPU mapping for 0x%08x at 0x%08x, "
	       "size 0x%08x.\n", MEM_BASE_dram_controller_1_alias,
	       CPU_remap3_address, 0x08000000);

	/*
	 * remap dram controller 1 at 0x08000000 -> 0x0fffffff (128MB)
	 * so Linux can see it in KSEG[01]
	 */
	gbus_write_reg32(REG_BASE_cpu_block + CPU_remap3,
		    MEM_BASE_dram_controller_1_alias);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_remap4,
		    MEM_BASE_dram_controller_1_alias + 0x04000000);
	iob();
#endif
#endif

	return 0;
}

arch_initcall(tangox_remap_setup);

extern int tangox_uart_enabled(int uart);
extern int tangox_uart_baudrate(int uart);
extern int tangox_uart_console_port(void);

#ifdef CONFIG_SERIAL_8250
struct tangox_uart_info {
	int irq;
	unsigned long base;
};

#ifdef CONFIG_TANGO3
static struct tangox_uart_info uinfo[3] = {
	{ LOG2_CPU_UART0_INT, CPU_UART0_base },
	{ LOG2_CPU_UART1_INT, CPU_UART1_base },
	{ LOG2_CPU_UART2_INT, CPU_UART2_base },
};
#else
static struct tangox_uart_info uinfo[2] = {
	{ LOG2_CPU_UART0_INT, CPU_UART0_base },
	{ LOG2_CPU_UART1_INT, CPU_UART1_base },
};
#endif
#endif

void __init plat_mem_setup(void)
{
#ifdef CONFIG_SERIAL_8250
	int i, idx;
	struct uart_port uart;
	int console_port = tangox_uart_console_port();
#endif

	board_time_init = tangox_time_init;
	//board_timer_setup = tangox_timer_setup;
	_machine_restart = tangox_machine_restart;
	_machine_halt = tangox_machine_halt;
	//_machine_power_off = tangox_machine_power_off;

#ifdef CONFIG_SERIAL_8250
	/* Handle console first */
	memset(&uart, 0, sizeof (uart));
	uart.line = 0;
	uart.uartclk = tangox_uart_baudrate(console_port) << 4;
	uart.irq = IRQ_CONTROLLER_IRQ_BASE + uinfo[console_port].irq;
	uart.flags = UPF_BOOT_AUTOCONF /* | UPF_SHARE_IRQ */;
	uart.membase = (unsigned char *)(REG_BASE_cpu_block + uinfo[console_port].base);
	uart.iotype = UPIO_MEM;
	uart.regshift = 2;

	if (early_serial_setup(&uart))
		printk("early_serial_setup failed\n");

	for (i = 0, idx = 1; (i < CONFIG_SERIAL_8250_NR_UARTS) && (i < (sizeof(uinfo) / sizeof(uinfo[0]))); i++) {
		if (console_port == i)
			continue;

		memset(&uart, 0, sizeof (uart));
		uart.line = idx++;
		uart.uartclk = tangox_uart_baudrate(i) << 4;
		uart.irq = IRQ_CONTROLLER_IRQ_BASE + uinfo[i].irq;
		uart.flags = UPF_BOOT_AUTOCONF /* | UPF_SHARE_IRQ */;
		uart.membase = (unsigned char *)(REG_BASE_cpu_block + uinfo[i].base);
		uart.iotype = UPIO_MEM;
		uart.regshift = 2;

		if (early_serial_setup(&uart))
			printk("early_serial_setup failed\n");
	}
#endif

	/*
	 * set I/O /mem regions limit
	 */
	ioport_resource.start = 0;
	ioport_resource.end = 0x80000000UL - 1;
	iomem_resource.start = 0;
	iomem_resource.end = 0x80000000UL - 1;

}
