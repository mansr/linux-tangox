/*
 * Copyright 2001 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * Copyright (C) 2007-2011 Sigma Designs, Inc.
 * arch/mips/tangox/setup.c
 *     The setup file for tango2/tango3/tango4.
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
#include <linux/serial_8250.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/init.h>
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

int coherentio = 0; 	/* init to 0 => no DMA cache coherency (may be set by user) */

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
	int i;
#ifdef CONFIG_TANGO2
	unsigned long tmp;
 	struct xrpc_block_header *pB;
	unsigned long base_addr;
	int loop;
#endif

        local_irq_disable();

#ifdef CONFIG_PCI
	tangox_pci_shutdown();
#endif

#if defined(CONFIG_TANGO2) 
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
#elif defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
		i < 8; 
#endif
		i++) {
		gbus_write_reg32(REG_BASE_cpu_block + CPU_remap + i * 4, em8xxx_remap_registers[i]);
	}
	iob();

	/* Now to handle CPU side */

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
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

union tangox_cycle_cnt_union {
	u64 cycle64;
	u32 cycle32[2];
};
static DEFINE_SPINLOCK(cycle_cnt_lock);
static union tangox_cycle_cnt_union tangox_cycle_cnt;
static u32 tangox_cycle_last = 0;

static cycle_t tangox_read_cycles(struct clocksource *cs)
{
	unsigned long flags;
	spin_lock_irqsave(&cycle_cnt_lock, flags);
	tangox_cycle_cnt.cycle32[0] = (u32)gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
	if (tangox_cycle_last > tangox_cycle_cnt.cycle32[0])	
		tangox_cycle_cnt.cycle32[1]++; /* low 32 overflowed, increase high 32 */

	tangox_cycle_last = tangox_cycle_cnt.cycle32[0];
	spin_unlock_irqrestore(&cycle_cnt_lock, flags);
	return (cycle_t)tangox_cycle_cnt.cycle64;
}

void tangox_set_cycles(unsigned long cnt_low, unsigned long cnt_high, unsigned long cnt_last)
{
	unsigned long flags;
	spin_lock_irqsave(&cycle_cnt_lock, flags);
	tangox_cycle_cnt.cycle32[0] = cnt_low;
	tangox_cycle_cnt.cycle32[1] = cnt_high;
	tangox_cycle_last = cnt_last;
	spin_unlock_irqrestore(&cycle_cnt_lock, flags);
}
EXPORT_SYMBOL(tangox_set_cycles);

void tangox_get_cycles(unsigned long *cnt_low, unsigned long *cnt_high, unsigned long *cnt_last)
{
	unsigned long flags;
	spin_lock_irqsave(&cycle_cnt_lock, flags);
	*cnt_low = tangox_cycle_cnt.cycle32[0];
	*cnt_high = tangox_cycle_cnt.cycle32[1];
	*cnt_last = tangox_cycle_last;
	spin_unlock_irqrestore(&cycle_cnt_lock, flags);
}
EXPORT_SYMBOL(tangox_get_cycles);

struct clocksource clocksource_tangox = {
	.name		= "TANGOX",
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.read		= tangox_read_cycles,
};

void __init plat_time_init(void)
{
	u64 temp;
	u32 shift;

	mips_hpt_frequency = em8xxx_cpu_frequency / 2;
	/* Clear heart beat counter */
	WR_CPU_REG32(LR_HB_CPU, 0);

	/* Setup clock source from SYS_xtal_in_cnt */
	clocksource_tangox.rating = em8xxx_cpu_frequency / 1000000;
	tangox_cycle_cnt.cycle32[0] = tangox_cycle_last = (u32)gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
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

	clocksource_register(&clocksource_tangox);
}

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
	m = (memcfg_t *)KSEG1ADDR(MEM_BASE_dram_controller_0 + FM_MEMCFG);

	if (m->dram1_size) {
		em86xx_tlb_dram1_map_size = ((m->dram1_size > 0x20000000) ? 
			0x20000000 : m->dram1_size); /* Max. 512MB */
		em86xx_tlb_dram1_map_base =
			(unsigned long)ioremap(MEM_BASE_dram_controller_1,
					       m->dram1_size);
		printk("tangox: creating TLB mapping for 0x%08x at 0x%08lx, "
		       "size 0x%08lx.\n", MEM_BASE_dram_controller_1,
		       em86xx_tlb_dram1_map_base, em86xx_tlb_dram1_map_size);
	} else {
		printk("tangox: dram1 size is 0, _not_ creating mapping\n");
	}
#else
	/*
	 * Use remap strategy (CPU_remap3/4 for 128MB resolution)
	 */
	printk("tangox: creating CPU mapping for 0x%08x at 0x%08x, "
	       "size 0x%08x.\n", MEM_BASE_dram_controller_1,
	       CPU_remap3_address, 0x08000000);

	/*
	 * remap dram controller 1 at 0x08000000 -> 0x0fffffff (128MB)
	 * so Linux can see it in KSEG[01]
	 */
	gbus_write_reg32(REG_BASE_cpu_block + CPU_remap3,
		    MEM_BASE_dram_controller_1);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_remap4,
		    MEM_BASE_dram_controller_1 + 0x04000000);
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

#if defined(CONFIG_TANGO3)
static struct tangox_uart_info __initdata uinfo[3] = {
	{ LOG2_CPU_UART0_INT, REG_BASE_cpu_block + CPU_UART0_base },
	{ LOG2_CPU_UART1_INT, REG_BASE_cpu_block + CPU_UART1_base },
	{ LOG2_CPU_UART2_INT, REG_BASE_cpu_block + CPU_UART2_base }, 
};
unsigned int tangox_uart_base[3] = {
	REG_BASE_cpu_block + CPU_UART0_base, REG_BASE_cpu_block + CPU_UART1_base, REG_BASE_cpu_block + CPU_UART2_base,
};
#elif defined(CONFIG_TANGO4) 
static struct tangox_uart_info __initdata uinfo[3] = {
	{ LOG2_SYS_UART0_INT, REG_BASE_system_block + 0x700 },
	{ LOG2_CPU_UART1_INT, REG_BASE_cpu_block + CPU_UART1_base },
	{ LOG2_CPU_UART2_INT, REG_BASE_cpu_block + CPU_UART2_base }, 
};
unsigned int tangox_uart_base[3] = {
	REG_BASE_cpu_block + CPU_UART0_base, REG_BASE_cpu_block + CPU_UART1_base, REG_BASE_cpu_block + CPU_UART2_base,
};
#else
static struct tangox_uart_info __initdata uinfo[2] = {
	{ LOG2_CPU_UART0_INT, REG_BASE_cpu_block + CPU_UART0_base },
	{ LOG2_CPU_UART1_INT, REG_BASE_cpu_block + CPU_UART1_base },
};
unsigned int tangox_uart_base[2] = {
	REG_BASE_cpu_block + CPU_UART0_base, REG_BASE_cpu_block + CPU_UART1_base,
};
#endif
#endif

void uart_init(int uart_idx, int baud, int fifo);

void __init plat_mem_setup(void)
{
#ifdef CONFIG_SERIAL_8250
	int i, idx;
	struct uart_port uart;
	int console_port = tangox_uart_console_port();
	int uinfo_size = sizeof(uinfo) / sizeof(uinfo[0]);
#if defined(CONFIG_TANGO3)
	unsigned long tangox_chip_id(void);
	unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
#endif
#endif

	_machine_restart = tangox_machine_restart;
	_machine_halt = tangox_machine_halt;
	//_machine_power_off = tangox_machine_power_off;
	pm_power_off = tangox_machine_power_off;

#ifdef CONFIG_TANGO3
	if ((chip_id == 0x8656) || (chip_id == 0x8672) || (chip_id == 0x8674))
		tangox_uart_base[0] = REG_BASE_system_block + 0x700;
#endif

#ifdef CONFIG_SERIAL_8250
	/* Handle console first */
	uart_init(console_port, tangox_uart_baudrate(console_port), 1);
	memset(&uart, 0, sizeof (uart));
	uart.line = 0;
#ifdef CONFIG_TANGOX_UART_USE_SYSCLK
	uart.uartclk = tangox_get_sysclock();
#else
#if defined(CONFIG_TANGO2)
	uart.uartclk = TANGOX_BASE_FREQUENCY;
#elif defined(CONFIG_TANGO3)
	uart.uartclk = TANGO3_UART_FREQUENCY;
#elif defined(CONFIG_TANGO4)
	uart.uartclk = TANGO4_UART_FREQUENCY;
#else
#error Unsupported platform.
#endif
#endif
	uart.irq = IRQ_CONTROLLER_IRQ_BASE + uinfo[console_port].irq;
	uart.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
#ifdef CONFIG_TANGO3
	if (((chip_id == 0x8656) || (chip_id == 0x8672) || (chip_id == 0x8674)) && (console_port == 0))
		uart.membase = (unsigned char *)(REG_BASE_system_block + 0x700);
	else
#endif
		uart.membase = (unsigned char *)uinfo[console_port].base;
	uart.iotype = UPIO_MEM;
	uart.regshift = 2;

	if (early_serial_setup(&uart))
		printk("early_serial_setup failed\n");

#ifdef CONFIG_TANGO3
	if ((chip_id != 0x8652) && ((chip_id & 0xfff0) != 0x8670) && ((chip_id & 0xfff0) != 0x8680))
		uinfo_size--;
#endif

	for (i = 0, idx = 1; (i < CONFIG_SERIAL_8250_NR_UARTS) && (i < uinfo_size); i++) {
		if (console_port == i)
			continue;

		uart_init(i, tangox_uart_baudrate(i), 1);
		memset(&uart, 0, sizeof (uart));
		uart.line = idx++;
#ifdef CONFIG_TANGOX_UART_USE_SYSCLK
		uart.uartclk = tangox_get_sysclock();
#else
#if defined(CONFIG_TANGO2)
		uart.uartclk = TANGOX_BASE_FREQUENCY;
#elif defined(CONFIG_TANGO3)
		uart.uartclk = TANGO3_UART_FREQUENCY;
#elif defined(CONFIG_TANGO4)
		uart.uartclk = TANGO4_UART_FREQUENCY;
#else
#error Unsupported platform.
#endif
#endif
		uart.irq = IRQ_CONTROLLER_IRQ_BASE + uinfo[i].irq;
		uart.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST /* | UPF_SHARE_IRQ */;
#ifdef CONFIG_TANGO3
		if (((chip_id == 0x8656) || (chip_id == 0x8672) || (chip_id == 0x8674)) && (i == 0))
			uart.membase = (unsigned char *)(REG_BASE_system_block + 0x700);
		else
#endif
			uart.membase = (unsigned char *)uinfo[i].base;
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

struct tangox_mutex_struct
{
	struct mutex mutex;
	unsigned int lock_cnt;
};

static struct tangox_mutex_struct tangox_mutex[NUM_TANGOX_MUTEX];

static int __init tangox_mutex_init(void)
{
	int i;
	for (i = 0; i < NUM_TANGOX_MUTEX; i++) {
		mutex_init(&tangox_mutex[i].mutex);
		tangox_mutex[i].lock_cnt = 0;
	}
	return 0;
}

int tangox_mutex_lock(unsigned int idx)
{
	if (in_interrupt() || in_atomic())
		return -1; /* not safe for context switching */
	else if (unlikely(idx >= NUM_TANGOX_MUTEX))
		return -1;

	mutex_lock(&tangox_mutex[idx].mutex);
	tangox_mutex[idx].lock_cnt++;
	return 0;
}

int tangox_mutex_unlock(unsigned int idx)
{
	if ((idx < NUM_TANGOX_MUTEX) && (tangox_mutex[idx].lock_cnt > 0)) {
		tangox_mutex[idx].lock_cnt--;
		mutex_unlock(&tangox_mutex[idx].mutex);
	}
	return 0;
}

__initcall(tangox_mutex_init);
EXPORT_SYMBOL(tangox_mutex_lock);
EXPORT_SYMBOL(tangox_mutex_unlock);

