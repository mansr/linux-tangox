
/*********************************************************************
 Copyright (C) 2001-2008
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/init.h>
#include <asm/bootinfo.h>
#include <asm/page.h>
#include <asm/cacheflush.h>
#include <linux/module.h>
#include <linux/mm.h>

#include "setup.h"

#ifdef CONFIG_TANGO3 
#include "xenv.h"
#include "xenvkeys.h"
#endif

/*
 * em8xxx_sys_frequency may be used later in the serial  code, DON'T mark
 * it as initdata
 */
unsigned long em8xxx_sys_frequency;
unsigned long em8xxx_cpu_frequency;
unsigned long em8xxx_kmem_start;
unsigned long em8xxx_kmem_size;
unsigned long em8xxx_sys_clkgen_pll;
unsigned long em8xxx_sys_premux;
unsigned long em8xxx_sys_mux;
unsigned long em8xxx_max_dx_size;
#ifdef CONFIG_TANGO3
unsigned long phy_remap;
unsigned long max_remap_size;
#endif 

unsigned long tangox_chip_id(void);
int is_tango2_chip(void);
int is_tango3_chip(void);
int is_tango3_es1(void);
int is_tango3_es2(void);

/*
 * we will restore remap registers before rebooting
 */
#ifdef CONFIG_TANGO2
unsigned long em8xxx_remap_registers[5];
#elif defined(CONFIG_TANGO3)
unsigned long em8xxx_remap_registers[8];
#endif 

/*
 * helper to access base registers
 */
#define RD_BASE_REG32(r)	\
		gbus_read_reg32(REG_BASE_system_block + (r))

/*
 * return system type (/proc/cpuinfo)
 */
const char *get_system_type(void)
{
	return "Sigma Designs TangoX";
}

#ifdef CONFIG_TANGOX_FIXED_FREQUENCIES
unsigned long tangox_get_pllclock(int pll)
{
	return(0);
}

unsigned long tangox_get_sysclock(void)
{
	return(CONFIG_TANGOX_SYS_FREQUENCY);
}

unsigned long tangox_get_cpuclock(void)
{
	return(CONFIG_TANGOX_CPU_FREQUENCY);
}

unsigned long tangox_get_dspclock(void)
{
	return(CONFIG_TANGOX_DSP_FREQUENCY);
}
#else
unsigned long tangox_get_pllclock(int pll)
{
	unsigned long sys_clkgen_pll, sysclk_mux;
	unsigned long n, m, freq, k, step;

	sysclk_mux = RD_BASE_REG32(SYS_sysclk_mux);
	if ((sysclk_mux & 0x1) == 0) {
		freq = TANGOX_BASE_FREQUENCY;
		goto done;
	}

	sys_clkgen_pll = RD_BASE_REG32(SYS_clkgen0_pll + (pll * 0x8));

	/* Not using XTAL_IN, cannot calculate */
	if ((sys_clkgen_pll & 0x07000000) != 0x01000000)
		goto freq_error;

#ifdef CONFIG_TANGO2
	m = (sys_clkgen_pll >> 16) & 0x1f;
	n = sys_clkgen_pll & 0x000003ff;
	k = (pll) ? 0 : ((sys_clkgen_pll >> 14) & 0x3); 
	step = 2;
#elif defined(CONFIG_TANGO3)
	if (pll != 0) { /* PLL1/PLL2 */
		m = (sys_clkgen_pll >> 16) & 0x1;
		n = sys_clkgen_pll & 0x0000007f;
		k = (sys_clkgen_pll >> 13) & 0x7;
		step = 1;
	} else {
		m = (sys_clkgen_pll >> 16) & 0x1f;
		n = sys_clkgen_pll & 0x000003ff;
		k = (sys_clkgen_pll >> 14) & 0x3; 
		step = 2;
	}
#else
#error Unsupported platform.
#endif
	freq = ((TANGOX_BASE_FREQUENCY / (m + step)) * (n + step)) / (1 << k);

done:
	return(freq);

freq_error:
	return(0);
}

static unsigned long tangox_get_clock(unsigned int clk_dom)
{
	unsigned long sysclk_mux, sysclk_premux;
	unsigned long div, mux, pll, pll_freq;
	static const unsigned char dividers[3][12] = {
		{ 2, 4, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4 },
		{ 2, 2, 2, 3, 3, 2, 3, 2, 4, 2, 4, 2 }, 
#ifdef CONFIG_TANGO2
		{ 2, 4, 3, 3, 3, 3, 2, 2, 4, 4, 2, 2 },
#else
		{ 2, 4, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4 },
#endif
	};

	sysclk_mux = RD_BASE_REG32(SYS_sysclk_mux);
	sysclk_premux = RD_BASE_REG32(SYS_sysclk_premux);
	pll = sysclk_premux & 0x3;

	if (((pll_freq = tangox_get_pllclock(pll)) == 0) || (clk_dom >= 3))
		goto freq_error;
	else if ((mux = ((sysclk_mux >> 8) & 0xf)) >= 12)
		goto freq_error; /* invalid mux value */

	div = (unsigned long)dividers[clk_dom][mux];

	return(pll_freq / div);

freq_error:
	return(0);
}

unsigned long tangox_get_sysclock(void)
{
	return(tangox_get_clock(0));
}

unsigned long tangox_get_cpuclock(void)
{
	return(tangox_get_clock(1));
}

unsigned long tangox_get_dspclock(void)
{
	return(tangox_get_clock(2));
}
#endif

extern int do_syslog(int type, char * buf, int len);
extern int __init xenv_config(void);
extern void __init tangox_device_info(void);
extern const char *tangox_xenv_cmdline(void);

#ifdef CONFIG_TANGO3
static inline unsigned long fixup_dram_address(unsigned long addr)
{
	if ((addr >= 0x10000000) && (addr < 0x20000000))
		addr = (addr - 0x10000000) + MEM_BASE_dram_controller_0; /* to DRAM0 */
	else if ((addr >= 0x20000000) && (addr < 0x30000000))
		addr = (addr - 0x20000000) + MEM_BASE_dram_controller_1; /* to DRAM1 */
	return(addr);
}

void update_lrrw_kend(unsigned long kend)
{
	xenv_set((void *)KSEG1ADDR(REG_BASE_cpu_block + LR_XENV2_RW), MAX_LR_XENV2_RW, XENV_LRRW_KERNEL_END, &kend, 0, sizeof(kend));
}
#endif

void __init prom_init(void)
{
	extern char _text;
	unsigned long offset, em8xxx_kmem_end;
	int clksel, xenv_res = 0, i;
	char *revStr = NULL;
#ifdef CONFIG_TANGO2
	memcfg_t *m;
#endif

#if 0
	/* For emulator, setup registers that typically got setup by bootloader */
	/* Temporary HACK */
	/* UART0/1 UART mode */
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_mode, 0xff00);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_mode, 0xff00);

	/* Set interrupt attributes, clear/disable all external interrupts */
	gbus_write_reg32(REG_BASE_cpu_block + CPU_irq_enableclr, 0xffffffff);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_config_rise_clr, 0xffffffff);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_config_fall_clr, 0xffffffff);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_config_rise_set, 0);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_config_fall_set, 0);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_rawstat, 0xffffffff);

	gbus_write_reg32(REG_BASE_cpu_block + CPU_irq_enableclr_hi, 0xffffffff);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_config_rise_clr_hi, 0xffffffff);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_config_fall_clr_hi, 0xffffffff);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_config_rise_set_hi, 0);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_config_fall_set_hi, 0);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_edge_rawstat_hi, 0xffffffff);
#endif
	/*
	 * save remap registers for reboot time
	 */
	for (i = 0;
#ifdef CONFIG_TANGO2
		i < 5;
#elif defined(CONFIG_TANGO3)
		i < 8;
#endif
		i++) {
		em8xxx_remap_registers[i] = gbus_read_reg32(REG_BASE_cpu_block + CPU_remap + (i * 4));
	}

	/* 
	 * Set remap so that 0x1fc00000 and 0x0 back to they should be...
	 */
	gbus_write_reg32(REG_BASE_cpu_block + CPU_remap, 0x1fc00000);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_remap1, 0x0);
	iob();
	
#ifdef CONFIG_TANGO3
#define REMAP_IDX      (((CPU_REMAP_SPACE-CPU_remap2_address)/0x04000000)+2)
#define MAX_KERNEL_MEMSIZE	(0x18000000-(((REMAP_IDX)-2)*0x04000000))
	phy_remap = fixup_dram_address(em8xxx_remap_registers[REMAP_IDX]);
	max_remap_size = 0x04000000; /* minimum 64MB */
	if (phy_remap != em8xxx_remap_registers[REMAP_IDX]) { /* fix up needed */
		gbus_write_reg32(REG_BASE_cpu_block + CPU_remap + REMAP_IDX * 4, phy_remap);
		iob();
	}
	for (i = REMAP_IDX + 1; i < 8; i++) {
		unsigned long newaddr = fixup_dram_address(em8xxx_remap_registers[i]);
		if (newaddr == (phy_remap + (0x04000000 * (i - REMAP_IDX)))) {
			max_remap_size += 0x04000000;
			if (newaddr != em8xxx_remap_registers[i]) { /* fix up needed */
				gbus_write_reg32(REG_BASE_cpu_block + CPU_remap + i * 4, newaddr);
				iob();
			}
		}
	}
	printk("Physical map 0x%08lx to 0x%08lx, max remap/kernel size: 0x%08lx/0x%08lx.\n",
		phy_remap, (unsigned long)CPU_REMAP_SPACE, max_remap_size, (unsigned long)MAX_KERNEL_MEMSIZE);
#endif
#ifdef CONFIG_TANGO2
#define MAX_KERNEL_MEMSIZE	(0x10000000)
#endif

#if defined(CONFIG_TANGO2_SMP863X)
	printk("Configured for SMP863%c (revision %s), ",
#if defined(CONFIG_TANGO2_ES1)
			'0', "ES1"
#elif defined(CONFIG_TANGO2_ES2)
			'0', "ES2"
#elif defined(CONFIG_TANGO2_ES3)
			'0', "ES3"
#elif defined(CONFIG_TANGO2_ES4)
			'4', "ES4"
#elif defined(CONFIG_TANGO2_ES5)
			'4', "ES5"
#elif defined(CONFIG_TANGO2_ES6)
			'x', "ES6+/RevA+"
#endif
	      );
#elif defined(CONFIG_TANGO3_SMP86XX)
	printk("Configured for SMP%s, ",
#if defined(CONFIG_TANGO3_865X)
			"865x"
#elif defined(CONFIG_TANGO3_864X)
			"864x"
#endif
	      );
#else
#error Unsupported platform.
#endif
	printk("detected SMP%lx (revision ", (tangox_chip_id()>>16)&0xffff);
	if (is_tango2_chip()) {
		unsigned long revision = tangox_chip_id() & 0xff;
		switch(revision) {
			case 0x81: /* ES1-3 */
				revStr = "ES1-3";
				break;
			case 0x82: /* ES4-5 */
				revStr = "ES4-5";
				break;
			case 0x83: /* ES6/RevA */
				revStr = "ES6/RevA";
				break;
			case 0x84: /* ES7/RevB */
				revStr = "ES7/RevB";
				break;
			case 0x85: /* ES8 */
				revStr = "ES8";
				break;
			case 0x86: /* ES9/RevC */
				revStr = "ES9/RevC";
				break;
			default: /* Unknown */
				revStr = "unknown";
				break;
		}
	} else if (is_tango3_chip()) {
		unsigned long revision = tangox_chip_id() & 0xff;
		switch(revision) {
			case 0x1: /* ES1 */
				revStr = "ES1";
				break;
			case 0x2: /* ES2 */
				revStr = "ES2";
				break;
			case 0x3: /* ES3 */
				revStr = "ES3";
				break;
			case 0x4: /* ES4 */
				revStr = "ES4";
				break;
			default: /* Unknown */
				revStr = "unknown";
				break;
		}
	} else
		revStr = "unknown";
	
	printk("%s).\n", revStr);
#ifdef CONFIG_TANGOX_FIXED_FREQUENCIES
	printk("Fixed CPU/System/DSP Frequencies: %ld.%02ld/%ld.%02ld/%ld.%02ldMHz\n",
		tangox_get_cpuclock() / 1000000, (tangox_get_cpuclock() / 10000) % 100,
		tangox_get_sysclock() / 1000000, (tangox_get_sysclock() / 10000) % 100,
		tangox_get_dspclock() / 1000000, (tangox_get_dspclock() / 10000) % 100);
#else
	printk("Detected CPU/System/DSP Frequencies: %ld.%02ld/%ld.%02ld/%ld.%02ldMHz\n",
		tangox_get_cpuclock() / 1000000, (tangox_get_cpuclock() / 10000) % 100,
		tangox_get_sysclock() / 1000000, (tangox_get_sysclock() / 10000) % 100,
		tangox_get_dspclock() / 1000000, (tangox_get_dspclock() / 10000) % 100);
#endif

	/*
	 * read xenv  configuration, we  need it quickly  to configure
	 * console accordingly.
	 *
	 * NOTE: We  may stay STUCK in  this if safe  mode is required
	 * and XENV is not valid !
	 */
	xenv_res = xenv_config();

	/*
	 * calculate cpu & sys frequency (may be needed for uart init)
	 */
	em8xxx_cpu_frequency = tangox_get_cpuclock();
	em8xxx_sys_frequency = tangox_get_sysclock();

	em8xxx_sys_clkgen_pll = RD_BASE_REG32(SYS_clkgen_pll);
	em8xxx_sys_premux = RD_BASE_REG32(SYS_sysclk_premux) & 0x3;
	em8xxx_sys_mux = RD_BASE_REG32(SYS_sysclk_mux) & 0xf1;

	/*
	 * program the right clock divider in both uart
	 */
#ifdef CONFIG_TANGOX_UART_USE_SYSCLK
	clksel = 0;
#else
	clksel = 1;
#endif
	gbus_write_reg32(REG_BASE_cpu_block + CPU_UART0_base + CPU_UART_CLKSEL, clksel);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_UART1_base + CPU_UART_CLKSEL, clksel);
#ifdef CONFIG_TANGO3
	gbus_write_reg32(REG_BASE_cpu_block + CPU_UART2_base + CPU_UART_CLKSEL, clksel);
#endif

	/*
	 * show KERN_DEBUG message on console
	 */
	do_syslog(8, NULL, 8);

#ifdef CONFIG_TANGOX_PROM_CONSOLE
	/* initialize uart and register early console */
	prom_console_register();
#endif

	/* warn user if we use failsafe values for xenv */
	if (xenv_res)
		printk("Invalid XENV content, using failsafe values\n");
	tangox_device_info();

	/*
	 * compute kernel memory start address/size
	 * _text section gives kernel address start
	 */
	em8xxx_kmem_start = ((unsigned long)(&_text)) & PAGE_MASK;

#if ((CONFIG_TANGOX_SYSTEMRAM_ACTUALSIZE<<20) > MAX_KERNEL_MEMSIZE)
	em8xxx_kmem_size = ((MAX_KERNEL_MEMSIZE + em8xxx_kmem_start) & 0xfff00000) - em8xxx_kmem_start;
#else
	em8xxx_kmem_size = (((CONFIG_TANGOX_SYSTEMRAM_ACTUALSIZE << 20) + em8xxx_kmem_start) & 0xfff00000) - em8xxx_kmem_start;
#endif

#ifdef CONFIG_TANGO3
	if (em8xxx_kmem_size > max_remap_size)
		em8xxx_kmem_size = max_remap_size;

#ifdef CONFIG_TANGOX_XENV_READ
	{
		unsigned long max_d0_size = 0, max_d1_size = 0, tmp;
		unsigned int size = sizeof(unsigned long);
		if ((xenv_get((void *)KSEG1ADDR(REG_BASE_cpu_block + LR_XENV2_RW), MAX_LR_XENV2_RW, XENV_LRRW_RUAMM0_GA, &tmp, &size) == 0) 
				&& (size == sizeof(unsigned long))) 
			max_d0_size = tmp - MEM_BASE_dram_controller_0;
		if ((xenv_get((void *)KSEG1ADDR(REG_BASE_cpu_block + LR_XENV2_RW), MAX_LR_XENV2_RW, XENV_LRRW_RUAMM1_GA, &tmp, &size) == 0) 
				&& (size == sizeof(unsigned long))) 
			max_d1_size = tmp - MEM_BASE_dram_controller_1;
		em8xxx_max_dx_size = (phy_remap >= MEM_BASE_dram_controller_1) ? max_d1_size : max_d0_size;
	}
	if ((em8xxx_max_dx_size != 0) && (em8xxx_kmem_size > em8xxx_max_dx_size)) { /* don't push into RUAMM area */
		em8xxx_kmem_size = em8xxx_max_dx_size;
		printk("Maximum kernel memory size is 0x%08lx with RUAMM restriction.\n", em8xxx_kmem_size);
	}
#endif
	em8xxx_kmem_end = KSEG1ADDR(em8xxx_kmem_start + em8xxx_kmem_size) - KSEG1ADDR(CPU_REMAP_SPACE);
	update_lrrw_kend(em8xxx_kmem_end);
#else
	/*
	 * check/fill the memcfg
	 */
	em8xxx_kmem_end = KSEG1ADDR(em8xxx_kmem_start + em8xxx_kmem_size) - KSEG1ADDR(MEM_BASE_dram_controller_0);
	m = (memcfg_t *)KSEG1ADDR(MEM_BASE_dram_controller_0_alias + FM_MEMCFG);
	if (is_valid_memcfg(m) == 0) {
		printk("Invalid MEMCFG, creating new one at 0x%08x.\n", MEM_BASE_dram_controller_0_alias + FM_MEMCFG);
		memset(m, 0, sizeof (memcfg_t));
		m->signature = MEMCFG_SIGNATURE;
		m->dram0_size = TANGOX_SYSTEMRAM_ACTUALSIZE;
		m->kernel_end = em8xxx_kmem_end;
		gen_memcfg_checksum(m);
	} else {
		printk("Valid MEMCFG found at 0x%08x.\n", MEM_BASE_dram_controller_0_alias + FM_MEMCFG);
		m->kernel_end = em8xxx_kmem_end;
		gen_memcfg_checksum(m);
	}
#endif

	/*
	 * tell kernel about available memory size/offset
	 */
#ifdef CONFIG_TANGO3
	offset = KSEG1ADDR(em8xxx_kmem_start) - KSEG1ADDR(CPU_REMAP_SPACE);
	add_memory_region(CPU_REMAP_SPACE + offset, em8xxx_kmem_size, BOOT_MEM_RAM);
#else
	offset = KSEG1ADDR(em8xxx_kmem_start) - KSEG1ADDR(MEM_BASE_dram_controller_0_alias);
	add_memory_region(MEM_BASE_dram_controller_0_alias + offset, em8xxx_kmem_size, BOOT_MEM_RAM);
#endif

#ifndef CONFIG_TANGOX_IGNORE_CMDLINE
	/*
	 * set up correct linux command line according to XENV, memcfg
	 * and YAMON args, if not told to ignore them
	 */

#ifdef CONFIG_TANGOX_XENV_READ
	/* If specified by xenv, override the command line */
	if (tangox_xenv_cmdline())
		strcpy(arcs_cmdline, tangox_xenv_cmdline());
#ifdef CONFIG_CMDLINE
	else
		strcpy(arcs_cmdline, CONFIG_CMDLINE);
#endif
#else
	strcpy(arcs_cmdline, CONFIG_CMDLINE);
#endif

	/* If specified by memcfg, override the command line */
//	if (m->linux_cmd != 0 && strlen((char *)KSEG1ADDR(m->linux_cmd)) > 0)
//disabled. e.m. 2006feb3rd		strcpy(arcs_cmdline, (char *)KSEG1ADDR(m->linux_cmd));

	/* take regular args given by bootloader */
	if ((fw_arg0 > 1) && (fw_arg0 < 65)) { /* Up to 64 arguments */
		int argc, i, pos;
		char **argv;

		argc = fw_arg0;
		arcs_cmdline[0] = '\0';
		argv = (char **) fw_arg1;
		pos = 0;
		for (i = 1; i < argc; i++) {
			int len;

			len = strlen(argv[i]);
			if (pos + 1 + len + 1 > sizeof (arcs_cmdline))
				break;
			if (pos)
				arcs_cmdline[pos++] = ' ';
			strcpy(arcs_cmdline + pos, argv[i]);
			pos += len;
		}
	}
#else
#ifdef CONFIG_CMDLINE
	strcpy(arcs_cmdline, CONFIG_CMDLINE);
#endif
#endif /* !CONFIG_TANGOX_IGNORE_CMDLINE */

	mips_machgroup = MACH_GROUP_SIGMADESIGNS;
	mips_machtype = MACH_TANGOX;
	return;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) 
unsigned long __init prom_free_prom_memory(void)
{
	return 0;
}
#else
void __init prom_free_prom_memory(void)
{
	return;
}
#endif
EXPORT_SYMBOL(tangox_get_sysclock);
EXPORT_SYMBOL(tangox_get_cpuclock);
EXPORT_SYMBOL(tangox_get_dspclock);
EXPORT_SYMBOL(tangox_get_pllclock);

unsigned long tangox_chip_id(void)
{
	return (((gbus_read_reg32(REG_BASE_host_interface + PCI_REG0) & 0xffff) << 16) |
                        (gbus_read_reg32(REG_BASE_host_interface + PCI_REG1) & 0xff));
}

int is_tango2_chip(void)
{
	unsigned long chip = (tangox_chip_id()>>16) & 0xfff0;
	return (chip == 0x8630) ? 1 : 0;
}

static inline int is_tango2_revision(unsigned char revid)
{
	unsigned char rev = tangox_chip_id() & 0xff;
	return (is_tango2_chip() && rev == revid) ? 1 : 0;
}

int is_tango2_es123(void)
{
	return(is_tango2_revision(0x81));
}

int is_tango2_es45(void)
{
	return(is_tango2_revision(0x82));
}

int is_tango2_es6(void)
{
	return(is_tango2_revision(0x83));
}

int is_tango2_es7(void)
{
	return(is_tango2_revision(0x84));
}

int is_tango2_es89(void)
{
	return(is_tango2_revision(0x85) || is_tango2_revision(0x86));
}

static inline int is_tango3_revision(unsigned char revid)
{
	unsigned char rev = tangox_chip_id() & 0xff;
	unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	if (!is_tango3_chip())
		return(0);
	else {
		switch(chip_id) {
			case 0x8652:
				rev += 2;
			case 0x8654:
			case 0x8644:
				return((rev == revid) ? 1 : 0);
			default:
				return(0);
		}
	}
}

int is_tango3_chip(void)
{
	unsigned long chip = (tangox_chip_id()>>16) & 0xfff0;
	return ((chip == 0x8640) || (chip == 0x8650)) ? 1 : 0;
}

int is_tango3_es1(void)
{
	return(is_tango3_revision(0x1));
}

int is_tango3_es2(void)
{
	return(is_tango3_revision(0x2));
}

int is_tango3_es3(void)
{
	return(is_tango3_revision(0x3));
}

int is_tango3_es4(void)
{
	return(is_tango3_revision(0x4));
}

int is_tango3_es5(void)
{
	return(is_tango3_revision(0x5));
}

void tangox_flush_cache_all(void)
{
	flush_cache_all();
}

EXPORT_SYMBOL(tangox_flush_cache_all);
EXPORT_SYMBOL(tangox_chip_id);
EXPORT_SYMBOL(is_tango2_chip);
EXPORT_SYMBOL(is_tango3_chip);
EXPORT_SYMBOL(is_tango2_es123);
EXPORT_SYMBOL(is_tango2_es45);
EXPORT_SYMBOL(is_tango2_es6);
EXPORT_SYMBOL(is_tango2_es7);
EXPORT_SYMBOL(is_tango2_es89);
EXPORT_SYMBOL(is_tango3_es1);
EXPORT_SYMBOL(is_tango3_es2);
EXPORT_SYMBOL(is_tango3_es3);
EXPORT_SYMBOL(is_tango3_es4);
EXPORT_SYMBOL(is_tango3_es5);
#ifdef CONFIG_TANGO3
EXPORT_SYMBOL(phy_remap);
EXPORT_SYMBOL(max_remap_size);
#endif

int tangox_get_order(unsigned long size)
{
	return(get_order(size));
}
EXPORT_SYMBOL(tangox_get_order);

void tangox_do_timer(unsigned long ticks)
{
	extern void do_timer(unsigned long ticks);

	write_seqlock(&xtime_lock);
	do_timer(ticks);
	write_sequnlock(&xtime_lock);
}
  
EXPORT_SYMBOL(tangox_do_timer);

#ifdef CONFIG_SD_DIRECT_DMA

/* Given an address and length, determine if this area is physically contiguous or not, and
   return the physical address of starting point, caller needs to ensure the page_table is
   locked so no change is allowed. */
int is_contiguous_memory(void __user *userbuf, unsigned int len, unsigned long *physaddr)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	unsigned long start = (unsigned long)userbuf;
	unsigned long paddr, ppaddr;
	unsigned long start_pg_addr, start_pg_offset, end_pg_addr, pg_addr;
	struct mm_struct *mm = current->mm;
	int ret = 0;

//printk("%s:%d: start=0x%08lx, len=0x%x\n", __FILE__, __LINE__, start, len);

	*physaddr = 0;
	start_pg_addr = start & PAGE_MASK; /* address of start page */
	start_pg_offset = start & (PAGE_SIZE - 1); /* offset within start page */
	end_pg_addr = ((start + len) & PAGE_MASK) - (((start + len) & (PAGE_SIZE - 1)) ? 0 : PAGE_SIZE); /* address of last page */

	for (ppaddr = 0, pg_addr = start_pg_addr; pg_addr <= end_pg_addr; pg_addr += PAGE_SIZE) {
		if (pg_addr > TASK_SIZE)
			pgd = pgd_offset_k(pg_addr);
		else
			pgd = pgd_offset_gate(mm, pg_addr);
		BUG_ON(pgd_none(*pgd));
		pud = pud_offset(pgd, pg_addr);
		BUG_ON(pud_none(*pud));
		pmd = pmd_offset(pud, pg_addr);
		if (pmd_none(*pmd)) 
			goto error;
		pte = pte_offset_map(pmd, pg_addr);
		if (pte_none(*pte)) {
			pte_unmap(pte);
			goto error;
		}
		paddr = pte_val(*pte) & PAGE_MASK;
//printk("TRANSLATED 0x%08lx, pte=0x%p, paddr=0x%lx\n", pg, pte, paddr);
		pte_unmap(pte);

		if (ppaddr == 0) { /* first page */
			ppaddr = paddr;
			*physaddr = (ppaddr | start_pg_offset);
		} else if ((ppaddr + PAGE_SIZE) != paddr) /* not contiguous */
			goto not_contiguous;
		else
			ppaddr = paddr;
	}
	ret = 1;

not_contiguous:
error:
//printk("%s:%d: return %d\n", __FILE__, __LINE__, ret);
	return ret;
}

EXPORT_SYMBOL(is_contiguous_memory);

#endif /* CONFIG_SD_DIRECT_DMA */

