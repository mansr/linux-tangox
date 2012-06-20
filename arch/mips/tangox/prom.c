
/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <asm/bootinfo.h>
#include <asm/page.h>
#include <asm/cacheflush.h>

#include "setup.h"

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
#include "xenv.h"
#include "xenvkeys.h"
#endif

#if defined(CONFIG_TANGO4)
//int gic_present = 0;
#endif

extern struct plat_smp_ops cmp_smp_ops;
int hw_coherentio = 0;

/*
 * em8xxx_sys_frequency may be used later in the serial  code, DON'T mark
 * it as initdata
 */
unsigned long em8xxx_sys_frequency;
unsigned long em8xxx_cpu_frequency;
unsigned long orig_cpu_freq;
unsigned long em8xxx_kmem_start;
unsigned long em8xxx_kmem_size;
unsigned long em8xxx_himem_start;
unsigned long em8xxx_himem_size;
unsigned long em8xxx_sys_clkgen_pll;
unsigned long em8xxx_sys_premux;
unsigned long em8xxx_sys_mux;
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
unsigned long max_remap_size = MAX_KERNEL_MEMSIZE;
#endif 

unsigned long tangox_chip_id(void);
int is_tango2_chip(void);
int is_tango3_chip(void);
int is_tango4_chip(void);
int is_tango3_es1(void);
int is_tango3_es2(void);
void tangox_get_himem_info(unsigned long *start, unsigned long *size);

/*
 * we will restore remap registers before rebooting
 */
#ifdef CONFIG_TANGO2
unsigned long em8xxx_remap_registers[5];
#elif defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
unsigned long em8xxx_remap_registers[9];
unsigned long tangox_zxenv[MAX_XENV_SIZE/sizeof(unsigned long)] = { 0 };
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
#if defined(CONFIG_TANGO4)
	return "Sigma Designs Tango4";
#elif defined(CONFIG_TANGO3)
	return "Sigma Designs Tango3";
#elif defined(CONFIG_TANGO2)
	return "Sigma Designs Tango2";
#else
	return "";
#endif
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

unsigned long tangox_get_cdclock(unsigned int cd)
{
	return(0);
}
#else
unsigned long tangox_get_pllclock(int pll)
{
	unsigned long sys_clkgen_pll, sysclk_mux;
	unsigned long n, m, freq, k, step;

	sysclk_mux = RD_BASE_REG32(SYS_sysclk_mux) & 0xf01;
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
#elif defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	if (pll != 0) { /* PLL1/PLL2 */
 		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id == 0x8646) || ((chip_id & 0xfff0) == 0x8670) || ((chip_id & 0xfff0) == 0x8680) || ((chip_id & 0xff00) == 0x8900))
			m = 0;
		else
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
	printk("%s:%d don't know how to calculate the frequency ..\n", __FILE__, __LINE__);
	BUG();
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

	sysclk_mux = RD_BASE_REG32(SYS_sysclk_mux) & 0xf01;
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

unsigned long tangox_get_cdclock(unsigned int cd)
{
	unsigned long sysclk_premux = (RD_BASE_REG32(SYS_sysclk_premux) & 0x700) >> 8;
	unsigned long cddiv = RD_BASE_REG32(SYS_cleandiv0_div + (cd * 8));
	u64 pllfreq, temp;

	pllfreq = (u64)tangox_get_pllclock(sysclk_premux >> 1);

	if (sysclk_premux & 1)  /* from PLLx_1 */
		do_div(pllfreq, (unsigned int)(RD_BASE_REG32(SYS_clkgen0_div + ((sysclk_premux >> 1) * 8)) & 0xf));

	/* use 64 bit for better precision, from formula
	   cd_out = pllfreq / (2 + cddiv * (2 ^ -27)) 
	   => cd_out = pllfreq / (2 + (cddiv >> 27)) 
	   => cd_out = pllfreq << 27 / (2 + (cddiv >> 27)) <<27
	   => cd_out = pllfreq << 27 / (2 << 27 + cddiv) 
	   => cd_out = pllfreq << 27 / (1 << 28 + cddiv) */
	temp = pllfreq << 27;
	do_div(temp, (unsigned int)((1 << 28) + cddiv));

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	if (RD_BASE_REG32(SYS_cleandiv0_div + 4 + (cd * 8)) & 1)
		return 0;
	else
#endif
		return((unsigned long)(temp & 0xffffffff));
}
#endif

extern int do_syslog(int type, char * buf, int len);
extern int __init xenv_config(void);
extern void __init tangox_device_info(void);
extern const char *tangox_xenv_cmdline(void);

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
#define REMAP_SIZE	0x04000000UL
static inline void update_remap(unsigned int remap, unsigned long mapaddr)
{
	if (gbus_read_reg32(REG_BASE_cpu_block + CPU_remap + (remap * 4)) != mapaddr) {
		gbus_write_reg32(REG_BASE_cpu_block + CPU_remap + (remap * 4), mapaddr);
		iob();
	}
	em8xxx_remap_registers[remap] = mapaddr & 0xfc000000UL;
}

#ifdef CONFIG_TANGOX_XENV_READ
unsigned long __init dram_remap_setup(unsigned long dsize)
{
	unsigned long max_d0_size = 0, max_d1_size = 0, tmp, tsize = 0, fd0 = 0;
	unsigned long dx_sizes[2], dx_addrs[2];
	unsigned int size = sizeof(unsigned long), i, j;

	/* check the boundaries for DRAM0 and DRAM1 */
	if ((xenv_get((void *)KSEG1ADDR(REG_BASE_cpu_block + LR_XENV2_RW), MAX_LR_XENV2_RW, XENV_LRRW_RUAMM0_GA, &tmp, &size) == 0) 
			&& (size == sizeof(unsigned long))) 
		fd0 = max_d0_size = tmp - MEM_BASE_dram_controller_0;
	if ((xenv_get((void *)KSEG1ADDR(REG_BASE_cpu_block + LR_XENV2_RW), MAX_LR_XENV2_RW, XENV_LRRW_RUAMM1_GA, &tmp, &size) == 0) 
			&& (size == sizeof(unsigned long))) 
		max_d1_size = tmp - MEM_BASE_dram_controller_1;

	/* Max. dynamic remap can do only 384MB */
	max_d0_size = (max_d0_size > 0x18000000UL) ? 0x18000000UL : max_d0_size;
	max_d1_size = (max_d1_size > 0x18000000UL) ? 0x18000000UL : max_d1_size;

	printk("Desired kernel memory size: 0x%08lx\n", dsize);
	printk("Max. DRAM0/1 size allowed: 0x%08lx/0x%08lx\n", max_d0_size, max_d1_size);

	if (gbus_read_reg32(REG_BASE_cpu_block + CPU_remap + (REMAP_IDX * 4)) >= MEM_BASE_dram_controller_1) {
		/* Use DRAM1 first */
		dx_sizes[0] = max_d1_size;
		dx_sizes[1] = max_d0_size;
		dx_addrs[0] = MEM_BASE_dram_controller_1;
		dx_addrs[1] = MEM_BASE_dram_controller_0;
	} else {
		/* Use DRAM0 first */
		dx_sizes[0] = max_d0_size;
		dx_sizes[1] = max_d1_size;
		dx_addrs[0] = MEM_BASE_dram_controller_0;
		dx_addrs[1] = MEM_BASE_dram_controller_1;
	}

	for (i = REMAP_IDX, j = 0; (dx_sizes[0] >= REMAP_SIZE) && (dsize >= REMAP_SIZE) && (i < 8); i++, j++) {
		if (i != REMAP_IDX)
			update_remap(i, dx_addrs[0] + (j * REMAP_SIZE));
		dx_sizes[0] -= REMAP_SIZE;
		dsize -= REMAP_SIZE;
		tsize += REMAP_SIZE;
		printk(" Mapped 0x%08lx(size 0x%08lx) via remap%d\n", 
			dx_addrs[0] + (j * REMAP_SIZE), REMAP_SIZE, i);
	}
	if (i < 8) {
		if (dsize > 0) {
			if (dsize < REMAP_SIZE) {
				if (dx_sizes[0] >= dsize) {
					update_remap(i, dx_addrs[0] + (j * REMAP_SIZE));
					tsize += dsize;
					printk(" Mapped 0x%08lx(size 0x%08lx) via remap%d\n", 
						dx_addrs[0] + (j * REMAP_SIZE), dsize, i);
					goto done;
				} else if (dx_sizes[0] > dx_sizes[1]) {
					update_remap(i, dx_addrs[0] + (j * REMAP_SIZE));
					tsize += dx_sizes[0];
					printk(" Mapped 0x%08lx(size 0x%08lx) via remap%d\n", 
						dx_addrs[0] + (j * REMAP_SIZE), dx_sizes[0], i);
					goto done;
				}
			} else {
				if (dx_sizes[0] > dx_sizes[1]) {
					update_remap(i, dx_addrs[0] + (j * REMAP_SIZE));
					tsize += dx_sizes[0];
					printk(" Mapped 0x%08lx(size 0x%08lx) via remap%d\n", 
						dx_addrs[0] + (j * REMAP_SIZE), dx_sizes[0], i);
					goto done;
				}
			}
		}
	} else {
#if defined(CONFIG_TANGO3)
		/* Check to see if the last, fixed remap works? */
		if (dsize > 0) {
			unsigned long msz = (fd0 > 0x0c000000UL) ? (fd0 - 0x0c000000UL) : 0UL;
			msz = (msz > dsize) ? dsize : msz;
			if (msz) {
				tsize += msz;
				printk(" Mapped 0x%08lx(size 0x%08lx) via remap%d\n", 
					MEM_BASE_dram_controller_0 + 0x0c000000UL, msz, i);
			}
		}
#endif
		goto done;
	}

	for (j = 0; (dx_sizes[1] >= REMAP_SIZE) && (dsize >= REMAP_SIZE) && (i < 8); i++, j++) {
		update_remap(i, dx_addrs[1] + (j * REMAP_SIZE));
		dx_sizes[1] -= REMAP_SIZE;
		dsize -= REMAP_SIZE;
		tsize += REMAP_SIZE;
		printk(" Mapped 0x%08lx(size 0x%08lx) via remap%d\n", 
			dx_addrs[1] + (j * REMAP_SIZE), REMAP_SIZE, i);
	}
	if (i < 8) {
		if (dsize > 0) {
			unsigned long msz = (dx_sizes[1] > dsize) ? dsize : dx_sizes[1];
			if (msz) {
				update_remap(i, dx_addrs[1] + (j * REMAP_SIZE));
				tsize += msz;
				printk(" Mapped 0x%08lx(size 0x%08lx) via remap%d\n", 
					dx_addrs[1] + (j * REMAP_SIZE), msz, i);
			}
			goto done;
		}
	} else {
#if defined(CONFIG_TANGO3)
		/* Check to see if the last, fixed remap works? */
		if (dsize > 0) {
			unsigned long msz = (fd0 > 0x0c000000UL) ? (fd0 - 0x0c000000UL) : 0UL;
			msz = (msz > dsize) ? dsize : msz;
			if (msz) {
				tsize += msz;
				printk(" Mapped 0x%08lx(size 0x%08lx) via remap%d\n", 
					MEM_BASE_dram_controller_0 + 0x0c000000UL, msz, i);
			}
		}
#endif
		goto done;
	}

done:
	printk("Final kernel memory size: 0x%08lx\n", tsize);
	return(tsize);
}
#endif

void __init update_lrrw_kend(unsigned long kend)
{
#ifdef CONFIG_TANGOX_XENV_READ
	/* it's not used */
//	xenv_set((void *)KSEG1ADDR(REG_BASE_cpu_block + LR_XENV2_RW), MAX_LR_XENV2_RW, XENV_LRRW_KERNEL_END, &kend, 0, sizeof(kend));
#endif
}
#endif /* TANGO3 || TANGO4 */

void __init tangox_mem_setup(unsigned long size)
{
	unsigned long em8xxx_kmem_end;
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
#ifdef CONFIG_TANGOX_XENV_READ
	em8xxx_kmem_size = dram_remap_setup(size);
#else
	em8xxx_kmem_size = size;
#endif
	em8xxx_kmem_end = KSEG1ADDR(em8xxx_kmem_start + em8xxx_kmem_size) - KSEG1ADDR(CPU_REMAP_SPACE);
	update_lrrw_kend(em8xxx_kmem_end);
#else
	/*
	 * check/fill the memcfg
	 */
	memcfg_t *m = (memcfg_t *)KSEG1ADDR(MEM_BASE_dram_controller_0 + FM_MEMCFG);
	em8xxx_kmem_size = ((size + em8xxx_kmem_start) & 0xfff00000) - em8xxx_kmem_start;
	em8xxx_kmem_end = KSEG1ADDR(em8xxx_kmem_start + em8xxx_kmem_size) - KSEG1ADDR(MEM_BASE_dram_controller_0);
	if (is_valid_memcfg(m) == 0) {
		printk("Invalid MEMCFG, creating new one at 0x%08x.\n", MEM_BASE_dram_controller_0 + FM_MEMCFG);
		memset(m, 0, sizeof (memcfg_t));
		m->signature = MEMCFG_SIGNATURE;
		m->dram0_size = TANGOX_SYSTEMRAM_ACTUALSIZE;
		m->kernel_end = em8xxx_kmem_end;
		gen_memcfg_checksum(m);
	} else {
		printk("Valid MEMCFG found at 0x%08x.\n", MEM_BASE_dram_controller_0 + FM_MEMCFG);
		m->kernel_end = em8xxx_kmem_end;
		gen_memcfg_checksum(m);
	}
#endif

	return;
}

#ifdef CONFIG_HIGHMEM
static void __init tangox_himem_setup(unsigned long *start, unsigned long *size)
{
	unsigned long himem_ga = *start, himem_sz = *size, himem_end;
	
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	int i;
	unsigned long remap_ga, remap_sz, remap_end, kmem_sz;

	if ((himem_ga == 0) || (himem_sz == 0))
		return;
	/* align highmem area to page boundary */
	himem_ga = (himem_ga & PAGE_MASK) + ((himem_ga & ~PAGE_MASK) ? PAGE_SIZE : 0);
	himem_end = (himem_ga + himem_sz) & PAGE_MASK;
	if (himem_ga >= himem_end) 
		goto no_highmem;

	for (i = REMAP_IDX, kmem_sz = em8xxx_kmem_size; (kmem_sz > 0) && (i < 8); i++) {
		remap_ga = em8xxx_remap_registers[i];
		remap_sz = (kmem_sz >= 0x04000000UL) ? 0x04000000UL : kmem_sz;
		remap_end = remap_ga + remap_sz;

		/* if overlap found in remapped memory and highmem area, we need
		   to adjust highmem area accordingly. */
		if ((himem_ga >= remap_ga) && (himem_ga < remap_end))
			himem_ga = remap_end;
		if ((himem_end > remap_ga) && (himem_end <= remap_end))
			himem_end = remap_ga;

		if (himem_ga >= himem_end) 
			goto no_highmem;
		kmem_sz -= remap_sz;
	}
#else
	memcfg_t *m = (memcfg_t *)KSEG1ADDR(MEM_BASE_dram_controller_0 + FM_MEMCFG);

	if ((himem_ga == 0) || (himem_sz == 0))
		return;
	/* align highmem area to page boundary */
	himem_ga = (himem_ga & PAGE_MASK) + ((himem_ga & ~PAGE_MASK) ? PAGE_SIZE : 0);
	himem_end = (himem_ga + himem_sz) & PAGE_MASK;
	if (himem_ga >= himem_end) 
		goto no_highmem;

	/* adjust highmem area to restrict it to DRAM1 only */
	if (himem_ga < MEM_BASE_dram_controller_1)
		himem_ga = MEM_BASE_dram_controller_1;
	else if (himem_ga > MEM_BASE_dram_controller_1 + m->dram1_size) 
		goto no_highmem;

	if (himem_end > MEM_BASE_dram_controller_1 + m->dram1_size)
		himem_end = MEM_BASE_dram_controller_1 + m->dram1_size;
	else if (himem_end < MEM_BASE_dram_controller_1) 
		goto no_highmem;

	if (himem_ga >= himem_end) 
		goto no_highmem;
#endif

	*start = himem_ga;
	*size = himem_end - himem_ga;
	return;

no_highmem:
	*start = *size = 0; /* no highmem available */
	return;
}
#endif

void __init prom_init(void)
{
	extern char _text;
	unsigned long offset;
	int clksel, xenv_res = 0, i;
	char *revStr = NULL;

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
#elif defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
		i < 8;
#endif
		i++) {
		em8xxx_remap_registers[i] = gbus_read_reg32(REG_BASE_cpu_block + CPU_remap + (i * 4));
	}
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	em8xxx_remap_registers[8] = MEM_BASE_dram_controller_0 + 0x0c000000UL; /* Fixed "remap" */
#endif

	/* 
	 * Set remap so that 0x1fc00000 and 0x0 back to they should be...
	 */
	gbus_write_reg32(REG_BASE_cpu_block + CPU_remap, 0x1fc00000);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_remap1, 0x0);
	iob();
	
#if defined(CONFIG_TANGO2_SMP863X)
	printk("Configured for SMP863%c (revision %s), ",
			'x', "ES6+/RevA+"
	      );
#elif defined(CONFIG_TANGO3_SMP86XX)
	printk("Configured for SMP%s, ",
#if defined(CONFIG_TANGO3_865X)
			"865x"
#elif defined(CONFIG_TANGO3_864X)
			"864x"
#elif defined(CONFIG_TANGO3_867X)
			"867x"
#elif defined(CONFIG_TANGO3_868X)
			"868x"
#endif
	      );
#elif defined(CONFIG_TANGO4_891X)
	printk("Configured for SM891%c (revision %s), ",
			'x', "ES2"
	      );
#else
#error Unsupported platform.
#endif
	printk("detected SMP%lx (revision ", (tangox_chip_id()>>16)&0xffff);
#if defined(CONFIG_TANGO2)
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
	} 
#elif defined(CONFIG_TANGO3)
	if (is_tango3_chip()) {
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
			case 0x5: /* ES5 */
				revStr = "ES5";
				break;
			case 0x6: /* ES6 */
				revStr = "ES6";
				break;
			default: /* Unknown */
				revStr = "unknown";
				break;
		}
	} 
#elif defined(CONFIG_TANGO4)
	if (is_tango4_chip()) {
		unsigned long revision = tangox_chip_id() & 0xff;
		switch(revision) {
			case 0x1: /* ES1 */
				revStr = "ES1";
				break;
			case 0x2: /* ES2 */
				revStr = "ES2";
				break;
			default: /* Unknown */
				revStr = "unknown";
				break;
		}
	}
#endif
	else
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
	orig_cpu_freq = em8xxx_cpu_frequency = tangox_get_cpuclock();
	em8xxx_sys_frequency = tangox_get_sysclock();

	em8xxx_sys_clkgen_pll = RD_BASE_REG32(SYS_clkgen_pll);
	em8xxx_sys_premux = RD_BASE_REG32(SYS_sysclk_premux) & 0x3;
	em8xxx_sys_mux = RD_BASE_REG32(SYS_sysclk_mux) & 0xf01;

	/*
	 * program the right clock divider in both uart
	 */
#ifdef CONFIG_TANGOX_UART_USE_SYSCLK
	clksel = 0;
#else
	clksel = 1;
#endif
#if defined(CONFIG_TANGO4)
	gbus_write_reg32(REG_BASE_system_block + 0x700 + CPU_UART_CLKSEL, clksel);
#elif defined(CONFIG_TANGO3)
 	if ((((tangox_chip_id() >> 16) & 0xfffe) == 0x8656) || (((tangox_chip_id() >> 16) & 0xfffe) == 0x8672) || (((tangox_chip_id() >> 16) & 0xfffe) == 0x8674))
 		gbus_write_reg32(REG_BASE_system_block + 0x700 + CPU_UART_CLKSEL, clksel);
 	else
		gbus_write_reg32(REG_BASE_cpu_block + CPU_UART0_base + CPU_UART_CLKSEL, clksel);
#else
	gbus_write_reg32(REG_BASE_cpu_block + CPU_UART0_base + CPU_UART_CLKSEL, clksel);
#endif
	gbus_write_reg32(REG_BASE_cpu_block + CPU_UART1_base + CPU_UART_CLKSEL, clksel);
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
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

	tangox_mem_setup(em8xxx_kmem_size);
#ifdef CONFIG_HIGHMEM
	tangox_get_himem_info(&em8xxx_himem_start, &em8xxx_himem_size);
	tangox_himem_setup(&em8xxx_himem_start, &em8xxx_himem_size);
#endif

	/*
	 * tell kernel about available memory size/offset
	 */
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	offset = KSEG1ADDR(em8xxx_kmem_start) - KSEG1ADDR(CPU_REMAP_SPACE);
	add_memory_region(CPU_REMAP_SPACE + offset, em8xxx_kmem_size, BOOT_MEM_RAM);
#else
	offset = KSEG1ADDR(em8xxx_kmem_start) - KSEG1ADDR(MEM_BASE_dram_controller_0);
	add_memory_region(MEM_BASE_dram_controller_0 + offset, em8xxx_kmem_size, BOOT_MEM_RAM);
#endif
#ifdef CONFIG_HIGHMEM
	if ((em8xxx_himem_start != 0) && (em8xxx_himem_size != 0)) {
		add_memory_region(em8xxx_himem_start, em8xxx_himem_size, BOOT_MEM_RAM);
		printk("adding [0x%08lx..0x%08lx) as highmem area.\n", em8xxx_himem_start, em8xxx_himem_start + em8xxx_himem_size);
	}
#endif

	arcs_cmdline[COMMAND_LINE_SIZE - 1] = '\0';
#ifndef CONFIG_TANGOX_IGNORE_CMDLINE
	/*
	 * set up correct linux command line according to XENV, memcfg
	 * and YAMON args, if not told to ignore them
	 */
#ifdef CONFIG_TANGOX_XENV_READ
	/* If specified by xenv, override the command line */
	if (tangox_xenv_cmdline())
		strncpy(arcs_cmdline, tangox_xenv_cmdline(), COMMAND_LINE_SIZE - 1);
#ifdef CONFIG_CMDLINE
	else
		strncpy(arcs_cmdline, CONFIG_CMDLINE, COMMAND_LINE_SIZE - 1);
#endif
#else
	strncpy(arcs_cmdline, CONFIG_CMDLINE, COMMAND_LINE_SIZE - 1);
#endif

	/* If specified by memcfg, override the command line */
//	if (m->linux_cmd != 0 && strnlen((char *)KSEG1ADDR(m->linux_cmd), COMMAND_LINE_SIZE - 1) > 0)
//disabled. e.m. 2006feb3rd		strncpy(arcs_cmdline, (char *)KSEG1ADDR(m->linux_cmd), COMMAND_LINE_SIZE - 1);

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

			len = strnlen(argv[i], COMMAND_LINE_SIZE - 1);
			if (pos + 1 + len + 1 > sizeof (arcs_cmdline))
				break;
			if (pos)
				arcs_cmdline[pos++] = ' ';
			strncpy(arcs_cmdline + pos, argv[i], COMMAND_LINE_SIZE - (pos + 1));
			pos += len;
		}
	}
#else
#ifdef CONFIG_CMDLINE
	strncpy(arcs_cmdline, CONFIG_CMDLINE, COMMAND_LINE_SIZE - 1);
#endif
#endif /* !CONFIG_TANGOX_IGNORE_CMDLINE */

	mips_machtype = MACH_TANGOX;

#ifdef CONFIG_MIPS_MT_SMP
#ifdef CONFIG_MIPS_CMP
	register_smp_ops(&cmp_smp_ops);
#else
	register_smp_ops(&vsmp_smp_ops);
#endif
#endif
	return;
}

void __init prom_free_prom_memory(void)
{
	return;
}

EXPORT_SYMBOL(tangox_get_sysclock);
EXPORT_SYMBOL(tangox_get_cpuclock);
EXPORT_SYMBOL(tangox_get_dspclock);
EXPORT_SYMBOL(tangox_get_pllclock);
EXPORT_SYMBOL(tangox_get_cdclock);

unsigned long tangox_chip_id(void)
{
	unsigned long chip_id = 0;
	if (chip_id == 0)
		chip_id = ((gbus_read_reg32(REG_BASE_host_interface + PCI_REG0) & 0xffff) << 16) |
				(gbus_read_reg32(REG_BASE_host_interface + PCI_REG1) & 0xff);
	return chip_id;
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
			case 0x8672:
			case 0x8674:
			case 0x8670:
			case 0x8680:
			case 0x8682:
			case 0x868A:
			case 0x8656:
			case 0x8646:
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
	return ((chip == 0x8640) || (chip == 0x8650) || (chip == 0x8670) || (chip == 0x8680)) ? 1 : 0;
}

int is_tango4_chip(void)
{
	unsigned long chip = (tangox_chip_id()>>16) & 0xff00;
	return (chip == 0x8900) ? 1 : 0;
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
	__flush_cache_all();
}

EXPORT_SYMBOL(tangox_flush_cache_all);
EXPORT_SYMBOL(tangox_chip_id);
EXPORT_SYMBOL(is_tango2_chip);
EXPORT_SYMBOL(is_tango3_chip);
EXPORT_SYMBOL(is_tango4_chip);
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
EXPORT_SYMBOL(em8xxx_kmem_size);
EXPORT_SYMBOL(em8xxx_kmem_start);
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
EXPORT_SYMBOL(max_remap_size);
EXPORT_SYMBOL(em8xxx_remap_registers);
#endif

int tangox_get_order(unsigned long size)
{
	return(get_order(size));
}
EXPORT_SYMBOL(tangox_get_order);

void tangox_do_timer(unsigned long ticks)
{
	extern void do_timer(unsigned long ticks);
	extern seqlock_t xtime_lock;

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
	start_pg_offset = start & ~PAGE_MASK; /* offset within start page */
	end_pg_addr = ((start + len) & PAGE_MASK) - (((start + len) & ~PAGE_MASK) ? 0 : PAGE_SIZE); /* address of last page */

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

/* convering virtual address to physical address (perform page table walking if needed) */
unsigned long tangox_virt_to_phys(void *pvaddr)
{
	unsigned long vpa = (unsigned long)pvaddr & PAGE_MASK;
	if (vpa >= KSEG2)
		return (pte_val(*(pte_t *)pte_offset(pmd_offset(pud_offset(pgd_offset_k(vpa), vpa), vpa), vpa)) & PAGE_MASK) + ((unsigned long)pvaddr & ~PAGE_MASK);
	else if (vpa >= KSEG0)
		return virt_to_phys((void *)CKSEG0ADDR(pvaddr));
 	else
		return (pte_val(*(pte_t *)pte_offset(pmd_offset(pud_offset(pgd_offset_gate(current->mm, vpa), vpa), vpa), vpa)) & PAGE_MASK) + ((unsigned long)pvaddr & ~PAGE_MASK);
}
EXPORT_SYMBOL(tangox_virt_to_phys);

#ifndef CONFIG_TANGOX_XENV_READ
int zxenv_get(char *recordname, void *dst, u32 *datasize)
{
	return -EIO;
}
EXPORT_SYMBOL(zxenv_get);
#endif

