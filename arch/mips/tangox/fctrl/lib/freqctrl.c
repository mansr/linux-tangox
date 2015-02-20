/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
 *
 */

#include "platform.h"
#include "freqctrl.h"

#include "fc/fcconfig.h"
#include "fc/fcbin.h"

#ifndef LOCALMEM_BASE
#error Anchor of handler need to be set
#endif

#define USE_AUDIOMEM_SIZE	(8*1024)

static unsigned long localmem[8192/4]; /* Up to 8KB */
static unsigned long audiomem[USE_AUDIOMEM_SIZE/4]; /* Up to 8KB */

struct engine_status
{
	unsigned long hb_addr;
	unsigned long suspend_addr;
	unsigned long hb;
	int active;
	int g2l_status;
	unsigned long g2lc_addr;
	const char *desc;
};

static struct engine_status engines[] = {
	{ REG_BASE_cpu_block + LR_HB_MPEG0, REG_BASE_cpu_block + LR_SUSPEND_ACK_MPEG0, 0, 0, 0, REG_BASE_mpeg_engine_0 + G2L_RESET_CONTROL, "MPEG0" },
	{ REG_BASE_cpu_block + LR_HB_MPEG1, REG_BASE_cpu_block + LR_SUSPEND_ACK_MPEG1, 0, 0, 0, REG_BASE_mpeg_engine_1 + G2L_RESET_CONTROL, "MPEG1" },
	{ REG_BASE_cpu_block + LR_HB_AUDIO0, REG_BASE_cpu_block + LR_SUSPEND_ACK_AUDIO0, 0, 0, 0, REG_BASE_audio_engine_0 + G2L_RESET_CONTROL, "AUDIO0" },
	{ REG_BASE_cpu_block + LR_HB_AUDIO1, REG_BASE_cpu_block + LR_SUSPEND_ACK_AUDIO1, 0, 0, 0, REG_BASE_audio_engine_1 + G2L_RESET_CONTROL, "AUDIO1" },
	{ REG_BASE_cpu_block + LR_HB_DEMUX, REG_BASE_cpu_block + LR_SUSPEND_ACK_DEMUX, 0, 0, 0, REG_BASE_demux_engine + G2L_RESET_CONTROL, "DEMUX" },
	{ REG_BASE_cpu_block + LR_HB_IH, REG_BASE_cpu_block + LR_SUSPEND_ACK_IH, 0, 0, 0, 0, "IH" },
};

#define SUSPEND_FLAG     1
#define RESUME_FLAG      0
#define MAX_DELTA        2700000 /* 100ms */
#define HB_MONITOR_TIME  100000  /* 100ms */

#ifdef WITH_INPUT_POLLING
int ir_open(void);
int fip_open(void);
int eth_open(void);
int ir_close(void);
int fip_close(void);
int eth_close(void);
#endif

static unsigned long xtal_delta(unsigned long start, unsigned long end)
{
	return (end > start) ? (end - start) : (0xffffffff - start + end + 1);
}

/* Suspend all engines */
static void suspend_engines(void)
{
	const int num_engines = sizeof(engines) / sizeof(struct engine_status);
	unsigned long start, end, delta;
	int i;

	/* First determine which engine is running */
	for (i = 0; i < num_engines; i++) {
		engines[i].hb = tangox_gbus_read_reg32(engines[i].hb_addr);
		if (engines[i].g2lc_addr != 0) /* update block status */
			engines[i].g2l_status = tangox_gbus_read_reg32(engines[i].g2lc_addr);
	}

	tangox_udelay(HB_MONITOR_TIME); /* Wait for heartbeat update */

	/* Mark the active flag associate to each engine */
	for (i = 0; i < num_engines; i++) 
		engines[i].active = ((engines[i].hb != tangox_gbus_read_reg32(engines[i].hb_addr)) ? 1 : 0);

	/* Start suspend */
	tangox_gbus_write_reg32(REG_BASE_cpu_block + LR_DRAM_DMA_SUSPEND, SUSPEND_FLAG);
	tangox_sync();

	/* Check all engines to be suspended */
	for (i = 0; i < num_engines; i++) {
		if (engines[i].active) {
			DBG_PRINT("Waiting for %s to be suspended.\n", engines[i].desc);
			for (delta = 0, start = tangox_gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt); 
					tangox_gbus_read_reg32(engines[i].suspend_addr) != SUSPEND_FLAG; ) {
				end = tangox_gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt); 
				delta = xtal_delta(start, end);
				if (delta > MAX_DELTA)
					break;
			}
			if (delta <= MAX_DELTA)
				DBG_PRINT("%s is suspended.\n", engines[i].desc);
			else
				DBG_PRINT("Wait for %s timeout.\n", engines[i].desc);
		} else {
			DBG_PRINT("%s is not active.\n", engines[i].desc);
		}
		if ((engines[i].g2lc_addr != 0) && (engines[i].g2l_status == 0)) { /* currently running */
			tangox_gbus_write_reg32(engines[i].g2lc_addr, 1); /* pause the block */
			tangox_sync();
		}
	}
}

/* Resume all engines */
static void resume_engines(void)
{
	const int num_engines = sizeof(engines) / sizeof(struct engine_status);
	int i;
	unsigned long start, end, delta;

	for (i = num_engines - 1; i >= 0; i--) {
		if ((engines[i].g2lc_addr != 0) && (engines[i].g2l_status == 0)) { /* used to be running */
			tangox_gbus_write_reg32(engines[i].g2lc_addr, 0); /* undo pause */
			tangox_sync();
		}
	}

	/* Start resume */
	tangox_gbus_write_reg32(REG_BASE_cpu_block + LR_DRAM_DMA_SUSPEND, RESUME_FLAG);
	tangox_sync();

	/* Check all engines to be suspended, in reversed order */
	for (i = num_engines - 1; i >= 0; i--) {
		if (engines[i].active) {
			DBG_PRINT("Waiting for %s to be resumed.\n", engines[i].desc);
			for (delta = 0, start = tangox_gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt); 
					tangox_gbus_read_reg32(engines[i].suspend_addr) == SUSPEND_FLAG; ) {
				end = tangox_gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt); 
				delta = xtal_delta(start, end);
				if (delta > MAX_DELTA)
					break;
			}
			if (delta <= MAX_DELTA)
				DBG_PRINT("%s is resumed.\n", engines[i].desc);
			else
				DBG_PRINT("Wait for %s timeout.\n", engines[i].desc);
		}
	}
}

/* Set to a given frequency profile */
#ifdef WITH_INPUT_POLLING
int set_freq_profile(const struct freq_profile *pptr, unsigned long *elapse, unsigned long *ret_key, 
	unsigned long *ir_keys, unsigned long *fip_keys, int input_ctrl, 
	unsigned long *cnt_low, unsigned long *cnt_high)
#else
int set_freq_profile(const struct freq_profile *pptr)
#endif
{
	unsigned long ehci_cmd, ohci_ctl;
	unsigned long flags, ret, old_pll2, old_pll1, old_pll0, audio0_stat, gpio_dir = 0, gpio_data = 0;
	int standby = 0;
	struct freq_handler_header *fs_hdr = (struct freq_handler_header *)(LOCALMEM_BASE + 0x10);

#ifdef WITH_INPUT_POLLING
	if (elapse)
		*elapse = 0;
	if (ret_key)
		*ret_key = 0;
#endif

	if (pptr->pll3 == 0)
		return -EINVAL;
	standby = pptr->standby;

	DBG_PRINT("TangoX Frequency Control\n");

	/* Suspend all engines first */
	suspend_engines();

	DBG_PRINT("Installing handler to 0x%08x, size 0x%x.\n", LOCALMEM_BASE, sizeof(binout));

	/* Save localmem content */
	memcpy(localmem, (void *)KSEG1ADDR(REG_BASE_cpu_block), LR_STACKTOP);
	memset((void *)KSEG1ADDR(REG_BASE_cpu_block), 0, LR_STACKTOP);

	audio0_stat = tangox_gbus_read_reg32(REG_BASE_audio_engine_0 + G2L_RESET_CONTROL);
	tangox_gbus_write_reg32(REG_BASE_audio_engine_0 + G2L_RESET_CONTROL, 1); 
	tangox_sync();

	memcpy(audiomem, (void *)KSEG1ADDR(LOCALMEM_BASE & PAGE_MASK), USE_AUDIOMEM_SIZE);
	memcpy((void *)(LOCALMEM_BASE), &binout[0], sizeof(binout));

	DBG_PRINT("Stopping PCI bus artitration...\n");
	tangox_gbus_write_reg32(REG_BASE_host_interface + PCI_host_reg4, 1);
	tangox_sync();

	DBG_PRINT("Stoping TangoX ethernet...\n");
	if (!standby) 
		eth_open();

	DBG_PRINT("Stopping TangoX EHCI\n");
        ehci_cmd = tangox_gbus_read_reg32(EHCI_CMD);
        ehci_cmd &= ~CMD_RUN;
        tangox_gbus_write_reg32(EHCI_CMD, ehci_cmd);

	DBG_PRINT("Stopping TangoX OHCI\n");
	ohci_ctl = tangox_gbus_read_reg32(OHCI_CONTROL);
	tangox_gbus_write_reg32(OHCI_CONTROL, ohci_ctl | HCCONTROL_HCFS);
	tangox_gbus_write_reg32(OHCI_INT_DISABLE, OHCI_INTR_MIE);
	tangox_usdelay(16*125); /*need to wait 16*125us*/

	if (pptr->gpio_dir != 0) {
		unsigned long dir, data;

		/* Save GPIO dir/data, to be restored later */
		gpio_dir = ((tangox_gbus_read_reg32(REG_BASE_host_interface + ETH_gpio_dir2) & 0xffff) << 16) |
			(tangox_gbus_read_reg32(REG_BASE_system_block + SYS_gpio_dir) & 0xffff);
		gpio_data = (((tangox_gbus_read_reg32(REG_BASE_host_interface + ETH_gpio_data2) & 0xffff) << 16) |
			(tangox_gbus_read_reg32(REG_BASE_system_block + SYS_gpio_data) & 0xffff)) & gpio_dir;

		if (pptr->gpio_dir & 0xffff) {
			dir = pptr->gpio_dir & 0xffff;
			data = (pptr->gpio_data & 0xffff) & dir;
			tangox_gbus_write_reg32(REG_BASE_system_block + SYS_gpio_data, (dir << 16) | data);
			tangox_gbus_write_reg32(REG_BASE_system_block + SYS_gpio_dir, (dir << 16) | dir);
		}
		if (pptr->gpio_dir & 0xffff0000) {
			dir = (pptr->gpio_dir >> 16) & 0xffff;
			data = ((pptr->gpio_data >> 16) & 0xffff) & dir;
			tangox_gbus_write_reg32(REG_BASE_host_interface + ETH_gpio_data2, (dir << 16) | data);
			tangox_gbus_write_reg32(REG_BASE_host_interface + ETH_gpio_dir2, (dir << 16) | dir);
		}
	}

	DBG_PRINT("Original config:\n");
	DBG_PRINT(" PLL3: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen3_pll)); 
	DBG_PRINT(" PLL2: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen2_pll)); 
	DBG_PRINT(" PLL1: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen1_pll)); 
	DBG_PRINT(" PLL0: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen0_pll)); 
	DBG_PRINT(" MUX:  0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_sysclk_mux)); 
	DBG_PRINT(" D0_CFG: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_cfg));
	DBG_PRINT(" D0_DELAY: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_delay0_ctrl));
	DBG_PRINT(" D1_CFG: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_cfg));
	DBG_PRINT(" D1_DELAY: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_delay0_ctrl));

	DBG_PRINT("Target config (0 = no change):\n");
	DBG_PRINT(" PLL3: 0x%08lx\n", pptr->pll3);
	DBG_PRINT(" PLL2: 0x%08lx\n", pptr->pll2);
	DBG_PRINT(" PLL1: 0x%08lx\n", pptr->pll1);
	DBG_PRINT(" PLL0: 0x%08lx\n", pptr->pll0);
	DBG_PRINT(" MUX:  0x%08lx\n", pptr->mux);
	DBG_PRINT(" D0_CFG: 0x%08lx\n", pptr->d0cfg);
	DBG_PRINT(" D0_DELAY: 0x%08lx\n", pptr->d0delays);
	DBG_PRINT(" D1_CFG: 0x%08lx\n", pptr->d1cfg);
	DBG_PRINT(" D1_DELAY: 0x%08lx\n", pptr->d1delays);

	old_pll2 = tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen2_pll);
	if (pptr->pll2) {
		/* Change PLL2 if needed */
		DBG_PRINT("Change PLL2 from 0x%08lx to 0x%08lx\n", old_pll2, pptr->pll2);
		tangox_gbus_write_reg32(REG_BASE_system_block+SYS_clkgen2_pll, pptr->pll2);
	}
	old_pll1 = tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen1_pll);
	if (pptr->pll1) {
		/* Change PLL1 if needed */
		DBG_PRINT("Change PLL1 from 0x%08lx to 0x%08lx\n", old_pll1, pptr->pll1);
		tangox_gbus_write_reg32(REG_BASE_system_block+SYS_clkgen1_pll, pptr->pll1);
	}
	old_pll0 = tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen0_pll);
	if (pptr->pll0) {
		/* Change PLL0 if needed */
		DBG_PRINT("Change PLL0 from 0x%08lx to 0x%08lx\n", old_pll0, pptr->pll0);
		tangox_gbus_write_reg32(REG_BASE_system_block+SYS_clkgen0_pll, pptr->pll0);
	}
	tangox_usdelay(10);

	DBG_PRINT("Saving to data 0x%08x\n", (unsigned int)fs_hdr->param_area);
	memcpy((void *)fs_hdr->param_area, pptr, sizeof(*pptr));

#ifdef WITH_INPUT_POLLING
	DBG_PRINT("IR table at 0x%08lx, FIP table at 0x%08lx\n", (unsigned long)fs_hdr->ir_table, (unsigned long)fs_hdr->fip_table);

	/* Clear the input table first */
	memset(fs_hdr->ir_table, 0, INPUT_TABLE_SIZE);
	memset(fs_hdr->fip_table, 0, INPUT_TABLE_SIZE);

	if (tangox_ir_enabled() && (ir_keys != NULL)) {
		memcpy(fs_hdr->ir_table, ir_keys, INPUT_TABLE_SIZE);
		DBG_PRINT("Setting up IR table\n");
	}
	if (tangox_fip_enabled() && (fip_keys != NULL)) {
		memcpy(fs_hdr->fip_table, fip_keys, INPUT_TABLE_SIZE);
		DBG_PRINT("Setting up FIP table\n");
	}

	/* From XENV or other means, determine which input is available */
	fs_hdr->input_ctrl = input_ctrl;

	DBG_PRINT("Polling ctrl: %ld\n", fs_hdr->input_ctrl);
#endif

	DBG_PRINT("Disable interrupts, doing frequency adjustment\n");

	tangox_save_flags(flags);

#ifdef WITH_INPUT_POLLING
	if (standby) {
		DBG_PRINT("Ready to enter stand-by mode at 0x%08x to [%s]\n", (unsigned int)fs_hdr->func_ptr, pptr->desc);
		fs_hdr->cnt_low = *cnt_low;
		fs_hdr->cnt_high = *cnt_high;
	} else {
		DBG_PRINT("Calling handler at 0x%08x to [%s]\n", (unsigned int)fs_hdr->func_ptr, pptr->desc);
	}
#else
	DBG_PRINT("Calling handler at 0x%08x to [%s]\n", (unsigned int)fs_hdr->func_ptr, pptr->desc);
#endif

#ifdef WITH_INPUT_POLLING
	if (standby) {
		if (input_ctrl & 1)
			ir_open();
		if (input_ctrl & 2)
			fip_open();
		if (input_ctrl & 4)
			eth_open();
	}
#endif

	tangox_flush_cache_all();

	ret = (*(fs_hdr->func_ptr))();

#ifdef WITH_INPUT_POLLING
	if (elapse)
		*elapse = fs_hdr->elapse;
	if (ret_key)
		*ret_key = fs_hdr->ret_key;

	if (standby) {
		if (input_ctrl & 4)
			eth_close();
		if (input_ctrl & 2)
			fip_close();
		if (input_ctrl & 1)
			ir_close();
	}
#endif

#ifdef WITH_INPUT_POLLING
	if (standby) {
		DBG_PRINT("\nReturning from standby mode.\n");
		if ((pptr->pll2) && (old_pll2 != tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen2_pll))) {
			DBG_PRINT("Restoring PLL2: 0x%08lx\n", old_pll2);
			tangox_gbus_write_reg32(REG_BASE_system_block+SYS_clkgen2_pll, old_pll2);
		}
		if ((pptr->pll1) && (old_pll1 != tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen1_pll))) {
			DBG_PRINT("Restoring PLL1: 0x%08lx\n", old_pll1);
			tangox_gbus_write_reg32(REG_BASE_system_block+SYS_clkgen1_pll, old_pll1);
		}
		if ((pptr->pll0) && (old_pll0 != tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen0_pll))) {
			DBG_PRINT("Restoring PLL0: 0x%08lx\n", old_pll0);
			tangox_gbus_write_reg32(REG_BASE_system_block+SYS_clkgen0_pll, old_pll0);
		}
		tangox_usdelay(10);
	} else {
		DBG_PRINT("\nReturning from handler.\n");
	}
#else
	DBG_PRINT("\nReturning from handler.\n");
#endif

	if (pptr->gpio_dir != 0) {
		unsigned long dir, data;

		/* Restoring GPIO */
		if (gpio_dir & 0xffff) {
			dir = gpio_dir & 0xffff;
			data = (gpio_data & 0xffff) & dir;
			tangox_gbus_write_reg32(REG_BASE_system_block + SYS_gpio_data, (dir << 16) | data);
			tangox_gbus_write_reg32(REG_BASE_system_block + SYS_gpio_dir, 0xffff0000 | dir);
		}
		if (gpio_dir & 0xffff0000) {
			dir = (gpio_dir >> 16) & 0xffff;
			data = ((gpio_data >> 16) & 0xffff) & dir;
			tangox_gbus_write_reg32(REG_BASE_host_interface + ETH_gpio_data2, (dir << 16) | data);
			tangox_gbus_write_reg32(REG_BASE_host_interface + ETH_gpio_dir2, 0xffff0000 | dir);
		}
	}

	DBG_PRINT("Re-enable TangoX OHCI\n");
	tangox_gbus_write_reg32(OHCI_CONTROL, ohci_ctl);
	tangox_gbus_write_reg32(OHCI_INT_ENABLE, OHCI_INTR_MIE);

	DBG_PRINT("Re-enable TangoX EHCI\n");
        ehci_cmd |= CMD_RUN;
        tangox_gbus_write_reg32(EHCI_CMD, ehci_cmd);

	DBG_PRINT("Re-enable TangoX ethernet\n");
	if (!standby)
		eth_close();

	DBG_PRINT("Re-starting PCI bus artitration...\n");
	tangox_gbus_write_reg32(REG_BASE_host_interface + PCI_host_reg4, 0);

#ifdef WITH_INPUT_POLLING
	/* Re-calibrate system timing */
	if (standby) 
		tangox_reset_timer(tangox_get_cpuclock(), tangox_get_sysclock());
#endif

	DBG_PRINT("NEW config:\n");
	DBG_PRINT(" PLL3: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen3_pll)); 
	DBG_PRINT(" PLL2: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen2_pll)); 
	DBG_PRINT(" PLL1: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen1_pll)); 
	DBG_PRINT(" PLL0: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_clkgen0_pll)); 
	DBG_PRINT(" MUX:  0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_system_block+SYS_sysclk_mux)); 
	DBG_PRINT(" D0_CFG: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_cfg));
	DBG_PRINT(" D0_DELAY: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_delay0_ctrl));
	DBG_PRINT(" D1_CFG: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_cfg));
	DBG_PRINT(" D1_DELAY: 0x%08lx\n", tangox_gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_delay0_ctrl));

	DBG_PRINT("NEW CPU frequency: %ldHz\n", tangox_get_cpuclock());
	DBG_PRINT("NEW SYS frequency: %ldHz\n", tangox_get_sysclock());
	DBG_PRINT("NEW DSP frequency: %ldHz\n", tangox_get_dspclock());

#ifdef WITH_INPUT_POLLING
	if (standby) {
		*cnt_low = fs_hdr->cnt_low;
		*cnt_high = fs_hdr->cnt_high;
	}
#endif

	tangox_restore_flags(flags);

	/* restore localmem content */
	memcpy((void *)KSEG1ADDR(REG_BASE_cpu_block), localmem, LR_STACKTOP);

	/* restore audio mem */
	memcpy((void *)KSEG1ADDR(LOCALMEM_BASE & PAGE_MASK), audiomem, USE_AUDIOMEM_SIZE);

	tangox_gbus_write_reg32(REG_BASE_audio_engine_0 + G2L_RESET_CONTROL, audio0_stat); 
	tangox_sync();

	/* Resume the engines */
	resume_engines();

	return 0;
}

/* Calculate the sys_clkgen_pll value for certain frequency */
/* freq = desired frequency, ratio = 1 (for PLL), 2-4 (for CPU/SYS/DSP) */
unsigned long frequency_to_pll(unsigned int freq, unsigned int ratio) 
{
	unsigned int m, n;
	unsigned long pll = 0;
  
	if ((freq < 255) && (ratio != 4)) {
		// Frequency given in MHz. Use Intermediate Freq of 0.5MHz
		// !! Impossible if ratio=4.
		m = 54 / ratio; 
		n = freq * 2; 
		pll = 0x1000000 + ((m - 2) << 16) + (n - 2);
	} else {
		unsigned int best_n, best_m, best_f, tmp;
		if (freq < 255) 
			freq *= 1000000;
		// Frequency in Hz. Find closest match (Try for all M's)
		best_f = 1000000000;
		best_m = 0;
		best_n = 0;
		for (m = 65; m > 2; m--) {
			n =((freq / 1000) * m * ratio + 13500) / 27000;
			if ((n <= 513) && (n >= 2)) {
				tmp = (TANGOX_BASE_FREQUENCY / (m*ratio)) * n;
				if (tmp < freq) 
					tmp = 2 * freq - tmp;
				if (tmp <= best_f) {
					best_f = tmp; 
					best_m = m; 
					best_n = n;
				}
			}
		}
		if (best_m != 0)
			pll = 0x1000000 + ((best_m - 2) << 16) + (best_n - 2);
	}
	return(pll);
}

