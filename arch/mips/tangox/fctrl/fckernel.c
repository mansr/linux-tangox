/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <asm/tlbflush.h>

#include "lib/fc/fcconfig.h"
#include "lib/freqctrl.h"

#include <linux/interrupt.h>
#include <asm/tango2/rmem86xxid.h>
#include <asm/tango2/rmdefs.h>
#include <asm/tango2/emhwlib_dram.h>
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/tango2.h>
#include <asm/tango2/tango2api.h>
#include <asm/tango2/memcfg.h>

/* The names used in proc_fs */
#define PROC_ROOT               "tangoxfreq"
#define PROC_PROFILE            "profile"
#define PROC_MUX                "mux"
#define PROC_PLL3               "pll3"
#define PROC_PLL2               "pll2"
#define PROC_PLL1               "pll1"
#define PROC_PLL0               "pll0"
#define PROC_FREQ               "freq"

#ifdef WITH_INPUT_POLLING
#define PROC_STANDBY            "standby"
#ifdef WITH_IR
#define PROC_IR                 "ir_table"
#endif
#ifdef WITH_FIP
#define PROC_FIP                "fip_table"
#endif
#endif

/* Get XENV parameters related to frequency profile */
extern void tangox_get_ps_config(unsigned long *pll3, unsigned long *pll2, unsigned long *pll1, 
		unsigned long *pll0, unsigned long *mux, unsigned long *d0cfg, unsigned long *d0delay, 
		unsigned long *d1cfg, unsigned long *d1delay);

#ifdef WITH_INPUT_POLLING
extern void tangox_get_standby_config(unsigned long *pll3, unsigned long *pll2, unsigned long *pll1, 
		unsigned long *pll0, unsigned long *mux, unsigned long *d0cfg, unsigned long *d0delay, 
		unsigned long *d1cfg, unsigned long *d1delay, unsigned long *gpio_dir, unsigned long *gpio_data);
#endif

extern int tangox_ir_enabled(void);
extern int tangox_fip_enabled(void);
extern int tangox_ethernet_enabled(void);
extern unsigned long tangox_get_cpuclock(void);
extern unsigned long tangox_get_sysclock(void);
extern unsigned long tangox_get_dspclock(void);
extern unsigned long tangox_get_pllclock(int idx);
extern void tangox_do_timer(unsigned long ticks);

union tangox_cycle_cnt_union {
	u64 cycle64;
	unsigned long cycle32[2];
};
extern union tangox_cycle_cnt_union tangox_cycle_cnt;

static struct freq_profile freq_table[] = {
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "ORIGINAL", 0 }, /* for saving the status when it's been loaded */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "PS XENV", 0 }, /* for getting the ps status when it's been loaded from XENV */
	{ 0x0110009e, 0, 0, 0, 0x301, 0xf21111ba, 0x88888, 0xf21111ba, 0x88888, 0, "SYS/CPU/DSP 80/80/80", 0, 0 },  /* SYS/CPU/DSP: 80/80/80MHz, 2/2.5 */
	{ 0x011000c6, 0, 0, 0, 0x701, 0xf21111ba, 0x97777, 0xf21111ba, 0x97777, 0, "SYS/CPU/DSP 100/150/150", 0, 0 },  /* SYS/CPU/DSP: 150/100/150MHz, 2/2.5 */
	{ 0x01100108, 0, 0, 0, 0x701, 0xf21111ba, 0x97777, 0xf21111ba, 0x97777, 0, "SYS/CPU/DSP 133/200/200", 0, 0 },  /* SYS/CPU/DSP: 200/133/200MHz, 2/2.5 */
	{ 0x0110014a, 0, 0, 0, 0x701, 0xf34111ba, 0xa7777, 0xf34111ba, 0xa7777, 0, "SYS/CPU/DSP 166/250/250", 0, 0 },  /* SYS/CPU/DSP: 250/166/250MHz, 3/4 */
	{ 0x01010041, 0, 0, 0, 0x701, 0xf34111ba, 0xa4444, 0xf34111ba, 0xa4444, 0, "SYS/CPU/DSP 200/300/300", 0, 0 },  /* SYS/CPU/DSP: 300/200/300MHz, 3/4 */
};

#ifdef WITH_INPUT_POLLING
static struct freq_profile standby_mode =
//      { 0x01010010, 0x202, 0x202, 0x202, 0x301, 0xf21111ba, 0x88888, 0xf21111ba, 0x88888, 1, "54/54/54", 0, 0 };  /* CPU/SYS/DSP: 54/54/54MHz, 2/2.5 PLL3 162MHz */
//	{ 0x0119006a, 0x202, 0x202, 0x202, 0x1, 0xf21111ba, 0x88888, 0xf21111ba, 0x88888, 1, "54/54/54", 0, 0 };  /* CPU/SYS/DSP: 54/54/54MHz, 2/2.5 PLL3 108MHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, "STANDBY XENV", 0, 0 };  /* get the settings from XENV */
#endif

static int profile_idx = 0;
static struct proc_dir_entry *root = NULL, *prof_ent = NULL, *mux_ent = NULL, *pll3_ent = NULL;
static struct proc_dir_entry *pll1_ent = NULL, *pll2_ent = NULL, *pll0_ent = NULL, *freq_ent = NULL;

#ifdef WITH_INPUT_POLLING
static struct proc_dir_entry *standby_ent;
#ifdef WITH_IR
static struct proc_dir_entry *ir_ent = NULL;
#endif
#ifdef WITH_FIP
static struct proc_dir_entry *fip_ent = NULL;
#endif
static unsigned long ir_keys[INPUT_TABLE_SIZE/sizeof(unsigned long)];
static unsigned long fip_keys[INPUT_TABLE_SIZE/sizeof(unsigned long)];
static int ir_key_cnt, fip_key_cnt;
#endif

static const unsigned char ratios_table[12][3] = {
	{ 2, 2, 2 },
	{ 4, 2, 4 },
	{ 3, 2, 3 },
	{ 3, 3, 3 },
	{ 3, 3, 3 },
	{ 3, 2, 3 },
	{ 3, 3, 2 },
	{ 3, 2, 2 },
	{ 4, 4, 4 },
	{ 4, 2, 4 },
	{ 4, 4, 2 },
	{ 4, 2, 2 }
};

static void recalc_frequency(void)
{
	extern void tangox_bmide_calc_speed(void);
	extern void tangox_pbide_calc_speed(void);

#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX
	tangox_bmide_calc_speed();
#endif
#ifdef CONFIG_BLK_DEV_PBIDE_TANGOX
	tangox_pbide_calc_speed();
#endif

	/* TODO: may need to re-adjust the CDs */
}

static int change_profile(const char *str)
{
	int prof = simple_strtol(str, NULL, 10);
	int ret = 0;

	if ((prof < 0) || (prof >= (sizeof(freq_table)/sizeof(struct freq_profile)))) 
		return -EINVAL;

	for (; prof < sizeof(freq_table)/sizeof(struct freq_profile); prof++) {
		/* Look for defined profile */
		if (freq_table[prof].pll3 != 0)
			break;
	}

	/* No valid profile found, use original one */
	if (prof >= (sizeof(freq_table)/sizeof(struct freq_profile)))
		prof = 0; 

	if (profile_idx == prof)
		return 0;

	profile_idx = prof;

#ifdef WITH_INPUT_POLLING
	ret = set_freq_profile(&freq_table[prof], NULL, NULL, NULL, NULL, 0, 
				&tangox_cycle_cnt.cycle32[0], &tangox_cycle_cnt.cycle32[1]);
#else
	ret = set_freq_profile(&freq_table[prof]);
#endif

	recalc_frequency();

	return ret;
}

static int change_mux(const char *mux_str)
{
	struct freq_profile freq_prof;
	int mux = simple_strtol(mux_str, NULL, 10);
	int ret = 0;

	if ((mux < 0) || (mux > 11))
		return -EINVAL;

	freq_prof.mux = (gbus_read_reg32(REG_BASE_system_block+SYS_sysclk_mux) & 0xfffff0ff) | (mux << 8);
	freq_prof.d0cfg = gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_cfg);
	freq_prof.d0delays = gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_delay0_ctrl);
	freq_prof.d1cfg = gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_cfg);
	freq_prof.d1delays = gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_delay0_ctrl);
	freq_prof.pll3 = gbus_read_reg32(REG_BASE_system_block+SYS_clkgen3_pll);
	freq_prof.pll2 = freq_prof.pll1 = freq_prof.pll0 = 0;
	freq_prof.standby = 0;

	if (freq_prof.mux != gbus_read_reg32(REG_BASE_system_block+SYS_sysclk_mux))
		profile_idx = -1;

#ifdef WITH_INPUT_POLLING
	ret = set_freq_profile(&freq_prof, NULL, NULL, NULL, NULL, 0, 
				&tangox_cycle_cnt.cycle32[0], &tangox_cycle_cnt.cycle32[1]);
#else
	ret = set_freq_profile(&freq_prof);
#endif

	recalc_frequency();

	return ret;
}

static int pllx_info_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char buf[64];
	unsigned long freq;
	struct freq_profile freq_prof;
	int idx = (int)data;

	if (count > ARRAY_SIZE(buf) - 1)
		count = ARRAY_SIZE(buf) - 1;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = '\0';
	freq = simple_strtol(buf, NULL, 10);

	freq_prof.mux = gbus_read_reg32(REG_BASE_system_block+SYS_sysclk_mux);
	freq_prof.d0cfg = gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_cfg);
	freq_prof.d0delays = gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_delay0_ctrl);
	freq_prof.d1cfg = gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_cfg);
	freq_prof.d1delays = gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_delay0_ctrl);
	freq_prof.standby = 0;
	freq_prof.pll3 = gbus_read_reg32(REG_BASE_system_block+SYS_clkgen3_pll);
	freq_prof.pll2 = gbus_read_reg32(REG_BASE_system_block+SYS_clkgen2_pll);
	freq_prof.pll1 = gbus_read_reg32(REG_BASE_system_block+SYS_clkgen1_pll);
	freq_prof.pll0 = gbus_read_reg32(REG_BASE_system_block+SYS_clkgen0_pll);

	if (idx == 3) {
		freq_prof.pll3 = frequency_to_pll(freq, 1);
	} else if (idx == 2) {
		freq_prof.pll2 = frequency_to_pll(freq, 1);
	} else if (idx == 1) {
		freq_prof.pll1 = frequency_to_pll(freq, 1);
	} else if (idx == 0) {
		freq_prof.pll0 = frequency_to_pll(freq, 1);
	}

	profile_idx = -1;
#ifdef WITH_INPUT_POLLING
	set_freq_profile(&freq_prof, NULL, NULL, NULL, NULL, 0, 
				&tangox_cycle_cnt.cycle32[0], &tangox_cycle_cnt.cycle32[1]);
#else
	set_freq_profile(&freq_prof);
#endif

	recalc_frequency();

	return count;
}

static int pllx_info_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int ret = 0;
	int idx = (int)data;

	ret = sprintf(page + off, "PLL%d: %ldHz\n", idx, tangox_get_pllclock(idx));
	*eof = 1;
	return ret;
}

static int freq_info_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int ret = 0;
	int i;
	for (i = 0; i < 4; i++) {
		ret += sprintf(page + off + ret, "PLL%d: %ldHz\n", i, tangox_get_pllclock(i));
	}
	ret += sprintf(page + off + ret, "SYS: %ldHz\n", tangox_get_sysclock());
	ret += sprintf(page + off + ret, "CPU: %ldHz\n", tangox_get_cpuclock());
	ret += sprintf(page + off + ret, "DSP: %ldHz\n", tangox_get_dspclock());
	*eof = 1;
	return ret;
}

static int profile_info_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char buf[16];

	if (count > ARRAY_SIZE(buf) - 1)
		count = ARRAY_SIZE(buf) - 1;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = '\0';

	change_profile(buf);

	return count;
}

static int profile_info_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int ret = 0;
	int i;

	for (i = 0; i < (sizeof(freq_table)/sizeof(struct freq_profile)); i++) {
		if (freq_table[i].pll3 == 0)
			continue;
		ret += sprintf(page + off + ret, "%c %d: %s\n", (profile_idx == i) ? '*' : ' ', 
				i, freq_table[i].desc);
	}
	*eof = 1;
	return ret;
}

static int mux_info_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char buf[16];

	if (count > ARRAY_SIZE(buf) - 1)
		count = ARRAY_SIZE(buf) - 1;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = '\0';

	change_mux(buf);

	return count;
}

static int mux_info_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int ret = 0, i;
	unsigned long mux = (gbus_read_reg32(REG_BASE_system_block+SYS_sysclk_mux) >> 8) & 0xf;
	unsigned long premux = gbus_read_reg32(REG_BASE_system_block+SYS_sysclk_premux);
	
	ret = sprintf(page + off, "PLL%ld: %ldHz\n", premux & 3, tangox_get_pllclock(premux & 3));
	for (i = 0; i < 12; i++) {
		ret += sprintf(page + off + ret, "%c %d (SYS/CPU/DSP: %d/%d/%d)\n",
				(i == mux) ? '*' : ' ', i,
				ratios_table[i][0], ratios_table[i][1], ratios_table[i][2]);
	}
	*eof = 1;
	return ret;
}

#ifdef WITH_INPUT_POLLING
static int standby_info_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char buf[16];
	unsigned long elapse, ret_key;
	unsigned long *ir_table, *fip_table;
	int eth_enable, ir_enable, fip_enable, input_ctrl = 0;

	if (count > ARRAY_SIZE(buf) - 1)
		count = ARRAY_SIZE(buf) - 1;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (standby_mode.pll3 == 0) { /* not yet setting up */
		printk("standby mode parameters not set up yet.\n");
		return -EINVAL;
	}

	buf[count] = '\0';

#ifdef WITH_IR
	ir_table = (ir_key_cnt ? ir_keys : NULL);
	ir_enable = (tangox_ir_enabled() ? ((ir_table != NULL) ? 1 : 0) : 0);
#else
	ir_table = NULL;
	ir_enabled = 0;
#endif
#ifdef WITH_FIP
	fip_table = (fip_key_cnt ? fip_keys : NULL);
	fip_enable = (tangox_fip_enabled() ? ((fip_table != NULL) ? 1 : 0) : 0);
#else
	fip_table = NULL;
	fip_enable = 0;
#endif
#ifdef WITH_ETH
	eth_enable = (tangox_ethernet_enabled() ? 1 : 0);
#else
	eth_enable = 0;
#endif
	
	input_ctrl |= (ir_enable ? 1 : 0);
	input_ctrl |= (fip_enable ? 2 : 0);
	input_ctrl |= (eth_enable ? 4 : 0);

	if (strncmp("standby", buf, strlen("standby")) == 0) {
		if ((ir_enable == 0) && (fip_enable == 0) && (eth_enable == 0)) {
			printk("No wake up device specified.\n");
			goto out;
		}

		printk("Entering standby mode ...\n");
		set_freq_profile(&standby_mode, &elapse, &ret_key, ir_table, fip_table, input_ctrl, 
				&tangox_cycle_cnt.cycle32[0], &tangox_cycle_cnt.cycle32[1]);
//		jiffies_64 += ((elapse * HZ) / 1000); /* elaspe in msec so we adjust it accordingly */ 
		tangox_do_timer((elapse * HZ) / 1000);
		printk("Returned key 0x%08lx, time elapse %ld jiffies (%ld.%ld sec).\n", ret_key, (elapse * HZ) / 1000, elapse / 1000, elapse % 1000);
	//	recalc_frequency();
	}

out:
	return count;
}

#if defined(WITH_IR) || defined(WITH_FIP)
static const char *hex2string(unsigned long *hex, const char *src_ptr)
{
	unsigned long res = 0;
	const char *src = src_ptr;

	for (; isxdigit(*src); src++) {
		if (isdigit(*src))
			res = res * 16 + ((*src) - '0');
		else
			res = res * 16 + (__toupper(*src) - 'A' + 10);
	}
	*hex = res;
	return(src);
}

static const char *dec2string(unsigned long *dec, const char *src_ptr)
{
	unsigned long res = 0;
	const char *src = src_ptr;

	for (; isdigit(*src); src++) {
		res = res * 10 + ((*src) - '0');
	}
	*dec = res;
	return(src);
}
#endif

#ifdef WITH_IR
static int ir_info_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int i;
	unsigned long cnt;
	const char *ptr = buffer, *end = buffer + count;

	memset(ir_keys, 0, INPUT_TABLE_SIZE);
	ir_key_cnt = 0;

	/* Parse the values */
	for (cnt = i = 0; (i < INPUT_TABLE_SIZE/sizeof(unsigned long)) && (cnt < count); i++) {
		for (; (!isdigit(*ptr)) && (ptr < end); ptr++);  /* skip leading unwanted stuff */
		if (ptr >= end)
			break; /* no more */
		else if ((*ptr == '0') && (__toupper(*(ptr + 1)) == 'X')) 
			ptr = hex2string(&ir_keys[i], ptr + 2);
		else
			ptr = dec2string(&ir_keys[i], ptr);
	}
	ir_key_cnt = i;
	return(count);
}

static int ir_info_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int i, ret = 0;
	for (i = 0; i < INPUT_TABLE_SIZE/sizeof(unsigned long); i++) {
		if (ir_keys[i]) 
			ret += sprintf(page + off + ret, "0x%lx ", ir_keys[i]);
	}
	if (ret)
		ret += sprintf(page + off + ret, "\n");
	else	
		ret += sprintf(page + off + ret, "No IR key defined.\n");
	*eof = 1;
	return(ret);
}
#endif
#ifdef WITH_FIP
static int fip_info_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int i;
	unsigned long cnt;
	const char *ptr = buffer, *end = buffer + count;

	memset(fip_keys, 0, INPUT_TABLE_SIZE);
	fip_key_cnt = 0;

	/* Parse the values */
	for (cnt = i = 0; (i < INPUT_TABLE_SIZE/sizeof(unsigned long)) && (cnt < count); i++) {
		for (; (!isdigit(*ptr)) && (ptr < end); ptr++);  /* skip leading unwanted stuff */
		if (ptr >= end)
			break; /* no more */
		else if ((*ptr == '0') && (__toupper(*(ptr + 1)) == 'X')) 
			ptr = hex2string(&fip_keys[i], ptr + 2);
		else
			ptr = dec2string(&fip_keys[i], ptr);
	}
	fip_key_cnt = i;
	return(count);
}

static int fip_info_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int i, ret = 0;
	for (i = 0; i < INPUT_TABLE_SIZE/sizeof(unsigned long); i++) {
		if (fip_keys[i]) 
			ret += sprintf(page + off + ret, "0x%lx ", fip_keys[i]);
	}
	if (ret)
		ret += sprintf(page + off + ret, "\n");
	else	
		ret += sprintf(page + off + ret, "No FIP key defined.\n");
	*eof = 1;
	return(ret);
}
#endif
#endif

static int tangox_freq_ctrl_init(void) 
{
	int i;

#ifdef WITH_INPUT_POLLING
	memset(ir_keys, 0, INPUT_TABLE_SIZE);
	memset(fip_keys, 0, INPUT_TABLE_SIZE);
	ir_key_cnt = fip_key_cnt = 0;
#endif

	freq_table[0].pll3 = gbus_read_reg32(REG_BASE_system_block+SYS_clkgen3_pll);
	freq_table[0].pll2 = gbus_read_reg32(REG_BASE_system_block+SYS_clkgen2_pll);
	freq_table[0].pll1 = gbus_read_reg32(REG_BASE_system_block+SYS_clkgen1_pll);
	freq_table[0].pll0 = gbus_read_reg32(REG_BASE_system_block+SYS_clkgen0_pll);
	freq_table[0].mux = gbus_read_reg32(REG_BASE_system_block+SYS_sysclk_mux);
	freq_table[0].d0cfg = gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_cfg);
	freq_table[0].d0delays = gbus_read_reg32(REG_BASE_dram_controller_0+DRAM_dunit_delay0_ctrl);
	freq_table[0].d1cfg = gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_cfg);
	freq_table[0].d1delays = gbus_read_reg32(REG_BASE_dram_controller_1+DRAM_dunit_delay0_ctrl);
	freq_table[0].gpio_dir = freq_table[0].gpio_data = 0;

	tangox_get_ps_config(&freq_table[1].pll3, &freq_table[1].pll2, &freq_table[1].pll1, 
			&freq_table[1].pll0, &freq_table[1].mux, &freq_table[1].d0cfg, 
			&freq_table[1].d0delays, &freq_table[1].d1cfg, &freq_table[1].d1delays);
	freq_table[1].gpio_dir = freq_table[1].gpio_data = 0;

#ifdef WITH_INPUT_POLLING
	tangox_get_standby_config(&standby_mode.pll3, &standby_mode.pll2, &standby_mode.pll1, 
			&standby_mode.pll0, &standby_mode.mux, &standby_mode.d0cfg, 
			&standby_mode.d0delays, &standby_mode.d1cfg, &standby_mode.d1delays, 
			&standby_mode.gpio_dir, &standby_mode.gpio_data);
#endif

	for (i = 2; i < (sizeof(freq_table)/sizeof(struct freq_profile)); i++) {
		freq_table[i].d0cfg = (freq_table[i].d0cfg & 0x0fffffff) | (freq_table[0].d0cfg & 0xf0000000);
		freq_table[i].d1cfg = (freq_table[i].d1cfg & 0x0fffffff) | (freq_table[0].d1cfg & 0xf0000000);
	}

	if ((root = proc_mkdir(PROC_ROOT, NULL)) == NULL)
		return -EINVAL;

	if ((prof_ent = create_proc_entry(PROC_PROFILE, S_IRUGO | S_IWUGO, root)) == NULL) 
		return -EINVAL;
	else {
		prof_ent->read_proc = profile_info_read_proc;
		prof_ent->write_proc = profile_info_write_proc;
	}

	if ((mux_ent = create_proc_entry(PROC_MUX, S_IRUGO | S_IWUGO, root)) == NULL) 
		return -EINVAL;
	else {
		mux_ent->read_proc = mux_info_read_proc;
		mux_ent->write_proc = mux_info_write_proc;
	}

	if ((freq_ent = create_proc_entry(PROC_FREQ, S_IRUGO, root)) == NULL) 
		return -EINVAL;
	else {
		freq_ent->read_proc = freq_info_read_proc;
	}

	if ((pll3_ent = create_proc_entry(PROC_PLL3, S_IRUGO | S_IWUGO, root)) == NULL) 
		return -EINVAL;
	else {
		pll3_ent->read_proc = pllx_info_read_proc;
		pll3_ent->write_proc = pllx_info_write_proc;
		pll3_ent->data = (void *)3;
	}

	if ((pll2_ent = create_proc_entry(PROC_PLL2, S_IRUGO | S_IWUGO, root)) == NULL) 
		return -EINVAL;
	else {
		pll2_ent->read_proc = pllx_info_read_proc;
		pll2_ent->write_proc = pllx_info_write_proc;
		pll2_ent->data = (void *)2;
	}

	if ((pll1_ent = create_proc_entry(PROC_PLL1, S_IRUGO | S_IWUGO, root)) == NULL) 
		return -EINVAL;
	else {
		pll1_ent->read_proc = pllx_info_read_proc;
		pll1_ent->write_proc = pllx_info_write_proc;
		pll1_ent->data = (void *)1;
	}

	if ((pll0_ent = create_proc_entry(PROC_PLL0, S_IRUGO | S_IWUGO, root)) == NULL) 
		return -EINVAL;
	else {
		pll0_ent->read_proc = pllx_info_read_proc;
		pll0_ent->write_proc = pllx_info_write_proc;
		pll0_ent->data = (void *)0;
	}

#ifdef WITH_INPUT_POLLING
	if ((standby_ent = create_proc_entry(PROC_STANDBY, S_IWUGO, root)) == NULL) 
		return -EINVAL;
	else 
		standby_ent->write_proc = standby_info_write_proc;
#ifdef WITH_IR
	if (tangox_ir_enabled()) {
		if ((ir_ent = create_proc_entry(PROC_IR, S_IRUGO | S_IWUGO, root)) == NULL) 
			return -EINVAL;
		else {	
			ir_ent->write_proc = ir_info_write_proc;
			ir_ent->read_proc = ir_info_read_proc;
		}
	}
#endif
#ifdef WITH_FIP
	if (tangox_fip_enabled()) {
		if ((fip_ent = create_proc_entry(PROC_FIP, S_IRUGO | S_IWUGO, root)) == NULL) 
			return -EINVAL;
		else {
			fip_ent->write_proc = fip_info_write_proc;
			fip_ent->read_proc = fip_info_read_proc;
		}
	}
#endif
#endif

	printk("Tango2 frequency scaling module installed");
#ifdef WITH_INPUT_POLLING
	printk(", (standby mode supported)");
#endif
	printk(".\n");
	return 0;
}

static void tangox_freq_ctrl_exit(void) /* Removes module from kernel */
{
	if (root) {
		if (prof_ent)
			remove_proc_entry(PROC_PROFILE, root);
		if (mux_ent)
			remove_proc_entry(PROC_MUX, root);
		if (pll3_ent)
			remove_proc_entry(PROC_PLL3, root);
		if (pll2_ent)
			remove_proc_entry(PROC_PLL2, root);
		if (pll1_ent)
			remove_proc_entry(PROC_PLL1, root);
		if (pll0_ent)
			remove_proc_entry(PROC_PLL0, root);
		if (freq_ent)
			remove_proc_entry(PROC_FREQ, root);
#ifdef WITH_INPUT_POLLING
		if (standby_ent)
			remove_proc_entry(PROC_STANDBY, root);
#ifdef WITH_IR
		if (ir_ent)
			remove_proc_entry(PROC_IR, root);
#endif
#ifdef WITH_FIP
		if (fip_ent)
			remove_proc_entry(PROC_FIP, root);
#endif
#endif
		remove_proc_entry(PROC_ROOT, NULL);
	}
	printk("tangox: frequency scaling module unloaded.\n");
}

module_init(tangox_freq_ctrl_init);
module_exit(tangox_freq_ctrl_exit);

MODULE_DESCRIPTION("SMP863x frequency scaling driver");
MODULE_AUTHOR("Craig Qu/YH Lin");
MODULE_LICENSE("GPL");

