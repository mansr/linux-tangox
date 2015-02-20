/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __FREQCTRL_H__
#define __FREQCTRL_H__

#ifndef __FSCONFIG_H__
#include "fc/fcconfig.h"
#endif

/* USB related registers */
#define EHCI_CMD           (REG_BASE_host_interface+0x1410)
#define OHCI_INTR_MIE      (1<<31)
#define OHCI_INT_ENABLE    (REG_BASE_host_interface+0x1510)
#define OHCI_INT_DISABLE   (REG_BASE_host_interface+0x1514)
#define OHCI_CONTROL       (REG_BASE_host_interface+0x1504)
#define HCCONTROL_HCFS     (3<<6)
#define HCCONTROL_USB_OPER (2<<6)
#define CMD_RUN            (1<<0)

/*
 * Mac/DMA registers offset, refer to documentation
 */
#define ENET_HOST_BASE		REG_BASE_host_interface

/* mac registers */
#define ENET_MAC_BASE		(ENET_HOST_BASE + 0x6000)
#define ENET_MAC_MACCR		(ENET_MAC_BASE + 0x0)
#define MACCR_F			(1 << 20)
#define MACCR_PM		(1 << 19)
#define MACCR_PR		(1 << 18)
#define MACCR_HP		(1 << 13)
#define MACCR_ASTP		(1 << 8)
#define MACCR_TE		(1 << 3)
#define MACCR_RE		(1 << 2)
#define ENET_MAC_MACAHR		(ENET_MAC_BASE + 0x4)
#define ENET_MAC_MACALR		(ENET_MAC_BASE + 0x8)
#define ENET_MAC_MAHR		(ENET_MAC_BASE + 0xc)
#define ENET_MAC_MALR		(ENET_MAC_BASE + 0x10)
#define ENET_MAC_MIIAR		(ENET_MAC_BASE + 0x14)
#define MIIAR_ADDR(x)		((x) << 11)
#define MIIAR_REG(x)		((x) << 6)
#define MIIAR_WRITE		(1 << 1)
#define MIIAR_BUSY		(1 << 0)
#define ENET_MAC_MIIDR		(ENET_MAC_BASE + 0x18)
#define ENET_MAC_FCR		(ENET_MAC_BASE + 0x1c)
#define FCR_ENABLE		(1 << 1)

/* dma registers */
#define ENET_DMA_BASE		(ENET_HOST_BASE + 0x7000)
#define ENET_DMA_BMR		(ENET_DMA_BASE + 0x00)
#define BMR_PBL(x)		((x & 0x3f) << 8)
#define BMR_SWR			(1 << 0)
#define ENET_DMA_TPDR		(ENET_DMA_BASE + 0x04)
#define ENET_DMA_RPDR		(ENET_DMA_BASE + 0x08)
#define ENET_DMA_RBAR		(ENET_DMA_BASE + 0x0c)
#define ENET_DMA_TBAR		(ENET_DMA_BASE + 0x10)
#define ENET_DMA_SR		(ENET_DMA_BASE + 0x14)
#define SR_EB(x)		(((x) >> 23) & 0x3)
#define SR_TS(x)		(((x) >> 20) & 0x7)
#define SR_RS(x)		(((x) >> 17) & 0x7)
#define SR_NIS			(1 << 16)
#define SR_AIS			(1 << 15)
#define SR_ERI			(1 << 14)
#define SR_FBE			(1 << 13)
#define SR_ETI			(1 << 12)
#define SR_RWT			(1 << 11)
#define SR_RPS			(1 << 8)
#define SR_RU			(1 << 7)
#define SR_R			(1 << 6)
#define SR_UNF			(1 << 5)
#define SR_TU			(1 << 4)
#define SR_TPS			(1 << 1)
#define SR_T			(1 << 0)
#define ENET_DMA_CR		(ENET_DMA_BASE + 0x18)
#define CR_SF			(1 << 21)
#define CR_ST			(1 << 13)
#define CR_SR			(1 << 1)
#define ENET_DMA_IER		(ENET_DMA_BASE + 0x1c)
#define IER_NIS			(1 << 16)
#define IER_AIS			(1 << 15)
#define IER_ERE			(1 << 14)
#define IER_FBE			(1 << 13)
#define IER_ETE			(1 << 10)
#define IER_RWT			(1 << 9)
#define IER_RS			(1 << 8)
#define IER_RU			(1 << 7)
#define IER_R			(1 << 6)
#define IER_UE			(1 << 5)
#define IER_TBU			(1 << 2)
#define IER_TS			(1 << 1)
#define IER_T			(1 << 0)
#define ENET_DMA_CHRBA		(ENET_DMA_BASE + 0x54)

typedef unsigned long (*FUNC_PTR)(void);

/* Profile for frequency to be switched to */
struct freq_profile
{
	unsigned long pll3;      /* setting for PLL3 */
	unsigned long pll2;      /* setting for PLL2 */
	unsigned long pll1;      /* setting for PLL1 */
	unsigned long pll0;      /* setting for PLL0 */
	unsigned long mux;       /* sysclk_mux */
	unsigned long d0cfg;     /* DRAM0 parameters */
	unsigned long d0delays;
	unsigned long d1cfg;     /* DRAM1 parameters */
	unsigned long d1delays;
	unsigned long standby;   /* standby invocation? */
	const char *desc;        /* description */
	unsigned long gpio_dir;  /* GPIO dir (standby) */
	unsigned long gpio_data; /* GPIO data (standby) */
};

/* For communicating with handler */
struct freq_handler_header
{
	FUNC_PTR func_ptr;      /* Entry point of handler */
	struct freq_profile *param_area;       /* parameter area */
#ifdef WITH_INPUT_POLLING
	unsigned long *ir_table;         /* Key filters for IR (standby) */
	unsigned long *fip_table;        /* Key filters for FIP (standby) */
	unsigned long ret_key;  /* Returned key */
	unsigned long elapse;   /* Time elapse in standby */
	unsigned long input_ctrl; /* Which input is enabled */
	unsigned long cnt_low;	/* xtal_cnt: low 32 */
	unsigned long cnt_high;	/* xtal_cnt: high 32 */
#endif
};

/* Prototyping */
#ifdef WITH_INPUT_POLLING
int set_freq_profile(const struct freq_profile *pptr, unsigned long *elapse, unsigned long *ret_key, 
	unsigned long *ir_keys, unsigned long *fip_keys, int input_ctrl,
	unsigned long *cnt_low, unsigned long *cnt_high);
#else
int set_freq_profile(const struct freq_profile *pptr);
#endif

unsigned long frequency_to_pll(unsigned int freq, unsigned int ratio);

#endif /* __FREQCTRL_H__ */

