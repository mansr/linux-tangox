/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/config.h>

#include <asm/mach-tango2/param.h>
#include <asm/tango2/tango2.h>
#include <asm/tango2/emhwlib_registers_tango2.h>
#include <asm/tango2/emhwlib_dram_tango2.h>

#ifndef KSEG1ADDR
#define KSEG1ADDR(x)    (0xa0000000|(x))
#endif

#ifdef WITH_IR
unsigned long ir_poll_key(unsigned long *table, unsigned long size);
#endif

#ifdef WITH_FIP
unsigned long fip_poll_key(unsigned long *table, unsigned long size);
#endif

#ifdef WITH_ETH
unsigned long eth_poll_key(void);
#endif

#ifdef FC_DEBUG
void uart_puts(const char *s);
#endif

/* Mirror the one in upper directory */
struct freq_handler_header
{
	void *func_ptr;      /* Entry point of handler */
	void *param_area;       /* parameter area */
	unsigned long *ir_table;         /* Key filters for IR (standby) */
	unsigned long *fip_table;        /* Key filters for FIP (standby) */
	unsigned long ret_key;  /* Returned key */
	unsigned long elapse;   /* Time elapse in standby */
	unsigned long input_ctrl; /* Which input is enabled */
	unsigned long cnt_low;	/* xtal_cnt: low 32 */
	unsigned long cnt_high;	/* xtal_cnt: high 32 */
};

/* Prototyping */
void udelay(unsigned int usec);
unsigned long __getxtal(void);
void __sync(void);

#define fast_iob()			\
	{				\
		int i = 0; __iob(i);	\
	}

unsigned long polling_input(unsigned long *ir_table, unsigned long *fip_table, struct freq_handler_header *hdr)
{
	const unsigned long input_ctrl = hdr->input_ctrl;
	unsigned long key = 0;
	unsigned long *elapse_ptr = &hdr->elapse;
	unsigned long cnt = hdr->cnt_low;

#ifdef UPDATE_XTAL
	unsigned long elapse = 0, diff_xtal = 0;
#endif

	do {
#ifdef WITH_IR
		if ((input_ctrl & 1) != 0) {
			if ((key = ir_poll_key(ir_table, INPUT_TABLE_SIZE / 4)) != 0)
				goto done;
		}
#endif
#ifdef WITH_FIP
		if ((input_ctrl & 2) != 0) {
			if ((key = fip_poll_key(fip_table, INPUT_TABLE_SIZE / 4)) != 0)
				goto done;
		}
#endif
#ifdef WITH_ETH
		if ((input_ctrl & 4) != 0) {
			if ((key = eth_poll_key()) != 0)
				goto done;
		}
#endif
		udelay(1000);

done:
		hdr->cnt_low = __getxtal();
		if (cnt > hdr->cnt_low)
			hdr->cnt_high++;
#ifdef UPDATE_XTAL
		diff_xtal += ((cnt > hdr->cnt_low) ? 
				((0xffffffff - cnt) + hdr->cnt_low + 1) : (hdr->cnt_low - cnt));
		elapse += (diff_xtal / (TANGOX_BASE_FREQUENCY / 1000));
		diff_xtal %= (TANGOX_BASE_FREQUENCY / 1000);
#endif
		cnt = hdr->cnt_low;
	} while (key == 0);

	*elapse_ptr = elapse;
	__sync();
	return key;
}

