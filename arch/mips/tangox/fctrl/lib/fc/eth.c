/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/config.h>

#include <asm/tango2/emhwlib_registers_tango2.h>
#include <asm/tango2/emhwlib_dram_tango2.h>

#define KSEG1ADDR(x)            (0xa0000000|(x))

#define fast_iob()			\
	{				\
		int i = 0; __iob(i);	\
	}

void __iob(int dummy);

#define WOL_CTRL   0x602c

extern void uart_putc(const char c);
extern void uart_puts(const char *s);

unsigned long eth_poll_key(void)
{
	unsigned long wol_stat = *((volatile unsigned long *)KSEG1ADDR(REG_BASE_host_interface + WOL_CTRL));
	if ((wol_stat & 0x60) != 0) {
		/* clear the status */
		*((volatile unsigned long *)KSEG1ADDR(REG_BASE_host_interface + WOL_CTRL)) = 0x60;
		fast_iob();
#ifdef FC_DEBUG
		uart_puts("Got WOL packet\n");
#endif
		return(1);
	} else
		return(0);
}

