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

#define fast_iob()			\
	{				\
		int i = 0; __iob(i);	\
	}

void __sync(void);
void __iob(int i);

unsigned long __getxtal(void)
{
	unsigned long res = *((volatile unsigned long *)KSEG1ADDR(REG_BASE_system_block + SYS_xtal_in_cnt));
	__sync();
	return res;
}

void udelay(unsigned int usec)
{
	/* SYS_xtal_in_cnt is a counter running off 27MHz, so 1 usec
           is roughly equivalent to 27 increase of count */
	unsigned long start = __getxtal();
	unsigned long end = start + (usec * (TANGOX_BASE_FREQUENCY / 1000000));

	if (end > start)
		while ((__getxtal() < end) && (__getxtal() >= start));
	else
		/* Handle overflow condition */
		while ((__getxtal() < end) || (__getxtal() >= start));
}

#ifdef FC_DEBUG

static const unsigned long cpu_uart_base = REG_BASE_cpu_block + CPU_UART0_base;

void uart_putc(const char c)
{
	if (c != '\n') {
		while (((*(volatile unsigned long *)KSEG1ADDR(cpu_uart_base+CPU_UART_LSR))& 0x20) == 0);
		*(volatile unsigned long *)KSEG1ADDR(cpu_uart_base+CPU_UART_THR) = (unsigned long)c;
		fast_iob();
		udelay(10);
	} else {
		while (((*(volatile unsigned long *)KSEG1ADDR(cpu_uart_base+CPU_UART_LSR))& 0x20) == 0);
		*(volatile unsigned long *)KSEG1ADDR(cpu_uart_base+CPU_UART_THR) = 0x0d;
		fast_iob();
		udelay(10);
		while (((*(volatile unsigned long *)KSEG1ADDR(cpu_uart_base+CPU_UART_LSR))& 0x20) == 0);
		*(volatile unsigned long *)KSEG1ADDR(cpu_uart_base+CPU_UART_THR) = 0x0a;
		fast_iob();
		udelay(10);
	}
	while (((*(volatile unsigned long *)KSEG1ADDR(cpu_uart_base+CPU_UART_LSR))& 0x20) == 0);
}

void uart_puts(const char *s)
{
	for (; s && (*s != '\0'); s++)
		uart_putc(*s);
}
#endif

