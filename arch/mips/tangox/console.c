
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
 * simple  uart support for  tango2/tango3 board,  register an  early console
 * that make boot problem easier to debug.
 *
 * this uart init code comes from zboot
 */

#include <linux/init.h>
#include <linux/config.h>
#include <linux/console.h>

#include "setup.h"

extern int tangox_uart_console_port(void);

#ifdef CONFIG_TANGO3
/*
 * helpers to access uart0/uart1/uart2 register
 */
#define RD_UART_REG32(r)							\
	((tangox_uart_console_port() == 0) ? 					\
		gbus_read_reg32(REG_BASE_cpu_block + CPU_UART0_base + (r)) :	\
		((tangox_uart_console_port() == 1) ? 				\
			gbus_read_reg32(REG_BASE_cpu_block + CPU_UART1_base + (r)) : 	\
			gbus_read_reg32(REG_BASE_cpu_block + CPU_UART2_base + (r))))

#define WR_UART_REG32(r, v)								\
	((tangox_uart_console_port() == 0) ? 						\
		gbus_write_reg32(REG_BASE_cpu_block + CPU_UART0_base + (r), (v)) :	\
		((tangox_uart_console_port() == 1) ? 					\
			gbus_write_reg32(REG_BASE_cpu_block + CPU_UART1_base + (r), (v)) : 	\
			gbus_write_reg32(REG_BASE_cpu_block + CPU_UART2_base + (r), (v)))) 
#else
/*
 * helpers to access uart0/uart1 register
 */
#define RD_UART_REG32(r)							\
	((tangox_uart_console_port() == 0) ? 					\
		gbus_read_reg32(REG_BASE_cpu_block + CPU_UART0_base + (r)) :	\
		gbus_read_reg32(REG_BASE_cpu_block + CPU_UART1_base + (r)))

#define WR_UART_REG32(r, v)								\
	((tangox_uart_console_port() == 0) ? 						\
		gbus_write_reg32(REG_BASE_cpu_block + CPU_UART0_base + (r), (v)) :	\
		gbus_write_reg32(REG_BASE_cpu_block + CPU_UART1_base + (r), (v)))
#endif

/*
 * print given char to uart0/uart1/uart2
 */
static void __init prom_putc(char c)
{
	/* if '\n', then print '\r' also */
	if (c == '\n') {
		prom_putc('\r');
	}

	while ((RD_UART_REG32(CPU_UART_LSR) & 0x20) == 0);
	WR_UART_REG32(CPU_UART_THR, (unsigned long)c);
	while ((RD_UART_REG32(CPU_UART_LSR) & 0x20) == 0);
}

/*
 * print given string to uart0/uart1/uart2
 */
void __init prom_puts(const char *s)
{
	while (*s)
		prom_putc(*s++);
}

/*
 * initialize uart0/uart1/uart2 with given parameters
 */
static void __init uart_init(int baud, int fifo)
{
	unsigned int div;

	WR_UART_REG32(CPU_UART_IER, 0x0);
	WR_UART_REG32(CPU_UART_FCR, (fifo ? 0x1f : 0x0));
	WR_UART_REG32(CPU_UART_LCR, 0x3);

#ifdef CONFIG_TANGOX_UART_USE_SYSCLK
	WR_UART_REG32(CPU_UART_CLKSEL, 0x0);
	div = (tangox_get_sysclock() / baud) >> 4;
	if (((((tangox_get_sysclock() * 10) / baud) >> 4) % 10) >= 5)
		div++;
	WR_UART_REG32(CPU_UART_CLKDIV, div);
#else
	WR_UART_REG32(CPU_UART_CLKSEL, 0x1);
#ifdef CONFIG_TANGO2
	div = (TANGO3_UART_FREQUENCY / baud) >> 4;
	if (((((TANGO3_UART_FREQUENCY * 10) / baud) >> 4) % 10) >= 5)
		div++;
#elif defined(CONFIG_TANGO3)
	div = (TANGO3_UART_FREQUENCY / baud) >> 4;
	if (((((TANGO3_UART_FREQUENCY * 10) / baud) >> 4) % 10) >= 5)
		div++;
#else
#error Unsupported platform.
#endif
	WR_UART_REG32(CPU_UART_CLKDIV, div);
#endif
}

/*
 * kernel console write callback
 */
static void __init prom_console_write(struct console *con, const char *s,
				      unsigned int c)
{
	prom_puts(s);
}

static struct console promcons __initdata = {
	.name   = "prom",
	.write  = prom_console_write,
	.flags  = CON_PRINTBUFFER | CON_BOOT,
	.index  = -1,
};

/*
 * init uart0/uart1/uart2 and register a console that will use our prom console
 * callbacks
 */
void __init prom_console_register(void)
{
	uart_init(tangox_uart_baudrate(tangox_uart_console_port()), 0);
	register_console(&promcons);

	/* hello world ! */
	printk(KERN_INFO "prom console registered\n");
}

