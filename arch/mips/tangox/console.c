#include <linux/init.h>
#include <asm/io.h>

#include "memmap.h"
#include "setup.h"
#include "uart.h"

static const unsigned long uart_addr[] __initconst = {
	UART0_BASE,
	UART1_BASE,
	UART2_BASE,
};

static void __iomem *uart_base;

static inline unsigned serial_in(int reg)
{
	return readl(uart_base + reg);
}

static inline void serial_out(int reg, unsigned value)
{
	writel(value, uart_base + reg);
}

void __init prom_console_init(void)
{
	int port = CONFIG_TANGOX_EARLY_CONSOLE_PORT;
	int baud = CONFIG_TANGOX_EARLY_CONSOLE_RATE;
	unsigned int div;

	uart_base = ioremap(uart_addr[port], 0x30);
	if (!uart_base)
		return;

	div = DIV_ROUND_CLOSEST(UART_CLOCK, 16 * baud);

	serial_out(UART_CLKSEL, 1);
	serial_out(UART_CLKDIV, div);

	serial_out(UART_IER, 0x0);
	serial_out(UART_FCR, 0);
	serial_out(UART_LCR, 0x3);
}

void prom_putchar(char c)
{
	unsigned int timeout;
	int status, bits;

	if (!uart_base)
		return;

	timeout = 0;
	bits = 0x60;

	do {
		status = serial_in(UART_LSR);

		if (--timeout == 0)
			break;
	} while ((status & bits) != bits);

	if (timeout)
		serial_out(UART_THR, c);
}
