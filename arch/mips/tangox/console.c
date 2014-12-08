#include <linux/init.h>
#include <asm/io.h>

#include "setup.h"
#include "uart.h"

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
	int port = tangox_uart_console_port();
	int baud = tangox_uart_baudrate(port);

	tangox_uart_init(port, baud, &uart_base);

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
