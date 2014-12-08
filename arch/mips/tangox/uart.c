#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <asm/io.h>

#include "irq.h"
#include "memmap.h"
#include "setup.h"
#include "uart.h"

#define DEFINE_UART(_base, _irq)					\
	{								\
		.mapbase	= _base,				\
		.irq		= _irq,					\
		.uartclk	= UART_CLOCK,				\
		.regshift	= 2,					\
		.iotype		= UPIO_AU,				\
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,	\
	}

static struct plat_serial8250_port uart_ports[] = {
	DEFINE_UART(UART0_BASE, UART0_IRQ),
	DEFINE_UART(UART1_BASE, UART1_IRQ),
	DEFINE_UART(UART2_BASE, UART2_IRQ),
	{ }
};

int __init tangox_uart_init(unsigned int port, unsigned int baud,
			    void __iomem **mmio)
{
	void __iomem *membase;

	membase = ioremap(uart_ports[port].mapbase, 0x30);
	if (!membase)
		return -EINVAL;

	writel(1, membase + UART_CLKSEL);

	if (baud) {
		unsigned int div = DIV_ROUND_CLOSEST(UART_CLOCK, 16 * baud);
		writel(div, membase + UART_CLKDIV);
	}

	if (mmio)
		*mmio = membase;
	else
		iounmap(membase);

	return 0;
}

static struct platform_device tangox_uart_device = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM,
	.dev		= {
		.platform_data		= &uart_ports,
	},
};

static int __init tangox_uart_register(void)
{
	int i;

	for (i = 0; uart_ports[i].flags; i++)
		tangox_uart_init(i, 0, NULL);

	return platform_device_register(&tangox_uart_device);
}
device_initcall(tangox_uart_register);
