#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/io.h>

#include "uart.h"

int __init tangox_uart_init(unsigned long addr, unsigned int baud,
			    void __iomem **mmio)
{
	void __iomem *membase;

	membase = ioremap(addr, 0x30);
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
