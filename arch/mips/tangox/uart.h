#ifndef __MIPS_TANGOX_UART_H
#define __MIPS_TANGOX_UART_H

#define UART_CLOCK	7372800

#define UART_RBR	0x00
#define UART_THR	0x04
#define UART_IER	0x08
#define UART_IIR	0x0c
#define UART_FCR	0x10
#define UART_LCR	0x14
#define UART_MCR	0x18
#define UART_LSR	0x1c
#define UART_MSR	0x20
#define UART_SCR	0x24
#define UART_CLKDIV	0x28
#define UART_CLKSEL	0x2c

int tangox_uart_init(unsigned long addr, unsigned int baud,
		     void __iomem **mmio);

#endif /* __MIPS_TANGOX_UART_H */
