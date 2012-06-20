
/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/module.h>
#include "setup.h"

int em86xx_uart0_get_gpio_mode(void)
{
	return gbus_read_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_mode) & 0x7f;
}

int em86xx_uart0_set_gpio_mode(int mode)
{
	int old_mode = em86xx_uart0_get_gpio_mode();
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_mode, 0x7f00 | (mode & 0x7f));
	return(old_mode);
}

int em86xx_uart0_gpio_read(int gpio)
{
	return (gbus_read_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_data) >> gpio) & 1;
}

void em86xx_uart0_gpio_write(int gpio, int data)
{
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_data,
			data ? UART_GPIO_DATA_SET(gpio) : UART_GPIO_DATA_CLEAR(gpio));
}

void em86xx_uart0_gpio_setdirection(int gpio, int dir)
{
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_dir,
			dir ? UART_GPIO_DIR_OUTPUT(gpio) : UART_GPIO_DIR_INPUT(gpio));
}

int em86xx_uart1_get_gpio_mode(void)
{
	return gbus_read_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_mode) & 0x7f;
}

int em86xx_uart1_set_gpio_mode(int mode)
{
	int old_mode = em86xx_uart1_get_gpio_mode();
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_mode, 0x7f00 | (mode & 0x7f));
	return(old_mode);
}

int em86xx_uart1_gpio_read(int gpio)
{
	return (gbus_read_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_data) >> gpio) & 1;
}

void em86xx_uart1_gpio_write(int gpio, int data)
{
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_data,
			data ? UART_GPIO_DATA_SET(gpio) : UART_GPIO_DATA_CLEAR(gpio));
}

void em86xx_uart1_gpio_setdirection(int gpio, int dir)
{
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_dir,
			dir ? UART_GPIO_DIR_OUTPUT(gpio) : UART_GPIO_DIR_INPUT(gpio));
}

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
int em86xx_uart2_get_gpio_mode(void)
{
	return gbus_read_reg32(REG_BASE_cpu_block + CPU_uart2_gpio_mode) & 0x7f;
}

int em86xx_uart2_set_gpio_mode(int mode)
{
	int old_mode = em86xx_uart2_get_gpio_mode();
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart2_gpio_mode, 0x7f00 | (mode & 0x7f));
	return(old_mode);
}

int em86xx_uart2_gpio_read(int gpio)
{
	return (gbus_read_reg32(REG_BASE_cpu_block + CPU_uart2_gpio_data) >> gpio) & 1;
}

void em86xx_uart2_gpio_write(int gpio, int data)
{
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart2_gpio_data,
			data ? UART_GPIO_DATA_SET(gpio) : UART_GPIO_DATA_CLEAR(gpio));
}

void em86xx_uart2_gpio_setdirection(int gpio, int dir)
{
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart2_gpio_dir,
			dir ? UART_GPIO_DIR_OUTPUT(gpio) : UART_GPIO_DIR_INPUT(gpio));
}
#endif

EXPORT_SYMBOL(em86xx_uart0_gpio_read);
EXPORT_SYMBOL(em86xx_uart0_gpio_write);
EXPORT_SYMBOL(em86xx_uart0_gpio_setdirection);
EXPORT_SYMBOL(em86xx_uart1_gpio_read);
EXPORT_SYMBOL(em86xx_uart1_gpio_write);
EXPORT_SYMBOL(em86xx_uart1_gpio_setdirection);
#ifdef CONFIG_TANGO3
EXPORT_SYMBOL(em86xx_uart2_gpio_read);
EXPORT_SYMBOL(em86xx_uart2_gpio_write);
EXPORT_SYMBOL(em86xx_uart2_gpio_setdirection);
#endif

#ifdef CONFIG_TANGO2
/* For Tango2, the GPIO pins are
 * 	0 .. 15: System GPIO
 *      16 .. 32: Secondary system GPIO
 * 	33 .. 34: TDMX GPIO0/1 
 * 	35 .. 41: UART0 GPIO: RXD, CTS, DSR, DCD, TXD, RTS, DTR
 *	42 .. 48: UART1 GPIO: RXD, CTS, DSR, DCD, TXD, RTS, DTR
 *      49 .. 51: SCARD CTL0, CTL1, CTL2
 *	52 .. 69: ETH GPIO: TXCLK, TXEN, TXD0, TXD1, TXD2, TXD3
 *			RXCLK, RXDV, RXER, RXD0, RXD1, RXD2, RXD3,
 *			CRS, COL, MDC, MDIO, MDINT#
 */

int em86xx_gpio_read(int gpio)
{
	if ((gpio >= 0) && (gpio < 16))
		return (gbus_read_reg32(REG_BASE_system_block +
				   SYS_gpio_data) >> gpio) & 1;
	else if ((gpio >= 16) && (gpio < 33))
		return (gbus_read_reg32(REG_BASE_host_interface +
				   ETH_gpio_data2) >> (gpio - 16)) & 1;
	else
		return -EIO; /* not implemented yet */
}

void em86xx_gpio_write(int gpio, int data)
{
	if ((gpio >= 0) && (gpio < 16))
		gbus_write_reg32(REG_BASE_system_block + SYS_gpio_data, data ?
			    GPIO_DATA_SET(gpio) : GPIO_DATA_CLEAR(gpio));
	else if ((gpio >= 16) && (gpio < 33))
		gbus_write_reg32(REG_BASE_host_interface + ETH_gpio_data2, data ?
			    GPIO_DATA_SET(gpio - 16) : GPIO_DATA_CLEAR(gpio - 16));
	else
		return; /* not implemented yet */
}

void em86xx_gpio_setdirection(int gpio, int dir)
{
	if ((gpio >= 0) && (gpio < 16))
		gbus_write_reg32(REG_BASE_system_block + SYS_gpio_dir, dir ?
			    GPIO_DIR_OUTPUT(gpio) : GPIO_DIR_INPUT(gpio));
	else if ((gpio >= 16) && (gpio < 33))
		gbus_write_reg32(REG_BASE_host_interface + ETH_gpio_dir2, dir ?
			    GPIO_DIR_OUTPUT(gpio - 16) : GPIO_DIR_INPUT(gpio - 16));
	else
		return; /* not implemented yet */
}

#elif defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)

/* For Tango3, the GPIO pins are
 * 	0 .. 15: System GPIO
 * 	16 .. 17: TDMX GPIO0/1 (only 864x)
 * 	18 .. 24: UART0 GPIO: RXD, CTS, DSR, DCD, TXD, RTS, DTR
 *      25:       SPI_CLK (only 8652/867X/868X)
 *	26 .. 32: UART1 GPIO: RXD, CTS, DSR, DCD, TXD, RTS, DTR
 *	33 .. 51: ETH0 GPIO: TXCLK, TXEN, TXD0, TXD1, TXD2, TXD3
 *			RXCLK, RXDV, RXER, RXD0, RXD1, RXD2, RXD3,
 *			CRS, COL, MDC, MDIO, MDINT#, TXER
 *	52 .. 70: ETH1 GPIO: TXCLK, TXEN, TXD0, TXD1, TXD2, TXD3
 *			RXCLK, RXDV, RXER, RXD0, RXD1, RXD2, RXD3,
 *			CRS, COL, MDC, MDIO, MDINT#, TXER
 *	71 .. 77: SCARD0 GPIO: (not 8652/867X/868X)
 *			RST, CLK, FCB, IO, CTL0, CTL1, CTL2
 *	78 .. 84: SCARD1 GPIO: (not 8652/867X/868X)
 *			RST, CLK, FCB, IO, CTL0, CTL1, CTL2
 *      85 .. 86: SCARD0 IO2, SCARD1 IO2 (only 8656)
 *	87 .. 93: UART2 GPIO: RXD, CTS, DSR, DCD, TXD, RTS, DTR (only RX/TX 8652/867X/868X)
 *
 * For Tango4, the GPIO pins are
 * 	0 .. 15: System GPIO
 * 	18 .. 25: CPU_GPIO0..7
 *	26 .. 32: UART1 GPIO: RXD, CTS, DSR, DCD, TXD, RTS, DTR
 *	33 .. 51: ETH0 GPIO: TXCLK, TXEN, TXD0, TXD1, TXD2, TXD3
 *			RXCLK, RXDV, RXER, RXD0, RXD1, RXD2, RXD3,
 *			CRS, COL, MDC, MDIO, MDINT#, TXER
 *	52 .. 70: ETH1 GPIO: TXCLK, TXEN, TXD0, TXD1, TXD2, TXD3
 *			RXCLK, RXDV, RXER, RXD0, RXD1, RXD2, RXD3,
 *			CRS, COL, MDC, MDIO, MDINT#, TXER
 *	71 .. 77: SCARD0 GPIO
 *			RST, CLK, FCB, IO, CTL0, CTL1, CTL2
 *	78 .. 84: SCARD1 GPIO
 *			RST, CLK, FCB, IO, CTL0, CTL1, CTL2
 *      85 .. 86: SCARD0 IO2, SCARD1 IO2
 */

#define M_GPIO	6
unsigned long tangox_chip_id(void);

int em86xx_gpio_read(int gpio)
{
	if ((gpio >= 0) && (gpio < 16)) {
		return (gbus_read_reg32(REG_BASE_system_block + SYS_gpio_data) >> gpio) & 1;
	} else if ((gpio == 16) || (gpio == 17)) { /* TDMX_GPIO0/1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id != 0x8644) && (chip_id != 0x8642))
			return -EINVAL;
		return -EIO; /* not yet implemented */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id & 0xff00) == 0x8900)
			return -EIO; /* not yet implemented */
		else
			return em86xx_uart0_gpio_read(gpio - 18);
	} else if (gpio == 25) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EIO; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8670)
			return -EIO; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8680)
			return -EIO; /* not yet implemented */
		else if ((chip_id & 0xff00) == 0x8900)
			return -EIO; /* not yet implemented */
	} else if ((gpio >= 26) && (gpio < 33)) { /* UART1 */
		return em86xx_uart1_gpio_read(gpio - 26);
	} else if ((gpio >= 33) && (gpio < 52)) { /* ETH0 */
		gpio -= 33;
		if (gpio < 16)
			return (gbus_read_reg32(REG_BASE_host_interface + 0x6408) >> gpio) & 1;
		else 
			return (gbus_read_reg32(REG_BASE_host_interface + 0x6410) >> (gpio - 16)) & 1;
	} else if ((gpio >= 52) && (gpio < 71)) { /* ETH1 */
		gpio -= 52;
		if (gpio < 16)
			return (gbus_read_reg32(REG_BASE_host_interface + 0x6c08) >> gpio) & 1;
		else 
			return (gbus_read_reg32(REG_BASE_host_interface + 0x6c10) >> (gpio - 16)) & 1;
	} else if ((gpio >= 71) && (gpio < 78)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8670)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8680)
			return -EINVAL;
		return (gbus_read_reg32(REG_BASE_cpu_block + 0xc35c) >> (gpio - 71)) & 1;
	} else if ((gpio >= 78) && (gpio < 85)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8670)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8680)
			return -EINVAL;
		return (gbus_read_reg32(REG_BASE_cpu_block + 0xc3dc) >> (gpio - 78)) & 1;
	} else if ((gpio == 85) || (gpio == 86)) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id == 0x8656) || ((chip_id & 0xff00) == 0x8900))
			return -EIO; /* not yet implemented */
	} else if ((gpio >= 87) || (gpio < 94)) { /* UART2 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return em86xx_uart2_gpio_read(gpio - 87);
		else if ((chip_id & 0xfff0) == 0x8670)
			return em86xx_uart2_gpio_read(gpio - 87);
		else if ((chip_id & 0xfff0) == 0x8680)
			return em86xx_uart2_gpio_read(gpio - 87);
	}
	return -EINVAL;
}

void em86xx_gpio_write(int gpio, int val)
{
	int data = (val != 0) ? 1 : 0;
	if ((gpio >= 0) && (gpio < 16)) {
		gbus_write_reg32(REG_BASE_system_block + SYS_gpio_data, data ?  GPIO_DATA_SET(gpio) : GPIO_DATA_CLEAR(gpio));
	} else if ((gpio == 16) || (gpio == 17)) { /* TDMX_GPIO0/1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id != 0x8644) && (chip_id != 0x8642))
			return;
		return; /* not yet implemented */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id & 0xff00) == 0x8900)
			return; /* not yet implemented */
		else
			em86xx_uart0_gpio_write(gpio - 18, data);
	} else if (gpio == 25) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8670)
			return; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8680)
			return; /* not yet implemented */
		else if ((chip_id & 0xff00) == 0x8900)
			return; /* not yet implemented */
	} else if ((gpio >= 26) && (gpio < 33)) { /* UART1 */
		em86xx_uart1_gpio_write(gpio - 26, data);
	} else if ((gpio >= 33) && (gpio < 52)) { /* ETH0 */
		gpio -= 33;
		if (gpio < 16)
			gbus_write_reg32(REG_BASE_host_interface + 0x6408, (1 << (gpio + 16)) | (data << gpio));
		else 
			gbus_write_reg32(REG_BASE_host_interface + 0x6410, (1 << gpio) | (data << (gpio - 16)));
	} else if ((gpio >= 52) && (gpio < 71)) { /* ETH1 */
		gpio -= 52;
		if (gpio < 16)
			gbus_write_reg32(REG_BASE_host_interface + 0x6c08, (1 << (gpio + 16)) | (data << gpio));
		else 
			gbus_write_reg32(REG_BASE_host_interface + 0x6c10, (1 << gpio) | (data << (gpio - 16)));
	} else if ((gpio >= 71) && (gpio < 78)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return;
		else if ((chip_id & 0xfff0) == 0x8670)
			return;
		else if ((chip_id & 0xfff0) == 0x8680)
			return;
		gpio -= 71;
		gbus_write_reg32(REG_BASE_cpu_block + 0xc35c, (1 << (gpio + 8)) | (data << gpio));
	} else if ((gpio >= 78) && (gpio < 85)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return;
		else if ((chip_id & 0xfff0) == 0x8670)
			return;
		else if ((chip_id & 0xfff0) == 0x8680)
			return;
		gpio -= 78;
		gbus_write_reg32(REG_BASE_cpu_block + 0xc3dc, (1 << (gpio + 8)) | (data << gpio));
	} else if ((gpio == 85) || (gpio == 86)) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id == 0x8656) || ((chip_id & 0xff00) == 0x8900))
			return; /* not yet implemented */
	} else if ((gpio >= 87) || (gpio < 94)) { /* UART2 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			em86xx_uart2_gpio_write(gpio - 87, data);
		else if ((chip_id & 0xfff0) == 0x8670)
			em86xx_uart2_gpio_write(gpio - 87, data);
		else if ((chip_id & 0xfff0) == 0x8680)
			em86xx_uart2_gpio_write(gpio - 87, data);
	}
}

void em86xx_gpio_setdirection(int gpio, int direction)
{
	int dir = (direction != 0) ? 1 : 0;
	if ((gpio >= 0) && (gpio < 16)) {
		gbus_write_reg32(REG_BASE_system_block + SYS_gpio_dir, dir ?  GPIO_DIR_OUTPUT(gpio) : GPIO_DIR_INPUT(gpio));
	} else if ((gpio == 16) || (gpio == 17)) { /* TDMX_GPIO0/1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id != 0x8644) && (chip_id != 0x8642))
			return;
		return; /* not yet implemented */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id & 0xff00) == 0x8900)
			return; /* not yet implemented */
		else
			em86xx_uart0_gpio_setdirection(gpio - 18, dir);
	} else if (gpio == 25) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8670)
			return; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8680)
			return; /* not yet implemented */
		else if ((chip_id & 0xff00) == 0x8900)
			return; /* not yet implemented */
	} else if ((gpio >= 26) && (gpio < 33)) { /* UART1 */
		em86xx_uart1_gpio_setdirection(gpio - 26, dir);
	} else if ((gpio >= 33) && (gpio < 52)) { /* ETH0 */
		gpio -= 33;
		if (gpio < 16)
			gbus_write_reg32(REG_BASE_host_interface + 0x6404, (1 << (gpio + 16)) | (dir << gpio));
		else 
			gbus_write_reg32(REG_BASE_host_interface + 0x640c, (1 << gpio) | (dir << (gpio - 16)));
	} else if ((gpio >= 52) && (gpio < 71)) { /* ETH1 */
		gpio -= 52;
		if (gpio < 16)
			gbus_write_reg32(REG_BASE_host_interface + 0x6c04, (1 << (gpio + 16)) | (dir << gpio));
		else 
			gbus_write_reg32(REG_BASE_host_interface + 0x6c0c, (1 << gpio) | (dir << (gpio - 16)));
	} else if ((gpio >= 71) && (gpio < 78)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return;
		else if ((chip_id & 0xfff0) == 0x8670)
			return;
		else if ((chip_id & 0xfff0) == 0x8680)
			return;
		gpio -= 71;
		gbus_write_reg32(REG_BASE_cpu_block + 0xc358, (1 << (gpio + 8)) | (dir << gpio));
	} else if ((gpio >= 78) && (gpio < 85)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return;
		else if ((chip_id & 0xfff0) == 0x8670)
			return;
		else if ((chip_id & 0xfff0) == 0x8680)
			return;
		gpio -= 78;
		gbus_write_reg32(REG_BASE_cpu_block + 0xc3d8, (1 << (gpio + 8)) | (dir << gpio));
	} else if ((gpio == 85) || (gpio == 86)) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id == 0x8656) || ((chip_id & 0xff00) == 0x8900))
			return; /* not yet implemented */
	} else if ((gpio >= 87) || (gpio < 94)) { /* UART2 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			em86xx_uart2_gpio_setdirection(gpio - 87, dir);
		else if ((chip_id & 0xfff0) == 0x8670)
			em86xx_uart2_gpio_setdirection(gpio - 87, dir);
		else if ((chip_id & 0xfff0) == 0x8680)
			em86xx_uart2_gpio_setdirection(gpio - 87, dir);
	}
}

int em86xx_gpio_getmode(int gpio)
{
	if ((gpio >= 0) && (gpio < 16)) {
		return 1; /* always in GPIO mode */
	} else if ((gpio == 16) || (gpio == 17)) { /* TDMX_GPIO0/1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id != 0x8644) && (chip_id != 0x8642))
			return -EINVAL;
		return -EIO; /* not yet implemented */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id & 0xff00) == 0x8900)
			return -EIO; /* not yet implemented */
		return ((em86xx_uart0_get_gpio_mode() >> (gpio - 18)) & 1) ? 1 : 0;
	} else if (gpio == 25) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EIO; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8670)
			return -EIO; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8680)
			return -EIO; /* not yet implemented */
		else if ((chip_id & 0xff00) == 0x8900)
			return -EIO; /* not yet implemented */
	} else if ((gpio >= 26) && (gpio < 33)) { /* UART1 */
		return ((em86xx_uart1_get_gpio_mode() >> (gpio - 26)) & 1) ? 1 : 0;
	} else if ((gpio >= 33) && (gpio < 52)) { /* ETH0 */
		unsigned int pad_mode = gbus_read_reg32(REG_BASE_host_interface + 0x6400) & 7;
		return (pad_mode == M_GPIO) ? 1 : 0;
	} else if ((gpio >= 52) && (gpio < 71)) { /* ETH1 */
		unsigned int pad_mode = gbus_read_reg32(REG_BASE_host_interface + 0x6c00) & 7;
		return (pad_mode == M_GPIO) ? 1 : 0;
	} else if ((gpio >= 71) && (gpio < 78)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8670)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8680)
			return -EINVAL;
		return (gbus_read_reg32(REG_BASE_cpu_block + 0xc360) >> (gpio - 71)) & 1;
	} else if ((gpio >= 78) && (gpio < 85)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8670)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8680)
			return -EINVAL;
		return (gbus_read_reg32(REG_BASE_cpu_block + 0xc3e0) >> (gpio - 78)) & 1;
	} else if ((gpio == 85) || (gpio == 86)) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id == 0x8656) || ((chip_id & 0xff00) == 0x8900))
			return -EIO; /* not yet implemented */
	} else if ((gpio >= 87) || (gpio < 94)) { /* UART2 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return ((em86xx_uart2_get_gpio_mode() >> (gpio - 87)) & 1) ? 1 : 0;
		else if ((chip_id & 0xfff0) == 0x8670)
			return ((em86xx_uart2_get_gpio_mode() >> (gpio - 87)) & 1) ? 1 : 0;
		else if ((chip_id & 0xfff0) == 0x8680)
			return ((em86xx_uart2_get_gpio_mode() >> (gpio - 87)) & 1) ? 1 : 0;
	}
	return -EINVAL;
}

int em86xx_gpio_setmode(int gpio, int mode, int *oldmode)
{
	int newmode = (mode != 0) ? 1 : 0;
	if ((gpio >= 0) && (gpio < 16)) {
		*oldmode = 1;
		if (newmode == 0)
			return -EINVAL; 
	} else if ((gpio == 16) || (gpio == 17)) { /* TDMX_GPIO0/1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id != 0x8644) && (chip_id != 0x8642))
			return -EINVAL;
		return -EIO; /* not yet implemented */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id & 0xff00) == 0x8900)
			return -EIO; /* not yet implemented */
		else {
			gpio -= 18;
			*oldmode = (gbus_read_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_mode) >> gpio) & 1;
			gbus_write_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_mode, (1 << (gpio + 8)) | (newmode << gpio));
		}
	} else if (gpio == 25) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EIO; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8670)
			return -EIO; /* not yet implemented */
		else if ((chip_id & 0xfff0) == 0x8680)
			return -EIO; /* not yet implemented */
		else if ((chip_id & 0xff00) == 0x8900)
			return -EIO; /* not yet implemented */
	} else if ((gpio >= 26) && (gpio < 33)) { /* UART1 */
		gpio -= 26;
		*oldmode = (gbus_read_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_mode) >> gpio) & 1;
		gbus_write_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_mode, (1 << (gpio + 8)) | (newmode << gpio));
	} else if ((gpio >= 33) && (gpio < 52)) { /* ETH0 */
		unsigned int pad_mode = gbus_read_reg32(REG_BASE_host_interface + 0x6400) & 7;
		if ((pad_mode != M_GPIO) && (newmode != 0)) {
			*oldmode = 0;
			gbus_write_reg32(REG_BASE_host_interface + 0x6400, (gbus_read_reg32(REG_BASE_host_interface + 0x6400) & ~7) | M_GPIO);
		}
	} else if ((gpio >= 52) && (gpio < 71)) { /* ETH1 */
		unsigned int pad_mode = gbus_read_reg32(REG_BASE_host_interface + 0x6c00) & 7;
		if ((pad_mode != M_GPIO) && (newmode != 0)) {
			*oldmode = 0;
			gbus_write_reg32(REG_BASE_host_interface + 0x6c00, (gbus_read_reg32(REG_BASE_host_interface + 0x6c00) & ~7) | M_GPIO);
		}
	} else if ((gpio >= 71) && (gpio < 78)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id == 0x8656) || ((chip_id & 0xff00) == 0x8900))
			return -EINVAL;
		gpio -= 71;
		*oldmode = (gbus_read_reg32(REG_BASE_cpu_block + 0xc360) >> gpio) & 1;
		gbus_write_reg32(REG_BASE_cpu_block + 0xc360, (1 << (gpio + 8)) | (newmode << gpio));
	} else if ((gpio >= 78) && (gpio < 85)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8670)
			return -EINVAL;
		else if ((chip_id & 0xfff0) == 0x8680)
			return -EINVAL;
		gpio -= 78;
		*oldmode = (gbus_read_reg32(REG_BASE_cpu_block + 0xc3e0) >> gpio) & 1;
		gbus_write_reg32(REG_BASE_cpu_block + 0xc3e0, (1 << (gpio + 8)) | (newmode << gpio));
	} else if ((gpio == 85) || (gpio == 86)) {
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id == 0x8656) || ((chip_id & 0xff00) == 0x8900))
			return -EIO; /* not yet implemented */
	} else if ((gpio >= 87) || (gpio < 94)) { /* UART2 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if ((chip_id == 0x8652) || ((chip_id & 0xfff0) == 0x8670) || ((chip_id & 0xfff0) == 0x8680)) {
			gpio -= 87;
			*oldmode = (gbus_read_reg32(REG_BASE_cpu_block + CPU_uart2_gpio_mode) >> gpio) & 1;
			gbus_write_reg32(REG_BASE_cpu_block + CPU_uart2_gpio_mode, (1 << (gpio + 8)) | (newmode << gpio));
		}
	} else
		return -EINVAL;
	return 0;
}
#else
#error Undefined platform.
#endif

EXPORT_SYMBOL(em86xx_gpio_read);
EXPORT_SYMBOL(em86xx_gpio_write);
EXPORT_SYMBOL(em86xx_gpio_setdirection);

#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
EXPORT_SYMBOL(em86xx_gpio_getmode);
EXPORT_SYMBOL(em86xx_gpio_setmode);
#endif

