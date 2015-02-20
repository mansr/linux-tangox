
/*********************************************************************
 Copyright (C) 2001-2007
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

#ifdef CONFIG_TANGO3
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
int em86xx_gpio_read(int gpio)
{
	if ((gpio >= 0) && (gpio < 16))
		return (gbus_read_reg32(REG_BASE_system_block +
				   SYS_gpio_data) >> gpio) & 1;
	/* Upper 16 */
	return (gbus_read_reg32(REG_BASE_host_interface +
			   ETH_gpio_data2) >> (gpio - 16)) & 1;
}

void em86xx_gpio_write(int gpio, int data)
{
	if ((gpio >= 0) && (gpio < 16))
		gbus_write_reg32(REG_BASE_system_block + SYS_gpio_data, data ?
			    GPIO_DATA_SET(gpio) : GPIO_DATA_CLEAR(gpio));
	else /* Upper 16 */
		gbus_write_reg32(REG_BASE_host_interface + ETH_gpio_data2, data ?
			    GPIO_DATA_SET(gpio - 16) : GPIO_DATA_CLEAR(gpio - 16));
}

void em86xx_gpio_setdirection(int gpio, int dir)
{
	if ((gpio >= 0) && (gpio < 16))
		gbus_write_reg32(REG_BASE_system_block + SYS_gpio_dir, dir ?
			    GPIO_DIR_OUTPUT(gpio) : GPIO_DIR_INPUT(gpio));
	else /* Upper 16 */
		gbus_write_reg32(REG_BASE_host_interface + ETH_gpio_dir2, dir ?
			    GPIO_DIR_OUTPUT(gpio - 16) : GPIO_DIR_INPUT(gpio - 16));
}

#elif defined(CONFIG_TANGO3)

/* For Tango3, the GPIO pins are
 * 	0 .. 15: System GPIO
 * 	16 .. 17: TDMX GPIO0/1 (only 864x)
 * 	18 .. 24: UART0 GPIO: RXD, CTS, DSR, DCD, TXD, RTS, DTR
 *	25 .. 31: UART1 GPIO: RXD, CTS, DSR, DCD, TXD, RTS, DTR
 *	32 .. 50: ETH0 GPIO: TXCLK, TXEN, TXD0, TXD1, TXD2, TXD3
 *			RXCLK, RXDV, RXER, RXD0, RXD1, RXD2, RXD3,
 *			CRS, COL, MDC, MDIO, MDINT#, TXER
 *	51 .. 69: ETH1 GPIO: TXCLK, TXEN, TXD0, TXD1, TXD2, TXD3
 *			RXCLK, RXDV, RXER, RXD0, RXD1, RXD2, RXD3,
 *			CRS, COL, MDC, MDIO, MDINT#, TXER
 *	70 .. 76: SCARD0 GPIO: (not 8652)
 *			RST, CLK, FCB, IO, CTL0, CTL1, CTL2
 *	77 .. 83: SCARD1 GPIO: (not 8652)
 *			RST, CLK, FCB, IO, CTL0, CTL1, CTL2
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
		return -EIO; /* TODO */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		return em86xx_uart0_gpio_read(gpio - 18);
	} else if ((gpio >= 25) && (gpio < 32)) { /* UART1 */
		return em86xx_uart1_gpio_read(gpio - 25);
	} else if ((gpio >= 32) && (gpio < 51)) { /* ETH0 */
		gpio -= 32;
		if (gpio < 16)
			return (gbus_read_reg32(REG_BASE_host_interface + 0x6408) >> gpio) & 1;
		else 
			return (gbus_read_reg32(REG_BASE_host_interface + 0x6410) >> (gpio - 16)) & 1;
	} else if ((gpio >= 51) && (gpio < 70)) { /* ETH1 */
		gpio -= 51;
		if (gpio < 16)
			return (gbus_read_reg32(REG_BASE_host_interface + 0x6c08) >> gpio) & 1;
		else 
			return (gbus_read_reg32(REG_BASE_host_interface + 0x6c10) >> (gpio - 16)) & 1;
	} else if ((gpio >= 70) && (gpio < 77)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		return (gbus_read_reg32(REG_BASE_cpu_block + 0xc35c) >> (gpio - 70)) & 1;
	} else if ((gpio >= 77) && (gpio < 84)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		return (gbus_read_reg32(REG_BASE_cpu_block + 0xcc5c) >> (gpio - 77)) & 1;
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
		return; /* TODO */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		em86xx_uart0_gpio_write(gpio - 18, data);
	} else if ((gpio >= 25) && (gpio < 32)) { /* UART1 */
		em86xx_uart1_gpio_write(gpio - 25, data);
	} else if ((gpio >= 32) && (gpio < 51)) { /* ETH0 */
		gpio -= 32;
		if (gpio < 16)
			gbus_write_reg32(REG_BASE_host_interface + 0x6408, (1 << (gpio + 16)) | (data << gpio));
		else 
			gbus_write_reg32(REG_BASE_host_interface + 0x6410, (1 << gpio) | (data << (gpio - 16)));
	} else if ((gpio >= 51) && (gpio < 70)) { /* ETH1 */
		gpio -= 51;
		if (gpio < 16)
			gbus_write_reg32(REG_BASE_host_interface + 0x6c08, (1 << (gpio + 16)) | (data << gpio));
		else 
			gbus_write_reg32(REG_BASE_host_interface + 0x6c10, (1 << gpio) | (data << (gpio - 16)));
	} else if ((gpio >= 70) && (gpio < 77)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return;
		gpio -= 70;
		gbus_write_reg32(REG_BASE_cpu_block + 0xc35c, (1 << (gpio + 8)) | (data << gpio));
	} else if ((gpio >= 77) && (gpio < 84)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return;
		gpio -= 77;
		gbus_write_reg32(REG_BASE_cpu_block + 0xcc5c, (1 << (gpio + 8)) | (data << gpio));
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
		return; /* TODO */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		em86xx_uart0_gpio_setdirection(gpio - 18, dir);
	} else if ((gpio >= 25) && (gpio < 32)) { /* UART1 */
		em86xx_uart1_gpio_setdirection(gpio - 25, dir);
	} else if ((gpio >= 32) && (gpio < 51)) { /* ETH0 */
		gpio -= 32;
		if (gpio < 16)
			gbus_write_reg32(REG_BASE_host_interface + 0x6404, (1 << (gpio + 16)) | (dir << gpio));
		else 
			gbus_write_reg32(REG_BASE_host_interface + 0x640c, (1 << gpio) | (dir << (gpio - 16)));
	} else if ((gpio >= 51) && (gpio < 70)) { /* ETH1 */
		gpio -= 51;
		if (gpio < 16)
			gbus_write_reg32(REG_BASE_host_interface + 0x6c04, (1 << (gpio + 16)) | (dir << gpio));
		else 
			gbus_write_reg32(REG_BASE_host_interface + 0x6c0c, (1 << gpio) | (dir << (gpio - 16)));
	} else if ((gpio >= 70) && (gpio < 77)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return;
		gpio -= 70;
		gbus_write_reg32(REG_BASE_cpu_block + 0xc358, (1 << (gpio + 8)) | (dir << gpio));
	} else if ((gpio >= 77) && (gpio < 84)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return;
		gpio -= 77;
		gbus_write_reg32(REG_BASE_cpu_block + 0xcc58, (1 << (gpio + 8)) | (dir << gpio));
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
		return -EIO; /* TODO */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		return ((em86xx_uart0_get_gpio_mode() >> (gpio - 18)) & 1) ? 1 : 0;
	} else if ((gpio >= 25) && (gpio < 32)) { /* UART1 */
		return ((em86xx_uart1_get_gpio_mode() >> (gpio - 25)) & 1) ? 1 : 0;
	} else if ((gpio >= 32) && (gpio < 51)) { /* ETH0 */
		unsigned int pad_mode = gbus_read_reg32(REG_BASE_host_interface + 0x6400) & 7;
		return (pad_mode == M_GPIO) ? 1 : 0;
	} else if ((gpio >= 51) && (gpio < 70)) { /* ETH1 */
		unsigned int pad_mode = gbus_read_reg32(REG_BASE_host_interface + 0x6c00) & 7;
		return (pad_mode == M_GPIO) ? 1 : 0;
	} else if ((gpio >= 70) && (gpio < 77)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		return (gbus_read_reg32(REG_BASE_cpu_block + 0xc360) >> (gpio - 70)) & 1;
	} else if ((gpio >= 77) && (gpio < 84)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		return (gbus_read_reg32(REG_BASE_cpu_block + 0xcc60) >> (gpio - 70)) & 1;
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
		return -EIO; /* TODO */
	} else if ((gpio >= 18) && (gpio < 25)) { /* UART0 */
		gpio -= 18;
		*oldmode = (gbus_read_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_mode) >> gpio) & 1;
		gbus_write_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_mode, (1 << (gpio + 8)) | (newmode << gpio));
	} else if ((gpio >= 25) && (gpio < 32)) { /* UART1 */
		gpio -= 25;
		*oldmode = (gbus_read_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_mode) >> gpio) & 1;
		gbus_write_reg32(REG_BASE_cpu_block + CPU_uart1_gpio_mode, (1 << (gpio + 8)) | (newmode << gpio));
	} else if ((gpio >= 32) && (gpio < 51)) { /* ETH0 */
		unsigned int pad_mode = gbus_read_reg32(REG_BASE_host_interface + 0x6400) & 7;
		if ((pad_mode != M_GPIO) && (newmode != 0)) {
			*oldmode = 0;
			gbus_write_reg32(REG_BASE_host_interface + 0x6400, (gbus_read_reg32(REG_BASE_host_interface + 0x6400) & ~7) | M_GPIO);
		}
	} else if ((gpio >= 51) && (gpio < 70)) { /* ETH1 */
		unsigned int pad_mode = gbus_read_reg32(REG_BASE_host_interface + 0x6c00) & 7;
		if ((pad_mode != M_GPIO) && (newmode != 0)) {
			*oldmode = 0;
			gbus_write_reg32(REG_BASE_host_interface + 0x6c00, (gbus_read_reg32(REG_BASE_host_interface + 0x6c00) & ~7) | M_GPIO);
		}
	} else if ((gpio >= 70) && (gpio < 77)) { /* SCARD0 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		gpio -= 70;
		*oldmode = (gbus_read_reg32(REG_BASE_cpu_block + 0xc360) >> gpio) & 1;
		gbus_write_reg32(REG_BASE_cpu_block + 0xc360, (1 << (gpio + 8)) | (newmode << gpio));
	} else if ((gpio >= 77) && (gpio < 84)) { /* SCARD1 */
		unsigned int chip_id = (tangox_chip_id() >> 16) & 0xfffe;
		if (chip_id == 0x8652)
			return -EINVAL;
		gpio -= 77;
		*oldmode = (gbus_read_reg32(REG_BASE_cpu_block + 0xcc60) >> gpio) & 1;
		gbus_write_reg32(REG_BASE_cpu_block + 0xcc60, (1 << (gpio + 8)) | (newmode << gpio));
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

#ifdef CONFIG_TANGO3
EXPORT_SYMBOL(em86xx_gpio_getmode);
EXPORT_SYMBOL(em86xx_gpio_setmode);
#endif

