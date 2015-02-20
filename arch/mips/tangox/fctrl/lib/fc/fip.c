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

void __sync(void);

extern void uart_putc(const char c);
extern void uart_puts(const char *s);
extern void udelay(unsigned int usec);

/* EM86XX */
#define	FIP_BASE				(REG_BASE_system_block+0x500)
#define	FIP_COMMAND				0x40
#define	FIP_DISPLAY_DATA			0x44
#define	FIP_LED_DATA				0x48
#define	FIP_KEY_DATA1				0x4c
#define	FIP_KEY_DATA2				0x50
#define	FIP_SWITCH_DATA				0x54
#define FIP_CONFIG				0x58
#define FIP_INT					0x5c

#define FIP_BUSY				0x200

/* FIP commands							*/
#define	FIP_CMD_DATA_SET_RW_MODE_WRITE_DISPLAY		0x40
#define	FIP_CMD_DATA_SET_RW_MODE_WRITE_LED_PORT		0x41
#define	FIP_CMD_DATA_SET_RW_MODE_READ_KEYS		0x42
#define	FIP_CMD_DATA_SET_RW_MODE_READ_SWITCHES		0x43
#define	FIP_CMD_DATA_SET_ADR_MODE_INCREMENT_ADR		0x40
#define	FIP_CMD_DATA_SET_ADR_MODE_FIXED_ADR		0x44
#define	FIP_CMD_DATA_SET_OP_MODE_NORMAL_OPERATION	0x40
#define	FIP_CMD_DATA_SET_OP_MODE_TEST_MODE		0x48
#define	FIP_CMD_ADR_SETTING				0xC0

static void fip_write_reg(unsigned int offset, unsigned int val);
static unsigned int fip_read_reg(unsigned int offset);

static unsigned int fip_read_reg(unsigned int offset)
{
	unsigned int val = *((volatile unsigned int *)KSEG1ADDR(FIP_BASE + offset));
	return(val);
}

static void fip_write_reg(unsigned int offset, unsigned int val)
{
	*((volatile unsigned int *)KSEG1ADDR(FIP_BASE + offset)) = val;
	__sync();
}

static int is_fip_busy(void)
{
	udelay(10);
	return((fip_read_reg(FIP_CONFIG) & FIP_BUSY) != 0);
}

static void fip_wait_ready(void)
{
	while (is_fip_busy());
}

unsigned long fip_poll_key(unsigned long *table, unsigned long size)
{
	unsigned long key = 0L;

	fip_wait_ready();
	fip_write_reg(FIP_COMMAND, FIP_CMD_DATA_SET_RW_MODE_READ_KEYS);
	fip_wait_ready();
	key = fip_read_reg(FIP_KEY_DATA1); 

	if ((key == 0) || (table == 0) || (*table == 0)) {
#ifdef FC_DEBUG
		if (table == 0)
			uart_puts("FIP NULL table\n");
		if (*table == 0)
			uart_puts("FIP NULL content\n");
		if (key != 0)
			uart_puts("Got any FIP key\n");
#endif
		return(key);
	} else {
		unsigned int i;
		for (i = 0; i < size; i++, table++) {
			if (*table == 0)
				break;
			else if (key == *table) {
#ifdef FC_DEBUG
				uart_puts("Got defined FIP key\n");
#endif
				return(key);
			}
		}
		return(0);
	}
}

