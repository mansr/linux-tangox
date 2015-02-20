/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
 *
 */

#include "platform.h"

/* For Tango2, it's always there */
#define WITH_RC5_CONTROL
#define WITH_RC6_CONTROL

#define SYS_gpio_base           SYS_gpio_dir

/* NEC Control */
#define IR_NEC_CTRL             (REG_BASE_system_block + SYS_gpio_base + 0x18)
#define IR_NEC_CAPTURE_DATA    	(REG_BASE_system_block + SYS_gpio_base + 0x1c)

#ifdef WITH_RC5_CONTROL
/* RC5 Control */
#define IR_RC5_CTRL             (REG_BASE_system_block + SYS_gpio_base + 0x20)
#define IR_RC5_DECODE_CLK_DIV  	(REG_BASE_system_block + SYS_gpio_base + 0x24)
#define IR_RC5_DECODER_DATA    	(REG_BASE_system_block + SYS_gpio_base + 0x28)
#endif /* WITH_RC5_CONTROL */

#ifdef WITH_RC6_CONTROL
/* RC6 Control */
#define RC6_DWORDS		5	/* 5 dwords = 20 bytes */
#define IR_RC6_CTRL             (REG_BASE_system_block + SYS_gpio_base + 0xe0)
#define IR_RC6_T_CTRL           (REG_BASE_system_block + SYS_gpio_base + 0xe4)
#define IR_RC6_DATA_OUT0        (REG_BASE_system_block + SYS_gpio_base + 0xe8)
#define IR_RC6_DATA_OUT1        (REG_BASE_system_block + SYS_gpio_base + 0xec)
#define IR_RC6_DATA_OUT2        (REG_BASE_system_block + SYS_gpio_base + 0xf0)
#define IR_RC6_DATA_OUT3        (REG_BASE_system_block + SYS_gpio_base + 0xf4)
#define IR_RC6_DATA_OUT4        (REG_BASE_system_block + SYS_gpio_base + 0xf8)
#endif

static unsigned long ir_nec_ctrl = 0;

#ifdef WITH_RC5_CONTROL
static const int rc5_clk_div = 48006;	/* 48006 = 1.778ms, 36018 = 1.334ms, 59994 = 2.222ms */
static unsigned long ir_rc5_ctrl = 0, old_rc5_clk = 0;
#endif /* WITH_RC5_CONTROL */
#ifdef WITH_RC6_CONTROL
static unsigned long ir_rc6_ctrl = 0, ir_rc6_t_ctrl = 0;
#endif /* WITH_RC6_CONTROL */

/* Open the device */
int ir_open(void)
{
	DBG_PRINT("Polling for IR key.\n");

	ir_nec_ctrl = tangox_gbus_read_reg32(IR_NEC_CTRL);
	tangox_gbus_write_reg32(IR_NEC_CTRL, 0);

#ifdef WITH_RC5_CONTROL
	ir_rc5_ctrl = tangox_gbus_read_reg32(IR_RC5_CTRL);
	tangox_gbus_write_reg32(IR_RC5_CTRL, 0);
#endif
#ifdef WITH_RC6_CONTROL
	ir_rc6_ctrl = tangox_gbus_read_reg32(IR_RC6_CTRL);
	tangox_gbus_write_reg32(IR_RC6_CTRL, 0xc0000000);
#endif
	tangox_sync();

	/* Enable the NEC device (CTRL register) */
	/*	31:30 - reserved */
	/*	29:24 	IR_CAPTURE_NBITS [5:0] -> set to 0x1f */
	/*	23:22 - reserved */
	/*	21:16 	GPIO_INFREARED_SEL [5:0] -> set to 0x0c */
	/*	15:14 - reserved */
	/*	13:0	IR_PREDIV_DEVIDER [13:0] -> set to 0x3b10 */
	tangox_gbus_write_reg32(REG_BASE_system_block + SYS_gpio_dir,
		tangox_gbus_read_reg32(REG_BASE_system_block + SYS_gpio_dir) | (1 << (16 + 12)));
	tangox_gbus_write_reg32(IR_NEC_CTRL, 0x1f0c3b10);

#ifdef WITH_RC5_CONTROL
	/* Enable the RC5 device (CTRL register) */
	/*	31:10 - reserved */
	/*	9	IR_RC5_INT_ENABLE -> set */
	/*	8	IR_NEC_INT_DISABLE */
	/*	7	IR_DEBOUNCE_SEL1 -> set */
	/*	6	IR_DEBOUNCE_SEL0 -> set */
	/*	5	IR_DEBOUNCE_ENABLE -> set */
	/*	4	IR_NEC_DISABLE */
	/*	3	IR_RSYNC_1/4 -> set */
	/*	2	IR_RSYNC_1/8 */
	/*	1	IR_SIGNAL_INVERT */
	/*	0	IR_DECODE_ENABLE -> set */
	old_rc5_clk = tangox_gbus_read_reg32(IR_RC5_DECODE_CLK_DIV);
	tangox_gbus_write_reg32(IR_RC5_DECODE_CLK_DIV, rc5_clk_div);
	tangox_gbus_write_reg32(IR_RC5_CTRL, 0x000002e9);
#endif /* WITH_RC5_CONTROL */

#ifdef WITH_RC6_CONTROL
	/* Enable the RC6 device (CTRL register) */
	/*	7	IR_RC6_DATA_IN_INT_ENABLE -> set */
	/*	6	IR_RC6_ERROR_INT_ENABLE -> set */
	/*	5:2	reserved */
	/*	1	IR_SIGNAL_INVERT */
	/*	0	IR_RC6_DECODE_ENABLE -> set */
	tangox_gbus_write_reg32(IR_RC6_CTRL, 0x000000c1);

	/* Tolerance and Duration */
	/*	31:18	Tolerance (typ. 0xbb8) */
	/*	17:0	Duration (typ. 0x2ee0) */
	ir_rc6_t_ctrl = tangox_gbus_read_reg32(IR_RC6_T_CTRL);
	tangox_gbus_write_reg32(IR_RC6_T_CTRL, (0xbb8 << 18) | 0x2ee0);
#endif

	tangox_sync();

	return 0;
}

int ir_close(void)
{
	DBG_PRINT("Stop polling for IR key.\n");
	tangox_gbus_write_reg32(IR_NEC_CTRL, 0);
#ifdef WITH_RC5_CONTROL
	tangox_gbus_write_reg32(IR_RC5_CTRL, 0);
#endif
#ifdef WITH_RC6_CONTROL
	tangox_gbus_write_reg32(IR_RC6_CTRL, 0xc0000000);
#endif
	tangox_sync();

	tangox_gbus_write_reg32(IR_NEC_CTRL, ir_nec_ctrl);
#ifdef WITH_RC5_CONTROL
	tangox_gbus_write_reg32(IR_RC5_DECODE_CLK_DIV, old_rc5_clk);
	tangox_gbus_write_reg32(IR_RC5_CTRL, ir_rc5_ctrl);
#endif
#ifdef WITH_RC6_CONTROL
	tangox_gbus_write_reg32(IR_RC6_CTRL, ir_rc6_ctrl);
	tangox_gbus_write_reg32(IR_RC6_T_CTRL, ir_rc6_t_ctrl);
#endif /* WITH_RC6_CONTROL */

	tangox_sync();
	tangox_usdelay(1000);

	return 0;
}

