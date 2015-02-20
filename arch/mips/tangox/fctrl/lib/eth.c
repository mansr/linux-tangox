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

#define WOL_CTRL   0x602c

static unsigned long wol_ctrl = 0;

int eth_open(void)
{
	DBG_PRINT("Listening for WOL packet.\n");

	wol_ctrl = tangox_gbus_read_reg32(REG_BASE_host_interface + WOL_CTRL);

	tangox_gbus_write_reg32(REG_BASE_host_interface + WOL_CTRL, 6);
	tangox_iob();

	return 0;
}

int eth_close(void)
{
	tangox_gbus_write_reg32(REG_BASE_host_interface + WOL_CTRL, 0x60);
	tangox_iob();

	tangox_gbus_write_reg32(REG_BASE_host_interface + WOL_CTRL, wol_ctrl);
	tangox_iob();

	DBG_PRINT("Stop listening to WOL packet.\n");

	tangox_usdelay(1000);

	return 0;
}

