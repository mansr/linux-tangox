
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
  
#ifndef __TANGOX_IDE_H
#define __TANGOX_IDE_H

#ifdef CONFIG_TANGO2
#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/tango2api.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/tango3api.h>
#endif

#endif /* __TANGOX_IDE_H */

