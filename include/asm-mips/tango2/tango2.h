
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/**
  @file   tango2.h
  @brief  

  <long description>

  @author Emmanuel Michon
  @date   2004-05-10
*/

#ifndef __TANGO2_H__
#define __TANGO2_H__
#include <linux/config.h>
#include <asm/tango2/hardware.h>
#define EM86XX_CHIP EM86XX_CHIPID_TANGO2
#include <asm/tango2/rmem86xxid.h>
#include <asm/tango2/emhwlib_lram.h>
#include <asm/tango2/emhwlib_resources_tango2.h>
#ifndef CONFIG_TANGOX_BASE_FREQUENCY
#define TANGOX_BASE_FREQUENCY	27000000 
#endif
/* Baudrate setting */
#if defined(CONFIG_TANGO2_SMP863X)
#ifndef CONFIG_TANGOX_BASE_BAUD
#define TANGOX_BASE_BAUD 38400
#else
#define TANGOX_BASE_BAUD CONFIG_TANGOX_BASE_BAUD
#endif
//#define TANGOX_CPU_FREQUENCY 333000000
#else
#error "Unsupported platform"
#endif /* CONFIG_TANGO2_SMP863X */

/* Memory size used by Linux */
#ifndef CONFIG_TANGOX_MEMSIZE
#if defined(CONFIG_TANGO2_SMP863X)
#define  TANGOX_SYSTEMRAM_ACTUALSIZE   (32*1024*1024)
#else
#error "Unsupported platform"
#endif /* CONFIG_TANGO2_SMP863X */
#else
#define TANGOX_SYSTEMRAM_ACTUALSIZE    CONFIG_TANGOX_MEMSIZE
#endif /* !CONFIG_TANGOX_MEMSIZE */

#define TANGOX_CTRLIRQ 0
#define TANGOX_CTRLFIQ 1
#define TANGOX_CTRLIIQ 2

#if defined(CONFIG_TANGO2_SMP863X)
#define SYS_clkgen_pll        SYS_clkgen3_pll
#endif

#endif // __TANGO2_H__

