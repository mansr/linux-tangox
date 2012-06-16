
/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/**
  @file   tango4.h
  @brief  

  <long description>

  @author Emmanuel Michon
  @date   2004-05-10
*/

#ifndef __TANGO4_H__
#define __TANGO4_H__
#include <asm/tango4/hardware.h>
//#ifndef EM86XX_CHIP
//#define EM86XX_CHIP EM86XX_CHIPID_TANGO4
//#endif
#include <asm/tango4/rmem86xxid.h>
#include <asm/tango4/emhwlib_lram.h>
#include <asm/tango4/emhwlib_resources_tango4.h>
#ifndef CONFIG_TANGOX_BASE_FREQUENCY
#define TANGOX_BASE_FREQUENCY	27000000 
#else
#define TANGOX_BASE_FREQUENCY CONFIG_TANGOX_BASE_FREQUENCY
#endif
#define TANGO3_UART_FREQUENCY   7372800
#define TANGO4_UART_FREQUENCY   7372800
/* Baudrate setting */
#if defined(CONFIG_TANGO3_SMP86XX) || defined(CONFIG_TANGO4_89XX)
#ifndef CONFIG_TANGOX_BASE_BAUD
#define TANGOX_BASE_BAUD 38400
#else
#define TANGOX_BASE_BAUD CONFIG_TANGOX_BASE_BAUD
#endif
//#define TANGOX_CPU_FREQUENCY 333000000
#else
#error "Unsupported platform"
#endif /* CONFIG_TANGO3_SMP86XX || CONFIG_TANGO4_89XX */

/* Memory size used by Linux */
#ifndef CONFIG_TANGOX_MEMSIZE
#if defined(CONFIG_TANGO3_SMP86XX) || defined(CONFIG_TANGO4_89XX)
#define  TANGOX_SYSTEMRAM_ACTUALSIZE   (32*1024*1024)
#else
#error "Unsupported platform"
#endif /* CONFIG_TANGO3_SMP86XX || CONFIG_TANGO4_89XX */
#else
#define TANGOX_SYSTEMRAM_ACTUALSIZE    CONFIG_TANGOX_MEMSIZE
#endif /* !CONFIG_TANGOX_MEMSIZE */

#define TANGOX_CTRLIRQ 0
#define TANGOX_CTRLFIQ 1
#define TANGOX_CTRLIIQ 2

#if defined(CONFIG_TANGO3_SMP86XX) || defined(CONFIG_TANGO4_89XX)
#define SYS_clkgen_pll        SYS_clkgen1_pll
#endif /* CONFIG_TANGO3_SMP86XX || CONFIG_TANGO4_89XX */

#define RMCHIP_ID_SMP8910 (8910)
#define RMCHIP_ID_SMP8920 (8920)

#endif // __TANGO4_H__

