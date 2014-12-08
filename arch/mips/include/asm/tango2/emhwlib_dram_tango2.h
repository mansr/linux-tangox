/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/**
  @file   emhwlib_dram_tango2.h
  @brief  

  Addresses appear in increasing order. It is assumed that computing
  FM_Y-FM_X is a proper way to access the max usable size for FM_X.

  See SMP8630 software spec 3.3

  @author Emmanuel Michon, YH Lin, Julien Soulier
  @date   2005-04-06
*/

#ifndef __EMHWLIB_DRAM_TANGO2_H__
#define __EMHWLIB_DRAM_TANGO2_H__

/* Spec 3.3.5: stage (S4) [fully functional player memory map] */
#define FM_GNET                    0x00000000
#define FM_SCRATCH                 0x00000f08 /* 184 bytes */
#define FM_MEMCFG                  0x00000fc0
#define FM_IRQHANDLER_API          0x00001000
#define FM_XTASK_API               0x00009e00 /* 512 bytes */
#define FM_XOSDBG                  0x0000a000
#define FM_XTASK1DBG               0x0000c000
#define FM_XTASK2DBG               0x0000d000
#define FM_XTASK3DBG               0x0000e000
#define FM_XTASK4DBG               0x0000f000
#define FM_SCRATCH2                0x00010000
#define FM_DRAMCALIBRATION         0x0001f000
#define FM_RESERVED                0x00020000

/*
  Spec 3.3.5: stage (S0) [bootstrap memory map]

  Because you will use zboot/yamon to download linux/CE
  at start of DRAM, the former are away from beginning.
*/
#define FM_ZBOOT                   0x01000000
#define FM_YAMON_text_ram          0x01000000
#define FM_YAMON__ftext_init       0x01200000
#define FM_yamon_appl__ftext       0x01210000
#define FM_linuxmips__ftext        0x00020000

#endif // __EMHWLIB_DRAM_TANGO2_H__
