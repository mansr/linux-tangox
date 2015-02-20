/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/**
  @file   emhwlib_lram.h
  @brief  

  Map of the localram (8KBytes)

  Traditionnally the start of localram is used to setup
  a few kilobytes bootstrap routine code+data
  (cache init, tlb init, load something bigger to DRAM, jump there).

  Fixed offsets are defined in this file as communication devices
  between hardware blocks.
  Even debug locations must be present here.

  The bootstrap routine is expected to preserve these and setup
  its stack under LR_STACKTOP.

  Keep addresses increasing in this file.

  See emhwlib_resources_shared.h how some resources bw. 0 and 0x100 are used already
  only when uCLinux is up with irq handler running

  @author Emmanuel Michon
  @date   2005-03-17
*/

#ifndef __EMHWLIB_LRAM_H__
#define __EMHWLIB_LRAM_H__

#if (EM86XX_CHIP<EM86XX_CHIPID_TANGO3)
#include "emhwlib_lram_others.h"
#elif (EM86XX_CHIP==EM86XX_CHIPID_TANGO3)
#include "emhwlib_lram_tango3.h"
#else
#error EM86XX_CHIP is not set in RMCFLAGS: refer to rmdef/rmem86xxid.h. 
#endif

#endif // __EMHWLIB_LRAM_H__
