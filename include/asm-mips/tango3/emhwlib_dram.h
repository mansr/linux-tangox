/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/**
  @file   emhwlib_dram.h
  @brief  

  fm stands for: first megabyte.

  THE CODE USING THESE SYMBOLS ASSUMES THAT THE END BOUNDARY OF AN
  ENTITY IS THE START BOUNDARY OF THE NEXT ENTITY
  
  htoinc.pl emhwlib_dram.h emhwlib_dram.inc
  
  @author Emmanuel Michon
  @date   2004-07-26
*/

#ifndef __EMHWLIB_DRAM_H__
#define __EMHWLIB_DRAM_H__

#if EM86XX_CHIP>=EM86XX_CHIPID_TANGO3
#include "emhwlib_dram_tango3.h"
#elif EM86XX_CHIP>=EM86XX_CHIPID_TANGO2
#include "emhwlib_dram_tango2.h"
#else
#include "emhwlib_dram_others.h"
#endif

#define MEMCFG_SIGNATURE	0x6766636d // `m' `c' `f' `g'

#ifndef __ASSEMBLY__

/* This is the memory map data structure, the size is 64 bytes */
typedef struct {
	unsigned int signature;                                                           // ...fc0
	unsigned int dram0_size;            /* The size of DRAM0 */
	unsigned int dram1_size;            /* The size of DRAM1 */
	unsigned int dram2_size;            /* The size of DRAM2 */
	unsigned int dram0_removable_topreserved;     /* The size of top reserved in DRAM0   ...fd0 */
	unsigned int dram1_removable_topreserved;     /* The size of top reserved in DRAM1 */
	unsigned int dram0_top_removable_area;    /* for special use such as splash screen */ 
	                                          /* users can use set and get properties on the memory reserved by this variable */
	unsigned int dram0_fixed_topreserved;     /* The size of top reserved in DRAM0 */
	unsigned int dram1_fixed_topreserved;     /* The size of top reserved in DRAM1       ...fe0 */
	unsigned int dram2_fixed_topreserved;     /* The size of top reserved in DRAM2 */
	unsigned int kernel_end;            /* The end offset of kernel */
	unsigned int checksum;		    /* The checksum */
#if EM86XX_CHIP>=EM86XX_CHIPID_TANGO2
	unsigned int dram1_kernel_end;	    /* The end offset of kernel used data in second dram */
	unsigned int curtainA0;                                                      
	unsigned int curtainB0;
	unsigned int curtainC;
#else
	unsigned int reserved[4];           /* Reserved for extension */
#endif
} memcfg_t;

#endif /* __ASSEMBLY__ */

#endif // __EMHWLIB_DRAM_H__
 
