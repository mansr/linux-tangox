
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/* For more memcfg operations */

#ifndef _MEMCFG_H_
#define _MEMCFG_H_

#include <linux/config.h>
#include <asm/tango3/tango3.h>
#include <asm/tango3/rmem86xxid.h>
#include <asm/tango3/emhwlib_lram.h>
#include <asm/tango3/emhwlib_dram.h>

static inline int is_valid_memcfg(memcfg_t *memcfg_ptr)
{
	unsigned int sum, i, *ptr;

	if ((memcfg_ptr->signature) != MEMCFG_SIGNATURE)
		return(0);
	for (sum = i = 0, ptr = (unsigned int *)memcfg_ptr;
		i < (sizeof(memcfg_t) / sizeof(unsigned int)); i++, ptr++)
		sum += (*ptr);
	return((sum == 0) ? 1 : 0);
}

static inline void gen_memcfg_checksum(memcfg_t *memcfg_ptr)
{
	unsigned int sum, i, *ptr;

	memcfg_ptr->checksum = 0;
	for (sum = i = 0, ptr = (unsigned int *)memcfg_ptr;
		i < (sizeof(memcfg_t) / sizeof(unsigned int)); i++, ptr++)
		sum += (*ptr);
	memcfg_ptr->checksum = ~sum + 1;
}

#endif /* _MEMCFG_H_ */

