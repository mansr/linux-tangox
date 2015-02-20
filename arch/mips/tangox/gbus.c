
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
 * export gbus symbol to modules
 */

#include <linux/module.h>
#include <asm/system.h>

#include "setup.h"

#define TMP_REMAPPED_REG	CPU_remap1
#define TMP_REMAPPED_BASE	CPU_remap1_address
#define TMP_REMAPPED_SIZE	0x00010000
#define TMP_REMAPPED_MASK	~(TMP_REMAPPED_SIZE-1)

#define TMP_REMAPPED_REG1	CPU_remap
#define TMP_REMAPPED_BASE1	CPU_remap_address
#define TMP_REMAPPED_SIZE1	0x00010000
#define TMP_REMAPPED_MASK1	~(TMP_REMAPPED_SIZE1-1)

#if defined(CONFIG_TANGO2) && defined(CONFIG_TANGOX_USE_TLB_REMAP_DRAM1)
unsigned long em86xx_tlb_dram1_map_base;
unsigned long em86xx_tlb_dram1_map_size;
#endif

static RMuint32 set_remap(RMuint32 remap_reg, RMuint32 value)
{
	RMuint32 orig = *((volatile RMuint32 *)KSEG1ADDR(REG_BASE_cpu_block + remap_reg));
	if (orig != value) {
		*((volatile RMuint32 *)KSEG1ADDR(REG_BASE_cpu_block + remap_reg)) = value;
		iob();
	}
	return(orig);
}

#if defined(CONFIG_TANGO3)

#define BUILD_GBUS_READ_OP(size)									\
RMuint##size gbus_read_uint##size(struct gbus *pgbus, RMuint32 byte_address) 				\
{													\
	RMuint32 remap;											\
	RMuint##size tmp;										\
	extern unsigned long phy_remap, max_remap_size;							\
	if (byte_address < CPU_remap2_address) {							\
		tmp = *((volatile RMuint##size *)KSEG1ADDR(byte_address)); 				\
		rmb();											\
	} else if ((byte_address >= phy_remap) && (byte_address < (phy_remap + max_remap_size))) {	\
		tmp = *((volatile RMuint##size *)KSEG1ADDR(CPU_REMAP_SPACE + (byte_address - phy_remap))); 	\
		rmb();											\
	} else {											\
		unsigned long flags;									\
		local_irq_save(flags); /* Ensure remap won't be changed */				\
		/* Use CPU_remapx to temporarily map the address */					\
		if (likely(byte_address != (REG_BASE_cpu_block + TMP_REMAPPED_REG))) {			\
			remap = set_remap(TMP_REMAPPED_REG, byte_address & TMP_REMAPPED_MASK);		\
			tmp = *((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE + 			\
								(byte_address & (TMP_REMAPPED_SIZE-1))));	\
			rmb();										\
			set_remap(TMP_REMAPPED_REG, remap & TMP_REMAPPED_MASK);				\
		} else {										\
			remap = set_remap(TMP_REMAPPED_REG1, byte_address & TMP_REMAPPED_MASK1);	\
			tmp = *((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE1 + 		\
								(byte_address & (TMP_REMAPPED_SIZE1-1))));	\
			rmb();										\
			set_remap(TMP_REMAPPED_REG1, remap & TMP_REMAPPED_MASK1);			\
		}											\
		local_irq_restore(flags);								\
	}												\
	return(tmp);											\
}

#define BUILD_GBUS_WRITE_OP(size)									\
void gbus_write_uint##size(struct gbus *pgbus, RMuint32 byte_address, RMuint##size data)		\
{													\
	RMuint32 remap;											\
	extern unsigned long phy_remap, max_remap_size;							\
	mb();												\
	if (byte_address < CPU_remap2_address) {							\
		*((volatile RMuint##size *)KSEG1ADDR(byte_address)) = data;				\
		iob();											\
	} else if ((byte_address >= phy_remap) && (byte_address < (phy_remap + max_remap_size))) {	\
		*((volatile RMuint##size *)KSEG1ADDR(CPU_REMAP_SPACE + (byte_address - phy_remap))) = data;	\
		iob();											\
	} else {											\
		unsigned long flags;									\
		local_irq_save(flags); /* Ensure remap won't be changed */				\
		/* Use CPU_remapx to temporarily map the address */					\
		if (likely(byte_address != (REG_BASE_cpu_block + TMP_REMAPPED_REG))) {			\
			remap = set_remap(TMP_REMAPPED_REG, byte_address & TMP_REMAPPED_MASK);		\
			*((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE + 			\
							(byte_address & (TMP_REMAPPED_SIZE-1)))) = data;	\
			iob();										\
			set_remap(TMP_REMAPPED_REG, remap & TMP_REMAPPED_MASK);				\
		} else {										\
			remap = set_remap(TMP_REMAPPED_REG1, byte_address & TMP_REMAPPED_MASK1);	\
			*((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE1 + 			\
							(byte_address & (TMP_REMAPPED_SIZE1-1)))) = data;	\
			iob();										\
			set_remap(TMP_REMAPPED_REG1, remap & TMP_REMAPPED_MASK1);			\
		}											\
		local_irq_restore(flags);								\
	}												\
}

#elif defined(CONFIG_TANGO2)

#ifdef CONFIG_TANGOX_USE_TLB_REMAP_DRAM1

#define BUILD_GBUS_READ_OP(size)									\
RMuint##size gbus_read_uint##size(struct gbus *pgbus, RMuint32 byte_address) 				\
{													\
	RMuint32 remap;											\
	RMuint##size tmp;										\
	if (byte_address < (MEM_BASE_dram_controller_0 + 0x10000000))					\
		return gbus_read_dram_uint##size(pgbus, byte_address);					\
	else if (byte_address < (MEM_BASE_dram_controller_1 + 0x10000000))				\
		if (byte_address < (MEM_BASE_dram_controller_1 + em86xx_tlb_dram1_map_size))		\
			return gbus_read_dram_uint##size(pgbus, byte_address);				\
		else {											\
			printk("accessing non-existed DRAM1 area 0x%08lx\n", byte_address);		\
			return(0);									\
		}											\
	else {												\
		unsigned long flags;									\
		local_irq_save(flags); /* Ensure remap won't be changed */				\
		/* Use CPU_remapx to temporarily map the address */					\
		if (likely(byte_address != (REG_BASE_cpu_block + TMP_REMAPPED_REG))) {			\
			remap = set_remap(TMP_REMAPPED_REG, byte_address & TMP_REMAPPED_MASK);		\
			tmp = *((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE + 			\
								(byte_address & (TMP_REMAPPED_SIZE-1))));	\
			set_remap(TMP_REMAPPED_REG, remap & TMP_REMAPPED_MASK);				\
		} else {										\
			remap = set_remap(TMP_REMAPPED_REG1, byte_address & TMP_REMAPPED_MASK1);	\
			tmp = *((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE1 + 		\
								(byte_address & (TMP_REMAPPED_SIZE1-1))));	\
			set_remap(TMP_REMAPPED_REG1, remap & TMP_REMAPPED_MASK1);			\
		}											\
		local_irq_restore(flags);								\
		return(tmp);										\
	}												\
}

#define BUILD_GBUS_WRITE_OP(size)									\
void gbus_write_uint##size(struct gbus *pgbus, RMuint32 byte_address, RMuint##size data)		\
{													\
	RMuint32 remap;											\
	if (byte_address < (MEM_BASE_dram_controller_0 + 0x10000000))					\
		gbus_write_dram_uint##size(pgbus, byte_address, data);					\
	else if (byte_address < (MEM_BASE_dram_controller_1 + 0x10000000))				\
		if (byte_address < (MEM_BASE_dram_controller_1 + em86xx_tlb_dram1_map_size))		\
			gbus_write_dram_uint##size(pgbus, byte_address, data);				\
		else 											\
			printk("accessing non-existed DRAM1 area 0x%08lx.\n", byte_address);		\
	else {												\
		unsigned long flags;									\
		local_irq_save(flags); /* Ensure remap won't be changed */				\
		/* Use CPU_remapx to temporarily map the address */					\
		if (likely(byte_address != (REG_BASE_cpu_block + TMP_REMAPPED_REG))) {			\
			remap = set_remap(TMP_REMAPPED_REG, byte_address & TMP_REMAPPED_MASK);		\
			*((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE + 			\
							(byte_address & (TMP_REMAPPED_SIZE-1)))) = data;	\
			set_remap(TMP_REMAPPED_REG, remap & TMP_REMAPPED_MASK);				\
		} else {										\
			remap = set_remap(TMP_REMAPPED_REG1, byte_address & TMP_REMAPPED_MASK1);	\
			*((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE1 + 			\
							(byte_address & (TMP_REMAPPED_SIZE1-1)))) = data;	\
			set_remap(TMP_REMAPPED_REG1, remap & TMP_REMAPPED_MASK1);			\
		}											\
		local_irq_restore(flags);								\
	}												\
	__sync();											\
}

#else /* CONFIG_TANGOX_USE_TLB_REMAP_DRAM1 */

#define BUILD_GBUS_READ_OP(size)									\
RMuint##size gbus_read_uint##size(struct gbus *pgbus, RMuint32 byte_address) 				\
{													\
	RMuint32 remap;											\
	RMuint##size tmp;										\
	if (byte_address < (MEM_BASE_dram_controller_0 + 0x10000000))					\
		return gbus_read_dram_uint##size(pgbus, byte_address);					\
	else if (byte_address < (MEM_BASE_dram_controller_1 + 0x10000000))				\
		return gbus_read_dram_uint##size(pgbus, byte_address);					\
	else {												\
		unsigned long flags;									\
		local_irq_save(flags); /* Ensure remap won't be changed */				\
		/* Use CPU_remapx to temporarily map the address */					\
		if (likely(byte_address != (REG_BASE_cpu_block + TMP_REMAPPED_REG))) {			\
			remap = set_remap(TMP_REMAPPED_REG, byte_address & TMP_REMAPPED_MASK);		\
			tmp = *((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE + 			\
								(byte_address & (TMP_REMAPPED_SIZE-1))));	\
			set_remap(TMP_REMAPPED_REG, remap & TMP_REMAPPED_MASK);				\
		} else {										\
			remap = set_remap(TMP_REMAPPED_REG1, byte_address & TMP_REMAPPED_MASK1);	\
			tmp = *((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE1 + 		\
								(byte_address & (TMP_REMAPPED_SIZE1-1))));	\
			set_remap(TMP_REMAPPED_REG1, remap & TMP_REMAPPED_MASK1);			\
		}											\
		local_irq_restore(flags);								\
		return(tmp);										\
	}												\
}

#define BUILD_GBUS_WRITE_OP(size)									\
void gbus_write_uint##size(struct gbus *pgbus, RMuint32 byte_address, RMuint##size data)		\
{													\
	RMuint32 remap;											\
	if (byte_address < (MEM_BASE_dram_controller_0 + 0x10000000))					\
		gbus_write_dram_uint##size(pgbus, byte_address, data);					\
	else if (byte_address < (MEM_BASE_dram_controller_1 + 0x10000000))				\
		gbus_write_dram_uint##size(pgbus, byte_address, data);					\
	else {												\
		unsigned long flags;									\
		local_irq_save(flags); /* Ensure remap won't be changed */				\
		/* Use CPU_remapx to temporarily map the address */					\
		if (likely(byte_address != (REG_BASE_cpu_block + TMP_REMAPPED_REG))) {			\
			remap = set_remap(TMP_REMAPPED_REG, byte_address & TMP_REMAPPED_MASK);		\
			*((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE + 			\
							(byte_address & (TMP_REMAPPED_SIZE-1)))) = data;	\
			set_remap(TMP_REMAPPED_REG, remap & TMP_REMAPPED_MASK);				\
		} else {										\
			remap = set_remap(TMP_REMAPPED_REG1, byte_address & TMP_REMAPPED_MASK1);	\
			*((volatile RMuint##size *)KSEG1ADDR(TMP_REMAPPED_BASE1 + 			\
							(byte_address & (TMP_REMAPPED_SIZE1-1)))) = data;	\
			set_remap(TMP_REMAPPED_REG1, remap & TMP_REMAPPED_MASK1);			\
		}											\
		local_irq_restore(flags);								\
	}												\
	__sync();											\
}

#endif /* CONFIG_TANGOX_USE_TLB_REMAP_DRAM1 */

#else
#error Unsupported platform.
#endif /* CONFIG_TANGO3 */

BUILD_GBUS_READ_OP(32);
BUILD_GBUS_READ_OP(16);
BUILD_GBUS_READ_OP(8);

BUILD_GBUS_WRITE_OP(32);
BUILD_GBUS_WRITE_OP(16);
BUILD_GBUS_WRITE_OP(8);

EXPORT_SYMBOL(gbus_read_uint32);
EXPORT_SYMBOL(gbus_write_uint32);
EXPORT_SYMBOL(gbus_read_uint16);
EXPORT_SYMBOL(gbus_write_uint16);
EXPORT_SYMBOL(gbus_read_uint8);
EXPORT_SYMBOL(gbus_write_uint8);

#if defined(CONFIG_TANGO2) && defined(CONFIG_TANGOX_USE_TLB_REMAP_DRAM1)
EXPORT_SYMBOL(em86xx_tlb_dram1_map_base);
EXPORT_SYMBOL(em86xx_tlb_dram1_map_size);
#endif

