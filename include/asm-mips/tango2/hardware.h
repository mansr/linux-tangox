/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
#ifndef __HARDWARE_H
#define __HARDWARE_H

#include <linux/version.h>
#include <asm/tango2/emhwlib_registers_tango2.h>
#include <asm/tango2/tango2_gbus.h>

// UART0
#define CPU_uart0_gpio_dir	    (CPU_UART0_base + CPU_UART_GPIODIR)
#define CPU_uart0_gpio_data	    (CPU_UART0_base + CPU_UART_GPIODATA)
#define CPU_uart0_gpio_mode	    (CPU_UART0_base + CPU_UART_GPIOMODE)

// UART0
#define CPU_uart1_gpio_dir	    (CPU_UART1_base + CPU_UART_GPIODIR)
#define CPU_uart1_gpio_data	    (CPU_UART1_base + CPU_UART_GPIODATA)
#define CPU_uart1_gpio_mode	    (CPU_UART1_base + CPU_UART_GPIOMODE)

#define MIPS_CPU_IRQ_BASE 0

#define IRQ_CONTROLLER_IRQ_BASE 8
#define FIQ_CONTROLLER_IRQ_BASE 72
#define IIQ_CONTROLLER_IRQ_BASE 136 // bit31 of iiq is linux irq 199
#define IRQ_COUNT               64  // 64 interrupt sources

#define IRQ_SOFTINT                     (IRQ_CONTROLLER_IRQ_BASE+0)   // gnet compatibility
// IDE interrupts
#define IRQ_IDECTRL_IDEDMA	(LOG2_CPU_DMAIDE_INT + IRQ_CONTROLLER_IRQ_BASE)
#define IRQ_IDECTRL_IDE		(LOG2_CPU_IDE_INT + IRQ_CONTROLLER_IRQ_BASE)

// MBUS interface
#define MIF_add_offset		    0x0
#define MIF_cnt_offset              (MIF_W0_CNT - MIF_W0_ADD) //0x04
#define MIF_add2_skip_offset        (MIF_W0_SKIP - MIF_W0_ADD) //0x08
#define MIF_cmd_offset              (MIF_W0_CMD - MIF_W0_ADD) //0x0c

// GPIO
#define GPIO_DIR_INPUT(gpio)        ((1 << (16 + (gpio))))
#define GPIO_DIR_OUTPUT(gpio)       ((1 << (16 + (gpio))) | (1 << (gpio)))
#define GPIO_DATA_SET(gpio)         ((1 << (16 + (gpio))) | (1 << (gpio)))
#define GPIO_DATA_CLEAR(gpio)       ((1 << (16 + (gpio))))

// UART GPIO
#define UART_GPIO_DIR_INPUT(gpio)        ((1 << (8 + (gpio))))
#define UART_GPIO_DIR_OUTPUT(gpio)       ((1 << (8 + (gpio))) | (1 << (gpio)))
#define UART_GPIO_DATA_SET(gpio)         ((1 << (8 + (gpio))) | (1 << (gpio)))
#define UART_GPIO_DATA_CLEAR(gpio)       ((1 << (8 + (gpio))))

/* PCI Memories */
#define MEMORY_BASE_PCI_CONFIG      0x50000000UL  /* PCI configuration */
#define MEMORY_BASE_PCI_IO          0x58000000UL  /* PCI I/O space */
#define MEMORY_BASE_PCI_MEMORY      0x60000000UL  /* PCI Memory Base */

#define MAX_LOG2_PCIMEM_MAP  	7   	/* 2^7 = 128MB */
#define MAX_PCIMEM_MAP_SIZE  	(((1<<MAX_LOG2_PCIMEM_MAP)*7)>>3)	/* Max ~112MB */

#define PCIBIOS_MIN_MEM_EM86XX  (MEMORY_BASE_PCI_MEMORY + 0x10000000UL)   /* base address of EM86xx PCI slave */

// Peripheral bus Registers
#define HOST_pb0_base               0x0000
#define HOST_pb_base_cs(n)          (HOST_pb0_base + (0x0200 * (n)))

#define PB_timing_slot(n)	    (PB_timing0 + (0x04 * (n)))

// Bus Master IDE
#define REG_BASE_host_interface_BMIDE         (REG_BASE_host_interface + IDECTRL_pri_cmdblock)

#define REG_BASE_host_interface_ISAIDE(x)  (REG_BASE_host_interface + HOST_pb_base_cs(x))

#ifndef __ASSEMBLY__
// Physical address mapping
static inline unsigned long tangox_dma_address(unsigned long physaddr)
{
#ifndef CONFIG_TANGOX_USE_TLB_REMAP_DRAM1
	if (physaddr >= MEM_BASE_dram_controller_0_alias)
		return(physaddr); /* for Tango2, it's the same */
	else if (physaddr >= CPU_remap4_address) 
		return(gbus_read_uint32(0, REG_BASE_cpu_block + CPU_remap4) + (physaddr - CPU_remap4_address));
	else if (physaddr >= CPU_remap3_address) 
		return(gbus_read_uint32(0, REG_BASE_cpu_block + CPU_remap3) + (physaddr - CPU_remap3_address));
	else if (physaddr >= CPU_remap2_address) 
		return(gbus_read_uint32(0, REG_BASE_cpu_block + CPU_remap2) + (physaddr - CPU_remap2_address));
#endif
	return(physaddr); 
}

// Inverted physical address mapping
static inline unsigned long tangox_inv_dma_address(unsigned long mapaddr)
{
#ifndef CONFIG_TANGOX_USE_TLB_REMAP_DRAM1
	if ((mapaddr >= MEM_BASE_dram_controller_0_alias) && (mapaddr < MEM_BASE_dram_controller_1))
		return(mapaddr); /* for Tango2, it's the same */
	else {
		unsigned long remap;
		remap = gbus_read_uint32(0, REG_BASE_cpu_block + CPU_remap4);
		if ((mapaddr >= remap) && (mapaddr < (remap + 0x04000000)))
			return(CPU_remap4_address + (mapaddr - remap));
		remap = gbus_read_uint32(0, REG_BASE_cpu_block + CPU_remap3);
		if ((mapaddr >= remap) && (mapaddr < (remap + 0x04000000)))
			return(CPU_remap3_address + (mapaddr - remap));
		remap = gbus_read_uint32(0, REG_BASE_cpu_block + CPU_remap2);
		if ((mapaddr >= remap) && (mapaddr < (remap + 0x04000000)))
			return(CPU_remap2_address + (mapaddr - remap));
	}
#endif
	return(mapaddr); /* for Tango2, it's the same */
}
#endif
		
#endif //__HARDWARE_H

