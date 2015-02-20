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
#include <asm/tango3/emhwlib_registers_tango3.h>
#include <asm/tango3/tango3_gbus.h>

// UART0
#define CPU_uart0_gpio_dir	    (CPU_UART0_base + CPU_UART_GPIODIR)
#define CPU_uart0_gpio_data	    (CPU_UART0_base + CPU_UART_GPIODATA)
#define CPU_uart0_gpio_mode	    (CPU_UART0_base + CPU_UART_GPIOMODE)

// UART1
#define CPU_uart1_gpio_dir	    (CPU_UART1_base + CPU_UART_GPIODIR)
#define CPU_uart1_gpio_data	    (CPU_UART1_base + CPU_UART_GPIODATA)
#define CPU_uart1_gpio_mode	    (CPU_UART1_base + CPU_UART_GPIOMODE)

// UART2
#define CPU_uart2_gpio_dir	    (CPU_UART2_base + CPU_UART_GPIODIR)
#define CPU_uart2_gpio_data	    (CPU_UART2_base + CPU_UART_GPIODATA)
#define CPU_uart2_gpio_mode	    (CPU_UART2_base + CPU_UART_GPIOMODE)

#define MIPS_CPU_IRQ_BASE 0

#if defined(CONFIG_TANGO3_SMP86XX)
#define IRQ_CONTROLLER_IRQ_BASE 8
#define FIQ_CONTROLLER_IRQ_BASE 72
#define IIQ_CONTROLLER_IRQ_BASE 136 // bit31 of iiq is linux irq 199
#define IRQ_COUNT               64  // 64 interrupt sources
#else
#error "Unsupported Tango3 chip."
#endif

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

#define MAX_LOG2_PCIMEM_MAP  	7   	/* 2^(7+3) = 1024MB */
#define MAX_PCIMEM_MAP_SIZE  	(((1<<(MAX_LOG2_PCIMEM_MAP+3))*7)>>3)	/* Max ~896MB */

#define PCIBIOS_MIN_MEM_EM86XX  (MEMORY_BASE_PCI_MEMORY + 0x40000000UL)   /* base address of EM86xx PCI slave */

// Peripheral bus Registers
#define HOST_pb0_base               0x0000
#define HOST_pb_base_cs(n)          (HOST_pb0_base + (0x0200 * (n)))

#define PB_timing_slot(n)	    (PB_timing0 + (0x04 * (n)))

// Bus Master IDE
#define REG_BASE_host_interface_BMIDE         (REG_BASE_host_interface + IDECTRL_pri_cmdblock)

#define REG_BASE_host_interface_ISAIDE(x)  (REG_BASE_host_interface + HOST_pb_base_cs(x))

/* Only applicable to Tango3 */
#define SDND_MAGIC		0x4d474953      /* SIGM */
#define SDND_MINORS		16
#define SDND_MAX_CS		4
#define KERNEL_SECTOR_SIZE	512

#ifndef __ASSEMBLY__

struct xenv_disk_marker
{
	unsigned int magic; 	/* magic# */
	unsigned int cs;
};

#ifdef CONFIG_PRINTK
int printk(const char * fmt, ...) __attribute__ ((format (printf, 1, 2)));
#else
static inline int printk(const char *s, ...) __attribute__ ((format (printf, 1, 2)));
#endif

// Physical address mapping
static inline unsigned long tangox_dma_address(unsigned long physaddr)
{
	extern unsigned long phy_remap, max_remap_size;

	/* for Tango3, another remap takes place */
	if ((physaddr >= CPU_REMAP_SPACE) && (physaddr < (CPU_REMAP_SPACE + max_remap_size)))
		return(phy_remap + (physaddr - CPU_REMAP_SPACE));
	else {
		printk("<3>" "dma_address conversion failure (0x%08lx in range 0x%08lx-0x%08lx)\n",
			physaddr, (unsigned long)CPU_REMAP_SPACE, (unsigned long)CPU_REMAP_SPACE + max_remap_size);
		return(physaddr); /* use whatever is specified */
	}
}

// Inverted physical address mapping
static inline unsigned long tangox_inv_dma_address(unsigned long mapaddr)
{
	extern unsigned long phy_remap, max_remap_size;

	if ((mapaddr >= phy_remap) && (mapaddr < (phy_remap + max_remap_size)))
		return(CPU_REMAP_SPACE + (mapaddr - phy_remap));
	else {
		printk("<3>" "dma_address inversion failure (0x%08lx in range 0x%08lx-0x%08lx)\n",
			mapaddr, phy_remap, phy_remap + max_remap_size);
		return(mapaddr); /* use whatever is specified */
	}
}
#endif

#endif //__HARDWARE_H

