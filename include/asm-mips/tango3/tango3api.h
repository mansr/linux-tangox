/*
 * include/asm-mips/tango3/tango3api.h
 *
 * Copyright 2002-2007, Sigma Designs, Inc 
 *
 * This file contains SMP86XX controling functions
 *
 */

#ifndef __ASM_ARCH_EM86XX_H
#define __ASM_ARCH_EM86XX_H
#include <asm/tango3/hardware.h>
//
// global variables
// 

//
// from arch/arm/arch-em86xx/irq.c
//

// irq
void em86xx_mask_irq(unsigned int irq);
void em86xx_unmask_irq(unsigned int irq);
void em86xx_wait_irq(unsigned int irq);

// fiq
void em86xx_mask_fiq(unsigned int fiq);
void em86xx_unmask_fiq(unsigned int fiq);

// software interrupt
int em86xx_softirq_isset(int irq);
void em86xx_softirq_set(int irq);
void em86xx_softirq_clr(int irq);
void em86xx_irq_clr(int irq);

//
// from arch/arm/arch-em86xx/em86xxapi.c
//

// Cache
// clean : write dirty buffer (D cache only)
// invalidate : invalidate the contents of cache (I & D cache)
// flush : clean + invalidate
void em86xx_get_cache_state(int *picache, int *pdcache, int *pwriteback);
void em86xx_enable_cache(int icache, int dcache, int writeback);
void em86xx_clean_cache_data(void);
void em86xx_clean_cache_data_region(unsigned int from, unsigned int to);
void em86xx_invalidate_cache_instruction(void);
void em86xx_invalidate_cache_instruction_region(unsigned int from, unsigned int to);
void em86xx_invalidate_cache_data(void);
void em86xx_invalidate_cache_data_region(unsigned int from, unsigned int to);

void em86xx_flush_cache_all(void);
void em86xx_flush_cache_data(void);
void em86xx_flush_cache_data_region(unsigned int from, unsigned int to);

// memory
unsigned int em86xx_get_pciregionsize(void);
unsigned int em86xx_get_dmamemorysize(void);

// switchbox (Host interface)
enum { 
	SBOX_MBUS_W0 = 0, SBOX_MBUS_W1, SBOX_PCIMASTER, SBOX_PCISLAVE, 
	SBOX_SATA1, SBOX_IDEFLASH, SBOX_IDEDVD, SBOX_SATA2, SBOX_MBUS_W2, SBOX_MAX
};

int em86xx_sbox_init(void);
#if 0
void em86xx_sbox_reset(void);
int em86xx_sbox_setup(void);
int em86xx_sbox_connect(int iface);
void em86xx_sbox_disconnect(int port);
#endif

// MBUS DMA 
typedef void (*mbus_irq_handler_t)(int irq, void *arg);

int em86xx_mbus_init(void);
int em86xx_mbus_alloc_dma(int sbox, int fromdev, unsigned long *pregbase, int *pirq);
void em86xx_mbus_free_dma(unsigned long regbase, int sbox);

int em86xx_mbus_setup_dma_common(unsigned int regbase, unsigned int addr, unsigned int count, mbus_irq_handler_t handler, void *arg, unsigned int flags);
void em86xx_mbus_setup_dma_linear(unsigned int regbase, unsigned int addr, unsigned int count, unsigned int flags);
void em86xx_mbus_setup_dma_double(unsigned int regbase, unsigned int addr, unsigned int count, unsigned int addr2, unsigned int count2, unsigned int flags);
void em86xx_mbus_setup_dma_rectangle(unsigned int regbase, unsigned int addr, unsigned int horiz, unsigned int lines, int skip, unsigned int flags);
void em86xx_mbus_setup_dma_void(unsigned int regbase);
int em86xx_mbus_setup_dma(unsigned int regbase, unsigned int addr, unsigned int count, mbus_irq_handler_t handler, void *arg, unsigned int flags);
int em86xx_mbus_inuse(unsigned int regbase);
int em86xx_mbus_wait(unsigned int regbase, int sbox);
int mbus_memcpy(unsigned int regbase, unsigned int src, unsigned int dst, unsigned int size);

// PCI master
void em86xx_pcimaster_setup_read(unsigned int addr, unsigned int count);
void em86xx_pcimaster_start_read(int start);
void em86xx_pcimaster_setup_write(unsigned int addr, unsigned int count);
void em86xx_pcimaster_start_write(int start);

// GPIO
#define GPIO_INPUT		0
#define GPIO_OUTPUT		1

int em86xx_gpio_read(int gpio);
void em86xx_gpio_write(int gpio, int data);
void em86xx_gpio_setdirection(int gpio, int dir);

int em86xx_gpio_getmode(int gpio);
int em86xx_gpio_setmode(int gpio, int mode, int *oldmode);

int em86xx_uart0_gpio_read(int gpio);
void em86xx_uart0_gpio_write(int gpio, int data);
void em86xx_uart0_gpio_setdirection(int gpio, int dir);
int em86xx_uart1_gpio_read(int gpio);
void em86xx_uart1_gpio_write(int gpio, int data);
void em86xx_uart1_gpio_setdirection(int gpio, int dir);

#endif

