/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2006  Ralf Baechle <ralf@linux-mips.org>
 *
 */
#ifndef __ASM_MACH_TANGO3_DMA_COHERENCE_H
#define __ASM_MACH_TANGO3_DMA_COHERENCE_H

struct device;

#ifdef CONFIG_TANGOX_REMAP
extern u8 tangox_remap[64];
extern u8 tangox_remap_inv[64];

static inline dma_addr_t tangox_phys_to_dma(phys_addr_t physaddr)
{
	unsigned long idx = physaddr >> 26;
	unsigned long offset = physaddr & ((1 << 26) - 1);

	return tangox_remap[idx] << 26 | offset;
}

static inline phys_addr_t tangox_dma_to_phys(dma_addr_t dma_addr)
{
	unsigned long idx = dma_addr >> 26;
	unsigned long offset = dma_addr & ((1 << 26) - 1);

	return tangox_remap_inv[idx] << 26 | offset;
}
#else
static inline dma_addr_t tangox_phys_to_dma(phys_addr_t physaddr)
{
	return physaddr;
}

static inline phys_addr_t tangox_dma_to_phys(dma_addr_t dma_addr)
{
	return dma_addr;
}
#endif

static inline dma_addr_t plat_map_dma_mem(struct device *dev, void *addr,
					  size_t size)
{
	return tangox_phys_to_dma(virt_to_phys(addr));
}

static inline dma_addr_t plat_map_dma_mem_page(struct device *dev,
					       struct page *page)
{
	return tangox_phys_to_dma(page_to_phys(page));
}

static inline unsigned long plat_dma_addr_to_phys(struct device *dev,
						  dma_addr_t dma_addr)
{
	return tangox_dma_to_phys(dma_addr);
}

static inline void plat_unmap_dma_mem(struct device *dev, dma_addr_t dma_addr,
	size_t size, enum dma_data_direction direction)
{
}

static inline int plat_dma_supported(struct device *dev, u64 mask)
{
	return 1;
}

static inline int plat_device_is_coherent(struct device *dev)
{
	return 0;
}

#endif /* __ASM_MACH_TANGO3_DMA_COHERENCE_H */
