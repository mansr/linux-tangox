/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2000  Ani Joshi <ajoshi@unixbox.com>
 * Copyright (C) 2000, 2001, 06  Ralf Baechle <ralf@linux-mips.org>
 * swiped from i386, and cloned for MIPS by Geert, polished by Ralf.
 *
 * Adopted for Sigma Designs's Tango2/Tango3 platforms.
 *
 * Copyright (C) 2007-2009  Sigma Designs, Inc.
 *
 */

#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/pci.h>

#include <asm/cache.h>
#include <asm/io.h>

#ifdef CONFIG_TANGO2
#include <asm/tango2/hardware.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#endif

extern unsigned long g_pcimem_busaddr;
extern unsigned long g_pcimem_physaddr;
extern unsigned long g_pcimem_physaddr_end;

#ifdef CONFIG_PCI
#define IS_PCIDEV(x)	((x)->bus == &pci_bus_type)
#else
#define IS_PCIDEV(x)	0
#endif

#ifdef CONFIG_PCI
static inline unsigned long __pci_virt_to_bus(unsigned long virt)
{
	if ((virt_to_phys((void *)virt) < g_pcimem_physaddr) ||
	    (virt_to_phys((void *)virt) >= g_pcimem_physaddr_end)) {
		printk("virt2bus: Not a dma-able address: 0x%08lx\n", virt);
		return 0;
	}
	return((unsigned long)(virt_to_phys((void *)virt) - g_pcimem_physaddr +
			       g_pcimem_busaddr));
}

static inline unsigned long __pci_bus_to_virt(unsigned long busaddr)
{
	if ((busaddr < g_pcimem_busaddr) ||
	    (busaddr >= (g_pcimem_busaddr + (g_pcimem_physaddr_end - g_pcimem_physaddr)))) {
		printk("bus2virt: Not a valid bus address: 0x%08lx\n",
		       busaddr);
		return 0;
	}
	return((unsigned long)phys_to_virt((busaddr - g_pcimem_busaddr) +
			       g_pcimem_physaddr));
}
#else
static inline unsigned long __pci_virt_to_bus(unsigned long addr)
{
	BUG();
	return 0;
}

static inline unsigned long __pci_bus_to_virt(unsigned long addr)
{
	BUG();
	return 0;
}
#endif

unsigned long pci_virt_to_bus(unsigned long addr)
{
	return __pci_virt_to_bus(addr);
}

EXPORT_SYMBOL(pci_virt_to_bus);

unsigned long pci_bus_to_virt(unsigned long addr)
{
	return __pci_bus_to_virt(addr);
}

EXPORT_SYMBOL(pci_bus_to_virt);

static inline dma_addr_t plat_map_dma_mem(struct device *dev, void *addr, size_t size)
{
	if (IS_PCIDEV(dev))
		return __pci_virt_to_bus((unsigned long)addr);
	return tangox_dma_address(virt_to_phys(addr));
}

static inline dma_addr_t plat_map_dma_mem_page(struct device *dev, struct page *page)
{
	if (IS_PCIDEV(dev))
		return __pci_virt_to_bus((unsigned long)page_to_phys(page));
	return tangox_dma_address(page_to_phys(page));
}

static inline unsigned long plat_dma_addr_to_phys(dma_addr_t dma_addr)
{
	return tangox_inv_dma_address(dma_addr);
}

static inline void plat_unmap_dma_mem(dma_addr_t dma_addr)
{
	/* nothing to be done */
}

static inline int plat_device_is_coherent(struct device *dev)
{
	return 0; /* Always noncoherent for Tango2/Tango3 */
}

static inline unsigned long dma_addr_to_virt(dma_addr_t dma_addr)
{
	unsigned long addr = plat_dma_addr_to_phys(dma_addr);

	return (unsigned long)phys_to_virt(addr);
}

/*
 * Warning on the terminology - Linux calls an uncached area coherent;
 * MIPS terminology calls memory areas with hardware maintained coherency
 * coherent.
 */

static inline int cpu_is_noncoherent_r10000(struct device *dev)
{
	return !plat_device_is_coherent(dev) &&
	       (current_cpu_data.cputype == CPU_R10000 &&
	       current_cpu_data.cputype == CPU_R12000);
}

void *dma_alloc_noncoherent(struct device *dev, size_t size,
	dma_addr_t * dma_handle, gfp_t gfp)
{
	void *ret;

	/* ignore region specifiers */
	gfp &= ~(__GFP_DMA | __GFP_HIGHMEM);

	if (dev == NULL || (dev->coherent_dma_mask < 0xffffffff))
		gfp |= GFP_DMA;
	ret = (void *) __get_free_pages(gfp, get_order(size));

	if (ret != NULL) {
//		memset(ret, 0, size); /* may not be needed?? */
		*dma_handle = plat_map_dma_mem(dev, ret, size);
	}

	return ret;
}

EXPORT_SYMBOL(dma_alloc_noncoherent);

void *dma_alloc_coherent(struct device *dev, size_t size,
	dma_addr_t * dma_handle, gfp_t gfp)
{
	void *ret;

	/* ignore region specifiers */
	gfp &= ~(__GFP_DMA | __GFP_HIGHMEM);

	if (dev == NULL || (dev->coherent_dma_mask < 0xffffffff))
		gfp |= GFP_DMA;
	ret = (void *) __get_free_pages(gfp, get_order(size));

	if (ret) {
//		memset(ret, 0, size); /* may not be needed?? */
		*dma_handle = plat_map_dma_mem(dev, ret, size);

		if (!plat_device_is_coherent(dev)) {
			dma_cache_wback_inv((unsigned long) ret, size);
			ret = UNCAC_ADDR(ret);
		}
	}

	return ret;
}

EXPORT_SYMBOL(dma_alloc_coherent);

void dma_free_noncoherent(struct device *dev, size_t size, void *vaddr,
	dma_addr_t dma_handle)
{
	free_pages((unsigned long) vaddr, get_order(size));
}

EXPORT_SYMBOL(dma_free_noncoherent);

void dma_free_coherent(struct device *dev, size_t size, void *vaddr,
	dma_addr_t dma_handle)
{
	unsigned long addr = (unsigned long) vaddr;

	if (!plat_device_is_coherent(dev))
		addr = CAC_ADDR(addr);

	free_pages(addr, get_order(size));
}

EXPORT_SYMBOL(dma_free_coherent);

static inline void __dma_sync(unsigned long addr, size_t size,
	enum dma_data_direction direction)
{
	switch (direction) {
	case DMA_TO_DEVICE:
		dma_cache_wback(addr, size);
		break;

	case DMA_FROM_DEVICE:
		dma_cache_inv(addr, size);
		break;

	case DMA_BIDIRECTIONAL:
		dma_cache_wback_inv(addr, size);
		break;

	default:
		BUG();
	}
}

dma_addr_t dma_map_single(struct device *dev, void *ptr, size_t size,
	enum dma_data_direction direction)
{
	unsigned long addr = (unsigned long) ptr;

	if (!plat_device_is_coherent(dev))
		__dma_sync(addr, size, direction);

	return plat_map_dma_mem(dev, ptr, size);
}

EXPORT_SYMBOL(dma_map_single);

void dma_unmap_single(struct device *dev, dma_addr_t dma_addr, size_t size,
	enum dma_data_direction direction)
{
	if (cpu_is_noncoherent_r10000(dev))
		__dma_sync(dma_addr_to_virt(dma_addr), size,
		           direction);

	plat_unmap_dma_mem(dma_addr);
}

EXPORT_SYMBOL(dma_unmap_single);

int dma_map_sg(struct device *dev, struct scatterlist *sg, int nents,
	enum dma_data_direction direction)
{
	int i;

	BUG_ON(direction == DMA_NONE);

	for (i = 0; i < nents; i++, sg++) {
		unsigned long addr;

		addr = (unsigned long) page_address(sg->page);
		if (!plat_device_is_coherent(dev) && addr)
			__dma_sync(addr + sg->offset, sg->length, direction);
		sg->dma_address = plat_map_dma_mem(dev,
				                   (void *)(addr + sg->offset),
						   sg->length);
	}

	return nents;
}

EXPORT_SYMBOL(dma_map_sg);

void dma_unmap_sg(struct device *dev, struct scatterlist *sg, int nhwentries,
	enum dma_data_direction direction)
{
	unsigned long addr;
	int i;

	BUG_ON(direction == DMA_NONE);

	for (i = 0; i < nhwentries; i++, sg++) {
		if (!plat_device_is_coherent(dev) &&
		    direction != DMA_TO_DEVICE) {
			addr = (unsigned long) page_address(sg->page);
			if (addr)
				__dma_sync(addr + sg->offset, sg->length,
				           direction);
		}
		plat_unmap_dma_mem(sg->dma_address);
	}
}

EXPORT_SYMBOL(dma_unmap_sg);

dma_addr_t dma_map_page(struct device *dev, struct page *page,
	unsigned long offset, size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (!plat_device_is_coherent(dev)) {
		unsigned long addr;

		addr = (unsigned long) page_address(page) + offset;
//		dma_cache_wback_inv(addr, size);
		__dma_sync(addr, size, direction);
	}

	return plat_map_dma_mem_page(dev, page) + offset;
}

EXPORT_SYMBOL(dma_map_page);

void dma_unmap_page(struct device *dev, dma_addr_t dma_address, size_t size,
	enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (!plat_device_is_coherent(dev) && direction != DMA_TO_DEVICE) {
		unsigned long addr;

		addr = plat_dma_addr_to_phys(dma_address);
//		dma_cache_wback_inv(addr, size);
		__dma_sync(addr, size, direction);
	}

	plat_unmap_dma_mem(dma_address);
}

EXPORT_SYMBOL(dma_unmap_page);

void dma_sync_single_for_cpu(struct device *dev, dma_addr_t dma_handle,
	size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (cpu_is_noncoherent_r10000(dev)) {
		unsigned long addr;

		addr = dma_addr_to_virt(dma_handle);
		__dma_sync(addr, size, direction);
	}
}

EXPORT_SYMBOL(dma_sync_single_for_cpu);

void dma_sync_single_for_device(struct device *dev, dma_addr_t dma_handle,
	size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (!plat_device_is_coherent(dev)) {
		unsigned long addr;

		addr = dma_addr_to_virt(dma_handle);
		__dma_sync(addr, size, direction);
	}
}

EXPORT_SYMBOL(dma_sync_single_for_device);

void dma_sync_single_range_for_cpu(struct device *dev, dma_addr_t dma_handle,
	unsigned long offset, size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (cpu_is_noncoherent_r10000(dev)) {
		unsigned long addr;

		addr = dma_addr_to_virt(dma_handle);
		__dma_sync(addr + offset, size, direction);
	}
}

EXPORT_SYMBOL(dma_sync_single_range_for_cpu);

void dma_sync_single_range_for_device(struct device *dev, dma_addr_t dma_handle,
	unsigned long offset, size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (!plat_device_is_coherent(dev)) {
		unsigned long addr;

		addr = dma_addr_to_virt(dma_handle);
		__dma_sync(addr + offset, size, direction);
	}
}

EXPORT_SYMBOL(dma_sync_single_range_for_device);

void dma_sync_sg_for_cpu(struct device *dev, struct scatterlist *sg, int nelems,
	enum dma_data_direction direction)
{
	int i;

	BUG_ON(direction == DMA_NONE);

	/* Make sure that gcc doesn't leave the empty loop body.  */
	for (i = 0; i < nelems; i++, sg++) {
		if (cpu_is_noncoherent_r10000(dev))
			__dma_sync((unsigned long)page_address(sg->page),
			           sg->length, direction);
		plat_unmap_dma_mem(sg->dma_address);
	}
}

EXPORT_SYMBOL(dma_sync_sg_for_cpu);

void dma_sync_sg_for_device(struct device *dev, struct scatterlist *sg, int nelems,
	enum dma_data_direction direction)
{
	int i;

	BUG_ON(direction == DMA_NONE);

	/* Make sure that gcc doesn't leave the empty loop body.  */
	for (i = 0; i < nelems; i++, sg++) {
		if (!plat_device_is_coherent(dev))
			__dma_sync((unsigned long)page_address(sg->page),
			           sg->length, direction);
		plat_unmap_dma_mem(sg->dma_address);
	}
}

EXPORT_SYMBOL(dma_sync_sg_for_device);

int dma_mapping_error(dma_addr_t dma_addr)
{
	return (dma_addr == 0) ? 1 : 0;
}

EXPORT_SYMBOL(dma_mapping_error);

int dma_supported(struct device *dev, u64 mask)
{
	/*
	 * we fall back to GFP_DMA when the mask isn't all 1s,
	 * so we can't guarantee allocations that must be
	 * within a tighter range than GFP_DMA..
	 */
	if (mask < 0x00ffffff)
		return 0;

	return 1;
}

EXPORT_SYMBOL(dma_supported);

int dma_is_consistent(struct device *dev, dma_addr_t dma_addr)
{
	return plat_device_is_coherent(dev);
}

EXPORT_SYMBOL(dma_is_consistent);

void dma_cache_sync(struct device *dev, void *vaddr, size_t size,
	       enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (!plat_device_is_coherent(dev))
		dma_cache_wback_inv((unsigned long)vaddr, size);
}

EXPORT_SYMBOL(dma_cache_sync);

