/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2000  Ani Joshi <ajoshi@unixbox.com>
 * Copyright (C) 2000, 2001, 06  Ralf Baechle <ralf@linux-mips.org>
 * swiped from i386, and cloned for MIPS by Geert, polished by Ralf.
 */

#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <linux/highmem.h>

#include <asm/cache.h>
#include <asm/io.h>

//#include <dma-coherence.h>

struct device;

#ifdef CONFIG_TANGO2
#include <asm/tango2/hardware.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#endif

extern unsigned long g_pcimem_busaddr;
extern unsigned long g_pcimem_physaddr;
extern unsigned long g_pcimem_physaddr_end;
extern struct bus_type pci_bus_type;

#ifdef CONFIG_PCI
#define IS_PCIDEV(x)	(((x) != NULL) && ((x)->bus == &pci_bus_type))
#else
#define IS_PCIDEV(x)	0
#endif

#ifdef CONFIG_PCI
static inline unsigned long __pci_phys_to_bus(unsigned long physaddr)
{
	if ((physaddr < g_pcimem_physaddr) || (physaddr >= g_pcimem_physaddr_end)) {
		printk("phys2bus: Not a dma-able address: 0x%08lx\n", physaddr);
		return 0;
	}
	return (unsigned long)((physaddr - g_pcimem_physaddr) + g_pcimem_busaddr);
}

static inline unsigned long __pci_virt_to_bus(unsigned long virt)
{
	return __pci_phys_to_bus(CPHYSADDR(virt));
}

static inline unsigned long __pci_bus_to_phys(unsigned long busaddr) 
{
	if ((busaddr < g_pcimem_busaddr) ||
	    (busaddr >= (g_pcimem_busaddr + (g_pcimem_physaddr_end - g_pcimem_physaddr)))) {
		printk("bus2phys: Not a valid bus address: 0x%08lx\n", busaddr);
		return 0;
	}
	return (unsigned long)(busaddr - g_pcimem_busaddr) + g_pcimem_physaddr;
}

static inline unsigned long __pci_bus_to_virt(unsigned long busaddr)
{
	return (unsigned long)phys_to_virt(__pci_bus_to_phys(busaddr));
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

static inline unsigned long plat_dma_addr_to_phys(struct device *dev, dma_addr_t dma_addr)
{
#ifdef CONFIG_PCI
	if (IS_PCIDEV(dev)) {
		if ((dma_addr >= g_pcimem_busaddr) &&
		    (dma_addr < (g_pcimem_busaddr + (g_pcimem_physaddr_end - g_pcimem_physaddr)))) 
			return __pci_bus_to_phys(dma_addr);
	}
#endif
	return tangox_inv_dma_address(dma_addr);
}

static inline struct page *dma_addr_to_page(struct device *dev, dma_addr_t dma_addr)
{
	return pfn_to_page(plat_dma_addr_to_phys(dev, dma_addr) >> PAGE_SHIFT);
}

static inline dma_addr_t plat_map_dma_mem(struct device *dev, void *physaddr, size_t size)
{
#ifdef CONFIG_PCI
	if (IS_PCIDEV(dev))
		return __pci_virt_to_bus((unsigned long)physaddr);
#endif
	return tangox_dma_address((unsigned long)physaddr);
}

static inline dma_addr_t plat_map_dma_mem_page(struct device *dev, struct page *page)
{
#ifdef CONFIG_PCI
	if (IS_PCIDEV(dev))
		return __pci_virt_to_bus((unsigned long)page_to_phys(page));
#endif
	return tangox_dma_address(page_to_phys(page));
}

static inline void plat_unmap_dma_mem(struct device *dev, dma_addr_t dma_addr)
{
	/* nothing to be done */
}

static inline int plat_device_is_coherent(struct device *dev)
{
	return 0; /* Always noncoherent for Tango2/3/4 */
}

static inline int plat_dma_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	return (dma_addr == 0) ? 1 : 0;
}

static inline int plat_dma_supported(struct device *dev, u64 mask)
{
	/* 
         * we fall back to GFP_DMA when the mask isn't all 1s,
         * so we can't guarantee allocations that must be
         * within a tighter range than GFP_DMA..
         */
	if (mask < DMA_BIT_MASK(24))
		return 0;
	return 1;
}

static inline void plat_extra_sync_for_device(struct device *dev)
{
	return;
}

/*
 * Warning on the terminology - Linux calls an uncached area coherent;
 * MIPS terminology calls memory areas with hardware maintained coherency
 * coherent.
 */

static inline int cpu_is_noncoherent_r10000(struct device *dev)
{
	return !plat_device_is_coherent(dev) &&
	       (current_cpu_type() == CPU_R10000 ||
	       current_cpu_type() == CPU_R12000);
}

static gfp_t massage_gfp_flags(const struct device *dev, gfp_t gfp)
{
	/* ignore region specifiers */
	gfp &= ~(__GFP_DMA | __GFP_DMA32 | __GFP_HIGHMEM);

#ifdef CONFIG_ZONE_DMA
	if (dev == NULL)
		gfp |= __GFP_DMA;
	else if (dev->coherent_dma_mask < DMA_BIT_MASK(24))
		gfp |= __GFP_DMA;
	else
#endif
#ifdef CONFIG_ZONE_DMA32
	     if (dev->coherent_dma_mask < DMA_BIT_MASK(32))
		gfp |= __GFP_DMA32;
	else
#endif
		;

	/* Don't invoke OOM killer */
	gfp |= __GFP_NORETRY;

	return gfp;
}

void *dma_alloc_noncoherent(struct device *dev, size_t size,
	dma_addr_t *dma_handle, gfp_t gfp)
{
	void *ret;

	gfp = massage_gfp_flags(dev, gfp);

	ret = (void *)__get_free_pages(gfp, get_order(size));

	if (ret != NULL) {
		memset(ret, 0, size);
		*dma_handle = plat_map_dma_mem(dev, (void *)virt_to_phys(ret), size);
	}

	return ret;
}
EXPORT_SYMBOL(dma_alloc_noncoherent);

void *dma_alloc_coherent(struct device *dev, size_t size,
	dma_addr_t *dma_handle, gfp_t gfp)
{
	void *ret;

	ret = dma_alloc_noncoherent(dev, size, dma_handle, gfp);
	if (ret != NULL) {
		if (!plat_device_is_coherent(dev)) {
			dma_cache_wback_inv((unsigned long)ret, size);
			ret = UNCAC_ADDR(ret);
		}
	}
	return ret;
}
EXPORT_SYMBOL(dma_alloc_coherent);

void dma_free_noncoherent(struct device *dev, size_t size, void *vaddr,
	dma_addr_t dma_handle)
{
	plat_unmap_dma_mem(dev, dma_handle);
	free_pages((unsigned long)vaddr, get_order(size));
}
EXPORT_SYMBOL(dma_free_noncoherent);

void dma_free_coherent(struct device *dev, size_t size, void *vaddr,
	dma_addr_t dma_handle)
{
	unsigned long addr = (unsigned long)vaddr;

	if (!plat_device_is_coherent(dev))
		addr = CAC_ADDR(addr);
	dma_free_noncoherent(dev, size, (void *)addr, dma_handle);
}
EXPORT_SYMBOL(dma_free_coherent);

static inline void __dma_sync_virtual(void *addr, size_t size,
	enum dma_data_direction direction)
{
	switch (direction) {
	case DMA_TO_DEVICE:
		dma_cache_wback((unsigned long)addr, size);
		break;

	case DMA_FROM_DEVICE:
		dma_cache_inv((unsigned long)addr, size);
		break;

	case DMA_BIDIRECTIONAL:
		dma_cache_wback_inv((unsigned long)addr, size);
		break;

	default:
		BUG();
	}
}

/* 
 * A single sg entry may refer to multiple physically contiguous
 * pages, But we still need to processs highmem pages individually.
 * If highmem is not configured then the bulk of this loops gets
 * optimized out.
 */
static inline void __dma_sync(struct page *page, unsigned long offset, 
				size_t size, enum dma_data_direction direction)
{
	size_t left = size;
	do {
		size_t len = left;
		if (PageHighMem(page)) {
			unsigned long flags;
			void *addr;
			if (offset + len > PAGE_SIZE) {
				if (offset >= PAGE_SIZE) {
					page += (offset >> PAGE_SHIFT);
					offset &= ~PAGE_MASK;
				}
				len = PAGE_SIZE - offset;
			}
			local_irq_save(flags);
			addr = kmap_atomic(page, KM_SYNC_DCACHE);
			__dma_sync_virtual(addr + offset, len, direction);
			kunmap_atomic(addr, KM_SYNC_DCACHE);
			local_irq_restore(flags);
		} else
			__dma_sync_virtual(page_address(page) + offset, size, direction);
		offset = 0;
		page++;
		left -= len;
	} while (left);
}

dma_addr_t dma_map_single(struct device *dev, void *ptr, size_t size,
	enum dma_data_direction direction)
{
	extern unsigned long tangox_virt_to_phys(void *);
	dma_addr_t dma_addr;
	unsigned long paddr = tangox_virt_to_phys(ptr);

	if (paddr < HIGHMEM_START)
		dma_addr = plat_map_dma_mem(dev, (void *)paddr, size);
	else
		dma_addr = paddr; /* direct gbus address, mostly by highmem */

	if (!plat_device_is_coherent(dev)) 
		__dma_sync(dma_addr_to_page(dev, dma_addr), dma_addr & ~PAGE_MASK, size, direction);
	return dma_addr;
}
EXPORT_SYMBOL(dma_map_single);

void dma_unmap_single(struct device *dev, dma_addr_t dma_addr, size_t size,
	enum dma_data_direction direction)
{
	if (cpu_is_noncoherent_r10000(dev))
		__dma_sync(dma_addr_to_page(dev, dma_addr), dma_addr & ~PAGE_MASK, size, direction);

	plat_unmap_dma_mem(dev, dma_addr);
}
EXPORT_SYMBOL(dma_unmap_single);

int dma_map_sg(struct device *dev, struct scatterlist *sg, int nents,
	enum dma_data_direction direction)
{
	int i;

	BUG_ON(direction == DMA_NONE);

	for (i = 0; i < nents; i++, sg++) {
		if (!plat_device_is_coherent(dev))
			__dma_sync(sg_page(sg), sg->offset, sg->length, direction);
		sg->dma_address = plat_map_dma_mem_page(dev, sg_page(sg)) + sg->offset;
	}

	return nents;
}
EXPORT_SYMBOL(dma_map_sg);

dma_addr_t dma_map_page(struct device *dev, struct page *page,
	unsigned long offset, size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (!plat_device_is_coherent(dev)) 
		__dma_sync(page, offset, size, direction);

	return plat_map_dma_mem_page(dev, page) + offset;
}
EXPORT_SYMBOL(dma_map_page);

void dma_unmap_page(struct device *dev, dma_addr_t dma_address, size_t size,
	enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (!plat_device_is_coherent(dev) && direction != DMA_TO_DEVICE) 
		__dma_sync(dma_addr_to_page(dev, dma_address), dma_address & ~PAGE_MASK, size, direction);

	plat_unmap_dma_mem(dev, dma_address);
}
EXPORT_SYMBOL(dma_unmap_page);

void dma_unmap_sg(struct device *dev, struct scatterlist *sg, int nhwentries,
	enum dma_data_direction direction)
{
	int i;

	BUG_ON(direction == DMA_NONE);

	for (i = 0; i < nhwentries; i++, sg++) {
		if (!plat_device_is_coherent(dev) && direction != DMA_TO_DEVICE) 
			__dma_sync(sg_page(sg), sg->offset, sg->length, direction);
		plat_unmap_dma_mem(dev, sg->dma_address);
	}
}
EXPORT_SYMBOL(dma_unmap_sg);

void dma_sync_single_for_cpu(struct device *dev, dma_addr_t dma_handle,
	size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (cpu_is_noncoherent_r10000(dev)) {
		__dma_sync(dma_addr_to_page(dev, dma_handle),
				dma_handle & ~PAGE_MASK, size, direction);
	}
}
EXPORT_SYMBOL(dma_sync_single_for_cpu);

void dma_sync_single_for_device(struct device *dev, dma_addr_t dma_handle,
	size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	plat_extra_sync_for_device(dev);
	if (!plat_device_is_coherent(dev)) {
		__dma_sync(dma_addr_to_page(dev, dma_handle),
				dma_handle & ~PAGE_MASK, size, direction);
	}
}
EXPORT_SYMBOL(dma_sync_single_for_device);

void dma_sync_single_range_for_cpu(struct device *dev, dma_addr_t dma_handle,
	unsigned long offset, size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	if (cpu_is_noncoherent_r10000(dev)) {
		__dma_sync(dma_addr_to_page(dev, dma_handle + offset),
				dma_handle & ~PAGE_MASK, size, direction);
	}
}
EXPORT_SYMBOL(dma_sync_single_range_for_cpu);

void dma_sync_single_range_for_device(struct device *dev, dma_addr_t dma_handle,
	unsigned long offset, size_t size, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);

	plat_extra_sync_for_device(dev);
	if (!plat_device_is_coherent(dev)) {
		__dma_sync(dma_addr_to_page(dev, dma_handle + offset),
				dma_handle & ~PAGE_MASK, size, direction);
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
			__dma_sync(sg_page(sg), sg->offset, sg->length, direction);
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
			__dma_sync(sg_page(sg), sg->offset, sg->length, direction);
	}
}
EXPORT_SYMBOL(dma_sync_sg_for_device);

int dma_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	return plat_dma_mapping_error(dev, dma_addr);
}
EXPORT_SYMBOL(dma_mapping_error);

int dma_supported(struct device *dev, u64 mask)
{
	return plat_dma_supported(dev, mask);
}
EXPORT_SYMBOL(dma_supported);

int dma_is_consistent(struct device *dev, dma_addr_t dma_addr)
{
	return plat_device_is_coherent(dev);
}
EXPORT_SYMBOL(dma_is_consistent);

