#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <asm/bootinfo.h>
#include <asm/page.h>
#include <asm/io.h>

#include "memmap.h"
#include "setup.h"

#define REMAP_IDX	2
#define REMAP_SIZE	0x04000000u
#define FIXMAP_ADDR	(DRAM0_MEM_BASE + 0x0c000000)

static void __iomem *remap_base;
static unsigned long tangox_remap[9];
static unsigned long kmem_size;

unsigned long tangox_remap_set(int reg, unsigned long addr)
{
	unsigned long old = tangox_remap[reg];

	writel(addr, remap_base + reg * 4);
	mb();

	tangox_remap[reg] = addr;

	return old;
}

void __init tangox_remap_init(void)
{
	int i;

	remap_base = ioremap(REMAP_CTL_BASE, 32);

	for (i = 0; i < 8; i++)
		tangox_remap[i] = readl(remap_base + i * 4);

	tangox_remap[8] = FIXMAP_ADDR;

	tangox_remap_set(0, 0x1fc00000);
	tangox_remap_set(1, 0);
}

static inline dma_addr_t tangox_phys_to_dma(phys_addr_t physaddr)
{
	unsigned long idx = ((physaddr >> 26) & 7) + 1;
	unsigned long offset = physaddr & (REMAP_SIZE - 1);

	if (physaddr < PHYS_OFFSET || physaddr >= PHYS_OFFSET + kmem_size)
		return physaddr;

	return tangox_remap[idx] + offset;
}

static inline phys_addr_t tangox_dma_to_phys(dma_addr_t dma_addr)
{
	unsigned long offset = dma_addr & (REMAP_SIZE - 1);
	unsigned long base = dma_addr & ~(REMAP_SIZE - 1);
	int i;

	for (i = 0; i < kmem_size >> 26; i++)
		if (tangox_remap[REMAP_IDX + i] == base)
			return ((i + 1) << 26) + offset;

	return dma_addr;
}

dma_addr_t plat_map_dma_mem(struct device *dev, void *addr, size_t size)
{
	return tangox_phys_to_dma(virt_to_phys(addr));
}

dma_addr_t plat_map_dma_mem_page(struct device *dev, struct page *page)
{
	return tangox_phys_to_dma(page_to_phys(page));
}

unsigned long plat_dma_addr_to_phys(struct device *dev, dma_addr_t dma_addr)
{
	return tangox_dma_to_phys(dma_addr);
}

static size_t __init tangox_remap_range(int *reg, phys_addr_t *pa, size_t *sz)
{
	phys_addr_t addr = *pa;
	size_t size = *sz;
	size_t map_size = min(size, REMAP_SIZE);
	size_t tsize;
	int i = *reg;

	if (!size)
		return 0;

	while (size >= map_size && i < 8) {
		pr_info("  remap%d: 0x%08x @ 0x%08x -> 0x%08x\n", i,
			map_size, addr, (i - 1) * REMAP_SIZE);
		tangox_remap_set(i, addr);
		addr += map_size;
		size -= map_size;
		i++;
	}

	tsize = addr - *pa;

	*reg = i;
	*pa = addr;
	*sz = size;

	return tsize;
}

static size_t __init tangox_remap_dram(void)
{
	phys_addr_t d_addr[2];
	size_t d_size[2];
	size_t m_size = 0;
	int ri = REMAP_IDX;
	int di;
	int i;

	d_addr[0] = DRAM0_MEM_BASE;
	d_size[0] = tangox_dram_size(0);

	d_addr[1] = DRAM1_MEM_BASE;
	d_size[1] = tangox_dram_size(1);

	for (i = 0; i < 2; i++)
		pr_info("DRAM%d: 0x%08x @ 0x%08x\n", i, d_size[i], d_addr[i]);

	pr_info("Mapping DRAM:\n");

	di = tangox_remap[REMAP_IDX] >= DRAM1_MEM_BASE;

	for (i = 0; i < 2; i++) {
		m_size += tangox_remap_range(&ri, &d_addr[di], &d_size[di]);
		di ^= 1;
		m_size += tangox_remap_range(&ri, &d_addr[di], &d_size[di]);
		di = d_size[1] > d_size[0];
	}

	if (d_addr[0] + d_size[0] > FIXMAP_ADDR)
		m_size += d_addr[0] + d_size[0] - FIXMAP_ADDR;

	return m_size;
}

void __init tangox_mem_setup(void)
{
	kmem_size = tangox_remap_dram();
	add_memory_region(PHYS_OFFSET, kmem_size, BOOT_MEM_RAM);
}
