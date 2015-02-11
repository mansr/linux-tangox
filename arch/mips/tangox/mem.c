#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <asm/bootinfo.h>
#include <asm/page.h>
#include <asm/io.h>

#include "memmap.h"
#include "setup.h"

#define REMAP_SIZE	0x04000000u

static void __iomem *remap_base;
static unsigned long tangox_remap[8];

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

	tangox_remap_set(0, 0x1fc00000);
	tangox_remap_set(1, 0);

	for (i = 0; i < 6; i++)
		tangox_remap_set(2 + i, REMAP2_BASE + i * REMAP_SIZE);
}
