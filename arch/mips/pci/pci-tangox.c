
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 
 *********************************************************************/

/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <asm/pci.h>
#include <asm/dma.h>

#ifdef CONFIG_TANGO2
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2_pci.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3_pci.h>
#endif

#include <asm/cacheflush.h>

/*
 * computed in prom.c
 */
extern unsigned long em8xxx_kmem_start;
extern unsigned long em8xxx_kmem_size;

/*
 * pci addresses used by dma subsystem
 */
unsigned long g_pcimem_busaddr;
unsigned long g_pcimem_physaddr;
unsigned long g_pcimem_physaddr_end;

void __iomem *pci_iomap(struct pci_dev *dev, int bar, unsigned long maxlen);
//void pci_iounmap(struct pci_dev *dev, void __iomem *addr);
unsigned char *__unused_ptr1__ = (unsigned char *)pci_iomap;
//unsigned char *__unused_ptr2__ = (unsigned char *)pci_iounmap;

/*
 * We can't  touch iospace directly from  kseg1, we need  to remap it.
 * No  need to  remap too  much space,  only 256  bytes per  device is
 * allowed, so 64k should be more than enough.
 */
#define MEMORY_SIZE_PCI_IO			(64 * 1024)

static struct resource tangox_pci_io_resource = {
	.name   = "tangox pci IO space",
 	.start  = MEMORY_BASE_PCI_IO,
 	.end    = MEMORY_BASE_PCI_IO + MEMORY_SIZE_PCI_IO - 1,
	.flags  = IORESOURCE_IO
};

/*
 * give 512MB for PCI memory space
 */
static struct resource tangox_pci_mem_resource = {
        .name   = "tangox pci memory space",
        .start  = MEMORY_BASE_PCI_MEMORY + 0x02000000, /* from 32MB offset */
        .end    = MEMORY_BASE_PCI_MEMORY + 0x1fffffff,
        .flags  = IORESOURCE_MEM
};


/*
 * Need to  remap config  space to access  it from kernel.   Note that
 * remap size is just enough for PCIEM86XX_IDSEL_MAX devices on 1 bus.
 */
#define MEMORY_SIZE_PCI_CONFIG	(PCIEM86XX_IDSEL_MAX * (1 << 11))

/*
 * in ops-tangox.c
 */
extern void __iomem *tangox_pci_config_base;
extern struct pci_ops tangox_pci_ops;
extern int enabled_devices[PCIEM86XX_IDSEL_MAX];

struct pci_controller tangox_controller = {
        .pci_ops        = &tangox_pci_ops,
        .io_resource    = &tangox_pci_io_resource,
        .mem_resource   = &tangox_pci_mem_resource,

	/*
	 * gbus  addresses are  not the  same as  pci  addresses, tell
	 * Linux about this so it can adjust resource addresses.
	 */
	.mem_offset	= MEMORY_BASE_PCI_MEMORY,
	.io_offset	= MEMORY_BASE_PCI_CONFIG,
};

/*
 * helpers to access host interface registers
 */
#define RD_HOST_REG32(r)	\
		gbus_read_reg32(REG_BASE_host_interface + (r))

#define WR_HOST_REG32(r, v)	\
		gbus_write_reg32(REG_BASE_host_interface + (r), (v))

#define RD_HOST_REG8(r)	\
		gbus_read_reg8(REG_BASE_host_interface + (r))

#define WR_HOST_REG8(r, v)	\
		gbus_write_reg8(REG_BASE_host_interface + (r), (v))


/*
 * This is the  interrupt handler for bus fault  interrupt. Just clear
 * it and warn user
 */
static irqreturn_t pci_busfault_intr(int irq, void *devinfo)
{
	static const char *reasons[] = {
		"OK", "Master Abort", "Retry timer expired", "Unknown" };
	static int faultcount = 0;
	unsigned int data;

	data = (RD_HOST_REG8(PCI_host_reg2 + 3) >> 1) & 3;
	WR_HOST_REG8(PCI_host_reg2 + 3, 1);
	WR_HOST_REG8(PCI_host_reg2 + 3, 0);

	/* don't flood */
	if (printk_ratelimit())
		printk("PCI: PCI bus fault (count %d): %s\n",
		       ++faultcount, reasons[data]);

	return IRQ_HANDLED;
}

/*
 * platform initialization code
 */
extern int tangox_pci_host_enabled(void);
extern int tangox_pcidev_enabled(int idsel);
extern unsigned long tangox_chip_id(void);

/* can turned on/off by XENV or cmd line */
extern int pci_enabled; 

/* Status of PCI host */
static int pci_active = 0;

/* the maximum idsel */
static int pci_idsel_max = PCIEM86XX_IDSEL_MAX;

static int __init tangox_pci_init(void)
{
	void __iomem *pci_io_base;
	unsigned long memsize, regsize, membase, max_dma_size, dma_addr;
	u32 data;
	int i;

	/* don't do anything if pci support is not enabled in xenv nor is it by cmd line */
	if (!pci_enabled) {
		return 0;
	} else if (!tangox_pci_host_enabled()) {
		printk(KERN_NOTICE "PCI: pci host support disabled\n");
		return 0;
	} 

	if (((tangox_chip_id() >> 16) & 0xfff0) == 0x8650)
		pci_idsel_max--; /* one less for 865x */

	printk("PCI: Initializing SMP86xx PCI host controller\n");

	/*
	 * Enable pci host support
	 */
	WR_HOST_REG32(PCI_chip_is_host, 0); /* Reset PCI host */
	udelay(1000);
	WR_HOST_REG32(PCI_chip_is_host, 1);

	/*
	 * HOST_REG1 :
	 * [31:16] : # of PCI retry cycle = 0xffff (default = 0xff)
	 * [8] : host Super Request = 0
	 * [3:0] arbitration level = 0x00 (Level 1)
	 */
	WR_HOST_REG32(PCI_host_reg1, 0xffff0000 | PCIEM86XX_ARBITER_LEVEL);

	/*
	 * PCI_CTRL1 :
	 * [17] : enable "Memory Read Multiple" and "Memory Read Line"
	 * [16] [7:0] : enable "prefetch" for PCI slave regions 2..7
	 * [17] : Always enabled
	 * [18] : additional bit for Tango. Long PCI memory read burst
	 */
	WR_HOST_REG32(PCI_pcictrl_reg1, 0x00030000);

	/*
	 * PCI_CTRL2 :
	 * [18] : fast back-to-back capable = 0 (default)
	 * [17] : read FIFO level = 1 (8 level deep, default)
	 * [16] : discard timer enable = 1 (default)
	 * [15:8] : subs latency = 0x06 (default = 0x08)
	 * [7:0] : initial latency = 0x0d (default = 0x0b)
	 */
	WR_HOST_REG32(PCI_pcictrl_reg2, 0x0003060d);

	/*
	 * PCI_CTRL3 :
	 * [16] : slave abort clear = 0
	 * [10:8] : abort interrupt enable = 0 (default)
	 * [2:0] : abort status = 0
	 */
	WR_HOST_REG32(PCI_pcictrl_reg3, 0);

	/* clear any pending PCI bus fault */
	if ((RD_HOST_REG8(PCI_host_reg2 + 3) >> 1) & 0x3) {
		WR_HOST_REG8(PCI_host_reg2 + 3, 1);
		WR_HOST_REG8(PCI_host_reg2 + 3, 0);
	}

	/* Setting pci_configuration_vld */
	WR_HOST_REG8(PCI_host_reg2 + 2, 1);

	/* grant timeout */
	WR_HOST_REG32(PCI_host_reg3, PCIEM86XX_ARBITER_GRANTTIMEOUT);
	WR_HOST_REG32(PCI_host_reg5,
		      PCIEM86XX_ARBITER_GRANTTIMEOUT & 0x000000ff);

	/* initialize arbiter */
	WR_HOST_REG32(PCI_host_reg4, 0);


	/*
	 * remap iorange and give port base to linux.
	 */
	pci_io_base = ioremap(MEMORY_BASE_PCI_IO, MEMORY_SIZE_PCI_IO);
	set_io_port_base((unsigned long)pci_io_base - MEMORY_BASE_PCI_IO);

	tangox_controller.io_map_base = (unsigned long)pci_io_base;

	printk("PCI: Remapped PCI I/O space 0x%08lx to 0x%p, size %u kB\n",
	       MEMORY_BASE_PCI_IO, pci_io_base, MEMORY_SIZE_PCI_IO / 1024);

	/*
	 * remap configuration space also
	 */
	tangox_pci_config_base = ioremap(MEMORY_BASE_PCI_CONFIG,
					 MEMORY_SIZE_PCI_CONFIG);

	printk("PCI: Remapped PCI config space 0x%08lx to 0x%p, size %u kB\n",
	       MEMORY_BASE_PCI_CONFIG, tangox_pci_config_base,
	       MEMORY_SIZE_PCI_CONFIG / 1024);

	memset(&enabled_devices, 0, sizeof (enabled_devices));
	for (i = 1; i < pci_idsel_max; i++) {
		if (tangox_pcidev_enabled(i)) {
			enabled_devices[i] = 1;
		} else {
			printk("PCI: device %d disabled\n", i);
		}
	}

	/*
	 * check that we can probe the EM8XXX at id 0
	 */
	if (tangox_cfg_read32(tangox_pci_config_base, &data)) {
		printk("PCI: Can't initialize EM86XX as a PCI slave\n");
		memset(&enabled_devices, 0, sizeof (enabled_devices));
		return 1;
	}

	/*
	 * configure it as a PCI slave
	 *
	 * set PCI memory size to maximum, so the PCI memory will cover the
	 * whole memory if the total DRAM size is smaller than 112MB/896MB (Tango2/Tango3).
	 * For Tango2, maximum = 128MB => 16MB per region => DMA memory = 16 * 7 = 112MB
	 * For Tango3, maximum = 1024MB => 128MB per region => DMA memory = 128 * 7 = 896MB
	 * But the current code doesn't care 2nd DRAM controller.
	 */
	WR_HOST_REG8(PCI_REG3, MAX_LOG2_PCIMEM_MAP);

	memsize = RD_HOST_REG32(PCI_REG3) & 0x07;

#ifdef CONFIG_TANGO2
	memsize = 1 << memsize;  /* from 1MB to 128MB */
#elif defined(CONFIG_TANGO3)
	memsize = 1 << (memsize + 3); /* from 8MB to 1024MB */
#else
#error Unsupported chip!
#endif

	/* get memory size in bytes / 8 */
	regsize = (memsize << 20) >> 3; /* regsize in MB */

	/* pci command */
	__raw_writew(PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER,
		     (void *)(tangox_pci_config_base + PCI_COMMAND));

	/* base address 0 */
	g_pcimem_busaddr = PCIBIOS_MIN_MEM_EM86XX - MEMORY_BASE_PCI_MEMORY;
	__raw_writel(g_pcimem_busaddr,
		     (void *)(tangox_pci_config_base + PCI_BASE_ADDRESS_0));

	/*
	 * PCI slave access
	 * region 0 (R) : Configuration area
	 * region 1 - 7: mapped to DRAM starting from DRAM_BASE */
	g_pcimem_busaddr += (regsize * 1);
	printk("PCI: Configured SMP86xx as PCI slave with %ldMB "
	       "PCI memory\n", memsize);
	membase = CPHYSADDR(em8xxx_kmem_start);
	max_dma_size = (unsigned long)MAX_DMA_ADDRESS - em8xxx_kmem_start;
	g_pcimem_physaddr = membase;
	g_pcimem_physaddr_end = g_pcimem_physaddr + em8xxx_kmem_size;

	dma_addr = tangox_dma_address(CPHYSADDR(membase));
	for (i = 1; (i < 8) && (max_dma_size > 0); ++i) {
		/* PCI region base registers should contain low address */
		WR_HOST_REG32(PCI_REGION_0_BASE + (i * 4), dma_addr);
		membase += regsize; 
		max_dma_size -= regsize;
		dma_addr += regsize;
	}

	if (g_pcimem_physaddr_end > membase)
		g_pcimem_physaddr_end = membase;
        printk("PCI: Region size is %ldKB\n", regsize >> 10);
        printk("PCI: Map DMA memory 0x%08lx-0x%08lx for PCI at 0x%08lx\n",
	       g_pcimem_physaddr, g_pcimem_physaddr_end, g_pcimem_busaddr);

	/*
	 * register the PCI bus fault interrupt
	 */
	if (request_irq(PCIEM86XX_PCIFAULT_INTR, pci_busfault_intr,
		    IRQF_SHARED, "tangox_pci_fault", &tangox_controller) != 0) {
		printk("PCI: fail to register PCI fault ISR(%d)\n", PCIEM86XX_PCIFAULT_INTR);
		return 0;
	}

	/* finally register pci controller */
	register_pci_controller(&tangox_controller);

	pci_active = 1;

	return 0;
}

arch_initcall(tangox_pci_init);

/* For shutting down all PCI devices on this bus */
void tangox_pci_shutdown(void)
{
	struct pci_bus *bus = tangox_controller.bus;
        struct list_head *list;

	if (pci_active && bus) {
		list_for_each(list, &bus->devices) {
			pci_disable_device(pci_dev_b(list));
		}
	}
}

int
pci_set_dma_mask(struct pci_dev *dev, u64 mask)
{
	if (!pci_dma_supported(dev, mask))
		return -EIO;

	if (em8xxx_kmem_size > (MAX_PCIMEM_MAP_SIZE << 20)) {
#if defined(CONFIG_TANGO2)
		dev->dma_mask = mask & 0x07ffffff;
#elif defined(CONFIG_TANGO3)
		dev->dma_mask = mask & 0x3fffffff;
#else
		dev->dma_mask = mask;
#endif
	} else
		dev->dma_mask = mask;

	return 0;
}
    
int
pci_set_consistent_dma_mask(struct pci_dev *dev, u64 mask)
{
	if (!pci_dma_supported(dev, mask))
		return -EIO;

	if (em8xxx_kmem_size > (MAX_PCIMEM_MAP_SIZE << 20)) {
#if defined(CONFIG_TANGO2)
		dev->dev.coherent_dma_mask = mask & 0x07ffffff;
#elif defined(CONFIG_TANGO3)
		dev->dev.coherent_dma_mask = mask & 0x3fffffff;
#else
		dev->dev.coherent_dma_mask = mask;
#endif
	} else
		dev->dev.coherent_dma_mask = mask;

	return 0;
}

void tangox_pci_enable(void)
{
	struct pci_bus *bus = tangox_controller.bus;
        struct list_head *list;

	if (pci_active) {
		list_for_each(list, &bus->devices) {
			pci_set_master(pci_dev_b(list));
		}
	}
}

EXPORT_SYMBOL(tangox_pci_shutdown);
EXPORT_SYMBOL(tangox_pci_enable);

