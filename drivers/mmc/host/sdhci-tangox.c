/*
 *  linux/drivers/mmc/host/sdhci-tangox.c - SDHCI on Sigma Designs' SDIO interface
 *
 *  Copyright (C) 2005-2009 Sigma Designs, Inc.
 *
 *  Origianlly from
 *
 *  linux/drivers/mmc/host/sdhci-pci.c - SDHCI on PCI bus interface
 *
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * Thanks to the following companies for their support:
 *
 *     - JMicron (hardware and technical support)
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <linux/mmc/host.h>
#if defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#include <asm/tango3/emhwlib_registers_tango3.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/hardware.h>
#include <asm/tango4/emhwlib_registers_tango4.h>
#endif
#include <asm/scatterlist.h>
#include <asm/io.h>

#include "sdhci.h"

#define MAX_SLOTS			2
#define LOG2_CPU_SDIO0_INT		60
#define LOG2_CPU_SDIO1_INT		61

struct sdio_controller {
	unsigned long base;
	unsigned int irq;
};
#define TANGOX_SDIO_BASE_ADDR		(REG_BASE_host_interface + 0x1000)
#define TANGOX_SDIO_BASE_ADDR1		(REG_BASE_host_interface + 0x1200)

static struct sdio_controller tangox_hosts[] = {
	{ KSEG1ADDR(TANGOX_SDIO_BASE_ADDR), IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_SDIO0_INT },
	{ KSEG1ADDR(TANGOX_SDIO_BASE_ADDR1), IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_SDIO1_INT },
};

struct sdhci_tangox_chip;
struct sdhci_tangox_slot;

struct sdhci_tangox_fixes {
	unsigned int		quirks;

	int			(*probe)(struct sdhci_tangox_chip*);

	int			(*probe_slot)(struct sdhci_tangox_slot*);
	void			(*remove_slot)(struct sdhci_tangox_slot*, int);

	int			(*suspend)(struct sdhci_tangox_chip*,
					pm_message_t);
	int			(*resume)(struct sdhci_tangox_chip*);
};

struct sdhci_tangox_slot {
	struct sdhci_tangox_chip	*chip;
	struct sdhci_host		*host;
};

struct sdhci_tangox_chip {
	unsigned int		quirks;
	struct sdhci_tangox_fixes *fixes;
	struct device		*pdev;

	int			num_slots;	/* Slots on controller */
	struct sdhci_tangox_slot	*slots[MAX_SLOTS]; /* Pointers to host slots */
};
static unsigned int sdio_hosts = 3; 
static int force_speed = -1; /* -1: auto, 0: low, 1: high */ 
/*****************************************************************************\
 *                                                                           *
 * Hardware specific quirk handling                                          *
 *                                                                           *
\*****************************************************************************/

static int tangox_probe(struct sdhci_tangox_chip *chip)
{
	return 0;
}

static int tangox_probe_slot(struct sdhci_tangox_slot *slot)
{
	return 0;
}

static void tangox_remove_slot(struct sdhci_tangox_slot *slot, int dead)
{
	return;
}

static int tangox_suspend(struct sdhci_tangox_chip *chip, pm_message_t state)
{
	return 0;
}

static int tangox_resume(struct sdhci_tangox_chip *chip)
{
	return 0;
}

static struct sdhci_tangox_fixes sdhci_tangox = {
	.quirks		= 0,
	.probe		= tangox_probe,

	.probe_slot	= tangox_probe_slot,
	.remove_slot	= tangox_remove_slot,

	.suspend	= tangox_suspend,
	.resume		= tangox_resume,
};

/*****************************************************************************\
 *                                                                           *
 * SDHCI core callbacks                                                      *
 *                                                                           *
\*****************************************************************************/

static int sdhci_tangox_enable_dma(struct sdhci_host *host)
{
	/* DMA should be OK */
	return 0;
}

static struct sdhci_ops sdhci_tangox_ops = {
	.enable_dma	= sdhci_tangox_enable_dma,
};

/*****************************************************************************\
 *                                                                           *
 * Suspend/resume                                                            *
 *                                                                           *
\*****************************************************************************/

#ifdef CONFIG_PM

static int sdhci_tangox_suspend (struct device *pdev, pm_message_t state)
{
	struct sdhci_tangox_chip *chip;
	struct sdhci_tangox_slot *slot;
	int i, ret;

	chip = dev_get_drvdata(pdev);
	if (!chip)
		return 0;

	for (i = 0; i < chip->num_slots; i++) {
		slot = chip->slots[i];
		if (!slot)
			continue;

		ret = sdhci_suspend_host(slot->host);

		if (ret) {
			for (i--; i >= 0; i--)
				sdhci_resume_host(chip->slots[i]->host);
			return ret;
		}
	}

	if (chip->fixes && chip->fixes->suspend) {
		ret = chip->fixes->suspend(chip, state);
		if (ret) {
			for (i = chip->num_slots - 1; i >= 0; i--)
				sdhci_resume_host(chip->slots[i]->host);
			return ret;
		}
	}

	// TODO: put it to suspend mode 

	return 0;
}

static int sdhci_tangox_resume(struct device *pdev)
{
	struct sdhci_tangox_chip *chip;
	struct sdhci_tangox_slot *slot;
	int i, ret;

	chip = dev_get_drvdata(pdev);
	if (!chip)
		return 0;

	// TODO: resume 

	if (chip->fixes && chip->fixes->resume) {
		ret = chip->fixes->resume(chip);
		if (ret)
			return ret;
	}

	for (i = 0; i < chip->num_slots; i++) {
		slot = chip->slots[i];
		if (!slot)
			continue;

		ret = sdhci_resume_host(slot->host);
		if (ret)
			return ret;
	}

	return 0;
}
#else 
#define sdhci_tangox_suspend NULL
#define sdhci_tangox_resume NULL
#endif /* CONFIG_PM */

/*****************************************************************************\
 *                                                                           *
 * Device probing/removal                                                    *
 *                                                                           *
\*****************************************************************************/

static struct sdhci_tangox_slot *sdhci_tangox_probe_slot(
				struct device *pdev, struct sdhci_tangox_chip *chip, int idx)
{
	struct sdhci_tangox_slot *slot;
	struct sdhci_host *host;
	int ret;

	host = sdhci_alloc_host(pdev, sizeof(struct sdhci_tangox_slot));
	if (IS_ERR(host)) {
		dev_err(pdev, "cannot allocate host\n");
		return ERR_PTR(PTR_ERR(host));
	}

	slot = sdhci_priv(host);

	slot->chip = chip;
	slot->host = host;

	host->hw_name = "SDIO";
	host->ops = &sdhci_tangox_ops;
	host->quirks = chip->quirks;

	host->irq = tangox_hosts[idx].irq;
	host->ioaddr = (void __iomem* )tangox_hosts[idx].base;

	//printk("sdhci_tangox_probe_slot: pad is set......!!");
	sdhci_writel(host, 0x00000008, 0x0100);

	if (chip->fixes && chip->fixes->probe_slot) {
		ret = chip->fixes->probe_slot(slot);
		if (ret)
			goto free;
	}

	ret = sdhci_add_host(host);
	if (ret)
		goto remove;

	
	return slot;

remove:
	if (chip->fixes && chip->fixes->remove_slot)
		chip->fixes->remove_slot(slot, 0);

free:
	sdhci_free_host(host);

	return ERR_PTR(ret);
}

static void sdhci_tangox_remove_slot(struct sdhci_tangox_slot *slot)
{
	int dead;
	u32 scratch;

	dead = 0;
	scratch = readl(slot->host->ioaddr + SDHCI_INT_STATUS);
	if (scratch == (u32)-1)
		dead = 1;

	sdhci_remove_host(slot->host, dead);

	if (slot->chip->fixes && slot->chip->fixes->remove_slot)
		slot->chip->fixes->remove_slot(slot, dead);

	sdhci_free_host(slot->host);
}

static int sdhci_tangox_probe(struct device *pdev)
{
	struct sdhci_tangox_chip *chip;
	struct sdhci_tangox_slot *slot;
	u8 slots;
	int ret, i;
#if defined(CONFIG_TANGO3) || defined(CONFIG_TANGO4)
	unsigned long tangox_chip_id(void);
	int tangox_sdio_enabled(int i);
	unsigned long tangox_get_cdclock(unsigned int cd);
	unsigned long chip_id = tangox_chip_id();
	unsigned int enabled_hosts = 0;
	unsigned int extra_quirks = 0;
	unsigned int infreq = tangox_get_cdclock(6) / 2000000; /* in MHz */

	if (
#ifdef CONFIG_TANGO3
		((chip_id & 0xfff00000) != 0x86400000) && ((chip_id & 0xfffe0000) != 0x86520000) && ((chip_id & 0xfff00000) != 0x86700000) && ((chip_id & 0xfff00000) != 0x86800000)
#endif
#ifdef CONFIG_TANGO4
		((chip_id & 0xfff00000) != 0x89100000)
#endif
		) {
		printk(KERN_ERR "SDIO is not supported.\n");
		return -EIO;
	} else {
		switch((chip_id & 0xfff00000) >> 16) {
#ifdef CONFIG_TANGO3
			case 0x8650: { /* only 8652 as all other 865x got filtered out prior */
					switch(chip_id & 0xff) { /* check revision */
						case 0x1:
							printk(KERN_ERR "SDIO is not supported.\n");
							return -EIO;
						default:
							extra_quirks |= SDHCI_QUIRK_BROKEN_ADMA;
							enabled_hosts = 3; /* 2 hosts, one bit per host */
							if (force_speed <= 0)
								extra_quirks |= SDHCI_QUIRK_NO_HISPD_BIT; /* extra quirks */
							break;
					}
				}
				break;
			case 0x8640: {
					enabled_hosts = 1; /* 1 host */
					extra_quirks |= SDHCI_QUIRK_BROKEN_ADMA;
					switch((chip_id & 0xfffe0000) >> 16) {
						case 0x8646:
							if (force_speed == 0)
								extra_quirks |= SDHCI_QUIRK_NO_HISPD_BIT; /* extra quirks */
							else
								gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x10c, 0x0fc00780); /* enable high speed mode */
							break;
						default:
							if (force_speed <= 0)
								extra_quirks |= SDHCI_QUIRK_NO_HISPD_BIT; /* extra quirks */
							break;
					}
				}
				break;
			case 0x8670: {
					enabled_hosts = 3; /* 2 hosts, one bit per host */
					switch((chip_id & 0xfffe0000) >> 16) {
						case 0x8670:
							extra_quirks |= SDHCI_QUIRK_BROKEN_ADMA;
							if (force_speed == 0)
								extra_quirks |= SDHCI_QUIRK_NO_HISPD_BIT; /* extra quirks */
							else {
								gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x128, 0x00400000); /* enable high speed mode */
								gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x128, 0x00400000); /* enable high speed mode */
							}
							break;
						case 0x8672:
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x100, 0x00000008); /* select SDIO from Eth1 pad */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x140, 0x247f8074 | (infreq << 7)); /* set up CAP_IN */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x144, 0x000c0008);
							/* ??? */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x128, 0x0004022c);

							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x100, 0x00000008); /* select SDIO */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x140, 0x247f8074 | (infreq << 7)); /* set up CAP_IN */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x144, 0x000c0008);
							/* ??? */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x128, 0x0004022c);

							if ((chip_id & 0xff) > 0x1) { /* ES2 or above */
								gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x12c, 0x00400002); /* falling edge clocking */
								gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x12c, 0x00400002); /* falling edge clocking */
							} else {
								gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x12c, 0x00000002);
								gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x12c, 0x00000002);
							}

							udelay(10); /* just to be sure ... */
							tangox_hosts[1].irq = IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_RTC_INT;

							if ((chip_id & 0xff) > 0x1) { /* ES2 or above */
								if (force_speed == 0)
									extra_quirks |= SDHCI_QUIRK_NO_HISPD_BIT; /* extra quirks */
							} else {
								if (force_speed <= 0)
									extra_quirks |= SDHCI_QUIRK_NO_HISPD_BIT; /* extra quirks */
							}
							break;
						case 0x8674:
							enabled_hosts = 1; /* 1 host */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x100, 0x00000008); /* select SDIO */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x140, 0x247f8074 | (infreq << 7)); /* set up CAP_IN */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x144, 0x000c0008);
							/* ??? */
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x128, 0x0004022c);
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x12c, 0x00000002);
							gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x150, 0x00400000); /* falling edge clocking */

							udelay(10); /* just to be sure ... */

							tangox_hosts[0].base = KSEG1ADDR(TANGOX_SDIO_BASE_ADDR1);
							if (force_speed == 0)
								extra_quirks |= SDHCI_QUIRK_NO_HISPD_BIT; /* extra quirks */
							break;
						default: /* Unknown 867x chip */
							printk(KERN_ERR "SDIO is not supported.\n");
							return -EIO;
					}
				}
				break;
			case 0x8680: {
					enabled_hosts = 3; /* 2 hosts, one bit per host */
					extra_quirks |= SDHCI_QUIRK_BROKEN_ADMA;
					if (force_speed == 0)
						extra_quirks |= SDHCI_QUIRK_NO_HISPD_BIT; /* extra quirks */
					else {
						gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x128, 0x00400000); /* enable high speed mode */
						gbus_write_reg32(TANGOX_SDIO_BASE_ADDR1 + 0x128, 0x00400000); /* enable high speed mode */
					}
				}
				break;
#endif
#ifdef CONFIG_TANGO4
			case 0x8910: {
					enabled_hosts = 1; /* 1 host */
					gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x100, 0x00000008); /* select SDIO from Eth1 pad */
					gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x140, 0x247f8074 | (infreq << 7)); /* set up CAP_IN */
					gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x144, 0x000c0008);
					/* ??? */
					gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x128, 0x0004022c);
					gbus_write_reg32(TANGOX_SDIO_BASE_ADDR + 0x12c, 0x00000002);

					udelay(10); /* just to be sure ... */

					if (force_speed <= 0)
						extra_quirks |= SDHCI_QUIRK_NO_HISPD_BIT; /* extra quirks */
				}
				break;
#endif
			default:
				printk(KERN_ERR "SDIO is not supported.\n");
				return -EIO;
		}
	}

	if (tangox_sdio_enabled(0) == 0)
		enabled_hosts &= ~1; /* host 0 is disabled */
	if (tangox_sdio_enabled(1) == 0)
		enabled_hosts &= ~2; /* host 1 is disabled */
	sdio_hosts &= enabled_hosts;
	if (sdio_hosts == 0) {
		printk(KERN_ERR "SDIO0/1 are disabled via XENV or module parameters.\n");
		return -EIO;
	} else {
		if (sdio_hosts & 1)
			printk(KERN_INFO "SDIO0 is enabled.\n");
		if (sdio_hosts & 2)
			printk(KERN_INFO "SDIO1 is enabled.\n");
	}
#endif
	BUG_ON(pdev == NULL);

	// TODO: hard coded 
	slots = MAX_SLOTS;

	BUG_ON(slots > MAX_SLOTS);

	chip = kzalloc(sizeof(struct sdhci_tangox_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto err;
	}

	chip->pdev = pdev;
	chip->fixes = (struct sdhci_tangox_fixes *)&sdhci_tangox;
	if (chip->fixes) {
		chip->fixes->quirks |= extra_quirks;
		chip->quirks = chip->fixes->quirks;
	}
	chip->num_slots = slots;

	dev_set_drvdata(pdev, chip);

	if (chip->fixes && chip->fixes->probe) {
		ret = chip->fixes->probe(chip);
		if (ret)
			goto free;
	}

	for (i = 0; i < slots; i++) {
		if (sdio_hosts & (i + 1)){
			slot = sdhci_tangox_probe_slot(pdev, chip, i);
			if (IS_ERR(slot)) {
				for (i--; i >= 0; i--)
					sdhci_tangox_remove_slot(chip->slots[i]);
				ret = PTR_ERR(slot);
				goto free;
			}

			chip->slots[i] = slot;
		}
		else
			chip->slots[i] = NULL;

	}

	return 0;

free:
	dev_set_drvdata(pdev, NULL);  /* clear private data ptr */
	kfree(chip);

err:
	// TODO: disabling device?
	return ret;
}

static int sdhci_tangox_remove(struct device *pdev)
{
	int i;
	struct sdhci_tangox_chip *chip;

	chip = dev_get_drvdata(pdev);

	if (chip) {
		for (i = 0; i < chip->num_slots; i++) {
			if (chip->slots[i]) 
				sdhci_tangox_remove_slot(chip->slots[i]);
		}
		dev_set_drvdata(pdev, NULL);  /* clear private data ptr */
		kfree(chip);
	}

	// TODO: disabling device?
	
	return 0;
}

static void tangox_sdio_release_dev(struct device * dev)
{
        dev->parent = NULL;
}

static struct device_driver sdhci_driver0 = {
	.name =		"sdhci0-tangox",
	.bus  =		&platform_bus_type,
	.probe = 	sdhci_tangox_probe,
	.remove =	sdhci_tangox_remove,
	.suspend =	sdhci_tangox_suspend,
	.resume	=	sdhci_tangox_resume,
};

static struct platform_device tangox_sdio_device0 = {
	.name = 	(char *)"sdhci0-tangox",
	.id = 		-1,
	.dev = {
		.release =	tangox_sdio_release_dev,
	},
	.num_resources  = 0,
	.resource       = 0,
};

static struct platform_device *tangox_platform_sdio0[] __initdata = {
        &tangox_sdio_device0,
};

/*****************************************************************************\
 *                                                                           *
 * Driver init/exit                                                          *
 *                                                                           *
\*****************************************************************************/

static int __init tangox_sdhci_drv_init(void)
{
	int status = 0;
  
	//printk("tangox_sdhci_drv_init in\n");
	/* Device 0 registration */
	status = platform_add_devices(tangox_platform_sdio0, ARRAY_SIZE(tangox_platform_sdio0));
	if (status) {
		printk(KERN_ERR "Failed to register device 0.\n");
		goto err_out;
	}
	/* Driver 0 registration*/
   	status = driver_register(&sdhci_driver0);
	if (status) {
		platform_device_unregister(&tangox_sdio_device0);
		printk(KERN_ERR "Failed to register driver 0.\n");
	}
	//printk("tangox_sdhci_drv_init out\n");
err_out:
	return status;
}

static void __exit tangox_sdhci_drv_exit(void)
{
	driver_unregister(&sdhci_driver0); 
	platform_device_unregister(&tangox_sdio_device0);
}

module_init(tangox_sdhci_drv_init);
module_exit(tangox_sdhci_drv_exit);
module_param(sdio_hosts,	uint,	0444);
module_param(force_speed,	uint,	0444);
MODULE_PARM_DESC(sdio_hosts,	"SDIO hosts being enabled");
MODULE_PARM_DESC(force_speed,	"-1: auto, 0: low, 1: high");
MODULE_AUTHOR("TANGOX standalone team");
MODULE_LICENSE("GPL");
MODULE_AUTHOR ("Sigma Designs Inc.");
MODULE_DESCRIPTION("TANGOX Secure Digital Host Controller Interface driver");

