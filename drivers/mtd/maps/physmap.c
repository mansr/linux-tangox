/*
 * $Id: physmap.c,v 1.39 2005/11/29 14:49:36 gleixner Exp $
 *
 * Normal mappings of chips in physical memory
 *
 * Copyright (C) 2003 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * 031022 - [jsun] add run-time configure and partition setup
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <asm/io.h>
#if defined(CONFIG_TANGOX) && defined(CONFIG_TANGOX_XENV_READ)

#ifdef CONFIG_TANGO2
#include <asm/tango2/emhwlib_registers_tango2.h>
#include <asm/tango2/tango2_gbus.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/emhwlib_registers_tango3.h>
#include <asm/tango3/tango3_gbus.h>
#endif

#define XENV_MAX_FLASH    4
#define XENV_MAX_FLASH_PARTITIONS   16
static struct mtd_info *mymtds[XENV_MAX_FLASH] = { NULL, NULL, NULL, NULL };
static struct mtd_partition *mtd_parts[XENV_MAX_FLASH] = { NULL, NULL, NULL, NULL };
static unsigned int p_cnts[XENV_MAX_FLASH] = { 0, 0, 0, 0 };
static unsigned int f_sizes[XENV_MAX_FLASH] = { 0, 0, 0, 0 };

struct map_info physmap_maps[XENV_MAX_FLASH] = {
        {
                .name = "CS0: Physically mapped flash",
                .phys = 0x40000000,
                .size = 0, /* To be filled by XENV */
                .bankwidth = 2, /* To be checked by PBI registers */
        },
        {
                .name = "CS1: Physically mapped flash",
                .phys = 0x44000000,
                .size = 0, /* To be filled by XENV */
                .bankwidth = 2, /* To be checked by PBI registers */
        },
        {
                .name = "CS2: Physically mapped flash",
                .phys = 0x48000000,
                .size = 0, /* To be filled by XENV */
                .bankwidth = 2, /* To be checked by PBI registers */
        },
        {
                .name = "CS3: Physically mapped flash",
                .phys = 0x4c000000,
                .size = 0, /* To be filled by XENV */
                .bankwidth = 2, /* To be checked by PBI registers */
        },
};
int tangox_flash_get_info(int cs, unsigned int *size, unsigned int *part_count);
int tangox_flash_get_parts(int cs, unsigned int offset[], unsigned int size[]);
#endif

struct physmap_flash_info {
	struct mtd_info		*mtd;
	struct map_info		map;
	struct resource		*res;
#ifdef CONFIG_MTD_PARTITIONS
	int			nr_parts;
	struct mtd_partition	*parts;
#endif
};


static int physmap_flash_remove(struct platform_device *dev)
{
#if defined(CONFIG_TANGOX) && defined(CONFIG_TANGOX_XENV_READ)
        int cs, p;
        struct mtd_partition *part_ptr;

        for (cs = 0; cs < XENV_MAX_FLASH; cs++) {
                if (f_sizes[cs] != 0) {
                        if (p_cnts[cs] != 0) {
                                for (part_ptr = mtd_parts[cs], p = 0; p < p_cnts[cs]; p++, part_ptr++) {
                                        if (part_ptr->name) {
                                                kfree(part_ptr->name);
                                                part_ptr->name = NULL;
                                        }
                                }
                                del_mtd_partitions(mymtds[cs]);
                                kfree(mtd_parts[cs]);
                                mtd_parts[cs] = NULL;
                        }
                        del_mtd_device(mymtds[cs]);
                        map_destroy(mymtds[cs]);
                        iounmap(physmap_maps[cs].virt);
                        physmap_maps[cs].virt = NULL;
                }
        }
#else
	struct physmap_flash_info *info;
	struct physmap_flash_data *physmap_data;

	info = platform_get_drvdata(dev);
	if (info == NULL)
		return 0;
	platform_set_drvdata(dev, NULL);

	physmap_data = dev->dev.platform_data;

	if (info->mtd != NULL) {
#ifdef CONFIG_MTD_PARTITIONS
		if (info->nr_parts) {
			del_mtd_partitions(info->mtd);
			kfree(info->parts);
		} else if (physmap_data->nr_parts) {
			del_mtd_partitions(info->mtd);
		} else {
			del_mtd_device(info->mtd);
		}
#else
		del_mtd_device(info->mtd);
#endif
		map_destroy(info->mtd);
	}

	if (info->map.virt != NULL)
		iounmap(info->map.virt);

	if (info->res != NULL) {
		release_resource(info->res);
		kfree(info->res);
	}
#endif
	return 0;
}

static const char *rom_probe_types[] = { "cfi_probe", "jedec_probe", "map_rom", NULL };

#ifndef CONFIG_TANGOX
#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probe_types[] = { "cmdlinepart", "RedBoot", NULL };
#endif
#endif

static int physmap_flash_probe(struct platform_device *dev)
{
	struct physmap_flash_data *physmap_data;
	struct physmap_flash_info *info;
	const char **probe_type;
	int err;

#if defined(CONFIG_TANGOX) && defined(CONFIG_TANGOX_XENV_READ)
        int cs;
        int part_num = 0;
        unsigned long csconfig = gbus_read_reg32(REG_BASE_host_interface + PB_CS_config) & 0xf;

        for (cs = 0; cs < XENV_MAX_FLASH; cs++) {

                /* Check XENV for availability */
                f_sizes[cs] = p_cnts[cs] = 0;

                tangox_flash_get_info(cs, &f_sizes[cs], &p_cnts[cs]);
                if (f_sizes[cs] == 0)
                        continue;
                else {
                        physmap_maps[cs].size = f_sizes[cs];
                        physmap_maps[cs].bankwidth = ((csconfig >> cs) & 0x1) ? 1 : 2;
                }

                printk(KERN_NOTICE "physmap flash device CS%d: 0x%x at 0x%x\n",
                                cs, (u32)physmap_maps[cs].size, (u32)physmap_maps[cs].phys);
                physmap_maps[cs].virt = ioremap(physmap_maps[cs].phys, physmap_maps[cs].size);

                if (!physmap_maps[cs].virt) {
                        printk("Failed to ioremap\n");
                        continue;
                }

                simple_map_init(&physmap_maps[cs]);

                mymtds[cs] = NULL;
                probe_type = rom_probe_types;
                for(; !mymtds[cs] && *probe_type; probe_type++) {
                        mymtds[cs] = do_map_probe(*probe_type, &physmap_maps[cs]);
                }

                if (mymtds[cs] && (mymtds[cs]->size != f_sizes[cs])) {
                        /* Redo ioremap if size specified is not the same as detected */
                        iounmap((void *)physmap_maps[cs].virt);
                        physmap_maps[cs].size = mymtds[cs]->size;
                        physmap_maps[cs].virt = ioremap(physmap_maps[cs].phys, physmap_maps[cs].size);

                        if (!physmap_maps[cs].virt) {
                                printk(KERN_NOTICE "Failed to ioremap at 0x%08x, size 0x%08x\n",
                                                (u32)physmap_maps[cs].phys, (u32)physmap_maps[cs].size);
                                continue;
                        }
                        printk(KERN_NOTICE "CS%d: flash size mismatched, re-do probing/initialization.\n", cs);
                        printk(KERN_NOTICE "physmap flash device CS%d: 0x%x at 0x%x (remapped 0x%x)\n",
                                        cs, (u32)physmap_maps[cs].size, (u32)physmap_maps[cs].phys, (u32)physmap_maps[cs].virt);

                        /* Re-do initialization */
                        simple_map_init(&physmap_maps[cs]);
                        mymtds[cs] = NULL;
                        probe_type = rom_probe_types;
                        for(; !mymtds[cs] && *probe_type; probe_type++) {
                                mymtds[cs] = do_map_probe(*probe_type, &physmap_maps[cs]);
                        }
                }

                if (mymtds[cs]) {
                        mymtds[cs]->owner = THIS_MODULE;
                        add_mtd_device(mymtds[cs]);
                        part_num++;

#ifdef CONFIG_MTD_PARTITIONS
                        if (p_cnts[cs] > 0) {
                                int p, pcnt;
                                struct mtd_partition *part_ptr;
                                unsigned int offsets[XENV_MAX_FLASH_PARTITIONS];
                                unsigned int szs[XENV_MAX_FLASH_PARTITIONS];

                                if ((mtd_parts[cs] = (struct mtd_partition *)kmalloc(
                                                sizeof(struct mtd_partition) * p_cnts[cs], GFP_KERNEL)) == NULL) {
                                        printk(KERN_NOTICE "Out of memory.\n");
                                        return -ENOMEM;
                                }
                                memset(mtd_parts[cs], 0, sizeof(struct mtd_partition) * p_cnts[cs]);
                                tangox_flash_get_parts(cs, offsets, szs);

                                printk(KERN_NOTICE "Using physmap partition definition\n");

                                /* Initialize each partition */
                                for (pcnt = 0, part_ptr = mtd_parts[cs], p = 0; p < p_cnts[cs]; p++) {
                                        if (((szs[p] & 0x7fffffff) + offsets[p]) > physmap_maps[cs].size) {
                                                printk(KERN_NOTICE "CS%d-Part%d (offset:0x%x, size:0x%x) outside physical map, removed.\n",
                                                                cs, p + 1, offsets[p], szs[p] & 0x7fffffff);
                                                        continue;
                                        }
                                        part_ptr->size = szs[p] & 0x7fffffff;
                                        part_ptr->offset = offsets[p];
                                        if (part_ptr->size & 0x80000000)
                                                part_ptr->mask_flags = MTD_WRITEABLE;
                                        part_ptr->name = (char *)kmalloc(16, GFP_KERNEL);
                                        if (part_ptr->name != NULL)
                                                sprintf(part_ptr->name, "CS%d-Part%d", cs, p + 1);
                                        pcnt++;
                                        part_ptr++;
                                }
                                p_cnts[cs] = pcnt;

                                if (p_cnts[cs] > 0) {
                                        printk(KERN_NOTICE "Adding partition #%d-#%d\n", part_num, part_num + p_cnts[cs] - 1);
                                        add_mtd_partitions(mymtds[cs], mtd_parts[cs], p_cnts[cs]);
                                        part_num += p_cnts[cs];
                                }
                        }
#endif /* CONFIG_MTD_PARTITIONS */
                }
        }
        return 0;
#else
	physmap_data = dev->dev.platform_data;
	if (physmap_data == NULL)
		return -ENODEV;

       	printk(KERN_NOTICE "physmap platform flash device: %.8llx at %.8llx\n",
	    (unsigned long long)(dev->resource->end - dev->resource->start + 1),
	    (unsigned long long)dev->resource->start);

	info = kzalloc(sizeof(struct physmap_flash_info), GFP_KERNEL);
	if (info == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	platform_set_drvdata(dev, info);

	info->res = request_mem_region(dev->resource->start,
			dev->resource->end - dev->resource->start + 1,
			dev->dev.bus_id);
	if (info->res == NULL) {
		dev_err(&dev->dev, "Could not reserve memory region\n");
		err = -ENOMEM;
		goto err_out;
	}

	info->map.name = dev->dev.bus_id;
	info->map.phys = dev->resource->start;
	info->map.size = dev->resource->end - dev->resource->start + 1;
	info->map.bankwidth = physmap_data->width;
	info->map.set_vpp = physmap_data->set_vpp;

	info->map.virt = ioremap(info->map.phys, info->map.size);
	if (info->map.virt == NULL) {
		dev_err(&dev->dev, "Failed to ioremap flash region\n");
		err = EIO;
		goto err_out;
	}

	simple_map_init(&info->map);

	probe_type = rom_probe_types;
	for (; info->mtd == NULL && *probe_type != NULL; probe_type++)
		info->mtd = do_map_probe(*probe_type, &info->map);
	if (info->mtd == NULL) {
		dev_err(&dev->dev, "map_probe failed\n");
		err = -ENXIO;
		goto err_out;
	}
	info->mtd->owner = THIS_MODULE;

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(info->mtd, part_probe_types, &info->parts, 0);
	if (err > 0) {
		add_mtd_partitions(info->mtd, info->parts, err);
		return 0;
	}

	if (physmap_data->nr_parts) {
		printk(KERN_NOTICE "Using physmap partition information\n");
		add_mtd_partitions(info->mtd, physmap_data->parts,
						physmap_data->nr_parts);
		return 0;
	}
#endif

	add_mtd_device(info->mtd);
	return 0;

err_out:
	physmap_flash_remove(dev);
	return err;
#endif /* CONFIG_TANGOX && CONFIG_TANGOX_XENV_READ */
}

#ifdef CONFIG_PM
static int physmap_flash_suspend(struct platform_device *dev, pm_message_t state)
{
	struct physmap_flash_info *info = platform_get_drvdata(dev);
	int ret = 0;

	if (info)
		ret = info->mtd->suspend(info->mtd);

	return ret;
}

static int physmap_flash_resume(struct platform_device *dev)
{
	struct physmap_flash_info *info = platform_get_drvdata(dev);
	if (info)
		info->mtd->resume(info->mtd);
	return 0;
}

static void physmap_flash_shutdown(struct platform_device *dev)
{
	struct physmap_flash_info *info = platform_get_drvdata(dev);
	if (info && info->mtd->suspend(info->mtd) == 0)
		info->mtd->resume(info->mtd);
}
#endif

static struct platform_driver physmap_flash_driver = {
	.probe		= physmap_flash_probe,
	.remove		= physmap_flash_remove,
#if  defined(CONFIG_PM) && !defined(CONFIG_TANGOX)  
	.suspend	= physmap_flash_suspend,
	.resume		= physmap_flash_resume,
	.shutdown	= physmap_flash_shutdown,
#endif
	.driver		= {
		.name	= "physmap-flash",
	},
};


#ifdef CONFIG_MTD_PHYSMAP_LEN
#if CONFIG_MTD_PHYSMAP_LEN != 0
#warning using PHYSMAP compat code
#define PHYSMAP_COMPAT
#endif
#endif

#ifdef PHYSMAP_COMPAT
static struct physmap_flash_data physmap_flash_data = {
	.width		= CONFIG_MTD_PHYSMAP_BANKWIDTH,
};

static struct resource physmap_flash_resource = {
	.start		= CONFIG_MTD_PHYSMAP_START,
	.end		= CONFIG_MTD_PHYSMAP_START + CONFIG_MTD_PHYSMAP_LEN - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device physmap_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &physmap_flash_data,
	},
	.num_resources	= 1,
	.resource	= &physmap_flash_resource,
};

void physmap_configure(unsigned long addr, unsigned long size,
		int bankwidth, void (*set_vpp)(struct map_info *, int))
{
	physmap_flash_resource.start = addr;
	physmap_flash_resource.end = addr + size - 1;
	physmap_flash_data.width = bankwidth;
	physmap_flash_data.set_vpp = set_vpp;
}

#ifdef CONFIG_MTD_PARTITIONS
void physmap_set_partitions(struct mtd_partition *parts, int num_parts)
{
	physmap_flash_data.nr_parts = num_parts;
	physmap_flash_data.parts = parts;
}
#endif
#endif

static int __init physmap_init(void)
{
	int err;

	err = platform_driver_register(&physmap_flash_driver);
#ifdef PHYSMAP_COMPAT
	if (err == 0)
		platform_device_register(&physmap_flash);
#endif

#ifdef CONFIG_TANGOX
	/* a hack to force probing here */
	err = physmap_flash_probe(NULL);
#endif

	return err;
}

static void __exit physmap_exit(void)
{
#ifdef CONFIG_TANGOX
	physmap_flash_remove(NULL);
#endif

#ifdef PHYSMAP_COMPAT
	platform_device_unregister(&physmap_flash);
#endif
	platform_driver_unregister(&physmap_flash_driver);
}

module_init(physmap_init);
module_exit(physmap_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_DESCRIPTION("Generic configurable MTD map driver");
