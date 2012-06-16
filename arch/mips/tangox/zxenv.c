
/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc. 
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h>

#if defined(CONFIG_TANGO3)
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/hardware.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/tango4_gbus.h>
#include <asm/tango4/hardware.h>
#else
#error "Unsupported platform"
#endif

#include "xenv.h"

/* The major device number and name */
#define ZXENV_DEV_MAJOR		0
#define ZXENV_DEV_NAME		"zxenv"
#define DRIVER_VERSION		"1.0"

extern unsigned long tangox_zxenv[];

MODULE_DESCRIPTION("TANGOX zxenv driver\n");
MODULE_AUTHOR("TANGOX standalone team");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

/* Some prototypes */
static int zxenv_open(struct inode *, struct file *);
static int zxenv_release(struct inode *, struct file *);
static int zxenv_read(struct file *, char *, size_t, loff_t *);

static int major_num = ZXENV_DEV_MAJOR;
module_param(major_num, int, 0);

/* Global data */
static char *zxenv_devname = ZXENV_DEV_NAME;

static struct file_operations zxenv_fops = {
	open: zxenv_open,
	read: zxenv_read,
	release: zxenv_release,
	owner: THIS_MODULE,
};

static struct cdev cdev;
static struct class *zxenv_class;

extern int xenv_get(u32 *base, u32 size, char *recordname, void *dst, u32 *datasize);

/* Getting xenv */
int zxenv_get(char *recordname, void *dst, u32 *datasize)
{
	int xenv_size = *((int *)tangox_zxenv);
	int res = xenv_get((u32 *)&tangox_zxenv[0], xenv_size, recordname, dst, datasize);

	return (res < 0) ? -EIO : res;
}

EXPORT_SYMBOL(zxenv_get);

/* Reading data */
static int zxenv_read(struct file *fptr, char *bufptr, size_t size, loff_t *fp)
{
	int xenv_size = *((int *)tangox_zxenv);

	if (size < xenv_size)
		return -EIO;

	/* Get the data to user */
	if (copy_to_user(bufptr, (char *)&tangox_zxenv[0], xenv_size)) 
		return -EFAULT;

	return xenv_size;
}

/* Open the device */
static int zxenv_open(struct inode *inode_ptr, struct file *fptr)
{
	return 0;
}

/* Close the device */
static int zxenv_release(struct inode *inode_ptr, struct file *fptr) 
{
	return 0;
}

int __init zxenv_init_module(void)
{
	int status = 0;
	dev_t devid;

	/* Register device, and may be allocating major# */
	if (major_num) {
		devid = MKDEV(major_num, 0);
		status = register_chrdev_region(devid, 1, "zxenv");
	} else {
		status = alloc_chrdev_region(&devid, 0, 1, "zxenv");
		major_num = MAJOR(devid);
	}

	if (status < 0) {
		printk(KERN_ERR "%s: cannot get chrdev_region\n", zxenv_devname);
		return status;
	}

	cdev_init(&cdev, &zxenv_fops);
	cdev.owner = THIS_MODULE;
	cdev.ops = &zxenv_fops;
	if ((status = cdev_add(&cdev, devid, 1)) < 0) {
		printk(KERN_ERR "%s: cannot get major number\n", zxenv_devname); 
		unregister_chrdev_region(MKDEV(major_num, 0), 1);
		return status;
	}
	if (IS_ERR(zxenv_class = class_create(THIS_MODULE, "zxenv_device"))) {
		printk(KERN_ERR "%s: error creating zxenv_device class.\n", zxenv_devname);
		cdev_del(&cdev);
		unregister_chrdev_region(MKDEV(major_num, 0), 1);
		return -EIO;
	}
	if (device_create(zxenv_class, NULL, MKDEV(major_num, 0), NULL, "zxenv") == NULL) {
		printk(KERN_ERR "%s: error creating zxenv_device.\n", zxenv_devname);
		class_destroy(zxenv_class);
		cdev_del(&cdev);
		unregister_chrdev_region(MKDEV(major_num, 0), 1);
		return(-EIO);
	}

	printk(KERN_INFO "SMP86xx %s (%d:0): driver loaded.\n", zxenv_devname, major_num);
	return 0;
}

void __exit zxenv_cleanup_module(void)
{
	device_destroy(zxenv_class, MKDEV(major_num, 0));
	class_destroy(zxenv_class);
	cdev_del(&cdev);
	unregister_chrdev_region(MKDEV(major_num, 0), 1);

	printk(KERN_INFO "%s: driver unloaded\n", zxenv_devname);
}

module_init(zxenv_init_module);
module_exit(zxenv_cleanup_module);

