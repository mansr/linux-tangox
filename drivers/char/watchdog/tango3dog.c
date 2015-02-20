/*
 *	Tango3Dog	0.1	A Hardware Watchdog Device for SMP864x/SMP865x
 *
 *	(c) Copyright 2007 Sigma Designs, Inc.
 *
 *	based on indydog.c by Guido Guenther <agx@sigxcpu.org>
 *
 *	(c) Copyright 2002 Guido Guenther <agx@sigxcpu.org>, All Rights Reserved.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	based on softdog.c by Alan Cox <alan@redhat.com>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <asm/uaccess.h>

#include <asm/tango3/tango3.h>
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/emhwlib_registers_tango3.h>

#define PFX "tango3dog: "

static int tango3dog_alive;

#define WATCHDOG_TIMEOUT 30		/* 30 sec default timeout */

static int nowayout = WATCHDOG_NOWAYOUT;
static unsigned int watchdog_timeout = WATCHDOG_TIMEOUT;

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=CONFIG_WATCHDOG_NOWAYOUT)");

module_param(watchdog_timeout, int, 0);
MODULE_PARM_DESC(watchdog_timeout, "Watchdog timeout (in seconds)");

static void tango3dog_start(void)
{
	/* Set WDT counter, and start counting */
	gbus_write_reg32(REG_BASE_system_block + SYS_watchdog_counter, TANGOX_BASE_FREQUENCY * watchdog_timeout);
}

static void tango3dog_stop(void)
{
	/* Disabling WDT */
	gbus_write_reg32(REG_BASE_system_block + SYS_watchdog_counter, 0);
	printk(KERN_INFO PFX "Stopped watchdog timer.\n");
}

static void tango3dog_ping(void)
{
	/* reset WDT counter */
	gbus_write_reg32(REG_BASE_system_block + SYS_watchdog_counter, TANGOX_BASE_FREQUENCY * watchdog_timeout);
}

/*
 *	Allow only one person to hold it open
 */
static int tango3dog_open(struct inode *inode, struct file *file)
{
	if (tango3dog_alive)
		return -EBUSY;

	if (nowayout)
		__module_get(THIS_MODULE);

	/* Activate timer */
	tango3dog_start();
	tango3dog_ping();

	tango3dog_alive = 1;
	printk(KERN_INFO "Started watchdog timer.\n");

	return nonseekable_open(inode, file);
}

static int tango3dog_release(struct inode *inode, struct file *file)
{
	/* Shut off the timer.
	 * Lock it in if it's a module and we defined ...NOWAYOUT */
	if (!nowayout)
		tango3dog_stop();		/* Turn the WDT off */

	tango3dog_alive = 0;

	return 0;
}

static ssize_t tango3dog_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
	/* Refresh the timer. */
	if (len) 
		tango3dog_ping();
	return len;
}

static int tango3dog_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int options, retval = -EINVAL;
	static struct watchdog_info ident = {
		.options		= WDIOF_KEEPALIVEPING |
					  WDIOF_MAGICCLOSE,
		.firmware_version	= 0,
		.identity		= "Hardware Watchdog for SMP86xx",
	};

	switch (cmd) {
		case WDIOC_GETSUPPORT:
			if (copy_to_user((struct watchdog_info *)arg, &ident, sizeof(ident)))
				return -EFAULT;
			return 0;
		case WDIOC_GETSTATUS:
		case WDIOC_GETBOOTSTATUS:
			return put_user(0, (int *)arg);
		case WDIOC_KEEPALIVE:
			tango3dog_ping();
			return 0;
		case WDIOC_GETTIMEOUT:
			return put_user(watchdog_timeout, (int *)arg);
		case WDIOC_SETTIMEOUT:
			return get_user(watchdog_timeout, (int *)arg);
		case WDIOC_SETOPTIONS:
		{
			if (get_user(options, (int *)arg))
				return -EFAULT;

			if (options & WDIOS_DISABLECARD) {
				tango3dog_stop();
				retval = 0;
			}

			if (options & WDIOS_ENABLECARD) {
				tango3dog_start();
				retval = 0;
			}

			return retval;
		}
		default:
			return -ENOIOCTLCMD;
	}
}

static int tango3dog_notify_sys(struct notifier_block *this, unsigned long code, void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT)
		tango3dog_stop();		/* Turn the WDT off */

	return NOTIFY_DONE;
}

static struct file_operations tango3dog_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= tango3dog_write,
	.ioctl		= tango3dog_ioctl,
	.open		= tango3dog_open,
	.release	= tango3dog_release,
};

static struct miscdevice tango3dog_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &tango3dog_fops,
};

static struct notifier_block tango3dog_notifier = {
	.notifier_call = tango3dog_notify_sys,
};

static char banner[] __initdata = KERN_INFO PFX "Hardware Watchdog Timer for SMP864x/SMP865x 0.1 (def. timeout: %d sec)\n";

static int __init watchdog_init(void)
{
	int ret;

	ret = register_reboot_notifier(&tango3dog_notifier);
	if (ret) {
		printk(KERN_ERR PFX "cannot register reboot notifier (err=%d)\n", ret);
		return ret;
	}

	ret = misc_register(&tango3dog_miscdev);
	if (ret) {
		printk(KERN_ERR PFX "cannot register miscdev on minor=%d (err=%d)\n", WATCHDOG_MINOR, ret);
		unregister_reboot_notifier(&tango3dog_notifier);
		return ret;
	}

	printk(banner, watchdog_timeout);

	/* Clear counter and use XTAL_IN as source */
	gbus_write_reg32(REG_BASE_system_block + SYS_watchdog_counter, 0);
	gbus_write_reg8(REG_BASE_system_block + SYS_watchdog_configuration, 0x1); 

	return 0;
}

static void __exit watchdog_exit(void)
{
	misc_deregister(&tango3dog_miscdev);
	unregister_reboot_notifier(&tango3dog_notifier);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_DESCRIPTION("Hardware Watchdog Device for SMP86xx");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);

