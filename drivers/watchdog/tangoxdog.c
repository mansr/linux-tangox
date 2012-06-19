/*
 *	TangoXDog 0.2 Hardware Watchdog Device for SMP864x/SMP865x/SMP867x/SMP89xx
 *
 *	(c) Copyright 2009-2011 Sigma Designs, Inc.
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

#if defined(CONFIG_TANGO3)
#include <asm/tango3/tango3.h>
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/emhwlib_registers_tango3.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/tango4.h>
#include <asm/tango4/tango4_gbus.h>
#include <asm/tango4/emhwlib_registers_tango4.h>
#endif

#define PFX "tangoxdog: "

static int tangoxdog_alive;
static DEFINE_SPINLOCK(tangoxdog_lock);	/* Spin lock */

#define WATCHDOG_TIMEOUT 30		/* 30 sec default timeout */

static int nowayout = WATCHDOG_NOWAYOUT;
static unsigned int watchdog_timeout = WATCHDOG_TIMEOUT;

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=CONFIG_WATCHDOG_NOWAYOUT)");

module_param(watchdog_timeout, int, 0);
MODULE_PARM_DESC(watchdog_timeout, "Watchdog timeout (in seconds)");

static inline void tangoxdog_ping(void)
{
	/* reset WDT counter */
	gbus_write_reg32(REG_BASE_system_block + SYS_watchdog_counter, TANGOX_BASE_FREQUENCY * watchdog_timeout);
}

static void tangoxdog_start(void)
{
	/* Set WDT counter, and start counting */
	tangoxdog_ping();
	printk(KERN_INFO PFX "Started watchdog timer.\n");
}

static void tangoxdog_stop(void)
{
	/* Disabling WDT */
	gbus_write_reg32(REG_BASE_system_block + SYS_watchdog_counter, 0);
	printk(KERN_INFO PFX "Stopped watchdog timer.\n");
}

/*
 *	Allow only one person to hold it open
 */
static int tangoxdog_open(struct inode *inode, struct file *file)
{
	unsigned long flags;

	spin_lock_irqsave(&tangoxdog_lock, flags);

	if (tangoxdog_alive) {
		spin_unlock_irqrestore(&tangoxdog_lock, flags);
		return -EBUSY;
	}

	if (nowayout)
		__module_get(THIS_MODULE);

	/* Activate timer */
	tangoxdog_start();
	tangoxdog_alive = 1;

	spin_unlock_irqrestore(&tangoxdog_lock, flags);

	return nonseekable_open(inode, file);
}

static int tangoxdog_release(struct inode *inode, struct file *file)
{
	unsigned long flags;
	spin_lock_irqsave(&tangoxdog_lock, flags);

	/* Shut off the timer.
	 * Lock it in if it's a module and we defined ...NOWAYOUT */
	if (!nowayout)
		tangoxdog_stop();		/* Turn the WDT off */

	tangoxdog_alive = 0;

	spin_unlock_irqrestore(&tangoxdog_lock, flags);
	return 0;
}

static ssize_t tangoxdog_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
	unsigned long flags;
	spin_lock_irqsave(&tangoxdog_lock, flags);
	/* Refresh the timer. */
	if (len) 
		tangoxdog_ping();
	spin_unlock_irqrestore(&tangoxdog_lock, flags);
	return len;
}

static long tangoxdog_unlocked_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	int options, retval = -EINVAL, tval;
	struct watchdog_info tmp;
	static struct watchdog_info ident = {
		.options		= WDIOF_KEEPALIVEPING |
					  WDIOF_MAGICCLOSE,
		.firmware_version	= 0,
		.identity		= "Hardware Watchdog for SMP8xxx",
	};

	switch (cmd) {
		case WDIOC_GETSUPPORT:
			spin_lock_irqsave(&tangoxdog_lock, flags);
			memcpy(&tmp, &ident, sizeof(ident));
			spin_unlock_irqrestore(&tangoxdog_lock, flags);
			if (copy_to_user((struct watchdog_info *)arg, &tmp, sizeof(ident))) 
				retval = -EFAULT;
			else
				retval = 0;
			break;
		case WDIOC_GETSTATUS:
		case WDIOC_GETBOOTSTATUS:
			retval = put_user(0, (int *)arg);
			break;
		case WDIOC_KEEPALIVE:
			spin_lock_irqsave(&tangoxdog_lock, flags);
			tangoxdog_ping();
			spin_unlock_irqrestore(&tangoxdog_lock, flags);
			retval = 0;
			break;
		case WDIOC_GETTIMEOUT:
			spin_lock_irqsave(&tangoxdog_lock, flags);
			tval = watchdog_timeout;
			spin_unlock_irqrestore(&tangoxdog_lock, flags);
			retval = put_user(tval, (int *)arg);
			break;
		case WDIOC_SETTIMEOUT:
			if ((retval = get_user(tval, (int *)arg)) == 0) {
				spin_lock_irqsave(&tangoxdog_lock, flags);
				watchdog_timeout = tval;
				spin_unlock_irqrestore(&tangoxdog_lock, flags);
			}
			break;
		case WDIOC_SETOPTIONS:
		{
			if (get_user(options, (int *)arg)) {
				retval = -EFAULT;
				break;
			}

			if (options & WDIOS_DISABLECARD) {
				spin_lock_irqsave(&tangoxdog_lock, flags);
				tangoxdog_stop();
				spin_unlock_irqrestore(&tangoxdog_lock, flags);
				retval = 0;
			}

			if (options & WDIOS_ENABLECARD) {
				spin_lock_irqsave(&tangoxdog_lock, flags);
				tangoxdog_start();
				spin_unlock_irqrestore(&tangoxdog_lock, flags);
				retval = 0;
			}

			break;
		}
		default:
			retval = -ENOIOCTLCMD;
			break;
	}

	return retval;
}

static int tangoxdog_notify_sys(struct notifier_block *this, unsigned long code, void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT)
		tangoxdog_stop();		/* Turn the WDT off */

	return NOTIFY_DONE;
}

static struct file_operations tangoxdog_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= tangoxdog_write,
	.unlocked_ioctl	= tangoxdog_unlocked_ioctl,
	.open		= tangoxdog_open,
	.release	= tangoxdog_release,
};

static struct miscdevice tangoxdog_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &tangoxdog_fops,
};

static struct notifier_block tangoxdog_notifier = {
	.notifier_call = tangoxdog_notify_sys,
};

static char banner[] __initdata = KERN_INFO PFX "Hardware Watchdog Timer for SMP864x/SMP865x/SMP867x/SMP89xx 0.2 (def. timeout: %d sec)\n";

static int __init watchdog_init(void)
{
	int ret;

	ret = register_reboot_notifier(&tangoxdog_notifier);
	if (ret) {
		printk(KERN_ERR PFX "cannot register reboot notifier (err=%d)\n", ret);
		return ret;
	}

	ret = misc_register(&tangoxdog_miscdev);
	if (ret) {
		printk(KERN_ERR PFX "cannot register miscdev on minor=%d (err=%d)\n", WATCHDOG_MINOR, ret);
		unregister_reboot_notifier(&tangoxdog_notifier);
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
	misc_deregister(&tangoxdog_miscdev);
	unregister_reboot_notifier(&tangoxdog_notifier);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_DESCRIPTION("Hardware Watchdog Device for SMP8xxx");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);

