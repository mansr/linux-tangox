
/*********************************************************************
 Copyright (C) 2001-2009
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/**
  @file  bwinfo.c
  @brief  

  Implementing interfaces to collect and dump the information
  on the bandwidth usage. Both /dev/bwinfo and /proc/bwinfo
  interfaces are supported.

  @author YH Lin
  @date   2009-12-01
*/

#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <asm/ioctl.h>

#ifdef CONFIG_TANGO3
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3api.h>
#include <asm/tango3/tango3_gbus.h>
#elif defined(CONFIG_TANGO4)
#error TO BE IMPLEMENTED!!!
#else
#error Only Tango3/Tango4 is supported ...
#endif

#include "setup.h"

#define SYS_GARB_spy_addlo	0x170
#define SYS_GARB_spy_addhi	0x174
#define SYS_GARB_spy_cfg	0x178
#define SYS_GARB_spy_cnt	0x17c

#define ADDR_LO			0x80000000
#define ADDR_HI			0xffffffff

#define USE_ORDER		4			/* 2^4 = 16 pages */
#define CAPTURE_PERIOD		10 			/* every 10msec */
#define INITIAL_DELAY		(5*HZ)			/* initial delay */

struct sample
{
	unsigned int diff_xtal;	/* the difference in xtal_cnt for this sample */
	unsigned int config;	/* the spy_config used for this sample */
	unsigned int count;	/* the count */
	unsigned int pad;	/* padding, to be extended if needed */
};

static const char *bwinfo_devname = "bwinfo";
static unsigned int begin = 0, end = 0;
static unsigned int size = 0;
static unsigned int lo_xtal = 0, save_xtal = 0;
static struct sample *sample_buffer = NULL;
static struct timer_list capture_timer;
static int status = 0;
static int cap_jiffies = 0;
static spinlock_t slock;		/* Spin lock */

#define PROC_ROOT		"bwinfo"
#define PROC_LIST		"masters"
#define PROC_CTRL		"control"
#define PROC_STAT		"stat"
static struct proc_dir_entry *root = NULL, *list_ent = NULL, *ctrl_ent = NULL, *stat_ent = NULL;
static atomic_t active;

#define MAX_MASTERS		256
static unsigned int masters[MAX_MASTERS] = {	/* default master list */
	0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0xffffffff,
};
static u64 xtal_sum[MAX_MASTERS], cnt_sum[MAX_MASTERS];
static int master_idx = 0;

static int use_order = USE_ORDER;
module_param(use_order, int, 0);
static unsigned int addlo = ADDR_LO;
module_param(addlo, int, 0);
static unsigned int addhi = ADDR_HI;
module_param(addhi, int, 0);
static unsigned int cap_period = CAPTURE_PERIOD;
module_param(cap_period, int, 0);

static int bwinfo_open(struct inode *inode_ptr, struct file *fptr);
static int bwinfo_release(struct inode *inode_ptr, struct file *fptr);
static int bwinfo_read(struct file *fptr, char *bufptr, size_t size, loff_t *fp);
static long bwinfo_unlocked_ioctl(struct file *fptr, unsigned int cmd, unsigned long arg);

static struct file_operations bwinfo_fops = {
	open: bwinfo_open,
	read: bwinfo_read,
	unlocked_ioctl: bwinfo_unlocked_ioctl,
	release: bwinfo_release,
	owner: THIS_MODULE,
};

#define BWINFO_IOC_MAGIC	'B'
#define BWINFO_IOCGETCOUT	_IO(BWINFO_IOC_MAGIC, 0)
#define BWINFO_IOCGETLIST	_IO(BWINFO_IOC_MAGIC, 1)
#define BWINFO_IOCSETLIST	_IO(BWINFO_IOC_MAGIC, 2)
#define BWINFO_IOCSTOP		_IO(BWINFO_IOC_MAGIC, 3)
#define BWINFO_IOCSTART		_IO(BWINFO_IOC_MAGIC, 4)
#define BWINFO_IOCRESET		_IO(BWINFO_IOC_MAGIC, 5)

/* Reset collection -- to be restarted */
static inline void reset_collection(void)
{
	master_idx = 0;
	memset(xtal_sum, 0, sizeof(u64) * MAX_MASTERS);
	memset(cnt_sum, 0, sizeof(u64) * MAX_MASTERS);
	lo_xtal = save_xtal = gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
	begin = end;
	gbus_write_reg32(REG_BASE_system_block + SYS_GARB_spy_cfg, masters[master_idx]); 
	gbus_write_reg32(REG_BASE_system_block + SYS_GARB_spy_cnt, 0);
}

/* Called by system timer to collect bandwidth information */
static void bwinfo_capture(unsigned long data)
{
	unsigned int cfg, cnt;
	unsigned int diff_xtal;
	unsigned long flags;

	if (atomic_read(&active) == 0)
		goto next;

	spin_lock_irqsave(&slock, flags);

	lo_xtal = gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
	cnt = gbus_read_reg32(REG_BASE_system_block + SYS_GARB_spy_cnt);
	cfg = gbus_read_reg32(REG_BASE_system_block + SYS_GARB_spy_cfg);

	diff_xtal = (lo_xtal >= save_xtal) ? (lo_xtal - save_xtal) : ((0xffffffff - save_xtal) + lo_xtal + 1);

	sample_buffer[end].diff_xtal = diff_xtal;
	sample_buffer[end].config = cfg;
	sample_buffer[end].count = cnt;
	sample_buffer[end].pad = 0;

	xtal_sum[master_idx] += diff_xtal;
	cnt_sum[master_idx] += cnt;

	end = (end + 1) % size;	/* adjust ring buffer */
	if (end == begin) 
		begin = (begin + 1) % size;

	spin_unlock_irqrestore(&slock, flags);

	master_idx++;
	if (masters[master_idx] == 0xffffffff)
		master_idx = 0;	/* loop back to beginning */
	cfg = masters[master_idx];

	gbus_write_reg32(REG_BASE_system_block + SYS_GARB_spy_cfg, cfg);
	gbus_write_reg32(REG_BASE_system_block + SYS_GARB_spy_cnt, 0);

next:
	save_xtal = lo_xtal = gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
	capture_timer.expires = jiffies + ((cap_period * HZ) / 1000);
	add_timer(&capture_timer);
}

static int bwinfo_open(struct inode *inode_ptr, struct file *fptr)
{
	/* This device is read-only */
	if ((fptr->f_flags & O_ACCMODE) != O_RDONLY) {
		printk(KERN_WARNING "%s: read-only device\n", bwinfo_devname);
		return -EIO;
	} 

	return 0;
}

static int bwinfo_release(struct inode *inode_ptr, struct file *fptr) 
{
	/* nothing needs to be done here */
	return 0;
}

static long bwinfo_unlocked_ioctl(struct file *fptr, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;

	switch(cmd) {
		case BWINFO_IOCGETCOUT:	/* size of data collection available */
			{
				unsigned int b, e, sz;
				spin_lock_irqsave(&slock, flags);
				b = begin;
				e = end;
				spin_unlock_irqrestore(&slock, flags);
				sz = ((e >= b) ? (e - b) : ((size - b) + e)) * sizeof(struct sample);
				if (copy_to_user((void *)arg, &sz, sizeof(unsigned int)))
					goto efault_out;
			}
			break;
		case BWINFO_IOCGETLIST: /* get the list of masters */
			{
				unsigned int *ptr = (unsigned int *)arg;
				unsigned int i, tmp[MAX_MASTERS];
				spin_lock_irqsave(&slock, flags);
				memcpy(tmp, masters, sizeof(unsigned int) * MAX_MASTERS);
				spin_unlock_irqrestore(&slock, flags);
				for (i = 0; (tmp[i] != 0xffffffff) && (i < MAX_MASTERS); i++);
				if (i > 0) {
					if (copy_to_user(ptr, &i, sizeof(unsigned int))) 
						goto efault_out;
					else if (copy_to_user(ptr + 1, tmp, sizeof(unsigned int) * i)) 
						goto efault_out;
				}
			}
			break;
		case BWINFO_IOCSETLIST: /* set the list of masters */
			{
				unsigned int *ptr = (unsigned int *)arg;
				unsigned int i, tmp[MAX_MASTERS];
				memset(tmp, 0, sizeof(unsigned int) * MAX_MASTERS);
				if (copy_from_user(&i, ptr, sizeof(unsigned int))) 
					goto efault_out;
				else if ((i <= 0) || (i > MAX_MASTERS)) 
					goto einval_out;
				else if (copy_from_user(tmp, ptr + 1, sizeof(unsigned int) * i)) 
					goto efault_out;
				spin_lock_irqsave(&slock, flags);
				memcpy(masters, tmp, sizeof(unsigned int) * MAX_MASTERS);
				reset_collection();
				spin_unlock_irqrestore(&slock, flags);
			}
			break;
		case BWINFO_IOCSTOP: /*	stop collection */
			atomic_set(&active, 0);
			break;
		case BWINFO_IOCSTART: /* start collection */
			atomic_set(&active, 1);
			break;
		case BWINFO_IOCRESET: /* reset collection */
			spin_lock_irqsave(&slock, flags);
			reset_collection();
			spin_unlock_irqrestore(&slock, flags);
			break;
		default:
			goto einval_out;
			break;
	}

	return 0;

einval_out:
	return -EINVAL;

efault_out:
	return -EFAULT;
}

static int bwinfo_read(struct file *fptr, char *bufptr, size_t read_cnt, loff_t *fp)
{
	size_t sz;
	int transfer;
	unsigned int b, e;
	unsigned long flags;

	spin_lock_irqsave(&slock, flags);
	b = begin;
	e = end;
	spin_unlock_irqrestore(&slock, flags);

	if (read_cnt % sizeof(struct sample)) {
		printk("%s: read size %d not multiple of %d\n", bwinfo_devname, read_cnt, sizeof(struct sample));
		return -EIO;
	}
	
	sz = ((e >= b) ? (e - b) : ((size - b) + e)) * sizeof(struct sample);
	transfer = (int)((sz >= read_cnt) ? read_cnt : sz); /* find out what max transfer size is */
	if (transfer == 0)
		return 0;	/* nothing to read */

	if (e >= b) {
		if (copy_to_user(bufptr, &sample_buffer[b], transfer))
			return -EFAULT;
	} else {
		if (copy_to_user(bufptr, &sample_buffer[b], (size - b) * sizeof(struct sample)))
			return -EFAULT;
		if (copy_to_user(bufptr + ((size - b) * sizeof(struct sample)), sample_buffer, e * sizeof(struct sample)))
			return -EFAULT;
	}

	return transfer;
}

static const char *hex2string(unsigned int *hex, const char *src_ptr)
{
	unsigned int res = 0;
	const char *src = src_ptr;

	for (; isxdigit(*src); src++) {
		if (isdigit(*src))
			res = res * 16 + ((*src) - '0');
		else
			res = res * 16 + (__toupper(*src) - 'A' + 10);
	}
	*hex = res;
	return src;
}

static const char *dec2string(unsigned int *dec, const char *src_ptr)
{
	unsigned int res = 0;
	const char *src = src_ptr;

	for (; isdigit(*src); src++) {
		res = res * 10 + ((*src) - '0');
	}
	*dec = res;
	return src;
}

static int list_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int i, ret = 0;

	for (i = 0; (masters[i] != 0xffffffff) && (i < MAX_MASTERS); i++) {
		ret += sprintf(page + off + ret, "0x%x ", masters[i]);
	}
	ret += sprintf(page + off + ret, "\n");
	*eof = 1;
	return ret;
}

static int list_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	unsigned int tmp[MAX_MASTERS];
	int i;
	unsigned long cnt, flags;
	const char *ptr = buffer, *end = buffer + count;

	memset(tmp, 0, sizeof(unsigned int) * MAX_MASTERS);

	/* Parse the values */
	for (cnt = i = 0; (i < MAX_MASTERS) && (cnt < count); i++) {
		for (; (!isdigit(*ptr)) && (ptr < end); ptr++);  /* skip leading unwanted stuff */
		if (ptr >= end)
			break; /* no more */
		else if ((*ptr == '0') && (__toupper(*(ptr + 1)) == 'X')) 
			ptr = hex2string(&tmp[i], ptr + 2);
		else
			ptr = dec2string(&tmp[i], ptr);
	}
	if (i == 0)
		goto done;
	tmp[i] = 0xffffffff;
	spin_lock_irqsave(&slock, flags);
	memcpy(masters, tmp, sizeof(unsigned int) * MAX_MASTERS);
	reset_collection();
	spin_unlock_irqrestore(&slock, flags);

done:
	return count;
}

static int ctrl_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char buf[16];
	unsigned long flags;

	if (count > ARRAY_SIZE(buf) - 1)
		count = ARRAY_SIZE(buf) - 1;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (strncmp("start", buf, strlen("start")) == 0) {
		if (atomic_read(&active) == 0)
			atomic_set(&active, 1);
	} else if (strncmp("stop", buf, strlen("stop")) == 0) {
		if (atomic_read(&active) != 0)
			atomic_set(&active, 0);
	} else if (strncmp("reset", buf, strlen("reset")) == 0) {
		spin_lock_irqsave(&slock, flags);
		reset_collection();
		spin_unlock_irqrestore(&slock, flags);
	}

	return count;
}

static int stat_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int i, ret = 0;
	u64 tmp64;
	u32 tmp32;

	for (i = 0; (masters[i] != 0xffffffff) && (i < MAX_MASTERS); i++) {
		tmp64 = xtal_sum[i];
		do_div(tmp64, 270000);
		tmp32 = (u32)(tmp64 & 0xffffffff);
		ret += sprintf(page + off + ret, "master %d(0x%x): xtal_cnt 0x%llx (%d.%02d sec), count 0x%llx\n",
				i, masters[i], xtal_sum[i], tmp32 / 100, tmp32 % 100, cnt_sum[i]);
	}

	*eof = 1;
	return ret;
}

int __init bwinfo_init(void)
{
	int ret = 0;

	cap_jiffies = (((cap_period * HZ) / 1000) == 0) ? 1 : ((cap_period * HZ) / 1000);
	if (addlo > addhi)
		return -EINVAL;

	memset(xtal_sum, 0, sizeof(u64) * MAX_MASTERS);
	memset(cnt_sum, 0, sizeof(u64) * MAX_MASTERS);
	atomic_set(&active, 1);

	size = ((1 << use_order) * PAGE_SIZE) / sizeof(struct sample);
	status = register_chrdev(0, bwinfo_devname, &bwinfo_fops);
	if (status < 0) {
		printk(KERN_ERR "%s: cannot get major number\n", bwinfo_devname); 
		return status;
	} else  if ((sample_buffer = (struct sample *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, use_order)) == NULL) { 
		printk("%s: memory allocation error\n", bwinfo_devname);
		ret = -ENOMEM;
		goto err1;
	} else if ((root = proc_mkdir(PROC_ROOT, NULL)) == NULL) {
		ret = -EINVAL;
		goto err2;
	} 

	if ((list_ent = create_proc_entry(PROC_LIST, S_IRUGO | S_IWUGO, root)) == NULL) {
		ret = -EINVAL;
		goto err3;
	} else {
		list_ent->read_proc = list_read_proc;
		list_ent->write_proc = list_write_proc;
	}

	if ((ctrl_ent = create_proc_entry(PROC_CTRL, S_IWUGO, root)) == NULL) {
		ret = -EINVAL;
		goto err4;
	} else {
		ctrl_ent->write_proc = ctrl_write_proc;
	}

	if ((stat_ent = create_proc_entry(PROC_STAT, S_IRUGO, root)) == NULL) {
		ret = -EINVAL;
		goto err5;
	} else {
		stat_ent->read_proc = stat_read_proc;
	}

	spin_lock_init(&slock);

	lo_xtal = save_xtal = gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
	gbus_write_reg32(REG_BASE_system_block + SYS_GARB_spy_addlo, addlo);
	gbus_write_reg32(REG_BASE_system_block + SYS_GARB_spy_addhi, addhi);
	gbus_write_reg32(REG_BASE_system_block + SYS_GARB_spy_cfg, masters[master_idx]); 
	gbus_write_reg32(REG_BASE_system_block + SYS_GARB_spy_cnt, 0);

	printk("Enabled SMP86xx bandwidth collection, major#: %d, max. samples: %d\n", status, size - 1);
	printk("Range: 0x%08x-0x%08x, Capture time: %d msec\n", addlo, addhi, (cap_jiffies * 1000) / HZ);

	init_timer(&capture_timer);
	capture_timer.data = (unsigned long)sample_buffer;
	capture_timer.function = bwinfo_capture;
	capture_timer.expires = jiffies + INITIAL_DELAY;	/* the timer starts in one second */
	add_timer(&capture_timer);

	printk("Starting collection in %d seconds...\n", INITIAL_DELAY / HZ);

	return ret;

err5:
	remove_proc_entry(PROC_CTRL, root);

err4:
	remove_proc_entry(PROC_LIST, root);

err3:
	remove_proc_entry(PROC_ROOT, NULL);

err2:
	if (sample_buffer)
		free_pages((u32)sample_buffer, use_order);
err1: 
	unregister_chrdev(status, bwinfo_devname);

	return ret;
}

void __exit bwinfo_cleanup(void)
{
	del_timer_sync(&capture_timer);
	remove_proc_entry(PROC_STAT, root);
	remove_proc_entry(PROC_CTRL, root);
	remove_proc_entry(PROC_LIST, root);
	remove_proc_entry(PROC_ROOT, NULL);
	free_pages((u32)sample_buffer, use_order);
	unregister_chrdev(status, bwinfo_devname);
}

module_init(bwinfo_init);
module_exit(bwinfo_cleanup);

