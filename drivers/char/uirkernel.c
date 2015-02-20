/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <linux/version.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <linux/module.h>
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/hardware.h>
#include <asm/tango3/uir.h>

#define DRIVER_VERSION	"0.11"

/* The major device number and name */
#define UIR_DEV_MAJOR	0	/* May need to be changed?? */

#define DEF_BUF_SIZE	(512+4)	/* Default buffer size */
#define DEF_QUEUE_SIZE	16	/* Default queue size */

MODULE_DESCRIPTION("TANGOX uir driver\n");
MODULE_AUTHOR("TANGOX team");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

#define UIR_DEV_NAME		"uir"

#define NONE			0
#define MEM1			1
#define MEM2			2
#define MEM1ANDMEM2		3
#define MEM2ANDMEM1		4
#define MEM_SEL			0x80
#define MEM1_PARTIAL		0x00100000
#define MEM2_PARTIAL		0x00200000
#define MEM1_VALID		0x00400000
#define MEM2_VALID		0x00800000
#define MEM1_LEN		0x000000ff
#define MEM2_LEN		0x00ff0000
#define MEM1_FULL		0x02000000
#define MEM2_FULL		0x01000000

/* Register offsets */
#define IR_UNIV_DEBOUNCE_DENOISE	0x00
#define IR_UNIV_CONFIG			0x04
#define IR_UNIV_CLK_DIV			0x08
#define IR_UNIV_TIMEOUT			0x0c
#define IR_UNIV_SUBSTRACT		0x10
#define IR_UNIV_MEM_ADDR		0x14
#define IR_UNIV_STAMP_CLK_DIV		0x18
#define IR_UNIV_MEM1_TIME_STAMP		0x1c
#define IR_UNIV_MEM2_TIME_STAMP		0x20

/* How many interfaces? */
#define NUM_UIR				2

// #define DEBUG_UIR

#ifdef DEBUG_UIR
#define DBG_PRINT	printk
#else
static void inline DBG_PRINT(const char *x, ...) { ; }
#endif

/* Some prototypes */
static int uir_open(struct inode *, struct file *);
static int uir_release(struct inode *, struct file *);
static int uir_read(struct file *, char *, size_t, loff_t *);
static int uir_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
static unsigned int uir_poll(struct file *, struct poll_table_struct *);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
static irqreturn_t uir_isr(int irq, void *dev_id);
#else
static irqreturn_t uir_isr(int irq, void *dev_id, struct pt_regs *regs);
#endif

struct uir_private {
	unsigned int ref_cnt;		/* Reference count */
	spinlock_t lock;		/* Spin lock */
	wait_queue_head_t uir_wq;	/* Wait queue */
	unsigned int regbase;		/* register base address */
	unsigned int membase;		/* memory base address */
	int irq;			/* IRQ */
	unsigned int *data;		/* Data */
	unsigned int p_idx;		/* Index of producer */
	unsigned int c_idx; 		/* Index of consumer */
	char devname[16];
};

static struct file_operations uir_fops = {
	open: uir_open,
	read: uir_read,
	ioctl: uir_ioctl,
	poll: uir_poll,
	release: uir_release,
	owner: THIS_MODULE,
};

/* Global data */
static const unsigned int uir_reg_base[NUM_UIR] = { REG_BASE_system_block + 0x800, REG_BASE_system_block + 0x880 };
static const unsigned int uir_mem_base[NUM_UIR] = { REG_BASE_system_block + 0x1000, REG_BASE_system_block + 0x1400 };
static struct uir_private *uir_privs[NUM_UIR];
static int major_num = UIR_DEV_MAJOR;
static const char *uir_devname = UIR_DEV_NAME;
static int uir_irq = LOG2_CPU_INFRARED_INT + IRQ_CONTROLLER_IRQ_BASE;
static int bufsize = DEF_BUF_SIZE; /* or above DEF_BUF_SIZE */
static int queuesize = DEF_QUEUE_SIZE; /* 1 or more */
static int enabled_uir = NUM_UIR; /* 1 or 2 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
module_param(major_num, int, 0);
module_param(bufsize, int, 0);
module_param(queuesize, int, 0);
module_param(enabled_uir, int, 0);
#else
MODULE_PARM(major_num, "i");
MODULE_PARM(bufsize, "i");
MODULE_PARM(queuesize, "i");
MODULE_PARM(enabled_uir, "i");
#endif

int __init uir_init_module(void);
void __exit uir_cleanup_module(void);

static int check_status(struct uir_private *priv, unsigned int status);
static int read_mem1(struct uir_private *priv, unsigned int status, unsigned int *buf);
static int read_mem2(struct uir_private *priv, unsigned int status, unsigned int *buf);
static int read_both_mem(struct uir_private *priv, unsigned int status, unsigned int *buf, int flags);

/* Put scancodes into buffer */
static void uir_queue_scancodes(struct uir_private *priv, unsigned int status)
{
	unsigned int pidx, *dptr, t_len;
	int m_flag;

	spin_lock(&priv->lock);

	pidx = priv->p_idx;	/* Save the old index before proceeding */
	dptr = &priv->data[priv->p_idx * bufsize];

	m_flag = check_status(priv, status);

	/* Save to buffer */
	switch(m_flag) {
		case MEM1:
			DBG_PRINT("Reading from MEM1 ..\n");
			t_len = read_mem1(priv, status, dptr + 4);
			break;
		case MEM2:
			DBG_PRINT("Reading from MEM1 ..\n");
			t_len = read_mem2(priv, status, dptr + 4);
			break;
		case MEM2ANDMEM1:
			DBG_PRINT("Reading from MEM2/1 ..\n");
			t_len = read_both_mem(priv, status, dptr + 4, m_flag);
			break;
		case MEM1ANDMEM2:
			DBG_PRINT("Reading from MEM1/2 ..\n");
			t_len = read_both_mem(priv, status, dptr + 4, m_flag);
			break;
		default:
			goto out;
			break;
	}

	/* Save the timestamps in the header */
	*(dptr + 0) = (unsigned int)(t_len + 4);	/* total length (# of unsigned int) */
	*(dptr + 1) = (unsigned int)((jiffies * 1000) / HZ);	/* kernel time, in msec */
	*(dptr + 2) = gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);	/* xtal value */
	*(dptr + 3) = 0; 				/* reserved */

	/* Adjust index */
	priv->p_idx = (priv->p_idx + 1) % queuesize;

	/* Buffer was empty, wake up the reader */
	if (priv->c_idx == pidx) 
		wake_up_interruptible(&priv->uir_wq);

out:
	spin_unlock(&priv->lock);
}

/* ISR for UIR device */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
static irqreturn_t uir_isr(int irq, void *dev_id)
#else
static irqreturn_t uir_isr(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
	struct uir_private *priv = (struct uir_private *)dev_id;
	unsigned int status;

	if (irq != priv->irq)
		return IRQ_NONE;

	status = gbus_read_reg32(priv->regbase + IR_UNIV_CONFIG);

	/* Check if the device is enabled */
	if (((status & 1) == 0) || ((status & 0xe0000000) == 0))
		return IRQ_NONE;

	DBG_PRINT("%s: interrupting ... 0x%08x\n", priv->devname, status);

	/* Clear interrupt flags first */
	gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, (status & 0xe0ffffff));

	if (status & 0x40000000)
		printk(KERN_ERR "%s: error occurred (0x%08x)\n", priv->devname, status);
	if (status & 0x20000000)
		printk(KERN_ERR "%s: overflow occurred (0x%08x)\n", priv->devname, status);
	if (status & 0x80000000) 
		uir_queue_scancodes(priv, status);

	return IRQ_HANDLED;
}

/* Opening UIR device */
static int uir_open(struct inode *inode_ptr, struct file *fptr)
{
	int minor = MINOR(inode_ptr->i_rdev);
	struct uir_private *priv = NULL;
	unsigned int status;

	if (minor >= enabled_uir)
		return(-EIO);
	else
		priv = uir_privs[minor];

	if ((fptr->f_flags & O_ACCMODE) != O_RDONLY) {
		printk(KERN_WARNING "%s: read-only device\n", priv->devname);
		return(-EINVAL);
	} 

	/* This device is exclusive, that is, only one process can use it */
	if (priv->ref_cnt != 0) {
		printk(KERN_WARNING "%s: exclusive access only\n", priv->devname);
		return(-EIO);
	} 

	/* increase reference count */
	priv->ref_cnt++;
	
	fptr->f_op = &uir_fops;
	fptr->private_data = (void *)priv;

	/* Flush data and pending interrupts (if any), enabling the device */
	status = gbus_read_reg32(priv->regbase + IR_UNIV_CONFIG);
	gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, status | 0xff00e001);

	return(0);
}

/* Closing UIR device */
static int uir_release(struct inode *inode_ptr, struct file *fptr) 
{
	struct uir_private *priv = (struct uir_private *)fptr->private_data;
	unsigned long flags;
	unsigned int status = gbus_read_reg32(priv->regbase + IR_UNIV_CONFIG);

	spin_lock_irqsave(&priv->lock, flags);

	/* Adjust reference count */
	priv->ref_cnt--;

	/* Disable device */
	gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, (status & ~0xe001) | 0xff000000);
	priv->p_idx = priv->c_idx = 0;

	spin_unlock_irqrestore(&priv->lock, flags);

	memset(priv->data, 0, sizeof(unsigned int) * bufsize * queuesize);

	return(0);
}

static int check_status(struct uir_private *priv, unsigned int status)
{
	unsigned int mem1_Tstmp, mem2_Tstmp;

	if ((status & MEM1_VALID) && (status & MEM2_VALID)) {
		if (status & MEM1_PARTIAL)
			return MEM2ANDMEM1; /* read mem2 first then mem1 */
		if (status & MEM2_PARTIAL)
			return MEM1ANDMEM2; /* read mem1 first then mem2 */

		mem1_Tstmp = gbus_read_reg32(priv->regbase + IR_UNIV_MEM1_TIME_STAMP);
		mem2_Tstmp = gbus_read_reg32(priv->regbase + IR_UNIV_MEM2_TIME_STAMP);
		if ((mem1_Tstmp < mem2_Tstmp) && (status & MEM1_FULL))
			return MEM1;
		else 
			return (status & MEM2_FULL) ? MEM2 : NONE;
	}

	if ((status & MEM1_VALID) && ((status & MEM2_VALID) == 0))
		return MEM1;
	else {
		if ((status & MEM2_VALID) && ((status & MEM1_VALID) == 0))
			return MEM2;
		else
			return NONE; /* none of the buffers is ready */
	}
}

static int read_mem1(struct uir_private *priv, unsigned int status, unsigned int *buf)
{
	unsigned int len = 0, i;

	DBG_PRINT("subtract 0x%08x scancode 0x%08x\n", (unsigned int)gbus_read_reg32(priv->regbase + IR_UNIV_SUBSTRACT), 
			(unsigned int)gbus_read_reg32(priv->regbase + IR_UNIV_MEM1_TIME_STAMP));

	if (status & MEM1_VALID) {
		gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, status & 0x00ffff7f);
		len = gbus_read_reg32(priv->regbase + IR_UNIV_MEM_ADDR) & MEM1_LEN;
		for (i = 0; i < len; i++) 
			buf[i] = gbus_read_reg32(priv->membase + (i * 4));
		gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, status & 0x01ffffff);
	}

	return len;
}

static int read_mem2(struct uir_private *priv, unsigned int status, unsigned int *buf)
{
	unsigned int len = 0, i;

	DBG_PRINT("subtract 0x%08x scancode 0x%08x\n", (unsigned int)gbus_read_reg32(priv->regbase + IR_UNIV_SUBSTRACT), 
			(unsigned int)gbus_read_reg32(priv->regbase + IR_UNIV_MEM2_TIME_STAMP));

	if (status & MEM2_VALID) {
		gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, (status & 0x00ffffff) | 0x00000080);
		len = (gbus_read_reg32(priv->regbase + IR_UNIV_MEM_ADDR) & MEM2_LEN) >> 16;
		for (i = 0; i < len; i++) 
			buf[i] = gbus_read_reg32(priv->membase + (i * 4));
		gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, status & 0x02ffffff);
	}
		
	return len;
}

static int read_both_mem(struct uir_private *priv, unsigned int status, unsigned int *buf, int flags)
{
	unsigned int len = 0, t_len = 0, i;

	switch(flags) {
		case MEM2ANDMEM1:
			/* set to read mem2 */
			gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, (status & 0x00ffffff) | 0x00000080);
			len = (gbus_read_reg32(priv->regbase + IR_UNIV_MEM_ADDR) & MEM2_LEN) >> 16;
			for (i = 0; i < len; i++)
				buf[i] = gbus_read_reg32(priv->membase + (i * 4));
			/* set to read mem1 */
			t_len = len;
			gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, status & 0x00ffff7f);
			len = gbus_read_reg32(priv->regbase + IR_UNIV_MEM_ADDR) & MEM1_LEN;
			for (i = 0; i < len; i++)
				buf[i + t_len] = gbus_read_reg32(priv->membase + (i * 4));
			t_len += len;
			break;

		case MEM1ANDMEM2:
			/* set to read mem1 */
			gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, status & 0x00ffff7f);
			len = gbus_read_reg32(priv->regbase + IR_UNIV_MEM_ADDR) & MEM1_LEN;
			for (i = 0; i < len; i++)
				buf[i] = gbus_read_reg32(priv->membase + (i * 4));
			/* set to read mem2 */
			t_len = len;
			gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, (status & 0x00ffffff) | 0x00000080);
			len = (gbus_read_reg32(priv->regbase + IR_UNIV_MEM_ADDR) & MEM2_LEN) >> 16;
			for (i = 0; i < len; i++)
				buf[i + t_len] = gbus_read_reg32(priv->membase + (i * 4));
			t_len += len;
			break;

		default:
			break;
	}

	if (t_len)
		gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, (status & 0x00ffffff) | 0x03000000);

	return t_len;
}

static int uir_read(struct file *fptr, char *bufptr, size_t size, loff_t *fp)
{
	struct uir_private *priv = (struct uir_private *)fptr->private_data;
	unsigned long flags;
	unsigned int buf[bufsize];
	int count = 0, i;

	/* Check the alignment */
	if (size < (bufsize * sizeof(unsigned int))) {
		printk(KERN_WARNING "%s: read size too small %d vs. %d\n",
			priv->devname, size, bufsize * sizeof(unsigned int));
		return(-EIO);
	}

	/* Sleep when buffer is empty */
	wait_event_interruptible(priv->uir_wq, priv->c_idx != priv->p_idx);

	if (signal_pending(current))
		return(count);

	spin_lock_irqsave(&priv->lock, flags);
	memcpy(&buf[0], &priv->data[priv->c_idx * bufsize], bufsize * sizeof(unsigned int));
	memset(&priv->data[priv->c_idx * bufsize], 0, bufsize * sizeof(unsigned int));
	priv->c_idx = (priv->c_idx + 1) % queuesize;
	spin_unlock_irqrestore(&priv->lock, flags);
	
	/* Get the data to user */
	count = buf[0] * sizeof(unsigned int);

	for (i = 4; i < buf[0]; i++) /* skipping header */
		buf[i] /= 27; /* convert to usec, instead of clocks */

	if (count && copy_to_user(bufptr, (char *)&buf[0], count)) 
		return(-EFAULT);

#ifdef DEBUG_UIR
	for (i = 0; i < buf[0]; i++) {
		if ((i % 8) == 0)
			printk("\n0x%08x: ", i);
		printk("0x%08x ", buf[i]);
	}
	printk("\n");
#endif

	return(count);
}

static unsigned int uir_poll(struct file *fptr, struct poll_table_struct *ptable)
{
	struct uir_private *priv = (struct uir_private *)fptr->private_data;
	unsigned int mask = 0;
	unsigned long flags;
	
	poll_wait(fptr, &priv->uir_wq, ptable);

	spin_lock_irqsave(&priv->lock, flags);
	if (priv->c_idx != priv->p_idx)
		mask |= (POLLIN | POLLRDNORM);
	spin_unlock_irqrestore(&priv->lock, flags);

	return(mask);
}

static int uir_ioctl(struct inode *inode, struct file *fptr, unsigned int cmd, unsigned long arg)
{
	struct uir_private *priv = (struct uir_private *)fptr->private_data;
	unsigned long *ptr = (unsigned long *)arg;
	unsigned long flags, val;
	int ret = 0;

	spin_lock_irqsave(&priv->lock, flags);
	switch(cmd) {
		case UIR_IOCSETDENOISETIME:
			val = gbus_read_reg32(priv->regbase + IR_UNIV_DEBOUNCE_DENOISE) & 0x800000ff;
			val |= ((arg & 0x7fffff) << 8);
			gbus_write_reg32(priv->regbase + IR_UNIV_DEBOUNCE_DENOISE, val);
			break;
		case UIR_IOCGETDENOISETIME:
			val = (gbus_read_reg32(priv->regbase + IR_UNIV_DEBOUNCE_DENOISE) & 0x7fffff00) >> 8;
			if (copy_to_user(ptr, &val, sizeof(unsigned long)))
				ret = -EFAULT;
			break;
		case UIR_IOCSETTIMEOUT:
			gbus_write_reg32(priv->regbase + IR_UNIV_TIMEOUT, arg);
			break;
		case UIR_IOCGETTIMEOUT:
			val = gbus_read_reg32(priv->regbase + IR_UNIV_TIMEOUT);
			if (copy_to_user(ptr, &val, sizeof(unsigned long)))
				ret = -EFAULT;
			break;
		case UIR_IOCSETSUBSTRACT:
			gbus_write_reg32(priv->regbase + IR_UNIV_SUBSTRACT, arg);
			break;
		case UIR_IOCGETSUBSTRACT:
			val = gbus_read_reg32(priv->regbase + IR_UNIV_SUBSTRACT);
			if (copy_to_user(ptr, &val, sizeof(unsigned long)))
				ret = -EFAULT;
			break;
		default:
			ret = -EIO;
			break;
	}
	spin_unlock_irqrestore(&priv->lock, flags);
	return(ret);
}

/* Driver initialization */
int __init uir_init_module(void)
{
	int status = 0, i;
	struct uir_private *priv = NULL;

	/* Checking module parameters */
	if ((enabled_uir == 0) || (enabled_uir > NUM_UIR))
		return(-EINVAL);
	else if ((bufsize < DEF_BUF_SIZE) || (queuesize == 0))
		return(-EINVAL);

	for (i = 0; i < enabled_uir; i++) {
		if ((uir_privs[i] = kmalloc(sizeof(struct uir_private), GFP_KERNEL)) == NULL) {
			printk(KERN_ERR "%s%d: out of memory.\n", uir_devname, i);
			status = -ENOMEM;
			goto err1_out;
		}

		/* Initialize private data structure */
		memset(uir_privs[i], 0, sizeof(struct uir_private)); 

		if ((uir_privs[i]->data = kmalloc(sizeof(unsigned int) * bufsize * queuesize, GFP_KERNEL)) == NULL) {
			printk(KERN_ERR "%s%d: out of memory.\n", uir_devname, i);
			status = -ENOMEM;
			goto err1_out;
		} else
			memset(uir_privs[i]->data, 0, sizeof(unsigned int) * bufsize * queuesize);
	}

	/* Register device, and may be allocating major# */
	status = register_chrdev(major_num, uir_devname, &uir_fops);
	if (status < 0) {
		printk(KERN_ERR "%s: cannot get major number %d %d\n", uir_devname, status, major_num); 
		goto err1_out;
	} else if (major_num == 0)
		major_num = status;	/* Dynamic major# allocation */

	for (i = 0; i < enabled_uir; i++) {
		priv = uir_privs[i];
		priv->regbase = uir_reg_base[i];
		priv->membase = uir_mem_base[i];
		priv->irq = uir_irq;
		sprintf(priv->devname, "%s%d", uir_devname, i);

		spin_lock_init(&priv->lock);

		if (request_irq(priv->irq, uir_isr, 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
				IRQF_SHARED ,
#else
				SA_SHIRQ ,
#endif
				priv->devname, priv) != 0) {
			printk(KERN_ERR "%s: cannot register IRQ (%d)\n", priv->devname, priv->irq);
			status = -EIO;
			goto err2_out;
		} else

		init_waitqueue_head(&priv->uir_wq);

		/* Initialize H/W */
		/* 
		 * IR_DENOISE_CLK_SEL: 27MHz = 0 [31]
		 * IR_DENOISE_TIME: 7 clocks = 7 [30:8]
		 * IR_DENOISE_GPIO_SELECT: pin 12 = 0xc [7:4]
		 * IR_DENOISE_EABLE: yes = 1 [3]
		 * IR_DEBOUNCE_SELECT: 8 system clocks = 0 [2:1]
		 * IR_DEBOUNCE_ENABLE: yes = 1 [0]
		 */
		gbus_write_reg32(priv->regbase + IR_UNIV_DEBOUNCE_DENOISE, 0x000007c9);
		/* 
		 * IR_UNI_DATA_OUT_INT_ENABLE: no = 0 [15]
		 * IR_UNI_ERROR_INT_ENABLE: no = 0 [14]
		 * IR_UNI_SCANCODE_TIME_STAMP_INT_ENABLE: no = 0 [13]
		 * IR_UNI_MEM_RD_SELECT: 0 = 0 [7]
		 * IR_UNI_MEM_WRAP_MODE_ENABLE: no = 0 [6]
		 * IR_UNI_FIRST_EDGE_FALLING: yes = 1 [5]
		 * IR_UNI_FIRST_EDGE_RISING: yes = 1 [4]
		 * IR_UNI_TIME_STAMP_MODE: delta = 0 [3]
		 * IR_UNI_SAMPLE_CLK_SELECT: 27MHz = 0 [2]
		 * IR_UNI_SIGNAL_INVERT: no = 0 [1]
		 * IR_UNI_DECODE_ENABLE: no = 0 [0]
		 */
		gbus_write_reg32(priv->regbase + IR_UNIV_CONFIG, 0xff000030);
		gbus_write_reg32(priv->regbase + IR_UNIV_TIMEOUT, 0x00041eb0);
	}

	printk("%s: SMP86xx uir (%d:0) driver loaded (bufsize=%d, queuesize=%d, enabled_uir=%d)\n", 
			uir_devname, major_num, bufsize, queuesize, enabled_uir);
	return(0);

err2_out:
	for (i = 0; i < enabled_uir; i++) {
		if (uir_privs[i]->irq != 0) {
			free_irq(uir_privs[i]->irq, priv);	
		}
	}
	unregister_chrdev(major_num, uir_devname);	

err1_out:
	for (i = 0; i < enabled_uir; i++) {
		if (uir_privs[i] != NULL) {
			if (uir_privs[i]->data != NULL)
				kfree(uir_privs[i]->data);
			kfree(uir_privs[i]);
		}
	}

	return(status);
}

/* Unloading the driver */
void __exit uir_cleanup_module(void)
{
	int i;
	struct uir_private *priv = NULL;
	unsigned long flags;

	unregister_chrdev(major_num, uir_devname);

	for (i = 0; i < enabled_uir; i++) {
		priv = uir_privs[i];
		spin_lock_irqsave(&priv->lock, flags);
		free_irq(priv->irq, priv);
		spin_unlock_irqrestore(&priv->lock, flags);
		if (priv->data != NULL)
			kfree(priv->data);
		kfree(priv);
	}
	printk("%s: uir driver unloaded\n", uir_devname);
}

module_init(uir_init_module);
module_exit(uir_cleanup_module);

