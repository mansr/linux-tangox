
/*********************************************************************
 Copyright (C) 2001-2007
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
#include <asm/io.h>

#ifdef CONFIG_TANGO2
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/hardware.h>
#include <asm/tango2/ir.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/hardware.h>
#include <asm/tango3/ir.h>
#endif

/* For all SMP86xx, it's always there */
#if defined(CONFIG_TANGO2_SMP863X) || defined(CONFIG_TANGO3_SMP86XX)
#define WITH_RC5_CONTROL
#define WITH_RC6_CONTROL
#endif

//#define DEBUG_IR
//#define ALL_KEYS_REPEATABLE

#define SYS_gpio_base		SYS_gpio_dir

/* NEC Control */
#define IR_NEC_CTRL             (REG_BASE_system_block + SYS_gpio_base + 0x18)
#define IR_NEC_CAPTURE_DATA    	(REG_BASE_system_block + SYS_gpio_base + 0x1c)

#ifdef WITH_RC5_CONTROL
/* RC5 Control */
#define IR_RC5_CTRL             (REG_BASE_system_block + SYS_gpio_base + 0x20)
#define IR_RC5_DECODE_CLK_DIV  	(REG_BASE_system_block + SYS_gpio_base + 0x24)
#define IR_RC5_DECODER_DATA    	(REG_BASE_system_block + SYS_gpio_base + 0x28)
#define IR_RC5_INT_STATUS    	(REG_BASE_system_block + SYS_gpio_base + 0x2c)
#endif /* WITH_RC5_CONTROL */

#ifdef WITH_RC6_CONTROL
/* RC6 Control */
#define RC6_DWORDS		5	/* 5 dwords = 20 bytes */
#define IR_RC6_CTRL             (REG_BASE_system_block + SYS_gpio_base + 0xe0)
#define IR_RC6_T_CTRL           (REG_BASE_system_block + SYS_gpio_base + 0xe4)
#define IR_RC6_DATA_OUT0        (REG_BASE_system_block + SYS_gpio_base + 0xe8)
#define IR_RC6_DATA_OUT1        (REG_BASE_system_block + SYS_gpio_base + 0xec)
#define IR_RC6_DATA_OUT2        (REG_BASE_system_block + SYS_gpio_base + 0xf0)
#define IR_RC6_DATA_OUT3        (REG_BASE_system_block + SYS_gpio_base + 0xf4)
#define IR_RC6_DATA_OUT4        (REG_BASE_system_block + SYS_gpio_base + 0xf8)
#endif

/* The buffer size defines the size of circular buffer to keep the IR data */
#ifdef WITH_RC6_CONTROL
#define BUF_SIZE		6 	/* Minimum 20 bytes */
#else
#define BUF_SIZE		2
#endif

/* Wait period, to avoid bouncing? */
#define WAIT_PERIOD		100

/* Max. size of key table */
#define MAX_KEYS		32

/* The major device number and name */
#define IR_DEV_MAJOR		0
#define IR_DEV_NAME		"ir"

#define DRIVER_VERSION         "1.17"

MODULE_DESCRIPTION("TANGOX ir remote driver\n");
MODULE_AUTHOR("TANGOX standalone team");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

#ifdef WITH_RC5_CONTROL
static int rc5_clk_div = 48006;	/* 48006 = 1.778ms, 36018 = 1.334ms, */
				/* 59994 = 2.222ms */
module_param(rc5_clk_div, int, 0);
#endif /* WITH_RC5_CONTROL */

static int wp_var; /* Variable wait period */
static int wait_period = ((((WAIT_PERIOD * HZ) / 1000) == 0) ? 1 : ((WAIT_PERIOD * HZ) / 1000));
module_param(wait_period, int, 0);

static int buffer_size = BUF_SIZE;
module_param(buffer_size, int, 0);

static int max_keys = MAX_KEYS;
module_param(max_keys, int,0);

static int major_num = IR_DEV_MAJOR;
module_param(major_num, int, 0);

static int repeat_sends_zero = 0;
module_param(repeat_sends_zero, int, 0);

/* Wait queue, may be used if block mode is on */
DECLARE_WAIT_QUEUE_HEAD(ir_wq);

/* Private data structure */
struct ir_private {
	unsigned long *buffer;		/* Circular buffer */
	unsigned long *key_table;	/* Table for repetition keys */
	unsigned p_idx;			/* Index of producer */
	unsigned c_idx; 		/* Index of consumer */
	unsigned ref_cnt;		/* Reference count */
	spinlock_t lock;		/* Spin lock */
	unsigned char b_mode;		/* Blocking mode or not */
	unsigned long last_jiffies;	/* Timestamp for last reception */
	unsigned int num_keys;		/* Number of keys in the table */
};

/* Some prototypes */
static int ir_open(struct inode *, struct file *);
static int ir_release(struct inode *, struct file *);
static int ir_read(struct file *, char *, size_t, loff_t *);
static int ir_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
static unsigned int ir_poll(struct file *, struct poll_table_struct *);

/* Global data */
static struct ir_private ir_priv;
static char *ir_devname = IR_DEV_NAME;
static int ir_irq = LOG2_CPU_INFRARED_INT + IRQ_CONTROLLER_IRQ_BASE;

static struct file_operations ir_fops = {
	open: ir_open,
	read: ir_read,
	ioctl: ir_ioctl,
	poll: ir_poll,
	release: ir_release,
	owner: THIS_MODULE,
};

/* Check to see if we can find the key in the repetition key table */
static int ir_findkey(struct ir_private *priv, unsigned long key)
{
#ifdef ALL_KEYS_REPEATABLE
	return(key);
#else
	register unsigned int i;
	unsigned long *ptr = priv->key_table;

	for (i = 0; i < priv->num_keys; i++, ptr++)
		if (key == *ptr)
			return(key);
	return(0);
#endif
}

/* Produce data */
static void ir_produce(struct ir_private *priv, unsigned long status)
{
	static unsigned long old_key = 0;
	static unsigned long save_key = 0;
	unsigned long data = 0;
	unsigned pidx;
	int repeat_key = 0;
#ifdef WITH_RC6_CONTROL
	static unsigned long save_rc6_key[RC6_DWORDS];	/* Only used for RC6 */
#endif

	spin_lock(&priv->lock);

#ifdef WITH_RC6_CONTROL
	if ((status & 0x80000000) != 0) {	// RC6 Data in IRQ
		unsigned long dx[RC6_DWORDS];
		dx[0] = gbus_read_reg32(IR_RC6_DATA_OUT0);
		dx[1] = gbus_read_reg32(IR_RC6_DATA_OUT1);
		dx[2] = gbus_read_reg32(IR_RC6_DATA_OUT2);
		dx[3] = gbus_read_reg32(IR_RC6_DATA_OUT3);
		dx[4] = gbus_read_reg32(IR_RC6_DATA_OUT4);
#ifdef DEBUG_IR
		printk(KERN_DEBUG "D0-4: 0x%08lx 0x%08lx 0x%08lx 0x%08lx 0x%08lx\n", dx[0], 
				dx[1], dx[2], dx[3], dx[4]);
#endif
		if ((dx[0] & 0x1f) != 0x1c) { 
#ifdef DEBUG_IR
			printk(KERN_DEBUG "Not acceptable RC6 code 0x%08lx\n", dx[0]);
#endif
			goto out;	/* Not valid */
		} else if (time_after(priv->last_jiffies + wait_period, jiffies) && 
			(memcmp(&save_rc6_key, &dx, sizeof(unsigned long) * RC6_DWORDS) == 0)) {
			/* Throw away this key if this is the same key and came too
		   	   fast */
#ifdef DEBUG_IR
			printk(KERN_DEBUG "%s: same data\n", ir_devname);
#endif
			save_key = 0;
			goto out;
		} 

		/* Save the key */
		memcpy(&save_rc6_key, &dx, sizeof(unsigned long) * RC6_DWORDS);

		priv->last_jiffies = jiffies;
		pidx = priv->p_idx;	/* Save the old index before proceeding */

		/* Save it to buffer */
		if (((priv->p_idx + 1) % buffer_size) == priv->c_idx) {
			/* Adjust consumer index since buffer is full */
			/* Keep the latest one and drop the oldest one */
			priv->c_idx = (priv->c_idx + 1) % buffer_size;
			printk(KERN_WARNING "%s: buffer full\n", ir_devname);
		} else if (((priv->p_idx + 2) % buffer_size) == priv->c_idx) {
			/* Adjust consumer index since buffer is full */
			/* Keep the latest one and drop the oldest ones */
			priv->c_idx = (priv->c_idx + 2) % buffer_size;
			printk(KERN_WARNING "%s: buffer full\n", ir_devname);
		} else if (((priv->p_idx + 3) % buffer_size) == priv->c_idx) {
			/* Adjust consumer index since buffer is full */
			/* Keep the latest one and drop the oldest ones */
			priv->c_idx = (priv->c_idx + 3) % buffer_size;
			printk(KERN_WARNING "%s: buffer full\n", ir_devname);
		} else if (((priv->p_idx + 4) % buffer_size) == priv->c_idx) {
			/* Adjust consumer index since buffer is full */
			/* Keep the latest one and drop the oldest ones */
			priv->c_idx = (priv->c_idx + 4) % buffer_size;
			printk(KERN_WARNING "%s: buffer full\n", ir_devname);
		} else if (((priv->p_idx + 5) % buffer_size) == priv->c_idx) {
			/* Adjust consumer index since buffer is full */
			/* Keep the latest one and drop the oldest ones */
			priv->c_idx = (priv->c_idx + 5) % buffer_size;
			printk(KERN_WARNING "%s: buffer full\n", ir_devname);
		}

		priv->buffer[priv->p_idx] = dx[0];
		priv->p_idx = (priv->p_idx + 1) % buffer_size;
		priv->buffer[priv->p_idx] = dx[1];
		priv->p_idx = (priv->p_idx + 1) % buffer_size;
		priv->buffer[priv->p_idx] = dx[2];
		priv->p_idx = (priv->p_idx + 1) % buffer_size;
		priv->buffer[priv->p_idx] = dx[3];
		priv->p_idx = (priv->p_idx + 1) % buffer_size;
		priv->buffer[priv->p_idx] = dx[4];
		priv->p_idx = (priv->p_idx + 1) % buffer_size;

		/* Buffer was empty and block mode is on, wake up the reader */
		if ((priv->b_mode != 0) && (priv->c_idx == pidx)) 
			wake_up_interruptible(&ir_wq);

		goto out;
	}
#endif

#ifdef WITH_RC5_CONTROL
	if (status & 0x00000001) {	// RC5 IRQ
		data = gbus_read_reg32(IR_RC5_DECODER_DATA);
		gbus_write_reg32(IR_RC5_DECODER_DATA, 0);
		if ((data & 0x80000000) != 0)  /* Invalid RC5 decoder data */
			goto out;
	} else if (status & 0x00000002) {	// NEC IRQ
		data = gbus_read_reg32(IR_NEC_CAPTURE_DATA);
		gbus_write_reg32(IR_NEC_CAPTURE_DATA, 0);
	} else
		goto out;
#else
	data = gbus_read_reg32(IR_NEC_CAPTURE_DATA);
	gbus_write_reg32(IR_NEC_CAPTURE_DATA, 0);
#endif

	/* Discard not used data if needed */
	if (data == 0) {
		if (save_key == 0)
			goto out;
		old_key = 0;
#ifdef DEBUG_IR
		printk(KERN_DEBUG "%s: no data\n", ir_devname);
#endif
		if (time_after(priv->last_jiffies + wp_var, jiffies)) {
#ifdef DEBUG_IR
			printk(KERN_DEBUG "%s: repetition too fast\n", ir_devname);
#endif
			goto out; 	/* Key repeats too fast, drop it */
		} else if (time_before(priv->last_jiffies + (wait_period * 4), jiffies)) {
#ifdef DEBUG_IR
			printk(KERN_DEBUG "%s: got slow repetition, glitch?\n", ir_devname);
#endif
			save_key = 0;	/* Disallow key repitition */
			goto out;	/* Repeat too slow, drop it */
		} else if (ir_findkey(priv, save_key) == 0) {
#ifdef DEBUG_IR
			printk(KERN_DEBUG "%s: not repeatable key 0x%lx\n", ir_devname, save_key);
#endif
			goto out; /* If the key is not in the table, drop it */
		}

#ifdef DEBUG_IR
		printk(KERN_DEBUG "%s: got repeated key 0x%lx\n", ir_devname, save_key);
#endif
		data = save_key; /* Valid repeated key */
		repeat_key = 1;
	} else if (time_after(priv->last_jiffies + wait_period, jiffies) && 
			(data == old_key)) {
		/* Throw away this key if this is the same key and came too
		   fast */
#ifdef DEBUG_IR
		printk(KERN_DEBUG "%s: same data\n", ir_devname);
#endif
		save_key = 0;
		goto out;
	} 

	/* Shrink the wait time for repeat key if current one is repeated */
	wp_var = (repeat_key ? (wait_period / 2) : wait_period); 
	priv->last_jiffies = jiffies;
	save_key = old_key = data;

#ifdef DEBUG_IR
	printk(KERN_DEBUG "%s: got data 0x%08lx\n", ir_devname, data);
#endif

	pidx = priv->p_idx;	/* Save the old index before proceeding */

	if (repeat_sends_zero && repeat_key)
		data = 0; /* clear the data */

	/* Save it to buffer */
	if (((priv->p_idx + 1) % buffer_size) == priv->c_idx) {
		/* Adjust consumer index since buffer is full */
		/* Keep the latest one and drop the oldest one */
		priv->c_idx = (priv->c_idx + 1) % buffer_size;

		printk(KERN_WARNING "%s: buffer full\n", ir_devname);
	}

	priv->buffer[priv->p_idx] = data;
	priv->p_idx = (priv->p_idx + 1) % buffer_size;

	/* Buffer was empty and block mode is on, wake up the reader */
	if ((priv->b_mode != 0) && (priv->c_idx == pidx)) 
		wake_up_interruptible(&ir_wq);

out:
	spin_unlock(&priv->lock);
}

/* ISR for IR device */
static irqreturn_t ir_isr(int irq, void *dev_id)
{
	struct ir_private *priv = (struct ir_private *)dev_id;
	unsigned long status = 0;

	if (priv != &ir_priv)		/* Paranoid check */
		return IRQ_NONE;

	// gbus_write_reg32(REG_BASE_CPU + CPU_edge_rawstat, IRQMASKOF(ir_irq));
	
#ifdef WITH_RC6_CONTROL 
	status = gbus_read_reg32(IR_RC6_CTRL);
	if ((status & 0xc0000000) != 0) {
		gbus_write_reg32(IR_RC6_CTRL, status); /* Clear interrupt */
#ifdef DEBUG_IR
		if ((status & 0x40000000) != 0) 
			printk(KERN_DEBUG "RC6 Err IRQ (0x%08lx)\n", status);
#endif
		if ((status & 0x80000000) != 0) {
			/* We have RC6 data */
#ifdef DEBUG_IR
			printk(KERN_DEBUG "RC6 Datain IRQ (0x%08lx)\n", status);
#endif
			ir_produce(priv, status);

#ifdef WITH_RC5_CONTROL
			/* Force to clear RC5 interrupt status */
			status = gbus_read_reg32(IR_RC5_INT_STATUS);
			if ((status & 0x00000003) != 0)
				gbus_write_reg32(IR_RC5_INT_STATUS, status); /* Clear interrupt if any */
#endif
			return IRQ_HANDLED;
		} 
	}
#endif

#ifdef WITH_RC5_CONTROL
	status = gbus_read_reg32(IR_RC5_INT_STATUS);
#ifdef DEBUG_IR
	if ((status & 0x00000001) != 0) {
		/* RC5 interrupt */
		printk(KERN_DEBUG "RC5 IRQ (0x%08lx)\n", status);
	}
	if ((status & 0x00000002) != 0) {
		/* NEC interrupt */
		printk(KERN_DEBUG "NEC IRQ (0x%08lx)\n", status);
	}
#endif
	gbus_write_reg32(IR_RC5_INT_STATUS, status); /* Clear interrupt */
	status &= 0x00000003;
#endif /* WITH_RC5_CONTROL */

	ir_produce(priv, status);
	return IRQ_HANDLED;
}

/* Reading from driver's buffer, note that it can return read size
   less than specified */
static int ir_consume(void *dev_id, unsigned long *buf, int count)
{
	struct ir_private *priv = (struct ir_private *)dev_id;
	int cnt = 0;
	unsigned long flags;

	/* If block mode is on, check the emptiness of buffer */
	if (priv->b_mode != 0) {
		/* Sleep when buffer is empty */
		wait_event_interruptible(ir_wq, priv->c_idx != priv->p_idx);
	}
	if (signal_pending(current))
		return(cnt);

	spin_lock_irqsave(&priv->lock, flags);

	/* Get the data out and adjust consumer index */
	for (cnt = 0; (priv->c_idx != priv->p_idx) && (cnt < count); cnt++) {
		*buf = priv->buffer[priv->c_idx];
		priv->c_idx = (priv->c_idx + 1) % buffer_size;
		buf++;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return(cnt);
}

/* Reading data */
static int ir_read(struct file *fptr, char *bufptr, size_t size, loff_t *fp)
{
	unsigned long buf[buffer_size];
	int count;

	/* Check the alignment */
	if (size % sizeof(unsigned long)) {
		printk(KERN_WARNING "%s: read size not aligned to %d\n",
			ir_devname, sizeof(unsigned long));
		return(-EIO);
	}

	count = ir_consume(fptr->private_data, &buf[0], 
			size / sizeof(unsigned long)) * sizeof(unsigned long);

	/* Get the data to user */
	if (count && copy_to_user(bufptr, (char *)&buf[0], count)) 
		return(-EFAULT);

	return(count);
}

/* ioctl function */
static int ir_ioctl(struct inode *inode, struct file *fptr, unsigned int cmd, unsigned long arg)
{
	unsigned long *ptr = (unsigned long *)arg;
	unsigned long key_no = 0;
	struct ir_private *priv = (struct ir_private *)fptr->private_data;
	unsigned long flags;

	if (ptr == NULL)
		return(-EIO);

	spin_lock_irqsave(&priv->lock, flags);

	switch(cmd) {
		case IR_IOCSETREPEATKEYS: /* Set the repetition keys */
			if (copy_from_user(&key_no, ptr, sizeof(unsigned long))) {
				spin_unlock_irqrestore(&priv->lock, flags);
				return(-EFAULT);
			} else if ((key_no <= 0) || (key_no > max_keys)) {
				spin_unlock_irqrestore(&priv->lock, flags);
				return(-EIO);
			} else
				priv->num_keys = key_no;
			copy_from_user(priv->key_table, ptr + 1, sizeof(unsigned long) * key_no);
			break;
		case IR_IOCGETREPEATKEYS: /* Get the repetition keys */
			key_no = priv->num_keys;
			copy_to_user(ptr, &key_no, sizeof(unsigned long));
			if (key_no > 0)
				copy_to_user(ptr + 1, priv->key_table, sizeof(unsigned long) * key_no);
			break;
		case IR_IOCSETWAITPERIOD:
			wait_period = wp_var = ((((arg * HZ) / 1000) == 0) ? 1 : ((arg * HZ) / 1000));
			break;
		case IR_IOCGETWAITPERIOD: {
				int wp = (wait_period * 1000) / HZ;
				copy_to_user(ptr, &wp, sizeof(int));
			}
			break;
		default:
			spin_unlock_irqrestore(&priv->lock, flags);
                        return(-EIO);
	}

	spin_unlock_irqrestore(&priv->lock, flags);
	return(0);
}

/* Poll function */
static unsigned int ir_poll(struct file *fptr, struct poll_table_struct *ptable)
{
	struct ir_private *priv = (struct ir_private *)fptr->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	poll_wait(fptr, &ir_wq, ptable);

	spin_lock_irqsave(&priv->lock, flags);
	if (priv->c_idx != priv->p_idx)
		mask |= (POLLIN | POLLRDNORM);
	spin_unlock_irqrestore(&priv->lock, flags);

	return(mask);
}

/* Open the device */
static int ir_open(struct inode *inode_ptr, struct file *fptr)
{
	/* This device is exclusive, that is, only one process can use it */
	if (ir_priv.ref_cnt != 0) {
		printk(KERN_WARNING "%s: exclusive access only\n", ir_devname);
		return(-EIO);
	/* This device is read-only */
	} else if ((fptr->f_flags & O_ACCMODE) != O_RDONLY) {
		printk(KERN_WARNING "%s: read-only device\n", ir_devname);
		return(-EIO);
	} 

	/* Set the block mode and increase reference count */
	ir_priv.ref_cnt++;
	ir_priv.b_mode = ((fptr->f_flags & O_NONBLOCK) ? 0 : 1);
	ir_priv.last_jiffies = jiffies;

	/* Flush the buffer */
	ir_priv.p_idx = ir_priv.c_idx = 0;

	fptr->f_op = &ir_fops;
	fptr->private_data = (void *)&ir_priv;

	/* Enable the NEC device (CTRL register) */
	/*	31:30 - reserved */
	/*	29:24 	IR_CAPTURE_NBITS [5:0] -> set to 0x1f */
	/*	23:22 - reserved */
	/*	21:16 	GPIO_INFREARED_SEL [5:0] -> set to 12 */
	/*	15:14 - reserved */
	/*	13:0	IR_PREDIV_DEVIDER [13:0] -> set to 0x3b10 */
	gbus_write_reg32(REG_BASE_system_block + SYS_gpio_dir, GPIO_DIR_INPUT(12));
	gbus_write_reg32(IR_NEC_CAPTURE_DATA, 0);
	gbus_write_reg32(IR_NEC_CTRL, 0x1f0c3b10);

	wp_var = wait_period;

	printk(KERN_DEBUG "%s: Enable NEC decoder (0x%08x)\n", 
			ir_devname, (u32)gbus_read_reg32(IR_NEC_CAPTURE_DATA));

#ifdef WITH_RC5_CONTROL
	/* Enable the RC5 device (CTRL register) */
	/*	31:10 - reserved */
	/*	9	IR_RC5_INT_ENABLE -> set */
	/*	8	IR_NEC_INT_DISABLE */
	/*	7	IR_DEBOUNCE_SEL1 -> set */
	/*	6	IR_DEBOUNCE_SEL0 -> set */
	/*	5	IR_DEBOUNCE_ENABLE -> set */
	/*	4	IR_NEC_DISABLE */
	/*	3	IR_RSYNC_1/4 -> set */
	/*	2	IR_RSYNC_1/8 */
	/*	1	IR_SIGNAL_INVERT */
	/*	0	IR_RC5_DECODE_ENABLE -> set */
	gbus_write_reg32(IR_RC5_DECODE_CLK_DIV, rc5_clk_div);
	gbus_write_reg32(IR_RC5_DECODER_DATA, 0);
	gbus_write_reg32(IR_RC5_CTRL, 0x000002e9);

	printk(KERN_DEBUG "%s: Enable RC5 decoder (0x%08x)\n", 
			ir_devname, (u32)gbus_read_reg32(IR_RC5_DECODER_DATA));
#endif /* WITH_RC5_CONTROL */

#ifdef WITH_RC6_CONTROL
	/* Enable the RC6 device (CTRL register) */
	/*	7	IR_RC6_DATA_IN_INT_ENABLE -> set */
	/*	6	IR_RC6_ERROR_INT_ENABLE -> set */
	/*	5:2	reserved */
	/*	1	IR_SIGNAL_INVERT */
	/*	0	IR_RC6_DECODE_ENABLE -> set */
	gbus_write_reg32(IR_RC6_CTRL, 0xc1);
	/* Tolerance and Duration */
	/*	31:18	Tolerance (typ. 0xbb8) */
	/*	17:0	Duration (typ. 0x2ee0) */
	gbus_write_reg32(IR_RC6_T_CTRL, (0xbb8 << 18) | 0x2ee0);

	printk(KERN_DEBUG "%s: Enable RC6 decoder\n", ir_devname);
#endif
	return(0);
}

/* Close the device */
static int ir_release(struct inode *inode_ptr, struct file *fptr) 
{
	/* Disable the NEC device */
	printk(KERN_DEBUG "%s: Disable NEC decoder\n", ir_devname);
	gbus_write_reg32(IR_NEC_CAPTURE_DATA, 0);
	gbus_write_reg32(IR_NEC_CTRL, 0);

#ifdef WITH_RC5_CONTROL
	/* Disable RC5 control */
	printk(KERN_DEBUG "%s: Disable RC5 decoder\n", ir_devname);
	gbus_write_reg32( IR_RC5_CTRL, (gbus_read_reg32(IR_RC5_CTRL) & 0xfffffdfe) | 0x00000100);
#endif /* WITH_RC5_CONTROL */

#ifdef WITH_RC6_CONTROL
	printk(KERN_DEBUG "%s: Disable RC6 decoder\n", ir_devname);
	gbus_write_reg32(IR_RC6_CTRL, 0xc0000000);
#endif /* WITH_RC6_CONTROL */

	/* Adjust reference count */
	ir_priv.ref_cnt--;

	return(0);
}

int __init ir_init_module(void)
{
	int status = 0;

	extern int tangox_ir_enabled(void);
	if (tangox_ir_enabled() == 0)
		return(0);

	/* Initialize private data structure */
	memset(&ir_priv, 0, sizeof(struct ir_private)); 
	spin_lock_init(&ir_priv.lock);

#ifdef WITH_RC6_CONTROL
	if (buffer_size < 6) {
		printk(KERN_ERR "%s: buffer size (%d) error, minimum 6.\n", ir_devname,
			buffer_size); 
		return(-EIO);
	} 
#else
	if (buffer_size < 2) {
		printk(KERN_ERR "%s: buffer size (%d) error, minimum 2.\n", ir_devname,
			buffer_size); 
		return(-EIO);
	} 
#endif

	if ((ir_priv.buffer = kmalloc(buffer_size * sizeof(unsigned long), GFP_KERNEL)) == NULL) {
		printk(KERN_ERR "%s: out of memory for buffer\n", ir_devname); 
		return(-ENOMEM);
	} else if ((ir_priv.key_table = kmalloc(max_keys * sizeof(unsigned long), GFP_KERNEL)) == NULL) {
		printk(KERN_ERR "%s: out of memory for key table\n", ir_devname); 
		kfree(ir_priv.buffer);
		return(-ENOMEM);
	}

	/* Register device, and may be allocating major# */
	status = register_chrdev(major_num, ir_devname, &ir_fops);
	if (status < 0) {
		printk(KERN_ERR "%s: cannot get major number\n", ir_devname); 
		kfree(ir_priv.buffer);
		kfree(ir_priv.key_table);
		return(status);
	} else if (major_num == 0)
		major_num = status;	/* Dynamic major# allocation */

	/* Make sure interrupt is disabled, will be re-enabled in device
	   open stage */
	gbus_write_reg32(IR_NEC_CTRL, 0);

#ifdef WITH_RC5_CONTROL
	gbus_write_reg32(IR_RC5_CTRL, (gbus_read_reg32(IR_RC5_CTRL) & 0xfffffdfe) | 0x00000100);
#endif

#ifdef WITH_RC6_CONTROL
	gbus_write_reg32(IR_RC6_CTRL, 0xc0000000);
#endif

	/* Hook up ISR */
	if (request_irq(ir_irq, ir_isr, IRQF_SHARED, ir_devname, 
			&ir_priv) != 0) {
		printk(KERN_ERR "%s: cannot register IRQ (%d)\n", ir_devname,
			ir_irq);
		unregister_chrdev(major_num, ir_devname);	
		kfree(ir_priv.buffer);
		kfree(ir_priv.key_table);
		return(-EIO);
	}

	printk(KERN_INFO "SMP86xx %s (%d:0): driver loaded (wait_period = %dms, "
		"buffer_size = %d)\n", ir_devname, major_num, wait_period, buffer_size);
	return(0);
}

void __exit ir_cleanup_module(void)
{
	extern int tangox_ir_enabled(void);
	if (tangox_ir_enabled() == 0)
		return;

	unregister_chrdev(major_num, ir_devname);
	free_irq(ir_irq, &ir_priv);

	if (ir_priv.buffer != NULL)
		kfree(ir_priv.buffer);
	if (ir_priv.key_table != NULL)
		kfree(ir_priv.key_table);

	printk(KERN_INFO "%s: driver unloaded\n", ir_devname);
}

module_init(ir_init_module);
module_exit(ir_cleanup_module);

