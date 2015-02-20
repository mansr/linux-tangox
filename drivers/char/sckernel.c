
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
#include <linux/delay.h>

#if !defined(CONFIG_TANGOX)
#error "Unsupport architecture (TANGOX only)."
#endif

#include <asm/io.h>

#ifdef CONFIG_TANGO2
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/tango2api.h>
#include <asm/tango2/scard.h>
#define NUM_SCARD          1
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/tango3api.h>
#include <asm/tango3/scard.h>
#define NUM_SCARD          2
#else
#error "Unsupport architecture."
#endif

//#define SCARD_DEBUG /* Uncomment this for debug message */
#define SCARD_ERRMSG /* Uncomment this for error message */
#define SCARD_MSG /* Uncomment this for normal message */

#ifdef SCARD_DEBUG
#define DBG_PRINT	if (debug) printk
#else
static void inline DBG_PRINT(const char *x, ...) { ; }
#endif /* SCARD_DEBUG */

#ifdef SCARD_ERRMSG
#define ERR_PRINT	printk
#else
static void inline ERR_PRINT(const char *x, ...) { ; }
#endif /* SCARD_ERRMSG */

#ifdef SCARD_MSG
#define MSG_PRINT	printk
#else
static void inline MSG_PRINT(const char *x, ...) { ; }
#endif /* SCARD_ERRMSG */

#define SCARD_TX_BYTE_REG(x)	((x) + 0x0000)
#define SCARD_TX_WORD_REG(x)	((x) + 0x0004)
#define SCARD_TX_DWORD_REG(x)	((x) + 0x0008)
#define SCARD_RX_BYTE_REG(x)	((x) + 0x0010)
#define SCARD_RX_WORD_REG(x)	((x) + 0x0014)
#define SCARD_RX_DWORD_REG(x)	((x) + 0x0018)
#define SCARD_CLK_HIGH_VAL(x)	((x) + 0x0020)
#define SCARD_CLK_LOW_VAL(x)	((x) + 0x0024)
#define SCARD_TIMEOUT_LOAD(x)	((x) + 0x0028)
#define SCARD_PARAM_REG(x)	((x) + 0x002c)
#define SCARD_EGT_ETU_REG(x)	((x) + 0x0030)
#define SCARD_SOFT_OUT_REG(x)	((x) + 0x0034)
#define SCARD_SCARD_CTRL_REG(x)	((x) + 0x0038)
#define SCARD_STATE_REG(x)	((x) + 0x003c)
#define SCARD_INT_REG(x)	((x) + 0x0040)
#define SCARD_INTEN_REG(x)	((x) + 0x0044)
#define SCARD_ALT_ETU_REG(x)	((x) + 0x0048)
#define SCARD_STATUS_REG(x)	((x) + 0x004c)
#define SCARD_CRC_REG(x)	((x) + 0x0050)
#define SCARD_CRC_INIT_REG(x)	((x) + 0x0054)

#ifdef CONFIG_TANGO3
#define SCARD_GPIO_OE_REG(x)       ((x) + 0x0058)
#define SCARD_GPIO_DATA_REG(x)     ((x) + 0x005c)
#define SCARD_GPIO_MODE_REG(x)     ((x) + 0x0060)
#define SCARD_RX_TO_TX_IDLE_REG(x) ((x) + 0x0064)
#endif

#define RD_BUF_SIZE		256	/* Default read buffer size */
#define WR_BUF_SIZE		256	/* Default write buffer size */
#define ANS_STR_SIZE		512	/* Max. size of ans-to-reset string */
#define DEF_FREQUENCY           2000000 /* Default operating frequency 2MHz */
#define TIMEOUT                 ((((2 * HZ) / 1000) == 0) ? 1 : ((2 * HZ) / 1000))

#define ATR_DONE                  0
#define ATR_WITH_NEGOTIATION      1
#define ATR_RESEND                0xf
#define ATR_INVALID               (-1)

/* The scard control pins: can be board dependent */
#define SCARD_OFF_PIN           0  /* SCARD_CTL[0] */
#define SCARD_5V_PIN            1  /* SCARD_CTL[1] */
#define SCARD_CMD_PIN           2  /* SCARD_CTL[2] */

#define SCARD1_OFF_PIN          0  /* SCARD_CTL[0] */
#define SCARD1_5V_PIN           1  /* SCARD_CTL[1] */
#define SCARD1_CMD_PIN          2  /* SCARD_CTL[2] */

/* The major device number and name */
#define SCARD_DEV_MAJOR		0
#define SCARD_DEV_NAME		"scard"

#define RST_WAIT		130	/* Wait time to drive RSTIN low */
					/* Min. 50, Max. 130 */
#define RST_WIDTH		5	/* Keep RSTIN for this long */

#define TX_FIFO_SIZE            0x0f

#define DRIVER_VERSION         "1.32"

MODULE_DESCRIPTION("EM86xx/SMP86xx smart card driver\n");
MODULE_AUTHOR("TANGO15/TANGO2/TANGO3 standalone team");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

static int major_num = SCARD_DEV_MAJOR;
static int rd_buffer_size = RD_BUF_SIZE;
static int wr_buffer_size = WR_BUF_SIZE;
static int frequency = DEF_FREQUENCY;
static int reverse_msb = 0; /* By default, don't reverse MSB */
static int reverse_pol = 0; /* By default, don't reverse polarity */
static int use_3v = 1; /* By default, start with 3V */
module_param(major_num, int, 0);
module_param(rd_buffer_size, int, 0);
module_param(wr_buffer_size, int, 0);
module_param(frequency, int, 0);
module_param(reverse_msb, int, 0);
module_param(reverse_pol, int, 0);
module_param(use_3v, int, 0);
#ifdef SCARD_DEBUG
static int debug = 1;
module_param(debug, int, 0);
#endif

extern unsigned long tangox_get_sysclock(void);
extern int tangox_scard_enabled(int);

/* The state for smart card */
enum SCARD_STATE {
	UNPWR_IDLE = 0,
	ACTIVATE = 1,
	COLD_RESET = 2,
	PWR_IDLE = 3,
	WARM_RESET = 4,
	CLOCK_STOP = 5,
	DEACTIVATE = 6,
	UNKNOWN = 7,
};

/* If given state is persistent state? */
#define PERSISTENT_STATE(x)	(((x) == UNPWR_IDLE) || ((x) == CLOCK_STOP) || ((x) == PWR_IDLE))

/* Private data structure */
struct scard_private {
	int enabled;			/* Enabled? */
	unsigned char *rd_buffer;	/* Circular buffer for read */
	unsigned char *wr_buffer;	/* Circular buffer for write */
	unsigned rd_p_idx;		/* Index of producer */
	unsigned rd_c_idx; 		/* Index of consumer */
	unsigned wr_p_idx;		/* Index of producer */
	unsigned wr_c_idx; 		/* Index of consumer */
	unsigned ref_cnt;		/* Reference count */
	spinlock_t lock;		/* Spin lock */
	unsigned char b_mode;		/* Blocking mode or not */
	unsigned long last_rd_jiffies;	/* Timestamp for last reception */
	unsigned long last_wr_jiffies;	/* Timestamp for last reception */
	atomic_t card_existed;		/* Is smart card existed? */
	atomic_t state;			/* State? UNPWR_IDLE or PWR_IDLE */
	atomic_t normal_mode;		/* Normal operation mode? */
	int rev_msb;			/* Reverse MSB? */
	int rev_pol;			/* Reverse polarity? */
	int volt3;			/* 3V card? */
	int clockstop;			/* clockstop supported */
	int reset_cnt;                  /* reset counter */
	int stop_thread;                
	unsigned long base_addr;        /* base address for scard controller */
	unsigned long write_cnt;
	unsigned long read_cnt;
	int scard_5v_pin;
	int scard_cmd_pin;
	int scard_off_pin;
	int scard_irq;
	char devname[32];
	int idx;                        /* controller index */
	unsigned char fi;
	unsigned char di;

	/* For answer to reset */
	atomic_t answer2reset;
	unsigned char ans_string[ANS_STR_SIZE];
	int ans_idx;

	/* Wait queue, may be used if block mode is on */
	wait_queue_head_t scard_rd_wq;
};

/* Static information for each scard controller */
struct scard_info {
	unsigned long base_addr;
	int scard_5v_pin;
	int scard_cmd_pin;
	int scard_off_pin;
	int scard_irq;
};

static struct scard_info scard_ctrlers[] = {
	{ (REG_BASE_cpu_block + 0xc300), SCARD_5V_PIN, SCARD_CMD_PIN, SCARD_OFF_PIN, IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_SMARTCARD_INT },
#ifdef CONFIG_TANGO3
	{ (REG_BASE_cpu_block + 0xc380), SCARD1_5V_PIN, SCARD1_CMD_PIN, SCARD1_OFF_PIN, IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_SMARTCARD1_INT },
#endif
};

/* Some prototypes */
static int scard_open(struct inode *, struct file *);
static int scard_release(struct inode *, struct file *);
static int scard_read(struct file *, char *, size_t, loff_t *);
static int scard_write(struct file *, const char *, size_t, loff_t *);
static int scard_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
static unsigned int scard_poll(struct file *, struct poll_table_struct *);

/* Global data */
static struct scard_private scard_priv[NUM_SCARD];
static char *scard_devname = SCARD_DEV_NAME;
static int num_scard = NUM_SCARD;

int tangox_get_scard_info(int scard_no, int *pin_5v, int *pin_cmd, int *pin_off);

/* From smartcard spec. */
static const unsigned fi_tab[16] = { 372, 372, 558, 744, 1116, 1488, 1860, 0, 0, 512, 768, 1024, 1536, 2048, 0, 0 };
static const unsigned di_tab[16] = { 0, 1, 2, 4, 8, 16, 32, 0, 12, 20, 0, 0, 0, 0, 0, 0 };

/* Keep track of system frequency */
static unsigned long sys_freq = 0;

static struct file_operations scard_fops = {
	open: scard_open,
	read: scard_read,
	write: scard_write,
	ioctl: scard_ioctl,
	poll: scard_poll,
	release: scard_release,
	owner: THIS_MODULE,
};

/* Prototype */
static void scard_reset(struct scard_private *priv, int cold);

static inline unsigned int scard_ctrlpin_read(int idx, unsigned int pin)
{
	unsigned int val;
#ifdef CONFIG_TANGO2
	val = (gbus_read_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_data) >> 16) & 0x7;
#elif defined(CONFIG_TANGO3)
	if ((pin >= 0) && (pin <= 2)) /* if this is one of the SCARD ctrl pins */
		val = (gbus_read_reg32(SCARD_GPIO_DATA_REG(scard_priv[idx].base_addr)) >> 4) & 0x7;
	else
		val = em86xx_gpio_read(pin);
#endif
	return((val >> pin) & 0x1);
}

static inline void scard_ctrlpin_write(int idx, int pin, unsigned int val)
{
	unsigned int v;
#ifdef CONFIG_TANGO2
	v = ((gbus_read_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_data) >> 16) & 0x7) & ~(1 << pin);
	v |= (((val != 0) ? 1 : 0) << pin);
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_data, v << 16);
#elif defined(CONFIG_TANGO3)
	if ((pin >= 0) && (pin <= 2)) { /* if this is one of the SCARD ctrl pins */
		v = (((val != 0) ? 1 : 0) << (pin + 4)) | (1 << (pin + 12));
		gbus_write_reg32(SCARD_GPIO_DATA_REG(scard_priv[idx].base_addr), v);
	} else {
		em86xx_gpio_write(pin, (val != 0) ? 1 : 0);
	}
#endif
}

static inline void scard_ctrlpin_init(int idx)
{
	/* Set SCARD_CTL[0..2] to 2 outputs for 5V, CMD, and 1 input for OFF */
#ifdef CONFIG_TANGO2
	gbus_write_reg32(REG_BASE_cpu_block + CPU_uart0_gpio_dir,
			((1 << scard_priv[idx].scard_cmd_pin) | (1 << scard_priv[idx].scard_5v_pin)) << 16);
#elif defined(CONFIG_TANGO3)
	unsigned int v, d;
	/* Set smard card mode, but leave CTL pins as GPIO mode */
	gbus_write_reg32(SCARD_GPIO_MODE_REG(scard_priv[idx].base_addr), 0x7f70); 
	v = ((1 << scard_priv[idx].scard_cmd_pin) | (1 << scard_priv[idx].scard_5v_pin)) << 4;
	d = (1 << scard_priv[idx].scard_cmd_pin) << 4;
	v |= 0x7000; /* With OE mask set, CMD/5V output, OFF input */
	d |= ((1 << scard_priv[idx].scard_cmd_pin) | (1 << scard_priv[idx].scard_5v_pin)) << 12;
	gbus_write_reg32(SCARD_GPIO_DATA_REG(scard_priv[idx].base_addr), d); /* Set values */
	gbus_write_reg32(SCARD_GPIO_OE_REG(scard_priv[idx].base_addr), v); /* Set outputs */
#endif
}

/* schedule timeout */
static void scard_schedule_timeout(unsigned int wait_jiffies)
{
	unsigned long old_jiffies = jiffies;
	if ((!in_interrupt()) && (!in_atomic())) {
		unsigned int wp = ((HZ / 1000) ? (HZ / 1000) : 1); /* at least one jiffy, or 1msec */
		DBG_PRINT("Sleep for %d jiffies.\n", wait_jiffies);
		while (time_after(old_jiffies + wait_jiffies, jiffies))  {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(wp);
		}
	} else {
		DBG_PRINT("Not safe to schedule out, wait for %d jiffies\n", wait_jiffies);
		while (time_after(old_jiffies + wait_jiffies, jiffies)) 
			/* busy waiting */ ;
	}
}

/* Read data from device */
static void scard_read_device(struct scard_private *priv)
{
	unsigned char data = 0;
	unsigned rd_pidx;
	unsigned int fifo_size;
	unsigned int i;

	spin_lock(&priv->lock);
	if ((fifo_size = ((gbus_read_reg32(SCARD_STATE_REG(priv->base_addr)) >> 20) & 0x1f)) == 0) {
		DBG_PRINT("%s: RX FIFO empty\n", priv->devname);
		goto out;
	}

	/* Read from FIFO into driver's buffer */
	for (i = 0; i < fifo_size; i++) {
		data = gbus_read_reg32(SCARD_RX_BYTE_REG(priv->base_addr));
		priv->read_cnt++;

		DBG_PRINT("%s: got data 0x%x (cnt: %ld)\n", priv->devname, data, priv->read_cnt);
		rd_pidx = priv->rd_p_idx;	/* Save the old index before proceeding */

		/* Save it to buffer */
		if (((priv->rd_p_idx + 1) % rd_buffer_size) == priv->rd_c_idx) {
			ERR_PRINT("%s: read buffer full\n", priv->devname);
			break;
		}

		priv->rd_buffer[priv->rd_p_idx] = data;
		priv->rd_p_idx = (priv->rd_p_idx + 1) % rd_buffer_size;

		/* Buffer was empty and block mode is on, wake up the reader */
		if ((priv->b_mode != 0) && (priv->rd_c_idx == rd_pidx)) 
			wake_up_interruptible(&(priv->scard_rd_wq));
	}
	priv->last_rd_jiffies = jiffies;

out:
	spin_unlock(&priv->lock);
}

/* write PPS to scard */
static void scard_write_pps(struct scard_private *priv, const unsigned char *pps, int size)
{
	int i;
	/* Assuming PPS size is smaller than TX_FIFO */
	for (i = 0; i < size; i++, pps++) {
		gbus_write_reg32(SCARD_TX_BYTE_REG(priv->base_addr), *pps);
		DBG_PRINT("%s: written pps data 0x%x (idx %d)\n", priv->devname, *pps, i);
	}
}

/* Wrtie data to device */
static void scard_write_device(struct scard_private *priv)
{
	unsigned char data;
	unsigned int fifo_size;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&priv->lock, flags);

	if (priv->wr_c_idx == priv->wr_p_idx) {
		goto out; /* Empty buffer */
	} else if ((fifo_size = ((gbus_read_reg32(SCARD_STATE_REG(priv->base_addr)) >> 12) & 0x1f)) == TX_FIFO_SIZE) {
		DBG_PRINT("%s: TX FIFO full (%d)\n", priv->devname, fifo_size);
		goto out;
	} else if (fifo_size > TX_FIFO_SIZE) {
		ERR_PRINT("%s: TX FIFO overflow (%d)\n", priv->devname, fifo_size);
		goto out;
	}

	for (i = 0; i < (TX_FIFO_SIZE - fifo_size); i++) {
		if (priv->wr_c_idx == priv->wr_p_idx) 
			break; /* Empty buffer */

		data = priv->wr_buffer[priv->wr_c_idx];
		priv->wr_c_idx = (priv->wr_c_idx + 1) % wr_buffer_size;
		gbus_write_reg32(SCARD_TX_BYTE_REG(priv->base_addr), data);
		priv->write_cnt++;

		DBG_PRINT("%s: written data 0x%x (fifo: %d, cnt: %ld)\n", priv->devname, data, fifo_size, priv->write_cnt);
	}

#ifdef SCARD_DEBUG
	if (fifo_size >= TX_FIFO_SIZE)
		DBG_PRINT("%s: TX FIFO full %d\n", priv->devname, fifo_size);
#endif
	
	priv->last_wr_jiffies = jiffies;

out:
	spin_unlock_irqrestore(&priv->lock, flags);
}

/* ISR for Smart card device */
static irqreturn_t scard_isr(int irq, void *dev_id)
{
	struct scard_private *priv = (struct scard_private *)dev_id;
	unsigned long status = 0;

	if ((status = gbus_read_reg32(SCARD_INT_REG(priv->base_addr))) == 0)
		return IRQ_NONE;

	gbus_write_reg32(SCARD_INT_REG(priv->base_addr), status);

	if (atomic_read(&(priv->normal_mode)) != 0) {
		if (status & (1 << 2)) { 
			/* For RX interrupts */
			scard_read_device(priv);
		}
		if (status & (1 << 0)) {
			/* For TX interrupts */
			scard_write_device(priv);
		}
		if (status & ((1 << 1) | (1 << 3) | (1 << 4) | (1 << 5))) {
			ERR_PRINT("%s: error interrupt (status: 0x%08lx)\n", priv->devname, status);
			/* TODO: does WARM_RESET, reset/un-reset FIFO ?? */
		}
	} else {
		/* Initializing connection, handle "answer to reset" */
		if (status & (1 << 0)) {
			/* For TX interrupts, sending PPS... */
			DBG_PRINT("TX completed.\n");
		}
		if (status & (1 << 2)) { 
			unsigned long data;
			atomic_set(&(priv->answer2reset), 1);

			while (((gbus_read_reg32(SCARD_STATE_REG(priv->base_addr)) >> 20) & 0x1f) != 0) {
				/* For RX interrupts */
				data = gbus_read_reg32(SCARD_RX_BYTE_REG(priv->base_addr)); 
				if (priv->ans_idx >= ANS_STR_SIZE) {
					gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0); /* disable interrupt */
					break;
				}
				priv->ans_string[priv->ans_idx++] = data & 0xff;
			}
		}
		if (status & ((1 << 1) | (1 << 3) | (1 << 4) | (1 << 5))) {
			gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0); /* disable interrupt */
			DBG_PRINT("%s: error interrupt (status: 0x%08lx)\n", priv->devname, status);
			DBG_PRINT("%s: SCARD_CTRL : 0x%08lx\n", priv->devname, gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)));
		}
	}

	return IRQ_HANDLED;
}

/* Check if scard existed */
static int is_card_present(struct scard_private *priv)
{
	/* Low: not present, high otherwise */
	return((scard_ctrlpin_read(priv->idx, priv->scard_off_pin) == 0) ? 0 : 1);
}

/* Handle negotiation mode if needed */
static int scard_negotiate(struct scard_private *priv)
{
	int res = -1;
	unsigned char pps[] = { 0xff, 0x00, 0x0, 0x0 };
	unsigned char xor = 0;
	int i;

	DBG_PRINT("Entering negotiation mode ...\n");

	/* change ETU back to FD/DD */
	DBG_PRINT("Default ETU of 371 is used.\n");
	gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), 
			(gbus_read_reg32(SCARD_EGT_ETU_REG(priv->base_addr)) & 0xffff0000) | ((372 -1) << 0));
	
	/* form pps request */
	pps[1] = 0x10 | 0; /* with PPS1 and T=0 */
	if ((priv->fi == 0) && (priv->di == 0))
		pps[2] = ((0 << 4) | 1); /* 371/1 is used */
	else
		pps[2] = ((priv->fi) << 4) | priv->di; /* use what's given by ATR */

	/* calculate pck */
	for (i = 0; i < (sizeof(pps) - 1); i++)
		xor ^= pps[i];	
	pps[sizeof(pps) - 1] = xor;
#ifdef SCARD_DEBUG
	DBG_PRINT("PPS Request: ");
	for (i = 0; i < sizeof(pps); i++)
		DBG_PRINT("0x%02x ", pps[i]);
	DBG_PRINT("\n");
#endif
	priv->ans_idx = 0; /* reset ATR string index */
	atomic_set(&(priv->answer2reset), 0);
	scard_write_pps(priv, pps, sizeof(pps));

	scard_schedule_timeout(HZ / 10); /* Wait for activity from SCARD (max. 0.1sec)*/ ;

	if (atomic_read(&(priv->answer2reset)) != 0) { 
		/* got pps reply */
		int is_pps1 = 0, is_pps2 = 0, is_pps3 = 0;

#ifdef SCARD_DEBUG
		DBG_PRINT("PPS Reply: ");
		for (i = 0; i < priv->ans_idx; i++) 
			DBG_PRINT("0x%02x ", priv->ans_string[i]);
		DBG_PRINT("\n");
#endif
		res = 0;
		if ((sizeof(pps) == priv->ans_idx) && (memcmp(pps, priv->ans_string, sizeof(pps)) == 0)) {
			/* set ETU back to what's indicated */
			DBG_PRINT("Reset ETU to %d\n", (fi_tab[priv->fi] / di_tab[priv->di]) - 1);
			gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), 
					(gbus_read_reg32(SCARD_EGT_ETU_REG(priv->base_addr)) & 0xffff0000) | 
						(((fi_tab[priv->fi] / di_tab[priv->di]) - 1) << 0));
			return(res);
		} else {
			is_pps1 = (priv->ans_string[1] & 0x10) ? 1 : 0;
			is_pps2 = (priv->ans_string[1] & 0x20) ? 1 : 0;
			is_pps3 = (priv->ans_string[1] & 0x40) ? 1 : 0;
			if ((pps[1] & 0xf) == (priv->ans_string[1] & 0xf)) {
				if (priv->ans_string[1] & 0x10) {
					if (priv->ans_string[2] != pps[2])
						--res;
					else {
						DBG_PRINT("Reset ETU to %d\n", (fi_tab[priv->fi] / di_tab[priv->di]) - 1);
						gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), 
							(gbus_read_reg32(SCARD_EGT_ETU_REG(priv->base_addr)) & 0xffff0000) | 
								(((fi_tab[priv->fi] / di_tab[priv->di]) - 1) << 0));
					}
				}
				if ((priv->ans_string[1] & 0x10) == 0) {
					if (is_pps1)
						--res;
					else {
						DBG_PRINT("Default ETU of 371 is used.\n");
						gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), 
							(gbus_read_reg32(SCARD_EGT_ETU_REG(priv->base_addr))
								& 0xffff0000) | ((372 -1) << 0));
					}
				}
				if (is_pps2)
					--res;
				if (is_pps3)
					--res;
			} else 
				--res;
		}
	} 
#ifdef SCARD_DEBUG
	else {
		DBG_PRINT("Timeout waiting for PPS reply.\n");
	}
#endif
	return(res);
}

/* Parse and handle "answer to reset" string */
static int parse_ans2reset(struct scard_private *priv, int v3, const unsigned char str[], const int len)
{
	int idx = 2, i = 1;
	int is_ta = 0, is_tb = 0, is_tc = 0, is_td = 0, is_tck = 0, k = 0, ta2 = 0;
	unsigned char ta = 0xff, tb = 0xff, tc = 0xff, td = 0xff, tx = 0xff;
	unsigned char tck = 0xff;
	unsigned char t_array[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	priv->fi = priv->di = 0;
	priv->volt3 = v3;
	switch(str[0]) {
		case 0x3b: /* Correct config detected */
		case 0x3f:
			DBG_PRINT("Correct ts detected 0x%x\n", str[0]);
			break;
		case 0xc4:
		case 0xc0:
			DBG_PRINT("Wrong ts detected 0x%x (need to reverse the polarity)\n", str[0]);
			priv->rev_pol = 1;
			goto reset_needed;
			break;
		case 0xdc:
		case 0xfc:
			DBG_PRINT("Wrong ts detected 0x%x (need to reverse the MSB)\n", str[0]);
			priv->rev_msb = 1;
			goto reset_needed;
			break;
		case 0x23:
		case 0x03:
			DBG_PRINT("Wrong ts detected 0x%x (need to reverse the polarity and MSB)\n", str[0]);
			priv->rev_msb = priv->rev_pol = 1;
			goto reset_needed;
			break;
		default:
			priv->reset_cnt++;
			if (priv->reset_cnt > 3)
				goto not_valid;
			else {
				switch(priv->reset_cnt) {
					case 1: priv->rev_pol = (priv->rev_pol != 0) ? 0 : 1;
						break;
					case 2: priv->rev_msb = (priv->rev_msb != 0) ? 0 : 1;
						break;
					case 3: priv->rev_pol = (priv->rev_pol != 0) ? 0 : 1;
						priv->rev_msb = (priv->rev_msb != 0) ? 0 : 1;
						break;
				}
				goto reset_needed;
			}
			break;
	}
	k = str[1] & 0xf;

	DBG_PRINT("Parsing answer string: t0=0x%x k=%d\n", str[1], k);
	
	for (tx = str[1]; idx < len; i++) {
		is_ta = ((tx & 0x10) != 0);	
		is_tb = ((tx & 0x20) != 0);
		is_tc = ((tx & 0x40) != 0);
		is_td = ((tx & 0x80) != 0);
		DBG_PRINT("*** Iteration %d (idx = %d)\n", i, idx);
#ifdef SCARD_DEBUG
               {
                       int j;
                       DBG_PRINT("Parsing: ");
                       for (j = idx; j < len; j++)
                               DBG_PRINT("0x%x ", str[j]);
                       DBG_PRINT("\n");
               }
#endif

		if (i == 1) {
			unsigned char fi = 0xff, di = 0xff, ii = 0xff, pi1 = 0xff, n = 0xff;
			if (is_ta != 0) {
				ta = str[idx++];
				priv->fi = fi = (ta & 0xf0) >> 4;
				priv->di = di = (ta & 0xf);
				/* Resetting ETU value */
				DBG_PRINT("Reset ETU to %d\n", (fi_tab[fi] / di_tab[di]) - 1);
				gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), 
						(gbus_read_reg32(SCARD_EGT_ETU_REG(priv->base_addr)) & 0xffff0000) | (((fi_tab[fi] / di_tab[di]) - 1) << 0));
				DBG_PRINT("ta=0x%x (fi=0x%x di=0x%x) ", ta, fi, di);
			}
			if (is_tb != 0) { 
				tb = str[idx++];
				ii = (tb & 0x60) >> 5;
				pi1 = (tb & 0x1f);
				DBG_PRINT("tb=0x%x (ii=0x%x pi1=0x%x) ", tb, ii, pi1);
			}
			if (is_tc != 0) {
				tc = str[idx++];
				n = tc;
				DBG_PRINT("tc=0x%x (n=0x%x) ", tc, n);
			}
			if (is_td != 0) {
				td = str[idx++];
				t_array[td & 0xf]++;
				tx = td;
				DBG_PRINT("td=0x%x", td);
			} else {
				t_array[0]++;
			}
			DBG_PRINT("\n");
			if (is_tc != 0) {
				if (n == 0xff) {
					if (t_array[0] != 0) {
						n = 3 - 1;
						DBG_PRINT("Reset EGT to %d\n", n);
						gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), (gbus_read_reg32(SCARD_EGT_ETU_REG(priv->base_addr)) & 0xffff) | (n << 16));
					} else if (t_array[1] != 0) {
						n = 2 - 1;
						DBG_PRINT("Reset EGT to %d\n", n);
						gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), (gbus_read_reg32(SCARD_EGT_ETU_REG(priv->base_addr)) & 0xffff) | (n << 16));
					}
				} else {
					if (t_array[15] == 0) {
						n = 2 + n - 1;
						DBG_PRINT("Reset EGT to %d\n", n);
						gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), (gbus_read_reg32(SCARD_EGT_ETU_REG(priv->base_addr)) & 0xffff) | (n << 16));
					} else {
						n = 2 + n - 1;
						DBG_PRINT("Reset EGT to %d\n", n);
						gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), (gbus_read_reg32(SCARD_EGT_ETU_REG(priv->base_addr)) & 0xffff) | (n << 16));
					}
				}
			}
		} else if (i == 2) {
			unsigned char pi2 = 0xff;
			if (is_ta != 0) {
				ta = str[idx++];
				DBG_PRINT("ta=0x%x ", ta);
				ta2 = 1;
			} else
				ta2 = 0;
			if (is_tb != 0) { 
				tb = str[idx++];
				pi2 = tb;
				DBG_PRINT("tb=0x%x (pi2=0x%x) ", tb, pi2);
			}
			if (is_tc != 0) {
				tc = str[idx++];
				DBG_PRINT("tc=0x%x ", tc);
			}
			if (is_td != 0) {
				td = str[idx++];
				t_array[td & 0xf]++;
				tx = td;
				DBG_PRINT("td=0x%x", td);
			}
			DBG_PRINT("\n");
			if ((is_ta != 0) && ((ta & 0x80) != 0) && ((ta & 0x0f) == 1) && (t_array[1] != 0)) {
				DBG_PRINT("Disabling txerr_check, rxerr_assert.\n");
				gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
						gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) & ~((1 << 6) | (1 << 16)));
			}
		} else {
			unsigned char xi = 0xff, ui = 0xff;
			if (is_ta != 0) {
				ta = str[idx++];
				if (t_array[15] != 0) {
					xi = (ta & 0xc0) >> 6;
					ui = (ta & 0x3f);
					/* xi = 0 <-- no clock stop */
					/* xi = 1 <-- clock stop on low */
					/* xi = 2 <-- clock stop on high */
					/* xi = 3 <-- NA */
					if (xi == 1) { 
						gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
								gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) & ~(1 << 28));
						priv->clockstop = 1;
					} else if (xi == 2) {
						gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
								gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) | (1 << 28));
						priv->clockstop = 1;
					} else
						priv->clockstop = 0;
					DBG_PRINT("ta=0x%x (xi=0x%x ui=0x%x) ", ta, xi, ui);
				} else {
					DBG_PRINT("ta=0x%x ", ta);
				}
			}
			if (is_tb != 0) { 
				tb = str[idx++];
				DBG_PRINT("tb=0x%x ", tb);
			}
			if (is_tc != 0) {
				tc = str[idx++];
				DBG_PRINT("tc=0x%x ", tc);
			}
			if (is_td != 0) {
				td = str[idx++];
				t_array[td & 0xf]++;
				tx = td;
				DBG_PRINT("td=0x%x", td);
			}
			DBG_PRINT("\n");
		}

		if (is_td == 0) 
			break;
		else {
			is_ta = is_tb = is_tc = is_td = 0;
			ta = tb = tc = td = 0xff;
		}
	}		

	idx += k;

#ifdef SCARD_DEBUG
	DBG_PRINT("T = x is set for ");
	for (i = 0; i < 16; i++) {
		if (t_array[i] != 0)
			DBG_PRINT("%d ", i);
	}
	DBG_PRINT("\n");
#endif
	
	if (t_array[0] != 0) {
		for (i = 1; i < 16; i++) {
			if (t_array[i] != 0)
				break;
		}
		if (i < 16) {
			is_tck = 1;
			tck = str[idx++];
		}
	} else {
		is_tck = 1;
		tck = str[idx++];
	}

	if (idx != len) {
		/* Check to see if T=0 protocol is used */
		if ((t_array[0] != 0) && ((idx + 2) == len)) {
			if ((str[idx] != 0x90) || (str[idx + 1] != 0x00)) {
				DBG_PRINT("Length mismatched: %d vs. %d\n", idx, len);
				goto not_valid;
			}
		} else {
			DBG_PRINT("Length mismatched: %d vs. %d\n", idx, len);
			goto not_valid;
		}
	} 
	if (is_tck != 0) {
		for (k = 0, i = 1; i < len; i++)
			k ^= str[i];
		if (k != 0) {
			DBG_PRINT("TCK sum mismatched: 0x%x\n", k);
			goto not_valid;
		}
	} else {
		DBG_PRINT("No TCK needed.\n");
	}

	return((ta2 == 0) ? ATR_WITH_NEGOTIATION : ATR_DONE);

reset_needed:
	return(ATR_RESEND);

not_valid:
	return(ATR_INVALID);
}

/* Activate scard */
static int activate_scard(struct scard_private *priv)
{
	unsigned long flags;
	int ret = -1;

	MSG_PRINT("%s: scard activation starts ..\n", priv->devname);

	spin_lock_irqsave(&priv->lock, flags);

	if (priv->volt3 != 0) {
		DBG_PRINT("%s: Try 3V ..\n", priv->devname);
		scard_ctrlpin_write(priv->idx, priv->scard_5v_pin, 0); /* Use 3V */
		scard_ctrlpin_write(priv->idx, priv->scard_cmd_pin, 0); /* Drive CMDVCC low */
		udelay(RST_WAIT);

		/* Drive RSTIN low */
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x20);
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x620);
		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x6620);
		udelay(RST_WIDTH);
		gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0x3f); /* Re-enable interrupt */
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) | 0x20);
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) | 0x620);
		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) | 0x6620);
		gbus_write_reg32(SCARD_STATE_REG(priv->base_addr), (unsigned long)PWR_IDLE);

		spin_unlock_irqrestore(&priv->lock, flags);

		scard_schedule_timeout(HZ); /* Wait for activity from SCARD (max. 1sec)*/ ;

		gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0); /* disable interrupt */
		spin_lock_irqsave(&priv->lock, flags);
	
		if (atomic_read(&(priv->answer2reset)) != 0) { 
#ifdef SCARD_DEBUG
			int i = 0;
			DBG_PRINT("Got response from 3V setting ..\nAnswering: ");
			for (i = 0; i < priv->ans_idx; i++)
				DBG_PRINT("0x%02x ", priv->ans_string[i]);
			DBG_PRINT("\n"); 
#endif
			if ((ret = parse_ans2reset(priv, 1, priv->ans_string, priv->ans_idx)) == ATR_INVALID) {
				DBG_PRINT("Ans2reset: string error\n");
				scard_reset(priv, 1); /* cold reset */
			} else if (ret == ATR_RESEND) {
				/* Perform warm reset */
				DBG_PRINT("Ans2reset: warm reset needed.\n");
				gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x40);
				gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
							gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) | (1 << 2));
				scard_reset(priv, 0); /* warm reset */
				goto out;
			} else {
				if (ret == ATR_WITH_NEGOTIATION) {
					int stat;
					spin_unlock_irqrestore(&priv->lock, flags);
					gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0x3f); /* Re-enable interrupt */
					stat = scard_negotiate(priv);
					gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0); /* disable interrupt */
					spin_lock_irqsave(&priv->lock, flags);

					if (stat < 0) {
						/* negotiation failed */
						DBG_PRINT("Negotiation failure, rejecting ..\n");
						scard_reset(priv, 1); /* cold reset */
					} else {
						/* The card is activated so we can turn on interrupts */
						gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0x3f); /* Re-enable interrupt */
					}
				} else {
					/* The card is activated so we can turn on interrupts */
					gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0x3f); /* Re-enable interrupt */
				}
			}
		}
	}

	if ((priv->volt3 == 0) || (atomic_read(&(priv->answer2reset)) == 0)) {	/* 3V doesn't get response */
		scard_ctrlpin_write(priv->idx, priv->scard_cmd_pin, 1); /* Drive CMDVCC high to deactive */
		udelay(1000);

		DBG_PRINT("%s: Try 5V ..\n", priv->devname);
		scard_ctrlpin_write(priv->idx, priv->scard_5v_pin, 1); /* Use 5V */
		scard_ctrlpin_write(priv->idx, priv->scard_cmd_pin, 0); /* Drive CMDVCC low */
		udelay(RST_WAIT);

		/* Drive RSTIN low */
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x20);
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x620);
		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x6620);
		udelay(RST_WIDTH);
		gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0x3f); /* Re-enable interrupt */
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) | 0x20);
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) | 0x620);
		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) | 0x6620);
		gbus_write_reg32(SCARD_STATE_REG(priv->base_addr), (unsigned long)PWR_IDLE);

		spin_unlock_irqrestore(&priv->lock, flags);

		scard_schedule_timeout(HZ); /* Wait for activity from SCARD (max. 1sec)*/ ;

		gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0); /* disable interrupt */
		spin_lock_irqsave(&priv->lock, flags);
	
		if (atomic_read(&(priv->answer2reset)) != 0) { 
#ifdef SCARD_DEBUG
			int i;
			DBG_PRINT("Got response from 5V setting ..\nAnswering: ");
			for (i = 0; i < priv->ans_idx; i++)
				DBG_PRINT("0x%02x ", priv->ans_string[i]);
			DBG_PRINT("\n"); 
#endif
			if ((ret = parse_ans2reset(priv, 0, priv->ans_string, priv->ans_idx)) == ATR_INVALID) {
				DBG_PRINT("Ans2reset: string error\n");
				scard_reset(priv, 1); /* cold reset */
			} else if (ret == ATR_RESEND) {
				/* Perform warm reset */
				DBG_PRINT("Ans2reset: warm reset needed.\n");
				gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x40);
				gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
							gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) | (1 << 2));
				scard_reset(priv, 0); /* warm reset */
				goto out;
			} else {
				if (ret == ATR_WITH_NEGOTIATION) {
					int stat;
					spin_unlock_irqrestore(&priv->lock, flags);
					gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0x3f); /* Re-enable interrupt */
					stat = scard_negotiate(priv);
					gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0); /* disable interrupt */
					spin_lock_irqsave(&priv->lock, flags);

					if (stat < 0) {
						/* negotiation failed */
						DBG_PRINT("Negotiation failure, rejecting ..\n");
						scard_reset(priv, 1); /* cold reset */
					} else {
						/* The card is activated so we can turn on interrupts */
						gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0x3f); /* Re-enable interrupt */
					}
				} else {
					/* The card is activated so we can turn on interrupts */
					gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0x3f); /* Re-enable interrupt */
				}
			}
		}
	}

out:
	if (atomic_read(&(priv->answer2reset)) != 0) {
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x600);
//		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) | 0x600);
		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x6600);
		gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) | 0x6600);
		MSG_PRINT("%s: got response from the scard --> switch to normal\n", priv->devname);
		atomic_set(&(priv->normal_mode), 1);
		ret = 1;
	} else if (ret < 0) {
		MSG_PRINT("%s: no response from the scard --> reject\n", priv->devname);
		scard_reset(priv, 1); /* cold reset */
		ret = 0;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return(ret);
}

/* Reading from driver's buffer, note that it can return read size
   less than specified */
static int scard_read(struct file *fptr, char *buffer, size_t count, loff_t *fp)
{
	struct scard_private *priv = (struct scard_private *)fptr->private_data;
	unsigned long flags;
	unsigned char buf[rd_buffer_size], *bufptr = &buf[0];
	int cnt = 0;
	
	if (atomic_read(&(priv->card_existed)) == 0)
		return(-EIO);
	else if (atomic_read(&(priv->normal_mode)) == 0)
		return(-EIO);
	
	/* If block mode is on, check the emptiness of buffer */
	if (priv->b_mode != 0) {
		/* Sleep when buffer is empty */
		wait_event_interruptible(priv->scard_rd_wq, priv->rd_c_idx != priv->rd_p_idx);

		if (atomic_read(&(priv->card_existed)) == 0) 
			return(-EIO);
		else if (atomic_read(&(priv->normal_mode)) == 0) 
			return(-EIO);
	}
	if (signal_pending(current))
		return(cnt);

	spin_lock_irqsave(&priv->lock, flags);

	/* Get the data out and adjust consumer index */
	for (cnt = 0; (priv->rd_c_idx != priv->rd_p_idx) && (cnt < count); cnt++) {
		*bufptr = priv->rd_buffer[priv->rd_c_idx];
		priv->rd_c_idx = (priv->rd_c_idx + 1) % rd_buffer_size;
		bufptr++;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	/* Get the data to user */
	if (cnt && copy_to_user(buffer, (char *)&buf[0], cnt)) 
		return(-EFAULT);

	return(cnt);
}

/* Writing to smart card device */
static int scard_write(struct file *fptr, const char *bufptr, size_t size, loff_t *fp)
{
	struct scard_private *priv = (struct scard_private *)fptr->private_data;
	unsigned long flags, cj;
	unsigned char buffer[wr_buffer_size];
	unsigned wr_pidx, wr_cidx;
	int count = 0;
	
	if (atomic_read(&(priv->card_existed)) == 0) {
		printk("Card not there\n");
		return(-EIO);
	} else if (atomic_read(&(priv->normal_mode)) == 0) {
		printk("Card not ready\n");
		return(-EIO);
	}
	
	if (size > wr_buffer_size) {
		ERR_PRINT("%s: exceed max. write size (%d)\n", 
			priv->devname, wr_buffer_size);
		return(-EIO);
	} else if (copy_from_user(&buffer[0], bufptr, size))
		return(-EFAULT);

	spin_lock_irqsave(&priv->lock, flags);

	cj = jiffies;
	while ((priv->last_rd_jiffies != 0) && time_after_eq(priv->last_rd_jiffies + TIMEOUT, cj)) {
		spin_unlock_irqrestore(&priv->lock, flags);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout((priv->last_rd_jiffies + TIMEOUT) - cj);
		spin_lock_irqsave(&priv->lock, flags);
		cj = jiffies;
	}

	wr_pidx = priv->wr_p_idx;	/* Save the old index before proceeding */
	wr_cidx = priv->wr_c_idx;

	for (count = 0; count < size; count++) {
		DBG_PRINT("%s: write data 0x%x\n", priv->devname, buffer[count]);
		/* Save it to buffer */
		if (((priv->wr_p_idx + 1) % wr_buffer_size) == wr_cidx) {
			ERR_PRINT("%s: write buffer full\n", priv->devname);
			goto out;
		}

		priv->wr_buffer[priv->wr_p_idx] = buffer[count];
		priv->wr_p_idx = (priv->wr_p_idx + 1) % wr_buffer_size;

	}

out:
	spin_unlock_irqrestore(&priv->lock, flags);

	/* Buffer was empty, trigger the writer */
	if (wr_cidx == wr_pidx) 
		scard_write_device(priv);

	return(count);
}

/* ioctl function */
static int scard_ioctl(struct inode *inode, struct file *fptr, unsigned int cmd, unsigned long arg)
{
//	unsigned long *ptr = (unsigned long *)arg;
	struct scard_private *priv = (struct scard_private *)fptr->private_data;
	
	if (atomic_read(&(priv->card_existed)) == 0)
		return(-EIO);
	else if (atomic_read(&(priv->normal_mode)) == 0)
		return(-EIO);
	
//	if (ptr == NULL)
//		return(-EIO);

	switch(cmd) {
		case SCARD_IOC_WARMRESET:
			gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x40);
			gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
						gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) | (1 << 2));
			udelay(1);
			scard_reset(priv, 0); /* warm reset */
			break;
		case SCARD_IOC_CLOCKSTOP:
			if (priv->clockstop == 0) 
				return(-EIO);	
			gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
						gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) | (1 << 3));
			udelay(1);
			atomic_set(&(priv->state), (int)CLOCK_STOP);
			break;
		case SCARD_IOC_CLOCKSTART:
			if (priv->clockstop == 0) 
				return(-EIO);	
			gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
					gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) & ~(1 << 3));
			udelay(1);
			atomic_set(&(priv->state), (int)PWR_IDLE);
			break;
		default:
                        return(-EIO);
	}
	return(0);
}

/* Poll function */
static unsigned int scard_poll(struct file *fptr, struct poll_table_struct *ptable)
{
	struct scard_private *priv = (struct scard_private *)fptr->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	if (atomic_read(&(priv->card_existed)) == 0)
		return(POLLERR);
	else if (atomic_read(&(priv->normal_mode)) == 0)
		return(POLLERR);

	spin_lock_irqsave(&priv->lock, flags);
	poll_wait(fptr, &(priv->scard_rd_wq), ptable);
	if (priv->rd_c_idx != priv->rd_p_idx)
		mask |= (POLLIN | POLLRDNORM);
	spin_unlock_irqrestore(&priv->lock, flags);

	return(mask);
}

/* Open the device */
static int scard_open(struct inode *inode_ptr, struct file *fptr)
{
	int minor = MINOR(inode_ptr->i_rdev);

	if (minor >= num_scard)
		return(EIO);
	else if (atomic_read(&(scard_priv[minor].card_existed)) == 0)
		return(-EIO);
	else if (atomic_read(&(scard_priv[minor].normal_mode)) == 0)
		return(-EIO);
	
	/* This device is exclusive, that is, only one process can use it */
	if (scard_priv[minor].ref_cnt != 0) {
		ERR_PRINT("%s: exclusive access only\n", scard_priv[minor].devname);
		return(-EIO);
	} 

	/* Set the block mode and increase reference count */
	scard_priv[minor].ref_cnt++;
	scard_priv[minor].b_mode = ((fptr->f_flags & O_NONBLOCK) ? 0 : 1);
	scard_priv[minor].last_rd_jiffies = jiffies;
	scard_priv[minor].last_wr_jiffies = jiffies;

	/* Flush the buffer */
	scard_priv[minor].rd_p_idx = scard_priv[minor].rd_c_idx = 0;
	scard_priv[minor].wr_p_idx = scard_priv[minor].wr_c_idx = 0;

	fptr->f_op = &scard_fops;
	fptr->private_data = (void *)&scard_priv[minor];

	atomic_set(&scard_priv[minor].state, (int)PWR_IDLE);

	return(0);
}

/* Close the device */
static int scard_release(struct inode *inode_ptr, struct file *fptr) 
{
	struct scard_private *priv = (struct scard_private *)fptr->private_data;
	unsigned long flags;

	if (atomic_read(&(priv->card_existed)) == 0)
		goto error_out;
	else if (atomic_read(&(priv->normal_mode)) == 0)
		goto error_out;

	spin_lock_irqsave(&priv->lock, flags);

	/* Reset the RX/TX FIFOs */
	gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
				gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) | ((1 << 24) | (1 << 25)));
	gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
				gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) & ~((1 << 24) | (1 << 25)));

	/* Adjust reference count */
	priv->ref_cnt--;
	priv->rd_p_idx = priv->rd_c_idx = 0;
	priv->wr_p_idx = priv->wr_c_idx = 0;

	spin_unlock_irqrestore(&priv->lock, flags);

	return(0);

error_out:
	/* Adjust reference count */
	priv->ref_cnt--;
	return(-EIO);
}

/* Reset scard to initial state */
static void scard_reset(struct scard_private *priv, int cold)
{
	unsigned long mask;
	unsigned int nclk;

	gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0); /* disable interrupt */

	if (cold != 0) { /* Cold reset */
		priv->volt3 = use_3v;
		priv->rev_msb = reverse_msb;
		priv->rev_pol = reverse_pol;
	}
	priv->ans_idx = 0;
	priv->clockstop = 0;
	priv->rd_p_idx = priv->rd_c_idx = 0;
	priv->wr_p_idx = priv->wr_c_idx = 0;

	/* Set CMDVCC high and Set RSTIN high */
	scard_ctrlpin_write(priv->idx, priv->scard_cmd_pin, 1); 

	if (priv->volt3 != 0)
		scard_ctrlpin_write(priv->idx, priv->scard_5v_pin, 0); 
	else
		scard_ctrlpin_write(priv->idx, priv->scard_5v_pin, 1); 

	gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) | 0x60);

	/* Initialize everything to be in known state */
	nclk = tangox_get_sysclock() / (frequency * 2);
	gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) & 0xffffffe0);
	gbus_write_reg32(SCARD_CLK_HIGH_VAL(priv->base_addr), nclk); 
	gbus_write_reg32(SCARD_CLK_LOW_VAL(priv->base_addr), nclk); 

	/* Th = (3 * 256) = 768, Tg = 2048, Tb = 512, Tc = 39936 */
	gbus_write_reg32(SCARD_PARAM_REG(priv->base_addr), (3 << 20) | (8 << 16) | (2 << 12) | (0x9c << 0));
	/* EGT = 4, ETU = 372 */
	gbus_write_reg32(SCARD_EGT_ETU_REG(priv->base_addr), ((4 - 1) << 16) | ((372 - 1) << 0));

	/* set up initial value (includes resetting FIFO */
	mask = (1 << 5) | (1 << 6) | (1 << 8) | (3 << 9) | (3 << 13) | (1 << 16) | 
		(2 << 17) | (1 << 20) | (1 << 24) | (1 << 25) | (1 << 27) | (1 << 7) | (1 << 12);
	if (priv->rev_msb != 0)
		mask ^= (1 << 7);
	if (priv->rev_pol != 0)
		mask ^= (1 << 12);
	gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), mask);

	udelay(1);

	/* Un-reset FIFO */
	gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
				gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) & ~((1 << 24) | (1 << 25)));
	gbus_write_reg32(SCARD_STATE_REG(priv->base_addr), 0);

	atomic_set(&(priv->card_existed), 0);
	atomic_set(&(priv->state), (int)UNKNOWN);
	atomic_set(&(priv->normal_mode), 0);
	atomic_set(&(priv->answer2reset), 0);

//	gbus_write_reg32(SCARD_INTEN_REG(priv->base_addr), 0x3f); /* Re-enable interrupt */
}

/* Monitor the status of smart card */
static int scard_monitor(void *priv_data)
{
	struct scard_private *priv = (struct scard_private *)priv_data;
	enum SCARD_STATE cur_state, set_state;
	int p_card_state;
	int c_card_state;
	int delay;

	daemonize(priv->devname);

        while (priv->stop_thread == 0) {
		p_card_state = atomic_read(&(priv->card_existed));
		c_card_state = is_card_present(priv);
//		delay = HZ / 5; /* Default 0.2 sec */
		delay = HZ; /* Default 1 sec */

//		spin_lock(&priv->lock);

		if (p_card_state != c_card_state) {
			DBG_PRINT("%s: card state change %d->%d\n", priv->devname,
					p_card_state, c_card_state);
			/* Update the state */
			atomic_set(&(priv->card_existed), c_card_state);

			/* Detected transition of card present status */
			if (c_card_state != 0) { /* Card is here now */
				MSG_PRINT("%s: scard present detected.\n", priv->devname);
				atomic_set(&(priv->card_existed), 1);
				atomic_set(&(priv->state), (int)PWR_IDLE);
				atomic_set(&(priv->normal_mode), 0);
				priv->reset_cnt = 0;
				gbus_write_reg32(SCARD_STATE_REG(priv->base_addr), 0);
				gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
						gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) & 0xffffffe0);
			} else { /* Card removed */
				MSG_PRINT("%s: scard removal detected.\n", priv->devname);
				scard_reset(priv, 1);
				priv->rd_p_idx = priv->rd_c_idx = 0;
				priv->wr_p_idx = priv->wr_c_idx = 0;
				wake_up_interruptible(&(priv->scard_rd_wq));
				goto out;
			}
		} else if (c_card_state != 0) {
			if (sys_freq != tangox_get_sysclock()) { /* If frequency changed, reset is needed */
				gbus_write_reg32(SCARD_SOFT_OUT_REG(priv->base_addr), gbus_read_reg32(SCARD_SOFT_OUT_REG(priv->base_addr)) & ~0x40);
				gbus_write_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr), 
						gbus_read_reg32(SCARD_SCARD_CTRL_REG(priv->base_addr)) | (1 << 2));
				udelay(1);
				scard_reset(priv, 0); /* warm reset */
				sys_freq = tangox_get_sysclock();
				goto out;
			}
		} else if (c_card_state == 0)
			goto out;

		/* We have card here, check the state transition */
		cur_state = (enum SCARD_STATE)(gbus_read_reg32(SCARD_STATE_REG(priv->base_addr)) & 0x7);
		if (!PERSISTENT_STATE(cur_state)) {
//			delay = HZ / 10; /* Change to 0.1 sec */
			delay = HZ / 2; /* Change to 0.5 sec */
			goto out;
		} 

		set_state = (enum SCARD_STATE)atomic_read(&(priv->state));
		if ((set_state == cur_state) || (set_state == UNKNOWN)) {
//			delay = HZ / 2;	/* State remain stable -- change to 0.5 sec */
			delay = HZ;	/* State remain stable -- change to 1 sec */
			goto out;
		} 

		DBG_PRINT("%s: expect state (%d) != current state(%d)\n", priv->devname,
				set_state, cur_state);
		if (set_state == PWR_IDLE) 
			activate_scard(priv);
		else 
			ERR_PRINT("%s: WARNING: inconsistent state %d\n", priv->devname, set_state);

out:
//		spin_unlock(&priv->lock);

                set_current_state(TASK_UNINTERRUPTIBLE);
                schedule_timeout(delay);
        }

	return(0);
}

int __init scard_init_module(void)
{
	int status = 0;
	int i;
	int xenv_scard_5v_pin, xenv_scard_off_pin, xenv_scard_cmd_pin;

#ifdef CONFIG_TANGO3
	unsigned long tangox_chip_id(void);
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;

	if (chip_id == 0x8652) 
		num_scard = 1;
#endif

	for (status = i = 0; i < num_scard; i++) {
		if (tangox_scard_enabled(i))
			status++;
	}

	if (status == 0) {
		ERR_PRINT("%s: device is disabled.\n", scard_devname);
		return(0);
	} else if (rd_buffer_size < 8) {
		ERR_PRINT("%s: read buffer size (%d) error, minimum 8.\n", scard_devname,
			rd_buffer_size); 
		return(-EIO);
	} else if (wr_buffer_size < 8) {
		ERR_PRINT("%s: write buffer size (%d) error, minimum 8.\n", scard_devname,
			wr_buffer_size); 
		return(-EIO);
	} 
	
	memset(&scard_priv[0], 0, sizeof(struct scard_private) * num_scard);

#ifdef CONFIG_TANGO3
	if (chip_id == 0x8652) {
		gbus_write_reg32(REG_BASE_cpu_block + CPU_UART0_base + 0x3c, 4);
		gbus_write_reg32(REG_BASE_cpu_block + CPU_UART0_base + 0x38, 0xff00);
	}
#endif

	/* Initialize private data structure */
	for (i = 0; i < num_scard; i++) {
		spin_lock_init(&scard_priv[i].lock);
		scard_priv[i].enabled = (tangox_scard_enabled(i) ? 1 : 0);
		if (scard_priv[i].enabled == 0) {
			MSG_PRINT("%s%d: interface is disabled.\n", scard_devname, i);
			continue;
		}

		if ((scard_priv[i].rd_buffer = kmalloc(rd_buffer_size, GFP_KERNEL)) == NULL) {
			ERR_PRINT("%s%d: out of memory for read buffer\n", scard_devname, i); 
			status = -ENOMEM;
			goto err_out1;
		} else if ((scard_priv[i].wr_buffer = kmalloc(wr_buffer_size, GFP_KERNEL)) == NULL) {
			ERR_PRINT("%s%d: out of memory for write buffer\n", scard_devname, i); 
			status = -ENOMEM;
			goto err_out1;
		}

		scard_priv[i].idx = i;
		scard_priv[i].base_addr = scard_ctrlers[i].base_addr;
		scard_priv[i].scard_5v_pin = scard_ctrlers[i].scard_5v_pin;
		scard_priv[i].scard_cmd_pin = scard_ctrlers[i].scard_cmd_pin;
		scard_priv[i].scard_off_pin = scard_ctrlers[i].scard_off_pin;
		scard_priv[i].scard_irq = scard_ctrlers[i].scard_irq;
		sprintf(scard_priv[i].devname, "%s%d", scard_devname, i);

		/* override the default from XENV settings */
		tangox_get_scard_info(i, &xenv_scard_5v_pin, &xenv_scard_cmd_pin, &xenv_scard_off_pin);
		scard_priv[i].scard_5v_pin = xenv_scard_5v_pin;
		scard_priv[i].scard_cmd_pin = xenv_scard_cmd_pin;
		scard_priv[i].scard_off_pin = xenv_scard_off_pin;
#ifdef CONFIG_TANGO3
		if (chip_id == 0x8652) {
			if (xenv_scard_5v_pin > 2) /* GPIO pin is used */
				em86xx_gpio_setdirection(xenv_scard_5v_pin, 0); /* Set to input */
			if (xenv_scard_cmd_pin > 2) /* GPIO pin is used */
				em86xx_gpio_setdirection(xenv_scard_cmd_pin, 0); /* Set to input */
			if (xenv_scard_off_pin > 2) /* GPIO pin is used */
				em86xx_gpio_setdirection(xenv_scard_off_pin, 0); /* Set to input */
		}
#endif
		init_waitqueue_head(&(scard_priv[i].scard_rd_wq));
	}

	/* Register device, and may be allocating major# */
	status = register_chrdev(major_num, scard_devname, &scard_fops);
	if (status < 0) {
		ERR_PRINT("%s: cannot get major number\n", scard_devname); 
		goto err_out1;
	} else if (major_num == 0)
		major_num = status;	/* Dynamic major# allocation */

	for (i = 0; i < num_scard; i++) {
		if (scard_priv[i].enabled == 0)
			continue;
		/* Hook up ISR */
		if (request_irq(scard_priv[i].scard_irq, scard_isr, IRQF_DISABLED, scard_priv[i].devname, &scard_priv[i]) != 0) {
			ERR_PRINT("%s: cannot register IRQ (%d)\n", scard_priv[i].devname, scard_priv[i].scard_irq);
			status = -EIO;
			goto err_out2;
		}
	}

#ifdef CONFIG_EM86XX_SMARTCARD
	MSG_PRINT("%s: driver loaded (rd_buffer_size = %d, wr_buffer_size = %d)\n", 
		scard_devname, rd_buffer_size, wr_buffer_size);
#else
	MSG_PRINT("%s: SMP86xx driver loaded (rd_buffer_size = %d, wr_buffer_size = %d, freq = %d, interfaces = %d)\n",
		scard_devname, rd_buffer_size, wr_buffer_size, frequency, num_scard);
#endif

	sys_freq = tangox_get_sysclock();

	for (i = 0; i < num_scard; i++) {
		if (scard_priv[i].enabled == 0)
			continue;
		/* Initialize control pins */
		scard_ctrlpin_init(i);

		/* Reset the interface */
		scard_reset(&scard_priv[i], 1);

		DBG_PRINT("%s: enabled smartcard interface.\n", scard_priv[i].devname);

		scard_priv[i].stop_thread = 0;
		if (kernel_thread(scard_monitor, (void *)&scard_priv[i], CLONE_FS|CLONE_FILES) < 0) {
			status = -ENODEV;
			goto err_out3;
		}
	}
	return(0);

err_out3:
	for (i = 0; i < num_scard; i++) {
		if (scard_priv[i].enabled == 0)
			continue;
		scard_priv[i].stop_thread = 1;
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(3*HZ);

		gbus_write_reg32(SCARD_SCARD_CTRL_REG(scard_priv[i].base_addr), 0);
		gbus_write_reg32(SCARD_STATE_REG(scard_priv[i].base_addr), 0);
		scard_ctrlpin_write(i, scard_priv[i].scard_5v_pin, 0); 
		scard_ctrlpin_write(i, scard_priv[i].scard_cmd_pin, 1); 
	}

err_out2:
	for (i = 0; i < num_scard; i++) {
		if (scard_priv[i].enabled == 0)
			continue;
		free_irq(scard_priv[i].scard_irq, &scard_priv[i]);
	}
	unregister_chrdev(major_num, scard_devname);	

err_out1:
	for (i = 0; i < num_scard; i++) {
		if (scard_priv[i].enabled == 0)
			continue;
		if (scard_priv[i].rd_buffer)
			kfree(scard_priv[i].rd_buffer);
		if (scard_priv[i].wr_buffer)
			kfree(scard_priv[i].wr_buffer);
	}
	return(status);
}

void __exit scard_cleanup_module(void)
{
	int i;

	for (i = 0; i < num_scard; i++) {
		if (scard_priv[i].enabled == 0)
			continue;
		gbus_write_reg32(SCARD_INTEN_REG(scard_priv[i].base_addr), 0);

		scard_priv[i].stop_thread = 1;
	}

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(3*HZ);

	for (i = 0; i < num_scard; i++) {
		gbus_write_reg32(SCARD_SCARD_CTRL_REG(scard_priv[i].base_addr), 0);
		gbus_write_reg32(SCARD_STATE_REG(scard_priv[i].base_addr), 0);
		scard_ctrlpin_write(i, scard_priv[i].scard_5v_pin, 0); 
		scard_ctrlpin_write(i, scard_priv[i].scard_cmd_pin, 1); 

		unregister_chrdev(major_num, scard_devname);
		free_irq(scard_priv[i].scard_irq, &scard_priv[i]);

		if (scard_priv[i].rd_buffer != NULL)
			kfree(scard_priv[i].rd_buffer);
		if (scard_priv[i].wr_buffer != NULL)
			kfree(scard_priv[i].wr_buffer);
		
		MSG_PRINT("%s: driver unloaded\n", scard_devname);
	}
}

module_init(scard_init_module);
module_exit(scard_cleanup_module);

