
/*********************************************************************
 Copyright (C) 2001-2009
 Sigma Designs, Inc. 
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
  
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ide.h>
#include <linux/hdreg.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <asm/ide.h>
#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/tango2api.h>

static DEFINE_SPINLOCK(bmide_tune_chipset_spin_lock);

#define PFX	"tango2_bmide: "

/* some drive cannot use mbus chaining, use bounce buffer instead */
#undef USE_BOUNCE_BUF
#ifdef USE_BOUNCE_BUF
/*maximum = 128 sectors = 2^7 sectors = 2^7 * 2^9 bytes = 2^16 bytes*/
#define DMA_BOUNCE_BUF_ORDER		(19 - PAGE_SHIFT)	
static unsigned char *g_bounce_buf = NULL;
#undef USE_MBUS_MEMCPY
#endif

/*
 * helper to access host interface
 */
#define HWIF(drive)		((ide_hwif_t *)((drive)->hwif))

#define RD_HOST_REG32(r)	\
		gbus_read_reg32(REG_BASE_host_interface + (r))

#define WR_HOST_REG32(r, v)	\
		gbus_write_reg32(REG_BASE_host_interface + (r), (v))

#define RD_HOST_REG8(r)	\
		gbus_read_reg8(REG_BASE_host_interface + (r))

#define WR_HOST_REG8(r, v)	\
		gbus_write_reg8(REG_BASE_host_interface + (r), (v))

#define CONFIG_RUNTIME_CLK_CALC

#ifdef CONFIG_RUNTIME_CLK_CALC
#define TIMING_MARGIN(x) (((x) * 105) / 100) /* add 5% margin */

typedef struct {
	unsigned long cycle_time;
	unsigned long active_time;
	unsigned long recovery_time;
	unsigned long setup_time;
	unsigned long hold_time;
} j2_tim_pio_dma_t;

typedef struct {
	unsigned long t_zah;//
	unsigned long t_mli;//
	unsigned long t_ack;//
	unsigned long t_env;//
	unsigned long t_rp;	//
	unsigned long t_cyc;//
	unsigned long t_cvh;//
	unsigned long t_cvs;//
} j2_tim_udma_t;

static j2_tim_pio_dma_t	j2_pio_timings[] = 
{
//	 cycle	, active, recov	, setup	, hold
	{600	, 165	, 70	, 70	, 30	},
	{383	, 125	, 70	, 50	, 20	},
	{240	, 100	, 70	, 30	, 15	},
	{180	, 80	, 70	, 30	, 10	},
	{120	, 70	, 25	, 25	, 10	},
};

static j2_tim_pio_dma_t	j2_dma_timings[] =
{
//	 cycle	, active, recov	, setup	, hold
	{480	, 215	, 215	, 50	, 20	},
	{150	, 80	, 50	, 30	, 15	},
	{120	, 70	, 25	, 25	, 10	},
};

// We round 16.8 to 17 - Ex: mode 5, t_cyc=16.8!!!
static j2_tim_udma_t j2_udma_timings[] =
{
//	 t_zah,  t_mli, t_ack,  t_env,  t_rp,  t_cyc   t_cvh   t_cvs
	{ 20,	 20,	 20,	20,		160,	120,	  7,	 70	},
	{ 20,	 20,	 20,	20,		125,	 80,	  7,	 48	},
	{ 20,	 20,	 20,	20,		100,	 60,	  7,	 31	},
	{ 20,	 20,	 20,	20,		100,	 45,	  7,	 20	},
	{ 20,	 20,	 20,	20,		100,	 30,	  7,	  7	},
	{ 20,	 20,	 20,	20,		 85,	 20,	 10,	 10	},
	{ 20,	 20,	 20,	20,		 85,	 15,	 10,	 10	},
	{ 20,	 20,	 20,	20,		 85,	 15,	 10,	 10	},
};

static unsigned int CalcRegValTiming_PIO_DMA(j2_tim_pio_dma_t *t, unsigned long ide_clock);
static unsigned int CalcRegValTiming1_UDMA(j2_tim_udma_t *t, unsigned long ide_clock);
static unsigned int CalcRegValTiming2_UDMA(j2_tim_udma_t *t, unsigned long ide_clock);
#endif /* CONFIG_RUNTIME_CLK_CALC */

/*
 * io helpers for PIO access
 */
static void tango2_bmide_outb(u8 data, unsigned long port)
{
	gbus_write_reg16(port, data);
}

static void tango2_bmide_outw(u16 data, unsigned long port)
{
	gbus_write_reg16(port, data);
}

static u8 tango2_bmide_inb(unsigned long port)
{
	u8 val;

	val = ((u8)(gbus_read_reg16(port) & 0xff));
	return val;
}

static u16 tango2_bmide_inw(unsigned long port)
{
	u16 val;

	val = ((u16)(gbus_read_reg16(port) & 0xffff));
	return val;
}
#ifdef CONFIG_BLK_DEV_BMIDE_TANGO2_DMA
/*
 * returns closest pio mode timings from given dma speed
 */
static u8 dma_2_pio(u8 xfer_rate) {

	switch (xfer_rate) {
	case XFER_UDMA_6:
	case XFER_UDMA_5:
	case XFER_UDMA_4:
	case XFER_UDMA_3:
	case XFER_UDMA_2:
	case XFER_UDMA_1:
	case XFER_UDMA_0:
	case XFER_MW_DMA_2:
	case XFER_PIO_4:
		return 4;

	case XFER_MW_DMA_1:
	case XFER_PIO_3:
		return 3;

	case XFER_SW_DMA_2:
	case XFER_PIO_2:
		return 2;

	case XFER_MW_DMA_0:
	case XFER_SW_DMA_1:
	case XFER_SW_DMA_0:
	case XFER_PIO_1:
	case XFER_PIO_0:
	case XFER_PIO_SLOW:
	default:
		return 0;
	}
}

/*
 * dma related callbacks
 */
static unsigned long g_mbus_reg = 0;
static unsigned int g_next_sg = 0;


static u8 tango2_bmide_dma_sff_read_status(ide_hwif_t *hwif)
{
    unsigned long addr = REG_BASE_host_interface + IDECTRL_bmis; 

	return	tango2_bmide_inb(addr);
}

static void tango2_bmide_dma_sff_write_status(ide_hwif_t *hwif, u8 val)
{
    unsigned long addr = REG_BASE_host_interface + IDECTRL_bmis; 

	tango2_bmide_outb(val,  addr);
}

void tango2_bmide_dma_host_set(ide_drive_t *drive, int on)
{
	ide_hwif_t *hwif = drive->hwif;
	u8 unit = drive->dn & 1;
	u8 dma_stat = hwif->dma_ops->dma_sff_read_status(hwif);

	if (on)
		dma_stat |= (1 << (5 + unit));
	else
		dma_stat &= ~(1 << (5 + unit));

	tango2_bmide_dma_sff_write_status(hwif, dma_stat);
}

static void tango2_mbus_intr(int irq, void *arg)
{
	ide_drive_t *drive = (ide_drive_t *)arg;
	ide_hwif_t *hwif = HWIF(drive);
	struct scatterlist *sg;

	if (g_next_sg > hwif->cmd.sg_nents)
		return; /* should be the last interrupt */

	em86xx_mbus_wait(g_mbus_reg, SBOX_IDEDVD);

	/*
	 * setup a new mbus transfer
	 */
	sg = &hwif->sg_table[g_next_sg];
	g_next_sg++;
	
	if (g_next_sg == hwif->cmd.sg_nents) {
		if (em86xx_mbus_setup_dma(g_mbus_reg, sg_dma_address(sg), sg_dma_len(sg),
					  NULL, drive, 1)) {
			printk(KERN_ERR PFX "fail to resetup dma, wait "
			       "for timeout...\n");
		}
	} else {
		if (em86xx_mbus_setup_dma(g_mbus_reg, sg_dma_address(sg), sg_dma_len(sg),
					  tango2_mbus_intr, drive, 0)) {
			printk(KERN_ERR PFX "fail to resetup dma, wait "
			       "for timeout...\n");
		}
	}
}

/* Get the I/O size of given request */
static inline int get_request_size(struct request *rq)
{
	unsigned int rq_size;
	struct bio *bio;
	
	for (rq_size = 0, bio = rq->bio; bio != NULL; bio = bio->bi_next)
		rq_size += bio->bi_size;
	
	return rq_size;
}

static void tango2_bmide_dma_start(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	struct request *rq;
	unsigned long val, len;

	rq = hwif->rq;

	len = get_request_size(rq);

	/* setup IDE DMA transfer len */
	gbus_write_reg32(REG_BASE_host_interface + IDECTRL_ide_dmalen, len);

	/* enable bus mastering */
	val = 0x05;
	if (hwif->cmd.sg_dma_direction == DMA_FROM_DEVICE)
		val |= 0x8;

	WR_HOST_REG32(IDECTRL_bmic, val);
}

static int tango2_bmide_dma_setup(ide_drive_t *drive, struct ide_cmd *cmd)
{
	ide_hwif_t *hwif = HWIF(drive);
	struct request *rq;
	int iswrite ;
	struct scatterlist *sg = hwif->sg_table;

	rq = hwif->rq;

	iswrite = (rq_data_dir(rq) == WRITE);
	cmd->sg_dma_direction = iswrite ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	/*
	 * ide_map_sg will merge contiguous memory zone for us
	 */
	ide_map_sg(drive, cmd);

	/* try to setup dma channel */
	if (em86xx_mbus_alloc_dma(SBOX_IDEDVD, iswrite ? 0 : 1, &g_mbus_reg,
				  NULL, 0)) {
		printk(KERN_ERR PFX "fail to alloc dma, fallback to pio\n");
		goto fallback_pio;
	}

	/*
	 * map and transfer first segment
	 */
	dma_map_sg(&hwif->gendev, sg, cmd->sg_nents, cmd->sg_dma_direction);
	g_next_sg = 1;

#ifdef USE_BOUNCE_BUF
	if((cmd->sg_nents > 1)) {
		int i; 
		unsigned long len = 0, total_len=0, addr;
		unsigned long buf =  KSEG1ADDR(g_bounce_buf);
		if(iswrite){ /* write using bounce buffer */
			for_each_sg(hwif->sg_table, sg, cmd->sg_nents, i) {
				len = sg_dma_len(sg);
				addr = (unsigned long) sg_virt(sg);
#ifndef USE_MBUS_MEMCPY
				memcpy((void *)buf, (void *)addr, len);
				mb();
#else
				mbus_memcpy(g_mbus_reg, tango2_dma_address(CPHYSADDR(buf)), sg_dma_address(sg), len);
#endif
				total_len += len;
				buf += len;
			}
			if (buf > (KSEG1ADDR(g_bounce_buf) + (1 << (DMA_BOUNCE_BUF_ORDER + PAGE_SHIFT)))){
				printk("IDE: Bounce buffer is too small to fit requested transfers\n");
				goto fallback_pio;
			}
#ifndef USE_MBUS_MEMCPY
			mb();
			dma_cache_wback_inv((unsigned long)g_bounce_buf, total_len);
#else
			dma_cache_inv((unsigned long)g_bounce_buf, total_len);
#endif
			if (em86xx_mbus_setup_dma(g_mbus_reg, tango2_dma_address(CPHYSADDR(g_bounce_buf)), total_len,
					   NULL,  drive, 1 )) {
				printk(KERN_ERR PFX "fail to setup dma, fallback to pio\n");
				dma_unmap_sg(&hwif->gendev, sg, cmd->sg_nents,
					     cmd->sg_dma_direction);
				em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);
				goto fallback_pio;
			}
		} else { /*read using bounce buffer */
			for(i = 0; i< cmd->sg_nents; i++){
				len = sg_dma_len(sg);
				total_len += len;
				sg = &hwif->sg_table[g_next_sg];
				g_next_sg++;
			}
			if (em86xx_mbus_setup_dma(g_mbus_reg, tango2_dma_address(CPHYSADDR(g_bounce_buf)), total_len,
					   NULL,  drive, 1 )) {
				printk(KERN_ERR PFX "fail to setup dma, fallback to pio\n");
				dma_unmap_sg(&hwif->gendev, sg, cmd->sg_nents,
					     cmd->sg_dma_direction);
				em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);
				goto fallback_pio;
			}
		}
	} else {

		/*
		 * setup mbus dma for this address.  we want an mbus interrupt
		 * only if this is not the last sg element,  so we can refeed
		 * mbus.
		 */
		if (em86xx_mbus_setup_dma(g_mbus_reg, sg_dma_address(sg), sg_dma_len(sg),  NULL , drive, 1 )) {
			printk(KERN_ERR PFX "fail to setup dma, fallback to pio\n");
			dma_unmap_sg(&hwif->gendev, sg, cmd->sg_nents,
				     cmd->sg_dma_direction);
			em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);
			goto fallback_pio;
		}
	}
#else
	/*
	 * setup mbus dma for this address.  we want an mbus interrupt
	 * only if this is not the last sg element,  so we can refeed
	 * mbus.
	 */
	if (em86xx_mbus_setup_dma(g_mbus_reg, sg_dma_address(sg),
				sg_dma_len(sg),
				(cmd->sg_nents == 1) ? NULL : tango2_mbus_intr,
				drive,
				(cmd->sg_nents == 1) ? 1 : 0)) {
		printk(KERN_ERR PFX "fail to setup dma, fallback to pio\n");
		dma_unmap_sg(&hwif->gendev, sg, cmd->sg_nents,
				cmd->sg_dma_direction);
		em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);
		goto fallback_pio;
	}
#endif

	drive->waiting_for_dma = 1;
	return 0;

fallback_pio:
	ide_map_sg(drive, cmd);
	return 1;
}

static int tango2_bmide_dma_end(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	u8 dma_stat;
	int mbus_stat;
        struct scatterlist *sg = hwif->sg_table;
	struct request *rq;

	rq = hwif->rq;

	dma_stat = hwif->dma_ops->dma_sff_read_status(hwif);
	/*
	 * make sure DMA is not in progress
	 */
	if ((dma_stat & 0x7) == 1) {
		printk(KERN_ERR PFX "huh ? dma_end called while dma still "
		       "in progress...dma_stat=0x%x\n", dma_stat);
	}

	/* clear the INTR & ERROR bits */
	tango2_bmide_dma_sff_write_status(hwif, dma_stat | 6);

	/*
	 * confirm  whether MBUS transfer  is done  due to  the memory
	 * arbitration, IDE  device thinks  the DMA transfer  is done,
	 * but the data might be held in MBUS FIFO
	 */
	mbus_stat = em86xx_mbus_wait(g_mbus_reg, SBOX_IDEDVD);

	/* release mbus */
	em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);

	/* stop bus mastering */
	WR_HOST_REG32(IDECTRL_bmic, 0x4);

	dma_unmap_sg(&hwif->gendev, sg, hwif->cmd.sg_nents,
				hwif->cmd.sg_dma_direction);

	drive->waiting_for_dma = 0;

#ifdef USE_BOUNCE_BUF
	if(!mbus_stat && ((dma_stat & 0x1) == 0) && 
		(hwif->sg_dma_direction == DMA_FROM_DEVICE) && 
		(hwif->sg_nents > 1)) {
		int i;
		unsigned long len = 0, addr;
		unsigned long buf =  KSEG1ADDR(g_bounce_buf);

		for_each_sg(hwif->sg_table, sg, hwif->sg_nents, i) {
			addr = (unsigned long) sg_virt(sg);
			len = sg_dma_len(sg);

#ifndef USE_MBUS_MEMCPY
			memcpy((void *)addr, (void *)buf, len);
			mb();
			dma_cache_wback_inv(addr, len);
#else
			dma_cache_inv(addr, len);
			mbus_memcpy(g_mbus_reg, sg_dma_address(sg), 
					tango2_dma_address(CPHYSADDR(buf)), len);
#endif
			buf += len;
		}
	}
#endif

	/* fake  dma  error  in  case  of mbus  timeout,  else  return
	 * dma_status error bit */
	return (mbus_stat == 0) ? (dma_stat & 1) : 0x1;
}

static int tango2_bmide_dma_test_irq(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	u8 dma_stat;

	dma_stat = hwif->dma_ops->dma_sff_read_status(hwif);

	if ((dma_stat & 0x4))
		return 1;
	return 0;
}

#endif

#ifndef CONFIG_RUNTIME_CLK_CALC
/*
 * timing values for each ide mode
 */
static const unsigned int s_pio_tim[] = {
	0xff230ee6, 0xd41b0fa4, 0xb4150f63, 0xa7110f62, 0x9a0f0552
};

static const unsigned int s_dma_tim[] = {
	0xf33333b4, 0xa3130b73, 0x9c100552
};
static const unsigned int s_udma_tim1[] = {
	0x35440b08, 0x35440a06, 0x35440804
};
static const unsigned int s_udma_tim2[] = {
	0x00000208, 0x00000206, 0x00000204
};

static const unsigned int s_udma_tim1_alt[] = {
	0x44442418, 0x44441c10, 0x4444160b, 0x44441608,
	0x44441605, 0x44441303, 0x44441302, 0x44441302
};
static const unsigned int s_udma_tim2_alt[] = {
	0x0000010f, 0x0000010a, 0x00000106, 0x00000104,
	0x00000101, 0x00000202, 0x00000202, 0x00000202
};
#endif /* !CONFIG_RUNTIME_CLK_CALC */

static u8 drive_speed[2] = { 0xff, 0xff };

static void tango2_bmide_set_pio_mode(ide_hwif_t *hwif, ide_drive_t *drive)
{
	int didx;
	unsigned int ctrlreg, newflag;
    	unsigned long flags;
	u8 xferspeed, pio;

#ifdef CONFIG_RUNTIME_CLK_CALC
	extern unsigned long tangox_get_sysclock(void);
	unsigned int ide_clock = TIMING_MARGIN(tangox_get_sysclock() / 1000000); /* in MHz */
#endif
	didx = drive->dn & 1;
	newflag = 0;

	xferspeed = drive->pio_mode;
	spin_lock_irqsave(&bmide_tune_chipset_spin_lock, flags);
	if (xferspeed >= XFER_PIO_0 && xferspeed <= XFER_PIO_4) {
		printk("%s: set to PIO mode %d\n", drive->name, drive->pio_mode - XFER_PIO_0);

		/* fast timing for PIO, enable IORDY sampling */
		newflag = 0x03;

#ifdef CONFIG_RUNTIME_CLK_CALC
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim), 
			    CalcRegValTiming_PIO_DMA(&(j2_pio_timings[drive->pio_mode - XFER_PIO_0]), ide_clock));
#else
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim), 
			    s_pio_tim[drive->pio_mode - XFER_PIO_0]);
#endif
	}

	ctrlreg = gbus_read_reg32(REG_BASE_host_interface + IDECTRL_pri_idectl);
	ctrlreg &= ~(didx == 0 ? 0xf : 0xf0);
	ctrlreg |= newflag << (didx * 4);
	gbus_write_reg32(REG_BASE_host_interface + IDECTRL_pri_idectl, ctrlreg);

	spin_unlock_irqrestore(&bmide_tune_chipset_spin_lock, flags);
	pio = xferspeed - XFER_PIO_0;
	drive_speed[didx] = xferspeed;
}

static void tango2_bmide_set_xfer_mode(ide_hwif_t *hwif, ide_drive_t *drive)
{
	int didx;
	unsigned int ctrlreg, newflag;
    unsigned long flags;
	u8 mode, pio;
	const u8 xferspeed	= drive->dma_mode;

#ifdef CONFIG_RUNTIME_CLK_CALC
	extern unsigned long tangox_get_sysclock(void);
	unsigned int ide_clock = TIMING_MARGIN(tangox_get_sysclock() / 1000000); /* in MHz */
#endif
	didx = drive->dn & 1;
	newflag = 0;

	spin_lock_irqsave(&bmide_tune_chipset_spin_lock, flags);
	if (xferspeed >= XFER_PIO_0 && xferspeed <= XFER_PIO_4) {
		/*
		 * setup timing for PIO mode
		 */
		mode = xferspeed - XFER_PIO_0;
		printk("%s: set to PIO mode %d\n", drive->name, mode);

		/* fast timing for PIO, enable IORDY sampling */
		newflag = 0x03;

#ifdef CONFIG_RUNTIME_CLK_CALC
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim), 
			    CalcRegValTiming_PIO_DMA(&(j2_pio_timings[mode]), ide_clock));
#else
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim), 
			    s_pio_tim[mode]);
#endif

	}
#ifdef CONFIG_BLK_DEV_BMIDE_TANGO2_DMA
	else if (xferspeed >= XFER_MW_DMA_0 && xferspeed <= XFER_MW_DMA_2) {
		/*
		 * setup timing for Multi-word DMA
		 */
		mode = xferspeed - XFER_MW_DMA_0;
		printk("%s: set to Multi-word DMA mode %d\n", drive->name,
		       mode);

		/* fast timing for PIO, prefetch enable */
		newflag = 0x05;

#ifdef CONFIG_RUNTIME_CLK_CALC
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim), 
			    CalcRegValTiming_PIO_DMA(&(j2_dma_timings[mode]), ide_clock));
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0udmatim1 : IDECTRL_pri_drv1udmatim1), 
			    CalcRegValTiming1_UDMA(&(j2_udma_timings[mode]), ide_clock));
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0udmatim2 : IDECTRL_pri_drv1udmatim2), 
			    CalcRegValTiming2_UDMA(&(j2_udma_timings[mode]), ide_clock));
#else
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim), 
			    s_dma_tim[mode]);
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0udmatim1 : IDECTRL_pri_drv1udmatim1), 
			    s_udma_tim1[mode]);
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0udmatim2 : IDECTRL_pri_drv1udmatim2), 
			    s_udma_tim2[mode]);
#endif

	} 
#endif
	else if (xferspeed >= XFER_UDMA_0 && xferspeed <= XFER_UDMA_7) {
		unsigned int val;

		/*
		 * setup timing for Ultra DMA
		 */
		mode = xferspeed - XFER_UDMA_0;
		printk("%s: set to Ultra DMA mode %d\n", drive->name, mode);

		newflag = 0x00;

		/* enable Ultra DMA */
		val = gbus_read_reg32(REG_BASE_host_interface + IDECTRL_udmactl);
		val |= (didx == 0) ? 0x01 : 0x02;
		gbus_write_reg32(REG_BASE_host_interface + IDECTRL_udmactl, val);

#ifdef CONFIG_RUNTIME_CLK_CALC
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0udmatim1 : IDECTRL_pri_drv1udmatim1), 
			    CalcRegValTiming1_UDMA(&(j2_udma_timings[mode]), ide_clock));
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0udmatim2 : IDECTRL_pri_drv1udmatim2), 
			    CalcRegValTiming2_UDMA(&(j2_udma_timings[mode]), ide_clock));
#else
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0udmatim1 : IDECTRL_pri_drv1udmatim1), 
			    s_udma_tim1_alt[mode]);
		gbus_write_reg32(REG_BASE_host_interface +
			    ((didx == 0) ? IDECTRL_pri_drv0udmatim2 : IDECTRL_pri_drv1udmatim2), 
			    s_udma_tim2_alt[mode]);
#endif
 	}
	else {
		printk("%s: unknown speed to be set %d\n", drive->name, xferspeed);
	}

	ctrlreg = gbus_read_reg32(REG_BASE_host_interface + IDECTRL_pri_idectl);
	ctrlreg &= ~(didx == 0 ? 0xf : 0xf0);
	ctrlreg |= newflag << (didx * 4);
	gbus_write_reg32(REG_BASE_host_interface + IDECTRL_pri_idectl, ctrlreg);

	spin_unlock_irqrestore(&bmide_tune_chipset_spin_lock, flags);

#ifdef CONFIG_BLK_DEV_BMIDE_TANGO2_DMA
	if (xferspeed >= XFER_SW_DMA_0)
		pio = dma_2_pio(xferspeed);
	else
#endif
		pio = xferspeed - XFER_PIO_0;

	drive_speed[didx] = xferspeed;
}

static void tango2_hw_setup(struct ide_hw *hw, unsigned long data_port,
		     unsigned long ctrl_port)
{
	int i;
	unsigned long reg;

	memset(hw, 0, sizeof (*hw));
	reg = data_port;

	for (i = 0; i <= 7; i++){
		hw->io_ports_array[i] = reg;
		reg += 4;
	}

	hw->io_ports.ctl_addr = ctrl_port;
}

/*
 * init function, ide controller is registred here
 */
extern int tangox_bmide_enabled(void);

static void tango2_bmide_clear_irq(ide_drive_t *drive)
{
	return;
}

static u8 tango2_bmide_udma_filter(ide_drive_t *drive)
{
	if ((gbus_read_reg32(REG_BASE_host_interface + IDECTRL_idestatus) 
				& 0x02) == 0x02){
		return ATA_UDMA2;
	} else{
		return ATA_UDMA5;
	}
}

static u8 tango2_bmide_cable_detect(ide_hwif_t *hwif)
{
	if ((gbus_read_reg32(REG_BASE_host_interface + IDECTRL_idestatus) 
				& 0x02) == 0x02){
		return ATA_CBL_PATA40;
	} else{
		return ATA_CBL_PATA80;
	}
}

static void tango2_bmide_init_hwif(ide_hwif_t *hwif)
{
	return;
}

static int tango2_bmide_init_dma(ide_hwif_t *hwif, const struct ide_port_info *d)
{
	return 0;
}

void tango2_bmide_dev_select(ide_drive_t *drive)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;

	u8 select = drive->select | ATA_DEVICE_OBS;
	tango2_bmide_outb(select, io_ports->device_addr);
}


static void tango2_bmide_tf_load(ide_drive_t *drive, struct ide_taskfile *tf, u8 valid)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;

	if (valid & IDE_VALID_FEATURE)
		tango2_bmide_outb(tf->feature, io_ports->feature_addr);
	if (valid & IDE_VALID_NSECT)
		tango2_bmide_outb(tf->nsect, io_ports->nsect_addr);
	if (valid & IDE_VALID_LBAL)
		tango2_bmide_outb(tf->lbal, io_ports->lbal_addr);
	if (valid & IDE_VALID_LBAM)
		tango2_bmide_outb(tf->lbam, io_ports->lbam_addr);
	if (valid & IDE_VALID_LBAH)
		tango2_bmide_outb(tf->lbah, io_ports->lbah_addr);
	if (valid & IDE_VALID_DEVICE)
		tango2_bmide_outb(tf->device, io_ports->device_addr);
}

static void tango2_bmide_tf_read(ide_drive_t *drive, struct ide_taskfile *tf, u8 valid)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;

	if (valid & IDE_VALID_ERROR)
		tf->error  = tango2_bmide_inb(io_ports->feature_addr);
	if (valid & IDE_VALID_NSECT)
		tf->nsect  = tango2_bmide_inb(io_ports->nsect_addr);
	if (valid & IDE_VALID_LBAL)
		tf->lbal   = tango2_bmide_inb(io_ports->lbal_addr);
	if (valid & IDE_VALID_LBAM)
		tf->lbam   = tango2_bmide_inb(io_ports->lbam_addr);
	if (valid & IDE_VALID_LBAH)
		tf->lbah   = tango2_bmide_inb(io_ports->lbah_addr);
	if (valid & IDE_VALID_DEVICE)
		tf->device = tango2_bmide_inb(io_ports->device_addr);
}

u8 tango2_bmide_read_status(ide_hwif_t *hwif)
{
	u8 status;
	status = tango2_bmide_inb(hwif->io_ports.status_addr);
	return status;
}

u8 tango2_bmide_read_altstatus(ide_hwif_t *hwif)
{
	u8 alt_status;
	alt_status = tango2_bmide_inb(hwif->io_ports.ctl_addr);
	return alt_status;
}

void tango2_bmide_write_devctl(ide_hwif_t *hwif, u8 ctl)
{
	tango2_bmide_outb(ctl, hwif->io_ports.ctl_addr);
}

void tango2_bmide_set_irq(ide_hwif_t *hwif, int on)
{
	u8 ctl = ATA_DEVCTL_OBS;

	if (on == 4) { /* hack for SRST */
		ctl |= 4;
		on &= ~4;
	}

	ctl |= on ? 0 : 2;

	tango2_bmide_outb(ctl, hwif->io_ports.ctl_addr);
}

static void tango2_bmide_input_data(ide_drive_t *drive, struct ide_cmd *cmd,
				void *buf, unsigned int len)
{
	unsigned long port = drive->hwif->io_ports.data_addr;
	unsigned short *ptr = buf;
	unsigned int count = (len + 1) / 2;

	while (count--)
		*ptr++ = cpu_to_le16(tango2_bmide_inw(port));
	__ide_flush_dcache_range((unsigned long)buf, roundup(len, 2));
}

static void tango2_bmide_output_data(ide_drive_t *drive, struct ide_cmd *cmd,
				void *buf, unsigned int len)
{
	unsigned long port = drive->hwif->io_ports.data_addr;
	unsigned short *ptr = buf;
	unsigned int count = (len + 1) / 2;

	while (count--) {
		tango2_bmide_outw(le16_to_cpu(*ptr), port);
		ptr++;
	}
	__ide_flush_dcache_range((unsigned long)buf, roundup(len, 2));
}
void tango2_bmide_exec_command(ide_hwif_t *hwif, u8 cmd)
{
	tango2_bmide_outb(cmd, hwif->io_ports.command_addr);
}

static const struct ide_tp_ops tango2_bmide_tp_ops = {
	.exec_command		= tango2_bmide_exec_command,
	.read_status		= tango2_bmide_read_status,
	.read_altstatus		= tango2_bmide_read_altstatus,
	.write_devctl		= tango2_bmide_write_devctl,

	.dev_select			= tango2_bmide_dev_select,
	.tf_load			= tango2_bmide_tf_load,
	.tf_read			= tango2_bmide_tf_read,

	.input_data			= tango2_bmide_input_data,
	.output_data		= tango2_bmide_output_data,
};

static const struct ide_port_ops tango2_bmide_port_ops = {
	.set_pio_mode		= tango2_bmide_set_pio_mode,
	.set_dma_mode		= tango2_bmide_set_xfer_mode,
	.clear_irq			= tango2_bmide_clear_irq,
	.udma_filter		= tango2_bmide_udma_filter,
	.cable_detect		= tango2_bmide_cable_detect,
};

static const struct ide_dma_ops tango2_bmide_dma_ops = {
	.dma_lost_irq		= ide_dma_lost_irq,
	.dma_host_set		= tango2_bmide_dma_host_set,
	.dma_setup			= tango2_bmide_dma_setup,
	.dma_start			= tango2_bmide_dma_start,
	.dma_end			= tango2_bmide_dma_end,
	.dma_test_irq		= tango2_bmide_dma_test_irq,
	.dma_sff_read_status= tango2_bmide_dma_sff_read_status,
	};

static const struct ide_port_info tango2_bmide_port_info = {
	.init_hwif		= tango2_bmide_init_hwif,
	.init_dma		= tango2_bmide_init_dma,
	.port_ops		= &tango2_bmide_port_ops,
	.dma_ops		= &tango2_bmide_dma_ops,
	.tp_ops			= &tango2_bmide_tp_ops,
	.host_flags		= IDE_HFLAG_MMIO,
	.pio_mask		= ATA_PIO4,
	.mwdma_mask		= ATA_MWDMA2,
	.udma_mask		= ATA_UDMA5,
	.chipset		= ide_generic,
};

static int tango2_bmide_probe(struct platform_device *pdev)
{
	struct ide_hw hw;
	struct ide_hw *hws[] = { &hw, NULL, NULL, NULL };
	struct ide_host *host;
	int  ret;
	unsigned int mapbase, mapctl;

	if (!tangox_bmide_enabled()) {
		printk(KERN_NOTICE PFX "bmide support is disabled\n");
		return -1;
	}

	/* Enable bus master IDE interface */
	WR_HOST_REG32(IDECTRL_pri_idectl, 0x8400);

	/* no skew : normal IORDY, normal data */
	WR_HOST_REG32(IDECTRL_idesrc, 0);

	/* Enable non-PRD mode */
	WR_HOST_REG32(IDECTRL_bmic, 0x4);

	/*
	 * set register offset for our controller
	 */
	mapbase = KSEG1ADDR(REG_BASE_host_interface_BMIDE);
	mapctl	= KSEG1ADDR(REG_BASE_host_interface + IDECTRL_pri_ctrlblock);	
	tango2_hw_setup(&hw, REG_BASE_host_interface_BMIDE,
			REG_BASE_host_interface + IDECTRL_pri_ctrlblock);
	
	/*
	 * Tango2   has   two   interrupts   for   IDE   controller   :
	 * IRQ_IDECTRL_IDE,  IRQ_IDECTRL_IDEDMA
	 *
	 * IRQ_IDECTRL_IDEDMA is to be used for DMA transfer but it is
	 * almost same with IRQ_IDECTRL_IDE interrupt.  Tango triggers
	 * IDEDMA interrupt  after DMA FIFO  is flushed, and  it stays
	 * high   until  BMIS   register  bit   2  is   cleared.   Use
	 * IRQ_IDECTRL_IDE interrupt for both of PIO and DMA.
	 */
	hw.irq = IRQ_IDECTRL_IDE;

	/*
	 * register our controller
	 */
	hw.dev = &pdev->dev;

	pr_info("TANGO2 Bus Mastering IDE interface (base 0x%x,ctl 0x%x,irq %d)\n", 
									mapbase, mapctl, hw.irq);
	host = ide_host_alloc(&tango2_bmide_port_info, hws, 1);
	if (!host)
		return -ENOMEM;
	/* use extra_base for base address of the all registers */
	host->ports[0]->extra_base = REG_BASE_host_interface_BMIDE;
	ret = ide_host_register(host, &tango2_bmide_port_info, hws);
	if (ret) {
		ide_host_free(host);
		return ret;
	}
	platform_set_drvdata(pdev, host);

#ifdef USE_BOUNCE_BUF
	if ((g_bounce_buf = (unsigned char *)__get_free_pages(GFP_KERNEL, DMA_BOUNCE_BUF_ORDER)) == NULL)
		printk("IDE: Can not allocate buffer for IDE transfer\n");

	printk("Bounce buffer starts at 0x%08lx\n", (unsigned long)g_bounce_buf);
#endif	
	
	return 0;
}

static int __exit tango2_bmide_remove(struct platform_device *pdev)
{
	struct ide_host *host = platform_get_drvdata(pdev);

	ide_host_remove(host);

	return 0;
}

static int tango2_bmide_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver tango2_bmide_driver = {
	.driver = {
		.name = "tango2_bmide",
		.owner = THIS_MODULE,
	},
	.probe  = tango2_bmide_probe,
	.remove = tango2_bmide_remove,
	.resume = tango2_bmide_resume,
};

static void tango2_bmide_release_dev(struct device * dev)
{
	dev->parent = NULL;
}

static struct platform_device tango2_bmide_device = {
	.name	= "tango2_bmide",
	.id		= -1,
	.dev	= {
		.release = tango2_bmide_release_dev,
	},
	.num_resources  = 0,
	.resource       = 0,
};

int __init tango2_bmide_init(void)
{
	int status = 0;

	platform_device_register(&tango2_bmide_device);
	platform_driver_register(&tango2_bmide_driver);
	return status;
}

static void __exit tango2_bmide_exit(void)
{
	platform_driver_unregister(&tango2_bmide_driver);
	platform_device_unregister(&tango2_bmide_device);
#ifdef USE_BOUNCE_BUF
	free_pages((unsigned long)g_bounce_buf, DMA_BOUNCE_BUF_ORDER);
#endif
	return;
}

module_init(tango2_bmide_init);
module_exit(tango2_bmide_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SMP86xx BM IDE driver");

#ifdef CONFIG_RUNTIME_CLK_CALC
/* The following is for runtime calculation of register value based on the mode and given
 * system frequency by Ivan K. */
static unsigned int CalcRegValTiming_PIO_DMA(j2_tim_pio_dma_t *t, unsigned long ide_clock)
{
	unsigned int dwRet;
	unsigned int cycle_clks;	// ide clocks / cycle
	unsigned int active_clks;	// ide clocks for active strobe
	unsigned int recov_clks;	// ide clocks for recovery
	unsigned int setup_clks;	// ide clocks for setup
	unsigned int hold_clks;	// ide clocks for hold

	// pio_timings in ns
	cycle_clks	= (ide_clock * t->cycle_time)/1000;
	active_clks	= (ide_clock * t->active_time)/1000;
	recov_clks	= (ide_clock * t->recovery_time)/1000;
	setup_clks	= (ide_clock * t->setup_time)/1000;
	hold_clks	= (ide_clock * t->hold_time)/1000;

	if(cycle_clks>0x7F) {
		cycle_clks = 0x7F;
	}
	
	if(active_clks>0x3F) {
		active_clks = 0x3F;
	}

	if(recov_clks>0x3F) {
		recov_clks = 0x3F;
	}

	if(setup_clks>0xF) {
		setup_clks = 0xF;
	}
	
	if(hold_clks>0x7) {
		setup_clks = 0x7;
	}
	
	dwRet	= (1<<31) | (cycle_clks<<24) | (active_clks<<16) | (recov_clks<<8) | (setup_clks<<4) | (hold_clks);

	return dwRet;
}

static unsigned int CalcRegValTiming1_UDMA(j2_tim_udma_t *t, unsigned long ide_clock)
{
	unsigned int dwRet;
	unsigned int t_zah,t_mli,t_ack,t_env,t_rp,t_cyc;

	// pio_timings in ns
	t_zah	= (ide_clock * t->t_zah)/1000;
	t_mli	= (ide_clock * t->t_mli)/1000;
	t_ack	= (ide_clock * t->t_ack)/1000;
	t_env	= (ide_clock * t->t_env)/1000;
	t_rp	= (ide_clock * t->t_rp) /1000;
	t_cyc	= (ide_clock * t->t_cyc)/1000;

	if ( t_zah > 0x0F ) {
		t_zah = 0x0F;
	}
	if ( t_mli > 0x0F ) {
		t_mli = 0x0F;
	}
	if ( t_ack > 0x0F ) {
		t_ack = 0x0F;
	}
	if ( t_env > 0x0F) {
		t_env = 0x0F;
	}
	if ( t_rp > 0x3F ) {
		t_rp = 0x3F;
	}
	if ( t_cyc > 0x3F ) {
		t_cyc = 0x3F;
	}
	
	dwRet	= (t_zah<<28) | (t_mli<<24) | (t_ack<<20) | (t_env<<16) | (t_rp<<8) | (t_cyc);

	return dwRet;
}

static unsigned int CalcRegValTiming2_UDMA(j2_tim_udma_t *t, unsigned long ide_clock)
{
	unsigned int dwRet;
	unsigned int t_cvh,t_cvs;
	
	// pio_timings in ns
	t_cvh	= (ide_clock * t->t_cvh)/1000;
	t_cvs	= (ide_clock * t->t_cvs)/1000;

	if ( t_cvh > 0x03 ) {
		t_cvh = 0x03;
	}
	if ( t_cvs > 0x3F ) {
		t_cvs = 0x3F;
	}

	dwRet	= (t_cvh<<8) | (t_cvs);

	return dwRet;
}
#endif /* CONFIG_RUNTIME_CLK_CALC */

/* to calcuate the speed needed,  this is used in fctrl module*/
void tango2_bmide_calc_speed(void)
{
	int didx;
	u8 xferspeed;
	u8 mode;
	unsigned long flags;
#ifdef CONFIG_RUNTIME_CLK_CALC
	extern unsigned long tangox_get_sysclock(void);
	unsigned int ide_clock = TIMING_MARGIN(tangox_get_sysclock() / 1000000); /* in MHz */
#endif
	
	if (!tangox_bmide_enabled())
		return;
	
	spin_lock_irqsave(&bmide_tune_chipset_spin_lock, flags);
	
	for (didx = 0; didx < 2; didx++) {
		if ((xferspeed = drive_speed[didx]) == 0xff)
			continue;
		
		if (xferspeed >= XFER_PIO_0 && xferspeed <= XFER_PIO_4) {
			/*
			 * setup timing for PIO mode
			 */
			mode = xferspeed - XFER_PIO_0;
#ifdef CONFIG_RUNTIME_CLK_CALC
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim),
					CalcRegValTiming_PIO_DMA(&(j2_pio_timings[mode]), ide_clock));
#else
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim),
					s_pio_tim[mode]);
#endif
		}
#ifdef CONFIG_BLK_DEV_BMIDE_TANGO2_DMA
		else if (xferspeed >= XFER_MW_DMA_0 && xferspeed <= XFER_MW_DMA_2) {
			/*
			 * setup timing for Multi-word DMA
			 */
			mode = xferspeed - XFER_MW_DMA_0;
#ifdef CONFIG_RUNTIME_CLK_CALC
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim),
					CalcRegValTiming_PIO_DMA(&(j2_dma_timings[mode]), ide_clock));
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0udmatim1 : IDECTRL_pri_drv1udmatim1),
					
					CalcRegValTiming1_UDMA(&(j2_udma_timings[mode]), ide_clock));
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0udmatim2 : IDECTRL_pri_drv1udmatim2),
					CalcRegValTiming2_UDMA(&(j2_udma_timings[mode]), ide_clock));
#else
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0tim : IDECTRL_pri_drv1tim),
					s_dma_tim[mode]);
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0udmatim1 : IDECTRL_pri_drv1udmatim1),
					s_udma_tim1[mode]);
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0udmatim2 : IDECTRL_pri_drv1udmatim2),
					s_udma_tim2[mode]);
#endif
		}
#endif
		else if (xferspeed >= XFER_UDMA_0 && xferspeed <= XFER_UDMA_7) {
			/*
			 * setup timing for Ultra DMA
			 */
			mode = xferspeed - XFER_UDMA_0;
#ifdef CONFIG_RUNTIME_CLK_CALC
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0udmatim1 : IDECTRL_pri_drv1udmatim1),
					CalcRegValTiming1_UDMA(&(j2_udma_timings[mode]), ide_clock));
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0udmatim2 : IDECTRL_pri_drv1udmatim2),
					CalcRegValTiming2_UDMA(&(j2_udma_timings[mode]), ide_clock));
#else
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0udmatim1 : IDECTRL_pri_drv1udmatim1),
					s_udma_tim1_alt[mode]);
			gbus_write_reg32(REG_BASE_host_interface +
					((didx == 0) ? IDECTRL_pri_drv0udmatim2 : IDECTRL_pri_drv1udmatim2),
					s_udma_tim2_alt[mode]);
#endif
		}
	}

	spin_unlock_irqrestore(&bmide_tune_chipset_spin_lock, flags);
}
EXPORT_SYMBOL(tango2_bmide_calc_speed);

