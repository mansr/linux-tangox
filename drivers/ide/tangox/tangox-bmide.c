
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
  
#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ide.h>

#include "tangox-ide.h"

static DEFINE_SPINLOCK(bmide_tune_chipset_spin_lock);

#define PFX	"tangox_bmide: "

#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX_DMA
#define CONFIG_BLK_DEV_BMIDE_TANGOX_UDMA /* enable UDMA as well */
#endif

/* some drive cannot use mbus chaining, use bounce buffer instead */
//#define USE_BOUNCE_BUF
#ifdef USE_BOUNCE_BUF
// maximum = 128 sectors = 2^7 sectors = 2^7 * 2^9 bytes = 2^16 bytes
#define DMA_BOUNCE_BUF_ORDER		(19 - PAGE_SHIFT)	
static unsigned char *g_bounce_buf = NULL;
//#define USE_MBUS_MEMCPY
#endif


/*
 * helper to access host interface
 */
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
static void tangox_bmide_outb(u8 data, unsigned long port)
{
	gbus_write_reg16(port, data);
}

static void tangox_bmide_outw(u16 data, unsigned long port)
{
	gbus_write_reg16(port, data);
}

static void tangox_bmide_outbsync(ide_drive_t *drive, u8 data,
				unsigned long port)
{
	gbus_write_reg16(port, data);
}

static u8 tangox_bmide_inb(unsigned long port)
{
	u8 val;

	val = ((u8)(gbus_read_reg16(port) & 0xff));
	return val;
}

static u16 tangox_bmide_inw(unsigned long port)
{
	u16 val;

	val = ((u16)(gbus_read_reg16(port) & 0xffff));
	return val;
}

static void tangox_bmide_insw(unsigned long port, void *addr, u32 count)
{
	while (count--) {
		*(u16 *)addr = (u16)(gbus_read_reg16(port) & 0xffff);
		addr += 2;
	}
}

static void tangox_bmide_outsw(unsigned long port, void *addr, u32 count)
{
	while (count--) {
		gbus_write_reg16(port, *(u16 *)addr);
		addr += 2;
	}
}

#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX_DMA
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
void tangox_bmide_dma_host_on(ide_drive_t *drive)
{
	if (drive->using_dma) {
		ide_hwif_t *hwif = HWIF(drive);
		u8 unit = (drive->select.b.unit & 0x01);
		u8 dma_stat = hwif->INB(hwif->dma_status);

		hwif->OUTB((dma_stat | (1 << (5 + unit))), hwif->dma_status);
	}
}

void tangox_bmide_dma_host_off(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	u8 unit = (drive->select.b.unit & 0x01);
	u8 dma_stat = hwif->INB(hwif->dma_status);

	hwif->OUTB((dma_stat & ~(1 << (5 + unit))), hwif->dma_status);
}

int tangox_bmide_dma_check(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	u8 pio, speed;

	if(ide_tune_dma(drive))
		return 0;

	/* fallback to pio mode */
	pio = ide_get_best_pio_mode(drive, 255, 5, NULL);
	speed = dma_2_pio(XFER_PIO_0 + pio) + XFER_PIO_0;
	hwif->speedproc(drive, speed);
	hwif->dma_off_quietly(drive);
	return 0; 
}

int tangox_bmide_dma_on(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);

	if (hwif->dma_host_on)
		hwif->dma_host_on(drive);

	if (drive->using_dma)
		return 0;

	printk("%s: DMA enabled for %s%s\n", hwif->name,
	       drive->media == ide_disk ? "ATA DISK " :
	       (drive->media == ide_cdrom ? "ATAPI CDROM " : ""),
	       drive->name);
	drive->using_dma = 1;

	return 0;
}

void tangox_bmide_dma_off_quietly(ide_drive_t *drive)
{
	drive->using_dma = 0;
}

/*
 * dma operations
 */
static unsigned long g_mbus_reg = 0;
static unsigned int g_next_sg = 0;

static ide_startstop_t tangox_dma_intr(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	u8 dma_stat, stat;
	struct request *rq;

	dma_stat = hwif->ide_dma_end(drive);
	stat = HWIF(drive)->INB(IDE_STATUS_REG);

	if (OK_STAT(stat, DRIVE_READY, drive->bad_wstat | DRQ_STAT)) {
		if (!(dma_stat & 1)) {
			rq = HWGROUP(drive)->rq;
			ide_end_request(drive, 1, rq->nr_sectors);
			return ide_stopped;
		}
		printk(KERN_ERR PFX "%s: bad DMA status (dma_stat=%x)\n",
		       drive->name, dma_stat);
	}

	return ide_error(drive, "dma_intr", 0);
}

static void tangox_mbus_intr(int irq, void *arg)
{
	ide_drive_t *drive = (ide_drive_t *)arg;
	ide_hwif_t *hwif = HWIF(drive);
        struct scatterlist *sg;

	if (drive->using_dma == 0) {
		printk(KERN_ERR PFX "bug: tangox_mbus_intr called while "
		       "using_dma == 0\n");
		return;
	}

	if (g_next_sg > hwif->sg_nents)
		return; /* should be the last interrupt */
	else if (g_next_sg == hwif->sg_nents) { /* no more sg */
		g_next_sg++;
		em86xx_mbus_setup_dma_void(g_mbus_reg); 
		return;
	}

	em86xx_mbus_wait(g_mbus_reg, SBOX_IDEDVD);

	/*
	 * setup a new mbus transfer
	 */
	sg = &hwif->sg_table[g_next_sg];
	g_next_sg++;

	if (em86xx_mbus_setup_dma(g_mbus_reg, sg_dma_address(sg), sg_dma_len(sg),
				  tangox_mbus_intr, drive, 0)) {
		printk(KERN_ERR PFX "fail to resetup dma, wait "
		       "for timeout...\n");
	}
}

static void tangox_dma_exec_cmd(ide_drive_t *drive, u8 command)
{
	/* issue  cmd to  drive,  and register  interrupt handler  for
	 * command completion */
	ide_execute_command(drive, command, &tangox_dma_intr, (2 * WAIT_CMD),
			    NULL);
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

static void tangox_dma_start(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	struct request *rq;
	unsigned long val, len;

	rq = HWGROUP(drive)->rq;
	len = get_request_size(rq);

	/* setup IDE DMA transfer len */
	gbus_write_reg32(REG_BASE_host_interface + IDECTRL_ide_dmalen, len);

	/* enable bus mastering */
	val = 0x05;
	if (hwif->sg_dma_direction == DMA_FROM_DEVICE)
		val |= 0x8;

	gbus_write_reg32(REG_BASE_host_interface + IDECTRL_bmic, val);
}

static int tangox_dma_setup(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	struct request *rq;
	int iswrite, phys_mapped;
	struct scatterlist *sg = hwif->sg_table;

        rq = HWGROUP(drive)->rq;

	iswrite = (rq_data_dir(rq) == WRITE);
	hwif->sg_dma_direction = iswrite ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	phys_mapped = (bio_flagged(rq->bio, BIO_PHYSICAL) ? 1 : 0);

	if (!phys_mapped) {
		/*
		 * ide_map_sg will merge contiguous memory zone for us
		 */
		ide_map_sg(drive, rq);
	}

	/* try to setup dma channel */
	if (em86xx_mbus_alloc_dma(SBOX_IDEDVD, iswrite ? 0 : 1, &g_mbus_reg,
				  NULL)) {
		printk(KERN_ERR PFX "fail to alloc dma, fallback to pio\n");
		goto fallback_pio;
	}

	if (!phys_mapped) {
		/*
		 * map and transfer first segment
		 */
		dma_map_sg(&hwif->gendev, sg, hwif->sg_nents, hwif->sg_dma_direction);
		g_next_sg = 1;

#ifdef USE_BOUNCE_BUF
		if((hwif->sg_nents > 1)) {
			int i; 
			unsigned long len = 0, total_len=0, addr;
			unsigned long buf = (unsigned long) g_bounce_buf;
			if(iswrite){ /* write using bounce buffer */
				//memset(g_bounce_buf, 0, 1 << (DMA_BOUNCE_BUF_ORDER + PAGE_SHIFT));
				for(i = 0; i< hwif->sg_nents; i++){
					len = sg_dma_len(sg);
					addr =	KSEG1ADDR(tangox_inv_dma_address(sg_dma_address(sg)));
#ifndef USE_MBUS_MEMCPY
					memcpy((void *)buf, (void *)addr, len);
#else
					dma_cache_wback((unsigned long)addr, len);
					mbus_memcpy(g_mbus_reg, tangox_dma_address(CPHYSADDR(buf)), sg_dma_address(sg), len);
#endif
					total_len += len;
					buf += len;
					sg = &hwif->sg_table[g_next_sg];
					g_next_sg++;
				}
				if (buf > ((unsigned long) g_bounce_buf + (1 << (DMA_BOUNCE_BUF_ORDER + PAGE_SHIFT)))){
					printk("IDE: Bounce buffer is too small to fit requested transfers\n");
					goto fallback_pio;
				}

#ifndef USE_MBUS_MEMCPY
				mb();
				dma_cache_wback_inv((unsigned long)g_bounce_buf, total_len);
#else
				dma_cache_inv((unsigned long)g_bounce_buf, total_len);
#endif

				if (em86xx_mbus_setup_dma(g_mbus_reg, CPHYSADDR(g_bounce_buf), total_len,
						   NULL,  drive, 1 )) {
					printk(KERN_ERR PFX "fail to setup dma, fallback to pio\n");
					dma_unmap_sg(&hwif->gendev, sg, hwif->sg_nents,
						     hwif->sg_dma_direction);
					em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);
					goto fallback_pio;
				}
			} else { /*read using bounce buffer */
				for(i = 0; i< hwif->sg_nents; i++){
					len = sg_dma_len(sg);
					total_len += len;
					sg = &hwif->sg_table[g_next_sg];
					g_next_sg++;
				}
				if (em86xx_mbus_setup_dma(g_mbus_reg, CPHYSADDR(g_bounce_buf), total_len,
						   NULL,  drive, 1 )) {
					printk(KERN_ERR PFX "fail to setup dma, fallback to pio\n");
					dma_unmap_sg(&hwif->gendev, sg, hwif->sg_nents,
						     hwif->sg_dma_direction);
					em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);
					goto fallback_pio;
				}
			}
		}
		else {

			/*
			 * setup mbus dma for this address.  we want an mbus interrupt
			 * only if this is not the last sg element,  so we can refeed
			 * mbus.
			 */
			if (em86xx_mbus_setup_dma(g_mbus_reg, sg_dma_address(sg),
					  sg_dma_len(sg),
					  (hwif->sg_nents == 1) ? NULL : tangox_mbus_intr, 
					  drive, 
					  (hwif->sg_nents == 1) ? 1 : 0)) {
				printk(KERN_ERR PFX "fail to setup dma, fallback to pio\n");
				dma_unmap_sg(&hwif->gendev, sg, hwif->sg_nents,
					     hwif->sg_dma_direction);
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
					(hwif->sg_nents == 1) ? NULL : tangox_mbus_intr,
					drive,
					(hwif->sg_nents == 1) ? 1 : 0)) {
			printk(KERN_ERR PFX "fail to setup dma, fallback to pio\n");
			dma_unmap_sg(&hwif->gendev, sg, hwif->sg_nents,
					hwif->sg_dma_direction);
			em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);
			goto fallback_pio;
		}
#endif
	} else {
		/*
		 * setup mbus dma for this address.  we want an mbus interrupt
		 * only if this  is not the last sg element,  so we can refeed
		 * mbus.
		 */
		/* physical addr was carried by bi_private*/
		if (em86xx_mbus_setup_dma(g_mbus_reg, (unsigned int)rq->bio->bi_private,
					rq->bio->bi_size, NULL, drive, 1)) {
			printk(KERN_ERR PFX "fail to setup dma, fallback to pio\n");
			em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);
			goto fallback_pio;
		}
	}

	drive->waiting_for_dma = 1;
	return 0;

fallback_pio:
	if (!phys_mapped) 
		ide_map_sg(drive, rq);
	return 1;
}

static int tangox_bmide_dma_end(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	u8 dma_stat;
	int mbus_stat;
        struct scatterlist *sg = hwif->sg_table;
	struct request *rq;
	int phys_mapped;

        rq = HWGROUP(drive)->rq;
	phys_mapped = (bio_flagged(rq->bio, BIO_PHYSICAL) ? 1 : 0);

	dma_stat = hwif->INB(hwif->dma_status);

	/*
	 * make sure DMA is not in progress
	 */
	if ((dma_stat & 0x7) == 1) {
		printk(KERN_ERR PFX "huh ? dma_end called while dma still "
		       "in progress...\n");
	}

	/* clear the INTR & ERROR bits */
	hwif->OUTB(dma_stat | 6, hwif->dma_status);

	/*
	 * confirm  whether MBUS transfer  is done  due to  the memory
	 * arbitration, IDE  device thinks  the DMA transfer  is done,
	 * but the data might be held in MBUS FIFO
	 */
	mbus_stat = em86xx_mbus_wait(g_mbus_reg, SBOX_IDEDVD);

	/* release mbus */
	em86xx_mbus_free_dma(g_mbus_reg, SBOX_IDEDVD);

	/* stop bus mastering */
	hwif->OUTB(0x4, hwif->dma_command);

	if (!phys_mapped) 
		dma_unmap_sg(&hwif->gendev, sg, hwif->sg_nents,
				hwif->sg_dma_direction);

	drive->waiting_for_dma = 0;

#ifdef USE_BOUNCE_BUF
	if(!mbus_stat && ((dma_stat & 0x1) == 0) && 
		(hwif->sg_dma_direction == DMA_FROM_DEVICE) && 
		(hwif->sg_nents > 1)) {
		int i;
		unsigned long len = 0, addr;
		unsigned long buf = (unsigned long) g_bounce_buf;
		for(i = 0; i< hwif->sg_nents; i++){
			addr = KSEG0ADDR(tangox_inv_dma_address(sg_dma_address(sg)));
			len = sg_dma_len(sg);

#ifndef USE_MBUS_MEMCPY
			memcpy((void *)addr, (void *)buf, len);
			mb();
			dma_cache_wback_inv(addr, len);
#else
			dma_cache_inv(addr, len);
			mbus_memcpy(g_mbus_reg, sg_dma_address(sg), 
					tangox_dma_address(CPHYSADDR(buf)), len);
#endif

			buf += len;
			sg = &hwif->sg_table[i+1];
		}
	}
#endif

	/* fake  dma  error  in  case  of mbus  timeout,  else  return
	 * dma_status error bit */
	return (mbus_stat == 0) ? (dma_stat & 1) : 0x1;
}

static int tangox_bmide_dma_test_irq(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	u8 dma_stat;

	dma_stat = hwif->INB(hwif->dma_status);
	if ((dma_stat & 0x4))
		return 1;
	return 0;
}

static int tangox_bmide_dma_lostirq(ide_drive_t *drive)
{
	return 0;
}

static int tangox_bmide_dma_timeout(ide_drive_t *drive)
{
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


/*
 * tangox_bmide_tune_drive
 */
static void tangox_bmide_tune_drive(ide_drive_t *drive, u8 pio)
{
	ide_hwif_t *hwif = HWIF(drive);

	if (pio == 255)
		pio = ide_get_best_pio_mode(drive, pio, 4, NULL);
	hwif->speedproc(drive, pio + XFER_PIO_0);
}

/* 
 * tangox_bmide_udma_filter
 */
static u8 tangox_bmide_udma_filter(ide_drive_t *drive)
{
	if ((gbus_read_reg32(REG_BASE_host_interface + IDECTRL_idestatus) & 0x02) == 0x02) {
		printk("%s: no 80 conductors cable, falling back to lower udma mode\n", drive->name);
		return 0x7; /* max. at UDMA2 */
	}
	return 0x3f; /* max. at UDMA5 */
}

/*
 * tangox_bmide_tune_chipset
 */
static u8 drive_speed[2] = { 0xff, 0xff };

static int tangox_bmide_tune_chipset(ide_drive_t *drive, u8 xferspeed)
{
	int didx;
	unsigned int ctrlreg, newflag;
        unsigned long flags;
	u8 mode, pio;

#ifdef CONFIG_RUNTIME_CLK_CALC
#define TIMING_MARGIN(x) (((x) * 105) / 100) /* add 5% margin */
//#define TIMING_MARGIN(x) (x) /* no margin added */
	extern unsigned long tangox_get_sysclock(void);
	unsigned int ide_clock = TIMING_MARGIN(tangox_get_sysclock() / 1000000); /* in MHz */
#endif
	
	didx = drive->select.b.unit;
	newflag = 0;

        spin_lock_irqsave(&bmide_tune_chipset_spin_lock, flags);

	if (xferspeed >= XFER_PIO_0 && xferspeed <= XFER_PIO_4) {
		/*
		 * setup timing for PIO mode
		 */
		mode = xferspeed - XFER_PIO_0;
		printk("%s: set to PIO mode %d\n", drive->name, mode);

		/* fast timing for PIO */
		newflag = 0x01;

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
#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX_DMA
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
#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX_UDMA
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
#endif
	else {
		printk("%s: unknown speed to be set %d\n", drive->name, xferspeed);
	}

	ctrlreg = gbus_read_reg32(REG_BASE_host_interface + IDECTRL_pri_idectl);
	ctrlreg &= ~(didx == 0 ? 0xf : 0xf0);
	ctrlreg |= newflag << (didx * 4);
	gbus_write_reg32(REG_BASE_host_interface + IDECTRL_pri_idectl, ctrlreg);

	spin_unlock_irqrestore(&bmide_tune_chipset_spin_lock, flags);

#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX_DMA
	if (xferspeed >= XFER_SW_DMA_0)
		pio = dma_2_pio(xferspeed);
	else
#endif
		pio = xferspeed - XFER_PIO_0;

	drive_speed[didx] = xferspeed;
	return ide_config_drive_speed(drive, xferspeed);
}

static void tangox_hw_setup(hw_regs_t *hw, unsigned long data_port,
		     unsigned long ctrl_port)
{
	int i;
	unsigned long reg;

	memset(hw, 0, sizeof (*hw));

	/* setup io_ports */
	reg = data_port;
	for (i = IDE_DATA_OFFSET; i <= IDE_STATUS_OFFSET; i++) {
		hw->io_ports[i] = reg;
		reg += 4;
	}

	hw->io_ports[IDE_CONTROL_OFFSET] = ctrl_port;
}


/*
 * init function, ide controller is registred here
 */
extern int tangox_bmide_enabled(void);

/* to calcuate the speed needed */
void tangox_bmide_calc_speed(void)
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
#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX_DMA
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
#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX_UDMA
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
#endif
	}

	spin_unlock_irqrestore(&bmide_tune_chipset_spin_lock, flags);
}

int __init tangox_bmide_init(void)
{
	ide_hwif_t *hwif;
	hw_regs_t hw;
	int index;
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
	tangox_hw_setup(&hw, REG_BASE_host_interface_BMIDE,
			REG_BASE_host_interface + IDECTRL_pri_ctrlblock);
	hw.dma = NO_DMA;
	/*
	 * Tango   has   two   interrupts   for   IDE   controller   :
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
	if ((index = ide_register_hw(&hw, 1, &hwif)) < 0) {
		printk("failed to register SMP86xx BM IDE controller\n");
		return -1;
	}
	printk("%s: SMP86xx Bus Mastering IDE controller\n", hwif->name);

	/*
	 * fill controller callbacks
	 */

        /* set up local I/O function entry points */
        hwif->INB = tangox_bmide_inb;
        hwif->INW = tangox_bmide_inw;
        hwif->INSW = tangox_bmide_insw;
        hwif->OUTB = tangox_bmide_outb;
        hwif->OUTW = tangox_bmide_outw;
        hwif->OUTBSYNC = tangox_bmide_outbsync;
        hwif->OUTSW = tangox_bmide_outsw;

	hwif->tuneproc = &tangox_bmide_tune_drive;
        hwif->speedproc = &tangox_bmide_tune_chipset;

	hwif->udma_filter = tangox_bmide_udma_filter;

	hwif->drives[0].autotune = 1;
	hwif->drives[1].autotune = 1;

#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX_DMA
	/* enable dma of both ide drives */
	hwif->drives[0].autodma = noautodma ? 0 : 1;
	hwif->drives[1].autodma = noautodma ? 0 : 1;
	hwif->rqsize = 256;

	/* we will do autotune at dmacheck time */
	if (noautodma == 0) {
		hwif->drives[0].autotune = 0;
		hwif->drives[1].autotune = 0;
	}

	/* enable  all  ultra dma  and  multiword  dma operation,  but
	 * disable single */
#ifdef CONFIG_BLK_DEV_BMIDE_TANGOX_UDMA
	hwif->ultra_mask = 0x3f; /* enable UDMA, up to UDMA5 */
#else
	hwif->ultra_mask = 0x0; /* disable UDMA */
#endif
	hwif->udma_four = 1;
	hwif->mwdma_mask = 0x07;
	hwif->swdma_mask = 0x0;

 	hwif->autodma = 1;
 	hwif->atapi_dma = 1;

	/* dma callbacks */
	hwif->ide_dma_on = tangox_bmide_dma_on;
	hwif->ide_dma_check = tangox_bmide_dma_check;
	hwif->dma_off_quietly = tangox_bmide_dma_off_quietly;
	hwif->dma_host_on = tangox_bmide_dma_host_on;
	hwif->dma_host_off = tangox_bmide_dma_host_off;
	hwif->ide_dma_test_irq = tangox_bmide_dma_test_irq;
	hwif->ide_dma_lostirq = tangox_bmide_dma_lostirq;
	hwif->ide_dma_timeout = tangox_bmide_dma_timeout;

	hwif->dma_setup = tangox_dma_setup;
	hwif->dma_exec_cmd = tangox_dma_exec_cmd;
	hwif->dma_start = tangox_dma_start;
	hwif->ide_dma_end = tangox_bmide_dma_end;

	hwif->dma_command = REG_BASE_host_interface + IDECTRL_bmic;
	hwif->dma_status = REG_BASE_host_interface + IDECTRL_bmis;

	em86xx_mbus_init();

#ifdef USE_BOUNCE_BUF
	if ((g_bounce_buf = (unsigned char *)__get_free_pages(GFP_KERNEL, DMA_BOUNCE_BUF_ORDER)) == NULL)
		printk("IDE: Can not allocate buffer for IDE transfer\n");

	printk("Bounce buffer starts at 0x%08lx\n", (unsigned long)g_bounce_buf);
#endif	

#endif
	return 0;
}

static void __exit tangox_bmide_exit(void)
{
#ifdef USE_BOUNCE_BUF
        free_pages((unsigned long)g_bounce_buf, DMA_BOUNCE_BUF_ORDER);
#endif
        return;
}


#ifdef MODULE
module_init(tangox_bmide_init);
module_exit(tangox_bmide_exit);
#endif

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

EXPORT_SYMBOL(tangox_bmide_calc_speed);

