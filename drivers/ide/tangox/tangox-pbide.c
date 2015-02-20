
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
  
/*
 * driver/ide/tangox/tangox_pbide.c
 * TANGOX PB IDE driver
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ide.h>
#include "tangox-ide.h"

extern int tangox_isaide_irq_map(void);
extern int tangox_isaide_enabled(void);
extern int tangox_isaide_cs_select(void);
extern int tangox_isaide_timing_slot(void);
extern void mbus_setup_dma_double(unsigned int regbase, unsigned int addr, 
		unsigned int count, unsigned int addr2, unsigned int count2, unsigned int width);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SMP86xx PB IDE driver");

static unsigned long g_regbase_read, g_regbase_write;

/*
 * io helpers for PIO access
 */
static void tangox_pbide_outb(u8 data, unsigned long port)
{
	gbus_write_reg16(port, data);
}

static void tangox_pbide_outw(u16 data, unsigned long port)
{
	gbus_write_reg16(port, data);
}

static void tangox_pbide_outbsync(ide_drive_t *drive, u8 data, unsigned long port)
{
	gbus_write_reg16(port, data);
}

static u8 tangox_pbide_inb(unsigned long port)
{
	u8 val;

	val = ((u8)(gbus_read_reg16(port) & 0xff));
	return val;
}

static u16 tangox_pbide_inw(unsigned long port)
{
	u16 val;

	val = ((u16)(gbus_read_reg16(port) & 0xffff));
	return val;
}

static void tangox_pbide_insw(unsigned long port, void *addr, u32 count)
{
	while (count--) {
		*(u16 *)addr = (u16)(gbus_read_reg16(port) & 0xffff);
		addr += 2;
	}
}

static void tangox_pbide_outsw(unsigned long port, void *addr, u32 count)
{
	while (count--) {
		gbus_write_reg16(port, *(u16 *)addr);
		addr += 2;
	}
}

//
// data structure initialization
//
static void tangox_pbide_init_hwif_ports(hw_regs_t *hw, unsigned long data_port, 
					unsigned long ctrl_port, int *irq)
{
	int i;
	unsigned long reg;

	memset(hw, 0, sizeof(*hw));

	// setup io_ports
	for (i = IDE_DATA_OFFSET, reg = data_port; i <= IDE_STATUS_OFFSET; i++) {
		hw->io_ports[i] = reg;
		reg += 4;
	}

	hw->io_ports[IDE_CONTROL_OFFSET] = ctrl_port;
	hw->dma = NO_DMA;

	if (irq) 
		*irq = tangox_isaide_irq_map();
}

/*
 * tangox_pbide_tune_drive
 */
static void tangox_pbide_tune_drive(ide_drive_t *drive, u8 pio)
{
        ide_hwif_t *hwif = HWIF(drive);

        if (pio == 255)
                pio = ide_get_best_pio_mode(drive, pio, 4, NULL);
        hwif->speedproc(drive, pio + XFER_PIO_0);
}

#ifdef CONFIG_BLK_DEV_PBIDE_TANGOX_DMA
static void tangox_pbide_dma_off_quietly(ide_drive_t *drive)
{
        drive->using_dma = 0;
}

static int tangox_pbide_dma_test_irq(ide_drive_t *drive)
{
        return 1;
}

static void tangox_pbide_dma_exec_cmd(ide_drive_t *drive, u8 command)
{
        /* issue  cmd to  drive,  and register  interrupt handler  for
         * command completion */
        //ide_execute_command(drive, command, &ide_dma_intr, (2 * WAIT_CMD), NULL);
}

static void tangox_pbide_dma_start(ide_drive_t *drive)
{
	return;
}

static int tangox_pbide_dma_lostirq(ide_drive_t *drive)
{
	return 0;
}

static int tangox_pbide_dma_timeout(ide_drive_t *drive)
{
	return 0;
}

static void tangox_pbide_dma_host_on(ide_drive_t *drive)
{
	return;
}

static void tangox_pbide_dma_host_off(ide_drive_t *drive)
{
	return;
}

static int tangox_pbide_dma_on(ide_drive_t *drive)
{
#if 0
        ide_hwif_t *hwif = HWIF(drive);
        if (hwif->ide_dma_host_on)
                hwif->ide_dma_host_on(drive);
#endif
        printk("IDE: DMA enabled for %s%s\n",
                drive->media == ide_disk ? "ATA DISK " :
                (drive->media == ide_cdrom ? "ATAPI CDROM " : ""),
                drive->name);
        drive->using_dma = 1;
        return 0;
}

/*
 * Calculate the peripheral bus timing needed for the register used by IDE
 * devices.
 */

/* 
 * The tables bellows use indices as Multi-word DMA 2, Multi-word DMA 1,
 * Multi-word DMA 0, PIO 4, PIO 3, PIO 2, PIO 1, and PIO 0 (From 0 to 7, 
 * respectively).
 */

/* For storing value pair */
typedef struct {
	unsigned char x;
	unsigned char y;
} value_pair;

/* Table used for calculate Ta and Tb */
static const value_pair tab_values[] = {
	{3,0}, {5,0}, {7,2}, {3,0}, {5,0}, {5,0}, {7,2}, {10,4}
};

/* Table used for calculate Tc */
static const value_pair tc_values[] = {
	{13,8}, {19,11}, {60,41}, {10,5}, {12,6}, {14,8}, {17,11}, {23,15}
};

/* Table used for calculate Td */
static const value_pair td_values[] = {
	{1,100}, {1,100}, {3,200}, {1,100}, {1,100}, {1,100}, {3,200}, {5,250}
};

/* 
 * Calculate the required timing for setting up peripheral bus timing register.
 *
 * Input: sysclk = system clock rate (in MHz)
 *        mode = IDE mode (0-2: Multiword DMA mode 2-0, 3-7: PIO mode 4-0)
 * Return: 0 = Error
 *         else = value for setting up the timing register
 */
static unsigned long get_pb_ide_timing(unsigned int sysclk, int mode)
{
	unsigned char ta = 0;
	unsigned char tb = 0;
	unsigned char tc = 0;
	unsigned char td = 0;
	long temp = 0;
	const value_pair *val_ptr = (const value_pair *)0;

	/* The formula is only effective between 100-240MHz */
	if ((sysclk < 100) && (sysclk > 240))
		return(0);
	else if ((mode < 0) && (mode > 7)) /* Mode index is 0-7 only */
		return(0);

	/* (x, y) = tab_values[mode] */
	/* (Ta + Tb) = Temp = (ceiling((sysclk - 100) * x / 140) + y) */
	/* Ta = ceiling(Temp / 2) */
	/* Tb = (Temp / 2) */
	val_ptr = &tab_values[mode];
	temp = ((sysclk - 100) * val_ptr->x) / 140;
	if ((temp * 140) < ((sysclk - 100) * val_ptr->x))
		temp++;
	temp += val_ptr->y;
	tb = ta = (unsigned char)((temp >> 1) & 0xff);
	if (temp & 0x1)
		ta++;

	/* (x, y) = tc_values[mode] */
	/* Tc = (ceiling((sysclk - 100) * x / 140) + y) */
	val_ptr = &tc_values[mode];
	tc = (unsigned char)(((sysclk - 100) * val_ptr->x) / 140) & 0xff;
	if ((tc * 140) < ((sysclk - 100) * val_ptr->x))
		tc++;
	tc += val_ptr->y;

	/* (x, y) = td_values[mode] */
	/* Td = ceiling(sysclk + y - 400) * x / y) */
	val_ptr = &td_values[mode];
	if ((temp = (sysclk + val_ptr->y) - 400) < 0)
		td = 0; /* It's possible that Td < 0, so we set it to 0 */
	else {
		td = (unsigned char)(((temp * val_ptr->x) / val_ptr->y) & 0xff);
		if ((td * val_ptr->y) < (temp * val_ptr->x))
			td++;
	}
	return((td << 24) | (tc << 16) | (tb << 8) | ta);
}

ide_startstop_t tangox_pbide_dma_intr(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	struct request *rq = HWGROUP(drive)->rq;

	hwif->ide_dma_end(drive);

	ide_end_request(drive, 1,rq->nr_sectors);

	return ide_stopped;
}

static int tangox_pbide_rwdma(ide_drive_t *drive, int read, struct request *rq)
{
	unsigned long size = 0;
	unsigned long addr = 0;

        ide_hwif_t *hwif = HWIF(drive);
        struct scatterlist *sg = hwif->sg_table;
	int phys_mapped;

	phys_mapped = (bio_flagged(rq->bio, BIO_PHYSICAL) ? 1 : 0);

	if (!phys_mapped) {
		dma_map_sg(&hwif->gendev, sg, hwif->sg_nents, hwif->sg_dma_direction);

		if (hwif->sg_nents > 2) { /* this shouldn't happen since we limited rqsize=16 for now*/
			printk("cannot handle multiple transfer yet. sg_nents=0x%x\n", hwif->sg_nents);
			dma_unmap_sg(&hwif->gendev, sg, hwif->sg_nents, hwif->sg_dma_direction);
			return 1;
		}
	}

	// setup switchbox and wait for its readiness
        if (em86xx_mbus_alloc_dma(SBOX_IDEFLASH,  1, &g_regbase_read, NULL)) {
                printk("fail to alloc read dma, fallback to pio.\n");
		return 1;
        }

        /* try to setup dma write channel */
        if (em86xx_mbus_alloc_dma(SBOX_IDEFLASH,  0, &g_regbase_write, NULL)) {
                printk("fail to alloc write dma, fallback to pio.\n");
		em86xx_mbus_free_dma(g_regbase_read, SBOX_IDEFLASH);
		return 1;
        }

	// setup handler
	if (drive->media == ide_disk)
		ide_set_handler(drive, &tangox_pbide_dma_intr, 2 * WAIT_CMD, NULL);

	if (!phys_mapped) {
		addr = sg_dma_address(sg); 
		size = sg_dma_len(sg);
	} else {
		addr = (unsigned int)rq->bio->bi_private;
		size = rq->bio->bi_size;
	}

	if (!phys_mapped) {
		if (hwif->sg_nents == 1) {

			/* perform a single (linear/rectangle) transfer */

			// setup PB automode registers
			gbus_write_reg32(REG_BASE_host_interface + PB_automode_start_address, 0);
			gbus_write_reg32(REG_BASE_host_interface + PB_automode_control,
					0x00140000 | ((read ? 1 : 0) << 16) | (size >> 1));

			if (read)
				dma_cache_inv((unsigned int)phys_to_virt(addr), size);
			else
				dma_cache_wback_inv((unsigned int)phys_to_virt(addr), size);

			em86xx_mbus_setup_dma(read ? g_regbase_read : g_regbase_write,
					addr, size, NULL, NULL, 1);
		} else { 
			/* perform a double transfer */
			unsigned int addr1 = 0;
			unsigned int size1 = 0;

			sg = &hwif->sg_table[1];

			addr1 = sg_dma_address(sg);
			size1 = sg_dma_len(sg);

			// setup PB automode registers
			gbus_write_reg32(REG_BASE_host_interface + PB_automode_start_address, 0);
			gbus_write_reg32(REG_BASE_host_interface + PB_automode_control,
					0x00140000 | ((read ? 1 : 0) << 16) | ((size+size1) >> 1));

			if (read) {
				dma_cache_inv((unsigned int)phys_to_virt(addr), size);
				dma_cache_inv((unsigned int)phys_to_virt(addr1), size1);
			} else {
				dma_cache_wback_inv((unsigned int)phys_to_virt(addr), size);
				dma_cache_wback_inv((unsigned int)phys_to_virt(addr1), size1);
			}

			mbus_setup_dma_double(read ? g_regbase_read : g_regbase_write, 
					addr, size, addr1, size1, 1);
		}
	} else {
		/* perform a single (linear/rectangle) transfer */

		// setup PB automode registers
		gbus_write_reg32(REG_BASE_host_interface + PB_automode_start_address, 0);
		gbus_write_reg32(REG_BASE_host_interface + PB_automode_control,
				0x00140000 | ((read ? 1 : 0) << 16) | (size >> 1));

		em86xx_mbus_setup_dma(read ? g_regbase_read : g_regbase_write, 
				addr, size, NULL, NULL, 1);
	}

	if (drive->media == ide_disk) {
		// send IDE command
		HWIF(drive)->OUTB(read ? (drive->addressing ? WIN_READDMA_EXT : WIN_READDMA) : 
			(drive->addressing ? WIN_WRITEDMA_EXT : WIN_WRITEDMA), IDE_COMMAND_REG);
	}

	return 0;
}

static int tangox_pbide_dma_setup(ide_drive_t *drive)
{
        ide_hwif_t *hwif = HWIF(drive);
        struct request *rq;
        int iswrite;
	int phys_mapped;

        drive->waiting_for_dma = 1;
        rq = HWGROUP(drive)->rq;
        iswrite = (rq_data_dir(rq) == WRITE);
        hwif->sg_dma_direction = iswrite ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
	phys_mapped = (bio_flagged(rq->bio, BIO_PHYSICAL) ? 1 : 0);
	
	if (!phys_mapped) {
		// this will do the merge
		ide_map_sg(drive, rq);
	}

        return tangox_pbide_rwdma(drive, !iswrite, rq);
}

static int tangox_ide_config_drive_for_dma(ide_drive_t *drive, int mode)
{
        ide_hwif_t *hwif = HWIF(drive);
        int config_allows_dma = 0;
        struct hd_driveid *id = drive->id;
        int speed;

        // enable DMA for CDROM by default
        if (drive->media == ide_disk || drive->media == ide_cdrom)
                config_allows_dma = 1;

        if (id && (id->capability & 1) && hwif->autodma && config_allows_dma) {
                speed = ide_max_dma_mode(drive);//ide_dma_speed(drive, mode);

                /* For CDROM, we do the max. MDMA mode 2 */
		//if ((speed > XFER_MW_DMA_2) && (drive->media == ide_cdrom))
		//	speed = XFER_MW_DMA_2;

                if (speed >= XFER_MW_DMA_0) {
                        if (hwif->speedproc)
                                hwif->speedproc(drive, speed);
                        ide_config_drive_speed(drive, speed);
                        return hwif->ide_dma_on(drive);
                }
        }

        tangox_pbide_dma_off_quietly(drive);
	return 0;
}

static int tangox_pbide_dma_check(ide_drive_t *drive)
{
	return tangox_ide_config_drive_for_dma(drive, 0);
}

static u8 drive_speed[2] = { 0xff, 0xff };

static void tangox_pbide_tune_chipset_drive(int drive, int speed, int verbose)
{
	extern unsigned long tangox_get_sysclock(void);
	int mode = speed & 0x0f;
	unsigned int timing;
	
	if (speed >= XFER_PIO_0 && speed <= XFER_PIO_4)
		mode -= 8;
	else if (speed >= XFER_MW_DMA_0 && speed <= XFER_MW_DMA_2)
		mode += 5;
	else
		return;	

	drive_speed[drive] = speed;

	// get_pb_ide_timing() receives mode in reversed order
	//   PIO : 7-3
	//   Multi-word DMA : 2-0
	mode = 7 - mode;
	timing = get_pb_ide_timing(tangox_get_sysclock() * 11 / (10*1000000), mode);
	gbus_write_reg32(REG_BASE_host_interface + PB_timing_slot(tangox_isaide_timing_slot()), timing);
}

static int tangox_pbide_tune_chipset(ide_drive_t *drive, u8 speed)
{
	tangox_pbide_tune_chipset_drive(drive->select.b.unit, speed, 1);

	return ide_config_drive_speed(drive, speed);
}

static int tangox_pbide_dma_end_io(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	struct request *rq = HWGROUP(drive)->rq;
        struct scatterlist *sg = hwif->sg_table;
	int phys_mapped;
	int stat = em86xx_mbus_wait(rq->cmd == READ ? g_regbase_read : 
					g_regbase_write, SBOX_IDEFLASH);

	phys_mapped = (bio_flagged(rq->bio, BIO_PHYSICAL) ? 1 : 0);

	if (!phys_mapped)
		dma_unmap_sg(&hwif->gendev, sg, hwif->sg_nents, hwif->sg_dma_direction);

	/* Free up MBUS channel */
	em86xx_mbus_free_dma(g_regbase_read, SBOX_IDEFLASH);
	em86xx_mbus_free_dma(g_regbase_write, SBOX_IDEFLASH);
	return((stat != 0) ? 1 : 0);
}

static int tangox_pbide_dma_end(ide_drive_t *drive)
{
	int status;

	drive->waiting_for_dma = 0;
	status = tangox_pbide_dma_end_io(drive);
	return(status);
}
#endif

/* to calcuate the speed needed */
void tangox_pbide_calc_speed(void)
{
	int didx;
	int mode, speed;
	unsigned int timing;
	// u8 xferspeed;
	extern unsigned long tangox_get_sysclock(void);
	
	if (tangox_isaide_enabled() == 0)
		return;
	
	for (didx = 0; didx < 2; didx++) {
		if ((speed = drive_speed[didx]) == 0xff)
			continue;
		
		mode = speed & 0x0f;
		
		if (speed >= XFER_PIO_0 && speed <= XFER_PIO_4)
			mode -= 8;
		else if (speed >= XFER_MW_DMA_0 && speed <= XFER_MW_DMA_2)
			mode += 5;
		else
			continue;
		
		// get_pb_ide_timing() receives mode in reversed order
		//   PIO : 7-3
		//   Multi-word DMA : 2-0
		mode = 7 - mode;
		timing = get_pb_ide_timing(tangox_get_sysclock() * 11 / (10*1000000), mode);
		gbus_write_reg32(REG_BASE_host_interface + PB_timing_slot(tangox_isaide_timing_slot()), timing);
	}
}

//
// initialize IDE driver
//
int __init tangox_pbide_init(void)
{
	int index, i;
	ide_hwif_t *hwif;
	hw_regs_t hw;
#ifdef MODULE
        ide_drive_t *drive = NULL;
#endif

	if (tangox_isaide_enabled() == 0)
		return 0;
	else
		tangox_pbide_init_hwif_ports(&hw, 
			REG_BASE_host_interface_ISAIDE(tangox_isaide_cs_select()), 0, &hw.irq);

       	index = ide_register_hw(&hw, 1, &hwif);
	printk("%s: SMP86xx PB IDE controller.\n", hwif->name);

	// initialize ide_hwifs data structure
	// supports only one interface
	for (i = index; i < index + 1; ++i) {
		// register DMA handler
		hwif = &ide_hwifs[i];

	        /* set up local I/O function entry points */
	        hwif->INB = tangox_pbide_inb;
	        hwif->INW = tangox_pbide_inw;
	        hwif->INSW = tangox_pbide_insw;
	        hwif->OUTB = tangox_pbide_outb;
	        hwif->OUTW = tangox_pbide_outw;
	        hwif->OUTBSYNC = tangox_pbide_outbsync;
	        hwif->OUTSW = tangox_pbide_outsw;

		hwif->rqsize = 16; 

	    if (!noautodma) {
#ifdef CONFIG_BLK_DEV_PBIDE_TANGOX_DMA 
		hwif->autodma = 1;
		hwif->atapi_dma = 1;
		hwif->ultra_mask = 0x00;	// dpbble all Ultra DMA
		hwif->mwdma_mask = 0x07;	// enable all Multi-word DMA
		hwif->swdma_mask = 0x80;	// dpbble all Single-word DMA

	        hwif->dma_setup = tangox_pbide_dma_setup;
	        hwif->dma_exec_cmd = tangox_pbide_dma_exec_cmd;
	        hwif->dma_start = tangox_pbide_dma_start;
		hwif->ide_dma_end = tangox_pbide_dma_end;

		hwif->ide_dma_check = tangox_pbide_dma_check;
		hwif->ide_dma_on = tangox_pbide_dma_on;
		hwif->ide_dma_test_irq = tangox_pbide_dma_test_irq;
		hwif->dma_off_quietly = tangox_pbide_dma_off_quietly;
		hwif->dma_host_on = tangox_pbide_dma_host_on;
		hwif->dma_host_off = tangox_pbide_dma_host_off;
		hwif->ide_dma_lostirq = tangox_pbide_dma_lostirq;
		hwif->ide_dma_timeout = tangox_pbide_dma_timeout;
	      	hwif->tuneproc = tangox_pbide_tune_drive;
		hwif->speedproc = tangox_pbide_tune_chipset;
#endif
	    }
	}

	return 0;
}

void __exit tangox_pbide_exit(void)
{
        return;
}

#ifdef MODULE
module_init(tangox_pbide_init);
module_exit(tangox_pbide_exit);
#endif

EXPORT_SYMBOL(tangox_pbide_calc_speed);

