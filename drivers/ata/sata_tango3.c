/*********************************************************************
 Copyright (C) 2001-2009
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/*
 * Driver for SMP864x/SMP865x builtin SATA Based on Synopsys DW SATA Host Core 
 * device driver and Linux libata driver support layer.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/config.h>
#include <linux/platform_device.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi.h>
#include <linux/blkdev.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_tcq.h>
#include <asm/scatterlist.h>
#include <linux/libata.h>
#include "libata.h"
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3api.h>

#define HSATA_TANGOX_DMA
#ifdef HSATA_TANGOX_DMA
//#define TANGOX_BOUNCE_BUF
#ifdef TANGOX_BOUNCE_BUF
#define TANGOX_MBUS_MEMCPY
#endif
#endif
#undef HSATA_VERBOSE

MODULE_AUTHOR ("Sigma Designs Inc.");
MODULE_DESCRIPTION ("TANGOX Bulid-in SATA Host Controller device driver");
MODULE_LICENSE ("GPL");  

#define DRV_NAME0    "Tangox SATA 0"
#define DRV_NAME1    "Tangox SATA 1"
#define DRIVER_VERSION "1.0"


/*HSATA Registers*/
#define HSATA_PIO_DATA_REG		0x0000
#define HSATA_SCR0_REG			0x0024
#define HSATA_SCR0_SPD_GET(v)	(((v) & 0x000000f0) >> 4)
#define HSATA_SCR1_REG			0x0028
#define HSATA_SCR2_REG			0x002C
#define HSATA_SCR3_REG			0x0030
#define HSATA_SCR4_REG			0x0034

#define HSATA_SERROR_REG        HSATA_SCR1_REG
#define HSATA_SERROR_ERR_BITS   0x0000ffff
#define HSATA_SCONTROL_REG      HSATA_SCR2_REG
#define HSATA_SACTIVE_REG       HSATA_SCR3_REG
#define HSATA_SNOTIFICATION_REG HSATA_SCR4_REG

#define HSATA_DMACR_TX_EN		0x01 /*| HSATA_DMACR_TXMODE_BIT*/
#define HSATA_DMACR_RX_EN		0x02 /*| HSATA_DMACR_TXMODE_BIT*/
#define HSATA_DMACR_TXRX_EN		0x03 | HSATA_DMACR_TXMODE_BIT
#define HSATA_DMACR_TXMODE_BIT	0x04
#define HSATA_DMACR_TXRX_CLEAR HSATA_DMACR_TXMODE_BIT

#ifdef HSATA_VERBOSE
#define CDR0 0x00
#define CDR1 0x04
#define CDR2 0x08
#define CDR3 0x0c
#define CDR4 0x10
#define CDR5 0x14
#define CDR6 0x18
#define CDR7 0x1c
#endif

#define HSATA_FEAT_REG           0x0004
#define HSATA_CMD_REG            0x001c
#define HSATA_STATUS_REG         0x001c
#define HSATA_CONTROL_REG        0x0020
#define HSATA_DMACR_REG          0x0070
#define HSATA_DBTSR_REG          0x0074
#define HSATA_INTPR_REG          0x0078
#define HSATA_INTPR_ERR_BIT      0x00000008
#define HSATA_INTPR_FP_BIT       0x00000002  /* new DMA setup FIS arrived */
#define HSATA_INTMR_REG          0x007C
#define HSATA_INTMR_ERRM_BIT     0x00000008
#define HSATA_INTMR_NEWFP_BIT    0x00000002
#define HSATA_ERRMR_REG          0x0080
#define HSATA_LLCR_REG           0x0084
#define HSATA_PHYCR_REG          0x0088
#define HSATA_PHYSR_REG          0x008C
#define HSATA_RXBISTPD_REG       0x0090
#define HSATA_RXBISTD1_REG       0x0094
#define HSATA_RXBISTD2_REG       0x0098
#define HSATA_TXBISTPD_REG       0x009C
#define HSATA_TXBISTD1_REG       0x00A0
#define HSATA_TXBISTD2_REG       0x00A4
#define HSATA_BISTCR_REG         0x00A8
#define HSATA_BISTFCTR_REG       0x00AC
#define HSATA_BISTSR_REG         0x00B0
#define HSATA_BISTDECR_REG       0x00B4
#define HSATA_TESTR_REG          0x00F4
#define HSATA_VER_REG			 0x00F8
#define HSATA_IDR_REG            0x00FC
#define HSATA_DMADR_REG          0x0400

#define TANGOX_SATA0_BASE		KSEG1ADDR(REG_BASE_host_interface + 0x3000) 
#define TANGOX_SATA1_BASE		KSEG1ADDR(REG_BASE_host_interface + 0x3800) 
#define TANGOX_SATA0_CTL_BASE	KSEG1ADDR(REG_BASE_host_interface + 0x4000) 
#define TANGOX_SATA1_CTL_BASE	KSEG1ADDR(REG_BASE_host_interface + 0x4040) 
#define TANGOX_SATA_IRQ0		IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_SATA_INT
#define TANGOX_SATA_DMA_IRQ0	IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_DMASATA_INT
#define TANGOX_SATA_IRQ1		IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_SATA1_INT
#define TANGOX_SATA_DMA_IRQ1	IRQ_CONTROLLER_IRQ_BASE  + LOG2_CPU_DMASATA1_INT

static const int tangox_sata_base[2]= {TANGOX_SATA0_BASE, TANGOX_SATA1_BASE};
static const int tangox_ctl_base[2] = {TANGOX_SATA0_CTL_BASE, TANGOX_SATA1_CTL_BASE};
static const int tangox_sata_irq[2] = {TANGOX_SATA_IRQ0, TANGOX_SATA_IRQ1};
static const int tangox_sata_dma_irq[2] = {TANGOX_SATA_DMA_IRQ0, TANGOX_SATA_DMA_IRQ1};
static const int tangox_sbox[2] = {SBOX_SATA1, SBOX_SATA2};

/*
 * Per device data struct
 */
#define HSATA_DEVICE_MAGIC  13
static unsigned int hsata_magic = HSATA_DEVICE_MAGIC;
struct hsata_device
{
	unsigned int    magic;
	struct device   *adev;
 	/* ptr to port info  whose priv data pts to us */
	struct ata_port_info *ppi;
	/* see call to dev_set_devdata() */
	struct ata_host  *host;  
	unsigned long membase;
	struct tasklet_struct	hotplug_tasklet;
};

#define HSDEV_FROM_HOST_SET(hs) (struct hsata_device*)hs->private_data
#define HSDEV_FROM_AP(ap) (struct hsata_device*)ap->host->private_data
#define HSDEV_FROM_QC(qc) (struct hsata_device*)qc->ap->host->private_data


#ifdef HSATA_TANGOX_DMA
static unsigned long g_mbus_reg[2] = {0, 0} ;
static unsigned int  g_next_sg[2]  = {0, 0};

#ifdef TANGOX_BOUNCE_BUF
/*maximum = 128 sectors = 2^7 sectors = 2^7 * 2^9 bytes = 2^16 bytes*/
/*refer to .max_sectors = 128 */
#define DMA_BOUNCE_BUF_ORDER  (19 - PAGE_SHIFT)
static unsigned char *bounce_buf0 = NULL;
static unsigned char *bounce_buf1 = NULL;
#endif

#define TANGOX_BURST_LENGTH_TX 16 
#define BURST_LENGTH_TX 24 
#define BURST_LENGTH_RX 64
#define HSATA_DMA_DBTSR  (BURST_LENGTH_RX << 16) | (BURST_LENGTH_TX << 0) 
#endif /* HSATA_TANGOX_DMA */

/* Throttle to gen1 speed */
static int gen1only = 0;

/* Interrupt stuff*/
static unsigned int  hsata_inum; /* number of interrupts we've seen */
static void hsata_enable_interrupts(struct hsata_device *hsdev) 
{ 
      volatile u32 val32; 
      /* enable all err interrupts */ 
      writel( 0xffffffff,(void *)(hsdev->membase + HSATA_ERRMR_REG)); 
      val32 = readl((void*)(hsdev->membase + HSATA_INTMR_REG)); 
      writel( val32 | HSATA_INTMR_ERRM_BIT | HSATA_INTMR_NEWFP_BIT,
			  (void*)(hsdev->membase + HSATA_INTMR_REG)); 
      val32 = readl((void*)(hsdev->membase + HSATA_INTMR_REG)); 
      DPRINTK("%s INTMR=0x%x\n", __FUNCTION__, val32 );
}
static void hsata_disable_interrupts(struct hsata_device *hsdev) 
{ 
	  volatile u32 val32;
      /* disable all err interrupts */ 
      writel(0, (void*)(hsdev->membase + HSATA_INTMR_REG)); 
      val32 = readl((void*)(hsdev->membase + HSATA_INTMR_REG)); 
      DPRINTK("%s INTMR=0x%x\n", __FUNCTION__, val32 );
}
static int hsata_scr_read(struct ata_port *ap, unsigned int sc_reg, u32 *val)
{
	void __iomem *mmio =  (void __iomem *)(ap->ioaddr.scr_addr + (sc_reg * 4));

	if (mmio) {
		*val = readl(mmio);
		return 0;
	}
	return -EINVAL;
}

static int hsata_scr_write(struct ata_port *ap, unsigned int sc_reg, u32 val)
{
	void __iomem *mmio =  (void __iomem *)(ap->ioaddr.scr_addr + (sc_reg * 4));

	if (mmio) {
		writel(val, mmio);
		return 0;
	}
	return -EINVAL;
}

static void tango3_sata_init(void)
{
	unsigned  int val;
	unsigned int cfg = 0;
	int tangox_get_sata_channel_cfg(unsigned int *);
	static int sata_init = 0;

	if (sata_init != 0)
		return;
	sata_init = 1;

	tangox_get_sata_channel_cfg(&cfg);

	/* bit14: force gen1? */
	gen1only = (cfg & (1 << 14)) ? 1 : 0; /* force gen1 speed? */

	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x0c));

	/* bit15: internal clock? */
	if (cfg & (1 << 15))
		val |= (1 << 24);	/* internal clock routing */
	else
		val &= ~(1 << 24);	/* external clock is used */
	writel(val, (void *)(TANGOX_SATA0_CTL_BASE + 0x0c));
	DPRINTK("PHY stat1(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x0c, val);

	/* 
	   bit0: RX SSC port0
	   bit1: RX SSC port1
	   bit2: TX SSC port0/1
	 */
	writel(0x28903 | 
		((cfg & 1) ? 0x200 : 0) | 
		((cfg & 2) ? 0x1000 : 0), 
		(void *)(TANGOX_SATA0_CTL_BASE + 0x10));

	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x14));
	val &= ~0x7fe;
	val |= ((cfg & 4) ? 0x400 : 0); /* TX SSC enable or not */

	/* bit7..4: reference clock frequency */
	switch((cfg >> 4) & 0xf) {
		case 0: /* 120MHz ref clock */
			val |= 0x12c;
			break;
		case 2: /* 60MHz ref clock */
			val |= 0x128;
			break;
		case 4: /* 30MHz ref clock */
			val |= 0x12a;
			break;
		case 1: /* 100MHz ref clock */
			val |= 0x234;
			break;
		case 3: /* 50MHz ref clock */
			val |= 0x230;
			break;
		case 5: /* 25MHz ref clock */
			val |= 0x232;
			break;
		default:
			DPRINTK("Invalid frequency selection specified: %d\n", (cfg >> 4) & 0xf);
			val |= 0x12c;
			break;
	}
	writel(val, (void *)(TANGOX_SATA0_CTL_BASE + 0x14));

	val = readl((void *)(TANGOX_SATA0_CTL_BASE + 0x10));
	DPRINTK("PHY stat2(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x10, val);
	val = readl((void*)(TANGOX_SATA0_CTL_BASE + 0x14));
	DPRINTK("PHY stat3(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x14, val);
	val = readl((void*)(TANGOX_SATA0_CTL_BASE + 0x18));
	DPRINTK("PHY stat4(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x18, val);
	val |= 1<<16; /* fast tech */
	val |= 1<<18; /* 3.3 v */
	DPRINTK("Setting PHY stat4(0x%x) to 0x%x\n", (TANGOX_SATA0_CTL_BASE + 0x18), val);
	writel(val, (void *)(TANGOX_SATA0_CTL_BASE + 0x18));
	val = readl( (void *)(TANGOX_SATA0_CTL_BASE + 0x18));
	DPRINTK("PHY stat4(0x%x)=0x%x\n", TANGOX_SATA0_CTL_BASE + 0x18, val);

	//DPRINTK("Resetting SATA controller...this should be done in libata\n");
	
	/* This need to be called once */
	em86xx_mbus_init();
}

#ifdef HSATA_VERBOSE
static void tango3_cdr_dump(int port)
{
	int val;
	/* this is done by hsata_setup_port*/ 
	//printk("CDR register dump on port=0x%x\n", port);
	//val = readw(tangox_sata_base[port] + CDR0);
	//printk("CDR0=0x%x\n", val&0xffff);
	val = readb(tangox_sata_base[port] + CDR1);
	printk("CDR1=0x%x\n", val&0xff);
	val = readb(tangox_sata_base[port] + CDR2);
	printk("CDR2=0x%x\n", val&0xff);
	val = readb(tangox_sata_base[port] + CDR3);
	printk("CDR3=0x%x\n", val&0xff);
	val = readb(tangox_sata_base[port] + CDR4);
	printk("CDR4=0x%x\n", val &0xff);
	val = readb(tangox_sata_base[port] + CDR5);
	printk("CDR5=0x%x\n", val &0xff);
	val = readb(tangox_sata_base[port] + CDR6);
	printk("CDR6=0x%x\n", val &0xff);
	val = readb(tangox_sata_base[port] + CDR7);
	printk("CDR7=0x%x\n", val & 0xff);
}
#endif

static int hsata_qc_complete( struct ata_port *ap, struct ata_queued_cmd *qc,
                              u32 check_status )
{
	u8 status = 0;
	int i=0;

	if (check_status) {
		/* check altstatus */
		i=0;
		do {
			/* check main status, clearing INTRQ */
			status = ata_chk_status(ap);
			DPRINTK("STATUS (0x%x) [%d]\n", status, i);
			if (status & ATA_BUSY)	{
        			DPRINTK("STATUS BUSY (0x%x) [%d]\n", status, i);
			}
			if (++i>10)
				break;
		} while (status & ATA_BUSY);

		if (status & (ATA_BUSY | ATA_ERR | ATA_DF))  {
			DPRINTK( "STATUS BUSY/ERROR (0x%x) [%d]\n", status, i);
		}
	}
	DPRINTK( "QC COMPLETE status=0x%x ata%u: protocol %d\n", 
        		status, ap->print_id, qc->tf.protocol);

	/* complete taskfile transaction */
	qc->err_mask |= ac_err_mask(status);
	ata_qc_complete( qc );

	return 0;
}

#ifdef HSATA_TANGOX_DMA
static int tangox_controller(struct hsata_device *hsdev)
{
	return (hsdev->membase == tangox_sata_base[0]? 0:1);
}

static void  tangox_mbus_intr(int irq, void *arg)
{
	struct ata_queued_cmd *qc = (struct ata_queued_cmd *)arg;
	struct scatterlist *sg = qc->__sg;  
	struct hsata_device *hsdev = HSDEV_FROM_QC(qc);
	int controller = tangox_controller(hsdev);

	if (g_next_sg[controller] > qc->n_elem)
		return; /* should be the last interrupt */
	else if (g_next_sg[controller] == qc->n_elem) { /* no more sg */
		g_next_sg[controller]++;
		em86xx_mbus_setup_dma_void(g_mbus_reg[controller]);
		return;
	}

	em86xx_mbus_wait(g_mbus_reg[controller], tangox_sbox[controller]);

	/*
	 * setup a new mbus transfer
	 */
	sg = &qc->__sg[g_next_sg[controller]];
	g_next_sg[controller]++;
#ifdef  HSATA_VERBOSE
	printk("%s setup_dma g_mbus_reg[%d] =0x%x address=0x%x len=0x%x n_elem=0x%x g_next_sg=0x%x\n",
			__FUNCTION__, controller, g_mbus_reg[controller], sg_dma_address(sg),sg_dma_len(sg),
			qc->n_elem,g_next_sg[controller] );
#endif
	if (em86xx_mbus_setup_dma(g_mbus_reg[controller], sg_dma_address(sg),
				  sg_dma_len(sg),
				  tangox_mbus_intr, qc, 0)) {
		printk("fail to resetup dma, wait for timeout...\n");
	}
}


static irqreturn_t hsata_dma_isr(int irq, void *dev_id, int id)
{
	struct ata_host *host = (struct ata_host*)dev_id;
	struct hsata_device *hsdev = HSDEV_FROM_HOST_SET(host);
	int val;
	unsigned long flags;
	int port = -1;

 	spin_lock_irqsave( &host->lock, flags );

	if(irq == tangox_sata_dma_irq[0])
        	port = 0;
	else if(irq == tangox_sata_dma_irq[1])
        	port = 1;
	else { 
		printk("irq=0x%x is not valid.\n", irq);
	}

	if( port != id ){
	   	spin_unlock_irqrestore( &host->lock, flags );
		return IRQ_HANDLED;
	}

	if(readl((void *)(tangox_ctl_base[port]+0x08))){
#ifdef HSATA_VERBOSE
		printk("%s got dma interrupt irq=0x%x, port =0x%x 08=0x%x\n", 
			__FUNCTION__, irq, port, readl(tangox_ctl_base[port]+0x08));
#endif
		/*clear dma interrupt*/
		writel( 0, (void*)(tangox_ctl_base[port]+0x08));

		val = readl((void *) (hsdev->membase + HSATA_DMACR_REG) );
	    if (val & 0x01) {
        		writel( 0/*HSATA_DMACR_TXRX_CLEAR*/,(void *)(hsdev->membase + HSATA_DMACR_REG));
	    }else if(val & 0x02) {
        		writel( 0, (void *)(hsdev->membase + HSATA_DMACR_REG));
		}

		em86xx_mbus_wait(g_mbus_reg[port], tangox_sbox[port]);
		em86xx_mbus_free_dma(g_mbus_reg[port], tangox_sbox[port]);

	}
   	spin_unlock_irqrestore( &host->lock, flags );
	return IRQ_HANDLED;
}

static irqreturn_t hsata_dma_isr_0(int irq, void *dev_id)
{
	return hsata_dma_isr(irq, dev_id, 0);
}

static irqreturn_t hsata_dma_isr_1(int irq, void *dev_id)
{
	return hsata_dma_isr(irq, dev_id, 1);
}

static unsigned long get_dma_len(struct ata_queued_cmd *qc)
{
        unsigned long len = 0;
	struct scatterlist *sg;

	ata_for_each_sg(sg, qc) {
	        len += sg_dma_len(sg);
	}

        return len;
}

static void hsata_dma_xfer_complete( struct ata_host *host, u32 check_status )
{
	struct ata_port *ap;
	struct ata_queued_cmd *qc;
	u8     tag = 0;

	ap = host->ports[0];

	tag = ap->active_tag;
	qc = ata_qc_from_tag( ap, tag );

	DPRINTK( "active_tag=%d  protocol=%d qc=0x%x \n", tag, qc->tf.protocol ,qc);

	switch (qc->tf.protocol) 
	{
		case ATA_PROT_DMA:
		case ATA_PROT_ATAPI_DMA:
#ifdef TANGOX_BOUNCE_BUF	
			/* if it's read, copy back from bounce buf*/
			if(qc->n_elem > 1 && (qc->dma_dir == DMA_FROM_DEVICE)){
				unsigned long tmp_buf;
				struct scatterlist *sg;
				struct hsata_device *hsdev = HSDEV_FROM_QC(qc);
				int controller = tangox_controller(hsdev);
				if(controller == 0)
					tmp_buf  = KSEG1ADDR(bounce_buf0);
				else
					tmp_buf  = KSEG1ADDR(bounce_buf1);
	
				ata_for_each_sg(sg, qc) {
					unsigned long len, addr;
                    len = sg_dma_len(sg);
               		addr= (unsigned long)page_address(sg->page);

#ifndef TANGOX_MBUS_MEMCPY
		            memcpy((void *)addr,(void *)tmp_buf, len);
					mb();
					dma_cache_wback_inv(addr, len);
#else
					dma_cache_inv(addr, len);
                    mbus_memcpy(g_mbus_reg[controller], tangox_dma_address(CPHYSADDR(addr)), 
					tangox_dma_address(CPHYSADDR(tmp_buf)), len);
#endif
        	       	tmp_buf += len;
				}
			}
#else /*chainning */
			if(qc->dma_dir == DMA_FROM_DEVICE) {
				struct scatterlist *sg;
				ata_for_each_sg(sg, qc) {
					unsigned long len, addr;

					len = sg_dma_len(sg);
					addr= (unsigned long)page_address(sg->page);
					dma_cache_inv(addr, len);
				}
			}
#endif
			dma_unmap_sg((struct device *)qc->dev, qc->__sg, qc->n_elem, qc->dma_dir);

			/* !!! FALL THRU TO NEXT CASE !!! */
		case ATA_PROT_ATAPI:
			if (unlikely(hsata_qc_complete( ap, qc, check_status )))
            			;
         	break;
      
		case ATA_PROT_ATAPI_NODATA:
		case ATA_PROT_NODATA:
			DPRINTK( KERN_ERR "WE SHOULDN'T GET HERE");
		 break;
	}
}
#endif

/*
 * hotplug callback
 */
static void hsata_link_check(unsigned long data)
{
	struct device *dev; 
	struct ata_host *host;
	struct ata_port *ap;
	struct ata_eh_info *ehi;

	dev = (struct device *)data;
	host = dev_get_drvdata( dev );
	ap =  host->ports[0];
	ehi = &ap->eh_info;

	ata_ehi_clear_desc(ehi);
	ata_ehi_hotplugged(ehi);
	ata_port_freeze(ap);
}

static irqreturn_t hsata_isr( int irq, void *dev_instance)
{
	struct ata_host *host = (struct ata_host*)dev_instance;
	struct hsata_device *hsdev = HSDEV_FROM_HOST_SET(host);
	unsigned int handled = 0;
	struct ata_port *ap;
	struct ata_queued_cmd *qc;
	unsigned long flags;
	u8 status = 0;
	volatile u32 val32;
	volatile u32 intpr;
	u32 err_interrupt;
	u32 tag_mask;
	volatile u32 sactive, sactive2;
	u8  tag;
	u32 num_processed;

	ap = host->ports[0];
	hsata_inum++;  
	
	spin_lock_irqsave( &host->lock, flags );

	/* clear all*/
	intpr = readl((void*)(hsdev->membase + HSATA_INTPR_REG));
	writel( intpr, ((void *)hsdev->membase + HSATA_INTPR_REG));  

	/* 
	 * ERROR INTERRUPT?
	 */
	if (intpr & HSATA_INTPR_ERR_BIT){
		val32 = readl((void *)(hsdev->membase + HSATA_SERROR_REG)); 
#ifdef HSATA_VERBOSE 
		DPRINTK("got interrupt  INTPR reg=0x%x hsm_task_state=0x%x\n", intpr, ap->hsm_task_state );
		DPRINTK("HSATA_DMACR_REG(0x70)=0x%x\n", (unsigned int)readl( hsdev->membase + HSATA_DMACR_REG ));
		DPRINTK("HSATA_DBTSR_REG(0x74)=0x%x\n", (unsigned int)readl( hsdev->membase + HSATA_DBTSR_REG ));
		DPRINTK("HSATA_INTPR_REG(0x78)=0x%x\n", (unsigned int)readl( hsdev->membase + HSATA_INTPR_REG ));
		DPRINTK("HSATA_INTMR_REG(0x7c)=0x%x\n", (unsigned int)readl( hsdev->membase + HSATA_INTMR_REG ));
		DPRINTK("HSATA_ERRMR_REG(0x80)=0x%x\n", (unsigned int)readl( hsdev->membase + HSATA_ERRMR_REG ));
#endif
		DPRINTK("SERROR=0x%08x INTPR=0x%x\n", val32, intpr );
		/* hotplug */
		if(val32 & (SERR_PHYRDY_CHG | SERR_DEV_XCHG));
			tasklet_schedule(&hsdev->hotplug_tasklet);

		writel( val32, (void *)(hsdev->membase + HSATA_SERROR_REG));  /* to clear */
		err_interrupt = 1;
		handled = 1;
		goto DONE;
		/* things are going to go to crap from here ... */
	}
	else
		err_interrupt = 0;

#ifdef HSATA_VERBOSE
   	{
		u32 val32 = readl((void *)(hsdev->membase + HSATA_SACTIVE_REG));
		DPRINTK( "SACTIVE=0x%x\n", val32 );
	}
#endif
	/* 
	 * ACTIVE TAG
	 * At this point we need to figure out for which tags we have gotten a
	 * completion interrupt.  One interrupt may serve as completion for 
	 * more than one operation when commands are queued (NCQ).
	 * We need to process each completed command.
	 */
PROCESS:  /* process completed commands */
	sactive = readl((void *)(hsdev->membase + HSATA_SACTIVE_REG));  /* remaining pending */
	if (sactive)
		DPRINTK( "UNEXPECTED SACTIVE???  sactive=0x%x\n", sactive );
	tag = ap->active_tag;
	tag_mask = 0x01 << tag;
	/* 
	 * Check main status, clearing INTRQ 
	 */
	status = ata_chk_status(ap); 
   	DPRINTK( "status=0x%x err_interrupt=0x%x\n", status , err_interrupt);

	if (!err_interrupt && (status & ATA_BUSY))	{
		DPRINTK("NOT OUR INTERRUPT - STATUS BUSY (0x%x)  INTPR=0x%x\n", status, intpr);
		goto NOTOURINT;
   	}

	tag = 0;
	num_processed = 0;
	while (tag_mask) {
		num_processed++;
		while (!(tag_mask & 0x01)) {
			tag++;
			tag_mask >>= 1;
		}
		tag_mask &= (~0x01);
		qc = ata_qc_from_tag( ap, tag );

		 /* to be picked up by downstream completion functions */
		qc->ap->active_tag = tag; 
   
		if (status & ATA_ERR) {
			DPRINTK( "INTERRUPT ATA_ERR (0x%x)\n", status);
			hsata_qc_complete( ap, qc, 1);
			handled = 1;
			goto DONE;
      	}
#ifdef HSATA_TANGOX_DMA
		/* Process completed command*/
		if((ap->hsm_task_state ==HSM_ST_LAST) && 
				(qc->tf.protocol == ATA_PROT_DMA || 
				 qc->tf.protocol == ATA_PROT_ATAPI_DMA))
			hsata_dma_xfer_complete(host, 1);
#endif
		handled = ata_host_intr(ap, qc);
		if(handled == 1)
			goto DONE;
	} /* while tag_mask */

	/*
	 * Check to see if any commands completed while we were processing our initial
	 * set of completed commands (reading of status clears interrupts, so we might
	 * miss a completed command interrupt if one came in while we were processing --
	 * we read status as part of processing a completed command).
	 */
	sactive2 = readl((void *)(hsdev->membase + HSATA_SACTIVE_REG));  
	if (sactive2 != sactive) {
	   DPRINTK(  "MORE COMPLETED - sactive=0x%x  sactive2=0x%x\n",
        		         sactive, sactive2 );
	   goto PROCESS;
	}

	handled = 1;
	goto DONE;

NOTOURINT:
	status = ata_chk_status(ap);
	DPRINTK( "NOT OUR INTERRUPT status=0x%x", status );
	handled = 0;
DONE:
	spin_unlock_irqrestore( &host->lock, flags );
	return IRQ_RETVAL(handled);
}

static void hsata_irq_clear( struct ata_port *ap )
{
	struct hsata_device *hsdev ;
	hsdev = HSDEV_FROM_AP(ap);

	DPRINTK( "id=%d [inum=%d]\n", ap->print_id, hsata_inum );

	/* read status reg to clear interrupt in controller */
	ata_chk_status(ap);

	/* reenable interrupt, for hotplug*/
	hsata_enable_interrupts( hsdev );
}

static void hsata_host_init(	struct hsata_device  *hsdev)
{
	unsigned int  pid, ver;
	u32 val32;


	/* Read IDR and Version registers*/
	pid = readl((void *)(hsdev->membase + HSATA_IDR_REG));
	ver = readl((void *)(hsdev->membase + HSATA_VER_REG));

	printk("SATA version 0x%x ID 0x%x is detected\n", ver, pid );

	/* some other initializations here*/
	tango3_sata_init();

	/*
	 * We clear this bit here so that we can later on check to see if other bus
	 * errors occured (during debug of this driver).
	 */
	val32 = readl((void *) (hsdev->membase + HSATA_SERROR_REG));
	writel( val32, (void *)(hsdev->membase + HSATA_SERROR_REG));
}

static void hsata_setup_port(struct ata_ioports *port, unsigned long base)
{
	port->cmd_addr       = (void __iomem *)(base + 0x00); 
	port->data_addr      = (void __iomem *)(base + 0x00);

	port->error_addr     = (void __iomem *)(base + 0x04);
	port->feature_addr   = (void __iomem *)(base + 0x04);

	port->nsect_addr     = (void __iomem *)(base + 0x08);

	port->lbal_addr      = (void __iomem *)(base + 0x0c);
	port->lbam_addr      = (void __iomem *)(base + 0x10);
	port->lbah_addr      = (void __iomem *)(base + 0x14);

	port->device_addr    = (void __iomem *)(base + 0x18);

	port->command_addr   = (void __iomem *)(base + 0x1c);
	port->status_addr    = (void __iomem *)(base + 0x1c);

	port->altstatus_addr = (void __iomem *)(base + 0x20);
	port->ctl_addr       = (void __iomem *)(base + 0x20);
	port->scr_addr		 = (void __iomem *)(base + HSATA_SCR0_REG);
	 /* we better never use this */
	port->bmdma_addr     = (void __iomem *)0xcdcdcdcd; 

}

static void hsata_reset_port( struct ata_port *ap )
{
	hsata_scr_write(  ap, 2, 0x301 | (gen1only<<4));
	mdelay(5);
}

static int hsata_port_start( struct ata_port *ap , int port)
{
	int status = 0;
	struct hsata_device *hsdev ;
	volatile u32    val32;
	hsdev = HSDEV_FROM_AP(ap);

	DPRINTK(  "id=%d port_num=%d port=0x%x\n", ap->print_id, ap->port_no, port);

	/* Grab ptr to this top-level device data.  */
	hsdev->host = ap->host;  

#ifdef HSATA_TANGOX_DMA
	status = request_irq( tangox_sata_dma_irq[port],
				((port == 0) ? hsata_dma_isr_0 : hsata_dma_isr_1),
				IRQF_DISABLED,
                ((port == 0) ? "HSATA0-DMA": "HSATA1-DMA"),    
                ap->host ); 
	if (status){
		DPRINTK(  "DMA request_irq FAILED  (status = %d) irq=0x%x\n", 
					status, tangox_sata_dma_irq[0]);
		status = -ENOMEM;
		goto CLEANUP;
	}

	/* Don't enable yet -- we do that right before a xfer */
	writel( 0/*HSATA_DMACR_TXMODE_BIT*/,(void *)(hsdev->membase + HSATA_DMACR_REG)); 
#endif 

	hsata_reset_port( ap );

	val32 = readl((void *)(hsdev->membase + HSATA_SCR0_REG));
	switch (HSATA_SCR0_SPD_GET(val32))
	{
		case 0x0:
			DPRINTK("****SCR0=0x%x NO NEGOTIATED SPEED!!! ****\n", val32);
		break;
		case 0x1:
			DPRINTK("****SCR0=0x%x GEN I RATE NEGOTIATED ****\n", val32);
		break;
		case 0x2:
			DPRINTK( "****SCR0=0x%x GEN II RATE NEGOTIATED ****\n", val32);
		break;
	}

CLEANUP:
	if (status) {
#ifdef HSATA_TANGOX_DMA
		free_irq(TANGOX_SATA_DMA_IRQ0, hsdev );
#endif
	}
	return status;
}


static int hsata_port_start_0( struct ata_port *ap )
{
	return hsata_port_start( ap , 0);
}

static int hsata_port_start_1( struct ata_port *ap )
{
	return hsata_port_start( ap , 1);
}

static int hsata_scsi_ioctl( struct scsi_device *scsidev, int cmd, 
                             void __user *arg )
{
	return ata_scsi_ioctl( scsidev, cmd, arg );
}

static struct ata_device * hsata_ata_find_dev(struct ata_port *ap, int id)
{
	if (likely(id < ATA_MAX_DEVICES))
		return &ap->device[id];
	return NULL;
}

static struct ata_device * hsata_ata_scsi_find_dev(struct ata_port *ap,
					const struct scsi_device *scsidev)
{
	/* skip commands not addressed to targets we simulate */
	if (unlikely(scsidev->channel || scsidev->lun))
		return NULL;

	return hsata_ata_find_dev(ap, scsidev->id);
}

static int hsata_scsi_queuecmd( struct scsi_cmnd *cmd, 
                                void (*done)(struct scsi_cmnd *) )
{

#ifdef HSATA_TANGOX_DMA
	struct ata_port *ap;
	struct ata_device *dev;
	struct scsi_device *scsidev = cmd->device;
	int dir, controller;
	struct hsata_device *hsdev;

	ap = ata_shost_to_port(scsidev->host);
	dev = hsata_ata_scsi_find_dev(ap, scsidev);
	dir = cmd->sc_data_direction;
	hsdev = HSDEV_FROM_AP(ap);
	controller = tangox_controller(hsdev);
	g_mbus_reg[controller] = 0;

	if ((cmd->request_bufflen < 1024) || ((cmd->request_bufflen % 2352) == 0) ||
		em86xx_mbus_alloc_dma(tangox_sbox[controller], (dir == DMA_FROM_DEVICE) ? 1 : 0, 
			&g_mbus_reg[controller], NULL)) {
		dev->flags |= ATA_DFLAG_PIO;
		DPRINTK("set to PIO mode len=0x%x\n", cmd->request_bufflen);
	}else{
		dev->flags &= ~ATA_DFLAG_PIO;
		DPRINTK("set to DMA mode len=0x%x\n", cmd->request_bufflen);
	}
#endif

	DPRINTK( "sn=%d\n", cmd->serial_number );
	return ata_scsi_queuecmd( cmd, done );
}

/*the same as ata_scsi_slave_config, remove later one*/
static int tangox_ata_scsi_slave_config(struct scsi_device *sdev)
{
        sdev->use_10_for_rw = 1;
        sdev->use_10_for_ms = 1;

        blk_queue_max_phys_segments(sdev->request_queue, LIBATA_MAX_PRD);
		sdev->manage_start_stop = 1;

        if (sdev->id < ATA_MAX_DEVICES) {
                struct ata_port *ap;
                struct ata_device *dev;

				ap = ata_shost_to_port(sdev->host);
				dev = hsata_ata_scsi_find_dev(ap, sdev);
				
                blk_queue_max_sectors(sdev->request_queue, ATA_MAX_SECTORS);
                if ((dev->flags & ATA_DFLAG_LBA48) &&
                    ((dev->flags & ATA_DFLAG_CDB_INTR) == 0)) {
                        /*
                         * do not overwrite sdev->host->max_sectors, since
                         * other drives on this host may not support LBA48
                         */
                        printk(KERN_INFO "ata%u: dev %u max request %d sectors (lba48)\n",
                               ap->print_id, sdev->id, sdev->host->max_sectors);
                 } else {
                        printk(KERN_INFO "ata%u: dev %u max request %d sectors (non lba48)\n",
                               ap->print_id, sdev->id, sdev->host->max_sectors);
                }

                /*
                 * SATA DMA transfers must be multiples of 4 byte, so
                 * we need to pad ATAPI transfers using an extra sg.
                 * Decrement max hw segments accordingly.
                 */
                if (dev->class == ATA_DEV_ATAPI) {
                        request_queue_t *q = sdev->request_queue;
                        blk_queue_max_hw_segments(q, q->max_hw_segments - 1);
                }

				if (dev->flags & ATA_DFLAG_NCQ) {
					int depth;
	
					depth = min(sdev->host->can_queue, ata_id_queue_depth(dev->id));
					depth = min(ATA_MAX_QUEUE - 1, depth);
					scsi_adjust_queue_depth(sdev, MSG_SIMPLE_TAG, depth);
				}	
        }

        return 0;       /* scsi layer doesn't check return value, sigh */
}


static int hsata_scsi_slave_cfg( struct scsi_device *sdev )
{
	   DPRINTK( "id=%d lun=%d ch=%d mfgr=%d\n", 
        	   sdev->id, sdev->lun, sdev->channel, sdev->manufacturer );
#if 1 
	return tangox_ata_scsi_slave_config( sdev );
#else
	return ata_scsi_slave_config( sdev );
#endif
}

static int hsata_std_bios_param( struct scsi_device *sdev, 
                                 struct block_device *bdev,
                                 sector_t capacity, int geom[] )
{
	DPRINTK( "id=%d lun=%d ch=%d mfgr=%d\n", 
        	 sdev->id, sdev->lun, sdev->channel, sdev->manufacturer );
	return ata_std_bios_param( sdev, bdev, capacity, geom );
}


static void hsata_port_disable( struct ata_port *ap )
{
	ata_port_disable( ap );
}
#ifdef  HSATA_VERBOSE
static int hsata_dump_dev_id( u16 *id, char *str )
{
	DPRINTK( "IDENTIFY DEVICE  "
	     "76:%04x 49:%04x 82:%04x 83:%04x 84:%04x 85:%04x 86:%04x 87:%04x 88:%04x\n",
	        id[76], id[49],
        	    id[82], id[83], id[84],
	            id[85], id[86], id[87],
	            id[88]);
	DPRINTK(  "HOST INITIATED PM=%s\n", 
	             ((id[76])&(1<<9)) ? "SUPPORTED" : "NOT-SUPPORTED" );
	DPRINTK(  "DEVICEE INITIATED PM=%s\n", 
	             ((id[78])&(1<<3)) ? "SUPPORTED" : "NOT-SUPPORTED" );

	DPRINTK( "PM FEATURE SET=%s\n", 
	             ((id[82])&(1<<3)) ? "SUPPORTED" : "NOT-SUPPORTED" );
	DPRINTK(  "APM FEATURE SET=%s\n", 
	             ((id[83])&(1<<3)) ? "SUPPORTED" : "NOT-SUPPORTED" );

	DPRINTK( "PM FEATURE SET=%s\n", 
	             ((id[85])&(1<<3)) ? "ENABLED" : "DISABLED" );
	DPRINTK( "APM FEATURE SET=%s\n", 
	             ((id[86])&(1<<3)) ? "ENABLED" : "DISABLED" );

	DPRINTK( "CURRENT APM VALUE=0x%04x", id[91] );

	if (str != NULL)
		return sprintf( str, "DEV: HPM=%d DPM=%d PMF=%d APMF=%d PMFE=%d APMFE=%d APMV=%d", 
		        ((id[76])&(1<<9)) ? 1 : 0,
			((id[78])&(1<<3)) ? 1 : 0,
        		((id[82])&(1<<3)) ? 1 : 0,
		        ((id[83])&(1<<3)) ? 1 : 0,
		        ((id[85])&(1<<3)) ? 1 : 0,
			((id[86])&(1<<3)) ? 1 : 0,
		        id[91]	);
	else
		return 0;
}
#endif
static void hsata_dev_config( struct ata_device *dev )
{
#ifdef HSATA_VERBOSE
	hsata_dump_dev_id( dev->id, NULL );
#endif
}

static void hsata_set_piomode( struct ata_port *ap, struct ata_device *dev )
{
}

static void hsata_set_dmamode( struct ata_port *ap, struct ata_device *dev )
{
}

static void hsata_tf_load( struct ata_port *ap, const struct ata_taskfile *tf )
{
#ifdef HSATA_VERBOSE
	unsigned int is_addr = tf->flags & ATA_TFLAG_ISADDR;
	DPRINTK("TFLAGS: %s %s %s %s\n", 
		(tf->flags & ATA_TFLAG_LBA48) ? "LBA48" : "NOT-LBA48", 
		(tf->flags & ATA_TFLAG_ISADDR) ? "ISADDR" : "", 
		(tf->flags & ATA_TFLAG_DEVICE) ? "DEVICE" : "", 
		(tf->flags & ATA_TFLAG_WRITE) ? "WRITE" : "READ"); 
	DPRINTK("CTL: %s %s %s %s\n", 
		(tf->ctl & ATA_HOB) ? "HOB" : "NOT-HOB", 
		(tf->ctl & ATA_SRST) ? "SRST" : "NOT-SRST", 
		(tf->ctl & ATA_NIEN) ? "NIEN" : "INTERRUPTS-ENABLED"); 
	if (is_addr && (tf->flags & ATA_TFLAG_LBA48)) {
		DPRINTK("hob: feat=0x%X nsect=0x%X, lba:0x%X 0x%X 0x%X\n",
			tf->hob_feature,
	  		tf->hob_nsect,
	  		tf->hob_lbal,
	  		tf->hob_lbam,
	  		tf->hob_lbah);
	}
	if (is_addr) {
		DPRINTK("feat=0x%X nsect=0x%X, lba:0x%X 0x%X 0x%X\n",
			tf->feature, tf->nsect, tf->lbal,
			tf->lbam, tf->lbah);
	}
	if (tf->flags & ATA_TFLAG_DEVICE)
		DPRINTK("ATA_TFLAG_DEVICE device=0x%X (%s)\n", tf->device, 
		       (tf->device & ATA_LBA) ? "LBA" : "CHS" );
	if (!is_addr) DPRINTK("is_addr=%d\n", is_addr);
#endif 

	ata_tf_load( ap, tf );
}

static void hsata_tf_read( struct ata_port *ap, struct ata_taskfile *tf )
{
#ifdef HSATA_VERBOSE
	DPRINTK("          nsect=0x%X, lba:0x%X 0x%X 0x%X device=0x%x\n",
		tf->nsect,tf->lbal,tf->lbam, tf->lbah, tf->device);
	if (tf->flags & ATA_TFLAG_LBA48) 
		DPRINTK("hob: feat=0x%X nsect=0x%X, lba:0x%X 0x%X 0x%X\n",
			tf->hob_feature,tf->hob_nsect,tf->hob_lbal,
			tf->hob_lbam,tf->hob_lbah);
#endif 

	ata_tf_read( ap, tf );
}

static u8 hsata_check_status( struct ata_port *ap )
{
#if 0
	/* wait bit 3 and 7 to be 0*/
	int val;
	val = readb(ap->ioaddr.status_addr);
	while(val & ((1<<3) | (1<<7) )){
        	val = readb(ap->ioaddr.status_addr);
	        printk("%s status=0x%x\n", __FUNCTION__, val);
	}
#endif
   return ata_check_status( ap );
}

static void hsata_exec_command_by_tag( struct ata_port *ap, const struct ata_taskfile *tf, u8 tag)
{
#ifdef HSATA_VERBOSE
	{
		volatile u32 val32;
		struct hsata_device *hsdev = HSDEV_FROM_AP(ap);
		switch (tf->command)
		{
			case ATA_CMD_CHK_POWER   	: DPRINTK("ATA_CMD_CHK_POWER - tag=%d\n", tag); break;
		       	case ATA_CMD_EDD       	: DPRINTK("ATA_CMD_EDD - tag=%d\n", tag); break;
			case ATA_CMD_FLUSH       	: DPRINTK("ATA_CMD_FLUSH - tag=%d\n", tag); break;
			case ATA_CMD_FLUSH_EXT   	: DPRINTK("ATA_CMD_FLUSH_EXT - tag=%d\n", tag); break;
			case ATA_CMD_ID_ATA      	: DPRINTK("ATA_CMD_ID_ATA - tag=%d\n", tag); break;
			case ATA_CMD_ID_ATAPI    	: DPRINTK("ATA_CMD_ID_ATAPI - tag=%d\n", tag); break;
			case ATA_CMD_READ        	: DPRINTK("ATA_CMD_READ - tag=%d\n", tag); break;
			case ATA_CMD_READ_EXT    	: DPRINTK("ATA_CMD_READ_EXT - tag=%d\n", tag); break;
			case ATA_CMD_WRITE       	: DPRINTK("ATA_CMD_WRITE - tag=%d\n", tag); break;
			case ATA_CMD_WRITE_EXT   	: DPRINTK("ATA_CMD_WRITE_EXT - tag=%d\n", tag); break;
			case ATA_CMD_PIO_READ    	: DPRINTK("ATA_CMD_PIO_READ - tag=%d\n", tag); break;
			case ATA_CMD_PIO_READ_EXT  	: DPRINTK("ATA_CMD_PIO_READ_EXT - tag=%d\n", tag); break;
			case ATA_CMD_PIO_WRITE 		: DPRINTK("ATA_CMD_PIO_WRITE - tag=%d\n", tag); break;
			case ATA_CMD_PIO_WRITE_EXT 	: DPRINTK("ATA_CMD_PIO_WRITE_EXT - tag=%d\n", tag); break;
			case ATA_CMD_SET_FEATURES  	: DPRINTK("ATA_CMD_SET_FEATURES - tag=%d\n", tag); break;
			case ATA_CMD_PACKET      	: DPRINTK("ATA_CMD_PACKET - tag=%d\n", tag); break;
			//case HSATA_CMD_QWRITE    	: DPRINTK("HSATA_CMD_QWRITE - tag=%d\n", tag); break;
			//case HSATA_CMD_QREAD     	: DPRINTK("HSATA_CMD_QREAD - tag=%d\n", tag); break;
			default						: DPRINTK("ATA_CMD_??? (0x%X)\n", tf->command); break;
		}
		val32 = readl( hsdev->membase + HSATA_SERROR_REG );
		DPRINTK("SERROR=0x%X\n", val32 );
		writel( val32, hsdev->membase + HSATA_SERROR_REG );
		val32 = readl( hsdev->membase + HSATA_INTPR_REG );
		DPRINTK( "INTPR=0x%x\n",    val32);
   	}
#endif 

	ata_exec_command( ap, tf );
}

static void hsata_exec_command(  struct ata_port *ap, const struct ata_taskfile *tf )
{
	hsata_exec_command_by_tag( ap, tf, 0 );
}

static void hsata_phy_reset( struct ata_port *ap )
{
  	struct hsata_device *hsdev = HSDEV_FROM_AP(ap);  
   	hsata_reset_port( ap );
	sata_phy_reset( ap );

	hsata_enable_interrupts( hsdev );
}



static u8   hsata_bmdma_status(   struct ata_port *ap )
{

#ifdef HSATA_VERBOSE
	int i;
	u32 val, regbase;
	struct hsata_device *hsdev = HSDEV_FROM_AP(ap);
	int port = tangox_controller(hsdev);
	printk("***********************dump all registers port=0x%x**********************\n", port);
	tango3_cdr_dump(0);
	printk("\n");
	tango3_cdr_dump(1);
	printk("\n");

	for (i =0; i< 40; i++ ){
		val = readl( tangox_sata_base[0] + i*4);
		printk("sata0 reg=0x%x =0x%x\n", (tangox_sata_base[0] + i*4), val);
	}
		printk("\n");

	for (i =0; i< 40; i++ ){
		val = readl( tangox_sata_base[1] + i*4);
		printk("sata1 reg=0x%x =0x%x\n", (tangox_sata_base[1] + i*4), val);
	}
		printk("\n");

	for (i =0; i< 16; i++ ){
		val = readl( tangox_ctl_base[0] + i*4);
		printk("ctl0 reg=0x%x =0x%x\n", (tangox_ctl_base[0] + i*4), val);
	}
		printk("\n");

	for (i =0; i< 16; i++ ){
		val = readl( tangox_ctl_base[1] + i*4);
		printk("ctl1 reg=0x%x =0x%x\n", (tangox_ctl_base[1] + i*4), val);
	}
		printk("\n");
	
	val = gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE);
	printk("route(0x%x)=0x%x\n",REG_BASE_host_interface + SBOX_ROUTE, val);
	val = gbus_read_reg32(REG_BASE_host_interface + SBOX_ROUTE2);
	printk("route(0x%x)=0x%x\n",REG_BASE_host_interface + SBOX_ROUTE2, val);
	printk("\n");

	regbase =0x2b000;
    printk("MBUS registers 0x%x: %08lx %08lx %08lx %08lx\n", regbase,
               gbus_read_reg32(regbase + MIF_add_offset),
               gbus_read_reg32(regbase + MIF_cnt_offset),
               gbus_read_reg32(regbase + MIF_add2_skip_offset),
               gbus_read_reg32(regbase + MIF_cmd_offset));


	regbase =0x2b080;
    printk("MBUS registers 0x%x: %08lx %08lx %08lx %08lx\n",regbase,
               gbus_read_reg32(regbase + MIF_add_offset),
               gbus_read_reg32(regbase + MIF_cnt_offset),
               gbus_read_reg32(regbase + MIF_add2_skip_offset),
               gbus_read_reg32(regbase + MIF_cmd_offset));


	regbase =0x2b100;
    printk("MBUS registers 0x%x: %08lx %08lx %08lx %08lx\n",regbase,
               gbus_read_reg32(regbase + MIF_add_offset),
               gbus_read_reg32(regbase + MIF_cnt_offset),
               gbus_read_reg32(regbase + MIF_add2_skip_offset),
               gbus_read_reg32(regbase + MIF_cmd_offset));

	regbase =0x2b140;
    printk("MBUS registers 0x%x: %08lx %08lx %08lx %08lx\n",regbase,
               gbus_read_reg32(regbase + MIF_add_offset),
               gbus_read_reg32(regbase + MIF_cnt_offset),
               gbus_read_reg32(regbase + MIF_add2_skip_offset),
               gbus_read_reg32(regbase + MIF_cmd_offset));
#endif
	return 0;
}

static void hsata_bmdma_stop( struct ata_queued_cmd *qc )
{
}

static void hsata_bmdma_setup_noexec( struct ata_queued_cmd *qc )
{
#ifdef HSATA_TANGOX_DMA
	struct ata_port *ap = qc->ap;
	struct scatterlist *sg = qc->__sg; 
	int dir;
	int nents, controller;
	u32 dma_len =  0;
	u32 dma_addr = 0;
	struct hsata_device *hsdev = HSDEV_FROM_AP(ap);

	controller = tangox_controller(hsdev);
	dir = qc->dma_dir;
	/* try to setup dma channel */
	if(g_mbus_reg[controller] == 0) { 
		if (em86xx_mbus_alloc_dma(tangox_sbox[controller], 
			(dir == DMA_FROM_DEVICE) ? 1 : 0, 
			&g_mbus_reg[controller], NULL)) {
			printk("fail to alloc dma on sbox 0x%x port=0x%x\n", 
				tangox_sbox[controller], controller);
			return;
		}
	}

	nents = dma_map_sg((struct device *)qc->dev, sg, qc->n_elem, dir);
	if(!nents) {
		printk("dma map sg failed, please check\n");
		return;
	}

#ifdef TANGOX_BOUNCE_BUF
	if(nents==1){
		dma_len = sg_dma_len(sg);
		dma_addr= sg_dma_address(sg);
	} else if (nents > 1){
		unsigned long tmp_buf = 0;
		struct hsata_device *hsdev = HSDEV_FROM_AP(ap);
		int controller = tangox_controller(hsdev);
		if(controller == 1){
			tmp_buf  =  KSEG1ADDR(bounce_buf0);	
		    dma_addr = tangox_dma_address(CPHYSADDR(bounce_buf0));
		} else {
			tmp_buf  =  KSEG1ADDR(bounce_buf1);	
		    dma_addr = tangox_dma_address(CPHYSADDR(bounce_buf1));
		}

		if (dir==DMA_TO_DEVICE) {
			ata_for_each_sg(sg, qc) {
				unsigned long len, addr;
               	len = sg_dma_len(sg);
               	addr = (unsigned long)page_address(sg->page);
				dma_cache_wback(addr, len);
#ifndef TANGOX_MBUS_MEMCPY
				memcpy((void *)tmp_buf, (void *)addr, len);
				mb();
#else
				mbus_memcpy(g_mbus_reg[controller], tangox_dma_address(CPHYSADDR(tmp_buf)), 
							tangox_dma_address(CPHYSADDR(addr)), len);

#endif
				tmp_buf += len;
				dma_len += len;
			}
		} else
			dma_len = get_dma_len(qc);
			 
	}

#ifdef HSATA_VERBOSE
	tango3_cdr_dump(controller);

	printk("setup_dma address=0x%x len=0x%x n_elem=0x%x nents=0x%x dir=0x%x dma_len=0x%x g_mbus_reg[%d]=0x%x\n",
		 dma_addr,dma_len,qc->n_elem, nents, dir, dma_len, controller,g_mbus_reg[controller]);

#endif
	/* write length in dword */
    writel((dma_len/4), tangox_ctl_base[controller] + 0x04);

	if (em86xx_mbus_setup_dma(g_mbus_reg[controller], dma_addr,dma_len, NULL, qc, 1)) {
		printk("fail to setup dma\n");
		em86xx_mbus_free_dma(g_mbus_reg[controller], tangox_sbox[controller]);
		return;
	}

#else /*not bounce buf*/
	if (dir==DMA_TO_DEVICE) {
		ata_for_each_sg(sg, qc) {
			unsigned long len, addr;
           	len = sg_dma_len(sg);
           	addr = (unsigned long) page_address(sg->page);
			dma_cache_wback(addr, len);
			dma_len += len;
		}
	} else  
			dma_len = get_dma_len(qc);

#ifdef HSATA_VERBOSE
	tango3_cdr_dump(controller);

	printk("total len=0x%x n_elem=0x%x nents=0x%x dir=0x%x block_len=0x%x dma_len/8k=0x%x\n",
		 dma_len,qc->n_elem, nents, dir, dma_len/4, dma_len/(8*1024));
#endif

	/* write length in dword */
    writel((dma_len/4),(void *)(tangox_ctl_base[controller] + 0x04));

	/* get the first one*/
	sg = qc->__sg; 
	g_next_sg[controller] = 1;
	dma_len = sg_dma_len(sg);
	dma_addr= sg_dma_address(sg); 
#ifdef  HSATA_VERBOSE
	printk("setup_dma address=0x%x len=0x%x n_elem=0x%x nents=0x%x dir=0x%x block_len=0x%x dma_len/8k=0x%x\n",
		 dma_addr,dma_len,qc->n_elem, nents, dir, dma_len/4, dma_len/(8*1024));
#endif

	if (em86xx_mbus_setup_dma(g_mbus_reg[controller], dma_addr,dma_len,
		(nents == 1)? NULL : tangox_mbus_intr, qc, (nents == 1) ? 1 : 0)) {
		printk("fail to setup dma\n");
		em86xx_mbus_free_dma(g_mbus_reg[controller], tangox_sbox[controller]);
		return;
	}
#endif /* BOUNCE_BUF */
#endif /* HSATA_TANGOX_DMA */
}

static void hsata_bmdma_setup( struct ata_queued_cmd *qc )
{
	hsata_bmdma_setup_noexec( qc );
	hsata_exec_command( qc->ap, &qc->tf );
}

static void hsata_bmdma_start( struct ata_queued_cmd *qc )
{
#ifdef HSATA_TANGOX_DMA
	struct hsata_device *hsdev = HSDEV_FROM_QC(qc);
	int dir = qc->dma_dir;
	int controller = tangox_controller(hsdev);

	/* wait for dma cleared */
	while(readl((void *)(tangox_ctl_base[controller] + 0x08)));

	/* set DBTSR */
   	writel(HSATA_DMA_DBTSR, (void *)(hsdev->membase + HSATA_DBTSR_REG));
	if (dir == DMA_TO_DEVICE) { 
	   	/* Tx Burst length */
	    writel(TANGOX_BURST_LENGTH_TX, (void *)(tangox_ctl_base[controller] + 0x00));
		/* Enable Tx*/
      	writel( HSATA_DMACR_TX_EN, (void *)(hsdev->membase + HSATA_DMACR_REG)); 
	}
	else {
	   	/* Rx Burst length */
	   	writel( BURST_LENGTH_RX,(void *)(tangox_ctl_base[controller] + 0x00));
		/* Enable Rx*/
		writel( HSATA_DMACR_RX_EN,(void *)(hsdev->membase + HSATA_DMACR_REG)); 
	}
#endif /* HSATA_TANGOX_DMA */
}


static void hsata_qc_prep_by_tag( struct ata_queued_cmd *qc, u8 tag )
{
#ifdef HSATA_TANGOX_DMA
	int dir;
#endif

	if (!(qc->flags & ATA_QCFLAG_DMAMAP))
	{
		DPRINTK( "NO DMA - exiting\n" );
		return;
	}

#ifndef HSATA_TANGOX_DMA
	DPRINTK(  "QC PREP  id=%d n_elem=%d\n", 
                qc->ap->print_id, 
                qc->n_elem );

#else 
	dir = qc->dma_dir;

	DPRINTK(  "QC PREP id=%d dma dir=%s n_elem=%d\n", 
               qc->ap->print_id, 
               (dir == DMA_FROM_DEVICE) ? "FROM_DEVICE" : "TO_DEVICE",
                qc->n_elem );
#endif

}

static void hsata_qc_prep( struct ata_queued_cmd *qc )
{
	hsata_qc_prep_by_tag( qc, 0 );
}

static unsigned int hsata_qc_issue_prot( struct ata_queued_cmd *qc )
{

#ifdef HSATA_VERBOSE
	DPRINTK( "id=%d\n",  qc->ap->print_id );
	switch (qc->tf.protocol) {
      	case ATA_PROT_DMA: DPRINTK("ATA_PROT_DMA\n"); break;
      	case ATA_PROT_PIO: DPRINTK("ATA_PROT_PIO\n"); break;
	}
#endif 
	return ata_qc_issue_prot( qc );
}

static struct scsi_host_template hsata_sht = {
	.module              = THIS_MODULE,
	.name                = DRV_NAME0,
	.ioctl               = hsata_scsi_ioctl,
	.queuecommand        = hsata_scsi_queuecmd,
	.can_queue           = ATA_DEF_QUEUE,
	.this_id             = ATA_SHT_THIS_ID,
	.sg_tablesize        = LIBATA_MAX_PRD,
	.max_sectors         = ATA_MAX_SECTORS,
	.cmd_per_lun         = ATA_SHT_CMD_PER_LUN,
	.emulated            = ATA_SHT_EMULATED,
	.use_clustering      = ATA_SHT_USE_CLUSTERING,
	.proc_name           = DRV_NAME0,
	.dma_boundary        = ATA_DMA_BOUNDARY,
	.slave_configure     = hsata_scsi_slave_cfg,
	.slave_destroy		 = ata_scsi_slave_destroy,
	.bios_param          = hsata_std_bios_param,
};

static const struct ata_port_operations hsata_ops_0 = {
	.port_disable     = hsata_port_disable,
	.dev_config       = hsata_dev_config,
	.set_piomode      = hsata_set_piomode,
	.set_dmamode      = hsata_set_dmamode,
	.tf_load          = hsata_tf_load,
	.tf_read          = hsata_tf_read,
	.check_status     = hsata_check_status,
	.exec_command     = hsata_exec_command,
	.dev_select       = ata_std_dev_select,
	.phy_reset        = hsata_phy_reset,
	.bmdma_setup      = hsata_bmdma_setup,
	.bmdma_start      = hsata_bmdma_start,
	.bmdma_stop       = hsata_bmdma_stop,
	.bmdma_status     = hsata_bmdma_status,
	.qc_prep          = hsata_qc_prep,
	.qc_issue         = hsata_qc_issue_prot,
	.data_xfer		  = ata_data_xfer,
	.freeze			  = ata_bmdma_freeze,
	.thaw			  = ata_bmdma_thaw,
	.error_handler	  = ata_bmdma_error_handler,
	.irq_clear        = hsata_irq_clear,
	.irq_on			  = ata_irq_on,
	.irq_ack		  = ata_irq_ack,
	.scr_read         = hsata_scr_read,
	.scr_write        = hsata_scr_write,
	.port_start       = hsata_port_start_0,
};
static const struct ata_port_operations hsata_ops_1 = {
	.port_disable     = hsata_port_disable,
	.dev_config       = hsata_dev_config,
	.set_piomode      = hsata_set_piomode,
	.set_dmamode      = hsata_set_dmamode,
	.tf_load          = hsata_tf_load,
	.tf_read          = hsata_tf_read,
	.check_status     = hsata_check_status,
	.exec_command     = hsata_exec_command,
	.dev_select       = ata_std_dev_select,
	.phy_reset        = hsata_phy_reset,
	.bmdma_setup      = hsata_bmdma_setup,
	.bmdma_start      = hsata_bmdma_start,
	.bmdma_stop       = hsata_bmdma_stop,
	.bmdma_status     = hsata_bmdma_status,
	.qc_prep          = hsata_qc_prep,
	.qc_issue         = hsata_qc_issue_prot,
	.data_xfer		  = ata_data_xfer,
	.freeze			  = ata_bmdma_freeze,
	.thaw			  = ata_bmdma_thaw,
	.error_handler	  = ata_bmdma_error_handler,
	.irq_clear        = hsata_irq_clear,
	.irq_on			  = ata_irq_on,
	.irq_ack		  = ata_irq_ack,
	.scr_read         = hsata_scr_read,
	.scr_write        = hsata_scr_write,
	.port_start       = hsata_port_start_1,
};

static struct ata_port_info hsata_port_info_0 = 
{
	.sht        = &hsata_sht,
	.flags		= ATA_FLAG_SATA |         
	              ATA_FLAG_NO_LEGACY |    /* no legacy mode check */
	              ATA_FLAG_SRST |         /* use ATA SRST, not E.D.D. */
	              ATA_FLAG_MMIO |         /* use MMIO, not PortIO */
#ifndef HSATA_TANGOX_DMA
        	      ATA_DFLAG_PIO |         /* set to NOT use DMA */
#endif
				  ATA_DFLAG_LBA48 |        /* READ/WRITE EXT support */
	              0x0,                  
	.pio_mask   = 0x1f, /* pio0-4    - IDENTIFY DEVICE word 63 */
#ifdef HSATA_TANGOX_DMA
	.mwdma_mask = 0x07, /* mwdma0-2  - IDENTIFY DEVICE word 64 */
	.udma_mask  = 0x7f, /* udma0-6   - IDENTIFY DEVICE word 88 */
#else
	.mwdma_mask = 0x00, 
	.udma_mask  = 0x00,
#endif
	.port_ops   = &hsata_ops_0,
};

static struct ata_port_info hsata_port_info_1 = 
{
	.sht        = &hsata_sht,
	.flags		= ATA_FLAG_SATA |         
	              ATA_FLAG_NO_LEGACY |    /* no legacy mode check */
	              ATA_FLAG_SRST |         /* use ATA SRST, not E.D.D. */
	              ATA_FLAG_MMIO |         /* use MMIO, not PortIO */
#ifndef HSATA_TANGOX_DMA
        	      ATA_DFLAG_PIO |         /* set to NOT use DMA */
#endif
				  ATA_DFLAG_LBA48 |        /* READ/WRITE EXT support */
	              0x0,                  
	.pio_mask   = 0x1f, /* pio0-4    - IDENTIFY DEVICE word 63 */
#ifdef HSATA_TANGOX_DMA
	.mwdma_mask = 0x07, /* mwdma0-2  - IDENTIFY DEVICE word 64 */
	.udma_mask  = 0x7f, /* udma0-6   - IDENTIFY DEVICE word 88 */
#else
	.mwdma_mask = 0x00, 
	.udma_mask  = 0x00,
#endif
	.port_ops   = &hsata_ops_1,
};

int hsata_probe( struct device *adev, int controller )
{
	int status = 0;
	int num_ports = 1;
	struct ata_host *host;
	struct ata_port *ap;
	struct hsata_device  *hsdev;
	const struct ata_port_info *ppi[1]; 

	if(controller == 0)
		ppi[0] = &hsata_port_info_0;
	else
		ppi[0] = &hsata_port_info_1;

	if (adev) {
		printk( "k_name=%s driver=%s\n", 
		        (adev->kobj.k_name)?adev->kobj.k_name:"NULL",
		        (adev->driver)?
		        ((adev->driver->name)?(adev->driver->name):"NO NAME") :
              		"NO DRIVER" );
	} else {
		printk( "NULL dev\n" );
		return -ENODEV;
	}

	/*
	 * Device data struct
	 */
	hsdev = kmalloc( sizeof(*hsdev), GFP_KERNEL );
	if (hsdev == NULL) {
		status = -ENOMEM;
		goto CLEANUP;
	}

	host = ata_host_alloc_pinfo(adev, ppi, num_ports);
	if (!host){
		printk("cannot alloc host port info.\n");
		return -ENOMEM;
	}

	memset( hsdev, 0, sizeof(*hsdev) );
	hsdev->magic = hsata_magic++;
	hsdev->adev = adev;
	hsdev->host = host;
	hsdev->ppi  = (struct ata_port_info *)ppi[0];

	hsdev->membase = tangox_sata_base[controller];
	DPRINTK( "SATA membase (0x%x)\n", hsdev->membase );
	host->dev = adev;
	host->private_data = hsdev;

	ap = host->ports[0];
	hsata_setup_port( &ap->ioaddr, hsdev->membase );

	/* Init the host controller*/
	hsata_host_init( hsdev );
	DPRINTK( "hsata_host_init done\n" );
	/*
	 * Interrupt management for SATA interrupts is done by the libata layer 
	 * (see ata_device_add).  See hsata_port_start() for init of DMAC
	 * interrupt.
	 */
	hsata_inum = 0;
	DPRINTK( "start ata_host_activate , status=0x%x irq=0x%x\n", status, tangox_sata_irq[controller] );

	status =  ata_host_activate(hsdev->host, tangox_sata_irq[controller], 
							hsata_isr, IRQF_DISABLED, &hsata_sht);

	DPRINTK( "ata_host_activate done, status=0x%x\n", status );

	/* init hotplug tasklet */
	hsata_enable_interrupts( hsdev );
	tasklet_init(&hsdev->hotplug_tasklet, hsata_link_check,
     (unsigned long)adev);

CLEANUP:
	if (status)	{
		dev_set_drvdata( adev, NULL );  /* clear private data ptr */

		if (hsdev && hsdev->adev)		/* decr device ref count */
			put_device( hsdev->adev);

		if (hsdev) 
			kfree( hsdev );
	}
	DPRINTK("DONE - %s - nports=%d\n", status?"ERROR":"OK", num_ports );
	return status;
}


int hsata_probe0( struct device *adev )
{
#ifdef TANGOX_BOUNCE_BUF
	if(!bounce_buf0)
		bounce_buf0 = (unsigned char *)__get_free_pages(GFP_KERNEL, DMA_BOUNCE_BUF_ORDER);
#endif

	return  hsata_probe(adev, 0);
}

int hsata_probe1( struct device *adev )
{
#ifdef TANGOX_BOUNCE_BUF
	if(!bounce_buf1)
		bounce_buf1 = (unsigned char *)__get_free_pages(GFP_KERNEL, DMA_BOUNCE_BUF_ORDER);
#endif

	return hsata_probe(adev, 1);
}

static int hsata_remove( struct device *adev, int controller )
{
	struct hsata_device  *hsdev;
	struct ata_host  *host;
	struct ata_port *ap;

	DPRINTK("k_name=%s driver=%s\n", 
        	(adev->kobj.k_name)?adev->kobj.k_name:"NULL",
	        (adev->driver)?
        	((adev->driver->name)?(adev->driver->name):"NO NAME") :
	        "NO DRIVER" );

	host = dev_get_drvdata( adev );
	hsdev = HSDEV_FROM_HOST_SET(host);
	ap =  host->ports[0];

	/* Wait and kill tasklet */
	tasklet_kill(&hsdev->hotplug_tasklet);

	hsata_disable_interrupts(hsdev);
	ata_port_disable(ap);

#ifdef HSATA_TANGOX_DMA
	free_irq(tangox_sata_dma_irq[controller], host );
#ifdef TANGOX_BOUNCE_BUF
	if(bounce_buf0)
		free_pages((unsigned long)bounce_buf0, DMA_BOUNCE_BUF_ORDER);
	if(bounce_buf1)
		free_pages((unsigned long)bounce_buf1, DMA_BOUNCE_BUF_ORDER);
#endif
#endif
	ata_host_detach(host);

	if (hsdev && hsdev->adev)
		put_device( hsdev->adev);

	if (hsdev) 
	      kfree( hsdev );

	return 0;
}

static int hsata_remove0( struct device *adev )
{
	hsata_remove( adev, 0 );
   	return 0;
}

static int hsata_remove1( struct device *adev )
{
	hsata_remove( adev, 1 );
	return 0;
}

static struct device_driver  hsata_driver0 = {
	.name    =      (char *)DRV_NAME0,
	.bus     =      &platform_bus_type,
	.probe   = hsata_probe0,
	.remove  = hsata_remove0,
};

static struct device_driver  hsata_driver1 = {
	.name    =      (char *)DRV_NAME1,
	.bus     =      &platform_bus_type,
	.probe   = hsata_probe1,
	.remove  = hsata_remove1,
};

static void tangox_sata_release_dev(struct device * dev)
{
        dev->parent = NULL;
}

static struct platform_device tangox_sata_device0 = {
        .name           = (char *)DRV_NAME0,
        .id             = -1,
        .dev = {
                .release                = tangox_sata_release_dev,
        },
       .num_resources  = 0,
       .resource       = 0,

};

static struct platform_device tangox_sata_device1 = {
        .name           = (char *)DRV_NAME1,
        .id             = -1,
        .dev = {
                .release                = tangox_sata_release_dev,
        },
       .num_resources  = 0,
       .resource       = 0,

};

static struct platform_device *tangox_platform_devices0[] __initdata = {
        &tangox_sata_device0,
};

static struct platform_device *tangox_platform_devices1[] __initdata = {
        &tangox_sata_device1,
};

static int __init tangox_hsata_module_init( void )
{
   int status = 0;
  
	/* Device 0 registration */
	status = platform_add_devices(tangox_platform_devices0, ARRAY_SIZE(tangox_platform_devices0));
	if(status){
		printk("Failed to register device 0.\n");
		return status;
	}
	/* Driver 0 registration*/
   	status = driver_register( &hsata_driver0 );
	if (status){
		printk("Failed to register driver 0.\n");
		return status;
	}
	/* Device 1 registration */
	status = platform_add_devices(tangox_platform_devices1, ARRAY_SIZE(tangox_platform_devices1));
	if(status){
		printk("Failed to register device 1.\n");
		return status;
	}
	/* Driver 1 registration*/
	status = driver_register( &hsata_driver1 );
	if (status){
		printk("Failed to register driver 1.\n");
		return status;
	}
	/*TODO: clean up */
   	return status;
}

static void __exit tangox_hsata_module_cleanup(void)
{
	driver_unregister( &hsata_driver0 ); 
	platform_device_unregister(&tangox_sata_device0);
	driver_unregister( &hsata_driver1 ); 
	platform_device_unregister(&tangox_sata_device1);
}

module_init( tangox_hsata_module_init );
module_exit( tangox_hsata_module_cleanup );
