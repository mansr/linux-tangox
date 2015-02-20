/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include "tango3_vusbhs.h"

#ifndef TANGO3_UDC_H
#define TANGO3_UDC_H

#include <asm/addrspace.h>

#if defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3_gbus.h>
#else
#error "Unknown architecture"
#endif

#define MAX_EP_TR_DESCRS                    (32)
#define USB_MAX_ENDPOINTS                   (16)
#define USB_MAX_CTRL_PAYLOAD                (64)
#define MEM32_ALIGN(n)                   ((n) + (-(n) & 31))
#define MEM1024_ALIGN(n)                 ((n) + (-(n) & 1023))
#define MEM2048_ALIGN(n)                 ((n) + (-(n) & 2047))

/* main registers, BAR0 + 0x0000 */
struct tango3_id_regs {
	// offset 0x0000
	u32		id;
	u32		hw_general;
	u32		hw_host;
	u32		hw_device;
	u32		hw_txbuf;
	u32		hw_rxbuf;
	u32		hw_tt_txbuf;
	u32		hw_tt_rxbuf;
} __attribute__ ((packed));

/*offset base + 0x80h 3.5 Device/Host Timer Registers (Non-EHCI)*/
struct tango3_timer_regs {
	u32		gptimer0ld;
	u32		gptimer0ctrl;
	u32		gptimer1ld;
	u32		gptimer1ctrl;
} __attribute__ ((packed));


/*offset 0x0100h 3.4 Device/Host Capability Registers*/
struct tango3_cap_regs {
	u8		caplength;
	u8		reserved0;
	u16		hciversion;
	u32		hcsparams;
	u32		hccparams;
	u32		reserved[5]; 
	u16		dciversion;
	u16		reserved1;
	u32		dccparams;
} __attribute__ ((packed));


/*offset base + 0x0140h 3.6 Device/Host Operational Registers*/
struct tango3_op_regs {
	u32		usbcmd;
	u32		usbsts;
	u32		usbintr;
	u32		frindex;
	u32		ctrldssegment;
	u32		deviceaddr;	/*host periodiclistbase; */  
	u32		eplistaddr;	/*host asynclistaddr; */  
	u32		ttctrl;
	u32		burstsize;
	u32		txfilltuning;
	u32		txttfilltuning;
	u32		reserved0;
	u32		ulpi_viewport;
	u32		reserved1;
	u32		ep_nak;
	u32		ep_nak_en;
	u32		config_flag;
	u32		port_sc[8];
	u32		otg_sc;
	u32		usb_mode;
	u32		ep_setupstat;
	u32		ep_prime;
	u32		ep_flush;
	u32		ep_stat;
	u32		ep_complete;
	u32		ep_ctrl[16];
} __attribute__ ((packed));

struct tango3_ep {
	struct usb_ep				ep;
	struct tango3_op_regs		__iomem *regs;
	dma_addr_t				td_dma;	/* of dummy */
	struct tango3_otg			*dev;
	unsigned long				irqs;

	/* analogous to a host-side qh */
	struct list_head			queue;
	const struct usb_endpoint_descriptor	*desc;
	unsigned				num : 4,
						//fifo_size : 12,
						stopped : 1,
						is_in : 1,
						is_iso : 1,
						responded : 1;
						
};

struct tango3_request {
	struct usb_request	req;
	dma_addr_t			td_dma;
	struct list_head	queue;
	unsigned			mapped : 1,
						valid : 1;
};


struct ep_qh {
	u32   max_pktlen;         /* Bits 16..26 Bit 15 is Interrupt On Setup */
	u32   curr_dtdptr;        /* Current dTD Pointer */
	u32   next_dtdptr;        /* Next dTD Pointer */
	u32   size_iocintsts;     /* Total bytes (16..30), IOC (15), INT (8), STS (0-7) */
	u32   buff_ptr0;          /* Buffer pointer Page 0 (12-31) */
	u32   buff_ptr1;          /* Buffer pointer Page 1 (12-31) */
	u32   buff_ptr2;          /* Buffer pointer Page 2 (12-31) */
	u32   buff_ptr3;          /* Buffer pointer Page 3 (12-31) */
	u32   buff_ptr4;          /* Buffer pointer Page 4 (12-31) */
	u32   reserve1;
	unsigned char setup_buff[8];
	u32   reserve2[4];
}__attribute__ ((aligned (32)));
 

struct dtd {
	u32     next_link_ptr; 
	u32     size_ioc_sts;     /* total bytes (16-30),
                                   ** IOC (15), Status (0-7)
                                   */
	u32     buff_ptr0;        /* Buffer pointer Page 0 */
	u32     buff_ptr1;        /* Buffer pointer Page 1 */
	u32     buff_ptr2;        /* Buffer pointer Page 2 */
	u32     buff_ptr3;        /* Buffer pointer Page 3 */
	u32     buff_ptr4;        /* Buffer pointer Page 4 */
	int	ddma;
	struct list_head	dtd_list;
}__attribute__ ((aligned (32)));

struct tango3_otg {
    /* each pci device provides one gadget, several endpoints */
    struct usb_gadget               gadget;
    spinlock_t                      lock;
    struct tango3_ep               ep [7];
    struct usb_gadget_driver        *driver;
    unsigned                        enabled : 1,
                                    protocol_stall : 1,
                                    softconnect : 1,
                                    got_irq : 1,
                                    region : 1;
    u16                             chiprev;
	u8                           	max_eps;        /* Max endpoints supported by this device */
   	u16                             usb_state;
   	int                         	bus_resetting; 
 
    /* memory pools */
    struct dma_pool        		*qh_pool;       /* qh*/
    struct dma_pool        		*dtd_pool;      

   	struct ep_qh	   		*ep_qhptr;	/* Endpoint Queue head*/
   	struct dtd         		*dtd_ptr;       /* Device transfer descriptor pool address */
   	struct dtd         		*dtd_head;      
   	struct dtd         		*dtd_tail;     


    /* pci state used to access those endpoints */
    struct platform_device			*pdev;
    struct tango3_cap_regs         __iomem *cap_regs;
    struct tango3_op_regs          __iomem *regs;

    struct pci_pool                 *requests;
    // statistics...
};


static void set_halt (struct tango3_ep *ep)
{
        struct tango3_otg           *dev = 0;

        if (!ep->dev->driver || 
		ep->dev->gadget.speed == USB_SPEED_UNKNOWN)
		    return;

        dev =  ep->dev;
		dev->regs->ep_ctrl[ep->num] &= 
		~(EHCI_EPCTRL_TX_ENABLE | EHCI_EPCTRL_RX_ENABLE);
}

static void clear_halt (struct tango3_ep *ep)
{
        struct tango3_otg           *dev = 0;

        if (!ep->dev->driver || 
		ep->dev->gadget.speed == USB_SPEED_UNKNOWN)
			return;

        dev =  ep->dev;
		dev->regs->ep_ctrl[ep->num] |= 
		(EHCI_EPCTRL_TX_ENABLE | EHCI_EPCTRL_RX_ENABLE);
}

/*
 * dma address translation
 */
static inline unsigned long PHYSADDR(void *addr) 
{
		return tangox_dma_address(CPHYSADDR((unsigned long)addr));
}

static inline int tangox_valid_dma_addr(unsigned long mapaddr)
{
	extern unsigned long phy_remap, max_remap_size;
	if ((mapaddr >= phy_remap) && (mapaddr < (phy_remap + max_remap_size)))
		return 1;
	else
		return 0;
}
#endif
