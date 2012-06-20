/*********************************************************************
 Sigma Designs, Inc. 
 2001-2011

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#if defined(CONFIG_TANGO3)
#include <asm/tango3/platform_dev.h>
#elif defined(CONFIG_TANGO4)
#include <asm/tango4/platform_dev.h>
#endif
#include "tangox_udc.h"

#define	DRIVER_DESC		"TANGOX 8652/867X/868X USB Peripheral Controller"
#define	DRIVER_VERSION	"Mar. 10, 2009"
#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)
#define	DIR_STRING(bAddress) (((bAddress) & USB_DIR_IN) ? "in" : "out")

#undef TANGOX_SD_DEBUG
#ifdef TANGOX_SD_DEBUG
#undef DEBUG
#define DEBUG(dev,fmt,args...) \
	printk(dev ,fmt , ## args)
#else
#define DEBUG(dev,fmt,args...) \
	do { } while (0)
#endif

/* for 867x/868x, it has two controllers */
static int controller = 0;		// 0 to 1
module_param (controller, int, S_IRUGO);
MODULE_PARM_DESC (controller, "TANGOX Controller 0 or 1, default is 0");

static const char driver_desc [] = DRIVER_DESC;
static struct tangox_otg	*the_controller;
static const struct usb_ep_ops tangox_ep_ops;
static LIST_HEAD(dtd_free_list);
static const char *const ep_name [] = {
	"ep0","ep-a", "ep-b", "ep-c", "ep-d",
};

#if defined(CONFIG_USB_GADGET_DEBUG_FILES) || defined (TANGOX_SD_DEBUG)
static char *type_string (u8 bmAttributes)
{
	switch ((bmAttributes) & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_BULK:	return "bulk";
	case USB_ENDPOINT_XFER_ISOC:	return "iso";
	case USB_ENDPOINT_XFER_INT:	return "intr";
	};
	return "control";
}
#endif

#ifdef TANGOX_SD_DEBUG
static void dump_registers(struct tangox_otg *dev, int yes)
{
	if (yes) {
        printk ("                 usbcmd       0x%08x  usbsts        0x%08x  usbintr      0x%08x\n \
                 frindex      0x%08x  ctrldssegment 0x%08x  deviceaddr   0x%08x\n \
                 eplistaddr   0x%08x  ttctrl        0x%08x  burstsize    0x%08x\n \
                 txfilltuning 0x%08x  ulpi_viewport 0x%08x  ep_nak       0x%08x\n \
                 ep_nak_en    0x%08x  config_flag   0x%08x  port_sc[0]   0x%08x\n \
                 otg_sc       0x%08x  usb_mode      0x%08x  ep_setupstat 0x%08x\n \
                 ep_prime     0x%08x  ep_flush      0x%08x  ep_stat      0x%08x\n \
                 ep_complete  0x%08x  ep_ctrl[0]    0x%08x  ep_ctrl[1]   0x%08x\n \
				 ep_ctrl[2]   0x%08x  ep_ctrl[3]    0x%08x  ep_ctrl[4]   0x%08x\n",
                readl (&dev->regs->usbcmd),readl (&dev->regs->usbsts),readl (&dev->regs->usbintr),
                readl (&dev->regs->frindex),readl (&dev->regs->ctrldssegment),readl (&dev->regs->deviceaddr),
                readl (&dev->regs->eplistaddr),readl (&dev->regs->ttctrl),readl (&dev->regs->burstsize),
                readl (&dev->regs->txfilltuning),readl (&dev->regs->ulpi_viewport),readl (&dev->regs->ep_nak),
                readl (&dev->regs->ep_nak_en),readl (&dev->regs->config_flag),readl (&dev->regs->port_sc[0]),
                readl (&dev->regs->otg_sc),readl (&dev->regs->usb_mode),readl (&dev->regs->ep_setupstat),
                readl (&dev->regs->ep_prime),readl (&dev->regs->ep_flush),readl (&dev->regs->ep_stat),
                readl (&dev->regs->ep_complete),readl (&dev->regs->ep_ctrl[0]),readl (&dev->regs->ep_ctrl[1]),
                readl (&dev->regs->ep_ctrl[2]),readl (&dev->regs->ep_ctrl[3]),readl (&dev->regs->ep_ctrl[4]));
	}
}


static void dump_dtd(struct dtd * dtd)
{
	printk("	dtd=0x%x\n", (u32)dtd);
	printk("	dtd->next_link_ptr=0x%x\n", dtd->next_link_ptr);
	printk("	dtd->size_ioc_sts=0x%x\n", dtd->size_ioc_sts);
	printk("	dtd->buff_ptr0=0x%x\n", dtd->buff_ptr0);
	printk("	dtd->buff_ptr1=0x%x\n", dtd->buff_ptr1);
	printk("	dtd->buff_ptr2=0x%x\n", dtd->buff_ptr2);
	printk("	dtd->buff_ptr3=0x%x\n", dtd->buff_ptr3);
	printk("	dtd->buff_ptr4=0x%x\n", dtd->buff_ptr4);
}
static void dump_dqh(struct tangox_otg *dev,struct ep_qh * dqh, int loc)
{
	printk(".....................dumping dqh........at loc=0x%x\n", loc);
	printk("dqh=0x%x\n", (u32)dqh);
	printk("dqh->max_pktlen=0x%x\n", dqh->max_pktlen);
	printk("dqh->curr_dtdptr=0x%x\n", dqh->curr_dtdptr);
	if (dqh->curr_dtdptr != VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE) {
		dump_dtd(get_dtd_from_dma_addr(dev, (u32)dqh->curr_dtdptr));
	}
	printk("dqh->next_dtdptr=0x%x\n", dqh->next_dtdptr);
	if (dqh->next_dtdptr != VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE) {
		dump_dtd(get_dtd_from_dma_addr(dev, (u32)dqh->next_dtdptr));
	}
	printk("dqh->size_iocintsts=0x%x\n", dqh->size_iocintsts);
	printk("dqh->buff_ptr0=0x%x\n", dqh->buff_ptr0);
	printk("dqh->buff_ptr1=0x%x\n", dqh->buff_ptr1);
	printk("dqh->buff_ptr2=0x%x\n", dqh->buff_ptr2);
	printk("dqh->buff_ptr3=0x%x\n", dqh->buff_ptr3);
	printk("dqh->buff_ptr4=0x%x\n", dqh->buff_ptr4);
	printk("............................................at loc=0x%x done\n", loc);
}

static void dump_ctrlrequest(struct usb_ctrlrequest *cr)
{
	printk("bRequestType=0x%x\n", cr->bRequestType &0xff);
	printk("bRequest=0x%x\n", cr->bRequest &0xff);
	printk("wValue=0x%x\n", cr->wValue &0xffff);
	printk("wIndex=0x%x\n", cr->wIndex &0xffff);
	printk("wLength=0x%x\n", cr->wLength &0xffff);
	return;
}

static void dump_mem(unsigned char * buff, int len)
{
	int i = 0 ;
	for (i= 0;  i< len; i++) {
	       if (i%16==0 && i!=0)
         	      printk("\n");

		printk("%02x ", buff[i] & 0xff);
	}
	printk("\n");
}

static void dump_usb_ep_desc(const struct usb_endpoint_descriptor *desc)
{
	printk("desc->bLength=0x%02x\n", desc->bLength);
	printk("desc->bDescriptorType=0x%02x\n", desc->bDescriptorType);
	printk("desc->bEndpointAddress=0x%02x\n", desc->bEndpointAddress);
	printk("desc->bmAttributes=0x%02x\n", desc->bmAttributes);
	printk("desc->wMaxPacketSize=0x%04x\n", desc->wMaxPacketSize);
	printk("desc->bInterval=0x%02x\n", desc->bInterval);
	printk("desc->bRefresh=0x%02x\n", desc->bRefresh);
	printk("desc->bSynchAddress=0x%02x\n", desc->bSynchAddress);
}
static void dump_req(struct tangox_request *req)
{

	if (req) {
		printk("req=0x%x\n", (u32)req);
		printk("req->queue=0x%x\n",(u32)&req->queue);
		printk("req->req.buf=0x%x\n",(u32)req->req.buf);
		printk("req->req.length=0x%x\n", req->req.length);
		printk("req->req.dma=0x%x\n", req->req.dma);
		printk("req->req.no_int=0x%x\n", req->req.no_interrupt);
		printk("req->req.zero=0x%x\n", req->req.zero);
		printk("req->req.short_not_ok=0x%x\n", req->req.short_not_ok);
		printk("req->req.complete=0x%x\n", (u32)req->req.complete);
		printk("req->req.list=0x%x\n", (u32)&req->req.list);
		printk("req->req.status=0x%x\n", req->req.status);
		printk("req->req.actual=0x%x\n\n", req->req.actual);
	} else
		printk("req ==  null\n");
}

static void dump_ep_queues(struct tangox_otg *dev)
{
	int i = 0;
	struct tangox_ep       *ep;
    struct tangox_request   *r;

	for (i=0; i< 3; i++) {
		ep = &dev->ep[i];
        if (!list_empty (&ep->queue)) {
			printk("dumping ep[%d]->queue... \n", i);
            list_for_each_entry (r, &ep->queue, queue) {
				dump_req(r);
            }
        } else
			printk("ep[%d]->queue is empty\n", i);
	}
}

static void dump_list(struct list_head *list)
{
	struct list_head        *tmp;
	int i=0;

	list_for_each (tmp, list) {
		printk("list[%d]=0x%x\n",i,(u32)tmp);
		i++;
		if (i == MAX_EP_TR_DESCRS)
			break;
	}
	return;
}
#else
#define dump_registers(dev, yes)	do { } while (0)
#define dump_dtd(dtd)				do { } while (0)
#define dump_dqh(dev,dqh,loc)		do { } while (0)
#define dump_usb_ep_desc(desc)		do { } while (0)
#define dump_req(req)				do { } while (0)
//#define dump_ctrlrequest(cr)		do { } while (0)
//#define dump_mem(buff, len)		do { } while (0)
//#define dump_ep_queues(dev)		do { } while (0)
//#define dump_list(list)			do { } while (0)
#endif



static void ep_reset (struct tangox_op_regs __iomem *regs, struct tangox_ep *ep)
{
	ep->desc = NULL;
	INIT_LIST_HEAD (&ep->queue);

	ep->ep.maxpacket = ~0;
	ep->ep.ops = &tangox_ep_ops;
}

static void usb_reset (struct tangox_otg *dev)
{
    DEBUG("%s\n", __FUNCTION__);
    INIT_LIST_HEAD (&dev->gadget.ep_list);
    list_add_tail (&dev->ep [1].ep.ep_list, &dev->gadget.ep_list);
    list_add_tail (&dev->ep [2].ep.ep_list, &dev->gadget.ep_list);
    list_add_tail (&dev->ep [3].ep.ep_list, &dev->gadget.ep_list);
    list_add_tail (&dev->ep [4].ep.ep_list, &dev->gadget.ep_list);
}

static void usb_reinit (struct tangox_otg *dev)
{
	u32	tmp;

	/* basic endpoint init */
	for (tmp = 0; tmp < 4; tmp++) {
		struct tangox_ep	*ep = &dev->ep [tmp];

		ep->ep.name = ep_name [tmp];
		ep->dev = dev;
		ep->num = tmp;

		ep->regs = &dev->regs [tmp];
		ep_reset (dev->regs, ep);
	}
	dev->ep [0].ep.maxpacket = 64;

	dev->gadget.ep0 = &dev->ep [0].ep;
	INIT_LIST_HEAD (&dev->gadget.ep0->ep_list);
}

static inline void qh_init(struct tangox_otg *dev)
{
	int i;
	struct ep_qh *ep_qhptr;

    ep_qhptr = dev->ep_qhptr;

    /* Initialize all device queue heads */
    for (i=0; i<(dev->max_eps * 2); i++) {
        /* Interrupt on Setup packet */
        (ep_qhptr + i)->max_pktlen = (((u32)USB_MAX_CTRL_PAYLOAD <<
	        VUSB_EP_QUEUE_HEAD_MAX_PKT_LEN_POS) | VUSB_EP_QUEUE_HEAD_IOS);
           (ep_qhptr + i)->next_dtdptr =  VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE;
    }
}


static int get_free_dtd_count(struct tangox_otg *dev, struct list_head *list)
{
	struct list_head        *tmp;
	int i = 0;

	list_for_each (tmp, list) { 
        i++;
		if (i== MAX_EP_TR_DESCRS)
			break;
 	}
	return i;
}

//static DEFINE_SPINLOCK(dtd_lock);
static inline void dtd_init (struct tangox_otg *dev, struct dtd  *dtd)
{

    memset (dtd, 0, sizeof(struct dtd));
	/* set the dtd to be invalid */
	dtd->next_link_ptr = VUSBHS_TD_NEXT_TERMINATE;
	/* set the reserved fields to 0 */
	dtd->size_ioc_sts = 0;//~VUSBHS_TD_RESERVED_FIELDS;
	/* set all buff pointers to null*/
	dtd->buff_ptr0 = 0;
	dtd->buff_ptr1 = 0;
	dtd->buff_ptr2 = 0;
	dtd->buff_ptr3 = 0;
	dtd->buff_ptr4 = 0;
}

static struct dtd *get_dtd(struct tangox_otg *dev, struct list_head *dtd_list)
{
	struct dtd *dtd = NULL;
	
	if (!list_empty(dtd_list)) {	
		dtd = (struct dtd *)dtd_list->next;
		list_del((struct list_head *)dtd);
	}
	return dtd;	  
}


static void free_dtd (struct tangox_otg *dev, struct dtd *dtd, int lock)
{
	u32 addr;

	addr = (u32)dtd;
	if (addr<=CPU_REMAP_SPACE)
		return;

	if (dtd->size_ioc_sts & VUSBHS_TD_STATUS_ACTIVE)
		return;
	dtd_init(dev,dtd);
	list_add_tail((struct list_head *)dtd, &dtd_free_list);
}


/*check all dtds and free those if possible*/
static void retire_dtd( struct tangox_otg *dev)
{
	struct dtd *dtd;
    struct list_head        *tmp;
    int i=0 , free;

	dtd = dev->dtd_ptr;

    for (i=0;i<MAX_EP_TR_DESCRS;i++) {
		free = 1;
       	list_for_each (tmp, &dtd_free_list) {
			if ( (unsigned long) dtd == (unsigned long)tmp) {
				free = 0; 
				break;
			}				
		}
		
		if (free && (!(dtd->size_ioc_sts & VUSBHS_TD_STATUS_ACTIVE)))
			free_dtd(dev, dtd, 0);

		dtd++;
	}
}


static void dev_mem_cleanup (struct tangox_otg *dev)
{
	u32 mem_size;
    mem_size = CEILING((dev->max_eps*2 * sizeof(struct ep_qh)), L1_CACHE_SHIFT) +
			   CEILING((MAX_EP_TR_DESCRS) * sizeof(struct dtd), L1_CACHE_SHIFT);

	dma_free_coherent(&dev->pdev->dev, mem_size, dev->ep_qhptr, 0);
}

static dma_addr_t dev_mem_init (struct tangox_otg *dev)
{
	dma_addr_t dma;
    u32     mem_size=0;
    unsigned char* drv_mem;
    dev->max_eps = dev->cap_regs->dccparams & 0x1f;
    /* calculate the total mem size */
    mem_size = CEILING((dev->max_eps*2 * sizeof(struct ep_qh)), L1_CACHE_SHIFT) +
			   CEILING((MAX_EP_TR_DESCRS) * sizeof(struct dtd), L1_CACHE_SHIFT);
	DEBUG("max_eps=0x%x size of qh=0x%x size of dtd=0x%x size of list_head=0x%x\n", 
		dev->max_eps, sizeof(struct ep_qh), sizeof(struct dtd), sizeof(struct list_head));
	/* allocate memory*/
	drv_mem = dma_alloc_coherent(&dev->pdev->dev, mem_size, &dma, GFP_KERNEL | GFP_DMA);
    if (drv_mem == NULL) {
            DEBUG("%s %s\n", __FUNCTION__, "memory allocation  failed");
            return -ENOMEM;
    }

    /* QH base*/
    dev->ep_qhptr = (struct ep_qh *)MEM2048_ALIGN((u32)drv_mem);
    DEBUG("drv_mem=0x%x ep_qhptr=0x%x dma=0x%x\n",drv_mem,  (u32)dev->ep_qhptr, dma);

    /* DTD base */
    dev->dtd_ptr =  (struct dtd *)MEMCACHELINE_ALIGN((u32)dev->ep_qhptr + CEILING((dev->max_eps*2 * sizeof(struct ep_qh)), L1_CACHE_SHIFT));
	dev->dtd_dma =	MEMCACHELINE_ALIGN(dma + CEILING((dev->max_eps*2 * sizeof(struct ep_qh)), L1_CACHE_SHIFT));
    DEBUG("dtd_ptr =0x%x dtd_dma=0x%x\n",  (u32)dev->dtd_ptr, dev->dtd_dma);

	/* init dtd free list*/
    INIT_LIST_HEAD (&dtd_free_list);

	return dma;
}


static int tangox_enable (struct usb_ep *_ep, 
				const struct usb_endpoint_descriptor *desc)
{
	struct tangox_otg		*dev;
	struct tangox_ep		*ep;
    struct ep_qh			*ep_qhptr;
	unsigned long			flags, which_bit;
	u32	max, tmp;
    u8  direction, type;
	int i,ret = -1;
	
	dump_usb_ep_desc(desc);
	ep = container_of (_ep, struct tangox_ep, ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep_name[0]
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;
	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	/* sanity check ep-e/ep-f since their fifos are small */
	max = le16_to_cpu (desc->wMaxPacketSize) & 0x1fff;
	if (ep->num > 4 && max > 64)
		return -ERANGE;

	spin_lock_irqsave (&dev->lock, flags);
	_ep->maxpacket = max & 0x7ff;
	ep->desc = desc;

	tmp = desc->bEndpointAddress;

	DEBUG ("enabled %s (ep%d%s-%s) bmAttr=0x%x\n",
		_ep->name, tmp & 0x0f, DIR_STRING (tmp),
		type_string (desc->bmAttributes), desc->bmAttributes);


    direction = ((desc->bEndpointAddress & USB_DIR_IN) == USB_DIR_IN)? 1:0 ;
    type = (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK);
	for(i = 0; i < 2; i++) {
		/* Get the endpoint queue head address */
		ep_qhptr = dev->ep_qhptr + 2*ep->num + i;
		which_bit = (1 << (16 * i + ep->num));

		/* Check if the Endpoint is Primed */
		if ((!(dev->regs->ep_prime & which_bit)) && 
			(!(dev->regs->ep_stat & which_bit))) {

			/* Set the max packet length, interrupt 
			   on Setup and Mult fields */
			if (type == USB_ENDPOINT_XFER_ISOC) {
	        		 /* Mult bit should be set for isochronous endpoints */
		        	 ep_qhptr->max_pktlen = ((_ep->maxpacket << 16) |
	           			 (1 << VUSB_EP_QUEUE_HEAD_MULT_POS));
			} else {
				if (type != USB_ENDPOINT_XFER_CONTROL) 
					ep_qhptr->max_pktlen = (_ep->maxpacket << 16); 
				else 
					ep_qhptr->max_pktlen = ((_ep->maxpacket << 16) |
											VUSB_EP_QUEUE_HEAD_IOS);
			}
			/*zero length termination off*/
			ep_qhptr->max_pktlen |= VUSB_EP_QUEUE_HEAD_ZERO_LEN_TER_SEL; 

			/* Enable the endpoint for Rx and Tx and set the endpoint type */
			dev->regs->ep_ctrl[ep->num] |=
				((direction ? (EHCI_EPCTRL_TX_ENABLE | 
				EHCI_EPCTRL_TX_DATA_TOGGLE_RST) :
				(EHCI_EPCTRL_RX_ENABLE | 
				 EHCI_EPCTRL_RX_DATA_TOGGLE_RST)) |
				(type << (direction ? EHCI_EPCTRL_TX_EP_TYPE_SHIFT : 
				EHCI_EPCTRL_RX_EP_TYPE_SHIFT)));
		
			ret = 0;
		} else {
			ret =  -EFAULT;
			break;
		}
	}
	spin_unlock_irqrestore (&dev->lock, flags);
	return ret;
}

static void nuke (struct tangox_ep *);

static int tangox_disable (struct usb_ep *_ep)
{
	struct tangox_ep	*ep;
	unsigned long		flags;

	ep = container_of (_ep, struct tangox_ep, ep);
	if (!_ep || !ep->desc || _ep->name == ep_name[0])
		return -EINVAL;

	spin_lock_irqsave (&ep->dev->lock, flags);
	nuke (ep);
	ep_reset (ep->dev->regs, ep);

	spin_unlock_irqrestore (&ep->dev->lock, flags);
	return 0;
}


static struct usb_request *
tangox_alloc_request (struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct tangox_ep	*ep;
	struct tangox_request	*req;

	if (!_ep)
		return NULL;
	ep = container_of (_ep, struct tangox_ep, ep);

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return NULL;
	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD (&req->queue);

	return &req->req;
}

static void tangox_free_request (struct usb_ep *_ep, 
						struct usb_request *_req)
{
	struct tangox_ep	*ep;
	struct tangox_request	*req;

	ep = container_of (_ep, struct tangox_ep, ep);
	if (!_ep || !_req)
		return;

	req = container_of (_req, struct tangox_request, req);
	WARN_ON (!list_empty (&req->queue));
	kfree (req);
}


/* check when an ep is done*/
static void
tangox_done (struct tangox_ep *ep, 
				struct tangox_request *req, int status)
{
	struct tangox_otg	*dev;
	struct ep_qh 		*ep_qhptr = 0;
    struct dtd          *dtd_ptr = 0;
	struct usb_request  *_req;
    int		errors = 0;
	u32		next_dtd_ptr;
	u32		remain = 0;
	u32		actual = 0;
	u32		tx_size = 0;

	_req =  &req->req;
	DEBUG("%s _req=0x%x\n", __FUNCTION__, (u32)_req);

	list_del_init (&req->queue);

	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	/* calculate the actual transfer length*/
	dev = ep->dev;
	ep_qhptr = dev->ep_qhptr + ep->num*2 + (!ep->is_in ? 1 : 0);

	if (ep_qhptr->curr_dtdptr == VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE) 
		return;

    dtd_ptr = get_dtd_from_dma_addr(dev, ep_qhptr->curr_dtdptr);
    ep_qhptr->curr_dtdptr = VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE;
	actual = 0;
	remain = 0; 

    /* Process all the dTDs for respective transfers */
    while (dtd_ptr) {
		if (dtd_ptr->size_ioc_sts & VUSBHS_TD_STATUS_ACTIVE) {
			/* No more dTDs to process. Next one is owned by VUSB */
			break;
		}

		/* Get the address of the next dTD */
		next_dtd_ptr = dtd_ptr->next_link_ptr;
		/* corrupted when unplug then plug*/
		if(next_dtd_ptr > (u32)dev->dtd_ptr) {
			next_dtd_ptr = VUSBHS_TD_NEXT_TERMINATE;
			dtd_init(dev, dtd_ptr);
		}

		/* Read the errors */
		errors = (dtd_ptr->size_ioc_sts & VUSBHS_TD_ERROR_MASK);
		if (!errors) {
			/* Get the length of transfer from the current dTD */
			remain = ((dtd_ptr->size_ioc_sts & VUSB_EP_TR_PACKET_SIZE) >> 16);
		    tx_size = _req->length - remain - actual;
			actual = _req->length - remain; 
		} else {
			if (errors & VUSBHS_TD_STATUS_HALTED) {
                /* Clear the errors and Halt condition */
                ep_qhptr->size_iocintsts &= ~errors;
				dump_dqh(dev, ep_qhptr, 9);
			}
		}
		/* Retire the dtd */
		free_dtd(dev, dtd_ptr,1);

		if (next_dtd_ptr == VUSBHS_TD_NEXT_TERMINATE)
			break;

		dtd_ptr = get_dtd_from_dma_addr(dev, next_dtd_ptr);
    }

	_req->actual = actual;		

	/* see if there is an error */
	if (errors && _req->actual != _req->length) {
       	DEBUG("errors=0x%x req length = 0x%x actual=0x%x \n", 
			errors, _req->length, _req->length);
		/* resubmition or halt */
	}

	//check and retire
	if ((u32)get_free_dtd_count(dev, &dtd_free_list) < 10)
		retire_dtd(dev);

	if (req->mapped && _req->length) {
		dma_unmap_single(&dev->gadget.dev, _req->dma, _req->length,
			ep->is_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		_req->dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	}

	spin_unlock (&dev->lock);
	req->req.complete (&ep->ep, &req->req);
	spin_lock (&dev->lock);
	
	DEBUG("%s req=0x%x done\n", __FUNCTION__,  (u32)_req);
}

#if 0
/* to unstall ep_num = 1 or 2*/
static void ep_unstall(struct tangox_otg *dev, int ep_num)
{ 
   dev->regs->ep_ctrl[ep_num] &= (ep_num == 1 ? 
		~EHCI_EPCTRL_TX_EP_STALL : ~EHCI_EPCTRL_RX_EP_STALL);

}
#endif

static int tangox_ep_irq (struct tangox_otg *dev);
static int tangox_completion(struct tangox_otg *dev);
static int tangox_queue (struct usb_ep *_ep, struct usb_request *_req, 
							gfp_t gfp_flags)
{
	struct tangox_request	*req = 0;
	struct tangox_ep		*ep = 0;
	struct tangox_otg		*dev = 0;
	struct ep_qh	*ep_qhptr = 0;
	struct dtd		*dtd_ptr = 0;
	struct dtd		*tmp_dtd_ptr = 0;
	struct dtd		*first_dtd_ptr = 0;
	unsigned long	flags;
	int prime = -1, need_prime = 0, i=0;
	u32 temp_ep_stat= 0;
	u32 cur_len, remaining, dma_addr;

	req = container_of (_req, struct tangox_request, req);
	if (!_req || !_req->complete || !_req->buf
			|| !list_empty (&req->queue))
		return -EINVAL;

	ep = container_of (_ep, struct tangox_ep, ep);
	if (!_ep || (!ep->desc && ep->num != 0))
		return -EINVAL;

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	/*set address's length=0*/
	if (_req->length == 0 && ep->num!=0)
		return -EOPNOTSUPP;

	if (ep->num == 1)
		ep->is_in = 0;
	else if (ep->num == 2)
		ep->is_in = 1;
	ep_qhptr = dev->ep_qhptr + ep->num*2 +(ep->is_in ? 0: 1);
	prime = (!ep->is_in ? 16 + ep->num : ep->num);

	while(!list_empty(&ep->queue)) {
		udelay(1);
	}

	spin_lock_irqsave (&dev->lock, flags);

	/* set up dma mapping in case the caller didn't */
	if (_req->dma == DMA_ADDR_INVALID && _req->length) {
		_req->dma = dma_map_single(&dev->gadget.dev, _req->buf, _req->length,
			ep->is_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->mapped = 1;
	}

	dma_addr = _req->dma;
	remaining = _req->length;
	do {
		if (remaining > VUSB_EP_MAX_LENGTH_TRANSFER) 
			cur_len = VUSB_EP_MAX_LENGTH_TRANSFER;
		else 
			cur_len = remaining;

      	remaining -= cur_len;
		i++;

		dtd_ptr = get_dtd( dev, &dtd_free_list);
		if (!dtd_ptr) {
			printk("FIXME 2: cann't get dtd, retry\n");
			spin_unlock_irqrestore (&dev->lock, flags);
			return -ENOMEM;
		}
		dtd_ptr->buff_ptr0 = cpu_to_le32(dma_addr);
		dtd_ptr->buff_ptr1 = cpu_to_le32(dma_addr + 0x1000);
		dtd_ptr->buff_ptr2 = cpu_to_le32(dma_addr + 0x2000);
		dtd_ptr->buff_ptr3 = cpu_to_le32(dma_addr + 0x3000);
		dtd_ptr->buff_ptr4 = cpu_to_le32(dma_addr + 0x4000);
		dtd_ptr->size_ioc_sts = ((cur_len << VUSBHS_TD_LENGTH_BIT_POS) |
									(VUSBHS_TD_STATUS_ACTIVE));
		if (!remaining)
                	dtd_ptr->size_ioc_sts |= VUSBHS_TD_IOC;
		else
			dma_addr += cur_len;

		dtd_ptr->size_ioc_sts &= ~VUSBHS_TD_RESERVED_FIELDS;
		dtd_ptr->next_link_ptr = VUSBHS_TD_NEXT_TERMINATE;

		if (tmp_dtd_ptr) 
			tmp_dtd_ptr->next_link_ptr = get_dma_addr_from_dtd(dev, dtd_ptr); 

		tmp_dtd_ptr = dtd_ptr;

		if (i==1)
			first_dtd_ptr = dtd_ptr;
		
	} while (remaining);

	spin_unlock_irqrestore (&dev->lock, flags);
again:
    if (list_empty (&ep->queue)) {
		if (ep_qhptr->next_dtdptr == VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE) {
			ep_qhptr->next_dtdptr = get_dma_addr_from_dtd(dev, first_dtd_ptr);
		}
		else 
			printk("oops ep_qhptr->next_dtdptr=0x%x\n", ep_qhptr->next_dtdptr);

	    ep_qhptr->size_iocintsts = 0;
		need_prime = 1;
	} else {
		if (dev->regs->ep_prime != 1<< prime) {
			while(1) {
				dev->regs->usbcmd |= EHCI_CMD_SETUP_TRIPWIRE_SET;
				temp_ep_stat = dev->regs->ep_stat;
				if (dev->regs->usbcmd & EHCI_CMD_SETUP_TRIPWIRE_SET) {
					break;
				}
			}

			dev->regs->usbcmd &= EHCI_CMD_SETUP_TRIPWIRE_CLEAR;
       
	       	if (!temp_ep_stat) {
				if (ep_qhptr->next_dtdptr == VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE)
					ep_qhptr->next_dtdptr =get_dma_addr_from_dtd(dev, first_dtd_ptr);
				else 
					printk("oops ep_qhptr->next_dtdptr=0x%x\n", ep_qhptr->next_dtdptr);

		        ep_qhptr->size_iocintsts = 0;
				need_prime = 1;
			} else
				goto again;
		
		} else
			goto again;
	}

    /* there may be a prev dtd need to free */
    //free_prev_dtd(dev, ep_qhptr);

    if (req)
        list_add_tail (&req->queue, &ep->queue);
    /* check and prime the ep */
	if (need_prime)
	        dev->regs->ep_prime |= (1 << prime);

	return 0;
}

/* dequeue ALL requests */
static void nuke (struct tangox_ep *ep)
{
	struct tangox_request	*req;

	/* called with spinlock held */
	while (!list_empty (&ep->queue)) {
		req = list_entry (ep->queue.next,
				struct tangox_request,
				queue);
		tangox_done (ep, req, -ESHUTDOWN);
	}
}

/* dequeue JUST ONE request */
static int tangox_dequeue (struct usb_ep *_ep, struct usb_request *_req)
{
	struct tangox_ep		*ep;
	struct tangox_request	*req;
	unsigned long			flags;

	DEBUG("%s\n", __FUNCTION__);
	ep = container_of (_ep, struct tangox_ep, ep);
	if (!_ep || (!ep->desc && ep->num != 0) || !_req)
		return -EINVAL;

	spin_lock_irqsave (&ep->dev->lock, flags);

	/* make sure it's still queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore (&ep->dev->lock, flags);
		return -EINVAL;
	}

	/*dequeue here */
    if (req)
		tangox_done (ep, req, -ECONNRESET);

	spin_unlock_irqrestore (&ep->dev->lock, flags);
	return 0;
}

static int tangox_set_halt (struct usb_ep *_ep, int value)
{
	struct tangox_ep	*ep;
	unsigned long		flags;
	int			retval = 0;

	DEBUG("%s...\n", __FUNCTION__);
	ep = container_of (_ep, struct tangox_ep, ep);
	if (!_ep || (!ep->desc && ep->num != 0))
		return -EINVAL;
	if (!ep->dev->driver || ep->dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	spin_lock_irqsave (&ep->dev->lock, flags);
	if (!list_empty (&ep->queue))
		retval = -EAGAIN;
	else if (ep->is_in && value)
		retval = -EAGAIN;
	else {
		DEBUG ("%s %s halt\n", _ep->name,
				value ? "set" : "clear");

		/* set/clear, then synch memory views with the device */
		if (value) {
			if (ep->num != 0)
				set_halt (ep);
		} else
			clear_halt (ep);
	}
	spin_unlock_irqrestore (&ep->dev->lock, flags);

	return retval;
}

static int tangox_fifo_status (struct usb_ep *_ep)
{
	struct tangox_ep	*ep;

	ep = container_of (_ep, struct tangox_ep, ep);
	if (!_ep || (!ep->desc && ep->num != 0))
		return -ENODEV;
	if (!ep->dev->driver || ep->dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	return -EOPNOTSUPP;
}

static void tangox_fifo_flush (struct usb_ep *_ep)
{
	struct tangox_ep	*ep;
	struct tangox_otg           *dev = 0;
	int bit_pos;

	ep = container_of (_ep, struct tangox_ep, ep);
	if (!_ep || (!ep->desc && ep->num != 0))
		return;
	if (!ep->dev->driver || ep->dev->gadget.speed == USB_SPEED_UNKNOWN)
		return;

	dev =  ep->dev;
	bit_pos = (1 << (16 * (!ep->is_in) + ep->num));
	
	/* Write 1 to the Flush register */
	dev->regs->ep_flush = bit_pos;

	/* Wait until flushing completed */
	while (dev->regs->ep_flush & bit_pos) 
		;
}

static const struct usb_ep_ops tangox_ep_ops = {
	.enable		= tangox_enable,
	.disable	= tangox_disable,

	.alloc_request	= tangox_alloc_request,
	.free_request	= tangox_free_request,

	.queue		= tangox_queue,
	.dequeue	= tangox_dequeue,

	.set_halt	= tangox_set_halt,
	.fifo_status	= tangox_fifo_status,
	.fifo_flush	= tangox_fifo_flush,
};


static int tangox_get_frame (struct usb_gadget *_gadget)
{
	return -EOPNOTSUPP;
}

static int tangox_wakeup (struct usb_gadget *_gadget)
{
	return 0;
}

static int tangox_set_selfpowered (struct usb_gadget *_gadget, int value)
{
	return 0;
}

static int tangox_pullup(struct usb_gadget *_gadget, int is_on)
{
	return 0;
}

static const struct usb_gadget_ops tangox_ops = {
	.get_frame	= tangox_get_frame,
	.wakeup		= tangox_wakeup,
	.set_selfpowered = tangox_set_selfpowered,
	.pullup		= tangox_pullup,
};


#ifdef	CONFIG_USB_GADGET_DEBUG_FILES

/* FIXME move these into procfs, and use seq_file.
 * Sysfs _still_ doesn't behave for arbitrarily sized files,
 * and also doesn't help products using this with 2.4 kernels.
 */

/* "function" sysfs attribute */
static ssize_t
show_function (struct device *_dev, struct device_attribute *attr, char *buf)
{
	struct tangox_otg	*dev = dev_get_drvdata (_dev);

	if (!dev->driver
			|| !dev->driver->function
			|| strlen (dev->driver->function) > PAGE_SIZE)
		return 0;
	return scnprintf (buf, PAGE_SIZE, "%s\n", dev->driver->function);
}
static DEVICE_ATTR (function, S_IRUGO, show_function, NULL);

static ssize_t
show_registers (struct device *_dev, struct device_attribute *attr, char *buf)
{
	struct tangox_otg		*dev;
	char			*next;
	unsigned		size, t;
	unsigned long		flags;
	const char		*s;

	dev = dev_get_drvdata (_dev);
	next = buf;
	size = PAGE_SIZE;
	spin_lock_irqsave (&dev->lock, flags);

	if (dev->driver)
		s = dev->driver->driver.name;
	else
		s = "(none)";

        /* USB Device/Host Capablility Registers */
        t = scnprintf (next, size,
                "caplength(1) 0x%08x  hciversion(2) 0x%08x  hcsparams    0x%08x\n"
                "hccparam     0x%08x  dciversion    0x%08x  dccparams    0x%08x\n\n",
                readl (&dev->cap_regs->caplength)  & 0x000000ff ,
                readl (&dev->cap_regs->hciversion) & 0x0000ffff,
                readl (&dev->cap_regs->hcsparams),
                readl (&dev->cap_regs->hccparams),
                readl (&dev->cap_regs->dciversion),
                readl (&dev->cap_regs->dccparams));
        size -= t;
        next += t;


        /* Device/Host Operational Registers */
        t = scnprintf (next, size,
                "usbcmd       0x%08x  usbsts        0x%08x  usbintr      0x%08x\n"
                "frindex      0x%08x  ctrldssegment 0x%08x  deviceaddr   0x%08x\n"
                "eplistaddr   0x%08x  ttctrl        0x%08x  burstsize    0x%08x\n"
                "txfilltuning 0x%08x  ulpi_viewport 0x%08x  ep_nak       0x%08x\n"
                "ep_nak_en    0x%08x  config_flag   0x%08x  port_sc[0]   0x%08x\n"
                "otg_sc       0x%08x  usb_mode      0x%08x  ep_setupstat 0x%08x\n"
                "ep_prime     0x%08x  ep_flush      0x%08x  ep_stat      0x%08x\n"
                "ep_complete  0x%08x  ep_ctrl[0]    0x%08x  ep_ctrl[1]   0x%08x\n\n",
                readl (&dev->regs->usbcmd),readl (&dev->regs->usbsts),readl (&dev->regs->usbintr),
                readl (&dev->regs->frindex),readl (&dev->regs->ctrldssegment),readl (&dev->regs->deviceaddr),
                readl (&dev->regs->eplistaddr),readl (&dev->regs->ttctrl),readl (&dev->regs->burstsize),
                readl (&dev->regs->txfilltuning),readl (&dev->regs->ulpi_viewport),readl (&dev->regs->ep_nak),
                readl (&dev->regs->ep_nak_en),readl (&dev->regs->config_flag),readl (&dev->regs->port_sc[0]),
                readl (&dev->regs->otg_sc),readl (&dev->regs->usb_mode),readl (&dev->regs->ep_setupstat),
                readl (&dev->regs->ep_prime),readl (&dev->regs->ep_flush),readl (&dev->regs->ep_stat),
                readl (&dev->regs->ep_complete),readl (&dev->regs->ep_ctrl[0]),readl (&dev->regs->ep_ctrl[1]));
        size -= t;
        next += t;

	spin_unlock_irqrestore (&dev->lock, flags);

	return PAGE_SIZE - size;
}
static DEVICE_ATTR (registers, S_IRUGO, show_registers, NULL);

#else

#define device_create_file(a,b)	(0)
#define device_remove_file(a,b)	do { } while (0)

#endif

static void dtd_list_init(struct tangox_otg *dev)
{
	struct dtd* dtd_ptr = 0;
	int i;

	if (!dev->dtd_ptr)
		return;

	while (!list_empty(&dtd_free_list)) {
		struct dtd *dtd_ptr;
		dtd_ptr = list_entry(dtd_free_list.next, struct dtd, dtd_list);
		if (&dtd_ptr->dtd_list)
			list_del(&dtd_ptr->dtd_list);
		else{
			break;
		}
	}

	/* init all dtds*/
	dtd_ptr = dev->dtd_ptr;	
	for (i=0;i<MAX_EP_TR_DESCRS;i++) {
		dtd_init(dev,dtd_ptr);
		list_add_tail((struct list_head *)dtd_ptr, &dtd_free_list);
		dtd_ptr++;
	}
}

static int tangox_udc_init(struct tangox_otg *dev, u8 inited)
{
	dma_addr_t  dma_hdl = 0;
    unsigned long temp;
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	int ctrl = controller;

	DEBUG("%s TangoX USB initializing...\n", __FUNCTION__);

	tangox_phy_power_up(ctrl);

	if (((chip_id & 0xfff0) == 0x8670) || ((chip_id & 0xfff0) == 0x8680))  {
		/* 0. Program the clean divider and clock multiplexors to provide 48MHz clock reference*/
		/* this is to be done in zboot */

		/*1. program the phy analog ctrl reg*/ 
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x2c, 0x0020DB91);
		wait_ms(5);

		/*2. program the PHY clock reference frequency*/ 
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0xC, 0x000F9930);
		wait_ms(5);
		 
		/* 3. Releasing USB PHY */
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x00000002);
		wait_ms(5);

		/* 4. Releasing the Host Controller */
		//PHY force unreset
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x10000002);
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x10000000);
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x00000000);
		wait_ms(5);

		/*5. Applying a 2nd HOST reset, after the NanoPHY 60MHz clock has become action */
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x10000002);
		wait_ms(5);
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x10000000);
		wait_ms(5);
		//Remove the PHY force unreset 
		gbus_write_reg32(tangox_ctrl_base[ctrl] + 0x0, 0x00000000);
		wait_ms(5);
	} else {
		/*0. set bit 1 of USB control register 0x21700*/ 
		temp = gbus_read_uint32(pGBus, tangox_ctrl_base[ctrl] + 0x0);
		gbus_write_uint32(pGBus, tangox_ctrl_base[ctrl] + 0x0, temp | 0x2);
		wait_ms(5);
 
		/* 1. Program the clean divider and clock multiplexors to provide 48MHz clock reference*/
		/* this is to be done in zboot */
	
		/* 2. Enable the USB PHY */
		temp = gbus_read_uint32(pGBus,tangox_ctrl_base[ctrl] + 0x0);
		gbus_write_uint32(pGBus,tangox_ctrl_base[ctrl] + 0x0, temp & 0xffffff7e);
		wait_ms(20);
	
		/* 3. Enable the USB Host EHCI/OHCI */
		temp = gbus_read_uint32(pGBus, tangox_ctrl_base[ctrl] + 0x0);
		gbus_write_uint32(pGBus, tangox_ctrl_base[ctrl] + 0x0, temp & 0xfffffffd);
		wait_ms(20);
	}	
	/*set it to dev mode*/
	temp = gbus_read_uint32(pGBus, tangox_ehci_base[ctrl] + TANGOX_EHCI_REG_OFFSET +0xA8) & 0xfffffffc;
	temp |= 2 ;
	gbus_write_uint32(pGBus, tangox_ehci_base[ctrl] + TANGOX_EHCI_REG_OFFSET +0xA8, temp);
	wait_ms(20);

	DEBUG("%s TangoX USB initializing...done\n", __FUNCTION__);

	if ( !inited) {
		dma_hdl = dev_mem_init (dev); 
		if (!dma_hdl)
			return -ENOMEM;
	}
	/* Stop the controller */
	dev->regs->usbcmd &= ~EHCI_CMD_RUN_STOP;
   	/* Program the controller to be the USB device controller */
	dev->regs->usb_mode = (VUSBHS_MODE_CTRL_MODE_DEV | 
			      VUSBHS_MODE_SETUP_LOCK_DISABLE);
    dev->regs->usbsts = dev->regs->usbsts;
	dev->regs->ep_setupstat = 0;
	/* Configure the Endpoint List Address */
	dev->regs->eplistaddr = dma_hdl;

	/* Initialize the endpoint 0 properties */
#if 0 //TODO
	dev->regs->ep_ctrl[0] =
      	(EHCI_EPCTRL_TX_DATA_TOGGLE_RST | EHCI_EPCTRL_RX_DATA_TOGGLE_RST);
	dev->regs->ep_ctrl[0] =
      	~(EHCI_EPCTRL_TX_EP_STALL | EHCI_EPCTRL_RX_EP_STALL);
#endif

	/*  interrupts*/
	dev->regs->usbintr =EHCI_INTR_INT_EN |
     		            EHCI_INTR_ERR_INT_EN |
     	        	    EHCI_INTR_PORT_CHANGE_DETECT_EN|
	     	            //EHCI_INTR_SOF_UFRAME_EN |
	     	            EHCI_INTR_DEVICE_SUSPEND |
					    EHCI_INTR_RESET_EN;
	dev->usb_state = USB_STATE_UNKNOWN;

	/* init all qh*/
	qh_init(dev);
	/* init dtd free list */
	dtd_list_init(dev);
	/* to init ep_list*/
	usb_reset (dev);
	usb_reinit (dev);

	/* Set the Run bit in the command register */
	//dev->regs->usbcmd = EHCI_CMD_RUN_STOP;
   	DEBUG("%s done\n",__FUNCTION__);
	return 0;
}


/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
int usb_gadget_probe_driver (struct usb_gadget_driver *driver,
			int (*bind)(struct usb_gadget *))
{
	struct tangox_otg		*dev = the_controller;
	int			retval;
	unsigned		i;

	/* insist on high speed support from the driver, since
	 * (dev->usb->xcvrdiag & FORCE_FULL_SPEED_MODE)
	 * "must not be used in normal operation"
	 */
	DEBUG("%s\n", __FUNCTION__);
	if (!driver
			/*|| driver->speed != USB_SPEED_HIGH*/
			|| !bind
			|| !driver->setup) {
		return -EINVAL;
	}
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	for (i = 0; i < 4; i++)
		dev->ep [i].irqs = 0;

	/* hook up the driver ... */
	driver->driver.bus = NULL;
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;
//	device_add(&dev->gadget.dev);
	retval = bind (&dev->gadget);
	if (retval) {
		DEBUG ("bind to driver %s --> %d\n",
				driver->driver.name, retval);
		dev->driver = NULL;
		dev->gadget.dev.driver = NULL;
		return retval;
	}
#if 0
	retval = device_create_file (&dev->pdev->dev, &dev_attr_function);
	if (retval) goto err_unbind;
	retval = device_create_file (&dev->pdev->dev, &dev_attr_queues);
	if (retval) goto err_func;
#endif
	printk ("Driver %s registered.\n", driver->driver.name);

	/* then start the controller*/
	dev->regs->usbcmd = EHCI_CMD_RUN_STOP;
	return 0;

	device_remove_file (&dev->pdev->dev, &dev_attr_function);

	//err_unbind:
	driver->unbind (&dev->gadget);
	dev->gadget.dev.driver = NULL;
	dev->driver = NULL;
	return retval;
}
EXPORT_SYMBOL (usb_gadget_probe_driver);

static void
stop_activity (struct tangox_otg *dev, struct usb_gadget_driver *driver)
{
	int			i;

	/* don't disconnect if it's not connected */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;

	/* Stop the controller */
	dev->regs->usbcmd &= ~EHCI_CMD_RUN_STOP;

	/* stop hardware; prevent new request submissions;
	 * and kill any outstanding requests.
	 */
	usb_reset (dev);
	for (i = 0; i < 4; i++)
		nuke (&dev->ep [i]);

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		spin_unlock (&dev->lock);
		driver->disconnect (&dev->gadget);
		spin_lock (&dev->lock);
	}

	usb_reinit (dev);
}

int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	struct tangox_otg	*dev = the_controller;
	unsigned long	flags;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver || !driver->unbind)
		return -EINVAL;

	spin_lock_irqsave (&dev->lock, flags);
	stop_activity (dev, driver);
	spin_unlock_irqrestore (&dev->lock, flags);

	tangox_pullup (&dev->gadget, 0);
	driver->unbind (&dev->gadget);
//	device_del(&dev->gadget.dev);

	dev->gadget.dev.driver = NULL;
	dev->driver = NULL;
#if 0
	device_remove_file (&dev->pdev->dev, &dev_attr_function);
	device_remove_file (&dev->pdev->dev, &dev_attr_queues);
#endif
	printk ("Driver %s unregistered.\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);

int get_ep_num(struct tangox_otg *dev, unsigned long stat)
{
	int i;
	int ep = -1;
	if (stat) {
		for(i = 0; i < 32; i++) {
			if (((stat >> i) & 1) == 1) {
				ep = i;
				break;
			}
		}	
	}
	return ep;
}

static int tangox_completion(struct tangox_otg *dev)
{
    struct tangox_ep        *ep = 0;
	struct tangox_request 	*req= 0;
	struct ep_qh            *ep_qhptr;
	struct dtd 		*dtd_ptr;
	int i;
	int completed = 0;
	int direction = 0;
	int done      = 0;
	int ep_num    = 0;
	int cbs[4] = {0,2,16,17};
	
	completed = dev->regs->ep_complete;
	dev->regs->ep_complete = completed;

	if (!completed)
		return 0;

    DEBUG("%s completion %04x\n", __FUNCTION__, completed);
	/* check though each complete bit in cbs[]*/
    for(i=0; i< 4; i++) {
		if (((completed >> cbs[i]) & 0x1) == 1)
            ep_num = cbs[i];
        else
            continue;

        if (ep_num <= 15) {
			/*out packet, device receive, need a response*/
           direction =  0; 
		}
        else { 
			/*in packet, submit from device */
            direction =  1;
            ep_num -= 16;
        }
	
		ep =  &dev->ep[ep_num];
        if (!list_empty (&ep->queue)) {
			struct tangox_request * tmp = 0 ;
            list_for_each_entry (req, &ep->queue, queue) {
				//dump_req(req);
				if ( tmp == req)
					break;

				tangox_done (ep, req, 0);
				tmp = req;
				done = 1;
            }
        }

		if ((ep_num == 0) && (direction == 1)) {
	        /* if it's an in packet, need a response */
			/* get a dtd from qh dtd free list*/
			dtd_ptr = get_dtd( dev, &dtd_free_list);
			if (!dtd_ptr) {
				printk("FIXME 3: cann't get dtd, retry\n");
				return -1;
			}
	        dtd_ptr->size_ioc_sts = (0 << VUSBHS_TD_LENGTH_BIT_POS) |
    	                           (VUSBHS_TD_IOC) | (VUSBHS_TD_STATUS_ACTIVE);
	        dtd_ptr->size_ioc_sts &= ~VUSBHS_TD_RESERVED_FIELDS;
            dtd_ptr->next_link_ptr = VUSBHS_TD_NEXT_TERMINATE;

	        ep_qhptr = dev->ep_qhptr;
	        ep_qhptr->next_dtdptr = get_dma_addr_from_dtd(dev, dtd_ptr); 
	        ep_qhptr->size_iocintsts = 0;

	        /* there may be a prev dtd need to free */
    	    //free_prev_dtd(dev, ep_qhptr);

    	    dev->regs->ep_prime |= 1 << ep_num;

		    free_dtd(dev, dtd_ptr, 1);
	    } 
    }

    DEBUG("%s completion %04x done\n", __FUNCTION__, completed);
	return 0;
}


static int tangox_ep_irq (struct tangox_otg *dev)
{
	struct tangox_ep	*ep = 0;
	struct ep_qh		*ep_qhptr;
	struct usb_ctrlrequest  *cr = NULL;
	int	tmp = 0;
	u32	ep_num = 0, setup_stat=0;

	/* try to get the setup buffer asap*/
	setup_stat = dev->regs->ep_setupstat;
	if (setup_stat) {
		ep_num = get_ep_num(dev,  setup_stat);
		if (ep_num == 0) {

			cr = kzalloc(sizeof(struct usb_ctrlrequest), GFP_NOIO);
			memset(cr, 0, sizeof(struct usb_ctrlrequest));

			ep_qhptr = dev->ep_qhptr +2*ep_num + 0;   
			/* Clear the bit in the ENDPTSETUPSTAT */
			dev->regs->ep_setupstat =  setup_stat;
			while(1) {
				/*setup tripwire*/
				dev->regs->usbcmd |= EHCI_CMD_SETUP_TRIPWIRE_SET;
				memcpy(cr, (unsigned char *)ep_qhptr->setup_buff, 8);
				mb();
				if (dev->regs->usbcmd & EHCI_CMD_SETUP_TRIPWIRE_SET) {
					dev->regs->usbcmd &= EHCI_CMD_SETUP_TRIPWIRE_CLEAR;
					break;
				}
			}
		} else {
			printk("wrong endpoint for setup packet: ep_num=0x%x", ep_num);
			return -1;
		}
	}

	/* precess those completed dtds*/
	tangox_completion(dev);

	if (!setup_stat) {
		return 0;
	}

	/*ep->is_in is from host to device*/
	ep = &dev->ep[ep_num];
	ep->is_in = (cr->bRequestType & USB_DIR_OUT) != 0;
	switch (cr->bRequest) {
		case USB_REQ_SET_ADDRESS: {
									  
        /* with hw assistance*/
        dev->regs->deviceaddr = (cr->wValue << VUSBHS_ADDRESS_BIT_SHIFT) |
                               (0x01 << (VUSBHS_ADDRESS_BIT_SHIFT -1));
        tmp = dev->driver->setup (&dev->gadget, cr);

		break;
		}
		case USB_REQ_GET_STATUS: {
			printk("USB_REQ_GET_STATUS 0x%x\n", cr->bRequest);
		   /* hw handles device and interface status */
			if (cr->bRequestType != (USB_DIR_IN|USB_RECIP_ENDPOINT))
	           goto out;
		}
		break;
		case USB_REQ_CLEAR_FEATURE: {
			DEBUG("USB_REQ_CLEAR_FEATURE 0x%x\n", cr->bRequest);
	    }
	    break;
		case USB_REQ_SET_FEATURE: {
			DEBUG("USB_REQ_SET_FEATURE 0x%x\n", cr->bRequest);
		}	
		break;
		default:
			spin_unlock (&dev->lock);
			tmp = dev->driver->setup (&dev->gadget, cr);
		    spin_lock (&dev->lock);
	}

	/* stall ep0 on error */
	if (tmp < 0) {
		printk ("req %02x.%02x protocol STALL; stat %d\n",
			cr->bRequestType, cr->bRequest, tmp);
	}
out:
	if (cr)
		kfree(cr);

	return 0;
}


static void tangox_dev_reset(struct tangox_otg   *dev)
{
	u32	i, temp;

	DEBUG("%s\n", __FUNCTION__);
	/* The address bits are past bit 25-31. Set the address */
	dev->regs->deviceaddr &= ~0xFE000000;

	/* Clear all the setup token semaphores */
	temp = dev->regs->ep_setupstat;
	dev->regs->ep_setupstat = temp;

	/* Clear all the endpoint complete status bits */
	temp = dev->regs->ep_complete;
	dev->regs->ep_complete = temp;

	/* Clear all the setup token semaphores */
	temp = dev->regs->ep_stat;
	dev->regs->ep_stat = temp;

	while (dev->regs->ep_prime & 0xFFFFFFFF) {
	   ;/* Wait until all ENDPTPRIME bits cleared */
	}

	/* Write 1s to the Flush register */
	dev->regs->ep_flush = 0xFFFFFFFF;

	/* Unstall all endpoints */
	for (i=0;i < dev->max_eps; i++) {
   	   dev->regs->port_sc[i] &=  ~EHCI_EPCTRL_RX_EP_STALL ;
   	   dev->regs->port_sc[i] &=  ~EHCI_EPCTRL_TX_EP_STALL ;
	}

	dtd_list_init(dev);

	DEBUG("%s done\n", __FUNCTION__);
#if 0
	if (dev->regs->port_sc[0] & EHCI_PORTSCX_PORT_RESET*/)
	{
		dev->bus_resetting = 1;
		dev->usb_state = USB_STATE_POWERED;
	} else {
		/* re-initialize */
		tangox_udc_init (dev, 1);
	}
#endif	
	return;
}

static void tangox_dev_error(struct tangox_otg   *dev)
{
        struct ep_qh            *ep_qhptr = 0;
	struct tangox_ep        *ep = 0;
	int i;

	dump_registers(dev, 1);
	for(i = 0; i < 3; i++) {
		ep =  &dev->ep[i];
		printk("ep[%d] ep->is_in=0%d\n", i, ep->is_in);	
        	ep_qhptr = dev->ep_qhptr + ep->num*2 + (!ep->is_in ? 1 : 0);
		dump_dqh(dev, ep_qhptr, 8);
	}
}

static irqreturn_t tangox_irq (int irq, void *_dev)
{
	struct tangox_otg	*dev = _dev;
	u32             stat = 0; 
	u32		intr = 0;

	intr = dev->regs->usbintr;
	stat = dev->regs->usbsts;

	if (!(intr & stat))
		return IRQ_NONE;

	spin_lock (&dev->lock);
  	/* disable intrrupts*/
	//dev->regs->usbintr = 0;

	/* Clear all the interrupts occured */
	dev->regs->usbsts =  stat;

	if (stat & EHCI_STS_INT) { 
		tangox_ep_irq (dev);
	}
	
	if (stat & EHCI_STS_RESET) {
		tangox_dev_reset(dev);
	}
	if (stat & EHCI_STS_PORT_CHANGE) {
   		//tangox_dev_port_change(dev);
	}
	if (stat & EHCI_STS_ERR) {
   		tangox_dev_error(dev);
	}

	if (stat & EHCI_STS_SUSPEND) {
		//tangox_dev_suspend(dev);
	}
	if (stat & EHCI_STS_SOF) {
   		//tangox_dev_SOF(dev);
	}

 	/* enable intrrupts*/
	//dev->regs->usbintr = intr;
	spin_unlock (&dev->lock);

	return IRQ_HANDLED;
}


static void gadget_release (struct device *_dev)
{
	struct tangox_otg	*dev = dev_get_drvdata (_dev);
	kfree (dev);
}

/* tear down the binding between this driver and the device */
static int tangox_remove (struct platform_device *pdev)
{
	struct tangox_otg *dev = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);
#ifdef CONFIG_TANGO3
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	if (((chip_id == 0x8672) || (chip_id == 0x8674)) && (controller == 1))
		irq = IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_DVD_INT;
#endif
	BUG_ON(dev->driver);

	iounmap(dev->base);
	dev_mem_cleanup(dev);
	free_irq(irq, dev);

	usb_gadget_unregister_driver(dev->driver);
	device_unregister (&dev->gadget.dev);
	device_remove_file (&pdev->dev, &dev_attr_registers);

	platform_set_drvdata(pdev, 0);
	the_controller = NULL;
	return 0;
}

/* wrap this driver around the specified device, but
 * don't respond over USB until a gadget driver binds to us.
 */
static int tangox_probe(struct platform_device *pdev)
{
	struct tangox_otg		*dev;
	int		retval = 0;
	int irq;
	struct resource *res;
#ifdef CONFIG_TANGO3
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
#endif

#ifdef CONFIG_TANGO3
	if (((chip_id & 0xfff0) == 0x8670) || ((chip_id & 0xfff0) == 0x8680)) {
		if (controller > 0)
			controller = 1;
	} else
#endif
		controller  = 0;

	/* if you want to support more than one controller in a system,
	 * usb_gadget_driver_{register,unregister}() must change.
	 */
	if (the_controller) 
		return -EBUSY;

	irq = platform_get_irq(pdev, 0);
#ifdef CONFIG_TANGO3
	if (((chip_id == 0x8672) || (chip_id == 0x8674)) && (controller == 1))
		irq = IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_DVD_INT;
#endif
	if (irq <= 0) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	/* alloc, and start init */
	dev = kzalloc (sizeof *dev, GFP_KERNEL);
	if (dev == NULL) {
		retval = -ENOMEM;
		goto done;
	}

	spin_lock_init (&dev->lock);
	dev->pdev = pdev;
	dev->gadget.ops = &tangox_ops;
	dev->gadget.speed = USB_SPEED_HIGH;
	DEBUG("%s gadget.speed=0x%x high=0x%x full=0x%x  low=0x%x\n", 
		__FUNCTION__,dev->gadget.speed, USB_SPEED_HIGH, USB_SPEED_FULL, USB_SPEED_LOW);

	/* the "gadget" abstracts/virtualizes the controller */
	dev_set_name(&dev->gadget.dev, "gadget");
	dev->gadget.dev.parent = &pdev->dev;
	dev->gadget.dev.dma_mask = pdev->dev.dma_mask;
	dev->gadget.dev.release = gadget_release;
#ifdef CONFIG_TANGO3
	if (controller != 0)
		dev->gadget.name = TANGOX_UDC_NAME1;
	else
#endif
		dev->gadget.name = TANGOX_UDC_NAME0;

	dev->base =  (void *)ioremap(res->start, resource_size(res));
    dev->cap_regs   = (struct tangox_cap_regs   __iomem *) (dev->base + TANGOX_EHCI_REG_OFFSET);
    dev->regs       = (struct tangox_op_regs    __iomem *) (dev->base +  TANGOX_EHCI_REG_OFFSET+0x40);

	if (request_irq(irq, (void *)tangox_irq, IRQF_DISABLED, dev->gadget.name, dev)
			!= 0) {
		printk("request interrupt %d failed\n", irq);
		retval = -EBUSY;
		goto done;
	}

	/* init tangox_otg device */
	if (tangox_udc_init(dev, 0)) {
		retval =  -EFAULT;
		goto done;
	}

	dev->chiprev = ((readl(dev->base)>>16) & 0x00ff);

	printk ("IRQ %d, MEM 0x%x, Chip Rev. %04x\n",
		 irq, (u32)dev->base, dev->chiprev);
	the_controller = dev;
	platform_set_drvdata(pdev, dev);

	retval = device_register (&dev->gadget.dev);
	retval = device_create_file (&pdev->dev, &dev_attr_registers);
	//retval = device_add(&dev->gadget.dev);
	//if (retval) goto done;

	return 0;

done:
	if (dev)
		tangox_remove (pdev);

	return retval;
}

static int tangox_suspend(struct platform_device *dev, pm_message_t message)
{
	return 0;
}

static int tangox_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver tangox_udc_driver[] = {
	{
	.driver.name	=	(char *)TANGOX_UDC_NAME0,
	.driver.bus		=	&platform_bus_type,
	.probe			=	tangox_probe,
	.remove			=	tangox_remove,
	.suspend		=	tangox_suspend,
	.resume			=	tangox_resume,
	},
#ifdef CONFIG_TANGO3
	{
	.driver.name	=	(char *)TANGOX_UDC_NAME1,
	.driver.bus		=	&platform_bus_type,
	.probe			=	tangox_probe,
	.remove			=	tangox_remove,
	.suspend		=	tangox_suspend,
	.resume			=	tangox_resume,
	}
#endif
};



extern unsigned long tangox_otg_bits;
static int __init init(void)
{
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	if (((chip_id != 0x8652) && ((chip_id & 0xfff0) != 0x8670) && ((chip_id & 0xfff0) != 0x8680)) ||
		((chip_id == 0x8652) && (controller > 0))) {
		printk("Not supported chip(%x).\n", (unsigned int)chip_id);
		return -EIO;
	}

#ifdef CONFIG_TANGO3
	if (controller == 0) {
#endif
		if (test_and_set_bit(0, &tangox_otg_bits) != 0) {
			printk("Controller %d is used in different mode.\n", controller);
			return -EIO;
		}
#ifdef CONFIG_TANGO3
	} else if (((chip_id & 0xfff0) == 0x8670) || ((chip_id & 0xfff0) == 0x8680)) {
		if (test_and_set_bit(1, &tangox_otg_bits) != 0) {
			printk("Controller %d is used in different mode.\n", controller);
			return -EIO;
		}
	}
#endif

	printk("controller %d: %s, version: %s\n",controller, DRIVER_DESC,  DRIVER_VERSION);
	return platform_driver_register(&tangox_udc_driver[controller]);
}

static void __exit cleanup (void)
{
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	platform_driver_unregister(&tangox_udc_driver[controller]);

	if (controller == 0) 
		clear_bit(0, &tangox_otg_bits);
	else if (((chip_id & 0xfff0) == 0x8670) || ((chip_id & 0xfff0) == 0x8680)) 
		clear_bit(1, &tangox_otg_bits);

	tangox_phy_power_down(controller);
}

module_init (init);
module_exit (cleanup);

MODULE_DESCRIPTION (DRIVER_DESC);
MODULE_AUTHOR ("Sigma Designs");
MODULE_LICENSE ("GPL");
