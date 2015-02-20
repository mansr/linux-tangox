/*********************************************************************
 Sigma Designs, Inc. 
 2001-2009

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
#include <linux/usb_gadget.h>
#include <linux/platform_device.h>
#include "tango3_udc.h"
#include "../core/tangox-usb.h"

#define	DRIVER_DESC		"TANGOX 8652 USB Peripheral Controller"
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


static const char driver_name [] = "tango3_otg";
static const char driver_desc [] = DRIVER_DESC;
static struct tango3_otg	*the_controller;
static const struct usb_ep_ops tango3_ep_ops;
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
static void dump_registers(struct tango3_otg *dev, int yes)
{
	if(yes){
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
static void dump_dqh(struct ep_qh * dqh, int loc)
{
	printk(".....................dumping dqh........at loc=0x%x\n", loc);
	printk("dqh=0x%x\n", (u32)dqh);
	printk("dqh->max_pktlen=0x%x\n", dqh->max_pktlen);
	printk("dqh->curr_dtdptr=0x%x\n", dqh->curr_dtdptr);
	if(dqh->curr_dtdptr != VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE){
		dump_dtd((struct dtd *)KSEG1ADDR(tangox_inv_dma_address((u32)dqh->curr_dtdptr)));
	}
	printk("dqh->next_dtdptr=0x%x\n", dqh->next_dtdptr);
	if(dqh->next_dtdptr != VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE){
		dump_dtd((struct dtd *)KSEG1ADDR(tangox_inv_dma_address((u32)dqh->next_dtdptr)));
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
	for (i= 0;  i< len; i++){
	       if(i%16==0 && i!=0)
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
static void dump_req(struct tango3_request *req)
{

	if(req){
		printk("req=0x%x\n", (u32)req);
		printk("req->queue=0x%x\n",(u32)&req->queue);
		printk("req->td_dam=0x%x\n", req->td_dma);
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
	}else
		printk("req ==  null\n");
}

static void dump_ep_queues(struct tango3_otg *dev)
{
	int i = 0;
	struct tango3_ep       *ep;
    struct tango3_request   *r;

	for (i=0; i< 3; i++){
		ep = &dev->ep[i];
        if (!list_empty (&ep->queue)) {
			printk("dumping ep[%d]->queue... \n", i);
            list_for_each_entry (r, &ep->queue, queue){
				dump_req(r);
            }
        }else
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
		if(i == MAX_EP_TR_DESCRS )
			break;
	}
	return;
}
#else
#define dump_registers(dev, yes)	do { } while (0)
#define dump_dtd(dtd)				do { } while (0)
#define dump_dqh(dqh,loc)			do { } while (0)
#define dump_usb_ep_desc(desc)		do { } while (0)
#define dump_req(req)				do { } while (0)
//#define dump_ctrlrequest(cr)		do { } while (0)
//#define dump_mem(buff, len)		do { } while (0)
//#define dump_ep_queues(dev)		do { } while (0)
//#define dump_list(list)			do { } while (0)
#endif



static void ep_reset (struct tango3_op_regs __iomem *regs, struct tango3_ep *ep)
{
	ep->desc = NULL;
	INIT_LIST_HEAD (&ep->queue);

	ep->ep.maxpacket = ~0;
	ep->ep.ops = &tango3_ep_ops;
}

static void usb_reset (struct tango3_otg *dev)
{
    DEBUG("%s\n", __FUNCTION__);
    INIT_LIST_HEAD (&dev->gadget.ep_list);
    list_add_tail (&dev->ep [1].ep.ep_list, &dev->gadget.ep_list);
    list_add_tail (&dev->ep [2].ep.ep_list, &dev->gadget.ep_list);
    list_add_tail (&dev->ep [3].ep.ep_list, &dev->gadget.ep_list);
    list_add_tail (&dev->ep [4].ep.ep_list, &dev->gadget.ep_list);
}

static void usb_reinit (struct tango3_otg *dev)
{
	u32	tmp;

	/* basic endpoint init */
	for (tmp = 0; tmp < 4; tmp++) {
		struct tango3_ep	*ep = &dev->ep [tmp];

		ep->ep.name = ep_name [tmp];
		ep->dev = dev;
		ep->num = tmp;

		ep->regs = &dev->regs [tmp];
		ep_reset (dev->regs, ep);
	}
	dev->ep [0].ep.maxpacket = 64;

	dev->gadget.ep0 = &dev->ep [0].ep;
	dev->ep [0].stopped = 0;
	INIT_LIST_HEAD (&dev->gadget.ep0->ep_list);
}

static inline void qh_init(struct tango3_otg *dev)
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


static int get_free_dtd_count(struct tango3_otg *dev, struct list_head *list)
{
	struct list_head        *tmp;
	int i = 0;

	list_for_each (tmp, list){ 
        i++;
		if(i== MAX_EP_TR_DESCRS )
			break;
 	}
	return i;
}

//static DEFINE_SPINLOCK(dtd_lock);
static inline void dtd_init (struct tango3_otg *dev, struct dtd  *dtd)
{

     memset (dtd, 0, sizeof *dtd);
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

static struct dtd *get_dtd(struct tango3_otg *dev, struct list_head *dtd_list)
{
	struct dtd *dtd = NULL;
	
	if(!list_empty(dtd_list)){	
		dtd = (struct dtd *)dtd_list->next;
		list_del((struct list_head *)dtd);
	}
#if 0 
	DEBUG("%s dtd=0x%x left=%d\n", 	__FUNCTION__, (u32)dtd, 
		(u32)get_free_dtd_count(dev, &dtd_free_list));
	dump_list(&dtd_free_list);
#endif
	return dtd;	  
}


static void free_dtd (struct tango3_otg *dev, struct dtd *dtd, int lock)
{
	u32 addr;

	addr = (u32)dtd;
	if(addr<=CPU_REMAP_SPACE)
		return;

	if(dtd->size_ioc_sts & VUSBHS_TD_STATUS_ACTIVE)
		return;
	dtd_init(dev,dtd);
	list_add_tail((struct list_head *)dtd, &dtd_free_list);
#if 0 
	DEBUG("%s dtd=0x%x left = %d\n",__FUNCTION__, (u32)dtd, 
		(u32)get_free_dtd_count(dev, &dtd_free_list));
	dump_list(&dtd_free_list);
#endif
}

#if 0
static void free_prev_dtd(struct tango3_otg *dev, struct ep_qh * dqh)
{
	struct dtd *dtd = 0;
	u32 addr= (u32)dqh->curr_dtdptr;

	if((addr != VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE) && tangox_valid_dma_addr(addr)){
		dtd = (struct dtd *)KSEG1ADDR(tangox_inv_dma_address(dqh->curr_dtdptr));
	    free_dtd(dev, dtd, 0);
	}
}
#endif

/*check all dtds and free those if possible*/
static void retire_dtd( struct tango3_otg *dev)
{
	struct dtd *dtd;
    struct list_head        *tmp;
    int i=0 , free;

	dtd = dev->dtd_ptr;

    for (i=0;i<MAX_EP_TR_DESCRS;i++) {
		free = 1;
       	list_for_each (tmp, &dtd_free_list) {
			if( (unsigned long) dtd == (unsigned long)tmp){
				free = 0; 
				break;
			}				
		}
		
		if(free && (!(dtd->size_ioc_sts & VUSBHS_TD_STATUS_ACTIVE)))
			free_dtd(dev, dtd, 0);

		dtd++;
	}
}


static void dev_mem_cleanup (struct tango3_otg *dev)
{
	u32 mem_size;

    mem_size = ((16 * sizeof(struct ep_qh)) + 64 ) +
                ((MAX_EP_TR_DESCRS * sizeof(struct dtd)) + 64);
	dma_free_coherent((struct device *)dev, mem_size, dev->ep_qhptr, 0);
}

static dma_addr_t dev_mem_init (struct tango3_otg *dev)
{
	dma_addr_t dma;
    void    *temp_ptr;
    u32     mem_size=0;
    unsigned char* drv_mem;

    dev->max_eps = dev->cap_regs->dccparams & 0x1f;

    /* calculate the total mem size */
    mem_size = ((dev->max_eps*2 * sizeof(struct ep_qh)) + 64 ) +
     	((MAX_EP_TR_DESCRS * sizeof(struct dtd)) + 64);

	DEBUG("max_eps=0x%x size of qh=0x%x size of dtd=0x%x size of list_head=0x%x\n", 
		dev->max_eps, sizeof(struct ep_qh), sizeof(struct dtd), sizeof(struct list_head));

	/* allocate memory*/
    drv_mem = dma_alloc_coherent(&dev->pdev->dev, mem_size, &dma, GFP_KERNEL | GFP_DMA);
    if (drv_mem == NULL) {
            DEBUG("%s %s\n", __FUNCTION__, "memory allocation  failed");
            return -ENOMEM;
    }

    /* QH base*/
    temp_ptr = (struct ep_qh *)drv_mem;
    dev->ep_qhptr = (struct ep_qh *)MEM2048_ALIGN((u32)temp_ptr);
    DEBUG("ep_qhptr=0x%x\n",  (u32)dev->ep_qhptr);


    /* DTD base */
    drv_mem += ((dev->max_eps*2 * sizeof(struct ep_qh)) + 64);
    temp_ptr = (struct dtd *)drv_mem;
    dev->dtd_ptr = (struct dtd *)MEM32_ALIGN((u32)temp_ptr);
    DEBUG("dtd_ptr =0x%x\n",  (u32)dev->dtd_ptr);

	/* init dtd free list*/
    INIT_LIST_HEAD (&dtd_free_list);

	return dma;
}


static int tango3_enable (struct usb_ep *_ep, 
				const struct usb_endpoint_descriptor *desc)
{
	struct tango3_otg		*dev;
	struct tango3_ep		*ep;
    struct ep_qh			*ep_qhptr;
	unsigned long			flags, which_bit;
	u32	max, tmp;
    u8  direction, type;
	int i,ret = -1;
	
	dump_usb_ep_desc(desc);
	ep = container_of (_ep, struct tango3_ep, ep);
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
	for(i = 0; i < 2; i++){
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

static void nuke (struct tango3_ep *);

static int tango3_disable (struct usb_ep *_ep)
{
	struct tango3_ep	*ep;
	unsigned long		flags;

	ep = container_of (_ep, struct tango3_ep, ep);
	if (!_ep || !ep->desc || _ep->name == ep_name[0])
		return -EINVAL;

	spin_lock_irqsave (&ep->dev->lock, flags);
	nuke (ep);
	ep_reset (ep->dev->regs, ep);

	spin_unlock_irqrestore (&ep->dev->lock, flags);
	return 0;
}


static struct usb_request *
tango3_alloc_request (struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct tango3_ep	*ep;
	struct tango3_request	*req;

	if (!_ep)
		return NULL;
	ep = container_of (_ep, struct tango3_ep, ep);

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return NULL;
	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD (&req->queue);

	return &req->req;
}

static void tango3_free_request (struct usb_ep *_ep, 
						struct usb_request *_req)
{
	struct tango3_ep	*ep;
	struct tango3_request	*req;

	ep = container_of (_ep, struct tango3_ep, ep);
	if (!_ep || !_req)
		return;

	req = container_of (_req, struct tango3_request, req);
	WARN_ON (!list_empty (&req->queue));
	kfree (req);
}


/*
 * dma-coherent memory allocation (for dma-capable endpoints)
 *
 * NOTE: the dma_*_coherent() API calls suck.  Most implementations are
 * (a) page-oriented, so small buffers lose big; and (b) asymmetric with
 * respect to calls with irqs disabled:  alloc is safe, free is not.
 * We currently work around (b), but not (a).
 */

static void *tango3_alloc_buffer (	struct usb_ep		*_ep,
										unsigned		bytes,
										dma_addr_t		*dma,
										gfp_t			gfp_flags )
{
	void			*retval;
	struct tango3_ep	*ep;

	ep = container_of (_ep, struct tango3_ep, ep);
	if (!_ep)
		return NULL;
	*dma = DMA_ADDR_INVALID;

	retval = dma_alloc_coherent(&ep->dev->pdev->dev,
			bytes, dma, gfp_flags);
	return retval;
}

static DEFINE_SPINLOCK(buflock);
static LIST_HEAD(buffers);

struct free_record {
	struct list_head	list;
	struct device		*dev;
	unsigned		bytes;
	dma_addr_t		dma;
};

static void do_free(unsigned long ignored)
{
	spin_lock_irq(&buflock);
	while (!list_empty(&buffers)) {
		struct free_record	*buf;

		buf = list_entry(buffers.next, struct free_record, list);
		list_del(&buf->list);
		spin_unlock_irq(&buflock);

		dma_free_coherent(buf->dev, buf->bytes, buf, buf->dma);

		spin_lock_irq(&buflock);
	}
	spin_unlock_irq(&buflock);
}

static DECLARE_TASKLET(deferred_free, do_free, 0);

static void tango3_free_buffer ( struct usb_ep *_ep,
									void *address,
									dma_addr_t dma,
									unsigned bytes) 
{
	/* free memory into the right allocator */
	if (dma != DMA_ADDR_INVALID) {
		struct tango3_ep	*ep;
		struct free_record	*buf = address;
		unsigned long		flags;

		ep = container_of(_ep, struct tango3_ep, ep);
		if (!_ep)
			return;

		ep = container_of (_ep, struct tango3_ep, ep);
		buf->dev = &ep->dev->pdev->dev;
		buf->bytes = bytes;
		buf->dma = dma;

		spin_lock_irqsave(&buflock, flags);
		list_add_tail(&buf->list, &buffers);
		tasklet_schedule(&deferred_free);
		spin_unlock_irqrestore(&buflock, flags);
	} else
		kfree (address);
}



/* check when an ep is done*/
static void
tango3_done (struct tango3_ep *ep, 
				struct tango3_request *req, int status)
{
	struct tango3_otg	*dev;
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

	if(ep_qhptr->curr_dtdptr == VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE) 
		return;

    dtd_ptr = (struct dtd *)KSEG1ADDR(tangox_inv_dma_address(ep_qhptr->curr_dtdptr));
	//printk("\n%s curr_dtd_ptr=0x%x dtd_ptr=0x%x\n", 
	//			__FUNCTION__, ep_qhptr->curr_dtdptr, dtd_ptr);
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
		next_dtd_ptr =dtd_ptr->next_link_ptr;

		/* Read the errors */
		errors = (dtd_ptr->size_ioc_sts & VUSBHS_TD_ERROR_MASK);
		if (!errors) {
			/* Get the length of transfer from the current dTD */
			remain = ((dtd_ptr->size_ioc_sts & VUSB_EP_TR_PACKET_SIZE) >> 16);
		    tx_size = _req->length - remain - actual;
			actual = _req->length - remain; 
		} else {
			DEBUG("**********************errors=0x%x\n", errors);
			if (errors & VUSBHS_TD_STATUS_HALTED) {
                /* Clear the errors and Halt condition */
                ep_qhptr->size_iocintsts &= ~errors;
				dump_dqh(ep_qhptr, 9);
			}
		}

		/* Retire the dtd */
		free_dtd(dev, dtd_ptr,1);

		if(next_dtd_ptr == VUSBHS_TD_NEXT_TERMINATE)
			break;

		 dtd_ptr = (struct dtd *)KSEG1ADDR(tangox_inv_dma_address(next_dtd_ptr));
    }

	_req->actual = actual;		

	/* see if there is an error */
	if(errors && _req->actual != _req->length){
       	DEBUG("errors=0x%x req length = 0x%x actual=0x%x \n", 
			errors, _req->length, _req->length);
		/* resubmition or halt */
	} 

	//check and retire
	if((u32)get_free_dtd_count(dev, &dtd_free_list) < 10)
		retire_dtd(dev);

	spin_unlock (&dev->lock);
	req->req.complete (&ep->ep, &req->req);
	spin_lock (&dev->lock);
	//ep->stopped = stopped;
	
	DEBUG("%s req=0x%x done\n", __FUNCTION__,  (u32)_req);
}

#if 0
/* to unstall ep_num = 1 or 2*/
static void ep_unstall(struct tango3_otg *dev, int ep_num )
{ 
   dev->regs->ep_ctrl[ep_num] &= (ep_num == 1 ? 
		~EHCI_EPCTRL_TX_EP_STALL : ~EHCI_EPCTRL_RX_EP_STALL);

}
#endif

static int tango3_ep_irq (struct tango3_otg *dev);
static int tango3_completion(struct tango3_otg *dev);
static int tango3_queue (struct usb_ep *_ep, struct usb_request *_req, 
							gfp_t gfp_flags)
{
	struct tango3_request	*req = 0;
	struct tango3_ep		*ep = 0;
	struct tango3_otg		*dev = 0;
	struct ep_qh	*ep_qhptr = 0;
	struct dtd		*dtd_ptr = 0;
	struct dtd		*tmp_dtd_ptr = 0;
	struct dtd		*first_dtd_ptr = 0;
	unsigned long	flags;
	int prime = -1, need_prime = 0, i=0;
	u32 temp_ep_stat= 0;
	u32 cur_len, remaining, dma_addr;

	req = container_of (_req, struct tango3_request, req);
	if (!_req || !_req->complete || !_req->buf
			|| !list_empty (&req->queue))
		return -EINVAL;

	ep = container_of (_ep, struct tango3_ep, ep);
	if (!_ep || (!ep->desc && ep->num != 0))
		return -EINVAL;

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	/*set address's length=0*/
	if (_req->length == 0 && ep->num!=0)
		return -EOPNOTSUPP;

	if(ep->num == 1)
		ep->is_in = 0;
	else if(ep->num == 2)
		ep->is_in = 1;
#if 0
	DEBUG("******%s ep->num=%d len=0x%x req=0x%x*******\n",
		__FUNCTION__, ep->num, _req->length, (u32)req );
	DEBUG("req=0x%x len=0x%x ep->num=%d\n",(u32)req, _req->length, ep->num);
#endif	
	ep_qhptr = dev->ep_qhptr + ep->num*2 +(ep->is_in ? 0: 1);
	prime = (!ep->is_in ? 16 + ep->num : ep->num);

	while(!list_empty(&ep->queue)){
		udelay(1);
	}
	spin_lock_irqsave (&dev->lock, flags);
	dma_addr  = _req->dma;
	remaining = _req->length;
	do {
		if (remaining > VUSB_EP_MAX_LENGTH_TRANSFER) 
			cur_len = VUSB_EP_MAX_LENGTH_TRANSFER;
		else 
			cur_len = remaining;

      	remaining -= cur_len;
		i++;

		dtd_ptr = get_dtd( dev, &dtd_free_list );
		if(!dtd_ptr){
			printk("FIXME 2: cann't get dtd, retry\n");
			spin_unlock_irqrestore (&dev->lock, flags);
			return -ENOMEM;
		}
		dtd_ptr->buff_ptr0 = dma_addr;
		dtd_ptr->buff_ptr1 = dma_addr + 1*4096;
		dtd_ptr->buff_ptr2 = dma_addr + 2*4096;
		dtd_ptr->buff_ptr3 = dma_addr + 3*4096;
		dtd_ptr->buff_ptr4 = dma_addr + 4*4096;
		dtd_ptr->size_ioc_sts = ((cur_len << VUSBHS_TD_LENGTH_BIT_POS) |
									(VUSBHS_TD_STATUS_ACTIVE));
		if(!remaining)
                	dtd_ptr->size_ioc_sts |= VUSBHS_TD_IOC;
		else
			dma_addr += cur_len;

		dtd_ptr->size_ioc_sts &= ~VUSBHS_TD_RESERVED_FIELDS;
		dtd_ptr->next_link_ptr = VUSBHS_TD_NEXT_TERMINATE;

		if(tmp_dtd_ptr) 
			tmp_dtd_ptr->next_link_ptr = PHYSADDR(dtd_ptr); 

		tmp_dtd_ptr = dtd_ptr;

		if(i==1)
			first_dtd_ptr = dtd_ptr;
		
	} while (remaining);

	spin_unlock_irqrestore (&dev->lock, flags);
again:
    if (list_empty (&ep->queue)){
		if(ep_qhptr->next_dtdptr == VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE){
			ep_qhptr->next_dtdptr = PHYSADDR((void *)first_dtd_ptr);
		}
		else 
			printk("oops ep_qhptr->next_dtdptr=0x%x\n", ep_qhptr->next_dtdptr);

	    ep_qhptr->size_iocintsts = 0;
		need_prime = 1;
	} else {
		if(dev->regs->ep_prime != 1<< prime){
			while(1){
				dev->regs->usbcmd |= EHCI_CMD_SETUP_TRIPWIRE_SET;
				temp_ep_stat = dev->regs->ep_stat;
				if(dev->regs->usbcmd & EHCI_CMD_SETUP_TRIPWIRE_SET){
					break;
				}
			}

			dev->regs->usbcmd &= EHCI_CMD_SETUP_TRIPWIRE_CLEAR;
       
	       	if(!temp_ep_stat){
				if(ep_qhptr->next_dtdptr == VUSB_EP_QUEUE_HEAD_NEXT_TERMINATE)
					ep_qhptr->next_dtdptr = PHYSADDR((void *)first_dtd_ptr);
				else 
					printk("oops ep_qhptr->next_dtdptr=0x%x\n", ep_qhptr->next_dtdptr);

		        ep_qhptr->size_iocintsts = 0;
				need_prime = 1;
			}else
				goto again;
		
		}else
			goto again;
	}

    /* there may be a prev dtd need to free */
    //free_prev_dtd(dev, ep_qhptr);

    if (req)
        list_add_tail (&req->queue, &ep->queue);
    /* check and prime the ep */
	if(need_prime)
	        dev->regs->ep_prime |= (1 << prime);

	DEBUG("** %s done *************\n\n",__FUNCTION__);
	return 0;
}

/* dequeue ALL requests */
static void nuke (struct tango3_ep *ep)
{
	struct tango3_request	*req;

	/* called with spinlock held */
	ep->stopped = 1;
	while (!list_empty (&ep->queue)) {
		req = list_entry (ep->queue.next,
				struct tango3_request,
				queue);
		tango3_done (ep, req, -ESHUTDOWN);
	}
}

/* dequeue JUST ONE request */
static int tango3_dequeue (struct usb_ep *_ep, struct usb_request *_req)
{
	int	stopped;
	struct tango3_ep		*ep;
	struct tango3_request	*req;
	unsigned long			flags;

	DEBUG("%s\n", __FUNCTION__);
	ep = container_of (_ep, struct tango3_ep, ep);
	if (!_ep || (!ep->desc && ep->num != 0) || !_req)
		return -EINVAL;

	spin_lock_irqsave (&ep->dev->lock, flags);
	stopped = ep->stopped;

	/* quiesce dma while we patch the queue */
	ep->stopped = 1;

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
		tango3_done (ep, req, -ECONNRESET);

    ep->stopped = stopped;

	spin_unlock_irqrestore (&ep->dev->lock, flags);
	return 0;
}


static int tango3_set_halt (struct usb_ep *_ep, int value)
{
	struct tango3_ep	*ep;
	unsigned long		flags;
	int			retval = 0;

	DEBUG("%s...\n", __FUNCTION__);
	ep = container_of (_ep, struct tango3_ep, ep);
	if (!_ep || (!ep->desc && ep->num != 0))
		return -EINVAL;
	if (!ep->dev->driver || ep->dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	spin_lock_irqsave (&ep->dev->lock, flags);
	if (!list_empty (&ep->queue))
		retval = -EAGAIN;
	else if (ep->is_in && value )
		retval = -EAGAIN;
	else {
		DEBUG ("%s %s halt\n", _ep->name,
				value ? "set" : "clear");

		/* set/clear, then synch memory views with the device */
		if (value) {
			if (ep->num == 0)
				ep->dev->protocol_stall = 1;
			else
				set_halt (ep);
		} else
			clear_halt (ep);
	}
	spin_unlock_irqrestore (&ep->dev->lock, flags);

	return retval;
}

static int tango3_fifo_status (struct usb_ep *_ep)
{
	struct tango3_ep	*ep;

	ep = container_of (_ep, struct tango3_ep, ep);
	if (!_ep || (!ep->desc && ep->num != 0))
		return -ENODEV;
	if (!ep->dev->driver || ep->dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	return -EOPNOTSUPP;
}

static void tango3_fifo_flush (struct usb_ep *_ep)
{
	struct tango3_ep	*ep;
	struct tango3_otg           *dev = 0;
	int bit_pos;

	ep = container_of (_ep, struct tango3_ep, ep);
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

static const struct usb_ep_ops tango3_ep_ops = {
	.enable		= tango3_enable,
	.disable	= tango3_disable,

	.alloc_request	= tango3_alloc_request,
	.free_request	= tango3_free_request,

	.alloc_buffer	= tango3_alloc_buffer,
	.free_buffer	= tango3_free_buffer,

	.queue		= tango3_queue,
	.dequeue	= tango3_dequeue,

	.set_halt	= tango3_set_halt,
	.fifo_status	= tango3_fifo_status,
	.fifo_flush	= tango3_fifo_flush,
};


static int tango3_get_frame (struct usb_gadget *_gadget)
{
	DEBUG("%s\n", __FUNCTION__);
	return -EOPNOTSUPP;
}

static int tango3_wakeup (struct usb_gadget *_gadget)
{
	DEBUG("%s\n", __FUNCTION__);
	return 0;
}

static int tango3_set_selfpowered (struct usb_gadget *_gadget, int value)
{
	DEBUG("%s\n", __FUNCTION__);
	return 0;
}

static int tango3_pullup(struct usb_gadget *_gadget, int is_on)
{

	DEBUG("%s\n", __FUNCTION__);
	return 0;
}

static const struct usb_gadget_ops tango3_ops = {
	.get_frame	= tango3_get_frame,
	.wakeup		= tango3_wakeup,
	.set_selfpowered = tango3_set_selfpowered,
	.pullup		= tango3_pullup,
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
	struct tango3_otg	*dev = dev_get_drvdata (_dev);

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
	struct tango3_otg		*dev;
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

static void dtd_list_init(struct tango3_otg *dev)
{
	struct dtd* dtd_ptr = 0;
	int i;

	if(!dev->dtd_ptr)
		return;

	while (!list_empty(&dtd_free_list)) {
		struct dtd *dtd_ptr;
		dtd_ptr = list_entry(dtd_free_list.next, struct dtd, dtd_list);
		if(&dtd_ptr->dtd_list)
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

static int tango3_udc_init(struct tango3_otg *dev, u8 inited)
{
	dma_addr_t  dma_hdl = 0;
    unsigned long temp;

	DEBUG("%s TangoX USB initializing...\n", __FUNCTION__);

	/*0. set bit 1 of USB control register 0x21700*/ 
	temp = gbus_read_uint32(pGBus, TANGOX_USB_CTL_STATUS_REG_BASE + 0x0);
    gbus_write_uint32(pGBus, TANGOX_USB_CTL_STATUS_REG_BASE + 0x0, temp | 0x2);
	 wait_ms(5);
 
	/* 1. Program the clean divider and clock multiplexors to provide 48MHz clock reference*/
	/* this is to be done in zboot */

	/* 2. Enable the USB PHY */
	temp = gbus_read_uint32(pGBus,TANGOX_USB_CTL_STATUS_REG_BASE + 0x0);
	gbus_write_uint32(pGBus,TANGOX_USB_CTL_STATUS_REG_BASE + 0x0, temp & 0xffffff7e);
	wait_ms(20);

	/* 3. Enable the USB Host EHCI/OHCI */
	temp = gbus_read_uint32(pGBus, TANGOX_USB_CTL_STATUS_REG_BASE + 0x0);
	gbus_write_uint32(pGBus, TANGOX_USB_CTL_STATUS_REG_BASE + 0x0, temp & 0xfffffffd);
	wait_ms(20);

	/* 4. set it to dev mode*/
	temp = gbus_read_uint32(pGBus, TANGOX_EHCI_BASE_ADDR + TANGOX_EHCI_REG_OFFSET +0xA8);
	temp |= 2 ;
	gbus_write_uint32(pGBus, TANGOX_EHCI_BASE_ADDR + TANGOX_EHCI_REG_OFFSET +0xA8, temp);
	wait_ms(20);

	DEBUG("%s TangoX USB initializing...done\n", __FUNCTION__);

	if( !inited){
		dma_hdl = dev_mem_init (dev); 
		if(!dma_hdl)
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
int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
	struct tango3_otg		*dev = the_controller;
	int			retval;
	unsigned		i;

	/* insist on high speed support from the driver, since
	 * (dev->usb->xcvrdiag & FORCE_FULL_SPEED_MODE)
	 * "must not be used in normal operation"
	 */
	DEBUG("%s\n", __FUNCTION__);
	if (!driver
			/*|| driver->speed != USB_SPEED_HIGH*/
			|| !driver->bind
			|| !driver->setup){
		return -EINVAL;
	}
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	for (i = 0; i < 4; i++)
		dev->ep [i].irqs = 0;

	/* hook up the driver ... */
	dev->softconnect = 1;
	driver->driver.bus = NULL;
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;
//	device_add(&dev->gadget.dev);
	retval = driver->bind (&dev->gadget);
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
EXPORT_SYMBOL (usb_gadget_register_driver);

static void
stop_activity (struct tango3_otg *dev, struct usb_gadget_driver *driver)
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
	struct tango3_otg	*dev = the_controller;
	unsigned long	flags;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver || !driver->unbind)
		return -EINVAL;

	spin_lock_irqsave (&dev->lock, flags);
	stop_activity (dev, driver);
	spin_unlock_irqrestore (&dev->lock, flags);

	tango3_pullup (&dev->gadget, 0);
	driver->unbind (&dev->gadget);
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

int get_ep_num(struct tango3_otg *dev, unsigned long stat)
{
	int i;
	int ep = -1;
	if(stat){
		for(i = 0; i < 32; i++){
			if(((stat >> i) & 1) == 1){
				ep = i;
				break;
			}
		}	
	}
	return ep;
}

static int tango3_completion(struct tango3_otg *dev)
{
    struct tango3_ep        *ep = 0;
	struct tango3_request 	*req= 0;
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

	if(!completed)
		return 0;

    DEBUG("%s completion %04x\n", __FUNCTION__, completed);
	/* check though each complete bit in cbs[]*/
    for(i=0; i< 4; i++){
		if(((completed >> cbs[i]) & 0x1) == 1)
            ep_num = cbs[i];
        else
            continue;

        if(ep_num <= 15){
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
			struct tango3_request * tmp = 0 ;
            list_for_each_entry (req, &ep->queue, queue){
				//dump_req(req);
				if( tmp == req)
					break;

				tango3_done (ep, req, 0);
				tmp = req;
				done = 1;
            }
        }

		if((ep_num == 0) && (direction == 1)){	
	        /* if it's an in packet, need a response */
			/* get a dtd from qh dtd free list*/
			dtd_ptr = get_dtd( dev, &dtd_free_list );
			if(!dtd_ptr){
				printk("FIXME 3: cann't get dtd, retry\n");
				return -1;
			}
	        dtd_ptr->size_ioc_sts = (0 << VUSBHS_TD_LENGTH_BIT_POS) |
    	                           (VUSBHS_TD_IOC) | (VUSBHS_TD_STATUS_ACTIVE);
	        dtd_ptr->size_ioc_sts &= ~VUSBHS_TD_RESERVED_FIELDS;
            dtd_ptr->next_link_ptr = VUSBHS_TD_NEXT_TERMINATE;

	        ep_qhptr = dev->ep_qhptr;
	        ep_qhptr->next_dtdptr = PHYSADDR((void *)dtd_ptr);
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


static int tango3_ep_irq (struct tango3_otg *dev)
{
	struct tango3_ep	*ep = 0;
	struct ep_qh		*ep_qhptr;
	struct usb_ctrlrequest  *cr = NULL;
	int	tmp = 0;
	u32	ep_num = 0, setup_stat=0;

	/* try to get the setup buffer asap*/
	setup_stat = dev->regs->ep_setupstat;
	if(setup_stat){
		ep_num = get_ep_num(dev,  setup_stat);
		DEBUG("--%s ep_setupstat %04x ep_num=0x%x---\n",
		 __FUNCTION__, setup_stat, ep_num);
		if(ep_num == 0){

			cr = kzalloc(sizeof(struct usb_ctrlrequest), GFP_NOIO);
			memset(cr, 0, sizeof(struct usb_ctrlrequest));

			ep_qhptr = dev->ep_qhptr +2*ep_num + 0;   
			/* Clear the bit in the ENDPTSETUPSTAT */
			dev->regs->ep_setupstat =  setup_stat;
			while(1){
				/*setup tripwire*/
				dev->regs->usbcmd |= EHCI_CMD_SETUP_TRIPWIRE_SET;
				memcpy(cr, (unsigned char *)ep_qhptr->setup_buff, 8);

				if(dev->regs->usbcmd & EHCI_CMD_SETUP_TRIPWIRE_SET){
					dev->regs->usbcmd &= EHCI_CMD_SETUP_TRIPWIRE_CLEAR;
					break;
				}
			}
		}else {
			printk("wrong endpoint for setup packet: ep_num=0x%x", ep_num);
			return -1;
		}
	}

	/* precess those completed dtds*/
	tango3_completion(dev);

	if(!setup_stat){
        	DEBUG("--setup packet--%s ep_setupstat %04x done--\n\n",
			 __FUNCTION__, setup_stat);
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
		dev->protocol_stall = 1;
	}
out:
	if(cr)
		kfree(cr);

    //DEBUG("--setup packet--%s ep_setupstat %04x done---\n\n",
	//	 __FUNCTION__, setup_stat);
	return 0;
}


static void tango3_dev_reset(struct tango3_otg   *dev)
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
		tango3_udc_init (dev, 1);
	}
#endif	
	return;
}

static void tango3_dev_error(struct tango3_otg   *dev)
{
        struct ep_qh            *ep_qhptr = 0;
	struct tango3_ep        *ep = 0;
	int i;

	dump_registers(dev, 1);
	for(i = 0; i < 3; i++){
		ep =  &dev->ep[i];
		printk("ep[%d] ep->is_in=0%d\n", i, ep->is_in);	
        	ep_qhptr = dev->ep_qhptr + ep->num*2 + (!ep->is_in ? 1 : 0);
		dump_dqh(ep_qhptr, 8);
	}
}

static irqreturn_t tango3_irq (int irq, void *_dev)
{
	struct tango3_otg	*dev = _dev;
	u32             stat = 0; 
	u32		intr = 0;

	intr = dev->regs->usbintr;
	stat = dev->regs->usbsts;

	if(!(intr & stat))
		return IRQ_NONE;

	spin_lock (&dev->lock);
  	/* disable intrrupts*/
	//dev->regs->usbintr = 0;

	/* Clear all the interrupts occured */
	dev->regs->usbsts =  stat;

	if(stat & EHCI_STS_INT){ 
		tango3_ep_irq (dev);
	}
	
	if (stat & EHCI_STS_RESET) {
		tango3_dev_reset(dev);
	}
	if (stat & EHCI_STS_PORT_CHANGE) {
   		//tango3_dev_port_change(dev);
	}
	if (stat & EHCI_STS_ERR) {
   		tango3_dev_error(dev);
	}

	if(stat & EHCI_STS_SUSPEND) {
		//tango3_dev_suspend(dev);
	}
	if(stat & EHCI_STS_SOF) {
   		//tango3_dev_SOF(dev);
	}

 	/* enable intrrupts*/
	//dev->regs->usbintr = intr;
	spin_unlock (&dev->lock);

	return IRQ_HANDLED;
}


static void gadget_release (struct device *_dev)
{
	struct tango3_otg	*dev = dev_get_drvdata (_dev);
	kfree (dev);
}

/* tear down the binding between this driver and the device */
static int tango3_remove (struct platform_device *pdev)
{
	struct tango3_otg *dev = platform_get_drvdata(pdev);

	BUG_ON(dev->driver);

	dev_mem_cleanup(dev);

	free_irq (TANGOX_EHCI_IRQ, dev);

	usb_gadget_unregister_driver(dev->driver);
	//device_unregister (&dev->gadget.dev);
	device_remove_file (&pdev->dev, &dev_attr_registers);

	printk("unbind\n");
	platform_set_drvdata(pdev, 0);
	the_controller = NULL;
	return 0;
}

/* wrap this driver around the specified device, but
 * don't respond over USB until a gadget driver binds to us.
 */
static int tango3_probe (struct platform_device *pdev)
{
	struct tango3_otg		*dev;
	void	__iomem *base = NULL;
	int		retval = 0;

	/* if you want to support more than one controller in a system,
	 * usb_gadget_driver_{register,unregister}() must change.
	 */
	if (the_controller) {
		//dev_warn (&pdev->dev, "ignoring\n");
		return -EBUSY;
	}

	/* alloc, and start init */
	dev = kzalloc (sizeof *dev, GFP_KERNEL);
	if (dev == NULL){
		retval = -ENOMEM;
		goto done;
	}

	spin_lock_init (&dev->lock);
	dev->pdev = pdev;
	dev->gadget.ops = &tango3_ops;
	dev->gadget.is_dualspeed = 1;
	dev->gadget.speed = USB_SPEED_HIGH;
	DEBUG("%s gadget.speed=0x%x high=0x%x full=0x%x  low=0x%x\n", 
		__FUNCTION__,dev->gadget.speed, USB_SPEED_HIGH, USB_SPEED_FULL, USB_SPEED_LOW);

	/* the "gadget" abstracts/virtualizes the controller */
	strcpy (dev->gadget.dev.bus_id, "gadget");
	dev->gadget.dev.parent = &pdev->dev;
	dev->gadget.dev.dma_mask = pdev->dev.dma_mask;
	dev->gadget.dev.release = gadget_release;
	dev->gadget.name = driver_name;
	dev->enabled = 1;
	dev->region = 1;

    base =  (void *)NON_CACHED(TANGOX_EHCI_BASE_ADDR);
    dev->cap_regs   = (struct tango3_cap_regs   __iomem *) (base + TANGOX_EHCI_REG_OFFSET);
    dev->regs       = (struct tango3_op_regs    __iomem *) (base +  TANGOX_EHCI_REG_OFFSET+0x40);

	if (request_irq ( TANGOX_EHCI_IRQ, (void *)tango3_irq, IRQF_DISABLED, driver_name, dev)
			!= 0) {
		printk("request interrupt %d failed\n", TANGOX_EHCI_IRQ);
		retval = -EBUSY;
		goto done;
	}

	/* init tango3_otg device */
	if(tango3_udc_init(dev, 0))
		return -EFAULT;

	dev->chiprev = ((readl(base)>>16) & 0x00ff);

	printk ("IRQ %d, MEM 0x%x, Chip Rev. %04x\n",
		 TANGOX_EHCI_IRQ, (u32)base, dev->chiprev);
	the_controller = dev;
	platform_set_drvdata(pdev, dev);


	retval = device_register (&dev->gadget.dev);
	device_create_file (&pdev->dev, &dev_attr_registers);
	//retval = device_add(&dev->gadget.dev);
	//if (retval) goto done;
	platform_device_put(pdev);

	return 0;

done:
	if (dev)
		tango3_remove (pdev);
	return retval;
}

static int tango3_suspend(struct platform_device *dev, pm_message_t message)
{
	return 0;
}

static int tango3_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver tango3_udc_driver = {
	.probe		= tango3_probe,
	.remove		= tango3_remove,
	.suspend	= tango3_suspend,
	.resume		= tango3_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= (char *) driver_name,
	},
};


static u64 tango3_dmamask = ~(u32)0;
static void tango3_udc_release_dev(struct device * dev)
{
        dev->parent = NULL;
}

static struct platform_device tango3_udc_device = {
	.name           = (char *)driver_name,
	.id             = -1,
	.dev = {
		.dma_mask               = &tango3_dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.release                = tango3_udc_release_dev,
	 },
	.num_resources  = 0,
	.resource       = 0,
};

static struct platform_device *tango3_platform_devices[] __initdata = {
        &tango3_udc_device,
};


extern unsigned long tangox_otg_bits;

static int __init init (void)
{
	unsigned long tangox_chip_id(void);
	unsigned long chip_id = (tangox_chip_id() >> 16) & 0xfffe;
	if (chip_id != 0x8652) {
		printk("Not supported chip.\n");
		return -EIO;
	} else if (test_and_set_bit(0, &tangox_otg_bits) != 0) {
		printk("OTG is used in different mode.\n");
		return -EIO;
	}

    platform_add_devices(tango3_platform_devices,
                                    ARRAY_SIZE(tango3_platform_devices));
	printk("%s, version: %s\n",DRIVER_DESC,  DRIVER_VERSION);
	return platform_driver_register(&tango3_udc_driver);
}

static void __exit cleanup (void)
{
	platform_driver_unregister(&tango3_udc_driver);
    platform_device_unregister(&tango3_udc_device);
	clear_bit(0, &tangox_otg_bits);
}

module_init (init);
module_exit (cleanup);

MODULE_DESCRIPTION (DRIVER_DESC);
MODULE_AUTHOR ("Sigma Designs");
MODULE_LICENSE ("GPL");


