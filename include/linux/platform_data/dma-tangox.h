#ifndef __PLATFORM_DATA_DMA_TANGOX_H
#define __PLATFORM_DATA_DMA_TANGOX_H

#include <linux/dmaengine.h>
#include <linux/ioport.h>

struct tangox_dma_chan_data {
	enum dma_transfer_direction direction;
	struct resource regs;
	int irq;
	int sbox_id;
};

struct tangox_dma_pdata {
	int num_chans;
	struct tangox_dma_chan_data *chan;
	int num_slaves;
	int *slave_id;
};

#define DEFINE_DMA_CHAN(_dir, _regs, _irq, _sbox_id)		\
	{							\
		.direction	= (_dir),			\
		.regs		= DEFINE_RES_MEM(_regs, 0x10),	\
		.irq		= (_irq),			\
		.sbox_id	= (_sbox_id),			\
	}

#endif /* __PLATFORM_DATA_DMA_TANGOX_H */
