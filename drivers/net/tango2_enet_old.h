
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
  
/*  
 * Definitions used to for ethernet module.
 */

#ifndef __EM86XX_ETH_H__ 
#define __EM86XX_ETH_H__ 

#ifndef BOOTLOADER
#include <asm/io.h>
#include <linux/delay.h>
#ifdef CONFIG_TANGOX
#include <asm/addrspace.h>
#ifdef CONFIG_TANGO2
#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/emhwlib_dram_tango2.h>
#endif
#define NON_CACHED(x)		KSEG1ADDR((u32)(x))
#define CACHED(x)		KSEG0ADDR((u32)(x))
#define PHYSADDR(x)		tangox_dma_address(CPHYSADDR(x))
#define IRQ_ETHERNET 		(IRQ_CONTROLLER_IRQ_BASE + LOG2_CPU_ETH_MAC_INT)//38
#elif defined(CONFIG_ARCH_TANGO15)
#include <asm/arch/hardware.h>
#define NON_CACHED(x)		em86xx_to_ncaddr((u32)(x))
#define CACHED(x)		em86xx_to_caddr((u32)(x))
#define PHYSADDR(x)		NON_CACHED(x)
#else
#error "Not support platform/architecture."
#endif

#include <asm/types.h>

#else

#include "config.h"
#include "version.h"
#include "hardware.h"
#include "io.h"

typedef unsigned int u32;
typedef unsigned short int u16;
typedef unsigned char u8;
typedef int     bool;

#define NON_CACHED(x)           (((u32)(x)) & 0x7fffffff)
#define PHYSADDR(x)		NON_CACHED(x)

struct net_device_stats
{
        unsigned long   rx_packets;             /* total packets received       */
        unsigned long   tx_packets;             /* total packets transmitted    */
        unsigned long   rx_bytes;               /* total bytes received         */
        unsigned long   tx_bytes;               /* total bytes transmitted      */
        unsigned long   rx_errors;              /* bad packets received         */
        unsigned long   tx_errors;              /* packet transmit problems     */
        unsigned long   rx_dropped;             /* no space in linux buffers    */
        unsigned long   tx_dropped;             /* no space available in linux  */
        unsigned long   multicast;              /* multicast packets received   */
        unsigned long   collisions;

        /* detailed rx_errors: */
        unsigned long   rx_length_errors;
        unsigned long   rx_over_errors;         /* receiver ring buff overflow  */
        unsigned long   rx_crc_errors;          /* recved pkt with crc error    */
        unsigned long   rx_frame_errors;        /* recv'd frame alignment error */
        unsigned long   rx_fifo_errors;         /* recv'r fifo overrun          */
        unsigned long   rx_missed_errors;       /* receiver missed packet       */

        /* detailed tx_errors */
        unsigned long   tx_aborted_errors;
        unsigned long   tx_carrier_errors;
        unsigned long   tx_fifo_errors;
        unsigned long   tx_heartbeat_errors;
        unsigned long   tx_window_errors;

        /* for cslip etc */
        unsigned long   rx_compressed;
        unsigned long   tx_compressed;

};

static inline int is_valid_ether_addr( u8 *addr )
{
        const char zaddr[6] = {0,};

        return !(addr[0]&1) && memcmp( addr, zaddr, 6);
}

#endif//BOOTLOADER

#define EM86XX_HOST_BASE	REG_BASE_host_interface
#define EM86XX_HOST_ETHMAC	0x00006000 
#define EM86XX_HOST_ETHDMA	0x00007000 
#define EM86XX_ETHDMA_BASE	CPHYSADDR(EM86XX_HOST_BASE + EM86XX_HOST_ETHDMA)
#define EM86XX_BMR_REG		(EM86XX_ETHDMA_BASE + 0x0)
#define EM86XX_TPDR_REG		(EM86XX_ETHDMA_BASE + 0x4)
#define EM86XX_RPDR_REG		(EM86XX_ETHDMA_BASE + 0x8)
#define EM86XX_RLBAR_REG	(EM86XX_ETHDMA_BASE + 0xc)
#define EM86XX_TLBAR_REG	(EM86XX_ETHDMA_BASE + 0x10)
#define EM86XX_SR_REG		(EM86XX_ETHDMA_BASE + 0x14)
#define EM86XX_CR_REG		(EM86XX_ETHDMA_BASE + 0x18)
#define EM86XX_IER_REG		(EM86XX_ETHDMA_BASE + 0x1c)
#define EM86XX_MFCR_REG		(EM86XX_ETHDMA_BASE + 0x20)
#define EM86XX_IFR_REG		(EM86XX_ETHDMA_BASE + 0x24) /*reserved ?*/
#define EM86XX_TDVPAR_REG	(EM86XX_ETHDMA_BASE + 0x4c) /*reserved ?*/
#define EM86XX_CHTBAR_REG	(EM86XX_ETHDMA_BASE + 0x50)
#define EM86XX_CHRBAR_REG	(EM86XX_ETHDMA_BASE + 0x54)

#define EM86XX_ETHMAC_BASE	CPHYSADDR(EM86XX_HOST_BASE + EM86XX_HOST_ETHMAC)
#define EM86XX_MACCR_REG	(EM86XX_ETHMAC_BASE + 0x0)
#define EM86XX_MACAHR_REG	(EM86XX_ETHMAC_BASE + 0x4)
#define EM86XX_MACALR_REG	(EM86XX_ETHMAC_BASE + 0x8)
#define EM86XX_MCHTHR_REG	(EM86XX_ETHMAC_BASE + 0xc)
#define EM86XX_MCHTLR_REG	(EM86XX_ETHMAC_BASE + 0x10)
#define EM86XX_MIIAR_REG	(EM86XX_ETHMAC_BASE + 0x14)
#define EM86XX_MIIDR_REG	(EM86XX_ETHMAC_BASE + 0x18)
#define EM86XX_FCR_REG		(EM86XX_ETHMAC_BASE + 0x1c)
#define EM86XX_V1TR_REG		(EM86XX_ETHMAC_BASE + 0x20)
#define EM86XX_V2TR_REG		(EM86XX_ETHMAC_BASE + 0x24)
#define EM86XX_WUFF_REG	        (EM86XX_ETHMAC_BASE + 0x28)
#define EM86XX_WUCS_REG		(EM86XX_ETHMAC_BASE + 0x2C)

/* MII interface */
#define MiiIfSpeed10M     	0x00000000
#define MiiIfSpeed100M		0x00000004
#define MiiIfExtPhyMii		0x00000000
#define MiiIfExtPhyRmii		0x00000001
#define MiiIfIntPhy    		0x00000002

#ifndef __ASSEMBLY__ 

/* Descriptor related ... */
struct em86xx_desc {
  volatile unsigned long desc0;
  volatile unsigned long desc1;
  volatile unsigned long desc2;
  volatile unsigned long desc3;
};

/**********************************************************
 * MAC110 Network interface registers
 **********************************************************/

enum MacControlReg      /* MAC Control register layout */
{                                         /* Bit description                        R/W   Reset value */
  MacFilterOff            = 0x80000000,     /* Receive all incoming packets         RW                */
  MacFilterOn             = 0,              /* Receive filtered packets only                  0       */

  MacBigEndian            = 0x40000000,     /* Big endian mode                      RW                */
  MacLittleEndian         = 0,              /* Little endian                                  0       */

  MacHeartBeatOff         = 0x10000000,     /* Heart beat signal quality disable    RW                */
  MacHeartBeatOn          = 0,              /* Heart beat signal quality enable               0       */

  MacSelectSrl            = 0x08000000,     /* Select SRL port                      RW                */
  MacSelectMii            = 0,              /* Select MII port                                0       */

  MacDisableRxOwn         = 0x00800000,     /* Disable receive own packets          RW                */
  MacEnableRxOwn          = 0,              /* Enable receive own packets                     0       */

  MacLoopbackExt          = 0x00400000,     /* External loopback                    RW                */
  MacLoopbackInt          = 0x00200000,     /* Internal loopback                                      */
  MacLoopbackOff          = 0,              /* Normal mode                                    00      */

  MacFullDuplex           = 0x00100000,     /* Full duplex mode                     RW                */
  MacHalfDuplex           = 0,              /* Half duplex mode                               0       */

  MacMulticastFilterOff   = 0x00080000,     /* Pass all multicast packets           RW                */
  MacMulticastFilterOn    = 0,              /* Pass filtered multicast packets                0       */

  MacPromiscuousModeOn    = 0x00040000,     /* Receive all valid packets            RW        1       */
  MacPromiscuousModeOff   = 0,              /* Receive filtered packets only                          */

  MacFilterInverce        = 0x00020000,     /* Inverse filtering                    RW                */
  MacFilterNormal         = 0,              /* Normal filtering                               0       */

  MacBadFramesEnable      = 0x00010000,     /* Pass bad frames                      RW                */
  MacBadFramesDisable     = 0,              /* Do not pass bad frames                         0       */

  MacPerfectFilterOff     = 0x00008000,     /* Hash filtering only                  RW                */
  MacPerfectFilterOn      = 0,              /* Both perfect and hash filtering                0       */

  MacHashFilterOn         = 0x00002000,     /* perfom hash filtering                RW                */
  MacHashFilterOff        = 0,              /* perfect filtering only                         0       */

  MacLateCollisionOn      = 0x00001000,     /* Enable late collision control        RW                */
  MacLateCollisionOff     = 0,              /* Disable late collision control                 0       */

  MacBroadcastDisable     = 0x00000800,     /* Disable reception of broadcast frames RW               */
  MacBroadcastEnable      = 0,              /* Enable broadcast frames                        0       */

  MacRetryDisable         = 0x00000400,     /* Disable retransmission               RW                */
  MacRetryEnable          = 0,              /* Enable retransmission                          0       */

  MacPadStripEnable       = 0x00000100,     /* Pad stripping enable                 RW                */
  MacPadStripDisable      = 0,              /* Pad stripping disable                          0       */

  MacBackoff10            = 0,              /* Backoff Limit (not documented)       RW        00      */
  MacBackoff8             = 0x00000040,     /* Backoff Limit (not documented)       RW        01      */
  MacBackoff4             = 0x00000080,     /* Backoff Limit (not documented)       RW        10      */
  MacBackoff1             = 0x000000c0,     /* Backoff Limit (not documented)       RW        11      */

  MacDeferralCheckEnable  = 0x00000020,     /* Deferral check enable                RW                */
  MacDeferralCheckDisable = 0,              /* Deferral check disable                         0       */

  MacTxEnable             = 0x00000008,     /* Transmitter enable                   RW                */
  MacTxDisable            = 0,              /* Transmitter disable                            0       */

  MacRxEnable             = 0x00000004,     /* Receiver enable                      RW                */
  MacRxDisable            = 0,              /* Receiver disable                               0       */
};

enum MiiRegisters
{
  GEN_ctl   = 0x00,	/* Basic Mode Control Register */
			/* bit 0-7, 10: Reserved, 
			   bit 8:  	Duplex Mode: 1=FD, 0=HD
			   bit 9:	Restart Auto negotiation 1=restart, 0=normal operation
			   bit11:	1=Power Down, 0=normal operation
			   bit12:	1=enable auto-negotiation, bit13 and 8 will be ingnored
					0=disable auto-negotiation, bit13 & 8 to determine the speed and mode
			   bit13	1=100Mbps 0=10Mbps
			   bit14	1=enable loopback 0=normal operation
			   bit15	1=software reset  0=normal operation*/     	
  GEN_sts   = 0x01,	/* Basic Mode Status Register  */
			/* bit 0:	1=extended register capability 0=basic register capability only
			   bit 1:	1=jabber condidtion detected 0=no jabber condition detected
			   bit 2:	1=valid link established 0=no link
			   bit 3:	1=Auto-negotiation ok 0=fail
			   bit 4:	1=remote fault 0=no remote fault
			   bit 5:	1=auto-negotiation process completed, 0=not completed
			   bit 6:	MF Preamble suppression
			   bit 7-10:	Reserved
			   bit 11:	1=enable 10Base-T HD 0=suppress 10/HD
			   bit 12:	1=enable 10Base-T FD 0=suppress 10/FD
			   bit 13:	1=enable 100Base-T FD 0=suppress 100/FD
			   bit 14:	1=enable 100Base-T FD 0=suppress 100/FD
			   bit 15:	1=enable 100Base-T4 0=suppress 100-T4*/
  GEN_id_hi = 0x02,	/* PHY Identifier Register 1   bit 0-15: High PHY identifier default=0000 for RTL8201BL*/
  GEN_id_lo = 0x03,	/* PHY Identifier Register 2   bit 0-15: Low PHY identifier  default=8201 for RTL8201BL*/
  AN_adv    = 0x04,	/* Auto-negotiation Advertisement Register */
			/* bit 0-4:	Selector, only CSMA/CD<0001> is specified
			   bit 5:	1=10Base-T is supported      0=10Base-T is not supported by local node
			   bit 6:	1=10Base-T FD is supported   0=10Base-T FD is not supported by local node
			   bit 7:	1=100Base-TX is supported    0=100Base-TX is not supported by local node
			   bit 8:	1=100Base-TX FD is supported 0=100Base-TX FD is not supported by local node
			   bit 9:	1=100Base-T4 is supported    0=100Base-T4 is not supported by local node
			   bit10 :	1=iflow control is supported 0=flow control is not supported by local node
			   bit11-12:	Reserved
			   bit13:	1=advertise remote fault detectioncapability 0=do not advertise remote fault detection capability
			   bit14:	1=ack reception of link partner capability data word 0=donot ack reception
			   bit15:	1=transmitting the protocol specific data page 0=transmitting the primary capability data page*/
  AN_lpa    = 0x05,	/* Auto-negotiation Link partner Ability Register */
  AN_exp    = 0x06,	/* Auto-negotiation Expansion Register */
  AN_np     = 0x07,	/* Auto-negotiation Next Page TX */
  TST       = 0x19,     /* Test register, for checking link10/100 is established*/
			/* bit 0:	1=Link 100Base Ok, 0=No 100Base link 
			   bit 1:	1=Link 10Base Ok,  0=No 10Base link */

};

enum Mii_GEN_ctl
{
  RESET		= 0x8000,	/* Reset */
  SPEED		= 0x2000,	/* 100 Mbit/s */
  AUTONEG_ENB	= 0x1000,	/* Auto negotiation enabled */
  POWER_DWN	= 0x800, 	/* Power-down enabled */
  AUTONEG_REST	= 0x200,	/* Restart auto negotiation */
  DUPLEX 	= 0x0100,	/* Duplex mode */
};

enum Mii_GEN_sts
{
  AUTOCMPLT 	= 0x0020,   	/* Autonegotiation completed */
  LINK     	= 0x0004,   	/* Link status */
};

enum Mii_AN_adv
{
  CSMACD	= 0x1,		/* CSMA/CD protocol */
  BASET10	= 0x20, 	/* 10 BaseT support */
  BASET10FD	= 0x40, 	/* 10 BaseT full duplex support */
  BASET100	= 0x80, 	/* 100 BaseT support */
  BASET100FD	= 0x100, 	/* 100 BaseT full duplux support */
  FLOWCONTROL	= 0x400, 	/* Flow control support */
  RF		= 0x2000,	/* Remote fault support */
};

enum Mii_TST
{
  LINK100	= 0x1,		/* Link 100 status */
  LINK10	= 0x2,		/* Link 10  status */
};


enum MacMiiAddrReg     		/* MII address register layout */
{
  MiiDevMask    = 0x0000F800,   /* MII device address */
  MiiDevShift   = 11,

  MiiRegMask    = 0x000007C0,   /* MII register */
  MiiRegShift   = 6,

  MiiWrite      = 0x00000002,   /* Write to register */
  MiiRead       = 0,            /* Read from register */
  MiiBusy       = 0x00000001,   /* MII interface is busy */
};

enum MacMiiDataReg     		/* MII address register layout */
{
  MiiDataMask   = 0x0000FFFF,   /* MII Data */
};

enum MacFlowControlReg 		/* MAC flow control register layout */
{                                         /* Bit description                        R/W   Reset value */
  MacPauseTimeMask        = 0xFFFF0000,   /* PAUSE TIME field in the control frame  RW      0000      */
  MacPauseTimeShift       = 15,

  MacControlFrameEnable   = 0x00000004,   /* Enable pass control frames to the host RW                */
  MacControlFrameDisable  = 0,            /* Do not pass control frames to host               0       */

  MacFlowControlEnable    = 0x00000002,   /* Enable flow control                    RW                */
  MacFlowControlDisable   = 0,            /* Disable flow control                             0       */

  MacSendPauseFrame       = 0x00000001,   /* send pause frame                       RW        0       */
};

/**********************************************************
 * DMA Engine registers
 **********************************************************/

enum DmaBusModeReg         /* DMA bus mode register */
{                                         /* Bit description                        R/W   Reset value */
  DmaBigEndianDesc        = 0x00100000,   /* Big endian data buffer descriptors     RW                */
  DmaLittleEndianDesc     = 0,            /* Little endian data descriptors                   0       */

  DmaBurstLength32        = 0x00002000,   /* Dma burst length = 32                  RW                */
  DmaBurstLength16        = 0x00001000,   /* Dma burst length = 16                                    */
  DmaBurstLength8         = 0x00000800,   /* Dma burst length = 8                                     */
  DmaBurstLength4         = 0x00000400,   /* Dma burst length = 4                                     */
  DmaBurstLength2         = 0x00000200,   /* Dma burst length = 2                                     */
  DmaBurstLength1         = 0x00000100,   /* Dma burst length = 1                                     */
  DmaBurstLength0         = 0x00000000,   /* Dma burst length = 0                             0       */

  DmaBigEndianData        = 0x00000080,   /* Big endian data buffers                RW                */
  DmaLittleEndianData     = 0,            /* Little endian data buffers                       0       */

  DmaDescriptorSkip16     = 0x00000040,   /* number of dwords to skip               RW                */
  DmaDescriptorSkip8      = 0x00000020,   /* between two unchained descriptors                        */
  DmaDescriptorSkip4      = 0x00000010,   /*                                                          */
  DmaDescriptorSkip2      = 0x00000008,   /*                                                          */
  DmaDescriptorSkip1      = 0x00000004,   /*                                                          */
  DmaDescriptorSkip0      = 0,            /*                                                  0       */

  DmaReceivePriorityOff   = 0x00000002,   /* equal rx and tx priorities             RW                */
  DmaReceivePriorityOn    = 0,            /* Rx has prioryty over Tx                          0       */

  DmaResetOn              = 0x00000001,   /* Reset DMA engine                       RW                */
  DmaResetOff             = 0,            /*                                                  0       */
};

enum DmaStatusReg         /* DMA Status register */
{                                         /* Bit description                        R/W   Reset value */
  DmaRxAbort              = 0x01000000,   /* receiver bus abort                     R         0       */
  DmaTxAbort              = 0x00800000,   /* transmitter bus abort                  R         0       */

  DmaTxState              = 0x00700000,   /* Transmit process state                 R         000     */
  DmaTxStopped            = 0x00000000,   /* Stopped                                                  */
  DmaTxFetching           = 0x00100000,   /* Running - fetching the descriptor                        */
  DmaTxWaiting            = 0x00200000,   /* Running - waiting for end of transmission                */
  DmaTxReading            = 0x00300000,   /* Running - reading the data from memory                   */
  DmaTxSuspended          = 0x00600000,   /* Suspended                                                */
  DmaTxClosing            = 0x00700000,   /* Running - closing descriptor                             */

  DmaRxState              = 0x000E0000,   /* Receive process state                  R         000     */
  DmaRxStopped            = 0x00000000,   /* Stopped                                                  */
  DmaRxFetching           = 0x00020000,   /* Running - fetching the descriptor                        */
  DmaRxChecking           = 0x00040000,   /* Running - checking for end of packet                     */
  DmaRxWaiting            = 0x00060000,   /* Running - waiting for packet                             */
  DmaRxSuspended          = 0x00080000,   /* Suspended                                                */
  DmaRxClosing            = 0x000A0000,   /* Running - closing descriptor                             */
  DmaRxFlushing           = 0x000C0000,   /* Running - flushing the current frame                     */
  DmaRxQueuing            = 0x000E0000,   /* Running - queuing the recieve frame into host memory     */

  DmaIntNormal            = 0x00010000,   /* Normal interrupt summary               RW        0       */
  DmaIntAbnormal          = 0x00008000,   /* Abnormal interrupt summary             RW        0       */

  DmaIntEarlyRx           = 0x00004000,   /* Early receive interrupt (Normal)       RW        0       */
  DmaIntBusError          = 0x00002000,   /* Fatal bus error (Abnormal)             RW        0       */
  DmaIntEarlyTx           = 0x00000400,   /* Early transmit interrupt (Abnormal)    RW        0       */
  DmaIntRxStopped         = 0x00000100,   /* Receive process stopped (Abnormal)     RW        0       */
  DmaIntRxNoBuffer        = 0x00000080,   /* Receive buffer unavailable (Abnormal)  RW        0       */
  DmaIntRxCompleted       = 0x00000040,   /* Completion of frame reception (Normal) RW        0       */
  DmaIntTxUnderflow       = 0x00000020,   /* Transmit underflow (Abnormal)          RW        0       */
  DmaIntTxNoBuffer        = 0x00000004,   /* Transmit buffer unavailable (Normal)   RW        0       */
  DmaIntTxStopped         = 0x00000002,   /* Transmit process stopped (Abnormal)    RW        0       */
  DmaIntTxCompleted       = 0x00000001,   /* Transmit completed (Normal)            RW        0       */
};

enum DmaControlReg        /* DMA control register */
{                                         /* Bit description                        R/W   Reset value */
  DmaStoreAndForward      = 0x00200000,   /* Store and forward                      RW        0       */
  DmaTxStart              = 0x00002000,   /* Start/Stop transmission                RW        0       */
  DmaTxSecondFrame        = 0x00000004,   /* Operate on second frame                RW        0       */
  DmaRxStart              = 0x00000002,   /* Start/Stop reception                   RW        0       */
};

enum  DmaInterruptReg     /* DMA interrupt enable register */
{                                         /* Bit description                        R/W   Reset value */
  DmaIeNormal            = DmaIntNormal     ,   /* Normal interrupt enable                 RW        0       */
  DmaIeAbnormal          = DmaIntAbnormal   ,   /* Abnormal interrupt enable               RW        0       */

  DmaIeEarlyRx           = DmaIntEarlyRx    ,   /* Early receive interrupt enable          RW        0       */
  DmaIeBusError          = DmaIntBusError   ,   /* Fatal bus error enable                  RW        0       */
  DmaIeEarlyTx           = DmaIntEarlyTx    ,   /* Early transmit interrupt enable         RW        0       */
  DmaIeRxStopped         = DmaIntRxStopped  ,   /* Receive process stopped enable          RW        0       */
  DmaIeRxNoBuffer        = DmaIntRxNoBuffer ,   /* Receive buffer unavailable enable       RW        0       */
  DmaIeRxCompleted       = DmaIntRxCompleted,   /* Completion of frame reception enable    RW        0       */
  DmaIeTxUnderflow       = DmaIntTxUnderflow,   /* Transmit underflow enable               RW        0       */
  DmaIeTxNoBuffer        = DmaIntTxNoBuffer ,   /* Transmit buffer unavailable enable      RW        0       */
  DmaIeTxStopped         = DmaIntTxStopped  ,   /* Transmit process stopped enable         RW        0       */
  DmaIeTxCompleted       = DmaIntTxCompleted,   /* Transmit completed enable               RW        0       */
};

/**********************************************************
 * DMA Engine descriptors
 **********************************************************/

enum DmaDescriptorStatus    /* status word of DMA descriptor */
{
  DescOwnByDma          = 0x80000000,   /* Descriptor is owned by DMA engine  */
  DescOwnByCPU          = 0x0,          /* Descriptor is owned by CPU  */

  DescFilteringFail	= 0x40000000,   /* Filtering fail*/

  DescFrameLengthMask   = 0x3FFF0000,   /* Receive descriptor frame length */
  DescFrameLengthShift  = 16,

  DescError             = 0x00008000,   /* Error summary bit  - OR of the following bits:    v  */

  DescRxTruncated       = 0x00004000,   /* Rx - no more descriptors for receive frame        E  */
  DescLengthError	= 0x00001000,   /* Rx - length doesn't match 			     E	*/
  DescRxDamaged         = 0x00000800,   /* Rx - frame was damaged by a collision             E  */
  DescRxMulticast       = 0x00000400,   /* Rx - received frame is multicast                     */
  DescRxFirst           = 0x00000200,   /* Rx - first descriptor of the frame                   */
  DescRxLast            = 0x00000100,   /* Rx - last descriptor of the frame                    */
  DescRxLongFrame       = 0x00000080,   /* Rx - frame is longer than 1518 bytes              E  */
  DescRxCollision       = 0x00000040,   /* Rx - frame was damaged by a collision             E  */
  DescRxFrameEther      = 0x00000020,   /* Rx - Frame type - Ethernet, otherwise 802.3          */
  DescRxMiiError        = 0x00000008,   /* Rx - error reported by MII interface              E  */
  DescRxDribbling       = 0x00000004,   /* Rx - frame contains noninteger multiple of 8 bits    */
  DescRxCrc             = 0x00000002,   /* Rx - CRC error                                    E  */

  DescTxTimeout         = 0x00004000,   /* Tx - Transmit jabber timeout                      E  */
  DescTxLostCarrier     = 0x00000800,   /* Tx - carrier lost during tramsmission             E  */
  DescTxNoCarrier       = 0x00000400,   /* Tx - no carrier signal from the tranceiver        E  */
  DescTxLateCollision   = 0x00000200,   /* Tx - transmission aborted due to collision        E  */
  DescTxExcCollisions   = 0x00000100,   /* Tx - transmission aborted after 16 collisions     E  */
  DescTxHeartbeatFail   = 0x00000080,   /* Tx - heartbeat collision check failure               */
  DescTxCollMask        = 0x00000078,   /* Tx - Collision count                                 */
  DescTxCollShift       = 3,
  DescTxExcDeferral     = 0x00000004,   /* Tx - excessive deferral                           E  */
  DescTxUnderflow       = 0x00000002,   /* Tx - late data arrival from the memory            E  */
  DescTxDeferred        = 0x00000001,   /* Tx - frame transmision deferred                      */
};

enum DmaDescriptorLength    /* length word of DMA descriptor */
{
  DescTxIntEnable       = 0x80000000,   /* Tx - interrupt on completion                         */
  DescTxLast            = 0x40000000,   /* Tx - Last segment of the frame                       */
  DescTxFirst           = 0x20000000,   /* Tx - First segment of the frame                      */
  DescTxDisableCrc      = 0x04000000,   /* Tx - Add CRC disabled (first segment only)           */

  DescEndOfRing         = 0x02000000,   /* End of descriptors ring                              */
  DescChain             = 0x01000000,   /* Second buffer address is chain address               */

  DescSize2Mask         = 0x003FF800,   /* Buffer 2 size                                        */
  DescSize2Shift        = 11,
  DescSize1Mask         = 0x000007FF,   /* Buffer 1 size                                        */
  DescSize1Shift        = 0,
};

/**********************************************************
 * Initial register values
 **********************************************************/

enum InitialRegisters
{
  MacControlInitFdx       /* Full-duplex mode with perfect filter on */
                          = MacFilterOn           | MacLittleEndian         | MacHeartBeatOn      | MacSelectMii
                          | MacEnableRxOwn        | MacLoopbackOff          | MacFullDuplex       | MacMulticastFilterOn
                          | MacPromiscuousModeOff | MacFilterNormal         | MacBadFramesDisable | MacPerfectFilterOn
                          | MacHashFilterOff      | MacLateCollisionOff     | MacBroadcastEnable  | MacRetryEnable
                          | MacPadStripDisable    | MacDeferralCheckDisable | MacTxEnable         | MacRxEnable,

  MacFlowControlInitFdx   /* Full-duplex mode */
                          = MacControlFrameDisable | MacFlowControlEnable,

  MacControlInitHdx       /* Half-duplex mode with perfect filter on */
                          = MacFilterOn           | MacLittleEndian         | MacHeartBeatOn      | MacSelectMii
                          | MacDisableRxOwn       | MacLoopbackOff          | MacHalfDuplex       | MacMulticastFilterOn 
                          | MacPromiscuousModeOff | MacFilterNormal         | MacBadFramesDisable | MacPerfectFilterOn
                          | MacHashFilterOff      | MacLateCollisionOff     | MacBroadcastEnable  | MacRetryEnable
                          | MacPadStripDisable    | MacDeferralCheckDisable | MacTxEnable         | MacRxEnable,

  MacFlowControlInitHdx   /* Half-duplex mode */
                          = MacControlFrameDisable | MacFlowControlDisable,

  DmaBusModeInit          /* Little-endian mode */
                          = DmaLittleEndianDesc   | DmaBurstLength8         | DmaLittleEndianData | DmaDescriptorSkip2
                          | DmaReceivePriorityOn  | DmaResetOff,

  DmaControlInit100       /* 100 Mb/s mode */
                          = DmaStoreAndForward,

  DmaControlInit10        /* 10 Mb/s mode */
                          = DmaStoreAndForward,

                          /* Interrupt groups */
  DmaIntErrorMask         = DmaIntBusError,           /* Error */
  DmaIntRxAbnMask         = DmaIntRxNoBuffer,         /* receiver abnormal interrupt */
  DmaIntRxNormMask        = DmaIntRxCompleted,        /* receiver normal interrupt   */
  DmaIntRxStoppedMask     = DmaIntRxStopped,          /* receiver stopped */
  DmaIntTxAbnMask         = DmaIntTxUnderflow,        /* transmitter abnormal interrupt */
  DmaIntTxNormMask        = DmaIntTxCompleted,        /* transmitter normal interrupt */
  DmaIntTxStoppedMask     = DmaIntTxStopped,          /* receiver stopped */

  DmaIntEnable            = DmaIeNormal | DmaIeAbnormal
                          | DmaIntErrorMask
                          | DmaIntRxAbnMask | DmaIntRxNormMask | DmaIntRxStoppedMask
                          | DmaIntTxAbnMask | DmaIntTxNormMask | DmaIntTxStoppedMask,

  DmaIntDisable           = 0,
};

/* some status test functions */
static inline int em86xx_dma_rx_valid( u32 Status )
{
  return ( (Status & DescError) == 0 )      /* no errors, whole frame is in the buffer */
      && ( (Status & DescRxFirst) != 0 )
      && ( (Status & DescRxLast) != 0 );
}

static inline u32 em86xx_dma_rx_length( u32 Status )
{
  return (Status & DescFrameLengthMask) >> DescFrameLengthShift;
}

static inline int em86xx_dma_rx_collisions( u32 Status )
{
  if( Status & (DescRxDamaged | DescRxCollision) ) return 1;
    return 0;
}

static inline int em86xx_dma_rx_crc( u32 Status )
{
  if( Status & DescRxCrc ) return 1;
    return 0;
}

static inline int em86xx_dma_tx_valid( u32 Status )   /* Test the status word if the descriptor is valid */
{
  return ( (Status & DescError) == 0 );
}

static inline int em86xx_dma_tx_collisions( u32 Status )
{
  return (Status & DescTxCollMask) >> DescTxCollShift;
}

static inline int em86xx_dma_tx_aborted( u32 Status )
{
  if( Status & (DescTxLateCollision | DescTxExcCollisions )) return 1;
    return 0;
}

static inline int em86xx_dma_tx_carrier( u32 Status )
{
  if( Status & (DescTxLostCarrier | DescTxNoCarrier )) return 1;
    return 0;
}

static inline int em86xx_rdesc_owned_by_host(volatile struct em86xx_desc *desc)
{
  return((desc->desc0 & 0x80000000) ? 0 : 1);
}

static inline int em86xx_rdesc_last_desc(volatile struct em86xx_desc *desc)
{
  return(desc->desc0 & 0x00000100);
}

static inline int em86xx_rdesc_first_desc(volatile struct em86xx_desc *desc)
{
  return(desc->desc0 & 0x00000200);
}

static inline unsigned long em86xx_rdesc_frame_len(volatile struct em86xx_desc *desc)
{
  return((desc->desc0 & 0x3fff0000) >> 16);
}

static inline int em86xx_tdesc_owned_by_host(volatile struct em86xx_desc *desc)
{
  return((desc->desc0 & 0x80000000) ? 0 : 1);
}

static u32 __inline__ em86xx_read_reg( u32 Reg )
{
#ifdef CONFIG_TANGOX
	u32 data = gbus_read_reg32(Reg);
	u32 tmp = 0;
	extern int is_tango2_es6(void);
	if (is_tango2_es6()) 
		tmp = gbus_read_reg32(REG_BASE_system_block + SYS_xtal_in_cnt);
#else
        u32 data = __raw_readl(Reg);
#endif

//        printk("read data=0x%08x from addr=0x%08x\n", data, Reg);
        return data;
}

static u32 em86xx_read_mac_reg(u32 Reg)
{
	return em86xx_read_reg( EM86XX_ETHMAC_BASE + Reg );
}

static u32 em86xx_read_dma_reg(u32 Reg)
{
	return em86xx_read_reg( EM86XX_ETHDMA_BASE + Reg );
}

static void __inline__ em86xx_write_reg( u32 Reg, u32 Data )
{
//       printk("write data=0x%08x to addr=0x%08x\n", Data, Reg);
#ifdef CONFIG_TANGOX
        gbus_write_reg32(Reg, Data); iob();
#else
        __raw_writel(Data,Reg);
#endif
}

static void em86xx_write_mac_reg(u32 Reg, u32 Data)
{
	em86xx_write_reg( EM86XX_ETHMAC_BASE + Reg, Data );
}

static void em86xx_write_dma_reg(u32 Reg, u32 Data)
{
	em86xx_write_reg( EM86XX_ETHDMA_BASE + Reg, Data );
}


static void __inline__ em86xx_set_reg( u32 reg, u32 data )
{

#ifdef CONFIG_TANGOX
	data |= gbus_read_reg32(reg);
	gbus_write_reg32(reg, data); iob();
#else
	data |= __raw_readl(reg);
	__raw_writel(data, reg);
#endif
}

static void __inline__ em86xx_clear_reg( u32 reg, u32 data )
{
#ifdef CONFIG_TANGOX
	gbus_write_reg32(reg, (gbus_read_reg32(reg) & (~data))); iob();
#else
	__raw_writel(( __raw_readl(reg) & (~data)), reg);
#endif
}


/* There are two registers, MII address (EM86XX_MIIAR_REG) and 
   MII data (EM86XX_MIIDR_REG), for accessing PHY registers.

   In order to access  mii register data, one need to program the
   MII address and wait till MII not busy.
   
   MII address register description:
	bit 31-16	reserved
	bit 15-11	PHY address
	bit 10-6	MII Register one want to access 
	bit 1		0:MII read 1:MII write
	bit 0		0:MII not busy 1:MII busy
*/
static inline u16 em86xx_mii_read(int phy_addr, u8 Reg )
{
        u32 addr;
        u16 data;

#ifdef CONFIG_TANGOX
	u32 count = 100;
#endif

        addr = ((phy_addr << MiiDevShift) & MiiDevMask) |
        	((Reg << MiiRegShift) & MiiRegMask);
        em86xx_write_reg(EM86XX_MIIAR_REG, addr );

#ifdef CONFIG_TANGOX
	do{
		udelay(1);
		count --;
		if(count == 0) break;
	} while( (em86xx_read_reg(EM86XX_MIIAR_REG ) & MiiBusy) == MiiBusy );
#else
        do{} while( (em86xx_read_reg(EM86XX_MIIAR_REG ) & MiiBusy) == MiiBusy );
#endif

        data = em86xx_read_reg(EM86XX_MIIDR_REG ) & 0xFFFF;
/* 	printk("em86xx_mii_read: addr %d - reg %d - value: %04x\n", */
/* 	       phy_addr, Reg, data); */
        return data;
}

#ifndef BOOTLOADER
static void em86xx_mii_write(int phy_addr, u8 Reg, u16 Data )
{
        u32 addr;

#ifdef CONFIG_TANGOX
	u32 count = 100;
#endif

        em86xx_write_reg( EM86XX_MIIDR_REG, Data );
        addr = ((phy_addr << MiiDevShift) & MiiDevMask) |
               ((Reg << MiiRegShift) & MiiRegMask) |
                 MiiWrite;

        em86xx_write_reg( EM86XX_MIIAR_REG, addr );

#ifdef CONFIG_TANGOX
	do{
		udelay(1);
		count --;
		if(count == 0) break;
	} while( (em86xx_read_reg(EM86XX_MIIAR_REG ) & MiiBusy) == MiiBusy );
#else
        do{} while( (em86xx_read_reg(EM86XX_MIIAR_REG ) & MiiBusy) == MiiBusy );
#endif

/* 	printk("em86xx_write_read: addr %d - reg %d - data: %04x\n", */
/* 	       phy_addr, Reg, Data); */

}
#endif
#if 0 /* for debug purpose*/
static void mac_dump(void)
{
	int i;
	u32 data, addr;
	addr = 0;
	printk("*********************** MAC Registers *************************\n");
        for(i = 0; i < 12; i++) {
		data = em86xx_read_mac_reg(addr);		
        	if(!(i%4))
                	printk( "0x%08x  ", addr);
	        printk( "%08x ", data );
        	if(!((i+1)%4))
                	printk("\n");
       		 addr += 4;
	}
}
static void dma_dump(void)
{
	int i;
	u32 data, addr;
	addr = 0;
	printk("*********************** DMA Registers *************************\n");
        for(i = 0; i < 24; i++) {
		data = em86xx_read_dma_reg(addr);		
        	if(!(i%4))
                	printk( "0x%08x  ", addr);
	        printk( "%08x ", data );
        	if(!((i+1)%4))
                	printk("\n");
       		 addr += 4;
	}

}
static void rx_desc_dump(void)
{
	int i;
	u32 data, addr;
	addr = em86xx_read_reg(EM86XX_RLBAR_REG);
	printk("*********************** RX DESC *************************\n");
        for(i = 0; i < 128; i++) {
		data = em86xx_read_reg(addr);		
        	if(!(i%4))
                	printk( "0x%08x  ", addr);
	        printk( "%08x ", data );
        	if(!((i+1)%4))
                	printk("\n");
       		 addr += 4;
	}
}
static void tx_desc_dump(void) 
{
	int i;
	u32 data, addr;
	addr = em86xx_read_reg(EM86XX_TLBAR_REG);
	printk("*********************** TX DESC *************************\n");
        for(i = 0; i < 128; i++) {
		data = em86xx_read_reg(addr);		
        	if(!(i%4))
                	printk( "0x%08x  ", addr);
	        printk( "%08x ", data );
        	if(!((i+1)%4))
                	printk("\n");
       		 addr += 4;
	}
}
#endif /* #if 0*/
#endif /* !__ASSEMBLY__ */
#endif /* __EM86XX_ETH_H__ */

