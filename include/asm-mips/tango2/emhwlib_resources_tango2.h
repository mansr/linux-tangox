/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/**
  @file   emhwlib_resources_tango2.h
  @brief  

  long description

  @author Emmanuel Michon
  @date   2004-01-28y
*/

#ifndef __EMHWLIB_RESOURCES_TANGO2_H__
#define __EMHWLIB_RESOURCES_TANGO2_H__

#define VSYNC_PARAM_MUTEX   ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex2))
#define PCI_IRQ_MUTEX       ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex3))
#define GFX_MUTEX           ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex4))
#define HOST_MBUS_MUTEX     ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex5))
#define SOFT_IRQ_MUTEX_TASK ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex6))
#define SOFT_IRQ_MUTEX_IRQ  ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex7))
#define SOFT_IRQ_MUTEX_FIQ  ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex8))
#define RTC_IRQ_MUTEX       ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex9))
#define XRPC_MUTEX          ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex10))
#define XTASK_MUTEX         ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex11))
#define IDMA_MUTEX          ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex12)) /* keep same as tango15 */
#define TIMER_IRQ_MUTEX     ((struct gbus_mutex *)(REG_BASE_host_interface  + host_mutex13)) 

#define AUDIO_0_IRQ_MUTEX   ((struct gbus_mutex *)(DMEM_BASE_audio_engine_0 + 4 * audio_mutex0))
#define AUDIO_1_IRQ_MUTEX   ((struct gbus_mutex *)(DMEM_BASE_audio_engine_1 + 4 * audio_mutex0))
#define VIDEO_0_FIFO_MUTEX  ((struct gbus_mutex *)(DMEM_BASE_mpeg_engine_0  + 4 * mpeg_mutex0))
#define VIDEO_1_FIFO_MUTEX  ((struct gbus_mutex *)(DMEM_BASE_mpeg_engine_1  + 4 * mpeg_mutex0))
#define DEMUX_IRQ_MUTEX     ((struct gbus_mutex *)(DMEM_BASE_demux_engine   + 4 * demux_mutex0))
#define DEMUX_EMHWLIB_MUTEX ((struct gbus_mutex *)(DMEM_BASE_demux_engine   + 4 * demux_mutex4))
#define TIMER_UPDATE_MUTEX  ((struct gbus_mutex *)(DMEM_BASE_demux_engine   + 4 * demux_mutex5))

#define AUDIO_1_RPC_MUTEX ((struct gbus_mutex *)(DMEM_BASE_audio_engine_1 + 4 * audio_mutex1))
#define AUDIO_1_ENET_MUTEX ((struct gbus_mutex *)(DMEM_BASE_audio_engine_1 + 4 * audio_mutex2))
#define AUDIO_1_INTSTATUS_MUTEX ((struct gbus_mutex *)(DMEM_BASE_audio_engine_1 + 4 * audio_mutex3))

#endif // __EMHWLIB_RESOURCES_TANGO2_H__
