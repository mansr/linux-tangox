/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/**
  @file   emhwlib_resources_shared.h
  @brief  

  long description

  @author Emmanuel Michon
  @date   2005-03-22
*/

#ifndef __EMHWLIB_RESOURCES_SHARED_H__
#define __EMHWLIB_RESOURCES_SHARED_H__

#define VIDEO_0_RPC_MUTEX ((struct gbus_mutex *)(DMEM_BASE_mpeg_engine_0  + 4 * mpeg_mutex1))
#define VIDEO_1_RPC_MUTEX ((struct gbus_mutex *)(DMEM_BASE_mpeg_engine_1  + 4 * mpeg_mutex1))
#define AUDIO_0_RPC_MUTEX ((struct gbus_mutex *)(DMEM_BASE_audio_engine_0 + 4 * audio_mutex1))
#define DEMUX_RPC_MUTEX   ((struct gbus_mutex *)(DMEM_BASE_demux_engine   + 4 * demux_mutex3))

// pt110 local ram map
#define UCLINUX_CURRENT_PROCESS (REG_BASE_cpu_block + 0x0000)

#define RESET_VECTOR   (REG_BASE_cpu_block + 0x0000)
#define UNDEF_VECTOR   (REG_BASE_cpu_block + 0x0004)
#define SWI_VECTOR     (REG_BASE_cpu_block + 0x0008)
#define I_ABORT_VECTOR (REG_BASE_cpu_block + 0x000c)
#define D_ABORT_VECTOR (REG_BASE_cpu_block + 0x0010)
#define RSV_VECTOR     (REG_BASE_cpu_block + 0x0014)
#define IRQ_VECTOR     (REG_BASE_cpu_block + 0x0018)
#define FIQ_VECTOR     (REG_BASE_cpu_block + 0x001c)

#define RESET_JUMP     (REG_BASE_cpu_block + 0x0020)
#define UNDEF_JUMP     (REG_BASE_cpu_block + 0x0024)
#define SWI_JUMP       (REG_BASE_cpu_block + 0x0028)
#define I_ABORT_JUMP   (REG_BASE_cpu_block + 0x002c)
#define D_ABORT_JUMP   (REG_BASE_cpu_block + 0x0030)
#define RSV_JUMP       (REG_BASE_cpu_block + 0x0034)
#define IRQ_JUMP       (REG_BASE_cpu_block + 0x0038)
#define FIQ_JUMP       (REG_BASE_cpu_block + 0x003c)

#define INFINITE_LOOP  (REG_BASE_cpu_block + 0x0040)

/* where to store uclinux interrupt handler */
#define UCLINUX_RESET_VECTOR   (REG_BASE_cpu_block + 0x0044)
#define UCLINUX_UNDEF_VECTOR   (REG_BASE_cpu_block + 0x0048)
#define UCLINUX_SWI_VECTOR     (REG_BASE_cpu_block + 0x004c)
#define UCLINUX_I_ABORT_VECTOR (REG_BASE_cpu_block + 0x0050)
#define UCLINUX_D_ABORT_VECTOR (REG_BASE_cpu_block + 0x0054)
#define UCLINUX_RSV_VECTOR     (REG_BASE_cpu_block + 0x0058)
#define UCLINUX_IRQ_VECTOR     (REG_BASE_cpu_block + 0x005c)
#define UCLINUX_FIQ_VECTOR     (REG_BASE_cpu_block + 0x0060)

/* where to store fiq/irq enable values */
#define UCLINUX_IRQ_ENABLE      (REG_BASE_cpu_block + 0x0064)
#define UCLINUX_FIQ_ENABLE      (REG_BASE_cpu_block + 0x0068)

/* we use this in uClinux to handshake llad with the hardware library. llad will 
   initialize the value to 0. As long as this value is 0, llad will mask the
   triggered interrupt. When the CPUBlock is done loading the IRQ handler, it
   will set this value to all the IRQ that are now being handled by itself. Next
   time llad receives an IRQ, it will read this value and if set to the proper
   interrupt value, it will return without masking the interrupt.
   This value is a mask of all the IRQs handled by the IRQ handler (see em8xxx
   hardware IRQ register).

   *** IMPORTANT *** This value must be update in llad.c if changed.
*/
#define UCLINUX_LLAD_IRQHANDLER_HANDSHAKE    (REG_BASE_cpu_block + 0x006C)

/* these symbols are used to store the entry point of the irqhandler
   loaded by the bootloader.  when uclinux boots it overwrites the
   interrupt vector, and when we load emhwlib we must restore the RUA
   entry point in the vector. Since the current version of the emhwlib
   loaded may not match the irqhandler loaded by bootloader, the entry
   points should not be determined at compilation time, but rather at
   runtime.
*/
#define IRQHANDLER_ENTRY   (REG_BASE_cpu_block + 0x0070) 
#define FIQHANDLER_ENTRY   (REG_BASE_cpu_block + 0x0074) 
#define UNDEFHANDLER_ENTRY (REG_BASE_cpu_block + 0x0078) 
#define JUMPTABLE_ADDRESS  (REG_BASE_cpu_block + 0x007c)

/* address of linux General exeption handler */
#define LINUX_GE (REG_BASE_cpu_block + 0x0080)

// random seeds (refer to gbuslib/include/gbus_random.h)
#define RANDOM0              (REG_BASE_cpu_block + LR_RANDOM_SEED + 0)
#define RANDOM1              (REG_BASE_cpu_block + LR_RANDOM_SEED + 4)

#define PCI_INTERRUPT_ENABLE    (REG_BASE_cpu_block + LR_PCI_INTERRUPT_ENABLE)
#define HOST_INTERRUPT_STATUS   (REG_BASE_cpu_block + LR_HOST_INTERRUPT_STATUS)

// next 8 dword locations are for local debug, they are reset to 0 at vsync_init time.
// Please do not affect them in cvs source.
#define DEBUG_PROBE0                  (REG_BASE_cpu_block + LR_LOCAL_DEBUG_PROBE + 0x00)
#define DEBUG_PROBE1                  (REG_BASE_cpu_block + LR_LOCAL_DEBUG_PROBE + 0x04)
#define DEBUG_PROBE2                  (REG_BASE_cpu_block + LR_LOCAL_DEBUG_PROBE + 0x08)
#define DEBUG_PROBE3                  (REG_BASE_cpu_block + LR_LOCAL_DEBUG_PROBE + 0x0c)
#define DEBUG_PROBE4                  (REG_BASE_cpu_block + LR_LOCAL_DEBUG_PROBE + 0x10)
#define DEBUG_PROBE5                  (REG_BASE_cpu_block + LR_LOCAL_DEBUG_PROBE + 0x14)
#define DEBUG_PROBE6                  (REG_BASE_cpu_block + LR_LOCAL_DEBUG_PROBE + 0x18)
#define DEBUG_PROBE7                  (REG_BASE_cpu_block + LR_LOCAL_DEBUG_PROBE + 0x1c)

// uses 8 entries, up to 0x1EF0
#define PARAM_VSYNC_PERIOD_DEC0        (REG_BASE_cpu_block + LR_VSYNC_PERIOD)  // video decoder 0

#endif // __EMHWLIB_RESOURCES_SHARED_H__
