/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
                                                                                
/*
 * IR related definitions, and function prototypes.
 */
#ifndef _IR_H_
#define _IR_H_
                                                                                
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/ioctl.h>
#elif !defined(BOOTLOADER)
#include <sys/ioctl.h>
#endif /* __KERNEL__ */

#if defined(__KERNEL__) || !defined(BOOTLOADER)
/* ioctl commands for user level applications*/
#define IR_IOC_MAGIC           'I'
#define IR_IOCSETREPEATKEYS	_IO(IR_IOC_MAGIC, 0)
#define IR_IOCGETREPEATKEYS	_IO(IR_IOC_MAGIC, 1)
#define IR_IOCSETWAITPERIOD	_IO(IR_IOC_MAGIC, 2)
#define IR_IOCGETWAITPERIOD	_IO(IR_IOC_MAGIC, 3)
#endif /* __KERNEL__ || !BOOTLOADER */
                                                                                
#endif /* _IR_H_ */

