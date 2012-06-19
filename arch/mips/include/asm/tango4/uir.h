/*********************************************************************
 Copyright (C) 2001-2009
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
                                                                                
/*
 * UIR related definitions, and function prototypes.
 */
#ifndef _UIR_H_
#define _UIR_H_
                                                                                
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/ioctl.h>
#else
#include <sys/ioctl.h>
#endif /* __KERNEL__ */

/* ioctl commands for user level applications*/
#define UIR_IOC_MAGIC           'U'
#define UIR_IOCSETDENOISETIME	_IO(UIR_IOC_MAGIC, 0)
#define UIR_IOCGETDENOISETIME	_IO(UIR_IOC_MAGIC, 1)
#define UIR_IOCSETTIMEOUT	_IO(UIR_IOC_MAGIC, 2)
#define UIR_IOCGETTIMEOUT	_IO(UIR_IOC_MAGIC, 3)
#define UIR_IOCSETSUBSTRACT	_IO(UIR_IOC_MAGIC, 4)
#define UIR_IOCGETSUBSTRACT	_IO(UIR_IOC_MAGIC, 5)

#endif /* _UIR_H_ */

