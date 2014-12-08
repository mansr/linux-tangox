/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
 * FIP related definitions, and function prototypes.
 */
#ifndef _FIP_H_
#define _FIP_H_

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/ioctl.h>
#endif /* __KERNEL__ */

/* Valid symbols */
#if defined(CONFIG_TANGOX_FIP_REF1)
#define DVD_FIP_ON			0
#define PLAY_FIP_ON			1
#define DTS_FIP_ON			2
#define MP3_FIP_ON			3
#define DOLBYDIGITAL_FIP_ON		4
#define MPEG4_FIP_ON			5
#define PAUSE_FIP_ON			6
#define DVI_FIP_ON              	7
#define TWIRL_1_FIP_ON			8
#define TWIRL_2_FIP_ON			9
#define TWIRL_3_FIP_ON			10
#define TWIRL_4_FIP_ON			11
#define TWIRL_5_FIP_ON			12
#define TWIRL_6_FIP_ON			13
#define ALL_FIP_ON			14
#define REPEAT_FIP_ON			15
#define COLON_MIN_SEC_FIP_ON		16
#define R1080_FIP_ON			17
#define R720_FIP_ON			18
#define COLON_HOUR_MIN_FIP_ON		19
#define R480_FIP_ON			20
#define PAL_FIP_ON			21
#define NTSC_FIP_ON			22
#elif defined(CONFIG_TANGOX_FIP_REF2)
#define DVD_FIP				201
#define VCD_FIP				202
#define MP3_FIP				203
#define CD_FIP				204
#define TITLE_FIP			205
#define TRACK_CHAPTER_FIP		206
#else
#error Unsupport front panel.
#endif

/* Alignment */
#define FIP_LEFT		0x0000	/* flags for fip_write_text() */
#define FIP_CENTER		0x0001
#define FIP_RIGHT		0x0002

/* FIP Keys */
#if defined(CONFIG_TANGOX_FIP_REF1)
#define FIP_KEY_EJECT		0x00000004
#define FIP_KEY_PREV		0x00000800
#define FIP_KEY_NEXT		0x00008000
#define FIP_KEY_FBWD		0x00000008
#define FIP_KEY_FFWD		0x00000080
#define FIP_KEY_PLAYPAUSE	0x00000040
#define FIP_KEY_STOP		0x00000400
#elif defined(CONFIG_TANGOX_FIP_REF2)
#define FIP_KEY_EJECT		0x00000001
#define FIP_KEY_PLAYPAUSE	0x00000002
#define FIP_KEY_STOP		0x00000004
#define FIP_KEY_PREV		0x00000008
#define FIP_KEY_NEXT		0x00000010
#define FIP_KEY_FBWD		0x00000020
#define FIP_KEY_FFWD		0x00000040
#define FIP_KEY_MENU		0x00000080
#else
#error Unsupport front panel.
#endif

#if defined(__KERNEL__) || !defined(BOOTLOADER)
/* ioctl commands for user level applications*/
#define FIP_IOC_MAGIC		'F'
#define FIP_IOCSHOWSYMBOL	_IO(FIP_IOC_MAGIC, 0)
#define FIP_IOCSHOWHMS		_IO(FIP_IOC_MAGIC, 1)
#define FIP_IOCDISPCHAR		_IO(FIP_IOC_MAGIC, 2)
#define FIP_IOCDISPRAW		_IO(FIP_IOC_MAGIC, 3)
#define FIP_IOCDISPTEXT		_IO(FIP_IOC_MAGIC, 4)
#define FIP_IOCCLEAR		_IO(FIP_IOC_MAGIC, 5)
#define FIP_IOCGETFPTYPE	_IO(FIP_IOC_MAGIC, 6)
#endif /* __KERNEL__ || !BOOTLOADER */

#ifdef __KERNEL__
static void fip_write_text(const int position, const char *text, const int flags);
static int fip_show_hms(int hour, int minute, int second);
static void fip_display_symbol(const int symbol, const int on);
static int fip_display_character(const int position, const char character);
static void fip_display_raw(const int byte, const int bit, const int on); 
static int is_fip_busy(void);
static void fip_wait_ready(void);
static void fip_clear(void);
#elif defined(BOOTLOADER)
void fip_write_text(const int position, const char *text, const int flags);
int fip_show_hms(int hour, int minute, int second);
void fip_display_symbol(const int symbol, const int on);
int fip_display_character(const int position, const char character);
void fip_display_raw(const int byte, const int bit, const int on); 
int is_fip_busy(void);
void fip_wait_ready(void);
void fip_clear(void);
int fip_init(void);
int fip_exit(void);
unsigned long fip_readkey(void);
#endif /* __KERNEL__ */

#endif

