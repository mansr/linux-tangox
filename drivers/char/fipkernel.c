
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 
 
 Modified for KiSS Front Panel support
 By Stefan Hallas Andersen
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifdef __KERNEL__ 
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <asm/delay.h>
#include <asm/io.h>
#ifdef CONFIG_TANGO2
#include <asm/tango2/tango2_gbus.h>
#include <asm/tango2/hardware.h>
#include <asm/tango2/fip.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/hardware.h>
#include <asm/tango3/fip.h>
#endif
#else
#include "version.h"
#include "config.h"
#include "io.h"
#endif /* __KERNEL__ */

extern int tangox_fip_enabled(void);

/* chip specific register definitions */

/* EM85XX */
#if 0
#define	FIP_BASE				0x00500D00
#define	FIP_COMMAND				0x00
#define	FIP_DISPLAY_DATA			0x04
#define	FIP_LED_DATA				0x08
#define	FIP_KEY_DATA1				0x0C
#define	FIP_KEY_DATA2				0x10
#define	FIP_SWITCH_DATA				0x14
#define	FIP_CLK_DIV				0x20
#define	FIP_TRISTATE_MODE			0x24

#define FIP_TRISTATE_MODE_MASK			0x2
#define FIP_DIVIDER				40
#endif

#ifndef CONFIG_TANGOX
#error "Only TANGOX is supported/tested."
#endif

/* TANGOX */
#define	FIP_BASE				(REG_BASE_system_block + 0x500)
#define	FIP_COMMAND				0x40
#define	FIP_DISPLAY_DATA			0x44
#define	FIP_LED_DATA				0x48
#define	FIP_KEY_DATA1				0x4c
#define	FIP_KEY_DATA2				0x50
#define	FIP_SWITCH_DATA				0x54
#define FIP_CONFIG				0x58
#define FIP_INT					0x5c

#if defined(CONFIG_TANGOX_FIP_REF1)
#define FIP_DIVIDER				27	/* default value */
#elif defined(CONFIG_TANGOX_FIP_REF2)
#define FIP_DIVIDER				54	/* default value */
#endif

#define FIP_BUSY				0x200
#define FIP_ENABLE				(0x400 | 0x100) /* tri-state mode */

/* FIP commands							*/
#define	FIP_CMD_DISP_MODE_08DIGITS_20SEGMENTS		0x00
#define	FIP_CMD_DISP_MODE_09DIGITS_19SEGMENTS		0x08
#define	FIP_CMD_DISP_MODE_10DIGITS_18SEGMENTS		0x09
#define	FIP_CMD_DISP_MODE_11DIGITS_17SEGMENTS		0x0a
#define	FIP_CMD_DISP_MODE_12DIGITS_16SEGMENTS		0x0b
#define	FIP_CMD_DISP_MODE_13DIGITS_15SEGMENTS		0x0c
#define	FIP_CMD_DISP_MODE_14DIGITS_14SEGMENTS		0x0d
#define	FIP_CMD_DISP_MODE_15DIGITS_13SEGMENTS		0x0e
#define	FIP_CMD_DISP_MODE_16DIGITS_12SEGMENTS		0x0f
#define	FIP_CMD_DATA_SET_RW_MODE_WRITE_DISPLAY		0x40
#define	FIP_CMD_DATA_SET_RW_MODE_WRITE_LED_PORT		0x41
#define	FIP_CMD_DATA_SET_RW_MODE_READ_KEYS		0x42
#define	FIP_CMD_DATA_SET_RW_MODE_READ_SWITCHES		0x43
#define	FIP_CMD_DATA_SET_ADR_MODE_INCREMENT_ADR		0x40
#define	FIP_CMD_DATA_SET_ADR_MODE_FIXED_ADR		0x44
#define	FIP_CMD_DATA_SET_OP_MODE_NORMAL_OPERATION	0x40
#define	FIP_CMD_DATA_SET_OP_MODE_TEST_MODE		0x48
#define	FIP_CMD_ADR_SETTING				0xC0
#define	FIP_CMD_DISP_CTRL_PULSE_WIDTH_1_16		0x80
#define	FIP_CMD_DISP_CTRL_PULSE_WIDTH_2_16		0x81
#define	FIP_CMD_DISP_CTRL_PULSE_WIDTH_4_16		0x82
#define	FIP_CMD_DISP_CTRL_PULSE_WIDTH_10_16		0x83
#define	FIP_CMD_DISP_CTRL_PULSE_WIDTH_11_16		0x84
#define	FIP_CMD_DISP_CTRL_PULSE_WIDTH_12_16		0x85
#define	FIP_CMD_DISP_CTRL_PULSE_WIDTH_13_16		0x86
#define	FIP_CMD_DISP_CTRL_PULSE_WIDTH_14_16		0x87
#define	FIP_CMD_DISP_CTRL_TURN_DISPLAY_OFF_MASK		0x87
#define	FIP_CMD_DISP_CTRL_TURN_DISPLAY_ON		0x88

#if defined(CONFIG_TANGOX_FIP_REF1)
#define FIP_DISPLAY_MODE	0xa
#define MAX_FIP_RAM		23
#define NUM_SYMBOLS		23
#define NUM_CHARACTERS		72
#define NUM_DIGITS		7
#elif defined(CONFIG_TANGOX_FIP_REF2)
#define FIP_DISPLAY_MODE	0x0a
#define MAX_FIP_RAM		0x2f
#define NUM_SYMBOLS		206
#define NUM_CHARACTERS		69
#define NUM_X_CHARACTERS	12
#define NUM_DIGITS		8
#define NUM_X_DIGITS		4
#endif

#define L_OFF			-1	//means light is or should be off
#define FIP_NO_CLEAR		0x0004

#if defined(CONFIG_TANGOX_FIP_REF1)
/*
  14 SEGMENT LCD (EXTENDED CHARACTERS MAP)

     a 
   -----
f |\j| /| b 
  |i\|/k|
  g-- --h
e |n/|\l| c
  |/m| \|
   -----
     d   
*/
#define DIGIT_L(b7,b6,c,l,m,n,e,d)	((b7 << 7) | (b6 << 6) | (c << 5) | (l << 4) | (m << 3) | (n << 2) | (e << 1) | d)
#define DIGIT_H(g,h,i,j,k,a,b,f)	((g << 7) | (h << 6) | (i << 5) | (j << 4) | (k << 3) | (a << 2) | (b << 1) | f)

/* sequence must match fipcharacters */
static const char fipcharactersmap[NUM_CHARACTERS+1] = " +-/0123456789<>ABCDEFGHIJKLMNOPQRSTUVWXYZ\\-abcdefghijklmnopqrstuvwxyz|_";
#elif defined(CONFIG_TANGOX_FIP_REF2)
#define DIGIT_L(M,C,E,R,P,B,D,U)	((M << 7) | (C << 6) | (E << 5) | (R << 4) | (P << 3) | (B << 2) | (D << 1) | U)
#define DIGIT_H(A,B,F,H,J,K,G,S)	((A << 7) | (B << 6) | (F << 5) | (H << 4) | (J << 3) | (K << 2) | (G << 1) | S)
// Following definition is for Title/Track/Chaspter digits
#define DIGIT_X(A,B,F,G,C,E,D,U)		((A << 7) | (B << 6) | (F << 5) | (G << 4) | (C << 3) | (E << 2) | (D << 1) | U)
 
static const char fipcharactersmap[NUM_CHARACTERS+1] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~";
static const char fipxcharactersmap[NUM_X_CHARACTERS+1] = "0123456789- ";
#endif

/* we use the inverted mask for clearing a digit without clearing other things */
static const char fipcharactermask[2] = {
#if defined(CONFIG_TANGOX_FIP_REF1)
	DIGIT_L(1,1,0,0,0,0,0,0), 
	DIGIT_H(0,0,0,0,0,0,0,0)
#elif defined(CONFIG_TANGOX_FIP_REF2)
	DIGIT_L(0,0,0,0,0,0,0,0), 
	DIGIT_H(0,0,0,0,0,0,0,0)
#endif
};

#if defined(CONFIG_TANGOX_FIP_REF1)
/* the format is lower byte, higher byte */
static const char fipcharacters[NUM_CHARACTERS][2] = {
	{DIGIT_L(0,0,0,0,0,0,0,0), DIGIT_H(0,0,0,0,0,0,0,0)},	//
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(1,1,0,1,0,0,0,0)},	// +
	{DIGIT_L(0,0,0,0,0,0,0,0), DIGIT_H(1,1,0,0,0,0,0,0)},	// -
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,0,1,0,0,0)},	// /

	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(0,0,0,0,0,1,1,1)},	// 0
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,1,0,0,0,0)},	// 1
	{DIGIT_L(0,0,0,0,0,0,1,1), DIGIT_H(1,1,0,0,0,1,1,0)},	// 2
	{DIGIT_L(0,0,1,0,0,0,0,1), DIGIT_H(1,1,0,0,0,1,1,0)},	// 3
	{DIGIT_L(0,0,1,0,0,0,0,0), DIGIT_H(1,1,0,0,0,0,1,1)},	// 4
	{DIGIT_L(0,0,1,0,0,0,0,1), DIGIT_H(1,1,0,0,0,1,0,1)},	// 5
	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(1,1,0,0,0,1,0,1)},	// 6
	{DIGIT_L(0,0,1,0,0,0,0,0), DIGIT_H(0,0,0,0,0,1,1,0)},	// 7
	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(1,1,0,0,0,1,1,1)},	// 8
	{DIGIT_L(0,0,1,0,0,0,0,1), DIGIT_H(1,1,0,0,0,1,1,1)},	// 9

	{DIGIT_L(0,0,0,1,0,0,0,0), DIGIT_H(0,0,0,0,1,0,0,0)},	// <
	{DIGIT_L(0,0,0,0,0,1,0,0), DIGIT_H(0,0,0,1,0,0,0,0)},	// >

	{DIGIT_L(0,0,1,0,0,0,1,0), DIGIT_H(1,1,0,0,0,1,1,1)},	// A
	{DIGIT_L(0,0,1,0,1,0,0,1), DIGIT_H(0,1,0,1,0,1,1,0)},	// B
	{DIGIT_L(0,0,0,0,0,0,1,1), DIGIT_H(0,0,0,0,0,1,0,1)},	// C
	{DIGIT_L(0,0,1,0,1,0,0,1), DIGIT_H(0,0,0,1,0,1,1,0)},	// D
	{DIGIT_L(0,0,0,0,0,0,1,1), DIGIT_H(1,1,0,0,0,1,0,1)},	// E
	{DIGIT_L(0,0,0,0,0,0,1,0), DIGIT_H(1,1,0,0,0,1,0,1)},	// F
	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(1,1,0,0,0,1,0,1)},	// G
	{DIGIT_L(0,0,1,0,0,0,1,0), DIGIT_H(1,1,0,0,0,0,1,1)},	// H
	{DIGIT_L(0,0,0,0,1,0,0,1), DIGIT_H(0,0,0,1,0,1,0,0)},	// I
	{DIGIT_L(0,0,1,0,0,0,0,1), DIGIT_H(0,0,0,0,0,0,1,0)},	// J
	{DIGIT_L(0,0,0,1,0,0,1,0), DIGIT_H(1,0,0,0,1,0,0,1)},	// K
	{DIGIT_L(0,0,0,0,0,0,1,1), DIGIT_H(0,0,0,0,0,0,0,1)},	// L
	{DIGIT_L(0,0,1,0,0,0,1,0), DIGIT_H(0,0,1,0,1,0,1,1)},	// M
	{DIGIT_L(0,0,1,1,0,0,1,0), DIGIT_H(0,0,1,0,0,0,1,1)},	// N
	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(0,0,0,0,0,1,1,1)},	// O
	{DIGIT_L(0,0,0,0,0,0,1,0), DIGIT_H(1,1,0,0,0,1,1,1)},	// P
	{DIGIT_L(0,0,1,1,0,0,1,1), DIGIT_H(0,0,0,0,0,1,1,1)},	// Q
	{DIGIT_L(0,0,0,1,0,0,1,0), DIGIT_H(1,1,0,0,0,1,1,1)},	// R
	{DIGIT_L(0,0,1,0,0,0,0,1), DIGIT_H(1,1,0,0,0,1,0,1)},	// S
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,1,0,1,0,0)},	// T
	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(0,0,0,0,0,0,1,1)},	// U
	{DIGIT_L(0,0,1,1,0,0,0,0), DIGIT_H(0,0,1,0,0,0,1,0)},	// V
	{DIGIT_L(0,0,1,1,0,1,1,0), DIGIT_H(0,0,0,0,0,0,1,1)},	// W
	{DIGIT_L(0,0,0,1,0,1,0,0), DIGIT_H(0,0,1,0,1,0,0,0)},	// X
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,1,0,1,0,0,0)},	// Y
	{DIGIT_L(0,0,0,0,0,1,0,1), DIGIT_H(0,0,0,0,1,1,0,0)},	// Z

	{DIGIT_L(0,0,0,1,0,0,0,0), DIGIT_H(0,0,1,0,0,0,0,0)},	// Slash
	{DIGIT_L(0,0,0,0,0,0,0,0), DIGIT_H(1,1,0,0,0,0,0,0)},	// -

	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(1,1,0,0,0,0,0,0)},	// a
	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(1,1,0,0,0,0,0,1)},	// b
	{DIGIT_L(0,0,0,0,0,0,1,1), DIGIT_H(1,1,0,0,0,0,0,0)},	// c
	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(1,1,0,0,0,0,1,0)},	// d
	{DIGIT_L(0,0,0,0,0,0,1,1), DIGIT_H(1,1,0,0,0,1,1,1)},	// e
	{DIGIT_L(0,0,0,0,0,0,1,0), DIGIT_H(1,0,0,0,0,1,0,1)},	// f
	{DIGIT_L(0,0,1,0,0,0,0,1), DIGIT_H(1,1,0,0,0,1,1,1)},	// g
	{DIGIT_L(0,0,1,0,0,0,1,0), DIGIT_H(1,1,0,0,0,0,0,1)},	// h
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,0,0,0,0,0)},	// i
	{DIGIT_L(0,0,1,0,0,0,0,1), DIGIT_H(0,0,0,0,0,0,0,0)},	// j
	{DIGIT_L(0,0,0,1,1,0,0,0), DIGIT_H(0,1,0,1,0,0,0,0)},	// k
	{DIGIT_L(0,0,0,0,0,0,1,1), DIGIT_H(0,0,0,0,0,0,0,1)},	// l
	{DIGIT_L(0,0,1,0,1,0,1,0), DIGIT_H(1,1,0,0,0,0,0,0)},	// m
	{DIGIT_L(0,0,1,0,0,0,1,0), DIGIT_H(1,1,0,0,0,0,0,0)},	// n
	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(1,1,0,0,0,0,0,0)},	// o
	{DIGIT_L(0,0,0,0,0,0,1,0), DIGIT_H(1,1,0,0,0,1,1,1)},	// p
	{DIGIT_L(0,0,1,0,0,0,0,0), DIGIT_H(1,1,0,0,0,1,1,1)},	// q
	{DIGIT_L(0,0,0,0,0,0,1,0), DIGIT_H(1,0,0,0,0,0,0,0)},	// r
	{DIGIT_L(0,0,1,0,0,0,0,1), DIGIT_H(1,1,0,0,0,1,0,1)},	// s
	{DIGIT_L(0,0,0,0,0,0,1,1), DIGIT_H(1,0,0,0,0,0,0,1)},	// t
	{DIGIT_L(0,0,1,0,0,0,1,1), DIGIT_H(0,0,0,0,0,0,0,0)},	// u
	{DIGIT_L(0,0,1,1,0,0,0,0), DIGIT_H(0,0,0,0,0,0,0,0)},	// v
	{DIGIT_L(0,0,1,1,0,1,1,0), DIGIT_H(0,0,0,0,0,0,0,0)},	// w
	{DIGIT_L(0,0,0,1,1,0,0,0), DIGIT_H(1,1,0,0,0,0,0,0)},	// x
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,1,1,0,0,0)},	// y
	{DIGIT_L(0,0,0,0,1,0,0,1), DIGIT_H(1,1,0,0,0,0,0,0)},	// z

	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,1,0,0,0,0)},	// |
	{DIGIT_L(0,0,0,0,0,0,0,1), DIGIT_H(0,0,0,0,0,0,0,0)}	// _
};
#elif defined(CONFIG_TANGOX_FIP_REF2)
/* the format is lower byte, higher byte */
static const char fipcharacters[NUM_CHARACTERS][2] = {
	{DIGIT_L(0,1,1,0,0,0,1,1), DIGIT_H(1,1,1,0,0,0,0,0)},	// 0
	{DIGIT_L(0,0,0,0,1,0,0,1), DIGIT_H(0,0,0,0,1,0,0,1)},	// 1
	{DIGIT_L(1,0,1,0,0,0,1,1), DIGIT_H(1,1,0,0,0,0,1,1)},	// 2
	{DIGIT_L(1,1,0,0,0,0,1,1), DIGIT_H(1,1,0,0,0,0,0,0)},	// 3
	{DIGIT_L(1,1,0,0,0,0,0,1), DIGIT_H(0,1,1,0,0,0,1,1)},	// 4
	{DIGIT_L(1,1,0,0,0,0,1,1), DIGIT_H(1,0,1,0,0,0,1,1)},	// 5
	{DIGIT_L(1,1,1,0,0,0,1,1), DIGIT_H(1,0,1,0,0,0,1,1)},	// 6
	{DIGIT_L(0,1,0,0,0,0,0,1), DIGIT_H(1,1,0,0,0,0,0,0)},	// 7
	{DIGIT_L(1,1,1,0,0,0,1,1), DIGIT_H(1,1,1,0,0,0,1,1)},	// 8
	{DIGIT_L(1,1,0,0,0,0,0,1), DIGIT_H(1,1,1,0,0,0,1,1)},	// 9
	{DIGIT_L(1,1,1,0,0,0,0,0), DIGIT_H(1,1,1,0,0,0,1,1)},	// A
	{DIGIT_L(1,1,0,0,1,0,1,0), DIGIT_H(1,1,0,0,1,0,0,1)},	// B
	{DIGIT_L(0,0,1,0,0,0,1,0), DIGIT_H(1,0,1,0,0,0,0,0)},	// C
	{DIGIT_L(0,1,0,0,1,0,1,0), DIGIT_H(1,1,0,0,1,0,0,1)},	// D
	{DIGIT_L(1,0,1,0,0,0,1,0), DIGIT_H(1,0,1,0,0,0,1,1)},	// E
	{DIGIT_L(0,0,1,0,0,0,0,0), DIGIT_H(1,0,1,0,0,0,1,1)},	// F
	{DIGIT_L(1,1,1,0,0,0,1,0), DIGIT_H(1,0,1,0,0,0,0,0)},	// G
	{DIGIT_L(1,1,1,0,0,0,0,0), DIGIT_H(0,1,1,0,0,0,1,1)},	// H
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,0,1,0,0,1)},	// I
	{DIGIT_L(0,1,1,0,0,0,1,0), DIGIT_H(0,1,0,0,0,0,0,0)},	// J
	{DIGIT_L(0,0,1,0,0,1,0,0), DIGIT_H(0,0,1,0,0,1,1,1)},	// K
	{DIGIT_L(0,0,1,0,0,0,1,0), DIGIT_H(0,0,1,0,0,0,0,0)},	// L
	{DIGIT_L(0,1,1,0,0,0,0,0), DIGIT_H(0,1,1,1,0,1,0,1)},	// M
	{DIGIT_L(0,1,1,0,0,1,0,0), DIGIT_H(0,1,1,1,0,0,0,1)},	// N
	{DIGIT_L(0,1,1,0,0,0,1,0), DIGIT_H(1,1,1,0,0,0,0,0)},	// O
	{DIGIT_L(1,0,1,0,0,0,0,0), DIGIT_H(1,1,1,0,0,0,1,1)},	// P
	{DIGIT_L(0,1,1,0,0,1,1,0), DIGIT_H(1,1,1,0,0,0,0,0)},	// Q
	{DIGIT_L(1,0,1,0,0,1,0,0), DIGIT_H(1,1,1,0,0,0,1,1)},	// R
	{DIGIT_L(1,1,0,0,0,0,1,0), DIGIT_H(1,0,1,0,0,0,1,1)},	// S
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(1,0,0,0,1,0,0,1)},	// T
	{DIGIT_L(0,1,1,0,0,0,1,0), DIGIT_H(0,1,1,0,0,0,0,0)},	// U
	{DIGIT_L(0,0,1,1,0,0,0,0), DIGIT_H(0,0,1,0,0,1,0,1)},	// V
	{DIGIT_L(0,1,1,1,0,1,0,0), DIGIT_H(0,1,1,0,0,0,0,1)},	// W
	{DIGIT_L(0,0,0,1,0,1,0,0), DIGIT_H(0,0,0,1,0,1,0,0)},	// X
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,1,0,1,0,1)},	// Y
	{DIGIT_L(0,0,0,1,0,0,1,0), DIGIT_H(1,0,0,0,0,1,0,1)},	// Z
	{0,0},							// Space
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,0,1,0,0,1)},	// !
	{DIGIT_L(0,0,0,0,0,0,0,0), DIGIT_H(0,0,1,0,1,0,0,0)},	// "
	{DIGIT_L(1,0,0,1,1,0,0,0), DIGIT_H(0,0,0,1,1,0,1,1)},	// #
	{DIGIT_L(1,1,0,0,1,0,1,0), DIGIT_H(1,0,1,0,1,0,1,1)},	// $
	{DIGIT_L(0,1,0,1,0,0,0,0), DIGIT_H(0,0,1,0,0,1,0,1)},	// %
	{DIGIT_L(0,0,1,0,1,0,1,0), DIGIT_H(1,0,0,1,0,1,1,1)},	// &
	{DIGIT_L(0,0,0,0,0,0,0,0), DIGIT_H(0,0,0,0,0,1,0,0)},	// '
	{DIGIT_L(0,0,0,0,0,1,0,0), DIGIT_H(0,0,0,0,0,1,0,0)}, 	// (
	{DIGIT_L(0,0,0,1,0,0,0,0), DIGIT_H(0,0,0,1,0,0,0,0)}, 	// )
	{DIGIT_L(1,0,0,1,1,1,0,0), DIGIT_H(0,0,0,1,1,1,1,1)}, 	// *
	{DIGIT_L(1,0,0,0,1,0,0,0), DIGIT_H(0,0,0,0,1,0,1,1)}, 	// +
	{DIGIT_L(0,0,0,1,0,0,0,0), DIGIT_H(0,0,0,0,0,0,0,0)}, 	// ,
	{DIGIT_L(1,0,0,0,0,0,0,0), DIGIT_H(0,0,0,0,0,0,1,1)}, 	// -
	{DIGIT_L(0,0,0,0,0,0,0,0), DIGIT_H(0,0,0,0,0,0,0,1)}, 	// .
	{DIGIT_L(0,0,0,1,0,0,0,0), DIGIT_H(0,0,0,0,0,1,0,1)}, 	// /
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,0,0,0,0,1)}, 	// :
	{DIGIT_L(0,0,0,1,0,0,0,0), DIGIT_H(0,0,0,0,0,0,0,1)}, 	// ;
	{DIGIT_L(0,0,0,0,0,1,0,0), DIGIT_H(0,0,0,0,0,1,0,1)}, 	// <
	{DIGIT_L(1,0,0,0,0,0,1,0), DIGIT_H(0,0,0,0,0,0,1,1)}, 	// =
	{DIGIT_L(0,0,0,1,0,0,0,0), DIGIT_H(0,0,0,1,0,0,0,1)}, 	// >
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(1,0,1,0,0,1,0,1)}, 	// ?
	{DIGIT_L(1,0,1,0,0,0,1,0), DIGIT_H(1,1,1,0,1,1,0,1)}, 	// @
	{DIGIT_L(0,0,0,1,0,0,1,0), DIGIT_H(1,0,0,1,0,0,0,0)}, 	// [
	{DIGIT_L(0,0,0,0,0,1,0,0), DIGIT_H(0,0,0,1,0,0,0,1)}, 	// / 
	{DIGIT_L(0,1,0,0,0,0,1,0), DIGIT_H(1,1,0,0,0,0,0,0)}, 	// ]
	{DIGIT_L(0,0,0,1,0,1,0,0), DIGIT_H(0,0,0,0,0,0,0,1)}, 	// ^
	{DIGIT_L(0,0,0,0,0,0,1,0), DIGIT_H(0,0,0,0,0,0,0,0)}, 	// _
	{DIGIT_L(0,0,0,0,0,0,0,0), DIGIT_H(0,0,0,1,0,0,0,0)}, 	// `
	{DIGIT_L(0,0,0,1,0,0,1,0), DIGIT_H(1,0,0,1,0,0,1,0)}, 	// {
	{DIGIT_L(0,0,0,0,1,0,0,0), DIGIT_H(0,0,0,0,1,0,0,1)}, 	// |
	{DIGIT_L(1,0,0,0,0,1,1,0), DIGIT_H(1,0,0,0,0,1,0,1)}, 	// }
	{DIGIT_L(1,0,0,0,0,0,0,0), DIGIT_H(0,0,0,0,0,0,1,0)}	// ~
};

static const char fipxcharacters[NUM_X_CHARACTERS] = {
	DIGIT_X(1,1,1,0,1,1,1,1),	// 0
	DIGIT_X(0,1,0,0,1,0,0,1),	// 1
	DIGIT_X(1,1,0,1,0,1,1,1),	// 2
	DIGIT_X(1,1,0,1,1,0,1,1),	// 3
	DIGIT_X(0,1,1,1,1,0,0,1),	// 4
	DIGIT_X(1,0,1,1,1,0,1,1),	// 5
	DIGIT_X(1,0,1,1,1,1,1,1),	// 6
	DIGIT_X(1,1,0,0,1,0,0,1),	// 7
	DIGIT_X(1,1,1,1,1,1,1,1),	// 8
	DIGIT_X(1,1,1,1,1,0,1,1),	// 9
	DIGIT_X(0,0,0,1,0,0,0,0),	// -
	DIGIT_X(0,0,0,0,0,0,0,0)	// 
};
#endif

#if defined(CONFIG_TANGOX_FIP_REF1)
/* this array is used to display individual symbols
   the format is [byte position][bit to turn on] - both zero based */
static const char fipsymbols[NUM_SYMBOLS][2] = {
	{0, 0}, 	/* DVD */
	{0, 1}, 	/* PLAY */
	{0, 2}, 	/* DTS */
	{0, 3}, 	/* MP3 */
	{0, 4}, 	/* DOLBY DIGITAL */
	{0, 5}, 	/* MPEG4 */
	{0, 6},		/* PAUSE */
	{0, 7}, 	/* DVI */
	{1, 0}, 	/* TWIRL1 */
	{1, 1}, 	/* TWIRL2 */
	{1, 2}, 	/* TWIRL3 */
	{1, 3}, 	/* TWIRL4 */
	{1, 4}, 	/* TWIRL5 */
	{1, 5}, 	/* TWIRL6 */
	{1, 6}, 	/* ALL */
	{1, 7}, 	/* REPEAT */
	{9, 6},		/* COLON_MIN_SEC */
	{9, 7}, 	/* R1080 */
	{12, 7}, 	/* R720 */
	{15, 6}, 	/* COLON_HOUR_MIN */
	{15, 7}, 	/* R480 */
	{18, 7}, 	/* PAL */
	{21, 7},	/* NTSC */
};
#elif defined(CONFIG_TANGOX_FIP_REF2)
/* this array is used to display individual symbols
   the format is [byte position][bit to turn on] - both zero based */
static const char fipsymbols[NUM_SYMBOLS][2] = {
	{0, 1},		// DVD
	{0, 2},		// VCD
	{0, 4},		// MP3
	{0, 8}, 	// CD
	{29, 1}, 	// Title
	{27, 1}		// Track
};
#endif

/* The buffer size defines the size of circular buffer to keep the FIP keys */
#define BUF_SIZE		2

/* Wait period, to avoid bouncing or repeatation? */
#define WAIT_PERIOD		100

/* Default brightness level */
#define BRIGHTNESS		0x7

/* The number of key polling per second */
#define POLL_PER_SECOND		10

#ifdef __KERNEL__
/* The major device number and name */
#define FIP_DEV_MAJOR		0

#define DRIVER_VERSION         "1.15"

MODULE_DESCRIPTION("TANGOX front panel fip driver\n");
MODULE_AUTHOR("TANGOX standalone team");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

/* Wait queue, may be used if block mode is on */
DECLARE_WAIT_QUEUE_HEAD(fip_wq);

static int major_num = FIP_DEV_MAJOR;
static int buffer_size = BUF_SIZE;
static int wait_period = WAIT_PERIOD;
static int brightness = BRIGHTNESS;
static unsigned int poll_per_sec = POLL_PER_SECOND;
module_param(major_num, int, 0);
module_param(buffer_size, int, 0);
module_param(wait_period, int, 0);
module_param(brightness, int, 0);
module_param(poll_per_sec, int, 0);

/* Some prototypes */
static int fip_open(struct inode *, struct file *);
static int fip_release(struct inode *, struct file *);
static int fip_read(struct file *, char *, size_t, loff_t *);
static int fip_write(struct file *, const char *, size_t, loff_t *);
static int fip_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
static unsigned int fip_poll(struct file *, struct poll_table_struct *);
static int fip_irq = LOG2_CPU_FRONTPANEL_INT + IRQ_CONTROLLER_IRQ_BASE;
static irqreturn_t fip_isr(int irq, void *dev_id);
static void fip_poll_key(unsigned long devid);

static struct timer_list fip_timer;
#if 0
static struct tq_struct immediate;
#endif

static void fip_write_text(const int position, const char *text, const int flags);
static int fip_show_hms(int hour, int minute, int second);
static void fip_display_symbol(const int symbol, const int on);
static int fip_display_character(const int position, const char character);
static void fip_display_raw(const int byte, const int bit, const int on); 
static int is_fip_busy(void);
static int is_fip_busy_nowait(void);
static void fip_clear(void);

#else /* For Bootloader */

void fip_write_text(const int position, const char *text, const int flags);
void fip_display_symbol(const int symbol, const int on);
int fip_display_character(const int position, const char character);
void fip_display_raw(const int byte, const int bit, const int on); 
int is_fip_busy(void);
void fip_clear(void);
int fip_init(void);
int fip_exit(void);

/* Some external functions from bootloader core */
void em86xx_usleep(int usec);
int uart_printf(const char *fmt, ...);
int strlen(const char *str);

#endif /* __KERNEL__ */

#define FIP_DEV_NAME		"fip"

static void fip_write_reg(unsigned int offset, unsigned int val);
static unsigned int fip_read_reg(unsigned int offset);

#ifdef __KERNEL__ 
#define CMDQ_SIZE	256

/* Private data structure */
struct cmd_request {
	unsigned int cmd;
	unsigned int data;
};

struct fip_private {
	unsigned long *buffer;		/* Circular buffer */
	unsigned p_idx;			/* Index of producer */
	unsigned c_idx; 		/* Index of consumer */
	unsigned ref_cnt;		/* Reference count */
	spinlock_t lock;		/* Spin lock */
	unsigned char b_mode;		/* Blocking mode or not */
	unsigned long last_jiffies;	/* Timestamp for last reception */
#ifdef ENABLE_WRITE_INTR
	struct cmd_request cmds[CMDQ_SIZE];
	unsigned cmd_pidx;
	unsigned cmd_cidx;
	unsigned cmdq_empty;
#endif
};

static struct file_operations fip_fops = {
	open: fip_open,
	read: fip_read,
	ioctl: fip_ioctl,
	poll: fip_poll,
	write: fip_write,
	release: fip_release,
	owner: THIS_MODULE,
};

static void fip_push_key(struct fip_private *priv, unsigned long key);
#ifdef ENABLE_WRITE_INTR
static void fip_issue_command(struct fip_private *priv);
static void fip_queue_command(unsigned int cmd, unsigned int data);
#endif

/* Global data */
static struct fip_private fip_priv;
#endif /* __KERNEL__ */

static char *fip_devname = FIP_DEV_NAME;
static unsigned long fip_base = (unsigned long)FIP_BASE;
static char fipram[MAX_FIP_RAM] = {0};

#ifdef __KERNEL__

static irqreturn_t fip_isr(int irq, void *dev_id)
{
	struct fip_private *priv = (struct fip_private *)dev_id;
	unsigned long stat;
	unsigned long key;

	if (irq == fip_irq) {
		//gbus_write_reg32(REG_BASE_CPU + CPU_edge_rawstat,  IRQMASKOF(irq));
		stat = (fip_read_reg(FIP_INT) & 0x3);
		fip_write_reg(FIP_INT, stat); /* Clear the interrupt */

		if (!is_fip_busy_nowait()) {
			if (stat & 0x2) {
				key = fip_read_reg(FIP_KEY_DATA1);
				fip_push_key(priv, key);
			}
#ifdef ENABLE_WRITE_INTR
			fip_issue_command(priv);
#endif
		}
		return IRQ_HANDLED;
	} else {
		printk("Unknown IRQ %d\n", irq);
		return IRQ_NONE;
	}

}

static void fip_poll_key(unsigned long devid)
{
	struct fip_private *priv = (struct fip_private *)devid;

	/* Sending command, and later ISR will pick up the interrupt and read
           the key */
	if (!is_fip_busy_nowait()) {
		fip_write_reg(FIP_COMMAND, FIP_CMD_DATA_SET_RW_MODE_READ_KEYS);
#ifdef ENABLE_WRITE_INTR
		if (priv->cmdq_empty != 0) {
			fip_issue_command(priv);
		}
#endif
	}

	if (priv->ref_cnt != 0) {
		mod_timer(&fip_timer, jiffies + (HZ / poll_per_sec));
#if 0
		queue_task(&immediate, &tq_immediate);
		mark_bh(IMMEDIATE_BH);
#endif
	}
}

/* Produce data */
static void fip_push_key(struct fip_private *priv, unsigned long key)
{
	unsigned pidx;
	static unsigned long oldkey = 0;

	spin_lock(&priv->lock);

	if ((key == 0) || (key == 0xffffffff)) {
		oldkey = 0;
		goto out;
	} else if (time_after(priv->last_jiffies + wait_period, jiffies) 
			&& (key == oldkey))
		goto out;
	else
		priv->last_jiffies = jiffies;
	
	printk(KERN_DEBUG "%s: got data 0x%08lx\n", fip_devname, key);

	pidx = priv->p_idx;	/* Save the old index before proceeding */

	/* Save it to buffer */
	if (((priv->p_idx + 1) % buffer_size) == priv->c_idx) {
		/* Adjust consumer index since buffer is full */
		/* Keep the latest one and drop the oldest one */
		priv->c_idx = (priv->c_idx + 1) % buffer_size;

		printk(KERN_WARNING "%s: buffer full\n", fip_devname);
	}

	priv->buffer[priv->p_idx] = oldkey = key;
	priv->p_idx = (priv->p_idx + 1) % buffer_size;

	/* Buffer was empty and block mode is on, wake up the reader */
	if ((priv->b_mode != 0) && (priv->c_idx == pidx))
		wake_up_interruptible(&fip_wq);

out:
	spin_unlock(&priv->lock);
}

/* Reading from driver's buffer, note that it can return read size
   less than specified */
static int fip_consume(void *dev_id, unsigned long *buf, int count)
{
	struct fip_private *priv = (struct fip_private *)dev_id;
	int cnt = 0;
	unsigned long flags;

	/* If block mode is on, check the emptiness of buffer */
	if (priv->b_mode != 0) {
		/* Sleep when buffer is empty */
		wait_event_interruptible(fip_wq, priv->c_idx != priv->p_idx);
	}
	if (signal_pending(current))
		return(cnt);

	spin_lock_irqsave(&priv->lock, flags);

	/* Get the data out and adjust consumer index */
	for (cnt = 0; (priv->c_idx != priv->p_idx) && (cnt < count); cnt++) {
		*buf = priv->buffer[priv->c_idx];
		priv->c_idx = (priv->c_idx + 1) % buffer_size;
		buf++;
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return(cnt);
}

/* Poll function */
static unsigned int fip_poll(struct file *fptr, struct poll_table_struct *ptable){
	struct fip_private *priv = (struct fip_private *)fptr->private_data;
	unsigned int mask = 0;

	poll_wait(fptr, &fip_wq, ptable);
	if (priv->c_idx != priv->p_idx)
		mask |= (POLLIN | POLLRDNORM);
	return(mask);
}

#ifdef ENABLE_WRITE_INTR
static void fip_issue_command(struct fip_private *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (priv->cmd_pidx == priv->cmd_cidx) {
		priv->cmdq_empty = 1;
		goto out;
	} else if (!is_fip_busy_nowait()) {
		priv->cmdq_empty = 0;
		fip_wait_ready();
		fip_write_reg(FIP_DISPLAY_DATA, priv->cmds[priv->cmd_cidx].data);
		fip_wait_ready();
		fip_write_reg(FIP_COMMAND, priv->cmds[priv->cmd_cidx].cmd);
		fip_wait_ready();
		priv->cmd_cidx = ((priv->cmd_cidx + 1) % CMDQ_SIZE);
		if (priv->cmd_pidx == priv->cmd_cidx) 
			priv->cmdq_empty = 1;
	}
out:
	spin_unlock_irqrestore(&priv->lock, flags);
}
#endif

static int fip_open(struct inode *inode_ptr, struct file *fptr)
{
	/* This device is exclusive, that is, only one process can use it */
	if (fip_priv.ref_cnt != 0) {
		printk(KERN_WARNING "%s: exclusive access only\n", fip_devname);
		return(-EIO);
	} 

	/* Set the block mode and increase reference count */
	fip_priv.ref_cnt++;
	fip_priv.b_mode = ((fptr->f_flags & O_NONBLOCK) ? 0 : 1);
	fip_priv.last_jiffies = jiffies;

	/* Flush the buffer */
	fip_priv.p_idx = fip_priv.c_idx = 0;

	fptr->f_op = &fip_fops;
	fptr->private_data = (void *)&fip_priv;

	fip_timer.function = fip_poll_key;
	fip_timer.data = (unsigned long)&fip_priv;
	fip_timer.expires = jiffies + (HZ / poll_per_sec);
	add_timer(&fip_timer);

#if 0
	immediate.sync = 0;
	immediate.data = (void *)&fip_priv;
	immediate.routine = fip_poll_key;
	queue_task(&immediate, &tq_immediate);
	mark_bh(IMMEDIATE_BH);
#endif

	fip_clear();
	return(0);
}

static int fip_release(struct inode *inode_ptr, struct file *fptr) 
{
	unsigned long start = 0;

	/* Adjust reference count */
	fip_priv.ref_cnt--;

	/* Wait for timer expiration */
	for (start = jiffies; time_after(start + 2 * (HZ / poll_per_sec), jiffies););
	del_timer_sync(&fip_timer);
	return(0);
}

static int fip_read(struct file *fptr, char *bufptr, size_t size, loff_t *fp)
{
	unsigned long buf[buffer_size];
	int count = 0;

	/* Check the alignment */
	if (size % sizeof(unsigned long)) {
		printk(KERN_WARNING "%s: read size not aligned to %d\n",
			fip_devname, sizeof(unsigned long));
		return(-EIO);
	}

	count = fip_consume(fptr->private_data, &buf[0], 
			size / sizeof(unsigned long)) * sizeof(unsigned long);

	/* Get the data to user */
	if (count && copy_to_user(bufptr, (char *)&buf[0], count)) 
		return(-EFAULT);

	return(count);
}

static int fip_write(struct file *fptr, const char *bufptr, size_t size, loff_t *fp)
{
	fip_wait_ready();
	fip_write_text(0, bufptr, FIP_CENTER); 
	fip_wait_ready();
	return(size);
}

static int fip_ioctl(struct inode *inode, struct file *fptr, unsigned int cmd, unsigned long arg)
{
	int on = 0;
	int symbol;
	int i, j, k;

	switch(cmd) {
		case FIP_IOCSHOWSYMBOL:
			on = ((arg & 0x80000000)) == 0 ? 0 : 1;
			symbol = (int)(arg & 0xff);
			fip_display_symbol(symbol, on);
			break;
		case FIP_IOCSHOWHMS:
			k = (int)(arg & 0xff);
			j = (int)((arg >> 8) & 0xff);
			i = (int)((arg >> 16) & 0xff);
			fip_show_hms(i, j, k);
			break;
		case FIP_IOCDISPCHAR:
			k = (int)(arg & 0xff);
			j = (int)((arg >> 8) & 0xff);
			fip_display_character(j, k);
			break;
		case FIP_IOCDISPRAW:
			on = ((arg & 0x80000000)) == 0 ? 0 : 1;
			k = (int)(arg & 0xff);
			j = (int)((arg >> 8) & 0xff);
			fip_display_raw(j, k, on);
			break;
		case FIP_IOCDISPTEXT:
			printk("%s: ioctl FIP_IOCDISPTEXT not implemented.\n", fip_devname);
			return(-EIO);
		case FIP_IOCCLEAR:
			fip_clear();
			break;
		case FIP_IOCGETFPTYPE:
			{
				unsigned long *ptr = (unsigned long *)arg;
				unsigned long type = 0;
#if defined(CONFIG_TANGOX_FIP_REF1)
				type = 1;
#elif defined(CONFIG_TANGOX_FIP_REF2)
				type = 2;
#else
				return(-EIO);
#endif
				if (copy_to_user(ptr, (char *)&type, sizeof(unsigned long))) 
					return(-EFAULT);
			}
			break;
		default:
			return(-EIO);
	}
	return(0);
}

#endif /* __KERNEL__ */

/* Micro-second sleep */
static void fip_usleep(unsigned usec)
{
#ifdef __KERNEL__
	udelay(usec);
#else
	em86xx_usleep(usec);
#endif /* __KERNEL__ */
}

static unsigned int fip_read_reg(unsigned int offset)
{
//	unsigned int val = *((volatile unsigned int *)(fip_base + offset));
	unsigned int val = gbus_read_reg32(fip_base + offset);
	return(val);
}

static void fip_write_reg(unsigned int offset, unsigned int val)
{
	fip_wait_ready();
//	*((volatile unsigned int *)(fip_base + offset)) = val;
	gbus_write_reg32(fip_base + offset, val);
	fip_wait_ready();
}

#ifdef __KERNEL__
#ifdef ENABLE_WRITE_INTR
/* To queue the write request */
static void fip_queue_command(unsigned int cmd, unsigned int data)
{
	unsigned long flags;

	spin_lock_irqsave(&fip_priv.lock, flags);
	if (((fip_priv.cmd_pidx + 1) % CMDQ_SIZE) == fip_priv.cmd_cidx) {
		printk(KERN_ERR "Command queue full.\n");
		fip_issue_command(&fip_priv);
	} else {
		fip_priv.cmds[fip_priv.cmd_pidx].cmd = cmd;
		fip_priv.cmds[fip_priv.cmd_pidx].data = data;
		fip_priv.cmd_pidx = ((fip_priv.cmd_pidx + 1) % CMDQ_SIZE);
	}
	spin_unlock_irqrestore(&fip_priv.lock, flags);
}
#endif
#endif

#ifdef __KERNEL__
static int is_fip_busy_nowait(void)
{
	return((fip_read_reg(FIP_CONFIG) & FIP_BUSY) != 0);
}
#endif

#ifdef __KERNEL__
static
#endif /* __KERNEL__ */
int is_fip_busy(void)
{
#if defined(CONFIG_TANGOX_FIP_REF1)
	fip_usleep(10);
#elif defined(CONFIG_TANGOX_FIP_REF2)
	fip_usleep(20);
#endif
	return((fip_read_reg(FIP_CONFIG) & FIP_BUSY) != 0);
}

#ifdef __KERNEL__
static
#endif /* __KERNEL__ */
void fip_wait_ready(void)
{
	while (is_fip_busy());
#if defined(CONFIG_TANGOX_FIP_REF2)
	fip_usleep(20);
#endif
}

static void fip_user_display(int adr, int data)
{
#ifdef __KERNEL__
#ifdef ENABLE_WRITE_INTR
	fip_wait_ready();
	fip_queue_command(FIP_CMD_ADR_SETTING | (adr), data);
#else
	fip_wait_ready();
	fip_write_reg(FIP_DISPLAY_DATA, data);
	fip_write_reg(FIP_COMMAND, FIP_CMD_ADR_SETTING | (adr));
#endif
#else
	fip_wait_ready();
	fip_write_reg(FIP_DISPLAY_DATA, data);
	fip_wait_ready();
	fip_write_reg(FIP_COMMAND, FIP_CMD_ADR_SETTING | (adr));
	fip_wait_ready();
#endif
}

#ifdef __KERNEL__
static
#endif /* __KERNEL__ */
int fip_display_character(const int position, const char character) 
{
	int i, byte1, byte2;
	unsigned char current_contents0, current_contents1;
#if defined(CONFIG_TANGOX_FIP_REF1)
	const int min_pos = 1;
#elif defined(CONFIG_TANGOX_FIP_REF2)
	const int min_pos = 0;
#endif
	
	if ((position < min_pos) || (position > NUM_DIGITS)) {
#ifdef __KERNEL__
		printk(KERN_DEBUG "%s: position %d not available/supported.\n",
				fip_devname, position);
#else
		uart_printf("%s: position %d not available/supported.\n",
				fip_devname, position);
#endif /* __KERNEL__ */
		return(0);
	}

	for (i = 0; i < NUM_CHARACTERS; i++) {
		if (character == fipcharactersmap[i]) {
			byte1 = 24 - (3 * position);
			byte2 = 25 - (3 * position);

			current_contents0 = fipram[byte1];
			current_contents1 = fipram[byte2];

			/* clear */	
			fipram[byte1] &= fipcharactermask[0];
			fipram[byte2] &= fipcharactermask[1];
			
			/* set new bits */
			fipram[byte1] |= fipcharacters[i][0];
			fipram[byte2] |= fipcharacters[i][1];

			/* display if necessary */
			if (current_contents0 != fipram[byte1])
				fip_user_display(byte1, fipram[byte1]);
			if (current_contents1 != fipram[byte2])
				fip_user_display(byte2, fipram[byte2]);
			return(1);
		}
	}
#ifdef __KERNEL__
	printk(KERN_DEBUG "%s: character '%c' not available/supported.\n", fip_devname, character);
#else
	uart_printf("%s: character '%c' not available/supported.\n", fip_devname, character);
#endif /* __KERNEL__ */
	return(0);
}

#ifdef __KERNEL__
static
#endif /* __KERNEL__ */
void fip_clear(void)
{
	register int i;

	for (i = 0; i < MAX_FIP_RAM; i++) {
		fipram[i] = 0;
		fip_user_display(i, fipram[i]);
	}
}

#ifdef __KERNEL__
static
#endif /* __KERNEL__ */
void fip_display_raw(const int byte, const int bit, const int on) 
{
	unsigned char current_contents;
	current_contents = fipram[byte];
	if (on != 0)
		fipram[byte] |= (1 << bit);
	else
		fipram[byte] &= ~(1 << bit);

	/* display only if necessary */
	if (current_contents != fipram[byte])
		fip_user_display(byte, fipram[byte]);
}

#ifdef __KERNEL__
static
#endif /* __KERNEL__ */
void fip_display_symbol(const int symbol, const int on) 
{
	if ((symbol < 0) || (symbol >= NUM_SYMBOLS)) {
#ifdef __KERNEL__
		printk(KERN_DEBUG "%s: symbol #%d not available/supported.\n", fip_devname, symbol);
#else
		uart_printf("%s: symbol #%d not available/supported.\n", fip_devname, symbol);
#endif /* __KERNEL__ */
		return;
	}

#if defined(CONFIG_TANGOX_FIP_REF1)
	fip_display_raw(fipsymbols[symbol][0], fipsymbols[symbol][1], on);
#elif defined(CONFIG_TANGOX_FIP_REF2)
	//0-99 displays tra/chap number field (0-99)
	//100-199 displays title number field (0-99)
	//200 clears all in this area
	//201-206 displays symbols without effecting other fields
	if (symbol > 200) {
		 //fip_display_raw(fipsymbols[symbol-200][0], fipsymbols[symbol-200][1], on);
		 switch(symbol) {
		 	case 201:
				fip_user_display(0, 1);
				break;
			case 202:
				fip_user_display(0, 2);
				break;
			case 203:
				fip_user_display(0, 4);
				break;
			case 204:
				fip_user_display(0, 8);
				break;
			case 205:
				fip_user_display(27, 1);
				break;
			case 206:
				fip_user_display(29, 1);
				break;
		 }
	} else if (symbol == 200) { 
		fipram[27] = fipxcharacters[11];
		fipram[28] = fipxcharacters[11];
		fipram[30] = fipxcharacters[11];
		fipram[31] = fipxcharacters[11];
		fip_user_display(27, fipram[27]);
		fip_user_display(28, fipram[28]);	
		fip_user_display(30, fipram[30]);
		fip_user_display(31, fipram[31]);
			
	} else if (symbol >= 100) { 
		fipram[27] = fipxcharacters[(symbol-100)/10];
		fipram[28] = fipxcharacters[(symbol-100)%10];
		fip_user_display(27, fipram[27]);
		fip_user_display(28, fipram[28]);
	} else if (symbol >= 0) {
		fipram[30] = fipxcharacters[symbol/10];
		fipram[31] = fipxcharacters[symbol%10];	
		fip_user_display(30, fipram[30]);
		fip_user_display(31, fipram[31]);	
	}
#endif
}

#ifdef __KERNEL__
static
#endif /* __KERNEL__ */
void fip_write_text(const int position, const char *text, const int flags) 
{
	int x, i, j;
	int textLen = strlen (text);

#if defined(CONFIG_TANGOX_FIP_REF1)
	if (flags & FIP_CENTER)
		x = (position > 0) ? position - textLen / 2 : (NUM_DIGITS - textLen) / 2 + 1;
	else if (flags & FIP_RIGHT)
		x = (position > 0) ? position - textLen : NUM_DIGITS - textLen + 1;
	else 
		x = (position > 0) ? position : 1;
	if (x < 1)
		x = 1;

	if ((flags & FIP_NO_CLEAR) == 0) {
		/* clear colons */
		fip_display_symbol(COLON_HOUR_MIN_FIP_ON, 0);
		fip_display_symbol(COLON_MIN_SEC_FIP_ON, 0);
	}
#elif defined(CONFIG_TANGOX_FIP_REF2)
	if (flags & FIP_CENTER)
		x = (position >= 0) ? position - textLen / 2 : (NUM_DIGITS - textLen) / 2 + 1;
	else if (flags & FIP_RIGHT)
		x = (position >= 0) ? position - textLen : NUM_DIGITS - textLen + 1;
	else 
		x = (position >= 0) ? position : 1;
	if (x < 1)
		x = 1;
#endif

	/* show/write text */
	j = 0;
	for (i = 1; i <= NUM_DIGITS; i++) {
		if ((i < x) || (i >= (x+textLen)))
			fip_display_character(i, ' ');
		else if (!fip_display_character(i, text[j++])) {
#ifdef __KERNEL__
			printk(KERN_DEBUG "%s: cannot show text '%s'.\n", fip_devname, text);
#else
			uart_printf("%s: cannot show text '%s'.\n", fip_devname, text);
#endif /* __KERNEL__ */
			break;
		}
	}
}

#ifdef __KERNEL__ 
static 
#endif /* __KERNEL__ */
int fip_show_hms(int hour, int minute, int second)
{
	if (hour < L_OFF || minute < L_OFF || second < L_OFF ||
		hour > 99 || minute > 59 || second > 59) {
#ifdef __KERNEL__
		printk(KERN_DEBUG "%s: parameters passed not in valid range\n", fip_devname);
#else
		uart_printf("%s: parameters passed not in valid range\n", fip_devname);
#endif
		return(1);
	}
 
#if defined(CONFIG_TANGOX_FIP_REF1)
	// hour
	fip_display_character(1, ' ');
	fip_display_character(2, (hour==L_OFF) ? ' ' : hour/10 + '0');
	fip_display_character(3, (hour==L_OFF) ? ' ' : hour%10 + '0');

	// minute 
	fip_display_character(4, (minute==L_OFF) ? ' ' : minute/10 + '0');
	fip_display_character(5, (minute==L_OFF) ? ' ' : minute%10 + '0');

	// second
	fip_display_character(6, (second==L_OFF) ? ' ' : second/10 + '0');
	fip_display_character(7, (second==L_OFF) ? ' ' : second%10 + '0');

	// show colons if needed
	fip_display_symbol(COLON_HOUR_MIN_FIP_ON, (hour == L_OFF) ? 0 : 1);
	fip_display_symbol(COLON_MIN_SEC_FIP_ON, ((hour == L_OFF) && (minute == L_OFF)) ? 0 : 1);
#elif defined(CONFIG_TANGOX_FIP_REF2)
	// hour
	fip_display_character(0, (hour==L_OFF) ? ' ' : hour/10 + '0');
	fip_display_character(1, (hour==L_OFF) ? ' ' : hour%10 + '0');

	// minute 
	fip_display_character(2, (minute==L_OFF) ? ' ' : minute/10 + '0');
	fip_display_character(3, (minute==L_OFF) ? ' ' : minute%10 + '0');

	// second
	fip_display_character(4, (second==L_OFF) ? ' ' : second/10 + '0');
	fip_display_character(5, (second==L_OFF) ? ' ' : second%10 + '0');
	fip_display_character(6, ' ');
	fip_display_character(7, ' ');
#endif
	return(0);
}

#ifdef __KERNEL__ 
static 
#endif /* __KERNEL__ */
int fip_init(void)
{
	static int initflag = 0;

	if (initflag != 0)
		return(0);

#ifdef __KERNEL__
	/* Disable FIP and interrupt first */
	fip_wait_ready();
	fip_write_reg(FIP_CONFIG, 0);
	fip_wait_ready();

#ifdef ENABLE_WRITE_INTR
	/* Enable FIP and interrupts (read: 0x20000, write: 0x10000) */
	fip_write_reg(FIP_CONFIG, (FIP_DIVIDER | FIP_ENABLE | 0x30000));
#else
	/* Enable FIP and interrupts (read: 0x20000) */
	fip_write_reg(FIP_CONFIG, (FIP_DIVIDER | FIP_ENABLE | 0x20000));
#endif
#else
	fip_wait_ready();
	/* Enable FIP wthout enabling interrupt */
	fip_write_reg(FIP_CONFIG, (FIP_DIVIDER | FIP_ENABLE));
#endif /* __KERNEL__ */
	fip_wait_ready();

	/* Clear exisiting IRQ, if any */
	fip_write_reg(FIP_INT, 0x3);
	fip_wait_ready();

	/* select display mode */
	fip_write_reg(FIP_COMMAND, FIP_DISPLAY_MODE);
	fip_wait_ready();

	/* select brightness of display and turn it on */
	fip_write_reg(FIP_COMMAND, FIP_CMD_DISP_CTRL_TURN_DISPLAY_ON | brightness);
	fip_wait_ready();

	/* select write to display and fixed addressing */
	fip_write_reg(FIP_COMMAND, FIP_CMD_DATA_SET_ADR_MODE_FIXED_ADR);
	fip_wait_ready();

	fip_clear();
	fip_wait_ready();
	initflag = 1;
	return(0);
}

#ifdef __KERNEL__ 
static 
#endif /* __KERNEL__ */
int fip_exit(void)
{
	fip_clear();

	/* Disable FIP and interrupt */
	fip_write_reg(FIP_CONFIG, 0);
	fip_wait_ready();
	return(0);
}

#ifndef __KERNEL__ 
unsigned long fip_readkey(void)
{
	unsigned long key = 0L;

	fip_wait_ready();
	fip_write_reg(FIP_COMMAND, FIP_CMD_DATA_SET_RW_MODE_READ_KEYS);
	key = fip_read_reg(FIP_KEY_DATA1); 
	return(key);
}
#endif /* __KERNEL__ */

#ifdef __KERNEL__
int __init fip_init_module(void)
{
	int status = 0;

	if (tangox_fip_enabled() == 0)
		return(0);

	/* Initialize private data structure */
	memset(&fip_priv, 0, sizeof(struct fip_private)); 
#ifdef ENABLE_WRITE_INTR
	fip_priv.cmdq_empty = 1;
#endif
	spin_lock_init(&fip_priv.lock);

	if (buffer_size < 1) {
		printk(KERN_ERR "%s: buffer size (%d) error\n", fip_devname,
			buffer_size); 
		return(-EIO);
	} 
	if ((fip_priv.buffer = kmalloc(buffer_size * sizeof(unsigned long), GFP_KERNEL)) == NULL) {
		printk(KERN_ERR "%s: out of memory for buffer\n", fip_devname); 
		return(-ENOMEM);
	}

	/* Register device, and may be allocating major# */
	status = register_chrdev(major_num, fip_devname, &fip_fops);
	if (status < 0) {
		printk(KERN_ERR "%s: cannot get major number\n", fip_devname); 
		if (fip_priv.buffer != NULL)
			kfree(fip_priv.buffer);
		return(status);
	} else if (major_num == 0)
		major_num = status;	/* Dynamic major# allocation */

	/* Hook up ISR */
	if (request_irq(fip_irq, fip_isr, IRQF_DISABLED, fip_devname, &fip_priv) != 0) {
		printk(KERN_ERR "%s: cannot register IRQ (%d)\n", fip_devname,
			fip_irq);
		unregister_chrdev(major_num, fip_devname);	
		if (fip_priv.buffer != NULL)
			kfree(fip_priv.buffer);
		return(-EIO);
	}

	fip_init();
	init_timer(&fip_timer);

	printk("SMP86xx %s (%d:0): driver loaded (buffer_size = %d)\n", 
		fip_devname, major_num, buffer_size);
	return(0);
}

void __exit fip_cleanup_module(void)
{
	if (tangox_fip_enabled() == 0)
		return;

	unregister_chrdev(major_num, fip_devname);
	free_irq(fip_irq, &fip_priv);

	if (fip_priv.buffer != NULL)
		kfree(fip_priv.buffer);

	fip_exit();

	printk(KERN_DEBUG "%s: driver unloaded\n", fip_devname);
}

module_init(fip_init_module);
module_exit(fip_cleanup_module);

#endif /* __KERNEL__ */

