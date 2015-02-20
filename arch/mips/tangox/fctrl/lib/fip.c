/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/*
 *
 */

#include "platform.h"

/* EM86XX */
#define	FIP_BASE				(REG_BASE_system_block+0x500)
#define	FIP_COMMAND				0x40
#define	FIP_DISPLAY_DATA			0x44
#define	FIP_LED_DATA				0x48
#define	FIP_KEY_DATA1				0x4c
#define	FIP_KEY_DATA2				0x50
#define	FIP_SWITCH_DATA				0x54
#define FIP_CONFIG				0x58
#define FIP_INT					0x5c

#define FIP_DIVIDER				27	/* default value */
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

#define FIP_DISPLAY_MODE	0xa
#define MAX_FIP_RAM		23
#define NUM_SYMBOLS		23
#define NUM_CHARACTERS		72
#define NUM_DIGITS		7

#define L_OFF			-1	//means light is or should be off
#define FIP_NO_CLEAR		0x0004

/* Valid symbols */
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

/* Alignment */
#define FIP_LEFT		0x0000	/* flags for fip_write_text() */
#define FIP_CENTER		0x0001
#define FIP_RIGHT		0x0002

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

/* we use the inverted mask for clearing a digit without clearing other things */
static const char fipcharactermask[2] = {
	DIGIT_L(1,1,0,0,0,0,0,0), 
	DIGIT_H(0,0,0,0,0,0,0,0)
};

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

/* Default brightness level */
#define BRIGHTNESS		0x7

static char fipram[MAX_FIP_RAM] = {0};
static int brightness = BRIGHTNESS;

static void fip_write_reg(unsigned int offset, unsigned int val);
static unsigned int fip_read_reg(unsigned int offset);

static unsigned long fip_config = 0;

#define ABS(x)  (((x) >= 0) ? (x) : -(x))
#define DIVIDE_FACTOR       2

static unsigned int fip_read_reg(unsigned int offset)
{
	unsigned int val = tangox_gbus_read_reg32(FIP_BASE + offset);
	return val;
}

static void fip_write_reg(unsigned int offset, unsigned int val)
{
	tangox_gbus_write_reg32(FIP_BASE + offset, val);
	tangox_sync();
}

static int is_fip_busy(void)
{
	tangox_usdelay(10);
	return (fip_read_reg(FIP_CONFIG) & FIP_BUSY) != 0;
}

static void fip_wait_ready(void)
{
	while (is_fip_busy());
}

static void fip_user_display(int adr, int data)
{
	fip_wait_ready();
	fip_write_reg(FIP_DISPLAY_DATA, data);
	fip_write_reg(FIP_COMMAND, FIP_CMD_ADR_SETTING | (adr));
}

static int fip_display_character(const int position, const char character) 
{
	int i, byte1, byte2;
	unsigned char current_contents0, current_contents1;
	
	if ((position < 1) || (position > NUM_DIGITS)) 
		return 0;

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
			return 1;
		}
	}
	return 0;
}

static void fip_clear(void)
{
	register int i;

	for (i = 0; i < MAX_FIP_RAM; i++) {
		fipram[i] = 0;
		fip_user_display(i, fipram[i]);
	}
}

static void fip_display_raw(const int byte, const int bit, const int on) 
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

static void fip_display_symbol(const int symbol, const int on) 
{
	if ((symbol < 0) || (symbol >= NUM_SYMBOLS)) 
		return;
	fip_display_raw(fipsymbols[symbol][0], fipsymbols[symbol][1], on);
}

static void fip_write_text(const int position, const char *text, const int flags) 
{
	int x, i, j;
	int textLen = strlen(text);

	if (flags & FIP_CENTER)
		x = (position > 0) ? position - textLen / 2 : (NUM_DIGITS - textLen) / 2 + 1;
	else if (flags & FIP_RIGHT)
		x = (position > 0) ? position - textLen : NUM_DIGITS - textLen + 1;
	else 
		x = (position > 0) ? position : 1;
	if (x < 1) x = 1;

	if ((flags & FIP_NO_CLEAR) == 0) {
		/* clear colons */
		fip_display_symbol(COLON_HOUR_MIN_FIP_ON, 0);
		fip_display_symbol(COLON_MIN_SEC_FIP_ON, 0);
	}

	/* show/write text */
	j = 0;
	for (i = 1; i <= NUM_DIGITS; i++) {
		if ((i < x) || (i >= (x+textLen)))
			fip_display_character(i, ' ');
		else if (!fip_display_character(i, text[j++])) {
			break;
		}
	}
}

static unsigned long fip_poll_key(unsigned long *table, unsigned long size)
{
	unsigned long key = 0L;

	fip_wait_ready();
	fip_write_reg(FIP_COMMAND, FIP_CMD_DATA_SET_RW_MODE_READ_KEYS);
	fip_wait_ready();
	key = fip_read_reg(FIP_KEY_DATA1); 

	if ((key == 0) || (table == 0) || (*table == 0)) {
#ifdef FC_DEBUG
		if (table == 0)
			DBG_PRINT("FIP NULL table\n");
		if (*table == 0)
			DBG_PRINT("FIP NULL content\n");
		if (key != 0)
			DBG_PRINT("Got any FIP key\n");
#endif
		return key;
	} else {
		unsigned int i;
		for (i = 0; i < size; i++, table++) {
			if (*table == 0)
				break;
			else if (key == *table) {

				DBG_PRINT("Got defined FIP key\n");
				return key;
			}
		}
		return 0;
	}
}

int fip_open(void)
{
	unsigned long key = 0L;

	DBG_PRINT("Polling for FIP key.\n");

	fip_config = fip_read_reg(FIP_CONFIG);

	if ((fip_config & FIP_ENABLE) == 0) {
		fip_write_reg(FIP_CONFIG, (FIP_DIVIDER | FIP_ENABLE));
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
		fip_write_text(0, "STANDBY", FIP_CENTER);

		/* Flush buffer */
		while ((key = fip_poll_key(0, 0)) != 0);
	}

	return 0;
}

int fip_close(void)
{
	DBG_PRINT("Stop polling for FIP key.\n");

	fip_write_reg(FIP_CONFIG, fip_config);
	tangox_usdelay(1000);

	return 0;
}

