/*
 * LP5521 LED chip driver.
 *
 * Copyright (C) 2010 Nokia Corporation
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/leds.h>
#include <linux/leds-lp5521.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

/*                                                                       */
#include <linux/kernel.h>
#include <linux/delay.h>
#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/platform_device.h>
#include <cust_acc.h>
/*                                                                        */

#ifdef CONFIG_MACH_MSM8X25_V7
#define LP5521_MAX_LED_CURRENT 217
#define LP5521_R_INDEX 7
#define LP5521_G_INDEX 4
#define LP5521_B_INDEX 8
#else
#define LP5521_MAX_LED_CURRENT 255
#define LP5521_R_INDEX 1
#define LP5521_G_INDEX 1
#define LP5521_B_INDEX 1
#endif

#define LP5521_PROGRAM_LENGTH	32	/* in bytes */

#define LP5521_MAX_LEDS			3	/* Maximum number of LEDs */
#define LP5521_MAX_ENGINES		3	/* Maximum number of engines */

#define LP5521_ENG_MASK_BASE	0x30	/* 00110000 */
#define LP5521_ENG_STATUS_MASK	0x07	/* 00000111 */

#define LP5521_CMD_LOAD			0x15	/* 00010101 */
#define LP5521_CMD_RUN			0x2a	/* 00101010 */
#define LP5521_CMD_DIRECT		0x3f	/* 00111111 */
#define LP5521_CMD_DISABLED		0x00	/* 00000000 */

/* Registers */
#define LP5521_REG_ENABLE		0x00
#define LP5521_REG_OP_MODE		0x01
#define LP5521_REG_R_PWM		0x02
#define LP5521_REG_G_PWM		0x03
#define LP5521_REG_B_PWM		0x04
#define LP5521_REG_R_CURRENT	0x05
#define LP5521_REG_G_CURRENT	0x06
#define LP5521_REG_B_CURRENT	0x07
#define LP5521_REG_CONFIG		0x08
#define LP5521_REG_R_CHANNEL_PC	0x09
#define LP5521_REG_G_CHANNEL_PC	0x0A
#define LP5521_REG_B_CHANNEL_PC	0x0B
#define LP5521_REG_STATUS		0x0C
#define LP5521_REG_RESET		0x0D
#define LP5521_REG_GPO			0x0E
#define LP5521_REG_R_PROG_MEM	0x10
#define LP5521_REG_G_PROG_MEM	0x30
#define LP5521_REG_B_PROG_MEM	0x50

#define LP5521_PROG_MEM_BASE	LP5521_REG_R_PROG_MEM
#define LP5521_PROG_MEM_SIZE	0x20

/* Base register to set LED current */
#define LP5521_REG_LED_CURRENT_BASE	LP5521_REG_R_CURRENT

/* Base register to set the brightness */
#define LP5521_REG_LED_PWM_BASE		LP5521_REG_R_PWM

/* Bits in ENABLE register */
#define LP5521_MASTER_ENABLE		0x40	/* Chip master enable */
#define LP5521_LOGARITHMIC_PWM		0x80	/* Logarithmic PWM adjustment */
#define LP5521_EXEC_RUN			0x2A
#define LP5521_ENABLE_DEFAULT	\
	(LP5521_MASTER_ENABLE | LP5521_LOGARITHMIC_PWM)
#define LP5521_ENABLE_RUN_PROGRAM	\
	(LP5521_ENABLE_DEFAULT | LP5521_EXEC_RUN)

/* Status */
#define LP5521_EXT_CLK_USED		0x08

/* default R channel current register value */
#define LP5521_REG_R_CURR_DEFAULT	0xAF

/* Current index max step */
#define PATTERN_CURRENT_INDEX_STEP_HAL 255

/* Pattern Mode */
#define PATTERN_OFF	0

/*K-Prj Favorite MissedNoit Pattern Mode*/
#define PATTERN_FAVORITE_MISSED_NOTI 14

#if 1 /* jjm_rgb */
#define PATTERN_BLINK_ON 0xFF
#endif

/* Program Commands */
#define CMD_SET_PWM			0x40
#define CMD_WAIT_LSB		0x00

#define MAX_BLINK_TIME		60000	/* 60 sec */

enum lp5521_rgb {
	LP5521_R = 0,
	LP5521_G,
	LP5521_B,
};

/*                                                                        */
// I2C variable
static struct lp5521_led_config lp5521_led_config[] = {
	{
		.name = "R",
		.chan_nr	= 0,
		.led_current	= 200,
		.max_current	= 255,
	},
	{
		.name = "G",
		.chan_nr	= 1,
		.led_current	= 200,
		.max_current	= 255,
	},
	{
		.name = "B",
		.chan_nr	= 2,
		.led_current	= 200,
		.max_current	= 255,
	},
};

//[pattern_id : 1, PowerOn_Animation]
static u8 mode1_red[] = {0xE0, 0x0C, 0x40, 0x00, 0x0C, 0x2F, 0x06, 0x28, 0x05, 0x2D, 0x06, 0x2A, 0x06, 0x25, 0x03, 0xDC, 0x02, 0xFA, 0x48, 0x00, 0x03, 0x54, 0x44, 0x01, 0x23, 0x06, 0x31, 0x84, 0x06, 0xA8, 0x0C, 0xAF};
static u8 mode1_green[] = {0xE0, 0x80, 0x40, 0x00, 0x52, 0x00, 0x0B, 0x15, 0x05, 0x2D, 0x03, 0x48, 0x03, 0x4B, 0x09, 0x1B, 0x02, 0x63, 0x19, 0x89, 0x03, 0xCA, 0x04, 0xC1, 0x05, 0xB2, 0x06, 0xA6, 0x12, 0x8D, 0x52, 0x00};
static u8 mode1_blue[] = {0xE0, 0x80, 0x40, 0x00, 0x12, 0xFE, 0x40, 0xC0, 0x0A, 0x18, 0x06, 0xA6, 0x06, 0xAA, 0x03, 0xCF, 0x04, 0xB6, 0x52, 0x00};

//[pattern_id : 2, LCDOn]
static u8 mode2_red[]={0x40, 0xff, 0x4d, 0x00, 0x0a, 0xff, 0x0a, 0xfe, 0xc0, 0x00};
static u8 mode2_green[]={0x40, 0xff, 0x4d, 0x00, 0x0a, 0xff, 0x0a, 0xfe, 0xc0, 0x00};
static u8 mode2_blue[]={0x40, 0xff, 0x4d, 0x00, 0x0a, 0xff, 0x0a, 0xfe, 0xc0, 0x00};

//[pattern_id : 3, Charging0_15]
static u8 mode3_red[] = {0x40, 0x0D, 0x44, 0x0C, 0x24, 0x32, 0x24, 0x32, 0x66, 0x00, 0x24, 0xB2, 0x24, 0xB2, 0x44, 0x8C};

//[pattern_id : 4, Charging100]
static u8 mode4_green[]={0x40, 0xff};

//[pattern_id : 5, Charging16_99] //not used any more
static u8 mode5_red[]={0x40, 0x19, 0x27, 0x19, 0xe0, 0x04, 0x0c, 0x65, 0xe0, 0x04, 0x0c, 0x65, 0xe0, 0x04, 0x0c, 0xe5, 0xe0, 0x04, 0x0c, 0xe5, 0xe0, 0x04, 0x29, 0x98, 0xe0, 0x04, 0x5a, 0x00};
static u8 mode5_green[]={0x40, 0x0c, 0x43, 0x0b, 0xe0, 0x80, 0x19, 0x30, 0xe0, 0x80, 0x19, 0x30, 0xe0, 0x80, 0x19, 0xb0, 0xe0, 0x80, 0x19, 0xb0, 0xe0, 0x80, 0x43, 0x8b, 0xe0, 0x80, 0x5a, 0x00};

//[pattern_id : 6, PowerOff]
static u8 mode6_red[] = {0xE0, 0x0C, 0x40, 0x00, 0x0C, 0x2F, 0x06, 0x28, 0x05, 0x2D, 0x06, 0x2A, 0x06, 0x25, 0x03, 0xDC, 0x02, 0xFA, 0x48, 0x00, 0x03, 0x54, 0x44, 0x01, 0x23, 0x06, 0x31, 0x84, 0x06, 0xA8, 0x0C, 0xAF};
static u8 mode6_green[] = {0xE0, 0x80, 0x40, 0x00, 0x52, 0x00, 0x0B, 0x15, 0x05, 0x2D, 0x03, 0x48, 0x03, 0x4B, 0x09, 0x1B, 0x02, 0x63, 0x19, 0x89, 0x03, 0xCA, 0x04, 0xC1, 0x05, 0xB2, 0x06, 0xA6, 0x12, 0x8D, 0x52, 0x00};
static u8 mode6_blue[] = {0xE0, 0x80, 0x40, 0x00, 0x12, 0xFE, 0x40, 0xC0, 0x0A, 0x18, 0x06, 0xA6, 0x06, 0xAA, 0x03, 0xCF, 0x04, 0xB6, 0x52, 0x00}; 

//[pattern_id : 7, MissedNoti]
static u8 mode7_green[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xff, 0x02, 0xd4, 0x02, 0xd4, 0x02, 0xd4, 0x48, 0x00, 0x40, 0xff, 0x02, 0xd4, 0x02, 0xd4, 0x02, 0xd4, 0x25, 0xfe};

//[pattern_id : 14, MissedNoti(favorite)]
static u8 mode8_red[]={0x40, 0x00, 0x10, 0xFE, 0x40, 0xE6, 0xE2, 0x00, 0x03, 0xF2, 0xE2, 0x00, 0x03, 0xF2, 0xE2, 0x00, 0x48, 0x00, 0x40, 0xE6, 0xE2, 0x00, 0x03, 0xF2, 0xE2, 0x00, 0x03, 0xF2, 0xE2, 0x00, 0x25, 0xFE,};
static u8 mode8_green[]={0x40, 0x00, 0x10, 0xFE, 0x40, 0x5D, 0xE2, 0x00, 0x07, 0xAD, 0xE2, 0x00, 0x07, 0xAE, 0xE2, 0x00, 0x48, 0x00, 0x40, 0x5D, 0xE2, 0x00, 0x07, 0xAD, 0xE2, 0x00, 0x07, 0xAE, 0xE2, 0x00, 0x25, 0xFE,};
static u8 mode8_blue[]={0x40, 0x00, 0x10, 0xFE, 0x40, 0x5D, 0xE0, 0x06, 0x07, 0xAD, 0xE0, 0x06, 0x07, 0xAE, 0xE0, 0x06, 0x48, 0x00, 0x40, 0x5D, 0xE0, 0x06, 0x07, 0xAD, 0xE0, 0x06, 0x07, 0xAE, 0xE0, 0x06, 0x25, 0xFE,};

#if 1 /* jjm_rgb */
/*[pattern_id : 17, MissedNoti(pink)]*/
static u8 mode9_red[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xff, 0x02, 0xff, 0xe2, 0x00, 0x02, 0xfe, 0xe2, 0x00, 0x48, 0x00, 0x40, 0xff, 0x02, 0xff, 0xe2, 0x00, 0x02, 0xfe, 0xe2, 0x00, 0x25, 0xfe};
static u8 mode9_green[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0x66, 0x06, 0xb2, 0xe2, 0x00, 0x06, 0xb2, 0xe2, 0x00, 0x48, 0x00, 0x40, 0x66, 0x06, 0xb2, 0xe2, 0x00, 0x06, 0xb2, 0xe2, 0x00, 0x25, 0xfe};
static u8 mode9_blue[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0x73, 0x04, 0xd5, 0xe0, 0x06, 0x0b, 0x9c, 0xe0, 0x06, 0x48, 0x00, 0x40, 0x73, 0x04, 0xd5, 0xe0, 0x06, 0x0b, 0x9c, 0xe0, 0x06, 0x25, 0xfe};

/*[pattern_id : 18, MissedNoti(blue)]*/
static u8 mode10_green[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0x7a, 0x05, 0xbc, 0xe0, 0x08, 0x05, 0xbc, 0xe0, 0x08, 0x48, 0x00, 0x40, 0x7a, 0x05, 0xbc, 0xe0, 0x08, 0x05, 0xbc, 0xe0, 0x08, 0x25, 0xfe};
static u8 mode10_blue[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xff, 0x02, 0xff, 0xe1, 0x00, 0x02, 0xfe, 0xe1, 0x00, 0x48, 0x00, 0x40, 0xff, 0x02, 0xff, 0xe1, 0x00, 0x02, 0xfe, 0xe1, 0x00, 0x25, 0xfe};

/*[pattern_id : 19, MissedNoti(orange)]*/
static u8 mode11_red[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xff, 0x02, 0xff, 0xe1, 0x00, 0x02, 0xfe, 0xe1, 0x00, 0x48, 0x00, 0x40, 0xff, 0x02, 0xff, 0xe1, 0x00, 0x02, 0xfe, 0xe1, 0x00, 0x25, 0xfe};
static u8 mode11_green[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0x66, 0x06, 0xb2, 0xe0, 0x02, 0x06, 0xb2, 0xe0, 0x02, 0x48, 0x00, 0x40, 0x66, 0x06, 0xb2, 0xe0, 0x02, 0x06, 0xb2, 0xe0, 0x02, 0x25, 0xfe};

/*[pattern_id : 20, MissedNoti(yellow)]*/
static u8 mode12_red[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xff, 0x02, 0xff, 0xe1, 0x00, 0x02, 0xfe, 0xe1, 0x00, 0x48, 0x00, 0x40, 0xff, 0x02, 0xff, 0xe1, 0x00, 0x02, 0xfe, 0xe1, 0x00, 0x25, 0xfe};
static u8 mode12_green[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xe6, 0x02, 0xff, 0xe0, 0x02, 0x03, 0xe5, 0xe0, 0x02, 0x48, 0x00, 0x40, 0xe6, 0x02, 0xff, 0xe0, 0x02, 0x03, 0xe5, 0xe0, 0x02, 0x25, 0xfe};

/*[pattern_id : 29, MissedNoti(turquoise)]*/
static u8 mode13_red[]={};
static u8 mode13_green[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xff, 0x02, 0xfe, 0xe2, 0x00, 0x02, 0xfe, 0xe2, 0x00, 0x48, 0x00, 0x40, 0xff, 0x02, 0xff, 0xe2, 0x00, 0x02, 0xfe, 0xe2, 0x00, 0x25, 0xfe};
static u8 mode13_blue[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0x7a, 0x06, 0xb2, 0xe0, 0x06, 0x06, 0xb2, 0xe0, 0x06, 0x48, 0x00, 0x40, 0x7a, 0x06, 0xb2, 0xe0, 0x06, 0x06, 0xb2, 0xe0, 0x06, 0x25, 0xfe};


/*[pattern_id : 30, MissedNoti(purple)]*/
static u8 mode14_red[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xb3, 0x06, 0xb2, 0xe1, 0x00, 0x02, 0xff, 0xe1, 0x00, 0x48, 0x00, 0x40, 0xb3, 0x06, 0xb2, 0xe1, 0x00, 0x02, 0xff, 0xe1, 0x00, 0x25, 0xfe};
static u8 mode14_green[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0x19, 0x06, 0xb2, 0xe0, 0x0a, 0x02, 0xff, 0xe0, 0x0a, 0x48, 0x00, 0x40, 0x19, 0x06, 0xb2, 0xe0, 0x0a, 0x02, 0xff, 0xe0, 0x0a, 0x25, 0xfe};
static u8 mode14_blue[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xff, 0x02, 0xff, 0xe1, 0x00, 0x02, 0xff, 0xe1, 0x00, 0x48, 0x00, 0x40, 0xff, 0x02, 0xff, 0xe1, 0x00, 0x02, 0xff, 0xe1, 0x00, 0x25, 0xfe};


/*[pattern_id : 31, MissedNoti(red)]*/
static u8 mode15_red[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xff, 0x02, 0xd4, 0x02, 0xd4, 0x02, 0xd4, 0x48, 0x00, 0x40, 0xff, 0x02, 0xd4, 0x02, 0xd4, 0x02, 0xd4, 0x25, 0xfe};
static u8 mode15_green[]={};
static u8 mode15_blue[]={};

/*[pattern_id : 32, MissedNoti(lime)]*/
static u8 mode16_red[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0x99, 0x06, 0xb2, 0xe0, 0x04, 0x02, 0xff, 0xe0, 0x04, 0x48, 0x00, 0x40, 0x99, 0x06, 0xb2, 0xe0, 0x04, 0x02, 0xff, 0xe0, 0x04, 0x25, 0xfe};
static u8 mode16_green[]={0x40, 0x00, 0x10, 0xfe, 0x40, 0xff, 0x02, 0xff, 0xe0, 0x80, 0x02, 0xff, 0xe0, 0x80, 0x48, 0x00, 0x40, 0xff, 0x02, 0xff, 0xe0, 0x80, 0x02, 0xff, 0xe0, 0x80, 0x25, 0xfe};
static u8 mode16_blue[]={};

/* [pattern_id : 37 ID_URGENT_CALL_MISSED_NOTI */
static u8 mode18_red[]={0x10, 0xFE, 0x40, 0xFF, 0x46, 0x00, 0x01, 0xFE, 0x01, 0xFF, 0x44, 0x00, 0xA1, 0x01, 0xE0, 0x04, 0x6E, 0x00, 0xE0, 0x04};
static u8 mode18_green[]={0x10, 0xFE, 0x51, 0x00, 0x40, 0x7F, 0x46, 0x00, 0x01, 0xFE, 0x01, 0xFF, 0x54, 0x00, 0xE0, 0x80, 0x6E, 0x00, 0xE0, 0x80};
static u8 mode18_blue[]={};

#endif

static struct lp5521_led_pattern board_led_patterns[] = {
	{ /* ID_POWER_ON = 1 */
		.r = mode1_red,
		.g = mode1_green,
		.b = mode1_blue,
		.size_r = ARRAY_SIZE(mode1_red),
		.size_g = ARRAY_SIZE(mode1_green),
		.size_b = ARRAY_SIZE(mode1_blue),
	},
	{ /* ID_LCD_ON = 2 */
		.r = mode2_red,
		.g = mode2_green,
		.b = mode2_blue,
		.size_r = ARRAY_SIZE(mode2_red),
		.size_g = ARRAY_SIZE(mode2_green),
		.size_b = ARRAY_SIZE(mode2_blue),
	},
	{ /* ID_CHARGING = 3 */
		.r = mode3_red,
//		.g = mode3_green,
//		.b = mode3_blue,
		.size_r = ARRAY_SIZE(mode3_red),
//		.size_g = ARRAY_SIZE(mode3_green),
//		.size_b = ARRAY_SIZE(mode3_blue),
	},
	{ /* ID_CHARGING_FULL = 4 */
//		.r = mode4_red,
		.g = mode4_green,
//		.b = mode4_blue,
//		.size_r = ARRAY_SIZE(mode4_red),
		.size_g = ARRAY_SIZE(mode4_green),
//		.size_b = ARRAY_SIZE(mode4_blue),
	},
	{ /* ID_CALENDAR_REMIND = 5 */
		.r = mode5_red,
		.g = mode5_green,
//		.b = mode5_blue,
		.size_r = ARRAY_SIZE(mode5_red),
		.size_g = ARRAY_SIZE(mode5_green),
//		.size_b = ARRAY_SIZE(mode5_blue),
	},
	{ /* ID_POWER_OFF = 6 */
		.r = mode6_red,
		.g = mode6_green,
		.b = mode6_blue,
		.size_r = ARRAY_SIZE(mode6_red),
		.size_g = ARRAY_SIZE(mode6_green),
		.size_b = ARRAY_SIZE(mode6_blue),
	},
	{ /* ID_MISSED_NOTI = 7 */
		//.r = mode7_red,
		.g = mode7_green,
		//.b = mode7_blue,
		//.size_r = ARRAY_SIZE(mode7_red),
		.size_g = ARRAY_SIZE(mode7_green),
		//.size_b = ARRAY_SIZE(mode7_blue),
	},
#if 1 /* jjm_rgb */
	/* Dummy Pattern IDs */
	{
		/* ID_ALARM = 8 */
	},
	{
		/* ID_CALL_01 = 9 */
	},
	{
		/* ID_CALL_02 = 10 */
	},
	{
		/* ID_CALL_03 = 11 */
	},
	{
		/* ID_VOLUME_UP = 12 */
	},
	{
		/* ID_VOLUME_DOWN = 13 */
	},
#endif
	{ /* ID_FAVORITE_MISSED_NOTI = 14 */
		.r = mode8_red,
		.g = mode8_green,
		.b = mode8_blue,
		.size_r = ARRAY_SIZE(mode8_red),
	    .size_g = ARRAY_SIZE(mode8_green),
		.size_b = ARRAY_SIZE(mode8_blue),
	},
#if 1 /* jjm_rgb */
	{
		/* ID_POWER_OFF_CHARGE_LOW = 15 */
	},
	{
		/* ID_POWER_OFF_CHARGE_MID = 16 */
	},
	{
		/* ID_MISSED_NOTI_PINK = 17 */
		.r = mode9_red,
		.g = mode9_green,
		.b = mode9_blue,
		.size_r = ARRAY_SIZE(mode9_red),
		.size_g = ARRAY_SIZE(mode9_green),
		.size_b = ARRAY_SIZE(mode9_blue),
	},
	{
		/* ID_MISSED_NOTI_BLUE = 18 */
		//.r = mode10_red,
		.g = mode10_green,
		.b = mode10_blue,
		//.size_r = ARRAY_SIZE(mode10_red),
		.size_g = ARRAY_SIZE(mode10_green),
		.size_b = ARRAY_SIZE(mode10_blue),
	},
	{
		/* ID_MISSED_NOTI_ORANGE = 19 */
		.r = mode11_red,
		.g = mode11_green,
		//.b = mode11_blue,
		.size_r = ARRAY_SIZE(mode11_red),
		.size_g = ARRAY_SIZE(mode11_green),
		//.size_b = ARRAY_SIZE(mode11_blue),
	},
	{
		/* ID_MISSED_NOTI_YELLOW = 20 */
		.r = mode12_red,
		.g = mode12_green,
		//.b = mode12_blue,
		.size_r = ARRAY_SIZE(mode12_red),
		.size_g = ARRAY_SIZE(mode12_green),
		//.size_b = ARRAY_SIZE(mode12_blue),
	},
	/* for dummy pattern IDs (defined LGLedRecord.java) */
	{
		/* ID_INCALL_PINK = 21 */
	},
	{
		/* ID_INCALL_BLUE = 22 */
	},
	{
		/* ID_INCALL_ORANGE = 23 */
	},
	{
		/* ID_INCALL_YELLOW = 24 */
	},
	{
		/* ID_INCALL_TURQUOISE = 25 */
	},
	{
		/* ID_INCALL_PURPLE = 26 */
	},
	{
		/* ID_INCALL_RED = 27 */
	},
	{
		/* ID_INCALL_LIME = 28 */
	},
	{
		/* ID_MISSED_NOTI_TURQUOISE = 29 */
		.r = mode13_red,
		.g = mode13_green,
		.b = mode13_blue,
		.size_r = ARRAY_SIZE(mode13_red),
		.size_g = ARRAY_SIZE(mode13_green),
		.size_b = ARRAY_SIZE(mode13_blue),
	},
	{
		/* ID_MISSED_NOTI_PURPLE = 30 */
		.r = mode14_red,
		.g = mode14_green,
		.b = mode14_blue,
		.size_r = ARRAY_SIZE(mode14_red),
		.size_g = ARRAY_SIZE(mode14_green),
		.size_b = ARRAY_SIZE(mode14_blue),
	},
	{
		/* ID_MISSED_NOTI_RED = 31 */
		.r = mode15_red,
		.g = mode15_green,
		.b = mode15_blue,
		.size_r = ARRAY_SIZE(mode15_red),
		.size_g = ARRAY_SIZE(mode15_green),
		.size_b = ARRAY_SIZE(mode15_blue),
	},
	{
		/* ID_MISSED_NOTI_LIME = 32 */
		.r = mode16_red,
		.g = mode16_green,
		.b = mode16_blue,
		.size_r = ARRAY_SIZE(mode16_red),
		.size_g = ARRAY_SIZE(mode16_green),
		.size_b = ARRAY_SIZE(mode16_blue),
	},
	{
		/* ID_NONE = 33 */
	},
	{
		/* ID_NONE = 34 */
	},
	{
		/* ID_INCALL = 35 */
	},
	{
		/* ID_NONE = 36 */
	},
	{
		/* ID_URGENT_CALL_MISSED_NOTI = 37 */
		.r = mode18_red,
		.g = mode18_green,
		.b = mode18_blue,
		.size_r = ARRAY_SIZE(mode18_red),
		.size_g = ARRAY_SIZE(mode18_green),
		.size_b = ARRAY_SIZE(mode18_blue),
	},
#endif
};

#define LP5521_CONFIGS (LP5521_PWM_HF | LP5521_PWRSAVE_EN | LP5521_CP_MODE_BYPASS | LP5521_CLOCK_EXT)

static int lp5521_setup(void)       
{
	mt_set_gpio_mode(GPIO30, GPIO_MODE_00); /* GPIO mode */
	mt_set_gpio_pull_enable(GPIO30, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO30, GPIO_DIR_OUT); /* GPIO direction */

	return 0;
}

static void lp5521_enable(bool state)
{
	if(state)
	{
		mt_set_gpio_mode(GPIO30, GPIO_MODE_00); /* GPIO mode */
		mt_set_gpio_pull_enable(GPIO30, GPIO_PULL_ENABLE);
		mt_set_gpio_dir(GPIO30, GPIO_DIR_OUT); /* GPIO direction */
		//usleep_range(1000, 2000);
		mt_set_gpio_out(GPIO30, GPIO_OUT_ONE);
		msleep(10);

		printk("lp5521_enable_set\n");
	}
	else
	{
		#if 0
		mt_set_gpio_mode(GPIO30, GPIO_MODE_00); /* GPIO mode */
		mt_set_gpio_pull_enable(GPIO30, GPIO_PULL_ENABLE);
		mt_set_gpio_dir(GPIO30, GPIO_DIR_OUT); /* GPIO direction */
		//usleep_range(1000, 2000);
		mt_set_gpio_out(GPIO30, GPIO_OUT_ZERO);
		msleep(10);
		#endif

		printk("lp5521_disable_set\n");
	}

	return;
}

static struct lp5521_platform_data lp5521_pdata = {
	.led_config = lp5521_led_config,
	.num_channels = ARRAY_SIZE(lp5521_led_config),
	#if 1 /* jjm_rgb */
	.clock_mode = LP5521_CLOCK_EXT,
	#else
	.clock_mode = LP5521_CLOCK_INT,
	#endif
	.update_config = LP5521_CONFIGS,
	.patterns = board_led_patterns,
	.num_patterns = ARRAY_SIZE(board_led_patterns),
    .setup_resources = lp5521_setup,
    .enable = lp5521_enable
};

static struct i2c_board_info lp5521_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("lp5521", 0x32), //  need verify the device address
		.platform_data = &lp5521_pdata, // uses platform data
	},
};
/*                                                                        */

enum lp5521_wait_type {
	LP5521_CYCLE_INVALID,
	LP5521_CYCLE_50ms,
	LP5521_CYCLE_100ms,
	LP5521_CYCLE_200ms,
	LP5521_CYCLE_500ms,
	LP5521_CYCLE_700ms,
	LP5521_CYCLE_920ms,
	LP5521_CYCLE_982ms,
	LP5521_CYCLE_MAX,
};

struct lp5521_engine {
	int		id;
	u8		mode;
	u8		prog_page;
	u8		engine_mask;
};

struct lp5521_led {
	int			id;
	u8			chan_nr;
	u8			led_current;
	u8			max_current;
	struct led_classdev	cdev;
	struct work_struct	brightness_work;
	u8			brightness;
};

struct lp5521_chip {
	struct lp5521_platform_data *pdata;
	struct mutex		lock; /* Serialize control */
	struct i2c_client	*client;
	struct lp5521_engine	engines[LP5521_MAX_ENGINES];
	struct lp5521_led	leds[LP5521_MAX_LEDS];
	u8			num_channels;
	u8			num_leds;
	#if 1 /* jjm_rgb */
	int			id_pattern_play;
	#endif
	u8			current_index;
};

struct lp5521_pattern_cmd {
	u8 r[LP5521_PROGRAM_LENGTH];
	u8 g[LP5521_PROGRAM_LENGTH];
	u8 b[LP5521_PROGRAM_LENGTH];
	unsigned pc_r;
	unsigned pc_g;
	unsigned pc_b;
};

struct lp5521_wait_param {
	unsigned cycle;
	unsigned limit;
	u8 cmd;
};

static const struct lp5521_wait_param lp5521_wait_params[LP5521_CYCLE_MAX] = {
	[LP5521_CYCLE_50ms] = {
		.cycle = 50,
		.limit = 3000,
		.cmd   = 0x43,
	},
	[LP5521_CYCLE_100ms] = {
		.cycle = 100,
		.limit = 6000,
		.cmd   = 0x46,
	},
	[LP5521_CYCLE_200ms] = {
		.cycle = 200,
		.limit = 10000,
		.cmd   = 0x4d,
	},
	[LP5521_CYCLE_500ms] = {
		.cycle = 500,
		.limit = 30000,
		.cmd   = 0x60,
	},
	[LP5521_CYCLE_700ms] = {
		.cycle = 700,
		.limit = 40000,
		.cmd   = 0x6d,
	},
	[LP5521_CYCLE_920ms] = {
		.cycle = 920,
		.limit = 50000,
		.cmd   = 0x7b,
	},
	[LP5521_CYCLE_982ms] = {
		.cycle = 982,
		.limit = 60000,
		.cmd   = 0x7f,
	},
};

static inline struct lp5521_led *cdev_to_led(struct led_classdev *cdev)
{
	return container_of(cdev, struct lp5521_led, cdev);
}

static inline struct lp5521_chip *engine_to_lp5521(struct lp5521_engine *engine)
{
	return container_of(engine, struct lp5521_chip,
			    engines[engine->id - 1]);
}

static inline struct lp5521_chip *led_to_lp5521(struct lp5521_led *led)
{
	return container_of(led, struct lp5521_chip,
			    leds[led->id]);
}

static void lp5521_led_brightness_work(struct work_struct *work);

static inline int lp5521_write(struct i2c_client *client, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static int lp5521_read(struct i2c_client *client, u8 reg, u8 *buf)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return -EIO;

	*buf = ret;
	return 0;
}

static int lp5521_set_engine_mode(struct lp5521_engine *engine, u8 mode)
{
	struct lp5521_chip *chip = engine_to_lp5521(engine);
	struct i2c_client *client = chip->client;
	int ret;
	u8 engine_state;

	/* Only transition between RUN and DIRECT mode are handled here */
	if (mode == LP5521_CMD_LOAD)
		return 0;

	if (mode == LP5521_CMD_DISABLED)
		mode = LP5521_CMD_DIRECT;

	ret = lp5521_read(client, LP5521_REG_OP_MODE, &engine_state);
	if (ret < 0)
		return ret;

	/* set mode only for this engine */
	engine_state &= ~(engine->engine_mask);
	mode &= engine->engine_mask;
	engine_state |= mode;
	return lp5521_write(client, LP5521_REG_OP_MODE, engine_state);
}

static int lp5521_load_program(struct lp5521_engine *eng, const u8 *pattern)
{
	struct lp5521_chip *chip = engine_to_lp5521(eng);
	struct i2c_client *client = chip->client;
	int ret;
	int addr;
	u8 mode = 0;

	/* move current engine to direct mode and remember the state */
	ret = lp5521_set_engine_mode(eng, LP5521_CMD_DIRECT);
	/* Mode change requires min 500 us delay. 1 - 2 ms  with margin */
	usleep_range(1000, 2000);
	ret |= lp5521_read(client, LP5521_REG_OP_MODE, &mode);

	/* For loading, all the engines to load mode */
	lp5521_write(client, LP5521_REG_OP_MODE, LP5521_CMD_DIRECT);
	/* Mode change requires min 500 us delay. 1 - 2 ms  with margin */
	usleep_range(1000, 2000);
	lp5521_write(client, LP5521_REG_OP_MODE, LP5521_CMD_LOAD);
	/* Mode change requires min 500 us delay. 1 - 2 ms  with margin */
	usleep_range(1000, 2000);

	addr = LP5521_PROG_MEM_BASE + eng->prog_page * LP5521_PROG_MEM_SIZE;
	i2c_smbus_write_i2c_block_data(client,
				addr,
				LP5521_PROG_MEM_SIZE,
				pattern);

	ret |= lp5521_write(client, LP5521_REG_OP_MODE, mode);
	return ret;
}

static int lp5521_set_led_current(struct lp5521_chip *chip, int led, u8 curr)
{
	return lp5521_write(chip->client,
		    LP5521_REG_LED_CURRENT_BASE + chip->leds[led].chan_nr,
		    curr);
}

static void lp5521_init_engine(struct lp5521_chip *chip)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(chip->engines); i++) {
		chip->engines[i].id = i + 1;
		chip->engines[i].engine_mask = LP5521_ENG_MASK_BASE >> (i * 2);
		chip->engines[i].prog_page = i;
	}
}

static int lp5521_configure(struct i2c_client *client)
{
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	int ret;
	u8 cfg;

	lp5521_init_engine(chip);

	/* Set all PWMs to direct control mode */
	ret = lp5521_write(client, LP5521_REG_OP_MODE, LP5521_CMD_DIRECT);

	cfg = chip->pdata->update_config ?
		: (LP5521_PWRSAVE_EN | LP5521_CP_MODE_AUTO | LP5521_R_TO_BATT);
	ret |= lp5521_write(client, LP5521_REG_CONFIG, cfg);

	/* Initialize all channels PWM to zero -> leds off */
	ret |= lp5521_write(client, LP5521_REG_R_PWM, 0);
	ret |= lp5521_write(client, LP5521_REG_G_PWM, 0);
	ret |= lp5521_write(client, LP5521_REG_B_PWM, 0);

	/* Set engines are set to run state when OP_MODE enables engines */
	ret |= lp5521_write(client, LP5521_REG_ENABLE,
			LP5521_ENABLE_RUN_PROGRAM);
	/* enable takes 500us. 1 - 2 ms leaves some margin */
	usleep_range(1000, 2000);

	return ret;
}

static int lp5521_run_selftest(struct lp5521_chip *chip, char *buf)
{
	int ret;
	u8 status;

	ret = lp5521_read(chip->client, LP5521_REG_STATUS, &status);
	if (ret < 0)
		return ret;

	/* Check that ext clock is really in use if requested */
	if (chip->pdata && chip->pdata->clock_mode == LP5521_CLOCK_EXT)
		if  ((status & LP5521_EXT_CLK_USED) == 0)
			return -EIO;
	return 0;
}

static void lp5521_set_brightness(struct led_classdev *cdev,
			     enum led_brightness brightness)
{
	struct lp5521_led *led = cdev_to_led(cdev);
	led->brightness = (u8)brightness;
	schedule_work(&led->brightness_work);
}

static void lp5521_led_brightness_work(struct work_struct *work)
{
	struct lp5521_led *led = container_of(work,
					      struct lp5521_led,
					      brightness_work);
	struct lp5521_chip *chip = led_to_lp5521(led);
	struct i2c_client *client = chip->client;

	mutex_lock(&chip->lock);
	lp5521_write(client, LP5521_REG_LED_PWM_BASE + led->chan_nr,
		led->brightness);
	mutex_unlock(&chip->lock);
}

/* Detect the chip by setting its ENABLE register and reading it back. */
static int lp5521_detect(struct i2c_client *client)
{
	int ret;
	u8 buf = 0;

	ret = lp5521_write(client, LP5521_REG_ENABLE, LP5521_ENABLE_DEFAULT);
	if (ret)
		return ret;
	/* enable takes 500us. 1 - 2 ms leaves some margin */
	usleep_range(1000, 2000);
	ret = lp5521_read(client, LP5521_REG_ENABLE, &buf);
	if (ret)
		return ret;
	if (buf != LP5521_ENABLE_DEFAULT)
		return -ENODEV;

	return 0;
}

/* Set engine mode and create appropriate sysfs attributes, if required. */
static int lp5521_set_mode(struct lp5521_engine *engine, u8 mode)
{
	int ret = 0;

	/* if in that mode already do nothing, except for run */
	if (mode == engine->mode && mode != LP5521_CMD_RUN)
		return 0;

	if (mode == LP5521_CMD_RUN) {
		ret = lp5521_set_engine_mode(engine, LP5521_CMD_RUN);
	} else if (mode == LP5521_CMD_LOAD) {
		lp5521_set_engine_mode(engine, LP5521_CMD_DISABLED);
		lp5521_set_engine_mode(engine, LP5521_CMD_LOAD);
	} else if (mode == LP5521_CMD_DISABLED) {
		lp5521_set_engine_mode(engine, LP5521_CMD_DISABLED);
	}

	engine->mode = mode;

	return ret;
}

static int lp5521_do_store_load(struct lp5521_engine *engine,
				const char *buf, size_t len)
{
	struct lp5521_chip *chip = engine_to_lp5521(engine);
	struct i2c_client *client = chip->client;
	int  ret, nrchars, offset = 0, i = 0;
	char c[3];
	unsigned cmd;
	u8 pattern[LP5521_PROGRAM_LENGTH] = {0};

	while ((offset < len - 1) && (i < LP5521_PROGRAM_LENGTH)) {
		/* separate sscanfs because length is working only for %s */
		ret = sscanf(buf + offset, "%2s%n ", c, &nrchars);
		if (ret != 2)
			goto fail;
		ret = sscanf(c, "%2x", &cmd);
		if (ret != 1)
			goto fail;
		pattern[i] = (u8)cmd;

		offset += nrchars;
		i++;
	}

	/* Each instruction is 16bit long. Check that length is even */
	if (i % 2)
		goto fail;

	mutex_lock(&chip->lock);
	if (engine->mode == LP5521_CMD_LOAD)
		ret = lp5521_load_program(engine, pattern);
	else
		ret = -EINVAL;
	mutex_unlock(&chip->lock);

	if (ret) {
		dev_err(&client->dev, "failed loading pattern\n");
		return ret;
	}

	return len;
fail:
	dev_err(&client->dev, "wrong pattern format\n");
	return -EINVAL;
}

static ssize_t store_engine_load(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	return lp5521_do_store_load(&chip->engines[nr - 1], buf, len);
}

#define store_load(nr)							\
static ssize_t store_engine##nr##_load(struct device *dev,		\
				     struct device_attribute *attr,	\
				     const char *buf, size_t len)	\
{									\
	return store_engine_load(dev, attr, buf, len, nr);		\
}
store_load(1)
store_load(2)
store_load(3)

static ssize_t show_engine_mode(struct device *dev,
				struct device_attribute *attr,
				char *buf, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	switch (chip->engines[nr - 1].mode) {
	case LP5521_CMD_RUN:
		return sprintf(buf, "run\n");
	case LP5521_CMD_LOAD:
		return sprintf(buf, "load\n");
	case LP5521_CMD_DISABLED:
		return sprintf(buf, "disabled\n");
	default:
		return sprintf(buf, "disabled\n");
	}
}

#define show_mode(nr)							\
static ssize_t show_engine##nr##_mode(struct device *dev,		\
				    struct device_attribute *attr,	\
				    char *buf)				\
{									\
	return show_engine_mode(dev, attr, buf, nr);			\
}
show_mode(1)
show_mode(2)
show_mode(3)

static ssize_t store_engine_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	struct lp5521_engine *engine = &chip->engines[nr - 1];
	mutex_lock(&chip->lock);

	if (!strncmp(buf, "run", 3))
		lp5521_set_mode(engine, LP5521_CMD_RUN);
	else if (!strncmp(buf, "load", 4))
		lp5521_set_mode(engine, LP5521_CMD_LOAD);
	else if (!strncmp(buf, "disabled", 8))
		lp5521_set_mode(engine, LP5521_CMD_DISABLED);

	mutex_unlock(&chip->lock);
	return len;
}

#define store_mode(nr)							\
static ssize_t store_engine##nr##_mode(struct device *dev,		\
				     struct device_attribute *attr,	\
				     const char *buf, size_t len)	\
{									\
	return store_engine_mode(dev, attr, buf, len, nr);		\
}
store_mode(1)
store_mode(2)
store_mode(3)

static ssize_t show_max_current(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5521_led *led = cdev_to_led(led_cdev);

	return sprintf(buf, "%d\n", led->max_current);
}

static ssize_t show_current(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5521_led *led = cdev_to_led(led_cdev);

	return sprintf(buf, "%d\n", led->led_current);
}

static ssize_t store_current(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5521_led *led = cdev_to_led(led_cdev);
	struct lp5521_chip *chip = led_to_lp5521(led);
	ssize_t ret;
	unsigned long curr;

	if (kstrtoul(buf, 0, &curr))
		return -EINVAL;

	if (curr > led->max_current)
		return -EINVAL;

	mutex_lock(&chip->lock);
	ret = lp5521_set_led_current(chip, led->id, curr);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		return ret;

	led->led_current = (u8)curr;

	return len;
}

static ssize_t lp5521_selftest(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->lock);
	ret = lp5521_run_selftest(chip, buf);
	mutex_unlock(&chip->lock);
	return sprintf(buf, "%s\n", ret ? "FAIL" : "OK");
}

static void lp5521_clear_program_memory(struct i2c_client *cl)
{
	int i;
	u8 rgb_mem[] = {
		LP5521_REG_R_PROG_MEM,
		LP5521_REG_G_PROG_MEM,
		LP5521_REG_B_PROG_MEM,
	};

	for (i = 0; i < ARRAY_SIZE(rgb_mem); i++) {
		lp5521_write(cl, rgb_mem[i], 0);
		lp5521_write(cl, rgb_mem[i] + 1, 0);
	}
}

static void lp5521_write_program_memory(struct i2c_client *cl,
				u8 base, u8 *rgb, int size)
{
	int i;

	if (!rgb || size <= 0)
		return;

	for (i = 0; i < size; i++)
		lp5521_write(cl, base + i, *(rgb + i));

	lp5521_write(cl, base + i, 0);
	lp5521_write(cl, base + i + 1, 0);
}

static inline struct lp5521_led_pattern *lp5521_get_pattern
					(struct lp5521_chip *chip, u8 offset)
{
	struct lp5521_led_pattern *ptn;
	ptn = chip->pdata->patterns + (offset - 1);
	return ptn;
}

static void _run_led_pattern(struct lp5521_chip *chip,
			struct lp5521_led_pattern *ptn)
{
	struct i2c_client *cl = chip->client;

	lp5521_write(cl, LP5521_REG_OP_MODE, LP5521_CMD_LOAD);
	usleep_range(1000, 2000);

	lp5521_clear_program_memory(cl);

	lp5521_write_program_memory(cl, LP5521_REG_R_PROG_MEM,
				ptn->r, ptn->size_r);
	lp5521_write_program_memory(cl, LP5521_REG_G_PROG_MEM,
				ptn->g, ptn->size_g);
	lp5521_write_program_memory(cl, LP5521_REG_B_PROG_MEM,
				ptn->b, ptn->size_b);

	lp5521_write(cl, LP5521_REG_OP_MODE, LP5521_CMD_RUN);
	usleep_range(1000, 2000);
	lp5521_write(cl, LP5521_REG_ENABLE, LP5521_ENABLE_RUN_PROGRAM);
}

static void lp5521_run_led_pattern(int mode, struct lp5521_chip *chip)
{
	struct lp5521_led_pattern *ptn;
	struct i2c_client *cl = chip->client;
	int num_patterns = chip->pdata->num_patterns;

	if (mode > num_patterns || !(chip->pdata->patterns))
		return;

	#if 1 /* jjm_rgb */
	chip->id_pattern_play = mode;
	#endif

	#if 0 /* jjm_rgb */
	if (mode == PATTERN_FAVORITE_MISSED_NOTI) {
	      mode = num_patterns;
	}
	#endif

	if (mode == PATTERN_OFF) {
		lp5521_write(cl, LP5521_REG_ENABLE, LP5521_ENABLE_DEFAULT);
		usleep_range(1000, 2000);
		lp5521_write(cl, LP5521_REG_OP_MODE, LP5521_CMD_DIRECT);
	} else {
		ptn = lp5521_get_pattern(chip, mode);
		if (!ptn)
			return;

		_run_led_pattern(chip, ptn);
	}
}

#if 1 /* jjm_rgb */
static ssize_t show_led_pattern(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lp5521_chip *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%d\n", chip->id_pattern_play);
}
#endif

static u8 get_led_current_value(u8 current_index)
{
	return current_index_mapped_value[current_index];
}

static ssize_t store_led_pattern(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct lp5521_chip *chip = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long val;
	int ret;

	#if 1 /* jjm_rgb */
	ret = strict_strtoul(buf, 10, &val);
	#else
	ret = strict_strtoul(buf, 16, &val);
	#endif
	if (ret)
		return ret;

	lp5521_run_led_pattern(val, chip);

	return len;
}

static ssize_t show_led_current_index(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lp5521_chip *chip = i2c_get_clientdata(to_i2c_client(dev));
	if (!chip)
		return 0;

	return sprintf(buf, "%d\n", chip->current_index);
}

static ssize_t store_led_current_index(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct lp5521_chip *chip = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long val;
	int ret, i;
	u8 max_current, modify_current;

	printk(KERN_INFO"[%s] current index (0~255) : %s", __func__, buf);

	ret = strict_strtoul(buf, 10, &val);
	if (ret)
		return ret;

	if (val > PATTERN_CURRENT_INDEX_STEP_HAL || val < 0)
		return -EINVAL;

	if (!chip)
		return 0;

	chip->current_index = val;

	mutex_lock(&chip->lock);
	for (i = 0; i < LP5521_MAX_LEDS ; i++)
	{
		max_current = chip->leds[i].max_current;
		modify_current = get_led_current_value(val);
		if (modify_current > LP5521_MAX_LED_CURRENT)
			modify_current = LP5521_MAX_LED_CURRENT;

		switch(i)
		{
			case LP5521_R:/*Red*/
				ret = lp5521_set_led_current(chip, i, modify_current);
				printk(KERN_INFO"[%s][R] modify_current : %d", __func__,  modify_current);
				break;
			case LP5521_G:/*Green*/
				modify_current = (modify_current/LP5521_R_INDEX)*LP5521_G_INDEX;
				ret = lp5521_set_led_current(chip, i, modify_current);
				printk(KERN_INFO"[%s][G] modify_current : %d", __func__,  modify_current);
				break;
			case LP5521_B:/*Blue*/
				modify_current = (modify_current/LP5521_R_INDEX)*LP5521_B_INDEX;
				ret = lp5521_set_led_current(chip, i, modify_current);
				printk(KERN_INFO"[%s][B] modify_current : %d", __func__,  modify_current);
				break;
			default:
				printk(KERN_INFO"[%s] This message should not print : %d", __func__, modify_current);
				break;
		}

		if (ret)
			return ret;
		chip->leds[i].led_current = modify_current;
	}
	mutex_unlock(&chip->lock);

	return len;
}

static void _set_pwm_cmd(struct lp5521_pattern_cmd *cmd, unsigned int color)
{
	u8 r = (color >> 16) & 0xFF;
	u8 g = (color >> 8) & 0xFF;
	u8 b = color & 0xFF;

	cmd->r[cmd->pc_r++] = CMD_SET_PWM;
	cmd->r[cmd->pc_r++] = r;
	cmd->g[cmd->pc_g++] = CMD_SET_PWM;
	cmd->g[cmd->pc_g++] = g;
	cmd->b[cmd->pc_b++] = CMD_SET_PWM;
	cmd->b[cmd->pc_b++] = b;
}

static enum lp5521_wait_type _find_wait_cycle_type(unsigned int ms)
{
	int i;

	for (i = LP5521_CYCLE_50ms ; i < LP5521_CYCLE_MAX ; i++) {
		if (ms > lp5521_wait_params[i-1].limit &&
		    ms <= lp5521_wait_params[i].limit)
			return i;
	}

	return LP5521_CYCLE_INVALID;
}

static void _set_wait_cmd(struct lp5521_pattern_cmd *cmd,
			unsigned int ms, u8 jump)
{
	enum lp5521_wait_type type = _find_wait_cycle_type(ms);
	unsigned int loop = ms / lp5521_wait_params[type].cycle;
	u8 cmd_msb = lp5521_wait_params[type].cmd;
	u8 msb;
	u8 lsb;
	u16 branch;

	WARN_ON(!cmd_msb);
	WARN_ON(loop > 64);

	/* wait command */
	cmd->r[cmd->pc_r++] = cmd_msb;
	cmd->r[cmd->pc_r++] = CMD_WAIT_LSB;
	cmd->g[cmd->pc_g++] = cmd_msb;
	cmd->g[cmd->pc_g++] = CMD_WAIT_LSB;
	cmd->b[cmd->pc_b++] = cmd_msb;
	cmd->b[cmd->pc_b++] = CMD_WAIT_LSB;

	/* branch command : if wait time is bigger than cycle msec,
			branch is used for command looping */
	if (loop > 1) {
		branch = (5 << 13) | ((loop - 1) << 7) | jump;
		msb = (branch >> 8) & 0xFF;
		lsb = branch & 0xFF;

		cmd->r[cmd->pc_r++] = msb;
		cmd->r[cmd->pc_r++] = lsb;
		cmd->g[cmd->pc_g++] = msb;
		cmd->g[cmd->pc_g++] = lsb;
		cmd->b[cmd->pc_b++] = msb;
		cmd->b[cmd->pc_b++] = lsb;
	}
}

static inline bool _is_pc_overflow(struct lp5521_led_pattern *ptn)
{
	return (ptn->size_r >= LP5521_PROGRAM_LENGTH ||
		ptn->size_g >= LP5521_PROGRAM_LENGTH ||
		ptn->size_b >= LP5521_PROGRAM_LENGTH);
}

static ssize_t store_led_blink(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct lp5521_chip *chip = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int rgb = 0;
	unsigned int on = 0;
	unsigned int off = 0;
	struct lp5521_led_pattern ptn = { };
	struct lp5521_pattern_cmd cmd = { };
	u8 jump_pc = 0;

	sscanf(buf, "0x%06x %d %d", &rgb, &on, &off);

	lp5521_run_led_pattern(PATTERN_OFF, chip);

	on = min_t(unsigned int, on, MAX_BLINK_TIME);
	off = min_t(unsigned int, off, MAX_BLINK_TIME);

	#if 1 /* jjm_rgb */
	if (!rgb || !on || !off)
	{
		chip->id_pattern_play = PATTERN_OFF;
		return len;
	}
	else
	{
		chip->id_pattern_play = PATTERN_BLINK_ON;
	}
	#else
	if (!rgb || !on || !off)
		return len;
	#endif

	/* on */
	_set_pwm_cmd(&cmd, rgb);
	_set_wait_cmd(&cmd, on, jump_pc);
	jump_pc = cmd.pc_r / 2; /* 16bit size program counter */

	/* off */
	_set_pwm_cmd(&cmd, 0);
	_set_wait_cmd(&cmd, off, jump_pc);

	ptn.r = cmd.r;
	ptn.size_r = cmd.pc_r;
	ptn.g = cmd.g;
	ptn.size_g = cmd.pc_g;
	ptn.b = cmd.b;
	ptn.size_b = cmd.pc_b;

	WARN_ON(_is_pc_overflow(&ptn));

	_run_led_pattern(chip, &ptn);

	return len;
}

/* led class device attributes */
static DEVICE_ATTR(led_current, S_IRUGO | S_IWUSR, show_current, store_current);
static DEVICE_ATTR(max_current, S_IRUGO , show_max_current, NULL);

static struct attribute *lp5521_led_attributes[] = {
	&dev_attr_led_current.attr,
	&dev_attr_max_current.attr,
	NULL,
};

static struct attribute_group lp5521_led_attribute_group = {
	.attrs = lp5521_led_attributes
};

/* device attributes */
static DEVICE_ATTR(engine1_mode, S_IRUGO | S_IWUSR,
		   show_engine1_mode, store_engine1_mode);
static DEVICE_ATTR(engine2_mode, S_IRUGO | S_IWUSR,
		   show_engine2_mode, store_engine2_mode);
static DEVICE_ATTR(engine3_mode, S_IRUGO | S_IWUSR,
		   show_engine3_mode, store_engine3_mode);
static DEVICE_ATTR(engine1_load, S_IWUSR, NULL, store_engine1_load);
static DEVICE_ATTR(engine2_load, S_IWUSR, NULL, store_engine2_load);
static DEVICE_ATTR(engine3_load, S_IWUSR, NULL, store_engine3_load);
static DEVICE_ATTR(selftest, S_IRUGO, lp5521_selftest, NULL);
#if 1 /* jjm_rgb */
static DEVICE_ATTR(led_pattern, S_IRUGO | S_IWUSR, show_led_pattern, store_led_pattern);
#else
static DEVICE_ATTR(led_pattern, S_IRUGO | S_IWUSR, NULL, store_led_pattern);
#endif
static DEVICE_ATTR(led_blink, S_IRUGO | S_IWUSR, NULL, store_led_blink);
static DEVICE_ATTR(led_current_index, S_IRUGO | S_IWUSR, show_led_current_index, store_led_current_index);

static struct attribute *lp5521_attributes[] = {
	&dev_attr_engine1_mode.attr,
	&dev_attr_engine2_mode.attr,
	&dev_attr_engine3_mode.attr,
	&dev_attr_selftest.attr,
	&dev_attr_engine1_load.attr,
	&dev_attr_engine2_load.attr,
	&dev_attr_engine3_load.attr,
	&dev_attr_led_pattern.attr,
	&dev_attr_led_blink.attr,
	&dev_attr_led_current_index.attr,
	NULL
};

static const struct attribute_group lp5521_group = {
	.attrs = lp5521_attributes,
};

static int lp5521_register_sysfs(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	return sysfs_create_group(&dev->kobj, &lp5521_group);
}

static void lp5521_unregister_sysfs(struct i2c_client *client)
{
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int i;

	sysfs_remove_group(&dev->kobj, &lp5521_group);

	for (i = 0; i < chip->num_leds; i++)
		sysfs_remove_group(&chip->leds[i].cdev.dev->kobj,
				&lp5521_led_attribute_group);
}

static int __devinit lp5521_init_led(struct lp5521_led *led,
				struct i2c_client *client,
				int chan, struct lp5521_platform_data *pdata)
//static int lp5521_init_led(struct lp5521_led *led,
				//struct i2c_client *client,
				//int chan, struct lp5521_platform_data *pdata)
{
	struct device *dev = &client->dev;
	char name[32];
	int res;

	if (chan >= LP5521_MAX_LEDS)
		return -EINVAL;

	if (pdata->led_config[chan].led_current == 0)
		return 0;

	led->led_current = pdata->led_config[chan].led_current;
	led->max_current = pdata->led_config[chan].max_current;
	led->chan_nr = pdata->led_config[chan].chan_nr;

	if (led->chan_nr >= LP5521_MAX_LEDS) {
		dev_err(dev, "Use channel numbers between 0 and %d\n",
			LP5521_MAX_LEDS - 1);
		return -EINVAL;
	}

	led->cdev.brightness_set = lp5521_set_brightness;
	if (pdata->led_config[chan].name) {
		led->cdev.name = pdata->led_config[chan].name;
	} else {
		snprintf(name, sizeof(name), "%s:channel%d",
			pdata->label ?: client->name, chan);
		led->cdev.name = name;
	}

	res = led_classdev_register(dev, &led->cdev);
	if (res < 0) {
		dev_err(dev, "couldn't register led on channel %d\n", chan);
		return res;
	}

	res = sysfs_create_group(&led->cdev.dev->kobj,
			&lp5521_led_attribute_group);
	if (res < 0) {
		dev_err(dev, "couldn't register current attribute\n");
		led_classdev_unregister(&led->cdev);
		return res;
	}
	return 0;
}

static int __devinit lp5521_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
//static int lp5521_probe(struct i2c_client *client,
			//const struct i2c_device_id *id)
{
	struct lp5521_chip		*chip;
	struct lp5521_platform_data	*pdata;
	int ret, i, led;
	u8 buf = 0;

	printk("LP5521: probe start \n\n");

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);

	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);
	chip->client = client;

	pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	mutex_init(&chip->lock);

	chip->pdata   = pdata;

	if (pdata->setup_resources) {
		ret = pdata->setup_resources();
		if (ret < 0)
			return ret;
	}

	if (pdata->enable) {
		pdata->enable(0);
		usleep_range(1000, 2000); /* Keep enable down at least 1ms */
		pdata->enable(1);
		usleep_range(1000, 2000); /* 500us abs min. */
	}

	lp5521_write(client, LP5521_REG_RESET, 0xff);
	usleep_range(10000, 20000); /*
				     * Exact value is not available. 10 - 20ms
				     * appears to be enough for reset.
				     */
	/*
	 * Make sure that the chip is reset by reading back the r channel
	 * current reg. This is dummy read is required on some platforms -
	 * otherwise further access to the R G B channels in the
	 * LP5521_REG_ENABLE register will not have any effect - strange!
	 */
	ret = lp5521_read(client, LP5521_REG_R_CURRENT, &buf);
	if (ret || buf != LP5521_REG_R_CURR_DEFAULT) {
		dev_err(&client->dev, "error in resetting chip\n");
		goto fail2;
	}
	usleep_range(10000, 20000);

	ret = lp5521_detect(client);

	if (ret) {
		dev_err(&client->dev, "Chip not found\n");
		goto fail2;
	}

	dev_info(&client->dev, "%s programmable led chip found\n", id->name);

	ret = lp5521_configure(client);
	if (ret < 0) {
		dev_err(&client->dev, "error configuring chip\n");
		goto fail1;
	}

	/* Initialize leds */
	chip->num_channels = pdata->num_channels;
	chip->num_leds = 0;
	led = 0;
	for (i = 0; i < pdata->num_channels; i++) {
		/* Do not initialize channels that are not connected */
		if (pdata->led_config[i].led_current == 0)
			continue;

		ret = lp5521_init_led(&chip->leds[led], client, i, pdata);
		if (ret) {
			dev_err(&client->dev, "error initializing leds\n");
			goto fail2;
		}
		chip->num_leds++;

		chip->leds[led].id = led;
		/* Set initial LED current */
		lp5521_set_led_current(chip, led,
				chip->leds[led].led_current);

		INIT_WORK(&(chip->leds[led].brightness_work),
			lp5521_led_brightness_work);

		led++;
	}

	/* Initialize current index for auto brightness (max step) */
	chip->current_index = chip->leds[0].led_current;

	ret = lp5521_register_sysfs(client);
	if (ret) {
		dev_err(&client->dev, "registering sysfs failed\n");
		goto fail2;
	}
	
	printk("LP5521: probe END\n\n");

	lp5521_run_led_pattern(1, chip);
	
	return ret;
fail2:
	for (i = 0; i < chip->num_leds; i++) {
		led_classdev_unregister(&chip->leds[i].cdev);
		cancel_work_sync(&chip->leds[i].brightness_work);
	}
fail1:
	if (pdata->enable)
		pdata->enable(0);
	if (pdata->release_resources)
		pdata->release_resources();
	return ret;
}

static int __devexit lp5521_remove(struct i2c_client *client)
{
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	int i;

	lp5521_run_led_pattern(PATTERN_OFF, chip);
	lp5521_unregister_sysfs(client);

	for (i = 0; i < chip->num_leds; i++) {
		led_classdev_unregister(&chip->leds[i].cdev);
		cancel_work_sync(&chip->leds[i].brightness_work);
	}

	if (chip->pdata->enable)
		chip->pdata->enable(0);
	if (chip->pdata->release_resources)
		chip->pdata->release_resources();

	#if 1 /* jjm_rgb */
	devm_kfree(&client->dev, chip);
	#else
	kfree(chip);
	#endif

	return 0;
}

#if 1 /* jjm_rgb */
static void lp5521_shutdown(struct i2c_client *client)
{
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	int i;

	if (!chip)
	{
		printk("LP5521: [%s] null pointer check!\n", __func__);
		return;
	}

	lp5521_set_led_current(chip, 0, 0);
	lp5521_set_led_current(chip, 1, 0);
	lp5521_set_led_current(chip, 2, 0);
	lp5521_run_led_pattern(PATTERN_OFF, chip);
	lp5521_unregister_sysfs(client);

	for (i = 0; i < chip->num_leds; i++) {
		led_classdev_unregister(&chip->leds[i].cdev);
		cancel_work_sync(&chip->leds[i].brightness_work);
	}

	if (chip->pdata->enable)
		chip->pdata->enable(0);
	if (chip->pdata->release_resources)
		chip->pdata->release_resources();

	devm_kfree(&client->dev, chip);
}

static int lp5521_suspend (struct i2c_client *client, pm_message_t mesg)
{
	struct lp5521_chip *chip = i2c_get_clientdata(client);

	if (!chip || !chip->pdata)
	{
		printk("LP5521: [%s] null pointer check!\n", __func__);
		return 0;
	}

	if (chip->pdata->enable && chip->id_pattern_play == PATTERN_OFF)
	{
		chip->pdata->enable(0);
	}

	return 0;
}

static int lp5521_resume (struct i2c_client *client)
{
	struct lp5521_chip *chip = i2c_get_clientdata(client);

	if (!chip || !chip->pdata)
	{
		printk("LP5521: [%s] null pointer check!\n", __func__);
		return 0;
	}

	if (chip->pdata->enable && chip->id_pattern_play == PATTERN_OFF)
	{
		chip->pdata->enable(0);
		mdelay(100);
		//usleep_range(1000, 2000); /* Keep enable down at least 1ms */

		chip->pdata->enable(1);
		mdelay(100);
		//usleep_range(1000, 2000); /* 500us abs min. */
	}

	return 0;
}
#endif

static const struct i2c_device_id lp5521_id[] = {
	{ "lp5521", 0 }, /* Three channel chip */
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp5521_id);

#if 1
static struct i2c_driver lp5521_driver = {
	.driver = {
		.name	= "lp5521",
	},
	.probe		= lp5521_probe,
	.remove		= __devexit_p(lp5521_remove),
	//.shutdown	= lp5521_shutdown,
	//.suspend	= lp5521_suspend,
	//.resume 	= lp5521_resume,
	.id_table	= lp5521_id,
};
#else
static struct i2c_driver lp5521_driver = {
	.driver = {
		.name	= "lp5521",
	},
	.probe		= lp5521_probe,
	//.remove		= __devexit_p(lp5521_remove),
	.remove		= lp5521_remove,
	.id_table	= lp5521_id,
};
#endif

/*                                                                        */

static int lp5521_pd_probe(struct platform_device *pdev) 
{
	printk("lp5521_pd_probe \n");
	// GPIO enable logic is shifted to other function
	i2c_add_driver(&lp5521_driver);
	return 0;
}

static int lp5521_pd_remove(struct platform_device *pdev)
{
	printk("lp5521_pd_remove \n");
    i2c_del_driver(&lp5521_driver);
    return 0;
}

static struct platform_driver lp5521_led_driver = {
	.probe      = lp5521_pd_probe,
	.remove     = lp5521_pd_remove,
	.driver     = {
		.name  = "lp5521",
		.owner = THIS_MODULE,
	}
};

static int __init lp5521_init(void){
    printk(" lp5521_init \n");

	i2c_register_board_info(1, lp5521_board_info,ARRAY_SIZE(lp5521_board_info));
    printk("lp5521_i2c_regiter_board_info\n\n\n\n\n\n");
	
	//register_early_suspend(&lm3639_early_suspend_desc); // not needed
	
	if(platform_driver_register(&lp5521_led_driver))
	{
		printk("failed to register driver\n");
		return -1;
	}

return 0;
}

static void __exit lp5521_exit(void)
{
	platform_driver_unregister(&lp5521_led_driver);
}
/*                                                                         */

//module_i2c_driver(lp5521_driver);

MODULE_AUTHOR("Mathias Nyman, Yuri Zaporozhets, Samu Onkalo");
MODULE_DESCRIPTION("LP5521 LED engine");
MODULE_LICENSE("GPL v2");

/*                                                                         */
module_init(lp5521_init);
module_exit(lp5521_exit);
/*                                                                         */
