/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_axi2mipi.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-01-2016
 * @brief	This file contains functions used to initialize
 * 			module AXI to MIPI
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "hal_axi2mipi.h"
#include "hal_vic.h"
#include "assert.h"
#include "board_config.h"
#include "cortex_r4.h"

/* Private define ------------------------------------------------------------*/
/* MIPI TX Controller register (not AXI to MIPI module) */
#define PHY_STATUS  			0x110
#define PHY_RSTZ_AD				0xE0
#define PHY_CTRL0_AD			0x114
#define PHY_CTRL1_AD			0x118
#define CSI2_RESETN_AD			0x4
#define PHY_IF_CFG_AD			0xE4
#define CLKMGR_CFG_AD			0xF0
#define LPCLK_CTRL				0xE8

#define	INSTANCE_OFFSET			0x1000

#define SHIFT_TO_BIT0			0
#define SHIFT_TO_BIT1			1
#define SHIFT_TO_BIT2			2
#define SHIFT_TO_BIT3			3
#define SHIFT_TO_BIT4			4
#define SHIFT_TO_BIT5			5
#define SHIFT_TO_BIT6			6
#define SHIFT_TO_BIT7			7
#define SHIFT_TO_BIT8			8
#define SHIFT_TO_BIT9			9
#define SHIFT_TO_BIT10			10
#define SHIFT_TO_BIT11			11
#define SHIFT_TO_BIT12			12
#define SHIFT_TO_BIT13			13
#define SHIFT_TO_BIT14			14
#define SHIFT_TO_BIT15			15
#define SHIFT_TO_BIT16			16
#define SHIFT_TO_BIT17			17
#define SHIFT_TO_BIT18			18
#define SHIFT_TO_BIT19			19
#define SHIFT_TO_BIT20			20
#define SHIFT_TO_BIT21			21
#define SHIFT_TO_BIT22			22
#define SHIFT_TO_BIT23			23
#define SHIFT_TO_BIT24			24
#define SHIFT_TO_BIT25			25
#define SHIFT_TO_BIT26			26
#define SHIFT_TO_BIT27			27
#define SHIFT_TO_BIT28			28
#define SHIFT_TO_BIT29			29
#define SHIFT_TO_BIT30			30
#define SHIFT_TO_BIT31			31

#define ISR_DISABLE_ALL			(uint32_t)(0x0fffffff)

#define	HALF_DWORD_MASK			0xffff0000

#define	EIGHT_BITS_PER_PIXEL				8
#define	TEN_BITS_PER_PIXEL					10
#define	TWELVE_BITS_PER_PIXEL				12
#define	FOURTEEN_BITS_PER_PIXEL				14

/* This is the best value in real testing */
#define HS_WIDTH_VAL	10

/* Private macro -------------------------------------------------------------*/
/* Macro to align number x in multiple of 16 */
#define GET_MULTIPLES_16(x) (((x - 1) | 15) + 1)

/* Write and Read directly to register macros */
#define SET_REG(addr, val) (*((volatile uint32_t *)(addr)) = (uint32_t)val)
#define READ_REG(addr) (*(volatile uint32_t *)(addr));

/* Private typedef -----------------------------------------------------------*/
typedef struct mipi_struct
{
	axi2mipi_callback callback;
	void *user_data;
	uint8_t is_init;
} mipi_struct_t;

/* Private variables ---------------------------------------------------------*/
mipi_struct_t axi2mipi_hw[2] =
{
	{
		.callback	= NULL,
		.user_data	= NULL,
		.is_init = 0
	},

	{
		.callback	= NULL,
		.user_data	= NULL,
		.is_init = 0
	}
};

/* This variable used for backup configurations to BRIDGE_CTRL_REG, because when
 * read back from register, several bits (enable bits) cannot read back
 */
__IO uint32_t bridge_reg[2];

/* Private function prototypes -----------------------------------------------*/
static void hal_axi2mipi_0_isr(void);
static void hal_axi2mipi_1_isr(void);
static void axi2mipi_isr_hdl(axi2mipi_channel_t channel);

/* Private functions ---------------------------------------------------------*/
static int convert_in_halfword(uint16_t width, axi2mipi_data_type_t data_type,
								axi2mipi_stream_mode_t mode)
{
	int num_of_halfword;
	int bits_per_pixel;
	switch(data_type)
	{
		case AXI2MIPI_RAW10:
		{
			bits_per_pixel = TEN_BITS_PER_PIXEL;
			break;
		}
		case AXI2MIPI_RAW12:
		{
			bits_per_pixel = TWELVE_BITS_PER_PIXEL;
			break;
		}
		case AXI2MIPI_RAW14:
		{
			bits_per_pixel = FOURTEEN_BITS_PER_PIXEL;
			break;
		}
		case AXI2MIPI_RAW8:
		default:
		{
			bits_per_pixel = EIGHT_BITS_PER_PIXEL;
			break;
		}
	}
	if(mode == AXI2MIPI_SNAPSHOT_MODE)
	{
		/* 10bits per pixel would be packed in 16 bytes */
		num_of_halfword = (width * bits_per_pixel) / 8;
		/* If it is not exactly multiple of 16 then do +1 to it */
		if((num_of_halfword % 16) != 0)
			num_of_halfword = (num_of_halfword / 16) + 1;
		else
			num_of_halfword = (num_of_halfword / 16);
	}
	else
	{
		/* num_of_halfword should be num_of_pixels*2/16
		 * because 1 pixel occupies 2 bytes */
		num_of_halfword = (width * 2) / 16;
	}
	return num_of_halfword;
}

static void axi2mipi_isr_hdl(axi2mipi_channel_t channel)
{
	__IO uint32_t isr_flag = 0;

	/* Read interrupt status register */
	isr_flag = READ_AXI2MIPI_REG(channel, INTR_STS_REG);
	if(axi2mipi_hw[channel].callback)
		/* Check all interrupt flags */
		axi2mipi_hw[channel].callback(
									channel,
									isr_flag,
									axi2mipi_hw[channel].user_data);

	/* Clear interrupt register */
	WRITE_AXI2MIPI_REG(channel, INTR_CLR_REG, isr_flag);
}
static void csi2_device_isr(void)
{
	unsigned int isr_st = readl(CSLDS0_BASE + 0x20);
	if(isr_st)
		printf("\r\nTX INT_ST_MAIN: 0x%x \r\n", isr_st);

	isr_st = readl(CSLDS0_BASE + 0x24);
	if(isr_st)
		printf("\r\nTX INT_ST_VPG: 0x%x \r\n", isr_st);

	isr_st = readl(CSLDS0_BASE + 0x28);
	if(isr_st)
		printf("\r\nTX INT_ST_IDI: 0x%x \r\n", isr_st);

	isr_st = readl(CSLDS0_BASE + 0x2c);
	if(isr_st)
		printf("\r\nTX INT_ST_MEM: 0x%x \r\n", isr_st);
}
static void hal_axi2mipi_0_isr(void)
{
	axi2mipi_isr_hdl(AXI2MIPI_BRIDGE_0);
}

static void hal_axi2mipi_1_isr(void)
{
	axi2mipi_isr_hdl(AXI2MIPI_BRIDGE_1);
}

static void phy_set_ctrl1(unsigned int addr_offset, unsigned int testen, unsigned int testdin)
{
	SET_REG(addr_offset, (testen << SHIFT_TO_BIT16) + testdin);
}

static void phy_set_ctrl0(unsigned int addr_offset, unsigned int testclk, unsigned int testclr)
{
	SET_REG(addr_offset, (testclk << SHIFT_TO_BIT1) + testclr);
}

#if BURST
static void mipi_tx_dphy_init(int ins_idx)
{
	__IO uint32_t CSIDPHY_base = CSLDS0_BASE + (INSTANCE_OFFSET * ins_idx);
#ifdef MIPI_SPEED_1500MHZ
	SET_REG(CSIDPHY_base + PHY_RSTZ_AD, 0x00000000);	/*phy rstz in reset*/

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x0);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x1);

	vTaskDelay(1);

	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	vTaskDelay(1);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x44);	/*for normal operation*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x78);	/*1.5G*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0) ;

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x19);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x30);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x17);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x02);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x18);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x05);	/*m bit[4:0]*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x18);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x85);	/*m bit[7:5]*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x10);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xb8);	/*vcorang=7*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x11);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x07);	/*icpctrl=7*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x12);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xc8);	/*lpfctrl=8*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x22);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xd8);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x22);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x1b);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);
 
/* macro is used for switching the mipi speed to 400Mhz*/
#else
	SET_REG(CSIDPHY_base + PHY_RSTZ_AD, 0x00000000);  /*phy rstz in reset*/

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x0);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x1);

	vTaskDelay(1);

	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	vTaskDelay(1);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x44);	/*for normal operation*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x4A);	/*400Mhz*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0) ;

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x19);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x30);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x17);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x02); /* n*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x18);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x0b);	/*m bit[4:0]*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x18);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x81);	/*m bit[7:5]*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x10);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x90);	/*vcorang=2*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x11);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x05);	/*icpctrl=5*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x12);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xc8);	/*lpfctrl=8*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x22);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xd8);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x22);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x1b);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);
#endif
	vTaskDelay(1);

	SET_REG(CSIDPHY_base + PHY_RSTZ_AD, 0x00000004);	/*phy rstz  enable clck*/ 
	SET_REG(CSIDPHY_base + PHY_RSTZ_AD, 0x00000007);	/*phy rstz  disable rstz*/
	SET_REG(CSIDPHY_base + LPCLK_CTRL,  0x00000001);	/*continuous clock mode*/
	SET_REG(CSIDPHY_base + CSI2_RESETN_AD, 0x00000001);	/*deassert csi2_reset*/
	SET_REG(CSIDPHY_base + PHY_IF_CFG_AD, 0x00003003);	/*config number of lanes*/
	SET_REG(CSIDPHY_base + CLKMGR_CFG_AD, 0x00000060);	/*config esc mode clock <20Mhz.*/
#ifdef P2
	unsigned int get_value;
	do 
	{
		get_value = readl(CSIDPHY_base + PHY_STATUS);
		vTaskDelay(1);
	} while((get_value & 0x40) != 0x40);
#endif
}
#else
static void mipi_tx_dphy_init(int ins_idx)
{
	__IO uint32_t CSIDPHY_base = CSLDS0_BASE + (INSTANCE_OFFSET * ins_idx);
#ifdef MIPI_SPEED_1500MHZ
	/*printf("\r\nMIPI TX DPHY INIT \r\n");*/
	SET_REG(CSIDPHY_base + PHY_RSTZ_AD, 0x00000000);  /*phy rstz in reset*/

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x0);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x1);

	vTaskDelay(1);

	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	vTaskDelay(1);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x44);	/*for normal operation*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x78);	/*1.5G*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0) ;

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x19);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x30);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x17);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x02);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x18);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x05);	/*m bit[4:0]*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x18);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x85);	/*m bit[7:5]*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x10);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xb8);	/*vcorang=7*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x11);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x07);	/*icpctrl=7*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x12);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xc8);	/*lpfctrl=8*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x22);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xd8);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x22);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x1b);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);
/* This is a macro to switch the mipi speed to 400Mhz.*/
#else
	SET_REG(CSIDPHY_base + PHY_RSTZ_AD, 0x00000000);	/*phy rstz in reset*/

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x0);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x1);

	vTaskDelay(1);

	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	vTaskDelay(1);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x44);	/*for normal operation*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x4A);	/*400Mhz*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0) ;

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x19);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x30);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x17);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x02); /* n*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x18);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x0b);	/*m bit[4:0]*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x18);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x81);	/*m bit[7:5]*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x10);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x90);	/*vcorang=2*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x11);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x05);	/*icpctrl=5*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x12);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xc8);	/*lpfctrl=8*/
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x22);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0xd8);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);

	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x1, 0x22);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x0, 0x0);
	phy_set_ctrl1(CSIDPHY_base + PHY_CTRL1_AD, 0x0, 0x1b);
	phy_set_ctrl0(CSIDPHY_base + PHY_CTRL0_AD, 0x1, 0x0);
#endif
	vTaskDelay(1);

	SET_REG(CSIDPHY_base + PHY_RSTZ_AD, 0x00000004);	/*phy rstz  enable clck*/
	SET_REG(CSIDPHY_base + PHY_RSTZ_AD, 0x00000007);	/*phy rstz  disable rstz*/
#if 0
	SET_REG(CSIDPHY_base + LPCLK_CTRL,  0x00000001);	/*Continuous clock mode*/
#endif
	SET_REG(CSIDPHY_base + CSI2_RESETN_AD, 0x00000001);	/* Deassert csi2_reset*/
	SET_REG(CSIDPHY_base + PHY_IF_CFG_AD, 0x00002003);	/*config number of lanes*/
	SET_REG(CSIDPHY_base + CLKMGR_CFG_AD, 0x00000060);	/*config esc mode clock <20Mhz.*/
	/*printf("No burst mode \r\n");*/
#ifdef P2
	unsigned int get_value;	
	do 
	{
		get_value = readl(CSIDPHY_base + PHY_STATUS);
		vTaskDelay(1);
	} while((get_value & 0x48) != 0x48);
#endif
}
#endif
/* Exported functions --------------------------------------------------------*/
void hal_axi2mipi_set_stream_property(axi2mipi_channel_t channel,
										axi2mipi_property_t *opt)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));
	assert_param(IS_AXI2MIPI_FIFO(opt->fifo_x));
	assert_param(IS_MIPI_DATA_TYPE(opt->img_type.rx_dt));
	assert_param(IS_MIPI_DATA_TYPE(opt->img_type.tx_dt));
	assert_param(IS_DIRECTLY_PASS(opt->img_type.ff_bypass));
	assert_param(IS_VC_FOR_FIFO(opt->vc_for_ff));
	assert_param(IS_MIPI_STATUS(opt->wrap.wrap_en));
	assert_param(IS_MIPI_STREAM_MODE(opt->stream_mode));
	assert_param(IS_MIPI_PACKED(opt->img_type.pack));

	uint32_t offset = 0;
	uint32_t tmp = 0;
	uint32_t fifo_option = opt->fifo_x;
	uint32_t current_fifo = 0;

	__IO uint32_t img_data_type;
	/* 1: data from master port; 0: data from slave */
	__IO uint32_t axi_is_ms_or_slv = 1;
	 /* 0 is packed (in snapshot) and 1 is unpacked (other mode) */
	__IO uint32_t packing_mode = 0;

	/*
	 * Configure Virtual channel for FIFO
	 */
	tmp = READ_AXI2MIPI_REG(channel, VC_SELECT_REG);
	if(opt->fifo_x & AXI2MIPI_FIFO_A)
	{
		tmp &= ~((uint32_t)((BIT1 | BIT0) << SHIFT_TO_BIT0));
		tmp |= (uint32_t)(opt->vc_for_ff << SHIFT_TO_BIT0);
	}
	if(opt->fifo_x & AXI2MIPI_FIFO_B)
	{
		tmp &= ~((uint32_t)((BIT1 | BIT0) << SHIFT_TO_BIT2));
		tmp |= (uint32_t)(opt->vc_for_ff << SHIFT_TO_BIT2);
	}
	if(opt->fifo_x & AXI2MIPI_FIFO_C)
	{
		tmp &= ~((uint32_t)((BIT1 | BIT0) << SHIFT_TO_BIT4));
		tmp |= (uint32_t)(opt->vc_for_ff << SHIFT_TO_BIT4);
	}
	if(opt->fifo_x & AXI2MIPI_FIFO_D)
	{
		tmp &= ~((uint32_t)((BIT1 | BIT0) << SHIFT_TO_BIT6));
		tmp |= (uint32_t)(opt->vc_for_ff << SHIFT_TO_BIT6);
	}
	WRITE_AXI2MIPI_REG(channel, VC_SELECT_REG, tmp);

	/*
	 * Calculate bits/frame
	 */
	uint32_t hde_width = 0, hs_width = 0;
	uint32_t bits_per_pixel = 0;
	uint32_t hsync_val = 0;

	switch(opt->img_type.rx_dt)
	{
		case AXI2MIPI_RAW10:
		{
			bits_per_pixel = TEN_BITS_PER_PIXEL;
			break;
		}
		case AXI2MIPI_RAW12:
		{
			bits_per_pixel = TWELVE_BITS_PER_PIXEL;
			break;
		}
		case AXI2MIPI_RAW14:
		{
			bits_per_pixel = FOURTEEN_BITS_PER_PIXEL;
			break;
		}
		case AXI2MIPI_RAW8:
		default:
		{
			bits_per_pixel = EIGHT_BITS_PER_PIXEL;
			break;
		}
	}

	hde_width = (uint32_t)(opt->img_size.width * bits_per_pixel) / 8;
	hs_width = HS_WIDTH_VAL;
	hsync_val = (uint32_t)((hs_width << SHIFT_TO_BIT16) + (hde_width << SHIFT_TO_BIT0));
	/*
	 * Scan all FIFO selected
	 */
	fifo_option = opt->fifo_x;
	current_fifo = 0;
	while(fifo_option)
	{
		if(fifo_option & 1)
		{
			/*
			 * Configure source address
			 */
			offset = current_fifo * (DMA_SRC_B_REG - DMA_SRC_A_REG);
			WRITE_AXI2MIPI_REG(channel, DMA_SRC_A_REG + offset, opt->src_addr);

			/*
			 * Configure Image Control Register
			 */
			offset = current_fifo * (IMG_CTRL_B_REG - IMG_CTRL_A_REG);

			if(opt->stream_mode == AXI2MIPI_SNAPSHOT_MODE)
			{
				axi_is_ms_or_slv = IS_MASTER_PORT;
			}
			else
			{
				axi_is_ms_or_slv = IS_SLAVE_PORT;
			}

			packing_mode = opt->img_type.pack;
			img_data_type = opt->img_type.tx_dt;
			tmp = (img_data_type << SHIFT_TO_BIT24)
					+ (packing_mode << SHIFT_TO_BIT20)
					+ (opt->img_type.ff_bypass << SHIFT_TO_BIT17)
					+ (axi_is_ms_or_slv << SHIFT_TO_BIT16);
			WRITE_AXI2MIPI_REG(channel, IMG_CTRL_A_REG + offset, tmp);

			/*
			 * Configure width and height pixel
			 */
			offset = current_fifo * (DMA_TRANS_B_REG - DMA_TRANS_A_REG);
			tmp = convert_in_halfword(opt->img_size.width,
										opt->img_type.rx_dt,
										opt->stream_mode);
			if(opt->wrap.wrap_en == DISABLE)
			{
				tmp |= (uint32_t)((opt->img_size.height << SHIFT_TO_BIT16) & HALF_DWORD_MASK);
			}
			else
			{
				tmp |= (uint32_t)((opt->wrap.wrap_num << SHIFT_TO_BIT16) & HALF_DWORD_MASK);
			}
			WRITE_AXI2MIPI_REG(channel, DMA_TRANS_A_REG + offset, tmp);

			/*
			 * Configure Horizontal control
			 */
			offset = current_fifo * (HOR_CTRL_B_REG - HOR_CTRL_A_REG);
			WRITE_AXI2MIPI_REG(channel, HOR_CTRL_A_REG + offset, hsync_val);
		}
		fifo_option >>= 1;
		current_fifo++;
	}
}

void hal_axi2mipi_hsync_cfg(axi2mipi_channel_t channel,
								axi2mipi_hsync_t *hsync)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));
	assert_param(IS_AXI2MIPI_FIFO(hsync->fifo_x));

	uint32_t hor_ctr_offset = 0, hor_blank_offset = 0;
	uint32_t tmp = 0;

	uint32_t fifo_option = hsync->fifo_x;
	uint32_t current_fifo = 0;

	/* Scan all virtual channel selected */
	while(fifo_option)
	{
		if(fifo_option & 1)
		{
			hor_ctr_offset = current_fifo * (HOR_CTRL_B_REG - HOR_CTRL_A_REG);
			hor_blank_offset = current_fifo * (HOR_BLANK_B_REG - HOR_BLANK_A_REG);

			/* Configure Horizontal control */
			tmp = (uint32_t)((hsync->hs_width << SHIFT_TO_BIT16) + (hsync->hde_width << SHIFT_TO_BIT0));
			WRITE_AXI2MIPI_REG(channel, HOR_CTRL_A_REG + hor_ctr_offset, tmp);

			tmp = (uint32_t)((hsync->hbp_width << SHIFT_TO_BIT16) + (hsync->hfp_width << SHIFT_TO_BIT0));
			WRITE_AXI2MIPI_REG(channel, HOR_BLANK_A_REG + hor_blank_offset, tmp);
		}

		fifo_option >>= 1;
		current_fifo++;
	}
}

void hal_axi2mipi_init(axi2mipi_channel_t channel)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));
	if(!axi2mipi_hw[channel].is_init)
	{
		mipi_tx_dphy_init(channel);
		axi2mipi_hw[channel].is_init = 1;
	}

	/* Reset configurations */
	bridge_reg[channel] = 0;

	/* Disable all interrupt */
	WRITE_AXI2MIPI_REG(channel, INTR_MSK_REG, ISR_DISABLE_ALL);
}


void hal_axi2mipi_programming_valid(axi2mipi_channel_t channel,
									axi2mipi_ff_t fifo_x,
									flag_status_t valid_en)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));
	assert_param(IS_AXI2MIPI_FIFO(fifo_x));
	assert_param(IS_FRAME_CONT(valid_en));

	uint32_t offset = 0, tmp = 0;
	uint32_t fifo_option = fifo_x;
	uint32_t current_fifo = 0;

	/* Scan all FIFOs selected */
	tmp = (uint32_t)(valid_en);
	while(fifo_option)
	{
		if(fifo_option & 1)
		{
			offset = current_fifo * (FRAME_CTRL_B_REG - FRAME_CTRL_A_REG);
			WRITE_AXI2MIPI_REG(channel, FRAME_CTRL_A_REG + offset, tmp);
		}
		fifo_option >>= 1;
		current_fifo++;
	}
}

void hal_axi2mipi_start(axi2mipi_channel_t channel, axi2mipi_ff_t fifo_x)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));
	assert_param(IS_AXI2MIPI_FIFO(fifo_x));

	uint8_t fifo_opt = fifo_x;
	uint8_t fifo_idx = 0;
	while(fifo_opt)
	{
		if(fifo_opt & 1)
		{
			bridge_reg[channel] |= ((uint32_t)(AXI2MIPI_FIFO_A << (SHIFT_TO_BIT4 + fifo_idx)));
		}
		fifo_idx++;
		fifo_opt >>= 1;
	}
	WRITE_AXI2MIPI_REG(channel, BRIDGE_CTRL_REG, bridge_reg[channel]);
}

void hal_axi2mipi_stop(axi2mipi_channel_t channel, axi2mipi_ff_t fifo_x)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));
	assert_param(IS_AXI2MIPI_FIFO(fifo_x));

	/* Enable by clear bit */
	uint8_t fifo_opt = fifo_x;
	uint8_t fifo_idx = 0;
	while(fifo_opt)
	{
		if(fifo_opt & 1)
		{
			bridge_reg[channel] &= ~((uint32_t)(AXI2MIPI_FIFO_A << (SHIFT_TO_BIT4 + fifo_idx)));
		}
		fifo_idx++;
		fifo_opt >>= 1;
	}
	WRITE_AXI2MIPI_REG(channel, BRIDGE_CTRL_REG, bridge_reg[channel]);
}


void hal_axi2mipi_set_frame_mode(axi2mipi_channel_t channel,
								axi2mipi_ff_t fifo_x,
								axi2mipi_cont_frame_t cont_mode)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));
	assert_param(IS_AXI2MIPI_FIFO(fifo_x));
	assert_param(IS_FRAME_CONT(cont_mode));

	uint32_t fifo_option = fifo_x;
	uint32_t current_fifo = 0;

	/* Scan all FIFOs selected */
	while(fifo_option)
	{
		if(fifo_option & 1)
		{
			bridge_reg[channel] &= ~((uint32_t)(BIT8 << current_fifo));
			bridge_reg[channel] |= (uint32_t)(((cont_mode) << SHIFT_TO_BIT8) << current_fifo);
		}
		fifo_option >>= 1;
		current_fifo++;
	}
	WRITE_AXI2MIPI_REG(channel, BRIDGE_CTRL_REG, bridge_reg[channel]);
}

void hal_axi2mipi_irq_enable(axi2mipi_channel_t channel,
							axi2mipi_callback irq_callback,
							void *user_data)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));
	assert_param(irq_callback != NULL);

	switch(channel)
	{
		case AXI2MIPI_BRIDGE_0:
		{
			axi2mipi_hw[channel].callback = irq_callback;
			axi2mipi_hw[channel].user_data = user_data;
			vic_register_irq(MIPI0_TX_IRQn, hal_axi2mipi_0_isr);
			break;
		}
		case AXI2MIPI_BRIDGE_1:
		{
			axi2mipi_hw[channel].callback = irq_callback;
			axi2mipi_hw[channel].user_data = user_data;
			vic_register_irq(MIPI1_TX_IRQn, hal_axi2mipi_1_isr);
			break;
		}
		default:
		{
			break;
		}
	}
	/* TODO: Configure interrupt for CSI2 device */
	vic_register_irq(CSID0_IRQn, csi2_device_isr);
	vic_enable_irq(CSID0_IRQn);
	/* Write all interrupt mask */
	writel(CSLDS0_BASE + 0x40, 0xFFFFFFFF);
	writel(CSLDS0_BASE + 0x48, 0xFFFFFFFF);
	writel(CSLDS0_BASE + 0x50, 0xFFFFFFFF);
}

void hal_axi2mipi_irq_disable(axi2mipi_channel_t channel)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));

	if(channel == AXI2MIPI_BRIDGE_0)
	{
		vic_unregister_irq(MIPI0_TX_IRQn);
		axi2mipi_hw[channel].callback = NULL;
		axi2mipi_hw[channel].user_data = NULL;
	}
	else if(channel == AXI2MIPI_BRIDGE_1)
	{
		vic_unregister_irq(MIPI1_TX_IRQn);
		axi2mipi_hw[channel].callback = NULL;
		axi2mipi_hw[channel].user_data = NULL;
	}
}

void hal_axi2mipi_irq_mask(axi2mipi_channel_t channel,
							axi2mipi_isr_flag_t irq_sel,
							uint8_t enable)
{
	/* Check parameters */
	assert_param(IS_MIPI_CHANNEL(channel));
	assert_param(IS_MIPI_STATUS(enable));

	/* Interrupt enable by clear bit */
	uint32_t interrupt_reg_val = READ_AXI2MIPI_REG(channel, INTR_MSK_REG);

	if(enable)
	{
		/* Enable interrupt by clear bit to 0 */
		irq_sel = (ISR_DISABLE_ALL) & (~(uint32_t)(irq_sel));
		interrupt_reg_val &= irq_sel;
	}
	else
	{
		/* Disable interrupt by set bit to 1 */
		interrupt_reg_val |= irq_sel;
	}

	WRITE_AXI2MIPI_REG(channel, INTR_MSK_REG, interrupt_reg_val);
}
unsigned int hal_axi2mipi_get_width_byte(uint16_t width, axi2mipi_data_type_t data_type,
								axi2mipi_stream_mode_t mode)
{
	return convert_in_halfword(width, data_type, mode);
}
void setup_tx_all(uint16_t width, uint16_t height)
{

	uint16_t hw = convert_in_halfword(4160, AXI2MIPI_RAW10, AXI2MIPI_PREVIEW_MODE);
	mipi_tx_dphy_init(0);
	WRITE_AXI2MIPI_REG(0, 0x0, 0x1);
	WRITE_AXI2MIPI_REG(0, 0x4, TXS0_BASE);
	WRITE_AXI2MIPI_REG(0, 0x100, 0xA0000 | (4160*10/8));
	WRITE_AXI2MIPI_REG(0, 0x8, (3120 << 16) | hw);
	WRITE_AXI2MIPI_REG(0, 0x38, 0x2B100000);
	WRITE_AXI2MIPI_REG(0, 0x34, 0);
	WRITE_AXI2MIPI_REG(0, 0x80, 0x100);
	WRITE_AXI2MIPI_REG(0, 0x48, 0x1);
	WRITE_AXI2MIPI_REG(0, 0x0, 0x00000110);

}
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
