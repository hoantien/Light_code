/*
 * drv_piezo_hwinit.h
 *
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "drv_piezo.h"
#include "drv_piezo_pwm.h"
//#include "hal_i2c_ex.h"


/* FreeRTOS includes. */
#include "FreeRTOS.h"
//#include "task.h"
#include "log.h"

/* Private typedef -----------------------------------------------------------*/

static void SPI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	/* Peripheral Clock Enable -------------------------------------------------*/
	/* Enable the SPI clock */
	CPLD_CLK_INIT(CPLD_CLK, ENABLE);

	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(CPLD_SCK_GPIO_CLK | CPLD_MISO_GPIO_CLK | CPLD_MOSI_GPIO_CLK | CPLD_SS_GPIO_CLK, ENABLE);


	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = CPLD_SCK_PIN;
	GPIO_Init(CPLD_SCK_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  CPLD_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(CPLD_MISO_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  CPLD_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(CPLD_MOSI_GPIO_PORT, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = CPLD_SS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CPLD_SS_GPIO_PORT, &GPIO_InitStructure);

	/* Connect SPI pins to AF */
	GPIO_PinAFConfig(CPLD_SCK_GPIO_PORT, CPLD_SCK_SOURCE, CPLD_SCK_AF);
	GPIO_PinAFConfig(CPLD_MISO_GPIO_PORT, CPLD_MISO_SOURCE, CPLD_MISO_AF);
	GPIO_PinAFConfig(CPLD_MOSI_GPIO_PORT, CPLD_MOSI_SOURCE, CPLD_MOSI_AF);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(CPLD, &SPI_InitStructure);

	/* Enable SPI_MASTER */
	SPI_Cmd(CPLD, ENABLE);
}

static void GPIO_Config(PiezoChannel *Channels)
{
	int i;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(AF_FAILN0_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN0_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN1_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN1_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN2_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN2_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN3_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN3_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN4_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN4_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN5_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN5_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN6_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN6_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN7_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN7_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN8_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN8_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN9_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN9_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(AF_FAILN10_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_AF_EN10_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(CPLD_CONFIG_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	for(i = 0; i < NUM_PIEZO_CHANNELS; i++)
	{
		GPIO_InitStructure.GPIO_Pin = Channels[i].Pin;
		GPIO_Init(Channels[i].Port, &GPIO_InitStructure);
	}

	GPIO_InitStructure.GPIO_Pin = CPLD_CONFIG_PIN;
	GPIO_Init(CPLD_CONFIG_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = AF_FAILN0_GPIO_PIN;
	GPIO_Init(AF_FAILN0_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN1_GPIO_PIN;
	GPIO_Init(AF_FAILN1_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN2_GPIO_PIN;
	GPIO_Init(AF_FAILN2_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN3_GPIO_PIN;
	GPIO_Init(AF_FAILN3_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN4_GPIO_PIN;
	GPIO_Init(AF_FAILN4_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN5_GPIO_PIN;
	GPIO_Init(AF_FAILN5_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN6_GPIO_PIN;
	GPIO_Init(AF_FAILN6_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN7_GPIO_PIN;
	GPIO_Init(AF_FAILN7_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN8_GPIO_PIN;
	GPIO_Init(AF_FAILN8_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN9_GPIO_PIN;
	GPIO_Init(AF_FAILN9_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = AF_FAILN10_GPIO_PIN;
	GPIO_Init(AF_FAILN10_GPIO_PORT, &GPIO_InitStructure);

	SS_DIS;
	GPIO_ResetBits(CPLD_CONFIG_GPIO_PORT , CPLD_CONFIG_PIN );
}

void InitPiezoHw(PiezoChannel *Channels)
{
	SPI_Config();
	GPIO_Config(Channels);
	InitPwm();
}
