/********************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "log.h"
#include "hal_comport.h"
#include "hal_i2c_ex.h"
#include "drv_cci.h"
#include "mems.h"
#include "drv_spi.h"
#include "fpga.h"
#include "hal_flash_ex.h"
#include "drv_piezo_pwm.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
interrupt_status intr_sta =
{
	.current_status		= NO_INTERRUPT,
	.previous_status	= NO_INTERRUPT
};
/* Private function prototypes -----------------------------------------------*/
STATIC void PlatformDriverInit(void);
STATIC void PlatformPeriphInit(void);
STATIC void PlatformLibraryInit(void);
STATIC void STM_IRQ_Init(void);
/* Exported functions --------------------------------------------------------*/
void PlatformInit(void)
{
	/* Enable Watchdog */
	/* Enable floating point*/
	/* System Initialization. */
	// SystemInit();	// Already called in startup_stm32f429xx.s

	/* Now SYSCLK is at 160MHz, set AHB divider to 1, APB1 to 4 and APB2 to 4 */
	/*
	 *  The frequency of the AHB domain is 160 MHz.
	 *  The frequency of the APBx domain is 42 MHz.
	 */
	SystemCoreClockUpdate();

	/* Select PLL as SYSCLK source clock */
	/* Setup the drivers */
	PlatformDriverInit();
	/* Setup the peripherals */
	PlatformPeriphInit();
	/* Setup the libraries */
	PlatformLibraryInit();
}

/* Private functions ---------------------------------------------------------*/
STATIC void PlatformDriverInit(void)
{
	// Init memory
	CCB_MEM_Init();
	//Init ExFlash
	SPIExFLASH_Init();
	// Initialization USART for Logging
	HAL_COM_Init(COM1);
	HAL_COM_Init(COM2);
	vLogInit();
}

STATIC int CPLD_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(CPLD_INITn_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(CPLD_RESET_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(CPLD_PROGRAMn_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(CPLD_DONE_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStructure.GPIO_Pin = CPLD_RESET_PIN;
	GPIO_Init(CPLD_RESET_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CPLD_PROGRAMn_PIN;
	GPIO_Init(CPLD_PROGRAMn_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CPLD_CONFIG_PIN;
	GPIO_Init(CPLD_CONFIG_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = CPLD_DONE_PIN;
	GPIO_Init(CPLD_DONE_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = CPLD_INITn_PIN;
	GPIO_Init(CPLD_INITn_GPIO_PORT, &GPIO_InitStructure);

	/* Reset CPLD */
	GPIO_ResetBits(CPLD_RESET_GPIO_PORT, CPLD_RESET_PIN);
	//vTaskDelay(1); 			/* Delay 1ms for reset effect */
	GPIO_SetBits(CPLD_RESET_GPIO_PORT, CPLD_RESET_PIN);

	/* Configure for CPLD at Programming mode */
	//log_printf(" - Set CPLD_CONFIG_PIN to high\n\r");
	GPIO_SetBits(CPLD_CONFIG_GPIO_PORT, CPLD_CONFIG_PIN);
	GPIO_SetBits(CPLD_PROGRAMn_GPIO_PORT, CPLD_PROGRAMn_PIN);
	return 1;
}

STATIC int CPLD_SPI_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Peripheral Clock Enable -------------------------------------------------*/
	/* Enable the SPI clock */
	CPLD_CLK_INIT(CPLD_CLK, ENABLE);

	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(CPLD_SCK_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(CPLD_MISO_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(CPLD_MOSI_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(CPLD_SS_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = CPLD_SCK_PIN;
	GPIO_Init(CPLD_SCK_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  CPLD_MISO_PIN;
	GPIO_Init(CPLD_MISO_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  CPLD_MOSI_PIN;
	GPIO_Init(CPLD_MOSI_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = CPLD_SS_PIN;
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
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(CPLD, &SPI_InitStructure);

	/* Enable SPI_MASTER */
	SPI_Cmd(CPLD, ENABLE);

	return 1;
}

STATIC void STM_IRQ_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(STM_IRQ_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStructure.GPIO_Pin = STM_IRQ_GPIO_PIN;
	GPIO_Init(STM_IRQ_GPIO_PORT, &GPIO_InitStructure);


}

STATIC void PlatformPeriphInit(void)
{
	CCB_CCI0_Init();
	HAL_I2CEx_Init();
	SNAP_SPISLAVE_Init();
	fpga_init();
	CPLD_GPIO_Init();
	CPLD_SPI_Init();
	STM_IRQ_Init();
}

void Trigger_STM_IRQ(uint32_t event)
{
    vPortEnterCritical();

	/* Keep previous interrupt event */
	intr_sta.previous_status = intr_sta.current_status;
	/* Update current interrupt event */
	intr_sta.current_status = event;

	/* Pull down STM_IRQ pin indicates interrupt signal */
	GPIO_ResetBits(STM_IRQ_GPIO_PORT, STM_IRQ_GPIO_PIN);
	/* Keep low level for 500 usec */
    DelayUSec(500);
	/* Pull up STM_IRQ for restore origin status */
	GPIO_SetBits(STM_IRQ_GPIO_PORT, STM_IRQ_GPIO_PIN);

    vPortExitCritical();
}

STATIC void PlatformLibraryInit(void)
{
}


/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE****/
