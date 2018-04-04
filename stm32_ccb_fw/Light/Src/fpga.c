/********************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "platform.h"
#include "log.h"
#include "fpga.h"
#include "drv_spi.h"
#include "cam_ctrl.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FPGA_SPI_SEND_RETRY		5
/* Private macro -------------------------------------------------------------*/
#define FPGA_SPI_CS_ENABLE(p)   GPIO_ResetBits(p->NSS.PORT , p->NSS.Pin);
#define FPGA_SPI_CS_DISABLE(p)  GPIO_SetBits(p->NSS.PORT , p->NSS.Pin);
/* Private variables ---------------------------------------------------------*/
static SPI_Dev FPGA_SPI_Dev = {
	{SPI_FPGA_SCK_PORT, SPI_FPGA_SCK_PIN, SPI_FPGA_SCK_CLK, SPI_FPGA_SCK_SOURCE, SPI_FPGA_SCK_AF},
	{SPI_FPGA_MOSI_PORT,SPI_FPGA_MOSI_PIN,SPI_FPGA_MOSI_CLK,SPI_FPGA_MOSI_SOURCE,SPI_FPGA_MOSI_AF},
	{SPI_FPGA_MISO_PORT,SPI_FPGA_MISO_PIN,SPI_FPGA_MISO_CLK,SPI_FPGA_MISO_SOURCE,SPI_FPGA_MISO_AF},
	{SPI_FPGA_NSS_PORT, SPI_FPGA_NSS_PIN, SPI_FPGA_NSS_CLK, SPI_FPGA_NSS_SOURCE, SPI_FPGA_NSS_AF}
};

__IO fpga_irq_status_t fpga_irq_trigger = FPGA_IRQ_WAITING;
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

static void fpga_spi_init(void)
{
	SPI_Dev * Setting = &FPGA_SPI_Dev;
	GPIO_InitTypeDef GPIO_InitStructure = {
			.GPIO_Mode = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd = GPIO_PuPd_NOPULL,
			.GPIO_Speed = GPIO_Speed_100MHz
	};
	SPI_InitTypeDef SPI_InitStructure;

	/* Enable Clock GPIO*/
	RCC->AHB1ENR |= Setting->SCK.clock|Setting->MISO.clock|Setting->MOSI.clock|Setting->NSS.clock;
	/* Enable Clock SPIx*/
	RCC->APB2ENR |= SPI_FPGA_CLK;

	/* AF Config*/
	GPIO_PinAFConfig(Setting->MISO.PORT,Setting->MISO.Source,Setting->MISO.AF);
	GPIO_PinAFConfig(Setting->MOSI.PORT,Setting->MOSI.Source,Setting->MOSI.AF);
	GPIO_PinAFConfig(Setting->SCK.PORT,Setting->SCK.Source,Setting->SCK.AF);

	/* GPIO Config*/
	/* SCK */
	GPIO_InitStructure.GPIO_Pin = Setting->SCK.Pin;
	GPIO_Init(Setting->SCK.PORT, &GPIO_InitStructure);
	/*MISO*/
	GPIO_InitStructure.GPIO_Pin = Setting->MISO.Pin;
	GPIO_Init(Setting->MISO.PORT,&GPIO_InitStructure);
	/* MOSI*/
	GPIO_InitStructure.GPIO_Pin = Setting->MOSI.Pin;
	GPIO_Init(Setting->MOSI.PORT,&GPIO_InitStructure);
	/* NSS Conf */

	GPIO_InitStructure.GPIO_Pin = Setting->NSS.Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Setting->NSS.PORT,&GPIO_InitStructure);

	/* SPI Configure */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_FPGA_BaudRatePreScaler;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_FPGA,&SPI_InitStructure);
	SPI_Cmd(SPI_FPGA,ENABLE);
}

static void fpga_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(FPGA_1P0V_PG_CLK, ENABLE);
	GPIO_InitStruct.GPIO_Pin = 	FPGA_1P0V_PG_PIN |
								FPGA_1P5V_PG_PIN |
								FPGA_1P8V_PG_PIN |
								CAM_2P8V_PG_PIN  |
								CAM_1P2V_PG_PIN  |
								CAM_3P3V_PG_PIN;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(FPGA_1P0V_PG_PORT, &GPIO_InitStruct);

	/* Set default GPIO	settings */
	RCC_AHB1PeriphClockCmd(FPGA_IRQ_GPIO_CLK, ENABLE);

	GPIO_InitStruct.GPIO_Pin = FPGA_IRQ_PIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(FPGA_IRQ_GPIO_PORT, &GPIO_InitStruct);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource15);
}

static void fpga_spi_transfer(UInt16 *tx, UInt16 *rx, UInt16 len)
{
  SPI_Dev * ptr = &FPGA_SPI_Dev;
  UInt16 i = 0;

  FPGA_SPI_CS_ENABLE(ptr);

  for (i = 0; i < len; i++)
  {
	  while (!(SPI_FPGA->SR & SPI_SR_TXE));
	  SPI_FPGA->DR = *(tx + i);
	  while (!(SPI_FPGA->SR & SPI_SR_RXNE));
	  *(rx + i) = SPI_FPGA->DR;
  }

  FPGA_SPI_CS_DISABLE(ptr);
}

void fpga_init(void)
{
	fpga_gpio_init();
	fpga_spi_init();
}

fpga_pg_status_t fpga_check_cam_pg(void)
{
	ENTER_FUNC;
	UInt8 PGStatus = 1;
	PGStatus &= GPIO_ReadInputDataBit(CAM_1P2V_PG_PORT, CAM_1P2V_PG_PIN);
	if (PGStatus == 0) log_printf("CAM_1P2V_PG_PIN failed !\n\r");
	PGStatus &= GPIO_ReadInputDataBit(CAM_2P8V_PG_PORT, CAM_2P8V_PG_PIN);
	if (PGStatus == 0) log_printf("CAM_2P8V_PG_PIN failed !\n\r");
	PGStatus &= GPIO_ReadInputDataBit(CAM_3P3V_PG_PORT, CAM_3P3V_PG_PIN);
	if (PGStatus == 0) log_printf("CAM_3P3V_PG_PIN failed !\n\r");
	EXIT_FUNC;
	if (PGStatus == 0)
		return FPGA_PWR_FAIL;
	else
		return FPGA_PWR_GOOD;
}

fpga_pg_status_t fpga_check_pg(void)
{
	ENTER_FUNC;
	UInt8 PGStatus = 1;
	PGStatus &= GPIO_ReadInputDataBit(FPGA_1P0V_PG_PORT, FPGA_1P0V_PG_PIN);
	if (PGStatus == 0) log_printf("FPGA_1P0V_PG_PIN failed !\n\r");
	PGStatus &= GPIO_ReadInputDataBit(FPGA_1P8V_PG_PORT, FPGA_1P8V_PG_PIN);
	if (PGStatus == 0) log_printf("FPGA_1P8V_PG_PIN failed !\n\r");
	PGStatus &= GPIO_ReadInputDataBit(FPGA_1P5V_PG_PORT, FPGA_1P5V_PG_PIN);
	if (PGStatus == 0) log_printf("FPGA_1P5V_PG_PIN failed !\n\r");
	EXIT_FUNC;
	if (PGStatus == 0)
		return FPGA_PWR_FAIL;
	else
		return FPGA_PWR_GOOD;
}

fpga_cmd_error_t fpga_send_command(UInt16 *tx, UInt16 *rx, UInt16 len, UInt32 timeout)
{
	//ENTER_FUNC;
	UInt8 retry = FPGA_SPI_SEND_RETRY;
	UInt32 tout = timeout;
	fpga_cmd_error_t ret = FPGA_CMD_WAITING;
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	fpga_spi_transfer(tx, rx, len);	// Send FTM (Quick check) command

#ifndef NO_DEBUG
// Using a #ifndef because log_debug would put newlines between transaction bytes.
	log_fpga("SPI send cmd to FPGA:");
	for (UInt8 i = 0; i < len; i++)
		log_fpga("[0x%04x]", tx[i]);
	log_fpga("\n\r");
#endif
    
	if(timeout != 0 && (tx[0] & (1<<7)))
	{
		do
		{
		        //vTaskDelay(1);
			if (fpga_irq_trigger == FPGA_IRQ_TRIGGERED)
			{
				fpga_irq_trigger = FPGA_IRQ_WAITING;
				ret = FPGA_CMD_SUCCESS;
				break;
			}
			tout--;
			if (tout == 0)
			{
				tout = timeout;
				retry--;
				log_error("FPGA not responding, retry: %d\n\r", FPGA_SPI_SEND_RETRY - retry);
				fpga_spi_transfer(tx, rx, len);	// Send FTM (Quick check) command
			}
		} while (retry > 0);
		if (retry <= 0) ret = FPGA_CMD_TOUT;
	}
	else
	{
		ret = FPGA_CMD_SUCCESS;
	}
	//EXIT_FUNC;
	return ret;
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		fpga_irq_trigger = FPGA_IRQ_TRIGGERED;
//		log_debug("FPGA_IRQ_PIN falling triggered");
		EXTI_InitStructure.EXTI_Line = EXTI_Line15;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;
		EXTI_Init(&EXTI_InitStructure);
		/* Clear the EXTI line 15 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line15);
	}
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE****/
