/**
  ******************************************************************************
  * @file    drv_usart.c
  * @author  Infonam Embedded Team
  * @version V1.1.0
  * @date    05-Mar-2015
  * @brief   This file provides set of firmware functions to USART debug
  * 		 on CCB Light Board.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "drv_usart.h"
#include "hal_comport.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_TypeDef* COM_USART[COMn] = {COM1_USART, COM2_USART};
const uint32_t COM_USART_CLK[COMn] = {COM1_CLK, COM2_CLK};

GPIO_TypeDef* COM_TX_PORT[COMn] = {COM1_TX_GPIO_PORT, COM2_TX_GPIO_PORT};
GPIO_TypeDef* COM_RX_PORT[COMn] = {COM1_RX_GPIO_PORT, COM2_RX_GPIO_PORT};
const uint32_t COM_TX_PORT_CLK[COMn] = {COM1_TX_GPIO_CLK, COM2_TX_GPIO_CLK};
const uint32_t COM_RX_PORT_CLK[COMn] = {COM1_RX_GPIO_CLK, COM2_RX_GPIO_CLK};
const uint16_t COM_TX_PIN[COMn] = {COM1_TX_PIN, COM2_TX_PIN};
const uint16_t COM_RX_PIN[COMn] = {COM1_RX_PIN, COM2_RX_PIN};
const uint16_t COM_TX_PIN_SOURCE[COMn] = {COM1_TX_SOURCE, COM2_TX_SOURCE};
const uint16_t COM_RX_PIN_SOURCE[COMn] = {COM1_RX_SOURCE, COM2_RX_SOURCE};
const uint16_t COM_TX_AF[COMn] = {COM1_TX_AF, COM2_TX_AF};
const uint16_t COM_RX_AF[COMn] = {COM1_RX_AF, COM2_RX_AF};

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void HAL_COM_Init(COM_Port_t COMx)
{
    USART_ClockInitTypeDef USART_ClockInitStruct;
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable needed clocks for uart.
    RCC_APB1PeriphClockCmd(COM_USART_CLK[COMx], ENABLE);
    RCC_AHB1PeriphClockCmd(COM_TX_PORT_CLK[COMx], ENABLE);
    RCC_AHB1PeriphClockCmd(COM_RX_PORT_CLK[COMx], ENABLE);

    // Make sure you use 'GPIO_PinSource2' and NOT 'GPIO_Pin_2'.  Using the
    // latter will not work!
    GPIO_PinAFConfig(COM_TX_PORT[COMx], COM_TX_PIN_SOURCE[COMx], COM_TX_AF[COMx]);
    GPIO_PinAFConfig(COM_RX_PORT[COMx], COM_RX_PIN_SOURCE[COMx], COM_RX_AF[COMx]);

    // Setup Tx / Rx pins.
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COMx];			// Tx Pin
    GPIO_Init(COM_TX_PORT[COMx], &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COMx];			// Rx Pin
    GPIO_Init(COM_RX_PORT[COMx], &GPIO_InitStructure);

    // Make sure syncro clock is turned off.
    USART_ClockStructInit(&USART_ClockInitStruct);
    if (COMx == COM2)
    	USART_ClockInit(COM_USART[COMx], &USART_ClockInitStruct);

    // Setup transmit complete irq.
    USART_ITConfig(COM_USART[COMx], USART_IT_TC, ENABLE);

    // Use defaults (except baud rate).
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_Init(COM_USART[COMx], &USART_InitStructure);
    USART_Cmd(COM_USART[COMx], ENABLE);
}

FlagStatus HAL_COM_GetFlagStatus(COM_Port_t COMx, UInt16 flag)
{
	return USART_GetFlagStatus(COM_USART[COMx], flag);
}

void HAL_COM_SendData(COM_Port_t COMx, UInt8 data)
{
	USART_SendData(COM_USART[COMx], (uint16_t)data);
}
/*********** Portions COPYRIGHT 2015 Infonam. Co., Ltd.*****END OF FILE****/
