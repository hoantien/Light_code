/**
  ******************************************************************************
  * \file    drv_spi.h
  * \author  Infonam Embedded Team
  * \version V1.1.0
  * \date    05-Mar-2015
  * \brief   Header file of SPI driver related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI__H
#define __SPI__H

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "types.h"

/* Exported define -----------------------------------------------------------*/
#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))

/*
 * TODO: Add structure description
 */
typedef struct
{
	GPIO_TypeDef *PORT;
	const UInt16 Pin;
	const UInt32 clock;
	const UInt8 Source;
	const UInt8 AF;
}GPIO_Dev;

/*
 * TODO: Add structure description
 */
typedef struct
{
	GPIO_Dev SCK;
	GPIO_Dev MOSI;
	GPIO_Dev MISO;
	GPIO_Dev NSS;
}SPI_Dev;

/*
 * TODO: Add structure description
 */
typedef struct {
	uint32_t TX_Channel;
	DMA_Stream_TypeDef* TX_Stream;
	uint32_t RX_Channel;
	DMA_Stream_TypeDef* RX_Stream;
}SPI_DMA_Dev;

#define	SLAVE_ADDR_TEMP1 		0x48
#define	SLAVE_ADDR_TEMP2 		0x49
#define SLAVE_ADDR_TEMP3 		0x4A
#define	SLAVE_ADDR_TEMP4 		0x4B
#define TEMPERATURE_MAX_NUM	 	4
/* Exported functions ------------------------------------------------------- */

/*
 * TODO: Add function description
 */
void SNAP_SPISLAVE_Init(void);
UInt16 Get_TempBoad(UInt8 SlaveAddr);
void Temp_Init(void);
#endif
