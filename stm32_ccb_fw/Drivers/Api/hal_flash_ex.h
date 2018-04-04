/**
  ******************************************************************************
  * \file    hal_flash_ex.h
  * \author  Infonam Embedded Team
  * \version V1.0.0
  * \date    May 4, 2015
  * \brief   Header file of SPI Flash related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL__EXFLASH__H
#define __HAL__EXFLASH__H

/* Includes ------------------------------------------------------------------*/
#include "types.h"

/* Exported functions ------------------------------------------------------- */

/*
 * TODO: Add function description
 */
void SPIExFLASH_Init(void);

/*
 * TODO: Add function description
 */
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);

/*
 * TODO: Add function description
 */
void SPI_FLASH_SectorErase(uint32_t SectorAddr);
/*
 * TODO: Add function description
 */
void SPI_FLASH_SubSectorErase(uint32_t SectorAddr);

/*
 * TODO: Add function description
 */
void SPI_FLASH_WriteEnable(void);

/*
 * TODO: Add function description
 */
void SPI_FLASH_WaitForWriteEnd(void);

/*
 * TODO: Add function description
 */
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

/*
 * TODO: Add function description
 */
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

/*
 * TODO: Add function description
 */
uint32_t SPI_FLASH_ReadID(void);
/*
 * TODO: Add function description
 */
void SPI_FLASH_BulkErase(void);

#endif
