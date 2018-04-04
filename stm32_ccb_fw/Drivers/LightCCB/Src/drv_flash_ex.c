/**
 *@file eeprom.c
 */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "hal_flash_ex.h"
#include "log.h"
#include "string.h"
#include "drv_spi.h"
/* Private variables -----------------------------------------------------------*/
static SPI_Dev SPIEE_Dev = {
		{SPIEE_SCK_PORT,SPIEE_SCK_PIN,SPIEE_SCK_CLK,SPIEE_SCK_SOURCE,SPIEE_SCK_AF},
		{SPIEE_MOSI_PORT,SPIEE_MOSI_PIN,SPIEE_MOSI_CLK,SPIEE_MOSI_SOURCE,SPIEE_MOSI_AF},
		{SPIEE_MISO_PORT,SPIEE_MISO_PIN,SPIEE_MISO_CLK,SPIEE_MISO_SOURCE,SPIEE_MISO_AF},
		{SPIEE_NSS_PORT,SPIEE_NSS_PIN,SPIEE_NSS_CLK,SPIEE_NSS_SOURCE,SPIEE_NSS_AF}
};

/* Private define ------------------------------------------------------------*/
#define SPI_FLASH_PAGESIZE    256

/* SPI Flash supported commands */
#define CMD_WRITE     0x02  /* Write to Memory instruction */
#define CMD_WRSR      0x01  /* Write Status Register instruction */
#define CMD_WREN      0x06  /* Write enable instruction */
#define CMD_READ      0x03  /* Read from Memory instruction */
#define CMD_RDSR      0x05  /* Read Status Register instruction  */
#define CMD_RDID      0x9F  /* Read identification */
#define CMD_SE        0xD8  /* Sector Erase instruction */
#define CMD_SUE		  0x20	/* SubSector Erase instruction*/
#define CMD_BE        0xC7  /* Bulk Erase instruction */
#define WIP_FLAG      0x01  /* Write In Progress (WIP) flag */

/*--------------------------------- Private function prototypes--------------------------------*/
STATIC UInt8 SPI_FLASH_SendByte(UInt8 byte);
STATIC void SPI_FLASH_CS_LOW(void);
STATIC void SPI_FLASH_CS_HIGH(void);
STATIC void SPI_FLASH_Check_WRIEN(void);
/* Private functions ---------------------------------------------------------*/

void SPI_FLASH_SectorErase(uint32_t SectorAddr)
{
  SPI_FLASH_WriteEnable();
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(CMD_SE);
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  SPI_FLASH_CS_HIGH();
  SPI_FLASH_WaitForWriteEnd();
}
void SPI_FLASH_SubSectorErase(uint32_t SectorAddr)
{
  SPI_FLASH_WriteEnable();
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(CMD_SUE);
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  SPI_FLASH_CS_HIGH();
  SPI_FLASH_WaitForWriteEnd();
}
void SPI_FLASH_WriteEnable(void)
{
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(CMD_WREN);
  SPI_FLASH_CS_HIGH();
  SPI_FLASH_Check_WRIEN();
}
static void SPI_FLASH_Check_WRIEN(void)
{
	__IO UInt8 status = 0;
	do
	{
		SPI_FLASH_CS_LOW();
		SPI_FLASH_SendByte(CMD_RDSR);
		status = SPI_FLASH_SendByte(0);
		status = SPI_FLASH_SendByte(0);
		SPI_FLASH_CS_HIGH();
	}while(!(status & 0x02));
}
void SPI_FLASH_WaitForWriteEnd(void)
{
	__IO uint8_t flashstatus = 0;
  do
  {
	SPI_FLASH_CS_LOW();
	SPI_FLASH_SendByte(CMD_RDSR);
    flashstatus = SPI_FLASH_SendByte(0x00);
    flashstatus = SPI_FLASH_SendByte(0x00);
    SPI_FLASH_CS_HIGH();
  } while ((flashstatus & WIP_FLAG) == SET);
}
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(CMD_READ);
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  SPI_FLASH_SendByte(ReadAddr & 0xFF);
  while (NumByteToRead--)
  {
    *pBuffer = SPI_FLASH_SendByte(0x00);
    pBuffer++;
  }
  SPI_FLASH_CS_HIGH();
}
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PAGESIZE;
  count = SPI_FLASH_PAGESIZE - Addr;
  NumOfPage =  NumByteToWrite / SPI_FLASH_PAGESIZE;
  NumOfSingle = NumByteToWrite % SPI_FLASH_PAGESIZE;

  if (Addr == 0)
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PAGESIZE */
    {
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PAGESIZE */
    {
      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PAGESIZE);
        WriteAddr +=  SPI_FLASH_PAGESIZE;
        pBuffer += SPI_FLASH_PAGESIZE;
      }

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /* WriteAddr is not SPI_FLASH_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PAGESIZE */
    {
      if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PAGESIZE */
      {
        temp = NumOfSingle - count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);
      }
      else
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PAGESIZE */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PAGESIZE;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PAGESIZE;

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PAGESIZE);
        WriteAddr +=  SPI_FLASH_PAGESIZE;
        pBuffer += SPI_FLASH_PAGESIZE;
      }

      if (NumOfSingle != 0)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  SPI_FLASH_WriteEnable();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(CMD_WRITE);
  SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  SPI_FLASH_SendByte(WriteAddr & 0xFF);
  while (NumByteToWrite--)
  {
    SPI_FLASH_SendByte(*pBuffer);
    pBuffer++;
  }
  SPI_FLASH_CS_HIGH();
  SPI_FLASH_WaitForWriteEnd();
}
void SPI_FLASH_BulkErase(void)
{
  SPI_FLASH_WriteEnable();
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(CMD_BE);
  SPI_FLASH_CS_HIGH();
  SPI_FLASH_WaitForWriteEnd();
}
uint32_t SPI_FLASH_ReadID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  SPI_FLASH_CS_LOW();

  SPI_FLASH_SendByte(0x9F);

  Temp0 = SPI_FLASH_SendByte(0x00);

  Temp1 = SPI_FLASH_SendByte(0x00);

  Temp2 = SPI_FLASH_SendByte(0x00);

  SPI_FLASH_CS_HIGH();

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

  return Temp;
}
void SPIExFLASH_Init(void)
{
	SPI_Dev * Setting = &SPIEE_Dev;
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
	RCC->APB1ENR |= SPIEE_CLK;

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
	SPI_InitStructure.SPI_BaudRatePrescaler = SPIEE_BaudRatePreScaler;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIEE,&SPI_InitStructure);
	SPI_Cmd(SPIEE,ENABLE);
}
STATIC uint8_t SPI_FLASH_SendByte(uint8_t byte)
{
  while(!SPI_I2S_GetFlagStatus(SPIEE,SPI_I2S_FLAG_TXE)){};
  SPI_I2S_SendData(SPIEE,byte);
  while (!SPI_I2S_GetFlagStatus(SPIEE,SPI_I2S_FLAG_RXNE)){};
  return SPI_I2S_ReceiveData(SPIEE);
}
STATIC void SPI_FLASH_CS_HIGH(void)
{
	SPI_Dev * Setting = &SPIEE_Dev;

	Setting->NSS.PORT->BSRRL = Setting->NSS.Pin;
}
STATIC void SPI_FLASH_CS_LOW(void)
{
	SPI_Dev * Setting = &SPIEE_Dev;

	Setting->NSS.PORT->BSRRH = Setting->NSS.Pin;
}
