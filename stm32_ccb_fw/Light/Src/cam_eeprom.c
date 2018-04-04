/**
  ******************************************************************************
  * @file    cam_eeprom.c
  * @author  Infonam Embedded Team
  * @version V1.1.0
  * @date    14-Jul-2015
  * @brief   This file provides set of firmware functions to erase/write/read
  * 		 AR0835 EEPROM data
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* Library includes. */
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Driver includes. */
#include "platform.h"
#include "log.h"
#include "cam_eeprom.h"
#include "fpga.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef BOARD_VERSION_P1
#define EEPROM_MAX_SIZE			(CAM_EEPROM_SECTOR_SIZE*CAM_EEPROM_SECTOR_NUM)
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1
#define EEPROM_MAX_SIZE			0x153C
#define EEPROM_MAX_ADDR			0x153C
#endif /* BOARD_VERSION_P1_1 */

#define EEPROM_BUFFER_SIZE		(EEPROM_MAX_SIZE*CAM_NUM_CAMERA_MAX)
#define EEPROM_END_ADDR			0xAF00

/* Private macro -------------------------------------------------------------*/
#define SELECT_SLAVE_ADDR(addr)	((addr / CAM_EEPROM_SECTOR_SIZE)*2 + CAM_EEPROM_SLAVE_ADDRESS)
#define SELECT_SUB_ADDR(addr)	(addr % CAM_EEPROM_SECTOR_SIZE)

// This marco auto increment slave address for eeprom GT24C16
#define AUTO_CALC_ADDR(addr, n)	(((addr + n) & 0x0100) ? (addr + n + CAM_EEPROM_SECTOR_SIZE) : (addr + n))
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
uint8_t *g_eeprom_buffer =  (uint8_t *)(CAM_EEPROM_RAM_BASE);

/* Private function prototypes -----------------------------------------------*/
/* Private function ----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
Error_t cam_eeprom_read (CamDevice_TypeDef *cam, UInt16 addr, UInt8* buf, UInt16 size)
{
	Error_t retVal = ERROR_NONE;
#ifdef BOARD_VERSION_P1
	UInt16 data, i, start;
	UInt8 slave_addr, mem_addr;
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
	UInt16 data, i, mem_addr;
#endif /* BOARD_VERSION_P1_1 */

	if ((buf == NULL) || (size == 0)) return ERROR_MEMORY_INVALID;
#ifdef BOARD_VERSION_P1
	if (size > CAM_EEPROM_SECTOR_SIZE*CAM_EEPROM_SECTOR_NUM) return ERROR_OUT_OF_MEMORY;
	if ((addr + ((size & 0x0F00) << 1)) > 0xB000) return ERROR_OUT_OF_MEMORY;
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
	if (size > EEPROM_MAX_SIZE) return ERROR_OUT_OF_MEMORY;
	if ((addr > EEPROM_MAX_ADDR)) return ERROR_OUT_OF_MEMORY;
#endif /* BOARD_VERSION_P1_1 */
#ifdef BOARD_VERSION_P1
	start = AUTO_CALC_ADDR(addr, 0);
#endif /* BOARD_VERSION_P1 */
	for (i = 0; i < size; i++)
	{
#ifdef BOARD_VERSION_P1
		slave_addr = (UInt8)(start>>8);
		mem_addr = (UInt8)(start);
		data = fpga_read_i2c_value(cam->camera_id, slave_addr, mem_addr, ADDR8_DATA8);
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
		mem_addr = addr + i;
		data = fpga_read_i2c_value(cam->camera_id, CAM_EEPROM_SLAVE_ADDRESS, mem_addr, ADDR16_DATA8);
#endif /* BOARD_VERSION_P1_1 */
		buf[i] = (UInt8)data;
#ifdef BOARD_VERSION_P1
		start = AUTO_CALC_ADDR(start, 1);
#endif /* BOARD_VERSION_P1 */
	}

	return retVal;

}

Error_t cam_eeprom_write(CamDevice_TypeDef *cam, UInt16 addr, UInt8* buf, UInt16 size)
{
	Error_t retVal = ERROR_NONE;
	UInt16 i, start, data;
	UInt8 slave_addr, mem_addr;

	if ((buf == NULL) || (size == 0)) return ERROR_MEMORY_INVALID;
	if (size > CAM_EEPROM_SECTOR_SIZE*CAM_EEPROM_SECTOR_NUM) return ERROR_OUT_OF_MEMORY;
	if ((addr + ((size & 0x0F00) << 1)) > 0xB000) return ERROR_OUT_OF_MEMORY;

	start = AUTO_CALC_ADDR(addr, 0);
    for (i = 0; i < size; i++)
    {
    	data = buf[i];
		slave_addr = (UInt8)(start>>8);
		mem_addr = (UInt8)(start);
		fpga_write_i2c_value( cam->camera_id, slave_addr, mem_addr, data, ADDR8_DATA8, EFalse );
		// Boundary of page or lastest data or enough two byte to send
		start = AUTO_CALC_ADDR(start, 1); 	// Update start point
		vTaskDelay(5);
    }

	return retVal;
}

void mem_dump(UInt8 *buf, UInt16 size)
{
	UInt16 i, j;

	/* Print the first line */
	log_printf("     ");
	for (i = 0; i < 16; i++)
	{
		log_printf("\e[32m%02X\e[39m ", i);
	}
	for (i = 0; i < size;)
	{
		/* Print the address */
		log_printf("\n\r\e[32m%02X%02X\e[39m ", SELECT_SLAVE_ADDR(i), SELECT_SUB_ADDR(i));
		for (j = 0; j < 16; j++, i++)
		{
			log_printf("%02X ", buf[i]);
		}
	}
	log_printf("\n\r");
}

void vCamEepromTest(void *pvParameters)
{
	(void)pvParameters;
	UInt16 size = CAM_EEPROM_SECTOR_SIZE*CAM_EEPROM_SECTOR_NUM;
	UInt8 eeprom[size];
	UInt16 rd_blk[] = {size};
	UInt16 i, j;
	CamDevice_TypeDef *cam = CamDeviceTbl[0]; // Reading data only camera B3

//	UInt8 buf[CAM_EEPROM_SECTOR_SIZE];
//	for (i = 0; i < CAM_EEPROM_SECTOR_SIZE; i++)
//	{
//		buf[i] = 0xFF;		// Initialization buffer to write
//	}
//
//	// Write full EEPROM (2Kb)
//	for (i = 0; i < CAM_EEPROM_SECTOR_NUM; i++)
//	{
//		cam_eeprom_write(cam, 0xAE60, buf, 128);
//	}

	/* As per most tasks, this task is implemented in an infinite loop. */
	while(1)
	{
		for (i = 0; i < ARRAY_COUNT(rd_blk); i++)
		{
			// Clear buffer
			memset(eeprom, 0x00, size);

			log_printf("Test read %s with RD_BLK = %d\n\r", CAM_EEPROM_NAME_STRING, rd_blk[i]);
			for (j = 0; j < size;)
			{
				if (cam_eeprom_read(cam, 0xA000 + j, &eeprom[j], rd_blk[i]) != 0)
					log_printf("Read Error\n\r");
				j += rd_blk[i];
			}
			mem_dump(eeprom, size);
		}
	}
}
/*********** Portions COPYRIGHT 2015 Light. Co., Ltd.*******END OF FILE********/
