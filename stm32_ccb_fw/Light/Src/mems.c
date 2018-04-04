/********************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *******************************************************************************/


/* Includes ------------------------------------------------------------------*/

#include "log.h"
#include <stdio.h>
#include <string.h>
#include "mems.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Declare control registers with initial value                               */
/* ===========================================================================*/
#if 0
/* Alias CCB_REG from Memory System */
__attribute__((__section__(".ccb_reg"))) uint8_t CCB_REG[CCB_REG_TOTAL_SIZE];
__attribute__((__section__(".ccb_mem"))) uint8_t CCB_MEM[CCB_MEM_TOTAL_SIZE];
#endif

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void CCB_MEM_Init(void)
{
	uint8_t *wr_flag;

	/* Restore to factory setting when Previous setting was corrupted before */
	wr_flag = (uint8_t *)(CCB_REG_PREV_BASE + CCB_REG_WRITE_FLAG_OFFSET);
	if (*wr_flag != CCB_REG_WRITE_FLAG_OK)
	{
		CCB_MEM_Update(CCB_REG_PREV_BASE, (uint32_t*)CCB_REG_MANF_BASE);
	}

	/* Restore to factory setting when Current setting was corrupted before */
	wr_flag = (uint8_t *)(CCB_REG_CURR_BASE + CCB_REG_WRITE_FLAG_OFFSET);
	if (*wr_flag != CCB_REG_WRITE_FLAG_OK)
	{
		CCB_MEM_Update(CCB_REG_CURR_BASE, (uint32_t*)CCB_REG_PREV_BASE);
	}

	/* Load currents current setting */
	memcpy(CCB_ASIC_MEM, CCB_ASIC_REG, sizeof(CCB_ASIC_Typedef));

	/* TODO: Refill DUMP register default value, Need to remove in real case */
	for (uint8_t i = 0; i < 50; i++)
	{
		CCB_ASIC_MEM->ASIC_DUMP[i] = i+1;
	}
	CCB_ASIC_MEM->ASIC_LIGHT_PROTOCOL = 0x0100;
	memset((void*)CCB_ASIC_MEM->ASIC_OS_VERSION, 0, 8);
	CCB_ASIC_MEM->ASIC_OS_VERSION[1] = 0x01;

	memset((void*)CCB_ASIC_CAM_RD, 0, 0x0040);
}

void CCB_MEM_Exit(void)
{
	/* Store current setting to flash before exit */
	/* TODO: Check error */
	CCB_MEM_Update(CCB_REG_CURR_BASE, (uint32_t*)CCB_MEM_BASE);
	CCB_MEM_Update(CCB_REG_PREV_BASE, (uint32_t*)CCB_REG_CURR_BASE);
}

void CCB_MEM_Update(uint32_t dst, uint32_t *src)
{
	uint32_t uwCnt = 0;
	int size = sizeof(CCB_ASIC_Typedef);

	/* TODO: Insert mutex/semaphore  */

	/* Unlock the Flash *********************************************************/
	/* Enable the flash control register access */
	FLASH_Unlock();

	/* Erase the user Flash area ************************************************/
	/* area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR */

	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
				  	FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	/* Start the erase operation */
	if (FLASH_EraseSector(CCB_ASIC_PREV_SECTOR, VoltageRange_3) != FLASH_COMPLETE)
	{
		/* Error occurred while sector erase.
		 User can add here some code to deal with this error  */
		FLASH_Lock();
		return; /* TODO: Assign return error code */
	}

	/* Program the user Flash area word by word ********************************/
	/* area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR */
	while(uwCnt < size)
	{
		if (FLASH_ProgramWord(dst + uwCnt, *(src + uwCnt)) == FLASH_COMPLETE)
		{
			uwCnt = uwCnt + 4;
		}
		else
		{
			FLASH_Lock();
			return;  /* TODO: Assign return error code */
		}
	}

	/* Set write flag to OK flag */
	if (FLASH_ProgramByte(dst + CCB_REG_WRITE_FLAG_OFFSET, CCB_REG_WRITE_FLAG_OK) == FLASH_COMPLETE)
	{
		FLASH_Lock();
#if 0
		return;  /* TODO: Assign return error code */
#endif
	}
	else
	{
		/* Lock the Flash to disable the flash control register access (recommended
		to protect the FLASH memory against possible unwanted operation) */
		FLASH_Lock();
	}
}


/*
 *  Function to receive command from host and transmit register value to host for
 *  particular purpose.
 */
void CCB_Cmd_Tx_Parser(CCI_Buffer *tx)
{
	switch(tx->cmd)
	{
		case ASIC_ID_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_ID>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_ID);
			tx->len = ASIC_ID_SIZE;
			break;

		case ASIC_LIGHT_PROTOCOL_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_LIGHT_PROTOCOL>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_LIGHT_PROTOCOL);
			tx->len = ASIC_LIGHT_PROTOCOL_SIZE;
			break;

		case ASIC_OS_VERSION_ADDR:
			memcpy(tx->data, (uint8_t *)CCB_ASIC_MEM->ASIC_OS_VERSION, 8);
			tx->len = ASIC_OS_VERSION_SIZE;
			break;

		case ASIC_CALIB_VERSION_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_CALIB_VERSION>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_CALIB_VERSION);
			tx->len = ASIC_CALIB_VERSION_SIZE;
			break;

		case ASIC_CFG_VERSION_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_CFG_VERSION>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_CFG_VERSION);
			tx->len = ASIC_CFG_VERSION_SIZE;
			break;

		case ASIC_FW_CHECKSUM_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_FW_CHECKSUM>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_FW_CHECKSUM);
			tx->len = ASIC_FW_CHECKSUM_SIZE;
			break;

		case ASIC_STATUS_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_STATUS>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_STATUS);
			tx->len = ASIC_STATUS_SIZE;
			break;

		case ASIC_CONFIG_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_CONFIG>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_CONFIG);
			tx->len = ASIC_CONFIG_SIZE;
			break;

		case ASIC_LOG_CTRL_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_LOG_CTRL>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_LOG_CTRL);
			tx->len = ASIC_LOG_CTRL_SIZE;
			break;

		case ASIC_SENSOR_STATUS_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_SENSOR_STATUS>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_SENSOR_STATUS);
			tx->len = ASIC_SENSOR_STATUS_SIZE;
			break;

		case ASIC_TMP1_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_TMP1>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_TMP1);
			tx->len = ASIC_TMP1_SIZE;
			break;

		case ASIC_TMP2_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_TMP2>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_TMP2);
			tx->len = ASIC_TMP2_SIZE;
			break;

		case ASIC_TMP3_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_TMP3>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_TMP3);
			tx->len = ASIC_TMP3_SIZE;
			break;

		case ASIC_TMP4_ADDR:
			tx->data[0] = (uint8_t)(CCB_ASIC_MEM->ASIC_TMP4>>8);
			tx->data[1] = (uint8_t)(CCB_ASIC_MEM->ASIC_TMP4);
			tx->len = ASIC_TMP4_SIZE;
			break;

		case ASIC_DUMP_ADDR:
			memcpy(tx->data, (uint8_t *)CCB_ASIC_MEM->ASIC_DUMP, ASIC_DUMP_SIZE);
			tx->len = ASIC_DUMP_SIZE;
			break;

		default:
			tx->len = 0;
			tx->cmd = 0;
			log_error("Invalid Command");
			return;
	}
}

/*
 *  Function to receive command from host and copy register value from host for
 *  particular purpose.
 */

void CCB_Cmd_Rx_Parser(CCI_Buffer *rx)
{
	switch(rx->cmd)
	{
		case ASIC_ID_ADDR:
			if (rx->len == ASIC_ID_SIZE)
				CCB_ASIC_MEM->ASIC_ID = rx->data[0]<<8 | rx->data[1];
			break;
		case ASIC_CONFIG_ADDR:
			if (rx->len == ASIC_CONFIG_SIZE)
				CCB_ASIC_MEM->ASIC_CONFIG = rx->data[0]<<8 | rx->data[1];
			break;
		case ASIC_LOG_CTRL_ADDR:
			if (rx->len == ASIC_LOG_CTRL_SIZE)
				CCB_ASIC_MEM->ASIC_LOG_CTRL = rx->data[0]<<8 | rx->data[1];
			break;
		case ASIC_DUMP_ADDR:
			memcpy((uint8_t *)CCB_ASIC_MEM->ASIC_DUMP, rx->data, ASIC_DUMP_SIZE);
			break;
		default:
			rx->len = 0;
			rx->cmd = 0;
			log_error("Invalid Command");
			break;
	}
}

/************ Portions COPYRIGHT 2015 Light.Co., Ltd.******END OF FILE******/
