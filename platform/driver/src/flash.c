/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file	flash.c
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Mar-10-2016
 * @brief	This file contains definitions of the flash driver.
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "assert.h"
#include "cortex_r4.h"
#include "std_type.h"
#include "flash.h"
#ifdef ASIC_FWS2
#include "os.h"
#endif
/* Private define ------------------------------------------------------------*/
#define FLASH_CMD_RDID		0x9F
#define FLASH_CMD_RDSR		0x05
#define FLASH_CMD_WRSR		0x01
#define FLASH_CMD_WREN		0x06
#define FLASH_CMD_CE		0xC7
#define FLASH_CMD_SE		0xD8
#define FLASH_CMD_SSE		0x20
#define FLASH_CMD_GBULK		0x98
#define FLASH_CMD_READ		0x03
#define FLASH_CMD_PP		0x02
#define FLASH_CMD_4READ		0xEB
#define FLASH_CMD_4PP		0x32
#define FLASH_WIP_MASK		0x01
#define FLASH_CMD_CLSR		0x50

#define BYTES_PER_READ		64
#define BYTES_PER_WRITE		256

#define SECTOR_SIZE			0x00010000
#define SUB_SECTOR_SIZE		0x00001000

/* Private macro -------------------------------------------------------------*/
/* Qspi address */
#define QSPI_ADDR				((qspi_register_t *)(uint32_t)(QSPI_BASE))
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
qspi_transfer_cmd_t qspi_transfer_cmd;
#ifdef ASIC_FWS2
static xSemaphoreHandle semaphore;
#endif
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
 * flash_init
 */
void flash_init(void)
{
	qspi_transfer_cmd.tx_buff_reg	= QSPI_ADDR->TX_BUFF_REG;
	qspi_transfer_cmd.tx_buff_reg8	= QSPI_ADDR->TX_BUFF_REG8;
	qspi_transfer_cmd.rx_buff_reg	= QSPI_ADDR->RX_BUFF_REG;
	qspi_transfer_cmd.rx_buff_reg8	= QSPI_ADDR->RX_BUFF_REG8;
#ifdef ASIC_FWS2
	if (semaphore == NULL)
	{
		semaphore = xSemaphoreCreateMutex();
		assert_param(semaphore);
	}
#endif
}

/*
 * read_id
 */
uint32_t read_id(void)
{
	uint32_t chip_id;
	/* check parameters */
	assert_param(qspi_transfer_cmd.tx_buff_reg);
	assert_param(qspi_transfer_cmd.tx_buff_reg8);
	assert_param(qspi_transfer_cmd.rx_buff_reg);
	assert_param(qspi_transfer_cmd.rx_buff_reg8);
#ifdef ASIC_FWS2
	xSemaphoreTake(semaphore, portMAX_DELAY);
#endif
	qspi_transfer_cmd.spi_mode_reg = PIO1_GO_MODE;

	qspi_transfer_cmd.pio_mode1.pio_step0_conf.cycle			= 32;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_output_en 	= FOUR_OUTPUTS;

	qspi_transfer_cmd.tx_buff_reg[0] = (FLASH_CMD_RDID << 24);

	transfer_cmd(&qspi_transfer_cmd);

	chip_id = qspi_transfer_cmd.rx_buff_reg[0] & 0x00FFFFFF;

#ifdef ASIC_FWS2
	xSemaphoreGive(semaphore);
#endif

	return chip_id;
}

/*
 * read_status
 */
static uint32_t read_status(void)
{
	uint32_t status;
	uint32_t wip;

	/* check parameters */
	assert_param(qspi_transfer_cmd.tx_buff_reg);
	assert_param(qspi_transfer_cmd.tx_buff_reg8);
	assert_param(qspi_transfer_cmd.rx_buff_reg);
	assert_param(qspi_transfer_cmd.rx_buff_reg8);

	qspi_transfer_cmd.spi_mode_reg = PIO1_GO_MODE;

	qspi_transfer_cmd.pio_mode1.pio_step0_conf.cycle			= 16;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_output_en 	= FOUR_OUTPUTS;

	qspi_transfer_cmd.tx_buff_reg[0] = (FLASH_CMD_RDSR << 8);

	transfer_cmd(&qspi_transfer_cmd);
	status = qspi_transfer_cmd.rx_buff_reg[0];
	wip = status & 0x00000001;

	while(wip)
	{
		transfer_cmd(&qspi_transfer_cmd);
		status = qspi_transfer_cmd.rx_buff_reg[0];
		wip = status & 0x00000001;
	}
	return (status & 0x000000FF);
}

/*
 * is_flash_busy
 */
static flash_status_t is_flash_busy(void)
{
	uint8_t get_rdsr;

	get_rdsr = read_status();
	if((get_rdsr & FLASH_WIP_MASK) == FLASH_WIP_MASK)
		return FLASH_BUSY;
	else
		return FLASH_SUCCESS;
}

/*
 * write_enable
 */
static void write_enable(void)
{
	uint32_t tmp;

	/* check parameters */
	assert_param(qspi_transfer_cmd.tx_buff_reg);
	assert_param(qspi_transfer_cmd.tx_buff_reg8);
	assert_param(qspi_transfer_cmd.rx_buff_reg);
	assert_param(qspi_transfer_cmd.rx_buff_reg8);

	qspi_transfer_cmd.spi_mode_reg = PIO1_GO_MODE;

	qspi_transfer_cmd.pio_mode1.pio_step0_conf.cycle			= 8;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_output_en 	= FOUR_OUTPUTS;

	qspi_transfer_cmd.tx_buff_reg[0] = FLASH_CMD_WREN;

	transfer_cmd(&qspi_transfer_cmd);

	do
	{
		tmp = read_status();
	} while((tmp & 0x02) == 0);
}

/*
 * bulk_erase
 */
flash_status_t bulk_erase(void)
{
	/* check parameters */
	assert_param(qspi_transfer_cmd.tx_buff_reg);
	assert_param(qspi_transfer_cmd.tx_buff_reg8);
	assert_param(qspi_transfer_cmd.rx_buff_reg);
	assert_param(qspi_transfer_cmd.rx_buff_reg8);

#ifdef ASIC_FWS2
	xSemaphoreTake(semaphore, portMAX_DELAY);
#endif
	if(is_flash_busy() == FLASH_BUSY)
	{
#ifdef ASIC_FWS2
		xSemaphoreGive(semaphore);
#endif
		return FLASH_BUSY;
	}

	write_enable();

	qspi_transfer_cmd.spi_mode_reg = PIO1_GO_MODE;

	qspi_transfer_cmd.pio_mode1.pio_step0_conf.cycle			= 8;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_output_en 	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_lanes 		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_output_en 	= FOUR_OUTPUTS;

	qspi_transfer_cmd.tx_buff_reg[0] = FLASH_CMD_CE;

	transfer_cmd(&qspi_transfer_cmd);

	read_status();
#ifdef ASIC_FWS2
	xSemaphoreGive(semaphore);
#endif

	return FLASH_SUCCESS;
}

/*
 * sector_erase
 */
static flash_status_t sector_erase(uint32_t flash_addr, uint32_t cmd)
{
	/* check parameters */
	assert_param(qspi_transfer_cmd.tx_buff_reg);
	assert_param(qspi_transfer_cmd.tx_buff_reg8);
	assert_param(qspi_transfer_cmd.rx_buff_reg);
	assert_param(qspi_transfer_cmd.rx_buff_reg8);

	if(is_flash_busy() == FLASH_BUSY)
		return FLASH_BUSY;

	write_enable();

	qspi_transfer_cmd.spi_mode_reg = PIO1_GO_MODE;

	qspi_transfer_cmd.pio_mode1.pio_step0_conf.cycle			= 32;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_lanes		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_output_en	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_lanes		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_output_en	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_lanes		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_output_en	= FOUR_OUTPUTS;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.cycle			= 0;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_lanes		= ONE_LANE;
	qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_output_en	= FOUR_OUTPUTS;

	qspi_transfer_cmd.tx_buff_reg[0] = (cmd << 24) | \
			(flash_addr & 0x00FFFFFF);

	transfer_cmd(&qspi_transfer_cmd);

	read_status();

	return FLASH_SUCCESS;
}

/*
 * cmd_read
 */
static flash_status_t cmd_read(uint32_t flash_addr, uint8_t *target_addr,\
		uint32_t byte_length, qspi_mode_t qmode)
{
	uint32_t i, cycle, cycle_rem, tmp;
	int j;

	/* check parameters */
	assert_param(qspi_transfer_cmd.tx_buff_reg);
	assert_param(qspi_transfer_cmd.tx_buff_reg8);
	assert_param(qspi_transfer_cmd.rx_buff_reg);
	assert_param(qspi_transfer_cmd.rx_buff_reg8);

	if(qmode == QSPI_MODE)
		byte_length += 2;
	cycle = byte_length >> 2;
	cycle_rem = byte_length % 4;

	qspi_transfer_cmd.spi_mode_reg = PIO1_GO_MODE;

	if(qmode == QSPI_MODE)
	{
		qspi_transfer_cmd.pio_mode1.pio_step0_conf.cycle		 = 8;
		qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_lanes	 = ONE_LANE;
		qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_output_en = FOUR_OUTPUTS;
		qspi_transfer_cmd.pio_mode1.pio_step1_conf.cycle		 = 6;
		qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_lanes	 = FOUR_LANES;
		qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_output_en = FOUR_OUTPUTS;
		qspi_transfer_cmd.pio_mode1.pio_step2_conf.cycle		 = 6;
		qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_lanes	 = FOUR_LANES;
		qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_output_en = FOUR_OUTPUTS;
		qspi_transfer_cmd.pio_mode1.pio_step3_conf.cycle		 = \
															(byte_length << 1);
		qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_lanes	 = FOUR_LANES;
		qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_output_en = NO_OUTPUT;
	}
	else
	{
		qspi_transfer_cmd.pio_mode1.pio_step0_conf.cycle		 = 32;
		qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_lanes	 = ONE_LANE;
		qspi_transfer_cmd.pio_mode1.pio_step0_conf.pio_output_en = FOUR_OUTPUTS;
		qspi_transfer_cmd.pio_mode1.pio_step1_conf.cycle		 = \
																8 * byte_length;
		qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_lanes	 = ONE_LANE;
		qspi_transfer_cmd.pio_mode1.pio_step1_conf.pio_output_en = FOUR_OUTPUTS;
		qspi_transfer_cmd.pio_mode1.pio_step2_conf.cycle		 = 0;
		qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_lanes	 = ONE_LANE;
		qspi_transfer_cmd.pio_mode1.pio_step2_conf.pio_output_en = FOUR_OUTPUTS;
		qspi_transfer_cmd.pio_mode1.pio_step3_conf.cycle		 = 0;
		qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_lanes	 = ONE_LANE;
		qspi_transfer_cmd.pio_mode1.pio_step3_conf.pio_output_en = FOUR_OUTPUTS;
	}

	for(i = 0; i < cycle; i++)
	{
		qspi_transfer_cmd.tx_buff_reg[i] = 0;
	}

	switch(cycle_rem)
	{
		case 0:
			if(qmode == QSPI_MODE)
			{
				qspi_transfer_cmd.tx_buff_reg[i] = flash_addr << 24;
				*((uint32_t *)(qspi_transfer_cmd.tx_buff_reg8 + (i + 1) * 4)) =\
					(FLASH_CMD_4READ << 16) | ((flash_addr >> 8) & 0x0000FFFF);
			}
			else
			{
				qspi_transfer_cmd.tx_buff_reg[i] = (FLASH_CMD_READ << 24) |\
				(flash_addr & 0x00FFFFFF);
			}
		break;

		case 1:
			if(qmode == QSPI_MODE)
			{
				qspi_transfer_cmd.tx_buff_reg[i] = 0x00000000;
				*((uint32_t *)(qspi_transfer_cmd.tx_buff_reg8 + (i + 1) * 4)) =\
						(FLASH_CMD_4READ << 24) | (flash_addr & 0x00FFFFFF);
			}
			else
			{
				qspi_transfer_cmd.tx_buff_reg[i] = \
						(flash_addr << 8) & 0xFFFFFF00;
				*((uint32_t *)(qspi_transfer_cmd.tx_buff_reg8 + (i + 1) * 4)) =\
						FLASH_CMD_READ;
			}
		break;

		case 2:
			if(qmode == QSPI_MODE)
			{
				qspi_transfer_cmd.tx_buff_reg[i] = 0x00000000;
				*((uint32_t *)(qspi_transfer_cmd.tx_buff_reg8 + (i + 1) * 4)) =\
						flash_addr << 8;
				*((uint32_t *)(qspi_transfer_cmd.tx_buff_reg8 + (i + 2) * 4)) =\
						FLASH_CMD_4READ;
			}
			else
			{
				qspi_transfer_cmd.tx_buff_reg[i] =\
						(flash_addr << 16) & 0xFFFF0000;
				*((uint32_t *)(qspi_transfer_cmd.tx_buff_reg8 + (i + 1) * 4)) =\
					(FLASH_CMD_READ << 8) | ((flash_addr >> 16) & 0x000000FF);
			}
		break;

		case 3:
			if(qmode == QSPI_MODE)
			{
				qspi_transfer_cmd.tx_buff_reg[i] = 0x00000000;
				*((uint32_t *)(qspi_transfer_cmd.tx_buff_reg8 + (i + 1) * 4)) =\
						flash_addr << 16;
				*((uint32_t *)(qspi_transfer_cmd.tx_buff_reg8 + (i + 2) * 4)) =\
					(FLASH_CMD_4READ << 8) | ((flash_addr >> 16) & 0x000000FF);
			}
			else
			{
				qspi_transfer_cmd.tx_buff_reg[i] = \
						(flash_addr << 24) & 0xFF000000;
				*((uint32_t *)(qspi_transfer_cmd.tx_buff_reg8 + (i + 1) * 4)) =\
					(FLASH_CMD_READ << 16) | ((flash_addr >> 8) & 0x0000FFFF);
			}
		break;
	}

	transfer_cmd(&qspi_transfer_cmd);

	if(cycle_rem)
		cycle++;

	if(qmode == QSPI_MODE)
		byte_length -= 2;
	j = byte_length - 1;
	for(i = 0; i < cycle; i++)
	{
		tmp = qspi_transfer_cmd.rx_buff_reg[i];

		target_addr[j] = (uint8_t)(tmp & 0x000000FF);
		j--; if(j < 0) break;
		target_addr[j] = (uint8_t)((tmp >> 8) & 0x000000FF);
		j--; if(j < 0) break;
		target_addr[j] = (uint8_t)((tmp >> 16) & 0x000000FF);
		j--; if(j < 0) break;
		target_addr[j] = (uint8_t)((tmp >> 24) & 0x000000FF);
		j--; if(j < 0) break;
	}

	return FLASH_SUCCESS;
}

/*
 * cmd_write
 */
static flash_status_t cmd_write(uint32_t flash_addr, uint8_t *buf,\
		uint32_t byte_length, qspi_mode_t qmode)
{
	uint32_t i, cycle;
	uint32_t *pdata;

	/* check parameters */
	assert_param(qspi_transfer_cmd.tx_buff_reg);
	assert_param(qspi_transfer_cmd.tx_buff_reg8);
	assert_param(qspi_transfer_cmd.rx_buff_reg);
	assert_param(qspi_transfer_cmd.rx_buff_reg8);

	if(is_flash_busy() == FLASH_BUSY)
		return FLASH_BUSY;

	cycle = byte_length >> 2;
	if(byte_length % 4)
		cycle++;

	write_enable();

	qspi_transfer_cmd.spi_mode_reg = PP_GO_MODE;

	if(qmode == QSPI_MODE)
	{
		qspi_transfer_cmd.pp_mode.pp_cmd_code				= FLASH_CMD_4PP;
		qspi_transfer_cmd.pp_mode.pp_addr_conf.pp_cmd_cycle = 24;
		qspi_transfer_cmd.pp_mode.pp_addr_conf.pp_lanes 	= ONE_LANE;
		qspi_transfer_cmd.pp_mode.pp_data_conf.pp_cmd_cycle = byte_length << 1;
		qspi_transfer_cmd.pp_mode.pp_data_conf.pp_lanes		= FOUR_LANES;
	}
	else
	{
		qspi_transfer_cmd.pp_mode.pp_cmd_code				= FLASH_CMD_PP;
		qspi_transfer_cmd.pp_mode.pp_addr_conf.pp_cmd_cycle = 24;
		qspi_transfer_cmd.pp_mode.pp_addr_conf.pp_lanes		= ONE_LANE;
		qspi_transfer_cmd.pp_mode.pp_data_conf.pp_cmd_cycle	= byte_length << 3;
		qspi_transfer_cmd.pp_mode.pp_data_conf.pp_lanes		= ONE_LANE;
	}

	qspi_transfer_cmd.pp_mode.pp_cmd_conf.pp_cmd_cycle		= 8;
	qspi_transfer_cmd.pp_mode.pp_cmd_conf.pp_lanes			= ONE_LANE;
	qspi_transfer_cmd.pp_mode.pp_addr_code					= flash_addr;
	qspi_transfer_cmd.pp_mode.pp_dummy_code					= 0x00000000;
	qspi_transfer_cmd.pp_mode.pp_dummy_conf.pp_cmd_cycle	= 0;
	qspi_transfer_cmd.pp_mode.pp_dummy_conf.pp_lanes		= NO_LANE;

	pdata = (uint32_t *)buf;
	for(i = 0; i < cycle; i++)
	{
		qspi_transfer_cmd.tx_buff_reg[i]	= *(pdata + i);
	}

	transfer_cmd(&qspi_transfer_cmd);

	read_status();

	return FLASH_SUCCESS;
}

/*
 * flash_read
 */
flash_status_t flash_read(uint32_t addr, uint32_t len,\
		uint8_t *buf, qspi_mode_t qmode)
{
#ifdef ASIC_FWS2
	xSemaphoreTake(semaphore, portMAX_DELAY);
#endif
	while(len)
	{
		if(len > BYTES_PER_READ)
		{
			cmd_read(addr, buf, BYTES_PER_READ, qmode);
			addr += BYTES_PER_READ;
			buf += BYTES_PER_READ;
			len -= BYTES_PER_READ;
		}
		else
		{
			cmd_read(addr, buf, len, qmode);
			len = 0;
		}
	}
#ifdef ASIC_FWS2
	xSemaphoreGive(semaphore);
#endif

	return FLASH_SUCCESS;
}

/*
 * flash_write
 */
flash_status_t flash_write(uint32_t addr, uint32_t len,\
		const uint8_t *buf, qspi_mode_t qmode)
{
	uint8_t *pbuf;
	uint32_t tlen;

	pbuf = (uint8_t *)buf;
	uint32_t offset;

#ifdef ASIC_FWS2
	xSemaphoreTake(semaphore, portMAX_DELAY);
#endif
	while(len)
	{
		offset = addr % BYTES_PER_WRITE;

		if ((len + offset) <= BYTES_PER_WRITE)
			tlen = len;
		else
			tlen = BYTES_PER_WRITE - offset;

		cmd_write(addr, pbuf, tlen, qmode);
		addr += tlen;
		pbuf += tlen;
		len -= tlen;
	}
#ifdef ASIC_FWS2
	xSemaphoreGive(semaphore);
#endif

	return FLASH_SUCCESS;
}

/*
 * flash_sector_erase
 */
flash_status_t flash_sector_erase(uint16_t first_sector,
									uint16_t number_sectors)
{
#ifdef ASIC_FWS2
	xSemaphoreTake(semaphore, portMAX_DELAY);
#endif
	for (int i = 0; i < number_sectors; i++)
	{
		if (sector_erase((first_sector + i) * SECTOR_SIZE, FLASH_CMD_SE)
				!= FLASH_SUCCESS)
		{
#ifdef ASIC_FWS2
			xSemaphoreGive(semaphore);
#endif
			return FLASH_ERROR;
		}
	}
#ifdef ASIC_FWS2
	xSemaphoreGive(semaphore);
#endif
	return FLASH_SUCCESS;
}

/*
 * flash_sub_sector_erase
 */
flash_status_t flash_sub_sector_erase(uint16_t first_sector,
										uint16_t number_sectors)
{
#ifdef ASIC_FWS2
	xSemaphoreTake(semaphore, portMAX_DELAY);
#endif
	for (int i = 0; i < number_sectors; i++)
	{
		if (sector_erase((first_sector + i) * SUB_SECTOR_SIZE, FLASH_CMD_SSE)
				!= FLASH_SUCCESS)
		{
#ifdef ASIC_FWS2
			xSemaphoreGive(semaphore);
#endif
			return FLASH_ERROR;
		}
	}
#ifdef ASIC_FWS2
	xSemaphoreGive(semaphore);
#endif
	return FLASH_SUCCESS;
}
/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
