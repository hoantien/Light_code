/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file	hal_qspi.c
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Mar-10-2016
 * @brief	This file contains definitions of the Qspi driver
 *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "assert.h"
#include "std_type.h"
#include "cortex_r4.h"
#include "hal_qspi.h"

/* Private define ------------------------------------------------------------*/
#define		TIMEOUT				1000000000
#define		ENABLE_SPI_MODE		0x01

/* Private variables ---------------------------------------------------------*/
/* Qspi address */
#define QSPI_ADDR				((qspi_register_t *)(uint32_t)(QSPI_BASE))
/* Private functions ---------------------------------------------------------*/
/*
 * qspi_wait_busy
 */
static qspi_status_t qspi_wait_busy(void)
{
	volatile uint32_t tmp, timeout = TIMEOUT;

	do
	{
		switch(QSPI_ADDR->SPI_MODE)
		{
			case PIO1_GO_MODE:
				tmp = QSPI_ADDR->PIO1_GO;
			break;

			case PIO2_GO_MODE:
				tmp = QSPI_ADDR->PIO2_GO;
			break;

			case PP_GO_MODE:
				tmp = QSPI_ADDR->PP_GO;
			break;

			default:
				tmp = QSPI_ADDR->PIO1_GO;
			break;
		}
	} while((tmp & 0x00000001) && (--timeout));

	if(timeout)
		return QSPI_SUCCESS;
	else
		return QSPI_TIMEOUT;
}

/**
 * @brief transfer_cmd
 * Qspi transfer command
 */
qspi_status_t transfer_cmd(qspi_transfer_cmd_t *qspi_transfer_cmd)
{
	uint32_t *tmp;

	/* check parameters */
	assert_param(qspi_transfer_cmd);
	assert_param(IS_QSPI_MODE(qspi_transfer_cmd->spi_mode_reg));
	assert_param(qspi_transfer_cmd->tx_buff_reg);
	assert_param(qspi_transfer_cmd->tx_buff_reg8);
	assert_param(qspi_transfer_cmd->rx_buff_reg);
	assert_param(qspi_transfer_cmd->rx_buff_reg8);

	QSPI_ADDR->SPI_MODE	= qspi_transfer_cmd->spi_mode_reg;

	if(qspi_transfer_cmd->spi_mode_reg == PIO1_GO_MODE)
	{
		assert_param(IS_QSPI_LANES\
				(qspi_transfer_cmd->pio_mode1.pio_step0_conf.pio_lanes));
		assert_param(IS_QSPI_PIO_OUTPUT\
				(qspi_transfer_cmd->pio_mode1.pio_step0_conf.pio_output_en));

		tmp = (uint32_t *) &qspi_transfer_cmd->pio_mode1.pio_step0_conf;
		QSPI_ADDR->PIO1_STEP0_CONF	= (uint32_t) (*tmp & 0xFFFFFF);
		tmp = (uint32_t *) &qspi_transfer_cmd->pio_mode1.pio_step1_conf;
		QSPI_ADDR->PIO1_STEP1_CONF	= (uint32_t) (*tmp & 0xFFFFFF);
		tmp = (uint32_t *) &qspi_transfer_cmd->pio_mode1.pio_step2_conf;
		QSPI_ADDR->PIO1_STEP2_CONF	= (uint32_t) (*tmp & 0xFFFFFF);
		tmp = (uint32_t *) &qspi_transfer_cmd->pio_mode1.pio_step3_conf;
		QSPI_ADDR->PIO1_STEP3_CONF	= (uint32_t) (*tmp & 0xFFFFFF);
		QSPI_ADDR->PIO1_GO			= ENABLE_SPI_MODE;
	}
	else if(qspi_transfer_cmd->spi_mode_reg == PIO2_GO_MODE)
	{
		assert_param(IS_QSPI_LANES\
				(qspi_transfer_cmd->pio_mode2.pio_step0_conf.pio_lanes));
		assert_param(IS_QSPI_PIO_OUTPUT\
				(qspi_transfer_cmd->pio_mode2.pio_step0_conf.pio_output_en));

		tmp = (uint32_t *) &qspi_transfer_cmd->pio_mode2.pio_step0_conf;
		QSPI_ADDR->PIO2_STEP0_CONF	= (uint32_t) (*tmp & 0xFFFFFF);
		tmp = (uint32_t *) &qspi_transfer_cmd->pio_mode2.pio_step1_conf;
		QSPI_ADDR->PIO2_STEP1_CONF	= (uint32_t) (*tmp & 0xFFFFFF);
		tmp = (uint32_t *) &qspi_transfer_cmd->pio_mode2.pio_step2_conf;
		QSPI_ADDR->PIO2_STEP2_CONF	= (uint32_t) (*tmp & 0xFFFFFF);
		tmp = (uint32_t *) &qspi_transfer_cmd->pio_mode2.pio_step3_conf;
		QSPI_ADDR->PIO2_STEP3_CONF	= (uint32_t) (*tmp & 0xFFFFFF);
		QSPI_ADDR->PIO2_GO			= ENABLE_SPI_MODE;
	}
	else if(qspi_transfer_cmd->spi_mode_reg == PP_GO_MODE)
	{
		assert_param(IS_QSPI_LANES\
				(qspi_transfer_cmd->pp_mode.pp_addr_conf.pp_lanes));
		assert_param(IS_QSPI_LANES\
				(qspi_transfer_cmd->pp_mode.pp_data_conf.pp_lanes));
		assert_param(IS_QSPI_LANES\
				(qspi_transfer_cmd->pp_mode.pp_cmd_conf.pp_lanes));
		assert_param(IS_QSPI_LANES\
				(qspi_transfer_cmd->pp_mode.pp_dummy_conf.pp_lanes));

		QSPI_ADDR->PP_CMD_CODE		= qspi_transfer_cmd->pp_mode.pp_cmd_code;
		tmp = (uint32_t *) &qspi_transfer_cmd->pp_mode.pp_cmd_conf;
		QSPI_ADDR->PP_CMD_CONF		= (uint32_t) (*tmp & 0x7FFFF);
		QSPI_ADDR->PP_ADDR_CODE		= qspi_transfer_cmd->pp_mode.pp_addr_code;
		tmp = (uint32_t *) &qspi_transfer_cmd->pp_mode.pp_addr_conf;
		QSPI_ADDR->PP_ADDR_CONF		= (uint32_t) (*tmp & 0x7FFFF);
		QSPI_ADDR->PP_DUMMY_CODE	= qspi_transfer_cmd->pp_mode.pp_dummy_code;
		tmp = (uint32_t *) &qspi_transfer_cmd->pp_mode.pp_dummy_conf;
		QSPI_ADDR->PP_DUMMY_CONF	= (uint32_t) (*tmp & 0x7FFFF);
		tmp = (uint32_t *) &qspi_transfer_cmd->pp_mode.pp_data_conf;
		QSPI_ADDR->PP_DATA_CONF		= (uint32_t) (*tmp & 0x7FFFF);
		QSPI_ADDR->PP_GO			= ENABLE_SPI_MODE;
	}
	return qspi_wait_busy();
}

/*
 * qspi_init
 */
void qspi_init(qspi_ctrl_mode_t *qspi_ctrl_mode)
{
	uint32_t *tmp;

	/* check parameters */
	assert_param(qspi_ctrl_mode);

	tmp = (uint32_t *) &qspi_ctrl_mode->pio1_ctrl;
	QSPI_ADDR->PIO1_CTRL	= (uint32_t) (*tmp & 0x3FFFFFF);

	tmp = (uint32_t *) &qspi_ctrl_mode->pio2_ctrl;
	QSPI_ADDR->PIO2_CTRL	= (uint32_t) (*tmp & 0x3FFFFFF);

	tmp = (uint32_t *) &qspi_ctrl_mode->pp_ctrl;
	QSPI_ADDR->PP_CTRL		= (uint32_t) (*tmp & 0x3FFFFFF);
}

/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
