/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_mipi.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jan 20, 2016
 * @brief   This file contains expand of the hal_mipi driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	07-July-2016	Initial revision:
 *                      - Infrastructure.
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _IT_HAL_MIPI_H_
#define _IT_HAL_MIPI_H_
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "board_config.h"
#include "hal_mipi2axi.h"
#include "hal_axi2mipi.h"
#include "qc_assert.h"
#include "it_log_swapper.h"
#include "qc_camera.h"
#include "hal_ddr.h"
/* Exported typedef ----------------------------------------------------------*/

typedef enum mipi_type
{
	mipi_tx,
	mipi_rx,
	mipi_all
}mipi_type;

typedef struct tx_callback_index
{
	uint8_t frame_start;
	uint8_t frame_end;
	uint8_t dma_line;
	uint8_t dma_frame;
	uint8_t fifo_empty;
	uint8_t fifo_full;
	uint8_t axi_error;
}tx_callback_index;

typedef struct rx_callback_index
{
	uint8_t frame_start;
	uint8_t frame_end;
	uint8_t dma_finish;
	uint8_t line_start;
	uint8_t line_end;
	uint8_t write_dma_finish;
	uint8_t write_dma_finish_WRAP;
}rx_callback_index;


/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
IMPORT int it_hal_mipi_handler(char** argv, int argc);

#endif /* _IT_HAL_COM_H_ */


/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
