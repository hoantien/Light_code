/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    spi_slave.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Feb-04-2016
 * @brief   This file contains header of the spi_slave driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_SLAVE_H__
#define __SPI_SLAVE_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported define------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/**
 * spi_slave_status_t typedef
 */
typedef enum spi_slave_status
{
	SPI_SLAVE_RX_COMPLETE,
	SPI_SPAVE_TX_COMPLETE
} spi_slave_status_t;

/**
 * spi_slave_t typedef
 */
typedef struct spi_slave
{
	void (*rx_clbk)(uint8_t *data, uint16_t len);/* Received data call back */
} spi_slave_t;

/* Exported functions --------------------------------------------------------*/
/**
 * task_spi_slave
 *
 */
void task_spi_slave(void *pvparameter);

/**
 * spi_slave_init
 *
 */
void spi_slave_init(spi_slave_t *spi_slave);

/**
 * spi_slave_tx
 *
 */
void spi_slave_tx(uint8_t *data, uint16_t len, uint8_t tx_en);

/**
 * spi_slave_release_semaphore
 *
 */
void spi_slave_release_semaphore(void);

#ifdef __cplusplus
}
#endif
#endif /* __SPI_SLAVE_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
