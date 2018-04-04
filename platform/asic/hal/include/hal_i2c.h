/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_i2c.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-09-2016
 * @brief   This file contains definitions of the I2C driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported typedef ----------------------------------------------------------*/
/*
 * @brief hal_i2c_return_t
 *
 * I2C return type
 */
typedef enum
{
	I2C_OK = 0,
	I2C_TIMED_OUT
} hal_i2c_return_t;

/*
 * @brief hal_i2c_channel_t
 *
 * I2C channels
 */
typedef enum
{
	I2C_CH0 = 0,
	I2C_CH1,
	I2C_CH2,
	I2C_CH3,
	I2C_CH4,
	I2C_CH5,
	I2C_CH6,
	I2C_CH7,
	I2C_CH8,
	I2C_CH9,
	I2C_CH10,
	I2C_CH11,
	I2C_CH12,
	I2C_CH13,
	I2C_CH14,
	I2C_CH15,
	I2C_CH16,
	I2C_CH17,
	I2C_CH18,
	I2CS_CH0,
	I2CS_CH1,
	I2C_CH_MAX_IDX,
} hal_i2c_channel_t;

/*
 * @brief hal_i2c_operation_mode_t
 *
 * I2C operation mode
 */
typedef enum
{
	I2C_MASTER = 0,
	I2C_SLAVE
} hal_i2c_operation_mode_t;

/*
 * @brief hal_i2c_address_mode_t
 *
 * I2C address mode
 */
typedef enum
{
	I2C_7BIT = 0,
	I2C_10BIT
} hal_i2c_address_mode_t;

/*
 * @brief hal_i2c_clockspeed_t
 *
 * I2C clock speed mode
 */
typedef enum
{
	I2C_SPEED_100KHz = 0,
	I2C_SPEED_400KHz,
	I2C_SPEED_1MHz
} hal_i2c_clockspeed_t;

/*
 * @brief hal_i2c_clockspeed_t
 *
 * I2C clock speed mode
 */
typedef struct
{
	uint8_t		*bytes;		/* data buffer */
	uint16_t	length;		/* data length */
} hal_i2c_buffer_t;

/*
 * @brief hal_i2c_status_t
 *
 * I2C status
 */
typedef enum
{
	I2C_IDLE = 0,
	I2C_RX_RECEIVING,
	I2C_RX_COMPLETED,
	I2C_TX_TRASMITTING,
	I2C_TX_COMPLETED,
	I2C_READ_REQUESTED,
	I2C_RESTARTED,
	I2C_ERROR
} hal_i2c_status_t;

/*
 * @brief hal_i2c_t
 *
 * I2C driver structure
 */
typedef struct
{
	hal_i2c_channel_t			chid;			/* channel id */
	hal_i2c_clockspeed_t		clock_speed;	/* clock speed */
	hal_i2c_address_mode_t		address_mode;	/* address mode */
	hal_i2c_operation_mode_t	operation_mode;	/* operation mode */
	uint16_t					owner_addr;		/* owner address */
	void		(*irq_handler)(hal_i2c_channel_t chid, hal_i2c_status_t status);
} hal_i2c_t;

/* Exported define -----------------------------------------------------------*/
#define I2CM_CH_MAX_IDX			(I2C_CH18 + 1)
#define I2CS_CH_MAX_IDX			(I2C_CH_MAX_IDX - I2CM_CH_MAX_IDX)

/*
 * Macro used for asserting channel ID
 */
#define IS_I2C_CHANNEL(CHANNEL)		((CHANNEL >= I2C_CH0) && \
									(CHANNEL < I2C_CH_MAX_IDX))

/*
 * Macro used for asserting speed mode
 */
#define IS_I2C_SPEEDMODE(SPEEDMODE)	((SPEEDMODE == I2C_SPEED_100KHz) || \
									(SPEEDMODE == I2C_SPEED_400KHz) || \
									(SPEEDMODE == I2C_SPEED_1MHz))

/* Exported functions --------------------------------------------------------*/

/*
 * @brief hal_i2c_init
 * Initializes I2C independent channel
 * @param i2c			pointer to hal_i2c_t structure
 */
void hal_i2c_init(hal_i2c_t *i2c);
/*
 * @brief hal_i2c_set_speed
 * Update speed for I2C independent channel
 * @param chid			I2C channel ID
 * @param speed			Speed of I2C independent channel
 */
void hal_i2c_set_speed(uint8_t chid, uint8_t speed);
/*
 * @brief hal_i2c_get_speed
 * Get speed of I2C independent channel
 * @param chid			I2C channel ID
 * @return				Speed of I2C independent channel
 */
hal_i2c_clockspeed_t hal_i2c_get_speed(hal_i2c_channel_t chid);
/*
 * @brief hal_i2c_init
 * De-initializes I2C independent channel
 * @param chid			I2C channel ID
 */
void hal_i2c_deinit(hal_i2c_channel_t chid);
/*
 * @brief hal_i2c_master_tx
 * Transmits data to another I2C slave
 * @param chid			I2C channel ID
 * @param slave_addr	the address of I2C slave
 * @param buf			data buffer which needs to be transmitted
 * @return				reference to hal_i2c_return_t
 */
int hal_i2c_master_tx(hal_i2c_channel_t chid, uint16_t slave_addr,
														hal_i2c_buffer_t *buf);
/*
 * @brief hal_i2c_master_rx
 * Receives data from another I2C slave
 * @param chid			I2C channel ID
 * @param slave_addr	the address of I2C slave
 * @param buf			data buffer to store the received data
 * @return				reference to hal_i2c_return_t
 */
int hal_i2c_master_rx(hal_i2c_channel_t chid, uint16_t slave_addr,
														hal_i2c_buffer_t *buf);
/*
 * @brief hal_i2c_slave_tx
 * Transmits data (responds) to another I2C master
 * @param chid			I2C channel ID
 * @param buf			data buffer which needs to be transmitted
 * @return				reference to hal_i2c_return_t
 */
int hal_i2c_slave_tx(hal_i2c_channel_t chid, hal_i2c_buffer_t *buf);
/*
 * @brief hal_i2c_slave_rx
 * Receives data from another I2C master
 * @param chid			I2C channel ID
 * @param buf			data buffer to store the received data
 * @return				reference to hal_i2c_return_t
 */
int hal_i2c_slave_rx(hal_i2c_channel_t chid, hal_i2c_buffer_t *buf);
/*
 * @brief hal_i2c_enable_irq
 * Enables interrupt for I2C channel
 * @param i2c			pointer to hal_i2c_t structure, used to get I2C channel
 * 									and upper layer interrupt handler
 */
void hal_i2c_enable_irq(hal_i2c_t *i2c);
/*
 * @brief hal_i2c_disable_irq
 * Disables interrupt for I2C channel
 * @param chid			I2C channel ID
 */
void hal_i2c_disable_irq(hal_i2c_channel_t chid);
/*
 * @brief hal_i2c_get_status
 * Gets I2C channel status
 * @param chid			I2C channel ID
 * @return				I2C channel status
 */
hal_i2c_status_t hal_i2c_get_status(hal_i2c_channel_t chid);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_I2C_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
