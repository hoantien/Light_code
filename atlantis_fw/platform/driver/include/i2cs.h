/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    i2cs.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-1-2016
 * @brief   This file contains expand of the i2c_slave driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_SLAVE_H__
#define __I2C_SLAVE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/** I2C Slave time sleep definition */
#define I2C_SLAVE_BUFFER_SIZE			512

/* Exported typedef-----------------------------------------------------------*/
/**
 * i2cs_error_t
 */
typedef enum i2cs_error
{
	I2CMS_ERROR_NONE = 0,
	I2CMS_ERROR_INVALID,
	I2CMS_ERROR_UNINITIALIZED,
	I2CMS_ERROR_TX_OVER_SIZE,
	I2CMS_ERROR_UNKNOWN
} i2cs_error_t;
/**
 * i2c_slave_t
 */
typedef struct i2c_slave
{
	/* Call back upper layer data when received */
	void (*clbk_hdl)(uint8_t data);
	/* Command from host which is received completely data */
	void (*receiver_hdl)(void);
	/* Command from host is received */
	void (*transmitter_hdl)(void);
	/* When receive a restart bit from host. */
	void (*restart_hdl)(void);
	uint8_t slave_address;
} i2c_slave_t;

/**
 * i2cs_t
 */
typedef struct
{
	/* Call back handler when receiver not empty */
	void (*cb_rx_not_empty)(uint8_t byte);
	/* Call back handler when receiver was received completely data */
	void (*cb_rx_completed)(void);
	/* When receive a restart bit from i2c master */
	void (*cb_tx_request)(void);
	/* Initialization status */
	bool is_initialized;
} i2cs_t;
/* Exported functions --------------------------------------------------------*/

/**
 * @brief i2c_slave_init
 * The function shall initialize i2c slave driver
 * @param i2c_slave - I2C Slave object pointer
 * @return None
 */
void i2c_slave_init(i2c_slave_t *i2c_slave);

/**
 * @brief i2c_slave_write
 * The function writes data to i2c slave
 * @param buf - point to buffer write
 * @param len - data length to write
 * @return None
 */
void i2c_slave_write(uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* __I2C_SLAVE_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
