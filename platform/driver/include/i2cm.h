/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    i2cm.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    May-29-2016
 * @brief   This file contains expand of the I2C master control driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_MASTER_H__
#define __I2C_MASTER_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "lcc_system.h"

/* Exported define------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/**
 * i2cm_error_t
 */
typedef enum i2cm_error
{
	I2CM_ERROR_NONE = 0,
	I2CM_ERROR_INVALID,
	I2CM_ERROR_UNINITIALIZED,
	I2CM_ERROR_QUEUE_FULL,
	I2CM_ERROR_UNKNOWN,
	I2CM_ERROR_TRANSMITTED,
	I2CM_ERROR_RECEIVED,
	I2CM_ERROR_TRANSCEIVED,
	I2CM_ERROR_BUSY =			(1 << 7)
} i2cm_error_t;

typedef enum {
	BYTE_ADDR8 = 0,
	BYTE_ADDR16,
	WORD_ADDR8,
	WORD_ADDR16
} i2cm_mode_t;

/**
 * i2cm_dev_t
 */
typedef struct
{
	/**
	 * @brief      I2C Master initialization hardware/parammeters.
	 * @param[in]  chid  I2C channel id.
	 * @return     reference to #i2cm_error_t
	 */
	i2cm_error_t (*init)(i2c_t chid);
	/**
	 * @brief      I2C Master deinitialization hardware, free parammeters.
	 * @param[in]  chid  I2C channel id.
	 * @return     reference to #i2cm_error_t
	 */
	i2cm_error_t (*deinit)(i2c_t chid);
	/**
	 * @brief      I2C transmit and/or receiver data to/from host.
	 * @param[in]  chid               I2C channel id.
	 * @param[in]  addr               I2C slave address.
	 * @param[in]  tx                 I2C tx data pointer.
	 * @param[in]  txlen              Length of transmit data.
	 * @param[in]  rx                 I2C receive data pointer.
	 * @param[in]  rxlen              Length of receive data.
	 *
	 * @return     reference to #i2cm_error_t
	 */
	i2cm_error_t (*transceiver)(i2c_t chid, uint8_t addr,
								uint8_t *tx, uint16_t txlen,
								uint8_t *rx, uint16_t rxlen);
	/**
	 * @brief      I2C read register with option of address and data.
	 * @param[in]  chid               I2C channel id.
	 * @param[in]  mode               Register address length and databyte size
	 * @param[in]  sla                I2C slave address.
	 * @param[in]  reg                Register offset
	 * @param[in]  byte               Data byte/word pointer.
	 *
	 * @return     reference to #i2cm_error_t
	 */
	i2cm_error_t (*read)(i2c_t i2c, i2cm_mode_t mode,
						uint8_t sla, uint16_t reg, uint8_t *byte);
	/**
	 * @brief      I2C write data with option of address and data.
	 * @param[in]  chid               I2C channel id.
	 * @param[in]  mode               Register address length and databyte size
	 * @param[in]  sla                I2C slave address.
	 * @param[in]  reg                Register offset
	 * @param[in]  byte               Data byte/word pointer.
	 *
	 * @return     reference to #i2cm_error_t
	 */
	i2cm_error_t (*write)(i2c_t i2c, i2cm_mode_t mode,
						uint8_t sla, uint16_t reg, uint8_t *byte);
} i2cm_dev_t;

/* Exported variables --------------------------------------------------------*/
extern i2cm_dev_t i2cm;
#ifdef __cplusplus
}
#endif
#endif /* __I2C_MASTER_H__*/
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
