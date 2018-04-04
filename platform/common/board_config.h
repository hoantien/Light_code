/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    board_config.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    June-26-2015
 * @brief   This file contains definitions specified to board
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported define------------------------------------------------------------*/
/** Board peripheral clock */
#define BOARD_PCLOCK						533000000
#define CLOCK_133MHZ						133000000
/** Board version ASB */
#define BOARD_ASB							1

/** Board select */
#define BOARD_VERSION						BOARD_ASB
#define LIGHT_SYSTEM_UCID_MAX				11

/** Config for debug port used in #log.c , reference parameters in #hal_com.h */
#if(BOARD_VERSION == BOARD_ASB)
#define CFG_LOG_DEBUG_PORT					COM1
#endif
#define CFG_LOG_DEBUG_BAUDRATE				COM_BAUD_115200
#define CFG_LOG_DEBUG_PRIORITY				0

#define FREERTOS_SYSTEM_TICK				HAL_TIM_CH1

/** Configure I2C Slave parameters is used in #i2c_slave.c */
#define I2C_SLAVE_HW_CHANNEL				I2CS_CH0
/* Replace I2C channel which choose to forward command.*/
#define I2C_FORWARD_ASIC2_CHANNEL			I2C_CH9
#define I2C_FORWARD_ASIC3_CHANNEL			I2C_CH15
#define I2C_FORWARD_SLAVEADDR				0x08
/** I2C slave address */
#define I2C_SLAVE_ADDRESS					0x08

/** Configure SPI Slave parameters is used in #spi_slave.c */
#define SPI_SLAVE_HW_CHANNEL				HAL_SPI_CH1

/** Configure is used in #lcc_log.c */
#define LCC_CMD_LOG_SIZE					25

/** Configure is used in #task_cmd_ctrl.c */
#define LCC_CMD_CTRL_SIZE					64

/** SPI Slave configuration is used in #spi_slave.c */
#define SPI_SLAVE_SPI						/*set channel ID*/
#define SPI_SLAVE_SPI_CLOCK					25000000
#define SPI_SLAVE_SPI_MODE					SPI_SLAVE
#define SPI_SLAVE_SPI_IRQ_PRIORITY			0
#define SPI_SLAVE_SPI_DATA_MODE				SPI_MODE_3
#define SPI_SLAVE_SPI_TX_RX_MODE			SPI_STD
#define SPI_SLAVE_SPI_DATASIZE				DATASIZE_8BIT

#define SPI_SLAVE_MAX_RX_BUFFER_SIZE		16

/** Configure Temperature sensor slave address is used in #temp_sensor.c */
#define HAL_I2C_TEMP						I2C_CH18
#define TEMP_SLAVE_ADDR						(0x90 >> 1)

#define HAL_I2C_MASTER						I2C_CH18
#define CAM_FLASH_ADDRESS					(0xC6 >> 1)
#define CAM_TOF_ADDRESS						(0x52 >> 1)
#define CAM_GYRO_ADDRESS					(0xD2 >> 1)

/* I2C Slave address 7-bit */
#define IMAGE_SENSOR_ADDRESS				(0x6C >> 1)  /* Aptina */
#define IMAGE_EEPROM_ADDRESS				(0xA0 >> 1)  /* */
#define HALL_MIRR_ADDRESS					(0xAE >> 1)  /* AMS AS5510 */
#define HALL_LENS_ADDRESS					(0x18 >> 1)  /* Allegro A1457 */
#define VCM_ADDRESS							(0xE4 >> 1)  /* LC898214 */
#define TEMP_ADDRESS						(0x90 >> 1)  /* TI TMP112 */
#define CURR_1P2V_ADDRESS					(0x80 >> 1)  /* INA231 - 1.2V */
#define CURR_VCCA_ADDRESS					(0x81 >> 1)  /* INA231 - VCCA */
#define CURR_3P3V_ADDRESS					(0x88 >> 1)  /* INA231 - 3.3V */
#define CURR_1P8V_ADDRESS					(0x8A >> 1)  /* INA231 - 1.8V */

#ifdef __cplusplus
}
#endif
#endif /* __BOARD_CONFIG_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
