/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    gyro.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jun-21-2016
 * @brief   Header file of GYRO sensor driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GYRO_H__
#define __GYRO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
#ifdef USING_GYRO_FUNC
/*
 * @brief gyro_status_t
 * GYRO return status
 */
typedef enum
{
	GYRO_OK = 				0,
	GYRO_I2C_BUSY = 		1,
	GYRO_ALERT = 			2,
	GYRO_VALUE_INVALID =	3,
	GYRO_BUSY = 			4,
} gyro_status_t;

/*
 * @brief gyro_mode_gyro_scale_t
 * GYRO gyro full scale select mode
 */
typedef enum
{
	MODE_250DPS =		0,
	MODE_500DPS =		1,
	MODE_1000DPS =		2,
	MODE_2000DPS =		3,
	MODE_31_25DPS =		5,
	MODE_62_5DPS =		6,
	MODE_125DPS =		7,
} gyro_mode_gyro_scale_t;

/*
 * @brief gyro_mode_accel_scale_t
 * GYRO accel full scale select mode
 */
typedef enum
{
	MODE_2G =		0,
	MODE_4G =		1,
	MODE_8G =		2,
	MODE_16G =		3,
} gyro_mode_accel_scale_t;

/*
 * @brief gyro_ois_mode_t
 * GYRO ois select mode
 */
typedef enum
{
	MODE_OIS_I2C =		0,
	MODE_OIS_SPI =		1,
} gyro_ois_mode_t;

/*
 * @brief gyro_command_t
 * GYRO return status
 */
typedef enum
{
	GYRO_ENABLE =			0,
	ACCEL_ENABLE =			1,
	GYRO_SHUTDOWN =			2,
	ACCEL_SHUTDOWN =		3,
} gyro_command_t;

/*
 * @brief gyro_command_t
 * GYRO return status
 */
typedef enum
{
	GYRO_SLEEP =			0,
	GYRO_STANDBY =			1,
} gyro_sleep_mode_t;

typedef enum
{
	GYRO_FLAG_DONE =		0,
	GYRO_FLAG_READ =		1,
	GYRO_FLAG_GET_SAMPLE =	2,
	GYRO_FLAG_BUSY =		3,
	GYRO_FLAG_ERROR =		4
} gyro_flag_t;

/*
 * @brief gyro_status_t
 * GYRO return status
 */

typedef union
{
	uint8_t buf[12];
	struct
	{
		union
		{
			struct
			{
				int16_t gyro_x;
				int16_t gyro_y;
				int16_t gyro_z;
			};
			int16_t gyro_buf[3];
		};
		union
		{
			struct
			{
				int16_t accel_x;
				int16_t accel_y;
				int16_t accel_z;
			};
			int16_t accel_buf[3];
		};
	};
} gyro_data_t;

typedef union
{
	uint8_t calib_buf[12];
	struct
	{
		union
		{
			struct
			{
				int16_t gyro_offset_x;
				int16_t gyro_offset_y;
				int16_t gyro_offset_z;
			};
			int16_t gyro_offset_buf[3];
		};
		union
		{
			struct
			{
				int16_t accel_offset_x;
				int16_t accel_offset_y;
				int16_t accel_offset_z;
			};
			int16_t accel_offset_buffer[3];
		};
	};
} gyro_calib_data_t;

typedef union
{
	uint8_t data_reg;
	struct
	{
		uint8_t reserved : 3;
		uint8_t fifo_gyro_xyz : 1;
		uint8_t fifo_accel_x : 1;
		uint8_t fifo_accel_y : 1;
		uint8_t fifo_accel_z : 1;
		uint8_t fifo_temp : 1;
	};
} gyro_fifo_enable_t;

typedef union
{
	uint8_t buf[3];
	struct
	{
		uint8_t acc_threshold_x;
		uint8_t acc_threshold_y;
		uint8_t acc_threshold_z;
	};
} gyro_wake_threshold_t;

typedef union
{
	uint8_t data;
	struct
	{
		uint8_t reserved : 2;
		uint8_t fsync_intr_en : 1;
		uint8_t fsync_intr_lvl : 1;
		uint8_t int_read_clear : 1;
		uint8_t latch_intr_en : 1;
		uint8_t int_open : 1;
		uint8_t int_lvl : 1;
	};
} gyro_intr_config_t;

typedef union
{
	uint8_t data;
	struct
	{
		uint8_t intr_data_rdy : 1;
		uint8_t reserved : 1;
		uint8_t intr_gdrive : 1;
		uint8_t reserved_2 : 1;
		uint8_t intr_fifo_ovf : 1;
		uint8_t intr_wom_z : 1;
		uint8_t intr_wom_y : 1;
		uint8_t intr_wom_x : 1;
	};
} gyro_intr_status_t;

typedef union
{
	uint8_t data;
	struct
	{
		uint8_t data_ready : 1;
		uint8_t reserved : 1;
		uint8_t gdrive_intr : 1;
		uint8_t reserved_2 : 1;
		uint8_t Oflow_en : 1;
		uint8_t wom_z_en : 1;
		uint8_t wom_y_en : 1;
		uint8_t wom_x_en : 1;
	};
} gyro_intr_enable_t;

typedef struct
{
	double temperature;
	uint8_t int_status;
	uint16_t fifo_count;
	uint8_t i2c_addr;			/* Gyro slave address */
	uint8_t flag;
	uint16_t sample_num;
	uint16_t count;
	uint32_t interval_us;
	uint8_t *buf;
	gyro_mode_gyro_scale_t gyro_scale;
	gyro_mode_accel_scale_t accel_scale;
	gyro_wake_threshold_t gyro_wake_threshold;
	gyro_intr_enable_t gyro_intr_enable;
	gyro_intr_status_t gyro_intr_status;
	gyro_intr_config_t gyro_intr_config;
	gyro_fifo_enable_t gyro_fifo_enable;
	gyro_data_t gyro_data;
	gyro_calib_data_t calib_data;
	gyro_ois_mode_t ois_mode;
} gyro_info_t;

/*
 * @brief gyro_sensor_t
 * Gyro sensor interface, ASIC1 only
 */
typedef struct
{
	i2c_t 	i2c;
	uint8_t slv_addr;
	gpio_t fsync;
	gpio_t intr1;			/* [in] GYRO interrupt source 1 */
	gpio_t intr2;			/* [in] GYRO interrupt source 2 */
	void (*cb_handler)(void);
} gyro_sensor_t;

/* Exported functions --------------------------------------------------------*/
/*
 * @brief gyro_init
 * Configure the gyro devices
 * @param chid I2C channel ID
 * @param *info gyro device pointer
 * @return gyro_status_t
 */
int gyro_init(i2c_t chid, gyro_info_t *info);

/*
 * @brief gyro_deinit
 * Shutdown the gyro devices
 * @param chid I2C channel ID
 * @param *info gyro device pointer
 * @return gyro_status_t
 */
int gyro_deinit(i2c_t chid, gyro_info_t *info);

/*
 * @brief gyro_get_info
 * Query Gyro information include: Gyro, Accel
 * @param chid I2C channel ID
 * @param *info Flash LED device pointer
 * @return gyro_status_t
 */
int gyro_get_info(i2c_t chid, gyro_info_t *info);

/*
 * @brief gyro_get_temp
 * Get the temperature information from gyro IC
 * @param chid I2C channel ID
 * @param *info Flash LED device pointer
 * @return gyro_status_t
 */
gyro_status_t gyro_get_temp(hal_i2c_channel_t chid, gyro_info_t *info);

/*
 * @brief gyro_active
 * Active the gyro devices, input gyro_scale, accel_scale
 * @return gyro_status_t
 */
int gyro_set_scale(hal_i2c_channel_t chid, gyro_info_t *info,
	gyro_mode_gyro_scale_t gyro_scale,
	gyro_mode_accel_scale_t accel_scale);

/*
 * gyro_get_int
 * @return gyro_status_t
 */
int gyro_get_int(i2c_t chid, gyro_info_t *info, uint8_t *value);

/*
 * gyro_clear_int
 * @return gyro_status_t
 */
int gyro_clear_int(i2c_t chid, gyro_info_t *info);

/*
 * gyro_set_sleep
 * @return gyro_status_t
 */
int gyro_set_sleep(i2c_t chid, gyro_info_t *info, gyro_sleep_mode_t mode);

/*
 * gyro_get_fifo_count
 * @return gyro_status_t
 */
int gyro_get_fifo_count(i2c_t chid, gyro_info_t *info, uint16_t *value);

/*
 * gyro_read_fifo
 * @return gyro_status_t
 */
int gyro_read_fifo(i2c_t chid, gyro_info_t *info, uint8_t *buf, uint16_t len);

/*
 * gyro_set_offset
 @return gyro_status_t
 */
gyro_status_t gyro_set_offset(hal_i2c_channel_t chid, gyro_info_t *info,
	int16_t gyro_offset_x, int16_t gyro_offset_y, int16_t gyro_offset_z,
	int16_t accel_offset_x, int16_t accel_offset_y, int16_t accel_offset_z);
#endif /* USING_GYRO_FUNC */
#ifdef __cplusplus
}
#endif
#endif /* __GYRO_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
