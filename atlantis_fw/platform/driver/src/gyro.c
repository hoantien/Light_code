/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file	gyro.c
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Jun-21-2016
 * @brief	This file contains expand for LM3644 flash LED driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "os.h"
#include "log.h"
#include "i2cm.h"
#include "lcc_system.h"
#include "gyro.h"

/* Privated define------------------------------------------------------------*/
#define SLOGF_ID						SLOG_ID_GYRO
#ifdef USING_GYRO_FUNC
/* GYRO Register map */
#define GYRO_REG_SELF_TEST_X_GYRO			0x00
#define GYRO_REG_SELF_TEST_Y_GYRO			0x01
#define GYRO_REG_SELF_TEST_Z_GYRO			0x02
#define GYRO_REG_SELF_TEST_X_ACCEL			0x0D
#define GYRO_REG_SELF_TEST_Y_ACCEL			0x0E
#define GYRO_REG_SELF_TEST_Z_ACCEL			0x0F
#define GYRO_REG_XG_OFFS_USRH				0x13
#define GYRO_REG_XG_OFFS_USRL				0x14
#define GYRO_REG_YG_OFFS_USRH				0x15
#define GYRO_REG_YG_OFFS_USRL				0x16
#define GYRO_REG_ZG_OFFS_USRH				0x17
#define GYRO_REG_ZG_OFFS_USRL				0x18
#define GYRO_REG_SMPLRT_DIV					0x19
#define GYRO_REG_CONFIG						0x1A
#define GYRO_REG_GYRO_CONFIG				0x1B
#define GYRO_REG_ACCEL_CONFIG				0x1C
#define GYRO_REG_ACCEL_CONFIG_2				0x1D
#define GYRO_REG_LP_MODE_CONFIG				0x1E
#define GYRO_REG_ACCEL_WOM_X_THR			0x20
#define GYRO_REG_ACCEL_WOM_Y_THR			0x21
#define GYRO_REG_ACCEL_WOM_Z_THR			0x22
#define GYRO_REG_FIFO_EN					0x23
#define GYRO_REG_I2C_MST_CTRL				0x24
#define GYRO_REG_I2C_SLV0_ADDR				0x25
#define GYRO_REG_I2C_SLV0_REG				0x26
#define GYRO_REG_I2C_SLV0_CTRL				0x27
#define GYRO_REG_I2C_SLV1_ADDR				0x28
#define GYRO_REG_I2C_SLV1_REG				0x29
#define GYRO_REG_I2C_SLV1_CTRL				0x2A
#define GYRO_REG_I2C_SLV2_ADDR				0x2B
#define GYRO_REG_I2C_SLV2_REG				0x2C
#define GYRO_REG_I2C_SLV2_CTRL				0x2D
#define GYRO_REG_ODR_DELAY_EN				0x2E
#define GYRO_REG_FSYNC_INT					0x36
#define GYRO_REG_INT_PIN_CFG				0x37
#define GYRO_REG_INT_ENABLE					0x38
#define GYRO_REG_FIFO_WM_INT_STATUS			0x39
#define GYRO_REG_INT_STATUS					0x3A
#define GYRO_REG_ACCEL_XOUT_H				0x3B
#define GYRO_REG_ACCEL_XOUT_L				0x3C
#define GYRO_REG_ACCEL_YOUT_H				0x3D
#define GYRO_REG_ACCEL_YOUT_L				0x3E
#define GYRO_REG_ACCEL_ZOUT_H				0x3F
#define GYRO_REG_ACCEL_ZOUT_L				0x40
#define GYRO_REG_TEMP_OUT_H					0x41
#define GYRO_REG_TEMP_OUT_L					0x42
#define GYRO_REG_GYRO_XOUT_H				0x43
#define GYRO_REG_GYRO_XOUT_L				0x44
#define GYRO_REG_GYRO_YOUT_H				0x45
#define GYRO_REG_GYRO_YOUT_L				0x46
#define GYRO_REG_GYRO_ZOUT_H				0x47
#define GYRO_REG_GYRO_ZOUT_L				0x48
#define GYRO_REG_EXT_SLV_SENS_DATA_00		0x49
#define GYRO_REG_EXT_SLV_SENS_DATA_01		0x4A
#define GYRO_REG_EXT_SLV_SENS_DATA_02		0x4B
#define GYRO_REG_EXT_SLV_SENS_DATA_03		0x4C
#define GYRO_REG_EXT_SLV_SENS_DATA_04		0x4D
#define GYRO_REG_EXT_SLV_SENS_DATA_05		0x4E
#define GYRO_REG_EXT_SLV_SENS_DATA_06		0x4F
#define GYRO_REG_EXT_SLV_SENS_DATA_07		0x50
#define GYRO_REG_EXT_SLV_SENS_DATA_08		0x51
#define GYRO_REG_EXT_SLV_SENS_DATA_09		0x52
#define GYRO_REG_EXT_SLV_SENS_DATA_10		0x53
#define GYRO_REG_EXT_SLV_SENS_DATA_11		0x54
#define GYRO_REG_ODR_DLY_CNT_HI				0x5F
#define GYRO_REG_ODR_DLY_CNT_LO				0x60
#define GYRO_REG_FIFO_WM_TH					0x61
#define GYRO_REG_I2C_SLV0_DO				0x63
#define GYRO_REG_I2C_SLV1_DO				0x64
#define GYRO_REG_I2C_SLV2_DO				0x65
#define GYRO_REG_I2C_MST_DELAY_CTRL			0x67
#define GYRO_REG_SIGNAL_PATH_RESET			0x68
#define GYRO_REG_ACCEL_INTEL_CTRL			0x69
#define GYRO_REG_USER_CTRL					0x6A
#define GYRO_REG_PWR_MGMT_1					0x6B
#define GYRO_REG_PWR_MGMT_2					0x6C
#define GYRO_REG_OIS_ENABLE					0x70
#define GYRO_REG_FIFO_COUNT_H				0x72
#define GYRO_REG_FIFO_COUNT_L				0x73
#define GYRO_REG_FIFO_R_W					0x74
#define GYRO_REG_WHO_AM_I					0x75
#define GYRO_REG_XA_OFFSET_H				0x77
#define GYRO_REG_XA_OFFSET_L				0x78
#define GYRO_REG_YA_OFFSET_H				0x7A
#define GYRO_REG_YA_OFFSET_L				0x7B
#define GYRO_REG_ZA_OFFSET_H				0x7D
#define GYRO_REG_ZA_OFFSET_L				0x7E

/* GYRO Registers specific to secondary interface in ois mode */
#define GYRO_REG_ACCEL_XOUT_OIS_H			0x00
#define GYRO_REG_ACCEL_XOUT_OIS_L			0x01
#define GYRO_REG_ACCEL_YOUT_OIS_H			0x02
#define GYRO_REG_ACCEL_YOUT_OIS_L			0x03
#define GYRO_REG_ACCEL_ZOUT_OIS_H			0x04
#define GYRO_REG_ACCEL_ZOUT_OIS_L			0x05
#define GYRO_REG_ACCEL_TEMP_OIS_H			0x06
#define GYRO_REG_ACCEL_TEMP_OIS_L			0x07
#define GYRO_REG_ACCEL_GYRO_XOUT_OIS_H		0x08
#define GYRO_REG_ACCEL_GYRO_XOUT_OIS_L		0x09
#define GYRO_REG_ACCEL_GYRO_YOUT_OIS_H		0x0A
#define GYRO_REG_ACCEL_GYRO_YOUT_OIS_L		0x0B
#define GYRO_REG_ACCEL_GYRO_ZOUT_OIS_H		0x0C
#define GYRO_REG_ACCEL_GYRO_ZOUT_OIS_L		0x0D

/* GYRO configuration, offset 0x1A */
#define GYRO_SHL_FIFO_MODE			(6)
#define GYRO_MSK_FIFO_MODE			(0x01 << 6)
#define GYRO_SHL_EXT_SYNC_SET		(3)
#define GYRO_MSK_EXT_SYNC_SET		(0x07 << 3)
#define GYRO_SHL_DLPF_CFG			(0)
#define GYRO_MSK_DLPF_CFG			(0x07 << 0)

#define GYRO_EXT_SYNC_SET_DISABLE		0x00
#define GYRO_EXT_SYNC_SET_TEMP_OUT		0x01
#define GYRO_EXT_SYNC_SET_GYRO_XOUT		0x02
#define GYRO_EXT_SYNC_SET_GYRO_YOUT		0x03
#define GYRO_EXT_SYNC_SET_GYRO_ZOUT		0x04
#define GYRO_EXT_SYNC_SET_ACCEL_XOUT	0x05
#define GYRO_EXT_SYNC_SET_ACCEL_YOUT	0x06
#define GYRO_EXT_SYNC_SET_ACCEL_ZOUT	0x07

#define GYRO_DLPF_CFG_250HZ		0x00
#define GYRO_DLPF_CFG_184HZ		0x01
#define GYRO_DLPF_CFG_92HZ		0x02
#define GYRO_DLPF_CFG_41HZ		0x03
#define GYRO_DLPF_CFG_20HZ		0x04
#define GYRO_DLPF_CFG_10HZ		0x05
#define GYRO_DLPF_CFG_5HZ		0x06
#define GYRO_DLPF_CFG_3600HZ	0x07

/* GYRO gyroscope configuration, offset 0x1B */
#define GYRO_SHL_XG_ST			(7)
#define GYRO_MSK_XG_ST			(0x01 << 7)
#define GYRO_SHL_YG_ST			(6)
#define GYRO_MSK_YG_ST			(0x01 << 6)
#define GYRO_SHL_ZG_ST			(5)
#define GYRO_MSK_ZG_ST			(0x01 << 5)
#define GYRO_SHL_FS_SEL			(2)
#define GYRO_MSK_FS_SEL			(0x07 << 2)
#define GYRO_SHL_FCHOICE_B		(0)
#define GYRO_MSK_FCHOICE_B		(0x03 << 0)

#define GYRO_FS_SEL_250DPS		0x00
#define GYRO_FS_SEL_500DPS		0x01
#define GYRO_FS_SEL_1000DPS		0x02
#define GYRO_FS_SEL_2000DPS		0x03
#define GYRO_FS_SEL_REV			0x04
#define GYRO_FS_SEL_31_25DPS	0x05
#define GYRO_FS_SEL_62_5DPS		0x06
#define GYRO_FS_SEL_125DPS		0x07

/* GYRO accelerometer configuration, offset 0x1C */
#define GYRO_SHL_XA_ST			(7)
#define GYRO_MSK_XA_ST			(0x01 << 7)
#define GYRO_SHL_YA_ST			(6)
#define GYRO_MSK_YA_ST			(0x01 << 6)
#define GYRO_SHL_ZA_ST			(5)
#define GYRO_MSK_ZA_ST			(0x01 << 5)
#define GYRO_SHL_AFS_SEL		(3)
#define GYRO_MSK_AFS_SEL		(0x03 << 2)
#define GYRO_SHL_AFS_OIS		(0)
#define GYRO_MSK_AFS_OIS		(0x03 << 0)

#define GYRO_AFS_SEL_2G			0x00
#define GYRO_AFS_SEL_4G			0x01
#define GYRO_AFS_SEL_8G			0x02
#define GYRO_AFS_SEL_16G		0x03

#define GYRO_AFS_OIS_2G			0x00
#define GYRO_AFS_OIS_4G			0x01
#define GYRO_AFS_OIS_8G			0x02
#define GYRO_AFS_OIS_16G		0x03

/* GYRO accelerometer configuration 2, offset 0x1D */
#define GYRO_SHL_DEC2_CFG			(4)
#define GYRO_MSK_DEC2_CFG			(0x03 << 4)
#define GYRO_SHL_ACCEL_FCHOICE_B	(3)
#define GYRO_MSK_ACCEL_FCHOICE_B	(0x01 << 3)
#define GYRO_SHL_A_DLPF_CFG			(0)
#define GYRO_MSK_A_DLPF_CFG			(0x07 << 0)

#define GYRO_A_DLPF_CFG_218_1HZ		0x00
#define GYRO_A_DLPF_CFG_99HZ		0x02
#define GYRO_A_DLPF_CFG_44_8HZ		0x03
#define GYRO_A_DLPF_CFG_21_2HZ		0x04
#define GYRO_A_DLPF_CFG_10_2HZ		0x05
#define GYRO_A_DLPF_CFG_5_05HZ		0x06
#define GYRO_A_DLPF_CFG_420HZ		0x07

#define GYRO_DEC2_CFG_1X			0x00	/* with ACCEL_FCHOICE_B = 1 */
#define GYRO_DEC2_CFG_4X			0x00	/* with ACCEL_FCHOICE_B = 0 */
#define GYRO_DEC2_CFG_8X			0x02
#define GYRO_DEC2_CFG_16X			0x03
#define GYRO_DEC2_CFG_32X			0x04

/* GYRO gyroscope low power mode configuration, offset 0x1E */
#define GYRO_SHL_GYRO_CYCLE			(7)
#define GYRO_MSK_GYRO_CYCLE			(0x01 << 7)
#define GYRO_SHL_GYRO_AVGCFG		(4)
#define GYRO_MSK_GYRO_AVGCFG		(0x07 << 4)

#define GYRO_GYRO_AVGCFG_1X			0x00
#define GYRO_GYRO_AVGCFG_2X			0x01
#define GYRO_GYRO_AVGCFG_4X			0x02
#define GYRO_GYRO_AVGCFG_8X			0x03
#define GYRO_GYRO_AVGCFG_16X		0x04
#define GYRO_GYRO_AVGCFG_32X		0x05
#define GYRO_GYRO_AVGCFG_64X		0x06
#define GYRO_GYRO_AVGCFG_128X		0x07

/* GYRO wake-on motion threshold (x-axis accelerometer), offset 0x20 */
#define GYRO_SHL_WOM_X_TH			(0)
#define GYRO_MSK_WOM_X_TH			(0xFF << 0)

/* GYRO wake-on motion threshold (y-axis accelerometer), offset 0x21 */
#define GYRO_SHL_WOM_Y_TH			(0)
#define GYRO_MSK_WOM_Y_TH			(0xFF << 0)

/* GYRO wake-on motion threshold (z-axis accelerometer), offset 0x22 */
#define GYRO_SHL_WOM_Z_TH			(0)
#define GYRO_MSK_WOM_Z_TH			(0xFF << 0)

/* GYRO fifo enable, offset 0x23 */
#define GYRO_SHL_TEMP_OUT			(7)
#define GYRO_MSK_TEMP_OUT			(0x01 << 7)
#define GYRO_SHL_GYRO_XOUT			(6)
#define GYRO_MSK_GYRO_XOUT			(0x01 << 6)
#define GYRO_SHL_GYRO_YOUT			(5)
#define GYRO_MSK_GYRO_YOUT			(0x01 << 5)
#define GYRO_SHL_GYRO_ZOUT			(4)
#define GYRO_MSK_GYRO_ZOUT			(0x01 << 4)
#define GYRO_SHL_ACCEL_XYZ_OUT		(3)
#define GYRO_MSK_ACCEL_XYZ_OUT		(0x01 << 3)

/* GYRO I2C master control, offset 0x24 */
#define GYRO_SHL_MULT_MST_EN			(7)
#define GYRO_MSK_MULT_MST_EN			(0x01 << 7)
#define GYRO_SHL_I2C_MST_P_NSR			(4)
#define GYRO_MSK_I2C_MST_P_NSR			(0x01 << 4)
#define GYRO_SHL_I2C_MST_CLK			(0)
#define GYRO_MSK_I2C_MST_CLK			(0x0F << 0)

#define GYRO_I2C_MST_CLK_341KHZ		0x00
#define GYRO_I2C_MST_CLK_315KHZ		0x02
#define GYRO_I2C_MST_CLK_293KHZ		0x04
#define GYRO_I2C_MST_CLK_273KHZ		0x06
#define GYRO_I2C_MST_CLK_256KHZ		0x08
#define GYRO_I2C_MST_CLK_410KHZ		0x0C
#define GYRO_I2C_MST_CLK_372KHZ		0x0E

/* GYRO I2C slave 0 physical address, offset 0x25 */
#define GYRO_SHL_I2C_SLV0_RNW			(7)
#define GYRO_MSK_I2C_SLV0_RNW			(0x01 << 7)
#define GYRO_SHL_I2C_ID_0				(0)
#define GYRO_MSK_I2C_ID_0				(0x7F << 0)

/* GYRO I2C slave 0 register address, offset 0x26 */
#define GYRO_SHL_I2C_SLV0_REG			(0)
#define GYRO_MSK_I2C_SLV0_REG			(0xFF << 0)

/* GYRO I2C slave 0 control, offset 0x27 */
#define GYRO_SHL_I2C_SLV0_EN			(7)
#define GYRO_MSK_I2C_SLV0_EN			(0x01 << 7)
#define GYRO_SHL_I2C_SLV0_BYTE_SW		(6)
#define GYRO_MSK_I2C_SLV0_BYTW_SW		(0x01 << 6)
#define GYRO_SHL_I2C_SLV0_REG_DIS		(5)
#define GYRO_MSK_I2C_SLV0_REG_DIS		(0x01 << 5)
#define GYRO_SHL_I2C_SLV0_GRP			(4)
#define GYRO_MSK_I2C_SLV0_LENG			(0x01 << 4)

/* GYRO I2C slave 1 physical address, offset 0x28 */
#define GYRO_SHL_I2C_SLV1_RNW			(7)
#define GYRO_MSK_I2C_SLV1_RNW			(0x01 << 7)
#define GYRO_SHL_I2C_ID_1				(0)
#define GYRO_MSK_I2C_ID_1				(0x7F << 0)

/* GYRO I2C slave 1 register address, offset 0x29 */
#define GYRO_SHL_I2C_SLV1_REG			(0)
#define GYRO_MSK_I2C_SLV1_REG			(0xFF << 0)

/* GYRO I2C slave 1 control, offset 0x2A */
#define GYRO_SHL_I2C_SLV1_EN			(7)
#define GYRO_MSK_I2C_SLV1_EN			(0x01 << 7)
#define GYRO_SHL_I2C_SLV1_BYTE_SW		(6)
#define GYRO_MSK_I2C_SLV1_BYTW_SW		(0x01 << 6)
#define GYRO_SHL_I2C_SLV1_REG_DIS		(5)
#define GYRO_MSK_I2C_SLV1_REG_DIS		(0x01 << 5)
#define GYRO_SHL_I2C_SLV1_GRP			(4)
#define GYRO_MSK_I2C_SLV1_LENG			(0x01 << 4)

/* GYRO I2C slave 2 physical address, offset 0x2B */
#define GYRO_SHL_I2C_SLV2_RNW			(7)
#define GYRO_MSK_I2C_SLV2_RNW			(0x01 << 7)
#define GYRO_SHL_I2C_ID_2				(0)
#define GYRO_MSK_I2C_ID_2				(0x7F << 0)

/* GYRO I2C slave 2 register address, offset 0x2C */
#define GYRO_SHL_I2C_SLV2_REG			(0)
#define GYRO_MSK_I2C_SLV2_REG			(0xFF << 0)

/* GYRO I2C slave 2 control, offset 0x2D */
#define GYRO_SHL_I2C_SLV2_EN			(7)
#define GYRO_MSK_I2C_SLV2_EN			(0x01 << 7)
#define GYRO_SHL_I2C_SLV2_BYTE_SW		(6)
#define GYRO_MSK_I2C_SLV2_BYTW_SW		(0x01 << 6)
#define GYRO_SHL_I2C_SLV2_REG_DIS		(5)
#define GYRO_MSK_I2C_SLV2_REG_DIS		(0x01 << 5)
#define GYRO_SHL_I2C_SLV2_GRP			(4)
#define GYRO_MSK_I2C_SLV2_LENG			(0x01 << 4)

/* GYRO fsync odr delay enable, offset 0x2E */
#define GYRO_SHL_ODR_DELAY_EN			(7)
#define GYRO_MSK_ODR_DELAY_EN			(0x01 << 7)

/* GYRO fsync interrupt status, offset 0x36 */
#define GYRO_SHL_FYSNC_INT				(7)
#define GYRO_MSK_FYSNC_INT				(0x01 << 7)

/* GYRO int pin configuration, offset 0x37 */
#define GYRO_SHL_INT_LEVEL				(7)
#define GYRO_MSK_INT_LEVEL				(0x01 << 7)
#define GYRO_SHL_INT_OPEN				(6)
#define GYRO_MSK_INT_OPEN				(0x01 << 6)
#define GYRO_SHL_LATCH_INT_EN			(5)
#define GYRO_MSK_LATCH_INT_EN			(0x01 << 5)
#define GYRO_SHL_INT_RD_CLEAR			(4)
#define GYRO_MSK_INT_RD_CLEAR			(0x01 << 4)
#define GYRO_SHL_FSYNC_INT_LEVEL		(3)
#define GYRO_MSK_FSYNC_INT_LEVEL		(0x01 << 3)
#define GYRO_SHL_FSYNC_INT_MODE_EN		(2)
#define GYRO_MSK_FSYNC_INT_MODE_EN		(0x01 << 2)

/* GYRO interrupt enable, offset 0x38 */
#define GYRO_SHL_WOM_X_INT_EN				(7)
#define GYRO_MSK_WOM_X_INT_EN				(0x01 << 7)
#define GYRO_SHL_WOM_Y_INT_EN				(6)
#define GYRO_MSK_WOM_Y_INT_EN				(0x01 << 6)
#define GYRO_SHL_WOM_Z_INT_EN				(5)
#define GYRO_MSK_WOM_Z_INT_EN				(0x01 << 5)
#define GYRO_SHL_FIFO_OFLOW_EN				(4)
#define GYRO_MSK_FIFO_OFLOW_EN				(0x01 << 4)
#define GYRO_SHL_GDRIVE_INT_EN				(3)
#define GYRO_MSK_GDRIVE_INT_EN				(0x01 << 3)
#define GYRO_SHL_DATA_RDY_INT_EN			(0)
#define GYRO_MSK_DATA_RDY_INT_EN			(0x01 << 0)

/* GYRO interrupt enable, offset 0x39 */
#define GYRO_SHL_FIFO_WM_INT				(6)
#define GYRO_MSK_FIFO_WM_INT				(0x01 << 6)

/* GYRO interrupt status, offset 0x3A */
#define GYRO_SHL_WOM_X_INT				(7)
#define GYRO_MSK_WOM_X_INT				(0x01 << 7)
#define GYRO_SHL_WOM_Y_INT				(6)
#define GYRO_MSK_WOM_Y_INT				(0x01 << 6)
#define GYRO_SHL_WOM_Z_INT				(5)
#define GYRO_MSK_WOM_Z_INT				(0x01 << 5)
#define GYRO_SHL_FIFO_OFLOW_INT			(4)
#define GYRO_MSK_FIFO_OFLOW_INT			(0x01 << 4)
#define GYRO_SHL_GDRIVE_INT				(2)
#define GYRO_MSK_GDRIVE_INT				(0x01 << 2)
#define GYRO_SHL_DATA_RDY_INT			(0)
#define GYRO_MSK_DATA_RDY_INT			(0x01 << 0)

/* GYRO high byte of accelerometer x-axis data, offset 0x3B */
#define GYRO_SHL_ACCEL_XOUT_H			(0)
#define GYRO_MSK_ACCEL_XOUT_H			(0xFF << 0)

/* GYRO low byte of accelerometer x-axis data, offset 0x3C */
#define GYRO_SHL_ACCEL_XOUT_L			(0)
#define GYRO_MSK_ACCEL_XOUT_L			(0xFF << 0)

/* GYRO high byte of accelerometer y-axis data, offset 0x3D */
#define GYRO_SHL_ACCEL_YOUT_H			(0)
#define GYRO_MSK_ACCEL_YOUT_H			(0xFF << 0)

/* GYRO low byte of accelerometer y-axis data, offset 0x3E */
#define GYRO_SHL_ACCEL_YOUT_L			(0)
#define GYRO_MSK_ACCEL_YOUT_L			(0xFF << 0)

/* GYRO high byte of accelerometer z-axis data, offset 0x3F */
#define GYRO_SHL_ACCEL_ZOUT_H			(0)
#define GYRO_MSK_ACCEL_ZOUT_H			(0xFF << 0)

/* GYRO low byte of accelerometer z-axis data, offset 0x40 */
#define GYRO_SHL_ACCEL_ZOUT_L			(0)
#define GYRO_MSK_ACCEL_ZOUT_L			(0xFF << 0)

/* GYRO high byte of temperature sensor data, offset 0x41 */
#define GYRO_SHL_TEMP_OUT_H				(0)
#define GYRO_MSK_TEMP_OUT_H				(0xFF << 0)

/* GYRO low byte of temperature sensor data, offset 0x42 */
#define GYRO_SHL_TEMP_OUT_L				(0)
#define GYRO_MSK_TEMP_OUT_L				(0xFF << 0)

#define TEMP_SENSITIVITY				326.8
#define TEMP_ROOM_OFFSET				25.0

/* GYRO high byte of gyroscope x-axis data, offset 0x43 */
#define GYRO_SHL_XOUT_H					(0)
#define GYRO_MSK_XOUT_H					(0xFF << 0)

/* GYRO low byte of gyroscope x-axis data, offset 0x44 */
#define GYRO_SHL_XOUT_L					(0)
#define GYRO_MSK_XOUT_L					(0xFF << 0)

/* GYRO high byte of gyroscope y-axis data, offset 0x45 */
#define GYRO_SHL_YOUT_H					(0)
#define GYRO_MSK_YOUT_H					(0xFF << 0)

/* GYRO low byte of gyroscope y-axis data, offset 0x46 */
#define GYRO_SHL_YOUT_L					(0)
#define GYRO_MSK_YOUT_L					(0xFF << 0)

/* GYRO high byte of gyroscope z-axis data, offset 0x47 */
#define GYRO_SHL_ZOUT_H					(0)
#define GYRO_MSK_ZOUT_H					(0xFF << 0)

/* GYRO low byte of gyroscope z-axis data, offset 0x48 */
#define GYRO_SHL_ZOUT_L					(0)
#define GYRO_MSK_ZOUT_L					(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x49 */
#define GYRO_SHL_EXT_SLV_SENS_DATA_00		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_00		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x4A */
#define GYRO_SHL_EXT_SLV_SENS_DATA_01		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_01		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x4B */
#define GYRO_SHL_EXT_SLV_SENS_DATA_02		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_02		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x4C */
#define GYRO_SHL_EXT_SLV_SENS_DATA_03		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_03		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x4D */
#define GYRO_SHL_EXT_SLV_SENS_DATA_04		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_04		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x4E */
#define GYRO_SHL_EXT_SLV_SENS_DATA_05		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_05		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x4F */
#define GYRO_SHL_EXT_SLV_SENS_DATA_06		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_06		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x50 */
#define GYRO_SHL_EXT_SLV_SENS_DATA_07		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_07		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x51 */
#define GYRO_SHL_EXT_SLV_SENS_DATA_08		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_08		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x52 */
#define GYRO_SHL_EXT_SLV_SENS_DATA_09		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_09		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x53 */
#define GYRO_SHL_EXT_SLV_SENS_DATA_10		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_10		(0xFF << 0)

/* GYRO sensor data from external I2C device, offset 0x54 */
#define GYRO_SHL_EXT_SLV_SENS_DATA_11		(0)
#define GYRO_MSK_EXT_SLV_SENS_DATA_11		(0xFF << 0)

/* GYRO fsync ord delay counter high byte, offset 0x5F */
#define GYRO_SHL_ODR_DLY_TIME_CNT_H			(0)
#define GYRO_MSK_ODR_DLY_TIME_CNT_H			(0xFF << 0)

/* GYRO fsync ord delay counter high byte, offset 0x60 */
#define GYRO_SHL_ODR_DLY_TIME_CNT_L			(0)
#define GYRO_MSK_ODR_DLY_TIME_CNT_L			(0xFF << 0)

/* GYRO fifo watermark threshold, offset 0x61 */
#define GYRO_SHL_FIFO_WM_TH				(0)
#define GYRO_MSK_FIFO_WM_TH				(0xFF << 0)

/* GYRO slave 0 data out, offset 0x63 */
#define GYRO_SHL_I2C_SLV0_DO			(0)
#define GYRO_MSK_I2C_SLV0_DO			(0xFF << 0)

/* GYRO slave 1 data out, offset 0x64 */
#define GYRO_SHL_I2C_SLV1_DO			(0)
#define GYRO_MSK_I2C_SLV1_DO			(0xFF << 0)

/* GYRO slave 2 data out, offset 0x65 */
#define GYRO_SHL_I2C_SLV2_DO			(0)
#define GYRO_MSK_I2C_SLV2_DO			(0xFF << 0)

/* GYRO i2c master delay control, offset 0x67 */
#define GYRO_SHL_DELAY_ES_SHADOW		(7)
#define GYRO_MSK_DELAY_ES_SHADOW		(0x01 << 7)
#define GYRO_SHL_I2C_SLV2_DELAY_EN		(2)
#define GYRO_MSK_I2C_SLV2_DELAY_EN		(0x01 << 2)
#define GYRO_SHL_I2C_SLV1_DELAY_EN		(1)
#define GYRO_MSK_I2C_SLV1_DELAY_EN		(0x01 << 1)
#define GYRO_SHL_I2C_SLV0_DELAY_EN		(0)
#define GYRO_MSK_I2C_SLV0_DELAY_EN		(0x01 << 0)

/* GYRO signal path reset, offset 0x68 */
#define GYRO_SHL_FS_SEL_OIS				(5)
#define GYRO_MSK_FS_SEL_OIS				(0x07 << 5)
#define GYRO_SHL_FCHOICE_OIS_B			(3)
#define GYRO_MSK_FCHOICE_OIS_B			(0x03 << 3)
#define GYRO_SHL_GYRO_RST				(2)
#define GYRO_MSK_GYRO_RST				(0x03 << 2)
#define GYRO_SHL_ACCEL_RST				(1)
#define GYRO_MSK_ACCEL_RST				(0x03 << 1)
#define GYRO_SHL_TEMP_RST				(0)
#define GYRO_MSK_TEMP_RST				(0x03 << 0)

#define GYRO_FS_SEL_OIS_250DPS		0x00
#define GYRO_FS_SEL_OIS_500DPS		0x00
#define GYRO_FS_SEL_OIS_1000DPS		0x00
#define GYRO_FS_SEL_OIS_2000DPS		0x00
#define GYRO_FS_SEL_OIS_31_25DPS	0x00
#define GYRO_FS_SEL_OIS_62_5DPS		0x00
#define GYRO_FS_SEL_OIS_125DPS		0x00

/* GYRO accelerometer intelligence control, offset 0x69 */
#define GYRO_SHL_ACCEL_INTEL_EN				(7)
#define GYRO_MSK_ACCEL_INTEL_EN				(0x01 << 7)
#define GYRO_SHL_ACCEL_INTEL_MODE			(6)
#define GYRO_MSK_ACCEL_INTEL_MODE			(0x01 << 6)
#define GYRO_SHL_ACCEL_FCHOICE_OIS_B		(4)
#define GYRO_MSK_ACCEL_FCHOICE_OIS_B		(0x03 << 4)
#define GYRO_SHL_WOM_INT_MODE				(0)
#define GYRO_MSK_WOM_INT_MODE				(0x01 << 0)

/* GYRO user control, offset 0x6A */
#define GYRO_SHL_FIFO_EN					(6)
#define GYRO_MSK_FIFO_EN					(0x01 << 6)
#define GYRO_SHL_I2C_MST_EN					(5)
#define GYRO_MSK_I2C_MST_EN					(0x01 << 5)
#define GYRO_SHL_FIFO_RST					(2)
#define GYRO_MSK_FIFO_RST					(0x01 << 2)
#define GYRO_SHL_SIG_COND_RST				(0)
#define GYRO_MSK_SIG_COND_RST				(0x01 << 0)

/* GYRO power management 1, offset 0x6B */
#define GYRO_SHL_DEVICE_RESET				(7)
#define GYRO_MSK_DEVICE_RESET				(0x01 << 7)
#define GYRO_SHL_SLEEP						(6)
#define GYRO_MSK_SLEEP						(0x01 << 6)
#define GYRO_SHL_CYCLE						(5)
#define GYRO_MSK_CYCLE						(0x01 << 5)
#define GYRO_SHL_STANDBY					(4)
#define GYRO_MSK_STANDBY					(0x01 << 4)
#define GYRO_SHL_TEMP_DIS					(3)
#define GYRO_MSK_TEMP_DIS					(0x01 << 3)
#define GYRO_SHL_CLKSEL						(0)
#define GYRO_MSK_CLKSEL						(0x03 << 0)

/* GYRO power management 2, offset 0x6C */
#define GYRO_SHL_LP_DIS						(7)
#define GYRO_MSK_LP_DIS						(0x01 << 7)
#define GYRO_SHL_STBY_XA					(5)
#define GYRO_MSK_STBY_XA					(0x01 << 5)
#define GYRO_SHL_STBY_YA					(4)
#define GYRO_MSK_STBY_YA					(0x01 << 4)
#define GYRO_SHL_STBY_ZA					(3)
#define GYRO_MSK_STBY_ZA					(0x01 << 3)
#define GYRO_SHL_STBY_XG					(2)
#define GYRO_MSK_STBY_XG					(0x01 << 2)
#define GYRO_SHL_STBY_YG					(1)
#define GYRO_MSK_STBY_YG					(0x01 << 1)
#define GYRO_SHL_STBY_ZG					(0)
#define GYRO_MSK_STBY_ZG					(0x01 << 0)

/* GYRO ois enable, offset 0x70 */
#define GYRO_SHL_OIS_ENABLE					(1)
#define GYRO_MSK_OIS_ENABLE					(0x01 << 1)

/* GYRO fifo count registers, offset 0x72 */
#define GYRO_SHL_FIFO_COUNT_H				(0)
#define GYRO_MSK_FIFO_COUNT_H				(0xFF << 0)

/* GYRO fifo count registers, offset 0x73 */
#define GYRO_SHL_FIFO_COUNT_L				(0)
#define GYRO_MSK_FIFO_COUNT_L				(0xFF << 0)

/* GYRO fifo count registers, offset 0x73 */
#define GYRO_SHL_FIFO_COUNT_L				(0)
#define GYRO_MSK_FIFO_COUNT_L				(0xFF << 0)

/* GYRO fifo read write, offset 0x74 */
#define GYRO_SHL_FIFO_DATA					(0)
#define GYRO_MSK_FIFO_DATA					(0xFF << 0)

/* GYRO who am i, offset 0x75 */
#define GYRO_SHL_WHO_AM_I					(0)
#define GYRO_MSK_WHO_AM_I					(0xFF << 0)
#define GYRO_REG_WHO_AM_I_VALUE				0x20

/* GYRO accelerometer offset registers, offset 0x77 */
#define GYRO_SHL_XA_OFFS_H					(0)
#define GYRO_MSK_XA_OFFS_H					(0xFF << 0)

/* GYRO accelerometer offset registers, offset 0x78 */
#define GYRO_SHL_XA_OFFS_L					(1)
#define GYRO_MSK_XA_OFFS_L					(0x7F << 1)

/* GYRO accelerometer offset registers, offset 0x7A */
#define GYRO_SHL_YA_OFFS_H					(0)
#define GYRO_MSK_YA_OFFS_H					(0xFF << 0)

/* GYRO accelerometer offset registers, offset 0x7B */
#define GYRO_SHL_YA_OFFS_L					(1)
#define GYRO_MSK_YA_OFFS_L					(0x7F << 1)

/* GYRO accelerometer offset registers, offset 0x7D */
#define GYRO_SHL_ZA_OFFS_H					(0)
#define GYRO_MSK_ZA_OFFS_H					(0xFF << 0)

/* GYRO accelerometer offset registers, offset 0x7E */
#define GYRO_SHL_ZA_OFFS_L					(1)
#define GYRO_MSK_ZA_OFFS_L					(0x7F << 1)

/* Privated functions prototypes ---------------------------------------------*/
/* Privated variables --------------------------------------------------------*/
/* Privated functions implementation -----------------------------------------*/
/*
 * gyro_read
 * Read byte value
 */
static int gyro_read(hal_i2c_channel_t i2c, uint8_t sla, uint8_t reg, uint8_t *data)
{
	if(I2CM_ERROR_TRANSCEIVED == i2cm.read(i2c, BYTE_ADDR8, sla, reg, data))
		return GYRO_OK;
	else
		return GYRO_BUSY;
}

/*
 * gyro_write
 * Byte configure
 */
static int gyro_write(hal_i2c_channel_t i2c, uint8_t sla, uint8_t reg, uint8_t *data)
{
	if(I2CM_ERROR_TRANSMITTED == i2cm.write(i2c, BYTE_ADDR8, sla, reg, data))
		return GYRO_OK;
	else
		return GYRO_BUSY;
}

/* Exported functions implementation -----------------------------------------*/
/*
 * flash_light_init
 * Configure the Flash LED devices, input gyro_info_t
 */
int gyro_init(hal_i2c_channel_t chid, gyro_info_t *info)
{
	gyro_status_t ret = GYRO_OK;
	uint8_t reg = 0;

	if (info->i2c_addr == 0)
		return GYRO_VALUE_INVALID;

	ret = gyro_read(chid, info->i2c_addr, GYRO_REG_WHO_AM_I, &reg);
	/* Cannot connect to devices */
	if (GYRO_OK == ret)
	{
		if(reg == GYRO_REG_WHO_AM_I_VALUE)
		{
			info->calib_data.gyro_offset_x = 0;
			info->calib_data.gyro_offset_y = 0;
			info->calib_data.gyro_offset_z = 0;
			info->gyro_scale = MODE_250DPS;
			info->accel_scale = MODE_2G;
			info->gyro_wake_threshold.acc_threshold_x = 0;
			info->gyro_wake_threshold.acc_threshold_y = 50;
			info->gyro_wake_threshold.acc_threshold_z = 50;
			info->calib_data.accel_offset_x = 0;
			info->calib_data.accel_offset_y = 0;
			info->calib_data.accel_offset_x = 0;
			info->gyro_fifo_enable.fifo_gyro_xyz = FALSE;
			info->gyro_fifo_enable.fifo_accel_x = FALSE;
			info->gyro_fifo_enable.fifo_accel_y = FALSE;
			info->gyro_fifo_enable.fifo_accel_z = FALSE;
			info->gyro_intr_enable.data_ready = TRUE;

			/* Reset internal registers */
			reg = GYRO_MSK_DEVICE_RESET;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_PWR_MGMT_1, &reg);
			vTaskDelay(10);

			/* x gyro offset adjustment register */
			reg = (info->calib_data.gyro_offset_x >> 8) & 0xFF;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_XG_OFFS_USRH, &reg);
			reg = info->calib_data.gyro_offset_x  & 0xFF;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_XG_OFFS_USRL, &reg);

			/* y gyro offset adjustment register */
			reg = (info->calib_data.gyro_offset_y >> 8) & 0xFF;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_YG_OFFS_USRH, &reg);
			reg = info->calib_data.gyro_offset_y  & 0xFF;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_YG_OFFS_USRL, &reg);

			/* z gyro offset adjustment register */
			reg = (info->calib_data.gyro_offset_z >> 8) & 0xFF;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_ZG_OFFS_USRH, &reg);
			reg = info->calib_data.gyro_offset_z  & 0xFF;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_ZG_OFFS_USRL, &reg);

			/* Set default value */
			reg = 0x00;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_SMPLRT_DIV, &reg);
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_CONFIG, &reg);
			gyro_set_scale(chid, info, info->gyro_scale, info->accel_scale);

			/* Write default value to accel config 2 reg */
			reg = 0x00;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_ACCEL_CONFIG_2,
				&reg);

			/* Write default value to low power config reg */
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_LP_MODE_CONFIG,
				&reg);

			/* wake on motion threshold - set to 0*/
			reg = info->gyro_wake_threshold.acc_threshold_x;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_ACCEL_WOM_X_THR,
				&reg);
			reg = info->gyro_wake_threshold.acc_threshold_y;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_ACCEL_WOM_Y_THR,
				&reg);
			reg = info->gyro_wake_threshold.acc_threshold_z;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_ACCEL_WOM_Z_THR,
				&reg);

			/* Fifo enable */
			reg = info->gyro_fifo_enable.data_reg;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_FIFO_EN, &reg);

			/* int pin configuration - set to deafault value */
			reg = info->gyro_intr_config.data;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_INT_PIN_CFG, &reg);

			/* interrupt enable */
			reg = info->gyro_intr_enable.data;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_INT_ENABLE, &reg);

			/* Set clock to auto select */
			reg = 1 << 1;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_PWR_MGMT_1, &reg);

			/* Turn on XYZ accel, XYZ gyro */
			reg = 0;
			ret = gyro_write(chid, info->i2c_addr, GYRO_REG_PWR_MGMT_2, &reg);
		}
		else
			SLOGF(SLOG_ERROR, "%s:[%d] Invalid device ID = %d, something me be"
				" incorrect", __FUNCTION__, __LINE__, reg);
	}
	else
		SLOGF(SLOG_ERROR, "%s:[%d] Read I2C failed",
			__FUNCTION__, __LINE__);
	return ret;
}

/*
 * gyro_deinit
 * Configure the Flash LED devices, input gyro_info_t
 */
int gyro_deinit(hal_i2c_channel_t chid, gyro_info_t *info)
{
	uint8_t reg;
	gyro_status_t ret = GYRO_OK;

	/* Reset internal registers */
	reg = GYRO_MSK_DEVICE_RESET;
	ret = gyro_write(chid, info->i2c_addr, GYRO_REG_PWR_MGMT_1, &reg);

	return ret;
}

/*
 * gyro_set_scale
 * Set gyro and accel scale for the Gyro devices
 */
int gyro_set_scale(hal_i2c_channel_t chid, gyro_info_t *info,
	gyro_mode_gyro_scale_t gyro_scale,
	gyro_mode_accel_scale_t accel_scale)
{
	uint8_t reg;
	gyro_status_t ret = GYRO_OK;

	ret = gyro_read(chid, info->i2c_addr, GYRO_REG_GYRO_CONFIG, &reg);
	reg |= gyro_scale << GYRO_SHL_FS_SEL;
	ret = gyro_write(chid, info->i2c_addr, GYRO_REG_GYRO_CONFIG, &reg);

	ret = gyro_read(chid, info->i2c_addr, GYRO_REG_ACCEL_CONFIG, &reg);
	reg |= accel_scale << GYRO_SHL_AFS_SEL;
	ret = gyro_write(chid, info->i2c_addr, GYRO_REG_ACCEL_CONFIG, &reg);

	return ret;
}

gyro_status_t gyro_set_offset(hal_i2c_channel_t chid, gyro_info_t *info,
	int16_t gyro_offset_x, int16_t gyro_offset_y, int16_t gyro_offset_z,
	int16_t accel_offset_x, int16_t accel_offset_y, int16_t accel_offset_z)
{
	uint8_t reg = 0;
	gyro_status_t ret = GYRO_OK;
	/* x gyro offset adjustment register */
	reg = (gyro_offset_x >> 8) & 0xFF;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_XG_OFFS_USRH, &reg);
	reg = info->calib_data.gyro_offset_x  & 0xFF;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_XG_OFFS_USRL, &reg);

	/* y gyro offset adjustment register */
	reg = (info->calib_data.gyro_offset_y >> 8) & 0xFF;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_YG_OFFS_USRH, &reg);
	reg = info->calib_data.gyro_offset_y  & 0xFF;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_YG_OFFS_USRL, &reg);

	/* z gyro offset adjustment register */
	reg = (info->calib_data.gyro_offset_z >> 8) & 0xFF;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_ZG_OFFS_USRH, &reg);
	reg = info->calib_data.gyro_offset_z  & 0xFF;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_ZG_OFFS_USRL, &reg);

	/* x accelerometer offset registers */
	reg = (info->calib_data.accel_offset_x >> 9) & 0xFF;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_XA_OFFSET_H, &reg);
	reg = (info->calib_data.accel_offset_x >> 1) & 0xFE;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_XA_OFFSET_L, &reg);

	/* y accelerometer offset registers */
	reg = (info->calib_data.accel_offset_y >> 9) & 0xFF;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_YA_OFFSET_H, &reg);
	reg = (info->calib_data.accel_offset_y >> 1) & 0xFE;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_YA_OFFSET_L, &reg);

	/* z accelerometer offset registers */
	reg = (info->calib_data.accel_offset_z >> 9) & 0xFF;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_ZA_OFFSET_H, &reg);
	reg = (info->calib_data.accel_offset_z >> 1) & 0xFE;
	ret |= gyro_write(chid, info->i2c_addr, GYRO_REG_ZA_OFFSET_L, &reg);

	return ret;
}
/*
 * gyro_get_info
 * Query GYRO information include: Temp, Gyro, Accel
 */

gyro_status_t gyro_get_temp(hal_i2c_channel_t chid, gyro_info_t *info)
{
	gyro_status_t status = GYRO_OK;
	uint8_t reg = 0;
	int16_t value = 0;

	if(info->i2c_addr == 0)
		return GYRO_VALUE_INVALID;

	/* get temperature */
	status |= gyro_read(chid, info->i2c_addr, GYRO_REG_TEMP_OUT_H, &reg);
	value = reg;
	status |= gyro_read(chid, info->i2c_addr, GYRO_REG_TEMP_OUT_L, &reg);
	value = (value << 8) | reg;
	info->temperature = (double)value / TEMP_SENSITIVITY + TEMP_ROOM_OFFSET;
	SLOGF(SLOG_INFO, "%s:[%d] Temp is %lf", __FUNCTION__, __LINE__,
		info->temperature);

	return status;
}
int gyro_get_info(hal_i2c_channel_t chid, gyro_info_t *info)
{
	gyro_status_t ret = GYRO_OK;
	uint8_t reg = 0;
	int16_t value = 0;

	/* Checking the input parameter and devices status */
	if (info->i2c_addr == 0)
		return GYRO_VALUE_INVALID;

	/* get gyro_x */
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_GYRO_XOUT_H, &reg);
	value = reg << 8;
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_GYRO_XOUT_L, &reg);
	info->gyro_data.gyro_x = value | reg;

	/* get gyro_y */
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_GYRO_YOUT_H, &reg);
	value = reg << 8;
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_GYRO_YOUT_H, &reg);
	info->gyro_data.gyro_y = value | reg;

	/* get gyro_z */
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_GYRO_ZOUT_H, &reg);
	value = reg << 8;
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_GYRO_ZOUT_H, &reg);
	info->gyro_data.gyro_z = value | reg;
	/*
	 * SLOGF(SLOG_INFO, "%s:[%d] Gyro Data is is\tX:%d\tY:%d\tZ:%d",
	 *	__FUNCTION__, __LINE__, info->gyro_data.gyro_x,
	 *	info->gyro_data.gyro_y, info->gyro_data.gyro_z);
	 */

	/* get accel_x */
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_ACCEL_XOUT_H, &reg);
	value = reg << 8;
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_ACCEL_XOUT_L, &reg);
	info->gyro_data.accel_x = value | reg;

	/* get accel_y */
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_ACCEL_YOUT_H, &reg);
	value = reg << 8;
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_ACCEL_YOUT_L, &reg);
	info->gyro_data.accel_y = value | reg;

	/* get accel_z */
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_ACCEL_ZOUT_H, &reg);
	value = reg << 8;
	ret |= gyro_read(chid, info->i2c_addr, GYRO_REG_ACCEL_ZOUT_L, &reg);
	info->gyro_data.accel_z = value | reg;
	/*
	 * SLOGF(SLOG_INFO, "%s:[%d] Accel Data is\tX:%d\tY:%d\tZ:%d",
	 *	__FUNCTION__, __LINE__, info->gyro_data.accel_x,
	 *	info->gyro_data.accel_y, info->gyro_data.accel_z);
	 */

	return ret;
}

/*
 * gyro_get_int
 */
int gyro_get_int(hal_i2c_channel_t chid, gyro_info_t *info, uint8_t *value)
{
	gyro_status_t ret = GYRO_OK;
	uint8_t reg;

	/* Read to cleare */
	ret = gyro_read(chid, info->i2c_addr, GYRO_REG_INT_STATUS, &reg);
	*value = reg;

	return ret;
}

/*
 * gyro_clear_int
 */
int gyro_clear_int(hal_i2c_channel_t chid, gyro_info_t *info)
{
	gyro_status_t ret = GYRO_OK;
	uint8_t reg;

	/* Read to cleare */
	ret = gyro_read(chid, info->i2c_addr, GYRO_REG_INT_STATUS, &reg);

	return ret;
}

/*
 * gyro_set_sleep
 * Enable sleep mode
 */
int gyro_set_sleep(hal_i2c_channel_t chid, gyro_info_t *info, gyro_sleep_mode_t mode)
{
	uint8_t reg;
	gyro_status_t ret = GYRO_OK;

	ret = gyro_read(chid, info->i2c_addr, GYRO_REG_PWR_MGMT_1, &reg);
	if (mode == GYRO_SLEEP)
	{
		reg |= GYRO_MSK_SLEEP;
	}
	else if (mode == GYRO_STANDBY)
	{
		reg |= GYRO_MSK_STANDBY;
	}
	ret = gyro_write(chid, info->i2c_addr, GYRO_REG_PWR_MGMT_1, &reg);

	return ret;
}

/*
 * gyro_get_fifo_count
 */
int gyro_get_fifo_count(hal_i2c_channel_t chid, gyro_info_t *info, uint16_t *value)
{
	gyro_status_t ret = GYRO_OK;
	uint8_t reg_h, reg_l;

	ret = gyro_read(chid, info->i2c_addr, GYRO_REG_FIFO_COUNT_L, &reg_l);
	ret = gyro_read(chid, info->i2c_addr, GYRO_REG_FIFO_COUNT_H, &reg_h);
	*value = ((uint16_t)reg_h << 8) | reg_l;

	return ret;
}

/*
 * gyro_read_fifo
 * Get fifo data GYRO
 */
int gyro_read_fifo(hal_i2c_channel_t chid, gyro_info_t *info, uint8_t *buf, uint16_t len)
{
	gyro_status_t ret = GYRO_OK;
	uint16_t fifo_len;
	uint8_t reg;

	ret = gyro_get_fifo_count(chid, info, &fifo_len);
	if (len > fifo_len)
	{
		len = fifo_len;
	}
	while (len--)
	{
		ret = gyro_read(chid, info->i2c_addr, GYRO_REG_FIFO_R_W, &reg);
		*buf = reg;
		buf++;
	}

	return ret;
}
#endif /* USING_GYRO_FUNC */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
