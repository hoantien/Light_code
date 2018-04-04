/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    pwr_ctrl.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Feb-07-2016
 * @brief   This file contains expand of the CCB power control
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __PWR_CTRL_H__
#define __PWR_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "lcc_system.h"

/* Exported define------------------------------------------------------------*/
/* Exported typedef  ---------------------------------------------------------*/
/*
 * @brief voltage_ctrl_t
 * Voltage control typedef
 */
typedef struct
{
	i2c_t 	i2c;
	uint8_t slv_addr;
	uint32_t voltage;
} voltage_ctrl_t;

/*
 * @brief curr_sensor_t
 * Current sensor typedef
 */
typedef struct
{
	i2c_t 	i2c;
	uint8_t slv_addr;
	uint32_t curr;		/* Current (uA)*/
	uint32_t volt;		/* Voltage (mV)*/
	uint32_t powr;		/* Power (mW)*/
} curr_sensor_t;

/*
 * @brief power_ctrl_t
 * Power control typedef
 */
typedef struct
{
	/* Power control */
#if (ASIC_NUM == ASIC1)
	gpio_t asic2_seq;			/* [out] ASIC2 Power enable sequence */
	gpio_t asic3_seq;			/* [out] ASIC3 Power enable sequence */

	/* PZT Power control */
	gpio_t pzt1_pwr_en;			/* [out] Piezo_VCC1_PWR_EN */
	gpio_t pzt2_pwr_en;			/* [out] Piezo_VCC2_PWR_EN */

	/* PZT Power select source */
	gpio_t pzt_cam_sel_70;		/* [out] PZT_CAM_SEL_70 */
	gpio_t pzt_cam_sel_150;		/* [out] PZT_CAM_SEL_150 */

	/* PZT Voltage control */
	voltage_ctrl_t pzt1_volt;
	voltage_ctrl_t pzt2_volt;

	/* Current measurement */
	curr_sensor_t vbat_curr;
	curr_sensor_t pzt1_curr;
	curr_sensor_t pzt2_curr;
#endif /* ASIC_NUM == ASIC1 */

	/* PZT Fail interrupt */
	gpio_t pzt_fail;			/* [in]  ASIC_PWM_AF_FAIL */
	void (*cb_pzt_fail)(void);
} power_ctrl_t;
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif /* __PWR_CTRL_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
