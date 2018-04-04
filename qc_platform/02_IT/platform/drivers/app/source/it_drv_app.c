/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_drv_app.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    1-June-2016
 * @brief   This file contains expand of driver application
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 *  1.0.0	1-June-2016	Initial revision:
 */
/******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "it_drv_app.h"
#include "it_log_swapper.h"
#include "sys_ctrl.h"
/* Exported define -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NUM_PARAMS_MAX			    32
/* Private typedef -----------------------------------------------------------*/

/* Static variables ----------------------------------------------------------*/
LOCAL char HELP[] =
{
"**************************************************************************\r\n"
"               WELCOME TO LIGHT'S VERIFICATION APPLICATION\r\n"
"**************************************************************************\r\n"
"Catalog:\r\n"
"\t1> AR1335 module\r\n"
"\t2> CAMERA module\r\n"
"\t3> FLASH module\r\n"
"\t4> HALL_SENSOR module\r\n"
"\t5> I2C MASTER module\r\n"
"\t6> I2C SLAVE module\r\n"
"\t7> INA231 module.\r\n"
"\t8> RTC module\r\n"
"\t9> SPI SLAVE module\r\n"
"\t10> TEMPERATOR SENSOR module\r\n"
"\t11> TIMER module\r\n"
"\t12> TIMESTAMP module\r\n"
"\t13> TRACE_TIMER module\r\n"
"\t14> VCM module\r\n"
"**************************************************************************\r\n"
"Please select appropriating number for testing specific module\r\n"
"Select module: "
};

/* Static functions ----------------------------------------------------------*/
/* Exported global variables -------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int main(void)
{
	//! Initialize system
	sys_init();
	//! Print welcome
	printf(HELP);
	//! Standby
	while(1);
}
/**! Local functions **********************************************************/

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
