/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_ddr.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    2-Aug, 2016
 * @brief   This file contains expand of the hal_ddr driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	2-Aug-2016	Initial revision:
 * * 1.0.1 10-Aug-2016  Implement test cases LPDDR3
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _IT_HAL_DDR_H_
#define _IT_HAL_DDR_H_
/* Includes ------------------------------------------------------------------*/
#include "hal_ddr.h"
#include "qc_assert.h"
#include "it_log_swapper.h"
/* Exported typedef ----------------------------------------------------------*/

/* Exported define -----------------------------------------------------------*/
#define     base_SCU             0x02000000
#define     base_DDRCs           0x03700000
#define     base_ddrp0           0x02036000
#define     base_ddrp1           0x02037000

#define DCR 0x0C*0x4
#define DTPR0 0x0D*0x4
#define DTPR1 0x0E*0x4
#define DTPR2 0x0F*0x4
#define MR0 0x10*0x4
#define MR1 0x11*0x4
#define MR2 0x12*0x4
#define MR3 0x13*0x4
#define DTPR0 0x0D*0x4
#define DTPR1 0x0E*0x4
#define DTPR2 0x0F*0x4
#define PGCR 0x02*0x4
#define DXCCR 0x0A*0x4
#define DSGCR 0x0B*0x4
#define DX0DQSTR 0x75*0x4
#define DX1DQSTR 0x85*0x4
#define DX2DQSTR 0x95*0x4
#define DX3DQSTR 0xa5*0x4
#define PGSR 0x03*0x4
#define PIR 0x01*0x4

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief DDR module's testing handler
 * @detail to DDR module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	: success
 *         Others: instruction failed
 */
IMPORT int it_hal_ddr_handler(char** argv, int argc);

#endif /**! _IT_HAL_DDR_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
