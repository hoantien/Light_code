/**
  ******************************************************************************
  * \file    drv_cpld.h
  * \author  Infonam Embedded Team
  * \version V1.1.0
  * \date    04-Dec-2015
  * \brief   Header file of CCI driver related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_CPLD_H
#define __DRV_CPLD_H
/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "types.h"
#include "mems.h"
/* Exported functions ------------------------------------------------------- */
/**
 *  \brief  This function return the major and minor version of CPLD
 */
void cpld_get_version(volatile uint8_t *major, volatile uint8_t *minor);

#endif /*__DRV_CPLD_H*/
