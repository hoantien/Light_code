/**
  ******************************************************************************
  * \file    drv_i2c_ex.h
  * \author  Infonam Embedded Team
  * \version V1.1.0
  * \date    05-Mar-2015
  * \brief   Header file of I2C driver related APIs
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CCB_I2C_EX_H
#define __CCB_I2C_EX_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "errors.h"
#include "platform.h"

/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define I2CEx_SLAVE1_ADDRESS				(0x72<<1)
#define I2CEx_SLAVE2_ADDRESS				(0x71<<1)
/* Exported functions ------------------------------------------------------- */

/**
 *  \brief  This function handles I2CEx Error interrupt request.
 */
void I2CEx_ER_IRQHandler(void);

/**
 *  \brief  This function handles I2CEx event interrupt request.
 */
void I2CEx_EV_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __CCB_I2C_EX_H */

 /*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE***********/
