/**
  ******************************************************************************
  * \file    hal_i2c_ex.h
  * \author  Infonam Embedded Team
  * \version V1.1.0
  * \date    05-Mar-2015
  * \brief   Header file of I2C Expander Hardware Abstract Layer related APIs
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_I2C_EX_H
#define __HAL_I2C_EX_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "errors.h"

/* Exported typedef ----------------------------------------------------------*/

/**
 * \brief  Enumeration of devices connected to I2CEx
 */
typedef enum {
	I2CEx_PZT0 = 0,
	I2CEx_PZT1,
	I2CEx_PZT2,
	I2CEx_PZT3,
	I2CEx_PZT4,
	I2CEx_PZT5,
	I2CEx_PZT6,
	I2CEx_PZT7,
	I2CEx_PZT8,
	I2CEx_PZT9,
	I2CEx_PZT10,
	I2CEx_TEMP,
	I2CEx_CPLD,
	I2CEx_SCLK,
#ifdef STM_CCB
	I2CEx_CAM1,
	I2CEx_CAM2,
#endif
}I2CEx_Ch_t;

/**
 * \brief  I2CEx buffer structure
 */
typedef struct {
	UInt8 *data;
	UInt32 len;
} I2CEx_Buf_t;

/**
 * \brief  I2CEx message structure
 */
typedef struct {
	I2CEx_Buf_t txbuf;
	I2CEx_Buf_t rxbuf;
	I2CEx_Ch_t ch;
	UInt8 addr;
	__IO Int8 *is_completed;
} I2CEx_Msg_t;

/* Exported define -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 *  \brief Function to initialize I2C expander (I2CEx).
 *
 *  \par Header File:
 *  hal_i2c_ex.h
 *
 *  \par Description:
 *  This function is used to configure I2CEx and start a task to handle I2C data
 *  transmission
 */
void HAL_I2CEx_Init(void);

/**
 *  \brief Function to transfer I2CEx message
 *
 *  \par Header File:
 *  hal_i2c_ex.h
 *
 *  \param msg (in) Pointer to I2CEx message to be sent
 *  \retval \c ERROR_NONE The function executed successfully.
 *  \retval \c ERROR_MEMORY_INVALID Memory transfer invalid
 *  \retval \c ERROR_OUT_OF_MEMORY Not enough storage
 */
Error_t HAL_I2CEx_Transfer(I2CEx_Msg_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* __HAL_I2C_EX_H */

 /*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE***********/
