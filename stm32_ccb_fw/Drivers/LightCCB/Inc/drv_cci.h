/**
  ******************************************************************************
  * \file    drv_cci.h
  * \author  Infonam Embedded Team
  * \version V1.1.0
  * \date    05-Mar-2015
  * \brief   Header file of CCI driver related APIs
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_CCI_H
#define __DRV_CCI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "types.h"

/* Exported typedef ----------------------------------------------------------*/
typedef enum {
    I2C_IDLE = 0,
    I2C_SENDING_START,
    I2C_SENDING_ADDRESS,
    I2C_SENDING_DATA,
    I2C_RECEIVING_DATA,
    I2C_SENDING_RESTART,
	I2C_SLAVE_SEND,
	I2C_SLAVE_RECEIVE,
    I2C_ERROR,
} I2C_State_t;

typedef struct {
	UInt8 *data;
	UInt16 len;
	UInt16 nbytes;
} I2C_Msg_t;

typedef struct {
	I2C_State_t status;
	UInt8 address;
	I2C_Msg_t tx_msg;
	I2C_Msg_t rx_msg;
	Handler_t hdl_tranfer;
	Handler_t hdl_slave;
	Handler_Agr_t hdl_arg;
} I2C_t;

/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CCI_H */

 /*********** Portions COPYRIGHT 2015 Infonam. Co., Ltd.*****END OF FILE****/
