/**
  ******************************************************************************
  * \file    hal_cci.h
  * \author  Infonam Embedded Team
  * \version V1.0.0
  * \date    05-Mar-2015
  * \brief   Header file of CCI Hardware Abstract Layer related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_CCI_H
#define __HAL_CCI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "errors.h"
#include "platform.h"

/* Exported typedef ----------------------------------------------------------*/
typedef I2C_TypeDef CCI_Reg_TypeDef;

/**
 * \brief  CCI device enumeration
 */
typedef enum {
	CCI0 = 0,
	CCI1,
} CCI_TypeDef;

/**
 * \brief  CCI State enumeration
 */
typedef enum {
	CCI_STATE_RESET         = 0x00,  /**< CCI not yet initialized or disabled         */
	CCI_STATE_READY         = 0x01,  /**< CCI initialized and ready for use           */
	CCI_STATE_BUSY          = 0x02,  /**< CCI internal process is ongoing             */
	CCI_STATE_BUSY_TX       = 0x12,  /**< Data Transmission process is ongoing        */
	CCI_STATE_BUSY_RX       = 0x22,  /**< Data Reception process is ongoing           */
	CCI_STATE_MEM_BUSY_TX   = 0x32,  /**< Memory Data Transmission process is ongoing */
	CCI_STATE_MEM_BUSY_RX   = 0x42,  /**< Memory Data Reception process is ongoing    */
	CCI_STATE_TIMEOUT       = 0x03,  /**< CCI timeout state                           */
	CCI_STATE_ERROR         = 0x04   /**< CCI error state                             */
} CCI_State_TypeDef;

/**
 * \brief CCI Error Code enumeration
 */
typedef enum {
	CCI_ERROR_NONE      = 0x00,    /**< No error             */
	CCI_ERROR_BERR      = 0x01,    /**< BERR error           */
	CCI_ERROR_ARLO      = 0x02,    /**< ARLO error           */
	CCI_ERROR_AF        = 0x04,    /**< AF error             */
	CCI_ERROR_OVR       = 0x08,    /**< OVR error            */
	CCI_ERROR_TIMEOUT   = 0x20     /**< Timeout error        */
} CCI_Error_TypeDef;

/**
 * \brief CCI Xfer buffer structure
 */
typedef struct {
	UInt16 cmd;
	UInt8 *data;
	UInt8 len;
	UInt8 flags;
	UInt16 num_rx_bytes;
} CCI_Buffer;

/**
 * \brief CCI handler structure
 */
typedef struct CCI_Handler_TypeDef {
	UInt8 *pXferBuff;
	UInt16 XferSize;
	__IO UInt16 XferCount;
	__IO CCI_State_TypeDef State;
	__IO CCI_Error_TypeDef ErrorCode;
	void (*TxCpltCallback)(void);
	void (*RxCpltCallback)(void);
	void (*ErroCallback)(void);
} CCI_Handler_TypeDef;

/* Exported define -----------------------------------------------------------*/
#define hCCI0	CCI_Handler[CCI0]
#define hCCI1	CCI_Handler[CCI1]

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern __IO uint16_t ReceiveCmd;
extern __IO uint16_t ReceiveTid;
extern __IO uint16_t ReceiveDataSize;
extern __IO uint16_t ReceivedFlags;
extern CCI_Handler_TypeDef *CCI_Handler[];
/* Exported functions ------------------------------------------------------- */

/*
 * TODO: Remove duplication to the function definition in drv_cci.h
 */
void CCB_CCI0_Init(void);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
void CCB_CCI1_Init(void);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
void HAL_CCI_Init(CCI_TypeDef CCIx);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
void HAL_CCI_DeInit(CCI_TypeDef CCIx);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
void HAL_CCI_Gpio_Init(CCI_TypeDef CCIx);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
void HAL_CCI_Gpio_DeInit(CCI_TypeDef CCIx);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
Error_t HAL_CCI_Transmit(CCI_Handler_TypeDef *CCIx, UInt8 *data, UInt16 size);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
Error_t HAL_CCI_Receive(CCI_Handler_TypeDef *CCIx, UInt8 *data, UInt16 size);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
void HAL_CCI_TxCpltCallback(CCI_Handler_TypeDef *CCIx);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
void HAL_CCI_RxCpltCallback(CCI_Handler_TypeDef *CCIx);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
void HAL_CCI_ErrorCallback(CCI_Handler_TypeDef *CCIx);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
CCI_State_TypeDef HAL_CCI_GetState(CCI_Handler_TypeDef *CCIx);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
UInt32 HAL_CCI_GetError(CCI_Handler_TypeDef *CCIx);

#ifdef __cplusplus
}
#endif

#endif /* __HAL_CCI_H */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*******END OF FILE*********/
