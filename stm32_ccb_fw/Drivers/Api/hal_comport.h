/**
  ******************************************************************************
  * \file    hal_comport.h
  * \author  Infonam Embedded Team
  * \version V1.0.0
  * \date    05-Mar-2015
  * \brief   Header file of COM Port Hardware Abstract Layer related APIs
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_COMPORT_H
#define __HAL_COMPORT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "errors.h"

/* Exported typedef ----------------------------------------------------------*/
/**
 * \brief  COM Port enumeration
 */
typedef enum {
	COM1 = 0,
	COM2 = 1,
} COM_Port_t;

/* Exported define -----------------------------------------------------------*/
#define COM_FLAG_TXE	USART_FLAG_TXE	/**< Transmit data register empty flag 		*/
#define COM_FLAG_TC		USART_FLAG_TC	/**< Transmission Complete flag 			*/
#define COM_FLAG_RXNE	USART_FLAG_RXNE	/**< Receive data register not empty flag 	*/
#define COM_FLAG_IDLE	USART_FLAG_IDLE	/**< Idle Line detection flag 				*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 *  \brief Function to configures COM port.
 *
 *  \par Header File:
 *  hal_comport.h
 *
 *  \par Parameters:
 *  \param COMx (in) Specifies the COM port to be configured.
 *  \arg \c COM1
 *  \arg \c COM2
 */
void HAL_COM_Init(COM_Port_t COMx);

/**
 *  \brief Function to send data byte to COM port device.
 *
 *  \par Header File:
 *  hal_comport.h
 *
 *  \par Parameters:
 *  \param COMx (in) Specifies the COM port to send data.
 *  \arg \c COM1
 *  \arg \c COM2
 *  \param data (in) Data to be sent in one byte.
 */
void HAL_COM_SendData(COM_Port_t COMx, UInt8 data);

/**
 *  \brief Function to get flag status of a COM port device.
 *
 *  \par Header File:
 *  hal_comport.h
 *
 *  \par Parameters:
 *  \param COMx (in) Specifies the COM port to get flag status.
 *  \arg \c COM1
 *  \arg \c COM2
 *  \param flag (in) Flag to get status.
 *  \arg \c COM_FLAG_TXE
 *  \arg \c COM_FLAG_TC
 *  \arg \c COM_FLAG_RXNE
 *  \arg \c COM_FLAG_IDLE
 *  \return \c FlagStatus Flag status of the specified COM port
 */
FlagStatus HAL_COM_GetFlagStatus(COM_Port_t COMx, UInt16 flag);

#ifdef __cplusplus
}
#endif

#endif /* __HAL_COMPORT_H */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*******END OF FILE*********/
