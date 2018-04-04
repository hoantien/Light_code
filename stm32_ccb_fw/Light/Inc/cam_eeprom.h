/**
  ******************************************************************************
  * \file    cam_eeprom.h
  * \author  Infonam Embedded Team
  * \version V1.1.0
  * \date    14-Jul-2015
  * \brief   Header file of cam_epprom related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __CAM_EEPROM_H
#define __CAM_EEPROM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "cam_ctrl.h"

/* Exported define -----------------------------------------------------------*/
#define CAM_EEPROM_NAME_STRING		"GT24C16"
#define CAM_EEPROM_SLAVE_ADDRESS	0xA0
#define CAM_EEPROM_SECTOR_NUM		8
#define CAM_EEPROM_SECTOR_SIZE		256
#define CAM_EEPROM_PAGE_SIZE		16
#define CAM_EEPROM_RAM_BASE			0x20028000	// Using the RAM memory 32KBytes lastest
/* Exported typedef ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern uint8_t *g_eeprom_buffer;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief cam_eeprom_read
 * The function read data from camera eeprom (GT26C16 EEPROM)
 * @param cam Pointer to camera module object
 * @param addr Eeprom address want to read data from
 * @param buf  Return buffer
 * @param size Size of data want to read from eeprom
 * @return ERROR_NONE: read success
 *		   ERROR_MEMORY_INVALID: Buffer invalid
 *		   ERROR_OUT_OF_MEMORY : Out of memory
 */
Error_t cam_eeprom_read (CamDevice_TypeDef *cam, UInt16 addr, UInt8* buf, UInt16 size);
/**
 * @brief cam_eeprom_write
 * The function write data to camera eeprom (GT26C16 EEPROM)
 * @param cam Pointer to camera module object
 * @param addr Eeprom address want to read data from
 * @param buf  Return buffer
 * @param size Size of data want to read from eeprom
 * @return ERROR_NONE: read success
 *		   ERROR_MEMORY_INVALID: Buffer invalid
 *		   ERROR_OUT_OF_MEMORY : Out of memory
 */
Error_t cam_eeprom_write(CamDevice_TypeDef *cam, UInt16 addr, UInt8* buf, UInt16 size);

/**
 * @brief vCamEepromTest
 * The function test write/read full data from camera B3 eeprom
 * @param  pvParameters Parameters
 */
void vCamEepromTest(void *pvParameters);
void mem_dump(UInt8 *buf, UInt16 size);
#ifdef __cplusplus
}
#endif

#endif /* __CAM_EEPROM_H */

 /*********** Portions COPYRIGHT 2015 Light. Co., Ltd.*******END OF FILE*******/
