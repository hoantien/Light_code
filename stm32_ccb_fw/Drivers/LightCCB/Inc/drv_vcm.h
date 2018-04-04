/**
  ******************************************************************************
  * \file    drv_vcm.h
  * \author  Infonam Embedded Team
  * \version V1.0.0
  * \date    25-May-2015
  * \brief   Header file of VCM module related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_VCM_H
#define __DRV_VCM_H

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <types.h>
#include <stdlib.h>
#include <math.h>
#include "cam_ctrl.h"
#include "Interpolators.h"

#include "af_ctrl.h"

/* Exported define -----------------------------------------------------------*/
#ifdef BOARD_VERSION_P1
#define EEPROM_SLAVE_ADDR	0xA0
#define AF_TARGET_POS_REG	0xA1
#define AF_DLYCLR			0x85
#define EEPROM_AF_CLOSELOOP_VCM_BIAS	0x11
#define EEPROM_AF_CLOSELOOP_VCM_OFFSET	0x12
#define EEPROM_AF_POS_MACRO 0x13
#define EEPROM_AF_POS_INF   0x1f
#define EEPROM_AF_DST_MACRO 0x21
#define EEPROM_AF_DST_INF   0x2d
#else /* BOARD_VERSION_P1_1 */
#define AF_CHTGTIM_USEC		171
#define AF_CHTGLOC			0xC4
#define AF_CHTGTIM			0xCB
#define AF_CHTGN			0x94
#define AFIC_EEPROM_ADDR	0xE6
#define AF_PIDZO			0x02
#define AF_EQENBL			0x87
#define AF_STMVENDH_REG		0xA0
#define AF_STMVENDL_REG		0xA1
#define AF_STMVEN			0x8A
#define AF_STMVEN_STMVEN	(0x1 << 0)
#define AF_STMVEN_STMFST	(0x1 << 4)
#define AF_STMVEN_STMCHTGST	(0x1 << 3)
#define AF_MSSET			0x8F
#define AF_MSSET_CHTGST		(0x1 << 0)
#define AF_CVER				0xF0
#define AF_AWDLCTRL			0xE0
#define EEPROM_AF_CLOSELOOP_VCM_BIAS	0x23
#define EEPROM_AF_CLOSELOOP_VCM_OFFSET	0x24
#define EEPROM_AF_POS_MACRO	0x25
#define EEPROM_AF_POS_INF	0x27
#define EEPROM_AF_DST_MACRO	0x2B
#define EEPROM_AF_DST_INF	0x2D
#endif

#define CHANNEL_ID_A1		1
#define CHANNEL_ID_A5		5
#define MAX_SUPPORTED_CAM	5
#define AF_SLAVE_ADDR 		(0x72<<1)
#define AF_STANDBY_REG 		0x84
#define AF_DIRECTION_REG	0x16

#define MAX_TRY_CNT 		5
#define MOVE_TO_INFINITY	0x0180
#define MOVE_TO_MACRO		0xfe80
#define AF_CROP_FACTOR		7.61
#define CAM_EEPROM_SLVADDR	0xA0

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

UInt8 af_controller_read_eeprom(UInt8 addr,  UInt8 ch_id);

/**
 *  \brief Function to initialize a VCM module
 *
 *  \par Header File:
 *  af_ctl.h
 *
 *  \par Parameters:
 *  \param pCam (in,out) Pointer to camera device structure
 *  \retval ETrue The function executed successfully.
 *  \retval EFalse The function executed unsuccessfully.
 *  \par Description:
 *  This function selects a VCM module at a specified I2C channel and initialize it
 */
Bool af_controller_init(CamDevice_TypeDef *pCam);

/**
 *  \brief Function to move a VCM module to a specified target point or distance
 *
 *  \par Header File:
 *  af_ctl.h
 *
 *  \par Parameters:
 *  \param pCam (in,out) Pointer to camera device structure
 *  \param wPos (in) the user-input target point to move to
 *  \param isRealPos (in) whether wPos is real distance in centimeter unit
 *  \retval ETrue The function executed successfully.
 *  \retval EFalse The function executed unsuccessfully.
 *  \par Description:
 *  This function selects a VCM module at a specified I2C channel and move it
 *  to a specified target point or distance
 */
Bool af_controller_move(CamDevice_TypeDef *pCam,UInt16 wPos, Bool isRealPos);

/**
 *  \brief Function to stop a VCM module
 *
 *  \par Header File:
 *  af_ctl.h
 *
 *  \par Parameters:
 *  \param pCam (in,out) Pointer to camera device structure
 *  \retval ETrue The function executed successfully.
 *  \retval EFalse The function executed unsuccessfully.
 *  \par Description:
 *  This function to stop moving VCM module and stop VCM module operation
 */
Bool af_controller_stop(CamDevice_TypeDef *pCam);


UInt16 af_controller_get_current_pos(AFData *af, UInt8 ch_id);

#endif /* __DRV_VCM_H */
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE***********/
