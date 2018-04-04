/**
  ******************************************************************************
  * \file    ucid.h
  * \author  Infonam Embedded Team
  * \version V1.0.0
  * \date    26-May-2015
  * \brief   This file provides functionalities LIGHT_ACTIVE_UCID
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UCID_H
#define __UCID_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <types.h>
#include "cam_ctrl.h"
#include "af_ctrl.h"
#include "drv_vcm.h"
#include "drv_piezo_move.h"
/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define MAX_USECASE 16

typedef enum {
	UC_DISABLED = 0x0,
	UC_UNKNOWN = 0x1,
	UC_DEBUG = 0x2,
	UC_PREVIEW = 0x3,
	UC_HIRES_CAPTURE = 0x5,
	UC_FOCAL_STACKING = 0x6,
	UC_HDR_CAPTURE = 0x7,
	UC_VIDEO = 0x4,
	UC_FTM_QUICK_CHECK = 0x9,
	UC_FTM_CALIBRATION = 0xa
} UCID;

typedef enum {
	BURST_REQUESTED     = (UInt16)(1<<0),
	RESOLUTION          = (UInt16)(1<<1),
	SENSITIVITY         = (UInt16)(1<<2),
	EXPOSURE_TIME       = (UInt16)(1<<3),
	FOCAL_LEN           = (UInt16)(1<<4),
	FOCUS_DISTANCE      = (UInt16)(1<<5),
	FOCUS_STATUS        = (UInt16)(1<<6),
	FPS                 = (UInt16)(1<<7),
	VCM_POSITION        = (UInt16)(1<<8),
	VCM_HALL            = (UInt16)(1<<9),
	LENS_POSITION       = (UInt16)(1<<10),
	LENS_HALL           = (UInt16)(1<<11),
	MIRROR_POSITION     = (UInt16)(1<<12),
	MIRROR_HALL         = (UInt16)(1<<13),
	MAX_ELEMENT         = 14
} ELEMENT;

typedef struct {
	UInt8 cam_idx;
	UInt16 element;
	UInt64 element_value;
} ElementSetting;

typedef struct {
	UInt16 numofelement;
	UInt16 ucid;
	ElementSetting settings[MAX_ELEMENT*CAM_NUM_CAMERA_MAX];
} UCSetting;

typedef struct {
	UInt16 numofusecase;
	UInt16 active_ucid;
	UInt16 is_activated;
	UCSetting ucids[MAX_USECASE];
} UseCaseManager;

#define ES_SIZE 4
#define UCS_SIZE 1024
#define UCM_SIZE 16390
#define STORAGE_SIZE sizeof(UseCaseManager)
#define STORAGE_ADDRESS 0x0
#define STORAGE_SUBSECTOR0_ADDRESS 0x00000000
#define STORAGE_SUBSECTOR1_ADDRESS 0x00001000
#define STORAGE_SUBSECTOR2_ADDRESS 0x00002000
#define STORAGE_SUBSECTOR3_ADDRESS 0x00003000
#define STORAGE_SUBSECTOR4_ADDRESS 0x00004000
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 *  \brief Function to add value for a specified setting element
 *
 *  \par Header File:
 *  ucid.h
 *
 *  \par Parameters:
 *  \param element (in) a specified element to add
 *  \param value (in) value of the element
 *  \param cam_idx (in) the camera index in CamDeviceTbl
 *  \param ucid (in) intended use-case id for this setting
 *  \retval ETrue The function executed successfully.
 *  \retval EFalse The function executed unsuccessfully.
 *  \par Description:
 *  This function is to add value for a specified setting element
 */
Bool add_setting(ELEMENT element, UInt64 value, UInt8 cam_idx, UInt16 ucid);

/**
 *  \brief Function to get active ucid
 *
 *  \par Header File:
 *  ucid.h
 *
 *  \par Parameters:
 *  \retval 0 no ucid is activated
 *  \retval >0 current active ucid
 *  \par Description:
 *  This function is to get current manipulation ucid
 */
UInt16 get_active_ucid(void);

/**
 *  \brief Function to activate a given ucid
 *
 *  \par Header File:
 *  ucid.h
 *
 *  \par Parameters:
 *  \param ucid (in) use-case id is being activated
 *  \retval ETrue The function executed successfully.
 *  \retval EFalse The function executed unsuccessfully.
 *  \par Description:
 *  This function is to activate a ucid
 */
Bool activate_ucid(UInt16 ucid);

/**
 *  \brief Function to check whether a use case has been activated
 *
 *  \par Header File:
 *  ucid.h
 *
 *  \par Parameters:
 *  \param none
 *  \retval 1: the active use case has been activated
 *  \retval 0: the active use case has not been activated
 *  \par Description:
 *  This function is to get the activated status of active use case
 */
UInt16 is_active_ucid_activated(void);

/**
 *  \brief Function to save all use-case to flash
 *
 *  \par Header File:
 *  ucid.h
 *
 *  \par Parameters:
 *  \retval ETrue The function executed successfully.
 *  \retval EFalse The function executed unsuccessfully.
 *  \par Description:
 *  This function is to save all use-case to flash
 */
Bool save_all_usecase(void);

/**
 *  \brief Function to restore all use-case from flash
 *
 *  \par Header File:
 *  ucid.h
 *
 *  \par Parameters:
 *  \retval ETrue The function executed successfully.
 *  \retval EFalse The function executed unsuccessfully.
 *  \par Description:
 *  This function is to restore all use-case from flash
 */
Bool initialize_usecase_manager(void);

/**
 *  \brief Function to clone setting from a use-case to another use-case
 *
 *  \par Header File:
 *  ucid.h
 *
 *  \par Parameters:
 *  \param src_ucid (in) use-case id of the source use-case
 *  \param dst_ucid (in) use-case id of the destination use-case
 *  \retval ETrue The function executed successfully.
 *  \retval EFalse The function executed unsuccessfully.
 *  \par Description:
 *  This function is to support intelligent mode of ucid manager.
 *  It can be use to clone setting from a use-case to another use-case
 *  For example: clone setting in PREVIEW use-case to HIRES use-case
 */
Bool inherit_setting(UInt16 src_ucid, UInt16 dst_ucid);

char * element_to_string( ELEMENT element);

char* ucid_to_text(UInt16 ucid);
UInt8 get_last_setting(UInt8 cam_idx, UInt16 ucid, UInt16 element, UInt64 *value);

#ifdef __TEST_UCID
void dumpData(UseCaseManager *ucm);
#endif

#endif //__UCID_H

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE***********/
