/**
  ******************************************************************************
  * \file    af_ctrl.h
  * \author  Infonam Embedded Team
  * \version V1.0.0
  * \date    25-May-2015
  * \brief   Header file of VCM module related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AF_CTRL_H
#define __AF_CTRL_H

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <types.h>
#include <stdlib.h>
#include <math.h>
#include "cam_ctrl.h"
#include "Interpolators.h"
#include "drv_piezo.h"

/* Exported typedef ----------------------------------------------------------*/
#define AF_MAX_DAC  		32767.0f /* 0x7fff */
#define AF_MIN_DAC			-32768.0f /* 0x8000 */
#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef CLAMP
#define CLAMP(a,min,max) MIN(MAX(a,min),max)
#endif
typedef struct
{
	UInt8  isValid; // Default is 0
	UInt16 EffectiveFocalLength; // In mm
	float  FocalLength; // In mm
	float  CropFactor;
	UInt32 FocusDistance; // In mm.

	// Calibration data
	UInt8  NumCalibrationPts;
	UInt16 CalibDistance[4]; // Calibration distances
	UInt16 CalibPosition[4];  // Calibration positions

	// Interpolation table
	InterpTable DacInterp;
	FloatPoint TableData[4]; // Storage for the interpolation table. 35mm uses 2 entries, 70 & 150 use 4.

	// AF state for this module.
	Bool IsOn;
	Int16 CurrentPosition; // This is a signed 16-bit number for the 35mm cameras.  Others may be different.
} AFData;

UInt8 InitAFInfo(CamDevice_TypeDef *cam);
UInt16 ComputeDacCode(AFData *Info);

Error_t SetFocusCalibration(PiezoModule *m, UInt32 FocusDistance, UInt16 FocusPosition);

#endif /* __AF_CTRL_H */
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE***********/
