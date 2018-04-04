/**
  ******************************************************************************
  * \file    drv_piezo.h
  * \author  Infonam Embedded Team
  * \version V1.1.0
  * \date    05-Mar-2015
  * \brief   Header file of PIEZO driver related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_PIEZO_MOVE_H
#define __DRV_PIEZO_MOVE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "cam_ctrl.h"
#include <stdlib.h>

/* Exported typedef ----------------------------------------------------------*/


Error_t PiezoNudge(PiezoActuator *a, UInt8 Direction, UInt16 DutyCycle, UInt8 RepeatCount);

Error_t PiezoMoveToPosition(PiezoActuator *a, UInt16 TargetPosition, UInt16 Tolerance, UInt16 DutyCycle);

Error_t PiezoMoveToDistance(PiezoActuator *a, UInt32 TargetDistance);

#ifdef __cplusplus
}
#endif
#endif /* __DRV_PIEZO_MOVE_H */
