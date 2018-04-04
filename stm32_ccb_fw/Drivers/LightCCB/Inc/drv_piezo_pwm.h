/*
 * drv_piezo_pwm.h
 *
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 */

#ifndef __DRV_PIEZO_PWM_H
#define __DRV_PIEZO_PWM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "errors.h"
#include "drv_piezo.h"

/* Private typedef -----------------------------------------------------------*/

// Data structure for a PWM device.
// For the P1 CCB, there is a single PWM source that uses the PWM channel on the
// STM + the CPLD.
typedef struct PiezoPwmRec
{
	UInt8  Active;        // Boolean.  Indicates if the PWM is on.
	UInt8  Direction;     // FAR = 0, NEAR = 1 (see #defines - enum TODO)
	UInt8  FlipDirection; // 0 = no flip, 1 = flip meaning of NEAR & FAR
	UInt32 Frequency;     // Units of Hz
	UInt16 DutyCycle;     // Units of 0.1%
	UInt8  TimerChannel;
} PiezoPwm;

// PWM Routines.
// TODO: Add docs.

void InitPwm(void);

Error_t
ConfigurePwm(PiezoPwm      *pwm,
			 PiezoActuator *a,
			 UInt16         DutyCycle,
			 UInt8          Direction,
			 UInt8          Active);

Error_t TurnOnPwm (PiezoPwm *pwm, PiezoActuator *a);
Error_t TurnOffPwm(PiezoPwm *pwm, PiezoActuator *a);

UInt32 GetPulseLength(PiezoPwm *pwm);
UInt32 GetTimePeriod(PiezoPwm *pwm);
void   DelayUSec(UInt32 usec);

#ifdef __cplusplus
}
#endif
#endif // #ifndef __DRV_PIEZO_PWM_H
