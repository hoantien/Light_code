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
#ifndef __DRV_PIEZO_H
#define __DRV_PIEZO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "cam_ctrl.h"
#include "drv_piezo_hall.h"
#include <stdlib.h>

/* Exported typedef ----------------------------------------------------------*/

/**
 * \brief  AFM structure. TODO: Add further description
 */
typedef enum {
	USM_IDLE = 0,
	USM_RUN,
} piezo_status;

/**
 * \brief   structure. TODO: Add further description
 */
typedef enum {
	LEN = 0,
	MIRROR,
} actuator;


typedef enum
{
	ActuatorType_None,
	ActuatorType_Lens,
	ActuatorType_Mirror,
	ActuatorType_Max
} ActuatorType;


// Calibration data for an actuator
#define MAX_PIEZO_CALIB_POINTS 4
typedef struct PiezoCalibrationDataRec
{
	// Data derived from the calibration data.
	float  DerivedFocalLength;     // Focal length of the lens (mm)
	float  DerivedHardStopOffset;  // Distance from sensor to hard stop (mm)
	float  DerivedMirrorDistance;  // Distance from sensor to mirror (mm)
	float  DerivedDistanceScale;   // Scale factor from calibrated distance to mm

	UInt32 Frequency;

	UInt16 Resolution;
	UInt16 Offset;
	UInt16 NearPosition; // ADC code from the Hall sensor
	UInt16 FarPosition; // ADC code from the Hall sensor
	UInt16 Distance[MAX_PIEZO_CALIB_POINTS];
	UInt16 Position[MAX_PIEZO_CALIB_POINTS];
	UInt8  Sensitivity; // 2 bits worth (format?)
	UInt8  Polarity;
	UInt8  NumCalibPts;
	bool   IsValid;     // Initially 0. Supports lazy init.
} PiezoCalibrationData;

typedef struct
{
	PiezoCalibrationData calib_data;
	uint32_t calib_header;
	uint16_t position;
} hall_calib_t;

typedef enum
{
	LENS_CALIBRATION = 0x1,
	MIRROR_CALIBRATION,
}CALIBRATION_TYPE;

typedef struct PiezoActuatorDataRec
{
	UInt8  FlipDirection; // 0 = no flip, 1 = reverse NEAR & FAR (for mirrors)

	UInt16 FullStroke;   // um
	UInt16 RealStroke;   // um
	UInt16 MinStroke;    // um
	UInt16 HomePosition; // um
} PiezoActuatorData;

// Channel data
typedef struct PiezoChannelRec
{
	UInt8         ChannelNumber; // For debug purposes
	GPIO_TypeDef *Port;
	UInt16        Pin;
} PiezoChannel;

typedef struct PiezoActuatorRec
{
	ActuatorType          Type;
	PiezoActuatorData     ActuatorData;
	PiezoChannel         *Channel;
	PiezoCalibrationData  CalibData;
	HallSensor            Hall;
	UInt8                 CpldRegister; // The register in the CPLD for this actuator.
	UInt8                 CpldBitMask;  // The bit within the register for this actuator.
	UInt8                 IsConnected;  // Really a boolean
} PiezoActuator;

// PiezoModule: The camera-module portion, likely to be refactored into a more global camera
//              module structure.
//              TBD: Do we need channel refs here?
//              Note that the calibration status is at the module level as mechanical
//              interactions require the coordination of calibration of both actuators in a module.
typedef struct PiezoModuleRec
{
	bool           IsCalibrated; // True if the calibration data is valid.
	UInt8          CameraId;
	PiezoActuator *Lens;
	PiezoActuator *Mirror;
} PiezoModule;

#define NUM_PIEZO_ACTUATORS 22
#define NUM_PIEZO_MODULES   11
#define NUM_PIEZO_CHANNELS  11

#define CHANNEL_DATA(n) { n, PWM_AF_EN##n##_GPIO_PORT, PWM_AF_EN##n##_GPIO_PIN }

/* Exported define -----------------------------------------------------------*/

#define SS_EN 		GPIO_ResetBits(CPLD_SS_GPIO_PORT, CPLD_SS_PIN)
#define SS_DIS  	GPIO_SetBits(CPLD_SS_GPIO_PORT, CPLD_SS_PIN)

// Driving Freq.
#define FREQL		128000			// lens freq.
#define FREQM		135000			// mirror freq.
#define FREQ_HI		150000			//
#define FREQ_LO		40000			//

#define DUTY		245
#define DUTY_LO		100
#define DUTY_HI		280				//
#ifdef BOARD_VERSION_P1
#define LENS_ADDR_WR			(0xac)	// AS5510 /0x56
#define LENS_ADDR_RD			(0xad)
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1
#define LENS_ADDR_WR			(0x18)	// A1457  /0x0C
#define LENS_ADDR_RD			(0x19)
#endif /* BOARD_VERSION_P1_1 */
#define MIRROR_ADDR_WR			(0xae)	// AS5510 /0x57
#define MIRROR_ADDR_RD			(0xaf)

#define AXIS_NO		22		// Number of actuator module!!!!!
// DIRECTION
#define FAR			(0)
#define NEAR		(1)

#define	USM_IDLE	0
#define	USM_RUN		1
#define STOPBEFORE	10		// stop motor before several "unit sensitivity" from the destination position
#define TUNETIME	750		// on-time when tuning calibration

#define KP	2.5
#define KI	0.15
#define KD	0.05

#define F_PID	500  // Frequency PID

#define FLASH_CALIB_DATA_START		0x00500000
#define FLASH_CALIB_DATA_HDR_SZ		0x4						//in bytes
//#define	FLASH_CALIB_HDR				0x10000001
#define	FLASH_CALIB_HDR				0xdeadbeef
#define SPI_FLASH_SUB_SEC_SIZE		0x1000

/* The PWM generate topology
P1: 	PWM generate by CPLD 	(STM_PWM = 0)
P1.1: 	PWM generate by STM32	(STM_PWM = 1)
*/
#if defined BOARD_VERSION_P1
  #define STM_PWM		0
#elif defined BOARD_VERSION_P1_1
// #define STM_PWM		1
  #define STM_PWM		2	/* Temporary set STM_PWM = 2 for May 5,2016 release
							This macro should be set to 1 for next release */
#endif

#define CAM_LENS_CALIBRATION_LOC(cam_id)	FLASH_CALIB_DATA_START + \
		(2 * (cam_id) * sizeof(hall_calib_t))
#define CAM_MIRROR_CALIBRATION_LOC(cam_id)	FLASH_CALIB_DATA_START +\
		((2 * (cam_id)) + 1) * sizeof(hall_calib_t)

#define m_offset(obj, m) ((size_t)(&((obj *)0)->m))

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/*
* TODO: Add function description
*/
void AF_control_Config(void);

/*
* TODO: Add function description
*/
Bool goto_macro(CamID camera_id, actuator act);

/*
* TODO: Add function description
*/
Bool goto_infinity(CamID camera_id, actuator act);

/*
* TODO: Add function description
*/
Bool goto_potsition(CamID camera_id, actuator act, uint16_t mm);

/*
* TODO: Add function description
*/
uint16_t get_position(CamID camera_id, actuator act);

/*
* TODO: Add function description
*/
uint16_t calibrate(CamDevice_TypeDef *camera, actuator act, UInt16 *min, UInt16 *max);

Error_t MovePiezoToPosition(PiezoActuator *a, UInt16 TargetPosition, UInt16 Tolerance, UInt16 DutyCycle);

// This is the internal helper function.
Error_t PiezoMoveToPosition(PiezoActuator *a, UInt16 TargetPosition, UInt16 Tolerance, UInt16 DutyCycle);

// Turn the PiezoMonitor task on & off.
void SetPiezoMonitor(int on, unsigned int bitmask);

void InitPiezoHw(PiezoChannel *Channels);

PiezoModule* CameraIdToModule(UInt8 CameraId);
Error_t CalibrateModule(PiezoModule *m, UInt16 Iterations, UInt16 DutyCycle);

/* Read the Hall sensor calibration data from the EEPROM. */
Error_t ReadHallCalibration(CamDevice_TypeDef *pCam, int CameraId, PiezoCalibrationData *LensData, PiezoCalibrationData *MirrorData);

/* Read calibration data from the EEPROM */
Error_t ReadEepromCalibration(int CameraId, PiezoCalibrationData *LensData, PiezoCalibrationData *MirrorData);

// Internal
void SetCurrentActuator(PiezoActuator *a);

int hall_calibration_save(CamDevice_TypeDef *pCam, CALIBRATION_TYPE type);

int hall_calibration_update_position(CamDevice_TypeDef *pCam, CALIBRATION_TYPE type);

int hall_calibration_read(CamDevice_TypeDef *pCam, hall_calib_t * lens_calib,
		hall_calib_t * mirror_calib);

void start_lens_calibration(int CameraId, 
                            CamDevice_TypeDef* pCam, 
                            PiezoActuator* lens, 
                            PiezoCalibrationData* LensData, 
                            hall_calib_t lens_calib);

void calibrate_lens  (CamDevice_TypeDef* pCam, PiezoActuator* actuator);

void calibrate_mirror(CamDevice_TypeDef* pCam, PiezoActuator* actuator);

Bool start_closed_control_lens(CamDevice_TypeDef* pCam,
								PiezoActuator* actuator,
								uint16_t CPLD_select,
								uint16_t destination,
								uint8_t is_calibration, uint8_t tolerance);
int start_closed_control_mirror(CamDevice_TypeDef* pCam,
								PiezoActuator* actuator,
								uint16_t CPLD_select,
								uint16_t destination,
								uint8_t is_calibration);

void start_PWM_control(PiezoActuator* actuator, uint16_t CPLD_select, uint8_t is_lens, uint32_t delay);
#ifdef __cplusplus
}
#endif
#endif /* __DRV_PIEZO_H */
