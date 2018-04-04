/**
  ******************************************************************************
  * \file    drv_piezo_hall.h
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_PIEZO_HALL_H
#define __DRV_PIEZO_HALL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "cam_ctrl.h"
#include <stdlib.h>

typedef struct HallSensorRec
{
    UInt8  IsInitialized; // Boolean, set to 1 in InitHallSensor
	UInt8  Polarity;	  // Really a boolean
	UInt8  Sensitivity;   // Valid values for AS5510: 0 (50mT), 1 (25mT), 2 (12.5mT), 3 (18.75mT)
	UInt16 Offset;

	UInt8  I2CSlaveAddress;
	UInt8  I2CExpanderChannel;
} HallSensor;

#ifdef BOARD_VERSION_P1
#define LENS_ADDR_WR			(0xac)	// AS5510 /0x56
#define LENS_ADDR_RD			(0xad)
#define MIRROR_ADDR_WR			(0xae)	// AS5510 /0x57
#define MIRROR_ADDR_RD			(0xaf)
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1
typedef struct lens_setup {
	int			mode;			// operating mode
	uint16_t	ontime;			// set ontime
	uint16_t	pause;			// set pause-time
	int			step;			// duty step
	int			speed;
	int			type;			// module type((0:af150, 1:af70)
	int			accuracy;		// poistion accuracy
	int			before;			// stop before
	long		repeat;			// repeat count for open-loop
} lens_setup_t;

#define LEND_ADDR				(0x0C)	// A1457
#define LENS_ADDR_WR			(0x18)
#define LENS_ADDR_RD			(0x19)

#define MIRROR_ADDR				(0x57)	// AS5510
#define MIRROR_ADDR_WR			(0xae)
#define MIRROR_ADDR_RD			(0xaf)
#endif /* BOARD_VERSION_P1_1 */
void   InitHallSensor(HallSensor *h);
int    CheckHallConnectivity(HallSensor *h); // Returns true if connected, false upon read error.
void   SetHallSensorSensitivity(HallSensor *h, UInt8 Sensitivity);
UInt16 ReadHallSensor(HallSensor *h);
void   PrintHallRetryStats(void);

#ifdef __cplusplus
}
#endif
#endif /* __DRV_PIEZO_HALL_H */