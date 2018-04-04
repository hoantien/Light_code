#include "af_ctrl.h"
#include "mems.h"
#include "CCBConfig.h"
#include "FreeRTOS.h"
#include "log.h"
#include "task.h"
#include "hal_i2c_ex.h"
#include "cam_ctrl.h"
#include "Interpolators.h"
#include "drv_vcm.h"

AFData AFInfo[MAX_SUPPORTED_CAM];

STATIC inline float ComputeLensPosition(float FocalLength, float FocusDistance);

// This assumes simple linear interpolation between the EEPROM calibration points.
UInt8 SetupLensPositionTable(AFData *Info)
{
	int i;
	FloatPoint *t = NULL;
	int result;

	int integer1, integer2, fraction1, fraction2; // Help with printing floats.

	log_debug("(%d)", __LINE__);

	// Validate inputs
	if (!Info) return 0;

	t = Info->TableData;

    // Handle the position appropriately.
    // For now, use the EFL.  This should be based on focuser properties, however.
	if (Info->EffectiveFocalLength == 35 || Info->EffectiveFocalLength == 28)
	{
	    // For the 35mm (VCM) modules, use full range values for the 'calibrated' distances.
	    // The Hall sensor calibration ensures this will work correctly with fixed distance values.
        t[0].x = ComputeLensPosition(Info->FocalLength, Info->CalibDistance[0]);
        t[1].x = ComputeLensPosition(Info->FocalLength, Info->CalibDistance[1]);
#ifdef BOARD_VERSION_P1
        t[0].y = AF_MIN_DAC; // Most negative 16-bit integer as a float.
        t[1].y = AF_MAX_DAC; // Most positive 16-bit integer as a float.
#else /* BOARD_VERSION_P1_1 */
        t[0].y = (float)(Int16)Info->CalibPosition[0];
	    t[1].y = (float)(Int16)Info->CalibPosition[1];
#endif
	}
	else
	{
	    // This is more generic and may work for 70mm and 150mm lenses.
	    // TODO: This is untested!
	    for(i = 0; i < Info->NumCalibrationPts; i++)
	    {
	        t[i].x = ComputeLensPosition(Info->FocalLength, Info->CalibDistance[i]);
            t[i].y = Info->CalibPosition[i];
        }
    }

    // Print the values.
    for (i = 0; i < Info->NumCalibrationPts; i++)
    {
        integer1 = t[i].x; fraction1 = (t[i].y - integer1) * 1000;
        integer2 = t[i].y; fraction2 = (t[i].y - integer2) * 1000;
        log_debug("(%d): i=%d pos=%d.%03d  dac=%d.%03d", __LINE__, i,
                integer1, fraction1, integer2, fraction2);
    }

	Info->DacInterp.DataType = InterpType_Float;
	Info->DacInterp.Size     = Info->NumCalibrationPts;
	Info->DacInterp.Table    = (void*)Info->TableData;
	result = SetupInterpFloatTable(&(Info->DacInterp));
	log_debug("(%d): SetupInterpFloatTable() returned %d", __LINE__, result)

	return result;
}

/*
 *  ComputeLensPosition(FocalLength, FocusDistance)
 *  - Returns the position of the lens.
 *  - Assumes that FocalLength and FocusDistance are in the same units (mm, for example)
 *  - Assumes FocalLength is the actual, not effective, FocalLength
 */
STATIC inline float ComputeLensPosition(float FocalLength, float FocusDistance)
{
	float LensPosition;

	if (FocusDistance == FocalLength) return 0.0f; // TODO: ASSERT()
	LensPosition = FocalLength * FocusDistance / (FocusDistance - FocalLength);

	return LensPosition;
}

// TODO: Refactor this - it's ugly prototype code.
UInt16 ComputeDacCode(AFData *Info)
{
	float        LensPosition = 0.f;
	Int16        DacCode = 0;
	InterpTable *DacTable = NULL;
	int          integer; // Help with printing floats.
	int          fraction; // Help with printing floats.

	// TODO: ASSERT(Info)
	DacTable = &(Info->DacInterp);

	log_debug("(%d): FocusDistance=%d mm", __LINE__,(unsigned int) Info->FocusDistance);

	// Step 1: Compute lens position.
	LensPosition = ComputeLensPosition(Info->FocalLength, Info->FocusDistance);

	integer = LensPosition;
	fraction = (LensPosition - integer) * 1000;
	log_debug("(%d): LensPosition=%d.%03d", __LINE__, integer, fraction);

	// Handle the DAC codes as either signed numbers (for 35mm modules) or unsigned (for others ?)
	if (Info->EffectiveFocalLength == 35 || Info->EffectiveFocalLength == 28)
	{
		float d;

		// Step 2: Map lens position to DAC code & round
		d = InterpLinearFloat(DacTable, LensPosition) + 0.5f;

		log_debug("(%d): DacCode=%d (pre-clamp)", __LINE__, (Int16) d);

		// Step 3: Clamp the DacCode to the calibration points in case the rounding
		//         moved the DacCode outside legal bounds.
#ifdef BOARD_VERSION_P1
        d = CLAMP(d, AF_MIN_DAC, AF_MAX_DAC);
#else /* BOARD_VERSION_P1_1 */
        d = CLAMP(d,(Int16) Info->CalibPosition[0], (Int16)Info->CalibPosition[1]);
#endif
		DacCode = (UInt16)((Int16)d);
		log_debug("(%d): DacCode=0x%04x (post-clamp)", __LINE__, DacCode);
	}
	else
	{
		// Step 2: Map lens position to DAC code & round
		DacCode = InterpLinearFloat(DacTable, LensPosition) + 0.5f;

		log_debug("(%d): DacCode=0x%04x (pre-clamp)", __LINE__, DacCode);

		// Step 3: Clamp the DacCode to the calibration points in case the rounding
		//         moved the DacCode outside legal bounds.
		DacCode = CLAMP(DacCode,
						MIN(Info->CalibPosition[0], Info->CalibPosition[Info->NumCalibrationPts-1]),
						MAX(Info->CalibPosition[0], Info->CalibPosition[Info->NumCalibrationPts-1]));

		log_debug("(%d): DacCode=0x%04x (post-clamp)", __LINE__, DacCode);
	}

	return DacCode;
}

static UInt16 AFControllerReadEeprom16(UInt8 addr, UInt8 ch_id)
{
	UInt16 data = 0;
	data  = af_controller_read_eeprom(addr, ch_id  ) << 8;
	data |= af_controller_read_eeprom(addr+1, ch_id);
	return data;
}

static const UInt16 EFL[] = {
#ifdef BOARD_VERSION_P1
		35,  // CAM_TYPE_35MM  = 0
#else /* BOARD_VERSION_P1_1 */
		28,
#endif
		70,  // CAM_TYPE_70MM  = 1
		150, // CAM_TYPE_150MM = 2
		0,	 // CAM_TYPE_MAX (for error checking)
};

static const UInt16 NumCalibrationPts[] = {
		2,  // CAM_TYPE_35MM  = 0
		4,  // CAM_TYPE_70MM  = 1
		4,  // CAM_TYPE_150MM = 2
		0,	// CAM_TYPE_MAX (for error checking)
};

// TODO: Split into static init & dynamic (EEPROM) init.
// NOTE: This can only safely be called when the camera is the currently selected device.
UInt8 InitAFInfo(CamDevice_TypeDef *cam)
{
	UInt8 i;
	UInt8 result = 0;
	/* This is static so we don't need to check more */
	AFData *info = &AFInfo[cam->camera_id - 1];

	/* Skip if this camera's AF data has already been initialized. */
	if (info->isValid)
		return 1;

	info->EffectiveFocalLength = EFL[cam->Type];

	/* Currently hard-coding this value based on 1/3.2" optical format. */
	info->CropFactor           = AF_CROP_FACTOR;
	info->FocalLength          = info->EffectiveFocalLength / info->CropFactor;

	// Read the calibration data from the EEPROM.
	// TODO: Pull this from a cached copy, or a structure w/the relevant data pushed from the host.
	// Macro position
	info->NumCalibrationPts = NumCalibrationPts[cam->Type];

	// Loop over the calibration points from Macro towards Infinity.
	// Note that different module types have different numbers of points, but they all are
	// contiguous, except infinity, and start at macro.
	UInt8 ch_id = cam->camera_id;
	for (i = 0; i < info->NumCalibrationPts - 1; i++)
	{
		/* TODO: please update below value with data from EEPROM */
		//AFControllerReadEeprom16(EEPROM_AF_DST_MACRO + 2*i, ch_id);
		info->CalibDistance[i] = 0x0064; // 100mm
		info->CalibPosition[i] = AFControllerReadEeprom16(EEPROM_AF_POS_MACRO + 2*i, ch_id);
	}

	// Finish with infinity.
	/* TODO: please update below value with data from EEPROM */
	//AFControllerReadEeprom16(EEPROM_AF_DST_INF, ch_id);
	info->CalibDistance[i] = 0x09C4; //2500mm
	info->CalibPosition[i] = AFControllerReadEeprom16(EEPROM_AF_POS_INF, ch_id);

	// Set up the position mapping table.
	result = SetupLensPositionTable(info);
	if (!result) goto done;

	info->FocusDistance = info->CalibDistance[info->NumCalibrationPts-1] / 10; /* mm -> cm */

	// Mark the data as valid.
	info->isValid = 1;
	result = 1;

done:
	return result;
}
