/*
 * ccb_AF_Control.c
 *
 *  Created on: Apr 21, 2015
 *      Author: linguyen
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "drv_piezo.h"
#include "drv_piezo_hall.h"
#include "drv_piezo_pwm.h"
#include "hal_i2c_ex.h"
#include "hal_flash_ex.h"
#include "drv_vcm.h"
#ifdef BOARD_VERSION_P1_1
#include "a1457_regs.h"
#endif /* BOARD_VERSION_P1_1 */
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"

/* Private typedef -----------------------------------------------------------*/


// This is temporary - borrowed from af_ctrl.h
#ifndef EEPROM_SLAVE_ADDR
#define EEPROM_SLAVE_ADDR	0xA0
#endif

//#define HARD_STOP_DELAY_PERIOD 50

#define LOG_CALIB_DATA 0
#define CALIB_DEBUG    0

#if CALIB_DEBUG
#define CalibDebug(...) log_debug(__VA_ARGS__)
#else // #if CALIB_DEBUG
#define CalibDebug(...)
#endif // #if CALIB_DEBUG


/**
* \brief  AFM structure. TODO: Add further description
*/
typedef struct {
UInt16 dir		: 1;	/** ( 0 : FAR , 1 : NEAR ) */
UInt16 toggle	: 1;	/** if 1, toggle the motor-direction */

UInt16 reg0;			/** data of reg0 */
UInt16 reg1;
uint16_t mvalue;		/** encoder value */
uint32_t freq;			/** driving frequency */
uint32_t period;
uint32_t duty;			/** duty by 0.01% unit */
} AFM;

typedef struct {
	Bool isCalibrated;
	Bool isConnect;
	UInt16 min;
	UInt16 max;
} ModuleData;
volatile AFM str_afm[AXIS_NO];				// AFM array of AF module

xTaskHandle xHandle_PID_Control = NULL;
xTaskHandle xHandle_PiezoMonitor = NULL;

/*
 *  John's Notes
 *
 *  Plan of attack:
 *  1. Refactor the code into better abstractions.
 *  2. Read the EEPROM data.
 *  3. Update the calibration routine per Sapna & Harpuneet.
 *  4. Update the position calculation based on the same.
 *  4.2 Plumb the code through into the main code.
 *  4.4 Provide defaults for EEPROM data that returns 0x0.
 *  5. Connect the distance-based AF controls to the piezos.
 *  6. Refactor the AF code to separate AF calculations from the actuator control.
 *
 * Theories of piezo movement (given current position X and desired target position Y):
 * a. From the position difference (Y-X) and the direction, determine polarity & duty cycle for PWM.
 *    Start the PWM, wait a bit, and the actuator will be at Y.
 * b. The duty cycle determines the amount of force to apply to the actuator.  Use a PID controller to
 *    converge (reduce the force as the actuator nears its target).
 *
 * SPI is for communication with CPLD.
 *
 * Need to sort out how to handle the polarity data:
 * - Set the bit in the Hall sensor
 * - Math on the STM
 */
/*
 *  TODO Cleanups:
 *  - Refine semantics to avoid repeated subroutine calls.
 */
/*
 * Terminology:
 * camera:   A camera module
 * actuator: A single piezo actuator
 * channel:  A control channel, which can control two actuators (currently a mirror and lens for the same camera).
 *           This is the low-level portion.
 *
 * Within the data structures, the hierarchy is:
 *     Module -> Actuator -> Channel
 */

/*
 * Current thoughts on piezos:
 * - need way to select between our own calibration vs. EEPROM data vs. override calibration data
 *   - start with EEPROM data (lowest priority)
 *   - override with our own calibration if run (middle priority)
 *   - override with external override data if provided (top priority)
 * - need way to calibrate mirror actuators without disturbing lenses (?)
 * - need to map distance to position through estimated focal length & offset
 * - need to improve positioning accuracy
 */


// Note: Indexed by HW channels (i.e., not in camera order)
static PiezoChannel Channels[NUM_PIEZO_CHANNELS] =
{
	    CHANNEL_DATA(0),
	    CHANNEL_DATA(1),
	    CHANNEL_DATA(2),
	    CHANNEL_DATA(3),
	    CHANNEL_DATA(4),
	    CHANNEL_DATA(5),
	    CHANNEL_DATA(6),
	    CHANNEL_DATA(7),
	    CHANNEL_DATA(8),
	    CHANNEL_DATA(9),
	    CHANNEL_DATA(10),
};

static PiezoActuator *CurrentActuator = NULL;

static PiezoActuator Actuators[NUM_PIEZO_ACTUATORS];
static PiezoModule   Modules  [NUM_PIEZO_MODULES];

// The P1 has but one Pwm.
PiezoPwm Pwm;


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// lens and mirror actuator mapping table

volatile int16_t pid_input=0;
volatile int16_t e_pid=0;
volatile piezo_status _piezo_status = USM_IDLE;
static int EnablePiezoMonitor = 0;
static int LastEnablePiezoMonitor = -1;
static unsigned int PiezoMonitorBitmask = 0;
uint8_t spi_flash_sector_data[SPI_FLASH_SUB_SEC_SIZE];
int16_t pid_value=0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void AF_control_Config(void);
static void vPID_Control(void *pvParameters);
static void vPiezoMonitor(void *pvParameters);
//static void vPID_test(void *pvParameters);
static void SPI_Transfer(uint8_t* dataW, uint8_t* dataR, uint16_t num_bytes);
static Bool check_connect(void);

static Error_t InitPiezoModules(void);

#define NUM_SENSITIVITIES 4

static void ScanHallSensitivity(HallSensor *h, UInt16 *Data, UInt16 Min, UInt16 Max);

// Error_t MovePiezoToPosition(PiezoActuator *a, UInt16 TargetPosition, UInt16 Tolerance, UInt16 DutyCycle);

// EEPROM data encoding bitfields
#define SENSITIVITY_RANGE 15:14
#define POLARITY_RANGE    13:13
#define RESOLUTION_RANGE  12: 0

// TODO: Move these macros to somewhere more global
#define HI_BIT(r)       (1?r)
#define LO_BIT(r)       (0?r)
#define MASK(n)         ((1<<(n))-1)
#define NBITS(r)        (HI_BIT(r) - LO_BIT(r) + 1)
#define GET_FIELD(x, r) (((x) >> LO_BIT(r)) & MASK(NBITS(r)))

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifdef BOARD_VERSION_P1_1
#ifndef ABS_DIFF
#define ABS_DIFF(a,b) ((a) > (b) ? (a) - (b) : (b) - (a))
#endif
#endif /* BOARD_VERSION_P1_1 */

// Magic constants
PiezoActuatorData LensData150   = { 0, 1400, 700, 300, 136 };
PiezoActuatorData LensData70    = { 0, 1100, 700, 300, 200 };
PiezoActuatorData MirrorData150 = { 1,  420, 410, 100,   5 };
PiezoActuatorData MirrorData70  = { 1,  420, 410, 100,   5 };

static UInt16 ReadEeprom16(UInt8 CameraId, UInt8 Address)
{
	UInt16 data = 0;

	data  = fpga_read_i2c_value(CameraId, EEPROM_SLAVE_ADDR, Address,     ADDR8_DATA8) << 8;
	data |= fpga_read_i2c_value(CameraId, EEPROM_SLAVE_ADDR, Address + 1, ADDR8_DATA8);

	return data;
}

static UInt32 ReadEeprom24(UInt8 CameraId, UInt8 Address)
{
	UInt32 data = 0;

	data  = fpga_read_i2c_value(CameraId, EEPROM_SLAVE_ADDR, Address,     ADDR8_DATA8) << 16;
	data |= fpga_read_i2c_value(CameraId, EEPROM_SLAVE_ADDR, Address + 1, ADDR8_DATA8) <<  8;
	data |= fpga_read_i2c_value(CameraId, EEPROM_SLAVE_ADDR, Address + 2, ADDR8_DATA8);

	return data;
}

/*
 * Convenience function:  Write three bytes over SPI.  Readback doesn't matter.
 */
void SPI_Write3(UInt8 a, UInt8 b, UInt8 c)
{
	uint8_t dataTx[3], dataRx[3];

	dataTx[0] = a;
	dataTx[1] = b;
	dataTx[2] = c;

	// log_debug("SPI XFER: [0x%02x 0x%02x 0x%02x]", dataTx[0], dataTx[1], dataTx[2]);

	SPI_Transfer(dataTx, dataRx, 3);
}

/*
 * CPLD Management Functions
 */
// Turn off all CPLD outputs.
void ResetCpld(void)
{
	//ENTER_FUNC
	SPI_Write3(0x02, 0x02, 0x00);
	SPI_Write3(0x02, 0x03, 0x00);
	SPI_Write3(0x02, 0x04, 0x00);
	SPI_Write3(0x02, 0x05, 0x00);
	// EXIT_FUNC
}

//  Note: Current implementation assumes the pwm is already off.
//        To gurantee a clean switch, the #if'd code should be completed.
Error_t SetCpld(PiezoActuator *a)
{
	Error_t e = ERROR_NONE;

	//ENTER_FUNC

	CHECK_ERROR(a == NULL ? ERROR_INVALID_ARG : ERROR_NONE);

	//log_debug("CpldReg:0x%02x Value:0x%02x", a->CpldRegister, a->CpldBitMask);
	SPI_Write3(0x02, a->CpldRegister, a->CpldBitMask);

	// EXIT_FUNC

	return e;
}

void SetCurrentActuator(PiezoActuator *a)
{
	//ENTER_FUNC

	// TODO: Optimize by early exit if a == CurrentActuator - but be conservative now.

	// Stop using the old actuator.
	ResetCpld(); // Turn off all CPLD outputs.  Later optimize.
	TurnOffPwm(&Pwm, CurrentActuator);

	SetCpld(a);
	CurrentActuator = a;

	// EXIT_FUNC
}


// TODO: Remove pre-defined search range and search for the right values.
void ComputeDerivedFocusParameters(PiezoCalibrationData *p)
{
	float PositionError;
	float FocusPosition;
	float HardStopOffset;
	float OffsetMin = 0;
	float OffsetMax = 0;
	float BestPositionError;
	int i;
	float FocalLength;

	// Temporary search constraints
	float FocalLengthIdeal;
	float FocalLengthMin;
	float FocalLengthMax;
	float FocalLengthStep;

	// Initialize variables
	BestPositionError = 1e20; // A large value.
	FocalLengthStep   = 0.00001;

	// For now, establish the focal length search range based on the
	// calibration data.  This is a hack - see the TODO above.}
	switch (p->Distance[3])
	{
	case 12000: // 70mm lens
	    p->DerivedMirrorDistance = 0.0;   // TODO: Estimate this.
	    p->DerivedDistanceScale  = 1.0; // TODO: Estimate this.
		FocalLengthIdeal = 9.19;
		break;

	case 54000: // 150mm lens
	    p->DerivedMirrorDistance = 0.0; // TODO: Estimate this.
        p->DerivedDistanceScale  = 1.0; // TODO: Estimate this.
		FocalLengthIdeal = 19.77;
		break;

	default:
		log_error("Unexpected focus distance data: Distance[3] = %d", p->Distance[3]);
		return;
	}

	FocalLengthMin = 0.8 * FocalLengthIdeal;
	FocalLengthMax = 1.5 * FocalLengthIdeal;

	// log_debug("FL range: [%2.4f, %2.4f] in steps of %f", FocalLengthMin, FocalLengthMax, FocalLengthStep);

	// Search for the best derived focal length & hard stop offset.
	for (FocalLength = FocalLengthMin; FocalLength <= FocalLengthMax; FocalLength += FocalLengthStep)
    {
		// Loop over all calibration points and compute the offsets
		// needed for the current FocalLength to pass through the measured
		// data.  Save the min & max offsets.
		for (i = 0; i < p->NumCalibPts; i++)
		{
			FocusPosition  = (p->Distance[i] * FocalLength)/(p->Distance[i]-FocalLength);
			HardStopOffset = FocusPosition - (p->Position[i]/1000.0); // Position data is in microns, FocusPosition in mm.

			// Update min & max
			OffsetMin = i ? MIN(OffsetMin, HardStopOffset) : HardStopOffset;
			OffsetMax = i ? MAX(OffsetMax, HardStopOffset) : HardStopOffset;
		}

		// Best value of d: midway between the extremes,
		// leaving the error as half the distance between the
		// extremes.
		HardStopOffset = (OffsetMin + OffsetMax) / 2.0;
		PositionError  = (OffsetMax - OffsetMin) / 2.0;

		if (PositionError < BestPositionError)
		{
			BestPositionError        = PositionError;
			p->DerivedFocalLength    = FocalLength;
			p->DerivedHardStopOffset = HardStopOffset;
		}
    }

	// log_debug("Result: FL:%2.4f  HS:%2.4f  e:%f", p->DerivedFocalLength, p->DerivedHardStopOffset, BestPositionError);
}

/*STATIC*/ void PrintPiezoCalibrationData(PiezoCalibrationData *p)
{
	int i;

	log_debug("Calibration Data:");
	log_debug("    Polarity:  %04d  Sense: %04d", (int) p->Polarity, (int) p->Sensitivity);
	log_debug("    Resolution:%04d  Offset:%04d", p->Resolution, p->Offset);
	log_debug("    N:%d", p->NumCalibPts);
	for(i = 0; i < p->NumCalibPts; i++)
	{
		log_debug("        %d: %d mm   %d um", i, p->Distance[i], p->Position[i]);
	}

	{
		// Print frequency in a pretty manner.
		UInt16 m, k, h; // MHz, KHz, Hz
		UInt32 t = p->Frequency; // Tmp.
		h = t % 1000; t /= 1000;
		k = t % 1000; t /= 1000;
		m = t % 1000; t /= 1000;
		if (m)
		{
			log_debug("    f:%d,%03d,%03d Hz", m, k, h);
		}
		else
		{
			log_debug("    f:%d,%03d Hz", k, h);
		}
	}

    log_debug("    Derived FL: %2.4f  HS offset: %2.4f",
              p->DerivedFocalLength, p->DerivedHardStopOffset);
}


int hall_calibration_read( CamDevice_TypeDef *pCam, hall_calib_t * lens_calib,
		hall_calib_t * mirror_calib)
{
    int ret = 0;

	/* Read from SPI flash */
    SPI_FLASH_BufferRead((uint8_t * )lens_calib, CAM_LENS_CALIBRATION_LOC(pCam->camera_id),
                          sizeof(hall_calib_t));

    if (lens_calib->calib_header != FLASH_CALIB_HDR)
    {
        ret = LENS_CALIBRATION;
    }

	SPI_FLASH_BufferRead((uint8_t * )mirror_calib, CAM_MIRROR_CALIBRATION_LOC(pCam->camera_id),
			sizeof(hall_calib_t));

	 log_debug("mirror_calib->calib_header  0x%x \r\n", (unsigned int )mirror_calib->calib_header);

    if (mirror_calib->calib_header != FLASH_CALIB_HDR)
    {
        if (pCam->Type != CAM_TYPE_35MM)
        ret |= MIRROR_CALIBRATION;
    }

	return ret;
}

int hall_calibration_save(CamDevice_TypeDef *pCam, CALIBRATION_TYPE type)
{
    int ret = 0;
    PiezoModule*  piezo_module = CameraIdToModule(pCam->camera_id);
    hall_calib_t calib;
    //HallSensor hall;
    uint32_t dst_address = 0;

    calib.calib_header = FLASH_CALIB_HDR;

    SPI_FLASH_BufferRead((uint8_t * )spi_flash_sector_data, FLASH_CALIB_DATA_START,
        sizeof(uint8_t) * SPI_FLASH_SUB_SEC_SIZE);

    switch (type)
    {
        case LENS_CALIBRATION:
            memcpy(&(calib.calib_data), &piezo_module->Lens->CalibData, sizeof(PiezoCalibrationData));

            //copy to the corresponding block
            dst_address = CAM_LENS_CALIBRATION_LOC(pCam->camera_id) - FLASH_CALIB_DATA_START;
            memcpy((spi_flash_sector_data + dst_address), &calib, sizeof(hall_calib_t));

            break;
        case MIRROR_CALIBRATION:
            if (pCam->Type == CAM_TYPE_70MM || pCam->Type == CAM_TYPE_150MM)
            {
            	log_debug("Mirror calibration save!\n");
                memcpy(&(calib.calib_data), &piezo_module->Mirror->CalibData, sizeof(PiezoCalibrationData));

                dst_address = CAM_MIRROR_CALIBRATION_LOC(pCam->camera_id) - FLASH_CALIB_DATA_START;
                memcpy((spi_flash_sector_data + dst_address), &calib, sizeof(hall_calib_t));
            }
            else
                log_debug("Mirror is not supported on 35mm modules!\n");

            break;
        default:
            log_debug("unknown type\n");
            ret = 1;
    }
	SPI_FLASH_SubSectorErase(CAM_LENS_CALIBRATION_LOC(pCam->camera_id));
	SPI_FLASH_BufferWrite((uint8_t * )spi_flash_sector_data, FLASH_CALIB_DATA_START,
        sizeof(uint8_t) * SPI_FLASH_SUB_SEC_SIZE);
	return ret;
}

int hall_calibration_update_position( CamDevice_TypeDef *pCam, CALIBRATION_TYPE type)
{
	int ret = 0;
	PiezoModule*  piezo_module = CameraIdToModule(pCam->camera_id);
	hall_calib_t calib;
	HallSensor hall;
	uint32_t dst_address = 0;

	SPI_FLASH_BufferRead((uint8_t * )spi_flash_sector_data, FLASH_CALIB_DATA_START,
				sizeof(uint8_t) * SPI_FLASH_SUB_SEC_SIZE);

	switch (type)
	{
        case LENS_CALIBRATION:
            if (pCam->Type == CAM_TYPE_35MM)
            {
                calib.position  = af_controller_get_current_pos(NULL,
                        pCam->camera_id);
            }
            else if (pCam->Type == CAM_TYPE_70MM || pCam->Type == CAM_TYPE_150MM)
            {
                hall = piezo_module->Lens->Hall;
                calib.position = ReadHallSensor(&hall);
            }

            //copy to the corresponding block
            dst_address = CAM_LENS_CALIBRATION_LOC(pCam->camera_id) +
                m_offset(hall_calib_t, position) - FLASH_CALIB_DATA_START;
            memcpy((spi_flash_sector_data + dst_address), &calib.position, sizeof(calib.position));
            break;

        case MIRROR_CALIBRATION:
            if (pCam->Type == CAM_TYPE_70MM || pCam->Type == CAM_TYPE_150MM)
            {
                hall = piezo_module->Mirror->Hall;
                calib.position = ReadHallSensor(&hall);
                //copy to the corresponding block
                dst_address = CAM_MIRROR_CALIBRATION_LOC(pCam->camera_id) +
                    m_offset(hall_calib_t, position) - FLASH_CALIB_DATA_START;
                memcpy((spi_flash_sector_data + dst_address), &calib.position, sizeof(calib.position));

                log_debug("updated mirror pos 0x%x for cam %s", calib.position, pCam->Name);
            }
            else
            {
                log_debug("Mirror not supported on other cameras\n");
            }
            break;
        default:
            log_debug("unknown type\n");
            ret = 1;
	}

	SPI_FLASH_SubSectorErase(CAM_LENS_CALIBRATION_LOC(pCam->camera_id));

	SPI_FLASH_BufferWrite((uint8_t * )spi_flash_sector_data, FLASH_CALIB_DATA_START,
			sizeof(uint8_t) * SPI_FLASH_SUB_SEC_SIZE);

	return ret;

}
// Note: Most values are not yet defined for the mirror data.
Error_t ReadHallCalibration(CamDevice_TypeDef *pCam, int CameraId, PiezoCalibrationData *LensData, PiezoCalibrationData *MirrorData)
{
	Error_t e = ERROR_NONE;
#ifdef BOARD_VERSION_P1
	UInt16 data;
	int ret = 0;
	hall_calib_t lens_calib;
	hall_calib_t mirror_calib;
	PiezoModule*   module = NULL;
	PiezoActuator* lens   = NULL;
	PiezoActuator* mirror = NULL;
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
	UInt16 fs_max, fs_min, hs_min, hs_max;
	I2CEx_Msg_t I2CBuffer;
	UInt8 txbuf[2];
	UInt8 rxbuf[2];
	HallSensor hall_sensor;
	__IO Int8 transfer_status = 0;
#endif /* BOARD_VERSION_P1_1 */

	//ENTER_FUNC

	if (!pCam || !LensData || !MirrorData || CameraId < 0 || CameraId > 16)
	{
		return ERROR_INVALID_ARG;
	}
#ifdef BOARD_VERSION_P1
	module = CameraIdToModule(CameraId);
	lens = module->Lens;
	mirror = module->Mirror;
    if ((ret = hall_calibration_read( pCam, &lens_calib, &mirror_calib)))
    {
    	log_debug(" hall read returned 0x%x \r\n", ret);

    	if (ret & LENS_CALIBRATION)
    	{
    		log_debug(" calibrating lens!\n");
    		data = ReadEeprom16(CameraId, 0x11);
    		LensData->Sensitivity = GET_FIELD(data, SENSITIVITY_RANGE);
    		LensData->Polarity    = GET_FIELD(data, POLARITY_RANGE);

    		if (LensData->Polarity == 1)
    			pCam->lens_hall_polarity = HALL_POLARITY_REVERSED;
    		else
    			pCam->lens_hall_polarity = HALL_POLARITY_NORMAL;

    		LensData->Resolution  = GET_FIELD(data, RESOLUTION_RANGE);
    		LensData->Offset = ReadEeprom16(CameraId, 0x2f);

            lens->Hall.Sensitivity = LensData->Sensitivity;
            lens->Hall.Polarity    = LensData->Polarity;

            InitHallSensor(&(lens->Hall));

            // Saves hard stops
            calibrate_lens(pCam, lens);

            if (pCam->Type == CAM_TYPE_35MM)
                lens_calib.position  = af_controller_get_current_pos(NULL, CameraId);

            else if (pCam->Type == CAM_TYPE_70MM || pCam->Type == CAM_TYPE_150MM)
                lens_calib.position = ReadHallSensor(&(lens->Hall));

    		hall_calibration_save(pCam, LENS_CALIBRATION);
    		log_debug(" saved calibrating lens!\n");
    	}
		if (ret & MIRROR_CALIBRATION )
		{
			log_debug(" calibrating mirror!\n");
			// Initialize mirror values to defaults
			// These values will be overwritten by mirror calibration if run
			MirrorData->Sensitivity = (UInt8) 3;
			pCam->mirror_hall_polarity = HALL_POLARITY_NORMAL;

            mirror->Hall.Sensitivity = MirrorData->Sensitivity;
            InitHallSensor(&(mirror->Hall));

			if (pCam->Type != CAM_TYPE_35MM)
            {
                // The mirror's hall sensor polarity is not stored in
                // EEPROM, so we need to calibrate to find it
				calibrate_mirror(pCam, mirror);
                InitHallSensor(&(mirror->Hall));
            }

            mirror_calib.position = ReadHallSensor(&(mirror->Hall));
            MirrorData->Polarity = pCam->mirror_hall_polarity;

			hall_calibration_save(pCam, MIRROR_CALIBRATION);
			log_debug("saved  calibrating mirror!\n");
		}
		log_debug("Saving calibration data for the first time!\n");
	}
	else
	{
		//PiezoCalibrationData *LensData, PiezoCalibrationData *MirrorData
		memcpy((void *)LensData, (void *)(&lens_calib.calib_data),
				sizeof(PiezoCalibrationData));
		memcpy((void *)MirrorData,(void *) &mirror_calib.calib_data,
				sizeof(PiezoCalibrationData));
		pCam->mirror_hall_polarity = MirrorData->Polarity;
		pCam->lens_hall_polarity = LensData->Polarity;
	}

    log_debug("%s mirror_hall_polarity: %d \n", pCam->Name, pCam->mirror_hall_polarity);
    log_debug("%s lens_hall_polarity: %d \n", pCam->Name, pCam->lens_hall_polarity);

    lens->Hall.Sensitivity       = LensData->Sensitivity;
    lens->Hall.Polarity          = LensData->Polarity;
    log_debug("lens->CalibData.NearPosition = 0x%04x\n", lens->CalibData.NearPosition);
    log_debug("lens->CalibData.FarPosition  = 0x%04x\n", lens->CalibData.FarPosition);

    mirror->Hall.Sensitivity       = MirrorData->Sensitivity;
    mirror->Hall.Polarity          = MirrorData->Polarity;
    log_debug("mirror->CalibData.NearPosition = 0x%04x\n", mirror->CalibData.NearPosition);
    log_debug("mirror->CalibData.FarPosition  = 0x%04x\n", mirror->CalibData.FarPosition);

#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1

	hall_sensor = CameraIdToModule(pCam->camera_id)->Lens->Hall;

	if (!hall_sensor.I2CSlaveAddress)
	{
		log_debug(" %s : No Hall I2C slave address", __FUNCTION__);
		return ERROR_MODULE_FAULT;
	}

	I2CBuffer.txbuf.data = txbuf;
	I2CBuffer.rxbuf.data = rxbuf;
	I2CBuffer.is_completed = &transfer_status;
	I2CBuffer.ch = pCam->I2CChannel;
	I2CBuffer.addr = hall_sensor.I2CSlaveAddress;

	LensData->Sensitivity = (I2C_8BitAddr_ReadByte(&I2CBuffer, A1457_EE_GAIN_MSB) << 8) |
			I2C_8BitAddr_ReadByte(&I2CBuffer,  A1457_EE_GAIN_LSB);

	hs_min = (I2C_8BitAddr_ReadByte(&I2CBuffer, A1457_EE_HSTOP_MIN_MSB) << 8) |
			I2C_8BitAddr_ReadByte(&I2CBuffer,  A1457_EE_HSTOP_MIN_LSB);
	hs_max = (I2C_8BitAddr_ReadByte(&I2CBuffer, A1457_EE_HSTOP_MAX_MSB) << 8) |
			I2C_8BitAddr_ReadByte(&I2CBuffer,  A1457_EE_HSTOP_MAX_LSB);
	fs_min = (I2C_8BitAddr_ReadByte(&I2CBuffer, A1457_EE_KHSTOP_MIN_MSB) << 8) |
			I2C_8BitAddr_ReadByte(&I2CBuffer,  A1457_EE_KHSTOP_MIN_LSB);
	fs_max = (I2C_8BitAddr_ReadByte(&I2CBuffer, A1457_EE_KHSTOP_MAX_MSB) << 8) |
			I2C_8BitAddr_ReadByte(&I2CBuffer,  A1457_EE_KHSTOP_MAX_LSB);

	log_debug("hs_min 0x%x \r\n", hs_min);
	log_debug("hs_max 0x%x \r\n", hs_max);

	log_debug("fs_min 0x%x \r\n", fs_min);
	log_debug("fs_max 0x%x \r\n", fs_max);

	log_debug("A1457_EE_BLOCK2_PIDCTL5 0x%x \r\n", I2C_8BitAddr_ReadByte(&I2CBuffer,  A1457_EE_BLOCK2_PIDCTL5));

	LensData->Resolution  = ABS_DIFF(fs_max, fs_min) * 1000 / ABS_DIFF(hs_max, hs_min);

	LensData->Polarity    = hs_max > hs_min ? 0 : 1; //  or find out during calibration ??
	//FIXME OFFSET??
	//LensData->Offset = ReadEeprom16(CameraId, 0x2f);

	// Initialize mirror values to defaults
	// These values will be overwritten by mirror calibration if run
	MirrorData->Sensitivity = (UInt8) 3;
	MirrorData->Polarity    = (UInt8) 0;

//	int f =0;
//
//	printf("EEPROM for %s \r\n",pCam->Name);
//	for (f = 0; f<= 0x100; f++)
//	{
//		printf("address 0x%x value 0x%x \r\n",f, fpga_read_i2c_value(CameraId, EEPROM_SLAVE_ADDR, f,     ADDR16_DATA8));
//	}

	int ret = 0;
	hall_calib_t lens_calib;
	hall_calib_t mirror_calib;
	PiezoModule*   module = NULL;
	PiezoActuator* lens   = NULL;
	PiezoActuator* mirror = NULL;

	module = CameraIdToModule(CameraId);
	lens = module->Lens;
	mirror = module->Mirror;
    if ((ret = hall_calibration_read( pCam, &lens_calib, &mirror_calib)))
    {
    	if (ret & LENS_CALIBRATION)
    	{
            start_lens_calibration(CameraId, pCam, lens, LensData, lens_calib);

    	}
		if (ret & MIRROR_CALIBRATION )
		{
			// Initialize mirror values to defaults
			// These values will be overwritten by mirror calibration if run
			MirrorData->Sensitivity = (UInt8) 3;
			pCam->mirror_hall_polarity = HALL_POLARITY_NORMAL;

            mirror->Hall.Sensitivity = MirrorData->Sensitivity;
            InitHallSensor(&(mirror->Hall));

			if (pCam->Type != CAM_TYPE_35MM)
            {
                // The mirror's hall sensor polarity is not stored in
                // EEPROM, so we need to calibrate to find it
				calibrate_mirror(pCam, mirror);
				InitHallSensor(&(mirror->Hall));
            }

            mirror_calib.position = ReadHallSensor(&(mirror->Hall));
            MirrorData->Polarity = pCam->mirror_hall_polarity;
			hall_calibration_save(pCam, MIRROR_CALIBRATION);
		}
		log_debug("Saving calibration data for the first time!\n");
	}
	else
	{
		//PiezoCalibrationData *LensData, PiezoCalibrationData *MirrorData
		memcpy((void *)LensData, (void *)(&lens_calib.calib_data),
				sizeof(PiezoCalibrationData));
		memcpy((void *)MirrorData,(void *) &mirror_calib.calib_data,
				sizeof(PiezoCalibrationData));
		pCam->mirror_hall_polarity = MirrorData->Polarity;
		pCam->lens_hall_polarity = LensData->Polarity;
	}

    log_debug("%s mirror_hall_polarity: %d \n", pCam->Name, pCam->mirror_hall_polarity);
    log_debug("%s lens_hall_polarity: %d \n", pCam->Name, pCam->lens_hall_polarity);

    lens->Hall.Sensitivity       = LensData->Sensitivity;
    lens->Hall.Polarity          = LensData->Polarity;
    log_debug("lens->CalibData.NearPosition = 0x%04x\n", lens->CalibData.NearPosition);
    log_debug("lens->CalibData.FarPosition  = 0x%04x\n", lens->CalibData.FarPosition);

    mirror->Hall.Sensitivity       = MirrorData->Sensitivity;
    mirror->Hall.Polarity          = MirrorData->Polarity;
    log_debug("mirror->CalibData.NearPosition = 0x%04x\n", mirror->CalibData.NearPosition);
    log_debug("mirror->CalibData.FarPosition  = 0x%04x\n", mirror->CalibData.FarPosition);

#endif /* BOARD_VERSION_P1_1 */

	// EXIT_FUNC
	return e;
}

void start_lens_calibration(int CameraId, CamDevice_TypeDef* pCam, PiezoActuator* lens, PiezoCalibrationData* LensData, hall_calib_t lens_calib)
{
    lens->Hall.Sensitivity = LensData->Sensitivity = 2;// As this gives 0.5um accuracy
    lens->Hall.Polarity    = 0;

    InitHallSensor(&(lens->Hall));
    vTaskDelay(20);

    calibrate_lens(pCam, lens);

    LensData->Polarity = (pCam->retracted_hard_stop > pCam->extended_hard_stop)
                          ? HALL_POLARITY_REVERSED : HALL_POLARITY_NORMAL;
    lens->Hall.Polarity = LensData->Polarity;

    if (HALL_POLARITY_REVERSED == LensData->Polarity)
    {
        LensData->NearPosition = pCam->extended_hard_stop;
        LensData->FarPosition  = pCam->retracted_hard_stop;

        pCam->extended_hard_stop = LensData->FarPosition;
        pCam->retracted_hard_stop = LensData->NearPosition;
    }

    log_debug("LensData->Polarity %d  %s\r\n",LensData->Polarity, pCam->Name);

    log_debug("pCam->retracted_hard_stop 0x%x\r\n",pCam->retracted_hard_stop);
    log_debug("pCam->extended_hard_stop 0x%x\r\n", pCam->extended_hard_stop);

    InitHallSensor(&(lens->Hall));

    if (pCam->Type == CAM_TYPE_35MM)
        lens_calib.position  = af_controller_get_current_pos(NULL, CameraId);

    else if (pCam->Type == CAM_TYPE_70MM || pCam->Type == CAM_TYPE_150MM)
        lens_calib.position = ReadHallSensor(&(lens->Hall));

    hall_calibration_save(pCam, LENS_CALIBRATION);
}


// Note: Most values are not yet defined for the mirror data.
Error_t ReadEepromCalibration(int CameraId, PiezoCalibrationData *LensData, PiezoCalibrationData *MirrorData)
{
	Error_t e = ERROR_NONE;
	UInt16 data;

	//ENTER_FUNC

	// TODO: Better validation.
	if (!LensData || !MirrorData || CameraId < 0 || CameraId > 16)
	{
		return ERROR_INVALID_ARG;
	}

    // adding error checking from piezo_dev2 branch
    // CHECK_ERROR(ReadHallCalibration(pCam, CameraId, LensData, MirrorData));

	data = ReadEeprom16(CameraId, 0x11);
	LensData->Sensitivity = GET_FIELD(data, SENSITIVITY_RANGE);
	LensData->Polarity    = GET_FIELD(data, POLARITY_RANGE);
	LensData->Resolution  = GET_FIELD(data, RESOLUTION_RANGE);

	LensData->NumCalibPts = 4;

	LensData->Position[0] = ReadEeprom16(CameraId, 0x13);
	LensData->Position[1] = ReadEeprom16(CameraId, 0x15);
	LensData->Position[2] = ReadEeprom16(CameraId, 0x17);
	LensData->Position[3] = ReadEeprom16(CameraId, 0x1f);

	LensData->Distance[0] = ReadEeprom16(CameraId, 0x21);
	LensData->Distance[1] = ReadEeprom16(CameraId, 0x23);
	LensData->Distance[2] = ReadEeprom16(CameraId, 0x25);
	LensData->Distance[3] = ReadEeprom16(CameraId, 0x2d);

	// Initialize mirror values to defaults
	// These values will be overwritten by mirror calibration if run
	//log_printf("setting the mirror sensitivity to 3\n");
	MirrorData->Sensitivity = (UInt8) 3;
	MirrorData->Polarity    = (UInt8) 0;
	//log_printf("MirrorData->Sensitivity = %d\n", MirrorData->Sensitivity);

	MirrorData->Frequency = ReadEeprom24(CameraId, 0x27);
	LensData->Frequency   = ReadEeprom24(CameraId, 0x2a);

	// Override!
	// MirrorData->Frequency = 135000;
	if (!MirrorData->Frequency)
	{
		log_error("No mirror piezo frequency in EEPROM.  Overriding.");
		MirrorData->Frequency = 135000;
	}
	if (!LensData->Frequency)
	{
		log_error("No lens piezo frequency in EEPROM.  Overriding.");
		LensData->Frequency = 128000;
	}

	LensData->Offset = ReadEeprom16(CameraId, 0x2f);

#if WAR_JIRATBD
	// To work around the bad calibration data of 150mm lenses, adjust
	// the values read from the EEPROM.
	if(is_150mm)
	{
		// Affected modules were calibrated at Sunny between 5/26 and 5/30
		// AF calibration date is 6/1 through 6/5
		UInt8 Month;
		UInt8 Day;
		UInt8 Year;

		Year  = ReadEeprom8(CameraId, 0x02); // TODO: Symbolic addresses
		Month = ReadEeprom8(CameraId, 0x03);
		Day   = ReadEeprom8(CameraId, 0x04);

		if ((Year == 15) &&	(Month == 5) &&
			(Day >= 26) && (Day <= 30))
		{
			// Move the 2m data to the 1m position.
			LensData->Position[0] = LensData->Position[1];
			// Approximate the missing data as 1/4 the 1M data + 3/4 the 4M data.
			// TODO: Derive a better approximation for the missing data.
			LensData->Position[1] = (LensData->Position[0] + 3 * LensData->Position[2])/4;
		}
	}
#endif // #if WAR_JIRATBD

	ComputeDerivedFocusParameters(LensData);

	log_debug("Lens Calibration Data:");
	PrintPiezoCalibrationData(LensData);
	//log_debug("Mirror Calibration Data:");
	//PrintPiezoCalibrationData(MirrorData);

	// EXIT_FUNC

	return e;
}

UInt8 CameraIdToModuleId(UInt8 CameraId)
{
	if (CameraId < CAM_ID_B1) return 0xff;
	if (CameraId > CAM_ID_C6) return 0xff;
	return CameraId - CAM_ID_B1;
}

UInt8 ModuleIdToCameraId(UInt8 ModuleId)
{
	if (ModuleId > NUM_PIEZO_MODULES) return 0xff;
	return ModuleId + CAM_ID_B1;
}

// Temporary code until the piezo code is better integrated.
PiezoModule* CameraIdToModule(UInt8 CameraId)
{
	UInt8 ModuleId = CameraIdToModuleId(CameraId);
	CalibDebug("CameraId:%d -> ModuleId:%d", CameraId, ModuleId);
	if (ModuleId == 0xff) return NULL;
	return &(Modules[ModuleId]);
}

char *CameraNames[] =
{
    "G",
    "A1", "A2", "A3", "A4", "A5",
    "B1", "B2", "B3", "B4", "B5",
    "C1", "C2", "C3", "C4", "C5", "C6",
};

static UInt8 CamIdToChannel[] =
{
    0,                 // Global ID
    0, 0, 0, 0, 0,     // A1 - A5
    4, 3, 2, 0, 1,     // B1 - B5
    5, 6, 8, 7, 10, 9, // C1 - C6
};

// Note: This sets up data structures but does not run calibration.
//       Calibration relies upon data in the EEPROM which is not accessible
//       until the modules are brought to SW standby or streaming.
//       To reduce future problems, this code does not expect this
//       to have happened until accessing the camera module (and there
//       is still more coordination to do with other code).
Error_t InitPiezoModules(void)
{
	int            i;
	int            Channel;
	int            CameraId;
	Error_t        e = ERROR_NONE;
	PiezoActuator *Lens;
	PiezoActuator *Mirror;

	ENTER_FUNC

	for (i = 0; i < NUM_PIEZO_MODULES; i++)
	{
		CameraId = ModuleIdToCameraId(i);   // Index into the CamIdToChannel starting with B1.
		Modules[i].CameraId = CameraId;
		Channel = CamIdToChannel[CameraId];

		CalibDebug("Initializing module %d ( %s ) to channel %d", i, CameraNames[CameraId], Channel);

		Lens   = &Actuators[2*i    ];
		Mirror = &Actuators[2*i + 1];

		Lens->Type                      = ActuatorType_Lens;
		Lens->Channel                   = &Channels[Channel];
		Lens->CalibData.IsValid         = 0;
		Lens->Hall.I2CSlaveAddress      = LENS_ADDR_WR;
		Lens->Hall.I2CExpanderChannel   = Channel;
		if (Channel < 5)
		{
			Lens->CpldRegister          = 0x02;
			Lens->CpldBitMask           = 1 << Channel;
		}
		else
		{
			Lens->CpldRegister          = 0x03;
			Lens->CpldBitMask           = 1 << (Channel - 5);
		}

		Mirror->Type                    = ActuatorType_Mirror;
		Mirror->Channel                 = &Channels[Channel];
		Mirror->CalibData.IsValid       = 0;
		Mirror->Hall.I2CSlaveAddress    = MIRROR_ADDR_WR;
		Mirror->Hall.I2CExpanderChannel = Channel;
		if (Channel < 5)
		{
			Mirror->CpldRegister        = 0x04;
			Mirror->CpldBitMask         = 1 << Channel;
		}
		else
		{
			Mirror->CpldRegister        = 0x05;
			Mirror->CpldBitMask         = 1 << (Channel - 5);
		}

		if (i < 5)
		{
			// This module is a 70mm B camera.
			Lens->ActuatorData   = LensData70;
			Mirror->ActuatorData = MirrorData70;
		}
		else
		{
			// This module is a 150mm C camera.
			Lens->ActuatorData   = LensData150;
			Mirror->ActuatorData = MirrorData150;
		}

		Modules[i].IsCalibrated = 0;
		Modules[i].Lens         = Lens;
		Modules[i].Mirror       = Mirror;
	}

	// EXIT_FUNC

	return e;
}
void calibrate_lens(CamDevice_TypeDef* pCam, PiezoActuator* actuator)
{
	// At this point, we already know if the hall sensor is reversed

	uint16_t CPLD_select = pCam->CPLD_select;


	start_closed_control_lens(pCam, actuator, CPLD_select, 0x0000, TRUE, 0);
	vTaskDelay(10); // add settling time
	//pCam->retracted_hard_stop = ReadHallSensor(&(actuator->Hall));
	pCam->retracted_hard_stop = ReadHallSensor(&(actuator->Hall)) + 100; // testing

	start_closed_control_lens(pCam, actuator, CPLD_select, 0xFFFF, TRUE,0);
	vTaskDelay(10); // add settling time
	//pCam->extended_hard_stop = ReadHallSensor(&(actuator->Hall));
	pCam->extended_hard_stop = ReadHallSensor(&(actuator->Hall)) - 100; // testing

	log_debug("\nModule %s\n", pCam->Name);
	log_debug("Retracted hard stop : 0x%04x\n", pCam->retracted_hard_stop);
	log_debug("Extended  hard stop : 0x%04x\n", pCam->extended_hard_stop);
	log_debug("Range               : 0x%04x\n",
				abs(pCam->extended_hard_stop - pCam->retracted_hard_stop));

	// Save the hard stops
	PiezoModule* piezo_module = CameraIdToModule(pCam->camera_id);

	piezo_module->Lens->CalibData.NearPosition = pCam->retracted_hard_stop;
	piezo_module->Lens->CalibData.FarPosition  = pCam->extended_hard_stop;

	log_printf("\n\n");
}

void calibrate_mirror(CamDevice_TypeDef* pCam, PiezoActuator* actuator)
{
	uint16_t CPLD_select = pCam->CPLD_select;
    PiezoModule* piezo_module = CameraIdToModule(pCam->camera_id);
    uint16_t tmp;


    // Drive to the opposite (?) ends
    start_closed_control_mirror(pCam, actuator, CPLD_select, 0x0000,TRUE);
    pCam->wide_hard_stop = ReadHallSensor(&(actuator->Hall));

	start_closed_control_mirror(pCam, actuator, CPLD_select, 0xFFFF,TRUE);
	pCam->narrow_hard_stop = ReadHallSensor(&(actuator->Hall));

    if (pCam->narrow_hard_stop < pCam->wide_hard_stop)
    {
    	log_info("Module %s mirror hall sensor polarity is reversed!\n\n",
    			pCam->Name);
    	pCam->mirror_hall_polarity = HALL_POLARITY_REVERSED;
    	piezo_module->Mirror->Hall.Polarity = HALL_POLARITY_REVERSED;

    	tmp = pCam->narrow_hard_stop;
    	pCam->narrow_hard_stop  = pCam->wide_hard_stop;
    	pCam->wide_hard_stop = tmp;
    }

	log_debug("\nModule %s\n", pCam->Name);
	log_debug("Narrow hard stop : 0x%04x\n", pCam->narrow_hard_stop);
	log_debug("Wide hard stop : 0x%04x\n", pCam->wide_hard_stop);
	log_debug("Range               : 0x%04x\n",
				abs(pCam->wide_hard_stop - pCam->narrow_hard_stop));

	// Save the hard stops
    piezo_module->Mirror->CalibData.NearPosition = pCam->wide_hard_stop;
    piezo_module->Mirror->CalibData.FarPosition  = pCam->narrow_hard_stop;

	log_printf("\n\n");
}

Bool start_closed_control_lens(CamDevice_TypeDef* pCam, PiezoActuator* actuator,
		uint16_t CPLD_select, uint16_t destination, uint8_t is_calibration, uint8_t tolerance)
{
	uint8_t   direction = 0xFF;
	Bool rs = ETrue;
	uint16_t current_position = ReadHallSensor(&(actuator->Hall));
	uint16_t start_position = current_position;
	int32_t  displacement = destination - current_position;
	uint32_t iterations = 0;
	uint8_t  stuck_count = 0;
	uint32_t  delay; // in us
	uint8_t  type, current_type = 0xFF, current_direction = 0xFF;
	uint32_t  current_delay =0xFFFFFFFF; // in us
#if (STM_PWM == 1)
	uint16_t duty = DUTY_HI;
#else
	uint8_t  tx_buffer[4], rx_buffer[4];
#endif
#ifdef BOARD_VERSION_P1_1
	uint32_t multiplier = 1;
	float weight = 0.0;
#endif
	unsigned int actual_move = 1;

	log_printf("starting closed control for lenses:\n\n");

	/*
	The CPLD needs to be reconfigured if the control type is switching between
	coarse or fine, or if the direction is changing between extending and
	retracting. The CPLD does not need to be reconfigured if the module being
	controlled changes.
	*/

	/*
	Read the current hall sensor value, and take the difference with the
	destination hall sensor value. If the difference is greater than the coarse
	threshold, use coarse control. If not, use fine control.
	Continue sending pulses until the difference is less than the accuracy
	range. If you overshoot, switch directions and come back.
	*/

//#if 0 //LEAVE THIS HERE 
    if (!is_calibration)
    {
        /* If the destination is within 100 of either hard stop, reassign the
         * destination to prevent it from hitting the hard stop. */
        log_debug("destination = 0x%x\n", destination);
        if ((destination - actuator->CalibData.NearPosition) < 100) 
        // within 100 or past hard stop (negative)
        {
            destination = actuator->CalibData.NearPosition + 100;
            log_debug("Clamping destination to 0x%x!\n", destination);
        }
        else if ((actuator->CalibData.FarPosition - destination) < 100) 
        // within 100 or past hard stop (negative)
        {
            destination = actuator->CalibData.FarPosition - 100;
            log_debug("Clamping destination to 0x%x!\n", destination);
        }
    }
//#endif

    // engage REALLY STUPID CONTROL LOOP
    while (abs(displacement) > tolerance)
    /*
    Keep in mind - while same_config_settings is set to 0, the CPLD will be
    reprogrammed on EVERY SINGLE ITERATION. We'll have to add logic to speed
    this up later.
    */
    {

    	start_position = current_position = ReadHallSensor(&(actuator->Hall));
        log_time("start_position = 0x%x\n", start_position);
    	displacement = destination - current_position;
        log_time("destination = 0x%x\n", destination);

    	if (abs(displacement) <= tolerance)
    	{
    		break;
    	}

    	log_debug("enter loop itr\r\n");

        if ((destination == 0x0000) || (destination == 0xFFFF))
        {
            type = COARSE_CONTROL;
#ifdef BOARD_VERSION_P1_1
            type = HARD_STOP;
#endif
            delay = HARD_STOP_DELAY_PERIOD;
        }
        else
        {
            if (abs(displacement) > FINE_CONTROL_THRESHOLD)
            {
            	type = COARSE_CONTROL;
                //delay in usec
#ifdef BOARD_VERSION_P1
                delay = COARSE_DELAY_PERIOD;
#endif
#ifdef BOARD_VERSION_P1_1
                weight = actual_move ? (float)multiplier/ (float)actual_move : 0x1;
                weight = weight == 0 ? 1 : weight;
                log_debug("calculated weight %f \r\n", weight);
                multiplier = (uint32_t)((float)abs(displacement) * weight);
                log_debug("calculated multiplier 0x%x \r\n", (unsigned int)multiplier);
                multiplier =  multiplier > (4000 / P1_NUDGE_FINE_DELAY_PERIOD ) ?
                		(4000 / P1_NUDGE_FINE_DELAY_PERIOD ): (multiplier == 0 ? 1 : multiplier);
                delay = P1_NUDGE_FINE_DELAY_PERIOD * multiplier;
                log_debug("calculated delay 0x%x \r\n", (unsigned int)delay);
#endif
#if (STM_PWM == 1)
                duty = DUTY;
#endif
            }
            else
            {
                type = FINE_CONTROL;
#ifdef BOARD_VERSION_P1
                delay = FINE_DELAY_PERIOD;
#endif

#ifdef BOARD_VERSION_P1_1
                weight = actual_move ? (float)multiplier/ (float)actual_move : 0x1;
                weight = weight == 0 ? 1 : weight;
                log_debug("calculated weight %f \r\n", weight);
                multiplier =  (uint32_t)((float)abs(displacement) * weight);
                log_debug("calculated multiplier 0x%x \r\n", (unsigned int)multiplier);
                multiplier =  multiplier > (4000 / P1_NUDGE_FINE_DELAY_PERIOD ) ?
                		(4000 / P1_NUDGE_FINE_DELAY_PERIOD ): (multiplier == 0 ? 1 : multiplier);
                delay = P1_NUDGE_FINE_DELAY_PERIOD * multiplier;
                log_debug("calculated delay 0x%x \r\n", (unsigned int)delay);
#endif
#if (STM_PWM == 1)
                duty = DUTY_LO;
#endif
            }
        }

        if (displacement > 0)
            direction = EXTEND;
        else
            direction = RETRACT;

        if ((type != current_type) || (direction != current_direction) || (current_delay != delay))
        {
            current_type = type;
            current_direction = direction;
            current_delay = delay;
#if (STM_PWM == 1)
            actuator->CalibData.Frequency = FREQL;
            ConfigurePwm(&Pwm, actuator, duty, 1 - direction, 1);
#else
#ifdef BOARD_VERSION_P1_1
            configure_CPLD(current_type, current_direction, NULL, multiplier);
#endif
#ifdef BOARD_VERSION_P1
            configure_CPLD(current_type, current_direction, NULL, 0);
#endif

#endif
        }

        start_PWM_control(actuator, CPLD_select, 1, current_delay);
        current_position = ReadHallSensor(&(actuator->Hall));
        displacement = destination - current_position;
        actual_move =  abs(current_position - start_position);


        if (actual_move < 5 && abs(displacement) > 5)
        {
            stuck_count++;
            if (stuck_count > MAX_CONTROL_LOOP_ITERATIONS/2)
            {
                rs = EFalse;
                log_info("The actuator is not moving. Ending the control loop now. \n");
                break;
            }
        }

		if (!is_calibration)
		{
			if ((abs(current_position - actuator->CalibData.NearPosition) < 5 ) &&
				 (current_direction == RETRACT))
			{
				rs = EFalse;
				log_debug("%s: Current position = 0x%04x\n", pCam->Name, current_position);
				log_info("%s lens has hit the RETRACTED hard stop!\n", pCam->Name);
				break;
			}

			if ((abs(current_position - actuator->CalibData.FarPosition) < 5 ) &&
				 (current_direction == EXTEND))
			{
				rs = EFalse;
				log_debug("%s: Current position = 0x%04x\n", pCam->Name, current_position);
				log_info("%s lens has hit the EXTENDED hard stop!\n", pCam->Name);
				break;
			}
		}

		// Add a max number of iterations to prevent
		// infinite looping also see the accuracy
		if (iterations++ == MAX_CONTROL_LOOP_ITERATIONS)
		{
			rs = EFalse;
			log_printf("Maximum number of iterations exceeded. \n");
			break;
		}

	}

	current_position = ReadHallSensor(&(actuator->Hall));


    log_debug("Control loop complete for cam %s \n\n", pCam->Name);
    log_debug("Requested position : 0x%04x\n", destination);
    log_debug("Start position     : 0x%04x\n", start_position);
    log_debug("Current position   : 0x%04x\n", current_position);
    log_debug("Net motion         : 0x%04x\n\n", (int) abs(current_position -
															start_position));
    log_debug("RETRACTED hard stop : 0x%04x\n", actuator->CalibData.NearPosition);
    log_debug("EXTENDED hard stop  : 0x%04x\n", actuator->CalibData.FarPosition);

#if (STM_PWM == 1)
	TurnOffPwm(&Pwm, actuator);
#else
	// Switch CPLD modes again
	tx_buffer[0] = 0x02;
	tx_buffer[1] = 0x06;
	tx_buffer[2] = 0x00;
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);
#endif

	return rs;
}

int start_closed_control_mirror(CamDevice_TypeDef* pCam, PiezoActuator* actuator,
		uint16_t CPLD_select, uint16_t destination, uint8_t is_calibration)
{
	int8_t   direction = -1;
	uint16_t current_position = ReadHallSensor(&(actuator->Hall));
	uint16_t start_position = current_position;
	int32_t  displacement = destination - current_position;
	uint32_t iterations = 0;
	uint8_t  stuck_count = 0;
	uint32_t  delay; // in us
	uint32_t  current_delay =0xFFFFFFFF; // in us
	uint8_t  type, current_type = 0xFF, current_direction = 0xFF;
#if (STM_PWM == 1)
	uint16_t duty = DUTY_HI;
#else
	uint8_t  tx_buffer[4], rx_buffer[4];
#endif

#ifdef BOARD_VERSION_P1_1
	uint32_t multiplier = 1;
	float weight = 0.0;
#endif
	unsigned int actual_move = 1;
	int res = 0;

    log_printf("starting closed control for mirrors:\n\n");

    /*
    The CPLD needs to be reconfigured if the control type is switching between
    coarse or fine, or if the direction is changing between extending and
    retracting. The CPLD does not need to be reconfigured if the module being
    controlled changes.
    */

    /*
    /ead the current hall sensor value, and take the difference with the
    destination hall sensor value. If the difference is greater than the coarse
    threshold, use coarse control. If not, use fine control.
    Continue sending pulses until the difference is less than the accuracy
    range. If you overshoot, switch directions and come back.
    */

    while (abs(displacement) > ACCURACY_RANGE)
    {
    	start_position = current_position = ReadHallSensor(&(actuator->Hall));
    	displacement = destination - current_position;

    	if (abs(displacement) <= ACCURACY_RANGE)
    	{
    		break;
    	}

        if ((destination == 0x0000) || (destination == 0xFFFF))
        {
            type = COARSE_CONTROL;
#ifdef BOARD_VERSION_P1_1
            type = HARD_STOP;
#endif
            delay = HARD_STOP_DELAY_PERIOD;
        }
        else
        {
            if (abs(displacement) > FINE_CONTROL_THRESHOLD)
            {
                type = COARSE_CONTROL;
#ifdef BOARD_VERSION_P1
                delay = COARSE_DELAY_PERIOD;
#endif
#ifdef BOARD_VERSION_P1_1
                log_debug("calculated actual_move 0x%x \r\n", actual_move);
                weight = actual_move ? (float)multiplier/ (float)actual_move : 0x1;
                weight = weight == 0 ? 1 : weight;
                log_debug("calculated weight %f \r\n", weight);
                multiplier = (uint32_t)((float)abs(displacement) * weight);
                log_debug("calculated multiplier 0x%x \r\n", (unsigned int)multiplier);
                multiplier =  multiplier > (4000 / P1_NUDGE_FINE_DELAY_PERIOD ) ?
                		(4000 / P1_NUDGE_FINE_DELAY_PERIOD ): (multiplier == 0 ? 1 : multiplier);
                delay = COARSE_DELAY_PERIOD;
                log_debug("calculated delay 0x%x \r\n", (unsigned int)delay);
#endif
#if (STM_PWM == 1)
                duty = DUTY;
#endif
            }
            else
            {
                type = FINE_CONTROL;
#ifdef BOARD_VERSION_P1
                delay = FINE_DELAY_PERIOD;
#endif
#ifdef BOARD_VERSION_P1_1
                weight = actual_move ? (float)multiplier/ (float)actual_move : 0x1;
                weight = weight == 0 ? 1 : weight;
                log_debug("calculated weight %f \r\n", weight);
                multiplier =  (uint32_t)((float)abs(displacement) * weight);
                log_debug("calculated multiplier 0x%x \r\n", (unsigned int)multiplier);
                multiplier =  multiplier > (4000 / P1_NUDGE_FINE_DELAY_PERIOD ) ?
                		(4000 / P1_NUDGE_FINE_DELAY_PERIOD ): (multiplier == 0 ? 1 : multiplier);
                delay = FINE_DELAY_PERIOD;
                log_debug("calculated delay 0x%x \r\n", (unsigned int)delay);
#endif
#if (STM_PWM == 1)
                duty = DUTY_LO;
#endif
            }
        }

        if (displacement > 0)
            direction = WIDE;
        else
            direction = NARROW;

        if ((type != current_type) || (direction != current_direction) || (current_delay != delay))
        {
        	current_type = type;
        	current_direction = direction;
        	current_delay = delay;
#if (STM_PWM == 1)
	    	actuator->CalibData.Frequency = FREQM;
	    	ConfigurePwm(&Pwm, actuator, duty, 1 - direction, 1);
#else
#ifdef BOARD_VERSION_P1_1
			configure_CPLD(current_type, current_direction, NULL, multiplier);
#endif
#ifdef BOARD_VERSION_P1
			configure_CPLD(current_type, current_direction, NULL, 0);
#endif
#endif
        }

        start_PWM_control(actuator, CPLD_select, 0, current_delay);

        current_position = ReadHallSensor(&(actuator->Hall));
        displacement = destination - current_position;
        actual_move =  abs(current_position - start_position);
        log_debug("current position 0x%04x destination 0x%04x \n", current_position, destination);
        if (actual_move < 5 && abs(displacement) > 5)
        {
        	stuck_count++;
        	if (stuck_count > MAX_CONTROL_LOOP_ITERATIONS/2)
        	{
        		res = EFalse;
        		log_info("The actuator is not moving. Ending the control loop now. \n");
        		break;
        	}
        }

		if(!is_calibration)
		{
			if ((abs(current_position - actuator->CalibData.NearPosition) < 5 ) &&
				 (current_direction == WIDE))
			{
				res = EFalse;
				log_debug("%s: Current position = 0x%04x\n", pCam->Name, current_position);
				log_info("%s mirror has hit the WIDE hard stop!\n", pCam->Name);
				break;
			}

			if ((abs(current_position - actuator->CalibData.FarPosition) < 5 ) &&
				 (current_direction == NARROW))
			{
				res = EFalse;
				log_debug("%s: Current position = 0x%04x\n", pCam->Name, current_position);
				log_info("%s mirror has hit the NARROW hard stop!\n", pCam->Name);
				break;
			}
		}

        // Add a max number of iterations to prevent
        // infinite looping
        if (iterations++ == MAX_CONTROL_LOOP_ITERATIONS - 1)
        {
            log_printf("Maximum number of iterations exceeded. \n");
            res = 1;
            break;
        }
    }

    current_position = ReadHallSensor(&(actuator->Hall));

    log_debug("Control loop complete for cam %s \n\n", pCam->Name);
    log_debug("Requested position : 0x%04x\n", destination);
    log_debug("Start position     : 0x%04x\n", start_position);
    log_debug("Current position   : 0x%04x\n", current_position);
    log_debug("Net motion         : 0x%04x\n\n", (int) abs(current_position -
															start_position));

	log_debug("WIDE hard stop      : 0x%04x\n", actuator->CalibData.NearPosition);
	log_debug("NARROW hard stop    : 0x%04x\n", actuator->CalibData.FarPosition);

#if (STM_PWM == 1)
	TurnOffPwm(&Pwm, actuator);
#else
	// Switch CPLD modes again
	tx_buffer[0] = 0x02;
	tx_buffer[1] = 0x06;
	tx_buffer[2] = 0x00;
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);
#endif

	return res;
}

void start_PWM_control(PiezoActuator* actuator, uint16_t CPLD_select, uint8_t is_lens, uint32_t delay)
{
	// By the time this function gets called, the CPLD should already be
	// configured to send pulses.
	uint8_t tx_buffer[4], rx_buffer[4];
	log_debug("delay in usec 0x%x \r\n",(unsigned int) delay);
	/* Enable the corresponding PWM channel that goes to the NJW IC */
	GPIO_SetBits(actuator->Channel->Port, actuator->Channel->Pin);

	// configure for write
	tx_buffer[0] = 0x02;

	// At the moment, I try not to move lenses and mirrors at the same time,
	// to prevent the possibility of them crashing into each other.
	if (is_lens)
	{
		// write the B module selection register for lenses
		tx_buffer[1] = 0x02;
		tx_buffer[2] = (uint8_t) (CPLD_select & 0xFF);
		CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

		// write the C module selection register for lenses
		tx_buffer[1] = 0x03;
		tx_buffer[2] = (uint8_t) ((CPLD_select >> 8) & 0xFF);
		CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

		// clear the B module selection register for mirrors
		tx_buffer[1] = 0x04;
		tx_buffer[2] = (uint8_t) 0;
		CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

		// clear the C module selection register for mirrors
		tx_buffer[1] = 0x05;
		tx_buffer[2] = (uint8_t) 0;
		CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);
	}
	else // is mirror
	{
		// clear the B module selection register for lenses
		tx_buffer[1] = 0x02;
		tx_buffer[2] = (uint8_t) 0;
		CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

		// clear the C module selection register for lenses
		tx_buffer[1] = 0x03;
		tx_buffer[2] = (uint8_t) 0;
		CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

		// write the B module selection register for mirrors
		tx_buffer[1] = 0x04;
		tx_buffer[2] = (uint8_t) (CPLD_select & 0xFF);
		CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

		// write the C module selection register for mirrors
		tx_buffer[1] = 0x05;
		tx_buffer[2] = (uint8_t) ((CPLD_select >> 8) & 0xFF);
		CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);
	}

	// Write a 1 to the startPWM address
	tx_buffer[1] = 0x07;
	tx_buffer[2] = 0x01;
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);
	DelayUSec(1000);

	// Wait for the actuator to finish moving
	// The delay is proportional to the pulse train we are sending
	DelayUSec(delay);

    /* 2016-05-05: Found an issue where disabling the NJW before
     * turning off the CPLD led to issues. If the last pulse generated
     * by the CPLD is high, and the NJW is shut off, the voltage will
     * slowly drain downwards due to the capcitors in the circuit */

	// Disable the CPLD's waveform generation
	// Write a 1 to the stopPWM address
	tx_buffer[1] = 0x08;
	tx_buffer[2] = 0x01;
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

	// Disable the NJW IC
	GPIO_ResetBits(actuator->Channel->Port, actuator->Channel->Pin);
}

#if LOG_CALIB_DATA
#define NUM_MOVE_POINTS 500
#define NUM_SENSITIVITIES 4
#define NUM_POLARITIES 2
// Indexing: [actuator][polarity][sensitivity][dir][npoints]
UInt16 MoveHallData[2][NUM_POLARITIES][NUM_SENSITIVITIES][2][NUM_MOVE_POINTS];
//unsigned MoveTimestamp[2][NUM_POLARITIES][NUM_SENSITIVITIES][2][NUM_MOVE_POINTS];
int MoveCount[2][NUM_POLARITIES][NUM_SENSITIVITIES][2];

static int CalibActuatorIndex;
static int CalibPolarity;
static int CalibSensitivity;

#endif // #if LOG_CALIB_DATA

// Preconditions: Set the Hall sensitivity if you care about the readings.
void MoveToEnd(PiezoPwm *pwm, PiezoActuator *a, UInt8 Direction, UInt16 DutyCycle)
{
#if LOG_CALIB_DATA
	unsigned time;
	unsigned finish;
	// int i;
	int count = 0;
	int missed = 0;
	UInt16 value;
	HallSensor *h;
	//ENTER_FUNC

	h = &(a->Hall);
#endif // #if LOG_CALIB_DATA

	SetCpld(a);

	ConfigurePwm(pwm, a, DutyCycle, Direction, 0);

#if LOG_CALIB_DATA
	time = xTaskGetTickCount();
	finish = time + 2 * 500 * portTICK_RATE_MS;

	TurnOnPwm(pwm, a);

	while (time < finish)
	{
		time = xTaskGetTickCount();
		value = ReadHallSensor(h);

		if (count < NUM_MOVE_POINTS)
		{
			MoveHallData [CalibActuatorIndex][CalibPolarity][CalibSensitivity][Direction][count] = value;
			//MoveTimestamp[CalibActuatorIndex][CalibPolarity][CalibSensitivity][Direction][count] = time;
			count++;
		}
		else
		{
			missed++;
		}
	}

	TurnOffPwm(pwm, a);

	MoveCount[CalibActuatorIndex][CalibPolarity][CalibSensitivity][Direction] = count;

#else // #if LOG_CALIB_DATA

	TurnOnPwm(pwm, a);

	vTaskDelay(TUNETIME);

	TurnOffPwm(pwm, a);

#endif // #if LOG_CALIB_DATA

	// EXIT_FUNC
}

#if LOG_CALIB_DATA

void DumpCalibLogData(void);

// Trickle out log data.
void DumpCalibLogData()
{
	int i;
	int a;
	int p;
	int s;
	int d;
	int max_count = 0;

	// Output the header data
	log_printf("Calibration Data\n\r");

	log_printf("Sample,");
	for (a = 0; a < 2; a++)
	{
		for (p = 0; p < NUM_POLARITIES; p++)
		{
			for (s = 0; s < NUM_SENSITIVITIES; s++)
			{
				for (d = 0; d < 2; d++)
				{
					log_printf("%s p%ds%d %s,",
							a ? "Mirror" : "Lens",
									p, s,
									d ? "Far" : "Near");
					max_count = MAX(max_count, MoveCount[a][p][s][d]);
				}
			}
		}
	}
	//log_printf("\n\r");
	log_printf(" \n");

	for (i = 0; i < max_count; i++)
	{
		log_printf("%d,", i);
		for (a = 0; a < 2; a++)
		{
			for (p = 0; p < NUM_POLARITIES; p++)
			{
				for (s = 0; s < NUM_SENSITIVITIES; s++)
				{
					for (d = 0; d < 2; d++)
					{
						if (i < MoveCount[a][p][s][d])
						{
							log_printf("%d,", MoveHallData[a][p][s][d][i]);
						}
						else
						{
							log_printf(", ");
						}
					}
				}
			}
		}

		log_printf("\n\r");

		vTaskDelay(10);
		if ((i % 10) == 0)
		{
			taskYIELD();
		}
	}
}

#endif // #if LOG_CALIB_DATA

static void ScanHallSensitivity(HallSensor *h, UInt16 *Data, UInt16 Min, UInt16 Max)
{
	UInt16 s;
	for(s = Min; s <= Max; s++)
	{
		SetHallSensorSensitivity(h, s);
		Data[s] = ReadHallSensor(h);
	}
}

// Newer version which moves back & forth only once and measures all sensitivities at single positions.
// TODO: REFACTOR THIS MESS!!!
// NOTE: Use the old version of this code to profile the actuator movement & Hall data.
// TODO: Refactor _that_ code into a separate function.

Error_t cali_as5510(PiezoActuator *a, UInt16 DutyCycle)
{
	// set the offset, sensitivity-level, resolution of AF module
	// "id" range is from 0 to 15
	// when return, str_afm[].min, max, offset, level and resolution are modified
	// also str_afm[] is not critical section in this routine.
	int		j, /*k,*/ lv, diff, min, /*mn,*/ max, /*mx,*/ cnt, min_stroke;
	int     HallStroke;
	// int     SpinCount;
	int     BestHallStroke, BestHallNear = 0, BestHallFar = 0, BestHallSensitivity = 0;
	int     BestHallPolarity = 0;
	int     Offset, Resolution, Sensitivity;
	int     MinSensitivity = 0;
	int     MaxSensitivity = 0;
	UInt16  HallNear[NUM_SENSITIVITIES];
	UInt16  HallFar [NUM_SENSITIVITIES];
	UInt32	l;
	HallSensor *h;
	Error_t	    e = ERROR_NONE;

	h = &(a->Hall);

	min_stroke = a->ActuatorData.MinStroke;
	h->Polarity = 0;   // reset the Polarity

	InitHallSensor(h); // Slow sampling, no polarity reversal

	CalibDebug("\n  ...1st Calibration...");
	cnt = lv = diff = min = max = 0;
	BestHallStroke = 0;

	// Set the starting point to one appropriate for the actuator type.
	// MinSensitivity = (a->Type == ActuatorType_Lens) ? 1 : 2;
	MinSensitivity = (a->Type == ActuatorType_Lens) ? 0 : 0;
	MaxSensitivity = NUM_SENSITIVITIES - 1;

	// Collect data for the first pass.

	// Read the Hall sensor at the nearest position.
	MoveToEnd(&Pwm, a, NEAR, DutyCycle);
	ScanHallSensitivity(h, HallNear, MinSensitivity, MaxSensitivity);

	// Read the Hall sensor at the farthest position.
	MoveToEnd(&Pwm, a, FAR, DutyCycle);
	ScanHallSensitivity(h, HallFar, MinSensitivity, MaxSensitivity);

	// Analyze the data to select the best sensitivity & polarity for
	// full-range operation.
	for(Sensitivity = MinSensitivity; Sensitivity <= MaxSensitivity; Sensitivity++)
	{
		UInt16 Near = HallNear[Sensitivity];
		UInt16 Far  = HallFar [Sensitivity];
		UInt16 Stroke;

		CalibDebug("  S:%d Near:0x%04x Far:0x%04x",
				Sensitivity, Near, Far);

		if(Near < Far)
		{
			h->Polarity = 0;
		}
		else
		{
			// Change polarity & invert data.
			h->Polarity = 1;
			// InitHallSensor(h); // Reinit
			Near = 1023 - Near;
			Far  = 1023 - Far;
		}

		HallStroke = Far - Near;

		// Check to see that the stroke is neither too wide nor too narrow.
		// Just for the lens, also require the hard stops are within the sensor range.
		if (((a->Type == ActuatorType_Lens) && ((Near <= 0) || (Far >= 1023))) ||
			(HallStroke < min_stroke))
		{
			cnt++;
			continue;
		}

		if(HallStroke > BestHallStroke)
		{

			BestHallStroke      = Stroke;
			BestHallNear        = Near;
			BestHallFar         = Far;
			BestHallSensitivity = Sensitivity;
			BestHallPolarity    = h->Polarity;
		}
	}

	// Check criteria for a valid calibration.
	if(cnt >= ((a->Type == ActuatorType_Lens) ? 3 : 2))
		return ERROR_CALIBRATION_FAILURE;

	Offset      = BestHallNear; // + HOMEOFFSET=0; //home position ahead of 0 pulses
	Sensitivity = BestHallSensitivity;

	// Resolution = 1000 * full_stroke / ADC_range_used
	j = BestHallFar - BestHallNear;
	l = (UInt32)a->ActuatorData.FullStroke*1000 / (UInt32)j;
	Resolution = (int)l;
	CalibDebug("-->sens=%d res=%dnm (%d:%d)", Sensitivity, (int) Resolution, BestHallNear-511, BestHallFar-511);
	log_printf("(0x%04x, 0x%04x)[s=%d,p=%d]\n", BestHallNear, BestHallFar, BestHallSensitivity, h->Polarity);

#if 0
	// For now, skip the 2nd calibration step and just perform calculations.

	/*
	 * 2nd calibration
	 */
	log_debug("\n  ...2nd Calibration...");
	// move from the physical far position to the real far position
	// the current position is at the physical far position
	SetHallSensorSensitivity(h, Sensitivity);

	// convert the actual far position to encoder value
	l  = a->ActuatorData.HomePosition + a->ActuatorData.RealStroke;
	k  = (int)(l*1000 / Resolution);				// "k" contains the encoder value
	k += Offset;

	log_debug(" Far-position(%d)->", k-511);

	// First leg
	DutyCycle = DUTY_LO;
	(void) ConfigurePwm(&Pwm, a, DutyCycle, NEAR, 1);   // Far position

	for(SpinCount = 0; SpinCount < 2000; SpinCount++)
	{
		// k
		mx = ReadHallSensor(h);
		logData[SpinCount] = mx - 511;

		//if((k+40) >= mx)
		if((k+5) >= mx)
			break;
	}
	TurnOffPwm(&Pwm, a); // disable the PWM Module
#if 0
	// Second leg (refactor!)
	//DutyCycle = (DUTY_LO * 3) / 4;
	//(void) ConfigurePwm(&Pwm, a, DutyCycle, NEAR, 1);   // Far position
	TurnOnPwm(&Pwm, a);

	for(; SpinCount < 2000; SpinCount++)
	{
		// k
		mx = ReadHallSensor(h);
		logData[SpinCount] = mx - 511;

		if((k+2) >= mx)
			break;
	}
	TurnOffPwm(&Pwm, a); // disable the PWM Module
	logData[SpinCount] = ReadHallSensor(h) - 511; // Once stopped.
#endif

	for(i = 0; i <= SpinCount; i++)
	{
		log_debug("%d: %d", i, logData[i]);
	}

	j = k - mx;
//	if(j > 3 || SpinCount >= 2000)
	if(j > 10 || SpinCount >= 2000)
	{
		// position accuracy
		log_error("Calibration failure!  SpinCount exceeded.");
		log_debug("     j:%d   k:%d  mx:%d  SpinCount: %d", j, k-511, mx-511, SpinCount);
		return ERROR_CALIBRATION_FAILURE;
	}

	// seek the optimal sensitivity level at the actual far-position
	ScanHallSensitivity(h, HallFar, MinSensitivity, MaxSensitivity);

	// move to the real home position
	// the current position is at the real far position
	// convert the actual home position to encoder value
	SetHallSensorSensitivity(h, Sensitivity);
	l = a->ActuatorData.HomePosition;
	k = (int)(l*1000 / Resolution);				// "k" contains the encoder value
	k += Offset;

	log_debug(" Home-position(%d)->", k-511);


	// First leg
	DutyCycle = DUTY_LO;
	(void) ConfigurePwm(&Pwm, a, DutyCycle, NEAR, 1);   // Far position

	for (SpinCount = 0; SpinCount < 2000; SpinCount++)
	{
		mn = ReadHallSensor(h);
		logData[SpinCount] = mn - 511;

		if((k+5) >= mn)
			break;
	}
	TurnOffPwm(&Pwm, a);						// disable the PWM Module

#if 0
	// Second leg (refactor!)
	//DutyCycle = (DUTY_LO * 3) / 4;
	//(void) ConfigurePwm(&Pwm, a, DutyCycle, NEAR, 1);   // Far position
	TurnOnPwm(&Pwm, a);

	for(; SpinCount < 2000; SpinCount++)
	{
		// k
		mn = ReadHallSensor(h);
		logData[SpinCount] = mn - 511;

		if((k+2) >= mn)
			break;
	}
	TurnOffPwm(&Pwm, a); // disable the PWM Module
	logData[SpinCount] = ReadHallSensor(h) - 511; // Once stopped.
#endif

	for(i = 0; i <= SpinCount; i++)
	{
		log_debug("%d: %d", i, logData[i]);
	}

	j = k - mn;

	if (SpinCount >= 2000 || j > 10)
	{
		log_error("Calibration failure!  SpinCount2 exceeded.");
		log_debug("     j:%d   k:%d  mn:%d  SpinCount: %d", j, k-511, mn-511, SpinCount);
		return ERROR_CALIBRATION_FAILURE;
	}

	// seek the optimal sensitivity level at the actual home-position
	ScanHallSensitivity(h, HallNear, MinSensitivity, MaxSensitivity);

	for(i=cnt=diff=0; i<4; i++) { // seek the optimum sensitivity level
//	for(i=cnt=diff=0; i<3; i++) { // seek the optimum sensitivity level
		if(i==3)
		{
			// Check for oversaturated measurements.
			// to avoid AS5510 error
			j = (HallFar[i]-511) + ((HallFar[i]-511)/3);
			if(j > 503) {
				cnt++;
				continue;
			}

			j = (HallNear[i]-511) + ((HallNear[i]-511)/3);
			if(j < -503) {
				cnt++;
				continue;
			}
		}
		k = HallFar[i] - HallNear[i];

		if((HallNear[i] <= 0) || (HallFar[i] >= 1023) || (k < min_stroke))
		{
			// Stroke is too wide or too narrow
			cnt++;
			continue;
		}
		if(k > diff) {
			BestHallStroke      = k;
			BestHallNear        = HallNear[i];
			BestHallFar         = HallFar[i];
			BestHallSensitivity = i;
		}
	}

	if(cnt >= 4)
	{
		log_error("Calibration failure!  No valid data points.");
		return ERROR_CALIBRATION_FAILURE;
	}

	// move to the real home position
	a->ActuatorData.HomePosition = 0; // AKA t_posi
	a->CalibData.Offset = BestHallNear;
	a->CalibData.Sensitivity = BestHallSensitivity;
	l = (UInt32)a->ActuatorData.RealStroke*1000 / (UInt32)BestHallStroke;
	a->CalibData.Resolution = (int)l;

#else
	// Hacked 2nd calibration.

	a->ActuatorData.HomePosition = 0; // AKA t_posi
	a->CalibData.Offset = Offset + (1000 * a->ActuatorData.HomePosition) / Resolution;
	a->CalibData.Sensitivity = BestHallSensitivity;
	a->CalibData.Resolution = (Resolution * a->ActuatorData.RealStroke) / a->ActuatorData.FullStroke;

#endif

	// set the sensitivity level and polarity of magnet
	h->Polarity = BestHallPolarity;
	InitHallSensor(h);
	SetHallSensorSensitivity(h, a->CalibData.Sensitivity);

	CalibDebug("Offset:%d  Resolution: %d", a->CalibData.Offset, a->CalibData.Resolution);

	return e;
}

Error_t CalibrateModule(PiezoModule *m, UInt16 Iterations, UInt16 DutyCycle)
{
	int i;
	Error_t e = ERROR_NONE;

	// ENTER_FUNC

	log_debug("CalibrateModule: Starting calibration for module %s with %d iterations", CameraNames[m->CameraId], Iterations);

	CHECK_ERROR(m == NULL ? ERROR_INVALID_ARG : ERROR_NONE);

#if 0
	// Disable this for now to allow repeated calibration.
	if (m->IsCalibrated) return e;
#endif

	for (i = 0; i < Iterations; i++)
	{
		// Set up the Hall sensors with default polarity.
		m->Lens->Hall.Polarity = 0;
		m->Mirror->Hall.Polarity = 0;
		InitHallSensor(&(m->Lens->Hall));
		InitHallSensor(&(m->Mirror->Hall));

		// Step 1: Read data from the EEPROM.
		e = ReadEepromCalibration(m->CameraId, &m->Lens->CalibData, &m->Mirror->CalibData);
		if (e)
		{
			log_error("Error reading EEPROM calibration data for camera %d", m->CameraId);
			return e;
		}

#if CALIB_DEBUG
		CalibDebug("After reading EEPROM data:");
		PrintPiezoCalibrationData(&(m->Lens->CalibData));
		PrintPiezoCalibrationData(&(m->Mirror->CalibData));
#endif

		// Step 2: Move Mirror out of the way.
		CalibDebug("\nMoving mirror out of the way.");
		SetCurrentActuator(m->Mirror);
		MoveToEnd(&Pwm, m->Mirror, NEAR, DutyCycle);
		CalibDebug("\nMoving lens out of the way.");
		SetCurrentActuator(m->Lens);
		MoveToEnd(&Pwm, m->Lens, FAR, DutyCycle);

		// Step 3: Calibrate the lens.
		CalibDebug("\nCalibrating lens.");
		SetCurrentActuator(m->Lens);
		log_printf("Module %s  LENS   ", CameraNames[m->CameraId]);

		e = cali_as5510(m->Lens, DutyCycle);
		if(e)
		{
			log_error("Calibration failed");
			// Continue with calibration anyway.
		}

		CalibDebug("After calibrating lens:  Lens Calibration:");
		PrintPiezoCalibrationData(&(m->Lens->CalibData));

		CalibDebug("\nMoving lens out of the way.");
		SetCurrentActuator(m->Lens);
		MoveToEnd(&Pwm, m->Lens, NEAR, DutyCycle);
		// PiezoMoveToPosition(m->Lens, 512, 5, DUTY_HI);

		// Step 4: Move the Lens to home.
		// TODO!

		// Step 5: Calibrate the mirror.
		CalibDebug("\nCalibrating mirror.");
		SetCurrentActuator(m->Mirror);
		log_printf("Module %s  MIRROR ", CameraNames[m->CameraId]);

		e = cali_as5510(m->Mirror, DutyCycle);
		if(e)
		{
			log_error("Calibration failed");
			// Continue with calibration anyway.
		}

#if CALIB_DEBUG
		CalibDebug("After calibrating mirror:  Mirror calibration:");
		PrintPiezoCalibrationData(&(m->Mirror->CalibData));
#endif

	}

	m->IsCalibrated = 1;

#if LOG_CALIB_DATA
	DumpCalibLogData();
#endif

	// EXIT_FUNC

	PrintHallRetryStats();

	return e;
}

void AF_control_Config(void)
{
	log_debug("Starting AF_control_Config()");

	// Initialization
	InitPiezoModules(); // Data structures only - no HW communication yet.
	InitPiezoHw(Channels);

	// TODO: Add more of these....
	CurrentActuator = NULL; // For now, just to keep the compiler happy.
	// End new init code

	ResetCpld();

	check_connect();

	xTaskCreate(	 vPID_Control,
					(const signed char * const)"AF Control",
					200,
					NULL,
					1,
					&xHandle_PID_Control);

	vTaskSuspend( xHandle_PID_Control );

	xTaskCreate(vPiezoMonitor,
				(const signed char * const) "Piezo Monitor",
				200,
				NULL,
				1,
				&xHandle_PiezoMonitor);
}

Bool check_connect(void)
{
	int            i;
	Bool           IsFailure = EFalse;
	PiezoActuator *Lens;
	PiezoActuator *Mirror;

	ENTER_FUNC

	for (i = 0; i < NUM_PIEZO_MODULES; i++)
	{
		Lens   = Modules[i].Lens;
		Mirror = Modules[i].Mirror;

		Lens  ->IsConnected = CheckHallConnectivity(&(Lens->Hall));
		Mirror->IsConnected = CheckHallConnectivity(&(Mirror->Hall));

		if (!Lens->IsConnected)
		{
			log_error("%s Lens is not connected.", CameraNames[Modules[i].CameraId]);
			IsFailure = ETrue;
		}

		if (!Mirror->IsConnected)
		{
			log_error("%s Mirror is not connected.", CameraNames[Modules[i].CameraId]);
			IsFailure = ETrue;
		}
	}

	EXIT_FUNC

	return IsFailure;
}

#define NSTEPS 10
Int16  PosErr[NSTEPS];
UInt16 Pos   [NSTEPS];
int    Pid   [NSTEPS];
Int16  Duty  [NSTEPS];
Error_t MovePiezoToPosition(PiezoActuator *a, UInt16 TargetPosition, UInt16 Tolerance, UInt16 DutyCycle)
{
	HallSensor *h;
	Error_t  e = ERROR_NONE;

	// TODO: Error checking.

	h = &(a->Hall);
	log_debug("TargetPosition = %d", TargetPosition);

	SetCurrentActuator(a);

	// Configure the Hall sensor
	InitHallSensor(h);
	SetHallSensorSensitivity(h, a->CalibData.Sensitivity);

	// TODO: Better naming for the internal routine....
	e = PiezoMoveToPosition(a, TargetPosition, Tolerance, DutyCycle);

	return e;
}

/*
 *  John's strategy for hacking together PID control:
 *  1. Port PZT's PID control loop into a function for providing a good duty cycle
 *     for a single update step.
 *  2. Build a simple state machine for managing the inner loop of the controller.
 *  3. Wrap the state machine into a task that wakes up when there is work to complete.
 *
 *  Later, provide support for canceling long-running requests.
 */

#define ICLAMP 50

#undef DUTY_LO
#define DUTY_LO 90

void vPID_Control(void *pvParameters)
{
#if 0
	uint16_t timer_stop=0;
	Bool timer_stop_l=0;
	int16_t P;
	static int16_t D=0,I=0;
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1000/F_PID;
    UInt16 Position;

    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
    	if (CurrentActuator)
		{
    		Position = ReadHallSensor(&(CurrentActuator->Hall));
			e_pid = pid_input - Position;
			P = KP*(float)e_pid;
			I += KI*(float)e_pid;
			D = KD*e_pid - D;
			I = CLAMP(I, -ICLAMP, ICLAMP)
			pid_value = (P + I + D);
			pid_value = CLAMP(pid_value, -DUTY_HI, DUTY_HI);

			if(pid_value>0)
			{
				str_afm[channel].dir = FAR;
				str_afm[channel].duty = (uint16_t)(pid_value+DUTY_LO);
			}
			else
			{
				str_afm[channel].dir = NEAR;
				str_afm[channel].duty = (uint16_t)(abs(pid_value)+DUTY_LO);
			}
			PWM_control(channel);

			if(((e_pid>0)&&(e_pid<STOPBEFORE))||((e_pid<0)&&(e_pid>-STOPBEFORE)))
			{
				timer_stop_l = 1;
			}
			if(timer_stop_l)
			{
				if(timer_stop++>50)
				{
					// js (to build) off_PWM(channel);
					vTaskSuspend( xHandle_PID_Control );
					timer_stop_l = 0;
					timer_stop = 0;
				}
			}
		}

    	vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
#endif
}


void SetPiezoMonitor(int on, unsigned int bitmask)
{
	EnablePiezoMonitor = on;
	PiezoMonitorBitmask = bitmask;
}

// When enabled, log the status of the piezo code & hall sensors.
// Note lots of ugliness here - utility first!
// Global vars control its execution - be sure they're volatile.
void vPiezoMonitor(void *pvParameters)
{
	while(1)
	{
		if (EnablePiezoMonitor)
		{
			int            i;

			if (EnablePiezoMonitor != LastEnablePiezoMonitor)
			{
				// Output csv header
				log_printf("Piezo Monitor\n");
				for (i = 0; i < NUM_PIEZO_MODULES; i++)
				{
					if (GET_FIELD(PiezoMonitorBitmask, Modules[i].CameraId : Modules[i].CameraId))
					{
						log_printf("%s Lens,%s Mirror,",
							CameraNames[Modules[i].CameraId],
							CameraNames[Modules[i].CameraId]);
					}
				}
				log_printf("\n\r");

				LastEnablePiezoMonitor = EnablePiezoMonitor;
			}

			for (i = 0; i < NUM_PIEZO_MODULES; i++)
			{
				UInt16 LensPosition;
				UInt16 MirrorPosition;

				if (GET_FIELD(PiezoMonitorBitmask, Modules[i].CameraId : Modules[i].CameraId))
				{
					LensPosition   = ReadHallSensor(&(Modules[i].Lens  ->Hall));
					MirrorPosition = ReadHallSensor(&(Modules[i].Mirror->Hall));

					log_printf("0x%04x,0x%04x,", LensPosition, MirrorPosition);
				}
			}
			log_printf("\n\r");

			vTaskDelay(500 / portTICK_RATE_MS); // Wait for 500ms.
		}
		else
		{
			vTaskDelay(2000 / portTICK_RATE_MS); // Wait for 2 seconds.
		}
	}
}

void SPI_Transfer(uint8_t* dataW, uint8_t* dataR, uint16_t num_bytes)
{
	SS_EN;
	for(int i=0;i<num_bytes;i++)
	{
		CPLD->DR = dataW[i]; // write data to be transmitted to the SPI data register
		while( !(CPLD->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
		while( !(CPLD->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
		while( CPLD->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
		dataR[i] = CPLD->DR; // return received data from SPI data register
	}
	SS_DIS;
}

Error_t SetFocusCalibration(PiezoModule *m, UInt32 FocusDistance, UInt16 FocusPosition)
{
    Error_t e = ERROR_NONE;

    log_debug("FD=%d mm  FP=%d um", (UInt16) FocusDistance, FocusPosition);


    return e;
}

