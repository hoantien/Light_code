/**
  ******************************************************************************
  * \file    cam_ctrl.h
  * \author  Infonam Embedded Team
  * \version V1.1.0
  * \date    05-Mar-2015
  * \brief   Header file of CCB Command Parsers related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CCB_CAM_CTRL_H
#define __CCB_CAM_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "log.h"
#include <stdio.h>
#include <string.h>
#include "mems.h"
#include "hal_i2c_ex.h"

/* Exported define ------------------------------------------------------------*/

#define CONVERT_ENDIANNESS(w, x, y, z)  (z << 24) || (y << 16)|| (x << 8) || w

#define CONVERT_ENDIANNESS_2_BYTE(x)	((uint16_t)((((x) >> 8) & 0x00ff) | \
													(((x) << 8) & 0xff00)))

#define CONVERT_ENDIANNESS_4_BYTE(x)	((uint32_t)((((x) >> 24) & 0x000000ff)|\
													(((x) >> 8)  & 0x0000ff00)|\
													(((x) << 8)  & 0x00ff0000)|\
													(((x) << 24) & 0xff000000)))
#ifndef BUILD_ID
#define BUILD_ID 123
#endif

#define LITTLE_ENDIAN                 0
#define BIG_ENDIAN                    1

#define MAX_CAM_TID_LOG_NUM	         64
#define CAM_NUM_CAMERA_MAX	         16			// STM_CCB: 4channel
#define A1_MODULE_SEL		         0x1
#define A5_MODULE_SEL		         0x2
#define B4_MODULE_SEL		         0x4
#define C5_MODULE_SEL		         0x8

#define SW_STANDBY	                 0x02
#define HW_STANDBY	                 0x01
#define CLOSE		                 0x00

#define CAM_CMD_QUEUE_SIZE	         128
#define FPGA_MAX_DATA_SIZE            64

#define CPLD_CONFIG_SIZE              50
#define FINE_CONTROL_THRESHOLD        10
#define ACCURACY_RANGE                 0
#define MAX_CONTROL_LOOP_ITERATIONS   20
#define USER_DEFINED_CONTROL           0
#define COARSE_CONTROL                 1
#define FINE_CONTROL                   2
#define HARD_STOP                      3
#define RETRACT                        0
#define EXTEND                         1
#define NARROW                         0
#define WIDE                           1
#ifdef BOARD_VERSION_P1
#define FINE_DELAY_PERIOD              (1 * 1000)
#define COARSE_DELAY_PERIOD            (2 * 1000)
#define HARD_STOP_DELAY_PERIOD        (50 * 1000)
#define NUDGE_FINE_DELAY_PERIOD       (16 * 1000)
#define P1_NUDGE_FINE_DELAY_PERIOD        64
#endif

#ifdef BOARD_VERSION_P1_1
#define FINE_DELAY_PERIOD              (4 * 1000)
#define COARSE_DELAY_PERIOD            (4 * 1000)
#define HARD_STOP_DELAY_PERIOD        (50 * 1000)
#define NUDGE_FINE_DELAY_PERIOD       (64)
#define P1_NUDGE_FINE_DELAY_PERIOD        32
#endif

#ifdef BOARD_VERSION_P1
#define TEMP_SENSOR_RESET			0x0000
#define TEMP_SENSOR_RESET_DATA		0x0021
#define TEMP_SENSOR_START_READ		0x0011
#define TEMP_SENSOR_CTRL_REG		0x30B4
#define TEMP_SENSOR_DATA_REG		0x30B2
#define TEMP_SENSOR_CAL1_REG		0x3FC6
#define TEMP_SENSOR_CAL2_REG		0x3FC8
#else /* BOARD_VERSION_P1_1 */
#define TEMP_SENSOR_RESET			0x0000
#define TEMP_SENSOR_RESET_DATA		0x0021
#define TEMP_SENSOR_START_READ		0x0011
#define TEMP_SENSOR_CTRL_REG		0x3126
#define TEMP_SENSOR_DATA_REG		0x3124
#define TEMP_SENSOR_CAL1_REG		0x3128
#define TEMP_SENSOR_CAL2_REG		0x312A
#endif

/* Exported typedef -----------------------------------------------------------*/

/**
 * \brief  Camera focal length type enumeration
 */
typedef enum {
	CAM_TYPE_35MM  = 0,
	CAM_TYPE_70MM  = 1,
	CAM_TYPE_150MM = 2,
} CamType_TypeDef;
/**
 * \brief	Camera identifier
 * A2 A1 A4 A5 A3 B4 B5 B3 B2 B1 C1 C2 C4 C3 C6 C5
 */
typedef enum {
	CAM_ID_A1 = 1,
	CAM_ID_A2,
	CAM_ID_A3,
	CAM_ID_A4,
	CAM_ID_A5,
	CAM_ID_B1,
	CAM_ID_B2,
	CAM_ID_B3,
	CAM_ID_B4,
	CAM_ID_B5,
	CAM_ID_C1,
	CAM_ID_C2,
	CAM_ID_C3,
	CAM_ID_C4,
	CAM_ID_C5,
	CAM_ID_C6,
	CAM_ID_MAX /* For checking error */
} CamID;

/**
 * \brief  Camera command status type definition
 */
typedef enum {
	CMD_UNKNOWN                             = 0x00000000,
	CMD_SUCCESS                             = 0x00000001,
	CMD_PENDING                             = 0x00000002,
	CMD_INVALID_ARG                         = 0x00000004,
	ERROR_INVALID_MBITMASK                  = 0x00000008,
	ERROR_ASIC_UNAVAILABLE                  = 0x00000010,
	ERROR_MODULE_FAULT                      = 0x00000020,
	ERROR_CAM_MODULE_RESOLUTION_INVALID_ARG = 0x00000040,
	ERROR_EEPROM_FAULT                      = 0x00000080,
	/* Need to add this ERROR to ensure that the size of this enum is 4 bytes */
	ERROR_UNUSED                            = 0x10000000
} cam_cmd_status;

typedef struct cmd_tid_log
{
	uint16_t tid;
	cam_cmd_status cmd_status;
} cmd_tid_log_t;

typedef struct cam_cmd_log
{
	cmd_tid_log_t cmd_log[MAX_CAM_TID_LOG_NUM];
	uint8_t cur_idx;
} cam_cmd_log_t;

#define GROUP_A  0x0000003E
#define GROUP_B  0x000007C0
#define GROUP_C  0x0001F800
#define GROUP_AB (GROUP_A | GROUP_B)
#define GROUP_BC 0x0000FFC0

/*
    Lookup table for the CPLD module select registers

    8'h02 : mem_rd_data <= {3'b000, CamLensGrpB[4:0]}; {B1, B2, B3, B5, B4}
    8'h03 : mem_rd_data <= {2'b00,  CamLensGrpC[5:0]}; {C5, C6, C3, C4, C2, C1}
 */

/**
 * \brief  Camera sensor address type definition
 */
typedef UInt8 CamSlaveAddress_TypeDef;

/**
 * \brief  I2C channel type definition
 */
typedef I2CEx_Ch_t CamI2CChannel_TypeDef;

/**
 * \brief  Camera status type enumeration
 */
typedef enum {
	CAM_STATUS_CLOSE = 0,
	CAM_STATUS_OPEN  = 1,
} CamStatus_TypeDef;

/**
 * \brief  AF module command type enumeration
 */
typedef enum {
	AF_ON    = 0,
	AF_START = 1,
	AF_STOP  = 2
}CamAF_State;

/**
 * \brief  Camera stream status type enumeration
 */
typedef enum {
	CAM_STREAM_OFF = 0,
	CAM_STREAM_ON  = 1,
} CamStream_TypeDef;

/**
 * \brief  Camera status type enumeration
 */
typedef enum   {
	CAM_CLOSE = 0,
	CAM_READY,
	CAM_STREAM
}cam_status;

// Note that the following are used as both enumerated types and
// for bitwise operations - so be careful when changing their
// values.
typedef enum
{
	CAM_ORIENTATION_NORMAL  = 0x0,
	CAM_ORIENTATION_FLIP_H  = 0x1,
	CAM_ORIENTATION_FLIP_V  = 0x2,
	CAM_ORIENTATION_FLIP_HV = 0x3,
} CamOrientation;

/**
 * \brief  Camera resolution structure
 */
typedef struct   {
	UInt8 X1;
	UInt8 X2;
	UInt8 Y1;
	UInt8 Y2;
} cam_res;

typedef enum
{
    HALL_POLARITY_NORMAL   = 0,
    HALL_POLARITY_REVERSED = 1,
} hall_sensor_polarity;

typedef enum
{
	CAM_VC_0 = 0x1,
	CAM_VC_1 = 0x2,
	CAM_VC_2 = 0x4,
	CAM_VC_3 = 0x8,
} cam_vc_t;
/**
 * \brief  I2C device structure
 */
// Do we care enough about space to repack this structure?
typedef struct {
	const char *Name;
	CamID	camera_id;
	CamType_TypeDef Type;
	CamOrientation  Orientation; // TODO: Query from EEPROM?
	CamI2CChannel_TypeDef I2CChannel;
	CamSlaveAddress_TypeDef Address;
	CamStatus_TypeDef CamStatus;
	CamStream_TypeDef StreamStatus;
	uint16_t CPLD_select;
    uint16_t retracted_hard_stop;
    uint16_t extended_hard_stop;
    uint16_t wide_hard_stop;
    uint16_t narrow_hard_stop;
    hall_sensor_polarity lens_hall_polarity;
    hall_sensor_polarity mirror_hall_polarity;

#ifdef BOARD_VERSION_P1_1
    uint16_t line_length_pclk;
    uint16_t frame_length_lines;
#endif
}CamDevice_TypeDef;

extern CamDevice_TypeDef *CamDeviceTbl[CAM_NUM_CAMERA_MAX];

/* Exported macro -------------------------------------------------------------*/
#ifdef BOARD_VERSION_P1
#define CAM_INIT(name, type, orientation, i2c_ch, addr, id, cpld_sel, retracted_stop, extended_stop, lens_polarity, wide_stop, narrow_stop, mirror_polarity) \
    CamDevice_TypeDef name = { \
    .Name = #name, \
    .camera_id = id, \
    .Type = type, \
    .Orientation = orientation, \
    .I2CChannel = i2c_ch, \
    .Address = addr, \
    .CamStatus = CAM_STATUS_CLOSE, \
    .StreamStatus = CAM_STREAM_OFF, \
    .CPLD_select = cpld_sel,\
    .retracted_hard_stop = retracted_stop,\
    .extended_hard_stop = extended_stop,\
    .lens_hall_polarity = lens_polarity,\
    .wide_hard_stop = wide_stop,\
    .narrow_hard_stop = narrow_stop,\
    .mirror_hall_polarity = mirror_polarity\
}
#endif

#ifdef BOARD_VERSION_P1_1
#define CAM_INIT(name, type, orientation, i2c_ch, addr, id, cpld_sel, retracted_stop, extended_stop, lens_polarity, wide_stop, narrow_stop, mirror_polarity, cam_line_length_pclk, cam_frame_length_lines) \
    CamDevice_TypeDef name = { \
    .Name = #name, \
    .camera_id = id, \
    .Type = type, \
    .Orientation = orientation, \
    .I2CChannel = i2c_ch, \
    .Address = addr, \
    .CamStatus = CAM_STATUS_CLOSE, \
    .StreamStatus = CAM_STREAM_OFF, \
    .CPLD_select = cpld_sel,\
    .retracted_hard_stop = retracted_stop,\
    .extended_hard_stop = extended_stop,\
    .lens_hall_polarity = lens_polarity,\
    .wide_hard_stop = wide_stop,\
    .narrow_hard_stop = narrow_stop,\
    .mirror_hall_polarity = mirror_polarity,\
	.line_length_pclk = cam_line_length_pclk,\
	.frame_length_lines = cam_frame_length_lines\
}
#endif
typedef enum
{
	FALSE,
	TRUE
}bool;

// Note that these values are assigned to match the encodings in
// the SPI communication protocol with the FPGA.
typedef enum
{
	ADDR8_DATA8   = 0,
	ADDR8_DATA16  = 2,
	ADDR16_DATA8  = 1,
	ADDR16_DATA16 = 3,
} I2CMESSAGETYPE;

/**
 * \brief  CAM Status structure
 */
typedef enum
{
	MODULE_POWER_ON					= 1 << 0,
	MODULE_CLOCK_ON					= 1 << 1,
	MODULE_HW_STANDBY				= 1 << 2,
	MODULE_SW_STANDBY				= 1 << 3,
	MODULE_OPEN						= 1 << 4,
	MODULE_STREAM_ON				= 1 << 5,
	ERROR_SENSOR_I2C_DETECT_FAILURE	= 1 << 6,
	ERROR_SENSOR_I2C_READ_FAILURE	= 1 << 7,
	ERROR_SENSOR_I2C_WRITE_FAILURE	= 1 << 8,
	MODULE_HALL_I2C_DETECT_FAILURE	= 1 << 9,
	ERROR_EEPROM_I2C_READ_FAILURE	= 1 << 10,
	ERROR_EEPROM_I2C_WRITE_FAILURE	= 1 << 11,
	ERROR_MIRRORS_DETECT			= 1 << 12,
	ERROR_LENS_DETECT				= 1 << 13,
	ERROR_VCM_I2C_READ_FAILURE		= 1 << 14,
	ERROR_VCM_I2C_WRITE_FAILURE		= 1 << 15,
	CSI_CAM_CHANNEL_0				= 1 << 16,
	CSI_CAM_CHANNEL_1				= 1 << 17,
}status_t;
/**
 * \brief  CAM focus Status structure
 */
typedef enum cam_focus_status
{
	IDLE			= 0x00,
	MOVING			= 0x01,
	MOVING_ERROR	= 0x02
}cam_focus_status_t;


/*
    registers to write:

    0x06        : debug_test, always 1
    0x0a - 0x15 : duty cycles, 2 bytes each, 12 bytes total
    0x20 - 0x37 : repetition counts, 4 bytes each, 24 bytes total
    0x50 - 0x57 : group repetition counts, 4 bytes each, 8 bytes total
    0x70 - 0x73 : overall repetition count, 4 bytes total
    0x80 - 0x81 : frequency, 2 bytes total

    50 bytes total, not including 0x06
    */

// Leave these byte arrays as struct, in case we want to
// extend their functionality in the future
#ifdef BOARD_VERSION_P1
typedef struct
{
        uint8_t byte[2];
} cpld_duty_t;

typedef struct
{
        uint8_t byte[4];
} cpld_rep_t;
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1
typedef struct
{
	union
	{
		uint16_t val;
		uint8_t byte[2];
	};
} cpld_duty_t;

typedef struct
{
	union
	{
		uint32_t val;
		uint8_t byte[4];
	};
} cpld_rep_t;
#endif /* BOARD_VERSION_P1_1 */
typedef struct
{
    cpld_duty_t duty_cycle1;        // 2
    cpld_duty_t duty_cycle2;        // 2
    cpld_duty_t duty_cycle3;        // 2
    cpld_duty_t duty_cycle4;        // 2
    cpld_duty_t duty_cycle5;        // 2 10
    cpld_duty_t duty_cycle6;        // 2
    cpld_rep_t rep_cnt1;            // 4
    cpld_rep_t rep_cnt2;            // 4 20
    cpld_rep_t rep_cnt3;            // 4
    cpld_rep_t rep_cnt4;            // 4
    cpld_rep_t rep_cnt5;            // 4 32
    cpld_rep_t rep_cnt6;            // 4
    cpld_rep_t rep_group_cnt1;      // 4 40
    cpld_rep_t rep_group_cnt2;      // 4
    cpld_rep_t rep_ovr_cnt1;        // 4
    cpld_rep_t freq;                // 2 50
} CPLD_CONFIG;
typedef struct
{
	uint32_t current_cam_status;
	uint32_t hw_standby_bitmask;
	uint32_t sw_standby_bitmask;
	uint32_t close_bitmask;
	uint16_t hw_standby_slave_order;
	uint16_t sw_standby_slave_order;
	uint16_t close_slave_order;
} cam_open_data_t;
/* Exported variables ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/

/* TODO: Those I2C_xxx functions should not be implemented in application layer */
Int8 I2C_8BitAddr_WriteByte(I2CEx_Msg_t *msg, UInt8 addr, UInt8 data);
Int8 I2C_8BitAddr_WriteWord(I2CEx_Msg_t *msg, UInt8 addr, UInt16 data);
UInt8 I2C_8BitAddr_ReadByte(I2CEx_Msg_t *msg, UInt8 addr);
UInt16 I2C_8BitAddr_ReadWord(I2CEx_Msg_t *msg, UInt8 addr);
Int8 I2C_16BitAddr_WriteWord(I2CEx_Msg_t *msg, UInt16 addr, UInt16 data);
Int8 I2C_16BitAddr_WriteByte(I2CEx_Msg_t *msg, UInt16 addr, UInt16 data);
UInt8 I2C_16BitAddr_ReadByte(I2CEx_Msg_t *msg, UInt16 addr);
Bool fpga_write_i2c_value(UInt8 chanid, UInt8 slave_addr, UInt16 reg_addr, UInt16 reg_data, I2CMESSAGETYPE mode, Bool interupt);
UInt16 fpga_read_i2c_value(UInt8 chanid, UInt8 slave_addr, UInt16 reg_addr, I2CMESSAGETYPE mode);
/**
 * \brief Function to reset all status
 */
void StatusResetAll(void);
/**
 *  \brief Function to enable status of camera
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param idx : index to choose camera;
 *	\param mask : status value
 *  \par Description:
 *  This function is used to enable status mask for camera
 *
 */
void StatusEnable(UInt8 idx,status_t mask);
/**
 *  \brief Function to disable status of camera
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param idx: index to choose camera;
 *	\param mask : status value
 *  \par Description:
 *  This function is used to disable status mask for camera
 *
 */
void StatusDisable(UInt8 idx,status_t mask);
/**
 *  \brief Function to return status mask of camera
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param idx: index to choose camera;
 *  \par return:
 *  Function will return status mask of camera if cam_bitmask's exist.
 *  Function will return 0xffff if cam_bitmask isn't exist;
 */
UInt32 StatusMask(UInt8 idx);
/**
 *  \brief Function to check status do enable.
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param idx: index to choose camera;
 *	\param mask : status value
 *  \par return:
 *  This function will return true if mask is enable, otherwise it'll return false;
 *
 */
Bool StatusActivated(UInt8 cam_BitMask, status_t mask);
/**
 *  \brief Function to save command status corresponding to specified tid
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param tid : corresponding transaction id
 *	\param cmd_status : status of the command execution for the transaction id
 *  \par Description:
 *  This function is used to save the command status of a corresponding
 *  transaction id into the global array. User then can know what is the last
 *  status of a command based on transaction id.
 *
 */
Bool save_command_log(uint16_t tid,cam_cmd_status cmd_status);
/**
 * \brief Function to reset all focus status
 */
void cam_focus_status_reset_all(void);
/**
 *  \brief Function to check focus status do enable.
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param index: index to choose camera;
 *	\param mask : status value
 *  \par return:
 *  This function will return true if mask is enable, otherwise it'll return false;
 *
 */
Bool check_cam_focus_status_actived(UInt8 index, cam_focus_status_t mask);
/**
 *  \brief Function to set focus status of camera
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param index : index to choose camera;
 *	\param mask : status value
 *  \par Description:
 *  This function is used to set status mask for camera
 *
 */
void set_cam_focus_status(UInt8 index, cam_focus_status_t mask);
/**
 *  \brief Function to reset focus status of camera
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param index: index to choose camera;
 *	\param mask : status value
 *  \par Description:
 *  This function is used to reset status mask for camera
 *
 */
void reset_cam_focus_status(UInt8 index, cam_focus_status_t mask);
/**
 *  \brief Function to return forcus status mask of camera
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param index: index to choose camera;
 *  \par return:
 *  Function will return focus status of camera if cam_bitmask's exist.
 *  Function will return 0xFFFFFFFF if cam_bitmask isn't exist;
 */
UInt32 read_cam_focus_status(UInt8 index);
/**
 *  \brief Function to handle camera control commands sent by host
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param pvParameters (in) TBD
 *
 *  \par Description:
 *  This function is used to initialize the camera and then handle camera
 *  control commands sent by host
 */
void vCamController(void *pvParameters);

/**
 *  \brief Function to set resolution for a camera device
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param cam (in,out) Pointer to camera device structure
 *  \param Resx (in) X-Resolution
 *  \param Resy (in) Y-Resolution
 *  \retval 1 The function executed successfully.
 *
 *  \par Description:
 *  This function selects a CSI channel and sets resolution for a camera
 */
UInt8 Set_Resolution(CamDevice_TypeDef *cam, UInt32 Resx, UInt32 Resy);

/**
 *  \brief Function to set exposure for a camera device
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param cam (in,out) Pointer to camera device structure
 *  \param data (in) Exposure value
 *
 *  \par Description:
 *  This function selects a CSI channel and sets exposure for a camera
 */
UInt8 Set_Exposure(CamDevice_TypeDef *cam, UInt16 data);

/**
 *  \brief Function to set sensitivity for a camera device
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param cam (in,out) Pointer to camera device structure
 *  \param data (in) Sensitivity value
 *
 *  \par Description:
 *  This function selects a CSI channel and sets sensitivity for a camera
 */
UInt8 Set_Sensitivity(CamDevice_TypeDef *cam, UInt16 data);

/**
 *  TODO: This function has not finished implementing
 *  \brief Function to set frame rate (frames per second) for a camera device
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param cam (in,out) Pointer to camera device structure
 *  \param fps (in) FPS value to set for camera
 *
 *  \par Description:
 *  This function selects a CSI channel and sets sensitivity for a camera
 */
UInt8 Set_FPS(CamDevice_TypeDef *cam, UInt16 fps);

/*
 * TODO: Add function description (Function has not implemented yet)
 */
UInt8 vAR0835_EEPROM(CamDevice_TypeDef *cam);

/**
 *  \brief Function to do factory test mode for all lens, mirrors modules
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *
 *  \par Description:
 *  This function will run test for all cameras and the return status will
 *  be read in the CAM_MODULE_STATUS command
 */
void FTM_cam_modules(void);

/**
 *  \brief Function to read four Temperature sensors
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param none
 *
 *  \par Description:
 *  This function dump temperature value
 */
void ReadTemperature(void);

/**
 *  \brief Function to send PreviewSimple cmd to FPGA
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param none
 *
 *  \par Description:
 *  This function will send PREVIEW_SIMPLE cmd to FGPA with
 *  specific Channel
 */
UInt8 PreviewSimple(CamID ChannelID, UInt8);
UInt8 Preview(UInt8 Intr, UInt8 count, UInt8 *CamOrder, UInt8 *ControlStatusReg, UInt8 *VirtualChannelIdentifier, UInt8 *DataType);
/**
 *  \brief Function to send SNAPSHOT cmd to FPGA
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param I2C_Channel_Mask (in): I2C channel mask of required camera
 *  \param Intr (in): Interrupt enable or not
 *
 *  \par Description:
 *  This function will send SNAPSHOT cmd to FGPA with
 *  specific I2C channel
 */
UInt8 Snapshot(UInt32 I2C_Channel_Mask, UInt8 Intr);

/*trigger: 1-> Send trigger
           0-> Stop Trigger
*/
UInt8 Trigger(UInt32 I2C_Channel_Mask, UInt8 trigger);
/**
 *  \brief Function to return mirror status mask of camera
 *
 *  \par Header File:
 *  cam_ctl.h
 *
 *  \par Parameters:
 *  \param none
 *  \par return:
 *  Function will return mirror status of camera.
 */
UInt32 read_cam_mirror_status(void);


//TODO: Have arguments for pulse width, pulse interval & pulse count.
//TODO :Add comments for functions
UInt8 Config_Trigger_for_Preview(CamID  camera_id);
UInt8 Preview_Off(uint32_t mask);
CamID find_max_exposure(UInt32 I2C_Channel_Mask, UInt64 *exposure);
UInt8 ConfigureFrameToCapture(UInt8 frame_index);
#ifdef BOARD_VERSION_P1
UInt8 Config_Trigger(UInt16 fll, UInt32 num_pulses);
UInt32 calculate_pulse_interval(UInt16 fll_value);
double calculate_fll_msec(UInt16 fll_value);
UInt8 Configure_FLL( UInt16 *fll, UInt32 i2c_channel_mask, CamID cam_id );
UInt8 Configure_LLPCLK(UInt32 i2c_channel_mask);
UInt8 Configure_CIT(UInt32 i2c_channel_mask);
#endif
#ifdef BOARD_VERSION_P1_1
UInt8 Config_Trigger(CamDevice_TypeDef *cam, UInt16 fll, UInt32 num_pulses);
UInt32 calculate_pulse_interval(CamDevice_TypeDef *cam, UInt16 fll_value);
double calculate_fll_msec(CamDevice_TypeDef *cam, UInt16 fll_value);
UInt8 Configure_FLL( UInt32 cam_group, UInt16 *fll, UInt32 i2c_channel_mask, CamID cam_id );
UInt8 Configure_LLPCLK(UInt32 cam_group, UInt32 i2c_channel_mask);
UInt8 Configure_CIT(UInt32 cam_group, UInt32 i2c_channel_mask);
#endif
UInt8 Configure_offsets_trigger(UInt32 i2c_channel_mask);
UInt8 Configure_offsets_trigger_p1_1(UInt32 i2c_channel_mask);



/*
 *  \brief Function to send bytes to the CPLD
 *
 *  \par Header File:
 *  cam_ctrl.h
 *
 *  \par Parameters:
 *  \param uint8_t* tx_buffer (in): Buffer that holds data to be sent to the
 *                                  CPLD
 *  \param uint8_t* rx_buffer (in): Buffer that holds data received from the
 *                                  CPLD
 *  \param uint16_t num_bytes (in): Size of the preceding buffers. They must
 *                                  match
 *
 *  \par Description:
 *  This function sends num_bytes bytes to the CPLD, either for a
 *  read or a write. The data has the structure
 *
 *      [0x02] [addr] [data] [...]
 *
 *      for a write, and
 *
 *      [0x0B] [addr] [OxFF] [...]
 *
 *      for a read.
 *
 *      [...] indicates that the number of bytes sent and received is
 *      variable, but must be at least one. num_bytes should be
 *      modified accordingly.
 *
 */
void CPLD_SPI_Transfer(uint8_t* tx_buffer, uint8_t* rx_buffer,
                       uint16_t num_bytes);
/*
 *  \brief Returns the i2c channel bit for a given module
 *
 *  \par Header File:
 *  cam_ctrl.h
 *
 *  \par Parameters:
 *  \param int32_t i (in) : the current value of the loop iterator
 *                          that loops through the cameras. Valid
 *                          inputs: 0 to CAM_NUM_CAMERA_MAX - 1
 *
 *  \par Description:
 *  This function takes the current module index (not bitmask) and
 *  returns the one-hot bitmask set for the two byte i2c channel bitmask.
 *
 */
uint16_t get_i2c_channel_bit(int32_t i);

/*
 *  \brief Writes  into the cam_m_status field with a given size/endianness
 *
 *  \par Header File:
 *  cam_ctrl.h
 *
 *  \par Parameters:
 *  \param void*   status_buffer (in) : the field in cam_m_status to write into
 *  \param uint8_t num_bytes     (in) : the size in bytes of status_buffer
 *  \param uint8_t endianness    (in) : the endianness of the data to be written
 *                                      0 is little endian
 *                                      1 is big endian
 *  \param void*   data          (in) : num_bytes of data to be written to
 *                                      status_buffer
 *
 *  \par Description:
 *  This function takes the current module index (not bitmask) and
 *  returns the one-hot bitmask set for the two byte i2c channel bitmask.
 *
 */
void save_cam_m_status_field(volatile uint8_t* status_buffer, uint8_t num_bytes,
                             uint8_t endianness, uint8_t* data);


void cam_module_open_iterate(uint8_t i, uint8_t cam_ctrl,
		CamDevice_TypeDef* pCam, cam_open_data_t* cam_open_data, uint16_t tid);
void cam_module_open_apply(uint8_t isGlobal, cam_open_data_t* cam_open_data);
void write_xshutdown_signal(uint8_t cam_ctrl, uint32_t slave_order_mask);
void configure_CPLD(uint8_t control_type, uint8_t direction, CPLD_CONFIG* cpld_config, uint32_t multiplier);
CPLD_CONFIG* set_config_buffer(uint8_t control_type, uint8_t direction);

void cam_sensor_temp_monitor(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* __CCB_CAM_CTRL_H */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE***********/
