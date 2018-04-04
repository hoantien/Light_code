/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *
 * @file    light_header.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    July-25-2016
 * @brief
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "light_header.h"
#include "img_sensor.h"
#include "light_system.h"
#include "task_cam_ctrl.h"
#include "temperature.h"
#include "timestamp.h"
#include "stdint.h"
#include "i2cm.h"
#include "img_sensor.h"
#include "sections.h"
#include "rtc.h"
#include "ar1335.h"
#include "gen_heap.h"
#include "ddr_heap.h"

/* Private typedef -----------------------------------------------------------*/

typedef enum color_t
{
	RED = 0,
	GREEN,
	BLUE,
	PANCHROMATIC
} color;

/* Private define ------------------------------------------------------------*/
#define SELECT_ALL_CAM					(uint32_t)0x1ffff
#define	GLOBAL_BIT_MASK					0x01
#define CAM_MODULE_OPEN					(1 << 4)

#define BYTE_MASK						0xff
#define RAW_MASK						0x3ff

#define VERTICAL_FLIP_MASK				BIT15
#define HORIZOTAL_FLIP_MASK				BIT14

#define CAM_A1							0xA1
#define CAM_A2							0xA2
#define CAM_A3							0xA3
#define CAM_A4							0xA4
#define CAM_A5							0xA5
#define CAM_B1							0xB1
#define CAM_B2							0xB2
#define CAM_B3							0xB3
#define CAM_B4							0xB4
#define CAM_B5							0xB5
#define CAM_C1							0xC1
#define CAM_C2							0xC2
#define CAM_C3							0xC3
#define CAM_C4							0xC4
#define CAM_C5							0xC5
#define CAM_C6							0xC6

#define TIME_STAMP_BUFF_SIZE			10

#define	FORMAT_RAW8						0x2A
#define	FORMAT_RAW10					0x2B
#define	FORMAT_RAW12					0x2C
#define	FORMAT_RAW14					0x2D
#define	FORMAT_LIGHT_RAW				0x30

#define TEMP_SENSOR_RESET				0x0000
#define TEMP_SENSOR_RESET_DATA			0x0021
#define TEMP_SENSOR_START_READ			0x0011
#define TEMP_SENSOR_CTRL_REG			0x3126
#define TEMP_SENSOR_DATA_REG			0x3124
#define TEMP_SENSOR_CAL1_REG			0x3128
#define TEMP_SENSOR_CAL2_REG			0x312A
#define CAMERA_IMAGE_SENSOR_ADDRESS		(0x6C >> 1)  /* Aptina */
#define TEMP_RAW_MASK					0x3ff

#define BLACK_LEVEL						(42)
#define WHITE_LEVEL						(1023)
#define CLIFF_SLOPE						(2)

#define MAX_ANALOG_GAIN						(float)(7.75)

static heap_attribute_t* const light_header_heap = &ddr_heap_attribute;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
LIGHT_HEADER_FUNC uint8_t get_cam_total_number(uint32_t m_bitmask);

LIGHT_HEADER_FUNC Ltpb__CameraID get_cam_id(cam_typedef_t *pcam);
LIGHT_HEADER_FUNC Ltpb__CameraModule *get_cam_module(cam_typedef_t *pcam);

/*
 * static uint8_t check_mirror_present(cam_typedef_t *pcam);
 * static Ltpb__Point2I *get_bayer_red_coord(cam_typedef_t *pcam);
 * static Ltpb__SensorType get_sensor_type(cam_typedef_t *pcam);
 */

LIGHT_HEADER_FUNC Ltpb__CameraModule__AFInfo *get_auto_focus_info(cam_typedef_t *pcam);

LIGHT_HEADER_FUNC int32_t get_img_focal_length(cam_typedef_t *ref_cam);
LIGHT_HEADER_FUNC char *get_device_asic_fw_version(void);
LIGHT_HEADER_FUNC char *get_device_fw_version(void);
LIGHT_HEADER_FUNC char *get_device_model_name(void);
LIGHT_HEADER_FUNC uint8_t  *get_device_unique_id(cam_typedef_t *pcam);

extern cam_typedef_t *get_cam_thumbnail(uint32_t m_bitmask);
extern uint8_t cam_mem_idx[6];
extern uint32_t img_bytes;

/* Private functions ---------------------------------------------------------*/
LIGHT_HEADER_FUNC Ltpb__Point2I *get_default_point2i(int x, int y)
{
	Ltpb__Point2I *p = gen_pvPortMalloc(light_header_heap, sizeof(Ltpb__Point2I));
	ltpb__point2_i__init(p);
	p->x = x;
	p->y = y;
	return p;
}
LIGHT_HEADER_FUNC Ltpb__Point2F *get_default_point2f(float x, float y)
{
	Ltpb__Point2F *p = gen_pvPortMalloc(light_header_heap, sizeof(Ltpb__Point2F));
	ltpb__point2_f__init(p);
	p->x = x;
	p->y = y;
	return p;

}
LIGHT_HEADER_FUNC short cam_read_temperature(cam_typedef_t *pcam)
{
	uint16_t tmp_raw = 0;
	uint16_t calib = 0;
	short temp = 0.0;

	img_sensor_t *img_sensor;
	img_sensor = pcam->image;

	/* Reset temp data */
	if (I2CM_ERROR_TRANSMITTED
			== img_sensor_write_reg(img_sensor, TEMP_SENSOR_CTRL_REG,
			                        TEMP_SENSOR_RESET, TWO_BYTES))
	{
		/* Turn on temp sensor */
		if (I2CM_ERROR_TRANSMITTED
				== img_sensor_write_reg(img_sensor, TEMP_SENSOR_CTRL_REG,
										TEMP_SENSOR_RESET_DATA, TWO_BYTES))
		{
			/* Start the temp sensor */
			if(I2CM_ERROR_TRANSMITTED
					== img_sensor_write_reg(img_sensor, TEMP_SENSOR_CTRL_REG,
											TEMP_SENSOR_START_READ, TWO_BYTES))
			{
				vTaskDelay(2);

				/* Read temp raw data */
				tmp_raw = img_sensor_read_reg(img_sensor, TEMP_SENSOR_DATA_REG,
						TWO_BYTES);

				if (tmp_raw)
				{
					tmp_raw &= 0x3FF;
					/* Clear temp data */
					if (I2CM_ERROR_TRANSMITTED
							== img_sensor_write_reg(img_sensor,
											TEMP_SENSOR_CTRL_REG,
											TEMP_SENSOR_RESET_DATA, TWO_BYTES))
					{
						/* Read the calib data */
						calib = img_sensor_read_reg(img_sensor,
						TEMP_SENSOR_CAL1_REG, TWO_BYTES);
						if (calib)
						{
							temp = (short) ((float) ((tmp_raw - calib + 90)
									/ 1.51));
						}
					}
				}
			}
		}
	}

	return temp;
}

LIGHT_HEADER_FUNC float get_sensitivity(cam_typedef_t *pcam)
{
	if(!pcam)
		return FALSE;
	uint8_t ch;
	uint16_t total_gain_code;
	float total_gain;
	ch = pcam->info.ch - 1;
	if(I2CM_ERROR_TRANSCEIVED == i2cm.read(ch, WORD_ADDR16,
	IMAGE_SENSOR_ADDRESS, CAM_REG_SENSITIVITY, (uint8_t *)&total_gain_code))
	{
		total_gain_code = __builtin_bswap16(total_gain_code);
		total_gain = get_total_gain(total_gain_code, UCID_HIRES_CAPTURE);
		return total_gain;
	}
	return FALSE;
}

LIGHT_HEADER_FUNC uint64_t get_exposure_ustime(cam_typedef_t *pcam)
{
	uint64_t exposure = 0;
	uint32_t llpclk = img_sensor_read_reg(pcam->image, LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
	uint32_t vtpixclk = img_sensor_get_vt_pix_clk_mhz(pcam->image);
	uint32_t cit = img_sensor_read_reg(pcam->image, CIT_CAM_REG, TWO_BYTES);
	double it_time = (double)(cit * llpclk * 1.0)	/ (double)(2 * vtpixclk * MHZ_TO_HZ * 1.0);

	exposure = (uint64_t)(it_time * 1000 * 1000 * 1000); /* convert to nano second */

	return exposure;
}

LIGHT_HEADER_FUNC protobuf_c_boolean get_cam_is_enabled(cam_typedef_t *pcam)
{
	uint32_t status = 0;
	uint8_t *pstatus;

	pstatus = pcam->settings->status;
	status = S_MODULE_SW_STANDBY & (*(uint32_t *)pstatus) ;

	if(status)
	{
		/* return true */
		return 1;
	}
	/* return false: */
	return 0;
}

LIGHT_HEADER_FUNC Ltpb__CameraModule__Surface *get_sensor_data_surface(cam_typedef_t *pcam)
{
	Ltpb__CameraModule__Surface *sensor_surface;

	sensor_surface = (Ltpb__CameraModule__Surface *)gen_pvPortMalloc(light_header_heap,
								sizeof(Ltpb__CameraModule__Surface));

	ltpb__camera_module__surface__init(sensor_surface);

	uint16_t width;

	width = pcam->image->x_size; //img_sensor_read_reg(pcam->image, X_OUTPUT_SIZE_CAM_REG, TWO_BYTES);

	uint16_t byte_per_raw = (width * BITS_PER_PIXEL_RAW10) / 8;

	if(byte_per_raw % 16 != 0)
	{
		byte_per_raw = ((byte_per_raw / 16) + 1) * 16;
	}

	/* TODO: */
	sensor_surface->data_offset = (uint32_t)(sizeof(lightheader_info_t)
								+ cam_mem_idx[pcam->info.ch - 1] * img_bytes);
	sensor_surface->row_stride = byte_per_raw;
	sensor_surface->format = LTPB__CAMERA_MODULE__SURFACE__FORMAT_TYPE__RAW_PACKED_10BPP;

	sensor_surface->data_scale = get_default_point2f(1, 1);
	sensor_surface->size = get_default_point2i(pcam->image->x_size, pcam->image->y_size);
	sensor_surface->start = get_default_point2i(0, 0);

	return sensor_surface;
}

LIGHT_HEADER_FUNC uint8_t get_cam_total_number(uint32_t m_bitmask)
{
	uint8_t cam_idx = 0;
	uint8_t total_cam = 0;

	m_bitmask &= light_system->m_filter;
	while(m_bitmask)
	{
		if(m_bitmask & 1)
		{
			if(idx_to_object(cam_idx) != NULL)
			{
				total_cam++;
			}
		}
		m_bitmask >>= 1;
		cam_idx++;
	}
	return total_cam;
}

LIGHT_HEADER_FUNC Ltpb__CameraID get_cam_id(cam_typedef_t *pcam)
{
	uint8_t cam_id = 0;

	if(pcam == NULL)
		return 0;
	switch(pcam->info.module)
	{
		case CAM_A1:
			cam_id = LTPB__CAMERA_ID__A1;
			break;
		case CAM_A2:
			cam_id = LTPB__CAMERA_ID__A2;
			break;
		case CAM_A3:
			cam_id = LTPB__CAMERA_ID__A3;
			break;
		case CAM_A4:
			cam_id = LTPB__CAMERA_ID__A4;
			break;
		case CAM_A5:
			cam_id = LTPB__CAMERA_ID__A5;
			break;
		case CAM_B1:
			cam_id = LTPB__CAMERA_ID__B1;
			break;
		case CAM_B2:
			cam_id = LTPB__CAMERA_ID__B2;
			break;
		case CAM_B3:
			cam_id = LTPB__CAMERA_ID__B3;
			break;
		case CAM_B4:
			cam_id = LTPB__CAMERA_ID__B4;
			break;
		case CAM_B5:
			cam_id = LTPB__CAMERA_ID__B5;
			break;
		case CAM_C1:
			cam_id = LTPB__CAMERA_ID__C1;
			break;
		case CAM_C2:
			cam_id = LTPB__CAMERA_ID__C2;
			break;
		case CAM_C3:
			cam_id = LTPB__CAMERA_ID__C3;
			break;
		case CAM_C4:
			cam_id = LTPB__CAMERA_ID__C4;
			break;
		case CAM_C5:
			cam_id = LTPB__CAMERA_ID__C5;
			break;
		case CAM_C6:
			cam_id = LTPB__CAMERA_ID__C6;
			break;
		default:
			cam_id = LTPB__CAMERA_ID__A1;
			break;
	}
	return cam_id;
}

LIGHT_HEADER_FUNC Ltpb__CameraModule__AFInfo *get_auto_focus_info(cam_typedef_t *pcam)
{
	Ltpb__CameraModule__AFInfo *af_info = gen_pvPortMalloc(light_header_heap, sizeof(Ltpb__CameraModule__AFInfo));
	/* Initialize */
	ltpb__camera_module__afinfo__init(af_info);
	af_info->has_roi_estimated_disparity = 1;
	af_info->mode = LTPB__CAMERA_MODULE__AFINFO__AFMODE__AUTO;
	af_info->roi_center = get_default_point2f(pcam->cam_common_data.roi_center_x, pcam->cam_common_data.roi_center_y);
	af_info->roi_estimated_disparity = light_system->settings->roi_transfer.distance_mm;

	return af_info;
}

LIGHT_HEADER_FUNC int32_t get_img_focal_length(cam_typedef_t *ref_cam)
{
	if(NULL == ref_cam)
		return 0;

	/* Get image focal length */
	uint16_t *p_focal_length = (uint16_t *)ref_cam->optical->settings->focal_length;
	int32_t focal_length = light_system->settings->zoom_factor * (*p_focal_length);
	/* image_focal_length is per capture and for the whole image */
	return focal_length;
}

LIGHT_HEADER_FUNC uint8_t  *get_device_unique_id(cam_typedef_t *pcam)
{
	/* TODO: will implement in future */
	return pcam->settings->uuid;
}

LIGHT_HEADER_FUNC char *get_device_model_name(void)
{
	/* TODO: will implement in future */
	char *device_model_name = NULL;

	return device_model_name;
}

LIGHT_HEADER_FUNC char *get_device_asic_fw_version(void)
{
	/* TODO: will implement in future */
	char *device_asic_fw_version = NULL;

	return device_asic_fw_version;
}

LIGHT_HEADER_FUNC char *get_device_fw_version(void)
{
    size_t len = sizeof(light_system->settings->fw_version);
    char* device_fw_version = (char*)gen_pvPortMalloc(light_header_heap, len);
    memcpy(device_fw_version, light_system->settings->fw_version, len);
    return device_fw_version;
}

LIGHT_HEADER_FUNC Ltpb__TimeStamp *get_time_stamp(void)
{
	Ltpb__TimeStamp *time = (Ltpb__TimeStamp *)gen_pvPortMalloc(light_header_heap, sizeof(Ltpb__TimeStamp));
	ltpb__time_stamp__init(time);

	rtc_t rtc_time = rtc_get_time();
	unsigned long long total_seconds = rtc_time.milisec / 1000;

	/* TODO: get system date */
	time->hour		= ((total_seconds / 60) / 60) % 24;
	time->minute	= (total_seconds / 60) % 60;
	time->second	= total_seconds % 60;

	time->day = 1;
	time->month = 9;
	time->year = 2016;

	return time;
}

LIGHT_HEADER_FUNC uint32_t get_iso(float sensitivity)
{
	uint32_t iso = 0;
	uint8_t i;

	/* the sensitivity smaller than smallest value in lookup table */
	if(sensitivity < iso_table[0].total_gain)
	{
		iso = iso_table[0].iso;
	}
	/* the sensitivity bigger than biggest value in lookup table */
	else if(sensitivity >= iso_table[iso_table_size - 1].total_gain)
	{
		iso = iso_table[iso_table_size - 1].iso;
	}
	/* other case */
	else
	{
		for(i = 0; i < (iso_table_size - 1); i++)
		{
			if((sensitivity >= iso_table[i].total_gain) && (sensitivity < iso_table[i + 1].total_gain))
			{
				iso = iso_table[i].iso;
				break;
			}
		}
	}

	return iso;
}

LIGHT_HEADER_FUNC Ltpb__SensorCharacterization__VstNoiseModel__VstModel *get_vst_noise_vst_model(
		cam_typedef_t *pcam, color cam_color)
{
	Ltpb__SensorCharacterization__VstNoiseModel__VstModel *vst_model;
	vst_model =
			(Ltpb__SensorCharacterization__VstNoiseModel__VstModel *)gen_pvPortMalloc(light_header_heap,
					sizeof(Ltpb__SensorCharacterization__VstNoiseModel__VstModel));

	ltpb__sensor_characterization__vst_noise_model__vst_model__init(vst_model);

	/* TODO: implement */
	vst_model->a = 0;
	vst_model->b = 0;

	return vst_model;
}

LIGHT_HEADER_FUNC Ltpb__SensorCharacterization__VstNoiseModel *get_sensor_characterization_vst_noise_model(
															cam_typedef_t *pcam)
{
	Ltpb__SensorCharacterization__VstNoiseModel *vst_noise;
	vst_noise = (Ltpb__SensorCharacterization__VstNoiseModel *)gen_pvPortMalloc(light_header_heap,
			sizeof(Ltpb__SensorCharacterization__VstNoiseModel));

	ltpb__sensor_characterization__vst_noise_model__init(vst_noise);

	/* TODO: what is vst noise ? */
	/* gain in ISO unit at which the noise was characterized */
	vst_noise->gain = 0;
	/* noise threshold (the same for all color channels) */
	vst_noise->threshold = 0;
	/* normalizes VST transfrom output to 0..1 range */
	vst_noise->scale = 0;

	vst_noise->red = get_vst_noise_vst_model(pcam, RED);
	vst_noise->green = get_vst_noise_vst_model(pcam, GREEN);
	vst_noise->blue = get_vst_noise_vst_model(pcam, BLUE);
	vst_noise->panchromatic = get_vst_noise_vst_model(pcam, PANCHROMATIC);

	return vst_noise;
}

LIGHT_HEADER_FUNC Ltpb__SensorCharacterization *get_sensor_characterization_data(cam_typedef_t *pcam)
{
	Ltpb__SensorCharacterization *sensor_char =
			(Ltpb__SensorCharacterization *)gen_pvPortMalloc(light_header_heap,
					sizeof(Ltpb__SensorCharacterization));

	ltpb__sensor_characterization__init(sensor_char);

	sensor_char->black_level = BLACK_LEVEL;
	sensor_char->white_level = WHITE_LEVEL;

	sensor_char->has_cliff_slope = TRUE;
	sensor_char->cliff_slope = CLIFF_SLOPE;

	sensor_char->n_vst_model = 0;
	/*sensor_char->vst_model =
	 (Ltpb__SensorCharacterization__VstNoiseModel **)gen_pvPortMalloc(light_header_heap,
	 sensor_char->n_vst_model
	 * sizeof(Ltpb__SensorCharacterization__VstNoiseModel));*/

	return sensor_char;
}


LIGHT_HEADER_FUNC Ltpb__Point2I *get_sensor_bayer_red_override(cam_typedef_t *pcam)
{
	Ltpb__Point2I *sensor_bayer_red_override = (Ltpb__Point2I *)gen_pvPortMalloc(light_header_heap,
			sizeof(Ltpb__Point2I));
	ltpb__point2_i__init(sensor_bayer_red_override);

	uint8_t reg = img_sensor_read_pixel_order(pcam->image);

	/* To handle pan chromatic sensor */
	if (pcam->image->sensor_type == AR1335_PAN_SENSOR)
	{
		sensor_bayer_red_override->x = -1;
		sensor_bayer_red_override->y = -1;
		return sensor_bayer_red_override;
	}
	switch(reg)
	{
	case 0:
		sensor_bayer_red_override->x = 1;
		sensor_bayer_red_override->y = 0;
		break;
	case 1:
		sensor_bayer_red_override->x = 0;
		sensor_bayer_red_override->y = 0;
		break;
	case 2:

		sensor_bayer_red_override->x = 1;
		sensor_bayer_red_override->y = 1;
		break;
	case 3:
		sensor_bayer_red_override->x = 0;

		sensor_bayer_red_override->y = 1;
		break;
	default:
		sensor_bayer_red_override->x = 1;
		sensor_bayer_red_override->y = 0;
		break;
	}

	return sensor_bayer_red_override;

}

LIGHT_HEADER_FUNC Ltpb__CameraModule *get_cam_module(cam_typedef_t *pcam)
{
	uint8_t *p_tmp;
	Ltpb__CameraModule *pcam_module = (Ltpb__CameraModule *)gen_pvPortMalloc(light_header_heap,
										sizeof(Ltpb__CameraModule));
	if(!pcam_module)
		return NULL;
	/* Initialize */
	ltpb__camera_module__init(pcam_module);

	/* Module properties */
	pcam_module->id = get_cam_id(pcam);

	/* Auto-focus related properties */
	pcam_module->af_info = get_auto_focus_info(pcam);

	/* set to false if sensor is damaged */
	pcam_module->is_enabled = get_cam_is_enabled(pcam);

	short mirror_position = 0;
	short lens_position = 0;
	/* Mirror position */
	if(pcam->info.grp != GRP_A)
	{
		pcam_module->has_mirror_position = TRUE;
		p_tmp = pcam->optical->settings->mirr_position;
		memcpy(&mirror_position, p_tmp, 2 * sizeof(uint8_t));
		pcam_module->mirror_position = mirror_position;
	}
	else
		pcam_module->has_mirror_position = FALSE;

	/* Lens position */
	p_tmp = pcam->optical->settings->lens_position;
	memcpy(&lens_position, p_tmp, 2 * sizeof(uint8_t));
	pcam_module->lens_position = lens_position;

	/* in ISO units, e.g., 100 == ISO100, might include digital gain */
	float sensitivity = get_sensitivity(pcam);
	pcam_module->sensor_total_gain = get_iso(sensitivity);

	/* this corresponds to the actual analog sensor gain,
	 * which often will be different than total_gain */
	pcam_module->sensor_analog_gain = sensitivity;

	/* exposure in nano */
	uint64_t exposure = 0;
	p_tmp = pcam->image->settings[light_system->active_ucid]->exposure;
	memcpy(&exposure, p_tmp, 8 * sizeof(uint8_t));
	if(exposure == 0)
	{
		exposure = get_exposure_ustime(pcam);
	}
	pcam_module->sensor_exposure = exposure;

	/* sensor data surface descriptor */
	pcam_module->sensor_data_surface = get_sensor_data_surface(pcam);

	/* temparature in degrees C */
	pcam_module->has_sensor_temparature = TRUE;
	pcam_module->sensor_temparature = cam_read_temperature(pcam);

	/* We fix the flip at the sensor level */
	pcam_module->has_sensor_is_horizontal_flip = FALSE;
	/* pcam_module->sensor_is_horizontal_flip = pcam->image->flip & FLIP_HORIZONTAL; */

	/* We fix the flip at the sensor level */
	pcam_module->has_sensor_is_vertical_flip = FALSE;
	/* pcam_module->sensor_is_vertical_flip = pcam->image->flip & FLIP_VERTICAL; */

	/* custom coordinate of red pixel in 2x2 bayer pattern */
	pcam_module->sensor_bayer_red_override = get_sensor_bayer_red_override(pcam);

	return pcam_module;
}

LIGHT_HEADER_FUNC Ltpb__SensorData *get_sensor_data(cam_typedef_t *pcam)
{
	Ltpb__SensorData *sensor_data =
			(Ltpb__SensorData *)gen_pvPortMalloc(light_header_heap,
					sizeof(Ltpb__SensorData));

	ltpb__sensor_data__init(sensor_data);

	/* sensor type */
	if (pcam->image->sensor_type == AR1335_COLOR_SENSOR )
	{
		sensor_data->type = LTPB__SENSOR_TYPE__SENSOR_AR1335;
	}
	else
	{
		sensor_data->type = LTPB__SENSOR_TYPE__SENSOR_AR1335_MONO;
	}
	/* noise characterization, black/white levels, etc.. */
	sensor_data->data = get_sensor_characterization_data(pcam);

	/* Time stamp of when the sensor was characterized. */
	sensor_data->time_stamp = get_time_stamp();

	return sensor_data;
}

LIGHT_HEADER_FUNC Ltpb__FlashCalibration *get_flash_calib(void)
{
	Ltpb__FlashCalibration *flash_calib =
			(Ltpb__FlashCalibration *)gen_pvPortMalloc(light_header_heap,
					sizeof(Ltpb__FlashCalibration));

	ltpb__flash_calibration__init(flash_calib);

	/* TODO: have to complete all fields bellow */
	/* measured lux in the box of cool white led */
	flash_calib->ledcool_lux = 0;
	/* maximum lumens of cool white led */
	flash_calib->ledcool_max_lumens = 0;
	/* cct of cool white led in Kelvin */
	flash_calib->ledcool_cct = 0;

	/* measured lux in the box of warm white led */
	flash_calib->ledwarm_lux = 0;
	/* maximum lumens of warm white led */
	flash_calib->ledwarm_max_lumens = 0;
	/* cct of warm white led in Kelvin */
	flash_calib->ledwarm_cct = 0;

	return flash_calib;
}

LIGHT_HEADER_FUNC Ltpb__ToFCalibration *get_tof_calib(void)
{
	Ltpb__ToFCalibration *tof_calib = (Ltpb__ToFCalibration *)gen_pvPortMalloc(light_header_heap,
			sizeof(Ltpb__ToFCalibration));

	ltpb__to_fcalibration__init(tof_calib);

	/* TODO: have to complete all fields */

	/* distance in mm of the offset chart */
	tof_calib->offset_distance = 0;
	/* measured distance in mm of the offset chart */
	tof_calib->offset_measurement = 0;
	/* distance in mm of the cross talk chart */
	tof_calib->xtalk_distance = 0;
	/* measured distance in mm of the cross talk chart */
	tof_calib->xtalk_measurement = 0;

	return tof_calib;
}

LIGHT_HEADER_FUNC Ltpb__FactoryDeviceCalibration *get_factory_device_cablib(void)
{
	Ltpb__FactoryDeviceCalibration *device_calib =
			(Ltpb__FactoryDeviceCalibration *)gen_pvPortMalloc(light_header_heap,
					sizeof(Ltpb__FactoryDeviceCalibration));

	ltpb__factory_device_calibration__init(device_calib);

	/* Flash calibration data */
	device_calib->flash = get_flash_calib();
	/* Time-of-Flight sensor calibration */
	device_calib->tof = get_tof_calib();
	/* Time stamp of when the device was calibrated in the factory. */
	device_calib->time_stamp = get_time_stamp();

	return device_calib;
}

LIGHT_HEADER_FUNC uint8_t check_mirror_present(cam_typedef_t *pcam)
{
	/* check mirror is glued or not present */
	if((pcam->info.grp == GRP_A) || (pcam->info.module == CAM_B4) ||
		(pcam->info.module == CAM_C5) || (pcam->info.module == CAM_C6))
	{
		/* These CAMs don't have Mirror */
		return 0;
	}
	return 1;
}

LIGHT_HEADER_FUNC Ltpb__CameraModuleHwInfo *get_cam_hw_info(cam_typedef_t *pcam)
{
	Ltpb__CameraModuleHwInfo *cam_hw_info =
			(Ltpb__CameraModuleHwInfo *)gen_pvPortMalloc(light_header_heap,
					sizeof(Ltpb__CameraModuleHwInfo));

	ltpb__camera_module_hw_info__init(cam_hw_info);

	cam_hw_info->id = get_cam_id(pcam);
	cam_hw_info->sensor = LTPB__SENSOR_TYPE__SENSOR_AR1335;


	if(pcam->info.grp != GRP_A)
	{
		cam_hw_info->has_lens = TRUE;
		cam_hw_info->lens = LTPB__CAMERA_MODULE_HW_INFO__LENS_TYPE__LENS_SUNNY;
	}
	else
	{
		cam_hw_info->has_lens = TRUE;
		cam_hw_info->lens = LTPB__CAMERA_MODULE_HW_INFO__LENS_TYPE__LENS_UNKNOWN;
	}

	/* TODO: what is mirror actuator ?*/
	cam_hw_info->has_mirror_actuator = 0;
	cam_hw_info->mirror_actuator = 0;

	cam_hw_info->has_mirror = check_mirror_present(pcam);
	if(cam_hw_info->mirror)
	{
		cam_hw_info->mirror = LTPB__CAMERA_MODULE_HW_INFO__MIRROR_TYPE__MIRROR_UNKNOWN;
	}

	return cam_hw_info;
}

LIGHT_HEADER_FUNC Ltpb__HwInfo *get_hw_info(uint32_t m_bitmask)
{
	Ltpb__HwInfo *hw_info = (Ltpb__HwInfo *)gen_pvPortMalloc(light_header_heap, sizeof(Ltpb__HwInfo));

	ltpb__hw_info__init(hw_info);

	hw_info->n_camera = get_cam_total_number(m_bitmask);
	if (hw_info->n_camera == 0)
	{
		gen_vPortFree(light_header_heap, hw_info);
		return NULL;
	}
	hw_info->camera = (Ltpb__CameraModuleHwInfo **)gen_pvPortMalloc(light_header_heap,
			sizeof(Ltpb__CameraModuleHwInfo) * hw_info->n_camera);

	/* Select all camera if global bit mask was set */
	uint8_t cam_idx = 0;
	cam_typedef_t *pcam;
	uint8_t i = 0;
	m_bitmask &= light_system->m_filter;
	while(m_bitmask)
	{
		if(m_bitmask & 1)
		{
			pcam = idx_to_object(cam_idx);
			if(pcam != NULL)
			{
				/* Get camera hardware informations */
				hw_info->camera[i] = get_cam_hw_info(pcam);
				i++;
			}
		}
		m_bitmask >>= 1;
		cam_idx++;
	}

	hw_info->has_flash = TRUE;
	hw_info->flash = LTPB__HW_INFO__FLASH_TYPE__FLASH_UNKNOWN;

	hw_info->has_tof = TRUE;
	hw_info->tof = LTPB__HW_INFO__TO_FTYPE__TOF_STMICRO_VL53L0;

	return hw_info;
}

LIGHT_HEADER_FUNC uint8_t get_n_module_calib(uint32_t m_bitmask)
{
	uint8_t cam_idx = 0;
	uint8_t cam_calib = 0;
	cam_typedef_t *pcam;

	m_bitmask &= light_system->m_filter;
	while(m_bitmask)
	{
		if(m_bitmask & 1)
		{
			pcam = idx_to_object(cam_idx);
			if(pcam != NULL)
			{
				if(pcam->info.grp != GRP_A)
					cam_calib++;
			}
		}
		m_bitmask >>= 1;
		cam_idx++;
	}
	return cam_calib;
}

LIGHT_HEADER_FUNC Ltpb__FactoryModuleCalibration *get_module_calibration(cam_typedef_t *pcam)
{
	Ltpb__FactoryModuleCalibration *module_calib = gen_pvPortMalloc(light_header_heap,
			sizeof(Ltpb__FactoryModuleCalibration));

	ltpb__factory_module_calibration__init(module_calib);

	/* TODO: */
	return module_calib;
}

/* Exported functions --------------------------------------------------------*/
LIGHT_HEADER_FUNC Ltpb__LightHeader* alloc_init_light_header()
{
    Ltpb__LightHeader* lh = (Ltpb__LightHeader*)gen_pvPortMalloc(light_header_heap,
            sizeof(Ltpb__LightHeader));
    assert_param(lh != NULL);
    ltpb__light_header__init(lh);
    return lh;
}

LIGHT_HEADER_FUNC lightheader_error_t get_light_header(uint32_t m_bitmask, Ltpb__LightHeader *pheader)
{
	cam_typedef_t *pcam;

	/* Fill CAM bitmask on ASIC board */
	uint32_t cam_bitmask = m_bitmask & light_system->m_filter;
	uint8_t cam_idx = 0;

	uint8_t i = 0;
	uint8_t *p_tmp;
	/* uint8_t cam_calib_idx = 0; */
	/*
	 * captured camera properties
	 */
	pheader->n_modules = get_cam_total_number(m_bitmask);
	if(pheader->n_modules == 0)
	{
		return LIGHT_HEADER_NO_CAM_ERROR;
	}

	/* Allocate memory for each camera */
	pheader->modules = (Ltpb__CameraModule **)gen_pvPortMalloc(light_header_heap,
			sizeof(Ltpb__CameraModule) * (pheader->n_modules));


	if(pheader->modules == NULL)
	{
		return LIGHT_HEADER_INVALID_ARG;
	}

	/*
	 * color calibration data for the "golden" module
	 */
	/*pheader->n_sensor_data = pheader->n_modules;
	pheader->sensor_data = (Ltpb__LightHeader__SensorData **)gen_pvPortMalloc(light_header_heap,
			sizeof(Ltpb__LightHeader__SensorData) * (pheader->n_sensor_data));
*/
	/*
	 * global per capture properties
	 */
	/*
	 * low 64 bits of 128 bit UUID
	 */
	pheader->has_image_unique_id_low = TRUE;
	pheader->has_image_unique_id_high = TRUE;
	p_tmp = light_system->settings->snapshot_uuid;
	memcpy(&(pheader->image_unique_id_high), &(p_tmp[0]), sizeof(uint64_t));
	memcpy(&(pheader->image_unique_id_low), &(p_tmp[8]), sizeof(uint64_t));
	pheader->image_time_stamp = get_time_stamp();

	/* TODO: what is image reference camera ?*/
	cam_typedef_t *ref_cam;
	ref_cam = get_cam_thumbnail(m_bitmask);
	if(ref_cam != NULL)
	{
		pheader->has_image_reference_camera = TRUE;
		pheader->image_reference_camera = get_cam_id(ref_cam);
	}
	else
	{
		pheader->has_image_reference_camera = FALSE;
		pheader->image_reference_camera = 0;
	}

	/* Image_focal_length is per capture and for the whole image */
	pheader->has_image_focal_length = TRUE;
	pheader->image_focal_length = get_img_focal_length(ref_cam);

	/*
	 * global per device properties
	 */
	/*
	 * low 64 bits of 128 bit UUID
	 */
	pheader->has_device_unique_id_low = FALSE;
	pheader->device_unique_id_low = 0;

	/*
	 * high 64 bits of 128 bit UUID
	 */
	pheader->has_device_unique_id_high = FALSE;
	pheader->device_unique_id_high = 0;

	pheader->device_model_name = get_device_model_name();
	pheader->device_fw_version = get_device_fw_version();
	pheader->device_asic_fw_version = get_device_asic_fw_version();

	/* Get device temperature
	 * On P2 boead doesn't has temperature sensor */
	pheader->device_temperature = (Ltpb__DeviceTemp *)gen_pvPortMalloc(light_header_heap,
			sizeof(Ltpb__DeviceTemp));
	ltpb__device_temp__init(pheader->device_temperature);

#if (ASIC_NUM == ASIC1)
	pheader->device_temperature->sensor_1 = temp_sensor_read();
#else
	pheader->device_temperature->sensor_1 = 0;
#endif
	pheader->device_temperature->sensor_2 = 0;
	pheader->device_temperature->sensor_3 = 0;
	pheader->device_temperature->sensor_4 = 0;

	/*
	 * captured camera properties
	 */

	/*pheader->device_calibration = get_factory_device_cablib();

	pheader->n_module_calibration = get_n_module_calib(m_bitmask);
	if(pheader->n_module_calibration)
	{
		pheader->module_calibration =
				(Ltpb__FactoryModuleCalibration **)gen_pvPortMalloc(light_header_heap,
						sizeof(Ltpb__FactoryModuleCalibration));
	}*/

	/*
	 * color calibration data for the "golden" module
	 */
	/* size_t n_gold_cc; */
	/* Ltpb__ColorCalibrationGold **gold_cc; */

	/* Time-of-flight sensor properties. ToF sensor reading, distance in mm */
#if (ASIC_NUM == ASIC1)
	pheader->has_tof_range = TRUE;
#ifdef USING_TOF_FUNC
	pheader->tof_range = light_system->ToF_dev.Data.LastRangeMeasure.RangeMilliMeter;
#else
	pheader->tof_range = 0;
#endif /* USING_TOF_FUNC */
#else
	pheader->has_tof_range = 0;
	pheader->tof_range = 0;
#endif

	/*
	 * Hardware specification information block
	 */
	pheader->hw_info = get_hw_info(m_bitmask);

	/* Select all camera if global bit mask was set */
	i = 0;

	cam_bitmask &= light_system->m_filter;
	while(cam_bitmask)
	{
		if(cam_bitmask & 1)
		{
			pcam = idx_to_object(cam_idx);
			if(pcam != NULL)
			{
				/* Get camera module informations */
				pheader->modules[i] = get_cam_module(pcam);

				/* Get sensor data */
				/* pheader->sensor_data[i] = get_sensor_data(pcam); */

				/* Calibration cam module */
				/*if(pcam->info.grp != GRP_A)
				{
					pheader->module_calibration[cam_calib_idx] =
							get_module_calibration(pcam);
					cam_calib_idx++;
				}*/
				i++;
			}
		}
		cam_bitmask >>= 1;
		cam_idx++;
	}

	return LIGHT_HEADER_ERROR_NONE;
}

static void* alloc_pbuf_lh(void *allocator_data, size_t size) {
    return gen_pvPortMalloc(light_header_heap, size);
}

static void free_pbuf_lh(void *allocator_data, void *pointer) {
    gen_vPortFree(light_header_heap, pointer);
}

LIGHT_HEADER_FUNC void free_mem_light_header(Ltpb__LightHeader *pheader)
{
    ProtobufCAllocator light_header_pbuf_alloc =
    {
        alloc_pbuf_lh, free_pbuf_lh, NULL
    };
    ltpb__light_header__free_unpacked(pheader, &light_header_pbuf_alloc);
}

/*********** Portions COPYRIGHT 2016 Light.Co., Ltd. ********END OF FILE*******/
