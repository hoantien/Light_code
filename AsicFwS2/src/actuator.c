/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    actuator.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jul-08-2016
 * @brief   This file contains actuator piezoelectric control source code
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include "std_type.h"
#include "os.h"
#include "log.h"
#include "actuator.h"
#include "i2cm.h"
#include "eeprom.h"
#include "assert.h"
#include "timer.h"
#include "af_lens_calib.h"
#include "flash.h"

/* Private define ------------------------------------------------------------*/

#define SLOGF_ID						SLOG_ID_ACTUATOR_LOG

#define ACTUATOR_LENS_HARD_STOP_BUFFER      0x50
#define ACTUATOR_MIRROR_HARD_STOP_BUFFER    0x30

#define FINE_CONTROL_THRESHOLD			10
#define MAX_CONTROL_LOOP_TICKS		    250
#define PIEZO_INERTIA_DELTA_MOVE        8
#define NUDGE_FINE_DELAY_PERIOD			32
#define MIN_PIEZO_LOOP_WEIGHT           0.5f
#define MAX_PIEZO_LOOP_WEIGHT           50.0f
#define PIEZO_TOLERANCE_MIN             2
#define PIEZO_MIN_EXTRA_PULSES          -20
#define PIEZO_MAX_EXTRA_PULSES          40
#define PIEZO_PWM_FREQ_MIN              90000
#define PIEZO_PWM_FREQ_MAX              155000

#define ACTUATOR_DUTY_MOVE_LENS			30
#define ACTUATOR_DUTY_MOVE_MIRR			30
#define ACTUATOR_DUTY_LOW				30
#define ACTUATOR_DUTY_HIGH				30
#define ACTUATOR_DUTY_NUDGE				16

#define ACTUATOR_MIN_STROKE				100
#define ACTUATOR_MIN_STEP				3

#define EEPROM_AF_LENS_OFFSET			0x0023
#define EEPROM_AF_MIRR_OFFSET			0x0017
#define EEPROM_AF_LENS_FREQ_REG			0xAF
#define EEPROM_AF_MIRR_BACKUP_OFFSET	0x002C
#define EEPROM_AF_MIRR_VALID_VALUE		0xDB

#define FLASH_SUB_SECTOR_SIZE			0x00001000
#define FLASH_CALIB_SUB_SECTOR			0x0150
#define FLASH_CALIB_VERIFICATION		0xDEADBEEF

#define ACTUATOR_NUM_CALIB_POINTS		4

#define IS_ACTUATOR_TYPE(TYPE)			((TYPE == ACTUATOR_LENS) || \
										(TYPE == ACTUATOR_MIRROR))

#define IS_ACTUATOR_DIR(DIR)			((DIR == ACTUATOR_DIR_RETRACT_NARROW) || \
										(DIR == ACTUATOR_DIR_EXTEND_WIDE))

#define IS_ACTUATOR_FREQ(FREQ)			((FREQ >= PIEZO_PWM_FREQ_MIN) && (FREQ <= PIEZO_PWM_FREQ_MAX))

/* Private typedef -----------------------------------------------------------*/
/*
 * @brief actuator_data_valid_t
 *
 * PIEZO EEPROM calibration data validation type
 */
typedef enum
{
	ACTUATOR_DATA_OK				= 0,
	ACTUATOR_INVALID_FREQ			= 1 << 0,
	ACTUATOR_INVALID_POLARITY		= 1 << 1,
	ACTUATOR_INVALID_SENSITIVITY	= 1 << 2,
	ACTUATOR_INVALID_HARD_STOP		= 1 << 3,
	ACTUATOR_NO_DATA				= 1 << 4
} data_valid_t;

/*
 * @brief eeprom_af_lens_reg_t
 *
 * EEPROM AF lens registers structure
 */
typedef struct
{
	__I uint8_t		POSITION_MACRO_H;
	__I uint8_t		POSITION_MACRO_L;
	__I uint8_t		POSITION_1M_H;
	__I uint8_t		POSITION_1M_L;
	__I uint8_t		POSITION_2M_H;
	__I uint8_t		POSITION_2M_L;
	__I uint8_t		POSITION_INFINITY_H;
	__I uint8_t		POSITION_INFINITY_L;
	__I uint8_t		Reserved[16];
	__I uint8_t		HARD_STOP_MACRO_H;
	__I uint8_t		HARD_STOP_MACRO_L;
	__I uint8_t		HARD_STOP_INFINITY_H;
	__I uint8_t		HARD_STOP_INFINITY_L;
	__I uint8_t		HOME_POSITION_H;
	__I uint8_t		HOME_POSITION_L;
	__I uint8_t		FULL_STROKE_H;
	__I uint8_t		FULL_STROKE_L;
	__I uint8_t		TEMPERATURE;
	__I uint8_t		POLARITY;
	__I uint8_t		SENSITIVITY;
} eeprom_af_lens_reg_t;

/*
 * @brief eeprom_af_mirr_reg_t
 *
 * EEPROM AF mirror registers structure
 */
typedef struct
{
	__I uint8_t		HARD_STOP_MACRO_H;
	__I uint8_t		HARD_STOP_MACRO_L;
	__I uint8_t		HARD_STOP_INFINITY_H;
	__I uint8_t		HARD_STOP_INFINITY_L;
	__I uint8_t		POSITION_45DEGREE_H;
	__I uint8_t		POSITION_45DEGREE_L;
	__I uint8_t		POLARITY;
	__I uint8_t		SENSITIVITY;
	__I uint8_t		Reserved[39];
	__I uint8_t		FREQUENCY;
} eeprom_af_mirr_reg_t;

/*
 * @brief lens_calib_data_t
 *
 * Lens calibration data structure
 */
typedef struct
{
	uint32_t	pwm_freq;
	af_calib_point_t  af_calib_points[ACTUATOR_NUM_CALIB_POINTS];
	af_calib_t  af_calib;
	uint16_t	hard_stop[2];
	uint8_t		polarity;
	uint8_t		sensitivity;
} lens_calib_data_t;

/*
 * @brief mirr_calib_data_t
 *
 * Mirror calibration data structure
 */
typedef struct
{
	uint32_t	verification;
	uint32_t	pwm_freq;
	uint16_t	position_45d;
	uint16_t	hard_stop[2];
	uint8_t		polarity;
	uint8_t		sensitivity;
} mirr_calib_data_t;

/* Private function prototype ------------------------------------------------*/
static actuator_return_t read_eeprom(actuator_t *piezo,
										data_valid_t *data_valid);
static void log_piezo_msg(actuator_t *piezo, uint8_t header,
							const char *format, ...);
static data_valid_t check_calib_info_validation(actuator_t *piezo);
static void print_calib_info(actuator_t *piezo);
static uint16_t convert_to_position(lens_calib_data_t *calib_data,
									uint32_t distance);
//static hall_return_t move_to_end(actuator_t *piezo, actuator_dir_t direction);
static actuator_return_t actuator_calibrate_hard_stop(actuator_t *piezo,
											uint16_t target_position);
static inline uint8_t get_derived_pwm_duty(uint8_t duty_cycle,
											actuator_dir_t direction);
static void pwm_pattern_callback(void *cb_param);
static actuator_return_t actuator_do_move_to_position(actuator_t *piezo,
                                            uint16_t *target_position,
                                            uint16_t tolerance,
                                            bool is_hard_stop_calibration);

/* Private variable ----------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
/*
 * @brief actuator_init
 * Initializes independent piezo module
 */
actuator_return_t actuator_init(actuator_t *piezo)
{
	actuator_return_t piezo_ret = ACTUATOR_OK;
	hall_return_t	hall_ret = HALL_OK;

	/* assert parameters */
	assert_param(piezo != NULL);
	assert_param(IS_ACTUATOR_TYPE(piezo->type));

	/* Do nothing if the actuator has already been initialized */
	if (piezo->is_initialized == FALSE)
	{
		data_valid_t data_valid;

		/* initialize I2C channels */
		i2cm.init(piezo->hall_i2c);
		i2cm.init(piezo->eeprom_i2c);

		/* assign hall object */
		piezo->hall_obj = (piezo->type == ACTUATOR_LENS) ? &a1457 : &as5510;
		/* reset the calibration status */
		piezo->is_calibrated = FALSE;
		/* update the initialization status */
		piezo->is_initialized = TRUE;
		for (size_t i = 0; i < ACTUATOR_NUM_DIR; ++i)
		{
            piezo->piezo_param.per_dir_param[i].weight_coarse = (piezo->type == ACTUATOR_LENS) ? 4.0f : 12.0f;
            piezo->piezo_param.per_dir_param[i].extra_num_pulses = 5;
		}
		/* allocate memory for storing calibration data */
		piezo->af_calib = assert_malloc(piezo->af_calib,
							(piezo->type == ACTUATOR_LENS) ?
							sizeof(lens_calib_data_t) :
							sizeof(mirr_calib_data_t));
		if (piezo->type == ACTUATOR_LENS)
		{
			/* read preset calibration data from EEPROM */
			piezo_ret = read_eeprom(piezo, &data_valid);
			if (piezo_ret == ACTUATOR_OK)
			{
				/* if the preset calibration data are invalid */
				if (data_valid != ACTUATOR_DATA_OK)
				{
					/* if lens frequency is invalid */
					if ((data_valid & ACTUATOR_INVALID_FREQ) ==
							ACTUATOR_INVALID_FREQ)
					{
						/* hard code lens frequency */
						((lens_calib_data_t *)piezo->af_calib)->pwm_freq =
												128000;
						/* clear this error flag from data_valid */
						data_valid &= ~ACTUATOR_INVALID_FREQ;
					}
				}
				/* if there are no other invalidations */
				if (data_valid != ACTUATOR_DATA_OK)
					piezo_ret = ACTUATOR_EEPROM_INVALID_DATA;
			}

			if (piezo_ret == ACTUATOR_OK)
			{
				/* set the sensitivity level and polarity of magnet */
				hall_ret = piezo->hall_obj->init(piezo->hall_i2c,
						((lens_calib_data_t *)piezo->af_calib)->polarity);
				hall_ret |= piezo->hall_obj->set_sensitivity(piezo->hall_i2c,
						((lens_calib_data_t *)piezo->af_calib)->sensitivity);

				if (hall_ret == HALL_OK)
					piezo->is_calibrated = TRUE;
				else
					piezo_ret = ACTUATOR_HALL_TIMED_OUT;
			}
		}
		else
		{
#ifndef ALWAYS_CALIB_MIRROR
			/* read preset calibration data from EEPROM */
			piezo_ret = read_eeprom(piezo, &data_valid);
			if (piezo_ret == ACTUATOR_OK && data_valid == ACTUATOR_DATA_OK)
			{
				/* Initialize the mirror */
				hall_ret = piezo->hall_obj->init(piezo->hall_i2c,
						((mirr_calib_data_t *)piezo->af_calib)->polarity);
				hall_ret |= piezo->hall_obj->set_sensitivity(piezo->hall_i2c,
						((mirr_calib_data_t *)piezo->af_calib)->sensitivity);

				if (hall_ret == HALL_OK)
					piezo->is_calibrated = TRUE;
				else
					piezo_ret = ACTUATOR_HALL_TIMED_OUT;
			}
			else
			{
				/* read calibration data from FLASH */
				log_piezo_msg(piezo, SLOG_DEBUG, "Prepare to read from FLASH");
				flash_read((FLASH_CALIB_SUB_SECTOR + piezo->cam_info.idx - 1) *
						FLASH_SUB_SECTOR_SIZE,
						sizeof(mirr_calib_data_t), piezo->af_calib, QSPI_MODE);

				if (((mirr_calib_data_t *)piezo->af_calib)->verification
						== FLASH_CALIB_VERIFICATION)
				{
					/* print the calibration info from FLASH */
					log_piezo_msg(piezo, SLOG_DEBUG,
							"Calibration info from FLASH:");
					print_calib_info(piezo);

					/* set the sensitivity level and polarity of magnet */
					hall_ret = piezo->hall_obj->init(piezo->hall_i2c,
						((mirr_calib_data_t *)piezo->af_calib)->polarity);
					hall_ret |=
						piezo->hall_obj->set_sensitivity(piezo->hall_i2c,
						((mirr_calib_data_t *)piezo->af_calib)->sensitivity);

					if (hall_ret == HALL_OK)
						piezo->is_calibrated = TRUE;
					else
						piezo_ret = ACTUATOR_HALL_TIMED_OUT;
				}
				/*
				 * do calibration
				 * as the data are not valid in EEPROM and FLASH
				 */
				else
				{
					log_piezo_msg(piezo, SLOG_DEBUG,
							"There is no calibration info from FLASH");
#endif
					log_piezo_msg(piezo, SLOG_DEBUG,
							"Prepare to calibrate actuator");
					/* calibrate mirror */
					piezo_ret = actuator_calibrate(piezo);
#ifndef ALWAYS_CALIB_MIRROR
				}
			}
#endif
		}
		piezo->piezo_param.pwm_freq = (piezo->type == ACTUATOR_LENS) ?
                ((lens_calib_data_t *)piezo->af_calib)->pwm_freq :
                ((mirr_calib_data_t *)piezo->af_calib)->pwm_freq;
		piezo->piezo_param.freq_search_step = 1;
        piezo->piezo_param.last_move_dir = ACTUATOR_NUM_DIR;
	}

	return piezo_ret;
}

/*
 * @brief actuator_calibrate
 * Calibrates independent piezo module
 */
actuator_return_t actuator_calibrate(actuator_t *piezo)
{
	actuator_return_t piezo_ret = ACTUATOR_OK;

	/* check parameter validation */
	if (piezo == NULL)
		piezo_ret = ACTUATOR_NULL_PTR;
	else if (!IS_ACTUATOR_TYPE(piezo->type))
		piezo_ret = ACTUATOR_INVALID_TYPE;
	else if (piezo->cam_info.grp == GRP_A)
		piezo_ret = ACTUATOR_UNSUPPORTED_CAM_GROUP;
	else if (piezo->is_initialized == FALSE)
		piezo_ret = ACTUATOR_UNINITIALIZED;
	else
	{
		if (piezo->type == ACTUATOR_LENS)
		{
			/* do nothing */
		}
		/* if (piezo->type == ACTUATOR_MIRROR) */
		else
		{
			mirr_calib_data_t	*calib_data = piezo->af_calib;
			hall_return_t	hall_ret = HALL_OK;
			hall_t		*hall_obj = piezo->hall_obj;
			i2c_t		i2c_chid = piezo->hall_i2c;
			uint16_t	near_positions[hall_obj->nbo_sensitivities];
			uint16_t	far_positions[hall_obj->nbo_sensitivities];
#if(LOG_VERBOSE == STD_ON)
			uint16_t	nearest_position = 0;
			uint16_t	farthest_position = 0;
#endif
			uint16_t	best_hall_stroke = ACTUATOR_MIN_STROKE;
			uint8_t		best_sensitivity = 0;
			bool		best_polarity_reversal = FALSE;

			if (!IS_ACTUATOR_FREQ(calib_data->pwm_freq))
			{
				/* hard code mirror frequency */
				calib_data->pwm_freq = 135000;
			}
			/* reset the polarity */
			hall_ret = hall_obj->init(i2c_chid, FALSE);
			/* sensitivity hard coded to 1 */
			hall_ret = hall_obj->set_sensitivity(i2c_chid, 1);

			if (hall_ret == HALL_OK)
			{
				piezo->piezo_param.pwm_freq = calib_data->pwm_freq;
				/* read hall sensor at the nearest position */
				hall_ret = actuator_calibrate_hard_stop(piezo,0xFFFF);
				hall_ret |= hall_obj->scan_sensitivities(i2c_chid,
														near_positions);
				/* read hall sensor at the farthest position */
				hall_ret |= actuator_calibrate_hard_stop(piezo,0x0000);
				hall_ret |= hall_obj->scan_sensitivities(i2c_chid,
														far_positions);
			}

			if (hall_ret == HALL_OK)
			{
				uint16_t	near_position;
				uint16_t	far_position;
				uint16_t	hall_stroke;
				uint8_t		sensitivity;
				bool		polarity_reversal;

				/* invalidate best hall stroke */
				best_hall_stroke = 0;
				/*
				 * analyze the data
				 * to select the best sensitivity & polarity
				 * for full-range operation
				 */
#ifdef MIRROR_HARD_CODE_SENSITIVITY
				sensitivity = 1;
#else
				for (sensitivity = 0;
						sensitivity < hall_obj->nbo_sensitivities;
						sensitivity++)
#endif
				{
					near_position = near_positions[sensitivity];
					far_position = far_positions[sensitivity];

					if (near_position > far_position)
						polarity_reversal = FALSE;
					else
					{
						/* change polarity & invert data */
						polarity_reversal = TRUE;
						near_position = hall_obj->max_position - near_position;
						far_position = hall_obj->max_position - far_position;
					}

					hall_stroke = near_position - far_position;

					/*
					 * check to see that
					 * the stroke is neither too wide nor too narrow
					 */
#ifndef MIRROR_HARD_CODE_SENSITIVITY
					if ((far_position > 0) &&
						(near_position < hall_obj->max_position) &&
						(hall_stroke > best_hall_stroke))
#endif
					{
						best_hall_stroke = hall_stroke;
#if(LOG_VERBOSE == STD_ON)
						nearest_position = near_position;
						farthest_position = far_position;
#endif
						best_sensitivity = sensitivity;
						best_polarity_reversal = polarity_reversal;
					}
				}
			}

#if(LOG_VERBOSE == STD_ON)
			/* print calibration data */
			if (hall_ret == HALL_OK)
			{
				log_piezo_msg(piezo, SLOG_DEBUG, "Calibration data:");
				SLOGF(SLOG_INFO, "Best hall polarity reversal: %d",
						best_polarity_reversal);
				SLOGF(SLOG_INFO, "Best hall sensitivity: %d",
						best_sensitivity);
				SLOGF((best_hall_stroke >= ACTUATOR_MIN_STROKE) ?
							SLOG_INFO : SLOG_ERROR,
							"Narrowest position: 0x%X", farthest_position);
				SLOGF((best_hall_stroke >= ACTUATOR_MIN_STROKE) ?
							SLOG_INFO : SLOG_ERROR,
							"Widest position: 0x%X", nearest_position);
			}
			else
				log_piezo_msg(piezo, SLOG_ERROR, "Hall access timed out");
#endif
			/* if the calibration was successful */
			if ((hall_ret == HALL_OK) &&
				(best_hall_stroke >= ACTUATOR_MIN_STROKE))
			{
				/* store the calibration data to buffer */
				calib_data->hard_stop[0] = nearest_position;
				calib_data->hard_stop[1] = farthest_position;
				calib_data->polarity = best_polarity_reversal;
				calib_data->sensitivity = best_sensitivity;
				calib_data->verification = FLASH_CALIB_VERIFICATION;
				/* and write into the FLASH */
				flash_sub_sector_erase(FLASH_CALIB_SUB_SECTOR +
											piezo->cam_info.idx - 1, 1);
				flash_write((FLASH_CALIB_SUB_SECTOR + piezo->cam_info.idx - 1) *
								FLASH_SUB_SECTOR_SIZE,
					sizeof(mirr_calib_data_t), piezo->af_calib, QSPI_MODE);
				log_piezo_msg(piezo, SLOG_DEBUG,
								"Wring to FLASH was successful");
				/* set the sensitivity level and polarity of magnet */
				hall_ret = hall_obj->init(i2c_chid,
											best_polarity_reversal);
				hall_ret |= hall_obj->set_sensitivity(i2c_chid,
											best_sensitivity);
			}

			piezo_ret = (hall_ret == HALL_OK) ?
							((best_hall_stroke >= ACTUATOR_MIN_STROKE) ?
								ACTUATOR_OK : ACTUATOR_MOVE_TIMED_OUT) :
						ACTUATOR_HALL_TIMED_OUT;
		}

		/* update calibration status */
		piezo->is_calibrated = (piezo_ret == ACTUATOR_OK) ? TRUE : FALSE;
	}

#if(LOG_VERBOSE == STD_ON)
	if (piezo_ret == ACTUATOR_OK)
		log_piezo_msg(piezo, SLOG_DEBUG, "Piezo calibration was successful");
	else
		log_piezo_msg(piezo, SLOG_ERROR, "Piezo calibration was failed");
#endif

	return piezo_ret;
}

actuator_return_t actuator_get_hard_stop_hall(actuator_t *piezo,
											uint16_t *hall_min,
											uint16_t *hall_max)
{
	if (NULL == piezo || hall_min == NULL || hall_max == NULL)
	{
		SLOGF(SLOG_ERROR, "%s: Null pointer", __FUNCTION__);
		return ACTUATOR_NULL_PTR;
	}
	else if (!IS_ACTUATOR_TYPE(piezo->type))
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Invalid type");
		return ACTUATOR_INVALID_TYPE;
	}
	else if (piezo->is_initialized == FALSE)
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Uninitialized");
		return ACTUATOR_UNINITIALIZED;
	}
	else if (piezo->is_calibrated == FALSE)
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Not calibrated yet");
		return ACTUATOR_NOT_CALIBRATED;
	}

	if (piezo->type == ACTUATOR_LENS)
	{
		*hall_min = (((lens_calib_data_t *)piezo->af_calib)->hard_stop[1] +
													ACTUATOR_LENS_HARD_STOP_BUFFER);
		*hall_max = (((lens_calib_data_t *)piezo->af_calib)->hard_stop[0] -
													ACTUATOR_LENS_HARD_STOP_BUFFER);
	}
	else
	{
		*hall_min = (((mirr_calib_data_t *)piezo->af_calib)->hard_stop[1] +
													ACTUATOR_MIRROR_HARD_STOP_BUFFER);
		*hall_max = (((mirr_calib_data_t *)piezo->af_calib)->hard_stop[0] -
													ACTUATOR_MIRROR_HARD_STOP_BUFFER);
	}

	return ACTUATOR_OK;
}

static actuator_return_t actuator_calibrate_hard_stop(actuator_t *piezo,
											uint16_t target_position)
{
    return actuator_do_move_to_position(piezo, &target_position, 0, true);
}

actuator_return_t actuator_read_position(actuator_t *piezo, uint16_t *hallcode)
{
    actuator_return_t   piezo_ret = ACTUATOR_OK;
    hall_return_t       hall_ret;
    hall_ret = piezo->hall_obj->read_position(piezo->hall_i2c, hallcode);
    if (hall_ret != HALL_OK)
        piezo_ret = ACTUATOR_HALL_TIMED_OUT;
    return piezo_ret;
}

/*
 * @brief actuator_move_to_position
 * Moves independent piezo module to a position
 */
actuator_return_t actuator_move_to_position(actuator_t *piezo,
                                            uint16_t *target_position,
                                            uint16_t tolerance)
{
	return actuator_do_move_to_position(piezo, target_position, tolerance, false);
}

static uint32_t actuator_get_pwm_freq(piezo_cl_param_t *piezo_param, actuator_dir_t direction)
{
    uint32_t pwm_freq = piezo_param->pwm_freq;
    if (direction != piezo_param->last_move_dir)
        return pwm_freq;
    if (pwm_freq <= PIEZO_PWM_FREQ_MIN)
        piezo_param->freq_search_step = 1;
    else if (pwm_freq >= PIEZO_PWM_FREQ_MAX)
        piezo_param->freq_search_step = -1;
    return pwm_freq + 1000 * piezo_param->freq_search_step;
}

/*
 * @brief actuator_move_to_position
 * Moves independent piezo module to a position
 */
static actuator_return_t actuator_do_move_to_position(actuator_t *piezo,
											uint16_t *target_position,
											uint16_t tolerance,
											bool is_hard_stop_calibration)
{
    if (tolerance < PIEZO_TOLERANCE_MIN)
        tolerance = PIEZO_TOLERANCE_MIN;
	actuator_return_t	piezo_ret = ACTUATOR_OK;
	hall_return_t		hall_ret;

	/* check parameter validation */
	if ((piezo == NULL) || (target_position == NULL))
	{
		SLOGF(SLOG_ERROR, "%s: Null pointer", __FUNCTION__);
		piezo_ret = ACTUATOR_NULL_PTR;
	}
	else if (!IS_ACTUATOR_TYPE(piezo->type))
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Invalid type");
		piezo_ret = ACTUATOR_INVALID_TYPE;
	}
	else if (piezo->is_initialized == FALSE)
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Uninitialized");
		piezo_ret = ACTUATOR_UNINITIALIZED;
	}
	else if ((!is_hard_stop_calibration) && (piezo->is_calibrated == FALSE))
	{
		/* update the current position to upper layer */
		hall_ret = piezo->hall_obj->read_position(piezo->hall_i2c,
				target_position);
		if (hall_ret != HALL_OK)
			*target_position = 0;
		piezo->last_move_hall_code = *target_position;
		log_piezo_msg(piezo, SLOG_ERROR, "Not calibrated yet");
		piezo_ret = ACTUATOR_NOT_CALIBRATED;
	}

	/* check whether the piezo module channel was calibrated */
	if (piezo_ret == ACTUATOR_OK)
	{
	    uint16_t target_pos = *target_position;
		uint16_t current_position;

		/* read current position of actuator */
		hall_ret = piezo->hall_obj->read_position(piezo->hall_i2c, &current_position);

		if (hall_ret == HALL_OK)
		{
			hal_pwm_init_t pwm_cfg;
			hal_pwm_output_t pwm_value;
		    uint16_t actual_move = 1;
		    uint16_t remaining_distance;
			uint16_t start_position;
			uint8_t duty_cycle;
			actuator_dir_t direction;

			/* PWM channel config */
			pwm_cfg.align_mode = PWM_ALIGN_RIGHT;
			pwm_cfg.output_gen_mode = PWM_FIXED_PATTERNS;
			pwm_value.pattern.num_pairs = 2;
			pwm_value.pattern.pair[0].duty = 0;
            pwm_value.pattern.pair[0].num_pulses = 1;
            pwm_value.pattern.pair[1].duty = 0;
            pwm_value.pattern.pair[1].num_pulses = 1;

			/* Enable IRQ */
			hal_pwm_irq_t pwm_irq;
			pwm_irq.irq_mode = PWM_IRQ_ONE_PATTERN_GENERATED;
			pwm_irq.callback_handler = pwm_pattern_callback;
			pwm_irq.cb_param = piezo;
			hal_pwm_enable_irq(piezo->pwm, &pwm_irq);

			if (!is_hard_stop_calibration)
			{
			    uint16_t hc_min;
			    uint16_t hc_max;
			    actuator_get_hard_stop_hall(piezo, &hc_min, &hc_max);
                /* Clamp the target position */
                target_pos = CLAMP(target_pos, hc_min, hc_max);
			}
			/* Save the target position after clamping */
			*target_position = target_pos;
			/* Compute the remaining distance */
			remaining_distance = abs(target_pos - current_position);

			TickType_t start_tick_count = xTaskGetTickCount();
			TickType_t end_tick_count = start_tick_count;

			while ((remaining_distance > tolerance) &&
					((end_tick_count - start_tick_count) <= MAX_CONTROL_LOOP_TICKS) &&
					(hall_ret == HALL_OK))
			{
				/* Store the start position */
				start_position = current_position;

				/* Adjust duty cycle and weight based on the remaining distance */
                bool is_fine_mode;
				if (remaining_distance > FINE_CONTROL_THRESHOLD)
				{
				    is_fine_mode = false;
				    duty_cycle = (piezo->type == ACTUATOR_LENS) ? ACTUATOR_DUTY_MOVE_LENS : ACTUATOR_DUTY_MOVE_MIRR;
				}
				else
				{
				    is_fine_mode = true;
					duty_cycle = ACTUATOR_DUTY_LOW;
				}

				/* Program the PWM block to send the correct number of pulses in a single pattern */
				float distance_factor = 1.0f;
				if (remaining_distance > 2000)
				    distance_factor = 0.3f;
				else if (remaining_distance > 500)
				    distance_factor = 0.7f;
				else if (remaining_distance > 100)
				    distance_factor = 0.9f;
				float expected_move = distance_factor * remaining_distance;
				/* Calculate direction for the next movement */
#ifdef EVT3_NEW_MIRR_MODULE
				if (piezo->type == ACTUATOR_LENS)
				{
					direction = target_pos > current_position ?
							ACTUATOR_DIR_EXTEND_WIDE : ACTUATOR_DIR_RETRACT_NARROW;
				}
				else
				{
					direction = target_pos > current_position ?
							ACTUATOR_DIR_RETRACT_NARROW : ACTUATOR_DIR_EXTEND_WIDE;
				}
#else
				direction = target_pos > current_position ?
						ACTUATOR_DIR_EXTEND_WIDE : ACTUATOR_DIR_RETRACT_NARROW;
#endif
				piezo_cl_per_dir_param_t* cl_param = &piezo->piezo_param.per_dir_param[direction];
				/* Update PWM duty cycle and start moving actuator */
				float weight = cl_param->weight_coarse;
				int32_t num_pulses = weight * expected_move + cl_param->extra_num_pulses;
				if (num_pulses < 1)
				    num_pulses = 1;
				pwm_value.pattern.pair[0].num_pulses = num_pulses;
				pwm_value.pattern.pair[0].duty =
									get_derived_pwm_duty(duty_cycle, direction);
	            uint32_t pwm_freq = actuator_get_pwm_freq(&piezo->piezo_param, direction);
	            /* Add 1ms of time at the end to ensure that piezo has come to a stop when we read the position */
	            pwm_value.pattern.pair[1].num_pulses = pwm_freq / 1000;
	            pwm_value.freq = pwm_freq;
	            hal_pwm_init(piezo->pwm, &pwm_cfg, &pwm_value);
				hal_pwm_start(piezo->pwm);
#if 0
				SLOGF(SLOG_DEBUG, "Piezo move loop, is_fine_mode: %d, num_pulses: %d, remaining_distance: %u, weight: %f, expected_move: %f",
				        is_fine_mode, num_pulses, remaining_distance, weight, expected_move);
#endif
				if (xSemaphoreTake(piezo->pwm_semaphore, portMAX_DELAY) != pdTRUE )
				{
					hal_pwm_stop(piezo->pwm);
                    log_piezo_msg(piezo, SLOG_ERROR, "Error in piezo control, giving up");
                    piezo_ret = ACTUATOR_MOVE_TIMED_OUT;
					break;
				}

				/* Read the current position */
				hall_ret = piezo->hall_obj->read_position(piezo->hall_i2c, &current_position);
				actual_move = abs(current_position - start_position);

				if (direction == piezo->piezo_param.last_move_dir)
				{
                    if (actual_move >= expected_move)
                        piezo->piezo_param.pwm_freq = pwm_freq;
                    else
                        piezo->piezo_param.freq_search_step *= -1;
				}
				piezo->piezo_param.last_move_dir = direction;
				float extra_pulses_comp = cl_param->extra_num_pulses;
				if (is_fine_mode)
				{
				    /* Adjust min number of pulses */
				    extra_pulses_comp = num_pulses - weight * actual_move;
				    if (extra_pulses_comp < PIEZO_MIN_EXTRA_PULSES)
				        extra_pulses_comp = PIEZO_MIN_EXTRA_PULSES;
				    else if (extra_pulses_comp > PIEZO_MAX_EXTRA_PULSES)
				        extra_pulses_comp = PIEZO_MAX_EXTRA_PULSES;
				    cl_param->extra_num_pulses = 0.5f * cl_param->extra_num_pulses + 0.5f * extra_pulses_comp;
				}
                /* Update weight */
				if ((actual_move > 0) || !is_fine_mode) {
                    if (actual_move == 0) {
                        weight = MAX_PIEZO_LOOP_WEIGHT;
                    } else {
                        weight = (float)(num_pulses - extra_pulses_comp) / actual_move;
                        if (weight > MAX_PIEZO_LOOP_WEIGHT)
                            weight = MAX_PIEZO_LOOP_WEIGHT;
                        else if (weight < MIN_PIEZO_LOOP_WEIGHT)
                            weight = MIN_PIEZO_LOOP_WEIGHT;
                    }
                    cl_param->weight_coarse = 0.5f * cl_param->weight_coarse + 0.5f * weight;
				}

#if 0
                SLOGF(SLOG_DEBUG, "Piezo move loop, start_pos: %u, current_pos: %u, new_pwm_freq; %u, new_weight: %f, new_extra_pulses: %i",
                        start_position, current_position, piezo->piezo_param.pwm_freq,
                        cl_param->weight_coarse, cl_param->extra_num_pulses);
#endif
                /* Update the remaining distance */
				remaining_distance = abs(target_pos - current_position);

	            end_tick_count = xTaskGetTickCount();
			}
			log_piezo_msg(piezo, SLOG_DEBUG, "Total move time (end_tick - start_tick) = %d",
					(end_tick_count - start_tick_count));

			/* Update the target position to upper layer */
			*target_position = current_position;
			piezo->last_move_hall_code = current_position;
		}

		if (hall_ret != HALL_OK)
		{
			*target_position = 0;
			piezo->last_move_hall_code = *target_position;
			log_piezo_msg(piezo, SLOG_ERROR, "Hall access timed out");
			piezo_ret = ACTUATOR_HALL_TIMED_OUT;
		}
		else if (abs(target_pos - current_position) > tolerance)
		{
            if (!is_hard_stop_calibration)
            {
                log_piezo_msg(piezo, SLOG_ERROR, "Movement timed out");
                piezo_ret = ACTUATOR_MOVE_TIMED_OUT;
            }
		}

        /* Debug logs */
        log_piezo_msg(piezo, SLOG_INFO, "Stopped at position: %u", current_position);
        log_piezo_msg(piezo, SLOG_DEBUG, "PWM frequency: %u, Weight/extra_num_pulses: %f, %i; %f, %i",
                piezo->piezo_param.pwm_freq,
                piezo->piezo_param.per_dir_param[0].weight_coarse, piezo->piezo_param.per_dir_param[0].extra_num_pulses,
                piezo->piezo_param.per_dir_param[1].weight_coarse, piezo->piezo_param.per_dir_param[1].extra_num_pulses);
	}

	return piezo_ret;
}

/*
 * @brief actuator_move_to_distance
 * Focuses independent lens piezo module to a distance
 */
actuator_return_t actuator_move_to_distance(actuator_t *piezo,
											uint32_t *distance)
{
	actuator_return_t	piezo_ret = ACTUATOR_OK;

	/* check parameter validation */
	if ((piezo == NULL) || (distance == NULL))
	{
		SLOGF(SLOG_ERROR, "%s: Null pointer", __FUNCTION__);
		piezo_ret = ACTUATOR_NULL_PTR;
	}
	else
	{
		uint16_t	target_position;

		/* compute lens position from distance */
		target_position = convert_to_position(piezo->af_calib, *distance);
		SLOGF(SLOG_INFO, "Target position: 0x%X", target_position);

		/* move the lens to this position */
		piezo_ret = actuator_move_to_position(piezo, &target_position, 3);
	}

	return piezo_ret;
}

/*
 * @brief actuator_nudge
 * Nudges independent piezo module in the desired direction
 * 		with a little burst of PWM.
 */
actuator_return_t actuator_nudge(actuator_t *piezo, actuator_dir_t direction,
				uint16_t multiplier, void (*cb_func)(void *), void *cb_param)
{
	actuator_return_t	piezo_ret;

	/* check parameter validation */
	if (piezo == NULL)
	{
		SLOGF(SLOG_ERROR, "%s: Null pointer", __FUNCTION__);
		piezo_ret = ACTUATOR_NULL_PTR;
	}
	else if (!IS_ACTUATOR_TYPE(piezo->type))
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Invalid type");
		piezo_ret = ACTUATOR_INVALID_TYPE;
	}
	else if (piezo->is_initialized == FALSE)
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Uninitialized");
		piezo_ret = ACTUATOR_UNINITIALIZED;
	}
	else if (piezo->is_calibrated == FALSE)
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Not calibrated yet");
		piezo_ret = ACTUATOR_NOT_CALIBRATED;
	}
	else if (!IS_ACTUATOR_DIR(direction))
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Invalid direction");
		piezo_ret = ACTUATOR_INVALID_DIR;
	}
	else
	{
		hal_pwm_init_t		pwm_cfg;
		hal_pwm_output_t	pwm_value;
		hal_pwm_irq_t		pwm_irq;

		/* configure PWM parameters */
		pwm_cfg.align_mode = PWM_ALIGN_RIGHT;
		pwm_cfg.output_gen_mode = PWM_FIXED_PATTERNS;
		pwm_value.freq = piezo->piezo_param.pwm_freq;
		pwm_value.num_patterns = 1;
		pwm_value.pattern.num_pairs = 1;
		pwm_value.pattern.pair[0].num_pulses = multiplier;
		pwm_value.pattern.pair[0].duty =
						get_derived_pwm_duty(ACTUATOR_DUTY_NUDGE, direction);
		pwm_irq.irq_mode = PWM_IRQ_ALL_PATTERNS_GENERATED;
		pwm_irq.callback_handler = cb_func;
		pwm_irq.cb_param = cb_param;
		hal_pwm_init(piezo->pwm, &pwm_cfg, &pwm_value);
		hal_pwm_enable_irq(piezo->pwm, &pwm_irq);
		/* start moving actuator */
		hal_pwm_start(piezo->pwm);

		piezo_ret = ACTUATOR_OK;
	}

	return piezo_ret;
}

actuator_return_t actuator_object_distance_to_lens_hallcode(actuator_t *piezo,
                uint32_t object_distance_mm, uint16_t *hallcode)
{
    *hallcode = convert_to_position(piezo->af_calib, object_distance_mm);
    return ACTUATOR_OK;
}

actuator_return_t actuator_lens_hallcode_to_object_distance(actuator_t *piezo,
                uint16_t hallcode, uint32_t *object_distance_mm)
{
    *object_distance_mm = hallcode_to_object_distance(&((lens_calib_data_t *)piezo->af_calib)->af_calib, hallcode);
    return ACTUATOR_OK;
}

/* Private function --------------------------------------------------------- */

/*
 * @brief read_eeprom
 * Reads and checks the calibration data from EEPROM of independent piezo module
 */
static actuator_return_t read_eeprom(actuator_t *piezo,
										data_valid_t *data_valid)
{
	actuator_return_t piezo_ret = ACTUATOR_OK;
	eeprom_status_t eeprom_ret;
	hall_return_t hall_ret = HALL_OK;
	uint32_t	pwm_freq = 0;

	/* read preset calibration data for actuator from the EEPROM */
	if (piezo->type == ACTUATOR_LENS)
	{
		eeprom_af_lens_reg_t	lens_regs;

		eeprom_ret = eeprom_read(piezo->eeprom_i2c, EEPROM_AF_LENS_OFFSET,
						sizeof(eeprom_af_lens_reg_t), (void *)&lens_regs);
		hall_ret = piezo->hall_obj->read_register(piezo->hall_i2c,
						EEPROM_AF_LENS_FREQ_REG, 1, (uint8_t *)&pwm_freq);
		if (eeprom_ret == EEPROM_OK && hall_ret == HALL_OK)
		{
			lens_calib_data_t	*lens_calib_data = piezo->af_calib;

			lens_calib_data->pwm_freq = pwm_freq * 1000;
			lens_calib_data->af_calib_points[0].hallcode =
									(lens_regs.POSITION_MACRO_H << 8) |
									lens_regs.POSITION_MACRO_L;
            lens_calib_data->af_calib_points[1].hallcode =
                                    (lens_regs.POSITION_1M_H << 8) |
									lens_regs.POSITION_1M_L;
            lens_calib_data->af_calib_points[2].hallcode =
                                    (lens_regs.POSITION_2M_H << 8) |
									lens_regs.POSITION_2M_L;
            lens_calib_data->af_calib_points[3].hallcode =
									(lens_regs.POSITION_INFINITY_H << 8) |
									lens_regs.POSITION_INFINITY_L;
			lens_calib_data->hard_stop[0] = (lens_regs.HARD_STOP_MACRO_H << 8) |
									lens_regs.HARD_STOP_MACRO_L;
			lens_calib_data->hard_stop[1] =
									(lens_regs.HARD_STOP_INFINITY_H << 8) |
									lens_regs.HARD_STOP_INFINITY_L;
			lens_calib_data->polarity = lens_regs.POLARITY;
			lens_calib_data->sensitivity = lens_regs.SENSITIVITY;

            if (piezo->cam_info.grp == GRP_B)   /* CAM_TYPE_70MM */
            {
                lens_calib_data->af_calib_points[0].object_distance_mm = 400;
                lens_calib_data->af_calib_points[1].object_distance_mm = 1000;
                lens_calib_data->af_calib_points[2].object_distance_mm = 2000;
                lens_calib_data->af_calib_points[3].object_distance_mm = 12000;
                lens_calib_data->af_calib.focal_length_mm = 9.19f;
            }
            else
            {
                lens_calib_data->af_calib_points[0].object_distance_mm = 1000;
                lens_calib_data->af_calib_points[1].object_distance_mm = 2000;
                lens_calib_data->af_calib_points[2].object_distance_mm = 4000;
                lens_calib_data->af_calib_points[3].object_distance_mm = 54000;
                lens_calib_data->af_calib.focal_length_mm = 19.77f;
            }
            lens_calib_data->af_calib.num_calib_points = ACTUATOR_NUM_CALIB_POINTS;
            lens_calib_data->af_calib.calib_point_arr = lens_calib_data->af_calib_points;
		}
	}
	else
	{
		eeprom_af_mirr_reg_t	mirr_regs;

		eeprom_ret = eeprom_read(piezo->eeprom_i2c,
									EEPROM_AF_MIRR_OFFSET,
									sizeof(eeprom_af_mirr_reg_t),
									(void *)&mirr_regs);
		if (eeprom_ret == EEPROM_OK)
		{
			mirr_calib_data_t	*mirr_calib_data = piezo->af_calib;

			mirr_calib_data->pwm_freq = mirr_regs.FREQUENCY * 1000;
			mirr_calib_data->position_45d =
									(mirr_regs.POSITION_45DEGREE_H << 8) |
									mirr_regs.POSITION_45DEGREE_L;
			mirr_calib_data->hard_stop[0] = (mirr_regs.HARD_STOP_MACRO_H << 8) |
									mirr_regs.HARD_STOP_MACRO_L;
			mirr_calib_data->hard_stop[1] =
									(mirr_regs.HARD_STOP_INFINITY_H << 8) |
									mirr_regs.HARD_STOP_INFINITY_L;
			mirr_calib_data->polarity = mirr_regs.POLARITY;
			mirr_calib_data->sensitivity = mirr_regs.SENSITIVITY;
			pwm_freq = mirr_regs.FREQUENCY * 1000;
#ifdef EVT3_REWORK
			/* this patch is used for from EVT3 to later only */
			mirr_calib_data->hard_stop[0] =
					(mirr_regs.HARD_STOP_INFINITY_H << 8) |
					mirr_regs.HARD_STOP_INFINITY_L;
			mirr_calib_data->hard_stop[1] =
					(mirr_regs.HARD_STOP_MACRO_H << 8) |
					mirr_regs.HARD_STOP_MACRO_L;
#endif
		}
	}

	if (eeprom_ret == EEPROM_OK && hall_ret == HALL_OK)
	{
		/* print EEPROM calibration data */
		log_piezo_msg(piezo, SLOG_DEBUG, "Calibration data from EEPROM:");
		print_calib_info(piezo);

		/* check data validation */
		*data_valid = check_calib_info_validation(piezo);
	}
	else if (eeprom_ret == EEPROM_OK)
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Reading camera EEPROM timed out");
		piezo_ret = ACTUATOR_EEPROM_TIMED_OUT;
	}
	else
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Reading lens EEPROM timed out");
		piezo_ret = ACTUATOR_HALL_TIMED_OUT;
	}

	return piezo_ret;
}

/*
 * @brief log_msg
 * Prints message with actuator information
 */
static void log_piezo_msg(actuator_t *piezo, uint8_t header,
							const char *format, ...)
{
#if(LOG_VERBOSE == STD_ON)
	char		*msg;
	va_list		argptr;
	uint16_t	msg_size;
	char		msg_body[128];
	char		piezo_info[16] = "CAM-";
	char		cam_module[3];

	/* get actuator information */
	sprintf(cam_module, "%X", piezo->cam_info.module);
	strcat(piezo_info, cam_module);
	strcat(piezo_info, (piezo->type == ACTUATOR_LENS) ?
							"-Lens, " : "-Mirror, ");
	/* get message body */
	va_start(argptr, format);
	msg_size = strlen(piezo_info) + vsprintf(msg_body, format, argptr) + 1;
	va_end(argptr);
	/* print message */
	msg = pvPortMalloc(msg_size * sizeof(char));
	if (msg)
	{
		memcpy(msg, piezo_info, strlen(piezo_info) + 1);
		strcat(msg, msg_body);
		SLOGF(header, msg);
		vPortFree(msg);
	}
#endif
}

/*
 * @brief check_calib_info_validation
 * Checks the calibration info validation of independent piezo module
 */
static data_valid_t check_calib_info_validation(actuator_t *piezo)
{
	data_valid_t data_valid = ACTUATOR_DATA_OK;
	uint32_t pwm_freq;
	uint16_t hard_stop_macro, hard_stop_infinity;
	uint8_t sensitivity, polarity;

	if (piezo->type == ACTUATOR_LENS)
	{
		lens_calib_data_t *lens_calib_data = piezo->af_calib;

		pwm_freq = lens_calib_data->pwm_freq;
		hard_stop_macro = lens_calib_data->hard_stop[0];
		hard_stop_infinity = lens_calib_data->hard_stop[1];
		sensitivity = lens_calib_data->sensitivity;
		polarity = lens_calib_data->polarity;
	}
	else
	{
		mirr_calib_data_t *lens_calib_data = piezo->af_calib;

		pwm_freq = lens_calib_data->pwm_freq;
		hard_stop_macro = lens_calib_data->hard_stop[0];
		hard_stop_infinity = lens_calib_data->hard_stop[1];
		sensitivity = lens_calib_data->sensitivity;
		polarity = lens_calib_data->polarity;
	}

	/* check data validation */
	if (!IS_ACTUATOR_FREQ(pwm_freq))
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Invalid PWM frequency");
		data_valid |= ACTUATOR_INVALID_FREQ;
	}
	if ((polarity != 0) && (polarity != 1))
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Invalid hall polarity");
		data_valid |= ACTUATOR_INVALID_POLARITY;
	}
	if (sensitivity >= piezo->hall_obj->nbo_sensitivities)
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Invalid hall sensitivity");
		data_valid |= ACTUATOR_INVALID_SENSITIVITY;
	}
	if ((hard_stop_macro == 0x0000) ||
		(hard_stop_macro == 0xFFFF) ||
		(hard_stop_infinity == 0x0000) ||
		(hard_stop_infinity == 0xFFFF) ||
		(hard_stop_macro <
				hard_stop_infinity + ACTUATOR_MIN_STROKE))
	{
		log_piezo_msg(piezo, SLOG_ERROR, "Invalid hard stop positions");
		data_valid |= ACTUATOR_INVALID_HARD_STOP;
	}

	return data_valid;
}

/*
 * @brief print_calib_info
 * Prints the calibration info of independent piezo module
 */
static void print_calib_info(actuator_t *piezo)
{
#if(LOG_VERBOSE == STD_ON)
	uint32_t pwm_freq;
	uint16_t hard_stop_macro, hard_stop_infinity;
	uint8_t sensitivity, polarity;

	if (piezo->type == ACTUATOR_LENS)
	{
		lens_calib_data_t *lens_calib_data = piezo->af_calib;

		pwm_freq = lens_calib_data->pwm_freq;
		hard_stop_macro = lens_calib_data->hard_stop[0];
		hard_stop_infinity = lens_calib_data->hard_stop[1];
		sensitivity = lens_calib_data->sensitivity;
		polarity = lens_calib_data->polarity;
	}
	else
	{
		mirr_calib_data_t *lens_calib_data = piezo->af_calib;

		pwm_freq = lens_calib_data->pwm_freq;
		hard_stop_macro = lens_calib_data->hard_stop[0];
		hard_stop_infinity = lens_calib_data->hard_stop[1];
		sensitivity = lens_calib_data->sensitivity;
		polarity = lens_calib_data->polarity;
	}

	SLOGF(SLOG_INFO, "PWM frequency: %d", pwm_freq);
	if (piezo->type == ACTUATOR_LENS)
	{
        lens_calib_data_t *lens_calib_data = piezo->af_calib;
		for (uint8_t i = 0; i < ACTUATOR_NUM_CALIB_POINTS; i++)
			SLOGF(SLOG_INFO, "Position %d: 0x%04X", i + 1, lens_calib_data->af_calib_points[i].hallcode);
	}
	else
	{
        mirr_calib_data_t *lens_calib_data = piezo->af_calib;
		SLOGF(SLOG_INFO, "Position at 45 degree: 0x%04X", lens_calib_data->position_45d);
	}
	SLOGF(SLOG_INFO, "Macro hard stop: 0x%04X", hard_stop_macro);
	SLOGF(SLOG_INFO, "Infinity hard stop: 0x%04X", hard_stop_infinity);
	SLOGF(SLOG_INFO, "Polarity: %d", polarity);
	SLOGF(SLOG_INFO, "Sensitivity: %d", sensitivity);
#endif
}

/*
 * @brief convert_to_position
 * Converts from distance to position
 */
static uint16_t convert_to_position(lens_calib_data_t *calib_data,
									uint32_t distance)
{
    uint16_t d;
    if (distance > 65535)
        d = 65535;
    else
        d = (uint16_t)distance;
    return object_distance_to_hallcode(&calib_data->af_calib, d);
}

/*
 * @brief get_derived_pwm_duty
 * Computes the derived pwm duty cycle from actuator duty cycle
 */
static inline uint8_t get_derived_pwm_duty(uint8_t duty_cycle,
										actuator_dir_t direction)
{
	return (direction == ACTUATOR_DIR_EXTEND_WIDE) ?
								duty_cycle : (100 - duty_cycle);
}

/*
 * This function is called by the PWM hardware block's ISR when it
 * finishes generating a pattern.
 */
static void pwm_pattern_callback(void *cb_param)
{
	BaseType_t higher_pri_task_woken = pdFALSE;

	hal_pwm_stop(((actuator_t*)cb_param)->pwm);

	xSemaphoreGiveFromISR(((actuator_t*)cb_param)->pwm_semaphore,
				&higher_pri_task_woken);
	portEND_SWITCHING_ISR(higher_pri_task_woken);
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
