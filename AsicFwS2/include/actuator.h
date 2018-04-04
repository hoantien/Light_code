/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    actuator.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-1-2016
 * @brief   This file contains expand of the actuator driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "hall.h"
#include "light_system.h"

/* Exported typedef-----------------------------------------------------------*/
/*
 * @brief actuator_return_t
 *
 * PIEZO return type
 */
typedef enum
{
	ACTUATOR_OK = 0,
	ACTUATOR_UNINITIALIZED,
	ACTUATOR_NULL_PTR,
	ACTUATOR_INVALID_TYPE,
	ACTUATOR_INVALID_DIR,
	ACTUATOR_UNSUPPORTED_CAM_GROUP,
	ACTUATOR_EEPROM_TIMED_OUT,
	ACTUATOR_EEPROM_INVALID_DATA,
	ACTUATOR_HALL_TIMED_OUT,
	ACTUATOR_NOT_CALIBRATED,
	ACTUATOR_MOVE_TIMED_OUT
} actuator_return_t;

/*
 * @brief actuator_type_t
 *
 * Actuator type
 */
typedef enum
{
	ACTUATOR_LENS = 0,
	ACTUATOR_MIRROR
} actuator_type_t;

/*
 * @brief actuator_dir_t
 *
 * Movement direction
 */
typedef enum
{
	ACTUATOR_DIR_RETRACT_NARROW = 0,
	ACTUATOR_DIR_EXTEND_WIDE,
	ACTUATOR_NUM_DIR
} actuator_dir_t;

typedef struct
{
    float       weight_coarse;
    int16_t    extra_num_pulses;
} piezo_cl_per_dir_param_t;

typedef struct
{
    uint32_t pwm_freq;
    int8_t freq_search_step;
    actuator_dir_t last_move_dir;
    piezo_cl_per_dir_param_t per_dir_param[ACTUATOR_NUM_DIR];
} piezo_cl_param_t;

/*
 * @brief actuator_t
 * Optical system using for CAMERA GROUP B/C
 */
typedef struct
{
	/* Actuator interface */
	cam_t cam_info;			/* Camera information */
	actuator_type_t type;	/* Actuator type */
	i2c_t hall_i2c;			/* Hall sensor I2C channel */
	i2c_t eeprom_i2c;		/* EEPROM I2C channel */
	pwm_t pwm;				/* PWM channel to control PZT IC */
	EventGroupHandle_t event;	/* Event update from LCC command */
	SemaphoreHandle_t semaphore;	/* Semaphore to access settings */
	QueueHandle_t		queue;		/* Queue data for LCC command */
	void (*cb_func)(void *param);
	void *cb_param;
	bool moving;

	/* Internal data */
	void *af_calib;			/* Calibration data backup from eeprom */
	hall_t *hall_obj;		/* Hall object */
	bool is_initialized;
	bool is_calibrated;
	int16_t last_move_hall_code;
	SemaphoreHandle_t pwm_semaphore;	/* Semaphore to handle pwm  */
	piezo_cl_param_t piezo_param;
} actuator_t;

/* Exported functions --------------------------------------------------------*/

/*
 * @brief actuator_init
 * Initializes independent piezo module
 * @param piezo		Pointer to actuator_t structure
 * @return			Reference to the actuator_return_t structure
 */
actuator_return_t actuator_init(actuator_t *piezo);

/*
 * @brief actuator_calibrate
 * Calibrates independent piezo module
 * @param piezo		Pointer to actuator_t structure
 * @return			Reference to the actuator_return_t structure
 */
actuator_return_t actuator_calibrate(actuator_t *piezo);

/*
 * @brief actuator_move_to_position
 * Moves independent piezo module to a position
 * @param piezo				Pointer to actuator_t structure
 * @param target_position	The target position
 * @param tolerance			The tolerance of position
 * @return					Reference to the actuator_return_t structure
 */
actuator_return_t actuator_move_to_position(actuator_t *piezo,
											uint16_t *target_position,
											uint16_t tolerance);

/*
 * @brief actuator_move_to_distance
 * Focuses independent lens piezo module to a distance
 * @param piezo		Pointer to actuator_t structure
 * @param distance	The target distance
 * @return			Reference to the actuator_return_t structure
 */
actuator_return_t actuator_move_to_distance(actuator_t *piezo,
											uint32_t *distance);

/*
 * @brief actuator_nudge
 * Nudges independent piezo module in the desired direction
 * 		with a little burst of PWM.
 * @param piezo			Pointer to actuator_t structure
 * @param direction		The movement direction
 * @param multiplier	The number of PWM pulses
 * @param cb_func		The call-back function after moving done
 * @param cb_param		The call-back parameter passed to the call-back function
 * @return				Reference to the actuator_return_t structure
 */
actuator_return_t actuator_nudge(actuator_t *piezo, actuator_dir_t direction,
				uint16_t multiplier, void (*cb_func)(void *), void *cb_param);

/*
 * @brief actuator_get_hard_stop_hall
 * Returns piezo module lens hard stop hallcodes
 * @param piezo				Pointer to actuator_t structure
 * @param hall_min			The min hard stop position
 * @param hall_max			The max hard stop position
 * @return					Reference to the actuator_return_t structure
 */
actuator_return_t actuator_get_hard_stop_hall(actuator_t *piezo,
											uint16_t *hall_min,
											uint16_t *hall_max);

/*
 * @brief actuator_object_distance_to_lens_hallcode
 * Returns piezo module lens hallcode for given object distance
 * @param piezo                 Pointer to actuator_t structure
 * @param object_distance_mm    Focus distance in mm
 * @param hallcode              The hall code to focus at given distance
 * @return                  actuator_return_t structure
 */
actuator_return_t actuator_object_distance_to_lens_hallcode(actuator_t *piezo,
                uint32_t object_distance_mm, uint16_t *hallcode);

/*
 * @brief actuator_lens_hallcode_to_object_distance
 * Returns piezo module object focus distance for given lens hall code
 * @param piezo                 Pointer to actuator_t structure
 * @param hallcode              Lens position hall code
 * @param object_distance_mm    The object focus distance for given hall code
 * @return                  actuator_return_t structure
 */
actuator_return_t actuator_lens_hallcode_to_object_distance(actuator_t *piezo,
                uint16_t hallcode, uint32_t *object_distance_mm);

/*
 * @brief actuator_object_distance_to_lens_hallcode
 * Returns piezo module current hallcode
 * @param piezo                 Pointer to actuator_t structure
 * @param hallcode              The current hall code
 * @return                  actuator_return_t structure
 */
actuator_return_t actuator_read_position(actuator_t *piezo, uint16_t *hallcode);

#ifdef __cplusplus
}
#endif
#endif /* __ACTUATOR_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
