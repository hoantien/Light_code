/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file	hal_pwm.h
 * @author  The LightCo
 * @version V1.0.0
 * @date	Nov-04-2015
 * @brief   This file contains definitions of the PWM driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_PWM_H__
#define __HAL_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
/* Exported typedef ----------------------------------------------------------*/

/**
 * @brief hal_pwm_channel_t
 * PWM channel enumeration
 */
typedef enum hal_pwm_channel
{
	PWM0_CH1 = 0x00,
	PWM0_CH2,
	PWM0_CH3,
	PWM0_CH4,
	PWM0_CH5,
	PWM0_CH6,
	PWM1_CH1,
	PWM1_CH2,
	PWM1_CH3,
	PWM1_CH4,
	PWM1_CH5,
	PWM1_CH6
} hal_pwm_channel_t;

/**
 * @brief hal_pwm_channel_t
 * PWM channel enumeration
 */
typedef enum hal_pwm_pin
{
	PWM_PIN_POS = 0,
	PWM_PIN_NEG
} hal_pwm_pin_t;

/**
 * @brief pwm_align_t
 * PWM alignment mode
 *	00 : Edge Aligned mode (Up count) Selected
 *	01 : Edge Aligned mode (Down count) Selected
 *	10 : Center Aligned mode (Up/Down count) Selected
 *	11 : RESERVED (No Mode Selected)
 */
typedef enum pwm_align
{
	PWM_ALIGN_LEFT = 0x00,
	PWM_ALIGN_RIGHT,
	PWM_ALIGN_CENTER
} pwm_align_t;

/**
 * @brief pwm_mode_t
 * PWM output generation mode
 *	0: PWM continuous mode is selected.
 *	1: PWM number of patterns mode is selected.
 */
typedef enum pwm_mode
{
	PWM_GPIO = 0,
	PWM_CONTINUOUS,
	PWM_FIXED_PATTERNS
} pwm_mode_t;

/**
 * @brief hal_pwm_pair_t
 * PWM pair structure
 *	duty: duty cycle of PWM output pulses in a pair
 *	num_pulses: the number of pulses in a pair
 */
typedef struct hal_pwm_pair
{
	uint8_t		duty;
	uint32_t	num_pulses;
} hal_pwm_pair_t;

/**
 * @brief hal_pwm_pattern_t
 * PWM pattern structure
 *	pair: reference to hal_pwm_pair_t structure
 *	num_pairs: the number of pair in a pattern
 */
typedef struct hal_pwm_pattern
{
	hal_pwm_pair_t	pair[3];
	uint8_t			num_pairs;
} hal_pwm_pattern_t;

/**
 * @brief hal_pwm_output_t
 * PWM output structure
 *	freq: PWM output signal frequency
 *	pattern: reference to hal_pwm_pattern_t structure
 *	num_patterns:
 *		In the PWM_CONTINUOUS mode, this field is ignored.
 *		In the PWM_FIXED_PATTERNS mode, this field specifies
 *			the number of PWM patterns
 */
typedef struct hal_pwm_output
{
	hal_pwm_pattern_t	pattern;
	uint32_t			freq;
	uint32_t			num_patterns;
} hal_pwm_output_t;

/**
 * @brief hal_pwm_init_t
 * PWM operation configuration structure
 */
typedef struct hal_pwm_init
{
	pwm_align_t	align_mode;
	pwm_mode_t	output_gen_mode;
} hal_pwm_init_t;

/**
 * @brief hal_pwm_irq_mode_t
 * Selects the condition to generate interrupt
 *	PWM_IRQ_ONE_PATTERN_GENERATED:
 *		Interrupt occurs every when one pattern is generated
 *	PWM_IRQ_ALL_PATTERNS_GENERATED:
 *		This is only used in the PWM_FIXED_PATTERNS mode. Interrupt
 *			occurs when all patterns are generated.
 */
typedef enum hal_pwm_irq_mode
{
	PWM_IRQ_ONE_PATTERN_GENERATED	= (1 << 8),
	PWM_IRQ_ALL_PATTERNS_GENERATED	= (1 << 12)
} hal_pwm_irq_mode_t;

/**
 * @brief hal_pwm_irq_t
 * PWM interrupt configuration structure
 */
typedef struct hal_pwm_irq
{
	hal_pwm_irq_mode_t		irq_mode;
	void					(*callback_handler)(void *param);
	void					*cb_param;
} hal_pwm_irq_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief hal_pwm_init
 * The function initializes independent pwm channel
 * @param pwm_channel: PWM channel
 * @param pwm_init_config: points to hal_pwm_init_t structure
 * @param pwm_value: points to hal_pwm_output_t structure
 * @return none
 */
void hal_pwm_init(hal_pwm_channel_t pwm_channel,
				hal_pwm_init_t *pwm_init_config, hal_pwm_output_t *pwm_value);

/**
 * @brief hal_pwm_start
 * The function starts independent pwm channel
 * @param pwm_channel: PWM channel
 * @return none
 */
void hal_pwm_start(hal_pwm_channel_t pwm_channel);

/**
 * @brief hal_pwm_stop
 * The function stops independent pwm channel
 * @param pwm_channel: PWM channel
 * @return none
 */
void hal_pwm_stop(hal_pwm_channel_t pwm_channel);

/**
 * @brief hal_pwm_enable_irq
 * The function enables interrupt of independent pwm channel
 * @param pwm_channel: PWM channel
 * @param pwm_intr_config: points to hal_pwm_irq_t structure
 * @return none
 */
void hal_pwm_enable_irq(hal_pwm_channel_t pwm_channel,
						hal_pwm_irq_t *pwm_intr_config);

/**
 * @brief hal_pwm_disable_irq
 * The function disables interrupt of independent pwm channel
 * @param pwm_channel: PWM channel
 * @return none
 */
void hal_pwm_disable_irq(hal_pwm_channel_t pwm_channel);

/**
 * @brief hal_pwm_update
 * The function update period, duty cycle and num pulses for pwm channel
 * @param	pwm_channel: PWM channel
 * @param	pwm_value: point to hal_pwm_output_t structure
 * @return none
 */
void hal_pwm_update(hal_pwm_channel_t pwm_channel, hal_pwm_output_t *pwm_value);

/**
 * @brief hal_pwm_set_high
 * The function set pwm pins to high
 * @param	pwm_channel: PWM channel
 * @return none
 * @note	This is the solution for HW_Trigger output
 */
void hal_pwm_set_high(hal_pwm_channel_t pwm_channel, hal_pwm_pin_t pin);

/**
 * @brief hal_pwm_set_low
 * The function set pwm pins to low
 * @param	pwm_channel: PWM channel
 * @return none
 * @note	This is the solution for HW_Trigger output
 */
void hal_pwm_set_low(hal_pwm_channel_t pwm_channel, hal_pwm_pin_t pin);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_PWM_H__ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd. ********END OF FILE*******/

