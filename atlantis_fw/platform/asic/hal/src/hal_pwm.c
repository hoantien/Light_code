/*******************************************************************************
 * Copyright (c) 2015,The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file	hal_pwm.c
 * @author  The LightCo
 * @version V1.0.0
 * @date	Nov-04-2015
 * @brief   This file contains expand of hal_pwm
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "assert.h"
#include "board_config.h"
#include "cortex_r4.h"
#include "hal_vic.h"
#include "hal_pwm.h"


/** #ASIC Hardware target */

/* Private define ------------------------------------------------------------*/

#define PWM0_CH1_BASE			(PWM0_BASE + 0x000)
#define PWM0_CH2_BASE			(PWM0_BASE + 0x100)
#define PWM0_CH3_BASE			(PWM0_BASE + 0x200)
#define PWM0_CH4_BASE			(PWM0_BASE + 0x300)
#define PWM0_CH5_BASE			(PWM0_BASE + 0x400)
#define PWM0_CH6_BASE			(PWM0_BASE + 0x500)

#define PWM1_CH1_BASE			(PWM1_BASE + 0x000)
#define PWM1_CH2_BASE			(PWM1_BASE + 0x100)
#define PWM1_CH3_BASE			(PWM1_BASE + 0x200)
#define PWM1_CH4_BASE			(PWM1_BASE + 0x300)
#define PWM1_CH5_BASE			(PWM1_BASE + 0x400)
#define PWM1_CH6_BASE			(PWM1_BASE + 0x500)

/**
 * @brief PWM address
 */
#define PWM0_CH1_ADDR			((pwm_reg_t *)(uint32_t)(PWM0_CH1_BASE))
#define PWM0_CH2_ADDR			((pwm_reg_t *)(uint32_t)(PWM0_CH2_BASE))
#define PWM0_CH3_ADDR			((pwm_reg_t *)(uint32_t)(PWM0_CH3_BASE))
#define PWM0_CH4_ADDR			((pwm_reg_t *)(uint32_t)(PWM0_CH4_BASE))
#define PWM0_CH5_ADDR			((pwm_reg_t *)(uint32_t)(PWM0_CH5_BASE))
#define PWM0_CH6_ADDR			((pwm_reg_t *)(uint32_t)(PWM0_CH6_BASE))
#define PWM1_CH1_ADDR			((pwm_reg_t *)(uint32_t)(PWM1_CH1_BASE))
#define PWM1_CH2_ADDR			((pwm_reg_t *)(uint32_t)(PWM1_CH2_BASE))
#define PWM1_CH3_ADDR			((pwm_reg_t *)(uint32_t)(PWM1_CH3_BASE))
#define PWM1_CH4_ADDR			((pwm_reg_t *)(uint32_t)(PWM1_CH4_BASE))
#define PWM1_CH5_ADDR			((pwm_reg_t *)(uint32_t)(PWM1_CH5_BASE))
#define PWM1_CH6_ADDR			((pwm_reg_t *)(uint32_t)(PWM1_CH6_BASE))

/* Definition of bits in Mode Register */
#define PWM_MODE_PWMMODE_MASK			0x00000003
#define PWM_MODE_CONTMMODEEN_MASK		0x00000010
#define PWM_MODE_PATTERNMODEEN_MASK		0x00000100
#define PWM_MODE_GPIO_CTRL_OFFSET		20

#define IS_PWM_CHANNEL(CHANNEL)		(((CHANNEL) >= PWM0_CH1) &&	\
									((CHANNEL) <= PWM1_CH6))

#define IS_PWM_ALIGN_MODE(MODE)		((MODE == PWM_ALIGN_LEFT) || \
									(MODE == PWM_ALIGN_RIGHT) ||\
									(MODE == PWM_ALIGN_CENTER))

#define IS_PWM_OUT_GEN_MODE(MODE)	((MODE == PWM_GPIO)	|| \
									(MODE == PWM_CONTINUOUS)	|| \
									(MODE == PWM_FIXED_PATTERNS))

#define IS_PWM_OUT_FREQ(FREQ)		((0 < FREQ) && (BOARD_PCLOCK >= FREQ))

#define IS_PWM_NUM_PAIRS(PAIRS)		(4 > PAIRS)

#define IS_PWM_DUTY_CYCLE(DUTY)		(101 > DUTY)

#define IS_PWM_IRQ_MODE(OUTMODE, IRQMODE)	\
							((IRQMODE == PWM_IRQ_ONE_PATTERN_GENERATED) || \
							((IRQMODE == PWM_IRQ_ALL_PATTERNS_GENERATED) && \
							(OUTMODE == PWM_FIXED_PATTERNS)))

/* Parameter initialization for PWM as defaults. Input PWM port and channel */
#define PWM_INIT_DEFAULTS(p, n)		{ \
										.pwmx = PWM##p##_CH##n##_ADDR, \
										.align = PWM_ALIGN_LEFT, \
										.mode = PWM_CONTINUOUS, \
										.state = PWM_STOPPED, \
										.is_irq_enabled = FALSE, \
										.callback_handler = NULL \
									}
/* Private typedef -----------------------------------------------------------*/

/**
 * @brief pwm_reg_sub_t
 */
typedef struct pwm_reg_sub
{
	__IO uint32_t DUTYV;			/* Duty-cycle Value Register */
	__IO uint32_t NUMPV;			/* Number of Pulses Register */
} pwm_reg_sub_t;

/**
 * @brief pwm_reg_t
 * PWM registers structure
 */
typedef struct pwm_reg
{
	pwm_reg_sub_t	PAIRS[3];	/* Pairs */
	__IO uint32_t	NUMPATS;	/* Number of Patterns Register */
	__IO uint32_t	ARLDV;		/* Auto Reload Register */
	__IO uint32_t	MODE;		/* Mode Register */
	__IO uint32_t	INTRSTAT;	/* Interrupt Status Register */
	__IO uint32_t	INTREN;		/* Interrupt Enable Register */
	__IO uint32_t	STATUS;		/* Status Register */
} pwm_reg_t;

/**
 * @brief pwm_state_t
 * PWM state machine
 */
typedef enum pwm_state
{
	PWM_STOPPED = 0,
	PWM_STARTED
} pwm_state_t;

typedef struct pwm_dev
{
	pwm_reg_t	*pwmx;
	pwm_align_t	align;
	pwm_mode_t	mode;
	pwm_state_t	state;
	bool		is_irq_enabled;
	void		(*callback_handler)(void *param);
	void		*cb_param;
} pwm_dev_t;

/* Private functions prototypes ----------------------------------------------*/

/* interrupt handlers */
static void hal_pwm_isr(int chid);
static void PWM0_IRQHandler(void);
static void PWM1_IRQHandler(void);

/* Private variables ---------------------------------------------------------*/

static pwm_dev_t hw_pwm[] =
{
	PWM_INIT_DEFAULTS(0, 1),
	PWM_INIT_DEFAULTS(0, 2),
	PWM_INIT_DEFAULTS(0, 3),
	PWM_INIT_DEFAULTS(0, 4),
	PWM_INIT_DEFAULTS(0, 5),
	PWM_INIT_DEFAULTS(0, 6),
	PWM_INIT_DEFAULTS(1, 1),
	PWM_INIT_DEFAULTS(1, 2),
	PWM_INIT_DEFAULTS(1, 3),
	PWM_INIT_DEFAULTS(1, 4),
	PWM_INIT_DEFAULTS(1, 5),
	PWM_INIT_DEFAULTS(1, 6)
};

/* Exported functions --------------------------------------------------------*/

/*
 * hal_pwm_init
 * The function initializes independent pwm channel
 */
void hal_pwm_init(hal_pwm_channel_t pwm_channel,
				hal_pwm_init_t *pwm_init_config, hal_pwm_output_t *pwm_value)
{
	pwm_reg_t		*pwmx;

	/* Check parameters */
	assert_param(NULL != pwm_init_config);
	assert_param(IS_PWM_ALIGN_MODE(pwm_init_config->align_mode));
	assert_param(IS_PWM_OUT_GEN_MODE(pwm_init_config->output_gen_mode));
	assert_param(IS_PWM_CHANNEL(pwm_channel));

	/* Get PWM channel base address */
	pwmx = hw_pwm[pwm_channel].pwmx;

	/* Set alignment mode */
	pwmx->MODE = (pwmx->MODE & (~PWM_MODE_PWMMODE_MASK)) |
					pwm_init_config->align_mode;
	/* Store align mode */
	hw_pwm[pwm_channel].align = pwm_init_config->align_mode;
	/* Store output generation mode */
	hw_pwm[pwm_channel].mode = pwm_init_config->output_gen_mode;
	/* Set state machine to STOPPED */
	hw_pwm[pwm_channel].state = PWM_STOPPED;
	/* Initialize the output values */
	hal_pwm_update(pwm_channel, pwm_value);
}

/*
 * hal_pwm_start
 * The function starts independent pwm channel
 */
void hal_pwm_start(hal_pwm_channel_t pwm_channel)
{
	pwm_reg_t		*pwmx;

	/* Check parameters */
	assert_param(IS_PWM_CHANNEL(pwm_channel));

	/* Get PWM channel base address */
	pwmx = hw_pwm[pwm_channel].pwmx;

	/* Update state machine */
	hw_pwm[pwm_channel].state = PWM_STARTED;

	/* Set output generation mode and start generating PWM output signal */
	if (PWM_CONTINUOUS == hw_pwm[pwm_channel].mode)
	{
		/* Disable number of patterns mode */
		pwmx->MODE &= ~PWM_MODE_PATTERNMODEEN_MASK;
		/* Enable continuous mode */
		pwmx->MODE |= PWM_MODE_CONTMMODEEN_MASK;
	}
	else if (PWM_FIXED_PATTERNS == hw_pwm[pwm_channel].mode)
	{
		/* Disable continuous mode */
		pwmx->MODE &= ~PWM_MODE_CONTMMODEEN_MASK;
		/* Enable number of patterns mode */
		pwmx->MODE |= PWM_MODE_PATTERNMODEEN_MASK;
	}
	else
	{
		/* Disable continuous mode */
		pwmx->MODE &= ~PWM_MODE_CONTMMODEEN_MASK;
		/* Enable number of patterns mode */
		pwmx->MODE &= ~PWM_MODE_PATTERNMODEEN_MASK;
	}
}

/*
 * hal_pwm_stop
 * The function stops independent pwm channel
 */
void hal_pwm_stop(hal_pwm_channel_t pwm_channel)
{
	/* Check parameters */
	assert_param(IS_PWM_CHANNEL(pwm_channel));

	/* Stop generating PWM output signal */
	hw_pwm[pwm_channel].pwmx->MODE &= (~PWM_MODE_CONTMMODEEN_MASK);
	hw_pwm[pwm_channel].pwmx->MODE &= (~PWM_MODE_PATTERNMODEEN_MASK);

	/* Update state machine */
	hw_pwm[pwm_channel].state = PWM_STOPPED;
}

/**
 * hal_pwm_update
 * The function updates period, duty cycle and num pulses for pwm channel
 */
void hal_pwm_update(hal_pwm_channel_t pwm_channel, hal_pwm_output_t *pwm_value)
{
	uint32_t period;
	uint32_t duty_cycle;
	uint8_t i;

	/* Check parameters */
	assert_param(IS_PWM_CHANNEL(pwm_channel));
	assert_param(NULL != pwm_value);
	assert_param(IS_PWM_OUT_FREQ(pwm_value->freq));
	assert_param(IS_PWM_NUM_PAIRS(pwm_value->pattern.num_pairs));
	for (i = 0; i < pwm_value->pattern.num_pairs; i++)
	{
		assert_param(0 < pwm_value->pattern.pair[i].num_pulses);
		assert_param(IS_PWM_DUTY_CYCLE(pwm_value->pattern.pair[i].duty));
	}

	/* Configure frequency */
	period = CLOCK_133MHZ / pwm_value->freq;
	if (PWM_ALIGN_CENTER != hw_pwm[pwm_channel].align)
		hw_pwm[pwm_channel].pwmx->ARLDV = period - 1;
	else
		hw_pwm[pwm_channel].pwmx->ARLDV = period >> 1;

	/* Configure patterns */
	for (i = 0; i < pwm_value->pattern.num_pairs; i++)
	{
		/* Duty cycle for pairs */
		duty_cycle = (pwm_value->pattern.pair[i].duty * period) / 100;
		if (PWM_ALIGN_CENTER != hw_pwm[pwm_channel].align)
			hw_pwm[pwm_channel].pwmx->PAIRS[i].DUTYV = duty_cycle;
		else
			hw_pwm[pwm_channel].pwmx->PAIRS[i].DUTYV = duty_cycle >> 1;
		/* The number of pulses for pairs */
		hw_pwm[pwm_channel].pwmx->PAIRS[i].NUMPV =
										pwm_value->pattern.pair[i].num_pulses;
	}
	/* Disable the unused pairs */
	for (; i < 3; i++)
		hw_pwm[pwm_channel].pwmx->PAIRS[i].NUMPV = 0;

	/*
	 * Set the number of patterns
	 * corresponding to the configured output generation mode
	 */
	hw_pwm[pwm_channel].pwmx->NUMPATS =
				(hw_pwm[pwm_channel].mode == PWM_FIXED_PATTERNS) ?
										pwm_value->num_patterns : 0;

	/* Take effect if PWM channel is running */
	if (PWM_STARTED == hw_pwm[pwm_channel].state)
		hal_pwm_start(pwm_channel);
}

/**
 * @brief hal_pwm_enable_irq
 * The function enables interrupt of independent pwm channel
 */
void hal_pwm_enable_irq(hal_pwm_channel_t pwm_channel,
						hal_pwm_irq_t *pwm_intr_config)
{
	/* Check parameters */
	assert_param(IS_PWM_CHANNEL(pwm_channel));
	assert_param(NULL != pwm_intr_config);
    //printf("hw_pwm[pwm_channel].mode: %d, pwm_intr_config->irq_mode: %d\n", hw_pwm[pwm_channel].mode, pwm_intr_config->irq_mode)
	assert_param(IS_PWM_IRQ_MODE(hw_pwm[pwm_channel].mode,
					pwm_intr_config->irq_mode));
	assert_param(NULL != pwm_intr_config->callback_handler);

	/* Set interrupt handler */
	hw_pwm[pwm_channel].callback_handler = pwm_intr_config->callback_handler;
	hw_pwm[pwm_channel].cb_param = pwm_intr_config->cb_param;
	hw_pwm[pwm_channel].is_irq_enabled = TRUE;
	/* Enable interrupt */
	hw_pwm[pwm_channel].pwmx->INTREN = pwm_intr_config->irq_mode;
	/* register interrupt handler */
	if ((PWM0_CH1 <= pwm_channel) && (PWM0_CH6 >= pwm_channel))
		vic_register_irq(PWM0_IRQn, PWM0_IRQHandler);
	else
		vic_register_irq(PWM1_IRQn, PWM1_IRQHandler);
}

/**
 * @brief hal_pwm_disable_irq
 * The function disables interrupt of independent pwm channel
 */
void hal_pwm_disable_irq(hal_pwm_channel_t pwm_channel)
{
	uint8_t		i;

	/* Check parameters */
	assert_param(IS_PWM_CHANNEL(pwm_channel));

	/* Disable all interrupts */
	hw_pwm[pwm_channel].pwmx->INTREN = 0;
	/* clear all pending interrupts */
	hw_pwm[pwm_channel].pwmx->INTRSTAT = 0xFFFFFFFF;
	/* clear interrupt handler */
	hw_pwm[pwm_channel].callback_handler = NULL;
	hw_pwm[pwm_channel].is_irq_enabled = FALSE;
	/* unregister interrupt handler */
	if ((PWM0_CH1 <= pwm_channel) && (PWM0_CH6 >= pwm_channel))
	{
		for (i = PWM0_CH1; i <= PWM0_CH6; i++)
			if (TRUE == hw_pwm[i].is_irq_enabled)
				break;
		if ((PWM0_CH6 + 1) == i)
			vic_unregister_irq(PWM0_IRQn);
	}
	else
	{
		for (i = PWM1_CH1; i <= PWM1_CH6; i++)
			if (TRUE == hw_pwm[i].is_irq_enabled)
				break;
		if ((PWM1_CH6 + 1) == i)
			vic_unregister_irq(PWM1_IRQn);
	}
}

/**
 * @brief hal_pwm_set_high
 * The function set high for the pins hardware trigger
 */
void hal_pwm_set_high(hal_pwm_channel_t pwm_channel, hal_pwm_pin_t pin)
{
	pwm_reg_t	*pwmx;

	/* Check parameters */
	assert_param(IS_PWM_CHANNEL(pwm_channel));
	/* Get PWM channel base address */
	pwmx = hw_pwm[pwm_channel].pwmx;

	hal_pwm_stop(pwm_channel);
	/* set pwm_p output value*/
	pwmx->MODE |= 1<<(PWM_MODE_GPIO_CTRL_OFFSET + pin);
}

/**
 * @brief hal_pwm_set_low
 * The function set low for the pins hardware trigger
 */
void hal_pwm_set_low(hal_pwm_channel_t pwm_channel, hal_pwm_pin_t pin)
{
	pwm_reg_t	*pwmx;

	/* Check parameters */
	assert_param(IS_PWM_CHANNEL(pwm_channel));
	/* Get PWM channel base address */
	pwmx = hw_pwm[pwm_channel].pwmx;

	hal_pwm_stop(pwm_channel);
	/* set pwm_n output value*/
	pwmx->MODE &= ~(1<<(PWM_MODE_GPIO_CTRL_OFFSET + pin));
}

/* Private functions --------- ----------------------------------------------*/

/*
 * @brief hal_pwm_isr
 * Internal PWM interrupt service routine
 */
static void hal_pwm_isr(int chid)
{
	pwm_reg_t	*pwmx = hw_pwm[chid].pwmx;

	/* check if ONE_PATTERN_GENERATED interrupt occurred */
	if (PWM_IRQ_ONE_PATTERN_GENERATED ==
							(pwmx->INTRSTAT & PWM_IRQ_ONE_PATTERN_GENERATED))
	{
		/* Clear pending interrupt */
		pwmx->INTRSTAT |= PWM_IRQ_ONE_PATTERN_GENERATED;
	}
	/* if ALL_PATTERNS_GENERATED interrupt occurred */
	else
	{
		/* Clear pending interrupt */
		pwmx->INTRSTAT |= PWM_IRQ_ALL_PATTERNS_GENERATED;
	}
	/* call back to upper layer function */
	hw_pwm[chid].callback_handler(hw_pwm[chid].cb_param);
}

/*
 * @brief PWM0_IRQHandler
 * Internal PWM0 interrupt handler
 */
void PWM0_IRQHandler(void)
{
	int		i;

	for (i = PWM0_CH1; i <= PWM0_CH6; i++)
	{
		if (0 != hw_pwm[i].pwmx->INTRSTAT)
			hal_pwm_isr(i);
	}
}

/*
 * @brief PWM1_IRQHandler
 * Internal PWM1 interrupt handler
 */
void PWM1_IRQHandler(void)
{
	int		i;

	for (i = PWM1_CH1; i <= PWM1_CH6; i++)
	{
		if (0 != hw_pwm[i].pwmx->INTRSTAT)
			hal_pwm_isr(i);
	}
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd. ********END OF FILE*******/
