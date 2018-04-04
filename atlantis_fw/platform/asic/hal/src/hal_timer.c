/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    hal_timer.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jan-28-2016
 * @brief   This file contains expand of hal_timer
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "board_config.h"
#include "cortex_r4.h"
#include "hal_vic.h"
#include "hal_timer.h"

/* Private typedef -----------------------------------------------------------*/
/*
 * @brief timer_regs
 *
 * Timer registers structure
 */
typedef struct timer_reg
{
	__IO uint32_t	LCR;		/* Load Count Register */
	__I  uint32_t	CVR;		/* Current Value Register */
	__IO uint32_t	TCR;		/* Control Register */
	__I  uint32_t	EOIR;		/* End Of Interrupt Register */
	__I  uint32_t	ISR;		/* Interrupt Status Register */
} timer_regs;

/*
 * @brief timer_config_t
 *
 * Timer configuration structure
 */
typedef struct timer_config
{
	timer_regs		*handle;							/* Timer handle */
	void			(*callback_handler)(void *params);	/* Interrupt handler */
	void			*params;
} timer_config_t;


/* Private define ------------------------------------------------------------*/

/* Timer handles */
#define TIMER1		(((timer_regs *)TIMER_BASE) + 0)
#define TIMER2		(((timer_regs *)TIMER_BASE) + 1)
#define TIMER4		(((timer_regs *)TIMER_BASE) + 3)
#define TIMER3		(((timer_regs *)TIMER_BASE) + 2)
#define TIMER5		(((timer_regs *)TIMER_BASE) + 4)
#define TIMER6		(((timer_regs *)TIMER_BASE) + 5)
#define TIMER7		(((timer_regs *)TIMER_BASE) + 6)
#define TIMER8		(((timer_regs *)TIMER_BASE) + 7)

#define TIMER_INTR_STATUS	(TIMER_BASE + 0xA0)

/* Definition of bits in Timer Control Register */
#define TIM_TCR_INT_MASK	BIT1
#define TIM_TCR_ENABLE		BIT0

/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void hal_timer_irq(uint8_t chid);
static void timer_irq_handler(void);
/* Private variables ---------------------------------------------------------*/
/* Timer configurations */
static timer_config_t	timer_configs[HAL_TIM_MAX] =
{
	/* Timer 1 */
	{
		.handle = TIMER1,
		.callback_handler = NULL_PTR,
		.params = NULL_PTR
	},
	/* Timer 2 */
	{
		.handle = TIMER2,
		.callback_handler = NULL_PTR,
		.params = NULL_PTR
	},
	/* Timer 3 */
	{
		.handle = TIMER3,
		.callback_handler = NULL_PTR,
		.params = NULL_PTR
	},
	/* Timer 4 */
	{
		.handle = TIMER4,
		.callback_handler = NULL_PTR,
		.params = NULL_PTR
	},
	/* Timer 5 */
	{
		.handle = TIMER5,
		.callback_handler = NULL_PTR,
		.params = NULL_PTR
	},
	/* Timer 6 */
	{
		.handle = TIMER6,
		.callback_handler = NULL_PTR,
		.params = NULL_PTR
	},
	/* Timer 7 */
	{
		.handle = TIMER7,
		.callback_handler = NULL_PTR,
		.params = NULL_PTR
	},
	/* Timer 8 */
	{
		.handle = TIMER8,
		.callback_handler = NULL_PTR,
		.params = NULL_PTR
	}
};

static uint8_t intr_int_flag;
/* Exported functions --------------------------------------------------------*/

/*
 * hal_timer_init
 * Initialize timer module
 */
hal_timer_return_t hal_timer_init(hal_timer_t *tim)
{
	hal_timer_return_t ret = HAL_TIM_OK;

	/* Check null pointer */
	if(NULL_PTR == tim)
	{
		ret = HAL_TIM_NULL_PTR;
	}
	else if((HAL_TIM_MAX <= tim->chid) || (HAL_TIM_CH1 > tim->chid))
	{
		ret = HAL_TIM_INVALID_CHANNEL;
	}
	else
	{
		timer_regs *timer = timer_configs[tim->chid].handle;

		/* Disable the selected timer */
		timer->TCR &= (~TIM_TCR_ENABLE);
		/* Enable interrupt for the selected timer */
		timer->TCR |= TIM_TCR_INT_MASK;
		/* Set interrupt handler */
		timer_configs[tim->chid].callback_handler = tim->callback_handler;
		/* Set the timer period */
		timer->LCR = tim->period * (CLOCK_133MHZ / 1000000);
		if(!intr_int_flag)
		{
			vic_register_irq(TIMER_IRQn, timer_irq_handler);
			vic_set_priority_irq(TIMER_IRQn, 15);
			intr_int_flag = 1;
		}
		ret = HAL_TIM_OK;
	}
	return ret;
}

/*
 * hal_timer_update_period
 * Update timer period
 */
hal_timer_return_t hal_timer_update_period(hal_timer_t *tim)
{
	hal_timer_return_t ret = HAL_TIM_OK;

	/* Check null pointer */
	if (NULL_PTR == tim)
	{
		ret = HAL_TIM_NULL_PTR;
	}
	else if ((HAL_TIM_MAX <= tim->chid) || (HAL_TIM_CH1 > tim->chid))
	{
		ret = HAL_TIM_INVALID_CHANNEL;
	}
	else
	{
		timer_regs		*timer = timer_configs[tim->chid].handle;

		/* Set the timer period */
		timer->LCR = tim->period * (CLOCK_133MHZ / 1000000);

		ret = HAL_TIM_OK;
	}
	return ret;
}

/*
 * hal_timer_start
 * Enable timer
 */
hal_timer_return_t hal_timer_start(hal_timer_channel_t chid)
{
	hal_timer_return_t ret = HAL_TIM_OK;

	if ((HAL_TIM_MAX <= chid) || (HAL_TIM_CH1 > chid))
	{
		ret = HAL_TIM_INVALID_CHANNEL;
	}
	else
	{
		timer_regs *timer = timer_configs[chid].handle;

		/* Enable the selected timer */
		timer->TCR |= TIM_TCR_ENABLE;

		ret = HAL_TIM_OK;
	}
	return ret;
}

/*
 * hal_timer_stop
 * Disable timer
 */
hal_timer_return_t hal_timer_stop(hal_timer_channel_t chid)
{
	hal_timer_return_t ret = HAL_TIM_OK;

	if ((HAL_TIM_MAX <= chid) || (HAL_TIM_CH1 > chid))
	{
		ret = HAL_TIM_INVALID_CHANNEL;
	}
	else
	{
		timer_regs *timer = timer_configs[chid].handle;

		/* Disable the selected timer */
		timer->TCR &= (~TIM_TCR_ENABLE);

		ret = HAL_TIM_OK;
	}
	return ret;
}

/*
 * hal_timer_read_counter_reg
 * Read value from counter register of Timer
 */
unsigned int hal_timer_read_counter_reg(hal_timer_channel_t chid)
{
	timer_regs *timer = timer_configs[chid].handle;
	return (unsigned int)(timer->CVR);
}

/*
 * hal_timer_set_period
 * Read value from counter register of Timer
 */
void hal_timer_set_period(hal_timer_channel_t chid, unsigned int value)
{
	timer_regs *timer = timer_configs[chid].handle;
	/* Set the timer period */
	timer->LCR = value;
}

/*
 * timer_irq_handler
 * @TODO: should be invoked in an interrupt vector table file
 */
static void timer_irq_handler(void)
{
	register uint8_t intr_n = readl(TIMER_INTR_STATUS);
	for(uint8_t i = 0; i < HAL_TIM_MAX; i++)
	{
		if(intr_n & (1 << i))
		{
			hal_timer_irq(i);
		}
	}
}
/* Private functions ---------------------------------------------------------*/

/**
 * hal_timer_irq
 * Process timer irq
 */
static void hal_timer_irq(uint8_t chid)
{
	register uint8_t	dummy;
	timer_regs		*timer = timer_configs[chid].handle;
	void			*params = timer_configs[chid].params;
	if(NULL_PTR != timer_configs[chid].callback_handler)
	{
		/* Call back function to upper layer */
		(*(timer_configs[chid].callback_handler))(params);
	}
	/* Dummy read the end-of-interrupt register to clear interrupt pending */
	dummy = timer->EOIR;
	(void)dummy;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
