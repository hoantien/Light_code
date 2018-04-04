/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_syncio.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-02-2016
 * @brief   This file provides firmware functions to manage the following
 *          functionalities of the SyncIO
 *           + Initialization and Configuration
 *           + Interrupts, events and flags management
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "hal_syncio.h"
#include "assert.h"
#include "cortex_r4.h"
#include "hal_vic.h"
#include "stdbool.h"

/* Private typedef -----------------------------------------------------------*/

#define SG_GEN_DIS			0x00000000	/*!< Pulse Generator Disable */
#define SG_GEN_EN			0x00001000	/*!< Pulse Generator Enable */
#define SG_SW_TRIGGER		0x00000001	/*!< Pulse Generator Software Trigger */
/**
  * @brief SyncIO Generator individual properties
  */
typedef struct
{
	__IO uint32_t LAT1;			/*!< Output 1 latency of pulse generator */
	__IO uint32_t LAT2;			/*!< Output 2 latency of pulse generator */
	__IO uint32_t WIDTH1;		/*!< Output 1 pulse width of pulse generator*/
	__IO uint32_t WIDTH2;		/*!< Output 2 pulse width of pulse generator*/
	__IO uint32_t CTRL;			/*!< Sync generator control register */
	__I  uint32_t COUNTER;		/*!< Counter of pulse generator*/
	__IO uint32_t RESERVED[2];	/*!< Reserved */
} syncio_chnl_reg_t;

/**
  * @brief SyncIO Generator registers
  */
typedef struct
{
	syncio_chnl_reg_t	SG[SG_CHANNEL_MAX];	/*!< SyncIO channels */
	__I  uint32_t		INT_STATE;			/*!< Interrupt state of SyncIO */
	__IO uint32_t		INT_MASK;			/*!< Interrupt mask of SyncIO */
	__O  uint32_t		INT_CLR;			/*!< Interrupt clear register */
	__O  uint32_t		GBL_CTRL;			/*!< Global control register */
} syncio_reg_t;

/*
 * @brief syncio_irq_cfg_t
 *
 * SyncIO interrupt configuration structure
 */
typedef struct syncio_irq_cfg
{
	void			(*callback_handler)(hal_syncio_channel_t chid,
										hal_syncio_irq_mode_t irq_status);
	bool			irq_is_enabled;
} syncio_irq_cfg_t;

/* Private definition --------------------------------------------------------*/
/**
 * SYNCIO: contains base address of SyncIO Pulse Generator
 */
#define SYNCIO			((syncio_reg_t *)PPG_BASE)

/**
 * CHANNEL(): bit mask for channel is used to configure registers
 */
#define CHANNEL(x)		(1 << x)

/**
 * PULSE_DONE(): bit mask for pulse done interrupt status.
 */
#define PULSE_DONE(x)	(1 << x)

/**
 * PULSE_DONE(): bit mask for repeat done interrupt status.
 */
#define REPEAT_DONE(x)	(1 << (x + 6))

/**
 * FALLING_EDGE(): bit mask for falling edge interrupt status.
 */
#define FALLING_EDGE(x)	(1 << (x + 12))

/**
 * REPEAT(): bit mask for repeat mode is used to configure registers
 */
#define REPEAT(x)		(x << 4)

/**
 * Macro is used for asserting channel number
 */
#define IS_SYNCIO_CHANNEL(CHANNEL)		((CHANNEL >= SG_CHANNEL_0) && \
										(CHANNEL < SG_CHANNEL_MAX))
/**
 * Macro is used for asserting infinite mode
 */
#define IS_SYNCIO_INF_MODE(INF_MODE)	((INF_MODE == SG_INF_EN) || \
										(INF_MODE == SG_INF_DIS))
/**
 * Macro is used for asserting trigger mode
 */
#define IS_SYNCIO_TRIG_MODE(TRIG_MODE)	(((TRIG_MODE) == SG_TRIG_SW) || \
										((TRIG_MODE) == SG_TRIG_HW))
/**
 * Macro is used for asserting interrupt modes
 */
#define SG_INTR_MODE_ALL				(SG_INTR_PULSE_DONE | \
										SG_INTR_REPEAT_DONE | \
										SG_INTR_FALLING_EDGE)

#define IS_SYNCIO_INTR_MODES(MODES)		((MODES | SG_INTR_MODE_ALL) == \
										SG_INTR_MODE_ALL)

/* Private variables ---------------------------------------------------------*/
/* SyncIO interrupt configurations */
static syncio_irq_cfg_t syncio_irq_cfgs[SG_CHANNEL_MAX] =
{
	{.callback_handler = NULL, .irq_is_enabled = FALSE},	/* SyncIO0 */
	{.callback_handler = NULL, .irq_is_enabled = FALSE},	/* SyncIO1 */
	{.callback_handler = NULL, .irq_is_enabled = FALSE},	/* SyncIO2 */
	{.callback_handler = NULL, .irq_is_enabled = FALSE},	/* SyncIO3 */
	{.callback_handler = NULL, .irq_is_enabled = FALSE},	/* SyncIO4 */
	{.callback_handler = NULL, .irq_is_enabled = FALSE}		/* SyncIO5 */
};

/* Local function prototypes -------------------------------------------------*/
static void syncio_irq_handler(void);

/* Exported functions---------------------------------------------------------*/
/**
 * hal_syncio_init
 * The function shall initialize SyncIO global features
 */
void hal_syncio_init(hal_syncio_channel_t chid)
{
	/* Reset pulse generator setting as default */
	SYNCIO->SG[chid].CTRL = (uint32_t)(SG_INF_EN | SG_TRIG_SW | SG_GEN_DIS);
	SYNCIO->SG[chid].LAT1 = 0;
	SYNCIO->SG[chid].LAT2 = 0;
	SYNCIO->SG[chid].WIDTH1 = 0;
	SYNCIO->SG[chid].WIDTH2 = 0;
	/* Disable all interrupts */
	SYNCIO->INT_MASK = 0;
	/* Clear all interrupts */
	SYNCIO->INT_CLR  = 0x0003FFFF;
}

/**
 * hal_syncio_config
 * Configure pulse generator channel parameters
 */
void hal_syncio_config(hal_syncio_channel_t chid, hal_syncio_cfg_t *cfg_val)
{
	assert_param(IS_SYNCIO_CHANNEL(chid));
	assert_param(cfg_val != NULL);
	assert_param(IS_SYNCIO_INF_MODE(cfg_val->inf_mode));
	assert_param(IS_SYNCIO_TRIG_MODE(cfg_val->trig_mode));

	/* Configure SG channel*/
	SYNCIO->SG[chid].CTRL = cfg_val->trig_mode;
	if(cfg_val->inf_mode)
	{
		SYNCIO->SG[chid].CTRL |= cfg_val->inf_mode;
	}
	else
	{
		SYNCIO->SG[chid].CTRL |= REPEAT(cfg_val->repeat);
	}
	SYNCIO->SG[chid].LAT1 = cfg_val->lat1;
	SYNCIO->SG[chid].LAT2 = cfg_val->lat2;
	SYNCIO->SG[chid].WIDTH1 = cfg_val->width1;
	SYNCIO->SG[chid].WIDTH2 = cfg_val->width2;
}

/**
 * @brief hal_syncio_enable
 * The function enables generating the SyncIO pulse of independent channel.
 */
void hal_syncio_enable(hal_syncio_channel_t chid)
{
	assert_param(IS_SYNCIO_CHANNEL(chid));

	/* Enable Pulse Generator mode */
	SYNCIO->SG[chid].CTRL |= SG_GEN_EN;
}

/**
 * hal_syncio_disable
 * The function disables generating the SyncIO pulse of independent channel.
 */
void hal_syncio_disable(hal_syncio_channel_t chid)
{
	assert_param(IS_SYNCIO_CHANNEL(chid));

	/* Disable Pulse Generator mode */
	SYNCIO->SG[chid].CTRL &= ~SG_GEN_EN;
}

/**
 * hal_syncio_trigger
 * The function triggers generating the SyncIO pulse
 * 		of the channels configured as SW trigger mode.
 */
void hal_syncio_trigger(void)
{
	SYNCIO->GBL_CTRL = SG_SW_TRIGGER;
}

/**
 * @brief hal_syncio_syncio_enable_irq
 * The function enable interrupt of independent SyncIO channel
 */
void hal_syncio_enable_irq(hal_syncio_channel_t chid,
												hal_syncio_irq_t *irq_cfg)
{
	assert_param(IS_SYNCIO_CHANNEL(chid));
	assert_param(irq_cfg != NULL);
	assert_param(IS_SYNCIO_INTR_MODES(irq_cfg->irq_modes));
	assert_param(irq_cfg->callback_handler != NULL);

	/* Store interrupt configuration */
	syncio_irq_cfgs[chid].callback_handler = irq_cfg->callback_handler;
	syncio_irq_cfgs[chid].irq_is_enabled = TRUE;
	/* Enable interrupt */
	SYNCIO->INT_MASK |= (irq_cfg->irq_modes << chid);
	/* Register interrupt handler */
	vic_register_irq(PPG_IRQn, syncio_irq_handler);
}

/**
 * @brief hal_syncio_syncio_disable_irq
 * The function disable interrupt of independent SyncIO channel
 */
void hal_syncio_disable_irq(hal_syncio_channel_t chid)
{
	uint8_t		i;

	assert_param(IS_SYNCIO_CHANNEL(chid));

	/* Disable interrupt */
	SYNCIO->INT_MASK &= ~((PULSE_DONE(chid) | REPEAT_DONE(chid) |
							FALLING_EDGE(chid)));
	/* Clear interrupt pending */
	SYNCIO->INT_CLR |= PULSE_DONE(chid) | REPEAT_DONE(chid) |
							FALLING_EDGE(chid);
	/* Reset interrupt configuration */
	syncio_irq_cfgs[chid].callback_handler = NULL;
	syncio_irq_cfgs[chid].irq_is_enabled = FALSE;
	/* Unregister interrupt handler */
	for (i = 0; i < SG_CHANNEL_MAX; i++)
		if (syncio_irq_cfgs[i].irq_is_enabled == TRUE)
			break;
	if (i == SG_CHANNEL_MAX)
		vic_unregister_irq(PPG_IRQn);
}

/**
 * @brief hal_syncio_get_counter
 * The function gets the counter register of independent SyncIO channel,
 * used for debug purpose.
 */
uint32_t hal_syncio_get_counter(hal_syncio_channel_t chid)
{
	assert_param(IS_SYNCIO_CHANNEL(chid));

	return SYNCIO->SG[chid].COUNTER;
}

/* Local function implementation ---------------------------------------------*/
/**
 * syncio_irq_handler
 * The local function which processes SyncIO Interrupts
 */
static void syncio_irq_handler(void)
{
	for (int i = 0; i < SG_CHANNEL_MAX; i++)
	{
		/* Check whether pulse done interrupt occurred */
		if((SYNCIO->INT_STATE & PULSE_DONE(i)) != RESET)
		{
			/* Clear interrupt bit */
			SYNCIO->INT_CLR |= PULSE_DONE(i);
			/* Jump to handler */
			syncio_irq_cfgs[i].callback_handler(i, SG_INTR_PULSE_DONE);
		}
		/* Check whether repeat done interrupt occurred */
		if((SYNCIO->INT_STATE & REPEAT_DONE(i)) != RESET)
		{
			/* Clear interrupt bit */
			SYNCIO->INT_CLR |= REPEAT_DONE(i);
			/* Jump to handler */
			syncio_irq_cfgs[i].callback_handler(i, SG_INTR_REPEAT_DONE);
		}
		/* Check whether falling edge interrupt occurred */
		if((SYNCIO->INT_STATE & FALLING_EDGE(i)) != RESET)
		{
			/* Clear interrupt bit */
			SYNCIO->INT_CLR |= FALLING_EDGE(i);
			/* Jump to handler */
			syncio_irq_cfgs[i].callback_handler(i, SG_INTR_FALLING_EDGE);
		}
	}
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
