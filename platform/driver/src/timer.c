/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    timer.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of the timer driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "assert.h"
#include "hal_timer.h"
#include "timer.h"
#include "log.h"
/* Privated define------------------------------------------------------------*/

#define SLOGF_ID			SLOG_ID_TIMER

/* Privated typedef  ---------------------------------------------------------*/

/*
 * Timer list typedef
 */
typedef struct timer_list
{
	struct ltimer_t *head;
	struct ltimer_t *tail;
	uint32_t total;
} timer_list_t;

/* Privated variables---------------------------------------------------------*/

/* Timer list */
static hal_timer_t hw_timer;
static timer_list_t		*timers;
static volatile uint8_t hw_timer_enabled = FALSE;

/* Privated functions---------------------------------------------------------*/
static void timer_hw_irq(void *params);

/* Exported functions --------------------------------------------------------*/
/**
 * @brief timer_init(void)
 * The function init system timer
 * @param None
 * @return None
 */
void timer_init(void)
{
	/* Allocate memory */
	timers = (timer_list_t *)assert_malloc(timers, sizeof(timer_list_t));
	timers->head = (struct ltimer_t *)assert_malloc(timers->head,
														sizeof(*timers->head));
	timers->tail = (struct ltimer_t *)assert_malloc(timers->tail,
														sizeof(*timers->tail));
	timers->head->next = timers->tail;
	timers->tail->next = timers->tail;
	timers->total = 0;
	/* Initialize timer */
	hw_timer.chid = HAL_TIM_CH2;
	/* 10 micro second */
	hw_timer.period = 10;
	hw_timer.callback_handler = timer_hw_irq;
	hal_timer_init(&hw_timer);
}

/**
 * @brief timer_create(void)
 * @param None
 * @return struct ltimer_t
 * NULL: timer create unsuccessful, valid pointer: create successful
 */
struct ltimer_t *timer_create(void)
{
	struct ltimer_t *tim = NULL;
	struct ltimer_t *ptr = NULL;
	/* Valid number of timer to create */
	if(timers->total < TIMER_MAX_NBR)
	{
		tim = (struct ltimer_t *) assert_malloc(tim, sizeof(*tim));
		/* Set default value for new timer */
		tim->id				= timers->total + 1;
		tim->autoreload		= OFF;
		tim->state			= TIMER_STOPPED;
		tim->interval		= 0;
		tim->time_expired	= 0;
		tim->handler		= NULL;
		tim->param			= NULL;
		/* Create new timer */
		tim->next			= timers->tail;
		ptr = timers->head;
		while (ptr->next != timers->tail) ptr = ptr->next;
		ptr->next = tim;
		/* Increment total msg is pushed */
		timers->total++;
	}
	else
	{
		tim = NULL;
	}
	return tim;
}

/**
 * @brief timer_start(void)
 * The function starts timer
 * @param *timer - point to timer valid
 * @return 0: Start successful, Start failed
 */
int timer_start(struct ltimer_t *timer)
{
	if(timer)
	{
		if(!hw_timer_enabled)
		{
			hw_timer_enabled = TRUE;
			hal_timer_start(hw_timer.chid);
		}
		timer->time_expired = 0;
		timer->state = TIMER_STARTED;
		return 0;
	}
	return -1;
}

/**
 * @brief timer_stop(void)
 * The function stops timer
 * @param *timer - point to timer valid
 * @return 0: stop successful, stop failed
 */
int timer_stop(struct ltimer_t *timer)
{
	if(timer)
	{
		struct ltimer_t *ptr = timers->head->next;
		uint8_t timers_running = FALSE;
		timer->state = TIMER_STOPPED;
		while (ptr != timers->tail)
		{
			/* check if exist timer is running */
			if(ptr->state == TIMER_STARTED)
			{
				timers_running = TRUE;
				break;
			}
			ptr = ptr->next;
		}
		if(hw_timer_enabled && !timers_running)
		{
			hw_timer_enabled = FALSE;
			hal_timer_stop(hw_timer.chid);
		}
		return 0;
	}
	return -1;
}

/**
 * @brief timer_delete(void)
 * The function deletes timer
 * @param *timer - point to timer valid
 * @return 0: delete successful, delete failed
 */
int timer_delete(struct ltimer_t **timer)
{
	if ((timer == NULL) || (*timer == NULL) || (timers->total == 0))
		return -1;
	else
	{
		struct ltimer_t *ptr = timers->head;
		while (ptr->next != timers->tail)
		{
			if (ptr->next->id == (*timer)->id)
			{
				ptr->next = (*timer)->next;
				vPortFree(*timer);
				*timer = NULL;
				timers->total--;
				/* Check if all SW timers were deleted */
				if(!timers->total)
				{
					hw_timer_enabled = FALSE;
					hal_timer_stop(hw_timer.chid);
				}
				return 0;
			}
			ptr = ptr->next;
		}
		return -1;
	}
}

/*
 * timer_hw_irq
 */
static void timer_hw_irq(void *params)
{
	struct ltimer_t *tim = NULL;
	/* Scan timer. */
	tim = timers->head->next;
	while (tim != timers->tail)
	{
		/* Timer expire. */
		if(tim->time_expired == tim->interval && tim->state == TIMER_STARTED)
		{
			/* Excute timer irq handler. */
			if(tim->handler)
			{
				(*tim->handler)(tim->param);
			}
			/* Check auto reload. */
			if(tim->autoreload == ON)
			{
				tim->state = TIMER_STARTED;
				/* Reset interval. */
				tim->time_expired = 0;
			}
			else
			{
				tim->state = TIMER_STOPPED;
			}
		}
		/* Timer is start. */
		else if(tim->state == TIMER_STARTED)
		{
			/* Increment Interval */
			tim->time_expired++;
		}
		/* Next timer */
		tim = tim->next;
	}
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
