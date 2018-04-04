/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    timer.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    aug-28-2016
 * @brief   This file contains expand of the main application.
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "hal_com.h"
#include "hal_gpio.h"
#include "hal_pwm.h"
#include "hal_ddr.h"
#include "flash.h"
#include "sb.h"

#define TICKS_PER_USEC          (533000000 / 1000000)
#define TICKS_PER_MSEC          (533000000 / 1000)
#define FREERUN_TIMERn			2
#define	TIMER_LoadCount_OFF	0x00
#define	TIMER_CurrentValue_OFF	0x04
#define	TIMER_Control_OFF	0x08
#define	TIMER_EOI_OFF		0x0C
#define	TIMER_IntStatus_OFF	0x10

void
timerN_enable( int n )
{
	unsigned int x;
	x = readl( TIMER_BASE + (n * 20) + TIMER_Control_OFF );
	writel(TIMER_BASE + (n * 20) + TIMER_Control_OFF,  x | 0x1);
}

void
timerN_disable( int n )
{
	unsigned int x;
	x = readl( TIMER_BASE + (n * 20) + TIMER_Control_OFF );
	writel(TIMER_BASE + (n * 20) + TIMER_Control_OFF, x &(~0x1));
}


void
timerN_config( int n, unsigned int rate, int enable_int)
{
	unsigned int x;

	timerN_disable( n );

	if (rate) {
		writel(533000000 / rate, TIMER_BASE + (n * 20) + TIMER_LoadCount_OFF);
		x = 0;
	}
	else {
		writel(TIMER_BASE + (n * 20) + TIMER_LoadCount_OFF, 0xFFFFFFFF);
		x = 2;
	}

	if( enable_int ) {
		writel(TIMER_BASE + (n * 20) + TIMER_Control_OFF, x);
	}
	else {
		writel(TIMER_BASE + (n * 20) + TIMER_Control_OFF, x | 0x4);
	}
}



unsigned int
timerN_get_count( int n )
{
	return( readl( TIMER_BASE + (n * 20) + TIMER_CurrentValue_OFF ) );
}



unsigned int
timerN_clear_int( int n )
{
	return( readl( TIMER_BASE + (n * 20) + TIMER_EOI_OFF ) );
}


unsigned int
timerN_get_int_status( int n )
{
	return( readl( TIMER_BASE + (n * 20) + TIMER_IntStatus_OFF ) );
}


unsigned int
timerN_get_LoadCount( int n )
{
	return( readl( TIMER_BASE + (n * 20) + TIMER_LoadCount_OFF ) );
}

void
udelay( unsigned long usec )
{
	unsigned int v, end_v, total_tick;
	volatile unsigned int x;

	total_tick = usec * TICKS_PER_USEC;

	v = timerN_get_count(FREERUN_TIMERn);
	if( v < total_tick ) {
		end_v = 0xFFFFFFFF - (total_tick - v);
		do {
			x = timerN_get_count(FREERUN_TIMERn);
		} while( x < v );

		do {
			x = timerN_get_count(FREERUN_TIMERn);
		} while( x > end_v );
	}
	else {
		end_v = v - total_tick;
		do {
			x = timerN_get_count(FREERUN_TIMERn);
		} while( x > end_v );
	}
}

