/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    rtc.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-07-2016
 * @brief   This file contains expand of rtc driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "timer.h"
#include "rtc.h"

/* Privited variables --------------------------------------------------------*/
volatile rtc_t rtc_system;

/* Privited functions --------------------------------------------------------*/
static unsigned long rtc_second_diff(
	uint16_t st_day,
	uint16_t st_month,
	uint16_t st_year,
	uint16_t fs_day,
	uint16_t fs_month,
	uint16_t fs_year,
	uint16_t fs_hour,
	uint16_t fs_min,
	uint16_t fs_sec);

static unsigned long rtc_parse(char *time, char *date);
static char *get_opt(char *cp, char **pNext, char bc);

/* Exported functions --------------------------------------------------------*/
/**
 * @brief rtc_init
 * The function will be initialiazation RTC system time
 * @param
 * @return None
 */
void rtc_init(void)
{
	/* Unused source build time. */
	char date[] = "Jan 01 2016";
	char time[] = "00:00:00";

	rtc_system.raw  = rtc_parse(time, date);
	rtc_system.milisec = 0;
}

/**
 * @brief rtc_set_time
 */
void rtc_set_time(rtc_t time)
{
	rtc_system.raw = time.raw;
}

/**
 * @brief rtc_get_time
 */
rtc_t rtc_get_time(void)
{
	rtc_t time;
	time.raw = rtc_system.raw;
	time.milisec = rtc_system.milisec;
	return time;
}

/**
  * @brief rtc_second_diff
  * Calculate number of days differece between past and current.
  * @param
  * @retval
  */
static unsigned long rtc_second_diff(
	uint16_t st_day,
	uint16_t st_month,
	uint16_t st_year,
	uint16_t fs_day,
	uint16_t fs_month,
	uint16_t fs_year,
	uint16_t fs_hour,
	uint16_t fs_min,
	uint16_t fs_sec)
{
	unsigned long _tmp = 0, s_day = 0, non_leap_year = 0, leap_year = 0;
	int i;

	_tmp = fs_year - st_year;
	if(_tmp > 0)
	{
		_tmp -= 2;
		non_leap_year += 2;
		if(fs_year%4 == 0)
		{
			leap_year = _tmp/4;
		}
		else
		{
			leap_year = _tmp/4 + 1;
		}
		non_leap_year += _tmp - leap_year;
	}
	for(i = 0; i < non_leap_year; i++)
	{
		s_day += 365;
	}
	for(i = 0; i < leap_year; i++)
	{
		s_day += 366;
	}
	/* Calculate leap year */
	for(i = 1; i < fs_month; i++)
	{
		switch(i)
		{
			case 1:case 3:case 5:case 7:case 8:case 10:case 12:
				s_day += 31;
				break;
			case 4:case 6:case 9:case 11:
				s_day += 30;
				break;
			case 2:
				s_day += (fs_year%4) ? 28 : 29;
				break;
			default:
				break;
		}
	}

	s_day += fs_day - 1;
	return((s_day*24*3600) + (fs_hour*3600) + (fs_min*60) + fs_sec);
}

/*
 * get_opt
 */
static char *get_opt(char *cp, char **pNext, char bc)
{
	char *sp, lfn = 0, sep_ch = bc;

	if (cp == NULL)
	{ /* Skip NULL pointers*/
		*pNext = cp;
		return cp;
	}

	for ( ; *cp == bc || *cp == '\"'; cp++)
	{
		/* Skip blanks and starting  " */
		if (*cp == '\"')
		{
			sep_ch = '\"';
			lfn = 1;
		}
		*cp = 0;
	}

	for (sp = cp; *sp != '\r' && *sp != '\n'; sp++)
	{
		if (lfn && *sp == '\"')	break;
		if (!lfn && *sp == bc)	break;
	}

	for ( ; *sp == sep_ch || *sp == '\r' || *sp == '\n'; sp++)
	{
		*sp = 0;
		if (lfn && *sp == sep_ch) { sp++; break; }
	}
	*pNext = (*sp) ? sp : NULL;/* Next arg */

	return cp;
}

static unsigned long rtc_parse(char *time, char *date)
{
	uint16_t day	= 0;
	uint16_t month	= 0;
	uint16_t year	= 0;
	uint16_t hour	= 0;
	uint16_t min	= 0;
	uint16_t sec	= 0;
	unsigned long ret = 0;
	char *mS[12] =
	{"Jan", "Feb", "Mar", "Apr", "May", "Jun",
	"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
	char *str, *next;
	/* Get TIME */
	str = get_opt(time, &next, ':');
	hour = atoi(str);
	str = get_opt(next, &next, ':');
	min = atoi(str);
	str = get_opt(next, &next, 0);
	sec = atoi(str);

	/* Get DATE */
	str = get_opt(date, &next, ' ');
	while (strcmp(str, mS[month]) && month < 12)
	{
		month++;
	}
	str = get_opt(next, &next, ' ');
	day = atoi(str);
	str = get_opt(next, &next, 0);
	year = atoi(str);
	ret = rtc_second_diff(1, 1, 1970, day, month + 1, year, hour, min, sec);

	return ret;
}

void rtc_timer_update_ms(void)
{
	if((++rtc_system.milisec % 1000) == 0)
		rtc_system.raw++;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
