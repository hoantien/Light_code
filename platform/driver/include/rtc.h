/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    rtc.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains header of rtc
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/**
 * @brief rtc_t
 * Real time clock
 */
typedef struct t_time {
	time_t raw;
	unsigned long long milisec;
} rtc_t;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief rtc_init
 * @param None
 * @return None
 */
void rtc_init(void);

/**
 * @brief rtc_set_raw_time
 * @param time RTC raw time want to set to rtc system
 * @return None
 */
void rtc_set_time(rtc_t time);

/**
 * @brief rtc_set_raw_time
 * @param None
 * @return RTC time
 */
rtc_t rtc_get_time(void);

/**
 * @brief rtc_get_time
 * @param None
 * @return None
 */
void rtc_timer_update_ms(void);
#ifdef __cplusplus
}
#endif
#endif /* __RTC_H__ */

/*********** Portions COPYRIGHT 2016 Light.Co., Ltd.*****END OF FILE****/
