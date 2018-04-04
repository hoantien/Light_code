/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 * @file    log.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Dec-3-2015
 * @brief   This file contains definitions of log driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOG_H__
#define __LOG_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "light_system.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define -----------------------------------------------------------*/
#define SLOG_FATAL		BIT0
#define SLOG_ERROR		BIT1
#define SLOG_WARN		BIT2
#define SLOG_INFO		BIT3
#define SLOG_DEBUG		BIT4

#define SLOG_LEVEL_0	0x00	/* No Log Message */
#define SLOG_LEVEL_1	0x01	/* FATAL */
#define SLOG_LEVEL_2	0x03	/* FATAL, ERROR */
#define SLOG_LEVEL_3	0x07	/* FATAL, ERROR, WARN */
#define SLOG_LEVEL_4	0x0F	/* FATAL, ERROR, WARN, INFO */
#define SLOG_LEVEL_5	0x1F	/* FATAL, ERROR, WARN, INFO, DEBUG */

#define COLOR_FATAL		"\033[91m"
#define COLOR_RED		"\033[91m"
#define COLOR_YELLOW	"\033[93m"
#define COLOR_GREEN		"\033[92m"
#define COLOR_WHITE		"\033[39m"

#define ERROR_HEADER()	fprintf(stderr, "\t\033[91mERROR  \t: ")
#define WARN_HEADER()	fprintf(stderr, "\t\033[93mWARNING\t: ")
#define INFO_HEADER()	fprintf(stderr, "\t\033[92mINFO   \t: ")
#define DEBUG_HEADER()	fprintf(stderr, "\t\033[39mDEBUG  \t: ")
#define COLOR_ENDL()	fprintf(stderr, "\033[39m")

#define TASK_CONSOLE_TIME_SLEEP	1

/* Exported macro ------------------------------------------------------------*/
#if(LOG_VERBOSE == STD_ON)
#define SLOGF(hdr, ...) slogf(SLOGF_ID, hdr, ##__VA_ARGS__)
#else
#define SLOGF(hdr, ...)
#endif
/* Exported functions---------------------------------------------------------*/

/**
 * @brief log_init
 * Initialize UART channel and necessary components for console function
 */
void log_init(void);

/**
  * @brief task_console
  * Command line
  */
void task_console(void *p);
/**
  * @brief  task_log
  * Command line
  */
void task_log(void *pv);
/**
  * @brief  task_queue_log
  * Command line
  */
void task_queue_log(void *pv);

/**
  * @brief slogf
  */
void slogf(uint8_t id, uint8_t header, const char *format, ...);

/**
 * @brief log_error
 * @param restriction
 * @return None
 */
#if(LOG_ERROR == STD_ON)
void log_error(const char *format, ...);
#else
#define log_error(...) {/* do nothing */}
#endif

/**
 * @brief log_warn
 * @param restriction
 * @return None
 */
#if(LOG_WARN == STD_ON)
void log_warn(const char *format, ...);
#else
#define log_warn(...) {/* do nothing */}
#endif

/**
 * @brief log_info
 * @param restriction
 * @return None
 */
#if(LOG_INFO == STD_ON)
void log_info(const char *format, ...);
#else
#define log_info(...) {/* do nothing */}
#endif

/**
 * @brief log_debug
 * @param restriction
 * @return None
 */
#if(LOG_DEBUG == STD_ON)
void log_debug(const char *format, ...);
#else
#define log_debug(...) {/* do nothing */}
#endif

/**
 * @brief log_msg
 * @param restriction
 * @return None
 */
#if(LOG_MSG == STD_ON)
void log_msg(const char *format, ...);
#else
#define log_msg(...) {/* do nothing */}
#endif

#ifdef __cplusplus
}
#endif
#endif /* __LOG_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
