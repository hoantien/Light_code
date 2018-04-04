/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    log.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Dec-3-2015
 * @brief   This file contains definitions of the log driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include "log.h"
#include "os.h"
#include "hal_com.h"
#include "board_config.h"
#include "camera.h"
#include "ccb_cmd_log.h"
#include "ccb_system.h"

/* Private define ------------------------------------------------------------*/
/* ASCII character */
#define CNTLQ				0x11
#define CNTLS				0x13
#define DEL					0x7F
#define BACKSPACE			0x08
#define CR					0x0D
#define LF					0x0A
#define ESC					0x1B
#define CTRL_C				0x03
#define CTRL_D				0x04
#define TAB					0x09

#define BB_COUNT			(sizeof(bb)/sizeof(bb[0]))

#define SLOG_INFO_HEADER	"\033[92mINFO\t: "
#define SLOG_FATAL_HEADER	"\033[91mFATAL\t: "
#define SLOG_ERROR_HEADER	"\033[91mERROR\t: "
#define SLOG_WARN_HEADER	"\033[93mWARN\t: "
#define SLOG_DEBUG_HEADER	"\033[39mDEBUG\t: "
//#define ASIC_FWS2
#if(LOG_VERBOSE == STD_ON)
#define LOG_DBG(...) fprintf(stderr, ##__VA_ARGS__)
#else
#define LOG_DBG(...)
#endif
/* Private typedef -----------------------------------------------------------*/
#ifdef ASIC_FWS2
/**
 * busybox_t typedef
 */
typedef struct busybox
{
	char name[8];
	void (*f)(char *p);
	const char *usage;
} busybox_t;

/**
 * slog_data_t structure
 */
struct slog_data_t {
	char *str;
	uint8_t id;
	struct slog_data_t *next;
};

/**
 * cmd_ctrl_queue_t structure
 */
typedef struct slog_queue
{
	struct slog_data_t *head;
	struct slog_data_t *tail;
	uint32_t total;
} slog_queue_t;
#endif /* ASIC_FWS2 */

/* Private function_ ---------------------------------------------------------*/
#ifdef ASIC_FWS2
static void     com_irq_handler(uint8_t c);
static uint8_t  getline(char *str, uint32_t cnt, uint32_t n);
static char    *get_arg(char *cp, char **pNext);
static char    *get_opt(char *cp, char **pNext, char bc);
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

/* Command promt */
static void bb_slog(char *p);
static void bb_use(char *p);
static void bb_clear(char *p);
static void bb_reset(char *p);
static void bb_kill(char *p);
static void bb_ps(char *p);
static void bb_time(char *p);
static void bb_thermal(char *p);
#if (_TRACE_ENABLE == 1)
static void bb_trace(char *p);
#endif /* _TRACE_ENABLE */
static void bb_cam(char *p);
static void bb_cmd_log(char *p);
#endif /* ASIC_FWS2 */

/* Private variables ---------------------------------------------------------*/
static uint8_t usart_data;
/* Local com driver configure */
static hal_com_t com_dbg =
{
	.port_name		= CFG_LOG_DEBUG_PORT,
	.baudrate		= CFG_LOG_DEBUG_BAUDRATE,
#ifdef ASIC_FWS2
	.irq_handler	= com_irq_handler,
#else
	.irq_handler	= NULL,
#endif /* ASIC_FWS2 */
	.data			= &usart_data
};

#ifdef ASIC_FWS2
/* Slog queue */
static volatile slog_queue_t *slog_queue;
static uint8_t slog_w = LOG_START_EN;
static uint8_t slog_w_new_msg;
static uint8_t slog_w_new_ch;
static xSemaphoreHandle slog_semaphore;
static xSemaphoreHandle log_semaphore;
static xQueueHandle data;
static const char *msg_header[5] =
{	SLOG_FATAL_HEADER,
	SLOG_ERROR_HEADER,
	SLOG_WARN_HEADER,
	SLOG_INFO_HEADER,
	SLOG_DEBUG_HEADER
};

/* Busybox list */
static const busybox_t bb[] =
{
	{"slog", bb_slog,
	"\tslog  \t [-w enable multi write] "
	"[-d disable multi write] "
	"[-c clear slog] "
	"[-l log level]\r\n"
	},
	{"use", bb_use,
	"\tuse   \t [command name]\r\n"
	},
	{"clear", bb_clear,
	"\tclear \t [clear screen]\r\n"
	},
	{"reset", bb_reset,
	"\treset \t [reset system]\r\n"
	},
	{"kill", bb_kill,
	"\tkill  \t [task name]\r\n"
	},
	{"ps", bb_ps,
	"\tps    \t [-a tasks list ] "
	"[-t task trace ]\r\n"
	},
	{"time", bb_time,
	"\ttime  \t [shows local time ] "
	"[-s 'hh:mm:ss dd/mm/yyyy' settings date time ]\r\n"
	},
	{"temp", bb_thermal,
	"\ttemp  \t [shows board temperature ]\r\n"
	},
#if (_TRACE_ENABLE == 1)
	{"trace", bb_trace,
	"\ttrace \t [-s start kernel trace ] "
	"[-p stop kernel trace]\r\n"
	},
#endif /* _TRACE_ENABLE */
	{"cam", bb_cam,
	"\tcam   \t [cam_name] properties detail\r\n"
	},
	{"cmdlog", bb_cmd_log,
	"\tcmdlog\t [-d ID to delete] "
	"[-c clear all ] shows command log from host\r\n"
	},
};
#endif /** ASIC_FWS2 */

/* Testing variable ----------------------------------------------------------*/
const char *string_test;

/* Exported functions --------------------------------------------------------*/

/*
 * _write
 * Retarget printf function with USART IP
 */
int _write(int file, char *ptr, int len)
{}

/*
 * _read
 * Retarget scanf function with USART IP
 */
int _read(int file, char *ptr, int len)
{}

/**
* log_init
* Initialize usart channel and necessary components for console function
*/
void log_init(void)
{}


/*
 * com_irq_handler
 */
static void com_irq_handler(uint8_t _c)
{}

/**
 * task_log
 * Command line
 */
void task_log(void *pv)
{}

/**
 * task_console
 * Command line
 */
void task_console(void *pv)
{}

/**
  * slogf
  */

void slogf(uint8_t id, uint8_t header, const char *format, ...)
{
	string_test = format;
}

/**
 * log_err
 */
#if(LOG_ERROR == STD_ON)
void log_error(const char *format, ...)
{}
#endif

/*
 * log_warn
 */
#if(LOG_WARN == STD_ON)
void log_warn(const char *format, ...)
{}
#endif

/**
 * log_info
 */
#if(LOG_INFO == STD_ON)
void log_info(const char *format, ...)
{}
#endif

/**
 * log_debug
 */
#if (LOG_DEBUG == STD_ON)
void log_debug(const char *format, ...)
{}
#endif

/*
 * log_msg
 */
#if (LOG_MSG == STD_ON)
void log_msg(const char *format, ...)
{}
#endif
#ifdef ASIC_FWS2
/*
 * getline
 */
static uint8_t getline(char *str, uint32_t cnt, uint32_t n)
{}

/*
 * get_arg
 */
static char *get_arg(char *cp, char **pNext)
{}

static char *get_opt(char *cp, char **pNext, char bc)
{}

/**
  * @brief rtc_second_diff
  * Calculate number of days differece between past and current
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
{}

static unsigned long rtc_parse(char *time, char *date)
{}

/* bb_slog */
static void bb_slog(char *p)
{}

/* bb_use */
static void bb_use(char *p)
{}

/* bb_clear */
static void bb_clear(char *p)
{}

/* bb_reset */
static void bb_reset(char *p)
{}

/* bb_kill */
static void bb_kill(char *p)
{}

/* bb_ps */
static void bb_ps(char *p)
{}

/* bb_time */
static void bb_time(char *p)
{}

/* bb_thermal */
static void bb_thermal(char *p)
{
	/*TODO: Print out the value of thermal sensor here.*/
}

/* bb_trace */
#if (_TRACE_ENABLE == 1)
static void bb_trace(char *p)
{}
#endif /* _TRACE_ENABLE */

/* bb_cam */
static void bb_cam(char *p)
{}

/*
 * bb_cmd_log
 */
void bb_cmd_log(char *p)
{}
#endif /* ASIC_FWS2 */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
