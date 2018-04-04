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
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include "assert.h"
#include "os.h"
#include "lcc_cmd_log.h"
#include "lcc_cmd.h"
#include "rtc.h"
#include "ddr_heap.h"
#include "log.h"

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

#define SLOG_CIRC_BUFFER_SIZE (512 * 1024)
#define MAX_LOG_STRING_LEN  512

#if(LOG_VERBOSE == STD_ON)
#define LOG_DBG(...)		{ \
								fprintf(stderr, ##__VA_ARGS__); \
								slog_w_new_msg = 1; \
								xSemaphoreGive(slog_semaphore); \
							}

#else
#define LOG_DBG(...)
#endif
/* Private typedef -----------------------------------------------------------*/
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
typedef struct
{
	char *str;
	uint8_t id;
} slog_data_t;

typedef struct
{
    char *buffer_start;
    const char *buffer_end;
    const char *buffer_wrap_th;
    const char *read_p;
    const char *buffer_used_end;
    char *write_p;
    SemaphoreHandle_t rwp_mutex;
} log_circ_buffer_t;

/* Private function_ ---------------------------------------------------------*/
static void     com_irq_handler(uint8_t c);
static uint8_t  getline(char *str);
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

/* Private variables ---------------------------------------------------------*/
static uint8_t usart_data;
/* Local com driver configure */
static hal_com_t com_dbg =
{
	.port_name		= CFG_LOG_DEBUG_PORT,
	.baudrate		= CFG_LOG_DEBUG_BAUDRATE,
	.irq_handler	= com_irq_handler,
	.data			= &usart_data
};

/* Slog queue */
static bool slog_w = true;
static uint8_t slog_w_new_msg;
static uint8_t slog_w_new_ch;
static SemaphoreHandle_t slog_semaphore;
static SemaphoreHandle_t slog_tx_semaphore;
static SemaphoreHandle_t log_semaphore;
static xQueueHandle data;
static const char *msg_header[5] =
{	SLOG_FATAL_HEADER,
	SLOG_ERROR_HEADER,
	SLOG_WARN_HEADER,
	SLOG_INFO_HEADER,
	SLOG_DEBUG_HEADER
};
static log_circ_buffer_t slog_circ_buffer;
static uint32_t num_dropped_logs = 0;

/* Busybox list */
static const busybox_t bb[] =
{
	{"slog", bb_slog,
	"\tslog  \t [-e enable multi write] "
	"[-d disable multi write] "
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

/* Exported variables --------------------------------------------------------*/
extern lcc_cmd_log_queue_t *lcc_cmd_log;		/* LCC Command log */

/* Exported functions --------------------------------------------------------*/

/*
 * _write
 * Retarget printf function with USART IP
 */
int _write(int file, char *ptr, int len)
{
	int n;
	for (n = 0; n < len; n++)
	{
		*(com_dbg.data) = *ptr++;
		hal_com_sendbyte(&com_dbg);
	}
	return n;
}

/*
 * _read
 * Retarget scanf function with USART IP
 */
int _read(int file, char *ptr, int len)
{
	int CharCnt = 0x00;
	while(CharCnt < len)
	{
		hal_com_readbyte(&com_dbg);
		*ptr++ = *(com_dbg.data);
		CharCnt++;
	}
	return CharCnt;
}

/**
* log_init
* Initialize usart channel and necessary components for console function
*/
void log_init(void)
{
    slog_circ_buffer.buffer_start = gen_pvPortMalloc(&ddr_heap_attribute, SLOG_CIRC_BUFFER_SIZE);
    slog_circ_buffer.buffer_end = slog_circ_buffer.buffer_start + SLOG_CIRC_BUFFER_SIZE;
    slog_circ_buffer.buffer_used_end = slog_circ_buffer.buffer_end;
    slog_circ_buffer.buffer_wrap_th = slog_circ_buffer.buffer_end - MAX_LOG_STRING_LEN;
    slog_circ_buffer.write_p = slog_circ_buffer.buffer_start;
    slog_circ_buffer.read_p = slog_circ_buffer.buffer_start;
    slog_circ_buffer.rwp_mutex = xSemaphoreCreateMutex();
    slog_semaphore = xSemaphoreCreateBinary();
    log_semaphore = xSemaphoreCreateMutex();
    slog_tx_semaphore = xSemaphoreCreateBinary();
    data = xQueueCreate(5, sizeof(uint8_t));

	hal_com_init(&com_dbg);
	hal_com_enable_irq(&com_dbg);
}

/*
 * com_irq_handler
 */
static void com_irq_handler(uint8_t _c)
{
	static BaseType_t xhigherpriority = pdFALSE;

	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xQueueSendFromISR(data, &_c, &xhigherpriority);
		if(xhigherpriority)
			portYIELD_FROM_ISR(xhigherpriority);
	}
}

/**
 * task_log
 * Command line
 */
void task_log(void *pv)
{
	task_handle_t *hdl = (task_handle_t *)(pv);

	/* Task ready id */
	hdl->state = TASK_READY;

	taskENTER_CRITICAL();
	log_msg("Start %s\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
	while(1)
	{
		xSemaphoreTake(slog_semaphore, portMAX_DELAY);
		if(slog_w_new_msg)
		{
			fflush(stderr);
			slog_w_new_msg = 0;
		}
		if(slog_w_new_ch)
		{
			fflush(stdout);
			slog_w_new_ch = 0;
		}
	}
}

/**
 * task_console
 * Command line
 */
void task_console(void *pv)
{
	task_handle_t *hdl = (task_handle_t *)(pv);
	char *sp, *cp, *next;
	char *cmd_line = assert_malloc(cmd_line, CONSOLE_LINE_LEN*sizeof(uint8_t));
	uint32_t i;

	while (TASK_READY != task_handler[task_query_tid("slog")].state);

	/* Task ready id */
	hdl->state = TASK_READY;
	/* Get task id */
	taskENTER_CRITICAL();
	log_msg("Start %s\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();

	while(1)
	{
		/* Display prompt for VT100 terminal. */
		fprintf(stderr, "%s ", COMMAND_PROMPT);
		slog_w_new_msg = 1;
		xSemaphoreGive(slog_semaphore);
		if (!getline(cmd_line))
			continue;
		/* Get argument */
		sp = get_arg(cmd_line, &next);
		if(*sp == 0)
			 continue;
		for(cp = sp; *cp && *cp != ' '; cp++)
		{
			*cp = tolower((unsigned char)(*cp));/* Command to lower-case. */
		}
		for(i = 0; i < BB_COUNT; i++)
		{
			if(strcmp(sp, (const char *)&bb[i].name))
				continue;
			bb[i].f(next); /* Execute command function. */
			break;
		}
		if(i == BB_COUNT)
		{
			fprintf(stderr, "\tUsing 'use' command to view detail!\r\n");
			slog_w_new_msg = 1;
			xSemaphoreGive(slog_semaphore);
		}
	}
}

static int slog_write_header(char* out_str, size_t max_len_out, uint8_t user_id, uint8_t header)
{
    /* Header of the slog message. */
    rtc_t rtc_time = rtc_get_time();
    struct tm *t_info = localtime(&rtc_time.raw);
    uint8_t hdr_idx;
    switch(header)
    {
        case SLOG_FATAL:
            hdr_idx = 0;
            break;
        case SLOG_ERROR:
            hdr_idx = 1;
            break;
        case SLOG_WARN:
            hdr_idx = 2;
            break;
        case SLOG_INFO:
            hdr_idx = 3;
            break;
        case SLOG_DEBUG:
        default:
            hdr_idx = 4;
            break;
    }

    /* Write to output */
    int ret = snprintf(out_str, max_len_out, " %03d\t%02d:%02d:%02d.%03d\t%s",
            user_id,
            t_info->tm_hour, t_info->tm_min, t_info->tm_sec, (int)(rtc_time.milisec%1000),
            msg_header[hdr_idx]);
    return ret;
}

/**
 * task_queue_log
 */
void task_queue_log(void *pv)
{
	task_handle_t *hdl = (task_handle_t *)(pv);

	while (TASK_READY != task_handler[task_query_tid("slog")].state);

	/* Task ready id */
	hdl->state = TASK_READY;
	/* Get task id */
	taskENTER_CRITICAL();
	log_msg("Start %s\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
	while (1)
	{
	    xSemaphoreTake(slog_tx_semaphore, portMAX_DELAY);

	    // Protect read of write_p and buffer_used_end
	    xSemaphoreTake(slog_circ_buffer.rwp_mutex, portMAX_DELAY);
	    const char *buf_write_p = slog_circ_buffer.write_p;
	    const char *buf_used_end = slog_circ_buffer.buffer_used_end;
	    xSemaphoreGive(slog_circ_buffer.rwp_mutex);
	    const char *buf_read_p = slog_circ_buffer.read_p;
	    while (buf_read_p != buf_write_p)
	    {
            int len = 0;
            if (buf_write_p > buf_read_p)
                len = buf_write_p - buf_read_p;
            else
                len = buf_used_end - buf_read_p;
            if (len > 0)
            {
                // Send data over UART
                _write(2, (char*)buf_read_p, len);
                // Update read_p
                buf_read_p += len;
                if (buf_read_p == buf_used_end)
                    buf_read_p = slog_circ_buffer.buffer_start;
                // Protect write of read_p
                xSemaphoreTake(slog_circ_buffer.rwp_mutex, portMAX_DELAY);
                slog_circ_buffer.read_p = buf_read_p;
                xSemaphoreGive(slog_circ_buffer.rwp_mutex);
            }
	    }
	}
}

/**
  * slogf
  */
void slogf(uint8_t id, uint8_t header, const char *format, ...)
{
    if (!slog_w)
        return;
    uint8_t header_f = light_system->settings->log_ctrl & header;
    if (header_f == 0)
        return;

    xSemaphoreTake(log_semaphore, portMAX_DELAY);
    // Protect read of read_p
    xSemaphoreTake(slog_circ_buffer.rwp_mutex, portMAX_DELAY);
    const char *buf_read_p = slog_circ_buffer.read_p;
    xSemaphoreGive(slog_circ_buffer.rwp_mutex);

    size_t max_len_out;
    if ((buf_read_p == slog_circ_buffer.buffer_start) || (buf_read_p >= slog_circ_buffer.buffer_wrap_th))
        max_len_out = slog_circ_buffer.buffer_wrap_th - slog_circ_buffer.write_p;
    else if (slog_circ_buffer.write_p >= buf_read_p)
        max_len_out = slog_circ_buffer.buffer_end - slog_circ_buffer.write_p;
    else
        max_len_out = buf_read_p - slog_circ_buffer.write_p;
    int len = slog_write_header(slog_circ_buffer.write_p, max_len_out, id, header_f);
    if (len < max_len_out)
    {
        va_list argptr;
        va_start(argptr, format);
        int slen = vsnprintf(slog_circ_buffer.write_p + len, max_len_out - len, format, argptr);
        va_end(argptr);
        if (slen < 0)
            slen = snprintf(slog_circ_buffer.write_p + len, max_len_out - len, "!!!ERROR formatting log!!!");
        len += slen;
        if (len < max_len_out)
            len += snprintf(slog_circ_buffer.write_p + len, max_len_out - len, "\033[39m\r\n");
    }
    if (len >= max_len_out)
    {
        ++num_dropped_logs; // TODO: later log error when logs were dropped
    }
    else
    {
        // Update write_p and possibly buffer_used_end
        char *new_write_p = slog_circ_buffer.write_p + len;
        if (new_write_p >= slog_circ_buffer.buffer_wrap_th)
        {
            slog_circ_buffer.buffer_used_end = new_write_p;
            new_write_p = slog_circ_buffer.buffer_start;
        }
        // Protect write of write_p
        xSemaphoreTake(slog_circ_buffer.rwp_mutex, portMAX_DELAY);
        slog_circ_buffer.write_p = new_write_p;
        xSemaphoreGive(slog_circ_buffer.rwp_mutex);

        xSemaphoreGive(slog_tx_semaphore);
    }
    xSemaphoreGive(log_semaphore);
}

/**
 * log_err
 */
#if(LOG_ERROR == STD_ON)
void log_error(const char *format, ...)
{
	ERROR_HEADER();
	va_list argptr;
	va_start(argptr, format);
	vfprintf(stderr, format, argptr);
	va_end(argptr);
	COLOR_ENDL();
}
#endif

/*
 * log_warn
 */
#if(LOG_WARN == STD_ON)
void log_warn(const char *format, ...)
{
	WARN_HEADER();
	va_list argptr;
	va_start(argptr, format);
	vfprintf(stderr, format, argptr);
	va_end(argptr);
	COLOR_ENDL();
}
#endif

/**
 * log_info
 */
#if(LOG_INFO == STD_ON)
void log_info(const char *format, ...)
{
	INFO_HEADER();
	va_list argptr;
	va_start(argptr, format);
	vfprintf(stderr, format, argptr);
	va_end(argptr);
	COLOR_ENDL();
}
#endif

/**
 * log_debug
 */
#if (LOG_DEBUG == STD_ON)
void log_debug(const char *format, ...)
{
	DEBUG_HEADER();
	va_list argptr;
	va_start(argptr, format);
	vfprintf(stderr, format, argptr);
	va_end(argptr);
	COLOR_ENDL();
}
#endif

/*
 * log_msg
 */
#if (LOG_MSG == STD_ON)
void log_msg(const char *format, ...)
{
	va_list argptr;
	va_start(argptr, format);
	vfprintf(stderr, format, argptr);
	va_end(argptr);
}
#endif

/*
 * getline
 */
static uint8_t getline(char *str)
{
	uint32_t cnt = 0;
	char c;

	do
	{
		xQueueReceive(data, &c, portMAX_DELAY);
		switch (c)
		{
			case CNTLQ:							/* Ignore Control S/Q */
			case CNTLS:
				xSemaphoreGive(slog_semaphore);
				break;
			case BACKSPACE:
			case DEL:
				if (cnt == 0)
					continue;
				cnt--;						/* Decrement count */
				str--;						/* And line pointer */
				putchar(0x08);				/* Echo backspace */
				putchar(' ');
				putchar(0x08);
				slog_w_new_ch = 1;
				xSemaphoreGive(slog_semaphore);
				break;
			case ESC:						/* ESC - stop editing line */
			case CTRL_C:					/* CTRL+C cancel editing line */
				*str = 0;
				putchar('\r');
				putchar('\n');
				slog_w_new_ch = 1;
				xSemaphoreGive(slog_semaphore);
				return 0;
			case CR:						/* CR - done, stop editing line */
				*str = c;
				str++;						/* Increment line pointer */
				c = LF;
				putchar('\r');
				putchar('\n');
				slog_w_new_ch = 1;
				xSemaphoreGive(slog_semaphore);
				break;
			default:
				putchar(*str = c);			/* Echo and store character */
				str++;						/* Increment line pointer */
				cnt++;						/* And count */
				slog_w_new_ch = 1;
				xSemaphoreGive(slog_semaphore);
				break;
		}
	} while (cnt < CONSOLE_LINE_LEN - 2  &&  c != LF);	/* Check limit and CR */
	*str = 0;								/* Mark end of string */
	return 1;
}

/*
 * get_arg
 */
static char *get_arg(char *cp, char **pNext)
{
	char *sp, lfn = 0, sep_ch = ' ';

	if (cp == NULL)
	{
		/* Skip NULL pointers */
		*pNext = cp;
		return cp;
	}

	for ( ; *cp == ' ' || *cp == '\"'; cp++)
	{
		/* Skip blanks and starting  " */
		if (*cp == '\"')
		{
			sep_ch = '\"';
			lfn = 1;
		}
		*cp = 0;
	}

	for (sp = cp; *sp != CR && *sp != LF; sp++)
	{
		if (lfn && *sp == '\"') break;
		if (!lfn && *sp == ' ') break;
	}

	for ( ; *sp == sep_ch || *sp == CR || *sp == LF; sp++)
	{
		*sp = 0;
		if (lfn && *sp == sep_ch) { sp++; break; }
	}

	*pNext = (*sp) ? sp : NULL; /* Next arg*/

	return cp;
}

static char *get_opt(char *cp, char **pNext, char bc)
{
	char *sp, lfn = 0, sep_ch = bc;

	if (cp == NULL)
	{
		/* Skip NULL pointers */
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
		if (lfn && *sp == '\"') break;
		if (!lfn && *sp == bc) break;
	}

	for ( ; *sp == sep_ch || *sp == '\r' || *sp == '\n'; sp++)
	{
		*sp = 0;
		if (lfn && *sp == sep_ch) { sp++; break; }
	}

	*pNext = (*sp) ? sp : NULL;	/* Next arg */

	return cp;
}

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
{
	unsigned long _tmp = 0, s_day = 0, non_leap_year = 0, leap_year = 0;
	int i;
	_tmp = fs_year - st_year;
	if(_tmp > 0)
	{
		_tmp -= 2;
		non_leap_year += 2;
		if(fs_year%4 == 0)
			leap_year = _tmp/4;
		else
			leap_year = _tmp/4 + 1;
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
	/* Calculate leep year */
	for(i = 1; i < fs_month; i++)
	{
		switch(i)
		{
			case 1:
			case 3:
			case 5:
			case 7:
			case 8:
			case 10:
			case 12:
				s_day += 31;
				break;
			case 4:
			case 6:
			case 9:
			case 11:
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

static unsigned long rtc_parse(char *time, char *date)
{
	uint16_t day	= 0;
	uint16_t month	= 0;
	uint16_t year	= 0;
	uint16_t hour	= 0;
	uint16_t min	= 0;
	uint16_t sec	= 0;
	unsigned long ret = 0;
	char *str, *next;

	/* Get TIME */
	str = get_opt(time, &next, ':');
	hour = atoi(str);
	str = get_opt(next, &next, ':');
	min = atoi(str);
	str = get_opt(next, &next, 0);
	sec = atoi(str);

	/* Get DATE */
	str = get_opt(date, &next, '/');
	day = atoi(str);
	str = get_opt(next, &next, '/');
	month = atoi(str);
	str = get_opt(next, &next, 0);
	year = atoi(str);

	ret = rtc_second_diff(1, 1, 1970, day, month + 1, year, hour, min, sec);

	return ret;
}

/* bb_slog */
static void bb_slog(char *p)
{
	char *str, *next;

	str = get_arg(p, &next);
	if(str != NULL)
	{
		if(!strcmp(str, "-e"))
		{
			slog_w = true;
			return;
		}
		else if(!strcmp(str, "-d"))
		{
			slog_w = false;
			return;
		}
		else if(!strcmp(str, "-l"))
		{
			str = get_arg(next, &next);
			if (str != NULL)
			{
				const uint8_t level[] =
					{
						SLOG_LEVEL_0,
						SLOG_LEVEL_1,
						SLOG_LEVEL_2,
						SLOG_LEVEL_3,
						SLOG_LEVEL_4,
						SLOG_LEVEL_5
					};
				uint8_t lc = atoi(str);
				if (lc < 6)
					light_system->settings->log_ctrl = level[lc];
				return;
			}
		}
	}

	for(uint8_t i = 0; i < BB_COUNT; i++)
	{
		if(!strcmp("slog", bb[i].name))
		{
			fprintf(stderr, "%s", bb[i].usage);
			slog_w_new_msg = 1;
			xSemaphoreGive(slog_semaphore);
			break;
		}
	}
}

/* bb_use */
static void bb_use(char *p)
{
	char *str, *next;
	int i;

	str = get_arg(p, &next);
	if(str == NULL)
	{
		i = 0;
		while(i < BB_COUNT)
		{
			fprintf(stderr, "%s", bb[i].usage);
			i++;
		}
		slog_w_new_msg = 1;
	}
	else
	{
		i = 0;
		while(i < BB_COUNT)
		{
			if(!strcmp(str, bb[i].name))
			{
				fprintf(stderr, "%s", bb[i].usage);
				break;
			}
			i++;
		}
		slog_w_new_msg = 1;
	}
}

/* bb_clear */
static void bb_clear(char *p)
{
	fprintf(stderr, "\033[2J");
	slog_w_new_msg = 1;
}

/* bb_reset */
static void bb_reset(char *p)
{
	/* TODO: Implement a reset function here. */
	__disable_irq();
	while(1);
}

/* bb_kill */
static void bb_kill(char *p)
{
	char *str, *next;
	int i;
	str = get_arg(p, &next);
	if(str == NULL)
	{
		i = 0;
		while(i < BB_COUNT)
		{
			if(!strcmp("kill", bb[i].name))
			{
				fprintf(stderr, "%s", bb[i].usage);
				slog_w_new_msg = 1;
				break;
			}
			i++;
		}
	}
	else
	{
		i = 0;
		while(i < TASK_NUM)
		{
			if(!strcmp(str, task_list[i].name))
			{
				vTaskDelete(task_handler[i].handle);
				break;
			}
			i++;
		}
		if(TASK_NUM == i)
		{
			fprintf(stderr, "Invalid Task Name\r\n");
			slog_w_new_msg = 1;
		}
	}
}

/* bb_ps */
static void bb_ps(char *p)
{
	char *next;
	char *str, *ps = NULL;
	int i;
	char c;
	int free_heap_space;

	str = get_arg(p, &next);
	if(str == NULL)
	{
		i = 0;
		while(i < BB_COUNT)
		{
			if(!strcmp("ps", bb[i].name))
			{
				fprintf(stderr, "%s", bb[i].usage);
				slog_w_new_msg = 1;
				xSemaphoreGive(slog_semaphore);
				break;
			}
			i++;
		}
	}
	else
	{
		if(!strcmp(str, "-a"))
		{
			fprintf(stderr,
					"\r\ntask          state   priority  stack   num\r\n");
			fprintf(stderr,
						"*******************************************\r\n");
			ps = assert_malloc(ps, 2048*sizeof(char));
			vTaskList(ps);
			fprintf(stderr, "%s\r\n", ps);
			vPortFree(ps);
			free_heap_space = xPortGetFreeHeapSize();
			fprintf(stderr, "\r\nHeap Free\r\n");
			fprintf(stderr, "*******************************\r\n");
			fprintf(stderr, "   %dB/%dB\r\n", free_heap_space, configTOTAL_HEAP_SIZE);
			slog_w_new_msg = 1;
			xSemaphoreGive(slog_semaphore);
		}
		if(!strcmp(str, "-t"))
		{
			ps = assert_malloc(ps, 1024*sizeof(char));
			while(1)
			{
				fprintf(stderr, "\033[2J");
				fprintf(stderr,
					"\r\ntask          state   priority  stack   num\r\n");
				fprintf(stderr,
						"*******************************************\r\n");
				vTaskList(ps);
				fprintf(stderr, "%s\r\n", ps);
				free_heap_space = xPortGetFreeHeapSize();
			fprintf(stderr, "\r\nHeap Free\r\n");
			fprintf(stderr, "*******************************\r\n");
			fprintf(stderr, "   %dB/%dB\r\n", free_heap_space, configTOTAL_HEAP_SIZE);
				slog_w_new_msg = 1;
				xSemaphoreGive(slog_semaphore);

				if(xQueueReceive(data, &c, 1000) == pdPASS)
				{
					if (c == CTRL_C)
						break;
				}
			}
			vPortFree(ps);
		}
	}
}

/* bb_time */
static void bb_time(char *p)
{
	rtc_t rtc_time;
	struct tm *t_info;
	char *buffer = assert_malloc(buffer, 80*sizeof(char));
	char *str, *next, *_day, *_time;

	str = get_arg(p, &next);
	if(str != NULL)
	{
		if(!strcmp(str, "-s"))
		{
			_time = get_arg(next, &next);
			_day = get_arg(next, &next);
			if(_day == NULL || _time == NULL)
			{
				/** TODO: What should we do here ? */
			}
			else
			{
				rtc_time.raw = rtc_parse(_time, _day);
				rtc_set_time(rtc_time);
			}
		}
	}

	rtc_time = rtc_get_time();
	time(&rtc_time.raw);
	t_info = localtime(&rtc_time.raw);
	strftime(buffer, 80, "\t%c Week %U %Z", t_info);
	fprintf(stderr, "%s\r\n", buffer);
	slog_w_new_msg = 1;
	vPortFree(buffer);
}

/* bb_thermal */
static void bb_thermal(char *p)
{
	/*TODO: Print out the value of thermal sensor here.*/
}

/* bb_trace */
#if (_TRACE_ENABLE == 1)
static void bb_trace(char *p)
{
	char *str, *next;

	str = get_arg(p, &next);
	if(str != NULL)
	{
		if(!strcmp(str, "-s"))
		{
			if (uiTraceStart() == 0)
				fprintf(stderr, "Could not start recorder\r\n");
			else
				fprintf(stderr, "Kernel trace is started\r\n");
			slog_w_new_msg = 1;
		}
		if(!strcmp(str, "-p"))
		{
			vTraceStop();
			fprintf(stderr, "Kernel trace stopped\r\n");
			slog_w_new_msg = 1;
		}
	}
}
#endif /* _TRACE_ENABLE */

/* bb_cam */
static void bb_cam(char *p)
{
	char *str, *next;
	str = get_arg(p, &next);

	if(str != NULL)
	{
		fprintf(stderr, "\r\nDetail %s\r\n", str);
/*TODO: Should re-enable */
#if 0
		cam_module_t *cam = NULL;
		uint8_t i = CAM_CH_A1;
		uint8_t EXIT = 0;
		while(i < CAM_CH_MAX_NUM && !EXIT)
		{
			cam = &light_system->cam_list[i];
			if(!strcmp(str, cam->name))
			{
				fprintf(stderr, "\tChannel ID\t: %d\r\n", cam->chid);
				fprintf(stderr, "\tName\t\t: %s\r\n", cam->name);
				fprintf(stderr, "\tCamera type\t: %s\r\n",
						cam->type == CAM_TYPE_35MM ? "35MM" :
						cam->type == CAM_TYPE_70MM ? "70MM" :
						cam->type == CAM_TYPE_150MM ? "150MM" : "Unknown");
				fprintf(stderr, "\tI2C Addr\t: 0x%02X\r\n", cam->slave_addr);
				fprintf(stderr, "\tResolution\t:\r\n");
				fprintf(stderr, "\t\tSensitivity\t: %u\r\n",
						cam_read_reg(cam, CAM_REG_SENSITIVITY, DATA_16BIT));
				fprintf(stderr, "\t\tExposure\t: %u\r\n",
						cam_read_reg(cam, CAM_REG_EXPOSURE, DATA_16BIT));
				fprintf(stderr, "\tOpen status\t: %s\r\n",
					cam->m_o_status == CAM_MODULE_CLOSE ? "CLOSE" :
					cam->m_o_status == CAM_MODULE_HW_STANDBY ? "HW STANDBY" :
					cam->m_o_status == CAM_MODULE_SW_STANDBY ? "SW STANDBY" :
					"Unknown");
				fprintf(stderr, "\tStream CSI\t:\r\n");
				fprintf(stderr, "\t\tControl status\t:\r\n");
				fprintf(stderr, "\t\t\tChannel\t\t: %s  ",
					(cam->m_stream.ctrl_status & 0x10) == 0x10 ? "CS0" : "");
				fprintf(stderr, "%s\r\n",
					(cam->m_stream.ctrl_status & 0x20) == 0x20 ? "CS1" : "");
				fprintf(stderr, "\t\t\tStatus\t\t: %s\r\n",
						(cam->m_stream.ctrl_status & 0x01) == 0x01 ? "Enable" :
						"Disable");
				fprintf(stderr,
						"\t\tVirtual Channel\t: %d\r\n", cam->m_stream.vc);
				fprintf(stderr, "\t\tData Type\t: %d\r\n", cam->m_stream.dt);
				fprintf(stderr, "\tStatus\t\t:"
						""BYTETOBINARYPATTERN""
						""BYTETOBINARYPATTERN""
						""BYTETOBINARYPATTERN""
						""BYTETOBINARYPATTERN"\r\n",
						BYTETOBINARY((uint8_t)((cam->status>>24)&0x000000ff)),
						BYTETOBINARY((uint8_t)((cam->status>>16)&0x000000ff)),
						BYTETOBINARY((uint8_t)((cam->status>>8)&0x000000ff)),
						BYTETOBINARY((uint8_t)((cam->status>>0)&0x000000ff)));
				EXIT = 1;
			}
			i++;
		}
#endif
	}

	slog_w_new_msg = 1;
}

/*
 * bb_cmd_log
 */
void bb_cmd_log(char *p)
{
	char *str, *next, *str_tid;
	struct lcc_cmd_log_t *ptr = NULL;
	uint16_t cmd_tid = 0;

	str = get_arg(p, &next);
	if(str != NULL)
	{
		if(!strcmp(str, "-d"))
		{
			str_tid = get_arg(next, &next);
			if(str_tid)
			{
				cmd_tid = (uint16_t)strtol(str_tid, NULL, 16);
				ptr = lcc_cmd_log->head->next;
				struct lcc_cmd_log_t *tmp = NULL;
				while (ptr != lcc_cmd_log->tail)
				{
					tmp = ptr->next;
					if(ptr->cmd_tid == cmd_tid)
					{
						lcc_cmd_log_delete_tid(ptr);
					}
					ptr = tmp;
				}
			}
		}

		if(!strcmp(str, "-c"))
		{
			ptr = lcc_cmd_log->head->next;
			struct lcc_cmd_log_t *tmp = NULL;
			while (ptr != lcc_cmd_log->tail)
			{
				tmp = ptr->next;
				lcc_cmd_log_delete_tid(ptr);
				ptr = tmp;
			}
		}
	}
	fprintf(stderr, "\r\nCommand Log\r\n");
	fprintf(stderr, "\r\n\tTID\tStatus\r\n");
	ptr = lcc_cmd_log->head->next;
	while (ptr != lcc_cmd_log->tail)
	{
		if(ptr)
		{
			fprintf(stderr, "\t%04X\t%s\r\n", ptr->cmd_tid,
					ptr->status == LCC_CMD_UNSUCCESS ?
					"LCC_CMD_UNSUCCESS" :
					ptr->status == LCC_CMD_SUCCESS ?
					"LCC_CMD_SUCCESS" :
					ptr->status == LCC_CMD_PENDING ?
					"LCC_CMD_PENDING" :
					ptr->status == LCC_CMD_INVALID_ARG ?
					"LCC_CMD_INVALID_ARG" :
					ptr->status == LCC_CMD_ERROR_INVALID_MBITMASK ?
					"LCC_CMD_ERROR_INVALID_MBITMASK" :
					ptr->status == LCC_CMD_ERROR_ASIC_UNAVAILABLE ?
					"LCC_CMD_ERROR_ASIC_UNAVAILABLE" :
					ptr->status == LCC_CMD_ERROR_MODULE_FAULT ?
					"LCC_CMD_ERROR_MODULE_FAULT" : "UNKNOWN");
			ptr = ptr->next;
		}
	}
	fprintf(stderr, "\r\nTotal: %ld \r\n", lcc_cmd_log->total);
	slog_w_new_msg = 1;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
