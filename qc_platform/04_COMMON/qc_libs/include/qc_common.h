/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    qc_common.h
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr-2016
 * @brief   This file contains expand of the hal_timer driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	20-Feb-2016	Initial revision:
 *                      - Infrastructure.
 * * 1.0.1	19-Apr-2016 Test HAL baseline 1.0.0
 */
/******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _QC_COMMON_H_
#define _QC_COMMON_H_
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#ifndef IMPORT
	#define IMPORT extern
#endif
#ifndef LOCAL
	#define LOCAL static
#endif
#ifndef NULL
	#define NULL ((void*)0)
#endif
#ifndef FALSE
	#define FALSE 0
#endif
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef ZERO
	#define ZERO 0
#endif
#ifndef INLINE
#define INLINE inline
#endif

typedef int (*qc_handler_t)(char** argv, int agrc);

typedef struct {
	char* cmd;
	qc_handler_t handler;
} it_map_t;

typedef enum
{
	E_OK = 0,
	E_PARAM,
	E_SYS,
	E_TIMEOUT,
	E_DEF,
	E_MEM,
	E_UNKNOW = 0xFFFF
}std_return_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @brief Handler parser
 * @detail To parse the handler based on the input command
 * @param[in] cmd		: input command
 * @param[in] map_table	: traced table
 * @param[out] NA
 * @return -1	: invalid command
 *          Other: index of command in map table
 */
IMPORT int handler_parser(char* cmd, it_map_t map_table[]);

/**
 * @brief Parse parameter from input string.
 * @detail To parse parameter from input string.
 * @param[in] str		: input string
 * @param[out] out		: output parameter list
 * @param[out] size		: number of parameters
 * @param[out] NA
 * @return 0	 : success
 *         Others: failed
 */
IMPORT int params_parser(char* str, char** out, int* range);

/**
 * @brief To make a delay in number of ms
 * @detail To make a delay in number of ms
 * @param[in] 	ms	: number of ms
 * @param[out] 	NA
 * @return 		NA
 */
IMPORT void _delay_ms(uint32_t ms);

/**
 * @brief To make a delay in number of us
 * @detail To make a delay in number of us
 * @param[in] 	us	: number of us
 * @param[out] 	NA
 * @return 		NA
 */
IMPORT void _delay_us(uint32_t us);
#endif /* _QC_COMMON_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
