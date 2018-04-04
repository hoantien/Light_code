/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_log_swapper.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    19-Apr, 2016
 * @brief   This file is used to print LOG information
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
#ifndef _IT_LOG_SWAPPER_H_
#define _IT_LOG_SWAPPER_H_
/* Includes ------------------------------------------------------------------*/
#include "hal_com.h"
#include "qc_common.h"
/* Exported typedef ----------------------------------------------------------*/
/**@brief indicate console status */
typedef enum{
	CONSOLE_UNINITIALIZED = 0,
	CONSOLE_INITIALIZED
}console_status_t;
/* Exported define -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define CR			0x0D
#define LF          0x0A
#define SPACE		0x20
#define BACKSPACE	0x08
#define DEL 		0x7F
#define DIGIT_0		0x30
#define DIGIT_9		0x39
#define UPPER_A		0x41
#define UPPER_Z		0x5A
#define LOWER_A		0x61
#define LOWER_Z		0x7A
#define ESC			0x1B
#define LEFT_ARROW  37
#define RIGHT_ARROW 39

#define REGISTER_DUMP(name, x) \
		log_printf("%12s : 0x%08x : 0x%08x\r\n", name, x, *((uint32_t*)(x)));
#define MEM8_DUMP(addr, size)

#define MEM16_DUMP(addr, size)

#define MEM32_DUMP(msg, addr, size)                                          \
log_printf(#msg "\r\n");                                                     \
for (uint32_t* ptr = (uint32_t*)addr; ptr < ((uint32_t*)addr + size); ptr++) \
{                                                                            \
	log_printf("0x%08x : 0x%08x\r\n", ptr, *(ptr));                          \
}
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables ------------------------------------------------------- */
IMPORT volatile console_status_t console_status;
/* Exported functions ------------------------------------------------------- */
/**
 * @brief Customize printf
 * @details Customize printf
 * @param[in] __format	: Output string format
 * @param[out] NA
 * @return 	NA
 */
void log_printf(const char *__format, ...);

/**
 * @brief print a string to console terminal
 * @details print a string to console terminal via serial port
 * @param[in] string[]	: indicated string that would be printed
 * @param[out] NA
 * @return NA
 */
IMPORT void console_putstring(char* string);

/**
 * @brief get a string from console terminal
 * @details get a string from console terminal via serial port
 * @param[in] NA
 * @param[out] NA
 * @return buffer address of gotten string
 */
IMPORT char* console_getstring(int* length);

#endif /**! _IT_LOG_SWAPPER_H_ */
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
