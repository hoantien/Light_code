/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_log_swapper.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr-2016
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

/* Includes ------------------------------------------------------------------*/
#include "it_log_swapper.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** Exported variables--------------------------------------------------------*/
volatile console_status_t console_status = CONSOLE_UNINITIALIZED;
/* Private variables ---------------------------------------------------------*/
LOCAL uint8_t console_buff = 0;
LOCAL hal_com_t console_obj = {COM1, COM_BAUD_115200, NULL, &console_buff };
/* Static functions ----------------------------------------------------------*/

/**
 * @brief get a fulfill a buffer
 * @details get fulfull a indicated buffer
 * @param[in] ptr	: buffer address
 * @param[in] len	: number of bytes
 * @param[out] NA
 * @return number of read bytes
 */
LOCAL int serial_get(char *ptr, int len);

/**
 * @brief send whole buffer
 * @details send whole indicated buffer
 * @param[in] ptr	: buffer address
 * @param[in] len	: number of bytes
 * @param[out] NA
 * @return number of sent bytes
 */
LOCAL int serial_send(char *ptr, int len);

/**
 * @brief Convert any character to its upper case
 * @details Convert any character to its upper case
 * @param[in] ch	: indicated character that would be converted
 * @param[out] NA
 * @return 	upper character if input is readable character
 * 			0               if input is unreadable character
 */
LOCAL char upper(char ch);

/* Exported functions ------------------------------------------------------- */

/**
 * @brief Customize printf
 * @details Customize printf
 * @param[in] __format	: Output string format
 * @param[out] NA
 * @return 	NA
 */
void log_printf(const char *__format, ...)
{
	char buffer[128];

	/**! Clear buffer */
	for (int i = 0; i < 128; i++)
	{
		buffer[i] = 0;
	}
	va_list argv;
	/* initialize argv for num number of arguments */
	va_start(argv, __format);

	/* access all the arguments assigned to argv */
	vsprintf(buffer, __format, argv);

	/* clean memory reserved for argv */
	va_end(argv);

	/**! Print buffer */
	console_putstring(buffer);
}

/**
 * @brief print a string to console terminal
 * @details print a string to console terminal via serial port
 * @param[in] string[]	: indicated string that would be printed
 * @param[out] NA
 * @return NA
 */
void console_putstring(char* string)
{
	volatile uint32_t delay = 0;
	/**! Check console bus */
	if (CONSOLE_UNINITIALIZED == console_status)
	{
		/**! Initializing log interface */
//		hal_com_init(&console_obj);
		/**! Mark console status to IDLE */
		console_obj.data = &console_buff;
		console_status = CONSOLE_INITIALIZED;
		while(delay++ < 200000);
	}
	/**! Get length of indicated string */
	int length = strlen(string);
	/**! Check input string */
	if(0 == length)
	{
		/**! Notify error to console */
		serial_send("Error: empty string\r\n", strlen("Error: empty string\r\n"));
		/**! Exit */
		return;
	}
	/**! Write indicated string to console */
	serial_send(string, length);
}

/**
 * @brief get a string from console terminal
 * @details get a string from console terminal via serial port
 * @param[in] NA
 * @param[out] NA
 * @return buffer address of gotten string
 */
char* console_getstring(int* length)
{
	char* buffer     = NULL;
	volatile int len = 0;
	/**! Check length memory allocation */
	if (NULL == length)
	{
		/**! NULL memory of length */
		return buffer;
	}
	char data[128] = {0};
	char ch = 0;
	*length = 0;
	/**! Check console bus */
	if (CONSOLE_UNINITIALIZED == console_status)
	{
		/**! Initializing log interface */
//		hal_com_init(&console_obj);
		console_obj.data = &console_buff;
		console_status = CONSOLE_INITIALIZED;
	}
	/**! Start getting a string */
	do
	{
		ch = 0;
		/**! Read single character */
		serial_get(&ch, 1);
		/**! Write back to console */
		serial_send(&ch, 1);
		/**! Verify input character */
		if ((CR == ch) || (LF == ch))
		{
			break;
		}
		else if (((DIGIT_0 <= ch) && (DIGIT_9 >= ch)) \
				|| ('_' == ch)                        \
				|| (',' == ch))
		{
			data[len++] = ch;
		}
		else if (((LOWER_A <= ch) && (LOWER_Z >= ch))
			  || ((UPPER_A <= ch) && (UPPER_Z >= ch)))
		{
			data[len++] = upper(ch);
		}
		else if ((BACKSPACE == ch) || (LEFT_ARROW == ch))
		{
			if (BACKSPACE == ch)
			{
				if(len != 0)
					data[len--] = 0;
			}
		}
		else if (RIGHT_ARROW == ch)
		{
			len++;
		}
		else 
		{
			/**! Do nothing */
		}
	} while (len < 128);

	/**! Validate and return string */
	if ((0 < len) && (len < 128))
	{
		/**! Provide returned buffer */
		buffer = malloc(len + 1);
		/**! Verify memory allocation */
		if (NULL == buffer)
		{
			return buffer;
		}
		else
		{
			/**! Identify data length */
			*length = len;
		}
		//! Clear buffer
		memset(buffer, 0x00, len + 1);
		/**! Copy data to returned buffer */
		strncpy(buffer, data, len);
	}
	/*************************************************************************/
	/**< Return data >*/
	return buffer;
}

/**
 * @brief get a fulfill a buffer
 * @details get fulfull a indicated buffer
 * @param[in] ptr	: buffer address
 * @param[in] len	: number of bytes
 * @param[out] NA
 * @return number of read bytes
 */
LOCAL int serial_get(char *ptr, int len)
{
	volatile int CharCnt = 0x00;

	while(CharCnt < len)
	{
		*(console_obj.data) = 0x00;
		while (COM_OK != hal_com_readbyte(&console_obj));
		ptr[CharCnt] = *((char*)console_obj.data);
		*(console_obj.data) = 0;
		CharCnt++;
	}
	return CharCnt;
}

/**
 * @brief send whole buffer
 * @details send whole indicated buffer
 * @param[in] ptr	: buffer address
 * @param[in] len	: number of bytes
 * @param[out] NA
 * @return number of sent bytes
 */
LOCAL int serial_send(char *ptr, int len)
{
	volatile int n;
	for (n = 0; n < len; n++)
	{
		*(console_obj.data) = ptr[n];
		while (COM_TIMEOUT == hal_com_sendbyte(&console_obj));
		*(console_obj.data) = 0x00;
	}
	return n;
}

/**
 * @brief Convert any character to its upper case
 * @details Convert any character to its upper case
 * @param[in] ch	: indicated character that would be converted
 * @param[out] NA
 * @return 	upper character if input is readable character
 * 			0               if input is unreadable character
 */
LOCAL char upper(char ch)
{
	if ((UPPER_A <= ch) && (UPPER_Z >= ch))
		return ch;
	else if ((LOWER_A <= ch) && (LOWER_Z >= ch))
		return (ch + (UPPER_A - LOWER_A));
	else
		return 0;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
