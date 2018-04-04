/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    qc_common.c
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

/* Includes ------------------------------------------------------------------*/
#include "qc_common.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CPU_FREQ_MHZ 50
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Static functions ----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief Handler parser
 * @detail To parse the handler based on the input command
 * @param[in] cmd		: input command
 * @param[in] map_table	: traced table
 * @param[out] NA
 * @return -1	: invalid command
 *          Other: index of command in map table
 */
int handler_parser(char* cmd, it_map_t map_table[])
{
	int i;
	for (i = 0; map_table[i].handler != NULL; i++)
	{
		if (0 == strcmp(cmd, map_table[i].cmd))
		{
			return i;
		}
	}
	return -1;
}

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
int params_parser(char* str, char** out, int* range)
{
	if (NULL == str) return -1;
	if (NULL == out) return -1;

	/**! Get the maximum size of string */
	uint16_t length = strlen(str);
	uint16_t chars_num = 0;
	uint16_t counter = 0;
	char* ptr = NULL;
	
	/**! Verify the string's length */
	if (0 == length) return -1;

	/**! Do parsing */
	for (counter = 0, ptr = str, *range = 0, \
			        chars_num = 0; counter < length; counter++)
	{
		/**! Meet separator or last character */
		if ((',' == str[counter]) || (counter == (length - 1)))
		{			
			/**! Get number of characters */
			if (counter == (length - 1))
			{
				chars_num = &str[counter] - ptr + 1;
			}
			else
			{
				chars_num = &str[counter] - ptr;
			}

			/**! Give memory to store current parameter */
			out[*range] = malloc(chars_num + 1);
			if (NULL == out[*range])
			{
				for (int i = 0; i < *range; i++)
					free(out[i]);
				return -1;
			}

			/**! Copy parameter */
			strncpy(out[*range], ptr, chars_num);

			/**! Insert string terminator */
			(out[*range])[chars_num] = '\0';

			if (counter != (length - 1))
			{
				/**! Point start address to the next character */
				ptr = &str[counter] + 1;
				/**! Reset number of characters */
				chars_num = 0;
			}
			/**! Update number of parameters */
			(*range) += 1;
		}
	}
	/**! Indicate success */
	return 0;
}

/**
 * @brief To make a delay in number of ms
 * @detail To make a delay in number of ms
 * @param[in] 	ms	: number of ms
 * @param[out] 	NA
 * @return 		NA
 */
void _delay_ms(uint32_t ms)
{
	volatile uint32_t nop_num = (ms * CPU_FREQ_MHZ * 1000)/6;

	while(--nop_num);
}

/**
 * @brief To make a delay in number of us
 * @detail To make a delay in number of us
 * @param[in] 	us	: number of us
 * @param[out] 	NA
 * @return 		NA
 */
void _delay_us(uint32_t us)
{
	volatile uint32_t nop_num = (us * CPU_FREQ_MHZ)/6;

	while(--nop_num);
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
