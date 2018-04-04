/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    assert.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    June-26-2015
 * @brief   This file contains:
 *              + The assert function is used for validate param of APIs
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <std_type.h>
#include "assert.h"

/* Exported define -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#ifdef _USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *func, uint32_t line)
{
	/* User can add his own implementation to report the file name and
	line number,
	ex: printf("Wrong parameters value: file %s on line %d\n\r", file, line)
	*/
	/* Disable interrupt */
	__asm volatile ("CPSID i\n"
					"DSB\n"
					"ISB");
	/* Assert */
	printf("[%s: %d] %s !\r\n", func, (unsigned int)line, __FUNCTION__);
	/* Infinite loop */
	while (1);
}

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_malloc error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void *malloc_failed(uint8_t *func, uint32_t line)
{
	/* Disable interrupt */
	__asm volatile ("CPSID i\n"
					"DSB\n"
					"ISB");
	/* Assert */
	printf("[%s: %d] %s !\r\n", func, (unsigned int)line, __FUNCTION__);
	/* Infinite loop */
	while (1);
}
#endif

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
