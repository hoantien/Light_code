/*
    FreeRTOS V6.1.0 - Copyright (C) 2010 Real Time Engineers Ltd.

    This file is part of the FreeRTOS distribution.

    This is example #1 from the book Using the FreeRTOS Real Time Kernel - A Practical Guide

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* Includes ------------------------------------------------------------------*/
/* System includes */
#include <stdio.h>
#include <stdint.h>

/* CMSIS / hardware includes */
#include "main.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* CCB drivers includes. */
#include "hal_i2c_ex.h"
#include "hal_cci.h"
#include "drv_spi.h"
#include "cam_ctrl.h"
#include "af_ctrl.h"
#include "log.h"
#include "drv_piezo.h"
#include "hal_flash_ex.h"
#include "drv_cpld.h"
#include "mems.h"
extern volatile ccb_cmd_base_t * cam_m_status;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		(0x3fffff)

/* Private variables ---------------------------------------------------------*/
/* Private functions prototype -----------------------------------------------*/
/* Private task functions prototype ------------------------------------------*/
volatile uint8_t cpld_ver_major, cpld_ver_minor;
void vSystemCounter(void *pvParameters);

unsigned int stm_ver;

/* Main function -------------------------------------------------------------*/
int main(void)
{
	/* System Initialization. */
	PlatformInit();
	/* Create one of the two tasks. */
	xTaskCreate(	vSystemCounter,
					(const signed char * const)"SysCounter",
					200,
					NULL,
					tskIDLE_PRIORITY + 3,
					NULL);

	/* Create one of the two tasks. */
	xTaskCreate(	vCamController,
					(const signed char * const)"CamCtlr",
					1000,
					NULL,
					1,
					NULL);

	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	while(1);
}


static const char __LIGHT_CCB_TEXT[] =
"\r\n--------------------------------------------\r\n"
" $$                                        \r\n"
" $$                    $$            $     \r\n"
" $$                    $$           $$     \r\n"
" $$                    $$           $$     \r\n"
" $$            $$$     $$           $$$$$  \r\n"
" $$    $$     $ $$$    $$  $$$      $$$$$  \r\n"
" $$    $$    $   $$$   $$ $ $$$     $$     \r\n"
" $$    $$   $$    $$   $$$   $$$    $$     \r\n"
" $$    $$   $$    $$   $$     $$    $$     \r\n"
" $$    $$    $$  $$    $$     $$    $$$ $  \r\n"
" $$    $$     $$$$     $$     $$     $$$   \r\n"
"                                           \r\n"
"                  $ $$                     \r\n"
"                $$   $$                    \r\n"
"                $$   $$                    \r\n"
"                  $$$                      \r\n"
"\r\n"
"-------------------------------------------\r\n\n\n";
void vSystemCounter(void *pvParameters)
{
	(void)pvParameters;
	unsigned int build_id=0;
	char  temp[4];
	printf("============================================\n\r");
    printf(__LIGHT_CCB_TEXT);
	//printf("       LightCCB firmware\n\r");

#ifdef BUILD_ID
	sprintf(temp,"%d",BUILD_ID);
	build_id = atoi(temp);
#ifdef BOARD_VERSION_P1
	stm_ver = 0x08 << 24 | (build_id & 0x00ffffff);
#else
	stm_ver = 0x0D << 24 | (build_id & 0x00ffffff);
#endif
	printf("  Build ID: %08x \n\r", stm_ver);
#endif
#ifdef BUILD_TIME
	printf("  Build time: %s \n\r", BUILD_TIME);
#endif
	cpld_get_version(&cpld_ver_major, &cpld_ver_minor);
	printf("  CPLD version: %02d.%02d \n\r", cpld_ver_major, cpld_ver_minor);
	printf("============================================\n\r\n\r");
	volatile unsigned int counter = 0;
	log_debug("system clock %d\n", (int)SystemCoreClock);
	memset(CCB_ASIC_MEM_RD, 0x1, 8192);
	Temp_Init();
	/* As per most tasks, this task is implemented in an infinite loop. */

	AF_control_Config();
	while(1)
	{
		//log_printf("SC: %08x\r", counter);
		//read_temp_sensor();
		/* Delay for a period. */
		vTaskDelay(1000);
		counter++;
//		if(counter % 5 == 0)
//			ReadTemperature();
	}
}

/* Global functions ----------------------------------------------------------*/
void vApplicationMallocFailedHook(void)
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining - and
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. */
	while(1);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	(void)pxTask;
	(void)pcTaskName;
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation and will only be performed if configCHECK_FOR_STACK_OVERFLOW
	is set to either 1 or 2 in FreeRTOSConfig.h. */
	while(1);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* This example does not use the idle hook to perform any processing.  The
	idle hook will only be called if configUSE_IDLE_HOOK is set to 1 in 
	FreeRTOSConfig.h. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* This example does not use the tick hook to perform any processing.   The
	tick hook will only be called if configUSE_TICK_HOOK is set to 1 in
	FreeRTOSConfig.h. */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* func, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\n\r", file, line) */
  log_printf("\e[31m[%s(%d)] Wrong parameters value\e[39m\n\r", func, (int)line);
  /* Infinite loop */
  while (1)
  {
  }
}
#endif
