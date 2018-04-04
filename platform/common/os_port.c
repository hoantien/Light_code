/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    os_assert.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of the os_assert
 *
 ******************************************************************************/
#include "os.h"
#include "log.h"
/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook(void)
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining - and
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. */
	printf("\r\n%s\r\n", __FUNCTION__);
	__disable_irq();
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
	printf("\r\n%s: %08X %s\r\n", __FUNCTION__, (unsigned int)pxTask, pcTaskName);
	__disable_irq();
	while(1);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* This example does not use the idle hook to perform any processing. The
	idle hook will only be called if configUSE_IDLE_HOOK is set to 1 in
	FreeRTOSConfig.h. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* This example does not use the tick hook to perform any processing. The
	tick hook will only be called if configUSE_TICK_HOOK is set to 1 in
	FreeRTOSConfig.h. */
}
/*-----------------------------------------------------------*/
void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used. If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function.
*/
	while(1);
}
/*-----------------------------------------------------------*/
void hard_fault_handler(void)
{
	while(1);
}
/*-----------------------------------------------------------*/
void bus_fault_handler(void)
{
	while(1);
}
/*-----------------------------------------------------------*/
void usage_fault_handler(void)
{
	while(1);
}
/*-----------------------------------------------------------*/
void mem_manage_handler(void)
{
	while(1);
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
