/******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    startup.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-09-2016
 * @brief   This file contains first instructions after receiving control from
 *          bootup program. Define stack, set sp, and jump to C code in main()
 *
 ******************************************************************************/

extern int main(void);

extern unsigned long __etext;
extern unsigned long __data_start__;
extern unsigned long __data_end__;
extern unsigned long __bss_start__;
extern unsigned long __bss_end__;


void startup_main(void)
{
	unsigned long *pulSrc, *pulDest;
	/* Copy the data segment initializers from flash to SRAM. */
	pulSrc = &__etext;
	pulDest = &__data_start__;

	if(pulSrc != pulDest)
	{
		for(; pulDest < &__data_end__;)
			*pulDest++ = *pulSrc++;
	}

	/* Zero fill the bss segment. */
	for(pulDest = &__bss_start__; pulDest < &__bss_end__;)
		*pulDest++ = 0;
	/* Jump to main function */
	main();
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
