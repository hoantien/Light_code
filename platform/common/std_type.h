/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    std_type.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jun-26-2015
 * @brief   This file contains expand of the std_type
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STD_TYPES_H__
#define __STD_TYPES_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define __IO		volatile
#define __O			volatile
#define __I			volatile const
#define NULL_PTR	((void *)0)


#ifndef FALSE
	#define FALSE	0
#endif
#ifndef TRUE
	#define TRUE	1
#endif


#ifndef STD_OFF
	#define STD_OFF	0
#endif
#ifndef STD_ON
	#define STD_ON	1
#endif

#ifndef RESET
	#define RESET	0
#endif
#ifndef SET
	#define SET		1
#endif

#ifndef OFF
	#define OFF		0
#endif
#ifndef ON
	#define ON		1
#endif

#ifndef DISABLE
	#define DISABLE	0
#endif
#ifndef ENABLE
	#define ENABLE	1
#endif

#ifndef MIN
	#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
	#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef CLAMP
	#define CLAMP(a, min, max)	MIN(MAX(a, min), max)
#endif

#define BIT0		(1<<0)
#define BIT1		(1<<1)
#define BIT2		(1<<2)
#define BIT3		(1<<3)
#define BIT4		(1<<4)
#define BIT5		(1<<5)
#define BIT6		(1<<6)
#define BIT7		(1<<7)
#define BIT8		(1<<8)
#define BIT9		(1<<9)
#define BIT10		(1<<10)
#define BIT11		(1<<11)
#define BIT12		(1<<12)
#define BIT13		(1<<13)
#define BIT14		(1<<14)
#define BIT15		(1<<15)
#define BIT16		(1<<16)
#define BIT17		(1<<17)
#define BIT18		(1<<18)
#define BIT19		(1<<19)
#define BIT20		(1<<20)
#define BIT21		(1<<21)
#define BIT22		(1<<22)
#define BIT23		(1<<23)
#define BIT24		(1<<24)
#define BIT25		(1<<25)
#define BIT26		(1<<26)
#define BIT27		(1<<27)
#define BIT28		(1<<28)
#define BIT29		(1<<29)
#define BIT30		(1<<30)
#define BIT31		(1<<31)

#define SIZE_256	0x100

#define ARRAY_SIZE(array)	(sizeof(array)/sizeof(array[0]))

#define BYTETOBINARYPATTERN	"%d%d%d%d%d%d%d%d"

#define BYTETOBINARY(byte)  \
	(byte & 0x80 ? 1 : 0), \
	(byte & 0x40 ? 1 : 0), \
	(byte & 0x20 ? 1 : 0), \
	(byte & 0x10 ? 1 : 0), \
	(byte & 0x08 ? 1 : 0), \
	(byte & 0x04 ? 1 : 0), \
	(byte & 0x02 ? 1 : 0), \
	(byte & 0x01 ? 1 : 0)

#define swap16(x) (((x) >> 8) | ((x) << 8))
#define swap24(x) (uint32_t)(((x)>>16 | ((uint16_t)(x) & 0xFF00)|(x)<<16))
#define swap32(x) (swap16(x & 0xFFFF) << 16 | (uint16_t)swap16(x >> 16))

#define U16(p)	(uint16_t)(*(uint16_t *)&p)		/* Casting buffer to uint16_t */
#define U32(p)	(uint32_t)(*(uint32_t *)&p)		/* Casting buffer to uint32_t */

#define writel(addr, value)	*(volatile unsigned int  *)(addr) = value
#define readl(addr)			*(volatile unsigned int  *)(addr)

/* Exported typedef  ---------------------------------------------------------*/

#endif /* __STD_TYPES_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
