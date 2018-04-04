/**
  ******************************************************************************
  * @file    types.h
  * @author  Infonam Embedded Team
  * @version V1.0.0
  * @date    18-Mar-2015
  * @brief   Types definition
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TYPES_H
#define __TYPES_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Global types --------------------------------------------------------------*/
typedef uint32_t 	UInt32;
typedef uint64_t	UInt64;
typedef uint16_t 	UInt16;
typedef uint8_t 	UInt8;
typedef int32_t 	Int32;
typedef int16_t 	Int16;
typedef int8_t 		Int8;
typedef void 		(*FPtr)(void);
typedef uint8_t		Bool;
#define EFalse		0
#define ETrue		1

typedef volatile UInt32 VUInt32;
typedef volatile Int32 	VInt32;
typedef volatile UInt16	VUInt16;
typedef volatile Int16 	VInt16;
typedef volatile UInt8 	VUInt8;
typedef volatile Int8 	VInt8;

#define STATIC static

/* Handles -------------------------------------------------------------------*/
typedef void *Handler_Agr_t;
typedef void (*Handler_t)(Handler_Agr_t arg);
typedef void (*Result_Handler_t)(Handler_Agr_t arg, UInt32 result);
typedef Handler_t Device_t;

/* Global macro --------------------------------------------------------------*/
#define ARRAY_COUNT(array)  (sizeof(array)/sizeof(array[0]))
#define CPUReg32(addr)      *((VUInt32 *)(addr))
#define CPUReg16(addr)      *((VUInt16 *)(addr))
#define CPUReg8(addr)       *((VUInt8 *)(addr))

#ifdef __cplusplus
}
#endif

#endif /* __TYPES_H */

/*********** Portions COPYRIGHT 2015 Light. Co., Ltd.*****END OF FILE****/
