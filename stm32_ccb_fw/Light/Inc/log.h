/********************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOG_H
#define __LOG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "CCBConfig.h"
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include "types.h"
/* Private define ------------------------------------------------------------*/

#ifndef NO_DEBUG_COLOR
#define DEBUG_HEADER()             printf("\e[32m[DEBUG: %s] ", __FUNCTION__)
#define INFO_HEADER()              printf("\e[33m[INFO: %s] ", __FUNCTION__)
#define WARNING_HEADER()           printf("\e[35m[WARNING: %s] ", __FUNCTION__)
#define ERROR_HEADER()             printf("\e[31m[ERROR: %s] ", __FUNCTION__)
#define NOT_IMPLEMENTED_HEADER()   printf("\e[36m[NOT IMPLEMENTED: %s] ", __FUNCTION__)
#else
#define DEBUG_HEADER()             printf("[DEBUG: %s] ", __FUNCTION__)
#define INFO_HEADER()              printf("[INFO: %s] ", __FUNCTION__)
#define WARNING_HEADER()           printf("[WARNING: %s] ", __FUNCTION__)
#define ERROR_HEADER()             printf("[ERROR: %s] ", __FUNCTION__)
#define NOT_IMPLEMENTED_HEADER()   printf("[NOT IMPLEMENTED: %s] ", __FUNCTION__)
#endif

#ifndef NO_DEBUG_COLOR
#define DEBUG_ENDL()               printf("\e[39m\n\r")
#else
#define DEBUG_ENDL()               printf("\n\r")
#endif

/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
typedef enum
{
	LOG_LEVEL_ALL		=	0x7F,
	LOG_LEVEL_TIME		=	0x40,
	LOG_LEVEL_FPGA		=	0x20,
	LOG_LEVEL_DEBUG		=	0x10,
	LOG_LEVEL_INFO		=	0x08,
	LOG_LEVEL_WARNING	=	0x04,
	LOG_LEVEL_ERROR		=	0x02,
	LOG_LEVEL_FATAL		=	0x01,
	LOG_LEVEL_NOLOG		=	0x00
} CCB_LOG_LEVEL;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
extern volatile UInt8 log_level;
#define log_debug(...) 	do {									\
							if ((log_level & LOG_LEVEL_DEBUG)){	\
								DEBUG_HEADER();					\
								printf(__VA_ARGS__);			\
								DEBUG_ENDL();					\
						}										\
						} while (0);

#define log_info(...) 	do {									\
							if ((log_level & LOG_LEVEL_INFO)){	\
								INFO_HEADER();					\
								printf(__VA_ARGS__);			\
								DEBUG_ENDL();					\
							}									\
						} while (0);

#define log_warning(...) 	do {										\
								if ((log_level & LOG_LEVEL_WARNING)){	\
									WARNING_HEADER();					\
									printf(__VA_ARGS__);				\
									DEBUG_ENDL();						\
								}										\
							} while (0);


#define log_error(...) 		do {										\
								if ((log_level & LOG_LEVEL_ERROR)){		\
									ERROR_HEADER();						\
									printf(__VA_ARGS__);				\
									DEBUG_ENDL();						\
								}										\
							} while (0);

#define log_not_implemented(...)	do {										\
										if ((log_level & LOG_LEVEL_ERROR)){		\
											NOT_IMPLEMENTED_HEADER();			\
											printf(__VA_ARGS__);				\
											DEBUG_ENDL();						\
										}										\
									} while (0);

#define log_console(...)	do {											\
								if ((log_level & LOG_LEVEL_FATAL)){			\
									printf(__VA_ARGS__);					\
								}											\
							} while (0)
#define log_fpga(...)		do {											\
								if ((log_level & LOG_LEVEL_FPGA)){			\
									printf(__VA_ARGS__);					\
								}											\
							} while (0)
#define log_time(...)		do {											\
								if ((log_level & LOG_LEVEL_TIME)){			\
									printf(__VA_ARGS__);					\
								}											\
							} while (0)
#ifndef NO_DEBUG
#define log_printf(...) do {											\
							if ((log_level & LOG_LEVEL_INFO)){			\
								printf(__VA_ARGS__);					\
                                DEBUG_ENDL();   						\
							}											\
						} while (0)
#else
#define log_printf(...)
#endif

#define		ENTER_FUNC		log_debug("Entered %s ", __FUNCTION__)
#define		EXIT_FUNC		log_debug("Exiting %s ", __FUNCTION__)


/* Exported functions ------------------------------------------------------- */

/**
 *  \brief Function to init log service
 *
 *  \par Header File:
 *  log.h
 *
 *  \par Description:
 *  This function initializes a logging handler to print out debug messages
 */
void vLogInit( void );

#ifdef __cplusplus
}
#endif

#endif /* __LOG_H */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE****/
