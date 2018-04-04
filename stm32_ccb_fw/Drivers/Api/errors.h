/**
  ******************************************************************************
  * \file    errors.h
  * \author  Infonam Embedded Team
  * \version V1.0.0
  * \date    18-Mar-2015
  * \brief   Errors Types definition
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ERRORS_H
#define __ERRORS_H

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * \brief  Error type enumeration
 */
typedef enum {
	ERROR_NONE = 0,					/**< The operation completed successfully */
	ERROR_OUT_OF_HANDLES = 1,
	ERROR_TIMEOUT = 2,				/**< The wait operation timed out */
	ERROR_HANDLE_INVALID = 3,		/**< The handle is invalid */
	ERROR_OUT_OF_MEMORY = 4,		/**< Not enough storage is available to complete this operation */
	ERROR_MEMORY_INVALID = 5,		/**< Invalid memory access */
	ERROR_SEMAPHORE_TIMEOUT = 6,	/**< The semaphore timeout period has expired */
	ERROR_BUSY = 7,					/**< The requested resource is in use */
	ERROR_INVALID_ARG = 8,
	ERROR_CALIBRATION_FAILURE = 9,
} Error_t;

// The following macro assumes the function has a local variable Error_t e
// and returns an error code Error_t.
#define CHECK_ERROR(x)         { e = (x); if (e) return(e); }
#define CHECK_ERROR_CLEANUP(x) { e = (x); if (e) goto fail; }


#ifdef __cplusplus
}
#endif

#endif /* __ERRORS_H */

/*********** Portions COPYRIGHT 2015 Light. Co., Ltd.*****END OF FILE****/
