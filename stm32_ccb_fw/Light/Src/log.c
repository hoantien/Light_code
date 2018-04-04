/**
  ******************************************************************************
  * @file    Log.c
  * @author  Infonam Embedded Team
  * @version V1.1.0
  * @date    05-Mar-2015
  * @brief   This file provides set of firmware functions to USART log
  * 		 on CCB Light Board.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* Library includes. */
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Driver includes. */
#include "platform.h"
#include "log.h"
#include "hal_comport.h"
/* Exported variables --------------------------------------------------------*/
volatile UInt8 log_level = LOG_LEVEL_INFO | LOG_LEVEL_WARNING \
		| LOG_LEVEL_ERROR | LOG_LEVEL_FATAL;
/* Private typedef -----------------------------------------------------------*/
/** Required for proper compilation. */
struct _reent r = {0, (FILE *) 0, (FILE *) 1, (FILE *) 0};
/* Private define ------------------------------------------------------------*/
#define mainLOG_TASK_PRIORITY	(tskIDLE_PRIORITY+4)
// Total buffer size for all debug messages.
#define LOG_QUEUE_SIZE			1024
#define MAX_STRING_SIZE    	 	64
/* Private macro -------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
xQueueHandle xLogQueue;

/* Private function prototypes -----------------------------------------------*/
STATIC void vLogInitQueue( void );
STATIC void vLogTask(void *pvParameters );

/* Exported functions --------------------------------------------------------*/
void vLogInit(void)
{
#ifdef LOG_LEVEL
	/* Default LOG_LEVEL in the Makefile is respected */
	log_level = LOG_LEVEL;
#endif
	vLogInitQueue();
	xTaskCreate((pdTASK_CODE)vLogTask,
				(const signed char * const)"Log",
				(unsigned short)configMINIMAL_STACK_SIZE,
				NULL,
				(unsigned portBASE_TYPE)mainLOG_TASK_PRIORITY,
				(xTaskHandle)NULL);

	// Clear the screen.
	 printf("\e[2J\e[H");
}


/* Private function ----------------------------------------------------------*/
void vLogInitQueue(void)
{
	xLogQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(char));
}

void vLogTask(void *pvParameters)
{
	char ch;
	portBASE_TYPE xStatus;

	/* The parameters are not used. */
	(void) pvParameters;

	while(1)
	{
		// As long as there are characters in the queue fifo this code should
		// pop them out and send them as quick as possible out the UART.
		if((HAL_COM_GetFlagStatus(COM1, COM_FLAG_TXE) != 0)
		&& (HAL_COM_GetFlagStatus(COM2, COM_FLAG_TXE) != 0))
		{
			// We don't want to block forever - need to check on Rx too.
			xStatus = xQueueReceive(xLogQueue, &ch, 10 / portTICK_RATE_MS);
			if(xStatus == pdPASS)
			{
				HAL_COM_SendData(COM1, ch);
				HAL_COM_SendData(COM2, ch);
			}
		}

		taskYIELD();
	}
}

/**
 * @brief  Writes a character inside the given string. Returns 1.
 *
 * @param  pStr	Storage string.
 * @param  c    Character to write.
 */
signed int PutChar(char *pStr, char c)
{
    *pStr = c;
    return 1;
}


/**
 * @brief  Writes a string inside the given string.
 *
 * @param  pStr     Storage string.
 * @param  pSource  Source string.
 * @return  The size of the written
 */
signed int PutString(char *pStr, const char *pSource)
{
    signed int num = 0;

    while (*pSource != 0) {

        *pStr++ = *pSource++;
        num++;
    }

    return num;
}


/**
 * @brief  Writes an unsigned int inside the given string, using the provided fill &
 *         width parameters.
 *
 * @param  pStr  Storage string.
 * @param  fill  Fill character.
 * @param  width  Minimum integer width.
 * @param  value  Integer value.
 */
signed int PutUnsignedInt(
    char *pStr,
    char fill,
    signed int width,
    unsigned int value)
{
    signed int num = 0;

    /* Take current digit into account when calculating width */
    width--;

    /* Recursively write upper digits */
    if ((value / 10) > 0) {

        num = PutUnsignedInt(pStr, fill, width, value / 10);
        pStr += num;
    }

    /* Write filler characters */
    else {

        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }
    }

    /* Write lower digit */
    num += PutChar(pStr, (value % 10) + '0');

    return num;
}

/**
 * @brief  Writes a signed int inside the given string, using the provided fill & width
 *         parameters.
 *
 * @param pStr   Storage string.
 * @param fill   Fill character.
 * @param width  Minimum integer width.
 * @param value  Signed integer value.
 */
signed int PutSignedInt(
    char *pStr,
    char fill,
    signed int width,
    signed int value)
{
    signed int num = 0;
    unsigned int absolute;

    /* Compute absolute value */
    if (value < 0) {

        absolute = -value;
    }
    else {

        absolute = value;
    }

    /* Take current digit into account when calculating width */
    width--;

    /* Recursively write upper digits */
    if ((absolute / 10) > 0) {

        if (value < 0) {

            num = PutSignedInt(pStr, fill, width, -(absolute / 10));
        }
        else {

            num = PutSignedInt(pStr, fill, width, absolute / 10);
        }
        pStr += num;
    }
    else {

        /* Reserve space for sign */
        if (value < 0) {

            width--;
        }

        /* Write filler characters */
        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }

        /* Write sign */
        if (value < 0) {

            num += PutChar(pStr, '-');
            pStr++;
        }
    }

    /* Write lower digit */
    num += PutChar(pStr, (absolute % 10) + '0');

    return num;
}


/**
 * @brief  Simplified implementation for outputting a floating point value
 *         inside the given string, using the provided fill & width
 *         parameters.
 *
 * @param pStr      Storage string
 * @param fill      Fill character
 * @param widthInt  Minimum integer width
 * @param widthFrac Minimum integer width
 * @param value     Float value
 */
signed int PutDouble(
    char       *pStr,
    char        fill,
    signed int  widthInt,
    signed int  widthFrac,
    double      value)
{
    signed int num = 0;
    double absolute;
    double frac;

    if (widthFrac == 0) widthFrac = 8;

    /* Compute absolute value */
    absolute = (value < 0) ? -value : value;

    /* Start by printing the integer portion */
    num = PutSignedInt(pStr, fill, widthInt, (int) value);
    frac = absolute - (int) absolute;
    pStr += num;
    if (frac == 0.0) return num;

    /* Output the decimal point. */
    num += PutChar(pStr, '.');
    pStr++;

    /* Write the fractional digits. */
    while(widthFrac)
    {
        int digit;

        if (frac == 0) break;

        frac = frac * 10;
        digit = (int)frac;
        frac -= digit;

        num += PutChar(pStr, digit + '0');
        pStr++;

        widthFrac--;
    }

    return num;
}


/**
 * @brief  Writes an hexadecimal value into a string, using the given fill, width &
 *         capital parameters.
 *
 * @param pStr   Storage string.
 * @param fill   Fill character.
 * @param width  Minimum integer width.
 * @param maj    Indicates if the letters must be printed in lower- or upper-case.
 * @param value  Hexadecimal value.
 *
 * @return  The number of char written
 */
signed int PutHexa(
    char *pStr,
    char fill,
    signed int width,
    unsigned char maj,
    unsigned int value)
{
    signed int num = 0;

    /* Decrement width */
    width--;

    /* Recursively output upper digits */
    if ((value >> 4) > 0) {

        num += PutHexa(pStr, fill, width, maj, value >> 4);
        pStr += num;
    }
    /* Write filler chars */
    else {

        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }
    }

    /* Write current digit */
    if ((value & 0xF) < 10) {

        PutChar(pStr, (value & 0xF) + '0');
    }
    else if (maj) {

        PutChar(pStr, (value & 0xF) - 10 + 'A');
    }
    else {

        PutChar(pStr, (value & 0xF) - 10 + 'a');
    }
    num++;

    return num;
}

/* Global Functions ----------------------------------------------------------- */
/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pStr    Destination string.
 * @param length  Length of Destination string.
 * @param pFormat Format string.
 * @param ap      Argument list.
 *
 * @return  The number of characters written.
 */
signed int vsnprintf(char *pStr, size_t length, const char *pFormat, va_list ap)
{
    char          fill;
    unsigned char width;
    unsigned char widthFrac = 0;
    signed int    num = 0;
    signed int    size = 0;

    /* Clear the string */
    if (pStr)
    {

        *pStr = 0;
    }

    /* Phase string */
    while (*pFormat != 0 && size < length)
    {

        /* Normal character */
        if (*pFormat != '%')
        {

            *pStr++ = *pFormat++;
            size++;
        }
        /* Escaped '%' */
        else if (*(pFormat+1) == '%')
        {

            *pStr++ = '%';
            pFormat += 2;
            size++;
        }
        /* Token delimiter */
        else
        {

            fill = ' ';
            width = 0;
            widthFrac = 0;
            pFormat++;

            /* Parse filler */
            if (*pFormat == '0')
            {

                fill = '0';
                pFormat++;
            }

            /* Parse width */
            while ((*pFormat >= '0') && (*pFormat <= '9'))
            {

                width = (width*10) + *pFormat-'0';
                pFormat++;
            }

            if(*pFormat == '.')
            {
                pFormat++; // Skip past the '.'

                /* Parse fractional width */
                while ((*pFormat >= '0') && (*pFormat <= '9'))
                {

                    widthFrac = (widthFrac*10) + *pFormat-'0';
                    pFormat++;
                }
            }

            /* Check if there is enough space */
            if (size + width > length)
            {

                width = length - size;
                widthFrac = 0;
            }

            /* Parse type */
            switch (*pFormat)
            {
				case 'd':
				case 'i': num = PutSignedInt(pStr, fill, width, va_arg(ap, signed int)); break;
				case 'u': fill = '0'; num = PutUnsignedInt(pStr, fill, width, va_arg(ap, unsigned int)); break;
				case 'x': fill = '0'; num = PutHexa(pStr, fill, width, 0, va_arg(ap, unsigned int)); break;
				case 'X': fill = '0'; num = PutHexa(pStr, fill, width, 1, va_arg(ap, unsigned int)); break;
				case 's': num = PutString(pStr, va_arg(ap, char *)); break;
				case 'c': num = PutChar(pStr, va_arg(ap, unsigned int)); break;
				case 'f': num = PutDouble(pStr, fill, width, widthFrac, va_arg(ap, double)); break;
				default:
					return EOF;
            }

            pFormat++;
            pStr += num;
            size += num;
        }
    }

    /* NULL-terminated (final \0 is not counted) */
    if (size < length)
    {

        *pStr = 0;
    }
    else
    {

        *(--pStr) = 0;
        size--;
    }

    return size;
}


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pStr    Destination string.
 * @param length  Length of Destination string.
 * @param pFormat Format string.
 * @param ...     Other arguments
 *
 * @return  The number of characters written.
 */
signed int snprintf(char *pString, size_t length, const char *pFormat, ...)
{
    va_list    ap;
    signed int rc;

    va_start(ap, pFormat);
    rc = vsnprintf(pString, length, pFormat, ap);
    va_end(ap);

    return rc;
}


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pString  Destination string.
 * @param length   Length of Destination string.
 * @param pFormat  Format string.
 * @param ap       Argument list.
 *
 * @return  The number of characters written.
 */
signed int vsprintf(char *pString, const char *pFormat, va_list ap)
{
   return vsnprintf(pString, MAX_STRING_SIZE, pFormat, ap);
}

/**
 * @brief  Outputs a formatted string on the given stream. Format arguments are given
 *         in a va_list instance.
 *
 * @param pStream  Output stream.
 * @param pFormat  Format string
 * @param ap       Argument list.
 */
signed int vfprintf(FILE *pStream, const char *pFormat, va_list ap)
{
    char pStr[MAX_STRING_SIZE];
    char pError[] = "stdio.c: increase MAX_STRING_SIZE\n\r";

    /* Write formatted string in buffer */
    if (vsprintf(pStr, pFormat, ap) >= MAX_STRING_SIZE)
    {

        fputs(pError, stderr);
        return EOF;
    }

    /* Display string */
    return fputs(pStr, pStream);
}


/**
 * @brief  Outputs a formatted string on the DBGU stream. Format arguments are given
 *         in a va_list instance.
 *
 * @param pFormat  Format string.
 * @param ap  Argument list.
 */
signed int vprintf(const char *pFormat, va_list ap)
{
    return vfprintf(stdout, pFormat, ap);
}


/**
 * @brief  Outputs a formatted string on the given stream, using a variable
 *         number of arguments.
 *
 * @param pStream  Output stream.
 * @param pFormat  Format string.
 */
signed int fprintf(FILE *pStream, const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    /* Forward call to vfprintf */
    va_start(ap, pFormat);
    result = vfprintf(pStream, pFormat, ap);
    va_end(ap);

    return result;
}


/**
 * @brief  Outputs a formatted string on the DBGU stream, using a variable number of
 *         arguments.
 *
 * @param  pFormat  Format string.
 */
signed int printf(const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    /* Forward call to vprintf */
    va_start(ap, pFormat);
    result = vprintf(pFormat, ap);
    va_end(ap);

    return result;
}

/**
 * @brief  Writes a formatted string inside another string.
 *
 * @param pStr     Storage string.
 * @param pFormat  Format string.
 */
signed int sprintf(char *pStr, const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    // Forward call to vsprintf
    va_start(ap, pFormat);
    result = vsprintf(pStr, pFormat, ap);
    va_end(ap);

    return result;
}


/**
 * @brief  Outputs a string on stdout.
 *
 * @param pStr  String to output.
 */
signed int puts(const char *pStr)
{
    return fputs(pStr, stdout);
}

/**
 * @brief  Implementation of fputs using the DBGU as the standard output. Required
 *         for printf().
 *
 * @param pStr     String to write.
 * @param pStream  Output stream.
 *
 * @return  Number of characters written if successful, or -1 if the output
 *          stream is not stdout or stderr.
 */
signed int fputs(const char *pStr, FILE *pStream) {

    signed int num = 0;
	portBASE_TYPE xStatus;

	/* Print the string, suspending the scheduler as method of mutual
	exclusion. */
	vTaskSuspendAll();

    while (*pStr != 0)
    {
    	if ((pStream == stdout) || (pStream == stderr))
    	{
    		xStatus = xQueueSendToBack(xLogQueue, pStr, 0);
    		if (xStatus == errQUEUE_FULL) break;
		}
		else
		{
			return EOF;
		}
        num++;
        pStr++;
    }

    /* Resume the scheduler as method of mutual exclusion. */
	xTaskResumeAll();

    return num;
}

/*********** Portions COPYRIGHT 2015 Light. Co., Ltd.*****END OF FILE****/
