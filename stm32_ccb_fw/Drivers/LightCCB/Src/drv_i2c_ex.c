/**
  ******************************************************************************
  * @file    drv_i2c_ex.c
  * @author  Infonam Embedded Team
  * @version V1.1.0
  * @date    05-Mar-2015
  * @brief   This file provides set of firmware functions to I2C driver.
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

#include "drv_i2c_ex.h"
#include "hal_i2c_ex.h"
#include "log.h"

/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2CEx_TASK_PRIORITY			(tskIDLE_PRIORITY+4)
// Total buffer size for all debug messages.
#define I2CEx_QUEUE_SIZE			32

/* I2CEx Mode Selected */
#define I2CEx_MODE_CONFIG   		0x00
#define I2CEx_MODE_TRANSMITTER      0x01
#define I2CEx_MODE_RECEIVER			0x02

#define I2CEx_STATE_IDLE			0
#define I2CEx_STATE_WAIT			1
#define I2CEx_STATE_CFG				2
#define I2CEx_STATE_TX				3
#define I2CEx_STATE_RX				4

#define I2CEx_STATUS_BUSY			0
#define I2CEx_STATUS_CFG_CPLT		1
#define I2CEx_STATUS_TX_CPLT		2
#define I2CEx_STATUS_RX_CPLT		3
#define I2CEx_STATUS_ERROR			4

#define I2CEx_I2C_CLOCKSPEED        400000


#define LOG_I2CEx 				0
#define BUGFIX_RESET_I2CEX 		1
#define BUGFIX_ANALOG_FILTER 	0
#define BUGFIX_DIGITAL_FILTER	0

/* Private variables ---------------------------------------------------------*/
xQueueHandle xI2CExQueue;
__IO uint32_t I2CEx_TxIdx = 0x00;
__IO uint32_t I2CEx_RxIdx = 0x00;
__IO uint8_t I2CEx_Mode = I2CEx_MODE_CONFIG;
__IO uint32_t NumberOfByteToReceive = 0;
I2CEx_Msg_t I2CEx_Msg;
__IO uint32_t I2CEx_Event;
__IO uint8_t I2CEx_State;
__IO uint8_t I2CEx_Status;
I2CEx_Ch_t Current_Ch = 0xFF;

/* Private function prototypes -----------------------------------------------*/
STATIC void I2CEx_InitQueue( void );
STATIC void I2CEx_Task(void *pvParameters );
STATIC void I2CEx_InitPort(void);
STATIC void I2CEx_Config(void);
STATIC void I2CEx_Reset(void);


/* Exported functions --------------------------------------------------------*/
void HAL_I2CEx_Init(void)
{
	I2CEx_InitPort();
	I2CEx_Config();
	I2CEx_InitQueue();
	xTaskCreate((pdTASK_CODE)I2CEx_Task,
				(const signed char * const)"I2CEx",
				(unsigned short)configMINIMAL_STACK_SIZE,
				NULL,
				(unsigned portBASE_TYPE)I2CEx_TASK_PRIORITY,
				(xTaskHandle)NULL);
}

Error_t HAL_I2CEx_Transfer(I2CEx_Msg_t *msg)
{
	portBASE_TYPE xStatus;

	if ((msg->txbuf.len == 0) && (msg->rxbuf.len == 0))
	{
		log_error("Memory transfer invalid !");
		return ERROR_MEMORY_INVALID;
	}

	// log_debug("ch:%d  addr:0x%02x  tx:%d  rx:%d", msg->ch, msg->addr, (int)(msg->txbuf.len), (int)(msg->rxbuf.len));

	*msg->is_completed = 0;

	/* Suspending the scheduler as method of mutual
	exclusion. */
//	vTaskSuspendAll();

	xStatus = xQueueSendToBack(xI2CExQueue, msg, 0);
	if (xStatus == errQUEUE_FULL)
	{
		/* Resume the scheduler as method of mutual exclusion. */
//		xTaskResumeAll();
		log_error("The I2C transfer queue is full!");
		return ERROR_OUT_OF_MEMORY;
	}

    /* Resume the scheduler as method of mutual exclusion. */
//	xTaskResumeAll();

	return ERROR_NONE;
}

/* Private function ----------------------------------------------------------*/
void I2CEx_InitQueue(void)
{
	xI2CExQueue = xQueueCreate(I2CEx_QUEUE_SIZE, sizeof(I2CEx_Msg_t));
}

void I2CEx_Reset(void)
{
	GPIO_ResetBits(I2CEx_RST_GPIO_PORT , I2CEx_RST_PIN );
	vTaskDelay(10);
	GPIO_SetBits(I2CEx_RST_GPIO_PORT , I2CEx_RST_PIN );
	vTaskDelay(10);
}

#if LOG_I2CEx

void DumpI2CExLog(void);

#define I2CExLOGSIZE 100
uint8_t StateLog [I2CExLOGSIZE];
uint8_t StatusLog[I2CExLOGSIZE];
uint8_t ModeLog  [I2CExLOGSIZE];
UInt16 LogCount = 0;

// Note: Extra spaces help with alignment.
char *StateName[] =
{
	"IDLE",
	"WAIT",
	"CFG ",
	"TX  ",
	"RX  ",
};

char *StatusName[] =
{
	"BUSY    ",
	"CFG_CPLT",
	"TX_CPLT ",
	"RX_CPLT ",
	"ERROR   ",
};

char *ModeName[] =
{
	"CONFIG     ",
	"TRANSMITTER",
	"RECEIVER   ",
};

#define StateString(x)  (((x) <= I2CEx_STATE_RX     ) ? StateName [x] : "<unknown>")
#define StatusString(x) (((x) <= I2CEx_STATUS_ERROR ) ? StatusName[x] : "<unknown>")
#define ModeString(x)   (((x) <= I2CEx_MODE_RECEIVER) ? ModeName  [x] : "<unknown>")

void DumpI2CExLog()
{
	UInt16 i;

	for (i = 0; i < LogCount; i++)
	{
		if(i > 0 &&
		   StateLog [i] == StateLog[i-1] &&
		   StatusLog[i] == StatusLog[i-1] &&
		   ModeLog  [i] == ModeLog[i-1])
		{
			continue;
		}

		log_debug("  State: %s  Status: %s  Mode: %s",
				StateString(StateLog[i]),
				StatusString(StatusLog[i]),
				ModeString(ModeLog[i]));


		vTaskDelay(10);
		if ((i % 10) == 0)
		{
			taskYIELD();
		}

	}

}
#endif // #if LOG_I2CEx

void I2CEx_Task(void *pvParameters)
{
	portBASE_TYPE xStatus;
	I2CEx_Msg_t *msg = &I2CEx_Msg;

	I2CEx_State = I2CEx_STATE_IDLE;
	I2CEx_Status = I2CEx_STATUS_BUSY;

	/* The parameters are not used. */
	(void) pvParameters;

	I2CEx_Reset();

#if LOG_I2CEx
	LogCount = 0;
#endif // #if LOG_I2CEx

	while(1)
	{

#if LOG_I2Ex
		if (I2CEx_State != I2CEx_STATE_IDLE)
		{
			StateLog [LogCount] = I2CEx_State;
			StatusLog[LogCount] = I2CEx_Status;
			ModeLog  [LogCount] = I2CEx_Mode;
			LogCount++;
		}
		else
		{
			DumpI2CExLog();
			LogCount = 0;
		}
#endif // #if LOG_I2Ex

		switch (I2CEx_State)
		{
			case I2CEx_STATE_WAIT:
				/* Reset I2C whenever bus busy */
				switch(I2CEx_Status)
				{
					case I2CEx_STATUS_ERROR:
						/* Reset all I2C2 registers */
						I2C_SoftwareResetCmd(I2CEx, ENABLE);
						I2C_SoftwareResetCmd(I2CEx, DISABLE);
						I2CEx_Config();
						//log_error("No ACK from channel[%x] with address [%x]", msg->ch, msg->addr);
						*msg->is_completed = -1;
						I2CEx_State = I2CEx_STATE_IDLE;
						break;
					case I2CEx_STATUS_CFG_CPLT:
						if (msg->txbuf.len > 0)
							I2CEx_State = I2CEx_STATE_TX;
						else if (msg->rxbuf.len > 0)
							I2CEx_State = I2CEx_STATE_RX;
						break;
					case I2CEx_STATUS_TX_CPLT:
						if (msg->rxbuf.len > 0)
							I2CEx_State = I2CEx_STATE_RX;
						else
						{
							*msg->is_completed = 1;
							I2CEx_State = I2CEx_STATE_IDLE;
						}
						break;
					case I2CEx_STATUS_RX_CPLT:
						*msg->is_completed = 1;
						I2CEx_State = I2CEx_STATE_IDLE;
						break;
					default:
						break;
				}
				break;

			case I2CEx_STATE_CFG:
				/* Re-config if setting config difference with last configuration */
				if (Current_Ch == msg->ch)
				{
					if (msg->txbuf.len > 0)
						I2CEx_State = I2CEx_STATE_TX;
					else if (msg->rxbuf.len > 0)
						I2CEx_State = I2CEx_STATE_RX;
					break;
				}

#if BUGFIX_RESET_I2CEX
				// Bugfix from p1-bring-up branch
				I2CEx_Reset();
#endif // BUGFIX_RESET_I2CEX

				/* Select I2C output channel */
				I2CEx_Mode = I2CEx_MODE_CONFIG;
				Current_Ch = msg->ch;

				/* Enable Error and Buffer Interrupts */
				I2C_ITConfig(I2CEx, (I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR), ENABLE);

				/* I2C ENABLE */
				I2C_Cmd(I2CEx, ENABLE);

				/* Generate the Start condition */
				I2C_GenerateSTART(I2CEx, ENABLE);
				I2CEx_State = I2CEx_STATE_WAIT;
				I2CEx_Status = I2CEx_STATUS_BUSY;
				break;

			case I2CEx_STATE_TX:
				/* Wait for transmit completed */
				while (I2C_GetFlagStatus(I2CEx, I2C_FLAG_BUSY)){};

				I2CEx_Mode = I2CEx_MODE_TRANSMITTER;
				I2CEx_TxIdx = 0x00;

				/* Enable Error and Buffer Interrupts */
				I2C_ITConfig(I2CEx, (I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR), ENABLE);

				/* I2C ENABLE */
				I2C_Cmd(I2CEx, ENABLE);

				/* Generate the Start condition */
				I2C_GenerateSTART(I2CEx, ENABLE);
				I2CEx_State = I2CEx_STATE_WAIT;
				I2CEx_Status = I2CEx_STATUS_BUSY;
				break;

			case I2CEx_STATE_RX:
				/* Wait for transmit completed */
				while (I2C_GetFlagStatus(I2CEx, I2C_FLAG_BUSY)){};

				I2CEx_Mode = I2CEx_MODE_RECEIVER;
				I2CEx_RxIdx = 0x00;
				NumberOfByteToReceive = msg->rxbuf.len;

				/* Enable Error and Buffer Interrupts */
				I2C_ITConfig(I2CEx, I2C_IT_EVT, ENABLE);

				/* Enable Acknowledge */
				I2C_AcknowledgeConfig(I2CEx, ENABLE);

				/* I2C ENABLE */
				I2C_Cmd(I2CEx, ENABLE);

				/* Generate the Start condition */
				I2C_GenerateSTART(I2CEx, ENABLE);
				I2CEx_State = I2CEx_STATE_WAIT;
				I2CEx_Status = I2CEx_STATUS_BUSY;
				break;

			case I2CEx_STATE_IDLE:
			default:
				xStatus = xQueueReceive(xI2CExQueue, msg, 10 / portTICK_RATE_MS);
				if(xStatus == pdPASS)
				{
					I2CEx_State = I2CEx_STATE_CFG;
					I2CEx_Status = I2CEx_STATUS_BUSY;
				}
				break;
		}
		taskYIELD();
	}
}

/*
 * Initializes the I2C3 as slave.
 * Enable clock supply
 * Configure GPIOB:
 *   Use PB8 : I2C_SCL
 *   Use PB9 : I2C_SDA
 * Configure interrupt.
 */
STATIC void I2CEx_InitPort(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* RCC Configuration */
	RCC_APB1PeriphClockCmd(I2CEx_CLK, ENABLE);

	/*SDA GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2CEx_SDA_GPIO_CLK, ENABLE);

	/*SCL GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2CEx_SCL_GPIO_CLK, ENABLE);

	/* Reset I2CEx IP */
	RCC_APB1PeriphResetCmd(I2CEx_CLK, ENABLE);

	/* Release reset signal of I2Cx IP */
	RCC_APB1PeriphResetCmd(I2CEx_CLK, DISABLE);

	/* Connect PB6 to I2C_SCL */
	GPIO_PinAFConfig(I2CEx_SCL_GPIO_PORT, I2CEx_SCL_SOURCE, I2CEx_SCL_AF);
	/* Connect PB9 to I2C_SDA */
	GPIO_PinAFConfig(I2CEx_SDA_GPIO_PORT, I2CEx_SDA_SOURCE, I2CEx_SDA_AF);

	/* GPIO Configuration */
	/*Configure I2C SCL pin */
	/* Set default I2C GPIO	settings */
	GPIO_InitStruct.GPIO_Pin = I2CEx_SCL_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	/* Init pins */
	GPIO_Init(I2CEx_SCL_GPIO_PORT, &GPIO_InitStruct);
	/*Configure I2C SDA pin */
	GPIO_InitStruct.GPIO_Pin = I2CEx_SDA_PIN;
	/* Init pins */
	GPIO_Init(I2CEx_SDA_GPIO_PORT, &GPIO_InitStruct);

	/* NVIC configuration */
	/* Configure the Priority Group to 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure the I2C event priority */
	NVIC_InitStruct.NVIC_IRQChannel = I2CEx_EV_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	/* Configure I2C error interrupt to have the higher priority */
	NVIC_InitStruct.NVIC_IRQChannel = I2CEx_ER_IRQn;
	NVIC_Init(&NVIC_InitStruct);

	/* GPIO Configuration */
	/*Configure I2C SCL pin */
	/* Set default I2C GPIO	settings */
	/*SCL GPIO clock enable */
	RCC_AHB1PeriphClockCmd(I2CEx_RST_GPIO_CLK, ENABLE);
	GPIO_InitStruct.GPIO_Pin = I2CEx_RST_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	/* Init pins */
	GPIO_Init(I2CEx_RST_GPIO_PORT, &GPIO_InitStruct);
}

/*
 * Initializes the I2C3 as slave.
 * Enable clock supply
 * Configure GPIOB:
 *   Use PB8 : I2C_SCL
 *   Use PB9 : I2C_SDA
 * Configure interrupt.
 */
STATIC void I2CEx_Config(void)
{
	I2C_InitTypeDef I2C_InitStruct;

	/* I2C Init structions */
	I2C_DeInit(I2CEx);

	/* Initialize I2C peripheral */
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_OwnAddress1 = 0x30;
	I2C_InitStruct.I2C_ClockSpeed = I2CEx_I2C_CLOCKSPEED;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	// JS experiment
#if BUGFIX_DIGITAL_FILTER
	I2C_DigitalFilterConfig(I2CEx, 15);
#endif
#if BUGFIX_ANALOG_FILTER
	I2C_AnalogFilterCmd(I2CEx, ENABLE);
#endif
	// End JS experiment

	/* Initialize I2C */
	I2C_Init(I2CEx, &I2C_InitStruct);
}
/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

void I2CEx_ER_IRQHandler(void)
{
	uint16_t err = I2C_ReadRegister(I2CEx, I2C_Register_SR1) & 0xFF00;
	/* Read SR1 register to get I2C error */
	/* Clears error flags */
	if ((err & 0xFF00) != 0x00)
	{
		/* Clears error flags */
		I2CEx_Status = I2CEx_STATUS_ERROR;
		//log_error("0x%x %s", err, (err & 0x400) ? "Don't receive ACK from device" : " ");
		I2CEx->SR1 &= 0x00FF;
	}
}


void I2CEx_EV_IRQHandler(void)
{
	__IO UInt32 cnt = 0;
	/* Once the Start condition is sent the master can be master receiver
	or master transmitter */
	if ((I2CEx_Mode == I2CEx_MODE_TRANSMITTER) || (I2CEx_Mode == I2CEx_MODE_CONFIG))
	{
		I2CEx_Event = I2C_GetLastEvent(I2CEx);
		/* Get Last I2C Event */
		switch (I2CEx_Event)
		{
		/* ************************************************************************/
		/*                        Master Transmitter Events                       */
		/*                                                                        */
		/* ************************************************************************/
		/* Sending the header sequence for Master Transmitter case ---------------*/

		/* Check on EV5 */
		case I2C_EVENT_MASTER_MODE_SELECT :
			/* Send slave Address for write */
			if(I2CEx_Mode == I2CEx_MODE_CONFIG)
			{
				if (Current_Ch < 8)
					I2C_Send7bitAddress(I2CEx, I2CEx_SLAVE1_ADDRESS, I2C_Direction_Transmitter);
				else
					I2C_Send7bitAddress(I2CEx, I2CEx_SLAVE2_ADDRESS, I2C_Direction_Transmitter);
			}
			else
			{
				I2C_Send7bitAddress(I2CEx, I2CEx_Msg.addr, I2C_Direction_Transmitter);
			}
			break;

		/* Check on EV6 */
		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
			/* Configuration I2C Expander  */
			if(I2CEx_Mode == I2CEx_MODE_CONFIG)
			{
				I2C_SendData(I2CEx, 1 << (Current_Ch % 8));
				I2CEx_Status = I2CEx_STATUS_CFG_CPLT;
			}
			else
			{
				I2C_SendData(I2CEx, I2CEx_Msg.txbuf.data[I2CEx_TxIdx++]);
			}
			break;

		/* Check on EV8 */
		case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
		case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
			if(I2CEx_Mode == I2CEx_MODE_TRANSMITTER)
			{
				if (I2CEx_TxIdx == I2CEx_Msg.txbuf.len)
				{
					/* Send STOP condition */
					for (cnt = 0; cnt < 5000; cnt++);
					I2C_GenerateSTOP(I2CEx, ENABLE);
					I2C_ITConfig(I2CEx, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
					I2CEx_Status = I2CEx_STATUS_TX_CPLT;
				}
				else
				{
					/* Transmit Data TxBuffer */
					I2C_SendData(I2CEx, I2CEx_Msg.txbuf.data[I2CEx_TxIdx++]);
				}
			}
			else
			{
				/* Send STOP condition */
				I2C_GenerateSTOP(I2CEx, ENABLE);
				I2C_ITConfig(I2CEx, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
			}
			break;

		default:
			break;
		}
	}

	/*************************************************************************/
	/*                        Master Receiver Events                         */
	/*                                                                       */
	/*************************************************************************/
	else /* MASTER_MODE_RECEIVER */
	{
		/* Check on EV5 */
		if(I2C_GetITStatus(I2CEx, I2C_IT_SB) == SET)
		{
			/* Send slave Address for read */
			I2C_Send7bitAddress(I2CEx, I2CEx_Msg.addr, I2C_Direction_Receiver);
			if (NumberOfByteToReceive == 0x03)
			{
				/* Disable buffer Interrupts */
				I2C_ITConfig(I2CEx, I2C_IT_BUF , DISABLE);
			}
			else
			{
				/* Enable buffer Interrupts */
				I2C_ITConfig(I2CEx, I2C_IT_BUF , ENABLE);
			}
		}

		else if(I2C_GetITStatus(I2CEx, I2C_IT_ADDR)== SET)
		{
			if (NumberOfByteToReceive == 1)
			{
				I2C_AcknowledgeConfig(I2CEx, DISABLE);
			}
			/* Clear ADDR Register */
			(void)(I2CEx->SR1);
			(void)(I2CEx->SR2);

			if (NumberOfByteToReceive == 1)
			{
				I2C_GenerateSTOP(I2CEx, ENABLE);
			}

			if (NumberOfByteToReceive == 2)
			{
				I2C_AcknowledgeConfig(I2CEx, DISABLE);
				I2C_NACKPositionConfig(I2CEx, I2C_NACKPosition_Next);
				/* Disable buffer Interrupts */
				I2C_ITConfig(I2CEx, I2C_IT_BUF , DISABLE);
			}
		}
		else if((I2C_GetITStatus(I2CEx, I2C_IT_RXNE)== SET)&&(I2C_GetITStatus(I2CEx, I2C_IT_BTF)== RESET))
		{
			/* Store I2C received data */
			I2CEx_Msg.rxbuf.data[I2CEx_RxIdx++] = I2C_ReceiveData (I2CEx);
			NumberOfByteToReceive--;

			if (NumberOfByteToReceive == 0x03)
			{
				/* Disable buffer Interrupts */
				I2C_ITConfig(I2CEx, I2C_IT_BUF , DISABLE);
			}

			if (NumberOfByteToReceive == 0x00)
			{
				/* Disable Error and Buffer Interrupts */
				I2C_ITConfig(I2CEx, (I2C_IT_EVT | I2C_IT_BUF), DISABLE);
				I2CEx_Status = I2CEx_STATUS_RX_CPLT;
			}
		}
		/* BUSY, MSL and RXNE flags */
		else if(I2C_GetITStatus(I2CEx, I2C_IT_BTF)== SET)
		{
			/* if Three bytes remaining for reception */
			if (NumberOfByteToReceive == 3)
			{
				I2C_AcknowledgeConfig(I2CEx, DISABLE);
				/* Store I2C received data */
				I2CEx_Msg.rxbuf.data[I2CEx_RxIdx++] = I2C_ReceiveData (I2CEx);
				NumberOfByteToReceive--;
			}
			else if (NumberOfByteToReceive == 2)
			{
				I2C_GenerateSTOP(I2CEx, ENABLE);

				/* Store I2C received data */
				I2CEx_Msg.rxbuf.data[I2CEx_RxIdx++] = I2C_ReceiveData (I2CEx);
				NumberOfByteToReceive--;
				/* Store I2C received data */
				I2CEx_Msg.rxbuf.data[I2CEx_RxIdx++] = I2C_ReceiveData (I2CEx);
				NumberOfByteToReceive--;
				/* Disable Error and Buffer Interrupts */
				I2C_ITConfig(I2CEx, (I2C_IT_EVT | I2C_IT_BUF), DISABLE);
				I2CEx_Status = I2CEx_STATUS_RX_CPLT;
			}
			else
			{
				/* Store I2C received data */
				I2CEx_Msg.rxbuf.data[I2CEx_RxIdx++] = I2C_ReceiveData (I2CEx);
				NumberOfByteToReceive--;
			}
		}
	}
}

/*********** Portions COPYRIGHT 2015 Infonam. Co., Ltd.*****END OF FILE****/
