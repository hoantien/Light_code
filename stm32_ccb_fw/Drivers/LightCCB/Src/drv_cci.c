/**
  ******************************************************************************
  * @file    drvI2C.h
  * @author  Infonam Embedded Team
  * @version V1.1.0
  * @date    05-Mar-2015
  * @brief   This file provides set of firmware functions to I2C driver.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "drv_cci.h"
#include "hal_cci.h"
#include "mems.h"
#include "log.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*
 * TODO: this define used to convert endianness,
 * if don't use, comment define bellow
 * have two define in two files: cam_ctrl.c and drv_cci.c
 */
#define CONVERT_TO_LITTLE_ENDIAN

#define I2C_SLAVE_BUFFER_SIZE		64
/* Private macro -------------------------------------------------------------*/
#define CONVERT_ENDIANNESS_2_BYTE(x)	((uint16_t)((((x) >> 8) & 0x00ff) | \
													(((x) << 8) & 0xff00)))

#define CONVERT_ENDIANNESS_4_BYTE(x)	((uint32_t)((((x) >> 24) & 0x000000ff)|\
													(((x) >> 8)  & 0x0000ff00)|\
													(((x) << 8)  & 0x00ff0000)|\
													(((x) << 24) & 0xff000000)))

/* Private variables ---------------------------------------------------------*/
extern unsigned int stm_ver;
/* Private variables ---------------------------------------------------------*/
uint8_t Buffer[I2C_SLAVE_BUFFER_SIZE];
static uint8_t *xdata = NULL;
__IO CCI_Buffer Transfer_Buf = {0x0000, &Buffer[0], 0, 0, 0};

__IO uint8_t Tx_Idx = 0x00;
__IO uint8_t Rx_Idx = 0x00;

__IO uint8_t Counter = 0x00;
__IO uint32_t Event = 0x00;

__IO uint16_t Addr2003 = 0xAA55;
uint8_t ReceiveEvent = 0;
__IO uint16_t ReceiveCmd = 0xFFFF;
__IO uint16_t ReceiveTid = 0;
__IO uint16_t ReceiveDataSize = 0;
__IO uint16_t ReceivedFlags = 0;

typedef enum  ReceiveState {
	RECEIVE_IDLE = 0,
	RECEIVE_TID,
	RECEIVE_CMD,
	RECEIVE_DATA,
	RECEIVE_M_BITMASK,
	RECEIVE_UCID,
	RECEIVE_TOLERANCE
}ReceiveState;

ReceiveState I2CReceiveState;
extern interrupt_status intr_sta;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void CCB_CCI0_Init(void)
{
	GPIO_InitTypeDef GPIO_I2C_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	/* RCC Configuration */
	RCC_APB1PeriphClockCmd(CCI0_CLK, ENABLE);

	/*SDA GPIO clock enable */
	RCC_AHB1PeriphClockCmd(CCI0_SDA_GPIO_CLK, ENABLE);

	/* Reset I2C3 IP */
	RCC_APB1PeriphResetCmd(CCI0_CLK, ENABLE);

	/* Release reset signal of I2Cx IP */
	RCC_APB1PeriphResetCmd(CCI0_CLK, DISABLE);

	/* GPIO Configuration */
	/*Configure I2C SCL pin */
	/* Set default I2C GPIO	settings */
	GPIO_I2C_InitStruct.GPIO_Pin = CCI0_SCL_PIN;
	GPIO_I2C_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_I2C_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_I2C_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_I2C_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	/* Init pins */
	GPIO_Init(CCI0_SCL_GPIO_PORT, &GPIO_I2C_InitStruct);
	/*Configure I2C SDA pin */
	GPIO_I2C_InitStruct.GPIO_Pin = CCI0_SDA_PIN;
	/* Init pins */
	GPIO_Init(CCI0_SDA_GPIO_PORT, &GPIO_I2C_InitStruct);

	/* Connect PB6 to I2C_SCL */
	GPIO_PinAFConfig(CCI0_SCL_GPIO_PORT, CCI0_SCL_SOURCE, CCI0_SCL_AF);
	/* Connect PB9 to I2C_SDA */
	GPIO_PinAFConfig(CCI0_SDA_GPIO_PORT, CCI0_SDA_SOURCE, CCI0_SDA_AF);

	/* NVIC configuration */
	/* Configure the Priority Group to 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure the I2C event priority */
	NVIC_InitStruct.NVIC_IRQChannel = CCI0_EV_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	/* Configure I2C error interrupt to have the higher priority */
	NVIC_InitStruct.NVIC_IRQChannel = CCI0_ER_IRQn;
	NVIC_Init(&NVIC_InitStruct);

	I2C_DeInit(CCI0_I2C);

	/* Initialize I2C peripheral */
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = CCI0_SLAVE_ADDRESS;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	/* Initialize I2C */
	I2C_Init(CCI0_I2C, &I2C_InitStruct);

	/* Enable Error and Buffer Interrupts */
	I2C_ITConfig(CCI0_I2C, (I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), ENABLE);

	/* Enable I2C */
	I2C_Cmd(CCI0_I2C, ENABLE);
}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles I2Cx Error interrupt request.
  * @param  None
  * @retval None
  */
void CCI0_ER_IRQHandler(void)
{
	uint16_t err = I2C_ReadRegister(CCI0_I2C, I2C_Register_SR1) & 0xFF00;
	/* Read SR1 register to get I2C error */
	/* Clears error flags */
	if (err != 0x00)
	{
//		log_error("0x%x", err);
		CCI0_I2C->SR1 &= 0x00FF;
	}
}

/**
  * @brief  This function handles I2Cx event interrupt request.
  * @param  None
  * @retval None
  */
void CCI0_EV_IRQHandler(void)
{
	/* Get Last I2C Event */
	volatile uint16_t tmp;
	volatile ccb_cmd_base_t * cam_m_status = (ccb_cmd_base_t *) CCB_M_WRITE;

	Event = I2C_GetLastEvent(CCI0_I2C);
	switch (Event)
	{
		/* ****************************************************************************/
		/*                          Slave Transmitter Events                          */
		/*                                                                            */
		/* ****************************************************************************/

		/* Check on EV1 */
		case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
            // 20160328: Hack to enable reading registers for slave CCI0
            Transfer_Buf.cmd = ReceiveTid;

			tmp = Transfer_Buf.cmd + Tx_Idx;
			if (tmp >= 0x200 && tmp <= 0x253)
			{
				I2C_SendData(CCI0_I2C, *(CCB_ASIC_MEM_RD + tmp - 0x200));
				I2C_ITConfig(CCI0_I2C, I2C_IT_BUF , ENABLE);
				Tx_Idx++;
			}
			else if((Transfer_Buf.cmd == 0x0028))
			{
				I2C_SendData(CCI0_I2C, *(((uint8_t *)cam_m_status->cam_m_status) + Tx_Idx++));
			}
			else if ((Transfer_Buf.cmd >= 0x0000) && (Transfer_Buf.cmd <= 0x2000))
			{
				//I2C_SendData(CCI0_I2C, *(CCB_ASIC_CAM_RD + Transfer_Buf.cmd + Tx_Idx++));
			    switch(Transfer_Buf.cmd)
			    {
			        case CAM_MODULE_OPEN:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_open;
			            break;
			        }
			        case CAM_STREAMING:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_stream;
			            break;
			        }
			        case CAM_SNAPSHOT_UUID:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_uuid;
			            break;
			        }
			        case CAM_COMMAND_STATUS:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_command;
			            break;
			        }
			        case CAM_MODULE_RESOLUTION:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_resolution;
			            break;
			        }
			        case CAM_MODULE_SENSITIVITY:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_sensitivity;
			            break;
			        }
			        case CAM_MODULE_EXPOSURE_TIME:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_exposure_time;
			            break;
			        }
			        case CAM_MODULE_FOCUS_DISTANCE:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_focus_distance;
			            break;
			        }
			        case CAM_MODULE_FPS:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_fps;
			            break;
			        }
			        case CAM_MODULE_LENS_HALL:
			        case CAM_MODULE_LENS_POSITION:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_lens_hall;
			            break;
			        }
			        case CAM_MODULE_MIRROR_HALL:
			        case CAM_MODULE_MIRROR_POSITION:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_mirror_hall;
			            break;
			        }
			        case CAM_MODULE_FOCAL_LEN:
			        {
			            xdata = (uint8_t *)cam_m_status->cam_m_focal_length;
			            break;
			        }
			        case CAM_MODULE_MIRROR_FOCAL_STATUS:
			        {
			        	xdata = (uint8_t *)cam_m_status->cam_m_mirror_status;
			        	break;
			        }
			        case CAM_MODULE_FOCUS_STATUS:
			        {
			        	xdata = (uint8_t *)cam_m_status->cam_m_focus_status;
			        	break;
			        }
			        case CAM_MODULE_GYRO_STATUS:
			        {
			        	xdata = (uint8_t *)cam_m_status->cam_m_gyro;
			        	break;
			        }
			        case CCB_INTR_SRC:
					{
						xdata = (uint8_t *)&(intr_sta.current_status);
						break;
					}
                    case LIGHT_ACTIVE_UCID:
                    {
                        xdata = (uint8_t *) cam_m_status->cam_m_light_active_ucid;
                        break;
                    }
                    case STM_VERSION:
                    {
                    	xdata = (uint8_t *) &stm_ver;
                    	break;
                    }
                    case CAM_MODULE_TEMP:
                    {
                    	xdata = (uint8_t *) cam_m_status->cam_m_temp;
                    	break;
                    }
                    case CAM_COMMAND_BOARD_TEMPERATURE:
                    {
                    	xdata = (uint8_t *) cam_m_status->cam_m_board_temp;
                    	break;
                    }
			        default:
			        {
			        	xdata = NULL;
			            break;
			        }
			    }
			    if(xdata)
			        I2C_SendData(CCI0_I2C, *(xdata + Tx_Idx++));
			    else
			        I2C_SendData(CCI0_I2C, *(CCB_ASIC_CAM_RD + Transfer_Buf.cmd + Tx_Idx++));
			}
			else if (Transfer_Buf.cmd == 0x2003)
			{
				I2C_SendData(CCI0_I2C, (uint8_t)((Addr2003>>8)&0xFF));
			}
			else
			{
				I2C_SendData(CCI0_I2C, 0xFF);
			}
			break;

		/* Check on EV3 */
		case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
		case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
			tmp = Transfer_Buf.cmd + Tx_Idx;
			if (tmp >= 0x200 && tmp <= 0x253)
			{
				I2C_SendData(CCI0_I2C, *(CCB_ASIC_MEM_RD + tmp - 0x200));
				Tx_Idx++;
			}
			else if((Transfer_Buf.cmd == 0x0028))
			{
				I2C_SendData(CCI0_I2C, *(((uint8_t *)cam_m_status->cam_m_status) + Tx_Idx++));
			}
			else if ((Transfer_Buf.cmd >= 0x0000) && (Transfer_Buf.cmd <= 0x2000))
			{
				if(xdata)
					I2C_SendData(CCI0_I2C, *(xdata + Tx_Idx++));
				else
					I2C_SendData(CCI0_I2C, *(CCB_ASIC_CAM_RD + Transfer_Buf.cmd + Tx_Idx++));
			}
			else if (Transfer_Buf.cmd == 0x2003)
			{
				I2C_SendData(CCI0_I2C, (uint8_t)Addr2003);
			}
			else
			{
				I2C_SendData(CCI0_I2C, 0xFF);
			}
			break;

		/* ****************************************************************************/
		/*                              Slave Receiver Events                         */
		/*                                                                            */
		/* ****************************************************************************/

		/* check on EV1*/
		case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
			I2CReceiveState = RECEIVE_TID;
			Rx_Idx = 0x00;
			Tx_Idx = 0x00;
			break;

		/* Check on EV2*/
		case I2C_EVENT_SLAVE_BYTE_RECEIVED:
		case (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_SR1_BTF):
			switch(I2CReceiveState)
			{
				case RECEIVE_TID:
					if (Rx_Idx == 0)
					{
						ReceiveTid = I2C_ReceiveData(CCI0_I2C);
						Rx_Idx++;
					}
					else
					{
						ReceiveTid <<= 8;
						ReceiveTid |= I2C_ReceiveData(CCI0_I2C);
						Rx_Idx = 0;
						I2CReceiveState = RECEIVE_CMD;
					}
					break;
				case RECEIVE_CMD:
					if (Rx_Idx == 0)
					{
						Transfer_Buf.cmd = I2C_ReceiveData(CCI0_I2C);
						Transfer_Buf.num_rx_bytes = 0;
						Rx_Idx++;
					}
					else
					{
						Transfer_Buf.cmd <<= 8;
						Transfer_Buf.cmd |= I2C_ReceiveData(CCI0_I2C);

#ifdef CONVERT_TO_LITTLE_ENDIAN
						/* convert endianness by using built in macro of gcc*/
						Transfer_Buf.cmd =  __builtin_bswap16(Transfer_Buf.cmd);
#endif
						Transfer_Buf.flags = ((CCB_FLAGS_MASK & Transfer_Buf.cmd) >> 12) & 0xFF ;
						Transfer_Buf.cmd = Transfer_Buf.cmd & CCB_CMD_MASK;

						if ((Transfer_Buf.cmd == CAM_MODULE_OPEN) 				    ||
							(Transfer_Buf.cmd == CAM_STREAMING) 				    ||
							(Transfer_Buf.cmd == RDI_CAPTURE)                       ||
							(Transfer_Buf.cmd == CAM_MODULE_SENSITIVITY) 		    ||
							(Transfer_Buf.cmd == CAM_MODULE_EXPOSURE_TIME) 		    ||
							(Transfer_Buf.cmd == CAM_MODULE_RESOLUTION) 		    ||
							(Transfer_Buf.cmd == CAM_MODULE_VCM_POSITION) 		    ||
							(Transfer_Buf.cmd == CAM_MODULE_FOCUS_DISTANCE) 	    ||
							(Transfer_Buf.cmd == CAM_MODULE_FPS) 				    ||
							(Transfer_Buf.cmd == CAM_MODULE_STATUS) 			    ||
							(Transfer_Buf.cmd == CAM_MODULE_FOCUS_STATUS)			||
							(Transfer_Buf.cmd == CAM_MODULE_MIRROR_CALIBRATION)     ||
							(Transfer_Buf.cmd == CAM_MODULE_LENS_POSITION) 		    ||
							(Transfer_Buf.cmd == CAM_MODULE_LENS_POSITION2)		    ||
							(Transfer_Buf.cmd == CAM_MODULE_LENS_HALL) 			    ||
							(Transfer_Buf.cmd == CAM_MODULE_LENS_NUDGE) 		    ||
							(Transfer_Buf.cmd == CAM_MODULE_MIRROR_POSITION) 	    ||
							(Transfer_Buf.cmd == CAM_MODULE_MIRROR_POSITION2) 	    ||
							(Transfer_Buf.cmd == CAM_MODULE_MIRROR_HALL) 		    ||
							(Transfer_Buf.cmd == CAM_MODULE_MIRROR_NUDGE) 		    ||
							(Transfer_Buf.cmd == CAM_MODULE_FINE_NUDGE_LENS)    	||
							(Transfer_Buf.cmd == CAM_MODULE_FINE_NUDGE_MIRROR)      ||
							(Transfer_Buf.cmd == CAM_MODULE_LENS_CALIBRATION)       ||
							(Transfer_Buf.cmd == CAM_MODULE_LENS_CALIBRATION2)      ||
							(Transfer_Buf.cmd == CAM_MODULE_DEBUG_I2C_READ)         ||
							(Transfer_Buf.cmd == CAM_MODULE_DEBUG_I2C_WRITE)        ||
							(Transfer_Buf.cmd == CAM_MODULE_DEBUG_PIEZO_MONITOR2)   ||
							(Transfer_Buf.cmd == CAM_MODULE_FOCUS_CALIBRATION_DATA) ||
							(Transfer_Buf.cmd == CAM_MODULE_NUDGE_CPLD)             ||
							(Transfer_Buf.cmd == CAM_MODULE_PIEZO_LENS_CONTROL)     ||
							(Transfer_Buf.cmd == CAM_MODULE_PIEZO_MIRROR_CONTROL)   ||
							(Transfer_Buf.cmd == CAM_MODULE_LENS_FREQ_CALIBRATION)  ||
							(Transfer_Buf.cmd == CAM_MODULE_MIRROR_FREQ_CALIBRATION)||
							(Transfer_Buf.cmd == CAM_MODULE_SAVE_POSITION)||
							(Transfer_Buf.cmd == CAM_MODULE_MIRROR_FOCAL_STATUS)||
							(Transfer_Buf.cmd == CAM_MODULE_SET_CAPTURE_PARAM)||
							(Transfer_Buf.cmd == CAM_MODULE_TEMP)||
							0)
						{
							I2CReceiveState = RECEIVE_M_BITMASK;
						}
						else if(Transfer_Buf.cmd == LIGHT_ACTIVE_UCID)
						{
						    I2CReceiveState = RECEIVE_UCID;
						}
						else
						{
							I2CReceiveState = RECEIVE_DATA;
						}
						Rx_Idx = 0;
					}
					break;

				case RECEIVE_M_BITMASK://Receive 3-bytes m_bitmask
					if(Rx_Idx < 2)
					{
						*(CCB_M_BITMASK + Rx_Idx) = I2C_ReceiveData(CCI0_I2C);
						Rx_Idx++;
					}
					else
					{
						*(CCB_M_BITMASK + Rx_Idx) = I2C_ReceiveData(CCI0_I2C);
						I2CReceiveState = RECEIVE_DATA;
						if ((Transfer_Buf.cmd == CAM_MODULE_SENSITIVITY)            ||
                            (Transfer_Buf.cmd == CAM_MODULE_EXPOSURE_TIME)          ||
                            (Transfer_Buf.cmd == CAM_MODULE_RESOLUTION)             ||
                            (Transfer_Buf.cmd == CAM_MODULE_VCM_POSITION)           ||
                            (Transfer_Buf.cmd == CAM_MODULE_FOCUS_DISTANCE)         ||
                            (Transfer_Buf.cmd == CAM_MODULE_FPS)                    ||
                            (Transfer_Buf.cmd == CAM_MODULE_LENS_HALL)              ||
                            (Transfer_Buf.cmd == CAM_MODULE_MIRROR_HALL)            ||
                            (Transfer_Buf.cmd == CAM_MODULE_FOCUS_STATUS)			||
                            0)
                        {
							I2CReceiveState = RECEIVE_UCID;
						}
						else if(Transfer_Buf.cmd == CAM_MODULE_LENS_POSITION)
						{
							I2CReceiveState = RECEIVE_TOLERANCE;
						}
						Rx_Idx = 0;
					}
					break;
				case RECEIVE_TOLERANCE:
				{
					*(CCB_M_TOLERANCE) = I2C_ReceiveData(CCI0_I2C);
					I2CReceiveState = RECEIVE_DATA;
					Rx_Idx = 0;
					break;
				}
				case RECEIVE_UCID:
					if(Rx_Idx < 1)
					{
						*(CCB_M_UCID + Rx_Idx) = I2C_ReceiveData(CCI0_I2C);
						Rx_Idx++;
					}
					else
					{
						*(CCB_M_UCID + Rx_Idx) = I2C_ReceiveData(CCI0_I2C);
						I2CReceiveState = RECEIVE_DATA;
						Rx_Idx = 0;
					}
					break;

				case RECEIVE_DATA:
					tmp = Transfer_Buf.cmd + Rx_Idx;
					if ((Transfer_Buf.cmd >= 0x0000) && (Transfer_Buf.cmd <= 0x2000))
					{
					    Transfer_Buf.num_rx_bytes ++;
						*(CCB_ASIC_CAM_RD + tmp) = I2C_ReceiveData(CCI0_I2C);
					}
					else
					{
						I2C_ReceiveData(CCI0_I2C);
					}
					Rx_Idx++;
					break;
				case RECEIVE_IDLE:
				default:
					break;
			}
			break;

		/* Check on EV4 */
		case I2C_EVENT_SLAVE_STOP_DETECTED:
			I2C_GetFlagStatus(CCI0_I2C, I2C_FLAG_STOPF);
			I2C_Cmd(CCI0_I2C, ENABLE);
			ReceiveCmd = Transfer_Buf.cmd;
			ReceiveDataSize = Transfer_Buf.num_rx_bytes;
			ReceivedFlags = Transfer_Buf.flags;
			I2CReceiveState = RECEIVE_IDLE;
			break;

		default:
			/* Read SR1 register to get I2C error */
			if ((I2C_ReadRegister(CCI0_I2C, I2C_Register_SR1) & 0xFF00) != 0x00)
			{
				/* Clears error flags */
				CCI0_I2C->SR1 &= 0x00FF;
			}
			break;
	}
}

/*********** Portions COPYRIGHT 2015 Infonam. Co., Ltd.*****END OF FILE****/
