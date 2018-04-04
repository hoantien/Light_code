/**
 * @file drv_spi_slave.c
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cam_ctrl.h"
#include "hal_flash_ex.h"
#include "cam_eeprom.h"
#include "errors.h"
#include "log.h"
/* Private define ------------------------------------------------------------*/
#define BUFFERSIZE	32
#define SPI_SLAVE_COMMAND_SIZE		14
#define READMODE  0
#define WRITEMODE 1

#define ASIC_MODULE_METADATA    0x0272
#define ASIC_TMPx			    0x021C
#define ASIC_LOG_CTRL		    0x0219

#define READ_FULL_EEPROM  		0x01
#define EEPROM_STATUS			0x02
#define READ_STATUS_EEPROM		0x03
#define READ_PARTIAL_EEPROM		0x04
#define WRITE_PARTIAL_EEPROM	0x05
#define MAX_TEMP			    4
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	CMD_RX,
	CMD_TX,
	CMD_PARSER,
	CMD_WAIT,
	CMD_LOG
}cmd_state;

typedef enum
{
	RX_STATUS,
	TX_STATUS,
	READ_EEPROM_FAIL,
	READ_EEPROM_COMPLETED,
	CMD_FAIL_STATUS,
	CMD_COMPLETE_STATUS,
	CAMERA_FAIL_STATUS,
	MODE_FAIL_STATUS,
	SIZE_OVERFLOW_STATUS
} status_flag;

typedef struct
{
	UInt16 tid;
	UInt16 cmd;
	UInt32 m_bitmask;
	UInt8 sub_cmd;
	UInt16 size_write;
	UInt16 size_read;
	UInt8 flag_status;
	UInt16 sub_addr;
	UInt8 data[2];
}msg_parser_t;
msg_parser_t msg_parser;
typedef struct
{
	cmd_state state;
	status_flag status;
}state_t;
state_t m_state;

extern Bool save_command_log(uint16_t tid,cam_cmd_status cmd_status);
/* Private variables ---------------------------------------------------------*/
static SPI_DMA_Dev SNAP_DMA ={SNAP_DMA_TX_CHANNEL, SNAP_DMA_TX_STREAM, SNAP_DMA_RX_CHANNEL, SNAP_DMA_RX_STREAM};
static DMA_InitTypeDef DMA_InitStruct;

static SPI_Dev SNAP_SPI_Dev = {
		{SNAP_SCK_PORT,SNAP_SCK_PIN,SNAP_SCK_CLK,SNAP_SCK_SOURCE,SNAP_SCK_AF},
		{SNAP_MISO_PORT,SNAP_MISO_PIN,SNAP_MISO_CLK,SNAP_MISO_SOURCE,SNAP_MISO_AF},
		{SNAP_MOSI_PORT,SNAP_MOSI_PIN,SNAP_MOSI_CLK,SNAP_MOSI_SOURCE,SNAP_MOSI_AF},
		{SNAP_NSS_PORT,SNAP_NSS_PIN,SNAP_NSS_CLK,SNAP_NSS_SOURCE,SNAP_NSS_AF}
};
UInt8 RxBuffer[BUFFERSIZE];
Int8 temp_status=0;
extern volatile UInt8 log_level;
/*--------------------------------- Private function prototypes--------------------------------*/
STATIC void SNAP_SPI_Init(SPI_Dev *setting);
STATIC void SNAP_DMA_Init(void);
STATIC void SNAP_DMA_Transfer(SPI_DMA_Dev *setting,UInt8 *txbuffer, UInt8 *rxbuffer,UInt16 size);
STATIC UInt8 SNAP_DMA_Check_Status(SPI_DMA_Dev *setting);
STATIC ErrorStatus Read_eeprom(UInt32 cam_bitmask, UInt16 addr,UInt16 size);
STATIC ErrorStatus Write_eeprom(UInt32 cam_bitmask,UInt16 addr,UInt16 size,UInt8 *data);
STATIC void vTask_SNAP_SPISLAVE(void* pvparameter);

/*-----------------------------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void SNAP_SPISLAVE_Init(void)
{
	SNAP_SPI_Init(&SNAP_SPI_Dev);
	SNAP_DMA_Init();
	xTaskCreate(vTask_SNAP_SPISLAVE,
				(const signed char * const)"SNAP SPI",
				400,
				NULL,
				1,
				NULL);
}
STATIC void vTask_SNAP_SPISLAVE(void* pvparameter)
{
	msg_parser_t *_msg_parser = &msg_parser;
	state_t *_state = &m_state;
	_state->state = CMD_RX;
	_msg_parser->size_read = SPI_SLAVE_COMMAND_SIZE;
	UInt8 total_cam = 0, status_Data=0;
    memset(RxBuffer, 0xFF, BUFFERSIZE);
	for(;;)
	{
		switch(_state->state)
		{
			case CMD_RX:
			{
                log_debug("Reached CMD_RX case");
				SNAP_DMA_Transfer(&SNAP_DMA,NULL,RxBuffer,_msg_parser->size_read);
				_state->status = RX_STATUS;
				_state->state = CMD_WAIT;
				break;
			}
			case CMD_WAIT:
			{
				switch(_state->status)
				{
					case RX_STATUS:
					{
						if(SNAP_DMA_Check_Status(&SNAP_DMA))
						{
							break;
						}
						_state->state = CMD_LOG;
						break;
					}
					case READ_EEPROM_COMPLETED:
					{
						if(SNAP_DMA_Check_Status(&SNAP_DMA))
						{
							break;
						}
						_state->state = CMD_TX;
						break;
					}
					case READ_EEPROM_FAIL:
					case MODE_FAIL_STATUS:
					case CMD_FAIL_STATUS:
					case SIZE_OVERFLOW_STATUS:
					case CAMERA_FAIL_STATUS:
					case CMD_COMPLETE_STATUS:
					case TX_STATUS:
					{
						if(SNAP_DMA_Check_Status(&SNAP_DMA))
						{
							break;
						}
						_state->state = CMD_RX;
						break;
					}
					default :
					{
						log_printf("+ Status not found!\n\r");
						break;
					}
				}
				break;
			}
			case CMD_TX:
			{
                log_debug("Reached CMD_TX case");
				if(_msg_parser->cmd == ASIC_TMPx)
				{
					SNAP_DMA_Transfer(&SNAP_DMA,CCB_M_TEMP,NULL,_msg_parser->size_write);
					_state->status = TX_STATUS;

				}
				else if(_msg_parser->cmd == ASIC_MODULE_METADATA)
				{
					switch (_msg_parser->sub_cmd)
					{
						case READ_FULL_EEPROM :
						{
							if(status_Data)
							{
								SNAP_DMA_Transfer(&SNAP_DMA,&status_Data,NULL,1);
								status_Data = 0;
								_state->status = READ_EEPROM_COMPLETED;
								_state->state = CMD_WAIT;
							}
							else
							{
								/* send data to host */
								SNAP_DMA_Transfer(&SNAP_DMA,g_eeprom_buffer,NULL,2048 * total_cam);
								_state->state = CMD_WAIT;
								_state->status = TX_STATUS;
							}
							break;
						}
						case EEPROM_STATUS:
						{
							/*todo: NOthing*/
							break;
						}
						case READ_STATUS_EEPROM:
						{
							/*todo: Nothing*/
							break;
						}
						case READ_PARTIAL_EEPROM:
						{
							if(status_Data) // check data eeprom ready
							{
								SNAP_DMA_Transfer(&SNAP_DMA,&status_Data,NULL,1);
								status_Data = 0;
								_state->status = READ_EEPROM_COMPLETED;
								_state->state = CMD_WAIT;
							}else
							{
								SNAP_DMA_Transfer(&SNAP_DMA,g_eeprom_buffer,NULL,_msg_parser->size_write * total_cam);
								_state->state = CMD_WAIT;
								_state->status = TX_STATUS;
							}
							break;
						}
						default :
						{
							_state->status = MODE_FAIL_STATUS;
							_state->state = CMD_WAIT;
							log_printf("please choose between two mode:\n\r");
							log_printf("+ 0x01 for choose other camera.\n\r");
							log_printf("+ 0x00 for read data eeprom.\n\r");
							break;
						}
					}
				}
				else if(_msg_parser->cmd == ASIC_LOG_CTRL)
				{
					UInt8 *datatosend = (UInt8 *)&log_level;
					SNAP_DMA_Transfer(&SNAP_DMA,datatosend,NULL,_msg_parser->size_write);
					_state->status = TX_STATUS;
					_state->state = CMD_WAIT;
				}
				break;
			}
			case CMD_PARSER:
			{
				log_debug("Reached CMD_PARSER case");
				_msg_parser->tid = RxBuffer[0] << 8 | RxBuffer[1];
				_msg_parser->cmd = RxBuffer[2] << 8 | RxBuffer[3];
				switch(_msg_parser->cmd)
				{
					case ASIC_MODULE_METADATA:
					{
						_msg_parser->m_bitmask  = RxBuffer[6] << 16 | RxBuffer[5] << 8 | RxBuffer[4];
						_msg_parser->sub_cmd = RxBuffer[7] ;
						/*calculate total camera */
						total_cam = 0;
						for(int i=0; i< CAM_NUM_CAMERA_MAX;i++)
						{
							if((_msg_parser->m_bitmask >>1 & 1<<i) || (_msg_parser->m_bitmask & 0x01)) // Is cam valid?
							{
								total_cam ++;
							}
						}
						/*******************************/
						if(_msg_parser->sub_cmd == READ_FULL_EEPROM)
						{
							log_debug("sub-cmd: READ_FULL_EEPROM");
							_msg_parser->sub_addr = RxBuffer[8] << 8 | RxBuffer[9]; // 2byte subaddr
							_msg_parser->size_write = RxBuffer[10] << 8 | RxBuffer [11]; // 2byte size to read data

							if(Read_eeprom(_msg_parser->m_bitmask,0xA000,2048)==SUCCESS)
							{
								log_debug("Read data eeprom successful !");
								_state->state = CMD_TX;
								status_Data=0x1;
								save_command_log(_msg_parser->tid, CMD_SUCCESS);
							}else
							{
								log_debug("Read fail!")
								_state->status = READ_EEPROM_FAIL;
								_state->state= CMD_WAIT;
								save_command_log(_msg_parser->tid, ERROR_EEPROM_FAULT);
							}
						}
						else if(_msg_parser->sub_cmd == WRITE_PARTIAL_EEPROM)
						{
							log_debug("sub-cmd: WRITE_PARTIAL_EEPROM");
							_msg_parser->sub_addr = RxBuffer[8] << 8 | RxBuffer[9]; // 2byte subaddr
							_msg_parser->size_write = RxBuffer[10] << 8 | RxBuffer [11]; // 2byte size to write data
							_msg_parser->data[0] = RxBuffer[12];
							_msg_parser->data[1] = RxBuffer[13]; // 2byte data
							/*write data to eeprom*/
							Write_eeprom(_msg_parser->m_bitmask,_msg_parser->sub_addr,_msg_parser->size_write,_msg_parser->data);
							_state->state= CMD_WAIT; // goto wait state and continues to receive data from host
							_state->status = TX_STATUS; // should fix this status
							save_command_log(_msg_parser->tid, CMD_SUCCESS);
						}
						else if(_msg_parser->sub_cmd == READ_PARTIAL_EEPROM)
						{
							log_debug("sub-cmd: READ_PARTIAL_EEPROM");
							_msg_parser->sub_addr = RxBuffer[8] << 8 | RxBuffer[9]; // 2byte subaddr
							_msg_parser->size_write = RxBuffer[10] << 8 | RxBuffer [11]; // 2byte size to read data
							if(Read_eeprom(_msg_parser->m_bitmask,_msg_parser->sub_addr,_msg_parser->size_write)==SUCCESS)
							{
								log_debug("Read data eeprom successful!");
								_state->state = CMD_TX; // go to tx state send data to host.
								status_Data = 0x1; // set status = 1 while host sending dummy data.
								save_command_log(_msg_parser->tid, CMD_SUCCESS);
							}else
							{
								log_debug("Read fail!");
								_state->status = READ_EEPROM_FAIL;
								_state->state= CMD_WAIT;
								save_command_log(_msg_parser->tid, ERROR_EEPROM_FAULT);
							}
						}
						else if(_msg_parser->sub_cmd == EEPROM_STATUS)
						{
							log_debug("sub-cmd: EEPROM_STATUS");
							/*Todo: Nothing*/
							save_command_log(_msg_parser->tid, CMD_SUCCESS);
						}
						else if(_msg_parser->sub_cmd == READ_STATUS_EEPROM)
						{
							log_debug("sub-cmd: READ_STATUS_EEPROM");
							/*Todo: Nothing*/
							save_command_log(_msg_parser->tid, CMD_SUCCESS);
						}
						else
						{
							_state->status = TX_STATUS;
							_state->state= CMD_WAIT;
							save_command_log(_msg_parser->tid, CMD_INVALID_ARG);
						}
						log_debug("Command:%04x",_msg_parser->cmd);
						log_debug("m_bitmask:%08x",(unsigned int)_msg_parser->m_bitmask);
						log_debug("sub_addr:%04x",_msg_parser->sub_addr);
						log_debug("size write :%04x",_msg_parser->size_write);
						log_debug("data:%04x",_msg_parser->data[0]<<8|_msg_parser->data[1]);
						log_debug("Total camera : %d",total_cam);
						break;
					}
					case ASIC_TMPx:
					{
						UInt16 tmp;
						UInt8 idx=0;
						uint16_t SlaveAddr[] = {0x48,0x49,0x4A,0x4B};
						_state->state = CMD_TX;
						for(int i=0; i< MAX_TEMP;i++)
						{
							tmp=Get_TempBoad(SlaveAddr[i]);
							*(CCB_M_TEMP+idx++) =(UInt8)(tmp >> 8);
							*(CCB_M_TEMP+idx++) =(UInt8)tmp;
						}
						_msg_parser->size_write = idx;
						save_command_log(_msg_parser->tid, CMD_SUCCESS);
						break;
					}
					case ASIC_LOG_CTRL:
					{
						log_debug("Hit the ASIC_LOG_CTRL case");
						uint8_t is_read = RxBuffer[4];
						uint8_t data    = RxBuffer[5];
						log_debug("Data = 0x%02x\n", data);

						/* Given that we have 14 bytes, we can use the 5th
						byte [4] as the read/write flag, and the 6th byte [4]
						as the log level argument. Using one bit as we were
						before is more space-efficient, but given that we
						always pass 12 bytes this is not a constraint. */

						if(is_read)
						{
							_msg_parser->size_write = 1;
							_state->state = CMD_TX;
							log_printf("Read log control level DONE \n\r");
							save_command_log(_msg_parser->tid, CMD_SUCCESS);
						}
						else
						{
							//log_printf("Write log control level value [0x%02x]\n\r", log_level);
							log_console("Current level value [0x%02x]\n\r", log_level);
							log_level = data & LOG_LEVEL_ALL;
							log_console("Log level value [0x%02x] written!\n\r", data & LOG_LEVEL_ALL);
							_state->state  = CMD_WAIT;
							_state->status = CMD_COMPLETE_STATUS;
							save_command_log(_msg_parser->tid, CMD_SUCCESS);
						}
						break;
					}
					default :
					{
						_state->status = CMD_FAIL_STATUS;
						_state->state = CMD_WAIT;
						log_printf("+ Command not found!\n\r");
						save_command_log(_msg_parser->tid, ERROR_INVALID_ARG);
						break;
					}
				}
				break;
			}
			case CMD_LOG:
			{
				log_debug("Data Received:");
				for(int i=0; i< _msg_parser->size_read;i++)
				{
					log_debug("[%02x]",RxBuffer[i]);
				}
				_state->state = CMD_PARSER;
				break;
			}
		}
	}
}

STATIC void SNAP_SPI_Init(SPI_Dev *setting)
{
	GPIO_InitTypeDef GPIO_InitStruct ={
			.GPIO_Mode = GPIO_Mode_AF,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd = GPIO_PuPd_NOPULL,
			.GPIO_Speed = GPIO_Speed_100MHz
	};
	SPI_InitTypeDef SPI_InitStruct = {
			.SPI_CPHA = SPI_CPHA_2Edge,
			.SPI_CPOL = SPI_CPOL_High,
			.SPI_CRCPolynomial = 7,
			.SPI_DataSize = SPI_DataSize_8b,
			.SPI_Direction = SPI_Direction_2Lines_FullDuplex,
			.SPI_FirstBit = SPI_FirstBit_MSB,
			.SPI_Mode = SPI_Mode_Slave,
			.SPI_NSS = SPI_NSS_Hard
	};
	RCC->AHB1ENR |=setting->MISO.clock|setting->MOSI.clock|setting->NSS.clock|setting->SCK.clock;
	RCC->APB2ENR |= SNAP_CLK;

	GPIO_PinAFConfig(setting->MISO.PORT,setting->MISO.Source,setting->MISO.AF);
	GPIO_PinAFConfig(setting->MOSI.PORT,setting->MOSI.Source,setting->MOSI.AF);
	GPIO_PinAFConfig(setting->NSS.PORT,setting->NSS.Source,setting->NSS.AF);
	GPIO_PinAFConfig(setting->SCK.PORT,setting->SCK.Source,setting->SCK.AF);

	GPIO_InitStruct.GPIO_Pin = setting->MISO.Pin;
	GPIO_Init(setting->MISO.PORT,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = setting->MOSI.Pin;
	GPIO_Init(setting->MOSI.PORT,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = setting->NSS.Pin;
	GPIO_Init(setting->NSS.PORT,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = setting->SCK.Pin;
	GPIO_Init(setting->SCK.PORT,&GPIO_InitStruct);

	SPI_Init(SNAP_SPI,&SPI_InitStruct);
	SPI_SSOutputCmd(SNAP_SPI,DISABLE);
	SPI_Cmd(SNAP_SPI,ENABLE);
}
STATIC void SNAP_DMA_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
}
STATIC void SNAP_DMA_Transfer(SPI_DMA_Dev *setting,UInt8 *txbuffer, UInt8 *rxbuffer,UInt16 size)
{
	if(setting->RX_Stream->NDTR||setting->TX_Stream->NDTR
			||(txbuffer==NULL && rxbuffer==NULL))
		return ;
	UInt8 dummy = 0;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (UInt32) &SNAP_SPI->DR;
	DMA_InitStruct.DMA_BufferSize = size;

	DMA_InitStruct.DMA_Channel = setting->TX_Channel;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	if(txbuffer !=NULL)
	{
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)txbuffer; // transfer tx buffer
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable; // increase pointer;
	}else
	{
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&dummy; // transfer tx buffer
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable; // not increase pointer;
	}
	DMA_DeInit(setting->TX_Stream);
	DMA_Init(setting->TX_Stream,&DMA_InitStruct);

	DMA_InitStruct.DMA_Channel = setting->RX_Channel;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	if(rxbuffer !=NULL)
	{
		/*clear pending byte*/
		SPI_I2S_ReceiveData(SNAP_SPI);
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)rxbuffer; // transfer tx buffer
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable; // increase pointer;
	}else
	{
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&dummy; // transfer tx buffer
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable; // not increase pointer;
	}
	DMA_DeInit(setting->RX_Stream);
	DMA_Init(setting->RX_Stream,&DMA_InitStruct);

	setting->RX_Stream->CR |= DMA_SxCR_EN;
	setting->TX_Stream->CR |= DMA_SxCR_EN;
	SNAP_SPI->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
}

STATIC UInt8 SNAP_DMA_Check_Status(SPI_DMA_Dev *setting)
{
	if(
		setting->RX_Stream->NDTR ||
		setting->TX_Stream->NDTR || // check data stream zero
		SPI_IS_BUSY(SNAP_SPI))
			return 1;
		else
			return 0;
}
STATIC ErrorStatus Read_eeprom(UInt32 cam_bitmask, UInt16 addr,UInt16 size)
{
	CamDevice_TypeDef *pCam;
	UInt16 eeprom_idx=0;
	ErrorStatus retVal = ERROR;
	for(int i=0; i< CAM_NUM_CAMERA_MAX;i++)
	{
		pCam = CamDeviceTbl[i];
		/*Is camera valid ? or 'G' mbitmask*/
		if((cam_bitmask >> 1 & 1<<i) || (cam_bitmask & 0x01))
		{
			if((pCam != NULL) && (cam_eeprom_read(pCam, addr,( g_eeprom_buffer + eeprom_idx), size) == ERROR_NONE))
			{
				log_info("%s: read %d byte data from address %02x completed",pCam->Name,size,addr);
				retVal = SUCCESS; /* read successfully*/
			}
			else
			{
				log_error("%s read data fail",pCam->Name);
				retVal = ERROR;/* eeprom can't read*/
			}
			eeprom_idx += size;
		}
	}
	return retVal; /* mbitmask's not exist */
}
STATIC ErrorStatus Write_eeprom(UInt32 cam_bitmask,UInt16 addr,UInt16 size,UInt8 *data)
{
	CamDevice_TypeDef *pCam;
	ErrorStatus retVal = ERROR;
	for(int i=0; i< CAM_NUM_CAMERA_MAX;i++)
	{
		pCam = CamDeviceTbl[i];
		/*Is camera valid ?*/
		if((cam_bitmask >> 1 & 1<<i) || (cam_bitmask & 0x01))
		{
			if((pCam != NULL) && (cam_eeprom_write(pCam, addr, data, size) == ERROR_NONE))
			{
				log_info("%s write eeprom successfully",pCam->Name);
				retVal = SUCCESS; /* read successfully*/
			}
			else
			{
				log_error("%s write eeprom fail", pCam->Name);
				retVal = ERROR;/* eeprom can't read*/

			}
		}
	}
	return retVal; /* mbitmask's not exist */
}
void Temp_Init(void)
{
	UInt8 SlaveAddr[4]={0x48,0x49,0x4A,0x4B};
	I2CEx_Msg_t Temp_Sensor;
	UInt8 txbuf[4];
	UInt8 rxbuf[4];
	Temp_Sensor.txbuf.data = txbuf;
	Temp_Sensor.rxbuf.data = rxbuf;
	Temp_Sensor.ch = I2CEx_TEMP;
	Temp_Sensor.is_completed = &temp_status;
	for(int i=0;i<4;i++)
	{
		Temp_Sensor.addr =SlaveAddr[i]<<1;
		I2C_8BitAddr_WriteWord(&Temp_Sensor,0x01,0x61A0);
		I2C_8BitAddr_WriteByte(&Temp_Sensor,0x01,0x61|0x80);
	}
}

UInt16 Get_TempBoad(UInt8 SlaveAddr)
{
	UInt16 temp=0;
	I2CEx_Msg_t Temp_Sensor;
	UInt8 txbuf[4];
	UInt8 rxbuf[4];
	Temp_Sensor.txbuf.data = txbuf;
	Temp_Sensor.rxbuf.data = rxbuf;
	Temp_Sensor.ch = I2CEx_TEMP;

	Temp_Sensor.addr = SlaveAddr<<1;
	Temp_Sensor.is_completed = &temp_status;

	I2C_8BitAddr_WriteWord(&Temp_Sensor,0x01,0x61A0);
	I2C_8BitAddr_WriteByte(&Temp_Sensor,0x01,0x61|0x80);
	vTaskDelay(50);
	temp=I2C_8BitAddr_ReadByte(&Temp_Sensor,0x00)<<8;
	temp|=I2C_8BitAddr_ReadByte(&Temp_Sensor,0x01);
	temp >>=4;
    if(temp & 1<<11)
     {
         temp |= 0xF000;
     }
    temp = (int)temp *0.0625;
	return temp;
}
