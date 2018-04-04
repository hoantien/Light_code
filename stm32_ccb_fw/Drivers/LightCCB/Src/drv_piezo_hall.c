/*
 * drv_piezo_hall.c
 *
 * TODO: API to control stats gathering
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "drv_piezo.h"
#include "drv_piezo_hall.h"
#include "hal_i2c_ex.h"
#include "hal_flash_ex.h"

#include "stm32f4xx_i2c.h"
#include "stm32f4xx.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"



/* Private typedef -----------------------------------------------------------*/

void   InitHallSensor(HallSensor *h);

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// lens and mirror actuator mapping table

// Hall sensor data encoding bitfields
#ifdef BOARD_VERSION_P1
// Configuration options
#define READ_HALL_AS_WORD			 1	// Use a single 16-bit I2C read when set - otherwise 2 8's
#define ENABLE_HALL_RETRY_HISTOGRAM  0	// Track the distribution of the number of attempts needed
										// to successfully read a value from the Hall sensor.
#define ENABLE_HALL_FAST_PATH		 0  // Bypass the normal I2CEx driver and use straightline, unpreempted code
#define MAX_RETRIES 				10

#define HALL_OUTPUT_OCF_RANGE        11:11
#define HALL_OUTPUT_PARITY_RANGE     10:10
#define HALL_OUTPUT_POSITION_RANGE    9: 0
#define HALL_CONFIG_POLARITY_RANGE    1: 1
#define HALL_CONFIG_POWERDOWN_RANGE   0: 0
#define HALL_CONFIG_SPEED_RANGE       2: 2
#define HALL_SENSE_SENSITIVITY_RANGE  1: 0

#define HALL_OUTPUT_REG   0x00 // Base register for 16-bit read
#define HALL_OUTPUT0_REG  0x00 // Lower byte of output
#define HALL_OUTPUT1_REG  0x01 // Upper byte of output + flags
#define HALL_CONFIG_REG   0x02
#define HALL_SENSE_REG    0x0b // TODO: or 0x08\?
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1
// Configuration options
#define READ_HALL_AS_WORD			 0	// Use a single 16-bit I2C read when set - otherwise 2 8's
#define ENABLE_HALL_RETRY_HISTOGRAM  0	// Track the distribution of the number of attempts needed
										// to successfully read a value from the Hall sensor.
#define ENABLE_HALL_FAST_PATH		 0  // Bypass the normal I2CEx driver and use straightline, unpreempted code
#define MAX_RETRIES 				10

#define HALL_MIRROR_OUTPUT_OCF_RANGE        11:11
#define HALL_MIRROR_OUTPUT_PARITY_RANGE     10:10
#define HALL_MIRROR_OUTPUT_POSITION_RANGE    9: 0
#define HALL_MIRROR_CONFIG_POLARITY_RANGE    1: 1
#define HALL_MIRROR_CONFIG_POWERDOWN_RANGE   0: 0
#define HALL_MIRROR_CONFIG_SPEED_RANGE       2: 2

#define HALL_MIRROR_OUTPUT_REG		0x00 // Base register for 16-bit read
#define HALL_MIRROR_OUTPUT0_REG		HALL_MIRROR_OUTPUT_REG // Lower byte of output
#define HALL_MIRROR_OUTPUT1_REG		(HALL_MIRROR_OUTPUT_REG + 0x01) // Upper byte of output + flags
#define HALL_MIRROR_CONFIG_REG		0x02
#define HALL_MIRROR_SENSE_REG		0x0b
#define HALL_LENS_OUTPUT_REG		0x05 // Base register for 16-bit read
#define HALL_LENS_OUTPUT0_REG		HALL_LENS_OUTPUT_REG // Lower byte of output
#define HALL_LENS_OUTPUT1_REG		(HALL_LENS_OUTPUT_REG + 0x01) // Upper byte of output + flags
#define HALL_LENS_PIDCTL0_REG		0x16
#define HALL_LENS_PIDCTL5_REG		0x1B
#define HALL_LENS_FACTRES3_REG		0x1F

#define HALL_MIRROR_SENSITIVITY_MAX		3
#define HALL_LENS_SENSITIVITY_MAX		7
#endif /* BOARD_VERSION_P1_1 */

// TODO: Move these macros to somewhere more global
#define HI_BIT(r)       (1?r)
#define LO_BIT(r)       (0?r)
#define MASK(n)         ((1<<(n))-1)
#define NBITS(r)        (HI_BIT(r) - LO_BIT(r) + 1)
#define GET_FIELD(x, r) (((x) >> LO_BIT(r)) & MASK(NBITS(r)))


__IO Int8 transfer_status = 0;

#define DECLARE_I2C_BUFFER(tsize, rsize) \
    I2CEx_Msg_t I2CBuffer;               \
    UInt8       txbuf[(tsize)];          \
    UInt8       rxbuf[(rsize)];

#define PREPARE_I2C_BUFFER(channel, address)   \
    I2CBuffer.txbuf.data = txbuf;              \
    I2CBuffer.rxbuf.data = rxbuf;              \
    I2CBuffer.is_completed = &transfer_status; \
    I2CBuffer.ch = (channel);                  \
    I2CBuffer.addr = (address);


// Statistics support
static UInt8  HallRetryStatsEnable = ENABLE_HALL_RETRY_HISTOGRAM;
static UInt16 HallRetryTotalSamples = 0;
static UInt16 HallRetryHistogram[MAX_RETRIES + 1];
static UInt16 HallSlowPathCount = 0;

void InitHallSensor(HallSensor *h)
{
#ifdef BOARD_VERSION_P1_1
	uint8_t slave_address = 0;
	slave_address = h->I2CSlaveAddress >> 1;
#endif /* BOARD_VERSION_P1_1 */


	DECLARE_I2C_BUFFER(2, 2);

	//ENTER_FUNC

	PREPARE_I2C_BUFFER(h->I2CExpanderChannel, h->I2CSlaveAddress);

#ifdef BOARD_VERSION_P1
	if(h->IsInitialized)
	        return;
	// Set PD=0 (not powered down), Polarity=h->Polarity, Fast/Slow mode = 1 (slower, lower noise)
	I2C_8BitAddr_WriteByte(&I2CBuffer, 0x02, 0x04 | ((h->Polarity & 0x1) << 1));
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
	/* Check for Mirror Hall Sensor - AS5510 */
	if (slave_address == MIRROR_ADDR)
	{
		// Set PD=0 (not powered down), Polarity=h->Polarity, Fast/Slow mode = 1 (slower, lower noise)
		I2C_8BitAddr_WriteByte(&I2CBuffer, 0x02, 0x04 | ((h->Polarity & 0x1) << 1));
	}
	/* Lens Hall Sensor A1457 */
	else
	{
		uint8_t PIDCTL5_val = 0;

		// Get the PIDCTL5 register
        //unsigned long time_start = 0, time_end = 0, total_time = 0;
        //time_start = xTaskGetTickCount();
		PIDCTL5_val = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_LENS_PIDCTL5_REG);
        //time_end = xTaskGetTickCount();
        //total_time = (time_end - time_start);
        //printf("Reading one byte from the hall sensor took %ld milliseconds.\n\n", total_time);
		// Set polarity = h->Polarity
		PIDCTL5_val  = (PIDCTL5_val & ~(0x01 << 7)) | ((h->Polarity & 0x1) << 7);
		I2C_8BitAddr_WriteByte(&I2CBuffer, HALL_LENS_PIDCTL5_REG, PIDCTL5_val);
	}
#endif /* BOARD_VERSION_P1_1 */

	SetHallSensorSensitivity(h, h->Sensitivity);
	h->IsInitialized = 1;

	vTaskDelay(10); // Delay to ensure this takes effect.  Just a guess - likely not needed.

	//EXIT_FUNC
}

// Really a boolean.  True = connected, False = no connectivity.
int CheckHallConnectivity(HallSensor *h)
{
#ifdef BOARD_VERSION_P1_1
	uint8_t slave_address = 0;
	slave_address = h->I2CSlaveAddress >> 1;
#endif /* BOARD_VERSION_P1_1 */
	DECLARE_I2C_BUFFER(2, 2);
	PREPARE_I2C_BUFFER(h->I2CExpanderChannel, h->I2CSlaveAddress);
#ifdef BOARD_VERSION_P1
	I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_OUTPUT_REG);
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
	/* Check for Mirror Hall Sensor - AS5510 */
	if (slave_address == MIRROR_ADDR)
	{
		I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_MIRROR_OUTPUT_REG);
	}
	/* Lens Hall Sensor A1457 */
	else
	{
		I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_LENS_OUTPUT_REG);
	}
#endif /* BOARD_VERSION_P1_1 */
	return (*I2CBuffer.is_completed >= 0);
}

void SetHallSensorSensitivity(HallSensor *h, UInt8 Sensitivity)
{
#ifdef BOARD_VERSION_P1_1
	uint8_t slave_address = 0;
	slave_address = h->I2CSlaveAddress >> 1;
#endif /* BOARD_VERSION_P1_1 */
	DECLARE_I2C_BUFFER(2, 2);
	PREPARE_I2C_BUFFER(h->I2CExpanderChannel, h->I2CSlaveAddress);

#ifdef BOARD_VERSION_P1
	Sensitivity = Sensitivity & MASK(NBITS(HALL_SENSE_SENSITIVITY_RANGE));
    h->Sensitivity = Sensitivity;
    // This routine does not save the sensitivity value inside the Hall sensor struct.
	I2C_8BitAddr_WriteByte(&I2CBuffer, HALL_SENSE_REG, Sensitivity);
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
	/* Check for Mirror Hall Sensor - AS5510 */
	if(slave_address == MIRROR_ADDR)
	{
		if (Sensitivity > HALL_MIRROR_SENSITIVITY_MAX)
		{
			Sensitivity = HALL_MIRROR_SENSITIVITY_MAX;
		}
		h->Sensitivity = Sensitivity;
		// This routine does not save the sensitivity value inside the Hall sensor struct.
		I2C_8BitAddr_WriteByte(&I2CBuffer, HALL_MIRROR_SENSE_REG, Sensitivity);
	}
	/* Lens Hall Sensor A1457 */
	else
	{
		uint8_t PIDCTL5_val = 0;	// HGAIN[1:0] (bit 5:4), address: 0x1B
		uint8_t FACTRES3_val = 0;	// HGAIN[2]	  (bit 6), address: 0x1F
		uint8_t PIDCTL0_val = 0;	// LOCKF	  (bit 7)

		if (Sensitivity > HALL_LENS_SENSITIVITY_MAX)
		{
			Sensitivity = HALL_LENS_SENSITIVITY_MAX;
		}
		/* Read the PIDCTL0 register */
		PIDCTL0_val = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_LENS_PIDCTL0_REG);
		/* Unlock the FACTRES3 register */
		PIDCTL0_val &= ~(1 << 7);
		I2C_8BitAddr_WriteByte(&I2CBuffer, HALL_LENS_PIDCTL0_REG, PIDCTL0_val);

		/* Read the registers containing the sensitivity */
		PIDCTL5_val = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_LENS_PIDCTL5_REG);
		FACTRES3_val = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_LENS_FACTRES3_REG);

		/* Configure sensitivity with "Sensitivity" argument */
		/* Configure bit [5:4] : HGAIN[1:0] */
		PIDCTL5_val  = (PIDCTL5_val & ~(0x03 << 4)) | ((Sensitivity & 0x03) << 4);
		/* configure bit 6 HGAIN[2] */
		FACTRES3_val = (FACTRES3_val & ~(0x01 << 6)) | ((Sensitivity & 0x04) << 4);

		/* Write to registers */
		I2C_8BitAddr_WriteByte(&I2CBuffer, HALL_LENS_PIDCTL5_REG, PIDCTL5_val);
		I2C_8BitAddr_WriteByte(&I2CBuffer, HALL_LENS_FACTRES3_REG, FACTRES3_val);

		/* Lock the FACTRES3 register */
		PIDCTL0_val |= (1 << 7);
		I2C_8BitAddr_WriteByte(&I2CBuffer, HALL_LENS_PIDCTL0_REG, PIDCTL0_val);
	}

#endif /* BOARD_VERSION_P1_1 */

}

// Returns 1 if the # of 1 bits is odd, 0 if even.
static inline UInt8 ComputeEvenParity(UInt16 x)
{
	UInt16 NumOnes = 0;
	while(x)
	{
		NumOnes++;
		x = x & (x-1);
	}

	return (NumOnes & 0x1);
}

#if ENABLE_HALL_FAST_PATH
// This is a hack to see just how fast we can read from the Hall sensor.
// It is not very safe...

#define I2C_TIMEOUT 20000
UInt16 SpecialI2CReadA8D16(UInt8 SlaveAddress, UInt8 RegAddress)
{
	UInt8  rxbuf[2];
	UInt16 Value;
	UInt16 Timeout;

	taskENTER_CRITICAL();

	// Start, transmitter mode, ack enable
	// Write register address
	// Stop
	// Start, receiver mode, ack enable
	// Read w/Ack
	// Read w/Nack

	// START
	I2C_GenerateSTART(I2CEx, ENABLE);

	// Send SlaveAddress w/write
	// Wait for I2C to be busy
	Timeout = I2C_TIMEOUT;
	while (!(I2CEx->SR1 & I2C_SR1_SB))
	{
		if (--Timeout == 0)
		{
			log_error("Timeout1");
			goto fail;
		}
	}

	I2C_AcknowledgeConfig(I2CEx, ENABLE); // Enable ACK
	I2C_SendData(I2CEx, SlaveAddress);

	Timeout = I2C_TIMEOUT;
	while (!(I2CEx->SR1 & I2C_SR1_ADDR)) // Wait until finished
	{
		if (--Timeout == 0)
		{
			log_error("Timeout2");
			goto fail;
		}
	}

	// Read status register to clear ADDR flag.
	Value = I2CEx->SR2;

	// Send RegAddress
	Timeout = I2C_TIMEOUT;
	while (!(I2CEx->SR1 & I2C_SR1_TXE))
	{
		if (--Timeout == 0)
		{
			log_error("Timeout3");
			goto fail;
		}
	}
	I2C_SendData(I2CEx, RegAddress);

	// Stop
	Timeout = I2C_TIMEOUT;
	while(!(I2CEx->SR1 & I2C_SR1_TXE) || !(I2CEx->SR1 & I2C_SR1_BTF))
	{
		if (--Timeout == 0)
		{
			log_error("Timeout4");
			goto fail;
		}
	}

	I2C_GenerateSTOP(I2CEx, ENABLE);


	// START
	I2C_GenerateSTART(I2CEx, ENABLE);
	Timeout = I2C_TIMEOUT;
	while (!(I2CEx->SR1 & I2C_SR1_SB))
	{
		if (--Timeout == 0)
		{
			log_error("Timeout5");
			goto fail;
		}
	}

	I2C_AcknowledgeConfig(I2CEx, ENABLE); // Enable ACK
	// Send Slave Address w/read
	I2C_SendData(I2CEx, SlaveAddress | 0x1);
	Timeout = I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2CEx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if (--Timeout == 0)
		{
			log_error("Timeout6");
			goto fail;
		}
	}

	// Read status register to clear ADDR flag.
	Value = I2CEx->SR2;

	// Read byte 1
	// Enable ACK
	I2C_AcknowledgeConfig(I2CEx, ENABLE);
	Timeout = I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2CEx, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		if (--Timeout == 0)
		{
			log_error("Timeout7");
			goto fail;
		}
	}

	rxbuf[0] = I2C_ReceiveData(I2CEx);

	// Read byte 2
	I2C_AcknowledgeConfig(I2CEx, DISABLE);
	I2C_GenerateSTOP(I2CEx, ENABLE);
	Timeout = I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2CEx, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		if (--Timeout == 0)
		{
			log_error("Timeout8");
			goto fail;
		}
	}

	rxbuf[1] = I2C_ReceiveData(I2CEx);
#ifdef BOARD_VERSION_P1
	Value = (rxbuf[1] << 8) | rxbuf[0];
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1
	if ((SlaveAddress >> 1) == MIRROR_ADDR)
	{
		Value = (rxbuf[1] << 8) | rxbuf[0];
	}
	else
	{
		Value = (rxbuf[0] << 4) | rxbuf[1];
	}
#endif /* BOARD_VERSION_P1_1 */

	taskEXIT_CRITICAL();

	return Value;

fail:
	taskEXIT_CRITICAL();
	return 0;

}

// This is temporary - it's a variable inside the I2CEx code.
extern I2CEx_Ch_t Current_Ch;
#endif // #if ENABLE_HALL_FAST_PATH

UInt16 ReadHallSensor(HallSensor *h)
{
	UInt16 Value;
	UInt16 Position;
#ifdef BOARD_VERSION_P1
	UInt8  OffsetCompStatus;
	UInt8  Parity;
	UInt8  ParityError;
#endif /* BOARD_VERSION_P1 */
	UInt8  ReadCount = 0; // Number of attempts made
#ifdef BOARD_VERSION_P1_1
	UInt8  SlaveAddress = h->I2CSlaveAddress >> 1;
#endif /* BOARD_VERSION_P1_1 */

	DECLARE_I2C_BUFFER(2, 3);
	PREPARE_I2C_BUFFER(h->I2CExpanderChannel, h->I2CSlaveAddress);

#ifdef BOARD_VERSION_P1
	do
	{
		ReadCount++;

#if READ_HALL_AS_WORD
		{
			UInt8 byte0;
			UInt8 byte1;
			byte0 = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_OUTPUT_REG);
			if (I2CBuffer.is_completed < 0)
			{
				log_error("Hall sensor read returned an error.");
			}
			byte1 = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_OUTPUT_REG+1);
			if (I2CBuffer.is_completed < 0)
			{
				log_error("Hall sensor read returned an error.");
			}
			Value = (byte1 << 8) | byte0;
		}
#endif // #if READ_HALL_AS_WORD

		OffsetCompStatus = GET_FIELD(Value, HALL_OUTPUT_OCF_RANGE);
		Parity           = GET_FIELD(Value, HALL_OUTPUT_PARITY_RANGE);
		Position         = GET_FIELD(Value, HALL_OUTPUT_POSITION_RANGE);

		ParityError = Parity != ComputeEvenParity(Position);
	} while ((!OffsetCompStatus || ParityError) && (ReadCount <= MAX_RETRIES));
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
	/* Check for Mirror Hall Sensor - AS5510 */
	if (SlaveAddress == MIRROR_ADDR)
	{
		UInt8	Parity;
		UInt8	OffsetCompStatus;
		UInt8	ParityError;

		do
		{
			ReadCount++;

#if READ_HALL_AS_WORD

#if !ENABLE_HALL_FAST_PATH
			// Read the Hall sensor as a 16-bit value.
            //unsigned long time_start = 0, time_end = 0, total_time = 0;
            //time_start = xTaskGetTickCount();
            I2C_8BitAddr_ReadWord(&I2CBuffer, HALL_MIRROR_OUTPUT_REG);
            //time_end = xTaskGetTickCount();
            //total_time = (time_end - time_start);
            log_time("Reading one byte from the hall sensor took %ld milliseconds.\n\n", total_time);
			if (I2CBuffer.is_completed < 0)
			{
				log_error("Hall sensor read returned an error.");
			}
			// Assemble the read data from the buffer, as the value is
			// stored little-endian but I2C_8BitAddr_ReadWord() assembles
			// the data big-endian.
			Value = (rxbuf[1] << 8) | rxbuf[0];
#else // #if !ENABLE_HALL_FAST_PATH
			if (Current_Ch != h->I2CExpanderChannel)
			{
				// Do the usual.
				// Read the Hall sensor as a 16-bit value.
                //unsigned long time_start = 0, time_end = 0, total_time = 0;
                //time_start = xTaskGetTickCount();
				I2C_8BitAddr_ReadWord(&I2CBuffer, HALL_MIRROR_OUTPUT_REG);
                //time_end = xTaskGetTickCount();
                //total_time = (time_end - time_start);
                //log_time("Reading one byte from the hall sensor took %ld milliseconds.\n\n", total_time);
				if (I2CBuffer.is_completed < 0)
				{
					log_error("Hall sensor read returned an error.");
				}
				// Assemble the read data from the buffer, as the value is
				// stored little-endian but I2C_8BitAddr_ReadWord() assembles
				// the data big-endian.
				Value = (rxbuf[1] << 8) | rxbuf[0];
				if (HallRetryStatsEnable)
				{
					HallSlowPathCount++;
				}
			}
			else
			{
				Value = SpecialI2CReadA8D16(h->I2CSlaveAddress, HALL_MIRROR_OUTPUT_REG);
			}
#endif // #if !ENABLE_HALL_FAST_PATH

#else // #if READ_HALL_AS_WORD
			{
				UInt8 byte0;
				UInt8 byte1;
                //unsigned long time_start = 0, time_end = 0, total_time = 0;
                //time_start = xTaskGetTickCount();
				byte0 = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_MIRROR_OUTPUT0_REG);
                //time_end = xTaskGetTickCount();
                //total_time = (time_end - time_start);
                //log_time("Reading one byte from the hall sensor took %ld milliseconds.\n\n", total_time);
				if (I2CBuffer.is_completed < 0)
				{
					log_error("Hall sensor read returned an error.");
				}
				byte1 = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_MIRROR_OUTPUT1_REG);
				if (I2CBuffer.is_completed < 0)
				{
					log_error("Hall sensor read returned an error.");
				}
				Value = (byte1 << 8) | byte0;
			}
#endif // #if READ_HALL_AS_WORD

			OffsetCompStatus	= GET_FIELD(Value, HALL_MIRROR_OUTPUT_OCF_RANGE);
			Parity				= GET_FIELD(Value, HALL_MIRROR_OUTPUT_PARITY_RANGE);
			Position			= GET_FIELD(Value, HALL_MIRROR_OUTPUT_POSITION_RANGE);

			ParityError			= (Parity != ComputeEvenParity(Position));
		} while ((!OffsetCompStatus || ParityError) && (ReadCount <= MAX_RETRIES));
	}
	/* Lens Hall Sensor A1457 */
	else
	{
#if READ_HALL_AS_WORD

#if !ENABLE_HALL_FAST_PATH
		// Read the Hall sensor as a 16-bit value.
		I2C_8BitAddr_ReadWord(&I2CBuffer, HALL_LENS_OUTPUT_REG);
		if (I2CBuffer.is_completed < 0)
		{
			log_error("Hall sensor read returned an error.");
		}
		// Assemble the read data from the buffer, as the value is
		// stored little-endian but I2C_8BitAddr_ReadWord() assembles
		// the data big-endian.
		Value = (rxbuf[0] << 4) | rxbuf[1];
#else // #if !ENABLE_HALL_FAST_PATH
		if (Current_Ch != h->I2CExpanderChannel)
		{
			// Do the usual.
			// Read the Hall sensor as a 16-bit value.
			I2C_8BitAddr_ReadWord(&I2CBuffer, HALL_LENS_OUTPUT_REG);
			if (I2CBuffer.is_completed < 0)
			{
				log_error("Hall sensor read returned an error.");
			}
			// Assemble the read data from the buffer, as the value is
			// stored little-endian but I2C_8BitAddr_ReadWord() assembles
			// the data big-endian.
			Value = (rxbuf[0] << 4) | rxbuf[1];
			if (HallRetryStatsEnable)
			{
				HallSlowPathCount++;
			}
		}
		else
		{
			Value = SpecialI2CReadA8D16(h->I2CSlaveAddress, HALL_LENS_OUTPUT_REG);
		}
#endif // #if !ENABLE_HALL_FAST_PATH

#else // #if READ_HALL_AS_WORD
		{
			UInt8 byte0;
			UInt8 byte1;
			byte0 = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_LENS_OUTPUT0_REG);
			if (I2CBuffer.is_completed < 0)
			{
				log_error("Hall sensor read returned an error.");
			}
			byte1 = I2C_8BitAddr_ReadByte(&I2CBuffer, HALL_LENS_OUTPUT1_REG);
			if (I2CBuffer.is_completed < 0)
			{
				log_error("Hall sensor read returned an error.");
			}
			Value = (byte0 << 4) | byte1;
		}
#endif // #if READ_HALL_AS_WORD

		Position = (Value + 0x0800) & 0x0FFF;
		log_printf("%s[%d]: Position: %x\n\r", __FUNCTION__, __LINE__, Position);
	}
#endif /* BOARD_VERSION_P1_1 */

	if (HallRetryStatsEnable)
	{
		HallRetryTotalSamples++;
		HallRetryHistogram[ReadCount]++;
	}

	// Verify that MSBs are cleared & OffsetCompStatus = 1
#ifdef BOARD_VERSION_P1
	if ((Value & 0xf800) != 0x0800)
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
	if ((SlaveAddress == MIRROR_ADDR) && ((Value & 0xf800) != 0x0800))
#endif /* BOARD_VERSION_P1_1 */
	{
		log_error("Hall sensor returned 0x%04x!", Value);
	}

	if (ReadCount == MAX_RETRIES)
	{
		log_error("Warning: Hall sensor data unreliable.\n");
	}

	return Position;

}


void PrintHallRetryStats()
{
	int i;

	if (HallRetryStatsEnable)
	{
		log_debug("Hall slow path count: %d", HallSlowPathCount);
		log_debug("Hall retry histogram (%d samples)", HallRetryTotalSamples);
		for (i = 1; i <= MAX_RETRIES; i++)
		{
			log_debug("hist:,%d,%d", i, HallRetryHistogram[i]);
		}
	}
}



