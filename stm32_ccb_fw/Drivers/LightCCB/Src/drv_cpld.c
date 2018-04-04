#include "drv_cpld.h"
#include "CCBConfig.h"

/**
 * Start or stop transmit and receive
 */
STATIC void cpld_start_stop(int start)
{
	if(start)
		GPIO_ResetBits(CPLD_SS_GPIO_PORT, CPLD_SS_PIN);
	else
		GPIO_SetBits(CPLD_SS_GPIO_PORT, CPLD_SS_PIN);
}

/**
 * Transmit and receive data
 */
STATIC void cpld_tx_rx(unsigned char *txbuf, unsigned char *rxbuf, int count, int trans)
{
	if(trans)
	{
		for(int i = 0; i < count; i++)
		{
			while(SPI_I2S_GetFlagStatus(CPLD, SPI_I2S_FLAG_BSY) == SET);
			while(SPI_I2S_GetFlagStatus(CPLD, SPI_I2S_FLAG_TXE) == RESET);
			SPI_I2S_SendData(CPLD, txbuf[i]);
			while(SPI_I2S_GetFlagStatus(CPLD, SPI_I2S_FLAG_RXNE) == RESET);
			rxbuf[i] = SPI_I2S_ReceiveData(CPLD);
		}
	}
	else
	{
		for(int i = 0; i < count; i ++)
		{
			while(SPI_I2S_GetFlagStatus(CPLD, SPI_I2S_FLAG_BSY) == SET);
			while(SPI_I2S_GetFlagStatus(CPLD, SPI_I2S_FLAG_TXE) == RESET);
			SPI_I2S_SendData(CPLD, rxbuf[i]);
			while(SPI_I2S_GetFlagStatus(CPLD, SPI_I2S_FLAG_RXNE) == RESET);
			rxbuf[i] = SPI_I2S_ReceiveData(CPLD);
		}
	}

}

/**
 * Get CPLD version
 */
STATIC int get_version(volatile uint8_t *major, volatile uint8_t *minor)
{
	uint8_t txbuf[3] = {0, 0, 0};
	uint8_t rxbuf[3] = {0, 0, 0};
	txbuf[0] = CPLD_READ;
	txbuf[1] = 0x01;
	txbuf[2] = 0xFF;
	GPIO_ResetBits(CPLD_CONFIG_GPIO_PORT, CPLD_CONFIG_PIN);
	cpld_start_stop(1);
	cpld_tx_rx(txbuf, rxbuf, 3, 1);
	cpld_tx_rx(txbuf, rxbuf, 3, 0);
	cpld_start_stop(0);
	*major = rxbuf[0];

	txbuf[0] = CPLD_READ;
	txbuf[1] = 0x00;
	txbuf[2] = 0xFF;
	cpld_start_stop(1);
	cpld_tx_rx(txbuf, rxbuf, 3, 1);
	cpld_tx_rx(txbuf, rxbuf, 3, 0);
	cpld_start_stop(0);
	*minor = rxbuf[0];

	GPIO_SetBits(CPLD_CONFIG_GPIO_PORT, CPLD_CONFIG_PIN);
	return 1;
}

/**
 * Reset CPLD
 */
STATIC void cpld_reset(void)
{
	uint8_t txbuf[3] = {0, 0, 0};
	uint8_t rxbuf[3] = {0, 0, 0};
	GPIO_ResetBits(CPLD_CONFIG_GPIO_PORT, CPLD_CONFIG_PIN);

	for(uint8_t idx=0; idx<4; idx++)
	{
		txbuf[0] = CPLD_WRITE;
		txbuf[1] = 2 + idx;
		txbuf[2] = 0x00;

		cpld_start_stop(1);
		cpld_tx_rx(txbuf, rxbuf, 3, 1);
		cpld_start_stop(0);
	}


	GPIO_SetBits(CPLD_CONFIG_GPIO_PORT, CPLD_CONFIG_PIN);
}

void cpld_get_version(volatile uint8_t *major, volatile uint8_t *minor)
{
	cpld_reset();
	get_version(major, minor);
}
