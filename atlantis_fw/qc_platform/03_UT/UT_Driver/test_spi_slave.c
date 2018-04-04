/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    test_spi_slave.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-8-2016
 * @brief   This file contains Test SPI Slave source code
 *
 ******************************************************************************/
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>
#include <malloc.h>
#include <stdbool.h>

#include "spi_slave.c"

/**** Mock variable and memory ************************************************/
extern unsigned int MOCK_SPI1_BASE[32];
extern spi_config_t test_hal_spi_init;
extern int hal_spi_init_count;
extern spi_config_t test_hal_spi_interrupt_deinit;
extern int hal_spi_interrupt_deinit_count;
extern spi_config_t test_hal_spi_interrupt_init_conf;
extern spi_interrupt_init_t test_hal_spi_interrupt_init_interrupt;
extern int hal_spi_interrupt_init_count;
extern spi_config_t test_hal_spi_receive_buf;
extern int hal_spi_receive_buf_count;
int mock_rx_clbk_count;
uint8_t *mock_rx_clbk_data;
uint16_t mock_rx_clbk_len;
extern BaseType_t test_xHigherPriorityTaskWoken;
extern int xQueueGiveFromISR_count;
extern int portEND_SWITCHING_ISR_count;
extern int xQueueGenericSend_count;
/******************************************************************************/

void mock_rx_clbk (uint8_t *data, uint16_t len)
{
	mock_rx_clbk_count ++;
	mock_rx_clbk_data = data;
	mock_rx_clbk_len = len;
}

void unit_spi_slave_init(void **state)
{
	spi_slave_t spi_slave;
	spi_slave.rx_clbk = &mock_rx_clbk;
	hal_spi_init_count = 0;
	hal_spi_interrupt_deinit_count = 0;
	hal_spi_interrupt_init_count = 0;
	spi_slave_init(&spi_slave);
	assert_int_equal(test_hal_spi_init.channel,SSI_CH1);
	assert_int_equal(test_hal_spi_init.data_size,SPI_SLAVE_SPI_DATASIZE);
	assert_int_equal(test_hal_spi_init.spi_mode,SPI_SLAVE_SPI_DATA_MODE);
	assert_int_equal(test_hal_spi_init.spi_type,SPI_STD);
	assert_int_equal(test_hal_spi_init.ssi_mode,SSI_MOTO_SPI);
	assert_int_equal(test_hal_spi_init.transfer_mode,TX_AND_RX);
	assert_int_equal(test_hal_spi_init.master_clk_freq,SPI_SLAVE_SPI_CLOCK);
	assert_int_equal(test_hal_spi_init.master_data_frame,0);
	assert_int_equal(hal_spi_init_count,1);
	assert_int_equal(test_hal_spi_interrupt_deinit.channel,SSI_CH1);
	assert_int_equal(test_hal_spi_interrupt_deinit.data_size,\
			SPI_SLAVE_SPI_DATASIZE);
	assert_int_equal(test_hal_spi_interrupt_deinit.spi_mode,\
			SPI_SLAVE_SPI_DATA_MODE);
	assert_int_equal(test_hal_spi_interrupt_deinit.spi_type,SPI_STD);
	assert_int_equal(test_hal_spi_interrupt_deinit.ssi_mode,SSI_MOTO_SPI);
	assert_int_equal(test_hal_spi_interrupt_deinit.transfer_mode,TX_AND_RX);
	assert_int_equal(test_hal_spi_interrupt_deinit.master_clk_freq,\
			SPI_SLAVE_SPI_CLOCK);
	assert_int_equal(test_hal_spi_interrupt_deinit.master_data_frame,0);
	assert_int_equal(hal_spi_interrupt_deinit_count,1);
	assert_int_equal(test_hal_spi_interrupt_init_conf.channel,SSI_CH1);
	assert_int_equal(test_hal_spi_interrupt_init_conf.data_size,\
			SPI_SLAVE_SPI_DATASIZE);
	assert_int_equal(test_hal_spi_interrupt_init_conf.spi_mode,\
			SPI_SLAVE_SPI_DATA_MODE);
	assert_int_equal(test_hal_spi_interrupt_init_conf.spi_type,SPI_STD);
	assert_int_equal(test_hal_spi_interrupt_init_conf.ssi_mode,SSI_MOTO_SPI);
	assert_int_equal(test_hal_spi_interrupt_init_conf.transfer_mode,TX_AND_RX);
	assert_int_equal(test_hal_spi_interrupt_init_conf.master_clk_freq,\
			SPI_SLAVE_SPI_CLOCK);
	assert_int_equal(test_hal_spi_interrupt_init_conf.master_data_frame,0);
	assert_int_equal(hal_spi_interrupt_init_count,1);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			rx_fifo_threshold,15);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			tx_fifo_threshold,0);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			rx_fifo_full,DISABLE);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			tx_fifo_empty,DISABLE);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			rx_fifo_overflow,DISABLE);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			rx_fifo_underflow,DISABLE);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			tx_fifo_overflow,DISABLE);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			multi_master_contention,DISABLE);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			callback_handler.rx_fifo_full_handler,&spi_slaver_rx_complete);
	assert_int_equal(test_hal_spi_interrupt_init_interrupt.\
			callback_handler.tx_fifo_empty_handler,&spi_slaver_tx_complete);
	assert_null(test_hal_spi_interrupt_init_interrupt.\
			callback_handler.rx_fifo_overflow_handler);
	assert_null(test_hal_spi_interrupt_init_interrupt.\
			callback_handler.rx_fifo_underflow_handler);
	assert_null(test_hal_spi_interrupt_init_interrupt.\
			callback_handler.tx_fifo_overflow_handler);
	assert_null(test_hal_spi_interrupt_init_interrupt.\
			callback_handler.multi_master_handler);
}

void unit_spi_slaver_rx_complete(void **state)
{
	hal_spi_receive_buf_count = 0;
	mock_rx_clbk_count = 0;
	mock_rx_clbk_len = 0;
	spi_slaver_rx_complete();
	assert_int_equal(test_hal_spi_receive_buf.channel,SSI_CH1);
	assert_int_equal(test_hal_spi_receive_buf.data_size,\
			SPI_SLAVE_SPI_DATASIZE);
	assert_int_equal(test_hal_spi_receive_buf.spi_mode,\
			SPI_SLAVE_SPI_DATA_MODE);
	assert_int_equal(test_hal_spi_receive_buf.spi_type,SPI_STD);
	assert_int_equal(test_hal_spi_receive_buf.ssi_mode,SSI_MOTO_SPI);
	assert_int_equal(test_hal_spi_receive_buf.transfer_mode,TX_AND_RX);
	assert_int_equal(test_hal_spi_receive_buf.master_clk_freq,\
			SPI_SLAVE_SPI_CLOCK);
	assert_int_equal(test_hal_spi_receive_buf.master_data_frame,0);
	for (uint16_t i=0; i<rx_len; i++)
	{
		assert_int_equal(*(rx_data+i),(1 + i));
	}
	assert_int_equal(hal_spi_receive_buf_count,1);
	assert_int_equal(mock_rx_clbk_count,1);
	assert_int_equal(mock_rx_clbk_len,16);
	for (uint16_t i=0; i<rx_len; i++)
	{
		assert_int_equal(*(mock_rx_clbk_data+i),*(rx_data + i));
	}
}

void unit_spi_slaver_tx_complete (void **state)
{
	hal_spi_interrupt_init_count = 0 ;
	portEND_SWITCHING_ISR_count = 0;
	MOCK_SPI1_BASE[8] = 0;
	xQueueGiveFromISR_count = 0;
	test_xHigherPriorityTaskWoken = pdTRUE;
	spi_slaver_tx_complete();
	assert_int_equal(xQueueGiveFromISR_count,1);
	assert_int_equal(hal_spi_interrupt_init_count,1);
	assert_int_equal(portEND_SWITCHING_ISR_count,1);

	hal_spi_interrupt_init_count = 0 ;
	portEND_SWITCHING_ISR_count = 0;
	MOCK_SPI1_BASE[8] = 0;
	xQueueGiveFromISR_count = 0;
	test_xHigherPriorityTaskWoken = pdFALSE;
	spi_slaver_tx_complete();
	assert_int_equal(xQueueGiveFromISR_count,1);
	assert_int_equal(hal_spi_interrupt_init_count,1);
	assert_int_equal(portEND_SWITCHING_ISR_count,0);

	hal_spi_interrupt_init_count = 0 ;
	portEND_SWITCHING_ISR_count = 0;
	MOCK_SPI1_BASE[8] = 1;
	xQueueGiveFromISR_count = 0;
	test_xHigherPriorityTaskWoken = pdTRUE;
	spi_slaver_tx_complete();
	assert_int_equal(xQueueGiveFromISR_count,0);
	assert_int_equal(hal_spi_interrupt_init_count,0);
	assert_int_equal(portEND_SWITCHING_ISR_count,0);
}

void unit_spi_slave_tx (void **state)
{
	uint8_t data[]={1,3,2,5};
	uint16_t len = 4;
	uint8_t tx_en;
	tx_en = FALSE;
	xQueueGenericSend_count = 0;
	spi_slave_tx(&data[0],len, tx_en);
	for (int i=0; i<len; i++)
	{
		assert_int_equal(tx_data[i],data[i]);
	}
	assert_int_equal(sm,SM_RX);
	assert_int_equal(xQueueGenericSend_count,1);

	xQueueGenericSend_count = 0;
	tx_en = TRUE;
	spi_slave_tx(&data[0],len, tx_en);
	assert_int_equal(sm,SM_TX);
	assert_int_equal(xQueueGenericSend_count,1);
}

void unit_spi_slave_release_semaphore(void **state)
{
	xQueueGenericSend_count = 0;
	spi_slave_release_semaphore();
	assert_int_equal(xQueueGenericSend_count,1);
}

int main(void)
{
	const struct CMUnitTest tests[] = {
			cmocka_unit_test(unit_spi_slave_init),
			cmocka_unit_test(unit_spi_slaver_rx_complete),
			cmocka_unit_test(unit_spi_slaver_tx_complete),
			cmocka_unit_test(unit_spi_slave_tx),
			cmocka_unit_test(unit_spi_slave_release_semaphore),
	};
	cmocka_set_message_output(CM_OUTPUT_XML);
	cmocka_run_group_tests_name("test_spi_slave",tests, NULL, NULL);
	return 0;
}
