/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    test_i2c_slave.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-8-2016
 * @brief   This file contains Test I2C Slave source code
 *
 ******************************************************************************/
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#include "i2c_slave.c"

int mock_clbk_hdl_count;
int mock_receiver_hdl_count;
int mock_transmitter_hdl_count;
extern hal_i2c_t test_hal_i2c_init;
extern int hal_i2c_init_count;
extern hal_i2c_t test_hal_i2c_enable_irq;
extern int hal_i2c_enable_irq_count;
extern int hal_i2c_slave_tx_count;

/* Mock function -------------------------------------------------------------*/
void mock_clbk_hdl(uint8_t data)
{
	mock_clbk_hdl_count ++;
}

void mock_receiver_hdl(void)
{
	mock_receiver_hdl_count ++;
}

void mock_transmitter_hdl(void)
{
	mock_transmitter_hdl_count ++;
}

/* Unit test functions -------------------------------------------------------*/
/**
 * @brief unit_i2c_slave_init
 * The function shall test i2c slave init function
 * @return None
 */
void unit_i2c_slave_init(void **state)
{
	i2c_slave_t i2c_slave;
	i2c_slave.clbk_hdl = &mock_clbk_hdl;
	i2c_slave.receiver_hdl = &mock_receiver_hdl;
	i2c_slave.transmitter_hdl = &mock_transmitter_hdl;
	hal_i2c_init_count = 0;
	hal_i2c_enable_irq_count = 0;
	tx_buf = NULL;
	rx_hdl = NULL;
	receiver_hdl = NULL;
	transmitter_hdl = NULL;
	i2c_slave_init(&i2c_slave);
	assert_int_equal(hal_i2c_init_count,1);
	assert_int_equal(test_hal_i2c_init.chid,I2C_SLAVE_HW_CHANNEL);
	assert_int_equal(test_hal_i2c_init.clock_speed,I2C_SPEED_400KHz);
	assert_int_equal(test_hal_i2c_init.address_mode,I2C_7BIT);
	assert_int_equal(test_hal_i2c_init.operation_mode,I2C_SLAVE);
	assert_int_equal(test_hal_i2c_init.owner_addr,(I2C_SLAVE_ADDRESS));
	assert_int_equal(hal_i2c_enable_irq_count,1);
	assert_int_equal(test_hal_i2c_enable_irq.chid,I2C_SLAVE_HW_CHANNEL);
	assert_int_equal(test_hal_i2c_enable_irq.clock_speed,I2C_SPEED_400KHz);
	assert_int_equal(test_hal_i2c_enable_irq.address_mode,I2C_7BIT);
	assert_int_equal(test_hal_i2c_enable_irq.operation_mode,I2C_SLAVE);
	assert_int_equal(test_hal_i2c_enable_irq.owner_addr,\
			(I2C_SLAVE_ADDRESS));
	assert_non_null (tx_buf->data);
	assert_non_null (tx_buf->head);
	assert_non_null (rx_hdl);
	assert_non_null (receiver_hdl);
	assert_non_null (transmitter_hdl);
}

void unit_i2c_slave_write(void **state)
{
	uint8_t buf[] = {0x01,0x02,0x03};
	uint16_t len = 3;
	tx_buf->len = 0;
	tx_buf->idx = 1;
	i2c_slave_write(&buf[0],len);
	assert_int_equal(tx_buf->data[0],0x01);
	assert_int_equal(tx_buf->data[1],0x02);
	assert_int_equal(tx_buf->data[2],0x03);
	assert_int_equal(tx_buf->idx,0);
	assert_int_equal(tx_buf->len,3);
}

void unit_i2c_slave_evt_handler(void **state)
{
	hal_i2c_status_t status;
	status = I2C_READ_REQUESTED;
	mock_transmitter_hdl_count = 0;
	mock_clbk_hdl_count = 0;
	mock_receiver_hdl_count = 0;
	tx_buf->idx = 1;
	hal_i2c_slave_tx_count = 0;
	i2c_slave_evt_handler(status);
	assert_int_equal(mock_transmitter_hdl_count,1);

	status = I2C_RX_RECEIVING;
	i2c_slave_evt_handler(status);
	assert_int_equal(mock_clbk_hdl_count,1);

	status = I2C_RX_COMPLETED;
	i2c_slave_evt_handler(status);
	assert_int_equal(mock_receiver_hdl_count,1);

	status = I2C_TX_TRASMITTING;
	i2c_slave_evt_handler(status);
	assert_int_equal(hal_i2c_slave_tx_count,1);
	assert_int_equal(tx_buf->idx,2);

	tx_buf->idx = I2C_SLAVE_BUFFER_SIZE;
	i2c_slave_evt_handler(status);
	assert_int_equal(tx_buf->idx,0);
}

int main(void)
{
	const struct CMUnitTest tests[] = {
			cmocka_unit_test(unit_i2c_slave_init),
			cmocka_unit_test(unit_i2c_slave_write),
			cmocka_unit_test(unit_i2c_slave_evt_handler),
	};
	cmocka_set_message_output(CM_OUTPUT_XML);
	cmocka_run_group_tests_name("test_i2c_slave",tests, NULL, NULL);
	return 0;
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
