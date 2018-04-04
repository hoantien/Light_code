/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    test_temp_sensor.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-8-2016
 * @brief   This file contains Test Temp Senspr source code
 *
 ******************************************************************************/
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>
#include <malloc.h>
#include <stdbool.h>

#include "temp_sensor.c"

extern int hal_i2c_master_tx_count;
extern int hal_i2c_master_tx_ret;
extern int hal_i2c_master_rx_count;
extern int hal_i2c_master_rx_ret;
extern hal_i2c_buffer_t hal_i2c_master_rx_buf;

void unit_temp_sensor_init (void **state)
{
	temp_sensor_channel_t chid;

	chid = 6;
	assert_int_equal(temp_sensor_init(chid),TEMP_INPUT_INVALID);

	chid = TEMP_SENS_CH1;
	hal_i2c_master_tx_count = 0;
	hal_i2c_master_tx_ret = I2C_OK;
	assert_int_equal(temp_sensor_init(chid),TEMP_OK);

	hal_i2c_master_tx_ret = I2C_TIMED_OUT;
	assert_int_equal(temp_sensor_init(chid),TEMP_TIMED_OUT);
	assert_int_equal(hal_i2c_master_tx_count,51);
}

void unit_temp_sensor_read (void **state)
{
	temp_sensor_channel_t chid;
	uint8_t data[] = {0x02, 0x01};

	chid = 6;
	assert_int_equal((int)temp_sensor_read(chid),0);

	chid = TEMP_SENS_CH1;
	hal_i2c_master_tx_count = 0;
	hal_i2c_master_rx_count = 0;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_rx_buf.bytes = &data[0];
	hal_i2c_master_rx_buf.length = 2;
	assert_int_equal((int)temp_sensor_read(chid),2);

	hal_i2c_master_rx_ret = I2C_TIMED_OUT;
	assert_int_equal(temp_sensor_read(chid),0);
	assert_int_equal(hal_i2c_master_tx_count,2);
	assert_int_equal(hal_i2c_master_rx_count,51);

	hal_i2c_master_tx_ret = I2C_TIMED_OUT;
	hal_i2c_master_tx_count = 0;
	assert_int_equal((int)temp_sensor_read(chid),0);
	assert_int_equal(hal_i2c_master_tx_count,50);


	data[0] = 0x80;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_rx_buf.bytes = &data[0];
	assert_int_equal((uint16_t)temp_sensor_read(chid),(3968|0xf000));
}

int main(void)
{
	const struct CMUnitTest tests[] = {
			cmocka_unit_test(unit_temp_sensor_init),
			cmocka_unit_test(unit_temp_sensor_read),
	};
	cmocka_set_message_output(CM_OUTPUT_XML);
	cmocka_run_group_tests_name("test_temp_sensor",tests, NULL, NULL);
	return 0;
}
