/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    test_vcm.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    May-9-2016
 * @brief   This file contains Test vcm source code
 *
 ******************************************************************************/
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>
#include <malloc.h>
#include <string.h>
#include <stdbool.h>

#include "vcm.c"

extern hal_i2c_buffer_t *buf_test;
extern hal_i2c_buffer_t hal_i2c_master_rx_buf;
extern int hal_i2c_master_tx_ret;
extern int hal_i2c_master_rx_ret;
extern int hal_i2c_master_tx_count;
extern int hal_i2c_master_rx_count;
extern int hal_i2c_master_tx_for_unit_read_eeprom;
extern int hal_i2c_master_rx_for_unit_read_eeprom;
extern int hal_i2c_master_rx_for_unit_drv_vcm_init;
extern int vTaskDelay_count;

void unit_vcm_write_reg(void **state)
{
	uint8_t chid;
	uint8_t regaddr = 24;
	uint8_t regval[] = {5, 8, 6, 9, 16, 26};
	uint8_t len = sizeof(regval);
	buf_test = NULL;
	vcm_write_reg(chid,regaddr, regval,len);
	for (int i =0;i<(len +1);i++)
	{
		if (1>i)
			assert_int_equal(*(buf_test->bytes ), regaddr);
		else
			assert_int_equal(*(buf_test->bytes +i), regval[i-1]);
	}
	free(buf_test);
}

void unit_vcm_write_add8_data8(void **state)
{
	uint8_t chid;
	uint8_t regaddr = 13;
	uint8_t regval = 6;
	buf_test = NULL;
	vcm_write_add8_data8(chid,regaddr, regval);
	assert_int_equal(*(buf_test->bytes), regaddr);
	assert_int_equal(*(buf_test->bytes+1), regval);
	free(buf_test);
}
void unit_vcm_write_add8_data16(void **state)
{
	uint8_t chid;
	uint8_t regaddr = 13;
	uint16_t regval = 0x1564;
	buf_test = NULL;
	vcm_write_add8_data16(chid,regaddr, regval);
	assert_int_equal(*(buf_test->bytes), regaddr);
	assert_int_equal(*(buf_test->bytes+1), (uint8_t)((regval>>8)&0xFF));
	assert_int_equal(*(buf_test->bytes+2), (uint8_t)((regval)&0xFF));
	free(buf_test);
}

void unit_vcm_read_reg (void **state)
{
	uint8_t chid;
	uint8_t regaddr = 25;
	uint8_t *regval = malloc(sizeof(uint8_t));
	uint8_t data[] = {14};
	buf_test = NULL;
	hal_i2c_master_rx_buf.bytes = &data[0];
	vcm_read_reg(chid, regaddr, regval);
	assert_int_equal(*(buf_test->bytes), regaddr);
	assert_int_equal(*regval, data[0]);
}

void unit_read_eeprom (void **state)
{
	uint8_t chid;
	uint16_t addr = 23;
	uint8_t data[1];
	hal_i2c_master_rx_buf.bytes = data;


	hal_i2c_master_tx_ret = I2C_TIMED_OUT;
	assert_int_equal(read_eeprom(chid,addr), 0);

	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_TIMED_OUT;
	assert_int_equal(read_eeprom(chid,addr), 0);

	data[0] = 0;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	assert_int_equal(read_eeprom(chid,addr), 0);

	data[0] = 14;
	hal_i2c_master_tx_count = 0;
	hal_i2c_master_rx_count = 0;
	assert_int_equal(read_eeprom(chid,addr), 14);
	assert_int_equal(hal_i2c_master_tx_count, 2);
	assert_int_equal(hal_i2c_master_rx_count, 2);

	data[0] = 14;
	hal_i2c_master_tx_count = 0;
	hal_i2c_master_rx_count = 0;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 1;
	hal_i2c_master_rx_for_unit_read_eeprom = 0 ;
	assert_int_equal(read_eeprom(chid,addr), 0);
	assert_int_equal(hal_i2c_master_tx_count, 2);
	assert_int_equal(hal_i2c_master_rx_count, 1);

	hal_i2c_master_tx_count = 0;
	hal_i2c_master_rx_count = 0;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 1 ;
	assert_int_equal(read_eeprom(chid,addr), 0);
	assert_int_equal(hal_i2c_master_tx_count, 2);
	assert_int_equal(hal_i2c_master_rx_count, 2);
}

void unit_is_camera_valid(void **state)
{
	uint8_t chid = 0;
	uint8_t data[1];
	hal_i2c_master_rx_buf.bytes = data;

	hal_i2c_master_tx_ret = I2C_TIMED_OUT;
	assert_int_equal(is_camera_valid(chid),0);

	data[0] = 2;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0 ;
	hal_i2c_master_tx_count = 0;
	hal_i2c_master_rx_count = 0;
	assert_int_equal(is_camera_valid(chid),0);

	data[0] = 4;
	af_data[chid].efl = 0;
	assert_int_equal(is_camera_valid(chid),1);
	assert_int_equal(af_data[chid].efl,EFOCAL_LENGTH_28);

	data[0] = 1;
	af_data[chid].efl = 0;
	assert_int_equal(is_camera_valid(chid),1);
	assert_int_equal(af_data[chid].efl,EFOCAL_LENGTH_35);
}

void unit_validate_params (void **state)
{
	uint8_t chid = 6;
	uint8_t data[1];
	hal_i2c_master_rx_buf.bytes = data;

	assert_int_equal(validate_params(chid),VCM_INVALID_ARG);

	chid = 2;
	hal_i2c_master_tx_ret = I2C_TIMED_OUT;
	assert_int_equal(validate_params(chid),VCM_INVALID_ARG);

	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0 ;
	data[0] = 4;
	assert_int_equal(validate_params(chid),VCM_OK);
}

void unit_calculate_lens_position (void **state)
{
	float focal_length;
	float focus_dis;
	float lens_position = 0.0f;

	focal_length = 35;
	focus_dis = 35;
	assert_int_equal((calculate_lens_position(focal_length,focus_dis) \
			== lens_position),1);

	focal_length = 35;
	focus_dis = 100;
	lens_position = focal_length * focus_dis / (focus_dis - focal_length);
	assert_int_equal((calculate_lens_position(focal_length,focus_dis) \
			== lens_position),1);
}

void unit_setup_interp_table (void **state)
{
	interp_table_t *tbl;
	float_point_t table_test[2];

	tbl = NULL;
	assert_int_equal(setup_interp_table(tbl),0);

	tbl = malloc(sizeof(interp_table_t));
	tbl->table = table_test;

	table_test[0].x = 2;
	table_test[1].x = 2;
	assert_int_equal(setup_interp_table(tbl),0);

	table_test[0].x = 2;
	table_test[1].x = 1;
	table_test[0].y = 3;
	table_test[1].y = 4;
	assert_int_equal(setup_interp_table(tbl),1);
	assert_int_equal(table_test[0].x,1);
	assert_int_equal(table_test[1].x,2);
	assert_int_equal(table_test[0].y,4);
	assert_int_equal(table_test[1].y,3);
	free(tbl);
}

void unit_init_af_data (void **state)
{
	uint8_t chid = 0;
	uint8_t data[1];
	hal_i2c_master_rx_buf.bytes = data;

	af_data[chid].isvalid = 1;
	assert_int_equal(init_af_data(chid),1);

	af_data[chid].efl = 35;
	af_data[chid].isvalid = 0;
	af_data[chid].crop_factor = 0;
	af_data[chid].focal_length = 0;
	af_data[chid].calib_distance[0]=0;
	af_data[chid].calib_distance[1]=0;
	af_data[chid].calib_position[0]=0;
	af_data[chid].calib_position[1]=0;
	af_data[chid].dac_interp.datatype = 0;
	af_data[chid].dac_interp.size = 0;
	af_data[chid].dac_interp.table = NULL;
	data[0] = 14;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	assert_int_equal(init_af_data(chid),0);
	assert_int_equal(af_data[chid].crop_factor,AF_CROP_FACTOR);
	assert_int_equal(af_data[chid].focal_length,\
			(af_data[chid].efl / af_data[chid].crop_factor));
	assert_int_equal(af_data[chid].calib_distance[0],14);
	assert_int_equal(af_data[chid].calib_distance[1],14);
	assert_int_equal(af_data[chid].calib_position[0],14);
	assert_int_equal(af_data[chid].calib_position[1],14);
	assert_int_equal(af_data[chid].dac_interp.datatype,INTERP_TYPE_FLOAT);
	assert_int_equal(af_data[chid].dac_interp.size,NUM_CALIB_POINTS);
	assert_non_null(af_data[chid].dac_interp.table);
	assert_int_equal(af_data[chid].tbl_data[0].x,(af_data[chid].focal_length*\
			af_data[chid].calib_distance[0]/(af_data[chid].calib_distance[0]-\
					af_data[chid].focal_length)));
	assert_int_equal(af_data[chid].tbl_data[1].x,(af_data[chid].focal_length*\
			af_data[chid].calib_distance[1]/(af_data[chid].calib_distance[1]-\
					af_data[chid].focal_length)));
	assert_int_equal(af_data[chid].tbl_data[0].y,data[0]);
	assert_int_equal(af_data[chid].tbl_data[1].y,data[0]);
}

void unit_drv_vcm_init(void **state)
{
	uint8_t chid;
	uint8_t data[1];
	hal_i2c_master_rx_buf.bytes = data;

	chid = 6;
	assert_int_equal(drv_vcm_init(chid),VCM_INVALID_ARG);

	chid = 2;
	data[0] = 4;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0;
	af_data[chid].is_on = 1;
	assert_int_equal(drv_vcm_init(chid),VCM_OK);

	data[0] = 4;
	chid = 2;
	af_data[chid].is_on = 0;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_buf.bytes = &data[0];
	assert_int_equal(drv_vcm_init(chid),VCM_INVALID_CVER);

	data[0] = 4;
	hal_i2c_master_rx_count=0;
	hal_i2c_master_rx_for_unit_drv_vcm_init =1;
	assert_int_equal(drv_vcm_init(chid),VCM_OP_TIMEOUT);

	data[0] = 4;
	hal_i2c_master_rx_count=0;
	hal_i2c_master_rx_for_unit_drv_vcm_init =3;
	assert_int_equal(drv_vcm_init(chid),VCM_ERROR);

	af_data[chid].isvalid = 1;
	data[0] = 4;
	hal_i2c_master_rx_count=0;
	hal_i2c_master_rx_for_unit_drv_vcm_init =3;
	assert_int_equal(drv_vcm_init(chid),VCM_OK);
}

void unit_drv_vcm_move_to_hall (void **state)
{
	uint8_t chid = 6;
	int16_t hall;
	uint8_t data[1];
	hal_i2c_master_rx_buf.bytes = data;

	assert_int_equal(drv_vcm_move_to_hall(chid, hall), VCM_INVALID_ARG);

	chid = 2;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0;
	data[0] = 4;
	af_data[chid].isvalid = 0;
	assert_int_equal(drv_vcm_move_to_hall(chid, hall), VCM_ERROR);

	chid = 2;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0;
	data[0] = 4;
	af_data[chid].isvalid = 1;
	af_data[chid].is_on = 1;
	assert_int_equal(drv_vcm_move_to_hall(chid, hall), VCM_OP_TIMEOUT);

	chid = 2;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_rx_for_unit_drv_vcm_init = 0;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0;
	data[0] = 1;
	af_data[chid].isvalid = 1;
	af_data[chid].is_on = 1;
	assert_int_equal(drv_vcm_move_to_hall(chid, hall), VCM_OK);
	/*
	 * TODO: Check to know how many condition can be stay on if condition
	 * Line 378:
	 */
}

void unit_drv_vcm_calculate_daccode(void **state)
{
	uint8_t chid;
	uint32_t distance;
	int16_t *dac = NULL;
	uint8_t data[1];
	float_point_t point[2];
	float dac_float = 0.0f;
	point[0].y = 113;
	point[1].y = 213;
	hal_i2c_master_rx_buf.bytes = data;

	chid = 6;
	assert_int_equal(drv_vcm_calculate_daccode(chid, distance, dac),\
			VCM_INVALID_ARG);

	chid = 2;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0;
	data[0] = 4;
	af_data[chid].isvalid = 0;
	assert_int_equal(drv_vcm_calculate_daccode(chid, distance, dac),\
			VCM_ERROR);

	chid = 2;
	af_data[chid].isvalid = 1;
	af_data[chid].is_on = 1;
	af_data[chid].dac_interp.datatype=0;
	assert_int_equal(drv_vcm_calculate_daccode(chid, distance, dac),\
			VCM_ERROR);

	chid = 2;
	point[0].y = 113;
	data[0] = 4;
	af_data[chid].isvalid = 1;
	af_data[chid].is_on = 1;
	af_data[chid].dac_interp.datatype=1;
	point[0].x = 2;
	point[1].x = 2;
	af_data[chid].dac_interp.table = point;
	assert_int_equal(drv_vcm_calculate_daccode(chid, distance, dac),\
			VCM_ERROR);

	af_data[chid].focal_length = 35;
	distance = 100;
	point[0].x = 100;
	point[1].x = 2;
	assert_int_equal(drv_vcm_calculate_daccode(chid, distance, dac),113);

	point[0].x = 13;
	point[1].x = 2;
	assert_int_equal(drv_vcm_calculate_daccode(chid, distance, dac),213);

	point[1].x = 70;
	assert_int_equal(drv_vcm_calculate_daccode(chid, distance, dac),VCM_OK);

	dac = malloc(sizeof(int16_t));
	dac_float = point[0].y + ((point[1].y - point[0].y)\
			* ((35 * distance / (distance - 35)) - point[0].x)\
			/ (point[1].x - point[0].x));
	assert_int_equal(drv_vcm_calculate_daccode(chid, distance, dac),VCM_OK);
	assert_int_equal(*dac,(int16_t)(dac_float+1));
}

void unit_drv_vcm_get_hall(void **state)
{
	uint8_t chid;
	int16_t *hall = NULL;
	uint8_t data[1];
	hal_i2c_master_rx_buf.bytes = data;

	chid = 6;
	assert_int_equal(drv_vcm_get_hall(chid, hall),VCM_INVALID_ARG);

	chid = 2;
	data[0] = 4;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0 ;
	assert_int_equal(drv_vcm_get_hall(chid, hall),VCM_INVALID_ARG);

	hall =malloc(sizeof(int16_t));
	assert_int_equal(drv_vcm_get_hall(chid, hall),VCM_OK);
	free (hall);
}

void unit_drv_vcm_deinit (void **state)
{
	uint8_t chid;
	uint8_t data[1];
	hal_i2c_master_rx_buf.bytes = data;

	chid = 6;
	assert_int_equal(drv_vcm_deinit(chid),VCM_INVALID_ARG);

	chid = 2;
	data[0] = 4;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0 ;
	vTaskDelay_count = 0;
	assert_int_equal(drv_vcm_deinit(chid),VCM_OK);
	assert_int_equal(vTaskDelay_count,30);
}

void unit_drv_vcm_focus(void **state)
{
	uint8_t chid;
	uint32_t distance = 100;
	uint8_t data[1];
	float_point_t point[2];
	float dac_float = 0.0f;
	point[0].y = 113;
	point[1].y = 213;
	hal_i2c_master_rx_buf.bytes = data;

	chid = 6;
	assert_int_equal(drv_vcm_focus(chid, distance),VCM_INVALID_ARG);

	chid = 2;
	hal_i2c_master_tx_ret = I2C_OK;
	hal_i2c_master_rx_ret = I2C_OK;
	hal_i2c_master_tx_for_unit_read_eeprom = 0;
	hal_i2c_master_rx_for_unit_read_eeprom = 0;
	data[0] = 4;
	af_data[chid].isvalid = 0;
	assert_int_equal(drv_vcm_focus(chid, distance),VCM_ERROR);

	af_data[chid].dac_interp.table = point;
	point[1].x = 2;
	point[0].x = 2;
	af_data[chid].isvalid = 1;
	af_data[chid].is_on = 1;
	assert_int_equal(drv_vcm_focus(chid, distance),VCM_ERROR);

	af_data[chid].dac_interp.table = point;
	point[1].x = 70;
	point[0].x = 13;
	af_data[chid].isvalid = 1;
	af_data[chid].is_on = 1;
	assert_int_equal(drv_vcm_focus(chid, distance),VCM_OK);
}

int main(void)
{
	const struct CMUnitTest tests[] = {
			cmocka_unit_test(unit_vcm_write_reg),
			cmocka_unit_test(unit_vcm_write_add8_data8),
			cmocka_unit_test(unit_vcm_write_add8_data16),
			cmocka_unit_test(unit_vcm_read_reg),
			cmocka_unit_test(unit_read_eeprom),
			cmocka_unit_test(unit_is_camera_valid),
			cmocka_unit_test(unit_validate_params),
			cmocka_unit_test(unit_calculate_lens_position),
			cmocka_unit_test(unit_setup_interp_table),
			cmocka_unit_test(unit_init_af_data),
			cmocka_unit_test(unit_drv_vcm_init),
			cmocka_unit_test(unit_drv_vcm_move_to_hall),
			cmocka_unit_test(unit_drv_vcm_calculate_daccode),
			cmocka_unit_test(unit_drv_vcm_get_hall),
			cmocka_unit_test(unit_drv_vcm_deinit),
			cmocka_unit_test(unit_drv_vcm_focus),
	};
	cmocka_set_message_output(CM_OUTPUT_XML);
	cmocka_run_group_tests_name("test_vcm",tests, NULL, NULL);
	return 0;
}
