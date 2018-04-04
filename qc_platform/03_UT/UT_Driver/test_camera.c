/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    test_camera.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-8-2016
 * @brief   This file contains Test camera source code
 *
 ******************************************************************************/
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>
#include <malloc.h>
#include <string.h>
#include <stdbool.h>

#define CAMERA_TBL {35,70,150}

#include "camera.c"

extern const char *string_test;
extern uint8_t *i2c_ctrl_push_msg_data;
extern uint8_t i2c_ctrl_push_msg_cam_status;
extern uint8_t i2c_ctrl_push_msg_count;

void unit_cam_init (void **state)
{
	cam_module_t cam =
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};

	cam_init(&cam);
	assert_non_null (cam.cmd);
	assert_non_null (cam.cmd->data);
	assert_int_equal(cam.cmd->status,CAM_CMD_IDLE);
	assert_int_equal (cam.eeprom.offset,(uint8_t)CAM_EEPROM_UUID_OFFSET);
	assert_int_equal (cam.eeprom.len,(uint16_t)CAM_EEPROM_UUID_LEN);
	assert_non_null (cam.uuid);
	assert_non_null (cam.eeprom.buf);
	assert_int_equal (cam.eeprom.flag,0);
}

void unit_cam_open(void **state)
{
	volatile cam_module_t cam =
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam.cmd = malloc(sizeof(cam_command_t));

	cam_open(&cam);
	assert_int_equal (cam.m_o_status,CAM_MODULE_CLOSE);

	i2c_ctrl_push_msg_data = NULL;
	cam.cmd->status = CAM_CMD_WRITE_DONE;
	cam_open(&cam);

	free(i2c_ctrl_push_msg_data);
	free(cam.cmd);
}

void unit_cam_set_reg_continuous(void **state)
{
	cam_module_t cam =
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	uint16_t reg_arr_size = 2;
	msm_camera_i2c_reg_array_t reg_arr;
	cam_data_mode_t reg_size;

	reg_size = 3;
	string_test = NULL;
	char *ptr = "Data size is not supported";
	cam_set_reg_continuous(&cam, reg_arr_size, &reg_arr, reg_size);
	assert_string_equal(string_test,ptr);

	ptr = "%s Wrote the register was timeout!";
	cam.cmd = malloc(sizeof(cam_command_t));
	cam.cmd->status = 0;
	reg_size = DATA_8BIT;
	reg_arr.reg_addr = 14;
	reg_arr.reg_val = 0x23;
	i2c_ctrl_push_msg_data = NULL;
	assert_int_equal (cam_set_reg_continuous(&cam, reg_arr_size, \
			&reg_arr, reg_size),0);
	assert_string_equal(string_test,ptr);

	cam.cmd->status = CAM_CMD_WRITE_DONE;
	assert_int_equal (cam_set_reg_continuous(&cam, reg_arr_size, \
			&reg_arr, reg_size),1);
	assert_int_equal(*i2c_ctrl_push_msg_data,reg_arr.reg_addr);
	assert_int_equal(*(i2c_ctrl_push_msg_data+2),reg_arr.reg_val);

	reg_size = DATA_16BIT;
	reg_arr.reg_addr = 32;
	reg_arr.reg_val = 0x2523;
	i2c_ctrl_push_msg_data = NULL;
	cam_set_reg_continuous(&cam, reg_arr_size, &reg_arr, reg_size);
	assert_int_equal (*i2c_ctrl_push_msg_data,32);
	assert_int_equal (*(i2c_ctrl_push_msg_data+2),0x23);
	assert_int_equal (*(i2c_ctrl_push_msg_data+3),0x25);
	free(i2c_ctrl_push_msg_data);
	free(cam.cmd);
}

void unit_cam_set_reg_noncontinuous(void **state)
{
	cam_module_t cam =
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam.cmd = malloc(sizeof(cam_command_t));
	uint16_t reg_arr_size = 1;
	msm_camera_i2c_reg_array_t reg_arr	;
	cam_data_mode_t reg_size;

	reg_size = 3;
	string_test = NULL;
	char *ptr = "Data size is not supported";
	cam_set_reg_noncontinuous(&cam, reg_arr_size, &reg_arr, reg_size);
	assert_string_equal(string_test,ptr);

	ptr = "%s Wrote the register was timeout!";
	reg_size = DATA_8BIT;
	reg_arr.reg_addr = 14;
	reg_arr.reg_val = 0x23;
	i2c_ctrl_push_msg_data = NULL;
	assert_int_equal(cam_set_reg_noncontinuous(\
			&cam, reg_arr_size, &reg_arr, reg_size),0);
	assert_int_equal (*(i2c_ctrl_push_msg_data+1),14);
	assert_int_equal (*(i2c_ctrl_push_msg_data+2),0x23);
	assert_string_equal(string_test,ptr);
	free(i2c_ctrl_push_msg_data);

	i2c_ctrl_push_msg_cam_status =1;
	i2c_ctrl_push_msg_data = NULL;
	assert_int_equal(cam_set_reg_noncontinuous(\
			&cam, reg_arr_size, &reg_arr, reg_size),1);
	assert_int_equal (*(i2c_ctrl_push_msg_data+1),14);
	assert_int_equal (*(i2c_ctrl_push_msg_data+2),0x23);
	free(i2c_ctrl_push_msg_data);

	reg_size = DATA_16BIT;
	reg_arr.reg_addr = 32;
	reg_arr.reg_val = 0x2523;
	i2c_ctrl_push_msg_data = NULL;
	assert_int_equal(cam_set_reg_noncontinuous(\
			&cam, reg_arr_size, &reg_arr, reg_size),1);
	assert_int_equal (*(i2c_ctrl_push_msg_data+1),32);
	assert_int_equal (*(i2c_ctrl_push_msg_data+2),0x25);
	assert_int_equal (*(i2c_ctrl_push_msg_data+3),0x23);
	free(i2c_ctrl_push_msg_data);

	free(cam.cmd);
}

void unit_cam_is_fps_supported (void **state)
{
	uint16_t fps;

	fps = 0;
	assert_int_equal (cam_is_fps_supported(fps),0);

	fps = 30;
	assert_int_equal (cam_is_fps_supported(fps),1);
}

void unit_cam_update_resolution(void **state)
{
	cam_module_t cam =
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam_x_resolution_type_t x;
	cam_y_resolution_type_t y;
	uint16_t ucid=3;

	x = 1;	y = 2;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),0);

	x = X_RES_3M;	y = Y_RES_3M;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),1);
	assert_int_equal(cam.ucid[ucid].res.x,X_RES_3M);
	assert_int_equal(cam.ucid[ucid].res.y,Y_RES_3M);
	assert_int_equal(cam.ucid[ucid].res.res_type,RES_3M);
	assert_int_equal(cam.ucid[ucid].host_updated ,CAM_UPDATED_RESOLUTION);

	y = Y_RES_13M;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),0);

	x = X_RES_13M;	y = Y_RES_13M;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),1);
	assert_int_equal(cam.ucid[ucid].res.x,X_RES_13M);
	assert_int_equal(cam.ucid[ucid].res.y,Y_RES_13M);
	assert_int_equal(cam.ucid[ucid].res.res_type,RES_13M);
	assert_int_equal(cam.ucid[ucid].host_updated ,CAM_UPDATED_RESOLUTION);

	y = Y_RES_3M;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),0);

	x = X_RES_720P;	y = Y_RES_720P;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),1);
	assert_int_equal(cam.ucid[ucid].res.x,X_RES_720P);
	assert_int_equal(cam.ucid[ucid].res.y,Y_RES_720P);
	assert_int_equal(cam.ucid[ucid].res.res_type,RES_720P);
	assert_int_equal(cam.ucid[ucid].host_updated ,CAM_UPDATED_RESOLUTION);

	y = Y_RES_3M;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),0);

	x = X_RES_1080P;	y = Y_RES_1080P;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),1);
	assert_int_equal(cam.ucid[ucid].res.x,X_RES_1080P);
	assert_int_equal(cam.ucid[ucid].res.y,Y_RES_1080P);
	assert_int_equal(cam.ucid[ucid].res.res_type,RES_1080P);
	assert_int_equal(cam.ucid[ucid].host_updated ,CAM_UPDATED_RESOLUTION);

	y = Y_RES_3M;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),0);

	x = X_RES_4K_UHD;	y = Y_RES_4K_UHD_CINEMA;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),1);
	assert_int_equal(cam.ucid[ucid].res.x,X_RES_4K_UHD);
	assert_int_equal(cam.ucid[ucid].res.y,Y_RES_4K_UHD_CINEMA);
	assert_int_equal(cam.ucid[ucid].res.res_type,RES_4K_UHD);
	assert_int_equal(cam.ucid[ucid].host_updated ,CAM_UPDATED_RESOLUTION);

	y = Y_RES_3M;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),0);

	x = X_RES_4K_CINEMA;	y = Y_RES_4K_UHD_CINEMA;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),1);
	assert_int_equal(cam.ucid[ucid].res.x,X_RES_4K_CINEMA);
	assert_int_equal(cam.ucid[ucid].res.y,Y_RES_4K_UHD_CINEMA);
	assert_int_equal(cam.ucid[ucid].res.res_type,RES_4K_CINEMA);
	assert_int_equal(cam.ucid[ucid].host_updated ,CAM_UPDATED_RESOLUTION);

	y = Y_RES_3M;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),0);

	x = X_RES_4K_CINEMA;	y = Y_RES_4K_UHD_CINEMA;
	assert_int_equal(cam_update_resolution(x, y, &cam, ucid),1);
}

void unit_cam_write_reg (void **state)
{
	volatile cam_module_t cam=
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam.cmd = malloc(sizeof(cam_command_t));
	uint16_t reg;
	uint16_t data;
	uint8_t d_mode;

	d_mode = 3;
	string_test = NULL;
	char *ptr = "Data size is invalid";
	assert_int_equal(cam_write_reg(&cam, reg, data, d_mode),0);
	assert_string_equal(string_test,ptr);

	d_mode = DATA_8BIT;
	reg = 123;
	data = 0x13;
	i2c_ctrl_push_msg_data = NULL;
	i2c_ctrl_push_msg_cam_status = 0;
	ptr = "%s Wrote reg was timeout!";
	assert_false(cam_write_reg(&cam, reg, data, d_mode));
	assert_int_equal (*(i2c_ctrl_push_msg_data+1),123);
	assert_int_equal (*(i2c_ctrl_push_msg_data+2),0x13);
	assert_string_equal(string_test,ptr);
	free(i2c_ctrl_push_msg_data);

	i2c_ctrl_push_msg_data = NULL;
	i2c_ctrl_push_msg_cam_status = 1;
	assert_true(cam_write_reg(&cam, reg, data, d_mode));
	free(i2c_ctrl_push_msg_data);


	d_mode = DATA_16BIT;
	reg= 32;
	data = 0x2523;
	i2c_ctrl_push_msg_data = NULL;
	i2c_ctrl_push_msg_cam_status = 1;
	/*TODO: Check core dump at line 195*/
	assert_true(0);
	assert_true (cam_write_reg(&cam, reg, data, d_mode));
	assert_int_equal (*(i2c_ctrl_push_msg_data+1),32);
	assert_int_equal (*(i2c_ctrl_push_msg_data+2),0x25);
	assert_int_equal (*(i2c_ctrl_push_msg_data+3),0x23);
	free(i2c_ctrl_push_msg_data);
}

void unit_cam_read_reg (void **state)
{
	volatile cam_module_t cam =
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam.cmd = malloc(sizeof(cam_command_t));
	uint16_t reg;
	uint8_t d_mode;

	d_mode = 3;
	string_test = NULL;
	char *ptr = "Data size is invalid";
	cam_read_reg(&cam, reg, d_mode);
	assert_string_equal(string_test,ptr);

	i2c_ctrl_push_msg_cam_status = 0;
	d_mode = DATA_16BIT;
	ptr = "%s Wrote the register was timeout!";
	string_test = NULL;
	reg = 123;
	assert_int_equal (cam_read_reg(&cam, reg, d_mode),FALSE);
	assert_string_equal(string_test,ptr);

	i2c_ctrl_push_msg_cam_status = 1;
	i2c_ctrl_push_msg_count = 0;
	i2c_ctrl_push_msg_data = NULL;
	d_mode = DATA_16BIT;
	reg = 123;
	assert_int_equal (cam_read_reg(&cam, reg, d_mode),FALSE);
	assert_int_equal(*(i2c_ctrl_push_msg_data+1), 123);

	i2c_ctrl_push_msg_cam_status = 3;
	i2c_ctrl_push_msg_count = 0;
	i2c_ctrl_push_msg_data = NULL;
	d_mode = DATA_16BIT;
	reg = 123;
	assert_int_equal (cam_read_reg(&cam, reg, d_mode),123);
	assert_int_equal(*(i2c_ctrl_push_msg_data+1), 123);

	i2c_ctrl_push_msg_cam_status = 1;
	i2c_ctrl_push_msg_count = 0;
	i2c_ctrl_push_msg_data = NULL;
	d_mode = DATA_8BIT;
	reg = 0x1423;
	assert_int_equal (cam_read_reg(&cam, reg, d_mode),FALSE);
	assert_int_equal(*(i2c_ctrl_push_msg_data), 0x14);

	i2c_ctrl_push_msg_cam_status = 3;
	i2c_ctrl_push_msg_count = 0;
	i2c_ctrl_push_msg_data = NULL;
	d_mode = DATA_8BIT;
	reg = 0x2354;
	assert_int_equal (cam_read_reg(&cam, reg, d_mode),0x23);
	assert_int_equal(*(i2c_ctrl_push_msg_data), 0x23);

	free(cam.cmd);
}

void unit_cam_ctrl_reg_group (void **state)
{
	volatile cam_module_t cam=
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam.cmd = malloc(sizeof(cam_command_t));
	uint8_t state_t = ON;
	i2c_ctrl_push_msg_data = NULL;
	i2c_ctrl_push_msg_cam_status = 1;
	cam_ctrl_reg_group(&cam, state_t);
	assert_int_equal (*i2c_ctrl_push_msg_data,1);
	free(i2c_ctrl_push_msg_data);
	free(cam.cmd);
}

void unit_cam_eeprom_write(void **state)
{
	volatile cam_module_t cam=
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam.cmd = malloc(sizeof(cam_command_t));
	uint8_t data[] = {1,2};

	cam.eeprom.len_counter = 1;
	cam.eeprom.len = 2;
	cam.eeprom.flag = CAM_EEPROM_MODULE_WRITE_FLAG;
	cam.eeprom.offset = 0;
	cam.eeprom.buf = data;
	cam_eeprom_write(&cam);
	assert_int_equal (cam.eeprom.flag,CAM_EEPROM_MODULE_WRITE_FLAG);
	assert_int_equal (cam.eeprom.len_counter,2);
	assert_int_equal (cam.eeprom.offset,1);

	cam.eeprom.len_counter = 1;
	cam.eeprom.len = 2;
	cam.eeprom.flag = (CAM_EEPROM_R_W_BUSY_FLAG|CAM_EEPROM_R_W_ERROR_FLAG|\
			CAM_EEPROM_MODULE_WRITE_FLAG);
	cam.eeprom.offset = 0;
	cam.eeprom.buf = data;
	string_test = NULL;
	char *ptr = "%s: WRITE EEPROM ERROR %s";
	cam_eeprom_write(&cam);
	assert_string_equal(string_test,ptr);
	assert_int_equal (cam.eeprom.flag,CAM_EEPROM_MODULE_WRITE_FLAG);
	assert_int_equal (cam.eeprom.len_counter,2);
	assert_int_equal (cam.eeprom.offset,1);

	cam.eeprom.len_counter = 2;
	cam.eeprom.len = 2;
	cam.eeprom.flag = CAM_EEPROM_MODULE_WRITE_FLAG;
	cam.eeprom.offset = 0;
	cam.eeprom.buf = data;
	cam_eeprom_write(&cam);
	assert_int_equal (cam.eeprom.flag,0);
	assert_int_equal (cam.eeprom.len_counter,2);
	assert_int_equal (cam.eeprom.offset,0);

	free(cam.cmd);
}

void unit_cam_eeprom_read(void **state)
{
	uint8_t data[5] ;

	string_test = NULL;
	char *ptr = "%s: NULL pointer ";
	cam_eeprom_read(NULL);
	assert_string_equal(string_test,ptr);

	volatile cam_module_t cam1=
	{
		.chid = 17,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	string_test = NULL;
	ptr = "%s: Invalid camera channel";
	cam_eeprom_read(&cam1);
	assert_string_equal(string_test,ptr);

	volatile cam_module_t cam2=
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam2.cmd = malloc(sizeof(cam_command_t));
	cam2.eeprom.len_counter = 0;
	cam2.eeprom.len = 2;
	cam2.eeprom.offset=0;
	cam2.eeprom.buf = data;
	cam2.eeprom.tmp = 17;
	i2c_ctrl_push_msg_count = 0;
	cam2.eeprom.flag = (CAM_EEPROM_MODULE_READ_FLAG|CAM_EEPROM_R_DONE_FLAG);
	string_test = NULL;
	cam_eeprom_read(&cam2);
	assert_int_equal (cam2.eeprom.flag,CAM_EEPROM_MODULE_READ_FLAG);
	assert_int_equal (cam2.eeprom.offset,1);
	assert_int_equal (cam2.eeprom.len_counter,1);
	assert_int_equal (*(cam2.eeprom.buf),17);

	cam2.eeprom.flag = (CAM_EEPROM_MODULE_READ_FLAG|CAM_EEPROM_R_W_ERROR_FLAG);
	string_test = NULL;
	ptr = "%s: READ EEPROM ERROR %s";
	cam_eeprom_read(&cam2);
	assert_string_equal(string_test,ptr);
	assert_int_equal (cam2.eeprom.flag,CAM_EEPROM_MODULE_READ_FLAG);
	assert_int_equal (cam2.eeprom.offset,2);
	assert_int_equal (cam2.eeprom.len_counter,2);
	assert_int_equal (*(cam2.eeprom.buf+1),17);

	cam2.eeprom.len_counter = 2;
	cam2.eeprom.len = 2;
	cam2.eeprom.len = 2;
	string_test = NULL;
	ptr = "Read %d of %s success!";
	cam_eeprom_read(&cam2);
	assert_string_equal(string_test,ptr);

	free(cam2.cmd);
}

void unit_cam_eeprom_callback (void **state)
{
	hal_i2c_status_t status;
	cam_module_t cam=
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam.cmd = malloc(sizeof(cam_command_t));

	status = I2C_TX_COMPLETED;
	cam.eeprom.flag = 0;
	cam_eeprom_callback(status, (void *)&cam);
	assert_int_equal(cam.eeprom.flag, CAM_EEPROM_W_DONE_FLAG);

	status = I2C_RX_COMPLETED;
	cam.eeprom.flag = 0;
	cam_eeprom_callback(status, (void *)&cam);
	assert_int_equal(cam.eeprom.flag, CAM_EEPROM_R_DONE_FLAG);


	status = I2C_IDLE;
	cam.eeprom.flag = 0;
	cam_eeprom_callback(status, (void *)&cam);
	assert_int_equal(cam.eeprom.flag, CAM_EEPROM_R_W_ERROR_FLAG);

	free(cam.cmd);
}

void unit_cam_command_callback (void **state)
{
	hal_i2c_status_t status;
	cam_module_t cam=
	{
		.chid = CAM_CH_A1,
		.name = NULL,
		.type = CAM_TYPE_35MM,
		.slave_addr =0,
		.cmd = NULL,
	};
	cam.cmd = malloc(sizeof(cam_command_t));

	status = I2C_RX_COMPLETED;
	cam_command_callback(status, (void *)&cam);
	assert_int_equal(cam.cmd->status, CAM_CMD_READ_DONE);

	status = I2C_TX_COMPLETED;
	cam_command_callback(status, (void *)&cam);
	assert_int_equal(cam.cmd->status, CAM_CMD_WRITE_DONE);


	status = I2C_IDLE;
	string_test = NULL;
	char *ptr = "%s:%d: Unknown error status %04X";
	cam_command_callback(status, (void *)&cam);
	assert_int_equal(cam.cmd->status, CAM_CMD_IDLE);
	assert_string_equal(string_test,ptr);

	free(cam.cmd);
}

int main(void)
{
	const struct CMUnitTest tests[] = {
			cmocka_unit_test(unit_cam_init),
			cmocka_unit_test(unit_cam_open),
			cmocka_unit_test(unit_cam_set_reg_continuous),
			cmocka_unit_test(unit_cam_set_reg_noncontinuous),
			cmocka_unit_test(unit_cam_is_fps_supported),
			cmocka_unit_test(unit_cam_update_resolution),
			cmocka_unit_test(unit_cam_write_reg),
			cmocka_unit_test(unit_cam_read_reg),
			cmocka_unit_test(unit_cam_ctrl_reg_group),
			cmocka_unit_test(unit_cam_eeprom_write),
			cmocka_unit_test(unit_cam_eeprom_read),
			cmocka_unit_test(unit_cam_eeprom_callback),
			cmocka_unit_test(unit_cam_command_callback),
	};
	cmocka_set_message_output(CM_OUTPUT_XML);
	cmocka_run_group_tests_name("test_camera",tests, NULL, NULL);
	return 0;
}
