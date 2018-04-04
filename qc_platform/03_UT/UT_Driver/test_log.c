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
#include <malloc.h>
#include <stdbool.h>

#include "log.c"

extern int hal_com_sendbyte_count;
extern int hal_com_readbyte_count;
extern int hal_com_init_count;

void unit_write (void **state)
{
	int file, len;
	char *ptr = "test write";
	len = 10;
	hal_com_sendbyte_count = 0;
	_write(file,ptr,len);
	assert_int_equal((*(com_dbg.data) == 'e'),1);
	assert_int_equal(hal_com_sendbyte_count,10);
}

void unit_read (void **state)
{
	int file, len;
	uint8_t *ptr = malloc(3*sizeof(uint8_t));
	len =1;
	hal_com_readbyte_count = 0;
	*(com_dbg.data) = 'c';
	_read(file, ptr, len);
	assert_int_equal((*(ptr--) == 'c'),1);
	assert_int_equal(hal_com_readbyte_count,1);
}

void unit_log_init (void **state)
{
	hal_com_init_count = 0;
	log_init();
	assert_int_equal(hal_com_init_count,1);
}

int main(void)
{
	const struct CMUnitTest tests[] = {
			cmocka_unit_test(unit_write),
			cmocka_unit_test(unit_read),
			cmocka_unit_test(unit_log_init),
	};
	cmocka_set_message_output(CM_OUTPUT_XML);
	cmocka_run_group_tests_name("test_log",tests, NULL, NULL);
	return 0;
}
