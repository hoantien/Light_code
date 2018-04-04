/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    sections.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jan-28-2016
 * @brief   This file contains definitions of Code/Data Region
 *
 ******************************************************************************/
#ifndef __SECTIONS__H__
#define __SECTIONS__H__

#if 0
/* ATCM Region */
#define ATCM_FUNC 		__attribute__ ((section(".atcm.text")))
#define ATCM_DATA 		__attribute__ ((section(".atcm.data")))
#define ATCM_CONST 		__attribute__ ((section(".atcm.rodata")))
#define ATCM_STATIC 	__attribute__ ((section(".atcm.bss")))

/* BTCM Region */
#define BTCM_FUNC		__attribute__ ((section(".btcm.text")))
#define BTCM_DATA		__attribute__ ((section(".btcm.data")))
#define BTCM_CONST		__attribute__ ((section(".btcm.rodata")))
#define BTCM_STATIC		__attribute__ ((section(".btcm.bss")))
#endif

/* DRAM Region */
#define DRAM_FUNC	 	__attribute__ ((section(".dram.text")))
#define DRAM_DATA	 	__attribute__ ((section(".dram.data")))
#define DRAM_CONST	 	__attribute__ ((section(".dram.rodata")))
#define DRAM_STATIC	 	__attribute__ ((section(".dram.bss")))

#define LIGHT_HEADER_FUNC
#define LIGHT_HEADER_DATA
#define LIGHT_HEADER_CONST
#define LIGHT_HEADER_STATIC

#endif /* __SECTIONS__H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
