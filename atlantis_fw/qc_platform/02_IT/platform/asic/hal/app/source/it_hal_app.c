/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_app.c
 * @author  The LightCo
 * @version V1.0.2
 * @date    19-Apr, 2016
 * @brief   This file contains expand of hal application
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	20-Feb-2016	Initial revision:
 *                      - Infrastructure.
 * * 1.0.1	19-Apr-2016 Test HAL baseline 1.0.0
 * * 1.0.2  29-Apr-2016 Update instruction() according to change of development
 *                     code structure of PWM
 * * 1.0.3  29-Apr-2016 Update test cases gpio,syncio,com and update
 *                                       instruction
 */
/******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "it_hal_app.h"
#include "it_log_swapper.h"
#ifdef REG_DUMP_ENABLE
regs_fail_info_t regs_dump[DUMP_BUFFER_MAX];
regs_fail_info_t* preg_dump = regs_dump;
#endif
/* Exported define -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NUM_PARAMS_MAX			    32
#define PARAM_LENGTH_MAX            32
/* Private typedef -----------------------------------------------------------*/

/* Static variables ----------------------------------------------------------*/
LOCAL it_map_t it_hal_comps[] = {
		{"1",  it_hal_com_handler   },
		{"2",  it_hal_i2c_handler   },
		{"3",  it_hal_pwm_handler   },
//		{"4",  it_hal_scu_handler   },
		{"5",  it_hal_syncio_handler},
		{"6",  it_hal_spi_handler   },
		{"7",  it_hal_wdt_handler   },
		{"8",  it_hal_timer_handler },
		{"9",  it_hal_gpio_handler  },
//		{"10", it_hal_qspi_handler  },
		{"4", it_hal_mipi_handler  },
		{"12", it_hal_ddr_handler	},
		{"" , NULL}
};
/**
 * Arguments list
 */

/* Static functions ----------------------------------------------------------*/
/**
 * @brief Print help catalog
 * @details Print help catalog to console
 * @param[in] NA
 * @param[out] NA
 * @return NA
 */
LOCAL void help(void);

/**
 * @brief Print instruction of specified tested module 
 * @details Print instruction of specified tested module
 * @param[in] argv	: module ID as string
 * @param[out] NA
 * @return NA
 */
LOCAL void instruction(char* argv);
/* Exported global variables -------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int main(void)
{
	char*              argv[NUM_PARAMS_MAX] = {NULL};
	char*              params_str	 		= NULL;
	int                size    = ZERO;
	int                index   = -1;
	int                ercd    = -1;
	char*              repeat  = NULL;
	#if (ASIC_NUM == ASIC1)
		log_printf("This is ASIC 1 test environment\r\n");
	#elif (ASIC_NUM == ASIC2)
		log_printf("This is ASIC 2 test environment\r\n");
	#elif (ASIC_NUM == ASIC3)
		log_printf("This is ASIC 3 test environment\r\n");
	#endif
	while (1)
	{
		log_printf("[SYS-LOG]This is HAL test for Asic FW stage 2\r\n");
		/**! Introduction */
		help();
		/**********************************************************************/
		/**! Get specific module or whole stage's modules */
		argv[0] = console_getstring(&size);

		if ((NULL == argv[0]))
		{
			log_printf("Invalid module\r\n");
			if (argv[0])
			{
				free(argv[0]);
				argv[0] = NULL;
			}
			log_printf("Scenario Done\r\n");
			continue;
		}
		if (0 == strcmp(argv[0], "0"))
		{
			free(argv[0]);
			argv[0] = NULL;
			log_printf("Exit\r\n");
			break;
		}
		/**! Reset index */
		index = -1;
		index = handler_parser(argv[0], it_hal_comps);
		/**! Validate index */
		if (-1 == index)
		{
			/**! Free argv[0] memory if any */
			free(argv[0]);
			argv[0] = NULL;
			/**! Notify invalid module */
			log_printf("Selected module is out of range\r\n");
			log_printf("Scenario Done\r\n");
			continue;
		}
		/**********************************************************************/
_MODULE_BEGIN:
		/**! Giving instruction */
		instruction(argv[0]);

		/**! Get test case ID */
		size = ZERO;
		log_printf("Select test application: ");
		/**! Get module handler ID */
		argv[1] = console_getstring(&size);
		log_printf("\r\n");
		/**! Validate test case */
		if (NULL == argv[1])
		{
			/**! Free argv[0] memory if any */
			free(argv[0]);
			argv[0] = NULL;
			/**! Notify invalid test case */
			log_printf("Invalid test case\r\n");
			log_printf("Scenario Done\r\n");
			continue;
		}
		/**********************************************************************/
		/**! Get configuration parameters */
		log_printf("Configuration parameters: ");
		params_str = console_getstring(&size);
		log_printf("\r\n");
		/**! Validate parameter */
		if (NULL == params_str)
		{
			/**! Free argv[0], argv[1] memory if any */
			free(argv[0]);
			argv[0] = NULL;
			free(argv[1]);
			argv[1] = NULL;
			/**! Notify invalid test case */
			log_printf("Invalid parameter or pattern\r\n");
			log_printf("Scenario Done\r\n");
			continue;
		}
		/**! Parsing parameters */
		size = ZERO;
		if (ZERO != params_parser(params_str, &argv[2], &size))
		{
			free(argv[0]);
			argv[0] =  NULL;

			free(argv[1]);
			argv[1] =  NULL;

			free(params_str);
			params_str = NULL;

			log_printf("Error parameters pattern\r\n");
			log_printf("Scenario Done\r\n");
			continue;
		}
		/**********************************************************************/
		/**!                      Call test handler                           */
		/**********************************************************************/
		//
		//
		//
		ercd = it_hal_comps[index].handler(&argv[1], size + 1);
		//
		//
		//
		/**********************************************************************/
		/**< Print test result >*/
		log_printf("\r\n|**********************************************"\
				                "********************************|\r\n");
		log_printf("|                                  TEST RESULT     "\
				                     "                            |\r\n");
		log_printf("|**************************************************"\
				                     "****************************|\r\n");
		/**********************************************************************/
		/**! Validate error code */
		if (-1 == ercd)
		{
			log_printf("Error test case. Please retry.\r\n");
			for (index = 0; index < size + 2; index++)
			{
				free(argv[index]);
				argv[index] = NULL;
			}

			free(params_str);
			params_str = NULL;
			log_printf("Scenario Done\r\n");
			continue;
		}
		log_printf("Auto result: ");
		log_printf("%s\r\n", qc_judgment);
		log_printf("***************************************************"\
				                    "*****************************\r\n");
		log_printf("Scenario Done\r\n");
		/**********************************************************************/
		/**! Indicate that the console have to be re-initialized in
		 *  case the tested module is COM
		 */
		 if (0 == strcmp("1", argv[0]))
		 {
			console_status = CONSOLE_UNINITIALIZED;
		 }
		/**! Free all memory */
		for (index = 1; index < 2 + size; index++)
		{
			free(argv[index]);
			argv[index] = NULL;
		}
		free(params_str);
		params_str = NULL;
		/**< Repeat opinion for testing >*/
		repeat = NULL;
		/**! Provide repeat action */
		log_printf("Please press YES to continue the test or ESC or NO "\
				                                         "to stop!\r\n");
		/**********************************************************************/
		/**! Get user opinion */
		repeat = console_getstring(&size);
		/**! Check error */
		if (NULL == repeat)
		{
			free(argv[0]);
			argv[0] = NULL;
			log_printf("System error: malloc failed.\r\n");
			break;
		}
		/**! Notify back to console interface */
		log_printf("Selected: ");
		log_printf("%s", repeat);
		log_printf("\r\n");
		/**< Check input key >*/
		if  (0 == strcmp(repeat, "YES"))
		{
			free(repeat);
			repeat = NULL;
			goto _MODULE_BEGIN;
		}
		free(argv[0]);
		argv[0] = NULL;
		free(repeat);
		repeat = NULL;
	}
	/* Infinitive loop  */
	while(1);
	return E_OK;
}

/**
 * @brief Print help catalog
 * @detail Print help catalog to console
 * @param[in] NA
 * @param[out] NA
 * @return NA
 */
LOCAL void help(void)
{
	log_printf("***************************************************"\
		                           "*****************************\r\n");
	log_printf("Welcome to Light's verification application\r\n");
	log_printf("***************************************************"\
			                           "*****************************\r\n");
	log_printf("Catalog:\r\n");
	log_printf("\t1> COM module\r\n");
	log_printf("\t2> I2C module (Only valid in stage 2)\r\n");
	log_printf("\t3> PWM module (Only valid in stage 2)\r\n");
	log_printf("\t4> SCU module\r\n");
	log_printf("\t5> SYNCIO module (Only valid in stage 2)\r\n");
	log_printf("\t6> SPI module (Only valid in stage 2)\r\n");
	log_printf("\t7> WDT module.\r\n");
	log_printf("\t8> TIMER module\r\n");
	log_printf("\t9> GPIO module\r\n");
	log_printf("\t10> QSPI module\r\n");
	log_printf("\t11> MIPI module\r\n");
	log_printf("\t12> DDR module\r\n");
	log_printf("\t0> Exit\r\n");
	log_printf("Please select appropriating number for testing "
												 "specific module\r\n");
	log_printf("Select module: ");
}

/**
 * @brief Print instruction of specified tested module 
 * @detail Print instruction of specified tested module
 * @param[in] argv	: module ID as string
 * @param[out] NA
 * @return NA
 */
LOCAL void instruction(char* argv)
{
	if (0 == strcmp(argv, "1"))
	{
		log_printf("COM test cases list:\r\n");
		log_printf("\t1.1 COM_001: Print specific string over usart.\r\n");
		log_printf("\t1.2 COM_002: Reads string number of characters "\
		"from COM port.\r\n");
		log_printf("\t1.3 COM_003: Can not read string from hardware "\
		"when using invalid COM port\r\n");
		log_printf("\t1.4 COM_004: To verify that fail to initialize "\
		"COM1 with invalid baud rate or Port.\r\n");
		log_printf("\t1.5 COM_005: To verify that there is timeout "\
		"error will be reported from COM1 driver in case there is no any "\
		"data is transferred.\r\n");
	}
	else if (0 == strcmp(argv, "2"))
	{
		log_printf("I2C test cases list:\r\n");
		log_printf("\t2.1 I2C_001: CCB protocol master " \
											  "transmit.\r\n");
		log_printf("\t2.2 I2C_002: CCB protocol slave "  \
											  "receive.\r\n");
		log_printf("\t2.3 I2C_003: CCB protocol slave " \
											 "transmit.\r\n");
		log_printf("\t2.4 I2C_004: I2C protocol master" \
											" receive.\r\n");
		log_printf("\t2.5 I2C_005: I2C configure temperature sensor "\
								"after read data register from sensor.\r\n");
		log_printf("\t2.6 I2C_006: I2C read temperature sensor.\r\n");
		log_printf("\t2.7 I2C_007: To verify initialize I2C with"\
		"invalid channel or clock or mode operation and mode address. \r\n");
		log_printf("\t2.8 I2C_008: To verify API transmit and" \
							" receive.\r\n");
		log_printf("\t2.9 I2C_009: To verify performance API.\r\n");

	}
	else if (0 == strcmp(argv, "3"))
	{
		log_printf("PWM test cases list:\r\n");
		log_printf("\t3.1 PWM_001: hal_pwm_init() and "
				                "hal_pwm_start(), normal\r\n");
		log_printf("\t3.2 PWM_002: To verify hal_pwm_stop(), "
				                                           "normal\r\n");
		log_printf("\t3.3 PWM_003: To verify hal_pwm_init() with "
				                 "invalid configure.\r\n");
		log_printf("\t3.4 PWM_004: To verify hal_pwm_start() if the "
				                 "input channel is invalid.\r\n");
		log_printf("\t3.5 PWM_005: To verify hal_pwm_stop() if the "
				                 "input channel is invaid.\r\n");
		log_printf("\t3.6 PWM_006: To verify hal_pwm_update() if the "
				                 "input channel have one invalid param\r\n");
		log_printf("\t3.7 PWM_007: hal_pwm_init() and "
				                "hal_pwm_start(), IRQ is enable \r\n");
		log_printf("\t3.8 PWM_008: To verify hal_pwm_stop(), IRQ\r\n");
		log_printf("\t3.9 PWM_009: hal_pwm_disable_irq() will disable"
				                                                  " IRQ\r\n");
		log_printf("\t3.10 PWM_010: To verify hal_pwm_update() with "
				                                       "IRQ is enabled.\r\n");
		log_printf("\t3.11 PWM_01: To verify performance API.\r\n");
	}
	else if(0 == strcmp(argv, "11"))
	{
		log_printf("SCU test cases list :\r\n");
		log_printf("\t4.1 SCU_001: Print SCU's registers\r\n");
	}
	else if (0 == strcmp(argv, "5"))
	{
		log_printf("SYNCIO test cases list:\r\n");
		log_printf("\t5.1 SYNCIO_001: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.2 SYNCIO_002: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.3 SYNCIO_003: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.4 SYNCIO_004: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.5 SYNCIO_005: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.6 SYNCIO_006: Obsoleted.\r\n");
		log_printf("\t5.7 SYNCIO_007: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.8 SYNCIO_008: Obsoleted.\r\n");
		log_printf("\t5.9 SYNCIO_009: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.10 SYNCIO_010: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.11 SYNCIO_011: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.12 SYNCIO_012: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.13 SYNCIO_013: To verify that SyncIO provide "\
		"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("\t5.15 SYNCIO_015: To verify that SyncIO provide "\
				"the output signal (synchronous pulses) as configured.\r\n");
		log_printf("t5.16 SYNCIO_016: To verify performance API.\r\n");
	}
	else if(0 == strcmp(argv, "6"))
	{
		log_printf("GPIO test cases list :\r\n");
		log_printf("\t6.1 SPI_001: Upstream TX without DMA, "
		                                  "full-duplex mode.\r\n");
		log_printf("\t6.2 SPI_002: Upstream TX without DMA, "
		                                  "half-duplex mode.\r\n");
		log_printf("\t6.3 SPI_003: Upstream RX without DMA, "
		                                  "full-duplex mode.\r\n");
		log_printf("\t6.4 SPI_004: Upstream RX without DMA, "
		                                  "half-duplex mode.\r\n");
		log_printf("\t6.5 SPI_005: Downstream RX without DMA, "
		                                  "full-duplex mode.\r\n");
		log_printf("\t6.6 SPI_006: Downstream RX without DMA, "
		                                  "half-duplex mode.\r\n");
		log_printf("\t6.7 SPI_007: Downstream TX without DMA, "
		                                  "full-duplex mode.\r\n");
		log_printf("\t6.8 SPI_008: Downstream TX without DMA, "
		                                  "half-duplex mode.\r\n");
		log_printf("\t6.9 SPI_009: Downstream RX use DMA, "
		                                  "full-duplex mode.\r\n");
		log_printf("\t6.10 SPI_010: Upstream RX use DMA, "
		                                  "half-duplex mode.\r\n");
		log_printf("\t6.11 SPI_011: Upstream TX use DMA, "
		                                  "half-duplex mode.\r\n");
		log_printf("\t6.12 SPI_012: Downstream TX use DMA, "
		                     "half-duplex mode to TX only slave.\r\n");
		log_printf("\t6.13 SPI_013: To Verify performance API transmit.\r\n ");
		log_printf("\t6.14 SPI_014: To Verify performance API receive.\r\n ");

	}
	else if (0 == strcmp(argv, "7"))
	{
		log_printf("SPI test cases list:\r\n");
		log_printf("\t7.1 WDT_001: initialize and start WatchDog. WDT "
		                                                            "ISR \r\n");
		log_printf("\t7.2 WDT_002: Reset and Stop WatchDog will not let "
		                                           "system reset appears.\r\n");
		log_printf("\t7.3 WDT_003: To Verify performance API.\r\n ");
	}
	else if (0 == strcmp(argv, "8"))
	{
		log_printf("SPI test cases list:\r\n");
		log_printf("\t8.1 TIMER_001: Initialize and enable timer "\
		                                                   "module.\r\n");
		log_printf("\t8.2 TIMER_002: Disable/Stop timer module.\r\n");
		log_printf("\t8.3 TIMER_003: Updates timer period.\r\n");
		log_printf("\t8.4 TIMER_004: To verify " \
								"hal_timer_update_period() is working.\r\n");
		log_printf("\t8.5 TIMER_005: To verify hal_timer_init() with "\
				                 "invalid configure.\r\n");
		log_printf("\t8.6 TIMER_006: To Verify performance API.\r\n ");
	}
	else if(0 == strcmp(argv, "9"))
	{
		log_printf("GPIO test cases list :\r\n");
		log_printf("\t9.1 GPIO_001: Initialize GPIO .\r\n");
		log_printf("\t9.2 GPIO_002: Test interrupt High, Low,"\
				                              "Falling, Rising.\r\n");
		log_printf("\t9.3 GPIO_003: Test interrupt Both.\r\n");
		log_printf("\t9.4 GPIO_004: Test state Set Low.\r\n");
		log_printf("\t9.5 GPIO_005: Test state Set High.\r\n");
		log_printf("\t9.6 GPIO_006: Test read state level.\r\n");
		log_printf("\t9.7 GPIO_007: Test toggle state.\r\n");
		log_printf("\t9.8 GPIO_008: To verify hal_gpio_init() with "
				               "invalid channel or Port_pin or direction.\r\n");
		log_printf("\t9.9 GPIO_009: To verify set low/high or toggle"\
		   "and read state with invalid channel or Port_pin or direction.\r\n");
		log_printf("\t9.10 GPIO_010: To Verify performance API.\r\n ");
	}
	else if(0 == strcmp(argv, "10"))
	{
		log_printf("QSPI test cases list :\r\n");
		log_printf("\t10.1 QSPI_001: Initialize GPIO .\r\n");
	}
	else if(0 == strcmp(argv, "4"))
	{
		log_printf("MIPI test cases list :\r\n");
		log_printf("\t11.1 MIPI_001: Test TX only\r\n");
		log_printf("\t11.2 MIPI_002: Test TX interrupt\r\n");
		log_printf("\t11.3 MIPI_003: Test RX interrupt\r\n");
		log_printf("\t11.4 MIPI_004: Test RX pass TX\r\n");
	}
	else if(0 == strcmp(argv, "12"))
	{
		log_printf("DDR test cases list :\r\n");
		log_printf("\t12.1 DDR_001: Test verify write data to memory.\r\n");
		log_printf("\t12.2 DDR_002: Test performance bus speed\r\n");
	}
}
/**! Local functions **********************************************************/

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
