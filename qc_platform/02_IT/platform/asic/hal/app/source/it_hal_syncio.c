/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_syncio.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    19-Apr-2016
 * @brief   This file contains expand of the hal_syncio driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	20-Feb-2016	Initial revision:
 *                      - Infrastructure.
 * * 1.0.1	19-Apr-2016 Test HAL baseline 1.0.0
 */
/******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "it_hal_syncio.h"
#include "it_log_swapper.h"
/**! Include GPIO to simulate hardware trigger signal */
#include "hal_gpio.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct hal_syncio_test
{
	hal_syncio_channel_t   CC; 			/**! Channels number */
	syncio_trig_mode_t     TT; 			/**! Trigger source */
	uint32_t               LLLLLLLL; 	/**! Latency signal 1 */
	uint32_t               HHHHHHHH; 	/**! Latency signal 2 */
	uint32_t               QQQQQQQQ; 	/**! Pulse width of signal 1 */
	uint32_t               PPPPPPPP; 	/**! Pulse width of signal 2 */
	uint8_t                RR; 			/**! Repeat number */
	syncio_infinite_mode_t EE; 	    	/**! Infinite mode */
} hal_syncio_test_t;

typedef struct
{
	volatile uint32_t gpio_swport_dr;
	volatile uint32_t gpio_swport_ddr;
	volatile uint32_t gpio_swport_ctl;
}port_regs_t ;
/* Private define ------------------------------------------------------------*/
#define IT_SYNCIO_001 1
#define IT_SYNCIO_002 2
#define IT_SYNCIO_003 3
#define IT_SYNCIO_004 4
#define IT_SYNCIO_005 5
#define IT_SYNCIO_006 6
#define IT_SYNCIO_007 7
#define IT_SYNCIO_008 8
#define IT_SYNCIO_009 9
#define IT_SYNCIO_010 10
#define IT_SYNCIO_011 11
#define IT_SYNCIO_012 12
#define IT_SYNCIO_013 13
#define IT_SYNCIO_015 15
/* Private macro -------------------------------------------------------------*/
/* Exported global variables --------------------------------------------------*/
LOCAL volatile uint16_t it_synio_current_test = 0;
LOCAL volatile uint32_t falling_edge_counter = 0;
LOCAL volatile uint32_t repeat_done_counter = 0;
LOCAL volatile uint32_t pulse_done_counter = 0;
LOCAL volatile uint16_t num_of_pulses = 0;
LOCAL volatile hal_syncio_channel_t tested_channel = SG_CHANNEL_MAX;
/* Static functions ----------------------------------------------------------*/
/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 */
LOCAL int it_hal_syncio_001(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_002(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_003(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_004(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_005(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_006(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_007(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_008(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_009(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_010(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_011(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_012(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_013(char** argv, int argc);

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_015(char** argv, int argc);

/**
 * @brief To verify that SyncIO performance API
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that Performance SyncIO provide the output signal
 *  (synchronous pulses) as configured.
 */
LOCAL int it_hal_syncio_016(char** argv, int argc);
/**
 * @brief SyncIO callback function
 * @param[in] NA
 * @param[out] child
 * @param[out] irq_status
 * @details SyncIO callback function
 */
LOCAL void it_syncio_callback(hal_syncio_channel_t chid,
		                              hal_syncio_irq_mode_t irq_status);

/**
 * @brief delay function
 * @detail add delay time to support for testing APIs of timer module
 * @param[in] time: delay time (ms)
 * @param[out] NA
 */
LOCAL void delay_ms(__IO uint32_t time);

/**
 * @brief: Simulate hardware trigger signal using Asic's own GPIO pin.
 * @param[in] pin		: pin number
 * @param[in] pulses	: number of pulses
 * @param[in] delay_clk	: number of instructions to delay between 2 
 *                        pulses
 * @param[out]: NA
 * @return: NA
 * 
 * @details:
 *  Simulate hardware trigger signal using Asic's own GPIO pin.
 **/
LOCAL void it_syncio_hw_trigger(hal_gpio_port_t port, hal_gpio_pin_t pin, \
                                     uint8_t pulses, uint32_t delay_clk);

/**
 * @brief Input parameters for testcase
 * @detail Input parameters for testcase
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
LOCAL hal_syncio_test_t config_testcase(char** argv, int argc)
{
	hal_syncio_test_t syncio_test;

	/**! Get Channel number via argv[0] */
	syncio_test.CC = (hal_syncio_channel_t)strtol(argv[0], NULL, 16);

	/**! Get Trigger source via argv[1] */
	syncio_test.TT = (syncio_trig_mode_t)strtol(argv[1], NULL, 16);

	/**! Get latency signal 1 via argv[2] */
	syncio_test.LLLLLLLL = (uint32_t)strtol(argv[2], NULL, 16);

	/**! Get Latency signal 2 via argv[3] */
	syncio_test.HHHHHHHH = (uint32_t)strtol(argv[3], NULL, 16);

	/**! Get Pulse width of signal 1 via argv[4] */
	syncio_test.QQQQQQQQ = (uint32_t)strtol(argv[4], NULL, 16);

	/**! Get Pulse width of signal 2 via argv[5] */
	syncio_test.PPPPPPPP = (uint32_t)strtol(argv[5], NULL, 16);

	/**! Get Repeat number via argv[6] */
	syncio_test.RR = (uint8_t)strtol(argv[6], NULL, 16);
	num_of_pulses = syncio_test.RR;

	/**! Get Infinite mode via argv[7] */
	if (strtol(argv[7], NULL, 10))
	{
		syncio_test.EE = SG_INF_EN;
	}
	else
	{
		syncio_test.EE = SG_INF_DIS;
	}

	return syncio_test;
}

/* Private variables ---------------------------------------------------------*/
LOCAL it_map_t it_syncio_tests_table[] = {
	{"SYNCIO_001", it_hal_syncio_001},
	{"SYNCIO_002", it_hal_syncio_002},
	{"SYNCIO_003", it_hal_syncio_003},
	{"SYNCIO_004", it_hal_syncio_004},
	{"SYNCIO_005", it_hal_syncio_005},
	{"SYNCIO_006", it_hal_syncio_006},
	{"SYNCIO_007", it_hal_syncio_007},
	{"SYNCIO_008", it_hal_syncio_008},
	{"SYNCIO_009", it_hal_syncio_009},
	{"SYNCIO_010", it_hal_syncio_010},
	{"SYNCIO_011", it_hal_syncio_011},
	{"SYNCIO_012", it_hal_syncio_012},
	{"SYNCIO_013", it_hal_syncio_013},
	{"SYNCIO_015", it_hal_syncio_015},
	{"SYNCIO_016", it_hal_syncio_016},
	{"", NULL}
};

/* Exported functions --------------------------------------------------------*/

/**
 * @brief SYNCIO module's testing handler
 * @detail to SYNCIO module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_hal_syncio_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_syncio_tests_table);
	if (-1 != index)
	{
		return it_syncio_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief: Simulate hardware trigger signal using Asic's own GPIO pin.
 * @param[in] pin		: pin number
 * @param[in] pulses	: number of pulses
 * @param[in] delay_clk	: number of instructions to delay between 2 
 *                        pulses
 * @param[out]: NA
 * @return: NA
 * 
 * @details:
 *  Simulate hardware trigger signal using Asic's own GPIO pin.
 **/
LOCAL void it_syncio_hw_trigger(hal_gpio_port_t port, hal_gpio_pin_t pin, \
                                         uint8_t pulses, uint32_t delay_clk)
{
	hal_gpio_t gpio;
	volatile uint8_t pulse = 0;
	volatile uint32_t clk;

	/**! Configure port A, pin 0 */
	gpio.port = port;
	gpio.pin = pin;
	gpio.direction = GPIO_DIR_OUT;

	/**! Initializing GPIO */
	hal_gpio_init(&gpio);

	while(pulse < pulses)
	{
		/**! Set output to high */
		hal_gpio_set_high(&gpio);
		/**! Set clk to create pulse width as 100 clk */
		clk = 100;
		while (--clk);
		/**! Clear gpio to low */
		hal_gpio_set_low(&gpio);
		/**! Set clk to create pulse width as 100 clk */
		clk = delay_clk;
		while (--clk);
		/**! Move on to next pulse */
		pulse++;
	}
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 */
LOCAL int it_hal_syncio_001(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	} 
	
	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Reset tested channel */
	tested_channel = SG_CHANNEL_MAX;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);
	
	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);
	
	/**! Enable SyncIO channel */
	hal_syncio_enable(channel);

	/**! Do software trigger */
	hal_syncio_trigger();

	/**! Assert to check IRQ has not been invoked */
	qc_assert(SG_CHANNEL_MAX == tested_channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_002(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_002;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! SyncIO IRQ profile */
	hal_syncio_irq_t irq_falling_profile = \
			{
				.irq_modes = SG_INTR_FALLING_EDGE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);

	/**! Enable IRQ falling edge */
	hal_syncio_enable_irq(channel, &irq_falling_profile);

	/**! Enable SyncIO channel */
	hal_syncio_enable(channel);

	/**! Do software trigger */
	hal_syncio_trigger();

	/**! Delay a period of time */
	delay_ms(5000);

	/**! Assert repeat done IRQ is invoked */
	qc_assert(1 < falling_edge_counter);

	/**! Assert IRQ channel */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_003(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_003;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! SyncIO IRQ profile */
	hal_syncio_irq_t irq_falling_profile = \
			{
				.irq_modes = SG_INTR_FALLING_EDGE,
				.callback_handler = it_syncio_callback
			};
	hal_syncio_irq_t irq_repeat_profile = \
			{
				.irq_modes = SG_INTR_REPEAT_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);

	/**! Enable IRQ falling edge */
	hal_syncio_enable_irq(channel, &irq_falling_profile);

	/**! Enable IRQ repeat done */
	hal_syncio_enable_irq(channel, &irq_repeat_profile);

	/**! Enable SyncIO channel */
	hal_syncio_enable(channel);

	/**! Do software trigger */
	hal_syncio_trigger();

	/**! Delay a period of time */
	delay_ms(5000);

	/**! Assert repeat done IRQ is invoked */
	qc_assert(1 == repeat_done_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_004(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_002;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! SyncIO IRQ profile */
	hal_syncio_irq_t irq_pulse_profile = \
			{
				.irq_modes = SG_INTR_PULSE_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);

	/**! Enable IRQ pulse done */
	hal_syncio_enable_irq(channel, &irq_pulse_profile);

	/**! Enable SyncIO channel */
	hal_syncio_enable(channel);

	/**! Do software trigger */
	hal_syncio_trigger();

	/**! Delay a period of time */
	delay_ms(5000);

	/**! Assert repeat done IRQ is invoked */
	qc_assert(1 < pulse_done_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_005(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_005;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! SyncIO IRQ profile */
	hal_syncio_irq_t irq_pulse_profile = \
			{
				.irq_modes = SG_INTR_PULSE_DONE,
				.callback_handler = it_syncio_callback
			};
	hal_syncio_irq_t irq_repeat_profile = \
			{
				.irq_modes = SG_INTR_REPEAT_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);

	/**! Enable IRQ pulse done */
	hal_syncio_enable_irq(channel, &irq_pulse_profile);

	/**! Enable IRQ repeat done */
	hal_syncio_enable_irq(channel, &irq_repeat_profile);

	/**! Enable SyncIO channel */
	hal_syncio_enable(channel);

	/**! Do software trigger */
	hal_syncio_trigger();

	/**! Delay a period of time */
	delay_ms(5000);

	/**! Assert repeat done IRQ is invoked */
	qc_assert(1 == repeat_done_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_006(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_006;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	hal_syncio_irq_t irq_repeat_profile = \
			{
				.irq_modes = SG_INTR_REPEAT_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);

	/**! Enable IRQ repeat done */
	hal_syncio_enable_irq(channel, &irq_repeat_profile);

	/**! Enable SyncIO channel */
	hal_syncio_enable(channel);

	/**! Do software trigger */
	hal_syncio_trigger();

	/**! Delay a period of time */
	delay_ms(5000);

	/**! Assert repeat done IRQ is invoked */
	qc_assert(1 < repeat_done_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_007(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((8 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_007;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	hal_syncio_irq_t irq_repeat_profile = \
			{
				.irq_modes = SG_INTR_REPEAT_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);

	/**! Enable IRQ falling edge */
	hal_syncio_enable_irq(channel, &irq_repeat_profile);

	/**! Enable SyncIO channel */
	hal_syncio_enable(channel);

	/**! Do software trigger */
	hal_syncio_trigger();

	/**! Delay a period of time */
	delay_ms(5000);

	/**! Assert repeat done IRQ is invoked */
	qc_assert(1 == repeat_done_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_008(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_008;

	/**! Number of HW trigger pulses */
	uint8_t pulses = (uint8_t)strtol(argv[8], NULL, 10);
	volatile uint8_t pulse = 0;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! IRQ repeat done profile */
	hal_syncio_irq_t irq_repeat_profile = \
			{
				.irq_modes = SG_INTR_REPEAT_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);

	/**! Enable IRQ falling edge */
	hal_syncio_enable_irq(channel, &irq_repeat_profile);

	while (pulse < pulses)
	{
		/**! Enable SyncIO channel */
		hal_syncio_enable(channel);

		/**! Hardware trigger mode (via UPSTREAM_SYNCIN pin) */
		it_syncio_hw_trigger(GPIO_PORTA, GPIO_PIN_0, 1, 1000);

		/**! Move on to next sequence */
		pulse++;
	}

	/**! Delay to make sure pulses is generated */
	delay_ms(5000);
	
	/**! Assert callback counter */
	qc_assert(pulses < repeat_done_counter);
	qc_assert(0 == falling_edge_counter);
	qc_assert(0 == pulse_done_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_009(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_009;

	/**! Number of HW trigger pulses */
	uint8_t pulses = (uint8_t)strtol(argv[8], NULL, 10);
	volatile  uint8_t pulse = 0;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! IRQ repeat done profile */
	hal_syncio_irq_t irq_repeat_profile = \
			{
				.irq_modes = SG_INTR_REPEAT_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
	{
		/**! Configured parameters */
		hal_syncio_config(channel, &syncio_profile);
	}

	/**! Get channel */
	channel = ret_config.CC;

	/**! Enable IRQ falling edge */
	hal_syncio_enable_irq(channel, &irq_repeat_profile);

	while (pulse < pulses)
	{
		for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
		{
			/**! Enable SyncIO channel */
			hal_syncio_enable(channel);
		}

		/**! Get channel */
		channel = ret_config.CC;

		/**! Hardware trigger mode (via UPSTREAM_SYNCIN pin) */
		it_syncio_hw_trigger(GPIO_PORTA, GPIO_PIN_0, 1, 1000);

		/**! Move on to next sequence */
		pulse++;
	}

	/**! Delay to make sure pulses is generated */
	delay_ms(5000);

	/**! Assert callback counter */
	qc_assert(pulses == repeat_done_counter);
	qc_assert(0 == falling_edge_counter);
	qc_assert(0 == pulse_done_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_010(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_010;

	/**! Number of HW trigger pulses */
	uint8_t pulses = (uint8_t)strtol(argv[8], NULL, 10);
	volatile uint8_t pulse = 0;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! SyncIO IRQ profile */
	hal_syncio_irq_t irq_falling_profile = \
			{
				.irq_modes = SG_INTR_FALLING_EDGE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
	{
		/**! Configured parameters */
		hal_syncio_config(channel, &syncio_profile);
	}

	/**! Get channel */
	channel = ret_config.CC;

	/**! Enable IRQ falling edge */
	hal_syncio_enable_irq(channel, &irq_falling_profile);

	while (pulse < pulses)
	{
		for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
		{
			/**! Enable SyncIO channel */
			hal_syncio_enable(channel);
		}

		/**! Get channel */
		channel = ret_config.CC;

		/**! Hardware trigger mode (via UPSTREAM_SYNCIN pin) */
		it_syncio_hw_trigger(GPIO_PORTA, GPIO_PIN_0, 1, 1000);

		/**! Move on to next sequence */
		pulse++;
	}

	/**! Delay to make sure pulses is generated */
	delay_ms(5000);

	/**! Assert callback counter */
	qc_assert(pulses < falling_edge_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_011(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_011;

	/**! Number of HW trigger pulses */
	uint8_t pulses = (uint8_t)strtol(argv[8], NULL, 10);
	volatile uint8_t pulse = 0;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! SyncIO IRQ profile */
	hal_syncio_irq_t irq_falling_profile = \
			{
				.irq_modes = SG_INTR_FALLING_EDGE,
				.callback_handler = it_syncio_callback
			};

	/**! IRQ repeat done profile */
	hal_syncio_irq_t irq_repeat_profile = \
			{
				.irq_modes = SG_INTR_REPEAT_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
	{
		/**! Configured parameters */
		hal_syncio_config(channel, &syncio_profile);
	}

	/**! Get channel */
	channel = ret_config.CC;

	/**! Enable IRQ falling edge */
	hal_syncio_enable_irq(channel, &irq_falling_profile);

	/**! Enable IRQ repeat done */
	hal_syncio_enable_irq(channel, &irq_repeat_profile);

	while (pulse < pulses)
	{
		for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
		{
			/**! Enable SyncIO channel */
			hal_syncio_enable(channel);
		}

		/**! Get channel */
		channel = ret_config.CC;

		/**! Hardware trigger mode (via UPSTREAM_SYNCIN pin) */
		it_syncio_hw_trigger(GPIO_PORTA, GPIO_PIN_0, 1, 1000);

		/**! Move on to next sequence */
		pulse++;
	}

	/**! Delay to make sure pulses is generated */
	delay_ms(5000);

	/**! Assert callback counter */
	qc_assert(pulses == repeat_done_counter);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_012(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_012;

	/**! Number of HW trigger pulses */
	uint8_t pulses = (uint8_t)strtol(argv[8], NULL, 10);
	volatile uint8_t pulse = 0;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! SyncIO IRQ profile */
	hal_syncio_irq_t irq_pulse_profile = \
			{
				.irq_modes = SG_INTR_PULSE_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
	{
		/**! Configured parameters */
		hal_syncio_config(channel, &syncio_profile);
	}

	/**! Get channel */
	channel = ret_config.CC;

	/**! Enable IRQ pulse done */
	hal_syncio_enable_irq(channel, &irq_pulse_profile);

	while (pulse < pulses)
	{
		for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
		{
			/**! Enable SyncIO channel */
			hal_syncio_enable(channel);
		}

		/**! Get channel */
		channel = ret_config.CC;

		/**! Hardware trigger mode (via UPSTREAM_SYNCIN pin) */
		it_syncio_hw_trigger(GPIO_PORTA, GPIO_PIN_0, 1, 1000);

		/**! Move on to next sequence */
		pulse++;
	}

	/**! Delay to make sure pulses is generated */
	delay_ms(5000);

	/**! Assert callback counter */
	qc_assert(pulses < pulse_done_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_013(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Indicate current test flag */
	it_synio_current_test = IT_SYNCIO_013;

	/**! Number of HW trigger pulses */
	uint8_t pulses = (uint8_t)strtol(argv[8], NULL, 10);
	volatile uint8_t pulse = 0;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! SyncIO IRQ profile */
	hal_syncio_irq_t irq_pulse_profile = \
			{
				.irq_modes = SG_INTR_PULSE_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! IRQ repeat done profile */
	hal_syncio_irq_t irq_repeat_profile = \
			{
				.irq_modes = SG_INTR_REPEAT_DONE,
				.callback_handler = it_syncio_callback
			};

	/**! Reset callback counters */ 
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
	{
		/**! Configured parameters */
		hal_syncio_config(channel, &syncio_profile);
	}

	/**! Get channel */
	channel = ret_config.CC;

	/**! Enable IRQ pulse done */
	hal_syncio_enable_irq(channel, &irq_pulse_profile);

	/**! Enable IRQ repeat done */
	hal_syncio_enable_irq(channel, &irq_repeat_profile);

	while (pulse < pulses)
	{
		for (channel = SG_CHANNEL_0; channel < SG_CHANNEL_MAX; channel++)
		{
			/**! Enable SyncIO channel */
			hal_syncio_enable(channel);
		}

		/**! Get channel */
		channel = ret_config.CC;

		/**! Hardware trigger mode (via UPSTREAM_SYNCIN pin) */
		it_syncio_hw_trigger(GPIO_PORTA, GPIO_PIN_0, 1, 1000);

		/**! Move on to next sequence */
		pulse++;
	}

	/**! Delay to make sure pulses is generated */
	delay_ms(5000);

	/**! Assert callback counter */
	qc_assert(pulses == repeat_done_counter);

	/**! Verify tested channel's IRQ is invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that SyncIO provide the output signal 
 *  (synchronous pulses) as configured. 
 */
LOCAL int it_hal_syncio_015(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (9 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Number of HW trigger pulses */
	uint8_t pulses = (uint8_t)strtol(argv[8], NULL, 10);
	volatile  uint8_t pulse = 0;

	/**! Reset number of pulse */
	num_of_pulses = 0;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Reset tested channel */
	tested_channel = channel;

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
		{
			.inf_mode  = ret_config.EE,
			.trig_mode = ret_config.TT,
			.lat1      = ret_config.LLLLLLLL,
			.lat2      = ret_config.HHHHHHHH,
			.width1    = ret_config.QQQQQQQQ,
			.width2    = ret_config.PPPPPPPP,
			.repeat    = ret_config.RR
		};

	/**! IRQ pulse done profile */
	hal_syncio_irq_t irq_pulse_done = \
		{
			.irq_modes = SG_INTR_PULSE_DONE,
			.callback_handler = it_syncio_callback

		};

	/**! Reset callback counters */
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;
	
	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);

	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);

	/**! Enable SyncIO channel */
	hal_syncio_enable(channel);

	/**! Enable IRQ pulse done */
	hal_syncio_enable_irq(channel, &irq_pulse_done);

	while (pulse < pulses)
	{
		/**! Hardware trigger mode (via UPSTREAM_SYNCIN pin) */
		it_syncio_hw_trigger(GPIO_PORTA, GPIO_PIN_0, 1, 1000);

		/**! Move on to next sequence */
		pulse++;
	}

	/**! Delay to make sure pulses is generated */
	delay_ms(5000);

	/**! Assert callback counter */
	qc_assert(pulses == pulse_done_counter);
	qc_assert(0 == repeat_done_counter);
	qc_assert(0 == falling_edge_counter);

	/**! Verify tested channel's IRQ invoked */
	qc_assert(tested_channel == channel);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that SyncIO performance API
 *  (synchronous pulses) as configured.
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that Performance SyncIO provide the output signal
 *  (synchronous pulses) as configured.
 */
LOCAL int it_hal_syncio_016(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((9 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (8 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Reset number of pulse */
	num_of_pulses = 0;
	uint8_t select = 0;
	hal_gpio_t gpio = {0, 0, 1};
	/**! Reset tested channel */
	tested_channel = SG_CHANNEL_MAX;

	/**! Get information from configuration parameters */
	hal_syncio_test_t ret_config = config_testcase(argv,argc);

	/**! Get channel */
	hal_syncio_channel_t channel = ret_config.CC;

	/**! Get select API */
	select = (uint8_t)strtol(argv[8], NULL, 10);

	/**! SyncIO configuration profile */
	hal_syncio_cfg_t syncio_profile = \
			{
				.inf_mode  = ret_config.EE,
				.trig_mode = ret_config.TT,
				.lat1      = ret_config.LLLLLLLL,
				.lat2      = ret_config.HHHHHHHH,
				.width1    = ret_config.QQQQQQQQ,
				.width2    = ret_config.PPPPPPPP,
				.repeat    = ret_config.RR
			};

	/**! Reset callback counters */
	repeat_done_counter  = 0;
	pulse_done_counter   = 0;
	falling_edge_counter = 0;

	/**! Initialize GPIO */
	hal_gpio_init(&gpio);
	/**! Reset all test point to default */
	qc_assert_reset();

	if(0 == select)
	{
	hal_gpio_set_high(&gpio);
	}
	/**! Initialize SyncIO global features */
	hal_syncio_init(channel);
	if(0 == select)
	{
	hal_gpio_set_low(&gpio);
	}

	if(1 == select)
	{
	hal_gpio_set_high(&gpio);
	}
	/**! Configured parameters */
	hal_syncio_config(channel, &syncio_profile);
	if(1 == select)
	{
	hal_gpio_set_low(&gpio);
	}

	if(2 == select)
	{
	hal_gpio_set_high(&gpio);
	}
	/**! Enable SyncIO channel */
	hal_syncio_enable(channel);
	if(2 == select)
	{
	hal_gpio_set_low(&gpio);
	}

	if(3 == select)
	{
	hal_gpio_set_high(&gpio);
	}
	/**! Do software trigger */
	hal_syncio_trigger();
	if(3 == select)
	{
	hal_gpio_set_low(&gpio);
	}
	/**! Assert to check IRQ has not been invoked */
	qc_assert(SG_CHANNEL_MAX == tested_channel);

	/**! Do judgment */
	qc_report();

	return 0;

}
/**
 * @brief delay function
 * @detail add delay time to support for testing APIs of timer module
 * @param[in] time: delay time (ms)
 * @param[out] NA
 */
LOCAL void delay_ms(__IO uint32_t time)
{
	_delay_ms(time);
}

/**
 * @brief SyncIO callback function
 * @param[in] NA
 * @param[out] child
 * @param[out] irq_status
 * @details SyncIO callback function
 */
LOCAL void it_syncio_callback(hal_syncio_channel_t chid,
		                              hal_syncio_irq_mode_t irq_status)
{
	/**! Indicate IRQ's channel */
	tested_channel &= chid;

	/**! Invoke upper state */
	switch (irq_status)
	{
		case SG_INTR_FALLING_EDGE:
		{
			falling_edge_counter++;
			break;
		}
		case SG_INTR_PULSE_DONE:
		{
			pulse_done_counter++;
			break;
		}
		case SG_INTR_REPEAT_DONE:
		{
			repeat_done_counter++;
			/**! Verify other IRQ */
			switch (it_synio_current_test)
			{
				case IT_SYNCIO_003:
				{
					qc_assert(0 == (falling_edge_counter % num_of_pulses));
					break;
				}
				case IT_SYNCIO_005:
				{
					qc_assert(0 == (pulse_done_counter % num_of_pulses));
					break;
				}
				case IT_SYNCIO_006:
				{
					qc_assert(0 == falling_edge_counter);
					qc_assert(0 == pulse_done_counter);
					break;
				}
				case IT_SYNCIO_007:
				{
					qc_assert(0 == falling_edge_counter);
					qc_assert(0 == pulse_done_counter);
					break;
				}
				case IT_SYNCIO_011:
				{
					qc_assert(0 == (falling_edge_counter % num_of_pulses));
					qc_assert(0 == pulse_done_counter);
					break;
				}
				case IT_SYNCIO_013:
				{
					qc_assert(0 == (pulse_done_counter % num_of_pulses));
					qc_assert(0 == falling_edge_counter);
					break;
				}
				default:
				{
					/**! Misra-C */
					break;
				}
			}
			break;
		}
		default:
		{
			/**! Misra-C */
			break;
		}
	}
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
