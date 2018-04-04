/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    it_hal_pwm.c
 * @author  The LightCo
 * @version V1.0.2
 * @date    April-19-2016
 * @brief   This file contains expand of the hal_pwm driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	20-Feb-2016	Initial revision:
 *                      - Infrastructure.
 * * 1.0.1	19-Apr-2016 Test HAL baseline 1.0.0
 * * 1.0.2  29-Apr-2016 Update according to change of development code structure
 */
/******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "it_hal_pwm.h"
#include "it_log_swapper.h"
#include "board_config.h"
#include "hal_gpio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PWM_CHANNEL_DEFAULT               PWM0_CH1
#define PWM_CFG_DEFAULT					  {PWM_ALIGN_LEFT, PWM_CONTINUOUS}
#define PWM_PROFILE_DEFAULT				  {{{{0,1},{0,1},{0,1}},3},0x00000000,0}
#define PWM_IRQ_PROFILE_DEFAULT			  {PWM_IRQ_ONE_PATTERN_GENERATED, NULL}
#define PWM_TOTAL_CHANNELS                12
#define PWM_PAIRS_MAX_PER_CHANNEL         3
/* Private macro -------------------------------------------------------------*/

/* Exported global varibles --------------------------------------------------*/

/* Static functions ----------------------------------------------------------*/

/**
 * @brief to verify PWM signal after started
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details - To verify that any valid pwm channel is initialized successfully
 *            via API hal_pwm_init(), start via hal_pwm_start()
 */
LOCAL int it_hal_pwm_001(char** argv, int argc);

/**
 * @brief To verify that any valid pwm channel is stopped by calling
 *        hal_pwm_stop()
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that any valid pwm channel is stopped by calling
 *          hal_pwm_stop()
 */
LOCAL int it_hal_pwm_002(char** argv, int argc);

/**
 * @brief to test hal_pwm_init API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details
 *         - To verify that API hal_pwm_init() will return error if the one
 *         input parameter is invalid.
 */
LOCAL int it_hal_pwm_003(char** argv, int argc);

/**
 * @brief to test hal_pwm_start() API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that API hal_pwm_start will return error if the
 *         input channel is invalid.
 */
LOCAL int it_hal_pwm_004(char** argv, int argc);

/**
 * @brief to test hal_pwm_stop() API in abnormal case
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that API hal_pwm_stop will return error if the input
 *          channel is invalid.
 */
LOCAL int it_hal_pwm_005(char** argv, int argc);

/**
 * @brief to test hal_pwm_update() API in abnormal case
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that API hal_pwm_update will return error if the input
 *          channel has not been initialized.
 */
LOCAL int it_hal_pwm_006(char** argv, int argc);

/**
 * @brief to test hal_pwm_init() and hal_pwm_start() API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that any valid pwm channel is initialized successfully
 *         via API hal_pwm_init(), start via hal_pwm_start() in case IRQ
 *         is enabled.
 */
LOCAL int it_hal_pwm_007(char** argv, int argc);

/**
 * @brief to test hal_pwm_stop API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that any valid pwm channel is initialized successfully
 *         via API hal_pwm_init(), start via hal_pwm_start() and stop after
 *         calling hal_pwm_stop() in case IRQ is enabled
 */
LOCAL int it_hal_pwm_008(char** argv, int argc);

/**
 * @brief to test hal_pwm_stop and hal_pwm_disable_irq() APIs
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details
 *          - To verify that any valid pwm channel is initialized
 *          successfully via API hal_pwm_init(), start via
 *          hal_pwm_start() and stop after calling hal_pwm_stop()
 *          in case IRQ is enables
 *          - IRQ of indicated channel is disable in next sequence
 *          after calling hal_pwm_disable_irq().
 */
LOCAL int it_hal_pwm_009(char** argv, int argc);

/**
 * @brief to test hal_pwm_update() API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details
 *           - To verify that any valid pwm channel is initialized successfully
 *           via API hal_pwm_init(), start via hal_pwm_start(). PWM profile will
 *           be updated after invoking API hal_pwm_update()
 */
LOCAL int it_hal_pwm_010(char** argv, int argc);

/**
 * @brief to test performance API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 */
LOCAL int it_hal_pwm_011(char** argv, int argc);
/**
 * @brief to convert configuration parameters from user input
 * @param[in] argv		: user input parameters as list of strings
 * @param[in] argc		: user input parameter counter
 * @param[out] pwm_cfg	: configuration modes
 * @param[out] pwm_profile	: desire pwm profile
 * @return 	NA
 * @detail to get configuration parameters from user input
 */
LOCAL void it_hal_pwm_config(hal_pwm_init_t* pwm_cfg, \
		                  hal_pwm_output_t* pwm_profile, char** argv, int argc);

/**
 * @brief to be set as callback function of PWM
 * @param[in] NA
 * @param[out] NA
 * @return 	NA
 * @detail to be set as callback function of PWM
 */
LOCAL void it_hal_pwm_callback(hal_pwm_channel_t channel);

/**
 * @brief to reset callback counter of PWM
 * @param[in] NA
 * @param[out] NA
 * @return 	NA
 * @detail to reset callback counter of PWM
 */
LOCAL void it_hal_pwm_reset_callback(hal_pwm_channel_t channel);

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
/* Private variables ---------------------------------------------------------*/
LOCAL volatile uint32_t it_hal_pwm_callback_counter[12];

LOCAL it_map_t it_pwm_tests_table[] = {
		{"PWM_001", it_hal_pwm_001},
		{"PWM_002", it_hal_pwm_002},
		{"PWM_003", it_hal_pwm_003},
		{"PWM_004", it_hal_pwm_004},
		{"PWM_005", it_hal_pwm_005},
		{"PWM_006", it_hal_pwm_006},
		{"PWM_007", it_hal_pwm_007},
		{"PWM_008", it_hal_pwm_008},
		{"PWM_009", it_hal_pwm_009},
		{"PWM_010", it_hal_pwm_010},
		{"PWM_011", it_hal_pwm_011},
		{"",  NULL}
};

/* Exported functions --------------------------------------------------------*/

/**
 * @brief PWM module's testing handler
 * @detail to PWM module's testing handler
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @return 0	 : success
 *         Others: instruction failed
 */
int it_hal_pwm_handler(char** argv, int argc)
{
	/**< Check command counter >*/
	if (1 >= argc) return -1;
	if (NULL == argv) return -1;
	/**< Get command index, mean test case name >*/
	int index = handler_parser(*argv, it_pwm_tests_table);
	if (-1 != index)
	{
		return it_pwm_tests_table[index].handler(&argv[1], argc - 1);
	}
	else
		return index;
}

/**
 * @brief to verify PWM signal after started
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details - To verify that any valid pwm channel is initialized successfully
 *            via API hal_pwm_init(), start via hal_pwm_start()
 */
LOCAL int it_hal_pwm_001(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((11 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (11 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = PWM_CHANNEL_DEFAULT;
	/**! PWM mode */
	hal_pwm_init_t    pwm_cfg = PWM_CFG_DEFAULT;
	/**! PWM profile */
	hal_pwm_output_t  pwm_profile = PWM_PROFILE_DEFAULT;

	/**! Reset all PWM channel to default state */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initialize PWM to default */
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
		/**! Disable IRQ */
		hal_pwm_disable_irq(channel);
	}

	/**! Convert inputs */
	it_hal_pwm_config(&pwm_cfg, &pwm_profile, argv, argc);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing PWM channels and check error.*/
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
	}

	/**! Start PWM channels.*/
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		hal_pwm_start(channel);
	}

	/**! Delay 1s */
	delay_ms(1000);

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief To verify that any valid pwm channel is stopped by calling
 *        hal_pwm_stop()
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that any valid pwm channel is stopped by calling
 *          hal_pwm_stop()
 */
LOCAL int it_hal_pwm_002(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((11 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (11 != argc || NULL == argv)"
						                                               " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = PWM_CHANNEL_DEFAULT;
	/**! PWM mode */
	hal_pwm_init_t    pwm_cfg = PWM_CFG_DEFAULT;
	/**! PWM profile */
	hal_pwm_output_t  pwm_profile = PWM_PROFILE_DEFAULT;

	/**! Reset PWM channels to default state */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initialize PWM to default */
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
		/**! Disable IRQ */
		hal_pwm_disable_irq(channel);
	}

	/**! Convert inputs */
	it_hal_pwm_config(&pwm_cfg, &pwm_profile, &argv[0], argc);

	/**! Initializing PWM channels.*/
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
	}

	/**! Start PWM channels.*/
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		hal_pwm_start(channel);
	}

	/**! Delay a period of time to do measurement */
	delay_ms(1000);

	/**! Invoke PWM stop to stop PWM */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		hal_pwm_stop(channel);
	}

	/**! Do judgment */
	qc_report();

	return 0;
}


/**
 * @brief to test hal_pwm_init API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details
 *         - To verify that API hal_pwm_init() will return error if the one
 *         input parameter is invalid.
 */
LOCAL int it_hal_pwm_003(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((12 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (11 != argc || NULL == argv)"
						  " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = (hal_pwm_channel_t)strtol(argv[0], NULL, 16);
	/**! PWM mode */
	hal_pwm_init_t pwm_cfg = PWM_CFG_DEFAULT;
	/**! PWM profile */
	hal_pwm_output_t pwm_profile = PWM_PROFILE_DEFAULT;

	/**! Convert inputs */
	it_hal_pwm_config(&pwm_cfg, &pwm_profile, &argv[1], argc - 1);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing PWM channels and check error.*/
	hal_pwm_init(channel, &pwm_cfg, &pwm_profile);

	/**! Check assert error */
	qc_assert(qc_assert_status());

	/**! Do judgment */
	qc_report();

	/**! Return 0*/
	return 0;
}

/**
 * @brief to test hal_pwm_start() API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that API hal_pwm_start will return error if the
 *         input channel is invalid.
 */
LOCAL int it_hal_pwm_004(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((1 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (1 != argc || NULL == argv)"
						                                               " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = (hal_pwm_channel_t)strtol(argv[0], NULL, 16);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing PWM channels and check error.*/
	hal_pwm_start(channel);

	/**! Check assert error */
	qc_assert(qc_assert_status());

	/**! Do judgment */
	qc_report();

	/**! Return 0*/
	return 0;
}

/**
 * @brief to test hal_pwm_stop() API in abnormal case
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that API hal_pwm_stop will return error if the input
 *          channel is invalid.
 */
LOCAL int it_hal_pwm_005(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((1 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (1 != argc || NULL == argv)"
						                                               " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = (hal_pwm_channel_t)strtol(argv[0], NULL, 16);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing PWM channels and check error.*/
	hal_pwm_stop(channel);

	/**! Check assert error */
	qc_assert(qc_assert_status());

	/**! Do judgment */
	qc_report();

	/**! Return 0*/
	return 0;
}

/**
 * @brief to test hal_pwm_update() API in abnormal case
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details To verify that API hal_pwm_update will return error if the input
 *          channel is configure with one invalid parameter.
 */
LOCAL int it_hal_pwm_006(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((10 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (10 != argc || NULL == argv)"
						                                               " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = (hal_pwm_channel_t)strtol(argv[0], NULL, 16);

	/**! PWM profile */
	hal_pwm_output_t pwm_profile = PWM_PROFILE_DEFAULT;

	/**! Get profile */
	pwm_profile.freq                       = strtol(argv[1], NULL, 10);
	pwm_profile.pattern.pair[0].duty       = strtol(argv[2], NULL, 10);
	pwm_profile.pattern.pair[1].duty       = strtol(argv[3], NULL, 10);
	pwm_profile.pattern.pair[2].duty       = strtol(argv[4], NULL, 10);
	pwm_profile.pattern.pair[0].num_pulses = strtol(argv[5], NULL, 10);
	pwm_profile.pattern.pair[1].num_pulses = strtol(argv[6], NULL, 10);
	pwm_profile.pattern.pair[2].num_pulses = strtol(argv[7], NULL, 10);
	pwm_profile.pattern.num_pairs          = strtol(argv[8], NULL, 10);
	pwm_profile.num_patterns               = strtol(argv[9], NULL, 10);

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initializing PWM channels and check error.*/
	hal_pwm_update(channel, &pwm_profile);

	/**! Check assert error */
	qc_assert(qc_assert_status());

	/**! Do judgment */
	qc_report();

	/**! Return 0*/
	return 0;
}

/**
 * @brief to test hal_pwm_init() and hal_pwm_start() API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that any valid pwm channel is initialized successfully
 *         via API hal_pwm_init(), start via hal_pwm_start() in case IRQ
 *         is enabled.
 */
LOCAL int it_hal_pwm_007(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((12 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (12 != argc || NULL == argv)"
						                                               " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = PWM_CHANNEL_DEFAULT;
	/**! PWM mode */
	hal_pwm_init_t    pwm_cfg = PWM_CFG_DEFAULT;
	/**! PWM profile */
	hal_pwm_output_t  pwm_profile = PWM_PROFILE_DEFAULT;

	/**! Reset PWM channels to default state */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initialize PWM to default */
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
		/**! Disable IRQ */
		hal_pwm_disable_irq(channel);
	}

	/**! PWM IRQ profile */
	hal_pwm_irq_t pwm_irq_profile = PWM_IRQ_PROFILE_DEFAULT;

	/**! Get configure mode */
	it_hal_pwm_config(&pwm_cfg, &pwm_profile, argv, argc);

	/**! Get PWM IRQ profile */
	if(strtol(argv[11], NULL, 10))
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ONE_PATTERN_GENERATED;
	}
	else
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ALL_PATTERNS_GENERATED;
	}

	pwm_irq_profile.callback_handler = it_hal_pwm_callback;

	/**! Reset all test point to default */
	qc_assert_reset();

	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initializing PWM channels for all.*/
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
	}

	/**! Enable PWM IRQ for all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Enable IRQ */
		hal_pwm_enable_irq(channel, &pwm_irq_profile);
		/**! Reset PWM IRQ callback */
		it_hal_pwm_reset_callback(channel);
	}

	/**! Start all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Start PWM channel */
		hal_pwm_start(channel);
	}

	/**! Delay a period of time to do measurement */
	delay_ms(1000);

	/**! Verify callback of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		if ((PWM_CONTINUOUS == pwm_cfg.output_gen_mode)
		 && (PWM_IRQ_ALL_PATTERNS_GENERATED == pwm_irq_profile.irq_mode))
		{
			/**! Verify PWM IRQ */
			qc_assert(!it_hal_pwm_callback_counter[channel]);
		}
		else
		{
			/**! Verify PWM IRQ */
			qc_assert(it_hal_pwm_callback_counter[channel]);
		}
	}

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief to test hal_pwm_stop API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @detail To verify that any valid pwm channel is initialized successfully
 *         via API hal_pwm_init(), start via hal_pwm_start() and stop after
 *         calling hal_pwm_stop() in case IRQ is enabled
 */
LOCAL int it_hal_pwm_008(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((12 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (12 != argc || NULL == argv)"
						                                               " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = PWM_CHANNEL_DEFAULT;
	/**! PWM mode */
	hal_pwm_init_t    pwm_cfg = PWM_CFG_DEFAULT;
	/**! PWM profile */
	hal_pwm_output_t  pwm_profile = PWM_PROFILE_DEFAULT;

	/**! Reset PWM channels to default state */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initialize PWM to default */
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
		/**! Disable IRQ */
		hal_pwm_disable_irq(channel);
	}

	/**! PWM IRQ profile */
	hal_pwm_irq_t pwm_irq_profile = PWM_IRQ_PROFILE_DEFAULT;

	/**! Get configure mode */
	it_hal_pwm_config(&pwm_cfg, &pwm_profile, argv, argc);

	/**! Get PWM IRQ profile */
	if(strtol(argv[11], NULL, 10))
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ONE_PATTERN_GENERATED;
	}
	else
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ALL_PATTERNS_GENERATED;
	}
	pwm_irq_profile.callback_handler = it_hal_pwm_callback;

	/**! Reset all test point to default */
	qc_assert_reset();

	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initializing PWM channels for all.*/
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
	}

	/**! Enable PWM IRQ for all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Enable IRQ */
		hal_pwm_enable_irq(channel, &pwm_irq_profile);
	}

	/**! Reset callback counters */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Reset PWM IRQ callback */
		it_hal_pwm_reset_callback(channel);
	}

	/**! Start all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Start PWM channel */
		hal_pwm_start(channel);
	}

	/**! Delay a period of time to do measurement */
	delay_ms(1000);

	/**! Verify callback of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		if ((PWM_CONTINUOUS != pwm_cfg.output_gen_mode)
		 && (PWM_IRQ_ALL_PATTERNS_GENERATED != pwm_irq_profile.irq_mode))
		{
			/**! Verify PWM IRQ */
			qc_assert(it_hal_pwm_callback_counter[channel]);
		}
		else
		{
			/**! Verify PWM IRQ */
			qc_assert(!it_hal_pwm_callback_counter[channel]);
		}
	}

	/**! Invoke PWM stop to stop PWM channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Stop channel */
		hal_pwm_stop(channel);
	}

	/**! Reset callback info */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Reset callback counter of channel */
		it_hal_pwm_reset_callback(channel);
	}

	/**! Delay a period of time to do measurement make sure PWM signal
	 * is stopped */
	delay_ms(1000);

	/**! Verify PWM IRQ of channels was stopped */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Verify PWM IRQ of channel was stopped */
		qc_assert(!it_hal_pwm_callback_counter[channel]);
	}

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief to test hal_pwm_stop and hal_pwm_disable_irq() APIs
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details
 *          - To verify that any valid pwm channel is initialized
 *          successfully via API hal_pwm_init(), start via
 *          hal_pwm_start() and stop after calling hal_pwm_stop()
 *          in case IRQ is enables
 *          - IRQ of indicated channel is disable in next sequence
 *          after calling hal_pwm_disable_irq().
 */
LOCAL int it_hal_pwm_009(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((12 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (12 != argc || NULL == argv)"
						                                               " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = PWM_CHANNEL_DEFAULT;
	/**! PWM mode */
	hal_pwm_init_t    pwm_cfg = PWM_CFG_DEFAULT;
	/**! PWM profile */
	hal_pwm_output_t  pwm_profile = PWM_PROFILE_DEFAULT;

	/**! Reset PWM channels to default state */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initialize PWM to default */
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
		/**! Disable IRQ */
		hal_pwm_disable_irq(channel);
	}

	/**! PWM IRQ profile */
	hal_pwm_irq_t pwm_irq_profile = PWM_IRQ_PROFILE_DEFAULT;

	/**! Get configure mode */
	it_hal_pwm_config(&pwm_cfg, &pwm_profile, argv, argc);

	/**! Get PWM IRQ profile */
	if(strtol(argv[11], NULL, 10))
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ONE_PATTERN_GENERATED;
	}
	else
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ALL_PATTERNS_GENERATED;
	}
	pwm_irq_profile.callback_handler = it_hal_pwm_callback;

	/**! Reset all test point to default */
	qc_assert_reset();

	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initializing PWM channels and check error.*/
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
	}

	/**! Enable PWM IRQ for all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Enable IRQ */
		hal_pwm_enable_irq(channel, &pwm_irq_profile);
	}

	/**! Reset callback counters */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Reset PWM IRQ callback */
		it_hal_pwm_reset_callback(channel);
	}

	/**! Start all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Start PWM channel */
		hal_pwm_start(channel);
	}

	/**! Delay a period of time to do measurement */
	delay_ms(1000);

	/**! Verify callback of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		if ((PWM_CONTINUOUS != pwm_cfg.output_gen_mode)
		 && (PWM_IRQ_ALL_PATTERNS_GENERATED != pwm_irq_profile.irq_mode))
		{
			/**! Verify PWM IRQ */
			qc_assert(it_hal_pwm_callback_counter[channel]);
		}
		else
		{
			/**! Verify PWM IRQ */
			qc_assert(!it_hal_pwm_callback_counter[channel]);
		}
	}

	/**! Invoke PWM stop to stop PWM channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Stop channel */
		hal_pwm_stop(channel);
	}

	/**! Reset callback info */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Reset callback counter of channel */
		it_hal_pwm_reset_callback(channel);
	}

	/**! Delay a period of time to do measurement make sure PWM signal
	 * is stopped */
	delay_ms(1000);

	/**! Verify callback of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Verify PWM IRQ */
		qc_assert(!it_hal_pwm_callback_counter[channel]);
	}

	/**! Invoke hal_pwm_disable_irq() of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Disable IRQ */
		hal_pwm_disable_irq(channel);
	}

	/**! re-start all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Start PWM channel */
		hal_pwm_start(channel);
	}

	/**! Invoke PWM stop to stop PWM channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Stop channel */
		hal_pwm_stop(channel);
	}

	/**! Verify PWM IRQ counter of channels is zero */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Verify PWM IRQ of channel was stopped */
		qc_assert(!it_hal_pwm_callback_counter[channel]);
	}

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief to test hal_pwm_update() API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 * @details
 *           - To verify that any valid pwm channel is initialized successfully
 *           via API hal_pwm_init(), start via hal_pwm_start(). PWM profile will
 *           be updated after invoking API hal_pwm_update()
 */
LOCAL int it_hal_pwm_010(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((12 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (12 != argc || NULL == argv)"
						                                               " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = PWM_CHANNEL_DEFAULT;
	/**! PWM mode */
	hal_pwm_init_t    pwm_cfg = PWM_CFG_DEFAULT;
	/**! PWM profile */
	hal_pwm_output_t  pwm_profile = PWM_PROFILE_DEFAULT;
	hal_pwm_output_t  pwm_desire_profile = PWM_PROFILE_DEFAULT;
	/**! Callback temporary counter buffer */
	uint32_t pwm_cb_cnt[2][PWM_TOTAL_CHANNELS];

	/**! Reset PWM channels to default state */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initialize PWM to default */
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
		/**! Disable IRQ */
		hal_pwm_disable_irq(channel);
		/**! Reset all pwm_cb_cnt */
		pwm_cb_cnt[0][channel] = 0;
		pwm_cb_cnt[1][channel] = 0;
	}

	/**! PWM IRQ profile */
	hal_pwm_irq_t pwm_irq_profile = PWM_IRQ_PROFILE_DEFAULT;

	/**! Get configure mode */
	it_hal_pwm_config(&pwm_cfg, &pwm_profile, argv, argc);
	/**! Copy PWM profile to desire profile */
	memcpy(&pwm_desire_profile, &pwm_profile, sizeof(hal_pwm_output_t));
	/**! Configure pwm desire profile */
	pwm_desire_profile.freq = pwm_desire_profile.freq << 1;
	pwm_desire_profile.num_patterns = pwm_desire_profile.num_patterns << 1;
	if (pwm_desire_profile.pattern.num_pairs == PWM_PAIRS_MAX_PER_CHANNEL)
		pwm_desire_profile.pattern.num_pairs -= 1;
	else
		pwm_desire_profile.pattern.num_pairs += 1;
	/**! Duty 0 */
	if (pwm_desire_profile.pattern.pair[0].duty)
		pwm_desire_profile.pattern.pair[0].duty >>= 1;
	else
		pwm_desire_profile.pattern.pair[0].duty = 25;
	/**! Duty 1 */
	if (pwm_desire_profile.pattern.pair[1].duty)
		pwm_desire_profile.pattern.pair[1].duty >>= 1;
	else
		pwm_desire_profile.pattern.pair[1].duty = 50;
	/**! Duty 2 */
	if (pwm_desire_profile.pattern.pair[2].duty)
		pwm_desire_profile.pattern.pair[2].duty >>= 1;
	else
		pwm_desire_profile.pattern.pair[2].duty = 75;
	/**! Pulses num 0 */
	if (pwm_desire_profile.pattern.pair[0].num_pulses)
		pwm_desire_profile.pattern.pair[0].num_pulses += 1;
	else
		pwm_desire_profile.pattern.pair[0].num_pulses = 2;
	/**! Pulses num 1 */
	if (pwm_desire_profile.pattern.pair[1].num_pulses)
		pwm_desire_profile.pattern.pair[1].num_pulses += 1;
	else
		pwm_desire_profile.pattern.pair[1].num_pulses = 3;
	/**! Pulses num 2 */
	if (pwm_desire_profile.pattern.pair[2].num_pulses)
		pwm_desire_profile.pattern.pair[2].num_pulses += 1;
	else
		pwm_desire_profile.pattern.pair[2].num_pulses = 4;

	/**! Get PWM IRQ profile */
	if(strtol(argv[11], NULL, 10))
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ONE_PATTERN_GENERATED;
	}
	else
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ALL_PATTERNS_GENERATED;
	}
	pwm_irq_profile.callback_handler = it_hal_pwm_callback;

	/**! Reset all test point to default */
	qc_assert_reset();

	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Initializing PWM channels for all.*/
		hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
	}

	//! TODO:
	MEM32_DUMP(hal_pwm_init(), 0x02019000, 12);

	/**! Enable PWM IRQ for all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Enable IRQ */
		hal_pwm_enable_irq(channel, &pwm_irq_profile);
	}

	//! TODO:
	MEM32_DUMP(hal_pwm_enable_irq(), 0x02019000, 12);

	/**! Reset callback counters */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Reset PWM IRQ callback */
		it_hal_pwm_reset_callback(channel);
	}

	/**! Start all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Start PWM channel */
		hal_pwm_start(channel);
	}

	//! TODO:
	MEM32_DUMP(hal_pwm_start(), 0x02019000, 12);

	/**! Delay a period of time to do measurement */
	delay_ms(1000);

	/**! Verify callback of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		if ((PWM_CONTINUOUS == pwm_cfg.output_gen_mode)
		 && (PWM_IRQ_ALL_PATTERNS_GENERATED == pwm_irq_profile.irq_mode))
		{
			/**! Verify PWM IRQ */
			qc_assert(0 == it_hal_pwm_callback_counter[channel]);
		}
		else
		{
			/**! Verify PWM IRQ */
			qc_assert(0 != it_hal_pwm_callback_counter[channel]);
		}
	}

	/**! Invoke PWM stop to stop PWM channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Stop channel */
		hal_pwm_stop(channel);
	}

	//! TODO:
	MEM32_DUMP(hal_pwm_stop(), 0x02019000, 12);

	delay_ms(1000);
	/**! Save pwm callback counter of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Save pwm callback counter of channel */
		pwm_cb_cnt[0][channel] = it_hal_pwm_callback_counter[channel];
	}

	/**! Reset callback info */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Reset PWM IRQ callback */
		it_hal_pwm_reset_callback(channel);
	}

	/**! Invoke hal_pwm_update() to update PWM profile of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Reset PWM IRQ callback */
		hal_pwm_update(channel, &pwm_desire_profile);
	}

	//! TODO:
	MEM32_DUMP(hal_pwm_update(), 0x02019000, 12);

	/**! Start all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Start PWM channel */
		hal_pwm_start(channel);
	}

	//! TODO:
	MEM32_DUMP(hal_pwm_start(), 0x02019000, 12);

	/**! Delay a period of time to do measurement */
	delay_ms(1000);

	/**! Invoke PWM stop to stop PWM channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Stop channel */
		hal_pwm_stop(channel);
	}

	//! TODO:
	MEM32_DUMP(hal_pwm_stop(), 0x02019000, 12);

	/**! Verify callback of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		if ((PWM_CONTINUOUS == pwm_cfg.output_gen_mode)
		 && (PWM_IRQ_ALL_PATTERNS_GENERATED == pwm_irq_profile.irq_mode))
		{
			/**! Verify PWM IRQ */
			qc_assert(0 == it_hal_pwm_callback_counter[channel]);
		}
		else
		{
			/**! Verify PWM IRQ */
			qc_assert(0 != it_hal_pwm_callback_counter[channel]);
		}
	}

	/**! Save pwm callback counter of all channels */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		/**! Save pwm callback counter of channel */
		pwm_cb_cnt[1][channel] = it_hal_pwm_callback_counter[channel];
	}

	/**! Compare two callback counter */
	for (channel = PWM0_CH1; channel <= PWM1_CH6; channel++)
	{
		if ((PWM_CONTINUOUS == pwm_cfg.output_gen_mode)
		 && (PWM_IRQ_ALL_PATTERNS_GENERATED == pwm_irq_profile.irq_mode))
		{
			/**! Compare two callback of channel */
			qc_assert(pwm_cb_cnt[0][channel] == pwm_cb_cnt[1][channel]);
		}
		else
		{
			/**! Compare two callback of channel */
			qc_assert(pwm_cb_cnt[0][channel] != pwm_cb_cnt[1][channel]);
		}
	}

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief to test performance API
 * @param[in] argv	: arguments list
 * @param[in] argc	: argument counter
 * @param[out] NA
 * @return 	0	: success
 *          Others: failed
 */
LOCAL int it_hal_pwm_011(char** argv, int argc)
{
	/**! Verify input parameters */
	if ((13 != argc) || (NULL == argv))
	{
		console_putstring("Error: invalid parameter (12 != argc || NULL == argv)"
						                                               " \r\n");
		return -1;
	}

	/**! Local variable as channel */
	hal_pwm_channel_t channel = PWM_CHANNEL_DEFAULT;
	/**! PWM mode */
	hal_pwm_init_t    pwm_cfg = PWM_CFG_DEFAULT;
	/**! PWM profile */
	hal_pwm_output_t  pwm_profile = PWM_PROFILE_DEFAULT;
	hal_pwm_output_t  pwm_desire_profile = PWM_PROFILE_DEFAULT;

	/**! Callback temporary counter buffer */
	uint32_t pwm_cb_cnt[2];
	hal_gpio_t gpio = {0, 0, 1};
	uint8_t select = 0;

	/**! Initialize PWM to default */
	hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
	/**! Disable IRQ */
	hal_pwm_disable_irq(channel);
	/**! Reset all pwm_cb_cnt */
	pwm_cb_cnt[0] = 0;
	pwm_cb_cnt[1] = 0;
	/**! Initialize GPIO */
	hal_gpio_init(&gpio);

	/**! PWM IRQ profile */
	hal_pwm_irq_t pwm_irq_profile = PWM_IRQ_PROFILE_DEFAULT;

	/**! Get configure mode */
	it_hal_pwm_config(&pwm_cfg, &pwm_profile, argv, argc);

	/**! Get select API */
	select = (uint8_t)strtol(argv[12], NULL, 10);

	/**! Copy PWM profile to desire profile */
	memcpy(&pwm_desire_profile, &pwm_profile, sizeof(hal_pwm_output_t));
	/**! Configure pwm desire profile */
	pwm_desire_profile.freq = pwm_desire_profile.freq << 1;
	pwm_desire_profile.num_patterns = pwm_desire_profile.num_patterns << 1;
	if (pwm_desire_profile.pattern.num_pairs == PWM_PAIRS_MAX_PER_CHANNEL)
		pwm_desire_profile.pattern.num_pairs -= 1;
	else
		pwm_desire_profile.pattern.num_pairs += 1;
	/**! Duty 0 */
	if (pwm_desire_profile.pattern.pair[0].duty)
		pwm_desire_profile.pattern.pair[0].duty >>= 1;
	else
		pwm_desire_profile.pattern.pair[0].duty = 25;
	/**! Duty 1 */
	if (pwm_desire_profile.pattern.pair[1].duty)
		pwm_desire_profile.pattern.pair[1].duty >>= 1;
	else
		pwm_desire_profile.pattern.pair[1].duty = 50;
	/**! Duty 2 */
	if (pwm_desire_profile.pattern.pair[2].duty)
		pwm_desire_profile.pattern.pair[2].duty >>= 1;
	else
		pwm_desire_profile.pattern.pair[2].duty = 75;
	/**! Pulses num 0 */
	if (pwm_desire_profile.pattern.pair[0].num_pulses)
		pwm_desire_profile.pattern.pair[0].num_pulses += 1;
	else
		pwm_desire_profile.pattern.pair[0].num_pulses = 2;
	/**! Pulses num 1 */
	if (pwm_desire_profile.pattern.pair[1].num_pulses)
		pwm_desire_profile.pattern.pair[1].num_pulses += 1;
	else
		pwm_desire_profile.pattern.pair[1].num_pulses = 3;
	/**! Pulses num 2 */
	if (pwm_desire_profile.pattern.pair[2].num_pulses)
		pwm_desire_profile.pattern.pair[2].num_pulses += 1;
	else
		pwm_desire_profile.pattern.pair[2].num_pulses = 4;

	/**! Get PWM IRQ profile */
	if(strtol(argv[11], NULL, 10))
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ONE_PATTERN_GENERATED;
	}
	else
	{
		pwm_irq_profile.irq_mode = PWM_IRQ_ALL_PATTERNS_GENERATED;
	}
	pwm_irq_profile.callback_handler = it_hal_pwm_callback;

	/**! Reset all test point to default */
	qc_assert_reset();

	/**! Initialize GPIO */
	hal_gpio_init(&gpio);

	if(0 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Initializing PWM channels for all.*/
	hal_pwm_init(channel, &pwm_cfg, &pwm_profile);
	if(0 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(1 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Enable IRQ */
	hal_pwm_enable_irq(channel, &pwm_irq_profile);
	if(1 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	/**! Reset PWM IRQ callback */
	it_hal_pwm_reset_callback(channel);

	if(2 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Start PWM channel */
	hal_pwm_start(channel);
	if(2 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Delay a period of time to do measurement */
	delay_ms(1000);

	/**! Verify callback of all channels */

	if ((PWM_CONTINUOUS == pwm_cfg.output_gen_mode)
	 && (PWM_IRQ_ALL_PATTERNS_GENERATED == pwm_irq_profile.irq_mode))
	{
		/**! Verify PWM IRQ */
		qc_assert(0 == it_hal_pwm_callback_counter[channel]);
	}
	else
	{
		/**! Verify PWM IRQ */
		qc_assert(0 != it_hal_pwm_callback_counter[channel]);
	}

	if(3 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Stop channel */
	hal_pwm_stop(channel);
	if(3 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	delay_ms(1000);

	/**! Save pwm callback counter of channel */
	pwm_cb_cnt[0] = it_hal_pwm_callback_counter[channel];

	/**! Reset PWM IRQ callback */
	it_hal_pwm_reset_callback(channel);

	if(4 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Invoke hal_pwm_update() to update PWM profile of  channels */
	hal_pwm_update(channel, &pwm_desire_profile);
	if(4 == select)
	{
		hal_gpio_set_low(&gpio);
	}

	if(5 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Start PWM channel */
	hal_pwm_start(channel);
	if(5 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Delay a period of time to do measurement */
	delay_ms(1000);
	if(6 == select)
	{
		hal_gpio_set_high(&gpio);
	}
	/**! Invoke PWM stop to stop PWM channels */
	hal_pwm_stop(channel);
	if(6 == select)
	{
		hal_gpio_set_low(&gpio);
	}
	/**! Verify callback of channels */
	if ((PWM_CONTINUOUS == pwm_cfg.output_gen_mode)
	 && (PWM_IRQ_ALL_PATTERNS_GENERATED == pwm_irq_profile.irq_mode))
	{
		/**! Verify PWM IRQ */
		qc_assert(0 == it_hal_pwm_callback_counter[channel]);
	}
	else
	{
		/**! Verify PWM IRQ */
		qc_assert(0 != it_hal_pwm_callback_counter[channel]);
	}

	/**! Save pwm callback counter of channel */
	pwm_cb_cnt[1] = it_hal_pwm_callback_counter[channel];

	/**! Compare two callback counter */
	if ((PWM_CONTINUOUS == pwm_cfg.output_gen_mode)
	 && (PWM_IRQ_ALL_PATTERNS_GENERATED == pwm_irq_profile.irq_mode))
	{
		/**! Compare two callback of channel */
		qc_assert(pwm_cb_cnt[0] == pwm_cb_cnt[1]);
	}
	else
	{
		/**! Compare two callback of channel */
		qc_assert(pwm_cb_cnt[0] != pwm_cb_cnt[1]);
	}

	/**! Do judgment */
	qc_report();

	return 0;
}

/**
 * @brief to convert configuration parameters from user input
 * @param[in] argv		: user input parameters as list of strings
 * @param[in] argc		: user input parameter counter
 * @param[out] pwm_cfg	: configuration modes
 * @param[out] pwm_profile	: desire pwm profile
 * @return 	NA
 * @detail to get configuration parameters from user input
 */
LOCAL void it_hal_pwm_config(hal_pwm_init_t* pwm_cfg, \
		                   hal_pwm_output_t* pwm_profile, char** argv, int argc)
{
	/**! Get configure mode */
	pwm_cfg->align_mode      = (pwm_align_t)strtol(argv[0], NULL, 16);
	pwm_cfg->output_gen_mode =  (pwm_mode_t)strtol(argv[1], NULL, 16);

	/**! Get profile */
	pwm_profile->freq                       = strtol(argv[2], NULL, 10);
	pwm_profile->pattern.pair[0].duty       = strtol(argv[3], NULL, 10);
	pwm_profile->pattern.pair[1].duty       = strtol(argv[4], NULL, 10);
	pwm_profile->pattern.pair[2].duty       = strtol(argv[5], NULL, 10);
	pwm_profile->pattern.pair[0].num_pulses = strtol(argv[6], NULL, 10);
	pwm_profile->pattern.pair[1].num_pulses = strtol(argv[7], NULL, 10);
	pwm_profile->pattern.pair[2].num_pulses = strtol(argv[8], NULL, 10);
	pwm_profile->pattern.num_pairs          = strtol(argv[9], NULL, 10);
	pwm_profile->num_patterns               = strtol(argv[10], NULL, 10);
}

/**
 * @brief to be set as callback function of PWM
 * @param[in] NA
 * @param[out] NA
 * @return 	NA
 * @detail to be set as callback function of PWM
 */
LOCAL void it_hal_pwm_callback(hal_pwm_channel_t channel)
{
	it_hal_pwm_callback_counter[channel]++;
}

/**
 * @brief to reset callback counter of PWM
 * @param[in] NA
 * @param[out] NA
 * @return 	NA
 * @detail to reset callback counter of PWM
 */
LOCAL void it_hal_pwm_reset_callback(hal_pwm_channel_t channel)
{
	it_hal_pwm_callback_counter[channel] = 0;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/

