/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_gpio.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Feb-17-2016
 * @brief   This file contains definitions of the Digital Input/Output driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "cortex_r4.h"

/* Exported typedef ----------------------------------------------------------*/
/*
 * @brief hal_gpio_port_t
 *
 * GPIO port name
 */
typedef enum hal_gpio_port
{
	GPIO_PORTA = 0,
	GPIO_PORTB,
	GPIO_PORTC,
	GPIO_PORT_MAX_IDX
} hal_gpio_port_t;

/*
 * @brief hal_gpio_pin_t
 *
 * GPIO pin name
 */
typedef enum hal_gpio_pin
{
	GPIO_PIN_0 = 0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15,
	GPIO_PIN_MAX_IDX
} hal_gpio_pin_t;

/*
 * @brief hal_gpio_dir_t
 *
 * GPIO direction
 */
typedef enum hal_gpio_dir
{
	GPIO_DIR_IN = 0,
	GPIO_DIR_OUT
} hal_gpio_dir_t;

/*
 * @brief hal_gpio_level_t
 *
 * GPIO logic level
 */
typedef enum hal_gpio_level
{
	GPIO_LEVEL_LOW = 0,
	GPIO_LEVEL_HIGH
} hal_gpio_level_t;

/*
 * @brief hal_gpio_exti_type_t
 *
 * GPIO external interrupt type
 */
typedef enum hal_gpio_exti_type
{
	GPIO_EXTI_FALLING_EDGE = 0,
	GPIO_EXTI_RISING_EDGE,
} hal_gpio_exti_type_t;

/*
 * @brief hal_gpio_t
 *
 * GPIO driver structure
 */
typedef struct hal_gpio
{
	hal_gpio_port_t		port;		/* port name */
	hal_gpio_pin_t		pin;		/* pin name */
	hal_gpio_dir_t		direction;	/* pin direction */
} hal_gpio_t;

/*
 * @brief hal_gpio_exti_t
 *
 * GPIO external interrupt structure
 */
typedef struct hal_gpio_exti
{
	hal_gpio_pin_t			pin;		/* pin name on port A */
	hal_gpio_exti_type_t	int_type;	/* interrupt type */
	/* pointer to function handled in irq when external interrupt occurs */
	void					(*irq_handler)(void);
} hal_gpio_exti_t;

/* Exported macro ------------------------------------------------------------*/
#define GPIO_PORT_PIN(port, pin)	((port)<<4 | (pin))
#define GPIO_PORT(port_pin)			(hal_gpio_port_t)(((port_pin) & 0xF0) >> 4)
#define GPIO_PIN(port_pin)			(hal_gpio_pin_t)(((port_pin) & 0x0F) >> 0)

/* Exported functions ------------------------------------------------------- */

/*
 * @brief hal_gpio_init
 * Initialize independent GPIO channel
 * @param gpio: pointer to hal_gpio_t structure
 * @return: none
 */
void hal_gpio_init(hal_gpio_t *gpio);

/*
 * @brief hal_gpio_set_low
 * Set low logic level on a GPIO pin
 * @param gpio: pointer to hal_gpio_t structure
 * @return: none
 */
void hal_gpio_set_low(hal_gpio_t *gpio);

/**
 * @brief hal_gpio_set_high
 * Set high logic level on a GPIO pin
 * @param gpio: pointer to hal_gpio_t structure
 * @return: none
 */
void hal_gpio_set_high(hal_gpio_t *gpio);

/**
 * @brief hal_gpio_toggle
 * Toggle logic level on a GPIO pin
 * @param gpio: pointer to hal_gpio_t structure
 * @return: none
 */
void hal_gpio_toggle(hal_gpio_t *gpio);

/*
 * @brief hal_gpio_read_output
 * Read state on a GPIO pin
 * @param gpio: pointer to hal_gpio_t structure
 * @return: the input data if configured as input,
 *          the output data if configured as output
 */
hal_gpio_level_t hal_gpio_read(hal_gpio_t *gpio);

/*
 * @brief hal_gpio_enable_exti
 * Enable external interrupt on a pin of PORTA
 * @param exti: pointer to hal_gpio_exti_t structure
 * @return: none
 */
void hal_gpio_enable_exti(hal_gpio_exti_t *exti);

/*
 * @brief hal_gpio_disable_exti
 * Disable external interrupt on a pin of PORTA
 * @param exti: pointer to hal_gpio_exti_t structure
 * @return: none
 */
void hal_gpio_disable_exti(hal_gpio_exti_t *exti);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_GPIO_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
