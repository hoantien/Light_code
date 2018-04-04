/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    hal_gpio.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Feb-17-2016
 * @brief   This file contains expand of hal_gpio
 *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "hal_gpio.h"
#include "cortex_r4.h"
#include "assert.h"
#include "hal_vic.h"

/* Private define ------------------------------------------------------------*/

/* GPIO handle */
#define GPIO							((hal_gpio_reg_t *)GPIO_BASE)

/* Definition of bits in Configuration Register 1 */
#define GPIO_CFG1_BOTHEDGE_MASK			0x00200000
#define GPIO_CFG1_INTERRUPT_MASK		0x00001000
#define GPIO_CFG1_NUMPORTS_MASK			0x0000000C

/* Definition of bits in Configuration Register 2 */
#define GPIO_CFG2_PWIDTHA_MASK			0x0000001F
#define GPIO_CFG2_PWIDTHB_MASK			0x000003E0
#define GPIO_CFG2_PWIDTHC_MASK			0x00007C00
#define GPIO_CFG2_PWIDTHD_MASK			0x000F8000

/* Used for asserting GPIO Pin */
#define IS_GPIO_PORT_PIN(PORT, PIN)		(((GPIO_PORTC > PORT) && \
											((GPIO_PIN_0 <= PIN) && \
											(GPIO_PIN_10 > PIN))) || \
										((GPIO_PORTC == PORT) && \
											(GPIO_PIN_0 <= PIN) && \
											(GPIO_PIN_MAX_IDX > PIN)))

/* Used for asserting GPIO Direction */
#define IS_GPIO_DIR(DIR)				((GPIO_DIR_IN == DIR) || \
										(GPIO_DIR_OUT == DIR))

/* Used for asserting external interrupt type */
#define IS_EXTI_TYPE(TYPE)				((GPIO_EXTI_FALLING_EDGE == TYPE) || \
										(GPIO_EXTI_RISING_EDGE == TYPE))

/* Private typedef -----------------------------------------------------------*/

/*
 * @brief hal_gpio_reg_t
 *
 * GPIO registers
 */
typedef struct
{
	__IO uint32_t DRA;			/* Port A data register */
	__IO uint32_t DDRA;			/* Port A data direction register */
	__IO uint32_t CTLA;			/* Port A data source register */
	__IO uint32_t DRB;			/* Port B data register */
	__IO uint32_t DDRB;			/* Port B data direction register */
	__IO uint32_t CTLB;			/* Port B data source register */
	__IO uint32_t DRC;			/* Port C data register */
	__IO uint32_t DDRC;			/* Port C data direction register */
	__IO uint32_t CTLC;			/* Port C data source register */
	__IO uint32_t DRD;			/* Port D data register */
	__IO uint32_t DDRD;			/* Port D data direction register */
	__IO uint32_t CTLD;			/* Port D data source register */
	__IO uint32_t INTEN;		/* Port A interrupt enable register */
	__IO uint32_t INTMASK;		/* Port A interrupt mask register */
	__IO uint32_t INTTYPLEVL;	/* Port A interrupt level register */
	__IO uint32_t INTPOL;		/* Port A interrupt polarity register */
	__I  uint32_t INTSTAT;		/* Port A interrupt status register */
	__I  uint32_t RAWINTSTAT;	/* Port A raw interrupt status register */
	__IO uint32_t DEBOUNCE;		/* Port A debounce enable register */
	__O  uint32_t EOI;			/* Port A clear interrupt register */
	__I  uint32_t EXTA;			/* Port A external port register */
	__I  uint32_t EXTB;			/* Port B external port register */
	__I  uint32_t EXTC;			/* Port C external port register */
	__I  uint32_t EXTD;			/* Port D external port register */
	__IO uint32_t LSSYNC;		/* Port A synchronization enable register*/
	__I  uint32_t IDCODE;		/* ID code register */
	__IO uint32_t INTBOTHEDGE;	/* Interrupt both edge type register */
	__I  uint32_t VERIDCODE;	/* Component version register */
	__I  uint32_t CFG2;			/* Configuration register 2 */
	__I  uint32_t CFG1;			/* Configuration register 1 */
} hal_gpio_reg_t;

/*
 * @brief exti_config_t
 *
 * External interrupt configuration
 */
typedef struct exti_config
{
	void		(*irq_handler)(void);
	uint8_t		enabled;
} exti_config_t;

/* Private variables -------------------------------------------------------- */

/* External interrupt handlers */
static exti_config_t exti_configs[GPIO_PIN_MAX_IDX] =
{
	{.irq_handler = NULL, .enabled = FALSE},	/* EXTI0 */
	{.irq_handler = NULL, .enabled = FALSE},	/* EXTI1 */
	{.irq_handler = NULL, .enabled = FALSE},	/* EXTI2 */
	{.irq_handler = NULL, .enabled = FALSE},	/* EXTI3 */
	{.irq_handler = NULL, .enabled = FALSE},	/* EXTI4 */
	{.irq_handler = NULL, .enabled = FALSE},	/* EXTI5 */
	{.irq_handler = NULL, .enabled = FALSE},	/* EXTI6 */
	{.irq_handler = NULL, .enabled = FALSE},	/* EXTI7 */
};

/* Private function prototypes -----------------------------------------------*/
static void EXTI_IRQHandler(void);

/* Exported functions --------------------------------------------------------*/

/*
 * hal_gpio_init
 * Initialize independent GPIO channel
 */
void hal_gpio_init(hal_gpio_t *gpio)
{
	hal_gpio_reg_t	*gpio_reg = GPIO;
	__IO uint32_t	*dir_reg;

	/* Assert parameters */
	assert_param(NULL_PTR != gpio);
	assert_param(IS_GPIO_PORT_PIN(gpio->port, gpio->pin));
	assert_param(IS_GPIO_DIR(gpio->direction));

	/* Get pointer to the direction register of the corresponding port */
	if (GPIO_PORTA == gpio->port)
		dir_reg = &(gpio_reg->DDRA);
	else if (GPIO_PORTB == gpio->port)
		dir_reg = &(gpio_reg->DDRB);
	else if (GPIO_PORTC == gpio->port)
		dir_reg = &(gpio_reg->DDRC);
	else
		dir_reg = &(gpio_reg->DDRD);

	/* Set direction */
	if(GPIO_DIR_IN == gpio->direction)
		*dir_reg &= (~(0x01 << gpio->pin));
	else
		*dir_reg |= (gpio->direction << gpio->pin);
}

/*
 * hal_gpio_set_low
 * Set low logic level on a GPIO pin
 */
void hal_gpio_set_low(hal_gpio_t *gpio)
{
	hal_gpio_reg_t	*gpio_reg = GPIO;

	/* Assert parameters */
	assert_param(NULL_PTR != gpio);
	assert_param(IS_GPIO_PORT_PIN(gpio->port, gpio->pin));
	assert_param(GPIO_DIR_OUT == gpio->direction);

	if(GPIO_PORTA == gpio->port)
		gpio_reg->DRA &= (~(0x01 << gpio->pin));
	else if(GPIO_PORTB == gpio->port)
		gpio_reg->DRB &= (~(0x01 << gpio->pin));
	else if(GPIO_PORTC == gpio->port)
		gpio_reg->DRC &= (~(0x01 << gpio->pin));
	else /* GPIO_PORTD */
		gpio_reg->DRD &= (~(0x01 << gpio->pin));
}

/*
 * hal_gpio_set_high
 * Set high logic level on a GPIO pin
 */
void hal_gpio_set_high(hal_gpio_t *gpio)
{
	hal_gpio_reg_t	*gpio_reg = GPIO;

	/* Assert parameters */
	assert_param(NULL_PTR != gpio);
	assert_param(IS_GPIO_PORT_PIN(gpio->port, gpio->pin));
	assert_param(GPIO_DIR_OUT == gpio->direction);

	if(GPIO_PORTA == gpio->port)
		gpio_reg->DRA |= (0x01 << gpio->pin);
	else if(GPIO_PORTB == gpio->port)
		gpio_reg->DRB |= (0x01 << gpio->pin);
	else if(GPIO_PORTC == gpio->port)
		gpio_reg->DRC |= (0x01 << gpio->pin);
	else /* GPIO_PORTD */
		gpio_reg->DRD |= (0x01 << gpio->pin);
}

/*
 * hal_gpio_toggle
 * Toggle logic level on a GPIO pin
 */
void hal_gpio_toggle(hal_gpio_t *gpio)
{
	hal_gpio_reg_t	*gpio_reg = GPIO;

	/* Assert parameters */
	assert_param(NULL_PTR != gpio);
	assert_param(IS_GPIO_PORT_PIN(gpio->port, gpio->pin));
	assert_param(GPIO_DIR_OUT == gpio->direction);

	if(GPIO_PORTA == gpio->port)
		gpio_reg->DRA ^= (0x01 << gpio->pin);
	else if(GPIO_PORTB == gpio->port)
		gpio_reg->DRB ^= (0x01 << gpio->pin);
	else if(GPIO_PORTC == gpio->port)
		gpio_reg->DRC ^= (0x01 << gpio->pin);
	else /* GPIO_PORTD */
		gpio_reg->DRD ^= (0x01 << gpio->pin);
}

/*
 * hal_gpio_read_input
 * Read state on a GPIO pin
 */
hal_gpio_level_t hal_gpio_read(hal_gpio_t *gpio)
{
	hal_gpio_reg_t		*gpio_reg = GPIO;
	hal_gpio_level_t	state;

	/* Assert parameters */
	assert_param(NULL_PTR != gpio);
	assert_param(IS_GPIO_PORT_PIN(gpio->port, gpio->pin));
	assert_param(IS_GPIO_DIR(gpio->direction));

	if(GPIO_PORTA == gpio->port)
	{
		if (GPIO_DIR_IN == gpio->direction)
			state = (gpio_reg->EXTA >> gpio->pin) & 0x01;
		else
			state = (gpio_reg->DRA >> gpio->pin) & 0x01;
	}
	else if(GPIO_PORTB == gpio->port)
	{
		if (GPIO_DIR_IN == gpio->direction)
			state = (gpio_reg->EXTB >> gpio->pin) & 0x01;
		else
			state = (gpio_reg->DRB >> gpio->pin) & 0x01;
	}
	else if(GPIO_PORTC == gpio->port)
	{
		if (GPIO_DIR_IN == gpio->direction)
			state = (gpio_reg->EXTC >> gpio->pin) & 0x01;
		else
			state = (gpio_reg->DRC >> gpio->pin) & 0x01;
	}
	else /* GPIO_PORTD */
	{
		if (GPIO_DIR_IN == gpio->direction)
			state = (gpio_reg->EXTD >> gpio->pin) & 0x01;
		else
			state = (gpio_reg->DRD >> gpio->pin) & 0x01;
	}

	return state;
}

/*
 * @brief hal_gpio_enable_exti
 * Enable external interrupt on a pin of PORTA
 */
void hal_gpio_enable_exti(hal_gpio_exti_t *exti)
{
	hal_gpio_reg_t	*gpio_reg = GPIO;
	uint32_t		pin_mask;

	/* Assert parameters */
	assert_param(NULL_PTR != exti);
	assert_param(IS_GPIO_PORT_PIN(GPIO_PORTA, exti->pin));
	assert_param(IS_EXTI_TYPE(exti->int_type));

	/* Get pin mask */
	pin_mask = 1 << exti->pin;

	/* Set external interrupt type */
	if (GPIO_EXTI_FALLING_EDGE == exti->int_type)
	{
		/* Select edge-sensitive */
		gpio_reg->INTTYPLEVL |= pin_mask;
		/* Select active-low polarity */
		gpio_reg->INTPOL &= (~pin_mask);
	}
	else /* GPIO_EXTI_RISING_EDGE */
	{
		/* Select edge-sensitive */
		gpio_reg->INTTYPLEVL |= pin_mask;
		/* Select active-high polarity */
		gpio_reg->INTPOL |= pin_mask;
	}
	/* Synchronize with pclk_intr */
	gpio_reg->LSSYNC |= pin_mask;
	/* Set interrupt handler */
	exti_configs[exti->pin].irq_handler = exti->irq_handler;
	exti_configs[exti->pin].enabled = TRUE;
	/* Register interrupt handler */
	vic_register_irq(GPIO_IRQn, EXTI_IRQHandler);
	/* Enable external interrupt */
	gpio_reg->INTEN |= pin_mask;
}

/*
 * @brief hal_gpio_disable_exti
 * Disable external interrupt on a pin of PORTA
 */
void hal_gpio_disable_exti(hal_gpio_exti_t *exti)
{
	hal_gpio_reg_t	*gpio_reg = GPIO;
	uint8_t			i;

	/* Assert parameters */
	assert_param(NULL_PTR != exti);
	assert_param(IS_GPIO_PORT_PIN(GPIO_PORTA, exti->pin));

	/* Disable external interrupt */
	gpio_reg->INTEN &= (~(0x01 << exti->pin));
	/* Clear interrupt handler */
	exti_configs[exti->pin].irq_handler = NULL_PTR;
	exti_configs[exti->pin].enabled = FALSE;
	/* Unregister interrupt handler */
	for (i = 0; i < GPIO_PIN_MAX_IDX; i++)
		if (TRUE == exti_configs[i].enabled)
			break;
	if (GPIO_PIN_MAX_IDX == i)
		vic_unregister_irq(GPIO_IRQn);
}

/*
 * @brief EXTI_IRQHandler
 * External interrupt internal handler
 */
static void EXTI_IRQHandler(void)
{
	hal_gpio_reg_t	*gpio_reg = GPIO;
	int				interrupt_pin = GPIO_PIN_MAX_IDX;
	int				i;

	/* Get the interrupt source */
	for(i = 0; i < GPIO_PIN_MAX_IDX; i++)
	{
		if(0x01 == ((gpio_reg->INTSTAT >> i) & 0x01))
		{
			interrupt_pin = i;
			break;
		}
	}

	/* Check again if the interrupt source is valid */
	if(GPIO_PIN_MAX_IDX > interrupt_pin)
	{
		/* Clear interrupt pending */
		gpio_reg->EOI |= (0x01 << interrupt_pin);

		if(NULL_PTR != exti_configs[interrupt_pin].irq_handler)
		{
			/* Call back function to upper module */
			exti_configs[interrupt_pin].irq_handler();
		}
	}
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
