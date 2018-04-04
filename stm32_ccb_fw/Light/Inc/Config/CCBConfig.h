/********************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CCB_CONFIG_H
#define __CCB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Global macro --------------------------------------------------------------*/
/* Setting color for terminal debugging */
#if(RELEASE > 0)
  #define NO_DEBUG
#endif

/* Setting color for terminal debugging */
#if(RELEASE == 0)
  #ifdef NO_DEBUG_COLOR
    #undef NO_DEBUG_COLOR
  #endif
#else
  #define NO_DEBUG_COLOR
#endif

#ifndef P1_CCB
#define STM_CCB
#endif

/* Hardware definitions ------------------------------------------------------*/
/* CCI driver definitions */
#define CCIn					2
#define CCI0_SLAVE_ADDRESS 		0x30
#define CCI1_SLAVE_ADDRESS 		0x30

/* CCI0 Communication boards Interface */

/* CCI0 Communication boards Interface */
#define CCI0_I2C				I2C1
#define CCI0_CLK				RCC_APB1Periph_I2C1
#define CCI0_EV_IRQn			I2C1_EV_IRQn
#define CCI0_ER_IRQn			I2C1_ER_IRQn
#define CCI0_EV_IRQHandler		I2C1_EV_IRQHandler
#define CCI0_ER_IRQHandler		I2C1_ER_IRQHandler

#define CCI0_SDA_GPIO_CLK		RCC_AHB1Periph_GPIOB
#define CCI0_SDA_PIN			GPIO_Pin_9
#define CCI0_SDA_GPIO_PORT		GPIOB
#define CCI0_SDA_SOURCE			GPIO_PinSource9
#define CCI0_SDA_AF				GPIO_AF_I2C1

#define CCI0_SCL_GPIO_CLK		RCC_AHB1Periph_GPIOB
#define CCI0_SCL_PIN			GPIO_Pin_8
#define CCI0_SCL_GPIO_PORT		GPIOB
#define CCI0_SCL_SOURCE			GPIO_PinSource8
#define CCI0_SCL_AF				GPIO_AF_I2C1

/* CCI1 Communication boards Interface */
#define CCI1_I2C				I2C2
#define CCI1_CLK				RCC_APB1Periph_I2C2
#define CCI1_EV_IRQn			I2C2_EV_IRQn
#define CCI1_ER_IRQn			I2C2_ER_IRQn
#define CCI1_EV_IRQHandler		I2C2_EV_IRQHandler
#define CCI1_ER_IRQHandler		I2C2_ER_IRQHandler

#define CCI1_SDA_GPIO_CLK		RCC_AHB1Periph_GPIOE
#define CCI1_SDA_PIN			GPIO_Pin_2
#define CCI1_SDA_GPIO_PORT		GPIOE
#define CCI1_SDA_SOURCE			GPIO_PinSource2
#define CCI1_SDA_AF				GPIO_AF_I2C2

#define CCI1_SCL_GPIO_CLK		RCC_AHB1Periph_GPIOF
#define CCI1_SCL_PIN			GPIO_Pin_1
#define CCI1_SCL_GPIO_PORT		GPIOF
#define CCI1_SCL_SOURCE			GPIO_PinSource1
#define CCI1_SCL_AF				GPIO_AF_I2C2

#define CCB_CCI0				I2C1
#define CCB_CCI0_CLK			RCC_APB1Periph_I2C1
#define CCB_CCI0_EV_IRQn		I2C1_EV_IRQn
#define CCB_CCI0_ER_IRQn		I2C1_ER_IRQn
#define CCB_CCI0_EV_IRQHandler	I2C1_EV_IRQHandler
#define CCB_CCI0_ER_IRQHandler	I2C1_ER_IRQHandler

#define CCB_CCI0_SDA_GPIO_CLK	RCC_AHB1Periph_GPIOB
#define CCB_CCI0_SDA_PIN		GPIO_Pin_9
#define CCB_CCI0_SDA_GPIO_PORT	GPIOB
#define CCB_CCI0_SDA_SOURCE		GPIO_PinSource9
#define CCB_CCI0_SDA_AF			GPIO_AF_I2C1

#define CCB_CCI0_SCL_GPIO_CLK	RCC_AHB1Periph_GPIOB
#define CCB_CCI0_SCL_PIN		GPIO_Pin_8
#define CCB_CCI0_SCL_GPIO_PORT	GPIOB
#define CCB_CCI0_SCL_SOURCE		GPIO_PinSource8
#define CCB_CCI0_SCL_AF			GPIO_AF_I2C1

/* CCI1 Communication boards Interface */
#define CCB_CCI1				I2C1
#define CCB_CCI1_CLK			RCC_APB1Periph_I2C1
#define CCB_CCI1_EV_IRQn		I2C1_EV_IRQn
#define CCB_CCI1_ER_IRQn		I2C1_ER_IRQn
#define CCB_CCI1_EV_IRQHandler	I2C1_EV_IRQHandler
#define CCB_CCI1_ER_IRQHandler	I2C1_ER_IRQHandler

#define CCB_CCI1_SDA_GPIO_CLK	RCC_AHB1Periph_GPIOB
#define CCB_CCI1_SDA_PIN		GPIO_Pin_9
#define CCB_CCI1_SDA_GPIO_PORT	GPIOB
#define CCB_CCI1_SDA_SOURCE		GPIO_PinSource9
#define CCB_CCI1_SDA_AF			GPIO_AF_I2C1

#define CCB_CCI1_SCL_GPIO_CLK	RCC_AHB1Periph_GPIOB
#define CCB_CCI1_SCL_PIN		GPIO_Pin_6
#define CCB_CCI1_SCL_GPIO_PORT	GPIOB
#define CCB_CCI1_SCL_SOURCE		GPIO_PinSource6
#define CCB_CCI1_SCL_AF			GPIO_AF_I2C1

/* I2CEx Communication boards Interface */
#define I2CEx					I2C3
#define I2CEx_CLK				RCC_APB1Periph_I2C3
#define I2CEx_EV_IRQn			I2C3_EV_IRQn
#define I2CEx_ER_IRQn			I2C3_ER_IRQn
#define I2CEx_EV_IRQHandler		I2C3_EV_IRQHandler
#define I2CEx_ER_IRQHandler		I2C3_ER_IRQHandler

#define I2CEx_SDA_GPIO_CLK		RCC_AHB1Periph_GPIOC
#define I2CEx_SDA_PIN			GPIO_Pin_9
#define I2CEx_SDA_GPIO_PORT		GPIOC
#define I2CEx_SDA_SOURCE		GPIO_PinSource9
#define I2CEx_SDA_AF			GPIO_AF_I2C3

#define I2CEx_SCL_GPIO_CLK		RCC_AHB1Periph_GPIOH
#define I2CEx_SCL_PIN			GPIO_Pin_7
#define I2CEx_SCL_GPIO_PORT		GPIOH
#define I2CEx_SCL_SOURCE		GPIO_PinSource7
#define I2CEx_SCL_AF			GPIO_AF_I2C3

#define I2CEx_RST_GPIO_CLK		RCC_AHB1Periph_GPIOG
#define I2CEx_RST_PIN			GPIO_Pin_5
#define I2CEx_RST_GPIO_PORT		GPIOG

/* Camera controller Interface */
#define CAM_1P8V_EN_GPIO_CLK	RCC_AHB1Periph_GPIOF
#define CAM_1P8V_EN_GPIO_PORT	GPIOF
#define CAM_1P8V_EN_PIN			GPIO_Pin_12

#define CAM_PWR_EN_GPIO_CLK		RCC_AHB1Periph_GPIOF
#define CAM_PWR_EN_GPIO_PORT	GPIOF
#define CAM_PWR_EN_PIN			GPIO_Pin_10

#define CAM_1P2V_EN_GPIO_CLK	RCC_AHB1Periph_GPIOG
#define CAM_1P2V_EN_GPIO_PORT	GPIOG
#define CAM_1P2V_EN_PIN			GPIO_Pin_7

#define CAM_2P8V_EN_GPIO_CLK	RCC_AHB1Periph_GPIOG
#define CAM_2P8V_EN_GPIO_PORT	GPIOG
#define CAM_2P8V_EN_PIN			GPIO_Pin_8

#define CAM_16P0V_EN_GPIO_CLK	RCC_AHB1Periph_GPIOF
#define CAM_16P0V_EN_GPIO_PORT	GPIOF
#define CAM_16P0V_EN_PIN		GPIO_Pin_11

#define CAM1_70MM_EN_GPIO_CLK	RCC_AHB1Periph_GPIOF
#define CAM1_70MM_EN_GPIO_PORT	GPIOF
#define CAM1_70MM_EN_PIN		GPIO_Pin_13

#define CAM2_70MM_EN_GPIO_CLK	RCC_AHB1Periph_GPIOF
#define CAM2_70MM_EN_GPIO_PORT	GPIOF
#define CAM2_70MM_EN_PIN		GPIO_Pin_14

#define CAM1_XSHUTN_GPIO_CLK	RCC_AHB1Periph_GPIOE
#define CAM1_XSHUTN_GPIO_PORT	GPIOE
#define CAM1_XSHUTN_PIN			GPIO_Pin_8

#define CAM2_XSHUTN_GPIO_CLK	RCC_AHB1Periph_GPIOE
#define CAM2_XSHUTN_GPIO_PORT	GPIOE
#define CAM2_XSHUTN_PIN			GPIO_Pin_9

/**
 * @brief Definition for COM port3, connected to USART1
 */
#define COMn					2

#define COM1_USART             	UART5
#define COM1_CLK            	RCC_APB1Periph_UART5
#define COM1_TX_PIN         	GPIO_Pin_12
#define COM1_TX_GPIO_PORT   	GPIOC
#define COM1_TX_GPIO_CLK    	RCC_AHB1Periph_GPIOC
#define COM1_TX_SOURCE      	GPIO_PinSource12
#define COM1_TX_AF          	GPIO_AF_UART5
#define COM1_RX_PIN         	GPIO_Pin_2
#define COM1_RX_GPIO_PORT   	GPIOD
#define COM1_RX_GPIO_CLK    	RCC_AHB1Periph_GPIOD
#define COM1_RX_SOURCE      	GPIO_PinSource2
#define COM1_RX_AF         		GPIO_AF_UART5
#define COM1_IRQn           	UART5_IRQn

/**
 * @brief Definition for COM port3, connected to USART1
 */
#define COM2_USART             	USART3
#define COM2_CLK            	RCC_APB1Periph_USART3
#define COM2_TX_PIN         	GPIO_Pin_10
#define COM2_TX_GPIO_PORT   	GPIOC
#define COM2_TX_GPIO_CLK    	RCC_AHB1Periph_GPIOC
#define COM2_TX_SOURCE      	GPIO_PinSource10
#define COM2_TX_AF          	GPIO_AF_USART3
#define COM2_RX_PIN         	GPIO_Pin_11
#define COM2_RX_GPIO_PORT   	GPIOC
#define COM2_RX_GPIO_CLK    	RCC_AHB1Periph_GPIOC
#define COM2_RX_SOURCE      	GPIO_PinSource11
#define COM2_RX_AF          	GPIO_AF_USART3
#define COM2_IRQn           	USART3_IRQn

/**
 * @brief Definition for SPI2 , Connected to EE_EXTERNAL FLASH
 */
#define SPIEE					SPI2
#define SPIEE_CLK				RCC_APB1Periph_SPI2
#define SPIEE_BaudRatePreScaler	SPI_BaudRatePrescaler_2  // 42/2 = 21 mhz

#define SPIEE_MISO_PIN			GPIO_Pin_2
#define SPIEE_MISO_PORT			GPIOI
#define SPIEE_MISO_CLK 			RCC_AHB1Periph_GPIOI
#define SPIEE_MISO_SOURCE		GPIO_PinSource2
#define SPIEE_MISO_AF			GPIO_AF_SPI2

#define SPIEE_MOSI_PIN			GPIO_Pin_3
#define SPIEE_MOSI_PORT			GPIOI
#define SPIEE_MOSI_CLK			RCC_AHB1Periph_GPIOI
#define SPIEE_MOSI_SOURCE		GPIO_PinSource3
#define SPIEE_MOSI_AF			GPIO_AF_SPI2

#define SPIEE_SCK_PIN			GPIO_Pin_3
#define SPIEE_SCK_PORT			GPIOD
#define SPIEE_SCK_CLK			RCC_AHB1Periph_GPIOD
#define SPIEE_SCK_SOURCE		GPIO_PinSource3
#define SPIEE_SCK_AF			GPIO_AF_SPI2

#define SPIEE_NSS_PIN			GPIO_Pin_0
#define SPIEE_NSS_PORT			GPIOI
#define SPIEE_NSS_CLK			RCC_AHB1Periph_GPIOI
#define SPIEE_NSS_SOURCE		GPIO_PinSource0
#define SPIEE_NSS_AF			GPIO_AF_SPI2

/**
 * @brief Definition for SPI5
 */

#define SNAP_SPI				SPI5
#define SNAP_CLK				RCC_APB2Periph_SPI5

#define SNAP_MISO_PIN			GPIO_Pin_8
#define SNAP_MISO_PORT			GPIOF
#define SNAP_MISO_CLK			RCC_AHB1Periph_GPIOF
#define SNAP_MISO_SOURCE		GPIO_PinSource8
#define SNAP_MISO_AF			GPIO_AF_SPI5

#define SNAP_MOSI_PIN			GPIO_Pin_9
#define SNAP_MOSI_PORT			GPIOF
#define SNAP_MOSI_CLK			RCC_AHB1Periph_GPIOF
#define SNAP_MOSI_SOURCE		GPIO_PinSource9
#define SNAP_MOSI_AF			GPIO_AF_SPI5

#define SNAP_SCK_PIN			GPIO_Pin_6
#define SNAP_SCK_PORT			GPIOH
#define SNAP_SCK_CLK			RCC_AHB1Periph_GPIOH
#define SNAP_SCK_SOURCE			GPIO_PinSource6
#define SNAP_SCK_AF				GPIO_AF_SPI5

#define SNAP_NSS_PIN			GPIO_Pin_5
#define SNAP_NSS_PORT			GPIOH
#define SNAP_NSS_CLK			RCC_AHB1Periph_GPIOH
#define SNAP_NSS_SOURCE			GPIO_PinSource5
#define SNAP_NSS_AF				GPIO_AF_SPI5

#define SNAP_DMA_TX_STREAM      DMA2_Stream6
#define SNAP_DMA_TX_CHANNEL     DMA_Channel_7

#define SNAP_DMA_RX_STREAM      DMA2_Stream5
#define SNAP_DMA_RX_CHANNEL     DMA_Channel_7


/**
 * @brief Definition for SPI1, connected to CPLD
 */
#define CPLD                    SPI1
#define CPLD_IRQn               SPI1_IRQn
#define CPLD_CLK                RCC_APB2Periph_SPI1
#define CPLD_CLK_INIT           RCC_APB2PeriphClockCmd
#define CPLD_IRQHANDLER         SPI1_IRQHandler

#define CPLD_SCK_PIN            GPIO_Pin_5
#define CPLD_SCK_GPIO_PORT      GPIOA
#define CPLD_SCK_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define CPLD_SCK_SOURCE         GPIO_PinSource5
#define CPLD_SCK_AF             GPIO_AF_SPI1

#define CPLD_MISO_PIN           GPIO_Pin_6
#define CPLD_MISO_GPIO_PORT     GPIOA
#define CPLD_MISO_GPIO_CLK      RCC_AHB1Periph_GPIOA
#define CPLD_MISO_SOURCE        GPIO_PinSource6
#define CPLD_MISO_AF            GPIO_AF_SPI1

#define CPLD_MOSI_PIN           GPIO_Pin_5
#define CPLD_MOSI_GPIO_PORT     GPIOB
#define CPLD_MOSI_GPIO_CLK      RCC_AHB1Periph_GPIOB
#define CPLD_MOSI_SOURCE        GPIO_PinSource5
#define CPLD_MOSI_AF            GPIO_AF_SPI1

#define CPLD_SS_PIN             GPIO_Pin_4
#define CPLD_SS_GPIO_PORT       GPIOA
#define CPLD_SS_GPIO_CLK        RCC_AHB1Periph_GPIOA

#define CPLD_RESET_PIN          GPIO_Pin_9
#define CPLD_RESET_GPIO_PORT    GPIOD
#define CPLD_RESET_GPIO_CLK     RCC_AHB1Periph_GPIOD

#define CPLD_PROGRAMn_PIN       GPIO_Pin_0
#define CPLD_PROGRAMn_GPIO_PORT GPIOD
#define CPLD_PROGRAMn_GPIO_CLK  RCC_AHB1Periph_GPIOD

#define CPLD_INITn_PIN       	GPIO_Pin_1
#define CPLD_INITn_GPIO_PORT 	GPIOD
#define CPLD_INITn_GPIO_CLK  	RCC_AHB1Periph_GPIOD

#define CPLD_DONE_PIN       	GPIO_Pin_2
#define CPLD_DONE_GPIO_PORT 	GPIOD
#define CPLD_DONE_GPIO_CLK  	RCC_AHB1Periph_GPIOD

#define CPLD_CONFIG_PIN       	GPIO_Pin_1
#define CPLD_CONFIG_GPIO_PORT 	GPIOG
#define CPLD_CONFIG_GPIO_CLK  	RCC_AHB1Periph_GPIOG

/**
 * @brief Definition for TIMER4, using for PID timing
 */
#define PID                     TIM4
#define PID_CLK                 RCC_APB1Periph_TIM4
#define PID_IRQHandler			TIM4_IRQHandler
#define PID_IRQn				TIM4_IRQn

// ------------ PWM -------------------------------
#define PWM                     TIM1
#define PWM_CLK                 RCC_APB2Periph_TIM1
#define PWM_AF             		GPIO_AF_TIM1

#define PWM_LENS_AFB_GPIO_CLK   RCC_AHB1Periph_GPIOA
#define PWM_LENS_AFB_GPIO_PORT  GPIOA
#define PWM_LENS_AFB_GPIO_PIN   GPIO_Pin_8
#define PWM_LENS_AFB_SOURCE     GPIO_PinSource8

#define PWM_LENS_AF_GPIO_CLK    RCC_AHB1Periph_GPIOE
#define PWM_LENS_AF_GPIO_PORT   GPIOE
#define PWM_LENS_AF_GPIO_PIN    GPIO_Pin_11
#define PWM_LENS_AF_SOURCE      GPIO_PinSource11

#define PWM_MIRR_AFB_GPIO_CLK   RCC_AHB1Periph_GPIOA
#define PWM_MIRR_AFB_GPIO_PORT  GPIOA
#define PWM_MIRR_AFB_PIN        GPIO_Pin_10
#define PWM_MIRR_AFB_SOURCE     GPIO_PinSource10

#define PWM_MIRR_AF_GPIO_CLK    RCC_AHB1Periph_GPIOA
#define PWM_MIRR_AF_GPIO_PORT   GPIOA
#define PWM_MIRR_AF_PIN         GPIO_Pin_11
#define PWM_MIRR_AF_SOURCE      GPIO_PinSource11

#define AF_FAILN0_GPIO_CLK     	RCC_AHB1Periph_GPIOI
#define AF_FAILN0_GPIO_PORT    	GPIOI
#define AF_FAILN0_GPIO_PIN     	GPIO_Pin_6
#define AF_FAILN0_SOURCE        GPIO_PinSource6

#define PWM_AF_EN0_GPIO_CLK     RCC_AHB1Periph_GPIOC
#define PWM_AF_EN0_GPIO_PORT    GPIOC
#define PWM_AF_EN0_GPIO_PIN     GPIO_Pin_6
#define PWM_AF_EN0_SOURCE       GPIO_PinSource6

#define AF_FAILN1_GPIO_CLK     	RCC_AHB1Periph_GPIOI
#define AF_FAILN1_GPIO_PORT    	GPIOI
#define AF_FAILN1_GPIO_PIN     	GPIO_Pin_10
#define AF_FAILN1_SOURCE        GPIO_PinSource10

#define PWM_AF_EN1_GPIO_CLK     RCC_AHB1Periph_GPIOC
#define PWM_AF_EN1_GPIO_PORT    GPIOC
#define PWM_AF_EN1_GPIO_PIN     GPIO_Pin_7
#define PWM_AF_EN1_SOURCE       GPIO_PinSource7

#define AF_FAILN2_GPIO_CLK     	RCC_AHB1Periph_GPIOI
#define AF_FAILN2_GPIO_PORT    	GPIOI
#define AF_FAILN2_GPIO_PIN     	GPIO_Pin_11
#define AF_FAILN2_SOURCE        GPIO_PinSource11

#define PWM_AF_EN2_GPIO_CLK     RCC_AHB1Periph_GPIOB
#define PWM_AF_EN2_GPIO_PORT    GPIOB
#define PWM_AF_EN2_GPIO_PIN     GPIO_Pin_0
#define PWM_AF_EN2_SOURCE       GPIO_PinSource0

#define AF_FAILN3_GPIO_CLK     	RCC_AHB1Periph_GPIOI
#define AF_FAILN3_GPIO_PORT    	GPIOI
#define AF_FAILN3_GPIO_PIN     	GPIO_Pin_8
#define AF_FAILN3_SOURCE        GPIO_PinSource8

#define PWM_AF_EN3_GPIO_CLK     RCC_AHB1Periph_GPIOB
#define PWM_AF_EN3_GPIO_PORT    GPIOB
#define PWM_AF_EN3_GPIO_PIN     GPIO_Pin_1
#define PWM_AF_EN3_SOURCE       GPIO_PinSource1

#define AF_FAILN4_GPIO_CLK     	RCC_AHB1Periph_GPIOH
#define AF_FAILN4_GPIO_PORT    	GPIOH
#define AF_FAILN4_GPIO_PIN     	GPIO_Pin_2
#define AF_FAILN4_SOURCE        GPIO_PinSource2

#define PWM_AF_EN4_GPIO_CLK     RCC_AHB1Periph_GPIOB
#define PWM_AF_EN4_GPIO_PORT    GPIOB
#define PWM_AF_EN4_GPIO_PIN     GPIO_Pin_6
#define PWM_AF_EN4_SOURCE       GPIO_PinSource6

#define AF_FAILN5_GPIO_CLK     	RCC_AHB1Periph_GPIOF
#define AF_FAILN5_GPIO_PORT    	GPIOF
#define AF_FAILN5_GPIO_PIN     	GPIO_Pin_2
#define AF_FAILN5_SOURCE        GPIO_PinSource2

#define PWM_AF_EN5_GPIO_CLK     RCC_AHB1Periph_GPIOB
#define PWM_AF_EN5_GPIO_PORT    GPIOB
#define PWM_AF_EN5_GPIO_PIN     GPIO_Pin_7
#define PWM_AF_EN5_SOURCE       GPIO_PinSource7

#define AF_FAILN6_GPIO_CLK     	RCC_AHB1Periph_GPIOF
#define AF_FAILN6_GPIO_PORT    	GPIOF
#define AF_FAILN6_GPIO_PIN     	GPIO_Pin_3
#define AF_FAILN6_SOURCE        GPIO_PinSource3

#define PWM_AF_EN6_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define PWM_AF_EN6_GPIO_PORT    GPIOD
#define PWM_AF_EN6_GPIO_PIN     GPIO_Pin_14
#define PWM_AF_EN6_SOURCE       GPIO_PinSource14

#define AF_FAILN7_GPIO_CLK     	RCC_AHB1Periph_GPIOF
#define AF_FAILN7_GPIO_PORT    	GPIOF
#define AF_FAILN7_GPIO_PIN     	GPIO_Pin_4
#define AF_FAILN7_SOURCE        GPIO_PinSource4

#define PWM_AF_EN7_GPIO_CLK     RCC_AHB1Periph_GPIOD
#define PWM_AF_EN7_GPIO_PORT    GPIOD
#define PWM_AF_EN7_GPIO_PIN     GPIO_Pin_15
#define PWM_AF_EN7_SOURCE       GPIO_PinSource15

#define AF_FAILN8_GPIO_CLK     	RCC_AHB1Periph_GPIOF
#define AF_FAILN8_GPIO_PORT    	GPIOF
#define AF_FAILN8_GPIO_PIN     	GPIO_Pin_5
#define AF_FAILN8_SOURCE        GPIO_PinSource5

#define PWM_AF_EN8_GPIO_CLK     RCC_AHB1Periph_GPIOH
#define PWM_AF_EN8_GPIO_PORT    GPIOH
#define PWM_AF_EN8_GPIO_PIN     GPIO_Pin_10
#define PWM_AF_EN8_SOURCE       GPIO_PinSource10

#define AF_FAILN9_GPIO_CLK     	RCC_AHB1Periph_GPIOF
#define AF_FAILN9_GPIO_PORT    	GPIOF
#define AF_FAILN9_GPIO_PIN     	GPIO_Pin_6
#define AF_FAILN9_SOURCE        GPIO_PinSource6

#define PWM_AF_EN9_GPIO_CLK     RCC_AHB1Periph_GPIOH
#define PWM_AF_EN9_GPIO_PORT    GPIOH
#define PWM_AF_EN9_GPIO_PIN     GPIO_Pin_11
#define PWM_AF_EN9_SOURCE       GPIO_PinSource11

#define AF_FAILN10_GPIO_CLK     RCC_AHB1Periph_GPIOF
#define AF_FAILN10_GPIO_PORT    GPIOF
#define AF_FAILN10_GPIO_PIN     GPIO_Pin_7
#define AF_FAILN10_SOURCE       GPIO_PinSource7

#define PWM_AF_EN10_GPIO_CLK    RCC_AHB1Periph_GPIOH
#define PWM_AF_EN10_GPIO_PORT   GPIOH
#define PWM_AF_EN10_GPIO_PIN    GPIO_Pin_12
#define PWM_AF_EN10_SOURCE      GPIO_PinSource12

/**
 * @brief Definition for Power Good , connected to FPGA, Camera sensors
 */

#define FPGA_1P0V_PG_PIN        GPIO_Pin_13
#define FPGA_1P0V_PG_PORT      	GPIOG
#define FPGA_1P0V_PG_CLK       	RCC_AHB1Periph_GPIOG

#define FPGA_1P8V_PG_PIN        GPIO_Pin_2
#define FPGA_1P8V_PG_PORT      	GPIOG
#define FPGA_1P8V_PG_CLK       	RCC_AHB1Periph_GPIOG

#define FPGA_1P5V_PG_PIN      	GPIO_Pin_3
#define FPGA_1P5V_PG_PORT      	GPIOG
#define FPGA_1P5V_PG_CLK       	RCC_AHB1Periph_GPIOG

#define CAM_2P8V_PG_PIN      	GPIO_Pin_0
#define CAM_2P8V_PG_PORT      	GPIOG
#define CAM_2P8V_PG_CLK       	RCC_AHB1Periph_GPIOG

#define CAM_1P2V_PG_PIN        	GPIO_Pin_10
#define CAM_1P2V_PG_PORT      	GPIOG
#define CAM_1P2V_PG_CLK       	RCC_AHB1Periph_GPIOG

#define CAM_3P3V_PG_PIN        	GPIO_Pin_11
#define CAM_3P3V_PG_PORT      	GPIOG
#define CAM_3P3V_PG_CLK       	RCC_AHB1Periph_GPIOG
/**
 * @brief Definition for SPI2 , Connected to EE_EXTERNAL FLASH
 */
#define SPI_FPGA					SPI4
#define SPI_FPGA_CLK				RCC_APB2Periph_SPI4
#define SPI_FPGA_BaudRatePreScaler	SPI_BaudRatePrescaler_16  // 42/2 = 21 mhz

#define SPI_FPGA_MISO_PIN			GPIO_Pin_5
#define SPI_FPGA_MISO_PORT			GPIOE
#define SPI_FPGA_MISO_CLK 			RCC_AHB1Periph_GPIOE
#define SPI_FPGA_MISO_SOURCE		GPIO_PinSource5
#define SPI_FPGA_MISO_AF			GPIO_AF_SPI4

#define SPI_FPGA_MOSI_PIN			GPIO_Pin_6
#define SPI_FPGA_MOSI_PORT			GPIOE
#define SPI_FPGA_MOSI_CLK			RCC_AHB1Periph_GPIOE
#define SPI_FPGA_MOSI_SOURCE		GPIO_PinSource6
#define SPI_FPGA_MOSI_AF			GPIO_AF_SPI4

#define SPI_FPGA_SCK_PIN			GPIO_Pin_2
#define SPI_FPGA_SCK_PORT			GPIOE
#define SPI_FPGA_SCK_CLK			RCC_AHB1Periph_GPIOE
#define SPI_FPGA_SCK_SOURCE			GPIO_PinSource2
#define SPI_FPGA_SCK_AF				GPIO_AF_SPI4

#define SPI_FPGA_NSS_PIN			GPIO_Pin_4
#define SPI_FPGA_NSS_PORT			GPIOE
#define SPI_FPGA_NSS_CLK			RCC_AHB1Periph_GPIOE
#define SPI_FPGA_NSS_SOURCE			GPIO_PinSource4
#define SPI_FPGA_NSS_AF				GPIO_AF_SPI4

#define FPGA_IRQ_GPIO_CLK			RCC_AHB1Periph_GPIOF
#define FPGA_IRQ_GPIO_PORT			GPIOF
#define FPGA_IRQ_PIN				GPIO_Pin_15

/**
 * @brief Definition for IRQ to Host , Connected STM_IRQ
 */
#define STM_IRQ_GPIO_CLK     RCC_AHB1Periph_GPIOC
#define STM_IRQ_GPIO_PORT    GPIOC
#define STM_IRQ_GPIO_PIN     GPIO_Pin_0
#define STM_IRQ_SOURCE       GPIO_PinSource0
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __CCB_CONFIG_H */

/************ Portions COPYRIGHT 2015 Light.Co., Ltd.******END OF FILE******/
