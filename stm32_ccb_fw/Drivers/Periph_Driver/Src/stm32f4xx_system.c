/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    14-December-2012
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *          This file contains the system clock configuration for STM32F4xx devices,
  *          and is generated by the clock configuration tool
  *          stm32f4xx_Clock_Configuration_V1.0.1.xls
  *
  * 1.  This file provides two functions and one global variable to be called from
  *     user application:
  *      - SystemInit(): Setups the system clock (System clock source, PLL Multiplier
  *                      and Divider factors, AHB/APBx prescalers and Flash settings),
  *                      depending on the configuration made in the clock xls tool.
  *                      This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  * 2. After each device reset the HSI (16 MHz) is used as system clock source.
  *    Then SystemInit() function is called, in "startup_stm32f4xx.s" file, to
  *    configure the system clock before to branch to main program.
  *
  * 3. If the system clock source selected by user fails to startup, the SystemInit()
  *    function will do nothing and HSI still used as system clock source. User can
  *    add some code to deal with this issue inside the SetSysClock() function.
  *
  * 4. The default value of HSE crystal is set to 25MHz, refer to "HSE_VALUE" define
  *    in "stm32f4xx.h" file. When HSE is used as system clock source, directly or
  *    through PLL, and you are using different crystal you have to adapt the HSE
  *    value to your own configuration.
  *
  * 5. This file configures the system clock as follows:
  *=============================================================================
  *=============================================================================
  *        Supported STM32F4xx device revision    | Rev A
  *-----------------------------------------------------------------------------
  *        System Clock source                    | PLL (HSE)
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 160000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 160000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 4
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 2
  *-----------------------------------------------------------------------------
  *        HSE Frequency(Hz)                      | 8000000
  *-----------------------------------------------------------------------------
  *        PLL_M                                  | 8
  *-----------------------------------------------------------------------------
  *        PLL_N                                  | 320
  *-----------------------------------------------------------------------------
  *        PLL_P                                  | 2
  *-----------------------------------------------------------------------------
  *        PLL_Q                                  | 7
  *-----------------------------------------------------------------------------
  *        PLLI2S_N                               | 384
  *-----------------------------------------------------------------------------
  *        PLLI2S_R                               | 2
  *-----------------------------------------------------------------------------
  *        I2S input clock(Hz)                    | 192000000
  *                                               |
  *        To achieve the following I2S config:   |
  *         - Master clock output (MCKO): OFF     |
  *         - Frame wide                : 16bit   |
  *         - Audio sampling freq (KHz) : 48      |
  *         - Error %                   : 0.0000  |
  *         - Prescaler Odd factor (ODD): 0       |
  *         - Linear prescaler (DIV)    : 21      |
  *-----------------------------------------------------------------------------
  *        VDD(V)                                 | 3.3
  *-----------------------------------------------------------------------------
  *        Main regulator output voltage          | Scale1 mode
  *-----------------------------------------------------------------------------
  *        Flash Latency(WS)                      | 5
  *-----------------------------------------------------------------------------
  *        Prefetch Buffer                        | OFF
  *-----------------------------------------------------------------------------
  *        Instruction cache                      | ON
  *-----------------------------------------------------------------------------
  *        Data cache                             | ON
  *-----------------------------------------------------------------------------
  *        Require 48MHz for USB OTG FS,          | Disabled
  *        SDIO and RNG clock                     |
  *-----------------------------------------------------------------------------
  *=============================================================================
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

#include "stm32f4xx.h"

/************************* PLL Parameters *************************************/
#define	PLL_M	   8															// PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
#define	PLL_N	   320
#define	PLL_P	   2															// SYSCLK = PLL_VCO / PLL_P

#define	PLL_Q	   7															// USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ
#define	PLLI2S_N   384															// PLLI2S_VCO = (HSE_VALUE Or HSI_VALUE / PLL_M) * PLLI2S_N
#define	PLLI2S_R   2															// I2SCLK = PLLI2S_VCO / PLLI2S_R

/******************************************************************************/

uint32_t SystemCoreClock = 160000000;

__I	uint8_t	AHBPrescTable[16] =	{0,	0, 0, 0, 0,	0, 0, 0, 1,	2, 3, 4, 6,	7, 8, 9};

static void	SetSysClock(void);

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the
  *         SystemFrequency variable.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{

//
//	Reset the RCC clock configuration to the default reset state
//
	RCC->CR |= (uint32_t)0x00000001;											// Set HSION bit
	RCC->CFGR	= 0x00000000;													// Reset CFGR register
	RCC->CR &= (uint32_t)0xFEF6FFFF;											// Reset HSEON, CSSON and PLLON bits
	RCC->PLLCFGR = 0x24003010;													// Reset PLLCFGR register
	RCC->CR &= (uint32_t)0xFFFBFFFF;											// Reset HSEBYP bit
	RCC->CIR = 0x00000000;														// Disable all interrupts

//
//	Set the system clock to 160MHz
//
	SetSysClock();

}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *           
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.         
  *     
  * @note   - The system frequency computed by this function is not the real 
  *           frequency in the chip. It is calculated based on the predefined 
  *           constant and the selected clock source:
  *             
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f4xx.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f4xx.h file (default value
  *              25 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *                
  *         - The result of this function will not be correct when using fractional
  *           value for HSE crystal.
  *     
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
	uint32_t tmp = 0,	pllvco = 0,	pllp = 2, pllsource	= 0, pllm =	2;

	tmp =	RCC->CFGR &	RCC_CFGR_SWS;											// Get SYSCLK source

	switch (tmp){
	case 0x00:
		SystemCoreClock	= HSI_VALUE;											// HSI used as system clock source
		break;

	case 0x04:
		SystemCoreClock = HSE_VALUE;											// HSE used as system clock source
	  	break;

	case 0x08:							   										//	PLL used as system clock source
		pllsource	= (RCC->PLLCFGR	& RCC_PLLCFGR_PLLSRC) >> 22;				//	PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
		pllm		= RCC->PLLCFGR & RCC_PLLCFGR_PLLM;							//	SYSCLK = PLL_VCO / PLL_P

		if (pllsource != 0)
			pllvco = (HSE_VALUE	/ pllm)	* ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >>	6);
		else
			pllvco = (HSI_VALUE	/ pllm)	* ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >>	6);

		pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1) *2;
		SystemCoreClock =	pllvco/pllp;
		break;

	default:
		SystemCoreClock = HSI_VALUE;
		break;
	}

	tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];					//	Get HCLK prescaler
	SystemCoreClock >>= tmp;
}

/**
  * @brief  Configures the System clock source, PLL Multiplier and Divider factors,
  *         AHB/APBx prescalers and Flash settings
  * @Note   This function should be called only once the RCC clock configuration
  *         is reset to the default reset state (done in SystemInit() function).
  * @param  None
  * @retval None
  */
static void	SetSysClock(void)
{
	__IO uint32_t	StartUpCounter = 0,	HSEStatus =	0;

//
//	Main PLL configuration:
//	This function configures the HSE + PLL to be the system clock
//	Note: This function assumes that the HSE is the system clock source
//
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);										//	Enable HSE

	do																			//	Wait till HSE is ready and if Time out is reached exit
	{
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
		HSEStatus =	(uint32_t)0x01;
	else
		HSEStatus =	(uint32_t)0x00;

	if (HSEStatus	== (uint32_t)0x01)
	{

		RCC->APB1ENR |=	RCC_APB1ENR_PWREN;										//	Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz
		PWR->CR |= PWR_CR_VOS;

		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;										//	HCLK = SYSCLK / 1
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;										//	PCLK2 = HCLK / 2
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;										//	PCLK1 = HCLK / 4

		RCC->PLLCFGR =	PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |		//	Configure the main PLL
						(RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);


		RCC->CR |= RCC_CR_PLLON;												//	Enable the main PLL

		while ((RCC->CR & RCC_CR_PLLRDY)	== 0)								//	Wait till the main PLL is ready
		{
		}


		FLASH->ACR = FLASH_ACR_ICEN	|FLASH_ACR_DCEN	|FLASH_ACR_LATENCY_5WS;		//	Configure Flash prefetch, Instruction cache, Data cache and wait state
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));						//	Select the main PLL as system clock source
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		while ((RCC->CFGR &	(uint32_t)RCC_CFGR_SWS) !=	RCC_CFGR_SWS_PLL);		//	Wait till the main PLL is used as system clock source
		{
		}
	}
	else																		//	If HSE fails to start-up,
	{																			//	the application will have an incorrect clock configuration.
	}      																		//	User must handle this error.

//
//	I2S clock configuration:
//
	RCC->CFGR &= ~RCC_CFGR_I2SSRC;												//	PLLI2S clock used as I2S clock source
	RCC->PLLI2SCFGR = (PLLI2S_N << 6) | (PLLI2S_R << 28);						//	Configure PLLI2S
	RCC->CR |= ((uint32_t)RCC_CR_PLLI2SON);										//	Enable PLLI2S
	while((RCC->CR & RCC_CR_PLLI2SRDY) == 0)									//	Wait till PLLI2S is ready
	{
	}
}

//
//	End of file
//
