/*
 * drv_piezo_pwm.c
 *
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "drv_piezo_pwm.h"
#include "hal_i2c_ex.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
UInt32 GetPulseLength(PiezoPwm *pwm);
UInt32 GetTimePeriod (PiezoPwm *pwm);

static void ConfigurePwmGpios(void);
static void InitTimer(void);
static void InitTimerOutputChannels(void);

/* Private variables ---------------------------------------------------------*/
static xSemaphoreHandle timer_semaphore;
static volatile uint64_t timer_count_down = 0;

void PrintPwm(PiezoPwm *pwm)
{
	if (pwm == NULL)
	{
		log_error("PrintPWM: Passed NULL");
		return;
	}

	log_debug("PWM: f:%d,%03d Hz D:%d.%d  %s",
			  (int)(pwm->Frequency / 1000),
			  (int)(pwm->Frequency % 1000),
			  pwm->DutyCycle / 10,
			  pwm->DutyCycle % 10,
			  pwm->Active ? "ON" : "OFF");
}


Error_t
ConfigurePwm(PiezoPwm      *pwm,
			 PiezoActuator *a,
			 UInt16         DutyCycle,
			 UInt8          Direction,
			 UInt8          Active)
{
	Error_t e = ERROR_NONE;
	UInt32 pulse; // Pulse width, in units of system clock ticks (?)

	//ENTER_FUNC

	CHECK_ERROR(pwm == NULL ? ERROR_INVALID_ARG : ERROR_NONE);

	// TurnOffPwm(pwm, a);

	pwm->Frequency     = a->CalibData.Frequency;
	pwm->DutyCycle     = DutyCycle;
	pwm->Direction     = Direction;
	pwm->FlipDirection = a->ActuatorData.FlipDirection;

	// TODO: Reprogram the hardware with the configuration data.

	TIM1->ARR = GetTimePeriod(pwm);
	TIM1->EGR = TIM_PSCReloadMode_Immediate;

	pulse = GetPulseLength(pwm);

	// log_debug("DutyCyle:%d.%d Dir:%d FlipDir:%d Pulse:%d",
	//		DutyCycle/10, DutyCycle%10, pwm->Direction,
	//		pwm->FlipDirection, (int) pulse);

	// log_debug("pulse = %d", (int)pulse);

	/* Program only CCR1 & CCR2 if module is Lens and only CCR3 & CCR4 for mirror */
	if (a->Type == ActuatorType_Lens)
	{
		TIM1->CCR1 = pulse;
		TIM1->CCR2 = pulse;
	}
	else if (a->Type == ActuatorType_Mirror)
	{
		TIM1->CCR3 = pulse;
		TIM1->CCR4 = pulse;
	}

	//vTaskDelay(1); // Insert a pause....
	//PrintPwm(pwm);
	//vTaskDelay(1);

	if (Active)
	{
		TurnOnPwm(pwm, a);
	}

	//vTaskDelay(1); // Insert a pause....
	//PrintPwm(pwm);
	//vTaskDelay(1);

	// EXIT_FUNC

	return e;
}


Error_t TurnOnPwm(PiezoPwm *pwm, PiezoActuator *a)
{
	Error_t e = ERROR_NONE;

	// ENTER_FUNC

	CHECK_ERROR(pwm        == NULL ? ERROR_INVALID_ARG : ERROR_NONE);
	CHECK_ERROR(a          == NULL ? ERROR_INVALID_ARG : ERROR_NONE);
	CHECK_ERROR(a->Channel == NULL ? ERROR_INVALID_ARG : ERROR_NONE);

	if (pwm->Active) return e; // Already on.

	// log_debug("Port: 0x%08x  Pin: %d", (int)a->Channel->Port, a->Channel->Pin);

	// Enable the PWM.
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	GPIO_SetBits(a->Channel->Port, a->Channel->Pin);

	pwm->Active = 1;

	//vTaskDelay(1); // Insert a pause....
	//PrintPwm(pwm);
	//vTaskDelay(1);

	// EXIT_FUNC

	return e;
}

Error_t TurnOffPwm(PiezoPwm *pwm, PiezoActuator *a)
{
	Error_t e = ERROR_NONE;

	// ENTER_FUNC

	CHECK_ERROR(pwm        == NULL ? ERROR_INVALID_ARG : ERROR_NONE);
	CHECK_ERROR(a          == NULL ? ERROR_INVALID_ARG : ERROR_NONE);
	CHECK_ERROR(a->Channel == NULL ? ERROR_INVALID_ARG : ERROR_NONE);

	if (!pwm->Active) return e; // Already off.

	// Disable the PWM.
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	GPIO_ResetBits(a->Channel->Port, a->Channel->Pin);

	pwm->Active = 0;

	//vTaskDelay(1); // Insert a pause....
	//PrintPwm(pwm);
	//vTaskDelay(1);

	// EXIT_FUNC

	return e;
}

/*
 * Calculate Capture Compare register value to program desired Duty cycle
 * pulse_length = ((TIM_Period + 1) * DutyCycle) / 1000 - 1
 * Duty Cycle is in 0.1 percent, between 0 & 100%
 */
UInt32 GetPulseLength(PiezoPwm *pwm)
{
	UInt32 TimePeriod;
	UInt32 EffectiveDutyCycle;
	UInt32 PulseLength;

	TimePeriod = GetTimePeriod(pwm);

	// Mirror is flipped:
	// Mirror & Dir = FAR  (0): duty
	// Mirror & Dir = NEAR (1): 1000-duty
	// Lens   & Dir = FAR  (0): 1000-duty
	// Lens   & Dir = NEAR (1): duty
	EffectiveDutyCycle = (pwm->Direction ^ pwm->FlipDirection) ? pwm->DutyCycle : (1000 - pwm->DutyCycle);

	PulseLength = (((UInt32)((TimePeriod + 1) * EffectiveDutyCycle) / 1000) -1);

	return PulseLength;
}

/*
 * Calculate Auto reload register value for TIM1 to program desired PWM frequency
 * PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
 */
UInt32 GetTimePeriod(PiezoPwm *pwm)
{
	return (UInt32)((SystemCoreClock / pwm->Frequency) - 1);
}

/*
 * Hardware Initialization Functions
 */

/* Initialize Timer 1 for PWM */
static void InitTimer()
{
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;

	/* Enable Clock for Timer 1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* Timer base configuration */

	/* We want timer to operate at max tick frequency so no pre scale required */
	TIM_TimeBaseStructure.TIM_Prescaler     = 0;
	TIM_TimeBaseStructure.TIM_Period        = 0;   // 65535
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* TIM1 enable counter */
	TIM_Cmd(TIM1, ENABLE);
}

/* Configure GPIOs for PWM for Lens & Mirror */
static void ConfigurePwmGpios()
{
	GPIO_InitTypeDef           GPIO_InitStructure;

	/* Enable Clock for Lens & Mirror GPIO which maps to GPIO A & E */
	RCC_AHB1PeriphClockCmd(PWM_LENS_AFB_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_LENS_AF_GPIO_CLK,  ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_MIRR_AFB_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_MIRR_AF_GPIO_CLK,  ENABLE);

	/* Configure GPIOs for Lens & Mirror */
	GPIO_InitStructure.GPIO_Pin   = PWM_LENS_AFB_GPIO_PIN ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
	GPIO_Init(PWM_LENS_AFB_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = PWM_LENS_AF_GPIO_PIN;
	GPIO_Init(PWM_LENS_AF_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = PWM_MIRR_AFB_PIN;
	GPIO_Init(PWM_MIRR_AFB_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = PWM_MIRR_AF_PIN;
	GPIO_Init(PWM_MIRR_AF_GPIO_PORT, &GPIO_InitStructure);

	/* Alternating functions for pins */
	GPIO_PinAFConfig(PWM_LENS_AFB_GPIO_PORT, PWM_LENS_AFB_SOURCE, PWM_AF);
	GPIO_PinAFConfig(PWM_LENS_AF_GPIO_PORT,  PWM_LENS_AF_SOURCE,  PWM_AF);
	GPIO_PinAFConfig(PWM_MIRR_AFB_GPIO_PORT, PWM_MIRR_AFB_SOURCE, PWM_AF);
	GPIO_PinAFConfig(PWM_MIRR_AF_GPIO_PORT,  PWM_MIRR_AF_SOURCE,  PWM_AF);

/*
    GPIO_SetBits((GPIO_TypeDef *)0x40020400, 64);//B1
    GPIO_SetBits((GPIO_TypeDef *)0x40020400, 2);//B2
    GPIO_SetBits((GPIO_TypeDef *)0x40020400, 1);//B3
    GPIO_SetBits((GPIO_TypeDef *)0x40020800, 64);//B4
    GPIO_SetBits((GPIO_TypeDef *)0x40020800, 128);//B5
    GPIO_SetBits((GPIO_TypeDef *)0x40020400, 128);//C1
    GPIO_SetBits((GPIO_TypeDef *)0x40020C00, 16384);//C2
    GPIO_SetBits((GPIO_TypeDef *)0x40021C00, 1024);//C3
    GPIO_SetBits((GPIO_TypeDef *)0x40020C00, 32768);//C4
    GPIO_SetBits((GPIO_TypeDef *)0x40021C00, 4096);//C5
    GPIO_SetBits((GPIO_TypeDef *)0x40021C00, 2048);//C6
*/
}

/* Initialize PWM output channels */
static void InitTimerOutputChannels()
{
	TIM_OCInitTypeDef          TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse        = 0;

	/* Channel 1 & Channel 2 for Lens */
#define InitChannel(n, p)                                     \
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_##p;  \
	TIM_OC##n##Init         (PWM, &TIM_OCInitStructure);      \
	TIM_OC##n##PreloadConfig(PWM,  TIM_OCPreload_Enable);

	InitChannel(1, High); // Lens
	InitChannel(2, Low ); // Lens
	InitChannel(3, High); // Mirror
	InitChannel(4, Low ); // Mirror
}



void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
      TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
      if(timer_count_down)
      {
          timer_count_down--;
      }
      else
      {
          portBASE_TYPE xHigherPriorityTaskWoken;
          xHigherPriorityTaskWoken = pdFALSE;
          xSemaphoreGiveFromISR(timer_semaphore, &xHigherPriorityTaskWoken);
      }
  }
}

void DelayUSec(UInt32 usec)
{
    /* Force task to wait exact its time out */
    if(xSemaphoreTake(timer_semaphore, usec/1000))
    {
        timer_count_down = usec;
        TIM_Cmd(TIM4, ENABLE);
        TIM_ITConfig(TIM4,TIM_IT_Update, ENABLE);
        /* Block timer to wait for elapse */
        xSemaphoreTake(timer_semaphore, usec/1000);
    }
    TIM_ITConfig(TIM4,TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM4, DISABLE);
}

static void delay_timer_init(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
  NVIC_InitTypeDef           NVIC_InitStructure;
  /* Initialize semaphore */
  timer_semaphore = xSemaphoreCreateBinary();
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock/8000000) - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM4,TIM_IT_Update, DISABLE);
  TIM_Cmd(TIM4, DISABLE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void InitPwm(void)
{
	ENTER_FUNC

	ConfigurePwmGpios();
	InitTimer();
	InitTimerOutputChannels();
	delay_timer_init();
	EXIT_FUNC
}

