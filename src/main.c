// includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "main.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx.h"

// declare static functions
static inline void MX_GPIO_Init(void); // GPIO
static inline void PWMLed(void); // TIM3

// declare main variables
uint32_t micros();
static volatile uint32_t lastMicros = 0;
static volatile uint32_t extraMillis = 0;
static volatile uint32_t extraMillis2 = 0;
#ifndef TIME_TIMER_32BIT
	static volatile uint16_t TXOV = 0;
#endif

//#define CONSOLE
#ifdef CONSOLE
	extern void initialise_monitor_handles(void); // console debug
#endif

/*
 * @brief Main Loop Function
 * @retval None
 */
int main(void)
{

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	SystemClock_Config();
	SystemTimerConfig();
	MX_GPIO_Init();
	TIM5_IRQHandler();

	PWMLed();
	#ifdef CONSOLE
		initialise_monitor_handles(); // console debug
		puts("Hello Arm World!");
		printf("System clock: %lu Hz\n", SystemCoreClock);
	#endif

//	LL_GPIO_TogglePin(LED_GPIO, LED1);
	LL_GPIO_TogglePin(LED_GPIO, LED2);

	//uint16_t DAC_OUT[4] = {0, 310, 620, 930};
	uint16_t DAC_OUT[13] = {0, 78, 155, 231, 310, 388, 466, 535, 620, 698, 775, 851, 930};
    //uint16_t DAC_OUT[931];

//    uint8_t i = 0;
//    for(int y = 0; y <= 930; y++){
//    	DAC_OUT[y] = y;
//    }

    int delta = 1;
    int count = 0;
    //int max = 930;
    int max = 13;
    int min = 0;

	while (1)
	{
	  lastMicros = micros();
	  if (extraMillis < lastMicros)
	  {
		  LL_GPIO_TogglePin(LED_GPIO, LED2);
		  extraMillis = lastMicros + 1500000;
	  }

	  if (extraMillis2 < lastMicros)
	  {
//		  TIM3->CCR1 = DAC_OUT[i++];
//		  if(i == 13)
//			  i = 0;
		  //TIM3->CCR1 = DAC_OUT[count];
		  LL_TIM_OC_SetCompareCH1(TIM3, DAC_OUT[count]);
		  count += delta;
		  if(count == min || count == max) delta = -delta;
		  //extraMillis2 = lastMicros + 1500; // for 1023 values
		  extraMillis2 = lastMicros + 75000; // for 13 values
	  }

	}

}

/*
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	/* set up the FLASH latency */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5); // Configure the FLASH PREFETCH and the LATENCY Related Settings

	if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5) // Assert if Flash Latency is not 5ws
	{
	Error_Handler();
	}

	/* set up the voltage regulator scaling */
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1); // enable the voltage regulator bit 14 in the register

	/* enable the HSE and wait for it to become ready */
	LL_RCC_HSE_Enable(); // in the RCC -> CR register set the RCC_CR_HSEON bit - this is already defined in the header file and is basically write a 1 at the 16th position

	/* wait till HSE is ready */
	while(LL_RCC_HSE_IsReady() != 1) // wait for the HSE ready bit to set in the CR register. This will indicate that the crystal is on.
	{

	}

	/* configure and enable the main PLL */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2); // RCC_PLLCFGR_PLLSRC_HSE, PLLM = 4, PLLN = 168, PLLP = 0(2)
	LL_RCC_PLL_Enable(); // Enable the PLL

	/* wait till PLL is ready */
	while(LL_RCC_PLL_IsReady() != 1)
	{

	}

	/* configure the PRESCALARS HCLK, PCLK1, PCLK2 */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1); // set the DPRE division to 1
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4); // set the PPRE1 division to 4
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2); // set the PPRE2 division to 2
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL); // set the clock source

	/* select the clock source and wait for it to become ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{

	}

	LL_Init1msTick(168000000); // configures the Cortex-M SysTick source to have 1ms time base.
	LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK); // configures the SysTick clock source
	LL_SetSystemCoreClock(168000000); // sets the SystemCoreClock CMSIS variable
}

/*
 * @brief Clock Source Configuration
 * @retval None
 */
void SystemClock_Config_Extra(void)
{
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3); // SET BIT RCC_APB1RSTR <- RCC_APB1ENR_TIM3EN
	LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_GPIOA); // SET BIT RCC_APB1RSTR <- RCC_AHB1ENR_GPIOAEN

    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3); // CLEAR BIT RCC_APB1RSTR <- RCC_APB1ENR_TIM3EN
	LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_GPIOA); // CLEAR BIT RCC_APB1RSTR <- RCC_AHB1ENR_GPIOAEN

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3); // SET BIT - READ BIT RCC_APB1RSTR <- RCC_APB1ENR_TIM3EN
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA); // SET BIT - READ BIT RCC_APB1RSTR <- RCC_AHB1ENR_GPIOAEN
}

/*
 * @brief System Timer Configuration on TIM5
 * @retval lastMicros
 */
void SystemTimerConfig(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

  NVIC_SetPriority(TIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), SYS_TIMER_PRIORITY_GROUP, SYS_TIMER_PRIORITY_SUBGROUP));
  NVIC_EnableIRQ(TIM5_IRQn);

  TIM_InitStruct.Prescaler = 84; // 84mhz
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  #ifdef TIME_TIMER_32BIT
  	  TIM_InitStruct.Autoreload  = 0xFFFFFFFF;
  #else
  	  TIM_InitStruct.Autoreload  = 0xFFFF;
  #endif
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;

  LL_TIM_Init(TIME_TIMER, &TIM_InitStruct);

  LL_TIM_EnableCounter(TIME_TIMER);
  LL_TIM_EnableIT_UPDATE(TIME_TIMER);

  LL_TIM_DisableARRPreload(TIME_TIMER);
  LL_TIM_SetTriggerOutput(TIME_TIMER, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIME_TIMER);

  lastMicros = micros();
}

/*
* @brief TIM3 Initialization Function
* @param None
* @retval None
*/
static inline void PWMLed(void) // PWM for LED1
{
	LL_GPIO_SetPinMode(LED_GPIO, LED1, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(LED_GPIO, LED1, LL_GPIO_AF_2);
	LL_GPIO_SetPinOutputType(LED_GPIO, LED1, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(LED_GPIO, LED1, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinSpeed(LED_GPIO, LED1, LL_GPIO_SPEED_FREQ_LOW);


	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	LL_TIM_SetPrescaler(TIM3, 0);//__LL_TIM_CALC_PSC(SystemCoreClock, 10000));
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP); // MODIFY_REG(TIMx->CR1, (TIM_CR1_DIR | TIM_CR1_CMS), CounterMode);
	LL_TIM_SetAutoReload(TIM3, 1023); // WRITE_REG(TIMx->ARR, AutoReload); // TIMx_ARR = signal frequency // HAL Period
	LL_TIM_SetClockDivision(TIM3, LL_TIM_CLOCKDIVISION_DIV1); // MODIFY_REG(TIMx->CR1, TIM_CR1_CKD, ClockDivision);
	LL_TIM_EnableARRPreload(TIM3); // SET_BIT(TIMx->CR1, TIM_CR1_ARPE);

	LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL); // MODIFY_REG(TIMx->SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE, ClockSource);

	LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET); // MODIFY_REG(TIMx->CR2, TIM_CR2_MMS, TimerSynchronization);
	LL_TIM_DisableMasterSlaveMode(TIM3); // CLEAR_BIT(TIMx->SMCR, TIM_SMCR_MSM);

	LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1); // MODIFY_REG(*pReg, ((TIM_CCMR1_OC1M  | TIM_CCMR1_CC1S) << SHIFT_TAB_OCxx[iChannel]),  Mode << SHIFT_TAB_OCxx[iChannel]); // CCMRx OCx = set PWM MODE
	LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH); // MODIFY_REG(TIMx->CCER, (TIM_CCER_CC1P << SHIFT_TAB_CCxP[iChannel]),  Polarity << SHIFT_TAB_CCxP[iChannel]);
	LL_TIM_OC_SetCompareCH1(TIM3, 0); // WRITE_REG(TIMx->CCR1, CompareValue); // TIMx_CCRx = duty cycle // Pulse??

	LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1); // SET_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));

	LL_TIM_EnableIT_CC1(TIM3); // SET_BIT(TIMx->DIER, TIM_DIER_CC1IE);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); // SET_BIT(TIMx->CCER, Channels);

	LL_TIM_EnableCounter(TIM3); // SET_BIT(TIMx->CCER, Channels);
	LL_TIM_EnableAllOutputs(TIM3); // SET_BIT(TIMx->BDTR, TIM_BDTR_MOE);
	LL_TIM_GenerateEvent_UPDATE(TIM3); // SET_BIT(TIMx->EGR, TIM_EGR_UG);
}

/*
 * @brief TIM5 IRQ handler
 */
void TIM5_IRQHandler(void) {
	if (TIME_TIMER->SR & LL_TIM_SR_UIF) {
		TIME_TIMER->SR = (uint16_t)~LL_TIM_SR_UIF;
		#ifndef TIME_TIMER_32BIT
			TXOV++;
		#endif
	}
}

/*
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static inline void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Set LED1 and LED2 as outputpin */
  LL_GPIO_SetOutputPin(GPIOA, LED2);
//  LL_GPIO_SetOutputPin(GPIOA, LED1|LED2);

  /* Config LED1 pin */
//  GPIO_InitStruct.Pin = LED1;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
//  LL_GPIO_Init(LED_GPIO, &GPIO_InitStruct);
//  LL_GPIO_ResetOutputPin(LED_GPIO, LED1);

  /* Config LED2 pin */
  GPIO_InitStruct.Pin = LED2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(LED_GPIO, LED2);

}

/*
 * @brief Micros() function
 * @retval Count
 */
uint32_t micros(){
	#ifdef TIME_TIMER_32BIT
		return TIME_TIMER->CNT;
	#else
		uint16_t TIMCounter = TIME_TIMER->CNT;
		uint16_t OVtest = TXOV;
		if(TIME_TIMER->CNT >= TIMCounter) return (uint32_t)(OVtest<<16)+TIMCounter;
		else return (uint32_t)(((uint16_t)OVtest+1)<<16)+TIME_TIMER->CNT;
	#endif
}

/*
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/*
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
