/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* include current source file only once in a single compilation. */
#pragma once

/* defines done in makefile */
//#define STM32F407xx
//#define USE_FULL_LL_DRIVER
//#define DEBUG

/* includes */
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* predefine functions */
void SystemClock_Config(void); // Startup
void SystemClock_Config_Extra(void); // for micros() funtion
void SystemTimerConfig(void); // TIM5
void TIM5_IRQHandler(void); // TIM5
void Error_Handler(void);


#ifndef NVIC_PRIORITYGROUP_0
	#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
																	 4 bits for subpriority */
	#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
																	 3 bits for subpriority */
	#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
																	 2 bits for subpriority */
	#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
																	 1 bit  for subpriority */
	#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
															  	  	   bit  for subpriority */
#endif

/* user defines */
#define LED_GPIO GPIOA
#define LED1 LL_GPIO_PIN_6
#define LED2 LL_GPIO_PIN_7

#define OSD_COMP_PRIORITY_GROUP 0
#define OSD_COMP_PRIORITY_SUBGROUP 0

#define SYS_TIMER_PRIORITY_GROUP 1
#define SYS_TIMER_PRIORITY_SUBGROUP 0

#define OSD_TIMER_PRIORITY_GROUP 1
#define OSD_TIMER_PRIORITY_SUBGROUP 0

#define ESC_DMA_PRIORITY_GROUP 2
#define ESC_DMA_PRIORITY_SUBGROUP 0

#define UART_PRIORITY_GROUP 3
#define UART_PRIORITY_SUBGROUP 0

#define SOFT_SERIAL_DMA_PRIORITY_GROUP 4
#define SOFT_SERIAL_DMA_PRIORITY_SUBGROUP 0

#define SOFT_SERIAL_TIMER_PRIORITY_GROUP 4
#define SOFT_SERIAL_TIMER_PRIORITY_SUBGROUP 0

#define SOFT_SERIAL_EXI_PRIORITY_GROUP 4
#define SOFT_SERIAL_EXI_PRIORITY_SUBGROUP 0

#define OSD_MDMA_PRIORITY_GROUP 5
#define OSD_MDMA_PRIORITY_SUBGROUP 0

#define RGBLED_PRIORITY_GROUP 6
#define RGBLED_PRIORITY_SUBGROUP 1

#define USB_PRIORITY_GROUP 6
#define USB_PRIORITY_SUBGROUP 0

#define TIME_TIMER TIM5
#define TIME_TIMER_32BIT


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
