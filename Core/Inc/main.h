/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define _TIM1K_ARR (16800-1)
#define _CAN_PSC _CAN_500_PSC
#define _TIM1K_PSC (10-1)
#define _HRTIM_PERIOD 10500
#define _CAN_SEG1 _CAN_500_SEG1
#define _LAMP_TIM_ARR 8400-1
#define _CAN_SEG2 _CAN_500_SEG2
#define _ADC_TIM _ADC_TIM_24000
#define _CAN_500_PSC 21
#define _ADC_TIM_24000 (7000-1)
#define _LED3_TIM_ARR 8400-1
#define _TIM10K_SPI_START 3000
#define _HRTIM_DT 300
#define _CAN_500_SEG1 13
#define _ADC_TIM_12800 (13125-1)
#define _TIM10K_SPI_IN_E 15000
#define _CAN_500_SEG2 2
#define _TIM10K_SPI_ARR (16800-1)
#define _TIM10K_SPI_SOEN 1000
#define _TIM_SPI_INOUT_20KHZ_ARR (20000-1)
#define _TIM_SPI_INOUT_20KHZ_INE 1000
#define _TIM_SPI_INOUT_20KHZ_SOEN (18000)
#define _TIM_SPI_INOUT_20KHZ_START 1600
#define NRST_Pin LL_GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
#define COMP_I1_Pin LL_GPIO_PIN_1
#define COMP_I1_GPIO_Port GPIOC
#define IA_A1_Pin LL_GPIO_PIN_0
#define IA_A1_GPIO_Port GPIOA
#define IA_A3_Pin LL_GPIO_PIN_1
#define IA_A3_GPIO_Port GPIOA
#define IA_A7_Pin LL_GPIO_PIN_2
#define IA_A7_GPIO_Port GPIOA
#define IA_A2_Pin LL_GPIO_PIN_3
#define IA_A2_GPIO_Port GPIOA
#define IA_A9_Pin LL_GPIO_PIN_6
#define IA_A9_GPIO_Port GPIOA
#define COMP_I2_Pin LL_GPIO_PIN_7
#define COMP_I2_GPIO_Port GPIOA
#define IA_A4_Pin LL_GPIO_PIN_4
#define IA_A4_GPIO_Port GPIOC
#define COMP_I3_Pin LL_GPIO_PIN_0
#define COMP_I3_GPIO_Port GPIOB
#define COMP_I4_Pin LL_GPIO_PIN_1
#define COMP_I4_GPIO_Port GPIOB
#define IA_A10_Pin LL_GPIO_PIN_7
#define IA_A10_GPIO_Port GPIOE
#define IA_A8_Pin LL_GPIO_PIN_8
#define IA_A8_GPIO_Port GPIOE
#define IA_A6_Pin LL_GPIO_PIN_9
#define IA_A6_GPIO_Port GPIOE
#define IA_TEMP1_Pin LL_GPIO_PIN_11
#define IA_TEMP1_GPIO_Port GPIOE
#define IA_TEMP2_Pin LL_GPIO_PIN_12
#define IA_TEMP2_GPIO_Port GPIOE
#define IA_A5_Pin LL_GPIO_PIN_13
#define IA_A5_GPIO_Port GPIOE
#define IA_A11_Pin LL_GPIO_PIN_14
#define IA_A11_GPIO_Port GPIOE
#define IA_A12_Pin LL_GPIO_PIN_15
#define IA_A12_GPIO_Port GPIOE
#define O_D_Led_Link_Pin LL_GPIO_PIN_10
#define O_D_Led_Link_GPIO_Port GPIOB
#define O_D_Led_Ready_Pin LL_GPIO_PIN_11
#define O_D_Led_Ready_GPIO_Port GPIOB
#define PWM_2H_Pin LL_GPIO_PIN_12
#define PWM_2H_GPIO_Port GPIOB
#define PWM_2L_Pin LL_GPIO_PIN_13
#define PWM_2L_GPIO_Port GPIOB
#define PWM_3H_Pin LL_GPIO_PIN_14
#define PWM_3H_GPIO_Port GPIOB
#define PWM_3L_Pin LL_GPIO_PIN_15
#define PWM_3L_GPIO_Port GPIOB
#define O_D_Led_Fault_Pin LL_GPIO_PIN_8
#define O_D_Led_Fault_GPIO_Port GPIOD
#define COMP_I6_Pin LL_GPIO_PIN_11
#define COMP_I6_GPIO_Port GPIOD
#define COMP_I5_Pin LL_GPIO_PIN_12
#define COMP_I5_GPIO_Port GPIOD
#define COMP_U3_Pin LL_GPIO_PIN_14
#define COMP_U3_GPIO_Port GPIOD
#define PWM_1H_Pin LL_GPIO_PIN_6
#define PWM_1H_GPIO_Port GPIOC
#define PWM_1L_Pin LL_GPIO_PIN_7
#define PWM_1L_GPIO_Port GPIOC
#define PWM_5H_Pin LL_GPIO_PIN_8
#define PWM_5H_GPIO_Port GPIOC
#define PWM_5L_Pin LL_GPIO_PIN_9
#define PWM_5L_GPIO_Port GPIOC
#define PWM_6H_Pin LL_GPIO_PIN_8
#define PWM_6H_GPIO_Port GPIOA
#define PWM_6L_Pin LL_GPIO_PIN_9
#define PWM_6L_GPIO_Port GPIOA
#define PWM_4H_Pin LL_GPIO_PIN_10
#define PWM_4H_GPIO_Port GPIOA
#define PWM_4L_Pin LL_GPIO_PIN_11
#define PWM_4L_GPIO_Port GPIOA
#define O_T_SPI_IN_E_Pin LL_GPIO_PIN_4
#define O_T_SPI_IN_E_GPIO_Port GPIOD
#define O_T_SPI_SOEN_Pin LL_GPIO_PIN_6
#define O_T_SPI_SOEN_GPIO_Port GPIOD
#define O_D_SPI_SR_Pin LL_GPIO_PIN_7
#define O_D_SPI_SR_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define MAIN_INV_SET_PWM_PERIOD(ARR)                   \
LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, ARR); \
LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_B, ARR); \
LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_C, ARR); \
LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_D, ARR); \
LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_E, ARR); \
LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_F, ARR);

#define MAIN_INV_START_CNT()  LL_HRTIM_TIM_CounterEnable(HRTIM1,	\
														  LL_HRTIM_TIMER_A \
														+ LL_HRTIM_TIMER_B \
														+ LL_HRTIM_TIMER_C \
														+ LL_HRTIM_TIMER_D \
														+ LL_HRTIM_TIMER_E \
														+ LL_HRTIM_TIMER_F \
														);

/* Enable HRTIM's outputs TD1 and TD2, and start Timer D and Master */
#define MAIN_INV_ENABLE_OUTS()	LL_HRTIM_EnableOutput(HRTIM1,						\
		  	  	  	  	  	  	  	LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | \
		  	  	  	  	  	  	  	LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TB2 | \
									LL_HRTIM_OUTPUT_TC1 | LL_HRTIM_OUTPUT_TC2 | \
									LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2 | \
									LL_HRTIM_OUTPUT_TE1 | LL_HRTIM_OUTPUT_TE2 | \
									LL_HRTIM_OUTPUT_TF1 | LL_HRTIM_OUTPUT_TF2	\
	  	  	  	  	  	  	  	  );

#define MAIN_INV_DISABLE_OUTS()	LL_HRTIM_DisableOutput(HRTIM1,						\
		  	  	  	  	  	  	  	LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | \
		  	  	  	  	  	  	  	LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TB2 | \
									LL_HRTIM_OUTPUT_TC1 | LL_HRTIM_OUTPUT_TC2 | \
									LL_HRTIM_OUTPUT_TD1 | LL_HRTIM_OUTPUT_TD2 | \
									LL_HRTIM_OUTPUT_TE1 | LL_HRTIM_OUTPUT_TE2 | \
									LL_HRTIM_OUTPUT_TF1 | LL_HRTIM_OUTPUT_TF2	\
	  	  	  	  	  	  	  	  );

#define MAIN_INV_SET_PWM(TIM,PWM)	(LL_HRTIM_TIM_SetCompare1(HRTIM1, TIM, PWM))

//В соответствии со схемой
#define MAIN_INV_TIM_PWM_1	(LL_HRTIM_TIMER_F)
#define MAIN_INV_TIM_PWM_2	(LL_HRTIM_TIMER_C)
#define MAIN_INV_TIM_PWM_3	(LL_HRTIM_TIMER_D)
#define MAIN_INV_TIM_PWM_4	(LL_HRTIM_TIMER_B)
#define MAIN_INV_TIM_PWM_5	(LL_HRTIM_TIMER_E)
#define MAIN_INV_TIM_PWM_6	(LL_HRTIM_TIMER_A)

#define MAIN_INV_SET_PWM_1(PWM)	MAIN_INV_SET_PWM(MAIN_INV_TIM_PWM_1, PWM)
#define MAIN_INV_SET_PWM_2(PWM)	MAIN_INV_SET_PWM(MAIN_INV_TIM_PWM_2, PWM)
#define MAIN_INV_SET_PWM_3(PWM)	MAIN_INV_SET_PWM(MAIN_INV_TIM_PWM_3, PWM)
#define MAIN_INV_SET_PWM_4(PWM)	MAIN_INV_SET_PWM(MAIN_INV_TIM_PWM_4, PWM)
#define MAIN_INV_SET_PWM_5(PWM)	MAIN_INV_SET_PWM(MAIN_INV_TIM_PWM_5, PWM)
#define MAIN_INV_SET_PWM_6(PWM)	MAIN_INV_SET_PWM(MAIN_INV_TIM_PWM_6, PWM)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
