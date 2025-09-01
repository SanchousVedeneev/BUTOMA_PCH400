/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* --------------- ПАМЯТКА ---------------
Нужно в программе STM32CubeProgrammer во вкладке "Options Byts" во вкладке "User Configuration" убрать галку "nSWBOOT0". 

Байт "nSWBOOT0" отвечает за запуск прошивки. 
Если опция выбрана, то запуск прошивки осуществяется с ножки МК (по аналогии Run/Stop), 
в противороложном случае запуск прошивки осуществяется программно (сразу после перепрошивки).

На сдвиговые регистры поинтерфейсу SPI необходимо подать 6 посылок,
а только потом выдать сигнал S_OEN.
Первые две посылки предназначены для управления релейными выходами,
последующие 4 для индикации на разъемах RJ45
--------------- ПАМЯТКА КОНЕЦ--------------- */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "comp.h"
#include "cordic.h"
#include "dac.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "App.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 FDCAN_TxHeaderTypeDef CAN_TxHeader;
uint8_t buf[5] = {1,2,3,4,5};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CORDIC_Init();
  MX_ADC1_Init();
  MX_ADC5_Init();
  MX_DAC1_Init();
  MX_FDCAN2_Init();
  MX_COMP3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC4_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_COMP4_Init();
  MX_COMP5_Init();
  MX_COMP6_Init();
  MX_COMP7_Init();
  MX_DAC2_Init();
  MX_DAC4_Init();
  MX_TIM17_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_HRTIM1_Init();
  /* USER CODE BEGIN 2 */

  // HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
  // HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);
  // HAL_DAC_Start(&hdac2,DAC_CHANNEL_1);
  // HAL_DAC_Start(&hdac4,DAC_CHANNEL_1);
  // HAL_DAC_Start(&hdac4,DAC_CHANNEL_2);

  // uint16_t val = 4000;
  // HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,val);
  // HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,val);
  // HAL_DAC_SetValue(&hdac2,DAC_CHANNEL_1,DAC_ALIGN_12B_R,val);
  // HAL_DAC_SetValue(&hdac4,DAC_CHANNEL_1,DAC_ALIGN_12B_R,val);
  // HAL_DAC_SetValue(&hdac4,DAC_CHANNEL_2,DAC_ALIGN_12B_R,val);

  // HAL_COMP_Start(&hcomp1);
  // HAL_COMP_Start(&hcomp2);
  // HAL_COMP_Start(&hcomp3);
  // HAL_COMP_Start(&hcomp4);
  // HAL_COMP_Start(&hcomp5);
  // HAL_COMP_Start(&hcomp6);
  // HAL_COMP_Start(&hcomp7);


 /* -------------------------------------- CAN --------------------------------------  */
  /* Configure Rx filter */
  FDCAN_FilterTypeDef CAN_sFilterConfig;
  CAN_sFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN_sFilterConfig.FilterIndex = 0;
  CAN_sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  CAN_sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  CAN_sFilterConfig.FilterID1 = 0x0;
  CAN_sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &CAN_sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &CAN_sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ARB_PROTOCOL_ERROR, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ARB_PROTOCOL_ERROR, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare Tx Header */
  CAN_TxHeader.Identifier = 0x322;
  CAN_TxHeader.IdType = FDCAN_STANDARD_ID;
  CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  CAN_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  CAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  CAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  CAN_TxHeader.MessageMarker = 0;
  /* -------------------------------------- CAN END-------------------------------------------  */

  app_main();
//-------------------------- управление в App.c ----------------------------------------------------------------

  //app_hardware_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp){

  asm("NOP");

    if(hcomp == &hcomp1){

    }else if(hcomp == &hcomp2){

    }else if(hcomp == &hcomp3){
      
    }else if(hcomp == &hcomp4){
      
    }else if(hcomp == &hcomp5){
      
    }else if(hcomp == &hcomp6){
      
    }else if(hcomp == &hcomp7){
      
    }else{
      asm("NOP");
    }

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
