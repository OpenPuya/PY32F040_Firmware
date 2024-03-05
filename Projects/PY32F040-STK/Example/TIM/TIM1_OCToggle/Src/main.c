/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define  PERIOD_VALUE       (uint32_t)(50 - 1)
#define  PULSE1_VALUE       10-1
#define  PULSE2_VALUE       20-1
#define  PULSE3_VALUE       30-1
#define  PULSE4_VALUE       40-1
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef    TimHandle;
TIM_OC_InitTypeDef sConfig;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  
  /* Configure Systemclock */
  APP_SystemClockConfig(); 

  TimHandle.Instance = TIM1;                                                  /* Select TIM1 */
  TimHandle.Init.Period            = 40 - 1;                                  /* Auto reload value： */
  TimHandle.Init.Prescaler         = 1 - 1;                                   /* Prescaler:1-1 */
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;                  /* Clock division: tDTS=tCK_INT */
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;                      /* CounterMode:Up */
  TimHandle.Init.RepetitionCounter = 1 - 1;                                   /* repetition counter value:1-1 */
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;          /* TIM1_ARR register is not buffered */
   /* Initializes the TIM Time base Unit */
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  sConfig.OCMode       = TIM_OCMODE_TOGGLE;                                   /* OC1REF toggles when TIMx_CNT=TIMx_CCR1 */
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;                                 /* OC channel active high */
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;                                  /* Output Compare fast disable */
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                /* OCN channel active high */
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              /* OC1N channel idle state is low level */
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;                               /* OC1 channel idle state is low level */
  
  sConfig.Pulse = PULSE1_VALUE;                                               /* TIM1_CCR1 value:10 */
  if (HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)/* Configure OC1 channel */
  {
    APP_ErrorHandler();
  }
  sConfig.Pulse = PULSE2_VALUE;                                               /* TIM1_CCR2 value:20 */
  if (HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)/* Configure OC2 channel */
  {
    APP_ErrorHandler();
  }
  sConfig.Pulse = PULSE3_VALUE;                                               /* TIM1_CCR3 value:30 */
  if (HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)/* Configure OC3 channel */
  {
    APP_ErrorHandler();
  }
  sConfig.Pulse = PULSE4_VALUE;                                               /* TIM1_CCR4 value:40 */
  if (HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)/* Configure OC4 channel */
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)                  /* Start OC1 channel output */
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)                  /* Start OC2 channel output */
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)                  /* Start OC3 channel output */
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)                  /* Start OC4 channel output */
  {
    APP_ErrorHandler();
  }

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Configure system clock
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* RCC Oscillators configuration */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE; /* Select oscillators HSE,HSI,LSI,LSE */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                          /* HSI ON */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                          /* HSI clock is not divided */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_16MHz; /* Configure HSI to 16Mhz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                         /* HSE OFF */
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* LSI OFF */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                         /* LSE OFF */
  /*RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                      /* PLL ON */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;              /* HSI clock selected as PLL entry clock source */
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  /* Initialize RCC Oscillators */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Clock source configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* Choose to configure HCLK,SYSCLK,PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; /* Select PLL as system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        /* SYSCLK not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         /* HCLK not divided */
  /* Configure Clock source */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  Error handling function
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  /* Infinite loop */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file：Pointer to the source file name
  * @param  line：assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add His own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
