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
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef       TimHandle;
TIM_Encoder_InitTypeDef sEncoderConfig;
uint32_t                uwDirection;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);

/**
  * @brief   Main program
  * @retval  int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  
  /* Configure Systemclock */
  APP_SystemClockConfig(); 
  
  /* Initialize USART */
  DEBUG_USART_Config();

  TimHandle.Instance = TIM1;                                           /* Select TIM1 */
  TimHandle.Init.Period            = 12800 - 1;                        /* Auto reload value：12800-1 */
  TimHandle.Init.Prescaler         = 1000 - 1;                         /* Prescaler:1000-1 */
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;           /* Clock division: tDTS=tCK_INT */
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;               /* CounterMode:Up */
  TimHandle.Init.RepetitionCounter = 1 - 1;                            /* repetition counter value:1-1 */
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;   /* TIM1_ARR register is not buffered */

  sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;            /* Encoder mode 3 */
  sEncoderConfig.IC1Polarity        = TIM_ICPOLARITY_RISING;           /* Capture triggered by rising edge */
  sEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;        /* CC1 channle is configured as input */
  sEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1;                  /* Capture input not divide */
  sEncoderConfig.IC1Filter          = 0;                               /* Input not filtered */

  sEncoderConfig.IC2Polarity        = TIM_ICPOLARITY_RISING;           /* Capture triggered by rising edge */
  sEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;        /* CC2 channle is configured as input */
  sEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1;                  /* Capture input not divide */
  sEncoderConfig.IC2Filter          = 0;                               /* Input not filtered */

  /* Initializes the TIM Encoder Interface */
  if (HAL_TIM_Encoder_Init(&TimHandle, &sEncoderConfig) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Encoder mode on for all channels */
  HAL_TIM_Encoder_Start(&TimHandle, TIM_CHANNEL_ALL);

  /* Infinite loop */
  while (1)
  {
    /* Indicates whether or not the TIM Counter is used as downcounter */
    uwDirection = __HAL_TIM_IS_TIM_COUNTING_DOWN(&TimHandle);
    
    /* Print count direction 1: down, 0: up */
    printf("Direction = 0x%x\r\n ", (unsigned int)uwDirection);
    
    /* Print the current count value */
    printf("CNT = %u\r\n",(unsigned int)__HAL_TIM_GET_COUNTER(&TimHandle));
  }
}

/**
  * @brief   Configure system clock
  * @param   None
  * @retval  None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI \
                                   | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;  /* Configure HSE、HSI、LSI、LSE */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                                             /* HSI ON */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                                             /* HSI not divided */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;                     /* Configure HSI to 8Mhz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                                            /* HSE OFF */
  /* RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz; */                                  /* HSE clock range 16~32MHz */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                                            /* LSI OFF */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                                            /* LSE OFF */
  /* RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM; */                             /* LSE medium drive capability */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;                                        /* PLL OFF */
  /* RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_NONE; */
  /* RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2; */
  /* Initialize RCC Oscillators */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;/* Configure SYSCLK、HCLK、PCLK */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS;                                     /* Select HSISYS as system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                                            /* SYSCLK not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                                             /* HCLK not divided */
  /* Configure Clock source */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
