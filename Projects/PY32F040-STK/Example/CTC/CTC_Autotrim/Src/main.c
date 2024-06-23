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
CTC_HandleTypeDef CTChadle;
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);

/**
  * @brief   Main program.
  * @retval  int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();
  
  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);
  
  /* Configure PA08 pin as MCO function, output PLL */
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
  
  /* System clock configuration */
  APP_SystemClockConfig(); 
  
  /* Turn on CTC clock */
  __HAL_RCC_CTC_CLK_ENABLE();

  /* Initialize CTC configuration */
  CTChadle.Instance             = CTC;                          /* CTC */
  CTChadle.Init.AutoTrim        = CTC_AUTO_TRIM_ENABLE;         /* Enable automatic calibration */
  CTChadle.Init.RefCLKSource    = CTC_REF_CLOCK_SOURCE_LSE;     /* Reference clock source LSE */
  CTChadle.Init.RefCLKDivider   = CTC_REF_CLOCK_DIV1;           /* Reference clock division 1 division */
  CTChadle.Init.RefCLKPolarity  = CTC_REF_CLOCK_POLARITY_RISING;/* Effective rising edge of reference clock */  
  CTChadle.Init.ReloadValue     = 1465-1;                       /* Counter reload value */
  CTChadle.Init.LimitValue      = 1;                            /* Clock calibration time base limit */
  HAL_CTC_Init(&CTChadle);
  
  /* Start calibration */
  HAL_CTC_Start(&CTChadle);

  /* infinite loop */
  while (1)
  {
    if (__HAL_CTC_GET_FLAG(&CTChadle, CTC_FLAG_CKOK) != 0)
    {
      /* Calibration successful, LED light on */
      BSP_LED_On(LED_GREEN);
    }
    else
    {
      /* Calibration not successful, LED light in off state */
      BSP_LED_Off(LED_GREEN);
    }
  }
}

/**
  * @brief   System clock configuration function
  * @param   None
  * @retval  None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;  /* Select RCC oscillator HSE, HSI, LSE */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                                                     /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                                                     /* 1 division frequency */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_16MHz;                            /* Configure the HSI output clock to 16MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                                                     /* Start HSE */
  RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;                                                /* HSE crystal oscillator operating frequency 16M~32M */
  /*RCC_OscInitStruct.LSIState = RCC_LSI_OFF;*/                                                /* Close LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;                                                     /* Enable LSE */
  RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_HIGH;                                             /* LSE high driver capability */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                                                 /* Enable PLL */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;                                         /* Select PLL source as HSI */
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;                                                 /* PLL triple frequency */
  /* Initialize RCC oscillator */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Initialize CPU, AHB, APB bus clock */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;/* RCC system clock type */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;                                        /* The source selection for SYSCLK is HSE */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                                            /* AHB clock does not divide frequency */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;                                             /* APB clock 2 division */
  /* Initialize the RCC system clock (FLASH_LATENCY_0=below 24M; FLASH_LATENCY_1=48M) */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
