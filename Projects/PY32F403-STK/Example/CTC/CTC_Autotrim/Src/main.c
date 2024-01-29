/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
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
  
  /* Configure PA08 pin as MCO function, output HSI48M */
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48M, RCC_MCODIV_1);
  
  /* System clock configuration */
  APP_SystemClockConfig(); 

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
  HAL_CTC_Start_IT(&CTChadle);

  /* infinite loop */
  while (1)
  {
  }
}

/**
  * @brief   System clock configuration function
  * @param   None
  * @retval  None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef  OscInitstruct = {0};
  RCC_ClkInitTypeDef  ClkInitstruct = {0};
  
  OscInitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE | 
                                  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSI48M;
  OscInitstruct.HSEState        = RCC_HSE_OFF;                              /* Close HSE */
/* OscInitstruct.HSEFreq         = RCC_HSE_16_32MHz; */                     /* Choose HSE frequency of 16-32MHz */
/* OscInitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;*/                   /* HSE as PLL clock source time division frequency value */
  OscInitstruct.HSI48MState     = RCC_HSI48M_ON;                            /* Enable HSI48M */
  OscInitstruct.HSIState        = RCC_HSI_ON;                               /* Enable HSI */
  OscInitstruct.LSEState        = RCC_LSE_ON;                               /* Enable LSE */
  OscInitstruct.LSEDriver       = RCC_LSEDRIVE_HIGH;                        /* LSE drive capability level: high */
  OscInitstruct.LSIState        = RCC_LSI_OFF;                              /* Close LSI */
  OscInitstruct.PLL.PLLState    = RCC_PLL_OFF;                              /* Close PLL */
/* OscInitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE; */                    /* PLL clock source selection HSE */
/* OscInitstruct.PLL.PLLMUL      = RCC_PLL_MUL6; */                        /* PLL clock source 6-fold frequency */
  /* Configure oscillator */
  if(HAL_RCC_OscConfig(&OscInitstruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  ClkInitstruct.ClockType       = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  ClkInitstruct.SYSCLKSource    = RCC_SYSCLKSOURCE_HSI;                 /* System clock selection HSI */
  ClkInitstruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;                      /* AHB clock 1 division */
  ClkInitstruct.APB1CLKDivider  = RCC_HCLK_DIV1;                        /* APB1 clock 1 division */
  ClkInitstruct.APB2CLKDivider  = RCC_HCLK_DIV2;                        /* APB2 clock 2 division */
  /* Configure Clock */
  if(HAL_RCC_ClockConfig(&ClkInitstruct, FLASH_LATENCY_0) != HAL_OK)
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
