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
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SetSysClock(uint32_t SYSCLKSource);
static void APP_SystemClockConfig(void);

/**
  * @brief   Main program
  * @retval  int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();
  
  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

  /* Suspend SysTick interrupt */
  HAL_SuspendTick();
  
  /* Configure the system clock to default HSI 8MHz, and then switch to LSI clock */
  APP_SystemClockConfig();

  /* Configure PA08 pin as MCO1 function to output the system clock */
  HAL_RCC_MCOConfig(0, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

  /* Wait for the button to be pressed to enable HSE */
  while (BSP_PB_GetState(BUTTON_KEY) == 1)
  {
  }

  /* Switch the system clock to HSE external crystal clock */
  APP_SetSysClock(RCC_SYSCLKSOURCE_HSE);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure clock source: HSE/HSI/LSE/LSI */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                                                    /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                                                    /* HSI not divided */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;                            /* Configure HSI output clock as 8MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                                                    /* Enable HSE */
  RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;                                               /* HSE crystal frequency range 16M~32M */
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;                                                    /* Enable LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                                                   /* Disable LSE */
  RCC_OscInitStruct.LSEDriver = RCC_ECSCR_LSE_DRIVER_1;                                       /* Default LSE  drive capability */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;                                               /* Disable PLL */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;                                        /* Select PLL source as HSE */
  /*RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;*/
  /* Initialize RCC oscillator */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Initialize CPU, AHB, and APB bus clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* RCC system clock types */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_LSI;                                         /* SYSCLK source is LSI */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                                             /* AHB clock not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                                              /* APB clock not divided */
  /* Initialize RCC system clock (FLASH_LATENCY_0=24M or below; FLASH_LATENCY_1=48M) */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief   Set system clock
  * @param   SYSCLKSource：System clock source
  *            @arg RCC_SYSCLKSOURCE_LSI: LSI as system clock source
  *            @arg RCC_SYSCLKSOURCE_LSE: LSE as system clock source
  *            @arg RCC_SYSCLKSOURCE_HSE: HSE as system clock source
  *            @arg RCC_SYSCLKSOURCE_HSISYS: HSI as system clock source
  *            @arg RCC_SYSCLKSOURCE_PLLCLK: PLL as system clock source
  * @retval  None
  */
static void APP_SetSysClock(uint32_t SYSCLKSource)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* RCC system clock types */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = SYSCLKSource;                            /* Select system clock source */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                        /* AHB clock not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;                         /* APB clock divided by 2 */

  /* Initialize RCC system clock (FLASH_LATENCY_0=24M or below; FLASH_LATENCY_1=48M) */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief   This function is executed in case of error occurrence.
  * @param   None
  * @retval  None
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
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
