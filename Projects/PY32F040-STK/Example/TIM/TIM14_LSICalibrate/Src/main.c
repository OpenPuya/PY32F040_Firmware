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
uint32_t             Temp_LSItrim;                /* LSI clock trimming */
uint32_t             Capture_last_cnt;            /* The number of capture when the last update interrupt occurred */ 
uint32_t             Capture_cnt;                 /* The current number of capture */ 
TIM_HandleTypeDef    TimHandle;
TIM_IC_InitTypeDef   sICConfig;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define RCC_LSI_CALIBRATIONVALUE_ADJUST(__LSICALIBRATIONVALUE__) \
                  MODIFY_REG(RCC->ICSCR, RCC_ICSCR_LSI_TRIM_Msk, (uint32_t)(__LSICALIBRATIONVALUE__) << RCC_ICSCR_LSI_TRIM_Pos)
#define RCC_GET_LSI_TRIM_VALUE()  ((uint32_t)(READ_BIT(RCC->ICSCR,RCC_ICSCR_LSI_TRIM)))

/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void TIM14Config(void);
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

  /* Initialize led */  
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN);
    
  /* Configure PA08 pin as MCO1 function to output the LSI clock */
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSI, RCC_MCODIV_1);

  /* Wait for the button to be pressed to enable LSI */
  while (BSP_PB_GetState(BUTTON_KEY) == 1)
  {
  }
  /* Configure system clock,and enable LSI */
  APP_SystemClockConfig();
  
  /* Initialize TIM14 */
  TIM14Config();
  
  while (1)
  {
  }
}

/**
  * @brief   Configure TIM14
  * @param   None
  * @retval  None
  */
static void TIM14Config(void)
{
  TimHandle.Instance = TIM14;                                          /* Select TIM14 */
  TimHandle.Init.Period            = 10001 - 1;                        /* Auto reload value：10001-1 */
  TimHandle.Init.Prescaler         = 100 - 1;                          /* Prescaler:100-1 */
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;           /* Clock division: tDTS=tCK_INT */
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;               /* CounterMode:Up */
  TimHandle.Init.RepetitionCounter = 1 - 1;                            /* repetition counter value:1-1 */
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;   /* TIM14_ARR register is not buffered */
  /* Initialize TIM14 */
  if (HAL_TIM_IC_Init(&TimHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  /* TIM14 TI1 is connected to MCO */
  if (HAL_TIMEx_RemapConfig(&TimHandle,TIM_TIM14_MCO) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;                       /* Capture triggered by rising edge */
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;                    /* CC1 channel is configured as input */
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;                              /* Capture input not divide */
  sICConfig.ICFilter    = 0;                                           /* Input not filtered */
  /* Initializes the TIM Input Capture Channel 1 */
  if (HAL_TIM_IC_ConfigChannel(&TimHandle, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  /* Enable update interrupt */
  __HAL_TIM_ENABLE_IT(&TimHandle, TIM_IT_UPDATE);
  
  /* Starts the TIM Input Capture measurement */
  if (HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_1) != HAL_OK)                    
  {
    APP_ErrorHandler();
  }
  
}

/**
  * @brief  TIM captures interrupt callback
  * @param  htim：TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{  
  Capture_cnt++;
}

/**
  * @brief  TIM update interrupt callback
  * @param  htim：TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t Temp_delta = 0;
  /* Count the number of times a capture occurs between update interrupts */
  Temp_delta = Capture_cnt - Capture_last_cnt;
  
  /* Get current trimming of LSI */
  Temp_LSItrim = RCC_GET_LSI_TRIM_VALUE();
  
  /* Adjust LSI trimming value, target frequency is 32.768KHz  */
  if (Temp_delta < 1023)
  {
    Temp_LSItrim = (Temp_LSItrim >> RCC_ICSCR_LSI_TRIM_Pos)+1;
    RCC_LSI_CALIBRATIONVALUE_ADJUST(Temp_LSItrim);
  }
  else if (Temp_delta > 1025)
  {
    Temp_LSItrim = (Temp_LSItrim >> RCC_ICSCR_LSI_TRIM_Pos)-1;
    RCC_LSI_CALIBRATIONVALUE_ADJUST(Temp_LSItrim);
  }
  else
  {
    /* Calibrate ok,STOP TIM14 */
    HAL_TIM_Base_Stop_IT(htim);
    HAL_TIM_IC_Stop_IT(htim,TIM_CHANNEL_1);
    
    /* LED on */
    BSP_LED_On(LED_GREEN);
  }    
  Capture_last_cnt = Capture_cnt;
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
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                                                      /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                                                      /* No HSI division */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_16MHz;                             /* Configure HSI output clock as 16MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                                                     /* Disable HSE */
  RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;                                                 /* HSE crystal frequency range 16M~32M */
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;                                                      /* Enable LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                                                     /* Disable LSE */
  RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;                                            /* LSE medium drive capability */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                                                  /* Enable PLL */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;                                          /* Set HSI as PLL entry clock */
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;                                                  /* SYSCLK = HSISYS*2 */
  /* Initialize RCC oscillator */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Initialize CPU, AHB, and APB bus clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* RCC system clock types */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;                                      /* SYSCLK source is PLL */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                                             /* AHB clock not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                                              /* APB clock not divided */
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
