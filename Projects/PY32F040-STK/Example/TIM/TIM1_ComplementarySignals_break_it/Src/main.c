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
#define  PULSE1_VALUE       (uint32_t)(400)/* Capture Compare 1 Value  */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef               TimHandle;
TIM_OC_InitTypeDef              sPWMConfig;
TIM_BreakDeadTimeConfigTypeDef  sBreakConfig;

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

  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);

  TimHandle.Instance = TIM1;                                           /* Select TIM1 */
  TimHandle.Init.Period            = 800 - 1;                          /* Auto reload value： */
  TimHandle.Init.Prescaler         = 1000 - 1;                         /* Prescaler:1000-1 */
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;           /* Clock division: tDTS=tCK_INT */
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;               /* CounterMode:Up */
  TimHandle.Init.RepetitionCounter = 1 - 1;                            /* repetition counter value:1-1 */
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;   /* TIM1_ARR register is not buffered */
  /* Initialize PWM output */
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  sPWMConfig.OCMode       = TIM_OCMODE_PWM1;                           /* Set as PWM1 mode */
  sPWMConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;                       /* Compare output polarity: HIGH */
  sPWMConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                      /* Compare complementary output polarity: HIGH */
  sPWMConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;                     /* Output Idle state: LOW */
  sPWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                    /* Complementary output Idle state: LOW */
  sPWMConfig.OCFastMode   = TIM_OCFAST_DISABLE;                        /* Output Compare fast disable */
  sPWMConfig.Pulse = PULSE1_VALUE;                                     /* Duty cycle of channel 1 */
  /* Configure Channel 1 */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  sBreakConfig.BreakState       = TIM_BREAK_ENABLE;                    /* Break input BRK is enabled */
  sBreakConfig.DeadTime         = 160;                                 /* Set the dead time */
  sBreakConfig.OffStateRunMode  = TIM_OSSR_ENABLE;                     /* Off state in run mode: output enable */
  sBreakConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;                     /* IDLE state in run mode: output enable */
  sBreakConfig.LockLevel        = TIM_LOCKLEVEL_OFF;                   /* LOCK off */
  sBreakConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;               /* Break input BRK is active LOW */
  sBreakConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;          /* Automatic output enable */
  sBreakConfig.BreakFilter      = 0;
  /* Configures the Break feature, dead time */
  if (HAL_TIMEx_ConfigBreakDeadTime(&TimHandle, &sBreakConfig) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Channel 1 outputs PWM */
  if (HAL_TIM_PWM_Start_IT(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  /* Channel 1N outputs PWM */
  if (HAL_TIMEx_PWMN_Start_IT(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Infinite loop */
  while (1)
  {

  }
}

/**
  * @brief  TIM brake interrupt callback
  * @param  htim：TIM handle
  * @retval None
  */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  BSP_LED_Toggle(LED_GREEN);
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
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;  /* Configure HSI to 8Mhz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                         /* HSE OFF */
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* LSI OFF */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                         /* LSE OFF */
  /*RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;                     /* PLL OFF */
  /*RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_NONE;*/
  /*RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;*/
  /* Initialize RCC Oscillators */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Clock source configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* Choose to configure HCLK,SYSCLK,PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS; /* Select HSISYS as system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* SYSCLK not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* HCLK not divided */
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
