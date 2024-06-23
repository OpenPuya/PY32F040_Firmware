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
TIM_HandleTypeDef    TimHandle;
TIM_OC_InitTypeDef   sConfig1,sConfig2,sConfig3;
TIM_BreakDeadTimeConfigTypeDef sBreakConfig;
uint32_t temp;
__IO uint32_t uwStep = 0;

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
  
  /* TIM configuration */
  TimHandle.Instance               = TIM1;                            /* Select TIM1 */
  TimHandle.Init.Period            = 1000 - 1;                        /* Auto reload value：1000-1 */
  TimHandle.Init.Prescaler         = 1 - 1;                           /* Prescaler:1-1 */
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;          /* Clock division: tDTS=tCK_INT */
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;              /* CounterMode:Up */
  TimHandle.Init.RepetitionCounter = 1 - 1;                           /* repetition counter value:1-1 */
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  /* TIM1_ARR register is not buffered */
  /* Initialize TIM1 */
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* PWM channel configuration */
  sConfig1.OCMode       = TIM_OCMODE_TIMING;                          /* OCMode：Frozen */
  sConfig1.OCPolarity   = TIM_OCPOLARITY_HIGH;                        /* Compare output polarity: HIGH */
  sConfig1.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                       /* Compare complementary output polarity: HIGH */
  sConfig1.OCIdleState  = TIM_OCIDLESTATE_RESET;                      /* Output Idle state: LOW */
  sConfig1.OCNIdleState = TIM_OCNIDLESTATE_RESET;                     /* Complementary output Idle state: LOW */
  sConfig1.OCFastMode   = TIM_OCFAST_DISABLE;                         /* Output Compare fast disable */
  /* Set the pulse value for channel 1 : 500-1 */
  sConfig1.Pulse        = 500 - 1;
  /* Configure TIM1_CH1 */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig1, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  sConfig2 = sConfig1;
  /* Set the pulse value for channel 1 : 250-1 */
  sConfig2.Pulse        = 250 - 1;
  /* Configure TIM1_CH2 */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig2, TIM_CHANNEL_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  sConfig3 = sConfig1;
  /* Set the pulse value for channel 1 : 125-1 */
  sConfig3.Pulse        = 125 - 1;
  /* Configure TIM1_CH3 */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig3, TIM_CHANNEL_3) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Configures the Break feature, dead time */
  sBreakConfig.BreakState       = TIM_BREAK_ENABLE;                      /* Break input BRK is enabled */
  sBreakConfig.DeadTime         = 1;                                     /* Dead time: 1 */
  sBreakConfig.OffStateRunMode  = TIM_OSSR_ENABLE;                       /* Off state in run mode: output enable */
  sBreakConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;                       /* IDLE state in run mode: output enable */
  sBreakConfig.LockLevel        = TIM_LOCKLEVEL_OFF;                     /* LOCK off */
  sBreakConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;                /* Break input BRK is active HIGH */
  sBreakConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;            /* Automatic output enable */
  sBreakConfig.BreakFilter      = 0; 
  /* Configures the Break feature, dead time */
  if (HAL_TIMEx_ConfigBreakDeadTime(&TimHandle, &sBreakConfig) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  /* Configure the TIM commutation event: set COMG bit by software. */
  HAL_TIMEx_ConfigCommutEvent_IT(&TimHandle, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE);
  
  /* Start all channles */
  if (HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief   Commutation event callback
  * @param   htim：TIM handle
  * @retval  None
  */
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim)
{
  if (uwStep == 0)
  {
    /* Step 1 Configuration */

    /* Set channle 1 mode: PWM1 */
    sConfig1.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig1, TIM_CHANNEL_1);
    
    /* Start channel 1 */
    HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
    /* Stop the channel 1N */
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_1);

    /* Set channle 3 mode: PWM1 */
    sConfig3.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig3, TIM_CHANNEL_3);
    
    /* Start channel 3N */
    HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_3);
    /* Stop channle 3 */
    HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3);

    /* Stop channle 2 */
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_2);
    /* Stop channle 2N */
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_2);
    
    uwStep = 1;
  }

  else if (uwStep == 1)
  {
    /* Step 2 Configuration */
    
    /* Set channle 2 mode: PWM1 */
    sConfig2.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig2, TIM_CHANNEL_2);
    
    /* Start channel 2N */
    HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_2);
    
    /* Stop channel 3N */
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_3);

    uwStep++;
  }

  else if (uwStep == 2)
  {
    /* Step 3 Configuration */
    
    /* Set channle 3 mode: PWM1 */
    sConfig3.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig3, TIM_CHANNEL_3);
    
    /* Start channel 3 */ 
    HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3);
    
    /* Stop channel 1 */
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_1);

    uwStep++;
  }

  else if (uwStep == 3)
  {
    /* Step 4 Configuration */
    
    /* Stop channel 2N */
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_2);
    
    /* Set channle 1 mode: PWM1 */
    sConfig1.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig1, TIM_CHANNEL_1);
    
    /* Start channel 1N */
    HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_1);
    
    uwStep++;
  }
  else if (uwStep == 4)
  {
    /* Step 5 Configuration */
    
    /* Stop channel 3 */
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_3);

    /* Set channle 2 mode: PWM1 */
    sConfig2.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig2, TIM_CHANNEL_2);
    
    /* Start channel 2 */
    HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2);

    uwStep++;
  }

  else if (uwStep == 5)
  {
    /* Step 6 Configuration */
    
    /* Set channle 3 mode: PWM1 */
    sConfig3.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig3, TIM_CHANNEL_3);
    
    /* Start channel 3N */
    HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_3);

    /* Stop channel channel 1N */
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_1);

    uwStep++;
  }

  else
  {
    /* Step 1 Configuration */
    
    /* Set channle 1 mode: PWM1 */
    sConfig1.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig1, TIM_CHANNEL_1);
    
    /* Start channel 1 */
    HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
    
    /* Stop channel 2 */
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_2);

    uwStep = 1;
  }
}

/**
  * @brief   Configure systemclock
  * @param   None
  * @retval  None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI \
                                   | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;  /* Configure HSE、HSI、LSI、LSE */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                                             /* HSI ON*/
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                                             /* HSI clock is not divided */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;                     /* HSI calibration frequency 8MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                                            /* HSE OFF*/
  /* RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz; */                                  /* HSE clock range 16~32MHz */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                                            /* LSI OFF*/
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                                            /* LSE OFF*/
  /* RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM; */                             /* LSE medium drive capability */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;                                        /* PLL OFF*/
  /* RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_NONE;*/
  /* RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;*/
  /* Initialize the RCC Oscillators */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;/* Configure SYSCLK、HCLK、PCLK */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS;                                     /* Select HSISYS as system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                                            /* SYSCLK not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                                             /* HCLK not divided */
  /* Set clock source */
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
