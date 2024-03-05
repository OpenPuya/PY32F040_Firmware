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
#include "py32f040xx_ll_Start_Kit.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LL_LPTIM_InitTypeDef LPTIM_InitStruct = {0};

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigLptimClock(void);
static void APP_ConfigLptim(void);
static void APP_EnterStop(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Configure system clock */
  APP_SystemClockConfig();

  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);
  
  /* Configure EXTI Line29 corresponding to LPTIM as event wake-up mode */
  LL_EXTI_DisableIT(LL_EXTI_LINE_29);   /* Disable EXTI Line 29 interrupt wakeup */
  LL_EXTI_EnableEvent(LL_EXTI_LINE_29); /* Enable EXTI Line 29 event wakeup */
  
  /* Configure LPTIM clock source as LSI */
  APP_ConfigLptimClock();
  
  /* Initialize LPTIM */
  LPTIM_InitStruct.Prescaler = LL_LPTIM_PRESCALER_DIV128;        /* Prescaler: 128 */
  LPTIM_InitStruct.UpdateMode = LL_LPTIM_UPDATE_MODE_IMMEDIATE;  /* Immediate update mode */
  if (LL_LPTIM_Init(LPTIM, &LPTIM_InitStruct) != SUCCESS)
  {
    APP_ErrorHandler();
  }
  
  /* Turn on LED */
  BSP_LED_On(LED_GREEN);
  
  /*  Wait for the button to be pressed  */
  while (BSP_PB_GetState(BUTTON_USER) != 0)
  {
  }

  /* Turn off LED */
  BSP_LED_Off(LED_GREEN);
  
  /* Configure LPTIM for continuous mode and enable interrupt */
  APP_ConfigLptim();
  
  while (1)
  {
    /* Enter STOP mode */
    APP_EnterStop();
    
    /* LED blinking function */
    BSP_LED_Toggle(LED_GREEN);
  }
}

/**
  * @brief  Configure LPTIM clock
  * @param  None
  * @retval None
  */
static void APP_ConfigLptimClock(void)
{
  /* Enable LSI */
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }
  
  /* Select LTPIM clock source as LSI */
  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSI);
  
  /* Enable LPTIM clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
}

/**
  * @brief  Configure LPTIM
  * @param  None
  * @retval None
  */
static void APP_ConfigLptim(void)
{   
  /* Enable LPTIM ARR match interrupt */
  LL_LPTIM_EnableIT_ARRM(LPTIM);
  
  /* Enable LPTIM */
  LL_LPTIM_Enable(LPTIM);
  
  /* Set auto-reload value */
  LL_LPTIM_SetAutoReload(LPTIM, 51);
  
  /* Start in continuous mode */
  LL_LPTIM_StartCounter(LPTIM, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
}

/**
  * @brief  Enter STOP mode
  * @param  None
  * @retval None
  */
static void APP_EnterStop(void)
{
  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  
  /* Enable Low Power Run mode */
  LL_PWR_EnableLowPowerRunMode();

  /* Enter DeepSleep mode */
  LL_LPM_EnableDeepSleep();
  
  /* Request Wait For Event */
  __SEV();
  __WFE();
  __WFE();

  LL_LPM_EnableSleep();
}

/**
  * @brief  System clock configuration function
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  /* Enable HSI */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB prescaler */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Configure HSISYS as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);

  /* Update system clock global variable SystemCoreClock (can also be updated by calling SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(8000000);
}

/**
  * @brief  This function is executed in case of error occurrence.
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
