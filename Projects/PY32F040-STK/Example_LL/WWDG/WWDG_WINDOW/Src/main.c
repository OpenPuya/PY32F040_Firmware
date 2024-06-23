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
#define WINDOW_IN                                   /* Refresh WWDG Within the window time */
/* #define WINDOW_UPPER */                          /* Refresh WWDG outside the window time upper limit */
/* #define WINDOW_LOWER */                          /* Refresh WWDG outside the window time lower limit */

/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_WwdgConfig(void);
static uint32_t APP_TimeoutCalculation(uint32_t timevalue);

/**
  * @brief  Main program
  * @param  None
  * @retval int
  */
int main(void)
{
  uint32_t delay = 0;
  
  /* Enable SYSCFG and PWR clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure Systemclock */
  APP_SystemClockConfig(); 
  
   /* Initialize LED */
  BSP_LED_Init(LED_GREEN);

  if (LL_RCC_IsActiveFlag_WWDGRST() != RESET)
  {
    /* LED on */
    BSP_LED_On(LED_GREEN);

    /* Delay 4s */
    LL_mDelay(4000);

    /* LED off */
    BSP_LED_Off(LED_GREEN);

    /* Delay 500ms */
    LL_mDelay(500);

    /* Clear the reset flags */
    LL_RCC_ClearResetFlags();
  }
  else
  {
    /* LED off */
    BSP_LED_Off(LED_GREEN);
  }

  /* WWDG config */
  APP_WwdgConfig();

#if defined(WINDOW_IN)
  delay = APP_TimeoutCalculation((0x7F - 0x50) + 1) + 1;                      /* Within the window */
#elif defined(WINDOW_UPPER)
  delay = APP_TimeoutCalculation((0x7F - 0x50) - 5) + 1;                      /* Outside the window time upper limit */
#else 
  delay = APP_TimeoutCalculation((0x7F - 0x3f) + 5) + 1;                      /* outside the window time lower limit */
#endif

  while (1)
  {
    BSP_LED_Toggle(LED_GREEN);

    /* Delay */
    LL_mDelay(delay);

    /* Refresh WWDG */
    LL_WWDG_SetCounter(WWDG, 0x7F);
  }
}

/**
  * @brief  Configure WWDG 
  * @param  None
  * @retval None
  */
static void APP_WwdgConfig(void)
{
  /* Enable WWDG clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);

  /* Set counter value */
  LL_WWDG_SetCounter(WWDG, 0x7F);
  
  /* Set prescaler */
  LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_8);
  
  /* Set window value */
  LL_WWDG_SetWindow(WWDG, 0x50);
  
  /* Enable WWDG */
  LL_WWDG_Enable(WWDG);
}

/**
  * @brief  Timeout Calculation
  * @param  timevalue：time
  * @retval int
  */
static uint32_t APP_TimeoutCalculation(uint32_t timevalue)
{
  uint32_t timeoutvalue = 0;
  LL_RCC_ClocksTypeDef RCC_Clocks = {0};
  uint32_t pclk1 = 0;
  uint32_t wdgtb = 0;

  /* Get PCLK value */
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
  pclk1 = RCC_Clocks.PCLK1_Frequency;

  /* Get prescaler */
  wdgtb = (1 << ((LL_WWDG_PRESCALER_8) >> 7)); /* 2^WDGTB[1:0] */

  /* Calculate timeout value */
  timeoutvalue = ((4096 * wdgtb * timevalue) / (pclk1 / 1000));

  return timeoutvalue;
}

/**
  * @brief  Configure Systemclock
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

  /* Set AHB divider:HCLK = SYSCLK*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* HSISYS used as SYSCLK source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(8000000);
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
