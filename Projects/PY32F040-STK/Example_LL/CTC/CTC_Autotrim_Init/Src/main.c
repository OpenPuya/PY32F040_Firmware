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
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_MCOConfig(void);
static void APP_CTCConfig(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Enable SYSCFG and PWR clocks */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure Systemclock */
  APP_SystemClockConfig();

  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);
  
  /* Configure PA08 as MCO multiplexing function */
  APP_MCOConfig();
  
  /* Configure CTC */
  APP_CTCConfig();

  while (1)
  {
    if (LL_CTC_IsActiveFlag_CKOK(CTC) == 1)
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
  * @brief  Configure system clock
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  /* Enable HSE */
  LL_RCC_HSE_SetFreqRegion(LL_RCC_HSE_16_32MHz);
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }
  
  /* Enable HSI (16MHz) */
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_16MHz);
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }
  
  /* Enable LSE */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();
  while(LL_PWR_IsEnabledBkUpAccess() == 0)
  {
  }
  LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_HIGH);
  LL_RCC_LSE_Enable();
  while(LL_RCC_LSE_IsReady() != 1)
  {
  }
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
  
  /* PLL for HSI (16MHz) 3rd harmonic */
  LL_RCC_PLL_Disable();
  while(LL_RCC_PLL_IsReady() != 0)
  {
  }
  LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSI);
  LL_RCC_PLL_SetMulFactor(LL_RCC_PLLMUL_3);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }  

  /* Set AHB frequency division*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Configure HSISYS as the system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE)
  {
  }

  /* Set APB1 frequency division */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(24000000);

  /* Update the system clock Global variable SystemCoreClock (you can also call the SystemCoreClockUpdate function to update) */
  LL_SetSystemCoreClock(24000000);
}

/**
  * @brief  Configure PA08 as MCO multiplexing function
  * @param  None
  * @retval None
  */
static void APP_MCOConfig(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIOA clock enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* Configure PA08 as MCO multiplexing function */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;                  /* Select pin 8 */
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;        /* Configure to reuse mode */
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; /* Output speed selection */
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; /* Output mode selection */
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;               /* No pull-down */
  GPIO_InitStruct.Alternate = LL_GPIO_AF0_MCO;          /* Select reuse as AF0 function */
  LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
  
  /* MCO output clock and frequency division initialization */
  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_PLLCLK, LL_RCC_MCO1_DIV_1);
}

/**
  * @brief  Configure CTC
  * @param  None
  * @retval None
  */
static void APP_CTCConfig(void)
{
  LL_CTC_InitTypeDef CTC_InitStruct = {0};
  
  /* Enable CTC clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CTC);
  
  /* CTC initialization */
  CTC_InitStruct.AutoTrim       = LL_CTC_AUTO_TRIM_ENABLE;          /* Enable automatic calibration */
  CTC_InitStruct.LimitValue     = 1;                                /* Clock calibration time base limit */
  CTC_InitStruct.ReloadValue    = 1464;                             /* Counter overload value */
  CTC_InitStruct.RefCLKSource   = LL_CTC_REF_CLOCK_SOURCE_LSE;      /* Reference clock source LSE */
  CTC_InitStruct.RefCLKDivider  = LL_CTC_REF_CLOCK_DIV1;            /* Reference clock division 1 division */
  CTC_InitStruct.RefCLKPolarity = LL_CTC_REF_CLOCK_POLARITY_RISING; /* Effective rising edge of reference clock */ 
  LL_CTC_Init(CTC, &CTC_InitStruct);
  
  /* Enable CTC counter */
  LL_CTC_EnableCount(CTC);
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
