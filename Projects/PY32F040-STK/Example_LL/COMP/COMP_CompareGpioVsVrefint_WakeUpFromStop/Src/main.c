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
__IO uint32_t wait_loop_index = 0;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_CompConfig(void);
static void APP_EnterStop(void);
static void APP_CompRccInit(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Enable SYSCFG clock and PWR clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  
  APP_SystemClockConfig();

  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);
  
  /* Initialization button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);
  
  /* Comparator clock switching */
  APP_CompRccInit();

  /* Configure Comparator */
  APP_CompConfig();
  
  /* LED on */
  BSP_LED_On(LED_GREEN);

  /* Wait for the user to press the button to start entering STOP mode */
  while(BSP_PB_GetState(BUTTON_KEY) == 1)
  {}
  
  /* LED off */
  BSP_LED_Off(LED_GREEN);
  
  /* Entering STOP mode */
  APP_EnterStop();

  while (1)
  {
    BSP_LED_Toggle(LED_GREEN);
    
    LL_mDelay(200); 
  }
}

/**
  * @brief  Comparator Configuration Function
  * @param  None
  * @retval None
  */
static void APP_CompConfig(void)
{
  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* Configure PA0 pin in analog mode */
  LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_0,LL_GPIO_MODE_ANALOG);
  
  /*Set PA0 as dropdown*/
  LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_0,LL_GPIO_PULL_DOWN);
  
  /* Enable ADC clock*/
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  
  /* Enable Vrefint */
  LL_ADC_SetCommonPathInternalCh(ADC1_COMMON,LL_ADC_PATH_INTERNAL_VREFINT);
    
  /* Enable comparator 1 clock*/
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_COMP1);
  
  /* Set comparator 1 Minus input IO13 VREFINT */
  LL_COMP_SetInputMinus(COMP1,LL_COMP_INPUT_MINUS_IO13);
  
  /* Set comparator 1 Plus input IO4 PA0 */
  LL_COMP_SetInputPlus(COMP1,LL_COMP_INPUT_PLUS_IO4);
  
  /* Set comparator 1 hysteresis off */
  LL_COMP_SetInputHysteresis(COMP1,LL_COMP_HYSTERESIS_DISABLE);
  
  /* Set comparator 1 output not to reverse */
  LL_COMP_SetOutputPolarity(COMP1,LL_COMP_OUTPUTPOL_NONINVERTED);
  
  /* Power consumption mode fast */
  LL_COMP_SetPowerMode(COMP1, LL_COMP_POWERMODE_HIGHSPEED);
 
  /* Window mode not enabled */
  LL_COMP_SetCommonWindowMode(__LL_COMP_COMMON_INSTANCE(COMP1), LL_COMP_WINDOWMODE_DISABLE);
   
  /* Enable rising edge interrupt */
  LL_EXTI_EnableRisingTrig(LL_EXTI_LINE_17);

  /* Enable interrupt */
  LL_EXTI_EnableIT(LL_EXTI_LINE_17);

  NVIC_SetPriority(ADC_COMP_IRQn, 0);
  NVIC_EnableIRQ(ADC_COMP_IRQn);

  /* Enable Comparator 1 */
  LL_COMP_Enable(COMP1);
  wait_loop_index = ((LL_COMP_DELAY_STARTUP_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
  while(wait_loop_index != 0UL)
  {
    wait_loop_index--;
  }
}

/**
  * @brief  Comparator clock switching
  * @param  None
  * @retval None
  */
static void APP_CompRccInit()
{
  /* Enable LSI */
  LL_RCC_LSI_Enable();
 
  /* Waiting for LSI to stabilize */
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }
 
  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Enable DBP */
  LL_PWR_EnableBkUpAccess();

  /* Set LSI to low speed clock */
  LL_RCC_LSCO_SetSource(LL_RCC_LSCO_CLKSOURCE_LSI);

  /* Enable low-speed clock */
  LL_RCC_LSCO_Enable();

  /* Set the clock source of COMP to LSC */
  LL_RCC_SetCOMPClockSource(LL_RCC_COMP1_CLKSOURCE_LSC);
}

/**
  * @brief  Enter the STOP mode function
  * @param  None
  * @retval None
  */
static void APP_EnterStop(void)
{
  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Low power operation in STOP mode */
  LL_PWR_EnableLowPowerRunMode();
 
  /* Enter DeepSleep mode */
  LL_LPM_EnableDeepSleep();

  /* Waiting for interrupt instructions */
  __WFI();

  LL_LPM_EnableSleep();
}

/**
  * @brief  Configure system clock
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

  /* Set AHB prescaler: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Select HSISYS as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB prescaler: PCLK = HCLK */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);

  /* Update the SystemCoreClock global variable(which can be updated also through SystemCoreClockUpdate function) */
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
