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
static void APP_ConfigTIM1OutputComparison(void);

/**
  * @brief  Main program
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Enable SYSCFG and PWR clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure Systemclock */
  APP_SystemClockConfig(); 
  
  /* Initialize LED*/
  BSP_LED_Init(LED3);
  
  /* Configure and enable TIM1 output comparison mode */
  APP_ConfigTIM1OutputComparison();

  while (1)
  {
  }
}

/**
  * @brief  TIM1 output comparison mode configuration function
  * @param  None
  * @retval None
  */
static void APP_ConfigTIM1OutputComparison(void)
{
  /* Enable TIM1 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  
  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* No divide */
  LL_TIM_SetClockDivision(TIM1,LL_TIM_CLOCKDIVISION_DIV1);
  
  /* Count mode:up */
  LL_TIM_SetCounterMode(TIM1,LL_TIM_COUNTERMODE_UP);
  
  /* Set auto-reload value to 1000 */
  LL_TIM_SetAutoReload(TIM1,1000-1);
  
  /* prescaler：8000 */
  LL_TIM_SetPrescaler(TIM1,8000-1);
  
  /* Configure channel 1 */
  /* Set output polarity to high */
  LL_TIM_OC_SetPolarity(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH);
  /* Set idle state to low */
  LL_TIM_OC_SetIdleState(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_OCIDLESTATE_LOW);
  /* Set compare value to 500 */
  LL_TIM_OC_SetCompareCH1(TIM1,500);
  /* Set TIM1 channel 1 to output compare toggle mode */
  LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_OCMODE_TOGGLE);
  
  /* Map channel 1 output to PA8 */
  LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_8,LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(GPIOA,LL_GPIO_PIN_8,LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetAFPin_8_15(GPIOA,LL_GPIO_PIN_8,LL_GPIO_AF_2);
  
  /* Enable channel 1 */
  LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
  
  /* Disable automatic output */
  LL_TIM_DisableAutomaticOutput(TIM1);
  
  /* Enable output */
  LL_TIM_EnableAllOutputs(TIM1);
  /* Enable TIM1 counter */
  LL_TIM_EnableCounter(TIM1);
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
