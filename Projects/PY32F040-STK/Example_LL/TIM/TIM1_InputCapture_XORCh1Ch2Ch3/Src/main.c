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
static void APP_ConfigTIM1XOR(void);

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
  
  /* Enable TIM1 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* Configure Systemclock */
  APP_SystemClockConfig(); 
  
  /* Initialize LED*/
  BSP_LED_Init(LED3);
    
  /* Configure and enable the TIM1 XOR mode */
  APP_ConfigTIM1XOR();

  while (1)
  {
  }
}

/**
  * @brief  Configure the TIM1 XOR mode
  * @param  None
  * @retval None
  */
static void APP_ConfigTIM1XOR(void)
{
  LL_TIM_InitTypeDef TIM1CountInit = {0};
  LL_GPIO_InitTypeDef TIM1ChannelInit = {0};
  
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;  /* Clock no divider */
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;      /* Count mode:up */
  TIM1CountInit.Prescaler           = 8000-1;                     /* prescaler：8000 */
  TIM1CountInit.Autoreload          = 1000-1;                     /* Autoreload value：1000 */
  TIM1CountInit.RepetitionCounter   = 0;                          /* RepetitionCounter value：0 */
  
  /* Initialize TIM1 */
  LL_TIM_Init(TIM1,&TIM1CountInit);
  
  /* Enable CC interrupt */
  LL_TIM_EnableIT_CC1(TIM1);

  NVIC_SetPriority(TIM1_CC_IRQn, 1);
  NVIC_EnableIRQ(TIM1_CC_IRQn);
  
  /* Connect the TIM1_CH1, CH2 and CH3 pins  to the TI1 input (XOR combination). */
  LL_TIM_IC_EnableXORCombination(TIM1);
  
  /* Set CH1, CH2, and CH3 in input mode */
  LL_TIM_IC_SetActiveInput(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetActiveInput(TIM1,LL_TIM_CHANNEL_CH2,LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetActiveInput(TIM1,LL_TIM_CHANNEL_CH3,LL_TIM_ACTIVEINPUT_DIRECTTI);
  
  /* Map CH1、CH2、CH3 to PA8、PA9、PA10 */
  TIM1ChannelInit.Pin        = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
  TIM1ChannelInit.Pull       = LL_GPIO_PULL_UP;
  TIM1ChannelInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1ChannelInit.Alternate  = LL_GPIO_AF_2;
  TIM1ChannelInit.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
  TIM1ChannelInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA,&TIM1ChannelInit);
  
  /* Enable CH1、CH2、CH3 */
  LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);

  /* Enable TIM1 */
  LL_TIM_EnableCounter(TIM1);
  
}

/**
  * @brief  TIM1 CH1input capture callback
  * @param  None
  * @retval None
  */
void APP_CCCallback(void)
{
  /* toggle LED */
  BSP_LED_Toggle(LED3);
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
