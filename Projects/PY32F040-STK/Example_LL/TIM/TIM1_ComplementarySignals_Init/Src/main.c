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
static void APP_ConfigTIM1Base(void);
static void APP_ConfigPWMChannel(void);

/**
  * @brief  Main program
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Configure Systemclock */
  APP_SystemClockConfig(); 
  
  /* Enable TIM1、GPIO clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA|LL_IOP_GRP1_PERIPH_GPIOB);
  
  /* Configure the TIM1 PWM channel */
  APP_ConfigPWMChannel();
  
  /* Configure and enable TIM1 PWM mode */
  APP_ConfigTIM1Base();
  
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Configure TIM1 PWM related GPIO
  * @param  None
  * @retval None
  */
static void APP_ConfigPWMChannel(void)
{
  LL_GPIO_InitTypeDef TIM1CH1MapInit= {0};
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct ={0};

  /* Set PA7/PA8/PA9/PA10 as TIM1_CH1N/TIM1_CH1/TIM1_CH2/TIM1_CH3 */
  TIM1CH1MapInit.Pin        = LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  TIM1CH1MapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1CH1MapInit.Alternate  = LL_GPIO_AF_2; 
  LL_GPIO_Init(GPIOA,&TIM1CH1MapInit);
  
  /* Set PB4/PB5 as TIM1_CH2N/TIM1_CH3N */
  TIM1CH1MapInit.Pin        = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  TIM1CH1MapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1CH1MapInit.Alternate  = LL_GPIO_AF_11; 
  LL_GPIO_Init(GPIOB,&TIM1CH1MapInit);
  
  /* Configure PWM channel */
  TIM_OC_Initstruct.OCMode        = LL_TIM_OCMODE_PWM1;     /* mode：PWM1 */
  TIM_OC_Initstruct.OCState       = LL_TIM_OCSTATE_ENABLE;  /* Channel enable */
  TIM_OC_Initstruct.OCNState      = LL_TIM_OCSTATE_ENABLE;  /* Complementary channel enable */
  TIM_OC_Initstruct.OCPolarity    = LL_TIM_OCPOLARITY_HIGH; /* Compare output polarity: HIGH */
  TIM_OC_Initstruct.OCNPolarity   = LL_TIM_OCPOLARITY_HIGH; /* Compare complementary output polarity: HIGH */
  TIM_OC_Initstruct.OCIdleState   = LL_TIM_OCIDLESTATE_LOW; /* Output Idle state: LOW */
  TIM_OC_Initstruct.OCNIdleState  = LL_TIM_OCIDLESTATE_LOW; /* Complementary output Idle state: LOW */
  /* Channel 1 comparison value:250 */
  TIM_OC_Initstruct.CompareValue  = 250;
  /* Configure channel 1 */
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH1,&TIM_OC_Initstruct);
  /* Channel 2 comparison value:500 */
  TIM_OC_Initstruct.CompareValue  = 500;
  /* Configure channel 2 */
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH2,&TIM_OC_Initstruct);
  /* Channel 3 comparison value:750 */
  TIM_OC_Initstruct.CompareValue  = 750;
  /* Configure channel 3 */
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH3,&TIM_OC_Initstruct);
}

/**
  * @brief  Configure TIM base
  * @param  None
  * @retval None
  */
static void APP_ConfigTIM1Base(void)
{
  /* Configure TIM1 */
  LL_TIM_InitTypeDef TIM1CountInit = {0};
 
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;/* Clock no divider */
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;    /* Count mode:up */
  TIM1CountInit.Prescaler           = 2400-1;                   /* prescaler：2400 */
  TIM1CountInit.Autoreload          = 1000-1;                   /* Autoreload value：1000 */
  TIM1CountInit.RepetitionCounter   = 0;                        /* RepetitionCounter value：0 */
  
  /* Initialize TIM1 */
  LL_TIM_Init(TIM1,&TIM1CountInit);
  
  /* Enable output */
  LL_TIM_EnableAllOutputs(TIM1);

  /* Enable TIM1 */
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
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
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
  LL_Init1msTick(24000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
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
