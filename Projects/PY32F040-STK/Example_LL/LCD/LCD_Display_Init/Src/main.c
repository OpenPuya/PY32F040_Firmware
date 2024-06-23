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
LL_LCD_InitTypeDef LCD_InitStruct;
LL_LCD_SegComTypeDef LcdSegCom;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigLcdClock(void);
static void APP_ConfigLcdPin(void);

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

  /* Configure system clock */
  APP_SystemClockConfig();
  
  /* Configure LCD clock */
  APP_ConfigLcdClock();
  
  /* Configure LCD pins */
  APP_ConfigLcdPin();
  
  /* Initialize LCD */
  LCD_InitStruct.Contrast = LL_LCD_CONTRASTLEVEL_0;
  LCD_InitStruct.BiasSrc = LL_LCD_BIAS_SRC_IN_RES_HIGH_POWER;
  LCD_InitStruct.Duty = LL_LCD_DUTY_1_4;
  LCD_InitStruct.Bias = LL_LCD_BIAS_1_3;
  LCD_InitStruct.ScanFre = LL_LCD_SCAN_FRE_128HZ;
  LCD_InitStruct.Mode = LL_LCD_MODE_0;
  LL_LCD_Init(LCD, &LCD_InitStruct);
  
  /* Set SEG COM output */
  LcdSegCom.Seg0_31 = 0xffffff00;                            /* Configure LCD_POEN0 register to enable SEG0~SEG7 */
  LcdSegCom.Seg32_39_Com0_7_t.Seg32_39_Com0_7 = 0xffffffff;  /* Initialize LCD_POEN1 register, all output ports closed */
  LcdSegCom.Seg32_39_Com0_7_t.SegComBit.Com0_3 = 0;          /* Enable COM0~COM3 */
  LcdSegCom.Seg32_39_Com0_7_t.SegComBit.Mux = 0;             /* This bit needs to be configured as 0 for internal resistor */
  LL_LCD_SetSegCom(LCD, &LcdSegCom);
  
  /* Display "88:88" on the LCD */
  LL_LCD_Write(LCD, LL_LCD_RAM_REGISTER0, 0x0f0f0f0f);
  LL_LCD_Write(LCD, LL_LCD_RAM_REGISTER1, 0x0f0f0f0f);

  while (1)
  {
  }
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
  * @brief  Configure LCD clock
  * @param  None
  * @retval None
  */
static void APP_ConfigLcdClock(void)
{
  /* Enable LSI */
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }
  
  /* Select LSI as the clock source for LCD */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();
  LL_RCC_LSCO_SetSource(LL_RCC_LSCO_CLKSOURCE_LSI);
  LL_RCC_LSCO_Enable();
  LL_PWR_DisableBkUpAccess();
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
  
  /* Enable LCD clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LCD);
}

/**
  * @brief  Configure LCD pins
  * @param  None
  * @retval None
  */
static void APP_ConfigLcdPin(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Enable LCD pin clocks */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  
  /* Configure LCD pins */
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);/* COM0 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);/* COM1 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);/* COM2 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);/* COM3 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);/* SEG0 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);/* SEG1 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);/* SEG2 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);/* SEG3 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);/* SEG4 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);/* SEG5 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);/* SEG6 */
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);/* SEG7 */
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
