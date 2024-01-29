/**
  ******************************************************************************
  * @file    py32f4xx_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**
  * @brief Initialize global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize MSP for CTC
  */
void HAL_CTC_MspInit(CTC_HandleTypeDef *hctc)
{
  GPIO_InitTypeDef ctcpininit;
  __HAL_RCC_CTC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_NVIC_EnableIRQ(RCC_CTC_IRQn);
  
  ctcpininit.Pin    = GPIO_PIN_10;
  ctcpininit.Mode   = GPIO_MODE_AF_PP;
  ctcpininit.Pull   = GPIO_PULLDOWN;
  ctcpininit.Speed  = GPIO_SPEED_FREQ_HIGH;
  ctcpininit.Alternate  = GPIO_AF3_SPI1;
  
  HAL_GPIO_Init(GPIOA,&ctcpininit);
}
/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
