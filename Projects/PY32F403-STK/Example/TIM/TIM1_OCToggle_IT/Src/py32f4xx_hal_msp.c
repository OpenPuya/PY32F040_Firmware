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
  * @brief Initialize MSP
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize TIM1 related MSP
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /* Enable TIM1 clock */
  __HAL_RCC_TIM1_CLK_ENABLE();                              
  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();                             
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     /* Push_pull output */
  GPIO_InitStruct.Pull = GPIO_NOPULL;             /* PULL UP */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
  /* Set the capture/compare interrupt priority */
  NVIC_SetPriority(TIM1_CC_IRQn, 1);
  /* Enable capture/compare interrupt in NVIC */
  NVIC_EnableIRQ(TIM1_CC_IRQn);
  
  /* Set the update interrupt priority */
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
  /* Enable update interrupt in NVIC */
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
