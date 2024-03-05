/**
  ******************************************************************************
  * @file    py32f040_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**
  * @brief   Initialize the MSP
  */
void HAL_MspInit(void)
{
}

/**
  * @brief   Initialize TIM1 related MSP
  */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /* Enable TIM1 clock */
  __HAL_RCC_TIM1_CLK_ENABLE();
  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       /* Alternate Function Push Pull Mode */
  GPIO_InitStruct.Pull = GPIO_PULLUP;           /* Pull up */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* GPIO speed:high speed */
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;    /* Alternate Function is TIM1 */

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  /* Initialize PA8 */
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
