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
  * @brief   Initialize global MSP
  */
void HAL_MspInit(void)
{
}

/**
  * @brief Initialize TIM14 related MSP
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* Enable TIM14 clock */
  __HAL_RCC_TIM14_CLK_ENABLE();
  
  /* Set the interrupt priority */
  HAL_NVIC_SetPriority(TIM14_IRQn, 3, 0);
  /* Enable TIM14 Interrupt */
  HAL_NVIC_EnableIRQ(TIM14_IRQn);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
