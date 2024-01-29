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
  * @brief Initialize ESMC MSP.
  */
void HAL_ESMC_MspInit(ESMC_HandleTypeDef *hesmc)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable ESMC Clock */
  __HAL_RCC_ESMC_CLK_ENABLE();

  /* Enable GPIO Clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* ESMC GPIO pin initialization */
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Alternate = GPIO_AF10_ESMC;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  /* CLK */
  GPIO_InitStructure.Pin = GPIO_PIN_10;
  GPIO_InitStructure.Pull =GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* IO0 */
  GPIO_InitStructure.Pin = GPIO_PIN_1;
  GPIO_InitStructure.Pull =GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* IO1 */
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Pull =GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* IO2 */
  GPIO_InitStructure.Pin = GPIO_PIN_7;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* IO3 */
  GPIO_InitStructure.Pin = GPIO_PIN_6;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* SS0 */
  GPIO_InitStructure.Pin = GPIO_PIN_2;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Esmc interrupt settings */
  HAL_NVIC_SetPriority(ESMC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ESMC_IRQn);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
