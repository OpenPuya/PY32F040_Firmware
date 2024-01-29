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

  /* Enable ESMC interrupt */
  HAL_NVIC_SetPriority(ESMC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ESMC_IRQn);

  __HAL_RCC_DMA1_CLK_ENABLE();                     /* Enable DMA1 Clock */
  __HAL_RCC_DMA2_CLK_ENABLE();                     /* Enable DMA2 Clock */

  /* fill the DMA Tx */
  DMA_TxHandle.Instance                   = DMA1_Channel1;

  DMA_TxHandle.Init.Direction             = DMA_MEMORY_TO_PERIPH;
  DMA_TxHandle.Init.PeriphInc             = DMA_PINC_DISABLE;
  DMA_TxHandle.Init.MemInc                = DMA_MINC_ENABLE;
  DMA_TxHandle.Init.PeriphDataAlignment   = DMA_PDATAALIGN_WORD;
  DMA_TxHandle.Init.MemDataAlignment      = DMA_MDATAALIGN_WORD;
  DMA_TxHandle.Init.Mode                  = DMA_NORMAL;
  DMA_TxHandle.Init.Priority              = DMA_PRIORITY_LOW;

  DMA_TxHandle.Lock                       = HAL_UNLOCKED;
  DMA_TxHandle.State                      = HAL_DMA_STATE_RESET;
  DMA_TxHandle.ErrorCode                  = HAL_DMA_ERROR_NONE;

  /* Init DMA */
  HAL_DMA_Init(&DMA_TxHandle);

  /* DMA MAP */
  HAL_DMA_ChannelMap(&DMA_TxHandle, DMA_CHANNEL_MAP_ESMC_TX);

  /* DMA handle associated with ESMC handle  */
  __HAL_LINKDMA(hesmc, hdmatx, DMA_TxHandle);

  /* DMA interrupt settings */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /* fill the DMA Rx */
  DMA_RxHandle.Instance                   = DMA2_Channel1;

  DMA_RxHandle.Init.Direction             = DMA_PERIPH_TO_MEMORY;
  DMA_RxHandle.Init.PeriphInc             = DMA_PINC_DISABLE;
  DMA_RxHandle.Init.MemInc                = DMA_MINC_ENABLE;
  DMA_RxHandle.Init.PeriphDataAlignment   = DMA_PDATAALIGN_WORD;
  DMA_RxHandle.Init.MemDataAlignment      = DMA_MDATAALIGN_WORD;
  DMA_RxHandle.Init.Mode                  = DMA_NORMAL;
  DMA_RxHandle.Init.Priority              = DMA_PRIORITY_LOW;

  DMA_RxHandle.Lock                       = HAL_UNLOCKED;
  DMA_RxHandle.State                      = HAL_DMA_STATE_RESET;
  DMA_RxHandle.ErrorCode                  = HAL_DMA_ERROR_NONE;

  /* Init DMA */
  HAL_DMA_Init(&DMA_RxHandle);

  /* DMA MAP */
  HAL_DMA_ChannelMap(&DMA_RxHandle, DMA_CHANNEL_MAP_ESMC_RX);

  /* DMA handle associated with ESMC handle */
  __HAL_LINKDMA(hesmc, hdmarx, DMA_RxHandle);

  /* DMA interrupt settings */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
