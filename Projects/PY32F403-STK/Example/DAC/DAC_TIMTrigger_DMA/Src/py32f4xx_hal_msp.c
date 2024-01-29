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
DMA_HandleTypeDef  hdma_dac1; 

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
  * @brief Initialize DAC MSP.
  */
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
  __HAL_RCC_DMA1_CLK_ENABLE();                                 /* Enable DMA clock */

  hdma_dac1.Instance = DMA1_Channel1;                          /* DMA1 Channel1 */
  hdma_dac1.Init.Direction = DMA_MEMORY_TO_PERIPH;             /* Transfer mode Memory to Peripth */
  hdma_dac1.Init.PeriphInc = DMA_PINC_DISABLE;                 /* Peripheral increment mode Disable */
  hdma_dac1.Init.MemInc = DMA_MINC_ENABLE;                     /* Memory increment mode Enable */
  hdma_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;    /* Peripheral data alignment : Word  */
  hdma_dac1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;       /* Memory data alignment : Word  */
  hdma_dac1.Init.Mode = DMA_CIRCULAR;                          /* Circular DMA mode */
  hdma_dac1.Init.Priority = DMA_PRIORITY_HIGH;                 /* Priority level : high  */

  HAL_DMA_Init(&hdma_dac1);
  __HAL_LINKDMA(hdac, DMA_Handle1, hdma_dac1);
  HAL_DMA_ChannelMap(&hdma_dac1,DMA_CHANNEL_MAP_DAC1);         /* DMA Channel Remap */
  
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  
}
/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
