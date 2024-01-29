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
extern DMA_HandleTypeDef HdmaCh1;

/**
  * @brief Initialize global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize ADC MSP.
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct={0};
  __HAL_RCC_SYSCFG_CLK_ENABLE();                              /* Enable SYSCFG clock */
  __HAL_RCC_DMA1_CLK_ENABLE();                                /* Enable DMA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();                               /* Enable GPIOA clock */
  
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |GPIO_PIN_7 ;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  HdmaCh1.Instance                 = DMA1_Channel1;           /* DMA1 Channel1 */
  HdmaCh1.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* Transfer mode Periph to Memory */
  HdmaCh1.Init.PeriphInc           = DMA_PINC_DISABLE;        /* Peripheral increment mode Disable */
  HdmaCh1.Init.MemInc              = DMA_MINC_ENABLE;         /* Memory increment mode Enable */
  HdmaCh1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;     /* Peripheral data alignment : Word  */
  HdmaCh1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;     /* Memory data alignment : Word  */
  HdmaCh1.Init.Mode                = DMA_CIRCULAR;            /* Circular DMA mode */
  HdmaCh1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;  /* Priority level : high  */

  HAL_DMA_DeInit(&HdmaCh1);                                   /* DMA1CH1 Deinitialize */
  HAL_DMA_Init(&HdmaCh1);                                     /* DMA1CH1 Initialize */
  
  HAL_DMA_ChannelMap(&HdmaCh1,DMA_CHANNEL_MAP_ADC1);          /* DMA Channel Remap */
  __HAL_LINKDMA(hadc, DMA_Handle, HdmaCh1);                  
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
