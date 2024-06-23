/**
  ******************************************************************************
  * @file    py32f4xx_hal_msp.c
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
static DMA_HandleTypeDef HdmaCh1;
static DMA_HandleTypeDef HdmaCh2;

/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**
  * @brief Initialize global MSP
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize I2S MSP
  * @param  hi2s：I2S handle
  * @retval None
  */
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  
  /* Enable clock */
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_DMA_CLK_ENABLE();
  
  /* Configure pins
  PB12------> I2S2_WS
  PB13------> I2S2_CK
  PB15------> I2S2_SD
  */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_I2S2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* Interrupt configuration */
  HAL_NVIC_SetPriority(SPI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);
  
  /* DMA_CH1 configuration */
  HdmaCh1.Instance                 = DMA1_Channel1;
  HdmaCh1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  HdmaCh1.Init.PeriphInc           = DMA_PINC_DISABLE;
  HdmaCh1.Init.MemInc              = DMA_MINC_ENABLE;
  HdmaCh1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  HdmaCh1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  HdmaCh1.Init.Mode                = DMA_NORMAL;
  HdmaCh1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  /* DMA initialization */
  HAL_DMA_Init(&HdmaCh1);
  /* Associate the initialized DMA handle to the the I2S handle */
  __HAL_LINKDMA(hi2s, hdmatx, HdmaCh1);

  /* DMA_CH2 configuration */
  HdmaCh2.Instance                 = DMA1_Channel2;
  HdmaCh2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  HdmaCh2.Init.PeriphInc           = DMA_PINC_DISABLE;
  HdmaCh2.Init.MemInc              = DMA_MINC_ENABLE;
  HdmaCh2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  HdmaCh2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  HdmaCh2.Init.Mode                = DMA_NORMAL;
  HdmaCh2.Init.Priority            = DMA_PRIORITY_LOW;
  /* DMA initialization */
  HAL_DMA_Init(&HdmaCh2);
  /* Associate the initialized DMA handle to the the I2S handle */
  __HAL_LINKDMA(hi2s, hdmarx, HdmaCh2);
  
  /* Set DMA request mapping */
  HAL_DMA_ChannelMap(&HdmaCh1, DMA_CHANNEL_MAP_SPI2_WR); /* I2S_SD:TX, DMA1_CH1 */
  HAL_DMA_ChannelMap(&HdmaCh2, DMA_CHANNEL_MAP_SPI2_RD); /* I2S_SD:RX, DMA1_CH2 */
  
  /* DMA interrupt settings */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/**
  * @brief Deinitialize I2S MSP
  * @param  hi2s：I2S handle
  * @retval None
  */
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s)
{
  /* Reset I2S peripheral */
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();

  /* Disable peripheral and GPIO clocks */
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15);

  HAL_NVIC_DisableIRQ(SPI2_IRQn);

  HAL_DMA_DeInit(&HdmaCh1);
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  HAL_DMA_DeInit(&HdmaCh2);
  HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
