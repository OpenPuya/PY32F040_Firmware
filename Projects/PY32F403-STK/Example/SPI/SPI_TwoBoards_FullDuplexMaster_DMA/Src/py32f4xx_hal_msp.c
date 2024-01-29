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
  * @brief Initialize the MSP
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize SPI related MSP
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  /* Initialize SPI1 */
  if (hspi->Instance == SPI1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();                   /* Enable GPIOB clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();                   /* Enable GPIOA clock */
    __HAL_RCC_SYSCFG_CLK_ENABLE();                  /* Enable SYSCFG clock */
    __HAL_RCC_SPI1_CLK_ENABLE();                    /* Enable SPI1 clock */
    __HAL_RCC_DMA1_CLK_ENABLE();                    /* Enable DMA clock */
    
    /*
      PB3-SCK  (AF3)
      PB4-MISO(AF3)
      PB5-MOSI(AF3)
      PA15-NSS(AF3)
    */
    /*SCK*/
    GPIO_InitStruct.Pin       = GPIO_PIN_3;
    if (hspi->Init.CLKPolarity == SPI_POLARITY_LOW)
    {
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
    else
    {
      GPIO_InitStruct.Pull = GPIO_PULLUP;
    }
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI NSS*/
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    /* GPIO configured as SPI：MISO/MOSI*/
    GPIO_InitStruct.Pin       = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* Interrupt configuration */
    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
    /* DMA_CH1 configuration */
    HdmaCh1.Instance                 = DMA1_Channel1;
    HdmaCh1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    HdmaCh1.Init.PeriphInc           = DMA_PINC_DISABLE;
    HdmaCh1.Init.MemInc              = DMA_MINC_ENABLE;
    if (hspi->Init.DataSize <= SPI_DATASIZE_8BIT)
    {
      HdmaCh1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      HdmaCh1.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    }
    else
    {
      HdmaCh1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      HdmaCh1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    }
    HdmaCh1.Init.Mode                = DMA_NORMAL;
    HdmaCh1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    /* Initialize DMA */
    HAL_DMA_Init(&HdmaCh1);
    /* DMA handle is associated with SPI handle */
    __HAL_LINKDMA(hspi, hdmatx, HdmaCh1);

    /* DMA_CH2 configuration */
    HdmaCh2.Instance                 = DMA1_Channel2;
    HdmaCh2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    HdmaCh2.Init.PeriphInc           = DMA_PINC_DISABLE;
    HdmaCh2.Init.MemInc              = DMA_MINC_ENABLE;
    if (hspi->Init.DataSize <= SPI_DATASIZE_8BIT)
    {
      HdmaCh2.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      HdmaCh2.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    }
    else
    {
      HdmaCh2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      HdmaCh2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    }
    HdmaCh2.Init.Mode                = DMA_NORMAL;
    HdmaCh2.Init.Priority            = DMA_PRIORITY_LOW;
    /* Initialize DMA*/
    HAL_DMA_Init(&HdmaCh2);
    /* DMA handle is associated with SPI handle*/
    __HAL_LINKDMA(hspi, hdmarx, HdmaCh2);
    
    /* Set DMA channel map. */
    HAL_DMA_ChannelMap(&HdmaCh1, DMA_CHANNEL_MAP_SPI1_WR); /* SPI1_TX DMA1_CH1 */
    HAL_DMA_ChannelMap(&HdmaCh2, DMA_CHANNEL_MAP_SPI1_RD); /* SPI1_RX DMA1_CH2 */
    
    /* DMA interrupt configuration*/
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  }
}

/**
  * @brief Deinit SPI MSP
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    /* Reset SPI peripheral */
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();

    /* Disable SPI and GPIO clock */
    /* Deinit SPI SCK */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

    HAL_NVIC_DisableIRQ(SPI1_IRQn);

    HAL_DMA_DeInit(&HdmaCh1);
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

  }
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
