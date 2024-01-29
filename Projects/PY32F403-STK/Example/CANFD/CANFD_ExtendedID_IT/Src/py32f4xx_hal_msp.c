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
  * @brief Initialize global MSP
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize MSP for CANFD
  */
void HAL_CANFD_MspInit(CANFD_HandleTypeDef *hcan)
{
  RCC_OscInitTypeDef  OscInitstruct = {0};
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Configure the clock source of PLL as HSE (24MHz), with PLL 5 times the frequency, and PLL reaching 120MHz */
  OscInitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  OscInitstruct.HSEState        = RCC_HSE_ON;                           /* Start HSE */
  OscInitstruct.HSEFreq         = RCC_HSE_16_32MHz;                     /* Choose HSE frequency of 16-32MHz */
  
  OscInitstruct.PLL.PLLState    = RCC_PLL_ON;                           /* Enable PLL */
  OscInitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;                    /* PLL clock source selection HSE */
  OscInitstruct.PLL.PLLMUL      = RCC_PLL_MUL5;                         /* PLL clock source 5th harmonic */
  if(HAL_RCC_OscConfig(&OscInitstruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* The clock source selection for CAN is PLL clock */
  __HAL_RCC_CAN_CONFIG(RCC_CANCLKSOURCE_PLL_DIV6); /* The clock source frequency of CAN is 20Mhz */
  
  /* enable clock */
  __HAL_RCC_CAN_CLK_ENABLE(); /* Enable CAN clock */
  __HAL_RCC_GPIOA_CLK_ENABLE(); /* Enable GPIOA clock */
  
  /* Configure CAN pins */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12; /* CAN_RX:PA11,CAN_TX:PA12 */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_CAN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* NVIC interrupt enabling CANFD */
  HAL_NVIC_SetPriority(CAN_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CAN_IRQn);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
