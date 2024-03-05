/**
  ******************************************************************************
  * @file    py32f040_it.c
  * @author  MCU Application Team
  * @brief   Interrupt Service Routines.
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
#include "py32f040_it.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*          Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* PY32F040 Peripheral Interrupt Handlers                                     */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/
/**
  * @brief This function handles USART2 Interrupt .
  */
void USART2_IRQHandler(void)
{
  /* UART in mode Receiver */
  if ((__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET) && \
      (__HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_RXNE) != RESET))
  {
    RxBuf[RxLen++] = (uint8_t)(UartHandle.Instance->DR & (uint8_t)0x00FF);
  }
  
  /* Idle frame detect */
  if ((__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_IDLE) != RESET) && \
      (__HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_IDLE) != RESET))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&UartHandle);
    
    CheckFlag = 1;             /* Received an idle frame and enabled the check to
                                  see if it is a single frame data receiving end� */
    CheckLen = RxLen;          /* Obtain the length of received single frame data */
    TickStart = HAL_GetTick(); /* Record the current time (for timeout judgment) */
  }
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
