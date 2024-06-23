/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
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

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
uint8_t RxBuf[RX_MAX_LEN] = {0}; /* Receive buffer */
uint32_t RxLen = 0;          /* Single frame data, actual received data length */
__IO uint32_t CheckFlag = 0; /* After the hardware detects an idle frame, the
                                "End of single frame data reception" check flag */
__IO uint32_t CheckLen = 0;  /* Used to determine whether new data has arrived within
                                the timeout period */
__IO uint32_t TickStart = 0; /* Used for timeout judgment */

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void APP_UartRxTimeOut(void);

/**
  * @brief  Main program
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  
  /* Initialize USART */
  UartHandle.Instance          = USART2;
  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&UartHandle);
  
  /* Enable idle frame interrupt */
  __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_IDLE);
  
  /* Enable RXNE interrupt */
  __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);

  while (1)
  {
    /* Uart receive timeout check */
    APP_UartRxTimeOut();
  }
}

/**
  * @brief  Uart receive timeout check.
  * @param  None.
  * @retval None
  */
void APP_UartRxTimeOut(void)
{
  /* Received an idle frame and turned on the check to see
     if "single frame data reception ended" */
  if (CheckFlag == 1)
  {
    /* If the 'timeout time' is exceeded,
       it is considered that receiving one frame of data has ended */
    if ((HAL_GetTick() - TickStart) > RX_TIMEOUT)
    {
      /* Send back received data */      
      HAL_UART_Transmit(&UartHandle, (uint8_t *)&RxBuf, RxLen, 5000);
      
      /* Enable next data reception */
      CheckFlag = 0;
      RxLen = 0;
    }
    
    /* Received new data, continue receiving data */
    if (CheckLen != RxLen)
    {
      CheckFlag = 0;
    }
  }
}

/**
  * @brief  Error handling function
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  /* Infinite loop */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file：Pointer to the source file name
  * @param  line：assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add His own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
