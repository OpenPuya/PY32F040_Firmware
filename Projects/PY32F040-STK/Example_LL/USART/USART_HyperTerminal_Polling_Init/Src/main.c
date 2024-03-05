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
#include "py32f040xx_ll_Start_Kit.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[] = "UART Test";
uint8_t aRxBuffer[30];

uint8_t *TxBuff = NULL;
__IO uint16_t TxCount = 0;

uint8_t *RxBuff = NULL;
__IO uint16_t RxCount = 0;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigUsart(void);
static void APP_UsartTransmit(USART_TypeDef *USARTx, uint8_t *pData, uint16_t Size);
static void APP_UsartReceive(USART_TypeDef *USARTx, uint8_t *pData, uint16_t Size);

/**
  * @brief  Main program
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Configure Systemclock */
  APP_SystemClockConfig(); 

  /* USART configuration */
  APP_ConfigUsart();
  
  /* Send string:"UART Test"，and wait send complete */
  APP_UsartTransmit(USART2, (uint8_t*)aTxBuffer, sizeof(aTxBuffer)-1);

  while (1)
  {
    /* Receive data */
    APP_UsartReceive(USART2, (uint8_t *)aRxBuffer, 12);
    
    /* Send data */
    APP_UsartTransmit(USART2, (uint8_t*)aRxBuffer, 12);
  }
}

/**
  * @brief  Configure Systemclock
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  /* Enable HSI */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB divider:HCLK = SYSCLK*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* HSISYS used as SYSCLK source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(8000000);
}

/**
  * @brief  USART2 configuration function
  * @param  None
  * @retval None
  */
static void APP_ConfigUsart(void)
{
  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /* Enable USART2 clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  /* GPIOA configuration */
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  /* Select pin 2 */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  /* Select alternate function mode */
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  /* Set output speed */
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  /* Set output type to push pull */
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  /* Enable pull up */
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  /* Set alternate function to USART2 function  */
  GPIO_InitStruct.Alternate = LL_GPIO_AF1_USART2;
  /* Initialize GPIOA */
  LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
  
  /* Select pin 3 */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  /* Set alternate function to USART2 function  */
  GPIO_InitStruct.Alternate = LL_GPIO_AF1_USART2;
  /* Initialize GPIOA */
  LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
  
  /* Set USART feature */
  LL_USART_InitTypeDef USART_InitStruct;
  /* Set baud rate */
  USART_InitStruct.BaudRate = 9600;
  /* set word length to 8 bits: Start bit, 8 data bits, n stop bits */
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  /* 1 stop bit */
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
 /* Parity control disabled  */
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  /* Initialize USART */
  LL_USART_Init(USART2, &USART_InitStruct);
  
  /* Set mode as full-duplex asynchronous mode */
  LL_USART_ConfigAsyncMode(USART2);
  
  /*Enable USART */
  LL_USART_Enable(USART2);
}

/**
  * @brief  USART transmission function
  * @param  USARTx：USART Instance，This parameter can be one of the following values:USART1、USART2
  * @param  pData：Pointer to transmission buffer
  * @param  Size：Size of transmission buffer
  * @retval None
  */
static void APP_UsartTransmit(USART_TypeDef *USARTx, uint8_t *pData, uint16_t Size)
{
  TxBuff = pData;
  TxCount = Size;
  
  /* transmit data */
  while (TxCount > 0)
  {
    /* Wait for TXE bit to be set */
    while(LL_USART_IsActiveFlag_TXE(USARTx) != 1);
    /* transmit data */
    LL_USART_TransmitData8(USARTx, *TxBuff);
    TxBuff++;
    TxCount--;
  }
  
  /* Wait for TC bit to be set */
  while(LL_USART_IsActiveFlag_TC(USARTx) != 1);
}

/**
  * @brief  USART receive function
  * @param  USARTx：USART Instance，This parameter can be one of the following values:USART1、USART2
  * @param  pData：Pointer to receive buffer
  * @param  Size：Size of receive buffer
  * @retval None
  */
static void APP_UsartReceive(USART_TypeDef *USARTx, uint8_t *pData, uint16_t Size)
{
  RxBuff = pData;
  RxCount = Size;
  
  /* Receive data */
  while (RxCount > 0)
  {
    /* Wait for RxNE bit to be set */
    while(LL_USART_IsActiveFlag_RXNE(USARTx) != 1);
    /* Receive data */
    *RxBuff = LL_USART_ReceiveData8(USARTx);
    RxBuff++;
    RxCount--;
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
