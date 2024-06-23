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
#define I2C_ADDRESS        0xA0             /* Own address 0xA0 */
#define I2C_SPEEDCLOCK   100000             /* Communication speed : 100K */
#define I2C_DUTYCYCLE    I2C_DUTYCYCLE_16_9 /* Duty cycle */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef I2cHandle;
#define RX_MAX_LEN 200     /* Single frame data, maximum received data length */
uint32_t RxLen = 0;        /* Single frame data, actual received data length */
uint8_t RxBuffer[RX_MAX_LEN] = {0}; /* Receive buffer */
__IO uint8_t RevOkFlag = 0;     /* Single frame data received completion flag */

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_SlaveReceive_IT(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  uint32_t i = 0;
  
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  
  /* System clock configuration */
  APP_SystemClockConfig();
  
  /* Initialize USART */
  BSP_USART_Config();
  
  /* I2C initialization */
  I2cHandle.Instance             = I2C1;                      /* I2C */
  I2cHandle.Init.ClockSpeed      = I2C_SPEEDCLOCK;            /* I2C communication speed */
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE;             /* I2C Duty cycle */
  I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;               /* I2C address */
  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;   /* 7-bit addressing mode */
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;   /* Disable dual address */
  /* I2cHandle.Init.OwnAddress2     = I2C_ADDRESS; */         /* Second address */
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;   /* Disable general call */
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;     /* Enable clock stretching */
  if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Slave receives data */
  APP_SlaveReceive_IT();
  
  while (1)
  {
    if (RevOkFlag == 1)
    {
      for (i = 0; i < RxLen; i++)
      {
        printf("%d ", RxBuffer[i]);
      }
      RxLen = 0;
      RevOkFlag = 0;
    }
  }
}

/**
  * @brief  System clock configuration function
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Oscillator configuration */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE; /* Select oscillators HSE, HSI, LSI, LSE */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                          /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                          /* HSI not divided */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;  /* Configure HSI clock as 8MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                         /* Disable HSE */
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* Disable LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                         /* Disable LSE */
  /*RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;                     /* Disable PLL */
  /*RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_NONE;*/
  /*RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;*/
  /* Configure oscillators */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Clock source configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* Select clock types HCLK, SYSCLK, PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS;                                      /* Select HSI as the system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                                             /* AHB  clock not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                                              /* APB  clock not divided */
  /* Configure clock source */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  I2C reception function
  * @param  pData：Pointer to data to be received
  * @param  Size：Size of data to be received
  * @retval None
  */
static void APP_SlaveReceive_IT(void)
{
  /* Clear POS bit */
  CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
  
  /* Enable acknowledge */
  SET_BIT(I2C1->CR1, I2C_CR1_ACK);
  
  /* Enable interrupt */
  __HAL_I2C_ENABLE_IT(&I2cHandle, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
}

/**
  * @brief  I2C interrupt callback function
  * @param  None
  * @retval None
  */
void APP_SlaveIRQCallback(void)
{
  /* Set ADDR flag */
  if ((__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_ADDR) != RESET) && (__HAL_I2C_GET_IT_SOURCE(&I2cHandle, I2C_IT_EVT) != RESET))
  {
    __HAL_I2C_CLEAR_ADDRFLAG(&I2cHandle);
  }
  /* Set STOP flag */
  else if (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_STOPF) != RESET)
  {
    __HAL_I2C_CLEAR_STOPFLAG(&I2cHandle);
    
    RevOkFlag = 1;
  }
  /* Slave Receive */
  else
  {
    /* Set RXNE flag, BTF flag is not set */
    if ((__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_RXNE) != RESET) && (__HAL_I2C_GET_IT_SOURCE(&I2cHandle, I2C_IT_BUF) != RESET) && \
         (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BTF) == RESET))
    {
        RxBuffer[RxLen++] = (uint8_t)(I2C1->DR);
    }
    /* Set BTF flag */
    else if ((__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BTF) != RESET) && (__HAL_I2C_GET_IT_SOURCE(&I2cHandle, I2C_IT_EVT) != RESET))
    {
        RxBuffer[RxLen++] = (uint8_t)(I2C1->DR);
    }
  }
}

/**
  * @brief  After the I2C master receives the last byte, send NACK to the slave, slave NACK 
  * @param  None
  * @retval None
  */
void APP_SlaveIRQCallback_NACK(void)
{
  if ((__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_AF) != RESET) && (__HAL_I2C_GET_IT_SOURCE(&I2cHandle, I2C_IT_ERR) != RESET))
  {
     __HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_AF);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
