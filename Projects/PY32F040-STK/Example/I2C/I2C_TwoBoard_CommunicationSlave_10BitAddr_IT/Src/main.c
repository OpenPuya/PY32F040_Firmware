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
#define DARA_LENGTH       15                /* Length of data */
#define I2C_ADDRESS        0xA0             /* Own address 0xA0 */
#define I2C_SPEEDCLOCK   100000             /* Communication speed : 100K */
#define I2C_DUTYCYCLE    I2C_DUTYCYCLE_2    /* Duty cycle */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef I2cHandle;
uint8_t aTxBuffer[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
uint8_t aRxBuffer[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_CheckEndOfTransfer(void);
static uint8_t APP_Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);
static void APP_LedBlinking(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  
  /* System clock configuration */
  APP_SystemClockConfig();
  
  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);
  
  /* I2C initialization */
  I2cHandle.Instance             = I2C1;                      /* I2C */
  I2cHandle.Init.ClockSpeed      = I2C_SPEEDCLOCK;            /* I2C communication speed */
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE;             /* I2C Duty cycle */
  I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;               /* I2C address */
  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_10BIT;  /* 10-bit addressing mode */
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;   /* Disable dual address */
  /* I2cHandle.Init.OwnAddress2     = I2C_ADDRESS; */         /* Second address */
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;   /* Disable general call */
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;     /* Enable clock stretching */
  if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Receive data in slave mode with interrupt */
  while (HAL_I2C_Slave_Receive_IT(&I2cHandle, (uint8_t *)aRxBuffer, DARA_LENGTH) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  /* Check the current I2C state */
  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY);
  /* Transmit data in slave mode with interrupt */
  while (HAL_I2C_Slave_Transmit_IT(&I2cHandle, (uint8_t *)aTxBuffer, DARA_LENGTH) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  /* Check the current I2C state */
  while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY);
  
  /* Check the received data */
  APP_CheckEndOfTransfer();
  
  while (1)
  {
  }
}

/**
  * @brief  Data verification function
  * @param  None
  * @retval None
  */
static void APP_CheckEndOfTransfer(void)
{
  /* Compare the transmitted data with the received data */
  if(APP_Buffercmp8((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, DARA_LENGTH))
  {
    /* Error handling */
    APP_LedBlinking();
  }
  else
  {
    /* If data received successfully, turn on the LED */
    BSP_LED_On(LED_GREEN);
  }
}

/**
  * @brief  Character comparison function
  * @param  pBuffer1：Pointer to the first buffer to be compared
  * @param  pBuffer2：Pointer to the second buffer to be compared
  * @param  BufferLength：Number of characters to compare
  * @retval 0: buffers are the same; 1: buffers are different
  */
static uint8_t APP_Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

/**
  * @brief  LED blinking function
  * @param  None
  * @retval None
  */
static void APP_LedBlinking(void)
{
  while (1)
  {
    BSP_LED_Toggle(LED_GREEN);; 
    HAL_Delay(500);
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS; /* Select HSI as the system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* AHB  clock not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* APB  clock not divided */
  /* Configure clock source */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
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
