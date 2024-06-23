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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Create queue and queueset */
QueueHandle_t    QueueHandle;
QueueHandle_t    SemphrHandle;
QueueSetHandle_t QueueSetHandle;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define  QUEUEU_NUM               1
#define  QUEUEUSET_NUM            2

/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void Task1(void *pvParamters);
static void Task2(void *pvParamters);
static void QueueSetCreate(void);

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

  /* Initialize UART */	
  BSP_USART_Config();

  /* Create the tasks that are created using the original xTaskCreate() API function. */
  xTaskCreate( Task1, "Task1", 128, NULL, 1, NULL );  
  xTaskCreate( Task2, "Task2", 128, NULL, 2, NULL );

  /* Create semaphore */
  QueueSetCreate();

  /* Start the scheduler. */
  vTaskStartScheduler();
}

/**
  * @brief  Send queue to queueset, give semaphore to queueset.
  * @param  *pvParamters: The parameters passed to the task function when the task is created.
  * @retval None
  */
static void Task1(void *pvParamters)
{
  uint8_t CountValue = 0;
  BaseType_t Err = 0;
  while (1)
  {
    /* If CountValue = 10,prepare send count_value to queue */
    if(CountValue == 10)
    {
      /* Send CountValue to queue,return a value to Err. */
      /* portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      Err = xQueueSend(QueueHandle, &CountValue, portMAX_DELAY);
      /* Err != pdFALSE,send success,output CountValue. */
      if(Err != pdFALSE)
      {
        printf("Task1: queue send success!\r\n");
      }
    }
    /* If CountValue = 20,prepare give semaphore */
    if(CountValue == 20)
    {	
      /* Give semaphoree,return a value to Err */
      Err = xSemaphoreGive(SemphrHandle);
      /* Err != pdFALSE,give success. */
      if(Err != pdFALSE)
      {
        printf("Task1: semaphore release success!\r\n");
      }
    }
    CountValue ++;
    /* If CountValue = 21,set CountValue = 0 */
    if(CountValue == 21)
    {
      CountValue = 0;
    }
    /* vTaskDelay(100): Blocking delay,This Task1 goes into a blocked state after invocation */
    vTaskDelay(100);
  }
}

/**
  * @brief  Get queue and semaphore data from queueset
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task2(void *pvParamters)
{
  /* Define a MemberHandle*/
  QueueSetMemberHandle_t MemberHandle = {0};
  uint8_t CountValue = 0;
  BaseType_t Err = 0;
  while(1)
  { 
    /* Get handle from QueueSetHandle,return a value to MemberHandle. */
    /* portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
    MemberHandle = xQueueSelectFromSet(QueueSetHandle, portMAX_DELAY);
    /* MemberHandle = QueueHandle,prepare reveive queue message */
    if(MemberHandle == QueueHandle)
    {
      /* Receive data from queue. portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      Err = xQueueReceive(MemberHandle, &CountValue, portMAX_DELAY);
      /* Err = pdTRUE,receive success. */
      if(Err == pdTRUE)
      {
        /* Output CountValue value */
        printf("Task2: Receive queue value = %d\r\n",CountValue);
      }
    }
    /* MemberHandle = QueueHandle,prepare take semaphore. */
    else if(MemberHandle == SemphrHandle)
    {
      /* Take semaphore. */
      /* portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      Err = xSemaphoreTake(MemberHandle, portMAX_DELAY);
      /* Err = pdTRUE,take success. */
      if(Err == pdTRUE)
      {
        printf("Task2: Take semaphore succuss!\r\n");
      }
    }
    else
    {
    }
  }
}

/**
  * @brief  Create queue、binary semaphore、queueset,Adds semaphore handles and queue handles to the queue set.
  * @param  None
  * @retval None
  */
static void QueueSetCreate()
{
  /* Create QueueSet,return a value to QueueSetHandle. */
  QueueSetHandle = xQueueCreateSet(QUEUEUSET_NUM);
  /* QueueSetHandle != NULL,create success. */
  if(QueueSetHandle != NULL)
  {
    printf("queue set create success!\r\n");
  }

  /* Create Queue,return a value to QueueHandle. */
  QueueHandle  = xQueueCreate(QUEUEU_NUM, sizeof(uint8_t));
  /* Create binary semaphore,return a value to SemphrHandle. */
  SemphrHandle = xSemaphoreCreateBinary();

  /* Add QueueHandle and SemphrHandle to QueueSetHandle */
  xQueueAddToSet(QueueHandle , QueueSetHandle);
  xQueueAddToSet(SemphrHandle, QueueSetHandle);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE; /* Select oscillator HSE, HSI, LSI, LSE */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                          /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                          /* HSI 1 frequency division */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;  /* Configure HSI clock 8MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                         /* Close HSE */
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* Close LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                         /* Close LSE */
  /*RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;                     /* Close PLL */
  /*RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_NONE;*/
  /*RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;*/
  /* Configure oscillator */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Clock source configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* Choose to configure clock HCLK, SYSCLK, PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS; /* Select HSISYS as the system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* AHB clock 1 division */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* APB clock 1 division */
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
  /* Infinite loop */
  while (1)
  {
  }
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file：pointer to the source file name
  * @param  line：assert_param error line source number
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
