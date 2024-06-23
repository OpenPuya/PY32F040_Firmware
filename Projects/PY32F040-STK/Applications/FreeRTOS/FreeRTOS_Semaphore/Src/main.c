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
#include "semphr.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Create Semaphore */
QueueHandle_t BinarySemphoreHandle;
QueueHandle_t CountSemphoreHandle;
QueueHandle_t MutexSemphoreHandle;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define  COUNT_MAX             50
#define  COUNT_INITIAL         0
#define  SEMAPHORE_BINRAY      1
#define  SEMAPHORE_COUNT       0
#define  SEMAPHORE_MUTEX       0

/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void Task1(void *pvParamters);
static void Task2(void *pvParamters);
static void Task3(void *pvParamters);
static void SemaphoreCreate(void);

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

  /* Create semaphore */
  SemaphoreCreate();

  /* Create the tasks that are created using the original xTaskCreate() API function. */
  xTaskCreate( Task1, "Task1", 128, NULL, 1, NULL );  
  xTaskCreate( Task2, "Task2", 128, NULL, 2, NULL );
  xTaskCreate( Task3, "Task3", 128, NULL, 3, NULL );

  /* Start the scheduler. */
  vTaskStartScheduler();
}

/**
  * @brief  Give binary/Count semaphore and Give/Take mutex semaphore
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task1(void *pvParamters)
{
  while (1)
  {
#if (SEMAPHORE_BINRAY)
    static uint8_t CountValue = 0;
    BaseType_t Err = 0;
    {
      /* If CountValue = 10,prepare give binary semaphore */
      if(CountValue == 10)
      {
        /* Check BinarySemphoreHandle */
        if(BinarySemphoreHandle != NULL)
        {
          /* Give binary semaphore,return a value to Err */
          Err = xSemaphoreGive(BinarySemphoreHandle);
          /* Err = pdPASS,give success. */
          if(Err == pdPASS)
          {
            printf("Task1: binary semphore give success!\r\n");
          }
          /* Err = pdFAULT,give fail. */
          else
          {
            printf("Task1: binary semphore give fail! binary semphore = 1\r\n");
          }
        }
      }
      CountValue++;
      if(CountValue == 11)
      {
        /* If CountValue = 11,set CountValue = 0 */
        CountValue = 0;
      }
      /* vTaskDelay(200): Blocking delay,Task1 goes into a blocked state after invocation */
      vTaskDelay(200);
    }
#elif (SEMAPHORE_COUNT) 
    static uint8_t CountValue = 0;
    BaseType_t Err = 0;
    {
      /* If CountValue = 10,prepare send count semaphore */
      if(CountValue == 10)
      {
        /* Check CountSemphoreHandle */
        if(CountSemphoreHandle != NULL)
        {
          /* Give count semaphore,return a value to Err */
          Err = xSemaphoreGive(CountSemphoreHandle);
          /* Err = pdPASS,give success. */
          if(Err == pdPASS)
          {
            printf("Task1: count semphore send success!\r\n");
          }
          /* Err = pdFAULT,give fail,queue is full. */
          else
          {
            printf("Task1: count semphore send fail! count semphore = %d \r\n",COUNT_MAX);
          }
        }
      }
      CountValue++;
      /* If CountValue = 11,set CountValue = 0 */
      if(CountValue == 11)
      {
        CountValue = 0;
      }
      /* vTaskDelay(200): Blocking delay,Task1 goes into a blocked state after invocation */
      vTaskDelay(200);
    }
#elif (SEMAPHORE_MUTEX)
    {
      printf("Task1: low task take semaphore!\r\n");
      /* Take mutex semaphore. portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      xSemaphoreTake(MutexSemphoreHandle, portMAX_DELAY);
      printf("Task1: low task is running!\r\n");
      HAL_Delay(3000);
      printf("Task1: low task give semaphore!\r\n");
      /* Give mutex semaphore. */
      xSemaphoreGive(MutexSemphoreHandle);
      /* vTaskDelay(1000): Blocking delay,Task1 goes into a blocked state after invocation */
      vTaskDelay(1000);
    }
#else
    {
      printf("Task1: task_flag falult!");
    }
#endif
  }
}
/**
  * @brief  Take binary/Count semaphore and give/take mutex semaphore
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task2(void *pvParamters)
{
  while(1)
  { 
#if (SEMAPHORE_BINRAY)

    BaseType_t Err = 0;
    {
      /* Take binary semaphore,return a value to Err. portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      Err = xSemaphoreTake(BinarySemphoreHandle, portMAX_DELAY);
      /* Err = pdTRUE,take success. */
      if(Err == pdTRUE)
      {
        printf("Task2: binary semphore take success!\r\n");
      }
    }
#elif (SEMAPHORE_COUNT)
    BaseType_t Err = 0;
    {
      /* Take count semaphore,return a value to Err. portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      Err = xSemaphoreTake(CountSemphoreHandle, portMAX_DELAY);
      /* Err = pdTRUE,take success. */
      if(Err == pdTRUE)
      {
        /* Use uxSemaphoreGetCount() output count semaphore value */
        printf("Task2: count semphore = %d\r\n",(int)uxSemaphoreGetCount(CountSemphoreHandle));
      }
      /* vTaskDelay(1000): Blocking delay,Task2 goes into a blocked state after invocation */
      vTaskDelay(1000);
    }
#elif (SEMAPHORE_MUTEX)
    {
      printf("Task2: middle task is running!\r\n");
    }
    /* vTaskDelay(1000): Blocking delay,Task2 goes into a blocked state after invocation */
    vTaskDelay(1000);
#else
    {
      printf("Task2: Error Unknown Semapore!");
    }
#endif
  }
}

/**
  * @brief  Give/Take mutex semaphore
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task3(void *pvParamters)
{
  while(1)
  {
#if (SEMAPHORE_MUTEX)
    {
      printf("Task3: high task get semaphore!\r\n");
      /* Take mutex semaphore. */
      /* portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      xSemaphoreTake(MutexSemphoreHandle, portMAX_DELAY);
      printf("Task3: high task is running!\r\n");
      HAL_Delay(500);
      printf("Task3: high task release semaphore!\r\n");
      /* Give mutex semaphore. */
      xSemaphoreGive(MutexSemphoreHandle);
      /* vTaskDelay(1000): Blocking delay,Task3 goes into a blocked state after invocation */
      vTaskDelay(1000);
    }
#else
    {
      printf("Task3 underuse, delete!\r\n");
      /* Delet Task3 */
      vTaskDelete(NULL);
    }
#endif
  }
}
/**
  * @brief  Create binary/count/mutex emaphore
  * @param  None
  * @retval None
  */
static void SemaphoreCreate(void)
{
  /* Create binary semaphore,return value to BinarySemphoreHandle */
  BinarySemphoreHandle = xSemaphoreCreateBinary();
  /* If BinarySemphoreHandle != NULL, binary semaphore create success. */	
  if(BinarySemphoreHandle != NULL)
  {
    printf("binary semaphore create success!\r\n");
  }
  /* If BinarySemphoreHandle == NULL, binary semaphore create fail */
  else
  {
    printf("binary semaphore create fail!\r\n");
  }

  /* Create count semaphore,return value to CountSemphoreHandle */
  CountSemphoreHandle = xSemaphoreCreateCounting(COUNT_MAX, COUNT_INITIAL);
  /* If CountSemphoreHandle != NULL,count semaphore create success */
  if(CountSemphoreHandle != NULL)
  {
    printf("count semaphore create success!\r\n");
  }
  /* If CountSemphoreHandle != NULL,count semaphore create fail */
  else
  {
    printf("count semaphore create fail!\r\n");
  }

  /* Create mutex semaphore,return value to MutexSemphoreHandle */
  MutexSemphoreHandle = xSemaphoreCreateMutex();
  /* If MutexSemphoreHandle != NULL,mutex semaphore create success */
  if(MutexSemphoreHandle != NULL)
  {
    printf("mutex semaphore create success!\r\n");
  }
  /* If MutexSemphoreHandle != NULL,mutex semaphore create fail */
  else
  {
    printf("mutex semaphore create fail!\r\n");
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
