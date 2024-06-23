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
#include "event_groups.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TaskHandle_t Task1Handler;
TaskHandle_t Task2Handler;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define NOTIFY_SEMAPHORE        1
#define NOTIFY_EVENTGROUPS      0
#define NOTIFY_MESSAGE          0
#define EVENTBIT_0             (1<<0)
#define EVENTBIT_1             (1<<1)

/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void Task1(void *pvParamters);
static void Task2(void *pvParamters);

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
  xTaskCreate( Task1, "Task1", 128, NULL, 1, &Task1Handler );  
  xTaskCreate( Task2, "Task2", 128, NULL, 2, &Task2Handler );

  /* Start the scheduler. */
  vTaskStartScheduler();
}

/**
  * @brief  Task notification simulates releasing semaphore, sending event flag bits, and sending messages.
            Send or release data stored in "pxTCB->ulNotifiedValue[ uxIndexToNotify ]"
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task1(void *pvParamters)
{
  static uint8_t CountValue = 0;
  while (1)
  {
#if(NOTIFY_SEMAPHORE)
    {
      /* If CountValue = 10,prepare give a semaphore to Task2 */
      if(CountValue == 10)
      {
        /* Give a semaphore to Task2,semaphore value add 1 */
        xTaskNotifyGive(Task2Handler);
        printf("Task1: semaphore give success\r\n");
      }
      CountValue ++;
      /* If CountValue = 11,set CountValue = 0 */
      if(CountValue ==11)
      {
        CountValue = 0;
      }
      /* vTaskDelay(100): Blocking delay,Task1 goes into a blocked state after invocation */
      vTaskDelay(100);
    }
#elif(NOTIFY_EVENTGROUPS)
    {
      /* If CountValue = 10,send event flag bit(EVENTBIT_0) to Task2 */
      if(CountValue == 10)
      {
        printf("Task1: Bit0 set 1\r\n");
        /* Write assgin bit 1.pxTCB->ulNotifiedValue[ uxIndexToNotify ] |= EVENTBIT_0 */
        xTaskNotify(Task2Handler, EVENTBIT_0, eSetBits);
      }
      /* If CountValue = 20,send event flag bit(EVENTBIT_1) to Task2 */
      else if(CountValue == 20)
      {
        printf("Task1: Bit1 set 1\r\n");
        /* Write assgin bit 1.pxTCB->ulNotifiedValue[ uxIndexToNotify ] |= EVENTBIT_1 */
        xTaskNotify(Task2Handler, EVENTBIT_1, eSetBits);
      }
      CountValue ++;
      /* If CountValue = 21,set CountValue = 0 */
      if(CountValue ==21)
      {
        CountValue = 0;
      }
      /* vTaskDelay(100): Blocking delay,Task1 goes into a blocked state after invocation */
      vTaskDelay(100);
    }
#elif(NOTIFY_MESSAGE)
    {
      /* If CountValue = 10,send message to Task2 */
      if(CountValue == 10)
      {
        /* Write a byte.pxTCB->ulNotifiedValue[ uxIndexToNotify ] = CountValue */
        /* eSetValueWithOverwrite: Overwrite the way to update notification values*/
        xTaskNotify(Task2Handler, CountValue, eSetValueWithOverwrite);
        printf("Task1: message send success\r\n");
      }
      /* If CountValue = 20,send message to Task2 */
      else if(CountValue == 20)
      {
        /* Write a byte.pxTCB->ulNotifiedValue[ uxIndexToNotify ] = CountValue */
        /* eSetValueWithOverwrite: Overwrite the way to update notification values*/
        xTaskNotify(Task2Handler, CountValue, eSetValueWithOverwrite);
        printf("Task1: message send success\r\n");
      }
      CountValue ++;
      /* If CountValue = 21,set CountValue = 0 */
      if(CountValue ==21)
      {
        CountValue = 0;
      }
      /* vTaskDelay(100): Blocking delay,Task1 goes into a blocked state after invocation */
      vTaskDelay(100);
    }
#else
    {
    }
#endif
  }
}

/**
  * @brief  Task notify simulute receive semaphore、event group、message
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task2(void *pvParamters)
{
  while(1)
  { 
#if(NOTIFY_SEMAPHORE)
    uint32_t RevValue = 0;
    {
      /* Take notification value. */
      /* pdTURE: simulate binary semaphore. */
/*      RevValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); */ 
      /* pdFALSE: simulate count semaphore. */
      /* portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      RevValue = ulTaskNotifyTake(pdFALSE, portMAX_DELAY); 
      if(RevValue != 0)
      {
        printf("Task2: rev_value = %d\r\n",(unsigned int) RevValue);
      }
      /* vTaskDelay(2000): Blocking delay,Task2 goes into a blocked state after invocation */
      vTaskDelay(2000); /* this line use count semaphore; */
    }
#elif(NOTIFY_EVENTGROUPS)
    uint32_t NotifyValue = 0;
    {
      /* Get NotifyValue from pxTCB->ulNotifiedValue[uxIndexToNotify].Then set pxTCB->ulNotifiedValue[uxIndexToNotify] = 0 */
      /* portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      xTaskNotifyWait(0, 0xFFFFFFFF, &NotifyValue, portMAX_DELAY);
      /* If NotifyValue != 0,output EventBit. */
      if(NotifyValue & (EVENTBIT_0 | EVENTBIT_1))
      {
        printf("Task2: NotifyValue = %d\r\n",(unsigned int)NotifyValue);
        NotifyValue = 0;
      }
    }
#elif(NOTIFY_MESSAGE)
    uint32_t NotifyValue = 0;
    {
      /* Read abyte.NotifyValue = pxTCB->ulNotifiedValue[uxIndexToNotify].Then set pxTCB->ulNotifiedValue[uxIndexToNotify] = 0 */
      /* portMAX_DELAY: don't get the data you want, keep waiting, task enters the blocked state. */
      xTaskNotifyWait(0, 0xFFFFFFFF, &NotifyValue, portMAX_DELAY);
 
      printf("Task2: notify_value = %d\r\n", (unsigned int)NotifyValue);
    }
#else
    {
    }
#endif
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
