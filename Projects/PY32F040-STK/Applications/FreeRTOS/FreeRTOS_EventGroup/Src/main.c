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
EventGroupHandle_t EventGroupHandle;  /* Define a EventGroupHandle */

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define EVENTBIT_0  (1<<0)
#define EVENTBIT_1  (1<<1)

/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void Task1(void *pvParamters);
static void Task2(void *pvParamters);
static void EventGroupCreate(void);

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

  /* Create event group */
  EventGroupCreate();

  /* Create the tasks that are created using the original xTaskCreate() API function. */
  xTaskCreate( Task1, "Task1", 128, NULL, 1, NULL );  
  xTaskCreate( Task2, "Task2", 128, NULL, 2, NULL );

  /* Start the scheduler. */
  vTaskStartScheduler();
}

/**
  * @brief  Set assign bit to event group, assign bit is stored in "pxEventBits->uxEventBits"
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task1(void *pvParamters)
{
  uint8_t CountValue = 0;
  while (1)
  {
    /* If CountValue = 10,prepare write EVENTBIT_0 to EventGroupHandle */
    if(CountValue == 10)
    {
      xEventGroupSetBits(EventGroupHandle, EVENTBIT_0);
      printf("Task1: Set bit0\r\n");
    }
    /* If CountValue = 20,prepare write EVENTBIT_1 to EventGroupHandle */
    else if(CountValue == 20)
    {
      xEventGroupSetBits(EventGroupHandle, EVENTBIT_1);
      printf("Task1: Set bit1\r\n");
    }
    CountValue++;
    /* If CountValue count 20,set CountValue = 0 */
    if(CountValue == 21)
    {
      CountValue = 0;
    }
    /* vTaskDelay(100): Blocking delay,Task1 goes into a blocked state after invocation */
    vTaskDelay(100);
  }
}

/**
  * @brief  Set assign bit to event group, assign bit is stored in "pxEventBits->uxEventBits"
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task2(void *pvParamters)
{
  EventBits_t EventBit = 0;
  while(1)
  { 
    /* Gets the event bit waiting to be specified from event group */
    EventBit = xEventGroupWaitBits( EventGroupHandle,         /* Specifies the event flag group to wait for */
                                    EVENTBIT_0 | EVENTBIT_1,  /* Specifies the event bits to wait for */
                                    pdTRUE,                   /* Before exiting this function.
                                                                 pdTRUE: Clear EVENTBIT_0 | EVENTBIT_1;
                                                                 pdFALSE: Keep EVENTBIT_0 | EVENTBIT_1 */
                                    pdTRUE,                   /* This function exits the blocking state condition
                                                                 pdTRUE:  All waiting event bits are set to one; 
                                                                 pdFALSE: Set any one of the waiting event bits to one */
                                    portMAX_DELAY);           /* Don't get the data you want, keep waiting, task enters the blocked state. */
    printf("Task2: EventBit:0x%x\r\n",(unsigned int)EventBit);
  }
}

/**
  * @brief  Create event group.
  * @param  None
  * @retval None
  */
static void EventGroupCreate()
{
  /* Create event group,return value to BinarySemphoreHandle */
  EventGroupHandle = xEventGroupCreate();
  /* If EventGroupHandle != NULL,event group create success. */
  if(EventGroupHandle != NULL)
  {
    printf("Event group create success!\r\n");
  }
  /* If EventGroupHandle == NULL,event group create fail */
  else
  {
    printf("Event group create fail!\r\n");
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
