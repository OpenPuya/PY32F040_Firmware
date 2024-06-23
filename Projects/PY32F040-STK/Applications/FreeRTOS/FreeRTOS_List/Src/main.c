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
#include "list.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
List_t      TestList;     /* Define a List */
ListItem_t  ListItem1;    /* Define a ListItem */
ListItem_t  ListItem2;    /* Define a ListItem */
ListItem_t  ListItem3;    /* Define a ListItem */

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
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
  xTaskCreate( Task1, "Task1", 128, NULL, 1, NULL );  
  xTaskCreate( Task2, "Task2", 128, NULL, 2, NULL );

  /* Start the scheduler. */
  vTaskStartScheduler();
}

/**
  * @brief  Express task1 is running.
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task1(void *pvParamters)
{
  while (1)
  {
    /* vTaskDelay(500): Blocking delay,Task1 goes into a blocked state after invocation */
    vTaskDelay(500);
    printf("Task1 is running\r\n");
  }
}

/**
  * @brief  Create list、listitem；insert、remove、insertend list items. output log once 1s
  * @param  *pvParamters: The parameters passed to the task function when the task is created
  * @retval None
  */
static void Task2(void *pvParamters)
{
  /* Initiatial List and LisIitem */
  vListInitialise(&TestList);
  vListInitialiseItem(&ListItem1);
  vListInitialiseItem(&ListItem2);
  vListInitialiseItem(&ListItem3);
  /* Initiatial LisIitemx.xItemValue */
  ListItem1.xItemValue = 10;
  ListItem2.xItemValue = 30;
  ListItem3.xItemValue = 20;

  /* Print List and LisIitem primary address */	
  printf("/*print List and ListItem address*/\r\n");
  printf("Item\t\t\t\t\tAddress\r\n");
  printf("TestList->pxIndex\t\t\t0x%p\t\r\n",&TestList.pxIndex);
  printf("TestList->xListEnd\t\t\t0x%p\t\r\n",&TestList.xListEnd);
  printf("ListItem1\t\t\t\t0x%p\t\r\n",&ListItem1);
  printf("ListItem2\t\t\t\t0x%p\t\r\n",&ListItem2);
  printf("ListItem3\t\t\t\t0x%p\t\r\n",&ListItem3);
  printf("\r\n");

  /* Insert ListItem1,Insert in order of xItemValue value size */
  vListInsert((List_t *)&TestList, (ListItem_t *)&ListItem1);
  printf("Item\t\t\t\t\tAddress\r\n");
  printf("TestList->xListEnd->pxNext\t\t0x%p\t\r\n",TestList.xListEnd.pxNext);
  printf("ListItem1\t\t\t\t0x%p\t\r\n",&ListItem1.pxNext);
  printf("TestList->xListEnd->pxPrevious\t\t0x%p\t\r\n", TestList.xListEnd.pxPrevious);
  printf("ListItem1->pxPrevious\t\t\t0x%p\t\r\n",&ListItem1.pxPrevious);
  printf("\r\n");

  /* Insert ListItem2,Insert in order of xItemValue value size */
  vListInsert((List_t *)&TestList, (ListItem_t *)&ListItem2);
  printf("Item\t\t\t\t\tAddress\r\n");
  printf("TestList->xListEnd->pxNext\t\t0x%p\t\r\n",TestList.xListEnd.pxNext);
  printf("ListItem1\t\t\t\t0x%p\t\r\n",&ListItem1.pxNext);
  printf("ListItem2\t\t\t\t0x%p\t\r\n",&ListItem2.pxNext);
  printf("TestList->xListEnd->pxPrevious\t\t0x%p\t\r\n", TestList.xListEnd.pxPrevious);
  printf("ListItem1->pxPrevious\t\t\t0x%p\t\r\n",&ListItem1.pxPrevious);
  printf("ListItem2->pxPrevious\t\t\t0x%p\t\r\n",&ListItem2.pxPrevious);
  printf("\r\n");

  /* Insert ListItem3,Insert in order of xItemValue value size */
  vListInsert((List_t *)&TestList, (ListItem_t *)&ListItem3);
  printf("Item\t\t\t\t\tAddress\r\n");
  printf("TestList->xListEnd->pxNext\t\t0x%p\t\r\n",TestList.xListEnd.pxNext);
  printf("ListItem1\t\t\t\t0x%p\t\r\n",&ListItem1.pxNext);
  printf("ListItem2\t\t\t\t0x%p\t\r\n",&ListItem2.pxNext);
  printf("ListItem3\t\t\t\t0x%p\t\r\n",&ListItem3.pxNext);
  printf("TestList->xListEnd->pxPrevious\t\t0x%p\t\r\n", TestList.xListEnd.pxPrevious);
  printf("ListItem1->pxPrevious\t\t\t0x%p\t\r\n",&ListItem1.pxPrevious);
  printf("ListItem2->pxPrevious\t\t\t0x%p\t\r\n",&ListItem2.pxPrevious);
  printf("ListItem3->pxPrevious\t\t\t0x%p\t\r\n",&ListItem3.pxPrevious);
  printf("\r\n");

  /* Remove ListItem2 */
  uxListRemove((ListItem_t *)&ListItem2);
  printf("Item\t\t\t\t\tAddress\r\n");
  printf("TestList->xListEnd->pxNext\t\t0x%p\t\r\n",TestList.xListEnd.pxNext);
  printf("ListItem1\t\t\t\t0x%p\t\r\n",&ListItem1.pxNext);
  printf("ListItem3\t\t\t\t0x%p\t\r\n",&ListItem3.pxNext);
  printf("TestList->xListEnd->pxPrevious\t\t0x%p\t\r\n", TestList.xListEnd.pxPrevious);
  printf("ListItem1->pxPrevious\t\t\t0x%p\t\r\n",&ListItem1.pxPrevious);
  printf("ListItem3->pxPrevious\t\t\t0x%p\t\r\n",&ListItem3.pxPrevious);
  printf("\r\n");

  /* Insert ListItem2,insert list end */
  vListInsertEnd((List_t *)&TestList, (ListItem_t *)&ListItem2);
  printf("Item\t\t\t\t\tAddress\r\n");
  printf("TestList->xListEnd->pxNext\t\t0x%p\t\r\n",TestList.xListEnd.pxNext);
  printf("ListItem1\t\t\t\t0x%p\t\r\n",&ListItem1.pxNext);
  printf("ListItem2\t\t\t\t0x%p\t\r\n",&ListItem2.pxNext);
  printf("ListItem3\t\t\t\t0x%p\t\r\n",&ListItem3.pxNext);
  printf("TestList->xListEnd->pxPrevious\t\t0x%p\t\r\n", TestList.xListEnd.pxPrevious);
  printf("ListItem1->pxPrevious\t\t\t0x%p\t\r\n",&ListItem1.pxPrevious);
  printf("ListItem2->pxPrevious\t\t\t0x%p\t\r\n",&ListItem2.pxPrevious);
  printf("ListItem3->pxPrevious\t\t\t0x%p\t\r\n",&ListItem3.pxPrevious);
  printf("\r\n");

  while(1)
  {
    /* vTaskDelay(1000): Blocking delay,Task2 goes into a blocked state after invocation */
    vTaskDelay(1000);
    printf("Task2 is running\r\n");
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
