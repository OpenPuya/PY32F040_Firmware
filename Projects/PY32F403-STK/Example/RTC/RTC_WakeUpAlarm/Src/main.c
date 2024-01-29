/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
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
RTC_HandleTypeDef RtcHandle;
RTC_TimeTypeDef RTCtime;
uint32_t gsecond, gminute, ghour;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_RtcHandle(void);
static void APP_RtcSetAlarm_IT(uint32_t Sec, uint32_t Min, uint32_t Hour);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  
  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);

  /* Initialization button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);

  /* Initialize debugging serial port (used by printf) */
  BSP_USART_Config();
  
  /* RTC initialization */
  APP_RtcHandle();
  
  /* Turn on LED */
  BSP_LED_On(LED_GREEN);
  
  /* Wait for button press */
  while (BSP_PB_GetState(BUTTON_USER) != 0)
  {
  }
  
   /* Turn off LED */
  BSP_LED_Off(LED_GREEN);
  
  while (1)
  {
    /* Wait for synchronization */
    HAL_RTC_WaitForSynchro(&RtcHandle);
    
    /* Get current RTC time in binary format */
    HAL_RTC_GetTime(&RtcHandle, &RTCtime, RTC_FORMAT_BIN);
    
    /* Set RTC alarm interrupt */
    ghour = RTCtime.Hours;
    gminute = RTCtime.Minutes;
    gsecond = RTCtime.Seconds;
    APP_RtcSetAlarm_IT(gsecond, gminute, ghour);
    
    /* Suspend systick interrupt */
    HAL_SuspendTick();
    
    /* Enter STOP mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    
    /* Resume systick interrupt */
    HAL_ResumeTick();
  }
}

/**
  * @brief  RTC initialization function, sets RTC to January 1, 2022, Saturday, 00:00:00
  * @param  None
  * @retval None
  */
static void APP_RtcHandle(void)
{
  RTC_TimeTypeDef Timeinit;

  RtcHandle.Instance = RTC;                               /* Select RTC */
  RtcHandle.Init.AsynchPrediv = RTC_AUTO_1_SECOND;        /* Automatic calculation of 1 S time base for RTC */
  
  /* RTC deinitialization */
  HAL_RTC_DeInit(&RtcHandle);
  
  /* RTC initialization */
  HAL_RTC_Init(&RtcHandle);
  
  /*Set RTC current time：2022-1-1-00:00:00*/
  RtcHandle.DateToUpdate.Year = 22;                       /* Year 22 */
  RtcHandle.DateToUpdate.Month = RTC_MONTH_JANUARY;       /* January */
  RtcHandle.DateToUpdate.Date = 1;                        /* 1st day */
  RtcHandle.DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;  /* Saturday */
  Timeinit.Hours = 0x00;                                /* 0 hours */
  Timeinit.Minutes = 0x00;                              /* 0 minutes */
  Timeinit.Seconds = 0x00;                              /* 0 seconds */
  HAL_RTC_SetTime(&RtcHandle, &Timeinit, RTC_FORMAT_BIN);
}

/**
  * @brief  Set RTC alarm interrupt
  * @param  Sec：seconds；
  * @param  Min：minutes；
  * @param  Hour：hours；
  * @retval None
  */
static void APP_RtcSetAlarm_IT(uint32_t Sec, uint32_t Min, uint32_t Hour)
{
  RTC_AlarmTypeDef Alarminit;
  /*00:00:5*/
  RtcHandle.Instance = RTC;
  Alarminit.AlarmTime.Hours = Hour;                           /* hours */
  Alarminit.AlarmTime.Minutes = Min;                          /* minutes */
  Alarminit.AlarmTime.Seconds = Sec + 1;                      /* seconds */
  HAL_RTC_SetAlarm_IT(&RtcHandle, &Alarminit, RTC_FORMAT_BIN);
}

/**
  * @brief  RTC alarm event callback function, LED turns on when exiting low-power mode
  * @param  hrtc：RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  BSP_LED_Toggle(LED_GREEN);
  printf("%02d:%02d:%02d\r\n", ghour, gminute, gsecond);
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
