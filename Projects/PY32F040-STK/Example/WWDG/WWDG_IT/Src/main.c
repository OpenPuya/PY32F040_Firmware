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
WWDG_HandleTypeDef   WwdgHandle;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Main program
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();                                         

  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);

  /* Initialize WWDG */
  WwdgHandle.Instance = WWDG;                         /* Select WWDG */
  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_8;       /* WWDG counter clock = (PCLK1/4096)/8 */
  WwdgHandle.Init.Window    = 127;                    /* WWDG window value */
  WwdgHandle.Init.Counter   = 127;                    /* WWDG free-running downcounter value(7 bit) */
  WwdgHandle.Init.EWIMode   = WWDG_EWI_ENABLE;        /* EWI Enable */
  /* Initialize WWDG */ 
  if (HAL_WWDG_Init(&WwdgHandle) != HAL_OK)           
  {
    APP_ErrorHandler();
  }

  /* Refresh the WWDG. */
  if (HAL_WWDG_Refresh(&WwdgHandle) != HAL_OK)        
  {
    APP_ErrorHandler();
  }

  while (1)
  {
    /* Delay 500ms */
    HAL_Delay(500);
    
    /* Toggle LED */
    BSP_LED_Toggle(LED_GREEN);
  }
}

/**
  * @brief  Early Wakeup Callback
  * @param  hwwdg：WWDG handle
  * @retval None
  */
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
  /* Refresh the WWDG */
  if (HAL_WWDG_Refresh(hwwdg) != HAL_OK)              
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  Error handling function
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
