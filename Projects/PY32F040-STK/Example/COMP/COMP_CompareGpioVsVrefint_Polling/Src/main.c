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
COMP_HandleTypeDef  hcomp1;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();       
  
  BSP_LED_Init(LED_GREEN);  

  hcomp1.Instance = COMP1;                                              /* COMP1 */
  hcomp1.Init.InputMinus      = COMP_INPUT_MINUS_IO13;                  /* Negative input is Vrefint (1.2V) */
  hcomp1.Init.InputPlus       = COMP_INPUT_PLUS_IO4;                    /* Positive input selection is PA0 */
  hcomp1.Init.OutputPol       = COMP_OUTPUTPOL_NONINVERTED;             /* COMP1 polarity Sexual selection is not reverse */
  hcomp1.Init.Mode            = COMP_POWERMODE_HIGHSPEED;               /* COMP1 power consumption mode is selected as High speed mode */
  hcomp1.Init.Hysteresis      = COMP_HYSTERESIS_DISABLE;                /* Hysteresis function off */
  hcomp1.Init.WindowMode      = COMP_WINDOWMODE_DISABLE;                /* Window Mode Off */
  hcomp1.Init.TriggerMode     = COMP_TRIGGERMODE_NONE;                  /* COMP1 external initialization not enabled */
  hcomp1.Init.VrefDiv         = COMP_VREF_DIV_32_64VREF;                /* Vrefcmp div 32/64 */
  hcomp1.Init.VrefSrc         = COMP_VREF_SRC_VCCA;                     /* Vref select VCCA */
  
  /* COMP1 initialization */
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)                                 
  {
    APP_ErrorHandler();
  }
  /* COMP1 Start */
  HAL_COMP_Start(&hcomp1); 

  while (1)
  {
    if(HAL_COMP_GetOutputLevel(&hcomp1) == 1)
    {
      BSP_LED_On(LED_GREEN);
    }
    else
    {
      BSP_LED_Off(LED_GREEN);
    }      
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
