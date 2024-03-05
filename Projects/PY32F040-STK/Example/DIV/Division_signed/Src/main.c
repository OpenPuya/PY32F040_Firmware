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
DIV_CalculatedTypeDef Calculatervalue;
DIV_HandleTypeDef Divhandle;
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
  
  /* Initialize debugging serial port (used by printf) */
  BSP_USART_Config();
  
  /* Enable DIV */
  __HAL_RCC_DIV_CLK_ENABLE();
  
  Divhandle.Instance        = DIV;                /* Divider base address         */
  Calculatervalue.Sign      = DIV_MODE_SIGNED;    /* Signed division           */
  Calculatervalue.Dividend  = -0x7250A3FB;        /* Dividend -1,917,887,483 */
  Calculatervalue.Divisor   = 0x257D;             /* Divisor 9,597            */

  /* Start Divider */
  HAL_DIV_Calculate(&Divhandle, &Calculatervalue);
  
  /* Print division results */
  printf("Quotient:0x%x\r\nRemainder:0x%x\r\n",(int)(Calculatervalue.Quotient),(int)(Calculatervalue.Remainder));
  
  /* infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Error executing function.
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
  /* Users can add their own printing information as needed,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
