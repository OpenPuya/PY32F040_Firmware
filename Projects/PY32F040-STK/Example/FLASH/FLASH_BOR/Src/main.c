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
FLASH_OBProgramInitTypeDef OBInitCfg={0};

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_FlashOBProgram(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();

  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);
  
  if(READ_BIT(FLASH->SDKR,FLASH_SDKR_BOR_EN | FLASH_SDKR_BOR_LEV) != OB_BOR_LEVEL_2p3_2p4)
  {
    APP_FlashOBProgram();
  }
  else
  {
    BSP_LED_On(LED_GREEN);
  }
  while(1);
}

/**
  * @brief  Write option space function.
  * @param  None
  * @retval None
  */
static void APP_FlashOBProgram(void)
{
  HAL_FLASH_Unlock();        /* Unlock FLASH */
  HAL_FLASH_OB_Unlock();     /* Unlock Option */
  
  OBInitCfg.OptionType = OPTIONBYTE_BOR;
  OBInitCfg.BORLevel= OB_BOR_LEVEL_2p3_2p4;
  
  /* Start option byte programming */
  HAL_FLASH_OBProgram(&OBInitCfg);
  
  HAL_FLASH_Lock();      /* Lock FLASH */
  HAL_FLASH_OB_Lock();   /* Lock Option */

  /* Generate a reset, option byte loading */
  HAL_FLASH_OB_Launch();
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
