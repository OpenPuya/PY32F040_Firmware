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
#include "py32f040xx_ll_Start_Kit.h"

/* Private define ------------------------------------------------------------*/
#define RSTPIN_MODE_GPIO
/* #define RSTPIN_MODE_RST */ 
 
/* Private variables ---------------------------------------------------------*/
FLASH_OBProgramInitTypeDef OBInitCfg={0};

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_FlashOBProgram(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Enable SYSCFG and PWR clocks */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  
  /* Initialize systick */
  HAL_Init();

  /* Initialize LED */  
  BSP_LED_Init(LED_GREEN);
  
  /* Initialize Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  
  /* Wait for the button to be pressed */
  while (BSP_PB_GetState(BUTTON_USER))
  {
  }

  /* Judging RST pins */
#if defined(RSTPIN_MODE_GPIO)
  if( READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE) == OB_RESET_MODE_RESET)
#else
  if( READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE) == OB_RESET_MODE_GPIO)
#endif
  {
    /* Write Option */
    APP_FlashOBProgram();
  }
  
  while(1)
  {
#if defined(RSTPIN_MODE_GPIO)
    if(READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE)== OB_RESET_MODE_GPIO )
#else
    if(READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE)== OB_RESET_MODE_RESET )
#endif
    {
      BSP_LED_On(LED_GREEN);
      while(1)
      {
      }
    }
  }
}

/**
  * @brief  Write option space function
  * @param  None
  * @retval None
  */
static void APP_FlashOBProgram(void)
{
  HAL_FLASH_Unlock();        /* Unlock FLASH */
  HAL_FLASH_OB_Unlock();     /* Unlock Option */
  
  OBInitCfg.OptionType = OPTIONBYTE_USER;
  
  OBInitCfg.USERType =    OB_USER_IWDG_SW | OB_USER_WWDG_SW | OB_USER_NRST_MODE | OB_USER_nBOOT1 | OB_USER_IWDG_STOP;

#if defined(RSTPIN_MODE_GPIO)
  /* Software mode watchdog/GPIO function/System memory as startup area/IWDG STOP mode continues counting */
  OBInitCfg.USERConfig = OB_IWDG_SW | OB_WWDG_SW | OB_RESET_MODE_GPIO | OB_BOOT1_SYSTEM | OB_IWDG_STOP_ACTIVE;
#else
  /* Software mode watchdog/RST function/System memory as startup area/IWDG STOP mode continues counting */
  OBInitCfg.USERConfig = OB_IWDG_SW | OB_WWDG_SW | OB_RESET_MODE_RESET | OB_BOOT1_SYSTEM | OB_IWDG_STOP_ACTIVE;
#endif
  
  /* Start option byte programming */
  HAL_FLASH_OBProgram(&OBInitCfg);
  
  HAL_FLASH_Lock();      /* Lock FLASH */
  HAL_FLASH_OB_Lock();   /* Lock Option */

  /* Generate a reset, option byte loading */
  HAL_FLASH_OB_Launch();
}

/**
  * @brief  Error handling function
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
