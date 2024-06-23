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
static void APP_RccInit(void);
static void APP_CompInit(void);
static void APP_CompIt(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init(); 
  
  /* Initialization button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
  
  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);  

  /* Clock setting initialization */
  APP_RccInit();

  /* Initialize COMP */
  APP_CompInit();
 
  /* Enable interrupt */
  APP_CompIt();
  
  /* COMP1 Start */
  HAL_COMP_Start(&hcomp1);
  
  BSP_LED_On(LED_GREEN);

  /* Wait for the button to be pressed */
  while(BSP_PB_GetState(BUTTON_USER) != 0)
  {
  }
  
  BSP_LED_Off(LED_GREEN);
  
  /* Turn off Systick interrupt */
  HAL_SuspendTick(); 

  /* Entering STOP mode */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
  
  /* Enable Systick interrupt */
  HAL_ResumeTick();

  BSP_LED_On(LED_GREEN);
  HAL_Delay(1000);
  while (1)
  {
    BSP_LED_Toggle(LED_GREEN); 
    HAL_Delay(200);    
  }
}


/**
  * @brief  Comparator clock switching function.
  * @param  None
  * @retval None
  */
static void APP_RccInit(void)
{                    
  RCC_OscInitTypeDef RCCCONF = {0};
  RCC_PeriphCLKInitTypeDef COMPRCC = {0};
  
  RCCCONF.OscillatorType = RCC_OSCILLATORTYPE_LSI;        /* RCC uses internal LSI */
  RCCCONF.LSIState = RCC_LSI_ON;                          /* Enable LSI */
  RCCCONF.PLL.PLLState = RCC_PLL_NONE;                    /* PLL configuration unchanged */
  /*RCCCONF.PLL.PLLSource = RCC_PLLSOURCE_HSI;*/
  /*RCCCONF.PLL.PLLMUL = RCC_PLL_MUL2;*/

  HAL_RCC_OscConfig(&RCCCONF);                            /* Clock initialization */
    
  HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSI);               /* LSC Select LSI */
  
  COMPRCC.PeriphClockSelection = RCC_PERIPHCLK_COMP1;     /* Peripheral selection COMP1 */
  COMPRCC.Comp1ClockSelection = RCC_COMP1CLKSOURCE_LSC;   /* Peripheral independent clock source selection LSC */
  HAL_RCCEx_PeriphCLKConfig(&COMPRCC);                    /* RCC extension peripheral clock initialization */
}

/**
  * @brief  Comparator initialization function
  * @param  None
  * @retval None
  */
static void APP_CompInit(void)
{
  __HAL_RCC_COMP1_CLK_ENABLE();                                         /* Enable COMP1 clock */
    
  hcomp1.Instance = COMP1;                                              /* COMP1 */
  hcomp1.Init.InputMinus      = COMP_INPUT_MINUS_IO13;                  /* Negative input is VREF (1.2V) */
  hcomp1.Init.InputPlus       = COMP_INPUT_PLUS_IO4;                    /* Positive input selection is PA0 */
  hcomp1.Init.OutputPol       = COMP_OUTPUTPOL_NONINVERTED;             /* COMP1 polarity Sexual selection is not reverse */
  hcomp1.Init.Mode            = COMP_POWERMODE_MEDIUMSPEED;             /* COMP1 power consumption mode is selected as Medium speed mode */
  hcomp1.Init.Hysteresis      = COMP_HYSTERESIS_DISABLE;                /* Hysteresis function off */
  hcomp1.Init.WindowMode      = COMP_WINDOWMODE_DISABLE;                /* Window Mode Off */
  hcomp1.Init.TriggerMode     = COMP_TRIGGERMODE_IT_RISING;             /* The triggering method for closing window mode is the rising edge interrupt triggering */
  hcomp1.Init.DigitalFilter   = 0;                                      /* Disable DigitalFilter */
  hcomp1.Init.VrefDiv         = COMP_VREF_DIV_32_64VREF;                /* Vrefcmp div 32/64 */
  hcomp1.Init.VrefSrc         = COMP_VREF_SRC_VCCA;                     /* Vref select VCCA */
  
  /* COMP1 initialization */
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)                                 
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  Comparator interrupt enable function.
  * @param  None
  * @retval None
  */
static void APP_CompIt(void)
{
  /*COMP interrupt enable*/
  HAL_NVIC_EnableIRQ(ADC_COMP_IRQn);
  HAL_NVIC_SetPriority(ADC_COMP_IRQn, 0x01, 0);
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
