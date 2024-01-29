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
#define DACx_CHANNEL                    DAC_CHANNEL_1

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef        DacHandle;
DAC_ChannelConfTypeDef   sConfig;
TIM_HandleTypeDef        htim;
TIM_MasterConfigTypeDef  sMasterConfig;
uint32_t aEscalator12bit[5] = {0x000,0x400,0x800,0xc00,0xfff};

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_TimConfig(void);
static void APP_DacConfig(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */ 
  HAL_Init();
  
  /* System clock configuration */
  APP_SystemClockConfig(); 
  
  /* DAC Config */
  APP_DacConfig();
  
  /* TIM Config */
  APP_TimConfig();
  while(1)
  {
    
  }
}

/**
  * @brief  DAC configure function
  * @param  None
  * @retval None
  */
static void APP_DacConfig(void)
{
  __HAL_RCC_DAC_CLK_ENABLE();
     
  DacHandle.Instance=DAC1;
  
  /* DAC Initializes */
  if (HAL_DAC_Init(&DacHandle) != HAL_OK)
  {

  }

  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  /* DAC Channel Configure */
  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DACx_CHANNEL) != HAL_OK)
  {

  }
  /* DAC Start with DMA */
  HAL_DAC_Start_DMA(&DacHandle,DACx_CHANNEL, (uint32_t *)aEscalator12bit, 5, DAC_ALIGN_12B_R);  
}
  
/**
  * @brief  TIM configure function
  * @param  None
  * @retval None
  */
static void APP_TimConfig(void)
{
  /* Enable TIM6 clock */
  __HAL_RCC_TIM6_CLK_ENABLE();
  htim.Instance = TIM6;

  htim.Init.Period            = 8000-1;                                /* Period = 8000-1  */               
  htim.Init.Prescaler         = 1000-1;                                /* Prescaler = 1000-1 */
  htim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;                /* ClockDivision = 0 */
  htim.Init.CounterMode       = TIM_COUNTERMODE_UP;                    /* Counter Direction = up */
  htim.Init.RepetitionCounter = 0;                                     /* Repetition = 0  */
  htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;        /* Auto-Reload register not buffered */
  HAL_TIM_Base_Init(&htim);                                            /* Initialize TIM6 */
 
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;                 /* Select Update Event as Trigger Source */
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;         /* Disable Master/Slave Mode  */    

  HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);        /* Configure TIM6 */

  HAL_TIM_Base_Start(&htim);                                           /* TIM6 Start */
}

/**
  * @brief  System clock configuration function.
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef  OscInitstruct = {0};
  RCC_ClkInitTypeDef  ClkInitstruct = {0};
  
  OscInitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE | 
                                  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSI48M;
  OscInitstruct.HSEState        = RCC_HSE_OFF;                              /* Disable HSE  */
/* OscInitstruct.HSEFreq         = RCC_HSE_16_32MHz; */                     /* HSE frequency range 16~32M */
  OscInitstruct.HSI48MState     = RCC_HSI48M_OFF;                           /* Disable HSI48M */
  OscInitstruct.HSIState        = RCC_HSI_ON;                               /* Enable HSI */
  OscInitstruct.LSEState        = RCC_LSE_OFF;                              /* Disable HSE */
/* OscInitstruct.LSEDriver       = RCC_LSEDRIVE_HIGH; */                    /* LSEDrive ：High */
  OscInitstruct.LSIState        = RCC_LSI_OFF;                              /* Disable LSI */
  OscInitstruct.PLL.PLLState    = RCC_PLL_OFF;                              /* Disable PLL */
/*  OscInitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE; */                   /* PLL clock source selection HSE */
/*  OscInitstruct.PLL.PLLMUL      = RCC_PLL_MUL6; */                        /* PLL clock source 6-fold frequency */
  /* Configure oscillator */
  if(HAL_RCC_OscConfig(&OscInitstruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  ClkInitstruct.ClockType       = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  ClkInitstruct.SYSCLKSource    = RCC_SYSCLKSOURCE_HSI;                 /* Select HSI as system clock */
  ClkInitstruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;                      /* AHB clock 1 division */
  ClkInitstruct.APB1CLKDivider  = RCC_HCLK_DIV1;                        /* APB1 clock 1 division */
  ClkInitstruct.APB2CLKDivider  = RCC_HCLK_DIV2;                        /* APB2 clock 1 division */
  /* Configure clock source */
  if(HAL_RCC_ClockConfig(&ClkInitstruct, FLASH_LATENCY_0) != HAL_OK)
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
