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
DMA_HandleTypeDef     DmaHandle;
uint32_t aSRC_Const_Buffer[BUFFER_SIZE];       /* Data transfer source buffer */
uint32_t aDST_Buffer[BUFFER_SIZE];             /* Data transfer destination buffer */
__IO uint32_t transferCompleteDetected=0;      /* When the transmission is complete,the bit set to 1 */
__IO uint32_t transferErrorDetected=0;         /* When a transmission error occurs,the bit set to 1 */
__IO uint32_t transferFailedDetected=0;        /* When data of transmission error,the bit set to 1 */

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_DmaConfig(void);
static void APP_TransferComplete(DMA_HandleTypeDef *DmaHandle);
static void APP_TransferError(DMA_HandleTypeDef *DmaHandle);
static void APP_SystemClockConfig(void);


/**
  * @brief  Main program
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  
  /* Configure Systemclock */
  APP_SystemClockConfig(); 
  
  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);
  
  /* Initialize DMA source buffer */
  for (uint8_t i = 0; i < BUFFER_SIZE; i++)
  {
    aSRC_Const_Buffer[i] = i;
  }

  /* Configure DMA */
  APP_DmaConfig();
    
  /* Start the DMA Transfer with interrupt enabled. */
  if (HAL_DMA_Start_IT(&DmaHandle, (uint32_t)&aSRC_Const_Buffer, (uint32_t)&aDST_Buffer, BUFFER_SIZE) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  while (1)
  {
    /* DMA transfer complete,but data error */
    if(transferFailedDetected == 1 && transferCompleteDetected == 1 )
    {
      while(1)
      {
        BSP_LED_Toggle(LED_GREEN);
        HAL_Delay(500);
      }
    }

    /* DMA transfer complete,and data is correct */
    if(transferFailedDetected == 0 && transferCompleteDetected == 1 )
    {
      BSP_LED_On(LED_GREEN);
      while(1)
      {
      }
    }

    /* DMA transfer error */
    if(transferErrorDetected == 1 )
    {
      BSP_LED_On(LED_GREEN);
      while(1)
      {
        BSP_LED_Toggle(LED_GREEN);
        HAL_Delay(500);
      }
    }
  }
}  
  
/**
  * @brief  Configure DMA
  * @param  None
  * @retval None
  */
static void APP_DmaConfig(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();                               /*Enable DMA1 clock*/
  
  DmaHandle.Init.Direction = DMA_MEMORY_TO_MEMORY;          /* M2M mode */
  DmaHandle.Init.PeriphInc = DMA_PINC_ENABLE;               /* Enable peripheral address increment mode */
  DmaHandle.Init.MemInc = DMA_MINC_ENABLE;                  /* Enable Memory address increment mode */
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; /* Peripheral data width is 32 bits */
  DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;    /* Memory data width is 32 bits */
  DmaHandle.Init.Mode = DMA_NORMAL;                         /* DMA cycle mode off */
  DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;              /* Channel priority is high */

  DmaHandle.Instance = DMA1_Channel1;                       /* Select DMA channel 1 */
  /* DMA initialization */
  if (HAL_DMA_Init(&DmaHandle) != HAL_OK)                   
  {
    APP_ErrorHandler();
  }

  /* Select the callback function to call after error transmission and correct transmission */
  HAL_DMA_RegisterCallback(&DmaHandle, HAL_DMA_XFER_CPLT_CB_ID, APP_TransferComplete);
  HAL_DMA_RegisterCallback(&DmaHandle, HAL_DMA_XFER_ERROR_CB_ID, APP_TransferError);
  /* DMA channel 1 interrupt enable */
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief  DMA transfer completion function
  * @param  DmaHandle：DMA handle
  * @retval None
  */
static void APP_TransferComplete(DMA_HandleTypeDef *DmaHandle)
{
  for(uint16_t i=0 ; i<BUFFER_SIZE; i++)
  {
    if(aDST_Buffer[i] != aSRC_Const_Buffer[i])
    {
      transferFailedDetected=1;
      break;
    }
  }
  transferCompleteDetected=1;
}

/**
  * @brief  DMA transfer error function
  * @param  DmaHandle：DMA handle
  * @retval None
  */
static void APP_TransferError(DMA_HandleTypeDef *DmaHandle)
{
  transferErrorDetected = 1;
}

/**
  * @brief   Configure systemclock
  * @param   None
  * @retval  None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef  OscInitstruct = {0};
  RCC_ClkInitTypeDef  ClkInitstruct = {0};
  
  OscInitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE | 
                                  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSI48M;
  OscInitstruct.HSEState        = RCC_HSE_OFF;                              /* HSE OFF */
/* OscInitstruct.HSEFreq         = RCC_HSE_16_32MHz; */                     /* HSE clock range 16~32MHz */
  OscInitstruct.HSI48MState     = RCC_HSI48M_OFF;                           /* Disable HSI48M clock */
  OscInitstruct.HSIState        = RCC_HSI_ON;                               /* HSI ON */
  OscInitstruct.LSEState        = RCC_LSE_OFF;                              /* LSE OFF */
/* OscInitstruct.LSEDriver       = RCC_LSEDRIVE_HIGH; */                    /* LSE high drive capability */
  OscInitstruct.LSIState        = RCC_LSI_OFF;                              /* LSI OFF */
  OscInitstruct.PLL.PLLState    = RCC_PLL_OFF;                              /* PLL OFF */
/*  OscInitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE; */                   /* HSE oscillator clock selected as PLL clock entry */
/*  OscInitstruct.PLL.PLLMUL      = RCC_PLL_MUL6; */                        /* PLLVCO = PLL clock entry x 6 */
  /* Initialize the RCC Oscillators */
  if(HAL_RCC_OscConfig(&OscInitstruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  ClkInitstruct.ClockType       = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  ClkInitstruct.SYSCLKSource    = RCC_SYSCLKSOURCE_HSI;                 /* Select HSI as system clock */
  ClkInitstruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;                      /* SYSCLK not divided: HCLK=SYSCLK */
  ClkInitstruct.APB1CLKDivider  = RCC_HCLK_DIV1;                        /* HCLK not divided: PCLK1=HCLK */
  ClkInitstruct.APB2CLKDivider  = RCC_HCLK_DIV2;                        /* HCLK divided by 2: PCLK2=HCLK/2 */
  /* Set clock source */
  if(HAL_RCC_ClockConfig(&ClkInitstruct, FLASH_LATENCY_0) != HAL_OK)
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
