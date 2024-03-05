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
#define BUFFER_SIZE              32

/* Private variables ---------------------------------------------------------*/
uint32_t aSRC_Const_Buffer[BUFFER_SIZE];       /* Data transmission source buffer */
uint32_t aDST_Buffer[BUFFER_SIZE];             /* Data transmission destination buffer */
__IO uint32_t transferCompleteDetected=0;      /* When the transmission is completed, this position 1 */
__IO uint32_t transferErrorDetected=0;         /* When a transmission error occurs, this position 1 */
__IO uint32_t transferFailedDetected=0;        /* When there is an error in transmitting data, this position 1 */

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_DmaConfig(void);

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

  /* Configure Systemclock */
  APP_SystemClockConfig();
  
  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);
  
  /* Initialize data for DMA source buffer */
  for(uint8_t i=0; i<BUFFER_SIZE; i++)
  {
    aSRC_Const_Buffer[i]=i;
  }

  /* Configuring DMA */
  APP_DmaConfig();

  while (1)
  {   
    /* DMA transfer completed, but data is incorrect */
    if(transferFailedDetected == 1 && transferCompleteDetected == 1 )
    {
      while(1)
      {
        BSP_LED_Toggle(LED_GREEN);
        LL_mDelay(500);
      }
    }

    /* DMA transmission completed and data correct */
    if(transferFailedDetected == 0 && transferCompleteDetected == 1 )
    {
      BSP_LED_On(LED_GREEN);
      while(1)
      {
      }
    }

    /* DMA transmission error */
    if(transferErrorDetected == 1 )
    {
      BSP_LED_On(LED_GREEN);
      while(1)
      {
        BSP_LED_Toggle(LED_GREEN);
        LL_mDelay(500);
      }
    } 
  }
}

/**
  * @brief  DMA configuration function
  * @param  None
  * @retval None
  */
static void APP_DmaConfig(void)
{
  LL_DMA_InitTypeDef dma_initstruct;

  /* Enable DMA clock */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* Configure DMA function parameters */
  dma_initstruct.PeriphOrM2MSrcAddress  = (uint32_t)&aSRC_Const_Buffer;           /* Source Address Settings */
  dma_initstruct.MemoryOrM2MDstAddress  = (uint32_t)&aDST_Buffer;                 /* Target Address Settings */
  dma_initstruct.Direction              = LL_DMA_DIRECTION_MEMORY_TO_MEMORY;      /* M2M mode */
  dma_initstruct.Mode                   = LL_DMA_MODE_NORMAL;                     /* DMA cycle mode off */
  dma_initstruct.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_INCREMENT;                /* Enable peripheral address increment mode */
  dma_initstruct.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;                /* Memory address increment mode enable */
  dma_initstruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;                 /* Peripheral data width is 32 bits */
  dma_initstruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;                 /* Memory data width is 32 bits */
  dma_initstruct.NbData                 = BUFFER_SIZE;
  dma_initstruct.Priority               = LL_DMA_PRIORITY_HIGH;                   /* Channel priority is high */
  /* Initialize DMA */
  if (LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &dma_initstruct) != SUCCESS)
  {
    APP_ErrorHandler();
  }

  /* Enable interrupt */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /* Enable DMA */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

/**
  * @brief  DMA transfer completion callback function
  * @param  None
  * @retval None
  */
void APP_TransferCompleteCallback(void)
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
  * @brief  DMA transfer error callback function
  * @param  None
  * @retval None
  */
void APP_TransferErrorCallback(void)
{
  transferErrorDetected = 1;
}

/**
  * @brief  Configure system clock
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  /* Enable HSI */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB prescaler: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Select HSISYS as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB prescaler: PCLK = HCLK */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);

  /* Update the SystemCoreClock global variable(which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(8000000);
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
