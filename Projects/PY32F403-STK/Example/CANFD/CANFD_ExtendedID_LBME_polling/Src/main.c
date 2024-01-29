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
CANFD_HandleTypeDef CanfdHandle;
CANFD_FilterTypeDef CanfdFilter;

CANFD_TxHeaderTypeDef CanfdTxHeader;
uint8_t TxData[64] = {0};

CANFD_RxHeaderTypeDef CanfdRxHeader;
uint8_t RxData[64] = {0};

uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  uint32_t i;
  
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();
  
  /* Configure system clock */
  APP_SystemClockConfig(); 
  
  /* Initialize debug USART (used for printf) */
  BSP_USART_Config();
  
  /* Initialize CANFD */
  CanfdHandle.Instance                      = CANFD;
  CanfdHandle.Init.FrameFormat              = CANFD_FRAME_FD_NO_BRS;
  CanfdHandle.Init.Mode                     = CANFD_MODE_LOOPBACK_EXT_ACK; /* External loop mode, enabling self response, requires short circuiting of CAN transceiver pins */
  CanfdHandle.Init.CANFDProtocol            = CANFD_FD_ISO_11898;
  CanfdHandle.Init.Prescaler                = 1U; 
  CanfdHandle.Init.NominalSyncJumpWidth     = 4U;/* 1MHz */
  CanfdHandle.Init.NominalTimeSeg1          = 16U;
  CanfdHandle.Init.NominalTimeSeg2          = 4U;
  CanfdHandle.Init.DataSyncJumpWidth        = 4U;/* 1MHz */
  CanfdHandle.Init.DataTimeSeg1             = 16U;
  CanfdHandle.Init.DataTimeSeg2             = 4U;
  CanfdHandle.Init.SecondSamplePointOffset  = 0U;
  if (HAL_CANFD_Init(&CanfdHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Configure CANFD Filter */
  CanfdFilter.IdType         = CANFD_EXTENDED_ID;
  CanfdFilter.FilterChannel  = CANFD_FILTER_CHANNEL_0;
  CanfdFilter.Rank           = CANFD_FILTER_RANK_CHANNEL_NUMBER;
  CanfdFilter.FilterID       = 0x12345678; /* Consistent with sending message ID */
  CanfdFilter.FilterFormat   = 0xFFFFFFFF;
  CanfdFilter.MaskID         = 0x0; /* When receiving, only the 29 digits of the ID are compared */
  CanfdFilter.MaskFormat     = 0xFFFFFFFF;
  if (HAL_CANFD_ConfigFilter(&CanfdHandle, &CanfdFilter) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Start CANFD */
  if (HAL_CANFD_Start(&CanfdHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Add message to PTB */
  CanfdTxHeader.Identifier   = 0x12345678; 
  CanfdTxHeader.IdType       = CANFD_EXTENDED_ID;
  CanfdTxHeader.TxFrameType  = CANFD_DATA_FRAME;
  CanfdTxHeader.FrameFormat  = CANFD_FRAME_FD_NO_BRS;
  CanfdTxHeader.Handle       = 0x0;
  CanfdTxHeader.DataLength   = CANFD_DLC_BYTES_64;
  for (i = 0; i <64; i++)
  {
    TxData[i] = i;
  }
  if (HAL_CANFD_AddMessageToTxFifo(&CanfdHandle, &CanfdTxHeader, TxData, CANFD_TX_FIFO_PTB) != HAL_OK)
  {
    APP_ErrorHandler();
  } 
  
  /* Enable PTB transmission */
  if (HAL_CANFD_ActivateTxRequest(&CanfdHandle, CANFD_TXFIFO_PTB_SEND) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Waiting for reception to complete */
  while (__HAL_CANFD_GET_RX_FIFO_FILL_LEVEL(&CanfdHandle) == CANFD_RX_FIFO_EMPTY)
  {
  }
  
  /* Read received data */
  if (HAL_CANFD_GetRxMessage(&CanfdHandle, &CanfdRxHeader, RxData) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  /* Print received data */
  printf("decimal data length: %d\r\n", DLCtoBytes[CanfdRxHeader.DataLength]);
  printf("hexadecimal data: \r\n");
  for (i = 0; i < DLCtoBytes[CanfdRxHeader.DataLength]; i++)
  {
    printf("%x ", RxData[i]);
  }
  
  /* infinite loop */
  while (1)
  {
    
  }
}

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef  OscInitstruct = {0};
  RCC_ClkInitTypeDef  ClkInitstruct = {0};
  
  OscInitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE | 
                                  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSI48M;
  OscInitstruct.HSEState        = RCC_HSE_OFF;                              /* Close HSE */
/* OscInitstruct.HSEFreq         = RCC_HSE_16_32MHz; */                     /* Choose HSE frequency of 16-32MHz */
  OscInitstruct.HSI48MState     = RCC_HSI48M_OFF;                           /* Close HSI48M */
  OscInitstruct.HSIState        = RCC_HSI_ON;                               /* Enable HSI */
  OscInitstruct.LSEState        = RCC_LSE_OFF;                              /* Close LSE */
/* OscInitstruct.LSEDriver       = RCC_LSEDRIVE_HIGH; */                    /* Drive capability level: high */
  OscInitstruct.LSIState        = RCC_LSI_OFF;                              /* Close LSI */
  OscInitstruct.PLL.PLLState    = RCC_PLL_OFF;                              /* Close PLL */
/* OscInitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE; */                    /* PLL clock source selection HSE */
/* OscInitstruct.PLL.PLLMUL      = RCC_PLL_MUL6; */                         /* PLL clock source 6-fold frequency */
  /* Configure oscillator */
  if(HAL_RCC_OscConfig(&OscInitstruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  ClkInitstruct.ClockType       = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  ClkInitstruct.SYSCLKSource    = RCC_SYSCLKSOURCE_HSI;                 /* System clock selection HSI */
  ClkInitstruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;                      /* AHB clock 1 division */
  ClkInitstruct.APB1CLKDivider  = RCC_HCLK_DIV1;                        /* APB1 clock 1 division */
  ClkInitstruct.APB2CLKDivider  = RCC_HCLK_DIV2;                        /* APB2 clock 2 division */
  /* Configure Clock */
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
  /* Infinite loop */
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
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line)  */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
