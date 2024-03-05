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
/* Private variables ---------------------------------------------------------*/
LL_I2S_InitTypeDef I2S_InitStruct = {0};

uint16_t      TxBuff[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
uint16_t      *pTxBuffPtr;
__IO uint16_t TxXferCount;

uint16_t RxBuff[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t      *pRxBuffPtr;
__IO uint16_t RxXferCount;

__IO ITStatus I2sReady = RESET;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define DATA_LENGTH       16

/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigI2SPin(void);
static void APP_ConfigDma(void);
static void APP_I2STransmit_DMA(uint16_t *pData, uint16_t Size);
static void APP_I2SReceive_DMA(uint16_t *pData, uint16_t Size);
static void APP_CheckEndOfTransfer(void);
static uint8_t APP_Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);
static void APP_LedBlinking(void);

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

  /* Configure system clock */
  APP_SystemClockConfig();

  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);

  /*  Wait for the button to be pressed  */
  while(BSP_PB_GetState(BUTTON_KEY) == 1)
  {
  }
  
  /* Configure I2S2 pins */
  APP_ConfigI2SPin();
  
  /*  Configure I2S2 DMA */
  APP_ConfigDma();
  
  /* Enable I2S2 clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  /* Initialize I2S2 ( Slave receive ) */
  I2S_InitStruct.Mode              = LL_I2S_MODE_SLAVE_RX; /* Slave receive */
  I2S_InitStruct.Standard          = LL_I2S_STANDARD_PHILIPS;
  I2S_InitStruct.DataFormat        = LL_I2S_DATAFORMAT_16B;
  I2S_InitStruct.MCLKOutput        = LL_I2S_MCLK_OUTPUT_DISABLE;
  I2S_InitStruct.AudioFreq         = LL_I2S_AUDIOFREQ_8K;
  I2S_InitStruct.ClockPolarity     = LL_I2S_POLARITY_HIGH;
  LL_I2S_Init(SPI2, &I2S_InitStruct);

  /* Receive data as slave */
  APP_I2SReceive_DMA((uint16_t *)RxBuff, DATA_LENGTH);
  /* Wait for receive completion  */
  while (I2sReady != SET)
  {
  }
  I2sReady = RESET;
  
  /* Initialize I2S2 ( Slave transmit ) */
  LL_I2S_Disable(SPI2); /* Disable to complete reinitialization */
  I2S_InitStruct.Mode              = LL_I2S_MODE_SLAVE_TX; /* Slave transmit */
  I2S_InitStruct.Standard          = LL_I2S_STANDARD_PHILIPS;
  I2S_InitStruct.DataFormat        = LL_I2S_DATAFORMAT_16B;
  I2S_InitStruct.MCLKOutput        = LL_I2S_MCLK_OUTPUT_DISABLE;
  I2S_InitStruct.AudioFreq         = LL_I2S_AUDIOFREQ_8K;
  I2S_InitStruct.ClockPolarity     = LL_I2S_POLARITY_HIGH;
  LL_I2S_Init(SPI2, &I2S_InitStruct);
  
  /* Transmit data as slave */
  APP_I2STransmit_DMA((uint16_t *)TxBuff, DATA_LENGTH);
  /* Wait for receive completion  */
  while (I2sReady != SET)
  {
  }
  I2sReady = RESET;

  /* Check the received data */
  APP_CheckEndOfTransfer();

  while (1)
  {
  }
}


/**
  * @brief  System clock configuration function
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

  /* Set AHB prescaler */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Configure HSISYS as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);

  /* Update system clock global variable SystemCoreClock (can also be updated by calling SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(8000000);
}

/**
  * @brief  Configure I2S2 pins
  * @param  None
  * @retval None
  */
static void APP_ConfigI2SPin(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**I2S2 pin configuration
  PB12------> I2S2_WS
  PB13------> I2S2_CK
  PB15------> I2S2_SD
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  DMA configuration function
  * @param  None
  * @retval None
  */
static void APP_ConfigDma(void)
{
  /* Enable DMA clock and syscfg clock */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  
  /* Use DMA channel LL_DMA_CHANNEL_1 for transmission */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH | \
                      LL_DMA_MODE_NORMAL                   | \
                      LL_DMA_PERIPH_NOINCREMENT  | \
                      LL_DMA_MEMORY_INCREMENT  | \
                      LL_DMA_PDATAALIGN_HALFWORD | \
                      LL_DMA_MDATAALIGN_HALFWORD | \
                      LL_DMA_PRIORITY_VERYHIGH);
  
 
  /* Use DMA channel LL_DMA_CHANNEL_2 for reception */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY | \
                      LL_DMA_MODE_NORMAL                   | \
                      LL_DMA_PERIPH_NOINCREMENT  | \
                      LL_DMA_MEMORY_INCREMENT  | \
                      LL_DMA_PDATAALIGN_HALFWORD | \
                      LL_DMA_MDATAALIGN_HALFWORD | \
                      LL_DMA_PRIORITY_LOW);

  /* Configure DMA request mapping */
  LL_SYSCFG_SetDMARemap(DMA1, LL_DMA_CHANNEL_1, LL_SYSCFG_DMA_MAP_SPI2_WR); /* I2S_SD:TX, DMA1_CH1 */
  LL_SYSCFG_SetDMARemap(DMA1, LL_DMA_CHANNEL_2, LL_SYSCFG_DMA_MAP_SPI2_RD); /* I2S_SD:RX, DMA1_CH2 */
  
  /* Set interrupt priority */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
  /*Enable interrupt*/
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  
  /* Set interrupt priority */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
  /*Enable interrupt*/
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/**
  * @brief  I2S2 transmission function using DMA
  * @param  None
  * @retval None
  */
static void APP_I2STransmit_DMA(uint16_t *pData, uint16_t Size)
{
  /* Record the transmission data and count */
  pTxBuffPtr = pData;
  TxXferCount = Size;
  
  /* Configure DMA channel 1 */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_ClearFlag_GI1(DMA1);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pTxBuffPtr);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_SPI_DMA_GetRegAddr(SPI2));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, TxXferCount);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  
  /* Enable I2S2 */
  LL_I2S_Enable(SPI2);
  
  /* Enable I2S2 DMA transmission request */
  LL_I2S_EnableDMAReq_TX(SPI2);
}

/**
  * @brief  I2S2 reception function using DMA
  * @param  None
  * @retval None
  */
static void APP_I2SReceive_DMA(uint16_t *pData, uint16_t Size)
{
  /* Record the received data and count */
  pRxBuffPtr = pData;
  RxXferCount = Size;
  
  /* Clear overrun flag for master reception */
  if (LL_I2S_GetTransferMode(SPI2) == LL_I2S_MODE_MASTER_RX)
  {
    LL_I2S_ClearFlag_OVR(SPI2);
  }
  
  /* Configure DMA channel 2 */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_ClearFlag_GI2(DMA1);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)pRxBuffPtr);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, LL_SPI_DMA_GetRegAddr(SPI2));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, RxXferCount);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  
  /* Enable I2S2 */
  LL_I2S_Enable(SPI2);
  
  /* Enable I2S2 DMA reception request */
  LL_I2S_EnableDMAReq_RX(SPI2);
}

/**
  * @brief  Wait for the end of transfer and verify data
  * @param  None
  * @retval None
  */
static void APP_CheckEndOfTransfer(void)
{
  /* Compare the transmitted data with the received data */
  if((APP_Buffercmp8((uint8_t*)TxBuff, (uint8_t*)RxBuff, DATA_LENGTH)))
  {
    /* Error handling */
    APP_LedBlinking();
  }
  else
  {
    /* If data received successfully, turn on the LED */
    BSP_LED_On(LED_GREEN);
  }
}

/**
  * @brief  Character comparison function
  * @param  pBuffer1：Pointer to the first buffer to be compared
  * @param  pBuffer2：Pointer to the second buffer to be compared
  * @param  BufferLength：Number of characters to compare
  * @retval 0: buffers are the same; 1: buffers are different
  */
static uint8_t APP_Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

/**
  * @brief  LED blinking function
  * @param  None
  * @retval None
  */
static void APP_LedBlinking(void)
{
  while (1)
  {
    BSP_LED_Toggle(LED_GREEN);; 
    LL_mDelay(200);
  }
}

/**
  * @brief  DMA channel 1 interrupt callback for transmission
  * @param  None
  * @retval None
  */
void APP_DmaChannel1IRQCallback(void)
{
  if((LL_DMA_IsActiveFlag_TC1(DMA1) == 1) && (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_1) == 1))
  {
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_ClearFlag_GI1(DMA1);
    
    LL_I2S_DisableDMAReq_TX(SPI2);
    TxXferCount = 0U;
    I2sReady = SET;
  }
}

/**
  * @brief  DMA channel 2/3 interrupt callback for reception
  * @param  None
  * @retval None
  */
void APP_DmaChannel2_3_IRQCallback(void)
{
  if((LL_DMA_IsActiveFlag_TC2(DMA1) == 1) && (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_2) == 1))
  {
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_ClearFlag_GI2(DMA1);
    
    LL_I2S_DisableDMAReq_RX(SPI2);
    RxXferCount = 0U;
    I2sReady = SET;
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
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
