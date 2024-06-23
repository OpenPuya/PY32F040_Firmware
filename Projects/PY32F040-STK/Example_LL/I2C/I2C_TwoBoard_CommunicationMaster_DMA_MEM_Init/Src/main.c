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
#define I2C_ADDRESS        0xA0     /* Local/Slave address */
#define I2C_SPEEDCLOCK     400000   /* Communication speed: 400K */
#define I2C_STATE_READY    0        /* Ready state */
#define I2C_STATE_BUSY_TX  1        /* Transmission state */
#define I2C_STATE_BUSY_RX  2        /* Reception state */

#define I2C_MEMADD_SIZE_8BIT            0x00000001U /* Slave internal address size: 8-bit */
#define I2C_MEMADD_SIZE_16BIT           0x00000010U /* Slave internal address size: 16-bit */

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
uint8_t aRxBuffer[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t         *pBuffPtr   = NULL;
__IO uint16_t   XferCount   = 0;
__IO uint32_t   Devaddress  = 0;
__IO uint32_t   State       = I2C_STATE_READY;

/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigI2cMaster(void);
static void APP_MasterTransmit_DMA_MEM(uint16_t DevAddress, uint16_t MemAddress,
  uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
static void APP_MasterReceive_DMA_MEM(uint16_t DevAddress, uint16_t MemAddress,
  uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
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
  /* Configure system clock */
  APP_SystemClockConfig();
  
  /* Initialize LED */
  BSP_LED_Init(LED_GREEN);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);
  
  /* Configure I2C */
  APP_ConfigI2cMaster();

  /* Wait for the button to be pressed */
  while(BSP_PB_GetState(BUTTON_KEY) == 1);
  
  /* Master transmits data */
  APP_MasterTransmit_DMA_MEM(I2C_ADDRESS, 0x00, I2C_MEMADD_SIZE_16BIT, (uint8_t *)aTxBuffer, sizeof(aTxBuffer));
  
  /* Wait for master to finish sending data */
  while (State != I2C_STATE_READY);
  
  /* Delay 100ms */
  LL_mDelay(100);
  
  /* Master receives data */  
  APP_MasterReceive_DMA_MEM(I2C_ADDRESS, 0x00, I2C_MEMADD_SIZE_16BIT, (uint8_t *)aRxBuffer, sizeof(aRxBuffer));
  
  /* Wait for master to finish receiving data */
  while (State != I2C_STATE_READY);
  
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
  * @brief  I2C configuration function
  * @param  None
  * @retval None
  */
static void APP_ConfigI2cMaster(void)
{
  /* Enable GPIOA peripheral clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /* Enable I2C1 peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  
  /* Enable DMA clock */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  
  /* Enable SYSCFG clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  /* Configure SCL pin: Alternative function, High speed, Open-drain, Pull-up */
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure SDA pin: Alternative function, High speed, Open-drain, Pull-up */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*  Reset I2C  */
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);
  
  /* Enable NVIC interrupt for I2C */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);
  
  /* Configure DMA request mapping */
  LL_SYSCFG_SetDMARemap(DMA1, LL_DMA_CHANNEL_1, LL_SYSCFG_DMA_MAP_I2C1_WR);
  LL_SYSCFG_SetDMARemap(DMA1, LL_DMA_CHANNEL_2, LL_SYSCFG_DMA_MAP_I2C1_RD);
  
  /* Initialize DMA channel 1 */
  LL_DMA_InitTypeDef DMA_InitStruct = {0};
  DMA_InitStruct.PeriphOrM2MSrcAddress  = 0x00000000U;
  DMA_InitStruct.MemoryOrM2MDstAddress  = 0x00000000U;
  DMA_InitStruct.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStruct.Mode                   = LL_DMA_MODE_NORMAL;
  DMA_InitStruct.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStruct.NbData                 = 0x00000000U;
  DMA_InitStruct.Priority               = LL_DMA_PRIORITY_VERYHIGH;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);
  
  /* Initialize DMA channel 2 */
  DMA_InitStruct.PeriphOrM2MSrcAddress  = 0x00000000U;
  DMA_InitStruct.MemoryOrM2MDstAddress  = 0x00000000U;
  DMA_InitStruct.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.Mode                   = LL_DMA_MODE_NORMAL;
  DMA_InitStruct.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStruct.NbData                 = 0x00000000U;
  DMA_InitStruct.Priority               = LL_DMA_PRIORITY_HIGH;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &DMA_InitStruct);
  
  /* Enable NVIC interrupt for DMA */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  
  /* I2C initialization */
  LL_I2C_InitTypeDef I2C_InitStruct = {0};
  I2C_InitStruct.PeripheralMode  = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed      = I2C_SPEEDCLOCK;
  I2C_InitStruct.DutyCycle       = LL_I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.OwnAddress1     = I2C_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_NACK;
  I2C_InitStruct.OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  
  /* Enable clock stretching */
  /* Reset value is clock stretching enabled */
  /* LL_I2C_EnableClockStretching(I2C1); */
  
  /* Enable general call */
  /* Reset value is general call disabled */
  /* LL_I2C_EnableGeneralCall(I2C1); */
}

/**
  * @brief  I2C transmission function
  * @param  DevAddress：Slave address
  * @param  MemAddress：Slave internal address
  * @param  MemAddSize：Size of slave internal address
  * @param  pData：Pointer to data to be sent
  * @param  Size：Size of data to be sent
  * @retval None
  */
static void APP_MasterTransmit_DMA_MEM(uint16_t DevAddress, uint16_t MemAddress,
  uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  /* Clear POS bit */
  LL_I2C_DisableBitPOS(I2C1);
  
  /* Assign slave address, data to be sent, data size, and state to global variables */
  pBuffPtr    = pData;
  XferCount   = Size;
  Devaddress  = DevAddress;
  State       = I2C_STATE_BUSY_TX;
  
  /* Enable DMA transfer interrupt */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  
  LL_DMA_ClearFlag_GI1(DMA1);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, Size);
  uint32_t dataRegAddr = LL_I2C_DMA_GetRegAddr(I2C1);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pData, dataRegAddr, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  
  LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
  
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  
  /* Generate start condition */
  LL_I2C_GenerateStartCondition(I2C1);
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  
  /* Send slave address */
  LL_I2C_TransmitData8(I2C1, (Devaddress & (uint8_t)(~0x01)));
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  LL_I2C_ClearFlag_ADDR(I2C1);
  
  /* Send slave internal address */
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    LL_I2C_TransmitData8(I2C1, (MemAddress & 0xFF));
  }
  else
  {
    LL_I2C_TransmitData8(I2C1, ((MemAddress & 0xFF00) >> 8));
    while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
    
    LL_I2C_TransmitData8(I2C1, (MemAddress & 0xFF));
  }
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  
  /* Enable I2C error interrupt */
  LL_I2C_EnableIT_ERR(I2C1);
  
  /* Enable DMA request */
  LL_I2C_EnableDMAReq_TX(I2C1);
}

/**
  * @brief  I2C reception function
  * @param  DevAddress：Slave address
  * @param  MemAddress：Slave internal address
  * @param  MemAddSize：Size of slave internal address
  * @param  pData：Pointer to data to be received
  * @param  Size：Size of data to be received
  * @retval None
  */
static void APP_MasterReceive_DMA_MEM(uint16_t DevAddress, uint16_t MemAddress,
  uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  /* Clear POS bit */
  LL_I2C_DisableBitPOS(I2C1);
  
  /* Assign slave address, data to be sent, and data size to global variables */
  pBuffPtr    = pData;
  XferCount   = Size;
  Devaddress  = DevAddress;
  State       = I2C_STATE_BUSY_RX;
  
  /* Enable DMA transfer interrupt */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  
  LL_DMA_ClearFlag_GI2(DMA1);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, Size);
  uint32_t dataRegAddr = LL_I2C_DMA_GetRegAddr(I2C1);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, dataRegAddr, (uint32_t)pData, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  
  LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
  
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  
  /* Enable acknowledge */
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  
  /* Generate start condition */
  LL_I2C_GenerateStartCondition(I2C1);
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  
  /* Send slave address (write)Send slave address (write) */
  LL_I2C_TransmitData8(I2C1, (Devaddress & (uint8_t)(~0x01)));
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  LL_I2C_ClearFlag_ADDR(I2C1);
  
  /* Send slave internal address */
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    LL_I2C_TransmitData8(I2C1, (MemAddress & 0xFF));
  }
  else
  {
    LL_I2C_TransmitData8(I2C1, ((MemAddress & 0xFF00) >> 8));
    while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
    
    LL_I2C_TransmitData8(I2C1, (MemAddress & 0xFF));
  }
  while(LL_I2C_IsActiveFlag_TXE(I2C1) != 1);
  
  /* Re-generate start condition*/
  LL_I2C_GenerateStartCondition(I2C1);
  while(LL_I2C_IsActiveFlag_SB(I2C1) != 1);
  
  /* Send slave address (read) */
  LL_I2C_TransmitData8(I2C1, (Devaddress | 0x1));
  while(LL_I2C_IsActiveFlag_ADDR(I2C1) != 1);
  LL_I2C_ClearFlag_ADDR(I2C1);
  
  if (XferCount == 1U)
  {
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
  }
  else
  {
    LL_I2C_EnableLastDMA(I2C1);
  }

  /* Enable I2C error interrupt */
  LL_I2C_EnableIT_ERR(I2C1);
  
  /* Enable DMA request */
  LL_I2C_EnableDMAReq_RX(I2C1);
}

/**
  * @brief  I2C interrupt callback function
  * @param  None
  * @retval None
  */
void APP_MasterI2cIRQCallback(void)
{
  
  /* Master send direction */
  if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_WRITE)
  {
    /* If DMA transmission is ongoing, do not check BTF flag */
    if (LL_I2C_IsEnabledDMAReq_TX(I2C1) != 1)
    {
      /* Set BTF flag */
      if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
      {
        if (XferCount == 0U)
        {
          LL_I2C_DisableIT_EVT(I2C1);
          LL_I2C_DisableIT_ERR(I2C1);
          
          LL_I2C_GenerateStopCondition(I2C1);
          State = I2C_STATE_READY;
        }
      }
    }
  }
  /* Master receive direction */
  else
  {
  }
}

/**
  * @brief  DMA interrupt callback function
  * @param  None
  * @retval None
  */
void APP_DmaIRQCallback(void)
{
  /* Transmission completed handling */
  if (State == I2C_STATE_BUSY_TX)
  {
    if ((LL_DMA_IsActiveFlag_TC1(DMA1) == 1) && (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_1) == 1))
    {
      /* Disable transfer complete interrupt */
      LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_1);
      LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_1);
      
      /* Clear transfer complete flag */
      LL_DMA_ClearFlag_TC1(DMA1);
      
      /* Disable I2C EVT and ERR interrupts */
      LL_I2C_DisableIT_EVT(I2C1);
      LL_I2C_DisableIT_ERR(I2C1);
      
      /* Disable I2C DMA requests */
      LL_I2C_DisableDMAReq_TX(I2C1);
      
      XferCount = 0U;
      
      /* Enable I2C EVT and ERR interrupts */
      LL_I2C_EnableIT_EVT(I2C1);
      LL_I2C_EnableIT_ERR(I2C1);
    }
  }
  /* Reception completed handling */
  else
  {
    if ((LL_DMA_IsActiveFlag_TC2(DMA1) == 1) && (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_2) == 1))
    {
      /* Disable transfer complete interrupt */
      LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_2);
      LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_2);
      
      /* Clear transfer complete flag */
      LL_DMA_ClearFlag_TC2(DMA1);
      
      /* Disable I2C EVT and ERR interrupts */
      LL_I2C_DisableIT_EVT(I2C1);
      LL_I2C_DisableIT_ERR(I2C1);
      
      /* I2C handling when transfer is complete */
      if (XferCount == (uint16_t)1)
      {
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
      }
      LL_I2C_GenerateStopCondition(I2C1);
      LL_I2C_DisableLastDMA(I2C1);
      LL_I2C_DisableDMAReq_RX(I2C1);
      XferCount = 0U;
      State = I2C_STATE_READY;
    }
  }
}

/**
  * @brief  Data verification function
  * @param  None
  * @retval None
  */
static void APP_CheckEndOfTransfer(void)
{
  /* Compare the transmitted data with the received data */
  if(APP_Buffercmp8((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, sizeof(aRxBuffer)))
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
    BSP_LED_Toggle(LED_GREEN);
    LL_mDelay(500);
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
