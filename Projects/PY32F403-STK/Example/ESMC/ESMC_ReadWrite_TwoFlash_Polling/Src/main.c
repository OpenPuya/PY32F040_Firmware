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
#define BUFFERSIZE  2048*2

/* Private variables ---------------------------------------------------------*/
ESMC_HandleTypeDef hesmc;
ESMC_InitTypeDef ESMC_initTypeDef;

uint8_t aRxBuffer[BUFFERSIZE];
uint8_t aTxBuffer[BUFFERSIZE];

uint32_t rxbuf[1024];
uint8_t aStatus[8];
uint8_t flashStatus;
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_EsmcInit(void);
static void APP_WriteEnable(void);
static void APP_EraseSector(uint32_t address);
static void APP_PageProgram(uint8_t *pdata, uint32_t addr, uint32_t Nbytes);
static void APP_DataRead(uint8_t *pData, uint32_t address, uint32_t nBytes);
static HAL_StatusTypeDef APP_DataCheck(uint8_t *pSrc, uint8_t *pDst, uint32_t length);
static void APP_SetFlashStatus(uint8_t status, uint8_t comm);
static uint8_t APP_GetFlashStatus(uint8_t comm);
static void APP_GetTwoFlashStatus(uint8_t command,uint8_t *pData);
static void APP_FlashWaitReady(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  uint32_t i, cnt, progAddr;
  uint8_t *pData;
  uint32_t address = 0;

  /* Reset of all peripherals, Initializes the Systick. */ 
  HAL_Init();
  
  BSP_LED_Init(LED_GREEN);

  /* Test Data Initializes */
  for(i = 0; i < BUFFERSIZE*2; i++)
  {
    aTxBuffer[i] = i%256;
  }

  /* Initialize ESMC */
  APP_EsmcInit();

  /* Write enable */
  APP_WriteEnable();
  
  /* Wait Flash Ready */
  APP_FlashWaitReady();

  /* Sector Erase */
  APP_EraseSector(address);

  /* Wait Flash Ready */
  APP_FlashWaitReady();
  
  flashStatus = APP_GetFlashStatus(P25Q64_READ_STATUS_REG_1_CMD);
  flashStatus |= 0x02;   /* Set Status QE */
  
  /* Write enable */
  APP_WriteEnable();
  
  /* Wait Flash Ready */
  APP_FlashWaitReady();
  
  /* Enable QE */
  APP_SetFlashStatus(flashStatus, P25Q64_WRITE_STATUS_REG_1_CMD);
  
  /* Wait Flash Ready */
  APP_FlashWaitReady();

  /* Page Program */
  cnt = BUFFERSIZE;
  progAddr = address;
  pData = aTxBuffer;
  while (cnt/(P25Q64_PAGE_SIZE*2) > 0)
  {
    APP_PageProgram(pData, progAddr, P25Q64_PAGE_SIZE*2);
    
    /* Wait Flash Ready */
    APP_FlashWaitReady();

    cnt      -= P25Q64_PAGE_SIZE*2;
    progAddr += P25Q64_PAGE_SIZE;
    pData    += P25Q64_PAGE_SIZE*2;

  }
  if (cnt > 0)
  {   
    APP_PageProgram(pData, progAddr, cnt);
    
    /* Wait Flash Ready */
    APP_FlashWaitReady();
  }
  
  /* Read data from flash */
  APP_DataRead(aRxBuffer, address, BUFFERSIZE);
  
  /* Compare the read data with write data */
  if (HAL_OK != APP_DataCheck(aRxBuffer, aTxBuffer, BUFFERSIZE))
  {
    BSP_LED_Off(LED_GREEN);
    APP_ErrorHandler();
  }

  /* Data verification correct, LED flashing */
  while(1)
  {
    HAL_Delay(500);
    BSP_LED_Toggle(LED_GREEN);
  }
}

/**
  * @brief  ESMC initialization function
  * @param  None
  * @retval None
  */
static void APP_EsmcInit()
{
  __HAL_RCC_ESMC_CLK_ENABLE();

  hesmc.Instance=ESMC;
  ESMC_initTypeDef.ClockPrescaler=0x8;
  ESMC_initTypeDef.ClockMode=ESMC_CLOCK_MODE_0;
  ESMC_initTypeDef.DualFlash=ESMC_DUALFLASH_ENABLE;
  hesmc.Init=ESMC_initTypeDef;
  HAL_ESMC_Init(&hesmc);
}

/**
  * @brief  Get the flash status register value
  * @param  command Status Command Value
  * @retval None
  */
static uint8_t APP_GetFlashStatus(uint8_t command)
{
  ESMC_CommandTypeDef cmdStc;
  uint8_t status;

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_SINGLE;
  cmdStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  cmdStc.Instruction       = command;
  cmdStc.AddressMode       = ESMC_ADDRESS_NONE;
  cmdStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  cmdStc.DummyCycles       = 0;
  cmdStc.DdrMode           = ESMC_DDR_DISABLE;
  cmdStc.DataMode          = ESMC_DATA_READ;
  cmdStc.NbData            = 1;
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0 | ESMC_SELECT_PIN_CS1 ;
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x0000FFFF);

  HAL_ESMC_Receive(&hesmc, &status, 0x0000FFFF);

  return status;
}

/**
  * @brief  Get the flash status register value
  * @param  command Status Command Value
  * @retval None
  */
static void APP_GetTwoFlashStatus(uint8_t command,uint8_t *pData1)
{
  ESMC_CommandTypeDef cmdStc;

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_OCTAL;
  cmdStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  cmdStc.Instruction       = command;
  cmdStc.AddressMode       = ESMC_ADDRESS_NONE;
  cmdStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  cmdStc.DummyCycles       = 0;
  cmdStc.DdrMode           = ESMC_DDR_DISABLE;
  cmdStc.DataMode          = ESMC_DATA_READ;
  cmdStc.NbData            = 8;
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0 | ESMC_SELECT_PIN_CS1 ;
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x0000FFFF);

  HAL_ESMC_Receive(&hesmc, pData1, 0x0000FFFF);
}

/**
  * @brief  Wait Flash Ready 
  * @retval None
  */
static void APP_FlashWaitReady(void)
{
  /* Wait Flash Ready */
  APP_GetTwoFlashStatus(P25Q64_READ_STATUS_REG_CMD,aStatus);

  while(((aStatus[7]&0x2)==0x2) || ((aStatus[7]&0x20)==0x20) )
  {
    APP_GetTwoFlashStatus(P25Q64_READ_STATUS_REG_CMD,aStatus);
  }
}

/**
  * @brief  Set the flash status register value
  * @param  status  status data
  * @param  command Status Command Value
  * @retval None
  */
static void APP_SetFlashStatus(uint8_t status, uint8_t command)
{
  ESMC_CommandTypeDef cmdStc;

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_OCTAL;
  cmdStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  cmdStc.Instruction       = command;
  cmdStc.AddressMode       = ESMC_ADDRESS_NONE;
  cmdStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  cmdStc.DummyCycles       = 0x0;
  cmdStc.DdrMode           = ESMC_DDR_DISABLE;
  cmdStc.DataMode          = ESMC_DATA_WRITE;
  cmdStc.NbData            = 8;
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0 | ESMC_SELECT_PIN_CS1 ; 
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x0000FFFF);
  uint8_t data[8];
    
  data[0]=((status&0x80)>>7) | 0xfe;
  data[1]=((status&0x40)>>6) | 0xfe;
  data[2]=((status&0x20)>>5) | 0xfe;
  data[3]=((status&0x10)>>4) | 0xfe;
  data[4]=((status&0x08)>>3) | 0xfe;
  data[5]=((status&0x04)>>2) | 0xfe;
  data[6]=((status&0x02)>>1) | 0xfe;
  data[7]=((status&0x01)>>0) | 0xfe;
  
  
  data[0]= (data[0]& 0x0f)|(data[0]<<4);
  data[1]= (data[1]& 0x0f)|(data[1]<<4);
  data[2]= (data[2]& 0x0f)|(data[2]<<4);
  data[3]= (data[3]& 0x0f)|(data[3]<<4);
  data[4]= (data[4]& 0x0f)|(data[4]<<4);
  data[5]= (data[5]& 0x0f)|(data[5]<<4);
  data[6]= (data[6]& 0x0f)|(data[6]<<4);
  data[7]= (data[7]& 0x0f)|(data[7]<<4);

  HAL_ESMC_Transmit(&hesmc, data, 0x0000FFFF);
}

/**
  * @brief  Write Enable function
  * @param  None
  * @retval None
  */
static void APP_WriteEnable()
{
  ESMC_CommandTypeDef commandStc;

  /* write enable */
  commandStc.TransferFormat    = ESMC_TRANSFER_FORMAT_OCTAL;
  commandStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  commandStc.Instruction       = P25Q64_WRITE_ENABLE_CMD;
  commandStc.AddressMode       = ESMC_ADDRESS_NONE;
  commandStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  commandStc.DummyCycles       = 0x0;
  commandStc.DdrMode           = ESMC_DDR_DISABLE;
  commandStc.DataMode          = ESMC_DATA_NONE;
  commandStc.CSPinSel          = ESMC_SELECT_PIN_CS0 | ESMC_SELECT_PIN_CS1 ;
  HAL_ESMC_Command(&hesmc, &commandStc, 0x0000FFFF);
}

/**
  * @brief  Sector Erase function
  * @param  address Erase Address
  * @retval None
  */
static void APP_EraseSector(uint32_t address)
{
  ESMC_CommandTypeDef cmdStc;

  /* write enable */
  APP_WriteEnable();

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_OCTAL;
  cmdStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  cmdStc.Instruction       = P25Q64_SECTOR_ERASE_4K_CMD;
  cmdStc.AddressMode       = ESMC_ADDRESS_SINGLE_LINE;
  cmdStc.AddressSize       = ESMC_ADDRESS_24_BITS;
  cmdStc.Address           = address;
  cmdStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  cmdStc.DummyCycles       = 0x0;
  cmdStc.DdrMode           = ESMC_DDR_DISABLE;
  cmdStc.DataMode          = ESMC_DATA_NONE;
  cmdStc.NbData            = 0;
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0 | ESMC_SELECT_PIN_CS1 ;
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x0000FFFF);
}

/**
  * @brief  Page Programming function
  * @param  pData    data pointer
  * @param  addr     Address Value
  * @param  nBytes   Length of data to be write
  * @retval None
  */
static void APP_PageProgram(uint8_t *pData, uint32_t addr, uint32_t nBytes)
{
  ESMC_CommandTypeDef cmdStc;

  /* write enable */
  APP_WriteEnable();

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_OCTAL;
  cmdStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  cmdStc.Instruction       = P25Q64_QUAD_INPUT_PAGE_PROG_CMD;
  cmdStc.AddressMode       = ESMC_ADDRESS_SINGLE_LINE;
  cmdStc.Address           = addr;
  cmdStc.AddressSize       = ESMC_ADDRESS_24_BITS;
  cmdStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  cmdStc.DummyCycles       = 0x0;
  cmdStc.DdrMode           = ESMC_DDR_DISABLE;
  cmdStc.DataMode          = ESMC_DATA_WRITE;
  cmdStc.NbData            = nBytes;
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0 | ESMC_SELECT_PIN_CS1 ;
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x0000FFFF);

  HAL_ESMC_Transmit(&hesmc, pData, 0x0000FFFF);
}

/**
  * @brief  Data Read function
  * @param  pData    data pointer
  * @param  address  Address Value
  * @param  nBytes   Length of data to be read
  * @retval None
  */
static void APP_DataRead(uint8_t *pData, uint32_t address, uint32_t nBytes)
{
  ESMC_CommandTypeDef cmdStc;

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_OCTAL;
  cmdStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  cmdStc.Instruction       = P25Q64_1I4O_FAST_READ_CMD;
  cmdStc.AddressMode       = ESMC_ADDRESS_SINGLE_LINE;
  cmdStc.Address           = address;
  cmdStc.AddressSize       = ESMC_ADDRESS_24_BITS;
  cmdStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  cmdStc.DummyCycles       = 8;
  cmdStc.DdrMode           = ESMC_DDR_DISABLE;
  cmdStc.DataMode          = ESMC_DATA_READ;
  cmdStc.NbData            = nBytes;
  cmdStc.CSPinSel          =ESMC_SELECT_PIN_CS0 | ESMC_SELECT_PIN_CS1 ; 
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x00FFFFFF);

  HAL_ESMC_Receive(&hesmc, pData, 0x00FFFFFF);
}

/**
  * @brief  Data comparison function
  * @param  pSrc   Source data pointer
  * @param  pDst   Target data pointer
  * @param  length Data length
  * @retval Error Code
  */
static HAL_StatusTypeDef APP_DataCheck(uint8_t *pSrc, uint8_t *pDst, uint32_t length)
{
  HAL_StatusTypeDef status = HAL_OK;

  for(uint32_t i=0; i<length; i++)
  {
    if(pSrc[i]!=pDst[i])
    {
      status = HAL_ERROR;
      break;
    }
  }

  return status;
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
