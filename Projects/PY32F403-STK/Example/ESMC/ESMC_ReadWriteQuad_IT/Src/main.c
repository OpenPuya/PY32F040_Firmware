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
#define BUFFERSIZE  4096

/* Private variables ---------------------------------------------------------*/
ESMC_HandleTypeDef hesmc;
ESMC_InitTypeDef ESMC_initTypeDef;

uint8_t aRxBuffer[BUFFERSIZE];
uint8_t aTxBuffer[BUFFERSIZE];
uint8_t flashStatus;
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_EsmcInit(void);
static void APP_WriteEnable(void);
static void APP_EraseSector(uint32_t address);
static void APP_PageProgram(uint8_t *pdata, uint32_t addr, uint32_t Nbytes);
static void APP_DataRead(uint8_t *pData, uint32_t address, uint32_t nBytes);
static HAL_StatusTypeDef APP_DataCheck(uint8_t *pSrc, uint8_t *pDst, uint32_t length);
static void APP_SetFlashStatus(uint8_t status, uint8_t comm);
static uint8_t APP_GetFlashStatus(uint8_t comm);

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

  /* System clock configuration */
  APP_SystemClockConfig();
  
  BSP_LED_Init(LED_GREEN);

  /* Test Data Initializes */
  for(i = 0; i < BUFFERSIZE; i++)
  {
    aTxBuffer[i] = i%256;
  }

  /* Initialize ESMC */
  APP_EsmcInit();

  /* Sector Erase */
  APP_EraseSector(address);
  /* Wait Erase Complete */
  while (APP_GetFlashStatus(P25Q64_READ_STATUS_REG_CMD) & 0x01) {;}

  /* Enable QE */
  flashStatus = APP_GetFlashStatus(P25Q64_READ_STATUS_REG_1_CMD);
  flashStatus |= 0x02;   /* Set Status QE */
    
  /* Write enable */
  APP_WriteEnable();
  /* Wait Flash Status */ 
  while (APP_GetFlashStatus(0x05) & 0x01) {}
    
  APP_SetFlashStatus(flashStatus, P25Q64_WRITE_STATUS_REG_1_CMD);
  while (APP_GetFlashStatus(P25Q64_READ_STATUS_REG_CMD) & 0x01) {;} /* Wait flash status register complete */

  /* Page Program */
  cnt = BUFFERSIZE;
  progAddr = address;
  pData = aTxBuffer;
  while (cnt/P25Q64_PAGE_SIZE > 0)
  {
    APP_PageProgram(pData, progAddr, P25Q64_PAGE_SIZE);
    /* Wait Flash Status */
    while (APP_GetFlashStatus(P25Q64_READ_STATUS_REG_CMD) & 0x01) {;}
    cnt      -= P25Q64_PAGE_SIZE;
    progAddr += P25Q64_PAGE_SIZE;
    pData    += P25Q64_PAGE_SIZE;
  }
  if (cnt > 0)
  {
    APP_PageProgram(pData, progAddr, cnt);
    /* Wait Flash Status */
    while (APP_GetFlashStatus(P25Q64_READ_STATUS_REG_CMD) & 0x01) {;}
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
  ESMC_initTypeDef.ClockPrescaler = 0x08;
  ESMC_initTypeDef.ClockMode = ESMC_CLOCK_MODE_0;
  ESMC_initTypeDef.DualFlash = ESMC_DUALFLASH_DISABLE;

  hesmc.Instance = ESMC;
  hesmc.Init     = ESMC_initTypeDef;
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
  cmdStc.DummyCycles       = 0x0;
  cmdStc.DdrMode           = ESMC_DDR_DISABLE;
  cmdStc.DataMode          = ESMC_DATA_READ;
  cmdStc.NbData            = 1;
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0;
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x0000FFFF);

  HAL_ESMC_Receive(&hesmc, &status, 0x0000FFFF);

  return status;
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

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_SINGLE;
  cmdStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  cmdStc.Instruction       = command;
  cmdStc.AddressMode       = ESMC_ADDRESS_NONE;
  cmdStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  cmdStc.DummyCycles       = 0x0;
  cmdStc.DdrMode           = ESMC_DDR_DISABLE;
  cmdStc.DataMode          = ESMC_DATA_WRITE;
  cmdStc.NbData            = 1;
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0;
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x0000FFFF);

  HAL_ESMC_Transmit(&hesmc, &status, 0x0000FFFF);
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
  commandStc.TransferFormat    = ESMC_TRANSFER_FORMAT_SINGLE;
  commandStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  commandStc.Instruction       = P25Q64_WRITE_ENABLE_CMD;
  commandStc.AddressMode       = ESMC_ADDRESS_NONE;
  commandStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  commandStc.DummyCycles       = 0x0;
  commandStc.DdrMode           = ESMC_DDR_DISABLE;
  commandStc.DataMode          = ESMC_DATA_NONE;
  commandStc.CSPinSel          = ESMC_SELECT_PIN_CS0;
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

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_SINGLE;
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
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0;
  HAL_ESMC_Command_IT(&hesmc, &cmdStc);

  /* Wait Command complete */
  while(hesmc.State != HAL_ESMC_STATE_READY);
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

  /* write enbale */
  APP_WriteEnable();

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_QUAD;
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
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0;
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x0000FFFF);

  HAL_ESMC_Transmit_IT(&hesmc, pData);

  while(hesmc.State != HAL_ESMC_STATE_READY);
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

  cmdStc.TransferFormat    = ESMC_TRANSFER_FORMAT_QUAD;
  cmdStc.InstructionMode   = ESMC_INSTRUCTION_SINGLE_LINE;
  cmdStc.Instruction       = P25Q64_1I4O_FAST_READ_CMD;
  cmdStc.AddressMode       = ESMC_ADDRESS_SINGLE_LINE;
  cmdStc.Address           = address;
  cmdStc.AddressSize       = ESMC_ADDRESS_24_BITS;
  cmdStc.AlternateByteMode = ESMC_ALTERNATE_BYTES_DISABLE;
  cmdStc.DummyCycles       = DUMMY_CLOCK_CYCLES_READ;
  cmdStc.DdrMode           = ESMC_DDR_DISABLE;
  cmdStc.DataMode          = ESMC_DATA_READ;
  cmdStc.NbData            = nBytes;
  cmdStc.CSPinSel          = ESMC_SELECT_PIN_CS0;
  HAL_ESMC_Command(&hesmc, &cmdStc, 0x0000FFFF);

  HAL_ESMC_Receive_IT(&hesmc, pData);

  while(hesmc.State != HAL_ESMC_STATE_READY);
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
  OscInitstruct.HSEState        = RCC_HSE_OFF;                              /* Disable HSE */
/* OscInitstruct.HSEFreq         = RCC_HSE_16_32MHz; */                     /* HSE frequency range 16~32M */
  OscInitstruct.HSI48MState     = RCC_HSI48M_OFF;                           /* Disable HSI48M */
  OscInitstruct.HSIState        = RCC_HSI_ON;                               /* Enable HSI */
  OscInitstruct.LSEState        = RCC_LSE_OFF;                              /* Disable LSE */
/* OscInitstruct.LSEDriver       = RCC_LSEDRIVE_HIGH; */                    /* LSEDriver：High */
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
  ClkInitstruct.APB1CLKDivider  = RCC_HCLK_DIV1;                        /* APB1 clock 1 division  */
  ClkInitstruct.APB2CLKDivider  = RCC_HCLK_DIV2;                        /* APB2 clock 2 division */
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
