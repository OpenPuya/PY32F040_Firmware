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
#define ADC_CALIBRATION_TIMEOUT_MS        2
#define VDDA_APPLI                        ((uint32_t)3300)
#define USE_TIMEOUT                       0

/* Private variables ---------------------------------------------------------*/
__IO uint32_t wait_loop_index = 0;
__IO uint32_t ADCxConvertedDatas[4];
__IO uint16_t uhADCxConvertedData_Voltage_mVolt = 0;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_AdcDmaConfig(void);
static void APP_AdcConfig(void);
static void APP_AdcCalibrate(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Enable SYSCFG clock and PWR clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure System clock */
  APP_SystemClockConfig();
  
  BSP_USART_Config();
  
  /* DMA configuration */
  APP_AdcDmaConfig();
  
  /* ADC configuration */
  APP_AdcConfig();
  
  /* ADC calibration */
  APP_AdcCalibrate();
  
  LL_mDelay(1);
  
  /* Enable ADC */
  LL_ADC_Enable(ADC1);

  LL_mDelay(1);
  while (1)
  {
    LL_ADC_REG_StartConversionSWStart(ADC1);
    
    LL_mDelay(1000);
  }
}

/**
  * @brief  DMA transfer completion callback function
  * @param  None
  * @retval None
  */
void APP_TransferCompleteCallback(void)
{
  /*Convert ADC sampling values to voltage values*/
  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, ADCxConvertedDatas[0], LL_ADC_RESOLUTION_12B);
  printf("%s%s%u mV\r\n","Channel4","Voltage:",uhADCxConvertedData_Voltage_mVolt);
  
  /*Convert ADC sampling values to voltage values*/
  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, ADCxConvertedDatas[1], LL_ADC_RESOLUTION_12B);
  printf("%s%s%u mV\r\n","Channel5","Voltage:",uhADCxConvertedData_Voltage_mVolt);
  
  /*Convert ADC sampling values to voltage values*/
  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, ADCxConvertedDatas[2], LL_ADC_RESOLUTION_12B);
  printf("%s%s%u mV\r\n","Channel6","Voltage:",uhADCxConvertedData_Voltage_mVolt);
  
  /*Convert ADC sampling values to voltage values*/
  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, ADCxConvertedDatas[3], LL_ADC_RESOLUTION_12B);
  printf("%s%s%u mV\r\n","Channel7","Voltage:",uhADCxConvertedData_Voltage_mVolt);
}

/**
  * @brief  DMA configuration function.
  * @param  None
  * @retval None
  */
void APP_AdcDmaConfig()
{
  /* Enable DMA1 clock */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* Enable SYSCFG clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  
  /* DMA1 Channel1 mapped to ADC1 */
  LL_SYSCFG_SetDMARemap(DMA1,LL_DMA_CHANNEL_1,LL_SYSCFG_DMA_MAP_ADC1);
 
  /* Configure DMA transmission direction to peripheral to memory */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Configure DMA priority to high */
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

  /* Configure DMA loop mode */
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  /* Configure DMA peripheral address unchanged mode */
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  /* Configure DMA storage address increment mode */
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  /* Configure DMA peripheral transmission method as word */
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);

  /* Configure DMA memory transfer method as word */
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);

  /* Configure DMA transmission length to 4 */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 4);

  /* Configure DMA peripherals and memory addresses */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR, (uint32_t)&ADCxConvertedDatas, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));

  /* Enable DMA transmission completion interrupt */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  
  /* Enable DMA1 Channel1  */
  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);

  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief  ADC Configuration Function.
  * @param  None
  * @retval None
  */
void APP_AdcConfig()
{
  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* Configure PA4 as analog output */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
  
  /* Configure PA5 as analog output */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);
  
  /* Configure PA6 as analog output */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
  
  /* Configure PA7 as analog output */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ANALOG);
  
  /* Enable ADC clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  
  /* Set ADC clock to PCLK quarter frequency */
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PCLK_DIV4);
  
  /* Set the internal mode to NONE */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

  /* Set the resolution to 12 bits */
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    
  /* Set data right alignment */
  LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);

  /* Enable scanning mode */
  LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);  
 
  /* Set General Group Software Trigger  */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    
  /* Set regular group single conversion mode */
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    
  /* Enable conventional group DMA mode */
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    
  /* Set regular group sequence length bit 4 */
  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);
    
  /* Turn off regular group interrupt mode */
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);

  /* Set regular group RANK1 channel 4 */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
  
  /* Set regular group RANK2 channel 5 */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_5);

  /* Set regular group RANK3 channel 6 */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_6);
  
  /* Set regular group RANK4 channel 7 */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_7);

  /* Set the sampling time of channel 4 to 41.5Cycles */
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_41CYCLES_5);   
  
  /* Set the sampling time of channel 5 to 41.5Cycles */
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_41CYCLES_5);  
  
  /* Set the sampling time of channel 6 to 41.5Cycles */
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_41CYCLES_5);  
  
  /* Set the sampling time of channel 7 to 41.5Cycles */
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_41CYCLES_5);   
}

/**
  * @brief  ADC calibration function.
  * @param  None
  * @retval None
  */
static void APP_AdcCalibrate()
{
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */
  
  if (LL_ADC_IsEnabled(ADC1) == 0)
  { 

    /* Enable ADC calibration */
    LL_ADC_StartCalibration(ADC1);
    
    /* Poll for ADC effectively calibrated */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
    
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
          APP_ErrorHandler();
        }
      }
    #endif /* USE_TIMEOUT */
    }  

/*  if(LL_ADC_GetCalibrationOffsetStatus(ADC1)!=LL_ADC_CAL_OFFSET_STATUS_SUCCESS)  */
/*  {                                                                              */
/*    APP_ErrorHandler();                                                          */
/*  }                                                                              */
     
  }
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
