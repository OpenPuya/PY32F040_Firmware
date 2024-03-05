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
#define ADC_CALIBRATION_TIMEOUT_MS         2
#define VAR_CONVERTED_DATA_INIT_VALUE      (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)
#define VDDA_APPLI                         ((uint32_t)3300)

/* Private variables ---------------------------------------------------------*/
__IO uint32_t wait_loop_index = 0;
__IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; 
__IO uint16_t uhADCxConvertedData_Voltage_mVolt = 0;  

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_TimerInit(void);
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

  /* Configure Systemclock */
  APP_SystemClockConfig();
  
  BSP_USART_Config();
  
  /* ADC configuration */
  APP_AdcConfig();
    
  /* ADC calibration */
  APP_AdcCalibrate();
  
  LL_mDelay(1);
 
  /* Enable ADC */
  LL_ADC_Enable(ADC1);

  LL_mDelay(1);
  
  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  
  /* TIM configuration */
  APP_TimerInit();
  
  while (1)
  {
    while(LL_ADC_IsActiveFlag_EOS(ADC1)==0)
    {

    }
    LL_ADC_ClearFlag_EOS(ADC1);
    uhADCxConvertedData = LL_ADC_REG_ReadConversionData12(ADC1); 
    uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);
    printf("Channel4: %umV\r\n",uhADCxConvertedData_Voltage_mVolt);
  }
}

/**
  * @brief  TIM Configuration Function.
  * @param  None
  * @retval None
  */
static void APP_TimerInit()
{
  /* Enable TIM3 clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* Set TIM3 prescale */
  LL_TIM_SetPrescaler(TIM3,2000);

  /* Set TIM3 auto-reload value */
  LL_TIM_SetAutoReload(TIM3, 4000);

  /* TIM3 Update event is used as trigger output */
  LL_TIM_SetTriggerOutput(TIM3,LL_TIM_TRGO_UPDATE);

  /* Enable TIM3 */
  LL_TIM_EnableCounter(TIM3);
}

/**
  * @brief  ADC Configuration Function.
  * @param  None
  * @retval None
  */
static void APP_AdcConfig()
{
  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* Configure PA4 as analog output */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
  
  /* Enable ADC clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  
  /* Set ADC to PCLK Quad */
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PCLK_DIV4);
  
  /* Set the internal mode to NONE */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

  /* Set the resolution to 12 bits */
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    
  /* Set data right alignment */
  LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);

  /* Turn off scanning mode */
  LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_DISABLE);  
 
  /* Set regular group TIM3 TRGO trigger */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM3_TRGO);
    
  /* Set regular group single conversion mode */
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    
  /* Set regular group DMA mode None */
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    
  /* Set regular group sequence length bit 1 */
  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    
  /* Turn off regular group interrupt mode */
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);

  /* Set regular group RANK1 channel 4 */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);

  /* Set the sampling time to 41.5Cycles */
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_41CYCLES_5);   
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
