
/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* ADC handler declaration */
ADC_HandleTypeDef    hadc2;
/* Variables for ADC conversion data */
__IO   uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* ADC group regular conversion data (array of data) */

/* Variables for ADC conversion data computation to physical values */
uint16_t   aADCxConvertedData_Voltage_mVolt[ADC_CONVERTED_DATA_BUFFER_SIZE];  /* Value of voltage calculated from ADC conversion data (unit: mV) (array of data) */
/* Variables for ADC conversion data computation to physical values */
uint16_t uhADCxConvertedData_VoltageGPIO_mVolt = 0UL;        /* Value of voltage calculated from ADC conversion data (unit: mV) */
uint16_t uhADCxConvertedData_VrefInt_mVolt = 0UL;            /* Value of internal voltage reference VrefInt calculated from ADC conversion data (unit: mV) */
 int16_t hADCxConvertedData_Temperature_DegreeCelsius = 0UL; /* Value of temperature calculated from ADC conversion data (unit: degree Celsius) */
uint16_t uhADCxConvertedData_VrefAnalog_mVolt = 0UL;         /* Value of analog reference voltage (Vref+), connected to analog voltage supply Vdda, calculated from ADC conversion data (unit: mV) */


/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
/*  2: DMA transfer has not yet been started yet (initial state)              */
__IO   uint8_t ubDmaTransferStatus = 2; /* Variable set into DMA interruption callback */

/* Variable to manage push button on board: interface between ExtLine interruption and main program */
__IO   uint8_t ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void Configure_ADC(void);
static void Generate_waveform_SW_update_Config(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t tmp_index_adc_converted_data = 0;
  uint32_t tmp_index;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initialize the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  if(IS_ENGINEERING_BOOT_MODE())
  {
    HAL_RCC_DeInit();
    SystemClock_Config();
  }

  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();

  /* USER CODE BEGIN SysInit */
  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure PMIC */
    BSP_PMIC_Init();
    BSP_PMIC_InitRegulators();
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  for (tmp_index_adc_converted_data = 0; tmp_index_adc_converted_data < ADC_CONVERTED_DATA_BUFFER_SIZE; tmp_index_adc_converted_data++)
  {
    aADCxConvertedData[tmp_index_adc_converted_data] = VAR_CONVERTED_DATA_INIT_VALUE;
  }
  
  /* Initialize LED on board */
  BSP_LED_Init(LED4);

 /* Configure ADC */
  /* Note: This function configures the ADC but does not enable it.           */
  /*       Only ADC internal voltage regulator is enabled by function         */
  /*       "HAL_ADC_Init()".                                                  */
  /*       To activate ADC (ADC enable and ADC conversion start), use         */
  /*       function "HAL_ADC_Start_xxx()".                                    */
  /*       This is intended to optimize power consumption:                    */
  /*       1. ADC configuration can be done once at the beginning             */
  /*          (ADC disabled, minimal power consumption)                       */
  /*       2. ADC enable (higher power consumption) can be done just before   */
  /*          ADC conversions needed.                                         */
  /*          Then, possible to perform successive ADC activation and         */
  /*          deactivation without having to set again ADC configuration.     */
  Configure_ADC();
  
  /* Run the ADC linear calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  /* Configure the DAC peripheral and generate a constant voltage of Vdda/2.  */
  Generate_waveform_SW_update_Config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  /*## Start ADC conversions ###############################################*/
  /* Start ADC group regular conversion with DMA */
  if (HAL_ADC_Start_DMA(&hadc2,
                        (uint32_t *)aADCxConvertedData,
                        ADC_CONVERTED_DATA_BUFFER_SIZE
                       ) != HAL_OK)
  {
    /* ADC conversion start error */
    Error_Handler();
  }  
  
  /* Toggle LED at each ADC conversion */
  BSP_LED_On(LED4);
  HAL_Delay(LED_BLINK_SLOW);
  BSP_LED_Off(LED4);
  HAL_Delay(LED_BLINK_SLOW);

  while (1)
  {
  /* USER CODE BEGIN 3 */
    /* Note: LED state depending on DMA transfer status is set into DMA       */
    /*       IRQ handler, refer to functions "HAL_ADC_ConvCpltCallback()"     */
    /*       and "HAL_ADC_ConvHalfCpltCallback()".                            */

    /* Note: ADC conversions data are stored into array                       */
    /*       "aADCConvertedData"                                              */
    /*       (for debug: see variable content into watch window).             */
    
    /* Note: ADC conversion data are computed to physical values              */
    /*       into array "aADCxConvertedData_Voltage_mVolt"                    */
    /*       using helper macro "__ADC_CALC_DATA_VOLTAGE()".                  */
    /*       (for debug: see variable content into watch window).             */

    /* Start ADC group regular conversion */
    if (HAL_ADC_Start(&hadc2) != HAL_OK)
    {
      /* Error: ADC conversion start could not be performed */
      Error_Handler();
    }

    /* Toggle LED at each ADC conversion */
    BSP_LED_On(LED4);
    HAL_Delay(LED_BLINK_SLOW);
    BSP_LED_Off(LED4);
    HAL_Delay(LED_BLINK_SLOW);

    /* Note: ADC group regular conversions data are stored into array         */
    /*       "uhADCxConvertedData"                                            */
    /*       (for debug: see variable content into watch window).             */
    /*       - uhADCxConvertedData[0]: ADC channel set on sequence rank 1     */
    /*                                 (ADC1 channel 6)                       */
    /*       - uhADCxConvertedData[1]: ADC channel set on sequence rank 2     */
    /*                                 (ADC1 internal channel VrefInt)        */
    /*       - uhADCxConvertedData[2]: ADC channel set on sequence rank 3     */
    /*                                 (ADC1 internal channel temper. sensor) */

    /* If ADC conversions and DMA transfer are completed, then process data */
    if(ubDmaTransferStatus == 1)
    {
      /* For this example purpose, calculate analog reference voltage (Vref+) */
      /* from ADC conversion of internal voltage reference VrefInt.           */
      /* This voltage should correspond to value of literal "VDDA_APPLI".     */
      /* Note: This calculation can be performed when value of voltage Vref+  */
      /*       is unknown in the application                                  */
      /*       (This is not the case in this example due to target board      */
      /*       supplied by a LDO regulator providing a known constant voltage */
      /*       of value "VDDA_APPLI").                                        */
      /*       In typical case of Vref+ connected to Vdd, it allows to        */
      /*       deduce Vdd value.                                              */
      uhADCxConvertedData_VrefAnalog_mVolt = __LL_ADC_CALC_VREFANALOG_VOLTAGE(aADCxConvertedData[1], LL_ADC_RESOLUTION_12B);

      /* Computation of ADC conversions raw data to physical values           */
      /* using LL ADC driver helper macro.                                    */
      uhADCxConvertedData_VoltageGPIO_mVolt        = __LL_ADC_CALC_DATA_TO_VOLTAGE(uhADCxConvertedData_VrefAnalog_mVolt, aADCxConvertedData[0], LL_ADC_RESOLUTION_12B);
      uhADCxConvertedData_VrefInt_mVolt            = __LL_ADC_CALC_DATA_TO_VOLTAGE(uhADCxConvertedData_VrefAnalog_mVolt, aADCxConvertedData[1], LL_ADC_RESOLUTION_12B);
      hADCxConvertedData_Temperature_DegreeCelsius = __LL_ADC_CALC_TEMPERATURE(uhADCxConvertedData_VrefAnalog_mVolt, aADCxConvertedData[2], LL_ADC_RESOLUTION_12B);

      /* Update status variable of DMA transfer */
      ubDmaTransferStatus = 0;

      /* Toggle LED 4 times */
      tmp_index = 4*2;
      while(tmp_index != 0)
      {
        BSP_LED_Toggle(LED4);
        HAL_Delay(LED_BLINK_FAST);
        tmp_index--;
      }
      HAL_Delay(500); /* Delay to highlight toggle sequence */
    }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;

  /**PLL1 Config
  */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 81;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLFRACV = 0x800;
  RCC_OscInitStruct.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL2 Config
    */
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL2.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL2.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL3 Config
    */
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 2;
  RCC_OscInitStruct.PLL3.PLLN = 34;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 17;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 0x1A04;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL3.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL4 Config
    */
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 99;
  RCC_OscInitStruct.PLL4.PLLP = 6;
  RCC_OscInitStruct.PLL4.PLLQ = 8;
  RCC_OscInitStruct.PLL4.PLLR = 8;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
  RCC_OscInitStruct.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
  Error_Handler();
  }
  /**RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                |RCC_CLOCKTYPE_PCLK5|RCC_CLOCKTYPE_MPU;
  RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;
  RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
  Error_Handler();
  }

  /**Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}


/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PE3   ------> SDMMC2_CK
     PA9   ------> SDMMC2_D5
     PB3   ------> SDMMC2_D2
     PB14   ------> SDMMC2_D0
     PD3   ------> SDMMC2_D7
     PB15   ------> SDMMC2_D1
     PA8   ------> SDMMC2_D4
     PE5   ------> SDMMC2_D6
     PB4   ------> SDMMC2_D3
     PG6   ------> SDMMC2_CMD
     PA10   ------> USB_OTG_HS_ID
     USB_DM2   ------> USB_OTG_HS_DM
     USB_DP2   ------> USB_OTG_HS_DP
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
}
/* USER CODE BEGIN 4 */


/**
  * @brief  Configure ADC (ADC instance: ADCx) and GPIO used by ADC channels.
  *         Configuration of GPIO:
  *           - Pin:                    PA.04 (on this STM32 device, ADC2 channel 16 is mapped on this GPIO)
  *           - Mode:                   analog
  *         Configuration of ADC:
  *         - Common to several ADC:
  *           - Conversion clock:       Asynchronous from PCLK
  *           - Internal path:          None                         (default configuration from reset state)
  *         - Multimode
  *           Feature not used: all parameters let to default configuration from reset state
  *           - Mode                    Independent                  (default configuration from reset state)
  *           - DMA transfer:           Disabled                     (default configuration from reset state)
  *           - Delay sampling phases   1 ADC clock cycle            (default configuration from reset state)
  *         - ADC instance
  *           - Resolution:             12 bits                      (default configuration from reset state)
  *           - Data alignment:         right aligned                (default configuration from reset state)
  *           - Low power mode:         disabled                     (default configuration from reset state)
  *           - Offset:                 none                         (default configuration from reset state)
  *         - Group regular
  *           - Trigger source:         SW start
  *           - Trigger edge:           not applicable with SW start
  *           - Continuous mode:        single conversion            (default configuration from reset state)
  *           - DMA transfer:           enabled, unlimited requests
  *           - Overrun:                data overwritten
  *           - Sequencer length:       enabled: 3 rank
  *           - Sequencer discont:      disabled: sequence done in 1 scan (default configuration from reset state)
  *           - Sequencer rank 1:       ADCx ADCx_CHANNELa
  *         - Group injected
  *           Feature not used: all parameters let to default configuration from reset state
  *           - Trigger source:         SW start                     (default configuration from reset state)
  *           - Trigger edge:           not applicable with SW start
  *           - Auto injection:         disabled                     (default configuration from reset state)
  *           - Contexts queue:         disabled                     (default configuration from reset state)
  *           - Sequencer length:       disabled: 1 rank             (default configuration from reset state)
  *           - Sequencer discont:      disabled: sequence done in 1 scan (default configuration from reset state)
  *           - Sequencer rank 1:       first channel available      (default configuration from reset state)
  *         - Channel
  *           - Sampling time:          ADCx ADCx_CHANNELa set to sampling time 810.5 ADC clock cycles (on this STM32 serie, sampling time is channel wise)
  *           - Differential mode:      single ended                 (default configuration from reset state)
  *         - Analog watchdog
  *           Feature not used: all parameters let to default configuration from reset state
  *           - AWD number:             1
  *           - Monitored channels:     none                         (default configuration from reset state)
  *           - Threshold high:         0x000                        (default configuration from reset state)
  *           - Threshold low:          0xFFF                        (default configuration from reset state)
  *         - Oversampling
  *           Feature not used: all parameters let to default configuration from reset state
  *           - Scope:                  none                         (default configuration from reset state)
  *           - Discontinuous mode:     disabled                     (default configuration from reset state)
  *           - Ratio:                  2                            (default configuration from reset state)
  *           - Shift:                  none                         (default configuration from reset state)
  *         - Interruptions
  *           None: with HAL driver, ADC interruptions are set using
  *           function "HAL_ADC_start_xxx()".
  * @note   Using HAL driver, configuration of GPIO used by ADC channels,
  *         NVIC and clock source at top level (RCC)
  *         are not implemented into this function,
  *         must be implemented into function "HAL_ADC_MspInit()".
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Configure_ADC(void)
{
  ADC_ChannelConfTypeDef   sConfig;
  
  /*## Configuration of ADC ##################################################*/
  
  /*## Configuration of ADC hierarchical scope: ##############################*/
  /*## common to several ADC, ADC instance, ADC group regular  ###############*/
  
  /* Set ADC instance of HAL ADC handle hadc2 */
  hadc2.Instance = ADCx;
  
  /* Configuration of HAL ADC handle init structure:                          */
  /* parameters of scope ADC instance and ADC group regular.                  */
  /* Note: On this STM32 serie, ADC group regular sequencer is                */
  /*       fully configurable: sequencer length and each rank                 */
  /*       affectation to a channel are configurable.                         */
  hadc2.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode          = ADC_SCAN_ENABLE;               /* Sequencer enabled (ADC conversion on 3 channels) */
  hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait      = DISABLE;
  hadc2.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  hadc2.Init.NbrOfConversion       = 3;                             /* Parameter discarded because sequencer is disabled */
  hadc2.Init.DiscontinuousConvMode = ENABLE;                        /* Parameter discarded because sequencer is disabled */
  hadc2.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode      = DISABLE;

  if (HAL_ADC_DeInit(&hadc2) != HAL_OK)
  {
    /* ADC Deinitialization error */
    Error_Handler();
  }
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }
  
  
  /*## Configuration of ADC hierarchical scope: ##############################*/
  /*## ADC group injected and channels mapped on group injected ##############*/
  
  /* Note: ADC group injected not used and not configured in this example.    */
  /*       Refer to other ADC examples using this feature.                    */
  /* Note: Call of the functions below are commented because they are         */
  /*       useless in this example:                                           */
  /*       setting corresponding to default configuration from reset state.   */
  
  
  /*## Configuration of ADC hierarchical scope: ##############################*/
  /*## channels mapped on group regular         ##############################*/
  
  /* Configuration of channel on ADCx regular group on sequencer rank x      */
  /* Note: On this STM32 serie, ADC group regular sequencer is                */
  /*       fully configurable: sequencer length and each rank                 */
  /*       affectation to a channel are configurable.                         */
  /* Note: Considering IT occurring after each ADC conversion                 */
  /*       (IT by ADC group regular end of unitary conversion),               */
  /*       select sampling time and ADC clock with sufficient                 */
  /*       duration to not create an overhead situation in IRQHandler.        */

  /** Configure Regular Channel Rank 1
  */
  sConfig.Channel      = ADCx_CHANNELa;               /* ADC channel selection */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* ADC group regular rank in which is mapped the selected ADC channel */
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  /* ADC channel sampling time */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* ADC channel differential mode */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* ADC channel affected to offset number */
  sConfig.Offset       = 0;                           /* Parameter discarded because offset correction is disabled */
  
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  /** Configure Regular Channel Rank 2
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel Rank 3
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  
  /*## Configuration of ADC hierarchical scope: multimode ####################*/
  /* Note: ADC multimode not used and not configured in this example.         */
  /*       Refer to other ADC examples using this feature.                    */
  
  
  /*## Configuration of ADC transversal scope: analog watchdog ###############*/
  
  /* Note: ADC analog watchdog not used and not configured in this example.   */
  /*       Refer to other ADC examples using this feature.                    */
  
  
  /*## Configuration of ADC transversal scope: oversampling ##################*/
  
  /* Note: ADC oversampling not used and not configured in this example.      */
  /*       Refer to other ADC examples using this feature.                    */
  
}

/**
  * @brief  For this example, generate a waveform voltage on a spare DAC
  *         channel, so user has just to connect a wire between DAC channel 
  *         (pin PA4) and ADC channel (pin PA4) to run this example.
  *         (this prevents the user from resorting to an external signal
  *         generator).
  *         This function configures the DAC and generates a constant voltage of Vdda/2.
  * @note   Voltage level can be modifying afterwards using function
  *         "Generate_waveform_SW_update()".
  * @param  None
  * @retval None
  */
static void Generate_waveform_SW_update_Config(void)
{
  /* Set DAC Channel data register: channel corresponding to ADC channel ADC2_CHANNEL_16 */
  /* Set DAC output to 1/2 of full range (4095 <=> Vdda=3.3V): 2048 <=> 1.65V */
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DIGITAL_SCALE_12BITS/2) != HAL_OK)
  {
    /* Setting value Error */
    Error_Handler();
  }
  
  /* Enable DAC Channel: channel corresponding to ADC channel ADC2_CHANNEL_16 */
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc: ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t tmp_index = 0;
  
  /* Computation of ADC conversions raw data to physical values               */
  /* using LL ADC driver helper macro.                                        */
  /* Management of the 2nd half of the buffer */
  for (tmp_index = (ADC_CONVERTED_DATA_BUFFER_SIZE/2); tmp_index < ADC_CONVERTED_DATA_BUFFER_SIZE; tmp_index++)
  {
    aADCxConvertedData_Voltage_mVolt[tmp_index] = __ADC_CALC_DATA_VOLTAGE(VDDA_APPLI, aADCxConvertedData[tmp_index]);
  }
  
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 1;
  
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode 
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t tmp_index = 0;
  
  /* Computation of ADC conversions raw data to physical values               */
  /* using LL ADC driver helper macro.                                        */
  /* Management of the 1st half of the buffer */
  for (tmp_index = 0; tmp_index < (ADC_CONVERTED_DATA_BUFFER_SIZE/2); tmp_index++)
  {
    aADCxConvertedData_Voltage_mVolt[tmp_index] = __ADC_CALC_DATA_VOLTAGE(VDDA_APPLI, aADCxConvertedData[tmp_index]);
  }
  
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 0;
  
}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* Turn on LED and remain in infinite loop */
  while(1) 
  {
	    BSP_LED_On(LED4);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

