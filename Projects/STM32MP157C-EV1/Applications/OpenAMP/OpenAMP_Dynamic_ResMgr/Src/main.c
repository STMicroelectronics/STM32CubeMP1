
/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy;  Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                       opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
IPCC_HandleTypeDef hipcc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
EXTI_HandleTypeDef hexti;
RCC_OscInitTypeDef RCC_OscInitStruct_LSI;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after Wkup/Tamper push-button interrupt  */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_IPCC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void EXTI14_IRQHandler_Config(void);
static void Exti14FallingCb(void);
static void Disable_ADC_Regulator(void);
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
  DAC_ChannelConfTypeDef sConfig;
  ResConfig_t config_in, config_out;
  
  
  
  /* STM32MPxx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initialize the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }
  /* USER CODE END Init */

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
  MX_IPCC_Init();
  MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
  ResMgr_Init(NULL, NULL);

  /* Configure LED4 */
  BSP_LED_Init(LED4);

  /* Configure EXTI14 (connected to PA.14 pin) in interrupt mode
     It is used for changing the gain */
  EXTI14_IRQHandler_Config();

  /* Do not use ADC regulator, so DAC has the full control of the regulator */
  Disable_ADC_Regulator();

  /*##########################################################################*/
  /*## NORMAL POWER MODE #####################################################*/

  /*##-0- Ask the resource manager for the current supply status #############*/
  config_in.regu.index = 0;
  if (ResMgr_GetConfig(RESMGR_ID_DAC1, NULL, RESMGR_REGU, &config_in, &config_out) != RESMGR_OK)
  {
    /* Cannot find regulator : is the DeviceTree correct? */
    Error_Handler();
  }

  if (config_out.regu.enable == 0)
  {
    /* Regulator is not enabled */
    Error_Handler();
  }

  /*##-1- Set DAC Channel1 DHR register ######################################*/
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0xFF) != HAL_OK)
  {
    /* Setting value Error */
    Error_Handler();
  }

  /*##-2- Enable DAC Channel1 ################################################*/
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Wait UserButtonStatus pushed */
  while (UserButtonStatus == 0)
  {
  }
  UserButtonStatus = 0;

  /*##########################################################################*/
  /*## STOP AND DISABLE REGULATOR ############################################*/

  /*##-1- Stop DAC ###########################################################*/
  if (HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /*##-2- Ask the resource manager to disable the regulator ##################*/
  config_in.regu.index = 0;
  config_in.regu.enable = 0;
  config_in.regu.min_voltage_mv = 0; /* unchanged */
  config_in.regu.max_voltage_mv = 0; /* unchanged */
  if (ResMgr_SetConfig(RESMGR_ID_DAC1, NULL, RESMGR_REGU, &config_in, &config_out) != RESMGR_OK)
  {
    /* Failed to update the regulator */
    Error_Handler();
  }
  if (config_out.regu.enable == 1)
  {
    /* Failed to disable the regulator */
    Error_Handler();
  }

  /* Wait UserButtonStatus pushed */
  while (UserButtonStatus == 0)
  {
  }
  UserButtonStatus = 0;

  /*##########################################################################*/
  /*## LOW POWER MODE ########################################################*/

  if(IS_ENGINEERING_BOOT_MODE())
  {
	/*## LSI already enabled in PRODUCTION Mode  #############################*/
    RCC_OscInitStruct_LSI.OscillatorType = RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct_LSI.LSIState       = RCC_LSI_ON;
    RCC_OscInitStruct_LSI.PLL.PLLState   = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct_LSI);
  }
  
  if (HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  
  /*##-0- DeInit the DAC peripheral ##########################################*/
  if (HAL_DAC_DeInit(&hdac1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-1- Ask the resource manager to enable the regulator ###################*/
  config_in.regu.index = 0;
  config_in.regu.enable = 1;
  config_in.regu.min_voltage_mv = 0; /* unchanged */
  config_in.regu.max_voltage_mv = 0; /* unchanged */
  if (ResMgr_SetConfig(RESMGR_ID_DAC1, NULL, RESMGR_REGU, &config_in, &config_out) != RESMGR_OK)
  {
    /* Failed to update the regulator */
    Error_Handler();
  }
  if (config_out.regu.enable == 0)
  {
    /* Failed to enable the regulator */
    Error_Handler();
  }
  
  /*##-2- Configure the DAC peripheral #######################################*/
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-3- Configure DAC channel1 #############################################*/
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_ENABLE;
   
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

  sConfig.DAC_SampleAndHoldConfig.DAC_SampleTime = 20;
  sConfig.DAC_SampleAndHoldConfig.DAC_HoldTime = 10;
  sConfig.DAC_SampleAndHoldConfig.DAC_RefreshTime = 5;
  
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Channel configuration Error */
    Error_Handler();
  }

  /*##-4- Set DAC Channel1 DHR register ######################################*/
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0xFF) != HAL_OK)
  {
    /* Setting value Error */
    Error_Handler();
  }

  /*##-5- Enable DAC Channel1 ################################################*/
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*Suspend Tick increment to prevent wakeup by Systick interrupt. 
  Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
  HAL_SuspendTick();

  /* Enter SLEEP Mode , wake up is done once Key push button is pressed */
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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

  if (HAL_DAC_DeInit(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

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
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
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
/**
  * @brief IPPC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
     Error_Handler();
  }
}
/* USER CODE BEGIN 4 */

/* USER CODE BEGIN 4 */
/**
  * @brief  Configures EXTI line 14 (connected to PA.14 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI14_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  EXTI_ConfigTypeDef EXTI_ConfigStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* Configure PA.14 pin as input floating */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = USER_BUTTON_PIN;
  PERIPH_LOCK(GPIOA);
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  PERIPH_UNLOCK(GPIOA);

  /* Set configuration except Interrupt and Event mask of Exti line 14*/
  EXTI_ConfigStructure.Line = EXTI_LINE_14;
  EXTI_ConfigStructure.Trigger = EXTI_TRIGGER_FALLING;
  EXTI_ConfigStructure.GPIOSel = EXTI_GPIOA;
  EXTI_ConfigStructure.Mode = EXTI_MODE_C2_INTERRUPT;
  PERIPH_LOCK(EXTI);
  HAL_EXTI_SetConfigLine(&hexti, &EXTI_ConfigStructure);
  PERIPH_UNLOCK(EXTI);

  /* Register callback to treat Exti interrupts in user Exti14FallingCb function */
  HAL_EXTI_RegisterCallback(&hexti, HAL_EXTI_FALLING_CB_ID, Exti14FallingCb);

  /* Enable and set line 14 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI14_IRQn, (DEFAULT_IRQ_PRIO + 2U), 0);
  HAL_NVIC_EnableIRQ(EXTI14_IRQn);
}

/**
  * @brief EXTI line detection callbacks
  * @param None:
  * @retval None
  */
static void Exti14FallingCb(void)
{
  UserButtonStatus = 1;
}

/**
  * @brief  Ask the Resources Manager to disable the AD regulator (if used)
  *         This ensures DAC to be the only consumer of this common ADC & DAC
  *         regulator and to have the full control of it.
  * @param  None
  * @retval None
  */
static void Disable_ADC_Regulator()
{
  ResConfig_t config_in, config_out;

  config_in.regu.index = 0;
  if (ResMgr_GetConfig(RESMGR_ID_ADC1, NULL, RESMGR_REGU, &config_in, &config_out) == RESMGR_OK)
  {
    config_in.regu.index = 0;
    config_in.regu.enable = 0;
    config_in.regu.min_voltage_mv = 0; /* unchanged */
    config_in.regu.max_voltage_mv = 0; /* unchanged */
    if (ResMgr_SetConfig(RESMGR_ID_ADC1, NULL, RESMGR_REGU, &config_in, &config_out) != RESMGR_OK)
    {
      /* Failed to update the regulator */
      Error_Handler();
    }
  }
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
  /* Error if LED4 is slowly blinking (1 sec. period) */
  while(1)
  {    
    BSP_LED_Toggle(LED4); 
    HAL_Delay(1000);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
