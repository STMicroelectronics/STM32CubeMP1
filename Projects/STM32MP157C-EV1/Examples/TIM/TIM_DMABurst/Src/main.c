
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
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_up;

uint32_t Prescaler ;

/* USER CODE BEGIN PV */

/* Capture Compare buffer */
uint32_t aSRC_Buffer[6] = {0x0FFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0555};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */

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
    HAL_RCC_DeInit();
    SystemClock_Config();
  }
  /* USER CODE END Init */

  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();

  /* USER CODE BEGIN SysInit */
  /* Initialize Prescaler */
  Prescaler = (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_TIMG1)/ (16 * 1000000)) - 1 ;
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Configure LED4 */
  BSP_LED_Init(LED4);
  
  /* -----------------------------------------------------------------------
    TIM2 Configuration: generate 1 PWM signal using the DMA burst mode:
   
    TIM2 input clock (TIM2CLK) is a multiple of APB1 clock (PCLK1).
        TIM2CLK is equal to the APB1 Timer Clocks
		=> TIM2CLK = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_TIMG1)
    
    To get TIM2 counter clock at 16 MHz, the prescaler is computed as follows:
      Prescaler = (TIM2CLK / TIM2 counter clock) - 1
      Prescaler = (RCC_PERIPHCLK_TIMG1 /16 MHz) - 1
  
    The TIM2 Frequency = TIM2 counter clock/(ARR + 1)
                       = 16 MHz / 4096 = 3.9 KHz
    TIM2 Channel4 duty cycle = (TIM2_CCR4/ TIM2_ARR)* 100 = 33.33%
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32mp1xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */

  /*## Start PWM signal generation in DMA mode ############################*/ 
  if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Starting PWM generation Error */
    Error_Handler();
  }
  
  /*## Start DMA Burst transfer ###########################################*/ 
  HAL_TIM_DMABurst_WriteStart(&htim2, TIM_DMABASE_ARR, TIM_DMA_UPDATE,
                              (uint32_t*)aSRC_Buffer, TIM_DMABURSTLENGTH_6TRANSFERS);


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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = Prescaler;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0xFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMAMUX_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, DEFAULT_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins
     PH5   ------> I2C2_SDA
     PH4   ------> I2C2_SCL
     PE13   ------> SAI2_FS_B
     PD5   ------> FMC_NWE
     PE3   ------> SDMMC2_CK
     PA9   ------> SDMMC2_D5
     PB3   ------> SDMMC2_D2
     PB14   ------> SDMMC2_D0
     PH10   ------> DCMI_D1
     PH11   ------> DCMI_D2
     PE14   ------> SAI2_MCLK_B
     PD1   ------> FMC_D3_DA3
     PD3   ------> SDMMC2_D7
     PB15   ------> SDMMC2_D1
     PA8   ------> SDMMC2_D4
     PH14   ------> DCMI_D4
     PE0   ------> SAI2_MCLK_A
     PD4   ------> FMC_NOE
     PD0   ------> FMC_D2_DA2
     PE5   ------> SDMMC2_D6
     PB4   ------> SDMMC2_D3
     PH13   ------> FDCAN1_TX
     PH8   ------> DCMI_HSYNC
     PG6   ------> SDMMC2_CMD
     PD2   ------> SDMMC1_CMD
     PC7   ------> SDMMC1_D123DIR
     PC9   ------> SDMMC1_D1
     PC11   ------> SDMMC1_D3
     PE12   ------> SAI2_SCK_B
     PH9   ------> DCMI_D0
     PE6   ------> DCMI_D7
     PC12   ------> SDMMC1_CK
     PC8   ------> SDMMC1_D0
     PI7   ------> SAI2_FS_A
     PI5   ------> SAI2_SCK_A
     PH12   ------> DCMI_D3
     PB7   ------> DCMI_VSYNC
     PB9   ------> SDMMC1_CDIR
     PF2   ------> SDMMC1_D0DIR
     PC10   ------> SDMMC1_D2
     PE4   ------> SDMMC1_CKIN
     PZ4   ------> I2C4_SCL
     PI6   ------> SAI2_SD_A
     PZ5   ------> I2C4_SDA
     PG12   ------> SPDIFRX_IN1
     PI9   ------> FDCAN1_RX
     PI4   ------> DCMI_D5
     PD15   ------> FMC_D1_DA1
     PD6   ------> FMC_NWAIT
     PD14   ------> FMC_D0_DA0
     PC3   ------> S_DATAIN1DFSDM1
     PG5   ------> ETH1_CLK125
     PC0   ------> QUADSPI_BK2_NCS
     PG11   ------> UART4_TX
     PA1   ------> ETH1_RX_CLK
     PB13   ------> S_CKOUTDFSDM1
     PF10   ------> QUADSPI_CLK
     PB2   ------> UART4_RX
     PF13   ------> S_DATAIN3DFSDM1
     PF11   ------> SAI2_SD_B
     PA6   ------> DCMI_PIXCLK
     PE7   ------> FMC_D4_DA4
     PE9   ------> FMC_D6_DA6
     PD12   ------> FMC_A17_ALE
     PB6   ------> QUADSPI_BK1_NCS
     PE10   ------> FMC_D7_DA7
     PG9   ------> FMC_NCE
     PC2   ------> ETH1_TXD2
     PE2   ------> ETH1_TXD3
     PB11   ------> ETH1_TX_CTL
     PH3   ------> QUADSPI_BK2_IO1
     PA7   ------> ETH1_RX_CTL
     PG7   ------> QUADSPI_BK2_IO3
     PE8   ------> FMC_D5_DA5
     PA10   ------> USB_OTG_HS_ID
     PG13   ------> ETH1_TXD0
     PG14   ------> ETH1_TXD1
     PB1   ------> ETH1_RXD3
     PC5   ------> ETH1_RXD1
     PB5   ------> SAI4_SD_A
     PG10   ------> QUADSPI_BK2_IO2
     PF7   ------> QUADSPI_BK1_IO2
     PF6   ------> QUADSPI_BK1_IO3
     USB_DM2   ------> USB_OTG_HS_DM
     PA2   ------> ETH1_MDIO
     PC1   ------> ETH1_MDC
     PG4   ------> ETH1_GTX_CLK
     PB0   ------> ETH1_RXD2
     PC4   ------> ETH1_RXD0
     PH2   ------> QUADSPI_BK2_IO0
     PB8   ------> DCMI_D6
     PD11   ------> FMC_A16_CLE
     PF8   ------> QUADSPI_BK1_IO0
     PF9   ------> QUADSPI_BK1_IO1
     USB_DP2   ------> USB_OTG_HS_DP
*/
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

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
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1) 
  {
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
  /* Infinite loop */
  while (1)
  {
  }

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
