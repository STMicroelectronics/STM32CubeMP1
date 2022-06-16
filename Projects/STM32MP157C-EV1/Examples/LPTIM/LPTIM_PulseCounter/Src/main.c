/**
  ******************************************************************************
  * @file    Examples/LPTIM/LPTIM_PulseCounter/Src/main.c
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

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
LPTIM_HandleTypeDef hlptim2;

/* USER CODE BEGIN PV */
RCC_ClkInitTypeDef  RCC_ClkInit;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPTIM2_Init(void);

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef SYSCLKConfig_STOP(void);
static void RCC_backupClocks(void);
static HAL_StatusTypeDef Prepare_IOComp(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Main program
  * @param  None
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    HAL_RCC_DeInit();
    SystemClock_Config();
  }

  /* USER CODE BEGIN SysInit */
  /* Prepare IO compensation mechanism */
  if (Prepare_IOComp() != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable IO compensation mechanism */
  if (HAL_SYSCFG_EnableIOCompensation() !=  HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPTIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Configure LED3 */
  BSP_LED_Init(LED3);

  /* ### Start counting in interrupt mode ############################# */
  /*
   *  Period = 1000
   */
  if (HAL_LPTIM_Counter_Start_IT(&hlptim2, 1000) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* Insert 1 second delay */
    HAL_Delay(500);

    __set_BASEPRI((DEFAULT_IRQ_PRIO + 1) << (8 - __NVIC_PRIO_BITS));

    /* Back up clock context */
    RCC_backupClocks();

    /* Disable IO Compensation */
    HAL_SYSCFG_DisableIOCompensation();

    /* Clear the Low Power MCU flags before going into CSTOP */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP);

    /* Enter STOP mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* ... STOP mode ... */

    /* Test if system was on STOP mode */
    if(__HAL_PWR_GET_FLAG(PWR_FLAG_STOP) == 1U)
    {
      /* Clear the Low Power MCU flags */
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP);

      /* Restore clocks */
      if (SYSCLKConfig_STOP() != HAL_OK)
      {
        Error_Handler();
      }
    }

    /* Enable IO Compensation */
    if (HAL_SYSCFG_EnableIOCompensation() != HAL_OK)
    {
      Error_Handler();
    }

    /* All level of ITs can interrupt */
    __set_BASEPRI(0U);

  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x0; /* Default reset value */
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = 0x10; /* Default reset value */

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

/**
  * @brief LPTIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM2_Init(void)
{

  hlptim2.Instance = LPTIM2;
  hlptim2.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim2.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim2.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  hlptim2.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim2.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim2.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim2.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim2.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  hlptim2.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim2.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Autoreload match callback in non blocking mode
  * @param  hlptim : LPTIM handle
  * @retval None
  */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  /* Turn on LED3 */
  BSP_LED_Toggle(LED3);
}

/* USER CODE END 4 */

/**
  * @brief  After a STOP platform mode re-enable PLL3 and restore the CM4
  *         clock source muxer and the CM4 prescaler.
  * @note   Use polling mode for timeout generation as code is used
  *         on critical section.
  * @note   RCC HAL macros used in this case to allow MCU to wake-up as quick
  *         as possible from STOP mode.
  * @param  None
  * @retval HAL_StatusTypeDef value
  */
static HAL_StatusTypeDef SYSCLKConfig_STOP(void)
{
  /* Update SystemCoreClock variable */
  SystemCoreClock = HAL_RCC_GetSystemCoreClockFreq();

  /* Reconfigure Systick */
  if (HAL_InitTick(uwTickPrio) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Enable PLL3 if needed */
  if (RCC_ClkInit.MCUInit.MCU_Clock == RCC_MCUSSOURCE_PLL3)
  {
    /* Enable PLL3 */
    __HAL_RCC_PLL3_ENABLE();

    /* Wait till PLL3 is ready */
    __WAIT_EVENT_TIMEOUT(__HAL_RCC_GET_FLAG(RCC_FLAG_PLL3RDY),
                         CLOCKSWITCH_TIMEOUT_VALUE);

    /* Enable PLL3 outputs */
    __HAL_RCC_PLL3CLKOUT_ENABLE(RCC_PLL3_DIVP | RCC_PLL3_DIVQ | RCC_PLL3_DIVR);
  }

  /* Configure MCU clock only */
  __HAL_RCC_MCU_SOURCE(RCC_ClkInit.MCUInit.MCU_Clock);

  /* Wait till MCU is ready */
  __WAIT_EVENT_TIMEOUT(__HAL_RCC_GET_FLAG(RCC_FLAG_MCUSSRCRDY),
                       CLOCKSWITCH_TIMEOUT_VALUE);

  /* Update SystemCoreClock variable */
  SystemCoreClock = HAL_RCC_GetSystemCoreClockFreq();

  /* Reconfigure Systick */
  if (HAL_InitTick(uwTickPrio) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Set MCU division factor */
  __HAL_RCC_MCU_DIV(RCC_ClkInit.MCUInit.MCU_Div);

  /* Wait till MCUDIV is ready */
  __WAIT_EVENT_TIMEOUT(__HAL_RCC_GET_FLAG(RCC_FLAG_MCUDIVRDY),
                       CLOCKSWITCH_TIMEOUT_VALUE);

  /* Update SystemCoreClock variable */
  SystemCoreClock = HAL_RCC_GetSystemCoreClockFreq();

  /* Reconfigure Systick */
  if (HAL_InitTick(uwTickPrio) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Back up clock tree
  * @param  None
  * @retval None
  */
static void RCC_backupClocks(void)
{
  uint32_t *pFLatency = NULL;

  /* Back up MCU clock configuration */
  HAL_RCC_GetClockConfig(&RCC_ClkInit, pFLatency);
}

/**
  * @brief  Prepare IO compensation
  *         For Configuring IO compensation SYSCFG peripheral must be clocked.
  *         For enabling IO compensation ensure CSI Oscillator is enabled and
  *         ready (in RCC).
  *  @note  The CSI oscillator is disabled automatically in Stop mode if not
  *         requested for other usages, and the CSI is always OFF in Standby mode.
  * @param  None
  * @retval HAL_StatusTypeDef value
  */
static HAL_StatusTypeDef Prepare_IOComp(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* SYSCFG Clock enable (used for IO compensation configuration) */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Checks CSI is enabled */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_CSIRDY) != 1U)
  {
    status = HAL_ERROR;
  }

  return status;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug *//* Suspend tick */
  HAL_SuspendTick();

  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
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

