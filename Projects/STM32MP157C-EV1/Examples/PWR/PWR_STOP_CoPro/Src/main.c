/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Examples/PWR/PWR_STOP_CoPro/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32MP1xx PWR HAL API to enter
  *          and exit the STOP mode.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_TOGGLE_DELAY         100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static __IO uint32_t TimingDelay = LED_TOGGLE_DELAY;
RCC_ClkInitTypeDef  RCC_ClkInit;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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

  /* USER CODE BEGIN 2 */

  /* Configure LED4 */
  BSP_LED_Init(LED4);

  /* User push-button (EXTI14) will be used to wakeup the system from STOP mode */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Insert 5 second delay */
    HAL_Delay(5000);

    /* CRITICAL SECTION STARTS HERE!
     * IRQs will be masked (Only RCC IRQ allowed).
     * Eg. SysTick IRQ won't be able to increment uwTick HAL variable, use a
     * polling method if delays or timeouts are required.
     */

    /* (C)STOP protection mechanism
     * Only the IT with the highest priority (0 value) can interrupt.
     * RCC_WAKEUP_IRQn IT is intended to have the highest priority and to be the
     * only one IT having this value
     * RCC_WAKEUP_IRQn is generated only when RCC is completely resumed from
     * CSTOP (protection mechanism)
     */

    __set_BASEPRI((RCC_WAKEUP_IRQ_PRIO + 1) << (8 - __NVIC_PRIO_BITS));

    /* Back up clock context */
    RCC_backupClocks();

    /* Disable IO Compensation */
    HAL_SYSCFG_DisableIOCompensation();

    /* Turn off LED4 */
    BSP_LED_Off(LED4);

    /* Clear the Low Power MCU flags before going into CSTOP */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP);

    /* Enter STOP mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* ... STOP mode ... */

    /* At  STOP mode exit, enable and select PLL3 as system clock source
       (PLL3 & PLL4 are disabled in STOP mode) */

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

    /* CRITICAL SECTION ENDS HERE */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
}

/* USER CODE BEGIN 4 */

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
  * @brief SYSTICK callback
  * @param None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
  if (TimingDelay != 0)
  {
    TimingDelay--;
  }
  else
  {
    /* Toggle LED4 */
    BSP_LED_Toggle(LED4);
    TimingDelay = LED_TOGGLE_DELAY;
  }
}

/**
  * @brief EXTI line detection rising callback
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USER_BUTTON_PIN)
  {
    /* Switch on LED4 */
    BSP_LED_On(LED4);
  }
}

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
