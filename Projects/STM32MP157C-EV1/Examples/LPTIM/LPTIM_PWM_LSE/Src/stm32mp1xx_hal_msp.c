/**
  ******************************************************************************
  * @file    stm32mp1xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions ------------------------------------------------------- */
extern void Error_Handler(void);

/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_HSEM_CLK_ENABLE();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef* hlptim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hlptim->Instance==LPTIM5)
  {
  /* USER CODE BEGIN LPTIM5_MspInit 0 */

    if(IS_ENGINEERING_BOOT_MODE())
    {
      RCC_OscInitTypeDef RCC_OscInitStruct;
      RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

      /* Enable LSE clock */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
      RCC_OscInitStruct.LSEState = RCC_LSE_ON;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
      RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
      RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
      RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      {
          Error_Handler();
      }

      /* Select the LSE clock as LPTIM5 peripheral clock */
      RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM45;
      RCC_PeriphCLKInitStruct.Lptim45ClockSelection = RCC_LPTIM45CLKSOURCE_LSE;
      HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
    }

    /* Force the LPTIM5 Periheral Clock Reset */
    __HAL_RCC_LPTIM5_FORCE_RESET();

    /* Release the LPTIM5 Periheral Clock Reset */
    __HAL_RCC_LPTIM5_RELEASE_RESET();

  /* USER CODE END LPTIM5_MspInit 0 */

    /* Peripheral clock enable */
    __HAL_RCC_LPTIM5_CLK_ENABLE();

    /**LPTIM5 GPIO Configuration
    PA3     ------> LPTIM5_OUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_LPTIM5;
    PERIPH_LOCK(GPIOA);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    PERIPH_UNLOCK(GPIOA);

  /* USER CODE BEGIN LPTIM5_MspInit 1 */

  /* USER CODE END LPTIM5_MspInit 1 */
  }

}

void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef* hlptim)
{
  if(hlptim->Instance==LPTIM5)
  {
  /* USER CODE BEGIN LPTIM5_MspDeInit 0 */

    if(IS_ENGINEERING_BOOT_MODE())
    {
      RCC_OscInitTypeDef RCC_OscInitStruct;
      RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

      /* Disable LSE clock */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
      RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
      RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
      RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
      RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      {
          Error_Handler();
      }

      /* Select the default clock as LPTIM5 peripheral clock */
      RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM45;
      RCC_PeriphCLKInitStruct.Lptim45ClockSelection = RCC_LPTIM45CLKSOURCE_PCLK3;
      HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
    }

	  /* Force the LPTIM5 Periheral Clock Reset */
    __HAL_RCC_LPTIM5_FORCE_RESET();

    /* Release the LPTIM5 Periheral Clock Reset */
    __HAL_RCC_LPTIM5_RELEASE_RESET();

  /* USER CODE END LPTIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPTIM5_CLK_DISABLE();

    /**LPTIM5 GPIO Configuration
    PA3     ------> LPTIM5_OUT
    */
    PERIPH_LOCK(GPIOA);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
    PERIPH_UNLOCK(GPIOA);

  /* USER CODE BEGIN LPTIM5_MspDeInit 1 */

  /* USER CODE END LPTIM5_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

