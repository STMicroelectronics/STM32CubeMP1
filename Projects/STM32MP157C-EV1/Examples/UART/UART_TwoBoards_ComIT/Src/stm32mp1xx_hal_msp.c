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

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */
    if(IS_ENGINEERING_BOOT_MODE())
    {
      RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  
      /*##-1- Configure the UART clock source #*/
      RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART35;
      RCC_PeriphCLKInitStruct.Uart35ClockSelection = RCC_UART35CLKSOURCE_HSI;
      HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
    }
  /* USER CODE END USART3_MspInit 0 */

  /* Peripheral clock enable */
  __HAL_RCC_USART3_CLK_ENABLE();

    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB12     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    PERIPH_LOCK(GPIOB);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    PERIPH_UNLOCK(GPIOB);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART3;
    PERIPH_LOCK(GPIOB);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    PERIPH_UNLOCK(GPIOB);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, DEFAULT_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */

  /* Peripheral clock disable */
  __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB12     ------> USART3_RX 
    */
    PERIPH_LOCK(GPIOB);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_12);
    PERIPH_UNLOCK(GPIOB);


    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

