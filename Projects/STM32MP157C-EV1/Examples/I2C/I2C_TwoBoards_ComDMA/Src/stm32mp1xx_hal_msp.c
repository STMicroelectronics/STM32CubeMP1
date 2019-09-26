/**
  ******************************************************************************
  * @file    stm32mp1xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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

extern DMA_HandleTypeDef hdma_i2c5_tx;

extern DMA_HandleTypeDef hdma_i2c5_rx;
extern void Error_Handler(void);

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

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C5)
  {
  /* USER CODE BEGIN I2C5_MspInit 0 */
    if(IS_ENGINEERING_BOOT_MODE())
    {
      RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  
      /*##-1- Configure the I2C clock source #*/
      RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C35;
      RCC_PeriphCLKInitStruct.I2c35ClockSelection = RCC_I2C35CLKSOURCE_HSI;
      HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
    }
  /* USER CODE END I2C5_MspInit 0 */
  
    /**I2C5 GPIO Configuration    
    PA12     ------> I2C5_SDA
    PA11     ------> I2C5_SCL 
    */

    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C5;
    PERIPH_LOCK(GPIOA);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    PERIPH_UNLOCK(GPIOA);

    /* Peripheral clock enable */
    __HAL_RCC_I2C5_CLK_ENABLE();
    /* I2C5 DMA Init */
    /* I2C5_TX Init */
    hdma_i2c5_tx.Instance = DMA2_Stream1;
    hdma_i2c5_tx.Init.Request = DMA_REQUEST_I2C5_TX;
    hdma_i2c5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c5_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c5_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c5_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    //HAL_DMA_DeInit(&hdma_i2c5_tx);
    if (HAL_DMA_Init(&hdma_i2c5_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2c,hdmatx,hdma_i2c5_tx);

    /* I2C5_RX Init */
    hdma_i2c5_rx.Instance = DMA2_Stream0;
    hdma_i2c5_rx.Init.Request = DMA_REQUEST_I2C5_RX;
    hdma_i2c5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c5_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c5_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2c5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    //HAL_DMA_DeInit(&hdma_i2c5_rx);
    if (HAL_DMA_Init(&hdma_i2c5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2c,hdmarx,hdma_i2c5_rx);

    /* I2C5 interrupt Init */
    HAL_NVIC_SetPriority(I2C5_EV_IRQn, DEFAULT_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(I2C5_EV_IRQn);
    HAL_NVIC_SetPriority(I2C5_ER_IRQn, DEFAULT_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(I2C5_ER_IRQn);
  /* USER CODE BEGIN I2C5_MspInit 1 */

  /* USER CODE END I2C5_MspInit 1 */
  }

}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C5)
  {
  /* USER CODE BEGIN I2C5_MspDeInit 0 */

  /* USER CODE END I2C5_MspDeInit 0 */

  /* Peripheral clock disable */
  __HAL_RCC_I2C5_CLK_DISABLE();

    /**I2C5 GPIO Configuration    
    PA12     ------> I2C5_SDA
    PA11     ------> I2C5_SCL 
    */
    PERIPH_LOCK(GPIOA);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    PERIPH_UNLOCK(GPIOA);

    /* I2C5 DMA DeInit */
    HAL_DMA_DeInit(hi2c->hdmatx);
    HAL_DMA_DeInit(hi2c->hdmarx);

    /* I2C5 interrupt DeInit */
    HAL_NVIC_DisableIRQ(I2C5_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C5_ER_IRQn);
  /* USER CODE BEGIN I2C5_MspDeInit 1 */

  /* USER CODE END I2C5_MspDeInit 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
