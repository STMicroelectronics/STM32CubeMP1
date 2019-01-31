/**
  ******************************************************************************
  * @file    stm32mp1xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   This file provides code for the MSP Initialization 
  *          and de-Initialization codes.
  ******************************************************************************
  *
  * @attention

  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                       opensource.org/licenses/BSD-3-Clause
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern DMA_HandleTypeDef hdma_hash_in;

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

void HAL_HASH_MspInit(HASH_HandleTypeDef* hhash)
{

  /* USER CODE BEGIN HASH_MspInit 0 */

  /* USER CODE END HASH_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_HASH2_CLK_ENABLE();
    __HAL_RCC_DMAMUX_CLK_ENABLE();
	
    /* Peripheral DMA init*/
  
    hdma_hash_in.Instance = DMA2_Stream7;
    hdma_hash_in.Init.Request = DMA_REQUEST_HASH2_IN;
    hdma_hash_in.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_hash_in.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_hash_in.Init.MemInc = DMA_MINC_ENABLE;
    hdma_hash_in.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_hash_in.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_hash_in.Init.Mode = DMA_NORMAL;
    hdma_hash_in.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_hash_in.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_hash_in.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_hash_in.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_hash_in.Init.PeriphBurst = DMA_PBURST_SINGLE;
    
	if (HAL_DMA_DeInit(&hdma_hash_in) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_DMA_Init(&hdma_hash_in) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hhash,hdmain,hdma_hash_in);

  /* USER CODE BEGIN HASH_MspInit 1 */

  /* USER CODE END HASH_MspInit 1 */
}

void HAL_HASH_MspDeInit(HASH_HandleTypeDef* hhash)
{

  /* USER CODE BEGIN HASH_MspDeInit 0 */
    /* Reset HASH peripheral */
    __HAL_RCC_HASH2_FORCE_RESET();
    __HAL_RCC_HASH2_RELEASE_RESET();
  /* USER CODE END HASH_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HASH2_CLK_DISABLE();

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hhash->hdmain);
  /* USER CODE BEGIN HASH_MspDeInit 1 */

  /* USER CODE END HASH_MspDeInit 1 */

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
