/**
  ******************************************************************************
  * @file    lock_resource.c
  * @author  MCD Application Team
  * @brief   This sample code provides hardware semaphore using HSEM for 
  *          synchronization and mutual exclusion between heterogeneous processors 
  *          and those not operating under a single, shared operating system.  
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
#include "lock_resource.h"

extern void Error_Handler(void);

/* Private macro -------------------------------------------------------------*/

/*
 * This macro provides a semaphore id for a dedicated peripheral.
 * This macro shall be configured by user according its needs and aligned with remote processors
 * (e.g. same semaphore id shall be used between processors for a dedicated peripheral)
 */
#define GET_HSEM_SEM_INDEX(__Peripheral__)   (uint8_t)(((GPIO_TypeDef *)(__Peripheral__) == (GPIOA))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOB))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOC))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOD))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOE))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOF))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOG))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOH))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOI))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOJ))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOK))? 0U :\
                                              ((GPIO_TypeDef *)(__Peripheral__) == (GPIOZ))? 0U :\
                                              ((EXTI_TypeDef *)(__Peripheral__) == (EXTI))?  1U : HSEM_SEMID_MAX + 1U)

/* Private user code ---------------------------------------------------------*/


/**
  * @brief  Periph_Lock function is used for register protection of shared @Peripheral
  *         and shall be called before accessing registers of this shared @Peripheral
  *         If Semaphore id is already taken, the function will busy loop waiting for it to
  *         be released, but give up after @Timeout msecs have elapsed.
  * @param  Peripheral: used to identify which peripheral to protect.
  *                     Semaphore id deduced from this peripheral.
  *         Timeout: timeout value in msecs
  * @retval Return Status
  */
LockResource_Status_t Periph_Lock(void* Peripheral, uint32_t Timeout)
{
  uint32_t tickstart = 0U;
  LockResource_Status_t ret = LOCK_RESOURCE_STATUS_OK;

  /* Init tickstart for timeout management*/
  tickstart = HAL_GetTick();

  /* Try to Take HSEM  assigned to the Peripheral */
  while (HAL_HSEM_FastTake(GET_HSEM_SEM_INDEX(Peripheral)) != HAL_OK)
  {

    if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
    {
       ret = LOCK_RESOURCE_STATUS_TIMEOUT;
       Error_Handler();
    }
  }

  return ret;
}

/**
  * @brief  The Periph_Unlock released a previously-acquired semaphore which we want to unlock
  * @param  Peripheral: used to identify which peripheral and the related semaphore
  * @retval None
  */
void Periph_Unlock(void* Peripheral)
{
  /* Release HSEM  */
  HAL_HSEM_Release(GET_HSEM_SEM_INDEX(Peripheral), 0);

}
