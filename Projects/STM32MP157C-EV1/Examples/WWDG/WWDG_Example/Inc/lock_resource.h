/**
  ******************************************************************************
  * @file    lock_resource.h
  * @author  MCD Application Team
  * @brief   Header for lock_resource.c
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOCK_RESOURCE_H__
#define __LOCK_RESOURCE_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32mp1xx_hal.h"


/* Exported types ------------------------------------------------------------*/
typedef enum
{
  LOCK_RESOURCE_STATUS_OK       = 0x00U,
  LOCK_RESOURCE_STATUS_ERROR    = 0x01U,
  LOCK_RESOURCE_STATUS_TIMEOUT  = 0x02U
} LockResource_Status_t;

/* Exported constants --------------------------------------------------------*/
#define LOCK_RESOURCE_TIMEOUT   100U /* timeout in ms */

/* Exported macro ------------------------------------------------------------*/
#define PERIPH_LOCK(__Periph__)       Periph_Lock(__Periph__, LOCK_RESOURCE_TIMEOUT)
#define PERIPH_UNLOCK(__Periph__)     Periph_Unlock(__Periph__)

/* Exported functions ------------------------------------------------------- */
LockResource_Status_t Periph_Lock(void* Peripheral, uint32_t Timeout);
void Periph_Unlock(void* Peripheral);




#endif /* __LOCK_RESOURCE_H__ */



