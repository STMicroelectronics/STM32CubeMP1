/**
  ******************************************************************************
  * @file    copro_sync.h
  * @author  MCD Application Team
  * @date    08-October-2018
  * @brief   header for the copro_sync.c file
  ******************************************************************************
  *
  * @attention
  *
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COPRO_SYNC_H__
#define __COPRO_SYNC_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32mp1xx_hal.h"
/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32_EVAL
  * @{
  */

/** @addtogroup Common
  * @{
  */

/** @addtogroup COPRO_SYNC
  * @{
  */

/** @defgroup COPRO_SYNC
  * @brief
  * @{
  */


/** @defgroup COPRO_SYNC_Exported_Defines
  * @{
  */


/**
  * @}
  */

/** @defgroup COPRO_SYNC_Exported_Types
  * @{
  */
typedef enum
{  COPROSYNC_OK       = 0x00U,
   COPROSYNC_ERROR    = 0x01U,
} CoproSync_Status_t;

typedef enum
{  COPROSYNC_OFF      = 0x00U,
   COPROSYNC_INIT     = 0x01U,
   COPROSYNC_RUN      = 0x02U,
   COPROSYNC_STOP     = 0x03U,
   COPROSYNC_STANDBY  = 0x04U,
   COPROSYNC_CRASH    = 0x05U,
} CoproSync_ProcState_t;


/**
  * @}
  */

/** @defgroup COPRO_SYNC_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup COPRO_SYNC_Exported_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup COPRO_SYNC_Exported_FunctionsPrototype
  * @{
  */
CoproSync_Status_t CoproSync_Init(void);
CoproSync_Status_t CoproSync_DeInit(void);
CoproSync_ProcState_t CoproSync_GetRProcState(void);
CoproSync_Status_t CoproSync_NotifyProcStateUdpate(CoproSync_ProcState_t LocalProcState);
void CoproSync_ShutdownCb(IPCC_HandleTypeDef * hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);
void CoproSync_RProcStateUpdateCb(IPCC_HandleTypeDef * hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);

/**
  * @}
  */


#endif /* __COPRO_SYNC_H__ */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
