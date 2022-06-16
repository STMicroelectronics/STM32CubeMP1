/**
  ******************************************************************************
  * @file    copro_sync.c
  * @author  MCD Application Team
  * @date    08-October-2018
  * @brief   This file provides services to handle synchronization between the
  *          processors.
  *
  ******************************************************************************
  *
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "copro_sync.h"

/** @addtogroup Utilities
  * @{
  */


/** @defgroup COPRO_SYNC_Private_Types
* @{
*/


/**
* @}
*/


/** @defgroup COPRO_SYNC_Private_Defines
* @{
*/


/**
* @}
*/


/** @defgroup COPRO_SYNC_Private_Macros
* @{
*/
#define COPRO_SYNC_CORTEX_A_STATE    TAMP->BKP18R
#define COPRO_SYNC_CORTEX_M_STATE    TAMP->BKP19R

/**
* @}
*/


/** @defgroup COPRO_SYNC_Private_Variables
* @{
*/
extern IPCC_HandleTypeDef hipcc;
CoproSync_ProcState_t RemoteProcState;

/**
* @}
*/


/** @defgroup COPRO_SYNC_Private_FunctionPrototypes
* @{
*/


/**
* @}
*/


/** @defgroup COPRO_SYNC_Private_Functions
* @{
*/

/**
  * @brief  Initializes the Copro Sync utility
  * @param  None
  * @retval Return status
  */
CoproSync_Status_t CoproSync_Init(void)
{
  CoproSync_Status_t ret = COPROSYNC_OK;

  if ((HAL_IPCC_ActivateNotification(&hipcc, COPRO_SYNC_SHUTDOWN_CHANNEL, IPCC_CHANNEL_DIR_RX,
          CoproSync_ShutdownCb) != HAL_OK) ||
      (HAL_IPCC_ActivateNotification(&hipcc, COPRO_SYNC_STATE_UPD_CHANNEL, IPCC_CHANNEL_DIR_RX,
          CoproSync_RProcStateUpdateCb) != HAL_OK))
    ret = COPROSYNC_ERROR;

  return ret;
}

/**
  * @brief  DeInitializes the Copro Sync utility
  * @param  None
  * @retval Return status
  */
CoproSync_Status_t CoproSync_DeInit(void)
{
  CoproSync_Status_t ret = COPROSYNC_OK;

  if ((HAL_IPCC_DeActivateNotification(&hipcc, COPRO_SYNC_SHUTDOWN_CHANNEL, IPCC_CHANNEL_DIR_RX) != HAL_OK) ||
      (HAL_IPCC_DeActivateNotification(&hipcc, COPRO_SYNC_STATE_UPD_CHANNEL, IPCC_CHANNEL_DIR_RX) != HAL_OK))
    ret = COPROSYNC_ERROR;

  return ret;
}

/**
  * @brief  Callback from IPCC Interrupt Handler: Remote Processor asks local processor to shutdown
  * @param  hipcc IPCC handle
  * @param  ChannelIndex Channel number
  * @param  ChannelDir Channel direction
  * @retval None
  */
__weak void CoproSync_ShutdownCb(IPCC_HandleTypeDef * hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
  /* When ready, notify the remote processor that we can be shut down */
  HAL_IPCC_NotifyCPU(hipcc, ChannelIndex, IPCC_CHANNEL_DIR_RX);

  /* And enter stop mode to prevent further activity */
  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
}

/**
  * @brief  Callback from IPCC Interrupt Handler: Remote Processor informs of its new state
  * @param  hipcc IPCC handle
  * @param  ChannelIndex Channel number
  * @param  ChannelDir Channel direction
  * @retval None
  */
__weak void CoproSync_RProcStateUpdateCb(IPCC_HandleTypeDef * hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
  RemoteProcState = (uint32_t)COPRO_SYNC_CORTEX_A_STATE;
  HAL_IPCC_NotifyCPU(hipcc, ChannelIndex, IPCC_CHANNEL_DIR_RX);
}

/**
  * @brief  Get Remote Processor state
  * @param  None
  * @retval Remote Processor state
  */
CoproSync_ProcState_t CoproSync_GetRProcState(void)
{
  return RemoteProcState;
}

/**
  * @brief  Inform the Remote Processor of the local Processor state update
  * @param  LocalProcState Local Processor State
  * @retval Return status
  */
CoproSync_Status_t CoproSync_NotifyProcStateUdpate(CoproSync_ProcState_t LocalProcState)
{
  CoproSync_Status_t ret = COPROSYNC_OK;

  COPRO_SYNC_CORTEX_M_STATE = LocalProcState;

  if (HAL_IPCC_NotifyCPU(&hipcc, COPRO_SYNC_STATE_UPD_CHANNEL, IPCC_CHANNEL_DIR_TX) != HAL_OK)
    ret = COPROSYNC_ERROR;

  return ret;
}

/**
  * @}
  */

/**
  * @}
  */

