/**
  ******************************************************************************
  * @file    res_mgr.h
  * @author  MCD Application Team
  * @date    15-December-2017
  * @brief   header for the res_mgr.c file
  ******************************************************************************
  *
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the License; You may not use this fileexcept in compliance with the 
  * License. You may obtain a copy of the License at:
  *                       opensource.org/licenses/BSD-3-Clause
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RES_MGR_H__
#define __RES_MGR_H__

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stddef.h>

#include "stm32mp1xx_hal.h"
#include "res_mgr_conf.h"
/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32_EVAL
  * @{
  */

/** @addtogroup Common
  * @{
  */

/** @addtogroup RES_MGR
  * @{
  */

/** @defgroup RES_MGR
  * @brief
  * @{
  */


/** @defgroup RES_MGR_Exported_Defines
  * @{
  */

/******** Messsages IDs *******************************************************/

#define RESMGR_MSG_INIT                   0x00U
#define RESMGR_MSG_DEINIT                 0xFFU

#define RESMGR_MSG_ASSIGN                 0x01U
#define RESMGR_MSG_RELEASE                0x02U
#define RESMGR_MSG_PEND                   0x03U
#define RESMGR_MSG_REJECT                 0x04U

#define RESMGR_MSG_ASSIGNED               0x11U
#define RESMGR_MSG_RELEASED               0x12U
#define RESMGR_MSG_PENDED                 0x13U
#define RESMGR_MSG_REJECTED               0x14U


/******** Requests Flags ******************************************************/

#define RESMGR_FLAGS_INHERIT_POS          0x00U
#define RESMGR_FLAGS_INHERIT_MSK          (0x01U << RESMGR_FLAGS_INHERIT_POS)
#define RESMGR_FLAGS_INHERIT_HANDLE       RESMGR_FLAGS_INHERIT_MSK


#define RESMGR_FLAGS_ACCESS_POS           0x01U
#define RESMGR_FLAGS_ACCESS_MSK           (0x03U << RESMGR_FLAGS_ACCESS_POS)
#define RESMGR_FLAGS_ACCESS_NORMAL        (0x01U << RESMGR_FLAGS_ACCESS_POS)
#define RESMGR_FLAGS_ACCESS_PEND          (0x02U << RESMGR_FLAGS_ACCESS_POS)

#define RESMGR_FLAGS_OWNER_POS            0x03U
#define RESMGR_FLAGS_OWNER_MSK            (0x03U << RESMGR_FLAGS_OWNER_POS)
#define RESMGR_FLAGS_CPU_SLAVE            (0x01U << RESMGR_FLAGS_OWNER_POS)
#define RESMGR_FLAGS_CPU_MASTER           (0x02U << RESMGR_FLAGS_OWNER_POS)

/******** Resource States *****************************************************/

#define RESMGR_STATE_RELEASED              0x00U
#define RESMGR_STATE_ASSIGNED              0x01U
#define RESMGR_STATE_PENDING               0x02U

/**
  * @}
  */

/** @defgroup RES_MGR_Exported_Types
  * @{
  */

typedef enum
{  RESMGR_OK       = 0x00U,
   RESMGR_ERROR    = 0x01U,
   RESMGR_BUSY     = 0x02U,
} ResMgr_Status_t;

typedef struct __ResMgr_Ctx_t
{
  uint32_t Flags;
  uint32_t State;
  void    *pHandle;
} ResMgr_Ctx_t;

typedef struct __ResEntry_t
{
  uint32_t     Spinlock;
  ResMgr_Ctx_t Ctx;
} ResEntry_t;

typedef enum
{  RESMGR_CLOCK    = 0x00U,
   RESMGR_REGU     = 0x01U,
} ResSystem_t;

typedef struct __ResConfig_t {
  union {
    struct {
      uint32_t index;
      uint8_t name[16];
      uint32_t rate;
    } clock;
    struct {
      uint32_t index;
      uint8_t name[16];
      uint32_t enable;
      uint32_t curr_voltage_mv; /* GetConfig */
      uint32_t min_voltage_mv; /* SetConfig */
      uint32_t max_voltage_mv; /* SetConfig */
    } regu;
  };
} ResConfig_t;

typedef void (* ResMgrSendMsg_t ) (uint32_t id, uint32_t msg);
typedef void (* ResMgrCallback_t ) (uint32_t id, uint32_t msg);

/**
  * @}
  */

/** @defgroup RES_MGR_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup RES_MGR_Exported_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup RES_MGR_Exported_FunctionsPrototype
  * @{
  */
ResMgr_Status_t ResMgr_Init(ResMgrSendMsg_t SendFunct, ResMgrCallback_t Callback);
ResMgr_Status_t ResMgr_DeInit(void);
ResMgr_Status_t ResMgr_Request(uint32_t id, uint32_t flags, uint32_t prio, void *phandle);
ResMgr_Status_t ResMgr_GetResContext(uint32_t id, ResMgr_Ctx_t  *ctx);
ResMgr_Status_t ResMgr_Release(uint32_t id);

ResMgr_Status_t ResMgr_SetConfig(uint32_t id, char *id_name, ResSystem_t res_type, ResConfig_t *config_in, ResConfig_t *config_out);
ResMgr_Status_t ResMgr_GetConfig(uint32_t id, char *id_name, ResSystem_t res_type, ResConfig_t *config_in, ResConfig_t *config_out);

/**
  * @}
  */


#endif /* __RES_MGR_H__ */

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
