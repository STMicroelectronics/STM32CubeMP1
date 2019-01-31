/**
  ******************************************************************************
  * @file    res_mgr.c
  * @author  MCD Application Team
  * @date    15-December-2017
  * @brief   This file provides the resources manager services for dual core
  *          products.
  *
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

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "res_mgr.h"
#ifdef RESMGR_WITH_RPMSG
#include "openamp.h"
#endif

/** @addtogroup Utilities
  * @{
  */

#ifdef RESMGR_WITH_RPMSG
extern struct remote_proc *proc; /* Declared and initialized in openamp.c*/
#endif

/** @defgroup RES_MGR_Private_Types
* @{
*/

typedef struct __device_config_t {
    uint32_t id;
    uint32_t device_address;
    uint8_t etpzc_index;
} device_config_t;

#ifdef RESMGR_WITH_RPMSG
/* The rpmsg_srm_message_t and related structures must be aligned with the ones
 * used by the remote processor */

typedef struct __clock_config_t {
    uint32_t index;
    uint8_t name[16];
    uint32_t rate;
} clock_config_t;

typedef struct __regu_config_t {
    uint32_t index;
    uint8_t name[16];
    uint32_t enable;
    uint32_t curr_voltage_mv; /* GetConfig */
    uint32_t min_voltage_mv; /* SetConfig */
    uint32_t max_voltage_mv; /* SetConfig */
} regu_config_t;

typedef struct __rpmsg_srm_message_t {
  uint32_t message_type;
  uint8_t device_id[32];
  uint32_t rsc_type;
  union {
    clock_config_t clock_config;
    regu_config_t regu_config;
  };
} rpmsg_srm_message_t;

typedef enum
{
  RX_NONE,
  RX_RECEIVED,
} rx_status_t;
#endif

/**
* @}
*/


/** @defgroup RES_MGR_Private_Defines
* @{
*/
/* virtual table address in shared memory */
#define SRAM_DECPROT_TABLE      (SRAM_BASE + 0x3F000)

/* ETZPC reg values */
#define DECPROT_MPU_SEC         0x00U
#define DECPROT_NO_SEC_MCU_ISOL 0x02U
#define DECPROT_NO_SEC          0x03U
#define DECPROT_MASK            0x03U
#define DECPROT_NB_BITS         0x02U
#define DECPROT_NB_ENTRIES      0x10U

#define DECPROT_UNLOCKED        0x00U
#define DECPROT_LOCKED          0x01U
#define DECPROT_LOCK_MASK       0x01U
#define DECPROT_LOCK_NB_BITS    0x01U
#define DECPROT_LOCK_NB_ENTRIES 0x20U

#define ETZPC_NO_INDEX          0xFF

#ifdef RESMGR_WITH_RPMSG
#define RESMGR_SERVICE_NAME     "rproc-srm"

#define RPMSG_MSG_GETCONFIG     0x00U
#define RPMSG_MSG_SETCONFIG     0x01U
#define RPMSG_MSG_ERROR         0xFFU

#define RPMSG_RSC_CLOCK         0x00U
#define RPMSG_RSC_REGU          0x01U
#define RPMSG_RSC_ERROR         0xFFU

#define RPMSG_TIMEOUT_MS        1000
#endif
/**
* @}
*/


/** @defgroup RES_MGR_Private_Macros
* @{
*/
/**
* @}
*/


/** @defgroup RES_MGR_Private_Variables
* @{
*/

/* Note: this is MP1 specific */
device_config_t mp1_device_config[] = {
/* Devices under ETZPC control */
  { RESMGR_ID_USART1,       USART1_BASE,    0x03 },
  { RESMGR_ID_SPI6,         SPI6_BASE,      0x04 },
  { RESMGR_ID_I2C4,         I2C4_BASE,      0x05 },
  { RESMGR_ID_RNG1,         RNG1_BASE,      0x07 },
  { RESMGR_ID_HASH1,        HASH1_BASE,     0x08 },
  { RESMGR_ID_CRYP1,        CRYP1_BASE,     0x09 },
  { RESMGR_ID_I2C6,         I2C6_BASE,      0x0C },
  { RESMGR_ID_TIM2,         TIM2_BASE,      0x10 },
  { RESMGR_ID_TIM3,         TIM3_BASE,      0x11 },
  { RESMGR_ID_TIM4,         TIM4_BASE,      0x12 },
  { RESMGR_ID_TIM5,         TIM5_BASE,      0x13 },
  { RESMGR_ID_TIM6,         TIM6_BASE,      0x14 },
  { RESMGR_ID_TIM7,         TIM7_BASE,      0x15 },
  { RESMGR_ID_TIM12,        TIM12_BASE,     0x16 },
  { RESMGR_ID_TIM13,        TIM13_BASE,     0x17 },
  { RESMGR_ID_TIM14,        TIM14_BASE,     0x18 },
  { RESMGR_ID_LPTIM1,       LPTIM1_BASE,    0x19 },
  { RESMGR_ID_SPI2,         SPI2_BASE,      0x1B },
  { RESMGR_ID_SPI3,         SPI3_BASE,      0x1C },
  { RESMGR_ID_SPDIFRX,      SPDIFRX_BASE,   0x1D },
  { RESMGR_ID_USART2,       USART2_BASE,    0x1E },
  { RESMGR_ID_USART3,       USART3_BASE,    0x1F },
  { RESMGR_ID_UART4,        UART4_BASE,     0x20 },
  { RESMGR_ID_UART5,        UART5_BASE,     0x21 },
  { RESMGR_ID_I2C1,         I2C1_BASE,      0x22 },
  { RESMGR_ID_I2C2,         I2C2_BASE,      0x23 },
  { RESMGR_ID_I2C3,         I2C3_BASE,      0x24 },
  { RESMGR_ID_I2C5,         I2C5_BASE,      0x25 },
  { RESMGR_ID_CEC,          CEC_BASE,       0x26 },
  { RESMGR_ID_DAC1,         DAC1_BASE,      0x27 },
  { RESMGR_ID_UART7,        UART7_BASE,     0x28 },
  { RESMGR_ID_UART8,        UART8_BASE,     0x29 },
  { RESMGR_ID_TIM1,         TIM1_BASE,      0x30 },
  { RESMGR_ID_TIM8,         TIM8_BASE,      0x31 },
  { RESMGR_ID_USART6,       USART6_BASE,    0x33 },
  { RESMGR_ID_SPI1,         SPI1_BASE,      0x34 },
  { RESMGR_ID_SPI4,         SPI4_BASE,      0x35 },
  { RESMGR_ID_TIM15,        TIM15_BASE,     0x36 },
  { RESMGR_ID_TIM16,        TIM16_BASE,     0x37 },
  { RESMGR_ID_TIM17,        TIM17_BASE,     0x38 },
  { RESMGR_ID_SPI5,         SPI5_BASE,      0x39 },
  { RESMGR_ID_SAI1,         SAI1_BASE,      0x3A },
  { RESMGR_ID_SAI2,         SAI2_BASE,      0x3B },
  { RESMGR_ID_SAI3,         SAI3_BASE,      0x3C },
  { RESMGR_ID_DFSDM1,       DFSDM1_BASE,    0x3D },
  { RESMGR_ID_FDCAN1,       FDCAN1_BASE,    0x3E }, /* same decprot for all FDCAN */
  { RESMGR_ID_FDCAN2,       FDCAN2_BASE,    0x3E }, /* same decprot for all FDCAN */
  { RESMGR_ID_FDCAN_CCU,    FDCAN_CCU_BASE, 0x3E }, /* same decprot for all FDCAN */
  { RESMGR_ID_LPTIM2,       LPTIM2_BASE,    0x40 },
  { RESMGR_ID_LPTIM3,       LPTIM3_BASE,    0x41 },
  { RESMGR_ID_LPTIM4,       LPTIM4_BASE,    0x42 },
  { RESMGR_ID_LPTIM5,       LPTIM5_BASE,    0x43 },
  { RESMGR_ID_SAI4,         SAI4_BASE,      0x44 },
  { RESMGR_ID_VREFBUF,      VREFBUF_BASE,   0x45 },
  { RESMGR_ID_DCMI,         DCMI_BASE,      0x46 },
  { RESMGR_ID_CRC2,         CRC2_BASE,      0x47 },
  { RESMGR_ID_ADC1,         ADC1_BASE,      0x48 }, /* same decprot for both ADC */
  { RESMGR_ID_ADC2,         ADC2_BASE,      0x48 }, /* same decprot for both ADC */
  { RESMGR_ID_HASH2,        HASH2_BASE,     0x49 },
  { RESMGR_ID_RNG2,         RNG2_BASE,      0x4A },
  { RESMGR_ID_CRYP2,        CRYP2_BASE,     0x4B },
  { RESMGR_ID_USB1_OTG_HS,  USBOTG_BASE,    0x55 },
  { RESMGR_ID_SDMMC3,       SDMMC3_BASE,    0x56 },
  { RESMGR_ID_DLYB_SDMMC3,  DLYB_SDMMC3_BASE,   0x57 },
  { RESMGR_ID_DMA1,         DMA1_BASE,      0x58 },
  { RESMGR_ID_DMA2,         DMA2_BASE,      0x59 },
  { RESMGR_ID_DMAMUX1,      DMAMUX1_BASE,   0x5A },
  { RESMGR_ID_FMC,          FMC_R_BASE,     0x5B },
  { RESMGR_ID_QUADSPI,      QSPI_R_BASE,    0x5C },
  { RESMGR_ID_DLYB_QUADSPI, DLYB_QSPI_BASE,     0x5D },
  { RESMGR_ID_ETH,          ETH_BASE,       0x5E },
/* Devices NOT under ETZPC control */
  { RESMGR_ID_CRC1,         CRC1_BASE,      ETZPC_NO_INDEX },
  { RESMGR_ID_DLYB_SDMMC1,  DLYB_SDMMC1_BASE,   ETZPC_NO_INDEX },
  { RESMGR_ID_DLYB_SDMMC2,  DLYB_SDMMC2_BASE,   ETZPC_NO_INDEX },
  { RESMGR_ID_DSI,          DSI_BASE,       ETZPC_NO_INDEX },
  { RESMGR_ID_GPU,          GPU_BASE,       ETZPC_NO_INDEX },
  { RESMGR_ID_IPCC,         IPCC_BASE,      ETZPC_NO_INDEX },
  { RESMGR_ID_IWDG1,        IWDG1_BASE,     ETZPC_NO_INDEX },
  { RESMGR_ID_IWDG2,        IWDG2_BASE,     ETZPC_NO_INDEX },
  { RESMGR_ID_LTDC,         LTDC_BASE,      ETZPC_NO_INDEX },
  { RESMGR_ID_RTC,          RTC_BASE,       ETZPC_NO_INDEX },
  { RESMGR_ID_SDMMC1,       SDMMC1_BASE,    ETZPC_NO_INDEX },
  { RESMGR_ID_SDMMC2,       SDMMC2_BASE,    ETZPC_NO_INDEX },
  { RESMGR_ID_USB1HSFSP1,   USB1HSFSP1_BASE, ETZPC_NO_INDEX },
  { RESMGR_ID_USB1HSFSP2,   USB1HSFSP2_BASE, ETZPC_NO_INDEX },
  { RESMGR_ID_USBPHYC,      USBPHYC_BASE,   ETZPC_NO_INDEX },
  { RESMGR_ID_DBGMCU,       DBGMCU_BASE,    ETZPC_NO_INDEX },
  { RESMGR_ID_HSEM,         HSEM_BASE,      ETZPC_NO_INDEX },
  { RESMGR_ID_MDIOS,        MDIOS_BASE,     ETZPC_NO_INDEX },
  { RESMGR_ID_MDMA,         MDMA_BASE,      ETZPC_NO_INDEX },
  { RESMGR_ID_SYSCFG,       SYSCFG_BASE,    ETZPC_NO_INDEX },
  { RESMGR_ID_TMPSENS,      DTS_BASE,       ETZPC_NO_INDEX },
  { RESMGR_ID_WWDG1,        WWDG1_BASE,     ETZPC_NO_INDEX },
};

ResEntry_t ResMgr_Tbl[RESMGR_ENTRY_NBR];

#ifdef RESMGR_WITH_RPMSG
struct rpmsg_endpoint resmgr_ept;
static uint8_t received_rpmsg[128];
static uint8_t received_status = RX_NONE;
#endif

/**
* @}
*/


/** @defgroup RES_MGR_Private_FunctionPrototypes
* @{
*/

/**
* @}
*/


/** @defgroup RES_MGR_Private_Functions
* @{
*/
#ifdef RESMGR_WITH_RPMSG
static int ResMgr_read_cb(struct rpmsg_endpoint *ept, void *data,  size_t len, uint32_t src, void *priv)
{
  memcpy(received_rpmsg, data, len > sizeof(received_rpmsg) ? sizeof(received_rpmsg) : len);
  received_status = SET;

  return 0;
}

static uint32_t ResMgr_DeviceAddress(uint32_t id)
{
  uint32_t count;

  for (count = 0; count < sizeof(mp1_device_config) / sizeof(mp1_device_config[0]); count++)
  {
    if (mp1_device_config[count].id == id)
    {
      return mp1_device_config[count].device_address;
    }
  }

  return 0;
}

static ResMgr_Status_t ResMgr_RpmsgInit(void)
{
  ResMgr_Status_t ret = RESMGR_OK;

  if (OPENAMP_create_endpoint(&resmgr_ept, RESMGR_SERVICE_NAME, RPMSG_ADDR_ANY,
                              ResMgr_read_cb, NULL) < 0)
  {
    ret = RESMGR_ERROR;
  }

  return ret;
}

static void ResMgr_RpmsgDeInit(void)
{
  OPENAMP_destroy_ept(&resmgr_ept);
}
#endif

static uint8_t ResMgr_EtzpcIndex(uint32_t id)
{
  uint32_t count;

  for (count = 0; count < sizeof(mp1_device_config) / sizeof(mp1_device_config[0]); count++)
  {
    if (mp1_device_config[count].id == id)
    {
      return mp1_device_config[count].etpzc_index;
    }
  }

  return ETZPC_NO_INDEX;
}

/**
  * @brief  Initializes the Resource Manager
  * @param  SendFunct : function pointer used to send message to the other core. NOT USED.
  * @param  Callback  : function pointer used to publish the status to user side. NOT USED.
  * @retval Return status
  */
ResMgr_Status_t ResMgr_Init(ResMgrSendMsg_t SendFunct, ResMgrCallback_t Callback)
{
  ResMgr_Status_t ret = RESMGR_OK;
  uint32_t count = 0;

  for( ; count < RESMGR_ENTRY_NBR; count++)
  {
    ResMgr_Tbl[count].Spinlock = 0;
    ResMgr_Tbl[count].Ctx.Flags = 0;
    ResMgr_Tbl[count].Ctx.State = RESMGR_STATE_RELEASED;
    ResMgr_Tbl[count].Ctx.pHandle = NULL;
  }

#ifdef RESMGR_WITH_RPMSG
  ret = ResMgr_RpmsgInit();
#endif

  /* SendFunct and Callback are ignored */

  return ret;
}

/**
  * @brief  DeInitializes the Resource Manager
  * @param  None
  * @retval Return status
  */
ResMgr_Status_t ResMgr_DeInit(void)
{
  uint32_t count = 0;

#ifdef RESMGR_WITH_RPMSG
  ResMgr_RpmsgDeInit();
#endif

  for( ; count < RESMGR_ENTRY_NBR; count++)
  {
    ResMgr_Tbl[count].Spinlock = 0;
    ResMgr_Tbl[count].Ctx.Flags = 0;
    ResMgr_Tbl[count].Ctx.State = RESMGR_STATE_RELEASED;
    ResMgr_Tbl[count].Ctx.pHandle = NULL;
  }

  return RESMGR_OK;
}

/**
  * @brief  Request a resource
  * @param  id      : Resource identifier
  * @param  flags   : Request options
  * @param  prio    : Request priority
  * @param  phandle : Resource handle
  * @retval Return status
  */
ResMgr_Status_t ResMgr_Request(uint32_t id, uint32_t flags, uint32_t prio, void *phandle)
{
  __IO uint32_t *regaddr;
  uint32_t reg_offset;
  uint32_t secu_status, lock_status;
  ResMgr_Status_t ret = RESMGR_OK;
  uint32_t etzpc_id;
  uint8_t etzpc_access = true;

  /* Valid case: Slave CPU + NO pending access + NO Inherit */
  if (((flags & RESMGR_FLAGS_OWNER_MSK) == RESMGR_FLAGS_CPU_SLAVE) &&
      ((flags & RESMGR_FLAGS_ACCESS_MSK) != RESMGR_FLAGS_ACCESS_PEND) &&
      ((flags & RESMGR_FLAGS_INHERIT_MSK) != RESMGR_FLAGS_INHERIT_HANDLE))
  {
    /* lock table modification (TBC) */
    while(ResMgr_Tbl[id].Spinlock);
    ResMgr_Tbl[id].Spinlock = 1;

    /* Check whether the resource is free */
    if(ResMgr_Tbl[id].Ctx.State == RESMGR_STATE_RELEASED)
    {
      etzpc_id = ResMgr_EtzpcIndex(id);
      if (etzpc_id != ETZPC_NO_INDEX) {
        /* Check from 'decprot' status if the resource can be assigned to that CPU */
        reg_offset = etzpc_id / DECPROT_NB_ENTRIES;
        regaddr = (&TZPC->DECPROT0 + reg_offset);
        secu_status = *regaddr;
        secu_status >>= DECPROT_NB_BITS * (etzpc_id % DECPROT_NB_ENTRIES);
        secu_status &= DECPROT_MASK;

        reg_offset = etzpc_id / DECPROT_LOCK_NB_ENTRIES;
        regaddr = (&TZPC->DECPROT_LOCK0 + reg_offset);
        lock_status = *regaddr;
        lock_status >>= DECPROT_LOCK_NB_BITS * (etzpc_id % DECPROT_LOCK_NB_ENTRIES);
        lock_status &= DECPROT_LOCK_MASK;

        /* DECPROT_NO_SEC_MCU_ISOL           : exclusively reserved to Cortex-M
         * DECPROT_NO_SEC + DECPROT_UNLOCKED : can be used by Cortex-A and Cortex-M
         * DECPROT_NO_SEC + DECPROT_LOCKED   : exclusively reserved to Cortex-A
         */
        etzpc_access = (secu_status == DECPROT_NO_SEC_MCU_ISOL) ||
                       ((secu_status == DECPROT_NO_SEC) && (lock_status == DECPROT_UNLOCKED));
      }
      if (etzpc_access)
      {
        /* Resource is free, update the table */
        ResMgr_Tbl[id].Ctx.Flags = (flags & ~RESMGR_FLAGS_ACCESS_MSK) | \
                                             RESMGR_FLAGS_ACCESS_NORMAL;
        ResMgr_Tbl[id].Ctx.State = RESMGR_STATE_ASSIGNED;
        if(phandle != NULL)
        {
          ResMgr_Tbl[id].Ctx.pHandle = phandle;
        }
      }
      else
      {
        /* Cannot be assigned to that CPU */
        ret = RESMGR_BUSY;
      }
    }
    else /* Resource already used */
    {
      ret = RESMGR_BUSY;
    }

    ResMgr_Tbl[id].Spinlock = 0;
  }
  else
  {
    ret = RESMGR_ERROR;
  }
  return ret;
}

/**
  * @brief  Release a resource
  * @param  id : Resource identifier
  * @retval Return status
  */
ResMgr_Status_t ResMgr_Release(uint32_t id)
{
  /* lock table modification (TBC) */
  while(ResMgr_Tbl[id].Spinlock);
  ResMgr_Tbl[id].Spinlock = 1;

  /* Update the table */
  ResMgr_Tbl[id].Ctx.State = RESMGR_STATE_RELEASED;

  ResMgr_Tbl[id].Spinlock = 0;

  return RESMGR_OK;
}

/**
  * @brief  Get resource context
  * @param  id  : Resource identifier
  * @param  ctx : Resource context pointer
  * @retval Return status
  */
ResMgr_Status_t ResMgr_GetResContext(uint32_t id, ResMgr_Ctx_t *ctx)
{
  while(ResMgr_Tbl[id].Spinlock);
  ResMgr_Tbl[id].Spinlock = 1;

  ctx->Flags = ResMgr_Tbl[id].Ctx.Flags;
  ctx->State = ResMgr_Tbl[id].Ctx.State;
  ctx->pHandle = ResMgr_Tbl[id].Ctx.pHandle;

  ResMgr_Tbl[id].Spinlock = 0;

  return RESMGR_OK;
}

#ifdef RESMGR_WITH_RPMSG
static uint32_t ResMgr_ToRscType(ResSystem_t res_type)
{
  uint32_t rsc_type;

  switch(res_type)
  {
    case RESMGR_CLOCK:
      rsc_type = RPMSG_RSC_CLOCK;
      break;
    case RESMGR_REGU:
      rsc_type = RPMSG_RSC_REGU;
      break;
    default:
      rsc_type = RPMSG_RSC_ERROR;
  }

  return rsc_type;
}

static ResMgr_Status_t ResMgr_DecodeConfigRpmsg(ResSystem_t res_type, ResConfig_t *config)
{
  ResMgr_Status_t ret = RESMGR_OK;
  rpmsg_srm_message_t *msg_in = (rpmsg_srm_message_t *)received_rpmsg;

  if ((msg_in->message_type != RPMSG_MSG_GETCONFIG) && (msg_in->message_type != RPMSG_MSG_SETCONFIG))
  {
    ret = RESMGR_ERROR;
  }
  else if (msg_in->rsc_type != ResMgr_ToRscType(res_type))
  {
    ret = RESMGR_ERROR;
  }
  else if (config)
  {
    /* decode answer */
    switch(res_type)
    {
      case RESMGR_CLOCK:
        config->clock.index = msg_in->clock_config.index;
        strlcpy((char *)config->clock.name, (char *)msg_in->clock_config.name, sizeof(config->clock.name));
        config->clock.rate = msg_in->clock_config.rate;
        break;
      case RESMGR_REGU:
        config->regu.index = msg_in->regu_config.index;
        strlcpy((char *)config->regu.name, (char *)msg_in->regu_config.name, sizeof(config->regu.name));
        config->regu.enable = msg_in->regu_config.enable;
        config->regu.curr_voltage_mv = msg_in->regu_config.curr_voltage_mv;
        break;
      default:
        ret = RESMGR_ERROR;
    }
  }

  return ret;
}
#endif

/**
  * @brief  Set the configuration of a system resource
  * @param  id              : resource identifier to which the system resource is attached
  * @param  id_name         : resource name to which the system resource is attached (see note)
  * @param  res_type        : resource type
  * @param  config_in       : system resource identification and configuration to set
  * @param  config_out[out] : system resource identification and configuration applied
  * @retval Return status
  *
  * @note RESMGR_WITH_RPSMG must be defined to use this function.
  *
  * @note "id" and "id_name" usage:
  *       With the ResMgr concept a system resource is attached to a device (physical or virtual).
  *       In the majority of cases where a system resource is attached to a physical device (example:
  *       a device clock), the "id" parameter is used to identify the device (ex: RESMGR_ID_UART4).
  *       In the minority of cases where a system resource is not attached to any device (example:
  *       a GPIO driving a LED), the "id_name" parameter is used to identify the virtual device to
  *       which the system resource is attached (ex: "led"). The "id" parameter shall then be set
  *       to RESMGR_ID_NONE.
  */
ResMgr_Status_t ResMgr_SetConfig(uint32_t id, char *id_name, ResSystem_t res_type, ResConfig_t *config_in, ResConfig_t *config_out)
{
#ifdef RESMGR_WITH_RPMSG
  ResMgr_Status_t ret = RESMGR_OK;
  rpmsg_srm_message_t msg_out;
  uint32_t addr, tickstart;

  if (id != RESMGR_ID_NONE)
  {
    addr = ResMgr_DeviceAddress(id);
    sprintf((char *)msg_out.device_id, "%x", (unsigned int) addr);
  }
  else
  {
    strlcpy((char *)msg_out.device_id, id_name, sizeof(msg_out.device_id));
  }

  switch(res_type)
  {
    case RESMGR_CLOCK:
      msg_out.clock_config.index = config_in->clock.index;
      msg_out.clock_config.rate = config_in->clock.rate;
      strlcpy((char *)msg_out.clock_config.name, (char *)config_in->clock.name, sizeof(msg_out.clock_config.name));
      break;
    case RESMGR_REGU:
      msg_out.regu_config.index = config_in->regu.index;
      msg_out.regu_config.enable = config_in->regu.enable;
      msg_out.regu_config.min_voltage_mv = config_in->regu.min_voltage_mv;
      msg_out.regu_config.max_voltage_mv = config_in->regu.max_voltage_mv;
      strlcpy((char *)msg_out.regu_config.name, (char *)config_in->regu.name, sizeof(msg_out.regu_config.name));
      break;
    default:
      ret = RESMGR_ERROR;
  }

  if (ret == RESMGR_OK)
  {
    msg_out.rsc_type = ResMgr_ToRscType(res_type);
    msg_out.message_type = RPMSG_MSG_SETCONFIG;
    received_status = RX_NONE;

    if (OPENAMP_send(&resmgr_ept, &msg_out, sizeof(msg_out)) < 0)
    {
      ret = RESMGR_ERROR;
    }
    else
    {
      tickstart = HAL_GetTick();
      while ((received_status == RX_NONE) && (ret == RESMGR_OK))
      {
        OPENAMP_check_for_message();
        if ((HAL_GetTick() - tickstart ) > RPMSG_TIMEOUT_MS)
        {
          ret = RESMGR_ERROR;
        }
      }

      if (ret == RESMGR_OK)
      {
        ret = ResMgr_DecodeConfigRpmsg(res_type, config_out);
      }
    }
  }

  return ret;
#else
  return RESMGR_ERROR;
#endif
}

/**
  * @brief  Get the configuration of a system resource
  * @param  id              : resource identifier to which the system resource is attached
  * @param  id_name         : resource name to which the system resource is attached (see note)
  * @param  res_type        : resource type
  * @param  config_in       : system resource identification
  * @param  config_out[out] : system resource identification and configuration
  * @retval Return status
  *
  * @note RESMGR_WITH_RPSMG must be defined to use this function.
  *
  * @note "id" and "id_name" usage: see ResMgr_SetConfig()
  */
ResMgr_Status_t ResMgr_GetConfig(uint32_t id, char *id_name, ResSystem_t res_type, ResConfig_t *config_in, ResConfig_t *config_out)
{
#ifdef RESMGR_WITH_RPMSG
  ResMgr_Status_t ret = RESMGR_OK;
  rpmsg_srm_message_t msg_out;
  uint32_t addr, tickstart;

  if (id != RESMGR_ID_NONE)
  {
    addr = ResMgr_DeviceAddress(id);
    sprintf((char *)msg_out.device_id, "%x", (unsigned int) addr);
  }
  else
  {
   strlcpy((char *)msg_out.device_id, id_name, sizeof(msg_out.device_id));
  }

  switch(res_type)
  {
    case RESMGR_CLOCK:
      msg_out.clock_config.index = config_in->clock.index;
      strlcpy((char *)msg_out.clock_config.name, (char *)config_in->clock.name, sizeof(msg_out.clock_config.name));
      break;
    case RESMGR_REGU:
      msg_out.regu_config.index = config_in->regu.index;
      strlcpy((char *)msg_out.regu_config.name, (char *)config_in->regu.name, sizeof(msg_out.regu_config.name));
      break;
    default:
      ret = RESMGR_ERROR;
  }

  if (ret == RESMGR_OK)
  {
    msg_out.rsc_type = ResMgr_ToRscType(res_type);
    msg_out.message_type = RPMSG_MSG_GETCONFIG;
    received_status = RX_NONE;

    if (OPENAMP_send(&resmgr_ept, &msg_out, sizeof(msg_out)) < 0)
    {
      ret = RESMGR_ERROR;
    }
    else
    {
      tickstart = HAL_GetTick();
      while ((received_status == RX_NONE) && (ret == RESMGR_OK))
      {
        OPENAMP_check_for_message();
        if ((HAL_GetTick() - tickstart ) > RPMSG_TIMEOUT_MS)
        {
          ret = RESMGR_ERROR;
        }
      }

      if (ret == RESMGR_OK)
      {
        ret = ResMgr_DecodeConfigRpmsg(res_type, config_out);
      }
    }
  }

  return ret;
#else
  return RESMGR_ERROR;
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
