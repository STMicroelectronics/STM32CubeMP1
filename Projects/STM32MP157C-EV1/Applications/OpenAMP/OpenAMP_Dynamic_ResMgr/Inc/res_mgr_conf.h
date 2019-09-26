/**
  ******************************************************************************
  * @file    res_mgr_conf.h
  * @author  MCD Application Team
  * @brief   Resources Manager configuration file.
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
#ifndef  RES_MGR_CONF_H__
#define  RES_MGR_CONF_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
/** @addtogroup Utilities
  * @{
  */

/** @addtogroup Common
  * @{
  */

/** @addtogroup RES_MGR_TABLE
  * @{
  */

/** @defgroup RES_MGR_TABLE
  * @brief
  * @{
  */


/** @defgroup RES_MGR_Exported_Defines
  * @{
  */

/* On stm32mp1xxx products, this part MUST NOT be modified */
/******** Shared Resource IDs *************************************************/
enum
{
  RESMGR_ID_ADC1                            ,
  RESMGR_ID_ADC2                            ,
  RESMGR_ID_CEC                             ,
  RESMGR_ID_CRC                             ,
  RESMGR_ID_CRC1 = RESMGR_ID_CRC            ,
  RESMGR_ID_CRC2                            ,
  RESMGR_ID_CRYP1                           ,
  RESMGR_ID_CRYP2                           ,
  RESMGR_ID_DAC1                            ,
  RESMGR_ID_DBGMCU                          ,
  RESMGR_ID_DCMI                            ,
  RESMGR_ID_DFSDM1                          ,
  RESMGR_ID_DLYB_QUADSPI                    ,
  RESMGR_ID_DLYB_SDMMC1                     ,
  RESMGR_ID_DLYB_SDMMC2                     ,
  RESMGR_ID_DLYB_SDMMC3                     ,
  RESMGR_ID_DMA1                            ,
  RESMGR_ID_DMA2                            ,
  RESMGR_ID_DMAMUX1                         ,
  RESMGR_ID_DSI                             ,
  RESMGR_ID_ETH                             ,
  RESMGR_ID_FDCAN_CCU                       ,
  RESMGR_ID_FDCAN1                          ,
  RESMGR_ID_FDCAN2                          ,
  RESMGR_ID_FMC                             ,
  RESMGR_ID_GPU                             ,
  RESMGR_ID_HASH1                           ,
  RESMGR_ID_HASH2                           ,
  RESMGR_ID_HSEM                            ,
  RESMGR_ID_I2C1                            ,
  RESMGR_ID_I2C2                            ,
  RESMGR_ID_I2C3                            ,
  RESMGR_ID_I2C4                            ,
  RESMGR_ID_I2C5                            ,
  RESMGR_ID_I2C6                            ,
  RESMGR_ID_IPCC                            ,
  RESMGR_ID_IWDG1                           ,
  RESMGR_ID_IWDG2                           ,
  RESMGR_ID_LPTIM1                          ,
  RESMGR_ID_LPTIM2                          ,
  RESMGR_ID_LPTIM3                          ,
  RESMGR_ID_LPTIM4                          ,
  RESMGR_ID_LPTIM5                          ,
  RESMGR_ID_LTDC                            ,
  RESMGR_ID_MDIOS                           ,
  RESMGR_ID_MDMA                            ,
  RESMGR_ID_QUADSPI                         ,
  RESMGR_ID_RNG                             ,
  RESMGR_ID_RNG1 = RESMGR_ID_RNG            ,
  RESMGR_ID_RNG2                            ,
  RESMGR_ID_RTC                             ,
  RESMGR_ID_SAI1                            ,
  RESMGR_ID_SAI2                            ,
  RESMGR_ID_SAI3                            ,
  RESMGR_ID_SAI4                            ,
  RESMGR_ID_SDMMC1                          ,
  RESMGR_ID_SDMMC2                          ,
  RESMGR_ID_SDMMC3                          ,
  RESMGR_ID_SPDIFRX                         ,
  RESMGR_ID_SPI1                            ,
  RESMGR_ID_SPI2                            ,
  RESMGR_ID_SPI3                            ,
  RESMGR_ID_SPI4                            ,
  RESMGR_ID_SPI5                            ,
  RESMGR_ID_SPI6                            ,
  RESMGR_ID_SYSCFG                          ,
  RESMGR_ID_TIM1                            ,
  RESMGR_ID_TIM12                           ,
  RESMGR_ID_TIM13                           ,
  RESMGR_ID_TIM14                           ,
  RESMGR_ID_TIM15                           ,
  RESMGR_ID_TIM16                           ,
  RESMGR_ID_TIM17                           ,
  RESMGR_ID_TIM2                            ,
  RESMGR_ID_TIM3                            ,
  RESMGR_ID_TIM4                            ,
  RESMGR_ID_TIM5                            ,
  RESMGR_ID_TIM6                            ,
  RESMGR_ID_TIM7                            ,
  RESMGR_ID_TIM8                            ,
  RESMGR_ID_DTS                             ,
  RESMGR_ID_UART4                           ,
  RESMGR_ID_UART5                           ,
  RESMGR_ID_UART7                           ,
  RESMGR_ID_UART8                           ,
  RESMGR_ID_USART1                          ,
  RESMGR_ID_USART2                          ,
  RESMGR_ID_USART3                          ,
  RESMGR_ID_USART6                          ,
  RESMGR_ID_USB1HSFSP1                      ,
  RESMGR_ID_USB1HSFSP2                      ,
  RESMGR_ID_USB1_OTG_HS                     ,
  RESMGR_ID_USBPHYC                         ,
  RESMGR_ID_VREFBUF                         ,
  RESMGR_ID_WWDG1                           ,
  RESMGR_ID_RESMGR_TABLE                    ,
};

#define RESMGR_ENTRY_NBR    ((uint32_t)RESMGR_ID_RESMGR_TABLE + 1UL)
#define RESMGR_ID_ALL                      0x0000FFFFU
#define RESMGR_ID_NONE                     0xFFFFFFFFU

/**
  * @}
  */


/** @defgroup RES_MGR_Default_ResTable
  * @{
  */
/* On stm32mp1xxx products, the default resource table is NOT used */

/**
  * @}
  */

/** @defgroup RES_MGR_Lock_Procedure
  * @{
  */

/* Customized Lock Procedure definition  begin */
/* On stm32mp1xxx products, the lock functions are NOT used */

/* Customized Lock Procedure definition  end */

/**
  * @}
  */

/** @defgroup RES_MGR_RPMSG extension
  * @{
  */
#define RESMGR_WITH_RPMSG

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
#endif /* RES_MGR_CONF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
