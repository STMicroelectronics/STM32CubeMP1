/**
  ******************************************************************************
  * @file    res_mgr_conf_stm32h7xx_template.h
  * @author  MCD Application Team
  * @brief   Resources Manager configuration template file.
  *          This file should be copied to the application folder and modified
  *          as follows:
  *            - Rename it to 'res_mgr_conf.h'.
  *            - Update it according your needs.
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

/* This Part may be filled/customized by CubeMX/User */
/******** Shared Resource IDs *************************************************/
enum
{
  RESMGR_ID_ADC1                            ,
  RESMGR_ID_ADC2                            ,
  RESMGR_ID_ADC3                            ,
  RESMGR_ID_BDMA                            ,
  RESMGR_ID_CEC                             ,
  RESMGR_ID_COMP1                           ,
  RESMGR_ID_COMP2                           ,
  RESMGR_ID_CRC                             ,
  RESMGR_ID_CRS                             ,
  RESMGR_ID_DAC1                            ,
  RESMGR_ID_DBGMCU                          ,
  RESMGR_ID_DCMI                            ,
  RESMGR_ID_DFSDM1                          ,
  RESMGR_ID_DLYB_QUADSPI                    ,
  RESMGR_ID_DLYB_SDMMC1                     ,
  RESMGR_ID_DLYB_SDMMC2                     ,
  RESMGR_ID_DMA1                            ,
  RESMGR_ID_DMA2                            ,
  RESMGR_ID_DMA2D                           ,
  RESMGR_ID_DMAMUX1                         ,
  RESMGR_ID_DMAMUX2                         ,
  RESMGR_ID_DSI                             ,
  RESMGR_ID_ETH                             ,
  RESMGR_ID_EXTI                            ,
  RESMGR_ID_EXTI_D1                         ,
  RESMGR_ID_EXTI_D2                         ,
  RESMGR_ID_FDCAN_CCU                       ,
  RESMGR_ID_FDCAN1                          ,
  RESMGR_ID_FDCAN2                          ,
  RESMGR_ID_FLASH                           ,
  RESMGR_ID_FMC                             ,
  RESMGR_ID_GPIOA                           ,
  RESMGR_ID_GPIOB                           ,
  RESMGR_ID_GPIOC                           ,
  RESMGR_ID_GPIOD                           ,
  RESMGR_ID_GPIOE                           ,
  RESMGR_ID_GPIOF                           ,
  RESMGR_ID_GPIOG                           ,
  RESMGR_ID_GPIOH                           ,
  RESMGR_ID_GPIOI                           ,
  RESMGR_ID_GPIOJ                           ,
  RESMGR_ID_GPIOK                           ,
  RESMGR_ID_HRTIM1                          ,
  RESMGR_ID_HSEM                            ,
  RESMGR_ID_I2C1                            ,
  RESMGR_ID_I2C2                            ,
  RESMGR_ID_I2C3                            ,
  RESMGR_ID_I2C4                            ,
  RESMGR_ID_IWDG1                           ,
  RESMGR_ID_IWDG2                           ,
  RESMGR_ID_JPEG                            ,
  RESMGR_ID_LPTIM1                          ,
  RESMGR_ID_LPTIM2                          ,
  RESMGR_ID_LPTIM3                          ,
  RESMGR_ID_LPTIM4                          ,
  RESMGR_ID_LPTIM5                          ,
  RESMGR_ID_LPUART1                         ,
  RESMGR_ID_LTDC                            ,
  RESMGR_ID_MDIOS                           ,
  RESMGR_ID_MDMA                            ,
  RESMGR_ID_OPAMP                           ,
  RESMGR_ID_OPAMP1 = RESMGR_ID_OPAMP        ,
  RESMGR_ID_OPAMP2                          ,
  RESMGR_ID_PWR                             ,
  RESMGR_ID_QUADSPI                         ,
  RESMGR_ID_RCC                             ,
  RESMGR_ID_RCC_C1                          ,
  RESMGR_ID_RCC_C2                          ,
  RESMGR_ID_RNG                             ,
  RESMGR_ID_RTC                             ,
  RESMGR_ID_SAI1                            ,
  RESMGR_ID_SAI2                            ,
  RESMGR_ID_SAI3                            ,
  RESMGR_ID_SAI4                            ,
  RESMGR_ID_SDMMC1                          ,
  RESMGR_ID_SDMMC2                          ,
  RESMGR_ID_SPDIFRX                         ,
  RESMGR_ID_SPI1                            ,
  RESMGR_ID_SPI2                            ,
  RESMGR_ID_SPI3                            ,
  RESMGR_ID_SPI4                            ,
  RESMGR_ID_SPI5                            ,
  RESMGR_ID_SPI6                            ,
  RESMGR_ID_SWPMI1                          ,
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
  RESMGR_ID_TMPSENS                         ,
  RESMGR_ID_UART4                           ,
  RESMGR_ID_UART5                           ,
  RESMGR_ID_UART7                           ,
  RESMGR_ID_UART8                           ,
  RESMGR_ID_USART1                          ,
  RESMGR_ID_USART2                          ,
  RESMGR_ID_USART3                          ,
  RESMGR_ID_USART6                          ,
  RESMGR_ID_USB1_OTG_HS                     ,
  RESMGR_ID_USB2_OTG_FS                     ,
  RESMGR_ID_VREFBUF                         ,
  RESMGR_ID_WWDG1                           ,
  RESMGR_ID_WWDG2                           ,
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

#define RESMGR_USE_DEFAULT_TBL

#ifdef RESMGR_USE_DEFAULT_TBL
static const uint8_t Default_ResTbl[RESMGR_ENTRY_NBR] = {
/* 0:Not assigned | 1:Assigned to core1 | 2:Assigned to core2 */
  0 , /* RESMGR_ID_ADC1                       */
  0 , /* RESMGR_ID_ADC2                       */
  0 , /* RESMGR_ID_ADC3                       */
  0 , /* RESMGR_ID_BDMA                       */
  0 , /* RESMGR_ID_CEC                        */
  0 , /* RESMGR_ID_COMP1                      */
  0 , /* RESMGR_ID_COMP2                      */
  0 , /* RESMGR_ID_CRC                        */
  0 , /* RESMGR_ID_CRS                        */
  0 , /* RESMGR_ID_DAC1                       */
  0 , /* RESMGR_ID_DBGMCU                     */
  0 , /* RESMGR_ID_DCMI                       */
  0 , /* RESMGR_ID_DFSDM1                     */
  0 , /* RESMGR_ID_DLYB_QUADSPI               */
  0 , /* RESMGR_ID_DLYB_SDMMC1                */
  0 , /* RESMGR_ID_DLYB_SDMMC2                */
  0 , /* RESMGR_ID_DMA1                       */
  0 , /* RESMGR_ID_DMA2                       */
  0 , /* RESMGR_ID_DMA2D                      */
  0 , /* RESMGR_ID_DMAMUX1                    */
  0 , /* RESMGR_ID_DMAMUX2                    */
  0 , /* RESMGR_ID_DSI                        */
  0 , /* RESMGR_ID_ETH                        */
  0 , /* RESMGR_ID_EXTI                       */
  0 , /* RESMGR_ID_EXTI_D1                    */
  0 , /* RESMGR_ID_EXTI_D2                    */
  0 , /* RESMGR_ID_FDCAN_CCU                  */
  0 , /* RESMGR_ID_FDCAN1                     */
  0 , /* RESMGR_ID_FDCAN2                     */
  0 , /* RESMGR_ID_FLASH                      */
  0 , /* RESMGR_ID_FMC                        */
  0 , /* RESMGR_ID_GPIOA                      */
  0 , /* RESMGR_ID_GPIOB                      */
  0 , /* RESMGR_ID_GPIOC                      */
  0 , /* RESMGR_ID_GPIOD                      */
  0 , /* RESMGR_ID_GPIOE                      */
  0 , /* RESMGR_ID_GPIOF                      */
  0 , /* RESMGR_ID_GPIOG                      */
  0 , /* RESMGR_ID_GPIOH                      */
  0 , /* RESMGR_ID_GPIOI                      */
  0 , /* RESMGR_ID_GPIOJ                      */
  0 , /* RESMGR_ID_GPIOK                      */
  0 , /* RESMGR_ID_HRTIM1                     */
  0 , /* RESMGR_ID_HSEM                       */
  0 , /* RESMGR_ID_I2C1                       */
  0 , /* RESMGR_ID_I2C2                       */
  0 , /* RESMGR_ID_I2C3                       */
  0 , /* RESMGR_ID_I2C4                       */
  0 , /* RESMGR_ID_IWDG1                      */
  0 , /* RESMGR_ID_IWDG2                      */
  0 , /* RESMGR_ID_JPEG                       */
  0 , /* RESMGR_ID_LPTIM1                     */
  0 , /* RESMGR_ID_LPTIM2                     */
  0 , /* RESMGR_ID_LPTIM3                     */
  0 , /* RESMGR_ID_LPTIM4                     */
  0 , /* RESMGR_ID_LPTIM5                     */
  0 , /* RESMGR_ID_LPUART1                    */
  0 , /* RESMGR_ID_LTDC                       */
  0 , /* RESMGR_ID_MDIOS                      */
  0 , /* RESMGR_ID_MDMA                       */
  0 , /* RESMGR_ID_OPAMP1                     */
  0 , /* RESMGR_ID_OPAMP2                     */
  0 , /* RESMGR_ID_PWR                        */
  0 , /* RESMGR_ID_QUADSPI                    */
  0 , /* RESMGR_ID_RCC                        */
  0 , /* RESMGR_ID_RCC_C1                     */
  0 , /* RESMGR_ID_RCC_C2                     */
  0 , /* RESMGR_ID_RNG                        */
  0 , /* RESMGR_ID_RTC                        */
  0 , /* RESMGR_ID_SAI1                       */
  0 , /* RESMGR_ID_SAI2                       */
  0 , /* RESMGR_ID_SAI3                       */
  0 , /* RESMGR_ID_SAI4                       */
  0 , /* RESMGR_ID_SDMMC1                     */
  0 , /* RESMGR_ID_SDMMC2                     */
  0 , /* RESMGR_ID_SPDIFRX                    */
  0 , /* RESMGR_ID_SPI1                       */
  0 , /* RESMGR_ID_SPI2                       */
  0 , /* RESMGR_ID_SPI3                       */
  0 , /* RESMGR_ID_SPI4                       */
  0 , /* RESMGR_ID_SPI5                       */
  0 , /* RESMGR_ID_SPI6                       */
  0 , /* RESMGR_ID_SWPMI1                     */
  0 , /* RESMGR_ID_SYSCFG                     */
  0 , /* RESMGR_ID_TIM1                       */
  0 , /* RESMGR_ID_TIM12                      */
  0 , /* RESMGR_ID_TIM13                      */
  0 , /* RESMGR_ID_TIM14                      */
  0 , /* RESMGR_ID_TIM15                      */
  0 , /* RESMGR_ID_TIM16                      */
  0 , /* RESMGR_ID_TIM17                      */
  0 , /* RESMGR_ID_TIM2                       */
  0 , /* RESMGR_ID_TIM3                       */
  0 , /* RESMGR_ID_TIM4                       */
  0 , /* RESMGR_ID_TIM5                       */
  0 , /* RESMGR_ID_TIM6                       */
  0 , /* RESMGR_ID_TIM7                       */
  0 , /* RESMGR_ID_TIM8                       */
  0 , /* RESMGR_ID_TMPSENS                    */
  0 , /* RESMGR_ID_UART4                      */
  0 , /* RESMGR_ID_UART5                      */
  0 , /* RESMGR_ID_UART7                      */
  0 , /* RESMGR_ID_UART8                      */
  0 , /* RESMGR_ID_USART1                     */
  0 , /* RESMGR_ID_USART2                     */
  0 , /* RESMGR_ID_USART3                     */
  0 , /* RESMGR_ID_USART6                     */
  0 , /* RESMGR_ID_USB1_OTG_HS                */
  0 , /* RESMGR_ID_USB2_OTG_FS                */
  0 , /* RESMGR_ID_VREFBUF                    */
  0 , /* RESMGR_ID_WWDG1                      */
  0 , /* RESMGR_ID_WWDG2                      */
  0 , /* RESMGR_ID_RESMGR_TABLE               */
};
#endif
/**
  * @}
  */

/* End of CubeMX/User Part*/

/** @defgroup RES_MGR_Lock_Procedure
  * @{
  */

/* Customized Lock Procedure definition  begin */

/* Customized Lock Procedure definition  end */

/**
  * @}
  */

/** @defgroup RES_MGR_RPMSG extension
  * @{
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

/**
  * @}
  */
#endif /* RES_MGR_CONF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
