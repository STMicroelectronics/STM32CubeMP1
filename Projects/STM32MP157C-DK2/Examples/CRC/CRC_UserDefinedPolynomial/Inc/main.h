/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @brief   Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32mp1xx_hal.h"
#include "lock_resource.h"

/* USER CODE BEGIN Includes */
#include "stm32mp15xx_disco.h"
/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
#define DEFAULT_IRQ_PRIO      1U

#define BT_HOST_WAKE_Pin GPIO_PIN_5
#define BT_HOST_WAKE_GPIO_Port GPIOH
#define USART2_TX_Pin GPIO_PIN_5
#define USART2_TX_GPIO_Port GPIOD
#define SDMMC2_CMD_Pin GPIO_PIN_6
#define SDMMC2_CMD_GPIO_Port GPIOG
#define SDMMC2_D2_Pin GPIO_PIN_3
#define SDMMC2_D2_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_2
#define INT_GPIO_Port GPIOF
#define WL_REG_ON_Pin GPIO_PIN_4
#define WL_REG_ON_GPIO_Port GPIOH
#define USART2_RTS_Pin GPIO_PIN_4
#define USART2_RTS_GPIO_Port GPIOD
#define WL_HOST_WAKE_Pin GPIO_PIN_0
#define WL_HOST_WAKE_GPIO_Port GPIOD
#define SDMMC2_D1_Pin GPIO_PIN_15
#define SDMMC2_D1_GPIO_Port GPIOB
#define SDMMC2_D3_Pin GPIO_PIN_4
#define SDMMC2_D3_GPIO_Port GPIOB
#define TE_Pin GPIO_PIN_6
#define TE_GPIO_Port GPIOC
#define SDMMC2_CK_Pin GPIO_PIN_3
#define SDMMC2_CK_GPIO_Port GPIOE
#define SDMMC2_D0_Pin GPIO_PIN_14
#define SDMMC2_D0_GPIO_Port GPIOB
#define BL_CTRL_Pin GPIO_PIN_15
#define BL_CTRL_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_6
#define USART2_RX_GPIO_Port GPIOD
#define uSD_DETECT_Pin GPIO_PIN_7
#define uSD_DETECT_GPIO_Port GPIOB
#define SDMMC1_CMD_Pin GPIO_PIN_2
#define SDMMC1_CMD_GPIO_Port GPIOD
#define SDMMC1_CK_Pin GPIO_PIN_12
#define SDMMC1_CK_GPIO_Port GPIOC
#define USART2_CTS_Pin GPIO_PIN_3
#define USART2_CTS_GPIO_Port GPIOD
#define SDMMC1_D2_Pin GPIO_PIN_10
#define SDMMC1_D2_GPIO_Port GPIOC
#define SDMMC1_D3_Pin GPIO_PIN_11
#define SDMMC1_D3_GPIO_Port GPIOC
#define SDMMC1_D1_Pin GPIO_PIN_9
#define SDMMC1_D1_GPIO_Port GPIOC
#define SDMMC1_D0_Pin GPIO_PIN_8
#define SDMMC1_D0_GPIO_Port GPIOC
#define RSTN_Pin GPIO_PIN_4
#define RSTN_GPIO_Port GPIOE
#define BT_PCM_SDI_Pin GPIO_PIN_1
#define BT_PCM_SDI_GPIO_Port GPIOZ
#define I2C4_SCL_Pin GPIO_PIN_4
#define I2C4_SCL_GPIO_Port GPIOZ
#define BT_PCM_CLK_Pin GPIO_PIN_0
#define BT_PCM_CLK_GPIO_Port GPIOZ
#define BT_PCM_WS_Pin GPIO_PIN_3
#define BT_PCM_WS_GPIO_Port GPIOZ
#define BT_REG_ON_Pin GPIO_PIN_6
#define BT_REG_ON_GPIO_Port GPIOZ
#define I2C4_SDA_Pin GPIO_PIN_5
#define I2C4_SDA_GPIO_Port GPIOZ
#define BT_DEV_WAKE_Pin GPIO_PIN_7
#define BT_DEV_WAKE_GPIO_Port GPIOZ
#define BT_PCM_SDO_Pin GPIO_PIN_2
#define BT_PCM_SDO_GPIO_Port GPIOZ
#define PMIC_WAKEUP_Pin GPIO_PIN_13
#define PMIC_WAKEUP_GPIO_Port GPIOC
#define LPO_32_Pin GPIO_PIN_8
#define LPO_32_GPIO_Port GPIOI
#define PA13_Pin GPIO_PIN_13
#define PA13_GPIO_Port GPIOA
#define STUSB1600_IRQOUTn_Pin GPIO_PIN_11
#define STUSB1600_IRQOUTn_GPIO_Port GPIOI
#define PA14_Pin GPIO_PIN_14
#define PA14_GPIO_Port GPIOA
#define HDMI_INT_Pin GPIO_PIN_1
#define HDMI_INT_GPIO_Port GPIOG
#define LED_Y_Pin GPIO_PIN_7
#define LED_Y_GPIO_Port GPIOH
#define ETH_TXD3_Pin GPIO_PIN_2
#define ETH_TXD3_GPIO_Port GPIOE
#define ETH_TXD2_Pin GPIO_PIN_2
#define ETH_TXD2_GPIO_Port GPIOC
#define I2C1_SDA_Pin GPIO_PIN_15
#define I2C1_SDA_GPIO_Port GPIOF
#define ETH_CLK125_Pin GPIO_PIN_5
#define ETH_CLK125_GPIO_Port GPIOG
#define STLINK_RX_Pin GPIO_PIN_11
#define STLINK_RX_GPIO_Port GPIOG
#define ETH_CLK_Pin GPIO_PIN_5
#define ETH_CLK_GPIO_Port GPIOB
#define AUDIO_RST_Pin GPIO_PIN_9
#define AUDIO_RST_GPIO_Port GPIOG
#define HDMI_CEC_Pin GPIO_PIN_6
#define HDMI_CEC_GPIO_Port GPIOB
#define STLINK_TX_Pin GPIO_PIN_2
#define STLINK_TX_GPIO_Port GPIOB
#define HDMI_NRST_Pin GPIO_PIN_10
#define HDMI_NRST_GPIO_Port GPIOA
#define I2C1_SCL_Pin GPIO_PIN_12
#define I2C1_SCL_GPIO_Port GPIOD
#define ETH_TXD1_Pin GPIO_PIN_14
#define ETH_TXD1_GPIO_Port GPIOG
#define ETH_TXD0_Pin GPIO_PIN_13
#define ETH_TXD0_GPIO_Port GPIOG
#define ETH_RX_CLK_Pin GPIO_PIN_1
#define ETH_RX_CLK_GPIO_Port GPIOA
#define ETH_MDC_Pin GPIO_PIN_1
#define ETH_MDC_GPIO_Port GPIOC
#define ETH_RXD3_Pin GPIO_PIN_1
#define ETH_RXD3_GPIO_Port GPIOB
#define ETH_TX_EN_Pin GPIO_PIN_11
#define ETH_TX_EN_GPIO_Port GPIOB
#define ETH_GTX_CLK_Pin GPIO_PIN_4
#define ETH_GTX_CLK_GPIO_Port GPIOG
#define PA0_WKUP_Pin GPIO_PIN_0
#define PA0_WKUP_GPIO_Port GPIOA
#define ETH_RXD2_Pin GPIO_PIN_0
#define ETH_RXD2_GPIO_Port GPIOB
#define ETH_RXD1_Pin GPIO_PIN_5
#define ETH_RXD1_GPIO_Port GPIOC
#define ETH_RX_DV_Pin GPIO_PIN_7
#define ETH_RX_DV_GPIO_Port GPIOA
#define ETH_MIO_Pin GPIO_PIN_2
#define ETH_MIO_GPIO_Port GPIOA
#define ETH_RXD0_Pin GPIO_PIN_4
#define ETH_RXD0_GPIO_Port GPIOC
#define ETH_MDINT_Pin GPIO_PIN_6
#define ETH_MDINT_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_11
#define LED_B_GPIO_Port GPIOD

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
