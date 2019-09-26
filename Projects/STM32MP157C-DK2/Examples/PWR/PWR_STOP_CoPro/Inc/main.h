/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Examples/PWR/PWR_STOP_CoPro/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "stm32mp1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32mp15xx_disco.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* Defines related to Clock configuration */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/** @brief  Wait until the condition becomes true or timeout expires
  * @param  __CONDITION__ Condition to be tested
  * @param  __TIMEOUT_VAL__ Time out value in ms
  * @note   Use polling mode as code could be used on critical section (IRQs disabled)
  * @retval HAL_TIMEOUT if time out
  */
#define __WAIT_EVENT_TIMEOUT(__CONDITION__, __TIMEOUT_VAL__)                 \
  do {                                                                       \
    __IO uint32_t count = __TIMEOUT_VAL__ * (SystemCoreClock / 20U / 1000U); \
    do                                                                       \
    {                                                                        \
      if (count-- == 0U)                                                     \
      {                                                                      \
        return  HAL_TIMEOUT;                                                 \
      }                                                                      \
    }                                                                        \
    while (__CONDITION__ == 0U);                                             \
  } while(0)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_WAKEUP_IRQ_PRIO 0U

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
