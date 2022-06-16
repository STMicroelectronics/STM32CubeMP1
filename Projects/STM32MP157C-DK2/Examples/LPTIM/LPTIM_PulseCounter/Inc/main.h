/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @brief   Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32mp1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32mp15xx_disco.h"
/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
#define DEFAULT_IRQ_PRIO      0U

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
