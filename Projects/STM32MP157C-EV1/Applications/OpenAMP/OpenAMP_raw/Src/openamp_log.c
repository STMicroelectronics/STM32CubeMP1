/**
  ******************************************************************************
  * @file    log.c
  * @author  MCD Application Team
  * @brief   Ressource table
  *
  *   This file provides services for logging
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
/** @addtogroup LOG
  * @{
  */

/** @addtogroup STM32MP1xx_log
  * @{
  */

/** @addtogroup STM32MP1xx_Log_Private_Includes
  * @{
  */
#include "openamp_log.h"
#include <stdio.h>
/**
  * @}
  */

/** @addtogroup STM32MP1xx_Log_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32MP1xx_Log_Private_Defines
  * @{
  */

/**
  * @}
  */

#if defined (__LOG_TRACE_IO_)
char system_log_buf[SYSTEM_TRACE_BUF_SZ];

__weak void log_buff(int ch)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
 static int offset = 0;

	if (offset + 1 >= SYSTEM_TRACE_BUF_SZ)
		offset = 0;

	system_log_buf[offset] = ch;
	system_log_buf[offset++ + 1] = '\0';
}

#endif

#if defined ( __CC_ARM) || (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#define PUTCHAR_PROTOTYPE int stdout_putchar(int ch)
#elif __GNUC__
/* With GCC/RAISONANCE, small log_info (option LD Linker->Libraries->Small log_info
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __attribute__(( weak )) __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int __attribute__(( weak )) fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#if defined (__LOG_UART_IO_) || defined (__LOG_TRACE_IO_)
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
#if defined (__LOG_UART_IO_)
extern UART_HandleTypeDef huart;
  HAL_UART_Transmit(&huart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
#endif
#if defined (__LOG_TRACE_IO_)
	log_buff(ch);
#endif
	return ch;
}
#else
/* No printf output */
#endif

