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
#include "stm32mp15xx_eval.h"
/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
#define DEFAULT_IRQ_PRIO      1U

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
/* MX25L51245G Macronix memory */
/* Size of the flash */
#define QSPI_FLASH_SIZE                      23
#define QSPI_PAGE_SIZE                       256

/* Read Device Identification */
#define READ_EMS_CMD					0x90
#define READ_ID_CMD					0x9F
#define READ_ES_CMD					0xAB

/* Register Access */
#define READ_STATUS_REG_CMD                             0x05
#define READ_CFG_REG_CMD                                0x15
#define WRITE_REG_CMD                                   0x01

#define WRITE_DISABLE_CMD                               0x04
#define WRITE_ENABLE_CMD                                0x06

/* VOL Register Operations */
#define READ_VOL_CFG_REG_CMD                 0x05
#define WRITE_VOL_CFG_REG_CMD                0x01
//#define READ_DATA_LEARNING_REG                        0x41
//#define WRITE_VOL_DATA_LEARNING_REG                   0x4A

/* Read Flash Array */
#define READ_CMD					0x03
#define READ_4_BYTE_ADDR_CMD				0x13

/* Fast Read Operations */
#define FAST_READ_CMD					0x0B
#define FAST_READ_4_BYTE_ADDR_CMD			0x0C

/* DDR Fast Read Operations */
#define FAST_READ_DDR_CMD				0x0D 
#define FAST_READ_DDR_4_BYTE_ADDR_CMD			0x0E

/* Dual Out Read Operations */
#define DUAL_OUT_FAST_READ_CMD				0x3B
#define DUAL_OUT_FAST_READ_4_BYTE_ADDR_CMD		0x3C

/* Quad Out Read Operations */
#define QUAD_OUT_FAST_READ_CMD				0x6B
#define QUAD_OUT_FAST_READ_4_BYTE_ADDR_CMD		0x6C

/* Dual InOut Read Operations */
#define DUAL_INOUT_FAST_READ_CMD			0xBB
#define DUAL_INOUT_FAST_READ_4_BYTE_ADDR_CMD		0xBC

/* DDR Dual InOut Read Operations */
#define DUAL_INOUT_FAST_READ_DDR_CMD			0xBD 
#define DUAL_INOUT_FAST_READ_DDR_4_BYTE_ADDR_CMD	0xBE

/* Quad InOut Read Operations */
#define QUAD_INOUT_FAST_READ_CMD			0xEB
#define QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD 		0xEC

/* DDR Quad InOut Read Operations */
#define QUAD_INOUT_FAST_READ_DDR_CMD			0xED
#define QUAD_INOUT_FAST_READ_DDR_4_BYTE_ADDR_CMD	0xEE 

/* Program Flash Array */
#define PAGE_PROG_CMD                                   0x02
#define PAGE_PROG_4_BYTE_ADDR_CMD                       0x12

#define QUAD_IN_FAST_PROG_CMD                           0x38
//#define QUAD_IN_FAST_PROG_2_CMD                         0x32
#define QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD               0x3E

/* Erase Flash Array */
#define BULK_ERASE_CMD                                  0x60
#define BULK_ERASE_2_CMD                                0xC7

#define SECTOR_ERASE_CMD                                0x20
#define SECTOR_ERASE_4_BYTE_ADDR_CMD                    0x21

#define BLOCK_ERASE_32K_CMD                             0x52
#define BLOCK_ERASE_64K_CMD                             0xD8
#define BLOCK_ERASE_32K_4_BYTE_ADDR_CMD                 0x5C
#define BLOCK_ERASE_64K_4_BYTE_ADDR_CMD                 0xDC

/* Reset */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99
#define RESET_FACTORY_CMD		                0x41

#define PROG_ERASE_RESUME_CMD                           0x30
#define PROG_ERASE_SUSPEND_CMD                          0xB0

/* 4-byte Address Mode Operations */
#define ENTER_4_BYTE_ADDR_MODE_CMD                      0xB7
#define EXIT_4_BYTE_ADDR_MODE_CMD                       0xE9

/* Quad Operations */
#define ENTER_QUAD_CMD                                  0x35
#define EXIT_QUAD_CMD                                   0xF5

/* Default dummy clocks cycles */
/* Dummy Cycle DC=11 */
/* Read */
#define DUMMY_CLOCK_CYCLES_READ                         0
#define DUMMY_CLOCK_CYCLES_READ_QUAD                    6

/* End address of the QSPI memory */
#define QSPI_END_ADDR              (1 << QSPI_FLASH_SIZE)

/* Size of buffers */
#define BUFFERSIZE                 (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)        (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
