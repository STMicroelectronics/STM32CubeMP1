/**
  @page QSPI_ReadWrite_IT QSPI Read/Write in interrupt mode example
  
  @verbatim
  ******************** (C) COPYRIGHT 2019 STMicroelectronics *******************
  * @file    QSPI/QSPI_ReadWrite_IT/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the QSPI Read/Write in interrupt mode example.
  ******************************************************************************
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                       opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  @endverbatim

@par Example Description

This example describes how to erase part of the QSPI memory, write data in IT mode, read data in IT mode 
and compare the result in a forever loop.

LED4 toggles each time a new comparison is good (many times per second)
LED4 toggles every half second as soon as an error is returned by HAL API

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the systick.
Then the System clock source is configured by the SystemClock_Config() function in case of Engineering Mode, this clock configuration is done
by the Firmware running on the Cortex-A7 in case of Production Mode.

In this example, ACLK is configured at 266 MHz.
QSPI prescaler is set to 3, so QSPI frequency is = ACLK/(QSPI_Prescaler+1) = 266 MHz/(3+1) 

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 

  - QSPI/QSPI_ReadWrite_IT/Inc/stm32mp1xx_hal_conf.h HAL configuration file
  - QSPI/QSPI_ReadWrite_IT/Inc/stm32mp1xx_it.h       Interrupt handlers header file
  - QSPI/QSPI_ReadWrite_IT/Inc/main.h               Header for main.c module  
  - QSPI/QSPI_ReadWrite_IT/Src/stm32mp1xx_it.c       Interrupt handlers
  - QSPI/QSPI_ReadWrite_IT/Src/main.c               Main program
  - QSPI/QSPI_ReadWrite_IT/Src/system_stm32mp1xx.c   STM32MP1xx system source file
  - QSPI/QSPI_ReadWrite_IT/Src/stm32mp1xx_hal_msp.c  HAL MSP file    


@par Hardware and Software environment

  - This example runs on STM32MP157CAAx devices.
    
  - This example has been tested with STM32MP157C-EV1 board and can be
    easily tailored to any other supported device and development board.

  - STM32MP157C-EV1 Set-up :
    - Board is configured by default to access QSPI memory

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
