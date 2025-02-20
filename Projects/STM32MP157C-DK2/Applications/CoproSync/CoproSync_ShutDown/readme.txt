/**
  @page CoproSync Shutdown example
  
  @verbatim
  ******************** (C) COPYRIGHT 2021 STMicroelectronics *******************
  * @file    CoproSync/CoproSync_Shutdown/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of CoproSync Shutdown example.
  ******************************************************************************
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @endverbatim

@par Example Description

Use of CoproSync library to make possible the sending of shutdown information 
to Cortex-M4 so that it is able to take necessary actions before going to reset state.

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the systick.
Then the System clock source is configured by the SystemClock_Config() function in case of Engineering Mode, this clock configuration is done
by the Firmware running on the Cortex-A7 in case of Production Mode.

The WWDG1 peripheral configuration is ensured by the HAL_WWDG_Init() function.
This later is calling the HAL_WWDG_MspInit()function which core is implementing
the configuration of the needed WWDG1 resources according to the used hardware (CLOCK 
and NVIC). You may update this function to change WWDG1 configuration.

The WWDG1 timeout is set, through counter value, to 21,376 ms. 
The refresh window is set in order to make user wait 20 ms after a wadchdog refresh, 
before writing again counter. Hence the WWDG1 counter is refreshed each 20 ms in the 
main program infinite loop to prevent a WWDG1 reset. 
LED7 is toggling at same frequency, indicating that the program is running.

An EXTI Line is connected to a GPIO pin, and configured to generate an interrupt
on the rising edge of the signal.

The EXTI Line is used to simulate a software failure: once the EXTI Line event 
occurs by pressing the User push-button (PA.14), the corresponding interrupt is served.

In the ISR, a write to invalid address generates a Hardfault exception containing
an infinite loop and preventing to return to main program (the WWDG counter is 
not refreshed).
As a result, when the WWDG counter falls to 0x3F, WWDG reset occurs.

If the WWDG reset is generated, after the system resumes from reset, LED7 is turned ON for 4 seconds.

If the EXTI Line event does not occur, the WWDG counter is indefinitely refreshed
in the main program infinite loop, and there is no WWDG reset.

LED7 is turned OFF if any error occurs.

@note This example must be tested in standalone mode (not in debug).


@note Care must be taken when using HAL_Delay(), this function provides accurate
      delay (in milliseconds) based on variable incremented in SysTick ISR. This
      implies that if HAL_Delay() is called from a peripheral ISR process, then 
      the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 

  - CoproSync/CoproSync_ShutDown/Inc/stm32mp1xx_hal_conf.h      HAL configuration file
  - CoproSync/CoproSync_ShutDown/Inc/stm32mp15xx_discol_conf.h  Board configuration file
  - CoproSync/CoproSync_ShutDown/Inc/stm32mp1xx_it.h            Interrupt handlers header file
  - CoproSync/CoproSync_ShutDown/Inc/main.h                     Header for main.c module
  - CoproSync/CoproSync_ShutDown/Inc/copro_sync.h               CoproSync header file
  - CoproSync/CoproSync_ShutDown/Inc/lock_resource.h            Header file for lock_resource.c
  - CoproSync/CoproSync_ShutDown/Inc/stm32mp1xx_it.c            Interrupt handlers
  - CoproSync/CoproSync_ShutDown/Inc/main.c                     Main program
  - CoproSync/CoproSync_ShutDown/Inc/stm32mp1xx_hal_msp.c       HAL MSP file
  - CoproSync/CoproSync_ShutDown/Inc/system_stm32mp1xx.c        STM32MP1xx system source file
  - CoproSync/CoproSync_ShutDown/Inc/copro_sync.c               Services to handle synchron between the processors
  - CoproSync/CoproSync_ShutDown/Inc/lock_resource.c            lock_resource file


@par Hardware and Software environment

  - This example runs on STM32MP157CACx devices.
    
  - This example has been tested with STM32MP157C-DK2 board and can be
    easily tailored to any other supported device and development board.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
 
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 