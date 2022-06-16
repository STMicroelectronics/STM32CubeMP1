/**
  @page LPTIM_PWM_LSE LPTIM PWM Low Power example with LSE clock source

  @verbatim
  ******************** (C) COPYRIGHT 2021 STMicroelectronics *******************
  * @file    LPTIM/LPTIM_PWM_LSE/readme.txt
  * @author  MCD Application Team
  * @brief   Description of the LPTIM PWM LSE example
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

This example describes how to configure and use LPTIM to generate a PWM in low power mode
using the LSE as a counter clock, through the HAL LPTIM API.

In this example, the LPTIM instance used is LPTIM5 and the low power mode is Stop mode.


The counter clock is LSE (32.768 KHz), Autoreload equal to 99 so the output
frequency (FrequencyOutput) will be equal to 327.680.

  FrequencyOutput = Counter Clock Frequency / (Autoreload + 1)
                  = 32768 / 100
                  = 327.680 Hz

Pulse value equal to 49 and the duty cycle (DutyCycle) is computed as follow:

  DutyCycle = 1 - ((PulseValue + 1)/ (Autoreload + 1))
  DutyCycle = 50%

To minimize the power consumption, after starting generating the PWM signal,
the MCU enters in Stop mode. Note that GPIOs are configured in Low Speed to
enhance the consumption.

User push-button pin (PA.14)is configured as input with external interrupt (EXTI14),
falling edge. When User push-button is pressed, wakeup event is generated and PWM signal
generation is stopped.

@note This example can not be used in DEBUG mode, this is due to the fact
      that the Cortex-M4 core is no longer clocked during low power mode
      so debugging features are disabled.

@note Care must be taken when using HAL_Delay(), this function provides accurate
      delay (in milliseconds) based on variable incremented in SysTick ISR. This
      implies that if HAL_Delay() is called from a peripheral ISR process, then
      the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note This example needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents

  - LPTIM/LPTIM_PWM_LSE/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - LPTIM/LPTIM_PWM_LSE/Inc/stm32mp1xx_it.h          Interrupt handlers header file
  - LPTIM/LPTIM_PWM_LSE/Inc/main.h                   Header for main.c module
  - LPTIM/LPTIM_PWM_LSE/Src/stm32mp1xx_it.c          Interrupt handlers
  - LPTIM/LPTIM_PWM_LSE/Src/main.c                   Main program
  - LPTIM/LPTIM_PWM_LSE/Src/stm32mp1xx_hal_msp.c     HAL MSP module
  - LPTIM/LPTIM_PWM_LSE/Src/system_stm32mp1xx.c      STM32MP1xx system source file


@par Hardware and Software environment

  - This example runs on STM32MP157CAAx devices.

  - This example has been tested with STMicroelectronics STM32MP157C-EV1
    board and can be easily tailored to any other supported device
    and development board.

  - Connect LPTIM5_OUT pin PA3 (pin 31 in CN21 connector) to an oscilloscope to monitor
    the generated PWM signal (327.680 Hz)

  - LSE oscillator must be mounted on the board, else this example does not work.

@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example
