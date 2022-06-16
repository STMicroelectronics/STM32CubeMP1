/**
  @page TIM_PWMOutput TIM PWM Output example

  @verbatim
  ******************** (C) COPYRIGHT 2019 STMicroelectronics *******************
  * @file    TIM/TIM_PWMOutput/readme.txt
  * @author  MCD Application Team
  * @brief   Description of the PWM signals generation using TIM8
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

This example shows how to configure the TIM peripheral in PWM (Pulse Width Modulation)
mode.

SystemCoreClock is set to 209 MHz for STM32MP1xx Devices.

    In this example TIM8 input clock (TIM8CLK) is set to 2 * APB2 clock (PCLK2),
    since APB2 prescaler is equal to 0.
      TIM8CLK = 2 * PCLK2
      PCLK2 = HCLK / 2
      => TIM8CLK = HCLK = SystemCoreClock

    To get TIM8 counter clock at 16 MHz, the prescaler is computed as follows:
       Prescaler = (TIM8CLK / TIM8 counter clock) - 1
       Prescaler = ((SystemCoreClock) /16 MHz) - 1

    To get TIM8 output clock at 24 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM8 counter clock / TIM8 output clock) - 1
           = 665

    TIM8 Channel1 duty cycle = (TIM8_CCR1/ TIM8_ARR + 1)* 100 = 50%
    TIM8 Channel2 duty cycle = (TIM8_CCR2/ TIM8_ARR + 1)* 100 = 37.5%
    TIM8 Channel3 duty cycle = (TIM8_CCR3/ TIM8_ARR + 1)* 100 = 25%
    TIM8 Channel4 duty cycle = (TIM8_CCR4/ TIM8_ARR + 1)* 100 = 12.5%


The PWM waveforms can be displayed using an oscilloscope.

@note The duty cycles values mentioned above are theoretical (obtained when the system clock frequency is exactly 209 MHz).
      They might be slightly different depending on system clock frequency precision.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note This example needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents

  - TIM/TIM_PWMOutput/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - TIM/TIM_PWMOutput/Inc/stm32mp1xx_it.h          Interrupt handlers header file
  - TIM/TIM_PWMOutput/Inc/main.h                  Header for main.c module
  - TIM/TIM_PWMOutput/Src/stm32mp1xx_it.c          Interrupt handlers
  - TIM/TIM_PWMOutput/Src/main.c                  Main program
  - TIM/TIM_PWMOutput/Src/stm32mp1xx_hal_msp.c     HAL MSP file
  - TIM/TIM_PWMOutput/Src/system_stm32mp1xx.c      STM32MP1xx system source file


@par Hardware and Software environment

  - This example runs on STM32MP157CAAx devices.
  - In this example, the clock is set to 209 MHz.

  - This example has been tested with STMicroelectronics STM32MP157C-EV1
    board and can be easily tailored to any other supported device
    and development board.

  - STM32MP157C-EV1 Set-up
    - Use MB1262C motherboard and make sure solder bridges SB63, SB66 and SB73 are replaced by 0 ohm resitor.
    - Connect the following pins to an oscilloscope to monitor the different waveforms:
          - TIM8_CH1 : PI.05 (pin 12 in CN21 connector)
          - TIM8_CH2 : PI.06 (pin 38 in CN21 connector)
          - TIM8_CH3 : PI.07 (pin 35 in CN21 connector)
          - TIM8_CH4 : PI.02 (pin 33 in CN21 connector)


@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
