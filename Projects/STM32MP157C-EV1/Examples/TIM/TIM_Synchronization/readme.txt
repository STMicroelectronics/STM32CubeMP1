/**
  @page TIM_Synchronization Timers Synchronization example

  @verbatim
  ******************** (C) COPYRIGHT 2021 STMicroelectronics *******************
  * @file    TIM/TIM_Synchronization/readme.txt
  * @author  MCD Application Team
  * @brief   How to command 2 Timers as slaves (TIM8 & TIM1) using a Timer
  *          as master (TIM2)
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

This example shows how to synchronize TIM2 and Timers (TIM8 and TIM1) in parallel mode.

Timers synchronisation in parallel mode:


                                                       ___________
                                                 ITR1 |  SLAVE 1  |
                                     _________________|    TIM8   |
   ___________                      |                 |___________|
  |   MASTER  |TRGO_Update          |
  |    TIM2   |---------------------|
  |___________|                     |                  ___________
                                    |_________________|  SLAVE 2  |
                                                 ITR1 |    TIM1   |
                                                      |___________|


1/ TIM2 is configured as Master Timer:
   - PWM Mode is used
   - The TIM2 Update event is used as Trigger Output

2/ TIM8 and TIM1 are slaves for TIM2,
   - PWM Mode is used
   - The ITR1(TIM2) is used as input trigger for both slaves
   - Gated mode is used, so starts and stops of slaves counters
     are controlled by the Master trigger output signal(update event).

  In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1),
  since APB1 prescaler is different from 1.
      TIM2CLK = 2 * PCLK1
      PCLK1 = HCLK / 2
      => TIM2CLK = HCLK = SystemCoreClock

  The TIM2 counter clock is equal to SystemCoreClock = 209 MHz.

  The Master Timer TIM2 is running at:
  TIM2 frequency = TIM2 counter clock / (TIM2_Period + 1) = 816,4 KHz and
  a duty cycle equal to: TIM2_CCR1/(TIM2_ARR + 1) = 25%
  (TIM2_ARR =  255)

  The TIM8 is running at:
  TIM8 frequency = (TIM2 frequency)/ (TIM8 period +1) = 81,6 KHz and
  a duty cycle equal to TIM8_CCR1/(TIM8_ARR + 1) = 30%
  (TIM8_ARR =  9)

  The TIM1 is running at:
  TIM1 frequency = (TIM2 frequency)/ (TIM1 period +1) = 163,3 KHz and
  a duty cycle equal to TIM1_CCR1/(TIM1_ARR + 1) = 60%
  (TIM1_ARR =  4)


The PWM waveform can be displayed using an oscilloscope.


@par Directory contents

  - TIM/TIM_ParallelSynchro/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - TIM/TIM_ParallelSynchro/Inc/stm32mp1xx_it.h          Interrupt handlers header file
  - TIM/TIM_ParallelSynchro/Inc/main.h                  Header for main.c module
  - TIM/TIM_ParallelSynchro/Src/stm32mp1xx_it.c          Interrupt handlers
  - TIM/TIM_ParallelSynchro/Src/main.c                  Main program
  - TIM/TIM_ParallelSynchro/Src/stm32mp1xx_hal_msp.c     HAL MSP file
  - TIM/TIM_ParallelSynchro/Src/system_stm32mp1xx.c      STM32MP1xx system source file


@par Hardware and Software environment

  - This example runs on STM32MP157CAAx devices.

  - This example has been tested with STMicroelectronics STM32MP157C-EV1
    board and can be easily tailored to any other supported device
    and development board.

  - STM32MP157C-EV1 Set-up
     - Use MB1262C motherboard and make sure solder bridges SB23 and SB52 are replaced by 0 ohm resitor.
     - Connect the following pins to an oscilloscope to monitor the different waveforms:
          - TIM2 CH1 (PG8) (pin 11 in CN21 connector)
          - TIM8 CH4 (PI2) (pin 33 in CN21 connector)
          - TIM1 CH4 (PA11) (pin 5 in CN21 connector)


@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files: Project->Rebuild all
 - Load project image: Project->Download and Debug
 - Run program: Debug->Go(F5)

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
