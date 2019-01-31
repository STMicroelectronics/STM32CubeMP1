/**
  @page TIM_DMABurst TIM_DMABurst example
  
  @verbatim
  ******************** (C) COPYRIGHT 2019 STMicroelectronics *******************
  * @file    TIM/TIM_DMABurst/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the TIM DMA Burst example.
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

This example shows how to update the TIMER TIM2_CH4 period and the duty cycle 
using the TIMER DMA burst feature.

Every update DMA request, the DMA will do 6 transfers of half words into Timer 
registers beginning from ARR register.
On the DMA update request, 0x0FFF will be transferred into ARR, 0x0000 will be transferred into RCR (if supported),
 0x0000 will be transferred into CCR1, 0x0000 will be transferred into CCR2,
 0x0000 will be transferred into CCR3, 0x0555 will be transferred into CCR4. 

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the systick.
Then the System clock source is configured by the SystemClock_Config() function in case of Engineering Mode, 
this clock configuration is done by the Firmware running on the Cortex-A7 in case of Production Mode.

This Peripheral clock frequency is get with RCC_PERIPHCLK_TIMG1.
To get TIM2 counter clock at 16 MHz the Prescaler is computed as following:
- Prescaler = (TIM2CLK / TIM2 counter clock) - 1

The TIM2 Frequency = TIM2 counter clock/(ARR + 1)
                   = 16 MHz / 4096 = 3.9 KHz

The TIM2 CCR4 register value is equal to 0x555, so the TIM2 Channel 4 generates a 
PWM signal with a frequency equal to 3.9 KHz and a duty cycle equal to 33.33%:
TIM2 Channel1 duty cycle = (TIM2_CCR4/ TIM2_ARR + 1)* 100 = 33.33%

The PWM waveform can be displayed using an oscilloscope.

@note Care must be taken when using HAL_Delay(), this function provides accurate
      delay (in milliseconds) based on variable incremented in SysTick ISR. This
      implies that if HAL_Delay() is called from a peripheral ISR process, then 
      the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note This example needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents  

  - TIM/TIM_DMABurst/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - TIM/TIM_DMABurst/Inc/stm32mp1xx_it.h          Interrupt handlers header file
  - TIM/TIM_DMABurst/Inc/main.h                  Header for main.c module  
  - TIM/TIM_DMABurst/Src/stm32mp1xx_it.c          Interrupt handlers
  - TIM/TIM_DMABurst/Src/main.c                  Main program
  - TIM/TIM_DMABurst/Src/stm32mp1xx_hal_msp.c     HAL MSP file
  - TIM/TIM_DMABurst/Src/system_stm32mp1xx.c      STM32MP1xx system source file

@par Hardware and Software environment

  - This example runs on STM32MP157CAAx devices.
    
  - This example has been tested with STMicroelectronics STM32MP157C-EV1 
    board and can be easily tailored to any other supported device 
    and development board.

  - STM32MP157C-EV1 Set-up
    - Connect the TIM2 output channel to an oscilloscope to monitor the different waveforms: 
    - TIM2 TIM_CHANNEL_4 (PA.03 (pin 31 in CN21 connector))

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
