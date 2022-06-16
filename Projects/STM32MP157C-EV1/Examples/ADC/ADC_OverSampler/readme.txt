/**
  @page ADC_OverSampler ADC OverSampler example

  @verbatim
  ******************** (C) COPYRIGHT 2021 STMicroelectronics *******************
  * @file    ADC/ADC_OverSampler/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the ADC Oversampler example.
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
  @endverbatim

@par Example Description 

This example describes how to configure and use the ADC to convert an external
analog input combined with oversampling feature to increase resolution through the HAL API.

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
The SystemClock_Config() function is used to configure the system clock for STM32MP1xx Devices :

The ADC is configured to convert continuously ADC_CHANNEL_16 (pin PA.04)
while the oversampler is enabled. 
Oversampling ratio is set to 1024 so the maximum output result 
is 0xFFF * 1024 = 0x3FFC00 (22 bits). Result is 6-bit right shift.

uwConvertedValue variable contains the 16-bit conversion result and 
uwInputVoltage yields the input voltage in mV. 

The converted value is monitored through debugger: uwConvertedValue and uwInputVoltage variables.

STM32 board LED can be used to monitor the conversion:
  - LED4 is ON when there is no error.
  - LED4 blinks when there is an error.


@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The example need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Keywords

Analog, ADC, Analog to Digital, oversampling, Continuous conversion, Measurement, Voltage

@par Directory contents 

  - ADC/ADC_OverSampler/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - ADC/ADC_OverSampler/Inc/stm32mp1xx_it.h          Interrupt handlers header file
  - ADC/ADC_OverSampler/Inc/main.h                   Header for main.c module  
  - ADC/ADC_OverSampler/Src/stm32mp1xx_it.c          Interrupt handlers
  - ADC/ADC_OverSampler/Src/main.c                   Main program
  - ADC/ADC_OverSampler/Src/stm32mp1xx_hal_msp.c     HAL MSP file 
  - ADC/ADC_OverSampler/Src/system_stm32mp1xx.c      STM32MP1xx system source file


@par Hardware and Software environment 

  - This example runs on STM32MP157CAAx devices.

  - This example has been tested with STM32MP157C-EV1 board and can be
    easily tailored to any other supported device and development board.

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example


 */
