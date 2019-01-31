/**
  @page DAC_SimpleConversion DAC Simple Conversion example
  
  @verbatim
  ******************** (C) COPYRIGHT 2019 STMicroelectronics *******************
  * @file    DAC/DAC_SimpleConversion/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the DAC Simple Conversion example.
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

How to use the DAC peripheral to do a simple conversion.

      - The example uses the DAC for a simple conversion in 8 bits right 
      	alignment of 0xFF value, the result of conversion can be seen by 
      	connecting PA4(DAC channel1) to an oscilloscope. 
      	The observed value is 3.3V.
      - The low power mode of DAC (sample and hold mode) can also be used while Cortex 
        is in sleep mode.
      - The tests steps are:
      - Step 0:
      	DAC:     normal power mode
      	Cortex:  run mode
      - Step 1:
      	DAC:     Low power mode
      	Cortex:  Sleep mode
      

STM32MP157C-EV1 board's LED can be used to monitor the process status:
  - LED4 is slowly blinking (1 sec. period) and example is stopped (using infinite loop)
  when there is an error during process.

@par Directory contents 

  - DAC/DAC_Simple_Conversion/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - DAC/DAC_Simple_Conversion/Inc/stm32mp1xx_it.h          DMA interrupt handlers header file
  - DAC/DAC_Simple_Conversion/Inc/main.h                  Header for main.c module  
  - DAC/DAC_Simple_Conversion/Src/stm32mp1xx_it.c          DMA interrupt handlers
  - DAC/DAC_Simple_Conversion/Src/main.c                  Main program
  - DAC/DAC_Simple_Conversion/Src/stm32mp1xx_hal_msp.c     HAL MSP file
  - DAC/DAC_Simple_Conversion/Src/system_stm32mp1xx.c      STM32MP1xx system source file
  

@par Hardware and Software environment  
  - This example runs on STM32MP157CAAx devices.
    
  - This example has been tested with STM32MP157C-EV1 board and can be
    easily tailored to any other supported device and development board.

  - STM32MP157C-EV1 Set-up 	
    
      - Connect PA4 (pin 1 on JP11) (DAC Channel1) to an oscilloscope.
      - Press User PA14 push-button to switch between steps.
      - You may redo the tests by changing the sample and hold parameters 
        of the DAC.          
      

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
  
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */

