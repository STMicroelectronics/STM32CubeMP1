/**
  @page OpenAMP_Dynamic_ResMgr OpenAMP Dynamic Resource Manager example
  
  @verbatim
  ******************** (C) COPYRIGHT 2019 STMicroelectronics *******************
  * @file    OpenAMP/OpenAMP_Dynamic_ResMgr/readme.txt
  * @author  MCD Application Team
  * @brief   Description of the OpenAMP Dynamic Resource Manager example.
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

How to use the DAC peripheral and its reference regulator to do a simple conversion.

      - The example uses the DAC for a simple conversion in 8 bits right 
      	alignment of 0xFF value, the result of conversion can be seen by 
      	connecting PA4(DAC channel1) to an oscilloscope. 
      	The observed value is 3.3V.
      - The regulator is a system resource which is controlled exclusively by
        the CA7. Using the ResourcesManager utility, the CM4 firmware asks CA7
        to disable /enable the input voltage reference regulator.
      - The low power mode of DAC (sample and hold mode) can also be used while
        CM4 is in sleep mode.

      - Pressing the User PA14 push-button, the firmware switches between the
        following test steps:
      - Step 0:
      	DAC:       normal power mode
      	CM4:       run mode
      	Regulator: enabled
      - Step 1:
      	DAC:       stopped
      	CM4:       run mode
      	Regulator: disabled
      - Step 2:
      	DAC:       low power mode
      	CM4:       sleep mode
      	Regulator: enabled

    The status (enabled / disabled) of the vdda regulator can be monitored by
    entering the following command in the Linux console on CA7 :

    > cat /sys/kernel/debug/regulator/regulator_summary
     regulator              use open bypass voltage current     min     max
     -----------------------------------------------------------------------
     ...
           vdda               x    1      0  2900mV     0mA  2900mV  2900mV
     ...

    'x', the value of 'use' defines whether the regulator is enabled (x >= 1)
    or disabled (x = 0, or vdda entry absent).

STM32MP157C-EV1 board's LED can be used to monitor the process status:
  - LED4 is slowly blinking (1 sec. period) and example is stopped (using infinite loop)
  when there is an error during process.

@par Directory contents 

  - OpenAMP/OpenAMP_Dynamic_ResMgr/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Inc/stm32mp1xx_it.h          DMA interrupt handlers header file
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Inc/main.h                   Header for main.c module
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Inc/mbox_ipcc.h              mailbox_ipcc_if.c MiddleWare configuration header file
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Inc/openamp.h                User OpenAMP init header file
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Inc/openamp_conf.h           Configuration file for OpenAMP MW
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Inc/res_mgr_conf.h           Resources Manager configuration file
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Inc/rsc_table.h              Resource_table for OpenAMP header file
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Src/stm32mp1xx_it.c          DMA interrupt handlers
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Src/main.c                   Main program
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Src/stm32mp1xx_hal_msp.c     HAL MSP file
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Src/system_stm32mp1xx.c      STM32MP1xx system source file
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Src/mbox_ipcc.c              mailbox_ipcc_if.c MiddleWare configuration
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Src/openamp.c                User OpenAMP init
  - OpenAMP/OpenAMP_Dynamic_ResMgr/Src/rsc_table.c              Resource_table for OpenAMP

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

