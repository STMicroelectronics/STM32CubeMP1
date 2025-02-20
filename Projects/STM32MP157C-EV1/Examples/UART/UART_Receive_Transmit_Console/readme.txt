/**
  @page UART_TwoBoards_ComIT UART Two Boards Communication IT example

  @verbatim
  ******************** (C) COPYRIGHT 2021 STMicroelectronics *******************
  * @file    UART/UART_Receive_Transmit_Console/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the UART Receive and Transmit on Console example.
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

UART transmission (printf/getchar) via console with user interaction.
The UART outputs a message on the HyperTerminal and receives some characters.

Board: STM32MP157C-EV1 (embeds a STM32MP157CAAx device)
Tx Pin: PB10 (CN21, pin 8)
Rx Pin: PB12 (CN21, pin 10)
   _________________________
  |           ______________|                      ________________
  |          |USART3        |                     | Hyperterminal  |
  |          |              |                     |                |
  |          |           TX |_____________________| TXD            |
  |          |              |                     |                |
  |          |              |                     |                |
  |          |              |       RS232 Cable   |                |
  |          |           RX |_____________________| RXD            |
  |          |              |                     |                |
  |          |______________|                     |                |
  |                     3V3 |_____________________| VCC            |
  |                         |                     |                |
  |                     GND |_____________________| GND            |
  |_STM32_Board 1___________|                     |________________|


The USART is configured as follows:
    - BaudRate = 115200 baud
    - Word Length = 8 Bits (7 data bit + 1 parity bit)
    - One Stop Bit
    - Odd parity
    - Hardware flow control disabled (RTS and CTS signals: not usefull for this example) 
    - Reception and transmission are enabled in the time

The user sees some printf messages on the Hyperterminal console.
He has to type "c" to continue with getchar test.
A menu is displayed, and the user can select:
- "Test1" to execute fast LED4 blink (during 5s)
- "Test2" to execute slow LED4 blink (during 5s)
- "Test3" to quit the example

STM32MP157C-EV1 board LED is used to monitor:
- LED4 is blinking fastly or slowly during Test1 and Test2.
- LED4 is on at the end of the test (if user selects "Test3")
- If there is an error, LED4 is blinking (2 sec. period).

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the systick.
Then the System clock source is configured by the SystemClock_Config() function in case of Engineering Mode, this clock configuration is done
by the Firmware running on the Cortex-A7 in case of Production Mode.

@note When the parity is enabled, the computed parity is inserted at the MSB
position of the transmitted data.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 

  - UART/UART_Receive_Transmit_Console/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - UART/UART_Receive_Transmit_Console/Inc/stm32mp1xx_it.h          IT interrupt handlers header file
  - UART/UART_Receive_Transmit_Console/Inc/main.h                   Header for main.c module  
  - UART/UART_Receive_Transmit_Console/Src/stm32mp1xx_it.c          IT interrupt handlers
  - UART/UART_Receive_Transmit_Console/Src/main.c                   Main program
  - UART/UART_Receive_Transmit_Console/Src/stm32mp1xx_hal_msp.c     HAL MSP module
  - UART/UART_Receive_Transmit_Console/Src/system_stm32mp1xx.c      STM32MP1xx system source file


@par Hardware and Software environment 

  - This example runs on STM32MP157CAAx devices.    
  - This example has been tested with two STM32MP157C-EV1 boards embedding
    a STM32MP157CAAx device and can be easily tailored to any other supported device 
    and development board.

  - STM32MP157C-EV1 set-up
    - Connect STM32_Board 1 PB10 (CN21, pin 8 ) to TXD RS232 cable
    - Connect STM32_Board 1 PB12 (CN21, pin 10) to RXD RS232 cable
    - Connect STM32_Board 1 GND  (CN21, pin 39) to GND RS232 cable
    - Connect STM32_Board 1 3V3  (CN21, pin 1 ) to VCC RS232 cable

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
