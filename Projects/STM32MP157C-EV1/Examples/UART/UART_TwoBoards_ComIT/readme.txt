/**
  @page UART_TwoBoards_ComIT UART Two Boards Communication IT example

  @verbatim
  ******************** (C) COPYRIGHT 2019 STMicroelectronics *******************
  * @file    UART/UART_TwoBoards_ComIT/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the UART Two Boards Communication IT example.
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

UART transmission (transmit/receive) in Interrupt mode 
between two boards.

Board: STM32MP157C-EV1 (embeds a STM32MP157CAAx device)
Tx Pin: PB10 (CN21, pin 8)
Rx Pin: PB12 (CN21, pin 10)
   _________________________                       _________________________ 
  |           ______________|                     |______________           |
  |          |USART         |                     |         USART|          |
  |          |              |                     |              |          |
  |          |           TX |_____________________| RX           |          |
  |          |              |                     |              |          |
  |          |              |                     |              |          |
  |          |              |                     |              |          |
  |          |           RX |_____________________| TX           |          |
  |          |              |                     |              |          |
  |          |______________|                     |______________|          |
  |                         |                     |                         |
  |                         |                     |                         |
  |                      GND|_____________________|GND                      |
  |_STM32_Board 1___________|                     |_STM32_Board 2___________|



Two identical boards are connected as shown on the picture above.
Board 1: transmitting then receiving board
Board 2: receiving then transmitting board

The user presses the User push-button on board 1.
Then, board 1 sends in interrupt mode a message to board 2 that sends it back to 
board 1 in interrupt mode as well.
Finally, board 1 and 2 compare the received message to that sent.
If the messages are the same, the test passes.


WARNING: as both boards do not behave the same way, "TRANSMITTER_BOARD" compilation
switch is defined in /Src/main.c and must be enabled
at compilation time before loading the executable in the board that first transmits
then receives.
The receiving then transmitting board needs to be loaded with an executable
software obtained with TRANSMITTER_BOARD disabled. 

STM32MP157C-EV1 board LED is used to monitor the transfer status:
- While board 1 is waiting for the user to press the User PA14 push-button, its LED4 is
  blinking rapidly (100 ms period).
- While board 2 is waiting for the message from board 1, its LED4 is emitting
  a couple of flashes every half-second.
- When the test passes, LED4 on both boards is turned on, otherwise the test has failed. 
- If there is an initialization or transfer error, LED4 is slowly blinking (1 sec. period).

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the systick.
Then the System clock source is configured by the SystemClock_Config() function in case of Engineering Mode, this clock configuration is done
by the Firmware running on the Cortex-A7 in case of Production Mode.


The UART is configured as follows:
    - BaudRate = 9600 baud  
    - Word Length = 8 bits (8 data bits, no parity bit)
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Reception and transmission are enabled in the time

@note USARTx/UARTx instance used and associated resources can be updated in "main.h"
file depending hardware configuration used.

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

  - UART/UART_TwoBoards_ComIT/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - UART/UART_TwoBoards_ComIT/Inc/stm32mp1xx_it.h          IT interrupt handlers header file
  - UART/UART_TwoBoards_ComIT/Inc/main.h                  Header for main.c module  
  - UART/UART_TwoBoards_ComIT/Src/stm32mp1xx_it.c          IT interrupt handlers
  - UART/UART_TwoBoards_ComIT/Src/main.c                  Main program
  - UART/UART_TwoBoards_ComIT/Src/stm32mp1xx_hal_msp.c     HAL MSP module
  - UART/UART_TwoBoards_ComIT/Src/system_stm32mp1xx.c      STM32MP1xx system source file


@par Hardware and Software environment 

  - This example runs on STM32MP157CAAx devices.    
  - This example has been tested with two STM32MP157C-EV1 boards embedding
    a STM32MP157CAAx device and can be easily tailored to any other supported device 
    and development board.

  - STM32MP157C-EV1 set-up
    - Connect STM32_Board 1 PB10 (CN21, pin 8) to STM32_Board 2 PB12 (CN21, pin 10)
    - Connect STM32_Board 1 PB12 (CN21, pin 10) to STM32_Board 2 PB10 (CN21, pin 8)
    - Connect STM32_Board 1 GND (CN21, pin 39) to STM32_Board 2 GND (CN21, pin 39)

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
