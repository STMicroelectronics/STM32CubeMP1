/**
  @page UART_TwoBoards_ComDMA UART Two Boards Communication DMA example

  @verbatim
  ******************** (C) COPYRIGHT 2019 STMicroelectronics *******************
  * @file    UART/UART_TwoBoards_ComDMA/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the UART Two Boards Communication DMA example.
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

UART transmission (transmit/receive) in DMA mode 
between two boards.

Board: STM32MP157C-DK2 (embeds a STM32MP157CACx device)
Tx Pin: PE8 (CN14, pin 2)
Rx Pin: PE7 (CN14, pin 1)
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
  |                      GND|_____________________|GND                      |
  |_STM32_Board 1___________|                     |_STM32_Board 2___________|



Two identical boards are connected as shown on the picture above.
Board 1: transmitting then receiving board
Board 2: receiving then transmitting board

The user presses the USER1 push-button on board 1.
Then, board 1 sends in DMA mode a message to board 2 that sends it back to 
board 1 in DMA mode as well.
Finally, board 1 and 2 compare the received message to that sent.
If the messages are the same, the test passes.


WARNING: as both boards do not behave the same way, "TRANSMITTER_BOARD" compilation
switch is defined in /Src/main.c and must be enabled
at compilation time before loading the executable in the board that first transmits
then receives.
The receiving then transmitting board needs to be loaded with an executable
software obtained with TRANSMITTER_BOARD disabled. 

STM32MP157C-DK2 board LED is used to monitor the transfer status:
- While board 1 is waiting for the user to press the USER1 push-button, its LED7 is
  blinking rapidly (100 ms period).
- While board 2 is waiting for the message from board 1, its LED7 is emitting
  a couple of flashes every half-second.
- When the test passes, LED7 on both boards is turned on, otherwise the test has failed. 
- If there is an initialization or transfer error, LED7 is slowly blinking (1 sec. period).

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

  - UART/UART_TwoBoards_ComDMA/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - UART/UART_TwoBoards_ComDMA/Inc/stm32mp1xx_it.h          DMA interrupt handlers header file
  - UART/UART_TwoBoards_ComDMA/Inc/main.h                  Header for main.c module  
  - UART/UART_TwoBoards_ComDMA/Src/stm32mp1xx_it.c          DMA interrupt handlers
  - UART/UART_TwoBoards_ComDMA/Src/main.c                  Main program
  - UART/UART_TwoBoards_ComDMA/Src/stm32mp1xx_hal_msp.c     HAL MSP module
  - UART/UART_TwoBoards_ComDMA/Src/system_stm32mp1xx.c      STM32MP1xx system source file


@par Hardware and Software environment

  - This example runs on STM32MP157CACx devices.    
  - This example has been tested with two STM32MP157C-DK2 boards embedding
    a STM32MP157CACx device and can be easily tailored to any other supported device 
    and development board.

  - STM32MP157C-DK2 set-up
    - Connect a wire between 1st board PE8 pin (Uart Tx) and 2nd board PE7 pin (Uart Rx)
    - Connect a wire between 1st board PE7 pin (Uart Rx) and 2nd board PE8 pin (Uart Tx)
    - Connect 1st board GND to 2nd Board GND    

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
