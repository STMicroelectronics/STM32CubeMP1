/**
  @page SPI_FullDuplex_ComDMA_Slave SPI Full Duplex DMA example

  @verbatim
  ******************** (C) COPYRIGHT 2021 STMicroelectronics *******************
  * @file    SPI/SPI_FullDuplex_ComDMA_Slave/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the SPI Full Duplex DMA example.
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

Data buffer transmission/reception between two boards via SPI using DMA.

Board: STM32MP157C-DK2 (embeds a STM32MP157CACx device)
CLK Pin: PE12 (CN13, pin D13)
MISO Pin: PE13 (CN13, pin D12)
MOSI Pin: PE14 (CN13, pin D11)
   _________________________                       __________________________
  |           ______________|                      |______________           |
  |          |SPI4          |                      |          SPI4|          |
  |          |              |                      |              |          |
  |          |    CLK(PE.12)|______________________|(PE.12)CLK    |          |
  |          |              |                      |              |          |
  |          |   MISO(PE.13)|______________________|(PE.13)MISO   |          |
  |          |              |                      |              |          |
  |          |   MOSI(PE.14)|______________________|(PE.14)MOSI   |          |
  |          |              |                      |              |          |
  |          |______________|                      |______________|          |
  |      __                 |                      |                         |
  |     |__|                |                      |                         |
  |    User push-button     |                      |                         |
  |                      GND|______________________|GND                      |
  |                         |                      |                         |
  |_STM32MP1xx Master_______|                      |_STM32MP1xx Slave________|

HAL architecture allows user to easily change code to move to Polling or IT 
mode. To see others communication modes please check following examples:
SPI\SPI_FullDuplex_ComPolling_Master and SPI\SPI_FullDuplex_ComPolling_Slave
SPI\SPI_FullDuplex_ComIT_Master and SPI\SPI_FullDuplex_ComIT_Slave

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the systick.
Then the System clock source is configured by the SystemClock_Config() function in case of Engineering Mode, this clock configuration is done
by the Firmware running on the Cortex-A7 in case of Production Mode.

The SPI peripheral configuration is ensured by the HAL_SPI_Init() function.
This later is calling the HAL_SPI_MspInit()function which core is implementing
the configuration of the needed SPI resources according to the used hardware (CLOCK, 
GPIO, DMA and NVIC). You may update this function to change SPI configuration.

The SPI communication is then initiated.
The HAL_SPI_TransmitReceive_DMA() function allows the reception and the 
transmission of a predefined data buffer at the same time (Full Duplex Mode). 
If the Master board is used, the project SPI_FullDuplex_ComDMA_Master must be used.
If the Slave board is used, the project SPI_FullDuplex_ComDMA_Slave must be used.

For this example the aTxBuffer is predefined and the aRxBuffer size is same as aTxBuffer.

In a first step after the user press the USER1 push-button, SPI Master starts the 
communication by sending aTxBuffer and receiving aRxBuffer through 
HAL_SPI_TransmitReceive_DMA(), at the same time SPI Slave transmits aTxBuffer 
and receives aRxBuffer through HAL_SPI_TransmitReceive_DMA(). 
The callback functions (HAL_SPI_TxRxCpltCallback and HAL_SPI_ErrorCallbackand) update 
the variable wTransferState used in the main function to check the transfer status.
Finally, aRxBuffer and aTxBuffer are compared through Buffercmp() in order to 
check buffers correctness.  

STM32 board's LEDs can be used to monitor the transfer status:
 - LED7 toggles quickly on master board waiting USER1 push-button to be pressed.
 - LED7 turns ON if transmission/reception is complete and OK.
 - LED7 toggles slowly when there is a timeout or an error in transmission/reception process.   

@note You need to perform a reset on Slave board, then perform it on Master board
      to have the correct behaviour of this example.
      
@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 

  - SPI/SPI_FullDuplex_ComDMA_Slave/Inc/stm32mp1xx_hal_conf.h   HAL configuration file
  - SPI/SPI_FullDuplex_ComDMA_Slave/Inc/stm32mp1xx_it.h         Interrupt handlers header file
  - SPI/SPI_FullDuplex_ComDMA_Slave/Inc/main.h                 Header for main.c module  
  - SPI/SPI_FullDuplex_ComDMA_Slave/Src/stm32mp1xx_it.c         Interrupt handlers
  - SPI/SPI_FullDuplex_ComDMA_Slave/Src/main.c                 Main program
  - SPI/SPI_FullDuplex_ComDMA_Slave/Src/system_stm32mp1xx.c     stm32mp1xx system source file
  - SPI/SPI_FullDuplex_ComDMA_Slave/Src/stm32mp1xx_hal_msp.c    HAL MSP file

@par Hardware and Software environment

  - This example runs on STM32MP157CACx devices.

  - This example has been tested with STM32MP157C-DK2 board and can be
    easily tailored to any other supported device and development board.

  - STM32MP157C-DK2 Set-up
    - Connect Master board PE12 (CN13, pin D13) to Slave Board PE12 (CN13, pin D13)
    - Connect Master board PE13 (CN13, pin D12) to Slave Board PE13 (CN13, pin D12)
    - Connect Master board PE14 (CN13, pin D11) to Slave Board PE14 (CN13, pin D11)
    - Connect Master board GND  to Slave Board GND

@par How to use it ? 

In order to make the program work, you must do the following:
 - Open your preferred toolchain 
 - Rebuild all files (master project) and load your image into target memory
    o Load the project in Master Board
 - Rebuild all files (slave project) and load your image into target memory
    o Load the project in Slave Board
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 