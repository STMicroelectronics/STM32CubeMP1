/**
  @page ADC_MultiChannelSingleConversion ADC example

  @verbatim
  ******************************************************************************
  * @file    Examples/ADC/ADC_MultiChannelSingleConversion/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the ADC_MultiChannelSingleConversion example.
  ******************************************************************************
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @endverbatim

@par Example Description

How to use an ADC peripheral to convert several channels. ADC conversions are 
performed successively in a scan sequence.
This example is based on the STM32MP1xx ADC HAL API.

Example configuration:
ADC is configured in single conversion mode, from SW trigger.
Sequencer of ADC group regular (default group available on ADC of all STM32 devices)
is configured to convert 3 channels: 1 channel from GPIO, 2 internal channels:
internal voltage reference VrefInt and temperature sensor.
DMA is configured to transfer conversion data in an array of size three elements
(one array address for conversion data of each channel) in RAM memory,
in circular mode.

Example execution:
Every second, ADC performs conversion of a channel among the 3 channels
of the scan sequence, successively at each conversion start (discontinuous mode enabled).

Note: In case of discontinuous mode is disabled, then the entire scan sequence
      is converted in burst from one conversion start.

ADC conversion data of each sequence rank is transferred by DMA into the array
"aADCxConvertedData":
- aADCxConvertedData[0]: ADC channel set on sequence rank 1 (GPIO as analog input)
- aADCxConvertedData[1]: ADC channel set on sequence rank 2 (VrefInt)
- aADCxConvertedData[2]: ADC channel set on sequence rank 3 (Temperature sensor)

When sequence is completed, it restarts from the beginning: first channel 
in the sequence, data transfer in first array address (rollback).

ADC conversions raw data are computed to physical values
using LL ADC driver helper macro:
- Value of analog reference voltage (Vref+), connected to analog voltage supply Vdda (unit: mV)
- Value of voltage on GPIO pin (on which is mapped ADC channel, cf pin below) (unit: mV)
- Value of internal voltage reference VrefInt (unit: mV)
- Value of temperature (unit: degree Celsius)

Note: For this example purpose, analog reference voltage (Vref+) is computed
      from ADC conversion of internal voltage reference VrefInt and used
      to compute other conversion data.
      This voltage should correspond to value of literal "VDDA_APPLI".
      This procedure can be performed when value of voltage Vref+
      is unknown in the application.
      (This is not the case in this example due to target board
      supplied by a LDO regulator providing a known constant voltage
      of value "VDDA_APPLI").
      In typical case of Vref+ connected to Vdd, it allows to
      deduce Vdd value.

LED4 is used to monitor program execution status:
- Normal operation: Activity of ADC scan sequence
  can be observed with LED toggle:
  - At each ADC conversion: LED toggle once (every 1sec)
  - At each scan conversion completed: LED toggle 4 times quickly (10Hz)
- Error: LED remaining turned on

Debug: variables to monitor with debugger watch window:
- "ubDmaTransferStatus": status of DMA transfer of ADC group regular conversions
- "uhADCxConvertedData_VrefAnalog_mVolt":         Value of analog reference voltage (Vref+), connected to analog voltage supply Vdda (unit: mV)
- "uhADCxConvertedData": ADC group regular conversion data
- "uhADCxConvertedData_VoltageGPIO_mVolt":        Value of voltage on GPIO pin (on which is mapped ADC channel) (unit: mV)
- "uhADCxConvertedData_VrefInt_mVolt":            Value of internal voltage reference VrefInt (unit: mV)
- "hADCxConvertedData_Temperature_DegreeCelsius": Value of temperature (unit: degree Celsius)

Connection needed:
None, if ADC channel and DAC channel are selected on the same GPIO.
Otherwise, connect a wire between DAC channel output and ADC input.

Other peripherals used:
  1 GPIO for LED4
  1 GPIO for analog input: PA4 (pin 1 on JP11)
  1 GPIO for DAC channel output PA4 (PA4)
  1 DMA channel

Board settings:
 - ADC is configured to convert ADC2_CHANNEL_16 (pin 1 on JP11).
 - The voltage input on ADC channel is provided from DAC (DAC_CHANNEL_1).
   ADC input from pin PA4 and DAC ouput to pin PA4:
   If same pin is used no connection is required, it is done internally. Otherwise, user need to connect a wire between pin 1 on JP11 and PA4
 - Voltage value should be between 0 and 2.9V.

To observe voltage level applied on ADC channel through GPIO, connect a voltmeter on
pin PA4 (pin 1 on JP11).

@par Keywords

ADC, analog digital converter, analog, conversion, voltage, channel, analog input, DMA transfer, sequence, temperature sensor, internal voltage reference, VrefInt, discontinuous

@par Directory contents 

  - ADC/ADC_MultiChannelSingleConversion/Inc/stm32mp15xx_eval_conf.h  BSP configuration file
  - ADC/ADC_MultiChannelSingleConversion/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - ADC/ADC_MultiChannelSingleConversion/Inc/stm32mp1xx_it.h          Interrupt handlers header file
  - ADC/ADC_MultiChannelSingleConversion/Inc/main.h                   Header for main.c module
  - ADC/ADC_MultiChannelSingleConversion/Src/stm32mp1xx_it.c          Interrupt handlers
  - ADC/ADC_MultiChannelSingleConversion/Src/stm32mp1xx_hal_msp.c     HAL MSP module
  - ADC/ADC_MultiChannelSingleConversion/Src/main.c                   Main program
  - ADC/ADC_MultiChannelSingleConversion/Src/system_stm32mp1xx.c      STM32MP1xx system source file


@par Hardware and Software environment

  - This example runs on STM32MP157CAAx devices.
    
  - This example has been tested with STM32MP157C-EV1 board and can be
    easily tailored to any other supported device and development board.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
