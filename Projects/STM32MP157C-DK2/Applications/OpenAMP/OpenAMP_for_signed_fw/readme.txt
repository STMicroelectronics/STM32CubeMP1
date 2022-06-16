/**
  @page OpenAMP_for_signed_fw OpenAMP TTY echo example

  @verbatim
  ******************************************************************************
  * @file    OpenAMP/OpenAMP_for_signed_fw/readme.txt
  * @author  MCD Application Team
  * @brief   Description of the OpenAMP TTY echo example.
  ******************************************************************************
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  @endverbatim

@par Application Description

How to use OpenAMP MW + Virtual UART to create an Inter-Processor Communication channel seen as TTY device in Linux OS
for protected firmware loaded by OP-TEE secure application.

This example deals with CPU2 (Cortex-M4) firmware and requires Linux OS running on CPU1 (Cortex-A7)

This example is a fork of the OpenAMP_TTY_echo example, with a realocation of  the resource_table and the
system_log_buf variables in MCU SRAM 3 at address 0x10048000.
Indeed when the load of the signed firmware is managed by OP-TEE, the RETRAM and The MCUSRAM 1 & 2 are not accessible
by the Linux. Only the MCU SRAM 3 is shared between the Cortex-M4 and the Cortex-A7

To work properly with the Linux kernel, the Linux device tree might need to be adapted as described in 
the wiki: https://wiki.st.com/stm32mpu/wiki/How_to_protect_the_coprocessor_firmware page for details.

OpenAMP MW uses the following HW resources
    * IPCC peripheral for event signal (mailbox) between CPU1(Cortex-A7) and CPU2(Cortex-M4)
    * MCUSRAM peripheral for buffer communications (virtio buffers) between CPU1(Cortex-A7) and CPU2(Cortex-M4)
            Reserved shared memeory region for this example: SHM_ADDR=0x10040000 and SHM_SIZE=128k.
            It is defined in platform_info.c file

In this example:
    - CPU2(Cortex-M4) turns on the LED7. the LED7 is fixed ON until OpenAMP middleware is initialized.
    - CPU2(Cortex-M4) initializes OPenAMP MW which initializes/configures IPCC peripheral through HAL.
    and setup openamp-rpmsg framwork infrastructure.
    - CPU2(Cortex-M4) creates 2 RPMsg channels for 2 virtual UART instance UART0 and UART1.
    - LED7 toggles when the RPMsg communication is initialised.
    - CPU2(Cortex-M4) is waiting for messages from CPU1(Cortex-A7) on these both channels.
    - When CPU2(Cortex-M4) receives a message on 1 Virtual UART instance/rpmsg channel
      it sends the message back to CPU1(Cortex-A7) on the same Virtual UART instance.
    - LED7 is turned OFF if any error occurs.

    Notes:
    - It requires Linux console to run example.
    - Cortex-M4 logging is redirected in Shared memory in MCUSRAM and can be displayed in Linux console for verdict
      using following command:
          cat /sys/kernel/debug/remoteproc/remoteproc0/trace0

    Following command should be done in Linux console on Cortex-A7 to run the example :

    > stty -onlcr -echo -F /dev/ttyRPMSG0
    > cat /dev/ttyRPMSG0 &
    > stty -onlcr -echo -F /dev/ttyRPMSG1
    > cat /dev/ttyRPMSG1 &
    > echo "Hello Virtual UART0" >/dev/ttyRPMSG0
    > echo "Hello Virtual UART1" >/dev/ttyRPMSG1

    You should get "Hello Virtual UART0" and "Hello Virtual UART1" in Linux console

@note Care must be taken when using HAL_Delay(), this function provides accurate
      delay (in milliseconds) based on variable incremented in HAL time base ISR.
      This implies that if HAL_Delay() is called from a peripheral ISR process, then
      the HAL time base interrupt must have higher priority (numerically lower) than
      the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the HAL time base interrupt priority you have to use HAL_NVIC_SetPriority()
      function.

@note The application needs to ensure that the HAL time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents
    - OpenAMP/OpenAMP_for_signed_fw/Inc/main.h                             Main program header file
    - OpenAMP/OpenAMP_for_signed_fw/Inc/mbox_ipcc.h                        mailbox_ipcc_if.c MiddleWare configuration header file
    - OpenAMP/OpenAMP_for_signed_fw/Inc/openamp.h                          User OpenAMP init header file
    - OpenAMP/OpenAMP_for_signed_fw/Inc/openamp_conf.h                     Configuration file for OpenAMP MW
    - OpenAMP/OpenAMP_for_signed_fw/Inc/rsc_table.h                        Resource_table for OpenAMP header file
    - OpenAMP/OpenAMP_for_signed_fw/Inc/stm32mp1xx_hal_conf.h         HAL Library Configuration file
    - OpenAMP/OpenAMP_for_signed_fw/Inc/stm32mp1xx_it.h               Interrupt handlers header file
    - OpenAMP/OpenAMP_for_signed_fw/Src/main.c                             Main program
    - OpenAMP/OpenAMP_for_signed_fw/Src/mbox_ipcc.c                        mailbox_ipcc_if.c MiddleWare configuration
    - OpenAMP/OpenAMP_for_signed_fw/Src/openamp.c                          User OpenAMP init
    - OpenAMP/OpenAMP_for_signed_fw/Src/rsc_table.c                        Resource_table for OpenAMP
    - OpenAMP/OpenAMP_for_signed_fw/Src/stm32mp1xx_it.c               Interrupt handlers
    - OpenAMP/OpenAMP_for_signed_fw/Src/system_stm32mp1xx.c           STM32MP1xx system clock configuration file


@par Hardware and Software environment

  - This example runs on STM32MP157xAAx devices.

  - This example has been tested with STM32MP157C-DK2 board and can be
    easily tailored to any other supported device and development board.

@par How to use it ?

In order to make the program work, you must do the following:
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
