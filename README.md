Nucleo STMicroelectronics F411RE Board

Project: Magnetic Core Memory: Principles, Construction and Utilizzation
	     in an Open Source Environmemt by Using "STM32 MCUs"

Libraries version: V1.6.0 of STM32 Cube F4

Copyright (C) 2015 Onewards Nicola Didomenico (nicola.didomenico@gmail.com)
Copyright (C) 2015 Onewards Salvatore Del Popolo (popolo@tin.it)

Software Requirements:
A) Ubuntu Mate 14.04.3 LTS (Trusty Tahr) for Intel X86;
B) GNU Toolchain ARM Cortex-M/R (https://launchpad.net/gcc-arm-embedded);
C) OpenOCD (http://openocd.org/) or ST-Link (https://github.com/texane/stlink/);
D) Minicom/Cutecom

The project has been testeed under the following environment:
1) Distribution: Ubuntu Mate 14.04.3 LTS (Trusty Tahr), for Intel X86, Kernel Version: 3.16.0-46-generic
2) GCC Version: 4.9.3 20150529 (Release) [ARM /embedded-4_9-branch revision 227977]
(GNU Tools for ARM Embedded Processors)
3) Open On-Chip Debugger 0.10.0-dev-00036-g48787e1
4) Cutecom Version 0.22.0-2

How to run the project:
1. Create in the user home directory the hidden file .hdbinit, with the following line:
set auto-load safe-path /home/user_home_directory

2. open a terminal and download in the user home directory the following
   template typing:
git clone https://github.com/gnunicky/projct_rai_nucleo_stm32f411re_hal.git

3. Move to the directory magnetic_core_memory_hal:
cd project_rai_nucleo_stm32f411re_hal/magnetic_core_memory_hal/

4. Create an additional, hidden file called .gdbinit, required from
   OpenoCD and GDB, with the following lines:
layout split
target extended localhost:3333
monitor arm semihosting enable
monitor reset halt
load
monitor reset init

5. Clean and compile the project with following commands:
- make clean
- make

6. Plug the Nucleo STM32F411RE, with the Driver Shield and the IBM DJB 373330,
   already connected to the board and type the following command:
- make flash_openocd OR make flash_stlink

7. Start the OpenOCD server with following command:
- make openocd

8. Open a terminal in a new consolle and start GDB with following command:
- make debug

9. Start a serial communication program, like minicom/cutecom, setting
   following parameters:
Device=/dev/ttyACM0
Baud Rate: 115200
Parity: None
Data Bits: 8
Stop Bits: 1

10. Come back to the screen displaying the debugger interface and insert,
	one at a time, following commands:
- run
- o
- r

11. Now, from the screen containing the serial communication program, you can
	interact with the software managing the Magnetic Core Memory.
