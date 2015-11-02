Nucleo StMicroelectronics F401/411RE board 

Project:"Magnetic Core Memory Principles, Construction and Utilizzation
         in an Open Source EnvironmenT by Using STM32 MCUs"

Version: "V1.6.0 of STM32 Cube F4"

Copyright (C) 2015 onwards Nicola Didomenico (nicola.didomenico@gmail.com)
Copyright (C) 2015 onwards Salvatore Del Popolo (popolo@tin.it) 

Requisiti software:
a) Distribuzione basata su Debian GNU/Linux; 
b) GNU Toolchain ARM Cortex-M/R (https://launchpad.net/gcc-arm-embedded);
c) OpenOCD (http://openocd.org/) oppure ST-Link (https://github.com/texane/stlink);
d) Minicom/Cutecom.

Tale progetto è stato testato:
a-bis) Distribuzione: Ubuntu Mate 14.04.3 LTS (Trusty Tahr), Kernel Version: 3.16.0-46-generic
b-bis) gcc version 4.9.3 20150529 (release) [ARM/embedded-4_9-branch revision 227977] 
(GNU Tools for ARM Embedded Processors) 
c-bis) Open On-Chip Debugger 0.10.0-dev-00036-g48787e1
d-bis) cutecom version 0.22.0-2

HOW-TO, per avviare il progetto:

1. Creare nella home dell'utente un file nascosto, nominato .gdbinit con dentro:
set auto-load safe-path /home/nome_utente

2. Scaricare nella home dell'utente il template con il seguente comando da terminale:
- git clone https://github.com/gnunicky/project_rai_nucleo_stm32f411re_hal.git  

3. Andare nella directory magnetic_core_memory_hal:
- cd project_rai_nucleo_stm32f411re_hal/magnetic_core_memory_hal/

4. Creare nella cartella un ulteriore file nascosto, nominato .gdbinit richiesto
dal server OpenOCD GDB:
layout split
target extended localhost:3333
monitor arm semihosting enable
monitor reset halt
load
monitor reset init
 
5. Ripulire e Compilare il progetto, con i seguenti comandi da terminale:
- make clean
- make

6. Attaccare la board Nucleo 411RE con la driver shield e la IBM DJB 373330, 
   con il seguente comando da terminale:
- make flash_openocd / flash_stlink

7. Avviare il server, con il seguente comando da terminale:
- make openocd

8. Avviare il debugger da un'altra scheda, con il seguente comando da terminale:
- make debug

9. Avviare un serial terminal tipo minicom/cutecom, settandolo con i seguenti parametri:
   Device=/dev/ttyACM0
   Baud Rate: 115200
   Parity: None
   Data bits: 8
   Stop bits: 1

10. Ritornare nella scheda del debbuger, digitando da terminale:
- run o r

11. Adesso dal serial terminal è possibile interagire con il software.
