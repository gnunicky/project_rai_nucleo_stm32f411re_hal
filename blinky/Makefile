# Basic variable definitions
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

PROJECT_NAME = blinky

PROJECT_SRC  = blinky

DEVICE_SRC   = ../Device/
BSP_SRC      = ../Drivers/BSP/STM32F4xx-Nucleo/
STM_SRC      = ../Drivers/STM32F4xx_HAL_Driver/Src/
USB_CORE_SRC = ../Drivers/STM32_USB_Device_Library/Core/Src/


vpath %.c $(PROJECT_SRC)
vpath %.c $(BSP_SRC)
vpath %.c $(DEVICE_SRC)
vpath %.c $(STM_SRC)
vpath %.c $(USB_CORE_SRC)

# Location of OpenOCD Board .cfg files
OPENOCD_BOARD_DIR = /usr/local/share/openocd/scripts/board
# GDB ARM Debugger
GDB=arm-none-eabi-gdb

SRCS = blinky.c
SRCS += ../Device/startup_stm32f401xe.s
SRCS += stm32f4xx_it.c
SRCS += system_stm32f4xx.c
SRCS += syscalls.c


EXT_SRCS  = stm32f4xx_hal.c
EXT_SRCS += stm32f4xx_hal_rcc.c
EXT_SRCS += stm32f4xx_hal_gpio.c
EXT_SRCS += stm32f4xx_hal_cortex.c
EXT_SRCS += stm32f4xx_hal_pcd.c
EXT_SRCS += stm32f4xx_hal_pcd_ex.c
EXT_SRCS += stm32f4xx_ll_usb.c
EXT_SRCS += stm32f4xx_hal_uart.c
EXT_SRCS += stm32f4xx_hal_dma.c
EXT_SRCS += stm32f4xx_hal_spi.c
EXT_SRCS += stm32f4xx_hal_adc.c
EXT_SRCS += stm32f4xx_hal_adc_ex.c
EXT_SRCS += stm32f4xx_hal_msp.c


EXT_SRCS += stm32f4xx_nucleo.c


EXT_OBJ = $(EXT_SRCS:.c=.o)

INC_DIRS  = ../magnetic_core_memory/
INC_DIRS += ../Device/
INC_DIRS += ../Drivers/BSP/STM32F4xx-Nucleo/
INC_DIRS += ../Drivers/CMSIS/Device/ST/STM32F4xx/Include/
INC_DIRS += ../Drivers/CMSIS/Include/
INC_DIRS += ../Drivers/STM32F4xx_HAL_Driver/Inc/
INC_DIRS += ../Drivers/STM32_USB_Device_Library/Core/Inc/
INC_DIRS += ../Drivers/STM32_USB_Device_Library/Class/HID/Inc/


INCLUDE = $(addprefix -I,$(INC_DIRS))

DEFS = -DSTM32F401xE

CFLAGS  = -ggdb -O0
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -Wl,--gc-sections

WFLAGS += -Wall -Wextra -Warray-bounds -Wno-unused-parameter

LFLAGS = -T../Device/STM32F401RE_FLASH.ld

.PHONY: all
all: $(PROJECT_NAME)

.PHONY: $(PROJECT_NAME)
$(PROJECT_NAME): $(PROJECT_NAME).elf

$(PROJECT_NAME).elf: $(SRCS) $(EXT_OBJ)
	$(CC) $(INCLUDE) $(DEFS) $(CFLAGS) $(WFLAGS) $(LFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(PROJECT_NAME).elf   $(PROJECT_NAME).hex
	$(OBJCOPY) -O binary $(PROJECT_NAME).elf $(PROJECT_NAME).bin

%.o: %.c
	$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^

clean:
	rm -f *.o $(PROJECT_NAME).elf $(PROJECT_NAME).hex $(PROJECT_NAME).bin

# Flashing boards using STLink
flash_stlink:
	st-flash write $(PROJECT_NAME).bin 0x8000000

# Flashing boards using OpenOCD
flash_openocd:
	openocd -f $(OPENOCD_BOARD_DIR)/st_nucleo_f4.cfg -c "program $(PROJECT_NAME).bin verify reset exit 0x8000000"

# Run debugger
debug:
	$(GDB) $(PROJECT_NAME).elf

# Start OpenOCD GDB server (supports semihosting)
openocd:
	openocd -f $(OPENOCD_BOARD_DIR)/st_nucleo_f4.cfg
