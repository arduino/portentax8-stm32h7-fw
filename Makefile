#
# Copyright 2018 Fabio Baltieri (fabio.baltieri@gmail.com)
#
# Based on the original ben-wpan code written by:
#   Werner Almesberger, Copyright 2010-2011
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
SHELL := /bin/bash

NAME ?= STM32H747AII6_CM7

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
AS = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size

FLASH = stm32flash
FLASHFLAGS = --reset --format ihex
BOOTLOADER = dfu-util
BOOTLOADER_FLAGS = -a 0 -s 0x08000000:leave

LINKER_SCRIPT = CM7/STM32H747AIIX_FLASH.ld

CFLAGS  = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -std=gnu11 -g3 -ffunction-sections -fdata-sections -fstack-usage
ASFLAGS = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -c -x assembler-with-cpp -g3 
LDFLAGS = --specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs -T$(LINKER_SCRIPT) -Wl,--start-group -lc -lm -Wl,--end-group

DEFINES = \
	  -DDEBUG \
	  -DCORE_CM7 \
	  -DUSE_HAL_DRIVER \
	  -DSTM32H747xx

INCLUDES = \
	   -ICM7/Core/Inc \
	   -IDrivers/STM32H7xx_HAL_Driver/Inc \
	   -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
	   -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
	   -IDrivers/CMSIS/Include

OBJS = \
	Common/Src/system_stm32h7xx_dualcore_boot_cm4_cm7.o \
	CM7/Core/Src/main.o \
	CM7/Core/Src/ringbuffer.o \
	CM7/Core/Src/stm32h7xx_hal_msp.o \
	CM7/Core/Src/stm32h7xx_it.o \
	CM7/Core/Src/syscalls.o \
	CM7/Core/Src/sysmem.o \
	CM7/Core/Startup/startup_stm32h747aiix.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hrtim.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_iwdg.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc_ex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.o \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.o


# ----- Verbosity control -----------------------------------------------------

CC_normal	:= $(CC)
BUILD_normal	:=
DEPEND_normal	:= $(CC) -MM -MG

CC_quiet	= @echo "  CC       " $@ && $(CC_normal)
BUILD_quiet	= @echo "  BUILD    " $@ && $(BUILD_normal)
DEPEND_quiet	= @$(DEPEND_normal)

ifeq ($(V),1)
    CC		= $(CC_normal)
    BUILD	= $(BUILD_normal)
    DEPEND	= $(DEPEND_normal)
else
    CC		= $(CC_quiet)
    BUILD	= $(BUILD_quiet)
    DEPEND	= $(DEPEND_quiet)
endif

# ----- Rules -----------------------------------------------------------------

.PHONY:		all clean

all:		$(NAME).bin $(NAME).hex

$(NAME).elf: $(OBJS)
	$(CC) -o $@ $(OBJS) $(DEFINES) $(CFLAGS) $(LDFLAGS)
	$(SIZE) $@

%.bin: %.elf
	$(BUILD) $(OBJCOPY) -O binary --remove-section .dma_buffers  $< $@

%.hex: %.elf
	$(BUILD) $(OBJCOPY) -O ihex $< $@

# ----- Cleanup ---------------------------------------------------------------

clean:
		rm -f $(NAME).bin $(NAME).elf $(NAME).hex
		rm -f $(NAME)_text.{bin,hex}
		rm -f $(OBJS) $(OBJS:.o=.d)
		rm -f *~

# ----- Dependencies ----------------------------------------------------------

MKDEP =									\
	$(DEPEND) $(CFLAGS) $(DEFINES) $(INCLUDES) $< |							\
	  sed 								\
	    -e 's|^$(basename $(notdir $<)).o:|$@:|'			\
	    -e '/^\(.*:\)\? */{p;s///;s/ *\\\?$$/ /;s/  */:\n/g;H;}'	\
	    -e '$${g;p;}'						\
	    -e d >$(basename $@).d;					\
	  [ "$${PIPESTATUS[*]}" = "0 0" ] ||				\
	  { rm -f $(basename $@).d; exit 1; }

%.o: %.c
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -c $< -o $@
	$(MKDEP)

%.o: %.S
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -D__ASSEMBLY__ -c $< -o $@
	$(MKDEP)

-include $(OBJS:.o=.d)

# ----- Programming and device control ----------------------------------------

.PHONY: load boot

load: $(NAME).hex
	$(FLASH) $(FLASHFLAGS) write $(NAME).hex

boot: $(NAME).bin
	$(BOOTLOADER) $(BOOTLOADER_FLAGS) -D $(NAME).bin