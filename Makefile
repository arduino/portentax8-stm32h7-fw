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
CXX = $(CROSS_COMPILE)g++
AS = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size

LINKER_SCRIPT = linker/STM32H747AIIX_FLASH.ld

CFLAGS  = -O2 -Wall -Werror -pedantic -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -std=gnu11 -g3 -ffunction-sections -fdata-sections -fstack-usage
CXXFLAGS  = -O2 -Wall -Werror -pedantic -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -std=c++11 -g3 -ffunction-sections -fdata-sections -fstack-usage
ASFLAGS = -O2 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -c -x assembler-with-cpp -g3 
LDFLAGS = --specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs -T$(LINKER_SCRIPT) -Wl,--start-group -lc -lm -Wl,--end-group

TAG_COMMIT := $(shell git rev-list --abbrev-commit --tags --max-count=1)
TAG := $(shell git describe --abbrev=0 --tags ${TAG_COMMIT} 2>/dev/null || true)
COMMIT := $(shell git rev-parse --short HEAD)
DATE := $(shell git log -1 --format=%cd --date=format:"%Y%m%d:%H%M%S")
VERSION := $(TAG:v%=%)
ifneq ($(COMMIT), $(TAG_COMMIT))
    VERSION := $(VERSION)-next-$(COMMIT)-$(DATE)
endif
ifeq ($(VERSION),)
    VERSION := $(COMMIT)-$(DATA)
endif
ifneq ($(shell git status --porcelain),)
    VERSION := $(VERSION)-dirty
endif

DEFINES = \
	  -DCORE_CM7 \
	  -DUSE_HAL_DRIVER \
	  -DSTM32H747xx \
	  -DVECT_TAB_SRAM \
	  -DMETAL_INTERNAL \
      -DVIRTIO_MASTER_ONLY \
      -DNO_ATOMIC_64_SUPPORT \
      -DMETAL_MAX_DEVICE_REGIONS=2 \
      -DRPMSG_BUFFER_SIZE=2000 \
      -DREALVERSION=\"$(VERSION)\"

INCLUDES = \
	   -Iinclude \
	   -Ilibraries/STM32H7xx_HAL_Driver/Inc \
	   -Ilibraries/STM32H7xx_HAL_Driver/Inc/Legacy \
	   -Ilibraries/CMSIS/Device/ST/STM32H7xx/Include \
	   -Ilibraries/CMSIS/Include \
	   -Ilibraries/openamp_arduino/src \
	   -Ilibraries/openamp_arduino/src \
	   -Ilibraries/openamp_arduino/openamp \
	   -Ilibraries/openamp_arduino/metal \

SRCS = \
	src/system_stm32h7xx_dualcore_boot_cm4_cm7.c \
	src/main.c \
	src/can.c \
	src/can_handler.c \
	src/can_util.c \
	src/error_handler.c \
	src/peripherals.c \
	src/ringbuffer.c \
	src/rpc.c \
	src/stm32h7xx_it.c \
	src/syscalls.c \
	src/sysmem.c \
	src/adc.c \
	src/adc_handler.c \
	src/uart.c \
	src/uart_handler.c \
	src/virtual_uart.c \
	src/virtual_uart_handler.c \
	src/pwm.c \
	src/pwm_handler.c \
	src/gpio.c \
	src/gpio_handler.c \
	src/timer.c \
	src/rtc.c \
	src/rtc_handler.c \
	src/spi.c \
	src/system.c \
	src/watchdog.c \
	src/h7_handler.c \
	src/m4_util.c \
	libraries/openamp_arduino/src/condition.c \
	libraries/openamp_arduino/src/device.c \
	libraries/openamp_arduino/src/generic_device.c \
	libraries/openamp_arduino/src/generic_init.c \
	libraries/openamp_arduino/src/generic_io.c \
	libraries/openamp_arduino/src/init.c \
	libraries/openamp_arduino/src/io.c \
	libraries/openamp_arduino/src/irq.c \
	libraries/openamp_arduino/src/log.c \
	libraries/openamp_arduino/src/mailbox_hsem_if.c \
	libraries/openamp_arduino/src/openamp.c \
	libraries/openamp_arduino/src/remoteproc_virtio.c \
	libraries/openamp_arduino/src/rpmsg.c \
	libraries/openamp_arduino/src/rpmsg_virtio.c \
	libraries/openamp_arduino/src/rsc_table.c \
	libraries/openamp_arduino/src/shmem.c \
	libraries/openamp_arduino/src/sys.c \
	libraries/openamp_arduino/src/time.c \
	libraries/openamp_arduino/src/virtio.c \
	libraries/openamp_arduino/src/virtqueue.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hrtim.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_iwdg.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc_ex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
	libraries/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c

SRCS_ASM = \
	startup/startup_stm32h747xx.s

BUILDDIR?=build

OBJS = $(patsubst %.c,$(BUILDDIR)/%.o,$(SRCS))
OBJS += $(patsubst %.s,$(BUILDDIR)/%.o,$(SRCS_ASM))

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

debug:		CFLAGS += -DDEBUG
debug:		CXXFLAGS += -DDEBUG
debug:		$(NAME).bin $(NAME).hex

builddir:
	mkdir -p $(BUILDDIR) && \
	mkdir -p $(BUILDDIR)/src && \
	mkdir -p $(BUILDDIR)/startup && \
	mkdir -p $(BUILDDIR)/libraries/openamp_arduino/src/ && \
	mkdir -p $(BUILDDIR)/libraries/STM32H7xx_HAL_Driver/Src/

$(NAME).elf: builddir $(OBJS)
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
		rm -f $(OBJS) $(OBJS:.o=.su)
		rm -f *~
		rm -rf $(BUILDDIR)

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

$(BUILDDIR)/%.o: %.c | builddir
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -c $< -o $@
	$(MKDEP)

$(BUILDDIR)/%.o: %.s | builddir
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -D__ASSEMBLY__ -c $< -o $@
	$(MKDEP)

-include $(OBJS:.o=.d)
