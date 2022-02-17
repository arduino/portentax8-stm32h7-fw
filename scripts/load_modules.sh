#!/bin/sh
#
# This script is stored within /usr/arduino/extra .
#
# The purpose of this script is to load the kernel
# modules allowing to access the extended IO interfaces
# provided by the STM32H7.

modprobe industrialio
insmod /usr/arduino/extra/x8h7_drv.ko
insmod /usr/arduino/extra/x8h7_can.ko
insmod /usr/arduino/extra/x8h7_gpio.ko
insmod /usr/arduino/extra/x8h7_adc.ko
insmod /usr/arduino/extra/x8h7_rtc.ko
insmod /usr/arduino/extra/x8h7_pwm.ko
insmod /usr/arduino/extra/x8h7_uart.ko
insmod /usr/arduino/extra/x8h7_ui.ko
insmod /usr/arduino/extra/x8h7_h7.ko
