#!/bin/sh
#
# This script is stored within /usr/arduino/extra .
#
# Programming the STM32H7 via stm32flash is deprecated
# and stm32flash is no longer part of the Yocto build.
#
# [IMX8] GPIO1_IO10 = [STM32H7] NRST
# [IMX8] GPIO1_IO11 = [STM32H7] BOOT0

lsmod | grep x8h7
if [ $? == 1 ]; then
    insmod /usr/arduino/extra/x8h7_drv.ko
    modprobe industrialio
    insmod /usr/arduino/extra/x8h7_adc.ko
fi

# Bug in stm32flash binary, RESET is not being set correctly
echo 10 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio10/direction
echo 1 > /sys/class/gpio/gpio10/value
echo 11 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio11/direction

stm32flash -o /dev/spidev2.0 -i '11,-10,,10,,:-11,-10,,10'
stm32flash -w /usr/arduino/extra/STM32H747AII6_CM7.bin /dev/spidev2.0 -i '11,-10,,10,,:-11,-10,,10' -e 0
#stm32flash -w /usr/arduino/extra/Blink.ino.bin -i '11,-10,,10,,:-11,-10,,10' -e 0 -S 0x8100000 /dev/spidev2.0
