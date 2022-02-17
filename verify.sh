#!/bin/sh
#
# This script is stored within /usr/arduino/extra .
#
# This script uses openocd to determine if the firmware
# loaded in the STM32H7 is different from the one currently
# stored on the Linux board.

openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -c "flash verify_image /usr/arduino/extra/STM32H747AII6_CM7.bin 0x8000000" -c "reset" -c "exit"
