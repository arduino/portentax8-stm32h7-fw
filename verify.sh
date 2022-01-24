#!/bin/sh

# copy me to the linux board ;)

openocd -f /usr/arduino/extra/openocd_script.cfg -c "flash verify_image /usr/arduino/extra/STM32H747AII6_CM7.bin 0x8000000" -c "reset" -c "exit"
