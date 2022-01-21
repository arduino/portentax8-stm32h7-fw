#!/bin/sh

# copy me to the linux board ;)

openocd -f /usr/arduino/extra/openocd_script.cfg -c "program /usr/arduino/extra/STM32H747AII6_CM7.bin verify reset exit"
