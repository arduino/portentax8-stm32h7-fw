#!/bin/sh

# copy me to the linux board ;)

openocd -f /home/fio/extra/openocd_script.cfg -c "program /home/fio/extra/STM32H747AII6_CM7.bin verify reset exit"
