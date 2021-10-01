# copy me to the linux board ;)

# Bug in stm32flash binary, RESET is not being set correctly
# CHANGEME: on actual hardware, the GPIO will likely be different
echo 10 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio10/direction
echo 1 > /sys/class/gpio/gpio10/value
echo 11 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio11/direction


stm32flash -o /dev/spidev2.0 -i '11,-10,,10,,:-11,-10,,10'
stm32flash -w /home/fio/STM32H747AII6_CM7.bin /dev/spidev2.0 -i '11,-10,,10,,:-11,-10,,10' -e 0
