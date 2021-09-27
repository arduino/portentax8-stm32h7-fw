# copy me to the linux board ;)

# Bug in stm32flash binary, RESET is not being set correctly
# CHANGEME: on actual hardware, the GPIO will likely be different
echo 7 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio7/direction


stm32flash -o /dev/spidev1.0 -i '8,-7,,7,,:-8,-7,,7'
stm32flash -w /home/fio/STM32H747AII6_CM7.bin /dev/spidev1.0 -i '8,-7,,7,,:-8,-7,,7' -e 0
