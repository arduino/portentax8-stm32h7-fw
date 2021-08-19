# copy me to the linux board ;)
/home/fio/stm32flash -o /dev/spidev1.1 -i '8,-7,,7,,:-8,-7,,7'
/home/fio/stm32flash -w /home/fio/STM32H747AII6_CM7.bin /dev/spidev1.1 -i '8,-7,,7,,:-8,-7,,7' -e 0
