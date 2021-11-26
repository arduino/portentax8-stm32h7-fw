#!/bin/bash

lsmod | grep x8h7_drv
if [ $? == 1 ]; then
insmod /home/root/extra/x8h7_drv.ko
fi

lsmod | grep x8h7i_gpio
if [ $? == 1 ]; then                  
insmod /home/root/extra/x8h7_gpio.ko
fi

fail=0

echo 161 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio161/direction
echo 1 > /sys/class/gpio/gpio161/value
echo 162 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio162/direction
if [ $(cat /sys/class/gpio/gpio162/value) == 1 ]; then
   echo "GPIO 1 set to 1 - GPIO 2 reads 1"
else
   echo "GPIO 1 set to 1 - GPIO 2 reads 0"
   let fail=1
fi

echo 0 > /sys/class/gpio/gpio161/value
if [ $(cat /sys/class/gpio/gpio162/value) == 0 ]; then
   echo "GPIO 1 set to 0 - GPIO 2 reads 0"
else
   echo "GPIO 1 set to 0 - GPIO 2 reads 1"
   let fail=1
fi

echo 161 > /sys/class/gpio/unexport
echo 162 > /sys/class/gpio/unexport

if [ $fail==1 ]; then
   echo "FAIL :("
   exit 0
else
   echo "PASS :)"
fi

