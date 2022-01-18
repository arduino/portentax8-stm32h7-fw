#!/bin/sh

# copy me to the linux board ;)

# P1.10: NRST

echo 10 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio10/direction
echo 0 > /sys/class/gpio/gpio10/value
sleep 0.25
echo 1 > /sys/class/gpio/gpio10/value

