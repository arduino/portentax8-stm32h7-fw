#!/bin/bash

lsmod | grep x8h7_drv
if [ $? == 1 ]; then
insmod /home/root/extra/x8h7_drv.ko
fi

lsmod | grep x8h7i_adc
if [ $? == 1 ]; then
modprobe industrialio                
insmod /home/root/extra/x8h7_adc.ko
fi

read -p "Please provide 3.3V to ADC0 and then press Enter to continue"

fail=0

ADC_CAPTURE=$(cat /sys/bus/iio/devices/iio:device0/in_voltage0_raw)
echo "A0 = $ADC_CAPTURE"

if [ "$ADC_CAPTURE" -lt `expr 65535 - 10` ]; then
   echo "First read failed!"
   let fail=1
fi

read -p "Please provide 1.8V to ADC0 and then press Enter to continue"

ADC_CAPTURE=$(cat /sys/bus/iio/devices/iio:device0/in_voltage0_raw)  
echo "A0 = $ADC_CAPTURE"

if [ "$ADC_CAPTURE" -lt `expr 35746 - 1000` ] | [ "$ADC_CAPTURE" -gt `expr 35746 + 1000` ] ; then                    
   echo "Second read failed! Limits: 34746 - 36746"
   let fail=1
fi

if [ $fail == 1 ]; then
   echo "FAIL :("
   exit 0
else
   echo "PASS :)"
fi

