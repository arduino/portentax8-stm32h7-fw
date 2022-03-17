#!/bin/bash

lsmod | grep x8h7_drv
if [ $? == 1 ]; then
insmod /home/root/extra/x8h7_drv.ko
fi

lsmod | grep x8h7_pwm
if [ $? == 1 ]; then
insmod /home/root/extra/x8h7_pwm.ko
fi

lsmod | grep x8h7_adc
if [ $? == 1 ]; then
modprobe industrialio
insmod /home/root/extra/x8h7_adc.ko
fi

read -p "Please connect PWM0 to PMW8 and then press Enter to continue"

PERIOD=20000
DUTY=12000

fail=0

if [ -d "/sys/class/pwm/pwmchip0/pwm0" ]; then
   cd /sys/class/pwm/pwmchip0/pwm0
   if [ $(cat enable) == 1 ]; then
      echo 0 > enable
# The values of period and duty_cycle need to be updated
# If you write twice the same values, since there is no change,
# the second pwm configuration packet will not be sent
      if [ $(cat period) -gt 10 ]; then
         echo 5 > duty_cycle
         echo 10 > period
      fi
   fi
else   
   echo 0 > /sys/class/pwm/pwmchip0/export
   cd /sys/class/pwm/pwmchip0/pwm0
fi
echo $PERIOD > period
echo $DUTY > duty_cycle
echo 1 > enable

if [ -d "/sys/class/pwm/pwmchip0/pwm8" ]; then
   cd /sys/class/pwm/pwmchip0/pwm8
else  
   echo 8 > /sys/class/pwm/pwmchip0/export
   cd /sys/class/pwm/pwmchip0/pwm8            
fi

echo "First capture:"

PWM_CAPTURE=$(cat capture)

#Extract period and duty_cycle values
PERIOD_CAPTURED=$(echo $PWM_CAPTURE | cut -d' ' -f1)
DUTY_CAPTURED=$(echo $PWM_CAPTURE | cut -d' ' -f2)

echo "PWM 0 period: $PERIOD  -  PWM 8 period captured: $PERIOD_CAPTURED"
echo "PWM 0 duty cycle: $DUTY  -  PWM 8 duty cycle captured: $DUTY_CAPTURED"

if [ "$PERIOD_CAPTURED" -lt `expr $PERIOD - 20` ] || [ "$PERIOD_CAPTURED" -gt `expr $PERIOD + 20` ]; then
   let fail=1
fi

if [ "$DUTY_CAPTURED" -lt `expr $DUTY - 20` ] || [ "$DUTY_CAPTURED" -gt `expr $DUTY + 20` ]; then
   let fail=1
fi

#Take a second capture

PERIOD=46320
DUTY=38270

cd ../pwm0
echo 0 > enable
echo $PERIOD > period
echo $DUTY > duty_cycle
echo 1 > enable
cd ../pwm8

echo "Second capture:"

PWM_CAPTURE=$(cat capture)

#Extract period and duty_cycle values
PERIOD_CAPTURED=$(echo $PWM_CAPTURE | cut -d' ' -f1)
DUTY_CAPTURED=$(echo $PWM_CAPTURE | cut -d' ' -f2)

echo "PWM 0 period: $PERIOD  -  PWM 8 period captured: $PERIOD_CAPTURED"
echo "PWM 0 duty cycle: $DUTY  -  PWM 8 duty cycle captured: $DUTY_CAPTURED"

if [ "$PERIOD_CAPTURED" -lt `expr $PERIOD - 10` ] || [ "$PERIOD_CAPTURED" -gt `expr $PERIOD + 10` ]; then
   let fail=1
fi

if [ "$DUTY_CAPTURED" -lt `expr $DUTY - 10` ] || [ "$DUTY_CAPTURED" -gt `expr $DUTY + 10` ]; then
   let fail=1
fi

if [ $fail == 1 ]; then
   echo "FAIL :("
   exit 0
else
   echo "PASS :)"
fi
