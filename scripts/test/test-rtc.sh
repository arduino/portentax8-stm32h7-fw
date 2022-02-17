lsmod | grep x8h7_drv
if [ $? == 1 ]; then
insmod /home/root/extra/x8h7_drv.ko
fi

lsmod | grep x8h7_rtc
if [ $? == 1 ]; then                   
insmod /home/root/extra/x8h7_rtc.ko
fi

DATE='2020-01-01'
hwclock --set -f /dev/rtc1 --date $DATE
RTC_GET=$(hwclock --get -f /dev/rtc1)

echo "${RTC_GET}"

if [[ "$RTC_GET" == *"$DATE"* ]]; then
   echo "PASS :) Rtc get returned correctly!"
else
   echo "FAIL :( Rtc get returned $RTC_GET"
   exit 0
fi
