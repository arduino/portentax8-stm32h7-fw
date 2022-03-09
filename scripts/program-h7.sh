#!/bin/sh
FIRMWARE_H7_ON_MCU=$(cat /sys/kernel/x8h7_firmware/version)
sudo -u fio dd if=/usr/arduino/extra/STM32H747AII6_CM7.bin of=/tmp/version bs=1 count=40 skip=$((0x40000))
FIRMWARE_H7_ON_LINUX=$(strings /tmp/version | head -n1)

if [ "$FIRMWARE_H7_ON_MCU" = "$FIRMWARE_H7_ON_LINUX" ]; then
  echo "Firmware on H7 matches firmware stored on M8. No Update."
else
  echo "Firmware on H7 does not match firmware stored on M8. Performing Update."
  echo "Version(H7)="$FIRMWARE_H7_ON_MCU
  echo "Version(M8)="$FIRMWARE_H7_ON_LINUX
  openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -c "program /usr/arduino/extra/STM32H747AII6_CM7.bin verify reset exit 0x8000000"
fi

