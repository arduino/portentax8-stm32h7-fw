`portentam8-stm32h7-fw`
=======================
This repository contains the firmware running on the `STM32H747AIIX`/Cortex-M7 core which, in combination with loadable kernel modules within the Linux distribution, provides access to various IO busses on the expansion headers of the Portenta X8.

### Developer Guide
#### Build
* Either `make`
```bash
make
```
* or `bitbake`.
```bash
bitbake linux-firmware-arduino-portenta-x8-stm32h7
```
#### Upload to `Portenta X8`
You can upload files to the Portenta X8 via `adb push`. Note: adb can only push `/tmp` and `/home/fio`.
```bash
adb push STM32H747AII6_CM7.bin /home/fio
adb push ...
```
Then open a shell to the X8 via `adb shell` and move the file from within `/home/fio` to `/usr/lib/firmware/arduino/stm32h7-fw` using `sudo mv`. Prior to that you need to remount `/usr` with read/write permissions (it's mounted read-only per default).
```bash
mount -o remount,rw /usr
sudo mv STM32H747AII6_CM7.bin /usr/lib/firmware/arduino/stm32h7-fw/STM32H747AII6_CM7.bin
```
#### Flash `STM32H747AIIX`/Cortex-M7 firmware
```bash
cd /usr/arduino/extra
sudo ./program.sh
```
### IMX8 / H7 Interface Block Diagram
<p align="center">
  <img src="doc/img/portenta-x8h7-interface-block-diagram.png" width="75%">
</p>
