:floppy_disk: `portentax8-stm32h7-fw`
=====================================
[![Smoke test status](https://github.com/arduino/portentax8-stm32h7-fw/actions/workflows/smoke-test.yml/badge.svg)](https://github.com/arduino/portentax8-stm32h7-fw/actions/workflows/smoke-test.yml)
[![Sync Labels status](https://github.com/arduino/portentax8-stm32h7-fw/actions/workflows/sync-labels.yml/badge.svg)](https://github.com/arduino/portentax8-stm32h7-fw/actions/workflows/sync-labels.yml)

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
**Note**: If you want to obtain the debug messages printed via `dbg_printf` you need to `make clean` followed by `make debug` and connect a 3V3 FTDI adapter to `UART0` on the [Portenta Breakout Board](https://store.arduino.cc/products/arduino-portenta-breakout).
#### Upload to `Portenta X8`
You can upload files to the Portenta X8 via `adb push`. Note: adb can only push `/tmp` and `/home/fio`.
```bash
adb push STM32H747AII6_CM7.bin /home/fio
adb push ...
```
Then open a shell to the X8 via `adb shell` and move the file from within `/home/fio` to `/usr/lib/firmware/arduino/stm32h7-fw` using `sudo mv`. Prior to that you need to remount `/usr` with read/write permissions (it's mounted read-only per default).
```bash
sudo mount -o remount,rw /usr
sudo mv STM32H747AII6_CM7.bin /usr/lib/firmware/arduino/stm32h7-fw/STM32H747AII6_CM7.bin
```
#### Flash `STM32H747AIIX`/Cortex-M7 firmware
```bash
cd /usr/arduino/extra
sudo ./program.sh
```
### IMX8 / H7 Interface Block Diagram
Every communication is initated by the IMX8 who serves as SPI controller. This happens either by the user issueing a `write` call on any of the interfaces or by the H7 requesting the start of a communication by setting `IRQ` to `1`. A detailed description of the used protocol can be found [here](doc/protocol.md).
<p align="center">
  <img src="doc/img/portenta-x8h7-interface-block-diagram.png" width="75%">
</p>
