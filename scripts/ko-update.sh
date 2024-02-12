#!/bin/sh
sudo rmmod x8h7_can
sudo mv *.ko /lib/modules/5.10.93-lmp-standard/extra/
sudo modprobe x8h7_can

