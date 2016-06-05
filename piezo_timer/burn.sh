#!/bin/sh
avrdude -C ../../avrdude.conf -c avrispmkII -p m328p -P usb -e -v -b 64000
avrdude -C ../../avrdude.conf -c avrispmkII -p m328p -P usb -U lfuse:w:0xf7:m -U hfuse:w:0xdf:m -U efuse:w:0x07:m -v -b 64000
avrdude -C ../../avrdude.conf -c avrispmkII -p m328p -P usb -U flash:w:obj/shottimer.hex -v -b 64000
avrdude -C ../../avrdude.conf -c avrispmkII -p m328p -P usb -U lock:w:0x00:m -v -b 64000

