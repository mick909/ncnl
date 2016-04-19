#!/bin/sh

avrdude -C ../../avrdude.conf -c avrispmkII -p t13 -P usb -U lfuse:w:0x6b:m -v -b 2152

avrdude -C ../../avrdude.conf -c avrispmkII -p t13 -P usb -U flash:w:obj/led_sensor.hex -v -b 2152

avrdude -C ../../avrdude.conf -c avrispmkII -p t13 -P usb -U lock:w:0x3c:m -v -b 2152