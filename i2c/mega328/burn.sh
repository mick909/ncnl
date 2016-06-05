#!/bin/sh
avrdude -C ../../../avrdude.conf -c avrispmkII -p m328p -P usb -U flash:w:obj/i2c_master.hex -v -b 19200

