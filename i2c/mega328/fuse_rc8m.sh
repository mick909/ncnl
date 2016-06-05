#!/bin/sh
avrdude -C ../../../avrdude.conf -c avrispmkII -p m328p -P usb -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0x07:m -v -b 19200

