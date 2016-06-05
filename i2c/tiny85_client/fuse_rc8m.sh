#!/bin/sh
avrdude -C ../../../avrdude.conf -c avrispmkII -p t85 -P usb -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m -v -b 19200

