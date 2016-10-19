#!/bin/sh
avrdude -C ../../avrdude.conf -c avrispmkII -p x32e5 -P usb -U flash:w:obj/vt_xmega.hex -v -b 64000

