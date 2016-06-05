#!/bin/sh
avrdude -C ../../../avrdude.conf -c avrispmkII -p t85 -P usb -U flash:w:obj/i2c_client.hex -v -b 19200

