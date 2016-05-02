#!/bin/sh
avrdude -C ../../avrdude.conf -c avrispmkII -p m328p -P usb -U flash:w:obj/mp3_player.hex -v -b 19200

