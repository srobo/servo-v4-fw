#!/bin/sh

# On reading a part code, will prefix it with 'sr' and bake it into a servo
# board, by:
#  * Taking the built sbv4.bin
#  * Processing it with dfu-bootloader/bake_serial_num.pl, to insert the serial
#    number on top of XXXX...
#  * Operating stm32flash (in PATH) to bake the board
#
# This requires that you set up the board into the serial bootloader by pressing
# the button, then scan the code. I also assume the FTDI cable is ttyUSB0.

while read x; do
	x="sr$x"
	./dfu-bootloader/bake_serial_num.pl $x < sbv4.bin > sbv4.sernum.bin
	flashpath=`which stm32flash` # sudo resets PATH
	sudo $flashpath -v -b 19200 -w sbv4.sernum.bin /dev/ttyUSB0
	rm sbv4.sernum.bin
done
