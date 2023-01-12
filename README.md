# sim5 - an i960 simulator targeting microcontrollers with an External Memory Interface

The ATmega2560,2561,1280,1281,640,641,8515, and 162 all have an external memory
bus which uses the unmapped part of the AVR data space to provide access to things like
external memory or other devices. While it only allows access to a 16-bit data
bus, one can use extra GPIOs to provide bank switching to emulate a 32-bit
address bus with an 8-bit data bus. I call this bus a 32/8 bus and it is where
the emulator pulls instructions and data from. It uses the upper 32k of the 64k
memory space to act as a view into the greater 32-bit memory bus. The lower
half of memory is used for internal purposes.

Practically, I have been using an Arduino Mega as the development board.
