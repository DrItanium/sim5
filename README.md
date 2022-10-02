# sim5 - an i960 simulator targeting the atmega8515

The atmega8515 is a very simple AVR microcontroller with 8k of flash and 512
bytes of SRAM. It is actually a very good target to emulate an i960 for one
very important reason. It comes with something called EBI/XMEM. This maps a
16-bit bus to the data space of the 8515. Thus it becomes simple to access
external devices very easily. Combining this with a series of external latches
and an SRAM chip allows one to create a 32-bit bus and also enough space to
cache values if necessary. This means we can emulate a 32-bit address bus with
an 8-bit data bus trivially. I call this bus a 32/8 bus and it is where the
emulator pulls instructions from. It maps the upper 32k of the EBI to this
window with the lower 32k being reserved for internal features like the latches
and anything else needed. The SRAM chip is optional but a good idea just in
case. 

It also provides the emulator with scratch space or rom space if necessary for
anything.

