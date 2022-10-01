// sim3
// Copyright (c) 2021, Joshua Scoggins
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The main for the arduino version of the simulator's main
// Created by jwscoggins on 8/21/21.
//
#include <Arduino.h>
using Address = uint32_t;
using ByteOrdinal = uint8_t;
using ByteInteger = int8_t;
using ShortOrdinal = uint16_t;
using ShortInteger = int16_t;
using Ordinal = uint32_t;
using Integer = int32_t;
using LongOrdinal = uint64_t;
using LongInteger = int64_t;
constexpr size_t ConfigurationAddress = 0x7F00;
template<typename T>
volatile T& memory(size_t address) noexcept {
    return *reinterpret_cast<volatile T*>(address);
}
struct ConfigRegisters {
    Address address;
};

static_assert(sizeof(ConfigRegisters) <= 256);

volatile ConfigRegisters& 
configRegs() noexcept {
    return memory<ConfigRegisters>(ConfigurationAddress);
}


void
set328BusAddress(Address address) noexcept {
    configRegs().address = address;
}

union SplitAddress {
    constexpr SplitAddress(Address a) : addr(a) { }
    Address addr;
    struct {
        Address lower : 15;
        Address rest : 17;
    };
};
    

Ordinal
load32(Address address) noexcept {
    set328BusAddress(address);
    SplitAddress split(address);
    return memory<Ordinal>(static_cast<size_t>(split.lower) + 0x8000);
}
void 
store32(Address address, Ordinal value) noexcept {
    set328BusAddress(address);
    SplitAddress split(address);
    memory<Ordinal>(static_cast<size_t>(split.lower) + 0x8000) = value;
}
union Register {
    constexpr explicit Register(Ordinal value = 0) : o(value) { }
    Ordinal o;
    Integer i;
    Address a;
    byte bytes[sizeof(Ordinal)];
    ShortOrdinal shorts[sizeof(Ordinal)/sizeof(ShortOrdinal)];
    struct {
        Integer displacement : 13;
        Ordinal m1: 1;
        Ordinal src2: 5;
        Ordinal src1: 5;
        Ordinal opcode : 8;
    } cobr;
    void clear() noexcept { 
        o = 0; 
    }
    [[nodiscard]] constexpr ByteOrdinal getOpcode() const noexcept { return bytes[3]; }
};
using RegisterFrame = Register[16];
Register ip, ac, pc, tc;
volatile bool int0Triggered = false;
volatile bool int1Triggered = false;
volatile bool int2Triggered = false;
volatile byte advanceBy = 0;
RegisterFrame globals;
RegisterFrame* locals;
RegisterFrame sets[4];
Register instruction;
Register optionalDisplacement;
void 
setup() {
    // cleave the address space in half via sector limits.
    // lower half is io space for the implementation
    // upper half is the window into the 32/8 bus
    SFIOR = 0; // this will setup the full 64k space, no pullup disable, no bus
               // keeper and leaving PSR10 alone
    EMCUCR = 0b0'100'00'0'0; // INT2 is falling edge, carve XMEM into two 32k
                             // sectors, no wait states, and make sure INT2 is
                             // falling edge
    MCUCR = 0b1000'10'10; // enable XMEM, leave sleep off, and set int1 and
                          // int0 to be falling edge
    set328BusAddress(0);
    locals = &sets[0];
    // so we need to do any sort of processor setup here
    ip.clear();
    for (int i = 0; i < 16; ++i) {
        globals[i].clear();
        sets[0][i].clear();
        sets[1][i].clear();
        sets[2][i].clear();
        sets[3][i].clear();
    }
}
void
raiseFault(byte code) noexcept {

}
void 
loop() {
    advanceBy = 4;
    instruction.o = load32(ip.a);
    switch (instruction.getOpcode()) {
        case 0x08:
            ip.i += instruction.cobr.displacement;
            advanceBy = 0;
            break;
        case 0x09:
            break;
        default:
            raiseFault(128);
            break;

    }
    // okay we got here so we need to start grabbing data off of the bus and
    // start executing the next instruction
    ip.o += advanceBy; 
}

ISR(INT0_vect) {
    int0Triggered = true;
}


ISR(INT1_vect) {
    int1Triggered = true;

}
ISR(INT2_vect) {
    int2Triggered = true;

}
