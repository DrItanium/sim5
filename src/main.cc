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
enum class Opcodes : uint8_t {
    b = 0x08,
    call,
    ret,
    bal,
    bno = 0x10,
    bg,
    be,
    bge,
    bl,
    bne,
    ble,
    bo,
    faultno,
    faultg,
    faulte,
    faultge,
    faultl,
    faultne, 
    faultle, 
    faulto,
    testno,
    testg,
    teste,
    testge,
    testl,
    testne,
    testle,
    testo,
    bbc = 0x30,
    cmpobg,
    cmpobe,
    cmpobge,
    cmpobl,
    cmpobne,
    cmpoble,
    bbs,
    cmpibno,
    cmpibg,
    cmpibe,
    cmpibge,
    cmpibl,
    cmpibne,
    cmpible,
    cmpibo,
    reg0 = 0x58,
    reg1,
    reg2,
    reg3,
    reg4,
    reg5,
    reg6,
    reg7,
    reg8, reg9, reg10, reg11, reg12, reg13, reg14, reg15,
    reg16, reg17, reg18, reg19, reg20, reg21, reg22, reg23,
    reg24, reg25, reg26, reg27, reg28, reg29, reg30, reg31,
    reg32, reg33, reg34, reg35, reg36, reg37, reg38, reg39,
    ldob = 0x80,
    stob = 0x82,
    bx = 0x84,
    balx,
    callx,
    ldos = 0x88,
    stos = 0x8a,
    lda = 0x8c,
    ld = 0x90,
    st = 0x92,
    ldl = 0x98,
    stl = 0x9a,
    ldt = 0xa0,
    stt = 0xa2,
    ldq = 0xb0,
    stq = 0xb2,
    ldib = 0xc0,
    stib = 0xc2,
    ldis = 0xc8,
    stis = 0xca,
};
enum class MEMFormatMode : uint8_t {
    AbsoluteOffset = 0b0000,
    Bad = 0b0001,
    RegisterIndirectWithOffset = 0b1000,
    RegisterIndirect = 0b0100, 
    IPWithDisplacement = 0b0101, 
    Reserved = 0b0110, 
    RegisterIndirectWithIndex = 0b0111,

    AbsoluteDisplacement = 0b1100,
    RegisterIndirectWithDisplacement = 0b1101,
    IndexWithDisplacement = 0b1110,
    RegisterIndirectWithIndexAndDisplacement = 0b1111,
};
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
    struct {
        Ordinal src1 : 5;
        Ordinal s1 : 1;
        Ordinal s2 : 1;
        Ordinal opcodeExt : 4;
        Ordinal m1 : 1;
        Ordinal m2 : 1;
        Ordinal m3 : 1;
        Ordinal src2 : 5;
        Ordinal srcDest : 5;
        Ordinal opcode : 8;
    } reg;
    struct {
        Integer displacement : 24;
        Ordinal opcode : 8;
    } ctrl;
    struct {
        Ordinal offset: 12;
        Ordinal selector : 1;
        Ordinal selector2 : 1;
        Ordinal abase : 5;
        Ordinal srcDest : 5;
        Ordinal opcode : 8;
    } mem;
    struct {
        Ordinal offset : 12;
        Ordinal fixed : 1;
        Ordinal action : 1;
        Ordinal abase : 5;
        Ordinal srcDest : 5;
        Ordinal opcode : 8;
    } mema;
    struct {
        Ordinal index : 5;
        Ordinal unused : 2;
        Ordinal scale : 3;
        Ordinal modeMinor : 2;
        Ordinal fixed : 1;
        Ordinal group: 1;
        Ordinal abase : 5;
        Ordinal srcDest : 5;
        Ordinal opcode : 8;
    } memb;
    struct {
        Ordinal index : 5;
        Ordinal unused : 2;
        Ordinal scale : 3;
        Ordinal registerIndirect : 1;
        Ordinal useIndex : 1;
        Ordinal fixed : 1;
        Ordinal group: 1;
        Ordinal abase : 5;
        Ordinal srcDest : 5;
        Ordinal opcode : 8;
    } memb_grp2;
    bool isMEMA() const noexcept { return !mem.selector; }
    bool isMEMB() const noexcept { return mem.selector; }
    bool isDoubleWide() const noexcept {
        return isMEMB() && (memb.group || (memb.modeMinor == 0b01));
    }
    void clear() noexcept { 
        o = 0; 
    }
    [[nodiscard]] constexpr auto getOpcode() const noexcept { return static_cast<Opcodes>(bytes[3]); }
};
static_assert(sizeof(Register) == sizeof(Ordinal));
Register ip, ac, pc, tc;
volatile bool int0Triggered = false;
volatile bool int1Triggered = false;
volatile bool int2Triggered = false;
volatile byte advanceBy = 0;
// On the i960 this is separated out into two parts, locals and globals
// The idea is that globals are always available and locals are per function.
// You are supposed to have multiple local frames on chip to accelerate
// operations. However, to simplify things I am not keeping any extra sets on
// chip for the time being so there is just a single register block to work
// with
Register gprs[32]; 
Register instruction;
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
    // so we need to do any sort of processor setup here
    ip.clear();
    for (int i = 0; i < 32; ++i) {
        gprs[i].clear();
    }
}
void
raiseFault(byte code) noexcept {

}
void 
call() {

}
void
ret() {

}
Ordinal
computeAddress() noexcept {
    if (instruction.isMEMA()) {
        Ordinal result = instruction.mem.offset;
        if (instruction.mema.action) {
            result += gprs[instruction.mem.abase].o;
        }
        return result;
    } else {
        // okay so we need to figure out the minor mode after figuring out if
        // it is a double wide operation or not
        if (instruction.memb.group) {
            // okay so it is going to be the displacement versions
            // load 32-bits into the optionalDisplacement field
            advanceBy += 4;
            Integer result = static_cast<Integer>(load32(ip.a + 4)); // load the optional displacement
            if (instruction.memb_grp2.useIndex) {
                result += (gprs[instruction.memb_grp2.index].i << static_cast<Integer>(instruction.memb_grp2.scale));
            }
            if (instruction.memb_grp2.registerIndirect) {
                result += gprs[instruction.memb_grp2.abase].i;
            }
            return static_cast<Ordinal>(result);
        } else {
            // okay so the other group isn't as cleanly designed
            switch (instruction.memb.modeMinor) {
                case 0b00: // Register Indirect
                    return gprs[instruction.memb.abase].o;
                case 0b01: // IP With Displacement 
                    advanceBy += 4;
                    return static_cast<Ordinal>(ip.i + static_cast<Integer>(load32(ip.a + 4)) + 8);
                case 0b11: // Register Indirect With Index
                    return gprs[instruction.memb.abase].o + (gprs[instruction.memb.index].o << instruction.memb.scale);
                default:
                    return -1;
            }
        }
    }
}
void 
loop() {
    advanceBy = 4;
    instruction.o = load32(ip.a);
    switch (instruction.getOpcode()) {
        case Opcodes::bal: // bal
            gprs[14].o = ip.o + 4;
            // then fallthrough and take the branch
        case Opcodes::b: // b
            ip.i += instruction.cobr.displacement;
            advanceBy = 0;
            break;
        case Opcodes::call: // call
            call();
            break;
        case Opcodes::ret: // ret
            ret();
            break;
        case Opcodes::ld: 
            gprs[instruction.mem.srcDest].o = load32(computeAddress());
            break;
        case Opcodes::st: 
            store32(computeAddress(), gprs[instruction.mem.srcDest].o);
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
