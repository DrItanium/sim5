// sim
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
template<typename T> struct TreatAs { };
using TreatAsOrdinal = TreatAs<Ordinal>;
using TreatAsInteger = TreatAs<Integer>;
constexpr size_t ConfigurationAddress = 0x7F00;
constexpr Ordinal SALIGN = 4;
constexpr Ordinal C = (SALIGN * 16) - 1;
constexpr Ordinal NotC = ~C;
template<typename T>
volatile T& memory(size_t address) noexcept {
    return *reinterpret_cast<volatile T*>(address);
}
struct ConfigRegisters {
    Address address;
    int faultPort; /// @todo remove this when we do this right, this is here
                    /// to make sure the optimizer doesn't get dumb
};

static_assert(sizeof(ConfigRegisters) <= 256);

volatile ConfigRegisters& 
configRegs() noexcept {
    return memory<ConfigRegisters>(ConfigurationAddress);
}
constexpr auto LOCKPIN = PIN_PE2;
void
lockBus() noexcept {
    while (digitalRead(LOCKPIN) == LOW);
    pinMode(LOCKPIN, OUTPUT);
}
void
unlockBus() noexcept {
    pinMode(LOCKPIN, INPUT);
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
    
template<typename T>
T load(Address address, TreatAs<T>) noexcept {
    set328BusAddress(address);
    SplitAddress split(address);
    return memory<T>(static_cast<size_t>(split.lower) + 0x8000);
}
template<typename T>
void store(Address address, T value, TreatAs<T>) noexcept {
    set328BusAddress(address);
    SplitAddress split(address);
    memory<T>(static_cast<size_t>(split.lower) + 0x8000) = value;
}
enum class Opcodes : uint16_t {
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
#if 0
#define X(value) reg_ ## value = value
    X(0x58),
    X(0x59),
    X(0x5a),
    X(0x5b),
    X(0x5c),
    X(0x5d),
    X(0x5e),
    X(0x5f),
    X(0x60),
    X(0x61),
    X(0x64),
    X(0x65),
    X(0x66),
    X(0x67),
    X(0x70),
    X(0x74),
#undef X
#endif
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
    notbit = 0x580,
    andOperation,
    andnot,
    setbit,
    notand,
    xorOperation = 0x586,
    orOperation,
    nor,
    xnor,
    notOperation,
    ornot,
    clrbit,
    notor,
    nand,
    alterbit,
    addo,
    addi,
    subo,
    subi,
    shro = 0x598,
    shrdi = 0x59a,
    shri,
    shlo,
    rotate,
    shli,
    cmpo = 0x5a0,
    cmpi,
    concmpo,
    concmpi,
    cmpinco,
    cmpinci,
    cmpdeco,
    cmpdeci,
    scanbyte = 0x5ac,
    chkbit = 0x5ae,
    addc = 0x5b0,
    subc = 0x5b2,
    mov = 0x5cc,
    movl = 0x5dc,
    movt = 0x5ec,
    movq = 0x5fc,
    synmov = 0x600,
    synmovl,
    synmovq,
    atmod = 0x610,
    atadd = 0x612,
    synld = 0x615,
    spanbit = 0x640,
    scanbit,
    modac = 0x645,
    modify = 0x650,
    extract,
    modtc = 0x654,
    modpc,
    calls = 0x660,
    mark = 0x66b,
    fmark,
    flushreg,
    syncf = 0x66f,
    emul,
    ediv,
    mulo = 0x700,
    remo = 0x708,
    divo = 0x70b,
    muli = 0x710,
    remi = 0x718,
    modi,
    divi = 0x71b,

};
union Register {
    constexpr explicit Register(Ordinal value = 0) : o(value) { }
    Ordinal o;
    Integer i;
    Address a;
    byte bytes[sizeof(Ordinal)];
    ShortOrdinal shorts[sizeof(Ordinal)/sizeof(ShortOrdinal)];
    struct {
        Ordinal unused : 24;
        Ordinal mask : 3;
        Ordinal major : 5;
    } instGeneric;
    struct {
        Integer aligned : 2;
        Integer important : 30;
    } alignedTransfer;
    struct {
        Ordinal s2 : 1;
        Ordinal t : 1;
        Integer displacement : 11;
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
    struct {
        Ordinal conditionCode : 3;
#ifdef NUMERICS_ARCHITECTURE
        Ordinal arithmeticStatus : 4;
        Ordinal unused0 : 1;
#else
        Ordinal unused0 : 5;
#endif // end defined(NUMERICS_ARCHITECTURE)
        Ordinal integerOverflowFlag : 1;
        Ordinal unused1 : 3;
        Ordinal integerOverflowMask : 1;
        Ordinal unused2 : 2;
        Ordinal noImpreciseFaults : 1;
#ifdef NUMERICS_ARCHITECTURE
        Ordinal floatingOverflowFlag : 1;
        Ordinal floatingUnderflowFlag : 1;
        Ordinal floatingInvalidOpFlag : 1;
        Ordinal floatingZeroDivideFlag : 1;
        Ordinal floatingInexactFlag : 1;
        Ordinal unused3 : 3;
        Ordinal floatingOverflowMask : 1;
        Ordinal floatingUnderflowMask : 1;
        Ordinal floatingInvalidOpMask : 1;
        Ordinal floatingZeroDivideMask : 1;
        Ordinal floatingInexactMask : 1;
        Ordinal floatingPointNormalizingMode : 1;
        Ordinal floatingPointRoundingControl : 2;
#else
        Ordinal unused3 : 16;
#endif // end defined(NUMERICS_ARCHITECTURE)
    } arith;
    struct {
        Ordinal rt : 3;
        Ordinal p : 1;
        Ordinal unused : 2; // according to the Sx manual these bits go unused
                            // but in the Hx manual they are used :/
        Ordinal a : 26;
    } pfp;
    struct {
        Ordinal align : 6;
        Ordinal proper : 26;
    } pfpAddress;
    struct {
        Ordinal traceEnable : 1;
        Ordinal executionMode : 1;
        Ordinal unused : 7;
        Ordinal resume : 1;
        Ordinal traceFaultPending : 1;
        Ordinal unused1 : 2;
        Ordinal state : 1;
        Ordinal unused2 : 2;
        Ordinal priority : 5;
        Ordinal internalState : 11;
    } pc;
    bool inSupervisorMode() const noexcept { return pc.executionMode; }
    bool inUserMode() const noexcept { return !inSupervisorMode(); }
    bool isMEMA() const noexcept { return !mem.selector; }
    bool isMEMB() const noexcept { return mem.selector; }
    bool isDoubleWide() const noexcept {
        return isMEMB() && (memb.group || (memb.modeMinor == 0b01));
    }
    void clear() noexcept { 
        o = 0; 
    }
    constexpr Ordinal getPFPAddress() noexcept {
        Register copy(o);
        copy.pfpAddress.align = 0;
        return copy.o;
    }
    void setPFPAddress(Ordinal address) noexcept {
        o = address;
        pfp.unused = 0;
        pfp.p = 0;
        pfp.rt = 0;
    }
    [[nodiscard]] constexpr auto isREGFormat() const noexcept {
        return bytes[3] >= 0x58 && bytes[3] < 0x80;
    }
    [[nodiscard]] constexpr auto getOpcode() const noexcept { 
        if (isREGFormat()) {
            uint16_t baseValue = static_cast<uint16_t>(bytes[3]) << 4;
            return static_cast<Opcodes>(baseValue | static_cast<uint16_t>(reg.opcodeExt));
        } else {
            return static_cast<Opcodes>(bytes[3]); 
        }
    }
    bool getCarryBit() const noexcept { return arith.conditionCode & 0b001; }
    [[nodiscard]] Ordinal modify(Ordinal mask, Ordinal src) noexcept;
    [[nodiscard]] constexpr byte getPriority() const noexcept { return pc.priority; }
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
constexpr auto LRIndex = 14;
constexpr auto FPIndex = 15;
constexpr auto PFPIndex = 16;
constexpr auto SPIndex = 17;
constexpr auto RIPIndex = 18;
Register gprs[32]; 
Register sfrs[32];
Register instruction;
Register& getGPR(byte index) noexcept {
    return gprs[index];
}
Register& getGPR(byte index, byte offset) noexcept {
    return getGPR((index + offset) & 0b11111);
}
void setGPR(byte index, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index).o = value; }
void setGPR(byte index, byte offset, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index, offset).o = value; }
void setGPR(byte index, Integer value, TreatAsInteger) noexcept { getGPR(index).i = value; }
void setGPR(byte index, byte offset, Integer value, TreatAsInteger) noexcept { getGPR(index, offset).i = value; }
Register& getSFR(byte index) noexcept;
Register& getSFR(byte index, byte offset) noexcept;
Ordinal getGPRValue(byte index, TreatAsOrdinal) noexcept { return getGPR(index).o; }
Ordinal getGPRValue(byte index, byte offset, TreatAsOrdinal) noexcept { return getGPR(index, offset).o; }
Integer getGPRValue(byte index, TreatAsInteger) noexcept { return getGPR(index).i; }
Integer getGPRValue(byte index, byte offset, TreatAsInteger) noexcept { return getGPR(index, offset).i; }
Ordinal unpackSrc1_REG(TreatAsOrdinal) noexcept;
Ordinal unpackSrc1_REG(byte offset, TreatAsOrdinal) noexcept;
Integer unpackSrc1_REG(TreatAsInteger) noexcept;
Ordinal unpackSrc2_REG(TreatAsOrdinal) noexcept;
Integer unpackSrc2_REG(TreatAsInteger) noexcept;
void checkForPendingInterrupts() noexcept;
template<typename T>
void moveGPR(byte destIndex, byte srcIndex, TreatAs<T>) noexcept {
    setGPR(destIndex, getGPRValue(srcIndex, TreatAs<T>{}), TreatAs<T>{});
}
template<typename T>
void moveGPR(byte destIndex, byte destOffset, byte srcIndex, byte srcOffset, TreatAs<T>) noexcept {
    setGPR(destIndex, destOffset, getGPRValue(srcIndex, srcOffset, TreatAs<T>{}), TreatAs<T>{});
}
[[nodiscard]] constexpr Ordinal rotateOperation(Ordinal src, Ordinal length) noexcept;
void scanbyte(Ordinal src2, Ordinal src1) noexcept;
void atadd(Register& dest, Ordinal src1, Ordinal src2) noexcept;
void atmod(Register& dest, Ordinal src1, Ordinal src2) noexcept;
int emul(Register& dest, Ordinal src1, Ordinal src2) noexcept;
int ediv(Register& dest, Ordinal src1, Ordinal src2) noexcept;
void arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) noexcept;
Ordinal getSystemProcedureTableBase() noexcept;
Ordinal 
unpackSrc1_COBR(TreatAsOrdinal) noexcept {
    if (instruction.cobr.m1) {
        // treat src1 as a literal
        return instruction.cobr.src1;
    } else {
        return getGPRValue(instruction.cobr.src1, TreatAsOrdinal{});
    }
}
Ordinal
unpackSrc2_COBR(TreatAsOrdinal) noexcept {
    if (instruction.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction.cobr.src2).o;
    } else {
        return getGPRValue(instruction.cobr.src2, TreatAsOrdinal{});
    }
}
Integer
unpackSrc1_COBR(TreatAsInteger) noexcept {
    if (instruction.cobr.m1) {
        // treat src1 as a literal
        return instruction.cobr.src1;
    } else {
        return getGPRValue(instruction.cobr.src1, TreatAsInteger{});
    }
}
Integer
unpackSrc2_COBR(TreatAsInteger) noexcept {
    if (instruction.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction.cobr.src2).i;
    } else {
        return getGPRValue(instruction.cobr.src2, TreatAsInteger{});
    }
}
Register& getSFR(byte index) noexcept {
    return sfrs[index];
}
Register& getSFR(byte index, byte offset) noexcept {
    return getSFR((index + offset) & 0b11111);
}
void
syncf() noexcept {
    /// @todo implement
}
void
flushreg() noexcept {
    /// @todo implement if it makes sense
}
void
mark() noexcept {
    /// @todo implement
}
void
fmark() noexcept {

    /// @todo implement
}
void restoreRegisterSet(Ordinal fp) noexcept;
void
restoreStandardFrame() noexcept {
    // need to leave the current call
    moveGPR(FPIndex, PFPIndex, TreatAsOrdinal{});
    // remember that the lowest 6 bits are ignored so it is important to mask
    // them out of the frame pointer address when using the address
    auto realAddress = getGPRValue(FPIndex, TreatAsOrdinal{}) & NotC;
    restoreRegisterSet(realAddress);
    ip.o = getGPRValue(RIPIndex, TreatAsOrdinal{});
    advanceBy = 0;
}
void
ret() {
    syncf();
    auto& pfp = getGPR(PFPIndex);
    switch (pfp.pfp.rt) {
        case 0b000: 
            restoreStandardFrame();
            break;
        case 0b001:
            {
                auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
                auto x = load(fp - 16, TreatAsOrdinal{});
                auto y = load(fp - 12, TreatAsOrdinal{});
                restoreStandardFrame();
                ac.o = y;
                if (pc.inSupervisorMode()) {
                    pc.o = x;
                }
                break;
            }
        case 0b010: 
            if (pc.inSupervisorMode()) {
                pc.pc.traceEnable = 0;
                pc.pc.executionMode = 0;
            }
            restoreStandardFrame();
            break;
        case 0b011: 
            if (pc.inSupervisorMode()) {
                pc.pc.traceEnable = 1;
                pc.pc.executionMode = 0;
            }
            restoreStandardFrame();
            break;
        case 0b111: 
            {
                // interrupt return
                auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
                auto x = load(fp - 16, TreatAsOrdinal{});
                auto y = load(fp - 12, TreatAsOrdinal{});
                restoreStandardFrame();
                ac.o = y;
                if (pc.inSupervisorMode()) {
                    pc.o = x;
                    checkForPendingInterrupts();
                }
                break;
            }

            break;
        default: 
            // undefined!
            /// @todo raise a fault?
            break;
    }
}
constexpr Ordinal 
computeBitPosition(Ordinal value) noexcept {
    return static_cast<Ordinal>(1u) << static_cast<Ordinal>(value);
}

template<bool checkClear>
void 
branchIfBitGeneric() {
    Ordinal bitpos = computeBitPosition(unpackSrc1_COBR(TreatAsOrdinal{}));
    Ordinal against = unpackSrc2_COBR(TreatAsOrdinal{});
    bool condition = false;
    // Branch if bit set
    if constexpr (checkClear) {
        condition = (bitpos & against) == 0;
    } else {
        condition = (bitpos & against) != 0;
    }
    if (condition) {
        ac.arith.conditionCode = checkClear ? 0b000 : 0b010;
        Register temp;
        temp.alignedTransfer.important = instruction.cobr.displacement;
        ip.alignedTransfer.important = ip.alignedTransfer.important + temp.alignedTransfer.important;
        ip.alignedTransfer.aligned = 0;
        advanceBy = 0;
    } else {
        ac.arith.conditionCode = checkClear ? 0b010 : 0b000;
    }
}
void
bbc() {
    branchIfBitGeneric<true>();
}
void
bbs() {
    branchIfBitGeneric<false>();
}
Ordinal
computeAddress() noexcept {
    if (instruction.isMEMA()) {
        Ordinal result = instruction.mem.offset;
        if (instruction.mema.action) {
            result += getGPRValue(instruction.mem.abase, TreatAsOrdinal{});
        }
        return result;
    } else {
        // okay so we need to figure out the minor mode after figuring out if
        // it is a double wide operation or not
        if (instruction.memb.group) {
            // okay so it is going to be the displacement versions
            // load 32-bits into the optionalDisplacement field
            advanceBy += 4;
            Integer result = static_cast<Integer>(load(ip.a + 4, TreatAsOrdinal{})); // load the optional displacement
            if (instruction.memb_grp2.useIndex) {
                result += (getGPRValue(instruction.memb_grp2.index, TreatAsInteger{}) << static_cast<Integer>(instruction.memb_grp2.scale));
            }
            if (instruction.memb_grp2.registerIndirect) {
                result += getGPRValue(instruction.memb_grp2.abase, TreatAsInteger{});
            }
            return static_cast<Ordinal>(result);
        } else {
            // okay so the other group isn't as cleanly designed
            switch (instruction.memb.modeMinor) {
                case 0b00: // Register Indirect
                    return getGPRValue(instruction.memb.abase, TreatAsOrdinal{});
                case 0b01: // IP With Displacement 
                    advanceBy += 4;
                    return static_cast<Ordinal>(ip.i + load(ip.a + 4, TreatAsInteger{}) + 8);
                case 0b11: // Register Indirect With Index
                    return getGPRValue(instruction.memb.abase, TreatAsOrdinal{}) + (getGPRValue(instruction.memb.index, TreatAsOrdinal{}) << instruction.memb.scale);
                default:
                    return -1;
            }
        }
    }
}
template<typename T>
void
cmpGeneric(T src1, T src2) noexcept {
    if (src1 < src2) {
        ac.arith.conditionCode = 0b100;
    } else if (src1 == src2) {
        ac.arith.conditionCode = 0b010;
    } else {
        ac.arith.conditionCode = 0b001;
    }
}
template<typename T>
void
cmpxbGeneric() noexcept {
    auto src1 = unpackSrc1_COBR(TreatAs<T>{});
    auto src2 = unpackSrc2_COBR(TreatAs<T>{});
    cmpGeneric(src1, src2);
    if ((instruction.instGeneric.mask & ac.arith.conditionCode) != 0) {
        Register temp;
        temp.alignedTransfer.important = instruction.cobr.displacement;
        ip.alignedTransfer.important = ip.alignedTransfer.important + temp.alignedTransfer.important;
        ip.alignedTransfer.aligned = 0;
        advanceBy = 0;
    }
}
void 
cmpobGeneric() noexcept {
    cmpxbGeneric<Ordinal>();
}
void
cmpibGeneric() noexcept {
    cmpxbGeneric<Integer>();
}
void
storeBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept {
    for (byte i = 0; i < count; ++i, baseAddress += 4) {
        store(baseAddress, getGPRValue(baseRegister, i, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
void
loadBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept {
    for (byte i = 0; i < count; ++i, baseAddress += 4) {
        setGPR(baseRegister, i, load(baseAddress, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
int
ldl() noexcept {
    if ((instruction.mem.srcDest & 0b1) != 0) {
        /// @todo fix
        return 130;
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 2);
        // support unaligned accesses
        return 0;
    }
}

int
stl() noexcept {
    if ((instruction.mem.srcDest & 0b1) != 0) {
        /// @todo fix
        return 130;
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 2);
        // support unaligned accesses
        return 0;
    }
}
int
ldt() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        /// @todo fix
        return 130;
        /// @note the hx manual shows that destination is modified o_O
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 3);
        // support unaligned accesses
        return 0;
    }
}

int
stt() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        /// @todo fix
        return 130;
        /// @note the hx manual shows that destination is modified o_O
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 3);
        // support unaligned accesses
        return 0;
    }
}

int 
ldq() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        /// @todo fix
        return 130;
        /// @note the hx manual shows that destination is modified o_O
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 4);
        // support unaligned accesses
        return 0;
    }
}

int
stq() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        /// @todo fix
        return 130;
        /// @note the hx manual shows that destination is modified o_O
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 4);
        // support unaligned accesses
        return 0;
    }
}

void
balx() noexcept {
    auto address = computeAddress();
    setGPR(instruction.mem.srcDest, ip.o + advanceBy, TreatAsOrdinal{});
    ip.o = address;
    advanceBy = 0;
}
bool 
registerSetAvailable() noexcept {
    /// @todo implement this properly when we implement support for register
    /// sets
    return false;
}
void
allocateNewRegisterFrame() noexcept {
    // do nothing right now
}
void 
saveRegisterSet(Ordinal fp) noexcept {
    // save the "next" register frame to main memory to reclaim it
    storeBlock(fp, 16, 16);
}
void
restoreRegisterSet(Ordinal fp) noexcept {
    loadBlock(fp, 16, 16);
}
void 
enterCall(Ordinal fp) noexcept {
    if (registerSetAvailable()) {
        allocateNewRegisterFrame();
    } else {
        saveRegisterSet(fp);
        allocateNewRegisterFrame();
    }
}
void
callx() noexcept {
    // wait for any uncompleted instructions to finish
    auto temp = (getGPRValue(SPIndex, TreatAsOrdinal{}) + C) & NotC; // round stack pointer to next boundary
    auto addr = computeAddress();
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    setGPR(RIPIndex, ip.o + advanceBy, TreatAsOrdinal{});
    enterCall(fp);
    ip.o = addr;
    setGPR(PFPIndex, fp, TreatAsOrdinal{});
    setGPR(FPIndex, temp, TreatAsOrdinal{});
    setGPR(SPIndex , temp + 64, TreatAsOrdinal{});
    advanceBy = 0;
}
void 
call() {
    // wait for any uncompleted instructions to finish
    auto temp = (getGPRValue(SPIndex, TreatAsOrdinal{}) + C) & NotC; // round stack pointer to next boundary
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    setGPR(RIPIndex, ip.o + advanceBy, TreatAsOrdinal{});
    enterCall(fp);
    ip.i += instruction.ctrl.displacement;
    setGPR(PFPIndex, fp, TreatAsOrdinal{});
    setGPR(FPIndex, temp, TreatAsOrdinal{});
    setGPR(SPIndex , temp + 64, TreatAsOrdinal{});
    advanceBy = 0;
}
Ordinal getSupervisorStackPointer() noexcept;
int
calls(Ordinal src1) noexcept {
    if (auto targ = src1; targ > 259) {
        return 0xFB; // protection length fault
    } else {
        syncf();
        auto tempPE = load(getSystemProcedureTableBase() + 48 + (4 * targ), TreatAsOrdinal{});
        auto type = tempPE & 0b11;
        auto procedureAddress = tempPE & ~0b11;
        // read entry from system-procedure table, where spbase is address of
        // system-procedure table from Initial Memory Image
        setGPR(RIPIndex, ip.o + advanceBy, TreatAsOrdinal{});
        ip.o = procedureAddress;
        Ordinal temp = 0, tempRRR = 0;
        if ((type == 0b00) || pc.inSupervisorMode()) {
            temp = (getGPRValue(SPIndex, TreatAsOrdinal{}) + C) & NotC;
            tempRRR = 0;
        } else {
            temp = getSupervisorStackPointer();
            tempRRR = 0b010 | (pc.pc.traceEnable ? 0b001 : 0);
            pc.pc.executionMode = 1;
            pc.pc.traceEnable = temp & 0b1;
        }
        enterCall(temp);
        /// @todo expand pfp and fp to accurately model how this works
        auto& pfp = getGPR(PFPIndex);
        // lowest six bits are ignored
        pfp.o = getGPRValue(FPIndex, TreatAsOrdinal{}) & ~0b1'111;
        pfp.pfp.rt = tempRRR;
        setGPR(FPIndex, temp, TreatAsOrdinal{});
        setGPR(SPIndex, temp + 64, TreatAsOrdinal{});
        advanceBy = 0;
        return 0;
    }
}
void
bx() noexcept {
    ip.o = computeAddress();
    advanceBy = 0;
}


void 
setup() {
    pinMode(LOCKPIN, OUTPUT);
    digitalWrite(LOCKPIN, LOW);
    pinMode(LOCKPIN, INPUT);
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
        getGPR(i).clear();
    }
}
int
performRegisterTransfer(byte mask, byte count) noexcept {
    auto result = 0;
    if (((instruction.reg.srcDest & mask) != 0) || ((instruction.reg.src1 & mask) != 0)) {
        /// @todo fix
        result = 0xFE; // operation.invalid operation
    }
    for (byte i = 0; i < count; ++i) {
        setGPR(instruction.reg.srcDest, i, unpackSrc1_REG(i, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
    return result;
}
constexpr Ordinal modify(Ordinal mask, Ordinal src, Ordinal srcDest) noexcept;
Ordinal 
Register::modify(Ordinal mask, Ordinal src) noexcept {
    auto tmp = o;
    o = ::modify(mask, src, o);
    return tmp;
}
void 
loop() {
    advanceBy = 4;
    int faultCode = 0;
    instruction.o = load(ip.a, TreatAsOrdinal{});
    auto& regDest = getGPR(instruction.reg.srcDest);
    auto src2o = unpackSrc2_REG(TreatAsOrdinal{});
    auto src2i = unpackSrc2_REG(TreatAsInteger{});
    auto src1o = unpackSrc1_REG(TreatAsOrdinal{});
    auto src1i = unpackSrc1_REG(TreatAsInteger{});
    auto invertResult = false;
    auto invertSrc1 = false;
    auto invertSrc2 = false;
    auto doXor = false;
    auto doOr = false;
    auto doAnd = false;
    auto ordinalOp = false;
    auto integerOp = false;
    auto performIncrement = false;
    auto performDecrement = false;
    auto performAdd = false;
    auto performSubtract = false;
    auto performCarry = false;
    auto performCompare = false;
    auto makeSrc1Negative = false;
    auto performLogical = false;
    auto src1IsBitPosition = false;
    auto performMultiply = false;
    auto performDivide = false;
    auto performRemainder = false;
    
    switch (instruction.getOpcode()) {
        case Opcodes::bal: // bal
            setGPR(LRIndex, ip.o + 4, TreatAsOrdinal{});
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
        case Opcodes::bno:
            if (ac.arith.conditionCode == 0) {
                ip.i += instruction.ctrl.displacement;
                advanceBy = 0;
            }
            break;
        case Opcodes::be:
        case Opcodes::bne:
        case Opcodes::bl:
        case Opcodes::ble:
        case Opcodes::bg:
        case Opcodes::bge:
        case Opcodes::bo:
            // the branch instructions have the mask encoded into the opcode
            // itself so we can just use it and save a ton of space overall
            if ((ac.arith.conditionCode & instruction.instGeneric.mask) != 0) {
                ip.i += instruction.ctrl.displacement;
                advanceBy = 0;
            }
            break;
        case Opcodes::faultno:
            if (ac.arith.conditionCode == 0) {
                /// @todo fix fault mechanism to be correct and use proper
                /// codes as well
                faultCode = 129;
            }
            break;
        case Opcodes::faulte:
        case Opcodes::faultne:
        case Opcodes::faultl:
        case Opcodes::faultle:
        case Opcodes::faultg:
        case Opcodes::faultge:
        case Opcodes::faulto:
            if ((ac.arith.conditionCode & instruction.instGeneric.mask) != 0) {
                /// @todo fix fault mechanism to be correct and use proper
                /// codes as well
                faultCode = 129; 
            }
            break;
        case Opcodes::testno:
            setGPR(instruction.cobr.src1, ac.arith.conditionCode == 0 ? 1 : 0, TreatAsOrdinal{});
            break;
        case Opcodes::testg:
        case Opcodes::teste:
        case Opcodes::testge:
        case Opcodes::testl:
        case Opcodes::testne:
        case Opcodes::testle:
        case Opcodes::testo:
            setGPR(instruction.cobr.src1, (ac.arith.conditionCode & instruction.instGeneric.mask) != 0 ? 1 : 0, TreatAsOrdinal{});
            break;
        case Opcodes::ld: 
            loadBlock(computeAddress(), instruction.mem.srcDest, 1);
            break;
        case Opcodes::st: 
            storeBlock(computeAddress(), instruction.mem.srcDest, 1);
            break;
        case Opcodes::lda:
            setGPR(instruction.mem.srcDest, computeAddress(), TreatAsOrdinal{});
            break;
        case Opcodes::bbc:
            bbc();
            break;
        case Opcodes::bbs:
            bbs();
            break;
        case Opcodes::cmpobg:
        case Opcodes::cmpobe:
        case Opcodes::cmpobge:
        case Opcodes::cmpobl:
        case Opcodes::cmpobne:
        case Opcodes::cmpoble:
            cmpobGeneric();
            break;
        case Opcodes::cmpibno: // never branches
        case Opcodes::cmpibg:
        case Opcodes::cmpibe:
        case Opcodes::cmpibge:
        case Opcodes::cmpibl:
        case Opcodes::cmpibne:
        case Opcodes::cmpible:
        case Opcodes::cmpibo: // always branches
            cmpibGeneric();
            break;
        case Opcodes::ldob:
            setGPR(instruction.mem.srcDest, load(computeAddress(), TreatAs<ByteOrdinal>{}), TreatAsOrdinal{});
            break;
        case Opcodes::stob:
            store<ByteOrdinal>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAs<Ordinal>{}), TreatAs<ByteOrdinal>{});
            break;
        case Opcodes::ldos:
            setGPR(instruction.mem.srcDest, load(computeAddress(), TreatAs<ShortOrdinal>{}), TreatAsOrdinal{});
            break;
        case Opcodes::stos:
            store<ShortOrdinal>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAsOrdinal{}), TreatAs<ShortOrdinal>{});
            break;
        case Opcodes::ldib:
            setGPR(instruction.mem.srcDest, load(computeAddress(), TreatAs<ByteInteger>{}), TreatAsInteger{});
            break;
        case Opcodes::stib:
            store<ByteInteger>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAsInteger{}), TreatAs<ByteInteger>{});
            break;
        case Opcodes::ldis:
            setGPR(instruction.mem.srcDest, load(computeAddress(), TreatAs<ShortInteger>{}), TreatAsInteger{});
            break;
        case Opcodes::stis:
            store<ShortInteger>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAsInteger{}), TreatAs<ShortInteger>{});
            break;
        case Opcodes::ldl:
            faultCode = ldl();
            break;
        case Opcodes::stl:
            faultCode = stl();
            break;
        case Opcodes::ldt:
            faultCode = ldt();
            break;
        case Opcodes::stt:
            faultCode = stt();
            break;
        case Opcodes::ldq:
            faultCode = ldq();
            break;
        case Opcodes::stq:
            faultCode = stq();
            break;
        case Opcodes::bx:
            bx();
            break;
        case Opcodes::balx:
            balx();
            break;
        case Opcodes::callx:
            callx();
            break;
    // in some of the opcodeExt values seem to reflect the resultant truth
    // table for the operation :). That's pretty cool
        case Opcodes::nand: // nand
            invertResult = true;
        case Opcodes::andOperation: // and
            performLogical = true;
            doAnd = true;
            break;
        case Opcodes::clrbit: // clrbit
                              // clrbit is src2 & ~computeBitPosition(src1)
                              // so lets use andnot
            src1IsBitPosition = true;
        case Opcodes::andnot: // andnot
            doAnd = true;
            invertSrc1 = true;
            performLogical = true;
            break;
        case Opcodes::notand: // notand
            doAnd = true;
            invertSrc2 = true;
            performLogical = true;
            break;
        case Opcodes::notbit: // notbit
                     // notbit is src2 ^ computeBitPosition(src1)
            src1IsBitPosition = true;
        case Opcodes::xorOperation: // xor
            doXor = true;
            performLogical = true;
            break;
        case Opcodes::setbit: // setbit
                     // setbit is src2 | computeBitPosition(src1o)
            src1IsBitPosition = true;
        case Opcodes::orOperation: // or
            doOr = true;
            performLogical = true;
            break;
        case Opcodes::nor: // nor
            doOr = true;
            invertResult = true;
            performLogical = true;
            break;
        case Opcodes::xnor: // xnor
            doXor = true;
            invertResult = true;
            performLogical = true;
            break;
        case Opcodes::notOperation: // not 
                     // perform fallthrough to ornot with src2 set to zero
            src2o = 0;
        case Opcodes::ornot: // ornot
            doOr = true;
            invertSrc1 = true;
            performLogical = true;
            break;
        case Opcodes::notor: // notor
            doOr = true;
            invertSrc2 = true;
            performLogical = true;
            break;
        case Opcodes::alterbit: // alterbit
            src1IsBitPosition = true;
            performLogical = true;
            if (ac.arith.conditionCode & 0b010) {
                doOr = true;
            } else {
                doAnd = true;
                invertSrc1 = true;
            }
            break;
        case Opcodes::addo: // addo
            performAdd = true;
            ordinalOp = true;
            break;
        case Opcodes::addi: // addi
            performAdd = true;
            integerOp = true;
            break;
        case Opcodes::subo: // subo
            // I remember this trick from college, subtraction is just addition
            // with a negative second argument :). I never gave it much thought
            // until now but it seems to be an effective trick to save space.
            performAdd = true;
            makeSrc1Negative = true;
            ordinalOp = true;
            break;
        case Opcodes::subi: // subi
            performAdd = true;
            makeSrc1Negative = true;
            integerOp = true;
            break;
        case Opcodes::shro: // shro
            regDest.o = src1o < 32 ? src2o >> src1o : 0;
            break;
        case Opcodes::shrdi: // shrdi
                  // according to the manual, equivalent to divi value, 2 so that is what we're going to do for correctness sake
            regDest.i = src1i < 32 && src1i >= 0 ? src2i / computeBitPosition(src1i) : 0;
            break;
        case Opcodes::shri: // shri
            /*
             * if (src >= 0) {
             *  if (len < 32) {
             *      dest <- src/2^len
             *  } else {
             *      dest <- 0
             *  }
             * }else {
             *  if (len < 32) {
             *      dest <- (src - 2^len + 1)/2^len;
             *  } else {
             *      dest <- -1;
             *   }
             * }
             *
             */
            /// @todo perhaps implement the extra logic if necessary
            regDest.i = src2i >> src1i;
            break;
        case Opcodes::shlo: // shlo
            regDest.o = src1o < 32 ? src2o << src1o : 0;
            break;
        case Opcodes::rotate: // rotate
            regDest.o = rotateOperation(src2o, src1o);
            break;
        case Opcodes::shli: // shli
            regDest.i = src2i << src1i;
            break;
        case Opcodes::cmpo: // cmpo
            performCompare = true;
            ordinalOp = true;
            break;
        case Opcodes::cmpi: // cmpi
            performCompare = true;
            integerOp = true;
            break;
        case Opcodes::concmpo: // concmpo
            if ((ac.arith.conditionCode & 0b100) == 0) {
                ac.arith.conditionCode = src1o <= src2o ? 0b010 : 0b001;
            }
            break;
        case Opcodes::concmpi: // concmpi
            if ((ac.arith.conditionCode & 0b100) == 0) {
                ac.arith.conditionCode = src1i <= src2i ? 0b010 : 0b001;
            }
            break;
        case Opcodes::cmpinco: // cmpinco
            performCompare = true;
            ordinalOp = true;
            performIncrement = true;
            break;
        case Opcodes::cmpinci: // cmpinci
            performCompare = true;
            integerOp = true;
            performIncrement = true;
            break;
        case Opcodes::cmpdeco: // cmpdeco
            performCompare = true;
            ordinalOp = true;
            performDecrement = true;
            break;
        case Opcodes::cmpdeci: // cmpdeci
            performCompare = true;
            integerOp = true;
            performDecrement = true;
            break;
        case Opcodes::scanbyte: // scanbyte
            scanbyte(src2o, src1o);
            break;
        case Opcodes::chkbit: // chkbit
            ac.arith.conditionCode = ((src2o & computeBitPosition(src1o)) == 0 ? 0b000 : 0b010);
            break;
        case Opcodes::addc: 
            performAdd = true;
            performCarry = true;
            break;
        case Opcodes::subc:
            performSubtract = true;
            performCarry = true;
            break;
        case Opcodes::mov:
            regDest.o = src1o;
            break;
        case Opcodes::movl:
            faultCode = performRegisterTransfer(0b1, 2);
            break;
        case Opcodes::movt:
            faultCode = performRegisterTransfer(0b11, 3);
            break;
        case Opcodes::movq:
            faultCode = performRegisterTransfer(0b11, 4);
            break;
        case Opcodes::syncf:
            syncf();
            break;
        case Opcodes::flushreg:
            flushreg();
            break;
        case Opcodes::fmark:
            fmark();
            break;
        case Opcodes::mark:
            mark();
            break;
        case Opcodes::mulo:
            performMultiply = true;
            ordinalOp = true;
            break;
        case Opcodes::muli:
            performMultiply = true;
            integerOp = true;
            break;
        case Opcodes::divi:
            performDivide = true;
            integerOp = true;
            break;
        case Opcodes::divo:
            performDivide = true;
            ordinalOp = true;
            break;
        case Opcodes::remo:
            performRemainder = true;
            ordinalOp = true;
            break;
        case Opcodes::remi:
            performRemainder = true;
            integerOp = true;
            break;
        case Opcodes::modi: 
            if (auto denominator = src1i; denominator == 0) {
                faultCode = 0xFC; // divide by zero
            } else {
                auto numerator = src2i;
                auto result = numerator - ((numerator / denominator) * denominator);
                if (((numerator * denominator) < 0) && (result != 0)) {
                    result += denominator;
                }
                regDest.i = result;
            }
            break;
        case Opcodes::modify:
            regDest.o = modify(src1o, src2o, regDest.o);
            break;
        case Opcodes::extract:
            // taken from the Hx manual as it isn't insane
            regDest.o = (regDest.o >> (src1o > 32 ? 32 : src1o)) & ~(0xFFFF'FFFF << src2o);
            break;
        case Opcodes::modac: 
            regDest.o = ac.modify(src1o, src2o); 
            break;
        case Opcodes::modtc: 
            regDest.o = tc.modify(src1o, src2o); 
            break;
        case Opcodes::modpc:
            if (auto mask = src1o; mask != 0) {
                if (!pc.inSupervisorMode()) {
                    /// @todo fix
                    faultCode = 0xFB; // type mismatch
                } else {
                    regDest.o = pc.modify(mask, src2o);
                    if (regDest.getPriority() > pc.getPriority()) {
                        checkForPendingInterrupts();
                    }
                }
            } else {
                regDest.o = pc.o;
            }
            break;
        case Opcodes::atadd:
            atadd(regDest, src1o, src2o);
            break;
        case Opcodes::atmod:
            atmod(regDest, src1o, src2o);
            break;
        case Opcodes::emul:
            faultCode = emul(regDest, src2o, src1o);
            break;
        case Opcodes::ediv:
            faultCode = ediv(regDest, src2o, src1o);
            break;
        case Opcodes::calls:
            faultCode = calls(src1o);
            break;


#if 0
        default:
            /// @todo implement properly
            faultCode = 0xFD;
            break;
#endif
    }
    if (performLogical) {
        if (src1IsBitPosition) {
            src1o = computeBitPosition(src1o);
        }
        if (invertSrc1) {
            src1o = ~src1o;
        }
        if (invertSrc2) {
            src2o = ~src2o;
        }
        if (doAnd) {
            regDest.o = src2o & src1o;
        } else if (doXor) {
            regDest.o = src2o ^ src1o;
        } else if (doOr) {
            regDest.o = src2o | src1o;
        }
        if (invertResult) {
            regDest.o = ~regDest.o;
        }
    }
    if (performCompare) {
        if (ordinalOp) {
            cmpGeneric(src1o, src2o);
            if (performIncrement) {
                regDest.o = src2o + 1;
            }
            if (performDecrement) {
                regDest.o = src2o - 1;
            }
        } else if (integerOp) {
            cmpGeneric(src1i, src2i);
            if (performIncrement) {
                regDest.i = src2i + 1;
            }
            if (performDecrement) {
                regDest.i = src2i - 1;
            }
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            faultCode = 0xFF; // invalid opcode/operation
        }
    }
    if (performCarry) {
        LongOrdinal result = 0;
        if (performAdd) {
            result = static_cast<LongOrdinal>(src2o) + static_cast<LongOrdinal>(src1o);
        } else if (performSubtract) {
            result = static_cast<LongOrdinal>(src2o) - static_cast<LongOrdinal>(src1o) - 1;
        }
        result += (ac.getCarryBit() ? 1 : 0);
        regDest.o = static_cast<Ordinal>(result);
        arithmeticWithCarryGeneric(static_cast<Ordinal>(result >> 32), 
                (src2o & 0x8000'0000), 
                (src1o & 0x8000'0000), 
                (regDest.o & 0x8000'0000));
    } else if (performAdd) {
        if (ordinalOp) {
            if (makeSrc1Negative) {
                src1o = -src1o;
            }
            regDest.o = src2o + src1o;
        } else if (integerOp) {
            if (makeSrc1Negative) {
                src1i = -src1i;
            }
            regDest.i = src2i + src1i;
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            faultCode = 0xFF; // invalid opcode
        }
    } else if (performMultiply) {
        if (ordinalOp) {
            regDest.o = src2o * src1o;
        } else if (integerOp) {
            regDest.i = src2i * src1i;
        }
    } else if (performDivide) {
        if (ordinalOp) {
            if (src1o == 0) {
                /// @todo fix this
                faultCode = 0xFC; // divide by zero
            } else {
                regDest.o = src2o / src1o;
            }
        } else if (integerOp) {
            if (src1i == 0) {
                /// @todo fix this
                faultCode = 0xFC; // divide by zero
            } else {
                regDest.i = src2i / src1i;
            }
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            faultCode = 0xFF; // invalid opcode
        }
    } else if (performRemainder) {
        if (ordinalOp) {
            if (src1o == 0) {
                /// @todo fix this
                faultCode = 0xFC; // divide by zero
            } else {
                // taken from the i960Sx manual
                //dest.setOrdinal(src2 - ((src2 / src1) * src1));
                regDest.o = src2o % src1o;
            }
        } else if (integerOp) {
            if (src1i == 0) {
                /// @todo fix this
                faultCode = 0xFC; // divide by zero
            } else {
                // taken from the i960Sx manual
                //dest.setInteger(src2 - ((src2 / src1) * src1));
                regDest.i = src2i % src1i;
            }
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            faultCode = 0xFF; // invalid opcode
        }
    }

    if (faultCode) {
        /// @todo implement this as the fallback operation when something bad
        /// happens
        ///
        /// Faults are basically fallback behavior when something goes wacky!
        configRegs().faultPort = faultCode;
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
Ordinal 
unpackSrc1_REG(TreatAsOrdinal) noexcept {
    if (instruction.reg.m1) {
        /// @todo what to do if s1 is also set?
        return instruction.reg.src1;
    } else if (instruction.reg.s1) {
        return getSFR(instruction.reg.src1).o;
    } else {
        return getGPRValue(instruction.reg.src1, TreatAsOrdinal{});
    }
}
Ordinal 
unpackSrc1_REG(byte offset, TreatAsOrdinal) noexcept {
    if (instruction.reg.m1) {
        // literals should always return zero if offset is greater than zero
        return offset == 0 ? instruction.reg.src1 : 0;
    } else if (instruction.reg.s1) {
        return getSFR(instruction.reg.src1, offset).o;
    } else {
        return getGPRValue(instruction.reg.src1, offset, TreatAsOrdinal{});
    }
}
Integer 
unpackSrc1_REG(TreatAsInteger) noexcept {
    if (instruction.reg.m1) {
        return instruction.reg.src1;
    } else if (instruction.reg.s1) {
        return getSFR(instruction.reg.src1).i;
    } else {
        return getGPRValue(instruction.reg.src1, TreatAsInteger{});
    }
}
Ordinal 
unpackSrc2_REG(TreatAsOrdinal) noexcept {
    if (instruction.reg.m2) {
        return instruction.reg.src2;
    } else if (instruction.reg.s2) {
        return getSFR(instruction.reg.src2).o;
    } else {
        return getGPRValue(instruction.reg.src2, TreatAsOrdinal{});
    }
}
Integer 
unpackSrc2_REG(TreatAsInteger) noexcept {
    if (instruction.reg.m2) {
        return instruction.reg.src2;
    } else if (instruction.reg.s2) {
        return getSFR(instruction.reg.src2).i;
    } else {
        return getGPRValue(instruction.reg.src2, TreatAsInteger{});
    }
}

constexpr Ordinal rotateOperation(Ordinal src, Ordinal length) noexcept {
    return (src << length)  | (src >> ((-length) & 31u));
}
void
scanbyte(Ordinal src2, Ordinal src1) noexcept {
    Register s2(src2);
    Register s1(src1);
    for (int i = 0; i < 4; ++i) {
        if (s1.bytes[i] == s2.bytes[i]) {
            ac.arith.conditionCode = 0b010;
            return;
        }
    }
    ac.arith.conditionCode = 0;
}
void
arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) noexcept {
    // set the carry bit
    ac.arith.conditionCode = 0;
    // set the overflow bit
    if ((src2MSB == src1MSB) && (src2MSB != destMSB)) {
        ac.arith.conditionCode |= 0b001;
    } else {
        ac.arith.conditionCode &= 0b110;
    }
    if (result != 0) {
        ac.arith.conditionCode |= 0b010;
    } else {
        ac.arith.conditionCode &= 0b101;
    }
}

void
checkForPendingInterrupts() noexcept {
    /// @todo implement
}

void
atadd(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    // adds the src (src2 internally) value to the value in memory location specified with the addr (src1 in this case) operand.
    // The initial value from memory is stored in dst (internally src/dst).
    syncf();
    lockBus();
    auto addr = src1 & 0xFFFF'FFFC;
    auto temp = load(addr, TreatAsOrdinal{});
    store(addr, temp + src2, TreatAsOrdinal{});
    dest.o = temp;
    unlockBus();
}

constexpr Ordinal modify(Ordinal mask, Ordinal src, Ordinal srcDest) noexcept {
    return (src & mask) | (srcDest & ~mask);
}
void
atmod(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    // copies the src/dest value (logical version) into the memory location specifeid by src1.
    // The bits set in the mask (src2) operand select the bits to be modified in memory. The initial
    // value from memory is stored in src/dest
    syncf();
    lockBus();
    auto addr = src1 & 0xFFFF'FFFC;
    auto temp = load(addr, TreatAsOrdinal{});
    store(addr, modify(src2, dest.o, temp), TreatAsOrdinal{});
    dest.o = temp;
    unlockBus();
}

int
emul(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    union {
        LongOrdinal lord;
        Ordinal parts[sizeof(LongOrdinal)/sizeof(Ordinal)];
    } result;
    int faultCode = 0;
    if ((instruction.reg.srcDest & 0b1) != 0) {
        /// @todo fix
        faultCode = 0xFD; // invalid operation
    }  else {
        result.lord = static_cast<LongOrdinal>(src2) * static_cast<LongOrdinal>(src1);
    }
    // yes this can be undefined by design :)
    // if we hit a fault then we just give up whats on the stack :)
    dest.o = result.parts[0];
    setGPR(instruction.reg.srcDest, 1, result.parts[1], TreatAsOrdinal{});
    return faultCode;
}

int
ediv(Register& dest, Ordinal src1, Ordinal src2Lower) noexcept {
    union {
        LongOrdinal lord;
        Ordinal parts[sizeof(LongOrdinal)/sizeof(Ordinal)];
    } result, src2;
    src2.parts[0] = src2Lower;
    int faultCode = 0;
    if ((instruction.reg.srcDest & 0b1) != 0 || (instruction.reg.src2 & 0b1) != 0) {
        /// @todo fix
        faultCode = 0xFD; // invalid operation
    } else if (src1 == 0) {
        // divide by zero
        /// @todo fix
        faultCode = 0xFC; // divide by zero
    } else {
        src2.parts[1] = getGPRValue(instruction.reg.src2, 1, TreatAsOrdinal{});
        result.parts[1] = src2.lord / src1; // quotient
        result.parts[0] = static_cast<Ordinal>(src2.lord - (src2.lord / src1) * src1); // remainder
    }
    // yes this can be undefined by design :)
    // if we hit a fault then we just give whats on the stack :)
    dest.o = result.parts[0];
    setGPR(instruction.reg.srcDest, 1, result.parts[1], TreatAsOrdinal{});
    return faultCode;
}
