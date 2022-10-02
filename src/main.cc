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
    byte faultPort; /// @todo remove this when we do this right, this is here
                    /// to make sure the optimizer doesn't get dumb
};

static_assert(sizeof(ConfigRegisters) <= 256);

volatile ConfigRegisters& 
configRegs() noexcept {
    return memory<ConfigRegisters>(ConfigurationAddress);
}

void
lockBus() noexcept {

}
void
unlockBus() noexcept {

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
Register& getSFR(byte index) noexcept;
Ordinal unpackSrc1_REG(TreatAsOrdinal) noexcept;
Integer unpackSrc1_REG(TreatAsInteger) noexcept;
Ordinal unpackSrc2_REG(TreatAsOrdinal) noexcept;
Integer unpackSrc2_REG(TreatAsInteger) noexcept;
[[nodiscard]] constexpr Ordinal performAndOperation(Ordinal src2, Ordinal src1, bool invertOutput, bool invertSrc2, bool invertSrc1) noexcept;
[[nodiscard]] constexpr Ordinal performOrOperation(Ordinal src2, Ordinal src1, bool invertOutput, bool invertSrc2, bool invertSrc1) noexcept;
[[nodiscard]] constexpr Ordinal performXorOperation(Ordinal src2, Ordinal src1, bool invertOutput) noexcept;
[[nodiscard]] constexpr Ordinal rotateOperation(Ordinal src, Ordinal length) noexcept;
void scanbyte(Ordinal src2, Ordinal src1) noexcept;
void arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) noexcept;
Ordinal 
unpackSrc1_COBR(TreatAsOrdinal) noexcept {
    if (instruction.cobr.m1) {
        // treat src1 as a literal
        return instruction.cobr.src1;
    } else {
        return getGPR(instruction.cobr.src1).o;
    }
}
Ordinal
unpackSrc2_COBR(TreatAsOrdinal) noexcept {
    if (instruction.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction.cobr.src2).o;
    } else {
        return getGPR(instruction.cobr.src2).o;
    }
}
Integer
unpackSrc1_COBR(TreatAsInteger) noexcept {
    if (instruction.cobr.m1) {
        // treat src1 as a literal
        return instruction.cobr.src1;
    } else {
        return getGPR(instruction.cobr.src1).i;
    }
}
Integer
unpackSrc2_COBR(TreatAsInteger) noexcept {
    if (instruction.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction.cobr.src2).i;
    } else {
        return getGPR(instruction.cobr.src2).i;
    }
}
Register& getSFR(byte index) noexcept {
    return sfrs[index & 0b11111];
}
void
raiseFault(byte code) noexcept {
    configRegs().faultPort = code;
}
void
syncf() {
    /// @todo implement
}
void restoreRegisterSet(Ordinal fp) noexcept;
void
restoreStandardFrame() noexcept {
    // need to leave the current call
    getGPR(FPIndex).o = getGPR(PFPIndex).o;
    // remember that the lowest 6 bits are ignored so it is important to mask
    // them out of the frame pointer address when using the address
    auto realAddress = getGPR(FPIndex).o & NotC;
    restoreRegisterSet(realAddress);
    ip.o = getGPR(RIPIndex).o;
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
                auto& fp = getGPR(FPIndex);
                auto x = load(fp.o - 16, TreatAsOrdinal{});
                auto y = load(fp.o - 12, TreatAsOrdinal{});
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
                auto& fp = getGPR(FPIndex);
                auto x = load(fp.o - 16, TreatAsOrdinal{});
                auto y = load(fp.o - 12, TreatAsOrdinal{});
                restoreStandardFrame();
                ac.o = y;
                if (pc.inSupervisorMode()) {
                    pc.o = x;
                    /// @todo check pending interrupts
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
    return static_cast<Ordinal>(1u) << static_cast<Ordinal>(value & 0b11111);
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
        // another lie in the i960Sx manual
        if constexpr (checkClear) {
            ac.arith.conditionCode = 0b000;
        } else {
            ac.arith.conditionCode = 0b010;
        }
        Register temp;
        temp.alignedTransfer.important = instruction.cobr.displacement;
        ip.alignedTransfer.important = ip.alignedTransfer.important + temp.alignedTransfer.important;
        ip.alignedTransfer.aligned = 0;
        advanceBy = 0;
    } else {
        if constexpr (checkClear) {
            ac.arith.conditionCode = 0b010;
        } else {
            ac.arith.conditionCode = 0b000;
        }
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
            result += getGPR(instruction.mem.abase).o;
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
                result += (getGPR(instruction.memb_grp2.index).i << static_cast<Integer>(instruction.memb_grp2.scale));
            }
            if (instruction.memb_grp2.registerIndirect) {
                result += getGPR(instruction.memb_grp2.abase).i;
            }
            return static_cast<Ordinal>(result);
        } else {
            // okay so the other group isn't as cleanly designed
            switch (instruction.memb.modeMinor) {
                case 0b00: // Register Indirect
                    return getGPR(instruction.memb.abase).o;
                case 0b01: // IP With Displacement 
                    advanceBy += 4;
                    return static_cast<Ordinal>(ip.i + load(ip.a + 4, TreatAsInteger{}) + 8);
                case 0b11: // Register Indirect With Index
                    return getGPR(instruction.memb.abase).o + (getGPR(instruction.memb.index).o << instruction.memb.scale);
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
    for (byte i = 0; i < count; ++i, baseAddress += 4, ++baseRegister) {
        store(baseAddress, getGPR(baseRegister).o, TreatAsOrdinal{});
    }
}
void
loadBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept {
    for (byte i = 0; i < count; ++i, baseAddress += 4, ++baseRegister) {
        getGPR(baseRegister).o = load(baseAddress, TreatAsOrdinal{});
    }
}
void
ldl() noexcept {
    if ((instruction.mem.srcDest & 0b1) != 0) {
        /// @todo fix
        raiseFault(130);
        /// @note the hx manual shows that destination is modified o_O
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 2);
        // support unaligned accesses
    }
}

void
stl() noexcept {
    if ((instruction.mem.srcDest & 0b1) != 0) {
        /// @todo fix
        raiseFault(130);
        /// @note the hx manual shows that destination is modified o_O
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 2);
        // support unaligned accesses
    }
}
void
ldt() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        /// @todo fix
        raiseFault(130);
        /// @note the hx manual shows that destination is modified o_O
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 3);
        // support unaligned accesses
    }
}

void
stt() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        /// @todo fix
        raiseFault(130);
        /// @note the hx manual shows that destination is modified o_O
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 3);
        // support unaligned accesses
    }
}

void
ldq() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        /// @todo fix
        raiseFault(130);
        /// @note the hx manual shows that destination is modified o_O
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 4);
        // support unaligned accesses
    }
}

void
stq() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        /// @todo fix
        raiseFault(130);
        /// @note the hx manual shows that destination is modified o_O
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 4);
        // support unaligned accesses
    }
}

void
balx() noexcept {
    auto address = computeAddress();
    getGPR(instruction.mem.srcDest).o = ip.o + advanceBy;
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
    auto temp = (getGPR(SPIndex).o + C) & NotC; // round stack pointer to next boundary
    auto addr = computeAddress();
    auto fp = getGPR(FPIndex).o;
    getGPR(RIPIndex).o = ip.o + advanceBy;
    enterCall(fp);
    ip.o = addr;
    getGPR(PFPIndex).o = fp;
    getGPR(FPIndex).o = temp;
    getGPR(SPIndex).o = temp + 64;
    advanceBy = 0;
}
void 
call() {
    // wait for any uncompleted instructions to finish
    auto temp = (getGPR(SPIndex).o + C) & NotC; // round stack pointer to next boundary
    auto fp = getGPR(FPIndex).o;
    getGPR(RIPIndex).o = ip.o + advanceBy;
    enterCall(fp);
    ip.i += instruction.ctrl.displacement;
    getGPR(PFPIndex).o = fp;
    getGPR(FPIndex).o = temp;
    getGPR(SPIndex).o = temp + 64;
    advanceBy = 0;
}
void
bx() noexcept {
    ip.o = computeAddress();
    advanceBy = 0;
}


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
        getGPR(i).clear();
    }
}

void 
loop() {
    advanceBy = 4;
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
    auto ordinalCompareOp = false;
    auto integerCompareOp = false;
    auto performIncrement = false;
    auto performDecrement = false;
    auto performAdd = false;
    auto performSubtract = false;
    auto performCarry = false;
    switch (instruction.getOpcode()) {
        case Opcodes::bal: // bal
            getGPR(LRIndex).o = ip.o + 4;
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
                raiseFault(129);
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
                raiseFault(129);
            }
            break;
        case Opcodes::testno:
            getGPR(instruction.cobr.src1).o = ac.arith.conditionCode == 0 ? 1 : 0;
            break;
        case Opcodes::testg:
        case Opcodes::teste:
        case Opcodes::testge:
        case Opcodes::testl:
        case Opcodes::testne:
        case Opcodes::testle:
        case Opcodes::testo:
            getGPR(instruction.cobr.src1).o = (ac.arith.conditionCode & instruction.instGeneric.mask) != 0 ? 1 : 0;
            break;
        case Opcodes::ld: 
            loadBlock(computeAddress(), instruction.mem.srcDest, 1);
            break;
        case Opcodes::st: 
            storeBlock(computeAddress(), instruction.mem.srcDest, 1);
            break;
        case Opcodes::lda:
            getGPR(instruction.mem.srcDest).o = computeAddress();
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
            getGPR(instruction.mem.srcDest).o = load(computeAddress(), TreatAs<ByteOrdinal>{});
            break;
        case Opcodes::stob:
            store<ByteOrdinal>(computeAddress(), getGPR(instruction.mem.srcDest).o, TreatAs<ByteOrdinal>{});
            break;
        case Opcodes::ldos:
            getGPR(instruction.mem.srcDest).o = load(computeAddress(), TreatAs<ShortOrdinal>{});
            break;
        case Opcodes::stos:
            store<ShortOrdinal>(computeAddress(), getGPR(instruction.mem.srcDest).o, TreatAs<ShortOrdinal>{});
            break;
        case Opcodes::ldib:
            getGPR(instruction.mem.srcDest).o = load(computeAddress(), TreatAs<ByteInteger>{});
            break;
        case Opcodes::stib:
            store<ByteInteger>(computeAddress(), getGPR(instruction.mem.srcDest).o, TreatAs<ByteInteger>{});
            break;
        case Opcodes::ldis:
            getGPR(instruction.mem.srcDest).o = load(computeAddress(), TreatAs<ShortInteger>{});
            break;
        case Opcodes::stis:
            store<ShortInteger>(computeAddress(), getGPR(instruction.mem.srcDest).o, TreatAs<ShortInteger>{});
            break;
        case Opcodes::ldl:
            ldl();
            break;
        case Opcodes::stl:
            stl();
            break;
        case Opcodes::ldt:
            ldt();
            break;
        case Opcodes::stt:
            stt();
            break;
        case Opcodes::ldq:
            ldq();
            break;
        case Opcodes::stq:
            stq();
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
            doAnd = true;
            break;
        case Opcodes::clrbit: // clrbit
                              // clrbit is src2 & ~computeBitPosition(src1)
                              // so lets use andnot
            src1o = computeBitPosition(src1o);
        case Opcodes::andnot: // andnot
            doAnd = true;
            invertSrc1 = true;
            break;
        case Opcodes::notand: // notand
            doAnd = true;
            invertSrc2 = true;
            break;
        case Opcodes::notbit: // notbit
                     // notbit is src2 ^ computeBitPosition(src1)
            src1o = computeBitPosition(src1o);
        case Opcodes::xorOperation: // xor
            doXor = true;
            break;
        case Opcodes::setbit: // setbit
                     // setbit is src2 | computeBitPosition(src1o)
            src1o = computeBitPosition(src1o);
        case Opcodes::orOperation: // or
            doOr = true;
            break;
        case Opcodes::nor: // nor
            doOr = true;
            invertResult = true;
            break;
        case Opcodes::xnor: // xnor
            doXor = true;
            invertResult = true;
            break;
        case Opcodes::notOperation: // not 
                     // perform fallthrough to ornot with src2 set to zero
            src2o = 0;
        case Opcodes::ornot: // ornot
            doOr = true;
            invertSrc1 = true;
            break;
        case Opcodes::notor: // notor
            doOr = true;
            invertSrc2 = true;
            break;
        case Opcodes::alterbit: // alterbit
            src1o = computeBitPosition(src1o);
            if (ac.arith.conditionCode & 0b010) {
                doOr = true;
            } else {
                doAnd = true;
                invertSrc1 = true;
            }
            break;
        case Opcodes::addo: // addo
            regDest.o = src2o + src1o;
            break;
        case Opcodes::addi: // addi
            regDest.i = src2i + src1i;
            break;
        case Opcodes::subo: // subo
            regDest.o = src2o - src1o;
            break;
        case Opcodes::subi: // subi
            regDest.i = src2i - src1i;
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
            ordinalCompareOp = true;
            break;
        case Opcodes::cmpi: // cmpi
            integerCompareOp = true;
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
            ordinalCompareOp = true;
            performIncrement = true;
            break;
        case Opcodes::cmpinci: // cmpinci
            integerCompareOp = true;
            performIncrement = true;
            break;
        case Opcodes::cmpdeco: // cmpdeco
            ordinalCompareOp = true;
            performDecrement = true;
            break;
        case Opcodes::cmpdeci: // cmpdeci
            integerCompareOp = true;
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
#if 0
        default:
            /// @todo implement properly
            raiseFault(0xFD);
            break;
#endif
    }
    if (doAnd) {
        regDest.o = performAndOperation(src2o, src1o, invertResult, invertSrc1, invertSrc2);
    } else if (doXor) {
        regDest.o = performXorOperation(src2o, src1o, invertResult);
    } else if (doOr) {
        regDest.o = performOrOperation(src2o, src1o, invertResult, invertSrc1, invertSrc2);
    }
    if (ordinalCompareOp) {
        cmpGeneric(src1o, src2o);
        if (performIncrement) {
            regDest.o = src2o + 1;
        }
        if (performDecrement) {
            regDest.o = src2o - 1;
        }
    } else if (integerCompareOp) {
        cmpGeneric(src1i, src2i);
        if (performIncrement) {
            regDest.i = src2i + 1;
        }
        if (performDecrement) {
            regDest.i = src2i - 1;
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
#if 0
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
#endif
Ordinal 
unpackSrc1_REG(TreatAsOrdinal) noexcept {
    if (instruction.reg.m1) {
        if (instruction.reg.s1) {
            // reserved so another complement of registers?
            raiseFault(0xFF); // bad operand
            return 0;
        } else {
            return instruction.reg.src1;
        }
    } else {
        if (instruction.reg.s1) {
            return getSFR(instruction.reg.src1).o;
        } else {
            return getGPR(instruction.reg.src1).o;
        }
    }
}
Integer 
unpackSrc1_REG(TreatAsInteger) noexcept {
    if (instruction.reg.m1) {
        if (instruction.reg.s1) {
            // reserved so another complement of registers?
            raiseFault(0xFF); // bad operand
            return 0;
        } else {
            return instruction.reg.src1;
        }
    } else {
        if (instruction.reg.s1) {
            return getSFR(instruction.reg.src1).i;
        } else {
            return getGPR(instruction.reg.src1).i;
        }
    }
}
Ordinal 
unpackSrc2_REG(TreatAsOrdinal) noexcept {
    if (instruction.reg.m2) {
        if (instruction.reg.s2) {
            // reserved so another complement of registers?
            raiseFault(0xFF); // bad operand
            return 0;
        } else {
            return instruction.reg.src2;
        }
    } else {
        if (instruction.reg.s2) {
            return getSFR(instruction.reg.src2).o;
        } else {
            return getGPR(instruction.reg.src2).o;
        }
    }
}
Integer 
unpackSrc2_REG(TreatAsInteger) noexcept {
    if (instruction.reg.m2) {
        if (instruction.reg.s2) {
            // reserved so another complement of registers?
            raiseFault(0xFF); // bad operand
            return 0;
        } else {
            return instruction.reg.src2;
        }
    } else {
        if (instruction.reg.s2) {
            return getSFR(instruction.reg.src2).i;
        } else {
            return getGPR(instruction.reg.src2).i;
        }
    }
}

[[nodiscard]] constexpr Ordinal performAndOperation(Ordinal src2, Ordinal src1, bool invertOutput, bool invertSrc2, bool invertSrc1) noexcept {
    auto result = (invertSrc2 ? ~src2 : src2) & (invertSrc1 ? ~src1 : src1);
    return invertOutput ? ~result : result;
}
[[nodiscard]] constexpr Ordinal performOrOperation(Ordinal src2, Ordinal src1, bool invertOutput, bool invertSrc2, bool invertSrc1) noexcept {
    auto result = (invertSrc2 ? ~src2 : src2) | (invertSrc1 ? ~src1 : src1);
    return invertOutput ? ~result : result;
}
[[nodiscard]] constexpr Ordinal performXorOperation(Ordinal src2, Ordinal src1, bool invertOutput) noexcept {
    auto result = src2 ^ src1;
    return invertOutput ? ~result : result;
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
