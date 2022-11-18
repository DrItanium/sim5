// sim
// Copyright (c) 2022, Joshua Scoggins
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
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include "Types.h"
#include "BinaryOperations.h"
constexpr size_t ConfigurationAddress = 0x7F00;
constexpr Ordinal SALIGN = 4;
constexpr Ordinal C = (SALIGN * 16) - 1;
constexpr Ordinal NotC = ~C;
/// faults
constexpr Ordinal NoFault = 0xFFFF'FFFF;
constexpr Ordinal ParallelFault = 0;
constexpr Ordinal TraceFaultBase = 0x00010000;
constexpr Ordinal InstructionTraceFault = TraceFaultBase | 0b00000010;
constexpr Ordinal BranchTraceFault = TraceFaultBase      | 0b00000100;
constexpr Ordinal CallTraceFault = TraceFaultBase        | 0b00001000;
constexpr Ordinal ReturnTraceFault = TraceFaultBase      | 0b00010000;
constexpr Ordinal PrereturnTraceFault = TraceFaultBase   | 0b00100000;
constexpr Ordinal SupervisorTraceFault = TraceFaultBase  | 0b01000000;
constexpr Ordinal MarkTraceFault = TraceFaultBase        | 0b10000000;
constexpr Ordinal InvalidOpcodeFault = 0x00020001;
constexpr Ordinal UnimplementedFault = 0x00020002;
constexpr Ordinal UnalignedFault = 0x00020003;
constexpr Ordinal InvalidOperandFault = 0x00020004;
constexpr Ordinal ArithmeticOverflowFault = 0x0003'0001;
constexpr Ordinal ZeroDivideFault = 0x0003'0002;
constexpr Ordinal ConstraintRangeFault = 0x0005'0001;
constexpr Ordinal ProtectionLengthFault = 0x0007'0002;
constexpr Ordinal ProtectionBadAccessFault = 0x0007'0020;

constexpr Ordinal Machine_ParityErrorFault = 0x0008'0002;
constexpr Ordinal TypeMismatchFault = 0x000a'0001;
constexpr Ordinal OverrideFault = 0x0010'0000;
template<typename T>
volatile T& memory(size_t address) noexcept {
    return *reinterpret_cast<volatile T*>(address);
}
Ordinal faultPortValue;
void 
setFaultPort(Ordinal value) noexcept {
    faultPortValue = value;
}
 
Ordinal getFaultPort() noexcept { 
    return faultPortValue;
}
constexpr auto LOCKPIN = 12;
constexpr auto FAILPIN = 13;
constexpr auto INTPIN = 2;
constexpr auto BUSYPIN = 3;
constexpr auto BANK3 = 42;
constexpr auto BANK2 = 43;
constexpr auto BANK1 = 44;
constexpr auto BANK0 = 45;
constexpr auto SDPin = 4;
constexpr auto TFTCS = 10;
constexpr auto TFTDC = 9;
Adafruit_FT6206 touchScreen;
Adafruit_ILI9341 tft(TFTCS, TFTDC);
void
setInternalBusAddress(const SplitWord32& address) noexcept {
    digitalWrite(BANK0, address.internalBankAddress.bank0);
    digitalWrite(BANK1, address.internalBankAddress.bank1);
    digitalWrite(BANK2, address.internalBankAddress.bank2);
    digitalWrite(BANK3, address.internalBankAddress.bank3);
}

void
set328BusAddress(const SplitWord32& address) noexcept {
    // set the upper
    PORTF = address.splitAddress.a24_31;
    PORTK = address.splitAddress.a16_23;
    digitalWrite(38, address.splitAddress.a15);
}
template<typename T>
T load(Address address, TreatAs<T>) noexcept {
    SplitWord32 split(address);
    set328BusAddress(split);
    return memory<T>(static_cast<size_t>(split.splitAddress.lower) + 0x8000);
}
template<typename T>
void store(Address address, T value, TreatAs<T>) noexcept {
    SplitWord32 split(address);
    set328BusAddress(split);
    memory<T>(static_cast<size_t>(split.splitAddress.lower) + 0x8000) = value;

}
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
signalBootFailure() noexcept {
    digitalWrite(FAILPIN, HIGH);
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
#ifdef SxKxInstructions
    synmov = 0x600,
    synmovl,
    synmovq,
#endif
    atmod = 0x610,
    atadd = 0x612,
#ifdef SxKxInstructions
    synld = 0x615,
#endif
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
        Ordinal arithmeticStatus : 4;
        Ordinal unused0 : 1;
        Ordinal integerOverflowFlag : 1;
        Ordinal unused1 : 3;
        Ordinal integerOverflowMask : 1;
        Ordinal unused2 : 2;
        Ordinal noImpreciseFaults : 1;
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
    struct {
        Ordinal unused0 : 1; // 0
        Ordinal instructionTraceMode : 1; // 1
        Ordinal branchTraceMode : 1; // 2
        Ordinal callTraceMode : 1; // 3
        Ordinal returnTraceMode : 1; // 4
        Ordinal prereturnTraceMode : 1; // 5
        Ordinal supervisorTraceMode : 1; // 6
        Ordinal breakpointTraceMode : 1; // 7
        Ordinal unused1 : 9; // 8, 9, 10, 11, 12, 13, 14, 15, 16
        Ordinal instructionTraceEvent : 1; // 17
        Ordinal branchTraceEvent : 1; // 18
        Ordinal callTraceEvent : 1; // 19
        Ordinal returnTraceEvent : 1; // 20
        Ordinal prereturnTraceEvent : 1; // 21
        Ordinal supervisorTraceEvent : 1; // 22
        Ordinal breakpointTraceEvent : 1; // 23
        Ordinal unused2 : 8;
    } trace;
    struct {
        Ordinal invertResult : 1;
        Ordinal invertSrc1 : 1;
        Ordinal zeroSrc1 : 1;
        Ordinal src1IsBitPosition : 1;
        Ordinal invertSrc2 : 1;
        Ordinal zeroSrc2 : 1;
        Ordinal doXor : 1;
        Ordinal doOr : 1;
        Ordinal doAnd : 1;
        Ordinal ordinalOp : 1;
        Ordinal integerOp : 1;
        Ordinal performIncrement : 1;
        Ordinal performDecrement : 1;
        Ordinal performAdd : 1;
        Ordinal performSubtract : 1;
        Ordinal performCarry : 1;
        Ordinal performCompare : 1;
        Ordinal performConditionalCompare : 1;
        Ordinal performLogical : 1;
        Ordinal performMultiply : 1;
        Ordinal performDivide : 1;
        Ordinal performRemainder : 1;
        Ordinal performSyncf : 1;
        Ordinal performAtomicOperation : 1;
        Ordinal performModify : 1;
        Ordinal dontAdvanceIP : 1;
        Ordinal performRegisterTransfer : 1;
    } ucode;
    struct {
        Ordinal mask : 8;
        Ordinal count : 4;
        Ordinal compareAgainst : 3;
        Ordinal ccOnTrue : 3;
        Ordinal ccOnFalse : 3;
    } ucode2;
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

    void setValue(Ordinal value, TreatAsOrdinal) noexcept { o = value; }
    void setValue(Integer value, TreatAsInteger) noexcept { i = value; }
    [[nodiscard]] Integer getValue(TreatAsInteger) const noexcept { return i; }
    [[nodiscard]] Ordinal getValue(TreatAsOrdinal) const noexcept { return o; }
    template<typename T>
    [[nodiscard]] T getValue() const noexcept {
        return getValue(TreatAs<T>{});
    }

    template<typename T>
    void setValue(T value) noexcept {
        setValue(value, TreatAs<T>{});
    }

    void invert(TreatAsOrdinal) noexcept { o = ~o; }
    void invert(TreatAsInteger) noexcept { i = ~i; }
    void increment(TreatAsOrdinal) noexcept { ++o; }
    void increment(TreatAsInteger) noexcept { ++i; }
    void decrement(TreatAsOrdinal) noexcept { --o; }
    void decrement(TreatAsInteger) noexcept { --i; }

};
static_assert(sizeof(Register) == sizeof(Ordinal));
volatile Ordinal systemAddressTableBase = 0;
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
class RegisterFrame {
    public:
        RegisterFrame() = default;
        Register& get(byte index) noexcept { return registers[index & 0b1111]; }
        const Register& get(byte index) const noexcept { return registers[index & 0b1111]; }
    private:
        Register registers[16];
};
/** 
 * @brief Holds onto two separate register frames
 */ 
class GPRBlock {
    public:
        GPRBlock() = default;
        Register& get(byte index) noexcept { 
            if (index < 16) {
                return globals.get(index);
            } else {
                return locals.get(index);
            }
        }
        const Register& get(byte index) const noexcept { 
            if (index < 16) {
                return globals.get(index);
            } else {
                return locals.get(index);
            }
        }
        template<typename T>
        void setValue(byte index, T value) noexcept {
            get(index).setValue(value, TreatAs<T>{});
        }
        template<typename T>
        T getValue(byte index) const noexcept {
            return get(index).getValue(TreatAs<T>{});
        }

    private:
        RegisterFrame globals;
        RegisterFrame locals;
};

class RegisterBlock32 {
    public:
        RegisterBlock32() = default;
        Register& get(byte index) noexcept { return registers_[index & 0b11111]; }
        const Register& get(byte index) const noexcept { return registers_[index & 0b11111]; }
        template<typename T>
        void setValue(byte index, T value) noexcept {
            get(index).setValue(value, TreatAs<T>{});
        }
        template<typename T>
        T getValue(byte index) const noexcept {
            return get(index).getValue(TreatAs<T>{});
        }
    private:
        Register registers_[32];
};
GPRBlock gpr;
RegisterBlock32 sfrs;
Register ip;
Register ac; 
Register pc;
Register tc; 
Register flags; 
Register flags2; 
Register faultCode; 
Register instruction;
byte advanceBy;
Register& getGPR(byte index) noexcept {
    return gpr.get(index);
}
Register& getGPR(byte index, byte offset) noexcept {
    return getGPR((index + offset) & 0b11111);
}
void setGPR(byte index, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index).setValue(value, TreatAsOrdinal{}); }
void setGPR(byte index, byte offset, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index, offset).setValue(value, TreatAsOrdinal{}); }
void setGPR(byte index, Integer value, TreatAsInteger) noexcept { getGPR(index).setValue(value, TreatAsInteger{}); }
void setGPR(byte index, byte offset, Integer value, TreatAsInteger) noexcept { getGPR(index, offset).setValue(value, TreatAsInteger{}); }
Register& getSFR(byte index) noexcept;
Register& getSFR(byte index, byte offset) noexcept;
Ordinal getGPRValue(byte index, TreatAsOrdinal) noexcept { return getGPR(index).getValue(TreatAsOrdinal{}); }
Ordinal getGPRValue(byte index, byte offset, TreatAsOrdinal) noexcept { return getGPR(index, offset).getValue(TreatAsOrdinal{}); }
Integer getGPRValue(byte index, TreatAsInteger) noexcept { return getGPR(index).getValue(TreatAsInteger{}); }
Integer getGPRValue(byte index, byte offset, TreatAsInteger) noexcept { return getGPR(index, offset).getValue(TreatAsInteger{}); }
Ordinal unpackSrc1_REG(TreatAsOrdinal) noexcept;
Ordinal unpackSrc1_REG(byte offset, TreatAsOrdinal) noexcept;
Integer unpackSrc1_REG(TreatAsInteger) noexcept;
Ordinal unpackSrc2_REG(TreatAsOrdinal) noexcept;
Integer unpackSrc2_REG(TreatAsInteger) noexcept;
void setFaultCode(Ordinal value) noexcept;
bool faultHappened() noexcept;
void checkForPendingInterrupts() noexcept;
template<typename T>
void moveGPR(byte destIndex, byte srcIndex, TreatAs<T>) noexcept {
    setGPR(destIndex, getGPRValue(srcIndex, TreatAs<T>{}), TreatAs<T>{});
}
template<typename T>
void moveGPR(byte destIndex, byte destOffset, byte srcIndex, byte srcOffset, TreatAs<T>) noexcept {
    setGPR(destIndex, destOffset, getGPRValue(srcIndex, srcOffset, TreatAs<T>{}), TreatAs<T>{});
}
void scanbyte(Ordinal src2, Ordinal src1) noexcept;
void emul(Register& dest, Ordinal src1, Ordinal src2) noexcept;
void ediv(Register& dest, Ordinal src1, Ordinal src2) noexcept;
void scanbit(Register& dest, Ordinal src1, Ordinal src2) noexcept;
void spanbit(Register& dest, Ordinal src1, Ordinal src2) noexcept;
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
    return sfrs.get(index);
}
Register& getSFR(byte index, byte offset) noexcept {
    return getSFR((index + offset) & 0b11111);
}
void
syncf() noexcept {
    // Wait for all faults to be generated that are associated with any prior
    // uncompleted instructions
    /// @todo implement if it makes sense since we don't have a pipeline
}
void
flushreg() noexcept {
    /// @todo implement if it makes sense since we aren't using register frames
}
void
mark() noexcept {
    if (pc.pc.traceEnable && tc.trace.breakpointTraceMode) {
        setFaultCode(MarkTraceFault);
    }
}
void
fmark() noexcept {
    if (pc.pc.traceEnable) {
        setFaultCode(MarkTraceFault);
    }
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
    ip.setValue(getGPRValue(RIPIndex, TreatAsOrdinal{}), TreatAsOrdinal{});
    flags.ucode.dontAdvanceIP = 1;
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
                ac.setValue(y, TreatAsOrdinal{});
                if (pc.inSupervisorMode()) {
                    pc.setValue(x, TreatAsOrdinal{});
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
                ac.setValue(y, TreatAsOrdinal{});
                if (pc.inSupervisorMode()) {
                    pc.setValue(x, TreatAsOrdinal{});
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
    flags.ucode.dontAdvanceIP = 1;
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
    flags.ucode.dontAdvanceIP = 1;
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
void 
ldl() noexcept {
    if ((instruction.mem.srcDest & 0b1) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 2);
        // support unaligned accesses
    }
}

void
stl() noexcept {
    if ((instruction.mem.srcDest & 0b1) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 2);
        // support unaligned accesses
    }
}
void
ldt() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 3);
        // support unaligned accesses
    }
}

void
stt() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 3);
        // support unaligned accesses
    }
}

void
ldq() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 4);
        // support unaligned accesses
    }
}

void
stq() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 4);
        // support unaligned accesses
    }
}

void
balx() noexcept {
    auto address = computeAddress();
    setGPR(instruction.mem.srcDest, ip.getValue(TreatAsOrdinal{}) + advanceBy, TreatAsOrdinal{});
    ip.setValue(address, TreatAsOrdinal{});
    flags.ucode.dontAdvanceIP = 1;
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
    setGPR(RIPIndex, ip.getValue(TreatAsOrdinal{}) + advanceBy, TreatAsOrdinal{});
    enterCall(fp);
    ip.setValue(addr, TreatAsOrdinal{});
    setGPR(PFPIndex, fp, TreatAsOrdinal{});
    setGPR(FPIndex, temp, TreatAsOrdinal{});
    setGPR(SPIndex , temp + 64, TreatAsOrdinal{});
    flags.ucode.dontAdvanceIP = 1;
}
void 
call() {
    // wait for any uncompleted instructions to finish
    auto temp = (getGPRValue(SPIndex, TreatAsOrdinal{}) + C) & NotC; // round stack pointer to next boundary
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    setGPR(RIPIndex, ip.getValue(TreatAsOrdinal{}) + advanceBy, TreatAsOrdinal{});
    enterCall(fp);
    ip.i += instruction.ctrl.displacement;
    setGPR(PFPIndex, fp, TreatAsOrdinal{});
    setGPR(FPIndex, temp, TreatAsOrdinal{});
    setGPR(SPIndex , temp + 64, TreatAsOrdinal{});
    flags.ucode.dontAdvanceIP = 1;
}
Ordinal getSupervisorStackPointer() noexcept;
void
calls(Ordinal src1) noexcept {
    if (auto targ = src1; targ > 259) {
        setFaultCode(ProtectionLengthFault);
    } else {
        syncf();
        auto tempPE = load(getSystemProcedureTableBase() + 48 + (4 * targ), TreatAsOrdinal{});
        auto type = tempPE & 0b11;
        auto procedureAddress = tempPE & ~0b11;
        // read entry from system-procedure table, where spbase is address of
        // system-procedure table from Initial Memory Image
        setGPR(RIPIndex, ip.getValue(TreatAsOrdinal{}) + advanceBy, TreatAsOrdinal{});
        ip.setValue(procedureAddress, TreatAsOrdinal{});
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
        pfp.setValue(getGPRValue(FPIndex, TreatAsOrdinal{}) & ~0b1'111, TreatAsOrdinal{});
        pfp.pfp.rt = tempRRR;
        setGPR(FPIndex, temp, TreatAsOrdinal{});
        setGPR(SPIndex, temp + 64, TreatAsOrdinal{});
        flags.ucode.dontAdvanceIP = 1;
    }
}
void
bx() noexcept {
    ip.setValue(computeAddress(), TreatAsOrdinal{});
    flags.ucode.dontAdvanceIP = 1;
}

volatile bool HasSDCard = false;
void 
setup() {
    Serial.begin(115200);
    Serial.println(F("i960 Simulator (C) 2022 and beyond Joshua Scoggins"));
    Serial.println(F("This software is open source software"));
    Serial.println(F("Base Platform: Arduino Mega2560"));
    Serial.println(F("Bringing up peripherals"));
    Serial.println();

    Serial.print(F("Configuring SPI..."));
    SPI.begin();
    Serial.println(F("DONE"));
    Serial.print(F("Configuring ILI9341..."));
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    Serial.println(F("DONE"));
    Serial.print(F("Configuring TWI..."));
    Wire.begin();
    Serial.println(F("DONE"));
    Serial.print(F("Configuring FT6206..."));
    if (!touchScreen.begin(40)) {
        Serial.println(F("FAILED"));
    } else {
        Serial.println(F("DONE"));
    }
    Serial.print(F("Configuring GPIOs..."));
    pinMode(BANK0, OUTPUT);
    pinMode(BANK1, OUTPUT);
    pinMode(BANK2, OUTPUT);
    pinMode(BANK3, OUTPUT);
    /// @todo configure ports f and k
    pinMode(FAILPIN, OUTPUT);
    digitalWrite(FAILPIN, LOW);

    pinMode(INTPIN, INPUT);
    pinMode(BUSYPIN, INPUT);

    pinMode(LOCKPIN, OUTPUT);
    digitalWrite(LOCKPIN, LOW);
    pinMode(LOCKPIN, INPUT);
    Serial.println(F("DONE"));
    Serial.print(F("Setting up EBI..."));
    // cleave the address space in half via sector limits.
    // lower half is io space for the implementation
    // upper half is the window into the 32/8 bus
    XMCRB = 0;           // No external memory bus keeper and full 64k address
                         // space
    XMCRA = 0b1100'0000; // Divide the 64k address space in half at 0x8000, no
                         // wait states activated either. Also turn on the EBI
    set328BusAddress(0);
    setInternalBusAddress(0);
    Serial.println(F("DONE"));

    HasSDCard = SD.begin(SDPin);
    if (!HasSDCard) {
        Serial.println(F("NO SD CARD"));
    } else {
        Serial.println(F("FOUND SD CARD"));
    }
    Serial.print(F("Configuring simulator structures..."));
    // so we need to do any sort of processor setup here
    ip.clear();
    for (int i = 0; i < 32; ++i) {
        getGPR(i).clear();
    }
    Serial.println(F("DONE"));
    Serial.println(F("BOOT COMPLETE!!"));
}
void
performRegisterTransfer(byte mask, byte count) noexcept {
    if (((instruction.reg.srcDest & mask) != 0) || ((instruction.reg.src1 & mask) != 0)) {
        setFaultCode(InvalidOpcodeFault);
    }
    for (byte i = 0; i < count; ++i) {
        setGPR(instruction.reg.srcDest, i, unpackSrc1_REG(i, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
Ordinal 
Register::modify(Ordinal mask, Ordinal src) noexcept {
    auto tmp = o;
    o = ::modify(mask, src, o);
    return tmp;
}
void 
invokeCore() noexcept {
    setFaultCode(NoFault);
    instruction.setValue(load(ip.a, TreatAsOrdinal{}), TreatAsOrdinal{});
    auto& regDest = getGPR(instruction.reg.srcDest);
    auto src2o = unpackSrc2_REG(TreatAsOrdinal{});
    auto src2i = unpackSrc2_REG(TreatAsInteger{});
    auto src1o = unpackSrc1_REG(TreatAsOrdinal{});
    auto src1i = unpackSrc1_REG(TreatAsInteger{});
    flags.clear();
    flags2.clear();
    advanceBy = 4;
    
    switch (instruction.getOpcode()) {
        case Opcodes::bal: // bal
            setGPR(LRIndex, ip.getValue<Ordinal>() + 4, TreatAsOrdinal{});
            // then fallthrough and take the branch
        case Opcodes::b: // b
            ip.i += instruction.cobr.displacement;
            flags.ucode.dontAdvanceIP = 1;
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
                flags.ucode.dontAdvanceIP = 1;
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
                flags.ucode.dontAdvanceIP = 1;
            }
            break;
        case Opcodes::faultno:
            if (ac.arith.conditionCode == 0) {
                setFaultCode(ConstraintRangeFault);
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
                setFaultCode(ConstraintRangeFault);
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
        case Opcodes::ld: 
            //flags2.ucode2.count = 1;
            loadBlock(computeAddress(), instruction.mem.srcDest, 1);
            break;
        case Opcodes::st: 
            //flags2.ucode2.count = 1;
            storeBlock(computeAddress(), instruction.mem.srcDest, 1);
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
            /// @todo fully implement fault detection
            store<ByteInteger>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAsInteger{}), TreatAs<ByteInteger>{});
            break;
        case Opcodes::ldis:
            setGPR(instruction.mem.srcDest, load(computeAddress(), TreatAs<ShortInteger>{}), TreatAsInteger{});
            break;
        case Opcodes::stis:
            /// @todo fully implement fault detection
            store<ShortInteger>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAsInteger{}), TreatAs<ShortInteger>{});
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
            flags.ucode.invertResult = 1;
        case Opcodes::andOperation: // and
            flags.ucode.performLogical = 1;
            flags.ucode.doAnd = 1;
            break;
        case Opcodes::clrbit: // clrbit
                              // clrbit is src2 & ~computeBitPosition(src1)
                              // so lets use andnot
            flags.ucode.src1IsBitPosition = 1;
        case Opcodes::andnot: // andnot
            flags.ucode.doAnd = 1;
            flags.ucode.invertSrc1 = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::notand: // notand
            flags.ucode.doAnd = 1;
            flags.ucode.invertSrc2 = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::notbit: // notbit
                     // notbit is src2 ^ computeBitPosition(src1)
            flags.ucode.src1IsBitPosition = 1;
        case Opcodes::xorOperation: // xor
            flags.ucode.doXor = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::setbit: // setbit
                     // setbit is src2 | computeBitPosition(src1o)
            flags.ucode.src1IsBitPosition = 1;
        case Opcodes::orOperation: // or
            flags.ucode.doOr = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::nor: // nor
            flags.ucode.doOr = 1;
            flags.ucode.invertResult = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::xnor: // xnor
            flags.ucode.doXor = 1;
            flags.ucode.invertResult = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::notOperation: // not 
                     // perform fallthrough to ornot with src2 set to zero
            flags.ucode.zeroSrc2 = 1;
        case Opcodes::ornot: // ornot
            flags.ucode.doOr = 1;
            flags.ucode.invertSrc1 = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::notor: // notor
            flags.ucode.doOr = 1;
            flags.ucode.invertSrc2 = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::alterbit: // alterbit
            flags.ucode.src1IsBitPosition = 1;
            flags.ucode.performLogical = 1;
            if (ac.arith.conditionCode & 0b010) {
                flags.ucode.doOr = 1;
            } else {
                flags.ucode.doAnd = 1;
                flags.ucode.invertSrc1 = 1;
            }
            break;
        case Opcodes::addo: // addo
            flags.ucode.performAdd = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::addi: // addi
            flags.ucode.performAdd = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::subo: // subo
            // I remember this trick from college, subtraction is just addition
            // with a negative second argument :). I never gave it much thought
            // until now but it seems to be an effective trick to save space.
            flags.ucode.performSubtract = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::subi: // subi
            flags.ucode.performSubtract = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::shro: // shro
            regDest.setValue<Ordinal>(src1o < 32 ? src2o >> src1o : 0);
            break;
        case Opcodes::shrdi: // shrdi
                  // according to the manual, equivalent to divi value, 2 so that is what we're going to do for correctness sake
            regDest.setValue<Integer>( src1i < 32 && src1i >= 0 ? src2i / computeBitPosition(src1i) : 0);
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
            regDest.setValue<Integer>(src2i >> src1i);
            break;
        case Opcodes::shlo: // shlo
            regDest.setValue<Ordinal>(src1o < 32 ? src2o << src1o : 0);
            break;
        case Opcodes::rotate: // rotate
            regDest.setValue<Ordinal>(rotateOperation(src2o, src1o));
            break;
        case Opcodes::shli: // shli
            regDest.setValue<Integer>(src2i << src1i);
            break;
        case Opcodes::cmpo: // cmpo
            flags.ucode.performCompare = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::cmpi: // cmpi
            flags.ucode.performCompare = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::concmpo: // concmpo
            flags.ucode.performConditionalCompare = 1;
            flags.ucode.ordinalOp = 1;
            flags.ucode.performCompare = 1;
            break;
        case Opcodes::concmpi: // concmpi
            flags.ucode.performConditionalCompare = 1;
            flags.ucode.integerOp = 1;
            flags.ucode.performCompare = 1;
            break;
        case Opcodes::cmpinco: // cmpinco
            flags.ucode.performCompare = 1;
            flags.ucode.ordinalOp = 1;
            flags.ucode.performIncrement = 1;
            break;
        case Opcodes::cmpinci: // cmpinci
            flags.ucode.performCompare = 1;
            flags.ucode.integerOp = 1;
            flags.ucode.performIncrement = 1;
            break;
        case Opcodes::cmpdeco: // cmpdeco
            flags.ucode.performCompare = 1;
            flags.ucode.ordinalOp = 1;
            flags.ucode.performDecrement = 1;
            break;
        case Opcodes::cmpdeci: // cmpdeci
            flags.ucode.performCompare = 1;
            flags.ucode.integerOp = 1;
            flags.ucode.performDecrement = 1;
            break;
        case Opcodes::scanbyte: // scanbyte
            scanbyte(src2o, src1o);
            break;
        case Opcodes::chkbit: // chkbit
            ac.arith.conditionCode = ((src2o & computeBitPosition(src1o)) == 0 ? 0b000 : 0b010);
            break;
        case Opcodes::addc: 
            flags.ucode.performAdd = 1;
            flags.ucode.performCarry = 1;
            break;
        case Opcodes::subc:
            flags.ucode.performSubtract = 1;
            flags.ucode.performCarry = 1;
            break;
        case Opcodes::mov:
            regDest.setValue<Ordinal>(src1o);
            break;
        case Opcodes::movl:
            flags.ucode.performRegisterTransfer = 1;
            flags2.ucode2.mask = 0b1;
            flags2.ucode2.count = 2; 
            //performRegisterTransfer(0b1, 2);
            break;
        case Opcodes::movt:
            flags.ucode.performRegisterTransfer = 1;
            flags2.ucode2.mask = 0b11;
            flags2.ucode2.count = 3; 
            //performRegisterTransfer(0b11, 3);
            break;
        case Opcodes::movq:
            flags.ucode.performRegisterTransfer = 1;
            flags2.ucode2.mask = 0b11;
            flags2.ucode2.count = 3; 
            //performRegisterTransfer(0b11, 4);
            break;
        case Opcodes::syncf:
            flags.ucode.performSyncf = 1;
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
            flags.ucode.performMultiply = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::muli:
            flags.ucode.performMultiply = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::divi:
            flags.ucode.performDivide = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::divo:
            flags.ucode.performDivide = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::remo:
            flags.ucode.performRemainder = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::remi:
            flags.ucode.performRemainder = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::modi: 
            if (auto denominator = src1i; denominator == 0) {
                setFaultCode(ZeroDivideFault);
            } else {
                auto numerator = src2i;
                auto result = numerator - ((numerator / denominator) * denominator);
                if (((numerator * denominator) < 0) && (result != 0)) {
                    result += denominator;
                }
                regDest.setValue<Integer>(result);
            }
            break;
        case Opcodes::modify:
            regDest.setValue<Ordinal>(modify(src1o, src2o, regDest.getValue<Ordinal>()));
            break;
        case Opcodes::extract:
            // taken from the Hx manual as it isn't insane
            regDest.setValue<Ordinal>((regDest.o >> (src1o > 32 ? 32 : src1o)) & ~(0xFFFF'FFFF << src2o));
            break;
        case Opcodes::modac: 
            regDest.setValue<Ordinal>(ac.modify(src1o, src2o));
            break;
        case Opcodes::modtc: 
            regDest.setValue<Ordinal>(tc.modify(src1o, src2o));
            break;
        case Opcodes::modpc:
            if (auto mask = src1o; mask != 0) {
                if (!pc.inSupervisorMode()) {
                    setFaultCode(TypeMismatchFault);
                } else {
                    regDest.setValue<Ordinal>(pc.modify(mask, src2o));
                    if (regDest.getPriority() > pc.getPriority()) {
                        checkForPendingInterrupts();
                    }
                }
            } else {
                regDest.setValue<Ordinal>(pc.getValue<Ordinal>());
            }
            break;
        case Opcodes::atadd:
            flags.ucode.performSyncf = 1;
            flags.ucode.performAtomicOperation = 1;
            flags.ucode.performAdd = 1;
            break;
        case Opcodes::atmod:
            flags.ucode.performSyncf = 1;
            flags.ucode.performAtomicOperation = 1;
            flags.ucode.performModify = 1;
            break;
        case Opcodes::emul:
            emul(regDest, src2o, src1o);
            break;
        case Opcodes::ediv:
            ediv(regDest, src2o, src1o);
            break;
        case Opcodes::calls:
            calls(src1o);
            break;
        case Opcodes::spanbit:
            spanbit(regDest, src2o, src1o);
            break;
        case Opcodes::scanbit:
            scanbit(regDest, src2o, src1o);
            break;
        default:
                setFaultCode(UnimplementedFault);
            break;
    }
    if (flags.ucode.performSyncf) {
        syncf();
    }
    if (flags.ucode.performAtomicOperation) {
        lockBus();
        auto addr = src1o & 0xFFFF'FFFC;
        auto temp = load(addr, TreatAsOrdinal{});
        Ordinal result = 0;
        if (flags.ucode.performAdd) {
            // adds the src (src2 internally) value to the value in memory location specified with the addr (src1 in this case) operand.
            // The initial value from memory is stored in dst (internally src/dst).
            result = temp + src2o;
        } else if (flags.ucode.performModify) {
            // copies the src/dest value (logical version) into the memory location specifeid by src1.
            // The bits set in the mask (src2) operand select the bits to be modified in memory. The initial
            // value from memory is stored in src/dest
            result = modify(src2o, regDest.o, temp);
        } else {
            // if we got here then it means we don't have something configured
            // correctly
                setFaultCode(InvalidOpcodeFault);
        }
        if (!faultHappened()) {
            store(addr, result, TreatAsOrdinal{});
            regDest.o = temp;
        }
        unlockBus();
    }
    if (flags.ucode.performRegisterTransfer) {
        performRegisterTransfer(flags2.ucode2.mask, flags2.ucode2.count);
    }
    if (flags.ucode.performLogical) {
        if (flags.ucode.src1IsBitPosition) {
            src1o = computeBitPosition(src1o);
        }
        if (flags.ucode.invertSrc1) {
            src1o = ~src1o;
        }
        if (flags.ucode.zeroSrc2) {
            src2o = 0;
        }
        if (flags.ucode.invertSrc2) {
            src2o = ~src2o;
        }
        if (flags.ucode.doAnd) {
            regDest.setValue(src2o & src1o, TreatAsOrdinal{});
        } else if (flags.ucode.doXor) {
            regDest.setValue(src2o ^ src1o, TreatAsOrdinal{});
        } else if (flags.ucode.doOr) {
            regDest.setValue(src2o | src1o, TreatAsOrdinal{});
        }
        if (flags.ucode.invertResult) {
            regDest.invert(TreatAsOrdinal{});
        }
    }
    if (flags.ucode.performCompare) {
        if (flags.ucode.performConditionalCompare) {
            if ((ac.arith.conditionCode & 0b100) == 0) {
                bool cond = false;
                if (flags.ucode.ordinalOp) {
                    cond = src1o <= src2o;
                } else if (flags.ucode.integerOp) {
                    cond = src1i <= src2i;
                } else {
                    // if we got here then it means we don't have something configured
                    // correctly
                    setFaultCode(InvalidOpcodeFault);
                }
                if (!faultHappened()) {
                    ac.arith.conditionCode = cond ? 0b010 : 0b001;
                }
            }
        } else {
            if (flags.ucode.ordinalOp) {
                cmpGeneric(src1o, src2o);
                if (flags.ucode.performIncrement) {
                    regDest.o = src2o + 1;
                }
                if (flags.ucode.performDecrement) {
                    regDest.o = src2o - 1;
                }
            } else if (flags.ucode.integerOp) {
                cmpGeneric(src1i, src2i);
                if (flags.ucode.performIncrement) {
                    regDest.i = src2i + 1;
                }
                if (flags.ucode.performDecrement) {
                    regDest.i = src2i - 1;
                }
            } else {
                // if we got here then it means we don't have something configured
                // correctly
                setFaultCode(InvalidOpcodeFault);
            }
        }
    }
    if (flags.ucode.performCarry) {
        LongOrdinal result = 0;
        if (flags.ucode.performAdd) {
            result = static_cast<LongOrdinal>(src2o) + static_cast<LongOrdinal>(src1o);
        } else if (flags.ucode.performSubtract) {
            result = static_cast<LongOrdinal>(src2o) - static_cast<LongOrdinal>(src1o) - 1;
        } else {
            // if we got here then it means we don't have something configured
            // correctly
                setFaultCode(InvalidOpcodeFault);
        }
        if (!faultHappened()) {
            result += (ac.getCarryBit() ? 1 : 0);
            regDest.o = static_cast<Ordinal>(result);
            arithmeticWithCarryGeneric(static_cast<Ordinal>(result >> 32), 
                    (src2o & 0x8000'0000), 
                    (src1o & 0x8000'0000), 
                    (regDest.o & 0x8000'0000));
        }
    } else if (flags.ucode.performAdd) {
        if (flags.ucode.ordinalOp) {
            regDest.o = src2o + src1o;
        } else if (flags.ucode.integerOp) {
            regDest.i = src2i + src1i;
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }
    } else if (flags.ucode.performSubtract) {
        if (flags.ucode.ordinalOp) {
            regDest.o = src2o - src1o;
        } else if (flags.ucode.integerOp) {
            regDest.i = src2i - src1i;
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }

    } else if (flags.ucode.performMultiply) {
        if (flags.ucode.ordinalOp) {
            regDest.o = src2o * src1o;
        } else if (flags.ucode.integerOp) {
            regDest.i = src2i * src1i;
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }
    } else if (flags.ucode.performDivide) {
        if (flags.ucode.ordinalOp) {
            if (src1o == 0) {
                /// @todo fix this
                setFaultCode(ZeroDivideFault);
            } else {
                regDest.o = src2o / src1o;
            }
        } else if (flags.ucode.integerOp) {
            if (src1i == 0) {
                /// @todo fix this
                setFaultCode(ZeroDivideFault);
            } else {
                regDest.i = src2i / src1i;
            }
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }
    } else if (flags.ucode.performRemainder) {
        if (flags.ucode.ordinalOp) {
            if (src1o == 0) {
                /// @todo fix this
                setFaultCode(ZeroDivideFault);
            } else {
                // taken from the i960Sx manual
                //dest.setOrdinal(src2 - ((src2 / src1) * src1));
                regDest.o = src2o % src1o;
            }
        } else if (flags.ucode.integerOp) {
            if (src1i == 0) {
                /// @todo fix this
                setFaultCode(ZeroDivideFault);
            } else {
                // taken from the i960Sx manual
                //dest.setInteger(src2 - ((src2 / src1) * src1));
                regDest.i = src2i % src1i;
            }
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }
    }
    if (faultHappened()) {
        /// @todo implement this as the fallback operation when something bad
        /// happens
        ///
        /// Faults are basically fallback behavior when something goes wacky!
        setFaultPort(faultCode.o);
    }
    // okay we got here so we need to start grabbing data off of the bus and
    // start executing the next instruction
    if (!flags.ucode.dontAdvanceIP) {
        ip.o += advanceBy; 
    }
}
void 
loop() {
    invokeCore();
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

void
scanbyte(Ordinal src2, Ordinal src1) noexcept {
    if (Register s2(src2), s1(src1); 
            s1.bytes[0] == s2.bytes[0] ||
            s1.bytes[1] == s2.bytes[1] ||
            s1.bytes[2] == s2.bytes[2] ||
            s1.bytes[3] == s2.bytes[3]) {
        ac.arith.conditionCode = 0b010;
    } else {
        ac.arith.conditionCode = 0;
    }
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
emul(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    SplitWord64 result;
    if ((instruction.reg.srcDest & 0b1) != 0) {
        setFaultCode(InvalidOpcodeFault);
    }  else {
        result.whole = static_cast<LongOrdinal>(src2) * static_cast<LongOrdinal>(src1);
    }
    // yes this can be undefined by design :)
    // if we hit a fault then we just give up whats on the stack :)
    dest.setValue(result.parts[0], TreatAsOrdinal{});
    setGPR(instruction.reg.srcDest, 1, result.parts[1], TreatAsOrdinal{});
}

void
ediv(Register& dest, Ordinal src1, Ordinal src2Lower) noexcept {
    SplitWord64 result, src2;
    result.whole = 0;
    src2.parts[0] = src2Lower;
    if ((instruction.reg.srcDest & 0b1) != 0 || (instruction.reg.src2 & 0b1) != 0) {
        /// @todo fix
        setFaultCode(InvalidOpcodeFault);
    } else if (src1 == 0) {
        // divide by zero
        setFaultCode(ZeroDivideFault);
    } else {
        src2.parts[1] = getGPRValue(instruction.reg.src2, 1, TreatAsOrdinal{});
        result.parts[1] = src2.whole / src1; // quotient
        result.parts[0] = static_cast<Ordinal>(src2.whole - (src2.whole / src1) * src1); // remainder
    }
    // yes this can be undefined by design :)
    // if we hit a fault then we just give whats on the stack :)
    // however, to get the compiler to shut up, I am zeroing out the result
    // field
    dest.setValue<Ordinal>(result.parts[0]);
    setGPR(instruction.reg.srcDest, 1, result.parts[1], TreatAsOrdinal{});
}
Ordinal 
getSystemAddressTableBase() noexcept { 
    return systemAddressTableBase; 
}

Ordinal
getSystemProcedureTableBase() noexcept {
    return load(getSystemAddressTableBase() + 120, TreatAsOrdinal{});
}

Ordinal
getSupervisorStackPointer() noexcept {
    return load((getSystemProcedureTableBase() + 12), TreatAsOrdinal{});
}
template<bool doScan>
void
xbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    for (Ordinal index = 0; index < 32; ++index) {
        if ((src1 & computeBitPosition(31 - index)) != 0) {
            if constexpr (doScan) {
                dest.o = (31 - index);
                ac.arith.conditionCode = 0b010;
                return;
            }
        } else {
            if constexpr (!doScan) {
                dest.o = (31 - index);
                ac.arith.conditionCode = 0b010;
                return;
            }
        }
    }
    dest.o = 0xFFFF'FFFF;
    ac.arith.conditionCode = 0;
}
void 
scanbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    xbit<true>(dest, src1, src2);
}
void 
spanbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    xbit<false>(dest, src1, src2);
}

void
setFaultCode(Ordinal fault) noexcept {
    faultCode.setValue(fault, TreatAsOrdinal{});
}
bool 
faultHappened() noexcept {
    return faultCode.getValue(TreatAsOrdinal{}) != NoFault;
}
