// sim5
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

#ifndef SIM5_CORE_H__
#define SIM5_CORE_H__

#include "Types.h"
#include "IAC.h"
#include "BinaryOperations.h"

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
    cmpstr,
    movqstr,
    movstr,
    atmod = 0x610,
    atadd = 0x612,
    inspacc,
    ldphy,
    synld,
    fill = 0x617,
    spanbit = 0x640,
    scanbit,
    daddc,
    dsubc,
    dmovt = 0x644,
    modac = 0x645,
    condrec = 0x646,
    modify = 0x650,
    extract,
    modtc = 0x654,
    modpc,
    receive,
    calls = 0x660,
    send = 0x662,
    sendserv = 0x663,
    resumeprcs = 0x664,
    schedprcs = 0x665,
    saveprcs = 0x666,
    condwait = 0x668,
    wait = 0x669,
    signal = 0x66a,
    mark = 0x66b,
    fmark,
    flushreg,
    syncf = 0x66f,
    emul,
    ediv,
    ldtime,
    mulo = 0x700,
    remo = 0x708,
    divo = 0x70b,
    muli = 0x710,
    remi = 0x718,
    modi,
    divi = 0x71b,
    // numerics extensions
    addr = 0x78f,
    addrl = 0x79f,
    atanr = 0x680,
    atanrl = 0x690,
    classr = 0x68F,
    classrl = 0x69F,
    cmpor = 0x684,
    cmporl = 0x694,
    cmpr = 0x685,
    cmprl = 0x695,
    cosr = 0x68D,
    cosrl = 0x69D,
    cpysre = 0x6E2,
    cpyrsre = 0x6E3,
    cvtir = 0x674,
    cvtilr = 0x675,
    cvtri = 0x6C0,
    cvtril,
    cvtzri,
    cvtzril,
    divr = 0x78B,
    divrl = 0x79B,
    expr = 0x689,
    exprl = 0x699,
    logbnr = 0x68A,
    logbnrl = 0x69A,
    logepr = 0x681,
    logeprl = 0x691,
    logr = 0x682,
    logrl = 0x692,
    movr = 0x6C9,
    movrl = 0x6D9,
    movre = 0x6E9,
    mulr = 0x78C,
    mulrl = 0x79C,
    remr = 0x683,
    remrl = 0x693,
    roundr = 0x68B,
    roundrl = 0x69B,
    scaler = 0x677,
    scalerl = 0x676,
    sinr = 0x68C,
    sinrl = 0x69C,
    sqrtr = 0x688,
    sqrtrl = 0x698,
    subr = 0x78D,
    subrl = 0x79D,
    tanr = 0x68E,
    tanrl = 0x69E,


    // new core instructions
    bswap = 0x5AD,
    dcctl = 0x65C,
    eshro = 0x5D8, 
    dcinva = 0xAD,  // hx specific instruction
    iccctl = 0x65B,
    intctl = 0x658,
    intdis = 0x5B4,
    inten = 0x5B5,
    addono = 0x780,
    addog = 0x790,
    addoe = 0x7a0,
    addoge = 0x7b0,
    addol = 0x7c0,
    addone = 0x7d0,
    addole = 0x7e0,
    addoo = 0x7f0,
    addino = 0x781,
    addig = 0x791,
    addie = 0x7a1,
    addige = 0x7b1,
    addil = 0x7c1,
    addine = 0x7d1,
    addile = 0x7e1,
    addio = 0x7f1,
    subono = 0x782,
    subog = 0x792,
    suboe = 0x7a2,
    suboge = 0x7b2,
    subol = 0x7c2,
    subone = 0x7d2,
    subole = 0x7e2,
    suboo = 0x7f2,
    subino = 0x783,
    subig = 0x793,
    subie = 0x7a3,
    subige = 0x7b3,
    subil = 0x7c3,
    subine = 0x7d3,
    subile = 0x7e3,
    subio = 0x7f3,
    selno = 0x784,
    selg = 0x794,
    sele = 0x7a4,
    selge = 0x7b4,
    sell = 0x7c4,
    selne = 0x7d4,
    selle = 0x7e4,
    selo = 0x7f4,
    sysctl = 0x659, // Jx instruction that I am going to support the same way
                    // we do IAC instructions. The format of the sysctl
                    // "packet" is nearly identical to an IAC with the order of
                    // fields in the first 32-bit being reversed
    cmpob = 0x594,
    cmpib = 0x595,
    cmpos = 0x596,
    cmpis = 0x597,

    halt = 0x65D, // page missing from the Hx manual but in the Jx manual...
                  // oops
    // Cx specific instructions
    sdma = 0x630,
    udma = 0x631,
};
enum class BootResult : uint8_t {
    Success,
    SelfTestFailure,
    ChecksumFail,
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
    } processControls;
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
    uint8_t interruptControl[4];
    constexpr auto getConditionCode() const noexcept { return arith.conditionCode; }
    void setPriority(Ordinal value) noexcept { processControls.priority = value; }
    bool inSupervisorMode() const noexcept { return processControls.executionMode; }
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
    [[nodiscard]] constexpr byte getPriority() const noexcept { return processControls.priority; }

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
    template<typename T>
    T& viewAs() noexcept {
        return T(*this);
    }
    template<typename T>
    const T& viewAs() const noexcept {
        return T(*this);
    }
};
static_assert(sizeof(Register) == sizeof(Ordinal));
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

class Core {
    public:
        void begin() noexcept;
        BootResult start() noexcept;
        void stop() noexcept;
        void cycle() noexcept;
        [[nodiscard]] constexpr bool running() const noexcept { return running_; }
    protected:
        void lockBus();
        void unlockBus();
        void signalBootFailure();
        void setFaultPort(Ordinal value) noexcept;
        [[nodiscard]] Ordinal getFaultPort() const noexcept;
        /// @todo insert iac dispatch here
        /// @todo insert routines for getting registers and such 
        [[nodiscard]] Register& getGPR(byte index) noexcept { return gpr_.get(index); }
        [[nodiscard]] Register& getGPR(byte index, byte offset) noexcept { return getGPR((index + offset) & 0b11111); }
        [[nodiscard]] inline Ordinal getGPRValue(byte index, TreatAsOrdinal) noexcept { return getGPR(index).getValue(TreatAsOrdinal{}); }
        [[nodiscard]] inline Ordinal getGPRValue(byte index, byte offset, TreatAsOrdinal) noexcept { return getGPR(index, offset).getValue(TreatAsOrdinal{}); }
        [[nodiscard]] inline Integer getGPRValue(byte index, TreatAsInteger) noexcept { return getGPR(index).getValue(TreatAsInteger{}); }
        [[nodiscard]] constexpr Ordinal getSystemAddressTableBase() const noexcept { return systemAddressTableBase_; }
        [[nodiscard]] Ordinal getSystemProcedureTableBase() const noexcept;
        [[nodiscard]] Ordinal getSupervisorStackPointer() const noexcept;
        void restoreRegisterSet() noexcept;
        inline void setGPR(byte index, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index).setValue(value, TreatAsOrdinal{}); }
        inline void setGPR(byte index, byte offset, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index, offset).setValue(value, TreatAsOrdinal{}); }
        inline void setGPR(byte index, Integer value, TreatAsInteger) noexcept { getGPR(index).setValue(value, TreatAsInteger{}); }
        [[nodiscard]] Register& getSFR(byte index) noexcept;
        [[nodiscard]] Register& getSFR(byte index, byte offset) noexcept;
        [[nodiscard]] Ordinal unpackSrc1_REG(TreatAsOrdinal) noexcept;
        [[nodiscard]] Ordinal unpackSrc1_REG(byte offset, TreatAsOrdinal) noexcept;
        [[nodiscard]] Integer unpackSrc1_REG(TreatAsInteger) noexcept;
        [[nodiscard]] Ordinal unpackSrc2_REG(TreatAsOrdinal) noexcept;
        [[nodiscard]] Integer unpackSrc2_REG(TreatAsInteger) noexcept;
        [[nodiscard]] Ordinal unpackSrc1_COBR(TreatAsOrdinal) noexcept;
        [[nodiscard]] Integer unpackSrc1_COBR(TreatAsInteger) noexcept;
        [[nodiscard]] Ordinal unpackSrc2_COBR(TreatAsOrdinal) noexcept;
        [[nodiscard]] Integer unpackSrc2_COBR(TreatAsInteger) noexcept;
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
        bool getMaskedConditionCode() noexcept;
        bool conditionCodeEqualsMask() noexcept;
        bool fullConditionCodeCheck() noexcept;
        Ordinal computeAddress() noexcept;
        void performRegisterTransfer(byte mask, byte count) noexcept;
    protected:
        void sendIAC(const iac::Message& msg) noexcept;
        void dispatchInterrupt(uint8_t vector) noexcept;
        void purgeInstructionCache() noexcept;
        void reinitializeProcessor(Ordinal satBase, Ordinal prcbBase, Ordinal startIP) noexcept;
        void setBreakpointRegister(Ordinal breakpointIp0, Ordinal breakpointIp1) noexcept;
        void storeSystemBase(Ordinal destinationAddress) noexcept;
        void testPendingInterrupts() noexcept;
        // Kx related IACs
        void freeze() noexcept;
        void continueInitialization() noexcept;

        // MC related IACs
        void checkProcessNotice(Ordinal processSegmentSelectorBase) noexcept;
        void flushLocalRegisters(Ordinal physicalStackPageAddress) noexcept;
        void flushProcess() noexcept;
        void flushTLB() noexcept;
        void flushTLBPageTableEntry(Ordinal offsetFromSegmentBase, Ordinal ssofSegmentThatContainsPage) noexcept;
        void flushTLBPhysicalPage(Ordinal basePhysicalAddressOfPage) noexcept;
        void flushTLBSegmentEntry(Ordinal ssForSegment) noexcept;
        void modifyProcessControls(Ordinal newProcessorControlWords, Ordinal mask) noexcept;
        void preemptProcess() noexcept;
        void restartProcessor(Ordinal segmentTableBase, Ordinal prcbBase) noexcept;
        void stopProcessor() noexcept;
        void storeProcessor() noexcept;
        void warmstartProcessor(Ordinal segmentTableBase, Ordinal prcbBase) noexcept;
    protected:
        // instructions
        void syncf() noexcept;
        void mark() noexcept;
        void fmark() noexcept;
        void synld(Register& dest, Ordinal src) noexcept;
        void synmov(Register& dest, Ordinal src) noexcept;
        void synmovl(Register& dest, Ordinal src) noexcept;
        void synmovq(Register& dest, Ordinal src) noexcept;
        void sysctl(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        template<bool doScan>
        inline void
        xbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            for (Ordinal index = 0; index < 32; ++index) {
                if ((src1 & computeBitPosition(31 - index)) != 0) {
                    if constexpr (doScan) {
                        dest.o = (31 - index);
                        ac_.arith.conditionCode = 0b010;
                        return;
                    }
                } else {
                    if constexpr (!doScan) {
                        dest.o = (31 - index);
                        ac_.arith.conditionCode = 0b010;
                        return;
                    }
                }
            }
            dest.o = 0xFFFF'FFFF;
            ac_.arith.conditionCode = 0;
        }
        void scanbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            xbit<true>(dest, src1, src2);
        }
        void spanbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            xbit<false>(dest, src1, src2);
        }
        void branch() noexcept;
        void branchConditional(bool condition) noexcept;
        void scanbyte(Ordinal src2, Ordinal src1) noexcept;
        void emul(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void ediv(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) noexcept;
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
                ac_.arith.conditionCode = checkClear ? 0b000 : 0b010;
                Register temp;
                temp.alignedTransfer.important = instruction_.cobr.displacement;
                ip_.alignedTransfer.important = ip_.alignedTransfer.important + temp.alignedTransfer.important;
                ip_.alignedTransfer.aligned = 0;
                advanceBy_ = 0;
            } else {
                ac_.arith.conditionCode = checkClear ? 0b010 : 0b000;
            }
        }
        void bbs() noexcept;
        void bbc() noexcept;
        template<typename T>
        void cmpGeneric(T src1, T src2) noexcept {
            if (src1 < src2) {
                ac_.arith.conditionCode = 0b100;
            } else if (src1 == src2) {
                ac_.arith.conditionCode = 0b010;
            } else {
                ac_.arith.conditionCode = 0b001;
            }
        }
        template<typename T>
        void cmpxbGeneric() noexcept {
            auto src1 = unpackSrc1_COBR(TreatAs<T>{});
            auto src2 = unpackSrc2_COBR(TreatAs<T>{});
            cmpGeneric(src1, src2);
            if ((instruction_.instGeneric.mask & ac_.getConditionCode()) != 0) {
                Register temp;
                temp.alignedTransfer.important = instruction_.cobr.displacement;
                ip_.alignedTransfer.important = ip_.alignedTransfer.important + temp.alignedTransfer.important;
                ip_.alignedTransfer.aligned = 0;
                advanceBy_ = 0;
            }
        }
        inline void cmpobGeneric() noexcept { cmpxbGeneric<Ordinal>(); }
        inline void cmpibGeneric() noexcept { cmpxbGeneric<Integer>(); }
        void flushreg() noexcept;
        void bx() noexcept;
        void balx() noexcept;
        void calls(Ordinal value) noexcept;
        void ldl() noexcept;
        void ldq() noexcept;
        void ldt() noexcept;
        void stq() noexcept;
        void stt() noexcept;
        void stl() noexcept;
        void ret() noexcept;
        void call() noexcept;
        void callx() noexcept;
    protected:
        void enterCall(Ordinal fp) noexcept;
        void leaveCall() noexcept;
        void allocateNewRegisterFrame() noexcept;
        bool registerSetAvailable() noexcept;
        void saveRegisterSet(Ordinal fp) noexcept;
        void restoreRegisterSet(Ordinal fp) noexcept;
        void restoreStandardFrame() noexcept;
        void storeBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept;
        void loadBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept;
    protected:
        void performConditionalSubtract(Register& dest, Integer src1, Integer src2, bool condition, TreatAsInteger) noexcept;
        void performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, bool condition, TreatAsOrdinal) noexcept;
        void performConditionalAdd(Register& dest, Integer src1, Integer src2, bool condition, TreatAsInteger) noexcept;
        void performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, bool condition, TreatAsOrdinal) noexcept;
        void performSelect(Register& dest, Ordinal src1, Ordinal src2, bool condition) noexcept;
    protected:
#define X(type) \
        type load(Address addr, TreatAs< type > ) const; \
        void store(Address addr, type value, TreatAs< type > )
        X(Integer);
        X(Ordinal);
        X(ByteInteger);
        X(ByteOrdinal);
        X(ShortInteger);
        X(ShortOrdinal);
#undef X
    protected:
        template<bool invert = false>
        inline void orOperation(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            destination.setValue(::orOperation<Ordinal, invert>(src2, src1), TreatAsOrdinal{});
        }
        template<bool invert = false>
        inline void andOperation(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            destination.setValue(::andOperation<Ordinal, invert>(src2, src1), TreatAsOrdinal{});
        }
        template<bool invert = false>
        inline void xorOperation(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            destination.setValue(::xorOperation<Ordinal, invert>(src2, src1), TreatAsOrdinal{});
        }
        template<typename T>
        void add(Register& destination, T src1, T src2, TreatAs<T>) noexcept {
            destination.setValue(::addOperation<T>(src2, src1), TreatAs<T>{});
        }
        template<typename T>
        void sub(Register& destination, T src1, T src2, TreatAs<T>) noexcept {
            destination.setValue(::subOperation<T>(src2, src1), TreatAs<T>{});
        }
        template<typename T>
        void mult(Register& destination, T src1, T src2, TreatAs<T>) noexcept {
            destination.setValue(::multiplyOperation<T>(src2, src1), TreatAs<T>{});
        }
        void setbit(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            // setbit is src2 | computeBitPosition(src1o)
            orOperation(destination, computeBitPosition(src1), src2);
        }

        void nor(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            orOperation<true>(destination, src1, src2);
        }
        void nand(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            andOperation<true>(destination, src1, src2);
        }
        inline void xnor(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            xorOperation<true>(destination, src1, src2);
        }
        inline void notbit(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            // notbit is src2 ^ computeBitPosition(src1)
            xorOperation(destination, computeBitPosition(src1), src2);
        }
        inline void ornot(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            orOperation(dest, ~src1, src2);
        }
        inline void notor(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            orOperation(dest, src1, ~src2);
        }
        inline void notOperation(Register& destination, Ordinal src) noexcept {
            destination.setValue(~src, TreatAsOrdinal{});
        }

        inline void andnot(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            andOperation(dest, ~src1, src2);
        }
        inline void notand(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            andOperation(dest, src1, ~src2);
        }
        inline void clrbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            // clrbit is src2 & ~computeBitPosition(src1)
            // so lets use andnot
            andnot(dest, computeBitPosition(src1), src2);
        }
        inline void modi(Register& dest, Integer src1, Integer src2) noexcept {
            if (auto denominator = src1; denominator == 0) {
                generateFault(ZeroDivideFault);
            } else {
                auto numerator = src2;
                auto result = numerator - ((numerator / denominator) * denominator);
                if (((numerator * denominator) < 0) && (result != 0)) {
                    result += denominator;
                }
                dest.setValue<Integer>(result);
            }
        }
        inline void alterbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            if (auto s1 = computeBitPosition(src1); ac_.getConditionCode() & 0b010) {
                orOperation(dest, s1, src2);
            } else {
                andnot(dest, s1, src2);
            }
        }
        inline void addc(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            LongOrdinal result = static_cast<LongOrdinal>(src2) + static_cast<LongOrdinal>(src1);
            result += (ac_.getCarryBit() ? 1 : 0);
            dest.setValue(result, TreatAsOrdinal{});
            arithmeticWithCarryGeneric(static_cast<Ordinal>(result >> 32), 
                    mostSignificantBit(src2),
                    mostSignificantBit(src1),
                    mostSignificantBit(dest.getValue<Ordinal>()));
        }
        inline void subc(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            LongOrdinal result = static_cast<LongOrdinal>(src2) - static_cast<LongOrdinal>(src1) - 1;
            result += (ac_.getCarryBit() ? 1 : 0);
            dest.setValue(result, TreatAsOrdinal{});
            arithmeticWithCarryGeneric(static_cast<Ordinal>(result >> 32), 
                    mostSignificantBit(src2),
                    mostSignificantBit(src1),
                    mostSignificantBit(dest.getValue<Ordinal>()));
        }
        template<typename T>
        void remainderOperation(Register& dest, T src1, T src2) noexcept {
            if (src1 == 0) {
                /// @todo fix this
                generateFault(ZeroDivideFault);
            } else {
                // taken from the i960Sx manual
                //dest.setValue(src2 - ((src2 / src1) * src1), TreatAs<T>{});
                dest.setValue(src2 % src1, TreatAs<T>{});
            }
        }
        void remi(Register& dest, Integer src1, Integer src2) noexcept {
            remainderOperation<Integer>(dest, src1, src2);
        }
        void remo(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            remainderOperation<Ordinal>(dest, src1, src2);
        }
        template<typename T>
        void divideOperation(Register& dest, T src1, T src2) noexcept {
            if (src1 == 0) {
                /// @todo fix this
                generateFault(ZeroDivideFault);
            } else {
                dest.setValue(src2 / src1, TreatAs<T>{});
            }
        }
        void divi(Register& dest, Integer src1, Integer src2) noexcept {
            divideOperation<Integer>(dest, src1, src2);
        }
        void divo(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            divideOperation<Ordinal>(dest, src1, src2);
        }
        void atadd(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            syncf();
            lockBus();
            auto addr = maskValue<decltype(src1), 0xFFFF'FFFC>(src1) ;
            auto temp = load(addr, TreatAsOrdinal{});
            // adds the src (src2 internally) value to the value in memory location specified with the addr (src1 in this case) operand.
            // The initial value from memory is stored in dst (internally src/dst).
            Ordinal result = temp + src2;
            store(addr, result, TreatAsOrdinal{});
            dest.setValue(temp, TreatAsOrdinal{});
            unlockBus();
        }
        void atmod(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            syncf();
            lockBus();
            auto addr = maskValue<decltype(src1), 0xFFFF'FFFC>(src1) ;
            auto temp = load(addr, TreatAsOrdinal{});
            // copies the src/dest value (logical version) into the memory location specifeid by src1.
            // The bits set in the mask (src2) operand select the bits to be modified in memory. The initial
            // value from memory is stored in src/dest
            Ordinal result = modify(src2, dest.getValue<Ordinal>(), temp);
            store(addr, result, TreatAsOrdinal{});
            dest.setValue(temp, TreatAsOrdinal{});
            unlockBus();

        }
        void cmpo(Ordinal src1, Ordinal src2) noexcept {
            cmpGeneric(src1, src2);
        }
        void cmpinco(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            cmpGeneric(src1, src2);
            dest.setValue(src2 + 1, TreatAsOrdinal{});
        }
        void cmpdeco(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            cmpGeneric(src1, src2);
            dest.setValue(src2 - 1, TreatAsOrdinal{});
        }
        void cmpi(Integer src1, Integer src2) noexcept {
            cmpGeneric(src1, src2);
        }
        void cmpinci(Register& dest, Integer src1, Integer src2) noexcept {
            cmpGeneric(src1, src2);
            dest.setValue(src2 + 1, TreatAsInteger{});
        }
        void cmpdeci(Register& dest, Integer src1, Integer src2) noexcept {
            cmpGeneric(src1, src2);
            dest.setValue(src2 - 1, TreatAsInteger{});
        }
        template<typename T>
        void concmpGeneric(T src1, T src2) noexcept {
            if ((ac_.getConditionCode() & 0b100) == 0) {
                ac_.arith.conditionCode = src1 <= src2 ? 0b010 : 0b001;
            }
        }
        void concmpo(Ordinal src1, Ordinal src2) noexcept {
            concmpGeneric<Ordinal>(src1, src2);
        }
        void concmpi(Integer src1, Integer src2) noexcept {
            concmpGeneric<Integer>(src1, src2);
        }
    protected:
        bool performSelfTest() noexcept;
        void assertFailureState() noexcept;
        void deassertFailureState() noexcept;
        void generateFault(Ordinal faultCode) noexcept;
        void addi(Register& dest, Integer src1, Integer src2) noexcept;
        void addo(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    private:
        Ordinal faultPortValue_;
        Ordinal systemAddressTableBase_ = 0;
        Ordinal prcbAddress_ = 0;
        GPRBlock gpr_;
        RegisterBlock32 sfrs_;
        Register ip_;
        Register ac_; 
        Register pc_;
        Register tc_; 
        Register faultCode_; 
        Register instruction_;
        Register ictl_;
        byte advanceBy_;
        bool running_;
        byte instructionLength_ = 0;
};
#endif // end SIM5_CORE_H__
