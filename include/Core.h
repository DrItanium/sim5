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
template<uint8_t subtype>
constexpr Ordinal TraceFaultSubType = TraceFaultBase | subtype;
constexpr Ordinal InstructionTraceFault = TraceFaultSubType<0b00000010>;
constexpr Ordinal BranchTraceFault = TraceFaultSubType<0b00000100>;
constexpr Ordinal CallTraceFault = TraceFaultSubType<0b00001000>;
constexpr Ordinal ReturnTraceFault = TraceFaultSubType<0b00010000>;
constexpr Ordinal PrereturnTraceFault = TraceFaultSubType<0b00100000>;
constexpr Ordinal SupervisorTraceFault = TraceFaultSubType<0b01000000>;
constexpr Ordinal MarkTraceFault = TraceFaultSubType<0b10000000>;
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
    call = 0x09,
    ret = 0x0a,
    bal = 0x0b,
    // the lowest three bits are used to determine the kind of operation to
    // perform when it comes to compares, tests, faults, branches, etc
    // This maps perfectly to the condition code to be used
    bno = 0x10,
    bg = 0x11,
    be = 0x12,
    bge = 0x13,
    bl = 0x14,
    bne = 0x15,
    ble = 0x16,
    bo = 0x17,
    faultno = 0x18,
    faultg = 0x19,
    faulte = 0x1a,
    faultge = 0x1b,
    faultl = 0x1c,
    faultne = 0x1d,
    faultle = 0x1e,
    faulto = 0x1f,
    testno = 0x20,
    testg = 0x21,
    teste = 0x22,
    testge = 0x23,
    testl = 0x24,
    testne = 0x25,
    testle = 0x26,
    testo = 0x27,
    
    bbc = 0x30,
    cmpobg = 0x31,
    cmpobe = 0x32,
    cmpobge = 0x33,
    cmpobl = 0x34,
    cmpobne = 0x35,
    cmpoble = 0x36,
    bbs = 0x37,
    cmpibno = 0x38,
    cmpibg = 0x39,
    cmpibe = 0x3a,
    cmpibge = 0x3b,
    cmpibl = 0x3c,
    cmpibne = 0x3d,
    cmpible = 0x3e,
    cmpibo = 0x3f,
    // mem instructions are denoted to have the most significant bit of the
    // opcode set.
    //
    // The layout is somewhat baffling to me but there seems to be a method to
    // the madness the more I look at it.
    //
    // the v in the names means virtual, only found on the extended
    // architecture which I am noting here for completeness
    //
    // So the least significant bit of the major opcode denotes a virtual
    // memory operation if the lower part is 0,1,2,3 or 8,9,a,b
    // loads are always 0,1, 8, or 9  and stores are always  2, 3, a, or b
    ldob = 0x80, ldvob = 0x81,
    stob = 0x82, stvob = 0x83,
    bx = 0x84,
    balx = 0x85,
    callx = 0x86,

    ldos = 0x88, ldvos = 0x89,
    stos = 0x8a, stvos = 0x8b,
    lda = 0x8c,

    ld = 0x90, ldv = 0x91,
    st = 0x92, stv = 0x93,
    ldl = 0x98, ldvl = 0x99,
    stl = 0x9a, stvl = 0x9b,

    ldt = 0xa0, ldvt = 0xa1,
    stt = 0xa2, stvt = 0xa3,
    dcinva = 0xad,  // hx specific instruction

    ldq = 0xb0, ldvq = 0xb1,
    stq = 0xb2, stvq = 0xb3,

    ldib = 0xc0, ldvib = 0xc1,
    stib = 0xc2, stvib = 0xc3,
    ldis = 0xc8, ldvis = 0xc9,
    stis = 0xca, stvis = 0xcb,

    ldm = 0xd0, ldvm = 0xd1,
    stm = 0xd2, stvm = 0xd3,
    ldml = 0xd8, ldvml = 0xd9,
    stml = 0xda, stvml = 0xdb,

    ldmq = 0xf0, ldvmq = 0xf1,
    stmq = 0xf2, stvmq = 0xf3,
    // register operations
    // They have an extra 4 bits of opcode used to expand the instruction space
    // to kinda 12-bit, the range in the major opcode is 0x58->0x7F
    notbit = 0x580,
    andOperation = 0x581,
    andnot = 0x582,
    setbit = 0x583,
    notand = 0x584,
    // hole
    xorOperation = 0x586,
    orOperation = 0x587,
    nor = 0x588,
    xnor = 0x589,
    notOperation = 0x58a,
    ornot = 0x58b,
    clrbit = 0x58c,
    notor = 0x58d,
    nand = 0x58e,
    alterbit = 0x58f,

    addo = 0x590,
    addi = 0x591,
    subo = 0x592,
    subi = 0x593,
    cmpob = 0x594,
    cmpib = 0x595,
    cmpos = 0x596,
    cmpis = 0x597,
    shro = 0x598,
    // hole
    shrdi = 0x59a,
    shri = 0x59b,
    shlo = 0x59c,
    rotate = 0x59d,
    shli = 0x59e,
    // hole
    cmpo = 0x5a0,
    cmpi = 0x5a1,
    concmpo = 0x5a2,
    concmpi = 0x5a3,
    cmpinco = 0x5a4,
    cmpinci = 0x5a5,
    cmpdeco = 0x5a6,
    cmpdeci = 0x5a7,
    chktag = 0x5a8,
    cmpm = 0x5aa,
    scanbyte = 0x5ac,
    bswap = 0x5ad,
    chkbit = 0x5ae,

    addc = 0x5b0,
    subc = 0x5b2,
    intdis = 0x5b4,
    inten = 0x5b5,

    mov = 0x5cc, movm = 0x5cd,

    eshro = 0x5d8, 
    movl = 0x5dc, movlm = 0x5dd,

    movt = 0x5ec,
    
    movq = 0x5fc, movqm = 0x5fd,

    synmov = 0x600,
    synmovl = 0x601,
    synmovq = 0x602,
    cmpstr = 0x603,
    movqstr = 0x604,
    movstr = 0x605,

    atmod = 0x610,
    atrep = 0x611,
    atadd = 0x612,
    inspacc = 0x613,
    ldphy = 0x614,
    synld = 0x615,
    fill = 0x617,
    // 0x62 not used
    // Cx specific instructions
    sdma = 0x630,
    udma = 0x631,
    
    spanbit = 0x640,
    scanbit = 0x641,
    daddc = 0x642,
    dsubc = 0x643,
    dmovt = 0x644,
    modac = 0x645,
    condrec = 0x646,
    cread = 0x648,
    ldtypedef = 0x649,
    ldglobals = 0x64a,

    modify = 0x650,
    extract = 0x651,
    restrictOperation = 0x652,
    amplify = 0x653,
    modtc = 0x654,
    modpc = 0x655,
    receive = 0x656,
    ldcsp = 0x657,
    intctl = 0x658,
    sysctl = 0x659, // Jx instruction that I am going to support the same way
                    // we do IAC instructions. The format of the sysctl
                    // "packet" is nearly identical to an IAC with the order of
                    // fields in the first 32-bit being reversed
    iccctl = 0x65b,
    dcctl = 0x65c,

    halt = 0x65d, // page missing from the Hx manual but in the Jx manual...
                  // oops

    calls = 0x660,
    calld = 0x661,
    send = 0x662,
    sendserv = 0x663,
    resumeprcs = 0x664,
    schedprcs = 0x665,
    saveprcs = 0x666,
    condwait = 0x668,
    wait = 0x669,
    signal = 0x66a,
    mark = 0x66b,
    fmark = 0x66c,
    flushreg = 0x66d,
    syncf = 0x66f,

    emul = 0x670,
    ediv = 0x671,
    cvtadr = 0x672,
    ldtime = 0x673,
    cvtir = 0x674,
    cvtilr = 0x675,
    scalerl = 0x676,
    scaler = 0x677,

    atanr = 0x680,
    logepr = 0x681,
    logr = 0x682,
    remr = 0x683,
    cmpor = 0x684,
    cmpr = 0x685,
    sqrtr = 0x688,
    expr = 0x689,
    logbnr = 0x68a,
    roundr = 0x68b,
    sinr = 0x68c,
    cosr = 0x68d,
    tanr = 0x68e,
    classr = 0x68f,

    atanrl = 0x690,
    logeprl = 0x691,
    logrl = 0x692,
    remrl = 0x693,
    cmporl = 0x694,
    cmprl = 0x695,
    sqrtrl = 0x698,
    exprl = 0x699,
    logbnrl = 0x69a,
    roundrl = 0x69b,
    sinrl = 0x69c,
    cosrl = 0x69d,
    tanrl = 0x69e,
    classrl = 0x69f,

    cvtri = 0x6c0,
    cvtril = 0x6c1,
    cvtzri = 0x6c2,
    cvtzril = 0x6c3,
    movr = 0x6c9,

    movrl = 0x6d9,

    cpysre = 0x6e2,
    cpyrsre = 0x6e3,
    movre = 0x6e9,

    mulo = 0x700,
    remo = 0x708,
    divo = 0x70b,

    muli = 0x710,
    remi = 0x718,
    modi = 0x719,
    divi = 0x71b,

    divr = 0x78b,
    mulr = 0x78c,
    subr = 0x78d,
    addr = 0x78f,

    divrl = 0x79b,
    mulrl = 0x79c,
    subrl = 0x79d,
    addrl = 0x79f,

    // new core instructions
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
};
enum class BootResult : uint8_t {
    Success,
    SelfTestFailure,
    ChecksumFail,
};
struct TreatAsREG { };
struct TreatAsCOBR { };
struct TreatAsCTRL { };
struct TreatAsMEM { };
union Register {
    constexpr Register(Ordinal value) : o(value) { }
    Register() = default;
    Ordinal o;
    Integer i;
    Address a;
    byte bytes[sizeof(Ordinal)];
    ShortOrdinal shorts[sizeof(Ordinal)/sizeof(ShortOrdinal)];
    constexpr uint8_t getInstructionMask() const noexcept { 
        return bytes[3] & 0b111;
    }
    constexpr uint8_t getMajorOpcode() const noexcept {
        return bytes[3];
    }
    constexpr bool isCTRL() const noexcept {
        return bytes[3] < 0x20;
    }
    constexpr bool isCOBR() const noexcept {
        return (bytes[3] & 0b1110'0000) == 0b0010'0000;
    }
    struct {
        Integer aligned : 2;
        Integer important : 30;
    } alignedTransfer;
    struct {
        uint8_t s2 : 1;
        uint8_t t : 1;
        Integer displacement : 11;
        uint8_t m1: 1;
        uint8_t src2: 5;
        uint8_t src1: 5;
        uint8_t opcode : 8;
    } cobr;
    struct {
        uint8_t src1 : 5;
        uint8_t s1 : 1;
        uint8_t s2 : 1;
        uint8_t opcodeExt : 4;
        uint8_t m1 : 1;
        uint8_t m2 : 1;
        uint8_t m3 : 1;
        uint8_t src2 : 5;
        uint8_t srcDest : 5;
        uint8_t opcode : 8;
    } reg;
    struct {
        Integer displacement : 24;
        uint8_t opcode : 8;
    } ctrl;
    struct {
        Ordinal offset: 12;
        uint8_t selector : 1;
        uint8_t selector2 : 1;
        uint8_t abase : 5;
        uint8_t srcDest : 5;
        uint8_t opcode : 8;
    } mem;
    struct {
        Ordinal offset : 12;
        uint8_t fixed : 1;
        uint8_t action : 1;
        uint8_t abase : 5;
        uint8_t srcDest : 5;
        uint8_t opcode : 8;
    } mema;
    struct {
        uint8_t index : 5;
        uint8_t unused : 2;
        uint8_t scale : 3;
        uint8_t modeMinor : 2;
        uint8_t fixed : 1;
        uint8_t group: 1;
        uint8_t abase : 5;
        uint8_t srcDest : 5;
        uint8_t opcode : 8;
    } memb;
    struct {
        uint8_t index : 5;
        uint8_t unused : 2;
        uint8_t scale : 3;
        uint8_t registerIndirect : 1;
        uint8_t useIndex : 1;
        uint8_t fixed : 1;
        uint8_t group: 1;
        uint8_t abase : 5;
        uint8_t srcDest : 5;
        uint8_t opcode : 8;
    } memb_grp2;
    struct {
        uint8_t conditionCode : 3;
        uint8_t arithmeticStatus : 4;
        uint8_t unused0 : 1;
        uint8_t integerOverflowFlag : 1;
        uint8_t unused1 : 3;
        uint8_t integerOverflowMask : 1;
        uint8_t unused2 : 2;
        uint8_t noImpreciseFaults : 1;
        uint8_t floatingOverflowFlag : 1;
        uint8_t floatingUnderflowFlag : 1;
        uint8_t floatingInvalidOpFlag : 1;
        uint8_t floatingZeroDivideFlag : 1;
        uint8_t floatingInexactFlag : 1;
        uint8_t unused3 : 3;
        uint8_t floatingOverflowMask : 1;
        uint8_t floatingUnderflowMask : 1;
        uint8_t floatingInvalidOpMask : 1;
        uint8_t floatingZeroDivideMask : 1;
        uint8_t floatingInexactMask : 1;
        uint8_t floatingPointNormalizingMode : 1;
        uint8_t floatingPointRoundingControl : 2;
    } arith;

    struct {
        uint8_t rt : 3;
        uint8_t p : 1;
        uint8_t unused : 2; // according to the Sx manual these bits go unused
                            // but in the Hx manual they are used :/
        Ordinal a : 26;
    } pfp;
    struct {
        Ordinal align : 6;
        Ordinal proper : 26;
    } pfpAddress;
    struct {
        uint8_t traceEnable : 1;
        uint8_t executionMode : 1;
        uint8_t unused : 7;
        uint8_t resume : 1;
        uint8_t traceFaultPending : 1;
        uint8_t unused1 : 2;
        uint8_t state : 1;
        uint8_t unused2 : 2;
        uint8_t priority : 5;
        Ordinal internalState : 11;
    } processControls;
    struct {
        uint8_t unused0 : 1; // 0
        uint8_t instructionTraceMode : 1; // 1
        uint8_t branchTraceMode : 1; // 2
        uint8_t callTraceMode : 1; // 3
        uint8_t returnTraceMode : 1; // 4
        uint8_t prereturnTraceMode : 1; // 5
        uint8_t supervisorTraceMode : 1; // 6
        uint8_t breakpointTraceMode : 1; // 7
        Ordinal unused1 : 9; // 8, 9, 10, 11, 12, 13, 14, 15, 16
        uint8_t instructionTraceEvent : 1; // 17
        uint8_t branchTraceEvent : 1; // 18
        uint8_t callTraceEvent : 1; // 19
        uint8_t returnTraceEvent : 1; // 20
        uint8_t prereturnTraceEvent : 1; // 21
        uint8_t supervisorTraceEvent : 1; // 22
        uint8_t breakpointTraceEvent : 1; // 23
        uint8_t unused2 : 8;
    } trace;
    uint8_t interruptControl[4];
    constexpr auto getConditionCode() const noexcept { return arith.conditionCode; }
    void setPriority(Ordinal value) noexcept { processControls.priority = value; }
    constexpr bool inSupervisorMode() const noexcept { return processControls.executionMode; }
    constexpr bool inUserMode() const noexcept { return !inSupervisorMode(); }
    constexpr bool isMEMA() const noexcept { return !mem.selector; }
    constexpr bool isMEMB() const noexcept { return mem.selector; }
    constexpr bool isDoubleWide() const noexcept {
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
    [[nodiscard]] constexpr bool isMEMFormat() const noexcept {
        return bytes[3] >= 0x80;
    }
    [[nodiscard]] constexpr auto isREGFormat() const noexcept {
        return bytes[3] >= 0x58 && bytes[3] < 0x80;
    }
    [[nodiscard]] constexpr auto getOpcode() const noexcept { 
        if (isREGFormat()) {
            uint16_t baseValue = static_cast<uint16_t>(getMajorOpcode()) << 4;
            return static_cast<Opcodes>(baseValue | static_cast<uint16_t>(reg.opcodeExt));
        } else {
            return static_cast<Opcodes>(getMajorOpcode());
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
    template<typename T>
    explicit constexpr operator T() const noexcept {
        return getValue(TreatAs<T>{});
    }

    explicit constexpr operator Opcodes() const noexcept {
        return getOpcode();
    }
    byte& operator[](byte index) noexcept {
        return bytes[index & 0b11];
    }
    constexpr const byte& operator[](byte index) const noexcept {
        return bytes[index & 0b11];
    }
};
static_assert(sizeof(Register) == sizeof(Ordinal));
union LongRegister {
    public:
        LongRegister() = default;
        [[nodiscard]] constexpr LongOrdinal getValue(TreatAsLongOrdinal) const noexcept { return lo; }
        [[nodiscard]] constexpr LongInteger getValue(TreatAsLongInteger) const noexcept { return li; }
        void setValue(LongOrdinal value, TreatAsLongOrdinal) noexcept { lo = value; }
        void setValue(LongInteger value, TreatAsLongInteger) noexcept { li = value; }
        Register& get(byte index) noexcept { return pair_[index & 0b1]; }
        const Register& get(byte index) const noexcept { return pair_[index & 0b1]; }

        template<typename T>
        void setValue(byte index, T value) noexcept {
            get(index).setValue<T>(value);
        }
        template<typename T>
        constexpr T getValue(byte index) const noexcept {
            return get(index).getValue<T>();
        }
        template<typename T>
        constexpr T getValue() const noexcept {
            return getValue(TreatAs<T>{});
        }
        template<typename T>
        void setValue(T value) noexcept {
            setValue(value, TreatAs<T>{});
        }
        Register& operator[](byte index) noexcept {
            return get(index);
        }
        const Register& operator[](byte index) const noexcept {
            return get(index);
        }
    private:
        Register pair_[2];
        LongOrdinal lo;
        LongInteger li;
};
static_assert(sizeof(LongRegister) == sizeof(LongOrdinal));

using TreatAsLongRegister = TreatAs<LongRegister>;
using TreatAsRegister = TreatAs<Register>;

union QuadRegister {
    public:
        QuadRegister() = default;

        template<typename T>
        void setValue(byte index, T value, TreatAsRegister) noexcept {
            get(index, TreatAsRegister{}). setValue<T>(value);
        }
        template<typename T>
        constexpr T getValue(byte index, TreatAsRegister) const noexcept {
            return get(index, TreatAsRegister{}).getValue<T>();
        }
        template<typename T>
        void setValue(byte index, T value, TreatAsLongRegister) noexcept {
            get(index, TreatAsLongRegister{}). setValue<T>(value);
        }
        template<typename T>
        constexpr T getValue(byte index, TreatAsLongRegister) const noexcept {
            return get(index, TreatAsLongRegister{}).getValue<T>();
        }
        Register& get(byte index, TreatAsRegister) noexcept { return quads_[index & 0b11]; }
        const Register& get(byte index, TreatAsRegister) const noexcept { return quads_[index & 0b11]; }
        LongRegister& get(byte index, TreatAsLongRegister) noexcept { return pairs_[index & 0b1]; }
        const LongRegister& get(byte index, TreatAsLongRegister) const noexcept { return pairs_[index & 0b1]; }
    private:
        Register quads_[4];
        LongRegister pairs_[2];
};
static_assert(sizeof(QuadRegister) == (2*sizeof(LongOrdinal)));
using TreatAsQuadRegister = TreatAs<QuadRegister>;
using TreatAsTripleRegister = TreatAsQuadRegister;
using TripleRegister = QuadRegister;
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
union RegisterFrame {
    public:
        RegisterFrame() = default;
        Register& get(byte index, TreatAsRegister) noexcept { return registers[index & 0b1111]; }
        const Register& get(byte index, TreatAsRegister) const noexcept { return registers[index & 0b1111]; }
        LongRegister& get(byte index, TreatAsLongRegister) noexcept { return longRegisters[index >> 1]; }
        const LongRegister& get(byte index, TreatAsLongRegister) const noexcept { return longRegisters[index >> 1]; }
        QuadRegister& get(byte index, TreatAsQuadRegister) noexcept { return quadRegisters[index >> 2]; }
        const QuadRegister& get(byte index, TreatAsQuadRegister) const noexcept { return quadRegisters[index >> 2]; }
    private:
        Register registers[16];
        LongRegister longRegisters[8];
        QuadRegister quadRegisters[4];
};
/**
 * @brief is the given byte index aligned to long register boundaries?
 */
constexpr bool aligned(byte index, TreatAsLongRegister) noexcept { return (index & 0b1) == 0; }
/**
 * @brief is the given byte index aligned to quad register boundaries?
 */
constexpr bool aligned(byte index, TreatAsQuadRegister) noexcept { return (index & 0b11) == 0; }
/** 
 * @brief Holds onto two separate register frames
 */ 
class GPRBlock {
    public:
        GPRBlock() = default;
        Register& get(byte index, TreatAsRegister) noexcept { 
            if (index < 16) {
                return globals.get(index, TreatAsRegister{});
            } else {
                return locals.get(index, TreatAsRegister{});
            }
        }
        const Register& get(byte index, TreatAsRegister) const noexcept { 
            if (index < 16) {
                return globals.get(index, TreatAsRegister{});
            } else {
                return locals.get(index, TreatAsRegister{});
            }
        }
        Register& get(byte index) noexcept { 
            return get(index, TreatAsRegister{});
        }
        const Register& get(byte index) const noexcept { 
            return get(index, TreatAsRegister{});
        }
        LongRegister& get(byte index, TreatAsLongRegister) noexcept { 
            if (index < 16) {
                return globals.get(index, TreatAsLongRegister{});
            } else {
                return locals.get(index, TreatAsLongRegister{});
            }
        }
        const LongRegister& get(byte index, TreatAsLongRegister) const noexcept { 
            if (index < 16) {
                return globals.get(index, TreatAsLongRegister{});
            } else {
                return locals.get(index, TreatAsLongRegister{});
            }
        }
        QuadRegister& get(byte index, TreatAsQuadRegister) noexcept { 
            if (index < 16) {
                return globals.get(index, TreatAsQuadRegister{});
            } else {
                return locals.get(index, TreatAsQuadRegister{});
            }
        }
        const QuadRegister& get(byte index, TreatAsQuadRegister) const noexcept { 
            if (index < 16) {
                return globals.get(index, TreatAsQuadRegister{});
            } else {
                return locals.get(index, TreatAsQuadRegister{});
            }
        }


        template<typename T>
        void setValue(byte index, T value, TreatAsRegister) noexcept {
            get(index, TreatAsRegister{}).setValue(value, TreatAs<T>{});
        }
        template<typename T>
        T getValue(byte index, TreatAsRegister) const noexcept {
            return get(index, TreatAsRegister{}).getValue(TreatAs<T>{});
        }

        template<typename T>
        void setValue(byte index, T value) noexcept {
            setValue<T>(index, value, TreatAsRegister{});
        }
        template<typename T>
        T getValue(byte index) const noexcept {
            return getValue<T>(index, TreatAsRegister{});
        }

        template<typename T>
        void setValue(byte index, T value, TreatAsLongRegister) noexcept {
            get(index, TreatAsLongRegister{}).setValue(value, TreatAs<T>{});
        }
        template<typename T>
        T getValue(byte index, TreatAsLongRegister) const noexcept {
            return get(index, TreatAsLongRegister{}).getValue(TreatAs<T>{});
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
    private:
        void lockBus();
        void unlockBus();
        void signalBootFailure();
        void setFaultPort(Ordinal value) noexcept;
        /// @todo insert iac dispatch here
        /// @todo insert routines for getting registers and such 
        [[nodiscard]] QuadRegister& getGPR(byte index, TreatAsQuadRegister) noexcept { return gpr_.get(index, TreatAsQuadRegister{}); }
        [[nodiscard]] const QuadRegister& getGPR(byte index, TreatAsQuadRegister) const noexcept { return gpr_.get(index, TreatAsQuadRegister{}); }
        [[nodiscard]] LongRegister& getGPR(byte index, TreatAsLongRegister) noexcept { return gpr_.get(index, TreatAsLongRegister{}); }
        [[nodiscard]] const LongRegister& getGPR(byte index, TreatAsLongRegister) const noexcept { return gpr_.get(index, TreatAsLongRegister{}); }
        [[nodiscard]] Register& getGPR(byte index) noexcept { return gpr_.get(index); }
        [[nodiscard]] Register& getGPR(byte index, byte offset) noexcept { return getGPR((index + offset) & 0b11111); }
        [[nodiscard]] const Register& getGPR(byte index) const noexcept { return gpr_.get(index); }
        [[nodiscard]] const Register& getGPR(byte index, byte offset) const noexcept { return getGPR((index + offset) & 0b11111); }
        [[nodiscard]] inline Ordinal getGPRValue(byte index, TreatAsOrdinal) const noexcept { return getGPR(index).getValue(TreatAsOrdinal{}); }
        [[nodiscard]] inline Ordinal getGPRValue(byte index, byte offset, TreatAsOrdinal) const noexcept { return getGPR(index, offset).getValue(TreatAsOrdinal{}); }
        [[nodiscard]] inline Integer getGPRValue(byte index, TreatAsInteger) const noexcept { return getGPR(index).getValue(TreatAsInteger{}); }
        [[nodiscard]] constexpr Ordinal getSystemAddressTableBase() const noexcept { return systemAddressTableBase_; }
        [[nodiscard]] Ordinal getSystemProcedureTableBase() const noexcept;
        [[nodiscard]] Ordinal getSupervisorStackPointer() const noexcept;
        void restoreRegisterSet() noexcept;
        inline void setGPR(byte index, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index).setValue(value, TreatAsOrdinal{}); }
        inline void setGPR(byte index, byte offset, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index, offset).setValue(value, TreatAsOrdinal{}); }
        inline void setGPR(byte index, Integer value, TreatAsInteger) noexcept { getGPR(index).setValue(value, TreatAsInteger{}); }
        [[nodiscard]] Register& getSFR(byte index) noexcept;
        [[nodiscard]] Register& getSFR(byte index, byte offset) noexcept;
        [[nodiscard]] const Register& getSFR(byte index) const noexcept;
        [[nodiscard]] const Register& getSFR(byte index, byte offset) const noexcept;
        [[nodiscard]] const Register& getSrc1Register(TreatAsREG) const noexcept;
        [[nodiscard]] const Register& getSrc2Register(TreatAsREG) const noexcept;
        [[nodiscard]] Ordinal unpackSrc1(TreatAsOrdinal, TreatAsREG) noexcept;
        [[nodiscard]] Ordinal unpackSrc1(byte offset, TreatAsOrdinal, TreatAsREG) noexcept;
        [[nodiscard]] Integer unpackSrc1(TreatAsInteger, TreatAsREG) noexcept;
        [[nodiscard]] Ordinal unpackSrc2(TreatAsOrdinal, TreatAsREG) noexcept;
        [[nodiscard]] Integer unpackSrc2(TreatAsInteger, TreatAsREG) noexcept;
        [[nodiscard]] Ordinal unpackSrc1(TreatAsOrdinal, TreatAsCOBR) noexcept;
        [[nodiscard]] Integer unpackSrc1(TreatAsInteger, TreatAsCOBR) noexcept;
        [[nodiscard]] Ordinal unpackSrc2(TreatAsOrdinal, TreatAsCOBR) noexcept;
        [[nodiscard]] Integer unpackSrc2(TreatAsInteger, TreatAsCOBR) noexcept;
        void checkForPendingInterrupts() noexcept;
        template<typename T>
        void moveGPR(byte destIndex, byte srcIndex, TreatAs<T>) noexcept {
            setGPR(destIndex, getGPRValue(srcIndex, TreatAs<T>{}), TreatAs<T>{});
        }
        template<typename T>
        void moveGPR(byte destIndex, byte destOffset, byte srcIndex, byte srcOffset, TreatAs<T>) noexcept {
            setGPR(destIndex, destOffset, getGPRValue(srcIndex, srcOffset, TreatAs<T>{}), TreatAs<T>{});
        }
        [[nodiscard]] bool getMaskedConditionCode(uint8_t mask) const noexcept;
        [[nodiscard]] bool conditionCodeEqualsMask(uint8_t mask) const noexcept;
        bool fullConditionCodeCheck() noexcept;
        bool fullConditionCodeCheck(uint8_t mask) noexcept;
        Ordinal computeAddress() noexcept;
        void performRegisterTransfer(byte mask, byte count) noexcept;
    private:
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
    private:
        // instructions
        void syncf() noexcept;
        void mark() noexcept;
        void fmark() noexcept;
        void synld(Register& dest, Ordinal src) noexcept;
        void synmov(const Register& dest, Ordinal src) noexcept;
        void synmovl(const Register& dest, Ordinal src) noexcept;
        void synmovq(const Register& dest, Ordinal src) noexcept;
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
        void branch(Integer displacement) noexcept;
        void branchConditional(bool condition, Integer displacement) noexcept;
        void scanbyte(Ordinal src2, Ordinal src1) noexcept;
        void emul(LongRegister& dest, Ordinal src1, Ordinal src2) noexcept;
        void ediv(LongRegister& dest, Ordinal src1, const LongRegister& src2) noexcept;
        void arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) noexcept;
        inline void advanceCOBRDisplacement(int16_t displacement) noexcept {
            Register temp{0};
            temp.alignedTransfer.important = displacement;
            ip_.alignedTransfer.important += temp.alignedTransfer.important;
            ip_.alignedTransfer.aligned = 0;
            advanceInstruction_ = false;
        }
        template<bool checkClear>
        void 
        branchIfBitGeneric(Ordinal bitpos, const Register& src2 , int16_t displacement) {
            //Ordinal bitpos = computeBitPosition(unpackSrc1(TreatAsOrdinal{}, TreatAsCOBR{}));
            //Ordinal against = unpackSrc2(TreatAsOrdinal{}, TreatAsCOBR{});
            Ordinal against = src2.getValue<Ordinal>();
            bool condition = false;
            // Branch if bit set
            if constexpr (checkClear) {
                condition = (bitpos & against) == 0;
            } else {
                condition = (bitpos & against) != 0;
            }
            if (condition) {
                ac_.arith.conditionCode = checkClear ? 0b000 : 0b010;
                advanceCOBRDisplacement(displacement);
            } else {
                ac_.arith.conditionCode = checkClear ? 0b010 : 0b000;
            }
        }
        inline void bbc(uint8_t bitpos, const Register& against, int16_t displacement) {
            return branchIfBitGeneric<true>(computeBitPosition(bitpos), against, displacement);
        }
        inline void bbc(const Register& bitpos, const Register& against, int16_t displacement) {
            return branchIfBitGeneric<true>(computeBitPosition(bitpos.bytes[0]), against, displacement);
        }
        inline void bbs(uint8_t bitpos, const Register& against, int16_t displacement) {
            return branchIfBitGeneric<false>(computeBitPosition(bitpos), against, displacement);
        }
        inline void bbs(const Register& bitpos, const Register& against, int16_t displacement) {
            return branchIfBitGeneric<false>(computeBitPosition(bitpos.bytes[0]), against, displacement);
        }
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
        void cmpxbGeneric(uint8_t mask, T src1, T src2, int16_t displacement, TreatAs<T>) noexcept {
            cmpGeneric<T>(src1, src2);
            if ((mask & ac_.getConditionCode()) != 0) {
                advanceCOBRDisplacement(displacement);
            } 
        }
        inline void cmpobGeneric(uint8_t mask, Ordinal src1, Ordinal src2, int16_t displacement) noexcept {
            cmpxbGeneric(mask, src1, src2, displacement, TreatAsOrdinal{}); 
        }
        inline void cmpibGeneric(uint8_t mask, Integer src1, Integer src2, int16_t displacement) noexcept { 
            cmpxbGeneric(mask, src1, src2, displacement, TreatAsInteger{});
        }
        void flushreg() noexcept;
        void balx(byte linkRegister, Ordinal branchTo) noexcept;
        void calls(Ordinal value) noexcept;
        void ldl(Address address, LongRegister& destination) noexcept;
        void ldq(Address address, QuadRegister& destination) noexcept;
        void ldt(Address address, TripleRegister& destination) noexcept;
        void stq(Address address, const QuadRegister& src) noexcept;
        void stt(Address address, const TripleRegister& src) noexcept;
        void stl(Address address, const LongRegister& src) noexcept;
        void ret() noexcept;
        void call(Integer displacement) noexcept;
        void callx() noexcept;
        void callx(Address effectiveAddress) noexcept;
    private:
        void enterCall(Ordinal fp) noexcept;
        void leaveCall() noexcept;
        void allocateNewRegisterFrame() noexcept;
        bool registerSetAvailable() noexcept;
        void saveRegisterSet(Ordinal fp) noexcept;
        void restoreRegisterSet(Ordinal fp) noexcept;
        void restoreStandardFrame() noexcept;
        void storeBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept;
        void loadBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept;
    private:
        void performConditionalSubtract(Register& dest, Integer src1, Integer src2, TreatAsInteger) noexcept;
        void performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) noexcept;
        void performConditionalAdd(Register& dest, Integer src1, Integer src2, TreatAsInteger) noexcept;
        void performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) noexcept;
        void performSelect(Register& dest, Ordinal src1, Ordinal src2) noexcept;
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
        X(LongInteger);
        X(LongOrdinal);
#undef X
    private:
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
                nextInstruction();
                /// @todo implement fault checks
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
                nextInstruction();
            }
        }
        void remi(Register& dest, Integer src1, Integer src2) noexcept {
            remainderOperation<Integer>(dest, src1, src2);
            /// @todo implement overflow checks
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
                nextInstruction();
            }
        }
        void divi(Register& dest, Integer src1, Integer src2) noexcept {
            divideOperation<Integer>(dest, src1, src2);
            /// @todo implement overflow fault checking
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
    private:
        bool performSelfTest() noexcept;
        void assertFailureState() noexcept;
        void deassertFailureState() noexcept;
        void generateFault(Ordinal faultCode) noexcept;
        void addi(Register& dest, Integer src1, Integer src2) noexcept;
        void addo(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void saveReturnAddress(byte registerIndex) noexcept;
        void saveReturnAddress(Register& linkRegister) noexcept;
        void setupNewFrameInternals(Ordinal fp, Ordinal temp) noexcept;
        Ordinal getStackPointer() const noexcept;
        Ordinal getNextFrameBase() const noexcept;
        void setStackPointer(Ordinal value, TreatAsOrdinal) noexcept;
        /**
         * @brief Advance ip by instruction length and then prevent further
         * advancement until the next cycle!
         */
        void nextInstruction() noexcept;
        void setIP(Ordinal value, TreatAsOrdinal) noexcept;
        void subo(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void subi(Register& dest, Integer src1, Integer src2) noexcept;
        void faultGeneric() noexcept;
        void balx(Register& linkRegister, Address ordinal) noexcept;
        void processInstruction(Opcodes opcode, Integer displacement, TreatAsCTRL) noexcept;
        void processInstruction(Opcodes opcode, uint8_t mask, uint8_t src1, const Register& src2, int16_t displacement, TreatAsCOBR) noexcept;
        void processInstruction(Opcodes opcode, uint8_t mask, Register& src1, const Register& src2, int16_t displacement, TreatAsCOBR) noexcept;
        void processInstruction(Opcodes opcode, Register& srcDest, Address effectiveAddress, TreatAsMEM) noexcept;
        void processInstruction(Opcodes opcode, Register& srcDest, const Register& src1, const Register& src2, TreatAsREG) noexcept;
    private:
        void modpc(Register& dest, Ordinal src1o, Ordinal src2o) noexcept;
        void modxc(Register& control, Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void shlo(Register& srcDest, Ordinal src1, Ordinal src2) noexcept;
        void shli(Register& srcDest, Integer src1, Integer src2) noexcept;
        void rotate(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void shri(Register& dest, Integer src1, Integer src2) noexcept;
    private:
        Ordinal systemAddressTableBase_ = 0;
        Ordinal prcbAddress_ = 0;
        GPRBlock gpr_;
        RegisterBlock32 sfrs_;
        RegisterBlock32 constants_;
        Register ip_;
        Register ac_; 
        Register pc_;
        Register tc_; 
        Register instruction_;
        Register ictl_;
        byte advanceBy_;
        bool running_;
        byte instructionLength_ = 0;
        bool advanceInstruction_ = false;
};
#endif // end SIM5_CORE_H__
