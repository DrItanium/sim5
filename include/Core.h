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

constexpr Ordinal DEFAULT_SALIGN = 4;
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
#ifdef __AVR__
    using BackingUnionType = uint8_t;
#else
    using BackingUnionType = Ordinal;
#endif
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
        BackingUnionType s2 : 1;
        BackingUnionType t : 1;
        Integer displacement : 11;
        BackingUnionType m1: 1;
        BackingUnionType src2: 5;
        BackingUnionType src1: 5;
        BackingUnionType opcode : 8;
    } cobr;
    struct {
        BackingUnionType src1 : 5;
        BackingUnionType s1 : 1;
        BackingUnionType s2 : 1;
        BackingUnionType opcodeExt : 4;
        BackingUnionType m1 : 1;
        BackingUnionType m2 : 1;
        BackingUnionType m3 : 1;
        BackingUnionType src2 : 5;
        BackingUnionType srcDest : 5;
        BackingUnionType opcode : 8;
    } reg;
    struct {
        Integer displacement : 24;
        BackingUnionType opcode : 8;
    } ctrl;
    struct {
        Ordinal offset: 12;
        BackingUnionType selector : 1;
        BackingUnionType selector2 : 1;
        BackingUnionType abase : 5;
        BackingUnionType srcDest : 5;
        BackingUnionType opcode : 8;
    } mem;
    struct {
        Ordinal offset : 12;
        BackingUnionType fixed : 1;
        BackingUnionType action : 1;
        BackingUnionType abase : 5;
        BackingUnionType srcDest : 5;
        BackingUnionType opcode : 8;
    } mema;
    struct {
        BackingUnionType index : 5;
        BackingUnionType unused : 2;
        BackingUnionType scale : 3;
        BackingUnionType modeMinor : 2;
        BackingUnionType fixed : 1;
        BackingUnionType group: 1;
        BackingUnionType abase : 5;
        BackingUnionType srcDest : 5;
        BackingUnionType opcode : 8;
    } memb;
    struct {
        BackingUnionType index : 5;
        BackingUnionType unused : 2;
        BackingUnionType scale : 3;
        BackingUnionType registerIndirect : 1;
        BackingUnionType useIndex : 1;
        BackingUnionType fixed : 1;
        BackingUnionType group: 1;
        BackingUnionType abase : 5;
        BackingUnionType srcDest : 5;
        BackingUnionType opcode : 8;
    } memb_grp2;
    struct {
        BackingUnionType conditionCode : 3;
        BackingUnionType arithmeticStatus : 4;
        BackingUnionType unused0 : 1;
        BackingUnionType integerOverflowFlag : 1;
        BackingUnionType unused1 : 3;
        BackingUnionType integerOverflowMask : 1;
        BackingUnionType unused2 : 2;
        BackingUnionType noImpreciseFaults : 1;
        BackingUnionType floatingOverflowFlag : 1;
        BackingUnionType floatingUnderflowFlag : 1;
        BackingUnionType floatingInvalidOpFlag : 1;
        BackingUnionType floatingZeroDivideFlag : 1;
        BackingUnionType floatingInexactFlag : 1;
        BackingUnionType unused3 : 3;
        BackingUnionType floatingOverflowMask : 1;
        BackingUnionType floatingUnderflowMask : 1;
        BackingUnionType floatingInvalidOpMask : 1;
        BackingUnionType floatingZeroDivideMask : 1;
        BackingUnionType floatingInexactMask : 1;
        BackingUnionType floatingPointNormalizingMode : 1;
        BackingUnionType floatingPointRoundingControl : 2;
    } arith;

    struct {
        BackingUnionType rt : 3;
        BackingUnionType p : 1;
        BackingUnionType unused : 2; // according to the Sx manual these bits go unused
                            // but in the Hx manual they are used :/
        Ordinal a : 26;
    } pfp;
    struct {
        Ordinal align : 6;
        Ordinal proper : 26;
    } pfpAddress;
    struct {
        BackingUnionType traceEnable : 1;
        BackingUnionType executionMode : 1;
        BackingUnionType unused : 7;
        BackingUnionType resume : 1;
        BackingUnionType traceFaultPending : 1;
        BackingUnionType unused1 : 2;
        BackingUnionType state : 1;
        BackingUnionType unused2 : 2;
        BackingUnionType priority : 5;
        Ordinal internalState : 11;
    } processControls;
    struct {
        BackingUnionType unused0 : 1; // 0
        BackingUnionType instructionTraceMode : 1; // 1
        BackingUnionType branchTraceMode : 1; // 2
        BackingUnionType callTraceMode : 1; // 3
        BackingUnionType returnTraceMode : 1; // 4
        BackingUnionType prereturnTraceMode : 1; // 5
        BackingUnionType supervisorTraceMode : 1; // 6
        BackingUnionType breakpointTraceMode : 1; // 7
        Ordinal unused1 : 9; // 8, 9, 10, 11, 12, 13, 14, 15, 16
        BackingUnionType instructionTraceEvent : 1; // 17
        BackingUnionType branchTraceEvent : 1; // 18
        BackingUnionType callTraceEvent : 1; // 19
        BackingUnionType returnTraceEvent : 1; // 20
        BackingUnionType prereturnTraceEvent : 1; // 21
        BackingUnionType supervisorTraceEvent : 1; // 22
        BackingUnionType breakpointTraceEvent : 1; // 23
        BackingUnionType unused2 : 8;
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
    Register& operator=(Ordinal value) noexcept {
        o = value;
        return *this;
    }
    constexpr bool operator==(const Register& other) const noexcept {
        return other.o == o;
    }
    constexpr bool operator!=(const Register& other) const noexcept {
        return other.o != o;
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
        template<typename T>
        explicit constexpr operator T() const noexcept {
            return getValue(TreatAs<T>{});
        }
        constexpr bool operator==(const LongRegister& other) const noexcept {
            return lo == other.lo;
        }
        constexpr bool operator!=(const LongRegister& other) const noexcept {
            return lo != other.lo;
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
        void setValue(byte index, T value) noexcept {
            get(index). setValue<T>(value);
        }
        template<typename T>
        constexpr T getValue(byte index) const noexcept {
            return get(index).getValue<T>();
        }
        Register& get(byte index) noexcept { return quads_[index & 0b11]; }
        const Register& get(byte index) const noexcept { return quads_[index & 0b11]; }
        Register& operator[](byte index) noexcept { return get(index); }
        const Register& operator[](byte index) const noexcept { return get(index); }
        bool operator==(const QuadRegister& other) const noexcept {
            for (int i = 0; i < 4; ++i) {
                if (quads_[i] != other.quads_[i]) {
                    return false;
                }
            }
            return true;
        }
        bool operator!=(const QuadRegister& other) const noexcept {
            for (int i = 0; i < 4; ++i) {
                if (quads_[i] == other.quads_[i]) {
                    return false;
                }
            }
            return true;
        }
    private:
        Register quads_[4];
};
static_assert(sizeof(QuadRegister) == (2*sizeof(LongOrdinal)));
using TreatAsQuadRegister = TreatAs<QuadRegister>;
class TripleRegister {
    public:
        TripleRegister() = default;
        TripleRegister& operator=(const TripleRegister& other) noexcept {
            setValue<Ordinal>(0, other.getValue<Ordinal>(0));
            setValue<Ordinal>(1, other.getValue<Ordinal>(1));
            setValue<Ordinal>(2, other.getValue<Ordinal>(2));
            return *this;
        }
        TripleRegister& operator=(TripleRegister&& other) noexcept {
            setValue<Ordinal>(0, other.getValue<Ordinal>(0));
            setValue<Ordinal>(1, other.getValue<Ordinal>(1));
            setValue<Ordinal>(2, other.getValue<Ordinal>(2));
            return *this;
        }
        template<typename T>
        void setValue(byte index, T value) noexcept {
            get(index). setValue<T>(value);
        }
        template<typename T>
        constexpr T getValue(byte index) const noexcept {
            return get(index).getValue<T>();
        }
        Register& get(byte index) noexcept { return backingStore_.get(index % 3); }
        const Register& get(byte index) const noexcept { return backingStore_.get(index % 3); }
        Register& operator[](byte index) noexcept { return get(index); }
        const Register& operator[](byte index) const noexcept { return get(index); }
        bool operator==(const TripleRegister& other) const noexcept {
            for (int i = 0; i < 3; ++i) {
                if (backingStore_[i] != other.backingStore_[i]) {
                    return false;
                }
            }
            return true;
        }
        bool operator!=(const TripleRegister& other) const noexcept {
            for (int i = 0; i < 3; ++i) {
                if (backingStore_[i] == other.backingStore_[i]) {
                    return false;
                }
            }
            return true;
        }
    private:
        QuadRegister backingStore_;
};
using TreatAsTripleRegister = TreatAs<TripleRegister>;
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
        TripleRegister& get(byte index, TreatAsTripleRegister) noexcept { return tripleRegisters[index >> 2]; }
        const TripleRegister& get(byte index, TreatAsTripleRegister) const noexcept { return tripleRegisters[index >> 2]; }
    private:
        Register registers[16];
        LongRegister longRegisters[8];
        TripleRegister tripleRegisters[4];
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
 * @brief is the given byte index aligned to quad register boundaries?
 */
constexpr bool aligned(byte index, TreatAsTripleRegister) noexcept { return aligned(index, TreatAsQuadRegister{}); }
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
        TripleRegister& get(byte index, TreatAsTripleRegister) noexcept { 
            if (index < 16) {
                return globals.get(index, TreatAsTripleRegister{});
            } else {
                return locals.get(index, TreatAsTripleRegister{});
            }
        }
        const TripleRegister& get(byte index, TreatAsTripleRegister) const noexcept { 
            if (index < 16) {
                return globals.get(index, TreatAsTripleRegister{});
            } else {
                return locals.get(index, TreatAsTripleRegister{});
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

template<typename T, Ordinal S = DEFAULT_SALIGN>
class Core {
    public:
        static constexpr Ordinal SALIGN = S;
        static constexpr Ordinal C = (SALIGN * 16) - 1;
        static constexpr Ordinal NotC = ~C;

    public:

        void begin() noexcept;
        BootResult start() noexcept;
        void cycle() noexcept;
    private:
        void lockBus();
        void unlockBus();
        void signalBootFailure();
        void setFaultPort(Ordinal value) noexcept;
        /// @todo insert iac dispatch here
        /// @todo insert routines for getting registers and such 
        [[nodiscard]] QuadRegister& getGPR(byte index, TreatAsQuadRegister) noexcept { return gpr_.get(index, TreatAsQuadRegister{}); }
        [[nodiscard]] const QuadRegister& getGPR(byte index, TreatAsQuadRegister) const noexcept { return gpr_.get(index, TreatAsQuadRegister{}); }
        [[nodiscard]] TripleRegister& getGPR(byte index, TreatAsTripleRegister) noexcept { return gpr_.get(index, TreatAsTripleRegister{}); }
        [[nodiscard]] const TripleRegister& getGPR(byte index, TreatAsTripleRegister) const noexcept { return gpr_.get(index, TreatAsTripleRegister{}); }
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
        template<typename Q>
        void moveGPR(byte destIndex, byte srcIndex, TreatAs<Q>) noexcept {
            setGPR(destIndex, getGPRValue(srcIndex, TreatAs<Q>{}), TreatAs<Q>{});
        }
        template<typename Q>
        void moveGPR(byte destIndex, byte destOffset, byte srcIndex, byte srcOffset, TreatAs<Q>) noexcept {
            setGPR(destIndex, destOffset, getGPRValue(srcIndex, srcOffset, TreatAs<Q>{}), TreatAs<Q>{});
        }
        [[nodiscard]] bool getMaskedConditionCode(uint8_t mask) const noexcept;
        [[nodiscard]] bool conditionCodeEqualsMask(uint8_t mask) const noexcept;
        bool fullConditionCodeCheck() noexcept;
        bool fullConditionCodeCheck(uint8_t mask) noexcept;
        Ordinal computeAddress() noexcept;
        void performRegisterTransfer(byte mask, byte count) noexcept;
    private:
        void sendIAC(const iac::Message& msg) noexcept {
            static_cast<T*>(this)->sendIAC_impl(msg);
        }
#if 0
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
#endif
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
        template<typename Q>
        void cmpGeneric(Q src1, Q src2) noexcept {
            if (src1 < src2) {
                ac_.arith.conditionCode = 0b100;
            } else if (src1 == src2) {
                ac_.arith.conditionCode = 0b010;
            } else {
                ac_.arith.conditionCode = 0b001;
            }
        }
        template<typename Q>
        void cmpxbGeneric(uint8_t mask, Q src1, Q src2, int16_t displacement, TreatAs<Q>) noexcept {
            cmpGeneric<Q>(src1, src2);
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
    protected:
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
        type load(Address addr, TreatAs< type > ) const { \
            return static_cast<const T*>(this)->load_impl(addr, TreatAs< type > {}); \
        } \
        void store(Address addr, type value, TreatAs< type > ) { \
            static_cast<T*>(this)->store_impl(addr, value, TreatAs< type > {}); \
        } 
        X(Integer);
        X(Ordinal);
        X(ByteInteger);
        X(ByteOrdinal);
        X(ShortInteger);
        X(ShortOrdinal);
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
        template<typename Q>
        void add(Register& destination, Q src1, Q src2, TreatAs<Q>) noexcept {
            destination.setValue(::addOperation<Q>(src2, src1), TreatAs<Q>{});
        }
        template<typename Q>
        void sub(Register& destination, Q src1, Q src2, TreatAs<Q>) noexcept {
            destination.setValue(::subOperation<Q>(src2, src1), TreatAs<Q>{});
        }
        template<typename Q>
        void mult(Register& destination, Q src1, Q src2, TreatAs<Q>) noexcept {
            destination.setValue(::multiplyOperation<Q>(src2, src1), TreatAs<Q>{});
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
        template<typename Q>
        void remainderOperation(Register& dest, Q src1, Q src2) noexcept {
            if (src1 == 0) {
                /// @todo fix this
                generateFault(ZeroDivideFault);
            } else {
                // taken from the i960Sx manual
                //dest.setValue(src2 - ((src2 / src1) * src1), TreatAs<Q>{});
                dest.setValue(src2 % src1, TreatAs<Q>{});
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
        template<typename Q>
        void divideOperation(Register& dest, Q src1, Q src2) noexcept {
            if (src1 == 0) {
                /// @todo fix this
                generateFault(ZeroDivideFault);
            } else {
                dest.setValue(src2 / src1, TreatAs<Q>{});
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
            Ordinal result = ::modify(src2, dest.getValue<Ordinal>(), temp);
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
        template<typename Q>
        void concmpGeneric(Q src1, Q src2) noexcept {
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
        void mulo(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void muli(Register& dest, Integer src1, Integer src2) noexcept;
        void modify(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void extract(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    protected:
        static decltype(auto) doRandom() {
            return random();
        }
        static decltype(auto) doRandomDisallow0() {
            while (true) {
                auto r = random();
                if (r != 0) {
                    return r;
                }
            }
        }
        template<typename W, typename ... Args>
        static
        bool runTestCases(W wrapper, Args ... args) noexcept {
                return ( ... && wrapper(args)());
        }

        static decltype(auto) makeTestRunner(auto body, auto setup, auto tearDown) noexcept {
            return [setup, body, tearDown]() noexcept -> bool {
                setup();
                auto result = body();
                tearDown();
                return result;
            };
        }

        static decltype(auto) compose(auto f, auto g) noexcept {
            return [f, g](auto... args) {
                return f(g(args...));
            };
        }
        static decltype(auto) compose(auto f, auto g, auto h) noexcept {
            return compose(f, compose(g, h));
        }
        static decltype(auto) compose(auto f, auto g, auto h, auto i) noexcept {
            return compose(f, g, compose(h, i));
        }
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
        byte instructionLength_ = 0;
        bool advanceInstruction_ = false;
};

template<typename T, Ordinal S>
void 
Core<T, S>::mulo(Register& regDest, Ordinal src1o, Ordinal src2o) noexcept {
    mult<Ordinal>(regDest, src1o, src2o, TreatAsOrdinal{});
}

template<typename T, Ordinal S>
void 
Core<T, S>::muli(Register& regDest, Integer src1o, Integer src2o) noexcept {
    mult<Integer>(regDest, src1o, src2o, TreatAsInteger{});
}
template<typename T, Ordinal S>
void 
Core<T, S>::modify(Register& regDest, Ordinal src1o, Ordinal src2o) noexcept {
    regDest.setValue<Ordinal>(::modify(src1o, src2o, regDest.getValue<Ordinal>()));
}
template<typename T, Ordinal S>
void 
Core<T, S>::extract(Register& regDest, Ordinal bitpos, Ordinal len) noexcept {
    // taken from the Hx manual as it isn't insane
    auto actualBitpos = bitpos > 32 ? 32 : bitpos;
    regDest.setValue<Ordinal>((static_cast<Ordinal>(regDest) >> actualBitpos) & ~(0xFFFF'FFFF << len));
}
template<typename T, Ordinal S>
const Register& 
Core<T, S>::getSrc1Register(TreatAsREG) const noexcept {
    if (instruction_.reg.m1) {
        /// @todo what to do if s1 is also set?
        return constants_.get(instruction_.reg.src1);
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1);
    } else {
        return getGPR(instruction_.reg.src1);
    }
}
template<typename T, Ordinal S>
const Register& 
Core<T, S>::getSrc2Register(TreatAsREG) const noexcept {
    if (instruction_.reg.m2) {
        /// @todo what to do if s1 is also set?
        return constants_.get(instruction_.reg.src2);
    } else if (instruction_.reg.s2) {
        return getSFR(instruction_.reg.src2);
    } else {
        return getGPR(instruction_.reg.src2);
    }
}
template<typename T, Ordinal S>
Ordinal 
Core<T, S>::unpackSrc1(TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        /// @todo what to do if s1 is also set?
        return constants_.getValue<Ordinal>(instruction_.reg.src1);
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1).o;
    } else {
        return getGPRValue(instruction_.reg.src1, TreatAsOrdinal{});
    }
}
template<typename T, Ordinal S>
Ordinal 
Core<T, S>::unpackSrc1(byte offset, TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        // literals should always return zero if offset is greater than zero
        return offset == 0 ? constants_.getValue<Ordinal>(instruction_.reg.src1) : 0;
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1, offset).o;
    } else {
        return getGPRValue(instruction_.reg.src1, offset, TreatAsOrdinal{});
    }
}
template<typename T, Ordinal S>
Integer 
Core<T, S>::unpackSrc1(TreatAsInteger, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        return constants_.getValue<Integer>(instruction_.reg.src1);
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1).i;
    } else {
        return getGPRValue(instruction_.reg.src1, TreatAsInteger{});
    }
}
template<typename T, Ordinal S>
Ordinal 
Core<T, S>::unpackSrc2(TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m2) {
        return constants_.getValue<Ordinal>(instruction_.reg.src2);
    } else if (instruction_.reg.s2) {
        return getSFR(instruction_.reg.src2).o;
    } else {
        return getGPRValue(instruction_.reg.src2, TreatAsOrdinal{});
    }
}
template<typename T, Ordinal S>
Integer 
Core<T, S>::unpackSrc2(TreatAsInteger, TreatAsREG) noexcept {
    if (instruction_.reg.m2) {
        return constants_.getValue<Integer>(instruction_.reg.src2);
    } else if (instruction_.reg.s2) {
        return getSFR(instruction_.reg.src2).i;
    } else {
        return getGPRValue(instruction_.reg.src2, TreatAsInteger{});
    }
}

template<typename T, Ordinal S>
void
Core<T, S>::scanbyte(Ordinal src2, Ordinal src1) noexcept {
    if (Register s2(src2), s1(src1); 
            s1.bytes[0] == s2.bytes[0] ||
            s1.bytes[1] == s2.bytes[1] ||
            s1.bytes[2] == s2.bytes[2] ||
            s1.bytes[3] == s2.bytes[3]) {
        ac_.arith.conditionCode = 0b010;
    } else {
        ac_.arith.conditionCode = 0;
    }
}
template<typename T, Ordinal S>
void
Core<T, S>::arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) noexcept {
    // set the carry bit
    ac_.arith.conditionCode = 0;
    // set the overflow bit
    if ((src2MSB == src1MSB) && (src2MSB != destMSB)) {
        ac_.arith.conditionCode |= 0b001;
    } else {
        ac_.arith.conditionCode &= 0b110;
    }
    if (result != 0) {
        ac_.arith.conditionCode |= 0b010;
    } else {
        ac_.arith.conditionCode &= 0b101;
    }
}

template<typename T, Ordinal S>
void
Core<T, S>::checkForPendingInterrupts() noexcept {
    static_cast<T*>(this)->checkForPendingInterrupts_impl();
}


template<typename T, Ordinal S>
void
Core<T, S>::emul(LongRegister& dest, Ordinal src1, Ordinal src2) noexcept {
    if (!aligned(instruction_.reg.srcDest, TreatAsLongRegister{})) {
        /// Since this is unaligned and the destination will always be aligned,
        /// we just do an expensive access of the two unaligned registers
        /// instead. Set the both to 0xFFFF'FFFF
        auto& lower = getGPR(instruction_.reg.srcDest);
        auto& upper = getGPR(instruction_.reg.srcDest + 1);
        lower.template setValue<Ordinal>(0xFFFF'FFFF);
        upper.template setValue<Ordinal>(0xFFFF'FFFF);
        generateFault(InvalidOpcodeFault);
    }  else {
        dest.template setValue<LongOrdinal>(static_cast<LongOrdinal>(src2) * static_cast<LongOrdinal>(src1));
    }
}

template<typename T, Ordinal S>
void
Core<T, S>::ediv(LongRegister& dest, Ordinal src1, const LongRegister& src2) noexcept {
    if (!aligned(instruction_.reg.srcDest, TreatAsLongRegister{}) || !aligned(instruction_.reg.src2, TreatAsLongRegister{})) {
        /// Since this is unaligned and the destination will always be aligned,
        /// we just do an expensive access of the two unaligned registers
        /// instead. Set the both to 0xFFFF'FFFF
        auto& lower = getGPR(instruction_.reg.srcDest);
        auto& upper = getGPR(instruction_.reg.srcDest + 1);
        lower.template setValue<Ordinal>(0xFFFF'FFFF);
        upper.template setValue<Ordinal>(0xFFFF'FFFF);
        generateFault(InvalidOpcodeFault);
    } else if (src1 == 0) {
        // divide by zero
        generateFault(ZeroDivideFault);
    } else {
        //src2.parts[1] = getGPRValue(instruction_.reg.src2, 1, TreatAsOrdinal{});
        auto sl2 = src2.getValue<LongOrdinal>();
        dest.template setValue<Ordinal>(1, sl2 / src1); // quotient
        dest.template setValue<Ordinal>(0, static_cast<Ordinal>(sl2 - (sl2 / src1) * src1)); // remainder
    }
}

template<typename T, Ordinal S>
Ordinal
Core<T, S>::getSystemProcedureTableBase() const noexcept {
    return load(getSystemAddressTableBase() + 120, TreatAsOrdinal{});
}

template<typename T, Ordinal S>
Ordinal
Core<T, S>::getSupervisorStackPointer() const noexcept {
    return load((getSystemProcedureTableBase() + 12), TreatAsOrdinal{});
}

template<typename T, Ordinal S>
Ordinal 
Core<T, S>::unpackSrc1(TreatAsOrdinal, TreatAsCOBR) noexcept {
    if (instruction_.cobr.m1) {
        // treat src1 as a literal
        return instruction_.cobr.src1;
    } else {
        return getGPRValue(instruction_.cobr.src1, TreatAsOrdinal{});
    }
}
template<typename T, Ordinal S>
Ordinal
Core<T, S>::unpackSrc2(TreatAsOrdinal, TreatAsCOBR) noexcept {
    if (instruction_.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction_.cobr.src2).o;
    } else {
        return getGPRValue(instruction_.cobr.src2, TreatAsOrdinal{});
    }
}
template<typename T, Ordinal S>
Integer
Core<T, S>::unpackSrc1(TreatAsInteger, TreatAsCOBR) noexcept {
    if (instruction_.cobr.m1) {
        // treat src1 as a literal
        return instruction_.cobr.src1;
    } else {
        return getGPRValue(instruction_.cobr.src1, TreatAsInteger{});
    }
}
template<typename T, Ordinal S>
Integer
Core<T, S>::unpackSrc2(TreatAsInteger, TreatAsCOBR) noexcept {
    if (instruction_.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction_.cobr.src2).i;
    } else {
        return getGPRValue(instruction_.cobr.src2, TreatAsInteger{});
    }
}
template<typename T, Ordinal S>
Register& 
Core<T, S>::getSFR(byte index) noexcept {
    return sfrs_.get(index);
}
template<typename T, Ordinal S>
Register& 
Core<T, S>::getSFR(byte index, byte offset) noexcept {
    return getSFR((index + offset) & 0b11111);
}
template<typename T, Ordinal S>
const Register& 
Core<T, S>::getSFR(byte index) const noexcept {
    return sfrs_.get(index);
}
template<typename T, Ordinal S>
const Register& 
Core<T, S>::getSFR(byte index, byte offset) const noexcept {
    return getSFR((index + offset) & 0b11111);
}
template<typename T, Ordinal S>
void
Core<T, S>::syncf() noexcept {
    // Wait for all faults to be generated that are associated with any prior
    // uncompleted instructions
    /// @todo implement if it makes sense since we don't have a pipeline
    static_cast<T*>(this)->synchronizeFaults();
}
template<typename T, Ordinal S>
void
Core<T, S>::flushreg() noexcept {
    /// @todo implement if it makes sense since we aren't using register frames
    static_cast<T*>(this)->flushRegisters();
}
template<typename T, Ordinal S>
void
Core<T, S>::mark() noexcept {
    nextInstruction();
    if (pc_.processControls.traceEnable && tc_.trace.breakpointTraceMode) {
        generateFault(MarkTraceFault);
    }
}
template<typename T, Ordinal S>
void
Core<T, S>::fmark() noexcept {
    // advance first so that our return value will always be correct
    nextInstruction();
    if (pc_.processControls.traceEnable) {
        generateFault(MarkTraceFault);
    }
}
template<typename T, Ordinal S>
void
Core<T, S>::restoreStandardFrame() noexcept {
    // need to leave the current call
    moveGPR(FPIndex, PFPIndex, TreatAsOrdinal{});
    // remember that the lowest 6 bits are ignored so it is important to mask
    // them out of the frame pointer address when using the address
    auto realAddress = getGPRValue(FPIndex, TreatAsOrdinal{}) & NotC;
    restoreRegisterSet(realAddress);
    setIP(getGPRValue(RIPIndex, TreatAsOrdinal{}), TreatAsOrdinal{});
    advanceInstruction_ = false;
}
template<typename T, Ordinal S>
void
Core<T, S>::ret() {
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
                ac_.setValue(y, TreatAsOrdinal{});
                if (pc_.inSupervisorMode()) {
                    pc_.setValue(x, TreatAsOrdinal{});
                }
                break;
            }
        case 0b010: 
            if (pc_.inSupervisorMode()) {
                pc_.processControls.traceEnable = 0;
                pc_.processControls.executionMode = 0;
            }
            restoreStandardFrame();
            break;
        case 0b011: 
            if (pc_.inSupervisorMode()) {
                pc_.processControls.traceEnable = 1;
                pc_.processControls.executionMode = 0;
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
                ac_.setValue(y, TreatAsOrdinal{});
                if (pc_.inSupervisorMode()) {
                    pc_.setValue(x, TreatAsOrdinal{});
                    checkForPendingInterrupts();
                }
                break;
            }

            break;
        default: 
            // undefined!
            generateFault(UnimplementedFault);
            break;
    }
}

template<typename T, Ordinal S>
Ordinal
Core<T, S>::computeAddress() noexcept {
    if (instruction_.isMEMA()) {
        Ordinal result = instruction_.mem.offset;
        if (instruction_.mema.action) {
            result += getGPRValue(instruction_.mem.abase, TreatAsOrdinal{});
        }
        return result;
    } else {
        // okay so we need to figure out the minor mode after figuring out if
        // it is a double wide operation or not
        if (instruction_.memb.group) {
            // okay so it is going to be the displacement versions
            // load 32-bits into the optionalDisplacement field
            instructionLength_ = 8;
            Integer result = static_cast<Integer>(load(ip_.a + 4, TreatAsOrdinal{})); // load the optional displacement
            if (instruction_.memb_grp2.useIndex) {
                result += (getGPRValue(instruction_.memb_grp2.index, TreatAsInteger{}) << static_cast<Integer>(instruction_.memb_grp2.scale));
            }
            if (instruction_.memb_grp2.registerIndirect) {
                result += getGPRValue(instruction_.memb_grp2.abase, TreatAsInteger{});
            }
            return static_cast<Ordinal>(result);
        } else {
            // okay so the other group isn't as cleanly designed
            switch (instruction_.memb.modeMinor) {
                case 0b00: // Register Indirect
                    return getGPRValue(instruction_.memb.abase, TreatAsOrdinal{});
                case 0b01: // IP With Displacement 
                    instructionLength_ = 8;
                    return static_cast<Ordinal>(ip_.i + load(ip_.a + 4, TreatAsInteger{}) + 8);
                case 0b11: // Register Indirect With Index
                    return getGPRValue(instruction_.memb.abase, TreatAsOrdinal{}) + (getGPRValue(instruction_.memb.index, TreatAsOrdinal{}) << instruction_.memb.scale);
                default:
                    return -1;
            }
        }
    }
}
template<typename T, Ordinal S>
void
Core<T, S>::storeBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept {
    for (byte i = 0; i < count; ++i, baseAddress += 4) {
        store(baseAddress, getGPRValue(baseRegister, i, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
template<typename T, Ordinal S>
void
Core<T, S>::loadBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept {
    for (byte i = 0; i < count; ++i, baseAddress += 4) {
        setGPR(baseRegister, i, load(baseAddress, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
template<typename T, Ordinal S>
void 
Core<T, S>::ldl(Address effectiveAddress, LongRegister& destination) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsLongRegister{})) {
        generateFault(InvalidOperandFault);
        /// @todo perform an unaligned load into registers
    } else {
        destination[0] = load(effectiveAddress + 0, TreatAsOrdinal{});
        destination[1] = load(effectiveAddress + 4, TreatAsOrdinal{});
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}

template<typename T, Ordinal S>
void
Core<T, S>::stl(Address effectiveAddress, const LongRegister& source) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsLongRegister{})) {
        generateFault(InvalidOperandFault);
        /// @todo perform an unaligned load into registers
    } else {
        // support unaligned accesses
        store(effectiveAddress + 0,  static_cast<Ordinal>(source[0]), TreatAsOrdinal{});
        store(effectiveAddress + 4,  static_cast<Ordinal>(source[1]), TreatAsOrdinal{});
    }
    // the instruction is invalid so we should complete after we are done
}
template<typename T, Ordinal S>
void
Core<T, S>::ldt(Address effectiveAddress, TripleRegister& destination) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsTripleRegister{})) {
        generateFault(InvalidOperandFault);
        /// @todo perform an unaligned load into registers
    } else {
        destination[0] = load(effectiveAddress + 0, TreatAsOrdinal{});
        destination[1] = load(effectiveAddress + 4, TreatAsOrdinal{});
        destination[2] = load(effectiveAddress + 8, TreatAsOrdinal{});
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}

template<typename T, Ordinal S>
void
Core<T, S>::stt(Address effectiveAddress, const TripleRegister& source) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsTripleRegister{})) {
        generateFault(InvalidOperandFault);
    } else {
        // support unaligned accesses
        store(effectiveAddress + 0,  static_cast<Ordinal>(source[0]), TreatAsOrdinal{});
        store(effectiveAddress + 4,  static_cast<Ordinal>(source[1]), TreatAsOrdinal{});
        store(effectiveAddress + 8,  static_cast<Ordinal>(source[2]), TreatAsOrdinal{});
    }
    // the instruction is invalid so we should complete after we are done
}

template<typename T, Ordinal S>
void
Core<T, S>::ldq(Address effectiveAddress, QuadRegister& destination) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsQuadRegister{})) {
        generateFault(InvalidOperandFault);
        /// @todo perform an unaligned load into registers
    } else {
        destination[0] = load(effectiveAddress + 0, TreatAsOrdinal{});
        destination[1] = load(effectiveAddress + 4, TreatAsOrdinal{});
        destination[2] = load(effectiveAddress + 8, TreatAsOrdinal{});
        destination[3] = load(effectiveAddress + 12, TreatAsOrdinal{});
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}

template<typename T, Ordinal S>
void
Core<T, S>::stq(Address effectiveAddress, const QuadRegister& source) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsQuadRegister{})) {
        generateFault(InvalidOperandFault);
    } else {
        store(effectiveAddress + 0,  static_cast<Ordinal>(source[0]), TreatAsOrdinal{});
        store(effectiveAddress + 4,  static_cast<Ordinal>(source[1]), TreatAsOrdinal{});
        store(effectiveAddress + 8,  static_cast<Ordinal>(source[2]), TreatAsOrdinal{});
        store(effectiveAddress + 12, static_cast<Ordinal>(source[3]), TreatAsOrdinal{});
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}
template<typename T, Ordinal S>
void
Core<T, S>::saveReturnAddress(Register& linkRegister) noexcept {
    linkRegister.setValue<Ordinal>(ip_.getValue(TreatAsOrdinal{}) + instructionLength_);
}
template<typename T, Ordinal S>
void
Core<T, S>::saveReturnAddress(byte linkRegister) noexcept {
    setGPR(linkRegister, ip_.getValue(TreatAsOrdinal{}) + instructionLength_, TreatAsOrdinal{});
}
template<typename T, Ordinal S>
void 
Core<T, S>::balx(byte linkRegister, Ordinal branchTo) noexcept {
    saveReturnAddress(linkRegister);
    setIP(branchTo, TreatAsOrdinal{});
}
template<typename T, Ordinal S>
void 
Core<T, S>::balx(Register& linkRegister, Ordinal branchTo) noexcept {
    saveReturnAddress(linkRegister);
    setIP(branchTo, TreatAsOrdinal{});
}
template<typename T, Ordinal S>
bool 
Core<T, S>::registerSetAvailable() noexcept {
    return static_cast<T*>(this)->haveAvailableRegisterSet();
}
template<typename T, Ordinal S>
void
Core<T, S>::allocateNewRegisterFrame() noexcept {
    static_cast<T*>(this)->makeNewRegisterFrame();
}
template<typename T, Ordinal S>
void 
Core<T, S>::saveRegisterSet(Ordinal fp) noexcept {
    // save the "next" register frame to main memory to reclaim it
    //storeBlock(fp, 16, 16);
    static_cast<T*>(this)->saveRegisters(fp);
}
template<typename T, Ordinal S>
void
Core<T, S>::restoreRegisterSet(Ordinal fp) noexcept {
    //loadBlock(fp, 16, 16);
    static_cast<T*>(this)->restoreRegisters(fp);
}
template<typename T, Ordinal S>
void 
Core<T, S>::enterCall(Ordinal fp) noexcept {
    if (!registerSetAvailable()) {
        saveRegisterSet(fp);
    }
    allocateNewRegisterFrame();
}
template<typename T, Ordinal S>
void
Core<T, S>::setupNewFrameInternals(Ordinal fp, Ordinal temp) noexcept {
    setGPR(PFPIndex, fp, TreatAsOrdinal{});
    setGPR(FPIndex, temp, TreatAsOrdinal{});
    setStackPointer(temp + 64, TreatAsOrdinal{});
}
template<typename T, Ordinal S>
void
Core<T, S>::callx(Address effectiveAddress) noexcept {
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    balx(RIPIndex, effectiveAddress);
    enterCall(fp);
    setupNewFrameInternals(fp, temp);
}

template<typename T, Ordinal S>
void 
Core<T, S>::call(Integer displacement) {
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    saveReturnAddress(RIPIndex);
    enterCall(fp);
    branch(displacement);
    setupNewFrameInternals(fp, temp);
}
template<typename T, Ordinal S>
void
Core<T, S>::calls(Ordinal src1) noexcept {
    if (auto targ = src1; targ > 259) {
        generateFault(ProtectionLengthFault);
    } else {
        syncf();
        auto tempPE = load(getSystemProcedureTableBase() + 48 + (4 * targ), TreatAsOrdinal{});
        auto type = static_cast<uint8_t>(tempPE & 0b11);
        auto procedureAddress = tempPE & ~0b11;
        // read entry from system-procedure table, where spbase is address of
        // system-procedure table from Initial Memory Image
        balx(RIPIndex, procedureAddress);
        Ordinal temp = 0;
        byte tempRRR = 0;
        if ((type == 0b00) || pc_.inSupervisorMode()) {
            temp = getNextFrameBase();
            tempRRR = 0;
        } else {
            temp = getSupervisorStackPointer();
            tempRRR = 0b010 | (pc_.processControls.traceEnable ? 0b001 : 0);
            pc_.processControls.executionMode = 1;
            pc_.processControls.traceEnable = temp & 0b1;
        }
        enterCall(temp);
        /// @todo expand pfp and fp to accurately model how this works
        auto& pfp = getGPR(PFPIndex);
        // lowest six bits are ignored
        pfp.setValue(getGPRValue(FPIndex, TreatAsOrdinal{}) & ~0b1'111, TreatAsOrdinal{});
        pfp.pfp.rt = tempRRR;
        setGPR(FPIndex, temp, TreatAsOrdinal{});
        setStackPointer(temp + 64, TreatAsOrdinal{});
    }
}
template<typename T, Ordinal S>
void
Core<T, S>::performRegisterTransfer(byte mask, byte count) noexcept {
    // perform the register transfer first and then check to see if we were
    // offset at all
    for (byte i = 0; i < count; ++i) {
        setGPR(instruction_.reg.srcDest, i, unpackSrc1(i, TreatAsOrdinal{}, TreatAsREG{}), TreatAsOrdinal{});
    }
    if (((instruction_.reg.srcDest & mask) != 0) || ((instruction_.reg.src1 & mask) != 0)) {
        nextInstruction();
        generateFault(InvalidOpcodeFault);
    }
}

template<typename T, Ordinal S>
void
Core<T, S>::lockBus() noexcept {
    static_cast<T*>(this)->busLock();
}
template<typename T, Ordinal S>
void
Core<T, S>::unlockBus() noexcept {
    static_cast<T*>(this)->busUnlock();
}
template<typename T, Ordinal S>
void
Core<T, S>::signalBootFailure() noexcept {
    static_cast<T*>(this)->raiseBootFailure();
}
template<typename T, Ordinal S>
void
Core<T, S>::branch(Integer displacement) noexcept {
    ip_.i += displacement;
    advanceInstruction_ = false;
}
template<typename T, Ordinal S>
void
Core<T, S>::branchConditional(bool condition, Integer displacement) noexcept {
    if (condition) {
        branch(displacement);
    }
}

/**
 * @brief And the condition code with the consistent mask found in the
 * instruction encoding; returns true if the value returned is not zero
 * @param mask The mask to apply to see if we get a value back
 */
template<typename T, Ordinal S>
bool
Core<T, S>::getMaskedConditionCode(uint8_t mask) const noexcept {
    return (ac_.getConditionCode() & mask) != 0;
}

template<typename T, Ordinal S>
bool
Core<T, S>::conditionCodeEqualsMask(uint8_t mask) const noexcept {
    return ac_.getConditionCode() == mask;
}
template<typename T, Ordinal S>
bool
Core<T, S>::fullConditionCodeCheck() noexcept {
    return fullConditionCodeCheck(instruction_.getInstructionMask());
}
template<typename T, Ordinal S>
bool
Core<T, S>::fullConditionCodeCheck(uint8_t mask) noexcept {
    // the second condition handles the case where we are looking at unordered
    // output where it is only true if it is equal to zero. So if it turns out
    // that the condition code is zero and the mask is the unordered kind then
    // return true :). In all other cases, the second check will either fail
    // (because the condition code is zero) or it will never fire because the
    // masked condition code will be non zero.
    return getMaskedConditionCode(mask) || conditionCodeEqualsMask(mask);
}
template<typename T, Ordinal S>
void
Core<T, S>::faultGeneric() noexcept {
    nextInstruction();
    if (fullConditionCodeCheck()) {
        generateFault(ConstraintRangeFault);
    }
}
template<typename T, Ordinal S>
void
Core<T, S>::cycle() noexcept {
    instruction_.setValue(load(ip_.a, TreatAsOrdinal{}), TreatAsOrdinal{});
    instructionLength_ = 4;
    advanceInstruction_ = true;
    if (auto opcode = instruction_.getOpcode(); instruction_.isCTRL()) {
        processInstruction(opcode, instruction_.ctrl.displacement, TreatAsCTRL{});
    } else if (instruction_.isCOBR()) {
        Register& src2 = instruction_.cobr.s2 ? getSFR(instruction_.cobr.src2) : getGPR(instruction_.cobr.src2);
        if (instruction_.cobr.m1) {
            auto src1Value = static_cast<uint8_t>(instruction_.cobr.src1);
            processInstruction(opcode,
                    instruction_.getInstructionMask(),
                    src1Value, 
                    src2,
                    instruction_.cobr.displacement,
                    TreatAsCOBR{});
        } else {
            processInstruction(opcode,
                    instruction_.getInstructionMask(),
                    getGPR(instruction_.cobr.src1),
                    src2,
                    instruction_.cobr.displacement,
                    TreatAsCOBR{});
        }
    } else if (instruction_.isMEMFormat()) {
        auto effectiveAddress = computeAddress();
        Register& destination = getGPR(instruction_.reg.srcDest);
        processInstruction(opcode, destination, effectiveAddress, TreatAsMEM{});
    } else if (instruction_.isREGFormat()) {
        auto& regDest = getGPR(instruction_.reg.srcDest);
        const auto& src1 = getSrc1Register(TreatAsREG{});
        const auto& src2 = getSrc2Register(TreatAsREG{});
        processInstruction(opcode, regDest, src1, src2, TreatAsREG{});
    } else {
        generateFault(UnimplementedFault);
    }
    if (advanceInstruction_) {
        nextInstruction();
    }
}


template<typename T, Ordinal S>
void
Core<T, S>::begin() noexcept {
    // so we need to do any sort of processor setup here
    ip_.clear();
    for (int i = 0; i < 32; ++i) {
        getGPR(i).clear();
        /// @todo setup SFRs
        constants_.setValue<Ordinal>(i, i);
    }
    static_cast<T*>(this)->begin_impl();
}

template<typename T, Ordinal S>
void 
Core<T, S>::synld(Register& dest, Ordinal src) noexcept {
    ac_.arith.conditionCode = 0b000;
    if (auto tempa = maskValue<decltype(src), 0xFFFF'FFFC>(src); tempa == 0xFF00'0004) {
        // interrupt control register needs to be read through this
        ac_.arith.conditionCode = 0b010;
        // copy the contents of the interrupt control register to a target
        // register
        dest.setValue(ictl_.o, TreatAsOrdinal{});
    } else {
        ac_.arith.conditionCode = 0b010;
        dest.setValue(load(tempa, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
template<typename T, Ordinal S>
void 
Core<T, S>::synmov(const Register& dest, Ordinal src) noexcept {
    ac_.arith.conditionCode = 0b000;
    if (auto tempa = maskValue<Ordinal, 0xFFFF'FFFC>(dest.getValue(TreatAsOrdinal{})); tempa == 0xFF00'0004) {
        ictl_ = load(src, TreatAsOrdinal{});
        ac_.arith.conditionCode = 0b010;
    } else {
        auto temp = load(src, TreatAsOrdinal{});
        store(tempa, temp, TreatAsOrdinal{});
        // wait for completion
        ac_.arith.conditionCode = 0b010;
    }
}
template<typename T, Ordinal S>
void 
Core<T, S>::synmovl(const Register& dest, Ordinal src) noexcept {
    ac_.arith.conditionCode = 0b000;
    auto tempa = maskValue<Ordinal, 0xFFFF'FFF8>(dest.getValue(TreatAsOrdinal{}));
    auto tempLower = load(src, TreatAsOrdinal{});
    auto tempUpper = load(src + 4, TreatAsOrdinal{});
    store(tempa, tempLower, TreatAsOrdinal{});
    store(tempa + 4, tempUpper, TreatAsOrdinal{});
    // wait for completion
    ac_.arith.conditionCode = 0b010;
}
template<typename T, Ordinal S>
void 
Core<T, S>::synmovq(const Register& dest, Ordinal src) noexcept {

    ac_.arith.conditionCode = 0b000;
    auto temp0 = load(src, TreatAsOrdinal{});
    auto temp1 = load(src+4, TreatAsOrdinal{});
    auto temp2 = load(src+8, TreatAsOrdinal{});
    auto temp3 = load(src+12, TreatAsOrdinal{});
    if (auto tempa = maskValue<Ordinal, 0xFFFF'FFF0>(dest.getValue(TreatAsOrdinal{})); tempa == 0xFF00'0010) {
        sendIAC(iac::Message{temp0, temp1, temp2, temp3});
        ac_.arith.conditionCode = 0b010;
    } else {
        store(tempa, temp0, TreatAsOrdinal{});
        store(tempa+4, temp1, TreatAsOrdinal{});
        store(tempa+8, temp2, TreatAsOrdinal{});
        store(tempa+12, temp3, TreatAsOrdinal{});
        // wait for completion
        ac_.arith.conditionCode = 0b010;
    }
}

template<typename T, Ordinal S>
void 
Core<T, S>::performSelect(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    dest.setValue(fullConditionCodeCheck() ? src2 : src1, TreatAsOrdinal{});
}
template<typename T, Ordinal S>
void
Core<T, S>::performConditionalSubtract(Register& dest, Integer src1, Integer src2, TreatAsInteger) noexcept {
    if (fullConditionCodeCheck()) {
        subi(dest, src1, src2);
    }
}

template<typename T, Ordinal S>
void
Core<T, S>::performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) noexcept {
    if (fullConditionCodeCheck()) {
        subo(dest, src1, src2);
    }
}

template<typename T, Ordinal S>
void
Core<T, S>::performConditionalAdd(Register& dest, Integer src1, Integer src2, TreatAsInteger) noexcept {
    if (fullConditionCodeCheck()) {
        addi(dest, src1, src2);
    }
}

template<typename T, Ordinal S>
void
Core<T, S>::performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) noexcept {
    if (fullConditionCodeCheck()) {
        addo(dest, src1, src2);
    }
}
template<typename T, Ordinal S>
BootResult
Core<T, S>::start() noexcept {
    assertFailureState();
    if (!performSelfTest()) {
        return BootResult::SelfTestFailure;
    } else {
        // start execution at this point, according to the docs this is what we
        // want to do. 
        deassertFailureState();
        // after this point we now need to jump 
        // Kx has a startup pin to denote if it is the startup processor or not, we
        // are not doing that here
        Ordinal x[8] = { 0 };
        for (int i = 0, j = 0; i < 8; ++i, j+=4) {
            x[i] = load(j, TreatAsOrdinal{});
        }
        

        ac_.arith.conditionCode = 0b000; // clear condition code
        Register temp_{0};
        addc(temp_, 0xFFFF'FFFF, x[0]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[1]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[2]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[3]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[4]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[5]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[6]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[7]);
        if (temp_.getValue(TreatAsOrdinal{}) != 0) {
            assertFailureState();
            return BootResult::ChecksumFail;
        } else {
            systemAddressTableBase_ = x[0];
            prcbAddress_ = x[1];
            ip_.setValue(x[3], TreatAsOrdinal{});
            // fetch IMI
            setGPR(FPIndex, load(prcbAddress_ + 24, TreatAsOrdinal{}), TreatAsOrdinal{});
            pc_.processControls.priority = 31;
            pc_.processControls.state = 1;
            // clear any latched external interrupt/IAC signals
            // begin execution
            return BootResult::Success;
        }
    }
}

template<typename T, Ordinal S>
void
Core<T, S>::assertFailureState() noexcept {
    static_cast<T*>(this)->assertFailureState_impl();
}

template<typename T, Ordinal S>
void
Core<T, S>::deassertFailureState() noexcept {
    static_cast<T*>(this)->deassertFailureState_impl();
}
template<typename T, Ordinal S>
void
Core<T, S>::addi(Register& dest, Integer src1, Integer src2) noexcept {
    add<Integer>(dest, src1, src2, TreatAsInteger{});
    nextInstruction();
    /// @todo implement overflow detection and fault generation
}

template<typename T, Ordinal S>
void
Core<T, S>::addo(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    add<Ordinal>(dest, src1, src2, TreatAsOrdinal{});
}

template<typename T, Ordinal S>
void
Core<T, S>::generateFault(Ordinal faultCode) noexcept {
    static_cast<T*>(this)->generateFault_impl(faultCode);
}

template<typename T, Ordinal S>
Ordinal
Core<T, S>::getStackPointer() const noexcept {
    return getGPRValue(SPIndex, TreatAsOrdinal{});
}
template<typename T, Ordinal S>
void
Core<T, S>::setStackPointer(Ordinal value, TreatAsOrdinal) noexcept {
    setGPR(SPIndex, value, TreatAsOrdinal{});
}
template<typename T, Ordinal S>
Ordinal
Core<T, S>::getNextFrameBase() const noexcept {
    return (getStackPointer() + C) & NotC;
}

template<typename T, Ordinal S>
void
Core<T, S>::nextInstruction() noexcept {
    ip_.o += instructionLength_;
    advanceInstruction_ = false;
}

template<typename T, Ordinal S>
void
Core<T, S>::setIP(Ordinal value, TreatAsOrdinal) noexcept {
    // when we set the ip during this instruction, we need to turn off the
    // automatic advancement!
    ip_.setValue<Ordinal>(value);
    advanceInstruction_ = false;
}
template<typename T, Ordinal S>
void
Core<T, S>::subi(Register& dest, Integer src1, Integer src2) noexcept {
    sub<Integer>(dest, src1, src2, TreatAsInteger{});
    nextInstruction();
    /// @todo implement overflow fault detection
}

template<typename T, Ordinal S>
void
Core<T, S>::subo(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    sub<Ordinal>(dest, src1, src2, TreatAsOrdinal{});
}
template<typename T, Ordinal S>
void 
Core<T, S>::processInstruction(Opcodes opcode, Integer displacement, TreatAsCTRL) noexcept {
    switch (opcode) {
        case Opcodes::bal: // bal
            saveReturnAddress(LRIndex);
            // then fallthrough and take the branch
        case Opcodes::b: // b
            branch(displacement);
            break;
        case Opcodes::call: // call
            call(displacement);
            break;
        case Opcodes::ret: // ret
            ret();
            break;
        case Opcodes::bno:
        case Opcodes::be:
        case Opcodes::bne:
        case Opcodes::bl:
        case Opcodes::ble:
        case Opcodes::bg:
        case Opcodes::bge:
        case Opcodes::bo:
            // the branch instructions have the mask encoded into the opcode
            // itself so we can just use it and save a ton of space overall
            branchConditional(fullConditionCodeCheck(), displacement);
            break;
        case Opcodes::faultno:
        case Opcodes::faulte:
        case Opcodes::faultne:
        case Opcodes::faultl:
        case Opcodes::faultle:
        case Opcodes::faultg:
        case Opcodes::faultge:
        case Opcodes::faulto: 
            faultGeneric();
            break;
        default:
            generateFault(UnimplementedFault);
            break;
    }
}
template<typename T, Ordinal S>
void
Core<T, S>::processInstruction(Opcodes opcode, uint8_t mask, uint8_t src1, const Register& src2, int16_t displacement, TreatAsCOBR) noexcept {
    switch(opcode) {
        case Opcodes::bbc:
            bbc(src1, src2, displacement);
            break;
        case Opcodes::bbs:
            bbs(src1, src2, displacement);
            break;
        case Opcodes::cmpobg:
        case Opcodes::cmpobe:
        case Opcodes::cmpobge:
        case Opcodes::cmpobl:
        case Opcodes::cmpobne:
        case Opcodes::cmpoble:
            cmpobGeneric(mask, src1, src2.getValue<Ordinal>(), displacement);
            break;
        case Opcodes::cmpibno: // never branches
        case Opcodes::cmpibg:
        case Opcodes::cmpibe:
        case Opcodes::cmpibge:
        case Opcodes::cmpibl:
        case Opcodes::cmpibne:
        case Opcodes::cmpible:
        case Opcodes::cmpibo: // always branches
            cmpibGeneric(mask, src1, src2.getValue<Integer>(), displacement);
            break;
        default:
            // test instructions perform modifications to src1 so we must error out
            // in this case!
            generateFault(UnimplementedFault);
            break;
    }
}
template<typename T, Ordinal S>
void 
Core<T, S>::processInstruction(Opcodes opcode, uint8_t mask, Register& src1, const Register& src2, int16_t displacement, TreatAsCOBR) noexcept {
    switch(opcode) {
        case Opcodes::bbc:
            bbc(src1, src2, displacement);
            break;
        case Opcodes::bbs:
            bbs(src1, src2, displacement);
            break;
        case Opcodes::testno:
        case Opcodes::testg:
        case Opcodes::teste:
        case Opcodes::testge:
        case Opcodes::testl:
        case Opcodes::testne:
        case Opcodes::testle:
        case Opcodes::testo:
            src1.setValue<Ordinal>(fullConditionCodeCheck(mask) ? 1 : 0);
            break;
        case Opcodes::cmpobg:
        case Opcodes::cmpobe:
        case Opcodes::cmpobge:
        case Opcodes::cmpobl:
        case Opcodes::cmpobne:
        case Opcodes::cmpoble:
            cmpobGeneric(mask, src1.getValue<Ordinal>(), src2.getValue<Ordinal>(), displacement);
            break;
        case Opcodes::cmpibno: // never branches
        case Opcodes::cmpibg:
        case Opcodes::cmpibe:
        case Opcodes::cmpibge:
        case Opcodes::cmpibl:
        case Opcodes::cmpibne:
        case Opcodes::cmpible:
        case Opcodes::cmpibo: // always branches
            cmpibGeneric(mask, src1.getValue<Integer>(), src2.getValue<Integer>(), displacement);
            break;
        default:
            generateFault(UnimplementedFault);
            break;
    }
}
template<typename T, Ordinal S>
void 
Core<T, S>::processInstruction(Opcodes opcode, Register& srcDest, Address effectiveAddress, TreatAsMEM) noexcept {
    switch (opcode) {
        case Opcodes::balx:
            balx(srcDest, effectiveAddress);
            break;
        case Opcodes::bx:
            setIP(effectiveAddress, TreatAsOrdinal{});
            break;
        case Opcodes::callx:
            callx(effectiveAddress);
            break;
        case Opcodes::st: 
            store(effectiveAddress, srcDest.getValue<Ordinal>(), TreatAs<Ordinal>{});
            break;
        case Opcodes::stob:
            store(effectiveAddress, srcDest.getValue<Ordinal>(), TreatAs<ByteOrdinal>{});
            break;
        case Opcodes::stos:
            store(effectiveAddress, srcDest.getValue<Ordinal>(), TreatAs<ShortOrdinal>{});
            break;
        case Opcodes::stib:
            // If the register data is too large to be stored as a byte or
            // short word, the value is truncated and the integer overflow
            // condition is signalled.
            /// @todo fully implement fault detection
            store(effectiveAddress, srcDest.getValue<Integer>(), TreatAs<ByteInteger>());
            break;
        case Opcodes::stis:
            // If the register data is too large to be stored as a byte or
            // short word, the value is truncated and the integer overflow
            // condition is signalled.
            store(effectiveAddress, srcDest.getValue<Integer>(), TreatAs<ShortInteger>{});
            /// @todo fully implement fault detection
            break;
        case Opcodes::ld: 
            srcDest.setValue<Ordinal>(load(effectiveAddress, TreatAsOrdinal{}));
            break;
        case Opcodes::ldob:
            srcDest.setValue(load(effectiveAddress, TreatAs<ByteOrdinal>{}), TreatAsOrdinal{});
            break;
        case Opcodes::ldos:
            srcDest.setValue(load(effectiveAddress, TreatAs<ShortOrdinal>{}), TreatAsOrdinal{});
            break;
        case Opcodes::ldib:
            srcDest.setValue<Integer>(load(effectiveAddress, TreatAs<ByteInteger>{}));
            break;
        case Opcodes::ldis:
            srcDest.setValue<Integer>(load(effectiveAddress, TreatAs<ShortInteger>{}));
            break;
        case Opcodes::ldl:
            ldl(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsLongRegister{}));
            break;
        case Opcodes::stl:
            stl(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsLongRegister{}));
            break;
        case Opcodes::ldt:
            ldt(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsTripleRegister{}));
            break;
        case Opcodes::stt:
            stt(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsTripleRegister{}));
            break;
        case Opcodes::ldq:
            ldq(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsQuadRegister{}));
            break;
        case Opcodes::stq:
            stq(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsQuadRegister{}));
            break;
        case Opcodes::lda:
            srcDest.setValue<Ordinal>(effectiveAddress);
            break;
        default:
            generateFault(UnimplementedFault);
            break;
    }
}
template<typename T, Ordinal S>
void 
Core<T, S>::processInstruction(Opcodes opcode, Register& regDest, const Register& src1, const Register& src2, TreatAsREG) noexcept {
    auto src2o = static_cast<Ordinal>(src2);
    auto src1o = static_cast<Ordinal>(src1);
    auto src2i = static_cast<Integer>(src2);
    auto src1i = static_cast<Integer>(src1);
    switch (opcode) {
        case Opcodes::nand: // nand
            nand(regDest, src1o, src2o);
            break;
        case Opcodes::andOperation: // and
            andOperation(regDest, src1o, src2o);
            break;
        case Opcodes::clrbit: // clrbit
            clrbit(regDest, src1o, src2o);
            break;
        case Opcodes::andnot: // andnot
            andnot(regDest, src1o, src2o);
            break;
        case Opcodes::notand: // notand
            notand(regDest, src1o, src2o);
            break;
        case Opcodes::notbit: // notbit
            notbit(regDest, src1o, src2o);
            break;
        case Opcodes::xorOperation:
            xorOperation(regDest, src1o, src2o);
            break;
        case Opcodes::setbit:
            setbit(regDest, src1o, src2o);
            break;
        case Opcodes::orOperation: // or
            orOperation(regDest, src1o, src2o);
            break;
        case Opcodes::nor: // nor
            nor(regDest, src1o, src2o);
            break;
        case Opcodes::xnor: 
            xnor(regDest, src1o, src2o);
            break;
        case Opcodes::notOperation: 
            notOperation(regDest, src1o);
            break;
        case Opcodes::ornot: // ornot
            ornot(regDest, src1o, src2o);
            break;
        case Opcodes::notor: // notor
            notor(regDest, src1o, src2o);
            break;
        case Opcodes::alterbit: // alterbit
            alterbit(regDest, src1o, src2o);
            break;
            // in some of the opcodeExt values seem to reflect the resultant truth
            // table for the operation :). That's pretty cool
        case Opcodes::addo:
            addo(regDest, src1o, src2o);
            break;
        case Opcodes::addi: // addi
            addi(regDest, src1i, src2i);
            break;
        case Opcodes::subo: // subo
                            // I remember this trick from college, subtraction is just addition
                            // with a negative second argument :). I never gave it much thought
                            // until now but it seems to be an effective trick to save space.
            subo(regDest, src1o, src2o);
            break;
        case Opcodes::subi: // subi
            subi(regDest, src1i, src2i);
            break;
        case Opcodes::shro: // shro
            regDest.setValue<Ordinal>(src1o < 32 ? src2o >> src1o : 0);
            break;
        case Opcodes::shrdi: // shrdi
                             // according to the manual, equivalent to divi value, 2 so that is what we're going to do for correctness sake
            regDest.setValue<Integer>( src1i < 32 && src1i >= 0 ? src2i / computeBitPosition(src1i) : 0);
            break;
        case Opcodes::shri: 
            shri(regDest, src1i, src2i);
            break;
        case Opcodes::shlo: 
            shlo(regDest, src1o, src2o);
            break;
        case Opcodes::rotate: 
            rotate(regDest, src1o, src2o);
            break;
        case Opcodes::shli: 
            shli(regDest, src1i, src2i);
            break;
        case Opcodes::cmpo: // cmpo
            cmpo(src1o, src2o);
            break;
        case Opcodes::cmpi: // cmpi
            cmpi(src1o, src2o);
            break;
        case Opcodes::concmpo: // concmpo
            concmpo(src1o, src2o);
            break;
        case Opcodes::concmpi: // concmpi
            concmpi(src1i, src2i);
            break;
        case Opcodes::cmpinco: // cmpinco
            cmpinco(regDest, src1o, src2o);
            break;
        case Opcodes::cmpinci: // cmpinci
            cmpinci(regDest, src1i, src2i);
            break;
        case Opcodes::cmpdeco: // cmpdeco
            cmpdeco(regDest, src1i, src2i);
            break;
        case Opcodes::cmpdeci: // cmpdeci
            cmpdeci(regDest, src1i, src2i);
            break;
        case Opcodes::scanbyte: // scanbyte
            scanbyte(src2o, src1o);
            break;
        case Opcodes::chkbit: // chkbit
            ac_.arith.conditionCode = ((src2o & computeBitPosition(src1o)) == 0 ? 0b000 : 0b010);
            break;
        case Opcodes::addc: 
            addc(regDest, src1o, src2o);
            break;
        case Opcodes::subc:
            subc(regDest, src1o, src2o);
            break;
        case Opcodes::mov:
            regDest.setValue<Ordinal>(src1o);
            break;
        case Opcodes::movl:
            performRegisterTransfer(0b1, 2);
            break;
        case Opcodes::movt:
            performRegisterTransfer(0b11, 3);
            break;
        case Opcodes::movq:
            performRegisterTransfer(0b11, 4);
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
            mulo(regDest, src1o, src2o);
            break;
        case Opcodes::muli:
            muli(regDest, src1i, src2i);
            break;
        case Opcodes::divi:
            divi(regDest, src1i, src2i);
            break;
        case Opcodes::divo:
            divo(regDest, src1o, src2o);
            break;
        case Opcodes::remo:
            remo(regDest, src1o, src2o);
            break;
        case Opcodes::remi:
            remi(regDest, src1i, src2i);
            break;
        case Opcodes::modi: 
            modi(regDest, src1i, src2i);
            break;
        case Opcodes::modify:
            modify(regDest, src1o, src2o);
            break;
        case Opcodes::extract:
            extract(regDest, src1o, src2o);
            break;
        case Opcodes::modac: 
            modxc(ac_, regDest, src1o, src2o);
            break;
        case Opcodes::modtc: 
            modxc(tc_, regDest, src1o, src2o);
            break;
        case Opcodes::modpc:
            modpc(regDest, src1o, src2o);
            break;
        case Opcodes::atadd:
            atadd(regDest, src1o, src2o);
            break;
        case Opcodes::atmod:
            atmod(regDest, src1o, src2o);
            break;
        case Opcodes::emul:
            emul(getGPR(instruction_.reg.srcDest, TreatAsLongRegister{}), src1o, src2o);
            break;
        case Opcodes::ediv:
            ediv(getGPR(instruction_.reg.srcDest, TreatAsLongRegister{}),
                 src1o,
                 getGPR(instruction_.reg.src2, TreatAsLongRegister{}));
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
        case Opcodes::synld:
            synld(regDest, src1o);
            break;
        case Opcodes::synmov:
            synmov(src1, src2o);
            break;
        case Opcodes::synmovl:
            synmovl(src1, src2o);
            break;
        case Opcodes::synmovq:
            synmovq(src1, src2o);
            break;
        case Opcodes::selno:
        case Opcodes::sele:
        case Opcodes::selg:
        case Opcodes::selge:
        case Opcodes::sell:
        case Opcodes::selne:
        case Opcodes::selle:
        case Opcodes::selo:
            performSelect(regDest, src1o, src2o);
            break;
        case Opcodes::addono:
        case Opcodes::addoe:
        case Opcodes::addog:
        case Opcodes::addoge:
        case Opcodes::addol:
        case Opcodes::addone:
        case Opcodes::addole:
        case Opcodes::addoo:
            performConditionalAdd(regDest, src1o, src2o, TreatAsOrdinal{});
            break;

        case Opcodes::addino:
        case Opcodes::addie:
        case Opcodes::addig:
        case Opcodes::addige:
        case Opcodes::addil:
        case Opcodes::addine:
        case Opcodes::addile:
        case Opcodes::addio:
            performConditionalAdd(regDest, src1i, src2i, TreatAsInteger{});
            break;
        case Opcodes::subono:
        case Opcodes::suboe:
        case Opcodes::subog:
        case Opcodes::suboge:
        case Opcodes::subol:
        case Opcodes::subone:
        case Opcodes::subole:
        case Opcodes::suboo:
            performConditionalSubtract(regDest, src1o, src2o, TreatAsOrdinal{});
            break;

        case Opcodes::subino:
        case Opcodes::subie:
        case Opcodes::subig:
        case Opcodes::subige:
        case Opcodes::subil:
        case Opcodes::subine:
        case Opcodes::subile:
        case Opcodes::subio:
            performConditionalSubtract(regDest, src1i, src2i, TreatAsInteger{});
            break;
        default:
            generateFault(UnimplementedFault);
            break;
    }
}


template<typename T, Ordinal S>
void
Core<T, S>::modpc(Register& regDest, Ordinal src1o, Ordinal src2o) noexcept {
    if (auto mask = src1o; mask != 0) {
        if (!pc_.inSupervisorMode()) {
            generateFault(TypeMismatchFault);
        } else {
            regDest.setValue<Ordinal>(pc_.modify(mask, src2o));
            if (regDest.getPriority() > pc_.getPriority()) {
                checkForPendingInterrupts();
            }
        }
    } else {
        regDest.setValue<Ordinal>(pc_.getValue<Ordinal>());
    }
}
template<typename T, Ordinal S>
void 
Core<T, S>::modxc(Register& control, Register& dest, Ordinal src1, Ordinal src2) noexcept {
    dest.setValue<Ordinal>(control.modify(src1, src2));
}
template<typename T, Ordinal S>
void 
Core<T, S>::shlo(Register& srcDest, Ordinal src1, Ordinal src2) noexcept {
    srcDest.setValue<Ordinal>(src1 < 32 ? src2 << src1 : 0);
}
template<typename T, Ordinal S>
void 
Core<T, S>::shli(Register& srcDest, Integer src1, Integer src2) noexcept {
    srcDest.setValue<Integer>(src2 << src1);
}
template<typename T, Ordinal S>
void 
Core<T, S>::rotate(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    dest.setValue<Ordinal>(rotateOperation(src2, src1));
}
template<typename T, Ordinal S>
void 
Core<T, S>::shri(Register& dest, Integer src1, Integer src2) noexcept {
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
    dest.setValue<Integer>(src2 >> src1);
}


template<typename T, Ordinal S>
bool
Core<T, S>::performSelfTest() noexcept {
    auto clearRegisters = [this]() {
        for (int i = 0; i < 32; ++i) {
            getGPR(i).clear();
        }
    };
    // test different instructions to see if they are working correctly
    auto testRegisters = [this]() {
        for (int i = 0; i < 32; ++i) {
            auto& temporary = getGPR(i);
            auto randomValue = static_cast<Ordinal>(random());
            auto randomInteger = static_cast<Integer>(random());
            temporary.template setValue<Ordinal>(randomValue);
            if (temporary.template getValue<Ordinal>() != randomValue) {
                return false;
            }
            temporary.template setValue<Integer>(randomInteger);
            if (temporary.template getValue<Integer>() != randomInteger) {
                return false;
            }
        }
        return true;
    };
    // test move operations
    // first mov
    auto testMoveOperations = [this]() {
        auto& g0 = getGPR(0);
        auto& g1 = getGPR(1);
        auto randomSourceValue = static_cast<Ordinal>(random());
        g0.template setValue<Ordinal>(randomSourceValue);
        g1.template setValue<Ordinal>(0xFFFF'FFFF);
        g1.template setValue<Ordinal>(g0.template getValue<Ordinal>()); 
        if (g1.template getValue<Ordinal>() != g0.template getValue<Ordinal>()) {
            return false;
        }
        auto randomSourceValue2 = static_cast<Ordinal>(random());
        auto& gl0 = getGPR(0, TreatAsLongRegister{});
        auto& gl1 = getGPR(2, TreatAsLongRegister{});
        gl0[0] = randomSourceValue;
        gl0[1] = randomSourceValue2;
        if (static_cast<Ordinal>(gl0[0]) != randomSourceValue) {
            return false;
        }
        if (static_cast<Ordinal>(gl0[1]) != randomSourceValue2) {
            return false;
        }
        gl1 = gl0;
        if (static_cast<Ordinal>(gl1[0]) != randomSourceValue) {
            return false;
        }
        if (static_cast<Ordinal>(gl1[1]) != randomSourceValue2) {
            return false;
        }
        return true;
    };
    auto makeGenericOperation = [this](auto maker, auto doIt, auto converter, auto name, auto genSrc1, auto genSrc2) {
        return [this, maker, doIt, converter, name, genSrc1, genSrc2](byte gpr0 = random() & 0b11111, 
                byte gpr1 = random() & 0b11111, 
                byte gpr2 = random() & 0b11111) -> bool {
            auto rs0 = converter(genSrc1());
            auto rs1 = converter(genSrc2());
            auto& src1 = getGPR(gpr0);
            src1 = rs0;
            auto& src2 = getGPR(gpr1);
            src2 = rs1;
            auto& dest = getGPR(gpr2);
            auto rs2 = maker(converter(src1), converter(src2), converter(dest));
            doIt(dest, converter(src1), converter(src2));
            if (converter(dest) != rs2) {
                return false;
            }
            return true;
        };
    };
    auto genericIntegerOperation = [this, makeGenericOperation](auto maker, auto doIt, auto name, auto genSrc1, auto genSrc2) {
        return makeGenericOperation(maker, doIt, [](auto value) { return static_cast<Integer>(value); }, name, genSrc1, genSrc2);
    };
    auto genericOrdinalOperation = [this, makeGenericOperation](auto maker, auto doIt, auto name, auto genSrc1, auto genSrc2) {
        return makeGenericOperation(maker, doIt, [](auto value) { return static_cast<Ordinal>(value); }, name, genSrc1, genSrc2);
    };
    auto makeIntegerOperation = [this, genericIntegerOperation](auto maker, auto doIt, auto name) {
        return genericIntegerOperation(maker, doIt, name, doRandom, doRandom);
    };
    auto makeOrdinalOperation = [this, genericOrdinalOperation](auto maker, auto doIt, auto name) {
        return genericOrdinalOperation(maker, doIt, name, doRandom, doRandom);
    };
    auto runTest = [this, clearRegisters](auto fn) {
        return makeTestRunner(fn, clearRegisters, clearRegisters);
    };
    return runTestCases(runTest,
            testMoveOperations,
            testRegisters,
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 + src1; }, 
                                  [this](auto& dest, auto src1, auto src2) { return addi(dest, src1, src2); },
                                  F("addi"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 + src1; },
                                  [this](auto& dest, auto src1, auto src2) { return addo(dest, src1, src2); },
                                  F("addo"))),
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 - src1; }, 
                                  [this](auto& dest, auto src1, auto src2) { return subi(dest, src1, src2); },
                                  F("subi"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 - src1; },
                                  [this](auto& dest, auto src1, auto src2) { return subo(dest, src1, src2); },
                                  F("subo"))),
            (genericIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 / src1; }, 
                                     [this](auto& dest, auto src1, auto src2) { return divi(dest, src1, src2); },
                                     F("divi"),
                                     doRandomDisallow0, doRandom)),
            (genericOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 / src1; },
                                     [this](auto& dest, auto src1, auto src2) { return divo(dest, src1, src2); },
                                     F("divo"),
                                     doRandomDisallow0, doRandom)),
            (genericIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 % src1; }, 
                                     [this](auto& dest, auto src1, auto src2) { return remi(dest, src1, src2); },
                                     F("remi"),
                                     doRandomDisallow0, doRandom)),
            (genericOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 % src1; },
                                     [this](auto& dest, auto src1, auto src2) { return remo(dest, src1, src2); },
                                     F("remo"),
                                     doRandomDisallow0, doRandom)),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 & src1); },
                                  [this](auto& dest, auto src1, auto src2) { return nand(dest, src1, src2); },
                                  F("nand"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 | src1); },
                                  [this](auto& dest, auto src1, auto src2) { return nor(dest, src1, src2); },
                                  F("nor"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 | src1); },
                                  [this](auto& dest, auto src1, auto src2) { return orOperation(dest, src1, src2); },
                                  F("or"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 & src1); },
                                  [this](auto& dest, auto src1, auto src2) { return andOperation(dest, src1, src2); },
                                  F("and"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 ^ src1); },
                                  [this](auto& dest, auto src1, auto src2) { return xorOperation(dest, src1, src2); },
                                  F("xor"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 ^ src1); },
                                  [this](auto& dest, auto src1, auto src2) { return xnor(dest, src1, src2); },
                                  F("xnor"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return rotateOperation(src2, src1); },
                                  [this](auto& dest, auto src1, auto src2) { return rotate(dest, src1, src2); },
                                  F("rotate"))),
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return (src2 >> src1); },
                                  [this](auto& dest, auto src1, auto src2) { return shri(dest, src1, src2); },
                                  F("shri"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src1 < 32 ? (src2 << src1) : 0; },
                                  [this](auto& dest, auto src1, auto src2) { return shlo(dest, src1, src2); },
                                  F("shlo"))),
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return (src2 << src1); },
                                  [this](auto& dest, auto src1, auto src2) { return shli(dest, src1, src2); },
                                  F("shli"))),
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 * src1; }, 
                                  [this](auto& dest, auto src1, auto src2) { return muli(dest, src1, src2); },
                                  F("muli"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 * src1; },
                                  [this](auto& dest, auto src1, auto src2) { return mulo(dest, src1, src2); },
                                  F("mulo"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal dest) { return ::modify(src1, src2, dest); },
                                  [this](auto& dest, auto src1, auto src2) { return modify(dest, src1, src2); },
                                  F("modify")))
                ) && static_cast<T*>(this)->runExtendedSelfTests();
}

#endif // end SIM5_CORE_H__
