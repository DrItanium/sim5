// sim5
// Copyright (c) 2022-2023, Joshua Scoggins
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

#include <cstdlib>
#include <functional>
#include <array>
#include "Types.h"
#include "IAC.h"
#include "BinaryOperations.h"
#include <istream>
#include <iostream>
constexpr uint8_t getDebugLoggingLevel() noexcept {
    return 0;
}
#define DEBUG_LOG_LEVEL(lvl) if constexpr (getDebugLoggingLevel() >= lvl)
#define DEBUG_ENTER_FUNCTION DEBUG_LOG_LEVEL(6) std::cout << "Entering Function: " << __PRETTY_FUNCTION__ << std::endl
#define DEBUG_LEAVE_FUNCTION DEBUG_LOG_LEVEL(6) std::cout << "Leaving Function: " << __PRETTY_FUNCTION__ << std::endl

constexpr Ordinal DEFAULT_SALIGN = 4;
/// faults
constexpr Ordinal NoFault = 0xFFFF'FFFF;
constexpr Ordinal ParallelFault = 0;
constexpr Ordinal TraceFaultBase = 0x0001'0000;
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
constexpr Ordinal InvalidOperandFault = 0x0002'0004;

constexpr Ordinal IntegerOverflowFault = 0x0003'0001;
constexpr Ordinal ZeroDivideFault = 0x0003'0002;
/// @todo support floating point faults
constexpr Ordinal ConstraintRangeFault = 0x0005'0001;
constexpr Ordinal InvalidSSFault = 0x0005'0002;
constexpr Ordinal InvalidSegmentTableEntryFault = 0x0006'0001;
constexpr Ordinal InvalidPageTableDirectoryEntryFault = 0x0006'0002;
constexpr Ordinal InvalidPageTableEntryFault = 0x0006'0003;
/// @todo support protection faults
constexpr Ordinal SegmentLengthFault = 0x0007'0002;
constexpr Ordinal PageRightsFault = 0x0007'0004;
constexpr Ordinal BadAccessFault = 0x0007'0020;
constexpr Ordinal Machine_BadAccessFault = 0x0008'0001;
constexpr Ordinal Machine_ParityErrorFault = 0x0008'0002;

constexpr Ordinal ControlFault = 0x0009'0001;
constexpr Ordinal DispatchFault = 0x0009'0002;
constexpr Ordinal IACFault = 0x0009'0003;

constexpr Ordinal TypeMismatchFault = 0x000a'0001;
constexpr Ordinal ContentsFault = 0x000a'0002;
constexpr Ordinal TimeSliceFault = 0x000c'0001;
constexpr Ordinal InvalidDescriptorFault = 0x000d'0001;
constexpr Ordinal EventNoticeFault = 0x000e'0001;
constexpr Ordinal OverrideFault = 0x0010'0000;

enum class Opcodes : uint16_t {
#define X(name, opcode, str, level, privileged) name = opcode ,
#include "Opcodes.def"
#undef X
};
template<Opcodes code>
constexpr bool IsPrivileged = false;
#define X(name, opcode, str, level, privileged) template<> constexpr bool IsPrivileged< Opcodes :: name > = privileged;
#include "Opcodes.def"
#undef X

template<Opcodes code>
constexpr auto ArchitectureLevel_v = ArchitectureLevel::Unknown;
#define X(name, opcode, str, level, privileged) template<> constexpr auto ArchitectureLevel_v< Opcodes :: name > = ArchitectureLevel:: level ;
#include "Opcodes.def"
#undef X

enum class BootResult : uint8_t {
    Success,
    SelfTestFailure,
    ChecksumFail,
};
struct TreatAsREG { };
struct TreatAsCOBR { };
struct TreatAsCTRL { };
struct TreatAsMEM { };
using BackingUnionType = Ordinal;
union Register {
    constexpr explicit Register(Ordinal value) : o(value) { }
    Register() = default;
    Ordinal o;
    Integer i;
    Address a;
    Real r;
    ByteOrdinal bytes[sizeof(Ordinal)];
    ShortOrdinal shorts[sizeof(Ordinal)/sizeof(ShortOrdinal)];
    [[nodiscard]] constexpr uint8_t getInstructionMask() const noexcept {
        return bytes[3] & 0b111;
    }
    [[nodiscard]] constexpr uint8_t getMajorOpcode() const noexcept {
        return bytes[3];
    }
    [[nodiscard]] constexpr bool isCTRL() const noexcept {
        return bytes[3] < 0x20;
    }
    [[nodiscard]] constexpr bool isCOBR() const noexcept {
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
        Integer displacement : 13;
        BackingUnionType m1: 1;
        BackingUnionType src2: 5;
        BackingUnionType src1: 5;
        BackingUnionType opcode : 8;
    } cobrDisplacement;
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
        /// @todo check the bp and sfr bits eventually
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
        BackingUnionType index : 5;
        BackingUnionType unused : 2;
        BackingUnionType scale : 3;
        BackingUnionType group: 4;
        BackingUnionType abase : 5;
        BackingUnionType srcDest : 5;
        BackingUnionType opcode : 8;
    } memb_grp3;
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
    [[nodiscard]] constexpr auto getConditionCode() const noexcept { return arith.conditionCode; }
    void setPriority(Ordinal value) noexcept { processControls.priority = value; }
    [[nodiscard]] constexpr ByteOrdinal getPriority() const noexcept { return processControls.priority; }
    [[nodiscard]] constexpr bool inSupervisorMode() const noexcept { return processControls.executionMode; }
    [[nodiscard]] constexpr bool inUserMode() const noexcept { return !inSupervisorMode(); }
    [[nodiscard]] constexpr bool inInterruptedState() const noexcept { return processControls.state != 0; }
    [[nodiscard]] constexpr bool inExecutingState() const noexcept { return processControls.state == 0; }
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
    [[nodiscard]] constexpr ByteOrdinal getReturnType() const noexcept { return pfp.rt; }
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
    bool getCarryBit() const noexcept { return arith.conditionCode & 0b010; }
    [[nodiscard]] Ordinal modify(Ordinal mask, Ordinal src) noexcept;
    void setValue(Real value, TreatAsReal) noexcept { r = value; }
    void setValue(Ordinal value, TreatAsOrdinal) noexcept {
        DEBUG_LOG_LEVEL(4)  {
            std::cout << __PRETTY_FUNCTION__ << ": 0x" << std::hex << o << " -> 0x" << std::hex << value << std::endl;
        }
        o = value;
    }
    void setValue(Integer value, TreatAsInteger) noexcept {
        DEBUG_LOG_LEVEL(4)  {
            std::cout << __PRETTY_FUNCTION__ << ": 0x" << std::hex << o << " -> 0x" << std::hex << value << std::endl;
        }
        i = value;
    }
    [[nodiscard]] Integer getValue(TreatAsInteger) const noexcept {
        DEBUG_LOG_LEVEL(4)  {
            std::cout << __PRETTY_FUNCTION__ << ": 0x" << std::hex << i << std::endl;
        }
        return i;
    }
    [[nodiscard]] Ordinal getValue(TreatAsOrdinal) const noexcept {
        DEBUG_LOG_LEVEL(4)  {
            std::cout << __PRETTY_FUNCTION__ << ": 0x" << std::hex << o << std::endl;
        }
        return o;
    }
    [[nodiscard]] Real getValue(TreatAsReal) const noexcept {
        DEBUG_LOG_LEVEL(4)  {
            std::cout << __PRETTY_FUNCTION__ << ": 0x" << std::hex << reinterpret_cast<Ordinal>(o) << std::endl;
        }
        return r;
    }
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
    ByteOrdinal& operator[](ByteOrdinal index) noexcept {
        return bytes[index & 0b11];
    }
    constexpr const ByteOrdinal& operator[](ByteOrdinal index) const noexcept {
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
    constexpr auto getDisplacement(TreatAsCTRL) const noexcept {
        // clear the lowest two bits since those can be reserved for other things
        return alignTo4ByteBoundaries(ctrl.displacement, TreatAs<Integer>{});
    }
    constexpr auto getDisplacement(TreatAsCOBR) const noexcept {
        // clear the lowest two bits
        return alignTo4ByteBoundaries(cobrDisplacement.displacement, TreatAs<Integer>{});
    }
};
static_assert(sizeof(Register) == sizeof(Ordinal));
union LongRegister {
    public:
        LongRegister() = default;
        [[nodiscard]] constexpr LongOrdinal getValue(TreatAsLongOrdinal) const noexcept { return lo; }
        [[nodiscard]] constexpr LongInteger getValue(TreatAsLongInteger) const noexcept { return li; }
        [[nodiscard]] constexpr LongReal getValue(TreatAsLongReal) const noexcept { return lr; }
        void setValue(LongOrdinal value, TreatAsLongOrdinal) noexcept { lo = value; }
        void setValue(LongInteger value, TreatAsLongInteger) noexcept { li = value; }
        void setValue(LongReal value, TreatAsLongReal) noexcept { lr = value; }
        Register& get(ByteOrdinal index) noexcept { return pair_[index & 0b1]; }
        const Register& get(ByteOrdinal index) const noexcept { return pair_[index & 0b1]; }

        template<typename T>
        void setValue(ByteOrdinal index, T value) noexcept {
            get(index).setValue<T>(value);
        }
        template<typename T>
        constexpr T getValue(ByteOrdinal index) const noexcept {
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
        Register& operator[](ByteOrdinal index) noexcept {
            return get(index);
        }
        const Register& operator[](ByteOrdinal index) const noexcept {
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
        LongReal lr;
};
static_assert(sizeof(LongRegister) == sizeof(LongOrdinal));

using TreatAsLongRegister = TreatAs<LongRegister>;
using TreatAsRegister = TreatAs<Register>;

union QuadRegister {
    public:
        QuadRegister() = default;
        template<typename T>
        void setValue(ByteOrdinal index, T value) noexcept {
            get(index). setValue<T>(value);
        }
        template<typename T>
        constexpr T getValue(ByteOrdinal index) const noexcept {
            return get(index).getValue<T>();
        }
        Register& get(ByteOrdinal index) noexcept { return quads_[index & 0b11]; }
        const Register& get(ByteOrdinal index) const noexcept { return quads_[index & 0b11]; }
        Register& operator[](ByteOrdinal index) noexcept { return get(index); }
        const Register& operator[](ByteOrdinal index) const noexcept { return get(index); }
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
        void setValue(ByteOrdinal index, T value) noexcept {
            get(index). setValue<T>(value);
        }
        template<typename T>
        constexpr T getValue(ByteOrdinal index) const noexcept {
            return get(index).getValue<T>();
        }
        Register& get(ByteOrdinal index) noexcept { return backingStore_.get(index % 3); }
        const Register& get(ByteOrdinal index) const noexcept { return backingStore_.get(index % 3); }
        Register& operator[](ByteOrdinal index) noexcept { return get(index); }
        const Register& operator[](ByteOrdinal index) const noexcept { return get(index); }
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
        [[nodiscard]] constexpr ExtendedReal getValue(TreatAsExtendedReal) const noexcept { return extendedReal_; }
        void setValue(ExtendedReal value, TreatAsExtendedReal) noexcept { extendedReal_ = value; }
    private:
        QuadRegister backingStore_;
        ExtendedReal extendedReal_;
};
using TreatAsTripleRegister = TreatAs<TripleRegister>;
// On the i960 this is separated out into two parts, locals and globals
// The idea is that globals are always available and locals are per function.
// You are supposed to have multiple local frames on chip to accelerate
// operations. However, to simplify things I am not keeping any extra sets on
// chip for the time being so there is just a single register block to work
// with
constexpr auto LocalRegisterBase = 0;
constexpr auto GlobalRegisterBase = 16;
constexpr auto LRIndex = GlobalRegisterBase + 14;
constexpr auto FPIndex = GlobalRegisterBase + 15;
constexpr auto PFPIndex = LocalRegisterBase + 0;
constexpr auto SPIndex = LocalRegisterBase + 1;
constexpr auto RIPIndex = LocalRegisterBase + 2;
union RegisterFrame {
private:
    constexpr static ByteOrdinal maskIndex(ByteOrdinal input) noexcept { return input & 0b1111; }
    constexpr static ByteOrdinal generateIndex(ByteOrdinal input, TreatAsRegister) noexcept { return maskIndex(input); }
    constexpr static ByteOrdinal generateIndex(ByteOrdinal input, TreatAsLongRegister) noexcept { return maskIndex(input) >> 1; }
    constexpr static ByteOrdinal generateIndex(ByteOrdinal input, TreatAsQuadRegister) noexcept { return maskIndex(input) >> 2; }
    constexpr static ByteOrdinal generateIndex(ByteOrdinal input, TreatAsTripleRegister) noexcept { return maskIndex(input) >> 2; }
    public:
        RegisterFrame() = default;
        [[nodiscard]] Register& get(ByteOrdinal index, TreatAsRegister) noexcept { return registers[generateIndex(index, TreatAsRegister{})]; }
        [[nodiscard]] const Register& get(ByteOrdinal index, TreatAsRegister) const noexcept { return registers[generateIndex(index, TreatAsRegister{})]; }
        [[nodiscard]] LongRegister& get(ByteOrdinal index, TreatAsLongRegister) noexcept { return longRegisters[generateIndex(index, TreatAsLongRegister{})]; }
        [[nodiscard]] const LongRegister& get(ByteOrdinal index, TreatAsLongRegister) const noexcept { return longRegisters[generateIndex(index, TreatAsLongRegister{})]; }
        [[nodiscard]] TripleRegister& get(ByteOrdinal index, TreatAsTripleRegister) noexcept { return tripleRegisters[generateIndex(index, TreatAsTripleRegister{})]; }
        [[nodiscard]] const TripleRegister& get(ByteOrdinal index, TreatAsTripleRegister) const noexcept { return tripleRegisters[generateIndex(index, TreatAsTripleRegister{})]; }
        [[nodiscard]] QuadRegister& get(ByteOrdinal index, TreatAsQuadRegister) noexcept { return quadRegisters[generateIndex(index, TreatAsQuadRegister{})]; }
        [[nodiscard]] const QuadRegister& get(ByteOrdinal index, TreatAsQuadRegister) const noexcept { return quadRegisters[generateIndex(index, TreatAsQuadRegister{})]; }
        void clear() noexcept {
            for (auto& a : registers) {
                a.clear();
            }
        }
    private:
        Register registers[16];
        LongRegister longRegisters[8];
        TripleRegister tripleRegisters[4];
        QuadRegister quadRegisters[4];
};
/**
 * @brief is the given byte index aligned to long register boundaries?
 */
constexpr bool aligned(ByteOrdinal index, TreatAsLongRegister) noexcept { return (index & 0b1) == 0; }
/**
 * @brief is the given byte index aligned to quad register boundaries?
 */
constexpr bool aligned(ByteOrdinal index, TreatAsQuadRegister) noexcept { return (index & 0b11) == 0; }
/**
 * @brief is the given byte index aligned to quad register boundaries?
 */
constexpr bool aligned(ByteOrdinal index, TreatAsTripleRegister) noexcept { return aligned(index, TreatAsQuadRegister{}); }

class RegisterBlock32 {
    public:
        RegisterBlock32() = default;
        Register& get(ByteOrdinal index) noexcept { return registers_[index & 0b11111]; }
        const Register& get(ByteOrdinal index) const noexcept { return registers_[index & 0b11111]; }
        template<typename T>
        void setValue(ByteOrdinal index, T value) noexcept {
            get(index).setValue(value, TreatAs<T>{});
        }
        template<typename T>
        T getValue(ByteOrdinal index) const noexcept {
            return get(index).getValue(TreatAs<T>{});
        }
    private:
        Register registers_[32];
};
constexpr Ordinal getSALIGNParameter() noexcept {
#ifdef SALIGN960
    return SALIGN960;
#else
    return DEFAULT_SALIGN;
#endif
}


class Core {
    public:
        static constexpr Ordinal SALIGN = getSALIGNParameter();
        static constexpr Ordinal C = (SALIGN * 16) - 1;
        static constexpr Ordinal NotC = ~C;
        static constexpr uint8_t NumberOfLocalRegisterFrames = 4;

    struct LocalRegisterSet {
    public:
        constexpr bool valid() const noexcept { return _valid; }
        constexpr auto getAddress() const noexcept { return _targetFramePointer; }
        RegisterFrame& getUnderlyingFrame() noexcept { return _theFrame; }
        const RegisterFrame& getUnderlyingFrame() const noexcept { return _theFrame; }
        void relinquishOwnership() noexcept {
            DEBUG_LOG_LEVEL(2) {
                std::cout << __PRETTY_FUNCTION__ << ": {" << std::endl;
                std::cout << __PRETTY_FUNCTION__ << ": &this: 0x" << std::hex << reinterpret_cast<uintptr_t>(this) << std::endl;
            }
            _valid = false;
            synchronizeOwnership(0);
            DEBUG_LOG_LEVEL(2) {
                std::cout << __PRETTY_FUNCTION__ << ": }" << std::endl;
            }
            /// @todo zero out the frame?
        }
        using SaveRegistersFunction = std::function<void(const RegisterFrame&, Address)>;
        using RestoreRegistersFunction = std::function<void(RegisterFrame&, Address)>;
        void
        relinquishOwnership(SaveRegistersFunction saveRegisters) {
            if (_valid) {
                DEBUG_LOG_LEVEL(2) {
                    std::cout << __PRETTY_FUNCTION__ << ": saving to 0x" << std::hex << _targetFramePointer << std::endl;
                }
                saveRegisters(_theFrame, _targetFramePointer);
            }
            relinquishOwnership();
        }
        void
        takeOwnership(Address newFP, SaveRegistersFunction saveRegisters) {
            DEBUG_ENTER_FUNCTION;
            if (valid()) {
                DEBUG_LOG_LEVEL(2) {
                    std::cout << __PRETTY_FUNCTION__ << ": saving to 0x" << std::hex << _targetFramePointer << std::endl;
                }
                saveRegisters(_theFrame, _targetFramePointer);
            }
            _valid = true;
            synchronizeOwnership(newFP);
            // don't clear out the registers
            DEBUG_LEAVE_FUNCTION;
        }
        void
        restoreOwnership(Address newFP,
                         SaveRegistersFunction saveRegisters,
                         RestoreRegistersFunction restoreRegisters) {
            DEBUG_ENTER_FUNCTION;
            DEBUG_LOG_LEVEL(2) {
                std::cout << __PRETTY_FUNCTION__ << ": &this: 0x" << std::hex << reinterpret_cast<uintptr_t>(this) << std::endl;
                std::cout << __PRETTY_FUNCTION__ << ": newFramePointer is 0x" << std::hex << newFP << std::endl;
            }
            if (valid()) {
                DEBUG_LOG_LEVEL(2) {
                    std::cout << __PRETTY_FUNCTION__ << ": targetFramePointer currently 0x" << std::hex << _targetFramePointer << std::endl;
                    std::cout << __PRETTY_FUNCTION__ << ": Current frame is valid" << std::endl;
                }
                // okay we have something valid in there right now, so we need to determine if it is valid or not
                if (newFP == _targetFramePointer) {
                    DEBUG_LOG_LEVEL(2) {
                        std::cout << __PRETTY_FUNCTION__ << ": match found!" << std::endl;
                    }
                    // okay so we got a match, great!
                    // just leave early
                    DEBUG_LEAVE_FUNCTION;
                    return;
                }
                DEBUG_LOG_LEVEL(2) {
                    std::cout << __PRETTY_FUNCTION__ << ": no match found!" << std::endl;
                    std::cout << __PRETTY_FUNCTION__ << ": saving to 0x" << std::hex << getAddress() << std::endl;
                }
                // got a mismatch, so spill this frame to memory first
                saveRegisters(getUnderlyingFrame(), getAddress());
            }
            DEBUG_LOG_LEVEL(2) {
                std::cout << __PRETTY_FUNCTION__ << ": Setting up frame!" << std::endl;
            }
            _valid = true;
            synchronizeOwnership(newFP);
            restoreRegisters(getUnderlyingFrame(), getAddress());
            DEBUG_LEAVE_FUNCTION;
        }
        void
        clear() noexcept {
            _theFrame.clear();
        }
        /**
         * Makes sure that this set has an up to date frame pointer
         * @param fp The frame pointer to update the targetFramePointer field to
         */
        void
        synchronizeOwnership(Ordinal fp) noexcept {
            DEBUG_LOG_LEVEL(2) {
                std::cout << __PRETTY_FUNCTION__ << ": 0x" << std::hex << _targetFramePointer << " -> 0x" << fp << std::endl;
            }
            _targetFramePointer = fp;
        }
    private:
        RegisterFrame _theFrame;
        Address _targetFramePointer = 0;
        bool _valid = false;
    };
/**
 * @brief Holds onto two separate register frames
 */
    public:
        void begin() noexcept;
        BootResult start() noexcept;
        void cycle() noexcept;
    private:
        void lockBus() noexcept;
        void unlockBus() noexcept;
        /// @todo insert iac dispatch here
        /// @todo insert routines for getting registers and such
        auto& currentLocalRegisterSet() noexcept { return frames_[localRegisterFrameIndex_]; }
        const auto& currentLocalRegisterSet() const noexcept { return frames_[localRegisterFrameIndex_]; }
        auto& getLocals() noexcept { return currentLocalRegisterSet().getUnderlyingFrame(); }
        const auto& getLocals() const noexcept { return currentLocalRegisterSet().getUnderlyingFrame(); }
        template<typename T>
        [[nodiscard]] T& getGPR(ByteOrdinal index, TreatAs<T>) {
            DEBUG_ENTER_FUNCTION;
            DEBUG_LOG_LEVEL(3) {
                std::cout << __PRETTY_FUNCTION__ << ": index is 0x" << std::hex << static_cast<int>(index) << std::endl;
            }
            if (index >= 16) {
                DEBUG_LEAVE_FUNCTION;
                return globals_.get(index, TreatAs<T>{});
            } else {
                DEBUG_LEAVE_FUNCTION;
                return getLocals().get(index, TreatAs<T>{});
            }
        }
        template<typename T>
        [[nodiscard]] const T& getGPR(ByteOrdinal index, TreatAs<T>) const {
            DEBUG_ENTER_FUNCTION;
            DEBUG_LOG_LEVEL(3) {
                std::cout << __PRETTY_FUNCTION__ << ": index is 0x" << std::hex << static_cast<int>(index) << std::endl;
            }
            if (index >= 16) {
                DEBUG_LEAVE_FUNCTION;
                return globals_.get(index, TreatAs<T>{});
            } else {
                DEBUG_LEAVE_FUNCTION;
                return getLocals().get(index, TreatAs<T>{});
            }
        }
        [[nodiscard]] Register& getGPR(ByteOrdinal index) noexcept {
            return getGPR(index, TreatAsRegister{});
        }
        [[nodiscard]] const Register& getGPR(ByteOrdinal index) const noexcept {
            return getGPR(index, TreatAsRegister{});
        }
        [[nodiscard]] Register& getGPR(ByteOrdinal index, ByteOrdinal offset) noexcept { return getGPR((index + offset) & 0b11111); }
        [[nodiscard]] const Register& getGPR(ByteOrdinal index, ByteOrdinal offset) const noexcept { return getGPR((index + offset) & 0b11111); }
        [[nodiscard]] inline Ordinal getGPRValue(ByteOrdinal index, TreatAsOrdinal) const noexcept { return getGPR(index).getValue(TreatAsOrdinal{}); }
        [[nodiscard]] inline Ordinal getGPRValue(ByteOrdinal index, ByteOrdinal offset, TreatAsOrdinal) const noexcept { return getGPR(index, offset).getValue(TreatAsOrdinal{}); }
        [[nodiscard]] inline Integer getGPRValue(ByteOrdinal index, TreatAsInteger) const noexcept { return getGPR(index).getValue(TreatAsInteger{}); }
        template<typename T>
        [[nodiscard]] T getGPRValue(ByteOrdinal index) const noexcept {
            return getGPRValue(index, TreatAs<T>{});
        }
        [[nodiscard]] constexpr Ordinal getSystemAddressTableBase() const noexcept { return systemAddressTableBase_; }
        [[nodiscard]] Ordinal getSystemProcedureTableBase() const noexcept;
        [[nodiscard]] Ordinal getSupervisorStackPointer() const noexcept;
        inline void setGPR(ByteOrdinal index, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index).setValue(value, TreatAsOrdinal{}); }
        inline void setGPR(ByteOrdinal index, ByteOrdinal offset, Ordinal value, TreatAsOrdinal) noexcept { getGPR(index, offset).setValue(value, TreatAsOrdinal{}); }
        inline void setGPR(ByteOrdinal index, Integer value, TreatAsInteger) noexcept { getGPR(index).setValue(value, TreatAsInteger{}); }
        [[nodiscard]] Register& getSFR(ByteOrdinal index) noexcept;
        [[nodiscard]] Register& getSFR(ByteOrdinal index, ByteOrdinal offset) noexcept;
        [[nodiscard]] const Register& getSFR(ByteOrdinal index) const noexcept;
        [[nodiscard]] const Register& getSFR(ByteOrdinal index, ByteOrdinal offset) const noexcept;
        [[nodiscard]] const Register& getSrc1Register(TreatAsREG) const noexcept;
        [[nodiscard]] const Register& getSrc2Register(TreatAsREG) const noexcept;
        [[nodiscard]] Ordinal unpackSrc1(ByteOrdinal offset, TreatAsOrdinal, TreatAsREG) noexcept;
        template<typename Q>
        void moveGPR(ByteOrdinal destIndex, ByteOrdinal srcIndex, std::function<Q(Q)> transform, TreatAs<Q>) noexcept {
            DEBUG_ENTER_FUNCTION;
            auto result = transform(getGPRValue(srcIndex, TreatAs<Q>{}));
            DEBUG_LOG_LEVEL(4) {
                std::cout << __PRETTY_FUNCTION__ << ": transformation result 0x" << std::hex << result << std::endl;
            }
            setGPR(destIndex, result, TreatAs<Q>{});
            DEBUG_LEAVE_FUNCTION;
        }
        template<typename Q>
        void moveGPR(ByteOrdinal destIndex, ByteOrdinal srcIndex, TreatAs<Q>) noexcept {
            setGPR(destIndex, getGPRValue(srcIndex, TreatAs<Q>{}), TreatAs<Q>{});
        }
        template<typename Q>
        void moveGPR(ByteOrdinal destIndex, ByteOrdinal destOffset, ByteOrdinal srcIndex, ByteOrdinal srcOffset, TreatAs<Q>) noexcept {
            setGPR(destIndex, destOffset, getGPRValue(srcIndex, srcOffset, TreatAs<Q>{}), TreatAs<Q>{});
        }
        [[nodiscard]] bool getMaskedConditionCode(uint8_t mask) const noexcept;
        [[nodiscard]] bool conditionCodeEqualsMask(uint8_t mask) const noexcept;
        bool fullConditionCodeCheck() noexcept;
        bool fullConditionCodeCheck(uint8_t mask) noexcept;
        Ordinal computeAddress() noexcept;
        void performRegisterTransfer(ByteOrdinal mask, ByteOrdinal count) noexcept;
    private:
        void sendIAC(const iac::Message& msg) noexcept;
        void dispatchInterrupt(uint8_t vector) noexcept;
        void purgeInstructionCache() noexcept;
        void reinitializeProcessor(Ordinal satBase, Ordinal prcbBase, Ordinal startIP) noexcept;
        void setBreakpointRegister(Ordinal breakpointIp0, Ordinal breakpointIp1) noexcept;
        void storeSystemBase(Ordinal destinationAddress) noexcept;
        void testPendingInterrupts();
#if 0
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
        void scanbyte(Ordinal src1, Ordinal src2) noexcept;
        void emul(LongRegister& dest, Ordinal src1, Ordinal src2) noexcept;
        void ediv(LongRegister& dest, Ordinal src1, const LongRegister& src2) noexcept;
        void arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) noexcept;
        inline void advanceCOBRDisplacement(Integer displacement) noexcept {
            ip_.i += displacement;
            advanceInstruction_ = false;
        }
        template<bool checkClear>
        void 
        branchIfBitGeneric(Ordinal bitpos, const Register& src2 , int16_t displacement) {
            Ordinal against = src2.getValue<Ordinal>();
            DEBUG_LOG_LEVEL(1) {
                std::cout << __PRETTY_FUNCTION__ << ": bitpos: 0x" << std::hex << bitpos
                << ", src2: 0x" << std::hex << against << std::endl;
            }
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
            DEBUG_LOG_LEVEL(1) {
                std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": before: "
                          << "src1: 0x" << std::hex << src1
                          << ", src2: 0x" << std::hex << src2
                          << std::endl;
            }
            ac_.arith.conditionCode = performCompare<Q>(src1, src2);
            DEBUG_LOG_LEVEL(1) {
                std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": after: "
                          << "ac.cc = 0x" << std::hex << static_cast<int>(ac_.getConditionCode())
                          << std::endl;
            }
        }
        template<typename Q>
        void cmpxbGeneric(uint8_t mask, Q src1, Q src2, int16_t displacement, TreatAs<Q>) noexcept {
            DEBUG_LOG_LEVEL(1) {
                std::cout << "\t\t" << __PRETTY_FUNCTION__
                << ": mask: 0x" << std::hex << static_cast<int>(mask)
                << ", src1: 0x" << std::hex << src1
                << ", src2: 0x" << std::hex << src2
                << ", displacement: 0x" << std::hex << displacement << std::endl;
            }
            cmpGeneric<Q>(src1, src2);
            DEBUG_LOG_LEVEL(1) {
                std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": ac result: 0x" << std::hex << ac_.getConditionCode() << std::endl;
            }
            if ((mask & ac_.getConditionCode()) != 0) {
                DEBUG_LOG_LEVEL(1) {
                    std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": branch taken" << std::endl;
                }
                advanceCOBRDisplacement(displacement);
            } else {
                DEBUG_LOG_LEVEL(1) {
                    std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": branch not taken" << std::endl;
                }
            }
        }
        inline void cmpobGeneric(uint8_t mask, Ordinal src1, Ordinal src2, int16_t displacement) noexcept {
            cmpxbGeneric(mask, src1, src2, displacement, TreatAsOrdinal{}); 
        }
        inline void cmpibGeneric(uint8_t mask, Integer src1, Integer src2, int16_t displacement) noexcept { 
            cmpxbGeneric(mask, src1, src2, displacement, TreatAsInteger{});
        }
        void flushreg() noexcept;
        void balx(ByteOrdinal linkRegister, Ordinal branchTo) noexcept;
        void calls(Ordinal value) noexcept;
        void ldl(Address address, LongRegister& destination) noexcept;
        void ldq(Address address, QuadRegister& destination) noexcept;
        void ldt(Address address, TripleRegister& destination) noexcept;
        void stq(Address address, const QuadRegister& src) noexcept;
        void stt(Address address, const TripleRegister& src) noexcept;
        void stl(Address address, const LongRegister& src) noexcept;
        void ret() noexcept;
        void call(Integer displacement) noexcept;
        void callx(Address effectiveAddress) noexcept;
    private:
        void enterCall(Ordinal fp);
        void leaveCall();
        void restoreStandardFrame() noexcept;
    private:
        void performConditionalSubtract(Register& dest, Integer src1, Integer src2, TreatAsInteger) noexcept;
        void performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) noexcept;
        void performConditionalAdd(Register& dest, Integer src1, Integer src2, TreatAsInteger) noexcept;
        void performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) noexcept;
        void performSelect(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    protected:
#define X(type) \
        type load(Address addr, TreatAs< type > ) const noexcept; \
        void store(Address addr, type value, TreatAs< type > ) noexcept
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
            destination.setValue(::orOperation<Ordinal, invert>(src1, src2), TreatAsOrdinal{});
        }
        template<bool invert = false>
        inline void andOperation(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            destination.setValue(::andOperation<Ordinal, invert>(src1, src2), TreatAsOrdinal{});
        }
        template<bool invert = false>
        inline void xorOperation(Register& destination, Ordinal src1, Ordinal src2) noexcept {
            destination.setValue(::xorOperation<Ordinal, invert>(src1, src2), TreatAsOrdinal{});
        }
        template<typename Q>
        void add(Register& destination, Q src1, Q src2, TreatAs<Q>) noexcept {
            auto result = ::addOperation<Q>(src2, src1);
            DEBUG_LOG_LEVEL(1) {
                std::cout << __PRETTY_FUNCTION__ << ": (+ 0x"
                          << std::hex << src2
                          << " 0x" << std::hex << src1
                          << ") -> 0x" << std::hex << result
                          << std::endl;
            }
            destination.setValue(result, TreatAs<Q>{});
        }
        template<typename Q>
        void sub(Register& destination, Q src1, Q src2, TreatAs<Q>) noexcept {
            auto result = ::subOperation<Q>(src2, src1);
            DEBUG_LOG_LEVEL(1) {
                std::cout << __PRETTY_FUNCTION__ << ": (- 0x"
                          << std::hex << src2
                          << " 0x" << std::hex << src1
                          << ") -> 0x" << std::hex << result
                          << std::endl;
            }
            destination.setValue(result, TreatAs<Q>{});
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
                zeroDivideFault();
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
            DEBUG_LOG_LEVEL(4) {
                std::cout << "addc result: 0x" << std::hex << result << std::endl;
            }
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
                zeroDivideFault();
            } else {
                // taken from the i960Sx manual
                //dest.setValue(src2 - ((src2 / src1) * src1), TreatAs<Q>{});
                dest.setValue(src2 % src1, TreatAs<Q>{});
                nextInstruction();
            }
        }
        void remi(Register& dest, Integer src1, Integer src2) noexcept {
            remainderOperation<Integer>(dest, src1, src2);
            nextInstruction();
            faultOnOverflow(dest);
        }
        void remo(Register& dest, Ordinal src1, Ordinal src2) noexcept {
            remainderOperation<Ordinal>(dest, src1, src2);
        }
        template<typename Q>
        void divideOperation(Register& dest, Q src1, Q src2) noexcept {
            if (src1 == 0) {
                /// @todo fix this
                zeroDivideFault();
            } else {
                dest.setValue(src2 / src1, TreatAs<Q>{});
                nextInstruction();
            }
        }
        void divi(Register& dest, Integer src1, Integer src2) noexcept {
            divideOperation<Integer>(dest, src1, src2);
            faultOnOverflow(dest);
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
        void zeroDivideFault() ;
        void integerOverflowFault() ;
        void constraintRangeFault() ;
        void invalidSSFault() ;
        void unimplementedFault();
        void typeMismatchFault();
        void typeContentsFault();
        void markTraceFault();
        void invalidOpcodeFault();
        void protectionLengthFault();
        void invalidOperandFault();
        void invalidDescriptorFault(SegmentSelector selector);
        void eventNoticeFault();
        void generateFault(const FaultRecord& record);
        void addi(Register& dest, Integer src1, Integer src2) noexcept;
        void addo(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void saveReturnAddress(ByteOrdinal registerIndex) noexcept;
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
        void shro(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void mulo(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void muli(Register& dest, Integer src1, Integer src2) noexcept;
        void modify(Register& dest, Ordinal src1, Ordinal src2) noexcept;
        void extract(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    protected:
        static decltype(auto) doRandom() {
            return rand();
        }
        static decltype(auto) doRandomDisallow0() {
            while (true) {
                auto r = rand();
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
        bool runNonPortableSelfTests() noexcept;
        void nonPortableBegin() noexcept;
    private:
        // fault handling components
        void pushFaultRecord(Address baseStorageAddress, const FaultRecord& record) noexcept;
        FaultTableEntry getFaultEntry(uint8_t index) const noexcept;
        void faultCallGeneric (const FaultRecord& record, Address destination, Address stackPointer) noexcept;
        void localProcedureEntry_FaultCall (const FaultRecord& record, Address destination) noexcept;
        void procedureTableEntry_FaultCall(const FaultRecord& record, const FaultTableEntry& entry) noexcept;
        void supervisorProcedureTableEntry_FaultCall(const FaultRecord& record, Address procedureAddress, Address tableBaseAddress) noexcept;
    private:
        template<uint8_t offset>
        Ordinal getFromPRCB() const noexcept { return load(prcbAddress_ + offset, TreatAsOrdinal{}); }
        Ordinal getProcessorControls() const noexcept { return getFromPRCB<4>(); }
        Ordinal getCurrentProcessSegmentSelector() const noexcept { return getFromPRCB<12>(); }
        Ordinal getDispatchPortSegmentSelector() const noexcept { return getFromPRCB<16>(); }
        Address getInterruptTablePointer() const noexcept { return getFromPRCB<20>(); }
        Address getInterruptStackAddress() const noexcept { return getFromPRCB<24>(); }
        Address getRegion3SegmentSelector() const noexcept { return getFromPRCB<32>(); }
        Address getSystemProcedureTableSegmentSelector() const noexcept { return getFromPRCB<36>(); }
        Address getFaultTableBaseAddress() const noexcept { return getFromPRCB<40>(); }
    private: // system address table / system procedure table
        SegmentDescriptor loadSegmentDescriptor(SegmentSelector offset) const noexcept;
    public:
        void checksumFail();
        void selfTestFailure();
    private:
        void badFault(const FaultRecord& record);
    private: // numerics extensions
        void dmovt(Register& dest, Ordinal src) noexcept;
        void classr(Real src) noexcept;
        void classrl(LongReal src) noexcept;
        void cosr(Register& dest, Real literal) noexcept;
        void cosrl(LongRegister& dest, LongReal literal) noexcept;
        inline void cosr(Register& dest, const Register& src) noexcept { cosr(dest, (Real)src); }
        inline void cosrl(LongRegister& dest, const LongRegister& src) noexcept { cosrl(dest, src.getValue<LongReal>()); }
        void sinr(Register& dest, Real literal) noexcept;
        void sinrl(LongRegister& dest, LongReal literal) noexcept;
        inline void sinr(Register& dest, const Register& src) noexcept { sinr(dest, (Real)src); }
        inline void sinrl(LongRegister& dest, const LongRegister& src) noexcept { sinrl(dest, src.getValue<LongReal>()); }
        void tanr(Register& dest, Real literal) noexcept;
        void tanrl(LongRegister& dest, LongReal literal) noexcept;
        inline void tanr(Register& dest, const Register& src) noexcept { tanr(dest, (Real)src); }
        inline void tanrl(LongRegister& dest, const LongRegister& src) noexcept { tanrl(dest, src.getValue<LongReal>()); }
        /// @todo add support for the dedicated floating point registers as overloaded forms
    private: // protected extensions instructions
        void signal(SegmentSelector sel) noexcept;
        void wait(SegmentSelector src) noexcept;
        void sendserv(SegmentSelector src) noexcept;
        void send(SegmentSelector target, Ordinal src1, SegmentSelector src2);
        void schedprcs(SegmentSelector src) noexcept;
        void saveprcs() noexcept;
        void resumprcs(SegmentSelector ss) noexcept;
        void receive(SegmentSelector src, Register& dest) noexcept;
        void ldtime(Register& dest) noexcept;
        void ldphy(Address address, Register& dest) noexcept;
        void inspacc(Ordinal src, Register& dest) noexcept;
        void condwait(SegmentSelector src) noexcept;
        void condrec(SegmentSelector src, Register& dest) noexcept;
        void cmpstr(Ordinal src1, Ordinal src2, Ordinal len) noexcept;
        void movstr(Ordinal destAddress, Ordinal srcAddress, Ordinal len) noexcept;
        void movqstr(Ordinal destAddress, Ordinal srcAddress, Ordinal len) noexcept;
        void fill(Ordinal dest, Ordinal value, Ordinal len) noexcept;
    private: // interrupt related
        Address getInterruptTableBaseAddress() const;
        void postInterrupt(InterruptVector vector);
        Ordinal getInterruptPendingPriorities() const { return load(getInterruptTablePointer(), TreatAsOrdinal{}); }
        void setInterruptPendingPriorities(Ordinal value) { store(getInterruptTablePointer(), value, TreatAsOrdinal{}); }
        Ordinal getPendingInterruptWord(uint8_t index) const {
            return load(getInterruptTableBaseAddress() + 4 + (sizeof(Ordinal) * (index & 0b111)), TreatAsOrdinal{});
        }
        Ordinal getPendingInterruptWord(InterruptVector vector) const {
            return getPendingInterruptWord(computeInterruptWordIndex(vector));
        }
        void setPendingInterruptWord(uint8_t index, Ordinal value) {
            store(getInterruptTableBaseAddress() + 4 + (sizeof(Ordinal) * (index & 0b111)), value, TreatAsOrdinal{});
        }
        void setPendingInterruptWord(InterruptVector vector, Ordinal value) {
            setPendingInterruptWord(computeInterruptWordIndex(vector), value);
        }

        Address getInterruptVectorAddress(uint8_t vector) const;
        Address getInterruptVectorAddress(InterruptVector vector) const { return getInterruptVectorAddress(static_cast<uint8_t>(vector)); }
        void receiveInterrupt(InterruptVector vector);
        void setPendingPriorityBit(uint8_t priority);
        void setPendingPriorityBit(InterruptVector vector) { setPendingPriorityBit(computeInterruptPriority(vector)); }
        void clearPendingPriorityBit(uint8_t priority);
        void clearPendingPriorityBit(InterruptVector vector) { clearPendingPriorityBit(computeInterruptPriority(vector)); }
        bool getPendingPriorityBit(uint8_t priority) const;
        bool getPendingPriorityBit(InterruptVector vector) const { return getPendingPriorityBit(computeInterruptPriority(vector)); }
        bool getPendingInterruptBit(InterruptVector vector) const;
        void setPendingInterruptBit(InterruptVector vector);
        void clearPendingInterruptBit(InterruptVector vector);
        void obtainedPendingVector(InterruptVector vector);
        bool pendingInterruptPriorityClear(InterruptVector vector) const;
        ByteOrdinal getPendingInterruptBitsForPriority(uint8_t priority) const;
        ByteOrdinal getHighestPostedInterruptVectorForPriority(uint8_t priority) const;
        InterruptVector highestPostedInterruptVector() const;
        InterruptVector serviceNextInterrupt();
        void serviceInterrupt(InterruptVector vector);
        void checkForPendingInterrupts();
    private:
        void localReturn();
        void faultReturn();
        void interruptReturn();
        void supervisorReturn(bool traceModeSetting);
    private:
        void faultOnOverflow(Register& dest);
        [[nodiscard]] Ordinal getFramePointerAddress() const { return getGPRValue(FPIndex, TreatAsOrdinal{}); }
        void restoreRIPToIP();
        [[nodiscard]] Ordinal getRIPContents() const { return getGPRValue(RIPIndex, TreatAsOrdinal{}); }
    private:
        Ordinal restorePCFromStack(Ordinal fp);
        Ordinal restoreACFromStack(Ordinal fp);
    private:
        void saveRegisterFrame(const RegisterFrame& theFrame, Address baseAddress);
        void restoreRegisterFrame(RegisterFrame& theFrame, Address baseAddress);
        LocalRegisterSet& getNextPack() noexcept {
            if constexpr (NumberOfLocalRegisterFrames > 1) {
                // do these as separate operations, otherwise gcc generates garbage
                uint8_t result = (localRegisterFrameIndex_ + 1);
                result %= NumberOfLocalRegisterFrames;
                return frames_[result];
            } else {
                return frames_[0];
            }
        }
        LocalRegisterSet& getPreviousPack() noexcept {
            if constexpr (NumberOfLocalRegisterFrames > 1) {
                // do these as separate operations, otherwise gcc generates garbage
                uint8_t result = (localRegisterFrameIndex_ - 1);
                result %= NumberOfLocalRegisterFrames;
                DEBUG_LOG_LEVEL(2) {
                    std::cout << __PRETTY_FUNCTION__ << ": 0x" << std::hex << static_cast<int>(result) << std::endl;
                }
                return frames_[result];
            } else {
                return frames_[0];
            }
        }
        [[nodiscard]] LocalRegisterSet& getCurrentPack() noexcept { return frames_[localRegisterFrameIndex_]; }
        void boot0(Address sat, Address pcb, Address startIP) noexcept;
    private:
        void bx(Address effectiveAddress);
        void bal(Integer displacement);
    private:
        Ordinal getElapsedExecutionTime() const noexcept { return executionTime_.getValue<Ordinal>(); }
        void setElapsedExecutionTime(Ordinal value) noexcept { executionTime_.setValue<Ordinal>(value); }
        Ordinal getResidualTimeSlice() const noexcept { return residualTimeSlice_.getValue<Ordinal>(); }
        void setResidualTimeSlice(Ordinal value) noexcept { residualTimeSlice_.setValue<Ordinal>(value); }
        Address translateToPhysicalAddress(Address virtualAddress) const noexcept;
    private:
        Ordinal systemAddressTableBase_ = 0;
        Ordinal prcbAddress_ = 0;
        RegisterFrame globals_;
        std::array<LocalRegisterSet, NumberOfLocalRegisterFrames> frames_;
        uint8_t localRegisterFrameIndex_ = 0;
        RegisterBlock32 sfrs_;
        RegisterBlock32 constants_;
        Register ip_;
        Register ac_; 
        Register pc_;
        Register tc_; 
        Register instruction_;
        Register ictl_;
        ByteOrdinal advanceBy_;
        ByteOrdinal instructionLength_ = 0;
        bool advanceInstruction_ = false;
        Address breakpoint0_ = 0;
        bool breakpoint0Active_ = false;
        Address breakpoint1_ = 0;
        bool breakpoint1Active_ = false;
        // protected architecture extensions
        Register executionTime_;
        Register residualTimeSlice_;
};

static_assert(computeNextFrame<Core::C, Core::NotC>(0xFDED'0000) == 0xFDED'0000);
static_assert(computeNextFrame<Core::C*2, Core::NotC>(0xFDED'0000) == 0xFDED'0040);
static_assert(computeNextFrame<Core::C*3, Core::NotC>(0xFDED'0000) == 0xFDED'0080);
static_assert(computeNextFrame<Core::C*4, Core::NotC>(0xFDED'0000) == 0xFDED'00C0);

void installToMainMemory(std::istream& stream, Address baseAddress);
#endif // end SIM5_CORE_H__
