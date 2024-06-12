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
#include <istream>
#include <iostream>
#include <optional>
#include <variant>
#include <cfenv>
#include "Types.h"
#include "IAC.h"
#include "BinaryOperations.h"
#include "ProcessManagement.h"
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
constexpr auto TypeContentsFault = ContentsFault;
constexpr Ordinal TimeSliceFault = 0x000c'0001;
constexpr Ordinal InvalidDescriptorFault = 0x000d'0001;
constexpr Ordinal EventNoticeFault = 0x000e'0001;
constexpr Ordinal OverrideFault = 0x0010'0000;

constexpr Ordinal FloatingPointFaultBase = 0x0004'0000;
template<uint8_t subtype>
constexpr Ordinal FloatingPointFaultSubType= FloatingPointFaultBase | subtype;

constexpr auto FloatingPointOverflowFault = FloatingPointFaultSubType<0b0000'0001>;
constexpr auto FloatingPointUnderflowFault  = FloatingPointFaultSubType<0b0000'0010>;
constexpr auto FloatingPointInvalidOperationFault  = FloatingPointFaultSubType<0b0000'0100>;
constexpr auto FloatingPointZeroDivideOperationFault = FloatingPointFaultSubType<0b0000'1000>;
constexpr auto FloatingPointInexactFault = FloatingPointFaultSubType<0b0001'0000>;
constexpr auto FloatingPointReservedEncodingFault = FloatingPointFaultSubType<0b0010'0000>;

constexpr bool isREGFormatOpcode(uint8_t value) noexcept {
    return (value >= 0x40) && (value < 0x80);
}
constexpr bool isMEMFormatOpcode(uint8_t value) noexcept {
    return (value >= 0x80);
}
constexpr bool isCOBRFormatOpcode(uint8_t value) noexcept {
    return (value >= 0x20) && (value < 0x40);
}
constexpr bool isCTRLFormatOpcode(uint8_t value) noexcept {
    return (value < 0x20);
}

constexpr uint32_t constructInplaceMask(uint8_t major, uint8_t minor) noexcept {
    auto full = static_cast<uint32_t>(major) << 24;
    if (isREGFormatOpcode(major)) {
        return full | static_cast<uint32_t>(static_cast<uint32_t>(minor) << 7);
    } else {
        return full;
    }
}

enum class RawOpcodes : uint32_t {
#define X(name, opcode, str, level, privileged, fmt, flt, minor) name = constructInplaceMask(opcode, minor),
#include "Opcodes.def"
#undef X
};
static_assert(static_cast<uint32_t>(RawOpcodes::b) == 0x0800'0000);
static_assert(static_cast<uint32_t>(RawOpcodes::andnot) == static_cast<uint32_t>(0b0101'1000'00000'00000'000'0010'00'00000));

constexpr uint16_t constructOpcode(uint8_t major, uint8_t minor) noexcept {
    return isREGFormatOpcode(major) ? ((static_cast<uint16_t>(major) << 4) | static_cast<uint16_t>(minor)) : static_cast<uint16_t>(major);
}
enum class Opcodes : uint16_t {
#define X(name, opcode, str, level, privileged, fmt, flt, minor) name = constructOpcode(opcode, minor),
#include "Opcodes.def"
#undef X
};
// sanity checking
static_assert(static_cast<uint16_t>(Opcodes::rotate) == 0x59d);
constexpr bool isPrivileged(Opcodes code) noexcept {
    switch (code)  {
#define X(name, opcode, str, level, privileged, fmt, flt, minor) case Opcodes:: name : return privileged;
#include "Opcodes.def"
#undef X
        default:
            return false;
    }
}
template<Opcodes code>
constexpr bool IsPrivileged = isPrivileged(code);
static_assert(IsPrivileged<Opcodes::send>);
static_assert(!IsPrivileged<Opcodes::addi>);
constexpr ArchitectureLevel getArchitectureLevel(Opcodes code) noexcept {
    switch (code) {
#define X(name, opcode, str, level, privileged, fmt, flt, minor) case Opcodes :: name : return ArchitectureLevel :: level ;
#include "Opcodes.def"
#undef X
        default:
            return ArchitectureLevel::Unknown;
    }
}
template<Opcodes code>
constexpr auto ArchitectureLevel_v = getArchitectureLevel(code);
static_assert(ArchitectureLevel_v<Opcodes::b> == ArchitectureLevel::Core);

constexpr bool isFloatingPointInstruction(Opcodes code) noexcept {
    switch (code) {
#define X(name, opcode, str, level, privileged, fmt, flt, minor) case Opcodes :: name : return flt ;
#include "Opcodes.def"
#undef X
        default:
            return false;
    }
}
template<Opcodes code>
constexpr auto IsFloatingPointInstruction_v = isFloatingPointInstruction(code);
static_assert(IsFloatingPointInstruction_v<Opcodes::cpyrsre>);
static_assert(IsFloatingPointInstruction_v<Opcodes::movre>);
enum class BootResult : uint8_t {
    Success,
    SelfTestFailure,
    ChecksumFail,
};
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
    [[nodiscard]] constexpr auto getConditionCode() const noexcept { return arith.conditionCode; }
    void setConditionCode(uint8_t code) noexcept { arith.conditionCode = code; }
    void setConditionResult(bool state) noexcept { setConditionCode(state ? 0b010 : 0b000); }
    void clearConditionCode() noexcept { arith.conditionCode = 0b000; }
    void setPriority(Ordinal value) noexcept { processControls.priority = value; }
    [[nodiscard]] constexpr ByteOrdinal getPriority() const noexcept { return processControls.priority; }
    [[nodiscard]] constexpr bool inSupervisorMode() const noexcept { return processControls.executionMode; }

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
        BackingUnionType unused : 4;
        BackingUnionType timeSliceReschedule :1;
        BackingUnionType timeSlice : 1;
        BackingUnionType timing : 1;
        BackingUnionType resume : 1;
        BackingUnionType traceFaultPending : 1;
        BackingUnionType preempt : 1;
        BackingUnionType refault : 1;
        BackingUnionType state : 2;
        BackingUnionType priority : 5;
        BackingUnionType internalState : 11;
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
    struct {
        BackingUnionType b0 : 1;
        BackingUnionType multiprocessorPreempt : 1;
        BackingUnionType state : 2;
        BackingUnionType b3 : 1;
        BackingUnionType nonpreemptLimit : 5;
        BackingUnionType addressingMode : 1;
        BackingUnionType checkDispatchPort : 1;
        BackingUnionType b7 : 4;
        BackingUnionType interimPriority : 5;
        BackingUnionType rest : 11;
    } processorControls;
    [[nodiscard]] constexpr bool inVirtualAddressingMode() const noexcept { return processorControls.addressingMode; }
    struct {
        BackingUnionType v : 1;
        BackingUnionType pageRights : 2;
        BackingUnionType unused : 9;
        BackingUnionType baseAddress : 20;
    } pageTableDirectoryEntry;
    struct {
        BackingUnionType v : 1;
        BackingUnionType pageRights : 2;
        BackingUnionType accessed : 1;
        BackingUnionType altered : 1;
        BackingUnionType unused0 : 1;
        BackingUnionType c : 1;
        BackingUnionType unused1 : 1;
        BackingUnionType unused2 : 4;
        BackingUnionType baseAddress : 20;
    } pageTableEntry;

    /// @todo add support for external IAC processing
    void clear() noexcept {
        o = 0;
    }
    [[nodiscard]] constexpr Ordinal getPFPAddress() noexcept {
        Register copy(o);
        copy.pfpAddress.align = 0;
        return copy.o;
    }
    [[nodiscard]] constexpr ByteOrdinal getReturnType() const noexcept { return pfp.rt; }
    [[nodiscard]] constexpr bool isCTRL() const noexcept { return ::isCTRL(o); }
    [[nodiscard]] constexpr bool isCOBR() const noexcept { return ::isCOBR(o); }
    [[nodiscard]] constexpr bool isMEMFormat() const noexcept { return ::isMEMFormat(o); }
    [[nodiscard]] constexpr auto isREGFormat() const noexcept { return ::isREGFormat(o); }
    [[nodiscard]] constexpr bool getCarryBit() const noexcept { return arith.conditionCode & 0b010; }
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
    Register& operator=(Integer value) noexcept {
        i = value;
        return *this;
    }
    constexpr bool operator==(const Register& other) const noexcept {
        return other.o == o;
    }
    constexpr bool operator!=(const Register& other) const noexcept {
        return other.o != o;
    }
    [[nodiscard]] constexpr Ordinal asBitPosition() const noexcept {
        return computeBitPosition(bytes[0]);
    }
};
static_assert(sizeof(Register) == sizeof(Ordinal));
union LongRegister {
public:
    explicit LongRegister(LongOrdinal value) noexcept : lo(value) { }
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
    [[nodiscard]] constexpr QuadOrdinal getValue(TreatAsQuadOrdinal) const noexcept { return ord_; }
    void setValue(QuadOrdinal value, TreatAsQuadOrdinal) noexcept { ord_ = value; }
    Register& get(ByteOrdinal index) noexcept { return quads_[index & 0b11]; }
    const Register& get(ByteOrdinal index) const noexcept { return quads_[index & 0b11]; }
    Register& operator[](ByteOrdinal index) noexcept { return get(index); }
    const Register& operator[](ByteOrdinal index) const noexcept { return get(index); }
    bool operator==(const QuadRegister& other) const noexcept {
        return ord_ == other.ord_;
    }
    bool operator!=(const QuadRegister& other) const noexcept {
        return other.ord_ != ord_;
    }
    explicit constexpr operator QuadOrdinal() const noexcept { return ord_; }
private:
    Register quads_[4];
    QuadOrdinal ord_;
};
static_assert(sizeof(QuadRegister) == (2*sizeof(LongOrdinal)));
using TreatAsQuadRegister = TreatAs<QuadRegister>;
union TripleRegister {
public:
    TripleRegister() = default;
    explicit constexpr TripleRegister(ExtendedReal value) noexcept : extendedReal_(value) {}
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
    TripleRegister& operator=(ExtendedReal value) noexcept {
        setValue(value, TreatAsExtendedReal{});
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
    void setValue(ExtendedReal value, TreatAsExtendedReal) noexcept {
        // this can lead to garbage so make sure that we mask out the upper 16-bits
        extendedReal_ = value;
        // clear out the uppermost 16-bits
        backingStore_[2].o &= 0x0000'FFFF;
    }
    template<typename T>
    constexpr T getValue() const noexcept {
        return getValue(TreatAs<T>{});
    }

private:
    QuadRegister backingStore_;
    ExtendedReal extendedReal_;
};

/**
 * @brief a 80-bit floating point register
 */
union FloatingPointRegister {
    long double floatValue;
    Ordinal components[3];
};
using TreatAsTripleRegister = TreatAs<TripleRegister>;
template<typename T>
concept MustBeRegisterType = std::same_as<T, Register> ||
                             std::same_as<T, LongRegister> ||
                             std::same_as<T, TripleRegister> ||
                             std::same_as<T, QuadRegister>;
class CTRLInstruction {
public:
    constexpr CTRLInstruction(ByteOrdinal code, Integer disp) : displacement(disp), opcode(code) {}
    explicit constexpr CTRLInstruction(Ordinal value) : backingStore_(value) {}
    explicit CTRLInstruction(const Register& backingStore) : CTRLInstruction(static_cast<Ordinal>(backingStore)) { }
    [[nodiscard]] constexpr Opcodes getOpcode() const noexcept { return static_cast<Opcodes>(opcode); }
    [[nodiscard]] constexpr Integer getDisplacement() const noexcept { return alignTo4ByteBoundaries(displacement, TreatAsInteger{}); }
    [[nodiscard]] constexpr bool getPredictionBit() const noexcept { return (displacement & 0b10); }
    [[nodiscard]] constexpr bool predictedTaken() const noexcept { return getPredictionBit() != 0; }
    [[nodiscard]] constexpr bool predictedNotTaken() const noexcept { return getPredictionBit() == 0; }
    [[nodiscard]] constexpr Ordinal getValue() const noexcept { return backingStore_; }
private:
    union {
        Ordinal backingStore_;
        struct {
            /// @todo check the bp and sfr bits eventually
            Integer displacement: 24;
            BackingUnionType opcode: 8;
        };
    };
};
constexpr CTRLInstruction dummyBranch{0x8, 0xFDEC};
static_assert(dummyBranch.getOpcode() == Opcodes::b);
static_assert(dummyBranch.getDisplacement() == 0xFDEC);
static_assert(dummyBranch.predictedNotTaken());
struct COBRInstruction {
public:
    constexpr COBRInstruction(ByteOrdinal code, ByteOrdinal s1, ByteOrdinal s2, bool m1_, Integer disp) : displacement(disp), m1(m1_), src2(s2), src1(s1), opcode(code) {}
    explicit constexpr COBRInstruction(Ordinal value) : raw_(value) { }
    explicit COBRInstruction(const Register& backingStore) : COBRInstruction((Ordinal)backingStore) {}
    [[nodiscard]] constexpr Integer getDisplacement() const noexcept { return alignTo4ByteBoundaries(displacement, TreatAsInteger{}); }
    [[nodiscard]] constexpr bool getS2() const noexcept { return (displacement & 0b1); }
    [[nodiscard]] constexpr bool getT() const noexcept { return (displacement & 0b10); }
    [[nodiscard]] constexpr bool getM1() const noexcept { return m1; }
    [[nodiscard]] constexpr ByteOrdinal getSrc2() const noexcept { return src2; }
    [[nodiscard]] constexpr ByteOrdinal getSrc1() const noexcept { return src1; }
    [[nodiscard]] constexpr Opcodes getOpcode() const noexcept { return static_cast<Opcodes>(opcode); }
    [[nodiscard]] constexpr Ordinal getValue() const noexcept { return raw_; }
    [[nodiscard]] constexpr ByteOrdinal getMask() const noexcept { return opcode & 0b111; }
private:
    union {
        Ordinal raw_;
        struct {
            Integer displacement : 13;
            BackingUnionType m1: 1;
            BackingUnionType src2: 5;
            BackingUnionType src1: 5;
            BackingUnionType opcode : 8;
        };
    };
};
constexpr COBRInstruction dummyCOBR { 0x32, 4, 5, false, 0xFF };
static_assert(dummyCOBR.getDisplacement() == 0xFC);
static_assert(dummyCOBR.getS2() == 1);
static_assert(dummyCOBR.getT());
static_assert(dummyCOBR.getSrc1() == 4);
static_assert(dummyCOBR.getSrc2() == 5);
static_assert(dummyCOBR.getOpcode() == Opcodes::cmpobe);
static_assert(dummyCOBR.getMask() == 0b010);
struct REGInstruction {
public:
    static constexpr Opcodes computeOpcode(ByteOrdinal opcode, ByteOrdinal opcode2) {
        return static_cast<Opcodes>((static_cast<ShortOrdinal>(opcode) << 4) | static_cast<ShortOrdinal>(opcode2));
    }
    constexpr REGInstruction(ByteOrdinal opcodePrimary,
                             ByteOrdinal sd,
                             ByteOrdinal src2r,
                             bool mf3, bool mf2, bool mf1,
                             ByteOrdinal opcode2,
                             bool sf2,
                             bool sf1,
                             ByteOrdinal src1r) : src1(src1r),
                                                  s1(sf1),
                                                  s2(sf2),
                                                  opcodeExt(opcode2),
                                                  m1(mf1),
                                                  m2(mf2),
                                                  m3(mf3),
                                                  src2(src2r),
                                                  srcDest(sd),
                                                  opcode(opcodePrimary)
                                                  {

    }

    explicit constexpr REGInstruction(Ordinal value) : raw_(value) { }
    explicit REGInstruction(const Register& backingStore) : REGInstruction(static_cast<Ordinal>(backingStore)) { }
    [[nodiscard]] constexpr Ordinal getValue() const noexcept { return raw_; }
    [[nodiscard]] constexpr Opcodes getOpcode() const noexcept { return computeOpcode(opcode, opcodeExt); }
    [[nodiscard]] constexpr ByteOrdinal getPrimaryOpcode() const noexcept { return opcode; }
    [[nodiscard]] constexpr ByteOrdinal getSecondaryOpcode() const noexcept { return opcodeExt; }
    [[nodiscard]] constexpr bool getS1() const noexcept { return s1; }
    [[nodiscard]] constexpr bool getS2() const noexcept { return s2; }
    [[nodiscard]] constexpr bool getM1() const noexcept { return m1; }
    [[nodiscard]] constexpr bool getM2() const noexcept { return m2; }
    [[nodiscard]] constexpr bool getM3() const noexcept { return m3; }
    [[nodiscard]] constexpr ByteOrdinal getSrc2() const noexcept { return src2; }
    [[nodiscard]] constexpr ByteOrdinal getSrc1() const noexcept { return src1; }
    [[nodiscard]] constexpr ByteOrdinal getSrcDest() const noexcept { return srcDest; }
    [[nodiscard]] constexpr ByteOrdinal getMask() const noexcept { return opcode & 0b111; }
    [[nodiscard]] constexpr bool src1IsLiteral() const noexcept { return getM1(); }
    [[nodiscard]] constexpr bool src2IsLiteral() const noexcept { return getM2(); }
    [[nodiscard]] constexpr bool src1IsFPLiteral() const noexcept {
        if (getM1()) {
            switch (src1) {
                case 0b10110:
                case 0b10000:
                    return true;
                default:
                    return false;
            }
        } else {
            return false;
        }
    }
    [[nodiscard]] constexpr bool src2IsFPLiteral() const noexcept {
        if (getM2()) {
            switch (src2) {
                case 0b10110:
                case 0b10000:
                    return true;
                default:
                    return false;
            }
        } else {
            return false;
        }
    }
private:
    union {
        Ordinal raw_;
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
        };
    };
};
constexpr REGInstruction dummyReg{0x58, 7, 8, false, true, false, 0x2, false, false, 9};
static_assert(dummyReg.getOpcode() == Opcodes::andnot);
static_assert(dummyReg.getPrimaryOpcode() == 0x58);
static_assert(dummyReg.getSecondaryOpcode() == 0x2);
struct MEMInstruction {
public:
    enum class AddressingMode : uint8_t {
        // MEMA
        AbsoluteOffset = 0b0000,
        // these are not real but for the sake of simplicity we are describing it this way
        Reserved0 = 0b0001,
        Reserved1 = 0b0010,
        Reserved2 = 0b0011,
        RegisterIndirect = 0b0100,
        IPWithDisplacement = 0b0101,
        Reserved3 = 0b0110,
        RegisterIndirectWithIndex = 0b0111,
        RegisterIndirectWithOffset = 0b1000,
        Reserved4 = 0b1001,
        Reserved5 = 0b1010,
        Reserved6 = 0b1011,
        AbsoluteDisplacement = 0b1100,
        RegisterIndirectWithDisplacement = 0b1101,
        IndexWithDisplacement = 0b1110,
        RegisterIndirectWithIndexAndDisplacement = 0b1111,
    };
    static constexpr bool valid(AddressingMode mode) noexcept {
        switch (mode) {
            case AddressingMode::AbsoluteOffset:
            case AddressingMode::RegisterIndirect:
            case AddressingMode::IPWithDisplacement:
            case AddressingMode::RegisterIndirectWithIndex:
            case AddressingMode::RegisterIndirectWithOffset:
            case AddressingMode::AbsoluteDisplacement:
            case AddressingMode::RegisterIndirectWithDisplacement:
            case AddressingMode::IndexWithDisplacement:
            case AddressingMode::RegisterIndirectWithIndexAndDisplacement:
                return true;
            default:
                return false;
        }
    }
    static constexpr bool usesOptionalDisplacement(AddressingMode mode) noexcept {
        switch (mode) {
            case AddressingMode::IPWithDisplacement:
            case AddressingMode::AbsoluteDisplacement:
            case AddressingMode::RegisterIndirectWithDisplacement:
            case AddressingMode::IndexWithDisplacement:
            case AddressingMode::RegisterIndirectWithIndexAndDisplacement:
                return true;
            default:
                return false;
        }
    }
    constexpr MEMInstruction(Ordinal base) : raw_(base) {}
    constexpr MEMInstruction(const Register& backingStore) : MEMInstruction(static_cast<Ordinal>(backingStore)) { }
    [[nodiscard]] constexpr Ordinal getValue() const noexcept { return raw_; }
    [[nodiscard]] constexpr Opcodes getOpcode() const noexcept { return static_cast<Opcodes>(generic.opcode); }
    [[nodiscard]] constexpr bool isMEMA() const noexcept { return discriminant.kind == 0; }
    [[nodiscard]] constexpr bool isMEMB() const noexcept { return discriminant.kind == 1; }
    [[nodiscard]] constexpr ByteOrdinal getABase() const noexcept { return generic.abase; }
    [[nodiscard]] constexpr ByteOrdinal getSrcDest() const noexcept { return generic.srcDest; }
    [[nodiscard]] constexpr AddressingMode getAddressingMode() const noexcept {
        return static_cast<AddressingMode>(memb.mode & (isMEMA() ? 0b1100 : 0b1111));
    }
    [[nodiscard]] constexpr bool validAddressingMode() const noexcept {
        return valid(getAddressingMode());
    }
    [[nodiscard]] constexpr Ordinal getOffset() const noexcept { return memaOffset; }
    [[nodiscard]] constexpr ByteOrdinal getIndex() const noexcept { return memb.index; }
    [[nodiscard]] constexpr ByteOrdinal getScale() const noexcept { return memb.scale; }
    [[nodiscard]] constexpr bool usesOptionalDisplacement() const noexcept { return usesOptionalDisplacement(getAddressingMode()); }
    template<typename T>
    [[nodiscard]] constexpr T scaleValue(T input) const noexcept {
        return input << memb.scale;
    }
private:
    union {
        Ordinal raw_;
        Ordinal memaOffset : 12;
        struct {
            Ordinal dc : 12;
            Ordinal kind : 1;
            Ordinal rest : 19;
        } discriminant;
        struct {
            Ordinal dc : 14;
            BackingUnionType abase : 5;
            BackingUnionType srcDest : 5;
            Ordinal opcode : 8;
        } generic;
        struct {
            Ordinal index : 5;
            Ordinal dc : 2;
            Ordinal scale : 3;
            Ordinal mode : 4;
            Ordinal abase : 5;
            Ordinal srcDest : 5;
            Ordinal opcode : 8;
        } memb;
    };
};
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
    [[nodiscard]] Register& get(ByteOrdinal index) noexcept { return registers_[index & 0b11111]; }
    [[nodiscard]] const Register& get(ByteOrdinal index) const noexcept { return registers_[index & 0b11111]; }
    template<typename T>
    void setValue(ByteOrdinal index, T value) noexcept {
        get(index).setValue(value, TreatAs<T>{});
    }
    template<typename T>
    [[nodiscard]] T getValue(ByteOrdinal index) const noexcept {
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
/**
 * @brief the base bitwise operation itself
 */
enum class BinaryBitwiseOperation : uint8_t {
    And,
    Or,
    Xor,
};
constexpr bool isValid(BinaryBitwiseOperation op) noexcept {
    using K = BinaryBitwiseOperation;
    switch (op) {
        case K::And:
        case K::Or:
        case K::Xor:
            return true;
        default:
            return false;
    }
}
enum class BitwiseMicrocodeArgumentFlags : uint8_t {
    Passthrough = 0b0000,
    Invert      = 0b0001,
    BitPosition = 0b0010,
    Increment   = 0b0100,
    Decrement   = 0b1000,
    BitPositionThenInvert = BitPosition | Invert,
};
constexpr BitwiseMicrocodeArgumentFlags operator|(BitwiseMicrocodeArgumentFlags l, BitwiseMicrocodeArgumentFlags r) noexcept {
    using K = std::underlying_type_t<BitwiseMicrocodeArgumentFlags>;
    return BitwiseMicrocodeArgumentFlags(static_cast<K>(l) | static_cast<K>(r));
}
constexpr BitwiseMicrocodeArgumentFlags operator&(BitwiseMicrocodeArgumentFlags l, BitwiseMicrocodeArgumentFlags r) noexcept {
    using K = std::underlying_type_t<BitwiseMicrocodeArgumentFlags>;
    return BitwiseMicrocodeArgumentFlags(static_cast<K>(l) & static_cast<K>(r));
}
constexpr BitwiseMicrocodeArgumentFlags operator^(BitwiseMicrocodeArgumentFlags l, BitwiseMicrocodeArgumentFlags r) noexcept {
    using K = std::underlying_type_t<BitwiseMicrocodeArgumentFlags>;
    return BitwiseMicrocodeArgumentFlags(static_cast<K>(l) ^ static_cast<K>(r));
}
template<BitwiseMicrocodeArgumentFlags field>
constexpr bool fieldSet(BitwiseMicrocodeArgumentFlags flags) noexcept {
    using K = std::underlying_type_t<decltype(field)>;
    return (static_cast<K>(flags) & static_cast<K>(field)) != 0;
}
constexpr bool invertField(BitwiseMicrocodeArgumentFlags flags) noexcept {
    return fieldSet<BitwiseMicrocodeArgumentFlags::Invert>(flags);
}
static_assert(invertField(BitwiseMicrocodeArgumentFlags::Invert));
static_assert(invertField(BitwiseMicrocodeArgumentFlags::Invert | BitwiseMicrocodeArgumentFlags::BitPosition));
static_assert(invertField(BitwiseMicrocodeArgumentFlags::Invert | BitwiseMicrocodeArgumentFlags::BitPosition | BitwiseMicrocodeArgumentFlags::Increment));
static_assert(invertField(BitwiseMicrocodeArgumentFlags::Invert | BitwiseMicrocodeArgumentFlags::Increment));
static_assert(!invertField(BitwiseMicrocodeArgumentFlags::Increment));
constexpr bool computeBitPositionOnField(BitwiseMicrocodeArgumentFlags flags) noexcept {
    return fieldSet<BitwiseMicrocodeArgumentFlags::BitPosition>(flags);
}
static_assert(!computeBitPositionOnField(BitwiseMicrocodeArgumentFlags::Invert));
static_assert(computeBitPositionOnField(BitwiseMicrocodeArgumentFlags::Invert | BitwiseMicrocodeArgumentFlags::BitPosition));
static_assert(computeBitPositionOnField(BitwiseMicrocodeArgumentFlags::Invert | BitwiseMicrocodeArgumentFlags::BitPosition | BitwiseMicrocodeArgumentFlags::Increment));
static_assert(!computeBitPositionOnField(BitwiseMicrocodeArgumentFlags::Invert | BitwiseMicrocodeArgumentFlags::Increment));
static_assert(!computeBitPositionOnField(BitwiseMicrocodeArgumentFlags::Increment));
constexpr bool performIncrement(BitwiseMicrocodeArgumentFlags flags) noexcept {
    return fieldSet<BitwiseMicrocodeArgumentFlags::Increment>(flags);
}
constexpr bool performDecrement(BitwiseMicrocodeArgumentFlags flags) noexcept {
    return fieldSet<BitwiseMicrocodeArgumentFlags::Decrement>(flags);
}
struct BinaryOperationMicrocodeFlags {
    consteval BinaryOperationMicrocodeFlags(BinaryBitwiseOperation op, 
            BitwiseMicrocodeArgumentFlags dest,
            BitwiseMicrocodeArgumentFlags src1,
            BitwiseMicrocodeArgumentFlags src2) : op_(op), dest_(dest), src1_(src1), src2_(src2)
    {

    } 
#define X(prefix, function) \
    [[nodiscard]] consteval bool prefix ## Destination() const noexcept { return function (dest_); } \
    [[nodiscard]] consteval bool prefix ## Src1() const noexcept { return function (src1_); } \
    [[nodiscard]] consteval bool prefix ## Src2() const noexcept { return function (src2_); }
    X(invert, invertField);
    X(computeBitPositionOn, computeBitPositionOnField);
    X(increment, performIncrement);
    X(decrement, performDecrement);
#undef X
    [[nodiscard]] consteval auto getOperation() const noexcept { return op_; }
    [[nodiscard]] consteval auto valid() const noexcept { return isValid(op_); }
    BinaryBitwiseOperation op_;
    BitwiseMicrocodeArgumentFlags dest_;
    BitwiseMicrocodeArgumentFlags src1_; 
    BitwiseMicrocodeArgumentFlags src2_;
};
template<BitwiseMicrocodeArgumentFlags dest = BitwiseMicrocodeArgumentFlags::Passthrough,
    BitwiseMicrocodeArgumentFlags src1 = BitwiseMicrocodeArgumentFlags::Passthrough,
    BitwiseMicrocodeArgumentFlags src2 = BitwiseMicrocodeArgumentFlags::Passthrough>
constexpr BinaryOperationMicrocodeFlags GenericOrOperation { BinaryBitwiseOperation::Or, dest, src1, src2 };
template<BitwiseMicrocodeArgumentFlags dest = BitwiseMicrocodeArgumentFlags::Passthrough,
    BitwiseMicrocodeArgumentFlags src1 = BitwiseMicrocodeArgumentFlags::Passthrough,
    BitwiseMicrocodeArgumentFlags src2 = BitwiseMicrocodeArgumentFlags::Passthrough>
constexpr BinaryOperationMicrocodeFlags GenericAndOperation { BinaryBitwiseOperation::And, dest, src1, src2 };
template<BitwiseMicrocodeArgumentFlags dest = BitwiseMicrocodeArgumentFlags::Passthrough,
    BitwiseMicrocodeArgumentFlags src1 = BitwiseMicrocodeArgumentFlags::Passthrough,
    BitwiseMicrocodeArgumentFlags src2 = BitwiseMicrocodeArgumentFlags::Passthrough>
constexpr BinaryOperationMicrocodeFlags GenericXorOperation { BinaryBitwiseOperation::Xor, dest, src1, src2 };

constexpr auto OrOperation = GenericOrOperation<>;
constexpr auto AndOperation = GenericAndOperation<>;
constexpr auto XorOperation = GenericXorOperation<>;
constexpr auto NorOperation = GenericOrOperation<BitwiseMicrocodeArgumentFlags::Invert>;
constexpr auto NandOperation = GenericAndOperation<BitwiseMicrocodeArgumentFlags::Invert>;
constexpr auto XnorOperation = GenericXorOperation<BitwiseMicrocodeArgumentFlags::Invert>;


class Core {
public:
    static constexpr Ordinal SALIGN = getSALIGNParameter();
    static constexpr Ordinal C = (SALIGN * 16) - 1;
    static constexpr Ordinal NotC = ~C;
    static constexpr uint8_t NumberOfLocalRegisterFrames = 4;

    struct LocalRegisterSet {
    public:
        using SaveRegistersFunction = std::function<void(const RegisterFrame&, Address)>;
        using RestoreRegistersFunction = std::function<void(RegisterFrame&, Address)>;
        [[nodiscard]] constexpr bool valid() const noexcept { return _valid; }
        [[nodiscard]] constexpr auto getAddress() const noexcept { return _targetFramePointer; }
        [[nodiscard]] RegisterFrame& getUnderlyingFrame() noexcept { return _theFrame; }
        [[nodiscard]] const RegisterFrame& getUnderlyingFrame() const noexcept { return _theFrame; }
        void relinquishOwnership() noexcept;
        void relinquishOwnership(SaveRegistersFunction saveRegisters);
        void takeOwnership(Address newFP, SaveRegistersFunction saveRegisters);
        void restoreOwnership(Address newFP, SaveRegistersFunction saveRegisters, RestoreRegistersFunction restoreRegisters);
        void clear() noexcept;
        /**
         * Makes sure that this set has an up to date frame pointer
         * @param fp The frame pointer to update the targetFramePointer field to
         */
        void synchronizeOwnership(Ordinal fp) noexcept;
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
    OptionalFaultRecord doDispatchInternal() noexcept;
    void lockBus() noexcept;
    void unlockBus() noexcept;
    /// @todo insert iac dispatch here
    /// @todo insert routines for getting registers and such
    [[nodiscard]] auto& currentLocalRegisterSet() noexcept { return frames_[localRegisterFrameIndex_]; }
    [[nodiscard]] const auto& currentLocalRegisterSet() const noexcept { return frames_[localRegisterFrameIndex_]; }
    [[nodiscard]] auto& getLocals() noexcept { return currentLocalRegisterSet().getUnderlyingFrame(); }
    [[nodiscard]] const auto& getLocals() const noexcept { return currentLocalRegisterSet().getUnderlyingFrame(); }
    template<typename T>
    requires MustBeRegisterType<T>
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
    requires MustBeRegisterType<T>
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
    [[nodiscard]] Register& getGPR(ByteOrdinal index) noexcept;
    [[nodiscard]] const Register& getGPR(ByteOrdinal index) const noexcept;
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
    void setGPR(ByteOrdinal index, Ordinal value, TreatAsOrdinal) noexcept;
    void setGPR(ByteOrdinal index, ByteOrdinal offset, Ordinal value, TreatAsOrdinal) noexcept;
    void setGPR(ByteOrdinal index, Integer value, TreatAsInteger) noexcept;
    template<typename T>
    void setGPRValue(ByteOrdinal index, T value) noexcept {
        setGPR(index, value, TreatAs<T>{});
    }
    template<typename T>
    void setGPRValue(ByteOrdinal index, ByteOrdinal offset, T value) noexcept {
        setGPR(index, offset, value, TreatAs<T>{});
    }
    [[nodiscard]] const Register& getSrc1Register(const REGInstruction&) const noexcept;
    [[nodiscard]] const Register& getSrc2Register(const REGInstruction&) const noexcept;
    [[nodiscard]] Register& getSrc1Register(const COBRInstruction&) noexcept;
    [[nodiscard]] const Register& getSrc2Register(const COBRInstruction&) const noexcept ;
    [[nodiscard]] Ordinal unpackSrc1(const REGInstruction&, ByteOrdinal offset, TreatAsOrdinal) noexcept;
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
    Ordinal computeAddress(const MEMInstruction&) noexcept;
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
    void syncf() ;
    OptionalFaultRecord mark() ;
    OptionalFaultRecord fmark() ;
    void synld(Register& dest, Ordinal src) ;
    void synmov(const Register& dest, Ordinal src) ;
    void synmovl(const Register& dest, Ordinal src) ;
    void synmovq(const Register& dest, Ordinal src) ;
    void scanbit(Register& dest, Ordinal src) noexcept;
    void spanbit(Register& dest, Ordinal src) noexcept;
    void branch(Integer displacement) noexcept;
    void branchConditional(bool condition, Integer displacement) noexcept;
    void scanbyte(Ordinal src1, Ordinal src2) ;
    OptionalFaultRecord emul(const REGInstruction& inst, LongRegister& dest, Ordinal src1, Ordinal src2) ;
    OptionalFaultRecord ediv(const REGInstruction& inst, LongRegister& dest, Ordinal src1, const LongRegister& src2) ;
    void arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) ;
    void advanceCOBRDisplacement(Integer displacement) noexcept;
    void bbc();
    void bbc(Ordinal bitpos, Ordinal against, int16_t displacement);
    void bbc(uint8_t bitpos, const Register& against, int16_t displacement);
    void bbc(const Register& bitpos, const Register& against, int16_t displacement);
    void bbs();
    void bbs(Ordinal bitpos, Ordinal against, int16_t displacement);
    void bbs(uint8_t bitpos, const Register& against, int16_t displacement);
    void bbs(const Register& bitpos, const Register& against, int16_t displacement);
    OptionalFaultRecord testGeneric();
    void cmpobGeneric();
    void cmpibGeneric();
    template<typename Q>
    requires Is960Comparable<Q>
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
    requires Is960Comparable<Q>
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
    void cmpobGeneric(uint8_t mask, Ordinal src1, Ordinal src2, int16_t displacement) noexcept;
    void cmpibGeneric(uint8_t mask, Integer src1, Integer src2, int16_t displacement) noexcept;
    void flushreg() ;
    void balx(ByteOrdinal linkRegister, Ordinal branchTo) ;
    OptionalFaultRecord calls(Ordinal value) ;
    OptionalFaultRecord calls();
    OptionalFaultRecord ldl(const MEMInstruction&, Address address, LongRegister& destination) ;
    OptionalFaultRecord ldq(const MEMInstruction&, Address address, QuadRegister& destination) ;
    OptionalFaultRecord ldt(const MEMInstruction&, Address address, TripleRegister& destination) ;
    OptionalFaultRecord stq(const MEMInstruction&, Address address, const QuadRegister& src) ;
    OptionalFaultRecord stt(const MEMInstruction&, Address address, const TripleRegister& src) ;
    OptionalFaultRecord stl(const MEMInstruction&, Address address, const LongRegister& src) ;
    OptionalFaultRecord ret() ;
    OptionalFaultRecord call(Integer displacement) ;
    OptionalFaultRecord call();
    OptionalFaultRecord callx(Address effectiveAddress) ;
    OptionalFaultRecord callx();
private:
    void enterCall(Ordinal fp);
    void leaveCall();
    void restoreStandardFrame() noexcept;
private:
    void performConditionalSubtract(Register& dest, Integer src1, Integer src2, TreatAsInteger) ;
    void performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) ;
    void performConditionalAdd(Register& dest, Integer src1, Integer src2, TreatAsInteger) ;
    void performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) ;
    void performSelect(Register& dest, Ordinal src1, Ordinal src2) ;
    template<typename T>
    void performConditionalAdd() {
        performConditionalAdd(getGPR(_regInstruction.getSrcDest()),
                              static_cast<T>(getSrc1Register(_regInstruction)),
                              static_cast<T>(getSrc2Register(_regInstruction)),
                              TreatAs<T>{});
    }
    template<typename T>
    void performConditionalSubtract() {
        performConditionalSubtract(getGPR(_regInstruction.getSrcDest()),
                              static_cast<T>(getSrc1Register(_regInstruction)),
                              static_cast<T>(getSrc2Register(_regInstruction)),
                              TreatAs<T>{});
    }
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
    X(LongOrdinal);
    X(QuadOrdinal);
#undef X
    template<typename T>
    [[nodiscard]] T load(Address address) const noexcept {
        return load(address, TreatAs<T>{});
    }
    template<typename T>
    void store(Address address, T value) noexcept {
        store(address, value, TreatAs<T>{});
    }
private:
    template<BinaryOperationMicrocodeFlags flags>
    inline void microcodedBitwiseOperation(Register& destination, Ordinal src1, Ordinal src2) {
        static_assert(flags.valid(), "Illegal bitwise microcode operation kind!");
        Ordinal s1 = src1;
        Ordinal s2 = src2;
        if constexpr (flags.computeBitPositionOnSrc1()) {
            s1 = computeBitPosition(s1);
        }
        if constexpr (flags.invertSrc1()) {
            s1 = ~s1;
        }
        if constexpr (flags.incrementSrc1()) {
            ++s1;
        }
        if constexpr (flags.decrementSrc1()) {
            --s1;
        }
        if constexpr (flags.computeBitPositionOnSrc2()) {
            s2 = computeBitPosition(s2);
        }
        if constexpr (flags.invertSrc2()) {
            s2 = ~s2;
        }
        if constexpr (flags.incrementSrc2()) {
            ++s2;
        }
        if constexpr (flags.decrementSrc2()) {
            --s2;
        }
        Ordinal result = 0;
        switch (flags.getOperation()) {
            case BinaryBitwiseOperation::And:
                result = s2 & s1;
                break;
            case BinaryBitwiseOperation::Or:
                result = s2 | s1;
                break;
            case BinaryBitwiseOperation::Xor:
                result = s2 ^ s1;
                break;
        }
        if constexpr (flags.invertDestination()) {
            result = ~result;
        }
        if constexpr (flags.incrementDestination()) {
            ++result;
        }
        if constexpr (flags.decrementDestination()) {
            --result;
        }
        destination.setValue<Ordinal>(result);
    }
    template<typename Q>
    requires MustBeOrdinalOrInteger<Q>
    void add(Register& destination, Q src1, Q src2, TreatAs<Q>) noexcept {
        auto result = ::addOperation<Q>(src2, src1);
        DEBUG_LOG_LEVEL(1) {
            std::cout << __PRETTY_FUNCTION__ << ": (+ 0x"
                      << std::hex << src2
                      << " 0x" << std::hex << src1
                      << ") -> 0x" << std::hex << result
                      << std::endl;
        }
        destination.setValue<Q>(result);
    }
    template<typename Q>
    requires MustBeOrdinalOrInteger<Q>
    void sub(Register& destination, Q src1, Q src2, TreatAs<Q>) noexcept {
        auto result = ::subOperation<Q>(src2, src1);
        DEBUG_LOG_LEVEL(1) {
            std::cout << __PRETTY_FUNCTION__ << ": (- 0x"
                      << std::hex << src2
                      << " 0x" << std::hex << src1
                      << ") -> 0x" << std::hex << result
                      << std::endl;
        }
        destination.setValue<Q>(result);
    }
    template<typename Q>
    requires MustBeOrdinalOrInteger<Q>
    void mult(Register& destination, Q src1, Q src2, TreatAs<Q>) noexcept {
        destination.setValue<Q>(::multiplyOperation<Q>(src2, src1));
    }
    void setbit(Register& destination, Ordinal src1, Ordinal src2) noexcept;
    void nor(Register& destination, Ordinal src1, Ordinal src2) noexcept;
    void nand(Register& destination, Ordinal src1, Ordinal src2) noexcept;
    void xnor(Register& destination, Ordinal src1, Ordinal src2) noexcept;
    void notbit(Register& destination, Ordinal src1, Ordinal src2) noexcept;
    void ornot(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    void notor(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    static void notOperation(Register& destination, Ordinal src) noexcept;

    void andnot(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    void notand(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    void clrbit(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    OptionalFaultRecord modi(Register& dest, Integer src1, Integer src2);
    void alterbit(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    void addc(Register& dest, Ordinal src1, Ordinal src2);
    void subc(Register& dest, Ordinal src1, Ordinal src2);
    template<typename Q>
    requires MustBeOrdinalOrInteger<Q>
    OptionalFaultRecord  checkForZeroDivideFault(Q value) {
        if (value == 0) {
            return zeroDivideFault();
        } else {
            return std::nullopt;
        }
    }
    template<typename Q>
    requires MustBeOrdinalOrInteger<Q>
    OptionalFaultRecord remainderOperation(Register& dest, Q src1, Q src2) {
        auto result = checkForZeroDivideFault(src1);
        if (result) {
            return result;
        } else {
            // taken from the i960Sx manual
            dest.setValue<Q>(src2 % src1);
            return std::nullopt;
        }
    }
    OptionalFaultRecord remi(Register& dest, Integer src1, Integer src2);
    OptionalFaultRecord remo(Register& dest, Ordinal src1, Ordinal src2);
    template<typename Q>
    requires MustBeOrdinalOrInteger<Q>
    OptionalFaultRecord divideOperation(Register& dest, Q src1, Q src2) {
        auto result = checkForZeroDivideFault(src1);
        if (result) {
            return result;
        } else {
            dest.setValue<Q>(src2 / src1);
            return std::nullopt;
        }
    }
    OptionalFaultRecord divi(Register& dest, Integer src1, Integer src2);
    OptionalFaultRecord divo(Register& dest, Ordinal src1, Ordinal src2);
    void atadd(Register& dest, Ordinal src1, Ordinal src2);
    void atmod(Register& dest, Ordinal src1, Ordinal src2);
    void cmpo(Ordinal src1, Ordinal src2) noexcept;
    void cmpinco(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    void cmpdeco(Register& dest, Ordinal src1, Ordinal src2) noexcept;
    void cmpi(Integer src1, Integer src2) noexcept;
    void cmpinci(Register& dest, Integer src1, Integer src2) noexcept;
    void cmpdeci(Register& dest, Integer src1, Integer src2) noexcept;
    template<typename Q>
    requires MustBeOrdinalOrInteger<Q>
    void concmpGeneric(Q src1, Q src2) noexcept {
        if ((ac_.getConditionCode() & 0b100) == 0) {
            ac_.arith.conditionCode = src1 <= src2 ? 0b010 : 0b001;
        }
    }
    void concmpo(Ordinal src1, Ordinal src2) noexcept;
    void concmpi(Integer src1, Integer src2) noexcept;
private:
    bool performSelfTest() noexcept;
    void assertFailureState() noexcept;
    void deassertFailureState() noexcept;
    template<Ordinal faultCode, bool saveReturnAddress>
    [[nodiscard]] FaultRecord constructFault() const noexcept {
        return FaultRecord{static_cast<Ordinal>(pc_),
                           static_cast<Ordinal>(ac_),
                           faultCode,
                           static_cast<Ordinal>(ip_),
                           saveReturnAddress
        };
    }
    [[nodiscard]] FaultRecord zeroDivideFault() const noexcept { return constructFault<ZeroDivideFault, true>(); }
    [[nodiscard]] OptionalFaultRecord integerOverflowFault()  noexcept {
        if (ac_.arith.integerOverflowMask == 0) {
            return constructFault<IntegerOverflowFault, true>();
            // saved ip will be the next instruction
        } else {
            ac_.arith.integerOverflowFlag = 1;
            return std::nullopt;
        }
    }

#define X(prefixName, kind, save) [[nodiscard]] FaultRecord prefixName ## Fault () const noexcept { return constructFault < kind ## Fault , save > () ; }
    X(constraintRange, ConstraintRange, false);
    X(invalidSS, InvalidSS, false);
    X(unimplemented, Unimplemented, false);
    X(invalidOperand, InvalidOperand, false);
    X(markTrace, MarkTrace, false);
    X(invalidOpcode, InvalidOpcode, false);
    X(eventNotice, EventNotice, true);
    X(protectionLength, SegmentLength, false);
    X(typeMismatch, TypeMismatch, false);
    X(typeContents, TypeContents, false); // @todo figure this out
#undef X
    [[nodiscard]]
    FaultRecord
    invalidDescriptorFault(SegmentSelector selector) noexcept {
        FaultRecord record = constructFault< InvalidDescriptorFault, false> ();
        record.faultData[1] = (selector & ~0b11111);
        setGPR(RIPIndex, static_cast<Ordinal>(ip_), TreatAsOrdinal{});
        return record;
    }
    [[nodiscard]] OptionalFaultRecord floatingInvalidOperationFault() noexcept {
        if (ac_.arith.floatingInvalidOpMask == 0) {
            return constructFault<FloatingPointInvalidOperationFault, true>();
        } else {
            ac_.arith.floatingInvalidOpFlag = 1;
            return std::nullopt;
        }
    }
    [[nodiscard]] OptionalFaultRecord floatingOverflowFault() noexcept {
        if (ac_.arith.floatingOverflowMask == 0)  {
            return constructFault<FloatingPointOverflowFault, true>();
        } else {
            ac_.arith.floatingOverflowFlag = 1;
            return std::nullopt;
        }
    }
    [[nodiscard]] OptionalFaultRecord floatingZeroDivideOperationFault() noexcept {
        if (ac_.arith.floatingZeroDivideMask == 0) {
            return constructFault<FloatingPointZeroDivideOperationFault, true>();
        } else {
            ac_.arith.floatingZeroDivideFlag = 1;
            return std::nullopt;
        }
    }
    [[nodiscard]] OptionalFaultRecord floatingUnderflowFault() noexcept {
        if (ac_.arith.floatingUnderflowMask == 0) {
            return constructFault<FloatingPointUnderflowFault, true>();
        } else {
            ac_.arith.floatingUnderflowFlag = 1;
            return std::nullopt;
        }
    }
    [[nodiscard]] OptionalFaultRecord floatingInexactFault() noexcept {
        if (ac_.arith.floatingZeroDivideMask == 0) {
            return constructFault<FloatingPointZeroDivideOperationFault, true>();
        } else {
            ac_.arith.floatingZeroDivideFlag = 1;
            return std::nullopt;
        }
    }
    [[nodiscard]] OptionalFaultRecord floatingReservedEncodingFault() const noexcept {
        if (ac_.arith.floatingPointNormalizingMode == 0)  {
            return constructFault<FloatingPointReservedEncodingFault, true>();
        } else {
            return std::nullopt;
        }
    }
    void generateFault(const FaultRecord& record) ;
    void addi(Register& dest, Integer src1, Integer src2);
    void addo(Register& dest, Ordinal src1, Ordinal src2);
    void saveReturnAddress(ByteOrdinal registerIndex) noexcept;
    void saveReturnAddress(Register& linkRegister) noexcept;
    void setupNewFrameInternals(Ordinal fp, Ordinal temp) noexcept;
    [[nodiscard]] Ordinal getStackPointer() const noexcept;
    [[nodiscard]] Ordinal getNextFrameBase() const noexcept;
    void setStackPointer(Ordinal value, TreatAsOrdinal) noexcept;
    /**
     * @brief Advance ip by instruction length and then prevent further
     * advancement until the next cycle!
     */
    void nextInstruction() noexcept;
    void setIP(Ordinal value) noexcept;
    void subo(Register& dest, Ordinal src1, Ordinal src2);
    OptionalFaultRecord subi(Register& dest, Integer src1, Integer src2);
    OptionalFaultRecord faultGeneric();
    void balx(Register& linkRegister, Address ordinal);
    void balx();
private:
    OptionalFaultRecord modpc(Register& dest, Ordinal src1o, Ordinal src2o);
    static void modxc(Register& control, Register& dest, Ordinal src1, Ordinal src2);
    static void shlo(Register& srcDest, Ordinal src1, Ordinal src2) ;
    static void shli(Register& srcDest, Integer src1, Integer src2) ;
    static void rotate(Register& dest, Ordinal src1, Ordinal src2) ;
    void shri();
    static void shri(Register& dest, Integer src1, Integer src2) ;
    static void shro(Register& dest, Ordinal src1, Ordinal src2) ;
    void mulo(Register& dest, Ordinal src1, Ordinal src2) ;
    void muli(Register& dest, Integer src1, Integer src2) ;
    static void modify(Register& dest, Ordinal src1, Ordinal src2) ;
    static void extract(Register& dest, Ordinal src1, Ordinal src2) ;
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
    [[nodiscard]] FaultTableEntry getFaultEntry(uint8_t index) const noexcept;
    void faultCallGeneric (const FaultRecord& record, Address destination, Address stackPointer) noexcept;
    void localProcedureEntry_FaultCall (const FaultRecord& record, Address destination) noexcept;
    void procedureTableEntry_FaultCall(const FaultRecord& record, const FaultTableEntry& entry) noexcept;
    void supervisorProcedureTableEntry_FaultCall(const FaultRecord& record, Address procedureAddress, Address tableBaseAddress) noexcept;
private:
    template<uint8_t offset>
    requires ((offset <= 172) && ((offset & 0b11) == 0)) // the PRCB is 176 bytes in size and is aligned to four byte boundaries
    [[nodiscard]] inline Ordinal getFromPRCB() const noexcept { return load<Ordinal>(prcbAddress_ + offset); }
    [[nodiscard]] inline Ordinal getProcessorControls() const noexcept { return getFromPRCB<4>(); }
    [[nodiscard]] inline Ordinal getCurrentProcessSegmentSelector() const noexcept { return getFromPRCB<12>(); }
    [[nodiscard]] inline Ordinal getDispatchPortSegmentSelector() const noexcept { return getFromPRCB<16>(); }
    [[nodiscard]] inline Address getInterruptTablePointer() const noexcept { return getFromPRCB<20>(); }
    [[nodiscard]] inline Address getInterruptStackAddress() const noexcept { return getFromPRCB<24>(); }
    [[nodiscard]] inline Address getSystemProcedureTableSegmentSelector() const noexcept { return getFromPRCB<36>(); }
    [[nodiscard]] inline Address getFaultTableBaseAddress() const noexcept { return getFromPRCB<40>(); }
private: // system address table / system procedure table
    [[nodiscard]] SegmentDescriptor loadSegmentDescriptor(SegmentSelector offset) const noexcept;
public:
    void checksumFail();
    void selfTestFailure();
private:
    void badFault(const FaultRecord& record);
private: // numerics extensions
    void dmovt(Register& dest, Ordinal src) noexcept;
    OptionalFaultRecord dsubc();
    OptionalFaultRecord daddc();
private: // protected extensions instructions
    void signal(SegmentSelector sel) ;
    void wait(SegmentSelector src) ;
    void sendserv(SegmentSelector src) ;
    void send(SegmentSelector target, Ordinal src1, SegmentSelector src2);
    void schedprcs(SegmentSelector src) ;
    void saveprcs() ;
    void resumprcs(SegmentSelector ss) ;
    void receive(SegmentSelector src, Register& dest) ;
    void ldtime(LongRegister& dest) ;
    void ldphy(Address address, Register& dest) ;
    void inspacc(Ordinal src, Register& dest) ;
    void condwait(SegmentSelector src) ;
    void condrec(SegmentSelector src, Register& dest) ;
    void cmpstr(Ordinal src1, Ordinal src2, Ordinal len) noexcept;
    void movstr(Ordinal destAddress, Ordinal srcAddress, Ordinal len) noexcept;
    void movqstr(Ordinal destAddress, Ordinal srcAddress, Ordinal len) noexcept;
    void fill(Ordinal dest, Ordinal value, Ordinal len) noexcept;
private: // interrupt related
    [[nodiscard]] Address getInterruptTableBaseAddress() const;
    void postInterrupt(InterruptVector vector);
    inline Ordinal getInterruptPendingPriorities() const { return load<Ordinal>(getInterruptTablePointer()); }
    void setInterruptPendingPriorities(Ordinal value) { store<Ordinal>(getInterruptTablePointer(), value); }
    [[nodiscard]] inline Ordinal getPendingInterruptWord(uint8_t index) const {
        return load<Ordinal>(getInterruptTableBaseAddress() + 4 + (sizeof(Ordinal) * (index & 0b111)));
    }
    [[nodiscard]] inline Ordinal getPendingInterruptWord(InterruptVector vector) const {
        return getPendingInterruptWord(computeInterruptWordIndex(vector));
    }
    void setPendingInterruptWord(uint8_t index, Ordinal value) {
        store<Ordinal>(getInterruptTableBaseAddress() + 4 + (sizeof(Ordinal) * (index & 0b111)), value);
    }
    void setPendingInterruptWord(InterruptVector vector, Ordinal value) {
        setPendingInterruptWord(computeInterruptWordIndex(vector), value);
    }

    [[nodiscard]] Address getInterruptVectorAddress(uint8_t vector) const;
    [[nodiscard]] inline Address getInterruptVectorAddress(InterruptVector vector) const { return getInterruptVectorAddress(static_cast<uint8_t>(vector)); }
    void receiveInterrupt(InterruptVector vector);
    void setPendingPriorityBit(uint8_t priority);
    void setPendingPriorityBit(InterruptVector vector) { setPendingPriorityBit(computeInterruptPriority(vector)); }
    void clearPendingPriorityBit(uint8_t priority);
    void clearPendingPriorityBit(InterruptVector vector) { clearPendingPriorityBit(computeInterruptPriority(vector)); }
    [[nodiscard]] bool getPendingPriorityBit(uint8_t priority) const;
    [[nodiscard]] inline bool getPendingPriorityBit(InterruptVector vector) const { return getPendingPriorityBit(computeInterruptPriority(vector)); }
    [[nodiscard]] bool getPendingInterruptBit(InterruptVector vector) const;
    void setPendingInterruptBit(InterruptVector vector);
    void clearPendingInterruptBit(InterruptVector vector);
    void obtainedPendingVector(InterruptVector vector);
    [[nodiscard]] bool pendingInterruptPriorityClear(InterruptVector vector) const;
    [[nodiscard]] ByteOrdinal getPendingInterruptBitsForPriority(uint8_t priority) const;
    [[nodiscard]] ByteOrdinal getHighestPostedInterruptVectorForPriority(uint8_t priority) const;
    [[nodiscard]] InterruptVector highestPostedInterruptVector() const;
    [[nodiscard]] InterruptVector serviceNextInterrupt();
    void serviceInterrupt(InterruptVector vector);
    void checkForPendingInterrupts();
private:
    void localReturn();
    void faultReturn();
    void interruptReturn();
    void supervisorReturn(bool traceModeSetting);
private:
    OptionalFaultRecord faultOnOverflow(Register& dest);
    [[nodiscard]] Ordinal getFramePointerAddress() const { return getGPRValue<Ordinal>(FPIndex); }
    [[nodiscard]] Ordinal getRIPContents() const { return getGPRValue<Ordinal>(RIPIndex); }
    void restoreRIPToIP();
private:
    Ordinal restorePCFromStack(Ordinal fp);
    Ordinal restoreACFromStack(Ordinal fp);
private:
    void saveRegisterFrame(const RegisterFrame& theFrame, Address baseAddress);
    void restoreRegisterFrame(RegisterFrame& theFrame, Address baseAddress);
    LocalRegisterSet& getNextPack() noexcept;
    LocalRegisterSet& getPreviousPack() noexcept;
    [[nodiscard]] inline LocalRegisterSet& getCurrentPack() noexcept { return frames_[localRegisterFrameIndex_]; }
    void boot0(Address sat, Address pcb, Address startIP) noexcept;
private:
    void bx();
    void bx(Address effectiveAddress);
    void bal();
    void bal(Integer displacement);
    void b();

private: // mmu
    template<uint8_t offset>
    requires (offset <= 232)
    [[nodiscard]] inline Ordinal loadFromProcessControlBlock() const noexcept {
        return load<Ordinal>(processControlBlockBaseAddress_.getValue<Ordinal>() + offset);
    }
    [[nodiscard]] Address translateToPhysicalAddress(Address virtualAddress) noexcept;
    [[nodiscard]] inline Register getProcessControls() const noexcept { return Register(loadFromProcessControlBlock<20>()); }
    [[nodiscard]] inline SegmentSelector getRegion0SegmentSelector() const noexcept { return loadFromProcessControlBlock<48>(); }
    [[nodiscard]] inline SegmentSelector getRegion1SegmentSelector() const noexcept { return loadFromProcessControlBlock<52>(); }
    [[nodiscard]] inline SegmentSelector getRegion2SegmentSelector() const noexcept { return loadFromProcessControlBlock<56>(); }
    [[nodiscard]] inline SegmentSelector getRegion3SegmentSelector() const noexcept { return getFromPRCB<32>(); }
    [[nodiscard]] inline Ordinal getProcessArithmeticControls() const noexcept { return getFromPRCB<60>();}
    [[nodiscard]] inline Ordinal getProcessTraceControls() const noexcept { return getFromPRCB<28>();}
    [[nodiscard]] inline Ordinal getProcessNextTimeSlice() const noexcept { return getFromPRCB<68>(); }
    [[nodiscard]] inline ByteOrdinal getProcessLock() const noexcept { return static_cast<ByteOrdinal>(loadFromProcessControlBlock<24>()); }
    [[nodiscard]] inline Ordinal getProcessNotice() const noexcept { return (loadFromProcessControlBlock<24>() >> 8); }
    [[nodiscard]] inline Ordinal getProcessResidualTimeSlice() const noexcept { return loadFromProcessControlBlock<16>(); }
    [[nodiscard]] inline LongOrdinal getProcessExecutionTime()  const noexcept { return load<LongOrdinal>(processControlBlockBaseAddress_.getValue<Address>() + 72); }
    [[nodiscard]] inline LongOrdinal getProcessQueueRecord()  const noexcept { return load<LongOrdinal>(processControlBlockBaseAddress_.getValue<Address>()); }
    [[nodiscard]] inline Ordinal getProcessReceiveMessage() const noexcept { return loadFromProcessControlBlock<8>(); }
    [[nodiscard]] inline SegmentSelector getDispatchPortSS() const noexcept { return loadFromProcessControlBlock<12>(); }
    [[nodiscard]] inline bool inVirtualMemoryMode() const noexcept { return getProcessControls().inVirtualAddressingMode(); }
    [[nodiscard]] inline ByteOrdinal getProcessPriority() const noexcept { return getProcessControls().processControls.priority; }
    [[nodiscard]] inline bool processIsBlocked() const noexcept { return getProcessControls().processControls.state == 0; }
    [[nodiscard]] inline bool processIsExecuting() const noexcept { return getProcessControls().processControls.state == 0; }
    [[nodiscard]] inline bool processIsReady() const noexcept { return getProcessControls().processControls.state == 0; }
    [[nodiscard]] inline bool processIsInterrupted() const noexcept { return getProcessControls().processControls.state == 0b01; }
    [[nodiscard]] inline bool processIsInUserMode() const noexcept { return getProcessControls().processControls.executionMode == 0; }
    [[nodiscard]] inline bool processIsInSupervisorMode() const noexcept { return getProcessControls().processControls.executionMode != 0; }
    void saveGlobalsAndFloatingPointRegsToPCB();
private:
    OptionalFaultRecord stib(Integer value, Address address);
    OptionalFaultRecord stis(Integer value, Address address);
    void ldib(Address address, Register& dest);
    void ldis(Address address, Register& dest);
private:

    OptionalFaultRecord movre(const REGInstruction& inst);
    OptionalFaultRecord movrl(const REGInstruction& inst);
    OptionalFaultRecord movr(const REGInstruction& inst);
    OptionalFaultRecord cpyrsre(const REGInstruction& inst);
    OptionalFaultRecord cpysre(const REGInstruction& inst);
    OptionalFaultRecord classr(const REGInstruction& inst);
    OptionalFaultRecord classrl(const REGInstruction& inst);
    OptionalFaultRecord cosr(const REGInstruction& inst);
    OptionalFaultRecord cosrl(const REGInstruction& inst);
    OptionalFaultRecord sinr(const REGInstruction& inst);
    OptionalFaultRecord sinrl(const REGInstruction& inst);
    OptionalFaultRecord tanr(const REGInstruction& inst);
    OptionalFaultRecord tanrl(const REGInstruction& inst);
    OptionalFaultRecord atanr(const REGInstruction& inst);
    OptionalFaultRecord atanrl(const REGInstruction& inst);
    OptionalFaultRecord sqrtr(const REGInstruction& inst);
    OptionalFaultRecord sqrtrl(const REGInstruction& inst);
    OptionalFaultRecord roundr(const REGInstruction& inst);
    OptionalFaultRecord roundrl(const REGInstruction& inst);
    OptionalFaultRecord scaler(const REGInstruction& inst);
    OptionalFaultRecord scalerl(const REGInstruction& inst);
    OptionalFaultRecord addr(const REGInstruction& inst);
    OptionalFaultRecord addrl(const REGInstruction& inst);
    OptionalFaultRecord subr(const REGInstruction& inst);
    OptionalFaultRecord subrl(const REGInstruction& inst);
    OptionalFaultRecord mulr(const REGInstruction& inst);
    OptionalFaultRecord mulrl(const REGInstruction& inst);
    OptionalFaultRecord divr(const REGInstruction& inst);
    OptionalFaultRecord divrl(const REGInstruction& inst);
    OptionalFaultRecord remr(const REGInstruction& inst);
    OptionalFaultRecord remrl(const REGInstruction& inst);
    OptionalFaultRecord cmpr(const REGInstruction& inst) ;
    OptionalFaultRecord cmprl(const REGInstruction& inst) ;
    OptionalFaultRecord cmpor(const REGInstruction& inst);
    OptionalFaultRecord cmporl(const REGInstruction& inst);
    OptionalFaultRecord cvtilr(const REGInstruction& inst);
    OptionalFaultRecord cvtir(const REGInstruction& inst);
    OptionalFaultRecord cvtri(const REGInstruction& inst);
    OptionalFaultRecord cvtril(const REGInstruction& inst);
    OptionalFaultRecord cvtzri(const REGInstruction& inst);
    OptionalFaultRecord cvtzril(const REGInstruction& inst);
    OptionalFaultRecord fpassignment(const REGInstruction& inst, FaultRecord& record, TreatAs<FaultRecord>);
    OptionalFaultRecord fpassignment(const REGInstruction& inst, ExtendedReal value, TreatAsExtendedReal);
    OptionalFaultRecord fpassignment(const REGInstruction& inst, Real value, TreatAsReal);
    OptionalFaultRecord fpassignment(const REGInstruction& inst, LongReal value, TreatAsLongReal);
    using MixedRealSourceArgument = VariantWithFaultRecord<Real, ExtendedReal>;
    using MixedLongRealSourceArgument = VariantWithFaultRecord<LongReal, ExtendedReal>;
    [[nodiscard]] MixedRealSourceArgument unpackSrc1(const REGInstruction& index, TreatAsReal) const;
    [[nodiscard]] MixedRealSourceArgument unpackSrc2(const REGInstruction& index, TreatAsReal) const;
    [[nodiscard]] MixedLongRealSourceArgument unpackSrc1(const REGInstruction& index, TreatAsLongReal) const;
    [[nodiscard]] MixedLongRealSourceArgument unpackSrc2(const REGInstruction& index, TreatAsLongReal) const;
    [[nodiscard]] VariantWithFaultRecord<ExtendedReal> unpackSrc1(const REGInstruction& index, TreatAsExtendedReal) const;
    [[nodiscard]] VariantWithFaultRecord<ExtendedReal> unpackSrc2(const REGInstruction& index, TreatAsExtendedReal) const;
    [[nodiscard]] LongOrdinal unpackSrc1(const REGInstruction& index, TreatAsLongOrdinal) const;
    [[nodiscard]] LongInteger unpackSrc1(const REGInstruction& index, TreatAsLongInteger) const;
    [[nodiscard]] Integer unpackSrc1(const REGInstruction& index, TreatAsInteger) const;

    [[maybe_unused]] [[nodiscard]] VariantWithFaultRecord<std::reference_wrapper<TripleRegister>> getFloatingPointRegister(ByteOrdinal index);
    [[nodiscard]] VariantWithFaultRecord<std::reference_wrapper<const TripleRegister>> getFloatingPointRegister(ByteOrdinal index) const;
    [[nodiscard]] VariantWithFaultRecord<ExtendedReal> getFloatingPointRegisterValue(ByteOrdinal index) const noexcept;

    template<typename T>
    requires std::floating_point<T>
    VariantWithFaultRecord<T> getFloatingPointLiteral(ByteOrdinal index) const {
        switch (index) {
            case 0b10000: // +0.0
                return static_cast<T>(+0.0);
            case 0b10110: // +1.0
                return static_cast<T>(+1.0);
            default:
                return invalidOpcodeFault();
        }
    }
    template<typename T>
    requires std::floating_point<T>
    void handleUnderflowCondition(T value, FaultRecord& record) {
        record.type |= FloatingPointUnderflowFault;
        /// @todo fully implement
    }
    template<typename T>
    requires std::floating_point<T>
    void handleOverflowCondition(T value, FaultRecord& record) {
        record.type |= FloatingPointOverflowFault;
        /// @todo fully implement
    }
    template<typename T>
    requires std::floating_point<T>
    void handlePureInexactCondition(T value, FaultRecord& record) {
        /// @todo fully implement
    }
    template<typename T>
    requires std::floating_point<T>
    void handleMixedInexactCondition(T value, FaultRecord& record) {
        /// @todo fully implement
    }
    template<typename T>
    requires std::floating_point<T>
    using FloatingPointFaultServicingResult = std::variant<OptionalFaultRecord, T>;
    /**
     * @brief Checks the error flags to see if a floating point exeception has taken place and generates one if allowed
     */
    template<typename T>
    requires std::floating_point<T>
    FloatingPointFaultServicingResult<T> serviceFloatingPointFault(T value) {
        if (auto exceptions = std::fetestexcept(FE_ALL_EXCEPT); exceptions != 0) {
            std::feclearexcept(FE_ALL_EXCEPT);
            if (exceptions & FE_DIVBYZERO) {
                return floatingZeroDivideOperationFault();
            } else if (exceptions & FE_INVALID) {
                return floatingInvalidOperationFault();
            } else {
                FaultRecord record((Ordinal) pc_,
                                   (Ordinal) ac_,
                                   0,
                                   (Ordinal) ip_,
                                   true);
                if (exceptions & FE_OVERFLOW) {
                    if (ac_.arith.floatingOverflowMask == 0) {
                        handleOverflowCondition<T>(value, record);
                    } else {
                        ac_.arith.floatingOverflowFlag = 1;
                    }
                }
                if (exceptions & FE_UNDERFLOW) {
                    if (ac_.arith.floatingUnderflowMask == 0) {
                        handleUnderflowCondition<T>(value, record);
                    } else {
                        ac_.arith.floatingUnderflowFlag = 1;
                    }
                }
                if (exceptions & FE_INEXACT) {
                    if (ac_.arith.floatingInexactMask == 0) {
                        record.type |= FloatingPointInexactFault;
                        if (record.type != 0) {
                            // If set, F0 indicates that the adjusted result has been rounded towards positive infinity
                            // If clear, F0 indicates that the adjusted result has been rounded toward negative infinity
                            handleMixedInexactCondition(value, record);
                        } else {
                            handlePureInexactCondition(value, record);
                        }
                    } else {
                        ac_.arith.floatingInexactFlag = 1;
                    }
                } else {
                    // SET F1 if the adjusted result has been bias adjusted because its exponent was outside the range of the extended-real format
                    if (record.type == FloatingPointUnderflowFault) {
                        handleUnderflowCondition<T>(value, record);
                    }
                    if (record.type == FloatingPointOverflowFault) {
                        handleOverflowCondition<T>(value, record);
                    }
                }
                if (record.type != 0) {
                    return record;
                } else {
                    return value;
                }
            }
        } else {
            return value;
        }
    }
    inline OptionalFaultRecord fpassignment(const REGInstruction& inst, OptionalFaultRecord value, TreatAs<OptionalFaultRecord>) { return value; }
    template<typename T>
    requires std::floating_point<T>
    OptionalFaultRecord fpassignment(const REGInstruction& inst, FloatingPointFaultServicingResult<T> value, TreatAs<T>) {
        return std::visit([this, &inst](auto&& value) { return fpassignment(inst, value, TreatAs<std::decay_t<decltype(value)>>{}); }, value);
    }
    void updateRoundingMode() const;
    template<typename T>
    requires std::floating_point<T>
    [[nodiscard]] VariantWithFaultRecord<T> handleSubnormalCase(T input) const {
        if (std::fpclassify(input) == FP_SUBNORMAL) {
            auto result = floatingReservedEncodingFault();
            if (result) {
                return *result;
            }
        }
        return input;
    }
    template<typename T>
    [[nodiscard]] VariantWithFaultRecord<T> handleSubnormalCase(const VariantWithFaultRecord<T>& input) const {
        if (std::holds_alternative<FaultRecord>(input)) {
            return input;
        } else {
            return handleSubnormalCase<T>(std::get<T>(input));
        }
    }
    OptionalFaultRecord performClassification(const FaultRecord& record) noexcept;
    template<typename T>
    requires std::floating_point<T>
    OptionalFaultRecord
    performClassification(T value) {
        switch(std::fpclassify(value)) {
            case FP_ZERO:
                ac_.arith.arithmeticStatus = 0;
                break;
            case FP_SUBNORMAL:
                ac_.arith.arithmeticStatus = 0b001;
                break;
            case FP_NORMAL:
                ac_.arith.arithmeticStatus = 0b010;
                break;
            case FP_INFINITE:
                ac_.arith.arithmeticStatus = 0b011;
                break;
            case FP_NAN:
                ac_.arith.arithmeticStatus = issignaling(value) ? 0b101 : 0b100;
                break;
            default:
                ac_.arith.arithmeticStatus = 0b110;
                break;
        }
        return std::nullopt;
    }
    template<typename T>
    requires std::floating_point<T>
    OptionalFaultRecord classifyGeneric(const REGInstruction& inst) noexcept {
        return std::visit([this](auto value) { return performClassification(value); }, unpackSrc1(inst, TreatAs<T>{}));
    }
    void logbnr(const REGInstruction& inst);
    void logbnrl(const REGInstruction& inst);
    void logr(const REGInstruction& inst);
    void logrl(const REGInstruction& inst);
    void logepr(const REGInstruction& inst);
    void logeprl(const REGInstruction& inst);
    /// @todo figure out what operations have not yet been implemented
private:
    OptionalFaultRecord movl(const REGInstruction& inst);
    OptionalFaultRecord movt(const REGInstruction& inst);
    OptionalFaultRecord movq(const REGInstruction& inst);
private:
    Opcodes getInstructionOpcode() const noexcept {
        if (instruction_.isREGFormat()) {
            return _regInstruction.getOpcode();
        } else {
            return static_cast<Opcodes>(instruction_.getMajorOpcode());
        }
    }
private:
    Ordinal systemAddressTableBase_ = 0;
    Ordinal prcbAddress_ = 0;
    RegisterFrame globals_;
    std::array<LocalRegisterSet, NumberOfLocalRegisterFrames> frames_;
    uint8_t localRegisterFrameIndex_ = 0;
    RegisterBlock32 constants_;
    RegisterFrame fp;
    Register ip_;
    Register ac_;
    Register pc_;
    Register tc_;
    union {
        Register instruction_{0};
        MEMInstruction _memInstruction;
        COBRInstruction _cobrInstruction;
        CTRLInstruction _ctrlInstruction;
        REGInstruction _regInstruction;
    };
    Register ictl_;
    ByteOrdinal instructionLength_ = 0;
    bool advanceInstruction_ = false;
    /// @todo implement breakpoint support
    /// @todo implement backing structures for the protected architecture
    // protected architecture extensions
    Register processControlBlockBaseAddress_;
    /**
     * @brief A 64-bit counter of the number of instructions executed since startup; Used as a pseudo random number source as well
     */
    LongOrdinal pseudoRandomSource_ = 0;
};

static_assert(computeNextFrame<Core::C, Core::NotC>(0xFDED'0000) == 0xFDED'0000);
static_assert(computeNextFrame<Core::C*2, Core::NotC>(0xFDED'0000) == 0xFDED'0040);
static_assert(computeNextFrame<Core::C*3, Core::NotC>(0xFDED'0000) == 0xFDED'0080);
static_assert(computeNextFrame<Core::C*4, Core::NotC>(0xFDED'0000) == 0xFDED'00C0);

void installToMainMemory(std::istream& stream, Address baseAddress);
void installToMainMemory(Address baseAddress, const char* data, Address size);
void clearMainMemory(Address baseAddress, Address size);
#endif // end SIM5_CORE_H__
