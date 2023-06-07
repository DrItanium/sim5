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

#ifndef SIM5_TYPES_H__
#define SIM5_TYPES_H__
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdint>
#define F(x) x
#endif // end defined(ARDUINO)
using Address = uint32_t;
using ByteOrdinal = uint8_t;
using ByteInteger = int8_t;
using ShortOrdinal = uint16_t;
using ShortInteger = int16_t;
using Ordinal = uint32_t;
using Integer = int32_t;
using LongOrdinal = uint64_t;
using LongInteger = int64_t;
using Real = float;
using LongReal = double;
using ExtendedReal = long double;
template<typename T> struct TreatAs { };
using TreatAsOrdinal = TreatAs<Ordinal>;
using TreatAsInteger = TreatAs<Integer>;
using TreatAsShortOrdinal = TreatAs<ShortOrdinal>;
using TreatAsByteOrdinal = TreatAs<ByteOrdinal>;
using TreatAsShortInteger = TreatAs<ShortInteger>;
using TreatAsByteInteger = TreatAs<ByteInteger>;
using TreatAsLongOrdinal = TreatAs<LongOrdinal>;
using TreatAsLongInteger = TreatAs<LongInteger>;
using TreatAsReal = TreatAs<Real>;
using TreatAsLongReal = TreatAs<LongReal>;
using TreatAsExtendedReal = TreatAs<ExtendedReal>;


constexpr Ordinal computeChecksumOffset(Address segmentTableBase, Address prcbBase, Address startAddress) noexcept {
    return -(segmentTableBase+prcbBase+startAddress);
}
static_assert(computeChecksumOffset(0, 0xb0, 0x6ec) == 0xffff'f864, "checksum offset is broken!");
constexpr Address alignTo16ByteBoundaries(Address value) noexcept {
    constexpr Address Mask = 0xFFFF'FFF0;
    return value & Mask;
}
constexpr Address alignTo4ByteBoundaries(Address value) noexcept {
    constexpr Address Mask = ~(0b11);
    return value & Mask;
}
constexpr Address alignTo8ByteBoundaries(Address value) noexcept {
    constexpr Address Mask = ~(0b111);
    return value & Mask;
}
constexpr Address alignTo64ByteBoundaries(Address value) noexcept {
    constexpr Address Mask = ~(0b11'1111);
    return value & Mask;
}
constexpr Address alignTo4096ByteBoundaries(Address value) noexcept {
    constexpr Address Mask = ~(0xFFF);
    return value & Mask;
}
using SegmentSelector = Ordinal;
constexpr Address translateSegmentDescriptorToOffset(SegmentSelector value) noexcept {
    return value >> 6;
}
static_assert(translateSegmentDescriptorToOffset(0x1ff) == 7);
static_assert(translateSegmentDescriptorToOffset(0x23f) == 8);
static_assert(translateSegmentDescriptorToOffset(0x27f) == 9);
constexpr SegmentSelector generateSegmentDescriptor(Address value) noexcept {
    return (value << 6) + 0b111'111;
}
static_assert(generateSegmentDescriptor(9) == 0x27f);
static_assert(generateSegmentDescriptor(8) == 0x23f);
static_assert(generateSegmentDescriptor(7) == 0x1ff);

/**
 * @brief the data structure the processor creates upon a fault being
 * triggered.
 */
struct [[gnu::packed]] FaultRecord {
    Ordinal unused;
    Ordinal overrideFaultData[3];
    Ordinal faultData[3];
    Ordinal overrideType;
    /**
     * @brief A copy of the ProcessControls register at the time the fault was
     * raised.
     */
    Ordinal pc;
    /**
     * @brief A copy of the ArithmeticControls register at the time the
     * fault was raised.
     */
    Ordinal ac;
    /**
     * @brief the type of the fault
     */
    Ordinal type;
    /**
     * @brief The address of the faulting instruction
     */
    Address addr;
    FaultRecord() = default;
    FaultRecord(Ordinal p, Ordinal a, Ordinal t, Address adr) : unused(0), overrideFaultData{0}, faultData{0}, overrideType(0), pc(p), ac(a), type(t), addr(adr) { }
    [[nodiscard]] constexpr uint8_t getFaultFlags() const noexcept { return static_cast<uint8_t>(type >> 24); }
    [[nodiscard]] constexpr uint8_t getFaultType() const noexcept { return static_cast<uint8_t>(type >> 16); }
    [[nodiscard]] constexpr uint8_t getFaultSubtype() const noexcept { return static_cast<uint8_t>(type); }
    [[nodiscard]] constexpr uint8_t getOverrideFlags() const noexcept { return static_cast<uint8_t>(overrideType >> 24); }
    [[nodiscard]] constexpr uint8_t getOverrideType() const noexcept { return static_cast<uint8_t>(overrideType >> 16); }
    [[nodiscard]] constexpr uint8_t getOverrideSubtype() const noexcept { return static_cast<uint8_t>(overrideType); }
    [[nodiscard]] constexpr bool clearTraceEnableBit() const noexcept {
        switch (getFaultType()) {
            case 0: // override or parallel
            case 1: // trace
                return true;
            default:
                return false;
        }
    }
};

struct [[gnu::packed]] FaultTableEntry {
    public:
        explicit constexpr FaultTableEntry(Ordinal addr, SegmentSelector selector) noexcept : handlerFunctionAddress_(addr), selector_(selector) { }
        constexpr FaultTableEntry() noexcept : FaultTableEntry(0, 0) { }
        constexpr bool isSystemTableEntry() const noexcept { return (handlerFunctionAddress_ & 0b11) == 0b10; }
        constexpr bool isLocalProcedureEntry() const noexcept { return (handlerFunctionAddress_ & 0b11) == 0; }
        constexpr auto getSegmentSelector() const noexcept { return selector_; }
        constexpr auto getFaultHandlerProcedureAddress() const noexcept { return handlerFunctionAddress_; }
        constexpr auto getFaultHandlerProcedureNumber() const noexcept { return (handlerFunctionAddress_ & (~0b11)); }
    private:
        Ordinal handlerFunctionAddress_;
        SegmentSelector selector_;
};


struct [[gnu::packed]] FaultTable {
    FaultTableEntry entries[32];
};
static_assert(sizeof(FaultTable) == 256);

struct [[gnu::packed]] InterruptTable {
    Ordinal pendingPriorities;
    Ordinal pendingInterrupts[8];
    Address interruptProcedureBases[248];
};

/**
 * @brief A data structure pushed onto the stack when an interrupt is
 * triggered.
 */
struct InterruptRecord {
    /**
     * @brief Contents of process controls register at the time of the
     * interrupt trigger.
     */
    Ordinal pc;
    /**
     * @brief Contents of arithmetic controls register at the time of the
     * interrupt trigger.
     */
    Ordinal ac;
    /**
     * @brief The vector being triggered
     */
    ByteOrdinal vectorNumber;
};

/**
 * @brief Used by the i960 when invoking the "calls" instruction; Allows
 * for indirect system calls based off of index; 
 */
struct SystemProcedureTable {
    Ordinal reserved[3];
    /**
     * @brief The address of the supervisorStack
     */
    Ordinal supervisorStack; 
    /**
     * @brief A set of 8 words that the i960 will preserve; It has no
     * external purpose; Must be zero
     */
    Ordinal preserved[8];

    /**
     * @brief The base address of the functions to be called indirectly by
     * the "calls" instruction.
     */
    Address entries[260];
};
struct [[gnu::packed]] SegmentDescriptor {
    constexpr SegmentDescriptor(Ordinal reserved0, Ordinal reserved1, Ordinal addr, Ordinal config) noexcept 
        : reserved{reserved0, reserved1}, address(addr), cfg{config} {

        }
    constexpr SegmentDescriptor() noexcept : SegmentDescriptor(0, 0, 0, 0) { }
    [[nodiscard]] constexpr bool valid() const noexcept { return cfg.bits.valid; }
    [[nodiscard]] constexpr ByteOrdinal getPagingMethod() const noexcept { return cfg.bits.pagingMethod; }
    [[nodiscard]] constexpr ByteOrdinal getAccessStatus() const noexcept { return cfg.bits.accessStatus; }
    [[nodiscard]] constexpr ByteOrdinal getSize() const noexcept { return cfg.bits.size; }
    [[nodiscard]] constexpr ByteOrdinal getSegmentType() const noexcept { return cfg.bits.segmentType; }
    [[nodiscard]] constexpr bool entryIsInvalid() const noexcept { return (cfg.raw & 0b111) == 0; }
    [[nodiscard]] constexpr bool isSemaphore() const noexcept { return cfg.raw == 0x4000'0001; }
    [[nodiscard]] constexpr Address getAddress() const noexcept { return address; }
    Ordinal reserved[2];
    Address address;
    union Configuration {
        constexpr Configuration(Ordinal value = 0) : raw(value) { }
        Ordinal raw = 0;
        struct {
            Ordinal valid : 1;
            Ordinal pagingMethod : 2;
            Ordinal accessStatus : 5;
            Ordinal reserved0 : 10;
            Ordinal size : 6;
            Ordinal reserved1 : 4;
            Ordinal segmentType : 4;
        } bits;
    } cfg;
};
static_assert(sizeof(SegmentDescriptor) == 16, "Segment descriptors must be 16 bytes in length");

/**
 * @brief A table of segment descriptors to give the i960 more information
 * about different parts of the memory space; On processors which do not
 * support the Protected Architecture, the use of this table is very
 * limited and is called the "System Address Table" instead; Only entry
 * 8 has a fixed purpose and must point to the first address of this table;
 * The segment table can be very large when dealing with the protected
 * architecture
 */
struct [[gnu::packed]] SegmentDescriptorTable {
    SegmentDescriptor descriptors[11];

    const SegmentDescriptor& getHeadPointer() const noexcept { return descriptors[8]; }
};
static_assert(sizeof(SegmentDescriptorTable) == 176);
using SystemAddressTable = SegmentDescriptorTable;
/**
 * @brief Write this value to reservedBootArgs of the PRCB; It will disable cpu
 * prefetching which can reduce performance
 */
constexpr Ordinal DisablePrefetching_Sx = 0xFFFF'FF10;

/**
 * @brief Used by the processor to describe initial system state
 */
struct [[gnu::packed]] ProcessorControlBlock {
    Ordinal reserved0;
    Ordinal processorControls; // this is a magic number of the Sx series
    Ordinal reserved1;
    Ordinal currentProcessSS;
    Ordinal dispatchPortSS;
    Ordinal interruptTableBase;
    Ordinal interruptStackBase;
    Ordinal reserved2;
    SegmentSelector region3Selector; 
    SegmentSelector systemProcedureTableSelector;
    Address faultTablePhysicalAddress;
    Ordinal reservedBootArgs; // @44
    Ordinal multiprocessorPreemption[3];
    Ordinal reserved3;
    LongOrdinal idleTime;
    Ordinal systemErrorFault;
    Ordinal reserved4;
    Ordinal resumptionRecord[12];
    FaultRecord systemErrorFaultRecord;
    //static_assert(sizeof(resumptionRecord) == 48);
};

using PRCB = ProcessorControlBlock;
static_assert(sizeof(ProcessorControlBlock) == 176);

union SplitWord128 {
    uint64_t longWords[2];
    uint32_t words[4];
    uint16_t halfWords[8];
    uint8_t bytes[16];
    double longReals[2];
    float reals[4];
};
static_assert(sizeof(double) == 8);
static_assert(sizeof(float) == 4);
static_assert(sizeof(SplitWord128) == 16);

constexpr uint8_t computeInterruptPriority(uint8_t vector) noexcept {
    return vector / 8;
}
constexpr uint8_t computeInterruptWordIndex(uint8_t value) noexcept {
    return (value >> 5);
}
constexpr uint8_t computeInterruptVectorOffset(uint8_t value) noexcept {
    return value & 0b111;
}
#define X(index) static_assert(computeInterruptPriority( index ) == (index / 8));
#include "Entry255.def"
#undef X
enum class InterruptVector : uint8_t { };

constexpr uint8_t computeInterruptPriority(InterruptVector vector) noexcept {
    return computeInterruptPriority(static_cast<uint8_t>(vector));
}

constexpr uint8_t computeInterruptWordIndex(InterruptVector vector) noexcept {
    return computeInterruptWordIndex(static_cast<uint8_t>(vector));
}

constexpr uint8_t computeInterruptVectorOffset(InterruptVector vector) noexcept {
    return computeInterruptVectorOffset(static_cast<uint8_t>(vector));
}

#endif // end SIM5_TYPES_H__
