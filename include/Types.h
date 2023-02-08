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
using TreatAsShortOrdinal = TreatAs<ShortOrdinal>;
using TreatAsByteOrdinal = TreatAs<ByteOrdinal>;
using TreatAsShortInteger = TreatAs<ShortInteger>;
using TreatAsByteInteger = TreatAs<ByteInteger>;

union SplitWord32 {
    constexpr SplitWord32 (Ordinal a) : whole(a) { }
    Ordinal whole;
    struct {
        Ordinal lower : 15;
        Ordinal a15 : 1;
        Ordinal a16_23 : 8;
        Ordinal a24_31 : 8;
    } splitAddress;
    struct {
        Ordinal lower : 15;
        Ordinal bank0 : 1;
        Ordinal bank1 : 1;
        Ordinal bank2 : 1;
        Ordinal bank3 : 1;
        Ordinal rest : 13;
    } internalBankAddress;
    struct {
        Ordinal offest : 28;
        Ordinal space : 4;
    } addressKind;
    [[nodiscard]] constexpr bool inIOSpace() const noexcept { return addressKind.space == 0b1111; }
};
union SplitWord64 {
    LongOrdinal whole;
    Ordinal parts[sizeof(LongOrdinal) / sizeof(Ordinal)];
};

// Pins
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

template<typename T>
volatile T& memory(size_t address) noexcept {
    return *reinterpret_cast<volatile T*>(address);
}
namespace ebi {
    void set328BusAddress(const SplitWord32& address) noexcept;
    void setInternalBusAddress(const SplitWord32& address) noexcept;
    template<typename T>
    [[nodiscard]] T 
    load(Address address, TreatAs<T>) noexcept {
        SplitWord32 split(address);
        set328BusAddress(split);
        return memory<T>(static_cast<size_t>(split.splitAddress.lower) + 0x8000);
    }
    template<typename T>
    void 
    store(Address address, T value, TreatAs<T>) noexcept {
        SplitWord32 split(address);
        set328BusAddress(split);
        memory<T>(static_cast<size_t>(split.splitAddress.lower) + 0x8000) = value;
    }
    void begin() noexcept;

}



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
};
static_assert(sizeof(FaultRecord) == 48);

struct [[gnu::packed]] FaultTableEntry {
    explicit constexpr FaultTableEntry(Ordinal addr, SegmentSelector selector) noexcept : handlerFunctionAddress_(addr), selector_(selector) { }
    constexpr FaultTableEntry() noexcept : FaultTableEntry(0, 0) { }
    constexpr bool isLocalEntry() const noexcept  { return selector_ == 0; }
    Ordinal handlerFunctionAddress_;
    SegmentSelector selector_;
    static constexpr FaultTableEntry makeLocalEntry(Address targetAddress) noexcept { 
        return FaultTableEntry{alignTo4ByteBoundaries(targetAddress), 0};
    }
    static constexpr FaultTableEntry makeSystemCallEntry(Ordinal index) noexcept {
        return FaultTableEntry{(index << 2) | 0b10, 0x0000'027F};
    }
};


struct [[gnu::packed]] FaultTable {
    FaultTableEntry entries[32];
};

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
    SegmentDescriptor() : reserved{0,0}, address(0), cfg(0) { }
    constexpr ByteOrdinal getKind() const noexcept { return static_cast<ByteOrdinal>(cfg) & 0b111; }
    bool invalid() const noexcept { return getKind() == 0; }
    Ordinal reserved[2];
    Address address;
    Ordinal cfg;
};
static_assert(sizeof(SegmentDescriptor) == 16, "Segment descriptors must be 16 bytes in length");

/**
 * @brief A table of segment descriptors to give the i960 more information
 * about different parts of the memory space; On processors which do not
 * support the Protected Architecture, the use of this table is very
 * limited and is called the "System Address Table" instead; Only entry
 * 8 has a fixed purpose and must point to the first address of this table;
 * The segment table can be very large bu
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
    union {
        Ordinal scratchSpace[24];
        struct {
            Ordinal resumptionRecord[12];
            FaultRecord systemErrorFaultRecord;
        };
    };
    //Ordinal resumptionRecord[12];
    //static_assert(sizeof(resumptionRecord) == 48);
    //FaultRecord systemError;
};

using PRCB = ProcessorControlBlock;
static_assert(sizeof(ProcessorControlBlock) == 176);
#endif // end SIM5_TYPES_H__
