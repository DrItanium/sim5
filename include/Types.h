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

#ifndef SIM5_TYPES_H__
#define SIM5_TYPES_H__
#include <cstdint>
#include <compare>
#include <type_traits>
#include <concepts>
#include <bitset>
#include <variant>
#include <optional>
#include "SoftFloatWrapper.h"

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
template<typename T> struct TreatAs { using BackingType = T; };
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

using int128_t = signed __int128;
using uint128_t = unsigned __int128;
template<typename T>
concept NonFloatingPointNumber = std::integral<T>;
template<typename T, uint8_t B>
requires NonFloatingPointNumber<T> && (B <= 128) && (B > 0)
struct LargeNumberPackage {
    using BackingStore = T;
    using Self = LargeNumberPackage<BackingStore, B>;
    constexpr LargeNumberPackage(BackingStore backing) : value_(backing) { }
    constexpr LargeNumberPackage(const Self& other) = default;
    constexpr LargeNumberPackage(Self&& other) = default;
    auto operator<=>(const Self& other) const noexcept = default;
    constexpr Self& operator+=(const Self& other) {
        value_ += other.value_;
        return *this;
    }
    constexpr friend Self operator+(Self lhs, const Self& rhs) {
        lhs += rhs;
        return lhs;
    }
    constexpr Self& operator-=(const Self& other) {
        value_ -= other.value_;
        return *this;
    }
    constexpr friend Self operator-(Self lhs, const Self& rhs) {
        lhs -= rhs;
        return lhs;
    }
    constexpr Self& operator*=(const Self& other) {
        value_ *= other.value_;
        return *this;
    }
    constexpr friend Self operator*(Self lhs, const Self& rhs) {
        lhs *= rhs;
        return lhs;
    }
    constexpr Self& operator%=(const Self& other) {
        value_ %= other.value_;
        return *this;
    }
    constexpr friend Self operator%(Self lhs, const Self& rhs) {
        lhs %= rhs;
        return lhs;
    }
    constexpr Self& operator/=(const Self& other) {
        value_ /= other.value_;
        return *this;
    }
    constexpr friend Self operator/(Self lhs, const Self& rhs) {
        lhs /= rhs;
        return lhs;
    }
    constexpr Self& operator&=(const Self& other) {
        value_ &= other.value_;
        return *this;
    }
    constexpr friend Self operator&(Self lhs, const Self& rhs) {
        lhs &= rhs;
        return lhs;
    }
    constexpr Self& operator|=(const Self& other) {
        value_ &= other.value_;
        return *this;
    }
    constexpr friend Self operator|(Self lhs, const Self& rhs) {
        lhs &= rhs;
        return lhs;
    }
    constexpr Self& operator^=(const Self& other) {
        value_ ^= other.value_;
        return *this;
    }
    constexpr friend Self operator^(Self lhs, const Self& rhs) {
        lhs ^= rhs;
        return lhs;
    }
    constexpr Self& operator>>=(const Self& other) {
        value_ >>= other.value_;
        return *this;
    }
    constexpr friend Self operator>>(Self lhs, const Self& rhs) {
        lhs >>= rhs;
        return lhs;
    }
    constexpr Self& operator<<=(const Self& other) {
        value_ <<= other.value_;
        return *this;
    }
    constexpr friend Self operator<<(Self lhs, const Self& rhs) {
        lhs <<= rhs;
        return lhs;
    }
    constexpr Self& operator~() noexcept {
        value_ = ~value_;
        return *this;
    }
    constexpr Self& operator+() noexcept {
        value_ = +value_;
        return *this;
    }
    constexpr Self& operator-() noexcept {
        value_ = -value_;
        return *this;
    }
    constexpr Self& operator++() {
        // prefix
        ++value_;
        return *this;
    }
    constexpr Self operator++(int) {
        // postfix
        Self old = *this;
        operator++();
        return old;
    }
    constexpr Self& operator--() {
        // prefix
        --value_;
        return *this;
    }
    constexpr Self operator--(int) {
        // postfix
        Self old = *this;
        operator--();
        return old;
    }
    constexpr explicit operator T() const noexcept {
        return value_;
    }
    template<typename Q>
    constexpr explicit operator Q() const noexcept {
        return static_cast<Q>(value_);
    }
    constexpr explicit operator bool() const noexcept {
        return value_ != 0;
    }
private:
    BackingStore value_ : B;
};
template<uint8_t B>
requires (B <= 128) && (B > 0)
using LargeIntPackage = std::conditional_t<(B == 128), int128_t, LargeNumberPackage<int128_t, B>>;
template<uint8_t B>
requires (B <= 128) && (B > 0)
using LargeUIntPackage = std::conditional_t<(B == 128), uint128_t, LargeNumberPackage<uint128_t, B>>;
using int96_t = LargeIntPackage<96>;
using uint96_t = LargeUIntPackage<96>;
using TripleOrdinal = uint96_t;
using TripleInteger = int96_t;
static_assert(TripleOrdinal{0} != TripleOrdinal{1});
static_assert(TripleInteger{0} != TripleInteger{1});
static_assert(TripleInteger{0} > TripleInteger{-1});
static_assert(static_cast<Integer>(TripleInteger{0xFDED}) == Integer{0xFDED});
static_assert(~TripleInteger{0} == TripleInteger{-1});
static_assert((TripleInteger{0} + TripleInteger{1}) == TripleInteger{1});
using QuadOrdinal = LargeUIntPackage<128>;
using QuadInteger = LargeIntPackage<128>;
static_assert(QuadOrdinal{0} != QuadOrdinal{1});
static_assert(QuadInteger{0} != QuadInteger{1});
static_assert(QuadInteger{0} > QuadInteger{-1});
static_assert(static_cast<Integer>(QuadInteger{0xFDED}) == Integer{0xFDED});

using TreatAsTripleInteger = TreatAs<TripleInteger>;
using TreatAsTripleOrdinal = TreatAs<TripleOrdinal>;
using TreatAsQuadInteger = TreatAs<QuadInteger>;
using TreatAsQuadOrdinal = TreatAs<QuadOrdinal>;

constexpr Ordinal computeChecksumOffset(Address segmentTableBase, Address prcbBase, Address startAddress) noexcept {
    return -(segmentTableBase+prcbBase+startAddress);
}
static_assert(computeChecksumOffset(0, 0xb0, 0x6ec) == 0xffff'f864, "checksum offset is broken!");
template<typename T>
constexpr T alignTo4ByteBoundaries(T value, TreatAs<T>) noexcept {
    constexpr T Mask = ~(0b11);
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
struct FaultRecord {
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
    bool saveReturnAddress;
    FaultRecord() = default;
    FaultRecord(Ordinal p, Ordinal a, Ordinal t, Address adr, bool mustSaveReturnAddress) : unused(0), overrideFaultData{0}, faultData{0}, overrideType(0), pc(p), ac(a), type(t), addr(adr), saveReturnAddress(mustSaveReturnAddress) { }
    [[nodiscard]] constexpr uint8_t getFaultFlags() const noexcept { return static_cast<uint8_t>(type >> 24); }
    void setFaultFlags(uint8_t value) noexcept {
        auto shifted = (static_cast<Ordinal>(value) << 24) & 0xFF00'0000;
        auto masked = type & 0x00FFFFFF;
        type = (shifted | masked);
    }
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

template<typename ... T>
using VariantWithFaultRecord = std::variant<FaultRecord, T...>;
using OptionalFaultRecord = std::optional<FaultRecord>;


struct FaultTableEntry {
public:
    explicit constexpr FaultTableEntry(Ordinal addr, SegmentSelector selector) noexcept : handlerFunctionAddress_(addr), selector_(selector) { }
    explicit constexpr FaultTableEntry(LongOrdinal value) noexcept : FaultTableEntry(static_cast<Ordinal>(value), static_cast<Ordinal>(value >> 32)) { }
    constexpr FaultTableEntry() noexcept : FaultTableEntry(0, 0) { }
    [[nodiscard]] constexpr bool isSystemTableEntry() const noexcept { return (handlerFunctionAddress_ & 0b11) == 0b10; }
    [[nodiscard]] constexpr bool isLocalProcedureEntry() const noexcept { return (handlerFunctionAddress_ & 0b11) == 0; }
    [[nodiscard]] constexpr auto getSegmentSelector() const noexcept { return selector_; }
    [[nodiscard]] constexpr auto getFaultHandlerProcedureAddress() const noexcept { return handlerFunctionAddress_; }
    [[nodiscard]] constexpr auto getFaultHandlerProcedureNumber() const noexcept { return (handlerFunctionAddress_ & (~0b11)); }
private:
    Ordinal handlerFunctionAddress_;
    SegmentSelector selector_;
};
static_assert(sizeof(FaultTableEntry) == 8);

struct SegmentDescriptor {
    constexpr SegmentDescriptor(Ordinal reserved0, Ordinal reserved1, Ordinal addr, Ordinal config) noexcept
            : reserved{reserved0, reserved1}, address(addr), cfg{config} {

    }
    constexpr explicit SegmentDescriptor(QuadOrdinal value) : SegmentDescriptor(static_cast<Ordinal>(value),
                                                                                static_cast<Ordinal>(value >> 32),
                                                                                static_cast<Ordinal>(value >> 64),
                                                                                static_cast<Ordinal>(value >> 96)) { }
    constexpr SegmentDescriptor() noexcept : SegmentDescriptor(0, 0, 0, 0) { }
    [[nodiscard]] constexpr bool valid() const noexcept { return cfg.bits.valid; }
    enum class PagingMethod : uint8_t {
        Error,
        Unpaged,
        Paged,
        Bipaged,
    };
    [[nodiscard]] constexpr auto getPagingMethod() const noexcept { return static_cast<PagingMethod>(cfg.bits.pagingMethod);  }
    [[nodiscard]] constexpr auto refersToBipagedRegion() const noexcept { return getPagingMethod() == PagingMethod::Bipaged; }
    [[nodiscard]] constexpr auto refersToPagedRegion() const noexcept { return getPagingMethod() == PagingMethod::Paged; }
    [[nodiscard]] constexpr auto refersToSimpleRegion() const noexcept { return getPagingMethod() == PagingMethod::Unpaged; }
    [[nodiscard]] constexpr ByteOrdinal getRawSize() const noexcept { return cfg.bits.size; }
    [[nodiscard]] constexpr auto computeSegmentSize() const noexcept { return 64 * (getRawSize() + 1); }
    [[nodiscard]] constexpr ByteOrdinal getSegmentType() const noexcept { return cfg.bits.segmentType; }
    [[nodiscard]] constexpr bool entryIsInvalid() const noexcept { return (cfg.raw & 0b111) == 0; }
    [[nodiscard]] constexpr bool isSemaphore() const noexcept { return cfg.raw == 0x4000'0001; }
    [[nodiscard]] constexpr Address getAddress() const noexcept { return address; }
    [[nodiscard]] constexpr Ordinal getBaseAddress() const noexcept { return address & ~(0xFFF); }
    [[nodiscard]] constexpr Ordinal getTableAddress() const noexcept { return address & ~(0b111'111); }
    [[nodiscard]] constexpr Address computePhysicalAddress(Address virtualAddress) const noexcept { return getBaseAddress() | (virtualAddress & 0xFFF); }
    [[nodiscard]] constexpr bool hasBeenAccessed() const noexcept { return cfg.bits.accessed; }
    [[nodiscard]] constexpr bool hasBeenAltered() const noexcept { return cfg.bits.altered; }
    void markAccessed() noexcept { cfg.bits.accessed = 1; }
    void markAltered() noexcept { cfg.bits.altered = 1; }
    [[nodiscard]] constexpr bool isCacheable() const noexcept { return cfg.bits.cacheable; }
    Ordinal reserved[2];
    Address address;
    union Configuration {
        constexpr explicit Configuration(Ordinal value = 0) : raw(value) { }
        Ordinal raw = 0;
        struct {
            Ordinal valid : 1; // b0
            Ordinal pagingMethod : 2;
            Ordinal accessed : 1; // b3
            Ordinal altered : 1; // b4
            Ordinal b5 : 1;
            Ordinal cacheable : 1;
            Ordinal b7 : 1;
            Ordinal reserved0 : 10;
            Ordinal size : 6;
            Ordinal reserved1 : 4;
            Ordinal segmentType : 4;
        } bits;
    } cfg;
    static_assert(sizeof(Configuration) == sizeof(Ordinal));
};
static_assert(sizeof(SegmentDescriptor) == 16, "Segment descriptors must be 16 bytes in length");


constexpr uint8_t computeInterruptPriority(uint8_t vector) noexcept {
    return vector / 8;
}
constexpr uint8_t computeInterruptWordIndex(uint8_t value) noexcept {
    return (value >> 5);
}
constexpr uint8_t computeInterruptVectorOffset(uint8_t value) noexcept {
    return value & 0b111;
}
constexpr uint8_t computeInterruptVectorByteOffset(uint8_t value) noexcept {
    // the middle two bits are important for offset purposes
    return computeInterruptPriority(value) & 0b11;
}
constexpr uint8_t computeInterruptVectorBitOffset(uint8_t value) noexcept {
    // the lower 5 bits are a bit offset
    return (value & 0b11111);
}
#define X(index) static_assert(computeInterruptPriority( index ) == (index / 8));
#include "Entry255.def"
#undef X
enum class InterruptVector : uint8_t { };

constexpr bool valid(InterruptVector vector) noexcept {
    return static_cast<uint8_t>(vector) >= 8;
}


constexpr uint8_t computeInterruptPriority(InterruptVector vector) noexcept {
    return computeInterruptPriority(static_cast<uint8_t>(vector));
}

constexpr uint8_t computeInterruptWordIndex(InterruptVector vector) noexcept {
    return computeInterruptWordIndex(static_cast<uint8_t>(vector));
}

constexpr uint8_t computeInterruptVectorOffset(InterruptVector vector) noexcept {
    return computeInterruptVectorOffset(static_cast<uint8_t>(vector));
}

constexpr uint8_t computeInterruptVectorByteOffset(InterruptVector vector) noexcept {
    return computeInterruptVectorByteOffset(static_cast<uint8_t>(vector));
}
constexpr uint8_t computeInterruptVectorBitOffset(InterruptVector vector) noexcept {
    return computeInterruptVectorBitOffset(static_cast<uint8_t>(vector));
}

constexpr bool canDispatchVector(InterruptVector vector, uint8_t systemPriority) noexcept {
    if (auto actualPriority = computeInterruptPriority(vector); actualPriority == 31) {
        return true;
    } else {
        return actualPriority > systemPriority;
    }
}
enum class ArchitectureLevel : uint8_t {
    Core,
    NewCore,
    Numerics,
    Protected,
    Extended,
    Unknown,
};
template<typename T>
concept MustBeOrdinalOrInteger = std::same_as<T, Integer> || std::same_as<T, Ordinal>;

union PortFlags {
    [[nodiscard]] constexpr bool isFifoPort() const noexcept { return kind == 0; }
    [[nodiscard]] constexpr bool isPriorityPort() const noexcept { return kind == 1; }
    Ordinal raw = 0;
    struct {
        uint8_t lock;
        uint8_t preserved;
        Ordinal kind : 1;
        Ordinal q : 1;
        Ordinal reserved : 14;
    };
};

struct ObjectOffset {
    Ordinal pageOffset : 12;
    Ordinal pageIndex : 10;
    Ordinal directoryIndex : 10;
    constexpr bool isSimple() const noexcept { return pageIndex == 0 && directoryIndex == 0; }
    constexpr bool isPaged() const noexcept { return directoryIndex == 0; }
};

struct AccessDescriptor {
    Ordinal readRights : 1;
    Ordinal writeRights : 1;
    Ordinal typeRights : 3;
    Ordinal local : 1;
    Ordinal objectIndex : 26;
};

struct VirtualAddressFormat {
    ObjectOffset offset;
    AccessDescriptor ad;
};
class TypeDefinitionObject {
public:
    enum class ObjectType : ByteOrdinal {
        Generic,
        TDO,
        ProcessObject,
        DomainObject,
        Semaphore,
        PortObject,
        Reserved0,
        Reserved1,
        Available0,
        Available1,
        Available2,
        Available3,
        Available4,
        Available5,
        Available6,
        Available7,
    };
public:
    constexpr TypeDefinitionObject() : raw_(0) { }
    constexpr explicit TypeDefinitionObject(Ordinal value) : raw_(value) { }
    [[nodiscard]] constexpr bool canAmplifyAnyAD() const noexcept { return superTDO; }
    [[nodiscard]] constexpr bool canAmplifyOnlyMatchingADs() const noexcept { return !superTDO; }
    [[nodiscard]] constexpr bool managesObjectsUsingThisTDO() const noexcept { return extended; }
    [[nodiscard]] constexpr bool managesObjectsUsingSameObjectType() const noexcept { return !extended; }
    [[nodiscard]] constexpr ObjectType getType() const noexcept { return static_cast<ObjectType>(type); }
private:
    union {
        Ordinal raw_;
        struct {
            Ordinal superTDO : 1;
            Ordinal extended : 1;
            Ordinal reserved : 26;
            Ordinal type : 4;
        };
    };
};
/**
 * @brief How the extended architecture describes its SegmentDescriptor
 */
class StorageDescriptor {
public:
    constexpr explicit StorageDescriptor(QuadOrdinal value) : raw_(value) { }
    [[nodiscard]] constexpr bool isSimpleObjectDescriptor() const noexcept { return entryType == 0b10; }
    [[nodiscard]] constexpr bool isPagedObjectDescriptor() const noexcept { return entryType == 0b10; }
    [[nodiscard]] constexpr bool isBipagedObjectDescriptor() const noexcept { return entryType == 0b11; }
    [[nodiscard]] constexpr bool isValid() const noexcept { return valid; }
    [[nodiscard]] constexpr bool hasBeenAccessed() const noexcept { return accessed; }
    [[nodiscard]] constexpr bool hasBeenAltered() const noexcept { return altered; }
    [[nodiscard]] constexpr bool isMixed() const noexcept { return mixed; }
    [[nodiscard]] constexpr bool isCacheable() const noexcept { return cacheable; }
    [[nodiscard]] constexpr bool isLocal() const noexcept { return local; }
    [[nodiscard]] constexpr Address getBaseAddress() const noexcept { return baseAddress & (~0b111111); }
    [[nodiscard]] constexpr explicit operator bool() const noexcept { return isValid(); }
    [[nodiscard]] constexpr QuadOrdinal getWholeValue() const noexcept { return raw_; }
    [[nodiscard]] const TypeDefinitionObject& getTDO() const noexcept { return *reinterpret_cast<const TypeDefinitionObject*>(&tdo); }
    [[nodiscard]] TypeDefinitionObject& getTDO() noexcept { return *reinterpret_cast<TypeDefinitionObject*>(&tdo); }
private:
    union {
        QuadOrdinal raw_;
        struct {
            Ordinal preserved0;
            Ordinal tdo;
            Ordinal baseAddress;
            Ordinal valid: 1;
            Ordinal entryType: 2;
            Ordinal accessed: 1;
            Ordinal altered: 1;
            Ordinal mixed: 1;
            Ordinal cacheable: 1;
            Ordinal local: 1;
            Ordinal preserved1: 10;
            Ordinal objectLength: 6;
            Ordinal preserved2: 4;
            Ordinal objectType: 4;
        };
    };
};
static_assert(sizeof(StorageDescriptor) == sizeof(QuadOrdinal));

/**
 * @brief An ordinal with a 33rd bit which denotes its type. It is encoded in a 64-bit number for simplicity.
 * According to the BiiN CPU manual:
 * "8.2.4 Tagging
 *      An object contains access descriptors and/or data, that is, any binary information. Access
 *      descriptors and data can reside in the same object and can be interleaved in any arbitrary order.
 *
 *      In some systems (such as BiiN(TM) Systems), a tag bit is associated with each 4-byte aligned
 *      word in memory to indicate whether the word is data or an access descriptor. An access descriptor must
 *      be aligned to 4-byte boundary with a tag bit of one. A tag bit of zero indicates data.
 *
 *      In other systems, the tag bit is not used. The interpretation of a word as a data or an access
 *      descriptor depends on the operation; see chapter 16
 *
 *      In a word-aligned read or write of the whole word, the tag bit is either preserved or set to zero depending
 *      on the operation. In a non-word aligned read, or a partial read of a word, the tag bit of the returned value is
 *      always forced to zero. Similarly, in a non-word aligned write, or a partial write of a word, the tag bit of any
 *      of the modified words is always forced to zero. The data manipulation (arithmetic or logical) instructions
 *      require source operands with zero tag bits, and generate values with zero tag bits.
 *      "
 *  Looking at the i960XA datasheet, I see that there is a pin labeled "Cache/Tag" (which interestingly enough
 *  seems to be in the same position as the Kx and Mx processors as well). It seems to be a multi function pin which
 *  acts as an output in the AS state and then a I/O pin during the DEN and WAIT states. I believe that the BiiN systems
 *  had either a separate memory store for these tag bits or something like it. Basically, there is one bit per 4-byte word
 *  that is loaded from when the accessors are aligned to 4-bytes. You cannot forge these tag bits because they are hard wired
 *  to zero except in specific cases.
 *
 *  From the simulator's perspective, the 33-bit values are actually a subset of a 64-bit processor. We use the extra
 *  bit as separate memory _spaces_ which keep them in separate areas. When using the normal i960 instructions we are
 *  operating on the _lower_ 4 gigabytes. Only specific instructions allow access to the upper 32-bit conceptual memory space.
 */
union TaggedWord {
    LongOrdinal whole_ : 33;
    struct {
        Ordinal data;
        Ordinal tag : 1;
    } dataWord;
    struct {
        AccessDescriptor descriptor;
        Ordinal tag : 1;
    } accessDescriptor;
};
/// @todo look into std::bitset

[[nodiscard]] constexpr bool isCTRL(Ordinal o) noexcept { return o < 0x2000'0000; }
[[nodiscard]] constexpr bool isCOBR(Ordinal o) noexcept { return (o >= 0x2000'0000) && (o < 0x4000'0000); }
[[nodiscard]] constexpr bool isMEMFormat(Ordinal o) noexcept { return o >= 0x8000'0000; }
[[nodiscard]] constexpr auto isREGFormat(Ordinal o) noexcept { return o >= 0x4000'0000 && o < 0x8000'0000; }

// taken from https://www.modernescpp.com/index.php/visiting-a-std-variant-with-the-overload-pattern/
template<typename ... Ts>
struct Overload : Ts... {
    using Ts::operator() ...;
};

template<typename... Ts> Overload(Ts...) -> Overload<Ts...>;

template<typename T0, typename T1, typename T2>
constexpr bool BothAreSameAs = std::is_same_v<T0, T2> && std::is_same_v<T1, T2>;
template<typename T0, typename T1>
constexpr bool BothAreReal = BothAreSameAs<T0, T1, Real>;
template<typename T0, typename T1>
constexpr bool BothAreLongReal = BothAreSameAs<T0, T1, LongReal>;

#endif // end SIM5_TYPES_H__
