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
#include <cstdint>
#include <compare>
#include <type_traits>
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


constexpr Ordinal computeChecksumOffset(Address segmentTableBase, Address prcbBase, Address startAddress) noexcept {
    return -(segmentTableBase+prcbBase+startAddress);
}
static_assert(computeChecksumOffset(0, 0xb0, 0x6ec) == 0xffff'f864, "checksum offset is broken!");
template<typename T>
constexpr Address alignTo4ByteBoundaries(T value, TreatAs<T>) noexcept {
    constexpr Address Mask = ~(0b11);
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
        [[nodiscard]] constexpr bool isSystemTableEntry() const noexcept { return (handlerFunctionAddress_ & 0b11) == 0b10; }
        [[nodiscard]] constexpr bool isLocalProcedureEntry() const noexcept { return (handlerFunctionAddress_ & 0b11) == 0; }
        [[nodiscard]] constexpr auto getSegmentSelector() const noexcept { return selector_; }
        [[nodiscard]] constexpr auto getFaultHandlerProcedureAddress() const noexcept { return handlerFunctionAddress_; }
        [[nodiscard]] constexpr auto getFaultHandlerProcedureNumber() const noexcept { return (handlerFunctionAddress_ & (~0b11)); }
    private:
        Ordinal handlerFunctionAddress_;
        SegmentSelector selector_;
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
    union [[gnu::packed]] Configuration {
        constexpr explicit Configuration(Ordinal value = 0) : raw(value) { }
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
using int128_t = signed __int128;
using uint128_t = unsigned __int128;
template<typename T, uint8_t B>
struct LargeNumberPackage {
    using BackingStore = T;
    using Self = LargeNumberPackage<BackingStore, B>;
    static_assert(B <= 128, "Cannot accept bit count greater than 128");
    static_assert(B > 0, "Bitwidth of zero is not allowed!");
    constexpr LargeNumberPackage(BackingStore backing) : value_(backing) { }
    constexpr LargeNumberPackage(const Self& other) = default;
    constexpr LargeNumberPackage(Self&& other) = default;
    auto operator<=>(const Self& other) const noexcept = default;
    Self& operator+=(const Self& other) {
        value_ += other.value_;
        return *this;
    }
    friend Self operator+(Self lhs, const Self& rhs) {
        lhs += rhs;
        return lhs;
    }
    Self& operator-=(const Self& other) {
        value_ -= other.value_;
        return *this;
    }
    friend Self operator-(Self lhs, const Self& rhs) {
        lhs -= rhs;
        return lhs;
    }
    Self& operator*=(const Self& other) {
        value_ *= other.value_;
        return *this;
    }
    friend Self operator*(Self lhs, const Self& rhs) {
        lhs *= rhs;
        return lhs;
    }
    Self& operator%=(const Self& other) {
        value_ %= other.value_;
        return *this;
    }
    friend Self operator%(Self lhs, const Self& rhs) {
        lhs %= rhs;
        return lhs;
    }
    Self& operator/=(const Self& other) {
        value_ /= other.value_;
        return *this;
    }
    friend Self operator/(Self lhs, const Self& rhs) {
        lhs /= rhs;
        return lhs;
    }
    Self& operator&=(const Self& other) {
        value_ &= other.value_;
        return *this;
    }
    friend Self operator&(Self lhs, const Self& rhs) {
        lhs &= rhs;
        return lhs;
    }
    Self& operator|=(const Self& other) {
        value_ &= other.value_;
        return *this;
    }
    friend Self operator|(Self lhs, const Self& rhs) {
        lhs &= rhs;
        return lhs;
    }
    Self& operator^=(const Self& other) {
        value_ ^= other.value_;
        return *this;
    }
    friend Self operator^(Self lhs, const Self& rhs) {
        lhs ^= rhs;
        return lhs;
    }
    Self& operator>>=(const Self& other) {
        value_ >>= other.value_;
        return *this;
    }
    friend Self operator>>(Self lhs, const Self& rhs) {
        lhs >>= rhs;
        return lhs;
    }
    Self& operator<<=(const Self& other) {
        value_ <<= other.value_;
        return *this;
    }
    friend Self operator<<(Self lhs, const Self& rhs) {
        lhs <<= rhs;
        return lhs;
    }
    Self& operator~() noexcept {
        value_ = ~value_;
        return *this;
    }
    Self& operator+() noexcept {
        value_ = +value_;
        return *this;
    }
    Self& operator-() noexcept {
        value_ = -value_;
        return *this;
    }
    Self& operator++() {
        // prefix
        ++value_;
        return *this;
    }
    Self operator++(int) {
        // postfix
        Self old = *this;
        operator++();
        return old;
    }
    Self& operator--() {
        // prefix
        --value_;
        return *this;
    }
    Self operator--(int) {
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
using LargeIntPackage = std::conditional_t<(B == 128), int128_t, LargeNumberPackage<int128_t, B>>;
template<uint8_t B>
using LargeUIntPackage = std::conditional_t<(B == 128), uint128_t, LargeNumberPackage<uint128_t, B>>;
using int96_t = LargeIntPackage<96>;
using uint96_t = LargeUIntPackage<96>;
using TripleOrdinal = uint96_t;
using TripleInteger = int96_t;
static_assert(TripleOrdinal{0} != TripleOrdinal{1});
static_assert(TripleInteger{0} != TripleInteger{1});
static_assert(TripleInteger{0} > TripleInteger{-1});
static_assert(static_cast<Integer>(TripleInteger{0xFDED}) == Integer{0xFDED});
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

#endif // end SIM5_TYPES_H__
