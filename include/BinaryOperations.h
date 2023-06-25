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

#ifndef SIM5_BINARY_OPERATIONS_H__
#define SIM5_BINARY_OPERATIONS_H__

#include "Types.h"
#include <bit>
#include <cmath>
#include <concepts>
template<typename T>
concept CanPerformAdd = requires (T a, T b) {
    a + b; // can add two things together
};
template<typename T>
concept CanPerformSubtract = requires (T a, T b) {
    a - b;
};
template<typename T>
concept CanPerformMultiply = requires (T a, T b) { a * b; };

template<typename T>
concept CanPerformAnd = requires (T a, T b) { a & b; };
template<typename T>
concept CanPerformOr = requires (T a, T b) { a | b; };
template<typename T>
concept CanPerformXor = requires (T a, T b) { a ^ b; };

template<typename T>
concept CanPerformNot = requires (T a) { ~a; };

template<typename T>
concept Is960Comparable = std::three_way_comparable<T> || requires (T a, T b) {
    { a < b };
    { a == b };
};
[[nodiscard]] constexpr Ordinal rotateOperation(Ordinal src, Ordinal length) noexcept {
    // the implementations are equivalent
#if 0
    return (src << length)  | (src >> ((-length) & 31u));
#else
    return std::rotl<Ordinal>(src, length);
#endif
}


[[nodiscard]] constexpr Ordinal computeBitPosition(Ordinal value) noexcept {
    constexpr Ordinal positionTable[32] {
            static_cast<Ordinal>(1ul << 0ul),
            static_cast<Ordinal>(1ul << 1ul),
            static_cast<Ordinal>(1ul << 2ul),
            static_cast<Ordinal>(1ul << 3ul),
            static_cast<Ordinal>(1ul << 4ul),
            static_cast<Ordinal>(1ul << 5ul),
            static_cast<Ordinal>(1ul << 6ul),
            static_cast<Ordinal>(1ul << 7ul),
            static_cast<Ordinal>(1ul << 8ul),
            static_cast<Ordinal>(1ul << 9ul),
            static_cast<Ordinal>(1ul << 10ul),
            static_cast<Ordinal>(1ul << 11ul),
            static_cast<Ordinal>(1ul << 12ul),
            static_cast<Ordinal>(1ul << 13ul),
            static_cast<Ordinal>(1ul << 14ul),
            static_cast<Ordinal>(1ul << 15ul),
            static_cast<Ordinal>(1ul << 16ul),
            static_cast<Ordinal>(1ul << 17ul),
            static_cast<Ordinal>(1ul << 18ul),
            static_cast<Ordinal>(1ul << 19ul),
            static_cast<Ordinal>(1ul << 20ul),
            static_cast<Ordinal>(1ul << 21ul),
            static_cast<Ordinal>(1ul << 22ul),
            static_cast<Ordinal>(1ul << 23ul),
            static_cast<Ordinal>(1ul << 24ul),
            static_cast<Ordinal>(1ul << 25ul),
            static_cast<Ordinal>(1ul << 26ul),
            static_cast<Ordinal>(1ul << 27ul),
            static_cast<Ordinal>(1ul << 28ul),
            static_cast<Ordinal>(1ul << 29ul),
            static_cast<Ordinal>(1ul << 30ul),
            static_cast<Ordinal>(1ul << 31ul),
    };
    return positionTable[static_cast<uint8_t>(value) & 0b11111];
}

[[nodiscard]] constexpr int countLeadingZeros(Ordinal value) noexcept {
    return std::countl_zero(value);
}

[[nodiscard]] constexpr int highestOne(Ordinal value) noexcept {
    return 31 - countLeadingZeros(value);
}
#define X(index) static_assert(highestOne(0xFFFF'FFFF >> (31 - index)) == index)
X(31);
X(30);
X(29);
X(28);
X(27);
X(26);
X(25);
X(24);
X(23);
X(22);
X(21);
X(20);
X(19);
X(18);
X(17);
X(16);
X(15);
X(14);
X(13);
X(12);
X(11);
X(10);
X(9);
X(8);
X(7);
X(6);
X(5);
X(4);
X(3);
X(2);
X(1);
X(0);
#undef X

[[nodiscard]] constexpr Ordinal modify(Ordinal mask, Ordinal src, Ordinal srcDest) noexcept {
    return (src & mask) | (srcDest & ~mask);
}

template<typename T, bool invertResult = false>
requires CanPerformOr<T> && CanPerformNot<T>
[[nodiscard]] constexpr T orOperation(T a, T b) noexcept {
    if constexpr (invertResult) {
        return ~(a | b);
    } else {
        return a | b;
    }
}
template<typename T, bool invertResult = false>
requires CanPerformAnd<T> && CanPerformNot<T>
[[nodiscard]] constexpr T andOperation(T a, T b) noexcept {
    if constexpr (invertResult) {
        return ~(a & b);
    } else {
        return a & b;
    }
}
template<typename T, bool invertResult = false>
requires CanPerformXor<T> && CanPerformNot<T>
[[nodiscard]] constexpr T xorOperation(T a, T b) noexcept {
    if constexpr (invertResult) {
        return ~(a ^ b);
    } else {
        return a ^ b;
    }
}

template<typename T>
requires CanPerformAdd<T>
[[nodiscard]] constexpr T addOperation(T a, T b) noexcept {
    return a + b;
}

template<typename T>
requires CanPerformSubtract<T>
[[nodiscard]] constexpr T subOperation(T a, T b) noexcept {
    return a - b;
}

template<typename T>
requires CanPerformMultiply<T>
[[nodiscard]] constexpr T multiplyOperation(T a, T b) noexcept {
    return a * b;
}

template<typename In, typename Out>
[[nodiscard]] constexpr Out decode(In input, In mask, In shift = static_cast<Out>(0)) noexcept {
    auto maskedValue = (input & mask);
    maskedValue >>= shift;
    return static_cast<Out>(maskedValue);
}

template<typename In, typename Out, In mask, In shift>
[[nodiscard]] constexpr Out decode(In input) noexcept {
    auto maskedValue = (input & mask);
    maskedValue >>= shift;
    return static_cast<Out>(maskedValue);
}

template<typename In, typename Out>
[[nodiscard]] constexpr Out encode(Out container, In value, Out mask, Out shift = static_cast<Out>(0)) noexcept {
    auto preservedBitsOnly = container & ~mask;
    auto valueToInsert = static_cast<Out>(value) << shift;
    valueToInsert &= mask;
    return static_cast<Out>(preservedBitsOnly | valueToInsert);
}

template<typename In, typename Out, Out mask, Out shift = static_cast<Out>(0)>
[[nodiscard]] constexpr Out encode(Out container, In value) noexcept {
    auto preservedBitsOnly = container & ~mask;
    auto valueToInsert = static_cast<Out>(value) << shift;
    valueToInsert &= mask;
    return static_cast<Out>(preservedBitsOnly | valueToInsert);
}


template<typename T, T mask>
[[nodiscard]] constexpr T maskValue(T value) noexcept {
    return value & mask;
}
template<typename T>
[[nodiscard]] constexpr T maskValue(T value, T mask) noexcept {
    return value & mask;
}
[[nodiscard]] constexpr uint8_t mostSignificantByte(Ordinal input) noexcept {
    return decode<Ordinal, uint8_t, 0xFF00'0000, 24>(input);
}
[[nodiscard]] constexpr uint16_t lowerHalf(Ordinal input) noexcept {
    return static_cast<uint16_t>(input);
}
[[nodiscard]] constexpr uint16_t upperHalf(Ordinal input) noexcept {
    return static_cast<uint16_t>(input >> 16);
}
[[nodiscard]] constexpr Ordinal mostSignificantBit(Ordinal input) noexcept {
    return maskValue<Ordinal, computeBitPosition(31)>(input);
}
static_assert(lowerHalf(0xFDEDABCD) == 0xABCD);
static_assert(upperHalf(0xFDEDABCD) == 0xFDED);
static_assert(mostSignificantBit(0xFDEDABCD) == computeBitPosition(31));
static_assert(mostSignificantByte(0xFDEDABCD) == 0xFD);

template<Ordinal C, Ordinal NotC>
constexpr Ordinal computeNextFrame(Ordinal base) noexcept {
    return (base + C) & NotC;
}
template<typename Q>
requires Is960Comparable<Q>
constexpr ByteOrdinal performCompare(Q src1, Q src2) noexcept {
    if (src1 < src2) {
        return 0b100;
    } else if (src1 == src2) {
        return 0b010;
    } else {
        if constexpr (std::is_floating_point_v<Q>) {
            // unordered checks, one or both are NaN if the greater than check fails
           if (src1 > src2)  {
               return 0b001;
           } else {
               return 0b000;
           }
        } else {
            return 0b001;
        }
    }
}
static_assert(performCompare<Ordinal>(0u, 1u) == 0b100);
static_assert(performCompare<Integer>(-1, 0) == 0b100);
static_assert(performCompare<Integer>(0, 0) == 0b010);
static_assert(performCompare<Ordinal>(0, 0) == 0b010);
static_assert(performCompare<Integer>(0, -1) == 0b001);
static_assert(performCompare<Ordinal>(20, 10) == 0b001);

template<typename Q>
constexpr bool isLessThan(Q src1, Q src2) noexcept {
    return performCompare<Q>(src1, src2) == 0b100;
}
static_assert(isLessThan<Ordinal>(0u, 1u));
static_assert(isLessThan<Integer>(-1, 0));
template<typename Q>
constexpr bool isEqual(Q src1, Q src2) noexcept {
    return performCompare<Q>(src1, src2) == 0b010;
}
static_assert(isEqual<Ordinal>(0, 0));
static_assert(isEqual<Integer>(0, 0));
static_assert(isEqual<Integer>(-1, -1));
template<typename Q>
constexpr bool isNotEqual(Q src1, Q src2) noexcept {
    return (performCompare<Q>(src1, src2) & 0b010) == 0;
}
static_assert(isNotEqual<Ordinal>(0, 1));
static_assert(isNotEqual<Integer>(0, 2));
static_assert(!isNotEqual<Integer>(-1, -1));

template<typename Q>
constexpr bool isGreaterThan(Q src1, Q src2) noexcept {
    return performCompare<Q>(src1, src2) == 0b001;
}

static_assert(isGreaterThan<Ordinal>(20, 10));
static_assert(isGreaterThan<Integer>(0, -1));

template<typename Q>
constexpr bool isGreaterThanOrEqualTo(Q src1, Q src2) noexcept {
    auto value = performCompare<Q>(src1, src2);
    return (value & 0b011) != 0;
}

static_assert(isGreaterThanOrEqualTo<Ordinal>(20, 10));
static_assert(isGreaterThanOrEqualTo<Integer>(0, -1));
static_assert(isGreaterThanOrEqualTo<Ordinal>(0, 0));
static_assert(isGreaterThanOrEqualTo<Integer>(0, 0));
static_assert(isGreaterThanOrEqualTo<Integer>(-1, -1));

template<typename Q>
constexpr bool isLessThanOrEqualTo(Q src1, Q src2) noexcept {
    auto value = performCompare<Q>(src1, src2);
    return (value & 0b110) != 0;
}
static_assert(isLessThanOrEqualTo<Ordinal>(0u, 1u));
static_assert(isLessThanOrEqualTo<Integer>(-1, 0));
static_assert(isLessThanOrEqualTo<Ordinal>(0, 0));
static_assert(isLessThanOrEqualTo<Integer>(0, 0));
static_assert(isLessThanOrEqualTo<Integer>(-1, -1));
static_assert(isLessThanOrEqualTo<Real>(1.0, 2.0));


#endif // end SIM5_BINARY_OPERATIONS_H__
