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

#include <Arduino.h>
#include "Types.h"
constexpr Ordinal rotateOperation(Ordinal src, Ordinal length) noexcept {
    return (src << length)  | (src >> ((-length) & 31u));
}

constexpr Ordinal computeBitPosition(Ordinal value) noexcept {
    return static_cast<Ordinal>(1u) << static_cast<Ordinal>(value);
}

constexpr Ordinal modify(Ordinal mask, Ordinal src, Ordinal srcDest) noexcept {
    return (src & mask) | (srcDest & ~mask);
}

template<typename T, bool invertResult = false>
constexpr T orOperation(T a, T b) noexcept {
    if constexpr (invertResult) {
        return ~(a | b);
    } else {
        return a | b;
    }
}
template<typename T, bool invertResult = false>
constexpr T andOperation(T a, T b) noexcept {
    if constexpr (invertResult) {
        return ~(a & b);
    } else {
        return a & b;
    }
}
template<typename T, bool invertResult = false>
constexpr T xorOperation(T a, T b) noexcept {
    if constexpr (invertResult) {
        return ~(a ^ b);
    } else {
        return a ^ b;
    }
}

template<typename T>
constexpr T addOperation(T a, T b) noexcept {
    return a + b;
}

template<typename T>
constexpr T subOperation(T a, T b) noexcept {
    return a - b;
}

template<typename T>
constexpr T multiplyOperation(T a, T b) noexcept {
    return a * b;
}

template<typename In, typename Out>
constexpr Out decode(In input, In mask, In shift = static_cast<Out>(0)) noexcept {
    auto maskedValue = (input & mask);
    maskedValue >>= shift;
    return static_cast<Out>(maskedValue);
}

template<typename In, typename Out, In mask, In shift>
constexpr Out decode(In input) noexcept {
    auto maskedValue = (input & mask);
    maskedValue >>= shift;
    return static_cast<Out>(maskedValue);
}

template<typename In, typename Out>
constexpr Out encode(Out container, In value, Out mask, Out shift = static_cast<Out>(0)) noexcept {
    auto preservedBitsOnly = container & ~mask;
    auto valueToInsert = static_cast<Out>(value) << shift;
    valueToInsert &= mask;
    return static_cast<Out>(preservedBitsOnly | valueToInsert);
}

template<typename In, typename Out, Out mask, Out shift = static_cast<Out>(0)>
constexpr Out encode(Out container, In value) noexcept {
    auto preservedBitsOnly = container & ~mask;
    auto valueToInsert = static_cast<Out>(value) << shift;
    valueToInsert &= mask;
    return static_cast<Out>(preservedBitsOnly | valueToInsert);
}


template<typename T, T mask>
constexpr T mask(T value) noexcept {
    return value & mask;
}
template<typename T>
constexpr T mask(T value, T mask) noexcept {
    return value & mask;
}

constexpr Ordinal mostSignificantBit(Ordinal input) noexcept {
    return mask<Ordinal, 0x8000'0000>(input);
}

#endif // end SIM5_BINARY_OPERATIONS_H__
