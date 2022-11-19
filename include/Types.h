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
void set328BusAddress(const SplitWord32& address) noexcept;
void setInternalBusAddress(const SplitWord32& address) noexcept;
#endif // end SIM5_TYPES_H__
