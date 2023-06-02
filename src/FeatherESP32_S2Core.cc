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
#include <Arduino.h>
#include "Core.h"
#ifdef ESP32
#ifdef ARDUINO_FEATHERS2
namespace {
    uint8_t* psMemory = nullptr;
}
constexpr auto LOCKPIN = 33;
constexpr auto INTPIN = 38;
constexpr auto BUSYPIN = 1;

constexpr auto FAILPIN = 3;
constexpr auto LEDPin = 13;
constexpr auto AmbientLightSensor = 4;
void 
Core::nonPortableBegin() noexcept {
    pinMode(LOCKPIN, OUTPUT);
    pinMode(FAILPIN, OUTPUT);
    pinMode(INTPIN, INPUT);
    pinMode(BUSYPIN, INPUT);
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, LOW);

    if (psramInit()) {
        // since we have access to 8 megabytes of PSRAM on this board, we need
        // to allocate the memory space entirely and keep it around
        psMemory = (uint8_t*)ps_calloc(8 * 1024 * 1024, sizeof(uint8_t));
    }
}




void 
Core::lockBus() noexcept {
    digitalWrite(LOCKPIN, LOW);
}

void 
Core::unlockBus() noexcept {
    digitalWrite(LOCKPIN, HIGH);
}


bool 
Core::runNonPortableSelfTests() noexcept {
    return psMemory != nullptr;
}

void 
Core::assertFailureState() noexcept {
    digitalWrite(FAILPIN, LOW);
}

void 
Core::deassertFailureState() noexcept {
    digitalWrite(FAILPIN, HIGH);
}

void
Core::purgeInstructionCache() noexcept {
    ///@todo implement when we have an instruction cache!
}

template<typename T>
T load(Address address) noexcept {
    switch (static_cast<uint8_t>(address >> 24)) {
        case 0x00:
            return *reinterpret_cast<volatile T*>(psMemory[address & 0x007F'FFFF]);
            /// @todo add support for IO and other memory spaces as well!
        default:
            return 0;
    }
}

template<typename T>
void store(Address address, T value) noexcept {
    switch (static_cast<uint8_t>(address >> 24)) {
        case 0x00:
            *reinterpret_cast<volatile T*>(psMemory[address & 0x007F'FFFF]) = value;
            break;
            /// @todo add support for IO and other memory spaces as well!
        default:
            break;
    }
}

Ordinal 
Core::load(Address address, TreatAsOrdinal) const noexcept {
    // we have 8 megabytes total of memory, just do a mirroring operation for
    // now!
    // don't check since the self tests will cancel startup if it turns out
    // that we don't have PSRAM available
    return ::load<Ordinal>(address);
}

Integer 
Core::load(Address address, TreatAsInteger) const noexcept {
    return ::load<Integer>(address);
}

ShortOrdinal 
Core::load(Address address, TreatAsShortOrdinal) const noexcept {
    return ::load<ShortOrdinal>(address);
}

ShortInteger 
Core::load(Address address, TreatAsShortInteger) const noexcept {
    return ::load<ShortInteger>(address);
}

ByteOrdinal 
Core::load(Address address, TreatAsByteOrdinal) const noexcept {
    return ::load<ByteOrdinal>(address);
}

ByteInteger 
Core::load(Address address, TreatAsByteInteger) const noexcept {
    return ::load<ByteInteger>(address);
}

void 
Core::store(Address address, Ordinal value, TreatAsOrdinal) noexcept {
    ::store<decltype(value)>(address, value);
}

void
Core::store(Address address, Integer value, TreatAsInteger) noexcept {
    ::store<decltype(value)>(address, value);
}

void
Core::store(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept {
    ::store<decltype(value)>(address, value);
}

void
Core::store(Address address, ShortInteger value, TreatAsShortInteger) noexcept {
    ::store<decltype(value)>(address, value);
}

void
Core::store(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept {
    ::store<decltype(value)>(address, value);
}

void
Core::store(Address address, ByteInteger value, TreatAsByteInteger) noexcept {
    ::store<decltype(value)>(address, value);
}
#endif /* defined ARDUINO_FEATHERS2 */
#endif /* defined ESP32 */

