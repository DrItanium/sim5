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
#ifdef CORE_TEENSY
#ifdef ARDUINO_TEENSY41
void 
Core::nonPortableBegin() noexcept {
    //pinMode(LOCKPIN, OUTPUT);
    //pinMode(FAILPIN, OUTPUT);
    //pinMode(INTPIN, INPUT);
    //pinMode(BUSYPIN, INPUT);
    //pinMode(Address15, OUTPUT); // A15
    //// Ports K and F are address lines
    //DDRF = 0xFF; // A23:16
    //DDRK = 0xFF; // A31:24
    //// setup the external bus interface and other features too!
}




void 
Core::lockBus() noexcept {
    //digitalWrite(LOCKPIN, LOW);
}

void 
Core::unlockBus() noexcept {
    //digitalWrite(LOCKPIN, HIGH);
}


bool 
Core::runNonPortableSelfTests() noexcept {
    return true;
}

void 
Core::assertFailureState() noexcept {
    //digitalWrite(FAILPIN, LOW);
}

void 
Core::deassertFailureState() noexcept {
    //digitalWrite(FAILPIN, HIGH);
}

void
Core::purgeInstructionCache() noexcept {
    ///@todo implement when we have an instruction cache!
}

Ordinal 
Core::load(Address address, TreatAsOrdinal) const noexcept {
    return 0;
}

Integer 
Core::load(Address address, TreatAsInteger) const noexcept {
    return 0;
}

ShortOrdinal 
Core::load(Address address, TreatAsShortOrdinal) const noexcept {
    return 0;
}

ShortInteger 
Core::load(Address address, TreatAsShortInteger) const noexcept {
    return 0;
}

ByteOrdinal 
Core::load(Address address, TreatAsByteOrdinal) const noexcept {
    return 0;
}

ByteInteger 
Core::load(Address address, TreatAsByteInteger) const noexcept {
    return 0;
}

void 
Core::store(Address address, Ordinal value, TreatAsOrdinal) noexcept {
}

void
Core::store(Address address, Integer value, TreatAsInteger) noexcept {
}

void
Core::store(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept {
}

void
Core::store(Address address, ShortInteger value, TreatAsShortInteger) noexcept {
}

void
Core::store(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept {
}

void
Core::store(Address address, ByteInteger value, TreatAsByteInteger) noexcept {
}
#endif
#endif

