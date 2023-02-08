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

#include <Arduino.h>
#include "Types.h"
#include "Core.h"
#include "Peripherals.h"

namespace ebi {

    void
    set328BusAddress(const SplitWord32& address) noexcept {
        // set the upper
        PORTF = address.splitAddress.a24_31;
        PORTK = address.splitAddress.a16_23;
        digitalWrite(38, address.splitAddress.a15);
    }

    void
    setInternalBusAddress(const SplitWord32& address) noexcept {
        digitalWrite(BANK0, address.internalBankAddress.bank0);
        digitalWrite(BANK1, address.internalBankAddress.bank1);
        digitalWrite(BANK2, address.internalBankAddress.bank2);
        digitalWrite(BANK3, address.internalBankAddress.bank3);
    }
    void 
    begin() noexcept {
        Serial.println(F("Setting up EBI..."));
        // cleave the address space in half via sector limits.
        // lower half is io space for the implementation
        // upper half is the window into the 32/8 bus
        XMCRB = 0;           // No external memory bus keeper and full 64k address
                             // space
        XMCRA = 0b1000'0000; // Divide the 64k address space in half at 0x8000,
                             // full wait states on the upper half. 
                             // Also turn on the EBI
        set328BusAddress(0);
        setInternalBusAddress(0);
        volatile byte* ptr = reinterpret_cast<volatile byte*>(RAMEND+1);
        for (size_t i = (RAMEND + 1); i < 0x8000; ++i, ++ptr) {
            volatile byte value = random();
            *ptr = value;
            if (*ptr != value) {
                Serial.print(F("IBUS@0x"));
                Serial.print(i, HEX);
                Serial.print(F(", MISMATCH! E: 0x"));
                Serial.print(value, HEX);
                Serial.print(F(", G: 0x"));
                Serial.println(*ptr, HEX);
            }
        }
        constexpr uint32_t TwoMegs = 2ul * 1024ul * 1024ul;
        for (uint32_t i = 0 ; i < TwoMegs; ++i) {
            volatile ByteOrdinal value = random();
            store<ByteOrdinal>(i, value, TreatAsByteOrdinal{});
            volatile auto result = load<ByteOrdinal>(i, TreatAsByteOrdinal{});
            if (result != value) {
                Serial.print(F("XBUS@0x"));
                Serial.print(i, HEX);
                Serial.print(F(", MISMATCH! E: 0x"));
                Serial.print(value, HEX);
                Serial.print(F(", G: 0x"));
                Serial.println(*ptr, HEX);
            }
        }
        Serial.print(F("ADDR OF PTR: 0x"));
        Serial.println(reinterpret_cast<uintptr_t>(ptr), HEX);
        Serial.println(F("EBI Setup Complete!"));
    }

}

Integer Core::load(Address addr, TreatAsInteger) const { return ebi::load<Integer>(addr, TreatAsInteger{}); }
Ordinal Core::load(Address addr, TreatAsOrdinal) const { 
    return ebi::load<Ordinal>(addr, TreatAsOrdinal{}); 
}
ByteInteger Core::load(Address addr, TreatAsByteInteger) const { return ebi::load<ByteInteger>(addr, TreatAsByteInteger{}); }
ByteOrdinal Core::load(Address addr, TreatAsByteOrdinal) const { return ebi::load<ByteOrdinal>(addr, TreatAsByteOrdinal{}); }
ShortInteger Core::load(Address addr, TreatAsShortInteger) const { return ebi::load<ShortInteger>(addr, TreatAsShortInteger{}); }
ShortOrdinal Core::load(Address addr, TreatAsShortOrdinal) const { return ebi::load<ShortOrdinal>(addr, TreatAsShortOrdinal{}); }

void Core::store(Address addr, Integer value, TreatAsInteger) { ebi::store<Integer>(addr, value, TreatAsInteger{}); }
void Core::store(Address addr, Ordinal value, TreatAsOrdinal) { ebi::store<Ordinal>(addr, value, TreatAsOrdinal{}); }
void Core::store(Address addr, ByteInteger value, TreatAsByteInteger) { ebi::store<ByteInteger>(addr, value, TreatAsByteInteger{}); }
void Core::store(Address addr, ByteOrdinal value, TreatAsByteOrdinal) { ebi::store<ByteOrdinal>(addr, value, TreatAsByteOrdinal{}); }
void Core::store(Address addr, ShortInteger value, TreatAsShortInteger) { ebi::store<ShortInteger>(addr, value, TreatAsShortInteger{}); }
void Core::store(Address addr, ShortOrdinal value, TreatAsShortOrdinal) { ebi::store<ShortOrdinal>(addr, value, TreatAsShortOrdinal{}); }


