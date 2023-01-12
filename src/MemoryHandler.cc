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

namespace {
    template<typename T>
    T load(Address address, TreatAs<T>) noexcept {
        SplitWord32 split(address);
        set328BusAddress(split);
        return memory<T>(static_cast<size_t>(split.splitAddress.lower) + 0x8000);
    }
    template<typename T>
    void store(Address address, T value, TreatAs<T>) noexcept {
        SplitWord32 split(address);
        set328BusAddress(split);
        memory<T>(static_cast<size_t>(split.splitAddress.lower) + 0x8000) = value;
    }
}

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


Integer Core::load(Address addr, TreatAsInteger) const { return ::load<Integer>(addr, TreatAsInteger{}); }
Ordinal Core::load(Address addr, TreatAsOrdinal) const { return ::load<Ordinal>(addr, TreatAsOrdinal{}); }
ByteInteger Core::load(Address addr, TreatAsByteInteger) const { return ::load<ByteInteger>(addr, TreatAsByteInteger{}); }
ByteOrdinal Core::load(Address addr, TreatAsByteOrdinal) const { return ::load<ByteOrdinal>(addr, TreatAsByteOrdinal{}); }
ShortInteger Core::load(Address addr, TreatAsShortInteger) const { return ::load<ShortInteger>(addr, TreatAsShortInteger{}); }
ShortOrdinal Core::load(Address addr, TreatAsShortOrdinal) const { return ::load<ShortOrdinal>(addr, TreatAsShortOrdinal{}); }

void Core::store(Address addr, Integer value, TreatAsInteger) { ::store<Integer>(addr, value, TreatAsInteger{}); }
void Core::store(Address addr, Ordinal value, TreatAsOrdinal) { ::store<Ordinal>(addr, value, TreatAsOrdinal{}); }
void Core::store(Address addr, ByteInteger value, TreatAsByteInteger) { ::store<ByteInteger>(addr, value, TreatAsByteInteger{}); }
void Core::store(Address addr, ByteOrdinal value, TreatAsByteOrdinal) { ::store<ByteOrdinal>(addr, value, TreatAsByteOrdinal{}); }
void Core::store(Address addr, ShortInteger value, TreatAsShortInteger) { ::store<ShortInteger>(addr, value, TreatAsShortInteger{}); }
void Core::store(Address addr, ShortOrdinal value, TreatAsShortOrdinal) { ::store<ShortOrdinal>(addr, value, TreatAsShortOrdinal{}); }
