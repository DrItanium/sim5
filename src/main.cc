// sim3
// Copyright (c) 2021, Joshua Scoggins
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

// The main for the arduino version of the simulator's main
// Created by jwscoggins on 8/21/21.
//
#ifdef ARDUINO
#include "Core.h"
#include <Arduino.h>
void 
setBankAddressRegisters(Address address) noexcept {

}
void
Core::begin() noexcept {

}
void
Core::lock() {

}

void
Core::unlock() {

}
Ordinal
Core::load32(Address address) noexcept {
    setBankAddressRegisters(address);
    return 0;
}
LongOrdinal
Core::load64(Address address) noexcept {
    setBankAddressRegisters(address);
    return 0;
}
ShortOrdinal
Core::load16(Address address) noexcept {
    setBankAddressRegisters(address);
    return 0;
}
ByteOrdinal
Core::load8(Address address) noexcept {
    setBankAddressRegisters(address);
    return 0;
}

void
Core::load96(Address address, TripleRegister& reg) noexcept {
    /// @todo implement
    setBankAddressRegisters(address);
}

void
Core::load128(Address address, QuadRegister& reg) noexcept {
    /// @todo implement
    setBankAddressRegisters(address);
}

void
Core::store8(Address address, ByteOrdinal value) noexcept {
    setBankAddressRegisters(address);
}

void
Core::store16(Address address, ShortOrdinal value) noexcept {
    setBankAddressRegisters(address);

}

void
Core::store32(Address address, Ordinal value) noexcept {
    setBankAddressRegisters(address);
}

void
Core::store64(Address address, LongOrdinal value) noexcept {
    setBankAddressRegisters(address);

}
void
Core::store96(Address address, const TripleRegister& reg) noexcept {
    setBankAddressRegisters(address);

}
void
Core::store128(Address address, const QuadRegister& reg) noexcept {
    setBankAddressRegisters(address);

}
// make it an Sx core with SALIGN of 4
Core theCore(4);
void setup() {
    Serial.begin(115200);
    while (!Serial);
    theCore.begin();

    theCore.boot();
}
void loop() {
    theCore.cycle();
    delay(1000);
}
#if __cplusplus >= 201402L
#ifdef ARDUINO_AVR_MEGA2560

void operator delete(void * ptr, size_t)
{
    ::operator delete(ptr);
}


void operator delete[](void * ptr, size_t)
{
    ::operator delete(ptr);
}
#endif
#endif // end language is C++14 or greater
#endif

