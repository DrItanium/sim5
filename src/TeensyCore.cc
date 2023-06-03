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
#ifdef CORE_TEENSY
#ifdef ARDUINO_TEENSY41
#include <Arduino.h>
#include "Core.h"
#include <string>
#include "Morse.h"
#include <SD.h>
constexpr auto PSRAMMemorySize = 16 * 1024 * 1024;
EXTMEM volatile char memoryBuffer[PSRAMMemorySize]; // 16 megabyte storage area
//constexpr auto LOCKPIN = 33;
constexpr auto FAILPIN = 36;
constexpr auto LEDPin = LED_BUILTIN;
void 
Core::nonPortableBegin() noexcept {
    //pinMode(LOCKPIN, OUTPUT);
    pinMode(FAILPIN, OUTPUT);
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, LOW);
    // clear main memory
    Serial.println("Testing 16-megabytes of PSRAM");
    for (int i = 0; i < PSRAMMemorySize; ++i) {
        digitalWrite(LEDPin, LOW);
        auto j = static_cast<uint8_t>(i);
        memoryBuffer[i] = j;
        digitalWrite(LEDPin, HIGH);
        if (memoryBuffer[i] != j) {
            while (true) {
                delay(1000);
            }
        }
    }
    digitalWrite(LEDPin, LOW);
    volatile uint32_t* buf32= reinterpret_cast<volatile uint32_t*>(memoryBuffer);
    Serial.println("Testing PSRAM in 32-bit memory mode");
    for (int i = 0; i < PSRAMMemorySize / sizeof(uint32_t); ++i) {
        digitalWrite(LEDPin, LOW);
        uint32_t value = random();
        buf32[i] = value;
        digitalWrite(LEDPin, HIGH);
        if (buf32[i] != value) {
            while (true) {
                delay(1000);
            }
        }
    }
    digitalWrite(LEDPin, LOW);
    Serial.println("PSRAM Test Successful!");
    while (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD Card not found...waiting");
        delay(1000);
    }
    Serial.println("SD Card Found!");
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
#endif /* defined ARDUINO_TEENSY41 */
#endif /* defined TEENSY_CORE */

