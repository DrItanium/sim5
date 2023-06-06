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
constexpr auto PSRAMMemorySize_Bytes = 16 * 1024 * 1024;
constexpr auto PSRAMMemorySize_Blocks = PSRAMMemorySize_Bytes / sizeof(SplitWord128);
EXTMEM volatile SplitWord128 memoryBuffer[PSRAMMemorySize_Blocks];
constexpr auto LEDPin = LED_BUILTIN;
void 
Core::nonPortableBegin() noexcept {
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, LOW);
    // clear main memory
    Serial.println("Testing 16-megabytes of PSRAM");
    for (uint32_t i = 0; i < PSRAMMemorySize_Blocks; ++i) {
        auto& currentBlock = memoryBuffer[i];
        for (uint32_t j = 0; j < 16; ++j) {
            digitalWrite(LEDPin, HIGH);
            auto k = static_cast<uint8_t>(random());
            currentBlock.bytes[j] = k;
            if (currentBlock.bytes[j] != k) {
                while (true) {
                    delay(1000);
                }
            }
            digitalWrite(LEDPin, LOW);
        }
        // test 32-bit values
        for (uint32_t j = 0; j < 4; ++j) {
            digitalWrite(LEDPin, HIGH);
            uint32_t k = static_cast<uint32_t>(random());
            currentBlock.words[j] = k;
            if (currentBlock.words[j] != k) {
                while (true) {
                    delay(1000);
                }
            }
            digitalWrite(LEDPin, LOW);
        }
    }
    digitalWrite(LEDPin, LOW);
#if 0
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
#endif
    Serial.println("PSRAM Test Successful!");
    while (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD Card not found...waiting");
        delay(1000);
    }
    Serial.println("SD Card Found!");
}




void 
Core::lockBus() noexcept {
}

void 
Core::unlockBus() noexcept {
}


bool 
Core::runNonPortableSelfTests() noexcept {
    return true;
}

void 
Core::assertFailureState() noexcept {
}

void 
Core::deassertFailureState() noexcept {
}

void
Core::purgeInstructionCache() noexcept {
    ///@todo implement when we have an instruction cache!
}

namespace {
Ordinal
psramLoad32(Address address, TreatAsOrdinal) noexcept {
    // allow unaligned loads
    return 0;
}
Ordinal
load32(Address address, TreatAsOrdinal) noexcept {
    switch (static_cast<uint8_t>(address) >> 24) {
        case 0x00: // PSRAM itself
            break;
        case 0xFE: // io space
            break;
        case 0xFF: // onboard devices
            break;
        default:
            return 0;
    }
}
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
void 
Core::checksumFail() {
    while(true) {
        // dot dot dot (S)
        morse::message("checksum failure");
        delay(1000);
    }
}
void 
Core::selfTestFailure() {
    while(true) {
        // dot dot dot (S)
        morse::message("self test failure");
        delay(1000);
    }
}
void 
Core::badFault(Ordinal faultCode) {
    while(true) {
        // dot dot dot (S)
        morse::message("bad fault");
        delay(1000);
    }
}
#endif /* defined ARDUINO_TEENSY41 */
#endif /* defined TEENSY_CORE */

