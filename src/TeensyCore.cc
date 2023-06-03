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
#include <string>
constexpr auto LOCKPIN = 33;
constexpr auto INTPIN = 34;
constexpr auto BUSYPIN = 35;

constexpr auto FAILPIN = 36;
constexpr auto LEDPin = LED_BUILTIN;
void 
Core::nonPortableBegin() noexcept {
    pinMode(LOCKPIN, OUTPUT);
    pinMode(FAILPIN, OUTPUT);
    pinMode(INTPIN, INPUT);
    pinMode(BUSYPIN, INPUT);
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, LOW);
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
namespace morse {
    constexpr auto UnitDuration = 100;
    constexpr auto DotDuration = UnitDuration;
    constexpr auto DashDuration = UnitDuration * 3;
    constexpr auto LetterDuration = UnitDuration * 3;
    constexpr auto WordDuration = UnitDuration * 7;
    void 
    dot() noexcept {
        digitalWrite(LEDPin, HIGH);
        delay(DotDuration);
        digitalWrite(LEDPin, LOW);
        delay(UnitDuration);
    }
    void
    dash() noexcept {
        digitalWrite(LEDPin, HIGH);
        delay(DashDuration);
        digitalWrite(LEDPin, LOW);
        delay(UnitDuration);
    }
    void parse (const std::string& str) noexcept;
    void letter( const std::string& sequence) noexcept;
    void parse(const std::string& str) noexcept {
        for (auto c : str) {
            switch (c) {
                case '.':
                    dot();
                    break;
                case '-':
                    dash();
                    break;
                case ' ': // word end
                    delay(WordDuration);
                    break;
                case 'A':
                case 'a':
                    letter(".-");
                    break;
                case 'B':
                case 'b':
                    letter("-...");
                    break;
                case 'C':
                case 'c':
                    letter("-.-.");
                    break;
                case 'D':
                case 'd':
                    letter("-..");
                    break;
                case 'E':
                case 'e':
                    letter(".");
                    break;
                case 'F':
                case 'f':
                    letter("..-.");
                    break;
                case 'G':
                case 'g':
                    letter("--.");
                    break;
                case 'H':
                case 'h':
                    letter("....");
                    break;
                case 'I':
                case 'i':
                    letter("..");
                    break;
                case 'J':
                case 'j':
                    letter(".---");
                    break;
                case 'K':
                case 'k':
                    letter("-.-");
                    break;
                case 'L':
                case 'l':
                    letter(".-..");
                    break;
                case 'M':
                case 'm':
                    letter("--");
                    break;
                case 'N':
                case 'n':
                    letter("-.");
                    break;
                case 'O':
                case 'o':
                    letter("---");
                    break;
                case 'P':
                case 'p':
                    letter(".--.");
                    break;
                case 'Q':
                case 'q':
                    letter("--.-");
                    break;
                case 'R':
                case 'r':
                    letter(".-.");
                    break;
                case 'S':
                case 's':
                    letter("...");
                    break;
                case 't':
                case 'T':
                    letter("-");
                    break;
                case 'u':
                case 'U':
                    letter("..-");
                    break;
                case 'V':
                case 'v':
                    letter("...-");
                    break;
                case 'W':
                case 'w':
                    letter(".--");
                    break;
                case 'X':
                case 'x':
                    letter("-..-");
                    break;
                case 'Y':
                case 'y':
                    letter("-.--");
                    break;
                case 'Z':
                case 'z':
                    letter("--..");
                    break;
                case '1':
                    letter(".----");
                    break;
                case '2':
                    letter("..---");
                    break;
                case '3':
                    letter("...--");
                    break;
                case '4':
                    letter("....-");
                    break;
                case '5':
                    letter(".....");
                    break;
                case '6':
                    letter("-....");
                    break;
                case '7':
                    letter("--...");
                    break;
                case '8':
                    letter("---..");
                    break;
                case '9':
                    letter("----.");
                    break;
                case '0':
                    letter("-----");
                    break;
                default:
                    break;
            }
        }
    }
    void 
    letter(const std::string& message) noexcept {
        parse(message);
        delay(LetterDuration);
    }
}
void 
Core::checksumFail() noexcept {
    while(true) {
        // dot dot dot (S)
        morse::parse("checksum failure");
        delay(1000);
    }
}
void 
Core::selfTestFailure() noexcept {
    while(true) {
        // dot dot dot (S)
        morse::parse("self test failure");
        delay(1000);
    }
}
#endif /* defined ARDUINO_TEENSY41 */
#endif /* defined TEENSY_CORE */

