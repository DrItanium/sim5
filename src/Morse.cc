// sim5
// Copyright (c) 2023, Joshua Scoggins
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
#include "Morse.h"

namespace morse {
    namespace {
        constexpr auto UnitDuration = 100;
        constexpr auto DotDuration = UnitDuration;
        constexpr auto DashDuration = UnitDuration * 3;
        constexpr auto LetterDuration = UnitDuration * 3;
        constexpr auto WordDuration = UnitDuration * 7;
    }
    void letter(const std::string& v) noexcept;
    void 
    dot() noexcept {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(DotDuration);
        digitalWrite(LED_BUILTIN, LOW);
        delay(UnitDuration);
    }
    void
    dash() noexcept {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(DashDuration);
        digitalWrite(LED_BUILTIN, LOW);
        delay(UnitDuration);
    }
    void message(const std::string& str) noexcept {
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
    letter(const std::string& letter) noexcept {
        message(letter);
        delay(LetterDuration);
    }
}
