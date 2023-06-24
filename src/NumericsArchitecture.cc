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

#include "Types.h"
#include "Core.h"
#include "BinaryOperations.h"
#include <cmath>

void
Core::dmovt(Register& dest, Ordinal src) noexcept {
    dest.setValue<Ordinal>(src);
    if (auto decimal = static_cast<uint8_t>(src); decimal >= 0b00110000 && decimal <= 0b00111001) {
        ac_.arith.conditionCode = 0b000;
    } else {
        ac_.arith.conditionCode = 0b010;
    }
}

void
Core::classr(Real src) noexcept {
    switch(std::fpclassify(src)) {
        case FP_ZERO:
            ac_.arith.arithmeticStatus = 0;
            break;
        case FP_SUBNORMAL:
            ac_.arith.arithmeticStatus = 0b001;
            break;
        case FP_NORMAL:
            ac_.arith.arithmeticStatus = 0b010;
            break;
        case FP_INFINITE:
            ac_.arith.arithmeticStatus = 0b011;
            break;
        case FP_NAN:
            ac_.arith.arithmeticStatus = issignaling(src) ? 0b101 : 0b100;
            break;
        default:
            ac_.arith.arithmeticStatus = 0b110;
            break;
    }
}
void
Core::classrl(LongReal src) noexcept {
    switch(std::fpclassify(src)) {
        case FP_ZERO:
            ac_.arith.arithmeticStatus = 0;
            break;
        case FP_SUBNORMAL:
            ac_.arith.arithmeticStatus = 0b001;
            break;
        case FP_NORMAL:
            ac_.arith.arithmeticStatus = 0b010;
            break;
        case FP_INFINITE:
            ac_.arith.arithmeticStatus = 0b011;
            break;
        case FP_NAN:
            ac_.arith.arithmeticStatus = issignaling(src) ? 0b101 : 0b100;
            break;
        default:
            ac_.arith.arithmeticStatus = 0b110;
            break;
    }
}

void
Core::cmpr(Real src1, Real src2) noexcept {
    cmpGeneric<Real>(src1, src2);
}

void
Core::cmprl(LongReal src1, LongReal src2) noexcept {
    cmpGeneric<LongReal>(src1, src2);
}

void
Core::movr(const REGInstruction &inst) {
    // so the manual does not state the registers have to be aligned!
    unimplementedFault();
}

void
Core::movrl(const REGInstruction &inst) {
    unimplementedFault();
}

void
Core::movre(const REGInstruction &inst) {
    unimplementedFault();
}
