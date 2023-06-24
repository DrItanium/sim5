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
    // this instruction does no modification but instead just acts as a transfer of bits

    auto internalOperation = [](TripleRegister& src, TripleRegister& dest) noexcept {
        dest.setValue<Ordinal>(0, src.getValue<Ordinal>(0));
        dest.setValue<Ordinal>(1, src.getValue<Ordinal>(1));
        dest.setValue<Ordinal>(2, src.getValue<Ordinal>(2) & 0x0000'FFFF);
    };
    FloatingPointRegister src;
    src.floatValue = 0.0;
    if (inst.getM1()) {
        // it is a floating point operation of some kind
        switch (inst.getSrc1()) {
            case 0b00000: // fp0
                src.floatValue = fp.get(0, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00001: // fp1
                src.floatValue = fp.get(4, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00010: // fp2
                src.floatValue = fp.get(8, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00011: // fp3
                src.floatValue = fp.get(12, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b10000: // +0.0
                src.floatValue = +0.0;
                break;
            case 0b10110: // +1.0
                src.floatValue = +1.0;
                break;
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        // okay so it is a GPR
        src.components[0] = getGPRValue<Ordinal>(inst.getSrc1());
        src.components[1] = getGPRValue<Ordinal>(inst.getSrc1() + 1);
        src.components[2] = getGPRValue<Ordinal>(inst.getSrc1() + 2); // mask out the upper 16-bits
    }

    // okay now checkout the destination as well
    if (inst.getM3()) {
        // fp
        switch (inst.getSrcDest()) {
            case 0b00000: // fp0
            {
                auto& tgt = fp.get(0, TreatAsTripleRegister{});
                tgt.setValue<Ordinal>(0, src.components[0]);
                tgt.setValue<Ordinal>(1, src.components[1]);
                tgt.setValue<Ordinal>(2, src.components[2] & 0x0000'FFFF);
                break;
            }
            case 0b00001: // fp1
            {
                auto& tgt = fp.get(4, TreatAsTripleRegister{});
                tgt.setValue<Ordinal>(0, src.components[0]);
                tgt.setValue<Ordinal>(1, src.components[1]);
                tgt.setValue<Ordinal>(2, src.components[2] & 0x0000'FFFF);
                break;
            }
            case 0b00010: // fp2
            {
                auto& tgt = fp.get(8, TreatAsTripleRegister{});
                tgt.setValue<Ordinal>(0, src.components[0]);
                tgt.setValue<Ordinal>(1, src.components[1]);
                tgt.setValue<Ordinal>(2, src.components[2] & 0x0000'FFFF);
                break;
            }
            case 0b00011: // fp3
            {
                auto& tgt = fp.get(12, TreatAsTripleRegister{});
                tgt.setValue<Ordinal>(0, src.components[0]);
                tgt.setValue<Ordinal>(1, src.components[1]);
                tgt.setValue<Ordinal>(2, src.components[2] & 0x0000'FFFF);
                break;
            }
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        // gpr
        auto& tgt = getGPR(inst.getSrcDest(), TreatAsTripleRegister{});
        tgt.setValue<Ordinal>(0, src.components[0]);
        tgt.setValue<Ordinal>(1, src.components[1]);
        tgt.setValue<Ordinal>(2, src.components[2] & 0x0000'FFFF);
    }
}
