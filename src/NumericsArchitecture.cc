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
    auto decimal = static_cast<uint8_t>(src) ;
    ac_.setConditionResult(decimal >= 0b00110000 && decimal <= 0b00111001);
}

void
Core::classr(const REGInstruction& inst) {
    Register src;
    if (inst.getM1()) {
        // it is a floating point operation of some kind
        switch (inst.getSrc1()) {
            case 0b00000: // fp0
                src.r= static_cast<Real>(fp.get(0, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00001: // fp1
                src.r = static_cast<Real>(fp.get(4, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00010: // fp2
                src.r = static_cast<Real>(fp.get(8, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00011: // fp3
                src.r = static_cast<Real>(fp.get(12, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b10000: // +0.0
                src.r = +0.0;
                break;
            case 0b10110: // +1.0
                src.r = +1.0;
                break;
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        // okay so it is a GPR
        src = getGPR(inst.getSrc1());
    }
    switch(std::fpclassify(src.r)) {
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
            ac_.arith.arithmeticStatus = issignaling(src.r) ? 0b101 : 0b100;
            break;
        default:
            ac_.arith.arithmeticStatus = 0b110;
            break;
    }
}
void
Core::classrl(const REGInstruction& inst) {
    LongReal src1 = 0.0;
    if (inst.getM1()) {
        // it is a floating point operation of some kind
        switch (inst.getSrc1()) {
            case 0b00000: // fp0
                src1 = static_cast<LongReal>(fp.get(0, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00001: // fp1
                src1 = static_cast<LongReal>(fp.get(4, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00010: // fp2
                src1 = static_cast<LongReal>(fp.get(8, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00011: // fp3
                src1 = static_cast<LongReal>(fp.get(12, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b10000: // +0.0
                src1 = +0.0;
                break;
            case 0b10110: // +1.0
                src1 = +1.0;
                break;
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        src1 = getGPR(inst.getSrc1(), TreatAsLongRegister{}).getValue<LongReal>();
    }
    switch(std::fpclassify(src1)) {
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
            ac_.arith.arithmeticStatus = issignaling(src1) ? 0b101 : 0b100;
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
    // this instruction does no modification but instead just acts as a transfer of bits
    Register src;
    if (inst.getM1()) {
        // it is a floating point operation of some kind
        switch (inst.getSrc1()) {
            case 0b00000: // fp0
                src.r= static_cast<Real>(fp.get(0, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00001: // fp1
                src.r = static_cast<Real>(fp.get(4, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00010: // fp2
                src.r = static_cast<Real>(fp.get(8, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00011: // fp3
                src.r = static_cast<Real>(fp.get(12, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b10000: // +0.0
                src.r = +0.0;
                break;
            case 0b10110: // +1.0
                src.r = +1.0;
                break;
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        // okay so it is a GPR
        src = getGPR(inst.getSrc1());
    }

    // okay now checkout the destination as well
    if (inst.getM3()) {
        // fp
        switch (inst.getSrcDest()) {
            case 0b00000: // fp0
            {
                auto& tgt = fp.get(0, TreatAsTripleRegister{});
                tgt.setValue(src.r, TreatAsExtendedReal{});
                break;
            }
            case 0b00001: // fp1
            {
                auto& tgt = fp.get(4, TreatAsTripleRegister{});
                tgt.setValue(src.r, TreatAsExtendedReal{});
                break;
            }
            case 0b00010: // fp2
            {
                auto& tgt = fp.get(8, TreatAsTripleRegister{});
                tgt.setValue(src.r, TreatAsExtendedReal{});
                break;
            }
            case 0b00011: // fp3
            {
                auto& tgt = fp.get(12, TreatAsTripleRegister{});
                tgt.setValue(src.r, TreatAsExtendedReal{});
                break;
            }
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        // gpr
        auto& tgt = getGPR(inst.getSrcDest(), TreatAsRegister{});
        tgt.setValue<Real>(src.r);
    }
    /// @todo implement floating point faults
}

void
Core::movrl(const REGInstruction &inst) {
    // this instruction does no modification but instead just acts as a transfer of bits
    union {
        double floatValue;
        Ordinal components[2];
    } src;
    src.floatValue = 0.0;
    if (inst.getM1()) {
        // it is a floating point operation of some kind
        switch (inst.getSrc1()) {
            case 0b00000: // fp0
                src.floatValue = static_cast<LongReal>(fp.get(0, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00001: // fp1
                src.floatValue = static_cast<LongReal>(fp.get(4, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00010: // fp2
                src.floatValue = static_cast<LongReal>(fp.get(8, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
                break;
            case 0b00011: // fp3
                src.floatValue = static_cast<LongReal>(fp.get(12, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{}));
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
    }

    // okay now checkout the destination as well
    if (inst.getM3()) {
        // fp
        switch (inst.getSrcDest()) {
            case 0b00000: // fp0
            {
                auto& tgt = fp.get(0, TreatAsTripleRegister{});
                tgt.setValue(src.floatValue, TreatAsExtendedReal{});
                break;
            }
            case 0b00001: // fp1
            {
                auto& tgt = fp.get(4, TreatAsTripleRegister{});
                tgt.setValue(src.floatValue, TreatAsExtendedReal{});
                break;
            }
            case 0b00010: // fp2
            {
                auto& tgt = fp.get(8, TreatAsTripleRegister{});
                tgt.setValue(src.floatValue, TreatAsExtendedReal{});
                break;
            }
            case 0b00011: // fp3
            {
                auto& tgt = fp.get(12, TreatAsTripleRegister{});
                tgt.setValue(src.floatValue, TreatAsExtendedReal{});
                break;
            }
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        /// @note unaligned register groups do not seem to be explicitly supported. My guess is that since the move
        /// and store operations are supposed to operate on aligned register packs there is no need to be that explicit.
        /// So I will also make that assumption as well.
        // gpr
        auto& tgt = getGPR(inst.getSrcDest(), TreatAsLongRegister{});
        tgt.setValue(src.floatValue, TreatAsLongReal{});
    }
    /// @todo implement floating point faults
}

void
Core::movre(const REGInstruction &inst) {
    // this instruction does no modification but instead just acts as a transfer of bits
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

void
Core::cpyrsre(const REGInstruction &inst) {

    /*
     * we transfer the sign bit from src2 into it
     * If src2 is negative then dst <- abs(src1)
     * else dst <- -abs(src1);
     * endif
     */
    ExtendedReal src1, src2;
    if (inst.getM1()) {
        // it is a floating point operation of some kind
        switch (inst.getSrc1()) {
            case 0b00000: // fp0
                src1 = fp.get(0, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00001: // fp1
                src1 = fp.get(4, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00010: // fp2
                src1 = fp.get(8, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00011: // fp3
                src1 = fp.get(12, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b10000: // +0.0
                src1 = +0.0;
                break;
            case 0b10110: // +1.0
                src1 = +1.0;
                break;
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        src1 = getGPR(inst.getSrc1(), TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
    }
    if (inst.getM2()) {
        // it is a floating point operation of some kind
        switch (inst.getSrc2()) {
            case 0b00000: // fp0
                src2 = fp.get(0, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00001: // fp1
                src2 = fp.get(4, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00010: // fp2
                src2 = fp.get(8, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00011: // fp3
                src2 = fp.get(12, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b10000: // +0.0
                src2 = +0.0;
                break;
            case 0b10110: // +1.0
                src2 = +1.0;
                break;
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        src2 = getGPR(inst.getSrc2(), TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
    }
    fpassignment(inst, std::signbit(src2) != 0 ? std::fabs(src1) : -std::fabs(src1), TreatAsExtendedReal{});
}

void
Core::cpysre(const REGInstruction &inst) {
    /*
     * we transfer the sign bit from src2 into it
     * If src2 is positive then dst <- abs(src1)
     * else dst <- -abs(src1);
     * endif
     */
    ExtendedReal src1, src2;
    if (inst.getM1()) {
        // it is a floating point operation of some kind
        switch (inst.getSrc1()) {
            case 0b00000: // fp0
                src1 = fp.get(0, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00001: // fp1
                src1 = fp.get(4, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00010: // fp2
                src1 = fp.get(8, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00011: // fp3
                src1 = fp.get(12, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b10000: // +0.0
                src1 = +0.0;
                break;
            case 0b10110: // +1.0
                src1 = +1.0;
                break;
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        src1 = getGPR(inst.getSrc1(), TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
    }
    if (inst.getM2()) {
        // it is a floating point operation of some kind
        switch (inst.getSrc2()) {
            case 0b00000: // fp0
                src2 = fp.get(0, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00001: // fp1
                src2 = fp.get(4, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00010: // fp2
                src2 = fp.get(8, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b00011: // fp3
                src2 = fp.get(12, TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
                break;
            case 0b10000: // +0.0
                src2 = +0.0;
                break;
            case 0b10110: // +1.0
                src2 = +1.0;
                break;
            default:
                invalidOpcodeFault();
                return;
        }
    } else {
        src2 = getGPR(inst.getSrc2(), TreatAsTripleRegister{}).getValue(TreatAsExtendedReal{});
    }
    fpassignment(inst, std::signbit(src2) == 0 ? std::fabs(src1) : -std::fabs(src1), TreatAsExtendedReal{});
}

void
Core::fpassignment(const REGInstruction &inst, ExtendedReal result, TreatAsExtendedReal) {
    if(inst.getM3()) {
        // fp
        switch (inst.getSrcDest()) {
            case 0b00000: // fp0
            {
                auto& tgt = fp.get(0, TreatAsTripleRegister{});
                tgt.setValue(result, TreatAsExtendedReal {});
                break;
            }
            case 0b00001: // fp1
            {
                auto& tgt = fp.get(4, TreatAsTripleRegister{});
                tgt.setValue(result, TreatAsExtendedReal {});
                break;
            }
            case 0b00010: // fp2
            {
                auto& tgt = fp.get(8, TreatAsTripleRegister{});
                tgt.setValue(result, TreatAsExtendedReal {});
                break;
            }
            case 0b00011: // fp3
            {
                auto& tgt = fp.get(12, TreatAsTripleRegister{});
                tgt.setValue(result, TreatAsExtendedReal {});
                break;
            }
            default:
                invalidOpcodeFault();
                break;
        }
    } else {
        // gpr
        getGPR(inst.getSrcDest(), TreatAsTripleRegister {}).setValue(result, TreatAsExtendedReal {});
    }
}

void
Core::cosr(const REGInstruction &inst) {

    unimplementedFault();
}

void
Core::cosrl(const REGInstruction &inst) {
    unimplementedFault();
}
namespace {
    ExtendedReal cosre(ExtendedReal input, TreatAsExtendedReal) {
        return std::cos(input);
    }
    LongReal cosrl(LongReal input, TreatAsLongReal) {
        return std::cos(input);
    }
    Real cosr(Real input, TreatAsReal) {
        return std::cos(input);
    }
}

void
Core::processFPInstruction(const REGInstruction &inst ) {
    if (isFloatingPointInstruction(inst.getOpcode())) {
        switch (inst.getOpcode()) {
            case Opcodes::movre:
                movre(inst);
                break;
            case Opcodes::cpyrsre:
                cpyrsre(inst);
                break;
            case Opcodes::cpysre:
                cpysre(inst);
                break;
#define X(name) case Opcodes :: name ## r : name ## r (inst) ; break;\
            case Opcodes:: name ## rl : name ## rl (inst); break
            X(class);
            X(cos);
            X(mov);
#undef X
            default:
                unimplementedFault();
                break;
        }
    } else {
        unimplementedFault();
    }
}

namespace {
    TripleRegister bogus;
}
const TripleRegister&
Core::getFloatingPointRegister(ByteOrdinal index) const {
    switch (index) {
        case 0b00000: // fp0
            return fp.get(0, TreatAsTripleRegister{});
        case 0b00001: // fp1
            return fp.get(4, TreatAsTripleRegister{});
        case 0b00010: // fp2
            return fp.get(8, TreatAsTripleRegister{});
        case 0b00011: // fp3
            return fp.get(12, TreatAsTripleRegister{});
        case 0b10000: // +0.0
            return fpZero;
        case 0b10110: // +1.0
            return fpOne;
        default:
            invalidOpcodeFault();
            return bogus;
    }
}
TripleRegister&
Core::getFloatingPointRegister(ByteOrdinal index) {
    switch (index) {
        case 0b00000: // fp0
            return fp.get(0, TreatAsTripleRegister{});
        case 0b00001: // fp1
            return fp.get(4, TreatAsTripleRegister{});
        case 0b00010: // fp2
            return fp.get(8, TreatAsTripleRegister{});
        case 0b00011: // fp3
            return fp.get(12, TreatAsTripleRegister{});
        default:
            invalidOpcodeFault();
            return bogus;
    }
}

Core::MixedLongRealSourceArgument
Core::unpackSrc1(const REGInstruction& inst, TreatAsLongReal) const {
    if (inst.getM1()) {
        return getFloatingPointRegister(inst.getSrc1()).getValue<ExtendedReal >();
    } else {
        return getGPR(inst.getSrc1(), TreatAsLongRegister{}).getValue<LongReal>();
    }
}
Core::MixedLongRealSourceArgument
Core::unpackSrc2(const REGInstruction& inst, TreatAsLongReal) const {
    if (inst.getM2()) {
        return getFloatingPointRegister(inst.getSrc2()).getValue<ExtendedReal >();
    } else {
        return getGPR(inst.getSrc2(), TreatAsLongRegister{}).getValue<LongReal>();
    }
}

Core::MixedRealSourceArgument
Core::unpackSrc1(const REGInstruction &inst, TreatAsReal) const {
    if (inst.getM1()) {
        return getFloatingPointRegister(inst.getSrc1()).getValue<ExtendedReal >();
    } else {
        return getGPR(inst.getSrc1()).getValue<Real>();
    }
}

Core::MixedRealSourceArgument
Core::unpackSrc2(const REGInstruction &inst, TreatAsReal) const {
    if (inst.getM2()) {
        return getFloatingPointRegister(inst.getSrc2()).getValue<ExtendedReal >();
    } else {
        return getGPR(inst.getSrc2()).getValue<Real>();
    }
}
