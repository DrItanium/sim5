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
#include <cmath>
#include <cfenv>

OptionalFaultRecord
Core::processFPInstruction(const REGInstruction &inst ) {
    if (auto opcode = inst.getOpcode(); isFloatingPointInstruction(opcode)) {
        switch (opcode) {
#define X(name) case Opcodes:: name : name (inst); break
            X(addr);
            X(addrl);
            X(atanr);
            X(atanrl);
            X(classr);
            X(classrl);
            X(cmpr);
            X(cmprl);
            X(cmpor);
            X(cmporl);
            X(cosr);
            X(cosrl);
            X(cpyrsre);
            X(cpysre);
            X(cvtri);
            X(cvtril);
            X(cvtzri);
            X(cvtzril);
            X(cvtilr);
            X(cvtir);
            X(divr);
            X(divrl);
            X(movr);
            X(movre);
            X(movrl);
            X(mulr);
            X(mulrl);
            X(remr);
            X(remrl);
            X(roundr);
            X(roundrl);
            X(subr);
            X(subrl);
            X(scaler);
            X(scalerl);
            X(sinr);
            X(sinrl);
            X(sqrtr);
            X(sqrtrl);
            X(tanr);
            X(tanrl);
            X(logeprl);
            X(logepr);
            X(logbnrl);
            X(logbnr);
            X(logr);
            X(logrl);

#undef X
            default:
                return unimplementedFault();
        }
    } else {
        return unimplementedFault();
    }
}
void
Core::dmovt(Register& dest, Ordinal src) noexcept {
    dest.setValue<Ordinal>(src);
    auto decimal = static_cast<uint8_t>(src) ;
    ac_.setConditionResult(decimal >= 0b00110000 && decimal <= 0b00111001);
}

OptionalFaultRecord
Core::classr(const REGInstruction& inst) {
    return std::visit([this](auto value) { return performClassification(value); }, unpackSrc1(inst, TreatAsReal{}));
}
OptionalFaultRecord
Core::classrl(const REGInstruction& inst) {
    return std::visit([this](auto value) { return performClassification(value); }, unpackSrc1(inst, TreatAsLongReal{}));
}

void
Core::movr(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) { fpassignment(inst, value, TreatAs<std::decay_t<decltype(value)>>{}); }, unpackSrc1(inst, TreatAsReal{}));
    /// @todo implement floating point faults
}

void
Core::movrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) { fpassignment(inst, value, TreatAs<std::decay_t<decltype(value)>>{}); }, unpackSrc1(inst, TreatAsLongReal{}));
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
    auto src2 = unpackSrc2(inst, TreatAsExtendedReal{});
    auto src1 = unpackSrc1(inst, TreatAsExtendedReal{});
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
    auto src2 = unpackSrc2(inst, TreatAsExtendedReal{});
    auto src1 = unpackSrc1(inst, TreatAsExtendedReal{});
    fpassignment(inst, std::signbit(src2) == 0 ? std::fabs(src1) : -std::fabs(src1), TreatAsExtendedReal{});
}

OptionalFaultRecord
Core::fpassignment(const REGInstruction &inst, ExtendedReal val, TreatAsExtendedReal) {
    auto result = serviceFloatingPointFault<ExtendedReal>(val);
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
                return invalidOpcodeFault();
        }
    } else {
        // gpr
        getGPR(inst.getSrcDest(), TreatAsTripleRegister {}).setValue(result, TreatAsExtendedReal {});
    }
    return std::nullopt;
}

OptionalFaultRecord
Core::fpassignment(const REGInstruction &inst, Real val, TreatAsReal) {
    auto result = serviceFloatingPointFault<Real>(val);
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
                tgt.setValue(result, TreatAsExtendedReal{});
                break;
            }
            case 0b00010: // fp2
            {
                auto& tgt = fp.get(8, TreatAsTripleRegister{});
                tgt.setValue(result, TreatAsExtendedReal{});
                break;
            }
            case 0b00011: // fp3
            {
                auto& tgt = fp.get(12, TreatAsTripleRegister{});
                tgt.setValue(result, TreatAsExtendedReal{});
                break;
            }
            default:
                return invalidOpcodeFault();
        }
    } else {
        // gpr
        getGPR(inst.getSrcDest()).setValue(result, TreatAsReal{});
    }
    return std::nullopt;
}

OptionalFaultRecord
Core::fpassignment(const REGInstruction &inst, LongReal val, TreatAsLongReal) {
    auto result = serviceFloatingPointFault<LongReal>(val);
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
                tgt.setValue(result, TreatAsExtendedReal{});
                break;
            }
            case 0b00010: // fp2
            {
                auto& tgt = fp.get(8, TreatAsTripleRegister{});
                tgt.setValue(result, TreatAsExtendedReal{});
                break;
            }
            case 0b00011: // fp3
            {
                auto& tgt = fp.get(12, TreatAsTripleRegister{});
                tgt.setValue(result, TreatAsExtendedReal{});
                break;
            }
            default:
                return invalidOpcodeFault();
        }
    } else {
        // gpr
        getGPR(inst.getSrcDest(), TreatAsLongRegister{}).setValue(result, TreatAsLongReal{});
    }
    return std::nullopt;
}

namespace {
    TripleRegister bogus;
}
VariantWithFaultRecord<std::reference_wrapper<const TripleRegister>>
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
        default:
            return invalidOpcodeFault();
    }
}

[[maybe_unused]] VariantWithFaultRecord<std::reference_wrapper<TripleRegister>>
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
            return invalidOpcodeFault();
    }
}

Core::MixedLongRealSourceArgument
Core::unpackSrc1(const REGInstruction& inst, TreatAsLongReal) const {
    auto src1Index = inst.getSrc1();
    if (inst.getM1()) {
        if (inst.src1IsFPLiteral()) {
            return std::visit([this](auto value) { return handleSubnormalCase(value); }, getFloatingPointLiteral<LongReal>(src1Index));
        } else {
            return std::visit([this](auto value) { return handleSubnormalCase(value); }, getFloatingPointRegister(src1Index));
        }
    } else {
        return std::visit([](auto value) { return value; }, handleSubnormalCase(getGPR(src1Index, TreatAsLongRegister{}).getValue<LongReal>()));
    }
}
Core::MixedLongRealSourceArgument
Core::unpackSrc2(const REGInstruction& inst, TreatAsLongReal) const {
    auto index = inst.getSrc2();
    if (inst.getM2()) {
        if (inst.src2IsFPLiteral()) {
            return std::visit([this](auto value) { return handleSubnormalCase(value); }, getFloatingPointLiteral<LongReal>(index));
        } else {
            return std::visit([this](auto value) { return handleSubnormalCase(value); }, getFloatingPointRegister(index));
        }
    } else {
        return std::visit([](auto value) { return value; }, handleSubnormalCase(getGPR(index, TreatAsLongRegister{}).getValue<LongReal>()));
    }
}

Core::MixedRealSourceArgument
Core::unpackSrc1(const REGInstruction& inst, TreatAsReal) const {
    if (inst.getM1()) {
        if (inst.src1IsFPLiteral()) {
            return handleSubnormalCase(getFloatingPointLiteral<Real>(inst.getSrc1()));
        } else {
            return handleSubnormalCase(getFloatingPointRegister(inst.getSrc1()).getValue<ExtendedReal >());
        }
    } else {
        return handleSubnormalCase(getGPR(inst.getSrc1()).getValue<Real>());
    }
}
Core::MixedRealSourceArgument
Core::unpackSrc2(const REGInstruction& inst, TreatAsReal) const {
    if (inst.getM2()) {
        if (inst.src2IsFPLiteral()) {
            return handleSubnormalCase(getFloatingPointLiteral<Real>(inst.getSrc2()));
        } else {
            return handleSubnormalCase(getFloatingPointRegister(inst.getSrc2()).getValue<ExtendedReal >());
        }
    } else {
        return handleSubnormalCase(getGPR(inst.getSrc2()).getValue<Real>());
    }
}

ExtendedReal
Core::unpackSrc1(const REGInstruction &inst, TreatAsExtendedReal) const {
    if (inst.getM1()) {
        if (inst.src1IsFPLiteral()) {
            return handleSubnormalCase(getFloatingPointLiteral<ExtendedReal>(inst.getSrc1()));
        } else {
            return handleSubnormalCase(getFloatingPointRegister(inst.getSrc1()).getValue<ExtendedReal >());
        }
    } else {
        return handleSubnormalCase(getGPR(inst.getSrc1(), TreatAsTripleRegister{}).getValue<ExtendedReal>());
    }
}

ExtendedReal
Core::unpackSrc2(const REGInstruction &inst, TreatAsExtendedReal) const {
    if (inst.getM2()) {
        if (inst.src2IsFPLiteral()) {
            return handleSubnormalCase(getFloatingPointLiteral<ExtendedReal>(inst.getSrc2()));
        } else {
            return handleSubnormalCase(getFloatingPointRegister(inst.getSrc2()).getValue<ExtendedReal>());
        }
    } else {
        return handleSubnormalCase(getGPR(inst.getSrc2(), TreatAsTripleRegister{}).getValue<ExtendedReal>());
    }
}
void
Core::cosr(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::cos(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsReal{}));
}

void
Core::cosrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::cos(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsLongReal{}));
}
void
Core::sinr(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::sin(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsReal{}));
}

void
Core::sinrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::sin(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsLongReal{}));
}
void
Core::tanr(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::tan(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsReal{}));
}

void
Core::tanrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::tan(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsLongReal{}));
}
void
Core::atanr(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::atan(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsReal{}));
}

void
Core::atanrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::atan(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsLongReal{}));
}
void
Core::sqrtr(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::sqrt(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsReal{}));
}

void
Core::sqrtrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::sqrt(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsLongReal{}));
}
void
Core::roundr(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::round(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsReal{}));
}

void
Core::roundrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto value) {
                   using K = std::decay_t<decltype(value)>;
                   fpassignment(inst, std::round(value), TreatAs<K>{});
               },
               unpackSrc1(inst, TreatAsLongReal{}));
}

void
Core::scaler(const REGInstruction &inst) {
    std::visit([this, &inst](auto src2) {
                   using K = std::decay_t<decltype(src2)>;
                   auto src1 = static_cast<Integer>(getSrc1Register(inst));
                   fpassignment(inst, std::scalbn(src2, src1), TreatAs<K>{});
               },
               unpackSrc2(inst, TreatAsReal{}));
}

void
Core::scalerl(const REGInstruction &inst) {
    std::visit([this, &inst](auto src2) {
                   using K = std::decay_t<decltype(src2)>;
                   auto src1 = static_cast<Integer>(getSrc1Register(inst));
                   fpassignment(inst, std::scalbn(src2, src1), TreatAs<K>{});
               },
               unpackSrc2(inst, TreatAsLongReal{}));
}
template<typename T0, typename T1, typename T2>
constexpr bool BothAreSameAs = std::is_same_v<T0, T2> && std::is_same_v<T1, T2>;
template<typename T0, typename T1>
constexpr bool BothAreReal = BothAreSameAs<T0, T1, Real>;
template<typename T0, typename T1>
constexpr bool BothAreLongReal = BothAreSameAs<T0, T1, LongReal>;
void
Core::addr(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreReal<K0, K1>) {
                       // if we get real/real then we assign as real, every other case mixed so treat them as long double/long double
                       fpassignment(inst, src2 + src1, TreatAsReal{});
                   } else {
                       // if they are different then it means mixed addition, so we should always do 80-bit operations
                       fpassignment(inst, src2 + src1, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsReal{}),
               unpackSrc2(inst, TreatAsReal{}));
}

void
Core::addrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreLongReal<K0, K1>) {
                       fpassignment(inst, src2 + src1, TreatAsLongReal{});
                   } else {
                       // if they are different then it means mixed addition, so we should always do 80-bit operations
                       fpassignment(inst, src2 + src1, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsLongReal{}),
               unpackSrc2(inst, TreatAsLongReal{}));
}

void
Core::subr(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreReal<K0, K1>) {
                       // if we get real/real then we assign as real, every other case mixed so treat them as long double/long double
                       fpassignment(inst, src2 - src1, TreatAsReal{});
                   } else {
                       // if they are different then it means mixed subtraction, so we should always do 80-bit operations
                       fpassignment(inst, src2 - src1, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsReal{}),
               unpackSrc2(inst, TreatAsReal{}));
}

void
Core::subrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreLongReal<K0, K1>) {
                       fpassignment(inst, src2 - src1, TreatAsLongReal{});
                   } else {
                       // if they are different then it means mixed subtraction, so we should always do 80-bit operations
                       fpassignment(inst, src2 - src1, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsLongReal{}),
               unpackSrc2(inst, TreatAsLongReal{}));
}
void
Core::divr(const REGInstruction &inst) {
    std::visit([this, &inst](auto denominator, auto numerator) {
                   using K0 = std::decay_t<decltype(numerator)>;
                   using K1 = std::decay_t<decltype(denominator)>;
                   if constexpr (BothAreReal<K0, K1>) {
                       // if we get real/real then we assign as real, every other case mixed so treat them as long double/long double
                       fpassignment(inst, numerator / denominator, TreatAsReal{});
                   } else {
                       fpassignment(inst, numerator / denominator, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsReal{}),
               unpackSrc2(inst, TreatAsReal{}));
}

void
Core::divrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto denominator, auto numerator) {
                   using K0 = std::decay_t<decltype(numerator)>;
                   using K1 = std::decay_t<decltype(denominator)>;
                   if constexpr (BothAreLongReal<K0, K1>) {
                       // if we get real/real then we assign as real, every other case mixed so treat them as long double/long double
                       fpassignment(inst, numerator / denominator, TreatAsLongReal{});
                   } else {
                       fpassignment(inst, numerator / denominator, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsLongReal{}),
               unpackSrc2(inst, TreatAsLongReal{}));
}

void
Core::cmpor(const REGInstruction &inst) {
    cmpr(inst);
    if (!ac_.getConditionCode()) {
        floatingInvalidOperationFault();
    }
}

void
Core::cmporl(const REGInstruction &inst) {
    cmprl(inst);
    if (!ac_.getConditionCode()) {
        floatingInvalidOperationFault();
    }
}

void
Core::cmpr(const REGInstruction &inst) {
    std::visit([this](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreReal<K0, K1>) {
                       cmpGeneric<Real>(src1, src2);
                   } else {
                       cmpGeneric<ExtendedReal>(src1, src2);
                   }
               },
               unpackSrc1(inst, TreatAsReal{}),
               unpackSrc2(inst, TreatAsReal{}));
}

void
Core::cmprl(const REGInstruction& inst) {
    std::visit([this](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreLongReal<K0, K1>) {
                       cmpGeneric<LongReal>(src1, src2);
                   } else {
                       cmpGeneric<ExtendedReal>(src1, src2);
                   }
               },
               unpackSrc1(inst, TreatAsLongReal{}),
               unpackSrc2(inst, TreatAsLongReal{}));
}
// dst <- real(src)

void
Core::updateRoundingMode() const {
    switch (ac_.arith.floatingPointRoundingControl) {
        case 0b00: // round to nearest (even)
            std::fesetround(FE_TONEAREST);
            break;
        case 0b01: // round down (towards negative infinity)
            std::fesetround(FE_DOWNWARD);
            break;
        case 0b10: // round up (towards positive infinity)
            std::fesetround(FE_UPWARD);
            break;
        case 0b11: // truncate (round toward zero)
            std::fesetround(FE_TOWARDZERO);
            break;
    }
}

void
Core::mulr(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreReal<K0, K1>) {
                       fpassignment(inst, src2 * src1, TreatAsReal{});
                   } else {
                       fpassignment(inst, src2 * src1, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsReal{}),
               unpackSrc2(inst, TreatAsReal{}));
}

void
Core::mulrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreLongReal<K0, K1>) {
                       fpassignment(inst, src2 * src1, TreatAsLongReal{});
                   } else {
                       fpassignment(inst, src2 * src1, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsLongReal{}),
               unpackSrc2(inst, TreatAsLongReal{}));
}
// arithmetic status bits for remainder operations
// bit 6: Q1, the next-to-last quotient bit
// bit 5: Q0, the last quotient bit
// bit 4: QR, the value the next quotient bit would have if one more reduction were performed (the "round" bit of the quotient)
// bit 3: QS, set if the remainder after the QR reductionw ould be nonzero (the "sticky" bit of the quotient)
void
Core::remr(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreReal<K0, K1>) {
                       int quad;
                       auto result = std::remquo(src2, src1, &quad);
                       /// @todo fix this up to do the right thing! Right now it is just the lowest quotient bits
                       ac_.arith.arithmeticStatus = quad;
                       // assign the quad bits to the arithmetic status register
                       fpassignment(inst, result, TreatAsReal{});
                   } else {
                       int quad;
                       auto result = std::remquo(src2, src1, &quad);
                       /// @todo fix this up to do the right thing! Right now it is just the lowest quotient bits
                       ac_.arith.arithmeticStatus = quad;
                       // assign the quad bits to the arithmetic status register
                       fpassignment(inst, result, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsReal{}),
               unpackSrc2(inst, TreatAsReal{}));
}

void
Core::remrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreLongReal<K0, K1>) {
                       int quad;
                       auto result = std::remquo(src2, src1, &quad);
                       /// @todo fix this up to do the right thing! Right now it is just the lowest quotient bits
                       ac_.arith.arithmeticStatus = quad;

                       // assign the quad bits to the arithmetic status register
                       fpassignment(inst, result, TreatAsLongReal{});
                   } else {
                       int quad;
                       auto result = std::remquo(src2, src1, &quad);
                       /// @todo fix this up to do the right thing! Right now it is just the lowest quotient bits
                       ac_.arith.arithmeticStatus = quad;
                       // assign the quad bits to the arithmetic status register
                       fpassignment(inst, result, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsLongReal{}),
               unpackSrc2(inst, TreatAsLongReal{}));
}

void
Core::logbnr(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1) {
                   using K = std::decay_t<decltype(src1)>;
                   if constexpr (std::is_same_v<K, ExtendedReal>) {
                       fpassignment(inst, std::logbl(src1), TreatAs<K>{});
                   } else {
                       fpassignment(inst, std::logbf(src1), TreatAs<K>{});
                   }
               },
               unpackSrc1(inst, TreatAsReal{}));
}
void
Core::logbnrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1) {
                   using K = std::decay_t<decltype(src1)>;
                   if constexpr (std::is_same_v<K, ExtendedReal>) {
                       fpassignment(inst, std::logbl(src1), TreatAs<K>{});
                   } else {
                       fpassignment(inst, std::logb(src1), TreatAs<K>{});
                   }
               },
               unpackSrc1(inst, TreatAsLongReal{}));
}
void
Core::logr(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreReal<K0, K1>) {
                       auto result = serviceFloatingPointFault<Real>(std::log2f(src1));

                       // assign the quad bits to the arithmetic status register
                       fpassignment(inst, src2 * result, TreatAsLongReal{});
                   } else {
                       auto result = serviceFloatingPointFault<ExtendedReal>(std::log2l(src1));
                       // assign the quad bits to the arithmetic status register
                       fpassignment(inst, src2 * result, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsReal{}),
               unpackSrc2(inst, TreatAsReal{}));
}
void
Core::logrl(const REGInstruction &inst) {
    std::visit([this, &inst](auto src1, auto src2) {
                   using K0 = std::decay_t<decltype(src1)>;
                   using K1 = std::decay_t<decltype(src2)>;
                   if constexpr (BothAreLongReal<K0, K1>) {
                       auto result = serviceFloatingPointFault<LongReal>(std::log2(src1));
                       fpassignment(inst, src2 * result, TreatAsLongReal{});
                   } else {
                       auto result = serviceFloatingPointFault<ExtendedReal>(std::log2l(src1));
                       fpassignment(inst, src2 * result, TreatAsExtendedReal{});
                   }
               },
               unpackSrc1(inst, TreatAsLongReal{}),
               unpackSrc2(inst, TreatAsLongReal{}));
}

void
Core::logepr(const REGInstruction &inst) {
   unimplementedFault();
}
void
Core::logeprl(const REGInstruction &inst) {
    unimplementedFault();
}

void
Core::cvtir(const REGInstruction& inst) {
    // convert integer to real
    fpassignment(inst, serviceFloatingPointFault<Real>(unpackSrc1(inst, TreatAsInteger{})), TreatAsReal{});
}
void
Core::cvtilr(const REGInstruction& inst) {
    //convert long integer to real
    // not a long real!
    // two instruction combination for converting integer to long real format
    // cvtir g6, fp3
    // movrl fp3, g8 # result stored in g8, g9
    fpassignment(inst, serviceFloatingPointFault<LongReal>(unpackSrc1(inst, TreatAsLongInteger{})), TreatAsLongReal{});
}

void
Core::cvtri(const REGInstruction &inst) {

    unimplementedFault();
}

void
Core::cvtril(const REGInstruction &inst) {

    unimplementedFault();
}

void
Core::cvtzri(const REGInstruction &inst) {

    unimplementedFault();
}

void
Core::cvtzril(const REGInstruction &inst) {

    unimplementedFault();
}
OptionalFaultRecord
Core::fpassignment(const REGInstruction&, FaultRecord& record, TreatAs<FaultRecord>) {
    return record;
}
