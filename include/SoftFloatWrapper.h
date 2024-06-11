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
//
// Created by jwscoggins on 6/9/24.
//

#ifndef SIM960_SOFTFLOATWRAPPER_H
#define SIM960_SOFTFLOATWRAPPER_H
#include <cstdint>
extern "C" {
#include "platform.h"
#include "softfloat_types.h"
#include "softfloat.h"
}
namespace SoftFloat {
    using Real_t = float32_t;
    using LongReal_t = float64_t;
    using ExtendedReal_t = extFloat80_t;
    inline LongReal_t toLongReal(Real_t value) noexcept { return f32_to_f64(value); }
    inline ExtendedReal_t toExtendedReal(Real_t value) noexcept {
        ExtendedReal_t storage;
        f32_to_extF80M(value, &storage);
        return storage;
    }
    inline Real_t toReal(LongReal_t value) noexcept { return f64_to_f32(value); }
    inline ExtendedReal_t toExtendedReal(LongReal_t value) noexcept {
        ExtendedReal_t storage;
        f64_to_extF80M(value, &storage);
        return storage;
    }
    inline Real_t toReal(const ExtendedReal_t& value) noexcept { return extF80M_to_f32(&value); }
    inline LongReal_t toLongReal(const ExtendedReal_t& value) noexcept { return extF80M_to_f64(&value); }
    inline bool equals(Real_t a, Real_t b) noexcept { return f32_eq(a, b); }
    inline bool notEquals(Real_t a, Real_t b) noexcept { return !equals(a, b); }
    inline bool lessThan(Real_t a, Real_t b) noexcept { return f32_lt(a, b); }
    inline bool lessThanOrEqual(Real_t a, Real_t b) noexcept { return f32_le(a, b); }
    inline bool greaterThan(Real_t a, Real_t b) noexcept { return f32_lt(b, a); }
    inline bool greaterThanOrEqual(Real_t a, Real_t b) noexcept { return f32_le(b, a); }
    inline bool equals(LongReal_t a, LongReal_t b) noexcept { return f64_eq(a, b); }
    inline bool notEquals(LongReal_t a, LongReal_t b) noexcept { return !equals(a, b); }
    inline bool lessThan(LongReal_t a, LongReal_t b) noexcept { return f64_lt(a, b); }
    inline bool lessThanOrEqual(LongReal_t a, LongReal_t b) noexcept { return f64_le(a, b); }
    inline bool greaterThan(LongReal_t a, LongReal_t b) noexcept { return f64_lt(b, a); }
    inline bool greaterThanOrEqual(LongReal_t a, LongReal_t b) noexcept { return f64_le(b, a); }

    inline bool equals(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept { return extF80M_eq(&a, &b); }
    inline bool notEquals(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept { return !equals(a, b); }
    inline bool lessThan(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept { return extF80M_lt(&a, &b); }
    inline bool lessThanOrEqual(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept { return extF80M_le(&a, &b); }
    inline bool greaterThan(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept { return extF80M_lt(&b, &a); }
    inline bool greaterThanOrEqual(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept { return extF80M_le(&b, &a); }

#define X(kind, name, prefix, operation) inline kind name (kind a, kind b) noexcept { return prefix ## _ ## operation (a, b); }
#define Y(kind, prefix) X(kind, add, prefix, add)  \
                        X(kind, subtract, prefix, sub) \
                        X(kind, multiply, prefix, mul) \
                        X(kind, divide, prefix, div)   \
                        X(kind, remainder, prefix, rem)
                        Y(Real_t, f32)
                        Y(LongReal_t, f64)
#undef Y
#undef X
    inline ExtendedReal_t add(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept {
        ExtendedReal_t out;
        extF80M_add(&a, &b, &out);
        return out;
    }
    inline ExtendedReal_t subtract(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept {
        ExtendedReal_t out;
        extF80M_sub(&a, &b, &out);
        return out;
    }
    inline ExtendedReal_t multiply(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept {
        ExtendedReal_t out;
        extF80M_mul(&a, &b, &out);
        return out;
    }
    inline ExtendedReal_t divide(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept {
        ExtendedReal_t out;
        extF80M_div(&a, &b, &out);
        return out;
    }
    inline ExtendedReal_t remainder(const ExtendedReal_t& a, const ExtendedReal_t& b) noexcept {
        ExtendedReal_t out;
        extF80M_rem(&a, &b, &out);
        return out;
    }
} // end namespace SoftFloat
inline auto operator+(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::add(a, b); }
inline auto operator-(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::subtract(a, b); }
inline auto operator*(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::multiply(a, b); }
inline auto operator/(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::divide(a, b); }
inline auto operator%(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::remainder(a, b); }
inline auto operator==(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::equals(a, b); }
inline auto operator!=(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::notEquals(a, b); }
inline auto operator<(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::lessThan(a, b); }
inline auto operator<=(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::lessThanOrEqual(a, b); }
inline auto operator>(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::greaterThan(a, b); }
inline auto operator>=(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return SoftFloat::greaterThanOrEqual(a, b); }

inline auto operator+(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::add(a, b); }
inline auto operator-(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::subtract(a, b); }
inline auto operator*(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::multiply(a, b); }
inline auto operator/(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::divide(a, b); }
inline auto operator%(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::remainder(a, b); }
inline auto operator==(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::equals(a, b); }
inline auto operator!=(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::notEquals(a, b); }
inline auto operator<(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::lessThan(a, b); }
inline auto operator<=(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::lessThanOrEqual(a, b); }
inline auto operator>(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::greaterThan(a, b); }
inline auto operator>=(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return SoftFloat::greaterThanOrEqual(a, b); }

inline auto operator+(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::add(a, b); }
inline auto operator-(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::subtract(a, b); }
inline auto operator*(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::multiply(a, b); }
inline auto operator/(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::divide(a, b); }
inline auto operator%(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::remainder(a, b); }
inline auto operator==(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::equals(a, b); }
inline auto operator!=(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::notEquals(a, b); }
inline auto operator<(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::lessThan(a, b); }
inline auto operator<=(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::lessThanOrEqual(a, b); }
inline auto operator>(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::greaterThan(a, b); }
inline auto operator>=(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept { return SoftFloat::greaterThanOrEqual(a, b); }
// keep it out of the namespace for simple use

/**
 * The following comes from "The 80960 Microprocessor Architecture" by Glenford J. Myers and David L. Budde on page 81
 *
 * A secondary purpose of the floating-point registers is providing the mechanism for mixed-length arithmetic.
 * This is accomplished by the following rule -
 *  if an operand of an instruction is a floating-point register (fp0-3), the operand is interpreted as an extended-real operand,
 *  regardless of the length specified by the instruction's opcode.
 *
 *  -----
 *  Later on, the book states that while mixed 80/32 or 80/64 are permitted 32/64 mixes are not permitted.
 *  In fact, the instruction set pretty much prevents that
 *
 *  What I need to figure out is the best way to handle these mixes. We have several of them to be denoted:
 *  (op ExtendedReal ExtendedReal) -> ExtendedReal (if destination is a floating-point register)
 *  (op Real Real) -> Real (we are saving to a GPR)
 *  (op LongReal LongReal) -> LongReal
 *  (op ExtendedReal Real) -> Depends on destination
 *  (op Real ExtendedReal) -> Depends on destination
 *  (op ExtendedReal LongReal) -> Depends on destination
 *  (op LongReal ExtendedReal) -> Depends on destination
 *
 *  addr r8, r9, r9 # 32-bit operands, result is 32-bits as well
 *  addr r8, r9, fp0 # 32-bit operands, result is 80 bits (add the two reals and then convert the result to 80-bit format)
 *  addr fp0, fp1, fp2  # 80-bit operands and being stored into an 80-bit register
 *  addrl r8, fp0, fp0  # add long real in r8 to extended real in fp0 and store in fp0, what does this operation look like?
 *
 */
#endif //SIM960_SOFTFLOATWRAPPER_H
