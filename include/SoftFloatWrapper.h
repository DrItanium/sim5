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
} // end namespace SoftFloat
// keep it out of the namespace for simple use
inline SoftFloat::Real_t operator+(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_add(a, b); }
inline SoftFloat::Real_t operator-(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_sub(a, b); }
inline SoftFloat::Real_t operator*(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_mul(a, b); }
inline SoftFloat::Real_t operator/(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_div(a, b); }
inline SoftFloat::Real_t operator%(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_rem(a, b); }
inline bool operator==(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_eq(a, b); }
inline bool operator!=(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return !f32_eq(a, b); }
inline bool operator<(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_lt(a, b); }
inline bool operator<=(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_le(a, b); }
inline bool operator>(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_lt(b, a); }
inline bool operator>=(const SoftFloat::Real_t& a, const SoftFloat::Real_t& b) noexcept { return f32_le(b, a); }

inline SoftFloat::LongReal_t operator+(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_add(a, b); }
inline SoftFloat::LongReal_t operator-(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_sub(a, b); }
inline SoftFloat::LongReal_t operator*(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_mul(a, b); }
inline SoftFloat::LongReal_t operator/(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_div(a, b); }
inline SoftFloat::LongReal_t operator%(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_rem(a, b); }
inline bool operator==(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_eq(a, b); }
inline bool operator!=(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return !f64_eq(a, b); }
inline bool operator<(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_lt(a, b); }
inline bool operator<=(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_le(a, b); }
inline bool operator>(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_lt(b, a); }
inline bool operator>=(const SoftFloat::LongReal_t& a, const SoftFloat::LongReal_t& b) noexcept { return f64_le(b, a); }
inline SoftFloat::ExtendedReal_t operator+(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept {
    SoftFloat::ExtendedReal_t result;
    extF80M_add(&a, &b, &result);
    return result;
}
inline SoftFloat::ExtendedReal_t operator-(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept {
    SoftFloat::ExtendedReal_t result;
    extF80M_sub(&a, &b, &result);
    return result;
}

inline SoftFloat::ExtendedReal_t operator*(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept {
    SoftFloat::ExtendedReal_t result;
    extF80M_mul(&a, &b, &result);
    return result;
}

inline SoftFloat::ExtendedReal_t operator/(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept {
    SoftFloat::ExtendedReal_t result;
    extF80M_div(&a, &b, &result);
    return result;
}

inline SoftFloat::ExtendedReal_t operator%(const SoftFloat::ExtendedReal_t& a, const SoftFloat::ExtendedReal_t& b) noexcept {
    SoftFloat::ExtendedReal_t result;
    extF80M_rem(&a, &b, &result);
    return result;
}

#endif //SIM960_SOFTFLOATWRAPPER_H
