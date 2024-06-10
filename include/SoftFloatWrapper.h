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
    inline Real_t operator+(const Real_t& a, const Real_t& b) noexcept { return f32_add(a, b); }
    inline Real_t operator-(const Real_t& a, const Real_t& b) noexcept { return f32_sub(a, b); }
    inline Real_t operator*(const Real_t& a, const Real_t& b) noexcept { return f32_mul(a, b); }
    inline Real_t operator/(const Real_t& a, const Real_t& b) noexcept { return f32_div(a, b); }
    inline Real_t operator%(const Real_t& a, const Real_t& b) noexcept { return f32_rem(a, b); }
    inline bool operator==(const Real_t& a, const Real_t& b) noexcept { return f32_eq(a, b); }
    inline bool operator!=(const Real_t& a, const Real_t& b) noexcept { return !f32_eq(a, b); }
    inline bool operator<(const Real_t& a, const Real_t& b) noexcept { return f32_lt(a, b); }
    inline bool operator<=(const Real_t& a, const Real_t& b) noexcept { return f32_le(a, b); }
    // according to the docs, the gt is the same as the lt but with the args reversed!
    inline bool operator>(const Real_t& a, const Real_t& b) noexcept { return f32_lt(b, a); }
    inline bool operator>=(const Real_t& a, const Real_t& b) noexcept { return f32_le(b, a); }
} // end namespace SoftFloat
#endif //SIM960_SOFTFLOATWRAPPER_H
