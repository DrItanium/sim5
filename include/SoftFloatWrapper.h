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
#include "softfloat.h"
#include "softfloat_types.h"
}
namespace SoftFloat {
    struct Real {
        using RawType = float32_t;
        constexpr Real(RawType value) noexcept : _raw(value) {}
        bool operator<(const Real& other) const noexcept { return f32_lt(_raw, other._raw); }
        bool operator<=(const Real& other) const noexcept { return f32_le(_raw, other._raw); }
        bool operator>(const Real& other) const noexcept { return !f32_le(_raw, other._raw); }
        bool operator>=(const Real& other) const noexcept { return !f32_lt(_raw, other._raw); }
        bool operator==(const Real& other) const noexcept { return f32_eq(_raw, other._raw); }
        bool operator!=(const Real& other) const noexcept { return !f32_eq(_raw, other._raw); }
        Real operator+(const Real& other) const noexcept { return f32_add(_raw, other._raw); }
        Real operator-(const Real& other) const noexcept { return f32_sub(_raw, other._raw); }
        Real operator*(const Real& other) const noexcept { return f32_mul(_raw, other._raw); }
        Real operator/(const Real& other) const noexcept { return f32_div(_raw, other._raw); }
        [[nodiscard]] constexpr auto value() const noexcept { return _raw; }

    private:
        RawType _raw;
    };

    struct LongReal {
        float64_t _raw;
    };
    struct ExtendedReal {
        extFloat80_t _raw;
    };
}
#endif //SIM960_SOFTFLOATWRAPPER_H
