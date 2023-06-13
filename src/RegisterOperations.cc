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

const Register&
Core::getSrc1Register(TreatAsREG) const noexcept {
    if (instruction_.reg.m1) {
        /// @todo what to do if s1 is also set?
        return constants_.get(instruction_.reg.src1);
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1);
    } else {
        return getGPR(instruction_.reg.src1);
    }
}

const Register&
Core::getSrc2Register(TreatAsREG) const noexcept {
    if (instruction_.reg.m2) {
        /// @todo what to do if s1 is also set?
        return constants_.get(instruction_.reg.src2);
    } else if (instruction_.reg.s2) {
        return getSFR(instruction_.reg.src2);
    } else {
        return getGPR(instruction_.reg.src2);
    }
}


Ordinal
Core::unpackSrc1(ByteOrdinal offset, TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        // literals should always return zero if offset is greater than zero
        return offset == 0 ? constants_.getValue<Ordinal>(instruction_.reg.src1) : 0;
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1, offset).o;
    } else {
        return getGPRValue(instruction_.reg.src1, offset, TreatAsOrdinal{});
    }
}

Ordinal
Register::modify(Ordinal mask, Ordinal src) noexcept {
    auto tmp = o;
    o = ::modify(mask, src, o);
    return tmp;
}
