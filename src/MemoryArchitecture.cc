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
//
// Created by jwscoggins on 6/8/23.
//
#include "Types.h"
#include "Core.h"

ShortOrdinal
Core::load(Address address, TreatAsShortOrdinal) const noexcept {
    // load the value and then discard the upper half, it will always be pointing to the lower half
    return static_cast<ShortOrdinal>(load(address, TreatAsOrdinal{}));
}

ShortInteger
Core::load(Address address, TreatAsShortInteger) const noexcept {
    return static_cast<ShortInteger>(load(address, TreatAsInteger{}));
}
ByteOrdinal
Core::load(Address address, TreatAsByteOrdinal) const noexcept {
    return static_cast<ByteOrdinal>(load(address, TreatAsOrdinal{}));
}

ByteInteger
Core::load(Address address, TreatAsByteInteger) const noexcept {
    return static_cast<ByteInteger>(load(address, TreatAsInteger{}));
}
