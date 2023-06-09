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

#if 0

        template<typename T>
        void setValue(ByteOrdinal index, T value, TreatAsRegister) noexcept {
            get(index, TreatAsRegister{}).setValue(value, TreatAs<T>{});
        }
        template<typename T>
        T getValue(ByteOrdinal index, TreatAsRegister) const noexcept {
            return get(index, TreatAsRegister{}).getValue(TreatAs<T>{});
        }

        template<typename T>
        void setValue(ByteOrdinal index, T value) noexcept {
            setValue<T>(index, value, TreatAsRegister{});
        }
        template<typename T>
        T getValue(ByteOrdinal index) const noexcept {
            return getValue<T>(index, TreatAsRegister{});
        }

        template<typename T>
        void setValue(ByteOrdinal index, T value, TreatAsLongRegister) noexcept {
            get(index, TreatAsLongRegister{}).setValue(value, TreatAs<T>{});
        }
        template<typename T>
        T getValue(ByteOrdinal index, TreatAsLongRegister) const noexcept {
            return get(index, TreatAsLongRegister{}).getValue(TreatAs<T>{});
        }
#endif