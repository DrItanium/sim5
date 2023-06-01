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

#ifndef SIM5_MEGABOARD_CORE_H__
#define SIM5_MEGABOARD_CORE_H__
#include "Core.h"

class MegaboardCore : public Core<MegaboardCore> {
    public:
        void checkForPendingInterrupts_impl();
        void sendIAC_impl(const iac::Message& msg);
        void synchronizeFaults();
        void flushRegisters();
        bool haveAvailableRegisterSet() noexcept;
        void makeNewRegisterFrame() noexcept;
        void saveRegisters(Ordinal fp) noexcept;
        void restoreRegisters(Ordinal fp) noexcept;
        void busLock() noexcept;
        void busUnlock() noexcept;
        Ordinal load_impl(Address address, TreatAsOrdinal) const noexcept;
        Integer load_impl(Address address, TreatAsInteger) const noexcept;
        ShortOrdinal load_impl(Address address, TreatAsShortOrdinal) const noexcept;
        ShortInteger load_impl(Address address, TreatAsShortInteger) const noexcept;
        ByteOrdinal load_impl(Address address, TreatAsByteOrdinal) const noexcept;
        ByteInteger load_impl(Address address, TreatAsByteInteger) const noexcept;
        void store_impl(Address address, Ordinal value, TreatAsOrdinal) noexcept;
        void store_impl(Address address, Integer value, TreatAsInteger) noexcept;
        void store_impl(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept;
        void store_impl(Address address, ShortInteger value, TreatAsShortInteger) noexcept;
        void store_impl(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept;
        void store_impl(Address address, ByteInteger value, TreatAsByteInteger) noexcept;
        void begin_impl() noexcept;
        bool runExtendedSelfTests() noexcept;
        void generateFault_impl(Ordinal faultCode) noexcept;
        void assertFailureState_impl() noexcept;
        void deassertFailureState_impl() noexcept;
    private:
        void setBankRegisters(Address addr) const noexcept;
};
#endif
