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

#include "MegaboardCore.h"

void 
MegaboardCore::checkForPendingInterrupts_impl() {

}

void MegaboardCore::sendIAC_impl(const iac::Message& msg) {

}

void MegaboardCore::synchronizeFaults() {

}

void MegaboardCore::flushRegisters() {

}

bool MegaboardCore::haveAvailableRegisterSet() noexcept {

}

void MegaboardCore::makeNewRegisterFrame() noexcept {

}

void MegaboardCore::saveRegisters(Ordinal fp) noexcept {

}

void MegaboardCore::restoreRegisters(Ordinal fp) noexcept {

}

void MegaboardCore::busLock() noexcept {

}

void MegaboardCore::busUnlock() noexcept {

}

Ordinal MegaboardCore::load_impl(Address address, TreatAsOrdinal) const noexcept {

    return 0;
}

Integer MegaboardCore::load_impl(Address address, TreatAsInteger) const noexcept {

    return 0;
}

ShortOrdinal MegaboardCore::load_impl(Address address, TreatAsShortOrdinal) const noexcept {

    return 0;
}

ShortInteger MegaboardCore::load_impl(Address address, TreatAsShortInteger) const noexcept {

    return 0;
}

ByteOrdinal MegaboardCore::load_impl(Address address, TreatAsByteOrdinal) const noexcept {

    return 0;
}

ByteInteger MegaboardCore::load_impl(Address address, TreatAsByteInteger) const noexcept {
    return 0;
}

void MegaboardCore::store_impl(Address address, Ordinal value, TreatAsOrdinal) noexcept {

}

void MegaboardCore::store_impl(Address address, Integer value, TreatAsInteger) noexcept {

}

void MegaboardCore::store_impl(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept {

}

void MegaboardCore::store_impl(Address address, ShortInteger value, TreatAsShortInteger) noexcept {

}

void MegaboardCore::store_impl(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept {

}

void MegaboardCore::store_impl(Address address, ByteInteger value, TreatAsByteInteger) noexcept {

}

void MegaboardCore::begin_impl() noexcept {

}

bool MegaboardCore::runExtendedSelfTests() noexcept {
    return true;
}

void MegaboardCore::generateFault_impl(Ordinal faultCode) noexcept {

}

void MegaboardCore::assertFailureState_impl() noexcept {

}

void MegaboardCore::deassertFailureState_impl() noexcept {

}


