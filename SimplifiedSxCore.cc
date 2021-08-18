// sim3
// Copyright (c) 2021, Joshua Scoggins
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
#include "SimplifiedSxCore.h"

void
SimplifiedSxCore::boot() {
    if (!initialized_) {
        initialized_ = true;
        auto q = loadQuad(0);
        systemAddressTableBase_ = q.getOrdinal(0);
        prcbBase_ = q.getOrdinal(1);
        // skip the check words
        ip_.setOrdinal(q.getOrdinal(3));
        executing_ = true;
        pc_.setPriority(31);
        pc_.setState(1); // needs to be set as interrupted
        getRegister(RegisterIndex::FP).setOrdinal(getInterruptStackPointer());
    }
}
Ordinal
SimplifiedSxCore::getSystemAddressTableBase() const noexcept {
    return systemAddressTableBase_;
}
Ordinal
SimplifiedSxCore::getPRCBPtrBase() const noexcept {
    return prcbBase_;
}
bool
SimplifiedSxCore::continueToExecute() const noexcept {
    return executing_;
}
void
SimplifiedSxCore::resetExecutionStatus() noexcept {
    executing_ = true;
}
void
SimplifiedSxCore::synchronizedStore(Core::Address destination, const DoubleRegister &value) noexcept {
    // no special IAC locations when dealing with long versions so cool beans
    store(destination, value.getLongOrdinal());
}
void
SimplifiedSxCore::synchronizedStore(Core::Address destination, const QuadRegister &value) noexcept {
    if (destination == 0xFF00'0010) {
        // there are special IAC messages we need to handle here
    } else {
        store(destination, value);
    }
}
void
SimplifiedSxCore::synchronizedStore(Core::Address destination, const Register &value) noexcept {
    // there is a lookup for an interrupt control register, in the Sx manual, we are going to ignore that for now
    store(destination, value.getOrdinal());
}
