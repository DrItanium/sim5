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

#include "IAC.h"
#include "Core.h"

void
Core::sendIAC(const iac::Message& msg) noexcept {
    DEBUG_ENTER_FUNCTION;
    /// @todo implement
    switch (msg.messageType) {
        /// @todo implement different message types
        case 0x40: // dispatch interrupt
            dispatchInterrupt(msg.field1);
            break;
        case 0x89: // purge instruction cache
            purgeInstructionCache();
            break;
        case 0x93: // reinitialize processor
            reinitializeProcessor(msg.field3, msg.field4, msg.field5);
            break;
        case 0x8f: // set breakpoint register
            setBreakpointRegister(msg.field3, msg.field4);
            break;
        case 0x80: // store system base
            storeSystemBase(msg.field3);
            break;
        case 0x41: // test for pending interrupts
            testPendingInterrupts();
            break;
        default:
            break;
    }
    DEBUG_LEAVE_FUNCTION;
}

void
Core::reinitializeProcessor(Ordinal satBase, Ordinal prcbBase, Ordinal startIP) noexcept {
    DEBUG_ENTER_FUNCTION;
    boot0(satBase, prcbBase, startIP);
    DEBUG_LEAVE_FUNCTION;
}

void
Core::setBreakpointRegister(Ordinal breakpointIp0, Ordinal breakpointIp1) noexcept {
    /// @todo do something with the breakpoint data
    breakpoint0_ = breakpointIp0 & 0xFFFFFFFC;
    breakpoint0Active_ = (breakpointIp0 & 0b10) != 0;
    breakpoint1_ = breakpointIp1 & 0xFFFFFFFC;
    breakpoint1Active_ = (breakpointIp1 & 0b10) != 0;
}

void
Core::storeSystemBase(Ordinal destinationAddress) noexcept {
    store(destinationAddress, systemAddressTableBase_, TreatAsOrdinal{});
    store(destinationAddress+4, prcbAddress_, TreatAsOrdinal{});
}

