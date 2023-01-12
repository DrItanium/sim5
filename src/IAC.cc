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
Core::sendIAC(const iac::Message& message) noexcept {
    noInterrupts();
    /// @todo implement
    switch (message.messageType) {
        case 0x40: dispatchInterrupt(message.field1); break;
        case 0x41: testPendingInterrupts(); break;
        case 0x80: storeSystemBase(message.field3); break;
        case 0x81: restartProcessor(message.field3, message.field4); break;
        case 0x83: stopProcessor(); break;
        case 0x84: flushLocalRegisters(message.field3); break;
        case 0x85: preemptProcess(); break;
        case 0x86: storeProcessor(); break;
        case 0x87: flushProcess(); break;
        case 0x88: flushTLBPhysicalPage(message.field3); break;
        case 0x89: purgeInstructionCache(); break;
        case 0x8A: flushTLB(); break;
        case 0x8B: flushTLBSegmentEntry(message.field3); break;
        case 0x8C: flushTLBPageTableEntry(message.field3, message.field4); break;
        case 0x8D: modifyProcessControls(message.field3, message.field4); break;
        case 0x8E: warmstartProcessor(message.field3, message.field4); break;
        case 0x8F: setBreakpointRegister(message.field3, message.field4); break;
        case 0x90: checkProcessNotice(message.field3); break;
        case 0x91: freeze(); break;
        case 0x92: continueInitialization(); break;
        case 0x93: reinitializeProcessor(message.field3, message.field4, message.field5); break;
        default: /* do nothing */ break;
    }
    interrupts();
}
void 
Core::dispatchInterrupt(uint8_t vector) noexcept {
    /// @todo implement
}

void 
Core::purgeInstructionCache() noexcept {
    // do nothing right now
}
void 
Core::reinitializeProcessor(Ordinal satBase, Ordinal prcbBase, Ordinal startIP) noexcept {
    /// @todo implement
}
void 
Core::setBreakpointRegister(Ordinal breakpointIp0, Ordinal breakpointIp1) noexcept {
    /// @todo implement
}
void 
Core::storeSystemBase(Ordinal destinationAddress) noexcept {
    /// @todo implement
}
void 
Core::testPendingInterrupts() noexcept {
    /// @todo implement
    // perhaps call checkForPendingInterrupts?
}
void 
Core::checkProcessNotice(Ordinal processSegmentSelectorBase) noexcept {

    /// @todo implement
}
void 
Core::flushLocalRegisters(Ordinal physicalStackPageAddress) noexcept { 
    /// @todo implement
}
void 
Core::flushProcess() noexcept { 
    /// @todo implement
}
void 
Core::flushTLB() noexcept { 
    /// @todo implement
}
void 
Core::flushTLBPageTableEntry(Ordinal offsetFromSegmentBase, Ordinal ssofSegmentThatContainsPage) noexcept { 
    /// @todo implement
}
void 
Core::flushTLBPhysicalPage(Ordinal basePhysicalAddressOfPage) noexcept { 
    /// @todo implement
}
void 
Core::flushTLBSegmentEntry(Ordinal ssForSegment) noexcept { 
    /// @todo implement
}
void 
Core::modifyProcessControls(Ordinal newProcessorControlWords, Ordinal mask) noexcept { 
    /// @todo implement
}

void 
Core::freeze() noexcept {
    /// @todo implement
}
void 
Core::continueInitialization() noexcept {
    /// @todo implement
}

void
Core::preemptProcess() noexcept {
    /// @todo implement
}

void 
Core::restartProcessor(Ordinal segmentTableBase, Ordinal prcbBase) noexcept {
    /// @todo implement
}

void 
Core::stopProcessor() noexcept {

}
void 
Core::storeProcessor() noexcept {

}
void 
Core::warmstartProcessor(Ordinal segmentTableBase, Ordinal prcbBase) noexcept {

}
