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

#ifndef IAC_H__
#define IAC_H__
#include "Types.h"

namespace iac {
struct Message {
    explicit Message(uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3) noexcept : messageType(w0 >> 24), field1(w0 >> 16), field2(w0), field3(w1), field4(w2), field5(w3) { }
    uint8_t messageType;
    uint8_t field1;
    uint16_t field2;
    uint32_t field3;
    uint32_t field4;
    uint32_t field5;
};
void dispatchInterrupt(uint8_t vector) noexcept;
void purgeInstructionCache() noexcept;
void reinitializeProcessor(Ordinal satBase, Ordinal prcbBase, Ordinal startIP) noexcept;
void setBreakpointRegister(Ordinal breakpointIp0, Ordinal breakpointIp1) noexcept;
void storeSystemBase(Ordinal destinationAddress) noexcept;
void testPendingInterrupts() noexcept;
// Kx related IACs
void freeze() noexcept;
void continueInitialization() noexcept;

// MC related IACs
void checkProcessNotice(Ordinal processSegmentSelectorBase) noexcept;
void flushLocalRegisters(Ordinal physicalStackPageAddress) noexcept;
void flushProcess() noexcept;
void flushTLB() noexcept;
void flushTLBPageTableEntry(Ordinal offsetFromSegmentBase, Ordinal ssofSegmentThatContainsPage) noexcept;
void flushTLBPhysicalPage(Ordinal basePhysicalAddressOfPage) noexcept;
void flushTLBSegmentEntry(Ordinal ssForSegment) noexcept;
void modifyProcessControls(Ordinal newProcessorControlWords, Ordinal mask) noexcept;
void preemptProcess() noexcept;
void restartProcessor(Ordinal segmentTableBase, Ordinal prcbBase) noexcept;
void stopProcessor() noexcept;
void storeProcessor() noexcept;
void warmstartProcessor(Ordinal segmentTableBase, Ordinal prcbBase) noexcept;
void send(const Message& message) noexcept;
} // end namespace iac

#endif // end !defined(IAC_H__)
