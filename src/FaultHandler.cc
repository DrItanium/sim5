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


#include "Core.h"


void
Core::handleFault(Ordinal faultType, Ordinal dat0, Ordinal dat1, Ordinal dat2) noexcept {
    /// @todo implement saving of the resumption record if it makes sense
    /// @todo implement proper return ip address support
    saveFaultRecord(faultType);
    if (auto& entry = getFaultTableEntry(faultType); entry.isLocalEntry()) {
        /// @todo add support for resuming at the current instruction if it makes sense
        callx(entry.getHandlerAddress());
        getGPR(PFPIndex).pfp.rt = 0b001;
    } else if (entry.isSystemProcedureTableEntry()) {
        /// @todo get the fault table number from the table entry and then jump
        /// to the segment table to find the system procedure table
        auto systemProcedureNumber = entry.getHandlerAddress();

    } else {
        /// @todo what to do here? Fail out?
    }
}

void
Core::saveFaultRecord(Ordinal faultType, Ordinal dat0, Ordinal dat1, Ordinal dat2) noexcept {
    // generate a fault record and on the top of the stack that the processor
    // is currently using.
    // okay so in both cases we are going to store the fault record to the
    // current stack
    FaultRecord rec;
    rec.unused = { 0 };
    rec.faultData[0] = dat0;
    rec.faultData[1] = dat1;
    rec.faultData[2] = dat2;
    rec.unused1 = 0;
    rec.pc = pc_.o;
    rec.ac = ac_.o;
    rec.kind.whole = faultType;
    rec.addr = ip_.o;
    /// @todo check and push onto the stack
}

const FaultTableEntry& 
Core::getFaultTableEntry(Ordinal faultType) noexcept {
    auto type = static_cast<uint8_t>(faultType >> 16);
    volatile auto& theTable = ebi::load<FaultTable>(load(prcbAddress_ + 40, TreatAsOrdinal{}), TreatAs<FaultTable>{});
    return theTable.getFaultTableEntry(type);
}



/* Taken from the Kx manual:
 *
 * Algorithm for implicit, local call/return
 * When the selected fault-handler entry in the fault table is an entry type
 * 0b00 (local procedure), the processor performs the following action:
 *
 * 1. The processor stores a fault record on the top of the stack that the
 * process is currently using. The stack can be the local stack, the supervisor
 * stack, or the interrupt stack
 *
 * 2. If the fault caused an instruction to be suspended, the processor
 * includes an instruction-resumption record on the current stack and sets the
 * resume flag in the saved process controls.
 *
 * 3. The processor creates a new frame on the current stack, with the
 * frame-return status field set to 0b001
 *
 * 4. Using the procedure address from the selected fault-table entry, the
 * processor performs an implicit call-extended (callx) operation to the fault
 * handler.
 *
 * If the fault handler is not able to perform a recovery action, it performs
 * one of the actions described in the section earlier in this chaper titled
 * "Possible Fault-Handler Actions."
 *
 * If the handler action results in a recovery from the fault, a ret
 * instruction in the fault handler allow processor control to return 
 * to the program that was being worked on when the fault occurred. On the
 * return, the processor performs the following action:
 *
 * 1. The processor deallocates the stack frame created for the fault handler
 *
 * 2. The processor copies the arithmetic controls field from the fault record
 * into the arithmetic controls register in the processor.
 *
 * 3. The processor then resumes work on the program it was working on when the
 * fault occurred at the instruction in the return IP register.
 *
 * My Commentary:
 *
 * The ret instruction handles the return-from-fault state already. I just need
 * to hook up the implicit callx instruction and describe the actions to be
 * performed in common terms provided to me. In all cases, when we raise a
 * fault it is a callx instruction with extra things to perform ahead of time.
 */
