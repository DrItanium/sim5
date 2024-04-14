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
// fault handling

void
Core::pushFaultRecord(Address baseStorageAddress, const FaultRecord& record) noexcept
{
    // okay so we have to stash this fault record into the area _before_ the
    // current frame pointer, this function assumes that fault table
    // frame pointer has already been setup. This function just does the store
    // operation itself
    //
    // I am not going to push a resumption record onto the stack because I
    // don't need it.
    store(baseStorageAddress, record.unused, TreatAsOrdinal{});
    store(baseStorageAddress + 4, record.overrideFaultData[0], TreatAsOrdinal{});
    store(baseStorageAddress + 8, record.overrideFaultData[1], TreatAsOrdinal{});
    store(baseStorageAddress + 12, record.overrideFaultData[2], TreatAsOrdinal{});
    store(baseStorageAddress + 16, record.faultData[0], TreatAsOrdinal{});
    store(baseStorageAddress + 20, record.faultData[1], TreatAsOrdinal{});
    store(baseStorageAddress + 24, record.faultData[2], TreatAsOrdinal{});
    store(baseStorageAddress + 28, record.overrideType, TreatAsOrdinal{});
    store(baseStorageAddress + 32, record.pc, TreatAsOrdinal{});
    store(baseStorageAddress + 36, record.ac, TreatAsOrdinal{});
    store(baseStorageAddress + 40, record.type, TreatAsOrdinal{});
    store(baseStorageAddress + 44, record.addr, TreatAsOrdinal{});
}

FaultTableEntry
Core::getFaultEntry(uint8_t index) const noexcept {
    auto faultTableBaseAddress = getFaultTableBaseAddress();
    auto maskedIndex = index & 0b0001'1111;
    auto realOffset = maskedIndex * (sizeof(FaultTableEntry));
    auto realAddress = faultTableBaseAddress + realOffset;

    return FaultTableEntry { load(realAddress, TreatAsLongOrdinal{}) };
}

SegmentDescriptor
Core::loadSegmentDescriptor(SegmentSelector selector) const noexcept {
    auto index = translateSegmentDescriptorToOffset(selector);
    // now that we have the table index, we just need to compute the base
    // offset
    auto baseIndex = systemAddressTableBase_;
    baseIndex += (index * sizeof(SegmentDescriptor));
    // so now we have the base address of the segment descriptor we want
    return SegmentDescriptor { load(baseIndex, TreatAsQuadOrdinal{}) };
}


void
Core::localProcedureEntry_FaultCall(const FaultRecord& record, Address address) noexcept {
    faultCallGeneric(record, address, getStackPointer());
}

void
Core::faultCallGeneric(const FaultRecord& record, Address address, Address stackPointer) noexcept {
    // first allocate a new frame on the stack that the processor is
    // currently using. Set the frame-return status field to 0b001
    //
    // allocate enough space before the start of the frame for the fault
    // record (and optionally a resumption record if necessary). Be lazy
    // and just allocate two frames worth of information to be on the safe
    // side! Three frames worth are necessary to make sure we have enough
    // padding.
    auto nextFrame = computeNextFrame<C*3, NotC>(stackPointer);
    auto fp = getGPRValue<Ordinal>(FPIndex);
    // save the current registers to the stack
    enterCall(fp);
    // manually setup the stack frame as needed
    auto& pfp = getGPR(PFPIndex);
    // clear the p bit as well to make sure
    pfp.setValue<Ordinal>(getGPRValue<Ordinal>(FPIndex) & ~0b1'111);
    pfp.pfp.rt = 0b001;
    setGPR(FPIndex, nextFrame, TreatAsOrdinal{});
    setStackPointer(nextFrame + 64, TreatAsOrdinal{});
    pushFaultRecord(nextFrame - 48, record);
    // no need to push a resumption record right now and set the resume flag in
    // the saved process controls
    setIP(address);
}


void
Core::generateFault(const FaultRecord& record) {
    if (record.saveReturnAddress) {
        saveReturnAddress(RIPIndex);
    }
    auto faultType = record.getFaultType();
    auto entry = getFaultEntry(faultType);
    // override faults are generated inside of this method
    // usually they are virtual memory faults of some kind while saving to disk
    // When I get to that point, I will then proceed to implement those fault
    // handlers
    if (entry.isLocalProcedureEntry()) {
        localProcedureEntry_FaultCall(record, entry.getFaultHandlerProcedureAddress());
    } else if (entry.isSystemTableEntry()) {
        procedureTableEntry_FaultCall(record, entry);
    } else {
        badFault(record);
    }
}




void
Core::procedureTableEntry_FaultCall(const FaultRecord& record, const FaultTableEntry& entry) noexcept {
    // okay, so we can go down different paths here
    // first thing to do is translate the procedure index into an absolute
    // address if it is tagged as a local procedure
    //
    // before that, we want to get the table entry to see what kind of
    // operation we are looking at!
    //
    // First, get the segment descriptor
    auto descriptor = loadSegmentDescriptor(entry.getSegmentSelector());
    // next, find the procedure number
    auto index = entry.getFaultHandlerProcedureNumber();
    /// @todo implement override fault if the segment descriptor is invalid or
    /// wrong for this target

    // now we can get the base offset table
    auto tableAddress = descriptor.getAddress();
    // get the starting offset to the procedure-table structure entries
    auto procedureEntry = load(tableAddress + 48 + index, TreatAsOrdinal{});
    auto procedureAddress = procedureEntry & 0xFFFF'FFFC;
    switch (procedureEntry & 0b11) {
        case 0b00:
            // okay so it is a local procedure entry, very easy to handle
            // overall. Just send this address over to the "local procedure"
            // version of this function
            localProcedureEntry_FaultCall(record, procedureAddress);
            break;
        case 0b10:
            supervisorProcedureTableEntry_FaultCall(record, procedureAddress, tableAddress);
            // this one is more complex, this one is like a calls instruction
            break;
        default:
            /// @todo handle this bad case!
            break;
    }

}

void
Core::supervisorProcedureTableEntry_FaultCall(const FaultRecord& record, Address procedureAddress, Address baseTableAddress) noexcept {
    Address baseStackAddress;
    if (pc_.inSupervisorMode()) {
        // already in supervisor mode so use the current stack
        baseStackAddress = getStackPointer();
    } else {
        // get the stack pointer from the current address
        baseStackAddress = load(baseTableAddress + 12, TreatAsOrdinal{});
        // switch to supervisor mode
        pc_.processControls.executionMode = 1;
        // we are now in supervisor mode :D
    }
    // do the trace enable actions
    if (record.clearTraceEnableBit()) {
        pc_.processControls.traceEnable = 0;
    } else {
        // transfer the trace enableBit as needed
        pc_.processControls.traceEnable = baseStackAddress & 0b1;
    }
    // now that we have the right area to be in, lets do the FaultCall as
    // though it is a local one!
    auto temp = baseStackAddress & 0xFFFF'FFFC;
    // at this point, it is safe to call the generic handler
    faultCallGeneric(record, procedureAddress, temp);
}


OptionalFaultRecord
Core::faultOnOverflow(Register& dest) {
    /// @todo implement
    return std::nullopt;
}
