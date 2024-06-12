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

void
Core::enterCall(Ordinal fp) {
    // make sure that we synchronize the frame pointer at the time of entering the call
    // this makes sure that the key we tie the set to is properly synchronized with system state
    // if you don't do this, then the old key will be used to save registers
    currentLocalRegisterSet().synchronizeOwnership(fp);
    getNextPack().takeOwnership(fp, [this](const RegisterFrame& frame, Address address) { saveRegisterFrame(frame, address); });
    ++localRegisterFrameIndex_;
    localRegisterFrameIndex_ %= NumberOfLocalRegisterFrames;
}
void
Core::leaveCall() {
    // perform the transfer and modification
    moveGPR<Ordinal>(FPIndex, PFPIndex, [](Ordinal input) -> Ordinal { return Register{input}.getPFPAddress(); }, TreatAsOrdinal{});
    auto saveRegistersToStack = [this](const RegisterFrame& frame, Address targetAddress) {
        saveRegisterFrame(frame, targetAddress);
    };
    // we are done with the current frame so just relinquish ownership of it
    currentLocalRegisterSet().relinquishOwnership();
    getPreviousPack().restoreOwnership(getFramePointerAddress(),
                                       saveRegistersToStack,
                                       [this](RegisterFrame& frame, Address targetAddress) {
                                           restoreRegisterFrame(frame, targetAddress);
                                       });
    --localRegisterFrameIndex_;
    localRegisterFrameIndex_ %= NumberOfLocalRegisterFrames;
}

void
Core::setupNewFrameInternals(Ordinal fp, Ordinal temp) noexcept {
    setGPR(PFPIndex, fp, TreatAsOrdinal{});
    setGPR(FPIndex, temp, TreatAsOrdinal{});
    setStackPointer(temp + 64, TreatAsOrdinal{});
}


OptionalFaultRecord
Core::callx(Address effectiveAddress) {
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getFramePointerAddress();
    balx(RIPIndex, effectiveAddress);
    enterCall(fp);
    setupNewFrameInternals(fp, temp);
    return std::nullopt;
}


OptionalFaultRecord
Core::call(Integer displacement) {
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getFramePointerAddress();
    saveReturnAddress(RIPIndex);
    enterCall(fp);
    setupNewFrameInternals(fp, temp);
    branch(displacement);
    return std::nullopt;
}
OptionalFaultRecord
Core::calls() {
    return calls(static_cast<Ordinal>(getSrc1Register(_regInstruction)));
}
OptionalFaultRecord
Core::calls(Ordinal src1) {
    if (auto targ = src1; targ > 259) {
        return protectionLengthFault();
    } else {
        syncf();
        auto tempPE = load<Ordinal>(getSystemProcedureTableBase() + 48 + (4 * targ));
        auto type = static_cast<uint8_t>(tempPE & 0b11);
        auto procedureAddress = tempPE & ~0b11;
        // read entry from system-procedure table, where spbase is address of
        // system-procedure table from Initial Memory Image
        balx(RIPIndex, procedureAddress);
        Ordinal temp = 0;
        auto fp = getFramePointerAddress();
        Register copy(fp);
        if ((type == 0b00) || pc_.inSupervisorMode()) {
            temp = getNextFrameBase();
            copy.pfp.rt = 0;
        } else {
            temp = getSupervisorStackPointer();
            copy.pfp.rt = 0b010 | (pc_.processControls.traceEnable ? 0b001 : 0);
            pc_.processControls.executionMode = 1;
            pc_.processControls.traceEnable = temp & 0b1;
            temp &= 0xFFFF'FFFC; // clear the lowest two bits after being done
            // here. If trace is active then that could
            // cause problems overall with the address
            // offset
        }
        // make a copy of the frame pointer and then modify it
        enterCall(fp);
        setupNewFrameInternals(copy.getValue<Ordinal>(), temp);
    }
    return std::nullopt;
}
void
Core::localReturn() {
    restoreStandardFrame();
}
Ordinal
Core::restoreACFromStack(Ordinal fp) {
    return load<Ordinal>(fp - 12);
}
Ordinal
Core::restorePCFromStack(Ordinal fp) {
    return load<Ordinal>(fp - 16);
}
void
Core::faultReturn() {
    auto fp = getFramePointerAddress();
    auto oldPC = restorePCFromStack(fp);
    auto oldAC = restoreACFromStack(fp);
    restoreStandardFrame();
    ac_.setValue(oldAC, TreatAsOrdinal{});
    if (pc_.inSupervisorMode()) {
        pc_.setValue(oldPC, TreatAsOrdinal{});
    }
}
void
Core::supervisorReturn(bool traceModeSetting) {
    if (pc_.inSupervisorMode()) {
        pc_.processControls.traceEnable = traceModeSetting;
        pc_.processControls.executionMode = 0;
    }
    restoreStandardFrame();
}
void
Core::interruptReturn() {
    // interrupt return
    auto fp = getFramePointerAddress();
    auto oldPC = restorePCFromStack(fp);
    auto oldAC = restoreACFromStack(fp);
    restoreStandardFrame();
    ac_.setValue(oldAC, TreatAsOrdinal{});
    if (pc_.inSupervisorMode()) {
        pc_.setValue(oldPC, TreatAsOrdinal{});
        checkForPendingInterrupts();
    }
}
OptionalFaultRecord
Core::ret() {
    syncf();
    auto& pfp = getGPR(PFPIndex);
    switch (pfp.getReturnType()) {
        case 0b000:
            localReturn();
            break;
        case 0b001:
            faultReturn();
            break;
        case 0b010:
            supervisorReturn(false);
            break;
        case 0b011:
            supervisorReturn(true);
            break;
        case 0b111:
            interruptReturn();
            break;
        default:
            // undefined!
            return unimplementedFault();
    }
    return std::nullopt;
}


void
Core::restoreStandardFrame() noexcept {

    leaveCall();
    restoreRIPToIP();
    advanceInstruction_ = false;
}

void
Core::restoreRIPToIP() {
    setIP(getRIPContents());
}

void
Core::flushreg() {
    if constexpr (NumberOfLocalRegisterFrames > 1) {
        for (auto curr = localRegisterFrameIndex_+ 1; curr != localRegisterFrameIndex_; curr = ((curr + 1) % NumberOfLocalRegisterFrames)) {
            frames_[curr].relinquishOwnership([this](const RegisterFrame& frame, Address dest) noexcept { saveRegisterFrame(frame, dest); });
        }
    }

    // if we only have a single frame then do not bother with flushing as it doesn't make sense at all!
}
void
Core::saveRegisterFrame(const RegisterFrame& theFrame, Address baseAddress) {
    for (auto i = 0; i < 16; ++i, baseAddress += 4) {
        store<Ordinal>(baseAddress, theFrame.get(i, TreatAsRegister{}).getValue(TreatAsOrdinal{}));
    }
}
void
Core::restoreRegisterFrame(RegisterFrame& theFrame, Address baseAddress) {
    for (auto i = 0; i < 16; ++i, baseAddress += 4) {
        theFrame.get(i, TreatAsRegister{}).setValue(load<Ordinal>(baseAddress), TreatAsOrdinal {});
    }
}

void
Core::saveReturnAddress(Register& linkRegister) noexcept {
    linkRegister.setValue<Ordinal>(ip_.getValue<Ordinal>() + instructionLength_);
}

void
Core::saveReturnAddress(ByteOrdinal linkRegister) noexcept {
    setGPR(linkRegister, ip_.getValue<Ordinal>() + instructionLength_, TreatAsOrdinal{});
}

void
Core::balx(ByteOrdinal linkRegister, Ordinal branchTo) {
    saveReturnAddress(linkRegister);
    setIP(branchTo);
}

void
Core::balx(Register& linkRegister, Ordinal branchTo) {
    saveReturnAddress(linkRegister);
    setIP(branchTo);
}

void
Core::bal(Integer displacement) {
    saveReturnAddress(LRIndex);
    branch(displacement);
}
void
Core::bal() {
    bal(_ctrlInstruction.getDisplacement());
}

void
Core::bx(Address effectiveAddress) {
    setIP(effectiveAddress);
}
void
Core::bx() {
    bx(computeAddress());
}

void
Core::b() {
    branch(_ctrlInstruction.getDisplacement());
}
OptionalFaultRecord
Core::call() {
    return call(_ctrlInstruction.getDisplacement());
}
void
Core::balx() {
    balx(getGPR(_memInstruction.getSrcDest()), computeAddress());
}

OptionalFaultRecord
Core::callx() {
    return callx(computeAddress());
}