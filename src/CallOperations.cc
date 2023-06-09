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

bool
Core::registerSetAvailable() noexcept {
    return false;
}
void
Core::enterCall(Ordinal fp) {
    getNextPack().takeOwnership(fp, [this](const RegisterFrame& frame, Address address) { saveRegisterFrame(frame, address); });
    ++localRegisterFrameIndex_;
    localRegisterFrameIndex_ %= NumberOfLocalRegisterFrames;

}
void
Core::leaveCall() {
    auto fn = [](Ordinal input) -> Ordinal {
        // we need to clear the lowest 6 bits
        Register tmp{input};
        tmp.pfpAddress.align = 0;
        return tmp.getValue<Ordinal>();
    };
    // perform the transfer and modification
    moveGPR<Ordinal>(FPIndex, PFPIndex, fn, TreatAsOrdinal {});
    auto targetAddress = getFramePointerAddress();
    frames_[localRegisterFrameIndex_].relinquishOwnership();
    getPreviousPack().restoreOwnership(targetAddress,
                                       [this](const RegisterFrame& frame, Address targetAddress) { saveRegisterFrame(frame, targetAddress); },
                                       [this](RegisterFrame& frame, Address targetAddress) { restoreRegisterFrame(frame, targetAddress); });
    --localRegisterFrameIndex_;
    localRegisterFrameIndex_ %= NumberOfLocalRegisterFrames;
}

void
Core::setupNewFrameInternals(Ordinal fp, Ordinal temp) noexcept {
    setGPR(PFPIndex, fp, TreatAsOrdinal{});
    setGPR(FPIndex, temp, TreatAsOrdinal{});
    setStackPointer(temp + 64, TreatAsOrdinal{});
}


void
Core::callx(Address effectiveAddress) noexcept {
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getFramePointerAddress();
    balx(RIPIndex, effectiveAddress);
    enterCall(fp);
    setupNewFrameInternals(fp, temp);
}


void
Core::call(Integer displacement) noexcept {
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getFramePointerAddress();
    saveReturnAddress(RIPIndex);
    enterCall(fp);
    setupNewFrameInternals(fp, temp);
    branch(displacement);
}

void
Core::calls(Ordinal src1) noexcept {
    if (auto targ = src1; targ > 259) {
        protectionLengthFault();
        //generateFault(ProtectionLengthFault);
    } else {
        syncf();
        auto tempPE = load(getSystemProcedureTableBase() + 48 + (4 * targ), TreatAsOrdinal{});
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
}
void
Core::localReturn() {
   restoreStandardFrame();
}
Ordinal
Core::restoreACFromStack(Ordinal fp) {
    return load(fp - 12, TreatAsOrdinal{});
}
Ordinal
Core::restorePCFromStack(Ordinal fp) {
    return load(fp - 16, TreatAsOrdinal{});
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
void
Core::ret() noexcept {
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
            unimplementedFault();
            break;
    }
}

void
Core::allocateNewRegisterFrame() noexcept {
    // making a new register frame is not necessary for this implementation
}

void
Core::restoreStandardFrame() noexcept {
    leaveCall();
    restoreRIPToIP();
    advanceInstruction_ = false;
}
void
Core::restoreFramePointerOnReturn() {
    moveGPR(FPIndex, PFPIndex, TreatAsOrdinal{});
}

void
Core::restoreRIPToIP() {
    setIP(getRIPContents(), TreatAsOrdinal{});
}
void
Core::saveRegisterSet(Ordinal fp) noexcept {

}

void
Core::restoreRegisterSet(Ordinal fp) noexcept {

}

void
Core::flushreg() noexcept {
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
        store(baseAddress, theFrame.get(i, TreatAsRegister{}).getValue(TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
void
Core::restoreRegisterFrame(RegisterFrame& theFrame, Address baseAddress) {
    for (auto i = 0; i < 16; ++i, baseAddress += 4) {
        theFrame.get(i, TreatAsRegister{}).setValue(load(baseAddress, TreatAsOrdinal{}), TreatAsOrdinal {});
    }
}
#if 0
void
Core::GPRBlock::saveLocalRegisters(Address fp, Core& core) {
    for (int i = 0, j = 0; i < 16; ++i, j+= 4) {
        core.store(fp + j, localRegisters().get(i, TreatAsRegister{}).getValue<Ordinal>(), TreatAsOrdinal{});
    }
}
void
Core::GPRBlock::restoreLocalRegisters(Address fp, Core& core) {
    for (int i = 0, j = 0; i < 16; ++i, j+= 4) {
        localRegisters().get(i, TreatAsRegister{}) = core.load(fp + j, TreatAsOrdinal{});
    }
}

void
Core::GPRBlock::enterCall(Ordinal fp, Core& core) {
    if constexpr (NumberOfLocalRegisterFrames > 1) {
        // okay so we actually have a rotating set of elements
        // first increment the frame index
        ++_currentLocalFrameIndex;
        // modulo the number of local register frames
        _currentLocalFrameIndex %= NumberOfLocalRegisterFrames;
    }
    // now we can get this new set and check to see if it is valid or not
    auto& currentLocalRegisterFrame = currentLocalRegisterEntry();
    if (currentLocalRegisterFrame._valid) {
        currentLocalRegisterFrame.commit(core);
    }
    currentLocalRegisterFrame._valid = true;
    currentLocalRegisterFrame._targetFramePointer = fp;
}

void
Core::GPRBlock::exitCall(Ordinal fp, Core& core) {
    if constexpr (NumberOfLocalRegisterFrames > 1) {
        // since we are exiting a call, decrement the frame index (with wraparound) to simulate the call chain
        --_currentLocalFrameIndex;
        _currentLocalFrameIndex %= NumberOfLocalRegisterFrames;
    }
    // get the current local register frame
    auto& currentLocalRegisterFrame = currentLocalRegisterEntry();
    if (currentLocalRegisterFrame._targetFramePointer != fp) {
        if (currentLocalRegisterFrame._valid) {
            currentLocalRegisterFrame.commit(core);
        }
        currentLocalRegisterFrame._valid = true;
        currentLocalRegisterFrame._targetFramePointer = fp;
        currentLocalRegisterFrame.restore(core);
    }

}


void
Core::GPRBlock::flushLocalRegisters() {

}

void
Core::GPRBlock::RegisterFrameWay::commit(Core& core) {

}

void
Core::GPRBlock::RegisterFrameWay::restore(Core& core) {

}


void
Core::GPRBlock::reinitializeLocalRegisterCache() {
    // just go through and clear out all registers
    for (auto i = 0; i < NumberOfLocalRegisterFrames; ++i) {

    }
    _locals[0]._valid = true;
}
#endif
