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
Core::enterCall(Ordinal fp) noexcept {
    if (!registerSetAvailable()) {
        saveRegisterSet(fp);
    }
    allocateNewRegisterFrame();
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
    // need to leave the current call
    restoreFramePointerOnReturn();
    // remember that the lowest 6 bits are ignored so it is important to mask
    // them out of the frame pointer address when using the address
    restoreRegisterSet(getFramePointerAddress() & NotC);
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
