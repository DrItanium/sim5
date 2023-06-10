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

void
Core::enterCall(Ordinal fp) {
    getNextPack().takeOwnership(fp, [this](const RegisterFrame& frame, Address address) { saveRegisterFrame(frame, address); });
    ++localRegisterFrameIndex_;
    localRegisterFrameIndex_ %= NumberOfLocalRegisterFrames;

}
void
Core::leaveCall() {
    // perform the transfer and modification
    moveGPR<Ordinal>(FPIndex, PFPIndex, [](Ordinal input) -> Ordinal { return Register{input}.getPFPAddress(); } , TreatAsOrdinal {});
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
    DEBUG_ENTER_FUNCTION;
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getFramePointerAddress();
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": (fp): 0x" << std::hex << fp << std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": (ea): 0x" << std::hex << effectiveAddress << std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": (rip before): 0x" << std::hex << getGPRValue<Ordinal>(RIPIndex) << std::endl;
    }
    balx(RIPIndex, effectiveAddress);
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": (rip after): 0x" << std::hex << getGPRValue<Ordinal>(RIPIndex) << std::endl;
    }
    enterCall(fp);
    setupNewFrameInternals(fp, temp);
    DEBUG_LEAVE_FUNCTION;
}


void
Core::call(Integer displacement) noexcept {
    DEBUG_ENTER_FUNCTION;
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getFramePointerAddress();
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": (fp): 0x" << std::hex << fp << std::endl;
    }
    saveReturnAddress(RIPIndex);
    enterCall(fp);
    setupNewFrameInternals(fp, temp);
    branch(displacement);
    DEBUG_LEAVE_FUNCTION;
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
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": (rip before): 0x" << std::hex << getGPRValue<Ordinal>(RIPIndex) << std::endl;
    }
   restoreStandardFrame();
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": (rip after): 0x" << std::hex << getGPRValue<Ordinal>(RIPIndex) << std::endl;
    }
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
    DEBUG_ENTER_FUNCTION;
    syncf();
    auto& pfp = getGPR(PFPIndex);
    switch (pfp.getReturnType()) {
        case 0b000:
            DEBUG_LOG_LEVEL(2) std::cout << __PRETTY_FUNCTION__ << ": local return" << std::endl;
            localReturn();
            break;
        case 0b001:
            DEBUG_LOG_LEVEL(2) std::cout << __PRETTY_FUNCTION__ << ": fault return" << std::endl;
            faultReturn();
            break;
        case 0b010:
            DEBUG_LOG_LEVEL(2) std::cout << __PRETTY_FUNCTION__ << ": supervisor return" << std::endl;
            supervisorReturn(false);
            break;
        case 0b011:
            DEBUG_LOG_LEVEL(2) std::cout << __PRETTY_FUNCTION__ << ": supervisor-trace return" << std::endl;
            supervisorReturn(true);
            break;
        case 0b111:
            DEBUG_LOG_LEVEL(2) std::cout << __PRETTY_FUNCTION__ << ": interrupt return" << std::endl;
            interruptReturn();
            break;
        default:
            // undefined!
            unimplementedFault();
            break;
    }
    DEBUG_LEAVE_FUNCTION;
}


void
Core::restoreStandardFrame() noexcept {
    leaveCall();
    restoreRIPToIP();
    advanceInstruction_ = false;
}

void
Core::restoreRIPToIP() {
    setIP(getRIPContents(), TreatAsOrdinal{});
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
