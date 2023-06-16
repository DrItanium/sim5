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
    DEBUG_ENTER_FUNCTION;
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": FP is now 0x" << std::hex << fp << std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": Frame Index: 0x" << std::hex << static_cast<int>(localRegisterFrameIndex_) << std::endl;
    }
    // make sure that we synchronize the frame pointer at the time of entering the call
    // this makes sure that the key we tie the set to is properly synchronized with system state
    // if you don't do this, then the old key will be used to save registers
    currentLocalRegisterSet().synchronizeOwnership(fp);
    getNextPack().takeOwnership(fp, [this](const RegisterFrame& frame, Address address) { saveRegisterFrame(frame, address); });
    ++localRegisterFrameIndex_;
    localRegisterFrameIndex_ %= NumberOfLocalRegisterFrames;
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": Frame Index: 0x" << std::hex << static_cast<int>(localRegisterFrameIndex_) << std::endl;
    }
    DEBUG_LEAVE_FUNCTION;

}
void
Core::leaveCall() {
    DEBUG_ENTER_FUNCTION;
    // perform the transfer and modification
    DEBUG_LOG_LEVEL(2) {
        auto fp = getFramePointerAddress();
        std::cout << __PRETTY_FUNCTION__ << ": FP is now 0x" << std::hex << fp << std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": transferring PFP back to FP" << std::endl;
    }
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": Frame Index: 0x" << std::hex << static_cast<int>(localRegisterFrameIndex_) << std::endl;
    }
    moveGPR<Ordinal>(FPIndex, PFPIndex, [](Ordinal input) -> Ordinal { return Register{input}.getPFPAddress(); }, TreatAsOrdinal{});
    DEBUG_LOG_LEVEL(2) {
        auto fp = getFramePointerAddress();
        std::cout << __PRETTY_FUNCTION__ << ": FP is now 0x" << std::hex << fp << std::endl;
    }
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
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": Frame Index: 0x" << std::hex << static_cast<int>(localRegisterFrameIndex_) << std::endl;
    }
    DEBUG_LEAVE_FUNCTION;
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
    DEBUG_LOG_LEVEL(1) {
        auto g0 = getGPRValue<Ordinal>(GlobalRegisterBase + 0);
        auto g1 = getGPRValue<Ordinal>(GlobalRegisterBase + 1);
        auto g2 = getGPRValue<Ordinal>(GlobalRegisterBase + 2);
        auto g3 = getGPRValue<Ordinal>(GlobalRegisterBase + 3);
        auto g4 = getGPRValue<Ordinal>(GlobalRegisterBase + 4);
        auto g5 = getGPRValue<Ordinal>(GlobalRegisterBase + 5);
        auto g6 = getGPRValue<Ordinal>(GlobalRegisterBase + 6);
        auto g7 = getGPRValue<Ordinal>(GlobalRegisterBase + 7);
        std::cout << __PRETTY_FUNCTION__ << ": (0x" << g0 <<
                  ", 0x" << g1 <<
                  ", 0x" << g2 <<
                  ", 0x" << g3 <<
                  ", 0x" << g4 <<
                  ", 0x" << g5 <<
                  ", 0x" << g6 <<
                  ", 0x" << g7 <<
                  ")" << std::endl;

        std::cout << __PRETTY_FUNCTION__ << ": (ea): 0x" << std::hex << effectiveAddress << std::endl;
    }
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": (fp): 0x" << std::hex << fp << std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": (newFP): 0x" << std::hex << temp << std::endl;
        auto rip = getGPRValue<Ordinal>(RIPIndex);
        std::cout << __PRETTY_FUNCTION__ << ": (rip before): 0x" << std::hex << rip << std::endl;
    }
    balx(RIPIndex, effectiveAddress);
    DEBUG_LOG_LEVEL(2) {
        auto rip = getGPRValue<Ordinal>(RIPIndex);
        std::cout << __PRETTY_FUNCTION__ << ": (rip after): 0x" << std::hex << rip << std::endl;
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
    DEBUG_LOG_LEVEL(1) {
        auto g0 = getGPRValue<Ordinal>(GlobalRegisterBase + 0);
        auto g1 = getGPRValue<Ordinal>(GlobalRegisterBase + 1);
        auto g2 = getGPRValue<Ordinal>(GlobalRegisterBase + 2);
        auto g3 = getGPRValue<Ordinal>(GlobalRegisterBase + 3);
        auto g4 = getGPRValue<Ordinal>(GlobalRegisterBase + 4);
        auto g5 = getGPRValue<Ordinal>(GlobalRegisterBase + 5);
        auto g6 = getGPRValue<Ordinal>(GlobalRegisterBase + 6);
        auto g7 = getGPRValue<Ordinal>(GlobalRegisterBase + 7);
        std::cout << __PRETTY_FUNCTION__ << ": (0x" << g0 <<
                  ", 0x" << g1 <<
                  ", 0x" << g2 <<
                  ", 0x" << g3 <<
                  ", 0x" << g4 <<
                  ", 0x" << g5 <<
                  ", 0x" << g6 <<
                  ", 0x" << g7 <<
                  ")" << std::endl;
    }
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": (fp): 0x" << std::hex << fp << std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": (disp): 0x" << std::hex << displacement << std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": (newFP): 0x" << std::hex << temp << std::endl;
        auto rip = getGPRValue<Ordinal>(RIPIndex);
        std::cout << __PRETTY_FUNCTION__ << ": (rip before): 0x" << std::hex << rip << std::endl;
    }
    saveReturnAddress(RIPIndex);
    DEBUG_LOG_LEVEL(2) {
        auto rip = getGPRValue<Ordinal>(RIPIndex);
        std::cout << __PRETTY_FUNCTION__ << ": (rip after): 0x" << std::hex << rip << std::endl;
    }
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
        DEBUG_LOG_LEVEL(1) {
            auto g0 = getGPRValue<Ordinal>(GlobalRegisterBase + 0);
            auto g1 = getGPRValue<Ordinal>(GlobalRegisterBase + 1);
            auto g2 = getGPRValue<Ordinal>(GlobalRegisterBase + 2);
            auto g3 = getGPRValue<Ordinal>(GlobalRegisterBase + 3);
            auto g4 = getGPRValue<Ordinal>(GlobalRegisterBase + 4);
            auto g5 = getGPRValue<Ordinal>(GlobalRegisterBase + 5);
            auto g6 = getGPRValue<Ordinal>(GlobalRegisterBase + 6);
            auto g7 = getGPRValue<Ordinal>(GlobalRegisterBase + 7);
            std::cout << __PRETTY_FUNCTION__ << ": (0x" << g0 <<
                      ", 0x" << g1 <<
                      ", 0x" << g2 <<
                      ", 0x" << g3 <<
                      ", 0x" << g4 <<
                      ", 0x" << g5 <<
                      ", 0x" << g6 <<
                      ", 0x" << g7 <<
                      ")" << std::endl;
        }
    }
}
void
Core::localReturn() {
    DEBUG_LOG_LEVEL(2) {
        auto value = getGPRValue<Ordinal>(RIPIndex);
        std::cout << __PRETTY_FUNCTION__ << ": (rip before): 0x" << std::hex << value << std::endl;
    }
    restoreStandardFrame();
    DEBUG_LOG_LEVEL(2) {
        auto value = getGPRValue<Ordinal>(RIPIndex);
        std::cout << __PRETTY_FUNCTION__ << ": (rip after): 0x" << std::hex << value <<  std::endl;
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
    DEBUG_ENTER_FUNCTION;
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": calling leaveCall" << std::endl;
    }

    leaveCall();
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": calling restoreRIPToIP" << std::endl;
    }
    restoreRIPToIP();
    advanceInstruction_ = false;
    DEBUG_LEAVE_FUNCTION;
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

void
Core::saveReturnAddress(Register& linkRegister) noexcept {
    linkRegister.setValue<Ordinal>(ip_.getValue(TreatAsOrdinal{}) + instructionLength_);
}

void
Core::saveReturnAddress(ByteOrdinal linkRegister) noexcept {
    setGPR(linkRegister, ip_.getValue(TreatAsOrdinal{}) + instructionLength_, TreatAsOrdinal{});
}

void
Core::balx(ByteOrdinal linkRegister, Ordinal branchTo) noexcept {
    saveReturnAddress(linkRegister);
    setIP(branchTo, TreatAsOrdinal{});
}

void
Core::balx(Register& linkRegister, Ordinal branchTo) noexcept {
    saveReturnAddress(linkRegister);
    setIP(branchTo, TreatAsOrdinal{});
}

void
Core::bal(Integer displacement) {
    DEBUG_ENTER_FUNCTION;
    saveReturnAddress(LRIndex);
    DEBUG_LOG_LEVEL(3) {
        std::cout << "\t" << __PRETTY_FUNCTION__ << ", lr: 0x" << getGPRValue(LRIndex, TreatAsOrdinal{}) << std::endl;
    }
    branch(displacement);
    DEBUG_LEAVE_FUNCTION;
}

void
Core::bx(Address effectiveAddress) {
    DEBUG_ENTER_FUNCTION;
    setIP(effectiveAddress, TreatAsOrdinal{});
    DEBUG_LEAVE_FUNCTION;
}
