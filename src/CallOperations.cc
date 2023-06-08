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
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include "Types.h"
#include "Core.h"
#include "BinaryOperations.h"

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
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    balx(RIPIndex, effectiveAddress);
    enterCall(fp);
    setupNewFrameInternals(fp, temp);
}


void
Core::call(Integer displacement) noexcept {
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    saveReturnAddress(RIPIndex);
    enterCall(fp);
    branch(displacement);
    setupNewFrameInternals(fp, temp);
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
        ByteOrdinal tempRRR = 0;
        if ((type == 0b00) || pc_.inSupervisorMode()) {
            temp = getNextFrameBase();
            tempRRR = 0;
        } else {
            temp = getSupervisorStackPointer();
            tempRRR = 0b010 | (pc_.processControls.traceEnable ? 0b001 : 0);
            pc_.processControls.executionMode = 1;
            pc_.processControls.traceEnable = temp & 0b1;
            temp &= 0xFFFF'FFFC; // clear the lowest two bits after being done
            // here. If trace is active then that could
            // cause problems overall with the address
            // offset
        }
        enterCall(temp);
        /// @todo expand pfp and fp to accurately model how this works
        auto& pfp = getGPR(PFPIndex);
        // lowest six bits are ignored
        pfp.setValue(getGPRValue(FPIndex, TreatAsOrdinal{}) & ~0b1'111, TreatAsOrdinal{});
        pfp.pfp.rt = tempRRR;
        setGPR(FPIndex, temp, TreatAsOrdinal{});
        setStackPointer(temp + 64, TreatAsOrdinal{});
    }
}

void
Core::ret() noexcept {
    syncf();
    auto& pfp = getGPR(PFPIndex);
    switch (pfp.pfp.rt) {
        case 0b000:
            restoreStandardFrame();
            break;
        case 0b001:
        {
            auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
            auto x = load(fp - 16, TreatAsOrdinal{});
            auto y = load(fp - 12, TreatAsOrdinal{});
            restoreStandardFrame();
            ac_.setValue(y, TreatAsOrdinal{});
            if (pc_.inSupervisorMode()) {
                pc_.setValue(x, TreatAsOrdinal{});
            }
            break;
        }
        case 0b010:
            if (pc_.inSupervisorMode()) {
                pc_.processControls.traceEnable = 0;
                pc_.processControls.executionMode = 0;
            }
            restoreStandardFrame();
            break;
        case 0b011:
            if (pc_.inSupervisorMode()) {
                pc_.processControls.traceEnable = 1;
                pc_.processControls.executionMode = 0;
            }
            restoreStandardFrame();
            break;
        case 0b111:
        {
            // interrupt return
            auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
            auto x = load(fp - 16, TreatAsOrdinal{});
            auto y = load(fp - 12, TreatAsOrdinal{});
            restoreStandardFrame();
            ac_.setValue(y, TreatAsOrdinal{});
            if (pc_.inSupervisorMode()) {
                pc_.setValue(x, TreatAsOrdinal{});
                checkForPendingInterrupts();
            }
            break;
        }

            break;
        default:
            // undefined!
            unimplementedFault();
            break;
    }
}
