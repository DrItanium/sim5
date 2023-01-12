// sim5
// Copyright (c) 2022, Joshua Scoggins
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

#include <Arduino.h>
#include "Types.h"
#include "Core.h"
Ordinal faultPortValue;
volatile Ordinal systemAddressTableBase = 0;
GPRBlock gpr;
RegisterBlock32 sfrs;
Register ip;
Register ac; 
Register pc;
Register tc; 
Register flags; 
Register flags2; 
Register faultCode; 
Register instruction;
Register ictl;
byte advanceBy;

template<typename T>
T load(Address address, TreatAs<T>) noexcept {
    SplitWord32 split(address);
    set328BusAddress(split);
    return memory<T>(static_cast<size_t>(split.splitAddress.lower) + 0x8000);
}
template<typename T>
void store(Address address, T value, TreatAs<T>) noexcept {
    SplitWord32 split(address);
    set328BusAddress(split);
    memory<T>(static_cast<size_t>(split.splitAddress.lower) + 0x8000) = value;
}
Register& Core::getGPR(byte index) noexcept {
    return gpr.get(index);
}
Register& Core::getGPR(byte index, byte offset) noexcept {
    return getGPR((index + offset) & 0b11111);
}
Ordinal 
Core::unpackSrc1_REG(TreatAsOrdinal) noexcept {
    if (instruction.reg.m1) {
        /// @todo what to do if s1 is also set?
        return instruction.reg.src1;
    } else if (instruction.reg.s1) {
        return getSFR(instruction.reg.src1).o;
    } else {
        return getGPRValue(instruction.reg.src1, TreatAsOrdinal{});
    }
}
Ordinal 
Core::unpackSrc1_REG(byte offset, TreatAsOrdinal) noexcept {
    if (instruction.reg.m1) {
        // literals should always return zero if offset is greater than zero
        return offset == 0 ? instruction.reg.src1 : 0;
    } else if (instruction.reg.s1) {
        return getSFR(instruction.reg.src1, offset).o;
    } else {
        return getGPRValue(instruction.reg.src1, offset, TreatAsOrdinal{});
    }
}
Integer 
Core::unpackSrc1_REG(TreatAsInteger) noexcept {
    if (instruction.reg.m1) {
        return instruction.reg.src1;
    } else if (instruction.reg.s1) {
        return getSFR(instruction.reg.src1).i;
    } else {
        return getGPRValue(instruction.reg.src1, TreatAsInteger{});
    }
}
Ordinal 
Core::unpackSrc2_REG(TreatAsOrdinal) noexcept {
    if (instruction.reg.m2) {
        return instruction.reg.src2;
    } else if (instruction.reg.s2) {
        return getSFR(instruction.reg.src2).o;
    } else {
        return getGPRValue(instruction.reg.src2, TreatAsOrdinal{});
    }
}
Integer 
Core::unpackSrc2_REG(TreatAsInteger) noexcept {
    if (instruction.reg.m2) {
        return instruction.reg.src2;
    } else if (instruction.reg.s2) {
        return getSFR(instruction.reg.src2).i;
    } else {
        return getGPRValue(instruction.reg.src2, TreatAsInteger{});
    }
}

void
Core::scanbyte(Ordinal src2, Ordinal src1) noexcept {
    if (Register s2(src2), s1(src1); 
            s1.bytes[0] == s2.bytes[0] ||
            s1.bytes[1] == s2.bytes[1] ||
            s1.bytes[2] == s2.bytes[2] ||
            s1.bytes[3] == s2.bytes[3]) {
        ac.arith.conditionCode = 0b010;
    } else {
        ac.arith.conditionCode = 0;
    }
}
void
Core::arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) noexcept {
    // set the carry bit
    ac.arith.conditionCode = 0;
    // set the overflow bit
    if ((src2MSB == src1MSB) && (src2MSB != destMSB)) {
        ac.arith.conditionCode |= 0b001;
    } else {
        ac.arith.conditionCode &= 0b110;
    }
    if (result != 0) {
        ac.arith.conditionCode |= 0b010;
    } else {
        ac.arith.conditionCode &= 0b101;
    }
}

void
Core::checkForPendingInterrupts() noexcept {
    /// @todo implement
}


void
Core::emul(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    SplitWord64 result;
    if ((instruction.reg.srcDest & 0b1) != 0) {
        setFaultCode(InvalidOpcodeFault);
    }  else {
        result.whole = static_cast<LongOrdinal>(src2) * static_cast<LongOrdinal>(src1);
    }
    // yes this can be undefined by design :)
    // if we hit a fault then we just give up whats on the stack :)
    dest.setValue(result.parts[0], TreatAsOrdinal{});
    setGPR(instruction.reg.srcDest, 1, result.parts[1], TreatAsOrdinal{});
}

void
Core::ediv(Register& dest, Ordinal src1, Ordinal src2Lower) noexcept {
    SplitWord64 result, src2;
    result.whole = 0;
    src2.parts[0] = src2Lower;
    if ((instruction.reg.srcDest & 0b1) != 0 || (instruction.reg.src2 & 0b1) != 0) {
        /// @todo fix
        setFaultCode(InvalidOpcodeFault);
    } else if (src1 == 0) {
        // divide by zero
        setFaultCode(ZeroDivideFault);
    } else {
        src2.parts[1] = getGPRValue(instruction.reg.src2, 1, TreatAsOrdinal{});
        result.parts[1] = src2.whole / src1; // quotient
        result.parts[0] = static_cast<Ordinal>(src2.whole - (src2.whole / src1) * src1); // remainder
    }
    // yes this can be undefined by design :)
    // if we hit a fault then we just give whats on the stack :)
    // however, to get the compiler to shut up, I am zeroing out the result
    // field
    dest.setValue<Ordinal>(result.parts[0]);
    setGPR(instruction.reg.srcDest, 1, result.parts[1], TreatAsOrdinal{});
}
Ordinal 
Core::getSystemAddressTableBase() noexcept { 
    return systemAddressTableBase; 
}

Ordinal
Core::getSystemProcedureTableBase() noexcept {
    return load(getSystemAddressTableBase() + 120, TreatAsOrdinal{});
}

Ordinal
Core::getSupervisorStackPointer() noexcept {
    return load((getSystemProcedureTableBase() + 12), TreatAsOrdinal{});
}

void
Core::setFaultCode(Ordinal fault) noexcept {
    faultCode.setValue(fault, TreatAsOrdinal{});
}
bool 
Core::faultHappened() noexcept {
    return faultCode.getValue(TreatAsOrdinal{}) != NoFault;
}
Ordinal getSystemProcedureTableBase() noexcept;
Ordinal 
Core::unpackSrc1_COBR(TreatAsOrdinal) noexcept {
    if (instruction.cobr.m1) {
        // treat src1 as a literal
        return instruction.cobr.src1;
    } else {
        return getGPRValue(instruction.cobr.src1, TreatAsOrdinal{});
    }
}
Ordinal
Core::unpackSrc2_COBR(TreatAsOrdinal) noexcept {
    if (instruction.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction.cobr.src2).o;
    } else {
        return getGPRValue(instruction.cobr.src2, TreatAsOrdinal{});
    }
}
Integer
Core::unpackSrc1_COBR(TreatAsInteger) noexcept {
    if (instruction.cobr.m1) {
        // treat src1 as a literal
        return instruction.cobr.src1;
    } else {
        return getGPRValue(instruction.cobr.src1, TreatAsInteger{});
    }
}
Integer
Core::unpackSrc2_COBR(TreatAsInteger) noexcept {
    if (instruction.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction.cobr.src2).i;
    } else {
        return getGPRValue(instruction.cobr.src2, TreatAsInteger{});
    }
}
Register& Core::getSFR(byte index) noexcept {
    return sfrs.get(index);
}
Register& Core::getSFR(byte index, byte offset) noexcept {
    return getSFR((index + offset) & 0b11111);
}
void
Core::syncf() noexcept {
    // Wait for all faults to be generated that are associated with any prior
    // uncompleted instructions
    /// @todo implement if it makes sense since we don't have a pipeline
}
void
Core::flushreg() noexcept {
    /// @todo implement if it makes sense since we aren't using register frames
}
void
mark() noexcept {
    if (pc.pc.traceEnable && tc.trace.breakpointTraceMode) {
        setFaultCode(MarkTraceFault);
    }
}
void
fmark() noexcept {
    if (pc.pc.traceEnable) {
        setFaultCode(MarkTraceFault);
    }
}
void restoreRegisterSet(Ordinal fp) noexcept;
void
restoreStandardFrame() noexcept {
    // need to leave the current call
    moveGPR(FPIndex, PFPIndex, TreatAsOrdinal{});
    // remember that the lowest 6 bits are ignored so it is important to mask
    // them out of the frame pointer address when using the address
    auto realAddress = getGPRValue(FPIndex, TreatAsOrdinal{}) & NotC;
    restoreRegisterSet(realAddress);
    ip.setValue(getGPRValue(RIPIndex, TreatAsOrdinal{}), TreatAsOrdinal{});
    flags.ucode.dontAdvanceIP = 1;
}
void
ret() {
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
                ac.setValue(y, TreatAsOrdinal{});
                if (pc.inSupervisorMode()) {
                    pc.setValue(x, TreatAsOrdinal{});
                }
                break;
            }
        case 0b010: 
            if (pc.inSupervisorMode()) {
                pc.pc.traceEnable = 0;
                pc.pc.executionMode = 0;
            }
            restoreStandardFrame();
            break;
        case 0b011: 
            if (pc.inSupervisorMode()) {
                pc.pc.traceEnable = 1;
                pc.pc.executionMode = 0;
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
                ac.setValue(y, TreatAsOrdinal{});
                if (pc.inSupervisorMode()) {
                    pc.setValue(x, TreatAsOrdinal{});
                    checkForPendingInterrupts();
                }
                break;
            }

            break;
        default: 
            // undefined!
            /// @todo raise a fault?
            break;
    }
}

void
Core::bbc() {
    branchIfBitGeneric<true>();
}
void
Core::bbs() {
    branchIfBitGeneric<false>();
}
Ordinal
Core::computeAddress() noexcept {
    if (instruction.isMEMA()) {
        Ordinal result = instruction.mem.offset;
        if (instruction.mema.action) {
            result += getGPRValue(instruction.mem.abase, TreatAsOrdinal{});
        }
        return result;
    } else {
        // okay so we need to figure out the minor mode after figuring out if
        // it is a double wide operation or not
        if (instruction.memb.group) {
            // okay so it is going to be the displacement versions
            // load 32-bits into the optionalDisplacement field
            advanceBy += 4;
            Integer result = static_cast<Integer>(load(ip.a + 4, TreatAsOrdinal{})); // load the optional displacement
            if (instruction.memb_grp2.useIndex) {
                result += (getGPRValue(instruction.memb_grp2.index, TreatAsInteger{}) << static_cast<Integer>(instruction.memb_grp2.scale));
            }
            if (instruction.memb_grp2.registerIndirect) {
                result += getGPRValue(instruction.memb_grp2.abase, TreatAsInteger{});
            }
            return static_cast<Ordinal>(result);
        } else {
            // okay so the other group isn't as cleanly designed
            switch (instruction.memb.modeMinor) {
                case 0b00: // Register Indirect
                    return getGPRValue(instruction.memb.abase, TreatAsOrdinal{});
                case 0b01: // IP With Displacement 
                    advanceBy += 4;
                    return static_cast<Ordinal>(ip.i + load(ip.a + 4, TreatAsInteger{}) + 8);
                case 0b11: // Register Indirect With Index
                    return getGPRValue(instruction.memb.abase, TreatAsOrdinal{}) + (getGPRValue(instruction.memb.index, TreatAsOrdinal{}) << instruction.memb.scale);
                default:
                    return -1;
            }
        }
    }
}
void
storeBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept {
    for (byte i = 0; i < count; ++i, baseAddress += 4) {
        store(baseAddress, getGPRValue(baseRegister, i, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
void
loadBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept {
    for (byte i = 0; i < count; ++i, baseAddress += 4) {
        setGPR(baseRegister, i, load(baseAddress, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
void 
ldl() noexcept {
    if ((instruction.mem.srcDest & 0b1) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 2);
        // support unaligned accesses
    }
}

void
stl() noexcept {
    if ((instruction.mem.srcDest & 0b1) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 2);
        // support unaligned accesses
    }
}
void
ldt() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 3);
        // support unaligned accesses
    }
}

void
stt() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 3);
        // support unaligned accesses
    }
}

void
ldq() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        loadBlock(computeAddress(), instruction.mem.srcDest, 4);
        // support unaligned accesses
    }
}

void
stq() noexcept {
    if ((instruction.mem.srcDest & 0b11) != 0) {
        setFaultCode(InvalidOperandFault);
    } else {
        storeBlock(computeAddress(), instruction.mem.srcDest, 4);
        // support unaligned accesses
    }
}

void
balx() noexcept {
    auto address = computeAddress();
    setGPR(instruction.mem.srcDest, ip.getValue(TreatAsOrdinal{}) + advanceBy, TreatAsOrdinal{});
    ip.setValue(address, TreatAsOrdinal{});
    flags.ucode.dontAdvanceIP = 1;
}
bool 
registerSetAvailable() noexcept {
    /// @todo implement this properly when we implement support for register
    /// sets
    return false;
}
void
allocateNewRegisterFrame() noexcept {
    // do nothing right now
}
void 
saveRegisterSet(Ordinal fp) noexcept {
    // save the "next" register frame to main memory to reclaim it
    storeBlock(fp, 16, 16);
}
void
restoreRegisterSet(Ordinal fp) noexcept {
    loadBlock(fp, 16, 16);
}
void 
enterCall(Ordinal fp) noexcept {
    if (registerSetAvailable()) {
        allocateNewRegisterFrame();
    } else {
        saveRegisterSet(fp);
        allocateNewRegisterFrame();
    }
}
void
callx() noexcept {
    // wait for any uncompleted instructions to finish
    auto temp = (getGPRValue(SPIndex, TreatAsOrdinal{}) + C) & NotC; // round stack pointer to next boundary
    auto addr = computeAddress();
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    setGPR(RIPIndex, ip.getValue(TreatAsOrdinal{}) + advanceBy, TreatAsOrdinal{});
    enterCall(fp);
    ip.setValue(addr, TreatAsOrdinal{});
    setGPR(PFPIndex, fp, TreatAsOrdinal{});
    setGPR(FPIndex, temp, TreatAsOrdinal{});
    setGPR(SPIndex , temp + 64, TreatAsOrdinal{});
    flags.ucode.dontAdvanceIP = 1;
}
void 
call() {
    // wait for any uncompleted instructions to finish
    auto temp = (getGPRValue(SPIndex, TreatAsOrdinal{}) + C) & NotC; // round stack pointer to next boundary
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    setGPR(RIPIndex, ip.getValue(TreatAsOrdinal{}) + advanceBy, TreatAsOrdinal{});
    enterCall(fp);
    ip.i += instruction.ctrl.displacement;
    setGPR(PFPIndex, fp, TreatAsOrdinal{});
    setGPR(FPIndex, temp, TreatAsOrdinal{});
    setGPR(SPIndex , temp + 64, TreatAsOrdinal{});
    flags.ucode.dontAdvanceIP = 1;
}
Ordinal getSupervisorStackPointer() noexcept;
void
calls(Ordinal src1) noexcept {
    if (auto targ = src1; targ > 259) {
        setFaultCode(ProtectionLengthFault);
    } else {
        syncf();
        auto tempPE = load(getSystemProcedureTableBase() + 48 + (4 * targ), TreatAsOrdinal{});
        auto type = tempPE & 0b11;
        auto procedureAddress = tempPE & ~0b11;
        // read entry from system-procedure table, where spbase is address of
        // system-procedure table from Initial Memory Image
        setGPR(RIPIndex, ip.getValue(TreatAsOrdinal{}) + advanceBy, TreatAsOrdinal{});
        ip.setValue(procedureAddress, TreatAsOrdinal{});
        Ordinal temp = 0, tempRRR = 0;
        if ((type == 0b00) || pc.inSupervisorMode()) {
            temp = (getGPRValue(SPIndex, TreatAsOrdinal{}) + C) & NotC;
            tempRRR = 0;
        } else {
            temp = getSupervisorStackPointer();
            tempRRR = 0b010 | (pc.pc.traceEnable ? 0b001 : 0);
            pc.pc.executionMode = 1;
            pc.pc.traceEnable = temp & 0b1;
        }
        enterCall(temp);
        /// @todo expand pfp and fp to accurately model how this works
        auto& pfp = getGPR(PFPIndex);
        // lowest six bits are ignored
        pfp.setValue(getGPRValue(FPIndex, TreatAsOrdinal{}) & ~0b1'111, TreatAsOrdinal{});
        pfp.pfp.rt = tempRRR;
        setGPR(FPIndex, temp, TreatAsOrdinal{});
        setGPR(SPIndex, temp + 64, TreatAsOrdinal{});
        flags.ucode.dontAdvanceIP = 1;
    }
}
void
Core::bx() noexcept {
    ip_.setValue(computeAddress(), TreatAsOrdinal{});
    flags_.ucode.dontAdvanceIP = 1;
}
void
Core::performRegisterTransfer(byte mask, byte count) noexcept {
    if (((instruction_.reg.srcDest & mask) != 0) || ((instruction_.reg.src1 & mask) != 0)) {
        setFaultCode(InvalidOpcodeFault);
    }
    for (byte i = 0; i < count; ++i) {
        setGPR(instruction_.reg.srcDest, i, unpackSrc1_REG(i, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}

void 
Core::setFaultPort(Ordinal value) noexcept {
    faultPortValue = value;
}
 
Ordinal getFaultPort() noexcept { 
    return faultPortValue;
}

void
Core::lockBus() noexcept {
    while (digitalRead(LOCKPIN) == LOW);
    pinMode(LOCKPIN, OUTPUT);
}
void
Core::unlockBus() noexcept {
    pinMode(LOCKPIN, INPUT);
}
void
Core::signalBootFailure() noexcept {
    digitalWrite(FAILPIN, HIGH);
}
void
Core::branch() noexcept {
    ip.i += instruction.ctrl.displacement;
    flags.ucode.dontAdvanceIP = 1;
}
void
Core::branchConditional(bool condition) noexcept {
    if (condition) {
        branch();
    }
}
void
Core::setFaultCode(Ordinal value, bool cond) noexcept {
    if (cond) {
        setFaultCode(value);
    }
}

/**
 * @brief And the condition code with the consistent mask found in the
 * instruction encoding; returns true if the value returned is not zero
 */
bool
Core::getMaskedConditionCode() noexcept {
    return (ac.getConditionCode() & instruction.instGeneric.mask) != 0;
}

bool
Core::conditionCodeEqualsMask() noexcept {
    return ac.getConditionCode() == instruction.instGeneric.mask;
}
bool
Core::fullConditionCodeCheck() noexcept {
    // the second condition handles the case where we are looking at unordered
    // output where it is only true if it is equal to zero. So if it turns out
    // that the condition code is zero and the mask is the unordered kind then
    // return true :). In all other cases, the second check will either fail
    // (because the condition code is zero) or it will never fire because the
    // masked condition code will be non zero.
    return getMaskedConditionCode() || conditionCodeEqualsMask();
}
void performSelect(Register& dest, Ordinal src1, Ordinal src2, bool condition) noexcept;
void performConditionalSubtract(Register& dest, Integer src1, Integer src2, bool condition, TreatAsInteger) noexcept;
void performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, bool condition, TreatAsOrdinal) noexcept;
void performConditionalAdd(Register& dest, Integer src1, Integer src2, bool condition, TreatAsInteger) noexcept;
void performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, bool condition, TreatAsOrdinal) noexcept;
bool
Core::cycle() noexcept {
    setFaultCode(NoFault);
    instruction.setValue(load(ip.a, TreatAsOrdinal{}), TreatAsOrdinal{});
    auto& regDest = getGPR(instruction.reg.srcDest);
    auto src2o = unpackSrc2_REG(TreatAsOrdinal{});
    auto src2i = unpackSrc2_REG(TreatAsInteger{});
    auto src1o = unpackSrc1_REG(TreatAsOrdinal{});
    auto src1i = unpackSrc1_REG(TreatAsInteger{});
    flags.clear();
    flags2.clear();
    advanceBy = 4;
    
    switch (instruction.getOpcode()) {
        case Opcodes::bal: // bal
            setGPR(LRIndex, ip.getValue<Ordinal>() + 4, TreatAsOrdinal{});
            // then fallthrough and take the branch
        case Opcodes::b: // b
            branch();
            break;
        case Opcodes::call: // call
            call();
            break;
        case Opcodes::ret: // ret
            ret();
            break;
        case Opcodes::bno:
        case Opcodes::be:
        case Opcodes::bne:
        case Opcodes::bl:
        case Opcodes::ble:
        case Opcodes::bg:
        case Opcodes::bge:
        case Opcodes::bo:
            // the branch instructions have the mask encoded into the opcode
            // itself so we can just use it and save a ton of space overall
            branchConditional(fullConditionCodeCheck());
            break;
        case Opcodes::faultno:
        case Opcodes::faulte:
        case Opcodes::faultne:
        case Opcodes::faultl:
        case Opcodes::faultle:
        case Opcodes::faultg:
        case Opcodes::faultge:
        case Opcodes::faulto:
            setFaultCode(ConstraintRangeFault, fullConditionCodeCheck());
            break;
        case Opcodes::testno:
        case Opcodes::testg:
        case Opcodes::teste:
        case Opcodes::testge:
        case Opcodes::testl:
        case Opcodes::testne:
        case Opcodes::testle:
        case Opcodes::testo:
            setGPR(instruction.cobr.src1, (fullConditionCodeCheck()) ? 1 : 0, TreatAsOrdinal{});
            break;
        case Opcodes::lda:
            setGPR(instruction.mem.srcDest, computeAddress(), TreatAsOrdinal{});
            break;
        case Opcodes::bbc:
            bbc();
            break;
        case Opcodes::bbs:
            bbs();
            break;
        case Opcodes::cmpobg:
        case Opcodes::cmpobe:
        case Opcodes::cmpobge:
        case Opcodes::cmpobl:
        case Opcodes::cmpobne:
        case Opcodes::cmpoble:
            cmpobGeneric();
            break;
        case Opcodes::cmpibno: // never branches
        case Opcodes::cmpibg:
        case Opcodes::cmpibe:
        case Opcodes::cmpibge:
        case Opcodes::cmpibl:
        case Opcodes::cmpibne:
        case Opcodes::cmpible:
        case Opcodes::cmpibo: // always branches
            cmpibGeneric();
            break;
        case Opcodes::ld: 
            //flags2.ucode2.count = 1;
            loadBlock(computeAddress(), instruction.mem.srcDest, 1);
            break;
        case Opcodes::st: 
            //flags2.ucode2.count = 1;
            storeBlock(computeAddress(), instruction.mem.srcDest, 1);
            break;
        case Opcodes::ldob:
            setGPR(instruction.mem.srcDest, load(computeAddress(), TreatAs<ByteOrdinal>{}), TreatAsOrdinal{});
            break;
        case Opcodes::stob:
            store<ByteOrdinal>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAs<Ordinal>{}), TreatAs<ByteOrdinal>{});
            break;
        case Opcodes::ldos:
            setGPR(instruction.mem.srcDest, load(computeAddress(), TreatAs<ShortOrdinal>{}), TreatAsOrdinal{});
            break;
        case Opcodes::stos:
            store<ShortOrdinal>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAsOrdinal{}), TreatAs<ShortOrdinal>{});
            break;
        case Opcodes::ldib:
            setGPR(instruction.mem.srcDest, load(computeAddress(), TreatAs<ByteInteger>{}), TreatAsInteger{});
            break;
        case Opcodes::stib:
            /// @todo fully implement fault detection
            store<ByteInteger>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAsInteger{}), TreatAs<ByteInteger>{});
            break;
        case Opcodes::ldis:
            setGPR(instruction.mem.srcDest, load(computeAddress(), TreatAs<ShortInteger>{}), TreatAsInteger{});
            break;
        case Opcodes::stis:
            /// @todo fully implement fault detection
            store<ShortInteger>(computeAddress(), getGPRValue(instruction.mem.srcDest, TreatAsInteger{}), TreatAs<ShortInteger>{});
            break;
        case Opcodes::ldl:
            ldl();
            break;
        case Opcodes::stl:
            stl();
            break;
        case Opcodes::ldt:
            ldt();
            break;
        case Opcodes::stt:
            stt();
            break;
        case Opcodes::ldq:
            ldq();
            break;
        case Opcodes::stq:
            stq();
            break;
        case Opcodes::bx:
            bx();
            break;
        case Opcodes::balx:
            balx();
            break;
        case Opcodes::callx:
            callx();
            break;
    // in some of the opcodeExt values seem to reflect the resultant truth
    // table for the operation :). That's pretty cool
        case Opcodes::nand: // nand
            flags.ucode.invertResult = 1;
        case Opcodes::andOperation: // and
            flags.ucode.performLogical = 1;
            flags.ucode.doAnd = 1;
            break;
        case Opcodes::clrbit: // clrbit
                              // clrbit is src2 & ~computeBitPosition(src1)
                              // so lets use andnot
            flags.ucode.src1IsBitPosition = 1;
        case Opcodes::andnot: // andnot
            flags.ucode.doAnd = 1;
            flags.ucode.invertSrc1 = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::notand: // notand
            flags.ucode.doAnd = 1;
            flags.ucode.invertSrc2 = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::notbit: // notbit
                     // notbit is src2 ^ computeBitPosition(src1)
            flags.ucode.src1IsBitPosition = 1;
        case Opcodes::xorOperation: // xor
            flags.ucode.doXor = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::setbit: // setbit
                     // setbit is src2 | computeBitPosition(src1o)
            flags.ucode.src1IsBitPosition = 1;
        case Opcodes::orOperation: // or
            flags.ucode.doOr = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::nor: // nor
            flags.ucode.doOr = 1;
            flags.ucode.invertResult = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::xnor: // xnor
            flags.ucode.doXor = 1;
            flags.ucode.invertResult = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::notOperation: // not 
                     // perform fallthrough to ornot with src2 set to zero
            flags.ucode.zeroSrc2 = 1;
        case Opcodes::ornot: // ornot
            flags.ucode.doOr = 1;
            flags.ucode.invertSrc1 = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::notor: // notor
            flags.ucode.doOr = 1;
            flags.ucode.invertSrc2 = 1;
            flags.ucode.performLogical = 1;
            break;
        case Opcodes::alterbit: // alterbit
            flags.ucode.src1IsBitPosition = 1;
            flags.ucode.performLogical = 1;
            if (ac.getConditionCode() & 0b010) {
                flags.ucode.doOr = 1;
            } else {
                flags.ucode.doAnd = 1;
                flags.ucode.invertSrc1 = 1;
            }
            break;
        case Opcodes::addo: // addo
            flags.ucode.performAdd = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::addi: // addi
            flags.ucode.performAdd = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::subo: // subo
            // I remember this trick from college, subtraction is just addition
            // with a negative second argument :). I never gave it much thought
            // until now but it seems to be an effective trick to save space.
            flags.ucode.performSubtract = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::subi: // subi
            flags.ucode.performSubtract = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::shro: // shro
            regDest.setValue<Ordinal>(src1o < 32 ? src2o >> src1o : 0);
            break;
        case Opcodes::shrdi: // shrdi
                  // according to the manual, equivalent to divi value, 2 so that is what we're going to do for correctness sake
            regDest.setValue<Integer>( src1i < 32 && src1i >= 0 ? src2i / computeBitPosition(src1i) : 0);
            break;
        case Opcodes::shri: // shri
            /*
             * if (src >= 0) {
             *  if (len < 32) {
             *      dest <- src/2^len
             *  } else {
             *      dest <- 0
             *  }
             * }else {
             *  if (len < 32) {
             *      dest <- (src - 2^len + 1)/2^len;
             *  } else {
             *      dest <- -1;
             *   }
             * }
             *
             */
            /// @todo perhaps implement the extra logic if necessary
            regDest.setValue<Integer>(src2i >> src1i);
            break;
        case Opcodes::shlo: // shlo
            regDest.setValue<Ordinal>(src1o < 32 ? src2o << src1o : 0);
            break;
        case Opcodes::rotate: // rotate
            regDest.setValue<Ordinal>(rotateOperation(src2o, src1o));
            break;
        case Opcodes::shli: // shli
            regDest.setValue<Integer>(src2i << src1i);
            break;
        case Opcodes::cmpo: // cmpo
            flags.ucode.performCompare = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::cmpi: // cmpi
            flags.ucode.performCompare = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::concmpo: // concmpo
            flags.ucode.performConditionalCompare = 1;
            flags.ucode.ordinalOp = 1;
            flags.ucode.performCompare = 1;
            break;
        case Opcodes::concmpi: // concmpi
            flags.ucode.performConditionalCompare = 1;
            flags.ucode.integerOp = 1;
            flags.ucode.performCompare = 1;
            break;
        case Opcodes::cmpinco: // cmpinco
            flags.ucode.performCompare = 1;
            flags.ucode.ordinalOp = 1;
            flags.ucode.performIncrement = 1;
            break;
        case Opcodes::cmpinci: // cmpinci
            flags.ucode.performCompare = 1;
            flags.ucode.integerOp = 1;
            flags.ucode.performIncrement = 1;
            break;
        case Opcodes::cmpdeco: // cmpdeco
            flags.ucode.performCompare = 1;
            flags.ucode.ordinalOp = 1;
            flags.ucode.performDecrement = 1;
            break;
        case Opcodes::cmpdeci: // cmpdeci
            flags.ucode.performCompare = 1;
            flags.ucode.integerOp = 1;
            flags.ucode.performDecrement = 1;
            break;
        case Opcodes::scanbyte: // scanbyte
            scanbyte(src2o, src1o);
            break;
        case Opcodes::chkbit: // chkbit
            ac.arith.conditionCode = ((src2o & computeBitPosition(src1o)) == 0 ? 0b000 : 0b010);
            break;
        case Opcodes::addc: 
            flags.ucode.performAdd = 1;
            flags.ucode.performCarry = 1;
            break;
        case Opcodes::subc:
            flags.ucode.performSubtract = 1;
            flags.ucode.performCarry = 1;
            break;
        case Opcodes::mov:
            regDest.setValue<Ordinal>(src1o);
            break;
        case Opcodes::movl:
            flags.ucode.performRegisterTransfer = 1;
            flags2.ucode2.mask = 0b1;
            flags2.ucode2.count = 2; 
            //performRegisterTransfer(0b1, 2);
            break;
        case Opcodes::movt:
            flags.ucode.performRegisterTransfer = 1;
            flags2.ucode2.mask = 0b11;
            flags2.ucode2.count = 3; 
            //performRegisterTransfer(0b11, 3);
            break;
        case Opcodes::movq:
            flags.ucode.performRegisterTransfer = 1;
            flags2.ucode2.mask = 0b11;
            flags2.ucode2.count = 3; 
            //performRegisterTransfer(0b11, 4);
            break;
        case Opcodes::syncf:
            flags.ucode.performSyncf = 1;
            break;
        case Opcodes::flushreg:
            flushreg();
            break;
        case Opcodes::fmark:
            fmark();
            break;
        case Opcodes::mark:
            mark();
            break;
        case Opcodes::mulo:
            flags.ucode.performMultiply = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::muli:
            flags.ucode.performMultiply = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::divi:
            flags.ucode.performDivide = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::divo:
            flags.ucode.performDivide = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::remo:
            flags.ucode.performRemainder = 1;
            flags.ucode.ordinalOp = 1;
            break;
        case Opcodes::remi:
            flags.ucode.performRemainder = 1;
            flags.ucode.integerOp = 1;
            break;
        case Opcodes::modi: 
            if (auto denominator = src1i; denominator == 0) {
                setFaultCode(ZeroDivideFault);
            } else {
                auto numerator = src2i;
                auto result = numerator - ((numerator / denominator) * denominator);
                if (((numerator * denominator) < 0) && (result != 0)) {
                    result += denominator;
                }
                regDest.setValue<Integer>(result);
            }
            break;
        case Opcodes::modify:
            regDest.setValue<Ordinal>(modify(src1o, src2o, regDest.getValue<Ordinal>()));
            break;
        case Opcodes::extract:
            // taken from the Hx manual as it isn't insane
            regDest.setValue<Ordinal>((regDest.o >> (src1o > 32 ? 32 : src1o)) & ~(0xFFFF'FFFF << src2o));
            break;
        case Opcodes::modac: 
            regDest.setValue<Ordinal>(ac.modify(src1o, src2o));
            break;
        case Opcodes::modtc: 
            regDest.setValue<Ordinal>(tc.modify(src1o, src2o));
            break;
        case Opcodes::modpc:
            if (auto mask = src1o; mask != 0) {
                if (!pc.inSupervisorMode()) {
                    setFaultCode(TypeMismatchFault);
                } else {
                    regDest.setValue<Ordinal>(pc.modify(mask, src2o));
                    if (regDest.getPriority() > pc.getPriority()) {
                        checkForPendingInterrupts();
                    }
                }
            } else {
                regDest.setValue<Ordinal>(pc.getValue<Ordinal>());
            }
            break;
        case Opcodes::atadd:
            flags.ucode.performSyncf = 1;
            flags.ucode.performAtomicOperation = 1;
            flags.ucode.performAdd = 1;
            break;
        case Opcodes::atmod:
            flags.ucode.performSyncf = 1;
            flags.ucode.performAtomicOperation = 1;
            flags.ucode.performModify = 1;
            break;
        case Opcodes::emul:
            emul(regDest, src2o, src1o);
            break;
        case Opcodes::ediv:
            ediv(regDest, src2o, src1o);
            break;
        case Opcodes::calls:
            calls(src1o);
            break;
        case Opcodes::spanbit:
            spanbit(regDest, src2o, src1o);
            break;
        case Opcodes::scanbit:
            scanbit(regDest, src2o, src1o);
            break;
        case Opcodes::synld:
            synld(regDest, src1o);
            break;
        case Opcodes::synmov:
            synmov(getGPR(instruction.reg.src1), src2o);
            break;
        case Opcodes::synmovl:
            synmovl(getGPR(instruction.reg.src1), src2o);
            break;
        case Opcodes::synmovq:
            synmovq(getGPR(instruction.reg.src1), src2o);
            break;
        case Opcodes::sysctl:
            sysctl(regDest, src1o, src2o);
            break;
        case Opcodes::selno:
        case Opcodes::sele:
        case Opcodes::selg:
        case Opcodes::selge:
        case Opcodes::sell:
        case Opcodes::selne:
        case Opcodes::selle:
        case Opcodes::selo:
            performSelect(regDest, src1o, src2o, fullConditionCodeCheck());
            break;
        case Opcodes::addono:
        case Opcodes::addoe:
        case Opcodes::addog:
        case Opcodes::addoge:
        case Opcodes::addol:
        case Opcodes::addone:
        case Opcodes::addole:
        case Opcodes::addoo:
            performConditionalAdd(regDest, src1o, src2o, fullConditionCodeCheck(), TreatAsOrdinal{});
            break;

        case Opcodes::addino:
        case Opcodes::addie:
        case Opcodes::addig:
        case Opcodes::addige:
        case Opcodes::addil:
        case Opcodes::addine:
        case Opcodes::addile:
        case Opcodes::addio:
            performConditionalAdd(regDest, src1i, src2i, fullConditionCodeCheck(), TreatAsInteger{});
            break;
        case Opcodes::subono:
        case Opcodes::suboe:
        case Opcodes::subog:
        case Opcodes::suboge:
        case Opcodes::subol:
        case Opcodes::subone:
        case Opcodes::subole:
        case Opcodes::suboo:
            performConditionalSubtract(regDest, src1o, src2o, fullConditionCodeCheck(), TreatAsOrdinal{});
            break;

        case Opcodes::subino:
        case Opcodes::subie:
        case Opcodes::subig:
        case Opcodes::subige:
        case Opcodes::subil:
        case Opcodes::subine:
        case Opcodes::subile:
        case Opcodes::subio:
            performConditionalSubtract(regDest, src1i, src2i, fullConditionCodeCheck(), TreatAsInteger{});
            break;
        default:
            setFaultCode(UnimplementedFault);
            break;
    }
    if (flags.ucode.performSyncf) {
        syncf();
    }
    if (flags.ucode.performAtomicOperation) {
        lockBus();
        auto addr = src1o & 0xFFFF'FFFC;
        auto temp = load(addr, TreatAsOrdinal{});
        Ordinal result = 0;
        if (flags.ucode.performAdd) {
            // adds the src (src2 internally) value to the value in memory location specified with the addr (src1 in this case) operand.
            // The initial value from memory is stored in dst (internally src/dst).
            result = temp + src2o;
        } else if (flags.ucode.performModify) {
            // copies the src/dest value (logical version) into the memory location specifeid by src1.
            // The bits set in the mask (src2) operand select the bits to be modified in memory. The initial
            // value from memory is stored in src/dest
            result = modify(src2o, regDest.o, temp);
        } else {
            // if we got here then it means we don't have something configured
            // correctly
                setFaultCode(InvalidOpcodeFault);
        }
        if (!faultHappened()) {
            store(addr, result, TreatAsOrdinal{});
            regDest.o = temp;
        }
        unlockBus();
    }
    if (flags.ucode.performRegisterTransfer) {
        performRegisterTransfer(flags2.ucode2.mask, flags2.ucode2.count);
    }
    if (flags.ucode.performLogical) {
        if (flags.ucode.src1IsBitPosition) {
            src1o = computeBitPosition(src1o);
        }
        if (flags.ucode.invertSrc1) {
            src1o = ~src1o;
        }
        if (flags.ucode.zeroSrc2) {
            src2o = 0;
        }
        if (flags.ucode.invertSrc2) {
            src2o = ~src2o;
        }
        if (flags.ucode.doAnd) {
            regDest.setValue(src2o & src1o, TreatAsOrdinal{});
        } else if (flags.ucode.doXor) {
            regDest.setValue(src2o ^ src1o, TreatAsOrdinal{});
        } else if (flags.ucode.doOr) {
            regDest.setValue(src2o | src1o, TreatAsOrdinal{});
        }
        if (flags.ucode.invertResult) {
            regDest.invert(TreatAsOrdinal{});
        }
    }
    if (flags.ucode.performCompare) {
        if (flags.ucode.performConditionalCompare) {
            if ((ac.getConditionCode() & 0b100) == 0) {
                bool cond = false;
                if (flags.ucode.ordinalOp) {
                    cond = src1o <= src2o;
                } else if (flags.ucode.integerOp) {
                    cond = src1i <= src2i;
                } else {
                    // if we got here then it means we don't have something configured
                    // correctly
                    setFaultCode(InvalidOpcodeFault);
                }
                if (!faultHappened()) {
                    ac.arith.conditionCode = cond ? 0b010 : 0b001;
                }
            }
        } else {
            if (flags.ucode.ordinalOp) {
                cmpGeneric(src1o, src2o);
                if (flags.ucode.performIncrement) {
                    regDest.o = src2o + 1;
                }
                if (flags.ucode.performDecrement) {
                    regDest.o = src2o - 1;
                }
            } else if (flags.ucode.integerOp) {
                cmpGeneric(src1i, src2i);
                if (flags.ucode.performIncrement) {
                    regDest.i = src2i + 1;
                }
                if (flags.ucode.performDecrement) {
                    regDest.i = src2i - 1;
                }
            } else {
                // if we got here then it means we don't have something configured
                // correctly
                setFaultCode(InvalidOpcodeFault);
            }
        }
    }
    if (flags.ucode.performCarry) {
        LongOrdinal result = 0;
        if (flags.ucode.performAdd) {
            result = static_cast<LongOrdinal>(src2o) + static_cast<LongOrdinal>(src1o);
        } else if (flags.ucode.performSubtract) {
            result = static_cast<LongOrdinal>(src2o) - static_cast<LongOrdinal>(src1o) - 1;
        } else {
            // if we got here then it means we don't have something configured
            // correctly
                setFaultCode(InvalidOpcodeFault);
        }
        if (!faultHappened()) {
            result += (ac.getCarryBit() ? 1 : 0);
            regDest.o = static_cast<Ordinal>(result);
            arithmeticWithCarryGeneric(static_cast<Ordinal>(result >> 32), 
                    (src2o & 0x8000'0000), 
                    (src1o & 0x8000'0000), 
                    (regDest.o & 0x8000'0000));
        }
    } else if (flags.ucode.performAdd) {
        if (flags.ucode.ordinalOp) {
            regDest.o = src2o + src1o;
        } else if (flags.ucode.integerOp) {
            regDest.i = src2i + src1i;
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }
    } else if (flags.ucode.performSubtract) {
        if (flags.ucode.ordinalOp) {
            regDest.o = src2o - src1o;
        } else if (flags.ucode.integerOp) {
            regDest.i = src2i - src1i;
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }

    } else if (flags.ucode.performMultiply) {
        if (flags.ucode.ordinalOp) {
            regDest.o = src2o * src1o;
        } else if (flags.ucode.integerOp) {
            regDest.i = src2i * src1i;
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }
    } else if (flags.ucode.performDivide) {
        if (flags.ucode.ordinalOp) {
            if (src1o == 0) {
                /// @todo fix this
                setFaultCode(ZeroDivideFault);
            } else {
                regDest.o = src2o / src1o;
            }
        } else if (flags.ucode.integerOp) {
            if (src1i == 0) {
                /// @todo fix this
                setFaultCode(ZeroDivideFault);
            } else {
                regDest.i = src2i / src1i;
            }
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }
    } else if (flags.ucode.performRemainder) {
        if (flags.ucode.ordinalOp) {
            if (src1o == 0) {
                /// @todo fix this
                setFaultCode(ZeroDivideFault);
            } else {
                // taken from the i960Sx manual
                //dest.setOrdinal(src2 - ((src2 / src1) * src1));
                regDest.o = src2o % src1o;
            }
        } else if (flags.ucode.integerOp) {
            if (src1i == 0) {
                /// @todo fix this
                setFaultCode(ZeroDivideFault);
            } else {
                // taken from the i960Sx manual
                //dest.setInteger(src2 - ((src2 / src1) * src1));
                regDest.i = src2i % src1i;
            }
        } else {
            // if we got here then it means we don't have something configured
            // correctly
            setFaultCode(InvalidOpcodeFault);
        }
    }
    if (faultHappened()) {
        /// @todo implement this as the fallback operation when something bad
        /// happens
        ///
        /// Faults are basically fallback behavior when something goes wacky!
        setFaultPort(faultCode.o);
    }
    // okay we got here so we need to start grabbing data off of the bus and
    // start executing the next instruction
    if (!flags.ucode.dontAdvanceIP) {
        ip.o += advanceBy; 
    }
    return true;
}

Ordinal 
Register::modify(Ordinal mask, Ordinal src) noexcept {
    auto tmp = o;
    o = ::modify(mask, src, o);
    return tmp;
}

void
Core::begin() noexcept {
    //Serial.print(F("Configuring simulator structures..."));
    // so we need to do any sort of processor setup here
    ip.clear();
    for (int i = 0; i < 32; ++i) {
        getGPR(i).clear();
    }
    //Serial.println(F("DONE"));
}

void 
Core::synld(Register& dest, Ordinal src) noexcept {
    ac.arith.conditionCode = 0b000;
    if (auto tempa = src & 0xFFFF'FFFC; tempa == 0xFF00'0004) {
        // interrupt control register needs to be read through this
        ac.arith.conditionCode = 0b010;
        // copy the contents of the interrupt control register to a target
        // register
        dest.setValue(ictl.o, TreatAsOrdinal{});
    } else {
        ac.arith.conditionCode = 0b010;
        dest.setValue(load(tempa, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
void 
Core::synmov(Register& dest, Ordinal src) noexcept {
    ac.arith.conditionCode = 0b000;
    if (auto tempa = dest.getValue(TreatAsOrdinal{}) & 0xFFFF'FFFC; tempa == 0xFF00'0004) {
        ictl.o = load(src, TreatAsOrdinal{});
        ac.arith.conditionCode = 0b010;
    } else {
        auto temp = load(src, TreatAsOrdinal{});
        store(tempa, temp, TreatAsOrdinal{});
        // wait for completion
        ac.arith.conditionCode = 0b010;
    }
}
void 
Core::synmovl(Register& dest, Ordinal src) noexcept {
    ac.arith.conditionCode = 0b000;
    auto tempa = dest.getValue(TreatAsOrdinal{}) & 0xFFFF'FFF8; 
    auto tempLower = load(src, TreatAsOrdinal{});
    auto tempUpper = load(src + 4, TreatAsOrdinal{});
    store(tempa, tempLower, TreatAsOrdinal{});
    store(tempa + 4, tempUpper, TreatAsOrdinal{});
    // wait for completion
    ac.arith.conditionCode = 0b010;

}
void 
Core::synmovq(Register& dest, Ordinal src) noexcept {

    ac.arith.conditionCode = 0b000;
    auto temp0 = load(src, TreatAsOrdinal{});
    auto temp1 = load(src+4, TreatAsOrdinal{});
    auto temp2 = load(src+8, TreatAsOrdinal{});
    auto temp3 = load(src+12, TreatAsOrdinal{});
    if (auto tempa = dest.getValue(TreatAsOrdinal{}) & 0xFFFF'FFF0; tempa == 0xFF00'0010) {
        iac::send(iac::Message{temp0, temp1, temp2, temp3});
        ac.arith.conditionCode = 0b010;
    } else {
        store(tempa, temp0, TreatAsOrdinal{});
        store(tempa+4, temp1, TreatAsOrdinal{});
        store(tempa+8, temp2, TreatAsOrdinal{});
        store(tempa+12, temp3, TreatAsOrdinal{});
        // wait for completion
        ac.arith.conditionCode = 0b010;
    }
}
void
Core::sysctl(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    ByteOrdinal type = src1 >> 8;
    ByteOrdinal field1 = src1;
    ShortOrdinal field2 = static_cast<ShortOrdinal>(src1 >> 16);
    Ordinal field3 = src2;
    Ordinal field4 = dest.getValue(TreatAsOrdinal{});
    
}

void 
performSelect(Register& dest, Ordinal src1, Ordinal src2, bool condition) noexcept {
    if (condition) {
        dest.setValue(src2, TreatAsOrdinal{});
    } else {
        dest.setValue(src1, TreatAsOrdinal{});
    }
}
void
performConditionalSubtract(Register& dest, Integer src1, Integer src2, bool condition, TreatAsInteger) noexcept {
    /// @todo implement
}

void
performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, bool condition, TreatAsOrdinal) noexcept {
    if (condition) {
        dest.setValue(src2 - src1, TreatAsOrdinal{});
    }
}

void
performConditionalAdd(Register& dest, Integer src1, Integer src2, bool condition, TreatAsInteger) noexcept {
    /// @todo implement
}

void
performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, bool condition, TreatAsOrdinal) noexcept {
    if (condition) {
        dest.setValue(src1 + src2, TreatAsOrdinal{});
    }
}
