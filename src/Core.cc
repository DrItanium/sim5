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

Ordinal 
Core::unpackSrc1(TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        /// @todo what to do if s1 is also set?
        return instruction_.reg.src1;
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1).o;
    } else {
        return getGPRValue(instruction_.reg.src1, TreatAsOrdinal{});
    }
}
Ordinal 
Core::unpackSrc1(byte offset, TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        // literals should always return zero if offset is greater than zero
        return offset == 0 ? instruction_.reg.src1 : 0;
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1, offset).o;
    } else {
        return getGPRValue(instruction_.reg.src1, offset, TreatAsOrdinal{});
    }
}
Integer 
Core::unpackSrc1(TreatAsInteger, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        return instruction_.reg.src1;
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1).i;
    } else {
        return getGPRValue(instruction_.reg.src1, TreatAsInteger{});
    }
}
Ordinal 
Core::unpackSrc2(TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m2) {
        return instruction_.reg.src2;
    } else if (instruction_.reg.s2) {
        return getSFR(instruction_.reg.src2).o;
    } else {
        return getGPRValue(instruction_.reg.src2, TreatAsOrdinal{});
    }
}
Integer 
Core::unpackSrc2(TreatAsInteger, TreatAsREG) noexcept {
    if (instruction_.reg.m2) {
        return instruction_.reg.src2;
    } else if (instruction_.reg.s2) {
        return getSFR(instruction_.reg.src2).i;
    } else {
        return getGPRValue(instruction_.reg.src2, TreatAsInteger{});
    }
}

void
Core::scanbyte(Ordinal src2, Ordinal src1) noexcept {
    if (Register s2(src2), s1(src1); 
            s1.bytes[0] == s2.bytes[0] ||
            s1.bytes[1] == s2.bytes[1] ||
            s1.bytes[2] == s2.bytes[2] ||
            s1.bytes[3] == s2.bytes[3]) {
        ac_.arith.conditionCode = 0b010;
    } else {
        ac_.arith.conditionCode = 0;
    }
}
void
Core::arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) noexcept {
    // set the carry bit
    ac_.arith.conditionCode = 0;
    // set the overflow bit
    if ((src2MSB == src1MSB) && (src2MSB != destMSB)) {
        ac_.arith.conditionCode |= 0b001;
    } else {
        ac_.arith.conditionCode &= 0b110;
    }
    if (result != 0) {
        ac_.arith.conditionCode |= 0b010;
    } else {
        ac_.arith.conditionCode &= 0b101;
    }
}

void
Core::checkForPendingInterrupts() noexcept {
    /// @todo implement
}


void
Core::emul(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    SplitWord64 result;
    if ((instruction_.reg.srcDest & 0b1) != 0) {
        generateFault(InvalidOpcodeFault);
    }  else {
        result.whole = static_cast<LongOrdinal>(src2) * static_cast<LongOrdinal>(src1);
    }
    // yes this can be undefined by design :)
    // if we hit a fault then we just give up whats on the stack :)
    dest.setValue(result.parts[0], TreatAsOrdinal{});
    setGPR(instruction_.reg.srcDest, 1, result.parts[1], TreatAsOrdinal{});
}

void
Core::ediv(Register& dest, Ordinal src1, Ordinal src2Lower) noexcept {
    SplitWord64 result, src2;
    result.whole = 0;
    src2.parts[0] = src2Lower;
    if ((instruction_.reg.srcDest & 0b1) != 0 || (instruction_.reg.src2 & 0b1) != 0) {
        /// @todo fix
        generateFault(InvalidOpcodeFault);
    } else if (src1 == 0) {
        // divide by zero
        generateFault(ZeroDivideFault);
    } else {
        src2.parts[1] = getGPRValue(instruction_.reg.src2, 1, TreatAsOrdinal{});
        result.parts[1] = src2.whole / src1; // quotient
        result.parts[0] = static_cast<Ordinal>(src2.whole - (src2.whole / src1) * src1); // remainder
    }
    // yes this can be undefined by design :)
    // if we hit a fault then we just give whats on the stack :)
    // however, to get the compiler to shut up, I am zeroing out the result
    // field
    dest.setValue<Ordinal>(result.parts[0]);
    setGPR(instruction_.reg.srcDest, 1, result.parts[1], TreatAsOrdinal{});
}

Ordinal
Core::getSystemProcedureTableBase() const noexcept {
    return load(getSystemAddressTableBase() + 120, TreatAsOrdinal{});
}

Ordinal
Core::getSupervisorStackPointer() const noexcept {
    return load((getSystemProcedureTableBase() + 12), TreatAsOrdinal{});
}

Ordinal 
Core::unpackSrc1(TreatAsOrdinal, TreatAsCOBR) noexcept {
    if (instruction_.cobr.m1) {
        // treat src1 as a literal
        return instruction_.cobr.src1;
    } else {
        return getGPRValue(instruction_.cobr.src1, TreatAsOrdinal{});
    }
}
Ordinal
Core::unpackSrc2(TreatAsOrdinal, TreatAsCOBR) noexcept {
    if (instruction_.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction_.cobr.src2).o;
    } else {
        return getGPRValue(instruction_.cobr.src2, TreatAsOrdinal{});
    }
}
Integer
Core::unpackSrc1(TreatAsInteger, TreatAsCOBR) noexcept {
    if (instruction_.cobr.m1) {
        // treat src1 as a literal
        return instruction_.cobr.src1;
    } else {
        return getGPRValue(instruction_.cobr.src1, TreatAsInteger{});
    }
}
Integer
Core::unpackSrc2(TreatAsInteger, TreatAsCOBR) noexcept {
    if (instruction_.cobr.s2) {
        // access the contents of the sfrs
        // at this point it is just a simple extra set of 32 registers
        return getSFR(instruction_.cobr.src2).i;
    } else {
        return getGPRValue(instruction_.cobr.src2, TreatAsInteger{});
    }
}
Register& 
Core::getSFR(byte index) noexcept {
    return sfrs_.get(index);
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
Core::mark() noexcept {
    nextInstruction();
    if (pc_.processControls.traceEnable && tc_.trace.breakpointTraceMode) {
        generateFault(MarkTraceFault);
    }
}
void
Core::fmark() noexcept {
    // advance first so that our return value will always be correct
    nextInstruction();
    if (pc_.processControls.traceEnable) {
        generateFault(MarkTraceFault);
    }
}
void
Core::restoreStandardFrame() noexcept {
    // need to leave the current call
    moveGPR(FPIndex, PFPIndex, TreatAsOrdinal{});
    // remember that the lowest 6 bits are ignored so it is important to mask
    // them out of the frame pointer address when using the address
    auto realAddress = getGPRValue(FPIndex, TreatAsOrdinal{}) & NotC;
    restoreRegisterSet(realAddress);
    setIP(getGPRValue(RIPIndex, TreatAsOrdinal{}), TreatAsOrdinal{});
    advanceInstruction_ = false;
}
void
Core::ret() {
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
            generateFault(UnimplementedFault);
            break;
    }
}

Ordinal
Core::computeAddress() noexcept {
    if (instruction_.isMEMA()) {
        Ordinal result = instruction_.mem.offset;
        if (instruction_.mema.action) {
            result += getGPRValue(instruction_.mem.abase, TreatAsOrdinal{});
        }
        return result;
    } else {
        // okay so we need to figure out the minor mode after figuring out if
        // it is a double wide operation or not
        if (instruction_.memb.group) {
            // okay so it is going to be the displacement versions
            // load 32-bits into the optionalDisplacement field
            instructionLength_ = 8;
            Integer result = static_cast<Integer>(load(ip_.a + 4, TreatAsOrdinal{})); // load the optional displacement
            if (instruction_.memb_grp2.useIndex) {
                result += (getGPRValue(instruction_.memb_grp2.index, TreatAsInteger{}) << static_cast<Integer>(instruction_.memb_grp2.scale));
            }
            if (instruction_.memb_grp2.registerIndirect) {
                result += getGPRValue(instruction_.memb_grp2.abase, TreatAsInteger{});
            }
            return static_cast<Ordinal>(result);
        } else {
            // okay so the other group isn't as cleanly designed
            switch (instruction_.memb.modeMinor) {
                case 0b00: // Register Indirect
                    return getGPRValue(instruction_.memb.abase, TreatAsOrdinal{});
                case 0b01: // IP With Displacement 
                    instructionLength_ = 8;
                    return static_cast<Ordinal>(ip_.i + load(ip_.a + 4, TreatAsInteger{}) + 8);
                case 0b11: // Register Indirect With Index
                    return getGPRValue(instruction_.memb.abase, TreatAsOrdinal{}) + (getGPRValue(instruction_.memb.index, TreatAsOrdinal{}) << instruction_.memb.scale);
                default:
                    return -1;
            }
        }
    }
}
void
Core::storeBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept {
    for (byte i = 0; i < count; ++i, baseAddress += 4) {
        store(baseAddress, getGPRValue(baseRegister, i, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
void
Core::loadBlock(Ordinal baseAddress, byte baseRegister, byte count) noexcept {
    for (byte i = 0; i < count; ++i, baseAddress += 4) {
        setGPR(baseRegister, i, load(baseAddress, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
void 
Core::ldl() noexcept {
    if ((instruction_.mem.srcDest & 0b1) != 0) {
        generateFault(InvalidOperandFault);
    } else {
        loadBlock(computeAddress(), instruction_.mem.srcDest, 2);
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}

void
Core::stl() noexcept {
    if ((instruction_.mem.srcDest & 0b1) != 0) {
        generateFault(InvalidOperandFault);
    } else {
        storeBlock(computeAddress(), instruction_.mem.srcDest, 2);
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}
void
Core::ldt() noexcept {
    if ((instruction_.mem.srcDest & 0b11) != 0) {
        generateFault(InvalidOperandFault);
    } else {
        loadBlock(computeAddress(), instruction_.mem.srcDest, 3);
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}

void
Core::stt() noexcept {
    if ((instruction_.mem.srcDest & 0b11) != 0) {
        generateFault(InvalidOperandFault);
    } else {
        storeBlock(computeAddress(), instruction_.mem.srcDest, 3);
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}

void
Core::ldq() noexcept {
    if ((instruction_.mem.srcDest & 0b11) != 0) {
        generateFault(InvalidOperandFault);
    } else {
        loadBlock(computeAddress(), instruction_.mem.srcDest, 4);
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}

void
Core::stq() noexcept {
    if ((instruction_.mem.srcDest & 0b11) != 0) {
        generateFault(InvalidOperandFault);
    } else {
        storeBlock(computeAddress(), instruction_.mem.srcDest, 4);
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}
void
Core::saveReturnAddress(byte linkRegister) noexcept {
    setGPR(linkRegister, ip_.getValue(TreatAsOrdinal{}) + instructionLength_, TreatAsOrdinal{});
}
void 
Core::balx(byte linkRegister, Ordinal branchTo) noexcept {
    saveReturnAddress(linkRegister);
    setIP(branchTo, TreatAsOrdinal{});
}
void 
Core::balx(byte linkRegister) noexcept {
    balx(linkRegister, computeAddress());
}
void
Core::balx() noexcept {
    balx(instruction_.mem.srcDest);
}
bool 
Core::registerSetAvailable() noexcept {
    /// @todo implement this properly when we implement support for register
    /// sets
    return false;
}
void
Core::allocateNewRegisterFrame() noexcept {
    // do nothing right now
}
void 
Core::saveRegisterSet(Ordinal fp) noexcept {
    // save the "next" register frame to main memory to reclaim it
    storeBlock(fp, 16, 16);
}
void
Core::restoreRegisterSet(Ordinal fp) noexcept {
    loadBlock(fp, 16, 16);
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
Core::callx() noexcept {
    // wait for any uncompleted instructions to finish
    auto temp = getNextFrameBase(); // round stack pointer to next boundary
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    balx(RIPIndex);
    enterCall(fp);
    setupNewFrameInternals(fp, temp);
}
void 
Core::call(Integer displacement) {
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
        generateFault(ProtectionLengthFault);
    } else {
        syncf();
        auto tempPE = load(getSystemProcedureTableBase() + 48 + (4 * targ), TreatAsOrdinal{});
        auto type = static_cast<uint8_t>(tempPE & 0b11);
        auto procedureAddress = tempPE & ~0b11;
        // read entry from system-procedure table, where spbase is address of
        // system-procedure table from Initial Memory Image
        balx(RIPIndex, procedureAddress);
        Ordinal temp = 0;
        byte tempRRR = 0;
        if ((type == 0b00) || pc_.inSupervisorMode()) {
            temp = getNextFrameBase();
            tempRRR = 0;
        } else {
            temp = getSupervisorStackPointer();
            tempRRR = 0b010 | (pc_.processControls.traceEnable ? 0b001 : 0);
            pc_.processControls.executionMode = 1;
            pc_.processControls.traceEnable = temp & 0b1;
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
Core::bx() noexcept {
    setIP(computeAddress(), TreatAsOrdinal{});
}
void
Core::performRegisterTransfer(byte mask, byte count) noexcept {
    if (((instruction_.reg.srcDest & mask) != 0) || ((instruction_.reg.src1 & mask) != 0)) {
        generateFault(InvalidOpcodeFault);
    }
    for (byte i = 0; i < count; ++i) {
        setGPR(instruction_.reg.srcDest, i, unpackSrc1(i, TreatAsOrdinal{}, TreatAsREG{}), TreatAsOrdinal{});
    }
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
Core::branch(Integer displacement) noexcept {
    ip_.i += displacement;
    advanceInstruction_ = false;
}
void
Core::branchConditional(bool condition, Integer displacement) noexcept {
    if (condition) {
        branch(displacement);
    }
}

/**
 * @brief And the condition code with the consistent mask found in the
 * instruction encoding; returns true if the value returned is not zero
 */
bool
Core::getMaskedConditionCode() noexcept {
    return (ac_.getConditionCode() & instruction_.getInstructionMask()) != 0;
}

bool
Core::conditionCodeEqualsMask() noexcept {
    return ac_.getConditionCode() == instruction_.getInstructionMask();
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
void
Core::faultGeneric() noexcept {
    nextInstruction();
    if (fullConditionCodeCheck()) {
        generateFault(ConstraintRangeFault);
    }
}
void
Core::cycle() noexcept {
    instruction_.setValue(load(ip_.a, TreatAsOrdinal{}), TreatAsOrdinal{});
    instructionLength_ = 4;
    advanceInstruction_ = true;
    if (auto opcode = instruction_.getOpcode(); instruction_.isCTRL()) {
        processInstruction(opcode, instruction_.ctrl.displacement, TreatAsCTRL{});
    } else if (instruction_.isCOBR()) {

    } else {
        auto& regDest = getGPR(instruction_.reg.srcDest);
        auto src2o = unpackSrc2(TreatAsOrdinal{}, TreatAsREG{});
        auto src2i = unpackSrc2(TreatAsInteger{}, TreatAsREG{});
        auto src1o = unpackSrc1(TreatAsOrdinal{}, TreatAsREG{});
        auto src1i = unpackSrc1(TreatAsInteger{}, TreatAsREG{});

        switch (instruction_.getOpcode()) {
            case Opcodes::lda:
                setGPR(instruction_.mem.srcDest, computeAddress(), TreatAsOrdinal{});
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
                loadBlock(computeAddress(), instruction_.mem.srcDest, 1);
                break;
            case Opcodes::st: 
                storeBlock(computeAddress(), instruction_.mem.srcDest, 1);
                break;
            case Opcodes::ldob:
                setGPR(instruction_.mem.srcDest, load(computeAddress(), TreatAs<ByteOrdinal>{}), TreatAsOrdinal{});
                break;
            case Opcodes::stob:
                store(computeAddress(), getGPRValue(instruction_.mem.srcDest, TreatAs<Ordinal>{}), TreatAs<ByteOrdinal>{});
                break;
            case Opcodes::ldos:
                setGPR(instruction_.mem.srcDest, load(computeAddress(), TreatAs<ShortOrdinal>{}), TreatAsOrdinal{});
                break;
            case Opcodes::stos:
                store(computeAddress(), getGPRValue(instruction_.mem.srcDest, TreatAsOrdinal{}), TreatAs<ShortOrdinal>{});
                break;
            case Opcodes::ldib:
                setGPR(instruction_.mem.srcDest, load(computeAddress(), TreatAs<ByteInteger>{}), TreatAsInteger{});
                break;
            case Opcodes::stib:
                /// @todo fully implement fault detection
                store(computeAddress(), getGPRValue(instruction_.mem.srcDest, TreatAsInteger{}), TreatAs<ByteInteger>{});
                break;
            case Opcodes::ldis:
                setGPR(instruction_.mem.srcDest, load(computeAddress(), TreatAs<ShortInteger>{}), TreatAsInteger{});
                break;
            case Opcodes::stis:
                /// @todo fully implement fault detection
                store(computeAddress(), getGPRValue(instruction_.mem.srcDest, TreatAsInteger{}), TreatAs<ShortInteger>{});
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
                nand(regDest, src1o, src2o);
                break;
            case Opcodes::andOperation: // and
                andOperation(regDest, src1o, src2o);
                break;
            case Opcodes::clrbit: // clrbit
                clrbit(regDest, src1o, src2o);
                break;
            case Opcodes::andnot: // andnot
                andnot(regDest, src1o, src2o);
                break;
            case Opcodes::notand: // notand
                notand(regDest, src1o, src2o);
                break;
            case Opcodes::notbit: // notbit
                notbit(regDest, src1o, src2o);
                break;
            case Opcodes::xorOperation:
                xorOperation(regDest, src1o, src2o);
                break;
            case Opcodes::setbit:
                setbit(regDest, src1o, src2o);
                break;
            case Opcodes::orOperation: // or
                orOperation(regDest, src1o, src2o);
                break;
            case Opcodes::nor: // nor
                nor(regDest, src1o, src2o);
                break;
            case Opcodes::xnor: 
                xnor(regDest, src1o, src2o);
                break;
            case Opcodes::notOperation: 
                notOperation(regDest, src1o);
                break;
            case Opcodes::ornot: // ornot
                ornot(regDest, src1o, src2o);
                break;
            case Opcodes::notor: // notor
                notor(regDest, src1o, src2o);
                break;
            case Opcodes::alterbit: // alterbit
                alterbit(regDest, src1o, src2o);
                break;
            case Opcodes::addo:
                addo(regDest, src1o, src2o);
                break;
            case Opcodes::addi: // addi
                addi(regDest, src1i, src2i);
                break;
            case Opcodes::subo: // subo
                                // I remember this trick from college, subtraction is just addition
                                // with a negative second argument :). I never gave it much thought
                                // until now but it seems to be an effective trick to save space.
                subo(regDest, src1o, src2o);
                break;
            case Opcodes::subi: // subi
                subi(regDest, src1i, src2i);
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
                cmpo(src1o, src2o);
                break;
            case Opcodes::cmpi: // cmpi
                cmpi(src1o, src2o);
                break;
            case Opcodes::concmpo: // concmpo
                concmpo(src1o, src2o);
                break;
            case Opcodes::concmpi: // concmpi
                concmpi(src1i, src2i);
                break;
            case Opcodes::cmpinco: // cmpinco
                cmpinco(regDest, src1o, src2o);
                break;
            case Opcodes::cmpinci: // cmpinci
                cmpinci(regDest, src1i, src2i);
                break;
            case Opcodes::cmpdeco: // cmpdeco
                cmpdeco(regDest, src1i, src2i);
                break;
            case Opcodes::cmpdeci: // cmpdeci
                cmpdeci(regDest, src1i, src2i);
                break;
            case Opcodes::scanbyte: // scanbyte
                scanbyte(src2o, src1o);
                break;
            case Opcodes::chkbit: // chkbit
                ac_.arith.conditionCode = ((src2o & computeBitPosition(src1o)) == 0 ? 0b000 : 0b010);
                break;
            case Opcodes::addc: 
                addc(regDest, src1o, src2o);
                break;
            case Opcodes::subc:
                subc(regDest, src1o, src2o);
                break;
            case Opcodes::mov:
                regDest.setValue<Ordinal>(src1o);
                break;
            case Opcodes::movl:
                performRegisterTransfer(0b1, 2);
                break;
            case Opcodes::movt:
                performRegisterTransfer(0b11, 3);
                break;
            case Opcodes::movq:
                performRegisterTransfer(0b11, 4);
                break;
            case Opcodes::syncf:
                syncf();
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
                mult<Ordinal>(regDest, src1o, src2o, TreatAsOrdinal{});
                break;
            case Opcodes::muli:
                mult<Integer>(regDest, src1i, src2i, TreatAsInteger{});
                break;
            case Opcodes::divi:
                divi(regDest, src1i, src2i);
                break;
            case Opcodes::divo:
                divo(regDest, src1o, src2o);
                break;
            case Opcodes::remo:
                remo(regDest, src1o, src2o);
                break;
            case Opcodes::remi:
                remi(regDest, src1i, src2i);
                break;
            case Opcodes::modi: 
                modi(regDest, src1i, src2i);
                break;
            case Opcodes::modify:
                regDest.setValue<Ordinal>(modify(src1o, src2o, regDest.getValue<Ordinal>()));
                break;
            case Opcodes::extract:
                // taken from the Hx manual as it isn't insane
                regDest.setValue<Ordinal>((regDest.o >> (src1o > 32 ? 32 : src1o)) & ~(0xFFFF'FFFF << src2o));
                break;
            case Opcodes::modac: 
                regDest.setValue<Ordinal>(ac_.modify(src1o, src2o));
                break;
            case Opcodes::modtc: 
                regDest.setValue<Ordinal>(tc_.modify(src1o, src2o));
                break;
            case Opcodes::modpc:
                if (auto mask = src1o; mask != 0) {
                    if (!pc_.inSupervisorMode()) {
                        generateFault(TypeMismatchFault);
                    } else {
                        regDest.setValue<Ordinal>(pc_.modify(mask, src2o));
                        if (regDest.getPriority() > pc_.getPriority()) {
                            checkForPendingInterrupts();
                        }
                    }
                } else {
                    regDest.setValue<Ordinal>(pc_.getValue<Ordinal>());
                }
                break;
            case Opcodes::atadd:
                atadd(regDest, src1o, src2o);
                break;
            case Opcodes::atmod:
                atmod(regDest, src1o, src2o);
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
                synmov(getGPR(instruction_.reg.src1), src2o);
                break;
            case Opcodes::synmovl:
                synmovl(getGPR(instruction_.reg.src1), src2o);
                break;
            case Opcodes::synmovq:
                synmovq(getGPR(instruction_.reg.src1), src2o);
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
                generateFault(UnimplementedFault);
                break;
        }
    }
    if (advanceInstruction_) {
        nextInstruction();
    }
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
    ip_.clear();
    for (int i = 0; i < 32; ++i) {
        getGPR(i).clear();
    }
    running_ = false;
    //Serial.println(F("DONE"));
}

void 
Core::synld(Register& dest, Ordinal src) noexcept {
    ac_.arith.conditionCode = 0b000;
    if (auto tempa = maskValue<decltype(src), 0xFFFF'FFFC>(src); tempa == 0xFF00'0004) {
        // interrupt control register needs to be read through this
        ac_.arith.conditionCode = 0b010;
        // copy the contents of the interrupt control register to a target
        // register
        dest.setValue(ictl_.o, TreatAsOrdinal{});
    } else {
        ac_.arith.conditionCode = 0b010;
        dest.setValue(load(tempa, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}
void 
Core::synmov(Register& dest, Ordinal src) noexcept {
    ac_.arith.conditionCode = 0b000;
    if (auto tempa = maskValue<Ordinal, 0xFFFF'FFFC>(dest.getValue(TreatAsOrdinal{})); tempa == 0xFF00'0004) {
        ictl_.o = load(src, TreatAsOrdinal{});
        ac_.arith.conditionCode = 0b010;
    } else {
        auto temp = load(src, TreatAsOrdinal{});
        store(tempa, temp, TreatAsOrdinal{});
        // wait for completion
        ac_.arith.conditionCode = 0b010;
    }
}
void 
Core::synmovl(Register& dest, Ordinal src) noexcept {
    ac_.arith.conditionCode = 0b000;
    auto tempa = maskValue<Ordinal, 0xFFFF'FFF8>(dest.getValue(TreatAsOrdinal{}));
    auto tempLower = load(src, TreatAsOrdinal{});
    auto tempUpper = load(src + 4, TreatAsOrdinal{});
    store(tempa, tempLower, TreatAsOrdinal{});
    store(tempa + 4, tempUpper, TreatAsOrdinal{});
    // wait for completion
    ac_.arith.conditionCode = 0b010;
}
void 
Core::synmovq(Register& dest, Ordinal src) noexcept {

    ac_.arith.conditionCode = 0b000;
    auto temp0 = load(src, TreatAsOrdinal{});
    auto temp1 = load(src+4, TreatAsOrdinal{});
    auto temp2 = load(src+8, TreatAsOrdinal{});
    auto temp3 = load(src+12, TreatAsOrdinal{});
    if (auto tempa = maskValue<Ordinal, 0xFFFF'FFF0>(dest.getValue(TreatAsOrdinal{})); tempa == 0xFF00'0010) {
        sendIAC(iac::Message{temp0, temp1, temp2, temp3});
        ac_.arith.conditionCode = 0b010;
    } else {
        store(tempa, temp0, TreatAsOrdinal{});
        store(tempa+4, temp1, TreatAsOrdinal{});
        store(tempa+8, temp2, TreatAsOrdinal{});
        store(tempa+12, temp3, TreatAsOrdinal{});
        // wait for completion
        ac_.arith.conditionCode = 0b010;
    }
}

void 
Core::performSelect(Register& dest, Ordinal src1, Ordinal src2, bool condition) noexcept {
    dest.setValue(condition ? src2 : src1, TreatAsOrdinal{});
}
void
Core::performConditionalSubtract(Register& dest, Integer src1, Integer src2, bool condition, TreatAsInteger) noexcept {
    if (condition) {
        subi(dest, src1, src2);
    }
}

void
Core::performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, bool condition, TreatAsOrdinal) noexcept {
    if (condition) {
        subo(dest, src1, src2);
    }
}

void
Core::performConditionalAdd(Register& dest, Integer src1, Integer src2, bool condition, TreatAsInteger) noexcept {
    if (condition) {
        addi(dest, src1, src2);
    }
}

void
Core::performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, bool condition, TreatAsOrdinal) noexcept {
    if (condition) {
        addo(dest, src1, src2);
    }
}
bool
Core::performSelfTest() noexcept {
    /// @todo add self test routines here to sanity check things before doing
    /// checksum work
    return true;
}
BootResult
Core::start() noexcept {
    assertFailureState();
    if (!performSelfTest()) {
        return BootResult::SelfTestFailure;
    } else {
        // start execution at this point, according to the docs this is what we
        // want to do. 
        deassertFailureState();
        // after this point we now need to jump 
        // Kx has a startup pin to denote if it is the startup processor or not, we
        // are not doing that here
        Ordinal x[8] = { 0 };
        for (int i = 0, j = 0; i < 8; ++i, j+=4) {
            x[i] = load(j, TreatAsOrdinal{});
            Serial.print(F("\tx["));
            Serial.print(i);
            Serial.print(F("]: 0x"));
            Serial.println(x[i], HEX);
        }

        ac_.arith.conditionCode = 0b000; // clear condition code
        Register temp_{0};
        addc(temp_, 0xFFFF'FFFF, x[0]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[1]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[2]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[3]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[4]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[5]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[6]);
        addc(temp_, temp_.getValue(TreatAsOrdinal{}), x[7]);
        if (temp_.getValue(TreatAsOrdinal{}) != 0) {
            assertFailureState();
            Serial.print(F("FAILED CHECKSUM VALUE: 0x"));
            Serial.println(temp_.getValue(TreatAsOrdinal{}), HEX);
            return BootResult::ChecksumFail;
        } else {
            systemAddressTableBase_ = x[0];
            prcbAddress_ = x[1];
            ip_.setValue(x[3], TreatAsOrdinal{});
            // fetch IMI
            setGPR(FPIndex, load(prcbAddress_ + 24, TreatAsOrdinal{}), TreatAsOrdinal{});
            pc_.processControls.priority = 31;
            pc_.processControls.state = 1;
            // clear any latched external interrupt/IAC signals
            // begin execution
            running_ = true;
            return BootResult::Success;
        }
    }
}
void
Core::stop() noexcept {
    running_ = false;
    // this will halt the cpu
}

void
Core::assertFailureState() noexcept {
    /// @todo implement
}

void
Core::deassertFailureState() noexcept {
    /// @todo implement
}
void
Core::addi(Register& dest, Integer src1, Integer src2) noexcept {
    add<Integer>(dest, src1, src2, TreatAsInteger{});
    nextInstruction();
    /// @todo implement overflow detection and fault generation
}

void
Core::addo(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    add<Ordinal>(dest, src1, src2, TreatAsOrdinal{});
}

void
Core::generateFault(Ordinal faultCode) noexcept {
    /// @todo implement
}

Ordinal
Core::getStackPointer() const noexcept {
    return getGPRValue(SPIndex, TreatAsOrdinal{});
}
void
Core::setStackPointer(Ordinal value, TreatAsOrdinal) noexcept {
    setGPR(SPIndex, value, TreatAsOrdinal{});
}
Ordinal
Core::getNextFrameBase() const noexcept {
    return (getStackPointer() + C) & NotC;
}

void
Core::nextInstruction() noexcept {
    ip_.o += instructionLength_;
    advanceInstruction_ = false;
}

void
Core::setIP(Ordinal value, TreatAsOrdinal) noexcept {
    // when we set the ip during this instruction, we need to turn off the
    // automatic advancement!
    ip_.setValue<Ordinal>(value);
    advanceInstruction_ = false;
}
void
Core::subi(Register& dest, Integer src1, Integer src2) noexcept {
    sub<Integer>(dest, src1, src2, TreatAsInteger{});
    nextInstruction();
    /// @todo implement overflow fault detection
}

void
Core::subo(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    sub<Ordinal>(dest, src1, src2, TreatAsOrdinal{});
}
void 
Core::processInstruction(Opcodes opcode, Integer displacement, TreatAsCTRL) noexcept {
    switch (opcode) {
        case Opcodes::bal: // bal
            saveReturnAddress(LRIndex);
            // then fallthrough and take the branch
        case Opcodes::b: // b
            branch(displacement);
            break;
        case Opcodes::call: // call
            call(displacement);
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
            branchConditional(fullConditionCodeCheck(), displacement);
            break;
        case Opcodes::faultno:
        case Opcodes::faulte:
        case Opcodes::faultne:
        case Opcodes::faultl:
        case Opcodes::faultle:
        case Opcodes::faultg:
        case Opcodes::faultge:
        case Opcodes::faulto: 
            faultGeneric();
            break;
        default:
            generateFault(UnimplementedFault);
            break;
    }
}
void
Core::processInstruction(Opcodes opcode, uint8_t src1, const Register& src2, int16_t displacement, TreatAsCOBR) noexcept {
    switch(opcode) {
        case Opcodes::bbc:
            bbc(src1, src2, displacement);
            break;
        case Opcodes::bbs:
            bbs(src1, src2, displacement);
            break;
        default:
            // test instructions perform modifications to src1 so we must error out
            // in this case!
            generateFault(UnimplementedFault);
            break;
    }
}
void 
Core::processInstruction(Opcodes opcode, Register& src1, const Register& src2, int16_t displacement, TreatAsCOBR) noexcept {
    switch(opcode) {
        case Opcodes::bbc:
            bbc(src1, src2, displacement);
            break;
        case Opcodes::bbs:
            bbs(src1, src2, displacement);
            break;
        case Opcodes::testno:
        case Opcodes::testg:
        case Opcodes::teste:
        case Opcodes::testge:
        case Opcodes::testl:
        case Opcodes::testne:
        case Opcodes::testle:
        case Opcodes::testo:
            src1.setValue<Ordinal>(fullConditionCodeCheck() ? 1 : 0);
            break;
        default:
            generateFault(UnimplementedFault);
            break;
    }
}
