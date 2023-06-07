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

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include "Types.h"
#include "Core.h"
#include "BinaryOperations.h"
Ordinal 
Register::modify(Ordinal mask, Ordinal src) noexcept {
    auto tmp = o;
    o = ::modify(mask, src, o);
    return tmp;
}


void 
Core::mulo(Register& regDest, Ordinal src1o, Ordinal src2o) noexcept {
    mult<Ordinal>(regDest, src1o, src2o, TreatAsOrdinal{});
}


void 
Core::muli(Register& regDest, Integer src1o, Integer src2o) noexcept {
    mult<Integer>(regDest, src1o, src2o, TreatAsInteger{});
}

void 
Core::modify(Register& regDest, Ordinal src1o, Ordinal src2o) noexcept {
    regDest.setValue<Ordinal>(::modify(src1o, src2o, regDest.getValue<Ordinal>()));
}

void 
Core::extract(Register& regDest, Ordinal bitpos, Ordinal len) noexcept {
    // taken from the Hx manual as it isn't insane
    auto actualBitpos = bitpos > 32 ? 32 : bitpos;
    regDest.setValue<Ordinal>((static_cast<Ordinal>(regDest) >> actualBitpos) & ~(0xFFFF'FFFF << len));
}

const Register& 
Core::getSrc1Register(TreatAsREG) const noexcept {
    if (instruction_.reg.m1) {
        /// @todo what to do if s1 is also set?
        return constants_.get(instruction_.reg.src1);
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1);
    } else {
        return getGPR(instruction_.reg.src1);
    }
}

const Register& 
Core::getSrc2Register(TreatAsREG) const noexcept {
    if (instruction_.reg.m2) {
        /// @todo what to do if s1 is also set?
        return constants_.get(instruction_.reg.src2);
    } else if (instruction_.reg.s2) {
        return getSFR(instruction_.reg.src2);
    } else {
        return getGPR(instruction_.reg.src2);
    }
}

Ordinal 
Core::unpackSrc1(TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        /// @todo what to do if s1 is also set?
        return constants_.getValue<Ordinal>(instruction_.reg.src1);
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1).o;
    } else {
        return getGPRValue(instruction_.reg.src1, TreatAsOrdinal{});
    }
}

Ordinal 
Core::unpackSrc1(ByteOrdinal offset, TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        // literals should always return zero if offset is greater than zero
        return offset == 0 ? constants_.getValue<Ordinal>(instruction_.reg.src1) : 0;
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1, offset).o;
    } else {
        return getGPRValue(instruction_.reg.src1, offset, TreatAsOrdinal{});
    }
}

Integer 
Core::unpackSrc1(TreatAsInteger, TreatAsREG) noexcept {
    if (instruction_.reg.m1) {
        return constants_.getValue<Integer>(instruction_.reg.src1);
    } else if (instruction_.reg.s1) {
        return getSFR(instruction_.reg.src1).i;
    } else {
        return getGPRValue(instruction_.reg.src1, TreatAsInteger{});
    }
}

Ordinal 
Core::unpackSrc2(TreatAsOrdinal, TreatAsREG) noexcept {
    if (instruction_.reg.m2) {
        return constants_.getValue<Ordinal>(instruction_.reg.src2);
    } else if (instruction_.reg.s2) {
        return getSFR(instruction_.reg.src2).o;
    } else {
        return getGPRValue(instruction_.reg.src2, TreatAsOrdinal{});
    }
}

Integer 
Core::unpackSrc2(TreatAsInteger, TreatAsREG) noexcept {
    if (instruction_.reg.m2) {
        return constants_.getValue<Integer>(instruction_.reg.src2);
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
Core::emul(LongRegister& dest, Ordinal src1, Ordinal src2) noexcept {
    if (!aligned(instruction_.reg.srcDest, TreatAsLongRegister{})) {
        /// Since this is unaligned and the destination will always be aligned,
        /// we just do an expensive access of the two unaligned registers
        /// instead. Set the both to 0xFFFF'FFFF
        auto& lower = getGPR(instruction_.reg.srcDest);
        auto& upper = getGPR(instruction_.reg.srcDest + 1);
        lower.template setValue<Ordinal>(0xFFFF'FFFF);
        upper.template setValue<Ordinal>(0xFFFF'FFFF);
        invalidOpcodeFault();
    }  else {
        dest.template setValue<LongOrdinal>(static_cast<LongOrdinal>(src2) * static_cast<LongOrdinal>(src1));
    }
}


void
Core::ediv(LongRegister& dest, Ordinal src1, const LongRegister& src2) noexcept {
    if (!aligned(instruction_.reg.srcDest, TreatAsLongRegister{}) || !aligned(instruction_.reg.src2, TreatAsLongRegister{})) {
        /// Since this is unaligned and the destination will always be aligned,
        /// we just do an expensive access of the two unaligned registers
        /// instead. Set the both to 0xFFFF'FFFF
        auto& lower = getGPR(instruction_.reg.srcDest);
        auto& upper = getGPR(instruction_.reg.srcDest + 1);
        lower.template setValue<Ordinal>(0xFFFF'FFFF);
        upper.template setValue<Ordinal>(0xFFFF'FFFF);
        invalidOpcodeFault();
    } else if (src1 == 0) {
        // divide by zero
        zeroDivideFault();
    } else {
        //src2.parts[1] = getGPRValue(instruction_.reg.src2, 1, TreatAsOrdinal{});
        auto sl2 = src2.getValue<LongOrdinal>();
        dest.template setValue<Ordinal>(1, sl2 / src1); // quotient
        dest.template setValue<Ordinal>(0, static_cast<Ordinal>(sl2 - (sl2 / src1) * src1)); // remainder
    }
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
Core::getSFR(ByteOrdinal index) noexcept {
    return sfrs_.get(index);
}

Register& 
Core::getSFR(ByteOrdinal index, ByteOrdinal offset) noexcept {
    return getSFR((index + offset) & 0b11111);
}

const Register& 
Core::getSFR(ByteOrdinal index) const noexcept {
    return sfrs_.get(index);
}

const Register& 
Core::getSFR(ByteOrdinal index, ByteOrdinal offset) const noexcept {
    return getSFR((index + offset) & 0b11111);
}


void
Core::mark() noexcept {
    nextInstruction();
    if (pc_.processControls.traceEnable && tc_.trace.breakpointTraceMode) {
        markTraceFault();
        //generateFault(MarkTraceFault);
    }
}

void
Core::fmark() noexcept {
    // advance first so that our return value will always be correct
    nextInstruction();
    if (pc_.processControls.traceEnable) {
        markTraceFault();
        //generateFault(MarkTraceFault);
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
Core::storeBlock(Ordinal baseAddress, ByteOrdinal baseRegister, ByteOrdinal count) noexcept {
    for (ByteOrdinal i = 0; i < count; ++i, baseAddress += 4) {
        store(baseAddress, getGPRValue(baseRegister, i, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}

void
Core::loadBlock(Ordinal baseAddress, ByteOrdinal baseRegister, ByteOrdinal count) noexcept {
    for (ByteOrdinal i = 0; i < count; ++i, baseAddress += 4) {
        setGPR(baseRegister, i, load(baseAddress, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
}

void 
Core::ldl(Address effectiveAddress, LongRegister& destination) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsLongRegister{})) {
        invalidOperandFault();
        /// @todo perform an unaligned load into registers
    } else {
        destination[0] = load(effectiveAddress + 0, TreatAsOrdinal{});
        destination[1] = load(effectiveAddress + 4, TreatAsOrdinal{});
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}


void
Core::stl(Address effectiveAddress, const LongRegister& source) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsLongRegister{})) {
        invalidOperandFault();
        /// @todo perform an unaligned load into registers
    } else {
        // support unaligned accesses
        store(effectiveAddress + 0,  static_cast<Ordinal>(source[0]), TreatAsOrdinal{});
        store(effectiveAddress + 4,  static_cast<Ordinal>(source[1]), TreatAsOrdinal{});
    }
    // the instruction is invalid so we should complete after we are done
}

void
Core::ldt(Address effectiveAddress, TripleRegister& destination) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsTripleRegister{})) {
        invalidOperandFault();
        /// @todo perform an unaligned load into registers
    } else {
        destination[0] = load(effectiveAddress + 0, TreatAsOrdinal{});
        destination[1] = load(effectiveAddress + 4, TreatAsOrdinal{});
        destination[2] = load(effectiveAddress + 8, TreatAsOrdinal{});
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}


void
Core::stt(Address effectiveAddress, const TripleRegister& source) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsTripleRegister{})) {
        invalidOperandFault();
    } else {
        // support unaligned accesses
        store(effectiveAddress + 0,  static_cast<Ordinal>(source[0]), TreatAsOrdinal{});
        store(effectiveAddress + 4,  static_cast<Ordinal>(source[1]), TreatAsOrdinal{});
        store(effectiveAddress + 8,  static_cast<Ordinal>(source[2]), TreatAsOrdinal{});
    }
    // the instruction is invalid so we should complete after we are done
}


void
Core::ldq(Address effectiveAddress, QuadRegister& destination) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsQuadRegister{})) {
        invalidOperandFault();
        /// @todo perform an unaligned load into registers
    } else {
        destination[0] = load(effectiveAddress + 0, TreatAsOrdinal{});
        destination[1] = load(effectiveAddress + 4, TreatAsOrdinal{});
        destination[2] = load(effectiveAddress + 8, TreatAsOrdinal{});
        destination[3] = load(effectiveAddress + 12, TreatAsOrdinal{});
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
}


void
Core::stq(Address effectiveAddress, const QuadRegister& source) noexcept {
    if (!aligned(instruction_.mem.srcDest, TreatAsQuadRegister{})) {
        invalidOperandFault();
    } else {
        store(effectiveAddress + 0,  static_cast<Ordinal>(source[0]), TreatAsOrdinal{});
        store(effectiveAddress + 4,  static_cast<Ordinal>(source[1]), TreatAsOrdinal{});
        store(effectiveAddress + 8,  static_cast<Ordinal>(source[2]), TreatAsOrdinal{});
        store(effectiveAddress + 12, static_cast<Ordinal>(source[3]), TreatAsOrdinal{});
        // support unaligned accesses
    }
    // the instruction is invalid so we should complete after we are done
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
Core::performRegisterTransfer(ByteOrdinal mask, ByteOrdinal count) noexcept {
    // perform the register transfer first and then check to see if we were
    // offset at all
    for (ByteOrdinal i = 0; i < count; ++i) {
        setGPR(instruction_.reg.srcDest, i, unpackSrc1(i, TreatAsOrdinal{}, TreatAsREG{}), TreatAsOrdinal{});
    }
    if (((instruction_.reg.srcDest & mask) != 0) || ((instruction_.reg.src1 & mask) != 0)) {
        nextInstruction();
        invalidOpcodeFault();
    }
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
 * @param mask The mask to apply to see if we get a value back
 */

bool
Core::getMaskedConditionCode(uint8_t mask) const noexcept {
    return (ac_.getConditionCode() & mask) != 0;
}


bool
Core::conditionCodeEqualsMask(uint8_t mask) const noexcept {
    return ac_.getConditionCode() == mask;
}

bool
Core::fullConditionCodeCheck() noexcept {
    return fullConditionCodeCheck(instruction_.getInstructionMask());
}

bool
Core::fullConditionCodeCheck(uint8_t mask) noexcept {
    // the second condition handles the case where we are looking at unordered
    // output where it is only true if it is equal to zero. So if it turns out
    // that the condition code is zero and the mask is the unordered kind then
    // return true :). In all other cases, the second check will either fail
    // (because the condition code is zero) or it will never fire because the
    // masked condition code will be non zero.
    return getMaskedConditionCode(mask) || conditionCodeEqualsMask(mask);
}

void
Core::faultGeneric() noexcept {
    nextInstruction();
    if (fullConditionCodeCheck()) {
        constraintRangeFault();
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
        Register& src2 = instruction_.cobr.s2 ? getSFR(instruction_.cobr.src2) : getGPR(instruction_.cobr.src2);
        if (instruction_.cobr.m1) {
            auto src1Value = static_cast<uint8_t>(instruction_.cobr.src1);
            processInstruction(opcode,
                    instruction_.getInstructionMask(),
                    src1Value, 
                    src2,
                    instruction_.cobr.displacement,
                    TreatAsCOBR{});
        } else {
            processInstruction(opcode,
                    instruction_.getInstructionMask(),
                    getGPR(instruction_.cobr.src1),
                    src2,
                    instruction_.cobr.displacement,
                    TreatAsCOBR{});
        }
    } else if (instruction_.isMEMFormat()) {
        auto effectiveAddress = computeAddress();
        Register& destination = getGPR(instruction_.reg.srcDest);
        processInstruction(opcode, destination, effectiveAddress, TreatAsMEM{});
    } else if (instruction_.isREGFormat()) {
        auto& regDest = getGPR(instruction_.reg.srcDest);
        const auto& src1 = getSrc1Register(TreatAsREG{});
        const auto& src2 = getSrc2Register(TreatAsREG{});
        processInstruction(opcode, regDest, src1, src2, TreatAsREG{});
    } else {
        unimplementedFault();
    }
    if (advanceInstruction_) {
        nextInstruction();
    }
}



void
Core::begin() noexcept {
    // so we need to do any sort of processor setup here
    ip_.clear();
    for (int i = 0; i < 32; ++i) {
        getGPR(i).clear();
        /// @todo setup SFRs
        constants_.setValue<Ordinal>(i, i);
    }
    nonPortableBegin();
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
Core::synmov(const Register& dest, Ordinal src) noexcept {
    ac_.arith.conditionCode = 0b000;
    if (auto tempa = maskValue<Ordinal, 0xFFFF'FFFC>(dest.getValue(TreatAsOrdinal{})); tempa == 0xFF00'0004) {
        ictl_ = load(src, TreatAsOrdinal{});
        ac_.arith.conditionCode = 0b010;
    } else {
        auto temp = load(src, TreatAsOrdinal{});
        store(tempa, temp, TreatAsOrdinal{});
        // wait for completion
        ac_.arith.conditionCode = 0b010;
    }
}

void 
Core::synmovl(const Register& dest, Ordinal src) noexcept {
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
Core::synmovq(const Register& dest, Ordinal src) noexcept {

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
Core::performSelect(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    dest.setValue(fullConditionCodeCheck() ? src2 : src1, TreatAsOrdinal{});
}

void
Core::performConditionalSubtract(Register& dest, Integer src1, Integer src2, TreatAsInteger) noexcept {
    if (fullConditionCodeCheck()) {
        subi(dest, src1, src2);
    }
}


void
Core::performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) noexcept {
    if (fullConditionCodeCheck()) {
        subo(dest, src1, src2);
    }
}


void
Core::performConditionalAdd(Register& dest, Integer src1, Integer src2, TreatAsInteger) noexcept {
    if (fullConditionCodeCheck()) {
        addi(dest, src1, src2);
    }
}


void
Core::performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) noexcept {
    if (fullConditionCodeCheck()) {
        addo(dest, src1, src2);
    }
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
            return BootResult::Success;
        }
    }
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




Ordinal
Core::getStackPointer() const noexcept {
    return getGPRValue(SPIndex, TreatAsOrdinal{});
}

void
Core::setStackPointer(Ordinal value, TreatAsOrdinal) noexcept {
    setGPR(SPIndex, value, TreatAsOrdinal{});
}
namespace {
template<Ordinal C, Ordinal NotC>
constexpr Ordinal computeNextFrame(Ordinal base) noexcept {
    return (base + C) & NotC;
}
static_assert(computeNextFrame<Core::C, Core::NotC>(0xFDED'0000) == 0xFDED'0000);
static_assert(computeNextFrame<Core::C*2, Core::NotC>(0xFDED'0000) == 0xFDED'0040);
static_assert(computeNextFrame<Core::C*3, Core::NotC>(0xFDED'0000) == 0xFDED'0080);
static_assert(computeNextFrame<Core::C*4, Core::NotC>(0xFDED'0000) == 0xFDED'00C0);
} // end namespace
Ordinal
Core::getNextFrameBase() const noexcept {
    return computeNextFrame<C, NotC>(getStackPointer());
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
            unimplementedFault();
            break;
    }
}

void
Core::processInstruction(Opcodes opcode, uint8_t mask, uint8_t src1, const Register& src2, int16_t displacement, TreatAsCOBR) noexcept {
    switch(opcode) {
        case Opcodes::bbc:
            bbc(src1, src2, displacement);
            break;
        case Opcodes::bbs:
            bbs(src1, src2, displacement);
            break;
        case Opcodes::cmpobg:
        case Opcodes::cmpobe:
        case Opcodes::cmpobge:
        case Opcodes::cmpobl:
        case Opcodes::cmpobne:
        case Opcodes::cmpoble:
            cmpobGeneric(mask, src1, src2.getValue<Ordinal>(), displacement);
            break;
        case Opcodes::cmpibno: // never branches
        case Opcodes::cmpibg:
        case Opcodes::cmpibe:
        case Opcodes::cmpibge:
        case Opcodes::cmpibl:
        case Opcodes::cmpibne:
        case Opcodes::cmpible:
        case Opcodes::cmpibo: // always branches
            cmpibGeneric(mask, src1, src2.getValue<Integer>(), displacement);
            break;
        default:
            // test instructions perform modifications to src1 so we must error out
            // in this case!
            unimplementedFault();
            break;
    }
}

void 
Core::processInstruction(Opcodes opcode, uint8_t mask, Register& src1, const Register& src2, int16_t displacement, TreatAsCOBR) noexcept {
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
            src1.setValue<Ordinal>(fullConditionCodeCheck(mask) ? 1 : 0);
            break;
        case Opcodes::cmpobg:
        case Opcodes::cmpobe:
        case Opcodes::cmpobge:
        case Opcodes::cmpobl:
        case Opcodes::cmpobne:
        case Opcodes::cmpoble:
            cmpobGeneric(mask, src1.getValue<Ordinal>(), src2.getValue<Ordinal>(), displacement);
            break;
        case Opcodes::cmpibno: // never branches
        case Opcodes::cmpibg:
        case Opcodes::cmpibe:
        case Opcodes::cmpibge:
        case Opcodes::cmpibl:
        case Opcodes::cmpibne:
        case Opcodes::cmpible:
        case Opcodes::cmpibo: // always branches
            cmpibGeneric(mask, src1.getValue<Integer>(), src2.getValue<Integer>(), displacement);
            break;
        default:
            unimplementedFault();
            break;
    }
}

void 
Core::processInstruction(Opcodes opcode, Register& srcDest, Address effectiveAddress, TreatAsMEM) noexcept {
    switch (opcode) {
        case Opcodes::balx:
            balx(srcDest, effectiveAddress);
            break;
        case Opcodes::bx:
            setIP(effectiveAddress, TreatAsOrdinal{});
            break;
        case Opcodes::callx:
            callx(effectiveAddress);
            break;
        case Opcodes::st: 
            store(effectiveAddress, srcDest.getValue<Ordinal>(), TreatAs<Ordinal>{});
            break;
        case Opcodes::stob:
            store(effectiveAddress, srcDest.getValue<Ordinal>(), TreatAs<ByteOrdinal>{});
            break;
        case Opcodes::stos:
            store(effectiveAddress, srcDest.getValue<Ordinal>(), TreatAs<ShortOrdinal>{});
            break;
        case Opcodes::stib:
            // If the register data is too large to be stored as a byte or
            // short word, the value is truncated and the integer overflow
            // condition is signalled.
            /// @todo fully implement fault detection
            store(effectiveAddress, srcDest.getValue<Integer>(), TreatAs<ByteInteger>());
            break;
        case Opcodes::stis:
            // If the register data is too large to be stored as a byte or
            // short word, the value is truncated and the integer overflow
            // condition is signalled.
            store(effectiveAddress, srcDest.getValue<Integer>(), TreatAs<ShortInteger>{});
            /// @todo fully implement fault detection
            break;
        case Opcodes::ld: 
            srcDest.setValue<Ordinal>(load(effectiveAddress, TreatAsOrdinal{}));
            break;
        case Opcodes::ldob:
            srcDest.setValue(load(effectiveAddress, TreatAs<ByteOrdinal>{}), TreatAsOrdinal{});
            break;
        case Opcodes::ldos:
            srcDest.setValue(load(effectiveAddress, TreatAs<ShortOrdinal>{}), TreatAsOrdinal{});
            break;
        case Opcodes::ldib:
            srcDest.setValue<Integer>(load(effectiveAddress, TreatAs<ByteInteger>{}));
            break;
        case Opcodes::ldis:
            srcDest.setValue<Integer>(load(effectiveAddress, TreatAs<ShortInteger>{}));
            break;
        case Opcodes::ldl:
            ldl(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsLongRegister{}));
            break;
        case Opcodes::stl:
            stl(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsLongRegister{}));
            break;
        case Opcodes::ldt:
            ldt(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsTripleRegister{}));
            break;
        case Opcodes::stt:
            stt(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsTripleRegister{}));
            break;
        case Opcodes::ldq:
            ldq(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsQuadRegister{}));
            break;
        case Opcodes::stq:
            stq(effectiveAddress, getGPR(instruction_.mem.srcDest, TreatAsQuadRegister{}));
            break;
        case Opcodes::lda:
            srcDest.setValue<Ordinal>(effectiveAddress);
            break;
        default:
            unimplementedFault();
            break;
    }
}

void 
Core::processInstruction(Opcodes opcode, Register& regDest, const Register& src1, const Register& src2, TreatAsREG) noexcept {
    auto src2o = static_cast<Ordinal>(src2);
    auto src1o = static_cast<Ordinal>(src1);
    auto src2i = static_cast<Integer>(src2);
    auto src1i = static_cast<Integer>(src1);
    switch (opcode) {
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
            // in some of the opcodeExt values seem to reflect the resultant truth
            // table for the operation :). That's pretty cool
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
        case Opcodes::shri: 
            shri(regDest, src1i, src2i);
            break;
        case Opcodes::shlo: 
            shlo(regDest, src1o, src2o);
            break;
        case Opcodes::rotate: 
            rotate(regDest, src1o, src2o);
            break;
        case Opcodes::shli: 
            shli(regDest, src1i, src2i);
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
            mulo(regDest, src1o, src2o);
            break;
        case Opcodes::muli:
            muli(regDest, src1i, src2i);
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
            modify(regDest, src1o, src2o);
            break;
        case Opcodes::extract:
            extract(regDest, src1o, src2o);
            break;
        case Opcodes::modac: 
            modxc(ac_, regDest, src1o, src2o);
            break;
        case Opcodes::modtc: 
            modxc(tc_, regDest, src1o, src2o);
            break;
        case Opcodes::modpc:
            modpc(regDest, src1o, src2o);
            break;
        case Opcodes::atadd:
            atadd(regDest, src1o, src2o);
            break;
        case Opcodes::atmod:
            atmod(regDest, src1o, src2o);
            break;
        case Opcodes::emul:
            emul(getGPR(instruction_.reg.srcDest, TreatAsLongRegister{}), src1o, src2o);
            break;
        case Opcodes::ediv:
            ediv(getGPR(instruction_.reg.srcDest, TreatAsLongRegister{}),
                 src1o,
                 getGPR(instruction_.reg.src2, TreatAsLongRegister{}));
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
            synmov(src1, src2o);
            break;
        case Opcodes::synmovl:
            synmovl(src1, src2o);
            break;
        case Opcodes::synmovq:
            synmovq(src1, src2o);
            break;
        case Opcodes::selno:
        case Opcodes::sele:
        case Opcodes::selg:
        case Opcodes::selge:
        case Opcodes::sell:
        case Opcodes::selne:
        case Opcodes::selle:
        case Opcodes::selo:
            performSelect(regDest, src1o, src2o);
            break;
        case Opcodes::addono:
        case Opcodes::addoe:
        case Opcodes::addog:
        case Opcodes::addoge:
        case Opcodes::addol:
        case Opcodes::addone:
        case Opcodes::addole:
        case Opcodes::addoo:
            performConditionalAdd(regDest, src1o, src2o, TreatAsOrdinal{});
            break;

        case Opcodes::addino:
        case Opcodes::addie:
        case Opcodes::addig:
        case Opcodes::addige:
        case Opcodes::addil:
        case Opcodes::addine:
        case Opcodes::addile:
        case Opcodes::addio:
            performConditionalAdd(regDest, src1i, src2i, TreatAsInteger{});
            break;
        case Opcodes::subono:
        case Opcodes::suboe:
        case Opcodes::subog:
        case Opcodes::suboge:
        case Opcodes::subol:
        case Opcodes::subone:
        case Opcodes::subole:
        case Opcodes::suboo:
            performConditionalSubtract(regDest, src1o, src2o, TreatAsOrdinal{});
            break;

        case Opcodes::subino:
        case Opcodes::subie:
        case Opcodes::subig:
        case Opcodes::subige:
        case Opcodes::subil:
        case Opcodes::subine:
        case Opcodes::subile:
        case Opcodes::subio:
            performConditionalSubtract(regDest, src1i, src2i, TreatAsInteger{});
            break;
        default:
            unimplementedFault();
            break;
    }
}



void
Core::modpc(Register& regDest, Ordinal src1o, Ordinal src2o) noexcept {
    if (auto mask = src1o; mask != 0) {
        if (!pc_.inSupervisorMode()) {
            typeMismatchFault();
        } else {
            regDest.setValue<Ordinal>(pc_.modify(mask, src2o));
            if (regDest.getPriority() > pc_.getPriority()) {
                checkForPendingInterrupts();
            }
        }
    } else {
        regDest.setValue<Ordinal>(pc_.getValue<Ordinal>());
    }
}

void 
Core::modxc(Register& control, Register& dest, Ordinal src1, Ordinal src2) noexcept {
    dest.setValue<Ordinal>(control.modify(src1, src2));
}

void 
Core::shlo(Register& srcDest, Ordinal src1, Ordinal src2) noexcept {
    srcDest.setValue<Ordinal>(src1 < 32 ? src2 << src1 : 0);
}

void 
Core::shli(Register& srcDest, Integer src1, Integer src2) noexcept {
    srcDest.setValue<Integer>(src2 << src1);
}

void 
Core::rotate(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    dest.setValue<Ordinal>(rotateOperation(src2, src1));
}

void 
Core::shri(Register& dest, Integer src1, Integer src2) noexcept {
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
    dest.setValue<Integer>(src2 >> src1);
}


bool
Core::performSelfTest() noexcept {
    auto clearRegisters = [this]() {
        for (int i = 0; i < 32; ++i) {
            getGPR(i).clear();
        }
    };
    // test different instructions to see if they are working correctly
    auto testRegisters = [this]() {
        for (int i = 0; i < 32; ++i) {
            auto& temporary = getGPR(i);
            auto randomValue = static_cast<Ordinal>(random());
            auto randomInteger = static_cast<Integer>(random());
            temporary.template setValue<Ordinal>(randomValue);
            if (temporary.template getValue<Ordinal>() != randomValue) {
                return false;
            }
            temporary.template setValue<Integer>(randomInteger);
            if (temporary.template getValue<Integer>() != randomInteger) {
                return false;
            }
        }
        return true;
    };
    // test move operations
    // first mov
    auto testMoveOperations = [this]() {
        auto& g0 = getGPR(0);
        auto& g1 = getGPR(1);
        auto randomSourceValue = static_cast<Ordinal>(random());
        g0.template setValue<Ordinal>(randomSourceValue);
        g1.template setValue<Ordinal>(0xFFFF'FFFF);
        g1.template setValue<Ordinal>(g0.template getValue<Ordinal>()); 
        if (g1.template getValue<Ordinal>() != g0.template getValue<Ordinal>()) {
            return false;
        }
        auto randomSourceValue2 = static_cast<Ordinal>(random());
        auto& gl0 = getGPR(0, TreatAsLongRegister{});
        auto& gl1 = getGPR(2, TreatAsLongRegister{});
        gl0[0] = randomSourceValue;
        gl0[1] = randomSourceValue2;
        if (static_cast<Ordinal>(gl0[0]) != randomSourceValue) {
            return false;
        }
        if (static_cast<Ordinal>(gl0[1]) != randomSourceValue2) {
            return false;
        }
        gl1 = gl0;
        if (static_cast<Ordinal>(gl1[0]) != randomSourceValue) {
            return false;
        }
        if (static_cast<Ordinal>(gl1[1]) != randomSourceValue2) {
            return false;
        }
        return true;
    };
    auto makeGenericOperation = [this](auto maker, auto doIt, auto converter, auto name, auto genSrc1, auto genSrc2) {
        return [this, maker, doIt, converter, name, genSrc1, genSrc2](ByteOrdinal gpr0 = random() & 0b11111, 
                ByteOrdinal gpr1 = random() & 0b11111, 
                ByteOrdinal gpr2 = random() & 0b11111) -> bool {
            auto rs0 = converter(genSrc1());
            auto rs1 = converter(genSrc2());
            auto& src1 = getGPR(gpr0);
            src1 = rs0;
            auto& src2 = getGPR(gpr1);
            src2 = rs1;
            auto& dest = getGPR(gpr2);
            auto rs2 = maker(converter(src1), converter(src2), converter(dest));
            doIt(dest, converter(src1), converter(src2));
            if (converter(dest) != rs2) {
                return false;
            }
            return true;
        };
    };
    auto genericIntegerOperation = [this, makeGenericOperation](auto maker, auto doIt, auto name, auto genSrc1, auto genSrc2) {
        return makeGenericOperation(maker, doIt, [](auto value) { return static_cast<Integer>(value); }, name, genSrc1, genSrc2);
    };
    auto genericOrdinalOperation = [this, makeGenericOperation](auto maker, auto doIt, auto name, auto genSrc1, auto genSrc2) {
        return makeGenericOperation(maker, doIt, [](auto value) { return static_cast<Ordinal>(value); }, name, genSrc1, genSrc2);
    };
    auto makeIntegerOperation = [this, genericIntegerOperation](auto maker, auto doIt, auto name) {
        return genericIntegerOperation(maker, doIt, name, doRandom, doRandom);
    };
    auto makeOrdinalOperation = [this, genericOrdinalOperation](auto maker, auto doIt, auto name) {
        return genericOrdinalOperation(maker, doIt, name, doRandom, doRandom);
    };
    auto runTest = [this, clearRegisters](auto fn) {
        return makeTestRunner(fn, clearRegisters, clearRegisters);
    };
    return runTestCases(runTest,
            testMoveOperations,
            testRegisters,
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 + src1; }, 
                                  [this](auto& dest, auto src1, auto src2) { return addi(dest, src1, src2); },
                                  F("addi"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 + src1; },
                                  [this](auto& dest, auto src1, auto src2) { return addo(dest, src1, src2); },
                                  F("addo"))),
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 - src1; }, 
                                  [this](auto& dest, auto src1, auto src2) { return subi(dest, src1, src2); },
                                  F("subi"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 - src1; },
                                  [this](auto& dest, auto src1, auto src2) { return subo(dest, src1, src2); },
                                  F("subo"))),
            (genericIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 / src1; }, 
                                     [this](auto& dest, auto src1, auto src2) { return divi(dest, src1, src2); },
                                     F("divi"),
                                     doRandomDisallow0, doRandom)),
            (genericOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 / src1; },
                                     [this](auto& dest, auto src1, auto src2) { return divo(dest, src1, src2); },
                                     F("divo"),
                                     doRandomDisallow0, doRandom)),
            (genericIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 % src1; }, 
                                     [this](auto& dest, auto src1, auto src2) { return remi(dest, src1, src2); },
                                     F("remi"),
                                     doRandomDisallow0, doRandom)),
            (genericOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 % src1; },
                                     [this](auto& dest, auto src1, auto src2) { return remo(dest, src1, src2); },
                                     F("remo"),
                                     doRandomDisallow0, doRandom)),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 & src1); },
                                  [this](auto& dest, auto src1, auto src2) { return nand(dest, src1, src2); },
                                  F("nand"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 | src1); },
                                  [this](auto& dest, auto src1, auto src2) { return nor(dest, src1, src2); },
                                  F("nor"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 | src1); },
                                  [this](auto& dest, auto src1, auto src2) { return orOperation(dest, src1, src2); },
                                  F("or"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 & src1); },
                                  [this](auto& dest, auto src1, auto src2) { return andOperation(dest, src1, src2); },
                                  F("and"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 ^ src1); },
                                  [this](auto& dest, auto src1, auto src2) { return xorOperation(dest, src1, src2); },
                                  F("xor"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 ^ src1); },
                                  [this](auto& dest, auto src1, auto src2) { return xnor(dest, src1, src2); },
                                  F("xnor"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return rotateOperation(src2, src1); },
                                  [this](auto& dest, auto src1, auto src2) { return rotate(dest, src1, src2); },
                                  F("rotate"))),
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return (src2 >> src1); },
                                  [this](auto& dest, auto src1, auto src2) { return shri(dest, src1, src2); },
                                  F("shri"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src1 < 32 ? (src2 << src1) : 0; },
                                  [this](auto& dest, auto src1, auto src2) { return shlo(dest, src1, src2); },
                                  F("shlo"))),
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return (src2 << src1); },
                                  [this](auto& dest, auto src1, auto src2) { return shli(dest, src1, src2); },
                                  F("shli"))),
            (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 * src1; }, 
                                  [this](auto& dest, auto src1, auto src2) { return muli(dest, src1, src2); },
                                  F("muli"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 * src1; },
                                  [this](auto& dest, auto src1, auto src2) { return mulo(dest, src1, src2); },
                                  F("mulo"))),
            (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal dest) { return ::modify(src1, src2, dest); },
                                  [this](auto& dest, auto src1, auto src2) { return modify(dest, src1, src2); },
                                  F("modify")))
                ) && runNonPortableSelfTests();
}

void
Core::checkForPendingInterrupts() noexcept {
    /// @todo implement
}

void 
Core::sendIAC(const iac::Message& msg) noexcept {
    /// @todo implement
    switch (msg.messageType) {
        /// @todo implement different message types
        case 0x40: // dispatch interrupt
            dispatchInterrupt(msg.field1);
            break;
        case 0x89: // purge instruction cache
            purgeInstructionCache();
            break;
        case 0x93: // reinitialize processor
            reinitializeProcessor(msg.field3, msg.field4, msg.field5);
            break;
        case 0x8f: // set breakpoint register
            setBreakpointRegister(msg.field3, msg.field4);
            break;
        case 0x80: // store system base
            storeSystemBase(msg.field3);
            break;
        case 0x41: // test for pending interrupts
            testPendingInterrupts();
            break;
        default:
            break;
    }
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
Core::allocateNewRegisterFrame() noexcept {
    // making a new register frame is not necessary for this implementation
}

void 
Core::saveRegisterSet(Ordinal fp) noexcept {
    storeBlock(fp, 16, 16);
}

void 
Core::restoreRegisterSet(Ordinal fp) noexcept {
    loadBlock(fp, 16, 16);
}

bool 
Core::registerSetAvailable() noexcept {
    return false;
}

void
Core::dispatchInterrupt(uint8_t vector) noexcept {
    
}


void 
Core::reinitializeProcessor(Ordinal satBase, Ordinal prcbBase, Ordinal startIP) noexcept {
    
}

void 
Core::setBreakpointRegister(Ordinal breakpointIp0, Ordinal breakpointIp1) noexcept {
    /// @todo do something with the breakpoint data
    breakpoint0_ = breakpointIp0 & 0xFFFFFFFC;
    breakpoint0Active_ = (breakpointIp0 & 0b10) != 0;
    breakpoint1_ = breakpointIp1 & 0xFFFFFFFC;
    breakpoint1Active_ = (breakpointIp1 & 0b10) != 0;
}

void 
Core::storeSystemBase(Ordinal destinationAddress) noexcept {
    store(destinationAddress, systemAddressTableBase_, TreatAsOrdinal{});
    store(destinationAddress+4, prcbAddress_, TreatAsOrdinal{});
}

void 
Core::testPendingInterrupts() noexcept {

}
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

    return FaultTableEntry { 
        load(realAddress, TreatAsOrdinal{}),
        load(realAddress + 4, TreatAsOrdinal{})
    };
}

SegmentDescriptor
Core::loadSegmentDescriptor(SegmentSelector selector) const noexcept {
    auto index = translateSegmentDescriptorToOffset(selector);
    // now that we have the table index, we just need to compute the base
    // offset
    auto baseIndex = systemAddressTableBase_;
    baseIndex += (index * sizeof(SegmentDescriptor));
    // so now we have the base address of the segment descriptor we want
    return SegmentDescriptor {
        load(baseIndex, TreatAsOrdinal{}),
        load(baseIndex+4, TreatAsOrdinal{}),
        load(baseIndex+8, TreatAsOrdinal{}),
        load(baseIndex+12, TreatAsOrdinal{}),
    };
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
    auto faultRecordStart = nextFrame - 48;
    auto fp = getGPRValue(FPIndex, TreatAsOrdinal{});
    // save the current registers to the stack
    enterCall(fp);
    // manually setup the stack frame as needed
    auto& pfp = getGPR(PFPIndex);
    // clear the p bit as well to make sure
    pfp.setValue(getGPRValue(FPIndex, TreatAsOrdinal{}) & ~0b1'111, TreatAsOrdinal{});
    pfp.pfp.rt = 0b001;
    setGPR(FPIndex, nextFrame, TreatAsOrdinal{});
    setStackPointer(nextFrame + 64, TreatAsOrdinal{});
    pushFaultRecord(nextFrame - 48, record);
    // no need to push a resumption record right now and set the resume flag in
    // the saved process controls
    setIP(address, TreatAsOrdinal{});
}


void
Core::generateFault(const FaultRecord& record) {
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
Core::zeroDivideFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       ZeroDivideFault,
                       (Ordinal)ip_);
    saveReturnAddress(RIPIndex);
    generateFault(record);
}

void
Core::integerOverflowFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       IntegerOverflowFault,
                       (Ordinal)ip_);
    saveReturnAddress(RIPIndex);
    // saved ip will be the next instruction
    generateFault(record);
}

void
Core::constraintRangeFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       ConstraintRangeFault,
                       (Ordinal)ip_);
    // saved ip isn't used!
    generateFault(record);
}

void
Core::invalidSSFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       InvalidSSFault,
                       (Ordinal)ip_);
    // saved ip isn't used
    generateFault(record);
}

void
Core::markTraceFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       MarkTraceFault,
                       (Ordinal)ip_);
    // saved ip isn't used
    generateFault(record);
}

void
Core::unimplementedFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       UnimplementedFault,
                       (Ordinal)ip_);
    // saved ip isn't used
    generateFault(record);
}

void
Core::invalidOpcodeFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       InvalidOpcodeFault,
                       (Ordinal)ip_);
    // saved ip isn't used
    generateFault(record);
}

void
Core::invalidOperandFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       InvalidOperandFault,
                       (Ordinal)ip_);
    // saved ip isn't used
    generateFault(record);
}

void
Core::protectionLengthFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       SegmentLengthFault,
                       (Ordinal)ip_);
    // saved ip isn't used
    generateFault(record);
}
void
Core::typeMismatchFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       TypeMismatchFault,
                       (Ordinal)ip_);
    // saved ip isn't used
    generateFault(record);
}

void
Core::invalidDescriptorFault(SegmentSelector ss) {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       InvalidDescriptorFault,
                       (Ordinal)ip_);
    record.faultData[1] = (ss & ~0b11111);
    setGPR(RIPIndex, (Ordinal)ip_, TreatAsOrdinal{});
    generateFault(record);
}
void
Core::eventNoticeFault() {
    FaultRecord record((Ordinal)pc_, 
                       (Ordinal)ac_,
                       EventNoticeFault,
                       (Ordinal)ip_);
    saveReturnAddress(RIPIndex);
    generateFault(record);
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
    Address baseStackAddress = 0;
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
Address
Core::getInterruptTableBaseAddress() const {
    return getInterruptTablePointer();
}

void
Core::postPendingInterrupt(InterruptVector vector) {
}

Address
Core::getInterruptVectorAddress(uint8_t vector) const {
    if (vector < 8) {
        // vectors 0 - 7 are not useful so just return zero
        return 0;
    } else {
        auto realizedOffset = vector - 8; // make this a clean offset by
                                        // subtracting 8 
        Address byteOffset = sizeof(Address) * realizedOffset;
        return load(getInterruptTableBaseAddress() + 36 + byteOffset, TreatAsOrdinal{});
    }
}

void
Core::receiveInterrupt(InterruptVector vector) {
    auto priority = computeInterruptPriority(vector);
    if (priority != 31) {
        auto systemPriority = pc_.processControls.priority;
        if (priority <= systemPriority) {
            postPendingInterrupt(vector);
            return;
        }
    } 
}

bool
Core::getPendingPriorityBit(uint8_t priority) const {
    return (getInterruptPendingPriorities() & computeBitPosition(priority)) != 0;
}

void
Core::setPendingPriorityBit(uint8_t priority) {
    auto pp = getInterruptPendingPriorities();
    pp |= computeBitPosition(priority);
    setInterruptPendingPriorities(pp);
}

void
Core::clearPendingPriorityBit(uint8_t priority) {
    auto pp = getInterruptPendingPriorities();
    pp &= (~(computeBitPosition(priority)));
    setInterruptPendingPriorities(pp);
}


bool
Core::vectorIsPending(InterruptVector vector) const {
    // need to retrieve the ordinal which contains our corresponding pending bits
    return (getPendingInterruptWord(vector) & computeBitPosition(computeInterruptVectorBitOffset(vector)));
}

void
Core::clearPendingVector(InterruptVector vector) {
    auto pi = getPendingInterruptWord(vector);
    pi &= (~(computeBitPosition(computeInterruptVectorBitOffset(vector))));
    setPendingInterruptWord(vector, pi);
}

void
Core::setPendingVector(InterruptVector vector) {
    auto pi = getPendingInterruptWord(vector);
    pi |= computeBitPosition(computeInterruptVectorBitOffset(vector));
    setPendingInterruptWord(vector, pi);
}
