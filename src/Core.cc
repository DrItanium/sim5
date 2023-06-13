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
#include "Disassembly.h"
#include <iostream>


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


void
Core::scanbyte(Ordinal src1, Ordinal src2) noexcept {
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
    // since we are clearing the conditionCode we should just do an assignment
    ac_.arith.conditionCode = 0;
    // set the overflow bit
    if ((src2MSB == src1MSB) && (src2MSB != destMSB)) {
        ac_.arith.conditionCode |= 0b001;
    }
    if (result != 0) {
        DEBUG_LOG_LEVEL(4) {
            std::cout << "carry bit set" << std::endl;
        }
        ac_.arith.conditionCode |= 0b010;
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




Ordinal
Core::computeAddress() noexcept {
    DEBUG_ENTER_FUNCTION;
    Ordinal result = 0;
    if (instruction_.isMEMA()) {
        DEBUG_LOG_LEVEL(4) {
            std::cout << '\t' << __PRETTY_FUNCTION__ << ": ismema operation" << std::endl;
        }
        result = instruction_.mem.offset;
        if (instruction_.mema.action) {
            result += getGPRValue(instruction_.mem.abase, TreatAsOrdinal{});
        }
    } else {
        DEBUG_LOG_LEVEL(4) {
            std::cout << '\t' << __PRETTY_FUNCTION__ << ": ismemb operation" << std::endl;
        }
        // okay so we need to figure out the minor mode after figuring out if
        // it is a double wide operation or not
        if (instruction_.memb.group) {
            DEBUG_LOG_LEVEL(4) {
                std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": group bit is high" << std::endl;
            }
            // okay so it is going to be the displacement versions
            // load 32-bits into the optionalDisplacement field
            instructionLength_ = 8;
            auto iresult = static_cast<Integer>(load(ip_.a + 4, TreatAsOrdinal{})); // load the optional displacement
            if (instruction_.memb_grp2.useIndex) {
                iresult += (getGPRValue(instruction_.memb_grp2.index, TreatAsInteger{}) << static_cast<Integer>(instruction_.memb_grp2.scale));
            }
            if (instruction_.memb_grp2.registerIndirect) {
                iresult += getGPRValue(instruction_.memb_grp2.abase, TreatAsInteger{});
            }
            result = static_cast<Ordinal>(iresult);
        } else {
            DEBUG_LOG_LEVEL(4) {
                std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": group bit is low" << std::endl;
            }
            // okay so the other group isn't as cleanly designed
            switch (instruction_.memb.modeMinor) {
                case 0b00: // Register Indirect
                    DEBUG_LOG_LEVEL(4) {
                        std::cout << "\t\t\t" << __PRETTY_FUNCTION__ << ": register indirect" << std::endl;
                        std::cout << "\t\t\t" << __PRETTY_FUNCTION__ << ": abase 0x" << std::hex << instruction_.memb.abase << std::endl;
                    }
                    result = getGPRValue(instruction_.memb.abase, TreatAsOrdinal{});
                    break;
                case 0b01: // IP With Displacement 
                    DEBUG_LOG_LEVEL(4){
                        std::cout << "\t\t\t" << __PRETTY_FUNCTION__ << ": ip with displacement" << std::endl;
                    }
                    instructionLength_ = 8;
                    result = static_cast<Ordinal>(ip_.i + load(ip_.a + 4, TreatAsInteger{}) + 8);
                    break;
                case 0b11: // Register Indirect With Index
                    DEBUG_LOG_LEVEL(4) {
                        std::cout << "\t\t\t" << __PRETTY_FUNCTION__ << ": register indirect with index" << std::endl;
                    }
                    result = getGPRValue(instruction_.memb.abase, TreatAsOrdinal{}) + (getGPRValue(instruction_.memb.index, TreatAsOrdinal{}) << instruction_.memb.scale);
                    break;
                default:
                    result = -1;
                    break;
            }
        }
    }
    DEBUG_LOG_LEVEL(1) {
        std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": result = 0x" << std::hex << result << std::endl;
    }
    DEBUG_LEAVE_FUNCTION;
    return result;
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
    DEBUG_ENTER_FUNCTION;
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
    DEBUG_LEAVE_FUNCTION;
    // the instruction is invalid so we should complete after we are done
}


void
Core::stq(Address effectiveAddress, const QuadRegister& source) noexcept {
    DEBUG_ENTER_FUNCTION;
    if (!aligned(instruction_.mem.srcDest, TreatAsQuadRegister{})) {
        invalidOperandFault();
    } else {
        store(effectiveAddress + 0,  static_cast<Ordinal>(source[0]), TreatAsOrdinal{});
        store(effectiveAddress + 4,  static_cast<Ordinal>(source[1]), TreatAsOrdinal{});
        store(effectiveAddress + 8,  static_cast<Ordinal>(source[2]), TreatAsOrdinal{});
        store(effectiveAddress + 12, static_cast<Ordinal>(source[3]), TreatAsOrdinal{});
        // support unaligned accesses
    }
    DEBUG_LEAVE_FUNCTION;
    // the instruction is invalid so we should complete after we are done
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
    DEBUG_ENTER_FUNCTION;
    DEBUG_LOG_LEVEL(1) {
        std::cout << "{" << std::endl;
        auto ipValue = ip_.getValue<Ordinal>();
        std::cout << "IP: 0x" << std::hex << ipValue << std::endl;
    }
    DEBUG_LOG_LEVEL(4) {
        std::cout << "\tGETTING INSTRUCTION CONTENTS" << std::endl;
    }
    instruction_.setValue(load(ip_.a, TreatAsOrdinal{}), TreatAsOrdinal{});
    instructionLength_ = 4;
    advanceInstruction_ = true;
    DEBUG_LOG_LEVEL(1) {
        std::cout << __PRETTY_FUNCTION__  << ": " << disassembleInstruction(ip_.getValue<Ordinal>(), instruction_) << std::endl;
    }
    DEBUG_LOG_LEVEL(4) {
        std::cout << "\tGOT INSTRUCTION CONTENTS" << std::endl;
    }
    if (auto opcode = instruction_.getOpcode(); instruction_.isCTRL()) {
        processInstruction(opcode, static_cast<Integer>(instruction_.getDisplacement(TreatAsCTRL{})), TreatAsCTRL{});
    } else if (instruction_.isCOBR()) {
        Register& src2 = instruction_.cobr.s2 ? getSFR(instruction_.cobr.src2) : getGPR(instruction_.cobr.src2);
        if (instruction_.cobr.m1) {
            auto src1Value = static_cast<uint8_t>(instruction_.cobr.src1);
            processInstruction(opcode,
                    instruction_.getInstructionMask(),
                    src1Value, 
                    src2,
                    static_cast<ShortInteger>(instruction_.getDisplacement(TreatAsCOBR{})),
                    TreatAsCOBR{});
        } else {
            processInstruction(opcode,
                    instruction_.getInstructionMask(),
                    getGPR(instruction_.cobr.src1),
                    src2,
                    static_cast<ShortInteger>(instruction_.getDisplacement(TreatAsCOBR{})),
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
    DEBUG_LOG_LEVEL(1) {
        std::cout << "}" << std::endl;
    }
    DEBUG_LEAVE_FUNCTION;
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
    for (int i = 0; i < NumberOfLocalRegisterFrames; ++i) {
        frames_[i].clear();
    }
    nonPortableBegin();
}


void 
Core::synld(Register& dest, Ordinal src) noexcept {
    DEBUG_ENTER_FUNCTION;
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
    DEBUG_LEAVE_FUNCTION;
}

void 
Core::synmov(const Register& dest, Ordinal src) noexcept {
    DEBUG_ENTER_FUNCTION;
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
    DEBUG_LEAVE_FUNCTION;
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
    DEBUG_ENTER_FUNCTION;
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
    DEBUG_LEAVE_FUNCTION;
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
void
Core::boot0(Address sat, Address pcb, Address startIP) noexcept {
    DEBUG_ENTER_FUNCTION;
    systemAddressTableBase_ = sat;
    prcbAddress_ = pcb;
    setIP(startIP, TreatAsOrdinal{});
    // fetch IMI
    auto theStackPointer = getInterruptStackAddress();
    // get the interrupt stack pointer base since we are starting in an interrupted context
    // set the frame pointer to the start of the interrupt stack
    setGPR(FPIndex, theStackPointer, TreatAsOrdinal{});
    pc_.setPriority(31);
    pc_.processControls.state = 1;
    localRegisterFrameIndex_ = 0;
    for (auto& a : frames_) {
        a.relinquishOwnership();
        a.clear();
    }
    // the current local register window needs to be owned on startup
    getCurrentPack().takeOwnership(theStackPointer, [](const auto&, auto) noexcept {});
    setStackPointer(theStackPointer + 64, TreatAsOrdinal{});
    setGPR(PFPIndex, theStackPointer, TreatAsOrdinal{});
    /// @todo clear the pending interrupts?
    // clear any latched external interrupt/IAC signals
    // begin execution
    DEBUG_LEAVE_FUNCTION;
}
BootResult
Core::start() noexcept {
    DEBUG_ENTER_FUNCTION;
    assertFailureState();
    if (!performSelfTest()) {
        DEBUG_LEAVE_FUNCTION;
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
            DEBUG_LOG_LEVEL(4) std::cout << "x[" << i << "]: 0x" << std::hex << x[i] << std::endl;
        }
        

        ac_.arith.conditionCode = 0b000; // clear condition code
        Register temp_{0};
        addc(temp_, 0xFFFF'FFFF, x[0]);
        DEBUG_LOG_LEVEL(4) {
            std::cout << "checksum p0: 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
        }
        addc(temp_, temp_.getValue<Ordinal>(), x[1]);
        DEBUG_LOG_LEVEL(4) {
            std::cout << "checksum p1: 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
        }
        addc(temp_, temp_.getValue<Ordinal>(), x[2]);
        DEBUG_LOG_LEVEL(4) {
            std::cout << "checksum p2: 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
        }
        addc(temp_, temp_.getValue<Ordinal>(), x[3]);
        DEBUG_LOG_LEVEL(4) {
            std::cout << "checksum p3: 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
        }
        addc(temp_, temp_.getValue<Ordinal>(), x[4]);
        DEBUG_LOG_LEVEL(4) {
            std::cout << "checksum p4: 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
        }
        addc(temp_, temp_.getValue<Ordinal>(), x[5]);

        DEBUG_LOG_LEVEL(4) {
            std::cout << "checksum p5: 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
        }
        addc(temp_, temp_.getValue<Ordinal>(), x[6]);
        DEBUG_LOG_LEVEL(4) {
            std::cout << "checksum p6: 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
        }
        addc(temp_, temp_.getValue<Ordinal>(), x[7]);
        DEBUG_LOG_LEVEL(4) {
            std::cout << "checksum p7: 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
        }
        if (temp_.getValue(TreatAsOrdinal{}) != 0) {
            assertFailureState();
            DEBUG_LEAVE_FUNCTION;
            return BootResult::ChecksumFail;
        } else {
            boot0(x[0], x[1], x[3]);
            DEBUG_LEAVE_FUNCTION;
            return BootResult::Success;
        }
    }
}


void
Core::addi(Register& dest, Integer src1, Integer src2) noexcept {
    add<Integer>(dest, src1, src2, TreatAsInteger{});
    nextInstruction();
    faultOnOverflow(dest);
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
    faultOnOverflow(dest);
}


void
Core::subo(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    sub<Ordinal>(dest, src1, src2, TreatAsOrdinal{});
}
void
Core::processInstruction(Opcodes opcode, Integer displacement, TreatAsCTRL) noexcept {
    switch (opcode) {
        case Opcodes::bal: // bal
            bal(displacement);
            break;
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
            bx(effectiveAddress);
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
            store(effectiveAddress, static_cast<ByteInteger>(srcDest.getValue<Integer>()), TreatAs<ByteInteger>());
            break;
        case Opcodes::stis:
            // If the register data is too large to be stored as a byte or
            // short word, the value is truncated and the integer overflow
            // condition is signalled.
            store(effectiveAddress, static_cast<ShortInteger>(srcDest.getValue<Integer>()), TreatAs<ShortInteger>{});
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
    switch (opcode) {
        case Opcodes::notbit:
            notbit(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::andOperation:
            andOperation(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::andnot:
            andnot(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::setbit:
            setbit(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::notand: // notand
            notand(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::xorOperation:
            xorOperation(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::orOperation: // or
            orOperation(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::nor: // nor
            nor(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::xnor:
            xnor(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::notOperation:
            notOperation(regDest, static_cast<Ordinal>(src1));
            break;
        case Opcodes::ornot: // ornot
            ornot(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::nand: // nand
            nand(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::clrbit: // clrbit
            clrbit(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::notor: // notor
            notor(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::alterbit: // alterbit
            alterbit(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
            // in some of the opcodeExt values seem to reflect the resultant truth
            // table for the operation :). That's pretty cool
        case Opcodes::addo:
            addo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::addi: // addi
            addi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::subo: // subo
                            // I remember this trick from college, subtraction is just addition
                            // with a negative second argument :). I never gave it much thought
                            // until now but it seems to be an effective trick to save space.
            subo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::subi: // subi
            subi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::shro: // shro
            regDest.setValue<Ordinal>(static_cast<Ordinal>(src1) < 32 ? static_cast<Ordinal>(src2) >> static_cast<Ordinal>(src1) : 0);
            break;
        case Opcodes::shrdi: // shrdi
                             // according to the manual, equivalent to divi value, 2 so that is what we're going to do for correctness sake
            regDest.setValue<Integer>( static_cast<Integer>(src1) < 32 && static_cast<Integer>(src1) >= 0 ? static_cast<Integer>(src2) / static_cast<Integer>(computeBitPosition(static_cast<Integer>(src1))) : 0);
            break;
        case Opcodes::shri: 
            shri(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::shlo: 
            shlo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::rotate: 
            rotate(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::shli: 
            shli(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::cmpo: // cmpo
            cmpo(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::cmpi: // cmpi
            cmpi(static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::concmpo: // concmpo
            concmpo(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::concmpi: // concmpi
            concmpi(static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::cmpinco: // cmpinco
            cmpinco(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::cmpinci: // cmpinci
            cmpinci(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::cmpdeco: // cmpdeco
            cmpdeco(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::cmpdeci: // cmpdeci
            cmpdeci(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::scanbyte: // scanbyte
            scanbyte(static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::chkbit: // chkbit
            ac_.arith.conditionCode = ((static_cast<Ordinal>(src2) & computeBitPosition(static_cast<Ordinal>(src1))) == 0 ? 0b000 : 0b010);
            break;
        case Opcodes::addc: 
            addc(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::subc:
            subc(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::mov:
            regDest.setValue<Ordinal>(static_cast<Ordinal>(src1));
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
            mulo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::muli:
            muli(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::divi:
            divi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::divo:
            divo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::remo:
            remo(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::remi:
            remi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::modi: 
            modi(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2));
            break;
        case Opcodes::modify:
            modify(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::extract:
            extract(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::modac: 
            modxc(ac_, regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::modtc: 
            modxc(tc_, regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::modpc:
            modpc(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::atadd:
            atadd(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::atmod:
            atmod(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::emul:
            emul(getGPR(instruction_.reg.srcDest, TreatAsLongRegister{}), static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::ediv:
            ediv(getGPR(instruction_.reg.srcDest, TreatAsLongRegister{}),
                 static_cast<Ordinal>(src1),
                 getGPR(instruction_.reg.src2, TreatAsLongRegister{}));
            break;
        case Opcodes::calls:
            calls(static_cast<Ordinal>(src1));
            break;
        case Opcodes::spanbit:
            spanbit(regDest, static_cast<Ordinal>(src2), static_cast<Ordinal>(src1));
            break;
        case Opcodes::scanbit:
            scanbit(regDest, static_cast<Ordinal>(src2), static_cast<Ordinal>(src1));
            break;
        case Opcodes::synld:
            synld(regDest, static_cast<Ordinal>(src1));
            break;
        case Opcodes::synmov:
            synmov(src1, static_cast<Ordinal>(src2));
            break;
        case Opcodes::synmovl:
            synmovl(src1, static_cast<Ordinal>(src2));
            break;
        case Opcodes::synmovq:
            synmovq(src1, static_cast<Ordinal>(src2));
            break;
        case Opcodes::selno:
        case Opcodes::sele:
        case Opcodes::selg:
        case Opcodes::selge:
        case Opcodes::sell:
        case Opcodes::selne:
        case Opcodes::selle:
        case Opcodes::selo:
            performSelect(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2));
            break;
        case Opcodes::addono:
        case Opcodes::addoe:
        case Opcodes::addog:
        case Opcodes::addoge:
        case Opcodes::addol:
        case Opcodes::addone:
        case Opcodes::addole:
        case Opcodes::addoo:
            performConditionalAdd(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2), TreatAsOrdinal{});
            break;

        case Opcodes::addino:
        case Opcodes::addie:
        case Opcodes::addig:
        case Opcodes::addige:
        case Opcodes::addil:
        case Opcodes::addine:
        case Opcodes::addile:
        case Opcodes::addio:
            performConditionalAdd(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2), TreatAsInteger{});
            break;
        case Opcodes::subono:
        case Opcodes::suboe:
        case Opcodes::subog:
        case Opcodes::suboge:
        case Opcodes::subol:
        case Opcodes::subone:
        case Opcodes::subole:
        case Opcodes::suboo:
            performConditionalSubtract(regDest, static_cast<Ordinal>(src1), static_cast<Ordinal>(src2), TreatAsOrdinal{});
            break;

        case Opcodes::subino:
        case Opcodes::subie:
        case Opcodes::subig:
        case Opcodes::subige:
        case Opcodes::subil:
        case Opcodes::subine:
        case Opcodes::subile:
        case Opcodes::subio:
            performConditionalSubtract(regDest, static_cast<Integer>(src1), static_cast<Integer>(src2), TreatAsInteger{});
            break;
        case Opcodes::dmovt:
            dmovt(regDest, static_cast<Ordinal>(src1));
            break;
        case Opcodes::classr:
            classr(static_cast<Real>(src1));
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





void 
Core::syncf() noexcept {
    // Wait for all faults to be generated that are associated with any prior
    // uncompleted instructions
    /// @todo implement if it makes sense since we don't have a pipeline
}







