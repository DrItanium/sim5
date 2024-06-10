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
Core::mulo(Register& regDest, Ordinal src1o, Ordinal src2o) {
    mult<Ordinal>(regDest, src1o, src2o, TreatAsOrdinal{});
}


void
Core::muli(Register& regDest, Integer src1o, Integer src2o) {
    mult<Integer>(regDest, src1o, src2o, TreatAsInteger{});
}

void
Core::modify(Register& regDest, Ordinal src1o, Ordinal src2o) {
    regDest.setValue<Ordinal>(::modify(src1o, src2o, regDest.getValue<Ordinal>()));
}

void
Core::extract(Register& regDest, Ordinal bitpos, Ordinal len) {
    // taken from the Hx manual as it isn't insane
    auto actualBitpos = bitpos > 32 ? 32 : bitpos;
    regDest.setValue<Ordinal>((static_cast<Ordinal>(regDest) >> actualBitpos) & ~(0xFFFF'FFFF << len));
}


void
Core::scanbyte(Ordinal src1, Ordinal src2) {
    DEBUG_LOG_LEVEL(1) {
        std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": "
                  << "src1 0x" << std::hex << src1
                  << ", src2 0x" << std::hex << src2
                  << std::endl;
    }
    Register s2(src2), s1(src1);
    ac_.setConditionResult(s1.bytes[0] == s2.bytes[0] ||
                           s1.bytes[1] == s2.bytes[1] ||
                           s1.bytes[2] == s2.bytes[2] ||
                           s1.bytes[3] == s2.bytes[3]);
}

void
Core::arithmeticWithCarryGeneric(Ordinal result, bool src2MSB, bool src1MSB, bool destMSB) {
    // since we are clearing the conditionCode we should just do an assignment
    ac_.clearConditionCode();
    uint8_t cc = 0b000;
    // set the overflow bit
    if ((src2MSB == src1MSB) && (src2MSB != destMSB)) {
        cc |= 0b001;
    }
    cc |= ((result & 0b1) ? 0b010 : 0b000);
    DEBUG_LOG_LEVEL(2) {
        std::cout << "condition code: 0x" << std::hex << static_cast<int>(cc) << std::endl;
    }
    ac_.arith.conditionCode = cc;
}




OptionalFaultRecord
Core::emul(const REGInstruction& inst, LongRegister& dest, Ordinal src1, Ordinal src2) {
    if (!aligned(inst.getSrcDest(), TreatAsLongRegister{})) {
        /// Since this is unaligned and the destination will always be aligned,
        /// we just do an expensive access of the two unaligned registers
        /// instead. Set the both to 0xFFFF'FFFF
        setGPRValue<Ordinal>(inst.getSrcDest(), 0xFFFF'FFFF);
        setGPRValue<Ordinal>(inst.getSrcDest(), 1, 0xFFFF'FFFF);
        return invalidOpcodeFault();
    }  else {
        dest.template setValue<LongOrdinal>(static_cast<LongOrdinal>(src2) * static_cast<LongOrdinal>(src1));
        return std::nullopt;
    }
}


OptionalFaultRecord
Core::ediv(const REGInstruction& inst, LongRegister& dest, Ordinal src1, const LongRegister& src2) {
    if (!aligned(inst.getSrcDest(), TreatAsLongRegister{}) || !aligned(inst.getSrc2(), TreatAsLongRegister{})) {
        /// Since this is unaligned and the destination will always be aligned,
        /// we just do an expensive access of the two unaligned registers
        /// instead. Set the both to 0xFFFF'FFFF
        setGPRValue<Ordinal>(inst.getSrcDest(), 0xFFFF'FFFF);
        setGPRValue<Ordinal>(inst.getSrcDest(), 1, 0xFFFF'FFFF);
        return invalidOpcodeFault();
    } else if (src1 == 0) {
        // divide by zero
        return zeroDivideFault();
    } else {
        //src2.parts[1] = getGPRValue(instruction_.reg.src2, 1, TreatAsOrdinal{});
        auto sl2 = src2.getValue<LongOrdinal>();
        dest.template setValue<Ordinal>(1, sl2 / src1); // quotient
        dest.template setValue<Ordinal>(0, static_cast<Ordinal>(sl2 - (sl2 / src1) * src1)); // remainder
        return std::nullopt;
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

OptionalFaultRecord
Core::mark() {
    nextInstruction();
    if (pc_.processControls.traceEnable && tc_.trace.breakpointTraceMode) {
        return markTraceFault();
    } else {
        return std::nullopt;
    }
}

OptionalFaultRecord
Core::fmark() {
    // advance first so that our return value will always be correct
    nextInstruction();
    if (pc_.processControls.traceEnable) {
        return markTraceFault();
        //generateFault(MarkTraceFault);
    } else {
        return std::nullopt;
    }
}

Ordinal
Core::computeAddress(const MEMInstruction& inst) noexcept {
    DEBUG_ENTER_FUNCTION;
    Integer displacement = 0;
    if (inst.usesOptionalDisplacement()) {
        instructionLength_ = 8;
        displacement = load(ip_.o + 4, TreatAsInteger{});
    }
    using K = MEMInstruction::AddressingMode;
    switch (inst.getAddressingMode()) {
        case K::AbsoluteOffset:
            return inst.getOffset();
        case K::RegisterIndirectWithOffset:
            return inst.getOffset() + getGPRValue<Ordinal>(inst.getABase());
        case K::RegisterIndirect:
            return getGPRValue<Ordinal>(inst.getABase());
        case K::IPWithDisplacement:
            return static_cast<Ordinal>(displacement + ip_.getValue<Integer>() + 8);
        case K::RegisterIndirectWithIndex:
            return getGPRValue<Ordinal>(inst.getABase()) + inst.scaleValue<Ordinal>(getGPRValue<Ordinal>(inst.getIndex()));
        case K::AbsoluteDisplacement:
            return static_cast<Ordinal>(displacement);
        case K::RegisterIndirectWithDisplacement:
            return static_cast<Ordinal>(getGPRValue<Integer>(inst.getABase()) + displacement);
        case K::IndexWithDisplacement:
            return static_cast<Ordinal>(inst.scaleValue<Integer>(getGPRValue<Integer>(inst.getIndex())) + displacement);
        case K::RegisterIndirectWithIndexAndDisplacement:
            return static_cast<Ordinal>(getGPRValue<Integer>(inst.getABase())
                    + inst.scaleValue<Integer>(getGPRValue<Integer>(inst.getIndex()))
                            + displacement);
        default:
            return 0;
    }
}

OptionalFaultRecord
Core::ldl(const MEMInstruction& inst, Address effectiveAddress, LongRegister& destination) {
    if (!aligned(inst.getSrcDest(), TreatAsLongRegister{})) {
        return invalidOperandFault();
        /// @todo perform an unaligned load into registers
    } else {
        destination.setValue<LongOrdinal>(load(effectiveAddress, TreatAsLongOrdinal{}));
        // support unaligned accesses
        return std::nullopt;
    }
    // the instruction is invalid, so we should complete after we are done
}


OptionalFaultRecord
Core::stl(const MEMInstruction& inst, Address effectiveAddress, const LongRegister& source) {
    if (!aligned(inst.getSrcDest(), TreatAsLongRegister{})) {
        return invalidOperandFault();
        /// @todo perform an unaligned load into registers
    } else {
        // support unaligned accesses
        store(effectiveAddress, source.getValue<LongOrdinal>(), TreatAsLongOrdinal{});
        return std::nullopt;
    }
    // the instruction is invalid, so we should complete after we are done
}

OptionalFaultRecord
Core::ldt(const MEMInstruction& inst, Address effectiveAddress, TripleRegister& destination) {
    if (!aligned(inst.getSrcDest(), TreatAsTripleRegister{})) {
        return invalidOperandFault();
        /// @todo perform an unaligned load into registers
    } else {
        destination[0] = load(effectiveAddress + 0, TreatAsOrdinal{});
        destination[1] = load(effectiveAddress + 4, TreatAsOrdinal{});
        destination[2] = load(effectiveAddress + 8, TreatAsOrdinal{});
        return std::nullopt;
    }
    // the instruction is invalid, so we should complete after we are done
}

OptionalFaultRecord
Core::stt(const MEMInstruction& inst, Address effectiveAddress, const TripleRegister& source) {
    if (!aligned(inst.getSrcDest(), TreatAsTripleRegister{})) {
        return invalidOperandFault();
    } else {
        // support unaligned accesses
        store(effectiveAddress + 0,  static_cast<Ordinal>(source[0]), TreatAsOrdinal{});
        store(effectiveAddress + 4,  static_cast<Ordinal>(source[1]), TreatAsOrdinal{});
        store(effectiveAddress + 8,  static_cast<Ordinal>(source[2]), TreatAsOrdinal{});
        return std::nullopt;
    }
    // the instruction is invalid, so we should complete after we are done
}


OptionalFaultRecord
Core::ldq(const MEMInstruction& inst, Address effectiveAddress, QuadRegister& destination) {
    DEBUG_ENTER_FUNCTION;
    if (!aligned(inst.getSrcDest(), TreatAsQuadRegister{})) {
        return invalidOperandFault();
    } else {
        destination.setValue(load(effectiveAddress, TreatAsQuadOrdinal{}), TreatAsQuadOrdinal{});
    }
    DEBUG_LEAVE_FUNCTION;
    // the instruction is invalid, so we should complete after we are done
    return std::nullopt;
}


OptionalFaultRecord
Core::stq(const MEMInstruction& inst, Address effectiveAddress, const QuadRegister& source) {
    DEBUG_ENTER_FUNCTION;
    if (!aligned(inst.getSrcDest(), TreatAsQuadRegister{})) {
        return invalidOperandFault();
    } else {
        store(effectiveAddress, source.getValue(TreatAsQuadOrdinal{}), TreatAsQuadOrdinal{});
    }
    DEBUG_LEAVE_FUNCTION;
    return std::nullopt;
    // the instruction is invalid, so we should complete after we are done
}




void
Core::branch(Integer displacement) noexcept {
    ip_.i += displacement;
    advanceInstruction_ = false;
}

void
Core::branchConditional(bool condition, Integer displacement) noexcept {

    if (condition) {
        DEBUG_LOG_LEVEL(1) {
            std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": branch taken" << std::endl;
        }
        branch(displacement);
    } else {
        DEBUG_LOG_LEVEL(1) {
            std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": branch not taken" << std::endl;
        }
    }
}

/**
 * @brief And the condition code with the consistent mask found in the
 * instruction encoding; returns true if the value returned is not zero
 * @param mask The mask to apply to see if we get a value back
 */

bool
Core::getMaskedConditionCode(uint8_t mask) const noexcept {
    DEBUG_LOG_LEVEL(1) {
        std::cout << "\t\t" << __PRETTY_FUNCTION__
                  << ": mask = 0x" << std::hex << static_cast<int>(mask)
                  << ", ac = 0x" << std::hex << static_cast<int>(ac_.getConditionCode())
                  << ", result = 0x" << std::hex << static_cast<int>(ac_.getConditionCode() & mask)
                  << std::endl;
    }
    return (ac_.getConditionCode() & mask) != 0;
}


bool
Core::conditionCodeEqualsMask(uint8_t mask) const noexcept {
    DEBUG_LOG_LEVEL(1) {
        std::cout << "\t\t" << __PRETTY_FUNCTION__
                  << ": mask = 0x" << std::hex << static_cast<int>(mask)
                  << ", ac = 0x" << std::hex << static_cast<int>(ac_.getConditionCode())
                  << ", result = " << std::boolalpha << (ac_.getConditionCode() == mask)
                  << std::endl;
    }
    return ac_.getConditionCode() == mask;
}

bool
Core::fullConditionCodeCheck() noexcept {
    return fullConditionCodeCheck(instruction_.getInstructionMask());
}

bool
Core::fullConditionCodeCheck(uint8_t mask) noexcept {
    DEBUG_LOG_LEVEL(1) {
        std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": mask = 0x" << std::hex << static_cast<int>(mask) << std::endl;
    }
    // the second condition handles the case where we are looking at unordered
    // output where it is only true if it is equal to zero. So if it turns out
    // that the condition code is zero and the mask is the unordered kind then
    // return true :). In all other cases, the second check will either fail
    // (because the condition code is zero) or it will never fire because the
    // masked condition code will be non-zero.
    return getMaskedConditionCode(mask) || conditionCodeEqualsMask(mask);
}

OptionalFaultRecord
Core::faultGeneric() {
    nextInstruction();
    if (fullConditionCodeCheck()) {
        return constraintRangeFault();
    } else {
        return std::nullopt;
    }
}
Register&
Core::getSrc1Register(const COBRInstruction& inst) noexcept {
    return getGPR(inst.getSrc1());
}
const Register&
Core::getSrc2Register(const COBRInstruction& inst) const noexcept {
    if (inst.getS2()) {
        return getSFR(inst.getSrc2());
    } else {
        return getGPR(inst.getSrc2());
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
    for (int i = 0; i < NumberOfLocalRegisterFrames; ++i) {
        frames_[i].clear();
    }
    nonPortableBegin();
}


void
Core::synld(Register& dest, Ordinal src) {
    DEBUG_ENTER_FUNCTION;
    ac_.clearConditionCode();
    if (auto tempa = maskValue<decltype(src), 0xFFFF'FFFC>(src); tempa == 0xFF00'0004) {
        // interrupt control register needs to be read through this
        ac_.setConditionResult(true);
        // copy the contents of the interrupt control register to a target
        // register
        dest.setValue(ictl_.o, TreatAsOrdinal{});
    } else {
        ac_.setConditionResult(true);
        dest.setValue(load(tempa, TreatAsOrdinal{}), TreatAsOrdinal{});
    }
    DEBUG_LEAVE_FUNCTION;
}

void
Core::synmov(const Register& dest, Ordinal src) {
    DEBUG_ENTER_FUNCTION;
    ac_.clearConditionCode();
    auto value = load(src, TreatAsOrdinal{});
    DEBUG_LOG_LEVEL(1) {
        std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": loaded value @ 0x" << std::hex << src
                  << " = 0x" << std::hex << value
                  << std::endl;
    }
    if (auto tempa = maskValue<Ordinal, 0xFFFF'FFFC>(dest.getValue(TreatAsOrdinal{})); tempa == 0xFF00'0004) {
        DEBUG_LOG_LEVEL(1) {
            std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": ictl before 0x" << std::hex << ictl_.getValue<Ordinal>() << std::endl;
        }
        ictl_.setValue<Ordinal>(load(src, TreatAsOrdinal{}));
        ac_.setConditionResult(true);
        DEBUG_LOG_LEVEL(1) {
            std::cout << "\t\t" << __PRETTY_FUNCTION__ << ": ictl after 0x" << std::hex << ictl_.getValue<Ordinal>() << std::endl;
        }
    } else {
        store(tempa, value, TreatAsOrdinal{});
        // wait for completion
        ac_.setConditionResult(true);
    }
    DEBUG_LEAVE_FUNCTION;
}

void
Core::synmovl(const Register& dest, Ordinal src) {
    ac_.clearConditionCode();
    auto tempa = maskValue<Ordinal, 0xFFFF'FFF8>(dest.getValue(TreatAsOrdinal{}));
    auto temp = load(src, TreatAsLongOrdinal{});
    // observing execution shows that there is a delay here on the Sx processor
    // not sure what it is doing
    store(tempa, temp, TreatAsLongOrdinal{});
    // wait for completion
    ac_.setConditionResult(true);
}

void
Core::synmovq(const Register& dest, Ordinal src) {
    DEBUG_ENTER_FUNCTION;
    ac_.clearConditionCode();
    auto temp = load(src, TreatAsQuadOrdinal{});
    if (auto tempa = maskValue<Ordinal, 0xFFFF'FFF0>(dest.getValue(TreatAsOrdinal{})); tempa == 0xFF00'0010) {
        sendIAC(iac::Message{temp});
        ac_.setConditionResult(true);
    } else {
        store(tempa, temp, TreatAsQuadOrdinal{});
        // wait for completion
        ac_.setConditionResult(true);
    }
    DEBUG_LEAVE_FUNCTION;
}


void
Core::performSelect(Register& dest, Ordinal src1, Ordinal src2) {
    dest.setValue(fullConditionCodeCheck() ? src2 : src1, TreatAsOrdinal{});
}

void
Core::performConditionalSubtract(Register& dest, Integer src1, Integer src2, TreatAsInteger) {
    if (fullConditionCodeCheck()) {
        subi(dest, src1, src2);
    }
}


void
Core::performConditionalSubtract(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) {
    if (fullConditionCodeCheck()) {
        subo(dest, src1, src2);
    }
}


void
Core::performConditionalAdd(Register& dest, Integer src1, Integer src2, TreatAsInteger) {
    if (fullConditionCodeCheck()) {
        addi(dest, src1, src2);
    }
}


void
Core::performConditionalAdd(Register& dest, Ordinal src1, Ordinal src2, TreatAsOrdinal) {
    if (fullConditionCodeCheck()) {
        addo(dest, src1, src2);
    }
}
void
Core::boot0(Address sat, Address pcb, Address startIP) noexcept {
    DEBUG_ENTER_FUNCTION;
    systemAddressTableBase_ = sat;
    prcbAddress_ = pcb;
    setIP(startIP);
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
        

        ac_.clearConditionCode();
        Register temp_{0xFFFF'FFFF};
        for (int i = 0; i < 8; ++i) {
            addc(temp_, temp_.getValue<Ordinal>(), x[i]);
            DEBUG_LOG_LEVEL(3) {
                std::cout << "checksum stage " << std::dec << i << ": 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
            }
        }
        DEBUG_LOG_LEVEL(2) {
            std::cout << "final checksum: 0x" << std::hex << temp_.getValue<Ordinal>() << std::endl;
        }
        if (temp_.getValue(TreatAsOrdinal{}) != 0) {
            assertFailureState();
            DEBUG_LEAVE_FUNCTION;
            return BootResult::ChecksumFail;
        } else {
            DEBUG_LOG_LEVEL(2) {
                std::cout << "sat: 0x" << std::hex << x[0] << std::endl;
                std::cout << "prcb: 0x" << std::hex << x[1] << std::endl;
                std::cout << "ip: 0x" << std::hex << x[3] << std::endl;
            }
            boot0(x[0], x[1], x[3]);
            DEBUG_LEAVE_FUNCTION;
            return BootResult::Success;
        }
    }
}


void
Core::addi(Register& dest, Integer src1, Integer src2) {
    add<Integer>(dest, src1, src2, TreatAsInteger{});
    nextInstruction();
    faultOnOverflow(dest);
}


void
Core::addo(Register& dest, Ordinal src1, Ordinal src2) {
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
Core::setIP(Ordinal value) noexcept {
    // when we set the ip during this instruction, we need to turn off the
    // automatic advancement!
    ip_.setValue<Ordinal>(value);
    advanceInstruction_ = false;
}

OptionalFaultRecord
Core::subi(Register& dest, Integer src1, Integer src2) {
    sub<Integer>(dest, src1, src2, TreatAsInteger{});
    nextInstruction();
    return faultOnOverflow(dest);
}


void
Core::subo(Register& dest, Ordinal src1, Ordinal src2) {
    sub<Ordinal>(dest, src1, src2, TreatAsOrdinal{});
}


OptionalFaultRecord
Core::modpc(Register& regDest, Ordinal src1o, Ordinal src2o) {
    if (auto mask = src1o; mask != 0) {
        if (!pc_.inSupervisorMode()) {
            return typeMismatchFault();
        } else {
            regDest.setValue<Ordinal>(pc_.modify(mask, src2o));
            if (regDest.getPriority() > pc_.getPriority()) {
                checkForPendingInterrupts();
            }
        }
    } else {
        regDest.setValue<Ordinal>(pc_.getValue<Ordinal>());
    }
    return std::nullopt;
}

void
Core::modxc(Register& control, Register& dest, Ordinal src1, Ordinal src2) {
    dest.setValue<Ordinal>(control.modify(src1, src2));
}

void
Core::shlo(Register& srcDest, Ordinal src1, Ordinal src2) {
    srcDest.setValue<Ordinal>(src1 < 32 ? src2 << src1 : 0);
}

void
Core::shli(Register& srcDest, Integer src1, Integer src2) {
    srcDest.setValue<Integer>(src2 << src1);
}

void
Core::rotate(Register& dest, Ordinal src1, Ordinal src2) {
    dest.setValue<Ordinal>(rotateOperation(src2, src1));
}

void
Core::shri(Register& dest, Integer src1, Integer src2) {
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
Core::syncf() {
    // Wait for all faults to be generated that are associated with any prior
    // uncompleted instructions
}




void
Core::shro(Register &dest, Ordinal src1, Ordinal src2) {
    dest.setValue<Ordinal>((static_cast<Ordinal>(src1) < 32) ? static_cast<Ordinal>(src2) >> static_cast<Ordinal>(src1) : 0);
}

OptionalFaultRecord
Core::stib(Integer value, Address address) {
    if (auto maskedValue = address & 0xFFFF'FF00; maskedValue != 0 && maskedValue != 0xFFFF'FF00) {
        store(address, static_cast<ByteInteger>(value), TreatAsByteInteger{});
        if (ac_.arith.integerOverflowMask == 1) {
            ac_.arith.integerOverflowFlag = 1;
        } else {
            return integerOverflowFault();
        }
    } else {
        store(address, static_cast<ByteInteger>(value), TreatAsByteInteger{});
    }
    return std::nullopt;
}

OptionalFaultRecord
Core::stis(Integer value, Address address) {
    // If the register data is too large to be stored as a byte or
    // short word, the value is truncated and the integer overflow
    // condition is signalled.
    if (auto maskedValue = address & 0xFFFF'0000; maskedValue != 0 && maskedValue != 0xFFFF'0000) {
        store(address, static_cast<ShortInteger>(value), TreatAsShortInteger{});
        if (ac_.arith.integerOverflowMask == 1) {
            ac_.arith.integerOverflowFlag = 1;
        } else {
            return integerOverflowFault();
        }
    } else {
        store(address, static_cast<ShortInteger>(value), TreatAsShortInteger{});
    }
    return std::nullopt;
}

void
Core::ldib(Address address, Register &dest) {
    dest.setValue<Integer>(load(address, TreatAsByteInteger{}));
}

void
Core::ldis(Address address, Register &dest) {
    dest.setValue<Integer>(load(address, TreatAsShortInteger{}));
}

OptionalFaultRecord
Core::modi(Register& dest, Integer src1, Integer src2) {
    if (auto denominator = src1; denominator == 0) {
        // dest becomes an undefined value
        dest.setValue<Ordinal>(0);
        return zeroDivideFault();
    } else {
        auto numerator = src2;
        auto result = numerator - ((numerator / denominator) * denominator);
        if (((numerator * denominator) < 0) && (result != 0)) {
            result += denominator;
        }
        dest.setValue<Integer>(result);
        nextInstruction();
        /// @todo implement fault checks
    }
    return std::nullopt;
}
void
Core::advanceCOBRDisplacement(Integer displacement) noexcept {
    ip_.i += displacement;
    advanceInstruction_ = false;
}
void
Core::atadd(Register& dest, Ordinal src1, Ordinal src2) {
    syncf();
    lockBus();
    auto addr = maskValue<decltype(src1), 0xFFFF'FFFC>(src1) ;
    auto temp = load<Ordinal>(addr);
    // adds the src (src2 internally) value to the value in memory location specified with the addr (src1 in this case) operand.
    // The initial value from memory is stored in dst (internally src/dst).
    Ordinal result = temp + src2;
    store<Ordinal>(addr, result);
    dest.setValue<Ordinal>(temp);
    unlockBus();
}
void
Core::atmod(Register& dest, Ordinal src1, Ordinal src2) {
    syncf();
    lockBus();
    auto addr = maskValue<decltype(src1), 0xFFFF'FFFC>(src1) ;
    auto temp = load<Ordinal>(addr);
    // copies the src/dest value (logical version) into the memory location specifeid by src1.
    // The bits set in the mask (src2) operand select the bits to be modified in memory. The initial
    // value from memory is stored in src/dest
    Ordinal result = ::modify(src2, dest.getValue<Ordinal>(), temp);
    store<Ordinal>(addr, result);
    dest.setValue<Ordinal>(temp);
    unlockBus();
}
void
Core::cmpo(Ordinal src1, Ordinal src2) noexcept {
    cmpGeneric(src1, src2);
}
void
Core::cmpinco(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    cmpGeneric(src1, src2);
    dest.setValue<Ordinal>(src2 + 1);
}
void
Core::cmpdeco(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    cmpGeneric(src1, src2);
    dest.setValue<Ordinal>(src2 - 1);
}
void
Core::cmpi(Integer src1, Integer src2) noexcept {
    cmpGeneric(src1, src2);
}
void
Core::cmpinci(Register& dest, Integer src1, Integer src2) noexcept {
    cmpGeneric(src1, src2);
    dest.setValue<Integer>(src2 + 1);
}
void
Core::cmpdeci(Register& dest, Integer src1, Integer src2) noexcept {
    cmpGeneric(src1, src2);
    dest.setValue<Integer>(src2 - 1);
}
void
Core::bbc(Ordinal bitpos, Ordinal against, int16_t displacement) {
    if ((against & bitpos) == 0) {
        ac_.setConditionCode(0b010);
        advanceCOBRDisplacement(displacement);
    } else {
        ac_.setConditionCode(0b000);
    }
}
void
Core::bbs(Ordinal bitpos, Ordinal against, int16_t displacement) {
    if ((against & bitpos) != 0) {
        ac_.setConditionCode(0b010);
        advanceCOBRDisplacement(displacement);
    } else {
        ac_.setConditionCode(0b000);
    }
}
void
Core::bbc() {
    if (_cobrInstruction.getM1()) {
        bbc(_cobrInstruction.getSrc1(), getSrc2Register(_cobrInstruction), static_cast<ShortInteger>(_cobrInstruction.getDisplacement()));
    } else {
        bbc(getSrc1Register(_cobrInstruction), getSrc2Register(_cobrInstruction), static_cast<ShortInteger>(_cobrInstruction.getDisplacement()));
    }
}
void
Core::bbc(uint8_t bitpos, const Register& against, int16_t displacement) {
    bbc(computeBitPosition(bitpos), static_cast<Ordinal>(against), displacement);
}
void
Core::bbc(const Register& bitpos, const Register& against, int16_t displacement) {
    bbc(bitpos.asBitPosition(), static_cast<Ordinal>(against), displacement);
}
void
Core::bbs() {
    if (_cobrInstruction.getM1()) {
        bbs(_cobrInstruction.getSrc1(), getSrc2Register(_cobrInstruction), static_cast<ShortInteger>(_cobrInstruction.getDisplacement()));
    } else {
        bbs(getSrc1Register(_cobrInstruction), getSrc2Register(_cobrInstruction), static_cast<ShortInteger>(_cobrInstruction.getDisplacement()));
    }
}
void
Core::bbs(uint8_t bitpos, const Register& against, int16_t displacement) {
    bbs(computeBitPosition(bitpos), static_cast<Ordinal>(against), displacement);
}
void
Core::bbs(const Register& bitpos, const Register& against, int16_t displacement) {
    bbs(bitpos.asBitPosition(), static_cast<Ordinal>(against), displacement);
}
void
Core::scanbit(Register& dest, Ordinal src1) noexcept {
    dest.setValue<Ordinal>(0xFFFF'FFFF);
    ac_.setConditionCode(0b000);
    for (int i = 31; i >= 0; --i) {
        if ((src1 & computeBitPosition(i)) != 0) {
            dest.setValue<Ordinal>(i);
            ac_.setConditionCode(0b010);
        }
    }
}
void
Core::spanbit(Register& dest, Ordinal src1) noexcept {
    dest.setValue<Ordinal>(0xFFFF'FFFF);
    ac_.setConditionCode(0b000);
    for (int i = 31; i >= 0; --i) {
        if ((src1 & computeBitPosition(i)) == 0) {
            dest.setValue<Ordinal>(i);
            ac_.setConditionCode(0b010);
        }
    }
}

void
Core::notOperation(Register& dest, Ordinal src) noexcept {
    dest.setValue<Ordinal>(~src);
}

void
Core::nor(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    microcodedBitwiseOperation<NorOperation>(dest, src1, src2);
}
void
Core::nand(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    microcodedBitwiseOperation<NandOperation>(dest, src1, src2);
}
void
Core::xnor(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    microcodedBitwiseOperation<XnorOperation>(dest, src1, src2);
}
void
Core::ornot(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    microcodedBitwiseOperation<GenericOrOperation<
        BitwiseMicrocodeArgumentFlags::Passthrough,
        BitwiseMicrocodeArgumentFlags::Invert,
        BitwiseMicrocodeArgumentFlags::Passthrough>>(dest, src1, src2);
}
void
Core::notor(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    microcodedBitwiseOperation<GenericOrOperation<
        BitwiseMicrocodeArgumentFlags::Passthrough,
        BitwiseMicrocodeArgumentFlags::Passthrough,
        BitwiseMicrocodeArgumentFlags::Invert>>(dest, src1, src2);
}

void
Core::andnot(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    microcodedBitwiseOperation<GenericAndOperation<
        BitwiseMicrocodeArgumentFlags::Passthrough,
        BitwiseMicrocodeArgumentFlags::Invert,
        BitwiseMicrocodeArgumentFlags::Passthrough>>(dest, src1, src2);
}
void
Core::notand(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    microcodedBitwiseOperation<GenericAndOperation<
        BitwiseMicrocodeArgumentFlags::Passthrough,
        BitwiseMicrocodeArgumentFlags::Passthrough,
        BitwiseMicrocodeArgumentFlags::Invert>>(dest, src1, src2);
}
void
Core::notbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    // notbit is src2 ^ computeBitPosition(src1)
    microcodedBitwiseOperation<GenericXorOperation<
        BitwiseMicrocodeArgumentFlags::Passthrough,
        BitwiseMicrocodeArgumentFlags::BitPosition,
        BitwiseMicrocodeArgumentFlags::Passthrough>>(dest, src1, src2);
}
void
Core::setbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    // setbit is src2 | computeBitPosition(src1o)
    microcodedBitwiseOperation<GenericOrOperation<
        BitwiseMicrocodeArgumentFlags::Passthrough,
        BitwiseMicrocodeArgumentFlags::BitPosition,
        BitwiseMicrocodeArgumentFlags::Passthrough>>(dest, src1, src2);
}
void
Core::clrbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    // clrbit is src2 & ~computeBitPosition(src1)
    // so lets use andnot to reduce duplication effort
    microcodedBitwiseOperation<GenericAndOperation<
        BitwiseMicrocodeArgumentFlags::Passthrough,
        BitwiseMicrocodeArgumentFlags::BitPositionThenInvert,
        BitwiseMicrocodeArgumentFlags::Passthrough>>(dest, src1, src2);
}
void
Core::alterbit(Register& dest, Ordinal src1, Ordinal src2) noexcept {
    if (ac_.getConditionCode() & 0b010) {
        setbit(dest, src1, src2);
    } else {
        clrbit(dest, src1, src2);
    }
}
void
Core::addc(Register& dest, Ordinal src1, Ordinal src2) {
    auto carry = ac_.getCarryBit() ? 1 : 0;
    LongOrdinal result = static_cast<LongOrdinal>(src2) + static_cast<LongOrdinal>(src1) + carry;
    //result += (ac_.getCarryBit() ? 1 : 0);
    DEBUG_LOG_LEVEL(3) {
        std::cout << "\t(addc->addo 0x" << std::hex << src2 << " 0x" << std::hex << src1 << " 0x" << std::hex << carry << ") => 0x" << std::hex << result << std::endl;
    }
    dest.setValue<Ordinal>(result);
    arithmeticWithCarryGeneric(static_cast<Ordinal>(result >> 32),
                               mostSignificantBit(src2),
                               mostSignificantBit(src1),
                               mostSignificantBit(dest.getValue<Ordinal>()));
}
void
Core::subc(Register& dest, Ordinal src1, Ordinal src2) {
    LongOrdinal result = static_cast<LongOrdinal>(src2) - static_cast<LongOrdinal>(src1) - 1;
    result += (ac_.getCarryBit() ? 1 : 0);
    dest.setValue<Ordinal>(result);
    arithmeticWithCarryGeneric(static_cast<Ordinal>(result >> 32),
                               mostSignificantBit(src2),
                               mostSignificantBit(src1),
                               mostSignificantBit(dest.getValue<Ordinal>()));
}
OptionalFaultRecord
Core::remi(Register& dest, Integer src1, Integer src2) {
    auto result = remainderOperation<Integer>(dest, src1, src2);
    if (result) {
        return result;
    } else {
        nextInstruction();
        return faultOnOverflow(dest);
    }
}
OptionalFaultRecord
Core::remo(Register& dest, Ordinal src1, Ordinal src2) {
    return remainderOperation<Ordinal>(dest, src1, src2);
}
OptionalFaultRecord
Core::divi(Register& dest, Integer src1, Integer src2) {
    auto result = divideOperation<Integer>(dest, src1, src2);
    if (result) {
        return result;
    } else {
        nextInstruction();
        return faultOnOverflow(dest);
    }
}
OptionalFaultRecord
Core::divo(Register& dest, Ordinal src1, Ordinal src2) {
    return divideOperation<Ordinal>(dest, src1, src2);
}
void
Core::concmpo(Ordinal src1, Ordinal src2) noexcept {
    concmpGeneric<Ordinal>(src1, src2);
}
void
Core::concmpi(Integer src1, Integer src2) noexcept {
    concmpGeneric<Integer>(src1, src2);
}
void
Core::cmpobGeneric(uint8_t mask, Ordinal src1, Ordinal src2, int16_t displacement) noexcept {
    cmpxbGeneric(mask, src1, src2, displacement, TreatAsOrdinal{});
}
void
Core::cmpibGeneric(uint8_t mask, Integer src1, Integer src2, int16_t displacement) noexcept {
    cmpxbGeneric(mask, src1, src2, displacement, TreatAsInteger{});
}
template<uint8_t mask>
constexpr bool aligned(uint8_t value) noexcept {
    return (value & mask) == 0;
}

OptionalFaultRecord
Core::movl(const REGInstruction& inst) {
    if ((!aligned<0b1>(inst.getSrcDest())) || (!aligned<0b1>(inst.getSrc1()))) {
        // don't corrupt anything as we can't recover if we do!
        nextInstruction();
        return invalidOpcodeFault();
    } else {
        getGPR(inst.getSrcDest(), TreatAsLongRegister{}).setValue(
                inst.src1IsLiteral() ? inst.getSrc1() : getGPR(inst.getSrc1(), TreatAsLongRegister{}).getValue(TreatAsLongOrdinal{}),
        TreatAsLongOrdinal {});
    }
    return std::nullopt;
}

OptionalFaultRecord
Core::movt(const REGInstruction& inst) {
    if ((!aligned<0b11>(inst.getSrcDest())) || (!aligned<0b11>(inst.getSrc1()))) {
        // don't corrupt anything as we can't recover if we do!
        nextInstruction();
        return invalidOpcodeFault();
    } else if (inst.src1IsLiteral()) {
        setGPR(inst.getSrcDest(), inst.getSrc1(), TreatAsOrdinal{});
        setGPR(inst.getSrcDest(), 1, 0, TreatAsOrdinal{});
        setGPR(inst.getSrcDest(), 2, 0, TreatAsOrdinal{});
    } else {
        moveGPR(inst.getSrcDest(), inst.getSrc1(), TreatAsOrdinal{});
        moveGPR(inst.getSrcDest(), 1, inst.getSrc1(), 1, TreatAsOrdinal{});
        moveGPR(inst.getSrcDest(), 2, inst.getSrc1(), 2, TreatAsOrdinal{});
    }
    return std::nullopt;
}

OptionalFaultRecord
Core::movq(const REGInstruction& inst) {
    if ((!aligned<0b11>(inst.getSrcDest())) || (!aligned<0b11>(inst.getSrc1()))) {
        // don't corrupt anything as we can't recover if we do!
        nextInstruction();
        return invalidOpcodeFault();
    } else if (inst.src1IsLiteral()) {
        setGPR(inst.getSrcDest(), inst.getSrc1(), TreatAsOrdinal{});
        setGPR(inst.getSrcDest(), 1, 0, TreatAsOrdinal{});
        setGPR(inst.getSrcDest(), 2, 0, TreatAsOrdinal{});
        setGPR(inst.getSrcDest(), 3, 0, TreatAsOrdinal{});
    } else {
        moveGPR(inst.getSrcDest(), inst.getSrc1(), TreatAsOrdinal{});
        moveGPR(inst.getSrcDest(), 1, inst.getSrc1(), 1, TreatAsOrdinal{});
        moveGPR(inst.getSrcDest(), 2, inst.getSrc1(), 2, TreatAsOrdinal{});
        moveGPR(inst.getSrcDest(), 3, inst.getSrc1(), 3, TreatAsOrdinal{});
    }
    return std::nullopt;
}
