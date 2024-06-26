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

const Register&
Core::getSrc1Register(const REGInstruction& inst) const noexcept {
    if (auto src1 = inst.getSrc1(); inst.getM1()) {
        /// @todo what to do if s1 is also set?
        return constants_.get(src1);
    } else {
        return getGPR(src1);
    }
}

const Register&
Core::getSrc2Register(const REGInstruction& inst) const noexcept {
    if (auto src2 = inst.getSrc2(); inst.getM2()) {
        /// @todo what to do if s2 is also set?
        return constants_.get(src2);
    } else {
        return getGPR(src2);
    }
}


Ordinal
Core::unpackSrc1(const REGInstruction& inst, ByteOrdinal offset, TreatAsOrdinal) noexcept {
    if (auto src1 = inst.getSrc1(); inst.getM1()) {
        // literals should always return zero if offset is greater than zero
        return offset == 0 ? src1 : 0;
    } else {
        return getGPRValue(src1, offset, TreatAsOrdinal{});
    }
}

Ordinal
Register::modify(Ordinal mask, Ordinal src) noexcept {
    auto tmp = o;
    o = ::modify(mask, src, o);
    return tmp;
}

Core::LocalRegisterSet&
Core::getNextPack() noexcept {
    if constexpr (NumberOfLocalRegisterFrames > 1) {
        // do these as separate operations, otherwise gcc generates garbage
        uint8_t result = (localRegisterFrameIndex_ + 1);
        result %= NumberOfLocalRegisterFrames;
        return frames_[result];
    } else {
        return frames_[0];
    }
}

Core::LocalRegisterSet&
Core::getPreviousPack() noexcept {
    if constexpr (NumberOfLocalRegisterFrames > 1) {
        // do these as separate operations, otherwise gcc generates garbage
        uint8_t result = (localRegisterFrameIndex_ - 1);
        result %= NumberOfLocalRegisterFrames;
        DEBUG_LOG_LEVEL(2) {
            std::cout << __PRETTY_FUNCTION__ << ": 0x" << std::hex << static_cast<int>(result) << std::endl;
        }
        return frames_[result];
    } else {
        return frames_[0];
    }
}
void
Core::setGPR(ByteOrdinal index, Ordinal value, TreatAsOrdinal) noexcept {
    getGPR(index).setValue(value, TreatAsOrdinal{});
}
void
Core::setGPR(ByteOrdinal index, ByteOrdinal offset, Ordinal value, TreatAsOrdinal) noexcept {
    getGPR(index, offset).setValue(value, TreatAsOrdinal{});
}
void
Core::setGPR(ByteOrdinal index, Integer value, TreatAsInteger) noexcept {
    getGPR(index).setValue(value, TreatAsInteger{});
}
Register&
Core::getGPR(ByteOrdinal index) noexcept {
    return getGPR(index, TreatAsRegister{});
}
const Register&
Core::getGPR(ByteOrdinal index) const noexcept {
    return getGPR(index, TreatAsRegister{});
}
void
Core::LocalRegisterSet::relinquishOwnership() noexcept {
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": {" << std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": &this: 0x" << std::hex << reinterpret_cast<uintptr_t>(this) << std::endl;
    }
    _valid = false;
    synchronizeOwnership(0);
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": }" << std::endl;
    }
    /// @todo zero out the frame?
}
void
Core::LocalRegisterSet::relinquishOwnership(SaveRegistersFunction saveRegisters) {
    if (_valid) {
        DEBUG_LOG_LEVEL(2) {
            std::cout << __PRETTY_FUNCTION__ << ": saving to 0x" << std::hex << _targetFramePointer << std::endl;
        }
        saveRegisters(_theFrame, _targetFramePointer);
    }
    relinquishOwnership();
}
void
Core::LocalRegisterSet::takeOwnership(Address newFP, SaveRegistersFunction saveRegisters) {
    DEBUG_ENTER_FUNCTION;
    if (valid()) {
        DEBUG_LOG_LEVEL(2) {
            std::cout << __PRETTY_FUNCTION__ << ": saving to 0x" << std::hex << _targetFramePointer << std::endl;
        }
        saveRegisters(_theFrame, _targetFramePointer);
    }
    _valid = true;
    synchronizeOwnership(newFP);
    // don't clear out the registers
    DEBUG_LEAVE_FUNCTION;
}
void
Core::LocalRegisterSet::restoreOwnership(Address newFP,
                 SaveRegistersFunction saveRegisters,
                 RestoreRegistersFunction restoreRegisters) {
    DEBUG_ENTER_FUNCTION;
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": &this: 0x" << std::hex << reinterpret_cast<uintptr_t>(this) << std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": newFramePointer is 0x" << std::hex << newFP << std::endl;
    }
    if (valid()) {
        DEBUG_LOG_LEVEL(2) {
            std::cout << __PRETTY_FUNCTION__ << ": targetFramePointer currently 0x" << std::hex << _targetFramePointer << std::endl;
            std::cout << __PRETTY_FUNCTION__ << ": Current frame is valid" << std::endl;
        }
        // okay we have something valid in there right now, so we need to determine if it is valid or not
        if (newFP == _targetFramePointer) {
            DEBUG_LOG_LEVEL(2) {
                std::cout << __PRETTY_FUNCTION__ << ": match found!" << std::endl;
            }
            // okay so we got a match, great!
            // just leave early
            DEBUG_LEAVE_FUNCTION;
            return;
        }
        DEBUG_LOG_LEVEL(2) {
            std::cout << __PRETTY_FUNCTION__ << ": no match found!" << std::endl;
            std::cout << __PRETTY_FUNCTION__ << ": saving to 0x" << std::hex << getAddress() << std::endl;
        }
        // got a mismatch, so spill this frame to memory first
        saveRegisters(getUnderlyingFrame(), getAddress());
    }
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": Setting up frame!" << std::endl;
    }
    _valid = true;
    synchronizeOwnership(newFP);
    restoreRegisters(getUnderlyingFrame(), getAddress());
    DEBUG_LEAVE_FUNCTION;
}
void
Core::LocalRegisterSet::clear() noexcept {
    _theFrame.clear();
}
/**
 * Makes sure that this set has an up to date frame pointer
 * @param fp The frame pointer to update the targetFramePointer field to
 */
void
Core::LocalRegisterSet::synchronizeOwnership(Ordinal fp) noexcept {
    DEBUG_LOG_LEVEL(2) {
        std::cout << __PRETTY_FUNCTION__ << ": 0x" << std::hex << _targetFramePointer << " -> 0x" << fp << std::endl;
    }
    _targetFramePointer = fp;
}

LongOrdinal
Core::unpackSrc1(const REGInstruction& inst, TreatAsLongOrdinal) const {
    return getGPR(inst.getSrc1(), TreatAsLongRegister{}).getValue<LongOrdinal>();
}

LongInteger
Core::unpackSrc1(const REGInstruction& inst, TreatAsLongInteger) const {
    return getGPR(inst.getSrc1(), TreatAsLongRegister{}).getValue<LongInteger>();
}

Integer
Core::unpackSrc1(const REGInstruction& inst, TreatAsInteger) const {
    return static_cast<Integer>(getGPR(inst.getSrc1(), TreatAsRegister{}));
}
