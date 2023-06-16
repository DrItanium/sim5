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
Address
Core::getInterruptTableBaseAddress() const {
    return getInterruptTablePointer();
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
        return load<Ordinal>(getInterruptTableBaseAddress() + 36 + byteOffset);
    }
}

void
Core::receiveInterrupt(InterruptVector vector) {
    if (valid(vector)) {
        if (canDispatchVector(vector, pc_.getPriority())) {
            serviceInterrupt(vector);
        } else {
            postInterrupt(vector);
        }
    }
}

bool
Core::getPendingPriorityBit(uint8_t priority) const {
    return (getInterruptPendingPriorities() & computeBitPosition(priority)) != 0;
}

void
Core::setPendingPriorityBit(uint8_t priority) {
    /// @todo make sure this is an atomic operation
    auto pp = getInterruptPendingPriorities();
    pp |= computeBitPosition(priority);
    setInterruptPendingPriorities(pp);
}

void
Core::clearPendingPriorityBit(uint8_t priority) {
    /// @todo make sure this is an atomic operation
    auto pp = getInterruptPendingPriorities();
    pp &= (~(computeBitPosition(priority)));
    setInterruptPendingPriorities(pp);
}


bool
Core::getPendingInterruptBit(InterruptVector vector) const {
    // need to retrieve the ordinal which contains our corresponding pending bits
    return (getPendingInterruptWord(vector) & computeBitPosition(computeInterruptVectorBitOffset(vector)));
}

void
Core::clearPendingInterruptBit(InterruptVector vector) {
    /// @todo make sure this is an atomic operation
    auto pi = getPendingInterruptWord(vector);
    pi &= (~(computeBitPosition(computeInterruptVectorBitOffset(vector))));
    setPendingInterruptWord(vector, pi);
}

void
Core::setPendingInterruptBit(InterruptVector vector) {
    /// @todo make sure this is an atomic operation
    auto pi = getPendingInterruptWord(vector);
    pi |= computeBitPosition(computeInterruptVectorBitOffset(vector));
    setPendingInterruptWord(vector, pi);
}

void
Core::postInterrupt(InterruptVector vector) {
    // According to the book the algorithm is as follows:
    // temp = atomic_read(pending_priorities);
    // temp1 = memory(pending_interrupts(i/8));
    // temp[i/8] = 1;
    // temp1[i mod 8] = 1;
    // memory(pending_interrupts(i/8)) = temp1;
    // atomic_write(pending_priorities) = temp;
    // -- Pg 53 of The 80960 Microprocessor Architecture by Glenford J. Myers & David L. Budde
    /// @todo make sure that these assignments are done indivisibly
    setPendingPriorityBit(vector);
    setPendingInterruptBit(vector);
}

// an interrupt vector i is considered to be pending when bit i is set in
// pending interrupts and bit i/32 is set in pending priorities
//
// -- Pg 53 of The 80960 Microprocessor Architecture by Glenford J. Myers & David L. Budde

void
Core::obtainedPendingVector(InterruptVector vector) {
    clearPendingInterruptBit(vector);
    if (pendingInterruptPriorityClear(vector)) {
        clearPendingPriorityBit(vector);
    }
}

bool
Core::pendingInterruptPriorityClear(InterruptVector vector) const {
    return getPendingInterruptBitsForPriority(computeInterruptPriority(vector)) == 0;
}
namespace {
    constexpr ByteOrdinal getCorrespondingByte(Ordinal value, uint8_t target) noexcept {
        switch (target & 0b11) {
            case 0b00:
                return static_cast<ByteOrdinal>(value);
            case 0b01:
                return static_cast<ByteOrdinal>(value >> 8);
            case 0b10:
                return static_cast<ByteOrdinal>(value >> 16);
            case 0b11:
                return static_cast<ByteOrdinal>(value >> 24);
        }
        return 0xFF; // should never get here!
    }
    static_assert(getCorrespondingByte(0xABCDEF01, 0x00) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x01) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x02) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x03) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x04) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x05) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x06) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x07) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x08) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x09) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x0a) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x0b) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x0c) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x0d) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x0e) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x0f) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x10) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x11) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x12) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x13) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x14) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x15) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x16) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x17) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x18) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x19) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x1a) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x1b) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x1c) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x1d) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x1e) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x1f) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x20) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x21) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x22) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x23) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x24) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x25) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x26) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x27) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x28) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x29) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x2a) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x2b) == 0xAB);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x2c) == 0x01);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x2d) == 0xEF);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x2e) == 0xCD);
    static_assert(getCorrespondingByte(0xABCDEF01, 0x2f) == 0xAB);
    static_assert(highestOne(getCorrespondingByte(0xABCDEF01, 0)) == 0);
    static_assert(highestOne(getCorrespondingByte(0xABCDEF01, 1)) == 7);
    static_assert(highestOne(getCorrespondingByte(0xABCDEF01, 2)) == 7);
    static_assert(highestOne(getCorrespondingByte(0xABCDEF01, 3)) == 7);
}
ByteOrdinal
Core::getHighestPostedInterruptVectorForPriority(uint8_t priority) const {
    return highestOne(getPendingInterruptBitsForPriority(priority));
}

InterruptVector
Core::highestPostedInterruptVector() const {
    // find the highest posted priority
    auto pendingPriorities = getInterruptPendingPriorities();
    auto targetPriority = highestOne(pendingPriorities);
    auto vectorOffset = getHighestPostedInterruptVectorForPriority(targetPriority);
    return static_cast<InterruptVector>((targetPriority << 3) | vectorOffset);
}
ByteOrdinal
Core::getPendingInterruptBitsForPriority(uint8_t priority) const {
    return getCorrespondingByte(getPendingInterruptWord(priority >> 2), priority);
}

InterruptVector
Core::serviceNextInterrupt() {
    if (auto nextInterrupt = highestPostedInterruptVector(); valid(nextInterrupt) && canDispatchVector(nextInterrupt, pc_.getPriority())) {
        // okay, it is a valid interrupt and system priority allows it so obtain it and then return it
        obtainedPendingVector(nextInterrupt);
        return nextInterrupt;
    }
    return static_cast<InterruptVector>(0);
}

void
Core::checkForPendingInterrupts() {
    // okay so we are checking for pending interrupts, we need to keep servicing valid interrupts until we are done
    if (auto vector = serviceNextInterrupt(); valid(vector)) {
        serviceInterrupt(vector);
    }
    // So this gets a little strange, I think servicing the interrupt will just
}

void
Core::testPendingInterrupts() {
    checkForPendingInterrupts();
}

void
Core::serviceInterrupt(InterruptVector vector) {
    /// @todo implement
}
void
Core::dispatchInterrupt(uint8_t vector) noexcept {
    receiveInterrupt(static_cast<InterruptVector>(vector));
}

void
Core::faultOnOverflow(Register& dest) {
    /// @todo implement
}
