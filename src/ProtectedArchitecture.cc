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
#include <cstring>

void
Core::saveprcs() noexcept {
    unimplementedFault();
}

void
Core::cmpstr(Ordinal src1Address, Ordinal src2Address, Ordinal len) noexcept {
    ac_.arith.conditionCode = 0b010;
    /// @todo figure out how to accelerate this to use strncmp instead of doing this manually
    for (Ordinal i = 0; i < len; ++i) {
        auto a = load(src1Address + i, TreatAsByteOrdinal {});
        auto b = load(src2Address + i, TreatAsByteOrdinal {});
        if (a > b) {
            ac_.arith.conditionCode = 0b001;
            return;
        } else if (a < b) {
            ac_.arith.conditionCode = 0b100;
            return;
        }
    }
}

void
Core::movstr(Ordinal destAddress, Ordinal srcAddress, Ordinal len) noexcept {
    // copy in reverse to make sure that no byte of the source string is overwritten before it is copied into the
    // destination string. If it is guaranteed that there are no overlaps, the movqstr instruction performs this operation
    // faster.
    /// @todo figure out how to accelerate this to use strncpy instead of doing this manually
    if (srcAddress <= destAddress) {
        using K = TreatAsByteOrdinal;
        // original pseudo code for this path:
        // for i in 1 .. len loop
        //  byte(dst + len - i) <- byte(src + len - i)
        // end loop;
        for (Ordinal i = 1; i <= len; ++i) {
            store(destAddress + len - i, load(srcAddress + len - i, K{}), K{});
        }
    } else {
        movqstr(destAddress, srcAddress, len);
    }
}

void
Core::movqstr(Ordinal destAddress, Ordinal srcAddress, Ordinal len) noexcept {
    /// @todo figure out how to accelerate this to use strncpy instead of doing this manually
    using K = TreatAsByteOrdinal;
    for (Ordinal i = 0; i < len; ++i) {
        store(destAddress + i, load(srcAddress + i, K{}), K{});
    }
}

void
Core::fill(Ordinal dest, Ordinal value, Ordinal len) noexcept {
    Ordinal numWords = len / 4;
    for (Ordinal i = 0; i < numWords; ++i)  {
        store(dest + (i * 4), value, TreatAsOrdinal{});
    }
    switch (len % 4) {
        case 0:
            break;
        case 1:
            store(dest + len - 1, value, TreatAsByteOrdinal{});
            break;
        case 2:
            store(dest + len - 2, value, TreatAsShortOrdinal{});
            break;
        case 3:
            store(dest + len - 3, value, TreatAsShortOrdinal{});
            store(dest + len - 1, static_cast<ByteOrdinal>(value >> 16), TreatAsByteOrdinal{});
            break;
    }

}
void
Core::ldtime(LongRegister &dest) noexcept {
    dest.setValue(getProcessExecutionTime() - getResidualTimeSlice());
}

void
Core::condwait(SegmentSelector src) noexcept {
    /// @todo implement
    unimplementedFault();
}

void
Core::inspacc(Ordinal src, Register &dest) noexcept {
    /// @todo implement
    unimplementedFault();
}
void
Core::ldphy(Address address, Register& dest) noexcept {
    dest.setValue<Ordinal>(translateToPhysicalAddress(address));
}
void
Core::wait(SegmentSelector src) noexcept {
    unimplementedFault();
}

void
Core::condrec(SegmentSelector src, Register &dest) noexcept {
    unimplementedFault();
}
void
Core::resumprcs(SegmentSelector src) noexcept {
    unimplementedFault();
}
void
Core::signal(SegmentSelector sel) noexcept {
    unimplementedFault();
}

void
Core::send(SegmentSelector target, Ordinal src1, SegmentSelector src2) {
    unimplementedFault();
}
void
Core::sendserv(SegmentSelector src) noexcept {
    unimplementedFault();
}
void
Core::schedprcs(SegmentSelector src) noexcept {
    unimplementedFault();
}

void
Core::receive(SegmentSelector src, Register &dest) noexcept {
    unimplementedFault();
}

/// MMU related stuff
Address
Core::translateToPhysicalAddress(Address virtualAddress) noexcept {
    if (inVirtualMemoryMode()) {
        SegmentSelector currentSelector = 0;
        switch ((virtualAddress >> 30 ) & 0b11) {
            case 0b00:
                currentSelector = getRegion0SegmentSelector();
                break;
            case 0b01:
                currentSelector = getRegion1SegmentSelector();
                break;
            case 0b10:
                currentSelector = getRegion2SegmentSelector();
                break;
            case 0b11:
                currentSelector = getRegion3SegmentSelector();
                break;
        }
        return getSegmentBaseAddress(getDescriptor(currentSelector));
    } else {
        // physical address mode
        return virtualAddress;
    }
}

