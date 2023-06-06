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
#ifndef ARDUINO
#include "Core.h"
#include <string>
namespace {
    union Cell {
        ByteOrdinal bytes[16];
        ByteInteger byteIntegers[16];
        ShortOrdinal shortOrdinals[8];
        ShortInteger shortIntegers[8];
        Ordinal ordinals[4];
        Integer integers[4];
        LongOrdinal longOrdinals[2];
        void setValue(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept;
        void setValue(Address address, ByteInteger value, TreatAsByteInteger) noexcept;
        void setValue(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept;
        void setValue(Address address, ShortInteger value, TreatAsShortInteger) noexcept;
        void setValue(Address address, Ordinal value, TreatAsOrdinal) noexcept;
        void setValue(Address address, Integer value, TreatAsInteger) noexcept;
        void setValue(Address address, LongOrdinal value, TreatAsLongOrdinal) noexcept;
        ByteOrdinal getValue(Address address, TreatAsByteOrdinal) const noexcept;
        ByteInteger getValue(Address address, TreatAsByteInteger) const noexcept;
        ShortOrdinal getValue(Address address, TreatAsShortOrdinal) const noexcept;
        ShortInteger getValue(Address address, TreatAsShortInteger) const noexcept;
        Ordinal getValue(Address address, TreatAsOrdinal) const noexcept;
        Integer getValue(Address address, TreatAsInteger) const noexcept;
        LongOrdinal getValue(Address address, TreatAsLongOrdinal) const noexcept;
    };
    Cell* physicalMemory = nullptr;
} // end namespace
void 
Core::nonPortableBegin() noexcept {
    if (!physicalMemory) {
        // allocate 4 gigabytes of memory
        physicalMemory = new Cell[(0x1'0000'0000) / sizeof(Cell)]();
    }
}




void 
Core::lockBus() noexcept {
}

void 
Core::unlockBus() noexcept {
}


bool 
Core::runNonPortableSelfTests() noexcept {
    return true;
}

void 
Core::assertFailureState() noexcept {
}

void 
Core::deassertFailureState() noexcept {
}

void
Core::purgeInstructionCache() noexcept {
    ///@todo implement when we have an instruction cache!
}

namespace {
constexpr uint8_t getOffset32(Address input) noexcept { return (input >> 2) & 0b11; }
constexpr uint8_t getOffset16(Address input) noexcept { return (input >> 1) & 0b111; }
constexpr uint8_t getOffset8(Address input) noexcept { return input & 0b1111; }
constexpr uint8_t getOffset(Address input, TreatAsLongOrdinal) noexcept { return (input >> 3) & 0b1; }
constexpr uint8_t getOffset(Address input, TreatAsOrdinal) noexcept { return getOffset32(input); }
constexpr uint8_t getOffset(Address input, TreatAsInteger) noexcept { return getOffset32(input); }
constexpr uint8_t getOffset(Address input, TreatAsShortOrdinal) noexcept { return getOffset16(input); }
constexpr uint8_t getOffset(Address input, TreatAsShortInteger) noexcept { return getOffset16(input); }
constexpr uint8_t getOffset(Address input, TreatAsByteOrdinal) noexcept { return getOffset8(input); }
constexpr uint8_t getOffset(Address input, TreatAsByteInteger) noexcept { return getOffset8(input); }
constexpr Address getCellAddress(Address input) noexcept { return input >> 4; }
Cell& 
getCell(Address address) noexcept {
    return physicalMemory[getCellAddress(address)];
}
ByteOrdinal
load8(Address address, TreatAsByteOrdinal) noexcept {
    switch (static_cast<uint8_t>(address) >> 24) {
        case 0xFE:
            return 0;
        case 0xFF:
            return 0;
        default:
            return getCell(address).bytes[getOffset(address, TreatAsByteOrdinal{})];
    }
}
ShortOrdinal
load16(Address address, TreatAsShortOrdinal) noexcept {
    if ((address & 0b1) != 0) {
        auto lower = load8(address, TreatAsByteOrdinal{});
        auto upper = load8(address+1, TreatAsByteOrdinal{});
        return static_cast<ShortOrdinal>(lower) |
              (static_cast<ShortOrdinal>(upper) << 8);
    } else {
        switch (static_cast<uint8_t>(address) >> 24) {
            case 0xFE:
                return 0;
            case 0xFF:
                return 0;
            default:
                return getCell(address).shortOrdinals[getOffset(address, TreatAsShortOrdinal{})];
        }
    }
}
ByteInteger
load8(Address address, TreatAsByteInteger) noexcept {
    switch (static_cast<uint8_t>(address) >> 24) {
        case 0xFE:
            return 0;
        case 0xFF:
            return 0;
        default:
            return getCell(address).byteIntegers[getOffset(address, TreatAsByteInteger{})];
    }
}
ShortInteger
load16(Address address, TreatAsShortInteger) noexcept {
    if ((address & 0b1) != 0) {
        auto lower = load8(address, TreatAsByteInteger{});
        auto upper = load8(address+1, TreatAsByteInteger{});
        return static_cast<ShortInteger>(lower) |
              (static_cast<ShortInteger>(upper) << 8);
    } else {
        switch (static_cast<uint8_t>(address) >> 24) {
            case 0xFE:
                return 0;
            case 0xFF:
                return 0;
            default:
                return getCell(address).shortIntegers[getOffset(address, TreatAsShortInteger{})];
        }
    }
}
void
store8(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept {
    switch (static_cast<uint8_t>(address) >> 24) {
        case 0xFE:
            break;
        case 0xFF:
            break;
        default:
            getCell(address).bytes[getOffset(address, TreatAsByteOrdinal{})] = value;
            break;
    }
}
void
store16(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept {
    if ((address & 0b1) != 0) {
        store8(address, value, TreatAsByteOrdinal{});
        store8(address+1, static_cast<ByteOrdinal>(value >> 8), TreatAsByteOrdinal{});
    } else {
        switch (static_cast<uint8_t>(address) >> 24) {
            case 0xFE:
                break;
            case 0xFF:
                break;
            default:
            getCell(address).bytes[getOffset(address, TreatAsByteOrdinal{})] = value;
                reinterpret_cast<ShortOrdinal*>(physicalMemory)[address >> 1] = value;
                break;
        }
    }
}
void
store8(Address address, ByteInteger value, TreatAsByteInteger) noexcept {
    switch (static_cast<uint8_t>(address) >> 24) {
        case 0xFE:
            break;
        case 0xFF:
            break;
        default:
            reinterpret_cast<ByteInteger*>(physicalMemory)[address] = value;
            break;
    }
}
void
store16(Address address, ShortInteger value, TreatAsShortInteger) noexcept {
    if ((address & 0b1) != 0) {
        store8(address, value, TreatAsByteInteger{});
        store8(address+1, static_cast<ByteInteger>(value >> 8), TreatAsByteInteger{});
    } else {
        switch (static_cast<uint8_t>(address) >> 24) {
            case 0xFE:
                break;
            case 0xFF:
                break;
            default:
                reinterpret_cast<ShortInteger*>(physicalMemory)[address >> 1] = value;
                break;
        }
    }
}
void
store32(Address address, Ordinal value, TreatAsOrdinal) noexcept {
    if ((address & 0b11) != 0) {
        store16(address, value, TreatAsShortOrdinal{});
        store16(address+2, static_cast<ShortOrdinal>(value >> 16), TreatAsShortOrdinal{});
    } else {
        switch (static_cast<uint8_t>(address) >> 24) {
            case 0xFE:
                break;
            case 0xFF:
                break;
            default:
                reinterpret_cast<Ordinal*>(physicalMemory)[address >> 2] = value;
                break;
        }
    }
}
void
store32(Address address, Integer value, TreatAsInteger) noexcept {
    if ((address & 0b11) != 0) {
        store16(address, value, TreatAsShortInteger{});
        store16(address+2, static_cast<ShortInteger>(value >> 16), TreatAsShortInteger{});
    } else {
        switch (static_cast<uint8_t>(address) >> 24) {
            case 0xFE:
                break;
            case 0xFF:
                break;
            default:
                reinterpret_cast<Integer*>(physicalMemory)[address >> 2] = value;
                break;
        }
    }
}
Ordinal
load32(Address address, TreatAsOrdinal) noexcept {
    if ((address & 0b11) != 0) {
        // unaligned 32-bit load so make sure that we break it up into
        // component pieces by loading each byte individually
        // this will make sure that we will automatically wrap around on an
        // unaligned load at the top of memory
        auto lowest = load8(address, TreatAsByteOrdinal{});
        auto lower = load8(address + 1, TreatAsByteOrdinal{});
        auto higher = load8(address + 2, TreatAsByteOrdinal{});
        auto highest = load8(address + 3, TreatAsByteOrdinal{});
        return static_cast<Ordinal>(lowest) | 
               (static_cast<Ordinal>(lower) << 8) |
               (static_cast<Ordinal>(higher) << 16) |
               (static_cast<Ordinal>(highest) << 24);
    } else {
        switch (static_cast<uint8_t>(address) >> 24) {
            case 0xFE: // io space
                return 0;
            case 0xFF: // onboard devices
                return 0;
            default:
                return reinterpret_cast<Ordinal*>(physicalMemory)[address >> 2];
        }
    }
}
Integer
load32(Address address, TreatAsInteger) noexcept {
    if ((address & 0b11) != 0) {
        // unaligned 32-bit load so make sure that we break it up into
        // component pieces by loading each byte individually
        // this will make sure that we will automatically wrap around on an
        // unaligned load at the top of memory
        auto lowest = load8(address, TreatAsByteInteger{});
        auto lower = load8(address + 1, TreatAsByteInteger{});
        auto higher = load8(address + 2, TreatAsByteInteger{});
        auto highest = load8(address + 3, TreatAsByteInteger{});
        return static_cast<Integer>(lowest) | 
               (static_cast<Integer>(lower) << 8) |
               (static_cast<Integer>(higher) << 16) |
               (static_cast<Integer>(highest) << 24);
    } else {
        switch (static_cast<uint8_t>(address) >> 24) {
            case 0xFE: // io space
                return 0;
            case 0xFF: // onboard devices
                return 0;
            default:
                return reinterpret_cast<Integer*>(physicalMemory)[address >> 2];
        }
    }
}
void Cell::setValue(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept { bytes[getOffset(address, TreatAsByteOrdinal{})] = value; }
void Cell::setValue(Address address, ByteInteger value, TreatAsByteInteger) noexcept { byteIntegers[getOffset(address, TreatAsByteInteger{})] = value; }
void Cell::setValue(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept { shortOrdinals[getOffset(address, TreatAsShortOrdinal{})] = value; }
void Cell::setValue(Address address, ShortInteger value, TreatAsShortInteger) noexcept { shortIntegers[getOffset(address, TreatAsShortInteger{})] = value; }
void Cell::setValue(Address address, Ordinal value, TreatAsOrdinal) noexcept { ordinals[getOffset(address, TreatAsOrdinal{})] = value; }
void Cell::setValue(Address address, Integer value, TreatAsInteger) noexcept { integers[getOffset(address, TreatAsInteger{})] = value; }
void Cell::setValue(Address address, LongOrdinal value, TreatAsLongOrdinal) noexcept { longOrdinals[getOffset(address, TreatAsLongOrdinal{})] = value; }
ByteOrdinal Cell::getValue(Address address, TreatAsByteOrdinal) const noexcept {

}

ByteInteger Cell::getValue(Address address, TreatAsByteInteger) const noexcept {

}

ShortOrdinal Cell::getValue(Address address, TreatAsShortOrdinal) const noexcept {

}

ShortInteger Cell::getValue(Address address, TreatAsShortInteger) const noexcept {

}

Ordinal Cell::getValue(Address address, TreatAsOrdinal) const noexcept {

}

Integer Cell::getValue(Address address, TreatAsInteger) const noexcept {

}

LongOrdinal Cell::getValue(Address address, TreatAsLongOrdinal) const noexcept {

}

} 

Ordinal 
Core::load(Address address, TreatAsOrdinal) const noexcept {
    return load32(address, TreatAsOrdinal{});
}

Integer 
Core::load(Address address, TreatAsInteger) const noexcept {
    return load32(address, TreatAsInteger{});
}

ShortOrdinal 
Core::load(Address address, TreatAsShortOrdinal) const noexcept {
    return load16(address, TreatAsShortOrdinal{});
}

ShortInteger 
Core::load(Address address, TreatAsShortInteger) const noexcept {
    return load16(address, TreatAsShortInteger{});
}
ByteOrdinal 
Core::load(Address address, TreatAsByteOrdinal) const noexcept {
    return load8(address, TreatAsByteOrdinal{});
}

ByteInteger 
Core::load(Address address, TreatAsByteInteger) const noexcept {
    return load8(address, TreatAsByteInteger{});
}


void 
Core::store(Address address, Ordinal value, TreatAsOrdinal) noexcept {
    store32(address, value, TreatAsOrdinal{});
}

void
Core::store(Address address, Integer value, TreatAsInteger) noexcept {
    store32(address, value, TreatAsInteger{});
}

void
Core::store(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept {
    store16(address, value, TreatAsShortOrdinal{});
}

void
Core::store(Address address, ShortInteger value, TreatAsShortInteger) noexcept {
    store16(address, value, TreatAsShortInteger{});
}

void
Core::store(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept {
    store8(address, value, TreatAsByteOrdinal {});
}

void
Core::store(Address address, ByteInteger value, TreatAsByteInteger) noexcept {
    store8(address, value, TreatAsByteInteger{});
}
void 
Core::checksumFail() noexcept {
}
void 
Core::selfTestFailure() noexcept {
}
#endif

