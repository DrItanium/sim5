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
#define _XOPEN_SOURCE 600
#include "Core.h"
#include <string>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <sstream>
#include <mutex>
#include <atomic>
#include <chrono>
namespace {
    union Cell {
        ByteOrdinal bytes[16];
        ByteInteger byteIntegers[16];
        ShortOrdinal shortOrdinals[8];
        ShortInteger shortIntegers[8];
        Ordinal ordinals[4];
        Integer integers[4];
        LongOrdinal longOrdinals[2];
        LongInteger longIntegers[2];
        float reals[4];
        double realLongs[2];
        long double extendedReal;
        QuadOrdinal quadOrdinal;
#define X(type) \
        void setValue(Address address, type value , TreatAs < type > ) noexcept; \
        type getValue(Address address, TreatAs < type > ) const noexcept
        X(ByteOrdinal);
        X(ByteInteger);
        X(ShortOrdinal);
        X(ShortInteger);
        X(Ordinal);
        X(Integer);
        X(LongOrdinal);
        X(LongInteger);
        X(float);
        X(double);
        X(long double);
        X(QuadOrdinal);
#undef X
    };
    std::unique_ptr<uint8_t[]> physicalMemory;
    std::chrono::time_point startup = std::chrono::system_clock::now();
    //bool* tagBits = nullptr;
} // end namespace
void
Core::nonPortableBegin() noexcept {
    if (!physicalMemory) {
        // allocate 4 gigabytes of memory
        physicalMemory = std::make_unique<uint8_t[]>((0x1'0000'0000));
    }
    //if (!tagBits) {
    //    // tagBits are aligned to 32-bit boundaries
    //    tagBits = new bool[(0x1'0000'0000) >> 2]();
    //}


    // setup the
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
    Cell&
    getCell(Address address) noexcept {
        return *reinterpret_cast<Cell*>(physicalMemory.get() + (address & 0xFFFF'FFF0));
    }
    template<typename T>
    T* getPointer(Address address) noexcept {
        return reinterpret_cast<T*>(physicalMemory.get() + address);
    }
    template<typename T>
    T& getMemoryReference(Address address) noexcept {
        return *(getPointer<T>(address));
    }
    template<typename T>
    T tryGetFromConsole() {
        auto value = static_cast<T>(std::cin.get());
        if (std::cin.fail()) {
            return static_cast<T>(-1);
        } else {
            return value;
        }
    }
    decltype(auto) getDurationSinceStartup() noexcept {
        return std::chrono::system_clock::now() - startup;
    }
    template<typename T>
    T micros() noexcept {
        return static_cast<T>(std::chrono::duration_cast<std::chrono::microseconds>(getDurationSinceStartup()).count());
    }
    template<typename T>
    T millis() noexcept {
        return static_cast<T>(std::chrono::duration_cast<std::chrono::milliseconds>(getDurationSinceStartup()).count());
    }
    template<typename T>
    T ioLoad(Address offset, TreatAs<T>) {
        DEBUG_LOG_LEVEL(1) {
            std::cout << __PRETTY_FUNCTION__ << "(0x" << std::hex << offset << ")" << std::endl;
        }
        if (auto target = offset & 0xFF'FFFF; target >= 0x10'0000) {
            return getMemoryReference<T>(offset);
        } else {
            switch (target) {
                case 0x00'0000:
                    return static_cast<T>(10 * 1024 * 1024);
                case 0x00'0004:
                    return static_cast<T>(20 * 1024 * 1024);
                case 0x00'0008:
                    return tryGetFromConsole<T>();
                case 0x00'0040:
                    return millis<T>();
                case 0x00'0044:
                    return micros<T>();
                default:
                    return 0;
            }
        }
    }
    template<typename T>
    void ioStore(Address offset, T value, TreatAs<T>) {
        DEBUG_LOG_LEVEL(1) {
            std::cout << __PRETTY_FUNCTION__ << "(0x" << std::hex << offset << ", 0x" << std::hex << static_cast<Ordinal>(value) << ")" << std::endl;
        }
        if (auto target = offset & 0xFF'FFFF; target >= 0x10'0000) {
            getMemoryReference<T>(offset) = value;
        } else {
            switch (target) {
                case 0x00'0008:
                    std::cout.put(static_cast<char>(value));
                    break;
                case 0x00'000C:
                    std::cout.flush();
                    break;
                default:
                    break;
            }
        }
    }
    template<typename T>
    T load(Address address, TreatAs<T>) {
        switch (static_cast<uint8_t>(address >> 24))  {
            case 0xFE:
                return ioLoad<T>(address, TreatAs<T>{});
            case 0xFF: // CPU reserved and also fix memory overflow problems
                return 0;
            default:
                return getMemoryReference<T>(address);
        }
    }
    ByteOrdinal
    load8(Address address, TreatAsByteOrdinal) noexcept {
        switch (static_cast<uint8_t>(address >> 24)) {
            case 0xFE:
                return ioLoad(address, TreatAsByteOrdinal{});
            case 0xFF:
                return 0;
            default:
                return getCell(address).getValue(address, TreatAsByteOrdinal{});
        }
    }
    ByteInteger
    load8(Address address, TreatAsByteInteger) noexcept {
        switch (static_cast<uint8_t>(address >> 24)) {
            case 0xFE:
                return ioLoad(address, TreatAsByteInteger{});
            case 0xFF:
                return 0;
            default:
                return getCell(address).getValue(address, TreatAsByteInteger{});
        }
    }
    template<typename T>
    void store(Address address, T value, TreatAs<T>) {
        switch (static_cast<uint8_t>(address >> 24))  {
            case 0xFE:
                ioStore<T>(address, value, TreatAs<T>{});
                break;
            case 0xFF: // CPU reserved and also fix memory overflow problems
                break;
            default:
                getMemoryReference<T>(address) = value;
                break;
        }
    }
    void
    store8(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept {
        store(address, value, TreatAsByteOrdinal{});
    }
    void
    store16(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept {
        store(address, value, TreatAsShortOrdinal{});
    }
    void
    store8(Address address, ByteInteger value, TreatAsByteInteger) noexcept {
        store(address, value, TreatAsByteInteger{});
    }
    void
    store16(Address address, ShortInteger value, TreatAsShortInteger) noexcept {
        store(address, value, TreatAsShortInteger {});
    }
    void
    store32(Address address, Ordinal value, TreatAsOrdinal) noexcept {
        store(address, value, TreatAsOrdinal{});
    }
    void
    store32(Address address, Integer value, TreatAsInteger) noexcept {
        store(address, value, TreatAsInteger{});
    }
    Ordinal
    load32(Address address, TreatAsOrdinal) noexcept {
        return load<Ordinal>(address, TreatAsOrdinal{});
    }
    Integer
    load32(Address address, TreatAsInteger) noexcept {
        return load<Integer>(address, TreatAsInteger{});
    }
    void Cell::setValue(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept { bytes[getOffset(address, TreatAsByteOrdinal{})] = value; }
    void Cell::setValue(Address address, ByteInteger value, TreatAsByteInteger) noexcept { byteIntegers[getOffset(address, TreatAsByteInteger{})] = value; }
    void Cell::setValue(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept { shortOrdinals[getOffset(address, TreatAsShortOrdinal{})] = value; }
    void Cell::setValue(Address address, ShortInteger value, TreatAsShortInteger) noexcept { shortIntegers[getOffset(address, TreatAsShortInteger{})] = value; }
    void Cell::setValue(Address address, Ordinal value, TreatAsOrdinal) noexcept { ordinals[getOffset(address, TreatAsOrdinal{})] = value; }
    void Cell::setValue(Address address, Integer value, TreatAsInteger) noexcept { integers[getOffset(address, TreatAsInteger{})] = value; }
    void Cell::setValue(Address address, LongOrdinal value, TreatAsLongOrdinal) noexcept { longOrdinals[getOffset(address, TreatAsLongOrdinal{})] = value; }
    ByteOrdinal
    Cell::getValue(Address address, TreatAsByteOrdinal) const noexcept {
        return bytes[getOffset(address, TreatAsByteOrdinal{})];
    }

    ByteInteger
    Cell::getValue(Address address, TreatAsByteInteger) const noexcept {
        return byteIntegers[getOffset(address, TreatAsByteInteger{})];
    }

    ShortOrdinal
    Cell::getValue(Address address, TreatAsShortOrdinal) const noexcept {
        return shortOrdinals[getOffset(address, TreatAsShortOrdinal{})];
    }

    ShortInteger
    Cell::getValue(Address address, TreatAsShortInteger) const noexcept {
        return shortIntegers[getOffset(address, TreatAsShortInteger{})];

    }
    Ordinal
    Cell::getValue(Address address, TreatAsOrdinal) const noexcept {
        return ordinals[getOffset(address, TreatAsOrdinal{})];
    }

    Integer
    Cell::getValue(Address address, TreatAsInteger) const noexcept {
        return integers[getOffset(address, TreatAsInteger{})];
    }

    LongOrdinal
    Cell::getValue(Address address, TreatAsLongOrdinal) const noexcept {
        return longOrdinals[getOffset(address, TreatAsLongOrdinal{})];
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
Core::checksumFail() {
    throw std::runtime_error("checksum fail");
    // cause termination to happen
}
void
Core::selfTestFailure() {
    throw std::runtime_error("self test failure!");
}
void
Core::badFault(const FaultRecord& record) {
    throw std::runtime_error("bad fault");
}

void
installToMainMemory(std::istream& stream, Address baseAddress) {
    while (stream.good()) {
        store8(baseAddress, stream.get(), TreatAsByteOrdinal {});
        ++baseAddress;
    }
}
void
installToMainMemory(Address baseAddress, const char* data, Address size) {
    for (auto addr = 0; addr < size; ++addr) {
        store8(baseAddress + addr, data[addr], TreatAsByteOrdinal{});
    }
}
void
clearMainMemory(Address baseAddress, Address size) {
    for (auto addr = 0; addr < size; ++addr) {
        store8(baseAddress + addr, 0, TreatAsByteOrdinal{});
    }
}
namespace {
    constexpr bool isAligned(Address address, TreatAsLongOrdinal) noexcept {
        return (address & 0b111) == 0;
    }
    constexpr bool isAligned(Address address, TreatAsQuadOrdinal) noexcept {
        return (address & 0b1111) == 0;
    }
    constexpr bool isAlignedToQuadBoundaries(Address address) noexcept {
        return isAligned(address, TreatAsQuadOrdinal{});
    }
    void
    Cell::setValue(Address, QuadOrdinal value, TreatAs<QuadOrdinal>) noexcept {
        // only works on aligned values so don't check address
        quadOrdinal = value;
    }
    QuadOrdinal Cell::getValue(Address, TreatAs<QuadOrdinal>) const noexcept { return quadOrdinal; }
}
void
Core::store(Address address, QuadOrdinal value, TreatAsQuadOrdinal) noexcept {
    if (isAligned(address, TreatAsQuadOrdinal{})) {
        getCell(address).setValue(address, value, TreatAsQuadOrdinal {});
    } else {
        store(address, static_cast<LongOrdinal>(value), TreatAsLongOrdinal{});
        store(address + 8, static_cast<LongOrdinal>(value >> 64), TreatAsLongOrdinal {});
    }
}

QuadOrdinal
Core::load(Address address, TreatAsQuadOrdinal) const noexcept {
    if (isAligned(address, TreatAsQuadOrdinal{})) {
        return getCell(address).getValue(address, TreatAsQuadOrdinal{});
    } else {
        // unaligned accesses need to be handled differently to make sure that we do a _wraparound_ instead
        // of reading into unowned memory.
        auto lower = static_cast<QuadOrdinal>(load(address, TreatAsLongOrdinal{}));
        auto upper = static_cast<QuadOrdinal>(load(address + 8, TreatAsLongOrdinal{}));
        return lower | upper;
    }
}

LongOrdinal
Core::load(Address address, TreatAs<LongOrdinal>) const noexcept {
    if (isAligned(address, TreatAsLongOrdinal{})) {
        return getCell(address).getValue(address, TreatAsLongOrdinal{});
    } else {
        auto lower = static_cast<LongOrdinal>(load(address, TreatAsOrdinal{}));
        auto upper = static_cast<LongOrdinal>(load(address + 4, TreatAsOrdinal{})) << 32;
        return lower | upper;
    }
}

void
Core::store(Address address, LongOrdinal value, TreatAs<LongOrdinal>) noexcept {
    if (isAligned(address, TreatAsLongOrdinal{})) {
        getCell(address).setValue(address, value, TreatAs<LongOrdinal>{});
    } else {
        store(address, static_cast<Ordinal>(value), TreatAsOrdinal{});
        store(address + 4, static_cast<Ordinal>(value >> 32), TreatAsOrdinal{});
    }
}
