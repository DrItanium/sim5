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
    std::unique_ptr<uint8_t[]> physicalMemory;
    std::chrono::time_point startup = std::chrono::system_clock::now();
    //bool* tagBits = nullptr;
} // end namespace
void
Core::nonPortableBegin() noexcept {
    if (!physicalMemory) {
        physicalMemory = std::make_unique<uint8_t[]>((getMemoryCapacity()));
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
        switch (offset & 0xFF'FFFF) {
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
    template<typename T>
    void ioStore(Address offset, T value, TreatAs<T>) {
        DEBUG_LOG_LEVEL(1) {
            std::cout << __PRETTY_FUNCTION__ << "(0x" << std::hex << offset << ", 0x" << std::hex << static_cast<Ordinal>(value) << ")" << std::endl;
        }
        switch (offset & 0xFF'FFFF) {
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
    template<typename T>
    T load(Address address, TreatAs<T>) {
        switch (static_cast<uint8_t>(address >> 24))  {
            case 0xFE:
                return ioLoad<T>(address, TreatAs<T>{});
            case 0xFF: // CPU reserved and also fix memory overflow problems
                return 0;
            default:
                if (address < getMemoryCapacity()) {
                    return getMemoryReference<T>(address);
                } else {
                    return 0; // unmapped memory returns zero
                }
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
                if (address < getMemoryCapacity()) {
                    getMemoryReference<T>(address) = value;
                } 
                // if writing to unmapped memory then don't do anything
                break;
        }
    }
    void
    store8(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept {
        store(address, value, TreatAsByteOrdinal{});
    }
}

Ordinal
Core::load(Address address, TreatAsOrdinal) const noexcept {
    return ::load(address, TreatAsOrdinal{});
}

Integer
Core::load(Address address, TreatAsInteger) const noexcept {
    return ::load(address, TreatAsInteger{});
}



void
Core::store(Address address, Ordinal value, TreatAsOrdinal) noexcept {
    ::store(address, value, TreatAsOrdinal{});
}

void
Core::store(Address address, Integer value, TreatAsInteger) noexcept {
    ::store(address, value, TreatAsInteger{});
}

void
Core::store(Address address, ShortOrdinal value, TreatAsShortOrdinal) noexcept {
    ::store(address, value, TreatAsShortOrdinal{});
}

void
Core::store(Address address, ShortInteger value, TreatAsShortInteger) noexcept {
    ::store(address, value, TreatAsShortInteger{});
}

void
Core::store(Address address, ByteOrdinal value, TreatAsByteOrdinal) noexcept {
    ::store(address, value, TreatAsByteOrdinal {});
}

void
Core::store(Address address, ByteInteger value, TreatAsByteInteger) noexcept {
    ::store(address, value, TreatAsByteInteger{});
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
void
Core::store(Address address, QuadOrdinal value, TreatAsQuadOrdinal) noexcept {
    ::store(address, value, TreatAsQuadOrdinal {});
}

QuadOrdinal
Core::load(Address address, TreatAsQuadOrdinal) const noexcept {
    return ::load(address, TreatAsQuadOrdinal{});
}

LongOrdinal
Core::load(Address address, TreatAs<LongOrdinal>) const noexcept {
    return ::load(address, TreatAsLongOrdinal{});
}

void
Core::store(Address address, LongOrdinal value, TreatAs<LongOrdinal>) noexcept {
    ::store(address, value, TreatAsLongOrdinal{});
}
