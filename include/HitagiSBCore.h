// sim3
// Copyright (c) 2021, Joshua Scoggins
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
//
// Created by jwscoggins on 8/21/21.
//

#ifndef SIM3_HITAGISBCORE_H
#define SIM3_HITAGISBCORE_H
#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#ifdef ARDUINO_GRAND_CENTRAL_M4
#include <SdFat.h>
#endif
#include "Types.h"
#include "SBCoreArduino.h"

/**
 * @brief A version of the SBCore for an ATMega1284p originally designed to act as the chipset for an actual i960Sx processor;
 * This is meant to be a drop in replacement for that hardware so it will hold the i960Sx in reset and not use the CLKO pin either
 */
class HitagiSBCore : public SBCoreArduino {
public:
    static constexpr Address RamSize = 64_MB;
    static constexpr Address RamStart = 0x0000'0000;
    static constexpr Address RamMask = RamSize - 1;
    struct CacheLine {
        constexpr CacheLine() noexcept : address_(0), dirty_(false), valid_(false), storage_{ 0 } { }
        Address address_ = 0;
        bool dirty_ = false;
        bool valid_ = false;
        byte storage_[32] = { 0 };
        void clear() noexcept;
    };
public:
    using Parent = SBCoreArduino;
    HitagiSBCore();
    ~HitagiSBCore() override;
    void begin() override;
protected:
    ByteOrdinal ioSpaceLoad(Address address, TreatAsByteOrdinal ordinal) override;
    ShortOrdinal ioSpaceLoad(Address address, TreatAsShortOrdinal ordinal) override;
    Ordinal ioSpaceLoad(Address address, TreatAsOrdinal ordinal) override;
    void ioSpaceStore(Address address, ByteOrdinal value) override;
    void ioSpaceStore(Address address, ShortOrdinal value) override;
    void ioSpaceStore(Address address, Ordinal value) override;
    ByteOrdinal doIACLoad(Address address, TreatAsByteOrdinal ordinal) override;
    ShortOrdinal doIACLoad(Address address, TreatAsShortOrdinal ordinal) override;
    Ordinal doIACLoad(Address address, TreatAsOrdinal ordinal) override;
    void doIACStore(Address address, ByteOrdinal value) override;
    void doIACStore(Address address, ShortOrdinal value) override;
    void doIACStore(Address address, Ordinal value) override;
    Ordinal doRAMLoad(Address address, TreatAsOrdinal ordinal) override;
    void doRAMStore(Address address, ByteOrdinal value) override;
    void doRAMStore(Address address, ShortOrdinal value) override;
    void doRAMStore(Address address, Ordinal value) override;
    bool inRAMArea(Address target) noexcept override;
    Address toRAMOffset(Address target) noexcept override;
#ifdef ARDUINO_AVR_ATmega1284
private:
    void setPSRAMId(byte id) noexcept;
    void setupPSRAMChips() noexcept;
    size_t psramBlockRead(Address address, byte* buf, size_t count);
    size_t psramBlockWrite(Address address, byte* buf, size_t count);
private:
    union Decomposition {
        constexpr explicit Decomposition(byte value = 0) : index(value) { }
        constexpr auto getIndex() const noexcept { return index; }
        byte index;
        struct {
            bool s0 : 1;
            bool s1 : 1;
            bool s2 : 1;
        };
    };
    Decomposition chipId_;
#elif defined(ARDUINO_GRAND_CENTRAL_M4)
    File memoryImage_;
    static constexpr auto TransferCacheSize = 48_KB;
    byte transferCache[TransferCacheSize] = { 0 };
#else
#endif
    // make space for the on chip request cache as well as the psram copy buffer
    // minimum size is going to be 8k or so (256 x 32) but for our current purposes we
    // are going to allocate a 4k buffer
};

using SBCore = HitagiSBCore;
#endif
#endif //SIM3_HITAGISBCORE_H

