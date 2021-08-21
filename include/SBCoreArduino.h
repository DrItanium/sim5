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

#ifndef SIM3_SBCOREARDUINO_H
#define SIM3_SBCOREARDUINO_H
#ifdef ARDUINO
#include "Types.h"
#include "SimplifiedSxCore.h"

class SBCoreArduino : public SimplifiedSxCore {
public:
    using Parent = SimplifiedSxCore;
    using Parent::Parent;
    static constexpr Address HaltRegisterOffset = 0x00FF'FFFC;
    static constexpr Address ConsoleRegisterOffset = 0x00E0'0000;
    static constexpr Address ConsoleFlushOffset = 0x00E0'0004;
    static constexpr Address IACBaseAddress = 0x0000'0010;
    SBCoreArduino();
    ~SBCoreArduino() override;
    virtual void begin();
protected:
    ShortOrdinal loadShort(Address destination) override;
    void storeShort(Address destination, ShortOrdinal value) override;
    ByteOrdinal loadByte(Address destination) override;
    Ordinal load(Address address) override;
    void store(Address address, Ordinal value) override;
    void generateFault(FaultType ) override;
protected:
    virtual
private:
    static constexpr bool inIOSpace(Address target) noexcept {
        return target >= 0xFE00'0000 && !inIACSpace(target);
    }
    static constexpr bool inIACSpace(Address target) noexcept {
        return target >= 0xFF00'0000;
    }
    static constexpr bool inRAMArea(Address target) noexcept {
        return target < 64_MB;
    }
};

using SBCore = SBCoreArduino;
#endif
#endif //SIM3_SBCOREARDUINO_H