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
//
// Created by jwscoggins on 6/10/24.
//

#ifndef SIM960_EXTENDEDOPCODE_H
#define SIM960_EXTENDEDOPCODE_H
#include <cstdint>
#include "Types.h"
constexpr bool isREGFormatOpcode(uint8_t value) noexcept {
    return (value >= 0x40) && (value < 0x80);
}
constexpr bool isREGFormatOpcode(Ordinal value) noexcept {
    return isREGFormatOpcode(static_cast<uint8_t>(value >> 24));
}
constexpr bool isMEMFormatOpcode(uint8_t value) noexcept {
    return (value >= 0x80);
}
constexpr bool isMEMFormatOpcode(Ordinal value) noexcept {
    return isMEMFormatOpcode(static_cast<uint8_t>(value >> 24));
}
constexpr bool isCOBRFormatOpcode(uint8_t value) noexcept {
    return (value >= 0x20) && (value < 0x40);
}
constexpr bool isCOBRFormatOpcode(Ordinal value) noexcept {
    return isCOBRFormatOpcode(static_cast<uint8_t>(value >> 24));
}
constexpr bool isCTRLFormatOpcode(uint8_t value) noexcept {
    return (value < 0x20);
}
constexpr bool isCTRLFormatOpcode(Ordinal value) noexcept {
    return isCTRLFormatOpcode(static_cast<uint8_t>(value >> 24));
}

constexpr uint32_t constructInplaceMask(uint8_t major, uint8_t minor) noexcept {
    auto full = static_cast<uint32_t>(major) << 24;
    if (isREGFormatOpcode(major)) {
        return full | static_cast<uint32_t>(static_cast<uint32_t>(minor) << 7);
    } else {
        return full;
    }
}
// X (
//    name : "the name of the instruction",
//    opcode : "the numerical opcode",
//    str    : "the string representation of the instruction",
//    level  : "architecture level",
//    privileged : "is the instruction privileged (boolean)",
//    fmt        : "instruction format",
//    flt        : "is it a float operation?",
//    minor      : "minor opcode",
// )
constexpr uint32_t generateCobrBits(bool m1, bool s2, bool t) noexcept {
    uint32_t result = 0;
    result |= m1 ? 0b10'0000000000'00 : 0;
    result |= s2 ? 0b00'0000000000'01 : 0;
    result |= t  ? 0b00'0000000000'10 : 0;
    return result;
}
constexpr uint32_t generateMemBits(uint8_t value) noexcept {
    return static_cast<uint32_t>(value& 0b1111) << 10;
}
static_assert(generateMemBits(0b1111) == 0b11'11'000'00'00000);
constexpr uint32_t generateCobrBits(uint8_t value) noexcept {
    return generateCobrBits(
            value & 0b001,
            value & 0b010,
            value & 0b100
    );
}
constexpr uint32_t generateRegBits(uint8_t value) noexcept {
    uint32_t result = 0;
    result |= (value & 0b00001) ? 0b000'0000'01'00000 : 0;
    result |= (value & 0b00010) ? 0b000'0000'10'00000 : 0;
    result |= (value & 0b00100) ? 0b001'0000'00'00000 : 0;
    result |= (value & 0b01000) ? 0b010'0000'00'00000 : 0;
    result |= (value & 0b10000) ? 0b100'0000'00'00000 : 0;
    return result;
}
enum class ExtendedOpcode : uint32_t {
#define COBR(name, opcode, str, level, privileged, flt, minor) \
    name ## _Type0 = constructInplaceMask(opcode, minor) | generateCobrBits(0),               \
    name ## _Type1 = constructInplaceMask(opcode, minor) | generateCobrBits(1),               \
    name ## _Type2 = constructInplaceMask(opcode, minor) | generateCobrBits(2),               \
    name ## _Type3 = constructInplaceMask(opcode, minor) | generateCobrBits(3),               \
    name ## _Type4 = constructInplaceMask(opcode, minor) | generateCobrBits(4),               \
    name ## _Type5 = constructInplaceMask(opcode, minor) | generateCobrBits(5),               \
    name ## _Type6 = constructInplaceMask(opcode, minor) | generateCobrBits(6),               \
    name ## _Type7 = constructInplaceMask(opcode, minor) | generateCobrBits(7),               \
    name = name ## _Type0,



#define CTRL(name, opcode, str, level, privileged, flt, minor)  \
    name ## _Type0 = constructInplaceMask(opcode, minor),                 \
    name ## _Type1 = constructInplaceMask(opcode, minor) | 0b10,\
    name = name ## _Type0,

#define REG(name, opcode, str, level, privileged, flt, minor) \
    name ## _Type0 = constructInplaceMask(opcode, minor) | generateRegBits(0),               \
    name ## _Type1 = constructInplaceMask(opcode, minor) | generateRegBits(1),               \
    name ## _Type2 = constructInplaceMask(opcode, minor) | generateRegBits(2),               \
    name ## _Type3 = constructInplaceMask(opcode, minor) | generateRegBits(3),               \
    name ## _Type4 = constructInplaceMask(opcode, minor) | generateRegBits(4),               \
    name ## _Type5 = constructInplaceMask(opcode, minor) | generateRegBits(5),               \
    name ## _Type6 = constructInplaceMask(opcode, minor) | generateRegBits(6),               \
    name ## _Type7 = constructInplaceMask(opcode, minor) | generateRegBits(7),               \
    name ## _Type8 = constructInplaceMask(opcode, minor) | generateRegBits(8),               \
    name ## _Type9 = constructInplaceMask(opcode, minor) | generateRegBits(9),               \
    name ## _Type10 = constructInplaceMask(opcode, minor) | generateRegBits(10),               \
    name ## _Type11 = constructInplaceMask(opcode, minor) | generateRegBits(11),               \
    name ## _Type12 = constructInplaceMask(opcode, minor) | generateRegBits(12),               \
    name ## _Type13 = constructInplaceMask(opcode, minor) | generateRegBits(13),               \
    name ## _Type14 = constructInplaceMask(opcode, minor) | generateRegBits(14),               \
    name ## _Type15 = constructInplaceMask(opcode, minor) | generateRegBits(15),               \
    name ## _Type16 = constructInplaceMask(opcode, minor) | generateRegBits(16),               \
    name ## _Type17 = constructInplaceMask(opcode, minor) | generateRegBits(17),               \
    name ## _Type18 = constructInplaceMask(opcode, minor) | generateRegBits(18),               \
    name ## _Type19 = constructInplaceMask(opcode, minor) | generateRegBits(19),               \
    name ## _Type20 = constructInplaceMask(opcode, minor) | generateRegBits(20),               \
    name ## _Type21 = constructInplaceMask(opcode, minor) | generateRegBits(21),               \
    name ## _Type22 = constructInplaceMask(opcode, minor) | generateRegBits(22),               \
    name ## _Type23 = constructInplaceMask(opcode, minor) | generateRegBits(23),               \
    name ## _Type24 = constructInplaceMask(opcode, minor) | generateRegBits(24),               \
    name ## _Type25 = constructInplaceMask(opcode, minor) | generateRegBits(25),               \
    name ## _Type26 = constructInplaceMask(opcode, minor) | generateRegBits(26),               \
    name ## _Type27 = constructInplaceMask(opcode, minor) | generateRegBits(27),               \
    name ## _Type28 = constructInplaceMask(opcode, minor) | generateRegBits(28),               \
    name ## _Type29 = constructInplaceMask(opcode, minor) | generateRegBits(29),               \
    name ## _Type30 = constructInplaceMask(opcode, minor) | generateRegBits(30),               \
    name ## _Type31 = constructInplaceMask(opcode, minor) | generateRegBits(31),               \
    name = name ## _Type0,

#define MEM(name, opcode, str, level, privileged, flt, minor) \
    name ## _Type0 = constructInplaceMask(opcode, minor) | generateMemBits(0),               \
    name ## _Type1 = constructInplaceMask(opcode, minor) | generateMemBits(1),               \
    name ## _Type2 = constructInplaceMask(opcode, minor) | generateMemBits(2),               \
    name ## _Type3 = constructInplaceMask(opcode, minor) | generateMemBits(3),               \
    name ## _Type4 = constructInplaceMask(opcode, minor) | generateMemBits(4),               \
    name ## _Type5 = constructInplaceMask(opcode, minor) | generateMemBits(5),               \
    name ## _Type6 = constructInplaceMask(opcode, minor) | generateMemBits(6),               \
    name ## _Type7 = constructInplaceMask(opcode, minor) | generateMemBits(7),               \
    name ## _Type8 = constructInplaceMask(opcode, minor) | generateMemBits(8),               \
    name ## _Type9 = constructInplaceMask(opcode, minor) | generateMemBits(9),               \
    name ## _Type10 = constructInplaceMask(opcode, minor) | generateMemBits(10),               \
    name ## _Type11 = constructInplaceMask(opcode, minor) | generateMemBits(11),               \
    name ## _Type12 = constructInplaceMask(opcode, minor) | generateMemBits(12),               \
    name ## _Type13 = constructInplaceMask(opcode, minor) | generateMemBits(13),               \
    name ## _Type14 = constructInplaceMask(opcode, minor) | generateMemBits(14),               \
    name ## _Type15 = constructInplaceMask(opcode, minor) | generateMemBits(15),               \
    name = name ## _Type0,

#define X(name, opcode, str, level, privileged, fmt, flt, minor) fmt(name, opcode, str, level, privileged, flt, minor)
#include "Opcodes.def"
#undef X
#undef REG
#undef MEM
#undef COBR
#undef CTRL
};
constexpr bool isMEMA(ExtendedOpcode opcode) noexcept { return (static_cast<Ordinal>(opcode) & 0x0000'1000) == 0; }
constexpr bool isMEMB(ExtendedOpcode opcode) noexcept { return (static_cast<Ordinal>(opcode) & 0x0000'1000) != 0; }
static_assert(isMEMA(static_cast<ExtendedOpcode>(generateMemBits(0b0000))));
static_assert(isMEMA(static_cast<ExtendedOpcode>(generateMemBits(0b0001))));
static_assert(isMEMA(static_cast<ExtendedOpcode>(generateMemBits(0b0010))));
static_assert(isMEMA(static_cast<ExtendedOpcode>(generateMemBits(0b0011))));
static_assert(isMEMA(static_cast<ExtendedOpcode>(generateMemBits(0b1000))));
static_assert(isMEMA(static_cast<ExtendedOpcode>(generateMemBits(0b1001))));
static_assert(isMEMA(static_cast<ExtendedOpcode>(generateMemBits(0b1010))));
static_assert(isMEMA(static_cast<ExtendedOpcode>(generateMemBits(0b1011))));
static_assert(isMEMB(static_cast<ExtendedOpcode>(generateMemBits(0b0100))));
static_assert(isMEMB(static_cast<ExtendedOpcode>(generateMemBits(0b0101))));
static_assert(isMEMB(static_cast<ExtendedOpcode>(generateMemBits(0b0110))));
static_assert(isMEMB(static_cast<ExtendedOpcode>(generateMemBits(0b0111))));
static_assert(isMEMB(static_cast<ExtendedOpcode>(generateMemBits(0b1100))));
static_assert(isMEMB(static_cast<ExtendedOpcode>(generateMemBits(0b1101))));
static_assert(isMEMB(static_cast<ExtendedOpcode>(generateMemBits(0b1110))));
static_assert(isMEMB(static_cast<ExtendedOpcode>(generateMemBits(0b1111))));

constexpr ExtendedOpcode convertToExtendedOpcode(Ordinal value) noexcept {
    if (isREGFormatOpcode(value)) {
        return static_cast<ExtendedOpcode>(0xFF00'3FE0 & value);
    } else if (isCOBRFormatOpcode(value)) {
        return static_cast<ExtendedOpcode>(0xFF00'2003 & value);
    } else if (isCTRLFormatOpcode(value)) {
        return static_cast<ExtendedOpcode>( 0xFF00'0002 & value);
    } else {
        return static_cast<ExtendedOpcode>(0xFF00'3C00 & value);
    }
}
static_assert(convertToExtendedOpcode(0x0800'0000) == ExtendedOpcode::b);
static_assert(ExtendedOpcode::b == ExtendedOpcode::b_Type0);
static_assert(ExtendedOpcode::b != ExtendedOpcode::b_Type1);
static_assert(ExtendedOpcode::subine != ExtendedOpcode::subine_Type6);
static_assert(ExtendedOpcode::subile_Type14 != ExtendedOpcode::subile_Type13);
static_assert(ExtendedOpcode::subole_Type2 != ExtendedOpcode::subole_Type1);
enum class MEMFormatAddressingMode : uint8_t {
    // MEMA
    AbsoluteOffset = 0b0000,
    // these are not real but for the sake of simplicity we are describing it this way
    Reserved0 = 0b0001,
    Reserved1 = 0b0010,
    Reserved2 = 0b0011,
    RegisterIndirect = 0b0100,
    IPWithDisplacement = 0b0101,
    Reserved3 = 0b0110,
    RegisterIndirectWithIndex = 0b0111,
    RegisterIndirectWithOffset = 0b1000,
    Reserved4 = 0b1001,
    Reserved5 = 0b1010,
    Reserved6 = 0b1011,
    AbsoluteDisplacement = 0b1100,
    RegisterIndirectWithDisplacement = 0b1101,
    IndexWithDisplacement = 0b1110,
    RegisterIndirectWithIndexAndDisplacement = 0b1111,
};
constexpr MEMFormatAddressingMode getAddressingMode(ExtendedOpcode opcode) noexcept {
    if (isMEMA(opcode))  {
        return static_cast<MEMFormatAddressingMode>((static_cast<Ordinal>(opcode) >> 10) & 0b1100);
    } else {
        return static_cast<MEMFormatAddressingMode>((static_cast<Ordinal>(opcode) >> 10) & 0b1111);
    }
}
constexpr bool valid(MEMFormatAddressingMode mode) noexcept {
    using K = MEMFormatAddressingMode;
    switch (mode) {
        case K::AbsoluteOffset:
        case K::RegisterIndirect:
        case K::IPWithDisplacement:
        case K::RegisterIndirectWithIndex:
        case K::RegisterIndirectWithOffset:
        case K::AbsoluteDisplacement:
        case K::RegisterIndirectWithDisplacement:
        case K::IndexWithDisplacement:
        case K::RegisterIndirectWithIndexAndDisplacement:
            return true;
        default:
            return false;
    }
}
constexpr bool usesOptionalDisplacement(MEMFormatAddressingMode mode) noexcept {
    using AddressingMode = MEMFormatAddressingMode;
    switch (mode) {
        case AddressingMode::IPWithDisplacement:
        case AddressingMode::AbsoluteDisplacement:
        case AddressingMode::RegisterIndirectWithDisplacement:
        case AddressingMode::IndexWithDisplacement:
        case AddressingMode::RegisterIndirectWithIndexAndDisplacement:
            return true;
        default:
            return false;
    }
}
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b0000)}) == MEMFormatAddressingMode::AbsoluteOffset);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b0001)}) == MEMFormatAddressingMode::AbsoluteOffset);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b0010)}) == MEMFormatAddressingMode::AbsoluteOffset);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b0011)}) == MEMFormatAddressingMode::AbsoluteOffset);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b0100)}) == MEMFormatAddressingMode::RegisterIndirect);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b0101)}) == MEMFormatAddressingMode::IPWithDisplacement);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b0110)}) == MEMFormatAddressingMode::Reserved3);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b0111)}) == MEMFormatAddressingMode::RegisterIndirectWithIndex);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b1000)}) == MEMFormatAddressingMode::RegisterIndirectWithOffset);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b1001)}) == MEMFormatAddressingMode::RegisterIndirectWithOffset);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b1010)}) == MEMFormatAddressingMode::RegisterIndirectWithOffset);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b1011)}) == MEMFormatAddressingMode::RegisterIndirectWithOffset);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b1100)}) == MEMFormatAddressingMode::AbsoluteDisplacement);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b1101)}) == MEMFormatAddressingMode::RegisterIndirectWithDisplacement);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b1110)}) == MEMFormatAddressingMode::IndexWithDisplacement);
static_assert(getAddressingMode(ExtendedOpcode{generateMemBits(0b1111)}) == MEMFormatAddressingMode::RegisterIndirectWithIndexAndDisplacement);
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b0000)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b0001)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b0010)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b0011)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b0100)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b0101)})) );
static_assert(!valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b0110)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b0111)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b1000)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b1001)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b1010)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b1011)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b1100)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b1101)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b1110)})) );
static_assert(valid(getAddressingMode(ExtendedOpcode{generateMemBits(0b1111)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b0000)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b0001)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b0010)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b0011)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b0100)})) );
static_assert(usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b0101)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b0110)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b0111)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b1000)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b1001)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b1010)})) );
static_assert(!usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b1011)})) );
static_assert(usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b1100)})) );
static_assert(usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b1101)})) );
static_assert(usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b1110)})) );
static_assert(usesOptionalDisplacement(getAddressingMode(ExtendedOpcode{generateMemBits(0b1111)})) );
#endif //SIM960_EXTENDEDOPCODE_H
