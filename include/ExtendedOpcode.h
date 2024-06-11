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
constexpr bool isREGFormatOpcode(uint8_t value) noexcept {
    return (value >= 0x40) && (value < 0x80);
}
constexpr bool isMEMFormatOpcode(uint8_t value) noexcept {
    return (value >= 0x80);
}
constexpr bool isCOBRFormatOpcode(uint8_t value) noexcept {
    return (value >= 0x20) && (value < 0x40);
}
constexpr bool isCTRLFormatOpcode(uint8_t value) noexcept {
    return (value < 0x20);
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
constexpr uint32_t generateRegBits(bool s1, bool s2, bool m1, bool m2, bool m3) noexcept {
    uint32_t result = 0;
    result |= s1 ? 0b000'0000'01'00000 : 0;
    result |= s2 ? 0b000'0000'01'00000 : 0;
    result |= m1 ? 0b001'0000'00'00000 : 0;
    result |= m2 ? 0b010'0000'00'00000 : 0;
    result |= m3 ? 0b100'0000'00'00000 : 0;
    return result;
}
constexpr uint32_t generateCobrBits(bool m1, bool s2, bool t) noexcept {
    uint32_t result = 0;
    result |= m1 ? 0b10'0000000000'00 : 0;
    result |= s2 ? 0b00'0000000000'01 : 0;
    result |= t  ? 0b00'0000000000'10 : 0;
    return result;
}
constexpr uint32_t generateCobrBits(uint8_t value) noexcept {
    return generateCobrBits(
            value & 0b001,
            value & 0b010,
            value & 0b100
    );
}
constexpr uint32_t generateRegBits(uint8_t value) noexcept {
    return generateRegBits(
            value & 0b00001,
            value & 0b00010,
            value & 0b00100,
            value & 0b01000,
            value & 0b10000
    );
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
    name ## _Type1 = constructInplaceMask(opcode, minor) | 0b10,

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

#define MEM(name, opcode, str, level, privileged, flt, minor)

#define X(name, opcode, str, level, privileged, fmt, flt, minor) fmt(name, opcode, str, level, privileged, flt, minor)
#undef X
#undef REG
#undef MEM
#undef COBR
#undef CTRL
};
#endif //SIM960_EXTENDEDOPCODE_H
