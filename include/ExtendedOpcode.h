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
enum class ExtendedOpcode : uint32_t {
#define COBR(name, opcode, str, level, privileged, flt, minor) \



#define MEM(name, opcode, str, level, privileged, flt, minor)
#define CTRL(name, opcode, str, level, privileged, flt, minor)
#define REG(name, opcode, str, level, privileged, flt, minor)
#define X(name, opcode, str, level, privileged, fmt, flt, minor) fmt(name, opcode, str, level, privileged, flt, minor)
#undef X
#undef REG
#undef MEM
#undef COBR
#undef CTRL
};

#endif //SIM960_EXTENDEDOPCODE_H
