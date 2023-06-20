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
// Created by jwscoggins on 6/12/23.
//

#include "Disassembly.h"
#include <sstream>
#include <string>

namespace {
    std::string getOpcodeMnemonic(Opcodes code) {
        switch (code) {
#define X(name, code, str, level, privileged, fmt) case Opcodes:: name : return str ;
#include "Opcodes.def"
#undef X
            default:
                return "unknown/unimplemented";
        }
    }
    std::string getOpcodeMnemonic(const Register& rawValue) {
        return getOpcodeMnemonic(rawValue.getOpcode());
    }
    const std::string RegisterDecodeTable[32] {
            "pfp",
            "sp",
            "rip",
#define X(index) "r" #index
            X(3),
            X(4),
            X(5),
            X(6),
            X(7),
            X(8),
            X(9),
            X(10),
            X(11),
            X(12),
            X(13),
            X(14),
            X(15),
#undef X
#define X(index) "g" #index
            X(0),
            X(1),
            X(2),
            X(3),
            X(4),
            X(5),
            X(6),
            X(7),
            X(8),
            X(9),
            X(10),
            X(11),
            X(12),
            X(13),
            X(14),
            X(15),
#undef X
    };
    std::string translateRegisterIndex(uint8_t index) {
        return RegisterDecodeTable[index & 0b11111];
    }
}
std::string
disassembleInstruction(Address addr, const Register& reg) {
    std::stringstream ss;
    ss << "@ 0x" << std::hex << addr << ": " << getOpcodeMnemonic(reg);
    auto str = ss.str();
    return str;
}

namespace {
    void
    disassembleInstruction(Address address, std::ostream& out, const CTRLInstruction& inst) {
        switch (inst.getOpcode()) {
            case Opcodes::ret:
            case Opcodes::faultno:
            case Opcodes::faulte:
            case Opcodes::faultg:
            case Opcodes::faultge:
            case Opcodes::faultl:
            case Opcodes::faultne:
            case Opcodes::faultle:
            case Opcodes::faulto:
                // no arguments
                break;
            default:
                out << " 0x" << std::hex << inst.getDisplacement();
                break;
        }
    }
    void
    disassembleInstruction(Address address, std::ostream& out, const REGInstruction& inst) {

    }
    void
    disassembleInstruction(Address address, std::ostream& out, const COBRInstruction& inst) {

    }
    void
    disassembleInstruction(Address address, std::ostream& out, const MEMInstruction& inst) {

    }
}

std::string
disassembleInstruction(Address address, Ordinal lower, Integer upper) {
    std::ostringstream ss;
    ss << getOpcodeMnemonic(Register{lower});
    if (isCTRL(lower)) {
        disassembleInstruction(address, ss, CTRLInstruction{lower});
    } else if (isCOBR(lower)) {
        disassembleInstruction(address, ss, COBRInstruction{lower});
    } else if (isREGFormat(lower)) {
        disassembleInstruction(address, ss, REGInstruction{lower});
    } else {
        // is mem operation
        disassembleInstruction(address, ss, MEMInstruction{lower, upper});
    }
    auto str = ss.str();
    return str;
}

std::string
disassembleInstruction(Address address, LongOrdinal value) {
    LongRegister temp{value};
    return disassembleInstruction(address, temp.getValue<Ordinal>(0), temp.getValue<Integer>(1));
}
