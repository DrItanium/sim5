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
#ifdef ARDUINO
#include "SBCoreArduino.h"
SBCoreArduino::SBCoreArduino() : Parent() {


}
void
SBCoreArduino::generateFault(FaultType ) {
    //std::cout << "FAULT GENERATED AT 0x" << std::hex << ip_.getOrdinal() << "! HALTING!" << std::endl;
    haltExecution();
}

void
SBCoreArduino::installToMemory(Address loc, Ordinal value) {
    auto alignedAddress = ((64_MB -1) & loc) >> 2;
    //memory_[alignedAddress].setOrdinalValue(value);
}
void
SBCoreArduino::installToMemory(Address loc, ByteOrdinal value) {
    auto alignedAddress = ((64_MB - 1) & loc) >> 2;
    auto offset = loc & 0b11;
    //memory_[alignedAddress].setByteOrdinal(value, offset);
}
void
SBCoreArduino::installBlockToMemory(Address base, Ordinal curr) noexcept  {
    installToMemory(base, curr);
}

SBCoreArduino::~SBCoreArduino() { }
#endif

