// sim5
// Copyright (c) 2022-2024, Joshua Scoggins
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
// Created by jwscoggins on 6/9/24.
//

#ifndef SIM960_PROCESSMANAGEMENT_H
#define SIM960_PROCESSMANAGEMENT_H

#include "Types.h"

struct [[gnu::packed]] ProcessControlBlock {
    LongOrdinal queueRecord;
    Ordinal receiveMessage;
    SegmentSelector dispatchPort;
    Ordinal residualTimeSlice;
    Ordinal processControls;
    Ordinal lock : 8;
    Ordinal processNotice : 24;
    Ordinal traceControls;
    Ordinal _unused0[4];
    SegmentSelector region0;
    SegmentSelector region1;
    SegmentSelector region2;
    Ordinal arithmeticControls;
    Ordinal _unused1;
    Ordinal nextTimeSlice;
    LongOrdinal executionTime;
    Ordinal resumptionRecord[12];
    Ordinal floatingPointRegisters[12];
    Ordinal globalRegisters[16];
};





#endif //SIM960_PROCESSMANAGEMENT_H
