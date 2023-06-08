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
// Created by jwscoggins on 6/8/23.
//
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include "Types.h"
#include "Core.h"
#include "BinaryOperations.h"
bool
Core::performSelfTest() noexcept {
    auto clearRegisters = [this]() {
        for (int i = 0; i < 32; ++i) {
            getGPR(i).clear();
        }
    };
    // test different instructions to see if they are working correctly
    auto testRegisters = [this]() {
        for (int i = 0; i < 32; ++i) {
            auto& temporary = getGPR(i);
            auto randomValue = static_cast<Ordinal>(random());
            auto randomInteger = static_cast<Integer>(random());
            temporary.template setValue<Ordinal>(randomValue);
            if (temporary.template getValue<Ordinal>() != randomValue) {
                return false;
            }
            temporary.template setValue<Integer>(randomInteger);
            if (temporary.template getValue<Integer>() != randomInteger) {
                return false;
            }
        }
        return true;
    };
    // test move operations
    // first mov
    auto testMoveOperations = [this]() {
        auto& g0 = getGPR(0);
        auto& g1 = getGPR(1);
        auto randomSourceValue = static_cast<Ordinal>(random());
        g0.template setValue<Ordinal>(randomSourceValue);
        g1.template setValue<Ordinal>(0xFFFF'FFFF);
        g1.template setValue<Ordinal>(g0.template getValue<Ordinal>());
        if (g1.template getValue<Ordinal>() != g0.template getValue<Ordinal>()) {
            return false;
        }
        auto randomSourceValue2 = static_cast<Ordinal>(random());
        auto& gl0 = getGPR(0, TreatAsLongRegister{});
        auto& gl1 = getGPR(2, TreatAsLongRegister{});
        gl0[0] = randomSourceValue;
        gl0[1] = randomSourceValue2;
        if (static_cast<Ordinal>(gl0[0]) != randomSourceValue) {
            return false;
        }
        if (static_cast<Ordinal>(gl0[1]) != randomSourceValue2) {
            return false;
        }
        gl1 = gl0;
        if (static_cast<Ordinal>(gl1[0]) != randomSourceValue) {
            return false;
        }
        if (static_cast<Ordinal>(gl1[1]) != randomSourceValue2) {
            return false;
        }
        return true;
    };
    auto makeGenericOperation = [this](auto maker, auto doIt, auto converter, auto name, auto genSrc1, auto genSrc2) {
        return [this, maker, doIt, converter, name, genSrc1, genSrc2](ByteOrdinal gpr0 = random() & 0b11111,
                                                                      ByteOrdinal gpr1 = random() & 0b11111,
                                                                      ByteOrdinal gpr2 = random() & 0b11111) -> bool {
            auto rs0 = converter(genSrc1());
            auto rs1 = converter(genSrc2());
            auto& src1 = getGPR(gpr0);
            src1 = rs0;
            auto& src2 = getGPR(gpr1);
            src2 = rs1;
            auto& dest = getGPR(gpr2);
            auto rs2 = maker(converter(src1), converter(src2), converter(dest));
            doIt(dest, converter(src1), converter(src2));
            if (converter(dest) != rs2) {
                return false;
            }
            return true;
        };
    };
    auto genericIntegerOperation = [this, makeGenericOperation](auto maker, auto doIt, auto name, auto genSrc1, auto genSrc2) {
        return makeGenericOperation(maker, doIt, [](auto value) { return static_cast<Integer>(value); }, name, genSrc1, genSrc2);
    };
    auto genericOrdinalOperation = [this, makeGenericOperation](auto maker, auto doIt, auto name, auto genSrc1, auto genSrc2) {
        return makeGenericOperation(maker, doIt, [](auto value) { return static_cast<Ordinal>(value); }, name, genSrc1, genSrc2);
    };
    auto makeIntegerOperation = [this, genericIntegerOperation](auto maker, auto doIt, auto name) {
        return genericIntegerOperation(maker, doIt, name, doRandom, doRandom);
    };
    auto makeOrdinalOperation = [this, genericOrdinalOperation](auto maker, auto doIt, auto name) {
        return genericOrdinalOperation(maker, doIt, name, doRandom, doRandom);
    };
    auto runTest = [this, clearRegisters](auto fn) {
        return makeTestRunner(fn, clearRegisters, clearRegisters);
    };
    return runTestCases(runTest,
                        testMoveOperations,
                        testRegisters,
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 + src1; },
                                              [this](auto& dest, auto src1, auto src2) { return addi(dest, src1, src2); },
                                              F("addi"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 + src1; },
                                              [this](auto& dest, auto src1, auto src2) { return addo(dest, src1, src2); },
                                              F("addo"))),
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 - src1; },
                                              [this](auto& dest, auto src1, auto src2) { return subi(dest, src1, src2); },
                                              F("subi"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 - src1; },
                                              [this](auto& dest, auto src1, auto src2) { return subo(dest, src1, src2); },
                                              F("subo"))),
                        (genericIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 / src1; },
                                                 [this](auto& dest, auto src1, auto src2) { return divi(dest, src1, src2); },
                                                 F("divi"),
                                                 doRandomDisallow0, doRandom)),
                        (genericOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 / src1; },
                                                 [this](auto& dest, auto src1, auto src2) { return divo(dest, src1, src2); },
                                                 F("divo"),
                                                 doRandomDisallow0, doRandom)),
                        (genericIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 % src1; },
                                                 [this](auto& dest, auto src1, auto src2) { return remi(dest, src1, src2); },
                                                 F("remi"),
                                                 doRandomDisallow0, doRandom)),
                        (genericOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 % src1; },
                                                 [this](auto& dest, auto src1, auto src2) { return remo(dest, src1, src2); },
                                                 F("remo"),
                                                 doRandomDisallow0, doRandom)),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 & src1); },
                                              [this](auto& dest, auto src1, auto src2) { return nand(dest, src1, src2); },
                                              F("nand"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 | src1); },
                                              [this](auto& dest, auto src1, auto src2) { return nor(dest, src1, src2); },
                                              F("nor"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 | src1); },
                                              [this](auto& dest, auto src1, auto src2) { return orOperation(dest, src1, src2); },
                                              F("or"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 & src1); },
                                              [this](auto& dest, auto src1, auto src2) { return andOperation(dest, src1, src2); },
                                              F("and"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 ^ src1); },
                                              [this](auto& dest, auto src1, auto src2) { return xorOperation(dest, src1, src2); },
                                              F("xor"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 ^ src1); },
                                              [this](auto& dest, auto src1, auto src2) { return xnor(dest, src1, src2); },
                                              F("xnor"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return rotateOperation(src2, src1); },
                                              [this](auto& dest, auto src1, auto src2) { return rotate(dest, src1, src2); },
                                              F("rotate"))),
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return (src2 >> src1); },
                                              [this](auto& dest, auto src1, auto src2) { return shri(dest, src1, src2); },
                                              F("shri"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src1 < 32 ? (src2 << src1) : 0; },
                                              [this](auto& dest, auto src1, auto src2) { return shlo(dest, src1, src2); },
                                              F("shlo"))),
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return (src2 << src1); },
                                              [this](auto& dest, auto src1, auto src2) { return shli(dest, src1, src2); },
                                              F("shli"))),
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 * src1; },
                                              [this](auto& dest, auto src1, auto src2) { return muli(dest, src1, src2); },
                                              F("muli"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 * src1; },
                                              [this](auto& dest, auto src1, auto src2) { return mulo(dest, src1, src2); },
                                              F("mulo"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal dest) { return ::modify(src1, src2, dest); },
                                              [this](auto& dest, auto src1, auto src2) { return modify(dest, src1, src2); },
                                              F("modify")))
    ) && runNonPortableSelfTests();
}
