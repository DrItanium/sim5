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
    auto testScanbyteOperation = [this]() {
        ac_.arith.conditionCode = 0;
        auto& g0 = getGPR(0);
        auto& g1 = getGPR(1);
        auto scanbyte_reference = [](Ordinal src1, Ordinal src2) noexcept {
            return ((src1 & 0x000000FF) == (src2 & 0x000000FF)) ||
                   ((src1 & 0x0000FF00) == (src2 & 0x0000FF00)) ||
                   ((src1 & 0x00FF0000) == (src2 & 0x00FF0000)) ||
                   ((src1 & 0xFF000000) == (src2 & 0xFF000000));
        };
        auto exec = [this, scanbyte_reference](Ordinal a0, Ordinal a1) noexcept {
            ac_.arith.conditionCode = 0;
            auto r0 = scanbyte_reference(a0, a1);
            scanbyte(a0, a1);
            switch(ac_.getConditionCode()) {
                case 0b000:
                    return !r0;
                case 0b010:
                    return r0;
                default: // we got something else
                    return false;
            }
        };
        Ordinal rand0 = static_cast<Ordinal>(random());
        return exec(0, 0) &&
               exec(0x11AB1100, 0x00AB0011) &&
               exec(0x00AB0011, 0x11AB1100) &&
               exec(rand0, rand0) &&
               exec(random(), random()) &&
               exec(0xFFFF'FFFF, 0xFFFF'FF00) &&
               exec(0x01010101, 0x02020202);
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
                        testScanbyteOperation,
                        (makeOrdinalOperation([this](Ordinal bitpos, Ordinal src, Ordinal) {
                            // implement it separately for comparison purposes
                                                  ac_.arith.conditionCode = random() & 0b111;
                                                  if ((ac_.getConditionCode() & 0b010) == 0) {
                                                      return src & (~computeBitPosition(bitpos & 0b11111));
                                                  } else {
                                                      return src | computeBitPosition(bitpos & 0b11111);
                                                  }
                                              },
                                              [this](auto& dest, auto src1, auto src2) { return alterbit(dest, src1, src2); },
                                              "alterbit")),
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 + src1; },
                                              [this](auto& dest, auto src1, auto src2) { return addi(dest, src1, src2); },
                                              "addi")),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 + src1; },
                                              [this](auto& dest, auto src1, auto src2) { return addo(dest, src1, src2); },
                                              ("addo"))),
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 - src1; },
                                              [this](auto& dest, auto src1, auto src2) { return subi(dest, src1, src2); },
                                              ("subi"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 - src1; },
                                              [this](auto& dest, auto src1, auto src2) { return subo(dest, src1, src2); },
                                              ("subo"))),
                        (genericIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 / src1; },
                                                 [this](auto& dest, auto src1, auto src2) { return divi(dest, src1, src2); },
                                                 ("divi"),
                                                 doRandomDisallow0, doRandom)),
                        (genericOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 / src1; },
                                                 [this](auto& dest, auto src1, auto src2) { return divo(dest, src1, src2); },
                                                 ("divo"),
                                                 doRandomDisallow0, doRandom)),
                        (genericIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 % src1; },
                                                 [this](auto& dest, auto src1, auto src2) { return remi(dest, src1, src2); },
                                                 ("remi"),
                                                 doRandomDisallow0, doRandom)),
                        (genericOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 % src1; },
                                                 [this](auto& dest, auto src1, auto src2) { return remo(dest, src1, src2); },
                                                 ("remo"),
                                                 doRandomDisallow0, doRandom)),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 | (~src1); },
                                              [this](auto& dest, auto src1, auto src2) { return ornot(dest, src1, src2); },
                                              ("ornot"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (~src2) | src1; },
                                              [this](auto& dest, auto src1, auto src2) { return notor(dest, src1, src2); },
                                              ("notor"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 & (~src1); },
                                              [this](auto& dest, auto src1, auto src2) { return andnot(dest, src1, src2); },
                                              ("andnot"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (~src2) & src1; },
                                              [this](auto& dest, auto src1, auto src2) { return notand(dest, src1, src2); },
                                              ("notand"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 & src1); },
                                              [this](auto& dest, auto src1, auto src2) { return nand(dest, src1, src2); },
                                              ("nand"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 | src1); },
                                              [this](auto& dest, auto src1, auto src2) { return nor(dest, src1, src2); },
                                              ("nor"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 | src1); },
                                              [this](auto& dest, auto src1, auto src2) { return orOperation(dest, src1, src2); },
                                              ("or"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 & src1); },
                                              [this](auto& dest, auto src1, auto src2) { return andOperation(dest, src1, src2); },
                                              ("and"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return (src2 ^ src1); },
                                              [this](auto& dest, auto src1, auto src2) { return xorOperation(dest, src1, src2); },
                                              ("xor"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return ~(src2 ^ src1); },
                                              [this](auto& dest, auto src1, auto src2) { return xnor(dest, src1, src2); },
                                              ("xnor"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return rotateOperation(src2, src1); },
                                              [this](auto& dest, auto src1, auto src2) { return rotate(dest, src1, src2); },
                                              ("rotate"))),
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return (src2 >> src1); },
                                              [this](auto& dest, auto src1, auto src2) { return shri(dest, src1, src2); },
                                              ("shri"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src1 < 32 ? (src2 << src1) : 0; },
                                              [this](auto& dest, auto src1, auto src2) { return shlo(dest, src1, src2); },
                                              ("shlo"))),
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return (src2 << src1); },
                                              [this](auto& dest, auto src1, auto src2) { return shli(dest, src1, src2); },
                                              ("shli"))),
                        (makeIntegerOperation([](Integer src1, Integer src2, Integer) { return src2 * src1; },
                                              [this](auto& dest, auto src1, auto src2) { return muli(dest, src1, src2); },
                                              ("muli"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal) { return src2 * src1; },
                                              [this](auto& dest, auto src1, auto src2) { return mulo(dest, src1, src2); },
                                              ("mulo"))),
                        (makeOrdinalOperation([](Ordinal src1, Ordinal src2, Ordinal dest) { return ::modify(src1, src2, dest); },
                                              [this](auto& dest, auto src1, auto src2) { return modify(dest, src1, src2); },
                                              ("modify")))
    ) && runNonPortableSelfTests();
}
