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
// Created by jwscoggins on 8/18/21.
//

#ifndef SIM3_REGISTER_H
#define SIM3_REGISTER_H
#include "Types.h"
union Register {
public:
    constexpr explicit Register(Ordinal value = 0) noexcept : ord_(value) { }
    constexpr bool getMostSignificantBit() const noexcept { return ord_ & 0x8000'0000; }
    constexpr auto getOrdinal() const noexcept {
        return ord_;
    }
    constexpr auto getInteger() const noexcept { return integer_; }
    constexpr auto getShortOrdinal(int which = 0) const noexcept { return sords_[which&0b01]; }
    constexpr auto getShortInteger(int which = 0) const noexcept { return sints_[which&0b01]; }
    constexpr auto getByteOrdinal(int which = 0) const noexcept { return bords_[which&0b11]; }
    constexpr auto getByteInteger(int which = 0) const noexcept { return bints_[which&0b11]; }
    void setOrdinal(Ordinal value) noexcept {
        ord_ = value;
    }
    void setInteger(Integer value) noexcept {
        integer_ = value;
    }
    void setShortOrdinal(ShortOrdinal value) noexcept { setOrdinal(value); }
    void setShortInteger(ShortInteger value) noexcept { setInteger(value); }
    void setByteOrdinal(ByteOrdinal value) noexcept { setOrdinal(value); }
    void setByteInteger(ByteInteger value) noexcept { setInteger(value); }
    void setShortOrdinal(ShortOrdinal value, int which) noexcept { sords_[which&0b01] = value; }
    void setShortInteger(ShortInteger value, int which) noexcept { sints_[which&0b01] = value; }
    void setByteOrdinal(ByteOrdinal value, int which) noexcept { bords_[which&0b11] = value; }
    void setByteInteger(ByteInteger value, int which) noexcept { bints_[which&0b11] = value; }
    void increment(Integer advance) noexcept { integer_ += advance; }
    void increment(Ordinal advance) noexcept { ord_ += advance; }
    void decrement(Integer advance) noexcept { integer_ -= advance; }
    void decrement(Ordinal advance) noexcept { ord_ -= advance; }
#ifdef NUMERICS_ARCHTIECTURE
    constexpr auto getReal() const noexcept { return real_; }
void setReal(Real value) noexcept { real_ = value; }
#endif
private:
    Ordinal ord_ = 0;
    Integer integer_;
    ShortOrdinal sords_[sizeof(Ordinal)/sizeof(ShortOrdinal)];
    ShortInteger sints_[sizeof(Integer)/sizeof(ShortInteger)];
    ByteOrdinal bords_[sizeof(Ordinal)/sizeof(ByteOrdinal)];
    ByteInteger bints_[sizeof(Integer)/sizeof(ByteInteger)];
#ifdef NUMERICS_ARCHITECTURE
    Real real_;
#endif
};

class PreviousFramePointer {
public:
    explicit PreviousFramePointer(Register& targetRegister) : reg_(targetRegister) {}
    constexpr bool getPrereturnTraceFlag() const noexcept { return (reg_.getOrdinal() & 0b1000); }
    void enablePrereturnTraceFlag() const noexcept { reg_.setOrdinal(reg_.getOrdinal() | 0b1000); }
    void disablePrereturnTraceFlag() const noexcept { reg_.setOrdinal(reg_.getOrdinal() & ~static_cast<Ordinal>(0b1000)); }
    void setPrereturnTraceFlag(bool value) const noexcept {
        value ? enablePrereturnTraceFlag() : disablePrereturnTraceFlag();
    }
    constexpr Ordinal getReturnType() const noexcept { return reg_.getOrdinal() & 0b111; }
    void setReturnType(Ordinal value) noexcept { reg_.setOrdinal((reg_.getOrdinal() & ~0b111) | (value & 0b111)); }
    constexpr Ordinal getAddress() const noexcept {
        // according to the i960Sx manual, the lowest 6 bits are ignored but the lowest 4 bits are always confirmed
        return reg_.getOrdinal() & ~0b1'111;
    }
    void setAddress(Ordinal value) noexcept {
        auto lowerBits = reg_.getOrdinal() & 0b1'111;
        reg_.setOrdinal((value & ~0b1'111) | lowerBits);
    }
    constexpr auto getWhole() const noexcept { return reg_.getOrdinal(); }
    void setWhole(Ordinal value) noexcept { reg_.setOrdinal(value); }
private:
    Register& reg_;
};
union TraceControls {
public:
    constexpr explicit TraceControls(Ordinal value = 0) noexcept : ord_(value) { }
    constexpr auto getValue() const noexcept { return ord_; }
    void setValue(Ordinal value) noexcept { ord_ = value; }
#define X(name, field, type) \
constexpr type get ## name () const noexcept { return field ; } \
void set ## name( type value) noexcept { field  = value ; }
#define Y(particleName, particleField)  \
X(particleName ## TraceMode, particleField ## TraceMode, bool); \
X(particleName ## TraceEvent, particleField ## TraceEvent, bool)
    Y(Instruction, instruction);
    Y(Branch, branch);
    Y(Call, call);
    Y(Return, return);
    Y(Prereturn, prereturn);
    Y(Supervisor, supervisor);
    Y(Breakpoint, breakpoint);
#undef Y
#undef X
    /**
     * Reads and modifies contents of this object
     */
    Ordinal modify(Ordinal mask, Ordinal src) noexcept;
private:
    Ordinal ord_ = 0;
    struct {
        Ordinal unused0 : 1; // 0
        Ordinal instructionTraceMode : 1; // 1
        Ordinal branchTraceMode : 1; // 2
        Ordinal callTraceMode : 1; // 3
        Ordinal returnTraceMode : 1; // 4
        Ordinal prereturnTraceMode : 1; // 5
        Ordinal supervisorTraceMode : 1; // 6
        Ordinal breakpointTraceMode : 1; // 7
        Ordinal unused1 : 9; // 8, 9, 10, 11, 12, 13, 14, 15, 16
        Ordinal instructionTraceEvent : 1; // 17
        Ordinal branchTraceEvent : 1; // 18
        Ordinal callTraceEvent : 1; // 19
        Ordinal returnTraceEvent : 1; // 20
        Ordinal prereturnTraceEvent : 1; // 21
        Ordinal supervisorTraceEvent : 1; // 22
        Ordinal breakpointTraceEvent : 1; // 23
        Ordinal unused2 : 8;
    };
};
static_assert (sizeof(TraceControls) == sizeof(Ordinal), "ArithmeticControls must be the width of a single Ordinal");
union ProcessControls {
public:
    constexpr explicit ProcessControls(Ordinal value = 0) noexcept : ord_(value) { }
    constexpr auto getValue() const noexcept { return ord_; }
    void setValue(Ordinal value) noexcept { ord_ = value; }
    constexpr bool inSupervisorMode() const noexcept { return getExecutionMode(); }
    constexpr bool inUserMode() const noexcept { return !getExecutionMode(); }

#define X(name, field, type) \
constexpr type get ## name () const noexcept { return field ; } \
void set ## name( type value) noexcept { field  = value ; }
    X(TraceEnable, traceEnable, bool);
    X(ExecutionMode, executionMode, bool);
    X(Resume, resume, bool);
    X(TraceFaultPending, traceFaultPending, bool);
    X(State, state, bool);
    X(Priority, priority, Ordinal);
    X(InternalState, internalState, Ordinal);
#undef X
    /**
     * Reads and modifies contents of this object
     */
    Ordinal modify(Ordinal mask, Ordinal src) noexcept;
private:
    Ordinal ord_ = 0;
    struct {
        Ordinal traceEnable : 1; // 0
        Ordinal executionMode : 1; // 1
        Ordinal unused0 : 7;  // 2, 3, 4, 5, 6, 7, 8
        Ordinal resume : 1; // 9
        Ordinal traceFaultPending : 1; // 10
        Ordinal unused1 : 2; // 11, 12
        Ordinal state : 1; // 13
        Ordinal unused2 : 2; // 14, 15
        Ordinal priority : 5; // 16, 17, 18, 19, 20
        Ordinal internalState : 11; // 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
    };
};
static_assert (sizeof(ProcessControls) == sizeof(Ordinal), "ArithmeticControls must be the width of a single Ordinal");


union ArithmeticControls {
public:
    constexpr explicit ArithmeticControls(Ordinal value = 0) noexcept : ord_(value) { }
    constexpr auto getValue() const noexcept { return ord_; }
    constexpr bool getOverflowBit() const noexcept { return getConditionCode() & 0b001; }
    constexpr bool getCarryBit() const noexcept { return getConditionCode() & 0b001; }
    void setCarryBit(bool value) noexcept {
        if (value) {
            setConditionCode(getConditionCode() | 0b010);
        } else {
            setConditionCode(getConditionCode() & 0b101);
        }
    }
    void setOverflowBit(bool value) noexcept {
        if (value) {
            setConditionCode(getConditionCode() | 0b001);
        } else {
            setConditionCode(getConditionCode() & 0b110);
        }
    }
    void setValue(Ordinal value) noexcept { ord_ = value; }
#define X(name, field, type) \
constexpr type get ## name () const noexcept { return field ; } \
void set ## name( type value) noexcept { field  = value ; }
    X(ConditionCode, conditionCode, Ordinal);
    X(NoImpreciseFaults, noImpreciseFaults, bool);
#ifdef NUMERICS_ARCHITECTURE
    X(ArithmeticStatus, arithmeticStatus, Ordinal);
    X(FloatingPointRoundingControl, floatingPointRoundingControl, Ordinal);
    X(FloatingPointNormalizingMode, floatingPointNormalizingMode, bool);
#endif
#define Y(name, field) \
X(name ## Flag , field ## Flag, bool); \
X(name ## Mask , field ## Mask, bool);
    Y(IntegerOverflow, integerOverflow);
#ifdef NUMERICS_ARCHITECTURE
    Y(FloatingOverflow, floatingOverflow);
    Y(FloatingUnderflow, floatingUnderflow);
    Y(FloatingInvalidOp, floatingInvalidOp);
    Y(FloatingZeroDivide, floatingZeroDivide);
    Y(FloatingInexact, floatingInexact);
#endif
#undef Y
#undef X
public:
    template<uint8_t value>
    constexpr bool conditionCodeIs() const noexcept {
        if constexpr ((value & 0b111) == 0b000) {
            return (getConditionCode() & (value & 0b111)) == 0;
        } else {
            return (getConditionCode() & (value & 0b111)) != 0;
        }
    };
    /**
     * Reads and modifies contents of this object
     */
    Ordinal modify(Ordinal mask, Ordinal src) noexcept;
private:
    Ordinal ord_ = 0;
    struct {
        Ordinal conditionCode : 3;
#ifdef NUMERICS_ARCHITECTURE
        Ordinal arithmeticStatus : 4;
        Ordinal unused0 : 1;
#else
        Ordinal unused0 : 5;
#endif // end defined(NUMERICS_ARCHITECTURE)
        Ordinal integerOverflowFlag : 1;
        Ordinal unused1 : 3;
        Ordinal integerOverflowMask : 1;
        Ordinal unused2 : 2;
        Ordinal noImpreciseFaults : 1;
#ifdef NUMERICS_ARCHITECTURE
        Ordinal floatingOverflowFlag : 1;
        Ordinal floatingUnderflowFlag : 1;
        Ordinal floatingInvalidOpFlag : 1;
        Ordinal floatingZeroDivideFlag : 1;
        Ordinal floatingInexactFlag : 1;
        Ordinal unused3 : 3;
        Ordinal floatingOverflowMask : 1;
        Ordinal floatingUnderflowMask : 1;
        Ordinal floatingInvalidOpMask : 1;
        Ordinal floatingZeroDivideMask : 1;
        Ordinal floatingInexactMask : 1;
        Ordinal floatingPointNormalizingMode : 1;
        Ordinal floatingPointRoundingControl : 2;
#else
        Ordinal unused3 : 16;
#endif // end defined(NUMERICS_ARCHITECTURE)
    };
};
static_assert (sizeof(ArithmeticControls) == sizeof(Ordinal), "ArithmeticControls must be the width of a single Ordinal");

constexpr Ordinal modify(Ordinal mask, Ordinal src, Ordinal srcDest) noexcept {
    return (src & mask) | (srcDest & ~mask);
}



union DoubleRegister {
public:
    constexpr explicit DoubleRegister(LongOrdinal value = 0) noexcept : lord_(value) { }
    constexpr DoubleRegister(Ordinal lower, Ordinal upper) noexcept : parts_{lower, upper} { }
    constexpr auto getLongOrdinal() const noexcept { return lord_; }
    constexpr auto getLongInteger() const noexcept { return lint_; }
    constexpr auto getOrdinal(int which = 0) const noexcept { return parts_[which & 0b01]; }
    void setLongOrdinal(LongOrdinal value) noexcept {
        lord_ = value;
    }
    void setLongInteger(LongInteger value) noexcept { lint_ = value; }
    void setOrdinal(Ordinal value, int which = 0) noexcept { parts_[which & 0b01] = value; }

#ifdef NUMERICS_ARCHITECTURE
    constexpr auto getLongReal() const noexcept { return lreal_; }
    void setLongReal(LongReal value) noexcept { lreal_ = value; }
#endif
    void performMov(const DoubleRegister& other) noexcept {
        lord_ = other.lord_;
    }
private:
    LongOrdinal lord_ = 0;
    LongInteger lint_;
    Ordinal parts_[sizeof(LongOrdinal)/ sizeof(Ordinal)];
#ifdef NUMERICS_ARCHITECTURE
    LongReal lreal_;
#endif
};

union TripleRegister {
public:
    constexpr explicit TripleRegister(Ordinal a = 0, Ordinal b = 0, Ordinal c = 0) noexcept : parts_{a, b, c, 0}{ }
    constexpr auto getOrdinal(int which = 0) const noexcept { return parts_[which % 3]; } // very expensive!
    void setOrdinal(Ordinal value, int which = 0) noexcept { parts_[which % 3] = value; }
#ifdef NUMERICS_ARCHITECTURE
    constexpr auto getExtendedReal() const noexcept { return lreal_; }
void setExtendedReal(LongReal value) noexcept { lreal_ = value; }
#endif
        void performMov(const TripleRegister& other) noexcept {
            parts_[0] = other.parts_[0];
            parts_[1] = other.parts_[1];
            parts_[2] = other.parts_[2];
        }
private:
    Ordinal parts_[4]; // the fourth Ordinal is for alignment purposes, there is no way to modify it through the class
#ifdef NUMERICS_ARCHITECTURE
    ExtendedReal lreal_;
#endif
};

union QuadRegister {
    public:
        constexpr explicit QuadRegister(Ordinal a = 0, Ordinal b = 0, Ordinal c = 0, Ordinal d = 0) noexcept : parts_{a, b, c, d}{ }
        constexpr explicit QuadRegister(LongOrdinal lower, LongOrdinal upper) noexcept : halves_{lower, upper} { }
        constexpr auto getOrdinal(int which = 0) const noexcept { return parts_[which & 0b11]; } // very expensive!
        void setOrdinal(Ordinal value, int which = 0) noexcept { parts_[which & 0b11] = value; }
        constexpr auto getHalf(int which = 0) const noexcept { return halves_[which & 0b01];}
        void setHalf(LongOrdinal value, int which = 0) noexcept { halves_[which & 0b01] = value; }
#ifdef NUMERICS_ARCHITECTURE
        constexpr auto getExtendedReal() const noexcept { return lreal_; }
        void setExtendedReal(LongReal value) noexcept { lreal_ = value; }
#endif
        void performMov(const QuadRegister& other) noexcept {
            halves_[0] = other.halves_[0];
            halves_[1] = other.halves_[1];
        }
private:
    Ordinal parts_[4];
    LongOrdinal halves_[2];
#ifdef NUMERICS_ARCHITECTURE
    ExtendedReal lreal_;
#endif
};

union RegisterFrame {
    RegisterFrame() noexcept : gprs { Register(), Register(), Register(), Register(),
                                      Register(), Register(), Register(), Register(),
                                      Register(), Register(), Register(), Register(),
                                      Register(), Register(), Register(), Register(),
    } {

    }
    const Register& getRegister(int index) const noexcept { return gprs[index & 0b1111]; }
    Register& getRegister(int index) noexcept { return gprs[index & 0b1111]; }
    const DoubleRegister& getDoubleRegister(int index) const noexcept { return dprs[(index >> 1) & 0b111]; }
    DoubleRegister& getDoubleRegister(int index) noexcept { return dprs[(index >> 1) & 0b111]; }
    const TripleRegister& getTripleRegister(int index) const noexcept { return tprs[(index >> 2) & 0b11]; }
    TripleRegister& getTripleRegister(int index) noexcept { return tprs[(index >> 2) & 0b11]; }
    const QuadRegister& getQuadRegister(int index) const noexcept { return qprs[(index >> 2) & 0b11]; }
    QuadRegister& getQuadRegister(int index) noexcept { return qprs[(index >> 2) & 0b11]; }

    constexpr auto getNumRegisters() const noexcept { return 16; }
    constexpr auto getNumDoubleRegisters() const noexcept { return 8; }
    constexpr auto getNumTripleRegisters () const noexcept { return 4; }
    constexpr auto getNumQuadRegisters () const noexcept { return 4; }

    Register gprs[16];
    DoubleRegister dprs[sizeof(gprs)/sizeof(DoubleRegister)];
    TripleRegister tprs[sizeof(gprs)/sizeof(TripleRegister)]; // this will have the same alignment as quad registers by ignoring the fourth ordinal
    QuadRegister qprs[sizeof(gprs)/sizeof(QuadRegister)];
    // we put the extended reals in a different location
};
#endif //SIM3_REGISTER_H
