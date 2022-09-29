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

#ifndef SIM3_CORE_H
#define SIM3_CORE_H
#include "Types.h"
#include "Instruction.h"
#include "Register.h"

enum class FaultType : Ordinal {
    Trace = 0x0001'0000,
    Instruction_Trace = Trace | 0x02,
    Branch_Trace = Trace | 0x04,
    Call_Trace = Trace | 0x08,
    Return_Trace = Trace | 0x10,
    Prereturn_Trace = Trace | 0x20,
    Supervisor_Trace = Trace | 0x40,
    Breakpoint_Trace = Trace | 0x80,
    Operation = 0x0002'0000,
    Operation_InvalidOpcode = Operation | 0x01,
    Operation_InvalidOperand = Operation | 0x04,
    Arithmetic = 0x0003'0000,
    Arithmetic_IntegerOverflow = Arithmetic | 0x01,
    Arithmetic_ArithmeticZeroDivide = Arithmetic | 0x02,
#ifdef NUMERICS_ARCHITECTURE
    FloatingPoint = 0x0004'0000,
/// @todo implement floating point faults
#endif
    Constraint = 0x0005'0000,
    Constraint_Range = Constraint | 0x01,
    Constraint_Privileged = Constraint | 0x02,

    Protection = 0x0007'0000,
    Protection_Length = Protection | 0x02,
    Type = 0x000A'0000,
    Type_Mismatch = Type | 0x01,
};
class Core {
public:
    /**
     * @brief The main node in a circular queue used to keep track of the on chip register entries
     */
    class LocalRegisterPack {
    public:
        LocalRegisterPack() = default;
        RegisterFrame& getUnderlyingFrame() noexcept { return underlyingFrame; }
        const RegisterFrame& getUnderlyingFrame() const noexcept { return underlyingFrame; }
        /**
         * @brief Relinquish ownership of the current register pack without saving the contents
         */
        void relinquishOwnership() noexcept {
            valid_ = false;
            framePointerAddress_ = 0;
            // the following code does something the original i960 spec does not do, clear registers out
            //for (auto& a : underlyingFrame.gprs) {
            //    a.setOrdinal(0);
            //}
        }
        /**
         * @brief Relinquish ownership of the current register pack after saving its contents to memory if valid
         * @tparam T The function signature used to save registers to the stack
         * @param saveRegisters the function used to save a valid register frame
         */
        template<typename T>
        void relinquishOwnership(T saveRegisters) noexcept {
            if (valid_) {
                saveRegisters(underlyingFrame, framePointerAddress_);
            }
            relinquishOwnership();
        }
        /**
         * @brief Take ownership of this pack and save whatever is currently there (if valid); No analysis of old vs new FP is done;
         * Generally this function is meant for generating new calls into the stack
         * @tparam T
         * @param saveRegisters
         */
        template<typename T>
        void takeOwnership(Address newFP, T saveRegisters) noexcept {
            if (valid_) {
                // we do not analyze to see if we got a match because that should never happen
                saveRegisters(underlyingFrame, framePointerAddress_);
            }
            valid_ = true;
            framePointerAddress_ = newFP;
            // don't clear out the registers
            //for (auto& a : underlyingFrame.dprs) {
            //    // clear out storage two registers at a time
            //    a.setLongOrdinal(0);
            //}
            // the instruction contents will be responsible for populating this back up
        }

        /**
         * @brief Use this pack to try and reclaim ownership to a given register frame, if the pack is currently valid and matches the frame pointer address then we are safe to use it as is
         * @tparam RestoreFunction The function type used for restoring registers from the stack
         * @tparam SaveFunction The function type used for saving registers to the stack
         * @param newFP The new frame pointer address that this pack will reflect
         * @param saveRegisters the function to use when saving registers to the stack
         * @param restoreRegisters the function to use when restoring registers from the stack
         */
        template<typename SaveFunction, typename RestoreFunction>
        void restoreOwnership(Address newFP, SaveFunction saveRegisters, RestoreFunction restoreRegisters) {
            if (valid_) {
                // okay we have something valid in there right now, so we need to determine if
                // it is currently valid or not
                if (newFP == framePointerAddress_) {
                    // okay so we got a match, great!
                    // just leave early since the frame is already setup
                    return;
                }
                // okay we got a mismatch, the goal is to now save the current frame contents to memory
                saveRegisters(underlyingFrame, framePointerAddress_);
                // now we continue on as though this pack was initially invalid
            }
            // either is not valid
            // now do the restore operation since it doesn't matter how we got here
            valid_ = true;
            framePointerAddress_ = newFP;
            restoreRegisters(underlyingFrame, framePointerAddress_);
        }
    private:
        RegisterFrame underlyingFrame;
        Address framePointerAddress_ = 0;
        bool valid_ = false;
    };
public:
    static constexpr Register OrdinalLiterals[32] {
#define X(base) Register(base + 0), Register(base + 1), Register(base + 2), Register(base + 3)
            X(0),
            X(4),
            X(8),
            X(12),
            X(16),
            X(20),
            X(24),
            X(28),
#undef X
    };
    static constexpr DoubleRegister LongOrdinalLiterals[32] {
#define X(base) DoubleRegister(base + 0), DoubleRegister(base + 1), DoubleRegister(base + 2), DoubleRegister(base + 3)
            X(0),
            X(4),
            X(8),
            X(12),
            X(16),
            X(20),
            X(24),
            X(28),
#undef X
    };
    static constexpr TripleRegister TripleOrdinalLiterals[32] {
#define X(base) TripleRegister(base + 0), TripleRegister(base + 1), TripleRegister(base + 2), TripleRegister(base + 3)
            X(0),
            X(4),
            X(8),
            X(12),
            X(16),
            X(20),
            X(24),
            X(28),
#undef X
    };
    static constexpr QuadRegister QuadOrdinalLiterals[32] {
#define X(base) QuadRegister(base + 0), QuadRegister(base + 1), QuadRegister(base + 2), QuadRegister(base + 3)
            X(0),
            X(4),
            X(8),
            X(12),
            X(16),
            X(20),
            X(24),
            X(28),
#undef X
    };
public:
public:
    explicit Core(Ordinal salign = 4);
    virtual ~Core() = default;
public:
    void cycle() noexcept;
    void begin() noexcept;
    void boot();
protected:
    void lock();
    void unlock();
    constexpr Ordinal getSystemAddressTableBase() const noexcept { return systemAddressTableBase_; }
    constexpr Ordinal getPRCBPtrBase() const noexcept { return prcbBase_; }
    virtual Ordinal getSystemProcedureTableBase() ;
    virtual Ordinal getFaultProcedureTableBase() ;
    virtual Ordinal getTraceTablePointer() ;
    virtual Ordinal getInterruptTableBase() ;
    virtual Ordinal getFaultTableBase() ;
    virtual Ordinal getInterruptStackPointer();
    virtual void generateFault(FaultType fault);

    byte load8(Address destination) noexcept;
    ShortOrdinal load16(Address destination) noexcept;
    Ordinal load32(Address destination) noexcept;
    inline void load32(Address destination, Register& reg) noexcept {
        reg.setOrdinal(load32(destination));
    }
    LongOrdinal load64(Address destination) noexcept;
    inline void load64(Address destination, DoubleRegister& reg) noexcept {
        reg.setLongOrdinal(load64(destination));
    }
    void load96(Address destination, TripleRegister& reg) noexcept;
    void load128(Address destination, QuadRegister& reg) noexcept;
    QuadRegister load128(Address destination) noexcept {
        QuadRegister tmp;
        load128(destination, tmp);
        return tmp;
    }

    void store8(Address destination, byte value) noexcept;
    void store32(Address destination, Ordinal value) noexcept;
    inline void store32(Address destination, const Register& reg) noexcept { store32(destination, reg.getOrdinal()); }
    void store16(Address destination, ShortOrdinal value) noexcept;
    void store64(Address destination, LongOrdinal value) noexcept;
    inline void store64(Address destination, const DoubleRegister& reg) noexcept { store64(destination, reg.getLongOrdinal()); }
    void store96(Address destination, const TripleRegister& reg) noexcept;
    void store128(Address destination, const QuadRegister& reg) noexcept;
    void storeShortInteger(Address destination, ShortInteger value) {
        union {
            ShortInteger in;
            ShortOrdinal out;
        } thing;
        thing.in = value;
        store16(destination, thing.out);
    }
    void storeByteInteger(Address destination, ByteInteger value) {
        union {
            ByteInteger in;
            ByteOrdinal out;
        } thing;
        thing.in = value;
        store8(destination, thing.out);
    }
    void synchronizedStore(Address destination, const DoubleRegister& value) noexcept;
    void synchronizedStore(Address destination, const QuadRegister& value) noexcept;
    void synchronizedStore(Address destination, const Register& value) noexcept;
    Register& getRegister(RegisterIndex targetIndex);
    const Register& getRegister(RegisterIndex targetIndex) const;
    const Register& getSourceRegister(RegisterIndex targetIndex) const { return getRegister(targetIndex); }
    DoubleRegister& getDoubleRegister(RegisterIndex targetIndex);
    const DoubleRegister& getDoubleRegister(RegisterIndex targetIndex) const;
    const DoubleRegister& getSourceDoubleRegister(RegisterIndex targetIndex) const { return getDoubleRegister(targetIndex); }
    TripleRegister& getTripleRegister(RegisterIndex targetIndex);
    const TripleRegister& getTripleRegister(RegisterIndex targetIndex) const;
    QuadRegister& getQuadRegister(RegisterIndex targetIndex);
    const QuadRegister& getQuadRegister(RegisterIndex targetIndex) const;
    Register& getStackPointer() noexcept {
        return getRegister(RegisterIndex::SP960);
    }
    const Register& getStackPointer() const noexcept {
        return getRegister(RegisterIndex::SP960);
    }
    Register& getFramePointer() noexcept {
        return getRegister(RegisterIndex::FP);
    }
    const Register& getFramePointer() const noexcept {
        return getRegister(RegisterIndex::FP);
    }
    Register& getPFP() noexcept {
        return getRegister(RegisterIndex::PFP);
    }
    const Register& getPFP() const noexcept {
        return getRegister(RegisterIndex::PFP);
    }
    Register& getRIP() noexcept {
        return getRegister(RegisterIndex::RIP);
    }
    const Register& getRIP() const noexcept {
        return getRegister(RegisterIndex::RIP);
    }
protected:
    inline Ordinal getSupervisorStackPointer() noexcept { return load32((getSystemProcedureTableBase() + 12)); }
    LocalRegisterPack& getCurrentPack() noexcept { return frames[currentFrameIndex_]; }
private:
    void lda(const Instruction& inst) noexcept;
    constexpr Ordinal shro(Ordinal src, Ordinal len) const noexcept {
        if (len < 32) {
            return src >> len;
        } else {
            return 0;
        }
    }
    constexpr Ordinal shlo(Ordinal src, Ordinal len) const noexcept {
        if (len < 32) {
            return src << len;
        } else {
            return 0;
        }
    }
    void flushreg() noexcept;
    void cmpibx(const Instruction& instruction, uint8_t mask, Integer src1, Integer src2) noexcept;
    void ipRelativeBranch(Integer displacement) noexcept {
        advanceIPBy = 0;
        ip_.setInteger(ip_.getInteger() + displacement);
    }
    Instruction loadInstruction(Address baseAddress) noexcept;
    void executeInstruction(const Instruction& instruction) noexcept;
    void cmpi(Integer src1, Integer src2) noexcept;
    void cmpo(Ordinal src1, Ordinal src2) noexcept;
    void syncf() noexcept;
    void setDestination(RegisterIndex index, Ordinal value, TreatAsOrdinal);
    void setDestination(RegisterIndex index, Integer value, TreatAsInteger);
    Integer getSourceRegisterValue(RegisterIndex index, TreatAsInteger) const;
    Ordinal getSourceRegisterValue(RegisterIndex index, TreatAsOrdinal) const;
    void cmpobx(const Instruction& instruction, uint8_t mask, Ordinal src1, Ordinal src2) noexcept;
    void branchGeneric(const Instruction& instruction, bool andLink) noexcept;
    void condBranch(const Instruction& instruction, uint8_t mask) noexcept;
private:
    void saveRegisterFrame(const RegisterFrame& theFrame, Address baseAddress) noexcept;
    void restoreRegisterFrame(RegisterFrame& theFrame, Address baseAddress) noexcept;
    Ordinal computeMemoryAddress(const Instruction& instruction) noexcept;
    void boot0(Ordinal sat, Ordinal pcb, Ordinal startIP) noexcept;
//protected:
    //void clearLocalRegisters() noexcept;
private:
    /**
     * @brief Number of register frames allocated "on-chip", shouldn't be too large as performance will suffer
     */
    static constexpr Ordinal NumRegisterFrames = 4;
    [[nodiscard]] RegisterFrame& getLocals() noexcept;
    [[nodiscard]] const RegisterFrame& getLocals() const noexcept;
    [[nodiscard]] LocalRegisterPack& getNextPack() noexcept;
    [[nodiscard]] LocalRegisterPack& getPreviousPack() noexcept;
    /// @todo finish implementing these two functions
    void enterCall(Address newAddress) noexcept;
    void exitCall() noexcept;
    void call(const Instruction& instruction) noexcept;
    void callx(const Instruction& instruction) noexcept;
    void calls(const Instruction& instruction) noexcept;
    void ret() noexcept;
    [[nodiscard]] Ordinal properFramePointerAddress() const noexcept;
protected:
    Register ip_; // start at address zero
    ArithmeticControls ac_;
    ProcessControls pc_;
    TraceControls tc_;
    // use a circular queue to store the last n local registers "on-chip"
    RegisterFrame globals;
#ifdef NUMERICS_ARCHITECTURE
    ExtendedReal fpRegs[4] = { 0 };
#endif
    Ordinal advanceIPBy = 0;
    Ordinal salign_;
    Ordinal c_;
    Ordinal currentFrameIndex_ = 0;
    LocalRegisterPack frames[NumRegisterFrames];
    Ordinal systemAddressTableBase_ = 0;
    Ordinal prcbBase_ = 0;
};
#endif //SIM3_CORE_H
