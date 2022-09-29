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

// The main for the arduino version of the simulator's main
// Created by jwscoggins on 8/21/21.
//
#ifdef ARDUINO
#include "Core.h"
#include <Arduino.h>
class IACMessage {
    public:
        explicit IACMessage(const QuadRegister& qr) noexcept :
            field0_(qr.getOrdinal(0)),
            field3_(qr.getOrdinal(1)),
            field4_(qr.getOrdinal(2)),
            field5_(qr.getOrdinal(3)) { }
        constexpr uint8_t getMessageType() const noexcept { return messageType_; }
        constexpr uint8_t getField1() const noexcept { return field1_; }
        constexpr uint16_t getField2() const noexcept { return field2_; }
        constexpr uint32_t getField3() const noexcept { return field3_; }
        constexpr uint32_t getField4() const noexcept { return field4_; }
        constexpr uint32_t getField5() const noexcept { return field5_; }
        constexpr uint32_t getField0() const noexcept { return field0_; }
    private:
        union {
            /// @todo revert this to do bit manipulation as the bitfield assumes little endian
            uint32_t field0_;
            struct {
                uint16_t field2_;
                uint8_t field1_;
                uint8_t messageType_;
            };
        };
        uint32_t field3_;
        uint32_t field4_;
        uint32_t field5_;
};
void
Core::begin() noexcept {

}
void
Core::lock() {

}

void
Core::unlock() {

}
void
Core::boot() {
    auto q = load128(0);
    boot0(q.getOrdinal(0), q.getOrdinal(1), q.getOrdinal(3));
}
void
Core::boot0(Ordinal sat, Ordinal pcb, Ordinal startIP) noexcept {
#ifdef EMULATOR_TRACE
#ifdef ARDUINO
    Serial.print("ENTERING ");
    Serial.println(__PRETTY_FUNCTION__ );
#endif
#endif
    systemAddressTableBase_ = sat;
    prcbBase_ = pcb;
    // skip the check words
    ip_.setOrdinal(startIP);
    pc_.setPriority(31);
    pc_.setState(true); // needs to be set as interrupted
    auto thePointer = getInterruptStackPointer();
#ifdef EMULATOR_TRACE
#ifdef ARDUINO
    Serial.print("THE POINTER: 0x");
    Serial.println(thePointer, HEX);
#endif
#endif
    // also make sure that we set the target pack to zero
    currentFrameIndex_ = 0;
    // invalidate all cache entries forcefully
    for (auto& a : frames) {
        a.relinquishOwnership();
        // at this point we want all of the locals to be cleared, this is the only time
        for (auto& reg : a.getUnderlyingFrame().dprs) {
            reg.setLongOrdinal(0);
        }
    }
    getRegister(RegisterIndex::FP).setOrdinal(thePointer);
    // we need to take ownership of the target frame on startup
    // we want to take ownership and throw anything out just in case so make the lambda do nothing
    getCurrentPack().takeOwnership(thePointer, [](const auto&, auto) noexcept { });
    // THE MANUAL DOESN'T STATE THAT YOU NEED TO SETUP SP and PFP as well
    getRegister(RegisterIndex::SP960).setOrdinal(thePointer + 64);
    getRegister(RegisterIndex::PFP).setOrdinal(thePointer);
    advanceIPBy = 0; // make sure that we don't do anything dumb at this point
#ifdef EMULATOR_TRACE
#ifdef ARDUINO
    Serial.print("EXITING ");
    Serial.println(__PRETTY_FUNCTION__ );
#endif
#endif
}

void
Core::generateFault(FaultType) {
    /// @todo implement this at some point
    // lookup fault information
    // setup fault data frame
    // call fault handler
    // probably should exit or something here
}
void
Core::synchronizedStore(Address destination, const DoubleRegister &value) noexcept {
    // no special IAC locations when dealing with long versions so cool beans
    store64(destination, value.getLongOrdinal());
}
void
Core::synchronizedStore(Address destination, const QuadRegister &value) noexcept {
    if (destination == 0xFF00'0010) {
        IACMessage message(value);
#ifdef EMULATOR_TRACE
#ifdef ARDUINO
        Serial.print("ENTERING ");
        Serial.println(__PRETTY_FUNCTION__ );
        Serial.print("IAC MESSAGE CODE: 0x");
        Serial.println(message.getMessageType(), HEX);
        Serial.print("IAC FIELD0: 0x");
        Serial.println(message.getField0(), HEX);
        Serial.print("IAC FIELD1: 0x");
        Serial.println(message.getField1(), HEX);
        Serial.print("IAC FIELD2: 0x");
        Serial.println(message.getField2(), HEX);
        Serial.print("IAC FIELD3: 0x");
        Serial.println(message.getField3(), HEX);
        Serial.print("IAC FIELD4: 0x");
        Serial.println(message.getField4(), HEX);
        Serial.print("IAC FIELD5: 0x");
        Serial.println(message.getField5(), HEX);
#endif
#endif
        switch (message.getMessageType()) {
            case 0x89: // purge instruction cache
                       // do nothing as we don't have an instruction cache
                break;
            case 0x93: // reinitialize processor
                boot0(message.getField3(), message.getField4(), message.getField5());
                break;
            case 0x8F:
                // set breakpoint register
                break;
            case 0x80: {
                           // store system base
                           // stores the current locations of the system address table and the prcb in a specified location in memory.
                           // The address of the system address table is stored in the word starting at the byte specified in field 3,
                           // and the address of the PRCB is stored in the next word in memory (field 3 address plus 4)
                           DoubleRegister pack(getSystemAddressTableBase(), getPRCBPtrBase());
                           store64(message.getField3(), pack.getLongOrdinal());
                           break;
                       }
            case 0x40: // interrupt
                       // Generates an interrupt request. The interrup vector is given in field 1 of the IAC message. The processor handles the
                       // interrupt request just as it does interrupts received from other sources. If the interrupt priority is higher than the prcessor's
                       // current priority, the processor services the interrupt immediately. Otherwise, it posts the interrup in the pending interrupts
                       // section of the interrupt table.
                       break;
            case 0x41: // Test pending interrupts
                       // tests for pending interrupts. The processor checks the pending interrupt section of the interrupt
                       // table for a pending interrupt with a priority higher than the prcoessor's current priority. If a higher
                       // priority interrupt is found, it is serviced immediately. Otherwise, no action is taken

                       /// @todo implement this
                       break;
            default:
                       break;

        }
#ifdef EMULATOR_TRACE
#ifdef ARDUINO
        Serial.print("EXITING ");
        Serial.println(__PRETTY_FUNCTION__ );
#endif
#endif
        // there are special IAC messages we need to handle here
    } else {
        // synchronized stores are always aligned but still go through the normal mechanisms
        store128(destination, value);
    }
}
void
Core::synchronizedStore(Address destination, const Register &value) noexcept {
    // there is a lookup for an interrupt control register, in the Sx manual, we are going to ignore that for now
    store32(destination, value.getOrdinal());
}
// make it an Sx core with SALIGN of 4
Core theCore(4);
void setup() {
    Serial.begin(115200);
    while (!Serial);
    theCore.begin();

    theCore.boot();
}
void loop() {
    theCore.cycle();
    delay(1000);
}
#if __cplusplus >= 201402L
#ifdef ARDUINO_AVR_MEGA2560

void operator delete(void * ptr, size_t)
{
    ::operator delete(ptr);
}


void operator delete[](void * ptr, size_t)
{
    ::operator delete(ptr);
}
#endif
#endif // end language is C++14 or greater
#endif

