// sim5
// Copyright (c) 2022, Joshua Scoggins
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
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Types.h"
#include "BinaryOperations.h"
#include "Core.h"
#include "ArduinoJson.h"
#include "Peripherals.h"

void
setupTWI() noexcept {
    Serial.print(F("Configuring TWI..."));
    Wire.begin();
    Serial.println(F("DONE"));
}
void
setupSPI() noexcept {
    Serial.print(F("Configuring SPI..."));
    SPI.begin();
    Serial.println(F("DONE"));
}
void
setupSerial(bool displayBootScreen = true) noexcept {
    Serial.begin(115200);
    if (displayBootScreen) {
        Serial.println(F("i960 Simulator (C) 2022 and beyond Joshua Scoggins"));
        Serial.println(F("This software is open source software"));
        Serial.println(F("Base Platform: Arduino Mega2560"));
    }
}
Core core;
void 
setup() {
    setupSerial();
    Serial.println(F("Bringing up peripherals"));
    Serial.println();
    setupSPI();
    setupTWI();
    setupPeripherals();
    Serial.print(F("Configuring GPIOs..."));
    pinMode(BANK0, OUTPUT);
    pinMode(BANK1, OUTPUT);
    pinMode(BANK2, OUTPUT);
    pinMode(BANK3, OUTPUT);
    /// @todo configure ports f and k
    pinMode(FAILPIN, OUTPUT);
    digitalWrite(FAILPIN, LOW);

    pinMode(INTPIN, INPUT);
    pinMode(BUSYPIN, INPUT);

    pinMode(LOCKPIN, OUTPUT);
    digitalWrite(LOCKPIN, LOW);
    pinMode(LOCKPIN, INPUT);
    Serial.println(F("DONE"));
    Serial.print(F("Setting up EBI..."));
    // cleave the address space in half via sector limits.
    // lower half is io space for the implementation
    // upper half is the window into the 32/8 bus
    XMCRB = 0;           // No external memory bus keeper and full 64k address
                         // space
    XMCRA = 0b1100'0000; // Divide the 64k address space in half at 0x8000, no
                         // wait states activated either. Also turn on the EBI
    set328BusAddress(0);
    setInternalBusAddress(0);
    Serial.println(F("DONE"));

    core.begin();
    configureSimulatorStructures();
    Serial.println(F("BOOT COMPLETE!!"));
}
void 
loop() {
    while (core.cycle());
}

