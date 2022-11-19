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
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include <Adafruit_SI5351.h>
#include "Types.h"
#include "BinaryOperations.h"
#include "Core.h"
#include "ArduinoJson.h"
Adafruit_FT6206 touchScreen;
Adafruit_ILI9341 tft(TFTCS, TFTDC);
Adafruit_SI5351 clockgen;
void
setInternalBusAddress(const SplitWord32& address) noexcept {
    digitalWrite(BANK0, address.internalBankAddress.bank0);
    digitalWrite(BANK1, address.internalBankAddress.bank1);
    digitalWrite(BANK2, address.internalBankAddress.bank2);
    digitalWrite(BANK3, address.internalBankAddress.bank3);
}

void
set328BusAddress(const SplitWord32& address) noexcept {
    // set the upper
    PORTF = address.splitAddress.a24_31;
    PORTK = address.splitAddress.a16_23;
    digitalWrite(38, address.splitAddress.a15);
}

    

volatile bool HasSDCard = false;
volatile bool HasTouchScreen = false;
volatile bool HasRTC = false;
volatile bool HasClockGen = false;
RTC_PCF8523 rtc;
void 
setupClockGenerator() noexcept {
    Serial.print(F("Configuring Si5351..."));
    HasClockGen = clockgen.begin() == ERROR_NONE;
    if (HasClockGen) {
        /// @todo configure clocks for default speeds here
        clockgen.enableOutputs(true);
        Serial.println(F("DONE"));
    } else {
        Serial.println(F("FAILED"));
    }
}
void 
setupDisplay() noexcept {
    Serial.print(F("Configuring ILI9341..."));
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    Serial.println(F("DONE"));
}
void
setupTouchScreen() noexcept {
    Serial.print(F("Configuring FT6206..."));
    HasTouchScreen = touchScreen.begin(40);
    if (!HasTouchScreen) {
        Serial.println(F("FAILED"));
    } else {
        Serial.println(F("DONE"));
    }
}
void
setupRTC() noexcept {
    Serial.print(F("Configuring PCF8523..."));
    HasRTC = rtc.begin();
    if (HasRTC) {
        if (!rtc.lostPower()) {
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        rtc.start();
        // The PCF8523 can be calibrated for:
        //        - Aging adjustment
        //        - Temperature compensation
        //        - Accuracy tuning
        // The offset mode to use, once every two hours or once every minute.
        // The offset Offset value from -64 to +63. See the Application Note for calculation of offset values.
        // https://www.nxp.com/docs/en/application-note/AN11247.pdf
        // The deviation in parts per million can be calculated over a period of observation. Both the drift (which can be negative)
        // and the observation period must be in seconds. For accuracy the variation should be observed over about 1 week.
        // Note: any previous calibration should cancelled prior to any new observation period.
        // Example - RTC gaining 43 seconds in 1 week
        float drift = 43; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
        float period_sec = (7 * 86400);  // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
        float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
        float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
                                 // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
        int offset = round(deviation_ppm / drift_unit);
        rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
        Serial.println(F("DONE"));
    } else {
        Serial.println(F("FAILED"));
    }
}
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

void 
setup() {
    setupSerial();
    Serial.println(F("Bringing up peripherals"));
    Serial.println();
    setupSPI();
    setupTWI();
    setupDisplay();
    setupTouchScreen();
    setupRTC();
    setupClockGenerator();
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

    HasSDCard = SD.begin(SDPin);
    if (!HasSDCard) {
        Serial.println(F("NO SD CARD"));
    } else {
        Serial.println(F("FOUND SD CARD"));
    }
    configureSimulatorStructures();
    Serial.println(F("BOOT COMPLETE!!"));
}
void 
loop() {
    invokeCore();
}

