// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

/*
  Before uploading, install the “Arduino-IRremote” library by shirriff, z3t0, ArminJo via Library Manager

  References:
  - Receiver code based on the “IRrecvDumpV2” example from the Arduino-IRremote library
    (https://github.com/Arduino-IRremote/Arduino-IRremote/blob/main/examples/IRrecvDumpV2/IRrecvDumpV2.ino)
*/    

#include <Arduino.h>
#include <IRremote.hpp>

#define IR_RECV_PIN 5   // your IR-receiver OUT pin

// Reverse the order of the lowest `bitCount` bits in `v`
// Because the library gives us LSB-first data, but we need MSB-first.
uint32_t reverseBits(uint32_t v, uint8_t bitCount) {
  uint32_t r = 0;
  for (uint8_t i = 0; i < bitCount; i++) {
    r = (r << 1) | (v & 1);
    v >>= 1;
  }
  return r;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n===== LG-AC: Capture Full 28-bit Codes =====");
  IrReceiver.begin(IR_RECV_PIN, DISABLE_LED_FEEDBACK); // start IR receiver
}

void loop() {
  if (!IrReceiver.decode()) return; // wait for a valid code
  auto &d = IrReceiver.decodedIRData;

  // 1) Library gives us a 27-bit value in LSB-first order
  uint32_t raw27 = d.decodedRawData;
  uint8_t  bits27 = d.numberOfBits;

  // 2) Reverse to MSB-first so the most significant bit is at the top
  uint32_t rev27 = reverseBits(raw27, bits27);

  // 3) Prepend the hidden header bit (value 1) to make a full 28-bit word
  uint32_t full28 = (1UL << bits27) | rev27;

  // Print the raw data so you can record the signals
  Serial.println("\n--- Captured LG-AC Frame ---");
  Serial.print("  Protocol: ");  
  Serial.println((uint16_t)d.protocol == 2 ? "LG-AC" : String((uint16_t)d.protocol));
  Serial.print("  Raw27  : 0x"); Serial.println(raw27, HEX);
  Serial.print("  Bits27 : ");      Serial.println(bits27);
  Serial.print("  Full28 : 0x");    Serial.println(full28, HEX);
  Serial.print("  Bits28 : 28");
  Serial.println("\n-----------------------------");

  IrReceiver.resume(); // ready to receive the next code
  delay(200);
}
