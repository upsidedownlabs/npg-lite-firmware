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
  - Sender code based on the “SendDemo” example from the Arduino-IRremote library
    (https://github.com/Arduino-IRremote/Arduino-IRremote/blob/main/examples/SendDemo/SendDemo.ino)
*/

#include <Arduino.h>
#include <IRremote.hpp>

#define IR_SEND_PIN 22   // your IR-LED pin (via resistor/transistor)
#define BOOT_BUTTON  9     // NPG-Lite BOOT button (active LOW)

int c = 0;   // Counter for maintaining ON/OFF state

// The full 28-bit MSB-first codes captured by the receiver sketch
static const uint32_t FULL_ON  = 0x8800B0B;  
static const uint32_t FULL_OFF = 0x88C0051;  

void setup() {
  Serial.begin(115200);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  Serial.println("\n===== LG-AC: Send Full 28-bit Codes =====");
  IrSender.begin(IR_SEND_PIN); // initialize the IR sender (RMT)
  Serial.println("Type '1' + ENTER → send ON");
  Serial.println("Type '0' + ENTER → send OFF");
}

void loop() {
  // When Boot button is pressed
  if(digitalRead(BOOT_BUTTON) == LOW)
  {
  if (c%2==0) {
    Serial.println("→ Sending AC ON");
    // sendLG automatically uses the correct 38 kHz carrier and timing for LG
    IrSender.sendLG(FULL_ON, 28);
    Serial.println("  [Done]\n");
    delay(500);
  }
  else {
    Serial.println("→ Sending AC OFF");
    IrSender.sendLG(FULL_OFF, 28);
    Serial.println("  [Done]\n");
    delay(500);
  }
  c=c+1;
  }
}
