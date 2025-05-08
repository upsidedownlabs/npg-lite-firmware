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

// Copyright (c) 2024 - 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

// References: https://github.com/z3t0/Arduino-IRremote/blob/master/examples/LGACSendDemo/LGACSendDemo.ino

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

// Choose your IR LED pin:
const uint16_t kIrLedPin = 4;
#define BOOT_BUTTON  9     // ESP32 BOOT button (active LOW)

int c=0;

IRsend irsend(kIrLedPin);

// LG‐AC codes are 28 bits long. Replace these with YOUR captured values:
//– OFF (you saw 0x4500188 → bit-reversed → 0x88C0051 in LG’s encoding)
const uint32_t kAcOff = 0x88C0051;
//– ON  (whatever your “power on” capture reversed to)
const uint32_t kAcOn  = 0x8800B4F;  

void setup() {
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  Serial.begin(115200);
  irsend.begin();
  Serial.println("LG AC Blaster ready!");
  Serial.println("Press Boot button to turn AC ON or OFF"); 
}

void loop() {
  if(digitalRead(BOOT_BUTTON) == LOW)
  {
  if (c%2==0) {
    Serial.println("→ Sending AC ON");
    irsend.sendLG(kAcOn, 28);
    delay(500);
  }
  else {
    Serial.println("→ Sending AC OFF");
    irsend.sendLG(kAcOff, 28);
    delay(500);
  }
  c=c+1;
  }
}
