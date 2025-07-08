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

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// ———— CONFIG ————
// Invert if an axis feels backwards:
const float INV_X =  1.0;  // +1 = normal, –1 = flip X
const float INV_Y =  1.0;  // +1 = normal, –1 = flip Y
const float INV_Z =  1.0;  // +1 = normal, –1 = flip Z

// How many samples to take for your “zero” calibration:
const int   CAL_SAMPLES = 200;
const float CAL_DELAY_MS = 5.0;  // wait between reads

// These will be filled in by calibrateOffsets():
float offsetX = 0.0;
float offsetY = 0.0;
float offsetZ = 0.0;

void calibrateOffsets() {
  float sumX = 0, sumY = 0, sumZ = 0;
  sensors_event_t a, g, temp;

  Serial.println("Calibrating… keep the board perfectly level and still");

  for (int i = 0; i < CAL_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z;
    delay(CAL_DELAY_MS);
  }
  offsetX = sumX / CAL_SAMPLES;
  offsetY = sumY / CAL_SAMPLES;
  offsetZ = sumZ / CAL_SAMPLES;

  Serial.print("Offsets → X: ");
  Serial.print(offsetX, 3);
  Serial.print("  Y: ");
  Serial.println(offsetY, 3);
  Serial.print("  Z: ");
  Serial.println(offsetZ, 3);
  Serial.println("Calibration done!");
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(230400);
  while (!Serial) {}  // wait for USB‑Serial

  if (!mpu.begin()) {
    Serial.println("MPU not found!");
    while (1) { delay(10); }
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

  // give sensor time to settle
  delay(100);

  calibrateOffsets();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply your calibration & optional inversion:
  float ax = (a.acceleration.x - offsetX) * INV_X;
  float ay = (a.acceleration.y - offsetY) * INV_Y;
  float az = (a.acceleration.z - offsetZ) * INV_Z;

  // Print only X and Y (comma‑separated):
  char buf[64];
  int len = snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f\n", ax, ay, az);
  Serial.write(buf, len);

  // Throttle rate if you like:
  // delay(5);
}