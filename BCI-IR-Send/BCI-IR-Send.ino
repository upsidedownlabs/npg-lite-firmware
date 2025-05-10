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
// Copyright (c) 2024 - 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!
/*
  FFT routines based on Espressif’s ESP-DSP examples:

    • Initialization (dsps_fft2r_init_fc32) from:
      https://github.com/espressif/esp-dsp/tree/master/examples/basic_math
      (examples/basic_math/main/dsps_math_main.c)

    • Two-real FFT processing
      (dsps_fft2r_fc32, dsps_bit_rev_fc32, dsps_cplx2reC_fc32)
      from: https://github.com/espressif/esp-dsp/tree/master/examples/fft
      (examples/fft/main/dsps_fft_main.c)
*/
/*
  Before uploading, install the “Arduino-IRremote” library by shirriff, z3t0, ArminJo via Library Manager

  References:
  - Sender code based on the “SendDemo” example from the Arduino-IRremote library
    (https://github.com/Arduino-IRremote/Arduino-IRremote/blob/main/examples/SendDemo/SendDemo.ino)
*/

#include <Arduino.h>
#include "esp_dsp.h"
#include <Adafruit_NeoPixel.h>
#include <IRremote.hpp>

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE       512           // samples per second
#define FFT_SIZE          512           // must be a power of two
#define BAUD_RATE         115200
#define INPUT_PIN         A0
#define PIXEL_PIN 15
#define PIXEL_COUNT 6
#define LED_PIN 7
#define IR_SEND_PIN 22   // your IR-LED pin (via resistor/transistor)

int c = 0;   // Counter for maintaining ON/OFF state

// The full 28-bit MSB-first codes captured by the receiver sketch
static const uint32_t FULL_ON  = 0x8800B0B;  
static const uint32_t FULL_OFF = 0x88C0051;  

// Onboard Neopixel at PIXEL_PIN
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// EEG bands (Hz)
#define DELTA_LOW    0.5f
#define DELTA_HIGH   4.0f
#define THETA_LOW    4.0f
#define THETA_HIGH   8.0f
#define ALPHA_LOW    8.0f
#define ALPHA_HIGH   13.0f
#define BETA_LOW     13.0f
#define BETA_HIGH    30.0f
#define GAMMA_LOW    30.0f
#define GAMMA_HIGH   45.0f

#define SMOOTHING_FACTOR 0.63f
#define EPS              1e-7f

// ----------------- BUFFERS & TYPES -----------------
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE/2];

// For two-real FFT trick
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];

typedef struct {
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;

BandpowerResults smoothedPowers = {0};

// --- Filter Functions ---
// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float Notch(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.58696045*z1 - 0.96505858*z2;
    output = 0.96588529*x + -1.57986211*z1 + 0.96588529*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.62761184*z1 - 0.96671306*z2;
    output = 1.00000000*x + -1.63566226*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Low-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 45.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float EEGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.24200128*z1 - 0.45885207*z2;
    output = 0.05421270*x + 0.10842539*z1 + 0.05421270*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// ----------------- BANDPOWER & SMOOTHING -----------------
BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize) {
  BandpowerResults r = {0};
  for(int i=1; i<halfSize; i++){
    float freq = i * binRes;
    float p    = ps[i];
    r.total   += p;
         if(freq>=DELTA_LOW && freq<DELTA_HIGH)  r.delta += p;
    else if(freq>=THETA_LOW && freq<THETA_HIGH)  r.theta += p;
    else if(freq>=ALPHA_LOW && freq<ALPHA_HIGH)  r.alpha += p;
    else if(freq>=BETA_LOW  && freq<BETA_HIGH)   r.beta  += p;
    else if(freq>=GAMMA_LOW && freq<GAMMA_HIGH)  r.gamma += p;
  }
  return r;
}

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s) {
  s->delta = SMOOTHING_FACTOR*raw->delta + (1-SMOOTHING_FACTOR)*s->delta;
  s->theta = SMOOTHING_FACTOR*raw->theta + (1-SMOOTHING_FACTOR)*s->theta;
  s->alpha = SMOOTHING_FACTOR*raw->alpha + (1-SMOOTHING_FACTOR)*s->alpha;
  s->beta  = SMOOTHING_FACTOR*raw->beta  + (1-SMOOTHING_FACTOR)*s->beta;
  s->gamma = SMOOTHING_FACTOR*raw->gamma + (1-SMOOTHING_FACTOR)*s->gamma;
  s->total = SMOOTHING_FACTOR*raw->total + (1-SMOOTHING_FACTOR)*s->total;
}

// ----------------- DSP FFT SETUP -----------------
void initFFT() {
  // initialize esp-dsp real-FFT (two-real trick)
  esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if(err != ESP_OK){
    Serial.println("FFT init failed");
    while(1) delay(10);
  }
}

// –– Global variables to hold timer state
unsigned long timer1Start = 0;
bool timerRunning = false;

// –– Call this to start (or restart) the timer
void startTimer()
{
  timer1Start = millis(); // remember the current time
  timerRunning = true;
}

// –– Call this to stop the timer (optional)
void stopTimer()
{
  timerRunning = false;
}

// –– Toggle switch both in hardware and in our state variable
void toggleSwitch()
{
  if (c%2==0) {
    Serial.println("→ Sending AC ON");
    // sendLG automatically uses the correct 38 kHz carrier and timing for LG
    IrSender.sendLG(FULL_ON, 28);
    Serial.println("  [Done]\n");
    delay(250);
  }
  else {
    Serial.println("→ Sending AC OFF");
    IrSender.sendLG(FULL_OFF, 28);
    Serial.println("  [Done]\n");
    delay(250);
  }
  c=c+1;
}

// -- Set Neopixel LEDs to indicate focus levels
void showPixels(int ledCount)
{
  for(int i=1;i<=ledCount;i++)
  {
    pixels.setPixelColor(i-1, pixels.Color(i*10, 0, 0)); // Level 1 to 6
  }
}

// ----------------- FFT + BANDPOWER + PEAK -----------------
void processFFT() {
  // pack real→complex: real=inputBuffer, imag=0
  for(int i=0; i<FFT_SIZE; i++){
    y_cf[2*i]   = inputBuffer[i];
    y_cf[2*i+1] = 0.0f;
  }

  // FFT
  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

  // magnitude² spectrum
  int half = FFT_SIZE/2;
  for(int i=0; i<half; i++){
    float re = y1_cf[2*i];
    float im = y1_cf[2*i+1];
    powerSpectrum[i] = re*re + im*im;
  }

  // detect peak bin (skip i=0)
  int maxIdx = 1;
  float maxP = powerSpectrum[1];
  for(int i=2; i<half; i++){
    if(powerSpectrum[i] > maxP){
      maxP = powerSpectrum[i];
      maxIdx = i;
    }
  }
  float binRes = float(SAMPLE_RATE)/FFT_SIZE;
  float peakHz = maxIdx * binRes;

  // bandpower & smoothing
  BandpowerResults raw = calculateBandpower(powerSpectrum, binRes, half);
  smoothBandpower(&raw, &smoothedPowers);
  float T = smoothedPowers.total + EPS;  // Total Bandpower
  float betaPerc = (smoothedPowers.beta/T)*100;

  pixels.clear();  // Sets all pixels to 0 (off)
  pixels.show();   // Apply changes

  if (betaPerc >= 0 && betaPerc <= 2) {
  showPixels(1);
  } 
  else if (betaPerc > 2 && betaPerc <= 5) {
  showPixels(2);
  } 
  else if (betaPerc > 5 && betaPerc <= 9) {
  showPixels(3);
  } 
  else if (betaPerc > 9 && betaPerc <= 12) {
  showPixels(4);
  } 
  else if (betaPerc > 12 && betaPerc <= 15) {
  showPixels(5);
  } 
  else if (betaPerc > 15) {
  showPixels(6);
  }

  // Update LEDs once at the end (instead of after each change)
  pixels.show();
  
  if(betaPerc>16.0)
  {
    // 1) just went above threshold → start timing
    if (!timerRunning)
    {
      startTimer();
    }
    // 2) already timing → has 1 s elapsed?
    else if ((millis() - timer1Start) >= 1000UL)
    {
      toggleSwitch(); // flip switch
      stopTimer(); // reset for next detection
    }
  }
  else
  {
    // dropped below before 4 s → cancel timing
    if (timerRunning)
    {
      stopTimer();
    }
  }
  

  // print: delta%, theta%, alpha%, beta%, gamma%, peakHz
  Serial.print((smoothedPowers.delta/T)*100, 1); Serial.print(',');
  Serial.print((smoothedPowers.theta/T)*100, 1); Serial.print(',');
  Serial.print((smoothedPowers.alpha/T)*100, 1); Serial.print(',');
  Serial.print((smoothedPowers.beta/ T)*100, 1); Serial.print(',');
  Serial.print((smoothedPowers.gamma/T)*100, 1); Serial.print(',');
  Serial.println(peakHz, 1);
}

// ----------------- SETUP & LOOP -----------------
void setup() {
  Serial.begin(BAUD_RATE);
  // while(!Serial) { delay(1); }
  pinMode(INPUT_PIN, INPUT);
  pinMode(7, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  IrSender.begin(IR_SEND_PIN, DISABLE_LED_FEEDBACK, 0); // initialize the IR sender (RMT)

  pixels.begin();  // Initialize NeoPixel library (REQUIRED)

  initFFT();

  // set initial LED state
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  static long timer = 0;
  timer -= dt;
  if(timer <= 0){
    timer += 1000000L / SAMPLE_RATE;
    int raw = analogRead(INPUT_PIN);
    float filt = EEGFilter(Notch(raw));
    inputBuffer[idx++] = filt;
  }

  if(idx >= FFT_SIZE){
    processFFT();
    idx = 0;
  }
}