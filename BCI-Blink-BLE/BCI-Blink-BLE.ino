// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

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

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "esp_dsp.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE       512           // samples per second
#define FFT_SIZE          512           // must be a power of two
#define BAUD_RATE         115200
#define INPUT_PIN         A0
#define PIXEL_PIN         15
#define PIXEL_COUNT       6

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

#define BLINK_SERVICE_UUID        "6910123a-eb0d-4c35-9a60-bebe1dcb549d"
#define BLINK_CHAR_UUID           "5f4f1107-7fc1-43b2-a540-0aa1a9f1ce78"

BLEServer*           pBleServer     = nullptr;
BLEService*          pBlinkService  = nullptr;
BLECharacteristic*   pBlinkChar     = nullptr;

// Onboard Neopixel at PIXEL_PIN
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ----------------- USER CONFIGURATION -----------------
// Add this after existing defines:
#define ENVELOPE_WINDOW_MS 100  // Smoothing window in milliseconds
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

const unsigned long BLINK_DEBOUNCE_MS   = 250;   // minimal spacing between individual blinks
const unsigned long DOUBLE_BLINK_MS     = 600;   // max time between the two blinks
unsigned long lastBlinkTime     = 0;             // time of most recent blink
unsigned long firstBlinkTime    = 0;             // time of the first blink in a pair
int         blinkCount         = 0;             // how many valid blinks so far (0–2)
bool        menu           = LOW;            // current LED state

int menuIndex = 0;
const unsigned long MENU_TIMEOUT_MS = 20000;  // 20 seconds
unsigned long menuStartTime = 0;
static bool clientConnected = false;

bool betaEventFired = false;
float BlinkThreshold = 75.0;

// ----------------- CALIBRATION VARIABLES -----------------
enum ProgramState { STATE_CALIBRATING, STATE_RUNNING };
ProgramState programState = STATE_CALIBRATING;
const unsigned long CALIBRATION_DURATION_MS = 10000; // 10 seconds
unsigned long calibrationStartTime = 0;
float maxBetaPct = 0.0;
float betaThreshold = 0.0; // Will be set to 60% of maxBetaPct
const float THRESHOLD_MULTIPLIER = 0.6;
unsigned long lastBlinkUpdate = 0;
unsigned long lastChaseUpdate = 0;
int chasePosition = 0;  // Current position in LED chase

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    clientConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    clientConnected = false;
    Serial.println("BLE client disconnected");
    // restart advertising so another client can connect
    pServer->getAdvertising()->start();
  }
};

// ----------------- BUFFERS & TYPES -----------------
// Add these after existing buffers:
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;

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

// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 5.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float highpass(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.91327599*z1 - 0.91688335*z2;
    output = 0.95753983*x + -1.91507967*z1 + 0.95753983*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

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

// ----------------- ENVELOPE FUNCTION -----------------
float updateEEGEnvelope(float sample) {
  float absSample = fabs(sample);  // Rectify EEG signal

  // Update circular buffer and running sum
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;

  return envelopeSum / ENVELOPE_WINDOW_SIZE;  // Return moving average
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

  float betaPct = (smoothedPowers.beta / T) * 100.0;

  // During calibration, track max beta percentage
  if (programState == STATE_CALIBRATING) {
    if (betaPct > maxBetaPct) {
      maxBetaPct = betaPct;
    }
    return; // Skip rest of processing during calibration
  }

  // Normal running state processing
  if (menu) {
    if (betaPct > betaThreshold && !betaEventFired) {
      // --- your one-time action ---
      pixels.setPixelColor(menuIndex-1, pixels.Color(0,10,0));
      pixels.show();

      uint8_t cmd[2] = { (uint8_t)'A', (uint8_t)menuIndex };
      if (clientConnected) {
        pBlinkChar->setValue(cmd, 2);
        pBlinkChar->notify();
      }

      betaEventFired = true;   // mark as handled
    }
  }
}

void showPixels()
{
  for(int i=0; i<PIXEL_COUNT; i++)
  {
    pixels.setPixelColor(i, pixels.Color(5, 5, 5)); // Level 1 to 6
  }
  pixels.show();
}

// ----------------- SETUP & LOOP -----------------
void setup() {
  Serial.begin(BAUD_RATE);
  delay(100);
  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  pixels.begin();
  pixels.clear();
  pixels.show();

  // Start calibration
  Serial.println("Starting calibration...");
  programState = STATE_CALIBRATING;
  calibrationStartTime = millis();
  lastBlinkUpdate = millis();

  // --- BLE init ---
  BLEDevice::init("ESP32C6_EEG");                    // device name
  pBleServer   = BLEDevice::createServer();
  pBleServer->setCallbacks(new MyServerCallbacks());
  pBlinkService = pBleServer->createService(BLINK_SERVICE_UUID);

  // create a NOTIFY-only characteristic:
  pBlinkChar = pBlinkService->createCharacteristic(
                  BLINK_CHAR_UUID,
                  BLECharacteristic::PROPERTY_NOTIFY
               );
  // ensure clients can subscribe:
  pBlinkChar->addDescriptor(new BLE2902());

  pBlinkService->start();

  // start advertising so a phone/PC can connect:
  BLEAdvertising* pAdvertising = pBleServer->getAdvertising();
  pAdvertising->start();
  Serial.println(">> BLE Advertising started");

  initFFT();
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
    float filtered = highpass(filt);
    currentEEGEnvelope = updateEEGEnvelope(filtered);
    
    inputBuffer[idx++] = filt;
  }

  // Process FFT when buffer is full
  if(idx >= FFT_SIZE){
    processFFT();
    idx = 0;
  }

  // Handle calibration state
  if (programState == STATE_CALIBRATING) {
    unsigned long currentTime = millis();
    
    // Update chasing LED effect (every 100ms)
    if (currentTime - lastChaseUpdate > 100) {
      lastChaseUpdate = currentTime;
      
      // Clear all LEDs
      pixels.clear();
      
      // Set current position to white
      pixels.setPixelColor(chasePosition, pixels.Color(100, 100, 100));  // White
      
      // Move to next position
      chasePosition = (chasePosition + 1) % PIXEL_COUNT;
      
      pixels.show();
    }

    // Check if calibration is complete
    if (currentTime - calibrationStartTime > CALIBRATION_DURATION_MS) {
      programState = STATE_RUNNING;
      pixels.clear();
      pixels.show();
      
      // Set threshold to 60% of max observed beta
      betaThreshold = maxBetaPct * THRESHOLD_MULTIPLIER;
      
      Serial.print("Calibration complete. Max Beta: ");
      Serial.print(maxBetaPct);
      Serial.print("%, Threshold: ");
      Serial.print(betaThreshold);
      Serial.println("%");
    }
    return; // Skip rest of loop during calibration
  }

  // Normal running state processing
  unsigned long nowMs = millis();

  // 1) Did we cross threshold and respect per‑blink debounce?
  if (currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
    lastBlinkTime = nowMs;    // mark this blink

    // 2) Count it
    if (blinkCount == 0) {
      // first blink of the pair
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
      // second blink in time → toggle LED
      if(!menu)
      {
        menu = !menu;
        menuStartTime = millis();      // start our 10s countdown
        Serial.println(0);
        if (clientConnected) 
        {
          uint8_t cmd = 0;
          pBlinkChar->setValue(&cmd, 1);
          // 2) Send the notification
          pBlinkChar->notify();
        }
        showPixels();
        menuIndex = 0;
        blinkCount = 0;         // reset for next pair
      }
      else
      {
        betaEventFired = false;
        menuStartTime = millis();      // restart the 10 second countdown
        if(menuIndex==6)
        {
          menuIndex=1;
        }
        else
        {
          menuIndex++;
        }
        Serial.println(menuIndex);
        uint8_t cmd[2];
        
        cmd[0] = (uint8_t)('S');
        cmd[1] = menuIndex;

        if (clientConnected) 
        {
          pBlinkChar->setValue(cmd, 2); // send exactly 2 bytes
          pBlinkChar->notify();
        }

        showPixels();
        pixels.setPixelColor(menuIndex-1, pixels.Color(0, 0 , 10));
        pixels.show();
        blinkCount = 0;
      }
    }
    else {
      // either too late or extra blink → restart sequence
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
  }

  // 3) Timeout: if we never got the second blink in time, reset
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS) {
    blinkCount = 0;
  }

  unsigned long resetTimer = millis();
  if (menu && (resetTimer - menuStartTime > MENU_TIMEOUT_MS)) {
    // 20 seconds elapsed with no double-blink
    menu = false;
    Serial.println("Menu timed out → off");
    // optionally send a notify to tell the client:
    if (clientConnected) {
      uint8_t cmd = 127;         // your “menu off” code
      pBlinkChar->setValue(&cmd, 1);
      pBlinkChar->notify();
    }
    // you might also clear pixels:
    pixels.clear();
    pixels.show();
  }
}