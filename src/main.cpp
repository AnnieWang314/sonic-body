#include <Arduino.h>
#include <CapacitiveSensor.h>

// CapacitiveSensor wiring:
// - Large resistor between pin 2 (send) and pin 4 (receive)
// - Touch wire connected to pin 4 (receive)
const uint8_t CAP_SEND_PIN = 2;
const uint8_t CAP_RECEIVE_PIN = 4;
CapacitiveSensor capSensor(CAP_SEND_PIN, CAP_RECEIVE_PIN);

const uint8_t CAP_SAMPLES = 20;
const uint8_t REQUIRED_CONSECUTIVE_READS = 3;
const unsigned long READ_INTERVAL_MS = 15;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 500;

// Thresholds are relative to baseline (delta = raw - baseline).
const long TOUCH_DELTA_THRESHOLD = 45;
const long RELEASE_DELTA_THRESHOLD = 20;
const uint8_t BASELINE_ALPHA_DIV = 16; // Larger = slower baseline tracking

bool isTouched = false;
uint8_t touchConfidence = 0;
uint8_t releaseConfidence = 0;
unsigned long lastReadMs = 0;
unsigned long lastDebugPrintMs = 0;
long baseline = 0;
bool baselineInitialized = false;

void setup() {
  Serial.begin(115200);
  capSensor.set_CS_AutocaL_Millis(0xFFFFFFFF); // Avoid auto-resetting baseline

  Serial.println("Capacitive touch ready (pins 2 -> 4)");
  Serial.println("Touch wire on pin 4 to trigger.");
}

void loop() {
  unsigned long nowMs = millis();
  if ((nowMs - lastReadMs) < READ_INTERVAL_MS) {
    return;
  }
  lastReadMs = nowMs;

  long raw = capSensor.capacitiveSensor(CAP_SAMPLES);
  if (raw < 0) {
    // Library returns negative on timeout/error.
    return;
  }

  if (!baselineInitialized) {
    baseline = raw;
    baselineInitialized = true;
  }

  long delta = raw - baseline;

  // Track ambient drift only while released.
  if (!isTouched) {
    baseline += (raw - baseline) / BASELINE_ALPHA_DIV;
  }

  if (!isTouched) {
    if (delta >= TOUCH_DELTA_THRESHOLD) {
      touchConfidence++;
      releaseConfidence = 0;
      if (touchConfidence >= REQUIRED_CONSECUTIVE_READS) {
        isTouched = true;
        touchConfidence = 0;
        Serial.println("Wire touched");
      }
    } else {
      touchConfidence = 0;
    }
  } else {
    if (delta <= RELEASE_DELTA_THRESHOLD) {
      releaseConfidence++;
      touchConfidence = 0;
      if (releaseConfidence >= REQUIRED_CONSECUTIVE_READS) {
        isTouched = false;
        releaseConfidence = 0;
        Serial.println("Wire released");
      }
    } else {
      releaseConfidence = 0;
    }
  }

  if ((nowMs - lastDebugPrintMs) >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMs = nowMs;
    Serial.print("Raw: ");
    Serial.print(raw);
    Serial.print(" | Baseline: ");
    Serial.print(baseline);
    Serial.print(" | Delta: ");
    Serial.print(delta);
    Serial.print(" | State: ");
    Serial.println(isTouched ? "TOUCHED" : "RELEASED");
  }
}