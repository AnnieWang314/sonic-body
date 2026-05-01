#include <Arduino.h>
#include <CapacitiveSensor.h>

// === Multi-yarn Capacitive Touch Setup ===
// Use ONE shared send pin for all yarn channels.
// This is the recommended topology for multi-touch with CapacitiveSensor.
const uint8_t NUM_YARNS = 6;

const uint8_t CAP_SEND_PIN = 2;
const uint8_t CAP_RECEIVE_PINS[NUM_YARNS] = {3, 4, 5, 6, 7, 9};

// Create CapacitiveSensor objects for all yarns (shared send pin).
CapacitiveSensor capSensors[NUM_YARNS] = {
    CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_PINS[0]),
    CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_PINS[1]),
    CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_PINS[2]),
    CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_PINS[3]),
    CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_PINS[4]),
    CapacitiveSensor(CAP_SEND_PIN, CAP_RECEIVE_PINS[5])
};

const uint8_t CAP_SAMPLES = 12;
const uint8_t TOUCH_CONSECUTIVE_READS = 2;
const uint8_t RELEASE_CONSECUTIVE_READS = 5;
const unsigned long READ_INTERVAL_MS = 8;
const unsigned long SENSOR_TIMEOUT_MS = 10;

// Thresholds are relative to baseline (delta = raw - baseline).
const long TOUCH_DELTA_THRESHOLD = 18;
const long RELEASE_DELTA_THRESHOLD = 10;
const uint8_t BASELINE_ALPHA_DIV = 16; // Larger = slower baseline tracking
const long INVALID_READ_SENTINEL = -9999;

// Per-yarn touch detection state
struct YarnState {
  bool isTouched = false;
  uint8_t touchConfidence = 0;
  uint8_t releaseConfidence = 0;
  long baseline = 0;
  bool baselineInitialized = false;
};

YarnState yarnStates[NUM_YARNS];
long lastRaw[NUM_YARNS] = {};
long lastDelta[NUM_YARNS] = {};
bool lastReadValid[NUM_YARNS] = {};
unsigned long timeoutCount[NUM_YARNS] = {};

unsigned long lastReadMs = 0;

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_YARNS; ++i) {
    capSensors[i].set_CS_AutocaL_Millis(0xFFFFFFFF); // Avoid auto-resetting baseline
    capSensors[i].set_CS_Timeout_Millis(SENSOR_TIMEOUT_MS);
  }
}

void loop() {
  unsigned long nowMs = millis();
  if ((nowMs - lastReadMs) < READ_INTERVAL_MS) {
    return;
  }
  lastReadMs = nowMs;

  long rawVals[NUM_YARNS] = {};
  long deltas[NUM_YARNS] = {};
  bool valid[NUM_YARNS] = {};
  bool freezeBaselines = false;

  // Pass 1: read every yarn first, then decide freeze policy.
  for (uint8_t i = 0; i < NUM_YARNS; ++i) {
    long raw = capSensors[i].capacitiveSensor(CAP_SAMPLES);
    if (raw < 0) {
      // Library returns negative on timeout/error. Keep previous state.
      lastReadValid[i] = false;
      timeoutCount[i]++;
      continue;
    }

    valid[i] = true;
    lastReadValid[i] = true;

    YarnState &state = yarnStates[i];
    if (!state.baselineInitialized) {
      state.baseline = raw;
      state.baselineInitialized = true;
    }

    rawVals[i] = raw;
    deltas[i] = raw - state.baseline;
    lastRaw[i] = raw;
    lastDelta[i] = deltas[i];

    // Freeze all baselines if any yarn is touched OR clearly rising toward touch.
    if (state.isTouched || deltas[i] >= TOUCH_DELTA_THRESHOLD) {
      freezeBaselines = true;
    }
  }

  // Pass 2: baseline update and state machine use synchronized frame data.
  for (uint8_t i = 0; i < NUM_YARNS; ++i) {
    if (!valid[i]) {
      continue;
    }

    YarnState &state = yarnStates[i];
    long raw = rawVals[i];
    long delta = deltas[i];

    if (!freezeBaselines && !state.isTouched) {
      state.baseline += (raw - state.baseline) / BASELINE_ALPHA_DIV;
      delta = raw - state.baseline;
      lastDelta[i] = delta;
    }

    if (!state.isTouched) {
      if (delta >= TOUCH_DELTA_THRESHOLD) {
        state.touchConfidence++;
        state.releaseConfidence = 0;
        if (state.touchConfidence >= TOUCH_CONSECUTIVE_READS) {
          state.isTouched = true;
          state.touchConfidence = 0;
        }
      } else {
        state.touchConfidence = 0;
      }
    } else {
      if (delta <= RELEASE_DELTA_THRESHOLD) {
        state.releaseConfidence++;
        state.touchConfidence = 0;
        if (state.releaseConfidence >= RELEASE_CONSECUTIVE_READS) {
          state.isTouched = false;
          state.releaseConfidence = 0;
        }
      } else {
        state.releaseConfidence = 0;
      }
    }
  }

  for (uint8_t i = 0; i < NUM_YARNS; ++i) {
    // Output live delta so tuning/debugging works even before touch latching.
    long out = lastReadValid[i] ? lastDelta[i] : INVALID_READ_SENTINEL;
    Serial.print(out);
    if (i < (NUM_YARNS - 1)) {
      Serial.print(" ");
    }
  }
  Serial.println();
}