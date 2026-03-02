#include <Arduino.h>

const uint8_t SENSOR_PINS[] = {10, 13};
const uint8_t SENSOR_COUNT = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);
const unsigned long SAMPLE_INTERVAL_MS = 2;
const uint8_t INTEGRATOR_MAX = 12; // 12 * 2 ms = about 24 ms debounce

bool debouncedState[SENSOR_COUNT];
uint8_t integrator[SENSOR_COUNT];
unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Touch sensors ready");

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);

    bool initial = digitalRead(SENSOR_PINS[i]);
    debouncedState[i] = initial;
    integrator[i] = (initial == LOW) ? INTEGRATOR_MAX : 0;
  }
}

void loop() {
  unsigned long now = millis();
  if ((now - lastSampleTime) < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleTime = now;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    bool raw = digitalRead(SENSOR_PINS[i]);

    // Integrator debounce: noisy reads move confidence gradually.
    if (raw == LOW) {
      if (integrator[i] < INTEGRATOR_MAX) {
        integrator[i]++;
      }
    } else {
      if (integrator[i] > 0) {
        integrator[i]--;
      }
    }

    bool newDebounced = debouncedState[i];
    if (integrator[i] == INTEGRATOR_MAX) {
      newDebounced = LOW;
    } else if (integrator[i] == 0) {
      newDebounced = HIGH;
    }

    if (newDebounced != debouncedState[i]) {
      debouncedState[i] = newDebounced;
      if (newDebounced == LOW) {
        Serial.print("Sensor ");
        Serial.print(SENSOR_PINS[i]);
        Serial.println(" touched");
      } else {
        Serial.print("Sensor ");
        Serial.print(SENSOR_PINS[i]);
        Serial.println(" released");
      }
    }
  }
}