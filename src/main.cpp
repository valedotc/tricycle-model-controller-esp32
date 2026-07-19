#include <Arduino.h>
#include "input_dualsense.h"
#include "battery.h"
#include "actuators.h"
#include "arbiter.h"
#include "calibration.h"

// Metti true per calibrare (setpoint fisso da seriale, niente controller).
// Metti false per guidare col DualSense. Istruzioni in calibration.h.
// (Da non confondere con TEST_MODE, il build flag che disabilita
//  l'attuazione reale: vedi actuators.h e platformio.ini.)
static const bool CALIB_MODE = false;

// Tutte le istanze sono globali: nessuna allocazione dinamica.
DualSenseInput input;
BatteryMonitor battery;
Actuators      actuators;
Arbiter        arbiter(input, battery, actuators);
Calibration    calibration(actuators);

// Loop di controllo a rate fisso.
static const uint32_t CONTROL_PERIOD_MS = 20;   // 50 Hz
uint32_t lastControl = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  battery.begin();
  actuators.begin();

  if (CALIB_MODE) {
    Serial.println("\n[BOOT] MODALITA' CALIBRAZIONE");
    Serial.println("comandi: p/i/d <val>, s <frazione>, max, run, stop");
  } else {
    Serial.println("\n[BOOT] MODALITA' GUIDA - pairing DualSense (PS + Share)");
    input.begin();
  }
}

void loop() {
  if (!CALIB_MODE) input.update();

  uint32_t now = millis();
  if (now - lastControl < CONTROL_PERIOD_MS) return;
  float dt = (now - lastControl) / 1000.0f;
  lastControl = now;

  if (CALIB_MODE) calibration.update(dt);
  else            arbiter.update(dt);
}
