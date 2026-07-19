#include "battery.h"
#include <Arduino.h>

const int   BatteryMonitor::kPin         = 39;
const float BatteryMonitor::kDivRatio    = 3.134f;  // (R1+R2)/R2 = 150k/50k. Calibra col
                                                    // multimetro se le tolleranze scostano.
const float BatteryMonitor::kVFull       = 8.4f;
const float BatteryMonitor::kVEmpty      = 6.0f;
const float BatteryMonitor::kVCutoff     = 6.2f;    // sotto: blocco motori (margine sul 6.0)
const float BatteryMonitor::kVCutoffHyst = 0.4f;    // risale sopra cutoff+hyst per sbloccare

void BatteryMonitor::begin() {
  // ADC batteria: 12 bit, attenuazione piena per arrivare fino a ~3.3V sul pin.
  analogReadResolution(12);
  analogSetPinAttenuation(kPin, ADC_11db);
}

float BatteryMonitor::readVoltage() const {
  long sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(kPin); delayMicroseconds(200); }
  float vPin = (sum / 16.0f / 4095.0f) * 3.3f;
  return vPin * kDivRatio;
}

// Chiamata a ogni ciclo di controllo; campiona a ~1 Hz e solo quando i motori
// sono quasi fermi. Aggiorna il lockout con isteresi.
bool BatteryMonitor::update(bool motorsIdle) {
  if (millis() - _lastCheck < 1000) return false;
  _lastCheck = millis();
  if (!motorsIdle) return false;

  float v = readVoltage();
  if (v < kVCutoff)                     _lockout = true;
  else if (v > kVCutoff + kVCutoffHyst) _lockout = false;

  _voltage = v;
  _percent = constrain((int)((v - kVEmpty) / (kVFull - kVEmpty) * 100.0f), 0, 100);

  Serial.printf("[BATT] %.2fV  %d%%  %s\n", v, _percent, _lockout ? "LOCKOUT" : "ok");
  return true;
}
