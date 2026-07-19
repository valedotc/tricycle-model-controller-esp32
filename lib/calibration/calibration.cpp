#include "calibration.h"
#include <Arduino.h>
#include <cstring>
#include <cstdlib>

Calibration::Calibration(Actuators& actuators) : _actuators(actuators) {}

// ----------------------------------------------------------------------------
//  Parser comandi seriali per tuning al volo. Niente String: accumulo i
//  caratteri in un buffer fisso e parso alla newline.
// ----------------------------------------------------------------------------
void Calibration::pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      _line[_len] = '\0';
      _len = 0;
      handleLine(_line);
    } else if (c != '\r' && _len < sizeof(_line) - 1) {
      _line[_len++] = c;
    }
  }
}

void Calibration::handleLine(const char* line) {
  while (*line == ' ') line++;             // trim sinistro
  if (*line == '\0') return;

  char  cmd = line[0];
  float val = strtof(line + 1, nullptr);

  if (strcmp(line, "max") == 0) {
    _rawMaxMode = true;
    Serial.println("[MAX] setSpeed(1.0) diretto - leggi i tick/s a regime");
  } else if (strcmp(line, "run") == 0) {
    _rawMaxMode = false;
    Serial.println("[RUN] modalita' PID");
  } else if (strcmp(line, "stop") == 0) {
    _rawMaxMode = false;
    _setpointFrac = 0.0f;
    _actuators.stop();
    Serial.println("[STOP]");
  } else if (cmd == 'p') {
    _kp = val;
    _actuators.setPidGains(_kp, _ki, _kd);
    Serial.printf("[GAIN] Kp=%.5f Ki=%.5f Kd=%.5f\n", _kp, _ki, _kd);
  } else if (cmd == 'i') {
    _ki = val;
    _actuators.setPidGains(_kp, _ki, _kd);
    _actuators.resetPids();
    Serial.printf("[GAIN] Kp=%.5f Ki=%.5f Kd=%.5f\n", _kp, _ki, _kd);
  } else if (cmd == 'd') {
    _kd = val;
    _actuators.setPidGains(_kp, _ki, _kd);
    Serial.printf("[GAIN] Kp=%.5f Ki=%.5f Kd=%.5f\n", _kp, _ki, _kd);
  } else if (cmd == 's') {
    _setpointFrac = constrain(val, -1.0f, 1.0f);
    _rawMaxMode = false;
    Serial.printf("[SET] setpoint = %.2f (%.0f tick/s)\n",
                  _setpointFrac, _setpointFrac * _actuators.maxTicksPerSec());
  }
}

void Calibration::update(float dt) {
  pollSerial();

  if (_rawMaxMode) {
    // Bypassa il PID: comando diretto a manetta per misurare tick/s.
    _actuators.setRawSpeed(1.0f);
    if (!_rawMaxInit) {
      _lastTicksL = _actuators.ticksLeft();
      _lastTicksR = _actuators.ticksRight();
      _rawMaxInit = true;
    }
    float vl = (_actuators.ticksLeft()  - _lastTicksL) / dt;
    float vr = (_actuators.ticksRight() - _lastTicksR) / dt;
    _lastTicksL = _actuators.ticksLeft();
    _lastTicksR = _actuators.ticksRight();
    Serial.printf("[MAX] tick/s  L=%.0f  R=%.0f\n", vl, vr);
    return;
  }

  float target = _setpointFrac * _actuators.maxTicksPerSec();
  _actuators.setWheelTargets(target, target);
  _actuators.update(dt);

  Serial.printf("SET=%.0f  MIS L=%.0f R=%.0f\n",
                target, _actuators.measuredLeft(), _actuators.measuredRight());
}
