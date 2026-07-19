#include "actuators.h"
#include <Arduino.h>

// ----------------------------------------------------------------------------
//  Mappatura hardware.
//  Encoder DX: fase B spostata da GPIO39 (VN) a D27 per liberare VN come
//  ingresso ADC1 della batteria (vedi battery.h).
//
//  PID: output limitato a 1.0 (Motor::setSpeed vuole [-1, 1]).
//  Tarati a vuoto (ruote in aria) il 2026-07-19 con encoder in quadratura x4:
//  Kp=0.0010 Ki=0.0025 Kd=0. Kd tenuto a 0: il termine derivativo amplifica il
//  rumore residuo (peggiora anche a x4). Kp basso di proposito: sopra ~0.0015 a
//  bassa velocita' compare un limit-cycle stick-slip. Sotto carico potrebbe
//  servire un filo piu' di Kp/Ki.
// ----------------------------------------------------------------------------
Actuators::Actuators()
  : _motorL(25, 26, 0),
    _motorR(33, 32, 1, /*invert=*/true),
    _encL(34, 35),
    _encR(36, 27, /*invert=*/true),
    _pidL(0.0010f, 0.0025f, 0.0f, 1.0f),
    _pidR(0.0010f, 0.0025f, 0.0f, 1.0f),
    _wheelL(_motorL, _encL, _pidL),
    _wheelR(_motorR, _encR, _pidR) {}

void Actuators::begin() {
  _wheelL.begin();
  _wheelR.begin();
}

void Actuators::setWheelTargets(float ticksL, float ticksR) {
  // Clamp sui limiti fisici: nessun setpoint oltre la velocita' massima.
  _wheelL.setTargetSpeed(constrain(ticksL, -kMaxTicksPerSec, kMaxTicksPerSec));
  _wheelR.setTargetSpeed(constrain(ticksR, -kMaxTicksPerSec, kMaxTicksPerSec));
}

void Actuators::update(float dt) {
  _wheelL.update(dt);
  _wheelR.update(dt);
}

void Actuators::stop() {
  _wheelL.stop();
  _wheelR.stop();
}

void Actuators::setRawSpeed(float speed) {
  _motorL.setSpeed(speed);
  _motorR.setSpeed(speed);
}

void Actuators::setPidGains(float kp, float ki, float kd) {
  _pidL.setGains(kp, ki, kd);
  _pidR.setGains(kp, ki, kd);
}

void Actuators::resetPids() {
  _pidL.reset();
  _pidR.reset();
}
