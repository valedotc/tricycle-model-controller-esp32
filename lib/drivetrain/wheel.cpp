#include "wheel.h"

Wheel::Wheel(Motor& motor, Encoder& enc, Pid& pid)
  : _motor(motor), _enc(enc), _pid(pid),
    _target(0.0f), _lastTicks(0), _measuredSpeed(0.0f) {}

void Wheel::begin() {
  _motor.begin();
  _enc.begin();
  _lastTicks = _enc.ticks();
}

void Wheel::setTargetSpeed(float ticksPerSec) {
  _target = ticksPerSec;
}

void Wheel::update(float dt) {
  /* Measure: speed = delta ticks / dt. */
  long now = _enc.ticks();
  _measuredSpeed = (float)(now - _lastTicks) / dt;
  _lastTicks = now;

  /* Regulate: PID outputs a normalized motor command in [-1, 1]. */
  float cmd = _pid.compute(_target, _measuredSpeed, dt);
  _motor.setSpeed(cmd);
}

void Wheel::stop() {
  _target = 0.0f;
  _pid.reset();
  _motor.stop();
}