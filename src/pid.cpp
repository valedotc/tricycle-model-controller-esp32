#include "pid.h"
#include <Arduino.h>

Pid::Pid(float kp, float ki, float kd, float outLimit)
  : _kp(kp), _ki(ki), _kd(kd), _outLimit(outLimit),
    _integral(0.0f), _prevError(0.0f) {}

float Pid::compute(float setpoint, float measured, float dt) {
  if (dt <= 0.0f) return 0.0f;

  float error = setpoint - measured;

  /* Proportional. */
  float p = _kp * error;

  /* Integral with clamping (anti-windup): keep the accumulated term
     within the output limits so it can't grow unbounded while saturated. */
  _integral += error * dt;
  float iMax = (_ki > 0.0f) ? (_outLimit / _ki) : 0.0f;
  _integral = constrain(_integral, -iMax, iMax);
  float i = _ki * _integral;

  /* Derivative on the error. */
  float d = _kd * (error - _prevError) / dt;
  _prevError = error;

  float out = p + i + d;
  return constrain(out, -_outLimit, _outLimit);
}

void Pid::reset() {
  _integral  = 0.0f;
  _prevError = 0.0f;
}

void Pid::setGains(float kp, float ki, float kd) {
  _kp = kp; _ki = ki; _kd = kd;
}