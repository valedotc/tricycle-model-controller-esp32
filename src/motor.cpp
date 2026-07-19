#include "motor.h"
#include <Arduino.h>

Motor::Motor(uint8_t in1, uint8_t in2, uint8_t pwmChannel, bool invert)
  : _in1(in1), _in2(in2), _pwmChannel(pwmChannel), _invert(invert) {}

void Motor::begin() {
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  ledcSetup(_pwmChannel, kPwmFreqHz, kPwmResBits);
  stop();
}

void Motor::setSpeed(float speed) {
  if (_invert) speed = -speed;
  speed = constrain(speed, -1.0f, 1.0f);

  const int maxDuty = (1 << kPwmResBits) - 1;   // 255
  int duty = (int)(fabsf(speed) * maxDuty);

  if (speed > 0.0f) {
    /* Forward: IN2 low, PWM on IN1. */
    ledcDetachPin(_in2);
    digitalWrite(_in2, LOW);
    ledcAttachPin(_in1, _pwmChannel);
    ledcWrite(_pwmChannel, duty);
  } else if (speed < 0.0f) {
    /* Reverse: IN1 low, PWM on IN2. */
    ledcDetachPin(_in1);
    digitalWrite(_in1, LOW);
    ledcAttachPin(_in2, _pwmChannel);
    ledcWrite(_pwmChannel, duty);
  } else {
    stop();
  }
}

void Motor::stop() {
  ledcDetachPin(_in1);
  ledcDetachPin(_in2);
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, LOW);
}