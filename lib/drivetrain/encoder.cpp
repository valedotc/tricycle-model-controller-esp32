#include "encoder.h"
#include <Arduino.h>

/* Quadrature transition table, indexed by (prevState << 2 | newState) where
   state = (A << 1 | B). +1/-1 for the two valid single-step directions, 0 for
   "no change" and for illegal double transitions (a missed edge). Kept in DRAM
   so the ISR can read it even while flash is busy. */
static const int8_t DRAM_ATTR kQuadLUT[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

Encoder::Encoder(uint8_t pinA, uint8_t pinB, bool invert)
  : _pinA(pinA), _pinB(pinB), _invert(invert), _ticks(0), _state(0) {}

void Encoder::begin() {
  pinMode(_pinA, INPUT);
  pinMode(_pinB, INPUT);
  /* Seed the decoder with the current line state so the first edge doesn't
     produce a spurious count. */
  _state = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
  /* Full x4 quadrature: interrupt on BOTH edges of BOTH channels. */
  attachInterruptArg(digitalPinToInterrupt(_pinA), isrTrampoline, this, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(_pinB), isrTrampoline, this, CHANGE);
}

void IRAM_ATTR Encoder::handleEdge() {
  /* Read the new A/B state, look the transition up in the quadrature table. */
  uint8_t s   = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
  int     step = kQuadLUT[(_state << 2) | s];
  _state = s;
  if (_invert) step = -step;
  _ticks += step;
}

void IRAM_ATTR Encoder::isrTrampoline(void* arg) {
  static_cast<Encoder*>(arg)->handleEdge();
}

long Encoder::ticks() const {
  /* 32-bit read is atomic on the ESP32, so no critical section needed. */
  return _ticks;
}

void Encoder::reset() {
  noInterrupts();
  _ticks = 0;
  interrupts();
}