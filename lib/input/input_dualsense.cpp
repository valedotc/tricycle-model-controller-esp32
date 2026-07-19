#include "input_dualsense.h"
#include <ps5Controller.h>
#include <cmath>

/*!
 * \file input_dualsense.cpp
 * \brief DualSenseInput implementation backed by the esp-ps5 library.
 *
 * \note The esp-ps5 library exposes a single global \c ps5 instance, so
 *       this class effectively wraps a singleton. Only one DualSenseInput
 *       should exist at a time.
 */

void DualSenseInput::begin() {
  ps5.begin(20); /* Try the MAC saved in NVS first, otherwise scan for 20 s. */
}

float DualSenseInput::applyDeadzone(float v) const {
  if (std::fabs(v) < _deadzone) return 0.0f;
  /* Rescale the remainder across the full range to avoid a jump at the edge. */
  float sign = (v > 0) ? 1.0f : -1.0f;
  return sign * (std::fabs(v) - _deadzone) / (1.0f - _deadzone);
}

void DualSenseInput::update() {
  _connected = ps5.isConnected();
  if (!_connected) {
    _state = InputState{}; /* Zero everything: safety, no phantom inputs. */
    return;
  }

  /* Raw axes are int8_t in [-127, 127]; triggers are uint8_t in [0, 255]. */
  _state.lx = applyDeadzone(ps5.lx / 127.0f);
  _state.ly = applyDeadzone(ps5.ly / 127.0f);
  _state.rx = applyDeadzone(ps5.rx / 127.0f);
  _state.ry = applyDeadzone(ps5.ry / 127.0f);
  _state.l2 = ps5.l2 / 255.0f;
  _state.r2 = ps5.r2 / 255.0f;

  _state.cross    = ps5.cross;
  _state.circle   = ps5.circle;
  _state.square   = ps5.square;
  _state.triangle = ps5.triangle;
  _state.l1       = ps5.l1;
  _state.r1       = ps5.r1;
  _state.dpadUp    = ps5.up;
  _state.dpadDown  = ps5.down;
  _state.dpadLeft  = ps5.left;
  _state.dpadRight = ps5.right;
}

bool DualSenseInput::isConnected() const { return _connected; }

void DualSenseInput::setLightbar(uint8_t r, uint8_t g, uint8_t b) {
  /* The esp-ps5 firmware tears the link down if output frames are sent
     faster than ~10 ms apart, so emit only when the color actually
     changed. Battery color changes rarely, so this stays well within the
     limit even when called every control cycle. */
  static uint8_t lastR = 1, lastG = 1, lastB = 1;   // impossibili all'avvio
  if (r == lastR && g == lastG && b == lastB) return;
  lastR = r; lastG = g; lastB = b;
  ps5.lightbar(r, g, b).send();
}
