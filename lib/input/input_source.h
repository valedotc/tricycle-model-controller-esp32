#pragma once
#include <cstdint>

/*!
 * \file input_source.h
 * \brief Abstract input source interface (\ref IInputSource) and its state.
 *
 * Decouples the drive logic from the physical controller: the arbiter only
 * talks to \ref IInputSource, so a fake/scripted implementation can replace
 * the real DualSense in unit tests or on the bench.
 */

/*!
 * \brief Snapshot of the operator inputs at a single point in time.
 *
 * Analog sticks are normalized to [-1.0, 1.0] with the deadzone already
 * applied; triggers are normalized to [0.0, 1.0]. All fields are zeroed
 * while the source is disconnected.
 */
struct InputState {
  float lx = 0; /*!< Left stick X, [-1.0, 1.0]. */
  float ly = 0; /*!< Left stick Y, [-1.0, 1.0] (up is negative). */
  float rx = 0; /*!< Right stick X, [-1.0, 1.0]. */
  float ry = 0; /*!< Right stick Y, [-1.0, 1.0] (up is negative). */
  float l2 = 0; /*!< Left analog trigger, [0.0, 1.0]. */
  float r2 = 0; /*!< Right analog trigger, [0.0, 1.0]. */

  bool cross = false;    /*!< Cross (X) button. */
  bool circle = false;   /*!< Circle (O) button. */
  bool square = false;   /*!< Square button. */
  bool triangle = false; /*!< Triangle button. */
  bool l1 = false;       /*!< Left shoulder button. */
  bool r1 = false;       /*!< Right shoulder button. */

  bool dpadUp = false;    /*!< D-pad up. */
  bool dpadDown = false;  /*!< D-pad down. */
  bool dpadLeft = false;  /*!< D-pad left. */
  bool dpadRight = false; /*!< D-pad right. */
};

/*!
 * \class IInputSource
 * \brief Abstract operator-input provider.
 *
 * Call \ref begin once at startup, \ref update once per loop iteration,
 * then read \ref isConnected and \ref state. Methods with a default body
 * are no-ops so trivial fakes only need to implement the accessors.
 */
class IInputSource {
public:
  virtual ~IInputSource() = default;

  /*! \brief Initialize the backend and start pairing/scanning. */
  virtual void begin() {}

  /*! \brief Refresh the cached input state from the backend. */
  virtual void update() {}

  /*! \brief Whether an input device is connected as of the last \ref update. */
  virtual bool isConnected() const = 0;

  /*! \brief Most recent input snapshot (zeroed while disconnected). */
  virtual const InputState& state() const = 0;

  /*!
   * \brief Operator feedback color (lightbar/LED), [0..255] per channel.
   * \note Default no-op: sources without a light simply ignore it.
   */
  virtual void setLightbar(uint8_t r, uint8_t g, uint8_t b) {
    (void)r; (void)g; (void)b;
  }
};
