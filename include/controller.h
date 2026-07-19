#pragma once
#include <cstdint>

/*!
 * \file Controller.h
 * \brief Hardware-agnostic gamepad wrapper.
 *
 * Encapsulates the underlying controller library so the rest of the
 * system never references it directly. Swapping the backing library or
 * the physical controller only requires changes inside this class.
 */

/*!
 * \class Controller
 * \brief High-level view of a game controller.
 *
 * Exposes normalized axes and debounced button states through a stable
 * interface. Call \ref begin once at startup and \ref update once per
 * loop iteration before reading \ref state.
 */
class Controller {
public:
  /*!
   * \brief Snapshot of the controller inputs at a single point in time.
   *
   * Analog sticks are normalized to [-1.0, 1.0] with the deadzone already
   * applied; triggers are normalized to [0.0, 1.0]. All fields are zeroed
   * while the controller is disconnected.
   */
  struct State {
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
   * \brief Initialize the controller backend and start pairing/scanning.
   *
   * Must be called once from setup() before any call to \ref update.
   */
  void begin();

  /*!
   * \brief Refresh the cached input state from the backend.
   *
   * Call once per loop iteration. When the controller is disconnected the
   * cached \ref State is reset to zero so stale inputs are never returned.
   */
  void update();

  /*!
   * \brief Report whether a controller is currently connected.
   * \return true if connected as of the last \ref update, false otherwise.
   */
  bool isConnected() const;

  /*!
   * \brief Access the most recent input snapshot.
   * \return Const reference to the state cached by the last \ref update.
   */
  const State& state() const { return _state; }

private:
  State _state;             /*!< Cached inputs from the last update. */
  bool  _connected = false; /*!< Connection status as of the last update. */
  float _deadzone = 0.10f;  /*!< Stick deadzone as a fraction of full scale. */

  /*!
   * \brief Apply the deadzone to a normalized axis and rescale the remainder.
   * \param v Raw normalized axis value in [-1.0, 1.0].
   * \return 0 inside the deadzone, otherwise the value rescaled so the
   *         deadzone edge maps to 0 and full deflection still maps to +/-1.0.
   */
  float applyDeadzone(float v) const;
};