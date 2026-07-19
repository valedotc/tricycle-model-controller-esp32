#pragma once
#include "input_source.h"

/*!
 * \file input_dualsense.h
 * \brief DualSense (PS5) implementation of \ref IInputSource.
 *
 * Encapsulates the underlying esp-ps5 library so the rest of the system
 * never references it directly. Swapping the backing library or the
 * physical controller only requires changes inside this class.
 */

/*!
 * \class DualSenseInput
 * \brief DualSense controller over Bluetooth, exposed as \ref IInputSource.
 *
 * Exposes normalized axes and button states through the stable
 * \ref IInputSource interface. Call \ref begin once at startup and
 * \ref update once per loop iteration before reading \ref state.
 */
class DualSenseInput : public IInputSource {
public:
  /*!
   * \brief Initialize the controller backend and start pairing/scanning.
   *
   * Must be called once from setup() before any call to \ref update.
   */
  void begin() override;

  /*!
   * \brief Refresh the cached input state from the backend.
   *
   * Call once per loop iteration. When the controller is disconnected the
   * cached \ref InputState is reset to zero so stale inputs are never
   * returned.
   */
  void update() override;

  /*!
   * \brief Report whether a controller is currently connected.
   * \return true if connected as of the last \ref update, false otherwise.
   */
  bool isConnected() const override;

  /*!
   * \brief Access the most recent input snapshot.
   * \return Const reference to the state cached by the last \ref update.
   */
  const InputState& state() const override { return _state; }

  /*!
   * \brief Set the controller lightbar color.
   * \param r Red   [0..255].
   * \param g Green [0..255].
   * \param b Blue  [0..255].
   * \note Only emits a frame when the color changes, to respect the
   *       library's ~10 ms minimum output interval and avoid link drops.
   */
  void setLightbar(uint8_t r, uint8_t g, uint8_t b) override;

private:
  InputState _state;        /*!< Cached inputs from the last update. */
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
