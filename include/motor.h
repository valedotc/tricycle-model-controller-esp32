#pragma once
#include <cstdint>

/*!
 * \file Motor.h
 * \brief HAL for a single DC motor driven by an L298N (HW-095) half-bridge.
 *
 * PWM is applied on the IN pins (not the enable), because a 3.3V PWM on the
 * L298N enable is not reliably recognized, whereas the IN pins respond to
 * 3.3V logic. The enable line is expected to be tied HIGH externally
 * (ENA/ENB jumpers fitted).
 */

/*!
 * \class Motor
 * \brief Signed-speed control of one DC motor channel.
 */
class Motor {
public:
  /*!
   * \brief Construct a motor bound to one L298N channel.
   * \param in1        First direction pin (INx).
   * \param in2        Second direction pin (INx+1).
   * \param pwmChannel LEDC channel to use (0..15).
   * \param invert     Swap positive/negative for a mirrored wheel.
   */
  Motor(uint8_t in1, uint8_t in2, uint8_t pwmChannel, bool invert = false);

  /*! \brief Configure GPIOs and PWM channel. Call once from setup(). */
  void begin();

  /*!
   * \brief Drive at a signed normalized speed.
   * \param speed [-1.0, 1.0]: sign = direction, magnitude = duty. Clamped.
   */
  void setSpeed(float speed);

  /*! \brief Coast: both IN pins low. */
  void stop();

private:
  uint8_t _in1;
  uint8_t _in2;
  uint8_t _pwmChannel;
  bool    _invert;

  static constexpr uint32_t kPwmFreqHz  = 20000; /*!< 20 kHz, above audible. */
  static constexpr uint8_t  kPwmResBits = 8;     /*!< 8-bit duty: 0..255. */
};