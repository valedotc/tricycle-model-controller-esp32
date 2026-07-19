#pragma once
#include <cstdint>

/*!
 * \file Encoder.h
 * \brief Quadrature encoder reader for one motor channel.
 *
 * Full x4 quadrature decode: an interrupt on BOTH edges of BOTH channels
 * feeds a state-transition table, so every valid A/B transition is counted.
 * This is 4x the resolution of A-rising-only counting, which makes the
 * per-window speed measurement far less coarse (less quantization noise,
 * so the speed PID chatters/vibrates much less). The tick count is signed
 * and cumulative.
 */

/*!
 * \class Encoder
 * \brief Interrupt-driven quadrature tick counter.
 *
 * Construct with the A and B GPIOs, call \ref begin once, then read the
 * accumulated position with \ref ticks or clear it with \ref reset.
 *
 * \note Channels A and B must be on interrupt-capable pins. On the ESP32
 *       every GPIO qualifies, including the input-only 34/35/36/39.
 */
class Encoder {
public:
  /*!
   * \brief Construct an encoder bound to two quadrature pins.
   * \param pinA Channel A GPIO (both edges trigger the interrupt).
   * \param pinB Channel B GPIO (both edges trigger the interrupt).
   * \param invert Flip the counting direction for a mirrored motor.
   */
  Encoder(uint8_t pinA, uint8_t pinB, bool invert = false);

  /*! \brief Configure pins and attach the interrupt. Call once from setup(). */
  void begin();

  /*!
   * \brief Read the accumulated signed tick count.
   * \return Cumulative ticks since the last \ref reset (may be negative).
   */
  long ticks() const;

  /*! \brief Zero the tick counter. */
  void reset();

private:
  uint8_t _pinA;
  uint8_t _pinB;
  bool    _invert;

  volatile long    _ticks;     /*!< Updated inside the ISR; read via \ref ticks. */
  volatile uint8_t _state;     /*!< Last (A<<1|B) quadrature state, for decoding. */

  void handleEdge();                 /*!< Per-instance ISR body (shared A/B). */
  static void isrTrampoline(void* arg); /*!< Dispatches to handleEdge(). */
};