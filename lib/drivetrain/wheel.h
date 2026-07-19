#pragma once
#include "motor.h"
#include "encoder.h"
#include "pid.h"

/*!
 * \file Wheel.h
 * \brief Closed-loop speed control for one driven wheel.
 *
 * Combines a \ref Motor actuator, an \ref Encoder sensor and a \ref Pid
 * regulator so the wheel tracks a commanded speed (in ticks/second)
 * regardless of load, battery sag or motor asymmetry.
 */

/*!
 * \class Wheel
 * \brief One wheel: Motor + Encoder + PID, tracking a speed setpoint.
 */
class Wheel {
public:
  /*!
   * \brief Construct a wheel from its parts.
   * \param motor Reference to the motor driving this wheel.
   * \param enc   Reference to the encoder on this wheel.
   * \param pid   Reference to the PID regulating this wheel.
   */
  Wheel(Motor& motor, Encoder& enc, Pid& pid);

  /*! \brief Initialize the underlying motor and encoder. */
  void begin();

  /*!
   * \brief Set the desired wheel speed.
   * \param ticksPerSec Target speed in encoder ticks per second (signed).
   */
  void setTargetSpeed(float ticksPerSec);

  /*!
   * \brief Run one control step. Call at a fixed rate.
   * \param dt Elapsed time since the last update, in seconds.
   */
  void update(float dt);

  /*! \brief Stop the motor and reset the PID. */
  void stop();

  /*! \brief Last measured speed in ticks/second (for logging/tuning). */
  float measuredSpeed() const { return _measuredSpeed; }

private:
  Motor&   _motor;
  Encoder& _enc;
  Pid&     _pid;

  float _target;         /*!< Setpoint, ticks/second. */
  long  _lastTicks;      /*!< Tick count at the previous update. */
  float _measuredSpeed;  /*!< Last measured speed, ticks/second. */
};