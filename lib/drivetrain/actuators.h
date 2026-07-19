#pragma once
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "wheel.h"

/*!
 * \file actuators.h
 * \brief Drivetrain actuation module (\ref Actuators).
 *
 * Owns the whole actuation chain of the differential drive: two Motor +
 * Encoder + Pid + Wheel sets, pin mapping and tuned gains included. The
 * rest of the system commands wheel speeds in ticks/second through this
 * class only.
 *
 * \par TEST_MODE
 * Build with \c -D \c TEST_MODE (see platformio.ini) to disable all real
 * actuation: \ref Motor never touches GPIO/PWM, while PID, encoders and
 * logging keep running. Useful on the bench or on a bare devkit.
 */

/*!
 * \class Actuators
 * \brief Differential drivetrain: left/right wheel speed control with
 *        clamping to the physical speed limit.
 */
class Actuators {
public:
  /*!
   * \brief Velocita' massima in tick/sec - MISURALA col comando "max" della
   *        modalita' calibrazione e aggiorna qui.
   */
  static constexpr float kMaxTicksPerSec = 4100.0f;

  Actuators();

  /*! \brief Initialize motors and encoders. Call once from setup(). */
  void begin();

  /*!
   * \brief Command the wheel speed setpoints.
   * \param ticksL Left wheel target, ticks/second (signed).
   * \param ticksR Right wheel target, ticks/second (signed).
   * \note Both targets are clamped to +/- \ref kMaxTicksPerSec.
   */
  void setWheelTargets(float ticksL, float ticksR);

  /*!
   * \brief Run one closed-loop control step on both wheels.
   * \param dt Elapsed time since the last update, in seconds.
   */
  void update(float dt);

  /*! \brief Stop both motors and reset the PIDs. */
  void stop();

  /*! \brief Physical speed limit, ticks/second. */
  float maxTicksPerSec() const { return kMaxTicksPerSec; }

  /*! \brief Last measured left wheel speed, ticks/second. */
  float measuredLeft() const { return _wheelL.measuredSpeed(); }
  /*! \brief Last measured right wheel speed, ticks/second. */
  float measuredRight() const { return _wheelR.measuredSpeed(); }

  // --- Accesso a basso livello per la modalita' calibrazione ---

  /*!
   * \brief Drive both motors open-loop, bypassing the PIDs.
   * \param speed Normalized [-1.0, 1.0]. Used by the "max" tuning command.
   */
  void setRawSpeed(float speed);

  /*! \brief Update the PID gains of both wheels (runtime tuning). */
  void setPidGains(float kp, float ki, float kd);

  /*! \brief Reset both PID integrators/derivative history. */
  void resetPids();

  /*! \brief Cumulative left encoder ticks (signed). */
  long ticksLeft() const { return _encL.ticks(); }
  /*! \brief Cumulative right encoder ticks (signed). */
  long ticksRight() const { return _encR.ticks(); }

private:
  Motor   _motorL;
  Motor   _motorR;
  Encoder _encL;
  Encoder _encR;
  Pid     _pidL;
  Pid     _pidR;
  Wheel   _wheelL;
  Wheel   _wheelR;
};
