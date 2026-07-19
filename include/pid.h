#pragma once

/*!
 * \file Pid.h
 * \brief Minimal PID controller for a fixed-rate control loop.
 *
 * Discrete PID with integral anti-windup (clamped) and output saturation.
 * Designed to be called at a fixed period; \c dt is passed in per update so
 * the caller owns the timing.
 */

/*!
 * \class Pid
 * \brief Single-input PID with clamped integrator and saturated output.
 */
class Pid {
public:
  /*!
   * \brief Construct a PID controller.
   * \param kp       Proportional gain.
   * \param ki       Integral gain.
   * \param kd       Derivative gain.
   * \param outLimit Symmetric output clamp: result stays in [-outLimit, +outLimit].
   */
  Pid(float kp, float ki, float kd, float outLimit);

  /*!
   * \brief Compute the control output for one step.
   * \param setpoint Desired process value.
   * \param measured Current measured process value.
   * \param dt       Elapsed time since the previous call, in seconds.
   * \return Saturated control output in [-outLimit, +outLimit].
   */
  float compute(float setpoint, float measured, float dt);

  /*! \brief Reset integrator and derivative history (e.g. when re-enabling). */
  void reset();

  /*! \brief Update gains at runtime (for tuning). */
  void setGains(float kp, float ki, float kd);

private:
  float _kp, _ki, _kd;
  float _outLimit;
  float _integral;   /*!< Accumulated integral term, anti-windup clamped. */
  float _prevError;  /*!< Previous error, for the derivative term. */
};