#pragma once

/*!
 * \file battery_sensor.h
 * \brief Abstract battery sensor interface (\ref IBatterySensor).
 *
 * Decouples the drive logic from the physical measurement chain (ADC,
 * divider, hysteresis): the arbiter only reads percent/lockout, so a fake
 * implementation can inject arbitrary battery states in tests.
 */

/*!
 * \class IBatterySensor
 * \brief Abstract motor-battery monitor.
 *
 * \ref update owns the sampling policy (rate limiting, load gating); the
 * accessors return the values from the most recent sample.
 */
class IBatterySensor {
public:
  virtual ~IBatterySensor() = default;

  /*! \brief Configure the measurement hardware. Call once from setup(). */
  virtual void begin() {}

  /*!
   * \brief Give the sensor a chance to take a new sample.
   * \param motorsIdle true when the motors are (almost) unloaded, so the
   *        reading is not skewed by voltage sag under load.
   * \return true if a new sample was taken this call, false otherwise.
   */
  virtual bool update(bool motorsIdle) { (void)motorsIdle; return false; }

  /*! \brief Pack voltage from the last sample, in volts (0 before the first). */
  virtual float voltage() const = 0;

  /*! \brief Charge estimate from the last sample, [0..100]. */
  virtual int percent() const = 0;

  /*! \brief Undervoltage lockout state (with hysteresis). true = block motors. */
  virtual bool lockout() const = 0;
};
