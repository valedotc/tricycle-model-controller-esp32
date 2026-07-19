#pragma once
#include <cstdint>
#include "battery_sensor.h"

/*!
 * \file battery.h
 * \brief Motor battery monitor (\ref BatteryMonitor), implements \ref IBatterySensor.
 *
 * Batteria motori 2S Li-ion (6.0-8.4V). Partitore R1=100k / R2=50k (due 100k
 * in parallelo) su GPIO39 (VN, ADC1). Rapporto pulito 1:3.
 * ADC1 obbligatorio: ADC2 non funziona con il Bluetooth attivo.
 * La lettura va fatta solo a motori quasi fermi (sotto carico la V crolla e
 * falserebbe); il chiamante lo segnala via il parametro \c motorsIdle di
 * \ref update. Isteresi sul cutoff per non rimbalzare on/off sulla soglia.
 */

/*!
 * \class BatteryMonitor
 * \brief ADC battery reader with 1 Hz sampling, percent estimate and
 *        undervoltage lockout (hysteresis).
 */
class BatteryMonitor : public IBatterySensor {
public:
  /*! \brief Configure ADC resolution and pin attenuation. Call from setup(). */
  void begin() override;

  /*!
   * \brief Sample the battery at most once per second, and only when idle.
   * \param motorsIdle true when the motors are (almost) unloaded.
   * \return true if a new sample was taken (voltage/percent/lockout updated).
   */
  bool update(bool motorsIdle) override;

  float voltage() const override { return _voltage; }
  int   percent() const override { return _percent; }
  bool  lockout() const override { return _lockout; }

private:
  static const int   kPin;         /*!< GPIO39 (VN, ADC1). */
  static const float kDivRatio;    /*!< (R1+R2)/R2. Calibra col multimetro. */
  static const float kVFull;       /*!< 8.4V = 100%. */
  static const float kVEmpty;      /*!< 6.0V = 0%. */
  static const float kVCutoff;     /*!< Sotto: blocco motori (margine sul 6.0). */
  static const float kVCutoffHyst; /*!< Risale sopra cutoff+hyst per sbloccare. */

  float    _voltage   = 0.0f;
  int      _percent   = 0;
  bool     _lockout   = false;
  uint32_t _lastCheck = 0;

  /*! \brief Averaged raw voltage read (16 samples through the divider). */
  float readVoltage() const;
};
