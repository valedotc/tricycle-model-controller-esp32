#pragma once
#include <cstdint>
#include "input_source.h"
#include "battery_sensor.h"
#include "actuators.h"

/*!
 * \file arbiter.h
 * \brief Drive-mode arbiter (\ref Arbiter): decides who commands the wheels.
 *
 * Today only two modes exist: MANUAL (teleop from the input source) and
 * ESTOP (motors stopped: input disconnected or battery lockout). The
 * structure is ready for an AUTO mode later: add the enum value, a
 * runAuto() step and its entry in the \ref update dispatch.
 */

/*! \brief Operating mode selected by the arbiter. */
enum class DriveMode : uint8_t {
  MANUAL, /*!< Teleop: input source commands the drivetrain. */
  ESTOP,  /*!< Motors stopped: no input source or battery lockout. */
  // AUTO, /*!< Predisposto: guida autonoma, da aggiungere in seguito. */
};

/*!
 * \class Arbiter
 * \brief Owns mode selection and the MANUAL drive feel/kinematics.
 *
 * Call \ref update at the fixed control rate, after the input source has
 * been refreshed. Depends only on the \ref IInputSource and
 * \ref IBatterySensor interfaces, so it can be tested with fakes.
 */
class Arbiter {
public:
  Arbiter(IInputSource& input, IBatterySensor& battery, Actuators& actuators);

  /*!
   * \brief Run one arbitration + control step.
   * \param dt Elapsed time since the last update, in seconds.
   */
  void update(float dt);

  /*! \brief Mode selected by the last \ref update. */
  DriveMode mode() const { return _mode; }

private:
  /*! \brief Enter/hold ESTOP: stop the drivetrain and reset the ramp. */
  void runEstop();

  /*! \brief One MANUAL teleop step: ramp, steering kinematics, mixing. */
  void runManual(float throttleRaw, float steering, float dt);

  /*! \brief Color the input lightbar from the battery state (rosso/ambra/verde). */
  void applyBatteryLightbar();

  IInputSource&   _input;
  IBatterySensor& _battery;
  Actuators&      _actuators;

  DriveMode _mode = DriveMode::ESTOP;

  /*! Comando di gas "rampato" - stato persistente tra i cicli. */
  float _throttleCmd = 0.0f;
};
