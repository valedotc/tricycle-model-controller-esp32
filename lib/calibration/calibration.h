#pragma once
#include <cstdint>
#include "actuators.h"

/*!
 * \file calibration.h
 * \brief Modalita' calibrazione PID via comandi seriali (\ref Calibration).
 *
 * ============================================================================
 *  CALIBRAZIONE PID - istruzioni rapide
 * ----------------------------------------------------------------------------
 *  1. Ruote IN ARIA per i primi test.
 *  2. Verifica i versi: con CALIB_MODE attivo il robot va a META' velocita'
 *     avanti. Le due misure (MIS L / MIS R) devono essere POSITIVE. Se una
 *     e' negativa, flippa l'invert di QUELL'encoder in actuators.cpp.
 *  3. Misura la velocita' massima: metti Kp=0,Ki=0,Kd=0 e usa il comando
 *     seriale "max" (vedi sotto) -> gira a setSpeed(1.0) e stampa i tick/s.
 *     Copia quel numero in Actuators::kMaxTicksPerSec.
 *  4. Tara: da seriale scrivi  "p 0.003"  "i 0.005"  "d 0"  per cambiare i
 *     guadagni al volo. Guarda MIS inseguire SET. Ordine: Kp -> Ki -> Kd.
 *  5. Quando sei contento, copia i valori nei costruttori Pid in
 *     actuators.cpp e metti CALIB_MODE a false per guidare col controller.
 *
 *  Comandi seriali (invio a fine riga):
 *     p <val>   imposta Kp (entrambe le ruote)
 *     i <val>   imposta Ki
 *     d <val>   imposta Kd
 *     s <val>   setpoint di test in FRAZIONE [-1..1] (es. s 0.5 = meta')
 *     max       gira a setSpeed(1.0) diretto (bypassa PID) per misurare i tick/s
 *     run       torna in modalita' PID col setpoint corrente
 *     stop      ferma tutto
 * ============================================================================
 */

/*!
 * \class Calibration
 * \brief Serial-driven PID tuning loop. Active only when main runs in
 *        CALIB_MODE; replaces the arbiter as the drivetrain commander.
 *
 * The serial parser uses a fixed char buffer: no Arduino String, no dynamic
 * allocation in the loop.
 */
class Calibration {
public:
  explicit Calibration(Actuators& actuators);

  /*!
   * \brief Run one calibration step: poll serial, drive the test setpoint.
   * \param dt Elapsed time since the last update, in seconds.
   */
  void update(float dt);

private:
  /*! \brief Accumulate serial chars (non-blocking); parse on newline. */
  void pollSerial();

  /*! \brief Execute one complete command line. */
  void handleLine(const char* line);

  Actuators& _actuators;

  float _setpointFrac = 0.0f;  /*!< Frazione [-1..1] del setpoint di test. */
  bool  _rawMaxMode   = false; /*!< true = setSpeed(1.0) diretto per misurare. */

  /* Guadagni correnti, ribaditi a ogni comando p/i/d. */
  float _kp = 0.0010f, _ki = 0.0025f, _kd = 0.0f;

  /* Buffer di linea seriale a dimensione fissa. */
  char    _line[32];
  uint8_t _len = 0;

  /* Misura tick/s in modalita' "max" (delta tick fra i cicli). */
  long _lastTicksL = 0, _lastTicksR = 0;
  bool _rawMaxInit = false;
};
