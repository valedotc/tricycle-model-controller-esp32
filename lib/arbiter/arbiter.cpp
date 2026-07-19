#include "arbiter.h"
#include <Arduino.h>

// ----------------------------------------------------------------------------
//  Feel / guidabilita' (solo modo MANUAL). Tutti ritoccabili a piacere.
// ----------------------------------------------------------------------------
// Rampa acceleratore: limita quanto in fretta il comando di gas puo' cambiare.
// La decel e' piu' dolce dell'accel, cosi' rilasciando R2 il robot rallenta
// gradualmente invece di inchiodare (e impennarsi in avanti).
//   unita' = frazione di gas al secondo.  1.5 => ~0.7 s per fermarsi da pieno.
static const float THROTTLE_ACCEL_RATE = 4.0f;  // salita (piu' alto = piu' pronto)
static const float THROTTLE_DECEL_RATE = 1.5f;  // verso lo zero (piu' basso = piu' dolce)

// --- Geometria (differential-drive; la ruota folle posteriore e' passiva) ---
//   Encoder x4 = 44 conteggi/giro-motore; riduttore 46:1 => 2024 tick/giro-ruota.
//   tick/metro = 2024 / (2*pi*raggio).  Raggio 0.108 m => 2983 tick/m.
static const float WHEEL_RADIUS_M   = 0.108f;   // raggio ruota
static const float TRACK_WIDTH_M    = 0.169f;   // carreggiata (centro-centro ruote)
static const float TICKS_PER_METER  = 2024.0f / (2.0f * PI * WHEEL_RADIUS_M);

// Sterzo (cinematica yaw-rate):
//   EXPO         ammorbidisce il centro della levetta (correzioni fini).
//                0 = lineare, 1 = tutto cubico.
//   MAX_YAW_RATE velocita' di rotazione a levetta piena, in rad/s.
//                A yaw-rate costante il raggio di curva cresce da solo con la
//                velocita' => niente sterzate nervose in velocita'.
//   STANDSTILL   quota di autorita' di sterzo disponibile da fermo [0..1]:
//                1 = yaw-rate pieno, gira su se stesso (spin-in-place);
//                0 = car-like, sterza solo in movimento.
static const float STEER_EXPO       = 0.35f;
static const float STEER_MAX_YAW    = 6.0f;     // rad/s (~340 deg/s)
static const float STEER_STANDSTILL = 1.0f;

// Differenziale per ruota (tick/s) a levetta piena, da yaw-rate e geometria:
//   omega = (vR - vL) / L ;  vwheel[m/s] = tick/s / k ;  => turn = omega*L*k/2.
static const float MAX_TURN_TICKS =
    STEER_MAX_YAW * TRACK_WIDTH_M * TICKS_PER_METER * 0.5f;

// Curva expo: mantiene il segno, lascia inalterati 0 e +/-1, addolcisce il centro.
static inline float applyExpo(float x, float expo) {
  return (1.0f - expo) * x + expo * x * x * x;
}

Arbiter::Arbiter(IInputSource& input, IBatterySensor& battery, Actuators& actuators)
  : _input(input), _battery(battery), _actuators(actuators) {}

void Arbiter::update(float dt) {
  // --- Selezione del modo. Con l'AUTO futuro: qui si decidera' anche la
  //     transizione MANUAL <-> AUTO (es. da un tasto del controller). ---
  if (!_input.isConnected()) {
    _mode = DriveMode::ESTOP;
    runEstop();
    return;
  }

  const InputState& s = _input.state();
  float throttleRaw = s.r2 - s.l2;   // [-1..1]
  float steering    = s.lx;          // [-1..1]

  // Batteria: campiona solo a gas quasi nullo (a motori fermi); a gas dato la
  // lettura viene saltata. A ogni nuova lettura aggiorno la lightbar.
  if (_battery.update(fabsf(throttleRaw) < 0.05f)) applyBatteryLightbar();

  if (_battery.lockout()) {
    _mode = DriveMode::ESTOP;
    runEstop();
    return;
  }

  _mode = DriveMode::MANUAL;
  runManual(throttleRaw, steering, dt);
}

void Arbiter::runEstop() {
  _actuators.stop();
  _throttleCmd = 0.0f;   // niente rampa "sporca" alla ripartenza
}

void Arbiter::runManual(float throttleRaw, float steering, float dt) {
  // Rampa acceleratore: avvicina _throttleCmd al comando grezzo, ma di poco per
  // ciclo. Decel piu' lenta dell'accel => rilascio = rallentamento graduale.
  bool  decelerating = fabsf(throttleRaw) < fabsf(_throttleCmd);
  float rate    = decelerating ? THROTTLE_DECEL_RATE : THROTTLE_ACCEL_RATE;
  float maxStep = rate * dt;
  _throttleCmd += constrain(throttleRaw - _throttleCmd, -maxStep, maxStep);

  // Sterzo cinematico: yaw-rate. expo sul centro, autorita' modulata da STANDSTILL.
  //   speedGate=1 (STANDSTILL=1) => yaw-rate costante (raggio cresce con la velocita').
  //   con STANDSTILL<1 lo sterzo da fermo e' ridotto (piu' car-like).
  float steerCmd  = applyExpo(steering, STEER_EXPO);
  float speedGate = STEER_STANDSTILL + (1.0f - STEER_STANDSTILL) * fabsf(_throttleCmd);
  float turnTicks = steerCmd * MAX_TURN_TICKS * speedGate;

  // Priorita' alla sterzata: se il differenziale non entra nel tetto, riduci la
  // velocita' avanti (non lo sterzo) => il raggio comandato vale a ogni gas.
  float maxTicks  = _actuators.maxTicksPerSec();
  float headroom  = fmaxf(0.0f, maxTicks - fabsf(turnTicks));
  float baseTicks = constrain(_throttleCmd * maxTicks, -headroom, headroom);

  _actuators.setWheelTargets(baseTicks + turnTicks, baseTicks - turnTicks);
  _actuators.update(dt);
}

void Arbiter::applyBatteryLightbar() {
  int pct = _battery.percent();
  if (_battery.lockout() || pct < 20) _input.setLightbar(255, 0,   0);   // rosso
  else if (pct < 40)                  _input.setLightbar(255, 120, 0);   // ambra
  else                                _input.setLightbar(0,   255, 0);   // verde
}
