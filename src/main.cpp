#include <Arduino.h>
#include "controller.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "wheel.h"

// ============================================================================
//  CALIBRAZIONE PID - istruzioni rapide
// ----------------------------------------------------------------------------
//  1. Ruote IN ARIA per i primi test.
//  2. Verifica i versi: con TEST_MODE attivo il robot va a META' velocita'
//     avanti. Le due misure (MIS L / MIS R) devono essere POSITIVE. Se una
//     e' negativa, flippa l'invert di QUELL'encoder qui sotto.
//  3. Misura la velocita' massima: metti Kp=0,Ki=0,Kd=0 e usa il comando
//     seriale "max" (vedi sotto) -> gira a setSpeed(1.0) e stampa i tick/s.
//     Copia quel numero in MAX_TICKS_PER_SEC.
//  4. Tara: da seriale scrivi  "p 0.003"  "i 0.005"  "d 0"  per cambiare i
//     guadagni al volo. Guarda MIS inseguire SET. Ordine: Kp -> Ki -> Kd.
//  5. Quando sei contento, copia i valori nei costruttori Pid e metti
//     TEST_MODE a false per guidare col controller.
//
//  Comandi seriali (invio a fine riga):
//     p <val>   imposta Kp (entrambe le ruote)
//     i <val>   imposta Ki
//     d <val>   imposta Kd
//     s <val>   setpoint di test in FRAZIONE [-1..1] (es. s 0.5 = meta')
//     max       gira a setSpeed(1.0) diretto (bypassa PID) per misurare i tick/s
//     run       torna in modalita' PID col setpoint corrente
//     stop      ferma tutto
// ============================================================================

// Metti true per calibrare (setpoint fisso da seriale, niente controller).
// Metti false per guidare col DualSense.
static const bool TEST_MODE = false;

// --- Hardware ---
Controller controller;

Motor   motorL(25, 26, 0);
Motor   motorR(33, 32, 1, /*invert=*/true);
Encoder encL (34, 35);
// Encoder DX: fase B spostata da GPIO39 (VN) a D27 per liberare VN come
// ingresso ADC1 della batteria (vedi sezione Batteria piu' sotto).
Encoder encR (36, 27, /*invert=*/true);

// PID: output limitato a 1.0 (Motor::setSpeed vuole [-1, 1]).
// Tarati a vuoto (ruote in aria) il 2026-07-19 con encoder in quadratura x4:
// Kp=0.0010 Ki=0.0025 Kd=0. Kd tenuto a 0: il termine derivativo amplifica il
// rumore residuo (peggiora anche a x4). Kp basso di proposito: sopra ~0.0015 a
// bassa velocita' compare un limit-cycle stick-slip. Sotto carico potrebbe
// servire un filo piu' di Kp/Ki.
Pid pidL(0.0010f, 0.0025f, 0.0f, 1.0f);
Pid pidR(0.0010f, 0.0025f, 0.0f, 1.0f);

Wheel wheelL(motorL, encL, pidL);
Wheel wheelR(motorR, encR, pidR);

// Velocita' massima in tick/sec - MISURALA col comando "max" e aggiorna qui.
float MAX_TICKS_PER_SEC = 4100.0f;

// ----------------------------------------------------------------------------
//  Batteria motori (2S Li-ion, 6.0-8.4V). Partitore R1=100k / R2=50k (due 100k
//  in parallelo) su GPIO39 (VN, ADC1). Rapporto pulito 1:3.
//  ADC1 obbligatorio: ADC2 non funziona con il Bluetooth attivo.
//  Lettura fatta solo a motori quasi fermi (sotto carico la V crolla e
//  falserebbe). Isteresi sul cutoff per non rimbalzare on/off sulla soglia.
// ----------------------------------------------------------------------------
static const int   BATT_PIN      = 39;
static const float DIV_RATIO     = 3.134f;   // (R1+R2)/R2 = 150k/50k. Calibra col
                                           // multimetro se le tolleranze scostano.
static const float V_FULL        = 8.4f;
static const float V_EMPTY       = 6.0f;
static const float V_CUTOFF      = 6.2f;   // sotto: blocco motori (margine sul 6.0)
static const float V_CUTOFF_HYST = 0.4f;   // risale sopra cutoff+hyst per sbloccare

bool batteryLockout = false;

static float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(BATT_PIN); delayMicroseconds(200); }
  float vPin = (sum / 16.0f / 4095.0f) * 3.3f;
  return vPin * DIV_RATIO;
}

// Chiamata a ~1 Hz, ma solo quando i motori sono quasi fermi. Aggiorna il
// lockout con isteresi e colora la lightbar del DualSense come indicatore.
static void updateBattery(bool motorsIdle) {
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < 1000) return;
  lastCheck = millis();
  if (!motorsIdle) return;

  float v = readBatteryVoltage();
  if (v < V_CUTOFF)                        batteryLockout = true;
  else if (v > V_CUTOFF + V_CUTOFF_HYST)   batteryLockout = false;

  int pct = constrain((int)((v - V_EMPTY) / (V_FULL - V_EMPTY) * 100.0f), 0, 100);
  if (batteryLockout || pct < 20)   controller.setLightbar(255, 0,   0);   // rosso
  else if (pct < 40)                controller.setLightbar(255, 120, 0);   // ambra
  else                              controller.setLightbar(0,   255, 0);   // verde

  Serial.printf("[BATT] %.2fV  %d%%  %s\n", v, pct, batteryLockout ? "LOCKOUT" : "ok");
}

// ----------------------------------------------------------------------------
//  Feel / guidabilita' (solo modalita' GUIDA). Tutti ritoccabili a piacere.
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
//   MAX_YAW_RATE velocita' di rotazione a levetta piena, in rad/s (~200 deg/s).
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

// Comando di gas "rampato" - stato persistente tra i cicli di loop().
float throttleCmd = 0.0f;

// Curva expo: mantiene il segno, lascia inalterati 0 e +/-1, addolcisce il centro.
static inline float applyExpo(float x, float expo) {
  return (1.0f - expo) * x + expo * x * x * x;
}

// Loop di controllo a rate fisso.
static const uint32_t CONTROL_PERIOD_MS = 20;   // 50 Hz
uint32_t lastControl = 0;

// Stato calibrazione.
float testSetpointFrac = 0.0f;   // frazione [-1..1] del setpoint di test
bool  rawMaxMode = false;        // true = setSpeed(1.0) diretto per misurare

// ----------------------------------------------------------------------------
//  Parser comandi seriali per tuning al volo.
// ----------------------------------------------------------------------------
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  float val = line.substring(1).toFloat();

  static float kp = 0.0010f, ki = 0.0025f, kd = 0.0f;

  if (line == "max") {
    rawMaxMode = true;
    Serial.println("[MAX] setSpeed(1.0) diretto - leggi i tick/s a regime");
  } else if (line == "run") {
    rawMaxMode = false;
    Serial.println("[RUN] modalita' PID");
  } else if (line == "stop") {
    rawMaxMode = false;
    testSetpointFrac = 0.0f;
    wheelL.stop(); wheelR.stop();
    Serial.println("[STOP]");
  } else if (cmd == 'p') {
    kp = val; pidL.setGains(kp, ki, kd); pidR.setGains(kp, ki, kd);
    Serial.printf("[GAIN] Kp=%.5f Ki=%.5f Kd=%.5f\n", kp, ki, kd);
  } else if (cmd == 'i') {
    ki = val; pidL.setGains(kp, ki, kd); pidR.setGains(kp, ki, kd);
    pidL.reset(); pidR.reset();
    Serial.printf("[GAIN] Kp=%.5f Ki=%.5f Kd=%.5f\n", kp, ki, kd);
  } else if (cmd == 'd') {
    kd = val; pidL.setGains(kp, ki, kd); pidR.setGains(kp, ki, kd);
    Serial.printf("[GAIN] Kp=%.5f Ki=%.5f Kd=%.5f\n", kp, ki, kd);
  } else if (cmd == 's') {
    testSetpointFrac = constrain(val, -1.0f, 1.0f);
    rawMaxMode = false;
    Serial.printf("[SET] setpoint = %.2f (%.0f tick/s)\n",
                  testSetpointFrac, testSetpointFrac * MAX_TICKS_PER_SEC);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // ADC batteria: 12 bit, attenuazione piena per arrivare fino a ~3.3V sul pin.
  analogReadResolution(12);
  analogSetPinAttenuation(BATT_PIN, ADC_11db);

  wheelL.begin();
  wheelR.begin();

  if (TEST_MODE) {
    Serial.println("\n[BOOT] MODALITA' CALIBRAZIONE");
    Serial.println("comandi: p/i/d <val>, s <frazione>, max, run, stop");
  } else {
    Serial.println("\n[BOOT] MODALITA' GUIDA - pairing DualSense (PS + Share)");
    controller.begin();
  }
}

void loop() {
  if (!TEST_MODE) controller.update();

  uint32_t now = millis();
  if (now - lastControl < CONTROL_PERIOD_MS) return;
  float dt = (now - lastControl) / 1000.0f;
  lastControl = now;

  // ---------------- MODALITA' CALIBRAZIONE ----------------
  if (TEST_MODE) {
    handleSerial();

    if (rawMaxMode) {
      // Bypassa il PID: comando diretto a manetta per misurare tick/s.
      motorL.setSpeed(1.0f);
      motorR.setSpeed(1.0f);
      // measuredSpeed non e' aggiornato in questa modalita', calcolo al volo
      // usando Wheel::update con target 0 solo per leggere la misura.
      wheelL.setTargetSpeed(0); wheelR.setTargetSpeed(0);
      // Nota: qui i motori sono gia' comandati sopra; update li ricomanderebbe.
      // Per misura pulita leggo direttamente gli encoder:
      static long lt = encL.ticks(), rt = encR.ticks();
      float vl = (encL.ticks() - lt) / dt;
      float vr = (encR.ticks() - rt) / dt;
      lt = encL.ticks(); rt = encR.ticks();
      Serial.printf("[MAX] tick/s  L=%.0f  R=%.0f\n", vl, vr);
      return;
    }

    float target = testSetpointFrac * MAX_TICKS_PER_SEC;
    wheelL.setTargetSpeed(target);
    wheelR.setTargetSpeed(target);
    wheelL.update(dt);
    wheelR.update(dt);

    Serial.printf("SET=%.0f  MIS L=%.0f R=%.0f\n",
                  target, wheelL.measuredSpeed(), wheelR.measuredSpeed());
    return;
  }

  // ---------------- MODALITA' GUIDA ----------------
  if (!controller.isConnected()) {
    wheelL.stop();
    wheelR.stop();
    throttleCmd = 0.0f;   // niente rampa "sporca" alla riconnessione
    return;
  }

  const auto& s = controller.state();
  float throttleRaw = s.r2 - s.l2;   // [-1..1]
  float steering    = s.lx;          // [-1..1]

  // Batteria: leggo solo a gas quasi nullo (a motori fermi), coloro la
  // lightbar e gestisco il cutoff. A gas dato la lettura viene saltata.
  updateBattery(fabsf(throttleRaw) < 0.05f);
  if (batteryLockout) {
    wheelL.stop();
    wheelR.stop();
    throttleCmd = 0.0f;
    return;
  }

  // Rampa acceleratore: avvicina throttleCmd al comando grezzo, ma di poco per
  // ciclo. Decel piu' lenta dell'accel => rilascio = rallentamento graduale.
  bool  decelerating = fabsf(throttleRaw) < fabsf(throttleCmd);
  float rate    = decelerating ? THROTTLE_DECEL_RATE : THROTTLE_ACCEL_RATE;
  float maxStep = rate * dt;
  throttleCmd += constrain(throttleRaw - throttleCmd, -maxStep, maxStep);

  // Sterzo cinematico: yaw-rate. expo sul centro, autorita' modulata da STANDSTILL.
  //   speedGate=1 (STANDSTILL=1) => yaw-rate costante (raggio cresce con la velocita').
  //   con STANDSTILL<1 lo sterzo da fermo e' ridotto (piu' car-like).
  float steerCmd  = applyExpo(steering, STEER_EXPO);
  float speedGate = STEER_STANDSTILL + (1.0f - STEER_STANDSTILL) * fabsf(throttleCmd);
  float turnTicks = steerCmd * MAX_TURN_TICKS * speedGate;

  // Priorita' alla sterzata: se il differenziale non entra nel tetto, riduci la
  // velocita' avanti (non lo sterzo) => il raggio comandato vale a ogni gas.
  float headroom  = fmaxf(0.0f, MAX_TICKS_PER_SEC - fabsf(turnTicks));
  float baseTicks = constrain(throttleCmd * MAX_TICKS_PER_SEC, -headroom, headroom);

  float tL = baseTicks + turnTicks;
  float tR = baseTicks - turnTicks;

  wheelL.setTargetSpeed(tL);
  wheelR.setTargetSpeed(tR);
  wheelL.update(dt);
  wheelR.update(dt);
}