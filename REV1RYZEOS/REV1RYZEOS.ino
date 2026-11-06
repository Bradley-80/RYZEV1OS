/*
 * REV1RYZEOS.ino  (RYZE OS V1)
 * Ground controller firmware for the RYZE drive-and-fly prototype.
 * Built from rc_control_test.ino with every change through Oct 26, 2026.
 * The Boxer drives all ground actuators through the passive CRSF tap.
 *
 *   SC switch (CH6)   -> both steppers extend or retract the arm racks
 *   Throttle (CH3)    -> both drive motors, DRIVE mode only
 *   Steering (CH1)    -> steering servo, DRIVE mode only, proportional
 *   FLIGHT mode       -> both drive motors brake constantly, steering holds centre
 *
 * Hardware: ELEGOO ESP32 DevKit (ESP-WROOM-32, 30-pin), RP3 receiver
 *           (CRSF tap on D35), 2x 28BYJ-48 on DRV8833 #1/#2, one SG90 steering
 *           servo on D18, 2x TT motor on DRV8833 #3 (motor 1 channel A on
 *           D21/D19, motor 2 channel B on RX2/D4).
 * Power:    UBEC-8A set to 5.2 V feeds the 5 V rail. The ESP32 runs from the
 *           rail through VIN (W49). USB is for uploads and the Serial Monitor.
 * Wiring:   RYZE-ELEC-01 REV G plus W49 (ESP32 VIN). No libraries needed.
 * Arduino:  board "ESP32 Dev Module". Works on ESP32 Arduino core 2.x and 3.x.
 *
 * Serial monitor: 115200 baud, line ending "Newline". Type h for help.
 *
 * RC SOURCE
 *   RC_FROM_SERIAL = false (default): live radio through the CRSF tap on D35.
 *   RC_FROM_SERIAL = true: typed commands stand in for the radio for bench work.
 *     drive / flight / mid   SC switch up / down / middle (CH6 2000 / 1000 / 1500)
 *     t 40                   throttle 40 %  (0-100)
 *     s -60                  steering 60 % left (-100 full left, 100 full right)
 *     c                      steering centre
 *     x                      throttle to zero
 *     link off / link on     simulate radio link loss
 *
 * TRANSFORMATION MATH
 *   Pinion 25 teeth (module about 1.375 mm), rack travel 35 teeth (about 151 mm).
 *   Output turns = 35 / 25 = 1.4 turns.
 *   28BYJ-48 true gear ratio is 63.68:1, so 4076 half-steps per output turn.
 *   Travel = 1.4 x 4076 = 5706 half-steps per motor.
 *   Full-step drive (two coils on): 2853 full steps per stroke.
 *   Cruise 2200 us per full step (about 13 RPM) -> stroke about 6.5 s.
 *   Each move ramps up from 4000 us and slows again near the end, so the
 *   motor locks on without skipping. Tune live with "spd N".
 *
 * SMOOTH DRIVE START
 *   Slow-decay PWM at 20 kHz. No full-power kick (it caused a start jerk).
 *   Throttle ramps up at MOTOR_RAMP_PER_S and starts right at the motors'
 *   breakaway duty (motor min duty). Find that duty once: arms retracted,
 *   wheels off the bench, type "find", press Enter the moment both wheels
 *   turn. Copy the printed number into MOTOR_MIN_DUTY_DEFAULT.
 *
 * MODE LOGIC
 *   CH6 below 1300 us = FLIGHT (arms out). CH6 above 1700 us = DRIVE (arms in).
 *   SC middle position sits between the two and changes nothing.
 *   60 ms debounce. Each switch flip moves the arms once, then holds.
 *   Flip back mid-move and the arms reverse from where they are.
 *   Drive and steering only work with both arms fully retracted.
 *   Throttle gate: after entering DRIVE, the motor stays off until the
 *   throttle stick goes to the bottom once. Stops a lurch after a transform.
 *   Link loss (no frame for 250 ms): motors brake, steering centres,
 *   steppers pause and resume when the link returns.
 *   Betaflight PREARM on AUX2 (CH6) keeps the flight controller disarmed in DRIVE.
 *
 * STARTUP RULES
 *   1. Power up with both arms fully retracted. The steppers have no sensor,
 *      so the code assumes position 0 = retracted at boot.
 *   2. If SC is in FLIGHT at boot, nothing moves. Flip SC to DRIVE, then
 *      back to FLIGHT to extend.
 *   3. If an upload fails, unplug W14 (D12), upload, reconnect.
 *   4. D14 pulses at boot. One twitch on stepper 2 is normal.
 *   5. Wait for the stroke to finish (about 6.5 s) before arming on SA.
 *
 * LOCKED stepper pins (do not change):
 *   M1: D32 AIN2, D33 AIN1, D25 BIN1, D26 BIN2
 *   M2: D27 AIN2, D14 AIN1, D12 BIN1, D13 BIN2
 */

#include <Arduino.h>
#include <WiFi.h>

// ===================== Pins (REV G) =====================
//                          AIN2 AIN1 BIN1 BIN2
static const int M1_PINS[4] = { 32, 33, 25, 26 };
static const int M2_PINS[4] = { 27, 14, 12, 13 };

static const int PIN_CRSF_RX = 35;   // W5, CRSF tap, input only
static const int PIN_STEER_L = 18;   // W18
static const int PIN_STEER_R = -1;   // single steering servo fitted. D22 stays free
static const int PIN_DRV_IN1 = 21;   // W16, DRV8833 #3 AIN1, drive motor 1
static const int PIN_DRV_IN2 = 19;   // W17, DRV8833 #3 AIN2, drive motor 1
static const int PIN_DRV_IN3 = 16;   // W45, DRV8833 #3 BIN1, drive motor 2 (RX2 / GPIO16)
static const int PIN_DRV_IN4 = 4;    // W46, DRV8833 #3 BIN2, drive motor 2 (D4 / GPIO4)

// ===================== RC channels (1-based) =====================
// EdgeTX default AETR order: CH1 aileron, CH2 elevator, CH3 throttle, CH4 rudder.
static const int CH_STEER    = 1;    // right stick left/right. Use 4 for left stick left/right.
static const int CH_THROTTLE = 3;    // left stick up/down
static const int CH_MODE     = 6;    // SC switch (AUX2)
static const int CH_REVERSE  = 0;    // 0 = forward only. Set to a switch channel (e.g. 7) for reverse.

// RC source.
// true  = sticks and SC switch come from typed Serial Monitor commands (bench work, no radio).
// false = live radio through the CRSF tap on D35.
static const bool RC_FROM_SERIAL = false;

static const int MODE_FLIGHT_BELOW_US = 1300;
static const int MODE_DRIVE_ABOVE_US  = 1700;
static const unsigned long MODE_DEBOUNCE_MS = 60;
static const unsigned long LINK_TIMEOUT_MS  = 250;

// ===================== Transformation =====================
static const int  PINION_TEETH  = 25;
static const int  RACK_TEETH    = 35;     // travel per side
static const long STEPS_PER_REV = 4076;   // half-steps per output turn (63.68:1)
static const long TRAVEL_STEPS  = (RACK_TEETH * STEPS_PER_REV + PINION_TEETH / 2) / PINION_TEETH;  // 5706

// Extend direction per motor. If a rack moves the wrong way, flip that sign.
// Mirrored motors usually need opposite signs.
static const int M1_EXTEND_DIR = +1;
static const int M2_EXTEND_DIR = +1;

// Full-step drive (two coils on) gives more torque than half-step and twice the
// distance per step. Set false to go back to half-step.
static const bool STEP_FULL = true;
// Cruise speed in microseconds per step (full steps when STEP_FULL).
// 2200 us = about 13 RPM, stroke about 6.5 s. Lower = faster.
// A 28BYJ-48 on 5 V starts to skip below about 1800 us under load. Skipped
// steps lose arm position (no sensor), so back off if you hear clicking.
static unsigned long stepIntervalUs = 2200;
static const unsigned long STEP_START_US = 4000;  // first step speed, lets the rotor lock on
static const unsigned long STEP_RAMP_US  = 20;    // faster by this much per step (4000 -> 2200 in 90 steps)

// ===================== Steering =====================
static const bool STEER_INVERT       = false;  // set true if stick left turns the wheels right
static const int  SERVO_MIN_US       = 1000;   // full left
static const int  SERVO_CENTER_US    = 1500;
static const int  SERVO_MAX_US       = 2000;   // full right
static const int  STEER_DEADBAND_US  = 15;     // stick noise around centre
static const int  SERVO_HZ = 50;
static const int  SERVO_BITS = 16;

// ===================== Drive motors (from drive_steer_test) =====================
// Both motors get the same command. Flip one if it runs backward
// (mirror-image mounting usually needs one inverted).
static const bool MOTOR1_INVERT    = false;
static const bool MOTOR2_INVERT    = false;
static const int  MOTOR_HZ         = 20000;
static const int  MOTOR_BITS       = 8;
static const int  MOTOR_MAX        = 255;
static const bool MOTOR_SLOW_DECAY = true;
static const int  MOTOR_MIN_DUTY_DEFAULT = 35;  // set this to the "find" result to keep it after reset
static int        motorMinDuty     = MOTOR_MIN_DUTY_DEFAULT;  // live value, change with "find" or "md N"
static const int  MOTOR_KICK_MS    = 0;     // full-power start pulse. 0 = off (caused the start jerk)
static const int  MOTOR_RAMP_PER_S = 400;   // throttle rise limit, speed units per second (0 -> 255 in 0.65 s)
static const int  THR_MIN_US       = 1050;  // below this = zero throttle
static const int  THR_MAX_US       = 1950;  // above this = full throttle

// Core 2.x LEDC channels. 0-1 timer 0 (servo 50 Hz), 2-3 timer 1 and 4-5 timer 2 (motors 20 kHz)
static const int CH_STEER_L = 0;
static const int CH_STEER_R = 1;
static const int CH_IN1     = 2;
static const int CH_IN2     = 3;
static const int CH_IN3     = 4;
static const int CH_IN4     = 5;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch, hz, bits) ledcAttach((pin), (hz), (bits))
  #define PWM_WRITE(pin, ch, duty)      ledcWrite((pin), (duty))
#else
  #define PWM_ATTACH(pin, ch, hz, bits) do { ledcSetup((ch), (hz), (bits)); ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty)      ledcWrite((ch), (duty))
#endif

// Modes as plain ints (Arduino IDE prototype issue with enum parameters)
#define MODE_UNKNOWN      -1
#define MODE_DRIVE         0
#define MODE_FLIGHT        1
#define MODE_TRANSFORMING  2

// ===================== CRSF tap =====================
HardwareSerial &crsf = Serial2;
uint8_t  frameBuf[64];
int      frameIdx = 0;
uint16_t chRaw[16];
bool     gotFrame = false;
unsigned long lastFrameMs = 0;
uint32_t frameCount = 0, crcErrors = 0, framesLastSec = 0;
unsigned long rateT0 = 0;

uint8_t crc8(const uint8_t *d, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
  }
  return c;
}

void handleFrame() {
  // frameBuf: [sync][len][type][payload...][crc]
  if (frameBuf[2] != 0x16 || frameBuf[1] != 24) return;  // RC channels packed only
  const uint8_t *p = &frameBuf[3];
  uint32_t bits = 0;
  int nbits = 0, ci = 0;
  for (int i = 0; i < 22 && ci < 16; i++) {
    bits |= (uint32_t)p[i] << nbits;
    nbits += 8;
    while (nbits >= 11 && ci < 16) {
      chRaw[ci++] = bits & 0x7FF;
      bits >>= 11;
      nbits -= 11;
    }
  }
  gotFrame = true;
  lastFrameMs = millis();
  frameCount++;
}

void pollCrsf() {
  while (crsf.available()) {
    uint8_t b = crsf.read();
    if (frameIdx == 0) {
      if (b == 0xC8 || b == 0xEE || b == 0xEA) frameBuf[frameIdx++] = b;
      continue;
    }
    if (frameIdx == 1) {
      if (b < 2 || b > 62) { frameIdx = 0; continue; }
      frameBuf[frameIdx++] = b;
      continue;
    }
    frameBuf[frameIdx++] = b;
    int total = frameBuf[1] + 2;
    if (frameIdx >= total) {
      if (crc8(&frameBuf[2], frameBuf[1] - 1) == frameBuf[total - 1]) handleFrame();
      else crcErrors++;
      frameIdx = 0;
    }
  }
}

bool linkOk() { return gotFrame && (millis() - lastFrameMs) < LINK_TIMEOUT_MS; }

// Serial RC: set a channel (1-based) to a pulse width, stored as CRSF ticks
bool simLink = true;
void setChUs(int n, int us) {
  if (n < 1 || n > 16) return;
  long t = 992 + (long)(us - 1500) * 8 / 5;
  chRaw[n - 1] = (uint16_t)constrain(t, 172L, 1811L);
}

// Channel n (1-based) in microseconds. CRSF 172..1811 -> about 988..2012 us.
int chUs(int n) {
  if (n < 1 || n > 16) return 1500;
  return 1500 + ((int)chRaw[n - 1] - 992) * 5 / 8;
}

// ===================== Steppers =====================
static const int NUM_PHASES = 8;
static const int SEQ[NUM_PHASES][4] = {   // {AIN2, AIN1, BIN1, BIN2}
  {0, 1, 1, 0}, {0, 0, 1, 0}, {1, 0, 1, 0}, {1, 0, 0, 0},
  {1, 0, 0, 1}, {0, 0, 0, 1}, {0, 1, 0, 1}, {0, 1, 0, 0},
};
const int *stepPins[2] = { M1_PINS, M2_PINS };
const int  extendDir[2] = { M1_EXTEND_DIR, M2_EXTEND_DIR };
long pos[2] = {0, 0};      // 0 = retracted, TRAVEL_STEPS = extended
long tgt[2] = {0, 0};
int  phase[2] = {0, 0};
bool coilsOn = false;
bool stepPaused = false;
unsigned long lastStepUs = 0;
long rampCount = 0;        // steps since the move started, for the speed ramp
int  moveDir[2] = {0, 0};
const int STEP_UNIT = STEP_FULL ? 2 : 1;   // half-step units per step

float strokeSeconds(long halfSteps) {
  return (labs(halfSteps) / STEP_UNIT) * (stepIntervalUs / 1e6f);
}

void applyPhase(int n) {
  for (int i = 0; i < 4; i++) digitalWrite(stepPins[n][i], SEQ[phase[n]][i]);
}

void allCoilsOff() {
  for (int n = 0; n < 2; n++)
    for (int i = 0; i < 4; i++) digitalWrite(stepPins[n][i], LOW);
  coilsOn = false;
}

bool armsMoving() { return pos[0] != tgt[0] || pos[1] != tgt[1]; }

void serviceSteppers() {
  if (!armsMoving()) {
    if (coilsOn) {
      allCoilsOff();
      Serial.printf(">> Arms stopped. M1 %ld, M2 %ld of %ld steps\n", pos[0], pos[1], TRAVEL_STEPS);
    }
    rampCount = 0;
    moveDir[0] = moveDir[1] = 0;
    return;
  }
  // Pause on link loss, but only if a radio was ever seen (serial-only bench moves still work)
  if (gotFrame && !linkOk()) {
    if (!stepPaused) { allCoilsOff(); stepPaused = true; Serial.println(">> Link lost. Arms paused."); }
    rampCount = 0;
    return;
  }
  if (stepPaused) { stepPaused = false; Serial.println(">> Link back. Arms resuming."); }

  // Trapezoid speed: start slow, speed up each step, slow down again near the target
  long remaining = max(labs(tgt[0] - pos[0]), labs(tgt[1] - pos[1])) / STEP_UNIT;
  long r = min(rampCount, remaining);
  long iv = (long)STEP_START_US - r * (long)STEP_RAMP_US;
  if (iv < (long)stepIntervalUs) iv = (long)stepIntervalUs;

  unsigned long now = micros();
  if (now - lastStepUs < (unsigned long)iv) return;
  lastStepUs = now;

  bool dirChanged = false;
  for (int n = 0; n < 2; n++) {
    long diff = tgt[n] - pos[n];
    if (diff == 0) continue;
    int d = (diff > 0) ? 1 : -1;
    if (moveDir[n] != 0 && d != moveDir[n]) dirChanged = true;
    moveDir[n] = d;
    int s = (labs(diff) >= STEP_UNIT) ? STEP_UNIT : 1;   // odd leftover finishes with a half-step
    pos[n] += d * s;
    phase[n] = ((phase[n] + d * s * extendDir[n]) % NUM_PHASES + NUM_PHASES) % NUM_PHASES;
    applyPhase(n);
  }
  rampCount = dirChanged ? 0 : rampCount + 1;   // reversing mid-move restarts the ramp
  coilsOn = true;
}

// ===================== Mode handling =====================
int requested = MODE_DRIVE;       // boot assumption: arms retracted
int swStable = MODE_UNKNOWN;
int swCandidate = MODE_UNKNOWN;
unsigned long swCandT = 0;

void setRequest(int m) {
  requested = m;
  long t = (m == MODE_FLIGHT) ? TRAVEL_STEPS : 0;
  tgt[0] = t;
  tgt[1] = t;
  Serial.printf(">> Request %s: arms %s (%ld steps, about %.1f s)\n",
                m == MODE_FLIGHT ? "FLIGHT" : "DRIVE",
                m == MODE_FLIGHT ? "extending" : "retracting",
                labs(t - pos[0]), strokeSeconds(t - pos[0]));
}

void readModeSwitch() {
  if (!linkOk()) return;
  int us = chUs(CH_MODE);
  int r = swCandidate;                                  // middle band holds the last state
  if (us < MODE_FLIGHT_BELOW_US) r = MODE_FLIGHT;
  else if (us > MODE_DRIVE_ABOVE_US) r = MODE_DRIVE;
  if (r != swCandidate) { swCandidate = r; swCandT = millis(); }
  if (swCandidate == MODE_UNKNOWN || swCandidate == swStable) return;
  if (millis() - swCandT < MODE_DEBOUNCE_MS) return;

  int old = swStable;
  swStable = swCandidate;
  if (old == MODE_UNKNOWN) {
    if (swStable != requested)
      Serial.println(">> SC reads FLIGHT at boot. Arms stay retracted. Flip SC to DRIVE, then FLIGHT.");
    else
      Serial.println(">> SC reads DRIVE. In sync with arms.");
    return;
  }
  setRequest(swStable);   // one flip = one transform
}

int vehicleMode() {
  if (armsMoving()) return MODE_TRANSFORMING;
  if (pos[0] == 0 && pos[1] == 0) return MODE_DRIVE;
  if (pos[0] == TRAVEL_STEPS && pos[1] == TRAVEL_STEPS) return MODE_FLIGHT;
  return MODE_TRANSFORMING;   // jogged or stopped part way, ground stays locked
}

const char *modeName(int m) {
  switch (m) {
    case MODE_DRIVE:  return "DRIVE";
    case MODE_FLIGHT: return "FLIGHT";
    case MODE_TRANSFORMING: return "TRANSFORM";
    default: return "?";
  }
}

// ===================== Steering servo =====================
int steerUs = -1;

uint32_t usToDuty(int us) {
  return (uint32_t)((uint64_t)us * ((1UL << SERVO_BITS) - 1) / 20000UL);
}

void steerWriteUs(int us) {
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  if (us == steerUs) return;
  steerUs = us;
  int out = STEER_INVERT ? (SERVO_MIN_US + SERVO_MAX_US - us) : us;
  PWM_WRITE(PIN_STEER_L, CH_STEER_L, usToDuty(out));
  if (PIN_STEER_R >= 0) PWM_WRITE(PIN_STEER_R, CH_STEER_R, usToDuty(out));
}

// Proportional: more stick = sharper turn. Full stick = full servo throw.
int steerFromStick(int stickUs) {
  int off = stickUs - 1500;
  if (abs(off) <= STEER_DEADBAND_US) return SERVO_CENTER_US;
  off = constrain(off, -500, 500);
  if (off < 0) return SERVO_CENTER_US + (long)off * (SERVO_CENTER_US - SERVO_MIN_US) / 500;
  return SERVO_CENTER_US + (long)off * (SERVO_MAX_US - SERVO_CENTER_US) / 500;
}

// ===================== Drive motors =====================
int  motorCmd = 0;            // last speed command, -255..255
bool motorBraked = false;
unsigned long kickUntil = 0;
int  motorDuty = 0;

struct DriveMotor { int pinA; int chA; int pinB; int chB; bool invert; };
static const DriveMotor DRIVE[2] = {
  { PIN_DRV_IN1, CH_IN1, PIN_DRV_IN2, CH_IN2, MOTOR1_INVERT },   // channel A
  { PIN_DRV_IN3, CH_IN3, PIN_DRV_IN4, CH_IN4, MOTOR2_INVERT },   // channel B
};

void channelWrite(const DriveMotor &m, int duty, int dir) {
  duty = constrain(duty, 0, MOTOR_MAX);
  if (m.invert) dir = -dir;
  if (dir == 0 || duty == 0) {
    PWM_WRITE(m.pinA, m.chA, 0);
    PWM_WRITE(m.pinB, m.chB, 0);
    return;
  }
  int pwm = MOTOR_SLOW_DECAY ? (MOTOR_MAX - duty) : duty;
  if (dir > 0) {
    PWM_WRITE(m.pinA, m.chA, MOTOR_SLOW_DECAY ? MOTOR_MAX : pwm);
    PWM_WRITE(m.pinB, m.chB, MOTOR_SLOW_DECAY ? pwm : 0);
  } else {
    PWM_WRITE(m.pinA, m.chA, MOTOR_SLOW_DECAY ? pwm : 0);
    PWM_WRITE(m.pinB, m.chB, MOTOR_SLOW_DECAY ? MOTOR_MAX : pwm);
  }
}

// Same command to both drive motors
void motorWriteRaw(int duty, int dir) {
  for (int i = 0; i < 2; i++) channelWrite(DRIVE[i], duty, dir);
}

void motorBrake() {
  for (int i = 0; i < 2; i++) {                // both inputs high = brake, both motors
    PWM_WRITE(DRIVE[i].pinA, DRIVE[i].chA, MOTOR_MAX);
    PWM_WRITE(DRIVE[i].pinB, DRIVE[i].chB, MOTOR_MAX);
  }
  motorCmd = 0;
  motorDuty = 0;
  motorBraked = true;
  kickUntil = 0;
}

// speed -255..255. Non-blocking kick so the CRSF parser keeps running.
void motorDrive(int speed) {
  speed = constrain(speed, -MOTOR_MAX, MOTOR_MAX);
  int dir = (speed > 0) - (speed < 0);       // per-motor invert is applied in channelWrite
  int mag = abs(speed);
  int duty = 0;
  if (mag > 0) duty = motorMinDuty + (long)(mag - 1) * (MOTOR_MAX - motorMinDuty) / (MOTOR_MAX - 1);

  bool fromRest = (motorCmd == 0) || ((motorCmd > 0) != (speed > 0));
  if (dir != 0 && fromRest && MOTOR_KICK_MS > 0) kickUntil = millis() + MOTOR_KICK_MS;
  motorCmd = speed;
  motorBraked = false;
  motorDuty = duty;

  bool kicking = (dir != 0) && (kickUntil != 0) && ((long)(millis() - kickUntil) < 0);
  motorWriteRaw(kicking ? MOTOR_MAX : duty, dir);
}

int throttleFromStick(int thrUs) {
  if (thrUs <= THR_MIN_US) return 0;
  if (thrUs >= THR_MAX_US) return MOTOR_MAX;
  return (long)(thrUs - THR_MIN_US) * MOTOR_MAX / (THR_MAX_US - THR_MIN_US);
}

// Rise-limited throttle. Speeding up is ramped. Slowing down and stopping are instant.
int rampSpeed = 0;
unsigned long lastRampMs = 0;

int slewSpeed(int target) {
  unsigned long now = millis();
  unsigned long dt = min(now - lastRampMs, 50UL);
  lastRampMs = now;
  if (target == 0 || (rampSpeed != 0 && (target > 0) != (rampSpeed > 0))) {
    rampSpeed = 0;                                   // stop, or direction change: drop to zero first
    if (target == 0) return 0;
  }
  if (abs(target) <= abs(rampSpeed)) { rampSpeed = target; return rampSpeed; }   // slowing down
  int step = max(1, (int)(MOTOR_RAMP_PER_S * dt / 1000));
  int dir = (target > 0) ? 1 : -1;
  rampSpeed += dir * step;
  if (abs(rampSpeed) > abs(target)) rampSpeed = target;
  return rampSpeed;
}

// Ramp the raw duty up from 0. Press Enter the moment the wheel turns.
void findStartDuty() {
  if (vehicleMode() != MODE_DRIVE) { Serial.println("find only runs in DRIVE with the arms retracted."); return; }
  Serial.println("Find start duty. Wheels off the bench. Press Enter the moment both wheels turn.");
  while (Serial.available()) Serial.read();
  int found = -1;
  for (int d = 0; d <= MOTOR_MAX; d += 5) {
    motorWriteRaw(d, 1);
    Serial.printf("  duty %d\n", d);
    unsigned long t = millis();
    while (millis() - t < 400) {
      if (Serial.available()) { found = d; break; }
      delay(5);
    }
    if (found >= 0) break;
  }
  while (Serial.available()) Serial.read();
  motorBrake();
  if (found >= 0) {
    motorMinDuty = max(0, found - 10);
    Serial.printf("Wheel started at duty %d. Motor min duty now %d.\n", found, motorMinDuty);
    Serial.printf("To keep it: set MOTOR_MIN_DUTY_DEFAULT = %d and upload.\n", motorMinDuty);
  } else {
    Serial.println("No response by 255. Check the 5 V rail under load and the motor leads.");
  }
}

// ===================== Ground outputs =====================
bool thrGateOpen = false;
bool gateMsgShown = false;
int  lastMode = MODE_UNKNOWN;

void updateGroundOutputs() {
  int m = vehicleMode();
  if (m != lastMode) {
    Serial.printf(">> Mode: %s\n", modeName(m));
    lastMode = m;
  }

  if (m != MODE_DRIVE || !linkOk()) {
    motorBrake();                       // FLIGHT, TRANSFORM or no link: brake hard
    steerWriteUs(SERVO_CENTER_US);      // steering locked at centre
    rampSpeed = 0;
    lastRampMs = millis();
    thrGateOpen = false;
    gateMsgShown = false;
    return;
  }

  // DRIVE mode with a live link
  steerWriteUs(steerFromStick(chUs(CH_STEER)));

  int thr = throttleFromStick(chUs(CH_THROTTLE));
  if (!thrGateOpen) {
    if (thr == 0) {
      thrGateOpen = true;
      Serial.println(">> Throttle gate open. Drive enabled.");
    } else {
      if (!gateMsgShown) { Serial.println(">> Lower the throttle to the bottom to enable drive."); gateMsgShown = true; }
      motorDrive(0);
      rampSpeed = 0;
      lastRampMs = millis();
      return;
    }
  }

  bool reverse = (CH_REVERSE > 0) && (chUs(CH_REVERSE) > 1700);
  int speed = slewSpeed(reverse ? -thr : thr);
  if (speed != motorCmd || motorBraked) motorDrive(speed);
  else if (kickUntil != 0 && (long)(millis() - kickUntil) >= 0) { kickUntil = 0; motorDrive(speed); }  // kick over, drop to set duty
}

// ===================== Status and serial commands =====================
bool statusOn = true;
unsigned long lastStatusMs = 0;

void printStatus() {
  if (RC_FROM_SERIAL) {
    Serial.printf("%-9s | serial RC link %s | SC %4d THR %4d ST %4d | motor %s %3d | steer %4d | M1 %5ld M2 %5ld /%ld\n",
                  modeName(vehicleMode()), linkOk() ? "on " : "OFF",
                  chUs(CH_MODE), chUs(CH_THROTTLE), chUs(CH_STEER),
                  motorBraked ? "BRAKE" : "run  ", motorCmd,
                  steerUs, pos[0], pos[1], TRAVEL_STEPS);
    return;
  }
  Serial.printf("%-9s | link %s %3lu Hz crcErr %lu | SC %4d THR %4d ST %4d | motor %s %3d | steer %4d | M1 %5ld M2 %5ld /%ld\n",
                modeName(vehicleMode()),
                linkOk() ? "OK " : "---", (unsigned long)framesLastSec, (unsigned long)crcErrors,
                linkOk() ? chUs(CH_MODE) : 0, linkOk() ? chUs(CH_THROTTLE) : 0, linkOk() ? chUs(CH_STEER) : 0,
                motorBraked ? "BRAKE" : "run  ", motorCmd,
                steerUs, pos[0], pos[1], TRAVEL_STEPS);
}

void printHelp() {
  if (RC_FROM_SERIAL) Serial.println(
    "\nRadio stand-in (serial RC)\n"
    "  drive        SC up   (CH6 2000): arms retract, ground controls live\n"
    "  flight       SC down (CH6 1000): arms extend, motor brakes\n"
    "  mid          SC middle (CH6 1500): no change\n"
    "  t N          throttle N % (0-100). Below 6 % reads as zero\n"
    "  s N          steering N % (-100 full left, 0 centre, 100 full right)\n"
    "  c            steering centre\n"
    "  x            throttle zero\n"
    "  ch N US      set any channel N to US microseconds\n"
    "  link off/on  simulate radio link loss / recovery");
  Serial.println(
    "\nCommands\n"
    "  p            status line on/off\n"
    "  ext / ret    force arms out (FLIGHT) / in (DRIVE), bench use\n"
    "  stop         stop the arms where they are\n"
    "  j1 N / j2 N  jog motor 1 or 2 by N half-steps (+ = extend direction)\n"
    "  zero         mark the current arm position as fully retracted\n"
    "  spd N        stepper cruise speed, us per step (1200-8000), default 2200\n"
    "  find         find the drive motors' start duty (arms retracted, wheels up)\n"
    "  md N         set motor min duty directly (0-200)\n"
    "  info         print the transform settings\n"
    "  h            help\n");
}

void printInfo() {
  Serial.printf("Pinion %d T, rack travel %d T -> %.2f output turns\n", PINION_TEETH, RACK_TEETH, (float)RACK_TEETH / PINION_TEETH);
  Serial.printf("%ld half-steps per turn -> %ld half-steps per stroke\n", STEPS_PER_REV, TRAVEL_STEPS);
  Serial.printf("%s drive, cruise %lu us per step -> stroke about %.1f s\n",
                STEP_FULL ? "Full-step" : "Half-step", stepIntervalUs, strokeSeconds(TRAVEL_STEPS));
  Serial.printf("Drive motors: min duty %d, ramp %d per s, kick %d ms, invert M1 %d M2 %d\n",
                motorMinDuty, MOTOR_RAMP_PER_S, MOTOR_KICK_MS, MOTOR1_INVERT, MOTOR2_INVERT);
  Serial.printf("Drive pins: motor 1 D%d/D%d, motor 2 GPIO%d (RX2)/GPIO%d (D4)\n",
                PIN_DRV_IN1, PIN_DRV_IN2, PIN_DRV_IN3, PIN_DRV_IN4);
  Serial.printf("Extend dir M1 %+d, M2 %+d\n", M1_EXTEND_DIR, M2_EXTEND_DIR);
  Serial.printf("Channels: steer CH%d, throttle CH%d, mode CH%d, reverse %s\n",
                CH_STEER, CH_THROTTLE, CH_MODE, CH_REVERSE ? "on" : "off");
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  // Radio stand-in commands
  if (RC_FROM_SERIAL) {
    if (cmd == "drive")  { setChUs(CH_MODE, 2000); Serial.println("SC -> DRIVE (2000)"); return; }
    if (cmd == "flight") { setChUs(CH_MODE, 1000); Serial.println("SC -> FLIGHT (1000)"); return; }
    if (cmd == "mid")    { setChUs(CH_MODE, 1500); Serial.println("SC -> middle (1500)"); return; }
    if (cmd.startsWith("t ")) {
      int pct = constrain((int)cmd.substring(2).toInt(), 0, 100);
      setChUs(CH_THROTTLE, 1000 + pct * 10);
      Serial.printf("Throttle %d %% (%d us)\n", pct, chUs(CH_THROTTLE));
      return;
    }
    if (cmd.startsWith("s ")) {
      int pct = constrain((int)cmd.substring(2).toInt(), -100, 100);
      setChUs(CH_STEER, 1500 + pct * 5);
      Serial.printf("Steering %d %% (%d us)\n", pct, chUs(CH_STEER));
      return;
    }
    if (cmd == "c") { setChUs(CH_STEER, 1500); Serial.println("Steering centre"); return; }
    if (cmd == "x") { setChUs(CH_THROTTLE, 1000); Serial.println("Throttle zero"); return; }
    if (cmd.startsWith("ch ")) {
      int sp = cmd.indexOf(' ', 3);
      if (sp > 0) {
        int n = cmd.substring(3, sp).toInt();
        int us = cmd.substring(sp + 1).toInt();
        setChUs(n, us);
        Serial.printf("CH%d -> %d us\n", n, chUs(n));
        return;
      }
    }
    if (cmd == "link off") { simLink = false; Serial.println("Link OFF (simulated loss)"); return; }
    if (cmd == "link on")  { simLink = true;  Serial.println("Link ON"); return; }
  }

  if      (cmd == "h")    printHelp();
  else if (cmd == "p")    { statusOn = !statusOn; Serial.printf("Status %s\n", statusOn ? "on" : "off"); }
  else if (cmd == "ext")  setRequest(MODE_FLIGHT);
  else if (cmd == "ret")  setRequest(MODE_DRIVE);
  else if (cmd == "stop") { tgt[0] = pos[0]; tgt[1] = pos[1]; Serial.println(">> Arms stopped by command"); }
  else if (cmd.startsWith("j1 ")) { tgt[0] = pos[0] + (cmd.substring(3).toInt() / STEP_UNIT) * STEP_UNIT; Serial.printf(">> Jog M1 to %ld\n", tgt[0]); }
  else if (cmd.startsWith("j2 ")) { tgt[1] = pos[1] + (cmd.substring(3).toInt() / STEP_UNIT) * STEP_UNIT; Serial.printf(">> Jog M2 to %ld\n", tgt[1]); }
  else if (cmd == "zero") {
    if (armsMoving()) Serial.println("Stop the arms first.");
    else { pos[0] = pos[1] = tgt[0] = tgt[1] = 0; requested = MODE_DRIVE; Serial.println(">> Position zeroed. Arms = retracted."); }
  }
  else if (cmd.startsWith("spd ")) {
    stepIntervalUs = (unsigned long)constrain((int)cmd.substring(4).toInt(), 1200, 8000);
    Serial.printf(">> Stepper cruise %lu us per step, stroke about %.1f s\n", stepIntervalUs, strokeSeconds(TRAVEL_STEPS));
  }
  else if (cmd == "find") findStartDuty();
  else if (cmd.startsWith("md ")) {
    motorMinDuty = constrain((int)cmd.substring(3).toInt(), 0, 200);
    Serial.printf(">> Motor min duty %d\n", motorMinDuty);
  }
  else if (cmd == "info") printInfo();
  else Serial.println("Unknown command. Type h.");
}

// ===================== Setup / loop =====================
void setup() {
  Serial.begin(115200);
  delay(300);

  // Radios off. ESP32 Wi-Fi/BT is the top suspect for RP3 link drops.
  WiFi.mode(WIFI_OFF);
  btStop();

  for (int n = 0; n < 2; n++)
    for (int i = 0; i < 4; i++) { pinMode(stepPins[n][i], OUTPUT); digitalWrite(stepPins[n][i], LOW); }

  PWM_ATTACH(PIN_STEER_L, CH_STEER_L, SERVO_HZ, SERVO_BITS);
  if (PIN_STEER_R >= 0) PWM_ATTACH(PIN_STEER_R, CH_STEER_R, SERVO_HZ, SERVO_BITS);
  steerWriteUs(SERVO_CENTER_US);

  for (int i = 0; i < 16; i++) chRaw[i] = 992;

  // CRSF: 420000 baud 8N1, RX only on D35. The UART claims TX2 (GPIO17) as an idle output.
  // Leave TX2 unconnected.
  if (RC_FROM_SERIAL) {
    // Safe starting stick positions: SC in DRIVE (matches retracted arms), throttle zero, steering centre
    setChUs(CH_MODE, 2000);
    setChUs(CH_THROTTLE, 1000);
    setChUs(CH_STEER, 1500);
    gotFrame = true;
    lastFrameMs = millis();
  } else {
    crsf.setRxBufferSize(1024);
    crsf.begin(420000, SERIAL_8N1, PIN_CRSF_RX, -1);
  }

  // Drive motor PWM goes on after Serial2 starts, so the LEDC owns RX2 (GPIO16)
  // even on core versions that default UART2 to GPIO16/17.
  for (int i = 0; i < 2; i++) {
    PWM_ATTACH(DRIVE[i].pinA, DRIVE[i].chA, MOTOR_HZ, MOTOR_BITS);
    PWM_ATTACH(DRIVE[i].pinB, DRIVE[i].chB, MOTOR_HZ, MOTOR_BITS);
  }
  motorBrake();

  Serial.println("\n=== RYZE OS V1 (REV1RYZEOS) ready ===");
  Serial.println(RC_FROM_SERIAL ? "RC source: SERIAL (type commands, no receiver needed)"
                                : "RC source: RP3 CRSF tap on D35");
  Serial.println("Arms assumed fully RETRACTED at boot.");
  printInfo();
  printHelp();
}

void loop() {
  if (RC_FROM_SERIAL) {
    if (simLink) lastFrameMs = millis();   // typed values count as a live link
  } else {
    pollCrsf();
  }
  readModeSwitch();
  serviceSteppers();

  static unsigned long lastOutMs = 0;
  if (millis() - lastOutMs >= 10) {          // ground outputs at 100 Hz
    lastOutMs = millis();
    updateGroundOutputs();
  }

  if (millis() - rateT0 >= 1000) {
    framesLastSec = frameCount;
    frameCount = 0;
    rateT0 = millis();
  }

  // 1 Hz status in serial mode so the monitor stays readable while you type
  if (statusOn && millis() - lastStatusMs >= (RC_FROM_SERIAL ? 1000UL : 250UL)) {
    lastStatusMs = millis();
    printStatus();
  }

  handleSerial();
}
