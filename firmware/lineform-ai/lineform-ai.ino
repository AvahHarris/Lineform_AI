#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#else
  #define DEBUG_SERIAL Serial
#endif

// writeControlTableItem() takes the library's named items
// (ControlTableItem::POSITION_P_GAIN, ...), NOT raw register addresses.

// --- Configuration Constants ---
const int NUM_MOTORS = 11;

// Commands from Processing (first byte of each message)
const byte SYNC_HEADER          = 255; // + 11 angle bytes        -> set all goal positions
const byte TORQUE_HEADER        = 254; // + id + 0/1              -> torque off / on
const byte READ_POSITION_HEADER = 253; // + id                    -> replies 2 bytes (deg*100)
const byte STIFFNESS_HEADER     = 252; // + id + level 0-3        -> Position P Gain from the table below
const byte TORQUE_LIMIT_HEADER  = 251; // + id + percent 0-100    -> goal current (0 = torque off / limp)
const byte READ_ALL_HEADER      = 250; // + seq                   -> framed reply: 11 x angle (deg*100)
const byte SET_ONE_GOAL_HEADER  = 249; // + id + angle byte       -> set ONE motor's goal position
const byte READ_ALL_LOAD_HEADER = 248; // + seq                   -> framed reply: 11 x torque used (% x10)
const byte STATUS_HEADER        = 247; // + seq                   -> framed reply: 11 x (mode*1000 + torqueOn*100 + hwError), -1 = not fitted
const byte STATUS_PWM_HEADER    = 246; // + seq                   -> framed reply: 11 x Goal PWM as % of PWM Limit (x10)
const byte STATUS_CUR_HEADER    = 245; // + seq                   -> framed reply: 11 x Goal Current as % of Current Limit (x10)
const byte STATUS_LAST_HEADER   = 244; // + seq                   -> framed reply: 11 x last torque % received*100 + PWM write ok*10 + current write ok
const byte STATUS_COUNTS_HEADER = 242; // + seq                   -> framed reply: [0] seconds since boot, [1..] commands received per type (see below)
const byte STATUS_ERR_HEADER    = 243; // + seq                   -> framed reply: 11 x lib error code of the last PWM write*100 + of the last current write
const byte HOLD_HERE_HEADER     = 241; // + id + percent 0-100    -> goal = EXACT present position, then torque on at percent (0 = stay limp)
                                       //    Used when the GUI leaves MANIPULATION mode: the arm keeps the pose it was
                                       //    moved to by hand, with no 1.44 deg angle-byte rounding and no turn guessing.
const byte STATUS_PGAIN_HEADER  = 237; // + seq                   -> framed reply: 11 x Position P Gain read back FROM the motor (-1 = n/a)
const byte STATUS_MODEL_HEADER  = 238; // + seq                   -> framed reply: 11 x model number the motor reports
                                       //    (+20000 = no MODEL_GAINS_LIST entry for it: uses the default set; -1 = n/a)
const byte STATUS_HOLD_HEADER   = 239; // + seq                   -> framed reply, per motor: torque on (1/0)*10000
                                       //    + goal write ok (1/0)*5000 + min(4999, |present - HOLD_HERE goal| in 0.1 deg);
                                       //    -1 = not fitted / no hold yet
const byte CHECK_RECOVER_HEADER = 240; // + seq                   -> checks EVERY motor and restarts any that is shut down;
                                       //    framed reply: 11 x (result code*1000 + hardware error found), -1 = not fitted
                                       //    codes: 0 ok, 1 torque was OFF -> restarted, 2 hardware error -> rebooted + restarted,
                                       //           3 not answering -> rebooted, back + restarted, 4 STILL not answering,
                                       //           5 restart failed, 6 torque 0% on purpose -> left off

// Framed replies (READ_ALL / READ_ALL_LOAD) so Processing can never mix up
// which bytes belong to which motor, even if a reply arrives late:
//   0xA5 0x5A <header> <seq> <22 data bytes, in DXL_IDS (chain) order> <checksum>
// checksum = (header + seq + all data bytes) & 0xFF
const byte FRAME_START_1 = 0xA5;
const byte FRAME_START_2 = 0x5A;

const float PROTOCOL_VERSION = 2.0;

// Torque limit. The motors run in CURRENT-BASED POSITION mode, so each
// motor's Goal Current caps how hard it can push. 100% = the motor's own
// Current Limit (read from the motor at startup, never written), so full
// torque is exactly as strong as before.

// Physical Dynamixel IDs in CHAIN ORDER - first entry = the joint next to
// the fixed base, last entry = the tip. On this arm the IDs count UP from
// the base: 1 next to the base ... 11 at the tip, same as the GUI labels.
// Create Shape frames, animations and the live readings use this order
// (slot i -> DXL_IDS[i]); single-motor commands use the ID itself.
const uint8_t DXL_IDS[NUM_MOTORS] = {
  1, 2, 3, 4, 5, 6,
  7, 8, 9, 10, 11
};

// Motors mounted facing the other way, listed by their GUI MOTOR NUMBER
// (1 = base ... 11 = tip, same as the slider labels) - not the hardware ID.
// Their angle is mirrored around the 180 deg neutral - sent as 360 - angle
// and read back the same way - so every motor bends the same direction for
// the same slider change.
//
// The list is specific to each physical arm, so it lives in local_config.h
// (git-ignored, never uploaded). Copy local_config.example.h to
// local_config.h and fill in REVERSED_MOTORS_LIST for your arm. Without that
// file nothing is reversed and the sketch still compiles.
#if __has_include("local_config.h")
  #include "local_config.h"
#endif
#ifndef REVERSED_MOTORS_LIST
  #define REVERSED_MOTORS_LIST
#endif

// Position P Gain for each stiffness level (0 = stiffest, used for normal
// holding; 1-3 = softer, used by hinge taps / animations / the GUI's twitch
// safety net). Lower = softer; higher = stiffer and more aggressive.
// Per arm (motor model!): override in local_config.h with
//   #define STIFFNESS_P_GAINS_LIST  1200, 700, 350, 120
// and scale single motors with P_GAIN_PCT_LIST (below).
#ifndef STIFFNESS_P_GAINS_LIST
  #define STIFFNESS_P_GAINS_LIST 1200, 700, 350, 120
#endif
// (Used as the DEFAULT gain set - see MODEL_GAINS_LIST below.)
// How many motors this arm really has (the first ACTIVE_MOTORS entries of
// DXL_IDS, base -> tip). The serial protocol always carries NUM_MOTORS
// slots so the same GUI works on every arm; slots past ACTIVE_MOTORS are
// ignored on writes and report 180 deg / 0 load on reads.
#ifndef ACTIVE_MOTORS
  #define ACTIVE_MOTORS NUM_MOTORS
#endif

// Per-motor angle correction in degrees, by GUI motor number (1 = base ...
// 11 = tip): offset = what the GUI SHOWS minus the REAL angle. E.g. a motor
// that reads 209 while physically straight (180) gets 29. Used for motors
// whose horn is mounted slightly rotated. Set in local_config.h.
#ifndef ANGLE_OFFSETS_DEG
  #define ANGLE_OFFSETS_DEG 0
#endif
const float ANGLE_OFFSETS[NUM_MOTORS] = { ANGLE_OFFSETS_DEG }; // missing entries = 0

// Gentler moves: caps each motor's speed/acceleration (RAM, re-applied on
// every torque-on). Big jumps like Reset then ramp instead of slamming, which
// cuts the peak load that trips the overload error. 0 = no limit.
// Units: velocity 0.229 rpm, acceleration 214.6 rev/min^2.
#ifndef MOVE_PROFILE_VELOCITY
  #define MOVE_PROFILE_VELOCITY 150   // ~34 rpm, ~200 deg/s
#endif
#ifndef MOVE_PROFILE_ACCELERATION
  #define MOVE_PROFILE_ACCELERATION 40
#endif

// Position I Gain - removes gravity droop. With P only, a loaded motor
// always rests a few degrees short of its goal; the I term slowly adds
// push until it is exactly there, so the GUI slider, saved shapes and the
// real arm agree. Only used at stiffness level 0 (full stiffness) - softer
// levels stay compliant. 0 = off. Raise if motors still droop, lower if
// they overshoot or wobble after a move. Override in local_config.h.
#ifndef DROOP_I_GAIN
  #define DROOP_I_GAIN 150
#endif
// JOINT RANGE: +/-105 deg off the straight pose - the normal +/-90 plus 15
// deg of extra travel some motors have. In the GUI-frame degrees used by
// this protocol (180 = straight) that is 75..285 - the GUI SHOWS it as
// -15..195 with 90 = straight. Every goal is clamped to this; readings are
// not (so the GUI can show a joint pushed past a limit at its true angle).
// Must match JOINT_MIN_DEG / JOINT_MAX_DEG in Gui_3_0.pde.
const float JOINT_MIN_DEG = 75;
const float JOINT_MAX_DEG = 285;

// ---- GAINS PER MOTOR MODEL (chains that mix motor variants) ----
// Every Dynamixel reports its model number (register 0), e.g. XL330-M077 =
// 1190, XL330-M288 = 1200 (the GUI's startup diagnostics print what each
// motor reports). MODEL_GAINS_LIST in local_config.h gives one gain set per
// model; each motor automatically uses the set for ITS model:
//   { model number, P level 0, P level 1, P level 2, P level 3, I gain (level 0) }
//   #define MODEL_GAINS_LIST  {1190, 800, 450, 220, 80, 30}, {1200, 1200, 700, 350, 120, 150}
//   (first = XL330-M077, second = XL330-M288 - one line, or end each line with a backslash)
// A motor whose model has no entry uses STIFFNESS_P_GAINS_LIST + DROOP_I_GAIN
// (the single-model settings above - still fine for an arm with one model).
// MOTOR_MODEL_OVERRIDE_LIST (by motor number, 0 = use what it reports) can
// force a motor onto another model's set.
struct GainSet { int32_t model; int32_t p[4]; int32_t i; };
const GainSet DEFAULT_GAIN_SET = { 0, { STIFFNESS_P_GAINS_LIST }, DROOP_I_GAIN };
#ifdef MODEL_GAINS_LIST
const GainSet MODEL_GAINS[] = { MODEL_GAINS_LIST };
const int MODEL_GAINS_COUNT = sizeof(MODEL_GAINS) / sizeof(MODEL_GAINS[0]);
#else
const GainSet MODEL_GAINS[] = { DEFAULT_GAIN_SET };
const int MODEL_GAINS_COUNT = 0; // no per-model sets: everyone uses the default
#endif
#ifndef MOTOR_MODEL_OVERRIDE_LIST
  #define MOTOR_MODEL_OVERRIDE_LIST 0
#endif
const int32_t MOTOR_MODEL_OVERRIDE[NUM_MOTORS] = { MOTOR_MODEL_OVERRIDE_LIST };

int32_t motorModel[NUM_MOTORS];          // what each motor reports (-1 = couldn't read)
const GainSet* motorGains[NUM_MOTORS];   // the set it uses

// The gain set for a model number (the default if it has no entry).
const GainSet* gainSetForModel(int32_t model) {
  for (int k = 0; k < MODEL_GAINS_COUNT; k++) if (MODEL_GAINS[k].model == model) return &MODEL_GAINS[k];
  return &DEFAULT_GAIN_SET;
}

// Per-motor stiffness trim on top of its model's set, % by motor number
// (1 = base ... 11 = tip). 0 or missing = 100%. For a single motor that
// oscillates ("twitches") more than others of its model. Set in local_config.h:
//   #define P_GAIN_PCT_LIST  100, 100, 100, 100, 100, 50, 50
#ifndef P_GAIN_PCT_LIST
  #define P_GAIN_PCT_LIST 100
#endif
const uint8_t P_GAIN_PCT[NUM_MOTORS] = { P_GAIN_PCT_LIST };

uint16_t pGainFor(int i, int level) {
  const GainSet* gs = motorGains[i] ? motorGains[i] : &DEFAULT_GAIN_SET;
  uint16_t pct = P_GAIN_PCT[i] == 0 ? 100 : P_GAIN_PCT[i];
  uint32_t g = (uint32_t)gs->p[constrain(level, 0, 3)] * pct / 100;
  return (uint16_t)(g < 1 ? 1 : (g > 16383 ? 16383 : g));
}

// I gain at full stiffness (level 0) for motor i - from its model's set.
uint16_t iGainFor(int i) {
  const GainSet* gs = motorGains[i] ? motorGains[i] : &DEFAULT_GAIN_SET;
  return (uint16_t)constrain(gs->i, 0, 16383);
}

// Leading 0 is a placeholder (not a motor number) so the array is never empty.
const uint8_t REVERSED_MOTORS[] = { 0, REVERSED_MOTORS_LIST };

// Takes the hardware ID; converts it to the GUI motor number (its position
// in DXL_IDS, 1 = base) before checking the list.
// GUI motor number (1 = base) for a hardware ID, or -1.
int motorNumberOf(uint8_t id) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (DXL_IDS[i] == id) return i + 1;
  }
  return -1;
}

float angleOffsetOf(uint8_t id) {
  int n = motorNumberOf(id);
  return (n >= 1) ? ANGLE_OFFSETS[n - 1] : 0;
}

bool isReversed(uint8_t id) {
  int motorNumber = motorNumberOf(id);
  if (motorNumber < 1) return false;
  for (uint8_t k = 0; k < sizeof(REVERSED_MOTORS); k++) {
    if (REVERSED_MOTORS[k] == motorNumber) return true;
  }
  return false;
}

// SYNC WRITE: all 11 goal positions go out in ONE bus packet (no per-motor
// replies to wait for) - several times faster than 11 separate writes. This
// is what lets Processing stream animation frames smoothly (~25 per second).
const uint16_t GOAL_POSITION_ADDR = 116; // X-series control table
typedef struct sw_data {
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;
sw_data_t sw_data[NUM_MOTORS];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[NUM_MOTORS];

// Per-motor settings, remembered so turning torque back on restores them.
uint16_t motorPGain[NUM_MOTORS];
uint16_t motorIGain[NUM_MOTORS];
uint8_t motorTorquePercent[NUM_MOTORS];
int32_t motorCurrentLimitRaw[NUM_MOTORS]; // the motor's Current Limit, in its own raw units
int32_t motorPwmLimitRaw[NUM_MOTORS];     // the motor's PWM Limit (885 = 100% on X-series)
// Commands received since boot, by header (reported by 242) - shows whether
// a command ever arrives, and the uptime shows whether the board restarted.
uint16_t cmdCount[256];
uint16_t cmdTimeouts = 0;
// Last TORQUE_LIMIT seen: raw id byte, raw percent byte, and what happened
// (0 = applied, 1 = timed out waiting for its 2 data bytes, 2 = id not one of ours).
int16_t lastTlId = -1, lastTlPct = -1, lastTlResult = -1;
int16_t cap16(uint32_t v) { return (int16_t)(v > 32000 ? 32000 : v); }

// Debug record of the last torque-limit write per motor (reported by 244/243).
int16_t motorLastPctRecv[NUM_MOTORS];     // -1 = no TORQUE_LIMIT received yet
bool motorLastCurOk[NUM_MOTORS], motorLastPwmOk[NUM_MOTORS];
int16_t motorLastCurErr[NUM_MOTORS], motorLastPwmErr[NUM_MOTORS];

// MULTI-TURN GUARD. In current-based position mode the motors count whole
// turns: Present Position can read e.g. 6000 (one turn + 1904) instead of
// staying in 0-4095 like plain position mode (what the backup firmware used).
// A goal of 2047 ("180 deg") sent to a motor sitting at 6000 makes it spin
// almost a full turn backwards, through 0/360 deg, into its mechanical stop
// -> overload, red LED, torque off. So every goal is placed in the turn the
// motor is currently in (motorTurnBase = start of that turn), and readings
// are folded back into 0-4095 before being sent to the GUI.
int32_t motorTurnBase[NUM_MOTORS];

// Start of the turn containing rawPosition (floor to a multiple of 4096).
int32_t turnBaseOf(int32_t rawPosition) {
  if (rawPosition >= 0) return (rawPosition / 4096) * 4096;
  return -(((-rawPosition) + 4095) / 4096) * 4096;
}

// rawPosition folded into 0-4095 (the angle within one turn).
int32_t foldToOneTurn(int32_t rawPosition) {
  return rawPosition - turnBaseOf(rawPosition);
}


Dynamixel2Arduino dxl(
  DXL_SERIAL,
  DXL_DIR_PIN
);

// Reads a motor's present position and remembers which turn it is in.
// A FAILED read (busy bus, missed reply) makes the library return 0 - which
// used to be passed on as a real "0 deg" reading: the canvas showed the joint
// snapping to its limit for a frame, and the GUI could even take it for a hand
// push and send the motor there. Failed reads now repeat the last good value.
int32_t motorLastGoodRaw[NUM_MOTORS];

// Present / goal position through the GENERIC register access
// (readControlTableItem / writeControlTableItem) - NOT the library's
// getPresentPosition() / setGoalPosition(). Those convenience functions look
// the motor model up in the library's own range table first and silently
// fail (or clamp to 0-4095) for models / library versions it doesn't list -
// the same problem setGoalCurrent() / setGoalPWM() had here. The generic
// access only needs the register address, so it works the same on every
// XL330 variant (M077, M288, ...). Raw units, multi-turn, signed.
int32_t readPresentRawDirect(uint8_t id) {
  return (int32_t)dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
}
bool writeGoalRawDirect(uint8_t id, int32_t raw) {
  return dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, raw);
}
bool motorHaveGoodRaw[NUM_MOTORS];

int32_t readPresentRaw(int i) {
  int32_t raw = readPresentRawDirect(DXL_IDS[i]);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) {
    return motorHaveGoodRaw[i] ? motorLastGoodRaw[i] : 2047; // unknown yet: report 180
  }
  motorLastGoodRaw[i] = raw;
  motorHaveGoodRaw[i] = true;
  motorTurnBase[i] = turnBaseOf(raw);
  return raw;
}

// Chain index 0-10 for a hardware ID 1-11, or -1 if it isn't one of ours.
int indexForId(int dxlId) {
  for (int i = 0; i < ACTIVE_MOTORS; i++) {
    if (DXL_IDS[i] == dxlId) return i;
  }
  return -1;
}

// Waits (briefly) for n bytes from Processing. Returns false on timeout so
// a half-sent command can never freeze the loop forever.
bool waitForBytes(int n) {
  unsigned long start = millis();
  while (DEBUG_SERIAL.available() < n) {
    if (millis() - start > 200) { cmdTimeouts++; return false; }
  }
  return true;
}

// Torque limit for motor i, as motorTorquePercent[i] of its full strength.
// Applied two ways so it works whatever mode the motor ended up in:
//  - Goal Current (current-based position mode): caps the current/torque.
//  - Goal PWM (any position mode): caps the output voltage. Without this the
//    torque slider did NOTHING if the motor fell back to plain position mode
//    (e.g. the library didn't accept OP_CURRENT_BASED_POSITION, or
//    USE_PLAIN_POSITION_MODE is set) - only 0% (torque off) had any effect.
void applyGoalCurrent(int i) {
  uint8_t id = DXL_IDS[i];
  int32_t raw = (int32_t)((long)motorCurrentLimitRaw[i] * motorTorquePercent[i] / 100);
  if (raw < 1) raw = 1;
  // Written with writeControlTableItem() - the same generic write that works
  // for P gain / profile etc. The convenience functions setGoalCurrent() /
  // setGoalPWM() silently did NOTHING here: they first look the motor model
  // up in the library's own range table and give up if it isn't listed, so
  // the motors kept their power-on defaults (100%) and the torque slider had
  // no effect (diagnostics showed "motor: 100%" with the bar at 2%).
  motorLastCurOk[i] = dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, id, raw);
  motorLastCurErr[i] = (int16_t)dxl.getLastLibErrCode();
  int32_t pwm = (int32_t)((long)motorPwmLimitRaw[i] * motorTorquePercent[i] / 100);
  if (pwm < 1) pwm = 1;
  motorLastPwmOk[i] = dxl.writeControlTableItem(ControlTableItem::GOAL_PWM, id, pwm);
  motorLastPwmErr[i] = (int16_t)dxl.getLastLibErrCode();
}

// Torque on with this motor's remembered P gain and torque limit. Only
// RAM settings are written, so a motor that's already on never goes limp.
// If the motor is in a hardware-error state (e.g. overload - its LED blinks
// red and it ignores torque-on), it is rebooted first to clear the error,
// so Reset / torque-on can recover it without power-cycling the arm.
void enableMotor(int i) {
  uint8_t id = DXL_IDS[i];
  if (dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id) != 0) {
    dxl.reboot(id);
    delay(100); // let the motor restart before talking to it again
  }
  readPresentRaw(i); // which turn is it in? (a reboot can also change this)
  dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, motorPGain[i]);
  dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, motorIGain[i]);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, MOVE_PROFILE_ACCELERATION);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, MOVE_PROFILE_VELOCITY);
  dxl.torqueOn(id);
  applyGoalCurrent(i);
}

// HOLD HERE: make the motor's goal its exact present position (raw,
// multi-turn - so no wrap-around and no angle-byte rounding), THEN torque
// on. Writing the goal before torque-on means the motor never drives back
// to an old goal from before it was moved by hand. If the position can't be
// read at all, the goal is left alone rather than guessed.
float rawToDegrees(uint8_t id, int32_t rawPosition); // defined further down
int32_t guiDegToRaw(uint8_t id, float degrees);

// What the last HOLD_HERE set, for the hold check (command 239).
int32_t motorHoldGoalRaw[NUM_MOTORS];
bool motorHoldGoalOk[NUM_MOTORS];

void holdHere(int i) {
  uint8_t id = DXL_IDS[i];
  if (dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id) != 0) {
    dxl.reboot(id);
    delay(100);
  }
  bool ok = false;
  int32_t raw = 0;
  for (int attempt = 0; attempt < 3 && !ok; attempt++) {
    raw = readPresentRawDirect(id);
    ok = (dxl.getLastLibErrCode() == DXL_LIB_OK);
  }
  if (!ok && motorHaveGoodRaw[i]) { raw = motorLastGoodRaw[i]; ok = true; }
  if (ok) {
    motorLastGoodRaw[i] = raw;
    motorHaveGoodRaw[i] = true;
    motorTurnBase[i] = turnBaseOf(raw);
    // Hold exactly here - unless the joint was pushed past a limit by hand:
    // then hold AT the limit (straight +/-90).
    int32_t goal = raw;
    float deg = rawToDegrees(id, raw);
    if (deg < JOINT_MIN_DEG || deg > JOINT_MAX_DEG) goal = motorTurnBase[i] + guiDegToRaw(id, deg);
    // Written BEFORE torque-on, so the motor never drives back to the goal
    // it had before it was moved by hand. Read back to make sure it stuck.
    bool wrote = false;
    for (int attempt = 0; attempt < 3 && !wrote; attempt++) {
      writeGoalRawDirect(id, goal);
      int32_t g = (int32_t)dxl.readControlTableItem(ControlTableItem::GOAL_POSITION, id);
      wrote = (dxl.getLastLibErrCode() == DXL_LIB_OK && g == goal);
    }
    motorHoldGoalRaw[i] = goal;
    motorHoldGoalOk[i] = wrote;
  } else {
    motorHoldGoalOk[i] = false; // position unknown: goal left as it was
  }
  enableMotor(i); // gains, profile, torque on, torque limit
}

// ---- Check & recover (command 240, sent by the GUI on every Reset) ----
bool pingRetry(uint8_t id, int tries) {
  for (int k = 0; k < tries; k++) {
    if (dxl.ping(id)) return true;
    delay(5);
  }
  return false;
}

// Generic read that also says whether the motor actually answered
// (readControlTableItem() returns 0 on failure, which looks like "torque off").
bool readItemOk(uint8_t item, uint8_t id, int32_t &out) {
  out = dxl.readControlTableItem(item, id);
  return dxl.getLastLibErrCode() == DXL_LIB_OK;
}

// Checks one motor and restarts it if it is shut down. A restarted motor
// HOLDS WHERE IT IS (holdHere) - the GUI re-sends its target afterwards.
int16_t checkAndRecover(int i) {
  uint8_t id = DXL_IDS[i];
  int code = 0;
  int32_t errBefore = 0;

  // 1. Is it answering at all? If not, try a reboot (clears a hung motor;
  //    can't help one that has really lost power or its cable).
  if (!pingRetry(id, 3)) {
    dxl.reboot(id);
    delay(200);
    if (!pingRetry(id, 3)) return 4000;
    code = 3;
  }

  // 2. Hardware error (overload, overheating, voltage...) and torque state.
  int32_t err = 0, on = 0;
  if (readItemOk(ControlTableItem::HARDWARE_ERROR_STATUS, id, err) && err != 0) {
    errBefore = err;
    if (code == 0) code = 2;
  }
  bool onOk = readItemOk(ControlTableItem::TORQUE_ENABLE, id, on);

  // Torque set to 0% from the GUI: limp on purpose - leave it off.
  if (motorTorquePercent[i] == 0) {
    if (onOk && on) dxl.torqueOff(id);
    return (int16_t)((code == 0 ? 6 : code) * 1000 + constrain(errBefore, 0, 255));
  }

  if (code == 0 && onOk && on && err == 0) return 0; // healthy

  // 3. Restart: reboot if in error, goal = where it is now, torque on, limits.
  if (code == 0) code = 1;
  holdHere(i);

  // 4. Did it work?
  int32_t on2 = 0, err2 = 0;
  bool on2Ok = readItemOk(ControlTableItem::TORQUE_ENABLE, id, on2);
  readItemOk(ControlTableItem::HARDWARE_ERROR_STATUS, id, err2);
  if (!on2Ok || !on2 || err2 != 0) code = 5;
  return (int16_t)(code * 1000 + constrain(errBefore, 0, 255));
}

// Sends one framed reply: 11 int16 values (high byte first), in DXL_IDS order.
void sendFrame(byte header, byte seq, int16_t values[NUM_MOTORS]) {
  byte sum = header + seq;
  DEBUG_SERIAL.write(FRAME_START_1);
  DEBUG_SERIAL.write(FRAME_START_2);
  DEBUG_SERIAL.write(header);
  DEBUG_SERIAL.write(seq);
  for (int i = 0; i < NUM_MOTORS; i++) {
    byte hi = (values[i] >> 8) & 0xFF;
    byte lo = values[i] & 0xFF;
    DEBUG_SERIAL.write(hi);
    DEBUG_SERIAL.write(lo);
    sum += hi;
    sum += lo;
  }
  DEBUG_SERIAL.write(sum);
}

// Present position in degrees, in the GUI's frame (reversed motors flipped).
float rawToDegrees(uint8_t id, int32_t rawPosition) {
  float deg = (foldToOneTurn(rawPosition) / 4095.0) * 360.0;
  if (isReversed(id)) deg = 360.0 - deg;
  return deg - angleOffsetOf(id); // per-motor correction
}

int16_t degreesX100(uint8_t id, int32_t rawPosition) {
  return (int16_t)(rawToDegrees(id, rawPosition) * 100);
}

void sendDegrees(uint8_t id, int32_t rawPosition) {
  float currentDegrees = rawToDegrees(id, rawPosition);
  // Sent as (degrees * 100) in 2 bytes, high byte first.
  int16_t encoded = (int16_t)(currentDegrees * 100);
  DEBUG_SERIAL.write((encoded >> 8) & 0xFF);
  DEBUG_SERIAL.write(encoded & 0xFF);
}

// GUI-frame degrees (180 = straight) -> raw position within one turn.
// Clamped to the joint range JOINT_MIN_DEG..JOINT_MAX_DEG (straight +/-90).
int32_t guiDegToRaw(uint8_t id, float degrees) {
  degrees = constrain(degrees, JOINT_MIN_DEG, JOINT_MAX_DEG);
  degrees += angleOffsetOf(id);                   // per-motor correction
  if (isReversed(id)) degrees = 360.0 - degrees; // flip motors mounted the other way
  int32_t raw = (int32_t)(degrees / 360.0 * 4095 + 0.5);
  // Last-resort guard in the motor's OWN frame (after offset + reversal):
  // 10..350 deg, i.e. never across 0/360. (Was 60..310, which with motor 3's
  // +40 deg offset would have silently capped it near the old limit.)
  return constrain(raw, 114, 3981);
}

int32_t angleByteToRaw(uint8_t id, int compressedAngle) {
  // Float math: Arduino's map() is integer-only and always rounded DOWN
  // (up to 1.44 deg low on every command).
  return guiDegToRaw(id, compressedAngle * 360.0 / 250.0);
}

// Goal for chain index i, placed in the turn that motor is currently in.
int32_t goalRawFor(int i, int compressedAngle) {
  return motorTurnBase[i] + angleByteToRaw(DXL_IDS[i], compressedAngle);
}

// All 11 goal positions at once (sync write); falls back to one-by-one
// writes if the sync write is ever refused.
void writeAllGoals(const byte angles[NUM_MOTORS]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    sw_data[i].goal_position = goalRawFor(i, angles[i]);
  }
  sw_infos.is_info_changed = true;
  if (!dxl.syncWrite(&sw_infos)) {
    for (int i = 0; i < ACTIVE_MOTORS; i++) {
      writeGoalRawDirect(DXL_IDS[i], sw_data[i].goal_position);
    }
  }
}

// Reads motor i's model and picks its gain set. Called from setup().
void assignGainSet(int i) {
  int32_t m = -1;
  if (i < ACTIVE_MOTORS) {
    for (int attempt = 0; attempt < 3 && m < 0; attempt++) {
      int32_t v = (int32_t)dxl.readControlTableItem(ControlTableItem::MODEL_NUMBER, DXL_IDS[i]);
      if (dxl.getLastLibErrCode() == DXL_LIB_OK && v > 0) m = v;
    }
  }
  motorModel[i] = m;
  int32_t use = MOTOR_MODEL_OVERRIDE[i] > 0 ? MOTOR_MODEL_OVERRIDE[i] : m;
  motorGains[i] = gainSetForModel(use);
}

// Operating mode via the register (torque must be off). True if it reads back.
bool setModeChecked(uint8_t id, int32_t mode) {
  for (int attempt = 0; attempt < 3; attempt++) {
    int32_t now = (int32_t)dxl.readControlTableItem(ControlTableItem::OPERATING_MODE, id);
    if (dxl.getLastLibErrCode() == DXL_LIB_OK && now == mode) return true;
    dxl.writeControlTableItem(ControlTableItem::OPERATING_MODE, id, mode);
    delay(10);
  }
  return false;
}

void setup() {
  // Communication with Processing.
  DEBUG_SERIAL.begin(115200);

  // Wait up to 3 seconds for USB serial.
  while (!DEBUG_SERIAL && millis() < 3000) {
  }

  // Communication with the Dynamixel chain.
  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL_VERSION);

  // Sync write setup (same pattern as ROBOTIS's sync_read_write_position example).
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = GOAL_POSITION_ADDR;
  sw_infos.addr_length = 4;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;
  for (int i = 0; i < ACTIVE_MOTORS; i++) {
    info_xels_sw[i].id = DXL_IDS[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;

  // Configure all motors: current-based position mode (position control
  // with a torque limit), full stiffness, 100% torque.
  for (int i = 0; i < NUM_MOTORS; i++) {
    assignGainSet(i);             // which model is this motor -> its gains
    motorPGain[i] = pGainFor(i, 0);
    motorIGain[i] = iGainFor(i);
    motorTorquePercent[i] = 100;
    motorTurnBase[i] = 0;
    motorLastPctRecv[i] = -1;
    motorLastCurOk[i] = motorLastPwmOk[i] = false;
    motorLastCurErr[i] = motorLastPwmErr[i] = 0;
    if (i >= ACTIVE_MOTORS) continue; // not fitted on this arm

    uint8_t motorID = DXL_IDS[i];
    motorCurrentLimitRaw[i] = dxl.readControlTableItem(ControlTableItem::CURRENT_LIMIT, motorID);
    if (motorCurrentLimitRaw[i] <= 0) motorCurrentLimitRaw[i] = 1000; // couldn't read it - safe fallback
    motorPwmLimitRaw[i] = dxl.readControlTableItem(ControlTableItem::PWM_LIMIT, motorID);
    if (motorPwmLimitRaw[i] <= 0) motorPwmLimitRaw[i] = 885; // X-series default

    dxl.torqueOff(motorID); // the operating mode can only change with torque off

    // Answer reads immediately (an older version of this sketch accidentally
    // set a ~0.5 ms reply delay). EEPROM - only written if it isn't 0 already.
    if (dxl.readControlTableItem(ControlTableItem::RETURN_DELAY_TIME, motorID) != 0) {
      dxl.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, motorID, 0);
    }
    // Fall back to plain position mode on models without current control
    // (torque limits then have no effect, everything else still works).
    // USE_PLAIN_POSITION_MODE (local_config.h) = the backup firmware's mode:
    // no torque limiting, but always single-turn and more forgiving.
    // Written to the register directly and read back (same model-table
    // reason as above - setOperatingMode() can fail silently, leaving the
    // motor in whatever mode it was last set to). 5 = current-based
    // position, 3 = plain position. EEPROM: only written if different.
#ifdef USE_PLAIN_POSITION_MODE
    setModeChecked(motorID, 3);
#else
    if (!setModeChecked(motorID, 5)) setModeChecked(motorID, 3);
#endif
    enableMotor(i);
  }
}

// =====================================================================
//  COMMAND LINK - framing + flow control (GUI <-> this sketch)
//
//  Every command from the GUI is:  <header> <data bytes...> <checksum>
//    header   : always >= HEADER_MIN (230; one of the *_HEADER constants)
//    data     : fixed count per header (dataLenOf), every byte < HEADER_MIN
//               (in practice <= 199: angles <= 198, percents <= 100, seq < 200)
//    checksum : (header + all data bytes) & 0x7F
//
//  * Sliced / lost bytes can never be misread: a byte >= HEADER_MIN where data or
//    the checksum is expected means the command lost bytes - it is dropped
//    and that byte is treated as the next header. A wrong checksum drops it
//    too. (Before, a lost byte shifted everything after it - e.g. a stray
//    "0" read as a torque percent switched a motor off.)
//  * No overflow: every command WITHOUT a reply (motion, torque, stiffness,
//    hold) is answered with ACK (0x06) once it has been fully EXECUTED - or
//    NAK (0x15) if it was dropped. The GUI waits for that before sending
//    anything else, so only one command is ever in flight. Commands WITH a
//    reply (all reads / status / check) are acknowledged by the reply itself.
// =====================================================================
const int HEADER_MIN = 230;  // was 240 until 239 (STATUS_HOLD) was added
const byte ACK_BYTE = 0x06;
const byte NAK_BYTE = 0x15;
uint16_t cmdTruncated = 0, cmdBadChecksum = 0, cmdStray = 0;

// Data bytes that follow each header (-1 = not a command).
int dataLenOf(byte h) {
  switch (h) {
    case SYNC_HEADER:          return NUM_MOTORS;
    case TORQUE_HEADER:
    case STIFFNESS_HEADER:
    case TORQUE_LIMIT_HEADER:
    case SET_ONE_GOAL_HEADER:
    case HOLD_HERE_HEADER:     return 2;
    case READ_POSITION_HEADER:
    case READ_ALL_HEADER:
    case READ_ALL_LOAD_HEADER:
    case STATUS_HEADER:
    case STATUS_PWM_HEADER:
    case STATUS_CUR_HEADER:
    case STATUS_LAST_HEADER:
    case STATUS_ERR_HEADER:
    case STATUS_COUNTS_HEADER:
    case CHECK_RECOVER_HEADER:
    case STATUS_HOLD_HEADER:
    case STATUS_MODEL_HEADER:
    case STATUS_PGAIN_HEADER:  return 1;
    default:                   return -1;
  }
}

// Every 1-data-byte command is a read that answers with its own reply.
bool hasReply(byte h) { return dataLenOf(h) == 1; }

// Next byte without consuming it, waiting up to 200 ms; -1 on timeout.
int peekByteWait() {
  if (!waitForBytes(1)) return -1;
  return DEBUG_SERIAL.peek();
}

void handleCommand(byte header, const byte d[]); // defined below loop()

void dropCommand(byte header) {
  if (header == TORQUE_LIMIT_HEADER) lastTlResult = 1;
  if (!hasReply(header)) DEBUG_SERIAL.write(NAK_BYTE); // GUI resends it once
}

void loop() {
  if (DEBUG_SERIAL.available() <= 0) return;

  // Re-sync: anything that isn't a header is a leftover from a broken command.
  if (DEBUG_SERIAL.peek() < HEADER_MIN) { DEBUG_SERIAL.read(); cmdStray++; return; }
  byte header = DEBUG_SERIAL.read();
  int n = dataLenOf(header);
  if (n < 0) { cmdStray++; return; }

  byte d[NUM_MOTORS];
  byte sum = header;
  for (int k = 0; k < n; k++) {
    int b = peekByteWait();
    if (b < 0 || b >= HEADER_MIN) { cmdTruncated++; dropCommand(header); return; } // lost bytes: a new header (left unread) arrived early
    d[k] = DEBUG_SERIAL.read();
    sum += d[k];
  }
  int c = peekByteWait();
  if (c < 0 || c >= HEADER_MIN) { cmdTruncated++; dropCommand(header); return; }
  DEBUG_SERIAL.read();
  if (c != (sum & 0x7F)) { cmdBadChecksum++; dropCommand(header); return; }

  if (cmdCount[header] < 65000) cmdCount[header]++;
  handleCommand(header, d);
  if (!hasReply(header)) DEBUG_SERIAL.write(ACK_BYTE); // done - GUI may send the next one
}

// Runs one complete, checksum-verified command. d[] = its data bytes.
void handleCommand(byte header, const byte d[]) {

  // CREATE SHAPE / ANIMATION FRAME - all 11 goal positions
  if (header == SYNC_HEADER) {
    byte angles[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) angles[i] = d[i];
    writeAllGoals(angles);
  }

  // ONE motor's goal position (live slider dragging)
  else if (header == SET_ONE_GOAL_HEADER) {
    int dxlId = d[0];
    int compressedAngle = d[1];
    int i = indexForId(dxlId);
    if (i >= 0) {
      writeGoalRawDirect(dxlId, goalRawFor(i, compressedAngle));
    }
  }

  // TORQUE ON / OFF (Processing sends the real hardware ID, 1-11)
  else if (header == TORQUE_HEADER) {
    int dxlId = d[0];
    int stateValue = d[1];
    int i = indexForId(dxlId);
    if (i >= 0) {
      if (stateValue == 1) enableMotor(i);
      else dxl.torqueOff(dxlId);
    }
  }

  // TORQUE LIMIT - percent of MAX_GOAL_CURRENT_MA; 0 = limp (torque off)
  else if (header == TORQUE_LIMIT_HEADER) {
    int dxlId = d[0];
    int rawPct = d[1];
    int percent = constrain(rawPct, 0, 100);
    lastTlId = dxlId;
    lastTlPct = rawPct;
    int i = indexForId(dxlId);
    lastTlResult = (i >= 0) ? 0 : 2;
    if (i >= 0) {
      motorLastPctRecv[i] = percent;
      motorTorquePercent[i] = percent;
      if (percent == 0) {
        dxl.torqueOff(dxlId);
      } else {
        enableMotor(i); // torque on (if it was off) + the new limit
      }
    }
  }

  // CHECK & RECOVER - every motor answering, torque on, no hardware error?
  // Restarts any that isn't (Reset sends this).
  else if (header == CHECK_RECOVER_HEADER) {
    byte seq = d[0];
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      values[i] = (i < ACTIVE_MOTORS) ? checkAndRecover(i) : -1;
    }
    sendFrame(header, seq, values);
  }

  // MODEL / GAIN DIAGNOSTICS - which model each motor is, whether
  // local_config.h has a gain set for it, and the P gain it really runs.
  else if (header == STATUS_MODEL_HEADER || header == STATUS_PGAIN_HEADER) {
    byte seq = d[0];
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (i >= ACTIVE_MOTORS) { values[i] = -1; continue; }
      if (header == STATUS_MODEL_HEADER) {
        int32_t use = MOTOR_MODEL_OVERRIDE[i] > 0 ? MOTOR_MODEL_OVERRIDE[i] : motorModel[i];
        bool matched = motorGains[i] != &DEFAULT_GAIN_SET || (MODEL_GAINS_COUNT == 0);
        values[i] = (int16_t)(constrain(use, -1, 9999) + (matched ? 0 : 20000));
      } else {
        int32_t g = (int32_t)dxl.readControlTableItem(ControlTableItem::POSITION_P_GAIN, DXL_IDS[i]);
        values[i] = (dxl.getLastLibErrCode() == DXL_LIB_OK) ? (int16_t)constrain(g, 0, 32000) : -1;
      }
    }
    sendFrame(header, seq, values);
  }

  // HOLD CHECK - after leaving manipulation mode: is each motor really
  // holding where HOLD_HERE put it?
  else if (header == STATUS_HOLD_HEADER) {
    byte seq = d[0];
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (i >= ACTIVE_MOTORS || motorLastPctRecv[i] < 0) { values[i] = -1; continue; }
      uint8_t id = DXL_IDS[i];
      int32_t on = (int32_t)dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id);
      bool onOk = dxl.getLastLibErrCode() == DXL_LIB_OK;
      int32_t pres = readPresentRawDirect(id);
      bool presOk = dxl.getLastLibErrCode() == DXL_LIB_OK;
      int32_t dev = presOk ? (int32_t)(fabs((float)(pres - motorHoldGoalRaw[i])) * 3600.0 / 4096.0) : 4999;
      values[i] = (int16_t)((onOk && on ? 10000 : 0) + (motorHoldGoalOk[i] ? 5000 : 0) + constrain(dev, 0, 4999));
    }
    sendFrame(header, seq, values);
  }

  // HOLD HERE - leave manipulation mode: hold the hand-posed angle exactly.
  else if (header == HOLD_HERE_HEADER) {
    int dxlId = d[0];
    int percent = constrain(d[1], 0, 100);
    int i = indexForId(dxlId);
    if (i >= 0) {
      motorLastPctRecv[i] = percent;
      motorTorquePercent[i] = percent;
      if (percent == 0) dxl.torqueOff(dxlId);
      else holdHere(i);
    }
  }

  // STIFFNESS - Position P Gain level 0-3 (used by animations)
  else if (header == STIFFNESS_HEADER) {
    int dxlId = d[0];
    int level = constrain(d[1], 0, 3);
    int i = indexForId(dxlId);
    if (i >= 0) {
      motorPGain[i] = pGainFor(i, level);
      motorIGain[i] = (level == 0) ? iGainFor(i) : 0; // softer levels: no I term
      dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, dxlId, motorIGain[i]);
      dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, dxlId, motorPGain[i]); // RAM item - fine with torque on
    }
  }

  // READ ONE POSITION - lets Processing ask where a motor actually is.
  else if (header == READ_POSITION_HEADER) {
    int dxlId = d[0];
    int i = indexForId(dxlId);
    if (i >= 0) sendDegrees(dxlId, readPresentRaw(i));
    else sendDegrees(dxlId, 2047); // not fitted: report 180 deg
  }

  // READ ALL TORQUE USE - how hard each motor is pushing right now, as a
  // percent of its Current Limit (same scale as the torque sliders), x10.
  else if (header == READ_ALL_LOAD_HEADER) {
    byte seq = d[0];
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (i >= ACTIVE_MOTORS) { values[i] = 0; continue; }
      // Generic read (see applyGoalCurrent() for why not getPresentCurrent()).
      // Present Current is a signed 16-bit value.
      int32_t rc = dxl.readControlTableItem(ControlTableItem::PRESENT_CURRENT, DXL_IDS[i]);
      if (rc > 32767) rc -= 65536;
      float raw = rc;
      float pct = motorCurrentLimitRaw[i] > 0 ? fabs(raw) * 100.0 / motorCurrentLimitRaw[i] : 0;
      values[i] = (int16_t)constrain(pct * 10.0, 0.0, 32000.0);
    }
    sendFrame(header, seq, values);
  }

  // DIAGNOSTICS - what each motor is REALLY set to (read back from the
  // motors, not from this sketch's memory). Printed by the GUI at startup
  // and on every Reset.
  else if (header == STATUS_COUNTS_HEADER) {
    byte seq = d[0];
    // [0] uptime s, [1] 255 sync, [2] 254 torque on/off, [3] 251 torque limit,
    // [4] 249 one goal, [5] 250 read all, [6] 252 stiffness, [7] 253 read one,
    // [8] DROPPED commands (lost bytes / bad checksum / timed out),
    // [9] stray bytes skipped while re-syncing,
    // [10] last TORQUE_LIMIT: result*1000 + id byte  (pct byte in [11]? no room -> see 244)
    int16_t values[NUM_MOTORS] = {
      cap16(millis() / 1000),
      cap16(cmdCount[255]), cap16(cmdCount[254]), cap16(cmdCount[251]), cap16(cmdCount[249]),
      cap16(cmdCount[250]), cap16(cmdCount[252]), cap16(cmdCount[253]),
      cap16((uint32_t)cmdTruncated + cmdBadChecksum + cmdTimeouts),
      cap16(cmdStray), (int16_t)(lastTlResult < 0 ? -1 : lastTlResult * 1000 + lastTlId) };
    sendFrame(header, seq, values);
  }

  else if (header == STATUS_LAST_HEADER || header == STATUS_ERR_HEADER) {
    byte seq = d[0];
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (header == STATUS_LAST_HEADER) {
        values[i] = (motorLastPctRecv[i] < 0) ? -1
          : (int16_t)(motorLastPctRecv[i] * 100 + (motorLastPwmOk[i] ? 10 : 0) + (motorLastCurOk[i] ? 1 : 0));
      } else {
        values[i] = (int16_t)(constrain(motorLastPwmErr[i], 0, 99) * 100 + constrain(motorLastCurErr[i], 0, 99));
      }
    }
    sendFrame(header, seq, values);
  }

  else if (header == STATUS_HEADER || header == STATUS_PWM_HEADER || header == STATUS_CUR_HEADER) {
    byte seq = d[0];
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (i >= ACTIVE_MOTORS) { values[i] = -1; continue; }
      uint8_t id = DXL_IDS[i];
      if (header == STATUS_HEADER) {
        int32_t mode = dxl.readControlTableItem(ControlTableItem::OPERATING_MODE, id);
        int32_t on = dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id);
        int32_t err = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);
        values[i] = (int16_t)(constrain(mode, 0, 31) * 1000 + (on ? 100 : 0) + constrain(err, 0, 99));
      } else if (header == STATUS_PWM_HEADER) {
        int32_t g = dxl.readControlTableItem(ControlTableItem::GOAL_PWM, id);
        values[i] = (int16_t)(motorPwmLimitRaw[i] > 0 ? g * 1000L / motorPwmLimitRaw[i] : -1);
      } else {
        int32_t g = dxl.readControlTableItem(ControlTableItem::GOAL_CURRENT, id);
        values[i] = (int16_t)(motorCurrentLimitRaw[i] > 0 ? g * 1000L / motorCurrentLimitRaw[i] : -1);
      }
    }
    sendFrame(header, seq, values);
  }

  // READ ALL POSITIONS - 11 x 2 bytes, base to tip (live angle sliders)
  else if (header == READ_ALL_HEADER) {
    byte seq = d[0];
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      values[i] = (i < ACTIVE_MOTORS) ? degreesX100(DXL_IDS[i], readPresentRaw(i)) : 18000;
    }
    sendFrame(header, seq, values);
  }
}
