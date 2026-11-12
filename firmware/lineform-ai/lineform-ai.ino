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

// Framed replies (READ_ALL / READ_ALL_LOAD) so Processing can never mix up
// which bytes belong to which motor, even if a reply arrives late:
//   0xA5 0x5A <header> <seq> <22 data bytes, in DXL_IDS (chain) order> <checksum>
// checksum = (header + seq + all data bytes) & 0xFF
const byte FRAME_START_1 = 0xA5;
const byte FRAME_START_2 = 0x5A;

const float PROTOCOL_VERSION = 2.0;

// Position P Gain for each stiffness level (0 = stiffest). Lower values
// feel softer; higher values feel stiffer and respond more aggressively.
const uint16_t STIFFNESS_P_GAINS[4] = {1200, 700, 350, 120};

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
bool motorHaveGoodRaw[NUM_MOTORS];

int32_t readPresentRaw(int i) {
  int32_t raw = (int32_t)dxl.getPresentPosition(DXL_IDS[i], UNIT_RAW);
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

int32_t angleByteToRaw(uint8_t id, int compressedAngle) {
  float degrees = map(compressedAngle, 0, 250, 0, 360);
  degrees = constrain(degrees, 60, 310);
  degrees += angleOffsetOf(id);                   // per-motor correction
  if (isReversed(id)) degrees = 360.0 - degrees; // flip motors mounted the other way
  int32_t raw = (degrees / 360.0) * 4095;
  return constrain(raw, 683, 3526);
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
      dxl.setGoalPosition(DXL_IDS[i], sw_data[i].goal_position);
    }
  }
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
    motorPGain[i] = STIFFNESS_P_GAINS[0];
    motorIGain[i] = DROOP_I_GAIN;
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
#ifdef USE_PLAIN_POSITION_MODE
    dxl.setOperatingMode(motorID, OP_POSITION);
#else
    if (!dxl.setOperatingMode(motorID, OP_CURRENT_BASED_POSITION)) {
      dxl.setOperatingMode(motorID, OP_POSITION);
    }
#endif
    enableMotor(i);
  }
}

void loop() {

  // Wait until at least one command byte arrives.
  if (DEBUG_SERIAL.available() <= 0) return;

  byte header = DEBUG_SERIAL.read();
  if (cmdCount[header] < 65000) cmdCount[header]++;

  // CREATE SHAPE / ANIMATION FRAME - all 11 goal positions
  if (header == SYNC_HEADER) {
    if (!waitForBytes(NUM_MOTORS)) return;
    byte angles[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) angles[i] = DEBUG_SERIAL.read();
    // If newer full frames are already waiting, jump straight to the newest
    // so the arm never lags behind (or overflows on) a fast animation stream.
    while (DEBUG_SERIAL.available() >= NUM_MOTORS + 1 && DEBUG_SERIAL.peek() == SYNC_HEADER) {
      DEBUG_SERIAL.read();
      for (int i = 0; i < NUM_MOTORS; i++) angles[i] = DEBUG_SERIAL.read();
    }
    writeAllGoals(angles);
  }

  // ONE motor's goal position (live slider dragging)
  else if (header == SET_ONE_GOAL_HEADER) {
    if (!waitForBytes(2)) return;
    int dxlId = DEBUG_SERIAL.read();
    int compressedAngle = DEBUG_SERIAL.read();
    int i = indexForId(dxlId);
    if (i >= 0) {
      dxl.setGoalPosition(dxlId, goalRawFor(i, compressedAngle));
    }
  }

  // TORQUE ON / OFF (Processing sends the real hardware ID, 1-11)
  else if (header == TORQUE_HEADER) {
    if (!waitForBytes(2)) return;
    int dxlId = DEBUG_SERIAL.read();
    int stateValue = DEBUG_SERIAL.read();
    int i = indexForId(dxlId);
    if (i >= 0) {
      if (stateValue == 1) enableMotor(i);
      else dxl.torqueOff(dxlId);
    }
  }

  // TORQUE LIMIT - percent of MAX_GOAL_CURRENT_MA; 0 = limp (torque off)
  else if (header == TORQUE_LIMIT_HEADER) {
    if (!waitForBytes(2)) { lastTlResult = 1; return; }
    int dxlId = DEBUG_SERIAL.read();
    int rawPct = DEBUG_SERIAL.read();
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

  // STIFFNESS - Position P Gain level 0-3 (used by animations)
  else if (header == STIFFNESS_HEADER) {
    if (!waitForBytes(2)) return;
    int dxlId = DEBUG_SERIAL.read();
    int level = constrain(DEBUG_SERIAL.read(), 0, 3);
    int i = indexForId(dxlId);
    if (i >= 0) {
      motorPGain[i] = STIFFNESS_P_GAINS[level];
      motorIGain[i] = (level == 0) ? DROOP_I_GAIN : 0; // softer levels: no I term
      dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, dxlId, motorIGain[i]);
      dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, dxlId, motorPGain[i]); // RAM item - fine with torque on
    }
  }

  // READ ONE POSITION - lets Processing ask where a motor actually is.
  else if (header == READ_POSITION_HEADER) {
    if (!waitForBytes(1)) return;
    int dxlId = DEBUG_SERIAL.read();
    int i = indexForId(dxlId);
    if (i >= 0) sendDegrees(dxlId, readPresentRaw(i));
    else sendDegrees(dxlId, 2047); // not fitted: report 180 deg
  }

  // READ ALL TORQUE USE - how hard each motor is pushing right now, as a
  // percent of its Current Limit (same scale as the torque sliders), x10.
  else if (header == READ_ALL_LOAD_HEADER) {
    if (!waitForBytes(1)) return;
    byte seq = DEBUG_SERIAL.read();
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
    if (!waitForBytes(1)) return;
    byte seq = DEBUG_SERIAL.read();
    // [0] uptime s, [1] 255 sync, [2] 254 torque on/off, [3] 251 torque limit,
    // [4] 249 one goal, [5] 250 read all, [6] 252 stiffness, [7] 253 read one,
    // [8] timeouts, [9] unknown headers (not >= 242),
    // [10] last TORQUE_LIMIT: result*1000 + id byte  (pct byte in [11]? no room -> see 244)
    uint32_t unknown = 0;
    for (int h = 0; h < 242; h++) unknown += cmdCount[h];
    int16_t values[NUM_MOTORS] = {
      cap16(millis() / 1000),
      cap16(cmdCount[255]), cap16(cmdCount[254]), cap16(cmdCount[251]), cap16(cmdCount[249]),
      cap16(cmdCount[250]), cap16(cmdCount[252]), cap16(cmdCount[253]), cap16(cmdTimeouts),
      cap16(unknown), (int16_t)(lastTlResult < 0 ? -1 : lastTlResult * 1000 + lastTlId) };
    sendFrame(header, seq, values);
  }

  else if (header == STATUS_LAST_HEADER || header == STATUS_ERR_HEADER) {
    if (!waitForBytes(1)) return;
    byte seq = DEBUG_SERIAL.read();
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
    if (!waitForBytes(1)) return;
    byte seq = DEBUG_SERIAL.read();
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
    if (!waitForBytes(1)) return;
    byte seq = DEBUG_SERIAL.read();
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      values[i] = (i < ACTIVE_MOTORS) ? degreesX100(DXL_IDS[i], readPresentRaw(i)) : 18000;
    }
    sendFrame(header, seq, values);
  }
}
