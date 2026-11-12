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
// the fixed base, last entry = the tip. On this arm the IDs count DOWN from
// the base: 11 next to the base ... 1 at the tip. ID 12 is the fixed
// (passive) base block and is never driven. Create Shape frames and the
// live readings use this order; single-motor commands use the ID itself.
const uint8_t DXL_IDS[NUM_MOTORS] = {
  11, 10, 9, 8, 7, 6,
  5, 4, 3, 2, 1
};

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
uint8_t motorTorquePercent[NUM_MOTORS];
int32_t motorCurrentLimitRaw[NUM_MOTORS]; // the motor's Current Limit, in its own raw units

Dynamixel2Arduino dxl(
  DXL_SERIAL,
  DXL_DIR_PIN
);

// Chain index 0-10 for a hardware ID 1-11, or -1 if it isn't one of ours.
int indexForId(int dxlId) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (DXL_IDS[i] == dxlId) return i;
  }
  return -1;
}

// Waits (briefly) for n bytes from Processing. Returns false on timeout so
// a half-sent command can never freeze the loop forever.
bool waitForBytes(int n) {
  unsigned long start = millis();
  while (DEBUG_SERIAL.available() < n) {
    if (millis() - start > 200) return false;
  }
  return true;
}

void applyGoalCurrent(int i) {
  int32_t raw = (int32_t)((long)motorCurrentLimitRaw[i] * motorTorquePercent[i] / 100);
  if (raw < 1) raw = 1;
  dxl.setGoalCurrent(DXL_IDS[i], raw, UNIT_RAW);
}

// Torque on with this motor's remembered P gain and torque limit. Only
// RAM settings are written, so a motor that's already on never goes limp.
void enableMotor(int i) {
  uint8_t id = DXL_IDS[i];
  dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, motorPGain[i]);
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

int16_t degreesX100(int32_t rawPosition) {
  return (int16_t)((rawPosition / 4095.0) * 360.0 * 100);
}

void sendDegrees(int32_t rawPosition) {
  float currentDegrees = (rawPosition / 4095.0) * 360.0;
  // Sent as (degrees * 100) in 2 bytes, high byte first.
  int16_t encoded = (int16_t)(currentDegrees * 100);
  DEBUG_SERIAL.write((encoded >> 8) & 0xFF);
  DEBUG_SERIAL.write(encoded & 0xFF);
}

int32_t angleByteToRaw(int compressedAngle) {
  float degrees = map(compressedAngle, 0, 250, 0, 360);
  degrees = constrain(degrees, 60, 310);
  int32_t raw = (degrees / 360.0) * 4095;
  return constrain(raw, 683, 3526);
}

// All 11 goal positions at once (sync write); falls back to one-by-one
// writes if the sync write is ever refused.
void writeAllGoals(const byte angles[NUM_MOTORS]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    sw_data[i].goal_position = angleByteToRaw(angles[i]);
  }
  sw_infos.is_info_changed = true;
  if (!dxl.syncWrite(&sw_infos)) {
    for (int i = 0; i < NUM_MOTORS; i++) {
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
  for (int i = 0; i < NUM_MOTORS; i++) {
    info_xels_sw[i].id = DXL_IDS[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;

  // Configure all motors: current-based position mode (position control
  // with a torque limit), full stiffness, 100% torque.
  for (int i = 0; i < NUM_MOTORS; i++) {
    motorPGain[i] = STIFFNESS_P_GAINS[0];
    motorTorquePercent[i] = 100;

    uint8_t motorID = DXL_IDS[i];
    motorCurrentLimitRaw[i] = dxl.readControlTableItem(ControlTableItem::CURRENT_LIMIT, motorID);
    if (motorCurrentLimitRaw[i] <= 0) motorCurrentLimitRaw[i] = 1000; // couldn't read it - safe fallback

    dxl.torqueOff(motorID); // the operating mode can only change with torque off

    // Answer reads immediately (an older version of this sketch accidentally
    // set a ~0.5 ms reply delay). EEPROM - only written if it isn't 0 already.
    if (dxl.readControlTableItem(ControlTableItem::RETURN_DELAY_TIME, motorID) != 0) {
      dxl.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, motorID, 0);
    }
    // Fall back to plain position mode on models without current control
    // (torque limits then have no effect, everything else still works).
    if (!dxl.setOperatingMode(motorID, OP_CURRENT_BASED_POSITION)) {
      dxl.setOperatingMode(motorID, OP_POSITION);
    }
    enableMotor(i);
  }
}

void loop() {

  // Wait until at least one command byte arrives.
  if (DEBUG_SERIAL.available() <= 0) return;

  byte header = DEBUG_SERIAL.read();

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
    if (indexForId(dxlId) >= 0) {
      dxl.setGoalPosition(dxlId, angleByteToRaw(compressedAngle));
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
    if (!waitForBytes(2)) return;
    int dxlId = DEBUG_SERIAL.read();
    int percent = constrain(DEBUG_SERIAL.read(), 0, 100);
    int i = indexForId(dxlId);
    if (i >= 0) {
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
      dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, dxlId, motorPGain[i]); // RAM item - fine with torque on
    }
  }

  // READ ONE POSITION - lets Processing ask where a motor actually is.
  else if (header == READ_POSITION_HEADER) {
    if (!waitForBytes(1)) return;
    int dxlId = DEBUG_SERIAL.read();
    sendDegrees(dxl.getPresentPosition(dxlId, UNIT_RAW));
  }

  // READ ALL TORQUE USE - how hard each motor is pushing right now, as a
  // percent of its Current Limit (same scale as the torque sliders), x10.
  else if (header == READ_ALL_LOAD_HEADER) {
    if (!waitForBytes(1)) return;
    byte seq = DEBUG_SERIAL.read();
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      float raw = dxl.getPresentCurrent(DXL_IDS[i], UNIT_RAW);
      float pct = motorCurrentLimitRaw[i] > 0 ? fabs(raw) * 100.0 / motorCurrentLimitRaw[i] : 0;
      values[i] = (int16_t)constrain(pct * 10.0, 0.0, 32000.0);
    }
    sendFrame(header, seq, values);
  }

  // READ ALL POSITIONS - 11 x 2 bytes, base to tip (live angle sliders)
  else if (header == READ_ALL_HEADER) {
    if (!waitForBytes(1)) return;
    byte seq = DEBUG_SERIAL.read();
    int16_t values[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      values[i] = degreesX100(dxl.getPresentPosition(DXL_IDS[i], UNIT_RAW));
    }
    sendFrame(header, seq, values);
  }
}
