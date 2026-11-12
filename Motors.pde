// =====================================================================
//  MOTORS - per-motor ANGLE / TORQUE switch, torque limits, live angles.
//
//  Each motor row has a switch:
//    ANGLE  - the slider is the motor's angle, LIVE both ways:
//             * dragging it moves that motor right away
//             * moving the arm by hand (at low torque) moves the slider,
//               and once you let go the motor holds the new angle
//             (A shape from the chat still waits for Create Shape - live
//             reading pauses on a motor until its new angle has been sent.)
//    TORQUE - the slider sets that motor's torque limit, 0-100%, and
//             shows LIVE how much torque the motor is using right now
//             (white line + "now NN%" on the slider). Torque only ever goes
//             limp when you slide it all the way to 0.
//
//  Needs the updated lineform-ai.ino (commands 249-252). With the old
//  Arduino code the torque sliders do nothing and live reading turns
//  itself off after a few failed tries.
// =====================================================================

final int SET_ONE_GOAL_HEADER = 249;
final int READ_ALL_HEADER = 250;
final int TORQUE_LIMIT_HEADER = 251;
final int READ_ALL_LOAD_HEADER = 248;

final int LIVE_READ_INTERVAL_MS = 120;  // how often live angles are read
final int LIVE_SETTLE_MS = 800;         // after sending a motor an angle, wait this long before following it
final float LIVE_FOLLOW_DEG = 2.5;      // hand movement smaller than this is ignored
final int LIVE_DRAG_SEND_MS = 50;       // fastest rate a dragged slider is sent
final int TORQUE_SEND_MS = 80;          // fastest rate a torque slider is sent

int[] motorTorquePct = new int[MOTOR_COUNT];
float[] motorLastSentDeg = new float[MOTOR_COUNT];
long[] motorLastSendMs = new long[MOTOR_COUNT];
float[] motorPrevActual = new float[MOTOR_COUNT];
int[] motorRestCount = new int[MOTOR_COUNT];
boolean[] motorHandMoved = new boolean[MOTOR_COUNT];
// true once a motor has actually arrived at its last sent angle - only then
// is a later difference treated as "moved by hand" (so a motor that is still
// travelling, or sagging under a low torque limit, never gets its target
// overwritten). Limp motors (0% torque) are always followed.
boolean[] motorReachedGoal = new boolean[MOTOR_COUNT];
volatile boolean motorsUserDragging = false; // pauses live reading so drags stay smooth
boolean[] motorDragPending = new boolean[MOTOR_COUNT];
long[] motorDragLastSendMs = new long[MOTOR_COUNT];
boolean[] motorTorquePending = new boolean[MOTOR_COUNT];
long[] motorTorqueLastSendMs = new long[MOTOR_COUNT];

volatile float[] motorLiveReading = null;  // newest reading from the reader thread
volatile float[] motorLoadPct = null;      // live torque use per motor, % of its limit (null = not read yet)
volatile boolean motorsLiveSupported = true;
int motorsReadFailures = 0;

// Creates the TORQUE slider for one row (sits exactly where the angle
// slider is; the row's switch shows one or the other). Called from setup().
void motorsAddTorqueSlider(int dxlId, float x, float y) {
  cp5.addSlider("Torque Level " + dxlId)
    .setPosition(x, y)
    .setRange(0, 100)
    .setValue(100)
    .setSize(SLIDER_W, SLIDER_H)
    .setLabel("Torque " + hwId(dxlId)) // the real Dynamixel ID
    .setVisible(false)
    .getValueLabel()
    .setSize(11);
}

// Called at the end of setup().
void motorsSetup() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorTorquePct[i] = 100;
    motorLastSentDeg[i] = cp5.getController("Motor " + (i + 2)).getValue();
    motorPrevActual[i] = motorLastSentDeg[i];
  }
  if (serialConnected()) thread("motorsLiveReaderLoop");
}

boolean motorShowsAngle(int i) {
  if (cp5 == null) return true;
  Controller t = cp5.getController("View " + (i + 2));
  return t == null || ((Toggle) t).getState();
}

// Shows the angle or the torque slider for a row, to match its switch.
void motorsApplyView(int dxlId) {
  int i = dxlId - 2;
  boolean angle = motorShowsAngle(i);
  Controller a = cp5.getController("Motor " + dxlId);
  Controller t = cp5.getController("Torque Level " + dxlId);
  if (a != null) a.setVisible(angle);
  if (t != null) t.setVisible(!angle);
  motorHandMoved[i] = false;
  motorRestCount[i] = 0;
}

// Back to ANGLE view and 100% torque (used by Reset).
void motorsResetMotor(int dxlId) {
  int i = dxlId - 2;
  Toggle v = cp5.get(Toggle.class, "View " + dxlId);
  if (v != null) {
    v.setBroadcast(false);
    v.setState(true);
    v.setBroadcast(true);
  }
  Controller t = cp5.getController("Torque Level " + dxlId);
  if (t != null) {
    t.setBroadcast(false);
    t.setValue(100);
    t.setBroadcast(true);
  }
  motorTorquePct[i] = 100;
  sendSerialBytes(TORQUE_LIMIT_HEADER, hwId(dxlId), 100);
  motorsApplyView(dxlId);
}

// Returns true if the event belonged to a motor row.
boolean motorsHandleControlEvent(String name, float value) {
  if (name.startsWith("View ")) {
    motorsApplyView(int(name.substring(5)));
    return true;
  }
  if (name.startsWith("Torque Level ")) {
    int i = int(name.substring(13)) - 2;
    if (i < 0 || i >= MOTOR_COUNT) return true;
    motorTorquePct[i] = constrain(round(value), 0, 100);
    if (motorTorqueBeforeAnim != null) motorTorqueBeforeAnim[i] = motorTorquePct[i];
    motorTorquePending[i] = true;   // sent (throttled) by motorsTick()
    return true;
  }
  if (name.startsWith("Motor ")) {
    // Only real slider drags get here (code moves sliders with broadcast off).
    if (updatingShapeControls || aiAnimating) return true;
    int i = int(name.substring(6)) - 2;
    if (i >= 0 && i < MOTOR_COUNT) motorDragPending[i] = true;
    return true;
  }
  return false;
}

// Remember what every motor was just sent (call right after a full 255 frame).
void motorsNoteSentAll() {
  long now = millis();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorLastSentDeg[i] = cp5.getController("Motor " + (i + 2)).getValue();
    motorLastSendMs[i] = now;
    motorHandMoved[i] = false;
    motorRestCount[i] = 0;
    motorReachedGoal[i] = false;
  }
}

void motorsSendOneGoal(int i, float degrees) {
  degrees = constrain(degrees, 60, 310);
  sendSerialBytes(SET_ONE_GOAL_HEADER, hwId(i + 2), int(map(degrees, 0, 360, 0, 250)));
  motorLastSentDeg[i] = degrees;
  motorLastSendMs[i] = millis();
  motorReachedGoal[i] = false;
}

// Called every frame from draw().
void motorsTick() {
  if (!serialConnected()) return;
  long now = millis();

  boolean dragging = false;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Controller s = cp5.getController("Motor " + (i + 2));
    if (motorDragPending[i] || (s != null && s.isMousePressed())) dragging = true;
  }
  motorsUserDragging = dragging;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    // dragged angle sliders -> that motor, right away (throttled)
    if (motorDragPending[i] && now - motorDragLastSendMs[i] >= LIVE_DRAG_SEND_MS) {
      motorDragPending[i] = false;
      motorDragLastSendMs[i] = now;
      motorsSendOneGoal(i, cp5.getController("Motor " + (i + 2)).getValue());
      motorHandMoved[i] = false;
    }
    // torque sliders -> that motor's torque limit (throttled)
    if (motorTorquePending[i] && now - motorTorqueLastSendMs[i] >= TORQUE_SEND_MS) {
      motorTorquePending[i] = false;
      motorTorqueLastSendMs[i] = now;
      sendSerialBytes(TORQUE_LIMIT_HEADER, hwId(i + 2), motorTorquePct[i]);
    }
  }

  // live readings -> angle sliders
  float[] r = motorLiveReading;
  if (r == null) return;
  motorLiveReading = null;
  if (aiAnimating) return;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    float actual = r[i];
    boolean moving = abs(actual - motorPrevActual[i]) > 0.6;
    motorPrevActual[i] = actual;
    if (!motorShowsAngle(i)) continue;
    Controller s = cp5.getController("Motor " + (i + 2));
    if (s.isMousePressed() || motorDragPending[i]) continue;
    if (now - motorLastSendMs[i] < LIVE_SETTLE_MS) continue;            // still travelling to its goal
    if (!motorHandMoved[i] && abs(s.getValue() - motorLastSentDeg[i]) > 0.8) continue; // new angle not sent yet (press Create Shape)
    if (abs(actual - motorLastSentDeg[i]) <= LIVE_FOLLOW_DEG) motorReachedGoal[i] = true;
    boolean limp = motorTorquePct[i] == 0;
    if (!motorHandMoved[i] && !motorReachedGoal[i] && !limp) continue; // still on its way (or too weak to get there) - leave its target alone

    if (abs(actual - s.getValue()) > LIVE_FOLLOW_DEG) {
      updatingShapeControls = true;
      s.setBroadcast(false);
      s.setValue(actual);
      s.setBroadcast(true);
      updatingShapeControls = false;
      motorHandMoved[i] = true;
    }
    if (motorHandMoved[i]) {
      motorRestCount[i] = moving ? 0 : motorRestCount[i] + 1;
      if (motorRestCount[i] >= 3) {         // let go: hold the new angle
        motorsSendOneGoal(i, s.getValue());
        motorHandMoved[i] = false;
        motorRestCount[i] = 0;
        motorReachedGoal[i] = true;         // it is already there
      }
    }
  }
}

// Background thread: reads all 11 motor angles every LIVE_READ_INTERVAL_MS
// while at least one motor shows ANGLE. Never touches the GUI itself.
void motorsLiveReaderLoop() {
  while (true) {
    delay(LIVE_READ_INTERVAL_MS);
    if (!serialConnected() || !motorsLiveSupported || aiAnimating || motorsUserDragging) continue;
    boolean anyAngle = false, anyTorque = false;
    for (int i = 0; i < MOTOR_COUNT; i++) {
      if (motorShowsAngle(i)) anyAngle = true;
      else anyTorque = true;
    }
    if (anyTorque) {
      float[] loads = readAllMotorValues(READ_ALL_LOAD_HEADER, 10.0);
      if (loads != null) motorLoadPct = loads;
    }
    if (!anyAngle) continue;
    float[] r = readAllMotorValues(READ_ALL_HEADER, 100.0);
    if (r == null) {
      if (++motorsReadFailures >= 5) {
        motorsLiveSupported = false;
        chatAdd("status", "Live motor angles are off: the Arduino didn't answer. Upload the updated "
          + "lineform-ai.ino to the Arduino, then restart this sketch.");
      }
      continue;
    }
    motorsReadFailures = 0;
    motorLiveReading = r;
  }
}

// One round trip for all 11 motors: angles (READ_ALL_HEADER, sent x100) or
// torque use (READ_ALL_LOAD_HEADER, sent x10). The Arduino answers with a
// framed reply - 0xA5 0x5A <header> <seq> <22 data bytes> <checksum> - and
// only a frame that answers THIS request (same header + seq) and passes the
// checksum is accepted, so values can never land on the wrong motor even if
// an old reply arrives late. Data is in motor order: ID 2 first, ID 12 last.
// Returns null on timeout or if nothing valid arrived.
int motorsReadSeq = 0;

synchronized float[] readAllMotorValues(int header, float scale) {
  if (!serialConnected()) return null;
  myPort.clear();
  motorsReadSeq = (motorsReadSeq + 1) % 200; // stays below every command byte (248-255)
  int seq = motorsReadSeq;
  sendSerialBytes(header, seq);

  // Collect bytes and look for a valid frame at EVERY 0xA5 0x5A position, so
  // a cut-off or damaged frame can't swallow the good one right behind it.
  final int frameLen = 2 + 2 + 2 * MOTOR_COUNT + 1; // start x2, header, seq, data, checksum
  ArrayList<Integer> bytes = new ArrayList<Integer>();
  int scanFrom = 0;
  long start = millis();
  while (millis() - start < 300) {
    while (myPort.available() > 0) {
      int b = myPort.read();
      if (b >= 0) bytes.add(b);
    }
    for (int k = scanFrom; k + frameLen <= bytes.size(); k++) {
      if (bytes.get(k) != 0xA5 || bytes.get(k + 1) != 0x5A) continue;
      int h = bytes.get(k + 2), sq = bytes.get(k + 3);
      int sum = (h + sq) & 0xFF;
      for (int d = 0; d < 2 * MOTOR_COUNT; d++) sum = (sum + bytes.get(k + 4 + d)) & 0xFF;
      if (h == header && sq == seq && sum == bytes.get(k + frameLen - 1)) {
        float[] out = new float[MOTOR_COUNT];
        for (int i = 0; i < MOTOR_COUNT; i++) {
          int encoded = (bytes.get(k + 4 + 2 * i) << 8) | bytes.get(k + 5 + 2 * i);
          if (encoded > 32767) encoded -= 65536;
          out[i] = encoded / scale;
        }
        return out;
      }
    }
    // everything before the last frameLen bytes has been fully checked
    scanFrom = max(0, bytes.size() - frameLen + 1);
    try {
      Thread.sleep(2);
    }
    catch (InterruptedException e) {
      return null;
    }
  }
  myPort.clear();
  return null;
}

// Live torque use on each TORQUE slider: a white line at the current use
// and "now NN%" at the right end. Drawn after cp5.draw() so it sits on top.
void motorsDrawLoadOverlay() {
  float[] loads = motorLoadPct;
  if (loads == null) return;
  pushStyle();
  textFont(hudFont);
  hudTextSize(12);
  textAlign(RIGHT, CENTER);
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (motorShowsAngle(i)) continue;
    float y = ROW_START_Y + i * ROW_STEP;
    float pct = constrain(loads[i], 0, 100);
    float lx = SLIDER_X + SLIDER_W * pct / 100.0;
    stroke(255);
    strokeWeight(max(2, 2 * uiScale));
    line(uiX(lx), uiY(y), uiX(lx), uiY(y + SLIDER_H));
    noStroke();
    fill(255);
    hudText("now " + round(loads[i]) + "%", SLIDER_X + SLIDER_W - 4, y + SLIDER_H / 2.0);
  }
  popStyle();
}

// ---------------------------------------------------------------------
//  Torque set by the agents
// ---------------------------------------------------------------------

// Sets one motor's torque from code (an agent): moves its torque slider and
// sends it right away. Call on the drawing thread.
void motorsSetTorque(int i, int pct) {
  pct = constrain(pct, 0, 100);
  motorTorquePct[i] = pct;
  Controller t = cp5.getController("Torque Level " + (i + 2));
  if (t != null) {
    t.setBroadcast(false);
    t.setValue(pct);
    t.setBroadcast(true);
  }
  motorTorquePending[i] = false;
  sendSerialBytes(TORQUE_LIMIT_HEADER, hwId(i + 2), pct);
}

void motorsSetTorqueAll(int[] pct) {
  for (int i = 0; i < MOTOR_COUNT && i < pct.length; i++) motorsSetTorque(i, pct[i]);
}

// An animation's own torque plan: remembers the torque from before, so
// motorsRestoreAnimationTorque() (called when the animation stops) puts it back.
int[] motorTorqueBeforeAnim = null;

void motorsApplyAnimationTorque(int[] pct) {
  if (motorTorqueBeforeAnim == null) motorTorqueBeforeAnim = motorTorquePct.clone();
  motorsSetTorqueAll(pct);
}

void motorsRestoreAnimationTorque() {
  if (motorTorqueBeforeAnim == null) return;
  int[] before = motorTorqueBeforeAnim;
  motorTorqueBeforeAnim = null;
  motorsSetTorqueAll(before);
}

String motorsTorqueList(int[] pct) {
  StringBuilder sb = new StringBuilder();
  for (int i = 0; i < pct.length; i++) sb.append(i == 0 ? "" : ", ").append(pct[i]);
  return "[" + sb + "]";
}

// For the agents: torque limits, base to tip, or "" when all are 100%.
String motorsTorqueNote() {
  boolean all = true;
  StringBuilder sb = new StringBuilder();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (motorTorquePct[i] != 100) all = false;
    sb.append(i == 0 ? "" : ", ").append(motorTorquePct[i]);
  }
  if (all) return "";
  return "; per-joint TORQUE LIMITS set by the user, base to tip, in % of full torque: [" + sb
    + "] (0 = limp, cannot move or hold; low = weak, slow, sags under load)";
}
