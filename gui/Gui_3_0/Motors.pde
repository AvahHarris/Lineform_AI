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
final int STATUS_HEADER = 247;
final int STATUS_PWM_HEADER = 246;
final int STATUS_CUR_HEADER = 245;
final int STATUS_LAST_HEADER = 244;
final int STATUS_ERR_HEADER = 243;
final int STATUS_COUNTS_HEADER = 242;
float[] motorDiagCounts = null; // firmware uptime + commands received (see firmware 242)

final int LIVE_READ_INTERVAL_MS = 120;  // how often live angles are read
final int LIVE_SETTLE_MS = 800;         // after sending a motor an angle, wait this long before following it
// Hand-move detection - defaults suit stiff motors; override per arm in
// local_config.json ("hand_move_threshold_deg", "hand_move_confirm_readings").
float LIVE_FOLLOW_DEG = 5.0;            // hand movement smaller than this is ignored (gravity sag stays below it)
int LIVE_FOLLOW_CONFIRM = 1;            // ...and it must show in this many readings in a row (raise if motors twitch)
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
// Where each motor actually came to rest after its last goal (NaN = not
// settled yet). A motor carrying load always sits a few degrees off its goal
// (gravity vs. the position controller), so "moved by hand" is measured
// against THIS, not against the goal. Measuring against the goal made the
// sag itself look like a hand move: the GUI re-sent the sagged angle as the
// new goal, the motor sagged a bit further, and so on - the arm crept and
// twitched on its own after Reset.
float[] motorRestBaseline = new float[MOTOR_COUNT];
float[] motorPrevForRest = new float[MOTOR_COUNT];
int[] motorPushCount = new int[MOTOR_COUNT]; // readings in a row beyond LIVE_FOLLOW_DEG
volatile boolean motorsUserDragging = false; // pauses live reading so drags stay smooth
boolean[] motorDragPending = new boolean[MOTOR_COUNT];
long[] motorDragLastSendMs = new long[MOTOR_COUNT];
boolean[] motorTorquePending = new boolean[MOTOR_COUNT];
long[] motorTorqueLastSendMs = new long[MOTOR_COUNT];

volatile float[] motorLiveReading = null;  // newest reading from the reader thread
// Latest real angle of each motor (for the canvas) and when it was read.
float[] motorActualDeg = new float[MOTOR_COUNT];
// What the live display last put on each angle bar (or the value last sent).
// If the bar's value differs from this, something else set it - a dragged
// value, an AI shape, Reset... - and it's a PENDING target, not a reading.
float[] motorDisplayedDeg = new float[MOTOR_COUNT];
boolean[] motorJumpSeen = new boolean[MOTOR_COUNT];
final float MAX_READING_JUMP_DEG = 45;
long motorActualMs = 0;
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
    .setLabel("Torque " + displayId(dxlId)) // 1 = base ... 11 = tip
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
    motorRestBaseline[i] = Float.NaN;
    motorDisplayedDeg[i] = motorLastSentDeg[i];
  }
  if (serialConnected()) {
    thread("motorsLiveReaderLoop");
    thread("motorsStartupDiagnostics");
  }
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
  Toggle pw = cp5.get(Toggle.class, "Power " + dxlId);
  if (pw != null) {
    pw.setBroadcast(false);
    pw.setState(true);
    pw.setBroadcast(true);
  }
  if (rowActive(dxlId)) sendSerialBytes(TORQUE_LIMIT_HEADER, hwId(dxlId), 100); // = torque ON at 100%
  motorsApplyView(dxlId);
}

// Returns true if the event belonged to a motor row.
boolean motorsHandleControlEvent(String name, float value) {
  if (name.startsWith("View ")) {
    motorsApplyView(int(name.substring(5)));
    return true;
  }
  if (name.startsWith("Power ")) {
    int i = int(name.substring(6)) - 2;
    if (i >= 0 && i < MOTOR_COUNT) motorsApplyPower(i, value > 0.5);
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
// Is row i's bar holding a value that hasn't been sent to the motor yet?
boolean motorSliderPending(int i) {
  float v = cp5.getController("Motor " + (i + 2)).getValue();
  return abs(v - motorDisplayedDeg[i]) > 0.8;
}

// The angle row i should be COMMANDED to: a pending bar value if there is
// one, otherwise the motor's current goal. (The bar itself normally shows
// the live reading, which can sit a little off the goal - sending that back
// as a new goal would let the arm creep down with every Create Shape.)
float motorTargetDeg(int i) {
  if (motorSliderPending(i)) return cp5.getController("Motor " + (i + 2)).getValue();
  return motorLastSentDeg[i];
}

// Set a bar from code to show a value without it counting as pending.
void motorsDisplay(int i, float deg) {
  Controller s = cp5.getController("Motor " + (i + 2));
  updatingShapeControls = true;
  s.setBroadcast(false);
  s.setValue(deg);
  s.setBroadcast(true);
  updatingShapeControls = false;
  motorDisplayedDeg[i] = s.getValue();
}

// Call right after a full 255 frame was sent with these targets.
void motorsNoteSentAll(float[] targets) {
  long now = millis();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorLastSentDeg[i] = targets[i];
    motorDisplayedDeg[i] = cp5.getController("Motor " + (i + 2)).getValue(); // no longer pending
    motorLastSendMs[i] = now;
    motorHandMoved[i] = false;
    motorRestCount[i] = 0;
    motorReachedGoal[i] = false;
    motorRestBaseline[i] = Float.NaN; // re-measure where it settles
  }
}

void motorsSendOneGoal(int i, float degrees) {
  degrees = constrain(degrees, 60, 310);
  if (rowActive(i + 2)) sendSerialBytes(SET_ONE_GOAL_HEADER, hwId(i + 2), int(map(degrees, 0, 360, 0, 250)));
  motorLastSentDeg[i] = degrees;
  motorDisplayedDeg[i] = cp5.getController("Motor " + (i + 2)).getValue(); // no longer pending
  motorLastSendMs[i] = millis();
  motorReachedGoal[i] = false;
  motorRestBaseline[i] = Float.NaN; // re-measure where it settles
}

int motorsTorqueSentCount = 0; // 251 commands sent from the torque sliders (compare with the firmware's count)
long motorsDiagDueMs = 0; // >0 = run diagnostics at this time (after torque slider changes)

// Called every frame from draw().
void motorsTick() {
  if (!serialConnected()) return;
  long now = millis();
  if (motorsDiagDueMs > 0 && now >= motorsDiagDueMs) {
    motorsDiagDueMs = 0;
    thread("motorsRunDiagnostics");
  }

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
      if (rowActive(i + 2) && motorPowerOn(i)) {
        sendSerialBytes(TORQUE_LIMIT_HEADER, hwId(i + 2), motorTorquePct[i]);
        motorsTorqueSentCount++;
        motorsLastTlSentId = hwId(i + 2);
        motorsLastTlSentPct = motorTorquePct[i];
        println("Motor " + displayId(i + 2) + " torque limit -> " + motorTorquePct[i] + "% (sent)");
        motorsDiagDueMs = now + 1000; // re-check what the motors really have, once the slider stops
      } else {
        println("Motor " + displayId(i + 2) + " torque limit " + motorTorquePct[i] + "% NOT sent ("
          + (!rowActive(i + 2) ? "not fitted" : "switched OFF") + ")");
      }
    }
  }

  // live readings -> angle sliders
  float[] r = motorLiveReading;
  if (r == null) return;
  motorLiveReading = null;
  // Glitch filter: with the speed limit a motor moves at most ~25 deg between
  // two readings (~120 ms), and a hand can't do much better - so a reading
  // that jumps more than MAX_READING_JUMP_DEG from the last one is a bad read
  // and is dropped (the motor keeps its previous value). Two such readings in
  // a row are believed, so a real big move still gets through.
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (motorActualMs > 0 && abs(r[i] - motorActualDeg[i]) > MAX_READING_JUMP_DEG && !motorJumpSeen[i]) {
      motorJumpSeen[i] = true;
      r[i] = motorActualDeg[i];
    } else {
      motorJumpSeen[i] = false;
    }
    motorActualDeg[i] = r[i];
  }
  motorActualMs = now;
  if (aiAnimating) return;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (!rowActive(i + 2)) continue; // not a real motor on this arm
    float actual = r[i];
    boolean moving = abs(actual - motorPrevActual[i]) > 0.6;
    motorPrevActual[i] = actual;
    if (!motorShowsAngle(i)) continue;
    Controller s = cp5.getController("Motor " + (i + 2));
    if (s.isMousePressed() || motorDragPending[i]) continue; // you're dragging it
    if (motorSliderPending(i)) continue;   // holds a new angle waiting for Create Shape

    // 1. DISPLAY - the bar always shows the real angle, every reading.
    motorsDisplay(i, actual);

    // 2. HOLD - separately, decide whether the arm was moved by hand and,
    //    when it's let go, make that the motor's new goal. Never runs while
    //    the motor is still travelling to a goal it was just sent.
    boolean limp = motorTorquePct[i] == 0 || !motorPowerOn(i);
    if (limp) { motorHandMoved[i] = false; continue; } // no torque: nothing to hold
    if (now - motorLastSendMs[i] < LIVE_SETTLE_MS) continue;

    if (!motorHandMoved[i]) {
      // Wait for the motor to come to rest, then remember where it settled.
      if (Float.isNaN(motorRestBaseline[i])) {
        // "At rest" = 3 readings in a row within 1.5 deg of each other, or
        // just take the reading after 2.5 s (a loaded motor can jitter a
        // little forever - it must still get a baseline).
        motorRestCount[i] = abs(actual - motorPrevForRest[i]) > 1.5 ? 0 : motorRestCount[i] + 1;
        motorPrevForRest[i] = actual;
        if (motorRestCount[i] >= 3 || now - motorLastSendMs[i] > 2500) {
          motorRestBaseline[i] = actual;
          motorReachedGoal[i] = true;
          motorRestCount[i] = 0;
        }
        continue;
      }
      // Only a real, sustained push away from the resting point counts.
      if (abs(actual - motorRestBaseline[i]) <= LIVE_FOLLOW_DEG) { motorPushCount[i] = 0; continue; }
      if (++motorPushCount[i] < LIVE_FOLLOW_CONFIRM) continue;
      motorPushCount[i] = 0;
      motorHandMoved[i] = true;
      motorRestCount[i] = 0;
    }

    // Being moved by hand: wait until it's held still, then hold it there.
    motorRestCount[i] = moving ? 0 : motorRestCount[i] + 1;
    if (motorRestCount[i] >= 3) {
      // The goal is simply where you left it (the firmware's I term removes
      // gravity droop, so it stays put).
      motorsSendOneGoal(i, constrain(actual, 60, 310));
      motorHandMoved[i] = false;
      motorRestCount[i] = 0;
      motorReachedGoal[i] = true;
    }
  }
}

// Background thread: reads all 11 motor angles every LIVE_READ_INTERVAL_MS
// while at least one motor shows ANGLE. Never touches the GUI itself.
void motorsLiveReaderLoop() {
  while (true) {
    delay(LIVE_READ_INTERVAL_MS);
    if (!serialConnected() || aiAnimating || motorsUserDragging) continue;
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
      // Missed replies happen while the Arduino is busy (e.g. Reset turning
      // 10 motors on). Never give up for good - after 5 misses in a row,
      // pause live angles, say so once, and keep retrying every 2 s.
      if (++motorsReadFailures == 5) {
        motorsLiveSupported = false;
        println("Live motor angles paused: the Arduino isn't answering READ_ALL (250). Retrying...");
      }
      if (motorsReadFailures >= 5) delay(2000);
      continue;
    }
    if (!motorsLiveSupported) {
      println("Live motor angles working again.");
    }
    motorsLiveSupported = true;
    motorsReadFailures = 0;
    motorLiveReading = r;
  }
}

// One round trip for all 11 motors: angles (READ_ALL_HEADER, sent x100) or
// torque use (READ_ALL_LOAD_HEADER, sent x10). The Arduino answers with a
// framed reply - 0xA5 0x5A <header> <seq> <22 data bytes> <checksum> - and
// only a frame that answers THIS request (same header + seq) and passes the
// checksum is accepted, so values can never land on the wrong motor even if
// an old reply arrives late. Data is in chain order, base to tip: ID 1 first, ID 11 last.
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
    // What the MOTOR reports it was set to (read back ~1 s after a change),
    // under the bar - so you can see whether the setting really arrived.
    if (motorDiagTorqueOn != null && motorDiagPwmPct != null) {
      textAlign(LEFT, TOP);
      fill(THEME_ORANGE);
      String rb = (!motorDiagTorqueOn[i] ? "motor: OFF" : ("motor: " + round(motorDiagPwmPct[i]) + "%"))
        + "  | " + motorsDiagLastText(i)
        + (motorDiagCounts == null ? "" : "  | fw up " + round(motorDiagCounts[0]) + "s, got 251 x" + round(motorDiagCounts[3])
           + " (GUI sent " + motorsTorqueSentCount + "), last: " + motorsLastTlText());
      hudTextSize(10);
      hudText(rb, SLIDER_X + 2, y + SLIDER_H + 1);
      hudTextSize(12);
      textAlign(RIGHT, CENTER);
    }
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
  if (rowActive(i + 2) && motorPowerOn(i)) sendSerialBytes(TORQUE_LIMIT_HEADER, hwId(i + 2), pct);
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

// Angle the CANVAS should draw for row i. Shows the REAL arm (latest live
// reading) whenever the slider isn't holding a newer, not-yet-sent value -
// so twisting a motor by hand, sag, or a motor that didn't reach its goal
// are all visible straight away. Falls back to the slider while dragging,
// during animations, with an unsent change (press Create Shape), when
// offline, for inactive rows, or if readings are stale.
float motorCanvasAngle(int i) {
  // The bars now show the live angle themselves (or a pending target), so
  // the canvas simply draws what the bars say.
  return cp5.get(Slider.class, "Motor " + (i + 2)).getValue();
}

// ---- Torque ON/OFF switch (one per row, like the backup GUI) ----
boolean motorPowerOn(int i) {
  if (cp5 == null) return true;
  Controller c = cp5.getController("Power " + (i + 2));
  return c == null || ((Toggle) c).getState();
}

// Sets the switch from code (no event) and applies it.
void motorsSetPowerSwitch(int i, boolean on) {
  Toggle pw = cp5.get(Toggle.class, "Power " + (i + 2));
  if (pw != null) {
    pw.setBroadcast(false);
    pw.setState(on);
    pw.setBroadcast(true);
  }
  motorsApplyPower(i, on);
}

// OFF: torque off (limp - pose it by hand; the slider follows).
// ON: first hold where the arm is NOW (so it doesn't snap back to an old
// goal after being moved by hand), then torque on at this row's torque %.
// Uses TORQUE_LIMIT rather than a bare torque-on, so the row's torque
// slider value is applied too (0% keeps it limp).
void motorsApplyPower(int i, boolean on) {
  if (frameCount < 1) return; // ControlP5 fires an event while setup() builds the switch - ignore it
  int rowId = i + 2;
  motorHandMoved[i] = false;
  motorRestCount[i] = 0;
  motorTorquePending[i] = false;
  if (!rowActive(rowId)) return;
  if (on) {
    float here = cp5.getController("Motor " + rowId).getValue();
    if (serialConnected() && millis() - motorActualMs < 600) here = motorActualDeg[i];
    else if (serialConnected()) here = readMotorPosition(rowId, here);
    Controller s = cp5.getController("Motor " + rowId);
    updatingShapeControls = true;
    s.setBroadcast(false);
    s.setValue(here);
    s.setBroadcast(true);
    updatingShapeControls = false;
    motorsSendOneGoal(i, here);
    sendSerialBytes(TORQUE_LIMIT_HEADER, hwId(rowId), motorTorquePct[i]);
  } else {
    sendSerialBytes(254, hwId(rowId), 0);
  }
  println("Motor " + displayId(rowId) + " torque " + (on ? "ON" : "OFF"));
}

// ---- Diagnostics: what each motor is REALLY set to ----
// Asks the Arduino (STATUS 247/246/245) and prints a table to the console
// plus a short summary in the chat. Runs once at startup and on every Reset
// (in a background thread). Shows at a glance whether:
//  - the Arduino runs the current firmware at all (no answer = old firmware)
//  - each motor is in current-based position mode (5) or plain position (3)
//  - torque is on, and any hardware error (e.g. 32 = overload)
//  - the torque limit actually reached the motor (goal PWM / current %)
void motorsStartupDiagnostics() {
  delay(2500); // let the Arduino finish booting / setting up the motors
  motorsRunDiagnostics();
}

// Latest read-back per motor (shown on the TORQUE bars): -1 = unknown.
float[] motorDiagPwmPct = null;   // goal PWM as % of PWM limit
float[] motorDiagLast = null;     // firmware's record: last % received*100 + pwmOk*10 + curOk
float[] motorDiagErr = null;      // lib error codes: pwm*100 + cur
boolean[] motorDiagTorqueOn = null;

void motorsRunDiagnostics() {
  if (!serialConnected()) return;
  float[] st = readAllMotorValues(STATUS_HEADER, 1.0);
  if (st == null) {
    println("DIAGNOSTICS: no answer to STATUS (247) - the Arduino is running OLD firmware. "
      + "Upload firmware/lineform-ai/lineform-ai.ino.");
    return;
  }
  float[] pwm = readAllMotorValues(STATUS_PWM_HEADER, 10.0);
  float[] cur = readAllMotorValues(STATUS_CUR_HEADER, 10.0);
  boolean[] torqueOnNow = new boolean[MOTOR_COUNT];
  for (int i = 0; i < MOTOR_COUNT; i++) torqueOnNow[i] = st[i] >= 0 && (round(st[i]) / 100) % 10 == 1;
  motorDiagTorqueOn = torqueOnNow;
  motorDiagPwmPct = pwm;
  motorDiagLast = readAllMotorValues(STATUS_LAST_HEADER, 1.0);
  motorDiagErr = readAllMotorValues(STATUS_ERR_HEADER, 1.0);
  motorDiagCounts = readAllMotorValues(STATUS_COUNTS_HEADER, 1.0);
  if (motorDiagCounts != null) {
    float[] c = motorDiagCounts;
    println("FIRMWARE: up " + round(c[0]) + " s | received: 255 frame x" + round(c[1]) + ", 254 on/off x" + round(c[2])
      + ", 251 torque x" + round(c[3]) + ", 249 goal x" + round(c[4]) + ", 250 read x" + round(c[5])
      + ", 252 stiff x" + round(c[6]) + ", 253 read1 x" + round(c[7]) + " | timeouts " + round(c[8])
      + " | unknown bytes " + round(c[9]) + " | GUI sent 251 x" + motorsTorqueSentCount
      + " | last 251: " + motorsLastTlText());
  }
  println("DIAGNOSTICS (firmware answered):");
  println("  motor | mode            | torque | error | goal PWM % | goal current % | GUI torque % | firmware's last torque write");
  int plainMode = 0, errors = 0, off = 0, active = 0;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int v = round(st[i]);
    String n = nf(displayId(i + 2), 2);
    if (v < 0) { println("  " + n + "    | not fitted"); continue; }
    active++;
    int mode = v / 1000, on = (v / 100) % 10, err = v % 100;
    String modeName = mode == 5 ? "current-based(5)" : mode == 3 ? "position(3)     " : ("other(" + mode + ")        ");
    if (mode != 5) plainMode++;
    if (err != 0) errors++;
    if (on == 0) off++;
    println("  " + n + "    | " + modeName + "| " + (on == 1 ? "ON " : "OFF") + "    | " + nf(err, 2) + "    | "
      + (pwm == null ? "?" : nf(pwm[i], 1, 1)) + "       | " + (cur == null ? "?" : nf(cur[i], 1, 1))
      + "           | " + motorTorquePct[i] + "   | " + motorsDiagLastText(i));
  }
  println("  -> " + active + " motors answered; " + plainMode + " not in current-based mode; "
    + errors + " with a hardware error; " + off + " with torque OFF.");
}

// "fw got 13%, PWM ok, cur ok" / "fw got nothing" / "...PWM FAILED e3"
String motorsDiagLastText(int i) {
  if (motorDiagLast == null) return "fw: ?";
  int v = round(motorDiagLast[i]);
  if (v < 0) return "fw got nothing";
  int pct = v / 100;
  boolean pwmOk = (v / 10) % 10 == 1, curOk = v % 10 == 1;
  int ePwm = -1, eCur = -1;
  if (motorDiagErr != null) { int e = round(motorDiagErr[i]); ePwm = e / 100; eCur = e % 100; }
  return "fw got " + pct + "%, PWM " + (pwmOk ? "ok" : "FAILED e" + ePwm) + ", cur " + (curOk ? "ok" : "FAILED e" + eCur);
}

// Firmware's verdict on the last TORQUE_LIMIT (242 slot 10) + what the GUI sent.
int motorsLastTlSentId = -1, motorsLastTlSentPct = -1;
String motorsLastTlText() {
  String gui = "GUI sent id " + motorsLastTlSentId + " " + motorsLastTlSentPct + "%";
  if (motorDiagCounts == null) return gui;
  int v = round(motorDiagCounts[10]);
  if (v < 0) return "fw saw none; " + gui;
  int result = v / 1000, id = v % 1000;
  String r = result == 0 ? "applied" : result == 1 ? "TIMED OUT waiting for data" : "UNKNOWN ID";
  return "fw id " + id + " -> " + r + "; " + gui;
}
