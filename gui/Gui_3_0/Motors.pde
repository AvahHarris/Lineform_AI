// =====================================================================
//  MOTORS - per-motor ANGLE / TORQUE switch, torque limits, live angles.
//
//  Each motor row has a switch:
//    ANGLE  - the slider is the motor's TARGET angle:
//             * dragging it moves that motor right away
//             * a powered motor always goes back to / holds its bar angle
//               (since 2026-11-18; pose by hand in Manipulation mode, or
//               switch the row OFF - limp rows' bars follow the real angle).
//               AUTOMATION_FOLLOW_HAND_MOVES = true restores the old "hand push
//               becomes the new goal" behaviour.
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
float LIVE_FOLLOW_DEG = 2.0;            // hand movement smaller than this is ignored (was 5 for the soft arm; stiff motors barely sag)
int LIVE_FOLLOW_CONFIRM = 1;            // ...and it must show in this many readings in a row (raise if motors twitch)
// HAND MOVES IN AUTOMATION MODE. true (default again since 2026-11-18, with lenient
// thresholds for the stiffer arm: local_config.json "hand_move_threshold_deg"
// 2, "hand_move_confirm_readings" 1): a hand push becomes the motor's new goal
// once it is let go. false ("follow_hand_moves": false) = BARS ARE THE TARGETS:
// a powered motor ALWAYS goes back to / holds the angle on its bar - pushing it
// by hand never changes its goal, and gravity sag never becomes a new goal.
// The bar shows the target; the 3D view still draws the REAL live angles.
// (Pose by hand in Manipulation mode, or switch a row OFF / to 0% - limp rows'
// bars follow the real angle.) true = the old soft-motor behaviour: a hand
// push past LIVE_FOLLOW_DEG becomes the new goal ("follow_hand_moves" in
// local_config.json).
boolean AUTOMATION_FOLLOW_HAND_MOVES = true;
boolean motorsBarsSynced = false; // first live reading adopted as the targets (see motorsTick)
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
// The TRUE value behind each bar, unclamped (internal degrees). The bar
// itself can only show JOINT_MIN..MAX, so a joint pushed past a limit by hand
// would otherwise read as exactly 0 / 180 - the readout and the 3D view use
// this instead, so angle detection is never distorted by the bar's range.
float[] motorShownDeg = new float[MOTOR_COUNT];
boolean[] motorJumpSeen = new boolean[MOTOR_COUNT];
final float MAX_READING_JUMP_DEG = 45;
long motorActualMs = 0;
volatile float[] motorLoadPct = null;      // live torque use per motor, % of its limit (null = not read yet)
volatile int motorLoadSeq = 0;             // +1 per new load reading (overload check counts each reading once)
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
    motorShownDeg[i] = motorLastSentDeg[i];
    // ControlP5's own value label would print the INTERNAL angle (e.g. 180);
    // motorsDrawAngleReadouts() draws the displayed 0-180 value instead.
    cp5.getController("Motor " + (i + 2)).getValueLabel().setVisible(false);
  }
  if (serialConnected()) {
    thread("motorsLiveReaderLoop");
    thread("motorsStartupDiagnostics");
  }
}

// ---- Per-row view: which bar the row shows (3-way selector) ----
final int VIEW_ANGLE = 0, VIEW_TORQUE = 1, VIEW_STIFF = 2;
int[] motorView = new int[MOTOR_COUNT]; // all start on ANGLE

boolean motorShowsAngle(int i) {
  if (manipulationMode) return true; // manipulation: every row shows its angle
  return motorView[i] == VIEW_ANGLE;
}
boolean motorShowsTorque(int i) { return !manipulationMode && motorView[i] == VIEW_TORQUE; }
boolean motorShowsStiff(int i)  { return !manipulationMode && motorView[i] == VIEW_STIFF; }

// Shows the angle, torque or stiffness bar for a row, to match its selector.
void motorsApplyView(int dxlId) {
  int i = dxlId - 2;
  Controller a = cp5.getController("Motor " + dxlId);
  Controller t = cp5.getController("Torque Level " + dxlId);
  Controller st = cp5.getController("Stiffness " + dxlId);
  if (a != null) a.setVisible(motorShowsAngle(i));
  if (t != null) t.setVisible(motorShowsTorque(i));
  if (st != null) st.setVisible(motorShowsStiff(i));
  motorHandMoved[i] = false;
  motorRestCount[i] = 0;
}

// ---- STIFFNESS bar ----
// 4 steps; the bar is FULL at full stiffness: value 3 = level 0 (normal
// holding) ... value 0 = level 3 (very soft). Level -> Position P gain is set
// per arm in firmware local_config.h (STIFFNESS_P_GAINS_LIST). Replaces the
// old clickable hinge markers in the 3D view.
final String[] STIFF_NAMES = {"full", "medium", "soft", "very soft"};

void motorsAddStiffnessSlider(int dxlId, float x, float y) {
  Slider s = cp5.addSlider("Stiffness " + dxlId)
    .setPosition(x, y)
    .setRange(0, NUM_STIFFNESS_LEVELS - 1)
    .setValue(NUM_STIFFNESS_LEVELS - 1)
    .setNumberOfTickMarks(NUM_STIFFNESS_LEVELS)
    .snapToTickMarks(true)
    .showTickMarks(false)
    .setSize(SLIDER_W, SLIDER_H)
    .setLabel("Stiff " + displayId(dxlId))
    .setVisible(false);
  s.getValueLabel().setVisible(false); // motorsDrawStiffOverlay() writes the level name
}

// Keeps each stiffness bar showing its joint's real level, whatever changed
// it (animations, Reset, the twitch safety net). Called every frame.
void motorsSyncStiffnessSliders() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Controller s = cp5.getController("Stiffness " + (i + 2));
    if (s == null || s.isMousePressed()) continue;
    float want = NUM_STIFFNESS_LEVELS - 1 - hingeStiffnessLevel[i];
    if (abs(s.getValue() - want) > 0.01) {
      s.setBroadcast(false);
      s.setValue(want);
      s.setBroadcast(true);
    }
  }
}

void motorsDrawStiffOverlay() {
  pushStyle();
  textFont(hudFont);
  hudTextSize(13);
  textAlign(LEFT, CENTER);
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (!motorShowsStiff(i)) continue;
    float y = ROW_START_Y + i * ROW_STEP + SLIDER_H / 2.0;
    int level = hingeStiffnessLevel[i];
    String t = STIFF_NAMES[constrain(level, 0, STIFF_NAMES.length - 1)] + "  (L" + level + ")";
    fill(0, 120);
    hudText(t, SLIDER_X + 5, y + 1);
    fill(255);
    hudText(t, SLIDER_X + 4, y);
  }
  popStyle();
}

// ---- 3-way selector: ANGLE | TORQUE | STIFF (drawn here, not ControlP5) ----
final float VIEW_SEL_W = 130;
final float[] VIEW_SEG_W = {40, 50, 40};
final String[] VIEW_NAMES = {"ANGLE", "TORQUE", "STIFF"};

color motorsViewColor(int v) {
  return v == VIEW_ANGLE ? THEME_ACCENT : (v == VIEW_TORQUE ? THEME_ORANGE : THEME_PURPLE);
}

void motorsDrawViewSelectors() {
  pushStyle();
  textFont(uiBoldFont(max(7, round(10 * uiScale))));
  textAlign(CENTER, CENTER);
  for (int i = 0; i < MOTOR_COUNT; i++) {
    float y = ROW_START_Y + i * ROW_STEP;
    float x = TOGGLE_X;
    int hover = motorsViewSegmentAt(mouseX, mouseY, i);
    for (int v = 0; v < 3; v++) {
      float w = VIEW_SEG_W[v];
      color c = motorsViewColor(v);
      boolean on = motorView[i] == v;
      float rl = v == 0 ? 6 * uiScale : 0, rr = v == 2 ? 6 * uiScale : 0;
      noStroke();
      fill(on ? c : themeShade(c, hover == v ? -0.45 : -0.68));
      rect(uiX(x), uiY(y), w * uiScale, TOGGLE_H * uiScale, rl, rr, rr, rl);
      fill(on ? color(15, 20, 35) : themeShade(c, -0.15));
      hudText(VIEW_NAMES[v], x + w / 2.0, y + TOGGLE_H / 2.0);
      x += w;
    }
  }
  popStyle();
}

// Which segment (0-2) of row i's selector is at pixel (mx, my), or -1.
int motorsViewSegmentAt(float mx, float my, int i) {
  float y = ROW_START_Y + i * ROW_STEP;
  if (my < uiY(y) || my > uiY(y + TOGGLE_H)) return -1;
  float x = TOGGLE_X;
  for (int v = 0; v < 3; v++) {
    if (mx >= uiX(x) && mx < uiX(x + VIEW_SEG_W[v])) return v;
    x += VIEW_SEG_W[v];
  }
  return -1;
}

boolean motorsViewMouseOver() {
  if (manipulationMode) return false;
  for (int i = 0; i < MOTOR_COUNT; i++) if (motorsViewSegmentAt(mouseX, mouseY, i) >= 0) return true;
  return false;
}

// Called from mousePressed(). Returns true if a selector was clicked.
boolean motorsViewMousePressed() {
  if (manipulationMode) return false;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int v = motorsViewSegmentAt(mouseX, mouseY, i);
    if (v < 0) continue;
    if (motorView[i] != v) {
      motorView[i] = v;
      motorsApplyView(i + 2);
    }
    return true;
  }
  return false;
}

// Back to ANGLE view and 100% torque (used by Reset).
void motorsResetMotor(int dxlId) {
  int i = dxlId - 2;
  motorView[i] = VIEW_ANGLE;
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
  // Manipulation mode: these controls are hidden/locked - swallow any stray event.
  if (manipulationMode && (name.startsWith("Stiffness ") || name.startsWith("Power ")
      || name.startsWith("Torque Level ") || name.startsWith("Motor "))) return true;
  if (name.startsWith("Stiffness ")) {
    int i = int(name.substring(10)) - 2;
    if (i < 0 || i >= MOTOR_COUNT) return true;
    int level = constrain(NUM_STIFFNESS_LEVELS - 1 - round(value), 0, NUM_STIFFNESS_LEVELS - 1);
    if (level != hingeStiffnessLevel[i]) {
      hingeStiffnessLevel[i] = level;
      sendHingeStiffness(i + 2, level);
      println("Motor " + displayId(i + 2) + " stiffness -> " + STIFF_NAMES[level] + " (level " + level
        + "; P gain per level is set in firmware local_config.h)");
    }
    return true;
  }
  if (name.startsWith("Power ")) {
    int i = int(name.substring(6)) - 2;
    if (i < 0 || i >= MOTOR_COUNT) return true;
    boolean on = value > 0.5;
    if (on && motorTorquePct[i] == 0) motorsSetTorqueBar(i, motorLastTorquePct[i]); // ON needs torque: back to its last value
    motorsApplyPower(i, on);
    return true;
  }
  if (name.startsWith("Torque Level ")) {
    int i = int(name.substring(13)) - 2;
    if (i < 0 || i >= MOTOR_COUNT) return true;
    motorTorquePct[i] = constrain(round(value), 0, 100);
    if (motorTorqueBeforeAnim != null) motorTorqueBeforeAnim[i] = motorTorquePct[i];
    // Torque and the ON/OFF switch are linked (2026-11-25): 0% = OFF, above 0 = ON.
    if (motorTorquePct[i] == 0 && motorPowerOn(i)) { motorsSetPowerSwitch(i, false); return true; }
    if (motorTorquePct[i] > 0 && !motorPowerOn(i)) { motorsSetPowerSwitch(i, true); return true; } // ON at this torque
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
  motorDisplayedDeg[i] = s.getValue(); // what the bar could show (clamped to its range)
  motorShownDeg[i] = deg;              // the true value, unclamped
}

// Call right after a full 255 frame was sent with these targets.
void motorsNoteSentAll(float[] targets) {
  motorsBarsSynced = true; // the bars ARE the targets now
  long now = millis();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorLastSentDeg[i] = targets[i];
    motorDisplayedDeg[i] = cp5.getController("Motor " + (i + 2)).getValue(); // no longer pending
    motorShownDeg[i] = motorDisplayedDeg[i]; // until the next reading arrives
    motorLastSendMs[i] = now;
    motorHandMoved[i] = false;
    motorRestCount[i] = 0;
    motorReachedGoal[i] = false;
    motorRestBaseline[i] = Float.NaN; // re-measure where it settles
  }
}

void motorsSendOneGoal(int i, float degrees) {
  degrees = clampJoint(degrees);
  if (rowActive(i + 2)) sendSerialBytes(SET_ONE_GOAL_HEADER, hwId(i + 2), round(map(degrees, 0, 360, 0, 250)));
  motorLastSentDeg[i] = degrees;
  motorDisplayedDeg[i] = cp5.getController("Motor " + (i + 2)).getValue(); // no longer pending
  motorShownDeg[i] = motorDisplayedDeg[i]; // until the next reading arrives
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

  // Overload protection: any motor at >= 95% torque -> Manipulation mode (Modes.pde).
  modesCheckOverload();

  // Manipulation mode: motors limp, bars just follow the real angles (Modes.pde).
  if (manipulationMode) {
    modesManipulationTick(now);
    return;
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

  if (!AUTOMATION_FOLLOW_HAND_MOVES) {
    // First reading after startup: the arm is wherever it was left, the bars
    // say 90. Take the real pose as every bar's target ONCE (and send it, so
    // the motors' goals and the bars agree) - nothing jumps on startup.
    if (!motorsBarsSynced) {
      motorsBarsSynced = true;
      for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!rowActive(i + 2)) continue;
        motorsDisplay(i, r[i]);
        motorsSendOneGoal(i, clampJoint(r[i]));
      }
      println("Startup: bars set to the arm's real pose; from now on the motors hold the bar angles.");
      return;
    }
    // Bars = targets. Powered motors just hold their bar's angle (the firmware
    // already has it as their goal), so nothing is sent and the bar is left
    // alone. Only LIMP rows (OFF / 0%) show the real angle on their bar.
    for (int i = 0; i < MOTOR_COUNT; i++) {
      if (!rowActive(i + 2)) continue;
      Controller s = cp5.getController("Motor " + (i + 2));
      if (s.isMousePressed() || motorDragPending[i] || motorSliderPending(i)) continue;
      boolean limp = motorTorquePct[i] == 0 || !motorPowerOn(i);
      if (limp) motorsDisplay(i, r[i]);
    }
    return;
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (!rowActive(i + 2)) continue; // not a real motor on this arm
    motorsTwitchCheck(i, r[i], now);  // oscillation safety net (below)
    float actual = r[i];
    boolean moving = abs(actual - motorPrevActual[i]) > 0.6;
    motorPrevActual[i] = actual;
    // Rows in TORQUE view are handled too: their (hidden) angle bar still
    // tracks the real angle - the 3D view draws it - and hand-move HOLD must
    // work whichever bar the row is showing. (Skipping them here is why a
    // motor in TORQUE view never held a new hand-posed angle.)
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
      motorsSendOneGoal(i, clampJoint(actual)); // past a limit by hand -> holds AT the limit
      motorHandMoved[i] = false;
      motorRestCount[i] = 0;
      motorReachedGoal[i] = true;
    }
  }
}

// Background thread: reads all 11 motor angles every LIVE_READ_INTERVAL_MS
// (plus torque use while any row shows TORQUE). Never touches the GUI itself.
void motorsLiveReaderLoop() {
  while (true) {
    delay(manipulationMode ? MANIP_READ_INTERVAL_MS : LIVE_READ_INTERVAL_MS);
    if (!serialConnected()) continue;
    // Torque use is read EVERY cycle in Automation mode - whatever bar each row shows,
    // and also during animations and slider drags - because the overload
    // protection (Modes.pde, modesCheckOverload) needs it. In manipulation
    // mode the motors are limp, so there is nothing to protect.
    if (!manipulationMode) {
      float[] loads = readAllMotorValues(READ_ALL_LOAD_HEADER, 10.0);
      if (loads != null) {
        motorLoadPct = loads;
        motorLoadSeq++;
      }
    }
    if (aiAnimating || motorsUserDragging) continue;
    // Angles are ALWAYS read (even if every row shows TORQUE): hand-move
    // hold and the 3D view need them.
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
  return readAllMotorValues(header, scale, 300);
}

synchronized float[] readAllMotorValues(int header, float scale, int timeoutMs) {
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
  while (millis() - start < timeoutMs) {
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

// Live torque use (the motor's output / force): a white line at the current
// use, 0-100% of its Current Limit across the bar, and "now NN%" at the
// right end. Since 2026-11-18 drawn on EVERY row's bar - ANGLE, TORQUE and
// STIFF view alike - not only on TORQUE bars. (Loads are read every cycle in
// Automation mode; in Manipulation mode the motors are limp, so nothing is drawn.)
// Drawn after cp5.draw() so it sits on top.
void motorsDrawLoadOverlay() {
  float[] loads = motorLoadPct;
  boolean live = loads != null && !manipulationMode && serialConnected();
  pushStyle();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (!rowActive(i + 2)) continue;
    float y = ROW_START_Y + i * ROW_STEP;

    // ---- the white marker ON the bar ----
    if (live) {
      float pct = constrain(loads[i], 0, 100);
      float lx = SLIDER_X + SLIDER_W * pct / 100.0;
      // Thick white marker (2026-11-19: was 2 px) with a dark outline, sticking
      // out a little above and below the bar, plus small caps top and bottom.
      float over = 2; // + 3 px caps: stays clear of the next row (rows are 12 px apart)
      strokeCap(SQUARE);
      stroke(0, 190);
      strokeWeight(max(7, 7 * uiScale));
      line(uiX(lx), uiY(y - over), uiX(lx), uiY(y + SLIDER_H + over));
      stroke(255);
      strokeWeight(max(4, 4 * uiScale));
      line(uiX(lx), uiY(y - over), uiX(lx), uiY(y + SLIDER_H + over));
      noStroke();
      fill(255);
      float cw = 4;
      triangle(uiX(lx - cw), uiY(y - over - 3), uiX(lx + cw), uiY(y - over - 3), uiX(lx), uiY(y - over + 1));
      triangle(uiX(lx - cw), uiY(y + SLIDER_H + over + 3), uiX(lx + cw), uiY(y + SLIDER_H + over + 3), uiX(lx), uiY(y + SLIDER_H + over - 1));
    }

    // ---- ALWAYS: small load text UNDER the bar (2026-11-19; replaces the
    // "now NN%" that used to sit on the bar's right end) ----
    String t;
    color tc = THEME_TEXT;
    if (manipulationMode) { t = "load: -  (torque OFF)"; tc = THEME_MUTED; }
    else if (!serialConnected()) { t = "load: -  (offline)"; tc = THEME_MUTED; }
    else if (loads == null) { t = "load: -"; tc = THEME_MUTED; }
    else {
      float v = loads[i];
      boolean limp = !motorPowerOn(i) || motorTorquePct[i] == 0;
      t = "load " + round(v) + "% of full torque" + (limp ? "  (OFF)" : "");
      if (limp) tc = THEME_MUTED;
      else if (OVERLOAD_TRIP_PCT > 0 && v >= OVERLOAD_TRIP_PCT) tc = THEME_RED;
      else if (v >= 75) tc = THEME_YELLOW;
    }
    textFont(uiBoldFont(max(7, round(10 * uiScale))));
    textAlign(LEFT, TOP);
    fill(tc);
    hudText(t, SLIDER_X + 2, y + SLIDER_H + 1);
    float textEnd = SLIDER_X + 2 + textWidth(t) / uiScale;

    if (!motorShowsTorque(i)) continue;  // the read-back text below is for TORQUE bars only
    // What the MOTOR reports it was set to (read back ~1 s after a change),
    // after the load text - so you can see whether the setting really arrived.
    if (motorDiagTorqueOn != null && motorDiagPwmPct != null) {
      textFont(hudFont);
      hudTextSize(10);
      fill(THEME_ORANGE);
      String rb = "  | " + (!motorDiagTorqueOn[i] ? "motor: OFF" : ("motor: " + round(motorDiagPwmPct[i]) + "%"))
        + "  | " + motorsDiagLastText(i)
        + (motorDiagCounts == null ? "" : "  | fw up " + round(motorDiagCounts[0]) + "s, got 251 x" + round(motorDiagCounts[3])
           + " (GUI sent " + motorsTorqueSentCount + "), last: " + motorsLastTlText());
      hudText(rb, textEnd, y + SLIDER_H + 1);
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
  motorsSetTorqueBar(i, pct);
  motorTorquePending[i] = false;
  // Linked to the ON/OFF switch (2026-11-25): 0% = OFF, above 0 = ON.
  if (pct == 0 && motorPowerOn(i)) { motorsSetPowerSwitch(i, false); return; }
  if (pct > 0 && !motorPowerOn(i)) { motorsSetPowerSwitch(i, true); return; } // ON at this torque
  if (rowActive(i + 2) && motorPowerOn(i)) sendSerialBytes(TORQUE_LIMIT_HEADER, hwId(i + 2), pct);
}

// Moves a row's torque bar (no event) and remembers it; nothing is sent.
int[] motorLastTorquePct = null; // last torque above 0 per row (for switching back ON)
void motorsSetTorqueBar(int i, int pct) {
  if (motorLastTorquePct == null) { motorLastTorquePct = new int[MOTOR_COUNT]; java.util.Arrays.fill(motorLastTorquePct, 100); }
  pct = constrain(pct, 0, 100);
  motorTorquePct[i] = pct;
  if (pct > 0) motorLastTorquePct[i] = pct;
  Controller t = cp5.getController("Torque Level " + (i + 2));
  if (t != null) {
    t.setBroadcast(false);
    t.setValue(pct);
    t.setBroadcast(true);
  }
}

// ---- ON / OFF colors (2026-11-25): green = ON, red = OFF, on the switch,
// its ON/OFF text and the row's caption (Motor N / Torque N / Stiff N).
// Grey = not fitted; red for every row in Manipulation mode (torque off).
final color POWER_ON_COLOR = color(46, 204, 113);
final color POWER_OFF_COLOR = color(235, 70, 70);

color motorsRowStateColor(int i) {
  if (!rowActive(i + 2)) return THEME_MUTED;
  if (manipulationMode) return POWER_OFF_COLOR;
  return motorPowerOn(i) ? POWER_ON_COLOR : POWER_OFF_COLOR;
}

// Called every frame (draw).
void motorsSyncRowColors() {
  if (motorLastTorquePct == null) { motorLastTorquePct = new int[MOTOR_COUNT]; java.util.Arrays.fill(motorLastTorquePct, 100); }
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int id = i + 2;
    if (motorTorquePct[i] > 0) motorLastTorquePct[i] = motorTorquePct[i];
    // Safety net for the torque <-> switch link: a row at 0% is OFF, whatever set it.
    if (motorTorquePct[i] == 0 && motorPowerOn(i) && !manipulationMode && frameCount > 1) {
      println("Motor " + displayId(id) + ": torque 0% but switch ON - switching it OFF to match.");
      motorsSetPowerSwitch(i, false);
    }
    color c = motorsRowStateColor(i);
    for (String n : new String[] {"Motor ", "Torque Level ", "Stiffness "}) {
      Controller ctl = cp5.getController(n + id);
      if (ctl != null) ctl.setColorCaptionLabel(c);
    }
    Controller pw = cp5.getController("Power " + id);
    if (pw != null) {
      boolean on = motorPowerOn(i);
      color sc = on ? POWER_ON_COLOR : POWER_OFF_COLOR;
      pw.setColorBackground(themeShade(sc, -0.6)); // track
      pw.setColorForeground(sc);                  // knob (ControlP5's SWITCH draws it with active/foreground)
      pw.setColorActive(sc);
    }
  }
}

void motorsSetTorqueAll(int[] pct) {
  for (int i = 0; i < MOTOR_COUNT && i < pct.length; i++) motorsSetTorque(i, pct[i]);
}

// An animation's own torque plan: remembers the torque from before, so
// motorsRestoreAnimationTorque() (called when the animation stops) puts it back.
int[] motorTorqueBeforeAnim = null;
boolean[] motorPowerBeforeAnim = null; // ON/OFF before the animation (an OFF row keeps its bar value)

void motorsApplyAnimationTorque(int[] pct) {
  if (motorTorqueBeforeAnim == null) {
    motorTorqueBeforeAnim = motorTorquePct.clone();
    motorPowerBeforeAnim = new boolean[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) motorPowerBeforeAnim[i] = motorPowerOn(i);
  }
  motorsSetTorqueAll(pct);
}

// Stop: back to the torque AND the ON/OFF state from before the animation.
// (Fix 2026-11-25: a row switched OFF by hand keeps its bar value, so
// restoring the bars alone switched it back ON.)
void motorsRestoreAnimationTorque() {
  if (motorTorqueBeforeAnim == null) return;
  int[] before = motorTorqueBeforeAnim;
  boolean[] wasOn = motorPowerBeforeAnim;
  motorTorqueBeforeAnim = null;
  motorPowerBeforeAnim = null;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (wasOn != null && !wasOn[i] && before[i] > 0) {
      motorsSetTorqueBar(i, before[i]);                 // its old bar value...
      motorTorquePending[i] = false;
      if (motorPowerOn(i)) motorsSetPowerSwitch(i, false); // ...but OFF, as it was
    } else {
      motorsSetTorque(i, before[i]);
    }
  }
}

// Torque as the motors really are: a row switched OFF counts as 0.
int[] motorsEffectiveTorque() {
  int[] t = new int[MOTOR_COUNT];
  for (int i = 0; i < MOTOR_COUNT; i++) t[i] = motorPowerOn(i) ? motorTorquePct[i] : 0;
  return t;
}

// "\nJoints switched OFF by the user ..." for agent prompts, or "".
String motorsUserOffNote() {
  String off = "";
  for (int i = 0; i < MOTOR_COUNT; i++) if (rowActive(i + 2) && !motorPowerOn(i)) off += (off.length() == 0 ? "" : ", ") + "joint " + (i + 1);
  return off.length() == 0 ? "" : "\nSwitched OFF by the user (limp on purpose): " + off
    + ". Keep them at torque 0 unless this object can't work with them limp; torque above 0 switches them back ON.";
}

// Motors (display ids) that are OFF now and that `plan` would switch back ON, e.g. "5, 6", or "".
String motorsSwitchedOnBy(int[] plan) {
  if (plan == null) return "";
  String s = "";
  for (int i = 0; i < MOTOR_COUNT && i < plan.length; i++) {
    if (rowActive(i + 2) && !motorPowerOn(i) && plan[i] > 0) s += (s.length() == 0 ? "" : ", ") + displayId(i + 2);
  }
  return s;
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
  // A pending target (dragged value, AI shape, animation frame) -> the bar.
  // Otherwise the TRUE angle behind the bar, unclamped - so a joint pushed
  // past its 0/180 limit is drawn where it really is, not at the limit.
  if (motorSliderPending(i)) return cp5.get(Slider.class, "Motor " + (i + 2)).getValue();
  // Bars = targets (AUTOMATION_FOLLOW_HAND_MOVES false): the bar shows the goal, the
  // 3D view the REAL arm from the latest live reading (if it is fresh).
  if (!manipulationMode && !AUTOMATION_FOLLOW_HAND_MOVES && !aiAnimating && rowActive(i + 2)
      && serialConnected() && motorActualMs > 0 && millis() - motorActualMs < 1000) return motorActualDeg[i];
  return motorShownDeg[i];
}

// The number shown on row i's angle bar, in INTERNAL degrees.
float motorReadoutDeg(int i) {
  if (motorSliderPending(i)) return cp5.getController("Motor " + (i + 2)).getValue();
  return motorShownDeg[i];
}

// Angle readouts on the bars, in DISPLAYED degrees (90 = straight, -15..195).
// Replaces ControlP5's own value label (which would show 90-270). Red with
// "LIMIT" when the true angle is past a limit (only possible by hand).
// Drawn after cp5.draw() so it sits on top of the bar.
void motorsDrawAngleReadouts() {
  pushStyle();
  textFont(hudFont);
  hudTextSize(13);
  textAlign(LEFT, CENTER);
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (!motorShowsAngle(i)) continue;
    float y = ROW_START_Y + i * ROW_STEP + SLIDER_H / 2.0;
    // Tick marks at displayed 0 and 180: the normal range. The bar runs
    // JOINT_EXTRA_DEG further each way for motors that can travel more.
    stroke(255, 150);
    strokeWeight(max(1, 1.5 * uiScale));
    for (float tick : new float[] {90, 270}) {
      float tx = SLIDER_X + SLIDER_W * (tick - JOINT_MIN_DEG) / (JOINT_MAX_DEG - JOINT_MIN_DEG);
      line(uiX(tx), uiY(y - SLIDER_H / 2.0), uiX(tx), uiY(y - SLIDER_H / 2.0 + 5));
      line(uiX(tx), uiY(y + SLIDER_H / 2.0 - 5), uiX(tx), uiY(y + SLIDER_H / 2.0));
    }
    noStroke();
    float v = motorReadoutDeg(i);
    boolean past = rowActive(i + 2) && (v < JOINT_MIN_DEG - 0.5 || v > JOINT_MAX_DEG + 0.5);
    String t = nf(toDisplayDeg(v), 1, 1) + "\u00B0" + (past ? "  LIMIT" : "");
    fill(0, 120);
    hudText(t, SLIDER_X + 5, y + 1);      // soft shadow so it reads on any fill
    fill(past ? color(255, 80, 80) : color(255));
    hudText(t, SLIDER_X + 4, y);
  }
  popStyle();
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
    // Goes to the angle on its bar (which, while the row was limp, followed
    // the real angle - so it holds where it is unless you dragged the bar).
    float here = cp5.getController("Motor " + rowId).getValue();
    if (AUTOMATION_FOLLOW_HAND_MOVES) { // old behaviour: hold exactly where it is now
      if (serialConnected() && millis() - motorActualMs < 600) here = motorActualDeg[i];
      else if (serialConnected()) here = readMotorPosition(rowId, here);
    }
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

// ---- Twitch (oscillation) safety net ----
// A holding motor whose reading keeps reversing direction - TWITCH_REVERSALS
// direction changes of at least TWITCH_MIN_STEP_DEG within TWITCH_WINDOW_MS -
// is oscillating (seen on XL330-M077s at a too-high P gain when the load on
// them changes). It is softened one stiffness level (lower P gain, I term
// off); if it is already at the softest level, it is switched OFF. Each step
// is reported in the chat. Reset restores full stiffness. The lasting fix is
// a lower P_GAIN_PCT_LIST entry for that motor in firmware local_config.h.
// Not checked during animations, while you drag that motor's bar, or for
// motors that are OFF / at 0%.
// OFF BY DEFAULT since 2026-11-18 (it switched motors off with no real
// problem; the stiffer arm doesn't need it). "twitch_guard": true in
// local_config.json turns it back on.
boolean TWITCH_GUARD_ON = false;
final float TWITCH_MIN_STEP_DEG = 2.5; // swings smaller than this are ignored (sensor noise up to ~+/-1 deg never triggers)
final int TWITCH_REVERSALS = 4;
final int TWITCH_WINDOW_MS = 1500;
float[] twLast = new float[MOTOR_COUNT];
int[] twDir = new int[MOTOR_COUNT];
long[][] twReversalMs = new long[MOTOR_COUNT][8];
int[] twReversalHead = new int[MOTOR_COUNT];
float[][] twRecent = new float[MOTOR_COUNT][8];
int[] twRecentHead = new int[MOTOR_COUNT];
long[] twLastActionMs = new long[MOTOR_COUNT];
boolean twInit = false;

void motorsTwitchReset(int i) {
  twLast[i] = Float.NaN;
  twDir[i] = 0;
  for (int k = 0; k < 8; k++) { twReversalMs[i][k] = 0; twRecent[i][k] = Float.NaN; }
}

void motorsTwitchCheck(int i, float v, long now) {
  if (!TWITCH_GUARD_ON) return;
  if (!twInit) { for (int k = 0; k < MOTOR_COUNT; k++) motorsTwitchReset(k); twInit = true; }
  Controller s = cp5.getController("Motor " + (i + 2));
  boolean dragging = motorDragPending[i] || (s != null && s.isMousePressed());
  if (dragging || !motorPowerOn(i) || motorTorquePct[i] == 0) { motorsTwitchReset(i); return; }

  if (Float.isNaN(twLast[i])) { twLast[i] = v; return; }
  float d = v - twLast[i];
  if (abs(d) < TWITCH_MIN_STEP_DEG) return; // jitter: ignore (slow drift still adds up)
  int dir = d > 0 ? 1 : -1;
  twRecent[i][twRecentHead[i]] = v;
  twRecentHead[i] = (twRecentHead[i] + 1) % 8;
  if (twDir[i] != 0 && dir != twDir[i]) {
    twReversalMs[i][twReversalHead[i]] = now;
    twReversalHead[i] = (twReversalHead[i] + 1) % 8;
  }
  twDir[i] = dir;
  twLast[i] = v;

  int n = 0;
  for (int k = 0; k < 8; k++) if (twReversalMs[i][k] > 0 && now - twReversalMs[i][k] <= TWITCH_WINDOW_MS) n++;
  if (n < TWITCH_REVERSALS || now - twLastActionMs[i] < 2000) return;

  float lo = 1e9, hi = -1e9;
  for (int k = 0; k < 8; k++) if (!Float.isNaN(twRecent[i][k])) { lo = min(lo, twRecent[i][k]); hi = max(hi, twRecent[i][k]); }
  twLastActionMs[i] = now;
  motorsTwitchReset(i);

  int rowId = i + 2;
  String who = "Motor " + displayId(rowId) + " is twitching (" + n + " direction changes in "
    + nf(TWITCH_WINDOW_MS / 1000.0, 1, 1) + " s, swinging ~" + round(hi - lo) + "\u00B0)";
  int level = hingeStiffnessLevel[i];
  String msg;
  if (level < NUM_STIFFNESS_LEVELS - 1) {
    level++;
    hingeStiffnessLevel[i] = level;
    sendHingeStiffness(rowId, level);
    msg = who + " - softened it to stiffness level " + level + ". Reset restores full stiffness; "
      + "for a lasting fix lower its P_GAIN_PCT_LIST entry in firmware local_config.h.";
  } else {
    motorsSetPowerSwitch(i, false);
    msg = who + " even at the softest level - switched it OFF. Check its cable/power, then switch it back ON.";
  }
  println("!!! " + msg);
  noticeAdd("error", msg);
}

// ---- Reset: check every motor and restart any that is shut down ----
// Runs in a background thread after Reset. Firmware command 240 checks each
// motor (answering? hardware error? torque on?) and restarts any that isn't
// running - rebooting it first if it's in an error state or not answering.
// A restarted motor holds where it is, so afterwards the Reset targets are
// sent again and it goes to the straight pose with the others. The result
// goes to the chat and the console, followed by the usual diagnostics table.
final int CHECK_RECOVER_HEADER = 240;

void motorsResetCheck() {
  delay(400); // let the Reset torque-on burst and neutral frame get processed first
  if (!serialConnected() || manipulationMode) return;
  float[] res = readAllMotorValues(CHECK_RECOVER_HEADER, 1.0, 6000); // reboots can take a few seconds
  if (res == null) {
    motorsPostNotice("error", "Motor check after Reset: no answer from the Arduino - upload the latest "
      + "firmware/lineform-ai/lineform-ai.ino (needs command 240).", false);
    delay(1500);
    motorsRunDiagnostics();
    return;
  }

  ArrayList<String> fixed = new ArrayList<String>();
  ArrayList<String> failed = new ArrayList<String>();
  int ok = 0, limp = 0;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int v = round(res[i]);
    if (v < 0) continue; // not fitted
    int code = v / 1000, err = v % 1000;
    String m = "motor " + displayId(i + 2);
    switch (code) {
    case 0: ok++; break;
    case 1: fixed.add(m + " (torque was OFF)"); break;
    case 2: fixed.add(m + " (" + motorsHwErrorText(err) + " - rebooted)"); break;
    case 3: fixed.add(m + " (wasn't answering - rebooted)"); break;
    case 4: failed.add(m + " NOT ANSWERING - check its power and cable"); break;
    case 5: failed.add(m + " restart FAILED" + (err != 0 ? " (" + motorsHwErrorText(err) + ")" : "")); break;
    case 6: limp++; break;
    default: failed.add(m + " unknown result " + v);
    }
  }

  String msg;
  String kind = "status";
  if (fixed.isEmpty() && failed.isEmpty()) {
    msg = "Motor check: all " + ok + " motors running" + (limp > 0 ? " (" + limp + " limp at 0% on purpose)" : "") + ".";
  } else {
    msg = "Motor check after Reset:";
    if (!fixed.isEmpty()) msg += " restarted " + String.join(", ", fixed) + ".";
    if (!failed.isEmpty()) { msg += " PROBLEM: " + String.join("; ", failed) + "."; kind = "error"; }
    msg += " " + ok + " motor(s) were fine.";
  }
  println(msg);
  motorsPostNotice(kind, msg, !fixed.isEmpty()); // re-send the Reset pose if anything was restarted
  delay(1500);
  motorsRunDiagnostics(); // console table, as before
}

// Posts a message to the bottom-left notice panel (never the AI chat), and
// optionally hands a re-send of the current targets to the drawing thread
// (serial frames must not be sent from a background thread).
void motorsPostNotice(final String kind, final String msg, final boolean resendTargets) {
  chatMainJobs.add(new Runnable() {
    public void run() {
      noticeAdd(kind, msg);
      if (resendTargets && !manipulationMode && !aiAnimating) aiSendSlidersOverSerial();
    }
  });
}

// Dynamixel X-series Hardware Error Status bits.
String motorsHwErrorText(int err) {
  ArrayList<String> s = new ArrayList<String>();
  if ((err & 0x01) != 0) s.add("input voltage");
  if ((err & 0x04) != 0) s.add("overheating");
  if ((err & 0x08) != 0) s.add("encoder");
  if ((err & 0x10) != 0) s.add("electrical shock");
  if ((err & 0x20) != 0) s.add("overload");
  return s.isEmpty() ? ("error " + err) : (String.join(" + ", s) + " error");
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
      + ", 252 stiff x" + round(c[6]) + ", 253 read1 x" + round(c[7]) + " | DROPPED (damaged) " + round(c[8])
      + " | stray bytes " + round(c[9]) + " | GUI: NAK x" + serialNakCount + ", ACK timeouts x" + serialAckTimeouts
      + " | GUI sent 251 x" + motorsTorqueSentCount
      + " | last 251: " + motorsLastTlText());
  }
  println("DIAGNOSTICS (firmware answered):");
  // Which model each motor is and the P gain it really runs (238 / 237).
  float[] models = readAllMotorValues(STATUS_MODEL_HEADER, 1.0);
  float[] pgains = readAllMotorValues(STATUS_PGAIN_HEADER, 1.0);
  println("  motor | model                  | P gain | mode            | torque | error | goal PWM % | goal current % | GUI torque % | firmware's last torque write");
  int plainMode = 0, errors = 0, off = 0, active = 0;
  ArrayList<String> unmatched = new ArrayList<String>();
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
    String modelText = "?                     ";
    if (models != null) {
      int mv = round(models[i]);
      boolean noSet = mv >= 19999;            // firmware adds 20000 when no set matched (-1 + 20000 = unreadable)
      int model = noSet ? mv - 20000 : mv;
      modelText = (motorModelName(model) + (noSet ? " (default!)" : "") + "                      ").substring(0, 22);
      if (noSet) unmatched.add("motor " + displayId(i + 2) + " = " + motorModelName(model));
    }
    String pText = pgains == null || pgains[i] < 0 ? "?     " : nf(round(pgains[i]), 5) + " ";
    println("  " + n + "    | " + modelText + " | " + pText + " | " + modeName + "| " + (on == 1 ? "ON " : "OFF") + "    | " + nf(err, 2) + "    | "
      + (pwm == null ? "?" : nf(pwm[i], 1, 1)) + "       | " + (cur == null ? "?" : nf(cur[i], 1, 1))
      + "           | " + motorTorquePct[i] + "   | " + motorsDiagLastText(i));
  }
  if (models == null) println("  (model / P gain columns need the latest firmware - commands 237/238)");
  if (!unmatched.isEmpty()) println("  NOTE: no MODEL_GAINS_LIST entry in firmware local_config.h for " + String.join(", ", unmatched)
    + " - they use the default STIFFNESS_P_GAINS_LIST / DROOP_I_GAIN.");
  println("  -> " + active + " motors answered; " + plainMode + " not in current-based mode; "
    + errors + " with a hardware error; " + off + " with torque OFF.");
}

final int STATUS_MODEL_HEADER = 238;
final int STATUS_PGAIN_HEADER = 237;

// Name for a Dynamixel model number (as reported by the motor); unknown
// numbers are shown as "model N". Only used for the console table.
String motorModelName(int model) {
  if (model < 0) return "unreadable";
  if (model == 1190) return "XL330-M077";
  if (model == 1200) return "XL330-M288";
  if (model == 1210) return "XC330-T181";
  if (model == 1220) return "XC330-T288";
  if (model == 1230) return "XC330-M181";
  if (model == 1240) return "XC330-M288";
  return "model " + model;
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
