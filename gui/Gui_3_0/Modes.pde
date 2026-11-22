// =====================================================================
//  MODES - Automation mode (normal) vs. Manipulation mode.
//
//  The switch at the bottom of the window flips between them.
//
//  Manipulation mode
//    * every motor's torque is OFF - the arm is limp, pose it by hand
//    * the angle bars and the 3D view follow the REAL angles, every reading,
//      with none of the Automation-mode filters (no 45 deg glitch filter, no
//      hand-move threshold, no "pending value" hold, no snap-to-goal)
//    * nothing that could move a motor is sent: Create Shape / Reset are
//      hidden, the bars can't be dragged, chat requests and animations are
//      refused, and sendSerialBytes() itself drops every motion command
//    * Save Shape still works - pose the arm by hand, then save it
//    * warm amber colors (Theme.pde) so it is obvious the motors are off
//
//  Back to Automation mode
//    * each motor holds EXACTLY where it was left (firmware command 241
//      HOLD_HERE: goal = its raw present position, then torque on), at the
//      torque % and ON/OFF state its row had before - rows that were OFF or
//      at 0% stay limp, just like in Automation mode
//    * the Automation-mode hold logic then takes over as usual: it waits for the
//      motor to settle (gravity sag is accepted as the new rest point, the
//      I term pulls most of it back) and only a real push moves it
//
//  Needs the updated lineform-ai.ino (command 241).
// =====================================================================

volatile boolean manipulationMode = false;

final int HOLD_HERE_HEADER = 241;
final int STATUS_HOLD_HEADER = 239;
final int MANIP_READ_INTERVAL_MS = 60; // read faster while posing by hand - smoother bars

// Bottom-center mode switch (logical 1600x870 coordinates). Drawn by hand
// (not a ControlP5 toggle) as a wide two-half switch, like the ANGLE/TORQUE
// switches: left half "AUTOMATION" in blue, right half "MANIPULATION" in amber.
// The active half is bright with an outline; the other half is dimmed.
// Click a half to pick that mode.
final float MODE_PANEL_X = 560;
final float MODE_PANEL_Y = 812;
final float MODE_PANEL_W = 580;
final float MODE_PANEL_H = 52;
final float MODE_SWITCH_W = 420;
final float MODE_SWITCH_H = 38;
final float MODE_SWITCH_X = MODE_PANEL_X + (MODE_PANEL_W - MODE_SWITCH_W) / 2.0;
final float MODE_SWITCH_Y = MODE_PANEL_Y + (MODE_PANEL_H - MODE_SWITCH_H) / 2.0;
final color MODE_AUTOMATION_COLOR    = color(64, 140, 255);  // blue  (= THEME_BLUE)
final color MODE_MANIP_COLOR = color(255, 150, 50);  // amber (= THEME_ORANGE, like the TORQUE half)

boolean modesMouseOver() {
  return mouseX >= uiX(MODE_SWITCH_X) && mouseX <= uiX(MODE_SWITCH_X + MODE_SWITCH_W)
      && mouseY >= uiY(MODE_SWITCH_Y) && mouseY <= uiY(MODE_SWITCH_Y + MODE_SWITCH_H);
}

// Called first thing in mousePressed(). Returns true if the click was on the switch.
boolean modesMousePressed() {
  if (!modesMouseOver()) return false;
  boolean rightHalf = mouseX >= uiX(MODE_SWITCH_X + MODE_SWITCH_W / 2.0);
  if (rightHalf) modeEnterManipulation();
  else modeExitManipulation();
  return true;
}

// ---------------------------------------------------------------------
//  OVERLOAD PROTECTION (Automation mode). If any powered motor's live torque use
//  (the white line - % of its full-strength Current Limit) reaches
//  OVERLOAD_TRIP_PCT, the GUI switches to Manipulation mode (all torque
//  OFF) and shows a warning. Works for every row, whether it shows its
//  ANGLE or TORQUE bar - loads are read every cycle in Automation mode.
//  ON (this is what switches to Manipulation mode automatically). Override
//  in local_config.json: "overload_trip_pct" (default 95; 0 = off),
//  "overload_confirm_readings" (readings in a row, ~120 ms apart, needed to
//  trip; 1 = first reading). The warning is pinned in the bottom-left
//  notice panel (Notices.pde).
// ---------------------------------------------------------------------
float OVERLOAD_TRIP_PCT = 95; // 0 = off
int OVERLOAD_CONFIRM_READINGS = 2;
int[] overloadCount = new int[MOTOR_COUNT];
int overloadLastSeq = -1;
String overloadWarning = null; // shown in Manipulation mode after a trip; cleared on return to Automation mode

// Called every frame from motorsTick().
void modesCheckOverload() {
  if (manipulationMode || OVERLOAD_TRIP_PCT <= 0) return;
  int seq = motorLoadSeq;
  if (seq == overloadLastSeq) return; // count each new reading once
  overloadLastSeq = seq;
  float[] loads = motorLoadPct;
  if (loads == null) return;

  String who = "";
  int trips = 0;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int rowId = i + 2;
    boolean powered = rowActive(rowId) && motorPowerOn(i) && motorTorquePct[i] > 0;
    if (powered && loads[i] >= OVERLOAD_TRIP_PCT) overloadCount[i]++;
    else overloadCount[i] = 0;
    if (overloadCount[i] >= OVERLOAD_CONFIRM_READINGS) {
      who += (trips == 0 ? "" : ", ") + "motor " + displayId(rowId) + " (" + round(loads[i]) + "%)";
      trips++;
    }
  }
  if (trips == 0) return;

  for (int i = 0; i < MOTOR_COUNT; i++) overloadCount[i] = 0;
  modeEnterManipulation();
  overloadWarning = "OVERLOAD: " + who + " reached " + round(OVERLOAD_TRIP_PCT)
    + "% torque. All motors switched OFF to protect them.";
  println("!!! " + overloadWarning);
  // (shown PINNED in the bottom-left notice panel - Notices.pde - not in the chat)
}

// ---------------------------------------------------------------------
//  Safety net: while in Manipulation mode only READ commands and
//  torque-OFF may reach the Arduino. Called by sendSerialBytes().
// ---------------------------------------------------------------------
boolean serialAllowedInManipulation(int[] bytes) {
  if (bytes == null || bytes.length == 0) return true;
  int h = bytes[0];
  if (h == 254) { // torque on/off: only "off" (one or more 254,id,0 triples)
    for (int k = 2; k < bytes.length; k += 3) if (bytes[k] != 0) return false;
    return true;
  }
  return h == READ_ALL_HEADER || h == READ_ALL_LOAD_HEADER || h == 253
    || (h >= STATUS_COUNTS_HEADER && h <= STATUS_HEADER); // 242-247 diagnostics
}

// ---------------------------------------------------------------------
//  Entering / leaving Manipulation mode (drawing thread)
// ---------------------------------------------------------------------
void modeEnterManipulation() {
  if (manipulationMode) return;
  // Finish anything that still sends motion/torque commands FIRST (stopping an
  // animation restores stiffness + torque), while those are still allowed.
  aiStopAnimation();

  manipulationMode = true;

  // All active motors limp, in ONE atomic write (the live reader can't cut in).
  ArrayList<Integer> off = new ArrayList<Integer>();
  for (int rowId = 2; rowId <= MOTOR_COUNT + 1; rowId++) {
    if (!rowActive(rowId)) continue;
    off.add(254); off.add(hwId(rowId)); off.add(0);
  }
  int[] frame = new int[off.size()];
  for (int k = 0; k < frame.length; k++) frame[k] = off.get(k);
  sendSerialBytes(frame);

  // Forget every Automation-mode hold / drag / pending state.
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorDragPending[i] = false;
    motorTorquePending[i] = false;
    motorHandMoved[i] = false;
    motorRestCount[i] = 0;
    motorPushCount[i] = 0;
    motorJumpSeen[i] = false;
  }
  motorsUserDragging = false;

  modesApplyControls();
  themeApplyMode();
  println("MANIPULATION MODE: all motors torque OFF - move the arm by hand.");
  noticeAdd("status", "Manipulation mode: all motors are OFF. Move the arm by hand - the bars show the real angles. "
    + "Switch back to Automation mode to hold the pose. (Any unsent bar values were dropped.)");
}

void modeExitManipulation() {
  if (!manipulationMode) return;
  manipulationMode = false;
  overloadWarning = null;
  for (int i = 0; i < MOTOR_COUNT; i++) overloadCount[i] = 0;
  overloadLastSeq = motorLoadSeq; // judge only readings taken after the motors hold again

  // Freshest real angles (fall back to the last reading / the bars).
  float[] here = readAllMotorValues(READ_ALL_HEADER, 100.0);
  long now = millis();
  int held = 0, limp = 0;
  String pastLimit = "";
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int rowId = i + 2;
    if (!rowActive(rowId)) continue;
    float deg = (here != null) ? here[i]
      : (motorActualMs > 0 ? motorActualDeg[i] : cp5.getController("Motor " + rowId).getValue());
    motorActualDeg[i] = deg;
    motorPrevActual[i] = deg;
    motorsDisplay(i, deg);           // bar = this pose, not "pending"

    // This pose is now the motor's goal - exactly like right after a sent goal,
    // so the Automation-mode hold logic (settle, rest baseline, push detection) restarts.
    // (Clamped: a joint left past 0/180 is pulled back to the limit - the
    // firmware's HOLD_HERE does the same.)
    motorLastSentDeg[i] = clampJoint(deg);
    motorLastSendMs[i] = now;
    motorReachedGoal[i] = false;
    motorRestBaseline[i] = Float.NaN;
    motorHandMoved[i] = false;
    motorRestCount[i] = 0;
    motorPushCount[i] = 0;
    motorJumpSeen[i] = false;

    // Same ON/OFF and torque % the row had in Automation mode.
    boolean hold = motorPowerOn(i) && motorTorquePct[i] > 0;
    if (hold) {
      sendSerialBytes(HOLD_HERE_HEADER, hwId(rowId), motorTorquePct[i]);
      held++;
    } else {
      limp++;
    }
    if (hold && (deg < JOINT_MIN_DEG - 0.5 || deg > JOINT_MAX_DEG + 0.5)) {
      pastLimit += (pastLimit.length() == 0 ? "" : ", ") + "motor " + displayId(rowId) + " ("
        + nf(toDisplayDeg(deg), 1, 1) + "\u00B0 -> " + round(toDisplayDeg(clampJoint(deg))) + "\u00B0)";
    }
  }
  if (here != null) motorActualMs = now;
  motorsBarsSynced = true; // the hand-made pose is now every bar's target

  modesApplyControls();
  themeApplyMode();
  println("AUTOMATION MODE: " + held + " motors holding the hand-posed angles" + (limp > 0 ? ", " + limp + " left limp (OFF / 0%)" : "") + ".");
  noticeAdd("status", "Automation mode: the arm holds the pose you made" + (limp > 0 ? " (" + limp + " motor(s) stay limp - switched OFF or at 0% torque)" : "") + ".");
  if (pastLimit.length() > 0) {
    String m = "Moved back inside the " + jointRangeText() + " range: " + pastLimit + ".";
    println(m);
    noticeAdd("status", m);
  }
  if (serialConnected()) {
    motorsDiagDueMs = millis() + 2500; // console diagnostics table, as before
    thread("modesVerifyHold");          // and: is every motor REALLY holding? (chat)
  }
}

// ~1.5 s after leaving Manipulation mode: asks the firmware (239) whether
// each motor that should hold really is - torque on, the hand-made pose
// written as its goal, and still near it - and says so in the chat. Pins
// down WHY a pose isn't held instead of just "it fell".
void modesVerifyHold() {
  delay(1500);
  if (!serialConnected() || manipulationMode) return;
  if (serialAckMisses >= 3) {
    motorsPostNotice("error", "Hold check: the Arduino isn't acknowledging commands - it runs OLD firmware, which doesn't "
      + "know HOLD_HERE (241), so the motors stay limp. Upload firmware/lineform-ai/lineform-ai.ino.", false);
    return;
  }
  float[] st = readAllMotorValues(STATUS_HOLD_HEADER, 1.0);
  if (st == null) {
    motorsPostNotice("error", "Hold check: no answer to command 239 - the Arduino runs older firmware. Upload "
      + "firmware/lineform-ai/lineform-ai.ino (needed for holding the pose after Manipulation mode).", false);
    return;
  }
  ArrayList<String> off = new ArrayList<String>(), noGoal = new ArrayList<String>(), drift = new ArrayList<String>();
  int checked = 0;
  float worst = 0;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (!rowActive(i + 2) || !motorPowerOn(i) || motorTorquePct[i] == 0) continue; // limp on purpose
    int v = round(st[i]);
    if (v < 0) continue;
    checked++;
    boolean on = v >= 10000;
    boolean goalOk = (v % 10000) >= 5000;
    float dev = (v % 5000) / 10.0;
    String m = "motor " + displayId(i + 2);
    if (!on) off.add(m);
    else if (!goalOk) noGoal.add(m);
    else if (dev > 5) drift.add(m + " (" + nf(dev, 1, 1) + "\u00B0)");
    if (on && goalOk) worst = max(worst, dev);
  }
  if (off.isEmpty() && noGoal.isEmpty() && drift.isEmpty()) {
    println("Hold check: all " + checked + " motors holding (within " + nf(worst, 1, 1) + " deg of the hand-made pose).");
    return;
  }
  String msg = "Hold check after leaving Manipulation mode:";
  if (!off.isEmpty()) msg += " TORQUE OFF on " + String.join(", ", off) + " (didn't turn back on - press Reset; if it repeats, check power/cable).";
  if (!noGoal.isEmpty()) msg += " The hand-made pose could NOT be written as the goal of " + String.join(", ", noGoal)
    + " - they may have driven back to their old pose.";
  if (!drift.isEmpty()) msg += " Drifted away from the pose: " + String.join(", ", drift)
    + " - too weak/soft for the load (raise the torque bar or stiffness), or pulled back to a joint limit.";
  println("!!! " + msg);
  motorsPostNotice("error", msg, false);
}

// Shows / hides / locks the left-panel controls for the current mode.
void modesApplyControls() {
  boolean m = manipulationMode;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int id = i + 2;
    Controller a = cp5.getController("Motor " + id);
    if (a != null) a.setLock(m);
    Controller p = cp5.getController("Power " + id);
    if (p != null) p.setVisible(!m);
    if (m) {
      if (a != null) a.setVisible(true);  // every row shows its angle bar
      Controller t = cp5.getController("Torque Level " + id);
      if (t != null) t.setVisible(false);
      Controller st = cp5.getController("Stiffness " + id);
      if (st != null) st.setVisible(false);
    } else {
      motorsApplyView(id);                // back to each row's ANGLE / TORQUE / STIFF choice
    }
  }
  // Create Shape / Reset, and the whole AI chat panel's controls.
  for (String n : new String[] {"Create Shape", "Reset", "Save Animation", "Load Shape", "Load Animation",
                                "Chat Input", "Send", "New Chat", "Critic Toggle", "Mode Shape", "Mode Animate"}) {
    Controller c = cp5.getController(n);
    if (c != null) c.setVisible(!m);
  }
  if (m && chatInput != null) chatInput.setFocus(false); // don't keep typing into a hidden box
}

// ---------------------------------------------------------------------
//  Live readings in Manipulation mode (called from motorsTick())
// ---------------------------------------------------------------------
void modesManipulationTick(long now) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorDragPending[i] = false;
    motorTorquePending[i] = false;
  }
  motorsUserDragging = false;

  float[] r = motorLiveReading;
  if (r == null) return;
  motorLiveReading = null;
  // Motors are limp, so the reading IS the pose: no glitch filter, no
  // thresholds, no pending holds. (The firmware already repeats the last good
  // value when a single read fails, so a failed read never shows as 0 deg.)
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorActualDeg[i] = r[i];
    motorPrevActual[i] = r[i];
    motorJumpSeen[i] = false;
    if (rowActive(i + 2)) motorsDisplay(i, r[i]);
  }
  motorActualMs = now;
}

// ---------------------------------------------------------------------
//  Drawing (inside cam.beginHUD())
// ---------------------------------------------------------------------
void drawModeSwitchPanel() {
  boolean m = manipulationMode;
  pushStyle();
  themePanel(MODE_PANEL_X, MODE_PANEL_Y, MODE_PANEL_W, MODE_PANEL_H);

  // "MODE" caption left of the switch
  textFont(uiBoldFont(max(8, round(13 * uiScale))));
  textAlign(RIGHT, CENTER);
  fill(THEME_TEXT);
  hudText("MODE", MODE_SWITCH_X - 10, MODE_SWITCH_Y + MODE_SWITCH_H / 2.0);

  // Two halves: always blue | amber. Active half bright, other half dimmed.
  float half = MODE_SWITCH_W / 2.0;
  float r = 8 * uiScale;
  boolean hover = modesMouseOver();
  boolean hoverRight = hover && mouseX >= uiX(MODE_SWITCH_X + half);
  noStroke();
  // left = AI (blue)
  color autoFill = !m ? MODE_AUTOMATION_COLOR : themeShade(MODE_AUTOMATION_COLOR, (hover && !hoverRight) ? -0.45 : -0.65);
  fill(autoFill);
  rect(uiX(MODE_SWITCH_X), uiY(MODE_SWITCH_Y), half * uiScale, MODE_SWITCH_H * uiScale, r, 0, 0, r);
  // right = MANIPULATION (amber)
  color mnFill = m ? MODE_MANIP_COLOR : themeShade(MODE_MANIP_COLOR, (hover && hoverRight) ? -0.45 : -0.65);
  fill(mnFill);
  rect(uiX(MODE_SWITCH_X + half), uiY(MODE_SWITCH_Y), half * uiScale, MODE_SWITCH_H * uiScale, 0, r, r, 0);
  // white outline around the active half
  noFill();
  stroke(255);
  strokeWeight(max(2, 3 * uiScale));
  float ax = m ? MODE_SWITCH_X + half : MODE_SWITCH_X;
  rect(uiX(ax), uiY(MODE_SWITCH_Y), half * uiScale, MODE_SWITCH_H * uiScale,
       m ? 0 : r, m ? r : 0, m ? r : 0, m ? 0 : r);
  noStroke();

  // labels inside the halves (dark on bright, muted on dimmed)
  textFont(uiBoldFont(max(8, round(16 * uiScale))));
  textAlign(CENTER, CENTER);
  float cy = MODE_SWITCH_Y + MODE_SWITCH_H / 2.0;
  fill(!m ? color(15, 20, 35) : color(150, 170, 210));
  hudText((!m ? "● " : "") + "AUTOMATION", MODE_SWITCH_X + half / 2.0, cy);
  fill(m ? color(40, 22, 5) : color(215, 165, 110));
  hudText((m ? "● " : "") + "MANIPULATION", MODE_SWITCH_X + half * 1.5, cy);
  popStyle();

  if (!m) return;
  // Banner over the 3D view
  pushStyle();
  float bx = 560, by = 14, bw = 580, bh = 46;
  fill(MANIP_ACCENT, 40);
  stroke(MANIP_ACCENT);
  strokeWeight(max(1, 2 * uiScale));
  rect(uiX(bx), uiY(by), bw * uiScale, bh * uiScale, 10 * uiScale);
  noStroke();
  textAlign(CENTER, CENTER);
  fill(MANIP_ACCENT);
  textFont(uiBoldFont(max(9, round(20 * uiScale))));
  hudText("MANIPULATION MODE - MOTORS OFF", bx + bw / 2, by + 15);
  textFont(hudFont);
  hudTextSize(13);
  fill(THEME_TEXT);
  hudText("Move the arm by hand. Switch back to Automation mode to hold this pose.", bx + bw / 2, by + 34);

  // (The overload warning is no longer drawn here: every warning sits in
  // the bottom-left notice panel - Notices.pde.)
  popStyle();
}

// Text in the left panel where Create Shape / Reset normally sit.
void drawManipulationHint() {
  if (!manipulationMode) return;
  pushStyle();
  int y = ROW_START_Y + MOTOR_COUNT * ROW_STEP + 20;
  textFont(uiBoldFont(max(8, round(15 * uiScale))));
  textAlign(LEFT, TOP);
  fill(MANIP_ACCENT);
  hudText("Motors are OFF - pose the arm by hand.", SLIDER_X, y + 4);
  textFont(hudFont);
  hudTextSize(13);
  fill(THEME_TEXT);
  hudText("Bars show the real angles. Save Shape suggests \"hand pose <time>\".", SLIDER_X, y + 28);
  hudText("Create Shape / Reset return in Automation mode.", SLIDER_X, y + 46);
  popStyle();
}
