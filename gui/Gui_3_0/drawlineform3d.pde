void drawlineform3d() {

  // Slider control names are "Motor 2".."Motor 12" (internal row ids);
  // motorAngles[i] is row i+2, base to tip.
  // Live angle per row: the real arm's reading when available, otherwise
  // the slider (see motorCanvasAngle() in Motors.pde).
  float[] motorAngles = new float[MOTOR_COUNT];
  for (int i = 0; i < MOTOR_COUNT; i++) motorAngles[i] = motorCanvasAngle(i);
  // All 11 rows are drawn; only the first ACTIVE_MOTORS bend (the rest are
  // not fitted on this arm - see local_config.json).

  // Geometry: see the "3D chain geometry" constants in Gui_3_0.pde (real
  // XL330 proportions, 0.8 cm bridge gap, pivots on the motor shafts).
  // aiComputeAchievableCurve() / aiSolveAnglesForCurve() use the same
  // constants, so the AI's predicted joint positions match what's drawn.

  // Collect each joint's actual 3D position as we walk the chain, so we
  // can draw a curve through the real linkage shape afterward. Reading
  // modelX/Y/Z here (rather than guessing a transform elsewhere) means
  // this curve is always exactly where the boxes are, however the chain
  // geometry changes.
  ArrayList<PVector> jointCurve = new ArrayList<PVector>();

  pushMatrix();
  translate(CHAIN_ORIGIN.x, CHAIN_ORIGIN.y, CHAIN_ORIGIN.z);   // = joint 1's pivot

  if (BASE_BLOCK_FIRST) {
    // Fixed (passive) block BEFORE the first motor - the unconnected ID 1 motor, labelled "1": its shaft is the first joint's
    // pivot, so it sits one step behind, never rotating.
    pushMatrix();
    translate(LINK_BOX_CENTER_X - LINK_STEP, 0, 0);
    drawLinkBlock(0, THEME_LINK_PASSIVE, PASSIVE_BLOCK_LABEL);
    popMatrix();
  } else {
    // No passive block at the base: a fixed mount plate behind
    // joint 1, so motor 1 visibly swings about something.
    pushMatrix();
    translate(-LINK_WIDTH * 0.45, 0, 0);
    fill(THEME_LINK_PASSIVE);
    stroke(255, 100);
    strokeWeight(1);
    box(LINK_WIDTH * 0.5, LINK_THICK * 1.6, LINK_THICK * 1.6);
    popMatrix();
  }

  // Pivot dot for motor 1 - the fixed point the whole arm rotates about.
  pushMatrix();
  translate(hingeOffsetX, hingeOffsetY, hingeOffsetZ);
  noStroke();
  fill(255, 220, 0);
  sphere(5);
  popMatrix();

  jointCurve.add(new PVector(modelX(0, 0, 0), modelY(0, 0, 0), modelZ(0, 0, 0)));

  for (int i = 0; i < MOTOR_COUNT; i++) {

    int motorNumber = i + 1;    // software "Motor N" slider index (1-11)
    int dxlId = motorNumber + 1; // internal row id (2-12)

    boolean fitted = i < ACTIVE_MOTORS; // a real motor on this arm?
    float safeAngle = constrain(motorAngles[i], 0, 360); // NOT the joint limits: the 3D view shows the TRUE angle, even past a limit
    float angle = fitted ? safeAngle - 180 : 0; // not fitted: always straight

    if (aiIsMirroredMotor(dxlId)) {
        angle = -angle;
    }

    float rad = radians(angle);

    // Rotate at the joint (the previous block's shaft) before drawing.
    if (motorNumber % 2 == 1) {
      rotateZ(-rad);
    } else {
      rotateY(rad);
    }

    // Bridge bracket across the 0.8 cm gap (turns with this joint).
    drawLinkBridge(motorNumber);

    // This motor's block.
    pushMatrix();
    translate(LINK_BOX_CENTER_X, 0, 0);
    color c = !fitted ? THEME_LINK_PASSIVE : (motorNumber % 2 == 1 ? THEME_LINK_A : THEME_LINK_B);
    color labelColor = color(255);
    if (fitted && loadColorsActive()) {   // load heat-map (2026-11-19)
      c = loadColor(blockLoadShown(i));
      labelColor = loadLabelColor(c);
    }
    drawLinkBlock(motorNumber, c, "" + displayId(dxlId), labelColor); // real ID: 2 next to the base ... 12 at the tip

    // Record this block's actual centre in world space, so the curve runs
    // straight through the blocks.
    jointCurve.add(new PVector(modelX(0, 0, 0), modelY(0, 0, 0), modelZ(0, 0, 0)));
    popMatrix();

    translate(LINK_STEP, 0, 0); // to the next pivot (this block's shaft)
  }

  // Fixed "12" block AFTER the last motor (BASE_BLOCK_FIRST = false in
  // local_config.json) - one more straight grey block.
  if (!BASE_BLOCK_FIRST) {
    drawLinkBridge(MOTOR_COUNT + 1);
    pushMatrix();
    translate(LINK_BOX_CENTER_X, 0, 0);
    drawLinkBlock(MOTOR_COUNT + 1, THEME_LINK_PASSIVE, PASSIVE_BLOCK_LABEL);
    popMatrix();
  }
  popMatrix();

  // (Clickable hinge markers removed 2026-11-16 - stiffness is set per row
  // with the ANGLE | TORQUE | STIFF selector in the left panel.)

  // Curve through the actual joint positions - the real 3D shape the
  // linkage takes given the current slider angles. Drawn in the same
  // space as the boxes above, so it always overlaps them.
  //
  // The physical chain is always just 11 straight rigid links, so a
  // straight-segment polyline through their positions makes even a
  // perfectly-fit smooth curve (a loop, a wave) look faceted/jagged. For a
  // shape that's actually meant to be curved (aiShapeIsCurved), draw a
  // smoothed spline through those same joint positions instead - this only
  // changes how the line is drawn, not any servo angle. Pure-LINE shapes
  // (square, hammer) keep straight segments, since their corners are
  // genuinely sharp and smoothing them would misrepresent the real shape.
  stroke(255, 220, 0);
  strokeWeight(3);
  noFill();
  beginShape();
  if (aiShapeIsCurved && jointCurve.size() >= 2) {
    // Processing's own curveVertex() draws Catmull-Rom too, but with no
    // control over tension or how many segments it tessellates each span
    // into - with only 12 points spread across the whole arm, that default
    // is what was reading as "weak"/underwhelming. aiCatmullRomChain (in
    // Gui_07_07.pde) subdivides explicitly (AI_CURVE_RENDER_STEPS_PER_SEGMENT
    // points per joint-to-joint span) at a tunable tension
    // (AI_CURVE_RENDER_TENSION), still passing through every real joint
    // position exactly - this only changes how the line LOOKS, not any
    // servo angle or joint position.
    ArrayList<PVector> smoothed = aiCatmullRomChain(jointCurve, AI_CURVE_RENDER_STEPS_PER_SEGMENT, AI_CURVE_RENDER_TENSION);
    for (PVector p : smoothed) vertex(p.x, p.y, p.z);
  } else {
    for (PVector p : jointCurve) vertex(p.x, p.y, p.z);
  }
  endShape();

  // Waiting item (Staging.pde): translucent green ghost arm, animated.
  drawStagedPreview();

  // AI target curve overlay - the shape Claude proposed, before it was
  // quantized down to 11 rigid-joint bend angles. Purely visual: nothing
  // here is sent to the motors. Lets you compare "what I asked for" (cyan)
  // against "what the arm can actually do" (yellow) before pressing
  // Create Shape.
  if (aiHasPreview && aiPreviewCurve.size() > 1) {
    stroke(0, 220, 255);
    strokeWeight(2);
    noFill();
    beginShape();
    for (PVector p : aiPreviewCurve) vertex(p.x, p.y, p.z);
    endShape();
  }

  // Closing stroke for a LOOP/SPIRAL head (e.g. a racket face): the arm is
  // a single OPEN serial chain (fixed base, free tip), so the last joint
  // can never physically touch back onto an earlier point in the same
  // chain - there's no joint after 12 to fold the tip back with. This line
  // is a visual closure only, drawn between the actual last joint and the
  // joint that got anchored to the loop's attach point on the handle
  // (aiGenerateFromSegments/aiResampleWithCornerAnchors set these), so the
  // shape still reads as closed/connected on screen. Its length is also an
  // honest readout of fit quality: short means the arm nearly closed the
  // loop on its own, long means it fell well short.
  if (aiShapeHasLoop && aiLoopAttachMotor >= 1 && aiLoopAttachMotor < jointCurve.size()) {
    PVector tip = jointCurve.get(jointCurve.size() - 1);
    PVector anchor = jointCurve.get(aiLoopAttachMotor);
    stroke(255, 220, 0, 140);
    strokeWeight(2);
    line(tip.x, tip.y, tip.z, anchor.x, anchor.y, anchor.z);
  }
}

// One motor block, centred at the current origin (on the hinge axis).
// Block n is THIN along the axis of joint n+1 (the joint at its far end,
// whose shaft it carries) and THICK in that joint's bending plane: joint
// n+1 bends about Z when n+1 is odd, about Y when it is even. So
// consecutive blocks are turned 90 deg to each other, like the real arm.
void drawLinkBlock(int n, color c, String label) {
  drawLinkBlock(n, c, label, color(255));
}

void drawLinkBlock(int n, color c, String label, color labelColor) {
  boolean nextAboutZ = (n + 1) % 2 == 1;
  float sy = nextAboutZ ? LINK_THICK : LINK_THIN;
  float sz = nextAboutZ ? LINK_THIN : LINK_THICK;
  fill(c);
  stroke(255, 100);
  strokeWeight(1);
  box(LINK_WIDTH, sy, sz);
  pushMatrix();
  translate(0, 0, sz / 2.0 + 1); // front face, for the number
  fill(labelColor);
  textAlign(CENTER, CENTER);
  textSize(16);
  text(label, 0, 0);
  popMatrix();
}

// Bridge bracket of joint j, drawn at its pivot (current origin, after the
// joint's rotation, so it turns with the block it drives): two side plates
// on the shaft ends, just outside the previous block's THIN sides, reaching
// across the gap to the next block's end face, plus the plate across that
// face. Visual only.
void drawLinkBridge(int j) {
  boolean aboutZ = j % 2 == 1;                  // this joint's axis
  float plateT = 1.6 * LINK_UNITS_PER_MM;       // bracket thickness
  float plateW = LINK_THIN * 0.8;               // width in the bending plane
  float x0 = -LINK_SHAFT_FROM_END * 0.9;        // from around the shaft...
  float x1 = LINK_SHAFT_FROM_END + LINK_GAP;    // ...to the next block's end face
  float off = LINK_THIN / 2.0 + plateT / 2.0 + 0.5;
  pushStyle();
  fill(150, 156, 168, 70);   // dim, see-through grey - the brackets shouldn't compete with the blocks
  stroke(255, 25);
  strokeWeight(1);
  for (int side = -1; side <= 1; side += 2) {
    pushMatrix();
    if (aboutZ) translate((x0 + x1) / 2.0, 0, side * off);
    else translate((x0 + x1) / 2.0, side * off, 0);
    if (aboutZ) box(x1 - x0, plateW, plateT);
    else box(x1 - x0, plateT, plateW);
    popMatrix();
  }
  pushMatrix(); // across the next block's end face
  translate(x1 - plateT / 2.0, 0, 0);
  if (aboutZ) box(plateT, plateW, 2 * off + plateT);
  else box(plateT, 2 * off + plateT, plateW);
  popMatrix();
  popStyle();
}

// =====================================================================
//  LOAD HEAT-MAP on the 3D motor blocks (2026-11-19)
//  Like a load simulation: each fitted motor block is colored by its live
//  torque use (the same % as the white line on its bar, 0-100% of its
//  Current Limit): blue (no load) -> cyan -> green -> yellow -> RED (full
//  load). Shown in Automation mode whenever live loads are being read; in
//  Manipulation mode (motors limp), offline, or with "load_colors_3d": false
//  in local_config.json, the blocks keep their normal colors.
// =====================================================================
boolean LOAD_COLORS_3D = true; // switched at runtime by the LOAD COLORS switch (top-right of the 3D view)
// Color multiplier: the block color shows load x this, so small loads can be
// pushed toward red. Set with the GAIN slider under the LOAD COLORS switch;
// "load_color_gain" in local_config.json sets the start value (1 = true scale).
// Only the 3D colors use it - the white line and the text under the bars
// always show the real load.
float LOAD_COLOR_GAIN = 1;
final float LOAD_GAIN_MAX = 20;           // slider range 1x .. 20x (logarithmic)
final float LOAD_COLOR_SMOOTHING = 0.25;  // per frame, 0..1 - higher = follows faster, lower = less flicker
float[] blockLoadSmoothed = new float[MOTOR_COUNT];
int blockLoadFrame = -1;

boolean loadColorsActive() {
  return LOAD_COLORS_3D && !manipulationMode && serialConnected() && motorLoadPct != null;
}

// Smoothed load % of block i (updated once per frame).
float blockLoadShown(int i) {
  if (blockLoadFrame != frameCount) {
    blockLoadFrame = frameCount;
    float[] loads = motorLoadPct;
    for (int k = 0; k < MOTOR_COUNT; k++) {
      // x LOAD_COLOR_GAIN (the multiplier slider): with gain 10, a 10% load is already red.
      float target = (loads == null) ? 0 : constrain(loads[k] * LOAD_COLOR_GAIN, 0, 100);
      blockLoadSmoothed[k] = lerp(blockLoadSmoothed[k], target, LOAD_COLOR_SMOOTHING);
    }
  }
  return blockLoadSmoothed[i];
}

final color[] LOAD_STOPS = {
  color(40, 90, 255),   //   0 %  blue
  color(0, 200, 255),   //  25 %  cyan
  color(40, 210, 90),   //  50 %  green
  color(255, 215, 0),   //  75 %  yellow
  color(255, 35, 35)    // 100 %  red
};

color loadColor(float pct) {
  float t = constrain(pct / 100.0, 0, 1) * (LOAD_STOPS.length - 1);
  int k = min(floor(t), LOAD_STOPS.length - 2);
  return lerpColor(LOAD_STOPS[k], LOAD_STOPS[k + 1], t - k);
}

// Dark number on light (yellow / green / cyan) blocks, white on dark ones.
color loadLabelColor(color c) {
  float lum = 0.299 * red(c) + 0.587 * green(c) + 0.114 * blue(c);
  return lum > 150 ? color(20) : color(255);
}

// ---- LOAD COLORS switch + color key, top-right of the 3D view (HUD) ----
// Click the switch to turn the heat-map on / off at any time (e.g. to watch
// an animation in the normal colors). Starts as "load_colors_3d" in
// local_config.json says. Shown in Automation mode; the color key only
// while the heat-map is actually on and live loads are coming in.
final float LOADSW_X = 960, LOADSW_Y = 14, LOADSW_W = 205, LOADSW_H = 26;

boolean loadSwitchVisible() { return !manipulationMode; }

boolean loadSwitchMouseOver() {
  return loadSwitchVisible()
      && mouseX >= uiX(LOADSW_X) && mouseX <= uiX(LOADSW_X + LOADSW_W)
      && mouseY >= uiY(LOADSW_Y) && mouseY <= uiY(LOADSW_Y + LOADSW_H);
}

// Switch OR multiplier slider under the mouse (keeps the camera from orbiting).
boolean loadPanelMouseOver() {
  return loadSwitchMouseOver() || loadGainMouseOver() || loadGainDragging;
}

// Called from mousePressed(). Returns true if the switch or slider was clicked.
boolean loadSwitchMousePressed() {
  if (loadGainMouseOver()) { loadGainDragging = true; loadGainSetFromMouse(); return true; }
  if (!loadSwitchMouseOver()) return false;
  LOAD_COLORS_3D = !LOAD_COLORS_3D;
  println("3D load colors " + (LOAD_COLORS_3D ? "ON" : "OFF"));
  return true;
}

void drawLoadLegend() {
  if (!loadSwitchVisible()) return;
  pushStyle();
  boolean on = LOAD_COLORS_3D;
  boolean hover = loadSwitchMouseOver();
  float x = LOADSW_X, y = LOADSW_Y, w = LOADSW_W, h = LOADSW_H, pad = 8;

  // the switch: caption left, ON/OFF pill right
  themePanel(x, y, w, h);
  textFont(uiBoldFont(max(7, round(11 * uiScale))));
  textAlign(LEFT, CENTER);
  fill(THEME_TEXT);
  hudText("LOAD COLORS (3D)", x + pad, y + h / 2.0);
  float pw = 52, ph = h - 8, px = x + w - pad - pw, py = y + 4;
  noStroke();
  color pill = on ? loadColor(100) : THEME_CTRL_BG;
  fill(hover ? themeShade(pill, on ? -0.15 : 0.25) : pill);
  rect(uiX(px), uiY(py), pw * uiScale, ph * uiScale, ph / 2.0 * uiScale);
  fill(on ? color(255) : THEME_MUTED);
  textAlign(CENTER, CENTER);
  hudText(on ? "ON" : "OFF", px + pw / 2.0, py + ph / 2.0);

  // the multiplier slider, under the switch, while the heat-map is switched on
  if (on) drawLoadGainSlider();

  // the color key, under the slider, while the heat-map is really showing
  if (loadColorsActive()) {
    float ky = LOADGAIN_Y + LOADGAIN_H + 4, kh = 34;
    themePanel(x, ky, w, kh);
    float bx = x + pad, by = ky + 7, bw = w - 2 * pad, bh = 9;
    int steps = 40;
    for (int s = 0; s < steps; s++) {
      fill(loadColor(100.0 * s / (steps - 1)));
      rect(uiX(bx + bw * s / steps), uiY(by), bw / steps * uiScale + 1, bh * uiScale);
    }
    textFont(hudFont);
    hudTextSize(10);
    fill(THEME_MUTED);
    // labels in REAL load: with gain g, red = 100/g % of full torque
    textAlign(LEFT, TOP);
    hudText("0%", bx, by + bh + 1);
    textAlign(CENTER, TOP);
    hudText(loadPctText(50 / LOAD_COLOR_GAIN) + " of full torque", bx + bw / 2, by + bh + 1);
    textAlign(RIGHT, TOP);
    hudText(loadPctText(100 / LOAD_COLOR_GAIN), bx + bw, by + bh + 1);
  }
  popStyle();
}

// ---- GAIN slider (under the LOAD COLORS switch) ----
final float LOADGAIN_Y = LOADSW_Y + LOADSW_H + 4, LOADGAIN_H = 30;
final float LOADGAIN_TRACK_X0 = LOADSW_X + 46, LOADGAIN_TRACK_X1 = LOADSW_X + LOADSW_W - 50;
boolean loadGainDragging = false;

boolean loadGainMouseOver() {
  return loadSwitchVisible() && LOAD_COLORS_3D
      && mouseX >= uiX(LOADSW_X) && mouseX <= uiX(LOADSW_X + LOADSW_W)
      && mouseY >= uiY(LOADGAIN_Y) && mouseY <= uiY(LOADGAIN_Y + LOADGAIN_H);
}

// Slider position 0..1 <-> gain 1..LOAD_GAIN_MAX, logarithmic (fine control near 1x).
float loadGainToPos(float g) { return log(constrain(g, 1, LOAD_GAIN_MAX)) / log(LOAD_GAIN_MAX); }
float loadPosToGain(float t) { return exp(constrain(t, 0, 1) * log(LOAD_GAIN_MAX)); }

void loadGainSetFromMouse() {
  float lx = (mouseX - uiOffsetX) / uiScale;
  float t = (lx - LOADGAIN_TRACK_X0) / (LOADGAIN_TRACK_X1 - LOADGAIN_TRACK_X0);
  float g = loadPosToGain(t);
  LOAD_COLOR_GAIN = g < 1.05 ? 1 : (g < 10 ? round(g * 10) / 10.0 : round(g)); // snap to 1x, tidy steps
}

void loadGainMouseDragged() { if (loadGainDragging) loadGainSetFromMouse(); }
void loadGainMouseReleased() {
  if (loadGainDragging) println("3D load color gain: x" + loadGainText());
  loadGainDragging = false;
}

String loadGainText() { return LOAD_COLOR_GAIN < 10 ? nf(LOAD_COLOR_GAIN, 1, 1) : str(round(LOAD_COLOR_GAIN)); }
String loadPctText(float pct) { return (pct < 10 ? nf(pct, 1, 1) : str(round(pct))) + "%"; }

void drawLoadGainSlider() {
  float x = LOADSW_X, y = LOADGAIN_Y, w = LOADSW_W, h = LOADGAIN_H, pad = 8;
  themePanel(x, y, w, h);
  float cy = y + h / 2.0;
  textFont(uiBoldFont(max(7, round(10 * uiScale))));
  textAlign(LEFT, CENTER);
  fill(THEME_TEXT);
  hudText("GAIN", x + pad, cy);
  // track, filled with the color scale up to the knob
  float x0 = LOADGAIN_TRACK_X0, x1 = LOADGAIN_TRACK_X1, th = 6;
  float t = loadGainToPos(LOAD_COLOR_GAIN), kx = x0 + (x1 - x0) * t;
  noStroke();
  fill(THEME_CTRL_BG);
  rect(uiX(x0), uiY(cy - th / 2), (x1 - x0) * uiScale, th * uiScale, th / 2 * uiScale);
  fill(loadColor(100 * t));
  rect(uiX(x0), uiY(cy - th / 2), (kx - x0) * uiScale, th * uiScale, th / 2 * uiScale);
  // knob
  boolean hot = loadGainDragging || loadGainMouseOver();
  fill(255);
  stroke(0, 150);
  strokeWeight(1);
  ellipse(uiX(kx), uiY(cy), (hot ? 14 : 12) * uiScale, (hot ? 14 : 12) * uiScale);
  noStroke();
  // value
  textAlign(RIGHT, CENTER);
  fill(LOAD_COLOR_GAIN > 1 ? THEME_YELLOW : THEME_TEXT);
  hudText("x" + loadGainText(), x + w - pad, cy);
}
