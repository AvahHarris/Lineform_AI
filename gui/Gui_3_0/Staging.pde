// =====================================================================
//  CREATE SHAPE GATE FOR NEW ITEMS (2026-11-24)
//
//  Only two things wait for Create Shape:
//    1. a NEW animation - made by the Motion agent (or the old prompt box)
//       or loaded with Load Animation. Even if another animation is
//       playing, that one keeps playing until Create Shape starts the new one.
//    2. a NEWLY LOADED shape file - its pose, torque and stiffness all go to
//       the arm on Create Shape; until then it is only the ghost arm (the
//       bars, the solid arm and a playing animation are left alone).
//    3. a NEW POSE from the Designer (or an adjusted loaded shape): ghost
//       preview until Create Shape puts it on the bars and sends it.
//  Everything else is instant:
//    * torque / stiffness changes (bars, agents, Settings Critic)
//    * changes to the animation that is playing (adjust_loaded_animation)
//  Changes to a new item that is still waiting update the waiting item.
//  Reset drops anything waiting.
//
//  PREVIEW: while something waits, the 3D view draws it as a translucent
//  green GHOST arm of motor blocks (drawStagedPreview, called from
//  drawlineform3d) - a waiting animation plays on the ghost in a loop. The
//  solid arm keeps showing the REAL motors (live angles, load colors),
//  and the message corner shows a pinned "READY - click Create Shape" note.
// =====================================================================

// ---- waiting new animation ----
AIAnimationResult stagedAnim = null;
int[] stagedAnimTorque = null;   // torque while it plays (null = keep)

// ---- waiting newly loaded shape ----
boolean stagedShape = false;
String stagedShapeName = "";
float[] stagedShapeAngles = null; // the pose, put on the bars at load (or on Create Shape if an animation plays)
boolean stagedShapeBarsPending = false; // true = not on the bars yet (always, since 2026-11-24)
boolean stagedShapeIsAI = false;        // a Designer shape: keep its AI preview / Shape Size state on Create Shape
int[] stagedShapeTorque = null, stagedShapeStiffness = null;

// Torque / stiffness: sent to the motors right away (drawing thread).
void applyJointSettingsNow(int[] torque, int[] stiffness) {
  if (torque != null) motorsSetTorqueAll(torque);
  if (stiffness != null) aiApplyStiffnessLevels(stiffness);
}

void stagedClear() {
  stagedAnim = null;
  stagedAnimTorque = null;
  stagedShape = false;
  stagedShapeAngles = null;
  stagedShapeBarsPending = false;
  stagedShapeTorque = null;
  stagedShapeStiffness = null;
  stagedShapeIsAI = false;
}

// 1. A NEW animation (drawing thread): always waits for Create Shape.
void stageAnimation(AIAnimationResult a, int[] torque) {
  if (a == null) return;
  stagedClear(); // replaces anything else that was waiting
  if (!a.isTravelingWave && a.recording == null && a.solvedPoses == null) stagedSolvePoses(a); // for the preview (and reused on Create Shape)
  stagedAnim = a;
  stagedAnimTorque = torque;
  stagedPreviewStartMs = millis();
}

// 2. A NEWLY LOADED shape (drawing thread): waits for Create Shape.
void stageLoadedShape(String name, float[] angles, int[] torque, int[] stiffness) {
  stagedClear();
  stagedShape = true;
  stagedShapeName = name;
  stagedShapeAngles = angles.clone();
  stagedShapeTorque = torque;
  stagedShapeStiffness = stiffness;
  // Bars and the real arm's 3D view stay as they are until Create Shape: the
  // loaded pose is shown as the green ghost arm (so the solid arm keeps
  // showing the real motors, with their live load colors).
  stagedShapeBarsPending = true;
}

boolean stagedShapeWaiting(String name) { return stagedShape && stagedShapeName.equals(name); }

void stagedShowShapeOnBars() {
  arrayCopy(stagedShapeAngles, generatedAngles);
  if (stagedShapeIsAI) { // Designer shape: its preview curve / Shape Size are already set up
    applyGeneratedAnglesToSliders();
    stagedShapeBarsPending = false;
    return;
  }
  selectedShape = stagedShapeName;
  aiHasPreview = false;
  aiPreviewCurve.clear();
  aiShapeHasLoop = false;
  aiLoopAttachMotor = -1;
  aiShapeIsCurved = false;
  Controller ss = cp5.getController("Shape Size"); // 0.5 = exactly the saved pose
  ss.setBroadcast(false);
  ss.setValue(0.5);
  ss.setLock(false);
  ss.setBroadcast(true);
  shapeSize = 0.5;
  applyGeneratedAnglesToSliders();
  stagedShapeBarsPending = false;
}

// A change to a waiting loaded shape updates the waiting one (agents' adjust tool).
void stagedUpdateShape(float[] angles, int[] torque, int[] stiffness) {
  if (!stagedShape) return;
  stagedShapeAngles = angles.clone();
  if (torque != null) stagedShapeTorque = torque.clone();
  if (stiffness != null) stagedShapeStiffness = stiffness.clone();
  if (!stagedShapeBarsPending) stagedShowShapeOnBars();
}

// 3. A Designer shape (or an adjusted loaded shape): its POSE waits as the
// ghost until Create Shape; torque / stiffness changes stay instant.
// Drops anything else waiting. (drawing thread)
void stageShapePose(String name, float[] angles, boolean isAI) {
  stagedClear();
  stagedShape = true;
  stagedShapeName = name;
  stagedShapeAngles = angles.clone();
  stagedShapeIsAI = isAI;
  stagedShapeBarsPending = true;
}

// Runs a solver step that writes the angle bars (aiGenerateFromSegments)
// and puts the bars back afterwards, so the solid arm keeps showing the
// real motors. (drawing thread)
void stagedWithBarsKept(Runnable solve) {
  float[] bars = new float[MOTOR_COUNT];
  for (int i = 0; i < MOTOR_COUNT; i++) bars[i] = cp5.getController("Motor " + (i + 2)).getValue();
  solve.run();
  updatingShapeControls = true;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Controller c = cp5.getController("Motor " + (i + 2));
    c.setBroadcast(false);
    c.setValue(bars[i]);
    c.setBroadcast(true);
  }
  updatingShapeControls = false;
}

// The Designer put a NEW shape on the bars: a waiting item is dropped
// (else Create Shape would start that instead).
void stagedNewShape() {
  if (stagedAnim != null) noticeAdd("status", "The ready animation \"" + stagedAnim.animationName + "\" was dropped - a new shape is on the bars.");
  stagedClear();
}

void startAnimationNow(AIAnimationResult a, int[] torque) {
  aiStartAnimation(a);
  if (!aiAnimating) return; // refused (e.g. Manipulation mode)
  if (a.recording != null) animRecAppliedFrame = -1;
  if (torque != null) motorsApplyAnimationTorque(torque); // remembers the torque from before, restored on Stop
}

// Called by Create Shape (drawing thread). Returns true if it started a
// waiting animation (then the normal "send the bars" step is skipped).
boolean stagedApplyOnCreateShape() {
  if (stagedAnim != null) {
    AIAnimationResult a = stagedAnim;
    int[] tq = stagedAnimTorque;
    stagedClear();
    startAnimationNow(a, tq);
    if (aiAnimating) noticeAdd("status", "Playing \"" + a.animationName + "\". Stop Animation ends it.");
    return true;
  }
  if (stagedShape) {
    if (stagedShapeBarsPending) { aiStopAnimation(); stagedShowShapeOnBars(); }
    applyJointSettingsNow(stagedShapeTorque, stagedShapeStiffness);
    stagedClear();
  }
  return false; // Create Shape now sends the bars as usual
}

// For the agents' arm-state note.
String stagedNoteForAgents() {
  if (stagedAnim != null) return "; READY but NOT started yet: the new animation '" + stagedAnim.animationName
    + "' (starts when the user presses Create Shape)";
  if (stagedShape) return "; the shape '" + stagedShapeName + "' is WAITING (shown as a ghost preview): it goes to the "
    + "arm when the user presses Create Shape";
  return "";
}

// ---------------- PREVIEW ----------------
int stagedPreviewStartMs = 0;
final color PREVIEW_COLOR = color(80, 255, 140);

// Solves a pose animation's poses into joint angles WITHOUT touching the
// bars / current shape (aiGenerateFromSegments writes those, so they are
// saved and put back). aiStartAnimation() then reuses result.solvedPoses.
void stagedSolvePoses(AIAnimationResult a) {
  float[] bars = new float[MOTOR_COUNT];
  for (int i = 0; i < MOTOR_COUNT; i++) bars[i] = cp5.getController("Motor " + (i + 2)).getValue();
  float[] gen = generatedAngles.clone(), sm = aiSmallAngles.clone(), lg = aiLargeAngles.clone();
  String sel = selectedShape;
  boolean hasPrev = aiHasPreview, loop = aiShapeHasLoop, curved = aiShapeIsCurved;
  int attach = aiLoopAttachMotor;
  ArrayList<PVector> prev = new ArrayList<PVector>(aiPreviewCurve);
  float fit = aiLastFitQuality, size = shapeSize;
  Controller ss = cp5.getController("Shape Size");
  float ssVal = ss.getValue();
  boolean ssLock = ss.isLock();

  a.solvedPoses = new ArrayList<float[]>();
  a.solvedFits = new ArrayList<Float>();
  for (int i = 0; i < a.poses.size(); i++) {
    AIAnimationPose pose = a.poses.get(i);
    aiGenerateFromSegments(pose.segments, pose.closed, a.animationName + " - " + pose.poseName);
    a.solvedPoses.add(generatedAngles.clone());
    a.solvedFits.add(aiLastFitQuality);
  }

  arrayCopy(gen, generatedAngles);
  arrayCopy(sm, aiSmallAngles);
  arrayCopy(lg, aiLargeAngles);
  selectedShape = sel;
  aiHasPreview = hasPrev;
  aiShapeHasLoop = loop;
  aiShapeIsCurved = curved;
  aiLoopAttachMotor = attach;
  aiPreviewCurve.clear();
  aiPreviewCurve.addAll(prev);
  aiLastFitQuality = fit;
  shapeSize = size;
  ss.setBroadcast(false);
  ss.setValue(ssVal);
  ss.setLock(ssLock);
  ss.setBroadcast(true);
  updatingShapeControls = true;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Controller c = cp5.getController("Motor " + (i + 2));
    c.setBroadcast(false);
    c.setValue(bars[i]);
    c.setBroadcast(true);
  }
  updatingShapeControls = false;
}

// Joint angles of the waiting item right now (null = nothing to preview).
float[] stagedPreviewAngles() {
  int t = millis() - stagedPreviewStartMs;
  if (stagedAnim != null) {
    AIAnimationResult a = stagedAnim;
    if (a.recording != null) return animRecordingAnglesAt(a.recording, t);
    if (a.isTravelingWave) {
      float phase = t / max(150, a.wavePeriodMs);
      float[] w = aiSampleTravelingWave(a.waveAmplitudeDeg, a.waveWavelengthJoints, phase, a.waveDirection);
      float drift = AI_ANIM_WAVE_DRIFT_DEG * sin(TWO_PI * t / AI_ANIM_WAVE_DRIFT_PERIOD_MS);
      w[0] = clampAI(w[0] + drift);
      w[1] = clampAI(w[1] + drift * 0.6);
      return w;
    }
    if (a.solvedPoses != null && a.solvedPoses.size() >= 2) {
      float loopMs = max(150, a.frameDurationMs) * a.solvedPoses.size();
      return aiSampleAnimationLoop(a.solvedPoses, (t % loopMs) / loopMs, AI_ANIM_SPLINE_TENSION);
    }
    return null;
  }
  if (stagedShape) return stagedShapeAngles; // a waiting loaded shape: ghost until Create Shape
  return null;
}

// Drawn by drawlineform3d() (world space): the waiting item as a GHOST ARM -
// translucent green motor blocks, same geometry as the real arm, which keeps
// showing the real motors (live angles + load colors) until Create Shape.
void drawStagedPreview() {
  float[] ang = stagedPreviewAngles();
  if (ang == null) return;
  pushStyle();
  hint(DISABLE_DEPTH_MASK); // see-through: never hides the real arm behind it
  pushMatrix();
  translate(CHAIN_ORIGIN.x, CHAIN_ORIGIN.y, CHAIN_ORIGIN.z);
  color ghost = color(red(PREVIEW_COLOR), green(PREVIEW_COLOR), blue(PREVIEW_COLOR), 70);
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int motorNumber = i + 1;
    boolean fitted = i < ACTIVE_MOTORS;
    float angle = fitted ? constrain(ang[i], 0, 360) - 180 : 0;
    if (aiIsMirroredMotor(motorNumber + 1)) angle = -angle;
    float rad = radians(angle);
    if (motorNumber % 2 == 1) rotateZ(-rad);
    else rotateY(rad);
    pushMatrix();
    translate(LINK_BOX_CENTER_X, 0, 0);
    drawLinkBlock(motorNumber, ghost, "", color(255, 0)); // no number on the ghost
    popMatrix();
    translate(LINK_STEP, 0, 0);
  }
  popMatrix();
  hint(ENABLE_DEPTH_MASK);
  popStyle();
}

// The pinned message-corner note (Notices.pde), or null.
String stagedReadyMessage() {
  if (stagedAnim != null) return "READY: the animation \"" + stagedAnim.animationName + "\" is prepared - click Create Shape to play it."
    + (aiAnimating ? " (The current animation keeps playing until then.)" : "") + " The green ghost arm shows a preview.";
  if (stagedShape) return "READY: the " + (stagedShapeIsAI ? "" : "loaded ") + "shape \"" + stagedShapeName + "\" is prepared - click Create Shape to send it"
    + (stagedShapeTorque != null || stagedShapeStiffness != null ? " with its torque / stiffness" : "") + "."
    + " The green ghost arm shows it.";
  return null;
}
