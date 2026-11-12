import controlP5.*;
import processing.serial.*;
import java.net.HttpURLConnection;
import java.net.URL;
import java.io.*;
import java.util.Set;


import peasy.PeasyCam;
import peasy.CameraState;


PeasyCam cam;

// Every AI-generation diagnostic line (segment dump, [repair] lines, fit
// quality) also gets written here, in addition to the console - the
// console's scrollback is easy to lose or lose track of mid-conversation;
// this file always has the full history to just open and copy from.
PrintWriter aiLogFile;

void aiLog(String s) {
  println(s);
  if (aiLogFile != null) {
    aiLogFile.println(s);
    aiLogFile.flush();
  }
}

ControlP5 cp5;
processing.serial.Serial myPort;
// Simulation mode does not require any physical Arduino or USB serial port.
// false = talk to the real Arduino over USB.
final boolean SIMULATION_MODE = false;
// Leave as "" to auto-detect the Arduino (Mac: /dev/cu.usbmodem... or
// /dev/cu.usbserial...). If auto-detect picks the wrong one, paste the exact
// name from the "Available serial ports" list in the console here instead.
final String ARDUINO_PORT = "";

// Picks the Arduino's port: ARDUINO_PORT if set, otherwise the first port
// that looks like a USB Arduino, otherwise null.
String findArduinoPort() {
  if (ARDUINO_PORT != null && ARDUINO_PORT.length() > 0) return ARDUINO_PORT;
  String[] ports = processing.serial.Serial.list();
  // Prefer the Mac "cu." ports - "tty." versions can hang when opened.
  for (String p : ports) if (p.startsWith("/dev/cu.usbmodem")) return p;
  for (String p : ports) if (p.startsWith("/dev/cu.usbserial")) return p;
  for (String p : ports) if (p.startsWith("/dev/cu.wchusbserial")) return p;
  for (String p : ports) if (p.startsWith("/dev/ttyACM") || p.startsWith("/dev/ttyUSB")) return p; // Linux
  for (String p : ports) if (p.startsWith("COM")) return p; // Windows
  return null;
}
boolean serialConnected() { return !SIMULATION_MODE && myPort != null; }
// Motor numbering. Inside this sketch the rows are named "Motor 2".."Motor 12"
// from the base to the tip (internal names only). Two numbers per row:
//
//   displayId(row) - what the USER sees (slider/torque labels, 3D box numbers,
//                    hinge tags): 1 next to the base ... 11 at the tip.
//   hwId(row)      - the REAL Dynamixel ID that every serial command uses.
//
// On this arm they are the same: Dynamixel ID 1 is next to the base, counting
// up to ID 11 at the tip. If the motors are ever renumbered, change hwId()
// only - the labels stay base = 1.
int displayId(int rowId) {
  return rowId - 1;
}

int hwId(int rowId) {
  return rowId - 1;
}

// ---- Per-device settings (local_config.json, git-ignored) ----
// Different arms have different numbers of motors. The GUI always has
// MOTOR_COUNT (11) rows/sliders and the AI always plans for 11 joints, but
// only the first ACTIVE_MOTORS rows (base -> tip) are real motors on THIS
// arm: inactive rows never send commands to the hardware, are skipped by
// live reading, and are drawn grey and straight on the canvas.
//   active_motors     - how many real motors this arm has (default 11)
//   base_block_first  - true: draw the fixed "12" block before motor 1 (at
//                       the base); false: draw it after the last motor
//   hand_move_threshold_deg / hand_move_confirm_readings - how big and how
//                       sustained a push must be to count as a hand move
//                       (defaults 5 / 1; raise for softer or twitchy motors)
// Copy local_config.example.json to local_config.json to change these.
int ACTIVE_MOTORS = 11; // = MOTOR_COUNT (declared further down, so not referenced here)
boolean BASE_BLOCK_FIRST = true;

void loadLocalConfig() {
  File f = new File(sketchPath("local_config.json"));
  if (!f.exists()) return;
  try {
    JSONObject c = loadJSONObject(f.getAbsolutePath());
    if (c.hasKey("active_motors")) ACTIVE_MOTORS = constrain(c.getInt("active_motors"), 1, MOTOR_COUNT);
    if (c.hasKey("base_block_first")) BASE_BLOCK_FIRST = c.getBoolean("base_block_first");
    if (c.hasKey("hand_move_threshold_deg")) LIVE_FOLLOW_DEG = max(1, c.getFloat("hand_move_threshold_deg"));
    if (c.hasKey("hand_move_confirm_readings")) LIVE_FOLLOW_CONFIRM = max(1, c.getInt("hand_move_confirm_readings"));
    println("local_config.json: " + ACTIVE_MOTORS + " active motors, base block "
      + (BASE_BLOCK_FIRST ? "before motor 1" : "after the last motor")
      + ", hand move > " + LIVE_FOLLOW_DEG + " deg for " + LIVE_FOLLOW_CONFIRM + " reading(s)");
  } catch (Exception e) {
    println("Couldn't read local_config.json (" + e.getMessage() + ") - using defaults.");
  }
}

// Is this row ("Motor 2".."Motor 12" internal id) a real motor on this arm?
boolean rowActive(int rowId) {
  int i = rowId - 2;
  return i >= 0 && i < ACTIVE_MOTORS;
}

synchronized void sendSerialBytes(int... bytes) {
  if (!serialConnected()) return;
  for (int b : bytes) myPort.write(b);
}

// ---- Release the motors when the GUI closes ----
// true  = closing the GUI window turns torque OFF on all motors (arm goes limp).
// false = closing the GUI leaves the motors as they are (holding their last pose).
// Closing the window (X button or Esc) runs this. The firmware turns torque
// back ON for every motor when the GUI reconnects (opening the port resets
// the Arduino), so each session still starts with the motors holding.
// Note: Processing's own Stop button can kill the program before this runs -
// close the GUI window itself to be sure the motors get released.
// Ported from lineform-ai-backup/Gui_07_07.
final boolean RELEASE_MOTORS_ON_EXIT = true;

// Torque ON/OFF for every motor (hardware IDs 1..MOTOR_COUNT), using the
// firmware's TORQUE_HEADER (254) command - same as flipping every torque switch.
void setAllMotorTorque(boolean on) {
  for (int rowId = 2; rowId <= MOTOR_COUNT + 1; rowId++) {
    if (rowActive(rowId)) sendSerialBytes(254, hwId(rowId), on ? 1 : 0);
  }
}

boolean motorsReleasedOnExit = false;

// synchronized (same lock as sendSerialBytes / the serial readers) so the
// live-reading thread can't be mid-command when the port is closed.
synchronized void releaseMotorsOnExit() {
  if (!RELEASE_MOTORS_ON_EXIT || motorsReleasedOnExit || !serialConnected()) return;
  motorsReleasedOnExit = true;
  aiAnimating = false; // stop the animation player from sending more poses
  println("GUI closing - turning torque OFF on all motors.");
  try {
    setAllMotorTorque(false);
    Thread.sleep(300); // let the bytes reach the Arduino before the port closes
    myPort.stop();
    myPort = null;     // background threads now see "not connected" and stop sending
  }
  catch (Exception e) {
    println("Couldn't release motors on exit: " + e.getMessage());
  }
}

void exit() {
  releaseMotorsOnExit();
  super.exit();
}

void dispose() {
  releaseMotorsOnExit();
  super.dispose();
}
Toggle toggle1;
Toggle toggle2;
Toggle toggle3;
Toggle toggle4;
Toggle toggle5;
Toggle toggle6;
Toggle toggle7;
Toggle toggle8;
Toggle toggle9;
Toggle toggle10;
Toggle toggle11;

final int MOTOR_COUNT = 11;

// ---- Motor row layout - shared by setup() (builds the controls) and
// draw() (positions the ON/OFF labels) so they can't drift out of sync. ----
final int SLIDER_X = 20;
final int SLIDER_W = 170;
final int SLIDER_H = 24;
final int TOGGLE_X = 280;
final int TOGGLE_W = 60;
final int TOGGLE_H = 22;
// Torque ON/OFF switch per row (as in the backup GUI), right of the
// ANGLE/TORQUE switch's label.
final int POWER_X = 420;
final int POWER_W = 44;
final int ROW_STEP = 36;
final int ROW_START_Y = 20;

// ---- 3D chain geometry - shared by drawlineform3d() (renders it) and
// aiComputeAchievableCurve() (predicts joint positions so the AI preview
// curve can be aligned to them) so the two can never drift out of sync. ----
final float LINK_GAP = 9;
final float LINK_WIDTH = 52;
final float LINK_HEIGHT = 36;
final float LINK_HINGE = LINK_WIDTH * 0.9;
final PVector CHAIN_ORIGIN = new PVector(1050, 360, -100);

// Which hardware Dynamixel IDs are physically mounted mirrored (their
// positive-angle direction is flipped relative to the others) - shared by
// drawlineform3d() (renders it), aiAnglesFromCurve(),
// aiComputeAchievableCurve(), and aiSolveAnglesForCurve() so they can't
// drift out of sync with each other. Takes the real hardware ID (2-12)
// directly - matching what's printed on the sliders and the 3D boxes - so
// there's no position/ID translation step left to get confused by.
// Determined empirically on the real robot - re-derive by testing each
// motor individually if the hardware changes again.
//
// 2026-09-15: this was a stub returning false for every ID (leftover from
// the passive-block-to-front rewiring - the old lineform build's mirrored
// set was never re-derived for the new dxlId numbering). Briefly set to
// mirror dxlId 3 and 5 after they were reported spinning backwards, then
// switched back to unmirrored on further hardware testing - so as of this
// version NOTHING is flagged as mirrored again. IMPORTANT: this flag only
// affects the 3D preview and the angles aiSolveAnglesForCurve()/
// aiAnglesFromCurve() choose for an AI-GENERATED shape - it does NOT touch
// what gets sent over serial for a manually-dragged slider (Create Shape
// writes each slider's raw value straight out, unmirrored, always has). If
// 3/5 are still backwards specifically when hand-driving sliders, this is
// the wrong place to fix it - see the dxlId loop in the "Create Shape"
// handler below, or the Arduino firmware's per-ID direction handling.
boolean aiIsMirroredMotor(int dxlId) {
  return false;
}

// ---- AI shape generator (calls Claude directly, no separate server) ----
String ANTHROPIC_API_URL = "https://api.anthropic.com/v1/messages";
String ANTHROPIC_MODEL   = "claude-opus-5"; // more capable spatial reasoning than Sonnet, costs more per call
String ANTHROPIC_API_KEY; // loaded in setup() - see loadApiKey()

// How far a single joint may bend off its 180 deg neutral. Was capped at 90,
// well inside the arm's actual mechanical safety range (sliders are allowed
// 60-310, i.e. up to +/-120-130 off neutral elsewhere in this file) - that
// unused headroom meant tight curvature (a loop closing back on itself, a
// wave's peaks/troughs) couldn't be reached: the solver would clamp several
// joints in a row right at the boundary and the achievable curve would fall
// short instead of completing the bend. Raised to match the tighter of the
// two real bounds (310-180=130, 180-60=120) so the solver can use nearly all
// of the same range that was already established as safe.
float AI_MAX_TURN_DEG = 120;

// ---- AI shape preview overlay ----
// The AI's ideal spline curve, rescaled and recentered onto the achievable
// joint curve's own centroid/extent (computed by aiComputeAchievableCurve),
// so it can be drawn alongside the real chain for a visual sanity check
// before "Create Shape" sends anything to the motors.
ArrayList<PVector> aiPreviewCurve = new ArrayList<PVector>();
boolean aiHasPreview = false;

// Toggled by the "Show Hinges" switch in the GUI. Draws a small marker +
// label at each of the 11 ACTIVE joints' actual pivot point (not the box
// center - see drawlineform3d.pde) so a shape can be designed by hand with
// a clear reference for exactly where each bend happens, independent of
// whatever shape/overlay is currently showing. Purely visual: changing this
// never touches a servo angle or the segment DSL.
boolean showHingeMarkers = true;

// Fine-tune offset for the hinge markers, in the same units as LINK_GAP/
// LINK_WIDTH/LINK_HINGE/LINK_HEIGHT. drawlineform3d.pde's joint loop was
// tuned for how the LINKS look, not for exactly where each real Dynamixel's
// rotation shaft sits inside that geometry - this offset is applied once,
// uniformly, to every joint's marker (in that joint's own local frame,
// right after its rotation and before anything else) to dial the marker
// onto the real shaft. X = forward/back along the chain at this joint,
// Y = sideways, Z = up/down. Adjustable live with arrow keys ([ and ] for
// Y) while comparing the on-screen marker against the physical rig -
// current values are echoed in the HUD and the console on every change.
// Not `final`, on purpose.
float hingeOffsetX = 0;
float hingeOffsetY = 0;
float hingeOffsetZ = 0;

// Set by mousePressed() when a click lands in the open 3D viewport (not on
// the ControlP5 panel); resolved against this frame's actual hinge marker
// screen positions inside drawlineform3d(), then cleared. Has to be
// resolved there and not inside mousePressed() itself, because screenX()/
// screenY() depend on PeasyCam's camera transform, which is only active
// while a draw() frame is actually rendering the 3D scene - a mouse event
// callback has no such transform to read from.
boolean hingeClickPending = false;
int hingeClickX = 0;
int hingeClickY = 0;
final float HINGE_CLICK_RADIUS_BASE = 18; // pixels at uiScale 1.0
float HINGE_CLICK_RADIUS = 18; // pixels - how close a tap must land; rescaled in layoutResponsiveGUI()

// Tapping a hinge marker cycles that joint through a small ladder of
// Position P Gain ("stiffness") presets instead of toggling torque - each
// tap feels softer, wrapping back to full stiffness after the last level.
// Index = position (1-11) - 1, i.e. hingeStiffnessLevel[0] is motor/hinge
// 2's level. STIFFNESS_P_GAIN_DISPLAY is for the HUD/console only - the
// Arduino firmware owns the real lookup table (STIFFNESS_P_GAINS in the
// .ino) and these two arrays have to be kept in sync by hand.
final int NUM_STIFFNESS_LEVELS = 4;
final int[] STIFFNESS_P_GAIN_DISPLAY = {1200, 700, 350, 120};
final byte STIFFNESS_HEADER = (byte) 252;
int[] hingeStiffnessLevel = new int[MOTOR_COUNT]; // all start at level 0 = firmware's default P gain

// Sends a hinge's new stiffness LEVEL (not a raw gain value) to the
// Arduino - STIFFNESS_HEADER=252, matching the .ino firmware, which looks
// the level up in its own STIFFNESS_P_GAINS table and writes it straight
// to Position P Gain (address 84) with torque left on.
void sendHingeStiffness(int dxlId, int level) {
  if (!rowActive(dxlId)) return;
  sendSerialBytes(252, hwId(dxlId), level);
}

// Pushes a full 11-length stiffness-LEVEL array (index 0 = hinge 2 ...
// index 10 = hinge 12, same ordering as hingeStiffnessLevel itself) out to
// the real hardware, skipping any joint that's already at the requested
// level so this is cheap to call every time an animation starts or stops.
// Shared by triggerAIAnimation (apply whatever feel the AI chose for this
// motion) and aiStopAnimation (restore every joint to full stiffness
// afterward, so a softened joint never lingers into manual slider use).
// "[0, 0, 1, 2, ...]" - just for log readability, nothing reads this back.
String aiJoinIntArray(int[] values) {
  String out = "[";
  for (int i = 0; i < values.length; i++) {
    if (i > 0) out += ", ";
    out += values[i];
  }
  return out + "]";
}

void aiApplyStiffnessLevels(int[] levels) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int level = constrain(levels[i], 0, NUM_STIFFNESS_LEVELS - 1);
    if (hingeStiffnessLevel[i] != level) {
      int dxlId = i + 2;
      hingeStiffnessLevel[i] = level;
      sendHingeStiffness(dxlId, level);
    }
  }
}

// Which motor (1-11) each corner in aiCorners lands on - the running total
// of joint counts through the segment ending at that corner (computed in
// aiGenerateFromSegments right after aiAllocateJointCounts). Lets
// aiGenerateFromSegments() (and, through aiLoopAttachMotor below, the
// renderer) know exactly which physical joint is "supposed to be" where a
// loop/spiral head branches off its handle.
int[] aiCornerAnchorMotors = new int[0];

// Set when the most recently generated AI shape has a LOOP/SPIRAL head, so
// the renderer can draw a stroke closing it back to its attach point on the
// handle - the arm is a single OPEN serial chain (fixed base, free tip), so
// the last joint can get close to that point but can never truly touch it;
// this is a visual closure, not a new physical capability, but it's what
// makes a "closed loop" shape actually read as closed/connected on screen.
boolean aiShapeHasLoop = false;
int aiLoopAttachMotor = -1; // 1-11 position index, or -1 if aiShapeHasLoop is false

// True when the most recently generated AI shape includes an ARC/LOOP/
// SPIRAL segment (a genuinely curved shape) rather than only LINE segments
// (a polygon/tool shape like a square or hammer, where the real joints
// SHOULD look sharp-cornered). The achievable curve is always just 11
// straight rigid links drawn corner to corner, so a smooth math curve
// still renders as a jagged, faceted polyline - this flag lets the
// renderer draw a smoothed spline through those same joint positions
// instead of straight segments for curved shapes, without touching any
// actual servo angle, while leaving genuinely sharp-cornered shapes alone.
boolean aiShapeIsCurved = false;

// Small/large angle sets for the most recently generated AI shape - a
// tighter and a more exaggerated version of the same bend pattern, derived
// by scaling each joint's deviation from 180 by AI_SIZE_MIN/MAX_SCALE.
// generateShapeAngles() blends between them with the Shape Size slider,
// same pattern as the HOOK/SPIRAL small/large presets, so the slider keeps
// working after an AI shape is generated - and scales the shape itself
// rather than flattening it toward straight.
float[] aiSmallAngles = new float[MOTOR_COUNT];
float[] aiLargeAngles = new float[MOTOR_COUNT];
float AI_SIZE_MIN_SCALE = 0.4; // bend amplitude at Shape Size = 0.0 (smaller/tighter)
float AI_SIZE_MAX_SCALE = 1.6; // bend amplitude at Shape Size = 1.0 (bigger/more exaggerated)

// ANIMATE: a small ordered list of fully-solved 11-angle poses - each one
// built through the EXACT same segment->angle pipeline as any static AI
// shape (aiGenerateFromSegments), just snapshotted into this list instead
// of being the only pose on the sliders. Played back as ONE continuous
// closed loop through all of them (pose 1 -> 2 -> ... -> N -> 1 -> 2 ->
// ...forever) via aiSampleAnimationLoop()'s cyclic Catmull-Rom spline
// (see near aiCatmullRom/aiCatmullRomChain) - NOT the old two-pose-at-a-
// time lerp, which had to decelerate to a dead stop at every single pose.
// With a real neighboring pose on both sides, the spline keeps non-zero
// velocity straight through each one; with exactly 2 poses there's no
// third point to curve through, so it degenerates to the same eased
// back-and-forth as before (still correct - that IS what a 2-pose motion
// like a heartbeat should look like).
ArrayList<float[]> aiAnimPoses = new ArrayList<float[]>();
String aiAnimName = "";
boolean aiAnimating = false;
int aiAnimIndex = 0;              // which pose the loop is currently nearest to - DISPLAY ONLY, HUD text
int aiAnimLoopStartMillis = 0;    // millis() when the current playback loop began
int aiAnimFrameDurationMs = 900;  // ms per pose-to-pose span; one full loop = this * aiAnimPoses.size()
int aiAnimLastSendMillis = 0;
// Tension for aiSampleAnimationLoop()'s spline - same meaning and default
// as AI_CURVE_RENDER_TENSION, kept separate since animation feel may want
// different tuning than render smoothing later.
float AI_ANIM_SPLINE_TENSION = 0.5;

// TRAVELING WAVE playback - the alternative to aiAnimPoses above, for a
// motion that's fundamentally a sine wave propagating along the chain
// (snake/eel/whip/flag) rather than a few whole-body poses. When
// aiAnimIsWave is true, aiUpdateAnimation() computes every joint's angle
// directly from these 4 numbers each frame (see aiSampleTravelingWave) -
// aiAnimPoses is left empty and ignored.
boolean aiAnimIsWave = false;
float aiAnimWaveAmplitudeDeg = 25;    // degrees each joint swings from straight (180) at the wave's peak
float aiAnimWaveWavelengthJoints = 7; // joints per full sine cycle
float aiAnimWavePeriodMs = 1000;      // ms for the pattern to travel one full wavelength
float aiAnimWaveDirection = 1;        // +1 = base(joint1)->tip(joint11), -1 = reverse

// "Locomotion" layer, always on for every traveling_wave animation (not
// AI-controlled - see aiUpdateAnimation). The sine wave above makes the
// body undulate correctly, but on its own that reads as wiggling in a
// fixed envelope, not going anywhere - a real snake also continuously
// re-aims its whole body as it travels. This superimposes a slow sine
// drift onto the two joints closest to the fixed base (which set the
// whole chain's overall heading), MUCH slower than the wave's own period,
// so the entire body sweeps its heading left and right over several
// seconds while the fast wave keeps rippling down its length underneath -
// the closest a fixed-base arm can get to an actual snake's path curving
// as it crawls, rather than confined to one static plane.
float AI_ANIM_WAVE_DRIFT_DEG = 18;
float AI_ANIM_WAVE_DRIFT_PERIOD_MS = 7000;
// Minimum milliseconds between serial writes during animation. Every OTHER
// write path in this sketch (Create Shape, Reset) is a one-off: a human
// clicks a button, one 255+11-byte frame goes out, then nothing until the
// next click. There is NO acknowledgment/flow-control in this protocol -
// Processing never waits to hear that the Arduino actually finished driving
// all 11 Dynamixels before it's allowed to send the next frame - because
// that was never needed before. Animation is the first thing that sends
// continuously, and if frames arrive faster than the Arduino's loop() can
// drain them, bytes queue up in the serial RX buffer; once that overflows,
// bytes get silently dropped and every frame after that can desync (a
// stray data byte misread as header 254/torque-toggle, followed by bytes
// that parse as a real dxlId + state=0, is exactly how "the motors just
// turn off" happens with no firmware bug at all).
// The updated lineform-ai.ino fixes that: it writes all 11 goals in ONE
// sync-write bus packet (several times faster than 11 separate writes) and,
// if frames ever queue up, skips straight to the newest one - so frames can
// now stream at ~25 per second for smooth, continuous motion. (No data byte
// can ever look like a command header: angles top out at 215, headers are
// 248-255.) If a particular arm stutters, raise this (e.g. 60-80); with the
// OLD Arduino code, set it back to 300.
int AI_ANIM_SEND_INTERVAL_MS = 40;

// Fit-quality tracking for the critique-and-refine loop: aiLastRefineError
// is the raw (world-unit) total error from the best ICP iteration,
// aiLastFitQuality is that same error normalized by the achievable shape's
// own size (so it's roughly scale-independent - "how bad is the fit
// relative to the shape itself", not an absolute distance). Lower is
// better. Set by aiFabrikRefineAnglesForCurve / aiGenerateFromSegments each time a
// shape is generated. AI_REFINE_QUALITY_THRESHOLD is a rough, empirically
// chosen cutoff (from standalone testing, good fits landed under ~0.4-0.6)
// above which triggerAIShape() asks Claude for one refined attempt.
float aiLastRefineError = 0;
float aiLastFitQuality = 0;
float AI_REFINE_QUALITY_THRESHOLD = 0.7;

// How many EXTRA refinement API calls triggerAIShape() may make beyond the
// initial generation (so up to AI_MAX_REFINE_ROUNDS+1 attempts total) while
// fit quality stays above AI_REFINE_QUALITY_THRESHOLD. Each round feeds
// Claude the full history of prior attempts and their feedback, not just
// the immediately preceding one, so it can see whether its own previous
// correction actually helped. Whichever of ALL attempts scores best is what
// ends up on the sliders at the end - a later round is not assumed to be
// better, since a refinement can just as easily make the fit worse.
int AI_MAX_REFINE_ROUNDS = 2;

// Per-joint diagnostics from the most recent aiGenerateFromSegments() call -
// which single joint drifted furthest from its target, and which joints hit
// the arm's hard safety clamp (60/310) while solving. Set right after the
// FABRIK refine step, using the SAME final angles/targets that produced
// aiLastFitQuality, so refinement feedback can point at a specific part of
// the shape instead of only a single aggregate number.
float[] aiLastJointError = new float[MOTOR_COUNT];
boolean[] aiLastJointSaturated = new boolean[MOTOR_COUNT];
int aiLastWorstJoint = -1; // 1-based motor position (1..MOTOR_COUNT), or -1 if nothing generated yet

// How far apart Claude's OWN raw curve's start and end points are, as a
// fraction of that curve's own path length - only meaningful when the
// shape was declared closed=true. This exists because "closed" is just a
// label the code uses for alignment/scaling math (aiAlignCurveToWorld,
// aiPathLength) - nothing actually forces the assembled segments to
// geometrically return to their own starting point. A shape can score a
// great fit-quality while being an open hook/spiral wearing a "closed"
// label (or, worse, get a great score specifically BECAUSE it quietly
// switched to closed=false, since an open path is mechanically easier for
// the arm to trace faithfully than a genuinely closed one) - this number
// is what lets aiGenerateFromSegments() penalize that and
// aiBuildRefinementFeedback() name it directly instead of Claude only ever
// seeing a deceptively good aggregate score.
float aiLastClosureGap = 0;

// v7: alignment transform from the most recent aiAlignCurveToWorld() call,
// so world-space joint positions can be converted back into the AI's own
// segment coordinates for refinement feedback.
float aiAlignScale = 1;
PVector aiAlignAiAnchor = new PVector();
PVector aiAlignAchAnchor = new PVector();

// v7: per-joint "wanted vs. got" table (in the AI's own units) from the most
// recent aiGenerateFromSegments() call - sent back to Claude in refinement
// rounds so it can see exactly where its shape went wrong, for any shape.
StringBuilder aiLastJointReport = new StringBuilder();

// v8: let Claude route exact synonyms/translations of a saved shape to its
// calibrated preset (see aiBuildRequestBody). false = always generate fresh.
boolean AI_USE_LIBRARY_MATCH = true;

// v9: two-phase anchoring. Phase 1 fits every trial with the shape's start
// point on the fixed base. Only if EVERY base-anchored trial scores worse
// than AI_REFINE_QUALITY_THRESHOLD does phase 2 try anchoring the shape's
// center on the arm's center instead. Whichever single trial (either phase)
// has the lowest fit quality is what ends up on the sliders.
boolean AI_TRY_CENTER_ANCHOR = true;  // false = base anchoring only
int AI_CENTER_REFINE_ROUNDS = 1;      // extra Claude calls in phase 2 (0 = only re-fit existing trials, no extra API cost)
boolean aiAnchorAtCenter = false;     // current mode, read by aiAlignCurveToWorld()

PVector aiToAiUnits(PVector world) {
  return PVector.add(aiAlignAiAnchor, PVector.div(PVector.sub(world, aiAlignAchAnchor), max(aiAlignScale, 0.000001)));
}
float AI_CLOSURE_GAP_WEIGHT = 2.0; // how heavily an unclosed "closed:true" shape is penalized in aiLastFitQuality

// How much more heavily an error near the FIXED BASE (joint 1) counts than
// one at the free TIP (the last joint), both when FABRIK decides how hard
// to pull each joint toward its target every relaxation step, and when
// comparing outer FABRIK rounds to pick the best one. Errors compound
// OUTWARD from the base - a bad angle at joint 1 throws off the position of
// every joint after it, while a bad angle at joint 11 only mislocates
// itself - so biasing correction toward getting the base right first, even
// at the tip's expense, makes the arm's overall silhouette read as more
// correct, which is what actually catches the eye, rather than treating a
// base-joint error and a tip-joint error as equally bad. 1.0 = no bias
// (original equal-weighting behavior).
float AI_BASE_JOINT_WEIGHT = 2.5;

// Per-joint weights for joint index j (0 = nearest the fixed base,
// numMotors-1 = the free tip): linearly interpolated from
// AI_BASE_JOINT_WEIGHT down to 1.0, then rescaled so their MEAN is exactly
// 1.0 - that keeps the aggregate weighted error on roughly the same scale
// as the old unweighted sum (so AI_REFINE_QUALITY_THRESHOLD and the
// fit-history CSV stay meaningful), while still shifting emphasis toward
// the base.
float[] aiBuildJointWeights(int numMotors) {
  float[] w = new float[numMotors];
  float sum = 0;
  for (int j = 0; j < numMotors; j++) {
    float t = (numMotors <= 1) ? 0 : (float) j / (numMotors - 1);
    w[j] = lerp(AI_BASE_JOINT_WEIGHT, 1.0, t);
    sum += w[j];
  }
  float mean = sum / numMotors;
  for (int j = 0; j < numMotors; j++) w[j] /= mean;
  return w;
}

// Cross-run fit-quality comparison: the best (lowest) fit quality ever
// recorded for each shape name (lowercased), loaded from
// AI_FIT_HISTORY_FILENAME at startup and updated every generation - lets
// aiRecordFitHistory() tell you whether a new attempt at a shape you've
// asked for before is actually more accurate than last time, and lets the
// on-screen readout show "best ever" next to the current fit quality.
HashMap<String, Float> aiBestFitQualityEver = new HashMap<String, Float>();
String AI_FIT_HISTORY_FILENAME = "logs/ai_fit_history.csv";


// No shape is selected when the program starts.
String selectedShape = "none";

// 0.0 = small/tight
// 1.0 = large/wide
float shapeSize = 0.5;

// Stores the currently generated shape.
float[] generatedAngles = new float[MOTOR_COUNT];

// Prevents slider updates from triggering unwanted repeated events.
boolean updatingShapeControls = false;

// Calibrated preset shapes, tuned directly on the real robot (Motor 2 ..
// Motor 12, in order), loaded from shape_library.json next to this sketch
// instead of being hardcoded here. Each entry has a name, a list of prompt
// synonyms that match straight to it (no AI call), and small/large angle
// arrays that Shape Size interpolates between - same mechanism as an
// AI-generated shape's aiSmallAngles/aiLargeAngles. Growing the library
// (new shape, new synonym) is now a data change (edit the JSON, or press
// "Save Shape" to capture the current sliders under a new name), not a code
// change - see loadShapeLibrary()/saveShapeToLibrary() below.
JSONArray shapeLibraryData;
HashMap<String, float[]> shapeLibrarySmall = new HashMap<String, float[]>();
HashMap<String, float[]> shapeLibraryLarge = new HashMap<String, float[]>();
String SHAPE_LIBRARY_FILENAME = "shape_library.json";

// Logical coordinate system: all original GUI positions are in this space.
final float UI_DESIGN_WIDTH = 1600.0;
final float UI_DESIGN_HEIGHT = 870.0;  // Includes diagnostic lines formerly clipped at y=810.
float uiScale = 1.0;
float uiOffsetX = 0, uiOffsetY = 0;
float UI_ZOOM = 1.0;   // 1.0 = normal, 1.2 = 20% bigger, 0.8 = 20% smaller
int lastLayoutWidth = -1, lastLayoutHeight = -1;

// Fonts are cached per pixel size, since resizing the window fires many
// layout passes and createFont() is slow.
HashMap<Integer, PFont> uiFontCache = new HashMap<Integer, PFont>();
PFont hudFont; // large base font for HUD text; textSize() scales it down crisply

// Fixed camera distance - the perspective projection handles window scaling.
final float CAM_DEFAULT_DISTANCE = 2150;
CameraState camDefaultState; // captured in setup() so Reset returns to THIS view

PFont uiFont(int px) {
  PFont f = uiFontCache.get(px);
  if (f == null) {
    f = createFont("Arial", px, true);
    uiFontCache.put(px, f);
  }
  return f;
}

// HUD helpers: take logical (1600x870) coordinates and design-size fonts,
// draw in physical pixels.
void hudTextSize(float designSize) {
  textSize(max(6, designSize * uiScale));
}
void hudText(String s, float x, float y) {
  text(s, uiX(x), uiY(y));
}
void hudTextBox(String s, float x, float y, float w, float h) {
  text(s, uiX(x), uiY(y), w * uiScale, h * uiScale);
}

final boolean ALLOW_WINDOW_RESIZE = false;

void settings() {
  // Windows display scaling (125%/150%/225%...) breaks P2D/P3D windows in
  // Processing 4: after maximizing, the sketch renders a canvas larger than
  // the window, so only the top-left corner is visible and mouse clicks land
  // in the wrong place (text box/buttons stop responding). Forcing Java's UI
  // scale to ~1 makes the window and canvas use the same pixels. "1.0" is
  // ignored by Java - "1.01" is the known working value. Must run before
  // the window is created, i.e. first thing in settings().
  System.setProperty("sun.java2d.uiScale", "1.01");
  // Open at a fraction of the screen, so it fits any monitor. Windows'
  // maximize button is disabled (see setup()) because it breaks P3D on
  // high-DPI displays; change these fractions to resize the window instead.
  size(max(720, round(displayWidth * 0.85)),
       max(480, round(displayHeight * 0.80)), P3D);
  pixelDensity(1); // Avoid differing GUI dimensions on high-DPI displays.
}

// Diagnostic: prints Processing's canvas size next to the real native
// window size on every resize. If these two disagree after maximizing,
// the DPI problem described in settings() is still happening.
void logWindowSizes() {
  String msg = "[layout] canvas " + width + "x" + height;
  try {
    Object nat = surface.getNative();
    if (nat instanceof com.jogamp.newt.opengl.GLWindow) {
      com.jogamp.newt.opengl.GLWindow w = (com.jogamp.newt.opengl.GLWindow) nat;
      msg += " | window " + w.getWidth() + "x" + w.getHeight()
        + " | surface " + w.getSurfaceWidth() + "x" + w.getSurfaceHeight();
    }
  } catch (Throwable t) {
    // diagnostic only
  }
  println(msg + " | uiScale " + nf(uiScale, 1, 2));
}

int uiPx(float n) { return max(1, round(n * uiScale)); }
float uiX(float x) { return uiOffsetX + x * uiScale; }
float uiY(float y) { return uiOffsetY + y * uiScale; }

void positionControl(String name, float x, float y, float w, float h) {
  Controller c = cp5.getController(name);
  if (c == null) return;
  c.setPosition(uiX(x), uiY(y));
  c.setSize(uiPx(w), uiPx(h));
}

// Relayout ControlP5's actual click targets (not just the drawn labels).
// This runs on window resize, without recreating controllers or losing values.
void layoutResponsiveGUI() {
  if (width == lastLayoutWidth && height == lastLayoutHeight) return;
  // Resizing controls can make ControlP5 fire events (the repeated
  // "Type square..." message); ignore them while laying out.
  updatingShapeControls = true;
  lastLayoutWidth = width;
  lastLayoutHeight = height;
  uiScale = min(width / UI_DESIGN_WIDTH, height / UI_DESIGN_HEIGHT) * UI_ZOOM;
  logWindowSizes();
  uiOffsetX = (width - UI_DESIGN_WIDTH * uiScale) * 0.5;
  uiOffsetY = (height - UI_DESIGN_HEIGHT * uiScale) * 0.5;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    int id = i + 2;
    float y = ROW_START_Y + i * ROW_STEP;
    positionControl("Motor " + id, SLIDER_X, y, SLIDER_W, SLIDER_H);
    positionControl("Torque Level " + id, SLIDER_X, y, SLIDER_W, SLIDER_H);
    positionControl("View " + id, TOGGLE_X, y, TOGGLE_W, TOGGLE_H);
    positionControl("Power " + id, POWER_X, y, POWER_W, TOGGLE_H);
  }
  int buttonY = ROW_START_Y + MOTOR_COUNT * ROW_STEP + 20;
  positionControl("Create Shape", SLIDER_X, buttonY, 130, 60);
  positionControl("Reset", SLIDER_X + 190, buttonY, 130, 60);
  positionControl("Shape Size", SLIDER_X, buttonY + 80, 250, 22);
  positionControl("Show Hinges", SLIDER_X, buttonY + 80, 100, 20);

  positionControl("Save Shape", SLIDER_X, buttonY + 118, 150, 36);
  positionControl("Stop Animation", SLIDER_X + 170, buttonY + 118, 150, 36);
  layoutChatPanel();
  // Camera distance is deliberately NOT changed on resize - perspective
  // already scales the scene with window height; shrinking the distance too
  // made the arm grow twice when maximized and reset zoom on every resize.
  HINGE_CLICK_RADIUS = HINGE_CLICK_RADIUS_BASE * max(0.75, uiScale);

  // ControlP5's Label.setSize() changes the glyph renderer's size in a way
  // that is NOT proportional to controller dimensions (and can become huge).
  // Instead, rebuild actual PFont objects at an appropriate SCREEN-pixel size.
  // Font sizes scale proportionally with the controls (no upper cap).
  int motorFontSize = max(8, round(13 * uiScale));
  int valueFontSize = max(8, round(13 * uiScale));
  int buttonFontSize = max(8, round(14 * uiScale));
  int promptFontSize = max(9, round(15 * uiScale));
  PFont motorFont = uiFont(motorFontSize);
  PFont valueFont = uiFont(valueFontSize);
  PFont buttonFont = uiBoldFont(buttonFontSize);
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Controller slider = cp5.getController("Motor " + (i + 2));
    if (slider != null) {
      slider.getCaptionLabel().setFont(motorFont);
      slider.getValueLabel().setFont(valueFont);
    }
  }
  String[] buttons = {"Create Shape", "Reset", "Save Shape", "Stop Animation",
                      "Send", "New Chat"};
  for (String name : buttons) {
    Controller button = cp5.getController(name);
    if (button != null) button.getCaptionLabel().setFont(buttonFont);
  }
  cp5.getController("Shape Size").getCaptionLabel().setFont(motorFont);
  cp5.getController("Show Hinges").getCaptionLabel().setFont(motorFont);
  if (chatInput != null) chatInput.setFont(uiFont(promptFontSize));
  updatingShapeControls = false;
}

void setup() {
  // P2D/P3D (OpenGL) sketches drop auto-repeat key events by default, so a
  // held key (e.g. Backspace in the AI Prompt box) only registers once.
  // Enabling key repeat passes the OS's repeat events through to ControlP5's
  // Textfield (and to keyPressed(), so held arrow keys also keep nudging the
  // hinge offset).
  keyRepeatEnabled = true;

  loadLocalConfig(); // per-device motor count etc. - before anything uses it

  // Resizing/maximizing a P3D window on a scaled Windows display is
  // unreliable in Processing 4 (the canvas stops matching the window), so
  // the window is fixed at the near-full-screen size chosen in settings().
  // Set ALLOW_WINDOW_RESIZE = true to re-enable dragging/maximizing.
  surface.setResizable(ALLOW_WINDOW_RESIZE);
  // Explicitly keep the title bar and left controls visible on Windows.
  // This also avoids an off-screen position restored from an earlier run.
  surface.setLocation(round(displayWidth * 0.075), round(displayHeight * 0.05));
  noStroke();

  // In simulation mode, skip USB entirely; the 3D preview and GUI still work.
  myPort = null;
  if (SIMULATION_MODE) {
    println("SIMULATION MODE: Arduino/DYNAMIXEL hardware is not required.");
  } else {
    println("Available serial ports:");
    printArray(processing.serial.Serial.list());
    String port = findArduinoPort();
    if (port == null) {
      println("No Arduino serial port found - is it plugged in? "
        + "Continuing in offline GUI mode; serial commands will be skipped.");
    } else {
      try {
        // The baud rate must match the Arduino firmware (115200).
        myPort = new processing.serial.Serial(this, port, 115200);
        println("Connected to Arduino on " + port);
        // Opening the port resets most Arduinos - give it time to boot
        // before the first command is sent.
        delay(2000);
        myPort.clear();
      } catch (Exception e) {
        println("Could not connect to " + port + ": " + e.getMessage());
        println("(Close the Arduino IDE's Serial Monitor if it's open - only one program can use the port.)");
        println("Continuing in offline GUI mode; serial commands will be skipped.");
        myPort = null;
      }
    }
  }

  cp5 = new ControlP5(this);
  hudFont = createFont("Arial", 48, true);

  aiLogFile = createWriter(sketchPath("logs/ai_debug_log.txt"));
  aiLog("=== New session " + year() + "-" + nf(month(),2) + "-" + nf(day(),2)
    + " " + nf(hour(),2) + ":" + nf(minute(),2) + ":" + nf(second(),2) + " ===");

  loadShapeLibrary();
  loadFitHistory();

  ANTHROPIC_API_KEY = loadApiKey();
  if (ANTHROPIC_API_KEY == null) {
    println("WARNING: no Anthropic API key found. Set an ANTHROPIC_API_KEY environment "
      + "variable before launching Processing, or put your key as the only line in a "
      + "file named anthropic_key.txt in this sketch's folder. The AI Shape button "
      + "won't work until one of those exists.");
  }
  
  // Orbits around roughly the middle of the chain's reach (not the world
  // origin, which is empty space up and to the left of the arm) so
  // dragging immediately rotates around the shape instead of some far-off
  // point. ControlP5 draws its own auto-draw hook AFTER draw() finishes,
  // which would otherwise inherit whatever camera transform PeasyCam left
  // active for the frame and go tumbling through 3D space along with the
  // model - setAutoDraw(false) below turns that off so the GUI panel can
  // be drawn explicitly inside cam.beginHUD()/endHUD() in draw() instead,
  // which resets to a fixed screen-space projection first.
  cam = new PeasyCam(this, CHAIN_ORIGIN.x + 250, CHAIN_ORIGIN.y, CHAIN_ORIGIN.z, CAM_DEFAULT_DISTANCE);
  cam.setMinimumDistance(200);
  cam.setMaximumDistance(5000);
  // Flat shapes like the calibrated square stay at a constant world Y
  // (only the X-Z-bending motors turn, X-Y-bending ones stay neutral), so
  // PeasyCam's straight-on default view looks at that plane almost
  // perfectly edge-on - a real flat square reads as a squashed zigzag from
  // there. Tilting the default view down toward a 3/4-from-above angle
  // means flat shapes are actually visible by default instead of needing
  // to be manually orbited into view every time. Sign/amount is a
  // best-effort guess (not verified against a running sketch) - drag to
  // adjust if it tilts the wrong way.
  cam.rotateX(-1.0);
  camDefaultState = cam.getState(); // includes the tilt above
  cp5.setAutoDraw(false);
  //slider to control each motor angle - left column, one row per motor
  //
  // `position` (1-11) is this joint's order in the chain, used for all the
  // physics (bend axis, mirror set, DXL_IDS array index on the Arduino
  // side). `dxlId` (2-12) is the real hardware Dynamixel ID and is what the
  // slider/toggle are actually NAMED and labeled as, since ID 1 is the
  // passive block and was never a slider - naming controls directly after
  // dxlId means what's on screen always matches the real hardware ID.
  for (int position = 1; position <= MOTOR_COUNT; position++) {
    int rowY = ROW_START_Y + (position - 1) * ROW_STEP;
    int dxlId = position + 1;

    cp5.addSlider("Motor " + dxlId)
      .setLabel("Motor " + displayId(dxlId)) // 1 = base ... 11 = tip
      .setPosition(SLIDER_X, rowY)
      .setRange(0, 360) // Safety boundaries slightly inside the 0-360 loop
      .setValue(180.0)      // Starts right at your neutral 180° center
      .setSize(SLIDER_W, SLIDER_H)
      .getValueLabel()
      .setSize(11);

    // Per-motor switch: ON = ANGLE slider (live), OFF = TORQUE slider (see Motors.pde)
    motorsAddTorqueSlider(dxlId, SLIDER_X, rowY);
    Toggle t = cp5.addToggle("View " + dxlId)
      .setPosition(TOGGLE_X, rowY)
      .setSize(TOGGLE_W, TOGGLE_H)
      .setValue(true)
      .setMode(ControlP5.SWITCH);
    t.setLabelVisible(false);

    // Torque ON/OFF switch (like the backup): OFF = limp, ON = holding.
    cp5.addToggle("Power " + dxlId)
      .setPosition(POWER_X, rowY)
      .setSize(POWER_W, TOGGLE_H)
      .setValue(true)
      .setMode(ControlP5.SWITCH)
      .setLabelVisible(false);

    if (position == 1) toggle1 = t;
    else if (position == 2) toggle2 = t;
    else if (position == 3) toggle3 = t;
    else if (position == 4) toggle4 = t;
    else if (position == 5) toggle5 = t;
    else if (position == 6) toggle6 = t;
    else if (position == 7) toggle7 = t;
    else if (position == 8) toggle8 = t;
    else if (position == 9) toggle9 = t;
    else if (position == 10) toggle10 = t;
    else if (position == 11) toggle11 = t;
  }

  int belowRowsY = ROW_START_Y + MOTOR_COUNT * ROW_STEP + 20; // 436

  // Shared layout constants for the Create Shape / Reset pair, instead of
  // hardcoding each button's position separately - both buttons always
  // stay the same size and baseline, and moving the pair later only means
  // changing these four numbers, not two unrelated sets of coordinates.
  int buttonY = belowRowsY;
  int buttonW = 130;
  int buttonH = 60;
  int buttonGap = 60;

  cp5.addButton("Create Shape")
    .setPosition(SLIDER_X, buttonY)
    .setSize(buttonW, buttonH)
    .getCaptionLabel()
    .setSize(14);

  cp5.addButton("Reset")
    .setPosition(SLIDER_X + buttonW + buttonGap, buttonY)
    .setSize(buttonW, buttonH)
    .getCaptionLabel()
    .setSize(14);

 //slider that controls size dynamically - only "square" uses this now
 //(type it in the AI Prompt box; no dedicated button)
  cp5.addSlider("Shape Size")
    .setPosition(SLIDER_X, buttonY + buttonH + 20)
    .setSize(250, 22)
    .setRange(0.0, 1.0)
    .setValue(0.5)
    .setLabel("Shape Size");
  cp5.getController("Shape Size").setLock(true);
  cp5.getController("Shape Size").setVisible(false); // removed from the GUI

  // Pure visual aid for hand-designing a shape on the sliders: marks each
  // active joint's real pivot point (not the box center) in the 3D view, so
  // it's obvious exactly where a bend happens. On by default; off removes
  // all clutter from the viewport with one click. Read directly off this
  // toggle's value in drawlineform3d() - no controlEvent() wiring needed
  // since nothing else depends on the moment it changes.
  cp5.addToggle("Show Hinges")
    .setPosition(SLIDER_X, buttonY + buttonH + 20 + 32)
    .setSize(100, 20)
    .setValue(showHingeMarkers)
    .setMode(ControlP5.SWITCH)
    .getCaptionLabel().setSize(11);

  // Captures whatever's on the sliders RIGHT NOW into shape_library.json
  // under whatever name is currently typed in the AI Prompt box above -
  // the other half of the shape library loop: once a shape looks right
  // (AI-generated, hand-tweaked, whatever), this is how it becomes an
  // instantly-reusable preset without me hand-transcribing numbers.
  cp5.addButton("Save Shape")
    .setPosition(SLIDER_X, buttonY + buttonH + 20 + 70)
    .setSize(150, 36)
    .setLabel("Save Shape");

  cp5.addButton("Stop Animation")
    .setPosition(SLIDER_X + 170, buttonY + buttonH + 20 + 70)
    .setSize(150, 36)
    .setLabel("Stop Animation");

  // Shape Chat panel on the right (see Chat.pde) - the one place to ask
  // Claude for shapes and animations and keep refining them.
  setupChatPanel();

  layoutResponsiveGUI();
  applyTheme();
  motorsSetup();
}

void draw() {
  layoutResponsiveGUI();
  // Shape/animation results from the chat's background thread are applied
  // here, on the drawing thread, so they never change the arm mid-frame.
  chatRunMainJobs();
  motorsTick();
  background(THEME_BG);

  // Don't let dragging inside the GUI panel (sliders, buttons, prompt box)
  // also orbit the camera - PeasyCam only knows about raw mouse events, not
  // that ControlP5 controls are logically "on top" of the 3D scene.
  cam.setActive(!cp5.isMouseOver() && !chatMouseOver());

  // Read the "Show Hinges" switch straight off its control each frame -
  // same pattern as the torque toggles below, no controlEvent() wiring
  // needed since drawlineform3d() just wants this frame's current value.
  showHingeMarkers = cp5.get(Toggle.class, "Show Hinges").getBooleanValue();

  // Advance the ANIMATE player (if one is running) BEFORE drawlineform3d()
  // reads the Motor sliders - so the 3D view this exact frame already
  // reflects this tick's interpolated pose, not last frame's.
  aiUpdateAnimation();

  // Rebuild the projection for the CURRENT window size every frame. Field of
  // view is fixed vertically, so the arm scales with window height and the
  // aspect ratio always matches the window - no stretching when maximized.
  perspective(PI / 3.0, width / (float) height, 10, 30000);

  drawlineform3d(); // now also draws the real joint-curve overlay itself

  // Everything below is 2D screen-space GUI, not part of the 3D scene -
  // beginHUD()/endHUD() temporarily resets to a fixed screen-aligned
  // projection so none of it spins/tilts with the camera orbit above.
  cam.beginHUD();
  // HUD text/images use the same logical (1600x870) coordinates as the
  // controls, converted to physical pixels via uiX()/uiY(), with font sizes
  // multiplied by uiScale. This keeps text crisp at any window size -
  // scaling the matrix instead stretches a small glyph texture and blurs it.
  textFont(hudFont);
  drawThemeBackdrops();

  // Fit quality / animation / hinge info - same width as the slider panel (Theme.pde).
  drawInfoBox();

  fill(255);
  textAlign(CENTER, CENTER);
  hudTextSize(15);

  // Torque switch ON/OFF labels, one per motor row - positioned just to the
  // right of each toggle switch (not on top of it) using the same layout
  // constants setup() built the rows with, so they can never drift apart.
  Toggle[] allToggles = {
    toggle1, toggle2, toggle3, toggle4, toggle5, toggle6,
    toggle7, toggle8, toggle9, toggle10, toggle11
  };
  textAlign(LEFT, CENTER);
  int toggleLabelX = TOGGLE_X + TOGGLE_W + 10;
  for (int i = 0; i < allToggles.length; i++) {
    int rowY = ROW_START_Y + i * ROW_STEP;
    int labelY = rowY + TOGGLE_H / 2;
    boolean angle = allToggles[i].getBooleanValue();
    fill(angle ? THEME_CYAN : THEME_ORANGE);
    hudText(angle ? "ANGLE" : "TORQUE", toggleLabelX, labelY);
    boolean on = motorPowerOn(i);
    fill(on ? THEME_TEXT : THEME_ORANGE);
    hudText(on ? "ON" : "OFF", POWER_X + POWER_W + 8, labelY);
  }

  drawChatPanel();

  // ControlP5 controls are already positioned in physical pixel coordinates.
  cp5.draw();
  motorsDrawLoadOverlay(); // live torque use on the TORQUE sliders
  if (!serialConnected()) {
    fill(255, 200, 80);
    textAlign(RIGHT, BOTTOM);
    hudTextSize(15);
    // Anchored to the real window corner, not the logical canvas.
    text("OFFLINE / SIMULATION - no physical motors connected", width - uiPx(12), height - uiPx(10));
  }
  cam.endHUD();
}

// Records a click as a possible hinge tap, as long as it didn't land on
// the ControlP5 panel (that's already handled by ControlP5/controlEvent()
// - same gate PeasyCam itself uses via cam.setActive() in draw() above).
// The actual hit-test happens inside drawlineform3d(), not here - see the
// comment on hingeClickPending above for why.
void mousePressed() {
  if (!cp5.isMouseOver() && !chatMouseOver()) {
    hingeClickPending = true;
    hingeClickX = mouseX;
    hingeClickY = mouseY;
  }
}

// Live calibration for the hinge markers (see hingeOffsetX/Y/Z above):
// nudge with arrow keys / [ and ], read the new offset off the HUD or the
// console, compare the marker against the real Dynamixel's shaft on the
// physical rig, repeat until it lines up. Once it looks right, tell me the
// final X/Y/Z numbers and I'll bake them in as the new defaults.
void keyPressed() {
  // Typing in the chat box must not nudge the hinge markers.
  if (chatInput != null && chatInput.isFocus()) return;
  float step = 1.0;

  if (key == CODED) {
    if (keyCode == LEFT) hingeOffsetX -= step;
    else if (keyCode == RIGHT) hingeOffsetX += step;
    else if (keyCode == UP) hingeOffsetZ -= step;
    else if (keyCode == DOWN) hingeOffsetZ += step;
    else return;
  } else if (key == '[') {
    hingeOffsetY -= step;
  } else if (key == ']') {
    hingeOffsetY += step;
  } else {
    return;
  }

  println("Hinge marker offset -> X:" + hingeOffsetX + "  Y:" + hingeOffsetY + "  Z:" + hingeOffsetZ);
}


// Asks the Arduino where a motor's ACTUAL present position is right now
// (READ_POSITION_HEADER=253, matching the .ino firmware) rather than
// trusting the slider - a motor that's had its torque toggled off can be
// freely hand-moved, and the slider has no way to know that happened.
// Blocks briefly for the 2-byte reply (encoded as degrees*100, matching
// the firmware's encoding); falls back to whatever the slider already had
// if nothing comes back within 200ms, so a disconnected/unresponsive motor
// can't hang the whole button click.
synchronized float readMotorPosition(int dxlId, float fallbackDegrees) {
  // Offline: the slider is the simulated joint position.
  if (!serialConnected() || !rowActive(dxlId)) return fallbackDegrees;
  sendSerialBytes(253, hwId(dxlId));

  long startTime = millis();
  while (myPort.available() < 2) {
    if (millis() - startTime > 200) {
      println("No position read-back from motor " + dxlId + " within 200ms - using the slider's current value instead.");
      return fallbackDegrees;
    }
  }

  int hi = myPort.read();
  int lo = myPort.read();
  int encoded = (hi << 8) | lo;
  // hi/lo may each read back as an unsigned byte (0-255) from Serial.read();
  // reconstruct the signed int16 the firmware sent in case the high bit is set.
  if (encoded > 32767) encoded -= 65536;
  return encoded / 100.0;
}

// The low-level "push whatever's on the sliders out over serial" step,
// factored out of the Create Shape button so the ANIMATE player
// (aiUpdateAnimation()) can reuse it too. Pure write, no read-back, no
// torque juggling - Create Shape still does its own torque-sync/delay(30)
// preamble above before calling this; the animation player skips all of
// that every tick (it would block draw() and stall the glide) and assumes
// torque stays on throughout a run.
synchronized void aiSendSlidersOverSerial() {
  if (!serialConnected()) return; // slider values still update the 3D simulation

  // Bytes go in chain order (first active joint to last) - the Arduino maps
  // position i to DXL_IDS[i]. Each motor gets its TARGET (a pending bar value,
  // else its current goal), not the live reading the bar may be showing.
  // Sent as one synchronized call so the live reader can't cut into it.
  float[] targets = new float[MOTOR_COUNT];
  int[] frame = new int[1 + MOTOR_COUNT];
  frame[0] = 255;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    targets[i] = constrain(aiAnimating ? cp5.getController("Motor " + (i + 2)).getValue() : motorTargetDeg(i), 50, 310);
    frame[1 + i] = int(map(targets[i], 0, 360, 0, 250));
  }
  sendSerialBytes(frame);
  motorsNoteSentAll(targets);
}

void controlEvent(ControlEvent theEvent) {
  if (!theEvent.isController()) return;

  String name = theEvent.getName();

  // Shape Chat panel (Send button, Enter in the chat box, New Chat).
  if (chatHandleControlEvent(name)) return;
  // Motor rows: ANGLE/TORQUE switches, torque sliders, live angle drags.
  if (motorsHandleControlEvent(name, theEvent.getValue())) return;

  // Stop Animation button -> halt playback wherever it currently is.
  if (name.equals("Stop Animation")) {
    aiStopAnimation();
    return;
  }

  // Save Shape button -> capture the CURRENT slider angles into
  // shape_library.json. Name = whatever is typed in the chat box (without
  // sending it), otherwise the current shape's name.
  if (name.equals("Save Shape")) {
    String rawName = (chatInput != null) ? chatInput.getText() : "";
    if (rawName == null || rawName.trim().length() == 0) {
      rawName = selectedShape.startsWith("ai: ") ? selectedShape.substring(4) : selectedShape;
    }
    if (rawName == null || rawName.trim().length() == 0 || rawName.trim().equals("none")) {
      chatAdd("status", "To save, type a name in the chat box (don't press Send), then press Save Shape.");
      return;
    }
    saveShapeToLibrary(rawName);
    if (chatInput != null) chatInput.clear();
    chatAdd("status", "Saved the current sliders as \"" + rawName.trim().toLowerCase() + "\".");
    return;
  }

  // Create Shape button
  if (name.equals("Create Shape")) {
    // For any motor that's currently torqued off - which may have been
    // hand-moved while free, since torque-off is exactly what lets you do
    // that - read its REAL present position back from the hardware and
    // sync the slider to match, before anything else happens. The slider
    // is pure software state; it has no way of knowing the arm was moved
    // by hand while unpowered, so without this step it would still be
    // showing whatever it was set to before torque was disabled, and
    // Create Shape would send that stale value instead of wherever the
    // arm is actually posed right now.
    //
    // (Per-motor torque is now a 0-100% limit set on each row - motors at
    // 0% stay limp; everything else just takes the new angles.)
    //
    // Motors switched OFF are turned back ON first, holding wherever they
    // were posed by hand (same as the backup GUI's Create Shape).
    for (int i = 0; i < MOTOR_COUNT; i++) {
      if (!motorPowerOn(i)) motorsSetPowerSwitch(i, true);
    }

    aiSendSlidersOverSerial();

    println(serialConnected() ? "Shape sent to Arduino" : "Shape applied to 3D simulation (no Arduino connected)");
    return;
  }
  
  if (updatingShapeControls) {
    return;
  }
  
  if (name.equals("Shape Size")) {

  // Slider does nothing until a shape is selected (ask for one in the chat).
  if (selectedShape.equals("none")) {
    println("Ask for a shape in the chat first.");
    return;
  }

  shapeSize =
    theEvent.getController().getValue();

  generateShapeAngles();
  applyGeneratedAnglesToSliders();

  println(
    selectedShape +
    " size: " +
    nf(shapeSize, 1, 2)
  );

  return;
  }
  
  
  

  // Reset button
if (name.equals("Reset")) {

  // Halt any running animation first - otherwise the next draw() frame's
  // aiUpdateAnimation() just overwrites everything this button is about to
  // set, and the arm would keep ping-ponging right through "Reset".
  aiStopAnimation();

  // Snap the camera back to its original framing too, not just the motor
  // angles - otherwise the arm returns to its neutral straight pose but
  // you're still looking at it from wherever the orbit was last left,
  // which doesn't read as "reset" visually even though it is mechanically.
  if (camDefaultState != null) cam.setState(camDefaultState, 400);
  else cam.reset(400);

  // Reset the selected preset and size value.
  selectedShape = "none";
  shapeSize = 0.5;
  aiHasPreview = false;
  aiPreviewCurve.clear();
  aiShapeHasLoop = false;
  aiLoopAttachMotor = -1;
  aiShapeIsCurved = false;

  // Prevent the Shape Size slider update from triggering its event.
  updatingShapeControls = true;

  Controller shapeSizeSlider =
    cp5.getController("Shape Size");

  shapeSizeSlider.setBroadcast(false);
  shapeSizeSlider.setValue(0.5);
  shapeSizeSlider.setLock(true);
  shapeSizeSlider.setBroadcast(true);

  updatingShapeControls = false;


  // 1+2. Update motor sliders and switches without firing extra control
  // events, and turn every motor's torque ON at 100%. motorsResetMotor()
  // sends TORQUE_LIMIT 100, which on the Arduino also turns torque on (and
  // clears a motor's overload error) - so no separate 254 torque-on loop is
  // needed; sending both just doubled the burst of bytes at the Arduino.
  for (int dxlId = 2; dxlId <= 12; dxlId++) {

    Controller slider =
      cp5.getController("Motor " + dxlId);

    slider.setBroadcast(false);
    slider.setValue(180);
    slider.setBroadcast(true);


    motorsResetMotor(dxlId); // ANGLE view, 100% torque
  }

  // Give the Arduino time to work through the 11 torque commands (each is
  // several Dynamixel bus writes, longer if a motor needs its error cleared)
  // before the goal frame arrives.
  if (serialConnected()) delay(150);


  // 3. Reset the generated-angle array as well.
  for (int i = 0; i < MOTOR_COUNT; i++) {
    generatedAngles[i] = 180;
  }


  // 4. Send the neutral 180-degree shape
  // to the physical Dynamixels.
  // Sent as ONE sendSerialBytes() call: it's synchronized with the live
  // reader thread (motorsLiveReaderLoop), so the reader's 2-byte READ_ALL
  // request can no longer land in the middle of this frame. (Writing the
  // bytes one by one here let that happen - the Arduino then read 250 /
  // the sequence byte as motor angles, drove those motors to 310 / 60 deg,
  // and they overloaded: red LED, torque off. It also shifted the rest of
  // the frame, so some motors never got 180 at all.)
  if (serialConnected()) {
    int neutralAngle = int(map(180, 0, 360, 0, 250));
    int[] frame = new int[1 + MOTOR_COUNT];
    frame[0] = 255;
    for (int i = 1; i <= MOTOR_COUNT; i++) frame[i] = neutralAngle;
    sendSerialBytes(frame);
    float[] neutral = new float[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) neutral[i] = 180;
    motorsNoteSentAll(neutral);
    thread("motorsStartupDiagnostics"); // print each motor's real state once things settle
  }


  println(
    "Reset complete: straight shape, " +
    "all torque ON, no preset selected"
  );

  return;
}
}

// =====================================================================
//  AI shape generator: prompt -> Claude (called directly, no server) ->
//  3D control points -> spline -> per-motor bend angle -> Motor N sliders.
//
//  NOTE: generateShapeAngles() / applyGeneratedAnglesToSliders(), called
//  above for HOOK/SPIRAL, aren't defined in this file - they're presumably
//  in another tab of your sketch. Everything below uses its own "ai"
//  prefix so it can't collide with those, whatever they turn out to do.
// =====================================================================

String AI_SYSTEM_PROMPT =
  "You describe a shape as a small number of geometric SEGMENTS, not raw "
  + "coordinates - each segment is one simple primitive (a straight line, a "
  + "constant-curvature arc, a closed loop, a spiral, or a freeform spline "
  + "of your own control points) with a few parameters, and each segment "
  + "after the first attaches to wherever the previous one ends (or partway "
  + "through it, for a branch). The caller turns your segment list into an "
  + "exact curve with real math and fits it to a physical 11-joint jointed "
  + "arm - your job is choosing which primitives to use and their "
  + "proportions, not drawing the curve yourself.\n\n"
  + "The five families: LINE is a straight segment - use it for handles, "
  + "sticks, straight edges. ARC is a constant-curvature bend - what matters "
  + "is the TOTAL SWEEP (curve_amount * length, in radians): for a single "
  + "arc meant to be the shape's main curved feature (a hook, a handset's "
  + "curve, a scoop), keep the sweep at LEAST 75-90 degrees (roughly 1.3-1.6 "
  + "radians) or it reads as barely bent at all rather than genuinely "
  + "curved - e.g. length 5 needs curve_amount around 0.3, not 0.1. A "
  + "sweep of roughly half a circle (about 180 degrees) makes a "
  + "side-profile scoop shape, like a ladle's bowl or a catcher's mitt seen "
  + "from the side - or a telephone handset's C-curve. For that kind of "
  + "smoothly, continuously curved shape, the curve should BE the shape - "
  + "don't sandwich it between straight LINE segments at both ends (that "
  + "produces two flat tips with only the middle curved, not a smooth C). "
  + "Either use the ARC alone for the whole thing, or if straight ends are "
  + "wanted at all, keep them short relative to the arc so the curve still "
  + "reads as the dominant feature. Small curve_amount values (around 0.1) "
  + "are for a GENTLE wave; a pronounced S-curve or letter S uses the exact "
  + "same alternating-sign pattern but with a much larger curve_amount "
  + "(0.3-0.5+, same magnitude on both arcs) - the pattern that makes it a "
  + "flowing S rather than a single arc's worth of bend is chaining 2-3 arcs "
  + "at attach_position 1.0 with alternating sign (positive, negative, "
  + "positive...) while KEEPING THE SAME direction on every arc in the "
  + "chain - not for a single arc "
  + "meant to be the main event. CRITICAL: do not mirror/reverse direction "
  + "between arcs - a wave/S-curve's arcs all keep traveling the same general way "
  + "and only alternate which side they bend toward (via curve_toward and "
  + "the sign of curve_amount); mirroring direction instead makes the "
  + "second arc curl back and CLOSE the path into a lens/ring shape next to "
  + "the first arc, not continue past it into a flowing S. LOOP is a closed "
  + "circle or oval (use aspect != 1 for oval) - use it alone for a ring, "
  + "bracelet, or anything whose defining shape is a round OPENING or RIM "
  + "viewed from above: a cup or bottle holder, a bowl, a basket, a nest, a "
  + "vase's mouth. If what makes something recognizable is that it's round "
  + "when you look down into it, that's a LOOP, not an ARC's side profile. "
  + "SPIRAL is a helix/coil - set turns and pitch for how tight and how "
  + "much it rises per turn. SPLINE is a freeform curve you define directly "
  + "with a small ordered list of control_points (2-8 points, each an "
  + "[x,y,z] OFFSET relative to where this segment starts, not an absolute "
  + "coordinate) - the caller draws a smooth curve through this segment's "
  + "start point and then each control point in order. Use SPLINE only when "
  + "a shape's outline is genuinely organic/asymmetric and can't be built "
  + "from a combination of LINE/ARC/LOOP/SPIRAL - a specific irregular "
  + "silhouette, a flourish, a lopsided curl, a cursive-style stroke, an "
  + "animal's back or tail, a leaf outline. For anything that's "
  + "fundamentally a constant-curvature bend, a symmetric pair of lobes, a "
  + "circle/oval, or a coil, prefer ARC/LOOP/SPIRAL instead - they fit the "
  + "physical arm far more predictably than an arbitrary point list, since "
  + "the arm can only bend at 11 fixed joints and a mis-spaced control "
  + "point is easy to get wrong. When you do use SPLINE: space the points "
  + "out to actually define distinct bends (don't cluster several near each "
  + "other and leave a long unshaped gap elsewhere), keep offsets in the "
  + "same rough -10..10 unit scale as everything else, and use 3-5 points "
  + "for most organic shapes. Like any other family, a SPLINE segment can "
  + "be a handle's head (attach_position ~0.85-0.95) or stand alone as the "
  + "whole shape.\n\n"
  + "Composition is what makes this expressive: a hammer, screwdriver, or "
  + "any handle-and-head tool is just a long LINE (the handle) followed by "
  + "a second segment (the head) with attach_position around 0.85-0.95 - "
  + "not 1.0. The head can be any family, not just LINE - a magnifying "
  + "glass, tennis racket, or lollipop is a LINE handle followed by a LOOP "
  + "head, same attachment rule. Make the head noticeably bigger than "
  + "real-world proportions would suggest - for a LINE/ARC head, around "
  + "40-60% of the handle's remaining length after the attach point, not "
  + "10-20%; for a LOOP/SPIRAL head, its diameter (2x radius) should be in "
  + "that same 40-60% range - since the arm only has 11 rigid joints and a "
  + "head that's a small fraction of the total path claims less than one "
  + "joint and disappears into the handle (or, for a loop, comes out as a "
  + "sharp-cornered kink instead of anything round) instead of reading as a "
  + "distinct part. A closed polygon (square, "
  + "triangle, star) is a chain of LINE segments each attaching at 1.0 to "
  + "the previous one, with closed=true on the final segment. Think in "
  + "terms of these parts and how they connect, not in terms of individual "
  + "points along an outline.\n\n"
  + "The arm's joints alternate bend planes - odd-numbered joints bend "
  + "within local X-Y, even-numbered within local X-Z - but that does NOT "
  + "mean every shape needs variation spread across both. Verified on the "
  + "real arm: a flat/planar shape (a circle, square, zigzag, hook) works "
  + "best by putting nearly ALL the bending into ONE of those two families "
  + "and leaving the other close to neutral, not by splitting curvature "
  + "evenly between them - each joint's local axes have already been "
  + "rotated by every joint before it, so a chain of same-family bends "
  + "still traces a real curved 3D path on its own. Only a shape that's "
  + "GENUINELY three-dimensional by nature - a spiral/coil (it has to "
  + "advance along its own axis while it turns) or a wave meant to weave "
  + "through depth, not just a flat letter or zigzag - actually needs "
  + "direction to vary across X, Y, AND Z. Default to flat/planar unless "
  + "the shape's real-world identity requires otherwise.\n\n"
  + "This is a physical arm with exactly 11 rigid segments of roughly equal "
  + "length. Two consequences: (1) don't use more than about 5-6 segments "
  + "with attach_position under 1.0 (true branch/direction-change points) "
  + "in one shape - each one needs a joint bending in the right plane for "
  + "it, and only about half the joints qualify for a given plane, so too "
  + "many tightly-packed branches will have some land on the wrong-plane "
  + "joint or get smoothed away; (2) segment lengths that are a small "
  + "fraction of the whole shape (much less than 1/11th of the total path "
  + "length) can't be resolved - a joint can't trace a wiggle smaller than "
  + "its own physical segment. Favor a small number of large, clear, "
  + "well-proportioned segments over many tiny ones.\n\n"
  + "Worked examples: a heart is a single SPLINE segment, closed=true - NOT "
  + "two ARC segments. A circular arc has the exact same curvature along its "
  + "entire length, so two mirrored arcs that close cleanly at the top are "
  + "mathematically forced to meet with that same sharpness at the bottom "
  + "too, which draws a symmetric pointed-at-both-ends lens/eye shape, not a "
  + "heart's sharp bottom point with a ROUNDED double-bump and a notch at "
  + "the top - those two ends need genuinely different curvature, which only "
  + "SPLINE's freeform control_points can give you. Use control_points that "
  + "trace up one lobe, across the top with a small dip at the notch, down "
  + "the other lobe, and back to EXACTLY this segment's own start point (the "
  + "bottom cusp) - the LAST control point must be (0, 0, 0) or the path "
  + "won't actually close no matter what closed is set to. A reliable "
  + "8-point recipe (offsets relative to the start point): (-1.9, 2.5, 0), "
  + "(-5.3, 7.0, 0), (-1.9, 9.5, 0), (0, 7.3, 0), (1.9, 9.5, 0), (5.3, 7.0, "
  + "0), (1.9, 2.5, 0), (0, 0, 0) - notice each point on the right half "
  + "mirrors one on the left, and the last one returns exactly to (0, 0, 0). "
  + "direction still needs some reasonable value (e.g. (0, 1, 0)) even "
  + "though it doesn't shape a spline's curve. closed=true is only a LABEL "
  + "for the aligning/scaling math, not something the code enforces "
  + "geometrically - only control_points that are genuinely symmetric AND "
  + "end back at (0, 0, 0) make the path actually close; get either wrong "
  + "and it reads as an open teardrop/comma shape instead of a heart, "
  + "however well the arm happens to trace it. "
  + "A hammer is two LINE segments - a long handle "
  + "(direction along the main axis, length ~7), then a head "
  + "(direction roughly perpendicular, length ~3, attach_position ~0.9), "
  + "closed=false. A tennis racket or magnifying glass is a LINE handle "
  + "(length ~7) followed by a LOOP head (radius ~2, so diameter ~4 - "
  + "comfortably in the 40-60% range) at attach_position ~0.9, closed=false. "
  + "A ring or cup holder on its own (no handle) is a single LOOP segment, "
  + "closed=false (the loop is already closed on its own). A gentle wave is "
  + "three ARC segments chained at attach_position 1.0 with curve_amount "
  + "alternating sign (e.g. +0.1, -0.1, +0.1), closed=false. A pronounced "
  + "S-curve or the letter S is the SAME chained-arc-with-alternating-sign "
  + "pattern, just with a bigger curve_amount (e.g. +0.35 then -0.35 on two "
  + "arcs of equal length) - direction stays IDENTICAL on both arcs (e.g. "
  + "both (1, 0, 0)), only curve_toward and the sign of curve_amount flip; "
  + "mirroring direction between the two arcs instead closes them into a "
  + "lens shape with matching start/end points "
  + "rather than an open S that travels from one end to the other. A coil "
  + "spring "
  + "or pencil holder is a single SPIRAL segment - keep turns around 1-1.5, "
  + "not more: even one recognizable turn takes several joints to trace, so "
  + "11 joints total can't resolve more than about 1.5 turns before it "
  + "knots up instead of coiling. Anything built from two roughly-PARALLEL "
  + "arms joined at one end - tongs, pliers, a flat iron/hair straightener, "
  + "a tuning fork, a hairpin, a staple or square bracket shape - is THREE "
  + "LINE segments, not a curve: the first arm (length ~6-7), then a SHORT "
  + "connecting segment (attach_position 1.0, direction roughly "
  + "perpendicular to the first arm, length only around 1/6-1/8 of an "
  + "arm's length - just enough to bridge the gap between the two arms), "
  + "then the second arm (attach_position 1.0, length similar to the "
  + "first, direction roughly OPPOSITE/anti-parallel to the first arm's "
  + "direction). Two sharp corners connected by a short straight bridge, "
  + "like a staple - NOT a smooth ARC/U-turn: a curved turn needs a whole "
  + "stretch of joints to sweep smoothly and tends to run out of range "
  + "before completing, while a sharp corner only needs ONE joint to turn "
  + "cleanly, which is far more reliable on this arm.\n\n"
  + "When in doubt, favor the simplest combination of segments that a "
  + "person would recognize as representing the request.\n\n"
  + "Before calling the tool, think it through in a few sentences: what "
  + "parts does this shape have, what family fits each part, what are the "
  + "key proportions between parts (e.g. handle length vs head length), and "
  + "how do they attach. Then call the emit_shape_segments tool with "
  + "segments that match that plan.";

// Built on top of AI_SYSTEM_PROMPT (string concatenation, not a copy) so the
// full segment vocabulary/arm-limits guidance above is always included
// automatically and never drifts out of sync with it - this just appends
// what's DIFFERENT about choosing poses for a looping animation instead of
// one static shape.
String AI_ANIMATION_SYSTEM_PROMPT = AI_SYSTEM_PROMPT + "\n\n"
  + "You are now choosing a LOOPING ANIMATION, not a single static shape. "
  + "Everything above about segments, families, and the physical arm's "
  + "limits still applies. First decide which of the two fundamentally "
  + "different motion types this really is - this choice matters far more "
  + "than any parameter inside it:\n\n"
  + "(A) A WHOLE-BODY motion that moves as one piece between a few distinct "
  + "shapes - breathing, pulsing, nodding, a simple hand-wave, opening and "
  + "closing. Use `poses`: 2-4 complete shapes (keyframes), each described "
  + "exactly like a static shape. The caller solves every pose into the "
  + "arm's 11 joint angles independently, then continuously cycles through "
  + "them in order as ONE closed loop, forever (pose 1 -> 2 -> ... -> N -> "
  + "back to 1 -> ...), with no pause at any pose. For exactly 2 poses this "
  + "still reads as an ordinary back-and-forth; for 3-4 it's a genuine "
  + "one-way cycle, not a bounce.\n\n"
  + "(B) A motion that IS a wave traveling down the body's length - a "
  + "snake/eel/worm/caterpillar slithering, a whip crack, a flag or cloth "
  + "edge rippling, a tentacle undulating. Use `traveling_wave` instead, "
  + "NOT poses. This is an easy trap: a few whole-body poses (e.g. "
  + "straight / bent-one-way / bent-the-other-way) cycling through each "
  + "other LOOKS LIKE the body flexing in and out together in place, "
  + "because every joint is always moving in lockstep toward whichever "
  + "pose is next - there is no sense of a bend traveling FROM one end "
  + "toward the other, which is the entire visual signature of a real "
  + "slither. `traveling_wave` fixes this structurally: instead of a few "
  + "shapes, you give an amplitude, a wavelength (in joints), a period, "
  + "and a direction, and the caller drives each of the 11 joints as its "
  + "own point on one continuous sine wave that keeps sliding along the "
  + "chain - at any instant the body traces a real sine/S shape, and that "
  + "shape visibly travels from one end toward the other as time passes. "
  + "If a request describes something slithering, rippling, whipping, or "
  + "undulating along its length, reach for traveling_wave by default, not "
  + "poses.\n\n"
  + "Whichever of the two you use, and for EVERY animation, not just the "
  + "ones below that happen to call it out by name: joint_stiffness and "
  + "what would make this specific motion read as ALIVE rather than "
  + "mechanical are things to deliberately decide every single time, not "
  + "an embellishment reserved for snakes and whips. Almost nothing that "
  + "actually moves in the real world is uniformly rigid along its whole "
  + "length - something leads the motion firmly, and something else gets "
  + "dragged along by it, lagging/wobbling/settling a beat behind rather "
  + "than tracking it exactly. Before finalizing joint_stiffness, ask: "
  + "which joints are doing the actual work of this motion (keep those "
  + "firm, at 0) and which are just getting carried along by the rest of "
  + "the body (those are the ones to soften, so they visibly trail rather "
  + "than move in perfect lockstep)? A motion where every joint holds "
  + "identical stiffness reads as a rigid machine arm swinging a prop "
  + "around, even when the pose/wave shape itself is exactly right - "
  + "stiffness is often what separates 'a mechanism is moving' from 'a "
  + "thing is alive.' This also means the deliberate answer is sometimes "
  + "ALL ZEROS, not a reflex to soften something - a heartbeat's whole "
  + "body pulses as one unit with nothing trailing behind, a rigid tool's "
  + "head should track its handle exactly, and getting that firm choice "
  + "right is just as much 'considering stiffness' as softening a tail "
  + "is.\n\n"
  + "Worked example for (B): a SNAKE/EEL SLITHER is traveling_wave with "
  + "amplitude_degrees ~30 (a visibly pronounced undulation, not a subtle "
  + "wiggle), wavelength_joints ~6-7 (so roughly 1.5-2 full S-bends are "
  + "visible across all 11 joints at once, reading as a snake's body, not "
  + "a single gentle arc), period_ms ~900-1300 for an easy swim or "
  + "~500-700 for a fast/urgent one, and direction +1 so the wave appears "
  + "to push from the fixed base toward the free tip (a snake driving "
  + "itself forward) - use -1 only if the motion should instead look like "
  + "it's whipping FROM the free tip inward. Also give the last 4-5 joints "
  + "a higher joint_stiffness (e.g. base joints 0,0,0,0,0,0 then tail "
  + "joints 1,2,2,3,3) so the tail trails loosely behind the traveling "
  + "wave instead of tracking it as precisely as the base does - a real "
  + "snake's tail has visible lag/whip, a uniformly-stiff arm doesn't. A "
  + "WHIP CRACK is similar but with direction -1 (the wave runs from the "
  + "free tip back toward the base), a short wavelength_joints (~4-5), a "
  + "short period_ms (~300-500), and the tip joints left at FULL stiffness "
  + "(0) rather than soft - a whip's snap comes from the wave's speed and "
  + "tightness, not from looseness.\n\n"
  + "Worked examples for (A): a HEARTBEAT/PULSE is 2 poses - the exact "
  + "same SPLINE heart recipe above at two scales (multiply every "
  + "control_point offset by ~0.85 for pose 1, ~1.15 for pose 2), all "
  + "joints left at the default stiffness (0) since a heart's whole body "
  + "pulses together, not just its tip - cycling between them reads as "
  + "beating/pulsing. A BREATHING COIL is 2 poses - the same SPIRAL at two "
  + "turn counts (e.g. turns 0.7 and 1.4, same axis/radius) - reads as "
  + "compressing and releasing like a spring. A WAVING/NODDING motion is 2 "
  + "poses - a LINE or ARC handle at two different direction angles - "
  + "reads as a wave or nod back and forth, and also benefits from softer "
  + "stiffness on the last 2-3 joints (like a loose wrist/hand flopping at "
  + "the end of the motion) while the base stays firm. For poses "
  + "specifically, vary which axis the motion actually lives on to match "
  + "the real thing rather than defaulting to the same handful of example "
  + "vectors above every time: tilting mostly in Y (e.g. (0,1,0)-leaning "
  + "direction/curve_toward) reads as an up/down nod, bob, or flap; "
  + "mostly Z reads as a side-to-side wag or shake; a diagonal mix reads "
  + "as a more circular/figure-8 motion (good for a wagging tail or a "
  + "orbiting/stirring motion). Don't let every animation end up moving "
  + "the same way just because earlier examples happened to use the same "
  + "axis - pick the one that matches what's actually being asked for.\n\n"
  + "Pick 2 poses for a simple back-and-forth and only go to 3-4 when the "
  + "motion genuinely needs to pass through a distinct middle state to "
  + "read correctly - more poses isn't automatically better and costs more "
  + "compute per animation. Choose frame_duration_ms (poses) or period_ms "
  + "(traveling_wave) to match the requested energy: faster for quick/"
  + "darting/urgent motion, slower for calm/graceful motion.\n\n"
  + "Before calling the tool, think it through in a few sentences and "
  + "answer ALL of these every time, even when the motion seems simple: "
  + "(1) is this a whole-body motion or a wave traveling along the body; "
  + "(2) what's actually changing, and on which axis; (3) joint_stiffness "
  + "- which joints lead this motion firmly and which just get carried "
  + "along by it, and does leaving everything at 0 actually serve this "
  + "particular motion or is that a reflex rather than a decision; (4) "
  + "what, specifically, would make this read as a living/moving thing "
  + "rather than a mechanism - is it a trailing tail, a settling wobble, a "
  + "loose end, or does this particular object's identity come from "
  + "being precise and rigid instead; and (5) why these specific choices "
  + "at this speed and feel will read as the requested motion.";

// Reads the API key from an ANTHROPIC_API_KEY environment variable first
// (works when Processing is launched from a terminal that has it exported),
// falling back to a local anthropic_key.txt file next to the sketch.
// NEVER share/upload/commit that file - it's your live API key.
String loadApiKey() {
  String envKey = System.getenv("ANTHROPIC_API_KEY");
  if (envKey != null && envKey.trim().length() > 0) return envKey.trim();

  File keyFile = new File(sketchPath("anthropic_key.txt"));
  if (keyFile.exists()) {
    String[] lines = loadStrings(keyFile.getAbsolutePath());
    if (lines != null && lines.length > 0 && lines[0].trim().length() > 0) {
      return lines[0].trim();
    }
  }
  return null;
}

// Loads shape_library.json (next to this sketch) into shapeLibraryData plus
// the shapeLibrarySmall/Large lookup maps used by generateShapeAngles().
// Called once from setup(). If the file is missing (fresh checkout, or it
// got deleted), starts with an empty library rather than failing - "Save
// Shape" can rebuild it from scratch, and aiMatchKnownShape() simply never
// matches until then, so every prompt just falls through to the AI path.
void loadShapeLibrary() {
  String path = sketchPath(SHAPE_LIBRARY_FILENAME);
  File f = new File(path);
  if (!f.exists()) {
    println("No " + SHAPE_LIBRARY_FILENAME + " found next to the sketch - starting with an "
      + "empty shape library. Use \"Save Shape\" to add to it.");
    shapeLibraryData = new JSONArray();
    return;
  }

  shapeLibraryData = loadJSONArray(path);
  shapeLibrarySmall.clear();
  shapeLibraryLarge.clear();
  for (int i = 0; i < shapeLibraryData.size(); i++) {
    JSONObject entry = shapeLibraryData.getJSONObject(i);
    String entryName = entry.getString("name");
    shapeLibrarySmall.put(entryName, entry.getJSONArray("small").getFloatArray());
    shapeLibraryLarge.put(entryName, entry.getJSONArray("large").getFloatArray());
  }
  println("Loaded " + shapeLibraryData.size() + " shape(s) from " + SHAPE_LIBRARY_FILENAME + ": "
    + shapeLibrarySmall.keySet());
}

// Captures the CURRENT slider angles as a new (or updated) library entry
// named `rawName`, derives small/large the same way the original five
// presets were derived (scale each motor's offset from 180 by
// AI_SIZE_MIN_SCALE/MAX_SCALE), writes shape_library.json back out, and
// updates the in-memory maps immediately so it's usable without restarting
// Processing. If `rawName` already matches an existing entry (by name or
// synonym), that entry is overwritten rather than duplicated.
void saveShapeToLibrary(String rawName) {
  String entryName = rawName.trim().toLowerCase();

  float[] current = new float[MOTOR_COUNT];
  float[] small = new float[MOTOR_COUNT];
  float[] large = new float[MOTOR_COUNT];
  for (int i = 0; i < MOTOR_COUNT; i++) {
    current[i] = cp5.get(Slider.class, "Motor " + (i + 2)).getValue();
    float turn = current[i] - 180;
    small[i] = 180 + turn * AI_SIZE_MIN_SCALE;
    large[i] = 180 + turn * AI_SIZE_MAX_SCALE;
  }

  // Overwrite in place if this name (or one of its synonyms) already
  // exists, instead of appending a duplicate entry.
  int existingIndex = -1;
  for (int i = 0; i < shapeLibraryData.size(); i++) {
    JSONObject entry = shapeLibraryData.getJSONObject(i);
    if (entry.getString("name").equals(entryName)) { existingIndex = i; break; }
    JSONArray syns = entry.getJSONArray("synonyms");
    for (int s = 0; s < syns.size(); s++) {
      if (syns.getString(s).equals(entryName)) { existingIndex = i; break; }
    }
    if (existingIndex != -1) break;
  }

  JSONObject entry = (existingIndex != -1) ? shapeLibraryData.getJSONObject(existingIndex) : new JSONObject();
  entry.setString("name", entryName);
  if (!entry.hasKey("synonyms")) {
    JSONArray syns = new JSONArray();
    syns.append(entryName);
    entry.setJSONArray("synonyms", syns);
  }
  JSONArray smallArr = new JSONArray();
  JSONArray largeArr = new JSONArray();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    smallArr.append(small[i]);
    largeArr.append(large[i]);
  }
  entry.setJSONArray("small", smallArr);
  entry.setJSONArray("large", largeArr);

  if (existingIndex != -1) {
    shapeLibraryData.setJSONObject(existingIndex, entry);
  } else {
    shapeLibraryData.append(entry);
  }

  saveJSONArray(shapeLibraryData, sketchPath(SHAPE_LIBRARY_FILENAME));
  shapeLibrarySmall.put(entryName, small);
  shapeLibraryLarge.put(entryName, large);

  println("Saved \"" + entryName + "\" to " + SHAPE_LIBRARY_FILENAME + " (" + current.length
    + " motor angles). Ask for \"" + entryName + "\" in the chat any time to load it.");
}

// Loads AI_FIT_HISTORY_FILENAME (a simple append-only CSV next to the
// sketch) into aiBestFitQualityEver, keeping only the best (lowest) fit
// quality ever recorded per shape name - across every past run, not just
// this session - so a fresh generation of a shape you've tried before can
// be compared against its own history from the moment it's ready, not only
// against attempts made in the current run. Missing/unreadable file is not
// an error - it just means no history exists yet.
void loadFitHistory() {
  File f = new File(sketchPath(AI_FIT_HISTORY_FILENAME));
  if (!f.exists()) return;

  String[] lines = loadStrings(f.getAbsolutePath());
  if (lines == null) return;

  for (int i = 1; i < lines.length; i++) { // row 0 is the header
    String[] cols = split(lines[i], ',');
    if (cols.length < 4) continue;
    String name = cols[1].trim().toLowerCase();
    if (name.length() == 0) continue;
    float quality;
    try {
      quality = Float.parseFloat(cols[3].trim());
    } catch (Exception e) {
      continue;
    }
    Float existing = aiBestFitQualityEver.get(name);
    if (existing == null || quality < existing) aiBestFitQualityEver.put(name, quality);
  }
  aiLog("Loaded fit-quality history for " + aiBestFitQualityEver.size() + " shape name(s) from "
    + AI_FIT_HISTORY_FILENAME);
}

// Called once per generation attempt (the initial one, and each refinement
// round) - this is the actual "cross compare the accuracy of shapes drawn"
// feature: logs how this attempt's fit quality stacks up against the best
// this exact shape name has ever scored, updates that running best, and
// appends a row to AI_FIT_HISTORY_FILENAME so every attempt (this session
// and every past one) can be reviewed later, e.g. by opening the CSV in a
// spreadsheet to see which prompts/shapes fit well vs poorly, or whether
// refinement rounds are actually helping over time.
void aiRecordFitHistory(String shapeName, int refineAttempt, float fitQuality) {
  String key = shapeName.trim().toLowerCase();
  Float previousBest = aiBestFitQualityEver.get(key);

  String comparison;
  if (previousBest == null) {
    comparison = "first time generating \"" + shapeName + "\"";
  } else if (fitQuality < previousBest - 0.01) {
    comparison = "BETTER than the previous best for \"" + shapeName + "\" (" + nf(previousBest, 1, 2) + ")";
  } else if (fitQuality > previousBest + 0.01) {
    comparison = "worse than the previous best for \"" + shapeName + "\" (" + nf(previousBest, 1, 2) + ")";
  } else {
    comparison = "about the same as the previous best for \"" + shapeName + "\" (" + nf(previousBest, 1, 2) + ")";
  }
  aiLog("Fit quality " + nf(fitQuality, 1, 2) + " - " + comparison + ".");

  if (previousBest == null || fitQuality < previousBest) {
    aiBestFitQualityEver.put(key, fitQuality);
  }

  try {
    File historyFile = new File(sketchPath(AI_FIT_HISTORY_FILENAME));
    historyFile.getParentFile().mkdirs();
    boolean isNew = !historyFile.exists();
    PrintWriter w = new PrintWriter(new FileWriter(historyFile, true));
    if (isNew) w.println("timestamp,shape_name,refine_attempt,fit_quality");
    String timestamp = year() + "-" + nf(month(), 2) + "-" + nf(day(), 2) + " "
      + nf(hour(), 2) + ":" + nf(minute(), 2) + ":" + nf(second(), 2);
    String safeName = shapeName.replace(",", " ").trim(); // keep the row a plain 4-column CSV, no quoting needed
    w.println(timestamp + "," + safeName + "," + refineAttempt + "," + nf(fitQuality, 1, 3));
    w.close();
  } catch (IOException e) {
    aiLog("Warning: couldn't write to " + AI_FIT_HISTORY_FILENAME + ": " + e.getMessage());
  }
}

// Keyword match against the shape library loaded above - deliberately
// exact-match only (not fuzzy/substring), so a functional description that
// happens to share a word with a saved shape doesn't silently hijack it.
// Checks the canonical name AND every synonym for each entry.
String aiMatchKnownShape(String promptLower) {
  if (shapeLibraryData == null) return null;
  for (int i = 0; i < shapeLibraryData.size(); i++) {
    JSONObject entry = shapeLibraryData.getJSONObject(i);
    if (entry.getString("name").equals(promptLower)) return entry.getString("name");
    JSONArray syns = entry.getJSONArray("synonyms");
    for (int s = 0; s < syns.size(); s++) {
      if (syns.getString(s).equals(promptLower)) return entry.getString("name");
    }
  }
  return null;
}

// Selects a calibrated preset shape - reads the current Shape Size slider
// value, unlocks it if needed, and applies the resulting angles straight to
// the sliders. No AI call, no preview overlay (there's nothing approximate
// to compare against here).
void aiUseKnownShape(String shapeKey, String originalPrompt) {
  selectedShape = shapeKey;
  aiHasPreview = false;
  aiPreviewCurve.clear();
  aiShapeHasLoop = false;
  aiLoopAttachMotor = -1;
  aiShapeIsCurved = false;

  Controller shapeSizeSlider = cp5.getController("Shape Size");
  shapeSizeSlider.setLock(false);
  shapeSize = shapeSizeSlider.getValue();

  generateShapeAngles();
  applyGeneratedAnglesToSliders();

  println("\"" + originalPrompt + "\" matched the calibrated " + shapeKey
    + " shape - using the tuned preset (no AI call needed). Press Create "
    + "Shape to send it, or drag Shape Size to resize it.");
}

void triggerAIShape(String prompt) {
  if (prompt == null || prompt.trim().length() == 0) {
    println("Type a shape description in the AI Prompt box first.");
    return;
  }

  // A static shape (known-preset or AI) should never fight with a running
  // ANIMATE player - without this, the shape would apply for one draw()
  // frame and then immediately get overwritten by aiUpdateAnimation() on
  // the next, since aiAnimating would still be true.
  aiStopAnimation();

  String promptLower = prompt.trim().toLowerCase();

  // If the prompt clearly matches one of the calibrated hook/spiral/zigzag/
  // circle/square presets above, use that tuned data directly instead of
  // asking the AI to reinvent it from scratch - it's real values already
  // validated on the robot, so it's strictly more accurate than a guess.
  String knownShape = aiMatchKnownShape(promptLower);
  if (knownShape != null) {
    aiUseKnownShape(knownShape, prompt.trim());
    return;
  }

  if (ANTHROPIC_API_KEY == null || ANTHROPIC_API_KEY.trim().length() == 0) {
    println("No Anthropic API key found - see the WARNING printed at startup.");
    return;
  }

  JSONObject requestBody = aiBuildRequestBody(prompt.trim());
  AIShapeResult result = aiCallAndParse(requestBody.toString());
  if (!result.ok) {
    aiLog(result.errorMessage);
    return;
  }

  // v7: if Claude says this is a variant of a calibrated preset, use the
  // tuned preset instead of fitting a fresh AI shape.
  JSONObject resultInput = result.toolUse.getJSONObject("input");
  if (AI_USE_LIBRARY_MATCH && resultInput.hasKey("library_match")) {
    String match = aiMatchKnownShape(resultInput.getString("library_match").trim().toLowerCase());
    if (match != null) {
      aiLog("Claude matched \"" + prompt.trim() + "\" to calibrated preset \"" + match + "\".");
      aiUseKnownShape(match, prompt.trim());
      return;
    }
    aiLog("Claude suggested library_match \"" + resultInput.getString("library_match")
      + "\", but no such preset exists - fitting its segments instead.");
  }

  // ---- v9: two-phase fitting (base-anchored first, center-anchored fallback) ----
  // Every trial's ORIGINAL segment list is kept (cloned before
  // aiGenerateFromSegments' repairs mutate it), so phase 2 can re-fit the
  // exact same proposals with a different anchor, and so the final winner
  // can be re-applied cleanly at the end.
  ArrayList<JSONArray> trialSegments = new ArrayList<JSONArray>();
  ArrayList<Boolean> trialClosed = new ArrayList<Boolean>();
  ArrayList<String> trialNames = new ArrayList<String>();

  JSONArray bestSegments = null;
  boolean bestClosed = false;
  String bestShapeName = result.shapeName;
  boolean bestCentered = false;
  float bestFitQuality = Float.MAX_VALUE;
  int trialCount = 0;

  // ---------- Phase 1: start point on the fixed base ----------
  aiAnchorAtCenter = false;
  aiLog("Phase 1: fitting with the shape's start point on the arm's base.");

  JSONArray firstPristine = aiCloneSegments(result.segments);
  aiGenerateFromSegments(aiCloneSegments(firstPristine), result.closed, result.shapeName);
  aiRecordFitHistory(result.shapeName, 0, aiLastFitQuality);
  trialSegments.add(firstPristine);
  trialClosed.add(result.closed);
  trialNames.add(result.shapeName);
  trialCount++;
  bestFitQuality = aiLastFitQuality;
  bestSegments = firstPristine;
  bestClosed = result.closed;

  // Refinement rounds: replay every prior tool call plus its feedback so
  // Claude can see its own attempt history and revise in context, stopping
  // early once a fit is good enough.
  ArrayList<JSONObject> roundToolUses = new ArrayList<JSONObject>();
  ArrayList<String> roundFeedback = new ArrayList<String>();
  JSONObject lastToolUse = result.toolUse;
  JSONArray lastPristine = firstPristine;
  boolean lastClosed = result.closed;
  String lastName = result.shapeName;
  float lastFitQuality = aiLastFitQuality;
  int attempt = 0;

  while (lastToolUse != null && lastFitQuality > AI_REFINE_QUALITY_THRESHOLD && attempt < AI_MAX_REFINE_ROUNDS) {
    attempt++;
    aiLog("Fit quality was " + nf(lastFitQuality, 1, 2) + " (aiming for under "
      + nf(AI_REFINE_QUALITY_THRESHOLD, 1, 2) + ") - asking Claude for a refined "
      + "shape (base-anchored attempt " + attempt + " of " + AI_MAX_REFINE_ROUNDS + "). This costs one more API call.");

    roundToolUses.add(lastToolUse);
    roundFeedback.add(aiBuildRefinementFeedback(lastFitQuality));

    AIShapeResult refined = aiCallAndParse(aiBuildRefinementRequestBody(prompt.trim(), roundToolUses, roundFeedback).toString());
    if (!refined.ok) {
      aiLog("Refinement attempt " + attempt + " failed (" + refined.errorMessage + ") - stopping refinement.");
      lastToolUse = null; // already has feedback in the history - must not be replayed twice
      break;
    }

    JSONArray pristine = aiCloneSegments(refined.segments);
    aiGenerateFromSegments(aiCloneSegments(pristine), refined.closed, refined.shapeName);
    aiRecordFitHistory(refined.shapeName, attempt, aiLastFitQuality);
    trialSegments.add(pristine);
    trialClosed.add(refined.closed);
    trialNames.add(refined.shapeName);
    trialCount++;

    lastFitQuality = aiLastFitQuality;
    lastToolUse = refined.toolUse;
    lastPristine = pristine;
    lastClosed = refined.closed;
    lastName = refined.shapeName;

    if (lastFitQuality < bestFitQuality) {
      aiLog("Base-anchored attempt " + attempt + " improved fit quality: "
        + nf(bestFitQuality, 1, 2) + " -> " + nf(lastFitQuality, 1, 2) + ".");
      bestFitQuality = lastFitQuality;
      bestSegments = pristine;
      bestClosed = refined.closed;
      bestShapeName = refined.shapeName;
    } else {
      aiLog("Base-anchored attempt " + attempt + " did not improve on the best fit so far ("
        + nf(lastFitQuality, 1, 2) + " vs " + nf(bestFitQuality, 1, 2) + ").");
    }
  }

  // ---------- Phase 2: center anchoring, only if EVERY base trial was bad ----------
  if (AI_TRY_CENTER_ANCHOR && bestFitQuality > AI_REFINE_QUALITY_THRESHOLD) {
    aiLog("Phase 2: all " + trialCount + " base-anchored trial(s) scored above "
      + nf(AI_REFINE_QUALITY_THRESHOLD, 1, 2) + " (best " + nf(bestFitQuality, 1, 2)
      + ") - trying center anchoring.");
    aiAnchorAtCenter = true;

    // 2a. Re-fit every proposal already received, centered. Pure local math,
    // no API calls.
    int baseTrials = trialSegments.size();
    for (int i = 0; i < baseTrials; i++) {
      aiGenerateFromSegments(aiCloneSegments(trialSegments.get(i)), trialClosed.get(i), trialNames.get(i));
      aiRecordFitHistory(trialNames.get(i), i, aiLastFitQuality);
      aiLog("Center-anchored re-fit of trial " + (i + 1) + ": fit quality " + nf(aiLastFitQuality, 1, 2) + ".");
      if (aiLastFitQuality < bestFitQuality) {
        aiLog("  -> new best (" + nf(bestFitQuality, 1, 2) + " -> " + nf(aiLastFitQuality, 1, 2) + ").");
        bestFitQuality = aiLastFitQuality;
        bestSegments = trialSegments.get(i);
        bestClosed = trialClosed.get(i);
        bestShapeName = trialNames.get(i);
        bestCentered = true;
      }
    }

    // 2b. Optional extra Claude rounds, with feedback measured under
    // center anchoring.
    int centerRound = 0;
    while (lastToolUse != null && bestFitQuality > AI_REFINE_QUALITY_THRESHOLD && centerRound < AI_CENTER_REFINE_ROUNDS) {
      centerRound++;
      attempt++;

      // Make sure the diagnostics the feedback reads describe the LAST
      // proposal fitted centered (2a's loop may have ended on a different one).
      aiGenerateFromSegments(aiCloneSegments(lastPristine), lastClosed, lastName);
      float centeredLast = aiLastFitQuality;
      aiLog("Asking Claude for a refined shape (center-anchored attempt " + centerRound + " of "
        + AI_CENTER_REFINE_ROUNDS + "). This costs one more API call.");

      roundToolUses.add(lastToolUse);
      roundFeedback.add(aiBuildRefinementFeedback(centeredLast));

      AIShapeResult refined = aiCallAndParse(aiBuildRefinementRequestBody(prompt.trim(), roundToolUses, roundFeedback).toString());
      if (!refined.ok) {
        aiLog("Center-anchored refinement failed (" + refined.errorMessage + ") - stopping.");
        break;
      }
  
      JSONArray pristine = aiCloneSegments(refined.segments);
      aiGenerateFromSegments(aiCloneSegments(pristine), refined.closed, refined.shapeName);
      aiRecordFitHistory(refined.shapeName, attempt, aiLastFitQuality);
      trialCount++;
      lastToolUse = refined.toolUse;
      lastPristine = pristine;
      lastClosed = refined.closed;
      lastName = refined.shapeName;

      if (aiLastFitQuality < bestFitQuality) {
        aiLog("Center-anchored attempt " + centerRound + " improved fit quality: "
          + nf(bestFitQuality, 1, 2) + " -> " + nf(aiLastFitQuality, 1, 2) + ".");
        bestFitQuality = aiLastFitQuality;
        bestSegments = pristine;
        bestClosed = refined.closed;
        bestShapeName = refined.shapeName;
        bestCentered = true;
      } else {
        aiLog("Center-anchored attempt " + centerRound + " did not improve on the best fit so far ("
          + nf(aiLastFitQuality, 1, 2) + " vs " + nf(bestFitQuality, 1, 2) + ").");
      }
    }
  }

  // ---------- Apply the single best trial from either phase ----------
  // Pure local math, no API call. The anchor mode is reset afterwards so
  // animations and any later generation start from base anchoring again.
  aiAnchorAtCenter = bestCentered;
  aiGenerateFromSegments(aiCloneSegments(bestSegments), bestClosed, bestShapeName);
  aiAnchorAtCenter = false;
  aiLog("Best fit quality across " + trialCount + " trial(s): " + nf(bestFitQuality, 1, 2)
    + " (" + (bestCentered ? "center-anchored" : "base-anchored") + ") - applied to sliders.");
}

// ANIMATE: ask Claude for 2-4 poses describing a looping motion, solve EACH
// one through the exact same aiGenerateFromSegments() pipeline a static
// shape uses (full reuse - no new geometry/fitting code at all), snapshot
// each pose's resulting 11-angle array, and start the ping-pong player.
// Deliberately simpler than triggerAIShape(): no multi-round critique/
// refine loop per pose (that loop replays a single shape's own tool_use/
// tool_result history, which doesn't generalize cleanly to N independent
// poses sharing one animation request) - an animation is about the overall
// MOTION reading right, not any one pose's fit-quality score, so a single
// solve per pose keeps this fast and simple. Fit quality is still logged
// per pose so a badly-fitting pose doesn't fail silently.
void triggerAIAnimation(String prompt) {
  if (prompt == null || prompt.trim().length() == 0) {
    println("Type a motion description in the AI Prompt box first (e.g. \"slither like a snake\").");
    return;
  }

  if (ANTHROPIC_API_KEY == null || ANTHROPIC_API_KEY.trim().length() == 0) {
    println("No Anthropic API key found - see the WARNING printed at startup.");
    return;
  }

  aiStopAnimation(); // never leave an old animation silently playing under a new one

  JSONObject requestBody = aiBuildAnimationRequestBody(prompt.trim());
  AIAnimationResult result = aiCallAndParseAnimation(requestBody.toString());
  if (!result.ok) {
    aiLog(result.errorMessage);
    return;
  }

  aiStartAnimation(result);
}

// Solves (poses) or sets up (traveling wave) an already-parsed animation and
// starts playing it. Shared by triggerAIAnimation() and the Shape Chat.
// How well each pose of the last pose-animation fit the real arm (same
// scale as a shape's fit quality) - reported back to the Motion agent.
ArrayList<String> aiAnimPoseNames = new ArrayList<String>();
ArrayList<Float> aiAnimPoseFits = new ArrayList<Float>();

void aiStartAnimation(AIAnimationResult result) {
  aiStopAnimation(); // never leave an old animation silently playing under a new one
  aiAnimName = result.animationName;
  aiAnimIsWave = result.isTravelingWave;
  aiAnimPoses.clear();
  aiAnimPoseNames.clear();
  aiAnimPoseFits.clear();

  if (aiAnimIsWave) {
    aiLog("--- Generating animation '" + result.animationName + "' (traveling wave, "
      + nf(result.waveAmplitudeDeg, 1, 0) + "° amplitude, " + nf(result.waveWavelengthJoints, 1, 1)
      + " joints/cycle, " + nf(result.wavePeriodMs, 1, 0) + "ms/cycle, direction "
      + (result.waveDirection > 0 ? "base->tip" : "tip->base") + ") ---");

    aiAnimWaveAmplitudeDeg = result.waveAmplitudeDeg;
    aiAnimWaveWavelengthJoints = result.waveWavelengthJoints;
    aiAnimWavePeriodMs = max(150, result.wavePeriodMs);
    aiAnimWaveDirection = result.waveDirection;

  } else {
    aiLog("--- Generating animation '" + result.animationName + "' (" + result.poses.size()
      + " poses, " + nf(result.frameDurationMs, 1, 0) + "ms/transition) ---");

    ArrayList<float[]> poses = new ArrayList<float[]>();
    for (int i = 0; i < result.poses.size(); i++) {
      AIAnimationPose pose = result.poses.get(i);
      aiLog("Pose " + (i + 1) + "/" + result.poses.size() + ": '" + pose.poseName + "'");
      aiGenerateFromSegments(pose.segments, pose.closed, result.animationName + " - " + pose.poseName);
      aiAnimPoseNames.add(pose.poseName);
      aiAnimPoseFits.add(aiLastFitQuality);

      float[] snapshot = new float[MOTOR_COUNT];
      arrayCopy(generatedAngles, snapshot);
      poses.add(snapshot);
    }

    aiAnimPoses = poses;
    aiAnimFrameDurationMs = max(150, round(result.frameDurationMs)); // floor so a malformed tiny duration can't spin draw() into a frame-skipping mess
  }

  aiAnimIndex = 0;
  aiAnimLoopStartMillis = millis();
  aiAnimLastSendMillis = 0;
  aiAnimating = true;

  // Apply whatever per-joint stiffness "feel" the AI chose for this motion
  // (all 0s/full stiffness if it omitted joint_stiffness or got it wrong -
  // see aiCallAndParseAnimation) - for the whole animation's run, not
  // pose-to-pose (or cycle-to-cycle). aiStopAnimation() below restores
  // everything to full stiffness again once playback ends.
  aiApplyStiffnessLevels(result.jointStiffness);
  aiLog("Joint stiffness for '" + aiAnimName + "': " + aiJoinIntArray(result.jointStiffness));

  // Shape Size would otherwise sit there doing nothing useful (every frame
  // of the animation overwrites the sliders it controls anyway) and inviting
  // a confusing drag mid-animation - lock it the same way Reset/a fresh AI
  // shape already lock/unlock it elsewhere, and restore it in aiStopAnimation().
  cp5.getController("Shape Size").setLock(true);

  aiLog("Animation '" + aiAnimName + "' playing - press Stop Animation to end it.");
}

// Halts playback wherever it currently is (the arm simply stops moving at
// its current interpolated pose - it does not snap back to neutral; press
// Reset for that). Also restores every joint to full stiffness (level 0),
// so a motion that asked for soft/trailing joints never leaves them loose
// once you're back to manual slider control. Safe to call even when
// nothing is animating.
void aiStopAnimation() {
  if (!aiAnimating) return;
  aiAnimating = false;
  aiAnimIsWave = false;
  cp5.getController("Shape Size").setLock(false);

  int[] fullStiffness = new int[MOTOR_COUNT]; // all 0s
  aiApplyStiffnessLevels(fullStiffness);
  motorsRestoreAnimationTorque(); // torque back to what it was before the animation

  aiLog("Animation '" + aiAnimName + "' stopped.");
}

// Called once per draw() frame. Two completely different sampling
// strategies depending on aiAnimIsWave (set in triggerAIAnimation from
// which one the AI returned), converging on the same
// applyGeneratedAnglesToSliders()/throttled-serial-write tail:
//   - WAVE: aiSampleTravelingWave() computes every joint directly from a
//     continuously-growing phase - a closed-form function of time, no
//     bookkeeping, no seam.
//   - POSES: advances a single progress value u (0..1, wrapping forever)
//     around the WHOLE pose sequence as one continuous closed loop via
//     aiSampleAnimationLoop()'s cyclic Catmull-Rom spline.
void aiUpdateAnimation() {
  if (!aiAnimating) return;
  if (!aiAnimIsWave && aiAnimPoses.size() < 2) return;

  int now = millis();
  float[] sampled;

  if (aiAnimIsWave) {
    float phase = (now - aiAnimLoopStartMillis) / aiAnimWavePeriodMs; // grows forever - only ever used inside sin(), which wraps it on its own
    sampled = aiSampleTravelingWave(aiAnimWaveAmplitudeDeg, aiAnimWaveWavelengthJoints, phase, aiAnimWaveDirection);

    // Locomotion layer (see AI_ANIM_WAVE_DRIFT_DEG above) - always applied,
    // regardless of what the AI chose, so the whole body keeps slowly
    // re-aiming instead of looking parked in one spot.
    float driftPhase = (now - aiAnimLoopStartMillis) / AI_ANIM_WAVE_DRIFT_PERIOD_MS;
    float drift = AI_ANIM_WAVE_DRIFT_DEG * sin(TWO_PI * driftPhase);
    sampled[0] = constrain(sampled[0] + drift, 60, 310);
    sampled[1] = constrain(sampled[1] + drift * 0.6, 60, 310);

    aiAnimIndex = 0; // not meaningful in wave mode - HUD text ignores it

  } else {
    int poseCount = aiAnimPoses.size();
    float loopDurationMs = aiAnimFrameDurationMs * poseCount;
    float elapsed = (now - aiAnimLoopStartMillis) % loopDurationMs;
    float u = elapsed / loopDurationMs; // 0..1, wraps forever

    sampled = aiSampleAnimationLoop(aiAnimPoses, u, AI_ANIM_SPLINE_TENSION);
    aiAnimIndex = ((int) floor(u * poseCount)) % poseCount; // display only - HUD text
  }

  arrayCopy(sampled, generatedAngles);
  applyGeneratedAnglesToSliders();

  if (now - aiAnimLastSendMillis >= AI_ANIM_SEND_INTERVAL_MS) {
    aiSendSlidersOverSerial();
    aiAnimLastSendMillis = now;
  }
}

// Deep-copies a segment list so a later aiGenerateFromSegments() call (which
// mutates segment JSONObjects in place via aiValidateAndRepairSegments)
// can't retroactively corrupt an earlier round's saved "best so far" copy.
JSONArray aiCloneSegments(JSONArray segments) {
  JSONArray clone = new JSONArray();
  for (int i = 0; i < segments.size(); i++) {
    clone.append(JSONObject.parse(segments.getJSONObject(i).toString()));
  }
  return clone;
}

// Everything a caller needs back from one Claude shape-generation call,
// bundled together since Processing/Java has no easy multi-return. `ok`
// tells the caller whether to trust the rest of the fields; `errorMessage`
// is set (and everything else left at defaults) when it isn't.
class AIShapeResult {
  boolean ok = false;
  String errorMessage = "";
  JSONObject toolUse;
  JSONArray segments;
  boolean closed = false;
  String shapeName = "custom";
}

// Calls Claude with the given request body and parses its emit_shape_segments
// tool call into an AIShapeResult. Shared by the initial generation call and
// the critique-and-refine follow-up call, since both need identical parsing.
// Per-field validation (missing/malformed numbers, bad vectors) is handled
// downstream by aiJsonVec3/aiJsonFloat's fallbacks in the geometry engine,
// not here - this just confirms Claude actually returned a usable segment list.
AIShapeResult aiCallAndParse(String requestBodyJson) {
  AIShapeResult result = new AIShapeResult();

  try {
    String json = aiCallAnthropic(requestBodyJson);
    JSONObject resp = JSONObject.parse(json);

    if (resp.hasKey("type") && resp.getString("type").equals("error")) {
      JSONObject err = resp.getJSONObject("error");
      result.errorMessage = "Claude API error: " + err.getString("message");
      return result;
    }

    JSONArray content = resp.getJSONArray("content");
    JSONObject toolUse = null;
    for (int i = 0; i < content.size(); i++) {
      JSONObject block = content.getJSONObject(i);
      if (block.hasKey("type") && block.getString("type").equals("tool_use")) {
        toolUse = block;
        break;
      }
    }
    if (toolUse == null) {
      result.errorMessage = "Claude didn't return structured shape data - try rephrasing the prompt.";
      return result;
    }

    JSONObject input = toolUse.getJSONObject("input");
    result.shapeName = input.hasKey("shape_name") ? input.getString("shape_name") : "custom";
    result.closed = input.hasKey("closed") && input.getBoolean("closed");
    JSONArray segments = input.hasKey("segments") ? input.getJSONArray("segments") : null;

    if (segments == null || segments.size() < 1) {
      result.errorMessage = "Claude returned no usable segments - try rephrasing.";
      return result;
    }

    result.segments = segments;
    result.toolUse = toolUse;
    result.ok = true;
    return result;
  }
  catch (Exception e) {
    result.errorMessage = "AI shape request failed: " + e.getMessage();
    return result;
  }
}

// One pose (keyframe) of an ANIMATE response - same shape as AIShapeResult's
// segments/closed pair, just without its own toolUse/shapeName bookkeeping
// since a whole animation shares one tool_use block across all its poses.
class AIAnimationPose {
  String poseName = "pose";
  boolean closed = false;
  JSONArray segments;
}

// Everything triggerAIAnimation() needs back from one emit_shape_animation
// call - mirrors AIShapeResult's ok/errorMessage contract exactly, just
// carrying a list of poses instead of one segment list.
class AIAnimationResult {
  boolean ok = false;
  String errorMessage = "";
  String animationName = "animation";
  float frameDurationMs = 900;
  ArrayList<AIAnimationPose> poses = new ArrayList<AIAnimationPose>();
  // One stiffness LEVEL (0-3) per joint, in position order (index 0 =
  // hinge 2 ... index 10 = hinge 12), applied for the whole animation's
  // run - see aiApplyStiffnessLevels(). Defaults to all 0s (full
  // stiffness); aiCallAndParseAnimation always fills this with something
  // valid, even when the model omits joint_stiffness entirely.
  int[] jointStiffness = new int[MOTOR_COUNT];

  // Set when the model chose traveling_wave instead of poses - a motion
  // that's a sine wave propagating along the chain (snake/eel/whip/flag),
  // computed live every frame (aiUpdateAnimation) directly from these 4
  // numbers instead of from aiAnimPoses. When true, poses above is empty
  // and irrelevant.
  boolean isTravelingWave = false;
  float waveAmplitudeDeg = 25;
  float waveWavelengthJoints = 7;
  float wavePeriodMs = 1000;
  float waveDirection = 1; // +1 or -1 only
}

// Calls Claude with the given request body and parses its
// emit_shape_animation tool call into an AIAnimationResult - same shape and
// same defensive checks as aiCallAndParse(), just walking a `poses` array of
// {pose_name, closed, segments} instead of one top-level segments array.
AIAnimationResult aiCallAndParseAnimation(String requestBodyJson) {
  AIAnimationResult result = new AIAnimationResult();

  try {
    String json = aiCallAnthropic(requestBodyJson);
    JSONObject resp = JSONObject.parse(json);

    if (resp.hasKey("type") && resp.getString("type").equals("error")) {
      JSONObject err = resp.getJSONObject("error");
      result.errorMessage = "Claude API error: " + err.getString("message");
      return result;
    }

    JSONArray content = resp.getJSONArray("content");
    JSONObject toolUse = null;
    for (int i = 0; i < content.size(); i++) {
      JSONObject block = content.getJSONObject(i);
      if (block.hasKey("type") && block.getString("type").equals("tool_use")) {
        toolUse = block;
        break;
      }
    }
    if (toolUse == null) {
      result.errorMessage = "Claude didn't return a structured animation - try rephrasing the prompt.";
      return result;
    }

    return aiParseAnimationInput(toolUse.getJSONObject("input"));
  }
  catch (Exception e) {
    result.errorMessage = "AI animation request failed: " + e.getMessage();
    return result;
  }
}

// Turns an emit_shape_animation tool call's "input" into an
// AIAnimationResult. Shared by triggerAIAnimation() and the Shape Chat.
AIAnimationResult aiParseAnimationInput(JSONObject input) {
  AIAnimationResult result = new AIAnimationResult();
  try {
    result.animationName = input.hasKey("animation_name") ? input.getString("animation_name") : "animation";

    // Exactly one of traveling_wave / poses should be present - schema
    // can't enforce that itself (see aiBuildAnimationToolDefinition), so
    // branch on whichever one actually showed up.
    boolean hasWave = input.hasKey("traveling_wave");
    boolean hasPoses = input.hasKey("poses");

    if (hasWave) {
      JSONObject wave = input.getJSONObject("traveling_wave");
      result.isTravelingWave = true;
      result.waveAmplitudeDeg = aiJsonFloat(wave, "amplitude_degrees", 25);
      result.waveWavelengthJoints = max(4, aiJsonFloat(wave, "wavelength_joints", 7)); // floor - shorter can't be resolved across fixed-length joints
      result.wavePeriodMs = max(150, aiJsonFloat(wave, "period_ms", 1000));
      float dir = aiJsonFloat(wave, "direction", 1);
      result.waveDirection = (dir < 0) ? -1 : 1; // only +1/-1 are meaningful - clamp anything else to the nearest of those

    } else if (hasPoses) {
      result.frameDurationMs = aiJsonFloat(input, "frame_duration_ms", 900);
      JSONArray poses = input.getJSONArray("poses");

      if (poses == null || poses.size() < 2) {
        result.errorMessage = "Claude returned fewer than 2 poses - an animation needs at least 2 to move between.";
        return result;
      }

      // Defensive cap even though the schema already says maxItems 4 -
      // tool schemas are a strong hint to the model, not a hard
      // server-side clamp.
      int poseCount = min(poses.size(), 4);
      for (int i = 0; i < poseCount; i++) {
        JSONObject poseObj = poses.getJSONObject(i);
        AIAnimationPose pose = new AIAnimationPose();
        pose.poseName = poseObj.hasKey("pose_name") ? poseObj.getString("pose_name") : ("pose " + (i + 1));
        pose.closed = poseObj.hasKey("closed") && poseObj.getBoolean("closed");
        JSONArray segs = poseObj.hasKey("segments") ? poseObj.getJSONArray("segments") : null;
        if (segs == null || segs.size() < 1) {
          result.errorMessage = "Pose " + (i + 1) + " (\"" + pose.poseName + "\") had no usable segments - try rephrasing.";
          return result;
        }
        pose.segments = segs;
        result.poses.add(pose);
      }

    } else {
      result.errorMessage = "Claude returned neither poses nor traveling_wave - try rephrasing the prompt.";
      return result;
    }

    // Optional. Wrong length or missing entirely just logs a note and
    // falls back to all-0s (full stiffness) rather than failing the whole
    // animation over one malformed extra field.
    if (input.hasKey("joint_stiffness")) {
      JSONArray stiffArr = input.getJSONArray("joint_stiffness");
      if (stiffArr != null && stiffArr.size() == MOTOR_COUNT) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
          result.jointStiffness[i] = constrain(stiffArr.getInt(i), 0, NUM_STIFFNESS_LEVELS - 1);
        }
      } else {
        aiLog("joint_stiffness had the wrong length (" + (stiffArr == null ? "null" : str(stiffArr.size()))
          + ", expected " + MOTOR_COUNT + ") - ignoring it, animation will play at full stiffness.");
      }
    }

    result.ok = true;
    return result;
  }
  catch (Exception e) {
    result.errorMessage = "Could not read the animation: " + e.getMessage();
    return result;
  }
}

// Composes the feedback text sent back to Claude for one refinement round,
// using the per-joint diagnostics aiGenerateFromSegments() just recorded
// (aiLastWorstJoint / aiLastJointSaturated) in addition to the single
// aggregate fit-quality number - so a refinement round has an actual place
// to look, not just a score to feel bad about.
String aiBuildRefinementFeedback(float fitQuality) {
  StringBuilder sb = new StringBuilder();
  sb.append("When fit to the actual 11-joint physical arm (alternating "
    + "bend planes, roughly a 5-6 branch-point budget, can't resolve segments "
    + "much shorter than about 1/11th of the shape's size), your segments "
    + "scored a fit-quality of " + nf(fitQuality, 1, 2) + " (a rough, unitless "
    + "measure of how far the achievable shape drifted from your intended "
    + "curve - lower is better, under " + nf(AI_REFINE_QUALITY_THRESHOLD, 1, 2)
    + " is a good fit).");

  if (aiLastClosureGap > 0.05) {
    sb.append(" IMPORTANT: this shape was marked closed=true, but your own segments' "
      + "endpoint landed " + nf(aiLastClosureGap * 100, 1, 0) + "% of the path's length "
      + "away from its own starting point - closed=true is just a label, nothing checks "
      + "or forces the path to actually return to its start. If the final segment is a "
      + "SPLINE, its LAST control_point needs to be exactly (0, 0, 0) - back at that "
      + "segment's own start; if it's a mirrored pair of ARCs, the final segment's "
      + "direction/curve_toward wasn't the true mirror image of the matching earlier "
      + "segment (note: two mirrored arcs can only close into a symmetric lens/eye shape, "
      + "never something like a heart that needs different curvature at each end - use "
      + "SPLINE for that instead). This matters even more than the fit-quality number "
      + "above: an open path that happens to be easy for the arm to trace can score well "
      + "numerically while not looking like the requested shape at all. Fix the closure "
      + "first, before worrying about fine-tuning proportions.");
  }

  if (aiLastWorstJoint > 0) {
    sb.append(" The single worst-fitting point was around joint " + aiLastWorstJoint
      + " of " + MOTOR_COUNT + " (counting from the fixed base, joint 1) - whichever "
      + "part of your path lands at roughly that fraction along its length is the "
      + "part most worth simplifying or re-proportioning.");
  }

  int saturatedCount = 0;
  for (int j = 0; j < MOTOR_COUNT; j++) if (aiLastJointSaturated[j]) saturatedCount++;
  if (saturatedCount > 0) {
    sb.append(" " + saturatedCount + " of " + MOTOR_COUNT + " joints hit the arm's hard "
      + "angle limit while solving - that usually means some part of the path asked for "
      + "a sharper turn than one joint can deliver; spreading that turn across more of "
      + "the shape's length, or reducing curve_amount/turns there, usually helps.");
  }

  // v7: concrete per-joint evidence, in the same coordinates as Claude's own
  // segments, so it can see WHAT went wrong rather than only a score.
  if (aiLastJointReport.length() > 0) {
    sb.append(" Here is where each joint was supposed to go versus where the real arm "
      + "actually reached, in the same coordinates as your segments ("
      + (aiAnchorAtCenter
        ? "this fit centered your shape on the arm, so your start point is NOT pinned to the arm's fixed base"
        : "your shape starts at its first segment's start point, which sits on the arm's fixed base")
      + "):\n"
      + aiLastJointReport
      + "Compare this with your intended shape: look for sides that came out shorter or "
      + "longer, corners that didn't turn far enough, or a path that doesn't close.");
  }

  sb.append(" That's not literal per-segment instructions, just a signal about where "
    + "the fit struggled. Propose a revised segment list for the same shape that "
    + "would fit this specific arm more accurately - usually that means fewer "
    + "segments, more even proportions between them, and making sure any branch "
    + "(attach_position under 1.0) is a genuinely short, distinct feature rather "
    + "than a second segment nearly as long as what it's attached to.");

  return sb.toString();
}

// Builds a follow-up request using the Anthropic API's normal multi-turn
// tool-use pattern: replay the user prompt, then EVERY prior round's tool
// call plus its feedback in order (not just the immediately preceding one),
// so Claude can see its own full attempt history - including whether its
// last correction actually helped - before proposing another revision.
// `priorToolUses`/`priorFeedback` are parallel lists, one entry per
// completed round so far.
JSONObject aiBuildRefinementRequestBody(String originalPrompt, ArrayList<JSONObject> priorToolUses, ArrayList<String> priorFeedback) {
  JSONObject body = new JSONObject();
  body.setString("model", ANTHROPIC_MODEL);
  body.setInt("max_tokens", 4096);
  body.setJSONArray("system", aiBuildCachedSystemBlocks());

  JSONArray tools = new JSONArray();
  tools.append(aiBuildToolDefinition());
  body.setJSONArray("tools", tools);

  // "auto" (not a forced "tool" choice) so Claude can write a few sentences
  // of reasoning about the object's proportions before committing to
  // segments, per the system prompt's instructions - aiCallAndParse()
  // already scans all content blocks for the tool_use one, so text before
  // it is simply ignored when parsing, no change needed there.
  JSONObject toolChoice = new JSONObject();
  toolChoice.setString("type", "auto");
  body.setJSONObject("tool_choice", toolChoice);

  JSONArray messages = new JSONArray();

  JSONObject userMsg1 = new JSONObject();
  userMsg1.setString("role", "user");
  userMsg1.setString("content", originalPrompt);
  messages.append(userMsg1);

  for (int r = 0; r < priorToolUses.size(); r++) {
    JSONObject roundToolUse = priorToolUses.get(r);

    JSONObject assistantMsg = new JSONObject();
    assistantMsg.setString("role", "assistant");
    JSONArray assistantContent = new JSONArray();
    assistantContent.append(roundToolUse);
    assistantMsg.setJSONArray("content", assistantContent);
    messages.append(assistantMsg);

    JSONObject toolResultBlock = new JSONObject();
    toolResultBlock.setString("type", "tool_result");
    toolResultBlock.setString("tool_use_id", roundToolUse.getString("id"));
    toolResultBlock.setString("content", priorFeedback.get(r));

    JSONObject userMsgN = new JSONObject();
    userMsgN.setString("role", "user");
    JSONArray userMsgNContent = new JSONArray();
    userMsgNContent.append(toolResultBlock);
    userMsgN.setJSONArray("content", userMsgNContent);
    messages.append(userMsgN);
  }

  body.setJSONArray("messages", messages);
  return body;
}

// Takes a shape's raw control points (whether from the AI or hand-authored)
// through the full fit-and-apply pipeline: detect corners, pick the
// matching curve type, align to the chain's world space, corner-aware
// resample, solve joint angles, and load them onto the sliders.
void aiGenerateFromSegments(JSONArray segments, boolean closed, String shapeName) {
  // Diagnostic dump - if a shape comes out wrong, this plus the fit-quality
  // line at the end of this function is what to paste back for debugging:
  // it shows exactly what Claude asked for, so a bad result can be traced
  // to "the AI proposed a bad shape" vs "the fitting math handled a good
  // shape badly" vs (if this all looks right but the real robot still
  // moves wrong) a hardware calibration issue.
  aiLog("--- Generating '" + shapeName + "' (" + segments.size() + " segments, "
    + (closed ? "closed" : "open") + ") ---");
  for (int i = 0; i < segments.size(); i++) {
    aiLog("  [" + i + "] " + segments.getJSONObject(i).toString());
  }

  // Validate/repair handle+head proportions (any branch segment) before
  // building geometry - catches cases where Claude's proposal drifted from
  // the system prompt's guidance, without needing another API round trip.
  aiValidateAndRepairSegments(segments, closed);

  // Geometry Engine: turns the (now-repaired) segment list into one
  // continuous curve procedurally (exact math, not AI-invented
  // coordinates), plus the exact points where segments join - known
  // corners, not inferred from an angle threshold, since we built the
  // curve ourselves and know precisely where the joins are.
  AISegmentAssembly assembly = aiAssembleSegments(segments);
  ArrayList<PVector> curve = assembly.curve;
  ArrayList<PVector> aiCorners = assembly.corners;
  aiLog("Assembled curve: " + curve.size() + " points, " + aiCorners.size() + " segment joins");

  if (curve.size() < 2) {
    aiLog("Curve build failed - try rephrasing the prompt.");
    return;
  }

  // Two-stage fit: get a rough initial angle estimate from the local
  // turn-angle heuristic (aiAnglesFromCurve - fast, scale-invariant, but
  // only looks at the ideal curve's own tangent direction, so small
  // per-joint errors can compound over 11 joints). Use that rough guess
  // purely to align the AI's abstract curve into the chain's real
  // world-space neighborhood, then re-solve each joint analytically
  // against the ACTUAL accumulated chain position (aiSolveAnglesForCurve)
  // instead of the idealized curve alone - this is what keeps errors from
  // compounding joint to joint.
  float[] roughAngles = aiAnglesFromCurve(curve, closed, MOTOR_COUNT);
  ArrayList<PVector> achievableForAlignment = aiComputeAchievableCurve(roughAngles);
  ArrayList<PVector> alignedCurve = aiAlignCurveToWorld(curve, achievableForAlignment, closed);

  // Joint allocation: instead of generating a smooth continuous curve and
  // then RESAMPLING it onto 11 joints by arc-length fraction (which forced
  // every segment's joint share to be inferred/guessed afterward, and
  // caused nearly every fitting bug today - bounding-box vs path-length
  // scaling, corners snapping to the wrong motor, a bridge segment scaling
  // to less than one physical link), each segment gets an EXPLICIT integer
  // joint count up front, proportional to its own arc length, guaranteed
  // to sum to exactly MOTOR_COUNT. A segment's targets are then resampled
  // only from ITS OWN allocated share of joints, so corners land exactly
  // where they should by construction - no guessing, no snapping.
  ArrayList<Float> segLengths = aiSegmentLengths(curve, aiCorners);
  int[] jointCounts = aiAllocateJointCounts(segLengths, MOTOR_COUNT);
  ArrayList<PVector> jointTargets = aiResampleByJointCounts(alignedCurve, curve, aiCorners, jointCounts);

  // Corner k (between segment k and segment k+1) sits at the LAST joint of
  // segment k - i.e. the running total of joint counts through segment k.
  aiCornerAnchorMotors = new int[aiCorners.size()];
  int runningJoints = 0;
  for (int k = 0; k < jointCounts.length; k++) {
    runningJoints += jointCounts[k];
    if (k < aiCorners.size()) aiCornerAnchorMotors[k] = runningJoints;
  }

  // A corner at index k in `aiCorners` is where segments[k+1] (the segment
  // AFTER the corner) branches off - see aiAssembleSegments(), which appends
  // one corner per segment in that same order. If that segment is a
  // LOOP/SPIRAL, this corner is a loop's attach point, and
  // aiCornerAnchorMotors[k] (populated just above) is the joint the
  // renderer should draw the closing stroke back to.
  aiShapeHasLoop = false;
  aiLoopAttachMotor = -1;
  for (int k = 0; k < aiCorners.size() && k + 1 < segments.size(); k++) {
    JSONObject headSeg = segments.getJSONObject(k + 1);
    String fam = headSeg.hasKey("family") ? headSeg.getString("family") : "line";
    if ((fam.equals("loop") || fam.equals("spiral")) && k < aiCornerAnchorMotors.length) {
      aiShapeHasLoop = true;
      aiLoopAttachMotor = aiCornerAnchorMotors[k];
    }
  }

  // A shape is "curved" (worth drawing the achievable curve as a smoothed
  // spline instead of a straight-segment polyline) if ANY segment is an
  // ARC/LOOP/SPIRAL - a pure-LINE shape (square, hammer) has genuinely
  // sharp corners that should stay sharp, not get smoothed away.
  aiShapeIsCurved = false;
  for (int k = 0; k < segments.size(); k++) {
    String fam = segments.getJSONObject(k).hasKey("family")
      ? segments.getJSONObject(k).getString("family") : "line";
    if (fam.equals("arc") || fam.equals("loop") || fam.equals("spiral")) {
      aiShapeIsCurved = true;
      break;
    }
  }

  // FABRIK-based refinement on top of the joint-count-allocated targets:
  // relaxes toward each joint's own fixed target while keeping every link
  // EXACTLY the real physical link length by construction (rather than
  // clamping angles after the fact), then solves axis-respecting angles
  // from those positions. Verified standalone in Python: an undersized/
  // mismatched target (the exact staple-bridge bug) gets absorbed mostly
  // by the joint nearest the mismatch, with the rest of the chain staying
  // within a small fraction of a unit of its own target, instead of the
  // error cascading through every joint after it.
  float[] angles = aiFabrikRefineAnglesForCurve(CHAIN_ORIGIN, jointTargets, MOTOR_COUNT, 10);

  // Normalize the raw fit error by the achievable shape's own size, so it's
  // a rough scale-independent "how bad is this fit" score that
  // triggerAIShape() can compare against AI_REFINE_QUALITY_THRESHOLD
  // regardless of how big or small this particular shape is.
  float referenceScale = max(aiBoundsExtent(achievableForAlignment), 0.001);
  aiLastFitQuality = aiLastRefineError / referenceScale;

  // Closure check on Claude's OWN raw curve (before any arm fitting) - see
  // aiLastClosureGap's declaration for why this matters: nothing else in
  // this pipeline checks whether a "closed:true" shape's segments actually
  // return to their own starting point, which let a heart-shaped request
  // score a great fit quality while being an open, unmirrored spiral. Added
  // directly into aiLastFitQuality so the refine loop can't route around a
  // real "doesn't look like the requested shape" problem by simply scoring
  // well on arm-fidelity to a badly-shaped target.
  aiLastClosureGap = 0;
  if (closed && curve.size() >= 2) {
    float gap = PVector.dist(curve.get(0), curve.get(curve.size() - 1));
    float curveOwnLength = max(aiPathLength(curve, false), 0.001);
    aiLastClosureGap = gap / curveOwnLength;
    aiLastFitQuality += aiLastClosureGap * AI_CLOSURE_GAP_WEIGHT;
  }

  aiLog("Fit quality: " + nf(aiLastFitQuality, 1, 2) + " (lower is better, under "
    + nf(AI_REFINE_QUALITY_THRESHOLD, 1, 2) + " is good)"
    + (closed ? "  [closure gap: " + nf(aiLastClosureGap * 100, 1, 0) + "% of path length]" : ""));

  // Per-joint breakdown for the SAME final angles that produced the fit
  // quality above - which single joint drifted furthest from its target,
  // and which joints had to clamp against the arm's hard safety limit.
  // Feeds aiBuildRefinementFeedback() with something more specific than one
  // aggregate number to react to.
  ArrayList<PVector> achievableFinal = aiComputeAchievableCurve(angles);
  float worstJointError = -1;
  aiLastWorstJoint = -1;
  for (int j = 0; j < MOTOR_COUNT; j++) {
    float jointErr = PVector.dist(achievableFinal.get(j + 1), jointTargets.get(j));
    aiLastJointError[j] = jointErr;
    aiLastJointSaturated[j] = (angles[j] <= 60.5 || angles[j] >= 309.5);
    if (jointErr > worstJointError) {
      worstJointError = jointErr;
      aiLastWorstJoint = j + 1;
    }
  }

  // v7: joint targets vs. where the arm really got, in the AI's own units.
  aiLastJointReport = new StringBuilder();
  for (int j = 0; j < MOTOR_COUNT; j++) {
    PVector t = aiToAiUnits(jointTargets.get(j));
    PVector a = aiToAiUnits(achievableFinal.get(j + 1));
    aiLastJointReport.append("joint " + (j + 1) + ": wanted (" + nf(t.x, 1, 1) + ", " + nf(t.y, 1, 1) + ", " + nf(t.z, 1, 1)
      + ") got (" + nf(a.x, 1, 1) + ", " + nf(a.y, 1, 1) + ", " + nf(a.z, 1, 1) + ")"
      + (aiLastJointSaturated[j] ? "  [hit angle limit]" : "") + "\n");
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    float turn = angles[i] - 180;
    aiSmallAngles[i] = 180 + turn * AI_SIZE_MIN_SCALE;
    aiLargeAngles[i] = 180 + turn * AI_SIZE_MAX_SCALE;
    generatedAngles[i] = angles[i];
  }

  aiPreviewCurve.clear();
  aiPreviewCurve.addAll(alignedCurve);
  aiHasPreview = true;

  selectedShape = "ai: " + shapeName;

  // Unlock the size slider and center it at 0.5 - the exact shape just
  // generated sits at the midpoint between the smaller/tighter (0.0) and
  // bigger/more exaggerated (1.0) versions, so there's no jump.
  updatingShapeControls = true;
  Controller shapeSizeSlider = cp5.getController("Shape Size");
  shapeSizeSlider.setBroadcast(false);
  shapeSizeSlider.setValue(0.5);
  shapeSizeSlider.setLock(false);
  shapeSizeSlider.setBroadcast(true);
  updatingShapeControls = false;
  shapeSize = 0.5;

  applyAIAnglesToSliders();

  aiLog("Shape '" + shapeName + "' loaded onto sliders - press Create Shape to send it, or drag Shape Size to resize it.");
}

// Builds a [x,y,z] number-array schema fragment - used for every direction/
// hint vector field below, so they're all defined identically.
JSONObject aiVec3Schema(String description) {
  JSONObject numberItem = new JSONObject();
  numberItem.setString("type", "number");

  JSONObject vec = new JSONObject();
  vec.setString("type", "array");
  vec.setJSONObject("items", numberItem);
  vec.setInt("minItems", 3);
  vec.setInt("maxItems", 3);
  vec.setString("description", description);
  return vec;
}

JSONObject aiNumberSchema(String description) {
  JSONObject num = new JSONObject();
  num.setString("type", "number");
  num.setString("description", description);
  return num;
}

// One segment's schema (family/direction/length/curve_amount/curve_toward/
// radius/aspect/turns/pitch/attach_position/control_points) - factored out
// of aiBuildToolDefinition() so emit_shape_segments' single shape AND
// emit_shape_animation's multiple poses (each pose is itself a segment list)
// share the exact same per-segment rules instead of two schemas drifting
// apart over time.
JSONObject aiBuildSegmentItemSchema() {
  JSONObject segProps = new JSONObject();

  JSONObject familyProp = new JSONObject();
  familyProp.setString("type", "string");
  JSONArray familyEnum = new JSONArray();
  familyEnum.append("line");
  familyEnum.append("arc");
  familyEnum.append("loop");
  familyEnum.append("spiral");
  familyEnum.append("spline");
  familyProp.setJSONArray("enum", familyEnum);
  familyProp.setString("description",
    "line = straight segment. arc = constant-curvature bend (use several "
    + "chained arcs for a wave/S-curve, or one large-sweep arc for a "
    + "U-shaped cradle/holder). loop = closed circle/oval. spiral = "
    + "helix/coil. spline = freeform curve through your own control_points "
    + "- use ONLY for a genuinely organic/asymmetric shape the other four "
    + "families can't express; prefer arc/loop/spiral whenever the shape is "
    + "actually a constant-curvature bend, a symmetric pair of lobes, or a "
    + "circle/coil, since those fit the physical arm far more predictably.");
  segProps.setJSONObject("family", familyProp);

  segProps.setJSONObject("direction",
    aiVec3Schema("initial travel direction for line/arc/spiral, or the tangent "
      + "direction at this loop's own start point for loop - need not be a unit "
      + "vector, just point the right way. Give this real variation across X, Y, "
      + "AND Z for most shapes (see depth note below), not just X/Y."));

  segProps.setJSONObject("length", aiNumberSchema(
    "arc length of this segment (line/arc). Abstract units, roughly 1-10 - "
    + "relative proportions between segments matter far more than the "
    + "absolute number, since the whole shape gets rescaled to fit the arm."));

  segProps.setJSONObject("curve_amount", aiNumberSchema(
    "arc only: signed curvature (roughly 1/radius). 0 or omitted = straight. "
    + "Small values (~0.05-0.15) = a gentle bend. Larger values (~0.3+) = a "
    + "tight hook. Sign picks which way it bends relative to curve_toward."));

  segProps.setJSONObject("curve_toward",
    aiVec3Schema("arc/loop/spiral: roughly which way the bend/loop/coil opens "
      + "toward - doesn't need to be precise or perpendicular, just point "
      + "generally the right way."));

  segProps.setJSONObject("radius", aiNumberSchema("loop/spiral: radius of the circle/coil."));
  segProps.setJSONObject("aspect", aiNumberSchema(
    "loop only: width/height ratio, 1.0 = circle, other values = oval."));
  segProps.setJSONObject("turns", aiNumberSchema("spiral only: number of full revolutions."));
  segProps.setJSONObject("pitch", aiNumberSchema(
    "spiral only: how far it rises along its axis per full revolution."));

  segProps.setJSONObject("attach_position", aiNumberSchema(
    "0.0-1.0, ignored for the first segment. 1.0 (default) continues smoothly "
    + "from exactly where the previous segment ends. A smaller value branches "
    + "off partway through the previous segment instead - the previous segment "
    + "gets cut short there and the path turns into this new segment, since "
    + "the physical arm can only trace one continuous path, never a true fork. "
    + "Use this for a handle-and-head tool: the head segment attaches near the "
    + "end of the handle segment (e.g. 0.85-0.95), not at 1.0."));

  JSONObject cpNumberItem = new JSONObject();
  cpNumberItem.setString("type", "number");
  JSONObject cpPointSchema = new JSONObject();
  cpPointSchema.setString("type", "array");
  cpPointSchema.setJSONObject("items", cpNumberItem);
  cpPointSchema.setInt("minItems", 3);
  cpPointSchema.setInt("maxItems", 3);

  JSONObject controlPointsProp = new JSONObject();
  controlPointsProp.setString("type", "array");
  controlPointsProp.setJSONObject("items", cpPointSchema);
  controlPointsProp.setInt("minItems", 2);
  controlPointsProp.setInt("maxItems", 8);
  controlPointsProp.setString("description",
    "spline family ONLY: an ordered list of 2-8 [x,y,z] points, each a "
    + "RELATIVE offset from where THIS segment starts (not an absolute "
    + "world coordinate) - same rough -10..10 unit scale as length "
    + "elsewhere. A smooth curve is drawn through this segment's own start "
    + "point and then every listed point in order. Use 3-5 points for most "
    + "organic shapes (2 is just a curved line; 6-8 only for something "
    + "genuinely intricate). Space points out to actually define distinct "
    + "bends - don't cluster several close together while leaving a long "
    + "unshaped gap elsewhere.");
  segProps.setJSONObject("control_points", controlPointsProp);

  JSONObject segItem = new JSONObject();
  segItem.setString("type", "object");
  segItem.setJSONObject("properties", segProps);
  JSONArray segRequired = new JSONArray();
  segRequired.append("family");
  segRequired.append("direction");
  segItem.setJSONArray("required", segRequired);
  return segItem;
}

JSONObject aiBuildToolDefinition() {
  JSONObject tool = new JSONObject();
  tool.setString("name", "emit_shape_segments");
  tool.setString("description",
    "Describe a shape as an ordered list of geometric segments (not raw "
    + "coordinates) - each segment is a simple primitive (line, arc, loop, "
    + "or spiral) with a few parameters, and each one after the first "
    + "attaches to wherever the previous segment left off (or partway "
    + "along it, for a branch). The caller turns this into an exact curve "
    + "procedurally and fits it to the physical arm.");

  JSONObject schema = new JSONObject();
  schema.setString("type", "object");

  JSONObject properties = new JSONObject();

  JSONObject shapeNameProp = new JSONObject();
  shapeNameProp.setString("type", "string");
  shapeNameProp.setString("description", "short label for the shape, e.g. 'hammer', 'ring', 'wave'");
  properties.setJSONObject("shape_name", shapeNameProp);

  JSONObject closedProp = new JSONObject();
  closedProp.setString("type", "boolean");
  closedProp.setString("description",
    "true only if the path closes back on its own starting point (a "
    + "single loop segment is inherently closed on its own already - use "
    + "this for a chain of segments that comes back around, not for loops)");
  properties.setJSONObject("closed", closedProp);

  JSONObject segmentsProp = new JSONObject();
  segmentsProp.setString("type", "array");
  segmentsProp.setJSONObject("items", aiBuildSegmentItemSchema());
  segmentsProp.setInt("minItems", 1);
  segmentsProp.setInt("maxItems", 12);
  segmentsProp.setString("description",
    "ordered list of segments forming the shape - one segment for a simple "
    + "primitive, several for a compound shape (a wave, a hammer's handle + "
    + "head, etc.)");
  properties.setJSONObject("segments", segmentsProp);

  // v7: optional routing to a calibrated library preset (see aiBuildRequestBody).
  JSONObject libraryMatchProp = new JSONObject();
  libraryMatchProp.setString("type", "string");
  libraryMatchProp.setString("description",
    "OPTIONAL. Exact name of a calibrated library shape (listed in the user message) - "
    + "ONLY when the request means exactly that shape (synonym/rephrasing/translation). "
    + "Omit if the request adds any qualifier such as 'isosceles' or 'equilateral'.");
  properties.setJSONObject("library_match", libraryMatchProp);


  schema.setJSONObject("properties", properties);

  JSONArray required = new JSONArray();
  required.append("shape_name");
  required.append("closed");
  required.append("segments");
  schema.setJSONArray("required", required);

  tool.setJSONObject("input_schema", schema);

  // Prompt caching: this tool definition is byte-for-byte identical on
  // every single call (initial generation AND every refinement round), so
  // marking it as an ephemeral cache breakpoint lets the API skip
  // re-processing it (and, since cache breakpoints are cumulative in
  // request order - tools, then system, then messages - everything
  // upstream of it) on the 2nd/3rd call of a shape-generation sequence.
  // Paired with the system-prompt breakpoint in aiBuildCachedSystemBlocks()
  // below, this is what actually shortens the now-longer (up to 3 call)
  // refine loop - no change in what's asked for, just less re-sent per call.
  tool.setJSONObject("cache_control", aiCacheControlEphemeral());
  return tool;
}

// ANIMATE's tool: same segment vocabulary as emit_shape_segments
// (aiBuildSegmentItemSchema(), shared not duplicated) but asks for a small
// ORDERED LIST of complete poses instead of one shape - see
// AI_ANIMATION_SYSTEM_PROMPT for what the caller does with them (solves
// each independently, then ping-pongs between them forever).
JSONObject aiBuildAnimationToolDefinition() {
  JSONObject tool = new JSONObject();
  tool.setString("name", "emit_shape_animation");
  tool.setString("description",
    "Describe a short LOOPING MOTION. Give EXACTLY ONE of two things, "
    + "depending on what kind of motion this really is: `poses` for a "
    + "WHOLE-BODY motion that moves between a few distinct shapes together "
    + "(breathing, pulsing, nodding, a simple wave) - describe 2-4 complete "
    + "poses (keyframes), each exactly like emit_shape_segments, and the "
    + "caller solves each one into the arm's 11 joint angles independently "
    + "then continuously cycles through them as ONE closed loop, forever, "
    + "with no pause. Or `traveling_wave` for a motion that is fundamentally "
    + "a WAVE PROPAGATING ALONG THE BODY'S LENGTH - a snake/eel/worm/"
    + "caterpillar slithering, a whip crack, a flag rippling, a tentacle "
    + "undulating - these do NOT look right as a few whole-body poses "
    + "flexing in and out together (that reads as breathing/flexing in "
    + "place, not as something traveling down the body); `traveling_wave` "
    + "drives each of the 11 joints as its own point on a continuous sine "
    + "wave that keeps moving along the chain, which is what an actual "
    + "traveling ripple looks like. Optionally also set joint_stiffness "
    + "(works with either mode) to make specific joints feel loose/"
    + "trailing rather than firm for the whole motion.");

  JSONObject schema = new JSONObject();
  schema.setString("type", "object");
  JSONObject properties = new JSONObject();

  JSONObject animNameProp = new JSONObject();
  animNameProp.setString("type", "string");
  animNameProp.setString("description", "short label for the motion, e.g. 'snake slither', 'heartbeat'");
  properties.setJSONObject("animation_name", animNameProp);

  properties.setJSONObject("frame_duration_ms", aiNumberSchema(
    "Only used with poses (ignored for traveling_wave, which has its own "
    + "period_ms instead): milliseconds to glide from one pose to the "
    + "next. 400-700 for a quick/darting motion, 800-1400 for a calm/"
    + "breathing motion. The same duration is used between every pair of "
    + "poses."));

  JSONObject poseNameProp = new JSONObject();
  poseNameProp.setString("type", "string");
  poseNameProp.setString("description", "short label for this one pose, e.g. 'straight', 'bent left'");

  JSONObject poseClosedProp = new JSONObject();
  poseClosedProp.setString("type", "boolean");
  poseClosedProp.setString("description", "same meaning as emit_shape_segments' own closed field");

  JSONObject poseSegmentsProp = new JSONObject();
  poseSegmentsProp.setString("type", "array");
  poseSegmentsProp.setJSONObject("items", aiBuildSegmentItemSchema());
  poseSegmentsProp.setInt("minItems", 1);
  poseSegmentsProp.setInt("maxItems", 12);
  poseSegmentsProp.setString("description", "this pose's segments - same rules as emit_shape_segments");

  JSONObject poseProps = new JSONObject();
  poseProps.setJSONObject("pose_name", poseNameProp);
  poseProps.setJSONObject("closed", poseClosedProp);
  poseProps.setJSONObject("segments", poseSegmentsProp);

  JSONObject poseItem = new JSONObject();
  poseItem.setString("type", "object");
  poseItem.setJSONObject("properties", poseProps);
  JSONArray poseRequired = new JSONArray();
  poseRequired.append("pose_name");
  poseRequired.append("segments");
  poseItem.setJSONArray("required", poseRequired);

  JSONObject posesProp = new JSONObject();
  posesProp.setString("type", "array");
  posesProp.setJSONObject("items", poseItem);
  posesProp.setInt("minItems", 2);
  posesProp.setInt("maxItems", 4);
  posesProp.setString("description",
    "REQUIRED unless traveling_wave is given instead - give exactly one of "
    + "the two. 2-4 poses, in the order the arm should glide through them, "
    + "for a WHOLE-BODY motion (breathing, pulsing, nodding, a simple hand-"
    + "wave): 2 poses for a simple back-and-forth, 3-4 only when there's a "
    + "genuine distinct middle state to pass through. Do NOT use poses for "
    + "a snake/eel/whip/flag/tentacle or anything else that should look "
    + "like a wave traveling down the body - use traveling_wave for that "
    + "instead; a few whole-body poses cycling through each other reads as "
    + "the body flexing in and out in place, not as a ripple moving along "
    + "it.");
  properties.setJSONObject("poses", posesProp);

  JSONObject waveAmplitudeProp = aiNumberSchema(
    "degrees each joint swings away from straight (180°) at the peak "
    + "of the wave. 15-25 for a subtle ripple, 25-40 for a pronounced "
    + "slither. Same amplitude for every joint - joint_stiffness is what "
    + "should vary along the body, not this.");

  JSONObject waveWavelengthProp = aiNumberSchema(
    "how many joints make up ONE full sine cycle (peak-to-peak). Roughly "
    + "5-7 fits about 1.5-2 full S-bends across all 11 joints at once (a "
    + "snake/eel look); larger (8-11) gives a single long, gentle sweep "
    + "down the body. Needs to be at least ~4 - anything shorter can't be "
    + "resolved across joints that are each a fixed physical length.");

  JSONObject wavePeriodProp = aiNumberSchema(
    "milliseconds for the wave pattern to travel one full wavelength's "
    + "worth of distance down the chain - this is the motion's SPEED, not "
    + "a per-pose duration (there are no poses in this mode). 800-1400 for "
    + "a slow/graceful slither, 400-700 for a fast/urgent one.");

  JSONObject waveDirectionProp = aiNumberSchema(
    "+1 for the wave traveling from joint 1 (the base, fixed end) toward "
    + "joint 11 (the free tip) - a snake pushing itself forward tail-first "
    + "- or -1 for the reverse (tip toward base, more like something "
    + "being whipped from the free end). Use exactly 1 or -1.");

  JSONObject waveProps = new JSONObject();
  waveProps.setJSONObject("amplitude_degrees", waveAmplitudeProp);
  waveProps.setJSONObject("wavelength_joints", waveWavelengthProp);
  waveProps.setJSONObject("period_ms", wavePeriodProp);
  waveProps.setJSONObject("direction", waveDirectionProp);

  JSONObject travelingWaveProp = new JSONObject();
  travelingWaveProp.setString("type", "object");
  travelingWaveProp.setJSONObject("properties", waveProps);
  JSONArray waveRequired = new JSONArray();
  waveRequired.append("amplitude_degrees");
  waveRequired.append("wavelength_joints");
  waveRequired.append("period_ms");
  waveRequired.append("direction");
  travelingWaveProp.setJSONArray("required", waveRequired);
  travelingWaveProp.setString("description",
    "REQUIRED unless poses is given instead - give exactly one of the two. "
    + "Drives all 11 joints directly as a continuous sine wave that keeps "
    + "propagating along the chain, rather than cycling through a few "
    + "whole-body poses - use this for a snake/eel/worm/caterpillar, a "
    + "whip, a rippling flag/cloth edge, a tentacle, or anything else that "
    + "should look like a wave actually traveling down the body's length. "
    + "Every joint bends from its own straight (180°) position; there "
    + "is no separate shape to design here, no segments, no poses - just "
    + "these four numbers.");
  properties.setJSONObject("traveling_wave", travelingWaveProp);

  JSONObject jointStiffnessItem = new JSONObject();
  jointStiffnessItem.setString("type", "integer");
  jointStiffnessItem.setInt("minimum", 0);
  jointStiffnessItem.setInt("maximum", NUM_STIFFNESS_LEVELS - 1);

  JSONObject jointStiffnessProp = new JSONObject();
  jointStiffnessProp.setString("type", "array");
  jointStiffnessProp.setJSONObject("items", jointStiffnessItem);
  jointStiffnessProp.setInt("minItems", MOTOR_COUNT);
  jointStiffnessProp.setInt("maxItems", MOTOR_COUNT);
  jointStiffnessProp.setString("description",
    "OPTIONAL. Exactly 11 integers, one per joint in chain order (joint 1 "
    + "= the base, closest to the fixed mount, through joint 11 = the free "
    + "tip). Each is a stiffness LEVEL, not a raw gain: 0 = full/default "
    + "stiffness (precise, holds its angle firmly), up to " + (NUM_STIFFNESS_LEVELS - 1)
    + " = softest (loose, more passive give/wobble under its own momentum "
    + "or gravity). Applies for this animation's ENTIRE run, not pose to "
    + "pose. Omit entirely (or use all 0s) for a normal firm motion - most "
    + "tools and rigid shapes (hammer, square, racket, flat iron) should "
    + "stay fully stiff. Raise it on joints where the REAL object would "
    + "look/feel loose or trailing rather than rigid - e.g. a snake or "
    + "whip's last few joints softer than its base so the tail trails "
    + "behind rather than tracking the base's motion exactly, or a flag/"
    + "cloth corner. Don't soften a joint just because it's near the tip "
    + "if the real object is rigid there.");
  properties.setJSONObject("joint_stiffness", jointStiffnessProp);

  schema.setJSONObject("properties", properties);
  JSONArray required = new JSONArray();
  required.append("animation_name");
  // Deliberately NOT listing "frame_duration_ms", "poses" or
  // "traveling_wave" - exactly one of poses/traveling_wave is required
  // (frame_duration_ms only matters for poses, and defaults sanely if
  // left out), which plain JSON Schema's flat "required" can't express
  // (that would need oneOf). aiCallAndParseAnimation() enforces "exactly
  // one, and a valid one" itself after the call comes back.
  schema.setJSONArray("required", required);

  tool.setJSONObject("input_schema", schema);
  tool.setJSONObject("cache_control", aiCacheControlEphemeral());
  return tool;
}

// {"type": "ephemeral"} - the marker Anthropic's prompt caching API uses to
// say "cache everything up to and including this block." Pulled into one
// helper since it's identical wherever it's used (the tool definition here,
// the system prompt block below).
JSONObject aiCacheControlEphemeral() {
  JSONObject cacheControl = new JSONObject();
  cacheControl.setString("type", "ephemeral");
  return cacheControl;
}

// The system prompt as a single cached content block. The "system" field
// normally takes a plain string, but caching requires the structured
// array-of-blocks form with cache_control on the block to cache - AI_SYSTEM_
// PROMPT is ~5000 characters and, like the tool definition above, is
// IDENTICAL on every call in a shape-generation sequence, so this is the
// single biggest win for cutting latency on refinement rounds 2 and 3.
JSONArray aiBuildCachedSystemBlocks() {
  JSONObject systemBlock = new JSONObject();
  systemBlock.setString("type", "text");
  systemBlock.setString("text", AI_SYSTEM_PROMPT);
  systemBlock.setJSONObject("cache_control", aiCacheControlEphemeral());

  JSONArray blocks = new JSONArray();
  blocks.append(systemBlock);
  return blocks;
}

JSONObject aiBuildRequestBody(String prompt) {
  JSONObject body = new JSONObject();
  body.setString("model", ANTHROPIC_MODEL);
  body.setInt("max_tokens", 4096);
  body.setJSONArray("system", aiBuildCachedSystemBlocks());

  JSONArray tools = new JSONArray();
  tools.append(aiBuildToolDefinition());
  body.setJSONArray("tools", tools);

  // "auto" (not a forced "tool" choice) so Claude can write a few sentences
  // of reasoning about the object's proportions before committing to
  // segments, per the system prompt's instructions - aiCallAndParse()
  // already scans all content blocks for the tool_use one, so text before
  // it is simply ignored when parsing, no change needed there.
  JSONObject toolChoice = new JSONObject();
  toolChoice.setString("type", "auto");
  body.setJSONObject("tool_choice", toolChoice);

  // v7: tell Claude which calibrated presets exist, so a variant/description/
  // translation of one ("isosceles triangle", "a triangle shape") can be
  // routed to the tuned preset via library_match instead of being
  // re-invented. Kept in the user message (not the cached system prompt) so
  // saving a new library shape never invalidates the prompt cache.
  // v8: STRICT matching - only true synonyms/rephrasings/translations route
  // to a preset. Any qualifier that changes proportions or subtype
  // ("isosceles", "equilateral", "tall", "right-angled", "small"...) means
  // the user wants something different from the preset, so Claude must
  // design it fresh. Set AI_USE_LIBRARY_MATCH = false to turn routing off.
  String libraryNote = (!AI_USE_LIBRARY_MATCH || shapeLibrarySmall.isEmpty()) ? "" :
    "\n\n(Calibrated shapes already tuned on the real arm: " + shapeLibrarySmall.keySet()
    + ". Set library_match to one of these names ONLY if this request means exactly "
    + "that same shape with nothing added - a synonym, a rephrasing, or a translation "
    + "(e.g. 'a triangle shape' or '三角形' -> triangle). If the request adds ANY "
    + "qualifier that changes the shape's type or proportions - e.g. 'isosceles', "
    + "'equilateral', 'right-angled', 'tall', 'narrow', 'lopsided', 'with a handle' - do "
    + "NOT set library_match; design that specific shape with segments instead. When "
    + "unsure, leave library_match out.)";

  JSONObject userMessage = new JSONObject();
  userMessage.setString("role", "user");
  userMessage.setString("content", prompt + libraryNote);
  JSONArray messages = new JSONArray();
  messages.append(userMessage);
  body.setJSONArray("messages", messages);

  return body;
}

// Same shape as aiBuildRequestBody(), swapping in AI_ANIMATION_SYSTEM_PROMPT
// and the emit_shape_animation tool instead of the single-shape pair - kept
// as its own function rather than parameterizing aiBuildRequestBody(),
// since the two request bodies (and the cached-block cache_control attached
// to each) are already simple enough that sharing one path would add more
// branching than it saves.
JSONObject aiBuildAnimationRequestBody(String prompt) {
  JSONObject body = new JSONObject();
  body.setString("model", ANTHROPIC_MODEL);
  body.setInt("max_tokens", 4096);

  JSONObject systemBlock = new JSONObject();
  systemBlock.setString("type", "text");
  systemBlock.setString("text", AI_ANIMATION_SYSTEM_PROMPT);
  systemBlock.setJSONObject("cache_control", aiCacheControlEphemeral());
  JSONArray systemBlocks = new JSONArray();
  systemBlocks.append(systemBlock);
  body.setJSONArray("system", systemBlocks);

  JSONArray tools = new JSONArray();
  tools.append(aiBuildAnimationToolDefinition());
  body.setJSONArray("tools", tools);

  JSONObject toolChoice = new JSONObject();
  toolChoice.setString("type", "auto");
  body.setJSONObject("tool_choice", toolChoice);

  JSONObject userMessage = new JSONObject();
  userMessage.setString("role", "user");
  userMessage.setString("content", prompt);
  JSONArray messages = new JSONArray();
  messages.append(userMessage);
  body.setJSONArray("messages", messages);

  return body;
}

String aiCallAnthropic(String jsonBody) throws IOException {
  URL url = new URL(ANTHROPIC_API_URL);
  HttpURLConnection conn = (HttpURLConnection) url.openConnection();
  conn.setRequestMethod("POST");
  conn.setRequestProperty("Content-Type", "application/json");
  conn.setRequestProperty("x-api-key", ANTHROPIC_API_KEY);
  conn.setRequestProperty("anthropic-version", "2023-06-01");
  conn.setDoOutput(true);
  conn.getOutputStream().write(jsonBody.getBytes("UTF-8"));
  int code = conn.getResponseCode();
  InputStream stream = (code >= 200 && code < 300) ? conn.getInputStream() : conn.getErrorStream();
  BufferedReader reader = new BufferedReader(new InputStreamReader(stream, "UTF-8"));
  StringBuilder sb = new StringBuilder();
  String line;
  while ((line = reader.readLine()) != null) sb.append(line);
  reader.close();
  return sb.toString();
}

// Fit a Catmull-Rom spline through the AI's control points, resample it by
// arc length into `numMotors` waypoints, and turn the direction change at
// each waypoint into a slider-friendly degree value centered on 180 (your
// sliders' neutral position).
//
// Two things matter for matching what actually gets rendered/driven:
//  1. drawlineform3d() alternates each joint's rotation axis - odd motors
//     rotateZ (bend within the X-Y plane), even motors rotateY (bend within
//     the X-Z plane). Measuring every joint's turn in the X-Y plane only
//     (as before) threw away all Z curvature for half the joints. Each
//     joint's turn angle is now measured in its own actual bend plane.
//  2. drawlineform3d() also negates the rendered angle for motors
//     3, 6, 7, 8, 10 (their physical mounting is mirrored). Pre-flipping the
//     sign here cancels that out, so the curve you see on screen matches the
//     curve that was actually asked for instead of being mirrored at those
//     5 joints.
float[] aiAnglesFromCurve(ArrayList<PVector> curve, boolean closed, int numMotors) {
  ArrayList<PVector> way = aiResampleByArcLength(curve, closed, numMotors);
  int n = way.size();

  float[] degreesOut = new float[numMotors];
  for (int j = 0; j < numMotors; j++) {
    PVector a = way.get(aiIdx(j - 1, n, closed));
    PVector b = way.get(aiIdx(j,     n, closed));
    PVector c = way.get(aiIdx(j + 1, n, closed));

    int motorNumber = j + 1;
    int dxlId = motorNumber + 1;
    boolean bendsInXY = (motorNumber % 2 == 1); // matches drawlineform3d()'s rotateZ/rotateY split

    float angleIn  = aiPlaneAngle(b.x - a.x, b.y - a.y, b.z - a.z, bendsInXY);
    float angleOut = aiPlaneAngle(c.x - b.x, c.y - b.y, c.z - b.z, bendsInXY);
    float turnDeg  = degrees(aiAngleDiff(angleOut, angleIn));
    turnDeg = constrain(turnDeg, -AI_MAX_TURN_DEG, AI_MAX_TURN_DEG);

    boolean mirrored = aiIsMirroredMotor(dxlId);
    if (mirrored) turnDeg = -turnDeg;

    degreesOut[j] = 180 + turnDeg;
  }
  return degreesOut;
}

// Direction angle of (dx,dy,dz) projected onto the joint's actual bend
// plane: X-Y for joints that rotateZ, X-Z for joints that rotateY.
float aiPlaneAngle(float dx, float dy, float dz, boolean bendsInXY) {
  if (bendsInXY) return atan2(dy, dx);
  return atan2(dx, dz);
}

// Still used by aiAnglesFromCurve() below (the "rough initial guess" pass
// used only for world-space alignment scale/centroid, not the final fit).
int aiIdx(int i, int n, boolean closed) {
  if (closed) return ((i % n) + n) % n;
  return constrain(i, 0, n - 1);
}

// =====================================================================
//  Geometry Engine: turns Claude's structured shape-segment description
//  (family + parameters + how each segment attaches to the previous one)
//  into one continuous ArrayList<PVector> curve, so everything downstream
//  (align to world, corner-anchored resample, ICP refine, solve) works the
//  same regardless of how the curve was produced.
//  Every generator below was verified standalone in Python first (exact
//  arc length, exact constant radius of curvature, correct sign/direction
//  behavior) before being ported here.
// =====================================================================

// How finely drawlineform3d() subdivides each joint-to-joint span when
// smoothing the yellow achievable-curve overlay for a curved shape, and how
// tight that curve pulls toward its control points (0.5 = standard
// Catmull-Rom; lower values = looser/rounder, higher = tighter/more
// pronounced bends). Was previously just whatever Processing's own
// curveVertex() did internally with no way to tune either knob - restoring
// explicit control here is what fixes a curve that reads as "weak."
int AI_CURVE_RENDER_STEPS_PER_SEGMENT = 12;
float AI_CURVE_RENDER_TENSION = 0.5;

// Same Catmull-Rom machinery, but for building a SPLINE segment's actual
// GEOMETRY (aiGenerateSplineSegmentPoints below), not just how the final
// achievable curve is drawn - kept as its own constant since it may want
// different tuning than pure render smoothing later.
float AI_SPLINE_TENSION = 0.5;

// Catmull-Rom interpolation between p1 and p2 at parameter t (0..1), using
// p0/p3 as the outer points that shape the tangent at each end - standard
// cubic Hermite form so `tension` (normally 0.5) is an explicit, tunable
// knob instead of whatever a library's default happens to be.
PVector aiCatmullRom(PVector p0, PVector p1, PVector p2, PVector p3, float t, float tension) {
  float t2 = t * t;
  float t3 = t2 * t;
  PVector m1 = PVector.mult(PVector.sub(p2, p0), tension);
  PVector m2 = PVector.mult(PVector.sub(p3, p1), tension);
  float h00 = 2 * t3 - 3 * t2 + 1;
  float h10 = t3 - 2 * t2 + t;
  float h01 = -2 * t3 + 3 * t2;
  float h11 = t3 - t2;

  PVector result = new PVector(0, 0, 0);
  result.add(PVector.mult(p1, h00));
  result.add(PVector.mult(m1, h10));
  result.add(PVector.mult(p2, h01));
  result.add(PVector.mult(m2, h11));
  return result;
}

// Catmull-Rom through an open (non-looping) chain of points, `stepsPerSeg`
// samples per span, at the given tension - passes through every point in
// `pts` exactly (each is both the end of one span and the start of the
// next), just with `stepsPerSeg`x as many points actually drawn between
// them instead of leaving tessellation to whatever curveVertex() does on
// its own. Clamps the outer control point at each end of the chain (there's
// no p[-1] or p[n], so the endpoint itself doubles as its own outer
// control) rather than wrapping, since the arm is a fixed-base OPEN chain,
// never a loop.
ArrayList<PVector> aiCatmullRomChain(ArrayList<PVector> pts, int stepsPerSeg, float tension) {
  ArrayList<PVector> out = new ArrayList<PVector>();
  int n = pts.size();
  if (n < 2) return pts;

  for (int seg = 0; seg < n - 1; seg++) {
    PVector p0 = pts.get(max(0, seg - 1));
    PVector p1 = pts.get(seg);
    PVector p2 = pts.get(seg + 1);
    PVector p3 = pts.get(min(n - 1, seg + 2));
    for (int s = 0; s < stepsPerSeg; s++) {
      float t = s / float(stepsPerSeg);
      out.add(aiCatmullRom(p0, p1, p2, p3, t, tension));
    }
  }
  out.add(pts.get(n - 1));
  return out;
}

// Same tension-based cubic Hermite form as aiCatmullRom above, generalized
// from a single PVector to an arbitrary-length float[] - each element
// (here, one joint's angle) is interpolated completely independently
// through the same 4 pose keyframes and the same t/tension, since
// Catmull-Rom is just applied component-wise anyway. Used to build one
// continuous motion curve through an ANIMATION's whole pose sequence.
float[] aiCatmullRomVec(float[] p0, float[] p1, float[] p2, float[] p3, float t, float tension) {
  int n = p1.length;
  float[] result = new float[n];

  float t2 = t * t;
  float t3 = t2 * t;
  float h00 = 2 * t3 - 3 * t2 + 1;
  float h10 = t3 - 2 * t2 + t;
  float h01 = -2 * t3 + 3 * t2;
  float h11 = t3 - t2;

  for (int i = 0; i < n; i++) {
    float m1 = (p2[i] - p0[i]) * tension;
    float m2 = (p3[i] - p1[i]) * tension;
    result[i] = p1[i] * h00 + m1 * h10 + p2[i] * h01 + m2 * h11;
  }
  return result;
}

// Samples a CLOSED loop through an animation's poses at overall progress u
// (0..1, wrapping) - unlike aiCatmullRomChain (an OPEN chain, clamped at
// both ends, since the physical arm itself is a fixed-base open chain),
// this wraps the pose INDICES cyclically, so the span from the last pose
// back to the first gets a real, continuous tangent too instead of being
// clamped/stopped. For exactly 2 poses, every control point collapses to
// one of the two poses, both tangents come out to exactly zero, and the
// formula quietly reduces to a plain eased lerp between them - the
// correct, deliberate degenerate case for a simple back-and-forth motion,
// not a bug.
float[] aiSampleAnimationLoop(ArrayList<float[]> poses, float u, float tension) {
  int n = poses.size();
  float scaled = u * n;
  int seg = ((int) floor(scaled)) % n;
  if (seg < 0) seg += n; // defensive - u should already be in [0,1), but keep this safe if it ever isn't
  float t = scaled - floor(scaled);

  float[] p0 = poses.get(((seg - 1) % n + n) % n);
  float[] p1 = poses.get(seg);
  float[] p2 = poses.get((seg + 1) % n);
  float[] p3 = poses.get((seg + 2) % n);

  return aiCatmullRomVec(p0, p1, p2, p3, t, tension);
}

// Drives all 11 joints directly as one continuous sine wave, rather than
// gliding between a few whole-body poses - this is what makes a snake/eel/
// whip/flag actually look like a wave TRAVELING down the body, instead of
// the body flexing in and out together in place. phase advances
// unboundedly with time (it's only ever used inside sin(), which wraps it
// on its own), so there's no modulo bookkeeping and no seam/jump ever,
// unlike aiSampleAnimationLoop's pose-index cycling above - this is a
// single closed-form function of (joint position, time), continuous in
// both.
//
//   angle[i] = 180 + amplitude * sin(2*PI * (jointPos/wavelengthJoints -
//              direction * phase))
//
// At any FIXED instant, that traces a real sine shape across the 11
// joints (a spatial wave); as phase increases, the whole pattern slides
// along the joint axis, which is exactly what a wave traveling down a
// body looks like. Clamped to the same mechanical safety limits used
// everywhere else motor angles get set directly (60-310), since this
// bypasses aiGenerateFromSegments()'s own clamp entirely.
float[] aiSampleTravelingWave(float amplitudeDeg, float wavelengthJoints, float phase, float direction) {
  float[] result = new float[MOTOR_COUNT];
  for (int i = 0; i < MOTOR_COUNT; i++) {
    float jointPos = i + 1; // 1..11, matching the hardware/HUD numbering convention used everywhere else
    float theta = TWO_PI * (jointPos / wavelengthJoints - direction * phase);
    float angle = 180 + amplitudeDeg * sin(theta);
    result[i] = constrain(angle, 60, 310);
  }
  return result;
}

// An arbitrary vector perpendicular to d, used when no usable hint was
// given to orthogonalize against.
PVector aiAnyPerpendicular(PVector d) {
  PVector ref = (abs(d.z) < 0.9) ? new PVector(0, 0, 1) : new PVector(1, 0, 0);
  PVector p = d.cross(ref);
  p.normalize();
  return p;
}

// Removes hint's component along d and normalizes what's left - used to
// turn a rough, not-necessarily-perpendicular "which way does this bend"
// hint from Claude into a clean, robust perpendicular axis.
PVector aiOrthogonalize(PVector hint, PVector d) {
  PVector dn = d.copy();
  dn.normalize();
  float along = hint.dot(dn);
  PVector perp = PVector.sub(hint, PVector.mult(dn, along));
  if (perp.mag() < 0.000001) return aiAnyPerpendicular(dn);
  perp.normalize();
  return perp;
}

// Straight segment.
ArrayList<PVector> aiGenerateLineSegmentPoints(PVector p0, PVector direction, float length, int samples) {
  ArrayList<PVector> pts = new ArrayList<PVector>();
  PVector d0 = direction.copy();
  d0.normalize();
  pts.add(p0.copy());
  for (int i = 1; i <= samples; i++) {
    float s = length * i / float(samples);
    pts.add(PVector.add(p0, PVector.mult(d0, s)));
  }
  return pts;
}

// Constant-curvature bend (a segment of a circle). curvature == 0 falls
// back to a straight line. Sign of curvature picks which way it bends
// (toward curveToward for positive, away for negative); magnitude sets how
// tight (radius = 1/|curvature|). Verified standalone: exact arc length,
// exact constant radius of curvature, both signs move forward correctly.
ArrayList<PVector> aiGenerateArcSegmentPoints(PVector p0, PVector direction, float length, float curvature, PVector curveToward, int samples) {
  ArrayList<PVector> pts = new ArrayList<PVector>();
  PVector d0 = direction.copy();
  d0.normalize();
  pts.add(p0.copy());

  if (abs(curvature) < 0.0001) {
    for (int i = 1; i <= samples; i++) {
      float s = length * i / float(samples);
      pts.add(PVector.add(p0, PVector.mult(d0, s)));
    }
    return pts;
  }

  PVector B = aiOrthogonalize(curveToward, d0);
  float R = 1.0 / abs(curvature);
  float bendSign = curvature > 0 ? 1.0 : -1.0;

  for (int i = 1; i <= samples; i++) {
    float s = length * i / float(samples);
    float theta = s / R;
    PVector p = PVector.add(p0, PVector.add(
      PVector.mult(B, bendSign * R * (1 - cos(theta))),
      PVector.mult(d0, R * sin(theta))
    ));
    pts.add(p);
  }
  return pts;
}

// Closed circle/oval. `direction` is the tangent at the loop's own start
// point (p0), `hint` picks which way the loop bulges away from that
// tangent, `aspect` stretches it into an oval (1.0 = circle).
//
// Used to add a small out-of-plane wobble here on the theory that a flat
// loop wastes whichever joint family (odd = local X-Y, even = local X-Z)
// doesn't match its plane. The calibrated preset library disproved that:
// every hand-verified planar shape (circle, square, zigzag, hook) works by
// putting nearly ALL the bend into ONE joint family and leaving the other
// close to its 180 deg neutral - not by splitting curvature evenly across
// both. That works because each joint's local axes have already been
// rotated by every joint before it, so a chain of same-family rotations
// still traces a genuinely curved 3D path on its own; the "wasted half"
// intuition was wrong. Kept flat again to match what's actually verified.
ArrayList<PVector> aiGenerateLoopSegmentPoints(PVector p0, PVector direction, PVector hint, float radius, float aspect, int samples) {
  ArrayList<PVector> pts = new ArrayList<PVector>();
  PVector V = direction.copy();
  V.normalize();
  PVector U = aiOrthogonalize(hint, V);
  PVector C = PVector.sub(p0, PVector.mult(U, radius));

  for (int i = 0; i <= samples; i++) {
    float theta = TWO_PI * i / float(samples);
    PVector p = PVector.add(C, PVector.add(
      PVector.mult(U, radius * cos(theta)),
      PVector.mult(V, radius * aspect * sin(theta))
    ));
    pts.add(p);
  }
  return pts;
}

// Helix. `axis` is the direction the coil advances along; `hint` picks the
// starting radial direction (orthogonalized against axis).
ArrayList<PVector> aiGenerateSpiralSegmentPoints(PVector p0, PVector axis, PVector hint, float radius, float turns, float pitch, int samples) {
  ArrayList<PVector> pts = new ArrayList<PVector>();
  PVector A = axis.copy();
  A.normalize();
  PVector U = aiOrthogonalize(hint, A);
  PVector V = A.cross(U);
  V.normalize();
  PVector C = PVector.sub(p0, PVector.mult(U, radius));

  float totalTheta = turns * TWO_PI;
  for (int i = 0; i <= samples; i++) {
    float theta = totalTheta * i / float(samples);
    PVector p = PVector.add(C, PVector.add(
      PVector.add(PVector.mult(U, radius * cos(theta)), PVector.mult(V, radius * sin(theta))),
      PVector.mult(A, (theta / TWO_PI) * pitch)
    ));
    pts.add(p);
  }
  return pts;
}

// Freeform curve through Claude-declared control points, each a RELATIVE
// offset from this segment's own start point p0 (so, like every other
// family, the segment is translation-independent regardless of where in
// the chain it ends up attaching). Reuses the exact same Catmull-Rom
// machinery that smooths the achievable-curve rendering
// (aiCatmullRom/aiCatmullRomChain) - here it's not cosmetic, it IS the
// segment's actual generated geometry. Passes through p0 and every
// declared point exactly, in the order given.
ArrayList<PVector> aiGenerateSplineSegmentPoints(PVector p0, ArrayList<PVector> controlPoints, int totalSamples) {
  ArrayList<PVector> waypoints = new ArrayList<PVector>();
  waypoints.add(p0.copy());
  for (PVector offset : controlPoints) {
    waypoints.add(PVector.add(p0, offset));
  }
  if (waypoints.size() < 2) {
    ArrayList<PVector> single = new ArrayList<PVector>();
    single.add(p0.copy());
    return single;
  }
  int stepsPerSpan = max(2, totalSamples / (waypoints.size() - 1));
  return aiCatmullRomChain(waypoints, stepsPerSpan, AI_SPLINE_TENSION);
}

// Everything aiAssembleSegments produces: the final continuous curve, plus
// the exact points where one segment meets the next (known corners - no
// angle-threshold guessing needed, since we built the curve and know
// exactly where the joins are).
class AISegmentAssembly {
  ArrayList<PVector> curve = new ArrayList<PVector>();
  ArrayList<PVector> corners = new ArrayList<PVector>();
}

// Detects any handle+head structure - a segment with attach_position < 1.0
// (a branch off the previous segment) - and repairs its proportions and
// orientation in place if they drift from what actually reads as
// recognizable, regardless of whether Claude's prompt guidance was
// followed. Deliberately general (triggers on the attach_position < 1.0
// pattern itself, not on the word "hammer"), so it also helps screwdriver/
// wrench/spoon/key and anything else with the same handle+head shape.
// Mutates `segments` in place; prints each repair it makes.
void aiValidateAndRepairSegments(JSONArray segments, boolean closed) {
  // SPLINE needs at least 2 control_points to define any shape at all - a
  // missing/malformed list would otherwise collapse this segment down to a
  // single point (p0 alone), silently breaking every corner/joint-count
  // calculation downstream. Back-fill a straight continuation in the
  // segment's own `direction` instead, so a bad spline degrades to "looks
  // like a plain line" rather than "breaks the shape."
  for (int i = 0; i < segments.size(); i++) {
    JSONObject seg = segments.getJSONObject(i);
    String fam = seg.hasKey("family") ? seg.getString("family") : "line";
    if (!fam.equals("spline")) continue;
    JSONArray cps = seg.hasKey("control_points") ? seg.getJSONArray("control_points") : null;
    if (cps == null || cps.size() < 2) {
      aiLog("  [repair] segment " + i + ": spline had fewer than 2 control_points - "
        + "falling back to a straight line so the shape doesn't collapse to a point");
      PVector dir = aiJsonVec3(seg, "direction", new PVector(1, 0, 0));
      dir.normalize();
      float fallbackLength = aiJsonFloat(seg, "length", 5.0);
      JSONArray fallbackPts = new JSONArray();
      JSONArray p1 = new JSONArray();
      p1.append(dir.x * fallbackLength * 0.5);
      p1.append(dir.y * fallbackLength * 0.5);
      p1.append(dir.z * fallbackLength * 0.5);
      JSONArray p2 = new JSONArray();
      p2.append(dir.x * fallbackLength);
      p2.append(dir.y * fallbackLength);
      p2.append(dir.z * fallbackLength);
      fallbackPts.append(p1);
      fallbackPts.append(p2);
      seg.setJSONArray("control_points", fallbackPts);
    }
  }

  // v7: the handle+head / staple / bridge repairs below are built for OPEN
  // tool-like shapes. A closed polygon's sides meet at sharp angles on
  // purpose, so these rules used to snap its corners to 90 deg or shrink a
  // side until it no longer closed. Skipped entirely for closed shapes
  // (the spline fallback above and spiral cap below still always run).
  for (int i = 1; !closed && i < segments.size(); i++) {
    JSONObject head = segments.getJSONObject(i);
    JSONObject handle = segments.getJSONObject(i - 1);
    float attachPos = aiJsonFloat(head, "attach_position", 1.0);

    // A segment counts as a "head" needing repair if it branches off before
    // the handle's end (attach_position < 1.0) OR points in a meaningfully
    // different direction than the handle even when it attaches exactly at
    // the tip (attach_position == 1.0) - Claude sometimes emits a hammer
    // head that way even though the prompt asks for ~0.85-0.95. Checking
    // attach_position alone missed that case entirely and let an undersized,
    // unrepaired head through.
    PVector handleDir = aiJsonVec3(handle, "direction", new PVector(1, 0, 0));
    PVector headDir = aiJsonVec3(head, "direction", aiAnyPerpendicular(handleDir));
    PVector handleDirN = handleDir.copy();
    handleDirN.normalize();
    PVector headDirN = headDir.copy();
    headDirN.normalize();
    float angleDeg = degrees(acos(constrain(handleDirN.dot(headDirN), -1, 1)));
    boolean looksLikeHead = attachPos < 0.999 || angleDeg >= 45;
    if (!looksLikeHead) continue; // a genuine straight/gentle continuation (e.g. a wave), not a head

    String headFamily = head.hasKey("family") ? head.getString("family") : "line";
    String handleFamily = handle.hasKey("family") ? handle.getString("family") : "line";

    // SPLINE segments carry their own exact geometry via control_points -
    // none of the primitive-specific repairs below (attach-position
    // targets tuned for LINE/ARC heads, size floors/ceilings keyed off a
    // "length"/"radius" field splines don't have, perpendicularity
    // snapping against a "direction" a spline's real curve may not follow)
    // apply cleanly to them, on either side of the pair - a spline HANDLE's
    // declared direction is no more trustworthy as its real exit tangent
    // than an arc's is (see handleIsArc below), and a spline HEAD's "size"
    // isn't expressed by any single field at all. Only the generic
    // attach-near-the-end nudge still makes sense, so it's reapplied here
    // directly instead of falling through into the family-specific logic.
    if (headFamily.equals("spline") || handleFamily.equals("spline")) {
      if (attachPos < 0.8) {
        aiLog("  [repair] segment " + i + ": attach_position " + nf(attachPos, 1, 2)
          + " -> 0.85 (a head attached partway through a spline, or a spline "
          + "head, should still attach near the handle's end)");
        head.setFloat("attach_position", 0.85);
      }
      continue;
    }

    // A segment that attaches at exactly 1.0 (a smooth continuation, never
    // cut short), is the SAME family as what it's attached to, and is
    // roughly the same SIZE as it, is the signature of a symmetric
    // multi-lobe curve - an S-curve, a wave's bigger swings, a bowtie/pair
    // of mirrored arcs - NOT a handle+head tool, even though its direction
    // differs by 45+ degrees (that's exactly what a MIRRORED second lobe
    // looks like; an S-curve's two arc directions are ~180 degrees apart).
    // (A heart is no longer built from two arcs at all - see AI_SYSTEM_
    // PROMPT's SPLINE-based heart recipe - but this exemption still matters
    // for any other symmetric arc pair Claude proposes.) The repair rules
    // below (attach-position nudging toward the
    // handle's end, and especially the terminal-head size ceiling a few
    // lines down) are built for a tool head that's meant to be SMALLER than
    // its handle - applying that ceiling here shrinks one of two intended
    // twin lobes down to a fraction of its partner's length, destroying the
    // very symmetry that makes the shape read as a heart/S rather than a
    // lopsided hook. This was confirmed in the debug log: every heart and
    // "letter S" attempt had its second arc silently cut to ~65% of the
    // first's length by this repair, even though Claude proposed the two
    // arcs with equal length and curve_amount on purpose.
    if (attachPos >= 0.999 && headFamily.equals(handleFamily)) {
      float handleMagnitude = aiJsonFloat(handle, "length", 5.0);
      float headMagnitude = aiJsonFloat(head, "length", 5.0);
      float magnitudeRatio = min(handleMagnitude, headMagnitude) / max(max(handleMagnitude, headMagnitude), 0.001);
      if (magnitudeRatio >= 0.6) continue; // comparable size, same family, full attach - a twin lobe, not a head
    }

    boolean headIsRadial = headFamily.equals("loop") || headFamily.equals("spiral");

    // 1. The head should attach near the END of the handle, not partway
    // through or at the middle - a hammer's head isn't halfway up the shaft.
    // LOOP/SPIRAL heads are the exception: going all the way around a loop
    // costs far more joints than a line head needs just to look distinct
    // (a hammer head only has to stick out; a racket face has to close a
    // full circle), so a loop needs to start much earlier and claim most of
    // the remaining chain, not attach at the very tip of a nearly-full-length
    // handle.
    float targetAttach = headIsRadial ? 0.55 : 0.85;
    if ((headIsRadial && (attachPos < 0.4 || attachPos > 0.7)) || (!headIsRadial && attachPos < 0.8)) {
      aiLog("  [repair] segment " + i + ": attach_position " + nf(attachPos, 1, 2)
        + " -> " + nf(targetAttach, 1, 2) + (headIsRadial
          ? " (a loop/spiral head needs most of the remaining chain to close)"
          : " (head should attach near the handle's end)"));
      head.setFloat("attach_position", targetAttach);
      attachPos = targetAttach;
    }

    // 2. A TERMINAL head (nothing attaches after it) needs to be a large
    // enough fraction of the WHOLE curve to claim multiple physical joints -
    // the arm only has 11 rigid segments, so a "realistic" tool head
    // (10-20% of the handle, per the prompt's own example) works out to
    // under 2 of them and reads as barely a stub no matter how correct the
    // proportions are on paper. Enforce a floor (relative to the kept
    // portion of the handle) big enough to reliably claim ~3-4 joints, in
    // addition to the existing ceiling that keeps it from turning into a
    // second handle. LINE/ARC heads are sized by "length"; LOOP/SPIRAL
    // heads (e.g. a tennis racket's face) are sized by "radius" instead - a
    // head using the wrong field silently passes this check untouched, so
    // branch on family and compare/repair the field that actually controls
    // that family's rendered size. Loop/spiral size is expressed as
    // diameter (2 * radius) so it's on the same "extent along the handle's
    // direction" scale as a line/arc head's length. A loop's ceiling is set
    // much higher than a line head's - closing a full circle needs to
    // dominate the curve's arc length, not just be "noticeably bigger than
    // the handle" the way a hammer head does.
    //
    // A NON-terminal "head" (something else attaches after it, e.g. the
    // short bridging segment in a tongs/staple's line-connector-line
    // pattern) is a fundamentally different role - it's meant to be SHORT,
    // just enough to span one joint's gap between two arms, not big enough
    // to claim several joints itself. Forcing the same floor onto it would
    // inflate a deliberately short connector into something proportioned
    // like a hammer head, breaking the staple shape it's part of. Skip
    // entirely for non-terminal segments.
    //
    // A terminal segment can ALSO be the closing arm of that same
    // line-connector-line staple pattern - it looks "terminal" (nothing
    // attaches after it) but its correct size reference is the FIRST arm
    // (two segments back), not its immediate "handle" (the short
    // connector) - sizing it against a deliberately-short 1-joint bridge
    // is exactly what clamped a proper ~7-unit closing arm down to under
    // 1. Detected by: two segments back exists and points roughly
    // opposite (>135 deg) this one - the staple's own signature, same
    // threshold as the prompt's "anti-parallel" guidance.
    boolean isStapleClosingArm = false;
    if (i >= 2) {
      PVector farDir = aiJsonVec3(segments.getJSONObject(i - 2), "direction", new PVector(1, 0, 0));
      PVector farDirN = farDir.copy();
      farDirN.normalize();
      float farAngle = degrees(acos(constrain(farDirN.dot(headDirN), -1, 1)));
      isStapleClosingArm = farAngle > 135;
    }
    boolean isTerminalHead = (i == segments.size() - 1) && !isStapleClosingArm;
    if (isTerminalHead) {
      float handleLength = aiJsonFloat(handle, "length", 5.0);
      float keptHandleLength = handleLength * attachPos;
      String extentField = headIsRadial ? "radius" : "length";
      float headExtent = headIsRadial
        ? aiJsonFloat(head, "radius", 1.5) * 2.0
        : aiJsonFloat(head, "length", 1.0);
      float minHeadExtent = keptHandleLength * (headIsRadial ? 0.9 : 0.4);
      float maxHeadExtent = keptHandleLength * (headIsRadial ? 1.4 : 0.65);
      if (headExtent < minHeadExtent) {
        aiLog("  [repair] segment " + i + ": " + extentField + " " + nf(headExtent, 1, 2) + " -> "
          + nf(minHeadExtent, 1, 2) + " extent (head was too small to read as a distinct part on "
          + "an 11-joint arm)");
        head.setFloat(extentField, headIsRadial ? minHeadExtent / 2.0 : minHeadExtent);
      } else if (headExtent > maxHeadExtent) {
        aiLog("  [repair] segment " + i + ": " + extentField + " " + nf(headExtent, 1, 2) + " -> "
          + nf(maxHeadExtent, 1, 2) + " extent (head should be shorter than the handle)");
        head.setFloat(extentField, headIsRadial ? maxHeadExtent / 2.0 : maxHeadExtent);
      }
    }

    // 2b. A staple's BRIDGE segment (this one, non-terminal, with a next
    // segment pointing back roughly opposite the handle - the same
    // signature as isStapleClosingArm, checked one segment earlier) has a
    // length requirement the generic min/max logic above doesn't capture:
    // its length AS A FRACTION OF THE WHOLE PATH needs to land close to
    // 1/11, not just "short." The whole curve gets uniformly rescaled by
    // total-path-length to match the arm's total reach - so if the bridge
    // is any other fraction, it scales to something other than one
    // physical link, and the joint in the middle of it (which is supposed
    // to stay straight) has to bend to make up the difference. That one
    // deviation throws off the chain's accumulated position for every
    // joint after it, compounding into exactly the widespread clamping
    // seen on "stapler"/"flat iron" attempts. Solving bridge/(arm1+bridge+
    // arm2) = 1/11 for bridge gives (arm1+arm2)/10.
    if (i + 1 < segments.size() && !headIsRadial
      && (handle.hasKey("family") ? handle.getString("family") : "line").equals("line")) {
      JSONObject nextSeg = segments.getJSONObject(i + 1);
      String nextFam = nextSeg.hasKey("family") ? nextSeg.getString("family") : "line";
      if (nextFam.equals("line")) {
        PVector nextDirN = aiJsonVec3(nextSeg, "direction", new PVector(1, 0, 0)).copy();
        nextDirN.normalize();
        float bridgeSignatureAngle = degrees(acos(constrain(handleDirN.dot(nextDirN), -1, 1)));
        if (bridgeSignatureAngle > 135) {
          float armLength = aiJsonFloat(handle, "length", 5.0);
          float nextArmLength = aiJsonFloat(nextSeg, "length", 5.0);
          float idealBridgeLength = (armLength + nextArmLength) / 10.0;
          float bridgeLength = aiJsonFloat(head, "length", 1.0);
          if (abs(bridgeLength - idealBridgeLength) > 0.15 * idealBridgeLength) {
            aiLog("  [repair] segment " + i + ": length " + nf(bridgeLength, 1, 2) + " -> "
              + nf(idealBridgeLength, 1, 2) + " (bridge must scale to almost exactly one physical "
              + "link or the joint between the two turns has to bend to compensate, throwing off "
              + "every joint after it)");
            head.setFloat("length", idealBridgeLength);
          }
        }
      }
    }

    // 2c. An ARC whose own handle (the previous segment) is a LINE, not
    // another arc, isn't part of a multi-arc wave chain - it's meant to be
    // a single prominent curved feature (a hook, a scoop, a telephone
    // handset's C-curve), and its total sweep (length * curve_amount)
    // needs to be substantial enough to actually read as curved. A gentle
    // curve_amount like 0.1 over a length-5 arc sweeps under 30 degrees -
    // barely a bend - and the shape reads as a sequence of kinked
    // near-straight segments instead of one smooth curve. Doesn't touch
    // wave-style arcs (chained after another arc), where small alternating
    // curve_amounts are the whole point.
    String headFamForSweep = head.hasKey("family") ? head.getString("family") : "line";
    String handleFamForSweep = handle.hasKey("family") ? handle.getString("family") : "line";
    if (headFamForSweep.equals("arc") && handleFamForSweep.equals("line")) {
      float curveAmount2 = aiJsonFloat(head, "curve_amount", 0.1);
      float arcLength = aiJsonFloat(head, "length", 5.0);
      float sweep = abs(curveAmount2) * arcLength;
      float minSweep = radians(75);
      if (sweep < minSweep) {
        float newCurveAmount = (minSweep / max(arcLength, 0.5)) * (curveAmount2 < 0 ? -1 : 1);
        aiLog("  [repair] segment " + i + ": curve_amount " + nf(curveAmount2, 1, 3) + " -> "
          + nf(newCurveAmount, 1, 3) + " (sweep was only " + nf(degrees(sweep), 1, 1)
          + " deg - too gentle to read as a curved feature)");
        head.setFloat("curve_amount", newCurveAmount);
      }
    }

    // 3. The head should point roughly perpendicular to the handle - close
    // to continuing straight would just look like a bent handle, not a
    // distinct head. Snap it to exactly perpendicular (keeping its general
    // "which way" intent via aiOrthogonalize) if it drifts too far from
    // that. Skip for LOOP/SPIRAL heads (their "direction" is one of the
    // shape's own in-plane axes, not a travel-away direction) AND skip
    // whenever the HANDLE itself is an ARC - `handleDir` is only the arc's
    // STARTING tangent, not where the curve actually points by the time
    // the head attaches to its end, so comparing the head's direction
    // against it measures the wrong thing entirely. A bowl's line-arc-line
    // (short rim, the bowl's actual curve, closing rim) hit exactly this:
    // the closing rim's direction was correct for continuing the bowl's
    // curve, but got judged against the arc's start and force-snapped to
    // ~90 deg, breaking the shape.
    boolean handleIsArc = handle.hasKey("family") && handle.getString("family").equals("arc");
    if (!headIsRadial && !handleIsArc && (angleDeg < 60 || angleDeg > 120)) {
      PVector fixed = aiOrthogonalize(headDir, handleDir);
      aiLog("  [repair] segment " + i + ": direction was " + nf(angleDeg, 1, 1)
        + " deg from the handle -> snapped to ~90 deg");
      JSONArray fixedArr = new JSONArray();
      fixedArr.append(fixed.x);
      fixedArr.append(fixed.y);
      fixedArr.append(fixed.z);
      head.setJSONArray("direction", fixedArr);
    }
  }

  // Spiral turn-count budget - applies to EVERY spiral segment, including a
  // standalone one (a "pencil holder" or "coil spring" with no handle,
  // segments.size() == 1), which the loop above never reaches since it only
  // walks segments[1..] looking for a head branching off a previous one.
  // Turn count isn't a size/proportion problem the way head length or loop
  // diameter are - shrinking or growing a 5-turn coil doesn't change how
  // many turns it has. It's a pure joint-budget problem: tracing even ONE
  // recognizable turn takes several joints, so 11 joints total can resolve
  // at most about 1.5 turns before consecutive joints have to double back
  // on each other to fit more revolutions in, which is what turns a coil
  // into the tangled knot near the origin instead of a spiral. Cap it
  // regardless of where in the segment list it appears.
  for (int i = 0; i < segments.size(); i++) {
    JSONObject seg = segments.getJSONObject(i);
    String fam = seg.hasKey("family") ? seg.getString("family") : "line";
    if (!fam.equals("spiral")) continue;
    float turns = aiJsonFloat(seg, "turns", 2.0);
    float maxTurns = 1.5;
    if (turns > maxTurns) {
      aiLog("  [repair] segment " + i + ": turns " + nf(turns, 1, 2) + " -> " + nf(maxTurns, 1, 2)
        + " (11 joints can only resolve about 1.5 turns before it knots up instead of coiling)");
      seg.setFloat("turns", maxTurns);
    } else if (turns < 0.6) {
      aiLog("  [repair] segment " + i + ": turns " + nf(turns, 1, 2) + " -> 0.6 (too few turns to "
        + "read as a coil rather than a bent line)");
      seg.setFloat("turns", 0.6);
    }
  }
}

// Turns Claude's segment list into one continuous curve. Each segment
// always attaches to the immediately PREVIOUS segment (not an arbitrary
// earlier one) - a single 11-joint serial arm can't physically trace a
// true fork, so "attach_position" (0-1, how far along the previous
// segment's own points this one starts) either continues smoothly at 1.0,
// or - for a T-shape/branch like a hammer's head - truncates the previous
// segment there and the path turns into the new segment instead of
// continuing straight. That keeps the result always a single physically
// traceable path, exactly matching what an 11-joint chain can actually do.
AISegmentAssembly aiAssembleSegments(JSONArray segments) {
  AISegmentAssembly result = new AISegmentAssembly();

  ArrayList<PVector> prevSegmentPoints = null;

  for (int i = 0; i < segments.size(); i++) {
    JSONObject seg = segments.getJSONObject(i);
    String family = seg.hasKey("family") ? seg.getString("family") : "line";

    PVector p0;
    if (i == 0 || prevSegmentPoints == null) {
      p0 = new PVector(0, 0, 0);
    } else {
      float attachPos = seg.hasKey("attach_position") ? seg.getFloat("attach_position") : 1.0;
      attachPos = constrain(attachPos, 0.0, 1.0);
      int cutIndex = round(attachPos * (prevSegmentPoints.size() - 1));
      p0 = prevSegmentPoints.get(cutIndex).copy();

      // Truncate what was already appended for the previous segment back
      // to the attach point, so the path doesn't jump past a branch point
      // before turning into this new segment. The previous segment's
      // points are exactly the tail of result.curve (they were just
      // appended last iteration), so remove from the end until only
      // `keep` of them remain.
      int keep = cutIndex + 1;
      int removeCount = prevSegmentPoints.size() - keep;
      for (int r = 0; r < removeCount && result.curve.size() > 0; r++) {
        result.curve.remove(result.curve.size() - 1);
      }
      result.corners.add(p0.copy());
    }

    PVector direction = aiJsonVec3(seg, "direction", new PVector(1, 0, 0));
    int samples = 60;

    ArrayList<PVector> segPoints;
    if (family.equals("arc")) {
      float length = aiJsonFloat(seg, "length", 5.0);
      float curvature = aiJsonFloat(seg, "curve_amount", 0.0);
      PVector curveToward = aiJsonVec3(seg, "curve_toward", aiAnyPerpendicular(direction));
      segPoints = aiGenerateArcSegmentPoints(p0, direction, length, curvature, curveToward, samples);
    } else if (family.equals("loop")) {
      float radius = aiJsonFloat(seg, "radius", 3.0);
      float aspect = aiJsonFloat(seg, "aspect", 1.0);
      PVector hint = aiJsonVec3(seg, "curve_toward", aiAnyPerpendicular(direction));
      segPoints = aiGenerateLoopSegmentPoints(p0, direction, hint, radius, aspect, samples);
    } else if (family.equals("spiral")) {
      float radius = aiJsonFloat(seg, "radius", 3.0);
      float turns = aiJsonFloat(seg, "turns", 2.0);
      float pitch = aiJsonFloat(seg, "pitch", 2.0);
      PVector hint = aiJsonVec3(seg, "curve_toward", aiAnyPerpendicular(direction));
      segPoints = aiGenerateSpiralSegmentPoints(p0, direction, hint, radius, turns, pitch, samples);
    } else if (family.equals("spline")) {
      ArrayList<PVector> controlPoints = aiJsonVec3Array(seg, "control_points");
      segPoints = aiGenerateSplineSegmentPoints(p0, controlPoints, samples);
    } else { // "line" (and unrecognized families fall back to a straight line)
      float length = aiJsonFloat(seg, "length", 5.0);
      segPoints = aiGenerateLineSegmentPoints(p0, direction, length, samples);
    }

    result.curve.addAll(segPoints);
    prevSegmentPoints = segPoints;
  }

  return result;
}

// Reads a 3-element [x,y,z] array field into a PVector, or returns
// fallback if the field is missing/malformed.
PVector aiJsonVec3(JSONObject obj, String key, PVector fallback) {
  if (!obj.hasKey(key)) return fallback;
  JSONArray arr = obj.getJSONArray(key);
  if (arr == null || arr.size() != 3) return fallback;
  float x = arr.getFloat(0), y = arr.getFloat(1), z = arr.getFloat(2);
  if (Float.isNaN(x) || Float.isNaN(y) || Float.isNaN(z)) return fallback;
  PVector v = new PVector(x, y, z);
  if (v.mag() < 0.000001) return fallback;
  return v;
}

float aiJsonFloat(JSONObject obj, String key, float fallback) {
  if (!obj.hasKey(key)) return fallback;
  float v = obj.getFloat(key);
  return Float.isNaN(v) ? fallback : v;
}

// Reads a "control_points"-style field - a JSON array of [x,y,z] arrays -
// into a list of PVectors. Skips (rather than aborts on) any malformed
// entry, so one bad point doesn't take down the whole shape.
ArrayList<PVector> aiJsonVec3Array(JSONObject obj, String key) {
  ArrayList<PVector> pts = new ArrayList<PVector>();
  if (!obj.hasKey(key)) return pts;
  JSONArray arr = obj.getJSONArray(key);
  if (arr == null) return pts;
  for (int i = 0; i < arr.size(); i++) {
    JSONArray p = arr.getJSONArray(i);
    if (p == null || p.size() != 3) continue;
    float x = p.getFloat(0), y = p.getFloat(1), z = p.getFloat(2);
    if (Float.isNaN(x) || Float.isNaN(y) || Float.isNaN(z)) continue;
    pts.add(new PVector(x, y, z));
  }
  return pts;
}

ArrayList<PVector> aiResampleByArcLength(ArrayList<PVector> curve, boolean closed, int numMotors) {
  ArrayList<PVector> result = new ArrayList<PVector>();
  int n = curve.size();
  if (n < 2) return result;
  float[] cum = new float[n];
  cum[0] = 0;
  for (int i = 1; i < n; i++) cum[i] = cum[i - 1] + PVector.dist(curve.get(i - 1), curve.get(i));
  float total = cum[n - 1] + (closed ? PVector.dist(curve.get(n - 1), curve.get(0)) : 0);
  int waypoints = numMotors + 1;
  int denom = closed ? waypoints : waypoints - 1;
  for (int w = 0; w < waypoints; w++) {
    float target = total * w / float(denom);
    result.add(aiPointAtArcLength(curve, cum, target));
  }
  return result;
}

PVector aiPointAtArcLength(ArrayList<PVector> curve, float[] cum, float target) {
  int n = curve.size();
  for (int i = 0; i < n - 1; i++) {
    if (cum[i + 1] >= target) {
      float segLen = cum[i + 1] - cum[i];
      float t = segLen > 0 ? (target - cum[i]) / segLen : 0;
      return PVector.lerp(curve.get(i), curve.get(i + 1), t);
    }
  }
  return curve.get(n - 1);
}

// Arc length of each segment, measured directly off the assembled curve
// (the concatenation of every segment's own generated points) rather than
// from a per-family length formula - works identically for line/arc/loop/
// spiral since it just walks the actual points. `corners` are the
// segment-join points in the same order aiAssembleSegments() produced them
// (one per segment after the first).
ArrayList<Float> aiSegmentLengths(ArrayList<PVector> curve, ArrayList<PVector> corners) {
  int n = curve.size();
  float[] cum = new float[n];
  for (int i = 1; i < n; i++) cum[i] = cum[i - 1] + PVector.dist(curve.get(i - 1), curve.get(i));

  int numSegs = corners.size() + 1;
  int[] boundaryIdx = new int[numSegs + 1];
  boundaryIdx[0] = 0;
  for (int k = 0; k < corners.size(); k++) boundaryIdx[k + 1] = aiNearestCurveIndex(curve, corners.get(k));
  boundaryIdx[numSegs] = n - 1;

  ArrayList<Float> lengths = new ArrayList<Float>();
  for (int k = 0; k < numSegs; k++) lengths.add(cum[boundaryIdx[k + 1]] - cum[boundaryIdx[k]]);
  return lengths;
}

// Splits MOTOR_COUNT joints across segments proportional to each one's own
// arc length, guaranteeing every segment gets at least 1 joint and the
// total is always exactly totalJoints - replaces resampling the WHOLE
// curve by arc-length fraction and hoping corners land on sensible motors
// afterward. Largest-remainder method: floor each segment's proportional
// share (min 1), then hand out whatever's left over to the segments with
// the biggest fractional remainders. Verified standalone in Python across
// the hammer/racket/staple/square cases, including the degenerate case of
// more segments than joints (shouldn't happen given the prompt's 5-6
// segment cap, but doesn't crash if it does - excess segments get 0).
int[] aiAllocateJointCounts(ArrayList<Float> segLengths, int totalJoints) {
  int n = segLengths.size();
  int[] floors = new int[n];
  if (n >= totalJoints) {
    for (int i = 0; i < n && i < totalJoints; i++) floors[i] = 1;
    return floors;
  }

  float totalLen = 0;
  for (float L : segLengths) totalLen += L;
  if (totalLen < 0.0001) totalLen = 0.0001;

  float[] raw = new float[n];
  for (int i = 0; i < n; i++) raw[i] = totalJoints * segLengths.get(i) / totalLen;
  for (int i = 0; i < n; i++) floors[i] = max(1, int(raw[i]));

  int sum = 0;
  for (int f : floors) sum += f;
  while (sum > totalJoints) {
    int bestIdx = -1, bestVal = -1;
    for (int i = 0; i < n; i++) {
      if (floors[i] > 1 && floors[i] > bestVal) { bestVal = floors[i]; bestIdx = i; }
    }
    if (bestIdx == -1) break;
    floors[bestIdx]--;
    sum--;
  }

  int remainder = totalJoints - sum;
  final float[] fracs = new float[n];
  for (int i = 0; i < n; i++) fracs[i] = raw[i] - int(raw[i]);
  Integer[] order = new Integer[n];
  for (int i = 0; i < n; i++) order[i] = i;
  java.util.Arrays.sort(order, new java.util.Comparator<Integer>() {
    public int compare(Integer a, Integer b) { return Float.compare(fracs[b], fracs[a]); }
  });
  for (int i = 0; i < remainder; i++) floors[order[i % n]]++;

  return floors;
}

// Resamples each segment using ONLY its own allocated joint count
// (jointCounts[k], from aiAllocateJointCounts above) instead of resampling
// the whole curve by global arc-length fraction - a segment's targets are
// evenly spaced within ITS OWN local arc length only, so a corner always
// lands exactly on the boundary between two segments' allocations, by
// construction, with no snapping or guessing involved.
//
// `curve` must be the same-length, same-order, PRE-alignment version of
// `alignedCurve` (both derived from the same aiAssembleSegments() output),
// so arc-length fractions computed on one apply identically to the other -
// same trick aiResampleByArcLength used.
ArrayList<PVector> aiResampleByJointCounts(
  ArrayList<PVector> alignedCurve, ArrayList<PVector> curve,
  ArrayList<PVector> corners, int[] jointCounts
) {
  int n = curve.size();
  float[] cum = new float[n];
  for (int i = 1; i < n; i++) cum[i] = cum[i - 1] + PVector.dist(curve.get(i - 1), curve.get(i));

  int numSegs = jointCounts.length;
  int[] boundaryIdx = new int[numSegs + 1];
  boundaryIdx[0] = 0;
  for (int k = 0; k < corners.size(); k++) boundaryIdx[k + 1] = aiNearestCurveIndex(curve, corners.get(k));
  boundaryIdx[numSegs] = n - 1;

  ArrayList<PVector> result = new ArrayList<PVector>();
  for (int k = 0; k < numSegs; k++) {
    float segStart = cum[boundaryIdx[k]];
    float segEnd = cum[boundaryIdx[k + 1]];
    int count = jointCounts[k];
    for (int m = 1; m <= count; m++) {
      float targetArcLen = segStart + (segEnd - segStart) * (m / float(count));
      result.add(aiPointAtArcLength(alignedCurve, cum, targetArcLen));
    }
  }
  return result;
}

int aiNearestCurveIndex(ArrayList<PVector> curve, PVector target) {
  int best = 0;
  float bestD = PVector.dist(curve.get(0), target);
  for (int i = 1; i < curve.size(); i++) {
    float d = PVector.dist(curve.get(i), target);
    if (d < bestD) { bestD = d; best = i; }
  }
  return best;
}

float aiAngleDiff(float a, float b) {
  float d = a - b;
  while (d > PI) d -= TWO_PI;
  while (d < -PI) d += TWO_PI;
  return d;
}

// Predicts the exact joint-box positions drawlineform3d() will render for a
// given set of slider angles - same forward-kinematics chain (rotateZ for
// odd motors, rotateY for even motors, the 3/6/7/8/10 mirror flip, the same
// box-center offset), just as plain vector math instead of Processing's
// transform stack, so it can be called any time (not only mid-draw()) to
// know where the arm will actually sit before it's rendered.
ArrayList<PVector> aiComputeAchievableCurve(float[] angles) {
  ArrayList<PVector> pts = new ArrayList<PVector>();

  PVector pos = CHAIN_ORIGIN.copy();
  PVector right   = new PVector(1, 0, 0); // world direction of local +X
  PVector up      = new PVector(0, 1, 0); // world direction of local +Y
  PVector forward = new PVector(0, 0, 1); // world direction of local +Z

  pts.add(pos.copy());

  float boxOffsetX = LINK_GAP + LINK_WIDTH / 2.0 + (LINK_WIDTH - LINK_HINGE);

  for (int i = 0; i < angles.length; i++) {
    int motorNumber = i + 1;
    int dxlId = motorNumber + 1;
    float safeAngle = constrain(angles[i], 60, 300);
    float angle = safeAngle - 180;

    boolean mirrored = aiIsMirroredMotor(dxlId);
    if (mirrored) angle = -angle;

    float rad = radians(angle);

    if (motorNumber % 2 == 1) {
      // rotateZ(-rad): rotates right/up around forward
      float theta = -rad;
      float c = cos(theta), s = sin(theta);
      PVector newRight = PVector.add(PVector.mult(right, c), PVector.mult(up, s));
      PVector newUp    = PVector.add(PVector.mult(right, -s), PVector.mult(up, c));
      right = newRight;
      up = newUp;
    } else {
      // rotateY(rad): rotates right/forward around up
      float theta = rad;
      float c = cos(theta), s = sin(theta);
      PVector newRight   = PVector.add(PVector.mult(right, c), PVector.mult(forward, -s));
      PVector newForward = PVector.add(PVector.mult(right, s), PVector.mult(forward, c));
      right = newRight;
      forward = newForward;
    }

    PVector boxCenter = PVector.add(pos, PVector.add(
      PVector.mult(right, boxOffsetX),
      PVector.mult(forward, LINK_HEIGHT / 2.0)
    ));
    pts.add(boxCenter);

    pos = PVector.add(pos, PVector.mult(right, LINK_HINGE + LINK_GAP));
  }

  return pts;
}

PVector aiCentroidOf(ArrayList<PVector> pts) {
  PVector sum = new PVector(0, 0, 0);
  for (PVector p : pts) sum.add(p);
  return PVector.div(sum, pts.size());
}

// Total arc length along `pts`, in order - the physically meaningful
// "how much chain does this need" measure, unlike aiBoundsExtent (bounding
// box), which understates it for any shape that folds back on itself.
float aiPathLength(ArrayList<PVector> pts, boolean closed) {
  float total = 0;
  for (int i = 1; i < pts.size(); i++) total += PVector.dist(pts.get(i - 1), pts.get(i));
  if (closed && pts.size() > 1) total += PVector.dist(pts.get(pts.size() - 1), pts.get(0));
  return total;
}

float aiBoundsExtent(ArrayList<PVector> pts) {
  PVector lo = pts.get(0).copy();
  PVector hi = pts.get(0).copy();
  for (PVector p : pts) {
    lo.x = min(lo.x, p.x); lo.y = min(lo.y, p.y); lo.z = min(lo.z, p.z);
    hi.x = max(hi.x, p.x); hi.y = max(hi.y, p.y); hi.z = max(hi.z, p.z);
  }
  return max(hi.x - lo.x, max(hi.y - lo.y, hi.z - lo.z));
}

// Rescales and repositions the AI's raw abstract-unit curve into the
// chain's world-space neighborhood - both so the cyan preview sits on top
// of where the arm actually is, and so aiSolveAnglesForCurve() has
// real-world-space target points to solve against instead of the AI's raw
// ~-10..10 unit coordinates.
//
// The anchor point differs by shape type: a closed loop has no natural
// "start," so centering the whole shape's centroid over the achievable
// curve's centroid is the right call. An OPEN curve does have a start that
// matters - and motor 1 is physically bolted down at CHAIN_ORIGIN and can't
// move its own base, so anchoring by centroid can leave the curve's actual
// start point tens of units away from where motor 1 has to begin, throwing
// off every joint's target from the very first one. Verified standalone on
// a hook shape: centroid anchoring left the start ~77 units off; anchoring
// by start point instead lands it exactly on CHAIN_ORIGIN.
//
// Scale is based on total PATH LENGTH, not bounding-box size. Bounding box
// badly understates how much physical chain a folded shape needs: two
// 7-unit arms 1 unit apart span a bounding box only ~7 units wide, but the
// chain has to travel ~15 units of actual path to trace both - scaling by
// the box let the target path end up far longer than the arm can reach,
// so joints ran out of range and the shape folded/clumped instead of
// spreading out. Path length is the physically meaningful reference for
// any shape that curls back on itself (a loop, a spiral, this staple).
ArrayList<PVector> aiAlignCurveToWorld(ArrayList<PVector> curve, ArrayList<PVector> referenceAchievable, boolean closed) {
  float aiExtent = aiPathLength(curve, closed);
  // v7: the arm itself is ALWAYS an open chain (fixed base, free tip), so its
  // reach is measured as an open path even when the target shape is closed.
  float achExtent = aiPathLength(referenceAchievable, false);
  float scale = achExtent / max(aiExtent, 0.001);

  // v7: every shape - open OR closed - is anchored at its own start point on
  // the fixed base. Closed shapes used to be anchored by centroid, which
  // asked joint 1 to start somewhere the bolted-down base can't be, and that
  // first-joint error compounded down the whole chain (bad triangles etc.).
  // v9: phase 2 of triggerAIShape() may switch to center anchoring
  // (centroid onto centroid) as a fallback when base anchoring fits badly.
  PVector aiAnchor = aiAnchorAtCenter ? aiCentroidOf(curve) : curve.get(0);
  PVector achAnchor = aiAnchorAtCenter ? aiCentroidOf(referenceAchievable) : referenceAchievable.get(0); // start == CHAIN_ORIGIN

  // Remembered so fit results can be mapped back into the AI's own units
  // for refinement feedback (see aiToAiUnits / aiLastJointReport).
  aiAlignScale = scale;
  aiAlignAiAnchor = aiAnchor.copy();
  aiAlignAchAnchor = achAnchor.copy();

  ArrayList<PVector> aligned = new ArrayList<PVector>();
  for (PVector p : curve) {
    aligned.add(new PVector(
      achAnchor.x + (p.x - aiAnchor.x) * scale,
      achAnchor.y + (p.y - aiAnchor.y) * scale,
      achAnchor.z + (p.z - aiAnchor.z) * scale
    ));
  }
  return aligned;
}

// Analytically solves for each joint's angle, one at a time from motor 1 to
// 11, walking the same forward-kinematics chain as aiComputeAchievableCurve.
// Each joint's box position is a single rotation around a fixed axis - i.e.
// it traces a circle as that one angle varies - so there's a closed-form
// best angle for reaching that joint's target point, computed from the
// chain's REAL accumulated position/orientation after the previous joints'
// actual (possibly clamped) angles. This is what keeps errors from quietly
// compounding joint to joint the way the local-tangent heuristic
// (aiAnglesFromCurve) can.
float[] aiSolveAnglesForCurve(ArrayList<PVector> targetPoints, int numMotors) {
  float[] outAngles = new float[numMotors];

  PVector pos = CHAIN_ORIGIN.copy();
  PVector right   = new PVector(1, 0, 0);
  PVector up      = new PVector(0, 1, 0);
  PVector forward = new PVector(0, 0, 1);

  float boxOffsetX = LINK_GAP + LINK_WIDTH / 2.0 + (LINK_WIDTH - LINK_HINGE);
  float halfHeight = LINK_HEIGHT / 2.0;

  for (int i = 0; i < numMotors; i++) {
    int motorNumber = i + 1;
    int dxlId = motorNumber + 1;
    boolean bendsInXY = (motorNumber % 2 == 1);
    boolean mirrored = aiIsMirroredMotor(dxlId);

    PVector targetVec = PVector.sub(targetPoints.get(i), pos);
    float theta;

    if (bendsInXY) {
      // world_offset(theta) = halfHeight*forward (fixed, forward doesn't
      // change under a Z-axis rotation) + boxOffsetX*(cos(theta)*right +
      // sin(theta)*up). Solve theta so that term points at the target.
      PVector remainder = PVector.sub(targetVec, PVector.mult(forward, halfHeight));
      float a = remainder.dot(right);
      float b = remainder.dot(up);
      theta = atan2(b, a);
    } else {
      // Both right and forward rotate together around 'up' for a Y-axis
      // rotation, so the box center traces a circle of radius
      // sqrt(boxOffsetX^2 + halfHeight^2) at a fixed phase offset from
      // the un-rotated (boxOffsetX, halfHeight) point. Solve for the
      // rotation that lands that circle point on the target.
      float a = targetVec.dot(right);
      float b = targetVec.dot(forward);
      float phase0 = atan2(halfHeight, boxOffsetX);
      float desiredPhase = atan2(b, a);
      theta = phase0 - desiredPhase;
    }

    // Undo the FK's own sign conventions (rotateZ(-rad) for odd motors,
    // the 3/6/7/8/10 mirror flip) to recover the actual slider angle.
    float rad = bendsInXY ? -theta : theta;
    float angleAfterMirror = degrees(rad);
    float angleBeforeMirror = mirrored ? -angleAfterMirror : angleAfterMirror;
    angleBeforeMirror = constrain(angleBeforeMirror, -AI_MAX_TURN_DEG, AI_MAX_TURN_DEG);
    float motorAngle = constrain(180 + angleBeforeMirror, 60, 310);
    outAngles[i] = motorAngle;

    // Advance the chain using the FINAL (possibly clamped) angle, so the
    // next joint solves against where the arm will really be - not the
    // unclamped ideal.
    float safeAngle = constrain(motorAngle, 60, 300);
    float angle = safeAngle - 180;
    if (mirrored) angle = -angle;
    float finalRad = radians(angle);

    if (bendsInXY) {
      float th = -finalRad;
      float c = cos(th), s = sin(th);
      PVector newRight = PVector.add(PVector.mult(right, c), PVector.mult(up, s));
      PVector newUp    = PVector.add(PVector.mult(right, -s), PVector.mult(up, c));
      right = newRight;
      up = newUp;
    } else {
      float th = finalRad;
      float c = cos(th), s = sin(th);
      PVector newRight   = PVector.add(PVector.mult(right, c), PVector.mult(forward, -s));
      PVector newForward = PVector.add(PVector.mult(right, s), PVector.mult(forward, c));
      right = newRight;
      forward = newForward;
    }

    pos = PVector.add(pos, PVector.mult(right, LINK_HINGE + LINK_GAP));
  }

  return outAngles;
}

// FABRIK (Forward And Backward Reaching Inverse Kinematics, Aristidou &
// Lasenby) adapted for MULTIPLE per-joint targets rather than the usual
// single end-effector target - we need the whole curve, not just where the
// tip ends up - with a permanently fixed base (motor 1 is physically
// bolted down, so classic FABRIK's backward pass, which exists to
// reconcile a movable base with a single end target, isn't needed; one
// forward sweep from the fixed base fully restores every link length).
// Each iteration: (1) pull every joint partway toward its OWN target, (2)
// re-project outward from the fixed base so every link is EXACTLY
// `linkLength` again. Verified standalone in Python: given a mismatched
// target (the exact staple-bridge bug - a gap scaled to less than one
// physical link), the correction gets absorbed mostly by the joint
// nearest the mismatch, while the rest of the chain stays within ~0.5
// units of its own target - a graceful degrade instead of the error
// cascading and amplifying through every joint after it.
//
// `warmStart`, if given, seeds the initial positions instead of starting
// fresh from `targets` each call - lets aiFabrikRefineAnglesForCurve carry
// over where the REAL (clamped, axis-constrained) chain actually ended up
// from one outer iteration to the next.
//
// `jointWeights`, if given, scales the per-joint pull strength (alpha) each
// relaxation step - a joint with weight > 1 gets pulled harder toward its
// own target than the baseline `alpha`, a joint with weight < 1 gets pulled
// more gently. Since every step also re-projects outward from the fixed
// base to restore exact link length, pulling the base joints harder makes
// them converge on their targets fastest and forces whatever slack is left
// over onto the joints after them - i.e. this is what actually makes
// AI_BASE_JOINT_WEIGHT bias the resulting SHAPE, not just which outer round
// gets picked as "best."
ArrayList<PVector> aiFabrikSolvePositions(
  PVector base, ArrayList<PVector> targets, float linkLength,
  int iterations, float alpha, ArrayList<PVector> warmStart, float[] jointWeights
) {
  int n = targets.size();
  ArrayList<PVector> p = new ArrayList<PVector>();
  for (int i = 0; i < n; i++) {
    p.add((warmStart != null) ? warmStart.get(i).copy() : targets.get(i).copy());
  }

  for (int iter = 0; iter < iterations; iter++) {
    for (int i = 0; i < n; i++) {
      PVector diff = PVector.sub(targets.get(i), p.get(i));
      float jointAlpha = (jointWeights != null && i < jointWeights.length)
        ? constrain(alpha * jointWeights[i], 0.05, 0.95) : alpha;
      p.set(i, PVector.add(p.get(i), PVector.mult(diff, jointAlpha)));
    }

    PVector prev = base;
    for (int i = 0; i < n; i++) {
      PVector direction = PVector.sub(p.get(i), prev);
      float dist = direction.mag();
      if (dist < 0.000001) {
        direction = new PVector(1, 0, 0);
        dist = 1;
      }
      direction.normalize();
      PVector fixedPt = PVector.add(prev, PVector.mult(direction, linkLength));
      p.set(i, fixedPt);
      prev = fixedPt;
    }
  }

  return p;
}

// Outer loop around the FABRIK position solver: FABRIK's idealized model
// assumes every link is exactly `linkLength` and free to point anywhere,
// but the real arm ALSO has to respect fixed rotation-plane constraints
// (odd joints bend local X-Y, even bend local X-Z) and angle clamps -
// aiSolveAnglesForCurve (unchanged, already verified) handles that part.
// Each outer round: (1) FABRIK-relax positions toward the fixed original
// targets, warm-started from wherever the chain really ended up last
// round, (2) solve axis-respecting angles from those positions, (3)
// compute where the REAL constrained chain actually lands, which may
// differ from FABRIK's idealized assumption once a clamp binds, and (4)
// feed those real positions back into FABRIK as the next round's warm
// start - so it learns where the real chain's constraints actually bind
// and adjusts the rest of the targets accordingly, instead of blindly
// re-relaxing the same idealized (and possibly unreachable) positions
// every time. Tracks and returns whichever round had the lowest total
// error against the ORIGINAL targets, since rounds aren't strictly
// monotonic.
float[] aiFabrikRefineAnglesForCurve(
  PVector base, ArrayList<PVector> targets, int numMotors, int outerIterations
) {
  float linkStep = LINK_HINGE + LINK_GAP;

  ArrayList<PVector> currentPositions = new ArrayList<PVector>();
  for (PVector t : targets) currentPositions.add(t.copy());

  // Same weights drive both how hard FABRIK pulls each joint toward its
  // target (below) and how the outer rounds get compared - a single source
  // of truth for "the base matters more," rather than two places that could
  // silently drift out of sync.
  float[] jointWeights = aiBuildJointWeights(numMotors);

  float[] bestAngles = null;
  float bestErr = Float.MAX_VALUE;

  for (int outer = 0; outer < outerIterations; outer++) {
    ArrayList<PVector> fabrikPositions =
      aiFabrikSolvePositions(base, targets, linkStep, 15, 0.5, currentPositions, jointWeights);

    float[] angles = aiSolveAnglesForCurve(fabrikPositions, numMotors);
    ArrayList<PVector> achievable = aiComputeAchievableCurve(angles);

    float err = 0;
    for (int j = 0; j < numMotors; j++) err += jointWeights[j] * PVector.dist(achievable.get(j + 1), targets.get(j));
    if (err < bestErr) {
      bestErr = err;
      bestAngles = angles;
    }

    currentPositions.clear();
    for (int j = 0; j < numMotors; j++) currentPositions.add(achievable.get(j + 1).copy());
  }

  aiLastRefineError = bestErr;
  return bestAngles;
}

void applyAIAnglesToSliders() {
  updatingShapeControls = true;
  for (int position = 1; position <= MOTOR_COUNT; position++) {
    int dxlId = position + 1;
    Controller slider = cp5.getController("Motor " + dxlId);
    slider.setBroadcast(false);
    slider.setValue(generatedAngles[position - 1]);
    slider.setBroadcast(true);
  }
  updatingShapeControls = false;
}
