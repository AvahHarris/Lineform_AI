// =====================================================================
//  ANIMATION RECORDING + REPLAY (2026-11-21)
//
//  * Save Animation (button, while an animation plays) RECORDS the next
//    ANIM_RECORD_SECONDS of it: every ANIM_RECORD_INTERVAL_MS, all 11
//    joint angles + torque + stiffness. Then a "Save animation as..."
//    dialog writes it to its own .json file (Files.pde).
//  * Load Animation (Files.pde) replays a file: the recording, then
//    ANIM_REPLAY_RESET_MS of smooth return to its first frame, then again -
//    until Stop Animation. No AI call. Both agents are told what was loaded.
//
//  Animation file:
//    { "type": "recording", "name": "snake slither",
//      "interval_ms": 40, "duration_ms": 3000, "reset_ms": 500,
//      "frames": [ {"t": 0, "angles": [11 x deg], "torque": [11 x 0-100],
//                   "stiffness": [11 x 0-3]}, ... ],
//      "source": { the Motion agent's original tool input, if known } }
//  Angles are internal degrees (180 = straight), base to tip; "t" = ms
//  from the start of the recording.
// =====================================================================

float ANIM_RECORD_SECONDS = 3.0;   // length of the recorded window
int ANIM_RECORD_INTERVAL_MS = 40;  // one frame every ... ms (= the animation send rate)
int ANIM_REPLAY_RESET_MS = 500;    // return to the first frame between loops

AIAnimationResult aiLastAnimation = null; // set by aiStartAnimation()
AnimRecording aiAnimRecording = null;     // the recording being replayed (null = none)

class AnimRecording {
  ArrayList<Integer> t = new ArrayList<Integer>();
  ArrayList<float[]> frames = new ArrayList<float[]>();
  ArrayList<int[]> torque = new ArrayList<int[]>();
  ArrayList<int[]> stiffness = new ArrayList<int[]>();
  int durationMs = 3000, resetMs = 500, intervalMs = 40;

  // Short description for the agents: per-joint range and the settings.
  String summaryForAgents() {
    StringBuilder lo = new StringBuilder(), hi = new StringBuilder();
    for (int j = 0; j < MOTOR_COUNT; j++) {
      float mn = 1e9, mx = -1e9;
      for (float[] f : frames) { mn = min(mn, f[j]); mx = max(mx, f[j]); }
      lo.append(j == 0 ? "" : ", ").append(round(mn - 180));
      hi.append(j == 0 ? "" : ", ").append(round(mx - 180));
    }
    return "each joint's bend off straight swings between [" + lo + "] and [" + hi + "] degrees (base to tip); "
      + "torque " + settingText(torque) + "; stiffness " + settingText(stiffness);
  }

  String settingText(ArrayList<int[]> list) {
    if (list.isEmpty()) return "unknown";
    boolean constant = true;
    for (int[] v : list) if (!java.util.Arrays.equals(v, list.get(0))) constant = false;
    return motorsTorqueList(list.get(0)) + (constant ? "" : " at the start (it changes during the recording)");
  }
}

// ---- recording (Save Animation) ----
AnimRecording animRec = null;      // being recorded right now
int animRecStartMs = 0, animRecLastMs = -1000000;
String animRecName = "";

void animLibrarySaveButton() {
  if (animRec != null) {
    noticeAdd("status", "Already recording \"" + animRecName + "\" - one moment.");
    return;
  }
  if (!aiAnimating) {
    noticeAdd("status", "Play the animation first, then press Save Animation - it records the next "
      + nf(ANIM_RECORD_SECONDS, 1, 1) + " s.");
    return;
  }
  String typed = (chatInput != null) ? chatInput.getText() : "";
  String name = (typed != null && typed.trim().length() > 0) ? typed : aiAnimName;
  name = name == null ? "" : name.trim().toLowerCase();
  if (name.length() == 0) name = "animation " + year() + nf(month(), 2) + nf(day(), 2) + "-" + nf(hour(), 2) + nf(minute(), 2);
  if (chatInput != null && typed != null && typed.trim().length() > 0) chatInput.clear();
  animRecName = name;
  animRec = new AnimRecording();
  animRec.durationMs = round(ANIM_RECORD_SECONDS * 1000);
  animRec.resetMs = ANIM_REPLAY_RESET_MS;
  animRec.intervalMs = ANIM_RECORD_INTERVAL_MS;
  animRecStartMs = millis();
  animRecLastMs = -1000000;
  noticeAdd("status", "Recording \"" + name + "\" for " + nf(ANIM_RECORD_SECONDS, 1, 1) + " s...");
}

// Called by aiUpdateAnimation() every frame with the angles just commanded.
void animRecordTick(float[] angles, int now) {
  if (animRec == null) return;
  int t = now - animRecStartMs;
  if (t > animRec.durationMs) { animRecordFinish(); return; }
  if (now - animRecLastMs < animRec.intervalMs) return;
  animRecLastMs = now;
  int[] tq = new int[MOTOR_COUNT];
  for (int i = 0; i < MOTOR_COUNT; i++) tq[i] = motorPowerOn(i) ? motorTorquePct[i] : 0;
  animRec.t.add(t);
  animRec.frames.add(angles.clone());
  animRec.torque.add(tq);
  animRec.stiffness.add(hingeStiffnessLevel.clone());
}

void animRecordCancel(String why) {
  if (animRec == null) return;
  noticeAdd("status", "Recording of \"" + animRecName + "\" cancelled (" + why + ") - nothing saved.");
  animRec = null;
}

void animRecordFinish() {
  AnimRecording r = animRec;
  animRec = null;
  if (r == null || r.frames.size() < 2) {
    noticeAdd("error", "Recording too short - nothing saved.");
    return;
  }
  JSONObject e = new JSONObject();
  e.setString("type", "recording");
  e.setString("name", animRecName);
  e.setInt("interval_ms", r.intervalMs);
  e.setInt("duration_ms", r.durationMs);
  e.setInt("reset_ms", r.resetMs);
  JSONArray frames = new JSONArray();
  for (int k = 0; k < r.frames.size(); k++) {
    JSONObject f = new JSONObject();
    f.setInt("t", r.t.get(k));
    JSONArray ang = new JSONArray();
    for (float v : r.frames.get(k)) ang.append(round(v * 10) / 10.0);
    f.setJSONArray("angles", ang);
    f.setJSONArray("torque", libraryJsonInts(r.torque.get(k)));
    f.setJSONArray("stiffness", libraryJsonInts(r.stiffness.get(k)));
    frames.append(f);
  }
  e.setJSONArray("frames", frames);
  AIAnimationResult src = aiLastAnimation;
  if (src != null && src.source != null) e.setJSONObject("source", src.source);

  noticeAdd("status", "Recorded \"" + animRecName + "\" (" + r.frames.size() + " frames, "
    + nf(r.durationMs / 1000.0, 1, 1) + " s) - choose where to save it.");
  filesAskSaveAnimation(e); // "Save animation as..." dialog (Files.pde)
}

// ---- replay ----
// Angles at time `elapsedMs` since the replay started: the recording, then
// a smooth return to the first frame, then again. Applies the frame's
// torque / stiffness whenever they change.
int animRecAppliedFrame = -1;

float[] animRecordingSample(AnimRecording r, int elapsedMs) {
  int n = r.frames.size();
  int lastT = r.t.get(n - 1);
  int cycle = lastT + max(1, r.resetMs);
  float e = elapsedMs % cycle;
  float[] out = new float[MOTOR_COUNT];
  int settingsFrame;
  if (e <= lastT) {
    int k = 0;
    while (k < n - 2 && r.t.get(k + 1) < e) k++;
    float t0 = r.t.get(k), t1 = r.t.get(k + 1);
    float u = t1 > t0 ? constrain((e - t0) / (t1 - t0), 0, 1) : 0;
    for (int j = 0; j < MOTOR_COUNT; j++) out[j] = lerp(r.frames.get(k)[j], r.frames.get(k + 1)[j], u);
    settingsFrame = u < 0.5 ? k : k + 1;
  } else {
    float u = (e - lastT) / (float) max(1, r.resetMs);
    u = u * u * (3 - 2 * u); // smoothstep: eases out of the last frame and into the first
    for (int j = 0; j < MOTOR_COUNT; j++) out[j] = lerp(r.frames.get(n - 1)[j], r.frames.get(0)[j], u);
    settingsFrame = n - 1;
  }
  if (settingsFrame != animRecAppliedFrame) {
    int[] tq = r.torque.get(settingsFrame), st = r.stiffness.get(settingsFrame);
    if (animRecAppliedFrame < 0 || !java.util.Arrays.equals(tq, r.torque.get(animRecAppliedFrame))) motorsSetTorqueAll(tq);
    aiApplyStiffnessLevels(st); // only sends joints whose level changes
    animRecAppliedFrame = settingsFrame;
  }
  for (int j = 0; j < MOTOR_COUNT; j++) out[j] = clampJoint(out[j]);
  return out;
}

// Angles of a recording at `elapsedMs` (looping, with the reset) - no
// torque / stiffness side effects. Used by the 3D preview (Staging.pde).
float[] animRecordingAnglesAt(AnimRecording r, int elapsedMs) {
  int n = r.frames.size();
  int lastT = r.t.get(n - 1);
  float e = elapsedMs % (lastT + max(1, r.resetMs));
  float[] out = new float[MOTOR_COUNT];
  if (e <= lastT) {
    int k = 0;
    while (k < n - 2 && r.t.get(k + 1) < e) k++;
    float t0 = r.t.get(k), t1 = r.t.get(k + 1);
    float u = t1 > t0 ? constrain((e - t0) / (t1 - t0), 0, 1) : 0;
    for (int j = 0; j < MOTOR_COUNT; j++) out[j] = lerp(r.frames.get(k)[j], r.frames.get(k + 1)[j], u);
  } else {
    float u = (e - lastT) / (float) max(1, r.resetMs);
    u = u * u * (3 - 2 * u);
    for (int j = 0; j < MOTOR_COUNT; j++) out[j] = lerp(r.frames.get(n - 1)[j], r.frames.get(0)[j], u);
  }
  for (int j = 0; j < MOTOR_COUNT; j++) out[j] = clampJoint(out[j]);
  return out;
}

// Torque of a recording at `elapsedMs` (nearest frame; the reset uses the last frame).
int[] animRecordingTorqueAt(AnimRecording r, int elapsedMs) {
  int n = r.frames.size();
  int lastT = r.t.get(n - 1);
  float e = elapsedMs % (lastT + max(1, r.resetMs));
  if (e > lastT) return r.torque.get(n - 1);
  int k = 0;
  while (k < n - 1 && r.t.get(k + 1) <= e) k++;
  return r.torque.get(k);
}

// Plays a loaded animation file's contents (call on the drawing thread).
void animLibraryPlay(JSONObject e, String fileName) {
  String name = e.getString("name");
  AnimRecording rec = new AnimRecording();
  JSONArray frames = e.hasKey("frames") ? e.getJSONArray("frames") : null;
  if (frames == null || frames.size() < 2) {
    noticeAdd("error", "Animation \"" + name + "\" has no recording (needs at least 2 frames) - can't play it.");
    return;
  }
  for (int k = 0; k < frames.size(); k++) {
    JSONObject f = frames.getJSONObject(k);
    float[] ang = f.getJSONArray("angles").getFloatArray();
    int[] tq = libraryIntArray(f, "torque", 0, 100);
    int[] st = libraryIntArray(f, "stiffness", 0, NUM_STIFFNESS_LEVELS - 1);
    if (ang.length != MOTOR_COUNT) {
      noticeAdd("error", "Saved animation \"" + name + "\": frame " + (k + 1) + " doesn't have " + MOTOR_COUNT + " angles.");
      return;
    }
    rec.t.add(f.hasKey("t") ? f.getInt("t") : k * ANIM_RECORD_INTERVAL_MS);
    rec.frames.add(ang);
    rec.torque.add(tq != null ? tq : motorTorquePct.clone());
    rec.stiffness.add(st != null ? st : hingeStiffnessLevel.clone());
  }
  rec.intervalMs = e.hasKey("interval_ms") ? e.getInt("interval_ms") : ANIM_RECORD_INTERVAL_MS;
  rec.durationMs = e.hasKey("duration_ms") ? e.getInt("duration_ms") : rec.t.get(rec.t.size() - 1);
  rec.resetMs = e.hasKey("reset_ms") ? e.getInt("reset_ms") : ANIM_REPLAY_RESET_MS;

  JSONObject source = e.hasKey("source") ? e.getJSONObject("source") : null;
  if (!animStageRecording(name, rec, source)) return;
  loadedSetAnimation(name, fileName, rec, source); // the current item for both agents (Loaded.pde)
  chatAdd("motion", "Loaded the animation \"" + name + "\" (" + nf(rec.durationMs / 1000.0, 1, 1)
    + " s recording, loops). Press Create Shape to start it."
    + " Ask me to change it - e.g. \"faster\".");
}

// ---- JSON helpers (shared with Files.pde) ----
// An int array of exactly MOTOR_COUNT values under `key`, clamped, or null.
int[] libraryIntArray(JSONObject e, String key, int lo, int hi) {
  if (e == null || !e.hasKey(key) || e.isNull(key)) return null;
  try {
    JSONArray a = e.getJSONArray(key);
    if (a == null || a.size() != MOTOR_COUNT) return null;
    int[] out = new int[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) out[i] = constrain(round(a.getFloat(i)), lo, hi);
    return out;
  }
  catch (Exception ex) {
    return null;
  }
}

JSONArray libraryJsonInts(int[] v) {
  JSONArray a = new JSONArray();
  for (int x : v) a.append(x);
  return a;
}

// A NEW recording (Load Animation): READY until Create Shape (Staging.pde).
boolean animStageRecording(String name, AnimRecording rec, JSONObject source) {
  if (manipulationMode) { noticeAdd("status", "Switch to Automation mode to load an animation."); return false; }
  AIAnimationResult r = new AIAnimationResult();
  r.ok = true;
  r.animationName = name;
  r.recording = rec;
  r.source = source;
  r.jointStiffness = rec.stiffness.get(0);
  r.stiffnessGiven = true;
  stageAnimation(r, rec.torque.get(0));
  return true;
}
