// =====================================================================
//  ADJUST THE CURRENT ANIMATION (2026-11-25) - Motion agent tool
//  adjust_current_animation: speed, size of the swing, torque, stiffness of
//  the animation that is PLAYING (applied instantly, no new preview, even if
//  a new one is waiting); only when nothing plays, the WAITING one. Works for
//  every kind: the agent's poses / traveling waves and loaded recordings.
//  Only a change to the MOTION itself needs a new animation (and preview).
// =====================================================================

boolean animAdjustAvailable() { return aiAnimating || stagedAnim != null; }

JSONObject animAdjustToolDefinition() {
  JSONObject props = new JSONObject();
  props.setJSONObject("speed_factor", loadedRangeNumber("OPTIONAL. Playback speed: 1.5 = 50% faster, 0.5 = half speed. Leave out to keep the speed.", 0.25, 4.0));
  props.setJSONObject("amplitude_factor", loadedRangeNumber("OPTIONAL. Size of the swing of every joint around its average angle: >1 = bigger motion, <1 = smaller. Leave out to keep it.", 0.3, 2.0));
  props.setJSONObject("message_to_user", chatSchemaProp("string", "1-2 short sentences for the user about what you changed."));
  JSONObject schema = new JSONObject();
  schema.setString("type", "object");
  schema.setJSONObject("properties", props);
  JSONArray req = new JSONArray();
  req.append("message_to_user");
  schema.setJSONArray("required", req);
  JSONObject tool = new JSONObject();
  tool.setString("name", "adjust_current_animation");
  tool.setString("description", "Change the CURRENT animation in place - the one PLAYING, applied immediately with no new "
    + "preview (even if a new one is waiting); only if nothing is playing, the one waiting for Create Shape: speed, size of the swing, torque, stiffness. The motion itself "
    + "stays exactly the same. Use this for 'faster', 'slower', 'bigger', 'smaller', 'softer', 'firmer', 'limp tail', "
    + "'stronger' and the like.");
  tool.setJSONObject("input_schema", schema);
  chatAddTorqueField(tool, "OPTIONAL. New torque for each joint for the animation, 11 integers 0-100, base to tip (0 = motor off, free-swinging). Leave out to keep it.");
  chatAddStiffnessField(tool, "OPTIONAL. New stiffness level for each joint for the animation, 11 integers 0-3, base to tip. Leave out to keep it.");
  return tool;
}

// Scales every joint's swing around its average over the given frames, in place.
void animScaleAround(ArrayList<float[]> frames, float amp) {
  if (frames == null || frames.isEmpty() || amp == 1) return;
  float[] mean = new float[MOTOR_COUNT];
  for (float[] f : frames) for (int j = 0; j < MOTOR_COUNT; j++) mean[j] += f[j] / frames.size();
  for (float[] f : frames) for (int j = 0; j < MOTOR_COUNT; j++) f[j] = clampJoint(mean[j] + (f[j] - mean[j]) * amp);
}

// Recording: time-scale in place (speed) and set its per-frame torque / stiffness.
void animRecordingAdjust(AnimRecording r, float speed, float amp, int[] tq, int[] st) {
  if (speed != 1) {
    for (int k = 0; k < r.t.size(); k++) r.t.set(k, round(r.t.get(k) / speed));
    r.durationMs = round(r.durationMs / speed);
    r.resetMs = max(150, round(r.resetMs / speed));
    r.intervalMs = max(1, round(r.intervalMs / speed));
  }
  animScaleAround(r.frames, amp);
  for (int k = 0; k < r.frames.size(); k++) {
    if (tq != null) r.torque.set(k, tq.clone());
    if (st != null) r.stiffness.set(k, st.clone());
  }
}

// Applies adjust_current_animation (drawing thread). Returns the result text for the agent.
String animApplyAdjust(JSONObject in) {
  if (!animAdjustAvailable()) return "ERROR: no animation is playing or waiting - nothing changed. Make a new animation instead.";
  float speed = in.hasKey("speed_factor") ? constrain(in.getFloat("speed_factor"), 0.25, 4.0) : 1;
  float amp = in.hasKey("amplitude_factor") ? constrain(in.getFloat("amplitude_factor"), 0.3, 2.0) : 1;
  int[] tq = agentsParseTorque(in), st = agentsParseStiffness(in);
  String name;
  // The CURRENT (playing) animation, changed instantly - even if a new one is
  // waiting as the preview. Only when nothing plays is the waiting one changed.
  boolean playing = aiAnimating;

  if (playing) {
    name = aiAnimName;
    int now = millis();
    if (aiAnimRecording != null) {
      int elapsed = now - aiAnimLoopStartMillis;
      animRecordingAdjust(aiAnimRecording, speed, amp, tq, st);
      aiAnimLoopStartMillis = now - round(elapsed / speed); // same point in the loop, new speed
    } else if (aiAnimIsWave) {
      if (speed != 1) {
        float phase = (now - aiAnimLoopStartMillis) / aiAnimWavePeriodMs;
        aiAnimWavePeriodMs = max(150, aiAnimWavePeriodMs / speed);
        aiAnimLoopStartMillis = now - round(phase * aiAnimWavePeriodMs);
      }
      aiAnimWaveAmplitudeDeg = min(AI_MAX_TURN_DEG, aiAnimWaveAmplitudeDeg * amp);
    } else {
      if (speed != 1 && aiAnimPoses.size() > 0) {
        float loop = aiAnimFrameDurationMs * aiAnimPoses.size();
        float u = ((now - aiAnimLoopStartMillis) % loop) / loop;
        aiAnimFrameDurationMs = max(150, round(aiAnimFrameDurationMs / speed));
        aiAnimLoopStartMillis = now - round(u * aiAnimFrameDurationMs * aiAnimPoses.size());
      }
      animScaleAround(aiAnimPoses, amp);
    }
    if (st != null) aiApplyStiffnessLevels(st); // instant
    if (tq != null) motorsSetTorqueAll(tq);    // instant (Stop still restores the torque from before the animation)
  } else {
    AIAnimationResult a = stagedAnim;
    name = a.animationName;
    if (a.recording != null) {
      animRecordingAdjust(a.recording, speed, amp, tq, st);
    } else if (a.isTravelingWave) {
      a.wavePeriodMs = max(150, a.wavePeriodMs / speed);
      a.waveAmplitudeDeg = min(AI_MAX_TURN_DEG, a.waveAmplitudeDeg * amp);
    } else {
      a.frameDurationMs = max(150, a.frameDurationMs / speed);
      animScaleAround(a.solvedPoses, amp);
    }
    if (st != null) { a.jointStiffness = st.clone(); a.stiffnessGiven = true; }
    if (tq != null) stagedAnimTorque = tq.clone();
  }
  if (loaded != null && loaded.kind.equals("animation") && loaded.name.equals(name)) loaded.edits++;

  return "Adjusted the animation '" + name + "' " + (playing ? "while it plays (applied immediately)" : "that is WAITING for Create Shape")
    + (speed != 1 ? ", speed x" + nf(speed, 1, 2) : "") + (amp != 1 ? ", swing x" + nf(amp, 1, 2) : "")
    + (tq != null ? ", torque " + motorsTorqueList(tq) : "") + (st != null ? ", stiffness " + motorsTorqueList(st) : "")
    + ". The motion itself is unchanged.";
}
