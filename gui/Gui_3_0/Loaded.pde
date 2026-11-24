// =====================================================================
//  LOADED FILE CONTEXT (2026-11-24)
//
//  After Load Shape / Load Animation, the loaded item is the CURRENT thing
//  we are working on, for BOTH agents, until an agent makes a NEW shape or
//  animation (its normal tool), the user loads something else, or Reset.
//
//  * Every message to either agent carries a LOADED note: the first time
//    with the file's full parameters, after that a short reminder.
//  * While a shape is loaded, the Designer also gets adjust_loaded_shape;
//    while an animation is loaded, the Motion agent gets
//    adjust_loaded_animation. They change the loaded item IN PLACE (bends,
//    speed, swing, torque, stiffness) instead of re-designing it, so
//    "faster" / "bigger" / "softer" keep everything else exactly as saved.
// =====================================================================

class LoadedContext {
  String kind;          // "shape" or "animation"
  String name, file;
  float[] angles;       // shape: internal degrees (180 = straight)
  int[] torque, stiffness;
  AnimRecording rec;    // animation
  JSONObject source;    // animation: the Motion agent's original input, if saved
  boolean toldDesigner = false, toldMotion = false;
  int edits = 0;        // adjustments made since loading
}

LoadedContext loaded = null;

void loadedSetShape(String name, String file, float[] angles, int[] torque, int[] stiffness) {
  LoadedContext c = new LoadedContext();
  c.kind = "shape";
  c.name = name;
  c.file = file;
  c.angles = angles.clone();
  c.torque = torque;
  c.stiffness = stiffness;
  loaded = c;
  // The Motion agent's "current shape" = the loaded one (no segment list).
  agentShapeName = name;
  agentShapeSegments = null;
  agentShapeClosed = false;
  agentShapeTorque = torque;
  agentShapeStiffness = stiffness;
}

void loadedSetAnimation(String name, String file, AnimRecording rec, JSONObject source) {
  LoadedContext c = new LoadedContext();
  c.kind = "animation";
  c.name = name;
  c.file = file;
  c.rec = rec;
  c.source = source;
  loaded = c;
}

void loadedClear(String why) {
  if (loaded == null) return;
  aiLog("[loaded] '" + loaded.name + "' is no longer the current item (" + why + ").");
  loaded = null;
}

boolean loadedIs(String kind) { return loaded != null && loaded.kind.equals(kind); }

// ---------------- the note for each agent ----------------
String loadedNoteFor(boolean motion) {
  LoadedContext c = loaded;
  if (c == null) return "";
  boolean first = motion ? !c.toldMotion : !c.toldDesigner;
  if (motion) c.toldMotion = true; else c.toldDesigner = true;
  boolean mine = motion == c.kind.equals("animation"); // the loaded item is this agent's kind
  String what = c.kind.equals("shape") ? "shape" : "animation";
  String adjustTool = c.kind.equals("shape") ? "adjust_loaded_shape" : "adjust_loaded_animation";

  String rules;
  if (mine) {
    rules = "RULES while it is loaded: (1) It is the CURRENT " + what + " we are working on and replaces anything "
      + "you made earlier in this chat - do NOT go back to your earlier " + what + "s and do NOT design a new "
      + what + " of your own. (2) If the user asks for a CHANGE to it ('faster', 'slower', 'bigger', 'smaller', "
      + "'softer', 'stiffer', 'curl the tip more', ...), use the " + adjustTool + " tool, which changes the loaded "
      + what + " in place and keeps everything else exactly as saved. (3) Only if the user clearly asks for a "
      + "DIFFERENT or NEW " + what + ", or a change the adjust tool cannot express, use your normal tool - and then "
      + (c.kind.equals("animation") && c.source != null
         ? "start from the original input below and change only what was asked. "
         : "reproduce the loaded " + what + " from the parameters below as closely as you can and change only what was asked. ")
      + "(4) Questions about it are answered from these parameters.";
  } else {
    rules = "It is what is on the arm now and what the user is working on. It is the other agent's kind of item: do "
      + "not replace it on your own. Only make a " + (motion ? "animation" : "shape") + " if the user asks you for one"
      + (c.kind.equals("shape") && motion ? " - then animate THIS loaded shape (start from its bends below)" : "")
      + (c.kind.equals("animation") && !motion ? " - a still shape then probably means a pose from this animation" : "")
      + ".";
  }

  if (!first) {
    return "\n\n[LOADED " + what.toUpperCase() + " STILL ACTIVE: '" + c.name + "' (from " + c.file + "; full parameters "
      + "were given earlier" + (c.edits > 0 ? ", adjusted " + c.edits + "x since - the latest values: " + loadedShortParams(c) : "")
      + "). " + rules + "]";
  }
  return "\n\n[LOADED " + what.toUpperCase() + ": the user loaded '" + c.name + "' from the file " + c.file
    + (c.kind.equals("shape") ? " - its pose, torque and stiffness go to the arm when the user presses Create Shape. "
                              : " - it is READY and starts when the user presses Create Shape (changes to it apply at once once it plays). ") + rules + "\nPARAMETERS FROM THE FILE:\n" + loadedFullParams(c) + "]";
}

String loadedShortParams(LoadedContext c) {
  if (c.kind.equals("shape")) {
    return "bends " + agentsTurnsList(c.angles) + ", torque " + loadedList(c.torque) + ", stiffness " + loadedList(c.stiffness);
  }
  return "duration " + c.rec.durationMs + " ms, " + c.rec.summaryForAgents();
}

String loadedList(int[] v) { return v == null ? "unchanged (not in the file)" : motorsTorqueList(v); }

String loadedFullParams(LoadedContext c) {
  if (c.kind.equals("shape")) {
    return "- joint bends off straight, degrees, base to tip (joint 1 = motor 2): " + agentsTurnsList(c.angles) + "\n"
      + "- torque %, base to tip: " + loadedList(c.torque) + "\n"
      + "- stiffness level, base to tip: " + loadedList(c.stiffness) + "\n"
      + "- (a shape file has no segment list - only these joint values)";
  }
  AnimRecording r = c.rec;
  StringBuilder sb = new StringBuilder();
  sb.append("- a ").append(nf(r.durationMs / 1000.0, 1, 1)).append(" s recording, replayed in a loop with ")
    .append(nf(r.resetMs / 1000.0, 1, 1)).append(" s of smooth return to its first frame between loops; ")
    .append(r.frames.size()).append(" frames every ~").append(r.intervalMs).append(" ms\n");
  sb.append("- ").append(r.summaryForAgents()).append("\n");
  // Keyframes about every 200 ms (bends off straight, base to tip).
  int step = max(1, round(200.0 / max(1, r.intervalMs)));
  sb.append("- keyframes (time ms: joint bends off straight, degrees, base to tip):\n");
  for (int k = 0; k < r.frames.size(); k += step) sb.append("    ").append(r.t.get(k)).append(": ").append(agentsTurnsList(r.frames.get(k))).append("\n");
  int last = r.frames.size() - 1;
  if (last % step != 0) sb.append("    ").append(r.t.get(last)).append(": ").append(agentsTurnsList(r.frames.get(last))).append("\n");
  sb.append(c.source != null ? "- the Motion agent's ORIGINAL emit_shape_animation input for it: " + c.source.format(-1)
                             : "- no original Motion-agent input is saved with it (only the recording above)");
  return sb.toString();
}

// ---------------- adjust tools ----------------
JSONObject loadedRangeNumber(String desc, float lo, float hi) {
  JSONObject p = chatSchemaProp("number", desc);
  p.setFloat("minimum", lo);
  p.setFloat("maximum", hi);
  return p;
}

JSONObject loadedShapeToolDefinition() {
  JSONObject props = new JSONObject();
  JSONObject item = new JSONObject();
  item.setString("type", "number");
  JSONObject bends = new JSONObject();
  bends.setString("type", "array");
  bends.setJSONObject("items", item);
  bends.setInt("minItems", MOTOR_COUNT);
  bends.setInt("maxItems", MOTOR_COUNT);
  bends.setString("description", "OPTIONAL. New bend of each joint off straight, degrees, base to tip (11 numbers, -"
    + round(AI_MAX_TURN_DEG) + " to " + round(AI_MAX_TURN_DEG) + "). Start from the loaded bends and change only the joints the request is about.");
  props.setJSONObject("joint_bends", bends);
  props.setJSONObject("bend_scale", loadedRangeNumber("OPTIONAL. Multiply every joint's bend (applied after joint_bends): >1 = bigger / more curled, <1 = smaller / flatter.", 0.3, 2.0));
  props.setJSONObject("message_to_user", chatSchemaProp("string", "1-2 short sentences for the user about what you changed."));
  JSONObject schema = new JSONObject();
  schema.setString("type", "object");
  schema.setJSONObject("properties", props);
  JSONArray req = new JSONArray();
  req.append("message_to_user");
  schema.setJSONArray("required", req);
  JSONObject tool = new JSONObject();
  tool.setString("name", "adjust_loaded_shape");
  tool.setString("description", "Change the LOADED shape in place (bends, torque, stiffness), keeping everything not mentioned exactly as saved. Use this for any change to the loaded shape.");
  tool.setJSONObject("input_schema", schema);
  chatAddTorqueField(tool, "OPTIONAL. New torque for each joint, 11 integers 0-100, base to tip (0 = motor off, a free hinge). Leave out to keep the loaded torque.");
  chatAddStiffnessField(tool, "OPTIONAL. New stiffness level for each joint, 11 integers 0-3, base to tip. Leave out to keep the loaded stiffness.");
  return tool;
}

JSONObject loadedAnimToolDefinition() {
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
  tool.setString("name", "adjust_loaded_animation");
  tool.setString("description", "Change the LOADED animation in place (speed, size of the swing, torque, stiffness), keeping the motion itself exactly as recorded. Use this for any change it can express.");
  tool.setJSONObject("input_schema", schema);
  chatAddTorqueField(tool, "OPTIONAL. New torque for each joint for the whole animation, 11 integers 0-100, base to tip (0 = motor off, free-swinging). Leave out to keep the recorded torque.");
  chatAddStiffnessField(tool, "OPTIONAL. New stiffness level for each joint for the whole animation, 11 integers 0-3, base to tip. Leave out to keep the recorded stiffness.");
  return tool;
}

// Applies adjust_loaded_shape (drawing thread). Returns the result text for the agent.
String loadedApplyShapeAdjust(JSONObject in) {
  LoadedContext c = loaded;
  if (c == null || !c.kind.equals("shape")) return "ERROR: no shape is loaded any more - nothing changed.";
  float[] ang = c.angles.clone();
  if (in.hasKey("joint_bends")) {
    try {
      JSONArray b = in.getJSONArray("joint_bends");
      if (b.size() == MOTOR_COUNT) for (int i = 0; i < MOTOR_COUNT; i++) ang[i] = 180 + b.getFloat(i);
    } catch (Exception ex) { }
  }
  if (in.hasKey("bend_scale")) {
    float s = constrain(in.getFloat("bend_scale"), 0.3, 2.0);
    for (int i = 0; i < MOTOR_COUNT; i++) ang[i] = 180 + (ang[i] - 180) * s;
  }
  for (int i = 0; i < MOTOR_COUNT; i++) ang[i] = clampAI(ang[i]);
  int[] tq = agentsParseTorque(in), st = agentsParseStiffness(in);

  // Still WAITING for its first Create Shape -> update the waiting item (Staging.pde).
  if (stagedShapeWaiting(c.name)) {
    stagedUpdateShape(ang, tq, st);
    if (tq != null) { c.torque = tq; agentShapeTorque = tq; }
    if (st != null) { c.stiffness = st; agentShapeStiffness = st; }
    c.angles = ang;
    c.edits++;
    return "Adjusted the loaded shape '" + c.name + "' (still the current shape, still WAITING): now " + loadedShortParams(c)
      + ". Pose, torque and stiffness go to the arm when the user presses Create Shape.";
  }
  float[] small = new float[MOTOR_COUNT], large = new float[MOTOR_COUNT];
  for (int i = 0; i < MOTOR_COUNT; i++) {
    small[i] = 180 + (ang[i] - 180) * AI_SIZE_MIN_SCALE;
    large[i] = 180 + (ang[i] - 180) * AI_SIZE_MAX_SCALE;
  }
  shapeLibrarySmall.put(c.name, small);
  shapeLibraryLarge.put(c.name, large);
  stageShapePose(c.name, ang, false); // new bends: ghost until Create Shape (Staging.pde)
  applyJointSettingsNow(tq, st);      // torque / stiffness: instant
  if (tq != null) { c.torque = tq; agentShapeTorque = tq; }
  if (st != null) { c.stiffness = st; agentShapeStiffness = st; }
  c.angles = ang;
  c.edits++;
  return "Adjusted the loaded shape '" + c.name + "' (still the current shape). Now: " + loadedShortParams(c)
    + ". The new bends are shown as a ghost preview (the user presses Create Shape to send them); torque / stiffness changes were applied immediately.";
}

// Applies adjust_loaded_animation (drawing thread). Returns the result text for the agent.
String loadedApplyAnimAdjust(JSONObject in) {
  LoadedContext c = loaded;
  if (c == null || !c.kind.equals("animation") || c.rec == null) return "ERROR: no animation is loaded any more - nothing changed.";
  AnimRecording o = c.rec, r = new AnimRecording();
  float speed = in.hasKey("speed_factor") ? constrain(in.getFloat("speed_factor"), 0.25, 4.0) : 1;
  float amp = in.hasKey("amplitude_factor") ? constrain(in.getFloat("amplitude_factor"), 0.3, 2.0) : 1;
  int[] tq = agentsParseTorque(in), st = agentsParseStiffness(in);
  float[] mean = new float[MOTOR_COUNT];
  for (float[] f : o.frames) for (int j = 0; j < MOTOR_COUNT; j++) mean[j] += f[j] / o.frames.size();
  for (int k = 0; k < o.frames.size(); k++) {
    r.t.add(round(o.t.get(k) / speed));
    float[] f = new float[MOTOR_COUNT];
    for (int j = 0; j < MOTOR_COUNT; j++) f[j] = clampJoint(mean[j] + (o.frames.get(k)[j] - mean[j]) * amp);
    r.frames.add(f);
    r.torque.add(tq != null ? tq.clone() : o.torque.get(k).clone());
    r.stiffness.add(st != null ? st.clone() : o.stiffness.get(k).clone());
  }
  r.durationMs = round(o.durationMs / speed);
  r.resetMs = max(150, round(o.resetMs / speed));
  r.intervalMs = max(1, round(o.intervalMs / speed));
  c.rec = r;
  c.edits++;
  String how = animApplyRecordingChange(c.name, r, c.source); // instant if playing (AnimLibrary.pde)
  return "Adjusted the loaded animation '" + c.name + "' (still the current animation; "
    + (how.equals("playing") ? "the change is playing now" : "READY - it starts when the user presses Create Shape") + ")"
    + (speed != 1 ? ", speed x" + nf(speed, 1, 2) : "") + (amp != 1 ? ", swing x" + nf(amp, 1, 2) : "")
    + (tq != null ? ", torque " + motorsTorqueList(tq) : "") + (st != null ? ", stiffness " + motorsTorqueList(st) : "")
    + ". Now: " + loadedShortParams(c) + ".";
}
