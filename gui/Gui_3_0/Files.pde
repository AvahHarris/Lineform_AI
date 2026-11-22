// =====================================================================
//  FILES (2026-11-21) - one JSON file per shape / animation, saved and
//  opened with the normal file dialogs (Processing's selectOutput /
//  selectInput: native dialogs on Windows, Mac and Linux).
//
//  * Save Shape      -> "Save shape as..."  -> e.g. shapes/nunchuck.json
//  * Load Shape      -> pick a shape file   -> it goes on the bars + its
//                       torque / stiffness; both agents are told (Agents.pde)
//  * Save Animation  -> records 3 s (AnimLibrary.pde), then "Save animation as..."
//  * Load Animation  -> pick an animation file -> it plays (looping)
//  Nothing is loaded at startup and typing a name in the chat loads nothing.
//  The dialogs start in the sketch's shapes/ and animations/ folders, but
//  files can live anywhere.
//
//  Shape file:
//    { "type": "shape", "name": "nunchuck",
//      "angles": [11 x deg],                (the saved pose, 180 = straight)
//      "small": [11], "large": [11],        (Shape Size range - optional)
//      "torque": [11 x 0-100], "stiffness": [11 x 0-3] }   (optional)
//  Animation file: see AnimLibrary.pde ("type": "recording").
// =====================================================================

String SHAPES_DIR = "shapes";
String ANIMATIONS_DIR = "animations";

File filesDefault(String dir, String name) {
  File d = new File(sketchPath(dir));
  if (!d.exists()) d.mkdirs();
  if (name == null) return d;
  String safe = name.trim().toLowerCase().replaceAll("[^a-z0-9 _-]", "").replaceAll("\\s+", "_");
  if (safe.length() == 0) safe = "untitled";
  return new File(d, safe + ".json");
}

String filesWithJson(File f) {
  String p = f.getAbsolutePath();
  return p.toLowerCase().endsWith(".json") ? p : p + ".json";
}

// Reads a .json file as an object. An array (e.g. an old shape_library.json)
// gives its first entry.
JSONObject filesReadObject(File f) {
  String[] lines = loadStrings(f.getAbsolutePath());
  if (lines == null) return null;
  String text = join(lines, "\n").trim();
  if (text.startsWith("[")) {
    JSONArray a = parseJSONArray(text);
    if (a == null || a.size() == 0) return null;
    if (a.size() > 1) noticeAdd("status", f.getName() + " holds " + a.size() + " entries - loaded the first one.");
    return a.getJSONObject(0);
  }
  return parseJSONObject(text);
}

void filesOnMain(Runnable r) { chatMainJobs.add(r); } // dialog callbacks run on their own thread

// ---------------- shapes ----------------
JSONObject filesPendingShape = null;

void filesSaveShapeButton() {
  String name;
  if (manipulationMode) {
    name = "hand pose " + year() + nf(month(), 2) + nf(day(), 2) + "-" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
  } else {
    String typed = (chatInput != null) ? chatInput.getText() : "";
    name = (typed != null && typed.trim().length() > 0) ? typed
      : (selectedShape.startsWith("ai: ") ? selectedShape.substring(4) : selectedShape);
    if (name == null || name.trim().length() == 0 || name.trim().equals("none")) name = "shape";
    if (chatInput != null && typed != null && typed.trim().length() > 0) chatInput.clear();
  }
  name = name.trim().toLowerCase();

  float[] cur = new float[MOTOR_COUNT];
  int[] tq = new int[MOTOR_COUNT];
  JSONArray ang = new JSONArray(), small = new JSONArray(), large = new JSONArray();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    cur[i] = cp5.getController("Motor " + (i + 2)).getValue();
    float turn = cur[i] - 180;
    ang.append(round(cur[i] * 10) / 10.0);
    small.append(round((180 + turn * AI_SIZE_MIN_SCALE) * 10) / 10.0);
    large.append(round((180 + turn * AI_SIZE_MAX_SCALE) * 10) / 10.0);
    tq[i] = motorPowerOn(i) ? motorTorquePct[i] : 0; // OFF rows are saved as limp
  }
  JSONObject e = new JSONObject();
  e.setString("type", "shape");
  e.setString("name", name);
  e.setJSONArray("angles", ang);
  e.setJSONArray("small", small);
  e.setJSONArray("large", large);
  e.setJSONArray("torque", libraryJsonInts(tq));
  e.setJSONArray("stiffness", libraryJsonInts(hingeStiffnessLevel.clone()));
  filesPendingShape = e;
  selectOutput("Save shape as...", "filesShapeSaveSelected", filesDefault(SHAPES_DIR, name));
}

void filesShapeSaveSelected(File f) {
  final JSONObject e = filesPendingShape;
  filesPendingShape = null;
  if (f == null || e == null) { noticeAdd("status", "Shape not saved (cancelled)."); return; }
  final String path = filesWithJson(f);
  saveJSONObject(e, path);
  noticeAdd("status", "Saved the shape \"" + e.getString("name") + "\" to " + new File(path).getName()
    + " (angles, torque, stiffness).");
  println("Saved shape to " + path);
}

void filesLoadShapeButton() {
  selectInput("Load a shape file", "filesShapeLoadSelected", filesDefault(SHAPES_DIR, null));
}

void filesShapeLoadSelected(final File f) {
  if (f == null) return;
  filesOnMain(new Runnable() {
    public void run() {
      JSONObject e = null;
      try { e = filesReadObject(f); } catch (Exception ex) { }
      if (e == null) { noticeAdd("error", "Couldn't read " + f.getName() + " as a shape file."); return; }
      if (e.hasKey("type") && !e.getString("type").equals("shape")) {
        noticeAdd("error", f.getName() + " is a " + e.getString("type") + " file, not a shape - use Load Animation.");
        return;
      }
      filesApplyShape(e, f.getName());
    }
  });
}

// Puts a shape entry on the bars (drawing thread).
void filesApplyShape(JSONObject e, String fileName) {
  if (manipulationMode) { noticeAdd("status", "Switch to Automation mode to load a shape."); return; }
  String name = e.hasKey("name") ? e.getString("name").trim().toLowerCase() : fileName.replaceAll("(?i)\\.json$", "");
  float[] small, large;
  float[] ang = e.hasKey("angles") ? e.getJSONArray("angles").getFloatArray() : null;
  if (ang != null && ang.length == MOTOR_COUNT) {
    small = new float[MOTOR_COUNT];
    large = new float[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) {
      small[i] = 180 + (ang[i] - 180) * AI_SIZE_MIN_SCALE;
      large[i] = 180 + (ang[i] - 180) * AI_SIZE_MAX_SCALE;
    }
  } else if (e.hasKey("small") && e.hasKey("large")) {
    small = e.getJSONArray("small").getFloatArray();
    large = e.getJSONArray("large").getFloatArray();
  } else {
    noticeAdd("error", fileName + " has no joint angles.");
    return;
  }
  if (small.length != MOTOR_COUNT || large.length != MOTOR_COUNT) {
    noticeAdd("error", fileName + " doesn't have " + MOTOR_COUNT + " joint angles.");
    return;
  }
  shapeLibrarySmall.put(name, small);
  shapeLibraryLarge.put(name, large);
  int[] tq = libraryIntArray(e, "torque", 0, 100);
  int[] st = libraryIntArray(e, "stiffness", 0, NUM_STIFFNESS_LEVELS - 1);
  if (tq != null) shapeLibraryTorque.put(name, tq); else shapeLibraryTorque.remove(name);
  if (st != null) shapeLibraryStiffness.put(name, st); else shapeLibraryStiffness.remove(name);

  aiStopAnimation();
  // Shape Size 0.5 = exactly the saved pose.
  Controller ss = cp5.getController("Shape Size");
  ss.setBroadcast(false);
  ss.setValue(0.5);
  ss.setBroadcast(true);
  aiUseKnownShape(name, fileName); // bars + torque / stiffness + tells both agents
  noticeAdd("status", "Loaded the shape \"" + name + "\" from " + fileName + ". Press Create Shape to send it to the arm.");
}

// ---------------- animations ----------------
JSONObject filesPendingAnim = null;

// Called by AnimLibrary.pde when a 3 s recording is finished.
void filesAskSaveAnimation(JSONObject e) {
  filesPendingAnim = e;
  selectOutput("Save animation as...", "filesAnimSaveSelected", filesDefault(ANIMATIONS_DIR, e.getString("name")));
}

void filesAnimSaveSelected(File f) {
  JSONObject e = filesPendingAnim;
  filesPendingAnim = null;
  if (f == null || e == null) { noticeAdd("status", "Animation not saved (cancelled)."); return; }
  String path = filesWithJson(f);
  saveJSONObject(e, path);
  noticeAdd("status", "Saved the animation \"" + e.getString("name") + "\" to " + new File(path).getName() + ".");
  println("Saved animation to " + path);
}

void filesLoadAnimationButton() {
  selectInput("Load an animation file", "filesAnimLoadSelected", filesDefault(ANIMATIONS_DIR, null));
}

void filesAnimLoadSelected(final File f) {
  if (f == null) return;
  filesOnMain(new Runnable() {
    public void run() {
      JSONObject e = null;
      try { e = filesReadObject(f); } catch (Exception ex) { }
      if (e == null) { noticeAdd("error", "Couldn't read " + f.getName() + " as an animation file."); return; }
      if (!e.hasKey("frames")) {
        noticeAdd("error", f.getName() + " isn't an animation recording" + (e.hasKey("type") && e.getString("type").equals("shape")
          ? " - it's a shape, use Load Shape." : "."));
        return;
      }
      if (!e.hasKey("name")) e.setString("name", f.getName().replaceAll("(?i)\\.json$", ""));
      animLibraryPlay(e);
    }
  });
}
