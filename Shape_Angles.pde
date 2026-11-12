
void generateShapeAngles() {
  float t = constrain(shapeSize, 0, 1);

  float[] small = null;
  float[] large = null;

  if (selectedShape.startsWith("ai: ")) {
    small = aiSmallAngles;
    large = aiLargeAngles;
  } else {
    String key = selectedShape.trim().toLowerCase();
    small = shapeLibrarySmall.get(key);
    large = shapeLibraryLarge.get(key);
  }

  if (small == null || large == null) {
    println("generateShapeAngles: no small/large angles for shape \"" + selectedShape + "\" - sliders left as they are.");
    return;
  }

  int n = min(MOTOR_COUNT, min(small.length, large.length));
  for (int i = 0; i < n; i++) {
    // Same safe range the rest of the sketch uses for the real servos.
    generatedAngles[i] = constrain(lerp(small[i], large[i], t), 60, 310);
  }
}

// Copies generatedAngles[] onto the "Motor 2".."Motor 12" sliders without
// firing controlEvent() for each one. Only moves the sliders / 3D preview -
// press Create Shape to send the angles to the Arduino.
void applyGeneratedAnglesToSliders() {
  updatingShapeControls = true;
  for (int position = 1; position <= MOTOR_COUNT; position++) {
    int dxlId = position + 1; // position 1 = hardware Dynamixel ID 2
    Controller slider = cp5.getController("Motor " + dxlId);
    if (slider == null) continue;
    slider.setBroadcast(false);
    slider.setValue(generatedAngles[position - 1]);
    slider.setBroadcast(true);
  }
  updatingShapeControls = false;
}
