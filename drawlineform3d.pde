void drawlineform3d() {

  // Read sliders - control names are "Motor 2".."Motor 12" (the real
  // hardware Dynamixel IDs; ID 1 is the passive block and has no slider),
  // but val_1..val_11 stay POSITION-named (1st through 11th active joint)
  // since that's what motorAngles[]/activeMotors below are indexed by.
  float val_1 = cp5.get(Slider.class, "Motor 2").getValue();
  float val_2 = cp5.get(Slider.class, "Motor 3").getValue();
  float val_3 = cp5.get(Slider.class, "Motor 4").getValue();
  float val_4 = cp5.get(Slider.class, "Motor 5").getValue();
  float val_5 = cp5.get(Slider.class, "Motor 6").getValue();
  float val_6 = cp5.get(Slider.class, "Motor 7").getValue();
  float val_7 = cp5.get(Slider.class, "Motor 8").getValue();
  float val_8 = cp5.get(Slider.class, "Motor 9").getValue();
  float val_9 = cp5.get(Slider.class, "Motor 10").getValue();
  float val_10 = cp5.get(Slider.class, "Motor 11").getValue();
  float val_11 = cp5.get(Slider.class, "Motor 12").getValue();

  float[] motorAngles = { val_1, val_2, val_3, val_4, val_5, val_6, val_7, val_8, val_9, val_10, val_11};
  int activeMotors = 11;

  float gap = LINK_GAP;
  float linkWidth = LINK_WIDTH;
  float linkHeight = LINK_HEIGHT;
  float hinge = LINK_HINGE;

  // Collect each joint's actual 3D position as we walk the chain, so we
  // can draw a curve through the real linkage shape afterward. Reading
  // modelX/Y/Z here (rather than guessing a transform elsewhere) means
  // this curve is always exactly where the boxes are, however the chain
  // geometry changes.
  ArrayList<PVector> jointCurve = new ArrayList<PVector>();

  // One entry per ACTIVE joint (1-11, parallel to the loop below) holding
  // that joint's actual pivot point - captured the instant AFTER this
  // joint's own rotate() but BEFORE translating out to its box center, so
  // it's the true hinge location, not the link's midpoint. jointCurve above
  // only ever captures box centers (plus the very first pivot, joint 1's,
  // which happens to sit at CHAIN_ORIGIN) - this is what "Show Hinges"
  // draws from.
  ArrayList<PVector> hingePositions = new ArrayList<PVector>();

  pushMatrix();
  translate(CHAIN_ORIGIN.x, CHAIN_ORIGIN.y, CHAIN_ORIGIN.z);   // where the first ACTIVE joint begins

  // Passive block (Dynamixel hardware ID 1, left untorqued in the new
  // lineform build) sits one segment BEHIND CHAIN_ORIGIN - drawn by
  // stepping backward before the active chain starts, so CHAIN_ORIGIN
  // itself keeps meaning exactly what it always has and the IK math
  // (aiComputeAchievableCurve / aiSolveAnglesForCurve) needs no changes,
  // only this drawing does. Never rotates - it isn't driven by any slider.
  pushMatrix();
  translate(-(hinge + gap), 0, 0);
  translate(gap + linkWidth / 2.0 + (linkWidth - hinge), 0, 0);
  translate(0, 0, linkHeight / 2.0);
  fill(THEME_LINK_PASSIVE);
  stroke(255, 100);
  strokeWeight(1);
  box(linkWidth, linkHeight, linkHeight);
  pushMatrix();
  translate(0, 0, linkHeight/2 + 1);
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(16);
  text("12", 0, 0); // real Dynamixel ID of the fixed (passive) base block
  popMatrix();
  popMatrix();

  jointCurve.add(new PVector(modelX(0, 0, 0), modelY(0, 0, 0), modelZ(0, 0, 0)));

  for (int i = 0; i < activeMotors; i++) {

    int motorNumber = i + 1;    // software "Motor N" slider index (1-11)
    int dxlId = motorNumber + 1; // real hardware Dynamixel ID (2-12) - ID 1 is the passive block above

    float safeAngle = constrain(motorAngles[i], 60, 300); //1. rotate first
    float angle = safeAngle - 180;

    if (aiIsMirroredMotor(dxlId)) {
        angle = -angle;
    }

    float rad = radians(angle);

  // Rotate at the joint before drawing the segment
    if (motorNumber % 2 == 1) {
      rotateZ(-rad);
    } else {
      rotateY(rad);
    }

    // This IS the hinge: the origin right here, after the rotate above and
    // before any translate toward the box, is exactly where this joint's
    // simulated pivot sits. hingeOffsetX/Y/Z (Gui_07_07.pde) nudges that
    // onto where the real Dynamixel's shaft actually is, in this same
    // local frame - same offset for every joint, since the hardware is
    // identical at each one. Captured now so "Show Hinges" can mark it
    // later, once we're back out of this matrix stack.
    pushMatrix();
    translate(hingeOffsetX, hingeOffsetY, hingeOffsetZ);
    hingePositions.add(new PVector(modelX(0, 0, 0), modelY(0, 0, 0), modelZ(0, 0, 0)));
    popMatrix();

    // draw current motor block first
    pushMatrix();
    translate(gap + linkWidth / 2.0 + (linkWidth - hinge), 0, 0); //draw where box will go lenght of gap + width + hinge? 
    translate(0, 0, linkHeight / 2.0); 

    stroke(255, 100);
    strokeWeight(1);

    if (motorNumber % 2 == 1) fill(THEME_LINK_A);
    else fill(THEME_LINK_B);

    box(linkWidth, linkHeight, linkHeight); //draw box

    // Record this box's actual center in world space (not the hinge point
    // beside it), so the curve is glued straight through the boxes instead
    // of floating along the pivot line next to them.
    jointCurve.add(new PVector(modelX(0, 0, 0), modelY(0, 0, 0), modelZ(0, 0, 0)));

    pushMatrix();

    translate(0, 0, linkHeight/2 + 1);  // move to front face for text

    fill(255);
    textAlign(CENTER, CENTER);
    textSize(16);
    text(hwId(dxlId), 0, 0); // real Dynamixel ID (11 next to the base ... 1 at the tip)

    popMatrix();

    popMatrix();

    translate(hinge+gap, 0, 0);
  }

  // No trailing passive block here anymore - the new lineform build moved
  // the passive Dynamixel to the FRONT (drawn above, before the loop).
  // Hardware ID 12, the old passive position, is now an active motor and
  // already drawn as the last iteration of the loop above.
  popMatrix();

  // Hinge reference markers - a small sphere + "Hn Lx" tag (n = the real
  // hardware Dynamixel ID, matching the numbers already printed on the
  // boxes; x = that joint's current stiffness level, 0 = full stiffness)
  // at each active joint's actual pivot point. Pure reference for
  // designing a shape by hand on the sliders: toggled on/off from the
  // "Show Hinges" switch, never read by any shape/fitting code.
  //
  // Color is red at level 0, fading toward pale pink as the level (and so
  // the Position P Gain) drops - a tap softens one more notch, so a
  // lighter marker reads as "this one's gone loose." The hit-test against
  // a pending click (hingeClickPending, set by mousePressed() in
  // Gui_07_07.pde) has to happen HERE, in the same draw() frame the
  // markers are actually rendered in, because screenX()/screenY() need
  // this frame's live camera transform to mean anything.
  if (showHingeMarkers) {
    float bestClickDist = HINGE_CLICK_RADIUS;
    int bestClickDxlId = -1;

    for (int i = 0; i < hingePositions.size(); i++) {
      PVector hp = hingePositions.get(i);
      int dxlId = i + 2; // position 1 (first active joint) = hardware ID 2
      int level = hingeStiffnessLevel[i];

      color markerColor = lerpColor(
        color(255, 0, 0),
        color(255, 190, 190),
        level / float(NUM_STIFFNESS_LEVELS - 1)
      );

      pushMatrix();
      translate(hp.x, hp.y, hp.z);
      noStroke();
      fill(markerColor);
      sphere(4);

      translate(0, 0, 14);
      fill(markerColor);
      textAlign(CENTER, CENTER);
      textSize(11);
      text("H" + hwId(dxlId) + " L" + level, 0, 0);
      popMatrix();

      if (hingeClickPending) {
        float sx = screenX(hp.x, hp.y, hp.z);
        float sy = screenY(hp.x, hp.y, hp.z);
        float d = dist(sx, sy, hingeClickX, hingeClickY);
        if (d < bestClickDist) {
          bestClickDist = d;
          bestClickDxlId = dxlId;
        }
      }
    }

    if (hingeClickPending && bestClickDxlId != -1) {
      // Cycle this joint's stiffness level and push it to the Arduino,
      // which looks the level up in its own table (STIFFNESS_P_GAINS) and
      // writes Position P Gain directly - torque stays on throughout, so
      // this is a live feel change, not a drop-then-recover snap.
      int idx = bestClickDxlId - 2;
      hingeStiffnessLevel[idx] = (hingeStiffnessLevel[idx] + 1) % NUM_STIFFNESS_LEVELS;
      sendHingeStiffness(bestClickDxlId, hingeStiffnessLevel[idx]);
      println("Hinge " + hwId(bestClickDxlId) + " stiffness level " + hingeStiffnessLevel[idx]
        + " (P gain " + STIFFNESS_P_GAIN_DISPLAY[hingeStiffnessLevel[idx]] + ")");
    }
  }

  // However the tap above resolved (hit a marker, missed, or markers were
  // hidden entirely this frame), the click is consumed now - it must never
  // carry over into next frame's hit-test.
  hingeClickPending = false;

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
