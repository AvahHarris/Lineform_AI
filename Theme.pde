// =====================================================================
//  THEME - colors, fonts and backing panels for the whole window.
//  Change the colors here to restyle everything in one place.
// =====================================================================

// Background and panels
final color THEME_BG           = color(13, 17, 31);        // deep navy behind the 3D arm
final color THEME_PANEL        = color(24, 30, 50, 230);   // backing panels
final color THEME_PANEL_EDGE   = color(62, 76, 120);
final color THEME_TEXT         = color(244, 247, 255);
final color THEME_MUTED        = color(165, 176, 205);

// Bright accents
final color THEME_BLUE         = color(64, 140, 255);
final color THEME_CYAN         = color(0, 200, 255);
final color THEME_TEAL         = color(0, 210, 170);
final color THEME_GREEN        = color(46, 204, 113);
final color THEME_ORANGE       = color(255, 150, 50);
final color THEME_PINK         = color(255, 77, 140);
final color THEME_PURPLE       = color(150, 105, 255);
final color THEME_YELLOW       = color(255, 214, 64);
final color THEME_RED          = color(255, 90, 90);

// Controls
final color THEME_CTRL_BG      = color(40, 49, 78);        // slider tracks, switch tracks, text box
final color THEME_CTRL_HOVER   = color(60, 72, 110);

// 3D arm links
final color THEME_LINK_A       = color(78, 90, 122);       // odd links
final color THEME_LINK_B       = color(80, 155, 255);      // even links
final color THEME_LINK_PASSIVE = color(55, 60, 75);        // the fixed passive block

HashMap<Integer, PFont> uiBoldFontCache = new HashMap<Integer, PFont>();

PFont uiBoldFont(int px) {
  PFont f = uiBoldFontCache.get(px);
  if (f == null) {
    f = createFont("Arial Bold", px, true);
    uiBoldFontCache.put(px, f);
  }
  return f;
}

// A slightly brighter / darker version of a color, for hover and pressed states.
color themeShade(color c, float amount) {
  if (amount >= 0) return lerpColor(c, color(255), amount);
  return lerpColor(c, color(0), -amount);
}

void themeButton(String name, color base) {
  Controller c = cp5.getController(name);
  if (c == null) return;
  c.setColorBackground(base);
  c.setColorForeground(themeShade(base, 0.18));
  c.setColorActive(themeShade(base, -0.15));
  c.setColorCaptionLabel(color(255));
}

// Called once at the end of setup(), after every control exists.
void applyTheme() {
  // Sliders: dark track, bright cyan fill, white labels
  for (int id = 2; id <= 12; id++) {
    Controller s = cp5.getController("Motor " + id);
    if (s != null) {
      s.setColorBackground(THEME_CTRL_BG);
      s.setColorForeground(THEME_CYAN);
      s.setColorActive(themeShade(THEME_CYAN, 0.3));
      s.setColorCaptionLabel(THEME_TEXT);
      s.setColorValueLabel(color(255));
    }
    // ANGLE / TORQUE switches: cyan knob = ANGLE, dark = TORQUE
    Controller t = cp5.getController("View " + id);
    if (t != null) {
      t.setColorBackground(themeShade(THEME_ORANGE, -0.35));
      t.setColorForeground(themeShade(THEME_CYAN, 0.2));
      t.setColorActive(THEME_CYAN);
    }
    // Torque sliders: orange fill
    Controller tq = cp5.getController("Torque Level " + id);
    if (tq != null) {
      tq.setColorBackground(THEME_CTRL_BG);
      tq.setColorForeground(THEME_ORANGE);
      tq.setColorActive(themeShade(THEME_ORANGE, 0.3));
      tq.setColorCaptionLabel(THEME_ORANGE);
      tq.setColorValueLabel(color(255));
    }
  }

  Controller size = cp5.getController("Shape Size");
  if (size != null) {
    size.setColorBackground(THEME_CTRL_BG);
    size.setColorForeground(THEME_PURPLE);
    size.setColorActive(themeShade(THEME_PURPLE, 0.3));
    size.setColorCaptionLabel(THEME_TEXT);
    size.setColorValueLabel(color(255));
  }
  Controller hinges = cp5.getController("Show Hinges");
  if (hinges != null) {
    hinges.setColorBackground(THEME_CTRL_BG);
    hinges.setColorForeground(themeShade(THEME_PINK, 0.2));
    hinges.setColorActive(THEME_PINK);
    hinges.setColorCaptionLabel(THEME_TEXT);
  }

  // Buttons: one bright color per job
  themeButton("Create Shape", THEME_GREEN);
  themeButton("Reset", THEME_ORANGE);
  themeButton("Save Shape", THEME_PURPLE);
  themeButton("Stop Animation", THEME_PINK);
  themeButton("Send", THEME_BLUE);
  themeButton("New Chat", color(70, 82, 120));

  // Chat text box
  if (chatInput != null) {
    chatInput.setColorBackground(color(32, 40, 66));
    chatInput.setColorForeground(THEME_PANEL_EDGE);   // border
    chatInput.setColorActive(THEME_CYAN);             // border while typing
    chatInput.setColorValueLabel(color(255));
    chatInput.setColorCursor(THEME_CYAN);
  }

  chatSetMode(chatMode); // re-color the Shape / Animate switch
  chatSetCritic(chatCriticOn); // and the Critic: ON/OFF button
}

// Rounded backing panel in logical (1600x870) coordinates.
void themePanel(float x, float y, float w, float h) {
  pushStyle();
  fill(THEME_PANEL);
  stroke(THEME_PANEL_EDGE);
  strokeWeight(max(1, uiScale));
  rect(uiX(x), uiY(y), w * uiScale, h * uiScale, 10 * uiScale);
  popStyle();
}

// Drawn first in the HUD, so the controls and text sit on top of them.
void drawThemeBackdrops() {
  // Left column: motors, torque switches and the buttons under them
  themePanel(8, 8, 400, 600);

}

// Bottom-left info box: fit quality, what the colored lines mean, the
// running animation and hinge-marker help. Same width as the slider panel
// above it; text wraps to fit and the box grows with the number of lines.
final float INFO_X = 8;
final float INFO_Y = 620;
final float INFO_W = 400;
final float INFO_TEXT = 14;

void drawInfoBox() {
  ArrayList<String> texts = new ArrayList<String>();
  ArrayList<Integer> colors = new ArrayList<Integer>();

  if (aiHasPreview) {
    texts.add("yellow = what the arm actually makes");
    colors.add(color(255, 220, 0));
    texts.add("cyan = target shape (" + (selectedShape.startsWith("ai: ") ? selectedShape.substring(4) : selectedShape) + ")");
    colors.add(color(0, 220, 255));
    String historyKey = selectedShape.startsWith("ai: ")
      ? selectedShape.substring(4).trim().toLowerCase() : selectedShape.trim().toLowerCase();
    Float bestEver = aiBestFitQualityEver.get(historyKey);
    texts.add("fit quality: " + nf(aiLastFitQuality, 1, 2) + " (under " + nf(AI_REFINE_QUALITY_THRESHOLD, 1, 2)
      + " is good)" + (bestEver == null ? "" : " - best ever: " + nf(bestEver, 1, 2)));
    colors.add(aiLastFitQuality <= AI_REFINE_QUALITY_THRESHOLD ? color(120, 255, 120) : color(255, 140, 90));
  }

  if (aiAnimating) {
    if (aiAnimIsWave) {
      texts.add("ANIMATING '" + aiAnimName + "' - traveling wave, " + nf(aiAnimWaveAmplitudeDeg, 1, 0)
        + " deg, " + nf(aiAnimWaveWavelengthJoints, 1, 1) + " joints/cycle, " + round(aiAnimWavePeriodMs) + " ms/cycle");
    } else {
      texts.add("ANIMATING '" + aiAnimName + "' - pose " + (aiAnimIndex + 1) + "/" + aiAnimPoses.size()
        + ", " + aiAnimFrameDurationMs + " ms/pose");
    }
    colors.add(color(255, 100, 220));
  }

  if (showHingeMarkers) {
    texts.add("hinge offset X:" + nf(hingeOffsetX, 1, 1) + " Y:" + nf(hingeOffsetY, 1, 1) + " Z:" + nf(hingeOffsetZ, 1, 1)
      + " (arrows = X/Z, [ ] = Y). Tap a marker to soften that joint.");
    colors.add(color(0, 255, 140));
  }

  if (texts.isEmpty()) return;

  pushStyle();
  textFont(hudFont);
  hudTextSize(INFO_TEXT);
  textAlign(LEFT, TOP);
  float pad = 10;
  float lineH = INFO_TEXT * 1.35;
  float maxPx = (INFO_W - 2 * pad - 4) * uiScale;
  ArrayList<String> lines = new ArrayList<String>();
  ArrayList<Integer> lineColors = new ArrayList<Integer>();
  for (int i = 0; i < texts.size(); i++) {
    for (String ln : chatWrap(texts.get(i), maxPx)) {
      lines.add(ln);
      lineColors.add(colors.get(i));
    }
  }
  themePanel(INFO_X, INFO_Y, INFO_W, lines.size() * lineH + 2 * pad);
  for (int k = 0; k < lines.size(); k++) {
    fill(lineColors.get(k));
    hudText(lines.get(k), INFO_X + pad + 2, INFO_Y + pad + k * lineH);
  }
  popStyle();
}
