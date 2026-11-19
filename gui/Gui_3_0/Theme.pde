// =====================================================================
//  THEME - colors, fonts and backing panels for the whole window.
//  Change the colors here to restyle everything in one place.
// =====================================================================

// Background and panels. NOT final: these (and the 3D link colors and
// THEME_ACCENT below) switch between the Automation-mode set and the
// manipulation-mode set - see themeApplyMode().
color THEME_BG                 = color(13, 17, 31);        // deep navy behind the 3D arm
color THEME_PANEL              = color(24, 30, 50, 230);   // backing panels
color THEME_PANEL_EDGE         = color(62, 76, 120);
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
color THEME_LINK_A             = color(78, 90, 122);       // odd links
color THEME_LINK_B             = color(80, 155, 255);      // even links
final color THEME_LINK_PASSIVE = color(55, 60, 75);        // the fixed passive block

// Angle-bar fill / mode accent (cyan in Automation mode, amber in Manipulation mode)
color THEME_ACCENT             = color(0, 200, 255);

// ---- Manipulation-mode color set: warm amber on dark brown, so it is
// obvious at a glance that the motors are OFF and the arm is free. ----
final color MANIP_BG           = color(34, 20, 10);
final color MANIP_PANEL        = color(58, 36, 18, 235);
final color MANIP_PANEL_EDGE   = color(200, 120, 40);
final color MANIP_ACCENT       = color(255, 170, 40);      // amber
final color MANIP_LINK_A       = color(130, 85, 45);
final color MANIP_LINK_B       = color(255, 160, 50);

// ---- Automation-mode color set (the original theme) ----
final color AI_BG              = color(13, 17, 31);
final color AI_PANEL           = color(24, 30, 50, 230);
final color AI_PANEL_EDGE      = color(62, 76, 120);
final color AI_ACCENT          = color(0, 200, 255);
final color AI_LINK_A          = color(78, 90, 122);
final color AI_LINK_B          = color(80, 155, 255);

// Switches the mode-dependent colors and re-colors the angle bars.
// Called from applyTheme() and whenever the Automation / Manipulation switch flips.
void themeApplyMode() {
  boolean m = manipulationMode;
  THEME_BG         = m ? MANIP_BG : AI_BG;
  THEME_PANEL      = m ? MANIP_PANEL : AI_PANEL;
  THEME_PANEL_EDGE = m ? MANIP_PANEL_EDGE : AI_PANEL_EDGE;
  THEME_ACCENT     = m ? MANIP_ACCENT : AI_ACCENT;
  THEME_LINK_A     = m ? MANIP_LINK_A : AI_LINK_A;
  THEME_LINK_B     = m ? MANIP_LINK_B : AI_LINK_B;
  if (cp5 == null) return;
  for (int id = 2; id <= 12; id++) {
    Controller s = cp5.getController("Motor " + id);
    if (s == null) continue;
    s.setColorForeground(THEME_ACCENT);
    s.setColorActive(themeShade(THEME_ACCENT, 0.3));
    s.setColorBackground(m ? color(80, 52, 28) : THEME_CTRL_BG);
  }
  if (chatInput != null) chatInput.setColorForeground(THEME_PANEL_EDGE);
}

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
    // Stiffness bars: purple fill (the STIFF segment of the row selector)
    Controller st = cp5.getController("Stiffness " + id);
    if (st != null) {
      st.setColorBackground(THEME_CTRL_BG);
      st.setColorForeground(THEME_PURPLE);
      st.setColorActive(themeShade(THEME_PURPLE, 0.3));
      st.setColorCaptionLabel(THEME_PURPLE);
    }
    // Torque ON/OFF switches
    Controller pw = cp5.getController("Power " + id);
    if (pw != null) {
      pw.setColorBackground(themeShade(THEME_ORANGE, -0.35));
      pw.setColorForeground(themeShade(THEME_CYAN, 0.2));
      pw.setColorActive(THEME_CYAN);
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
  themeApplyMode();      // Automation / Manipulation colors (angle bars, backdrops, 3D links)
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
  themePanel(8, 8, 540, 600); // wider for the longer bars (was 500)

}

// Bottom-left info box: fit quality, what the colored lines mean, the
// running animation and hinge-marker help. Same width as the slider panel
// above it; text wraps to fit and the box grows with the number of lines.
final float INFO_X = 8;
final float INFO_Y = 620;
final float INFO_W = 400;
final float INFO_TEXT = 14;

// Returns the box's bottom edge (logical y), or -1 if nothing was drawn -
// the notice panel (Notices.pde) stays below it.
float drawInfoBox() {
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

  if (texts.isEmpty()) return -1;

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
  float boxH = lines.size() * lineH + 2 * pad;
  themePanel(INFO_X, INFO_Y, INFO_W, boxH);
  for (int k = 0; k < lines.size(); k++) {
    fill(lineColors.get(k));
    hudText(lines.get(k), INFO_X + pad + 2, INFO_Y + pad + k * lineH);
  }
  popStyle();
  return INFO_Y + boxH;
}
