// =====================================================================
//  SHAPE CHAT - the panel on the right side of the window.
//
//  Three AI agents work behind this one conversation (see Agents.pde):
//    - DESIGNER      (Shape mode)   makes and edits still shapes
//    - MOTION        (Animate mode) turns the current shape into motion
//    - VISUAL CRITIC (switchable)   looks at a picture of what the arm
//                                   actually made and tells the Designer
//                                   if it doesn't read as what you asked
//  Each agent has its own memory. The Designer's current shape is handed to
//  the Motion agent, so "make it wave" moves the shape you just made.
//
//  Requests run on a background thread so the window, sliders and any
//  running animation keep working while the agents think. Anything that
//  changes the arm is handed back to the drawing thread via chatRunOnMain().
// =====================================================================

// Panel layout, in the same logical 1600x870 coordinates as the rest of the GUI.
final float CHAT_X = 1180;
final float CHAT_Y = 20;
final float CHAT_W = 405;
final float CHAT_H = 812;
final float CHAT_PAD = 10;
final float CHAT_HEADER_H = 40;
final float CHAT_INPUT_H = 44;
final float CHAT_SEND_W = 76;
final float CHAT_TEXT_SIZE = 16;
final float CHAT_BUBBLE_PAD = 8;
final float CHAT_BUBBLE_GAP = 10;
final float CHAT_MODE_H = 26;      // Shape / Animate switch row, just above the input box
final float CHAT_MODE_W = 92;
final float CHAT_CRITIC_W = 100;   // "Critic: ON" button in the header

class ChatEntry {
  String who;   // "you", "claude" (Designer), "motion", "critic", "status", "error"
  String text;
  ChatEntry(String who, String text) {
    this.who = who;
    this.text = text;
  }
}

ArrayList<ChatEntry> chatLog = new ArrayList<ChatEntry>();  // what the panel shows

Textfield chatInput;
// "shape" -> the Designer answers, "animate" -> the Motion agent answers.
String chatMode = "shape";
boolean chatCriticOn = true;
volatile boolean chatBusy = false;
volatile String chatStatus = "";
volatile String chatQueuedMessage = null;
volatile String chatQueuedMode = "shape";
float chatScroll = 0;      // logical px scrolled up from the newest message
float chatMaxScroll = 0;   // updated every frame by drawChatPanel()

java.util.concurrent.ConcurrentLinkedQueue<Runnable> chatMainJobs = new java.util.concurrent.ConcurrentLinkedQueue<Runnable>();

// ---------------------------------------------------------------------
//  Controls and layout
// ---------------------------------------------------------------------

void setupChatPanel() {
  chatInput = cp5.addTextfield("Chat Input")
    .setPosition(CHAT_X + CHAT_PAD, CHAT_Y + CHAT_H - CHAT_PAD - CHAT_INPUT_H)
    .setSize(round(CHAT_W - 3 * CHAT_PAD - CHAT_SEND_W), round(CHAT_INPUT_H))
    .setFont(createFont("Arial", 16))
    .setAutoClear(false)
    .setLabel("");
  chatInput.getCaptionLabel().setVisible(false);

  cp5.addButton("Send")
    .setPosition(CHAT_X + CHAT_W - CHAT_PAD - CHAT_SEND_W, CHAT_Y + CHAT_H - CHAT_PAD - CHAT_INPUT_H)
    .setSize(round(CHAT_SEND_W), round(CHAT_INPUT_H))
    .setLabel("Send");

  cp5.addButton("New Chat")
    .setPosition(CHAT_X + CHAT_W - CHAT_PAD - 100, CHAT_Y + 7)
    .setSize(100, 26)
    .setLabel("New chat");

  cp5.addButton("Critic Toggle")
    .setPosition(CHAT_X + CHAT_W - CHAT_PAD - 100 - 6 - CHAT_CRITIC_W, CHAT_Y + 7)
    .setSize(round(CHAT_CRITIC_W), 26)
    .setLabel("Critic: ON");

  cp5.addButton("Mode Shape")
    .setPosition(CHAT_X + CHAT_PAD + 52, CHAT_Y + CHAT_H - CHAT_PAD - CHAT_INPUT_H - CHAT_MODE_H - 8)
    .setSize(round(CHAT_MODE_W), round(CHAT_MODE_H))
    .setLabel("Shape");
  cp5.addButton("Mode Animate")
    .setPosition(CHAT_X + CHAT_PAD + 52 + CHAT_MODE_W + 6, CHAT_Y + CHAT_H - CHAT_PAD - CHAT_INPUT_H - CHAT_MODE_H - 8)
    .setSize(round(CHAT_MODE_W), round(CHAT_MODE_H))
    .setLabel("Animate");
  chatSetMode("shape");
  chatSetCritic(true);

  chatAdd("claude", "Hi! Three agents work in this chat. In Shape mode I'm the Designer - "
    + "tell me a shape (\"a hook\", \"a heart\") and keep adjusting it (\"curl the tip "
    + "tighter\", \"bigger\"). The Critic checks a picture of what the arm really made "
    + "and sends me fixes. Switch to Animate and the Motion agent brings your shape to "
    + "life (\"make it wave\", \"slower\", \"softer tail\").");
}

// Called from layoutResponsiveGUI() whenever the window size changes.
void layoutChatPanel() {
  float inputY = CHAT_Y + CHAT_H - CHAT_PAD - CHAT_INPUT_H;
  positionControl("Chat Input", CHAT_X + CHAT_PAD, inputY, CHAT_W - 3 * CHAT_PAD - CHAT_SEND_W, CHAT_INPUT_H);
  positionControl("Send", CHAT_X + CHAT_W - CHAT_PAD - CHAT_SEND_W, inputY, CHAT_SEND_W, CHAT_INPUT_H);
  positionControl("New Chat", CHAT_X + CHAT_W - CHAT_PAD - 100, CHAT_Y + 7, 100, 26);
  positionControl("Critic Toggle", CHAT_X + CHAT_W - CHAT_PAD - 100 - 6 - CHAT_CRITIC_W, CHAT_Y + 7, CHAT_CRITIC_W, 26);
  float modeY = inputY - CHAT_MODE_H - 8;
  positionControl("Mode Shape", CHAT_X + CHAT_PAD + 52, modeY, CHAT_MODE_W, CHAT_MODE_H);
  positionControl("Mode Animate", CHAT_X + CHAT_PAD + 52 + CHAT_MODE_W + 6, modeY, CHAT_MODE_W, CHAT_MODE_H);
  PFont f = uiBoldFont(max(8, round(14 * uiScale)));
  for (String n : new String[] {"Mode Shape", "Mode Animate", "Critic Toggle"}) {
    Controller c = cp5.getController(n);
    if (c != null) c.getCaptionLabel().setFont(f);
  }
}

// Highlights the chosen mode's button.
void chatSetMode(String mode) {
  chatMode = mode;
  Controller shapeBtn = cp5.getController("Mode Shape");
  Controller animBtn = cp5.getController("Mode Animate");
  if (shapeBtn == null || animBtn == null) return;
  // Shape = blue, Animate = pink; the one not chosen stays dark.
  color off = color(40, 49, 78);
  color offHover = color(60, 72, 110);
  boolean s = mode.equals("shape");
  shapeBtn.setColorBackground(s ? color(64, 140, 255) : off);
  shapeBtn.setColorForeground(s ? color(100, 165, 255) : offHover);
  shapeBtn.setColorCaptionLabel(s ? color(255) : color(165, 176, 205));
  animBtn.setColorBackground(!s ? color(255, 77, 140) : off);
  animBtn.setColorForeground(!s ? color(255, 115, 165) : offHover);
  animBtn.setColorCaptionLabel(!s ? color(255) : color(165, 176, 205));
}

// Turns the Visual Critic on/off and recolors its button (yellow = on).
void chatSetCritic(boolean on) {
  chatCriticOn = on;
  Controller b = cp5.getController("Critic Toggle");
  if (b == null) return;
  b.setLabel(on ? "Critic: ON" : "Critic: OFF");
  b.setColorBackground(on ? color(255, 214, 64) : color(40, 49, 78));
  b.setColorForeground(on ? color(255, 228, 120) : color(60, 72, 110));
  b.setColorActive(on ? color(220, 180, 40) : color(30, 38, 62));
  b.setColorCaptionLabel(on ? color(30, 30, 40) : color(165, 176, 205));
}

boolean chatMouseOver() {
  return mouseX >= uiX(CHAT_X) && mouseX <= uiX(CHAT_X + CHAT_W)
      && mouseY >= uiY(CHAT_Y) && mouseY <= uiY(CHAT_Y + CHAT_H);
}

// Mouse wheel over the panel scrolls the conversation (the camera is
// switched off there in draw(), so it doesn't zoom at the same time).
void mouseWheel(processing.event.MouseEvent event) {
  if (!chatMouseOver()) return;
  chatScroll = constrain(chatScroll - event.getCount() * 40, 0, chatMaxScroll);
}

// Returns true if the event belonged to the chat panel.
boolean chatHandleControlEvent(String name) {
  if (name.equals("Send") || name.equals("Chat Input")) {
    chatSubmit();
    return true;
  }
  if (name.equals("Mode Shape")) {
    chatSetMode("shape");
    return true;
  }
  if (name.equals("Mode Animate")) {
    chatSetMode("animate");
    return true;
  }
  if (name.equals("Critic Toggle")) {
    chatSetCritic(!chatCriticOn);
    chatAdd("status", chatCriticOn
      ? "Visual Critic is ON - new shapes get checked and revised once if needed (a bit slower)."
      : "Visual Critic is OFF - shapes come back faster, checked by fit quality only.");
    return true;
  }
  if (name.equals("New Chat")) {
    if (chatBusy) {
      chatAdd("status", "Wait for the current reply to finish, then start a new chat.");
      return true;
    }
    synchronized (chatLog) {
      chatLog.clear();
    }
    agentsResetMemory();
    chatScroll = 0;
    chatAdd("claude", "New chat started - the arm keeps its current pose. What should we make?");
    return true;
  }
  return false;
}

void chatAdd(String who, String text) {
  synchronized (chatLog) {
    chatLog.add(new ChatEntry(who, text));
  }
  chatScroll = 0; // jump to the newest message
}

// ---------------------------------------------------------------------
//  Drawing (called from draw(), inside cam.beginHUD())
// ---------------------------------------------------------------------

// Name tag shown at the top of each agent's bubble ("" = no tag).
String chatTagFor(String who) {
  if (who.equals("claude")) return "Designer";
  if (who.equals("motion")) return "Motion";
  if (who.equals("critic")) return "Critic";
  return "";
}

color chatTagColor(String who) {
  if (who.equals("motion")) return THEME_PINK;
  if (who.equals("critic")) return THEME_YELLOW;
  return THEME_TEAL;
}

void drawChatPanel() {
  pushStyle();

  // Panel background + header
  fill(THEME_PANEL);
  stroke(THEME_PANEL_EDGE);
  strokeWeight(max(1, uiScale));
  rect(uiX(CHAT_X), uiY(CHAT_Y), CHAT_W * uiScale, CHAT_H * uiScale, 10 * uiScale);
  noStroke();
  // header: accent dot + bold title + colored rule
  fill(THEME_TEAL);
  ellipse(uiX(CHAT_X + CHAT_PAD + 9), uiY(CHAT_Y + CHAT_HEADER_H / 2), 10 * uiScale, 10 * uiScale);
  fill(THEME_TEXT);
  textAlign(LEFT, CENTER);
  textFont(uiBoldFont(max(9, round(20 * uiScale))));
  hudText("Shape Chat", CHAT_X + CHAT_PAD + 22, CHAT_Y + CHAT_HEADER_H / 2);
  textFont(hudFont);
  fill(THEME_TEAL);
  rect(uiX(CHAT_X + CHAT_PAD), uiY(CHAT_Y + CHAT_HEADER_H), (CHAT_W - 2 * CHAT_PAD) * uiScale, max(2, 2 * uiScale));
  fill(THEME_BLUE);
  rect(uiX(CHAT_X + CHAT_PAD + (CHAT_W - 2 * CHAT_PAD) * 0.5), uiY(CHAT_Y + CHAT_HEADER_H), (CHAT_W - 2 * CHAT_PAD) * 0.5 * uiScale, max(2, 2 * uiScale));

  float inputY = CHAT_Y + CHAT_H - CHAT_PAD - CHAT_INPUT_H;
  float modeY = inputY - CHAT_MODE_H - 8;
  float areaTop = CHAT_Y + CHAT_HEADER_H + 6;
  float areaBottom = modeY - 28; // room for the status line
  float areaH = areaBottom - areaTop;
  float bubbleMaxW = CHAT_W - 2 * CHAT_PAD - 40;          // logical px
  float lineH = CHAT_TEXT_SIZE * 1.32;                     // logical px
  float tagSize = CHAT_TEXT_SIZE - 2;

  ArrayList<ChatEntry> entries;
  synchronized (chatLog) {
    entries = new ArrayList<ChatEntry>(chatLog);
  }

  // Wrap every message (text is measured in physical pixels at the current scale).
  hudTextSize(CHAT_TEXT_SIZE);
  float maxTextPx = (bubbleMaxW - 2 * CHAT_BUBBLE_PAD) * uiScale;
  ArrayList<ArrayList<String>> wrapped = new ArrayList<ArrayList<String>>();
  float totalH = 0;
  for (ChatEntry e : entries) {
    ArrayList<String> lines = chatWrap(e.text, maxTextPx);
    wrapped.add(lines);
    int tagLines = chatTagFor(e.who).length() > 0 ? 1 : 0;
    totalH += (lines.size() + tagLines) * lineH + 2 * CHAT_BUBBLE_PAD + CHAT_BUBBLE_GAP;
  }
  chatMaxScroll = max(0, totalH - areaH);
  chatScroll = constrain(chatScroll, 0, chatMaxScroll);

  // Newest at the bottom; scrolling moves everything down.
  float y = areaBottom - totalH + chatScroll;
  clip(uiX(CHAT_X), uiY(areaTop), CHAT_W * uiScale, areaH * uiScale);
  for (int i = 0; i < entries.size(); i++) {
    ChatEntry e = entries.get(i);
    ArrayList<String> lines = wrapped.get(i);
    String tag = chatTagFor(e.who);
    int tagLines = tag.length() > 0 ? 1 : 0;
    float h = (lines.size() + tagLines) * lineH + 2 * CHAT_BUBBLE_PAD;
    if (y + h >= areaTop && y <= areaBottom) {
      hudTextSize(CHAT_TEXT_SIZE);
      float widest = 0;
      for (String ln : lines) widest = max(widest, textWidth(ln) / uiScale);
      if (tagLines > 0) {
        textFont(uiBoldFont(max(8, round(tagSize * uiScale))));
        widest = max(widest, textWidth(tag) / uiScale);
        textFont(hudFont);
        hudTextSize(CHAT_TEXT_SIZE);
      }
      float w = min(bubbleMaxW, widest + 2 * CHAT_BUBBLE_PAD);
      boolean mine = e.who.equals("you");
      boolean note = e.who.equals("status") || e.who.equals("error");
      float x = mine ? CHAT_X + CHAT_W - CHAT_PAD - w : CHAT_X + CHAT_PAD;

      if (mine) fill(THEME_BLUE);
      else if (e.who.equals("error")) fill(170, 45, 75);
      else if (note) fill(22, 60, 66);
      else if (e.who.equals("motion")) fill(82, 40, 86);
      else if (e.who.equals("critic")) fill(78, 68, 26);
      else fill(48, 59, 92);
      rect(uiX(x), uiY(y), w * uiScale, h * uiScale, 9 * uiScale);

      float textTop = y + CHAT_BUBBLE_PAD;
      if (tagLines > 0) {
        fill(chatTagColor(e.who));
        textAlign(LEFT, TOP);
        textFont(uiBoldFont(max(8, round(tagSize * uiScale))));
        hudText(tag, x + CHAT_BUBBLE_PAD, textTop);
        textFont(hudFont);
        hudTextSize(CHAT_TEXT_SIZE);
        textTop += lineH;
      }

      fill(note ? color(150, 245, 225) : color(255));
      textAlign(LEFT, TOP);
      for (int k = 0; k < lines.size(); k++) {
        hudText(lines.get(k), x + CHAT_BUBBLE_PAD, textTop + k * lineH);
      }
    }
    y += h + CHAT_BUBBLE_GAP;
  }
  noClip();

  // Status line just above the mode switch
  String status = chatStatus;
  if (chatBusy && status.length() > 0) {
    int dots = (millis() / 400) % 4;
    String d = "";
    for (int k = 0; k < dots; k++) d += ".";
    fill(THEME_YELLOW);
    textAlign(LEFT, CENTER);
    hudTextSize(15);
    hudText(status + d, CHAT_X + CHAT_PAD + 4, areaBottom + 14);
  } else if (chatMaxScroll > 0 && chatScroll > 0) {
    fill(150);
    textAlign(LEFT, CENTER);
    hudTextSize(13);
    hudText("(scroll down for newer messages)", CHAT_X + CHAT_PAD + 4, areaBottom + 14);
  }

  // Mode switch label + which agent answers in that mode
  fill(THEME_TEXT);
  textAlign(LEFT, CENTER);
  hudTextSize(15);
  hudText("Mode:", CHAT_X + CHAT_PAD + 4, modeY + CHAT_MODE_H / 2);
  fill(chatMode.equals("animate") ? THEME_PINK : THEME_BLUE);
  hudTextSize(13);
  hudText(chatMode.equals("animate") ? "Motion agent" : "Designer",
    CHAT_X + CHAT_PAD + 52 + 2 * CHAT_MODE_W + 16, modeY + CHAT_MODE_H / 2);

  popStyle();
}

// Word-wraps text to a pixel width at the current text size. Keeps the
// message's own line breaks and splits words that are too long by themselves.
ArrayList<String> chatWrap(String text, float maxPx) {
  ArrayList<String> out = new ArrayList<String>();
  String[] paragraphs = text.split("\n", -1);
  for (String para : paragraphs) {
    String[] words = para.split(" ");
    String line = "";
    for (String w : words) {
      String candidate = line.length() == 0 ? w : line + " " + w;
      if (textWidth(candidate) <= maxPx) {
        line = candidate;
        continue;
      }
      if (line.length() > 0) out.add(line);
      // a single word wider than the bubble: hard-split it
      while (textWidth(w) > maxPx && w.length() > 1) {
        int cut = w.length() - 1;
        while (cut > 1 && textWidth(w.substring(0, cut)) > maxPx) cut--;
        out.add(w.substring(0, cut));
        w = w.substring(cut);
      }
      line = w;
    }
    out.add(line);
  }
  return out;
}

// ---------------------------------------------------------------------
//  Sending a message
// ---------------------------------------------------------------------

void chatSubmit() {
  if (chatInput == null) return;
  String msg = chatInput.getText();
  if (msg == null || msg.trim().length() == 0) return;
  if (chatBusy) {
    chatAdd("status", "Still working on your last message - one moment.");
    return;
  }
  if (ANTHROPIC_API_KEY == null || ANTHROPIC_API_KEY.trim().length() == 0) {
    chatAdd("error", "No Anthropic API key found. Put your key in a file named "
      + "anthropic_key.txt in this sketch's folder and restart the sketch.");
    return;
  }
  chatInput.clear();
  chatAdd("you", msg.trim());
  chatQueuedMessage = msg.trim();
  chatQueuedMode = chatMode;
  chatStatus = chatMode.equals("animate") ? "Motion agent is thinking" : "Designer is thinking";
  chatBusy = true;
  thread("chatWorker");
}

// Runs on its own thread (started by chatSubmit()).
void chatWorker() {
  String text = chatQueuedMessage;
  String mode = chatQueuedMode;
  try {
    if (mode.equals("animate")) motionRunTurn(text);
    else designerRunTurn(text);
  }
  catch (Exception e) {
    chatAdd("error", "Something went wrong: " + e.getMessage());
    aiLog("[chat] error: " + e);
  }
  finally {
    chatStatus = "";
    chatBusy = false;
  }
}

// ---------------------------------------------------------------------
//  Running arm changes on the drawing thread
// ---------------------------------------------------------------------

// Called by the chat thread: queues `job` for the drawing thread and waits
// until it has run.
void chatRunOnMain(final Runnable job) {
  final java.util.concurrent.CountDownLatch done = new java.util.concurrent.CountDownLatch(1);
  final Throwable[] failure = new Throwable[1];
  chatMainJobs.add(new Runnable() {
    public void run() {
      try {
        job.run();
      }
      catch (Throwable t) {
        failure[0] = t;
      }
      finally {
        done.countDown();
      }
    }
  });
  try {
    done.await();
  }
  catch (InterruptedException ie) {
    return;
  }
  if (failure[0] != null) throw new RuntimeException(failure[0]);
}

// Called at the top of draw().
void chatRunMainJobs() {
  Runnable job;
  while ((job = chatMainJobs.poll()) != null) job.run();
}
