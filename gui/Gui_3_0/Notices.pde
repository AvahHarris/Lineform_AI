// =====================================================================
//  NOTICES - every motor / mode warning and status message, in ONE panel
//  in the bottom-left corner (2026-11-18). Nothing robot-related goes into
//  the AI chat any more: the chat only holds the conversation with the
//  agents.
//
//  * noticeAdd(kind, text) - "error" (red) or "status" (neutral). Safe to
//    call from any thread. Also printed to the console.
//  * The overload warning (when the overload trip is enabled and fires)
//    is PINNED at the top of the panel until you switch back to AI mode.
//  * Status notices fade out after NOTICE_STATUS_MS, errors after
//    NOTICE_ERROR_MS. Click the panel to clear them.
//  * Only the newest NOTICE_MAX are kept; the panel grows upward from the
//    bottom-left corner and never covers the info box above it.
// =====================================================================

class Notice {
  String kind, text;
  long ms;
  Notice(String kind, String text) { this.kind = kind; this.text = text; this.ms = millis(); }
}

ArrayList<Notice> notices = new ArrayList<Notice>();
final int NOTICE_MAX = 4;
final int NOTICE_STATUS_MS = 15000;
final int NOTICE_ERROR_MS = 90000;
final float NOTICE_X = 8;
final float NOTICE_W = 540;     // ends left of the AI / manipulation mode switch (x 560)
final float NOTICE_BOTTOM = 862;
final float NOTICE_TEXT = 13;
// Where the panel was last drawn (logical coords), for clicks.
float noticeDrawnTop = -1, noticeDrawnBottom = -1;

void noticeAdd(String kind, String text) {
  if (text == null || text.length() == 0) return;
  synchronized (notices) {
    notices.add(new Notice(kind, text));
    while (notices.size() > NOTICE_MAX) notices.remove(0);
  }
}

void noticeClear() {
  synchronized (notices) { notices.clear(); }
}

boolean noticeMouseOver() {
  if (noticeDrawnTop < 0) return false;
  return mouseX >= uiX(NOTICE_X) && mouseX <= uiX(NOTICE_X + NOTICE_W)
      && mouseY >= uiY(noticeDrawnTop) && mouseY <= uiY(noticeDrawnBottom);
}

// Called from mousePressed(). Click = clear (the pinned overload warning stays).
boolean noticeMousePressed() {
  if (!noticeMouseOver()) return false;
  noticeClear();
  return true;
}

// Draws the panel. topLimit = the lowest point anything above it uses
// (logical y), so the panel never overlaps the info box.
void drawNoticePanel(float topLimit) {
  noticeDrawnTop = noticeDrawnBottom = -1;
  long now = millis();
  ArrayList<Notice> list = new ArrayList<Notice>();
  synchronized (notices) {
    for (int k = notices.size() - 1; k >= 0; k--) {
      Notice n = notices.get(k);
      long life = n.kind.equals("error") ? NOTICE_ERROR_MS : NOTICE_STATUS_MS;
      if (now - n.ms > life) notices.remove(k);
    }
    list.addAll(notices);
  }
  String pinned = overloadWarning;
  if (pinned != null) list.add(new Notice("pinned", "!! " + pinned
    + " Check what the arm is pushing against before switching back to AI mode."));
  if (list.isEmpty()) return;

  pushStyle();
  textFont(hudFont);
  hudTextSize(NOTICE_TEXT);
  textAlign(LEFT, TOP);
  float pad = 8, gap = 4, lineH = NOTICE_TEXT * 1.35, headH = 16;
  float maxPx = (NOTICE_W - 2 * pad - 8) * uiScale;

  // Newest first; pinned always on top. Drop the oldest if it doesn't fit.
  ArrayList<ArrayList<String>> wrapped = new ArrayList<ArrayList<String>>();
  ArrayList<Notice> shown = new ArrayList<Notice>();
  float avail = NOTICE_BOTTOM - max(topLimit, 0) - 2 * pad - headH;
  float used = 0;
  ArrayList<Notice> order = new ArrayList<Notice>();
  if (pinned != null) order.add(list.get(list.size() - 1));
  for (int k = list.size() - 1 - (pinned != null ? 1 : 0); k >= 0; k--) order.add(list.get(k));
  for (Notice n : order) {
    ArrayList<String> lines = chatWrap(n.text, maxPx);
    float h = lines.size() * lineH + 2 * 3 + gap;
    if (used + h > avail && !shown.isEmpty()) break;
    if (used + h > avail) { // first one alone too tall: cut it
      int fit = max(1, floor((avail - 6 - gap) / lineH));
      while (lines.size() > fit) lines.remove(lines.size() - 1);
      h = lines.size() * lineH + 6 + gap;
    }
    shown.add(n);
    wrapped.add(lines);
    used += h;
  }

  float h = headH + used + 2 * pad - gap;
  float y = NOTICE_BOTTOM - h;
  themePanel(NOTICE_X, y, NOTICE_W, h);
  noticeDrawnTop = y;
  noticeDrawnBottom = NOTICE_BOTTOM;

  textFont(uiBoldFont(max(7, round(11 * uiScale))));
  fill(THEME_MUTED);
  hudText("WARNINGS & STATUS" + (notices.isEmpty() ? "" : "   (click to clear)"), NOTICE_X + pad, y + pad - 2);
  textFont(hudFont);
  hudTextSize(NOTICE_TEXT);

  float cy = y + pad + headH;
  for (int k = 0; k < shown.size(); k++) {
    Notice n = shown.get(k);
    ArrayList<String> lines = wrapped.get(k);
    float bh = lines.size() * lineH + 6;
    boolean err = !n.kind.equals("status");
    noStroke();
    if (n.kind.equals("pinned")) fill(170, 30, 40, 235);
    else if (err) fill(120, 30, 40, 200);
    else fill(255, 255, 255, 18);
    rect(uiX(NOTICE_X + pad), uiY(cy), (NOTICE_W - 2 * pad) * uiScale, bh * uiScale, 5 * uiScale);
    fill(err ? THEME_RED : THEME_MUTED);
    rect(uiX(NOTICE_X + pad), uiY(cy), 3 * uiScale, bh * uiScale);
    fill(n.kind.equals("pinned") ? color(255) : (err ? color(255, 205, 205) : THEME_TEXT));
    for (int j = 0; j < lines.size(); j++) hudText(lines.get(j), NOTICE_X + pad + 8, cy + 3 + j * lineH);
    cy += bh + gap;
  }
  popStyle();
}
