// =====================================================================
//  VOICE INPUT - hold-to-talk mic button between the chat box and Send.
//
//  Hold the mic button and speak; release to stop. The recording is sent to
//  OpenAI's speech-to-text API and the words are put into the chat box
//  (added after anything already typed) so you can check / fix them before
//  pressing Send. Nothing is sent to the agents automatically.
//
//  Claude's API has no speech-to-text, so this needs an OpenAI key:
//  OPENAI_API_KEY environment variable, or the only line of openai_key.txt
//  in this sketch's folder (git-ignored).
//
//  local_config.json (optional):
//    "voice_model":    "gpt-4o-mini-transcribe" (default) | "gpt-4o-transcribe" | "whisper-1"
//    "voice_language": "en" (default), "zh", ... or "" = auto-detect
//
//  Audio: 16 kHz, 16-bit mono WAV, recorded with javax.sound (no libraries).
// =====================================================================

final float CHAT_MIC_W = 48;                 // logical px, mic button width
final int VOICE_MAX_MS = 120000;             // recording auto-stops after 2 min
final int VOICE_MIN_MS = 400;                // shorter holds are ignored
final String VOICE_API_URL = "https://api.openai.com/v1/audio/transcriptions";

String OPENAI_API_KEY = null;
String VOICE_MODEL = "gpt-4o-mini-transcribe";
String VOICE_LANGUAGE = "en";

// 0 = idle, 1 = recording, 2 = transcribing
final int VOICE_IDLE = 0, VOICE_REC = 1, VOICE_BUSY = 2;
volatile int voiceState = VOICE_IDLE;
volatile float voiceLevel = 0;     // 0..1, live mic loudness for the meter
volatile float voicePeak = 0;      // loudest sample of this recording, 0..1
int voiceStartMs = 0;

javax.sound.sampled.TargetDataLine voiceLine = null;
java.io.ByteArrayOutputStream voiceBuffer = null;
Thread voiceReader = null;
final javax.sound.sampled.AudioFormat VOICE_FORMAT =
  new javax.sound.sampled.AudioFormat(16000f, 16, 1, true, false);

// Words that help the recognizer spell this project's terms right.
final String VOICE_HINT = "Lineform arm. Designer, Motion agent, Critic. Shape, animate, "
  + "torque, stiffness, joint, segment, bend, curl, wave, slither, hook, spiral, zigzag, "
  + "limp, softer, stiffer, Create Shape.";

// ---------------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------------

// Called from setup() after setupChatPanel().
void setupVoice() {
  OPENAI_API_KEY = loadOpenAiKey();
  File f = new File(sketchPath("local_config.json"));
  if (f.exists()) {
    try {
      JSONObject c = loadJSONObject(f.getAbsolutePath());
      if (c.hasKey("voice_model")) VOICE_MODEL = c.getString("voice_model").trim();
      if (c.hasKey("voice_language")) VOICE_LANGUAGE = c.getString("voice_language").trim();
    } catch (Exception e) { /* defaults */ }
  }
  println("Voice input: " + (OPENAI_API_KEY == null ? "OFF (no OpenAI key - put it in openai_key.txt)"
    : "model " + VOICE_MODEL + ", language " + (VOICE_LANGUAGE.length() == 0 ? "auto" : VOICE_LANGUAGE)));
}

String loadOpenAiKey() {
  String envKey = System.getenv("OPENAI_API_KEY");
  if (envKey != null && envKey.trim().length() > 0) return envKey.trim();
  File keyFile = new File(sketchPath("openai_key.txt"));
  if (keyFile.exists()) {
    String[] lines = loadStrings(keyFile.getAbsolutePath());
    if (lines != null && lines.length > 0 && lines[0].trim().length() > 0) return lines[0].trim();
  }
  return null;
}

// ---------------------------------------------------------------------
//  Button geometry, mouse
// ---------------------------------------------------------------------

float voiceBtnX() { return CHAT_X + CHAT_W - CHAT_PAD - CHAT_SEND_W - CHAT_PAD - CHAT_MIC_W; }
float voiceBtnY() { return CHAT_Y + CHAT_H - CHAT_PAD - CHAT_INPUT_H; }

boolean voiceMouseOver() {
  if (manipulationMode) return false;
  return mouseX >= uiX(voiceBtnX()) && mouseX <= uiX(voiceBtnX() + CHAT_MIC_W)
      && mouseY >= uiY(voiceBtnY()) && mouseY <= uiY(voiceBtnY() + CHAT_INPUT_H);
}

// From mousePressed(). Returns true if the press was on the mic button.
boolean voiceMousePressed() {
  if (!voiceMouseOver()) return false;
  if (voiceState == VOICE_BUSY) {
    noticeAdd("status", "Still transcribing the last recording - one moment.");
    return true;
  }
  if (voiceState == VOICE_IDLE) voiceStart();
  return true;
}

// From mouseReleased(). Releasing anywhere stops the recording.
void voiceMouseReleased() {
  if (voiceState == VOICE_REC) voiceStop();
}

// From draw(), every frame: safety stop for very long holds (or a release
// the window never saw, e.g. focus lost while holding).
void voiceTick() {
  if (voiceState == VOICE_REC && millis() - voiceStartMs > VOICE_MAX_MS) {
    noticeAdd("status", "Voice: recording stopped after " + (VOICE_MAX_MS / 1000) + " s.");
    voiceStop();
  } else if (voiceState == VOICE_REC && !mousePressed && millis() - voiceStartMs > 300) {
    voiceStop(); // the mouse is already up - we missed the release
  }
}

// ---------------------------------------------------------------------
//  Recording
// ---------------------------------------------------------------------

void voiceStart() {
  if (OPENAI_API_KEY == null) {
    noticeAdd("error", "Voice input needs an OpenAI API key (Claude's API can't transcribe audio). "
      + "Put the key as the only line of openai_key.txt in the sketch folder and restart.");
    return;
  }
  try {
    javax.sound.sampled.DataLine.Info info =
      new javax.sound.sampled.DataLine.Info(javax.sound.sampled.TargetDataLine.class, VOICE_FORMAT);
    if (!javax.sound.sampled.AudioSystem.isLineSupported(info)) {
      noticeAdd("error", "Voice: no microphone found (or it doesn't support 16 kHz mono).");
      return;
    }
    voiceLine = (javax.sound.sampled.TargetDataLine) javax.sound.sampled.AudioSystem.getLine(info);
    voiceLine.open(VOICE_FORMAT);
    voiceLine.start();
  } catch (Exception e) {
    noticeAdd("error", "Voice: couldn't open the microphone (" + e.getMessage() + ").");
    aiLog("[voice] open failed: " + e);
    voiceLine = null;
    return;
  }
  voiceBuffer = new java.io.ByteArrayOutputStream();
  voiceLevel = 0;
  voicePeak = 0;
  voiceStartMs = millis();
  voiceState = VOICE_REC;
  final javax.sound.sampled.TargetDataLine line = voiceLine;
  final java.io.ByteArrayOutputStream buf = voiceBuffer;
  voiceReader = new Thread(new Runnable() {
    public void run() {
      byte[] chunk = new byte[1600]; // 50 ms
      while (true) {
        int n = line.read(chunk, 0, chunk.length);
        if (n <= 0) break; // line stopped / closed
        buf.write(chunk, 0, n);
        double sum = 0;
        int peak = 0;
        for (int i = 0; i + 1 < n; i += 2) {
          int s = (short) ((chunk[i] & 0xff) | (chunk[i + 1] << 8));
          sum += (double) s * s;
          peak = max(peak, abs(s));
        }
        float rms = (float) Math.sqrt(sum / max(1, n / 2)) / 32768f;
        voiceLevel = constrain(rms * 6, 0, 1);
        voicePeak = max(voicePeak, peak / 32768f);
      }
    }
  });
  voiceReader.setDaemon(true);
  voiceReader.start();
}

void voiceStop() {
  if (voiceState != VOICE_REC || voiceLine == null) return;
  int heldMs = millis() - voiceStartMs;
  try {
    voiceLine.stop();
    voiceLine.flush();
    voiceLine.close();
  } catch (Exception e) { /* ignore */ }
  try { if (voiceReader != null) voiceReader.join(500); } catch (InterruptedException ie) { }
  voiceLine = null;
  voiceReader = null;
  voiceLevel = 0;

  if (heldMs < VOICE_MIN_MS) {
    voiceState = VOICE_IDLE;
    noticeAdd("status", "Voice: hold the mic button while you speak, then release.");
    return;
  }
  if (voicePeak < 0.002) {
    voiceState = VOICE_IDLE;
    noticeAdd("error", "Voice: the microphone gave only silence. Check Windows Settings > Privacy & "
      + "security > Microphone > \"Let desktop apps access your microphone\", and the input device.");
    return;
  }
  final byte[] wav = voiceWav(voiceBuffer.toByteArray());
  voiceBuffer = null;
  voiceState = VOICE_BUSY;
  new Thread(new Runnable() {
    public void run() { voiceTranscribe(wav); }
  }).start();
}

// Raw 16-bit PCM -> a WAV file in memory.
byte[] voiceWav(byte[] pcm) {
  int rate = (int) VOICE_FORMAT.getSampleRate();
  java.nio.ByteBuffer b = java.nio.ByteBuffer.allocate(44 + pcm.length).order(java.nio.ByteOrder.LITTLE_ENDIAN);
  b.put("RIFF".getBytes()).putInt(36 + pcm.length).put("WAVE".getBytes());
  b.put("fmt ".getBytes()).putInt(16).putShort((short) 1).putShort((short) 1)
    .putInt(rate).putInt(rate * 2).putShort((short) 2).putShort((short) 16);
  b.put("data".getBytes()).putInt(pcm.length).put(pcm);
  return b.array();
}

// ---------------------------------------------------------------------
//  Transcription (background thread)
// ---------------------------------------------------------------------

void voiceTranscribe(byte[] wav) {
  try {
    String boundary = "----lineform" + System.currentTimeMillis();
    ByteArrayOutputStream body = new ByteArrayOutputStream();
    voiceField(body, boundary, "model", VOICE_MODEL);
    if (VOICE_LANGUAGE.length() > 0) voiceField(body, boundary, "language", VOICE_LANGUAGE);
    voiceField(body, boundary, "prompt", VOICE_HINT);
    body.write(("--" + boundary + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"voice.wav\"\r\n"
      + "Content-Type: audio/wav\r\n\r\n").getBytes("UTF-8"));
    body.write(wav);
    body.write(("\r\n--" + boundary + "--\r\n").getBytes("UTF-8"));

    HttpURLConnection conn = (HttpURLConnection) new URL(VOICE_API_URL).openConnection();
    conn.setRequestMethod("POST");
    conn.setConnectTimeout(15000);
    conn.setReadTimeout(60000);
    conn.setRequestProperty("Authorization", "Bearer " + OPENAI_API_KEY);
    conn.setRequestProperty("Content-Type", "multipart/form-data; boundary=" + boundary);
    conn.setDoOutput(true);
    conn.getOutputStream().write(body.toByteArray());
    int code = conn.getResponseCode();
    InputStream stream = (code >= 200 && code < 300) ? conn.getInputStream() : conn.getErrorStream();
    BufferedReader reader = new BufferedReader(new InputStreamReader(stream, "UTF-8"));
    StringBuilder sb = new StringBuilder();
    String line;
    while ((line = reader.readLine()) != null) sb.append(line).append('\n');
    reader.close();
    String reply = sb.toString();
    aiLog("[voice] " + (wav.length / 32000.0) + " s audio -> HTTP " + code + ": " + reply.trim());

    JSONObject j = JSONObject.parse(reply);
    if (code < 200 || code >= 300 || j == null || !j.hasKey("text")) {
      String msg = reply.trim();
      if (j != null && j.hasKey("error")) {
        JSONObject err = j.getJSONObject("error");
        if (err != null && err.hasKey("message")) msg = err.getString("message");
      }
      noticeAdd("error", "Voice: transcription failed (HTTP " + code + "): " + msg);
      return;
    }
    final String text = j.getString("text").trim();
    if (text.length() == 0) {
      noticeAdd("status", "Voice: no words heard - try again a bit closer to the mic.");
      return;
    }
    chatMainJobs.add(new Runnable() {
      public void run() { voiceInsert(text); }
    });
  } catch (Exception e) {
    noticeAdd("error", "Voice: transcription failed (" + e.getMessage() + ").");
    aiLog("[voice] error: " + e);
  } finally {
    voiceState = VOICE_IDLE;
  }
}

void voiceField(ByteArrayOutputStream out, String boundary, String name, String value) throws IOException {
  out.write(("--" + boundary + "\r\nContent-Disposition: form-data; name=\"" + name + "\"\r\n\r\n"
    + value + "\r\n").getBytes("UTF-8"));
}

// Drawing thread: add the words after whatever is already in the chat box.
void voiceInsert(String text) {
  if (chatInput == null) return;
  String cur = chatInput.getText();
  if (cur == null) cur = "";
  String joined = cur.trim().length() == 0 ? text : cur.replaceAll("\\s+$", "") + " " + text;
  chatInput.setText(joined);
  chatInput.setFocus(true); // Enter sends it
}

// ---------------------------------------------------------------------
//  Drawing (inside drawChatPanel, HUD coordinates)
// ---------------------------------------------------------------------

void drawVoiceButton() {
  float x = voiceBtnX(), y = voiceBtnY(), w = CHAT_MIC_W, h = CHAT_INPUT_H;
  boolean hover = voiceMouseOver();
  pushStyle();
  noStroke();
  if (voiceState == VOICE_REC) {
    float pulse = 0.5 + 0.5 * sin(millis() / 120.0);
    fill(lerpColor(color(200, 50, 60), THEME_RED, pulse));
  } else if (voiceState == VOICE_BUSY) {
    fill(THEME_YELLOW);
  } else {
    fill(hover ? THEME_CTRL_HOVER : THEME_CTRL_BG);
  }
  rect(uiX(x), uiY(y), w * uiScale, h * uiScale, 6 * uiScale);

  // mic icon: capsule, cradle arc, stem, base
  color ink = voiceState == VOICE_BUSY ? color(30, 30, 40)
            : (OPENAI_API_KEY == null ? THEME_MUTED : color(255));
  float cx = uiX(x + w / 2), cy = uiY(y + h / 2);
  float s = uiScale;
  fill(ink);
  rect(cx - 5 * s, cy - 13 * s, 10 * s, 17 * s, 5 * s);
  noFill();
  stroke(ink);
  strokeWeight(max(1, 2 * s));
  arc(cx, cy - 2 * s, 18 * s, 18 * s, 0, PI);
  line(cx, cy + 7 * s, cx, cy + 12 * s);
  line(cx - 5 * s, cy + 12 * s, cx + 5 * s, cy + 12 * s);
  popStyle();
}

// Status line while recording / transcribing. Returns true if it drew
// something (then the chat's own status line is skipped this frame).
boolean drawVoiceStatus(float x, float y) {
  if (voiceState == VOICE_IDLE) return false;
  pushStyle();
  textAlign(LEFT, CENTER);
  hudTextSize(15);
  if (voiceState == VOICE_REC) {
    int secs = (millis() - voiceStartMs) / 1000;
    String t = "Listening " + secs / 60 + ":" + nf(secs % 60, 2) + " - release to stop";
    fill(THEME_RED);
    hudText(t, x, y);
    // level meter after the text
    float tx = x + textWidth(t) / uiScale + 10;
    float mw = 90;
    noStroke();
    fill(THEME_CTRL_BG);
    rect(uiX(tx), uiY(y - 4), mw * uiScale, 8 * uiScale, 4 * uiScale);
    fill(THEME_GREEN);
    rect(uiX(tx), uiY(y - 4), mw * voiceLevel * uiScale, 8 * uiScale, 4 * uiScale);
  } else {
    int dots = (millis() / 400) % 4;
    String d = "";
    for (int k = 0; k < dots; k++) d += ".";
    fill(THEME_YELLOW);
    hudText("Transcribing" + d, x, y);
  }
  popStyle();
  return true;
}
