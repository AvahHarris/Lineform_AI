// =====================================================================
//  AGENTS - the three Claude roles behind the Shape Chat.
//
//  DESIGNER (Shape mode)
//    Own conversation memory. Only tool: emit_shape_segments. Its shape is
//    fitted to the arm by ordinary code (no AI), which also measures
//    "fit quality".
//
//  VISUAL CRITIC (after each new still shape, when "Critic: ON")
//    No memory and never sees the Designer's plan. It gets ONE picture of
//    the line the arm actually makes (front / top / side views) plus what
//    the user asked for, and answers with submit_critique. If it says the
//    shape doesn't read right (or the fit is poor), its notes go back to
//    the Designer for ONE revision, the Critic checks that too, and the
//    better-judged version stays on the arm.
//
//  MOTION (Animate mode)
//    Own conversation memory. Only tool: emit_shape_animation. Every
//    message includes the Designer's current shape, so animations are
//    built around it.
//
//  API calls per message:
//    Shape mode, Critic ON  : 2 (Designer + Critic), up to 4 if a revision is needed
//    Shape mode, Critic OFF : 1, or 2 if the fit is poor
//    Animate mode           : 1
// =====================================================================

// Critic score (1-10) a shape needs to pass without a revision.
final int CRITIC_PASS_SCORE = 7;
// Extra Designer rounds when the Critic is OFF and a shape fits badly.
final int CHAT_AUTO_REFINE_ROUNDS = 1;

// Designer memory (Shape mode)
JSONArray chatHistory = new JSONArray();
ArrayList<JSONObject> chatPendingToolResults = new ArrayList<JSONObject>();
ArrayList<String> designerRequests = new ArrayList<String>(); // what the user asked, for the Critic

// Motion memory (Animate mode)
JSONArray motionHistory = new JSONArray();
ArrayList<JSONObject> motionPendingToolResults = new ArrayList<JSONObject>();

// The Designer's current shape - handed to the Motion agent.
String agentShapeName = "";
boolean agentShapeClosed = false;
JSONArray agentShapeSegments = null;   // null = none yet, or a calibrated preset
int[] agentShapeTorque = null;         // the Designer's torque plan for the shape (null = none)

void agentsResetMemory() {
  chatHistory = new JSONArray();
  chatPendingToolResults.clear();
  designerRequests.clear();
  motionHistory = new JSONArray();
  motionPendingToolResults.clear();
  // the shape stays on the arm, so the Motion agent can still use it
}

// =====================================================================
//  DESIGNER
// =====================================================================

String DESIGNER_SYSTEM_HEAD =
  "You are the DESIGNER agent inside a chat panel that drives a physical "
  + "robotic arm made of 11 hinged links (Dynamixel servos). The user talks to "
  + "you over several messages to design a STILL SHAPE and refine it step by "
  + "step. Your one tool is emit_shape_segments. The full technical guidance "
  + "follows.\n\n=== SHAPE GUIDANCE ===\n\n";

String DESIGNER_SYSTEM_TAIL =
  "\n\n=== HOW TO WORK IN THIS CHAT ===\n"
  + "- You work with two other agents. A VISUAL CRITIC looks at a picture of the "
  + "line the real arm actually made (it never sees your segments) and its notes "
  + "may come back to you in a tool result - when they do, fix what it points "
  + "out while keeping what works. A MOTION agent handles animation when the user "
  + "switches the chat to Animate mode; if the user asks for motion while in "
  + "Shape mode, briefly tell them to switch the mode to Animate.\n"
  + "- Every message continues the same design session. When the user asks for "
  + "a change ('make the head bigger', 'curl the tip the other way'), start from "
  + "YOUR MOST RECENT shape in this conversation and change only what they asked "
  + "for. Always send the COMPLETE updated shape, never just the changed part.\n"
  + "- Each tool result tells you what is now on the arm and how well it fit "
  + "(fit quality: lower is better, under 0.70 is good). Each user message ends "
  + "with a bracketed note about the arm's current state; use it, but don't "
  + "repeat it back.\n"
  + "- If a request is truly ambiguous, ask ONE short question - but usually "
  + "make a reasonable attempt the user can react to.\n"
  + "- If the arm-state note lists torque limits, joints at 0% are limp and can't hold "
  + "a bend, and weak joints may sag - prefer shapes that bend at the stronger joints.\n"
  + "- You can set joint_torque (11 values, 0-100, base to tip) when the object has parts "
  + "that should be soft or loose: e.g. a NUNCHUCK is two rigid sticks (100) joined by a "
  + "loose chain (the joints in between at 0 = limp, free-swinging); a rope or tail end can "
  + "be low. The MOTION agent receives your torque plan with the shape and builds "
  + "animations around it. Leave joint_torque out to keep the user's current torque.\n"
  + "- You may think briefly before a tool call, but the user does NOT see that "
  + "text. Put what the user should read in the tool's message_to_user field: "
  + "1-2 short, plain sentences about what you made or changed (no segment "
  + "jargon). When you reply without a tool, keep it to a few sentences.";

JSONObject designerBuildRequestBody(JSONArray turn) {
  JSONObject body = new JSONObject();
  body.setString("model", ANTHROPIC_MODEL);
  body.setInt("max_tokens", 4096);

  JSONArray system = new JSONArray();
  JSONObject sys1 = chatTextBlock(DESIGNER_SYSTEM_HEAD + AI_SYSTEM_PROMPT + DESIGNER_SYSTEM_TAIL);
  sys1.setJSONObject("cache_control", aiCacheControlEphemeral());
  system.append(sys1);
  String lib = chatLibraryNote();
  if (lib.length() > 0) system.append(chatTextBlock(lib));
  body.setJSONArray("system", system);

  JSONObject shapeTool = aiBuildToolDefinition(); // keeps its cache marker
  chatAddMessageField(shapeTool);
  chatAddTorqueField(shapeTool,
    "OPTIONAL. Torque for each joint while holding this shape, 11 integers 0-100, base to "
    + "tip. 100 = holds rigidly, lower = softer/compliant, 0 = LIMP free hinge that holds no "
    + "angle (e.g. the chain between the two sticks of a nunchuck). Leave out to keep the "
    + "user's current torque.");
  JSONArray tools = new JSONArray();
  tools.append(shapeTool);
  body.setJSONArray("tools", tools);
  body.setJSONObject("tool_choice", chatAutoToolChoice());

  JSONArray messages = new JSONArray();
  for (int i = 0; i < chatHistory.size(); i++) messages.append(chatHistory.getJSONObject(i));
  for (int i = 0; i < turn.size(); i++) messages.append(turn.getJSONObject(i));
  body.setJSONArray("messages", messages);
  return body;
}

String chatLibraryNote() {
  if (!AI_USE_LIBRARY_MATCH || shapeLibrarySmall.isEmpty()) return "";
  return "Calibrated shapes already tuned on the real arm: " + shapeLibrarySmall.keySet()
    + ". In emit_shape_segments, set library_match to one of these names ONLY when the "
    + "user wants exactly that shape with nothing added or changed (a synonym, rephrasing "
    + "or translation). Any qualifier or requested change (bigger, tilted, with a handle, "
    + "isosceles...) means design it with segments instead. When unsure, leave it out.";
}

// One full Shape-mode turn.
void designerRunTurn(String userText) {
  aiLog("[designer] you: " + userText);
  designerRequests.add(userText);

  JSONArray content = new JSONArray();
  for (JSONObject tr : chatPendingToolResults) content.append(tr);
  content.append(chatTextBlock(userText + chatStateNote()));
  JSONArray turn = new JSONArray();
  turn.append(chatMessage("user", content));

  JSONObject resp = agentCall(designerBuildRequestBody(turn), "Designer");
  if (resp == null) return;
  turn.append(chatMessage("assistant", resp.getJSONArray("content")));

  JSONObject toolUse = chatFindToolUse(resp);
  String plainText = chatJoinText(resp);
  if (plainText.length() > 0) aiLog("[designer] " + plainText);

  // plain answer
  if (toolUse == null) {
    chatCommitTo(chatHistory, chatPendingToolResults, turn, null, null);
    chatAdd("claude", plainText.length() > 0 ? plainText : "(no reply)");
    return;
  }
  JSONObject input = toolUse.getJSONObject("input");
  if (!toolUse.getString("name").equals("emit_shape_segments")) {
    chatCommitTo(chatHistory, chatPendingToolResults, turn, toolUse, "ERROR: unknown tool.");
    chatAdd("error", "The Designer used an unknown tool.");
    return;
  }

  // calibrated preset from shape_library.json?
  if (AI_USE_LIBRARY_MATCH && input.hasKey("library_match")) {
    final String match = aiMatchKnownShape(input.getString("library_match").trim().toLowerCase());
    if (match != null) {
      final String prompt = userText;
      chatRunOnMain(new Runnable() {
        public void run() {
          aiStopAnimation();
          aiUseKnownShape(match, prompt);
        }
      });
      agentShapeName = match;
      agentShapeClosed = false;
      agentShapeSegments = null;
      agentShapeTorque = null;
      chatCommitTo(chatHistory, chatPendingToolResults, turn, toolUse, "Used the calibrated preset '"
        + match + "' from the shape library (tuned on the real arm). It is now on the arm.");
      chatAdd("claude", chatMessageForUser(input, "That's my calibrated \"" + match + "\" - loaded it."));
      return;
    }
  }

  ChatShapeTry first = chatTryShape(toolUse, 0);
  if (first == null) {
    chatCommitTo(chatHistory, chatPendingToolResults, turn, toolUse, "ERROR: no usable segments - nothing changed on the arm.");
    chatAdd("error", "The Designer's shape had no usable segments - try rephrasing.");
    return;
  }
  ChatShapeTry best = first;
  ChatShapeTry last = first;
  int revisions = 0;

  if (chatCriticOn) {
    // ---- Critic review, then at most one revision ----
    first.critique = criticReview(first);
    chatShowCritique(first.critique);
    boolean criticUnhappy = first.critique != null && (!first.critique.recognizable || first.critique.score < CRITIC_PASS_SCORE);
    boolean fitPoor = first.quality > AI_REFINE_QUALITY_THRESHOLD;
    if (criticUnhappy || fitPoor) {
      revisions++;
      chatStatus = "Designer is revising";
      String feedback = "Your shape is on the arm now."
        + (first.critique != null ? " " + first.critique.asFeedback() : "")
        + (fitPoor ? " " + aiBuildRefinementFeedback(first.quality)
                   : " Fit quality was " + nf(first.quality, 1, 2) + " (good), so focus on how it LOOKS.")
        + " Send a revised complete shape that fixes these points and keeps what already works.";
      ChatShapeTry next = designerRevise(turn, last, feedback, 1);
      if (next != null) {
        last = next;
        next.critique = criticReview(next);
        chatShowCritique(next.critique);
        if (chatBetter(next, best)) best = next;
      }
    }
  } else {
    // ---- No critic: fit-quality touch-up only ----
    while (last.quality > AI_REFINE_QUALITY_THRESHOLD && revisions < CHAT_AUTO_REFINE_ROUNDS) {
      revisions++;
      chatStatus = "Fit " + nf(last.quality, 1, 2) + " - Designer is tightening it up";
      ChatShapeTry next = designerRevise(turn, last, aiBuildRefinementFeedback(last.quality), revisions);
      if (next == null) break;
      last = next;
      if (chatBetter(next, best)) best = next;
    }
  }

  // Put the winner on the arm and close out the turn.
  chatApplyBest(best, last);
  agentShapeName = best.name;
  agentShapeClosed = best.closed;
  agentShapeSegments = aiCloneSegments(best.segments);
  final int[] torquePlan = agentsParseTorque(best.input);
  agentShapeTorque = torquePlan;
  if (torquePlan != null) {
    chatRunOnMain(new Runnable() {
      public void run() {
        motorsSetTorqueAll(torquePlan);
      }
    });
    chatAdd("status", "Designer set torque, base to tip: " + motorsTorqueList(torquePlan) + " (0 = limp).");
  }

  String owed = "Applied to the arm. Fit quality " + nf(best.quality, 1, 2)
    + " (lower is better; under " + nf(AI_REFINE_QUALITY_THRESHOLD, 1, 2) + " is good).";
  if (best.critique != null) owed += " Visual critic: " + best.critique.score + "/10, says it looks like: " + best.critique.looksLike + ".";
  if (best != last) owed += " Your latest revision was judged worse, so the arm is showing your earlier version ('" + best.name + "').";
  // If the last assistant message was text only (it answered instead of revising), nothing is owed.
  JSONObject lastTool = chatLastToolUse(turn);
  chatCommitTo(chatHistory, chatPendingToolResults, turn, lastTool, owed);

  chatAdd("claude", chatMessageForUser(best.input, "Here's \"" + best.name + "\"."));
  String verdict = best.critique != null ? " | Critic " + best.critique.score + "/10" : "";
  chatAdd("status", "Fit quality " + nf(best.quality, 1, 2) + chatFitWord(best.quality) + verdict
    + (revisions > 0 ? " | revised " + revisions + "x" : "") + ". Press Create Shape to send it to the arm.");
}

// Sends `feedback` as the result of `prev`'s tool call and fits whatever
// the Designer sends back. Returns null if it failed or answered in words
// (that answer is shown in the chat).
ChatShapeTry designerRevise(JSONArray turn, ChatShapeTry prev, String feedback, int attempt) {
  turn.append(chatMessage("user", chatResultsFor(turn, prev.toolUse, feedback)));
  JSONObject resp = agentCall(designerBuildRequestBody(turn), "Designer");
  if (resp == null) {
    turn.remove(turn.size() - 1); // feedback never arrived - it'll be owed next turn instead
    return null;
  }
  turn.append(chatMessage("assistant", resp.getJSONArray("content")));
  JSONObject tu = chatFindToolUse(resp);
  if (tu == null || !tu.getString("name").equals("emit_shape_segments")) {
    String t = chatJoinText(resp);
    if (t.length() > 0) chatAdd("claude", t);
    return null;
  }
  return chatTryShape(tu, attempt);
}

class ChatShapeTry {
  JSONObject toolUse;
  JSONObject input;
  JSONArray segments;   // pristine copy (fitting mutates its own clone)
  boolean closed;
  String name;
  float quality;
  boolean centered;
  boolean curved;       // drawn as a smooth curve (aiShapeIsCurved)
  float[] angles;       // the 11 joint angles it ended up with
  ChatCritique critique;
}

// Fits one emit_shape_segments call to the arm (on the drawing thread).
// Base-anchored first; if that fits badly, also tries centering the shape
// and keeps whichever is better. Returns null if there were no segments.
ChatShapeTry chatTryShape(JSONObject toolUse, final int attempt) {
  JSONObject input = toolUse.getJSONObject("input");
  JSONArray segs = input.hasKey("segments") ? input.getJSONArray("segments") : null;
  if (segs == null || segs.size() < 1) return null;

  final ChatShapeTry t = new ChatShapeTry();
  t.toolUse = toolUse;
  t.input = input;
  t.segments = aiCloneSegments(segs);
  t.closed = input.hasKey("closed") && input.getBoolean("closed");
  t.name = input.hasKey("shape_name") ? input.getString("shape_name") : "custom";

  chatStatus = "Fitting \"" + t.name + "\" to the arm";
  chatRunOnMain(new Runnable() {
    public void run() {
      aiStopAnimation();
      aiAnchorAtCenter = false;
      aiGenerateFromSegments(aiCloneSegments(t.segments), t.closed, t.name);
      t.quality = aiLastFitQuality;
      t.centered = false;
      if (AI_TRY_CENTER_ANCHOR && t.quality > AI_REFINE_QUALITY_THRESHOLD) {
        aiAnchorAtCenter = true;
        aiGenerateFromSegments(aiCloneSegments(t.segments), t.closed, t.name);
        if (aiLastFitQuality < t.quality) {
          t.quality = aiLastFitQuality;
          t.centered = true;
        } else {
          aiAnchorAtCenter = false;
          aiGenerateFromSegments(aiCloneSegments(t.segments), t.closed, t.name);
        }
      }
      aiAnchorAtCenter = false;
      t.angles = new float[MOTOR_COUNT];
      arrayCopy(generatedAngles, t.angles);
      t.curved = aiShapeIsCurved;
      aiRecordFitHistory(t.name, attempt, t.quality);
    }
  });
  aiLog("[designer] fit '" + t.name + "': " + nf(t.quality, 1, 2) + (t.centered ? " (centered)" : ""));
  return t;
}

// Puts `best` back on the arm if something else was fitted after it.
void chatApplyBest(final ChatShapeTry best, ChatShapeTry lastFitted) {
  if (best == lastFitted) return;
  chatRunOnMain(new Runnable() {
    public void run() {
      aiAnchorAtCenter = best.centered;
      aiGenerateFromSegments(aiCloneSegments(best.segments), best.closed, best.name);
      aiAnchorAtCenter = false;
    }
  });
}

// Which of two versions to keep: the Critic's score decides when both were
// judged; otherwise (or on a tie) the better fit wins.
boolean chatBetter(ChatShapeTry a, ChatShapeTry b) {
  if (a.critique != null && b.critique != null && a.critique.score != b.critique.score) {
    return a.critique.score > b.critique.score;
  }
  return a.quality < b.quality;
}

String chatFitWord(float q) {
  if (q <= AI_REFINE_QUALITY_THRESHOLD) return " (good)";
  if (q <= AI_REFINE_QUALITY_THRESHOLD * 1.6) return " (rough)";
  return " (poor)";
}

// =====================================================================
//  VISUAL CRITIC
// =====================================================================

class ChatCritique {
  boolean recognizable = false;
  int score = 0;
  String looksLike = "";
  String problems = "";
  String suggestions = "";
  String note = "";

  String asFeedback() {
    return "A VISUAL CRITIC looked at a picture of the line the real arm made (it did not see your "
      + "segments) and scored it " + score + "/10" + (recognizable ? "" : " - NOT recognizable as requested")
      + ". It says it looks like: " + chatSentence(looksLike)
      + (problems.length() > 0 ? " Problems: " + chatSentence(problems) : "")
      + (suggestions.length() > 0 ? " Suggested changes: " + chatSentence(suggestions) : "");
  }
}

String CRITIC_SYSTEM =
  "You are the VISUAL CRITIC for a physical robotic arm made of 11 straight, hinged "
  + "links. You are shown three orthographic views (front, top, side) of the line the "
  + "real arm actually makes - the green dot is the fixed base, the red dot is the free "
  + "tip, small dots are the joints. You never see the designer's plan, only the result. "
  + "Decide whether an ordinary person who was told what it is meant to be would "
  + "recognize it.\n\n"
  + "Judge the overall silhouette from the most informative view - a flat shape shows "
  + "up clearly in one view and as a nearly straight line in another, which is expected. "
  + "Expect simplification: 11 rigid links can only approximate curves, corners come out "
  + "slightly angular or rounded, and a closed shape may leave a gap between the tip and "
  + "the start because the arm is an open chain. Don't penalize those. DO penalize wrong "
  + "proportions, missing or extra parts, curls or bends going the wrong way, parts that "
  + "collapse into a squiggle, and results that read as something else.\n\n"
  + "Score 1-10: 7 or more means clearly recognizable. In suggestions give the designer "
  + "1-3 concrete geometric changes, describing positions along the arm from the green "
  + "base to the red tip (e.g. 'the curl at the tip only turns about 90 degrees - make it "
  + "close to 180', 'the handle is too short compared to the head - roughly double it'). "
  + "Always answer with the submit_critique tool.";

JSONObject criticToolDefinition() {
  JSONObject props = new JSONObject();
  props.setJSONObject("recognizable", chatSchemaProp("boolean", "Would a person told what it should be recognize it?"));
  JSONObject score = chatSchemaProp("integer", "1-10. 7 or more = clearly recognizable.");
  score.setInt("minimum", 1);
  score.setInt("maximum", 10);
  props.setJSONObject("score", score);
  props.setJSONObject("looks_like", chatSchemaProp("string", "In a few words, what the line actually looks like."));
  props.setJSONObject("problems", chatSchemaProp("string", "What is wrong, if anything. Empty if nothing."));
  props.setJSONObject("suggestions", chatSchemaProp("string", "1-3 concrete geometric changes for the designer, positions given from base to tip."));
  props.setJSONObject("message_to_user", chatSchemaProp("string", "One short, friendly sentence for the user with your verdict."));

  JSONObject schema = new JSONObject();
  schema.setString("type", "object");
  schema.setJSONObject("properties", props);
  JSONArray required = new JSONArray();
  required.append("recognizable");
  required.append("score");
  required.append("looks_like");
  required.append("suggestions");
  required.append("message_to_user");
  schema.setJSONArray("required", required);

  JSONObject tool = new JSONObject();
  tool.setString("name", "submit_critique");
  tool.setString("description", "Report your judgement of the arm's actual shape.");
  tool.setJSONObject("input_schema", schema);
  return tool;
}

JSONObject chatSchemaProp(String type, String description) {
  JSONObject p = new JSONObject();
  p.setString("type", type);
  p.setString("description", description);
  return p;
}

// Shows the Critic a picture of what the arm made for `t`. Returns null if
// the Critic couldn't be reached (the turn simply carries on without it).
ChatCritique criticReview(ChatShapeTry t) {
  chatStatus = "Critic is looking at the arm";
  try {
    ArrayList<PVector> joints = aiComputeAchievableCurve(t.angles);
    ArrayList<PVector> line = t.curved && joints.size() >= 2
      ? aiCatmullRomChain(joints, 8, AI_CURVE_RENDER_TENSION) : joints;
    String png = chatRenderArmViewsB64(line, joints);

    StringBuilder asked = new StringBuilder();
    int from = max(0, designerRequests.size() - 4);
    for (int i = from; i < designerRequests.size(); i++) {
      asked.append(i == from ? "" : " -> ").append("\"").append(designerRequests.get(i)).append("\"");
    }
    String text = "What the user asked for (oldest first; later messages are edits to the same shape): "
      + asked + ". The designer called the shape \"" + t.name + "\""
      + (t.input.hasKey("message_to_user") ? " and describes it as: \"" + t.input.getString("message_to_user") + "\"" : "")
      + ". " + (t.closed ? "It is meant to be a CLOSED outline (a gap between tip and start is expected). " : "")
      + "Judge the picture.";

    JSONObject source = new JSONObject();
    source.setString("type", "base64");
    source.setString("media_type", "image/png");
    source.setString("data", png);
    JSONObject image = new JSONObject();
    image.setString("type", "image");
    image.setJSONObject("source", source);
    JSONArray content = new JSONArray();
    content.append(image);
    content.append(chatTextBlock(text));
    JSONArray messages = new JSONArray();
    messages.append(chatMessage("user", content));

    JSONObject body = new JSONObject();
    body.setString("model", ANTHROPIC_MODEL);
    body.setInt("max_tokens", 1024);
    JSONArray system = new JSONArray();
    system.append(chatTextBlock(CRITIC_SYSTEM));
    body.setJSONArray("system", system);
    JSONArray tools = new JSONArray();
    tools.append(criticToolDefinition());
    body.setJSONArray("tools", tools);
    JSONObject choice = new JSONObject();
    choice.setString("type", "tool");
    choice.setString("name", "submit_critique");
    body.setJSONObject("tool_choice", choice);
    body.setJSONArray("messages", messages);

    JSONObject resp = agentCall(body, "Critic");
    if (resp == null) return null;
    JSONObject tu = chatFindToolUse(resp);
    if (tu == null) return null;
    JSONObject in = tu.getJSONObject("input");
    ChatCritique c = new ChatCritique();
    c.recognizable = in.hasKey("recognizable") && in.getBoolean("recognizable");
    c.score = in.hasKey("score") ? constrain(in.getInt("score"), 1, 10) : 1;
    c.looksLike = in.hasKey("looks_like") ? in.getString("looks_like") : "";
    c.problems = in.hasKey("problems") ? in.getString("problems") : "";
    c.suggestions = in.hasKey("suggestions") ? in.getString("suggestions") : "";
    c.note = in.hasKey("message_to_user") ? in.getString("message_to_user") : "";
    aiLog("[critic] " + c.score + "/10, looks like: " + c.looksLike + " | problems: " + c.problems + " | suggestions: " + c.suggestions);
    return c;
  }
  catch (Exception e) {
    aiLog("[critic] failed: " + e);
    chatAdd("status", "The Critic couldn't look this time (" + e.getMessage() + ") - continuing without it.");
    return null;
  }
}

void chatShowCritique(ChatCritique c) {
  if (c == null) return;
  String msg = (c.note.length() > 0 ? c.note : "Looks like: " + c.looksLike + ".") + "  (" + c.score + "/10)";
  chatAdd("critic", msg);
}

// Draws the arm's actual line in three orthographic views side by side
// (plain Java2D, so it works off the drawing thread) and returns a base64 PNG.
// `line` is what gets drawn, `joints` get dots. Screen-style axes: y points down.
String chatRenderArmViewsB64(ArrayList<PVector> line, ArrayList<PVector> joints) throws IOException {
  int panelW = 380, panelH = 340, titleH = 34, margin = 34;
  int W = panelW * 3, H = panelH;
  java.awt.image.BufferedImage img = new java.awt.image.BufferedImage(W, H, java.awt.image.BufferedImage.TYPE_INT_RGB);
  java.awt.Graphics2D g = img.createGraphics();
  g.setRenderingHint(java.awt.RenderingHints.KEY_ANTIALIASING, java.awt.RenderingHints.VALUE_ANTIALIAS_ON);
  g.setRenderingHint(java.awt.RenderingHints.KEY_TEXT_ANTIALIASING, java.awt.RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
  g.setColor(java.awt.Color.WHITE);
  g.fillRect(0, 0, W, H);

  String[] titles = {"FRONT view", "TOP view (from above)", "SIDE view"};
  // which coordinates go across (u) and down (v) in each view: 0=x 1=y 2=z
  int[][] axes = {{0, 1}, {0, 2}, {2, 1}};

  // one scale for all three views so sizes are comparable
  float scale = Float.MAX_VALUE;
  float[][] mins = new float[3][2];
  float[][] spans = new float[3][2];
  for (int v = 0; v < 3; v++) {
    float uMin = Float.MAX_VALUE, uMax = -Float.MAX_VALUE, vMin = Float.MAX_VALUE, vMax = -Float.MAX_VALUE;
    for (PVector p : line) {
      float uu = chatAxis(p, axes[v][0]), vv = chatAxis(p, axes[v][1]);
      uMin = min(uMin, uu); uMax = max(uMax, uu);
      vMin = min(vMin, vv); vMax = max(vMax, vv);
    }
    mins[v][0] = uMin; mins[v][1] = vMin;
    spans[v][0] = max(uMax - uMin, 1); spans[v][1] = max(vMax - vMin, 1);
    scale = min(scale, min((panelW - 2 * margin) / spans[v][0], (panelH - titleH - 2 * margin) / spans[v][1]));
  }

  for (int v = 0; v < 3; v++) {
    int x0 = v * panelW;
    g.setColor(new java.awt.Color(225, 225, 225));
    if (v > 0) g.drawLine(x0, 0, x0, H);
    g.setColor(new java.awt.Color(30, 30, 30));
    g.setFont(new java.awt.Font("SansSerif", java.awt.Font.BOLD, 18));
    g.drawString(titles[v], x0 + 12, 24);

    float cx = x0 + panelW / 2.0, cy = titleH + (panelH - titleH) / 2.0;
    float offU = mins[v][0] + spans[v][0] / 2.0, offV = mins[v][1] + spans[v][1] / 2.0;

    g.setColor(new java.awt.Color(20, 20, 20));
    g.setStroke(new java.awt.BasicStroke(5, java.awt.BasicStroke.CAP_ROUND, java.awt.BasicStroke.JOIN_ROUND));
    for (int i = 1; i < line.size(); i++) {
      PVector a = line.get(i - 1), b = line.get(i);
      g.drawLine(round(cx + (chatAxis(a, axes[v][0]) - offU) * scale), round(cy + (chatAxis(a, axes[v][1]) - offV) * scale),
                 round(cx + (chatAxis(b, axes[v][0]) - offU) * scale), round(cy + (chatAxis(b, axes[v][1]) - offV) * scale));
    }
    g.setColor(new java.awt.Color(90, 90, 90));
    for (PVector p : joints) {
      int px = round(cx + (chatAxis(p, axes[v][0]) - offU) * scale), py = round(cy + (chatAxis(p, axes[v][1]) - offV) * scale);
      g.fillOval(px - 4, py - 4, 8, 8);
    }
    PVector base = joints.get(0), tip = joints.get(joints.size() - 1);
    g.setColor(new java.awt.Color(0, 170, 70));
    g.fillOval(round(cx + (chatAxis(base, axes[v][0]) - offU) * scale) - 9, round(cy + (chatAxis(base, axes[v][1]) - offV) * scale) - 9, 18, 18);
    g.setColor(new java.awt.Color(220, 40, 40));
    g.fillOval(round(cx + (chatAxis(tip, axes[v][0]) - offU) * scale) - 8, round(cy + (chatAxis(tip, axes[v][1]) - offV) * scale) - 8, 16, 16);
  }
  g.dispose();

  ByteArrayOutputStream out = new ByteArrayOutputStream();
  javax.imageio.ImageIO.write(img, "png", out);
  return java.util.Base64.getEncoder().encodeToString(out.toByteArray());
}

// Trims text and makes sure it ends with sentence punctuation.
String chatSentence(String t) {
  t = t.trim();
  if (t.length() == 0) return t;
  char last = t.charAt(t.length() - 1);
  return (last == '.' || last == '!' || last == '?') ? t : t + ".";
}

float chatAxis(PVector p, int axis) {
  return axis == 0 ? p.x : (axis == 1 ? p.y : p.z);
}

// =====================================================================
//  MOTION
// =====================================================================

String MOTION_SYSTEM_HEAD =
  "You are the MOTION agent inside a chat panel that drives a physical robotic arm "
  + "made of 11 hinged links (Dynamixel servos). A separate DESIGNER agent makes still "
  + "shapes; your job is to bring the current shape to life as a LOOPING ANIMATION and "
  + "refine it with the user over several messages. Your one tool is "
  + "emit_shape_animation. The full technical guidance follows.\n\n"
  + "=== SHAPE AND ANIMATION GUIDANCE ===\n\n";

String MOTION_SYSTEM_TAIL =
  "\n\n=== HOW TO WORK IN THIS CHAT ===\n"
  + "- Each user message ends with the Designer's CURRENT SHAPE (its segments, when "
  + "available) and the arm's state. Build the motion around that shape: for poses, "
  + "start each pose from those same segments and change only what the motion needs "
  + "(curve_amount, direction, curve_toward, lengths, scale); for a traveling wave, keep "
  + "the amplitude modest enough that the shape still reads. If there is no shape yet, "
  + "invent a sensible body for the motion.\n"
  + "- Every message continues the same session. For follow-ups ('slower', 'bigger "
  + "swing', 'softer tail', 'more like a wave'), start from YOUR MOST RECENT animation in "
  + "this conversation and change only what was asked. Always send the COMPLETE "
  + "animation.\n"
  + "- The user can limit each joint's TORQUE (shown in the arm-state note when any joint "
  + "is below 100%). Respect it: joints at 0% are limp and will not move or hold, so build "
  + "the motion from the other joints and keep the limp ones still; low-torque joints are "
  + "weak and slow, so give them smaller, gentler moves, avoid making them hold big bends "
  + "or carry the joints after them, and let the stronger joints do the work. Never set "
  + "joint_stiffness stiffer (lower level) on a low-torque joint than a strong one needs.\n"
  + "- The Designer may hand you a TORQUE PLAN with the shape (in the shape note). Use it as "
  + "the starting point for joint_torque: keep its limp (0) and soft joints unless the user "
  + "asks otherwise. Limp joints swing PASSIVELY - they are thrown around by the motion of "
  + "the strong joints, so animate the strong joints to make them swing (e.g. nunchuck: "
  + "whip the rigid stick at the base back and forth and let the limp chain and far stick "
  + "fly). Their angles in your poses are ignored by the hardware. joint_torque only "
  + "applies while the animation plays; the previous torque comes back when it stops.\n"
  + "- If the user asks for a still shape, briefly tell them to switch the mode to Shape "
  + "(the Designer handles that).\n"
  + "- You may think briefly before the tool call, but the user does NOT see that text. "
  + "Put what the user should read in the tool's message_to_user field: 1-2 short, plain "
  + "sentences about the motion. When you reply without a tool, keep it short.";

JSONObject motionBuildRequestBody(JSONArray turn) {
  JSONObject body = new JSONObject();
  body.setString("model", ANTHROPIC_MODEL);
  body.setInt("max_tokens", 4096);

  JSONArray system = new JSONArray();
  JSONObject sys1 = chatTextBlock(MOTION_SYSTEM_HEAD + AI_ANIMATION_SYSTEM_PROMPT + MOTION_SYSTEM_TAIL);
  sys1.setJSONObject("cache_control", aiCacheControlEphemeral());
  system.append(sys1);
  body.setJSONArray("system", system);

  JSONObject animTool = aiBuildAnimationToolDefinition(); // keeps its cache marker
  chatAddMessageField(animTool);
  chatAddTorqueField(animTool,
    "OPTIONAL. Torque for each joint WHILE this animation plays, 11 integers 0-100, base to "
    + "tip (restored when it stops). 100 = full strength, lower = weaker/softer, 0 = LIMP: "
    + "the joint swings freely, driven only by the motion of the joints around it (chains, "
    + "ropes, a nunchuck's link). Start from the Designer's torque plan if one was given. "
    + "Leave out to keep the current torque.");
  JSONArray tools = new JSONArray();
  tools.append(animTool);
  body.setJSONArray("tools", tools);
  body.setJSONObject("tool_choice", chatAutoToolChoice());

  JSONArray messages = new JSONArray();
  for (int i = 0; i < motionHistory.size(); i++) messages.append(motionHistory.getJSONObject(i));
  for (int i = 0; i < turn.size(); i++) messages.append(turn.getJSONObject(i));
  body.setJSONArray("messages", messages);
  return body;
}

// The Designer's current shape, as context for the Motion agent.
String motionShapeContext() {
  if (agentShapeSegments != null) {
    return "\n\n[Designer's current shape: '" + agentShapeName + "', closed=" + agentShapeClosed
      + ", segments: " + agentShapeSegments.format(-1)
      + (agentShapeTorque != null ? "; Designer's TORQUE PLAN, base to tip (0 = limp): " + motorsTorqueList(agentShapeTorque) : "")
      + "]";
  }
  if (agentShapeName.length() > 0) {
    return "\n\n[Designer's current shape: the calibrated preset '" + agentShapeName
      + "' (no segment list available - see the joint angles below).]";
  }
  return "\n\n[No shape has been designed yet.]";
}

// One full Animate-mode turn.
void motionRunTurn(String userText) {
  aiLog("[motion] you: " + userText);

  String angles = "";
  if (!aiAnimating) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < MOTOR_COUNT; i++) {
      sb.append(i == 0 ? "" : ", ").append(round(cp5.getController("Motor " + (i + 2)).getValue()));
    }
    angles = "\n[Current joint angles, base to tip (180 = straight): [" + sb + "]]";
  }

  JSONArray content = new JSONArray();
  for (JSONObject tr : motionPendingToolResults) content.append(tr);
  content.append(chatTextBlock(userText + motionShapeContext() + angles + chatStateNote()));
  JSONArray turn = new JSONArray();
  turn.append(chatMessage("user", content));

  JSONObject resp = agentCall(motionBuildRequestBody(turn), "Motion agent");
  if (resp == null) return;
  turn.append(chatMessage("assistant", resp.getJSONArray("content")));

  JSONObject toolUse = chatFindToolUse(resp);
  String plainText = chatJoinText(resp);
  if (plainText.length() > 0) aiLog("[motion] " + plainText);

  if (toolUse == null) {
    chatCommitTo(motionHistory, motionPendingToolResults, turn, null, null);
    chatAdd("motion", plainText.length() > 0 ? plainText : "(no reply)");
    return;
  }
  if (!toolUse.getString("name").equals("emit_shape_animation")) {
    chatCommitTo(motionHistory, motionPendingToolResults, turn, toolUse, "ERROR: unknown tool.");
    chatAdd("error", "The Motion agent used an unknown tool.");
    return;
  }

  JSONObject input = toolUse.getJSONObject("input");
  chatStatus = "Building the animation";
  final AIAnimationResult anim = aiParseAnimationInput(input);
  if (!anim.ok) {
    chatCommitTo(motionHistory, motionPendingToolResults, turn, toolUse, "ERROR: " + anim.errorMessage + " Nothing changed on the arm.");
    chatAdd("error", anim.errorMessage);
    return;
  }
  final int[] animTorque = agentsParseTorque(input);
  chatRunOnMain(new Runnable() {
    public void run() {
      aiStartAnimation(anim);
      if (animTorque != null) motorsApplyAnimationTorque(animTorque);
    }
  });
  if (animTorque != null) chatAdd("status", "Torque during this animation, base to tip: "
    + motorsTorqueList(animTorque) + " (0 = limp; restored when it stops).");
  String summary = anim.isTravelingWave
    ? "traveling wave, " + nf(anim.waveAmplitudeDeg, 1, 0) + " deg, " + nf(anim.waveWavelengthJoints, 1, 1)
      + " joints/cycle, " + round(anim.wavePeriodMs) + " ms/cycle"
    : anim.poses.size() + " poses, " + round(anim.frameDurationMs) + " ms per pose";
  chatCommitTo(motionHistory, motionPendingToolResults, turn, toolUse, "Animation '" + anim.animationName + "' ("
    + summary + ") is now playing on the arm. Joint stiffness: " + aiJoinIntArray(anim.jointStiffness) + ".");
  chatAdd("motion", chatMessageForUser(input, "Playing \"" + anim.animationName + "\"."));
  chatAdd("status", "Playing: " + anim.animationName + " (" + summary + "). Stop Animation ends it.");
}

// =====================================================================
//  Shared plumbing
// =====================================================================

// Sends a request; returns the parsed reply, or null after showing the
// problem in the chat.
JSONObject agentCall(JSONObject body, String who) {
  try {
    String json = aiCallAnthropic(body.toString());
    JSONObject resp = JSONObject.parse(json);
    if (resp == null) {
      chatAdd("error", who + ": empty reply - try again.");
      return null;
    }
    if (resp.hasKey("type") && resp.getString("type").equals("error")) {
      chatAdd("error", who + " - Claude API error: " + resp.getJSONObject("error").getString("message"));
      return null;
    }
    if (!resp.hasKey("content")) {
      chatAdd("error", who + ": unexpected reply - try again.");
      return null;
    }
    return resp;
  }
  catch (Exception e) {
    chatAdd("error", who + " couldn't reach Claude: " + e.getMessage());
    return null;
  }
}

// Adds the optional joint_torque field (11 x 0-100) to a tool's schema.
void chatAddTorqueField(JSONObject tool, String description) {
  JSONObject item = new JSONObject();
  item.setString("type", "integer");
  item.setInt("minimum", 0);
  item.setInt("maximum", 100);
  JSONObject prop = new JSONObject();
  prop.setString("type", "array");
  prop.setJSONObject("items", item);
  prop.setInt("minItems", MOTOR_COUNT);
  prop.setInt("maxItems", MOTOR_COUNT);
  prop.setString("description", description);
  tool.getJSONObject("input_schema").getJSONObject("properties").setJSONObject("joint_torque", prop);
}

// joint_torque from a tool call, or null if absent / not exactly 11 values.
int[] agentsParseTorque(JSONObject input) {
  if (input == null || !input.hasKey("joint_torque")) return null;
  try {
    JSONArray a = input.getJSONArray("joint_torque");
    if (a == null || a.size() != MOTOR_COUNT) return null;
    int[] out = new int[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) out[i] = constrain(round(a.getFloat(i)), 0, 100);
    return out;
  }
  catch (Exception e) {
    return null;
  }
}

JSONObject chatAutoToolChoice() {
  JSONObject c = new JSONObject();
  c.setString("type", "auto");
  c.setBoolean("disable_parallel_tool_use", true); // one change per reply
  return c;
}

void chatAddMessageField(JSONObject tool) {
  tool.getJSONObject("input_schema").getJSONObject("properties").setJSONObject("message_to_user",
    chatSchemaProp("string", "1-2 short, friendly sentences shown to the user in the chat about what you "
      + "made or changed. Plain language, no segment details."));
}

// Adds a finished turn to an agent's memory. If it ended on a tool call,
// that call's result is owed at the start of the agent's next message.
void chatCommitTo(JSONArray history, ArrayList<JSONObject> pending, JSONArray turn, JSONObject owedToolUse, String owedResult) {
  JSONArray owed = chatResultsFor(turn, owedToolUse, owedResult == null ? "Done." : owedResult);
  for (int i = 0; i < turn.size(); i++) history.append(turn.getJSONObject(i));
  pending.clear();
  for (int i = 0; i < owed.size(); i++) pending.add(owed.getJSONObject(i));
}

// tool_result blocks answering EVERY tool call in the turn's last assistant
// message (the API rejects the next request otherwise): `main` gets
// `mainText`, any others are marked as skipped.
JSONArray chatResultsFor(JSONArray turn, JSONObject main, String mainText) {
  JSONArray results = new JSONArray();
  JSONObject lastMsg = turn.getJSONObject(turn.size() - 1);
  if (!lastMsg.getString("role").equals("assistant")) return results;
  JSONArray content = lastMsg.getJSONArray("content");
  String mainId = (main == null) ? "" : main.getString("id");
  for (int i = 0; i < content.size(); i++) {
    JSONObject block = content.getJSONObject(i);
    if (!block.hasKey("type") || !block.getString("type").equals("tool_use")) continue;
    boolean isMain = block.getString("id").equals(mainId);
    results.append(chatToolResult(block, isMain ? mainText
      : "Skipped - only one change per reply is applied."));
  }
  return results;
}

// The tool call in the turn's last assistant message, if any.
JSONObject chatLastToolUse(JSONArray turn) {
  JSONObject lastMsg = turn.getJSONObject(turn.size() - 1);
  if (!lastMsg.getString("role").equals("assistant")) return null;
  JSONArray content = lastMsg.getJSONArray("content");
  for (int i = 0; i < content.size(); i++) {
    JSONObject block = content.getJSONObject(i);
    if (block.hasKey("type") && block.getString("type").equals("tool_use")) return block;
  }
  return null;
}

// What an agent should know about the arm right now, appended to each user message.
String chatStateNote() {
  String state;
  if (aiAnimating) state = "playing the animation '" + aiAnimName + "'";
  else if (selectedShape.equals("none")) state = "straight / neutral (nothing designed yet)";
  else state = "holding the shape '" + (selectedShape.startsWith("ai: ") ? selectedShape.substring(4) : selectedShape) + "'";

  String hand = "";
  if (!aiAnimating && !selectedShape.equals("none")) {
    boolean changed = false;
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < MOTOR_COUNT; i++) {
      float v = cp5.getController("Motor " + (i + 2)).getValue();
      if (abs(v - generatedAngles[i]) > 2) changed = true;
      sb.append(i == 0 ? "" : ", ").append(round(v));
    }
    if (changed) {
      hand = "; the user has hand-adjusted the sliders since, and the joint angles are now ["
        + sb + "] (degrees, base to tip, 180 = straight)";
    }
  }
  return "\n\n[Arm right now: " + state + hand + motorsTorqueNote() + ".]";
}

JSONObject chatTextBlock(String text) {
  JSONObject b = new JSONObject();
  b.setString("type", "text");
  b.setString("text", text);
  return b;
}

JSONObject chatMessage(String role, JSONArray content) {
  JSONObject m = new JSONObject();
  m.setString("role", role);
  m.setJSONArray("content", content);
  return m;
}

JSONObject chatToolResult(JSONObject toolUse, String text) {
  JSONObject r = new JSONObject();
  r.setString("type", "tool_result");
  r.setString("tool_use_id", toolUse.getString("id"));
  r.setString("content", text);
  return r;
}

JSONObject chatFindToolUse(JSONObject resp) {
  JSONArray content = resp.getJSONArray("content");
  for (int i = 0; i < content.size(); i++) {
    JSONObject block = content.getJSONObject(i);
    if (block.hasKey("type") && block.getString("type").equals("tool_use")) return block;
  }
  return null;
}

String chatJoinText(JSONObject resp) {
  JSONArray content = resp.getJSONArray("content");
  StringBuilder sb = new StringBuilder();
  for (int i = 0; i < content.size(); i++) {
    JSONObject block = content.getJSONObject(i);
    if (block.hasKey("type") && block.getString("type").equals("text")) {
      if (sb.length() > 0) sb.append("\n");
      sb.append(block.getString("text").trim());
    }
  }
  return sb.toString().trim();
}

String chatMessageForUser(JSONObject input, String fallback) {
  if (input != null && input.hasKey("message_to_user")) {
    String m = input.getString("message_to_user").trim();
    if (m.length() > 0) return m;
  }
  return fallback;
}
