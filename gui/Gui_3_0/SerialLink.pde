// =====================================================================
//  SERIAL LINK - framing + flow control for every command to the Arduino.
//  (Matches the "COMMAND LINK" section of lineform-ai.ino.)
//
//  Each command goes out as:  <header> <data bytes...> <checksum>
//    header >= 230, every data byte < 230, checksum = (header + data) & 0x7F
//    (230 = HEADER_MIN in the firmware; was 240 until header 239 was added)
//  so the firmware can detect lost / sliced bytes and drop the broken
//  command instead of misreading it.
//
//  Speed limit = flow control: after a command WITHOUT a reply (motion,
//  torque, stiffness, hold) we wait for the Arduino's ACK - sent once the
//  command has been fully executed - before anything else is sent. So at
//  most one command is ever in flight and its input buffer can't overflow,
//  however fast the GUI produces commands (a burst just takes a little
//  longer). A NAK (command arrived damaged) -> sent again, once.
//  Commands WITH a reply (reads/status/check) are paced by their reply in
//  readAllMotorValues() / readMotorPosition().
//
//  Old firmware never ACKs: after 3 missed ACKs in a row the link falls
//  back to a fixed gap between commands and says so in the console.
// =====================================================================

final int SERIAL_ACK = 0x06;
final int SERIAL_NAK = 0x15;
final int SERIAL_ACK_TIMEOUT_MS = 600;   // torque-on may reboot a motor first (~100 ms+)
final int SERIAL_FALLBACK_GAP_MS = 20;   // pacing when the firmware doesn't ACK
int serialAckMisses = 0;
boolean serialNoAckWarned = false;
int serialNakCount = 0, serialAckTimeouts = 0;

// Data bytes per header (-1 = unknown). Must match dataLenOf() in the firmware.
int serialDataLen(int h) {
  if (h == 255) return MOTOR_COUNT;                    // all goals
  if (h == 254 || h == 252 || h == 251 || h == 249 || h == 241) return 2;
  if (h == 253 || h == 250 || h == 248 || (h >= 242 && h <= 247) || (h >= 237 && h <= 240)) return 1;
  return -1;
}

boolean serialHasReply(int h) { return serialDataLen(h) == 1; }

// Called (inside the synchronized sendSerialBytes) with one or more commands.
void serialLinkSend(int[] bytes) {
  if (!serialConnected() || bytes == null) return;
  int k = 0;
  while (k < bytes.length) {
    int h = bytes[k];
    int n = serialDataLen(h);
    if (n < 0 || k + 1 + n > bytes.length) {
      println("[serial] malformed command (header " + h + ", " + (bytes.length - k) + " bytes) - not sent");
      return;
    }
    int[] cmd = new int[1 + n];
    arrayCopy(bytes, k, cmd, 0, 1 + n);
    k += 1 + n;

    // Manipulation mode (Modes.pde): only reads and torque-OFF may go out.
    if (manipulationMode && !serialAllowedInManipulation(cmd)) {
      println("[Manipulation mode] blocked serial command " + h);
      continue;
    }
    serialSendOne(cmd);
  }
}

void serialSendOne(int[] cmd) {
  // Data must stay below 230 or the firmware would take it for a header.
  int sum = 0;
  for (int j = 0; j < cmd.length; j++) {
    if (j > 0 && (cmd[j] < 0 || cmd[j] >= 230)) {
      println("[serial] data byte " + cmd[j] + " out of range in command " + cmd[0] + " - clamped");
      cmd[j] = constrain(cmd[j], 0, 229);
    }
    sum += cmd[j];
  }
  int checksum = sum & 0x7F;

  boolean needAck = !serialHasReply(cmd[0]);
  for (int attempt = 0; attempt < 2; attempt++) {
    if (needAck) serialDrainStale();
    for (int b : cmd) myPort.write(b);
    myPort.write(checksum);
    if (!needAck) return;                 // the caller reads the reply
    int r = serialWaitAck();
    if (r != SERIAL_NAK) return;          // ACK (or timeout / old firmware)
    serialNakCount++;
    println("[serial] command " + cmd[0] + " arrived damaged (NAK) - " + (attempt == 0 ? "resending" : "giving up"));
  }
}

// Throw away bytes left over from an earlier, timed-out reply, so a stale
// byte can't be mistaken for this command's ACK.
void serialDrainStale() {
  while (myPort.available() > 0) myPort.read();
}

// Returns SERIAL_ACK, SERIAL_NAK, or -1 on timeout.
int serialWaitAck() {
  boolean fallback = serialAckMisses >= 3;
  int timeout = fallback ? SERIAL_FALLBACK_GAP_MS : SERIAL_ACK_TIMEOUT_MS;
  long start = millis();
  while (millis() - start < timeout) {
    while (myPort.available() > 0) {
      int b = myPort.read();
      if (b == SERIAL_ACK || b == SERIAL_NAK) {
        if (serialAckMisses >= 3) println("[serial] Arduino ACKs again - flow control back on.");
        serialAckMisses = 0;
        return b;
      }
    }
    if (millis() - start < 3) Thread.yield();
    else delay(1);
  }
  if (!fallback) {
    serialAckTimeouts++;
    if (++serialAckMisses == 3 && !serialNoAckWarned) {
      serialNoAckWarned = true;
      println("[serial] The Arduino isn't acknowledging commands - it is probably running OLD firmware. "
        + "Upload firmware/lineform-ai/lineform-ai.ino. Using a fixed " + SERIAL_FALLBACK_GAP_MS + " ms gap meanwhile.");
    }
  }
  return -1;
}
