// One-time tool: find each connected Dynamixel's CURRENT ID (read from the
// motor itself by scanning the bus), print it, then change ONE motor to NEW_ID.
// Not part of the robot firmware - re-upload firmware/lineform-ai/lineform-ai.ino when done.
//
// Workflow: build the chain one motor at a time (power off before plugging).
// Set NEW_ID = 2 for the base motor, upload, check the Serial Monitor; then
// add the next motor, NEW_ID = 3, upload again ... up to 11.

#include <DynamixelShield.h>

DynamixelShield dxl;

const uint8_t NEW_ID = 11;             // change for each motor: 2, 3, ... 11
const uint32_t TARGET_BAUD = 57600;   // what firmware/lineform-ai/lineform-ai.ino uses
const float PROTOCOL = 2.0;

// IDs already assigned to motors earlier in the chain are 2 .. NEW_ID-1,
// so the motor to change is the one whose ID is NOT in that range.
bool alreadyAssigned(uint8_t id) {
  return id >= 2 && id < NEW_ID;
}

// Scans IDs 0-252 at one baud rate. Prints every motor found and returns
// how many there were; the IDs are written into found[].
int scanAtBaud(uint32_t baud, uint8_t found[], int maxFound) {
  dxl.begin(baud);
  dxl.setPortProtocolVersion(PROTOCOL);
  int count = 0;
  for (int id = 0; id <= 252; id++) {
    if (dxl.ping(id)) {
      Serial.print("  Found motor: ID ");
      Serial.print(id);
      Serial.print("  model ");
      Serial.print(dxl.getModelNumber(id));
      Serial.print("  baud ");
      Serial.println(baud);
      if (count < maxFound) found[count] = id;
      count++;
    }
  }
  return count;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}   // wait for the Serial Monitor so nothing is missed

  Serial.println();
  Serial.print("=== Set Motor ID -> target ID ");
  Serial.print(NEW_ID);
  Serial.println(" ===");

  // 1) Read the current IDs from the bus. Try the target baud first, then
  //    other common Dynamixel bauds in case a motor was set differently.
  //    Scans EVERY baud (doesn't stop at the first hit), since already-set
  //    motors answer at 57600 while a new one might sit at another baud.
  // (2M/3M/4M left out: the MKR + shield can hang trying to talk at those
  // speeds, and new/factory motors are never set to them.)
  const uint32_t bauds[] = {57600, 1000000, 115200, 9600};
  uint8_t found[20];
  uint32_t foundAt[20];
  int count = 0;
  for (uint8_t b = 0; b < sizeof(bauds) / sizeof(bauds[0]); b++) {
    Serial.print("Scanning at ");
    Serial.print(bauds[b]);
    Serial.println(" baud...");
    uint8_t ids[20];
    int n = scanAtBaud(bauds[b], ids, 20);
    for (int k = 0; k < n && k < 20 && count < 20; k++) {
      found[count] = ids[k];
      foundAt[count] = bauds[b];
      count++;
    }
  }

  if (count == 0) {
    Serial.println("No motor found at any baud - check motor power, cable and shield.");
    return;
  }

  // 2) Pick the ONE motor to change: the one not already assigned earlier.
  int target = -1;
  uint32_t foundBaud = TARGET_BAUD;
  int candidates = 0;
  bool newIdTaken = false;
  if (count == 1) {
    // Only ONE motor on the bus (one-at-a-time workflow): change it,
    // whatever its current ID is.
    if (found[0] == NEW_ID) {
      newIdTaken = true;
    } else {
      target = found[0];
      foundBaud = foundAt[0];
      candidates = 1;
    }
  } else {
    // Several motors chained: change the one not already assigned earlier.
    for (int i = 0; i < count; i++) {
      if (found[i] == NEW_ID) newIdTaken = true;
      if (!alreadyAssigned(found[i]) && found[i] != NEW_ID) {
        target = found[i];
        foundBaud = foundAt[i];
        candidates++;
      }
    }
  }

  // Talk to the target motor at whatever baud it was found at.
  if (candidates == 1) {
    dxl.begin(foundBaud);
    dxl.setPortProtocolVersion(PROTOCOL);
  }

  if (newIdTaken && candidates == 0) {
    Serial.print("A motor already has ID ");
    Serial.print(NEW_ID);
    Serial.println(" - nothing to change. Add the next motor and increase NEW_ID.");
    return;
  }
  if (newIdTaken) {
    Serial.print("ID ");
    Serial.print(NEW_ID);
    Serial.println(" is already used by another motor - pick a different NEW_ID. Nothing changed.");
    return;
  }
  if (candidates == 0) {
    Serial.println("The new motor wasn't detected - only already-set motors answered.");
    Serial.println("Check its cable is fully seated in the previous motor's free port,");
    Serial.println("motor power is on, and its LED blinks once at power-up. Nothing changed.");
    return;
  }
  if (candidates != 1) {
    Serial.print(candidates);
    Serial.println(" unassigned motors found - connect only ONE new motor at a time. Nothing changed.");
    Serial.println("(If several share the same ID, they answer together and look like one -");
    Serial.println(" so also make sure only one NEW motor is plugged in.)");
    return;
  }

  Serial.print("Old ID: ");
  Serial.print(target);
  Serial.print("  ->  New ID: ");
  Serial.println(NEW_ID);

  // 3) Change the ID (ID and baud live in EEPROM, which needs torque off).
  dxl.torqueOff(target);
  if (!dxl.setID(target, NEW_ID)) {
    Serial.println("Failed to change ID.");
    return;
  }
  delay(100);

  // 4) If the motor was at a different baud, move it to the firmware's baud.
  if (foundBaud != TARGET_BAUD) {
    if (dxl.setBaudrate(NEW_ID, TARGET_BAUD)) {
      Serial.print("Baud changed from ");
      Serial.print(foundBaud);
      Serial.print(" to ");
      Serial.println(TARGET_BAUD);
    } else {
      Serial.println("ID changed, but failed to change baud - fix it in Dynamixel Wizard.");
    }
    delay(100);
    dxl.begin(TARGET_BAUD);
    dxl.setPortProtocolVersion(PROTOCOL);
  }

  // 5) Verify.
  if (dxl.ping(NEW_ID)) {
    Serial.print("Done: motor now answers at ID ");
    Serial.print(NEW_ID);
    Serial.print(", ");
    Serial.print(TARGET_BAUD);
    Serial.println(" baud.");
  } else {
    Serial.println("Changed, but the motor doesn't answer at the new ID - power-cycle it and rescan.");
  }
}

void loop() {}
