// One-time tool: shift the IDs of ALL connected Dynamixels at once by SHIFT
// (default -1, i.e. new ID = old ID - 1), with the whole chain plugged in.
// Not part of the robot firmware - re-upload firmware/lineform-ai/lineform-ai.ino when done.
//
// Why this is safe with every motor connected: IDs are changed in ASCENDING
// order for a -1 shift (and descending for +1), so each motor moves into an
// ID that was just vacated - two motors never share an ID mid-way.
//
// Set DRY_RUN = true first: it only prints what WOULD change.

#include <DynamixelShield.h>

DynamixelShield dxl;

const int SHIFT = -1;            // new ID = old ID + SHIFT
const bool DRY_RUN = true;        // true = just print the plan; false = actually change IDs
const uint8_t MIN_ID = 1;        // only motors with IDs in [MIN_ID, MAX_ID] are shifted
const uint8_t MAX_ID = 252;
const uint32_t BAUD = 57600;     // what firmware/lineform-ai/lineform-ai.ino uses
const float PROTOCOL = 2.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}   // wait for the Serial Monitor

  Serial.println();
  Serial.print("=== Shift Motor IDs by ");
  Serial.print(SHIFT);
  Serial.println(DRY_RUN ? "  (DRY RUN - nothing will change) ===" : " ===");

  dxl.begin(BAUD);
  dxl.setPortProtocolVersion(PROTOCOL);

  // 1) Read every motor's current ID from the bus.
  uint8_t ids[32];
  int count = 0;
  Serial.println("Scanning at 57600 baud...");
  for (int id = 0; id <= 252 && count < 32; id++) {
    if (dxl.ping(id)) {
      ids[count++] = id;
      Serial.print("  Found motor: ID ");
      Serial.print(id);
      Serial.print("  model ");
      Serial.println(dxl.getModelNumber(id));
    }
  }
  if (count == 0) {
    Serial.println("No motors found - check motor power and cable.");
    return;
  }

  // 2) Check the plan is valid before touching anything.
  for (int i = 0; i < count; i++) {
    if (ids[i] < MIN_ID || ids[i] > MAX_ID) continue;
    int newId = ids[i] + SHIFT;
    if (newId < 1 || newId > 252) {
      Serial.print("ID ");
      Serial.print(ids[i]);
      Serial.print(" would become ");
      Serial.print(newId);
      Serial.println(" (out of range 1-252). Nothing changed.");
      return;
    }
    // A motor OUTSIDE the shifted range already sitting on a target ID would collide.
    for (int j = 0; j < count; j++) {
      bool jShifts = ids[j] >= MIN_ID && ids[j] <= MAX_ID;
      if (!jShifts && ids[j] == newId) {
        Serial.print("Target ID ");
        Serial.print(newId);
        Serial.println(" is used by a motor that isn't being shifted. Nothing changed.");
        return;
      }
    }
  }

  // 3) Change IDs in an order that never creates a duplicate:
  //    ascending for a negative shift, descending for a positive one.
  Serial.println("Plan:");
  for (int k = 0; k < count; k++) {
    int i = (SHIFT < 0) ? k : (count - 1 - k);
    uint8_t oldId = ids[i];
    if (oldId < MIN_ID || oldId > MAX_ID) continue;
    uint8_t newId = oldId + SHIFT;

    Serial.print("  ID ");
    Serial.print(oldId);
    Serial.print(" -> ");
    Serial.print(newId);

    if (DRY_RUN) {
      Serial.println("  (dry run)");
      continue;
    }

    dxl.torqueOff(oldId);   // ID lives in EEPROM, which needs torque off
    if (dxl.setID(oldId, newId) && dxl.ping(newId)) {
      Serial.println("  OK");
    } else {
      Serial.println("  FAILED - stopping here. Rescan before trying again.");
      return;
    }
    delay(50);
  }

  Serial.println(DRY_RUN
    ? "Dry run finished. Set DRY_RUN = false and upload again to apply."
    : "All done. Re-upload firmware/lineform-ai/lineform-ai.ino (check its DXL_IDS list matches).");
}

void loop() {}
