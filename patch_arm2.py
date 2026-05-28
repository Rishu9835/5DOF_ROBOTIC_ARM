import re

with open("arm.ino", "r") as f:
    ino = f.read()

# Commands to add
commands = """  // ── Presets ───────────────────────────────────────────────
  else if (strcmp(cmd, "preset_save") == 0) {
    const char* name = doc["name"] | "Unknown";
    uint8_t angles[NUM_SERVOS];
    JsonArray arr = doc["angles"];
    for(int i=0; i<NUM_SERVOS; i++) {
        angles[i] = arr[i] | HOME_ANGLES[i];
    }
    servoCtrl.savePreset(name, angles);
  }
  else if (strcmp(cmd, "preset_goto") == 0) {
    const char* name = doc["name"] | "";
    servoCtrl.goToPreset(name);
  }
"""

ino = ino.replace('  // ── home ────────────────────────────────────────────────', commands + '\n  // ── home ────────────────────────────────────────────────')

with open("arm.ino", "w") as f:
    f.write(ino)
