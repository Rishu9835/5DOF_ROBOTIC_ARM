import re

with open("arm.ino", "r") as f:
    ino = f.read()

# Commands to add
commands = """  // ── Recording / Playback ────────────────────────────────
  else if (strcmp(cmd, "record_start") == 0) {
    recorder.startRecording();
  }
  else if (strcmp(cmd, "record_stop") == 0) {
    recorder.stopRecording();
  }
  else if (strcmp(cmd, "playback_start") == 0) {
    recorder.speedMult = doc["speed"] | 1.0f;
    recorder.loopPlayback = doc["loop"] | false;
    recorder.startPlayback();
  }
  else if (strcmp(cmd, "playback_stop") == 0) {
    recorder.stopPlayback();
  }
"""

ino = ino.replace('  // ── home ────────────────────────────────────────────────', commands + '\n  // ── home ────────────────────────────────────────────────')

with open("arm.ino", "w") as f:
    f.write(ino)
