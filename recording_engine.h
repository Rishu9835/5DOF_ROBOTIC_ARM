/*
 * recording_engine.h
 * Records servo angle sequences with timestamps.
 * Playback with smooth inter-frame interpolation.
 * Adjustable speed multiplier (0.25× – 4×).
 */
#pragma once
#include <Arduino.h>
#include "servo_controller.h"   // for NUM_SERVOS

// ── Config ───────────────────────────────────────────────────
constexpr uint16_t MAX_FRAMES      = 500;   // ~500 keyframes in RAM
constexpr uint16_t SAMPLE_INTERVAL = 100;   // record a frame every 100ms

// ── Frame ────────────────────────────────────────────────────
struct Frame {
  uint32_t ts;                    // timestamp (ms since recording start)
  uint8_t  angles[NUM_SERVOS];
};

// ── States ───────────────────────────────────────────────────
enum class RecState { IDLE, RECORDING, PLAYING };

// ── RecordingEngine ──────────────────────────────────────────
class RecordingEngine {
public:
  RecState state        = RecState::IDLE;
  float    speedMult    = 1.0f;    // playback speed multiplier
  uint16_t frameCount   = 0;
  uint16_t playIdx      = 0;
  bool     loopPlayback = false;

  // ── Recording ────────────────────────────────────────────
  bool startRecording() {
    if (state == RecState::PLAYING) return false;
    frameCount  = 0;
    recStart_   = millis();
    lastSample_ = recStart_ - SAMPLE_INTERVAL; // force immediate first capture
    state       = RecState::RECORDING;
    Serial.println("[Rec] Recording started.");
    return true;
  }

  void stopRecording() {
    if (state != RecState::RECORDING) return;
    state = RecState::IDLE;
    Serial.printf("[Rec] Recording stopped. Frames: %d\n", frameCount);
  }

  // Call from loop() while recording to capture snapshots
  void captureIfNeeded(const ServoController& sc) {
    if (state != RecState::RECORDING) return;
    uint32_t now     = millis();
    uint32_t elapsed = now - recStart_;
    // Guard: require at least SAMPLE_INTERVAL ms since last capture
    if ((now - lastSample_) < SAMPLE_INTERVAL) return;
    if (frameCount >= MAX_FRAMES) { stopRecording(); return; }
    frames_[frameCount].ts = elapsed;
    memcpy(frames_[frameCount].angles, sc.currentAngle, NUM_SERVOS);
    frameCount++;
    lastSample_ = now;
  }

  // ── Playback ─────────────────────────────────────────────
  bool startPlayback() {
    if (frameCount < 2) return false;
    playIdx    = 0;
    pbStart_   = millis();
    pbDone_    = false;
    state      = RecState::PLAYING;
    Serial.printf("[Rec] Playback started. Frames: %d  Speed: %.2f×\n",
                  frameCount, speedMult);
    return true;
  }

  void stopPlayback() {
    state = RecState::IDLE;
    Serial.println("[Rec] Playback stopped.");
  }

  // Call from loop() — drives servo targets during playback
  void update(ServoController& sc) {
    // Always capture when recording
    captureIfNeeded(sc);

    if (state != RecState::PLAYING) return;
    if (pbDone_) {
      if (loopPlayback) startPlayback();
      else              stopPlayback();
      return;
    }

    // Scaled playback time (float to preserve precision at high speeds)
    float pbElapsed = (float)(millis() - pbStart_) * speedMult;

    // Find current segment
    while (playIdx < frameCount - 1 &&
           (float)frames_[playIdx + 1].ts <= pbElapsed) {
      playIdx++;
    }

    if (playIdx >= frameCount - 1) {
      // Last frame reached → apply exactly then finish
      for (int j = 0; j < NUM_SERVOS; j++)
        sc.setTarget(j, frames_[frameCount - 1].angles[j]);
      pbDone_ = true;
      return;
    }

    // Linear interpolation between playIdx and playIdx+1
    const Frame& f0 = frames_[playIdx];
    const Frame& f1 = frames_[playIdx + 1];
    float seg = (float)(f1.ts - f0.ts);
    float t   = (seg == 0.0f) ? 1.0f :
                (pbElapsed - (float)f0.ts) / seg;
    t = constrain(t, 0.0f, 1.0f);

    for (int j = 0; j < NUM_SERVOS; j++) {
      float angle = f0.angles[j] + t * (f1.angles[j] - f0.angles[j]);
      sc.setTarget(j, angle);
    }
  }

  const char* stateString() const {
    switch (state) {
      case RecState::RECORDING: return "recording";
      case RecState::PLAYING:   return "playing";
      default:                  return "idle";
    }
  }

private:
  Frame    frames_[MAX_FRAMES];
  uint32_t recStart_   = 0;
  uint32_t lastSample_ = 0;
  uint32_t pbStart_    = 0;
  bool     pbDone_     = false;
};
