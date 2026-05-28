/*
 * servo_controller.h
 * Manages 6 servos — all via PCA9685.
 *
 * Features:
 *   • Angle limits per joint
 *   • Smooth linear interpolation (no sudden jumps)
 *   • Emergency stop (holds current position)
 *   • Gripper passive-hold: PWM silenced after reaching target
 */
#pragma once
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <Wire.h>

// ── Config ───────────────────────────────────────────────────
constexpr uint8_t NUM_SERVOS = 6;
constexpr uint32_t I2C_FREQ = 400000; // 400 kHz fast-mode
constexpr uint16_t PWM_FREQ = 50;     // 50 Hz for all servos

// ── PCA9685 pulse width in "ticks" (4096 steps / 20 ms) ──────
// Calibrate MIN/MAX per channel for your specific servos.
constexpr uint16_t SERVO_MIN[NUM_SERVOS] = {102, 102, 102, 102, 102, 102};
constexpr uint16_t SERVO_MAX[NUM_SERVOS] = {512, 512, 512, 512, 512, 512};

// Angle limits [min, max] per joint (degrees)
constexpr uint8_t ANGLE_MIN[NUM_SERVOS] = {0, 0, 0,
                                           0, 0, 80}; // gripper min = 80
constexpr uint8_t ANGLE_MAX[NUM_SERVOS] = {
    180, 90, 90, 150, 180, 180}; // elbow (ch2) capped at 90°

// Home position
constexpr uint8_t HOME_ANGLES[NUM_SERVOS] = {90, 90, 45, 43, 110, 80};

// Interpolation step size per update call (degrees per tick)
// At 20ms loop -> 1.0 deg/tick = 50 deg/s  (smooth, no overshoot)
constexpr float INTERP_STEP = 1.0f;

// Deadband: stop interpolating when within this many degrees of target.
// Prevents micro-oscillations around the target position.
constexpr float DEADBAND = 2.0f;

// ── Gripper (ch5, SG90) — passive-hold config ─────────────────
// After reaching target the PWM signal is floated so the servo stops
// fighting the load (hunting). Mechanical friction holds the grip.
constexpr uint8_t GRIPPER_CH = 5;
constexpr float GRIPPER_DEADBAND = 3.0f; // degrees

// ── Wrist Pitch (ch3) — parallel mirror servo ─────────────────
// A second SG90 is mounted facing the opposite direction on the same
// shaft to double the torque. It receives the MIRRORED angle so both
// servos push in the same physical direction.
// Wire its signal line to PCA9685 channel WRIST_PITCH_MIRROR_CH.
constexpr uint8_t WRIST_PITCH_CH = 3; // primary channel
constexpr uint8_t WRIST_PITCH_MIRROR_CH =
    6; // mirror channel (free PCA9685 slot)
// Pulse range for the mirror servo — same SG90 range as all others.
constexpr uint16_t MIRROR_PULSE_MIN = 102;
constexpr uint16_t MIRROR_PULSE_MAX = 512;

// ── Servo names (for JSON keys) ───────────────────────────────
const char *const SERVO_NAMES[NUM_SERVOS] = {
    "base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"};

// ── Named preset ─────────────────────────────────────────────
struct Preset {
  char name[24];
  uint8_t angles[NUM_SERVOS];
};

constexpr uint8_t MAX_PRESETS = 8;

// ── ServoController ──────────────────────────────────────────
class ServoController {
public:
  uint8_t currentAngle[NUM_SERVOS];
  float targetAngle[NUM_SERVOS];
  bool eStop = false;
  Preset presets[MAX_PRESETS];
  uint8_t presetCount = 0;

private:
  // Tracks last angle actually sent to hardware — suppresses redundant PWM
  // writes.
  uint8_t lastWrittenAngle[NUM_SERVOS];
  // Gripper passive-hold flag.
  // true  -> target reached, PWM silenced, no writes until next setTarget().
  // false -> actively moving, normal PWM updates.
  bool gripperHeld = false;

public:
  ServoController() {
    for (int i = 0; i < NUM_SERVOS; i++) {
      currentAngle[i] = HOME_ANGLES[i];
      targetAngle[i] = HOME_ANGLES[i];
      lastWrittenAngle[i] = 255; // sentinel — forces write on first begin()
    }
  }

  void begin() {
    Wire.begin();
    Wire.setClock(I2C_FREQ);
    pca_.begin();
    pca_.setPWMFreq(PWM_FREQ);
    delay(10);
    // Drive all servos to home
    for (int i = 0; i < NUM_SERVOS; i++) {
      writeAngle(i, HOME_ANGLES[i]);
    }
    Serial.println("[Servo] PCA9685 initialised — all 6 channels at HOME.");
  }

  // Called from loop() — advances each servo one interpolation step.
  // PWM is only written when the angle actually changes (no redundant writes).
  // Gripper (ch5): PWM is silenced after reaching target to prevent hunting.
  void update() {
    if (eStop)
      return;
    for (int i = 0; i < NUM_SERVOS; i++) {

      // ── Gripper: skip entirely while passively held ──────────
      if (i == GRIPPER_CH && gripperHeld)
        continue;

      float cur = (float)currentAngle[i];
      float tgt = targetAngle[i];

      // ── Deadband selection ───────────────────────────────────
      float db = (i == GRIPPER_CH) ? GRIPPER_DEADBAND : DEADBAND;

      if (abs(cur - tgt) < db) {
        currentAngle[i] = (uint8_t)tgt;

        if (i == GRIPPER_CH) {
          // Send one final accurate pulse then silence the signal
          if (currentAngle[i] != lastWrittenAngle[i]) {
            writeAngle(i, currentAngle[i]);
            lastWrittenAngle[i] = currentAngle[i];
          }
          silenceGripper(); // float the PWM line — servo stops holding
          gripperHeld = true;
          Serial.printf("[Servo] Gripper held @ %d deg — PWM silenced\n",
                        currentAngle[i]);
        } else {
          // Non-gripper: write once on snap, then stop
          if (currentAngle[i] != lastWrittenAngle[i]) {
            writeAngle(i, currentAngle[i]);
            lastWrittenAngle[i] = currentAngle[i];
          }
        }
        continue;
      }

      // ── Interpolation step ────────────────────────────────────
      float step = (tgt > cur) ? INTERP_STEP : -INTERP_STEP;
      float next = cur + step;
      // Clamp overshoot
      if ((step > 0 && next > tgt) || (step < 0 && next < tgt))
        next = tgt;
      uint8_t newAngle = (uint8_t)constrain(next, ANGLE_MIN[i], ANGLE_MAX[i]);

      // ── Only write PWM if angle actually changed ─────────────
      if (newAngle != lastWrittenAngle[i]) {
        currentAngle[i] = newAngle;
        lastWrittenAngle[i] = newAngle;
        writeAngle(i, newAngle);
      }
    }
  }

  // Set a target angle (will be reached smoothly).
  // For the gripper: always clears the passive-hold flag so playback
  // and any new command can move it regardless of angle delta.
  void setTarget(uint8_t ch, float angle) {
    if (ch >= NUM_SERVOS)
      return;
    angle = constrain(angle, ANGLE_MIN[ch], ANGLE_MAX[ch]);
    if (ch == GRIPPER_CH) {
      // Always wake the gripper — recording playback may set the same
      // angle repeatedly, and we must not silently drop those writes.
      gripperHeld = false;
    }
    targetAngle[ch] = angle;
    eStop = false;
  }

  // Instantly move (used during playback for exact timing)
  void setImmediate(uint8_t ch, uint8_t angle) {
    if (ch >= NUM_SERVOS)
      return;
    angle = constrain(angle, ANGLE_MIN[ch], ANGLE_MAX[ch]);
    if (ch == GRIPPER_CH)
      gripperHeld = false; // wake gripper before writing
    currentAngle[ch] = angle;
    targetAngle[ch] = angle;
    writeAngle(ch, angle);
  }

  // Emergency stop — freeze in place
  void emergencyStop() {
    eStop = true;
    for (int i = 0; i < NUM_SERVOS; i++)
      targetAngle[i] = currentAngle[i];
    Serial.println("[Servo] !! EMERGENCY STOP !!");
  }

  // Save preset
  bool savePreset(const char *name, const uint8_t angles[NUM_SERVOS]) {
    for (int i = 0; i < presetCount; i++) {
      if (strcmp(presets[i].name, name) == 0) {
        memcpy(presets[i].angles, angles, NUM_SERVOS);
        return true;
      }
    }
    if (presetCount >= MAX_PRESETS)
      return false;
    strncpy(presets[presetCount].name, name, 23);
    memcpy(presets[presetCount].angles, angles, NUM_SERVOS);
    presetCount++;
    return true;
  }

  // Go to preset by name
  bool goToPreset(const char *name) {
    for (int i = 0; i < presetCount; i++) {
      if (strcmp(presets[i].name, name) == 0) {
        for (int j = 0; j < NUM_SERVOS; j++)
          setTarget(j, presets[i].angles[j]);
        return true;
      }
    }
    return false;
  }

  // Serialise full state to JSON
  void toJson(JsonObject &obj) const {
    JsonObject angles = obj.createNestedObject("angles");
    for (int i = 0; i < NUM_SERVOS; i++)
      angles[SERVO_NAMES[i]] = currentAngle[i];
    obj["eStop"] = eStop;
    JsonArray pArr = obj.createNestedArray("presets");
    for (int i = 0; i < presetCount; i++) {
      JsonObject p = pArr.createNestedObject();
      p["name"] = presets[i].name;
      JsonArray a = p.createNestedArray("angles");
      for (int j = 0; j < NUM_SERVOS; j++)
        a.add(presets[i].angles[j]);
    }
  }

private:
  Adafruit_PWMServoDriver pca_{0x40}; // default I2C address

  // ── silenceGripper: float PCA9685 ch5 output ─────────────────
  // Sets both on and off ticks to 0 -> PCA9685 drives the pin LOW
  // -> servo sees no valid pulse -> internal driver de-energises.
  // The grip is maintained by mechanical friction, not holding torque.
  void silenceGripper() {
    pca_.setPWM(GRIPPER_CH, 0, 0); // signal line goes LOW
    lastWrittenAngle[GRIPPER_CH] =
        255; // invalidate cache -> next move forces real write
  }

  // ── writeAngle: all channels -> PCA9685 ────────────────────────
  // For ch3 (wrist pitch) a mirror servo on WRIST_PITCH_MIRROR_CH is
  // driven simultaneously with the inverted angle (180 - deg) because
  // the two servos face each other on the same shaft.
  void writeAngle(uint8_t ch, uint8_t deg) {
    uint16_t pulse = map(deg, 0, 180, SERVO_MIN[ch], SERVO_MAX[ch]);
    pca_.setPWM(ch, 0, pulse);

    if (ch == WRIST_PITCH_CH) {
      // Mirror servo: physically reversed -> invert the angle
      uint8_t mirrorDeg = 180 - deg;
      uint16_t mirrorPulse =
          map(mirrorDeg, 0, 180, MIRROR_PULSE_MIN, MIRROR_PULSE_MAX);
      pca_.setPWM(WRIST_PITCH_MIRROR_CH, 0, mirrorPulse);
    }
  }
};
