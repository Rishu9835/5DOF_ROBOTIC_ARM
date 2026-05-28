/*
 * ============================================================
 *  5-DOF Robotic Arm — ESP32 Firmware (WebSocket CLIENT)
 *  Hardware : ESP32 + PCA9685 (all 6 channels)
 *  Framework: Arduino (ESP32 Arduino Core)
 *
 *  Architecture: ESP32 is a pure WS client.
 *    Browser → Python Backend → ESP32
 *
 *  No AsyncWebServer. No heap-heavy WS server. Stable.
 * ============================================================
 */

#include "ik_solver.h"
#include "recording_engine.h"
#include "servo_controller.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h> // arduinoWebSockets by Markus Sattler
#include <WiFi.h>

// ── WiFi credentials (STA mode only) ─────────────────────────
constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";

// ── Backend WebSocket server ──────────────────────────────────
constexpr char WS_HOST[] = ""; // ← set to your server IP
constexpr uint16_t WS_PORT = 8765;
constexpr char WS_PATH[] = "/ws/esp32";
// ── Reconnect interval ────────────────────────────────────────
constexpr uint32_t RECONNECT_MS = 5000;

// ── Globals ───────────────────────────────────────────────────
ServoController servoCtrl;
RecordingEngine recorder;
WebSocketsClient wsClient;

// ── Forward declarations ──────────────────────────────────────
void onWsEvent(WStype_t type, uint8_t *payload, size_t length);
void handleCommand(const uint8_t *payload, size_t length);
void connectWiFi();

// ─────────────────────────────────────────────────────────────
//  setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] Robotic Arm — WS-client mode");
  Serial.printf("[BOOT] Free heap: %u bytes\n", ESP.getFreeHeap());

  servoCtrl.begin();
  connectWiFi();

  // Configure WebSocket client
  wsClient.begin(WS_HOST, WS_PORT, WS_PATH);
  wsClient.onEvent(onWsEvent);
  wsClient.setReconnectInterval(RECONNECT_MS);
  // Optional: enable heartbeat (ping every 15s, timeout 3s, disconnect after 2
  // misses)
  wsClient.enableHeartbeat(15000, 3000, 2);

  Serial.printf("[BOOT] Connecting to WS ws://%s:%u%s\n", WS_HOST, WS_PORT,
                WS_PATH);
  Serial.printf("[BOOT] Free heap after init: %u bytes\n", ESP.getFreeHeap());
}

// ─────────────────────────────────────────────────────────────
//  loop
// ─────────────────────────────────────────────────────────────
void loop() {
  // Drive the WS client state machine — non-blocking
  wsClient.loop();

  const uint32_t now = millis();

  // ── 20ms tick: servo interpolation ───────────────────────
  // 20ms matches the 50Hz PWM period — no benefit sending faster.
  static uint32_t lastTick = 0;
  if (now - lastTick >= 20) {
    lastTick = now;
    servoCtrl.update();
    recorder.update(servoCtrl);
  }

  // ── 5s heap monitor ──────────────────────────────────────
  static uint32_t lastHeap = 0;
  if (now - lastHeap >= 5000) {
    lastHeap = now;
    Serial.printf("[MEM] Free heap: %u  Min: %u\n", ESP.getFreeHeap(),
                  ESP.getMinFreeHeap());
  }
}

// ─────────────────────────────────────────────────────────────
//  WiFi helpers
// ─────────────────────────────────────────────────────────────
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries++ < 40) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected. IP: %s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] FAILED — will retry in loop.");
  }
}

// ─────────────────────────────────────────────────────────────
//  WebSocket event handler
// ─────────────────────────────────────────────────────────────
void onWsEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_CONNECTED:
    Serial.printf("[WS] Connected to %s\n", (char *)payload);
    // Announce ourselves
    wsClient.sendTXT("{\"role\":\"esp32\"}");
    break;

  case WStype_DISCONNECTED:
    Serial.println("[WS] Disconnected — will auto-reconnect.");
    break;

  case WStype_TEXT:
    handleCommand(payload, length);
    break;

  case WStype_ERROR:
    Serial.printf("[WS] Error: %s\n", payload ? (char *)payload : "unknown");
    break;

  default:
    break;
  }
}

// ─────────────────────────────────────────────────────────────
//  Command dispatcher
//  Uses StaticJsonDocument — fixed 256-byte stack allocation,
//  no heap fragmentation.
// ─────────────────────────────────────────────────────────────
void handleCommand(const uint8_t *payload, size_t length) {
  // Guard against oversized packets before parsing
  if (length > 255) {
    Serial.println("[CMD] Packet too large, ignored.");
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("[CMD] JSON error: %s\n", err.c_str());
    return;
  }

  const char *cmd = doc["cmd"] | "";

  // ── set_servo ───────────────────────────────────────────
  if (strcmp(cmd, "set_servo") == 0) {
    int ch = doc["ch"] | -1;
    float angle = doc["angle"] | -1.0f;
    if (ch < 0 || ch >= NUM_SERVOS || angle < 0) {
      Serial.println("[CMD] set_servo: invalid params");
      return;
    }
    servoCtrl.setTarget((uint8_t)ch, angle);
    Serial.printf("[CMD] set_servo ch=%d angle=%.1f\n", ch, angle);
  }

  // ── estop ───────────────────────────────────────────────
  else if (strcmp(cmd, "estop") == 0) {
    servoCtrl.emergencyStop();
    recorder.stopPlayback();
  }

  // ── Recording / Playback ────────────────────────────────
  else if (strcmp(cmd, "record_start") == 0) {
    recorder.startRecording();
  } else if (strcmp(cmd, "record_stop") == 0) {
    recorder.stopRecording();
  } else if (strcmp(cmd, "playback_start") == 0) {
    recorder.speedMult = doc["speed"] | 1.0f;
    recorder.loopPlayback = doc["loop"] | false;
    recorder.startPlayback();
  } else if (strcmp(cmd, "playback_stop") == 0) {
    recorder.stopPlayback();
  }

  // ── Presets ───────────────────────────────────────────────
  else if (strcmp(cmd, "preset_save") == 0) {
    const char *name = doc["name"] | "Unknown";
    uint8_t angles[NUM_SERVOS];
    JsonArray arr = doc["angles"];
    for (int i = 0; i < NUM_SERVOS; i++) {
      angles[i] = arr[i] | HOME_ANGLES[i];
    }
    servoCtrl.savePreset(name, angles);
  } else if (strcmp(cmd, "preset_goto") == 0) {
    const char *name = doc["name"] | "";
    servoCtrl.goToPreset(name);
  }

  // ── home ────────────────────────────────────────────────
  else if (strcmp(cmd, "home") == 0) {
    for (int i = 0; i < NUM_SERVOS; i++)
      servoCtrl.setTarget(i, HOME_ANGLES[i]);
    Serial.println("[CMD] home");
  }

  // ── ik_move (optional IK integration) ───────────────────
  else if (strcmp(cmd, "ik_move") == 0) {
    float x = doc["x"] | 0.0f;
    float y = doc["y"] | 0.0f;
    float z = doc["z"] | 150.0f;
    float pitch = doc["pitch"] | 0.0f;
    // Accept both the current field names and the older dashboard names.
    float grip =
        doc["grip"].isNull() ? (doc["gripper"] | 90.0f) : (doc["grip"] | 90.0f);
    float roll = doc["roll"].isNull() ? (doc["wrist_roll"] | 90.0f)
                                      : (doc["roll"] | 90.0f);

    IKResult res = IKSolver::solve(x, y, z, pitch, grip, roll);
    if (res.reachable) {
      for (int i = 0; i < NUM_SERVOS; i++)
        servoCtrl.setTarget(i, res.theta[i]);
      Serial.printf("[CMD] ik_move: %s\n", res.msg);
    } else {
      Serial.printf("[CMD] ik_move UNREACHABLE: %s\n", res.msg);
    }
  }

  else {
    Serial.printf("[CMD] Unknown command: %s\n", cmd);
  }
}
