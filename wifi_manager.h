/*
 * wifi_manager.h — STA-only mode (no AP, no web server)
 *
 * NOTE: In the new architecture the ESP32 is a pure WebSocket
 * client. WiFi is managed directly in main.ino using WiFi.begin().
 * This header is kept for reference / future use but is NOT
 * #included by main.ino in the refactored build.
 *
 * If you ever need to swap credentials from one place, edit here
 * and re-include from main.ino.
 */
#pragma once
// (intentionally minimal — credentials now live in main.ino)
