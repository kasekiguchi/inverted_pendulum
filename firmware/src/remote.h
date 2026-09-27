// Smartphone remote control: the FIRE runs a Wi-Fi access point and serves a
// joystick page; the page talks to the robot over a WebSocket.
#pragma once
#include <Arduino.h>

namespace remote {

// Joystick command, both in [-1, 1]. v: + forward, w: + turn right.
struct Cmd {
  float v = 0, w = 0;
};

// Starts the access point and the web server (http://192.168.4.1).
void begin(const char* ssid, const char* password);

// Latest joystick command; zero if nothing arrived for `timeout_ms`.
Cmd get(uint32_t now_ms, uint32_t timeout_ms = 500);

// One-shot button requests from the page.
bool takeArm();
bool takeStop();

// Sends a status line to connected pages (call at a few Hz) and cleans up clients.
void publish(const char* state, float th_deg, float vin, float v_ref, const char* warning);

}  // namespace remote
