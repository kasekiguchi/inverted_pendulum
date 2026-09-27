// Smartphone remote control: the FIRE runs a Wi-Fi access point and serves a
// page (drive / tune / log) at http://192.168.4.1; the page talks to the robot
// over a WebSocket. Everything received is queued and applied by the control
// loop (take*), never from the network task.
#pragma once
#include <Arduino.h>

namespace remote {

// Joystick command, both in [-1, 1]. v: + forward, w: + turn right.
struct Cmd {
  float v = 0, w = 0;
};

// Parameters the tuning page can change; the order matches the page.
enum Param : uint8_t { kK0, kK1, kK2, kK3, kTrim, kVmax, kAmax, kJmax, kLean, kYaw, kParamCount };

enum class Action : uint8_t { kNone, kSet, kSave, kRevert, kParamsRequest, kRecStart, kRecStop, kMark };

struct Msg {
  Action action = Action::kNone;
  uint8_t param = 0;  // for kSet
  float value = 0;    // for kSet
};

// Starts the access point and the web server (http://192.168.4.1).
void begin(const char* ssid, const char* password);

// Latest joystick command; zero if nothing arrived for `timeout_ms`.
Cmd get(uint32_t now_ms, uint32_t timeout_ms = 500);

// One-shot button requests from the page.
bool takeArm();
bool takeStop();

// Next queued page request (parameter change, save, recording, ...); false if none.
bool take(Msg& m);

// Status line for the pages (call at ~20 Hz); also cleans up clients.
struct Status {
  const char* state;
  float th_deg, vin, v_ref, u;
  const char* warning;
  float rec_s;     // seconds in the on-board recorder
  bool rec_all;    // continuous recording requested from the page
};
void publish(const Status& s);

// Sends a raw text line to all pages ("p,..." parameters, "m,..." messages).
void sendText(const char* msg);

// Log download (/log.csv): the recorder provides the rows. `freeze` is called
// with true when a download starts and false when it ends, so rows stay put.
struct LogSource {
  size_t (*count)();
  size_t (*line)(size_t i, char* buf, size_t len);  // CSV row i, returns length
  void (*freeze)(bool on);
  const char* header;
};
void setLogSource(const LogSource& src);

}  // namespace remote
